#!/usr/bin/env python3
"""Stage 3C.51: simulated, supervised HOME -> pre-grasp joint motion.

ONLY Isaac Lab simulation. No hardware, no external robot interface, no finger
motion, no cube grasp/descent, no robot asset edits. Uses the same scene,
contact reporting, and gravity-biased position controller as Stage 3C.50 and
historically tested Stage 3C.37. Requires local Stage49 proxy and Stage50
positive-control JSON; checks them against the selected goal before motion.

CRITICAL: self collisions remain disabled in the Isaac asset; preexisting HOME
self-mesh overlaps are NOT waived or validated by this test. Contact sensors
screen EXTERNAL contact only. No real-world motion authorization is granted.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf',type=Path,required=True)
parser.add_argument('--stage49',type=Path,required=True)
parser.add_argument('--stage50',type=Path,required=True)
parser.add_argument('--out',type=Path,default=Path('/tmp/lexium_stage3c51_physx_pregrasp_motion_trial.json'))
parser.add_argument('--warmup-steps',type=int,default=150)
parser.add_argument('--motion-steps',type=int,default=1200)
parser.add_argument('--hold-steps',type=int,default=180)
parser.add_argument('--contact-threshold-n',type=float,default=.5)
parser.add_argument('--num-envs',type=int,default=1)
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if args.num_envs!=1 or not(120<=args.warmup_steps<=360) or not(900<=args.motion_steps<=1800) or not(120<=args.hold_steps<=360):
    parser.error('Requires one environment; warmup 120..360, motion 900..1800, hold 120..360 steps')
if not .1<=args.contact_threshold_n<=1.0:
    parser.error('External robot contact threshold must be 0.1..1.0 N')
launcher=AppLauncher(args)
simulation_app=launcher.app

import numpy as np
import torch
import omni.usd
import isaaclab.sim as sim_utils
from pxr import UsdPhysics, PhysxSchema
from isaaclab.assets import ArticulationCfg,AssetBaseCfg,RigidObjectCfg
from isaaclab.scene import InteractiveScene,InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
from lexium_pick_place.robot_cfg import LEXIUM_COBOT_CFG,LEXIUM_ARM_JOINT_NAMES,LEXIUM_TCP_OFFSET,LEXIUM_USD_PATH
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import parse_urdf,Q_HOME,rpy_to_R,axis_rotation
from lexium_pick_place.lexium_stage3c25_corrected_gravity_route_audit import compute_root_frame_gravity

ROBOT_CFG=LEXIUM_COBOT_CFG.replace(
    prim_path='{ENV_REGEX_NS}/Robot',
    spawn=LEXIUM_COBOT_CFG.spawn.replace(activate_contact_sensors=True),
)

def table_cfg():
    return AssetBaseCfg(
        prim_path='{ENV_REGEX_NS}/Table',
        spawn=sim_utils.CuboidCfg(
            size=(.90,1.20,.40),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=PhysxRigidBodyMaterialCfg(static_friction=1.,dynamic_friction=.8,restitution=0.),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.35,.35,.42))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(.62,0.,-.20)))

def cube_cfg():
    return RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/Cube',
        spawn=sim_utils.CuboidCfg(
            size=(.04,.04,.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,solver_velocity_iteration_count=2,disable_gravity=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            activate_contact_sensors=True,
            physics_material=PhysxRigidBodyMaterialCfg(static_friction=1.,dynamic_friction=.8,restitution=0.),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.15,.75,.25))),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(.30,-.30,.12)))

@configclass
class Stage51Scene(InteractiveSceneCfg):
    table: AssetBaseCfg=table_cfg()
    robot: ArticulationCfg=ROBOT_CFG
    cube: RigidObjectCfg=cube_cfg()
    robot_contact: ContactSensorCfg=ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*',update_period=0.)
    cube_contact: ContactSensorCfg=ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Cube',update_period=0.)
    light: AssetBaseCfg=AssetBaseCfg(
        prim_path='/World/Light',spawn=sim_utils.DomeLightCfg(intensity=2500.))

def abort(code):
    raise RuntimeError('STAGE51_SAFETY_ABORT='+str(code))

def quat_R(q):
    x,y,z,w=np.asarray(q,dtype=float)
    n=np.linalg.norm([x,y,z,w])
    if n<1e-8:abort('INVALID_QUATERNION')
    x,y,z,w=np.asarray([x,y,z,w])/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]],dtype=float)

def fk_relative(joints,children,root_name,q):
    qmap=dict(zip(LEXIUM_ARM_JOINT_NAMES,q))
    poses={root_name:(np.eye(3),np.zeros(3))}
    pending=[root_name]
    while pending:
        parent=pending.pop()
        Rp,pp=poses[parent]
        for jn in children.get(parent,()):
            j=joints[jn]
            Rj=Rp@rpy_to_R(j['rpy'])
            pj=pp+Rp@j['xyz']
            qi=float(qmap.get(jn,0.))
            if j['type'] in ('revolute','continuous'):
                Rchild,pchild=Rj@axis_rotation(j['axis'],qi),pj
            elif j['type']=='prismatic':
                Rchild,pchild=Rj,pj+Rj@j['axis']*qi
            else:
                Rchild,pchild=Rj,pj
            poses[j['child']]=(Rchild,pchild)
            pending.append(j['child'])
    if 'gripper_base_1' not in poses:abort('URDF_GRIPPER_BASE_NOT_FOUND')
    return poses['gripper_base_1']

def rotation_diff_deg(A,B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T@B)-1.)*.5,-1.,1.))))

def save_report(report):
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print('STAGE51_REPORT_JSON='+str(args.out),flush=True)

def load_prereqs():
    for p in (args.urdf,args.stage49,args.stage50):
        if not p.is_file():abort('MISSING_INPUT_'+str(p))
    if not Path(LEXIUM_USD_PATH).is_file():abort('USD_FILE_MISSING')
    a=json.loads(args.stage49.read_text(encoding='utf8'))
    b=json.loads(args.stage50.read_text(encoding='utf8'))
    if a.get('stage')!='3C49' or b.get('stage')!='3C50':abort('PREREQ_STAGE_MISMATCH')
    if a.get('uses_physx_cooked_collision_geometry') is not False or a.get('uses_authoring_mesh_convex_hulls') is not True:
        abort('PROXY_REPORT_PROVENANCE_INVALID')
    for key in ('frames_with_new_hull_self_pairs_vs_home','frames_with_hull_table_overlap','frames_with_hull_cube_overlap','inconclusive_lp_queries'):
        if a.get(key)!=0:abort('PROXY_SCREEN_FAIL_'+key)
    if a.get('hull_endpoint_overlap_pairs')!=[]:abort('NONEMPTY_ENDPOINT_HULL_OVERLAPS')
    if a.get('hull_home_overlap_pairs') is None or len(a['hull_home_overlap_pairs'])!=5:
        abort('PROXY_HOME_GEOMETRY_CHANGED')
    if not(b.get('home_contact_positive_control_passed') is True and b.get('home_only') is True):
        abort('HOME_PHYSX_SENSOR_PREFLIGHT_NOT_PASSED')
    if b.get('pregrasp_motion_executed') is not False or b.get('physx_self_collisions_enabled') is not False:
        abort('PREREQ_POLICY_CHANGED')
    if not(.2<=b.get('cube_table_contact_positive_control_n',0)<=10.):
        abort('CUBE_TABLE_SENSOR_CONTROL_NOT_VALIDATED')
    if not(b.get('max_robot_external_contact_n',99.)<.1 and b.get('max_home_joint_tracking_error_rad',99)<.05):
        abort('HOME_BASELINE_NOT_STABLE')
    qhome=np.asarray(Q_HOME,dtype=float)
    target=np.asarray(a['target_q_rad'],dtype=float)
    if qhome.shape!=(6,) or target.shape!=(6,) or not np.isfinite(np.r_[qhome,target]).all():
        abort('BAD_TARGET_OR_HOME_DIMENSIONS')
    if not np.allclose(a['home_q_rad'],qhome,atol=1e-4):abort('HOME_MISMATCH')
    if not np.allclose(b['stage49_path_target_q_rad'],target,atol=1e-6):abort('TARGET_MISMATCH_BETWEEN_STAGES')
    if not np.allclose(target,[.5116462574036054,1.6679322364615043,2.3215496095643826,
                               1.4630436610150346,1.4414003069368027,.26596198934162374],atol=1e-5):
        abort('TARGET_NOT_APPROVED_FOR_SIMULATION_TRIAL')
    return a,b,qhome,target

def main():
    s49,s50,qhome,qgoal=load_prereqs()
    links,joints,children,root_name=parse_urdf(args.urdf)
    print('STAGE=3C51 MODE=ISAAC_LAB_SIMULATION_SUPERVISED_PREGRASP_TRIAL',flush=True)
    print('STAGE51_SIMULATION_ONLY=True HARDWARE_CONNECTED=False',flush=True)
    print('STAGE51_FINGER_COMMANDS=HOME_FIXED CUBE_GRASP_COMMANDED=False',flush=True)
    print('STAGE51_USD_ASSETS_MODIFIED=False SELF_COLLISION_POLICY_CHANGED=False',flush=True)
    print('STAGE51_SELF_COLLISIONS_NOT_VALIDATED=True',flush=True)
    print('STAGE51_PRIOR_SELF_HULL_OVERLAP_PAIRS_IN_HOME='+str(len(s49['hull_home_overlap_pairs'])),flush=True)
    print('STAGE51_PREREQ_HOME_CUBE_CONTACT_N={:.6f}'.format(s50['cube_table_contact_positive_control_n']),flush=True)
    print('STAGE51_TARGET_Q_RAD='+json.dumps(qgoal.tolist()),flush=True)
    sim=SimulationContext(sim_utils.SimulationCfg(device=args.device,dt=1./60.,gravity=(0.,0.,-9.81)))
    scene=InteractiveScene(Stage51Scene(num_envs=1,env_spacing=2.5))
    sim.reset();scene.reset();scene.update(sim.get_physics_dt())
    robot=scene['robot'];cube=scene['cube'];rs=scene['robot_contact'];cs=scene['cube_contact']
    if not robot.is_fixed_base:abort('ROBOT_NOT_FIXED_BASE')
    jnames=list(robot.data.joint_names);bnames=list(robot.data.body_names)
    try:
        ids=[jnames.index(n) for n in LEXIUM_ARM_JOINT_NAMES]
        rootidx=bnames.index(root_name)
        toolidx=bnames.index('gripper_base_1')
    except ValueError as e:abort('JOINT_OR_BODY_NOT_FOUND_'+str(e))
    stage=omni.usd.get_context().get_stage()
    for name in ('/World/envs/env_0/Robot','/World/envs/env_0/Table','/World/envs/env_0/Cube'):
        if not stage.GetPrimAtPath(name).IsValid():abort('SCENE_PRIM_MISSING_'+name)
    sc=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            attr=prim.GetAttribute('physxArticulation:enabledSelfCollisions')
            sc.append(attr.Get() if attr.IsValid() else None)
    if sc!=[False]:abort('SELF_COLLISIONS_POLICY_CHANGED_'+repr(sc))
    reporters=sum(str(p.GetPath()).startswith('/World/envs/env_0/Robot/')
                  for p in stage.Traverse() if p.HasAPI(PhysxSchema.PhysxContactReportAPI))
    if reporters<10:abort('MISSING_ROBOT_CONTACT_REPORTERS')
    if tuple(rs.data.net_normal_forces_w.torch.shape)[0]!=1 or rs.data.net_normal_forces_w.torch.shape[1]<10:
        abort('BAD_ROBOT_SENSOR_SHAPE')
    if tuple(cs.data.net_normal_forces_w.torch.shape)!=(1,1,3):abort('BAD_CUBE_SENSOR_SHAPE')
    k=robot.data.joint_stiffness.torch[0,ids].detach().cpu().numpy().astype(float)
    limit_eff=robot.data.joint_effort_limits.torch[0,ids].detach().cpu().numpy().astype(float)
    limits=robot.data.joint_pos_limits.torch[0,ids].detach().cpu().numpy().astype(float)
    if not np.allclose(k,[400,800,800,400,400,400],atol=.1):abort('UNEXPECTED_STIFFNESS')
    if not np.allclose(limit_eff,[100.]*6,atol=.1):abort('UNEXPECTED_ACTUATOR_LIMITS')
    if np.any(qgoal<=limits[:,0]+.08) or np.any(qgoal>=limits[:,1]-.08):
        abort('GOAL_TOO_CLOSE_TO_JOINT_LIMITS')
    full_home=robot.data.default_joint_pos.torch.clone()
    if not np.allclose(full_home[0,ids].detach().cpu().numpy(),qhome,atol=1e-4):
        abort('SCENE_DEFAULT_HOME_CHANGED')
    robot.write_joint_state_to_sim_index(position=full_home,velocity=torch.zeros_like(full_home))
    scene.update(0.0)
    cube.write_root_pose_to_sim_index(root_pose=cube.data.default_root_pose.torch.clone())
    cube.write_root_velocity_to_sim_index(root_velocity=cube.data.default_root_vel.torch.clone())
    rootpose=robot.data.body_link_pose_w.torch[0,rootidx].detach().cpu().numpy().astype(float)
    Rroot=quat_R(rootpose[3:7]);groot=Rroot.T@np.array([0.,0.,-9.81])
    if not np.allclose(groot,[0.,-9.81,0.],atol=.01):abort('ROOT_GRAVITY_FRAME_CHANGED')
    Rlocal,plocal=fk_relative(joints,children,root_name,qgoal)
    expected_tool_xyz=rootpose[:3]+Rroot@plocal
    expected_tool_R=Rroot@Rlocal
    tcp_offset=np.asarray(LEXIUM_TCP_OFFSET,dtype=float)
    expected_tcp_xyz=expected_tool_xyz+expected_tool_R@tcp_offset
    if not np.allclose(tcp_offset,[.002,.1,.148],atol=1e-6):abort('TCP_OFFSET_CHANGED')
    if not (.15<expected_tool_xyz[0]<.40 and -.45<expected_tool_xyz[1]<-.15 and .15<expected_tool_xyz[2]<.50):
        abort('GOAL_WORLD_TOOL_NOT_IN_EXPECTED_REGION_'+repr(expected_tool_xyz.tolist()))
    peakgrav=np.zeros(6,dtype=float)
    for s in np.linspace(0.,1.,301):
        q=qhome+s*(qgoal-qhome)
        tq=np.asarray(compute_root_frame_gravity(links,joints,children,root_name,q,groot),dtype=float)
        if not np.isfinite(tq).all():abort('NONFINITE_GRAVITY_PREFLIGHT')
        peakgrav=np.maximum(peakgrav,np.abs(tq))
        cmd=q+tq/k
        if np.any(cmd<limits[:,0]+1e-4) or np.any(cmd>limits[:,1]-1e-4):
            abort('GRAVITY_BIASED_COMMAND_OUT_OF_LIMITS')
    print('STAGE51_OFFLINE_GRAVITY_PEAK_NM='+json.dumps(np.round(peakgrav,4).tolist()),flush=True)
    if np.max(peakgrav)>=85.:abort('GRAVITY_PREFLIGHT_GT_85NM')
    print('STAGE51_EXPECTED_GRIPPER_BASE_WORLD_XYZ_M='+json.dumps(np.round(expected_tool_xyz,7).tolist()),flush=True)
    print('STAGE51_EXPECTED_TCP_WORLD_XYZ_M='+json.dumps(np.round(expected_tcp_xyz,7).tolist()),flush=True)

    qcmd=full_home.clone()
    dt=sim.get_physics_dt()
    total=args.warmup_steps+args.motion_steps+args.hold_steps
    sample_steps={1,30,60,90,args.warmup_steps,
                  args.warmup_steps+1,
                  args.warmup_steps+args.motion_steps//4,
                  args.warmup_steps+args.motion_steps//2,
                  args.warmup_steps+3*args.motion_steps//4,
                  args.warmup_steps+args.motion_steps,
                  total-120,total-30,total}
    samples=[];last_hold=[]
    peakforce=0.;max_tracking=0.;maxvel=0.;max_effort=0.;cube_force_end=0.
    exceed_eff=0;exceed_track=0;motion_started=False
    root_anchor=robot.data.root_link_pose_w.torch[0,:3].detach().clone()
    report={'stage':'3C51','mode':'physx_simulation_supervised_pregrasp_joint_motion',
      'sim_only':True,'hardware_connected':False,'assets_modified':False,
      'physx_self_collisions_enabled':False,'self_collision_geometry_validated':False,
      'home_self_proxy_hull_overlaps_unresolved':s49['hull_home_overlap_pairs'],
      'safety_approval_for_hardware':False,'grasp_approved':False,
      'fingers_commanded':False,'grasp_attempted':False,
      'home_q_rad':qhome.tolist(),'target_q_rad':qgoal.tolist(),
      'warmup_steps':args.warmup_steps,'motion_steps':args.motion_steps,'hold_steps':args.hold_steps,
      'expected_tool_world_xyz_m':expected_tool_xyz.tolist(),
      'expected_tcp_world_xyz_m':expected_tcp_xyz.tolist(),
      'preflight_gravity_peaks_nm':peakgrav.tolist(),'max_robot_external_contact_threshold_n':args.contact_threshold_n,
      'simulation_motion_passed':False,'trajectory_fully_executed':False,
      'sampled':samples,
      'disclaimer':'A SIMULATION-ONLY motion experiment. Existing HOME self-hull overlaps remain; enabledSelfCollisions=false. External contact sensor cannot validate self clearance, and offline 41-sample proxies are not swept/continuous collision checks. No finger closure or grasp, and no hardware motion authorization.'}
    print('STAGE51_SIM_MOTION_PLANNED_STEPS={} WARMUP={} HOLD={} CONTACT_ABORT_N={}'.format(args.motion_steps,args.warmup_steps,args.hold_steps,args.contact_threshold_n),flush=True)
    try:
        for step in range(1,total+1):
            if not simulation_app.is_running():abort('SIM_APP_STOPPED_STEP_'+str(step))
            if step<=args.warmup_steps:
                phase='WARMUP';v=0.;a=0.
            elif step<=args.warmup_steps+args.motion_steps:
                phase='MOVE';v=(step-args.warmup_steps)/args.motion_steps
                a=10*v**3-15*v**4+6*v**5
                if not motion_started:
                    print('STAGE51_SIM_PREGRASP_MOTION_STARTED=True',flush=True)
                    motion_started=True
            else:
                phase='HOLD';v=1.;a=1.
            qref=qhome+a*(qgoal-qhome)
            tg=np.asarray(compute_root_frame_gravity(links,joints,children,root_name,qref,groot),float)
            bias=tg/k
            cmd=qref+bias
            if not np.isfinite(cmd).all() or np.max(np.abs(bias))>.25:abort('INVALID_COMMAND_STEP_'+str(step))
            if np.any(cmd<limits[:,0]+1e-4) or np.any(cmd>limits[:,1]-1e-4):abort('CMD_JOINT_LIMIT_STEP_'+str(step))
            if np.max(np.abs(tg))>=85.:abort('GRAVITY_LIMIT_STEP_'+str(step))
            for i,j in enumerate(ids):qcmd[0,j]=float(cmd[i])
            robot.actuators.target_command.set_position_index(value=qcmd,full_data=True)
            scene.write_data_to_sim()
            sim.step();scene.update(dt)
            robot_forces=rs.data.net_normal_forces_w.torch[0]
            cube_forces=cs.data.net_normal_forces_w.torch[0]
            if not bool(torch.isfinite(robot_forces).all().item() and torch.isfinite(cube_forces).all().item()):
                abort('NONFINITE_CONTACT_DATA_STEP_'+str(step))
            force=float(torch.linalg.vector_norm(robot_forces,dim=-1).max().item())
            cforce=float(torch.linalg.vector_norm(cube_forces,dim=-1).max().item())
            q=robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
            vel=robot.data.joint_vel.torch[0,ids].detach().cpu().numpy().astype(float)
            eff=robot.actuators.applied_effort.torch[0,ids].detach().cpu().numpy().astype(float)
            cube_xyz=cube.data.root_pos_w.torch[0].detach().cpu().numpy().astype(float)
            if not np.isfinite(np.r_[q,vel,eff,cube_xyz]).all():abort('NONFINITE_PHYSICS_STEP_'+str(step))
            err=float(np.max(np.abs(q-qref)));vmax=float(np.max(np.abs(vel)));effmax=float(np.max(np.abs(eff)))
            peakforce=max(peakforce,force);max_tracking=max(max_tracking,err)
            maxvel=max(maxvel,vmax);max_effort=max(max_effort,effmax)
            exceed_eff=exceed_eff+1 if effmax>=95. else 0
            exceed_track=exceed_track+1 if err>=.18 else 0
            if force>=args.contact_threshold_n:abort('EXTERNAL_ROBOT_CONTACT_STEP_{}_N_{:.4f}'.format(step,force))
            if effmax>=99.:abort('ACTUATOR_LIMIT_STEP_'+str(step))
            if exceed_eff>=12:abort('SUSTAINED_TORQUE_GT95_STEP_'+str(step))
            if err>=.45 or exceed_track>=20:abort('TRACKING_ERROR_STEP_'+str(step))
            if vmax>=2.5:abort('JOINT_SPEED_HIGH_STEP_'+str(step))
            if np.any(q<limits[:,0]+.01) or np.any(q>limits[:,1]-.01):abort('ACTUAL_JOINT_LIMIT_STEP_'+str(step))
            if float(torch.linalg.vector_norm(robot.data.root_link_pose_w.torch[0,:3]-root_anchor).item())>.001:
                abort('ROBOT_BASE_MOVED_STEP_'+str(step))
            if step>args.warmup_steps:
                if np.linalg.norm(cube_xyz[:2]-np.array([.3,-.3]))>.008 or abs(cube_xyz[2]-.02)>.008:
                    abort('CUBE_MOVED_STEP_'+str(step))
                if cforce<.2 or cforce>5.:abort('CUBE_TABLE_SENSOR_LOST_STEP_'+str(step))
            if step==args.warmup_steps:
                if not(.2<=cforce<=5. and abs(cube_xyz[2]-.02)<.008):
                    abort('CUBE_TABLE_POSITIVE_CONTROL_FAIL')
                if err>.04:abort('HOME_SETTLING_TRACKING_FAIL')
                print('STAGE51_HOME_SENSOR_BASELINE_PASS=True CUBE_CONTACT_N={:.6f}'.format(cforce),flush=True)
            if phase=='HOLD' and step>total-30:
                last_hold.append((vmax,err))
            if step in sample_steps:
                xyz=robot.data.body_link_pose_w.torch[0,toolidx,:3].detach().cpu().numpy().astype(float)
                row={'step':step,'phase':phase,'progress':float(v),'q_actual_rad':q.tolist(),
                     'max_track_error_rad':err,'max_speed_rad_s':vmax,'max_effort_nm':effmax,
                     'robot_contact_n':force,'cube_contact_n':cforce,'cube_xyz_m':cube_xyz.tolist(),
                     'gripper_base_xyz_m':xyz.tolist()}
                samples.append(row)
                print('STAGE51_SAMPLE step={} phase={} progress={:.3f} tracking_rad={:.5f} effort_nm={:.3f} speed_rad_s={:.4f} robot_contact_n={:.5f} cube_contact_n={:.5f}'.format(step,phase,v,err,effmax,vmax,force,cforce),flush=True)
            if step==total: cube_force_end=cforce
        qend=robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
        tool=robot.data.body_link_pose_w.torch[0,toolidx].detach().cpu().numpy().astype(float)
        toolR=quat_R(tool[3:7]);tool_pos_err=float(np.linalg.norm(tool[:3]-expected_tool_xyz))
        tool_angle_err=rotation_diff_deg(expected_tool_R,toolR)
        final_joint_err=float(np.max(np.abs(qend-qgoal)))
        last_vmax=max(v for v,_ in last_hold) if last_hold else float('inf')
        if not(np.isfinite(tool).all() and np.isfinite(qend).all()):abort('NONFINITE_FINAL_POSE')
        stable=(tool_pos_err<.005 and tool_angle_err<1. and final_joint_err<.025 and
                last_vmax<.1 and max_effort<95. and peakforce<args.contact_threshold_n)
        report.update({'trajectory_fully_executed':True,'simulation_motion_passed':bool(stable),
          'final_q_rad':qend.tolist(),'final_q_target_max_error_rad':final_joint_err,
          'final_gripper_base_pose_xyzw':tool.tolist(),
          'final_tool_position_error_mm':tool_pos_err*1000.,'final_tool_orientation_error_deg':tool_angle_err,
          'last30_hold_max_speed_rad_s':last_vmax,'cube_table_final_contact_n':cube_force_end,
          'peak_robot_external_contact_n':peakforce,'peak_tracking_error_rad':max_tracking,
          'peak_applied_effort_nm':max_effort,'peak_joint_speed_rad_s':maxvel})
        save_report(report)
        print('STAGE51_FINAL_ARM_Q_RAD='+json.dumps(qend.tolist()),flush=True)
        print('STAGE51_FINAL_GRIPPER_WORLD_XYZ_M='+json.dumps(tool[:3].tolist()),flush=True)
        print('STAGE51_FINAL_TOOL_ERROR_MM={:.5f} ORI_DEG={:.5f}'.format(tool_pos_err*1000.,tool_angle_err),flush=True)
        print('STAGE51_PEAK_EXTERNAL_CONTACT_N={:.6f}'.format(peakforce),flush=True)
        print('STAGE51_PEAK_EFFORT_NM={:.4f} PEAK_TRACKING_RAD={:.5f}'.format(max_effort,max_tracking),flush=True)
        print('STAGE51_SIM_MOTION_PASS='+str(bool(stable)),flush=True)
        print('STAGE51_SELF_COLLISIONS_UNTESTED=True NO_HARDWARE_MOTION_PERMISSION=True',flush=True)
        print('STAGE51_GRASP_EXECUTED=False',flush=True)
        print('LEXIUM_STAGE3C51_PHYSX_PREGRASP_MOTION_TRIAL_COMPLETE=True',flush=True)
        print('RESULT='+('STAGE3C51_SIM_PREGRASP_TRACKING_PASSED_SELF_COLLISIONS_NOT_VALIDATED' if stable
                        else 'STAGE3C51_SIM_PREGRASP_TRACKING_NOT_PASSED'),flush=True)
        if not stable:abort('FINAL_HOLD_OR_TARGET_POSE_NOT_STABLE')
    except Exception as exc:
        report.update({'simulation_motion_passed':False,
           'abort_or_error':str(exc),'trajectory_fully_executed':False,
           'peak_robot_external_contact_n':peakforce,'peak_tracking_error_rad':max_tracking,
           'peak_applied_effort_nm':max_effort,'peak_joint_speed_rad_s':maxvel})
        save_report(report)
        print('STAGE51_SIMULATION_ABORTED=True REASON='+str(exc),flush=True)
        raise

if __name__=='__main__':
    try:main()
    finally:simulation_app.close()
