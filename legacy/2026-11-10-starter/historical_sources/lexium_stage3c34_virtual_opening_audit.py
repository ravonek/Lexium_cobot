#!/usr/bin/env python3
"""Stage 3C.33: static collision-vertex cross-section audit for Lexium gripper.

Replays the verified gravity-compensated HOME-to-Candidate1 arm trajectory,
checks contact sensors, then measures four finger collision-mesh envelopes
from runtime PhysX link poses. Never commands finger joints, descends, grasps,
or edits source robot assets. Self-collisions remain disabled; all mesh-vertex
and slab gaps are only proxies, not continuous collision/surface clearance.
"""
from __future__ import annotations
import argparse
import json
import math
import re
from pathlib import Path
from isaaclab.app import AppLauncher

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--candidates-json',type=Path,default=Path('/tmp/lexium_stage3c26_candidates.json'))
p.add_argument('--preflight-log',type=Path,default=Path('/tmp/lexium_stage3c29_preflight.log'))
p.add_argument('--static-log',type=Path,default=Path('/tmp/lexium_stage3c28_candidate1_gravity_bias.log'))
p.add_argument('--urdf',type=Path,default=Path.home()/'Desktop/adi/lexium_rl_workspace/lexium_pick_place/assets/lexium_cobot.urdf')
p.add_argument('--warmup-steps',type=int,default=120)
p.add_argument('--motion-steps',type=int,default=900)
p.add_argument('--hold-steps',type=int,default=300)
p.add_argument('--num_envs',type=int,default=1)
p.add_argument('--scene-variant',choices=('table','full'),required=True)
p.add_argument('--table-center-x',type=float,default=0.62)
p.add_argument('--cube-x',type=float,default=0.30)
p.add_argument('--cube-y',type=float,default=-0.30)
p.add_argument('--robot-contact-threshold-n',type=float,default=0.5)
p.add_argument('--calibration-mode',choices=('audit','probe'),default='audit')
p.add_argument('--stage33-report',type=Path,default=Path('/tmp/lexium_stage3c33_static_gripper_geometry.json'))
p.add_argument('--stage34-report',type=Path,default=Path('/tmp/lexium_stage3c34_virtual_opening_audit.json'))
p.add_argument('--stage32b-report',type=Path,default=Path('/tmp/lexium_stage3c32b_limit_aware_probe.json'))
p.add_argument('--probe-rad',type=float,default=0.004)
p.add_argument('--probe-ramp-steps',type=int,default=120)
p.add_argument('--probe-hold-steps',type=int,default=60)
p.add_argument('--stage32-report',type=Path,default=Path('/tmp/lexium_stage3c32_gripper_calibration.json'))
AppLauncher.add_app_launcher_args(p)
args_cli=p.parse_args()
launcher=AppLauncher(args_cli)
simulation_app=launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from lexium_pick_place.robot_cfg import LEXIUM_COBOT_CFG, LEXIUM_ARM_JOINT_NAMES, LEXIUM_TCP_OFFSET, LEXIUM_USD_PATH
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import (
    parse_urdf, Q_HOME, FINGER_HOME, rpy_to_R, axis_rotation,
)
from lexium_pick_place.lexium_stage3c25_corrected_gravity_route_audit import compute_root_frame_gravity


# Contact reporting is enabled for this TEST SPAWN only; USD and robot_cfg.py remain unchanged.
CONTACT_ROBOT_CFG = LEXIUM_COBOT_CFG.replace(
    prim_path='{ENV_REGEX_NS}/Robot',
    spawn=LEXIUM_COBOT_CFG.spawn.replace(activate_contact_sensors=True),
)


def make_table():
    return AssetBaseCfg(
        prim_path='{ENV_REGEX_NS}/Table',
        spawn=sim_utils.CuboidCfg(
            size=(0.90, 1.20, 0.40),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35,0.35,0.42)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(args_cli.table_center_x,0.,-0.20)),
    )


def make_cube():
    return RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/Cube',
        spawn=sim_utils.CuboidCfg(
            size=(0.04,0.04,0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=2,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            activate_contact_sensors=True,
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15,0.75,0.25)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(args_cli.cube_x,args_cli.cube_y,0.12)),
    )


@configclass
class Stage3C30TableCfg(InteractiveSceneCfg):
    table: AssetBaseCfg = make_table()
    robot: ArticulationCfg = CONTACT_ROBOT_CFG
    robot_contact = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*', update_period=0.0)
    light = AssetBaseCfg(prim_path='/World/Light',
        spawn=sim_utils.DomeLightCfg(intensity=2500.0))


@configclass
class Stage3C30FullCfg(InteractiveSceneCfg):
    table: AssetBaseCfg = make_table()
    robot: ArticulationCfg = CONTACT_ROBOT_CFG
    cube: RigidObjectCfg = make_cube()
    robot_contact = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*', update_period=0.0)
    cube_contact = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Cube', update_period=0.0)
    light = AssetBaseCfg(prim_path='/World/Light',
        spawn=sim_utils.DomeLightCfg(intensity=2500.0))


def quat_to_rot(quat):
    x,y,z,w=map(float,quat)
    mag=(x*x+y*y+z*z+w*w)**0.5
    if mag<1e-8:raise ValueError('Zero root quaternion')
    x,y,z,w=[u/mag for u in (x,y,z,w)]
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]],dtype=float)


def angular_error_deg(R0,R1):
    return float(np.degrees(np.arccos(np.clip((np.trace(R0.T@R1)-1)*.5,-1,1))))


def fk_gripper_base(links,joints,children,root,q):
    qmap=dict(zip(LEXIUM_ARM_JOINT_NAMES,q))
    qmap.update(FINGER_HOME)
    poses={root:(np.eye(3),np.zeros(3))}
    stack=[root]
    while stack:
        parent=stack.pop()
        Rp,pp=poses[parent]
        for jn in children.get(parent,()):
            j=joints[jn]
            Rj=Rp@rpy_to_R(j['rpy'])
            pj=pp+Rp@j['xyz']
            qi=float(qmap.get(jn,0))
            if j['type'] in ('revolute','continuous'):
                Rc,pc=Rj@axis_rotation(j['axis'],qi),pj
            elif j['type']=='prismatic':
                Rc,pc=Rj,pj+Rj@j['axis']*qi
            else:Rc,pc=Rj,pj
            poses[j['child']]=(Rc,pc)
            stack.append(j['child'])
    return poses['gripper_base_1']


def abort(message):
    print(f'SAFETY_ABORT={message}',flush=True)
    raise RuntimeError(f'Safety guard: {message}')


def load_verified_candidate():
    for f in (args_cli.candidates_json,args_cli.static_log,args_cli.preflight_log,args_cli.urdf):
        if not f.is_file():raise FileNotFoundError(f'Missing prerequisite {f}')
    pre=args_cli.preflight_log.read_text(encoding='utf8',errors='replace')
    for s in ('PRELIMINARY_STATIC_GRAVITY_GATE_PASSED=True','STAGE3C29_PREFLIGHT_COMPLETE=True'):
        if not re.search('^'+re.escape(s)+'$',pre,re.M):raise RuntimeError('Offline preflight not approved: '+s)
    st=args_cli.static_log.read_text(encoding='utf8',errors='replace')
    for s in ('STAGE3C28_ACCEPTABLE_STATIC_POSE=True','LEXIUM_STAGE3C28_DIAGNOSTIC_COMPLETE=True'):
        if not re.search('^'+re.escape(s)+'$',st,re.M):raise RuntimeError('Stage 3C28 static test incomplete: '+s)
    doc=json.loads(args_cli.candidates_json.read_text(encoding='utf8'))
    candidate=doc['valid_distinct'][0]
    qtarget=np.asarray(candidate['q_rad'],dtype=float)
    goal=np.asarray(doc['goal_xyz_world_m'],dtype=float)
    if candidate['orientation']!='TOOL_Y_+45DEG' or not candidate['valid_pose']:\
        raise RuntimeError('Unexpected top-ranked IK candidate')
    if qtarget.shape!=(6,) or not np.isfinite(qtarget).all() or not np.allclose(goal,[.3,-.3,.2],atol=1e-7):
        raise RuntimeError('Candidate changed')
    match=re.search(r'^ARM_Q_REFERENCE_RAD=(\[[^\n]+\])$',st,re.M)
    if not match or not np.allclose(json.loads(match.group(1)),qtarget,atol=1e-6):
        raise RuntimeError('Candidate does not match previously physically validated candidate')
    return qtarget,goal



# Stage 3C.31 add-on: collision-mesh vertex geometry at the *actual*
# stabilized PhysX articulation pose. This is a sampled-vertex proxy,
# NOT a continuous mesh contact or fingertip calibration.
def report_stage31_gripper_geometry(robot, cube, stage, tcp_world, tcp_offset, tool_R, contact_passed, stable):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    if cube is None:
        raise RuntimeError('STAGE31_GEOMETRY_REQUIRES_CUBE')
    if not (contact_passed and stable):
        raise RuntimeError('STAGE31_REQUIRES_VERIFIED_STAGE30_PHYSICAL_MOTION')

    body_names=list(robot.data.body_names)
    wanted=('gripper_base_1', 'gripper_finger_1_1', 'gripper_finger_2_1',
            'gripper_finger_3_1', 'gripper_finger_4_1')
    for name in wanted:
        if name not in body_names:
            raise RuntimeError('MISSING_PHYSX_RIGID_BODY='+name)

    rigid_prims=[p for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.RigidBodyAPI)
                 and str(p.GetPath()).startswith('/World/envs/env_0/Robot/')]
    # PhysX collision meshes in a USD referenced model may be instance proxies.
    meshes=[p for p in stage.Traverse(Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)
            and str(p.GetPath()).startswith('/World/envs/env_0/Robot/')]
    xf=UsdGeom.XformCache(Usd.TimeCode.Default())
    body_poses=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    cube_xyz=cube.data.root_pos_w.torch[0].detach().cpu().numpy().astype(float)
    cube_q=cube.data.root_quat_w.torch[0].detach().cpu().numpy().astype(float)
    Rcube=quat_to_rot(cube_q)
    half=np.asarray([0.02, 0.02, 0.02],dtype=float)
    corner_sign=np.asarray([[sx,sy,sz] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)],dtype=float)
    cube_corners=cube_xyz[None,:]+(corner_sign*half[None,:])@Rcube.T
    cube_top=float(cube_corners[:,2].max())
    cube_bottom=float(cube_corners[:,2].min())
    by_body={}
    local_by_body={}
    print('STAGE3C31_GEOMETRY_STARTED=True',flush=True)
    print('GEOMETRY_SOURCE=USD_COLLISION_MESH_VERTICES_AT_RUNTIME_PHYSX_LINK_POSE',flush=True)
    print('GEOMETRY_METHOD=SAMPLED_VERTICES_ONLY_NOT_EXACT_MESH_SURFACE_DISTANCE',flush=True)
    print(f'CUBE_ACTUAL_CENTER_XYZ_M={cube_xyz.tolist()} CUBE_QUAT_XYZW={cube_q.tolist()} '
          f'CUBE_ACTUAL_Z_EXTENTS_M={[cube_bottom,cube_top]}',flush=True)
    print(f'ACTUAL_TCP_WORLD_XYZ_M={tcp_world.tolist()} TCP_TO_CUBE_CENTER_M={float(np.linalg.norm(tcp_world-cube_xyz)):.6f}',flush=True)
    # Useful for planning: is the cube below the gripper in its current local frame?
    local_vec=tool_R.T@(cube_xyz-tcp_world)
    print(f'CUBE_CENTER_RELATIVE_TO_TCP_IN_TOOL_XYZ_M={local_vec.tolist()}',flush=True)
    print(f'GRIPPER_TOOL_LOCAL_Z_AXIS_WORLD={tool_R[:,2].tolist()} '
          f'LOCAL_Z_VERTICAL_ABS_ALIGNMENT={abs(float(tool_R[2,2])):.6f}',flush=True)
    results={}
    max_sample=3000
    for name in wanted:
        candidates=[p for p in rigid_prims if p.GetName()==name]
        if len(candidates)!=1:
            raise RuntimeError(f'RIGID_BODY_PRIM_COUNT_{name}={len(candidates)}')
        rigid=candidates[0]
        subtree=str(rigid.GetPath()).rstrip('/')+'/'
        parts=[p for p in meshes if str(p.GetPath()).startswith(subtree)]
        if not parts:
            raise RuntimeError(f'MISSING_COLLISION_MESH_FOR_{name}')
        all_pts=[]
        total_pts=0
        for mesh in parts:
            pts=UsdGeom.Mesh(mesh).GetPointsAttr().Get()
            if pts is None or len(pts)==0:
                raise RuntimeError(f'EMPTY_COLLISION_MESH={mesh.GetPath()}')
            total_pts+=len(pts)
            ids=np.unique(np.linspace(0,len(pts)-1,num=min(len(pts),max_sample),dtype=int))
            # Rendered USD transform may not track PhysX after stepping. It is
            # only used to convert USD mesh points to *rigid-link-local* XYZ.
            mesh_world=xf.GetLocalToWorldTransform(mesh)
            world_to_link=xf.GetLocalToWorldTransform(rigid).GetInverse()
            for ix in ids:
                v=pts[int(ix)]
                pt=Gf.Vec3d(float(v[0]),float(v[1]),float(v[2]))
                p=world_to_link.Transform(mesh_world.Transform(pt))
                all_pts.append((float(p[0]),float(p[1]),float(p[2])))
        local=np.asarray(all_pts,dtype=float)
        if not np.isfinite(local).all() or local.shape[0]<3:
            raise RuntimeError(f'INVALID_COLLIDER_POINTS_{name}')
        pose=body_poses[body_names.index(name)]
        R=quat_to_rot(pose[3:7])
        world=pose[:3][None,:]+local@R.T
        lower=world.min(axis=0);upper=world.max(axis=0)
        centroid=world.mean(axis=0)
        # Signed distance of sampled VERTICES to oriented cube box. A negative
        # value means a sampled vertex lies inside the cube; it cannot detect
        # face-to-face contact when no sampled vertex is inside either surface.
        points_in_cube=(world-cube_xyz[None,:])@Rcube
        delta=np.abs(points_in_cube)-half[None,:]
        outside=np.linalg.norm(np.maximum(delta,0.0),axis=1)
        inside=np.minimum(np.max(delta,axis=1),0.)
        sdf=outside+inside
        min_vertex_surface=float(np.min(np.abs(sdf)))
        min_signed=float(np.min(sdf))
        inside_count=int(np.count_nonzero(sdf<0))
        within_10mm=int(np.count_nonzero(np.abs(sdf)<0.010))
        xy_distance=float(np.linalg.norm(centroid[:2]-cube_xyz[:2]))
        print(f'COLLIDER_WORLD name={name} meshes={len(parts)} vertices_total={total_pts} '
              f'vertices_sampled={len(world)} aabb_min_m={lower.tolist()} '
              f'aabb_max_m={upper.tolist()} sampled_centroid_m={centroid.tolist()}',flush=True)
        print(f'COLLIDER_CUBE_VERTEX_PROXY name={name} min_abs_vertex_box_surface_m={min_vertex_surface:.6f} '
              f'min_signed_vertex_box_sdf_m={min_signed:.6f} vertices_inside_box={inside_count} '
              f'vertices_within_10mm_of_box={within_10mm} '
              f'min_world_z_above_table_m={float(lower[2]):.6f} '
              f'sampled_centroid_xy_dist_to_cube_m={xy_distance:.6f}',flush=True)
        results[name]={
            'mesh_count':len(parts), 'total_vertices':int(total_pts),
            'sampled_vertices':int(len(world)), 'aabb_world_min_m':lower.tolist(),
            'aabb_world_max_m':upper.tolist(), 'sampled_vertex_mean_world_m':centroid.tolist(),
            'nearest_sampled_vertex_to_cube_surface_m':min_vertex_surface,
            'min_signed_vertex_cube_sdf_m':min_signed,
            'sampled_vertices_inside_cube_count':inside_count,
            'sampled_vertices_within_10mm_cube_count':within_10mm,
        }
        by_body[name]=world
        local_by_body[name]=local
    # Opposing pairs 1/2 vs 3/4 from earlier 3B. This remains just a projected
    # collider envelope, not actual fingertip opening or grasp clearance.
    p0=by_body['gripper_finger_1_1'];p1=by_body['gripper_finger_2_1']
    p2=by_body['gripper_finger_3_1'];p3=by_body['gripper_finger_4_1']
    group_a=np.concatenate((p0,p1),axis=0)
    group_b=np.concatenate((p2,p3),axis=0)
    axis=group_b.mean(axis=0)-group_a.mean(axis=0)
    norm=np.linalg.norm(axis)
    if norm<=1e-8:
        raise RuntimeError('UNDEFINED_GROUP_SEPARATION_AXIS')
    axis/=norm
    gap=float(np.min(group_b@axis)-np.max(group_a@axis))
    print(f'FINGER_GROUP_PROJECTED_COLLIDER_GAP_M={gap:.6f} GROUP_GAP_AXIS_WORLD={axis.tolist()} '
          'GROUPING=FINGERS_1_2_VS_3_4',flush=True)
    # Save small report, not heavy point clouds.
    report={
        'stage':'3C31', 'source':'runtime_physx_pose_usd_sampled_collision_mesh',
        'cube_center_xyz_m':cube_xyz.tolist(), 'cube_quat_xyzw':cube_q.tolist(),
        'cube_top_world_z_m':cube_top, 'actual_tcp_world_xyz_m':tcp_world.tolist(),
        'cube_center_relative_tcp_tool_xyz_m':local_vec.tolist(),
        'gripper_tool_local_z_axis_world':tool_R[:,2].tolist(),
        'collider_geometry':results,
        'opposing_group_projected_gap_m':gap,
        'contact_motion_verified':True,
        'disclaimer':'Vertex-to-box and projected gaps are PROXIES; do not establish exact mesh clearance or grasp feasibility.',
    }
    out=Path('/tmp/lexium_stage3c31_gripper_geometry.json')
    out.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(f'GEOMETRY_REPORT_JSON={out}',flush=True)
    print('GRIPPER_FINGER_COLLIDER_COUNT=4',flush=True)
    print('GRASP_VALIDATED=False FINGERTIP_TCP_CALIBRATED=False',flush=True)
    print('LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True',flush=True)
    return {'local_points':local_by_body,'group_axis':axis.copy(), 'finger_names':list(wanted[1:])}

# Stage 3C.32 has no independent arm-motion controller: it is appended to the
# original, unmodified Stage3C31 trajectory and safety screening above.
STAGE32_FINGER_NAMES=(
    'gripper_Revolute_1', 'gripper_Revolute_2',
    'gripper_Revolute_5', 'gripper_Revolute_6',
)


def stage32_geometry_snapshot(robot, cube, cached):
    """Sampled collider-vertex envelope, rigid-link PhysX poses, fixed baseline axis.

    This is a PROXY; it does not certify distance to continuous surfaces, nor
    certify swept motion, fingertips, or internal robot self-collision.
    """
    names=list(robot.data.body_names)
    poses=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    by_name={}
    lower_z=float('inf')
    cxyz=cube.data.root_pos_w.torch[0].detach().cpu().numpy().astype(float)
    cq=cube.data.root_quat_w.torch[0].detach().cpu().numpy().astype(float)
    Rc=quat_to_rot(cq)
    min_proxy=float('inf')
    half=np.full(3,0.02)
    for name,local in cached['local_points'].items():
        pose=poses[names.index(name)]
        xyz=pose[:3][None,:]+local@quat_to_rot(pose[3:7]).T
        by_name[name]=xyz
        if name.startswith('gripper_finger'):
            lower_z=min(lower_z,float(xyz[:,2].min()))
            delta=np.abs((xyz-cxyz[None,:])@Rc)-half[None,:]
            sdf=(np.linalg.norm(np.maximum(delta,0),axis=1)
                 +np.minimum(np.max(delta,axis=1),0))
            min_proxy=min(min_proxy,float(sdf.min()))
    axis=cached['group_axis']
    ga=np.concatenate([by_name['gripper_finger_1_1'],by_name['gripper_finger_2_1']])
    gb=np.concatenate([by_name['gripper_finger_3_1'],by_name['gripper_finger_4_1']])
    gap=float((gb@axis).min()-(ga@axis).max())
    return {
        'fixed_axis_projected_gap_m':gap,
        'min_finger_z_m':lower_z,
        'min_sampled_finger_vertex_cube_sdf_m':min_proxy,
        'cube_xyz_m':cxyz.tolist(),
        'finger_colliders_world_centroid_xyz_m':{
            n:by_name[n].mean(axis=0).tolist() for n in cached['finger_names']},
    }


def run_stage32(robot, cube, sensor, cube_sensor, sim, scene, armids,
                base_target, tcp_offset, toolid, goal, desired_tool_R,
                rootstart, cached, arm_max_efforts_nm, urdf_joints):
    if not 0 < args_cli.probe_rad <= 0.006:
        abort('STAGE32B_PROBE_SIZE_NOT_WITHIN_0_TO_0P006_RAD')
    if not 90 <= args_cli.probe_ramp_steps <= 300 or not 45 <= args_cli.probe_hold_steps <= 180:
        abort('STAGE32_PROBE_TIMING_OUTSIDE_GUARDS')
    jointnames=list(robot.data.joint_names)
    print(f'STAGE32_ALL_GRIPPER_JOINT_NAMES={[n for n in jointnames if "gripper" in n.lower()]}',flush=True)
    if any(n not in jointnames for n in STAGE32_FINGER_NAMES):
        abort(f'STAGE32_MISSING_FINGER_JOINTS_{[n for n in STAGE32_FINGER_NAMES if n not in jointnames]}')
    fingerids=[jointnames.index(n) for n in STAGE32_FINGER_NAMES]
    q=robot.data.joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
    vel=robot.data.joint_vel.torch[0,fingerids].detach().cpu().numpy().copy()
    limits=robot.data.joint_pos_limits.torch[0,fingerids].detach().cpu().numpy().copy()
    kp=robot.data.joint_stiffness.torch[0,fingerids].detach().cpu().numpy().copy()
    kd=robot.data.joint_damping.torch[0,fingerids].detach().cpu().numpy().copy()
    effort_limits=robot.data.joint_effort_limits.torch[0,fingerids].detach().cpu().numpy().copy()
    default_q=robot.data.default_joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
    if not all(np.isfinite(a).all() for a in (q,vel,limits,kp,kd,effort_limits,default_q)):
        abort('STAGE32_NONFINITE_FINGER_METADATA')
    geom=stage32_geometry_snapshot(robot,cube,cached)
    print('LEXIUM_STAGE3C32_AUDIT_STARTED=True',flush=True)
    print(f'STAGE32_MODE={args_cli.calibration_mode.upper()} PROBE_RAD={args_cli.probe_rad:.6f}',flush=True)
    print(f'STAGE32_START_GEOMETRY={json.dumps(geom)}',flush=True)
    for n,qq,v,lim,k,d,eff,df in zip(STAGE32_FINGER_NAMES,q,vel,limits,kp,kd,effort_limits,default_q):
        print(f'STAGE32_FINGER_JOINT name={n} pos_rad={qq:.9f} vel_rad_s={v:.7f} '
              f'limits_rad={lim.tolist()} stiffness={k:.6f} damping={d:.6f} '
              f'effort_limit_nm={eff:.6f} default_rad={df:.9f}',flush=True)
    warnings=[]
    if (geom['min_finger_z_m']<0.08 or
        geom['min_sampled_finger_vertex_cube_sdf_m']<0.07 or
        geom['fixed_axis_projected_gap_m']<0.015):
        warnings.append('STARTING_GEOMETRY_TOO_CLOSE_TO_TABLE_OR_CUBE')
    if np.any(q < limits[:,0]+0.001) or np.any(q > limits[:,1]-0.001):
        warnings.append('FINGER_POSITION_TOO_CLOSE_TO_JOINT_LIMIT')
    if np.any(np.abs(vel)>0.1):
        warnings.append('FINGER_NOT_AT_REST')
    if np.any(kp<=0) or np.any(effort_limits<=0):
        warnings.append('NO_POSITIVE_POSITION_DRIVE_OR_EFFORT_LIMIT')
    urdf_metadata={}
    for n in STAGE32_FINGER_NAMES:
        if n not in urdf_joints:
            warnings.append('NOT_IN_URDF_'+n)
            continue
        j=urdf_joints[n]
        urdf_metadata[n]={
            'type':j['type'],'parent':j['parent'],'child':j['child'],
            'axis_xyz':np.asarray(j['axis']).tolist(),
            'origin_xyz_m':np.asarray(j['xyz']).tolist(),
            'origin_rpy_rad':np.asarray(j['rpy']).tolist(),
            'urdf_home_rad':float(FINGER_HOME[n]) if n in FINGER_HOME else None,
        }
        print(f'STAGE32_FINGER_URDF name={n} '
              f'type={j["type"]} parent={j["parent"]} child={j["child"]} '
              f'axis_xyz={np.asarray(j["axis"]).tolist()}',flush=True)
    # A joint slightly at a hard stop can safely be diagnosed with INWARD-ONLY
    # small probes; all other audit warnings remain hard blockers.
    hard_warnings=[w for w in warnings if w!='FINGER_POSITION_TOO_CLOSE_TO_JOINT_LIMIT']
    allowed_q=np.clip(q,limits[:,0]+0.001,limits[:,1]-0.001)
    # Numerical tolerance for a stabilised PhysX joint close to its lower stop.
    if np.any(q < limits[:,0]-0.0005) or np.any(q > limits[:,1]+0.0005):
        hard_warnings.append('FINGER_OUTSIDE_JOINT_LIMIT_BY_MORE_THAN_0P0005_RAD')
    probe_plan=[]
    for j,name in enumerate(STAGE32_FINGER_NAMES):
        for sign in (1,-1):
            goal_j=float(allowed_q[j]+sign*args_cli.probe_rad)
            valid=bool(limits[j,0]+0.002 < goal_j < limits[j,1]-0.002)
            probe_plan.append({'joint':name,'sign':sign,'target_rad':goal_j,'planned':valid})
    limit_aware_eligible=not hard_warnings and any(x['planned'] for x in probe_plan)
    print(f'STAGE32_MICRO_PROBE_ELIGIBLE={not warnings} WARNINGS={warnings}',flush=True)
    print(f'STAGE32B_LIMIT_AWARE_ELIGIBLE={limit_aware_eligible} HARD_WARNINGS={hard_warnings}',flush=True)
    print(f'STAGE32B_LIMIT_AWARE_BASELINE_RAD={allowed_q.tolist()}',flush=True)
    for row in probe_plan:
        print(f'STAGE32B_DIRECTION joint={row["joint"]} sign={row["sign"]:+d} '
              f'target_rad={row["target_rad"]:.7f} allowed={row["planned"]}',flush=True)
    report={
        'stage':'3C32B', 'mode':args_cli.calibration_mode,
        'robot_joint_names':jointnames,
        'finger_names':list(STAGE32_FINGER_NAMES),
        'finger_q_start_rad':q.tolist(),
        'finger_default_q_rad':default_q.tolist(),
        'finger_joint_pos_limits_rad':limits.tolist(),
        'finger_joint_stiffness':kp.tolist(),
        'finger_joint_damping':kd.tolist(),
        'finger_joint_effort_limits_nm':effort_limits.tolist(),
        'finger_urdf_metadata':urdf_metadata,
        'micro_probe_eligible':not warnings,
        'limit_aware_eligible':limit_aware_eligible,
        'limit_aware_probe_plan':probe_plan,
        'limit_aware_hold_rad':allowed_q.tolist(),
        'audit_warnings':warnings,
        'start_geometry_proxy':geom,
        'cube_size_m':0.04, 'cube_is_dynamic':True,
        'max_arm_effort_prior_nm':arm_max_efforts_nm.tolist(),
        'self_collisions_enabled':False,
        'fingertip_tcp_calibrated':False,
        'grasp_validated':False,
        'note':'Projection and sampled vertices are PROXIES; not swept collision nor valid grasp.',
        'probes':[],
    }
    if args_cli.calibration_mode=='audit':
        args_cli.stage32_report.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(f'STAGE32_REPORT_JSON={args_cli.stage32_report}',flush=True)
        print('LEXIUM_STAGE3C32_JOINT_AUDIT_PASSED=True',flush=True)
        print('FINGER_MOVEMENT_COMMANDED=False',flush=True)
        return

    if not limit_aware_eligible:
        abort('STAGE32B_PREFLIGHT_NOT_ELIGIBLE_'+','.join(hard_warnings))
    # The audit run is a required operator checkpoint before enabling motors.
    audit=Path('/tmp/lexium_stage3c32b_limit_aware_audit.json')
    if not audit.is_file():abort('STAGE32_PROBE_REQUIRES_PREVIOUS_AUDIT_REPORT')
    saved=json.loads(audit.read_text())
    if saved.get('mode')!='audit' or not saved.get('limit_aware_eligible') or saved.get('finger_names')!=list(STAGE32_FINGER_NAMES):
        abort('STAGE32_SAVED_AUDIT_MISMATCH')
    if not np.allclose(saved['finger_q_start_rad'],q,atol=.005):
        abort('STAGE32_START_FINGER_POSE_CHANGED_SINCE_AUDIT')
    if not np.allclose(saved['finger_joint_pos_limits_rad'],limits,atol=1e-5):
        abort('STAGE32_FINGER_LIMITS_CHANGED_SINCE_AUDIT')
    print('LEXIUM_STAGE3C32B_LIMIT_AWARE_PROBE_STARTED=True',flush=True)
    print('STAGE32_POLICY=ONE_FINGER_JOINT_AT_A_TIME_NO_DESCENT_NO_SIMULTANEOUS_GRASP',flush=True)
    original_xyz=np.asarray(geom['cube_xyz_m'])
    dt=sim.get_physics_dt()
    bias_full=base_target.clone()
    q_hold=allowed_q.copy()
    # During every probe the arm target and gravity compensation remain fixed.
    # No modifications to any source USD, URDF or robot_cfg.py.
    def guarded_step(target_pos, index_tag):
        if not simulation_app.is_running():abort('STAGE32_SIM_STOPPED')
        commanded=bias_full.clone()
        for j,idj in enumerate(fingerids):commanded[0,idj]=float(target_pos[j])
        robot.actuators.target_command.set_position_index(value=commanded,full_data=True)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        rforce=sensor.data.net_normal_forces_w.torch[0]
        if not bool(torch.isfinite(rforce).all().item()):abort('STAGE32_INVALID_ROBOT_CONTACT_SENSOR')
        rc=float(torch.linalg.vector_norm(rforce,dim=-1).max().item())
        if rc>=args_cli.robot_contact_threshold_n:
            abort(f'STAGE32_EXTERNAL_CONTACT_{index_tag}_{rc:.4f}N')
        cforce=cube_sensor.data.net_normal_forces_w.torch[0]
        if not bool(torch.isfinite(cforce).all().item()):abort('STAGE32_INVALID_CUBE_SENSOR')
        cforce_max=float(torch.linalg.vector_norm(cforce,dim=-1).max().item())
        if cforce_max<0.15:
            abort(f'STAGE32_CUBE_TABLE_CONTACT_LOST_{index_tag}_{cforce_max:.4f}N')
        cpos=cube.data.root_pos_w.torch[0].detach().cpu().numpy()
        if np.linalg.norm(cpos-original_xyz)>0.005:
            abort(f'STAGE32_CUBE_MOVED_{index_tag}_{cpos.tolist()}')
        eff=robot.actuators.applied_effort.torch[0]
        actual=robot.data.joint_pos.torch[0]
        actual_f=actual[fingerids].detach().cpu().numpy()
        fvel=robot.data.joint_vel.torch[0,fingerids].detach().cpu().numpy()
        if not np.isfinite(actual_f).all() or not np.isfinite(fvel).all():
            abort(f'STAGE32_INVALID_FINGER_PHYSICS_{index_tag}')
        # The original audit measured finger 6 at most ~1.3e-5 rad below its
        # nominal lower limit. Reject any larger excursion (>5e-4 rad), rather
        # than rejecting harmless simulation tolerance before the inward ramp.
        if np.any(actual_f<limits[:,0]-0.0005) or np.any(actual_f>limits[:,1]+0.0005):
            abort(f'STAGE32B_ACTUAL_FINGER_OUTSIDE_LIMIT_TOLERANCE_{index_tag}')
        if np.any(fvel>0.2) or np.any(fvel < -0.2):
            abort(f'STAGE32B_FINGER_SPEED_LIMIT_{index_tag}')
        if np.max(np.abs(actual_f-q))>0.035:
            abort(f'STAGE32_FINGER_OUTSIDE_SMALL_CALIBRATION_RANGE_{index_tag}')
        if np.max(np.abs(eff[armids].detach().cpu().numpy()))>=95.:
            abort(f'STAGE32_ARM_EFFORT_OVER_95NM_{index_tag}')
        f_eff=np.abs(eff[fingerids].detach().cpu().numpy())
        if np.any(f_eff>np.minimum(15.0,0.20*effort_limits)):
            abort(f'STAGE32_FINGER_EFFORT_NEAR_LIMIT_{index_tag}_{f_eff.tolist()}')
        toolpose=robot.data.body_link_pose_w.torch[0,toolid].detach().cpu().numpy()
        Rtool=quat_to_rot(toolpose[3:7])
        tcp=toolpose[:3]+Rtool@tcp_offset
        err=float(np.linalg.norm(tcp-goal))
        ori_err=angular_error_deg(desired_tool_R,Rtool)
        rootdisp=float(torch.linalg.vector_norm(robot.data.root_link_pose_w.torch[0,:3]-rootstart).item())
        if err>=0.005 or ori_err>=1.0 or rootdisp>=0.001:
            abort(f'STAGE32_ARM_HOLD_DRIFT_{index_tag}_tcp_m_{err:.6f}_ori_deg_{ori_err:.3f}')
        return actual_f, f_eff, rc

    ramp=args_cli.probe_ramp_steps
    hold=args_cli.probe_hold_steps
    # Bring any at-limit finger *inward* by at most ~0.001 rad before probes.
    # For this audited pose, only gripper_Revolute_6 needs the gentle adjustment.
    # No arm motion, no grasp, and no simultaneous finger opening.
    if np.any(np.abs(q_hold-q)>1e-9):
        print('STAGE32B_LIMIT_RECENTER_STARTED=True',flush=True)
        for t in range(1,ramp+hold+1):
            a=(10*(t/ramp)**3-15*(t/ramp)**4+6*(t/ramp)**5) if t<=ramp else 1.0
            target=q+(q_hold-q)*a
            guarded_step(target,f'LIMIT_RECENTER_{t}')
        qpre=robot.data.joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
        if np.max(np.abs(qpre-q_hold))>0.002:
            abort(f'STAGE32B_LIMIT_RECENTER_NOT_TRACKED_{qpre.tolist()}')
        pre_snap=stage32_geometry_snapshot(robot,cube,cached)
        if (pre_snap['min_finger_z_m']<0.08 or
            pre_snap['min_sampled_finger_vertex_cube_sdf_m']<0.07 or
            pre_snap['fixed_axis_projected_gap_m']<0.015):
            abort('STAGE32B_LIMIT_RECENTER_GEOMETRY_GUARD')
        print(f'STAGE32B_LIMIT_RECENTER_DONE=True actual_rad={qpre.tolist()}',flush=True)
    expected_passes=sum(row['planned'] for row in probe_plan)
    for j,name in enumerate(STAGE32_FINGER_NAMES):
        for sign in (1,-1):
            goal_rad=float(q_hold[j]+sign*args_cli.probe_rad)
            if not limits[j,0]+0.002<goal_rad<limits[j,1]-0.002:
                print(f'STAGE32_PROBE_SKIPPED joint={name} sign={sign:+d} reason=NEAR_JOINT_LIMIT',flush=True)
                report['probes'].append({'joint':name,'sign':sign,'status':'SKIPPED_LIMIT'})
                continue
            # All 4 target joints start from original calibrated baseline.
            startpos=robot.data.joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
            if np.max(np.abs(startpos-q_hold))>0.003:
                abort(f'STAGE32_NOT_RECENTERED_BEFORE_{name}_sign_{sign}')
            max_f_eff=np.zeros(len(fingerids))
            max_contact=0.0
            for t in range(1,ramp+hold+1):
                a=(10*(t/ramp)**3-15*(t/ramp)**4+6*(t/ramp)**5) if t<=ramp else 1.0
                target=q_hold.copy()
                target[j]+=sign*args_cli.probe_rad*a
                act,fe,rc=guarded_step(target,f'{name}_{sign}_OUT_{t}')
                if t%20==0 or t==ramp+hold:
                    snap_during=stage32_geometry_snapshot(robot,cube,cached)
                    if (snap_during['min_finger_z_m']<0.08 or
                        snap_during['min_sampled_finger_vertex_cube_sdf_m']<0.07 or
                        snap_during['fixed_axis_projected_gap_m']<0.015):
                        abort(f'STAGE32B_INTERMEDIATE_GEOMETRY_GUARD_{name}_{sign}_{t}')
                max_f_eff=np.maximum(max_f_eff,fe)
                max_contact=max(max_contact,rc)
            qout=robot.data.joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
            snap=stage32_geometry_snapshot(robot,cube,cached)
            if (snap['min_finger_z_m']<0.08 or
                snap['min_sampled_finger_vertex_cube_sdf_m']<0.07 or
                snap['fixed_axis_projected_gap_m']<0.015):
                abort(f'STAGE32_GEOMETRY_TOO_CLOSE_AFTER_{name}_sign_{sign}')
            moved=float(qout[j]-q_hold[j])
            if abs(moved-sign*args_cli.probe_rad)>0.003:
                abort(f'STAGE32_FINGER_COMMAND_NOT_TRACKED_{name}_sign_{sign}_measured_{moved:.6f}')
            # Return ALL finger targets to audit baseline, same gentle ramp.
            for t in range(1,ramp+hold+1):
                a=(10*(t/ramp)**3-15*(t/ramp)**4+6*(t/ramp)**5) if t<=ramp else 1.0
                target=q_hold.copy()
                target[j]=qout[j]+(q_hold[j]-qout[j])*a
                act,fe,rc=guarded_step(target,f'{name}_{sign}_RETURN_{t}')
                if t%20==0 or t==ramp+hold:
                    snap_during=stage32_geometry_snapshot(robot,cube,cached)
                    if (snap_during['min_finger_z_m']<0.08 or
                        snap_during['min_sampled_finger_vertex_cube_sdf_m']<0.07 or
                        snap_during['fixed_axis_projected_gap_m']<0.015):
                        abort(f'STAGE32B_RETURN_GEOMETRY_GUARD_{name}_{sign}_{t}')
                max_f_eff=np.maximum(max_f_eff,fe)
                max_contact=max(max_contact,rc)
            qr=robot.data.joint_pos.torch[0,fingerids].detach().cpu().numpy().copy()
            if np.max(np.abs(qr-q_hold))>0.003:
                abort(f'STAGE32_FINGERS_FAILED_TO_RECENTER_{name}_sign_{sign}')
            snap_home=stage32_geometry_snapshot(robot,cube,cached)
            if abs(snap_home['fixed_axis_projected_gap_m']-geom['fixed_axis_projected_gap_m'])>0.004:
                abort(f'STAGE32_GEOMETRY_NOT_RETURNED_{name}_sign_{sign}')
            entry={
                'joint':name,'sign':sign,'status':'PASS',
                'command_delta_rad':sign*args_cli.probe_rad,
                'actual_joint_delta_rad':moved,
                'actual_all_finger_delta_rad':(qout-q).tolist(),
                'fixed_axis_gap_at_probe_m':snap['fixed_axis_projected_gap_m'],
                'fixed_axis_gap_change_m':snap['fixed_axis_projected_gap_m']-geom['fixed_axis_projected_gap_m'],
                'min_finger_z_m':snap['min_finger_z_m'],
                'min_sampled_finger_vertex_cube_sdf_m':snap['min_sampled_finger_vertex_cube_sdf_m'],
                'max_finger_effort_nm':max_f_eff.tolist(),
                'max_robot_contact_n':max_contact,
            }
            report['probes'].append(entry)
            print(f'STAGE32_MICRO_PROBE joint={name} sign={sign:+d} '
                  f'measured_delta_rad={moved:+.7f} '
                  f'projected_gap_delta_m={entry["fixed_axis_gap_change_m"]:+.7f} '
                  f'min_cube_vertex_proxy_m={snap["min_sampled_finger_vertex_cube_sdf_m"]:.5f} '
                  f'peak_robot_contact_n={max_contact:.5f} status=PASS',flush=True)
    report['end_geometry_proxy']=stage32_geometry_snapshot(robot,cube,cached)
    actual_passes=sum(x['status']=='PASS' for x in report['probes'])
    skipped=sum(x['status']=='SKIPPED_LIMIT' for x in report['probes'])
    report['probe_complete']=(len(report['probes'])==8
                              and actual_passes==expected_passes
                              and actual_passes>0
                              and actual_passes+skipped==8)
    report['expected_passes']=expected_passes
    report['actual_passes']=actual_passes
    report['skipped_limit']=skipped
    args_cli.stage32_report.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(f'STAGE32_REPORT_JSON={args_cli.stage32_report}',flush=True)
    print(f'LEXIUM_STAGE3C32B_LIMIT_AWARE_PROBE_COMPLETE={report["probe_complete"]} PASSES={actual_passes} SKIPPED={skipped}',flush=True)
    print('GRASP_VALIDATED=False FINGERTIP_TCP_CALIBRATED=False OPENING_40MM_VALIDATED=False',flush=True)


def _nearest_vertex_pair_distance_m(a, b):
    # MINIMUM sampled VERTEX-TO-VERTEX distance; NOT a certified mesh clearance.
    # Process bounded chunks to avoid building a huge NxMx3 temporary array.
    best2 = float('inf')
    b2 = np.sum(b*b, axis=1)
    bt = b.T
    for start in range(0, len(a), 96):
        aa=a[start:start+96]
        d2=np.sum(aa*aa,axis=1)[:,None]+b2[None,:]-2.0*(aa@bt)
        best2=min(best2,float(np.min(d2)))
    return float(np.sqrt(max(best2,0.0)))


def _sampled_gap_for_groups(a,b,sep):
    return float(np.min(b@sep)-np.max(a@sep))


def run_stage33_static_geometry(robot,cube,cached,tcp_offset,toolid,goal):
    # No physics stepping or finger targets are changed in this stage.
    assert cached is not None and cube is not None
    body_names=list(robot.data.body_names)
    poses=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    names=cached['finger_names']
    by_name={}
    for name in names:
        local=cached['local_points'][name]
        pose=poses[body_names.index(name)]
        by_name[name]=pose[:3][None,:]+local@quat_to_rot(pose[3:7]).T
    group_a=np.concatenate((by_name[names[0]],by_name[names[1]]))
    group_b=np.concatenate((by_name[names[2]],by_name[names[3]]))
    sep=np.array(cached['group_axis'],dtype=float)
    sep=sep/np.linalg.norm(sep)
    gap=_sampled_gap_for_groups(group_a,group_b,sep)
    toolpose=poses[toolid]
    Rt=quat_to_rot(toolpose[3:7])
    tcp=toolpose[:3]+Rt@tcp_offset
    cubepose=cube.data.root_pos_w.torch[0].detach().cpu().numpy().astype(float)
    Rc=quat_to_rot(cube.data.root_quat_w.torch[0].detach().cpu().numpy())
    signs=np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])
    corners=cubepose[None,:]+(signs*.02)@Rc.T
    cube_width=float(np.ptp(corners@sep))
    cube_local=(cubepose-tcp)@Rt
    cube_corners_tool=(corners-tcp)@Rt
    sep_tool=sep@Rt
    print('LEXIUM_STAGE3C33_GEOMETRY_STARTED=True',flush=True)
    print('STAGE33_FINGER_MOTION_COMMANDED=False',flush=True)
    print(f'STAGE33_FIXED_AXIS_COLLIDER_GAP_M={gap:.9f} CUBE_WIDTH_ALONG_FIXED_AXIS_M={cube_width:.9f}',flush=True)
    print(f'STAGE33_SEPARATION_AXIS_WORLD={sep.tolist()} TOOL={sep_tool.tolist()}',flush=True)
    print(f'STAGE33_TCP_WORLD={tcp.tolist()} CUBE_CENTER_TOOL={cube_local.tolist()}',flush=True)
    align=np.abs(Rt[2,:]); print(f'STAGE33_TOOL_AXIS_VERTICAL_ABS_ALIGNMENT_XYZ={align.tolist()} (NOT_VERIFIED_GRASP_APPROACH_AXIS)',flush=True)
    local_report={}
    for name in names:
        vertices=by_name[name]
        in_tool=(vertices-tcp)@Rt
        local_report[name]={
           'sampled_count':int(len(vertices)),
           'min_xyz_tool_m':in_tool.min(axis=0).tolist(),
           'max_xyz_tool_m':in_tool.max(axis=0).tolist(),
           'sampled_centroid_tool_m':in_tool.mean(axis=0).tolist(),
           'sampled_centroid_world_m':vertices.mean(axis=0).tolist(),
           'min_z_world_m':float(vertices[:,2].min()),
        }
        print(f'STAGE33_FINGER_TOOL_AABB name={name} min={local_report[name]["min_xyz_tool_m"]} '
              f'max={local_report[name]["max_xyz_tool_m"]}',flush=True)
    pairs=[]
    for i in (0,1):
        for j in (2,3):
            a=by_name[names[i]]; b=by_name[names[j]]
            closest=_nearest_vertex_pair_distance_m(a,b)
            pp={'finger_a':names[i],'finger_b':names[j],
                'min_sampled_vertex_vertex_distance_m':closest,
                'pair_projected_envelope_gap_m':_sampled_gap_for_groups(a,b,sep)}
            pairs.append(pp)
            print(f'STAGE33_FINGER_PAIR a={names[i]} b={names[j]} '
                  f'vertex_vertex_min_m={closest:.6f} projected_gap_m={pp["pair_projected_envelope_gap_m"]:.6f}',flush=True)
    # Cross-sections of sampled vertices. The slices are for diagnosis only;
    # polygonal mesh surfaces can cross slices even when no vertex lies on one.
    slice_sets={
        'world_z':(group_a[:,2],group_b[:,2]),
        'tool_x':((group_a-tcp)@Rt[:,0],(group_b-tcp)@Rt[:,0]),
        'tool_y':((group_a-tcp)@Rt[:,1],(group_b-tcp)@Rt[:,1]),
        'tool_z':((group_a-tcp)@Rt[:,2],(group_b-tcp)@Rt[:,2]),
    }
    sections={}
    width=0.010  # 10 mm-wide bins; this is NOT fingertip mesh clearance.
    for key,(aa,bb) in slice_sets.items():
        left=max(float(aa.min()),float(bb.min()))
        right=min(float(aa.max()),float(bb.max()))
        entries=[]
        if right>left:
            centers=np.arange(left+width/2,right,width)
            for center in centers:
                mask_a=np.abs(aa-center)<width/2
                mask_b=np.abs(bb-center)<width/2
                if np.sum(mask_a)<15 or np.sum(mask_b)<15: continue
                sl=_sampled_gap_for_groups(group_a[mask_a],group_b[mask_b],sep)
                entry={'center_m':float(center),'sampled_a':int(np.sum(mask_a)),
                       'sampled_b':int(np.sum(mask_b)), 'projected_gap_m':sl}
                entries.append(entry)
        sections[key]=entries
        if entries:
            vals=[x['projected_gap_m'] for x in entries]
            print(f'STAGE33_SLICE_SUMMARY axis={key} bins={len(entries)} '
                  f'min_gap_m={min(vals):.6f} max_gap_m={max(vals):.6f}',flush=True)
        else:
            print(f'STAGE33_SLICE_SUMMARY axis={key} bins=0 INSUFFICIENT_VERTEX_COVERAGE',flush=True)
    # This report is explicitly *not* a check that a 40 mm cube fits.
    # Self-collisions disabled, and sampled vertices do not contain a swept-volume or mesh-surface test.
    report={
       'stage':'3C33','mode':'static_audit','source':'runtime_usd_collision_vertices_at_physx_link_poses',
       'no_finger_motion':True,'self_collisions_enabled':False,
       'tcp_world_xyz_m':tcp.tolist(),'cube_center_world_xyz_m':cubepose.tolist(),
       'cube_center_in_tool_xyz_m':cube_local.tolist(),
       'cube_corners_in_tool_m':cube_corners_tool.tolist(),
       'tool_axis_vertical_abs_alignment':align.tolist(),
       'separation_axis_world':sep.tolist(), 'separation_axis_tool':sep_tool.tolist(),
       'fixed_axis_sampled_envelope_gap_m':gap,
       'cube_projection_width_along_separation_axis_m':cube_width,
       'finger_local_tool_aabb':local_report,'opposing_pairs':pairs,
       'sampled_vertex_cross_sections_10mm':sections,
       'grasp_40mm_validated':False,'fingertip_tcp_calibrated':False,
       'disclaimer':'Sampled mesh vertices, axis projections and slice gaps are geometric proxies, NOT actual minimum mesh-surface distances or a proof of collision-free opening or grasp. Joint self-collisions are disabled.',
    }
    outfile=args_cli.stage33_report
    outfile.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(f'STAGE33_REPORT_JSON={outfile}',flush=True)
    print('LEXIUM_STAGE3C33_STATIC_GEOMETRY_AUDIT_COMPLETE=True',flush=True)
    print('GRASP_VALIDATED=False FINGERTIP_TCP_CALIBRATED=False OPENING_40MM_VALIDATED=False',flush=True)



# Stage 3C.34: virtual FK-only sweep at current physical hold.
# NO set_joint_position_target, physics stepping, arm motion or source mutation.
# This is an approximated rigid-hinge geometry forecast, not a trajectory.

def _rotate_about_pivot(points, pivot, axis, radians):
    a=np.asarray(axis,dtype=float)
    a=a / np.linalg.norm(a)
    c=math.cos(float(radians)); ss=math.sin(float(radians))
    # Rodrigues' rotation formula in the WORLD frame.
    v=points-pivot[None,:]
    return pivot[None,:]+(v*c + np.cross(a[None,:],v)*ss + (v@a)[:,None]*a[None,:]*(1.0-c))


def run_stage34_virtual_sweep(robot, cube, cached, joints):
    if cached is None or cube is None:
        abort('STAGE34_REQUIRES_FULL_SCENE_AND_CACHED_COLLISION_GEOMETRY')
    if not args_cli.stage32b_report.is_file():
        abort('STAGE34_MISSING_PREVIOUS_STAGE32B_REPORT')
    prev=json.loads(args_cli.stage32b_report.read_text(encoding='utf8'))
    if (prev.get('stage')!='3C32B' or prev.get('probe_complete') is not True
            or prev.get('actual_passes') != 7 or prev.get('skipped_limit') != 1):
        abort('STAGE34_STAGE32B_PROBE_NOT_VERIFIED')

    body_names=list(robot.data.body_names)
    jnames=list(robot.data.joint_names)
    finger_joints=list(STAGE32_FINGER_NAMES)
    finger_bodies=['gripper_finger_1_1','gripper_finger_2_1',
                   'gripper_finger_3_1','gripper_finger_4_1']
    if any(j not in jnames for j in finger_joints):
        abort('STAGE34_MISSING_FINGER_JOINTS')
    for n,jn in zip(finger_bodies,finger_joints):
        if joints[jn]['child']!=n or joints[jn]['parent']!='gripper_base_1':
            abort('STAGE34_URDF_JOINT_PARENT_CHILD_MISMATCH_'+jn)
    poses=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    bpose=poses[body_names.index('gripper_base_1')]
    Rb=quat_to_rot(bpose[3:7]);pb=bpose[:3]
    fid=[jnames.index(j) for j in finger_joints]
    q=robot.data.joint_pos.torch[0,fid].detach().cpu().numpy()
    limits=robot.data.joint_pos_limits.torch[0,fid].detach().cpu().numpy()
    sep=np.asarray(cached['group_axis'],dtype=float)
    sep/=np.linalg.norm(sep)
    world={}
    hinges={}
    for name, jn in zip(finger_bodies, finger_joints):
        local=cached['local_points'][name]
        pose=poses[body_names.index(name)]
        world[name]=pose[:3][None,:]+local@quat_to_rot(pose[3:7]).T
        info=joints[jn]
        origin=pb+Rb@np.asarray(info['xyz'],dtype=float)
        axis=Rb@rpy_to_R(info['rpy'])@np.asarray(info['axis'],dtype=float)
        axis/=np.linalg.norm(axis)
        hinges[jn]={'pivot_world_xyz_m':origin.tolist(),'axis_world_xyz':axis.tolist()}

    def getgap(points):
        a=np.concatenate((points[finger_bodies[0]],points[finger_bodies[1]]))
        b=np.concatenate((points[finger_bodies[2]],points[finger_bodies[3]]))
        return float(np.min(b@sep)-np.max(a@sep))

    def virtual_geometry(deltas):
        transformed={}
        for i,(name,jn) in enumerate(zip(finger_bodies,finger_joints)):
            d=float(deltas[i])
            origin=np.asarray(hinges[jn]['pivot_world_xyz_m'])
            axis=np.asarray(hinges[jn]['axis_world_xyz'])
            transformed[name]=_rotate_about_pivot(world[name],origin,axis,d)
        gap=getgap(transformed)
        lower_z=min(float(a[:,2].min()) for a in transformed.values())
        # This also checks SAME-side pairs in the vertex metric, not mesh collision.
        pair_gaps={}
        for ia,ib in ((0,2),(0,3),(1,2),(1,3),(0,1),(2,3)):
            an,bn=finger_bodies[ia],finger_bodies[ib]
            # Fast conservative planar AABB overlap is NOT a collision check.
            aa=transformed[an];bb=transformed[bn]
            aabb_gap=np.maximum(np.maximum(aa.min(axis=0)-bb.max(axis=0),
                                           bb.min(axis=0)-aa.max(axis=0)),0.)
            pair_gaps[an+'__'+bn]=float(np.linalg.norm(aabb_gap))
        return gap, lower_z, pair_gaps

    baseline=getgap(world)
    previous_start=float(prev['start_geometry_proxy']['fixed_axis_projected_gap_m'])
    if abs(baseline-previous_start)>0.001:
        abort(f'STAGE34_BASELINE_AXIS_GAP_CHANGED_{baseline:.7f}_VS_{previous_start:.7f}')
    if baseline <0.015:
        abort('STAGE34_BASELINE_TOO_SMALL')
    print('LEXIUM_STAGE3C34_VIRTUAL_SWEEP_STARTED=True',flush=True)
    print('STAGE34_PHYSICAL_FINGER_MOTION_COMMANDED=False',flush=True)
    print('STAGE34_ARM_DESCENT_COMMANDED=False',flush=True)
    print('STAGE34_MODEL=URDF_PARENT_HINGE_RIGID_LINK_ROTATIONS_SAMPLED_USD_COLLISION_VERTICES',flush=True)
    print(f'STAGE34_BASELINE_FIXED_AXIS_GAP_M={baseline:.9f}',flush=True)
    print(f'STAGE34_MEASURED_Q_RAD={q.tolist()}',flush=True)
    single=[]
    # Scan single joint, both directions, at modest virtual increments.
    # This changes only NUMPY arrays; it is not a movement recommendation.
    for i, jn in enumerate(finger_joints):
        for d in (-0.004,0.004,-0.02,0.02,-0.05,0.05,-0.1,0.1):
            target=float(q[i]+d)
            within=bool(limits[i,0]+0.002<target<limits[i,1]-0.002)
            if not within:
                print(f'STAGE34_VIRTUAL_SINGLE joint={jn} delta_rad={d:+.3f} LIMIT_SKIP=True',flush=True)
                continue
            delta=np.zeros(4);delta[i]=d
            gp,loz,pairs=virtual_geometry(delta)
            row={'joint':jn,'delta_rad':d,'target_rad':target,
                 'predicted_projected_gap_m':gp,
                 'predicted_gap_change_m':gp-baseline,
                 'predicted_lowest_finger_z_world_m':loz,
                 'pair_aabb_separations_m':pairs}
            single.append(row)
            print(f'STAGE34_VIRTUAL_SINGLE joint={jn} delta_rad={d:+.3f} '
                  f'projected_gap_change_mm={(gp-baseline)*1000:+.4f} '
                  f'gap_mm={gp*1000:.4f} LOWEST_Z_M={loz:.5f} '
                  f'VIRTUAL_ONLY=True',flush=True)

    # Compare predicted +/−0.004 changes to the previous physical probe.
    # Because its measurements include settling/drift, this is NOT model certification.
    comparisons=[]
    for probe in prev.get('probes',[]):
        if probe.get('status')!='PASS':continue
        jn=probe['joint']; ix=finger_joints.index(jn)
        delta=np.zeros(4)
        delta[ix]=float(probe['actual_joint_delta_rad'])
        gp,_,_=virtual_geometry(delta)
        obs=float(probe['fixed_axis_gap_change_m'])
        comparisons.append({'joint':jn,'sign':probe['sign'],
                            'observed_gap_delta_m':obs,
                            'predicted_gap_delta_m':gp-baseline,
                            'difference_m':(gp-baseline)-obs})
    maxdiff=max(abs(v['difference_m']) for v in comparisons)
    # A permissive guard rejects only a major disagreement (>=0.2 mm).
    # Underlying actual data shifts of ~0.005mm cannot certify the virtual FK.
    rough_consistent=bool(maxdiff<0.0002)
    print(f'STAGE34_PREVIOUS_PROBE_MODEL_SANITY_CHECK={rough_consistent} '
          f'MAX_ABS_GAP_DIFFERENCE_MM={maxdiff*1000:.5f} '
          'NOT_VALIDATION_OF_SWEPT_COLLISION',flush=True)
    # Grid forecast with ±0.05rad and 0 on all joints; up to 81 static states.
    # Only grid points that respect all 4 joint position limits are considered.
    import itertools
    combinations=[]
    for dtuple in itertools.product((-0.05,0.0,0.05), repeat=4):
        dvec=np.asarray(dtuple,dtype=float)
        target=q+dvec
        if np.any(target<=limits[:,0]+0.002) or np.any(target>=limits[:,1]-0.002):
            continue
        gp,lowz,pg=virtual_geometry(dvec)
        combinations.append({'delta_rad':dvec.tolist(),
                             'projected_gap_m':gp,
                             'gap_change_m':gp-baseline,
                             'lowest_finger_world_z_m':lowz,
                             'pair_aabb_separations_m':pg})
    if not combinations:abort('STAGE34_NO_STATIC_COMBINATIONS_IN_LIMITS')
    ranked=sorted(combinations,key=lambda a:a['projected_gap_m'],reverse=True)
    print('STAGE34_BEST_VIRTUAL_COMBINATIONS='+json.dumps(
        [{'delta_rad':r['delta_rad'],'gap_m':r['projected_gap_m'],
          'gap_change_mm':round(r['gap_change_m']*1000,5)}
         for r in ranked[:5]]),flush=True)
    previous_report=json.loads(args_cli.stage33_report.read_text(encoding='utf8'))
    if previous_report.get('stage')!='3C33':abort('STAGE34_STAGE33_REPORT_WRONG')
    cube_width=float(previous_report['cube_projection_width_along_separation_axis_m'])
    report={'stage':'3C34','mode':'static_virtual_fk_only',
            'no_finger_motion':True,'no_arm_descent':True,
            'self_collision_enabled':False,'grasp_validated':False,
            'opening_40mm_validated':False,
            'sampled_baseline_projected_gap_m':baseline,
            'cube_full_projection_width_current_orientation_m':cube_width,
            'virtual_hinge_model':hinges,
            'actual_finger_joint_q_rad':q.tolist(),
            'joint_limits_rad':limits.tolist(),
            'single_joint_virtual_samples':single,
            'previous_probe_comparison':comparisons,
            'model_roughly_consistent_with_prior_tiny_probes':rough_consistent,
            'virtual_combination_grid_count':len(combinations),
            'best_virtual_combinations_up_to_5':ranked[:5],
            'disclaimer': 'VIRTUAL RIGID-HINGE KINEMATIC PROXIES ONLY. Does not move fingers, validate actual geometry, continuous mesh surface/swept collisions, finger-base collisions, TCP calibration, object fit or grasp. Same-side finger AABB separations are not exact collision checks. No permission for actual simultaneous finger motion.'}
    args_cli.stage34_report.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(f'STAGE34_CUBE_FULL_PROJECTION_MM={cube_width*1000:.4f} '
          f'CURRENT_GAP_MM={baseline*1000:.4f}',flush=True)
    print(f'STAGE34_VIRTUAL_COMBINATIONS_CHECKED={len(combinations)}',flush=True)
    print(f'STAGE34_REPORT_JSON={args_cli.stage34_report}',flush=True)
    print('LEXIUM_STAGE3C34_STATIC_VIRTUAL_AUDIT_COMPLETE=True',flush=True)
    print('GRASP_VALIDATED=False OPENING_40MM_VALIDATED=False '
          'FINGER_MOVEMENT_COMMANDED=False',flush=True)

def main():
    if args_cli.scene_variant != 'full':
        raise ValueError('STAGE31_REQUIRES_FULL_SCENE')
    if not (0.0 <= args_cli.table_center_x <= 2.0):
        raise ValueError('table center outside diagnostic limits')
    if not (args_cli.table_center_x-0.39 <= args_cli.cube_x <= args_cli.table_center_x+0.39
            and -0.54 <= args_cli.cube_y <= 0.54):
        raise ValueError('cube not safely above table with >= 4 cm margin')
    if not 0.1 <= args_cli.robot_contact_threshold_n <= 5.0:
        raise ValueError('invalid contact threshold')
    if args_cli.num_envs!=1 or not (90<=args_cli.warmup_steps<=360) or not (600<=args_cli.motion_steps<=1800) or not (180<=args_cli.hold_steps<=600):
        raise ValueError('Unexpected simulation lengths or environment count')
    qtarget,goal=load_verified_candidate()
    qhome=np.asarray(Q_HOME,dtype=float)
    links,joints,children,root=parse_urdf(args_cli.urdf)
    sim=SimulationContext(sim_utils.SimulationCfg(device=args_cli.device,dt=1./60.,gravity=(0.,0.,-9.81)))
    scene_cls=Stage3C30TableCfg if args_cli.scene_variant=='table' else Stage3C30FullCfg
    scene=InteractiveScene(scene_cls(num_envs=1,env_spacing=2.5))
    sim.reset();scene.reset();scene.update(sim.get_physics_dt())
    robot=scene['robot']
    cube=scene['cube'] if args_cli.scene_variant=='full' else None
    sensor=scene['robot_contact']
    cube_sensor=scene['cube_contact'] if cube is not None else None
    if not robot.is_fixed_base:abort('ROOT_NOT_FIXED')
    jointnames=list(robot.data.joint_names)
    armids=[jointnames.index(n) for n in LEXIUM_ARM_JOINT_NAMES]
    body_names=list(robot.data.body_names)
    baseid=body_names.index(root)
    toolid=body_names.index('gripper_base_1')
    import omni.usd
    from pxr import UsdPhysics
    stage=omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath('/World/envs/env_0/Table').IsValid():
        abort('MISSING_TABLE')
    if stage.GetPrimAtPath('/World/envs/env_0/Cube').IsValid() != (cube is not None):
        abort('CUBE_SCENE_MISMATCH')
    from pxr import PhysxSchema
    contact_reporting_bodies = [p for p in stage.Traverse()
                                if p.HasAPI(PhysxSchema.PhysxContactReportAPI)]
    robot_contact_prim_count = sum(str(p.GetPath()).startswith('/World/envs/env_0/Robot/')
                                   for p in contact_reporting_bodies)
    print(f'CONTACT_REPORTER_ROBOT_BODIES={robot_contact_prim_count}',flush=True)
    if robot_contact_prim_count < 10:
        abort('CONTACT_REPORTER_MISSING_ON_ROBOT_BODIES')
    # Validate sensor binding; a missing sensor is a failure, not zero contacts.
    sensor_tensor=sensor.data.net_normal_forces_w.torch
    print(f'ROBOT_CONTACT_SENSOR_SHAPE={list(sensor_tensor.shape)}',flush=True)
    if sensor_tensor.ndim!=3 or sensor_tensor.shape[0]!=1 or sensor_tensor.shape[1]<10 or sensor_tensor.shape[2]!=3:
        abort(f'ROBOT_CONTACT_SENSOR_INCOMPLETE_{list(sensor_tensor.shape)}')
    if cube is not None:
        cforce=cube_sensor.data.net_normal_forces_w.torch
        if cforce.ndim!=3 or cforce.shape[1]!=1 or cforce.shape[2]!=3:
            abort(f'CUBE_CONTACT_SENSOR_INCOMPLETE_{list(cforce.shape)}')
        cube.write_root_pose_to_sim_index(root_pose=cube.data.default_root_pose.torch.clone())
        cube.write_root_velocity_to_sim_index(root_velocity=cube.data.default_root_vel.torch.clone())
    flags=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            a=prim.GetAttribute('physxArticulation:enabledSelfCollisions')
            flags.append(a.Get() if a.IsValid() else None)
    if flags != [False]:abort(f'SELF_COLLISION_CONFIG_UNEXPECTED_{flags}')
    k=robot.data.joint_stiffness.torch[0,armids].detach().cpu().numpy()
    eff_limits=robot.data.joint_effort_limits.torch[0,armids].detach().cpu().numpy()
    if not np.allclose(k,[400,800,800,400,400,400],atol=.1):
        abort(f'UNEXPECTED_KP_{k.tolist()}')
    if not np.allclose(eff_limits,[100]*6,atol=.1):
        abort(f'UNEXPECTED_EFFORT_LIMITS_{eff_limits.tolist()}')
    joint_limits=robot.data.joint_pos_limits.torch[0,armids].detach().cpu().numpy()
    base_target=robot.data.default_joint_pos.torch.clone()
    for i,idj in enumerate(armids):base_target[0,idj]=float(qhome[i])
    robot.write_joint_state_to_sim_index(position=base_target,velocity=torch.zeros_like(base_target))
    scene.update(0.0)
    rootstart=robot.data.root_link_pose_w.torch[0,:3].detach().clone()
    basepose=robot.data.body_link_pose_w.torch[0,baseid].detach().cpu().numpy()
    Rroot=quat_to_rot(basepose[3:7])
    g_root=Rroot.T@np.asarray([0.,0.,-9.81])
    if not np.allclose(g_root,[0,-9.81,0],atol=.01):abort(f'ROOT_GRAVITY_FRAME_{g_root.tolist()}')
    tcp_offset=np.asarray(LEXIUM_TCP_OFFSET,dtype=float)
    if not np.allclose(tcp_offset,[.002,.1,.148],atol=1e-6):abort(f'TCP_OFFSET_CHANGED_{tcp_offset.tolist()}')
    desired_link_R,desired_link_xyz=fk_gripper_base(links,joints,children,root,qtarget)
    desired_tool_R=Rroot@desired_link_R
    goal_fk=basepose[:3]+Rroot@desired_link_xyz+desired_tool_R@tcp_offset
    if np.linalg.norm(goal_fk-goal)>.001:abort(f'OFFLINE_FK_TARGET_MISMATCH_{goal_fk.tolist()}')
    # Critical: independently recalculate static gravity limits using live root frame.
    max_preflight=np.zeros(6)
    for a in np.linspace(0,1,301):
        tq=compute_root_frame_gravity(links,joints,children,root,qhome+a*(qtarget-qhome),g_root)
        max_preflight=np.maximum(max_preflight,np.abs(tq))
    print(f'RUNTIME_OFFLINE_PEAK_GRAVITY_ARM_NM={max_preflight.tolist()}',flush=True)
    if np.max(max_preflight)>=85.:abort('RUNTIME_OFFLINE_GRAVITY_GATE_85NM_FAILED')

    dt=sim.get_physics_dt()
    warmup=args_cli.warmup_steps
    motion=args_cli.motion_steps
    hold=args_cli.hold_steps
    total=warmup+motion+hold
    limit_torque_frames=0
    limit_tracking_frames=0
    max_effort_seen=np.zeros(6)
    max_g_seen=np.zeros(6)
    max_tracking=0.
    sample_indices={1,warmup,warmup+1,warmup+motion//4,warmup+motion//2,
                   warmup+3*motion//4,warmup+motion,warmup+motion+30,warmup+motion+120,total}
    last_samples=[]
    robot_peak_contact_n=0.0
    robot_contact_frames=0
    cube_contact_rest_max_n=0.0
    cube_start_xy=np.asarray([args_cli.cube_x,args_cli.cube_y],dtype=float)
    print('LEXIUM_STAGE3C30_MOTION_STARTED=True',flush=True)
    print(f'SCENE_VARIANT={args_cli.scene_variant.upper()} TABLE_CENTER_X_M={args_cli.table_center_x:.3f} CUBE_XY_M={[args_cli.cube_x,args_cli.cube_y]}',flush=True)
    print(f'CONTACT_POLICY_ROBOT_MAX_NORMAL_N={args_cli.robot_contact_threshold_n:.3f} CUBE_TABLE_POSITIVE_CONTROL_REQUIRED={cube is not None}',flush=True)
    print('SCENE=TABLE_OR_FULL FIXED_BASE=True SELF_COLLISIONS_OFF=True GRAVITY_Z=-9.81',flush=True)
    print('INITIALIZATION=HOME_ONLY CANDIDATE_NOT_TELEPORTED=True',flush=True)
    print(f'Q_HOME_RAD={qhome.tolist()} Q_TARGET_CANDIDATE1_RAD={qtarget.tolist()}',flush=True)
    print(f'CANDIDATE_GOAL_TCP_XYZ={goal.tolist()} MOTION_STEPS={motion} WARMUP_STEPS={warmup} HOLD_STEPS={hold}',flush=True)
    print('CONTROL=QUINTIC_JOINTSPACE_WITH_URDF_GRAVITY_POSITION_BIAS',flush=True)
    print('SAFETY_THRESHOLDS=max_12_consecutive_frames_effort_over_95Nm;max_20_consecutive_frames_tracking_over_0.18rad;gravity_over_90Nm',flush=True)
    for step in range(1,total+1):
        if not simulation_app.is_running():abort('ISAAC_SIM_STOPPED')
        if step<=warmup:
            phase='WARMUP'; a=0.; v=0.
        elif step<=warmup+motion:
            phase='MOVE'; v=(step-warmup)/motion
            a=10*v**3-15*v**4+6*v**5
        else:
            phase='HOLD'; a=1.;v=1.
        qref=qhome+a*(qtarget-qhome)
        tg=compute_root_frame_gravity(links,joints,children,root,qref,g_root)
        bias=tg/k
        cmd=qref+bias
        if not np.isfinite(cmd).all():abort(f'NONFINITE_COMMAND_step_{step}')
        if np.max(np.abs(bias))>0.25:abort(f'GRAVITY_BIAS_OUT_OF_RANGE_step_{step}')
        if np.any(cmd<joint_limits[:,0]+1e-4) or np.any(cmd>joint_limits[:,1]-1e-4):
            abort(f'BIASED_TARGET_JOINT_LIMIT_step_{step}')
        if np.max(np.abs(tg))>85.:abort(f'STATIC_TORQUE_REFERENCE_ABOVE_GATE_step_{step}')
        for i,jid in enumerate(armids):base_target[0,jid]=float(cmd[i])
        robot.actuators.target_command.set_position_index(value=base_target,full_data=True)
        scene.write_data_to_sim()
        if step==1:print('BEFORE_FIRST_PHYSICS_STEP=True',flush=True)
        sim.step()
        if step==1:print('AFTER_FIRST_PHYSICS_STEP=True',flush=True)
        scene.update(dt)
        rforces=sensor.data.net_normal_forces_w.torch[0]
        if not bool(torch.isfinite(rforces).all().item()):
            abort(f'NONFINITE_ROBOT_CONTACT_SENSOR_step_{step}')
        rcontact=float(torch.linalg.vector_norm(rforces,dim=-1).max().item())
        robot_peak_contact_n=max(robot_peak_contact_n,rcontact)
        if rcontact>0.1: robot_contact_frames+=1
        if rcontact >= args_cli.robot_contact_threshold_n:
            print(f'ROBOT_CONTACT_EVENT step={step} force_n={rcontact:.6f}',flush=True)
            abort(f'ROBOT_EXTERNAL_CONTACT_STEP_{step}_N_{rcontact:.3f}')
        cforce=0.0
        if cube is not None:
            cforces=cube_sensor.data.net_normal_forces_w.torch[0]
            if not bool(torch.isfinite(cforces).all().item()):
                abort(f'NONFINITE_CUBE_CONTACT_SENSOR_step_{step}')
            cforce=float(torch.linalg.vector_norm(cforces,dim=-1).max().item())
            cxyz=cube.data.root_pos_w.torch[0].detach().cpu().numpy()
            if step>args_cli.warmup_steps:
                cube_contact_rest_max_n=max(cube_contact_rest_max_n,cforce)
                if abs(cxyz[2]-0.020)>0.012 or np.linalg.norm(cxyz[:2]-cube_start_xy)>0.01:
                    abort(f'CUBE_DISTURBED_step_{step}_XYZ_{cxyz.tolist()}')
            if step == args_cli.warmup_steps:
                print(f'CUBE_SETTLE_POSITION_XYZ_M={cxyz.tolist()} CUBE_NORMAL_CONTACT_N={cforce:.6f}',flush=True)
                if not (0.2 <= cforce <= 10.0 and abs(cxyz[2]-0.020)<0.012):
                    abort('CUBE_CONTACT_SENSOR_POSITIVE_CONTROL_FAILED')
        pos=robot.data.joint_pos.torch[0,armids].detach().cpu().numpy()
        vel=robot.data.joint_vel.torch[0,armids].detach().cpu().numpy()
        efforts=robot.actuators.applied_effort.torch[0,armids].detach().cpu().numpy()
        g_physx=robot.data.gravity_compensation_forces.torch[0,armids].detach().cpu().numpy()
        if not np.isfinite(pos).all() or not np.isfinite(vel).all() or not np.isfinite(efforts).all():
            abort(f'NONFINITE_PHYSICS_step_{step}')
        max_effort_seen=np.maximum(max_effort_seen,np.abs(efforts))
        max_g_seen=np.maximum(max_g_seen,np.abs(g_physx))
        track=float(np.max(np.abs(pos-qref)))
        max_tracking=max(max_tracking,track)
        # Brief transients are logged, sustained poor tracking or saturation aborts.
        limit_torque_frames=limit_torque_frames+1 if np.max(np.abs(efforts))>=95. else 0
        limit_tracking_frames=limit_tracking_frames+1 if track>=.18 else 0
        if limit_torque_frames>=12:abort(f'SUSTAINED_ACTUATOR_NEAR_100NM_step_{step}')
        if limit_tracking_frames>=20:abort(f'SUSTAINED_JOINT_TRACKING_ERROR_step_{step}')
        if np.max(np.abs(g_physx))>90.:abort(f'ACTUAL_POSE_GRAVITY_OVER_90NM_step_{step}')
        disp=float(torch.linalg.vector_norm(robot.data.root_link_pose_w.torch[0,:3]-rootstart).item())
        if disp>0.001:abort(f'FIXED_BASE_MOVED_step_{step}')
        toolpose=robot.data.body_link_pose_w.torch[0,toolid].detach().cpu().numpy()
        Rtool=quat_to_rot(toolpose[3:7])
        tcp=toolpose[:3]+Rtool@tcp_offset
        err=float(np.linalg.norm(tcp-goal))
        ori_err=angular_error_deg(desired_tool_R,Rtool)
        speed=float(np.max(np.abs(vel)))
        if phase=='HOLD' and step>total-30:
            last_samples.append((speed,err,ori_err,float(efforts[1])))
        if step in sample_indices:
            print(f'MOTION_SAMPLE step={step} phase={phase} progress={v:.4f} '
                  f'joint2_gravity_physx_nm={g_physx[1]:+.3f} joint2_applied_nm={efforts[1]:+.3f} '
                  f'max_arm_applied_nm={np.max(np.abs(efforts)):.3f} max_tracking_rad={track:.5f} '
                  f'max_arm_speed_rad_s={speed:.5f} tcp_goal_error_m={err:.5f} '
                  f'orientation_goal_error_deg={ori_err:.4f} robot_contact_n={rcontact:.5f} cube_contact_n={cforce:.5f}',flush=True)
    if len(last_samples)<30:abort('MISSING_LAST30_HOLD_WINDOW')
    qfinal_act=robot.data.joint_pos.torch[0,armids].detach().cpu().numpy()
    efffinal=robot.actuators.applied_effort.torch[0,armids].detach().cpu().numpy()
    toolpose=robot.data.body_link_pose_w.torch[0,toolid].detach().cpu().numpy()
    toolR=quat_to_rot(toolpose[3:7])
    tcp=toolpose[:3]+toolR@tcp_offset
    ferr=float(np.linalg.norm(tcp-goal));oerr=angular_error_deg(desired_tool_R,toolR)
    maxholdspeed=max(row[0] for row in last_samples)
    stable=bool(maxholdspeed<.1 and ferr<.005 and oerr<1.0 and np.max(max_effort_seen)<95.0)
    print(f'FINAL_ARM_Q_RAD={qfinal_act.tolist()}',flush=True)
    print(f'FINAL_ARM_APPLIED_EFFORT_NM={efffinal.tolist()}',flush=True)
    print(f'FINAL_TCP_XYZ_M={tcp.tolist()} FINAL_TCP_ERROR_M={ferr:.7f}',flush=True)
    print(f'FINAL_ORIENTATION_ERROR_DEG={oerr:.6f}',flush=True)
    print(f'LAST30_MAX_ARM_SPEED_RAD_S={maxholdspeed:.7f}',flush=True)
    print(f'PEAK_APPLIED_EFFORT_ARM_NM={max_effort_seen.tolist()}',flush=True)
    print(f'PEAK_PHYSX_GRAVITY_ARM_NM={max_g_seen.tolist()}',flush=True)
    print(f'PEAK_TRACKING_ERROR_RAD={max_tracking:.6f}',flush=True)
    print(f'JOINT2_PEAK_APPLIED_NM={max_effort_seen[1]:.6f}',flush=True)
    print(f'JOINT2_PEAK_STATIC_GRAVITY_PHYSX_NM={max_g_seen[1]:.6f}',flush=True)
    print(f'FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE={stable}',flush=True)
    print(f'ROBOT_PEAK_CONTACT_FORCE_N={robot_peak_contact_n:.6f}',flush=True)
    print(f'ROBOT_FRAMES_ABOVE_0P1_N={robot_contact_frames}',flush=True)
    print(f'CUBE_REST_CONTACT_MAX_N={cube_contact_rest_max_n:.6f}',flush=True)
    print(f'EXTERNAL_CONTACT_SCREEN_PASSED={robot_peak_contact_n<args_cli.robot_contact_threshold_n and (cube is None or cube_contact_rest_max_n>0.2)}',flush=True)
    # Before claiming stage 3C31 complete: verify the contact screen passed,
    # then inspect collision meshes at exactly the final, physically held pose.
    motion_contact_ok = bool(robot_peak_contact_n<args_cli.robot_contact_threshold_n
                         and (cube is None or cube_contact_rest_max_n>0.2))
    if args_cli.scene_variant=='full' and stable and motion_contact_ok:
        stage32_cache = report_stage31_gripper_geometry(
            robot, cube, stage, tcp.astype(float), tcp_offset, toolR,
            contact_passed=motion_contact_ok, stable=stable)
    print('IMPORTANT=PHYSX_CONTACT_SCREEN_ONLY_SELF_COLLISIONS_OFF_NOT_SWEPT_COLLISION_CERTIFIED',flush=True)
    print('LEXIUM_STAGE3C30_MOTION_COMPLETE=True',flush=True)
    if not stable or robot_peak_contact_n>=args_cli.robot_contact_threshold_n or (cube is not None and cube_contact_rest_max_n<=0.2):
        raise RuntimeError('Stage 3C30 contact and/or motion criteria did not pass')
    run_stage33_static_geometry(robot, cube, stage32_cache, tcp_offset, toolid, goal)
    run_stage34_virtual_sweep(robot, cube, stage32_cache, joints)


if __name__=='__main__':
    try:main()
    finally:simulation_app.close()
