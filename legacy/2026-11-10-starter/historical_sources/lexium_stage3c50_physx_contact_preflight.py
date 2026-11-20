#!/usr/bin/env python3
"""Stage 3C50: Isaac Lab full-scene HOME contact-sensor preflight.

Proves the table/cube and contact-reporting chain are working in the same
simulation scene used by the previously successful Stage3C37 replay.
No PREGRASP arm motion, no finger motion, no asset changes, no self-collision
policy changes. The robot is explicitly initialized to its default HOME joint
state and held there with the previously used URDF gravity position bias.

This script does NOT validate PhysX self-collisions (disabled in this asset),
actual cooked shape separation, or a continuous/safe approach trajectory.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', type=Path, required=True)
parser.add_argument('--stage49', type=Path, required=True)
parser.add_argument('--out', type=Path, default=Path('/tmp/lexium_stage3c50_physx_contact_preflight.json'))
parser.add_argument('--warmup-steps', type=int, default=150)
parser.add_argument('--robot-contact-threshold-n', type=float, default=0.5)
parser.add_argument('--num-envs', type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.num_envs != 1 or not (120 <= args.warmup_steps <= 360):
    parser.error('Stage50 requires 1 environment and 120..360 HOME-only simulation steps')
if not (0.1 <= args.robot_contact_threshold_n <= 2.0):
    parser.error('Robot-contact threshold must be in [0.1,2.0] N')
launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np
import torch
import omni.usd
import isaaclab.sim as sim_utils
from pxr import UsdPhysics, PhysxSchema
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
from lexium_pick_place.robot_cfg import LEXIUM_COBOT_CFG, LEXIUM_ARM_JOINT_NAMES, LEXIUM_USD_PATH
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import parse_urdf, Q_HOME
from lexium_pick_place.lexium_stage3c25_corrected_gravity_route_audit import compute_root_frame_gravity

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
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.42)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.62, 0.0, -0.20)),
    )


def make_cube():
    return RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/Cube',
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04),
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
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.75, 0.25)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.30, -0.30, 0.12)),
    )


@configclass
class Stage50FullScene(InteractiveSceneCfg):
    table: AssetBaseCfg = make_table()
    robot: ArticulationCfg = CONTACT_ROBOT_CFG
    cube: RigidObjectCfg = make_cube()
    robot_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*', update_period=0.0)
    cube_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Cube', update_period=0.0)
    light: AssetBaseCfg = AssetBaseCfg(
        prim_path='/World/Light', spawn=sim_utils.DomeLightCfg(intensity=2500.0))


def abort(reason):
    raise RuntimeError('STAGE50_SAFETY_ABORT=' + reason)


def quat_to_rot(q):
    x, y, z, w = map(float, q)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-8:
        abort('ROOT_QUAT_INVALID')
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def main():
    if not args.urdf.is_file() or not args.stage49.is_file():
        abort('MISSING_URDF_OR_STAGE49')
    if not Path(LEXIUM_USD_PATH).is_file():
        abort('MISSING_LEXIUM_USD')
    stage49 = json.loads(args.stage49.read_text(encoding='utf8'))
    if stage49.get('stage') != '3C49':
        abort('WRONG_STAGE49_REPORT')
    if stage49.get('uses_physx_cooked_collision_geometry') is not False:
        abort('STAGE49_SOURCE_UNEXPECTED')
    if stage49.get('frames_with_new_hull_self_pairs_vs_home') != 0 or \
       stage49.get('frames_with_hull_table_overlap') != 0 or \
       stage49.get('frames_with_hull_cube_overlap') != 0 or \
       stage49.get('inconclusive_lp_queries') != 0:
        abort('STAGE49_OFFLINE_GATE_NOT_MET')
    qhome = np.asarray(Q_HOME, dtype=float)
    if qhome.shape != (6,) or not np.allclose(stage49['home_q_rad'], qhome, atol=1e-4):
        abort('HOME_Q_SOURCE_MISMATCH')
    if not np.allclose(stage49['target_q_rad'],
                       [0.5116462574036054, 1.6679322364615043,
                        2.3215496095643826, 1.4630436610150346,
                        1.4414003069368027, 0.26596198934162374], atol=1e-5):
        abort('STAGE49_PREFERRED_TARGET_CHANGED')
    print('STAGE=3C50 MODE=PHYSX_FULL_SCENE_HOME_CONTACT_PREFLIGHT', flush=True)
    print('STAGE50_FULL_SCENE=True HOME_ONLY=True NO_PREGRASP_MOTION=True', flush=True)
    print('STAGE50_ORIGINAL_ASSETS_MODIFIED=False SELF_COLLISIONS_POLICY_CHANGED=False', flush=True)
    print('STAGE50_ACTUAL_PHYSX_SELF_COLLISIONS_TESTED=False', flush=True)
    links, joints, children, rootname = parse_urdf(args.urdf)
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1./60,
                                                   gravity=(0., 0., -9.81)))
    scene = InteractiveScene(Stage50FullScene(num_envs=1, env_spacing=2.5))
    sim.reset()
    scene.reset()
    scene.update(sim.get_physics_dt())
    robot = scene['robot']
    cube = scene['cube']
    robot_sensor = scene['robot_contact']
    cube_sensor = scene['cube_contact']
    if not robot.is_fixed_base:
        abort('ROBOT_ROOT_NOT_FIXED')
    jnames = list(robot.data.joint_names)
    bnames = list(robot.data.body_names)
    try:
        ids = [jnames.index(n) for n in LEXIUM_ARM_JOINT_NAMES]
        rootidx = bnames.index(rootname)
        toolidx = bnames.index('gripper_base_1')
    except ValueError as e:
        abort('MISSING_JOINT_OR_BODY_' + str(e))
    stage = omni.usd.get_context().get_stage()
    for prim_path in ('/World/envs/env_0/Robot', '/World/envs/env_0/Table', '/World/envs/env_0/Cube'):
        if not stage.GetPrimAtPath(prim_path).IsValid():
            abort('SCENE_PRIM_MISSING_' + prim_path)
    flags = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            attr = prim.GetAttribute('physxArticulation:enabledSelfCollisions')
            flags.append(attr.Get() if attr.IsValid() else None)
    if flags != [False]:
        abort('SELF_COLLISION_SETTING_CHANGED_' + repr(flags))
    reporter_count = sum(str(p.GetPath()).startswith('/World/envs/env_0/Robot/')
                         for p in stage.Traverse() if p.HasAPI(PhysxSchema.PhysxContactReportAPI))
    if reporter_count < 10:
        abort('ROBOT_CONTACT_REPORTER_PRIMS_TOO_FEW_' + str(reporter_count))
    rs = robot_sensor.data.net_normal_forces_w.torch
    cs = cube_sensor.data.net_normal_forces_w.torch
    if tuple(rs.shape[:1]) != (1,) or rs.ndim != 3 or rs.shape[1] < 10 or rs.shape[2] != 3:
        abort('ROBOT_CONTACT_SENSOR_SHAPE_INVALID_' + repr(tuple(rs.shape)))
    if cs.ndim != 3 or tuple(cs.shape) != (1, 1, 3):
        abort('CUBE_CONTACT_SENSOR_SHAPE_INVALID_' + repr(tuple(cs.shape)))
    print('STAGE50_ROBOT_CONTACT_REPORTERS='+str(reporter_count), flush=True)
    print('STAGE50_SENSOR_SHAPES_ROBOT_CUBE='+json.dumps([list(rs.shape), list(cs.shape)]),flush=True)
    k = robot.data.joint_stiffness.torch[0, ids].detach().cpu().numpy().astype(float)
    effort_limits = robot.data.joint_effort_limits.torch[0, ids].detach().cpu().numpy().astype(float)
    if not np.allclose(k, [400, 800, 800, 400, 400, 400], atol=0.1):
        abort('UNEXPECTED_JOINT_KP_'+repr(k.tolist()))
    if not np.allclose(effort_limits, [100]*6, atol=0.1):
        abort('UNEXPECTED_EFFORT_LIMITS_'+repr(effort_limits.tolist()))
    full_home = robot.data.default_joint_pos.torch.clone()
    q_cfg = full_home[0, ids].detach().cpu().numpy()
    if not np.allclose(q_cfg, qhome, atol=1e-4):
        abort('CONFIGURED_HOME_MISMATCH')
    robot.write_joint_state_to_sim_index(position=full_home, velocity=torch.zeros_like(full_home))
    scene.update(0.0)
    cube.write_root_pose_to_sim_index(root_pose=cube.data.default_root_pose.torch.clone())
    cube.write_root_velocity_to_sim_index(root_velocity=cube.data.default_root_vel.torch.clone())
    rootpose = robot.data.body_link_pose_w.torch[0, rootidx].detach().cpu().numpy().astype(float)
    g_root = quat_to_rot(rootpose[3:7]).T @ np.asarray([0., 0., -9.81])
    if not np.allclose(g_root, [0., -9.81, 0.], atol=.01):
        abort('ROOT_GRAVITY_FRAME_INVALID_'+repr(g_root.tolist()))
    tg = compute_root_frame_gravity(links, joints, children, rootname, qhome, g_root)
    bias = np.asarray(tg, float)/k
    if np.max(np.abs(tg)) >= 85.0 or np.max(np.abs(bias)) >= .25:
        abort('HOME_GRAVITY_TORQUE_OR_BIAS_TOO_HIGH')
    jt_limits = robot.data.joint_pos_limits.torch[0, ids].detach().cpu().numpy()
    cmd = qhome + bias
    if np.any(cmd < jt_limits[:,0]+1e-4) or np.any(cmd > jt_limits[:,1]-1e-4):
        abort('HOME_GRAVITY_BIASED_COMMAND_OUTSIDE_LIMITS')
    cmdfull = full_home.clone()
    for i, j in enumerate(ids):
        cmdfull[0,j] = float(cmd[i])
    max_contact = 0.0
    max_torque = 0.0
    max_tracking = 0.0
    max_vel = 0.0
    robot_sensor_was_finite = True
    cube_settle_n = 0.0
    cube_settle_xyz = None
    sampled = []
    print('STAGE50_HOME_GRAVITY_COMPENSATION_NM='+json.dumps(np.round(tg,4).tolist()), flush=True)
    print('STAGE50_CUBE_START_XYZ_M=[0.3, -0.3, 0.12]',flush=True)
    for step in range(1, args.warmup_steps + 1):
        if not simulation_app.is_running():
            abort('SIMULATION_STOPPED')
        robot.actuators.target_command.set_position_index(value=cmdfull,full_data=True)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
        rf = robot_sensor.data.net_normal_forces_w.torch[0]
        cf = cube_sensor.data.net_normal_forces_w.torch[0]
        if not bool(torch.isfinite(rf).all().item() and torch.isfinite(cf).all().item()):
            abort('NONFINITE_CONTACT_SENSOR_DATA_STEP_'+str(step))
        rforce = float(torch.linalg.vector_norm(rf,dim=-1).max().item())
        cforce = float(torch.linalg.vector_norm(cf,dim=-1).max().item())
        q = robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
        qvel = robot.data.joint_vel.torch[0,ids].detach().cpu().numpy().astype(float)
        eff = robot.actuators.applied_effort.torch[0,ids].detach().cpu().numpy().astype(float)
        if not np.isfinite(np.r_[q,qvel,eff]).all():
            abort('NONFINITE_ROBOT_STATE_STEP_'+str(step))
        cube_xyz = cube.data.root_pos_w.torch[0].detach().cpu().numpy().astype(float)
        max_contact = max(max_contact,rforce)
        max_torque = max(max_torque,float(np.max(np.abs(eff))))
        max_tracking = max(max_tracking,float(np.max(np.abs(q-qhome))))
        max_vel = max(max_vel,float(np.max(np.abs(qvel))))
        if rforce >= args.robot_contact_threshold_n:
            abort('UNEXPECTED_EXTERNAL_ROBOT_CONTACT_STEP_'+str(step)+'_N_'+str(round(rforce,4)))
        if np.max(np.abs(q-qhome)) >= 0.12:
            abort('HOME_TRACKING_ERROR_GT_0P12_RAD_STEP_'+str(step))
        if np.max(np.abs(eff)) >= 99.0:
            abort('HOME_ACTUATOR_NEAR_100NM_STEP_'+str(step))
        if np.max(np.abs(qvel)) >= 2.5:
            abort('HOME_JOINT_VELOCITY_TOO_HIGH_STEP_'+str(step))
        if step in (1,30,60,90,120,args.warmup_steps):
            sampled.append({'step':step,'robot_contact_max_n':rforce,'cube_contact_n':cforce,
                            'cube_xyz_m':cube_xyz.tolist(),'max_q_home_error_rad':float(np.max(np.abs(q-qhome))),
                            'max_applied_effort_nm':float(np.max(np.abs(eff)))})
            print('STAGE50_SAMPLE step={} robot_contact_n={:.5f} cube_contact_n={:.5f} cube_z_m={:.5f} qhome_error_rad={:.5f}'.format(step,rforce,cforce,cube_xyz[2],np.max(np.abs(q-qhome))),flush=True)
        if step == args.warmup_steps:
            cube_settle_n = cforce
            cube_settle_xyz = cube_xyz.tolist()
            if not (0.2 <= cforce <= 10.0):
                abort('CUBE_TABLE_CONTACT_POSITIVE_CONTROL_NOT_VERIFIED')
            if not np.linalg.norm(cube_xyz[:2]-np.array([.30,-.30])) <= .01:
                abort('CUBE_DRIFTED_LATERALLY')
            if not abs(cube_xyz[2]-.02) <= .012:
                abort('CUBE_NOT_SETTLED_ON_TABLE')
    qfinal = robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
    finalpose = robot.data.body_link_pose_w.torch[0,toolidx].detach().cpu().numpy().astype(float)
    if not np.isfinite(np.r_[qfinal,finalpose]).all():
        abort('NONFINITE_FINAL_GEOMETRY')
    result = {
        'stage':'3C50','mode':'physx_full_scene_home_contact_positive_control',
        'sim_only':True,'home_only':True,'pregrasp_motion_executed':False,
        'finger_motion_executed':False,'usd_modified':False,'self_collision_policy_modified':False,
        'physx_self_collisions_enabled':False,
        'stage49_proxy_only':True,
        'stage49_path_target_q_rad':stage49['target_q_rad'],
        'home_q_rad':qhome.tolist(),'final_arm_q_rad':qfinal.tolist(),
        'warmup_steps':args.warmup_steps,'contact_reporter_prims':reporter_count,
        'robot_contact_sensor_shape':list(rs.shape),'cube_contact_sensor_shape':list(cs.shape),
        'cube_settled_xyz_m':cube_settle_xyz,'cube_table_contact_positive_control_n':cube_settle_n,
        'max_robot_external_contact_n':max_contact,'max_home_joint_tracking_error_rad':max_tracking,
        'max_applied_effort_nm':max_torque,'max_home_joint_velocity_rad_s':max_vel,
        'final_gripper_base_pose_xyzw':finalpose.tolist(),
        'sample_summaries':sampled,
        'home_contact_positive_control_passed':True,
        'pregrasp_motion_validated':False,'self_collision_geometry_validated':False,
        'gripper_cube_opening_validated':False,'grasp_validated':False,
        'robot_motion_permission':False,'grasp_permission':False,
        'disclaimer':'Full scene, real PhysX sensor positive control for cube on table, and HOME tracking only. No pregrasp movement. Self-collisions remain disabled; robot contact sensor cannot certify absence of self-collision. This stage alone does not authorize new movement or grasp.'
    }
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print('STAGE50_HOME_SENSOR_BASELINE_PASS=True',flush=True)
    print('STAGE50_CUBE_TABLE_POSITIVE_CONTROL_N={:.6f}'.format(cube_settle_n),flush=True)
    print('STAGE50_MAX_ROBOT_EXTERNAL_CONTACT_N={:.6f}'.format(max_contact),flush=True)
    print('STAGE50_FINAL_ARM_Q_RAD='+json.dumps(qfinal.tolist()),flush=True)
    print('STAGE50_PREGRASP_MOTION_EXECUTED=False',flush=True)
    print('STAGE50_SELF_COLLISIONS_PHYSX_ENABLED=False',flush=True)
    print('STAGE50_NO_MOTION_PERMISSION=True NO_GRASP_PERMISSION=True',flush=True)
    print('STAGE50_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C50_HOME_CONTACT_PREFLIGHT_COMPLETE=True',flush=True)
    print('RESULT=STAGE3C50_FULL_SCENE_HOME_POSITIVE_CONTROL_CAPTURED_PREGRASP_NOT_MOVED',flush=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        simulation_app.close()
