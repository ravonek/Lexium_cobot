#!/usr/bin/env python3
"""Stage 3C.44: explicitly set HOME in simulation then cross-check PhysX and URDF.

Checks URDF FK with live PhysX joint coordinates after explicit HOME setting,
and checks the world-frame mount against robot_cfg.py. Does NOT replay
Candidate1, plan movement, change USD/URDF, or authorize grasp. One simulation physics step is used after writing the default HOME state; there is no cube or table.

No Stage 3C26 candidate file or old logs are required. This stage DOES NOT
recover the true Candidate1 joint state, and DOES NOT authorize any grasp.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', type=Path, required=True)
parser.add_argument('--out', type=Path, default=Path('/tmp/lexium_stage3c44_explicit_home_fk_report.json'))
parser.add_argument('--num-envs', type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.num_envs != 1:
    parser.error('Stage44 requires num-envs=1')
launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from lexium_pick_place.robot_cfg import (
    LEXIUM_COBOT_CFG, LEXIUM_ARM_JOINT_NAMES,
    LEXIUM_BASE_ROTATION_XYZW, LEXIUM_BASE_POSITION,
    LEXIUM_TCP_OFFSET, LEXIUM_USD_PATH,
)
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import (
    parse_urdf, rpy_to_R, axis_rotation,
)

@configclass
class ExplicitHomeSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = LEXIUM_COBOT_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    light: AssetBaseCfg = AssetBaseCfg(prim_path='/World/Light',
        spawn=sim_utils.DomeLightCfg(intensity=2000.0))


def quat_xyzw_to_R(quat):
    x, y, z, w = np.asarray(quat, dtype=float)
    mag = math.sqrt(x*x + y*y + z*z + w*w)
    if mag < 1e-8:
        raise RuntimeError('Invalid quaternion from PhysX')
    x,y,z,w = [float(v/mag) for v in (x,y,z,w)]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def rot_error_deg(Ra, Rb):
    return float(np.rad2deg(np.arccos(np.clip((np.trace(Ra.T@Rb)-1)*.5,-1.,1.))))


def arm_fk_rel_root(joints, children, root_name, q):
    """Independent URDF chain transform, root link -> gripper_base_1."""
    qmap = dict(zip(LEXIUM_ARM_JOINT_NAMES, q))
    poses = {root_name: (np.eye(3), np.zeros(3))}
    todo = [root_name]
    while todo:
        parent = todo.pop()
        Rparent, pparent = poses[parent]
        for name in children.get(parent, ()):
            j = joints[name]
            Rjoint = Rparent @ rpy_to_R(j['rpy'])
            pjoint = pparent + Rparent @ j['xyz']
            typ = j['type']
            qi = float(qmap.get(name, 0.0))
            if typ in ('revolute', 'continuous'):
                Rchild, pchild = Rjoint @ axis_rotation(j['axis'], qi), pjoint
            elif typ == 'prismatic':
                Rchild, pchild = Rjoint, pjoint + Rjoint @ j['axis'] * qi
            else:
                Rchild, pchild = Rjoint, pjoint
            poses[j['child']] = (Rchild, pchild)
            todo.append(j['child'])
    if 'gripper_base_1' not in poses:
        raise RuntimeError('gripper_base_1 missing from URDF FK')
    return poses['gripper_base_1']


def main():
    if not args.urdf.is_file():
        raise FileNotFoundError('Missing URDF: '+str(args.urdf))
    if not Path(LEXIUM_USD_PATH).is_file():
        raise FileNotFoundError('Missing USD: '+LEXIUM_USD_PATH)
    print('STAGE=3C44 MODE=PHYSX_EXPLICIT_HOME_INITIALIZATION_FK',flush=True)
    print('STAGE44_SIMULATION_ONLY=True',flush=True)
    print('STAGE44_DEFAULT_HOME_STATE_WRITTEN_TO_SIM=True',flush=True)
    print('STAGE44_ONE_PHYSICS_STEP=True',flush=True)
    print('STAGE44_CANDIDATE1_REPLAY=False FINGER_TRAJECTORY_COMMANDED=False',flush=True)
    print('STAGE44_CUBE_TABLE_PRESENT=False STAGE44_ASSETS_MODIFIED=False',flush=True)
    links, joints, children, root_name = parse_urdf(args.urdf)
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1./60, gravity=(0,0,-9.81)))
    scene = InteractiveScene(ExplicitHomeSceneCfg(num_envs=1, env_spacing=2.5))
    sim.reset()
    scene.reset()
    scene.update(sim.get_physics_dt())
    robot = scene['robot']
    if not robot.is_fixed_base:
        raise RuntimeError('ROOT_NOT_FIXED')
    jnames = list(robot.data.joint_names)
    bnames = list(robot.data.body_names)
    ids = [jnames.index(n) for n in LEXIUM_ARM_JOINT_NAMES]
    rid = bnames.index(root_name)
    tid = bnames.index('gripper_base_1')
    q_initial = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(float)
    q_default = robot.data.default_joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
    full_default = robot.data.default_joint_pos.torch.clone()
    if not np.isfinite(np.r_[q_initial, q_default]).all():
        raise RuntimeError('NONFINITE_INITIAL_JOINT_STATE')

    # Crucial difference from Stage43: Stage37 physically writes HOME joint
    # state before its verified Candidate1 arm motion. Reproduce that
    # initialization, but NEVER command the Candidate1 trajectory.
    robot.write_joint_state_to_sim_index(position=full_default, velocity=torch.zeros_like(full_default))
    scene.update(0.0)
    q_after_write = robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)

    # One step updates the runtime rigid-body positions and allows comparing
    # an actual PhysX FK pose rather than a stale body pose cache.
    robot.actuators.target_command.set_position_index(value=full_default, full_data=True)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    q = robot.data.joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
    root = robot.data.body_link_pose_w.torch[0,rid].detach().cpu().numpy().astype(float)
    tool = robot.data.body_link_pose_w.torch[0,tid].detach().cpu().numpy().astype(float)
    if not np.isfinite(np.r_[q, q_default, root, tool]).all():
        raise RuntimeError('NONFINITE_PHYSX_STATE')
    Rroot = quat_xyzw_to_R(root[3:7])
    Rtool = quat_xyzw_to_R(tool[3:7])
    Rrel, prel = arm_fk_rel_root(joints, children, root_name, q)
    predicted_xyz = root[:3] + Rroot @ prel
    predicted_R = Rroot @ Rrel
    position_error_mm = float(np.linalg.norm(predicted_xyz-tool[:3])*1000)
    angle_error_deg = rot_error_deg(predicted_R, Rtool)
    Rcfg = quat_xyzw_to_R(LEXIUM_BASE_ROTATION_XYZW)
    mount_position_error_mm = float(np.linalg.norm(np.asarray(LEXIUM_BASE_POSITION)-root[:3])*1000)
    mount_angle_error_deg = rot_error_deg(Rcfg,Rroot)
    offset = np.asarray(LEXIUM_TCP_OFFSET,dtype=float)
    tcp_observed = tool[:3] + Rtool@offset
    tcp_predicted = predicted_xyz + predicted_R@offset
    tcp_error_mm = float(np.linalg.norm(tcp_observed-tcp_predicted)*1000)
    initial_err = float(np.max(np.abs(q_initial-q_default)))
    post_write_err = float(np.max(np.abs(q_after_write-q_default)))
    final_err = float(np.max(np.abs(q-q_default)))
    # FK matching alone does not establish that state was HOME.
    fk_match = bool(position_error_mm<2.0 and angle_error_deg<0.5)
    home_match = bool(fk_match and final_err<0.02 and post_write_err<0.02)
    mount_match = bool(mount_position_error_mm<2.0 and mount_angle_error_deg<0.5)
    report = {
        'stage':'3C44',
        'mode':'physx_explicit_home_initialization_fk',
        'sim_only':True,
        'one_physics_step':True,
        'explicit_default_joint_state_written':True,
        'candidate1_replayed':False,
        'arm_trajectory_commanded':False,
        'finger_trajectory_commanded':False,
        'cube_or_table_present':False,
        'assets_modified':False,
        'arm_joint_names':list(LEXIUM_ARM_JOINT_NAMES),
        'arm_q_at_sim_reset_rad':q_initial.tolist(),
        'arm_q_default_home_rad':q_default.tolist(),
        'arm_q_immediately_after_home_write_rad':q_after_write.tolist(),
        'arm_q_after_one_physx_step_rad':q.tolist(),
        'initial_max_home_joint_error_rad':initial_err,
        'after_write_max_home_joint_error_rad':post_write_err,
        'after_step_max_home_joint_error_rad':final_err,
        'physx_root_pose_xyzw':root.tolist(),
        'physx_gripper_pose_xyzw':tool.tolist(),
        'urdf_predicted_gripper_xyz_m':predicted_xyz.tolist(),
        'urdf_physx_position_error_mm':position_error_mm,
        'urdf_physx_orientation_error_deg':angle_error_deg,
        'urdf_physx_tcp_error_mm':tcp_error_mm,
        'robot_cfg_mount_position_error_mm':mount_position_error_mm,
        'robot_cfg_mount_orientation_error_deg':mount_angle_error_deg,
        'arm_fk_match':fk_match,
        'explicit_home_fk_match':home_match,
        'robot_cfg_mount_match':mount_match,
        'observed_home_tcp_xyz_m':tcp_observed.tolist(),
        'candidate1_actual_joint_q_recovered':False,
        'stage40_candidate1_baseline_independently_validated':False,
        'full_arm_collision_validated':False,
        'grasp_validated':False,
        'physical_motion_permission':False,
        'disclaimer':('The stage only checks joint-state initialization at the explicitly written HOME config and URDF/PhysX FK after one sim-only physics step. Stage40 Candidate1 q and entire path remain unverified. No movement to the cube or grasp authorized.'),
    }
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('STAGE44_Q_AT_SIM_RESET_RAD='+json.dumps(q_initial.tolist()),flush=True)
    print('STAGE44_Q_DEFAULT_HOME_RAD='+json.dumps(q_default.tolist()),flush=True)
    print('STAGE44_Q_AFTER_HOME_WRITE_RAD='+json.dumps(q_after_write.tolist()),flush=True)
    print('STAGE44_Q_AFTER_ONE_STEP_RAD='+json.dumps(q.tolist()),flush=True)
    print('STAGE44_INITIAL_MAX_Q_DEFAULT_ERROR_RAD=%.8f' % initial_err,flush=True)
    print('STAGE44_POST_WRITE_MAX_Q_DEFAULT_ERROR_RAD=%.8f' % post_write_err,flush=True)
    print('STAGE44_FINAL_MAX_Q_DEFAULT_ERROR_RAD=%.8f' % final_err,flush=True)
    print('STAGE44_URDF_VS_PHYSX_GRIPPER_ERROR_MM=%.6f ORI_DEG=%.6f' % (position_error_mm,angle_error_deg),flush=True)
    print('STAGE44_HOME_STATE_AND_FK_MATCH='+str(home_match),flush=True)
    print('STAGE44_CFG_MOUNT_MATCH='+str(mount_match),flush=True)
    print('STAGE44_CANDIDATE1_PHYSX_Q_RECOVERED=False',flush=True)
    print('STAGE44_NO_MOTION_TO_CUBE=True',flush=True)
    print('STAGE44_NO_GRASP_PERMISSION=True',flush=True)
    print('STAGE44_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C44_EXPLICIT_HOME_AUDIT_COMPLETE=True',flush=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        simulation_app.close()
