#!/usr/bin/env python3
"""Stage 3C.43: inspect the UNMOVED initial HOME articulation in Isaac Lab/PhysX.

Checks URDF FK (with live PhysX joint coordinates) against live PhysX body pose,
and separately checks the world-frame mount against robot_cfg.py. Does NOT replay
Candidate1, command finger/arm motion, change USD/URDF, or authorize grasp.

No Stage 3C26 candidate file or old logs are required. The HOME measurement
is independent of any estimated Stage40 Candidate1 q; it does NOT recover
FINAL_ARM_Q_RAD measured at Candidate1.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', type=Path, required=True)
parser.add_argument('--out', type=Path, default=Path('/tmp/lexium_stage3c43_physx_home_fk_report.json'))
parser.add_argument('--num-envs', type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.num_envs != 1:
    parser.error('Stage43 requires num-envs=1')
launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np
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
class HomeReadOnlySceneCfg(InteractiveSceneCfg):
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
    print('STAGE=3C43 MODE=PHYSX_INITIAL_HOME_FK_READ_ONLY', flush=True)
    print('STAGE43_SIMULATION_ONLY=True', flush=True)
    print('STAGE43_ARM_TRAJECTORY_COMMANDED=False FINGER_MOTION_COMMANDED=False', flush=True)
    print('STAGE43_CANDIDATE1_REPLAY=False STAGE43_ASSETS_MODIFIED=False', flush=True)
    links, joints, children, root_name = parse_urdf(args.urdf)
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1/60, gravity=(0,0,-9.81)))
    scene = InteractiveScene(HomeReadOnlySceneCfg(num_envs=1, env_spacing=2.5))
    sim.reset()
    scene.reset()
    # Match the read-only scene initialization from Stage37. No manual state
    # teleports, actuator targets, sim.step calls, or candidate movements.
    scene.update(sim.get_physics_dt())
    robot = scene['robot']
    if not robot.is_fixed_base:
        raise RuntimeError('ROOT_NOT_FIXED')
    jnames = list(robot.data.joint_names)
    bnames = list(robot.data.body_names)
    ids = [jnames.index(n) for n in LEXIUM_ARM_JOINT_NAMES]
    rid = bnames.index(root_name)
    tid = bnames.index('gripper_base_1')
    q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(float)
    q_default = robot.data.default_joint_pos.torch[0,ids].detach().cpu().numpy().astype(float)
    root = robot.data.body_link_pose_w.torch[0,rid].detach().cpu().numpy().astype(float)
    tool = robot.data.body_link_pose_w.torch[0,tid].detach().cpu().numpy().astype(float)
    if not np.isfinite(np.r_[q, q_default, root, tool]).all():
        raise RuntimeError('Nonfinite PhysX initial state')
    Rroot = quat_xyzw_to_R(root[3:7]); Rtool = quat_xyzw_to_R(tool[3:7])
    Rrel, prel = arm_fk_rel_root(joints, children, root_name, q)
    predicted_xyz = root[:3] + Rroot @ prel
    predicted_R = Rroot @ Rrel
    position_error_mm = float(np.linalg.norm(predicted_xyz-tool[:3]) * 1000)
    angle_error_deg = rot_error_deg(predicted_R,Rtool)
    Rcfg = quat_xyzw_to_R(LEXIUM_BASE_ROTATION_XYZW)
    mount_position_error_mm = float(np.linalg.norm(np.asarray(LEXIUM_BASE_POSITION)-root[:3])*1000)
    mount_angle_error_deg = rot_error_deg(Rcfg,Rroot)
    tcp_offset = np.asarray(LEXIUM_TCP_OFFSET,dtype=float)
    tcp_actual = tool[:3] + Rtool @ tcp_offset
    tcp_urdf = predicted_xyz + predicted_R @ tcp_offset
    tcp_error_mm = float(np.linalg.norm(tcp_actual-tcp_urdf)*1000)
    q_default_error = float(np.max(np.abs(q-q_default)))
    home_fk_match = bool(position_error_mm < 2.0 and angle_error_deg < 0.5 and q_default_error < 0.02)
    mount_match = bool(mount_position_error_mm < 2.0 and mount_angle_error_deg < 0.5)
    report = {
        'stage': '3C43', 'mode': 'physx_initial_home_fk_read_only',
        'sim_only': True, 'arm_trajectory_commanded': False,
        'finger_motion_commanded': False, 'candidate1_replayed': False,
        'assets_modified': False,
        'arm_joint_names': list(LEXIUM_ARM_JOINT_NAMES),
        'arm_q_initial_physx_rad': q.tolist(),
        'arm_q_default_rad': q_default.tolist(),
        'max_q_default_error_rad': q_default_error,
        'physx_root_link_pose_xyzw': root.tolist(),
        'physx_gripper_base_pose_xyzw': tool.tolist(),
        'urdf_gripper_base_predicted_xyz_m': predicted_xyz.tolist(),
        'home_fk_position_error_mm': position_error_mm,
        'home_fk_orientation_error_deg': angle_error_deg,
        'home_tcp_fk_error_mm': tcp_error_mm,
        'robot_cfg_base_mount_position_error_mm': mount_position_error_mm,
        'robot_cfg_base_mount_orientation_error_deg': mount_angle_error_deg,
        'home_physx_urdf_fk_match': home_fk_match,
        'robot_cfg_mount_matches_physx': mount_match,
        'observed_home_tcp_xyz_m': tcp_actual.tolist(),
        'prior_candidate1_physx_q_recovered': False,
        'stage40_baseline_independently_validated': False,
        'physical_motion_permission': False,
        'disclaimer': ('Only the initial HOME state was read; Candidate1 was NOT replayed. '
          'A HOME FK match tests the URDF/PhysX chain in that configuration but '
          'does not independently validate Stage40 inferred Candidate1 q, motion '
          'paths, real collision shapes, gripper closure or grasp.'),
    }
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2),encoding='utf-8')
    print('STAGE43_INITIAL_ARM_Q_RAD='+json.dumps(q.tolist()),flush=True)
    print('STAGE43_MAX_HOME_JOINT_ERROR_RAD=%.8f' % q_default_error,flush=True)
    print('STAGE43_URDF_VS_PHYSX_GRIPPER_ERROR_MM=%.6f ORI_DEG=%.6f' % (position_error_mm,angle_error_deg),flush=True)
    print('STAGE43_ROBOT_CFG_MOUNT_ERROR_MM=%.6f ORI_DEG=%.6f' % (mount_position_error_mm,mount_angle_error_deg),flush=True)
    print('STAGE43_HOME_PHYSX_URDF_FK_MATCH='+str(home_fk_match),flush=True)
    print('STAGE43_CFG_MOUNT_MATCH='+str(mount_match),flush=True)
    print('STAGE43_CANDIDATE1_PHYSX_Q_RECOVERED=False',flush=True)
    print('STAGE43_NO_MOTION_PERMISSION=True',flush=True)
    print('STAGE43_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C43_HOME_FK_AUDIT_COMPLETE=True',flush=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        simulation_app.close()
