"""Verification script to inspect the Schneider Lexium Cobot Articulation.

Spawns the robot alone in Isaac Sim and inspects:
  - Body links & hierarchy
  - Joint names and active articulation order
  - Joint position / velocity / effort limits
  - Base mounting orientation
  - Gripper TCP world position in the initial home configuration
"""

from __future__ import annotations

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect Schneider Lexium Cobot Articulation.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of inspection envs.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg
import isaaclab.sim as sim_utils  # VERIFY

from lexium_pick_place.robot_cfg import (
    LEXIUM_ARM_JOINT_NAMES,
    LEXIUM_COBOT_CFG,
    LEXIUM_GRIPPER_JOINT_NAMES,
    LEXIUM_TCP_OFFSET,
)


class RobotVerificationSceneCfg(InteractiveSceneCfg):
    """Scene containing only ground, lighting, and the standalone robot."""

    ground = sim_utils.GroundPlaneCfg().as_asset_base_cfg(prim_path="/World/ground")  # VERIFY
    light = sim_utils.DistantLightCfg(intensity=2000.0).as_asset_base_cfg(prim_path="/World/light")  # VERIFY

    robot = LEXIUM_COBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/gripper_base_1",
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                name="tcp",
                offset=sim_utils.TransformCfg(pos=LEXIUM_TCP_OFFSET, rot=(0.0, 0.0, 0.0, 1.0)),  # XYZW
            )
        ],
    )


def main():
    """Inspect and print all articulation parameters."""
    print("=" * 80)
    print("SCHNEIDER LEXIUM COBOT ROBOT ARTICULATION INSPECTION")
    print("=" * 80)

    scene_cfg = RobotVerificationSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim = sim_utils.SimulationContext.instance()  # VERIFY

    sim.reset()
    scene.reset()
    scene.update(dt=0.02)

    robot = scene["robot"]
    ee_frame = scene["ee_frame"]

    print("\n--- BODY LINKS ---")
    for idx, name in enumerate(robot.data.body_names):
        print(f"  [{idx:2d}] {name}")

    print("\n--- ARTICULATION JOINT ORDER & RESOLVED INDICES ---")
    all_joint_names = robot.data.joint_names
    for idx, name in enumerate(all_joint_names):
        print(f"  [{idx:2d}] {name}")

    print("\n--- ARM JOINTS (EXPLICIT ORDER) ---")
    for name in LEXIUM_ARM_JOINT_NAMES:
        j_id, _ = robot.find_joints(name)
        print(f"  {name:26s} -> Articulation Index {j_id}")

    print("\n--- GRIPPER JOINTS (EXPLICIT ORDER) ---")
    for name in LEXIUM_GRIPPER_JOINT_NAMES:
        j_id, _ = robot.find_joints(name)
        print(f"  {name:26s} -> Articulation Index {j_id}")

    print("\n--- JOINT LIMITS (PHYSX SIMULATION) ---")
    pos_limits = robot.data.soft_joint_pos_limits[0].cpu().numpy()
    vel_limits = robot.data.joint_vel_limits[0].cpu().numpy()
    effort_limits = robot.data.joint_effort_limits[0].cpu().numpy()

    print(f"  {'Joint Name':28s} {'Pos Lower (rad)':17s} {'Pos Upper (rad)':17s} {'Vel Limit':12s} {'Effort':10s}")
    for idx, name in enumerate(all_joint_names):
        print(
            f"  {name:28s} {pos_limits[idx, 0]:+16.4f}  {pos_limits[idx, 1]:+16.4f}  "
            f"{vel_limits[idx]:11.2f}  {effort_limits[idx]:9.1f}"
        )

    print("\n--- BASE ORIENTATION & POSE ---")
    base_pos = robot.data.root_pos_w[0].cpu().numpy()
    base_quat = robot.data.root_quat_w[0].cpu().numpy()
    print(f"  Base Position (x, y, z): {base_pos}")
    print(f"  Base Orientation (x, y, z, w): {base_quat} [XYZW order]")

    print("\n--- INITIAL HOME TCP POSE ---")
    tcp_pos = ee_frame.data.target_pos_w[0, 0].cpu().numpy()
    tcp_quat = ee_frame.data.target_quat_w[0, 0].cpu().numpy()  # XYZW order
    print(f"  TCP Position (x, y, z): {tcp_pos}")
    print(f"  TCP Orientation (x, y, z, w): {tcp_quat} [XYZW order]")

    print("\n" + "=" * 80)
    print("VERIFICATION COMPLETE: Stepping 50 simulation steps for visual inspection...")
    print("=" * 80)

    for _ in range(50):
        sim.step()
        scene.update(dt=0.02)

    simulation_app.close()


if __name__ == "__main__":
    main()
