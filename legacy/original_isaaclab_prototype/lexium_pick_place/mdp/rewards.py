"""Staged reward terms and regularization penalties for Lexium Cobot pick-and-place."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg  # VERIFY
from isaaclab.sensors import ContactSensor, FrameTransformer  # VERIFY

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def reach_cube_reward(
    env: ManagerBasedEnv,
    std: float = 0.10,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Staged Reach: Tanh kernel on Euclidean distance between TCP and cube.

    r_reach = 1.0 - tanh(||p_tcp - p_cube|| / std)
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cube: RigidObject = env.scene[cube_cfg.name]

    tcp_pos = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    cube_pos = cube.data.root_pos_w  # (num_envs, 3)

    distance = torch.norm(tcp_pos - cube_pos, dim=-1)
    return 1.0 - torch.tanh(distance / std)


def grasp_cube_reward(
    env: ManagerBasedEnv,
    distance_threshold: float = 0.035,
    force_threshold: float = 0.5,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    finger_contact_cfg: SceneEntityCfg = SceneEntityCfg("finger_contact_sensor"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Staged Grasp: Non-zero contact on opposing finger banks (f1/f2 and f3/f4)

    plus gripper closed and TCP near the cube.
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cube: RigidObject = env.scene[cube_cfg.name]
    contact_sensor: ContactSensor = env.scene[finger_contact_cfg.name]
    robot: Articulation = env.scene[robot_cfg.name]

    tcp_pos = ee_frame.data.target_pos_w[:, 0, :]
    cube_pos = cube.data.root_pos_w
    distance = torch.norm(tcp_pos - cube_pos, dim=-1)
    near_cube = distance < distance_threshold

    # Contact forces on finger bodies
    # net_forces_w shape: (num_envs, num_bodies, 3)
    forces = contact_sensor.data.net_forces_w
    force_norms = torch.norm(forces, dim=-1)  # (num_envs, 4)

    # Opposing banks: fingers 0 and 1 (f1, f2) vs fingers 2 and 3 (f3, f4)
    bank_1_contact = (force_norms[:, 0] > force_threshold) | (force_norms[:, 1] > force_threshold)
    bank_2_contact = (force_norms[:, 2] > force_threshold) | (force_norms[:, 3] > force_threshold)
    both_banks_contact = bank_1_contact & bank_2_contact

    # Gripper closed check: mean finger angle > 0.0 rad (closed towards +0.3403)
    finger_joints = robot.data.joint_pos[:, [6, 7, 8, 9]]  # gripper joints
    gripper_closed = torch.mean(finger_joints, dim=-1) > 0.0

    valid_grasp = near_cube & both_banks_contact & gripper_closed
    return valid_grasp.float()


def lift_cube_reward(
    env: ManagerBasedEnv,
    table_height: float = 0.0,
    target_lift_height: float = 0.15,
    lift_threshold: float = 0.02,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Staged Lift: Linear reward for lifting cube above table threshold."""
    cube: RigidObject = env.scene[cube_cfg.name]
    cube_z = cube.data.root_pos_w[:, 2]

    lift_offset = cube_z - table_height
    is_lifted = lift_offset > lift_threshold

    reward = torch.clamp((lift_offset - lift_threshold) / (target_lift_height - lift_threshold), 0.0, 1.0)
    return is_lifted.float() * reward


def transport_cube_reward(
    env: ManagerBasedEnv,
    std: float = 0.15,
    table_height: float = 0.0,
    lift_threshold: float = 0.02,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Staged Transport: Reward bringing lifted cube to target placement site."""
    cube: RigidObject = env.scene[cube_cfg.name]
    cube_pos = cube.data.root_pos_w
    target_pos = env.command_manager.get_command("object_pose")[:, :3]  # VERIFY

    is_lifted = (cube_pos[:, 2] - table_height) > lift_threshold
    distance = torch.norm(cube_pos - target_pos, dim=-1)
    transport_rew = 1.0 - torch.tanh(distance / std)

    return is_lifted.float() * transport_rew


def place_cube_reward(
    env: ManagerBasedEnv,
    dist_threshold: float = 0.03,
    vel_threshold: float = 0.10,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Staged Place: Bonus when cube is at target position, stable, and gripper is released."""
    cube: RigidObject = env.scene[cube_cfg.name]
    robot: Articulation = env.scene[robot_cfg.name]

    cube_pos = cube.data.root_pos_w
    cube_vel = torch.norm(cube.data.root_lin_vel_w, dim=-1)
    target_pos = env.command_manager.get_command("object_pose")[:, :3]  # VERIFY

    at_target = torch.norm(cube_pos - target_pos, dim=-1) < dist_threshold
    is_still = cube_vel < vel_threshold

    # Gripper released check: mean finger angle <= 0.0 rad (opening towards -0.3403 / -0.5500)
    finger_joints = robot.data.joint_pos[:, [6, 7, 8, 9]]
    gripper_opened = torch.mean(finger_joints, dim=-1) <= 0.0

    placed = at_target & is_still & gripper_opened
    return placed.float()


def joint_limit_proximity_penalty(
    env: ManagerBasedEnv,
    margin_ratio: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize arm joints entering within margin_ratio (5%) of their limits."""
    robot: Articulation = env.scene[asset_cfg.name]
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    limits_lower = robot.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    limits_upper = robot.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]

    span = limits_upper - limits_lower
    margin = margin_ratio * span

    too_low = torch.clamp((limits_lower + margin) - joint_pos, min=0.0)
    too_high = torch.clamp(joint_pos - (limits_upper - margin), min=0.0)

    penalty = torch.sum(too_low + too_high, dim=-1)
    return penalty


def arm_collision_penalty(
    env: ManagerBasedEnv,
    force_threshold: float = 1.0,
    arm_contact_cfg: SceneEntityCfg = SceneEntityCfg("arm_contact_sensor"),
) -> torch.Tensor:
    """Penalize unintended contact forces on link1_1..link4_1, LINK5_1, and gripper_base_1."""
    arm_sensor: ContactSensor = env.scene[arm_contact_cfg.name]
    forces = torch.norm(arm_sensor.data.net_forces_w, dim=-1)  # (num_envs, num_arm_bodies)
    in_collision = torch.any(forces > force_threshold, dim=-1)
    return in_collision.float()
