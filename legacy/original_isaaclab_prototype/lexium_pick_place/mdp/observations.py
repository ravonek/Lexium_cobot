"""Observation functions for Schneider Lexium Pick-and-Place."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg  # VERIFY

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def arm_joint_pos_rel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Relative position of the 6 arm joints with respect to default home values."""
    robot: Articulation = env.scene[asset_cfg.name]
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    default_joint_pos = robot.data.default_joint_pos[:, asset_cfg.joint_ids]
    return joint_pos - default_joint_pos


def arm_joint_vel_rel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Joint velocity of the 6 arm joints relative to zero velocity."""
    robot: Articulation = env.scene[asset_cfg.name]
    return robot.data.joint_vel[:, asset_cfg.joint_ids]


def gripper_aperture(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Normalized scalar aperture of the 4-finger gripper [0: closed, 1: open].

    Calculated from the mean displacement of fingers 2, 5, and 6 from closed (+0.3403) to open (-0.3403).
    """
    robot: Articulation = env.scene[asset_cfg.name]
    finger_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    # At closed (+0.3403), normalized = 0.0; at open (-0.3403), normalized = 1.0
    mean_val = torch.mean(finger_pos, dim=-1, keepdim=True)
    aperture = (0.3403 - mean_val) / 0.6806
    return aperture.clamp(0.0, 1.0)


def cube_position_in_robot_frame(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Position of the manipuland cube expressed in the robot base frame."""
    robot: Articulation = env.scene[robot_cfg.name]
    cube: RigidObject = env.scene[cube_cfg.name]

    # World poses
    base_pos = robot.data.root_pos_w
    base_quat = robot.data.root_quat_w  # XYZW order (x, y, z, w)
    cube_pos = cube.data.root_pos_w

    # Relative position in world coordinates
    rel_pos_w = cube_pos - base_pos

    # Rotate vector into base frame using quaternion inverse: q_inv * v * q
    # For unit quaternion q = [x, y, z, w], q_inv = [-x, -y, -z, w]
    x, y, z, w = base_quat[:, 0], base_quat[:, 1], base_quat[:, 2], base_quat[:, 3]
    # Standard quaternion rotation of a vector v by conjugate q*
    t = 2.0 * torch.cross(base_quat[:, :3], rel_pos_w, dim=-1)
    # Forward rotation by q*: v' = v - w*t + cross(q_vec, t)
    pos_b = rel_pos_w - w.unsqueeze(-1) * t + torch.cross(base_quat[:, :3], t, dim=-1)
    return pos_b


def target_position_in_robot_frame(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Position of the target placement site expressed in the robot base frame."""
    robot: Articulation = env.scene[robot_cfg.name]
    target_pos_w = env.command_manager.get_command("object_pose")[:, :3]  # VERIFY

    base_pos = robot.data.root_pos_w
    base_quat = robot.data.root_quat_w  # XYZW order (x, y, z, w)

    rel_pos_w = target_pos_w - base_pos
    w = base_quat[:, 3].unsqueeze(-1)
    q_vec = base_quat[:, :3]
    t = 2.0 * torch.cross(q_vec, rel_pos_w, dim=-1)
    pos_b = rel_pos_w - w * t + torch.cross(q_vec, t, dim=-1)
    return pos_b
