"""Termination conditions for Lexium pick-and-place task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg  # VERIFY

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def cube_dropped_off_table(
    env: ManagerBasedEnv,
    height_threshold: float = -0.05,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Terminate episode if the cube falls below the tabletop surface."""
    cube: RigidObject = env.scene[cube_cfg.name]
    cube_z = cube.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return cube_z < height_threshold


def place_success_termination(
    env: ManagerBasedEnv,
    dist_threshold: float = 0.03,
    vel_threshold: float = 0.05,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate successfully when cube is accurately placed and released."""
    cube: RigidObject = env.scene[cube_cfg.name]
    robot: Articulation = env.scene[robot_cfg.name]

    cube_pos = cube.data.root_pos_w
    cube_vel = torch.norm(cube.data.root_lin_vel_w, dim=-1)
    target_pos = env.command_manager.get_command("object_pose")[:, :3]  # VERIFY

    at_target = torch.norm(cube_pos - target_pos, dim=-1) < dist_threshold
    is_still = cube_vel < vel_threshold

    finger_joints = robot.data.joint_pos[:, [6, 7, 8, 9]]
    gripper_opened = torch.mean(finger_joints, dim=-1) <= 0.0

    return at_target & is_still & gripper_opened
