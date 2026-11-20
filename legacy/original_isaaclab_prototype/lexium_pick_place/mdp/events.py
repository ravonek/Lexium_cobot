"""Reset and domain randomization events for Lexium pick-and-place."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg  # VERIFY

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def reset_arm_joints_with_noise(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    position_noise_range: tuple[float, float] = (-0.05, 0.05),
    velocity_noise_range: tuple[float, float] = (-0.01, 0.01),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> None:
    """Reset the 6 arm joints to default pose with additive uniform noise."""
    robot: Articulation = env.scene[asset_cfg.name]
    joint_pos = robot.data.default_joint_pos[env_ids].clone()
    joint_vel = robot.data.default_joint_vel[env_ids].clone()

    pos_noise = (
        torch.rand((len(env_ids), len(asset_cfg.joint_ids)), device=env.device)
        * (position_noise_range[1] - position_noise_range[0])
        + position_noise_range[0]
    )
    vel_noise = (
        torch.rand((len(env_ids), len(asset_cfg.joint_ids)), device=env.device)
        * (velocity_noise_range[1] - velocity_noise_range[0])
        + velocity_noise_range[0]
    )

    joint_pos[:, asset_cfg.joint_ids] += pos_noise
    joint_vel[:, asset_cfg.joint_ids] += vel_noise

    robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)


def reset_cube_position_on_table(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    x_range: tuple[float, float] = (0.25, 0.45),
    y_range: tuple[float, float] = (-0.20, 0.20),
    z_table: float = 0.025,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> None:
    """Spawn the manipuland cube at a randomized position on the tabletop."""
    cube: RigidObject = env.scene[cube_cfg.name]
    num_resets = len(env_ids)

    root_state = cube.data.default_root_state[env_ids].clone()

    # Table workspace random positions
    x = torch.rand(num_resets, device=env.device) * (x_range[1] - x_range[0]) + x_range[0]
    y = torch.rand(num_resets, device=env.device) * (y_range[1] - y_range[0]) + y_range[0]
    z = torch.full((num_resets,), z_table, device=env.device)

    root_state[:, 0] = x + env.scene.env_origins[env_ids, 0]
    root_state[:, 1] = y + env.scene.env_origins[env_ids, 1]
    root_state[:, 2] = z + env.scene.env_origins[env_ids, 2]

    # Upright identity orientation in XYZW: (0, 0, 0, 1)
    root_state[:, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=env.device)
    root_state[:, 7:13] = 0.0  # Zero velocities

    cube.write_root_state_to_sim(root_state, env_ids=env_ids)


def randomize_object_friction(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    static_friction_range: tuple[float, float] = (0.8, 1.5),
    dynamic_friction_range: tuple[float, float] = (0.7, 1.3),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> None:
    """Randomize the physics material friction of the cube across episodes."""
    # VERIFY: In Isaac Lab 3.0, material property randomization can be applied directly to rigid objects
    obj: RigidObject = env.scene[asset_cfg.name]
    materials = obj.root_physx_view.get_material_properties()  # VERIFY
    if materials is not None:
        num_resets = len(env_ids)
        static_f = (
            torch.rand(num_resets, device=env.device) * (static_friction_range[1] - static_friction_range[0])
            + static_friction_range[0]
        )
        dynamic_f = (
            torch.rand(num_resets, device=env.device) * (dynamic_friction_range[1] - dynamic_friction_range[0])
            + dynamic_friction_range[0]
        )
        materials.static_friction[env_ids] = static_f
        materials.dynamic_friction[env_ids] = dynamic_f
        obj.root_physx_view.set_material_properties(materials, env_ids)  # VERIFY
