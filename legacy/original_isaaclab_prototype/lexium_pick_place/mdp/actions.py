"""Custom action terms for 4-finger coupled gripper and joint position control."""

from __future__ import annotations

import torch
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg  # VERIFY

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

# -----------------------------------------------------------------------------
# Discrete / Continuous Target Positions for the 4 Fingers
# Open (a_grip > 0): aperture ~12.2 cm
# Closed (a_grip <= 0): fingertips meet symmetrically at center
# -----------------------------------------------------------------------------
OPEN_FINGER_POSITIONS = {
    "gripper_Revolute_1": -0.5500,
    "gripper_Revolute_2": -0.3403,
    "gripper_Revolute_5": -0.3403,
    "gripper_Revolute_6": -0.3403,
}

CLOSED_FINGER_POSITIONS = {
    "gripper_Revolute_1": 0.1309,
    "gripper_Revolute_2": 0.3403,
    "gripper_Revolute_5": 0.3403,
    "gripper_Revolute_6": 0.3403,
}


class CoupledFourFingerGripperAction(ActionTerm):
    """Coupled 1-D scalar action driving all 4 fingers synchronously.

    Action in [-1, 1]:
      a_grip <= 0 -> Closed target
      a_grip > 0  -> Open target
    Supports either binary switching or smooth linear interpolation between open and closed.
    """

    cfg: CoupledFourFingerGripperActionCfg

    def __init__(self, cfg: CoupledFourFingerGripperActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        self._robot = env.scene[cfg.asset_name]

        # Resolve explicit joint indices in the robot articulation
        self._joint_names = [
            "gripper_Revolute_1",
            "gripper_Revolute_2",
            "gripper_Revolute_5",
            "gripper_Revolute_6",
        ]
        self._joint_indices, _ = self._robot.find_joints(self._joint_names)

        # Pre-construct target tensors for open and closed states
        open_list = [OPEN_FINGER_POSITIONS[name] for name in self._joint_names]
        closed_list = [CLOSED_FINGER_POSITIONS[name] for name in self._joint_names]

        self._open_targets = torch.tensor(open_list, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )
        self._closed_targets = torch.tensor(closed_list, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )

        # Internal raw action storage (num_envs, 1)
        self._raw_actions = torch.zeros(self.num_envs, 1, dtype=torch.float32, device=self.device)
        self._processed_targets = torch.zeros(
            self.num_envs, len(self._joint_indices), dtype=torch.float32, device=self.device
        )

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_targets

    def process_actions(self, actions: torch.Tensor) -> None:
        # Clamp action to [-1, 1]
        self._raw_actions[:] = actions.view(self.num_envs, 1).clamp(-1.0, 1.0)

        if self.cfg.binary_mode:
            # Binary switch: open if > 0, closed if <= 0
            open_mask = (self._raw_actions > 0.0).repeat(1, 4)
            self._processed_targets[:] = torch.where(open_mask, self._open_targets, self._closed_targets)
        else:
            # Continuous interpolation: -1.0 is fully closed, +1.0 is fully open
            # alpha: 0.0 (closed) to 1.0 (open)
            alpha = (self._raw_actions + 1.0) * 0.5
            self._processed_targets[:] = (1.0 - alpha) * self._closed_targets + alpha * self._open_targets

    def apply_actions(self) -> None:
        self._robot.set_joint_position_target(self._processed_targets, joint_ids=self._joint_indices)


class CoupledFourFingerGripperActionCfg(ActionTermCfg):
    """Configuration for coupled 4-finger gripper action."""

    class_type: type[ActionTerm] = CoupledFourFingerGripperAction
    asset_name: str = MISSING
    binary_mode: bool = False
