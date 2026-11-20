"""MDP terms for the Schneider Lexium Cobot pick-and-place environment."""

# Re-export built-in standard terms from isaaclab.envs.mdp
from isaaclab.envs.mdp import *  # VERIFY

# Custom actions
from .actions import (
    CoupledFourFingerGripperAction,
    CoupledFourFingerGripperActionCfg,
    OPEN_FINGER_POSITIONS,
    CLOSED_FINGER_POSITIONS,
)

# Custom observations
from .observations import (
    arm_joint_pos_rel,
    arm_joint_vel_rel,
    gripper_aperture,
    cube_position_in_robot_frame,
    target_position_in_robot_frame,
)

# Custom rewards
from .rewards import (
    reach_cube_reward,
    grasp_cube_reward,
    lift_cube_reward,
    transport_cube_reward,
    place_cube_reward,
    joint_limit_proximity_penalty,
    arm_collision_penalty,
)

# Custom events
from .events import (
    reset_arm_joints_with_noise,
    reset_cube_position_on_table,
    randomize_object_friction,
)

# Custom terminations
from .terminations import (
    cube_dropped_off_table,
    place_success_termination,
)
