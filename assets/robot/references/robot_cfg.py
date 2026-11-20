"""Lexium Cobot articulation configuration for Isaac Lab v3.0.0-EA / PhysX.

The URDF-to-USD conversion is done separately in convert_urdf.py.
Keep all USD payloads next to the converted main .usda file.
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sim import UsdFileCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
from isaaclab_physx.sim.schemas import PhysxArticulationRootPropertiesCfg

# Base mount constants. The quaternion uses (x, y, z, w) = XYZW order.
LEXIUM_BASE_ROTATION_XYZW = (0.7071068, 0.0, 0.0, 0.7071068)  # XYZW order: +90 degrees about X.
LEXIUM_BASE_POSITION = (0.0, 0.0, 0.0)

LEXIUM_TCP_OFFSET = (0.002, 0.100, 0.148)

LEXIUM_ARM_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6_gripper_rotation",
]
LEXIUM_GRIPPER_JOINT_NAMES = [
    "gripper_Revolute_1",
    "gripper_Revolute_2",
    "gripper_Revolute_5",
    "gripper_Revolute_6",
]

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
LEXIUM_URDF_PATH = os.path.join(ASSETS_DIR, "lexium_cobot.urdf")
# Actual path returned by your successful conversion (NOT the old guessed .usd path).
LEXIUM_USD_PATH = "/home/aybergen/Desktop/adi/lexium_rl_workspace/lexium_pick_place/assets/usd/lexium_cobot_lab_no_newton_trial_v2/lexium_cobot.usda"

# This config is available to the pick/place scene. Assigning this material to
# the *finger collision meshes* is a separate required step before PPO training.
FINGER_MATERIAL_CFG = PhysxRigidBodyMaterialCfg(
    static_friction=1.2,
    dynamic_friction=1.1,
    restitution=0.0,
    friction_combine_mode="multiply",
)

# The converted USD retains the URDF joint limits and collision geometry.
LEXIUM_COBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=UsdFileCfg(
        usd_path=LEXIUM_USD_PATH,
        activate_contact_sensors=False,
        articulation_props=PhysxArticulationRootPropertiesCfg(
            fix_root_link=True,
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=2,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=LEXIUM_BASE_POSITION,
        rot=LEXIUM_BASE_ROTATION_XYZW,  # XYZW order.
        joint_pos={
            "joint_1": 0.0,
            "joint_2": -0.7854,
            "joint_3": 1.5708,
            "joint_4": -0.7854,
            "joint_5": 1.5708,
            "joint_6_gripper_rotation": 0.0,
            "gripper_Revolute_1": -0.5500,
            "gripper_Revolute_2": -0.3403,
            "gripper_Revolute_5": -0.3403,
            "gripper_Revolute_6": -0.3403,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arm_tuned_23": ImplicitActuatorCfg(
                joint_names_expr=['joint_2', 'joint_3'],
                stiffness=800.0,
                damping=60.0,
                joint_effort_limit=100.0,
                joint_velocity_limit=1.0,
            ),
        "arm_remaining_1456": ImplicitActuatorCfg(
                joint_names_expr=['joint_1', 'joint_4', 'joint_5', 'joint_6_gripper_rotation'],
                stiffness=400.0,
                damping=40.0,
                joint_effort_limit=100.0,
                joint_velocity_limit=1.0,
            ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=LEXIUM_GRIPPER_JOINT_NAMES,
            stiffness=2000.0,
            damping=100.0,
            joint_effort_limit=100.0,
            joint_velocity_limit=100.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
