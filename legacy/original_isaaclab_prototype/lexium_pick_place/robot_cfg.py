"""Robot Articulation and URDF-to-USD configuration for Schneider Lexium Cobot."""

import os
from isaaclab.actuators import ImplicitActuatorCfg  # VERIFY
from isaaclab.assets import ArticulationCfg
from isaaclab.sim.converters import UrdfConverterCfg  # VERIFY
from isaaclab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg  # VERIFY

# -----------------------------------------------------------------------------
# Base Mounting & Coordinate Transform Constants
# URDF joint_1 has axis [0, -1, 0] in base_link (CAD Y-Up).
# Rotating +90 degrees about X aligns CAD +Y with Isaac Sim +Z (Z-Up).
# Quaternion format: (x, y, z, w) as required in Isaac Lab 3.0.
# -----------------------------------------------------------------------------
# (x, y, z, w) for +90 deg rotation about X: sin(45 deg) = 0.7071068, cos(45 deg) = 0.7071068
LEXIUM_BASE_ROTATION_XYZW = (0.7071068, 0.0, 0.0, 0.7071068)  # XYZW order
LEXIUM_BASE_POSITION = (0.0, 0.0, 0.0)

# End-effector TCP local offset from gripper_base_1 frame
# Centroid between opposing fingertip banks at grasp depth
LEXIUM_TCP_OFFSET = (0.002, 0.100, 0.148)

# Joint names in strict kinematic sequence
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

# Asset paths
ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
LEXIUM_URDF_PATH = os.path.join(ASSETS_DIR, "lexium_cobot.urdf")
LEXIUM_USD_PATH = os.path.join(ASSETS_DIR, "usd", "lexium_cobot.usd")

# -----------------------------------------------------------------------------
# High-friction physics material for gripper fingers
# -----------------------------------------------------------------------------
FINGER_MATERIAL_CFG = RigidBodyMaterialCfg(
    static_friction=1.2,
    dynamic_friction=1.1,
    restitution=0.0,
    friction_combine_mode="multiply",  # VERIFY
)

# -----------------------------------------------------------------------------
# URDF to USD Converter Configuration
# -----------------------------------------------------------------------------
LEXIUM_URDF_CONVERTER_CFG = UrdfConverterCfg(
    asset_path=LEXIUM_URDF_PATH,
    usd_dir=os.path.join(ASSETS_DIR, "usd"),
    usd_file_name="lexium_cobot.usd",
    force_usd_conversion=False,
    fix_base=True,
    merge_fixed_joints=False,
    self_collision=True,
    collider_type="convex_decomposition",  # VERIFY
    replace_cylinders_with_capsules=False,
)

# -----------------------------------------------------------------------------
# Robot Articulation Configuration
# -----------------------------------------------------------------------------
LEXIUM_COBOT_CFG = ArticulationCfg(
    spawn=LEXIUM_URDF_CONVERTER_CFG,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=LEXIUM_BASE_POSITION,
        rot=LEXIUM_BASE_ROTATION_XYZW,  # (x, y, z, w) XYZW order
        joint_pos={
            # Arm joints posed so gripper hovers facing downward above table workspace
            "joint_1": 0.0,
            "joint_2": -0.7854,
            "joint_3": 1.5708,
            "joint_4": -0.7854,
            "joint_5": 1.5708,
            "joint_6_gripper_rotation": 0.0,
            # Gripper fingers initialized to open aperture
            "gripper_Revolute_1": -0.5500,
            "gripper_Revolute_2": -0.3403,
            "gripper_Revolute_5": -0.3403,
            "gripper_Revolute_6": -0.3403,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6_gripper_rotation",
            ],
            stiffness=400.0,
            damping=40.0,
            effort_limit=100.0,
            velocity_limit=1.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[
                "gripper_Revolute_1",
                "gripper_Revolute_2",
                "gripper_Revolute_5",
                "gripper_Revolute_6",
            ],
            stiffness=2000.0,
            damping=100.0,
            effort_limit=100.0,
            velocity_limit=100.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
