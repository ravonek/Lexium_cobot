"""Environment configuration for Schneider Lexium Cobot Pick-and-Place."""

from __future__ import annotations

import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils  # VERIFY
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    CurriculumTermCfg,
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg
from isaaclab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg  # VERIFY

import lexium_pick_place.mdp as mdp
from lexium_pick_place.robot_cfg import (
    FINGER_MATERIAL_CFG,
    LEXIUM_ARM_JOINT_NAMES,
    LEXIUM_COBOT_CFG,
    LEXIUM_GRIPPER_JOINT_NAMES,
    LEXIUM_TCP_OFFSET,
)

# -----------------------------------------------------------------------------
# Scene Configuration
# -----------------------------------------------------------------------------
class LexiumSceneCfg(InteractiveSceneCfg):
    """Pick-and-place workspace scene configuration."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # Table surface
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(0.9, 1.2, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.35, 0.0, -0.2),
            rot=(0.0, 0.0, 0.0, 1.0),  # XYZW order
        ),
    )

    # Robot arm + 4-finger gripper
    robot: ArticulationCfg = LEXIUM_COBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Manipuland cube with high friction
    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.7, 0.2)),
            physics_material=FINGER_MATERIAL_CFG,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=2,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.001),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.35, 0.0, 0.02),
            rot=(0.0, 0.0, 0.0, 1.0),  # XYZW order
        ),
    )

    # Target placement visualization marker (sphere)
    target_marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.SphereCfg(
            radius=0.025,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.9, 0.1, 0.1), opacity=0.6  # VERIFY
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.35, 0.25, 0.025),
            rot=(0.0, 0.0, 0.0, 1.0),  # XYZW order
        ),
    )

    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.9)),
    )

    # TCP Frame Transformer on gripper_base_1
    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/gripper_base_1",
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                name="tcp",
                offset=sim_utils.TransformCfg(
                    pos=LEXIUM_TCP_OFFSET,
                    rot=(0.0, 0.0, 0.0, 1.0),  # XYZW order
                ),
            )
        ],
    )

    # Contact sensor on the 4 finger links
    finger_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/gripper_finger_.*",
        update_period=0.0,
        history_length=1,
        track_air_time=False,
    )

    # Contact sensor on arm links to penalize collisions
    arm_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/(link1_1|link2_1|link3_1|link4_1|LINK5_1|gripper_base_1)",
        update_period=0.0,
        history_length=1,
        track_air_time=False,
    )


# -----------------------------------------------------------------------------
# Actions Configuration
# -----------------------------------------------------------------------------
class ActionsCfg:
    """Arm delta position + 1-D coupled gripper action."""

    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=LEXIUM_ARM_JOINT_NAMES,
        scale=0.05,
        use_default_offset=True,
    )

    gripper_action = mdp.CoupledFourFingerGripperActionCfg(
        asset_name="robot",
        binary_mode=False,
    )


# -----------------------------------------------------------------------------
# Commands Configuration (Target placement generation)
# -----------------------------------------------------------------------------
class CommandsCfg:
    """Command manager for target placement location."""

    object_pose = mdp.UniformPoseCommandCfg(  # VERIFY
        asset_name="robot",
        body_name="base_link",
        resampling_time_range=(1e9, 1e9),  # Fixed per episode
        ranges=mdp.UniformPoseCommandCfg.Ranges(  # VERIFY
            pos_x=(0.25, 0.45),
            pos_y=(0.10, 0.30),
            pos_z=(0.025, 0.025),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
    )


# -----------------------------------------------------------------------------
# Observations Configuration
# -----------------------------------------------------------------------------
class ObservationsCfg:
    """Observations policy and value groups."""

    class PolicyCfg(ObservationGroupCfg):
        """Actor observation group."""

        arm_joint_pos = ObservationTermCfg(
            func=mdp.arm_joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_ARM_JOINT_NAMES)},
        )
        arm_joint_vel = ObservationTermCfg(
            func=mdp.arm_joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_ARM_JOINT_NAMES)},
        )
        gripper_state = ObservationTermCfg(
            func=mdp.gripper_aperture,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_GRIPPER_JOINT_NAMES)},
        )
        cube_pose_rel = ObservationTermCfg(
            func=mdp.cube_position_in_robot_frame,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "cube_cfg": SceneEntityCfg("cube"),
            },
        )
        target_pose_rel = ObservationTermCfg(
            func=mdp.target_position_in_robot_frame,
            params={"robot_cfg": SceneEntityCfg("robot")},
        )
        last_action = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# -----------------------------------------------------------------------------
# Events Configuration (Resets & Domain Randomization)
# -----------------------------------------------------------------------------
class EventsCfg:
    """Reset events and physics randomization."""

    reset_arm = EventTermCfg(
        func=mdp.reset_arm_joints_with_noise,
        mode="reset",
        params={
            "position_noise_range": (-0.05, 0.05),
            "velocity_noise_range": (-0.01, 0.01),
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_ARM_JOINT_NAMES),
        },
    )

    reset_cube = EventTermCfg(
        func=mdp.reset_cube_position_on_table,
        mode="reset",
        params={
            "x_range": (0.25, 0.45),
            "y_range": (-0.20, 0.0),
            "z_table": 0.025,
            "cube_cfg": SceneEntityCfg("cube"),
        },
    )

    randomize_friction = EventTermCfg(
        func=mdp.randomize_object_friction,
        mode="reset",
        params={
            "static_friction_range": (0.8, 1.5),
            "dynamic_friction_range": (0.7, 1.3),
            "asset_cfg": SceneEntityCfg("cube"),
        },
    )


# -----------------------------------------------------------------------------
# Rewards Configuration
# -----------------------------------------------------------------------------
class RewardsCfg:
    """Staged pick-and-place rewards and regularization penalties."""

    # 1. Staged Task Rewards
    reach = RewardTermCfg(
        func=mdp.reach_cube_reward,
        weight=2.0,
        params={"std": 0.10, "ee_frame_cfg": SceneEntityCfg("ee_frame"), "cube_cfg": SceneEntityCfg("cube")},
    )

    grasp = RewardTermCfg(
        func=mdp.grasp_cube_reward,
        weight=4.0,
        params={
            "distance_threshold": 0.035,
            "force_threshold": 0.5,
            "ee_frame_cfg": SceneEntityCfg("ee_frame"),
            "cube_cfg": SceneEntityCfg("cube"),
            "finger_contact_cfg": SceneEntityCfg("finger_contact_sensor"),
            "robot_cfg": SceneEntityCfg("robot"),
        },
    )

    lift = RewardTermCfg(
        func=mdp.lift_cube_reward,
        weight=10.0,
        params={
            "table_height": 0.0,
            "target_lift_height": 0.15,
            "lift_threshold": 0.02,
            "cube_cfg": SceneEntityCfg("cube"),
        },
    )

    transport = RewardTermCfg(
        func=mdp.transport_cube_reward,
        weight=15.0,
        params={
            "std": 0.15,
            "table_height": 0.0,
            "lift_threshold": 0.02,
            "cube_cfg": SceneEntityCfg("cube"),
        },
    )

    place = RewardTermCfg(
        func=mdp.place_cube_reward,
        weight=25.0,
        params={
            "dist_threshold": 0.03,
            "vel_threshold": 0.10,
            "cube_cfg": SceneEntityCfg("cube"),
            "robot_cfg": SceneEntityCfg("robot"),
        },
    )

    # 2. Regularization Penalties
    action_rate = RewardTermCfg(
        func=mdp.action_rate_l2,  # VERIFY
        weight=-1e-4,
    )

    joint_vel = RewardTermCfg(
        func=mdp.joint_vel_l2,  # VERIFY
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_ARM_JOINT_NAMES)},
    )

    joint_limit_proximity = RewardTermCfg(
        func=mdp.joint_limit_proximity_penalty,
        weight=-1.0,
        params={
            "margin_ratio": 0.05,
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEXIUM_ARM_JOINT_NAMES),
        },
    )

    collision_penalty = RewardTermCfg(
        func=mdp.arm_collision_penalty,
        weight=-5.0,
        params={
            "force_threshold": 1.0,
            "arm_contact_cfg": SceneEntityCfg("arm_contact_sensor"),
        },
    )


# -----------------------------------------------------------------------------
# Terminations Configuration
# -----------------------------------------------------------------------------
class TerminationsCfg:
    """Episode termination conditions."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)  # VERIFY

    cube_dropped = TerminationTermCfg(
        func=mdp.cube_dropped_off_table,
        params={"height_threshold": -0.05, "cube_cfg": SceneEntityCfg("cube")},
    )

    success = TerminationTermCfg(
        func=mdp.place_success_termination,
        params={
            "dist_threshold": 0.03,
            "vel_threshold": 0.05,
            "cube_cfg": SceneEntityCfg("cube"),
            "robot_cfg": SceneEntityCfg("robot"),
        },
    )


# -----------------------------------------------------------------------------
# Curriculum Configuration
# -----------------------------------------------------------------------------
class CurriculumCfg:
    """Curriculum increasing penalty weights as training progresses."""

    action_rate_penalty = CurriculumTermCfg(
        func=mdp.modify_reward_weight,  # VERIFY
        params={"term_name": "action_rate", "weight": -1e-2, "num_steps": 10000},
    )

    joint_vel_penalty = CurriculumTermCfg(
        func=mdp.modify_reward_weight,  # VERIFY
        params={"term_name": "joint_vel", "weight": -1e-2, "num_steps": 10000},
    )


# -----------------------------------------------------------------------------
# Top-level ManagerBasedRLEnvCfg
# -----------------------------------------------------------------------------
class LexiumPickPlaceEnvCfg(ManagerBasedRLEnvCfg):
    """Complete environment configuration for Lexium Pick-and-Place."""

    scene: LexiumSceneCfg = LexiumSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post-initialization configurations for PhysX and simulation loop."""
        self.decimation = 2  # Control frequency: 60Hz / 2 = 30Hz
        self.episode_length_s = 6.0

        # Simulation PhysX parameters matching Hard Constraint 4
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        self.sim.physx.solver_type = 1  # TGS
        self.sim.physx.max_position_iteration_count = 16
        self.sim.physx.min_position_iteration_count = 16
        self.sim.physx.max_velocity_iteration_count = 2
        self.sim.physx.min_velocity_iteration_count = 2
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.friction_offset_threshold = 0.04
        self.sim.physx.friction_correlation_distance = 0.025
        self.sim.physx.enable_stabilization = True
