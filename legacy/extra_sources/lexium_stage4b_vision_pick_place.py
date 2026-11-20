import asyncio
import importlib
import json
import math
import random
import sys
import time
import traceback

import cumotion
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
import warp as wp

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import isaacsim.robot_motion.experimental.motion_generation as mg
from isaacsim.core.experimental.prims import Articulation
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_robot,
)

# Ensure Script Editor (which runs from /tmp) can import lexium_stage4b_vision
VISION_MODULE_DIR = "/home/aybergen/Desktop/adi/lexium_cobot_2"
if VISION_MODULE_DIR not in sys.path:
    sys.path.insert(0, VISION_MODULE_DIR)

import lexium_stage4b_vision  # noqa: E402

importlib.reload(lexium_stage4b_vision)
from lexium_stage4b_vision import (  # noqa: E402
    RGBDCubeDetector,
    compute_diagnostic_accuracy_metrics,
    print_vision_diagnostic_report,
    save_vision_debug_artifacts,
)


# ==========================================================
# STAGE 4B CONFIGURABLE BENCHMARK PARAMETERS
# ==========================================================

# Phase 4 (Single Vision Pick & Place): set NUM_TRIALS = 1
# Phase 5 (Full 10-Trial Benchmark):    set NUM_TRIALS = 10
NUM_TRIALS = 10
RANDOM_SEED = 42

# Same verified randomization zone as Stage 4A
PICK_X_RANGE = (0.43, 0.48)
PICK_Y_RANGE = (0.05, 0.10)
CUBE_REST_Z = 0.025

REFERENCE_PICK = [
    0.45581603050231934,
    0.07331596314907074,
    0.025,
]

PLACE_POSITION = [
    0.35,
    0.25,
    0.028,
]

# Validation tolerances (meters)
GRASP_TOLERANCE = 0.035
SLIP_TOLERANCE = 0.035
MIN_LIFT_CUBE_Z = 0.120
PLACE_TOLERANCE = 0.020
FLOOR_Z_TOLERANCE = 0.015

MAX_SAMPLE_ATTEMPTS = 50
DRY_RUN = False

REPORT_JSON_PATH = "/home/aybergen/Desktop/adi/lexium_stage4b_report.json"


# ==========================================================
# SCENE & ROBOT PATHS
# ==========================================================

CONFIG_DIR = (
    "/home/aybergen/Desktop/adi/"
    "Lexium_Cobot_URDF/lexium_cobot_description"
)

ROBOT_PATH = "/lexium_cobot/Geometry/base_link"
PICK_CUBE_PATH = "/World/PickCube"
TARGET_PATH = "/World/TargetCube"
FLOOR_PATH = "/World/Floor"
TOOL_FRAME = "grasp_point"


# ==========================================================
# VERIFIED GRASP POINT & TOP-DOWN ORIENTATION (DO NOT CHANGE)
# ==========================================================

GRASP_POINT_LOCAL = [
    0.0242600831,
    0.0946719936,
    0.1380574717,
]

GRASP_ORIENTATION = [
    -0.234174,
    0.856810,
    0.416341,
    -0.194164,
]


# ==========================================================
# VERIFIED GRIPPER CONFIGURATION (DO NOT CHANGE)
# ==========================================================

ARM_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6_gripper_rotation",
]

GRIPPER_JOINTS = [
    "gripper_Revolute_1",
    "gripper_Revolute_2",
    "gripper_Revolute_5",
    "gripper_Revolute_6",
]

OPEN = [
    -0.550,
    -0.340,
    -0.315,
    -0.315,
]

CLOSE = [
    -0.200,
    0.010,
    0.035,
    0.035,
]

# 4 vertically aligned spheres per finger = 16 active pad spheres total
ACTIVE_TIP_SPHERE = {
    "gripper_finger_1_1": "collision_sphere_1",
    "gripper_finger_2_1": "collision_sphere_1",
    "gripper_finger_3_1": "collision_sphere_1",
    "gripper_finger_4_1": "collision_sphere_6",
}

EXTRA_FINGER_PAD_SPHERES = {
    "gripper_finger_1_1": [
        ("pad_sphere_1", 0.01385, (0.001464, 0.056315, 0.049946), (0.15, 0.85, 0.35)),
        ("pad_sphere_2", 0.01385, (0.002671, 0.049265, 0.042951), (0.15, 0.75, 0.85)),
        ("pad_sphere_3", 0.01385, (0.003959, 0.042179, 0.036002), (0.85, 0.45, 0.15)),
    ],
    "gripper_finger_2_1": [
        ("pad_sphere_1", 0.01385, (0.014157, 0.049084, 0.055316), (0.15, 0.85, 0.35)),
        ("pad_sphere_2", 0.01385, (0.013658, 0.042896, 0.047471), (0.15, 0.75, 0.85)),
        ("pad_sphere_3", 0.01385, (0.013242, 0.036658, 0.039656), (0.85, 0.45, 0.15)),
    ],
    "gripper_finger_3_1": [
        ("pad_sphere_1", 0.01385, (0.016220, 0.044620, 0.056924), (0.15, 0.85, 0.35)),
        ("pad_sphere_2", 0.01385, (0.014185, 0.039455, 0.048601), (0.15, 0.75, 0.85)),
        ("pad_sphere_3", 0.01385, (0.012070, 0.034346, 0.040259), (0.85, 0.45, 0.15)),
    ],
    "gripper_finger_4_1": [
        ("pad_sphere_1", 0.01385, (0.016221, 0.044621, 0.056924), (0.15, 0.85, 0.35)),
        ("pad_sphere_2", 0.01385, (0.014185, 0.039455, 0.048601), (0.15, 0.75, 0.85)),
        ("pad_sphere_3", 0.01385, (0.012071, 0.034347, 0.040259), (0.85, 0.45, 0.15)),
    ],
}


# ==========================================================
# MOTION PARAMETERS
# ==========================================================

APPROACH_HEIGHT = 0.12
LIFT_HEIGHT = 0.18


# ==========================================================
# CANCEL OLD TASKS
# ==========================================================

for _name in [
    "lexium_follow_task",
    "gripper_test_task",
    "grasp_task",
    "pick_place_task",
    "stage4a_task",
    "stage4b_camera_task",
    "stage4b_vision_task",
    "stage4b_task",
]:
    _old_task = globals().get(_name)
    if _old_task is not None:
        try:
            _old_task.cancel()
            print("Stopped old task:", _name)
        except Exception:
            pass


# ==========================================================
# STAGE
# ==========================================================

stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("No open USD stage")


# ==========================================================
# HELPERS
# ==========================================================

async def frames(count):
    for _ in range(count):
        await omni.kit.app.get_app().next_update_async()


def prim_at(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError("Prim not found: " + path)
    return prim


def world_position(prim):
    if not prim.IsValid():
        raise RuntimeError("Invalid USD prim")

    transform = (
        UsdGeom.Xformable(prim)
        .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    )
    t = transform.ExtractTranslation()
    return [float(t[0]), float(t[1]), float(t[2])]


def set_or_create_attr(prim, name, sdf_type, value):
    attr = prim.GetAttribute(name)
    if attr.IsValid():
        attr.Set(value)
    else:
        prim.CreateAttribute(name, sdf_type).Set(value)


def dist3d(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def dist2d(a, b):
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2
    )


def quat_wxyz_to_cumotion_rotation(wxyz):
    w, x, y, z = [float(v) for v in wxyz]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    angle = 2.0 * math.acos(min(1.0, max(-1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8:
        return cumotion.Rotation3.from_scaled_axis(np.zeros(3))
    axis = np.array([x / s, y / s, z / s], dtype=np.float64)
    return cumotion.Rotation3.from_scaled_axis(axis * angle)


# ==========================================================
# IK REACHABILITY & WORKSPACE SAFETY VALIDATION
# ==========================================================

def is_pick_candidate_valid(kinematics, candidate_xyz, orientation=GRASP_ORIENTATION):
    x, y, z = [float(v) for v in candidate_xyz]

    if z < 0.015 or z > 0.050:
        return False, f"Invalid Z height (z={z:.4f} m, expected ~0.025 m)"

    radial_xy = math.sqrt(x * x + y * y)
    if radial_xy < 0.20:
        return False, "Too close to robot base_link"

    if dist2d([x, y], PLACE_POSITION[:2]) < 0.08:
        return False, "Overlaps placement zone"

    rot = quat_wxyz_to_cumotion_rotation(orientation)
    ik_cfg = cumotion.IkConfig()

    test_heights = [
        ("PRE-GRASP", z + APPROACH_HEIGHT),
        ("GRASP", z),
        ("LIFT", z + LIFT_HEIGHT),
    ]

    for stage_name, target_z in test_heights:
        pose = cumotion.Pose3(
            rot,
            np.array([x, y, target_z], dtype=np.float64),
        )
        res = cumotion.solve_ik(kinematics, pose, TOOL_FRAME, ik_cfg)
        if not res.success:
            return False, f"IK unreachable at {stage_name} (z={target_z:.3f} m)"

    return True, "OK"


def sample_reachable_pick_position(rng, kinematics):
    for attempt in range(1, MAX_SAMPLE_ATTEMPTS + 1):
        px = round(rng.uniform(PICK_X_RANGE[0], PICK_X_RANGE[1]), 4)
        py = round(rng.uniform(PICK_Y_RANGE[0], PICK_Y_RANGE[1]), 4)
        candidate = [px, py, CUBE_REST_Z]
        valid, reason = is_pick_candidate_valid(kinematics, candidate)
        if valid:
            return candidate, True, attempt, reason
        print(
            f"  [Sampler] Rejected candidate {candidate} (attempt {attempt}): {reason}"
        )

    return candidate, False, MAX_SAMPLE_ATTEMPTS, reason


# ==========================================================
# CONFIGURE PHYSICS (VERIFIED STAGE 4A GRASP PARAMETERS)
# ==========================================================

def configure_grasp_physics():
    # 1. Hide visual /World/TargetCube
    target_prim = stage.GetPrimAtPath(TARGET_PATH)
    if target_prim.IsValid():
        UsdGeom.Imageable(target_prim).MakeInvisible()

    # 2. Disable articulation self-collisions and increase solver iterations
    base_link_prim = stage.GetPrimAtPath(ROBOT_PATH)
    if base_link_prim.IsValid():
        set_or_create_attr(
            base_link_prim,
            "physxArticulation:enabledSelfCollisions",
            Sdf.ValueTypeNames.Bool,
            False,
        )
        set_or_create_attr(
            base_link_prim,
            "physxArticulation:solverPositionIterationCount",
            Sdf.ValueTypeNames.Int,
            128,
        )
        set_or_create_attr(
            base_link_prim,
            "physxArticulation:solverVelocityIterationCount",
            Sdf.ValueTypeNames.Int,
            16,
        )

    # 3. Configure PickCube mass, solver iterations & maxDepenetrationVelocity
    cube_prim = stage.GetPrimAtPath(PICK_CUBE_PATH)
    if cube_prim.IsValid():
        set_or_create_attr(
            cube_prim,
            "physics:mass",
            Sdf.ValueTypeNames.Float,
            0.025,
        )
        set_or_create_attr(
            cube_prim,
            "physxRigidBody:maxDepenetrationVelocity",
            Sdf.ValueTypeNames.Float,
            0.2,
        )
        set_or_create_attr(
            cube_prim,
            "physxRigidBody:solverPositionIterationCount",
            Sdf.ValueTypeNames.Int,
            128,
        )
        set_or_create_attr(
            cube_prim,
            "physxRigidBody:solverVelocityIterationCount",
            Sdf.ValueTypeNames.Int,
            16,
        )

    # 4. High friction + "multiply" combine mode on both PickCube & gripper materials
    for mat_path in [
        "/World/Physics_Materials/physics_material",
        "/lexium_cobot/grip",
    ]:
        mat_prim = stage.GetPrimAtPath(mat_path)
        if mat_prim.IsValid():
            UsdPhysics.MaterialAPI.Apply(mat_prim)
            set_or_create_attr(
                mat_prim,
                "physics:staticFriction",
                Sdf.ValueTypeNames.Float,
                10.0,
            )
            set_or_create_attr(
                mat_prim,
                "physics:dynamicFriction",
                Sdf.ValueTypeNames.Float,
                10.0,
            )
            set_or_create_attr(
                mat_prim,
                "physics:restitution",
                Sdf.ValueTypeNames.Float,
                0.0,
            )
            set_or_create_attr(
                mat_prim,
                "physxMaterial:frictionCombineMode",
                Sdf.ValueTypeNames.Token,
                "multiply",
            )
            set_or_create_attr(
                mat_prim,
                "physxMaterial:restitutionCombineMode",
                Sdf.ValueTypeNames.Token,
                "min",
            )

    grip_mat_prim = stage.GetPrimAtPath("/lexium_cobot/grip")
    grip_mat = (
        UsdShade.Material(grip_mat_prim) if grip_mat_prim.IsValid() else None
    )

    # 5. Fix PickCube and Floor contactOffset & realistic torsional patch radius
    for p_path in [PICK_CUBE_PATH, FLOOR_PATH]:
        p = stage.GetPrimAtPath(p_path)
        if p.IsValid():
            set_or_create_attr(
                p,
                "physxCollision:contactOffset",
                Sdf.ValueTypeNames.Float,
                0.004,
            )
            set_or_create_attr(
                p,
                "physxCollision:restOffset",
                Sdf.ValueTypeNames.Float,
                0.0,
            )
            set_or_create_attr(
                p,
                "physxCollision:torsionalPatchRadius",
                Sdf.ValueTypeNames.Float,
                0.02,
            )
            set_or_create_attr(
                p,
                "physxCollision:minTorsionalPatchRadius",
                Sdf.ValueTypeNames.Float,
                0.01,
            )

    # 6. Create 3 additional pad spheres along the middle of each finger
    #    and enable PhysicsCollisionAPI ONLY on the 4 aligned column spheres
    #    per finger (16 spheres total)
    base_path = (
        "/lexium_cobot/Geometry/base_link/link1_1/link2_1/"
        "link3_1/link4_1/LINK5_1/gripper_base_1"
    )

    total_active_spheres = 0
    for fn, extra_list in EXTRA_FINGER_PAD_SPHERES.items():
        fn_path = f"{base_path}/{fn}"
        fn_prim = stage.GetPrimAtPath(fn_path)
        if not fn_prim.IsValid():
            continue

        if grip_mat is not None:
            mb_fn = UsdShade.MaterialBindingAPI.Apply(fn_prim)
            mb_fn.Bind(
                grip_mat,
                UsdShade.Tokens.strongerThanDescendants,
                "physics",
            )

        allowed_names = {ACTIVE_TIP_SPHERE[fn]}
        for sp_name, radius, local_pos, color in extra_list:
            allowed_names.add(sp_name)
            sp_path = f"{fn_path}/{sp_name}"
            sp_prim = stage.GetPrimAtPath(sp_path)
            if not sp_prim.IsValid():
                sphere_geom = UsdGeom.Sphere.Define(stage, sp_path)
                sphere_geom.GetRadiusAttr().Set(radius)
                sphere_geom.GetExtentAttr().Set(
                    [
                        Gf.Vec3f(-radius, -radius, -radius),
                        Gf.Vec3f(radius, radius, radius),
                    ]
                )
                sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
                xform = UsdGeom.Xformable(sphere_geom.GetPrim())
                xform.AddTranslateOp().Set(Gf.Vec3d(*local_pos))
                sp_prim = sphere_geom.GetPrim()
            else:
                t_op = sp_prim.GetAttribute("xformOp:translate")
                if t_op.IsValid():
                    t_op.Set(Gf.Vec3d(*local_pos))

        for p in Usd.PrimRange(fn_prim):
            if p.IsA(UsdGeom.Mesh):
                col_attr = p.GetAttribute("physics:collisionEnabled")
                if col_attr.IsValid():
                    col_attr.Set(False)
            elif p.IsA(UsdGeom.Sphere):
                # Ensure finger_1 collision spheres are cyan so only PickCube is red
                if fn == "gripper_finger_1_1" and p.GetName().startswith("collision_sphere_"):
                    UsdGeom.Gprim(p).GetDisplayColorAttr().Set(
                        [Gf.Vec3f(0.15, 0.75, 0.85)]
                    )
                if p.GetName() in allowed_names:
                    UsdPhysics.CollisionAPI.Apply(p)
                    set_or_create_attr(
                        p,
                        "physics:collisionEnabled",
                        Sdf.ValueTypeNames.Bool,
                        True,
                    )
                    set_or_create_attr(
                        p,
                        "physxCollision:contactOffset",
                        Sdf.ValueTypeNames.Float,
                        0.004,
                    )
                    set_or_create_attr(
                        p,
                        "physxCollision:restOffset",
                        Sdf.ValueTypeNames.Float,
                        0.0,
                    )
                    set_or_create_attr(
                        p,
                        "physxCollision:torsionalPatchRadius",
                        Sdf.ValueTypeNames.Float,
                        0.02,
                    )
                    set_or_create_attr(
                        p,
                        "physxCollision:minTorsionalPatchRadius",
                        Sdf.ValueTypeNames.Float,
                        0.01,
                    )
                    if grip_mat is not None:
                        mb = UsdShade.MaterialBindingAPI.Apply(p)
                        mb.Bind(
                            grip_mat,
                            UsdShade.Tokens.strongerThanDescendants,
                            "physics",
                        )
                        mb.Bind(
                            grip_mat,
                            UsdShade.Tokens.strongerThanDescendants,
                        )
                    total_active_spheres += 1
                else:
                    col_attr = p.GetAttribute("physics:collisionEnabled")
                    if col_attr.IsValid():
                        col_attr.Set(False)

    print(
        f"Configured {total_active_spheres} aligned high-friction pad spheres (4 per finger)."
    )

    # 7. Restore original stable USD arm joint stiffness/damping
    original_arm_gains = {
        "joint_1": (10.0, 1.0),
        "joint_2": (12.0, 1.2),
        "joint_3": (10.0, 1.0),
        "joint_4": (5.0, 0.5),
        "joint_5": (6.0, 0.6),
        "joint_6_gripper_rotation": (5.0, 0.5),
    }
    for jn, (stiff_val, damp_val) in original_arm_gains.items():
        jp = stage.GetPrimAtPath(f"/lexium_cobot/Physics/{jn}")
        if jp.IsValid():
            stiff = jp.GetAttribute("drive:angular:physics:stiffness")
            damp = jp.GetAttribute("drive:angular:physics:damping")
            if stiff.IsValid():
                stiff.Set(stiff_val)
            if damp.IsValid():
                damp.Set(damp_val)

    # 8. Compliant, well-damped finger stiffness (10.0 / 2.0)
    for jn in GRIPPER_JOINTS:
        jp = stage.GetPrimAtPath(f"/lexium_cobot/Physics/{jn}")
        if jp.IsValid():
            set_or_create_attr(
                jp,
                "drive:angular:physics:stiffness",
                Sdf.ValueTypeNames.Float,
                10.0,
            )
            set_or_create_attr(
                jp,
                "drive:angular:physics:damping",
                Sdf.ValueTypeNames.Float,
                2.0,
            )
            set_or_create_attr(
                jp,
                "drive:angular:physics:maxForce",
                Sdf.ValueTypeNames.Float,
                60.0,
            )


def reset_trial_scene_state(spawn_position):
    """Safely reset PickCube and robot joint states while the timeline is stopped."""
    cube_prim = prim_at(PICK_CUBE_PATH)

    t_attr = cube_prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(*[float(v) for v in spawn_position]))

    r_attr = cube_prim.GetAttribute("xformOp:orient")
    if r_attr.IsValid():
        r_attr.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

    set_or_create_attr(
        cube_prim,
        "physics:velocity",
        Sdf.ValueTypeNames.Vector3f,
        Gf.Vec3f(0.0, 0.0, 0.0),
    )
    set_or_create_attr(
        cube_prim,
        "physics:angularVelocity",
        Sdf.ValueTypeNames.Vector3f,
        Gf.Vec3f(0.0, 0.0, 0.0),
    )

    for jn in ARM_JOINTS:
        jp = stage.GetPrimAtPath(f"/lexium_cobot/Physics/{jn}")
        if jp.IsValid():
            for attr_name in [
                "state:angular:physics:position",
                "state:angular:physics:velocity",
                "drive:angular:physics:targetPosition",
            ]:
                a = jp.GetAttribute(attr_name)
                if a.IsValid():
                    a.Set(0.0)

    for idx, jn in enumerate(GRIPPER_JOINTS):
        jp = stage.GetPrimAtPath(f"/lexium_cobot/Physics/{jn}")
        if jp.IsValid():
            open_deg = float(math.degrees(OPEN[idx]))
            pos_a = jp.GetAttribute("state:angular:physics:position")
            if pos_a.IsValid():
                pos_a.Set(open_deg)
            vel_a = jp.GetAttribute("state:angular:physics:velocity")
            if vel_a.IsValid():
                vel_a.Set(0.0)
            tgt_a = jp.GetAttribute("drive:angular:physics:targetPosition")
            if tgt_a.IsValid():
                tgt_a.Set(open_deg)


# ==========================================================
# VISUAL GRASP POINT
# ==========================================================

def visual_grasp_point():
    base = next(
        (
            p for p in stage.Traverse()
            if p.GetName() == "gripper_base_1"
            and p.IsA(UsdGeom.Xform)
            and str(p.GetPath()).startswith("/lexium_cobot/")
        ),
        None,
    )

    if base is None:
        raise RuntimeError("gripper_base_1 Xform not found")

    point_path = base.GetPath().AppendChild("GraspPoint")
    point = stage.GetPrimAtPath(point_path)

    if not point.IsValid():
        xform = UsdGeom.Xform.Define(stage, point_path)
        xform.AddTranslateOp().Set(Gf.Vec3d(*GRASP_POINT_LOCAL))
        point = xform.GetPrim()

    marker_path = point_path.AppendChild("Marker")
    marker_prim = stage.GetPrimAtPath(marker_path)
    if marker_prim.IsValid():
        UsdGeom.Imageable(marker_prim).MakeInvisible()

    return point


# ==========================================================
# ROBOT STATE HELPERS
# ==========================================================

def current_state(robot):
    names = robot.dof_names
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=names,
            positions=(names, robot.get_dof_positions()),
            velocities=(names, robot.get_dof_velocities()),
        )
    )


def desired_state(position, orientation=GRASP_ORIENTATION):
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=[TOOL_FRAME],
            positions=(
                [TOOL_FRAME],
                wp.array([position], dtype=wp.float32),
            ),
            orientations=(
                [TOOL_FRAME],
                wp.array([orientation], dtype=wp.float32),
            ),
        )
    )


# ==========================================================
# SINGLE VISION-GUIDED TRIAL EXECUTION
# ==========================================================

async def execute_vision_trial(
    trial_num,
    spawn_pos,
    cumotion_config,
    kinematics,
    camera_detector: RGBDCubeDetector,
    grasp_prim,
    cube_prim,
):
    timeline = omni.timeline.get_timeline_interface()

    record = {
        "trial": trial_num,
        "random_seed": RANDOM_SEED,
        "spawn_xyz": [round(x, 4) for x in spawn_pos],
        "detection_success": False,
        "pixel_center": None,
        "bounding_box_xywh": None,
        "depth_m": None,
        "camera_coordinates_xyz": None,
        "estimated_world_xyz": None,
        "ground_truth_xyz": None,
        "position_error_x_mm": None,
        "position_error_y_mm": None,
        "position_error_z_mm": None,
        "position_error_mm": None,
        "grasp_success": False,
        "lift_success": False,
        "transport_success": False,
        "place_success": False,
        "grasp_error_mm": None,
        "placement_error_mm": None,
        "final_cube_xyz": None,
        "stage_timings_sec": {},
        "status": "FAILED",
        "failure_reason": None,
    }

    t_trial_start = time.perf_counter()

    # 1. Stop timeline cleanly before resetting USD transforms for the trial
    if timeline.is_playing():
        timeline.stop()
        await frames(15)

    reset_trial_scene_state(spawn_pos)
    await frames(10)

    # 2. Start timeline and wait for PhysX tensor entity & cube settlement
    timeline.play()
    await frames(45)

    robot = Articulation(ROBOT_PATH)
    for _ in range(120):
        if robot.is_physics_tensor_entity_valid():
            break
        await omni.kit.app.get_app().next_update_async()

    if not robot.is_physics_tensor_entity_valid():
        record["failure_reason"] = "Physics tensor entity did not initialize after Play"
        return record

    indices = [robot.dof_names.index(name) for name in GRIPPER_JOINTS]

    world = CumotionWorldInterface()
    world.update_world_to_robot_root_transforms(robot.get_world_poses())

    controller = RmpFlowController(
        cumotion_robot=cumotion_config,
        cumotion_world_interface=world,
        robot_joint_space=robot.dof_names,
        robot_site_space=[TOOL_FRAME],
        tool_frame=TOOL_FRAME,
        rmp_flow_configuration_filename="rmp_flow.yaml",
    )
    controller.get_rmp_flow_config().set_param(
        "cspace_target_rmp/metric_scalar",
        1.0,
    )

    gripper_target = OPEN.copy()
    last_arm_positions = None
    last_arm_indices = None

    async def command_gripper(target, name, ramp_frames=90, hold_frames=60):
        nonlocal gripper_target
        t0 = time.perf_counter()
        start_target = gripper_target.copy()

        print(f"  >>> GRIPPER {name} target={target}")

        total_frames = ramp_frames + hold_frames
        for frame in range(total_frames):
            if not timeline.is_playing() or not robot.is_physics_tensor_entity_valid():
                raise asyncio.CancelledError("Timeline stopped")

            if frame < ramp_frames:
                alpha = float(frame + 1) / float(ramp_frames)
                gripper_target = [
                    start_target[i] + alpha * (target[i] - start_target[i])
                    for i in range(4)
                ]
            else:
                gripper_target = target.copy()

            if last_arm_positions is not None and last_arm_indices is not None:
                robot.set_dof_position_targets(
                    positions=last_arm_positions,
                    dof_indices=last_arm_indices,
                )

            robot.set_dof_position_targets(
                positions=gripper_target,
                dof_indices=indices,
            )
            await omni.kit.app.get_app().next_update_async()

        record["stage_timings_sec"][f"gripper_{name.lower()}"] = round(
            time.perf_counter() - t0, 3
        )

    async def move_arm(
        position,
        label,
        timeout_frames=1200,
        tolerance=0.02,
        settle_frames=5,
        check_cube_held=False,
        expected_grasp_offset=None,
    ):
        nonlocal last_arm_positions, last_arm_indices
        t0 = time.perf_counter()

        if (
            any(abs(float(position[i])) > 2 for i in range(2))
            or not (0 <= position[2] <= 2)
        ):
            raise RuntimeError("Unsafe target: " + str(position))

        print(
            f"  >>> MOVE: {label} | TARGET: {[round(x, 4) for x in position]} "
            f"| TOL: {tolerance}"
        )

        target = desired_state(position)
        world.update_world_to_robot_root_transforms(robot.get_world_poses())

        t = 0.0
        ok = controller.reset(
            current_state(robot),
            target,
            t,
        )
        if not ok:
            raise RuntimeError(f"{label}: RMPflow reset failed")

        best = float("inf")
        count_close = 0
        dt_sub = 1.0 / 240.0

        for frame in range(timeout_frames):
            await omni.kit.app.get_app().next_update_async()

            if not timeline.is_playing() or not robot.is_physics_tensor_entity_valid():
                raise asyncio.CancelledError("Timeline stopped")

            world.update_world_to_robot_root_transforms(
                robot.get_world_poses()
            )
            est = current_state(robot)

            # 4 substeps at 240 Hz = 1/60 s per frame
            command = None
            for _ in range(4):
                t += dt_sub
                command = controller.forward(est, target, t)

            if (
                command is not None
                and command.joints is not None
                and command.joints.positions is not None
            ):
                last_arm_positions = command.joints.positions.numpy().copy()
                last_arm_indices = command.joints.position_indices

                robot.set_dof_position_targets(
                    positions=last_arm_positions,
                    dof_indices=last_arm_indices,
                )

            robot.set_dof_position_targets(
                positions=gripper_target,
                dof_indices=indices,
            )

            if frame % 6 == 0:
                actual = world_position(grasp_prim)
                cube_now = world_position(cube_prim)
                distance = dist3d(actual, position)
                best = min(best, distance)

                if frame % 120 == 0:
                    print(
                        f"    POINT: {[round(x, 4) for x in actual]} "
                        f"| CUBE: {[round(x, 4) for x in cube_now]} "
                        f"| DIST: {round(distance, 4)} m"
                    )

                if check_cube_held and expected_grasp_offset is not None and frame > 30:
                    rel_now = [cube_now[i] - actual[i] for i in range(3)]
                    slip_dist = dist3d(rel_now, expected_grasp_offset)
                    if slip_dist > SLIP_TOLERANCE:
                        record["stage_timings_sec"][label] = round(
                            time.perf_counter() - t0, 3
                        )
                        return (
                            False,
                            f"GRASP FAILED during {label}: cube slipped by {slip_dist * 1000:.1f} mm",
                        )

                if distance <= tolerance:
                    count_close += 1
                else:
                    count_close = 0

                if count_close >= settle_frames:
                    record["stage_timings_sec"][label] = round(
                        time.perf_counter() - t0, 3
                    )
                    print(
                        f"  REACHED: {label} (dist={distance:.4f} m, "
                        f"cube={[round(x, 4) for x in cube_now]})"
                    )
                    return True, "OK"

        record["stage_timings_sec"][label] = round(time.perf_counter() - t0, 3)
        return (
            False,
            f"TARGET NOT REACHED ({label}): best distance {best:.4f} m > {tolerance} m",
        )

    # ==========================================================
    # 3. STREAM A (PERCEPTION -> CONTROL): RGB-D CAMERA DETECTION
    # ==========================================================
    t_vis = time.perf_counter()
    detection = await camera_detector.detect(warmup_frames=25, save_images=True)
    record["stage_timings_sec"]["vision_detection"] = round(
        time.perf_counter() - t_vis, 3
    )

    # Save per-trial debug images on trial 1 (and always update main debug files)
    save_vision_debug_artifacts(detection)

    # ==========================================================
    # 4. STREAM B (DIAGNOSTICS ONLY): GROUND TRUTH COMPARISON
    # ==========================================================
    gt_xyz = world_position(cube_prim)
    record["ground_truth_xyz"] = [round(v, 5) for v in gt_xyz]

    if not detection.success:
        print_vision_diagnostic_report(detection, gt_xyz=gt_xyz)
        record["detection_success"] = False
        record["failure_reason"] = (
            f"Object detection failed: {detection.failure_reason}"
        )
        return record

    record["detection_success"] = True
    record["pixel_center"] = [int(detection.pixel_u), int(detection.pixel_v)]
    record["bounding_box_xywh"] = list(detection.bbox_xywh)
    record["depth_m"] = float(detection.depth_m)
    record["camera_coordinates_xyz"] = list(detection.camera_coordinates_xyz)
    record["estimated_world_xyz"] = list(detection.world_position)

    diag_metrics = compute_diagnostic_accuracy_metrics(
        detection.world_position,
        gt_xyz,
    )
    record["position_error_x_mm"] = diag_metrics["error_x_mm"]
    record["position_error_y_mm"] = diag_metrics["error_y_mm"]
    record["position_error_z_mm"] = diag_metrics["error_z_mm"]
    record["position_error_mm"] = diag_metrics["error_3d_mm"]

    print_vision_diagnostic_report(detection, gt_xyz=gt_xyz)

    # STRICT REQUIREMENT: RMPflow receives coordinates EXCLUSIVELY from camera vision!
    vision_pick_xyz = list(detection.world_position)

    # Verify IK reachability for the camera-estimated coordinates before moving
    valid_ik, ik_reason = is_pick_candidate_valid(kinematics, vision_pick_xyz)
    if not valid_ik:
        record["failure_reason"] = (
            f"Camera-estimated position {vision_pick_xyz} failed safety/IK check: {ik_reason}"
        )
        return record

    # 5. Calculate dynamic waypoints EXCLUSIVELY from camera-estimated XYZ
    approach = [
        vision_pick_xyz[0],
        vision_pick_xyz[1],
        vision_pick_xyz[2] + APPROACH_HEIGHT,
    ]
    grasp_target = [
        vision_pick_xyz[0],
        vision_pick_xyz[1],
        vision_pick_xyz[2],
    ]
    lift = [
        vision_pick_xyz[0],
        vision_pick_xyz[1],
        vision_pick_xyz[2] + LIFT_HEIGHT,
    ]
    place_above = [
        PLACE_POSITION[0],
        PLACE_POSITION[1],
        PLACE_POSITION[2] + LIFT_HEIGHT,
    ]

    # 6. Open gripper
    await command_gripper(OPEN, "OPEN", ramp_frames=30, hold_frames=30)

    # 7. Move to PRE-GRASP (guided by vision_pick_xyz)
    ok, msg = await move_arm(
        approach,
        "1_PRE_GRASP",
        timeout_frames=1200,
        tolerance=0.025,
        settle_frames=5,
    )
    if not ok:
        record["failure_reason"] = msg
        return record

    # 8. Move to GRASP (guided by vision_pick_xyz)
    ok, msg = await move_arm(
        grasp_target,
        "2_GRASP",
        timeout_frames=1200,
        tolerance=0.010,
        settle_frames=6,
    )
    if not ok:
        record["failure_reason"] = msg
        return record

    if DRY_RUN:
        record["status"] = "SUCCESS"
        return record

    # 9. Close gripper onto cube
    await command_gripper(CLOSE, "CLOSE", ramp_frames=90, hold_frames=75)

    # 10. Validate initial grasp acquisition
    gp_after_close = world_position(grasp_prim)
    cube_after_close = world_position(cube_prim)
    grasp_error = dist3d(gp_after_close, cube_after_close)
    record["grasp_error_mm"] = round(grasp_error * 1000.0, 2)
    initial_grasp_offset = [
        cube_after_close[i] - gp_after_close[i] for i in range(3)
    ]

    if grasp_error > GRASP_TOLERANCE:
        print("  GRASP FAILED: cube displaced during closure")
        record["failure_reason"] = (
            f"GRASP FAILED at closure: grasp_error={grasp_error * 1000:.1f} mm > "
            f"{GRASP_TOLERANCE * 1000:.1f} mm"
        )
        return record

    record["grasp_success"] = True

    # 11. Lift cube and validate physical lift
    ok, msg = await move_arm(
        lift,
        "3_LIFT",
        timeout_frames=1200,
        tolerance=0.020,
        settle_frames=5,
        check_cube_held=True,
        expected_grasp_offset=initial_grasp_offset,
    )
    gp_after_lift = world_position(grasp_prim)
    cube_after_lift = world_position(cube_prim)
    lift_displacement = cube_after_lift[2] - CUBE_REST_Z

    if (
        not ok
        or cube_after_lift[2] < MIN_LIFT_CUBE_Z
        or dist3d(gp_after_lift, cube_after_lift) > GRASP_TOLERANCE
    ):
        print("  GRASP FAILED: cube slipped or failed to lift")
        record["failure_reason"] = (
            msg
            if not ok
            else (
                f"GRASP FAILED after lift: cube_z={cube_after_lift[2]:.4f} m "
                f"(lifted {lift_displacement * 1000:.1f} mm)"
            )
        )
        return record

    record["lift_success"] = True

    # 12. Transport cube to placement zone and verify cube stays held
    ok, msg = await move_arm(
        place_above,
        "4_TRANSPORT",
        timeout_frames=1500,
        tolerance=0.025,
        settle_frames=5,
        check_cube_held=True,
        expected_grasp_offset=initial_grasp_offset,
    )
    gp_after_transport = world_position(grasp_prim)
    cube_after_transport = world_position(cube_prim)

    if (
        not ok
        or cube_after_transport[2] < MIN_LIFT_CUBE_Z
        or dist3d(gp_after_transport, cube_after_transport) > GRASP_TOLERANCE
    ):
        print("  GRASP FAILED: cube lost during transport")
        record["failure_reason"] = (
            msg
            if not ok
            else f"GRASP FAILED at transport end: cube_z={cube_after_transport[2]:.4f} m"
        )
        return record

    record["transport_success"] = True

    # 13. Lower cube to PLACE_POSITION
    ok, msg = await move_arm(
        PLACE_POSITION,
        "5_LOWER",
        timeout_frames=1200,
        tolerance=0.015,
        settle_frames=6,
    )
    if not ok:
        record["failure_reason"] = msg
        return record

    # 14. Release cube and let it settle on the floor
    await command_gripper(OPEN, "RELEASE", ramp_frames=60, hold_frames=40)
    await frames(20)

    # 15. Retreat safely above placement zone
    ok, msg = await move_arm(
        place_above,
        "6_RETREAT",
        timeout_frames=1200,
        tolerance=0.025,
        settle_frames=5,
    )
    if not ok:
        record["failure_reason"] = msg
        return record

    await frames(20)

    # 16. Verify final placement on floor
    final_cube = world_position(cube_prim)
    final_gp = world_position(grasp_prim)
    record["final_cube_xyz"] = [round(x, 4) for x in final_cube]

    xy_error = dist2d(final_cube, PLACE_POSITION)
    record["placement_error_mm"] = round(xy_error * 1000.0, 2)
    record["stage_timings_sec"]["total_trial"] = round(
        time.perf_counter() - t_trial_start, 3
    )

    on_floor = abs(final_cube[2] - CUBE_REST_Z) <= FLOOR_Z_TOLERANCE
    released_cleanly = dist3d(final_gp, final_cube) >= 0.10

    if not on_floor:
        record["failure_reason"] = (
            f"Cube not resting on floor after release (z={final_cube[2]:.4f} m)"
        )
        return record

    if not released_cleanly:
        record["failure_reason"] = "Gripper did not separate cleanly from cube"
        return record

    if xy_error > PLACE_TOLERANCE:
        record["failure_reason"] = (
            f"Placement XY error {xy_error * 1000:.1f} mm exceeds tolerance "
            f"{PLACE_TOLERANCE * 1000:.1f} mm"
        )
        return record

    record["place_success"] = True
    record["status"] = "SUCCESS"
    return record


# ==========================================================
# STAGE 4B BENCHMARK RUNNER & REPORT GENERATOR
# ==========================================================

def print_and_save_stage4b_report(results):
    total_trials = len(results)
    detection_success_count = sum(1 for r in results if r["detection_success"])
    detection_failures = total_trials - detection_success_count

    pos_errors = [
        r["position_error_mm"]
        for r in results
        if r["position_error_mm"] is not None
    ]
    avg_3d_error_mm = float(np.mean(pos_errors)) if pos_errors else 0.0
    max_3d_error_mm = float(np.max(pos_errors)) if pos_errors else 0.0

    grasp_success_count = sum(1 for r in results if r["grasp_success"])
    pick_place_success_count = sum(1 for r in results if r["status"] == "SUCCESS")
    success_rate = (
        (100.0 * pick_place_success_count / total_trials)
        if total_trials
        else 0.0
    )

    place_errors = [
        r["placement_error_mm"]
        for r in results
        if r["placement_error_mm"] is not None
    ]
    avg_place_error_mm = float(np.mean(place_errors)) if place_errors else 0.0

    print("")
    print("=======================================")
    print("LEXIUM STAGE 4B — VISION BENCHMARK")
    print("=======================================")
    print("")
    for r in results:
        print(
            f"Trial {r['trial']}: STATUS={r['status']} "
            f"| Det={r['detection_success']} "
            f"| Pixel={r['pixel_center']} "
            f"| Depth={r['depth_m']} m "
            f"| EstXYZ={r['estimated_world_xyz']} "
            f"| GT_XYZ={r['ground_truth_xyz']} "
            f"| 3D_Err={r['position_error_mm']} mm "
            f"| Place_Err={r['placement_error_mm']} mm"
        )
        if r["failure_reason"]:
            print(f"  Failure Reason: {r['failure_reason']}")
    print("")
    print("=======================================")
    print("LEXIUM STAGE 4B — VISION BENCHMARK")
    print("=======================================")
    print("")
    print(f"TOTAL TRIALS: {total_trials}")
    print("")
    print(f"DETECTION SUCCESS: {detection_success_count}")
    print(f"DETECTION FAILURES: {detection_failures}")
    print("")
    print(f"AVERAGE 3D ESTIMATION ERROR: {avg_3d_error_mm:.2f} mm")
    print(f"MAXIMUM 3D ESTIMATION ERROR: {max_3d_error_mm:.2f} mm")
    print("")
    print(f"GRASP SUCCESS: {grasp_success_count} / {total_trials}")
    print(f"PICK & PLACE SUCCESS: {pick_place_success_count} / {total_trials}")
    print("")
    print(f"SUCCESS RATE: {success_rate:.1f} %")
    print("")
    print("=======================================")

    report_payload = {
        "benchmark": "Lexium Cobot Stage 4B - Autonomous Vision-Guided Pick & Place",
        "num_trials": total_trials,
        "detection_success_count": detection_success_count,
        "detection_failure_count": detection_failures,
        "average_3d_estimation_error_mm": round(avg_3d_error_mm, 2),
        "maximum_3d_estimation_error_mm": round(max_3d_error_mm, 2),
        "grasp_success_count": grasp_success_count,
        "pick_place_success_count": pick_place_success_count,
        "success_rate_percent": round(success_rate, 2),
        "average_placement_error_mm": round(avg_place_error_mm, 2),
        "random_seed": RANDOM_SEED,
        "pick_x_range": list(PICK_X_RANGE),
        "pick_y_range": list(PICK_Y_RANGE),
        "place_position": PLACE_POSITION,
        "trials": results,
    }

    with open(REPORT_JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(report_payload, f, indent=2, ensure_ascii=False)

    print(f"Saved Stage 4B JSON report to: {REPORT_JSON_PATH}")


async def run_stage4b_vision_benchmark():
    print("")
    print("==========================================================")
    print("LEXIUM COBOT STAGE 4B — VISION-GUIDED PICK & PLACE")
    print(f"NUM_TRIALS={NUM_TRIALS} | SEED={RANDOM_SEED} | X={PICK_X_RANGE} | Y={PICK_Y_RANGE}")
    print("==========================================================")

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
        await frames(15)

    configure_grasp_physics()
    grasp_prim = visual_grasp_point()
    cube_prim = prim_at(PICK_CUBE_PATH)

    camera_detector = RGBDCubeDetector(stage)

    cumotion_config = load_cumotion_robot(
        directory=CONFIG_DIR,
        urdf_filename="lexium_cobot.urdf",
        xrdf_filename="lexium_cobot.xrdf",
    )

    frames_available = cumotion_config.robot_description.tool_frame_names()
    if TOOL_FRAME not in frames_available:
        raise RuntimeError("grasp_point missing from XRDF")

    kinematics = cumotion_config.robot_description.kinematics()
    rng = random.Random(RANDOM_SEED)

    results = []

    for trial_num in range(1, NUM_TRIALS + 1):
        print("")
        print("----------------------------------------------------------")
        print(f"STARTING VISION TRIAL {trial_num} / {NUM_TRIALS}")
        print("----------------------------------------------------------")

        if NUM_TRIALS == 1:
            # Phase 4 single-trial test uses REFERENCE_PICK or current valid cube position
            candidate_pos = list(REFERENCE_PICK)
            print(f"  [Single Trial] Spawn cube at reference position: {candidate_pos}")
        else:
            candidate_pos, reachable, attempts, reason = (
                sample_reachable_pick_position(rng, kinematics)
            )
            print(
                f"  [Sampler] Spawned cube at {candidate_pos} "
                f"(reachable={reachable}, attempts={attempts})"
            )

        trial_result = await execute_vision_trial(
            trial_num=trial_num,
            spawn_pos=candidate_pos,
            cumotion_config=cumotion_config,
            kinematics=kinematics,
            camera_detector=camera_detector,
            grasp_prim=grasp_prim,
            cube_prim=cube_prim,
        )
        results.append(trial_result)

        print(
            f"  [Trial {trial_num} Summary] STATUS={trial_result['status']} "
            f"| Vision 3D Error={trial_result['position_error_mm']} mm "
            f"| Placement Error={trial_result['placement_error_mm']} mm"
        )

    print_and_save_stage4b_report(results)


async def _safe_run():
    try:
        await run_stage4b_vision_benchmark()
    except asyncio.CancelledError:
        print("Stage 4B vision Pick & Place task stopped cleanly")
    except Exception:
        print("")
        print("========== STAGE 4B VISION PICK & PLACE ERROR ==========")
        traceback.print_exc()


stage4b_task = asyncio.ensure_future(_safe_run())
pick_place_task = stage4b_task

print(
    f"Lexium Stage 4B Vision Pick & Place task created (NUM_TRIALS={NUM_TRIALS}, SEED={RANDOM_SEED})."
)
