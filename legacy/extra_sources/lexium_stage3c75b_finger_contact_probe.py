#!/usr/bin/env python3
"""Stage 3C.75b: HOME-hold positive control of cube-filtered finger contact sensors.

Derived from lexium_stage3c75_gravity_fixed.py (which is preserved unchanged).

Changes vs. gravity_fixed, each justified by real PhysX logs:
  * HOME hold keeps the Stage50/73 gravity-compensated command
    (q_cmd = q_home + tau_g / K). diagnose_stage75_home_drift_ab.py proved that the
    raw HOME command droops joint_2 by ~0.073 rad (> 0.07 gate) at tick 10.
  * Probe windows are bounded: the cube balances on a ~35-38 deg convex-hull apex
    of the finger and slides off onto other robot bodies (finger2 / finger4),
    which previously ended in FORCE_GT_TEST_LIMIT. A probe now ends as soon as the
    cube touches any non-target robot body, leaves the probe region, or after a
    confirmed contact window. Global force / tracking safety gates are unchanged.
  * Positive detection requires sustained (N consecutive ticks) contact that is
    independently attributed to the target finger body by the unfiltered
    per-body robot contact sensor. False attribution to the other finger is
    checked against the same per-body ground truth.
  * Abort prefix renamed to STAGE75_ABORT (the old STAGE74_* prefix was a
    copy-paste artefact) and a JSON report is written on failure as well.

No finger closure, no grasp, no lift, no arm trajectory. Simulation only.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage73', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--steps', type=int, default=150)
parser.add_argument('--num-envs', type=int, default=1)
parser.add_argument('--stage74',type=Path,required=True)
parser.add_argument('--stage50',type=Path,required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

if args.num_envs != 1 or not 120 <= args.steps <= 300:
    parser.error('Require one env and 120..300 steps')

if not args.stage73.is_file():
    parser.error('Missing Stage73 report')

prior = json.loads(args.stage73.read_text())

if not (
    prior.get('stage') == '3C73'
    and prior.get('finger_precontact_passed') is True
    and prior.get('simulation_motion_passed') is True
    and prior.get('grasp_attempted') is False
    and prior.get('physx_self_collisions_enabled') is False
):
    parser.error('Stage73 provenance not valid')

for path in (args.stage74,args.stage50):
    if not path.is_file():
        parser.error('MISSING_'+str(path))

s74=json.loads(args.stage74.read_text())
s50=json.loads(args.stage50.read_text())

if not (
    s74.get('stage')=='3C74'
    and s74.get('sensor_wiring_passed') is True
    and s74.get('cube_table_positive_control_passed') is True
    and s74.get('contact_closure_ready') is False
    and s50.get('stage')=='3C50'
    and s50.get('home_contact_positive_control_passed') is True
    and s50.get('physx_self_collisions_enabled') is False
):
    parser.error('STAGE74_OR_50_PREREQUISITES_FAILED')

launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils

from isaaclab.assets import (
    ArticulationCfg, AssetBaseCfg, RigidObjectCfg
)
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab_physx.sim.spawners.materials import (
    PhysxRigidBodyMaterialCfg
)
from lexium_pick_place.robot_cfg import (
    LEXIUM_COBOT_CFG,
    LEXIUM_ARM_JOINT_NAMES,
)
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import (
    parse_urdf,
    Q_HOME,
)
from lexium_pick_place.lexium_stage3c25_corrected_gravity_route_audit import (
    compute_root_frame_gravity,
)


ROBOT_CFG = LEXIUM_COBOT_CFG.replace(
    prim_path='{ENV_REGEX_NS}/Robot',
    spawn=LEXIUM_COBOT_CFG.spawn.replace(
        activate_contact_sensors=True
    ),
)


def table_cfg():
    return AssetBaseCfg(
        prim_path='{ENV_REGEX_NS}/Table',
        spawn=sim_utils.CuboidCfg(
            size=(.90, 1.20, .40),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.,
                dynamic_friction=.8,
                restitution=0.
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(.35, .35, .42)
            )
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(.62, 0., -.20)
        )
    )


def cube_cfg():
    return RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/Cube',
        spawn=sim_utils.CuboidCfg(
            size=(.04, .04, .04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=2,
                disable_gravity=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(
                mass=.10
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            activate_contact_sensors=True,
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.,
                dynamic_friction=.8,
                restitution=0.
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(.15, .75, .25)
            )
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(.30, -.30, .12)
        )
    )


@configclass
class Stage74Scene(InteractiveSceneCfg):
    table: AssetBaseCfg = table_cfg()
    robot: ArticulationCfg = ROBOT_CFG
    cube: RigidObjectCfg = cube_cfg()

    robot_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*',
        update_period=0.
    )

    cube_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Cube',
        update_period=0.
    )

    finger1_cube: ContactSensorCfg = ContactSensorCfg(
        prim_path=(
            '{ENV_REGEX_NS}/Robot/Geometry/'
            'gripper_finger_1_1'
        ),
        filter_prim_paths_expr=[
            '{ENV_REGEX_NS}/Cube'
        ],
        update_period=0.
    )

    finger3_cube: ContactSensorCfg = ContactSensorCfg(
        prim_path=(
            '{ENV_REGEX_NS}/Robot/Geometry/'
            'gripper_finger_3_1'
        ),
        filter_prim_paths_expr=[
            '{ENV_REGEX_NS}/Cube'
        ],
        update_period=0.
    )

    light: AssetBaseCfg = AssetBaseCfg(
        prim_path='/World/Light',
        spawn=sim_utils.DomeLightCfg(intensity=2500.)
    )


def abort(reason):
    raise RuntimeError(
        'STAGE75_ABORT=' + str(reason)
    )


# Probe parameters (contact detection only; no force targets are commanded).
CONTACT_N = 0.05            # same detection threshold as the original Stage75
CONFIRM_TICKS = 5           # consecutive clean, attributed contact ticks
CONFIRMED_HOLD_TICKS = 30   # extra observation after confirmation
RELEASE_TICKS = 3           # consecutive no-contact ticks after confirmation
PROBE_MAX_TICKS = 90        # same probe length as the original Stage75
PROBE_MIN_CUBE_Z = 0.10     # m; below this the cube has left the finger region

# Filled progressively by main(); written as a FAIL report on any exception.
RESULT = {'stage': '3C75B', 'status': 'RUNNING', 'verified_pass': False}


def filtered_force(sensor, label):
    data = sensor.data

    found = None
    accessor = None

    for attr in (
        'normal_force_matrix_w',
        'force_matrix_w'
    ):
        try:
            buffer = getattr(data, attr)
        except (AttributeError, NotImplementedError):
            continue

        if buffer is not None:
            found = (
                buffer.torch
                if hasattr(buffer, 'torch')
                else buffer
            )
            accessor = attr
            break

    if found is None:
        abort('MISSING_FILTERED_MATRIX_' + label)

    if tuple(found.shape) != (1, 1, 1, 3):
        abort(
            'BAD_FILTERED_SHAPE_'
            + label
            + '_'
            + repr(tuple(found.shape))
        )

    if not bool(torch.isfinite(found).all().item()):
        abort('NONFINITE_FILTERED_FORCE_' + label)

    force = float(
        torch.linalg.vector_norm(
            found[0, 0, 0]
        ).item()
    )

    return force, accessor




def main():
    from scipy.spatial.transform import Rotation

    print(
        'STAGE=3C75B MODE=ISAAC_LAB_POSITIVE_FINGER_SENSOR_CONTROL',
        flush=True
    )
    print(
        'STAGE75_HOME_HOLD_ONLY=True CUBE_RELOCATED_FOR_CONTACT=True',
        flush=True
    )
    print(
        'STAGE75_FINGERS_MOVED=False GRASP=False HARDWARE=False',
        flush=True
    )

    sim=SimulationContext(
        sim_utils.SimulationCfg(
            device=args.device,
            dt=1/60,
            gravity=(0,0,-9.81)
        )
    )

    scene=InteractiveScene(
        Stage74Scene(num_envs=1,env_spacing=2.5)
    )

    sim.reset()
    scene.reset()
    scene.update(sim.get_physics_dt())

    robot,cube=scene['robot'],scene['cube']
    rs,cs=scene['robot_contact'],scene['cube_contact']

    sensors={
        'FINGER1':scene['finger1_cube'],
        'FINGER3':scene['finger3_cube']
    }

    names={
        'FINGER1':'gripper_finger_1_1',
        'FINGER3':'gripper_finger_3_1'
    }

    if not robot.is_fixed_base:
        abort('ROBOT_NOT_FIXED')

    def filtered(sensor,label):
        return filtered_force(sensor,label)

    for label in sensors:
        if list(sensors[label].body_names)!=[names[label]]:
            abort('FILTERED_SENSOR_BODY_MISMATCH_'+label)
        filtered(sensors[label],label)

    # Unfiltered per-body robot contact sensor = independent ground truth for
    # attribution (self-collisions are disabled and the table is far from the
    # gripper, so in this scene only the cube can touch robot bodies).
    robot_sensor_bodies = list(rs.body_names)
    print(
        'STAGE75_ROBOT_SENSOR_BODIES=' + json.dumps(robot_sensor_bodies),
        flush=True,
    )
    for body in names.values():
        if robot_sensor_bodies.count(body) != 1:
            abort('ROBOT_SENSOR_BODY_MISSING_' + body)
    body_idx = {
        label: robot_sensor_bodies.index(names[label]) for label in names
    }
    last_body_forces = [np.zeros(len(robot_sensor_bodies))]
    RESULT['robot_sensor_bodies'] = robot_sensor_bodies

    qhome=robot.data.default_joint_pos.torch.clone()

    robot.write_joint_state_to_sim_index(
        position=qhome,
        velocity=torch.zeros_like(qhome)
    )

    cube.write_root_pose_to_sim_index(
        root_pose=cube.data.default_root_pose.torch.clone()
    )

    cube.write_root_velocity_to_sim_index(
        root_velocity=cube.data.default_root_vel.torch.clone()
    )

    scene.update(0.)
    dt=sim.get_physics_dt()

    urdf_path = (
        Path(__file__).resolve().parent / 'assets/lexium_cobot.urdf'
    )
    if not urdf_path.is_file():
        abort('MISSING_GRAVITY_URDF')

    links, arm_joints, children, root_name = parse_urdf(urdf_path)
    joint_names = list(robot.data.joint_names)
    body_names = list(robot.data.body_names)
    arm_ids = [
        joint_names.index(n) for n in LEXIUM_ARM_JOINT_NAMES
    ]
    root_idx = body_names.index(root_name)

    q_arm = (
        qhome[0, arm_ids].detach().cpu().numpy().astype(float)
    )
    if not np.allclose(q_arm, Q_HOME, atol=1e-4):
        abort('CONFIGURED_HOME_MISMATCH')

    stiffness = (
        robot.data.joint_stiffness.torch[0, arm_ids]
        .detach().cpu().numpy().astype(float)
    )
    limits = (
        robot.data.joint_pos_limits.torch[0, arm_ids]
        .detach().cpu().numpy().astype(float)
    )
    if not np.allclose(
        stiffness, [400, 800, 800, 400, 400, 400], atol=0.1
    ):
        abort('UNEXPECTED_ARM_STIFFNESS')

    root_pose = (
        robot.data.body_link_pose_w.torch[0, root_idx]
        .detach().cpu().numpy().astype(float)
    )
    gravity_root = Rotation.from_quat(root_pose[3:7]).inv().apply(
        [0.0, 0.0, -9.81]
    )
    if not np.allclose(
        gravity_root, [0.0, -9.81, 0.0], atol=0.01
    ):
        abort('UNEXPECTED_GRAVITY_REFERENCE_FRAME')

    gravity_torque = np.asarray(
        compute_root_frame_gravity(
            links, arm_joints, children,
            root_name, q_arm, gravity_root
        ),
        dtype=float,
    )
    bias = gravity_torque / stiffness
    arm_command = q_arm + bias

    if not np.isfinite(
        np.r_[gravity_torque, bias, arm_command]
    ).all():
        abort('NONFINITE_GRAVITY_COMMAND')
    if (
        np.max(np.abs(gravity_torque)) >= 85.0
        or np.max(np.abs(bias)) >= 0.25
    ):
        abort('HOME_GRAVITY_LIMIT')
    if (
        np.any(arm_command <= limits[:, 0] + 0.04)
        or np.any(arm_command >= limits[:, 1] - 0.04)
    ):
        abort('GRAVITY_COMMAND_OUTSIDE_SAFE_JOINT_MARGIN')

    cmdfull = qhome.clone()
    for idx_i, joint_j in enumerate(arm_ids):
        cmdfull[0, joint_j] = float(arm_command[idx_i])

    print(
        'STAGE75_HOME_GRAVITY_TORQUE_NM='
        + json.dumps(np.round(gravity_torque, 4).tolist()),
        flush=True,
    )
    print(
        'STAGE75_HOME_GRAVITY_BIAS_RAD='
        + json.dumps(np.round(bias, 6).tolist()),
        flush=True,
    )
    print(
        'STAGE75_HOME_GRAVITY_COMPENSATED_COMMAND=True',
        flush=True,
    )
    tick_count = [0]

    def tick():
        tick_count[0] += 1
        if not simulation_app.is_running():
            abort('APP_STOPPED')

        robot.actuators.target_command.set_position_index(
            value=cmdfull,
            full_data=True
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)

        raw_r=rs.data.net_normal_forces_w.torch
        raw_c=cs.data.net_normal_forces_w.torch

        if not bool(
            torch.isfinite(raw_r).all().item()
            and torch.isfinite(raw_c).all().item()
        ):
            abort('NONFINITE_SENSOR_OUTPUT')

        vals={
            label:filtered(sensor,label)[0]
            for label,sensor in sensors.items()
        }

        vals['ROBOT']=float(
            torch.linalg.vector_norm(
                raw_r[0],dim=-1
            ).max().item()
        )

        last_body_forces[0] = (
            torch.linalg.vector_norm(raw_r[0], dim=-1)
            .detach().cpu().numpy().astype(float)
        )

        vals['CUBE']=float(
            torch.linalg.vector_norm(
                raw_c[0],dim=-1
            ).max().item()
        )

        vals['TRACKING']=float(
            torch.max(
                torch.abs(
                    robot.data.joint_pos.torch-qhome
                )
            ).item()
        )

        if not np.isfinite(list(vals.values())).all():
            abort('NONFINITE_STATE')

        if tick_count[0] in (1, 30, args.steps):
            print(
                'STAGE75_BASELINE_SAMPLE step={} track_rad={:.6f} '
                'cube_n={:.4f} robot_n={:.4f} '
                'f1_n={:.4f} f3_n={:.4f}'.format(
                    tick_count[0],
                    vals['TRACKING'],
                    vals['CUBE'],
                    vals['ROBOT'],
                    vals['FINGER1'],
                    vals['FINGER3'],
                ),
                flush=True,
            )

        if vals['TRACKING'] > .07:
            measured = (
                robot.data.joint_pos.torch[0]
                .detach().cpu().numpy().astype(float)
            )
            err = measured - (
                qhome[0].detach().cpu().numpy().astype(float)
            )
            bad_idx = int(np.argmax(np.abs(err)))
            print(
                'STAGE75_HOME_DRIFT_TICK=' + str(tick_count[0])
                + ' JOINT=' + joint_names[bad_idx]
                + ' ERR_RAD=' + str(float(err[bad_idx])),
                flush=True,
            )
            abort('ROBOT_HOME_DRIFT')

        if (
            max(
                vals['FINGER1'],
                vals['FINGER3'],
                vals['ROBOT']
            )>25
            or vals['CUBE']>40
        ):
            abort(
                'FORCE_GT_TEST_LIMIT tick={} vals={} max_body={}:{:.3f}'.format(
                    tick_count[0], json.dumps(vals),
                    robot_sensor_bodies[int(np.argmax(last_body_forces[0]))],
                    float(np.max(last_body_forces[0])))
            )

        return vals

    # Negative control: cube on table, fingers far away.
    for _ in range(args.steps):
        baseline=tick()

    if not (
        .2<=baseline['CUBE']<=5
        and baseline['ROBOT']<.05
        and baseline['FINGER1']<.01
        and baseline['FINGER3']<.01
    ):
        abort(
            'NEGATIVE_BASELINE_FAILED_'
            +repr(baseline)
        )

    print(
        'STAGE75_HOME_NEGATIVE_CONTROL_PASS=True',
        flush=True
    )
    RESULT['negative_control_baseline'] = dict(baseline)
    RESULT['negative_control_passed'] = True

    print(
        'STAGE75_CUBE_TABLE_FORCE_N={:.5f}'.format(
            baseline['CUBE']
        ),
        flush=True
    )

    # Points on upper HOME finger STL surfaces.
    # These are sensor calibration positions, not grasp targets.
    points={
        'FINGER1':np.array([-.0850,-.3295,.2365]),
        'FINGER3':np.array([-.1972,-.3131,.1794])
    }

    ref=np.asarray(
        s50['final_gripper_base_pose_xyzw'],
        float
    )

    if ref.shape!=(7,):
        abort('BAD_STAGE50_HOME_POSE')

    bn=list(robot.data.body_names)

    if 'gripper_base_1' not in bn:
        abort('TOOL_BODY_MISSING')

    now=(
        robot.data.body_link_pose_w.torch[
            0,bn.index('gripper_base_1')
        ]
        .detach().cpu().numpy().astype(float)
    )

    rot_ref=Rotation.from_quat(ref[3:])
    rot_now=Rotation.from_quat(now[3:])
    pos_err_m = float(np.linalg.norm(now[:3] - ref[:3]))
    rot_err_rad = float((rot_now * rot_ref.inv()).magnitude())
    print(
        'STAGE75B_HOME_TOOL_REF_CHECK pos_err_mm={:.4f} rot_err_rad={:.6f}'
        .format(pos_err_m * 1000.0, rot_err_rad),
        flush=True,
    )
    RESULT['home_tool_ref_pos_err_m'] = pos_err_m
    RESULT['home_tool_ref_rot_err_rad'] = rot_err_rad

    if (
        np.linalg.norm(now[:3]-ref[:3])>.005
        or (rot_now*rot_ref.inv()).magnitude()>.04
    ):
        abort('HOME_POSE_REFERENCE_MISMATCH')

    def reset_cube(xyz):
        pose=cube.data.default_root_pose.torch.clone()

        pose[0,:3]=torch.as_tensor(
            xyz,
            dtype=pose.dtype,
            device=pose.device
        )

        cube.write_root_pose_to_sim_index(
            root_pose=pose
        )

        cube.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros_like(
                cube.data.default_root_vel.torch
            )
        )

        scene.update(0.)

    results={}

    for name in ('FINGER1','FINGER3'):

        local=rot_ref.inv().apply(
            points[name]-ref[:3]
        )

        surface=now[:3]+rot_now.apply(local)

        # Cube half-height = 20 mm.
        # Start with ~13 mm gap above target surface.
        start=surface+np.array([0.,0.,.033])

        reset_cube(start)

        print(
            'STAGE75_'+name+'_START_XYZ_M='
            +json.dumps(np.round(start,6).tolist()),
            flush=True
        )

        other=(
            'FINGER3'
            if name=='FINGER1'
            else 'FINGER1'
        )

        t_idx = body_idx[name]
        o_idx = body_idx[other]
        peak_target = 0.0
        peak_other = 0.0
        peak_cube = 0.0
        peak_nontarget_body = 0.0
        first_contact = None
        confirmed_at = None
        streak = 0
        no_contact_after_confirm = 0
        false_attribution_ticks = []
        target_mismatch_ticks = []
        end_reason = 'TIMEOUT_NO_CONFIRMATION'
        trace = []

        for i in range(1, PROBE_MAX_TICKS + 1):
            v = tick()
            body_f = last_body_forces[0].copy()
            nontarget = body_f.copy()
            nontarget[t_idx] = 0.0
            nt_i = int(np.argmax(nontarget))
            nt_max = float(nontarget[nt_i])
            cxyz = (
                cube.data.root_pos_w.torch[0]
                .detach().cpu().numpy().astype(float)
            )
            peak_target = max(peak_target, v[name])
            peak_other = max(peak_other, v[other])
            peak_cube = max(peak_cube, v['CUBE'])
            peak_nontarget_body = max(peak_nontarget_body, nt_max)

            # Cross-check filtered sensors against unfiltered per-body truth.
            if v[other] > CONTACT_N and body_f[o_idx] < CONTACT_N:
                false_attribution_ticks.append(i)
            if abs(v[name] - body_f[t_idx]) > max(0.02, 0.1 * v[name]):
                target_mismatch_ticks.append(i)

            if first_contact is None and v[name] > CONTACT_N:
                first_contact = i

            clean = (
                v[name] > CONTACT_N
                and body_f[t_idx] > CONTACT_N
                and v['CUBE'] > CONTACT_N
                and v[other] < CONTACT_N
                and nt_max < CONTACT_N
            )
            streak = streak + 1 if clean else 0
            if confirmed_at is None and streak >= CONFIRM_TICKS:
                confirmed_at = i
                print(
                    'STAGE75B_' + name + '_CONTACT_CONFIRMED_TICK=' + str(i),
                    flush=True,
                )

            row = {
                'tick': i,
                'target_n': v[name],
                'other_n': v[other],
                'target_body_net_n': float(body_f[t_idx]),
                'other_body_net_n': float(body_f[o_idx]),
                'max_nontarget_body_n': nt_max,
                'max_nontarget_body': robot_sensor_bodies[nt_i],
                'cube_n': v['CUBE'],
                'tracking_rad': v['TRACKING'],
                'cube_xyz_m': np.round(cxyz, 5).tolist(),
            }
            trace.append(row)
            print(
                'STAGE75B_{} tick={} target_n={:.4f} body_t={:.4f} '
                'other_n={:.4f} body_o={:.4f} nontarget_max={:.4f}({}) '
                'cube_n={:.4f} track={:.5f} cube_xyz={}'.format(
                    name, i, v[name], body_f[t_idx], v[other],
                    body_f[o_idx], nt_max, robot_sensor_bodies[nt_i],
                    v['CUBE'], v['TRACKING'],
                    json.dumps(np.round(cxyz, 4).tolist()),
                ),
                flush=True,
            )

            # Bounded probe: stop before uncontrolled secondary impacts.
            if nt_max > CONTACT_N:
                end_reason = (
                    'CUBE_TOUCHED_NON_TARGET_BODY_'
                    + robot_sensor_bodies[nt_i]
                )
                break
            if cxyz[2] < PROBE_MIN_CUBE_Z:
                end_reason = 'CUBE_LEFT_PROBE_REGION'
                break
            if confirmed_at is not None:
                no_contact_after_confirm = (
                    no_contact_after_confirm + 1
                    if v[name] < CONTACT_N else 0
                )
                if no_contact_after_confirm >= RELEASE_TICKS:
                    end_reason = 'CONTACT_RELEASED_AFTER_CONFIRMATION'
                    break
                if i >= confirmed_at + CONFIRMED_HOLD_TICKS:
                    end_reason = 'CONFIRMED_OBSERVATION_WINDOW_DONE'
                    break

        ok = (
            confirmed_at is not None
            and peak_other < CONTACT_N
            and not false_attribution_ticks
            and not target_mismatch_ticks
        )

        results[name] = {
            'target_body': names[name],
            'first_contact_tick': first_contact,
            'confirmed_tick': confirmed_at,
            'confirm_ticks_required': CONFIRM_TICKS,
            'probe_ticks_executed': len(trace),
            'probe_end_reason': end_reason,
            'peak_target_filtered_n': peak_target,
            'peak_other_filtered_n': peak_other,
            'peak_cube_net_n': peak_cube,
            'peak_nontarget_body_net_n': peak_nontarget_body,
            'false_attribution_ticks': false_attribution_ticks,
            'target_filtered_vs_body_mismatch_ticks': target_mismatch_ticks,
            'probe_start_xyz_m': start.tolist(),
            'trace': trace,
            'pass': bool(ok),
        }
        RESULT.setdefault('individual_results', {})[name] = results[name]

        print('STAGE75B_' + name + '_PROBE_END_REASON=' + end_reason, flush=True)
        print(
            'STAGE75B_' + name + '_POSITIVE_CONTROL_PASS=' + str(bool(ok)),
            flush=True,
        )
        print(
            'STAGE75B_{}_PEAK_TARGET_N={:.6f} PEAK_OTHER_N={:.6f}'.format(
                name, peak_target, peak_other),
            flush=True,
        )

        # Restore the cube to the table before the next trial.
        reset_cube([.30,-.30,.12])

        for _ in range(40):
            gap=tick()

        gap_body = last_body_forces[0].copy()
        if (
            gap['FINGER1']>.02
            or gap['FINGER3']>.02
            or float(gap_body.max()) > .02
        ):
            abort('SENSOR_NOT_CLEARED_AFTER_PROBE')
        if not .2 <= gap['CUBE'] <= 5.:
            abort('CUBE_TABLE_CONTACT_NOT_RESTORED_' + repr(gap['CUBE']))
        print(
            'STAGE75B_' + name + '_POST_PROBE_CLEAR f1={:.4f} f3={:.4f} '
            'robot_max={:.4f} cube_table={:.4f}'.format(
                gap['FINGER1'], gap['FINGER3'], float(gap_body.max()),
                gap['CUBE']),
            flush=True,
        )

    passed=all(
        x['pass']
        for x in results.values()
    )

    report = dict(RESULT)
    report.update({
        'stage': '3C75B',
        'status': 'PASS' if passed else 'FAIL',
        'verified_pass': bool(passed),
        'source_stage74_passed': True,
        'both_finger_positive_controls_passed': bool(passed),
        'method': 'cube_gravity_drop_on_finger_convex_hull_apex_at_HOME',
        'physx_collider_approximation': 'convexHull (from robot USD)',
        'home_gravity_compensated': True,
        'home_gravity_torque_nm': gravity_torque.tolist(),
        'home_gravity_bias_rad': bias.tolist(),
        'home_tool_ref_pos_err_m': pos_err_m,
        'home_tool_ref_rot_err_rad': rot_err_rad,
        'negative_control_baseline': baseline,
        'detection_threshold_n': CONTACT_N,
        'confirm_ticks': CONFIRM_TICKS,
        'force_safety_limits_n': {'finger_or_robot': 25.0, 'cube': 40.0},
        'home_tracking_limit_rad': 0.07,
        'individual_results': results,
        'contact_closure_ready': False,
        'grasp_validated': False,
        'fingers_moved': False,
        'finger_closure_commanded': False,
        'arm_trajectory_commanded': False,
        'self_collision_validated': False,
        'physx_self_collisions_enabled': False,
        'hardware_permission': False,
    })

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    RESULT['report_written'] = True

    print('STAGE75B_BOTH_FINGER_POSITIVE_CONTROLS_PASS=' + str(bool(passed)), flush=True)
    print('STAGE75B_CONTACT_CLOSURE_READY=False', flush=True)
    print('STAGE75B_GRASP_EXECUTED=False', flush=True)
    print('STAGE75B_REPORT=' + str(args.out), flush=True)

    if not passed:
        abort('POSITIVE_CONTROL_NOT_BOTH_CONFIRMED')

    print('LEXIUM_STAGE3C75B_COMPLETE=True', flush=True)


def write_failure_report(exc):
    """Persist a machine-readable FAIL report unless a final report exists."""
    if RESULT.get('report_written'):
        return
    try:
        fail = dict(RESULT)
        fail.update({
            'status': 'FAIL',
            'verified_pass': False,
            'failure': type(exc).__name__ + ': ' + repr(exc),
            'grasp_validated': False,
            'hardware_permission': False,
        })
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(fail, indent=2, default=str) + '\n')
        print('STAGE75B_FAILURE_REPORT=' + str(args.out), flush=True)
    except Exception as report_exc:  # never mask the original failure
        print('STAGE75B_FAILURE_REPORT_ERROR=' + repr(report_exc), flush=True)


if __name__=='__main__':
    try:
        main()
    except BaseException as exc:
        print(
            'STAGE75B_EXPLICIT_FAILURE='
            + type(exc).__name__ + ': ' + repr(exc),
            flush=True,
        )
        write_failure_report(exc)
        raise
    finally:
        simulation_app.close()
