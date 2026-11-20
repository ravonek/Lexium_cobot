#!/usr/bin/env python3
"""Stage 3C.22: read-only URDF gravity torque estimate at LOGGED robot joint poses.

Computes STATIC gravitational joint efforts with URDF forward kinematics and mass
centers, plus per-link contributions to joint_2. This does not run PhysX,
change the USD, or claim dynamic/actuator torque equivalence.

Use:
    python3 lexium_stage3c22_urdf_gravity_torque_audit.py \
      --urdf path/to/lexium_cobot.urdf \
      --log /tmp/lexium_stage3c20_x030_robot.log

Requires numpy. Use Isaac Lab's uv Python if the system Python lacks numpy.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

ARM_NAMES = (
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6_gripper_rotation",
)
FINGER_HOME = {
    "gripper_Revolute_1": -0.55,
    "gripper_Revolute_2": -0.3403,
    "gripper_Revolute_5": -0.3403,
    "gripper_Revolute_6": -0.3403,
}
Q_HOME = [0.0, -0.7854, 1.5708, -0.7854, 1.5708, 0.0]


def vec(s: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not s:
        return np.array(default, dtype=float)
    out = np.fromstring(s, sep=" ", dtype=float)
    if out.shape != (3,) or not np.all(np.isfinite(out)):
        raise ValueError(f"Expected finite XYZ vector, got {s!r}")
    return out


def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ], dtype=float)


def axis_rotation(axis: np.ndarray, q: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-10:
        raise ValueError("Zero-length joint axis in URDF")
    a = axis / norm
    x, y, z = a
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(q)*skew + (1.0-math.cos(q))*(skew@skew)


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        if not name:
            raise ValueError("URDF link without name")
        inertial = link.find("inertial")
        mass = 0.0
        com = np.zeros(3)
        if inertial is not None:
            mass_el = inertial.find("mass")
            if mass_el is not None and mass_el.get("value") is not None:
                mass = float(mass_el.get("value"))
            origin = inertial.find("origin")
            if origin is not None:
                com = vec(origin.get("xyz"), (0.,0.,0.))
        if not (math.isfinite(mass) and mass >= 0):
            raise ValueError(f"Invalid mass {mass} for {name}")
        links[name] = {"mass": mass, "com": com}
    joints = {}
    children = {}
    all_child_links = set()
    for joint in root.findall("joint"):
        name, kind = joint.get("name"), joint.get("type")
        parent_el, child_el = joint.find("parent"), joint.find("child")
        if not name or parent_el is None or child_el is None:
            raise ValueError("Malformed URDF joint")
        parent, child = parent_el.get("link"), child_el.get("link")
        if parent not in links or child not in links:
            raise ValueError(f"Invalid joint references {name}: {parent} -> {child}")
        org = joint.find("origin")
        xyz = vec(org.get("xyz") if org is not None else None, (0.,0.,0.))
        rpy = vec(org.get("rpy") if org is not None else None, (0.,0.,0.))
        axis_el = joint.find("axis")
        axis = vec(axis_el.get("xyz") if axis_el is not None else None, (1.,0.,0.))
        if kind not in ("fixed", "revolute", "continuous", "prismatic"):
            raise ValueError(f"Unsupported joint type {kind!r}: {name}")
        joints[name] = {"type": kind, "parent": parent, "child": child,
                        "xyz": xyz, "rpy": rpy, "axis": axis}
        children.setdefault(parent, []).append(name)
        if child in all_child_links:
            raise ValueError(f"URDF contains multiple parent joints for {child}")
        all_child_links.add(child)
    roots = sorted(set(links) - all_child_links)
    if len(roots) != 1:
        raise ValueError(f"Expected one root link, got {roots}")
    for name in ARM_NAMES:
        if name not in joints or joints[name]["type"] not in ("revolute", "continuous"):
            raise ValueError(f"Missing/invalid arm joint {name}")
    return links, joints, children, roots[0]


def compute_torque(links, joints, children, root: str, arm_q: list[float], gravity=9.81):
    q_by_name = dict(zip(ARM_NAMES, arm_q))
    q_by_name.update(FINGER_HOME)
    pose = {root: (np.eye(3), np.zeros(3))}
    axes = {}
    joint_points = {}
    # DFS over source URDF tree with URDF joint conventions:
    # T_parent_child = T_xyz_rpy @ T_axis_rotation(q).
    stack = [root]
    while stack:
        parent = stack.pop()
        R0, p0 = pose[parent]
        for name in children.get(parent, ()):
            j = joints[name]
            R_j = R0 @ rpy_to_R(j["rpy"])
            p_j = p0 + R0 @ j["xyz"]
            ax = R_j @ j["axis"]
            ax_norm = float(np.linalg.norm(ax))
            if ax_norm == 0:
                raise ValueError(f"Zero world axis: {name}")
            axes[name] = ax / ax_norm
            joint_points[name] = p_j
            q = float(q_by_name.get(name, 0.0))
            if j["type"] in ("revolute", "continuous"):
                Rchild = R_j @ axis_rotation(j["axis"], q)
                pchild = p_j
            elif j["type"] == "prismatic":
                Rchild = R_j
                pchild = p_j + R_j @ j["axis"] * q
            else:
                Rchild, pchild = R_j, p_j
            pose[j["child"]] = (Rchild, pchild)
            stack.append(j["child"])
    if set(pose) != set(links):
        raise RuntimeError(f"Unreachable URDF links {sorted(set(links)-set(pose))}")

    # Masses downstream of the requested joint (including child link).
    def descendants(jname):
        out = []
        todo = [joints[jname]["child"]]
        while todo:
            link = todo.pop()
            out.append(link)
            todo.extend(joints[other]["child"] for other in children.get(link, ()))
        return out

    grav_force_per_kg = np.array([0.0, 0.0, -float(gravity)])
    torque_by_joint = {}
    joint2_contrib = []
    for joint_name in ARM_NAMES:
        ax = axes[joint_name]
        pivot = joint_points[joint_name]
        total_ext_tau = 0.0
        for link_name in descendants(joint_name):
            mass = links[link_name]["mass"]
            if mass == 0:
                continue
            R_link, p_link = pose[link_name]
            p_com_world = p_link + R_link @ links[link_name]["com"]
            Fg = mass * grav_force_per_kg
            # Gravity torque exerted on the subtree around the joint axis.
            tau_ext = float(np.dot(ax, np.cross(p_com_world - pivot, Fg)))
            total_ext_tau += tau_ext
            if joint_name == "joint_2":
                joint2_contrib.append((link_name, mass, -tau_ext,
                                       float(np.linalg.norm(p_com_world - pivot))))
        # Positive motor compensation torque cancels external gravitational torque.
        torque_by_joint[joint_name] = -total_ext_tau
    return torque_by_joint, joint2_contrib


def logged_poses(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    out = []
    match = re.search(r"WARMUP_ARM_Q_RAD=(\[[^\n]*?\])", text)
    if match:
        data = json.loads(match.group(1))
        if len(data) == 6:
            out.append(("WARMUP", list(map(float, data))))
    for m in re.finditer(r"WAYPOINT_DIAGNOSTICS=(\d+)\s+[^\n]*?ARM_Q_RAD=(\[[^\n]*?\])", text):
        data = json.loads(m.group(2))
        if len(data) != 6:
            raise ValueError(f"Expected 6 joint angles in {path}: {m.group(0)}")
        out.append((f"WP{m.group(1)}", list(map(float, data))))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    default_project = Path.home() / "Desktop/adi/lexium_rl_workspace/lexium_pick_place"
    p.add_argument("--urdf", type=Path, default=default_project / "assets/lexium_cobot.urdf")
    p.add_argument("--log", action="append", type=Path,
                   help="Path to Stage 3C.18/19/20 log; may repeat")
    p.add_argument("--joint-limit-nm", type=float, default=100.0)
    args = p.parse_args()
    urdf = args.urdf.expanduser().resolve()
    if not urdf.is_file():
        p.error(f"URDF not found: {urdf}")
    if not (args.joint_limit_nm > 0.0):
        p.error("--joint-limit-nm must be positive")
    links, joints, children, root = parse_urdf(urdf)
    print("LEXIUM_STAGE3C22_STARTED=True", flush=True)
    print(f"URDF_PATH={urdf}", flush=True)
    print(f"URDF_LINKS={len(links)} URDF_JOINTS={len(joints)} ROOT_LINK={root}", flush=True)
    print(f"URDF_TOTAL_MASS_KG={sum(v['mass'] for v in links.values()):.6f}", flush=True)
    print("METHOD=URDF_FK_COM_GRAVITY_CROSS_PRODUCT", flush=True)
    print(f"EFFORT_LIMIT_REFERENCE_NM={args.joint_limit_nm:.3f}", flush=True)
    print("INTERPRETATION=static_theoretical_gravity_compensation_only_no_PhysX_or_dynamic_effects", flush=True)

    tau_home, _ = compute_torque(links, joints, children, root, Q_HOME)
    print("GRAVITY_POSE label=HOME q_rad=" + json.dumps(Q_HOME) +
          " torque_compensation_nm=" + json.dumps([round(tau_home[n], 4) for n in ARM_NAMES]) +
          f" joint2_compensation_nm={tau_home['joint_2']:.4f}" +
          f" joint2_abs_margin_nm={args.joint_limit_nm - abs(tau_home['joint_2']):.4f}", flush=True)

    log_paths = args.log or [
        Path("/tmp/lexium_stage3c18_radius036.log"),
        Path("/tmp/lexium_stage3c19_nearside_robot_only.log"),
        Path("/tmp/lexium_stage3c20_x030_robot.log"),
    ]
    found = 0
    for log in log_paths:
        if not log.is_file():
            print(f"LOG_MISSING path={log}", flush=True)
            continue
        poses = logged_poses(log)
        if not poses:
            print(f"LOG_NO_POSES path={log} (need WAYPOINT_DIAGNOSTICS with ARM_Q_RAD)", flush=True)
            continue
        print(f"LOG_POSES source={log.name} count={len(poses)}", flush=True)
        found += 1
        for label, q in poses:
            tau, contrib = compute_torque(links, joints, children, root, q)
            # Print all poses; include per-link breakdown for the last pose only.
            print("GRAVITY_POSE" +
                  f" source={log.name} waypoint={label}" +
                  " q_rad=" + json.dumps([round(a, 5) for a in q]) +
                  " torque_compensation_nm=" + json.dumps([round(tau[n], 3) for n in ARM_NAMES]) +
                  f" joint2_compensation_nm={tau['joint_2']:.3f}" +
                  f" joint2_abs_margin_nm={args.joint_limit_nm - abs(tau['joint_2']):.3f}", flush=True)
        label, q = poses[-1]
        _, contrib = compute_torque(links, joints, children, root, q)
        print(f"JOINT2_SUBTREE_BREAKDOWN source={log.name} waypoint={label}", flush=True)
        print("JOINT2_DOWNSTREAM_MASS_KG=" + f"{sum(a[1] for a in contrib):.6f}", flush=True)
        for link, mass, tau, pivot_dist in sorted(contrib, key=lambda x: abs(x[2]), reverse=True):
            print(f"JOINT2_LINK_CONTRIBUTION source={log.name} link={link} mass_kg={mass:.5f} "
                  f"gravity_compensation_nm={tau:.4f} com_to_joint_distance_m={pivot_dist:.4f}", flush=True)

    print(f"LOGS_ANALYZED={found}", flush=True)
    if found == 0:
        print("AUDIT_WARNING=No readable logs found; HOME pose calculation only", flush=True)
    print("FILES_MODIFIED=False", flush=True)
    print("LEXIUM_STAGE3C22_GRAVITY_AUDIT_COMPLETE=True", flush=True)


if __name__ == "__main__":
    main()
