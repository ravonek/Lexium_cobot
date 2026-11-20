#!/usr/bin/env python3
"""Lexium Stage 3C.40: OFFLINE, READ-ONLY URDF FK/IK + gripper-only path screen.

This is a DIAGNOSTIC, not a motion controller. It never imports Isaac Lab/PhysX,
changes robot assets, sends joint commands, or approves physical movement.
Inputs: robot URDF and robot_cfg.py; measured Stage35 hinge pivots; Stage36
hypothetical centers; Stage39 shortlist; clean authored collision meshes Stage38.

IK uses exact URDF revolute-joint transforms and the mounted world base pose.
Current gripper_base_1 pose is reconstructed from four measured PhysX hinge
pivots, cross-checked against world hinge axis and observed TCP. Stage39 whole-
gripper rigid hypotheses are mapped to gripper_base_1 pose targets, then numerically
screened against six arm joint limits. Optional Stage35/37 real FINAL_ARM_Q_RAD log
cross-checks the URDF-to-PhysX joint-coordinate mapping before ranking anything.

Mesh checks sample FROZEN gripper geometry at discretized IK interpolation poses.
No arm-link geometry, full robot self collision, continuous sweep, actual PhysX
convex decomposition, stability, gripper closure/contact, or dynamic torque is proven.
"""
from __future__ import annotations
import argparse
import ast
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ARM = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6_gripper_rotation")
FINGERS = ("gripper_Revolute_1", "gripper_Revolute_2", "gripper_Revolute_5", "gripper_Revolute_6")
BODIES = ("gripper_base_1", "gripper_finger_1_1", "gripper_finger_2_1", "gripper_finger_3_1", "gripper_finger_4_1")


def stop(message):
    raise SystemExit("STAGE40_INPUT_ERROR=" + str(message))


def vector(text):
    x = np.fromstring(str(text), sep=" ", dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        stop("BAD_XYZ=" + str(text))
    return x


def read_cfg(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    vars = {}
    for n in tree.body:
        if isinstance(n, ast.Assign):
            for target in n.targets:
                if isinstance(target, ast.Name) and target.id in (
                    "LEXIUM_BASE_ROTATION_XYZW", "LEXIUM_BASE_POSITION", "LEXIUM_TCP_OFFSET", "LEXIUM_ARM_JOINT_NAMES"):
                    vars[target.id] = ast.literal_eval(n.value)
    for key in ("LEXIUM_BASE_ROTATION_XYZW", "LEXIUM_BASE_POSITION", "LEXIUM_TCP_OFFSET", "LEXIUM_ARM_JOINT_NAMES"):
        if key not in vars:
            stop("MISSING_CFG_VARIABLE_" + key)
    if tuple(vars["LEXIUM_ARM_JOINT_NAMES"]) != ARM:
        stop("UNEXPECTED_ARM_NAMES_IN_CFG")
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == "joint_pos" and isinstance(node.value, ast.Dict):
            obj = ast.literal_eval(node.value)
            if all(n in obj for n in ARM):
                vars["q_home"] = np.array([float(obj[n]) for n in ARM])
                break
    if "q_home" not in vars:
        stop("JOINT_POS_NOT_FOUND_IN_CFG")
    rot = np.asarray(vars["LEXIUM_BASE_ROTATION_XYZW"], dtype=float)
    if rot.shape != (4,):
        stop("BAD_BASE_QUAT_XYZW")
    vars["R_mount"] = Rotation.from_quat(rot).as_matrix()
    vars["mount_pos"] = np.asarray(vars["LEXIUM_BASE_POSITION"], dtype=float)
    vars["tcp_offset"] = np.asarray(vars["LEXIUM_TCP_OFFSET"], dtype=float)
    if vars["mount_pos"].shape != (3,) or vars["tcp_offset"].shape != (3,):
        stop("INVALID_MOUNT_POS_OR_TCP_OFFSET")
    return vars


def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}; parent_joint = {}; children = {}; masses = {}
    for l in root.findall("link"):
        intr = l.find("inertial")
        mass = float(intr.find("mass").get("value")) if intr is not None else 0.0
        org = intr.find("origin") if intr is not None else None
        com = vector(org.get("xyz", "0 0 0")) if org is not None else np.zeros(3)
        masses[l.get("name")] = (mass, com)
    for j in root.findall("joint"):
        name = j.get("name")
        org = j.find("origin")
        if org is None:stop("MISSING_JOINT_ORIGIN_" + name)
        axisnode = j.find("axis")
        axis = vector(axisnode.get("xyz")) if axisnode is not None else np.array([1.,0.,0.])
        axis /= np.linalg.norm(axis)
        lim = j.find("limit")
        limits = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else (-math.pi, math.pi)
        parent = j.find("parent").get("link"); child = j.find("child").get("link")
        joints[name] = dict(type=j.get("type"), parent=parent, child=child,
            xyz=vector(org.get("xyz", "0 0 0")), rpy=vector(org.get("rpy", "0 0 0")),
            axis=axis, limits=limits)
        if child in parent_joint: stop("MULTIPLE_PARENT_JOINTS_" + child)
        parent_joint[child] = name
        children.setdefault(parent, []).append(name)
    chain = []; link = "gripper_base_1"
    while link in parent_joint:
        jname = parent_joint[link];chain.append(jname);link = joints[jname]["parent"]
    chain = list(reversed(chain))
    if tuple(chain) != ARM or link != "base_link":
        stop("BAD_URDF_ARM_CHAIN=" + repr(chain))
    for j in ARM+FINGERS:
        if j not in joints or joints[j]["type"] != "revolute":stop("EXPECTED_REVOLUTE=" + j)
    return joints, children, chain, masses


def fk_chain(q, cfg, joints, chain):
    R = cfg["R_mount"].copy(); p = cfg["mount_pos"].copy()
    for jname, value in zip(chain, q):
        j = joints[jname]
        p = p + R @ j["xyz"]
        R = R @ Rotation.from_euler("xyz", j["rpy"]).as_matrix()
        R = R @ Rotation.from_rotvec(j["axis"]*value).as_matrix()
    return R, p


def all_fk(q, cfg, joints, children, masses):
    """All URDF link poses and joint world-space pivot/axis for gravity estimate."""
    finger_default = {"gripper_Revolute_1":-0.55, "gripper_Revolute_2":-0.3403,
                      "gripper_Revolute_5":-0.3403,"gripper_Revolute_6":-0.3403}
    qmap = dict(zip(ARM,q));qmap.update(finger_default)
    poses={"base_link":(cfg["R_mount"],cfg["mount_pos"])}
    jointaxis={};jointpivot={}
    stack=["base_link"]
    while stack:
        parent=stack.pop();Rp,pp=poses[parent]
        for n in children.get(parent,[]):
            j=joints[n]
            po=pp + Rp@j["xyz"]
            Ro=Rp@Rotation.from_euler("xyz",j["rpy"]).as_matrix()
            jointaxis[n]=Ro@j["axis"]
            jointpivot[n]=po
            angle=qmap.get(n,0.0)
            Rchild=Ro@Rotation.from_rotvec(j["axis"]*angle).as_matrix()
            poses[j["child"]]=(Rchild,po)
            stack.append(j["child"])
    return poses,jointaxis,jointpivot


def gravity_efforts(q,cfg,joints,children,masses):
    poses,axes,pivots=all_fk(q,cfg,joints,children,masses)
    force_per_mass=np.array([0.,0.,-9.81])
    effort={}
    for armj in ARM:
        pending=[joints[armj]["child"]];sub=[]
        while pending:
            link=pending.pop();sub.append(link)
            pending.extend(joints[j]["child"] for j in children.get(link,[]))
        torque=0.
        for link in sub:
            m, com = masses[link]
            if m<=0:continue
            R,p=poses[link]
            worldcom=p+R@com
            torque-=float(np.dot(axes[armj],np.cross(worldcom-pivots[armj],m*force_per_mass)))
        effort[armj]=torque
    return np.array([effort[x] for x in ARM])


def measured_pose(joints, s35, cfg):
    pivots=np.asarray(s35["hinge_pivots_world_xyz_m"],dtype=float)
    local=np.array([joints[n]["xyz"] for n in FINGERS],dtype=float)
    if pivots.shape!=(4,3):stop("BAD_HINGE_PIVOTS")
    a=local.mean(0);b=pivots.mean(0)
    U,S,Vt=np.linalg.svd((local-a).T@(pivots-b))
    R=Vt.T@np.diag([1.,1.,np.linalg.det(Vt.T@U.T)])@U.T
    p=b-R@a
    pivot_error_mm=(np.linalg.norm(local@R.T+p-pivots,axis=1)*1000.)
    measured_axis=np.asarray(s35["hinge_axis_mean_world"],dtype=float)
    predicted_axis=R@joints[FINGERS[0]]["axis"]
    axis_deg=float(np.rad2deg(np.arccos(np.clip(np.dot(predicted_axis,measured_axis)/(np.linalg.norm(predicted_axis)*np.linalg.norm(measured_axis)),-1,1))))
    tcp=p+R@cfg["tcp_offset"]
    return (R,p),pivot_error_mm,axis_deg,tcp


def errors(q,goal,cfg,joints,chain):
    R,p=fk_chain(q,cfg,joints,chain)
    Rgoal,pgoal=goal
    rotvec=Rotation.from_matrix(Rgoal@R.T).as_rotvec()
    # Orientation angular error in radians scaled into units of meters.
    return np.r_[p-pgoal, rotvec*.12]


def quality(q,goal,cfg,joints,chain):
    R,p=fk_chain(q,cfg,joints,chain);Rt,pt=goal
    return float(np.linalg.norm(p-pt)*1000),float(np.linalg.norm(Rotation.from_matrix(Rt@R.T).as_rotvec())*180/math.pi)


def solve_ik(goal, seeds, cfg,joints,chain,low,high, max_nfev=220):
    candidates=[]
    for seed in seeds:
        seed=np.clip(seed,low+1e-5,high-1e-5)
        solution=least_squares(errors,seed,args=(goal,cfg,joints,chain),
          bounds=(low,high),max_nfev=max_nfev,xtol=2e-9,ftol=2e-9,gtol=2e-9)
        poserr,angerr=quality(solution.x,goal,cfg,joints,chain)
        candidates.append((poserr+angerr, solution.x.copy(),poserr,angerr,int(solution.nfev)))
    candidates.sort(key=lambda x:(x[0],np.linalg.norm(x[1]-seeds[0])))
    return candidates[0]


def parse_log(path):
    if not path or not path.is_file():return None
    text=path.read_text(encoding="utf-8",errors="replace")
    matches=re.findall(r"FINAL_ARM_Q_RAD=(\[[^\r\n]+?\])",text)
    if not matches:return None
    try: q=np.array(json.loads(matches[-1]),dtype=float)
    except Exception:return None
    return q if q.shape==(6,) and np.isfinite(q).all() else None


def cube_triangle_sat(verts,triangles,cube):
    """Broadphase + triangle/AABB separating-axis test; cube STATIC at current yaw."""
    h=np.full(3,0.020)
    tri=verts[triangles]-cube[None,None,:]
    mask=(np.all(tri.min(1)<=h,axis=1) & np.all(tri.max(1)>=-h,axis=1))
    tri=tri[mask]
    if len(tri)==0:return 0
    e0=tri[:,1,:]-tri[:,0,:];e1=tri[:,2,:]-tri[:,1,:];e2=tri[:,0,:]-tri[:,2,:]
    axes=[np.broadcast_to(a,tri[:,0,:].shape) for a in np.eye(3)]
    axes.append(np.cross(e0,e1))
    for edge in (e0,e1,e2):
        for a in np.eye(3):axes.append(np.cross(edge,a))
    valid=np.ones(len(tri),dtype=bool)
    for a in axes:
        pp=np.sum(tri*a[:,None,:],axis=2)
        r=np.sum(np.abs(a)*h[None,:],axis=1)
        valid&=(pp.min(1)<=r+1e-10)&(pp.max(1)>=-r-1e-10)
        if not valid.any():return 0
    return int(valid.sum())


def load_mesh(npz):
    all_v=[];tri=[];finger_start=None;off=0
    with np.load(npz,allow_pickle=False) as ar:
        for i,name in enumerate(BODIES):
            v=np.asarray(ar[name+"_world_vertices_m"],dtype=float)
            f=np.asarray(ar[name+"_triangle_indices"],dtype=np.int64)
            if v.ndim!=2 or v.shape[1]!=3 or f.ndim!=2 or f.shape[1]!=3 or f.min()<0 or f.max()>=len(v):
                stop("BAD_CLEAN_MESH="+name)
            if i==1:finger_start=off
            all_v.append(v);tri.append(f+off);off+=len(v)
    return np.concatenate(all_v), np.concatenate(tri), finger_start


def rigid_mesh_screen(q0,q1, cfg,joints,chain,baseline_pose,all_v,faces,cube, samples=19):
    R0,p0=baseline_pose
    # Remap each current authored mesh vertex to moving gripper_base frame.
    local=(all_v-p0[None,:])@R0
    min_z=float('inf');worst_table=0;worst_cube=0;bad_count=0
    for t in np.linspace(0.,1.,samples):
        q=q0*(1-t)+q1*t
        R,p=fk_chain(q,cfg,joints,chain)
        verts=local@R.T+p
        zmin=float(verts[:,2].min());min_z=min(min_z,zmin)
        if zmin<.005:worst_table+=1
        cube_hits=cube_triangle_sat(verts,faces,cube)
        worst_cube=max(worst_cube,cube_hits)
        if cube_hits:bad_count+=1
    return dict(samples=samples,minimum_gripper_vertex_z_mm=round(min_z*1000,3),
      poses_below_5mm_table_clearance=worst_table,
      max_static_mesh_triangle_cube_hits=worst_cube,
      poses_with_cube_surface_intersection=bad_count,
      gripper_only_discrete_screen_pass=worst_table==0 and worst_cube==0)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--urdf",type=Path,required=True)
    ap.add_argument("--robot-cfg",type=Path,required=True)
    ap.add_argument("--stage35",type=Path,required=True)
    ap.add_argument("--stage36",type=Path,required=True)
    ap.add_argument("--stage39",type=Path,required=True)
    ap.add_argument("--mesh",type=Path,required=True)
    ap.add_argument("--baseline-log",type=Path)
    ap.add_argument("--out",type=Path,required=True)
    ap.add_argument("--max-candidates",type=int,default=12)
    args=ap.parse_args()
    if not 1<=args.max_candidates<=50:stop("MAX_CANDIDATES_OUT_OF_RANGE")
    for path in (args.urdf,args.robot_cfg,args.stage35,args.stage36,args.stage39,args.mesh):
        if not path.is_file():stop("MISSING_INPUT="+str(path))
    cfg=read_cfg(args.robot_cfg);joints,children,chain,masses=parse_urdf(args.urdf)
    s35=json.loads(args.stage35.read_text());s36=json.loads(args.stage36.read_text());s39=json.loads(args.stage39.read_text())
    if [s35.get("stage"),s36.get("stage"),s39.get("stage")] != ["3C35","3C36","3C39"]:
        stop("PREREQUISITE_STAGE_MISMATCH")
    if any(x.get("no_arm_motion") is not True for x in (s39,)) or s35.get("no_finger_motion") is not True:
        stop("UNEXPECTED_PRIOR_MOTION")
    print("STAGE=3C40 MODE=OFFLINE_URDF_IK_AND_DISCRETE_GRIPPER_ONLY_MESH_SCREEN",flush=True)
    print("STAGE40_ISAAC_LAB_STARTED=False ROBOT_MOTION_COMMANDED=False FINGER_MOTION_COMMANDED=False",flush=True)
    print("STAGE40_ASSETS_MODIFIED=False SELFCOLLISIONS_VALIDATED=False",flush=True)
    print("STAGE40_ARM_JOINTS="+repr(chain),flush=True)
    print("STAGE40_BASE_QUAT_XYZW="+repr(cfg["LEXIUM_BASE_ROTATION_XYZW"]),flush=True)
    print("STAGE40_TCP_OFFSET_M="+repr(cfg["tcp_offset"].tolist()),flush=True)

    (R0,p0),pivot_mm,axis_deg,tcp=measured_pose(joints,s35,cfg)
    if pivot_mm.max()>.5 or axis_deg>.3:
        stop("HINGE_POSE_RECONSTRUCTION_MISMATCH")
    reference_tcp=np.array([.30004824285467957,-.2999579338804156,.19986460732783992])
    tcp_mm=float(np.linalg.norm(tcp-reference_tcp)*1000.)
    if tcp_mm>2.0:stop("TCP_FROM_HINGE_POSE_MISMATCH")
    print(f"STAGE40_BASELINE_HINGE_FIT_MAX_MM={pivot_mm.max():.7f} AXIS_ERROR_DEG={axis_deg:.6f} TCP_CROSSCHECK_MM={tcp_mm:.5f}",flush=True)

    low=np.array([joints[n]["limits"][0] for n in chain]); high=np.array([joints[n]["limits"][1] for n in chain])
    qhome=cfg["q_home"]
    qlog=parse_log(args.baseline_log)
    observed_calibrated=False
    if qlog is not None:
        pmm,odeg=quality(qlog,(R0,p0),cfg,joints,chain)
        observed_calibrated=bool(pmm<2. and odeg<.5)
        print(f"STAGE40_LOGGED_ARM_Q_FOUND=True BASELINE_LOG_FK_ERROR_MM={pmm:.6f} BASELINE_LOG_FK_ORI_DEG={odeg:.6f} BASELINE_LOG_MATCH={observed_calibrated}",flush=True)
        if not observed_calibrated:stop("URDF_PHYSX_LOGGED_Q_COORDINATES_MISMATCH_NO_IK_SCREEN")
    else:
        print("STAGE40_LOGGED_ARM_Q_FOUND=False BASELINE_PHYSX_Q_NOT_INDEPENDENTLY_VERIFIED=True",flush=True)

    rng=np.random.default_rng(34040)
    baseline_seeds=[qlog if qlog is not None else qhome, qhome,np.zeros(6)]
    baseline_seeds += [qhome+rng.normal(0,.45,6) for _ in range(7)]
    _,qbase,bpos,bdeg,_=solve_ik((R0,p0),baseline_seeds,cfg,joints,chain,low,high)
    baseline_valid=bpos<1.0 and bdeg<.2
    print(f"STAGE40_BASELINE_IK_ERROR_MM={bpos:.7f} ERROR_DEG={bdeg:.7f} Q_RAD={np.round(qbase,6).tolist()} VALID={baseline_valid}",flush=True)
    if not baseline_valid:stop("BASELINE_URDF_IK_FAILED")
    if qlog is not None:
        # Keep actual logged pose for interpolation, rather than an alternative branch.
        qbase=qlog

    mesh,faces,_=load_mesh(args.mesh)
    cube=np.asarray(s39["cube_center_world_m"],dtype=float)
    if cube.shape!=(3,):stop("BAD_CUBE")
    rows=s39["top_static_candidates"][:args.max_candidates]
    if not rows:stop("NO_STAGE39_STATIC_CANDIDATES")
    centers=s36["candidate_aperture_centers_top_10"]
    results=[]
    for index,row in enumerate(rows):
        seedix=int(row["seed_candidate_index"])
        if not (0<=seedix<len(centers)):stop("BAD_SEED_INDEX")
        anchor=np.asarray(centers[seedix]["virtual_cube_center_world_m"],dtype=float)
        angles=np.asarray(row["world_angles_deg"],dtype=float)
        shift=np.asarray(row["world_translation_offset_from_seed_m"],dtype=float)
        Rextra=Rotation.from_euler("xyz",angles,degrees=True).as_matrix()
        # Identical rigid transform to Stage39: (v - anchor) @ R.T + cube + shift.
        goal_R=Rextra@R0
        goal_p=Rextra@(p0-anchor)+cube+shift
        goal=(goal_R,goal_p)
        seeds=[qbase, qhome, np.zeros(6), np.clip(qbase+np.array([0.,-.4,.6,-.4,0.,.3]),low,high)]
        seeds += [qbase+rng.normal(0,.5,6) for _ in range(5)]
        _,qsol,poserr,orierr,nfev=solve_ik(goal,seeds,cfg,joints,chain,low,high)
        ik_pass=bool(poserr<=2. and orierr<=1.)
        margin=float(np.min(np.minimum(qsol-low,high-qsol)))
        torque=gravity_efforts(qsol,cfg,joints,children,masses)
        path_torque=np.array([gravity_efforts(qbase*(1-a)+qsol*a,cfg,joints,children,masses) for a in np.linspace(0.,1.,21)])
        peak_est=float(np.abs(path_torque).max())
        estimated_torque_pass=bool(peak_est<85.)
        meshcheck=None
        if ik_pass:
            meshcheck=rigid_mesh_screen(qbase,qsol,cfg,joints,chain,(R0,p0),mesh,faces,cube,samples=19)
        print("STAGE40_IK_CANDIDATE index={} seed={} pose_mm={:.3f} ori_deg={:.3f} min_joint_margin_deg={:.2f} gravity_path_peak_nm={:.2f} discrete_gripper_path_pass={} IK_PASS={}".format(
          index,seedix,poserr,orierr,margin*180/math.pi,peak_est,
          meshcheck["gripper_only_discrete_screen_pass"] if meshcheck else "NOT_TESTED",ik_pass),flush=True)
        results.append(dict(index=index,source_seed_candidate=seedix,stage39_world_angles_deg=angles.tolist(),
           stage39_translation_offset_m=shift.tolist(),target_gripper_base_world_xyz_m=goal_p.tolist(),
           target_gripper_base_world_quat_xyzw=Rotation.from_matrix(goal_R).as_quat().tolist(),
           q_candidate_rad=qsol.tolist(),urdf_fk_position_error_mm=round(poserr,5),
           urdf_fk_orientation_error_deg=round(orierr,5), min_joint_limit_margin_deg=round(margin*180/math.pi,4),
           ik_pose_match=ik_pass,gravity_peak_estimate_path_nm=round(peak_est,4),
           gravity_estimate_under_85nm=estimated_torque_pass,gripper_discrete_mesh_path=meshcheck,
           geometry_stage39_bilateral_clearance_mm=[row["group_A_face_clearance_mm"],row["group_B_opposite_face_clearance_mm"]],
           stage39_static_triangle_hits=row.get("cube_triangle_hits"),
           warning="UNVALIDATED_WITH_PHYSX_AND_ARM_COLLISIONS"))
    matched=sum(bool(row["ik_pose_match"]) for row in results)
    safe_gripper=sum(bool(r["gripper_discrete_mesh_path"] and r["gripper_discrete_mesh_path"]["gripper_only_discrete_screen_pass"]) for r in results)
    report=dict(stage="3C40",mode="offline_urdf_ik_gripper_mesh_discrete_precheck",
       no_isaac_lab=True,no_physics_simulation=True,no_robot_motion=True,no_finger_motion=True,
       self_collisions_enabled=False,urdf_used=str(args.urdf),robot_cfg_used=str(args.robot_cfg),
       arm_joint_names=list(ARM),arm_urdf_limits_rad=[joints[n]["limits"] for n in chain],
       q_home_rad=qhome.tolist(),baseline_estimated_q_rad=qbase.tolist(),
       logged_arm_q_used=qlog is not None,logged_q_to_measured_pose_calibrated=observed_calibrated,
       measured_baseline_gripper_base_world_xyz_m=p0.tolist(),
       measured_baseline_gripper_base_world_quat_xyzw=Rotation.from_matrix(R0).as_quat().tolist(),
       measured_baseline_tcp_world_xyz_m=tcp.tolist(),
       baseline_hinge_reconstruction_max_error_mm=float(pivot_mm.max()),
       baseline_hinge_axis_discrepancy_deg=axis_deg,baseline_tcp_crosscheck_error_mm=tcp_mm,
       cube_center_world_m=cube.tolist(),
       input_stage39=str(args.stage39),input_stage39_may_be_example="example" in str(args.stage39).lower(),
       candidates_checked=len(results),ik_pose_matches=matched,gripper_only_discrete_path_precheck_passes=safe_gripper,
       candidate_results=results,
       arm_fk_physx_independently_validated=observed_calibrated,
       full_robot_collision_validated=False,physx_colliders_validated=False,whole_trajectory_collision_validated=False,
       force_closure_or_grasp_validated=False,physical_grasp_permission=False,
       disclaimer="Offline URDF FK/IK and sampled frozen-gripper authored-mesh path tests only. "
                  "NO physical motion authorized. Arm link/robot self collisions and PhysX convex decomposition absent; "
                  "IK and sampled meshes alone cannot certify continuous motion, torque under dynamics or grasp.")
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(f"STAGE40_REPORT_JSON={args.out}",flush=True)
    print(f"STAGE40_IK_POSE_MATCHES={matched}/{len(results)} GRIPPER_ONLY_DISCRETE_PATH_PRECHECKS={safe_gripper}/{len(results)}",flush=True)
    print("STAGE40_NO_GRASP_PERMISSION=True",flush=True)
    print("LEXIUM_STAGE3C40_OFFLINE_IK_AUDIT_COMPLETE=True",flush=True)
    print("RESULT=STAGE3C40_OFFLINE_IK_RECORDED_NOT_PHYSX_VALIDATED_NOT_GRASP_VALIDATED",flush=True)

if __name__ == "__main__":
    main()
