#!/usr/bin/env python3
"""Stage 3C.45: OFFLINE candidate-5 lifted pre-grasp IK/whole-arm mesh screen.

Uses verified Stage44 HOME joint initialization as the beginning of each
HYPOTHETICAL joint-space interpolation. Lifts the Stage40 candidate-5 gripper
base goal in world Z without changing its orientation. Tries independently
solved IK per lift, using Stage40 candidate-5 q only as an IK seed.

For every joint interpolation sample: authored URDF STL arm and frozen finger
meshes are tested against the inflated finite table and cube volume. This is
STATIC DISCRETE mesh geometry, NOT PhysX contact, not continuous collision,
not an approved trajectory, and not a grasp. No robot commands are sent.

IMPORTANT: Candidate-5 target came from a Stage39 EXAMPLE shortlist; it is an
unvalidated geometric reference and the finger aperture is NOT a 40mm grasp.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from lexium_stage3c40_ik_reachability_audit import (
    ARM, BODIES, read_cfg, parse_urdf, measured_pose,
    all_fk, errors, quality, gravity_efforts,
)
from lexium_stage3c41_full_arm_collision_audit import (
    ARM_LINKS, BODY_NAMES, load_urdf_arm_collision,
    load_frozen_gripper, triangle_aabb_intersections,
    aabb_distance, graph_distance,
)


def fail(reason):
    raise SystemExit('STAGE45_FAIL=' + str(reason))


def get_options():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--robot-cfg', type=Path, required=True)
    p.add_argument('--stage35', type=Path, required=True)
    p.add_argument('--stage40', type=Path, required=True)
    p.add_argument('--stage44', type=Path, required=True)
    p.add_argument('--clean-npz', type=Path, required=True)
    p.add_argument('--out', type=Path, default=Path('/tmp/lexium_stage3c45_offline_pregrasp_report.json'))
    p.add_argument('--lift-mm', type=str, default='60,70,80,90,100,120')
    p.add_argument('--samples', type=int, default=41)
    p.add_argument('--table-margin-mm', type=float, default=5.0)
    p.add_argument('--cube-margin-mm', type=float, default=0.0)
    return p.parse_args()


def mesh_box_screen(v, faces, blo, bhi):
    """Broadphase, vertices within box, and triangle surface intersection SAT.

    The triangle SAT tests authored triangles versus AABB, but cannot prove
    continuous swept clearance or the geometry of PhysX convex approximations.
    """
    vmin=v.min(axis=0); vmax=v.max(axis=0)
    if np.any(vmin>bhi) or np.any(vmax<blo):
        return 0,0
    inside=int(np.count_nonzero(np.all((v>=blo)&(v<=bhi),axis=1)))
    # AABBs can overlap with no mesh intersection, so run narrow-phase for
    # every shape passing broadphase even when vertex count is zero.
    triangles=triangle_aabb_intersections(v, faces, blo, bhi)
    return inside, triangles


def body_geometry(q, cfg, joints, children, masses, geometries):
    poses,_,_=all_fk(q,cfg,joints,children,masses)
    shapes={}
    for name in BODY_NAMES:
        R,p=poses[name]
        g=geometries[name]
        verts=g['vertices']@R.T+p
        shapes[name]=(verts,g['triangles'])
    return shapes


def probe_pose(q, cfg, joints, children, masses, geometries, tablelo,tablehi,cubelo,cubehi,pairs):
    shapes=body_geometry(q,cfg,joints,children,masses,geometries)
    table_hits={};cube_hits={};min_arm=1e8;min_gripper=1e8
    bounds={}
    for name,(pts,faces) in shapes.items():
        lo=pts.min(0);hi=pts.max(0);bounds[name]=(lo,hi)
        if name in ARM_LINKS:min_arm=min(min_arm,float(lo[2]))
        else:min_gripper=min(min_gripper,float(lo[2]))
        inside,tri=mesh_box_screen(pts,faces,tablelo,tablehi)
        if inside or tri:table_hits[name]={'inside_vertices':inside,'triangle_hits':tri}
        inside,tri=mesh_box_screen(pts,faces,cubelo,cubehi)
        if inside or tri:cube_hits[name]={'inside_vertices':inside,'triangle_hits':tri}
    unresolved=[];near=[]
    for a,b in pairs:
        dist,overlap=aabb_distance(bounds[a],bounds[b])
        if overlap: unresolved.append([a,b])
        elif dist<.005:near.append([a,b,round(dist*1000,3)])
    return {
        'table_hits_per_body':table_hits,
        'cube_hits_per_body':cube_hits,
        'min_arm_z_m':min_arm,
        'min_gripper_z_m':min_gripper,
        'self_aabb_overlap_unresolved_pairs':unresolved,
        'self_aabb_within_5mm_unresolved_pairs':near,
    }


def main():
    args=get_options()
    for f in [args.urdf,args.robot_cfg,args.stage35,args.stage40,args.stage44,args.clean_npz]:
        if not f.is_file():fail('MISSING_INPUT='+str(f))
    if not 5<=args.samples<=101:fail('SAMPLES_RANGE_5_TO_101_REQUIRED')
    if args.table_margin_mm<0 or args.table_margin_mm>20:fail('TABLE_MARGIN_RANGE')
    if args.cube_margin_mm<0 or args.cube_margin_mm>5:fail('CUBE_MARGIN_RANGE')
    lifts=[]
    for raw in args.lift_mm.split(','):
        try: lift=float(raw)
        except ValueError:fail('INVALID_LIFT='+repr(raw))
        if not 30<=lift<=250:fail('LIFT_OUTSIDE_SAFE_OFFLINE_SEARCH_BOUNDS='+str(lift))
        lifts.append(lift)
    lifts=sorted(set(lifts))
    if len(lifts)>15:fail('TOO_MANY_LIFTS')

    st44=json.loads(args.stage44.read_text(encoding='utf8'))
    st40=json.loads(args.stage40.read_text(encoding='utf8'))
    st35=json.loads(args.stage35.read_text(encoding='utf8'))
    if st44.get('stage')!='3C44' or not st44.get('explicit_home_fk_match') or not st44.get('robot_cfg_mount_match'):
        fail('STAGE44_NOT_VERIFIED_HOME_FK')
    if st44.get('arm_trajectory_commanded') or st44.get('candidate1_replayed'):
        fail('STAGE44_SHOULD_BE_HOME_ONLY')
    if st40.get('stage')!='3C40' or st35.get('stage')!='3C35':fail('INVALID_PRIOR_STAGE')
    if st40.get('physical_grasp_permission') is not False:fail('PRIOR_REPORT_GRASP_POLICY')
    cfg=read_cfg(args.robot_cfg)
    joints,children,chain,masses=parse_urdf(args.urdf)
    qhome=np.asarray(st44['arm_q_immediately_after_home_write_rad'],dtype=float)
    if qhome.shape!=(6,) or not np.allclose(qhome,cfg['q_home'],atol=1e-4):
        fail('HOME_JOINT_CONFIG_DISAGREEMENT')
    if max(abs(np.asarray(st44['arm_q_after_one_physx_step_rad'])-qhome))>.02:
        fail('PHYSX_HOME_NOT_STABLE_AFTER_ONE_STEP')
    target=next((r for r in st40.get('candidate_results',[]) if r['index']==5),None)
    if not target or target.get('ik_pose_match') is not True:
        fail('STAGE40_CANDIDATE5_MISSING')
    if st40.get('input_stage39_may_be_example') is not True:
        print('STAGE45_NOTE=STAGE39_EXAMPLE_FLAG_FALSE_VERIFY_PROVENANCE_SEPARATELY',flush=True)
    qseed=np.asarray(target['q_candidate_rad'],dtype=float)
    Rgoal=Rotation.from_quat(np.asarray(target['target_gripper_base_world_quat_xyzw'],dtype=float)).as_matrix()
    pgoal=np.asarray(target['target_gripper_base_world_xyz_m'],dtype=float)
    if qseed.shape!=(6,) or pgoal.shape!=(3,):fail('CANDIDATE5_BAD_TARGET_SHAPE')
    low=np.asarray([joints[n]['limits'][0] for n in ARM]); high=np.asarray([joints[n]['limits'][1] for n in ARM]);
    if np.any(qhome<low) or np.any(qhome>high):fail('HOME_OUTSIDE_LIMITS')

    (Rbaseline,pbaseline),pivot_err,axis_err,_=measured_pose(joints,st35,cfg)
    if max(pivot_err)>.5 or axis_err>.3:
        fail('FROZEN_GRIPPER_BASELINE_HINGE_CALIBRATION_FAILED')
    arm=load_urdf_arm_collision(args.urdf)
    grip=load_frozen_gripper(args.clean_npz,(Rbaseline,pbaseline))
    geometries={**arm,**grip}
    table=np.array([[.17,1.07],[-.6,.6],[-.4,0.]],dtype=float)
    tablelo=table[:,0]-args.table_margin_mm/1000
    tablehi=table[:,1]+args.table_margin_mm/1000
    cube=np.asarray(st40['cube_center_world_m'],dtype=float)
    cubelo=cube-.02-args.cube_margin_mm/1000
    cubehi=cube+.02+args.cube_margin_mm/1000
    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child'])
        adj.setdefault(j['child'],set()).add(j['parent'])
    pairs=[(a,b)for i,a in enumerate(BODY_NAMES) for b in BODY_NAMES[i+1:] if graph_distance(a,b,adj)>1]
    print('STAGE=3C45 MODE=OFFLINE_LIFTED_PREGRASP_FULL_ARM_MESH_SCREEN',flush=True)
    print('STAGE45_ISAAC_LAB_STARTED=False PHYSICAL_MOTION_COMMANDED=False ASSETS_MODIFIED=False',flush=True)
    print('STAGE45_SOURCE_CANDIDATE5_STAGE39_MAY_BE_EXAMPLE='+str(bool(st40.get('input_stage39_may_be_example'))),flush=True)
    print('STAGE45_STAGE44_HOME_FK_MATCH=True',flush=True)
    print('STAGE45_FULL_PHYSX_COLLISIONS_VALIDATED=False',flush=True)
    print('STAGE45_Q_HOME_RAD='+json.dumps(qhome.tolist()),flush=True)
    print('STAGE45_Q_CANDIDATE5_UNSAFE_SEED_RAD='+json.dumps(qseed.tolist()),flush=True)

    # HOME is a verified configuration of the articulation itself, but Stage44
    # ran WITHOUT a table. Thus even HOME must undergo the CURRENT table test.
    initial=probe_pose(qhome,cfg,joints,children,masses,geometries,tablelo,tablehi,cubelo,cubehi,pairs)
    if initial['table_hits_per_body'] or initial['cube_hits_per_body']:
        fail('HOME_INTERSECTS_TABLE_OR_CUBE_IN_AUTHORED_GEOMETRY_SCREEN')
    results=[]
    for lift_mm in lifts:
        pose_goal=(Rgoal,pgoal+np.array([0.,0.,lift_mm/1000]))
        # Staged qseed is an initial guess only. No commands to this q occur.
        sol=least_squares(errors,np.clip(qseed,low+1e-4,high-1e-4),
                          args=(pose_goal,cfg,joints,chain),bounds=(low,high),
                          max_nfev=300,xtol=1e-10,ftol=1e-10,gtol=1e-10)
        q=sol.x
        pe_mm,oe_deg=quality(q,pose_goal,cfg,joints,chain)
        margin_deg=float(np.rad2deg(np.min(np.minimum(q-low,high-q))))
        matched=bool(pe_mm<.5 and oe_deg<.5 and margin_deg>5)
        row={
            'lift_mm':lift_mm,'target_gripper_base_world_xyz_m':pose_goal[1].tolist(),
            'q_ik_rad':q.tolist(),'ik_position_error_mm':pe_mm,
            'ik_orientation_error_deg':oe_deg,'min_joint_limit_margin_deg':margin_deg,
            'ik_matched_with_5deg_margin':matched,'ik_solver_nfev':int(sol.nfev),
            'source_target_is_unvalidated_stage39_geometry':True,
            'samples':None,'sampled_table_hit_count':None,'sampled_cube_hit_count':None,
            'sampled_self_aabb_overlap_count_unresolved':None,
            'sampled_vertex_and_triangle_table_cube_clear':False,
            'waypoints_approved_for_simulation_motion':False,
        }
        if matched:
            samples=[]; peakgravity=0.
            for k,a in enumerate(np.linspace(0,1,args.samples)):
                qk=qhome*(1-a)+q*a
                gravity=np.asarray(gravity_efforts(qk,cfg,joints,children,masses),dtype=float)
                peakgravity=max(peakgravity,float(np.max(np.abs(gravity))))
                frame=probe_pose(qk,cfg,joints,children,masses,geometries,tablelo,tablehi,cubelo,cubehi,pairs)
                if not np.isfinite([frame['min_arm_z_m'],frame['min_gripper_z_m'],peakgravity]).all():
                    fail('NONFINITE_SIMULATED_GEOMETRY')
                samples.append({
                    'index':k,'fraction':round(float(a),5),
                    'table_hits_per_body':frame['table_hits_per_body'],
                    'cube_hits_per_body':frame['cube_hits_per_body'],
                    'self_aabb_overlap_pair_count_unresolved':len(frame['self_aabb_overlap_unresolved_pairs']),
                    'min_arm_z_mm':round(frame['min_arm_z_m']*1000,3),
                    'min_gripper_z_mm':round(frame['min_gripper_z_m']*1000,3),
                })
            nt=sum(bool(x['table_hits_per_body']) for x in samples)
            nc=sum(bool(x['cube_hits_per_body']) for x in samples)
            ns=sum(x['self_aabb_overlap_pair_count_unresolved']>0 for x in samples)
            row.update({'samples':samples,'sampled_table_hit_count':nt,
                        'sampled_cube_hit_count':nc,'sampled_self_aabb_overlap_count_unresolved':ns,
                        'gravity_path_peak_estimate_nm':peakgravity,
                        'sampled_vertex_and_triangle_table_cube_clear':bool(nt==0 and nc==0 and peakgravity<85),
                        'target_gripper_min_z_mm':samples[-1]['min_gripper_z_mm']})
        print('STAGE45_LIFT lift_mm={:.1f} ik={} pose_mm={:.4f} angle_deg={:.4f} joint_margin_deg={:.2f} table_hit_frames={} cube_hit_frames={} self_aabb_unresolved_frames={} nominal_screen_clear={}'.format(
            lift_mm,matched,pe_mm,oe_deg,margin_deg,row['sampled_table_hit_count'],row['sampled_cube_hit_count'],
            row['sampled_self_aabb_overlap_count_unresolved'],row['sampled_vertex_and_triangle_table_cube_clear']),flush=True)
        results.append(row)
    clear=[r for r in results if r['sampled_vertex_and_triangle_table_cube_clear']]
    report={
        'stage':'3C45','mode':'offline_lifted_pregrasp_from_explicit_home',
        'no_isaac_lab':True,'no_physical_motion':True,'assets_modified':False,
        'stage44_home_fk_verified':True,'stage44_q_after_write_rad':qhome.tolist(),
        'stage44_actual_candidate1_q_recovered':False,
        'stage40_candidate_reference_index':5,
        'stage40_stage39_input_may_be_example':bool(st40.get('input_stage39_may_be_example')),
        'stage40_reference_world_gripper_base_position_m':pgoal.tolist(),
        'stage40_reference_world_gripper_base_quat_xyzw':target['target_gripper_base_world_quat_xyzw'],
        'reference_fingers_frozen':True,'gripper_40mm_opening_confirmed':False,
        'cube_center_world_m':cube.tolist(),'cube_40mm_assumed':True,
        'table_bounds_m':table.tolist(),'table_margin_mm':args.table_margin_mm,
        'cube_margin_mm':args.cube_margin_mm,
        'joint_path_model':'straight_joints_from_explicit_HOME; STATIC sampling only',
        'samples_per_path':args.samples,
        'home_authored_mesh_initial_screen':initial,
        'tested_lifts_mm':lifts,
        'target_rows':results,
        'number_static_sampled_table_cube_clear':len(clear),
        'shortlisted_unvalidated_lifts_mm':[r['lift_mm'] for r in clear],
        'all_physx_collision_shapes_validated':False,
        'robot_self_collision_narrowphase_validated':False,
        'continuous_swept_path_validated':False,
        'pregrasp_contact_or_40mm_grasp_validated':False,
        'physical_motion_permission':False,
        'no_grasp_permission':True,
        'disclaimer':'Only offline URDF/STL and authored USD mesh static discrete checks for hypothetically elevated Stage40 candidate-5 gripper pose (which is sourced from an example Stage39 shortlist). Reference fingers frozen; neither actual PhysX contact nor whole-robot self-narrowphase nor continuous trajectories nor 40mm aperture/grasp demonstrated. All q values are DATA ONLY, never execution instructions.',
    }
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print('STAGE45_NOMINAL_STATIC_CLEAR_LIFTS_MM='+json.dumps(report['shortlisted_unvalidated_lifts_mm']),flush=True)
    print('STAGE45_PHYSICAL_MOTION_PERMISSION=False',flush=True)
    print('STAGE45_GRASP_PERMISSION=False',flush=True)
    print('STAGE45_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C45_OFFLINE_PREGRASP_AUDIT_COMPLETE=True',flush=True)

if __name__=='__main__':main()
