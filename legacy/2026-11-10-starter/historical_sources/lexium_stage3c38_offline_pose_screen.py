#!/usr/bin/env python3
"""Stage 3C.38: offline, read-only authored-mesh integrity & rigid-pose screen.

This script never imports Isaac Lab/PhysX, moves joints, edits robot assets, or
certifies a grasp. All transforms are hypothetical rigid whole-gripper poses.
Checks: exclude descendant finger geometry accidentally bundled in base;
re-evaluate table vertex penetration, and perform coarse world-frame orientation
hypothesis screening about UNCALIBRATED Stage36 center candidates.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np

BODIES = ['gripper_base_1','gripper_finger_1_1','gripper_finger_2_1','gripper_finger_3_1','gripper_finger_4_1']
FINGERS=BODIES[1:]


def fatal(msg):
    raise SystemExit('STAGE38_INPUT_ERROR='+str(msg))


def rotation_xyz_deg(roll,pitch,yaw):
    a,b,c=np.deg2rad([roll,pitch,yaw]);ca,sa=np.cos(a),np.sin(a);cb,sb=np.cos(b),np.sin(b);cc,sc=np.cos(c),np.sin(c)
    rx=np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
    ry=np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
    rz=np.array([[cc,-sc,0],[sc,cc,0],[0,0,1]])
    return rz@ry@rx


def json_nice(x):return json.dumps(x, ensure_ascii=False, indent=2)


def inside_table(pts,bounds):
    x0,x1,y0,y1,z0,z1=bounds
    return ((pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>z0)&(pts[:,2]<z1)).sum()


def table_stats(pts,bounds):
    return int(inside_table(pts,bounds)),float(pts[:,2].min())


def count_cube_triangle_hits(all_vertices, triangles, cube_center, halfwidth=0.020):
    """Exact triangle-vs-axis-aligned-box SAT at one hypothetical pose.

    Tests triangle surfaces, not enclosure of the box by a closed mesh,
    continuous trajectories, or true PhysX decomposition. Returns count.
    """
    h = np.array([halfwidth]*3, dtype=np.float64)
    v = all_vertices[triangles] - cube_center[None,None,:]
    broad = np.all(v.min(axis=1) <= h, axis=1) & np.all(v.max(axis=1) >= -h, axis=1)
    v = v[broad]
    if not len(v):return 0
    e0 = v[:,1,:] - v[:,0,:]
    e1 = v[:,2,:] - v[:,1,:]
    e2 = v[:,0,:] - v[:,2,:]
    edges = [e0,e1,e2]
    axes=[np.broadcast_to(k,v[:,0,:].shape) for k in np.eye(3)]
    axes.append(np.cross(e0,e1))
    for e in edges:
        for unit in np.eye(3):
            axes.append(np.cross(e,unit[None,:]))
    intersects = np.ones(len(v),dtype=bool)
    for a in axes:
        p=np.sum(v*a[:,None,:],axis=2)
        radius=np.sum(np.abs(a)*h[None,:],axis=1)
        intersects &= (p.min(axis=1) <= radius + 1e-10) & (p.max(axis=1) >= -radius - 1e-10)
        if not np.any(intersects):return 0
    return int(intersects.sum())


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--npz',required=True,type=Path)
    ap.add_argument('--gate-json',required=True,type=Path)
    ap.add_argument('--stage36-json',required=True,type=Path)
    ap.add_argument('--report',required=True,type=Path)
    ap.add_argument('--clean-npz',required=True,type=Path)
    args=ap.parse_args()
    src=json.loads(args.gate_json.read_text())
    prev=json.loads(args.stage36_json.read_text())
    if src.get('stage')!='3C37' or prev.get('stage')!='3C36':fatal('STAGE_MISMATCH')
    if src.get('no_arm_descent') is not True or src.get('no_finger_motion') is not True:fatal('UNEXPECTED_ACTIVE_MOTION')
    if len(prev.get('candidate_aperture_centers_top_10',[]))!=10:fatal('TOP10_MISSING')
    table=np.asarray(src['table_bounds_xyz_m'],dtype=float)
    cube=np.asarray(src['cube_center_world_m'],dtype=float)
    if table.shape!=(6,) or cube.shape!=(3,):fatal('BAD_BOUNDS_OR_CUBE')
    print('STAGE=3C38 MODE=OFFLINE_MESH_RECONSTRUCTION_AND_RIGID_POSE_SCREEN',flush=True)
    print('STAGE38_ROBOT_SIMULATION_STARTED=False FINGER_MOTION_COMMANDED=False ARM_DESCENT_COMMANDED=False',flush=True)
    with np.load(args.npz,allow_pickle=False) as arch:
        meshes={}
        for name in BODIES:
            v=np.asarray(arch[name+'_world_vertices_m'],dtype=np.float64)
            f=np.asarray(arch[name+'_triangle_indices'],dtype=np.int32)
            if len(v)==0 or len(f)==0 or v.shape[1]!=3 or f.shape[1]!=3 or f.min()<0 or f.max()>=len(v):fatal('BAD_MESH_'+name)
            meshes[name]=(v.copy(),f.copy())

    # Base export uses a recursive USD subtree. Its four finger mesh descendants
    # should NOT be treated as part of the rigid palm when assessing motion.
    original_v,original_f=meshes[BODIES[0]]
    parts=src['per_body_mesh']['gripper_base_1']['part_metadata']
    if len(parts)!=5:fatal('EXPECTED_FIVE_BASE_EXPORT_PARTS')
    vstart=0; fstart=0; clean_parts=[]; duplicate_parts=[]
    for p in parts:
        nv=int(p['vertex_count']);nf=int(p['triangle_count'])
        vstop=vstart+nv; fstop=fstart+nf
        fv=original_v[vstart:vstop];ff=original_f[fstart:fstop]-vstart
        if np.any(ff<0) or np.any(ff>=nv):fatal('NONCONTIGUOUS_MESH_PART_'+p['path'])
        basename=p['path'].rsplit('/',1)[-1]
        if basename == 'gripper_base_1':
            clean_parts.append((fv.copy(),ff.copy()))
        else:
            duplicate_parts.append({'name':basename,'vertex_count':nv,'triangle_count':nf})
        vstart=vstop;fstart=fstop
    if vstart!=len(original_v) or fstart!=len(original_f):fatal('PART_COUNTS_MISMATCH')
    if len(clean_parts)!=1 or len(duplicate_parts)!=4:fatal('EXPECTED_ONE_PALM_AND_FOUR_DESCENDANTS')
    meshes[BODIES[0]]=clean_parts[0]
    print(f'STAGE38_RECONSTRUCTED_PALM vertices={len(meshes[BODIES[0]][0])} triangles={len(meshes[BODIES[0]][1])} removed_descendant_vertices={sum(p["vertex_count"] for p in duplicate_parts)}',flush=True)
    print('STAGE38_EXPORTED_BASE_CONTAINED_DESCENDANT_FINGER_MESHES=True',flush=True)
    for name in BODIES:
        v,f=meshes[name]
        print(f'STAGE38_CLEAN_BODY name={name} vertices={len(v)} triangles={len(f)} lowest_z_mm={1000*v[:,2].min():.3f}',flush=True)

    clean_arrays={}
    face_parts=[];v_off=0
    for name,(v,f) in meshes.items():
        clean_arrays[name+'_world_vertices_m']=v
        clean_arrays[name+'_triangle_indices']=f
        face_parts.append(f+v_off)
        v_off+=len(v)
    all_triangles=np.concatenate(face_parts,axis=0)
    args.clean_npz.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.clean_npz,**clean_arrays)
    # Table gate recheck: authored mesh vertices; no continuous collision proof.
    gates=[]
    for row in src['candidate_table_gates']:
        ix=row['candidate_index'];delta=np.asarray(row['hypothetical_rigid_translation_m'])
        by_body={};total=0
        for name in BODIES:
            n,minz=table_stats(meshes[name][0]+delta,table)
            by_body[name]={'table_vertices':n,'lowest_z_m':minz};total+=n
        r={'candidate_index':ix,'delta_xyz_m':delta.tolist(),'table_vertex_count_clean_mesh':total,
           'body_results':by_body,'passes_table_vertex_gate':total==0 and min(v['lowest_z_m'] for v in by_body.values())>=0.005}
        gates.append(r)
        print(f'STAGE38_CORRECTED_TABLE_GATE index={ix} table_vertices={total} palm_vertices={by_body[BODIES[0]]["table_vertices"]} palm_lowest_z_mm={1000*by_body[BODIES[0]]["lowest_z_m"]:.3f} result={"PASS_PRECHECK" if r["passes_table_vertex_gate"] else "REJECT"}',flush=True)
    # Search Rz(yaw)@Ry(pitch)@Rx(roll) around each *hypothetical*, uncalibrated
    # candidate working center, then translate center to cube center. These are
    # world rotations, NOT validated robot wrist commands or IK solutions.
    rolls=(-120,-90,-60,-30,0,30,60,90,120,180)
    pitches=(-120,-90,-60,-30,0,30,60,90,120,180)
    yaws=(-90,-45,0,45,90)
    # coarse AABB >= 5mm above z=0 is conservative for a table at z<=0,
    # and avoids expensive in-box checks in geometrically impossible poses.
    visible=[];orientations_per_cand=[];scanned=0
    all_v=np.concatenate([meshes[name][0] for name in BODIES],axis=0)
    finger_vertex_start=len(meshes['gripper_base_1'][0])
    for row in prev['candidate_aperture_centers_top_10']:
        ix=len(orientations_per_cand)
        anchor=np.asarray(row['virtual_cube_center_world_m'],dtype=float)
        relative=all_v-anchor[None,:]
        possible=0;possible_no_cube_triangle_hits=0;best_z=-1e9;chosen=None;closest_finger_mm=1e9
        for roll in rolls:
            for pitch in pitches:
                for yaw in yaws:
                    scanned+=1
                    R=rotation_xyz_deg(roll,pitch,yaw)
                    vv=relative@R.T+cube[None,:]
                    lowest=float(vv[:,2].min());best_z=max(best_z,lowest)
                    if lowest<0.005:
                        continue
                    # strict 5 mm table top clearance implies no table contact;
                    # remains a broad shape proxy, not full robot/swept safety.
                    collisions=int(inside_table(vv,table))
                    if collisions:
                        continue
                    possible+=1
                    cube_hits=count_cube_triangle_hits(vv, all_triangles, cube)
                    finger_pts=vv[finger_vertex_start:]
                    # Signed box distance is not exact point-mesh distance; a
                    # positive number is a lower bound on sample point clearance.
                    outside=np.maximum(np.abs(finger_pts-cube[None,:])-0.020,0.)
                    nearest_finger_mm=float(np.linalg.norm(outside,axis=1).min()*1000)
                    closest_finger_mm=min(closest_finger_mm,nearest_finger_mm)
                    if cube_hits==0:possible_no_cube_triangle_hits+=1
                    d={'candidate_index':ix,'roll_deg':roll,'pitch_deg':pitch,'yaw_deg':yaw,
                       'min_z_mm':round(lowest*1000,3),'table_vertex_hits':collisions,
                       'cube_triangle_overlap_count':cube_hits,
                       'nearest_sampled_finger_vertex_to_cube_box_mm':round(nearest_finger_mm,3),
                       'unvalidated_hypothesis':True}
                    # Favor close finger proximity only as a diagnostic hint;
                    # neither contact nor grasp can be inferred.
                    if chosen is None or (d['cube_triangle_overlap_count']==0, -d['nearest_sampled_finger_vertex_to_cube_box_mm'], d['min_z_mm']) > (chosen['cube_triangle_overlap_count']==0, -chosen['nearest_sampled_finger_vertex_to_cube_box_mm'], chosen['min_z_mm']):chosen=d
        orientations_per_cand.append({'candidate_index':ix,'coarse_poses_above_table':possible,
           'poses_without_cube_triangle_surface_hits':possible_no_cube_triangle_hits,
           'min_finger_vertex_cube_distance_mm':None if possible==0 else round(closest_finger_mm,3),
           'max_min_z_mm':round(best_z*1000,3),'closest_no_surface_overlap_hypothesis':chosen})
        if chosen:visible.append(chosen)
        print(f'STAGE38_COARSE_ORIENTATION_SCREEN index={ix} tested={len(rolls)*len(pitches)*len(yaws)} table_clear_5mm={possible} no_cube_triangle_hits={possible_no_cube_triangle_hits} min_finger_vertex_cube_mm={closest_finger_mm if possible else -1:.3f}',flush=True)
    report={
        'stage':'3C38','mode':'offline_clean_mesh_and_coarse_whole_gripper_orientation_screen',
        'no_physics_simulation':True,'no_finger_motion':True,'no_arm_motion':True,'no_urdf_usd_changes':True,
        'source_npz':str(args.npz),'corrected_clean_npz':str(args.clean_npz),
        'duplicate_descendant_meshes_removed_from_palm':duplicate_parts,
        'table_bounds_xyz_m':table.tolist(),'cube_center_world_m':cube.tolist(),
        'corrected_candidate_table_gates':gates,'candidate_orientation_scan':orientations_per_cand,
        'scan_angles_world_deg':{'roll':rolls,'pitch':pitches,'yaw':yaws},'rotations_screened':scanned,
        'coarse_pose_5mm_table_clear_candidates':len(visible),
        'important_limitations':[
            'Hypothetical centers from Stage36 are not calibrated grasp centers.',
            'Coarse world-frame orientations are not physically reachable robot poses or IK solutions.',
            'Triangle-vs-cube static mesh surface intersections are checked at candidate pose, but containment, swept collisions, robot-link clearance, self-collision and actual PhysX convex collision checks are NOT.',
            'A 5mm floor clearance test alone cannot establish gripper-object fit or stable grasp.',
            'Finger meshes are separately exported at their current PhysX poses; joint articulation is frozen in every virtual rigid pose.'
        ],'grasp_validated':False,'opening_40mm_validated':False,'no_grasp_permission':True
    }
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json_nice(report)+'\n')
    print(f'STAGE38_COARSE_POSES_SCREENED={scanned} CANDIDATES_WITH_POSSIBLE_TABLE_CLEARANCE={len(visible)}',flush=True)
    print(f'STAGE38_REPORT_JSON={args.report}',flush=True)
    print(f'STAGE38_CLEAN_MESH_NPZ={args.clean_npz}',flush=True)
    print('LEXIUM_STAGE3C38_OFFLINE_AUDIT_COMPLETE=True',flush=True)
    print('STAGE38_NO_GRASP_PERMISSION=True',flush=True)
    print('RESULT=STAGE3C38_OFFLINE_GEOMETRY_SCREENED_NOT_GRASP_VALIDATED',flush=True)

if __name__=='__main__':main()
