#!/usr/bin/env python3
"""Stage 3C.52 -- CORRECTED frozen-gripper frame audit of Stage3C51 (OFFLINE).

Critical fix: Stage3C38 gripper world meshes are first converted into the
GRIPPER-BASE reference frame. All 5 gripper meshes therefore require the SAME
WORLD GRIPPER-BASE TRANSFORM while four finger joint angles remain frozen.
Stage3C41..49 wrongly applied each finger link's FK pose to these base-frame
vertices, making their mesh/collision results unreliable. Stage3C47 independently
verified the correct rigid-base transform at HOME within 0.035 mm.

Reads prior reports, URDF, STL, and frozen Stage3C38 gripper geometry. Rebuilds
41-sample 11-body convex-hull proxy checks, compares their results with Stage49,
and measures actual Stage51 endpoint gripper mesh vs the cube. Does not start
IsaacLab, move robot, open/close fingers, edit USD, or approve grasp.

Convex hull checks are discrete & authored-mesh proxy only, NOT actual cooked
PhysX colliders or continuous swept collision, and do NOT check force closure.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from lexium_stage3c40_ik_reachability_audit import (ARM, BODIES, read_cfg, parse_urdf, all_fk, measured_pose)
from lexium_stage3c41_full_arm_collision_audit import (ARM_LINKS, BODY_NAMES,
    load_urdf_arm_collision, load_frozen_gripper, graph_distance)
from lexium_stage3c49_convex_hull_proxy_audit import hull_local, world_hull, box_hull, intersects


def fail(s):
    raise SystemExit('STAGE52_FAIL='+str(s))


def parse_args():
    pa=argparse.ArgumentParser(description=__doc__)
    for a in ('urdf','robot_cfg','stage35','stage47','stage49','stage51','clean_npz'):
        pa.add_argument('--'+a.replace('_','-'),type=Path,required=True)
    pa.add_argument('--samples',type=int,default=41)
    pa.add_argument('--out',type=Path,default=Path('/tmp/lexium_stage3c52_corrected_gripper_geometry_report.json'))
    return pa.parse_args()


def pose_xyzw_to_Rp(pose):
    p=np.asarray(pose[:3],dtype=float)
    R=Rotation.from_quat(np.asarray(pose[3:],dtype=float)).as_matrix()
    return R,p


def wpts(local,pose):
    R,p=pose
    return local@R.T+p


def main():
    a=parse_args()
    if not 5 <= a.samples <= 101:fail('SAMPLES_OUT_OF_RANGE')
    for p in (a.urdf,a.robot_cfg,a.stage35,a.stage47,a.stage49,a.stage51,a.clean_npz):
        if not p.is_file():fail('MISSING_INPUT='+str(p))
    s35=json.loads(a.stage35.read_text());s47=json.loads(a.stage47.read_text())
    s49=json.loads(a.stage49.read_text());s51=json.loads(a.stage51.read_text())
    if [s35.get('stage'),s47.get('stage'),s49.get('stage'),s51.get('stage')]!=['3C35','3C47','3C49','3C51']:
        fail('WRONG_STAGE_INPUT')
    if s47.get('finger_reference_geometry_matches_home_at_0_5mm') is not True:
        fail('HOME_FINGER_FROZEN_REFERENCE_NOT_VERIFIED')
    if s51.get('simulation_motion_passed') is not True or s51.get('trajectory_fully_executed') is not True:
        fail('STAGE51_DID_NOT_COMPLETE')
    if s51.get('physx_self_collisions_enabled') is not False or s49.get('uses_physx_cooked_collision_geometry') is not False:
        fail('UNEXPECTED_PHYSX_POLICY_OR_PROXY_SOURCE')
    cfg=read_cfg(a.robot_cfg)
    joints,children,chain,masses=parse_urdf(a.urdf)
    (Rref,pref),pe,oe,_=measured_pose(joints,s35,cfg)
    if max(pe)>.5 or oe>.3:fail('REFERENCE_GRIPPER_POSE_INVALID')
    geoms={**load_urdf_arm_collision(a.urdf),**load_frozen_gripper(a.clean_npz,(Rref,pref))}
    hulls={name:hull_local(geoms[name]['vertices']) for name in BODY_NAMES}
    qhome=np.asarray(s49['home_q_rad'],dtype=float)
    qtarget=np.asarray(s49['target_q_rad'],dtype=float)
    qfinal=np.asarray(s51['final_q_rad'],dtype=float)
    if any(q.shape!=(6,) for q in (qhome,qtarget,qfinal)) or np.max(np.abs(qtarget-qfinal))>.02:
        fail('STAGE49_AND_STAGE51_TARGETS_DISAGREE')
    if np.max(np.abs(qhome-cfg['q_home']))>.001:fail('HOME_DISAGREEMENT')
    stage51_pose=np.asarray(s51['final_gripper_base_pose_xyzw'],dtype=float)
    if stage51_pose.shape!=(7,) or not np.isfinite(stage51_pose).all():fail('BAD_STAGE51_FINAL_POSE')
    actual_pose=pose_xyzw_to_Rp(stage51_pose)
    cube=np.asarray(s49['cube_center_world_m'],dtype=float)
    if s51.get('sampled'):
        cube=np.asarray(s51['sampled'][-1]['cube_xyz_m'],dtype=float)
    tablebounds=np.asarray(s49['table_bounds_m'],dtype=float).reshape(3,2)
    tbox=box_hull(tablebounds[:,0]-.005,tablebounds[:,1]+.005)
    cbox=box_hull(cube-.02,cube+.02)
    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child']);adj.setdefault(j['child'],set()).add(j['parent'])
    pairs=[(x,y) for i,x in enumerate(BODY_NAMES) for y in BODY_NAMES[i+1:] if graph_distance(x,y,adj)>1]
    if len(pairs)!=45:fail('PAIR_GRAPH_MISMATCH')

    print('STAGE=3C52 MODE=OFFLINE_CORRECTED_BASE_FRAME_FINGER_GEOMETRY',flush=True)
    print('STAGE52_NO_ISAAC_LAB=True NO_ROBOT_MOTION=True ASSETS_MODIFIED=False',flush=True)
    print('STAGE52_PREVIOUS_OFFLINE_FINGER_TRANSFORM_WRONG=True',flush=True)
    print('STAGE52_REFERENCE_FINGERS_VALIDATED_BY_STAGE47=True',flush=True)
    home_poses,_,_=all_fk(qhome,cfg,joints,children,masses)
    wrong_frame_disagreement_mm={}
    for name in BODIES:
        vv=geoms[name]['vertices']
        wrong=wpts(vv,home_poses[name])
        correct=wpts(vv,home_poses['gripper_base_1'])
        wrong_frame_disagreement_mm[name]=float(np.linalg.norm(wrong-correct,axis=1).max()*1000)
        print('STAGE52_HOME_TRANSFORM_ERROR name={} max_vertex_displacement_mm={:.3f}'.format(
            name,wrong_frame_disagreement_mm[name]),flush=True)
    if max(wrong_frame_disagreement_mm[n] for n in BODIES[1:])<1.:
        fail('EXPECTED_FRAME_BUG_NOT_REPRODUCED')

    frames=[];unknowns=0
    for index,t in enumerate(np.linspace(0.,1.,a.samples)):
        q=(1-t)*qhome+t*qtarget
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        # CRITICAL FIX: a gripper mesh already in gripper_base's local frame
        # moves with gripper_base, not with the finger body's own FK matrix.
        shapes={n:world_hull(hulls[n],poses[n if n in ARM_LINKS else 'gripper_base_1']) for n in BODY_NAMES}
        hits=[];table=[];cubecoll=[];uncertain=[]
        for x,y in pairs:
            hit,reason=intersects(shapes[x],shapes[y])
            if hit is True:hits.append(x+' / '+y)
            elif hit is None:uncertain.append(x+' / '+y+': '+reason)
        for n in BODY_NAMES:
            hit,reason=intersects(shapes[n],tbox)
            if hit is True:table.append(n)
            elif hit is None:uncertain.append(n+' / Table: '+reason)
            hit,reason=intersects(shapes[n],cbox)
            if hit is True:cubecoll.append(n)
            elif hit is None:uncertain.append(n+' / Cube: '+reason)
        unknowns+=len(uncertain)
        frames.append({'index':index,'fraction':round(float(t),6),'self_proxy_pairs':sorted(hits),
                       'table_proxy_bodies':table,'cube_proxy_bodies':cubecoll,'inconclusive':uncertain})
        if index in (0,a.samples-1) or index%max(1,(a.samples-1)//8)==0:
            print('STAGE52_FRAME index={} self_proxy_pairs={} table={} cube={} inconclusive={}'.format(
                index,len(hits),len(table),len(cubecoll),len(uncertain)),flush=True)
    home_pairs=set(frames[0]['self_proxy_pairs'])
    for f in frames:
        f['new_self_pairs_vs_corrected_home']=sorted(set(f['self_proxy_pairs'])-home_pairs)
    n_self=sum(bool(f['self_proxy_pairs']) for f in frames)
    n_new=sum(bool(f['new_self_pairs_vs_corrected_home']) for f in frames)
    n_table=sum(bool(f['table_proxy_bodies']) for f in frames)
    n_cube=sum(bool(f['cube_proxy_bodies']) for f in frames)
    print('STAGE52_HOME_CORRECTED_SELF_PAIRS='+json.dumps(sorted(home_pairs)),flush=True)
    print('STAGE52_ENDPOINT_CORRECTED_SELF_PAIRS='+json.dumps(frames[-1]['self_proxy_pairs']),flush=True)
    print('STAGE52_CORRECTED_SELF_OVERLAP_FRAMES={}/{}'.format(n_self,a.samples),flush=True)
    print('STAGE52_CORRECTED_NEW_VS_HOME_FRAMES={}/{}'.format(n_new,a.samples),flush=True)
    print('STAGE52_CORRECTED_TABLE_FRAMES={}/{} CUBE_FRAMES={}/{}'.format(n_table,a.samples,n_cube,a.samples),flush=True)
    print('STAGE52_INCONCLUSIVE_LP='+str(unknowns),flush=True)

    # Infer only the actual frozen finger geometry *as simulated*. A grasp
    # center is NOT assumed equal to TCP, and finger pose is frozen here.
    Rphys,p_phys=actual_pose
    center_tcp=p_phys+Rphys@cfg['tcp_offset']
    tcp_cube_distance_mm=float(np.linalg.norm(center_tcp-cube)*1000.)
    tcp_cube_y_mm=float((center_tcp-cube)[1]*1000.)
    finger_rows={}
    for name in BODIES:
        points=wpts(geoms[name]['vertices'],actual_pose)
        lo=points.min(0);hi=points.max(0)
        sq=np.maximum(np.maximum(cube-.02-points,points-(cube+.02)),0.)
        sampled_vertex_to_cube_mm=float(np.linalg.norm(sq,axis=1).min()*1000.)
        sqxy=np.maximum(np.maximum(cube[:2]-.02-points[:,:2],points[:,:2]-(cube[:2]+.02)),0.)
        xy_vertex_to_cube_mm=float(np.linalg.norm(sqxy,axis=1).min()*1000.)
        finger_rows[name]={
            'world_min_xyz_m':lo.tolist(),'world_max_xyz_m':hi.tolist(),
            'min_mesh_vertex_to_cube_AABB_mm':sampled_vertex_to_cube_mm,
            'min_mesh_vertex_to_cube_XY_footprint_mm':xy_vertex_to_cube_mm,
            'conservative_vertical_separation_above_cube_top_mm':float((lo[2]-(cube[2]+.020))*1000.)}
        print('STAGE52_ACTUAL_FINAL_MESH name={} min_z_mm={:.2f} vert_separation_above_cube_top_mm={:.2f} nearest_vertex_to_cube_mm={:.2f}'.format(
              name,lo[2]*1000,finger_rows[name]['conservative_vertical_separation_above_cube_top_mm'],sampled_vertex_to_cube_mm),flush=True)
    min_finger_z_m=min(finger_rows[n]['world_min_xyz_m'][2] for n in BODIES[1:])
    cube_top_m=cube[2]+.02
    physical_final_finger_vertical_sep_mm=(min_finger_z_m-cube_top_m)*1000.
    cube_local=Rphys.T@(cube-p_phys)
    print('STAGE52_CUBE_IN_GRIPPER_BASE_LOCAL_M='+json.dumps(np.round(cube_local,6).tolist()),flush=True)
    print('STAGE52_TCP_VS_CUBE_DELTA_MM='+json.dumps(np.round((center_tcp-cube)*1000,3).tolist()),flush=True)
    print('STAGE52_TCP_VS_CUBE_Y_MM={:.3f}'.format(tcp_cube_y_mm),flush=True)
    print('STAGE52_LOWEST_FINGER_ABOVE_CUBE_TOP_MM={:.3f}'.format(physical_final_finger_vertical_sep_mm),flush=True)
    mismatch=(any(f['new_self_pairs_vs_corrected_home'] for f in frames) or n_table or n_cube or unknowns)
    report={
        'stage':'3C52','mode':'offline_corrected_frozen_gripper_base_frame_collision_recheck',
        'no_isaac_lab':True,'no_motion':True,'original_assets_modified':False,
        'bug':'Stage3C41..49 load_frozen_gripper returns gripper_base-local vertices for ALL gripper bodies, but old collision screens transformed finger vertices using the individual finger FK frames instead of gripper_base FK.',
        'stage47_reference_finger_geometry_validated':True,
        'old_vs_corrected_home_max_vertex_mm':wrong_frame_disagreement_mm,
        'original_stage49_home_proxy_pairs':s49['hull_home_overlap_pairs'],
        'original_stage49_endpoint_proxy_pairs':s49['hull_endpoint_overlap_pairs'],
        'corrected_home_proxy_pairs':sorted(home_pairs),
        'corrected_endpoint_proxy_pairs':frames[-1]['self_proxy_pairs'],
        'corrected_frames_with_any_self_proxy_overlap':n_self,
        'corrected_frames_with_new_vs_home_self_proxy_overlap':n_new,
        'corrected_frames_with_table_proxy_overlap':n_table,
        'corrected_frames_with_cube_proxy_overlap':n_cube,
        'inconclusive_lp_queries':unknowns,
        'poses_sampled':a.samples,'frames':frames,
        'stage51_sim_motion_passed':True,
        'stage51_final_q_rad':qfinal.tolist(),
        'stage51_actual_final_gripper_base_pose_xyzw':stage51_pose.tolist(),
        'stage51_measured_cube_center_xyz_m':cube.tolist(),
        'stage51_final_tcp_xyz_m':center_tcp.tolist(),
        'stage51_tcp_cube_delta_xyz_mm':((center_tcp-cube)*1000.).tolist(),
        'stage51_tcp_cube_distance_mm':tcp_cube_distance_mm,
        'cube_in_gripper_base_local_xyz_m':cube_local.tolist(),
        'stage51_final_finger_min_z_m':min_finger_z_m,
        'stage51_final_cube_top_z_m':cube_top_m,
        'stage51_final_finger_vertical_gap_above_cube_top_mm':physical_final_finger_vertical_sep_mm,
        'stage51_final_mesh_body_proximity':finger_rows,
        'previous_offline_collision_claims_require_revalidation':True,
        'corrected_geometry_requires_new_candidate_search':bool(mismatch),
        'actual_cooked_physx_self_collisions_validated':False,
        'finger_opening_40mm_validated':False,'grasp_center_validated':False,
        'grasp_authorized':False,'hardware_motion_authorized':False,
        'disclaimer':('Frozen gripper meshes verified at HOME by Stage47 but in dynamic motion finger values may deviate. '
             'Stage52 uses corrected gripper_base rigid transform for all five meshes, 41 discrete proxy hull steps only. '
             'Reference Stage39 seed may be example. Simulation-only Stage51 external contacts do NOT prove self collision clearance. '
             'TCP is not necessarily the true grasp center; only offsets and mesh extents are measured. No descent/grasp or hardware actuation permission.'),
    }
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print('STAGE52_CORRECTED_GEOMETRY_REVALIDATION_REQUIRED=True',flush=True)
    print('STAGE52_NO_CUBE_DESCENT=True STAGE52_NO_GRASP_PERMISSION=True',flush=True)
    print('LEXIUM_STAGE3C52_CORRECTED_FINGER_FRAME_AUDIT_COMPLETE=True',flush=True)
    print('STAGE52_REPORT_JSON='+str(a.out),flush=True)

if __name__=='__main__':main()
