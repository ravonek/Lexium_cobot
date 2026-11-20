#!/usr/bin/env python3
"""Stage 3C.42: read-only URDF/mesh STATIC ENDPOINT audit for all Stage 3C.40 IK solutions.

No motion, no Isaac Lab, no changes to USD/URDF. Not a continuous path,
PhysX-contact, complete self-collision or grasp certification.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from lexium_stage3c40_ik_reachability_audit import read_cfg, parse_urdf, all_fk, measured_pose
from lexium_stage3c41_full_arm_collision_audit import (
    ARM_LINKS, BODY_NAMES, load_urdf_arm_collision, load_frozen_gripper,
    triangle_aabb_intersections, aabb_distance, graph_distance
)

def check(path: Path):
    if not path.is_file(): raise SystemExit(f'STAGE42_MISSING={path}')
    return path

def box_vertex_hits(v, low, high):
    return int(np.count_nonzero(np.all((v>=low) & (v<=high),axis=1)))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--urdf',required=True,type=Path)
    ap.add_argument('--robot-cfg',required=True,type=Path)
    ap.add_argument('--stage35',required=True,type=Path)
    ap.add_argument('--stage40',required=True,type=Path)
    ap.add_argument('--clean-gripper-npz',required=True,type=Path)
    ap.add_argument('--out',required=True,type=Path)
    ap.add_argument('--table-margin-mm',type=float,default=5.0)
    ap.add_argument('--cube-margin-mm',type=float,default=0.0)
    args=ap.parse_args()
    for p in (args.urdf,args.robot_cfg,args.stage35,args.stage40,args.clean_gripper_npz):check(p)
    if not (0.0 <= args.table_margin_mm <= 15.0 and 0.0 <= args.cube_margin_mm <= 10.0):
        raise SystemExit('STAGE42_MARGIN_OUT_OF_RANGE')
    r40=json.loads(args.stage40.read_text());r35=json.loads(args.stage35.read_text())
    if r40.get('stage')!='3C40' or r35.get('stage')!='3C35':raise SystemExit('STAGE42_STAGE_MISMATCH')
    if not r40.get('no_robot_motion') or not r35.get('no_arm_descent'):
        raise SystemExit('STAGE42_UNEXPECTED_PRIOR_MOTION')
    cfg=read_cfg(args.robot_cfg)
    joints,children,chain,masses=parse_urdf(args.urdf)
    (R0,p0),pivot_error,axis_error,tcp=measured_pose(joints,r35,cfg)
    if float(np.max(pivot_error))>.5 or axis_error>.3:raise SystemExit('STAGE42_BASELINE_POSE_FIT_FAILED')
    arm=load_urdf_arm_collision(args.urdf)
    grip=load_frozen_gripper(args.clean_gripper_npz,(R0,p0))
    meshes={**arm,**grip}
    table=np.array([[.17,1.07],[-.6,.6],[-.4,0.]],dtype=float)
    center=np.asarray(r40['cube_center_world_m'],dtype=float)
    tablelo=table[:,0]-args.table_margin_mm/1000.
    tablehi=table[:,1]+args.table_margin_mm/1000.
    cubelo=center-.02-args.cube_margin_mm/1000.
    cubehi=center+.02+args.cube_margin_mm/1000.
    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child'])
        adj.setdefault(j['child'],set()).add(j['parent'])
    nonadjacent=[(a,b) for i,a in enumerate(BODY_NAMES) for b in BODY_NAMES[i+1:] if graph_distance(a,b,adj)>1]
    results=[]
    print('STAGE=3C42 MODE=OFFLINE_STATIC_TARGET_MESH_GATES')
    print('STAGE42_ISAAC_LAB_STARTED=False ROBOT_MOTION_COMMANDED=False ASSETS_MODIFIED=False')
    print(f'STAGE42_BASELINE_ACTUAL_PHYSX_Q_VALIDATED={bool(r40.get("logged_q_to_measured_pose_calibrated"))}')
    print('STAGE42_IK_INPUT_MAY_BE_EXAMPLE='+str(bool(r40.get('input_stage39_may_be_example'))))
    for candidate in r40['candidate_results']:
        i=int(candidate['index'])
        out={'index':i,'ik_pose_match':bool(candidate.get('ik_pose_match',False)),
             'source_seed_candidate':candidate.get('source_seed_candidate')}
        if not out['ik_pose_match']:
            out.update(endpoint_screen='SKIP_IK_NOT_MATCHED',table_mesh_surface_hits=None,cube_mesh_surface_hits=None)
            results.append(out)
            print(f'STAGE42_TARGET index={i} status=SKIP_IK_NOT_MATCHED')
            continue
        q=np.asarray(candidate['q_candidate_rad'],dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise SystemExit('STAGE42_INVALID_IK_Q')
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        bounds={}; table_hits={};cube_hits={};table_inside={};cube_inside={}
        for name in BODY_NAMES:
            R,p=poses[name]
            v=meshes[name]['vertices']@R.T +p
            f=meshes[name]['triangles']
            bounds[name]=(v.min(axis=0),v.max(axis=0))
            if not (np.any(bounds[name][0]>tablehi) or np.any(bounds[name][1]<tablelo)):
                surf=triangle_aabb_intersections(v,f,tablelo,tablehi)
                inside=box_vertex_hits(v,tablelo,tablehi)
                if surf: table_hits[name]=surf
                if inside: table_inside[name]=inside
            if not (np.any(bounds[name][0]>cubehi) or np.any(bounds[name][1]<cubelo)):
                surf=triangle_aabb_intersections(v,f,cubelo,cubehi)
                inside=box_vertex_hits(v,cubelo,cubehi)
                if surf:cube_hits[name]=surf
                if inside:cube_inside[name]=inside
        self_aabb=[]
        for a,b in nonadjacent:
            _,overlap=aabb_distance(bounds[a],bounds[b])
            if overlap:self_aabb.append([a,b])
        table_sum=sum(table_hits.values());cube_sum=sum(cube_hits.values())
        in_table_sum=sum(table_inside.values()); in_cube_sum=sum(cube_inside.values())
        static_surface_precheck=not (table_sum or cube_sum or in_table_sum or in_cube_sum)
        out.update(endpoint_screen='PASS_STATIC_SURFACE_VERTEX_PRECHECK_ONLY' if static_surface_precheck else 'REJECT_STATIC_GEOMETRY',
                   table_mesh_surface_hits=table_hits,cube_mesh_surface_hits=cube_hits,
                   table_inside_vertex_hits=table_inside,cube_inside_vertex_hits=cube_inside,
                   minimum_arm_world_z_mm=round(min(bounds[n][0][2] for n in ARM_LINKS)*1000,3),
                   minimum_gripper_world_z_mm=round(min(bounds[n][0][2] for n in BODY_NAMES if n not in ARM_LINKS)*1000,3),
                   unresolved_nonadjacent_self_aabb_overlaps=self_aabb,
                   static_authored_surface_vertex_precheck_pass=static_surface_precheck)
        results.append(out)
        print(f'STAGE42_TARGET index={i} ik=True table_triangle_hits={table_sum} table_inside_vertices={in_table_sum} cube_triangle_hits={cube_sum} cube_inside_vertices={in_cube_sum} self_aabb_unresolved={len(self_aabb)} static_precheck={static_surface_precheck}',flush=True)
    ik_count=sum(x['ik_pose_match'] for x in results)
    static_count=sum(x.get('static_authored_surface_vertex_precheck_pass',False) for x in results)
    report={'stage':'3C42','mode':'offline_all_ik_candidate_static_endpoint_gates',
       'no_isaac_lab':True,'no_robot_motion':True,'no_finger_motion':True,'no_asset_changes':True,
       'stage40_input_report':str(args.stage40),
       'baseline_actual_physx_q_validated':bool(r40.get('logged_q_to_measured_pose_calibrated')),
       'stage39_source_may_be_example':bool(r40.get('input_stage39_may_be_example')),
       'checked_candidates':len(results),'ik_matched':ik_count,'static_endpoint_prechecks_passed':static_count,
       'table_margin_mm':args.table_margin_mm,'cube_margin_mm':args.cube_margin_mm,
       'results':results,'physical_motion_permission':False,'grasp_validated':False,
       'limitations':[
         'STATIC ENDPOINTS ONLY: no swept trajectory or waypoints checked.',
         'Authored STL and USD meshes are not equivalent to PhysX collision approximations.',
         'Triangle-vs-box surface SAT and contained vertices are not exhaustive shape containment or robot self-collision narrowphase.',
         'Nonadjacent-link AABB overlap is a broadphase warning only.',
         'Finger joints remain frozen at the Stage38 captured pose.',
         'URDF baseline q and Stage39 candidates require independent physical validation when flags are false.'
       ]}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(f'STAGE42_IK_TARGETS={ik_count}/{len(results)} STATIC_ENDPOINT_PRECHECKS={static_count}/{len(results)}')
    print('STAGE42_NO_MOTION_PERMISSION=True')
    print('LEXIUM_STAGE3C42_OFFLINE_ENDPOINT_AUDIT_COMPLETE=True')
    print('STAGE42_REPORT_JSON='+str(args.out))
    print('RESULT=STAGE3C42_ENDPOINTS_SCREENED_NOT_PHYSX_VALIDATED_NOT_GRASP_VALIDATED')

if __name__=='__main__':main()
