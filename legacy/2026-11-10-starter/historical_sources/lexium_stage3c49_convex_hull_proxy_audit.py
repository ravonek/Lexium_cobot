#!/usr/bin/env python3
"""Stage 3C49: read-only conservative convex-hull proxy analysis.

Analyzes convex hulls of authored collision mesh vertices in URDF frames, NOT
actual PhysX-cooked convex hulls, collision filtering, or narrowphase contacts.
Each proxy encloses its source mesh, but can create false-positive overlaps.
The test does not modify assets, run Isaac Lab, or command movement.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from lexium_stage3c40_ik_reachability_audit import parse_urdf, read_cfg, all_fk, measured_pose
from lexium_stage3c41_full_arm_collision_audit import (BODY_NAMES, load_urdf_arm_collision,
    load_frozen_gripper, graph_distance)


def fail(message):
    raise SystemExit('STAGE49_FAIL='+str(message))


def hull_local(vertices):
    # deduping STL triangles reduces Qhull workload without modifying the hull.
    vertices=np.unique(np.round(vertices,10),axis=0)
    if len(vertices)<4:fail('TOO_FEW_HULL_VERTICES')
    hull=ConvexHull(vertices)
    equations=np.asarray(hull.equations,float)
    # Geometry: n.x+d<=0. Hull vertices suffice for bounding boxes.
    points=vertices[hull.vertices]
    return {'points':points,'equations':equations,'vertex_count':len(points),
            'plane_count':len(equations)}


def world_hull(local,pose):
    R,p=pose
    points=local['points']@R.T+p
    normals=local['equations'][:,:3]@R.T
    offsets=local['equations'][:,3]-normals@p
    return {'min':points.min(axis=0),'max':points.max(axis=0),
            'normals':normals,'offsets':offsets}


def box_hull(lo,hi):
    normals=np.vstack([np.eye(3),-np.eye(3)])
    offsets=np.r_[-hi,lo]
    return {'min':lo,'max':hi,'normals':normals,'offsets':offsets}


def intersects(a,b,eps=1e-8):
    if np.any(a['max']<b['min']-eps) or np.any(b['max']<a['min']-eps):
        return False, 'AABB_SEPARATED'
    normals=np.vstack([a['normals'],b['normals']])
    # A_ub @ x <= b_ub; add a small numerical tolerance for near-touching.
    rhs=-np.r_[a['offsets'],b['offsets']]+eps
    out=linprog(np.zeros(3),A_ub=normals,b_ub=rhs,
                bounds=((None,None),)*3,method='highs',options={'presolve':True})
    if out.status==0: return True, 'PROXY_HULL_OVERLAP_OR_TOUCH'
    if out.status==2: return False, 'PROXY_HULL_SEPARATED'
    return None, 'LP_INCONCLUSIVE_'+str(out.status)+'_'+str(out.message)


def main():
    pa=argparse.ArgumentParser(description=__doc__)
    for arg in ('urdf','robot_cfg','stage35','stage45','stage47','stage48','clean_npz'):
        pa.add_argument('--'+arg.replace('_','-'),type=Path,required=True)
    pa.add_argument('--samples',type=int,default=41)
    pa.add_argument('--out',type=Path,default=Path('/tmp/lexium_stage3c49_convex_hull_proxy_report.json'))
    args=pa.parse_args()
    for p in (args.urdf,args.robot_cfg,args.stage35,args.stage45,args.stage47,args.stage48,args.clean_npz):
        if not p.is_file():fail('MISSING_INPUT='+str(p))
    if not 5<=args.samples<=81:fail('INVALID_SAMPLE_COUNT')
    s35=json.loads(args.stage35.read_text());s45=json.loads(args.stage45.read_text());s47=json.loads(args.stage47.read_text());s48=json.loads(args.stage48.read_text())
    if s35.get('stage')!='3C35' or s45.get('stage')!='3C45' or s47.get('stage')!='3C47' or s48.get('stage')!='3C48':fail('WRONG_STAGE_INPUT')
    if not s47.get('finger_reference_geometry_matches_home_at_0_5mm'):fail('NO_HOME_MESH_PROVENANCE')
    if s48.get('robot_motion_permission') is not False or s48.get('grasp_permission') is not False:fail('PRIOR_PERMISSION_UNEXPECTED')
    if s48.get('preferred_orientation_world_xyz_deg')!=[30,-15,30]:fail('PREFERRED_POSE_CHANGED')
    cfg=read_cfg(args.robot_cfg)
    joints,children,_,masses=parse_urdf(args.urdf)
    (R0,p0),perr,aerr,_=measured_pose(joints,s35,cfg)
    if max(perr)>.5 or aerr>.3:fail('GRIPPER_BASE_CALIBRATION')
    geometry={**load_urdf_arm_collision(args.urdf),**load_frozen_gripper(args.clean_npz,(R0,p0))}
    proxy={name:hull_local(geometry[name]['vertices']) for name in BODY_NAMES}
    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child']);adj.setdefault(j['child'],set()).add(j['parent'])
    pairs=[(a,b) for i,a in enumerate(BODY_NAMES) for b in BODY_NAMES[i+1:] if graph_distance(a,b,adj)>1]
    if len(pairs)!=45:fail('EXPECTED_45_PAIRS')
    # Stage48 path results omit per-step q. The full path is linear from
    # robot_cfg HOME to its preferred IK solution.
    qhome=np.asarray(cfg['q_home'],dtype=float)
    qgoal=np.asarray(s48['preferred_q_ik_rad'],dtype=float)
    if qhome.shape!=(6,) or qgoal.shape!=(6,):fail('BAD_JOINT_VECTOR')
    rows=s48['sampled_full_path_candidates']
    if len(rows)!=1 or not np.allclose(rows[0]['q_ik_rad'],qgoal,atol=1e-8):fail('PREFERRED_ROW_NOT_FOUND')
    # Read environment bounds from original Stage45 report, not hardcoded.
    table_bounds=np.asarray(s45['table_bounds_m'],dtype=float).reshape(3,2)
    cube=np.asarray(s45['cube_center_world_m'],dtype=float)
    tm=float(s45.get('table_margin_mm',5.0))*1e-3
    cm=float(s45.get('cube_margin_mm',0.0))*1e-3
    tablebox=box_hull(table_bounds[:,0]-tm,table_bounds[:,1]+tm)
    cubebox=box_hull(cube-.020-cm,cube+.020+cm)
    print('STAGE=3C49 MODE=OFFLINE_CONVEX_HULL_ENCLOSURE_PROXY',flush=True)
    print('STAGE49_NO_ISAAC_LAB=True NO_ACTUATION=True ASSETS_MODIFIED=False',flush=True)
    print('STAGE49_NOT_ACTUAL_PHYSX_COOKED_SHAPES=True',flush=True)
    print(f'STAGE49_SAMPLES={args.samples} PAIRS_PER_FRAME={len(pairs)}',flush=True)
    for name in BODY_NAMES:
        print(f'STAGE49_PROXY_SHAPE body={name} hull_points={proxy[name]["vertex_count"]} hull_planes={proxy[name]["plane_count"]}',flush=True)
    frames=[];unresolved_total=0;cnt=Counter()
    for i,t in enumerate(np.linspace(0,1,args.samples)):
        q=(1-t)*qhome+t*qgoal
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        shapes={name:world_hull(proxy[name],poses[name]) for name in BODY_NAMES}
        overlapping=[];unresolved=[];table=[];cube_hits=[]
        for a,b in pairs:
            yes,reason=intersects(shapes[a],shapes[b])
            if yes is True: overlapping.append(a+' / '+b)
            elif yes is None:unresolved.append([a+' / '+b,reason])
        for name,shape in shapes.items():
            a,st=intersects(shape,tablebox)
            b,sc=intersects(shape,cubebox)
            if a is True:table.append(name)
            if b is True:cube_hits.append(name)
            if a is None:unresolved.append([name+' / Table',st])
            if b is None:unresolved.append([name+' / Cube',sc])
        unresolved_total+=len(unresolved)
        for p in overlapping:cnt[p]+=1
        frames.append({'index':i,'fraction':round(float(t),6),'proxy_hull_overlap_pairs':overlapping,
                       'proxy_hull_table_overlap_bodies':table,'proxy_hull_cube_overlap_bodies':cube_hits,
                       'inconclusive':unresolved})
        if i==0 or i==args.samples-1 or i%5==0:
            print(f'STAGE49_SAMPLE index={i} t={t:.3f} self_proxy_pairs={len(overlapping)} table_proxy_bodies={len(table)} cube_proxy_bodies={len(cube_hits)} unresolved={len(unresolved)}',flush=True)
    home=set(frames[0]['proxy_hull_overlap_pairs'])
    for f in frames:f['proxy_hull_new_pairs_vs_home']=sorted(set(f['proxy_hull_overlap_pairs'])-home)
    n_self=sum(bool(f['proxy_hull_overlap_pairs']) for f in frames)
    n_new=sum(bool(f['proxy_hull_new_pairs_vs_home']) for f in frames)
    n_table=sum(bool(f['proxy_hull_table_overlap_bodies']) for f in frames)
    n_cube=sum(bool(f['proxy_hull_cube_overlap_bodies']) for f in frames)
    result={'stage':'3C49','mode':'offline_conservative_full_source_mesh_convex_hull_proxy',
            'uses_physx_cooked_collision_geometry':False,'uses_authoring_mesh_convex_hulls':True,
            'no_robot_motion':True,'no_isaac_lab':True,'assets_modified':False,
            'source_stage39_may_be_example':s48['stage39_reference_may_be_example'],
            'cube_center_world_m':cube.tolist(),'table_bounds_m':table_bounds.tolist(),
            'self_collisions_enabled_physx':s48['stage47_physx_self_collisions_enabled'],
            'home_q_rad':qhome.tolist(),'target_q_rad':qgoal.tolist(),
            'hull_geometries':{n:{'hull_points':g['vertex_count'],'hull_planes':g['plane_count']} for n,g in proxy.items()},
            'hull_home_overlap_pairs':sorted(home),'hull_endpoint_overlap_pairs':frames[-1]['proxy_hull_overlap_pairs'],
            'hull_endpoint_new_vs_home':frames[-1]['proxy_hull_new_pairs_vs_home'],
            'frames_with_any_hull_self_overlap':n_self,'frames_with_new_hull_self_pairs_vs_home':n_new,
            'frames_with_hull_table_overlap':n_table,'frames_with_hull_cube_overlap':n_cube,
            'inconclusive_lp_queries':unresolved_total,'pair_frame_frequency':dict(cnt),
            'frames':frames,'simulation_motion_authorized':False,'grasp_authorized':False,
            'disclaimer':'This test builds exact convex hulls enclosing AUTHORED mesh vertices. USD convexHull typically undergoes PhysX cooking, vertex reduction, filtering; these are NOT actual runtime colliders. Hull overlap can be a false-positive; hull separation is stronger evidence but NOT continuous swept path proof. HOME already has authored surface intersections and PhysX self collisions disabled. No sim or physical motion permission.'}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(f'STAGE49_HOME_HULL_OVERLAP_PAIRS={len(home)}',flush=True)
    print(f'STAGE49_ENDPOINT_HULL_OVERLAP_PAIRS={len(frames[-1]["proxy_hull_overlap_pairs"])}',flush=True)
    print(f'STAGE49_FRAMES_WITH_NEW_VS_HOME={n_new}/{args.samples}',flush=True)
    print(f'STAGE49_FRAMES_WITH_TABLE_PROXY={n_table}/{args.samples} CUBE_PROXY={n_cube}/{args.samples}',flush=True)
    print(f'STAGE49_INCONCLUSIVE_LP={unresolved_total}',flush=True)
    print('STAGE49_NO_MOTION_PERMISSION=True STAGE49_NO_GRASP_PERMISSION=True',flush=True)
    print('LEXIUM_STAGE3C49_CONVEX_HULL_PROXY_AUDIT_COMPLETE=True',flush=True)
    print('STAGE49_REPORT_JSON='+str(args.out),flush=True)

if __name__=='__main__':main()
