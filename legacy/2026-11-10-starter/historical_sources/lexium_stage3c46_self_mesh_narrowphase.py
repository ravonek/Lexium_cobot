#!/usr/bin/env python3
"""Stage 3C.46: offline robot self-mesh SURFACE-intersection screening.

Uses Stage45 +100 mm pregrasp joint interpolation and original URDF/STL meshes
plus Stage38 frozen-articulation gripper triangles. Runs a conservative world
AABB BVH and triangle-triangle SAT including coplanar separating axes.

NO Isaac Lab, actuation, collision disabling or asset edits.
A negative authored-mesh surface test does NOT validate actual PhysX colliders,
watertight-solid containment, 5 mm clearance, continuous/swept motion, stability,
40 mm finger opening, or grasp. No motion permission is produced.
"""
from __future__ import annotations
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
from lexium_stage3c40_ik_reachability_audit import ARM, BODIES, read_cfg, parse_urdf, all_fk, measured_pose
from lexium_stage3c41_full_arm_collision_audit import (
    ARM_LINKS, BODY_NAMES, load_urdf_arm_collision, load_frozen_gripper,
    aabb_distance, graph_distance
)


def fail(msg):
    raise SystemExit('STAGE46_INPUT_ERROR=' + str(msg))


def options():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('urdf','robot_cfg','stage35','stage45','clean_npz'):
        p.add_argument('--'+name.replace('_','-'),required=True,type=Path)
    p.add_argument('--lift-mm',type=float,default=100.)
    p.add_argument('--samples',type=int,default=41)
    p.add_argument('--leaf-size',type=int,default=12)
    p.add_argument('--max-node-tests',type=int,default=160000)
    p.add_argument('--self-test',action='store_true',help='Run triangle SAT unit tests and exit')
    p.add_argument('--out',type=Path,default=Path('/tmp/lexium_stage3c46_self_mesh_narrowphase_report.json'))
    return p.parse_args()


class LocalBVH:
    """Fixed triangle-index BVH in local body frame; rotated AABBs each frame."""
    def __init__(self,geometry,leafsize):
        v=geometry['vertices'];f=geometry['triangles']
        self.tris=v[f]
        fl=self.tris.min(axis=1);fh=self.tris.max(axis=1)
        self.bounds_lo=[];self.bounds_hi=[];self.children=[];self.leaves=[]
        mid=(fl+fh)*0.5

        def make(indices):
            idx=len(self.children)
            self.children.append(None); self.leaves.append(None)
            self.bounds_lo.append(fl[indices].min(0))
            self.bounds_hi.append(fh[indices].max(0))
            if len(indices)<=leafsize:
                self.leaves[idx]=indices.copy()
            else:
                axis=int(np.argmax(np.ptp(mid[indices],axis=0)))
                # Stable sorting for fully deterministic test conditions.
                sorter=indices[np.argsort(mid[indices,axis],kind='mergesort')]
                n=len(sorter)//2
                self.children[idx]=(make(sorter[:n]),make(sorter[n:]))
            return idx
        make(np.arange(len(f),dtype=np.int32))
        self.bounds_lo=np.asarray(self.bounds_lo)
        self.bounds_hi=np.asarray(self.bounds_hi)
        self.center=(self.bounds_lo+self.bounds_hi)*0.5
        self.extent=(self.bounds_hi-self.bounds_lo)*0.5
        self.face_count=len(f)

    def in_world(self,R,p):
        center=self.center@R.T+p
        extent=self.extent@np.abs(R).T
        return (center-extent-1.e-10,center+extent+1.e-10,
                self.tris@R.T+p)


def tri_tri_sat(A,B,eps=1.e-10):
    """Conservative triangle-triangle SAT for non-degenerate 3D triangles.

    Inputs: (N,3,3) and (N,3,3), outputs boolean length N.
    Axes: each face normal, 9 edge cross-edge axes, and coplanar-safe 6
    in-plane edge normals. Touching/near-touching is treated as positive.
    """
    n=len(A)
    if n==0:return np.empty((0,),dtype=bool)
    aa=[A[:,(i+1)%3]-A[:,i] for i in range(3)]
    bb=[B[:,(i+1)%3]-B[:,i] for i in range(3)]
    na=np.cross(aa[0],aa[1]);nb=np.cross(bb[0],bb[1])
    active=np.ones(n,dtype=bool)
    axes=[na,nb]+[np.cross(x,y) for x in aa for y in bb]
    axes += [np.cross(na,x) for x in aa]
    axes += [np.cross(nb,y) for y in bb]
    for axis in axes:
        if not active.any():break
        idx=np.flatnonzero(active)
        norm=np.linalg.norm(axis[idx],axis=1)
        meaningful=norm>1.e-20
        if not meaningful.any():continue
        idx=idx[meaningful]
        directions=axis[idx]/norm[meaningful,None]
        pa=np.einsum('nij,nj->ni',A[idx],directions,optimize=True)
        pb=np.einsum('nij,nj->ni',B[idx],directions,optimize=True)
        separated=(pa.max(1)<pb.min(1)-eps)|(pb.max(1)<pa.min(1)-eps)
        active[idx[separated]]=False
    return active


def mesh_pair_screen(aa,bb,views_a,views_b,maxnodes):
    alo,ahi,at=views_a
    blo,bhi,bt=views_b
    if np.any(alo[0]>bhi[0]) or np.any(blo[0]>ahi[0]):
        return 'AABB_SEPARATED',0,0,None
    stack=[(0,0)]
    n_nodes=0; n_triangles=0
    while stack:
        na,nb=stack.pop()
        n_nodes+=1
        if n_nodes>maxnodes:
            return 'INCONCLUSIVE_NODE_BUDGET',n_nodes,n_triangles,None
        if np.any(alo[na]>bhi[nb]) or np.any(blo[nb]>ahi[na]):continue
        a_child=aa.children[na];b_child=bb.children[nb]
        if a_child is None and b_child is None:
            ia=aa.leaves[na];ib=bb.leaves[nb]
            ta=at[ia];tb=bt[ib]
            tlo_a=ta.min(axis=1);thi_a=ta.max(axis=1)
            tlo_b=tb.min(axis=1);thi_b=tb.max(axis=1)
            overlap=np.all((tlo_a[:,None,:]<=thi_b[None,:,:]+1.e-10) &
                           (tlo_b[None,:,:]<=thi_a[:,None,:]+1.e-10),axis=2)
            i,j=np.nonzero(overlap)
            if not len(i):continue
            n_triangles+=len(i)
            hits=tri_tri_sat(ta[i],tb[j])
            if hits.any():
                k=int(np.flatnonzero(hits)[0]);return 'SURFACE_INTERSECTION',n_nodes,n_triangles,[int(ia[i[k]]),int(ib[j[k]])]
            continue
        if a_child is None:
            stack.extend(((na,b_child[0]),(na,b_child[1])))
        elif b_child is None:
            stack.extend(((a_child[0],nb),(a_child[1],nb)))
        else:
            ea=ahi[na]-alo[na];eb=bhi[nb]-blo[nb]
            if float(np.prod(ea))>=float(np.prod(eb)):
                stack.extend(((a_child[0],nb),(a_child[1],nb)))
            else:
                stack.extend(((na,b_child[0]),(na,b_child[1])))
    return 'NO_SURFACE_INTERSECTION_FOUND',n_nodes,n_triangles,None


def self_test():
    a=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]])
    disjoint=np.array([[[2.,2.,0.],[3.,2.,0.],[2.,3.,0.]]])
    coplanar=np.array([[[.2,.2,0.],[.8,.2,0.],[.2,.8,0.]]])
    vertical=np.array([[[.2,.2,-1.],[.2,.2,1.],[.8,.2,0.]]])
    parallel=a+np.array([0.,0.,.002])
    assert not bool(tri_tri_sat(a,disjoint)[0]),'COPLANAR_DISJOINT_TEST'
    assert bool(tri_tri_sat(a,coplanar)[0]),'COPLANAR_OVERLAP_TEST'
    assert bool(tri_tri_sat(a,vertical)[0]),'TRANSVERSE_CROSSING_TEST'
    assert not bool(tri_tri_sat(a,parallel)[0]),'PARALLEL_SEPARATE_TEST'
    print('STAGE46_TRIANGLE_SAT_SELF_TEST_PASS=True')


def main():
    args=options()
    self_test()
    if args.self_test:return
    for p in (args.urdf,args.robot_cfg,args.stage35,args.stage45,args.clean_npz):
        if not p.is_file():fail('MISSING_FILE='+str(p))
    if args.samples<3 or args.samples>101:fail('INVALID_SAMPLES')
    if args.leaf_size<4 or args.leaf_size>32:fail('INVALID_LEAF_SIZE')
    if args.max_node_tests<1000:fail('INVALID_NODE_BUDGET')
    stage45=json.loads(args.stage45.read_text())
    if stage45.get('stage')!='3C45' or not stage45.get('stage44_home_fk_verified'):
        fail('STAGE45_NOT_VERIFIED')
    if stage45.get('no_isaac_lab') is not True or stage45.get('no_physical_motion') is not True:
        fail('STAGE45_NOT_READ_ONLY')
    if stage45.get('physical_motion_permission') is not False or stage45.get('no_grasp_permission') is not True:
        fail('PRIOR_SAFETY_GATE_UNEXPECTED')
    row=next((x for x in stage45.get('target_rows',[]) if abs(float(x['lift_mm'])-args.lift_mm)<1e-8),None)
    if row is None or not row.get('sampled_vertex_and_triangle_table_cube_clear'):
        fail('LIFT_NOT_STATIC_TABLE_CUBE_SCREENED_CLEAR')
    if row.get('sampled_self_aabb_overlap_count_unresolved') is None:
        fail('NO_STAGE45_AABB_DATA')
    st35=json.loads(args.stage35.read_text())
    if st35.get('stage')!='3C35':fail('BAD_STAGE35')
    cfg=read_cfg(args.robot_cfg)
    joints,children,chain,masses=parse_urdf(args.urdf)
    (Rbase,pbase),pivot_err,axis_err,_=measured_pose(joints,st35,cfg)
    if max(pivot_err)>.5 or axis_err>.3:fail('INVALID_GRIPPER_BASELINE')
    geom={**load_urdf_arm_collision(args.urdf),**load_frozen_gripper(args.clean_npz,(Rbase,pbase))}
    bvhs={name:LocalBVH(geom[name],args.leaf_size) for name in BODY_NAMES}
    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child'])
        adj.setdefault(j['child'],set()).add(j['parent'])
    pairs=[(a,b) for i,a in enumerate(BODY_NAMES) for b in BODY_NAMES[i+1:] if graph_distance(a,b,adj)>1]
    qhome=np.asarray(stage45['stage44_q_after_write_rad'],dtype=float)
    qtarget=np.asarray(row['q_ik_rad'],dtype=float)
    if qhome.shape!=(6,) or qtarget.shape!=(6,) or not np.isfinite([*qhome,*qtarget]).all():
        fail('BAD_Q')
    if not np.allclose(qhome,cfg['q_home'],atol=1e-4):fail('HOME_CONFIG_CHANGED')
    print('STAGE=3C46 MODE=OFFLINE_TRIANGLE_SURFACE_NARROWPHASE',flush=True)
    print('STAGE46_ISAAC_LAB_STARTED=False PHYSICAL_MOTION_COMMANDED=False ASSETS_MODIFIED=False',flush=True)
    print(f'STAGE46_PREGRASP_LIFT_MM={args.lift_mm} JOINT_PATH_SAMPLES={args.samples} NONADJACENT_BODY_PAIRS={len(pairs)}',flush=True)
    print('STAGE46_SOURCE_STAGE39_MAY_BE_EXAMPLE='+str(bool(stage45.get('stage40_stage39_input_may_be_example'))),flush=True)
    frames=[];summary=Counter();first_hits={}
    for k,fraction in enumerate(np.linspace(0.,1.,args.samples)):
        q=qhome*(1-fraction)+qtarget*fraction
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        views={name:bvhs[name].in_world(*poses[name]) for name in BODY_NAMES}
        hits=[];inconclusive=[];aabb_overlap=0;triangle_pairs=0;node_pairs=0
        for a,b in pairs:
            if np.any(views[a][0][0]>views[b][1][0]) or np.any(views[b][0][0]>views[a][1][0]):
                continue
            aabb_overlap+=1
            status,nodes,tris,evidence=mesh_pair_screen(bvhs[a],bvhs[b],views[a],views[b],args.max_node_tests)
            triangle_pairs+=tris;node_pairs+=nodes
            if status=='SURFACE_INTERSECTION':
                hits.append([a,b,evidence]);summary[a+' / '+b]+=1
                first_hits.setdefault(a+' / '+b,k)
            elif status.startswith('INCONCLUSIVE'):
                inconclusive.append([a,b,status])
        data={'sample_index':k,'fraction':round(float(fraction),6),
              'q_rad':np.round(q,7).tolist(),
              'nonadjacent_body_AABB_overlap_count':aabb_overlap,
              'authored_surface_intersections':hits,
              'unresolved_budget_pairs':inconclusive,
              'triangle_pair_SAT_tests':triangle_pairs,
              'BVH_node_pair_visits':node_pairs}
        frames.append(data)
        if k in (0,args.samples-1) or k % max(1,args.samples//8)==0 or hits and k<3:
            print('STAGE46_SAMPLE index={} t={:.3f} AABB_pairs={} surface_intersection_pairs={} unresolved={} triangle_tests={}'.format(
                k,fraction,aabb_overlap,len(hits),len(inconclusive),triangle_pairs),flush=True)
            if hits:print('STAGE46_PAIRS_WITH_INTERSECTING_SURFACES index={} pairs={}'.format(k,json.dumps([x[:2] for x in hits])),flush=True)
    n_hit=sum(bool(x['authored_surface_intersections']) for x in frames)
    n_unresolved=sum(bool(x['unresolved_budget_pairs']) for x in frames)
    n_aabb=sum(x['nonadjacent_body_AABB_overlap_count']>0 for x in frames)
    surface_screen_clear=bool(n_hit==0 and n_unresolved==0)
    rep={'stage':'3C46','mode':'offline_discrete_authored_triangle_surface_self_narrowphase',
         'no_isaac_lab':True,'no_physical_motion':True,'assets_modified':False,
         'stage45_file':str(args.stage45),'source_stage39_may_be_example':bool(stage45.get('stage40_stage39_input_may_be_example')),
         'lift_mm':args.lift_mm,'samples':args.samples,'q_home_rad':qhome.tolist(),'q_target_rad':qtarget.tolist(),
         'nonadjacent_graph_pairs':pairs,'frame_results':frames,
         'AABB_overlap_frames':n_aabb,'authored_surface_intersection_frames':n_hit,'node_budget_unresolved_frames':n_unresolved,
         'surface_intersection_pairs_counted_by_frames':dict(summary),
         'first_surface_intersection_frame_per_pair':first_hits,
         'all_sampled_authored_surface_pairs_without_intersection_or_unresolved_budget':surface_screen_clear,
         'stage45_table_cube_sampled_screen_pass':True,
         'physx_convex_shape_collision_validated':False,
         'authored_solid_volume_containment_validated':False,
         '5mm_self_clearance_validated':False,
         'continuous_swept_collision_validated':False,
         'actual_candidate1_joint_angles_validated':False,
         'finger_opening_40mm_validated':False,
         'robot_motion_permission':False,'grasp_permission':False,
         'disclaimer':'Authoring mesh surface intersection only at discrete joint-space points; body pairs separated by one URDF joint excluded, other pairs tested. Triangle SAT includes coplanar axes. AABB overlap alone not an intersection. Closed-solid containment, margin clearance, real PhysX collision shapes and continuous path untested. Geometry target comes from Stage39 example. No simulation or actuation. NO MOTION AUTHORIZATION.'}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(rep,ensure_ascii=False,indent=2)+'\n')
    print(f'STAGE46_AABB_OVERLAP_FRAMES={n_aabb}/{len(frames)}',flush=True)
    print(f'STAGE46_AUTHORED_SURFACE_INTERSECTION_FRAMES={n_hit}/{len(frames)}',flush=True)
    print(f'STAGE46_NARROWPHASE_BUDGET_UNRESOLVED_FRAMES={n_unresolved}/{len(frames)}',flush=True)
    print('STAGE46_SURFACE_ONLY_PRECHECK_CLEAR='+str(surface_screen_clear),flush=True)
    print('STAGE46_PHYSX_COLLISIONS_VALIDATED=False',flush=True)
    print('STAGE46_NO_MOTION_PERMISSION=True',flush=True)
    print('STAGE46_NO_GRASP_PERMISSION=True',flush=True)
    print('STAGE46_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C46_SELF_MESH_NARROWPHASE_COMPLETE=True',flush=True)

if __name__=='__main__':main()
