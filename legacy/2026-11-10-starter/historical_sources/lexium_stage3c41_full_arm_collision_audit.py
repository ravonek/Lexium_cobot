#!/usr/bin/env python3
"""Stage 3C.41 — read-only OFFLINE full-arm authored-mesh collision PRECHECK.

Reads original URDF collision mesh geometry and PhysX-exported frozen gripper meshes;
performs *discrete* robot FK at joint-space interpolation samples.
Does NOT start Isaac Lab or command actuators. No motion permission is implied.

The URDF-IK starting q is INFERRED, not validated against recorded PhysX arm q.
This precheck does NOT simulate PhysX convex decomposition, dynamic trajectories,
continuous sweep, or exact self-collision surface-surface narrow phase.
"""
from __future__ import annotations
import argparse
import json
import math
import struct
import sys
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

# Stage40 module exists in the same project directory after prior installation.
from lexium_stage3c40_ik_reachability_audit import (
    ARM, BODIES, read_cfg, parse_urdf, fk_chain, all_fk, measured_pose
)

ARM_LINKS = ('base_link','link1_1','link2_1','link3_1','link4_1','LINK5_1')
BODY_NAMES = ARM_LINKS + BODIES


def fail(message):
    raise SystemExit('STAGE41_INPUT_ERROR='+str(message))


def xyz(s):
    v=np.fromstring(s or '0 0 0', sep=' ', dtype=np.float64)
    if v.shape != (3,) or not np.isfinite(v).all():fail('INVALID_XYZ='+repr(s))
    return v


def load_binary_stl(path):
    """Return triangle vertices in *authoring units* (binary STL, no trimesh dep)."""
    raw=path.read_bytes()
    if len(raw)<84:fail('INVALID_BINARY_STL_SHORT='+str(path))
    count=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+50*count or count<1:fail('INVALID_BINARY_STL_SIZE='+str(path))
    d=np.dtype([('normal','<f4',(3,)),('tri','<f4',(3,3)),('attr','<u2')])
    t=np.frombuffer(raw,dtype=d,count=count,offset=84)['tri'].copy().astype('float64')
    if not np.isfinite(t).all():fail('NAN_IN_STL='+str(path))
    return t.reshape(-1,3),np.arange(t.size//3,dtype=np.int32).reshape(-1,3)


def load_urdf_arm_collision(urdf):
    root=ET.parse(urdf).getroot()
    out={}
    for link in root.findall('link'):
        name=link.get('name')
        if name not in ARM_LINKS:continue
        collisions=link.findall('collision')
        if len(collisions)!=1:fail(f'EXPECTED_ONE_COLLISION_{name}_FOUND_{len(collisions)}')
        c=collisions[0]; mesh=c.find('geometry/mesh')
        if mesh is None:fail('STL_MESH_REQUIRED_'+name)
        filename=mesh.get('filename')
        if not filename or filename.startswith('/') or '..' in Path(filename).parts:fail('UNSAFE_MESH_FILENAME_'+repr(filename))
        path=urdf.parent / filename
        if not path.is_file():fail('MISSING_MESH_'+str(path))
        scale=xyz(mesh.get('scale','1 1 1'))
        if np.any(scale<=0) or np.any(scale>1):fail('UNEXPECTED_MESH_SCALE_'+name)
        verts,faces=load_binary_stl(path)
        verts=verts*scale
        org=c.find('origin'); off=xyz(org.get('xyz','0 0 0')) if org is not None else np.zeros(3)
        rpy=xyz(org.get('rpy','0 0 0')) if org is not None else np.zeros(3)
        Roff=Rotation.from_euler('xyz',rpy).as_matrix()
        # Collision origin is in URDF link frame. Keep vertices local to this link.
        verts=verts@Roff.T+off
        bounds=np.stack((verts.min(axis=0),verts.max(axis=0)))
        out[name]=dict(vertices=verts,triangles=faces,mesh_path=str(path),scale=scale.tolist(),
                       local_bounds_m=bounds.tolist(), triangles_count=len(faces))
        print(f'STAGE41_LOAD_ARM_MESH name={name} vertices={len(verts)} triangles={len(faces)} scale={scale.tolist()}')
    if set(out)!=set(ARM_LINKS):fail('NOT_ALL_SIX_ARM_COLLISION_MESHES_FOUND')
    return out


def load_frozen_gripper(npz,baseline_pose):
    R0,p0=baseline_pose
    out={}
    with np.load(npz,allow_pickle=False) as ar:
        for name in BODIES:
            v=np.asarray(ar[name+'_world_vertices_m'],dtype=np.float64)
            f=np.asarray(ar[name+'_triangle_indices'],dtype=np.int32)
            if v.ndim!=2 or v.shape[1]!=3 or f.ndim!=2 or f.shape[1]!=3 or len(v)==0 or len(f)==0:
                fail('BAD_GRIPPER_ARRAYS_'+name)
            if f.max()>=len(v) or f.min()<0 or not np.isfinite(v).all():fail('BAD_GRIPPER_INDICES_'+name)
            # Original meshes are world-space at measured PhysX reference pose.
            # Remap to baseline gripper base frame for frozen-joint FK screen.
            local=(v-p0)@R0
            out[name]=dict(vertices=local,triangles=f,triangles_count=len(f))
            print(f'STAGE41_LOAD_GRIPPER_MESH name={name} vertices={len(v)} triangles={len(f)}')
    return out


def triangle_aabb_intersections(vertices,faces,boxlo,boxhi):
    """Exact triangle-to-axis-aligned-box SAT for surface intersections only.

    Does not detect closed containment when no surfaces intersect.
    """
    bmin=np.asarray(boxlo);bmax=np.asarray(boxhi)
    if np.any(vertices.min(0)>bmax) or np.any(vertices.max(0)<bmin):return 0
    center=(bmin+bmax)/2;h=(bmax-bmin)/2
    tris=vertices[faces]
    mask=np.all(tris.min(axis=1)<=bmax,axis=1)&np.all(tris.max(axis=1)>=bmin,axis=1)
    if not mask.any():return 0
    tri=tris[mask]-center
    e0=tri[:,1,:]-tri[:,0,:]
    e1=tri[:,2,:]-tri[:,1,:]
    e2=tri[:,0,:]-tri[:,2,:]
    axes=[np.broadcast_to(a,tri[:,0,:].shape) for a in np.eye(3)]
    axes.append(np.cross(e0,e1))
    for edge in (e0,e1,e2):
        for a in np.eye(3):axes.append(np.cross(edge,a))
    candidate=np.ones(len(tri),dtype=bool)
    for a in axes:
        pp=np.sum(tri*a[:,None,:],axis=2)
        radius=np.sum(np.abs(a)*h[None,:],axis=1)
        candidate &= (pp.min(axis=1) <= radius+1e-11) & (pp.max(axis=1) >= -radius-1e-11)
        if not candidate.any():break
    return int(candidate.sum())


def aabb_distance(a,b):
    lo1,hi1=a;lo2,hi2=b
    delta=np.maximum(0.,np.maximum(lo1-hi2,lo2-hi1))
    return float(np.linalg.norm(delta)),bool(np.all(delta==0.))


def graph_distance(a,b,adj):
    if a==b:return 0
    todo=deque([(a,0)]);seen={a}
    while todo:
        x,k=todo.popleft()
        for nbr in adj.get(x,()):
            if nbr==b:return k+1
            if nbr not in seen:seen.add(nbr);todo.append((nbr,k+1))
    return 100


def execute(args):
    for p in (args.urdf,args.robot_cfg,args.stage35,args.stage40,args.clean_gripper_npz):
        if not p.is_file():fail('MISSING_INPUT='+str(p))
    stage35=json.loads(args.stage35.read_text())
    rep=json.loads(args.stage40.read_text())
    if rep.get('stage')!='3C40' or stage35.get('stage')!='3C35':fail('STAGE_MISMATCH')
    if not rep.get('no_robot_motion',False) or not stage35.get('no_arm_descent',False):fail('MOTION_PREREQ_BAD')
    rows=rep.get('candidate_results')
    target=next((c for c in rows if c.get('index')==args.candidate),None)
    if target is None:fail('CANDIDATE_NOT_PRESENT')
    if not target.get('ik_pose_match'):fail('CANDIDATE_IK_INVALID')

    cfg=read_cfg(args.robot_cfg)
    joints,children,chain,masses=parse_urdf(args.urdf)
    (R0,p0),pivot_err,axis_err,tcp=measured_pose(joints,stage35,cfg)
    if pivot_err.max()>.5 or axis_err>.3:fail('BASELINE_HINGE_FIT_FAILED')
    q0=np.asarray(rep['baseline_estimated_q_rad'],dtype=float)
    qgoal=np.asarray(target['q_candidate_rad'],dtype=float)
    if q0.shape!=(6,) or qgoal.shape!=(6,):fail('BAD_JOINT_VECTOR')
    p_err_mm=np.linalg.norm(fk_chain(q0,cfg,joints,chain)[1]-p0)*1000.
    if p_err_mm>1.:fail('STAGE40_ESTIMATED_Q_BASELINE_POSITION_MISMATCH')
    low=np.array([joints[n]['limits'][0] for n in ARM]);high=np.array([joints[n]['limits'][1] for n in ARM])
    if np.any(q0<low) or np.any(q0>high) or np.any(qgoal<low) or np.any(qgoal>high):fail('JOINT_LIMIT_VIOLATION')

    arm=load_urdf_arm_collision(args.urdf)
    grip=load_frozen_gripper(args.clean_gripper_npz,(R0,p0))
    geoms={**arm,**grip}

    table=np.asarray(args.table_bounds,dtype=float).reshape(3,2)
    cube=np.asarray(rep['cube_center_world_m'],dtype=float)
    if not np.isfinite(cube).all():fail('BAD_CUBE_LOCATION')
    cubelo=cube-args.cube_size/2-args.cube_margin
    cubehi=cube+args.cube_size/2+args.cube_margin
    tablelo=table[:,0]-args.table_margin
    tablehi=table[:,1]+args.table_margin
    if np.any(tablelo>=tablehi):fail('BAD_TABLE_BOUNDS')

    adj={}
    for j in joints.values():
        adj.setdefault(j['parent'],set()).add(j['child'])
        adj.setdefault(j['child'],set()).add(j['parent'])
    pairs=[]
    for i,a in enumerate(BODY_NAMES):
        for b in BODY_NAMES[i+1:]:
            if graph_distance(a,b,adj)>1:pairs.append((a,b))

    print('STAGE=3C41 MODE=OFFLINE_FULL_ARM_AUTHORED_MESH_DISCRETE_PRECHECK')
    print('STAGE41_ISAAC_LAB_STARTED=False STAGE41_ROBOT_MOTION_COMMANDED=False')
    print('STAGE41_PHYSX_BASELINE_Q_INDEPENDENTLY_VERIFIED='+str(bool(rep.get('logged_q_to_measured_pose_calibrated',False))))
    print(f'STAGE41_CANDIDATE={args.candidate} JOINT_SPACE_SAMPLES={args.samples}')
    print(f'STAGE41_JOINT_START_RAD={q0.round(5).tolist()}')
    print(f'STAGE41_JOINT_TARGET_RAD={qgoal.round(5).tolist()}')

    frames=[];best_nonadj_aabb=float('inf');worstA=[]
    for k,t in enumerate(np.linspace(0,1,args.samples)):
        q=(1-t)*q0+t*qgoal
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        shapes={};bounds={};table_hits={};cube_hits={}
        for name in BODY_NAMES:
            geom=geoms[name];R,p=poses[name]
            verts=geom['vertices']@R.T+p
            faces=geom['triangles']
            shapes[name]=verts
            bounds[name]=(verts.min(0),verts.max(0))
            if not (np.any(bounds[name][0]>tablehi) or np.any(bounds[name][1]<tablelo)):
                table_hits[name]=triangle_aabb_intersections(verts,faces,tablelo,tablehi)
            if not (np.any(bounds[name][0]>cubehi) or np.any(bounds[name][1]<cubelo)):
                cube_hits[name]=triangle_aabb_intersections(verts,faces,cubelo,cubehi)
        collisions=[];overlaps=[]
        for a,b in pairs:
            dist,overlap=aabb_distance(bounds[a],bounds[b])
            best_nonadj_aabb=min(best_nonadj_aabb,dist)
            if overlap:
                overlaps.append([a,b])
            elif dist<args.self_margin:
                collisions.append([a,b,round(dist*1000,3)])
        row={
            'sample_index':k,'t':round(float(t),6),
            'q_rad':q.round(7).tolist(),
            'table_triangle_hits_per_body':table_hits,
            'cube_triangle_hits_per_body':cube_hits,
            'nonadjacent_aabb_overlaps':overlaps,
            'nonadjacent_aabb_near_pairs_lt_margin':collisions,
            'min_arm_world_z_mm':round(min(float(bounds[name][0][2]) for name in ARM_LINKS)*1000,3),
            'min_gripper_world_z_mm':round(min(float(bounds[name][0][2]) for name in BODIES)*1000,3),
        }
        frames.append(row)
        if k==0 or k==args.samples-1 or k%max(1,args.samples//8)==0:
            print('STAGE41_SAMPLE index={} t={:.3f} table_hits={} cube_hits={} self_aabb_overlap_pairs={} min_arm_z_mm={:.2f}'.format(
                k,t,sum(table_hits.values()),sum(cube_hits.values()),len(overlaps),row['min_arm_world_z_mm']),flush=True)
    n_table=sum(bool(x['table_triangle_hits_per_body']) and sum(x['table_triangle_hits_per_body'].values())>0 for x in frames)
    n_cube=sum(bool(x['cube_triangle_hits_per_body']) and sum(x['cube_triangle_hits_per_body'].values())>0 for x in frames)
    n_overlap=sum(bool(x['nonadjacent_aabb_overlaps']) for x in frames)
    near=sum(bool(x['nonadjacent_aabb_near_pairs_lt_margin']) for x in frames)
    authors_mesh_screen=(n_table==0 and n_cube==0 and n_overlap==0 and near==0)
    report={
        'stage':'3C41',
        'mode':'offline_full_arm_authored_stl_mesh_discrete_screen',
        'candidate_index':args.candidate,'sample_count':args.samples,
        'no_isaac_lab':True,'no_robot_or_finger_motion':True,'robot_assets_modified':False,
        'stage40_input_report':str(args.stage40),
        'stage40_baseline_physx_q_independently_verified':bool(rep.get('logged_q_to_measured_pose_calibrated',False)),
        'stage40_input_stage39_may_be_example':bool(rep.get('input_stage39_may_be_example',False)),
        'q_start_rad':q0.tolist(),'q_target_rad':qgoal.tolist(),
        'arm_mesh_source':'URDF collision-origin + 0.001-scaled binary STL',
        'gripper_mesh_source':'Stage3C38 PhysX-exported authored meshes, frozen articulation; no double-counting descendants',
        'arm_meshes':{name:{'vertices':len(g['vertices']),'triangles':g['triangles_count'],
                            'local_bounds_m':g['local_bounds_m']} for name,g in arm.items()},
        'table_bounds_xyz_m':table.tolist(), 'cube_center_xyz_m':cube.tolist(),
        'table_margin_m':args.table_margin,'cube_margin_m':args.cube_margin,
        'self_aabb_margin_m':args.self_margin,
        'nonadjacent_pairs_checked':len(pairs),
        'samples_with_table_surface_intersections':n_table,
        'samples_with_cube_surface_intersections':n_cube,
        'samples_with_nonadjacent_link_aabb_overlaps_UNRESOLVED':n_overlap,
        'samples_with_nonadjacent_link_aabb_near_UNRESOLVED':near,
        'sampled_authored_mesh_precheck_clear':authors_mesh_screen,
        'full_robot_collision_validated':False,
        'physx_collision_shapes_validated':False,
        'actual_baseline_joint_coordinates_calibrated':bool(rep.get('logged_q_to_measured_pose_calibrated',False)),
        'whole_trajectory_continuous_collision_validated':False,
        'robot_self_collision_narrowphase_validated':False,
        'robot_grasp_validated':False,
        'physical_motion_permission':False,
        'sample_summaries':frames,
        'disclaimer':'Samples a straight joint interpolation between INFERRED Stage40 baseline IK and candidate. '
                     'URDF STL authored collision surfaces are not proven identical to PhysX convex colliders. '
                     'Table/cube triangle-vs-AABB SAT detects surface intersection but not enclosed containment; '
                     'self-collision test is AABB-only (overlap is uncertain, nonoverlap is conservative clearance). '
                     'No robot dynamics, continuous collision sweep, measured PhysX arm q, or physical movement. '
                     'No movement toward cube is authorized.'
    }
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('STAGE41_REPORT_JSON='+str(args.out))
    print(f'STAGE41_TABLE_HIT_SAMPLES={n_table} CUBE_HIT_SAMPLES={n_cube} SELF_AABB_OVERLAP_SAMPLES={n_overlap} SELF_AABB_NEAR_SAMPLES={near}',flush=True)
    print('STAGE41_DISCRETE_AUTHORED_MESH_PRECHECK_CLEAR='+str(authors_mesh_screen))
    print('STAGE41_PHYSICAL_MOTION_PERMISSION=False')
    print('LEXIUM_STAGE3C41_OFFLINE_COLLISION_AUDIT_COMPLETE=True')
    print('RESULT=STAGE3C41_FULL_ARM_MESH_DISCRETE_PRECHECK_RECORDED_NOT_PHYSX_VALIDATED')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--robot-cfg',type=Path,required=True)
    p.add_argument('--stage35',type=Path,required=True)
    p.add_argument('--stage40',type=Path,required=True)
    p.add_argument('--clean-gripper-npz',type=Path,required=True)
    p.add_argument('--candidate',type=int,default=5)
    p.add_argument('--samples',type=int,default=41)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--table-bounds',type=float,nargs=6,default=[.17,1.07,-.6,.6,-.4,0.])
    p.add_argument('--table-margin',type=float,default=.005)
    p.add_argument('--cube-size',type=float,default=.04)
    p.add_argument('--cube-margin',type=float,default=0.)
    p.add_argument('--self-margin',type=float,default=.005)
    args=p.parse_args()
    if not (5<=args.samples<=401):fail('SAMPLES_OUT_OF_RANGE')
    if any(x<0 or x>.02 for x in [args.table_margin,args.cube_margin,args.self_margin]):fail('UNEXPECTED_MARGIN')
    if not .005<=args.cube_size<=.2:fail('UNEXPECTED_CUBE_SIZE')
    execute(args)

if __name__=='__main__':main()
