#!/usr/bin/env python3
"""Stage 3C.39: OFFLINE read-only coarse opposing-finger contact screen.

Given *authored* collision triangles from the prior static PhysX pose, search
small rigid-body translations and orientation changes around Stage3C38's
hypothetical whole-gripper poses. Checks geometrical proximity of fingers
1+2 and 3+4 to OPPOSITE cube faces, not merely nearest single vertex.

IMPORTANT: All finger joint positions are frozen. None of these transforms are
IK solutions, robot commands, confirmed contact, collision-free swept paths,
or demonstrated grasps. Never apply a reported pose directly to a robot.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np

BODIES=("gripper_base_1","gripper_finger_1_1","gripper_finger_2_1",
        "gripper_finger_3_1","gripper_finger_4_1")

def Rxyz(roll,pitch,yaw):
    a,b,c=np.deg2rad([roll,pitch,yaw]);ca,sa=np.cos(a),np.sin(a);cb,sb=np.cos(b),np.sin(b);cc,sc=np.cos(c),np.sin(c)
    return np.array([[cc*cb,cc*sb*sa-sc*ca,cc*sb*ca+sc*sa],
                     [sc*cb,sc*sb*sa+cc*ca,sc*sb*ca-cc*sa],
                     [-sb,cb*sa,cb*ca]])

def cube_triangles_sat(v,faces,cube,halfwidth=0.02):
    """Count triangle intersections with axis-aligned cube, static only."""
    vv=v[faces]-cube
    half=np.array([halfwidth]*3)
    broad=(np.all(vv.min(1)<=half,axis=1) & np.all(vv.max(1)>=-half,axis=1))
    tri=vv[broad]
    if not len(tri):return 0
    e0=tri[:,1,:]-tri[:,0,:];e1=tri[:,2,:]-tri[:,1,:];e2=tri[:,0,:]-tri[:,2,:]
    axes=[np.broadcast_to(e,tri[:,0,:].shape) for e in np.eye(3)]
    axes.append(np.cross(e0,e1))
    for edge in (e0,e1,e2):
        for one in np.eye(3):axes.append(np.cross(edge,one))
    ok=np.ones(len(tri),dtype=bool)
    for ax in axes:
        projection=(tri*ax[:,None,:]).sum(2)
        radius=(np.abs(ax)*half).sum(1)
        ok&=(projection.min(1)<=radius+1e-10)&(projection.max(1)>=-radius-1e-10)
        if not ok.any():return 0
    return int(ok.sum())

def face_min_distance(P,cube,axis,sign,margin=0.003):
    # Contact hypotheses: points on the exterior side of an x/y cube face,
    # with the other horizontal coordinate and z overlapping the face patch.
    cross=1-axis
    in_patch=(np.abs(P[:,cross]-cube[cross])<=0.020+margin)&(np.abs(P[:,2]-cube[2])<=0.020+margin)
    plane=cube[axis]+sign*0.020
    if sign>0:in_patch&=(P[:,axis]>=plane)
    else:in_patch&=(P[:,axis]<=plane)
    if not in_patch.any():return None
    distances=sign*(P[in_patch,axis]-plane)
    return float(np.min(distances))

def bilateral_score(fingerA,fingerB,cube):
    proposals=[]
    for axis in (0,1):
        for sign in (-1,1):
            # Group A outside one face and group B outside opposite face.
            a=face_min_distance(fingerA,cube,axis,sign)
            b=face_min_distance(fingerB,cube,axis,-sign)
            if a is not None and b is not None:
                proposals.append((max(a,b),a+b,'xyz'[axis],sign,a,b))
    if not proposals:return None
    score=min(proposals)
    return {'worst_side_clearance_mm':round(score[0]*1000,3),
            'sum_side_clearance_mm':round(score[1]*1000,3),
            'contact_axis_world':score[2],
            'group_A_cube_face_sign':int(score[3]),
            'group_A_face_clearance_mm':round(score[4]*1000,3),
            'group_B_opposite_face_clearance_mm':round(score[5]*1000,3)}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--npz',type=Path,required=True)
    ap.add_argument('--stage38-json',type=Path,required=True)
    ap.add_argument('--stage36-json',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    s38=json.loads(a.stage38_json.read_text());s36=json.loads(a.stage36_json.read_text())
    if s38.get('stage')!='3C38' or s36.get('stage')!='3C36':raise SystemExit('STAGE39_INPUT_ERROR=STAGE_JSON_MISMATCH')
    if s38.get('no_arm_motion') is not True or s38.get('no_finger_motion') is not True:raise SystemExit('STAGE39_INPUT_ERROR=UNEXPECTED_PRIOR_MOTION')
    centers=s36.get('candidate_aperture_centers_top_10',[])
    if len(centers)!=10:raise SystemExit('STAGE39_INPUT_ERROR=EXPECTED_10_CENTERS')
    with np.load(a.npz,allow_pickle=False) as arr:
        bodies=[np.asarray(arr[n+'_world_vertices_m'],dtype=np.float64) for n in BODIES]
        triangles=[np.asarray(arr[n+'_triangle_indices'],dtype=np.int32) for n in BODIES]
    if any(v.ndim!=2 or v.shape[1]!=3 for v in bodies):raise SystemExit('STAGE39_INPUT_ERROR=MESH_SHAPE')
    cube=np.array(s38['cube_center_world_m'],dtype=float)
    if cube.shape!=(3,):raise SystemExit('STAGE39_INPUT_ERROR=CUBE_SHAPE')
    offsets=np.cumsum([0]+[len(v) for v in bodies]);all_v=np.concatenate(bodies)
    all_faces=np.concatenate([triangles[i]+offsets[i] for i in range(len(BODIES))],axis=0)
    A_slice=slice(offsets[1],offsets[3]); B_slice=slice(offsets[3],offsets[5])
    # Conservatively keep all authored mesh vertices >= 5 mm above table top.
    # Only a sampled cube proximity/contact criterion; table/cube faces are
    # checked exactly at shortlisted static poses below.
    shifts=np.arange(-0.030,0.031,0.010)
    perturb=(-15.0,0.0,15.0)
    evaluated=0;table_clear=0;bilateral=0;prechecks=[]
    print('STAGE=3C39 MODE=OFFLINE_OPPOSING_FINGER_CONTACT_HYPOTHESES',flush=True)
    print('STAGE39_ISAAC_LAB_STARTED=False ARM_MOTION=False FINGER_MOTION=False',flush=True)
    for row in s38['candidate_orientation_scan']:
        seed=row.get('closest_no_surface_overlap_hypothesis')
        if seed is None:continue
        ix=int(row['candidate_index'])
        anchor=np.array(centers[ix]['virtual_cube_center_world_m'],dtype=float)
        n_for_seed=0
        for droll in perturb:
            for dpitch in perturb:
                for dyaw in perturb:
                    angles=(seed['roll_deg']+droll,seed['pitch_deg']+dpitch,seed['yaw_deg']+dyaw)
                    rotation=Rxyz(*angles)
                    base=(all_v-anchor)@rotation.T+cube
                    zmin=float(base[:,2].min())
                    for dz_target in (0.005,0.010,0.020):
                        dz=dz_target-zmin
                        if abs(dz)>0.040:continue
                        for dx in shifts:
                            for dy in shifts:
                                evaluated+=1
                                v=base+np.array([dx,dy,dz])
                                # The selected dz_target is the lowest point of the
                                # full mesh, which guarantees table clearance when
                                # the entire table top is z=0.
                                table_clear+=1
                                score=bilateral_score(v[A_slice],v[B_slice],cube)
                                if score is None:continue
                                bilateral+=1;n_for_seed+=1
                                candidate={'seed_candidate_index':ix,'world_angles_deg':[float(q) for q in angles],
                                           'world_translation_offset_from_seed_m':[float(dx),float(dy),float(dz)],
                                           'gripper_lowest_z_mm':round(dz_target*1000,3),**score}
                                prechecks.append(candidate)
        print(f'STAGE39_SEED index={ix} opposite_face_hypotheses={n_for_seed}',flush=True)
    prechecks.sort(key=lambda x:(x['worst_side_clearance_mm'],x['sum_side_clearance_mm']))
    # For the most promising geometrical near-contact hypotheses, use full
    # authored triangle-vs-cube SAT (not just mesh vertices).
    tested=[]
    # Re-evaluate a diverse shortlist; same orientation but different offsets
    # can have different physical plausibility. This screen is NOT exhaustive.
    for r in prechecks[:250]:
        ix=r['seed_candidate_index'];anchor=np.asarray(centers[ix]['virtual_cube_center_world_m'])
        R=Rxyz(*r['world_angles_deg']);delta=np.asarray(r['world_translation_offset_from_seed_m'])
        pts=(all_v-anchor)@R.T+cube+delta
        hits=cube_triangles_sat(pts,all_faces,cube)
        inside_box=int(np.all(np.abs(pts-cube)<0.02,axis=1).sum())
        palm=pts[:offsets[1]]
        palm_outside=np.maximum(np.abs(palm-cube)-0.020,0)
        palm_closest=float(np.linalg.norm(palm_outside,axis=1).min()*1000)
        entry={**r,'cube_triangle_hits':hits,'vertices_strictly_inside_cube':inside_box,
               'palm_nearest_sampled_vertex_cube_mm':round(palm_closest,3),
               'passes_static_authored_mesh_precheck':hits==0 and inside_box==0}
        tested.append(entry)
    eligible=[r for r in tested if r['passes_static_authored_mesh_precheck']]
    within10=[r for r in eligible if r['worst_side_clearance_mm']<=10.0]
    within5=[r for r in eligible if r['worst_side_clearance_mm']<=5.0]
    report={'stage':'3C39','mode':'offline_near_contact_opposite_group_screen',
            'no_arm_motion':True,'no_finger_motion':True,'no_physics_simulation':True,
            'cube_center_world_m':cube.tolist(),'seed_orientation_count':sum(x.get('closest_no_surface_overlap_hypothesis') is not None for x in s38['candidate_orientation_scan']),
            'virtual_poses_screened':evaluated,'table_clear_5mm_prechecks':table_clear,
            'poses_with_bilateral_face_sampled_support':bilateral,
            'static_full_triangle_cube_checks':len(tested),
            'static_cube_nonintersecting_of_shortlist':len(eligible),
            'qualified_bilateral_clearance_both_sides_at_most_10mm':len(within10),
            'qualified_bilateral_clearance_both_sides_at_most_5mm':len(within5),
            'top_static_candidates':eligible[:12],
            'top_bilateral_candidates_regardless_of_triangle_hits':tested[:12],
            'grasp_validated':False,'arm_ik_validated':False,'continuous_collision_validated':False,
            'actual_physx_colliders_validated':False,'no_grasp_permission':True,
            'disclaimer':'Only authored mesh static checks in a coarse, non-exhaustive local search; sampled vertex face proximity with 3mm patch margin; no tested pose is a robot command or a grasp. Finger joints frozen; no force closure, full robot links, IK, PhysX shape, self-collision or swept path certified.'}
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(f'STAGE39_POSES_SCREENED={evaluated} TABLE_CLEAR_5MM_PRECHECKS={table_clear} BILATERAL_SUPPORT={bilateral}',flush=True)
    print(f'STAGE39_TRIANGLE_CUBE_CHECKS={len(tested)} STATIC_NONINTERSECTING={len(eligible)} BILATERAL_WITHIN_10MM={len(within10)} BILATERAL_WITHIN_5MM={len(within5)}',flush=True)
    for i,r in enumerate(eligible[:5]):
        print('STAGE39_CANDIDATE index={} seed={} angle={} translation={} side_clearances_mm=[{},{}] cube_hits={}'.format(i,r['seed_candidate_index'],r['world_angles_deg'],r['world_translation_offset_from_seed_m'],r['group_A_face_clearance_mm'],r['group_B_opposite_face_clearance_mm'],r['cube_triangle_hits']),flush=True)
    print(f'STAGE39_REPORT_JSON={a.out}',flush=True)
    print('STAGE39_NO_GRASP_PERMISSION=True',flush=True)
    print('RESULT=STAGE3C39_OFFLINE_OPPOSING_CONTACT_SCREEN_RECORDED_NOT_GRASP_VALIDATED',flush=True)
if __name__=='__main__':main()
