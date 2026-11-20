#!/usr/bin/env python3
"""Stage 3C48: OFFLINE pre-grasp orientation/IK branch search + sampled mesh audit.

No Isaac Lab, no robot motion, no USD/URDF edits and NO motion/grasp permission.
Searches 75 world-frame XYZ orientation perturbations of the Stage45 +100mm
pregrasp reference. Finds distinct URDF IK branches, screens all 11 authored
meshes vs table/cube and 45 nonadjacent mesh pairs at ENDPOINT, then performs
41-point full-robot sampled checks for endpoint-screened alternatives.

Important: Stage47 found PhysX enabledSelfCollisions=false. The authored meshes
intersect even at the initial verified HOME, so a clean endpoint or no NEW
intersections along a sampled path does not certify a collision-free path.
Actual PhysX convex collision shapes, continuous swept motion, dynamics,
40mm finger aperture and force-closure remain untested.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from lexium_stage3c40_ik_reachability_audit import ARM, read_cfg, parse_urdf, errors, quality, all_fk, measured_pose
from lexium_stage3c41_full_arm_collision_audit import (
    BODY_NAMES, graph_distance, load_urdf_arm_collision, load_frozen_gripper,
)
from lexium_stage3c45_offline_pregrasp_lift_screen import probe_pose
from lexium_stage3c46_self_mesh_narrowphase import LocalBVH, mesh_pair_screen, self_test


def die(msg):
    raise SystemExit('STAGE48_FAIL=' + str(msg))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--robot-cfg', type=Path, required=True)
    p.add_argument('--stage35', type=Path, required=True)
    p.add_argument('--stage45', type=Path, required=True)
    p.add_argument('--stage47', type=Path, required=True)
    p.add_argument('--clean-npz', type=Path, required=True)
    p.add_argument('--out', type=Path, default=Path('/tmp/lexium_stage3c48_orientation_branch_report.json'))
    p.add_argument('--samples', type=int, default=41)
    return p.parse_args()


def main():
    args = parse_args()
    for path in (args.urdf, args.robot_cfg, args.stage35, args.stage45, args.stage47, args.clean_npz):
        if not path.is_file():
            die('MISSING=' + str(path))
    if args.samples < 5 or args.samples > 101:
        die('INVALID_SAMPLES')
    s45 = json.loads(args.stage45.read_text())
    s47 = json.loads(args.stage47.read_text())
    s35 = json.loads(args.stage35.read_text())
    if s45.get('stage') != '3C45' or s47.get('stage') != '3C47' or s35.get('stage') != '3C35':
        die('STAGE_INPUT_MISMATCH')
    if not s45.get('stage44_home_fk_verified') or not s47.get('finger_reference_geometry_matches_home_at_0_5mm'):
        die('HOME_OR_FINGER_MODEL_NOT_VERIFIED')
    if s47.get('motion_permission') is not False or s47.get('grasp_permission') is not False:
        die('STAGE47_UNEXPECTED_PERMISSION')
    if s45.get('physical_motion_permission') is not False:
        die('STAGE45_UNEXPECTED_PERMISSION')
    if s47.get('actual_physx_narrowphase_contact_validated') is not False:
        die('STAGE47_CONTACT_VALIDATION_FLAG_UNEXPECTED')
    policies = s47.get('articulation_self_collision_policies', [])
    if not policies or not all(x.get('enabled_self_collisions') is False for x in policies):
        die('SELF_COLLISION_POLICY_CHANGED_VERIFY_BEFORE_REUSE')
    if not s45.get('stage40_stage39_input_may_be_example', False):
        print('STAGE48_NOTE=SOURCE_PROVENANCE_CHANGED_REVIEW_MANUALLY', flush=True)

    row = next((r for r in s45.get('target_rows',[]) if abs(r['lift_mm']-100.)<1e-6), None)
    if row is None or row.get('sampled_vertex_and_triangle_table_cube_clear') is not True:
        die('STAGE45_LIFT100_NOT_AVAILABLE_OR_NOT_TABLE_CUBE_PRECHECKED')
    cfg = read_cfg(args.robot_cfg)
    joints,children,chain,masses = parse_urdf(args.urdf)
    low = np.array([joints[n]['limits'][0] for n in ARM]); high = np.array([joints[n]['limits'][1] for n in ARM])
    qhome = np.asarray(s45['stage44_q_after_write_rad'], dtype=float)
    qorig = np.asarray(row['q_ik_rad'], dtype=float)
    pgoal = np.asarray(row['target_gripper_base_world_xyz_m'], dtype=float)
    Rgoal = Rotation.from_quat(np.asarray(s45['stage40_reference_world_gripper_base_quat_xyzw'],dtype=float)).as_matrix()
    if qhome.shape != (6,) or qorig.shape != (6,) or pgoal.shape != (3,):
        die('BAD_SHAPES')
    if not np.allclose(qhome, cfg['q_home'], atol=1e-4):
        die('HOME_CONFIG_CHANGED')
    q_alt_ik_seed = np.array([-0.288095371550809, 1.5849087856278774, 2.385172970163964,
                              0.882998866594432, 0.07224029151475227, 0.5965287662558995])
    # This seed is ONLY an optimizer initial guess, NEVER a robot command.
    (Rreference,preference),perr,aerr,_=measured_pose(joints,s35,cfg)
    if max(perr) > .5 or aerr > .3:
        die('GRIPPER_MESH_REFERENCE_CALIBRATION_FAILED')
    mesh = {**load_urdf_arm_collision(args.urdf),
            **load_frozen_gripper(args.clean_npz,(Rreference,preference))}
    bvh = {name:LocalBVH(mesh[name],12) for name in BODY_NAMES}
    graph={}
    for j in joints.values():
        graph.setdefault(j['parent'],set()).add(j['child'])
        graph.setdefault(j['child'],set()).add(j['parent'])
    pairs=[(a,b) for i,a in enumerate(BODY_NAMES) for b in BODY_NAMES[i+1:]
           if graph_distance(a,b,graph)>1]
    if len(pairs) != 45:
        die('EXPECTED_45_NONADJACENT_PAIRS')
    cube=np.asarray(s45['cube_center_world_m'],dtype=float)
    table=np.asarray(s45['table_bounds_m'],dtype=float)
    table_margin=float(s45.get('table_margin_mm',5.))/1000
    cube_margin=float(s45.get('cube_margin_mm',0.))/1000
    tablelo=table[:,0]-table_margin
    tablehi=table[:,1]+table_margin
    cubelo=cube-.020-cube_margin
    cubehi=cube+.020+cube_margin

    def screen(q):
        poses,_,_=all_fk(q,cfg,joints,children,masses)
        views={name:bvh[name].in_world(*poses[name]) for name in BODY_NAMES}
        intersections=[];unresolved=[]
        for a,b in pairs:
            status,_,_,_ = mesh_pair_screen(bvh[a],bvh[b],views[a],views[b],160000)
            if status=='SURFACE_INTERSECTION': intersections.append(a+' / '+b)
            if status=='INCONCLUSIVE_NODE_BUDGET': unresolved.append(a+' / '+b)
        geom = probe_pose(q,cfg,joints,children,masses,mesh,tablelo,tablehi,cubelo,cubehi,pairs)
        return {'self_intersection_pairs':intersections,
                'unresolved_budget_pairs':unresolved,
                'table_hit_bodies':sorted(geom['table_hits_per_body']),
                'cube_hit_bodies':sorted(geom['cube_hits_per_body']),
                'min_arm_z_mm':round(geom['min_arm_z_m']*1000.,3),
                'min_gripper_z_mm':round(geom['min_gripper_z_m']*1000.,3)}

    self_test()
    print('STAGE=3C48 MODE=OFFLINE_ORIENTATION_IK_AND_FULL_MESH_SCREEN',flush=True)
    print('STAGE48_ISAAC_LAB_STARTED=False MOTION_COMMANDED=False ASSETS_MODIFIED=False',flush=True)
    print('STAGE48_REFERENCE_LIFT_MM=100.0 STAGE48_SOURCE_MAY_BE_EXAMPLE=True',flush=True)
    print('STAGE48_FINGER_REFERENCE_HOME_MATCH=True',flush=True)
    print('STAGE48_PHYSX_SELF_COLLISIONS_ENABLED=False',flush=True)

    examined=0;ik_unique=0;screened=[];clean=[]
    for roll in (-30,-15,0,15,30):
        for pitch in (-30,-15,0,15,30):
            for yaw in (-30,0,30):
                examined+=1
                perturb=np.array([roll,pitch,yaw],dtype=float)
                goal=(Rotation.from_euler('xyz',perturb,degrees=True).as_matrix()@Rgoal,pgoal)
                unique=[]
                for seed in (qorig,q_alt_ik_seed):
                    fit=least_squares(errors,np.clip(seed,low+1e-4,high-1e-4),
                        args=(goal,cfg,joints,chain),bounds=(low,high),max_nfev=120,
                        xtol=1.e-8,ftol=1.e-8,gtol=1.e-8)
                    poserr,orierr=quality(fit.x,goal,cfg,joints,chain)
                    margin=float(np.rad2deg(np.min(np.minimum(fit.x-low,high-fit.x))))
                    if poserr<.5 and orierr<.5 and margin>5 and all(np.linalg.norm(fit.x-q)>.1 for q in unique):
                        unique.append(fit.x)
                for q in unique:
                    ik_unique+=1
                    endpoint=screen(q)
                    summary={'orientation_world_xyz_deg':[roll,pitch,yaw],
                             'q_ik_rad':[float(v) for v in q],
                             'endpoint':endpoint}
                    if not endpoint['self_intersection_pairs'] and not endpoint['unresolved_budget_pairs'] \
                        and not endpoint['table_hit_bodies'] and not endpoint['cube_hit_bodies']:
                        clean.append(summary)
                        print('STAGE48_ENDPOINT_CLEAR orientation={} min_gripper_z_mm={:.2f} q_rad={}'.format(
                            summary['orientation_world_xyz_deg'],endpoint['min_gripper_z_mm'],
                            np.round(q,4).tolist()),flush=True)
                    screened.append(summary)
    print(f'STAGE48_ORIENTATIONS_TESTED={examined} IK_SOLUTIONS={ik_unique} ENDPOINT_MESH_CLEAR={len(clean)}',flush=True)

    home_screen=screen(qhome)
    home_pairs=set(home_screen['self_intersection_pairs'])
    # Full path audit only for a pre-selected, previously screened orientation.
    # Other six candidate IK endpoints remain available for later path planning.
    chosen=[c for c in clean if c['orientation_world_xyz_deg']==[30,-15,30]]
    if len(chosen)!=1:
        die('PREFERRED_ENDPOINT_NOT_UNIQUELY_FOUND')
    print('STAGE48_FULL_PATHS_SELECTED=1 OTHER_CANDIDATES_ENDPOINT_ONLY='+str(len(clean)-1),flush=True)
    results=[]
    for c in chosen:
        path=[]
        q=np.array(c['q_ik_rad'])
        for i,t in enumerate(np.linspace(0.,1.,args.samples)):
            qstep=qhome*(1.-t)+q*t
            f=screen(qstep)
            f.update({'sample_index':i,'fraction':round(float(t),6),
                      'new_relative_to_home_intersection_pairs':sorted(set(f['self_intersection_pairs'])-home_pairs)})
            path.append(f)
        c={**c,'path_samples':args.samples,
           'path_frames_with_any_surface_intersection':sum(bool(f['self_intersection_pairs']) for f in path),
           'path_frames_with_new_surface_intersection':sum(bool(f['new_relative_to_home_intersection_pairs']) for f in path),
           'path_frames_with_table_hit':sum(bool(f['table_hit_bodies']) for f in path),
           'path_frames_with_cube_hit':sum(bool(f['cube_hit_bodies']) for f in path),
           'path_frames_with_budget_unresolved':sum(bool(f['unresolved_budget_pairs']) for f in path),
           'path_results':path}
        results.append(c)
        print('STAGE48_PATH orientation={} self_hits={}/{} new_vs_home={}/{} table_hits={} cube_hits={}'.format(
            c['orientation_world_xyz_deg'],c['path_frames_with_any_surface_intersection'],args.samples,
            c['path_frames_with_new_surface_intersection'],args.samples,
            c['path_frames_with_table_hit'],c['path_frames_with_cube_hit']),flush=True)
    results.sort(key=lambda c:(c['path_frames_with_table_hit']+c['path_frames_with_cube_hit'],
                              c['path_frames_with_new_surface_intersection'],
                              c['path_frames_with_any_surface_intersection'],
                              -c['endpoint']['min_gripper_z_mm']))
    preferred=results[0] if results else None
    report={'stage':'3C48','mode':'offline_orientation_ik_mesh_search','no_isaac_lab':True,
            'no_robot_motion':True,'assets_modified':False,'orientation_grid_count':examined,
            'distinct_ik_solutions_tested':ik_unique,'clean_mesh_endpoints':len(clean),
            'stage45_reference_lift_mm':100.,'stage39_reference_may_be_example':True,
            'stage47_home_finger_mesh_reference_verified':True,
            'stage47_physx_self_collisions_enabled':False,
            'urdf_home_mesh_surface_intersections':sorted(home_pairs),
            'home_table_hit_bodies':home_screen['table_hit_bodies'],
            'home_cube_hit_bodies':home_screen['cube_hit_bodies'],
            'endpoint_clear_candidates':clean,
            'sampled_full_path_candidates':results,
            'preferred_orientation_world_xyz_deg':preferred['orientation_world_xyz_deg'] if preferred else None,
            'preferred_q_ik_rad':preferred['q_ik_rad'] if preferred else None,
            'preferred_path_frames_with_any_surface_intersection':preferred['path_frames_with_any_surface_intersection'] if preferred else None,
            'preferred_path_frames_with_new_surface_intersection':preferred['path_frames_with_new_surface_intersection'] if preferred else None,
            'physx_narrowphase_validated':False,
            'continuous_motion_collision_validated':False,
            'gripper_opening_40mm_validated':False,
            'grasp_force_closure_validated':False,
            'robot_motion_permission':False,'grasp_permission':False,
            'disclaimer':('Every candidate is OFFLINE, discrete, authored mesh only. HOME has existing ' 
                          'authored surface intersections. Filtering home pairs from the NEW pair metric ' 
                          'does not mean the remaining path is collision-free. PhysX has self collisions ' 
                          'disabled, so absence of self contacts is not proof of clearance. ' 
                          'Target orientation derives from Stage39 example geometry. No motion/grasp permission.')}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print('STAGE48_HOME_PREEXISTING_SURFACE_PAIRS='+json.dumps(sorted(home_pairs)),flush=True)
    if preferred:
        print('STAGE48_PREFERRED_WORLD_RPY_DEG='+json.dumps(preferred['orientation_world_xyz_deg']),flush=True)
        print('STAGE48_PREFERRED_Q_RAD='+json.dumps([round(x,6) for x in preferred['q_ik_rad']]),flush=True)
        print('STAGE48_PREFERRED_NEW_SELF_FRAMES='+str(preferred['path_frames_with_new_surface_intersection']),flush=True)
        print('STAGE48_PREFERRED_TABLE_FRAMES='+str(preferred['path_frames_with_table_hit']),flush=True)
        print('STAGE48_PREFERRED_CUBE_FRAMES='+str(preferred['path_frames_with_cube_hit']),flush=True)
    else:
        print('STAGE48_PREFERRED=NONE_FOUND',flush=True)
    print('STAGE48_ROBOT_MOTION_PERMISSION=False',flush=True)
    print('STAGE48_GRASP_PERMISSION=False',flush=True)
    print('STAGE48_REPORT_JSON='+str(args.out),flush=True)
    print('LEXIUM_STAGE3C48_OFFLINE_ORIENTATION_AUDIT_COMPLETE=True',flush=True)
    print('RESULT=STAGE3C48_OFFLINE_IK_ALTERNATIVES_FOUND_NOT_PHYSX_OR_GRASP_VALIDATED',flush=True)


if __name__=='__main__':
    main()
