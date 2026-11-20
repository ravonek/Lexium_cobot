#!/usr/bin/env python3
"""Stage 3C.47 — HOME PhysX geometry/provenance diagnostic.

Read-only regarding USD assets and target trajectories: the script explicitly
writes the DEFAULT HOME joint state to *simulation memory* and performs one
physics step, like the verified Stage44. It does NOT replay Candidate1, command
pregrasp, alter USD files, enable self collisions, or authorize movement.

Compares actual authored USD collision mesh vertices at HOME with the rigidly
transformed frozen finger/reference geometry exported at Candidate1 in Stage38.
Also checks live joint coordinates and every URDF rigid-body pose.
This is not a physical contact or convex-collider collision test.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf',type=Path,required=True)
parser.add_argument('--stage40',type=Path,required=True)
parser.add_argument('--stage44',type=Path,required=True)
parser.add_argument('--clean-npz',type=Path,required=True)
parser.add_argument('--out',type=Path,default=Path('/tmp/lexium_stage3c47_physx_home_mesh_fingerprint.json'))
parser.add_argument('--num-envs',type=int,default=1)
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if args.num_envs!=1:parser.error('Only 1 env supported')
launcher=AppLauncher(args)
simulation_app=launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from pxr import Usd,UsdGeom,UsdPhysics,Gf
import omni.usd
from lexium_pick_place.robot_cfg import LEXIUM_COBOT_CFG,LEXIUM_ARM_JOINT_NAMES,LEXIUM_USD_PATH
from lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit import parse_urdf,rpy_to_R,axis_rotation

FINGERS=('gripper_finger_1_1','gripper_finger_2_1','gripper_finger_3_1','gripper_finger_4_1')
GRIPPER=('gripper_base_1',)+FINGERS
ARM_BODIES=('base_link','link1_1','link2_1','link3_1','link4_1','LINK5_1')
BODIES=ARM_BODIES+GRIPPER
FINGER_JOINTS=('gripper_Revolute_1','gripper_Revolute_2','gripper_Revolute_5','gripper_Revolute_6')

def quat_xyzw_to_R(quat):
    x,y,z,w=np.asarray(quat,dtype=np.float64)
    mag=math.sqrt(x*x+y*y+z*z+w*w)
    if mag<1.e-8:fail('INVALID_QUATERNION')
    x,y,z,w=x/mag,y/mag,z/mag,w/mag
    return np.array([
        [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]],dtype=np.float64)

def rot_error_deg(a,b):
    return float(np.rad2deg(np.arccos(np.clip((np.trace(a.T@b)-1.)*.5,-1.,1.))))

@configclass
class HomeMeshSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg=LEXIUM_COBOT_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    light: AssetBaseCfg=AssetBaseCfg(prim_path='/World/Light',spawn=sim_utils.DomeLightCfg(intensity=2000.))


def fail(s):raise RuntimeError('STAGE47_'+s)


def recursive_link_fk(joints,children,root,qmap,root_world):
    """Compute URDF link world transforms using ALL measured runtime joints."""
    poses={root:root_world};todo=[root]
    while todo:
        parent=todo.pop()
        Rp,pp=poses[parent]
        for name in children.get(parent,()):
            j=joints[name]
            Rj=Rp@rpy_to_R(j['rpy'])
            pj=pp+Rp@j['xyz']
            qi=float(qmap.get(name,0.))
            typ=j['type']
            if typ in ('revolute','continuous'):
                Rc,pc=Rj@axis_rotation(j['axis'],qi),pj
            elif typ=='prismatic':
                Rc,pc=Rj,pj+Rj@j['axis']*qi
            else:
                Rc,pc=Rj,pj
            poses[j['child']]=(Rc,pc)
            todo.append(j['child'])
    return poses


def closest_rigid_body(prim):
    p=prim.GetParent()
    while p and p.IsValid() and str(p.GetPath())!='/':
        if p.HasAPI(UsdPhysics.RigidBodyAPI):return p
        p=p.GetParent()
    return None


def runtime_gripper_meshes(stage, robot, body_names):
    """Export each mesh exactly once under its closest rigid-body ancestor.

    Mirror Stage37 local coordinates: USD mesh -> rigid transform local ->
    *actual* PhysX rigid body pose. Does not use persistent USD edits.
    """
    xf=UsdGeom.XformCache(Usd.TimeCode.Default())
    meshes=[p for p in stage.Traverse(Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)
            and str(p.GetPath()).startswith('/World/envs/env_0/Robot/')]
    rigid=[p for p in stage.Traverse()
           if p.HasAPI(UsdPhysics.RigidBodyAPI)
           and str(p.GetPath()).startswith('/World/envs/env_0/Robot/')]
    rmap={}
    for body in GRIPPER:
        arr=[p for p in rigid if p.GetName()==body]
        if len(arr)!=1:fail(f'RIGID_COUNT_{body}_{len(arr)}')
        rmap[body]=arr[0]
    grouped={b:[] for b in GRIPPER}
    for mesh in meshes:
        parent=closest_rigid_body(mesh)
        if parent is not None:
            for body,rprim in rmap.items():
                if parent.GetPath()==rprim.GetPath():
                    grouped[body].append(mesh)
    result={}
    poses=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    for name in GRIPPER:
        if name not in body_names:fail('NO_PHYSX_BODY_'+name)
        prim=rmap[name]
        parts=grouped[name]
        if not parts:fail('NO_AUTHORED_COLLISION_MESH_'+name)
        world_to_link=xf.GetLocalToWorldTransform(prim).GetInverse()
        pose=poses[body_names.index(name)]
        R=quat_xyzw_to_R(pose[3:7]);t=pose[:3].astype(np.float64)
        verts=[];part_meta=[]
        for mesh in parts:
            raw=UsdGeom.Mesh(mesh).GetPointsAttr().Get()
            if raw is None or not len(raw):fail('EMPTY_MESH_'+str(mesh.GetPath()))
            mesh_world=xf.GetLocalToWorldTransform(mesh)
            local=np.empty((len(raw),3),dtype=np.float64)
            for i,p0 in enumerate(raw):
                p1=world_to_link.Transform(mesh_world.Transform(Gf.Vec3d(*map(float,p0))))
                local[i]=[float(p1[0]),float(p1[1]),float(p1[2])]
            verts.append(local@R.T+t)
            part_meta.append({'path':str(mesh.GetPath()),'vertex_count':len(raw)})
        result[name]={'vertices':np.concatenate(verts),'parts':part_meta}
    return result,meshes


def main():
    for p in (args.urdf,args.stage40,args.stage44,args.clean_npz):
        if not p.is_file():fail('MISSING_INPUT_'+str(p))
    if not Path(LEXIUM_USD_PATH).is_file():fail('MISSING_USD_'+LEXIUM_USD_PATH)
    st40=json.loads(args.stage40.read_text())
    st44=json.loads(args.stage44.read_text())
    if st40.get('stage')!='3C40' or st44.get('stage')!='3C44' or st44.get('explicit_home_fk_match') is not True:
        fail('PRIOR_STAGE_MISMATCH')
    if st44.get('physical_motion_permission') is not False:fail('PRIOR_POLICY_CHANGED')
    print('STAGE=3C47 MODE=SIM_ONLY_EXPLICIT_HOME_GEOMETRY_FINGERPRINT',flush=True)
    print('STAGE47_NO_CANDIDATE_REPLAY=True NO_PREGRASP_MOVEMENT=True',flush=True)
    print('STAGE47_ONE_PHYSICS_STEP=True USD_FILES_MODIFIED=False SELF_COLLISIONS_CHANGED=False',flush=True)
    links,joints,children,rootname=parse_urdf(args.urdf)
    sim=SimulationContext(sim_utils.SimulationCfg(device=args.device,dt=1./60.,gravity=(0.,0.,-9.81)))
    scene=InteractiveScene(HomeMeshSceneCfg(num_envs=1,env_spacing=2.5))
    sim.reset();scene.reset();scene.update(sim.get_physics_dt())
    robot=scene['robot']
    if not robot.is_fixed_base:fail('BASE_NOT_FIXED')
    jnames=list(robot.data.joint_names)
    bnames=list(robot.data.body_names)
    for name in (*LEXIUM_ARM_JOINT_NAMES,*FINGER_JOINTS):
        if name not in jnames:fail('JOINT_MISSING_'+name)
    for name in BODIES:
        if name not in bnames:fail('BODY_MISSING_'+name)
    qreset=robot.data.joint_pos.torch[0].detach().cpu().numpy().copy()
    qdefault=robot.data.default_joint_pos.torch[0].detach().cpu().numpy().copy()
    full_default=robot.data.default_joint_pos.torch.clone()
    # Explicit HOME joint-state write is a SIMULATION RESET/TELEPORT, not a trajectory.
    robot.write_joint_state_to_sim_index(position=full_default,velocity=torch.zeros_like(full_default))
    scene.update(0.0)
    robot.actuators.target_command.set_position_index(value=full_default,full_data=True)
    scene.write_data_to_sim()
    sim.step();scene.update(sim.get_physics_dt())
    qnow=robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
    bodyposes=robot.data.body_link_pose_w.torch[0].detach().cpu().numpy().astype(float)
    if not np.isfinite(qnow).all() or not np.isfinite(bodyposes).all():fail('NONFINITE_PHYSX')
    qmap=dict(zip(jnames,qnow))
    pose_root=bodyposes[bnames.index(rootname)]
    pred=recursive_link_fk(joints,children,rootname,qmap,(quat_xyzw_to_R(pose_root[3:7]),pose_root[:3]))
    per_body_fk={}
    for name in BODIES:
        if name not in pred:fail('MISSING_FK_'+name)
        R,p=pred[name]
        runtime=bodyposes[bnames.index(name)]
        pe=float(np.linalg.norm(p-runtime[:3])*1e3)
        oe=rot_error_deg(R,quat_xyzw_to_R(runtime[3:7]))
        per_body_fk[name]={'position_error_mm':pe,'orientation_error_deg':oe}
        print(f'STAGE47_BODY_FK name={name} pos_error_mm={pe:.5f} ori_error_deg={oe:.6f}',flush=True)
    max_pos=max(v['position_error_mm'] for v in per_body_fk.values())
    max_ori=max(v['orientation_error_deg'] for v in per_body_fk.values())
    homejoint=np.asarray([qmap[n] for n in LEXIUM_ARM_JOINT_NAMES])
    defjoint=np.asarray([qdefault[jnames.index(n)] for n in LEXIUM_ARM_JOINT_NAMES])
    homeerr=float(np.max(np.abs(homejoint-defjoint)))
    fingerjoints={name:{'physx_rad':float(qmap[name]),'default_rad':float(qdefault[jnames.index(name)]),
                        'delta_rad':float(qmap[name]-qdefault[jnames.index(name)])} for name in FINGER_JOINTS}
    # Actual HOME collision mesh vs previously exported Candidate1 geometry,
    # frozen rigidly in the gripper_base frame. Point order is expected to match
    # because Stage38 keeps the same USD point order.
    import omni.usd
    stage=omni.usd.get_context().get_stage()
    actual,collision_meshes=runtime_gripper_meshes(stage,robot,bnames)
    ref_p=np.asarray(st40['measured_baseline_gripper_base_world_xyz_m'],dtype=float)
    ref_R=quat_xyzw_to_R(st40['measured_baseline_gripper_base_world_quat_xyzw'])
    at_home=bodyposes[bnames.index('gripper_base_1')]
    Rh=quat_xyzw_to_R(at_home[3:7]);ph=at_home[:3]
    differences={}
    with np.load(args.clean_npz,allow_pickle=False) as ar:
        for name in GRIPPER:
            ref=np.asarray(ar[name+'_world_vertices_m'],dtype=np.float64)
            obs=actual[name]['vertices']
            entry={'reference_vertices':len(ref),'runtime_home_vertices':len(obs),
                   'actual_runtime_mesh_parts':actual[name]['parts']}
            if len(ref)==len(obs) and len(ref)>0:
                predicted=(ref-ref_p)@ref_R@Rh.T+ph
                delta=np.linalg.norm(predicted-obs,axis=1)*1e3
                entry.update({'max_vertex_delta_mm':float(delta.max()),'mean_vertex_delta_mm':float(delta.mean()),
                              'centroid_delta_mm':float(np.linalg.norm(predicted.mean(0)-obs.mean(0))*1e3),
                              'reference_frozen_geometry_matches_home_within_0_5mm':bool(delta.max()<.5)})
            else:
                entry['comparison_status']='VERTEX_COUNT_MISMATCH_CANNOT_POINTWISE_VERIFY'
            differences[name]=entry
            print('STAGE47_GRIPPER_MESH name={} vertices={} reference={} max_mismatch_mm={} static_frozen_matches={}'.format(
                name,len(obs),len(ref),
                f"{entry['max_vertex_delta_mm']:.4f}" if 'max_vertex_delta_mm' in entry else 'N_A',
                entry.get('reference_frozen_geometry_matches_home_within_0_5mm','UNRESOLVED')),flush=True)

    # Read, do not modify, collision schemas and effective articulation policy.
    art_flags=[];col_stats={}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            value=prim.GetAttribute('physxArticulation:enabledSelfCollisions')
            art_flags.append({'path':str(prim.GetPath()),'enabled_self_collisions':value.Get() if value.IsValid() else None})
    for prim in collision_meshes:
        owner=closest_rigid_body(prim)
        body=owner.GetName() if owner is not None else 'NO_RIGID_ANCESTOR'
        entry=col_stats.setdefault(body,{'mesh_collision_prims':0,'mesh_approximations':{},'collision_enabled_counts':{}})
        entry['mesh_collision_prims']+=1
        aprox='NOT_AUTHORED'
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            ap=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr()
            if ap.IsValid():aprox=str(ap.Get())
        entry['mesh_approximations'][aprox]=entry['mesh_approximations'].get(aprox,0)+1
        ena=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        ev=str(ena.Get()) if ena.IsValid() else 'NOT_AUTHORED'
        entry['collision_enabled_counts'][ev]=entry['collision_enabled_counts'].get(ev,0)+1
    print('STAGE47_ARM_Q_AFTER_HOME_RAD='+json.dumps(homejoint.tolist()),flush=True)
    print('STAGE47_FINGER_JOINTS_RAD='+json.dumps({k:round(v['physx_rad'],6) for k,v in fingerjoints.items()}),flush=True)
    print(f'STAGE47_HOME_JOINT_MAX_ERROR_RAD={homeerr:.6f}',flush=True)
    print(f'STAGE47_URDF_FK_MAX_ALL_BODY_POS_MM={max_pos:.6f} ORI_DEG={max_ori:.6f}',flush=True)
    print('STAGE47_SELF_COLLISIONS_POLICY='+json.dumps(art_flags),flush=True)
    finger_compatible=all(differences[n].get('reference_frozen_geometry_matches_home_within_0_5mm') is True for n in FINGERS)
    print('STAGE47_FROZEN_FINGER_REFERENCE_VALID_AT_HOME='+str(finger_compatible),flush=True)
    report={'stage':'3C47','mode':'physx_home_runtime_geometry_provenance','sim_only':True,
            'explicit_home_state_written':True,'physics_steps':1,'pregrasp_motion_commanded':False,
            'candidate1_replayed':False,'usd_assets_modified':False,'self_collisions_modified':False,
            'table_cube_present':False,'home_joint_q_physx_rad':homejoint.tolist(),
            'home_arm_q_error_from_default_rad':homeerr,'finger_joint_readings':fingerjoints,
            'urdf_runtime_rigid_body_fk_errors':per_body_fk,
            'max_urdf_runtime_position_error_mm_all_bodies':max_pos,
            'max_urdf_runtime_orientation_error_deg_all_bodies':max_ori,
            'reference_stage40_candidate1_is_inferred_only':bool(not st40.get('logged_arm_q_used')),
            'reference_baseline_gripper_base_xyz_m':ref_p.tolist(),
            'home_runtime_gripper_base_xyz_m':ph.tolist(),
            'frozen_gripper_geometry_home_comparison':differences,
            'finger_reference_geometry_matches_home_at_0_5mm':finger_compatible,
            'articulation_self_collision_policies':art_flags,
            'usd_authored_collision_mesh_schema_summary':col_stats,
            'actual_physx_narrowphase_contact_validated':False,
            'full_robot_contact_free_path_validated':False,
            'gripper_cube_40mm_opening_validated':False,
            'motion_permission':False,'grasp_permission':False,
            'disclaimer':('This stage tests only HOME via a one-step PhysX snapshot and authored USD mesh fingerprint. '
               'It does not test active PhysX convex shapes, contact manifold or self-collision narrowphase. '
               'If enabledSelfCollisions=false, zero PhysX self-contact is not proof of clearance. '
               'No cube/table, no trajectory or physical approach tested.')}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print('STAGE47_REPORT_JSON='+str(args.out),flush=True)
    print('STAGE47_NO_MOTION_PERMISSION=True STAGE47_NO_GRASP_PERMISSION=True',flush=True)
    print('LEXIUM_STAGE3C47_HOME_GEOMETRY_AUDIT_COMPLETE=True',flush=True)

try:
    main()
finally:
    simulation_app.close()
