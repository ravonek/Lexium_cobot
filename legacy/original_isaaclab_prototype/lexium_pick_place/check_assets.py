#!/usr/bin/env python3
"""Check URDF paths, ordered joints and STL references without Isaac Lab.
Run: python3 check_assets.py
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / 'lexium_pick_place' / 'assets' / 'lexium_cobot.urdf'
ARM = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6_gripper_rotation']
FINGERS = ['gripper_Revolute_1', 'gripper_Revolute_2', 'gripper_Revolute_5', 'gripper_Revolute_6']
print(f'URDF: {URDF}')
if not URDF.is_file():
    print('ERROR: Place lexium_cobot.urdf in lexium_pick_place/assets/')
    sys.exit(1)
try:
    tree = ET.parse(URDF)
except ET.ParseError as err:
    sys.exit(f'ERROR: Invalid URDF XML: {err}')
root = tree.getroot()
joints = {j.get('name'): j for j in root.findall('.//joint')}
links = {l.get('name') for l in root.findall('.//link')}
errors = 0
for name in ARM + FINGERS:
    joint = joints.get(name)
    if joint is None:
        print(f'ERROR: Missing joint: {name}')
        errors += 1
        continue
    limit = joint.find('limit')
    lo = limit.get('lower', '-') if limit is not None else '-'
    hi = limit.get('upper', '-') if limit is not None else '-'
    vel = limit.get('velocity', '-') if limit is not None else '-'
    effort = limit.get('effort', '-') if limit is not None else '-'
    print(f'{name:28s} {joint.get("type", "unknown"):12s} lower={lo:>10s} upper={hi:>10s} velocity={vel:>7s} effort={effort:>7s}')
for name in ['base_link','link1_1','link2_1','link3_1','link4_1','LINK5_1','gripper_base_1'] + [f'gripper_finger_{i}_1' for i in range(1,5)]:
    if name not in links:
        print(f'ERROR: Missing link: {name}')
        errors += 1
meshes = [m.get('filename') for m in root.findall('.//mesh') if m.get('filename')]
for mesh in sorted(set(meshes)):
    if '://' in mesh or Path(mesh).is_absolute():
        print(f'ERROR: Mesh reference is not relative: {mesh}')
        errors += 1
        continue
    target = (URDF.parent / mesh).resolve()
    if not target.is_file():
        print(f'ERROR: Missing STL/mesh file: {mesh} (expected {target})')
        errors += 1
print(f'Checked {len(ARM)+len(FINGERS)} expected joints and {len(meshes)} mesh occurrences. Errors: {errors}')
sys.exit(1 if errors else 0)
