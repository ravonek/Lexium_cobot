Lexium Cobot Stage 3C48 — Orientation/IK branch search + authored mesh offline audit

Files:
- lexium_stage3c48_orientation_branch_screen.py
- run_lexium_stage3c48_orientation_branch_screen.sh
- lexium_stage3c48_offline_example_report.json (tested in an offline environment using previously supplied robot assets)

Install into:
  ~/Desktop/adi/lexium_rl_workspace/lexium_pick_place

Prerequisites (previous stages):
  Python: numpy scipy, existing Stage40/Stage41/Stage45/Stage46 Python modules.
  assets/lexium_cobot.urdf and assets/meshes/*.stl
  robot_cfg.py
  stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json
  stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz
  ~/Downloads/lexium_stage3c45_offline_pregrasp_report.json
  ~/Downloads/lexium_stage3c47_physx_home_mesh_fingerprint.json

Run:
  PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
  bash "$PROJECT/run_lexium_stage3c48_orientation_branch_screen.sh"

Output:
  ~/Downloads/lexium_stage3c48_orientation_branch_report.json

This searches 75 world orientation offsets (roll,pitch,yaw = -30..30 deg), two IK
initialization seeds per offset, screening all 11 authored body meshes at the
endpoint. The preferred orientation (+30,-15,+30) is also tested at 41 sampled
joint interpolation points from explicit HOME. The other endpoint-only candidates
have NOT undergone full path sampling in this stage.

IMPORTANT NOT A MOTION-CLEARANCE CERTIFICATE:
- PhysX articulation enabledSelfCollisions is FALSE (Stage47).
- Authored mesh surface intersections already exist at HOME. The preferred
  path has no NEW nonadjacent body pair intersections relative to these HOME
  pairs in the 41 offline samples, but preexisting intersections persist in
  some samples. Zero new pairs is NOT equivalent to a collision-free path.
- PhysX convex colliders, actual narrow-phase contacts, margin separation,
  continuous swept volumes, joint control dynamics, finger opening, and 40mm
  cube grasp are NOT validated.
- Goal inherited from Stage39 example pose, not a confirmed optimal grasp.
- DO NOT send the reported joint positions to the robot or PhysX actuators.
