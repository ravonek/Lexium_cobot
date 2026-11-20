Lexium Cobot — Stage 3C.41
FULL ARM COLLISION & PRE-GRASP: READ-ONLY OFFLINE PRECHECK
========================================================

WHAT THIS DOES
- Reads the six URDF collision STL meshes with the exact URDF origin and scale 0.001.
- Uses frozen calibrated Stage3C38 gripper mesh geometry without duplicate finger meshes.
- Uses Stage3C40 inferred baseline IK and target candidate #5 by default.
- Checks sampled straight-joint interpolation against a static table box (5mm margin)
  and cube (no margin) via triangle-vs-box SAT, plus non-adjacent link AABB proximity.
- Writes /tmp/lexium_stage3c41_full_arm_collision_report.json.

WHAT THIS DOES NOT DO
- NEVER starts Isaac Lab, commands a joint, or edits URDF/USD/config.
- Does NOT independently verify the Stage40 inferred baseline q against PhysX.
- Does NOT perform exact narrow-phase self-collision or a continuous swept-path check.
- Does NOT establish actual PhysX collision geometry, motion safety, grasp or lift.
- The Stage40 input may be based on an example Stage39 report.

PREREQUISITES
The Lexium project previously installed Stage40:
  PROJECT=$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place
  $PROJECT/lexium_stage3c40_ik_reachability_audit.py
  $PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json
  $PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz
  /tmp/lexium_stage3c40_ik_reachability_report.json
or downloaded Stage40 report in ~/Downloads.
User-provided STL meshes must be at $PROJECT/assets/meshes/<name>.stl

INSTALL
  unzip -o ~/Downloads/lexium_stage3c41_full_arm_audit_bundle.zip -d "$PROJECT"

RUN
  bash "$PROJECT/run_lexium_stage3c41_full_arm_collision_audit.sh"

COPY RESULTS
  cp /tmp/lexium_stage3c41_full_arm_collision_report.json ~/Downloads/

EXAMPLE IN BUNDLE (computed from provided data)
  Stage41 candidate 5, 41 joint interpolation samples:
  29/41 samples with table surface intersections in URDF/STL model (link2_1).
  0 cube-triangle surface hits; 41/41 samples with non-adjacent AABB overlaps,
  which are UNRESOLVED warnings, not proven self-collisions.
  Baseline arm q inferred only, independent PhysX joint coordinates unavailable.
  NO MOTION AUTHORIZED.
