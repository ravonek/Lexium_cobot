STAGE 3C.42: ALL IK ENDPOINTS — STATIC AUTHORED-MESH PRECHECK
=============================================================
Prerequisites: Project already contains Stage 3C.40/3C.41 Python files,
URDF/robot_cfg, six STL collision meshes and stage40_inputs mesh and JSON.

Installation:
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
unzip -o "$HOME/Downloads/lexium_stage3c42_all_ik_target_gates_bundle.zip" -d "$PROJECT"
python3 -m py_compile "$PROJECT/lexium_stage3c42_all_ik_target_gates.py"
bash -n "$PROJECT/run_lexium_stage3c42_all_ik_target_gates.sh"

Run (offline only, no Isaac Lab, no actuators):
bash "$PROJECT/run_lexium_stage3c42_all_ik_target_gates.sh"

Output:
/tmp/lexium_stage3c42_all_ik_target_gates.json

IMPORTANT: ALL candidate target poses from the included EXAMPLE stage40
report are rejected by a static authored-mesh/table precheck.
The current Stage40 baseline joint vector was INFERRED, not independently
verified using a logged PhysX FINAL_ARM_Q_RAD. Stage39 source may also be
an example. Do not authorize movement on this report.

To look for an already recorded PhysX arm-q log without rerunning simulation:
  grep -H 'FINAL_ARM_Q_RAD=' /tmp/lexium_stage3c3{3,5,6,7}*.log 2>/dev/null | tail -n 5

If a verified log exists, Stage40 wrapper can be rerun offline:
  STAGE40_BASELINE_LOG=/path/to/physx.log bash "$PROJECT/run_lexium_stage3c40_ik_reachability_audit.sh"
Then rerun 3C42 against the newly calculated Stage40 report.
Do not run previous Stage41/42 as motor commands; they are offline-only.

Static screen uses SAT triangle-vs-AABB and contained mesh vertices, NOT
full PhysX collider shapes, exact continuous collision, or robust
self-collision checks. Finger joints are frozen and table boundaries are
those in the previous reports. No proof of feasible grasp.
