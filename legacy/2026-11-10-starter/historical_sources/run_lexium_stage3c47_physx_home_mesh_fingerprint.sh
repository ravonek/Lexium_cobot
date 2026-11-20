#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
ISAACLAB="$HOME/IsaacLab-3.0-EA"
SCRIPT="$PROJECT/lexium_stage3c47_physx_home_mesh_fingerprint.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
STAGE40=/tmp/lexium_stage3c40_ik_reachability_report.json
STAGE44=/tmp/lexium_stage3c44_explicit_home_fk_report.json
CLEAN="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
LOG=/tmp/lexium_stage3c47_physx_home_mesh_fingerprint.log
REPORT=/tmp/lexium_stage3c47_physx_home_mesh_fingerprint.json
for f in "$SCRIPT" "$URDF" "$PROJECT/robot_cfg.py" "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" "$STAGE40" "$STAGE44" "$CLEAN"; do
 [[ -s "$f" ]] || { echo "STAGE47_REQUIRED_FILE_MISSING=$f" >&2; exit 3; }
done
[[ -d "$ISAACLAB" ]] || { echo "STAGE47_ISAACLAB_MISSING=$ISAACLAB" >&2; exit 3; }
python3 -m py_compile "$SCRIPT"
echo 'STAGE=3C47 MODE=PHYSX_HOME_GEOMETRY_FINGERPRINT'
echo 'SIM_ONLY=True ONE_PHYSICS_STEP=True NO_PREGRASP_MOVEMENT=True'
echo "LOG=$LOG REPORT=$REPORT"
cd "$ISAACLAB"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --num-envs 1 --visualizer none --urdf "$URDF" \
  --stage40 "$STAGE40" --stage44 "$STAGE44" \
  --clean-npz "$CLEAN" --out "$REPORT" >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== STAGE3C47 =========='
grep -E '^STAGE47_|^LEXIUM_STAGE3C47_' "$LOG" || true
echo '========== DIAGNOSTIC ERRORS =========='
grep -Ei '(Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if (( status!=0 )); then tail -n 40 "$LOG"; exit "$status"; fi
for mark in 'STAGE47_NO_MOTION_PERMISSION=True STAGE47_NO_GRASP_PERMISSION=True' 'LEXIUM_STAGE3C47_HOME_GEOMETRY_AUDIT_COMPLETE=True'; do
 grep -Fxq "$mark" "$LOG" || { echo "STAGE47_MISSING_MARKER=$mark" >&2; exit 4; }
done
[[ -s "$REPORT" ]] || { echo 'STAGE47_REPORT_MISSING' >&2; exit 4; }
python3 - "$REPORT" <<'PYVERIFY'
import json,sys,math
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C47' and r['sim_only'] and r['physics_steps']==1
assert r['pregrasp_motion_commanded'] is False and r['grasp_permission'] is False
assert len(r['finger_joint_readings'])==4 and len(r['frozen_gripper_geometry_home_comparison'])==5
assert math.isfinite(r['max_urdf_runtime_position_error_mm_all_bodies'])
print('STAGE47_REPORT_VERIFIED=True')
PYVERIFY
cp "$REPORT" "$HOME/Downloads/lexium_stage3c47_physx_home_mesh_fingerprint.json"
echo "STAGE47_DOWNLOADS_REPORT=$HOME/Downloads/lexium_stage3c47_physx_home_mesh_fingerprint.json"
echo 'RESULT=STAGE3C47_HOME_FINGERPRINT_CAPTURED_NOT_MOTION_AUTHORIZED'
