#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c33_static_gripper_geometry.py"
LOG=/tmp/lexium_stage3c33_static_gripper_geometry.log
REPORT=/tmp/lexium_stage3c33_static_gripper_geometry.json
PREV=/tmp/lexium_stage3c32b_limit_aware_probe.json
BASE=/tmp/lexium_stage3c30_full_contact.log
STAGE31_LOG=/tmp/lexium_stage3c31_gripper_geometry.log
for f in "$SCRIPT" "$PREV" "$BASE" "$STAGE31_LOG" \
 "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
 "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
 "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
 /tmp/lexium_stage3c29_preflight.log \
 /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
 /tmp/lexium_stage3c26_candidates.json; do
  [[ -s "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
python3 - "$PREV" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r.get('stage')=='3C32B' and r.get('mode')=='probe', 'NOT_3C32B_PROBE_REPORT'
assert r.get('probe_complete') is True, 'STAGE32B_PROBE_NOT_COMPLETE'
assert r.get('actual_passes')==7 and r.get('skipped_limit')==1, 'STAGE32B_EXPECTED_7_PASS_1_SKIP'
assert r.get('self_collisions_enabled') is False, 'SELF_COLLISION_BASELINE_CHANGED'
print('PREVIOUS_STAGE32B_REPORT_VERIFIED=True')
PY
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$BASE" || { echo "MISSING_BASELINE_MARKER=$marker" >&2; exit 4; }
done
for marker in 'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$STAGE31_LOG" || { echo "MISSING_STAGE31_MARKER=$marker" >&2; exit 5; }
done
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo 'STAGE=3C33 MODE=STATIC_AUDIT'
echo 'Replay verified trajectory; measure collider cross-sections after physical hold.'
echo 'NO gripper motion, NO descent, NO simultaneous opening, NO URDF/USD changes.'
echo "LOG=$LOG"
echo "REPORT=$REPORT"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --scene-variant full --table-center-x 0.62 --cube-x 0.30 --cube-y -0.30 \
  --robot-contact-threshold-n 0.5 \
  --warmup-steps 120 --motion-steps 900 --hold-steps 300 \
  --num_envs 1 --visualizer none \
  --stage33-report "$REPORT" >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== SAFETY / BASELINE =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|EXTERNAL_CONTACT_SCREEN_PASSED=|LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=)' "$LOG" || true
echo '========== STAGE 3C33 =========='
grep -E '^(LEXIUM_STAGE3C33_|STAGE33_|GRASP_VALIDATED=)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 12 || true
if ((status!=0)); then echo 'RESULT=STAGE3C33_FAILED'; tail -n 28 "$LOG"; exit "$status"; fi
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True' \
              'LEXIUM_STAGE3C33_STATIC_GEOMETRY_AUDIT_COMPLETE=True' \
              'STAGE33_FINGER_MOTION_COMMANDED=False'; do
  grep -Fxq "$marker" "$LOG" || { echo "RESULT=STAGE3C33_MISSING_MARKER=$marker" >&2; exit 7; }
done
[[ -s "$REPORT" ]] || { echo 'RESULT=STAGE3C33_REPORT_MISSING'; exit 9; }
echo 'RESULT=STAGE3C33_STATIC_GEOMETRY_RECORDED_NOT_GRASP_VALIDATED'
