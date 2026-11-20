#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-audit}"
if [[ "$MODE" != 'audit' && "$MODE" != 'probe' ]]; then
  echo 'USAGE: bash run_lexium_stage3c32b_limit_aware.sh audit|probe' >&2
  exit 2
fi
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c32b_limit_aware_calibration.py"
BASE=/tmp/lexium_stage3c30_full_contact.log
STAGE31_LOG=/tmp/lexium_stage3c31_gripper_geometry.log
LOG="/tmp/lexium_stage3c32b_limit_aware_${MODE}.log"
REPORT="/tmp/lexium_stage3c32b_limit_aware_${MODE}.json"
for f in "$SCRIPT" "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
    "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
    "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
    /tmp/lexium_stage3c29_preflight.log \
    /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
    /tmp/lexium_stage3c26_candidates.json "$BASE" "$STAGE31_LOG" \
    /tmp/lexium_stage3c31_gripper_geometry.json; do
  [[ -f "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$BASE" || { echo "STAGE3C30_NOT_VALIDATED=$marker" >&2; exit 4; }
done
for marker in 'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$STAGE31_LOG" || { echo "STAGE3C31_NOT_VALIDATED=$marker" >&2; exit 5; }
done
if [[ "$MODE" == probe ]]; then
  [[ -s /tmp/lexium_stage3c32b_limit_aware_audit.json ]] || {
    echo 'FIRST_RUN_AUDIT_AND_REVIEW_RESULT_BEFORE_PROBING' >&2; exit 6;
  }
fi
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo "STAGE=3C32B MODE=$MODE"
echo 'Same verified HOME -> Candidate 1, full scene, gravity -9.81 m/s^2, arm hold.'
if [[ "$MODE" == audit ]]; then
  echo 'AUDIT ONLY: finger joints are NOT commanded to move.'
else
  echo 'LIMIT-AWARE MICRO PROBE: one finger joint at a time, 0.004 rad, only allowed directions; recenter after each.'
  echo 'NO descent, NO simultaneous grip, no source USD/URDF/robot_cfg modification.'
fi
echo "LOG=$LOG"
echo "REPORT=$REPORT"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --scene-variant full --table-center-x 0.62 --cube-x 0.30 --cube-y -0.30 \
  --robot-contact-threshold-n 0.5 \
  --warmup-steps 120 --motion-steps 900 --hold-steps 300 \
  --num_envs 1 --visualizer none \
  --calibration-mode "$MODE" --probe-rad 0.004 \
  --probe-ramp-steps 120 --probe-hold-steps 60 \
  --stage32-report "$REPORT" >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== BASELINE / SAFETY =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|EXTERNAL_CONTACT_SCREEN_PASSED=|LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=)' "$LOG" || true
echo '========== STAGE 3C32 =========='
grep -E '^(LEXIUM_STAGE3C32B_|STAGE32B_|STAGE32_MODE=|STAGE32_FINGER_JOINT |STAGE32_FINGER_URDF |STAGE32_ALL_GRIPPER_JOINT_NAMES=|STAGE32_MICRO_PROBE_ELIGIBLE=|STAGE32_START_GEOMETRY=|STAGE32_MICRO_PROBE |STAGE32_PROBE_SKIPPED |STAGE32_REPORT_JSON=|FINGER_MOVEMENT_COMMANDED=|GRASP_VALIDATED=)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if (( status != 0 )); then
  echo 'RESULT=STAGE3C32B_FAILED_OR_SAFETY_ABORTED'
  tail -n 35 "$LOG"
  exit "$status"
fi
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$LOG" || {
    echo "RESULT=STAGE3C32_MISSING_BASELINE_MARKER=$marker" >&2
    exit 7
  }
done
if [[ "$MODE" == audit ]]; then
  grep -Fxq 'LEXIUM_STAGE3C32_JOINT_AUDIT_PASSED=True' "$LOG" || exit 8
  echo 'RESULT=STAGE3C32B_LIMIT_AWARE_AUDIT_PASS_NO_FINGER_MOTION'
else
  grep -Eq '^LEXIUM_STAGE3C32B_LIMIT_AWARE_PROBE_COMPLETE=True ' "$LOG" || exit 8
  echo 'RESULT=STAGE3C32B_LIMIT_AWARE_PROBES_RECORDED_NOT_GRASP_VALIDATED'
fi
[[ -s "$REPORT" ]] || { echo 'RESULT=STAGE3C32_REPORT_MISSING'; exit 9; }
