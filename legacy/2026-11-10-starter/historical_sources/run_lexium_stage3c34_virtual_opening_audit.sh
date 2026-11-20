#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c34_virtual_opening_audit.py"
PREV32=/tmp/lexium_stage3c32b_limit_aware_probe.json
PREV33=/tmp/lexium_stage3c33_static_gripper_geometry.json
LOG=/tmp/lexium_stage3c34_virtual_opening_audit.log
REPORT=/tmp/lexium_stage3c34_virtual_opening_audit.json
for f in "$SCRIPT" "$PREV32" "$PREV33" \
  "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
  "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
  "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
  /tmp/lexium_stage3c29_preflight.log \
  /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
  /tmp/lexium_stage3c26_candidates.json; do
  [[ -s "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
python3 - "$PREV32" "$PREV33" <<'PY'
import json,sys
p32=json.load(open(sys.argv[1],encoding='utf8'))
p33=json.load(open(sys.argv[2],encoding='utf8'))
assert p32.get('stage')=='3C32B' and p32.get('probe_complete') is True
assert p32.get('actual_passes')==7 and p32.get('skipped_limit')==1
assert p33.get('stage')=='3C33' and p33.get('no_finger_motion') is True
assert p33.get('grasp_40mm_validated') is False
assert p33.get('self_collisions_enabled') is False
print('PRIOR_STAGE32B_STAGE33_REPORTS_VERIFIED=True')
PY
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo 'STAGE=3C34 MODE=VIRTUAL_OPENING_KINEMATICS_STATIC'
echo 'Replays verified HOME -> Candidate 1, then forecasts hinge geometry numerically.'
echo 'NO finger commands, NO opening or descent, NO USD/URDF/robot_cfg modification.'
echo "LOG=$LOG"
echo "REPORT=$REPORT"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --scene-variant full --table-center-x 0.62 --cube-x 0.30 --cube-y -0.30 \
  --robot-contact-threshold-n 0.5 \
  --warmup-steps 120 --motion-steps 900 --hold-steps 300 \
  --num_envs 1 --visualizer none \
  --stage33-report "$PREV33" \
  --stage32b-report "$PREV32" \
  --stage34-report "$REPORT" >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== PHYSICAL BASELINE =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|EXTERNAL_CONTACT_SCREEN_PASSED=|LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=)' "$LOG" || true
echo '========== VIRTUAL OPENING AUDIT =========='
grep -E '^(LEXIUM_STAGE3C34_|STAGE34_|GRASP_VALIDATED=)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 14 || true
if ((status!=0)); then
  echo 'RESULT=STAGE3C34_FAILED'
  tail -n 35 "$LOG"
  exit "$status"
fi
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True' \
              'LEXIUM_STAGE3C33_STATIC_GEOMETRY_AUDIT_COMPLETE=True' \
              'LEXIUM_STAGE3C34_STATIC_VIRTUAL_AUDIT_COMPLETE=True' \
              'STAGE34_PHYSICAL_FINGER_MOTION_COMMANDED=False' \
              'STAGE34_ARM_DESCENT_COMMANDED=False'; do
   grep -Fxq "$marker" "$LOG" || { echo "RESULT=STAGE3C34_MISSING_MARKER=$marker" >&2; exit 7; }
done
python3 - "$REPORT" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r.get('stage')=='3C34'
assert r.get('no_finger_motion') is True
assert r.get('grasp_validated') is False
assert r.get('opening_40mm_validated') is False
assert r.get('virtual_combination_grid_count',0)>0
print('STAGE3C34_JSON_VERIFIED=True')
PY
echo 'RESULT=STAGE3C34_VIRTUAL_SENSITIVITY_RECORDED_NOT_GRASP_VALIDATED'
