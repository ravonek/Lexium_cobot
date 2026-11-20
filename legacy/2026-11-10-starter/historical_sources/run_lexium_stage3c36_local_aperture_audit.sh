#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c36_local_aperture_audit.py"
PREV32=/tmp/lexium_stage3c32b_limit_aware_probe.json
PREV33=/tmp/lexium_stage3c33_static_gripper_geometry.json
PREV34=/tmp/lexium_stage3c34_virtual_opening_audit.json
PREV35=/tmp/lexium_stage3c35_bottleneck_axis_audit.json
LOG=/tmp/lexium_stage3c36_local_aperture_audit.log
REPORT=/tmp/lexium_stage3c36_local_aperture_report.json
for f in "$SCRIPT" "$PREV32" "$PREV33" "$PREV34" "$PREV35" \
  "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
  "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
  "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
  /tmp/lexium_stage3c29_preflight.log \
  /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
  /tmp/lexium_stage3c26_candidates.json; do
  [[ -s "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
python3 - "$PREV32" "$PREV33" "$PREV34" "$PREV35" <<'PY'
import json,sys
p32=json.load(open(sys.argv[1],encoding='utf8'))
p33=json.load(open(sys.argv[2],encoding='utf8'))
p34=json.load(open(sys.argv[3],encoding='utf8'))
p35=json.load(open(sys.argv[4],encoding='utf8'))
assert p32.get('stage')=='3C32B' and p32.get('probe_complete') is True
assert p32.get('actual_passes')==7 and p32.get('skipped_limit')==1
assert p33.get('stage')=='3C33' and p33.get('no_finger_motion') is True
assert p33.get('grasp_40mm_validated') is False
assert p33.get('self_collisions_enabled') is False
assert p34.get('stage')=='3C34' and p34.get('no_finger_motion') is True
assert p34.get('opening_40mm_validated') is False
assert p35.get('stage')=='3C35' and p35.get('no_finger_motion') is True
assert p35.get('grasp_validated') is False and p35.get('opening_40mm_validated') is False
print('PRIOR_STAGE32B_STAGE33_STAGE34_STAGE35_REPORTS_VERIFIED=True')
PY
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo 'STAGE=3C36 MODE=STATIC_LOCAL_APERTURE_AUDIT'
echo 'Replays verified HOME -> Candidate 1; measures hinge-normal matched local cross-sections and candidate centers.'
echo 'NO finger commands, NO descent, NO USD/URDF/robot_cfg modification.'
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
  --stage34-report "$PREV34" --stage35-report "$PREV35" --stage36-report "$REPORT" >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== PHYSICAL BASELINE =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|EXTERNAL_CONTACT_SCREEN_PASSED=|LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=)' "$LOG" || true
echo '========== STAGE 3C36 LOCAL APERTURE AUDIT =========='
grep -E '^(LEXIUM_STAGE3C36_|STAGE36_|GRASP_VALIDATED=)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 14 || true
if ((status!=0)); then
  echo 'RESULT=STAGE3C36_FAILED'
  tail -n 35 "$LOG"
  exit "$status"
fi
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True' \
              'LEXIUM_STAGE3C33_STATIC_GEOMETRY_AUDIT_COMPLETE=True' \
              'LEXIUM_STAGE3C34_STATIC_VIRTUAL_AUDIT_COMPLETE=True' \
              'STAGE34_PHYSICAL_FINGER_MOTION_COMMANDED=False' \
              'STAGE34_ARM_DESCENT_COMMANDED=False' \
              'LEXIUM_STAGE3C35_BOTTLENECK_AXIS_AUDIT_COMPLETE=True' \
              'STAGE35_PHYSICAL_FINGER_MOTION_COMMANDED=False' \
              'STAGE35_ARM_DESCENT_COMMANDED=False' \
              'LEXIUM_STAGE3C36_LOCAL_APERTURE_AUDIT_COMPLETE=True' \
              'STAGE36_PHYSICAL_FINGER_MOTION_COMMANDED=False' \
              'STAGE36_ARM_DESCENT_COMMANDED=False' \
              'STAGE36_NO_GRASP_PERMISSION=True'; do
   grep -Fxq "$marker" "$LOG" || { echo "RESULT=STAGE3C36_MISSING_MARKER=$marker" >&2; exit 7; }
done
python3 - "$REPORT" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r.get('stage')=='3C36'
assert r.get('no_finger_motion') is True
assert r.get('grasp_validated') is False
assert r.get('opening_40mm_validated') is False
assert r.get('no_arm_descent') is True
assert r.get('approach_trajectory_validated') is False
assert 'candidate_aperture_centers_top_10' in r
assert 'cube_width_best_yaw_along_u' in r
print('STAGE3C36_JSON_VERIFIED=True')
PY
echo 'RESULT=STAGE3C36_LOCAL_APERTURE_RECORDED_NOT_GRASP_VALIDATED'
