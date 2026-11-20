#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c31_gripper_collider_geometry.py"
LOG=/tmp/lexium_stage3c31_gripper_geometry.log
BASE=/tmp/lexium_stage3c30_full_contact.log
for f in "$SCRIPT" "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
    "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
    "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
    /tmp/lexium_stage3c29_preflight.log \
    /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
    /tmp/lexium_stage3c26_candidates.json "$BASE"; do
  [[ -f "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True'; do
  grep -Fxq "$marker" "$BASE" || { echo "BASELINE_3C30_FULL_NOT_VALIDATED=$marker" >&2; exit 4; }
done
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo 'STAGE=3C31 FULL_SCENE_GEOMETRY_CAPTURE_AFTER_VERIFIED_MOTION'
echo 'Same HOME -> candidate1, gravity -9.81, table & cube, robot sensors; fingers NOT moved.'
echo 'Measures collision-mesh vertex geometry after physical HOLD; no grasp or descent.'
echo "LOG=$LOG"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --scene-variant full --table-center-x 0.62 --cube-x 0.30 --cube-y -0.30 \
  --robot-contact-threshold-n 0.5 \
  --warmup-steps 120 --motion-steps 900 --hold-steps 300 \
  --num_envs 1 --visualizer none >"$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== DYNAMICS / CONTACT =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|ROBOT_PEAK_CONTACT_FORCE_N=|CUBE_REST_CONTACT_MAX_N=|FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE=|EXTERNAL_CONTACT_SCREEN_PASSED=)' "$LOG" || true
echo '========== GEOMETRY =========='
grep -E '^(STAGE3C31_GEOMETRY_STARTED=|GEOMETRY_SOURCE=|GEOMETRY_METHOD=|CUBE_ACTUAL_CENTER_XYZ_M=|ACTUAL_TCP_WORLD_XYZ_M=|CUBE_CENTER_RELATIVE_TO_TCP_IN_TOOL_XYZ_M=|GRIPPER_TOOL_LOCAL_Z_AXIS_WORLD=|COLLIDER_WORLD |COLLIDER_CUBE_VERTEX_PROXY |FINGER_GROUP_PROJECTED_COLLIDER_GAP_M=|GRIPPER_FINGER_COLLIDER_COUNT=|GRASP_VALIDATED=|GEOMETRY_REPORT_JSON=|LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if (( status != 0 )); then
  echo 'RESULT=STAGE3C31_FAILED_OR_ABORTED'
  tail -n 45 "$LOG"
  exit "$status"
fi
for marker in 'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
              'FINAL_STATIC_HOLD_AND_TRAJECTORY_ACCEPTABLE=True' \
              'EXTERNAL_CONTACT_SCREEN_PASSED=True' \
              'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True'; do
  if ! grep -Fxq "$marker" "$LOG"; then
    echo "RESULT=STAGE3C31_INCOMPLETE_MISSING_MARKER=$marker"
    tail -n 45 "$LOG"
    exit 6
  fi
done
[[ -s /tmp/lexium_stage3c31_gripper_geometry.json ]] || { echo 'RESULT=GEOMETRY_JSON_NOT_WRITTEN'; exit 7; }
echo 'RESULT=STAGE3C31_GEOMETRY_RECORDED_NOT_GRASP_VALIDATED'
