#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
ISAACLAB="$HOME/IsaacLab-3.0-EA"
SCRIPT="$PROJECT/lexium_stage3c50_physx_contact_preflight.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
STAGE49=/tmp/lexium_stage3c49_convex_hull_proxy_report.json
[[ -s "$STAGE49" ]] || STAGE49="$HOME/Downloads/lexium_stage3c49_convex_hull_proxy_report.json"
LOG=/tmp/lexium_stage3c50_physx_contact_preflight.log
REPORT=/tmp/lexium_stage3c50_physx_contact_preflight.json
for f in "$SCRIPT" "$URDF" "$STAGE49" "$PROJECT/robot_cfg.py" \
  "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
  "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE50_MISSING_FILE=$f" >&2; exit 3; }
done
[[ -d "$ISAACLAB" ]] || { echo "STAGE50_MISSING_ISAACLAB=$ISAACLAB" >&2; exit 3; }
python3 -m py_compile "$SCRIPT"
echo 'STAGE=3C50 MODE=PHYSX_FULL_SCENE_HOME_SENSOR_PREFLIGHT'
echo 'SIM_ONLY=True HOME_ONLY=True PREGRASP_MOTION=False'
echo "LOG=$LOG REPORT=$REPORT"
cd "$ISAACLAB"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --num-envs 1 --visualizer none --warmup-steps 150 \
  --urdf "$URDF" --stage49 "$STAGE49" --out "$REPORT" > "$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== STAGE 3C50 =========='
grep -E '^STAGE50_|^LEXIUM_STAGE3C50_|^RESULT=' "$LOG" || true
echo '========== DIAGNOSTIC ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 20 || true
if ((status!=0)); then
  tail -n 45 "$LOG"
  echo 'RESULT=STAGE3C50_FAILED_NO_MOTION'
  exit "$status"
fi
for marker in 'STAGE50_HOME_SENSOR_BASELINE_PASS=True' \
              'STAGE50_PREGRASP_MOTION_EXECUTED=False' \
              'LEXIUM_STAGE3C50_HOME_CONTACT_PREFLIGHT_COMPLETE=True'; do
  grep -Fxq "$marker" "$LOG" || { echo "STAGE50_MISSING_MARKER=$marker" >&2; exit 4; }
done
[[ -s "$REPORT" ]] || { echo 'STAGE50_REPORT_MISSING' >&2; exit 4; }
python3 - "$REPORT" <<'PYVERIFY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C50' and r['home_only'] and r['sim_only']
assert r['home_contact_positive_control_passed'] is True
assert r['pregrasp_motion_executed'] is False
assert r['robot_motion_permission'] is False and r['grasp_permission'] is False
assert r['cube_table_contact_positive_control_n'] >= .2
print('STAGE50_REPORT_VERIFIED=True')
PYVERIFY
cp "$REPORT" "$HOME/Downloads/lexium_stage3c50_physx_contact_preflight.json"
echo "STAGE50_DOWNLOADS_REPORT=$HOME/Downloads/lexium_stage3c50_physx_contact_preflight.json"
echo 'RESULT=STAGE3C50_HOME_CONTACT_PREFLIGHT_PASSED_NO_PREGRASP_MOTION'
