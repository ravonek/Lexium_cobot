#!/usr/bin/env bash
# Stage3C43: Isaac Lab starts with the configured HOME joint state only.
# No arm/finger trajectory is commanded. No Stage26 candidate is needed.
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
ISAACLAB="$HOME/IsaacLab-3.0-EA"
SCRIPT="$PROJECT/lexium_stage3c43_physx_home_fk_audit.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
LOG=/tmp/lexium_stage3c43_physx_home_fk_audit.log
REPORT=/tmp/lexium_stage3c43_physx_home_fk_report.json
for f in "$SCRIPT" "$URDF" "$PROJECT/robot_cfg.py" "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE43_MISSING_FILE=$f" >&2; exit 3; }
done
[[ -d "$ISAACLAB" ]] || { echo "STAGE43_MISSING_ISAACLAB=$ISAACLAB" >&2; exit 3; }
python3 -m py_compile "$SCRIPT"
echo 'STAGE=3C43 MODE=PHYSX_HOME_ONLY_FK_CALIBRATION'
echo 'SIMULATION_ONLY=True CANDIDATE1_REPLAY=False MOTION_COMMANDED=False'
echo "LOG=$LOG REPORT=$REPORT"
cd "$ISAACLAB"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --num-envs 1 --visualizer none --urdf "$URDF" --out "$REPORT" \
  >"$LOG" 2>&1
ret=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$ret"
echo '========== STAGE 3C43 =========='
grep -E '^STAGE43_|^LEXIUM_STAGE3C43_' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if ((ret!=0)); then
  tail -n 35 "$LOG"
  exit "$ret"
fi
for marker in 'STAGE43_CANDIDATE1_PHYSX_Q_RECOVERED=False' 'STAGE43_NO_MOTION_PERMISSION=True' 'LEXIUM_STAGE3C43_HOME_FK_AUDIT_COMPLETE=True'; do
  grep -Fxq "$marker" "$LOG" || { echo "STAGE43_MISSING_MARKER=$marker" >&2; exit 4; }
done
[[ -s "$REPORT" ]] || { echo 'STAGE43_REPORT_MISSING' >&2; exit 4; }
python3 - "$REPORT" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C43'
assert r['sim_only'] and not r['arm_trajectory_commanded']
assert not r['candidate1_replayed'] and not r['physical_motion_permission']
assert len(r['arm_q_initial_physx_rad'])==6
print('STAGE43_REPORT_VALIDATED=True')
PY
echo 'RESULT=STAGE3C43_HOME_FK_RECORDED_CANDIDATE1_PHYSX_Q_STILL_NOT_INDEPENDENTLY_VERIFIED'
