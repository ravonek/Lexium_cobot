#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
ISAACLAB="$HOME/IsaacLab-3.0-EA"
SCRIPT="$PROJECT/lexium_stage3c44_explicit_home_fk_audit.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
LOG=/tmp/lexium_stage3c44_explicit_home_fk_audit.log
REPORT=/tmp/lexium_stage3c44_explicit_home_fk_report.json
for f in "$SCRIPT" "$URDF" "$PROJECT/robot_cfg.py" "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE44_MISSING_FILE=$f" >&2; exit 3; }
done
[[ -d "$ISAACLAB" ]] || { echo "STAGE44_MISSING_ISAACLAB=$ISAACLAB" >&2; exit 3; }
python3 -m py_compile "$SCRIPT"
echo 'STAGE=3C44 MODE=PHYSX_EXPLICIT_HOME_INITIALIZATION'
echo 'SIMULATION_ONLY=True ONE_PHYSICS_STEP=True CANDIDATE1_REPLAY=False'
echo "LOG=$LOG REPORT=$REPORT"
cd "$ISAACLAB"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --num-envs 1 --visualizer none --urdf "$URDF" --out "$REPORT" \
  >"$LOG" 2>&1
ret=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$ret"
echo '========== STAGE 3C44 =========='
grep -E '^STAGE44_|^LEXIUM_STAGE3C44_' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if ((ret!=0)); then
  tail -n 35 "$LOG"
  exit "$ret"
fi
for marker in 'STAGE44_CANDIDATE1_PHYSX_Q_RECOVERED=False' 'STAGE44_NO_MOTION_TO_CUBE=True' 'STAGE44_NO_GRASP_PERMISSION=True' 'LEXIUM_STAGE3C44_EXPLICIT_HOME_AUDIT_COMPLETE=True'; do
  grep -Fxq "$marker" "$LOG" || { echo "STAGE44_MISSING_MARKER=$marker" >&2; exit 4; }
done
[[ -s "$REPORT" ]] || { echo 'STAGE44_REPORT_MISSING' >&2; exit 4; }
python3 - "$REPORT" <<'PYVERIFY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C44' and r['sim_only'] and r['explicit_default_joint_state_written']
assert not r['candidate1_replayed'] and not r['physical_motion_permission']
assert len(r['arm_q_immediately_after_home_write_rad'])==6
print('STAGE44_REPORT_VERIFIED=True')
PYVERIFY
echo 'RESULT=STAGE3C44_EXPLICIT_HOME_INITIALIZATION_RECORDED_CANDIDATE1_STILL_NOT_VERIFIED'
