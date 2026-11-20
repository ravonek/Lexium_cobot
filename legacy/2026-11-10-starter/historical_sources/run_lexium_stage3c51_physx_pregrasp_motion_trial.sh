#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
ISAACLAB="$HOME/IsaacLab-3.0-EA"
SCRIPT="$PROJECT/lexium_stage3c51_physx_pregrasp_motion_trial.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
STAGE49=/tmp/lexium_stage3c49_convex_hull_proxy_report.json
[[ -s "$STAGE49" ]] || STAGE49="$HOME/Downloads/lexium_stage3c49_convex_hull_proxy_report.json"
STAGE50=/tmp/lexium_stage3c50_physx_contact_preflight.json
[[ -s "$STAGE50" ]] || STAGE50="$HOME/Downloads/lexium_stage3c50_physx_contact_preflight.json"
REPORT=/tmp/lexium_stage3c51_physx_pregrasp_motion_trial.json
LOG=/tmp/lexium_stage3c51_physx_pregrasp_motion_trial.log
for f in "$SCRIPT" "$URDF" "$STAGE49" "$STAGE50" \
    "$PROJECT/robot_cfg.py" "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
    "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE51_MISSING_FILE=$f" >&2; exit 3; }
done
[[ -d "$ISAACLAB" ]] || { echo "STAGE51_MISSING_ISAACLAB=$ISAACLAB" >&2; exit 3; }
python3 -m py_compile "$SCRIPT"
python3 - "$STAGE49" "$STAGE50" <<'PYGATE'
import json,sys,numpy as np
s49=json.load(open(sys.argv[1]));s50=json.load(open(sys.argv[2]))
assert s49['stage']=='3C49' and s50['stage']=='3C50'
assert s50['home_contact_positive_control_passed'] is True
assert s49['frames_with_new_hull_self_pairs_vs_home']==0
assert s49['frames_with_hull_cube_overlap']==0 and s49['frames_with_hull_table_overlap']==0
assert np.allclose(s49['target_q_rad'],s50['stage49_path_target_q_rad'],atol=1e-8)
print('STAGE51_OFFLINE_AND_HOME_PREFLIGHT_GATE=True')
PYGATE
printf 'STAGE=3C51 MODE=PHYSX_SIM_ONLY_PREGRASP_MOTION\n'
printf 'NO_HARDWARE=True NO_GRASP=True SELF_COLLISIONS_REMAIN_DISABLED=True\n'
printf 'LOG=%s REPORT=%s\n' "$LOG" "$REPORT"
cd "$ISAACLAB"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
    --num-envs 1 --visualizer none --warmup-steps 150 --motion-steps 1200 \
    --hold-steps 180 --contact-threshold-n .5 \
    --urdf "$URDF" --stage49 "$STAGE49" --stage50 "$STAGE50" --out "$REPORT" > "$LOG" 2>&1
status=$?
set -e
printf 'ISAAC_LAB_EXIT_CODE=%s\n' "$status"
printf '========== STAGE 3C51 ==========\n'
grep -E '^STAGE51_|^LEXIUM_STAGE3C51_|^RESULT=' "$LOG" || true
printf '========== DIAGNOSTIC ERRORS ==========\n'
grep -Ei 'SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:' "$LOG" | tail -n 25 || true
if [[ -s "$REPORT" ]]; then
  cp "$REPORT" "$HOME/Downloads/lexium_stage3c51_physx_pregrasp_motion_trial.json"
  echo "STAGE51_DOWNLOADS_REPORT=$HOME/Downloads/lexium_stage3c51_physx_pregrasp_motion_trial.json"
fi
if ((status!=0)); then
  tail -n 45 "$LOG"
  echo 'RESULT=STAGE3C51_ABORTED_OR_FAILED_REPORT_AVAILABLE_IF_WRITTEN'
  exit "$status"
fi
for marker in 'STAGE51_SIM_MOTION_PASS=True' 'STAGE51_GRASP_EXECUTED=False' \
              'LEXIUM_STAGE3C51_PHYSX_PREGRASP_MOTION_TRIAL_COMPLETE=True'; do
    grep -Fxq "$marker" "$LOG" || { echo "STAGE51_MISSING_MARKER=$marker" >&2; exit 4; }
done
python3 - "$REPORT" <<'PYVERIFY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C51' and r['sim_only'] is True
assert r['simulation_motion_passed'] is True and r['trajectory_fully_executed'] is True
assert r['physx_self_collisions_enabled'] is False
assert r['safety_approval_for_hardware'] is False and r['grasp_approved'] is False
assert r['grasp_attempted'] is False and r['fingers_commanded'] is False
print('STAGE51_REPORT_VERIFIED=True')
PYVERIFY
echo 'RESULT=STAGE3C51_SIM_PREGRASP_PASSED_WITH_SELF_COLLISIONS_UNRESOLVED'
