#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
SCRIPT="$PROJECT/lexium_stage3c45_offline_pregrasp_lift_screen.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
CFG="$PROJECT/robot_cfg.py"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
STAGE40=/tmp/lexium_stage3c40_ik_reachability_report.json
STAGE44=/tmp/lexium_stage3c44_explicit_home_fk_report.json
CLEAN="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
REPORT=/tmp/lexium_stage3c45_offline_pregrasp_report.json
for f in "$SCRIPT" "$URDF" "$CFG" "$STAGE35" "$STAGE40" "$STAGE44" "$CLEAN" \
  "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE45_MISSING=$f" >&2; exit 3; }
done
python3 -m py_compile "$SCRIPT"
echo 'STAGE=3C45 MODE=OFFLINE_READ_ONLY_PREGRASP_SEARCH'
echo 'ISAAC_LAB_STARTED=False PHYSICAL_MOTION_COMMANDED=False'
python3 "$SCRIPT" \
  --urdf "$URDF" --robot-cfg "$CFG" \
  --stage35 "$STAGE35" --stage40 "$STAGE40" --stage44 "$STAGE44" \
  --clean-npz "$CLEAN" \
  --lift-mm '60,70,80,90,100,120' --samples 41 \
  --out "$REPORT"
[[ -s "$REPORT" ]] || { echo 'STAGE45_REPORT_MISSING' >&2; exit 4; }
python3 - "$REPORT" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C45' and r['stage44_home_fk_verified']
assert r['no_isaac_lab'] and r['no_physical_motion']
assert not r['physical_motion_permission'] and not r['no_grasp_permission'] is False
assert len(r['target_rows'])==6
print('STAGE45_REPORT_VERIFIED=True')
PY
echo 'RESULT=STAGE3C45_OFFLINE_PREGRASP_GEOMETRY_RECORDED_NOT_PHYSX_VALIDATED'
