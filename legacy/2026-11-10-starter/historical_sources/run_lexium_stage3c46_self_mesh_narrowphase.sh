#!/usr/bin/env bash
set -euo pipefail

PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
SCRIPT="$PROJECT/lexium_stage3c46_self_mesh_narrowphase.py"
URDF="$PROJECT/assets/lexium_cobot.urdf"
CFG="$PROJECT/robot_cfg.py"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
CLEAN="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
STAGE45="/tmp/lexium_stage3c45_offline_pregrasp_report.json"
OUT="/tmp/lexium_stage3c46_self_mesh_narrowphase_report.json"

if [[ ! -s "$STAGE45" && -s "$HOME/Downloads/lexium_stage3c45_offline_pregrasp_report.json" ]]; then
  STAGE45="$HOME/Downloads/lexium_stage3c45_offline_pregrasp_report.json"
fi
for f in "$SCRIPT" "$URDF" "$CFG" "$STAGE35" "$STAGE45" "$CLEAN" \
  "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" \
  "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py"; do
  [[ -s "$f" ]] || { echo "STAGE46_MISSING_REQUIRED_FILE=$f" >&2; exit 3; }
done
python3 -m py_compile "$SCRIPT"
bash -n "$0"
echo 'STAGE=3C46 MODE=OFFLINE_AUTHORED_MESH_SELF_NARROWPHASE'
echo 'ISAAC_LAB_STARTED=False ROBOT_MOTION_COMMANDED=False ASSETS_MODIFIED=False'
echo "STAGE46_STAGE45_SOURCE=$STAGE45"
python3 "$SCRIPT" \
  --urdf "$URDF" --robot-cfg "$CFG" \
  --stage35 "$STAGE35" --stage45 "$STAGE45" \
  --clean-npz "$CLEAN" \
  --lift-mm 100 --samples 41 \
  --out "$OUT"
python3 - "$OUT" <<'PY'
import json,sys
p=sys.argv[1]
r=json.load(open(p,encoding='utf-8'))
assert r['stage']=='3C46'
assert r['lift_mm']==100.0 and r['samples']==41
assert r['no_isaac_lab'] and r['no_physical_motion']
assert not r['robot_motion_permission'] and not r['grasp_permission']
assert len(r['frame_results'])==41
assert r['authored_surface_intersection_frames']+r['node_budget_unresolved_frames']>=0
print('STAGE46_REPORT_VALIDATED=True')
PY
printf 'STAGE46_JSON_READY=%s\n' "$OUT"
echo 'RESULT=STAGE3C46_OFFLINE_AUTHORED_SELF_SURFACE_INTERSECTIONS_RECORDED_NO_MOTION_PERMISSION'
