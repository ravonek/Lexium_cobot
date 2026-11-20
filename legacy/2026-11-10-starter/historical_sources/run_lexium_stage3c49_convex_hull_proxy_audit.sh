#!/usr/bin/env bash
set -euo pipefail
PROJECT="${PROJECT:-$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place}"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
NPZ="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
STAGE45="$HOME/Downloads/lexium_stage3c45_offline_pregrasp_report.json"
STAGE47="$HOME/Downloads/lexium_stage3c47_physx_home_mesh_fingerprint.json"
STAGE48="$HOME/Downloads/lexium_stage3c48_orientation_branch_report.json"
[[ -s "$STAGE45" ]] || STAGE45=/tmp/lexium_stage3c45_offline_pregrasp_report.json
[[ -s "$STAGE47" ]] || STAGE47=/tmp/lexium_stage3c47_physx_home_mesh_fingerprint.json
[[ -s "$STAGE48" ]] || STAGE48=/tmp/lexium_stage3c48_orientation_branch_report.json
OUT=/tmp/lexium_stage3c49_convex_hull_proxy_report.json
for f in "$PROJECT/assets/lexium_cobot.urdf" "$PROJECT/robot_cfg.py" \
  "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" \
  "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py" \
  "$PROJECT/lexium_stage3c49_convex_hull_proxy_audit.py" \
  "$STAGE35" "$NPZ" "$STAGE45" "$STAGE47" "$STAGE48"; do
  [[ -s "$f" ]] || { echo "STAGE49_MISSING=$f" >&2; exit 3; }
done
cd "$PROJECT"
python3 "$PROJECT/lexium_stage3c49_convex_hull_proxy_audit.py" \
  --urdf "$PROJECT/assets/lexium_cobot.urdf" \
  --robot-cfg "$PROJECT/robot_cfg.py" \
  --stage35 "$STAGE35" \
  --stage45 "$STAGE45" \
  --stage47 "$STAGE47" \
  --stage48 "$STAGE48" \
  --clean-npz "$NPZ" --samples 41 --out "$OUT"
cp "$OUT" "$HOME/Downloads/lexium_stage3c49_convex_hull_proxy_report.json"
echo "STAGE49_DOWNLOADS_REPORT=$HOME/Downloads/lexium_stage3c49_convex_hull_proxy_report.json"
