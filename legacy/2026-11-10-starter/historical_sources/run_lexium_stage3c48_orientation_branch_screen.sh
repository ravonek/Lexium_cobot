#!/usr/bin/env bash
set -euo pipefail
PROJECT="${PROJECT:-$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place}"
OUT="/tmp/lexium_stage3c48_orientation_branch_report.json"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
NPZ="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
STAGE45="$HOME/Downloads/lexium_stage3c45_offline_pregrasp_report.json"
STAGE47="$HOME/Downloads/lexium_stage3c47_physx_home_mesh_fingerprint.json"
if [[ ! -f "$STAGE45" ]]; then STAGE45=/tmp/lexium_stage3c45_offline_pregrasp_report.json; fi
if [[ ! -f "$STAGE47" ]]; then STAGE47=/tmp/lexium_stage3c47_physx_home_mesh_fingerprint.json; fi
for f in "$PROJECT/assets/lexium_cobot.urdf" "$PROJECT/robot_cfg.py" \
         "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" \
         "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py" \
         "$PROJECT/lexium_stage3c45_offline_pregrasp_lift_screen.py" \
         "$PROJECT/lexium_stage3c46_self_mesh_narrowphase.py" \
         "$STAGE35" "$NPZ" "$STAGE45" "$STAGE47"; do
  if [[ ! -s "$f" ]]; then echo "STAGE48_MISSING_INPUT=$f" >&2; exit 2; fi
done
printf '%s\n' 'STAGE=3C48 MODE=OFFLINE_READ_ONLY_SEARCH' 'ISAAC_LAB_STARTED=False MOTION_COMMANDED=False'
cd "$PROJECT"
python3 "$PROJECT/lexium_stage3c48_orientation_branch_screen.py" \
  --urdf "$PROJECT/assets/lexium_cobot.urdf" \
  --robot-cfg "$PROJECT/robot_cfg.py" \
  --stage35 "$STAGE35" \
  --stage45 "$STAGE45" \
  --stage47 "$STAGE47" \
  --clean-npz "$NPZ" \
  --samples 41 \
  --out "$OUT"
cp -f "$OUT" "$HOME/Downloads/lexium_stage3c48_orientation_branch_report.json"
printf 'STAGE48_DOWNLOADS_REPORT=%s\n' "$HOME/Downloads/lexium_stage3c48_orientation_branch_report.json"
