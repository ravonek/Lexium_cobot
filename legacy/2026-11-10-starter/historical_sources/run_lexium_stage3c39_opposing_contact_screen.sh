#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
SCRIPT="$PROJECT/lexium_stage3c39_opposing_contact_screen.py"
NPZ="${STAGE38_CLEAN_NPZ:-/tmp/lexium_stage3c38_clean_collision_meshes.npz}"
REPORT38="${STAGE38_REPORT:-/tmp/lexium_stage3c38_offline_pose_screen.json}"
REPORT36="${STAGE36_REPORT:-/tmp/lexium_stage3c36_local_aperture_report.json}"
OUT="${STAGE39_REPORT:-/tmp/lexium_stage3c39_opposing_contact_screen.json}"
for p in "$SCRIPT" "$NPZ" "$REPORT38" "$REPORT36"; do
  [[ -s "$p" ]] || { echo "MISSING_REQUIRED_FILE=$p" >&2; exit 2; }
done
python3 -m py_compile "$SCRIPT"
echo "STAGE=3C39 OFFLINE STATIC SCREEN ONLY; NO ISAAC LAB / NO ROBOT MOVEMENT"
python3 "$SCRIPT" --npz "$NPZ" --stage38-json "$REPORT38" --stage36-json "$REPORT36" --out "$OUT"
