#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
SCRIPT="$PROJECT/lexium_stage3c38_offline_pose_screen.py"
NPZ="${STAGE37_NPZ:-/tmp/lexium_stage3c37_full_collision_triangles.npz}"
REPORT37="${STAGE37_REPORT:-/tmp/lexium_stage3c37_table_gate.json}"
REPORT36="${STAGE36_REPORT:-/tmp/lexium_stage3c36_local_aperture_report.json}"
OUT="/tmp/lexium_stage3c38_offline_pose_screen.json"
CLEAN="/tmp/lexium_stage3c38_clean_collision_meshes.npz"
for p in "$SCRIPT" "$NPZ" "$REPORT37" "$REPORT36"; do
 [[ -s "$p" ]] || { echo "MISSING_REQUIRED_FILE=$p" >&2; exit 2; }
done
python3 -m py_compile "$SCRIPT"
echo "STAGE=3C38 MODE=OFFLINE_READ_ONLY_WITHOUT_ISAAC_LAB"
# Use uv's IsaacLab environment only if python3 has no numpy.
if python3 -c 'import numpy' >/dev/null 2>&1; then
  python3 "$SCRIPT" --npz "$NPZ" --gate-json "$REPORT37" --stage36-json "$REPORT36" --report "$OUT" --clean-npz "$CLEAN"
else
  cd "$HOME/IsaacLab-3.0-EA"
  uv run --no-sync python "$SCRIPT" --npz "$NPZ" --gate-json "$REPORT37" --stage36-json "$REPORT36" --report "$OUT" --clean-npz "$CLEAN"
fi
echo "STAGE38_DONE_REPORT=$OUT"
