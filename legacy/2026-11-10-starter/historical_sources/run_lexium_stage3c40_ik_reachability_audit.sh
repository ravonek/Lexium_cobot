#!/usr/bin/env bash
# Lexium Stage3C40: OFFLINE only. Reads robot_cfg.py/URDF; never loads Isaac Lab or writes robot assets.
set -euo pipefail
PROJECT="${LEXIUM_PROJECT:-$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUTS="$HERE/stage40_inputs"
OUT="${STAGE40_REPORT_OUT:-/tmp/lexium_stage3c40_ik_reachability_report.json}"
LOG_CANDIDATES=(
  /tmp/lexium_stage3c37_mesh_export_table_gate.log
  /tmp/lexium_stage3c35_bottleneck_axis_audit.log
  /tmp/lexium_stage3c36_local_aperture_audit.log
  /tmp/lexium_stage3c33_static_gripper_geometry.log
)
LOG="${STAGE40_BASELINE_LOG:-}"
if [[ -z "$LOG" ]]; then
  for x in "${LOG_CANDIDATES[@]}"; do
    if [[ -s "$x" ]] && grep -q 'FINAL_ARM_Q_RAD=' "$x"; then LOG="$x"; break; fi
  done
fi
STAGE39="${STAGE40_STAGE39_REPORT:-$INPUTS/lexium_stage3c39_offline_example_report.json}"
MESH="${STAGE40_CLEAN_MESH:-$INPUTS/lexium_stage3c38_clean_collision_meshes.npz}"
ARGS=(
 --urdf "$PROJECT/assets/lexium_cobot.urdf"
 --robot-cfg "$PROJECT/robot_cfg.py"
 --stage35 "$INPUTS/lexium_stage3c35_bottleneck_axis_audit.json"
 --stage36 "$INPUTS/lexium_stage3c36_local_aperture_report.json"
 --stage39 "$STAGE39"
 --mesh "$MESH"
 --out "$OUT"
)
if [[ -n "$LOG" ]]; then ARGS+=(--baseline-log "$LOG"); fi
for f in "$HERE/lexium_stage3c40_ik_reachability_audit.py" \
         "$PROJECT/assets/lexium_cobot.urdf" "$PROJECT/robot_cfg.py" \
         "$INPUTS/lexium_stage3c35_bottleneck_axis_audit.json" \
         "$INPUTS/lexium_stage3c36_local_aperture_report.json" "$STAGE39" "$MESH"; do
    [[ -s "$f" ]] || { echo "STAGE40_MISSING_REQUIRED_FILE=$f" >&2; exit 2; }
done
python3 -m py_compile "$HERE/lexium_stage3c40_ik_reachability_audit.py"
echo "STAGE=3C40 MODE=OFFLINE_READ_ONLY_URDF_IK"
echo "ISAAC_LAB_LAUNCHED=False PHYSICAL_MOTION=False USD_URDF_MODIFIED=False"
echo "BASELINE_PHYSX_LOG=${LOG:-NONE_URDF_BASELINE_INFERRED_ONLY}"
echo "STAGE39_SOURCE=$STAGE39"
echo "OUTPUT_JSON=$OUT"
if python3 -c 'import numpy, scipy' >/dev/null 2>&1; then
    python3 "$HERE/lexium_stage3c40_ik_reachability_audit.py" "${ARGS[@]}"
else
    ISAACLAB="${ISAACLAB_ROOT:-$HOME/IsaacLab-3.0-EA}"
    if [[ ! -d "$ISAACLAB" ]]; then
      echo "STAGE40_MISSING_SCIPY_AND_ISAACLAB_ENV: install scipy in a venv or use existing IsaacLab environment" >&2
      exit 3
    fi
    (cd "$ISAACLAB" && uv run --no-sync python "$HERE/lexium_stage3c40_ik_reachability_audit.py" "${ARGS[@]}")
fi
[[ -s "$OUT" ]] || { echo "STAGE40_REPORT_NOT_WRITTEN" >&2; exit 4; }
echo "STAGE40_JSON_READY=$OUT"
