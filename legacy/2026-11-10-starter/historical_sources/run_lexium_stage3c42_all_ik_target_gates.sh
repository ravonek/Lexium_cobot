#!/usr/bin/env bash
# Read-only screening of all Stage40 IK target poses, not a robot controller.
set -euo pipefail
PROJECT="${LEXIUM_PROJECT:-$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place}"
OUT="${STAGE42_REPORT_OUT:-/tmp/lexium_stage3c42_all_ik_target_gates.json}"
R40="${STAGE42_STAGE40_REPORT:-/tmp/lexium_stage3c40_ik_reachability_report.json}"
if [[ ! -s "$R40" && -s "$HOME/Downloads/lexium_stage3c40_ik_reachability_report.json" ]]; then
  R40="$HOME/Downloads/lexium_stage3c40_ik_reachability_report.json"
fi
SCRIPT="$PROJECT/lexium_stage3c42_all_ik_target_gates.py"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
MESH="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
for p in "$SCRIPT" "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py" "$PROJECT/assets/lexium_cobot.urdf" "$PROJECT/robot_cfg.py" "$STAGE35" "$MESH" "$R40"; do
  [[ -s "$p" ]] || { echo "STAGE42_MISSING=$p" >&2; exit 2; }
done
for m in base_link.stl link1_1.stl link2_1.stl link3_1.stl link4_1.stl LINK5_1.stl; do
  [[ -s "$PROJECT/assets/meshes/$m" ]] || { echo "STAGE42_MISSING_MESH=$m" >&2; exit 3; }
done
python3 -m py_compile "$SCRIPT" "$PROJECT/lexium_stage3c40_ik_reachability_audit.py" "$PROJECT/lexium_stage3c41_full_arm_collision_audit.py"
echo 'STAGE=3C42 MODE=OFFLINE_STATIC_TARGET_GATES'
echo "STAGE42_STAGE40_SOURCE=$R40"
echo 'ISAAC_LAB_STARTED=False MOTION_COMMANDED=False ASSETS_MODIFIED=False'
ARGS=(--urdf "$PROJECT/assets/lexium_cobot.urdf" --robot-cfg "$PROJECT/robot_cfg.py" --stage35 "$STAGE35" --stage40 "$R40" --clean-gripper-npz "$MESH" --out "$OUT")
if python3 -c 'import numpy, scipy' >/dev/null 2>&1; then
  python3 "$SCRIPT" "${ARGS[@]}"
else
  ISAACLAB="${ISAACLAB_ROOT:-$HOME/IsaacLab-3.0-EA}"
  [[ -d "$ISAACLAB" ]] || { echo 'STAGE42_MISSING_SCIPY_AND_ISAACLAB_ENV' >&2; exit 4; }
  (cd "$ISAACLAB" && uv run --no-sync python "$SCRIPT" "${ARGS[@]}")
fi
[[ -s "$OUT" ]] || { echo STAGE42_REPORT_NOT_WRITTEN >&2; exit 5; }
echo "STAGE42_JSON_READY=$OUT"
