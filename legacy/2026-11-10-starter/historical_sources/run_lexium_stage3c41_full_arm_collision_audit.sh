#!/usr/bin/env bash
# Stage3C41: OFFLINE only. No Isaac Lab startup or commands to real/virtual robot.
set -euo pipefail
PROJECT="${LEXIUM_PROJECT:-$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place}"
REPORT="${STAGE41_REPORT_OUT:-/tmp/lexium_stage3c41_full_arm_collision_report.json}"
CANDIDATE="${STAGE41_CANDIDATE:-5}"
SAMPLES="${STAGE41_SAMPLES:-41}"
STAGE35="$PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json"
STAGE40="${STAGE41_STAGE40_REPORT:-/tmp/lexium_stage3c40_ik_reachability_report.json}"
MESH="$PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz"
SCRIPT="$PROJECT/lexium_stage3c41_full_arm_collision_audit.py"
STAGE40_SCRIPT="$PROJECT/lexium_stage3c40_ik_reachability_audit.py"
if [[ ! -s "$STAGE40" && -s "$HOME/Downloads/lexium_stage3c40_ik_reachability_report.json" ]]; then
  STAGE40="$HOME/Downloads/lexium_stage3c40_ik_reachability_report.json"
fi
for f in "$PROJECT/assets/lexium_cobot.urdf" "$PROJECT/robot_cfg.py" "$STAGE35" "$STAGE40" "$MESH" "$SCRIPT" "$STAGE40_SCRIPT"; do
  if [[ ! -s "$f" ]]; then echo "STAGE41_MISSING_REQUIRED_FILE=$f" >&2; exit 2; fi
done
for m in base_link.stl link1_1.stl link2_1.stl link3_1.stl link4_1.stl LINK5_1.stl; do
  if [[ ! -s "$PROJECT/assets/meshes/$m" ]]; then echo "STAGE41_MISSING_ARM_STL=$PROJECT/assets/meshes/$m" >&2; exit 2; fi
done
python3 -m py_compile "$SCRIPT" "$STAGE40_SCRIPT"
echo 'STAGE=3C41 MODE=OFFLINE_FULL_ARM_STL_DISCRETE_SCREEN'
echo 'ISAAC_LAB_STARTED=False ROBOT_MOTION_COMMANDED=False ORIGINAL_ASSETS_MODIFIED=False'
echo 'WARNING=STAGE40_BASELINE_Q_INFERRED_NOT_INDEPENDENTLY_PHYSX_VALIDATED'
echo "STAGE41_SOURCE_REPORT=$STAGE40"
echo "STAGE41_OUTPUT_JSON=$REPORT"
echo "STAGE41_CANDIDATE=$CANDIDATE STAGE41_SAMPLES=$SAMPLES"
if python3 -c 'import numpy, scipy' >/dev/null 2>&1; then
  python3 "$SCRIPT" --urdf "$PROJECT/assets/lexium_cobot.urdf" \
    --robot-cfg "$PROJECT/robot_cfg.py" --stage35 "$STAGE35" \
    --stage40 "$STAGE40" --clean-gripper-npz "$MESH" \
    --candidate "$CANDIDATE" --samples "$SAMPLES" --out "$REPORT"
else
  ISAACLAB="${ISAACLAB_ROOT:-$HOME/IsaacLab-3.0-EA}"
  if [[ ! -d "$ISAACLAB" ]]; then echo 'STAGE41_SCIPY_MISSING_NO_PYTHON_ENV' >&2; exit 3; fi
  ( cd "$ISAACLAB" && uv run --no-sync python "$SCRIPT" \
    --urdf "$PROJECT/assets/lexium_cobot.urdf" \
    --robot-cfg "$PROJECT/robot_cfg.py" --stage35 "$STAGE35" \
    --stage40 "$STAGE40" --clean-gripper-npz "$MESH" \
    --candidate "$CANDIDATE" --samples "$SAMPLES" --out "$REPORT" )
fi
[[ -s "$REPORT" ]] || { echo 'STAGE41_REPORT_NOT_WRITTEN' >&2; exit 4; }
echo "STAGE41_JSON_READY=$REPORT"
