#!/usr/bin/env bash
set -euo pipefail
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
WORK="$HOME/Desktop/adi/lexium_rl_workspace"
SCRIPT="$PROJECT/lexium_stage3c37_collision_mesh_export_table_gate.py"
PREV32=/tmp/lexium_stage3c32b_limit_aware_probe.json
PREV33=/tmp/lexium_stage3c33_static_gripper_geometry.json
PREV34=/tmp/lexium_stage3c34_virtual_opening_audit.json
PREV35=/tmp/lexium_stage3c35_bottleneck_axis_audit.json
PREV36=/tmp/lexium_stage3c36_local_aperture_report.json
LOG=/tmp/lexium_stage3c37_mesh_export_table_gate.log
REPORT=/tmp/lexium_stage3c37_table_gate.json
NPZ=/tmp/lexium_stage3c37_full_collision_triangles.npz
for f in "$SCRIPT" "$PREV32" "$PREV33" "$PREV34" "$PREV35" "$PREV36" \
  "$PROJECT/robot_cfg.py" "$PROJECT/assets/lexium_cobot.urdf" \
  "$PROJECT/lexium_stage3c22_urdf_gravity_torque_audit.py" \
  "$PROJECT/lexium_stage3c25_corrected_gravity_route_audit.py" \
  /tmp/lexium_stage3c29_preflight.log \
  /tmp/lexium_stage3c28_candidate1_gravity_bias.log \
  /tmp/lexium_stage3c26_candidates.json; do
  [[ -s "$f" ]] || { echo "MISSING_FILE=$f" >&2; exit 3; }
done
python3 - "$PREV36" <<'PY'
import json,sys
r=json.load(open(sys.argv[1],encoding='utf8'))
assert r.get('stage')=='3C36' and r.get('no_arm_descent') is True
assert r.get('matched_slice_candidate_count',0)>0
assert r.get('grasp_validated') is False
print('PRIOR_STAGE36_REPORT_VERIFIED=True')
PY
python3 -m py_compile "$SCRIPT"
cd "$HOME/IsaacLab-3.0-EA"
echo 'STAGE=3C37 MODE=STATIC_COLLISION_MESH_EXPORT_TABLE_GATE'
echo 'Replay physically verified HOME -> Candidate1; export FULL authored USD collision mesh triangles.'
echo 'Evaluate hypothetical rigid translations only; NO fingers commanded, NO descent, NO robot asset edits.'
echo "LOG=$LOG"
echo "REPORT=$REPORT"
echo "GEOMETRY_NPZ=$NPZ"
set +e
PYTHONPATH="$WORK" uv run isaaclab -p "$SCRIPT" \
  --scene-variant full --table-center-x 0.62 --cube-x 0.30 --cube-y -0.30 \
  --robot-contact-threshold-n 0.5 \
  --warmup-steps 120 --motion-steps 900 --hold-steps 300 \
  --num_envs 1 --visualizer none \
  --stage33-report "$PREV33" --stage32b-report "$PREV32" \
  --stage34-report "$PREV34" --stage35-report "$PREV35" \
  --stage36-report "$PREV36" > "$LOG" 2>&1
status=$?
set -e
echo "ISAAC_LAB_EXIT_CODE=$status"
echo '========== BASELINE =========='
grep -E '^(FINAL_TCP_XYZ_M=|FINAL_ORIENTATION_ERROR_DEG=|JOINT2_PEAK_APPLIED_NM=|EXTERNAL_CONTACT_SCREEN_PASSED=)' "$LOG" || true
echo '========== STAGE 3C37 =========='
grep -E '^(LEXIUM_STAGE3C37_|STAGE37_)' "$LOG" || true
echo '========== ERRORS =========='
grep -Ei '(SAFETY_ABORT|Traceback|RuntimeError|Exception|Error:)' "$LOG" | tail -n 15 || true
if ((status!=0)); then
  echo 'RESULT=STAGE3C37_FAILED'
  tail -n 35 "$LOG"
  exit "$status"
fi
for marker in \
  'LEXIUM_STAGE3C30_MOTION_COMPLETE=True' \
  'LEXIUM_STAGE3C31_GEOMETRY_AUDIT_COMPLETE=True' \
  'EXTERNAL_CONTACT_SCREEN_PASSED=True' \
  'LEXIUM_STAGE3C36_LOCAL_APERTURE_AUDIT_COMPLETE=True' \
  'STAGE36_ARM_DESCENT_COMMANDED=False' \
  'STAGE36_PHYSICAL_FINGER_MOTION_COMMANDED=False' \
  'LEXIUM_STAGE3C37_MESH_EXPORT_TABLE_GATE_COMPLETE=True' \
  'STAGE37_NO_GRASP_PERMISSION=True'; do
    grep -Fxq "$marker" "$LOG" || { echo "RESULT=STAGE3C37_MISSING_MARKER=$marker" >&2; exit 7; }
done
[[ -s "$REPORT" ]] && [[ -s "$NPZ" ]] || { echo 'RESULT=STAGE3C37_MISSING_REPORT' >&2; exit 8; }
python3 - "$REPORT" "$NPZ" <<'PY'
import json, numpy as np, sys
r=json.load(open(sys.argv[1]))
assert r['stage']=='3C37' and r['no_finger_motion'] and r['no_arm_descent']
assert r['no_grasp_permission']
assert len(r['candidate_table_gates'])==10
with np.load(sys.argv[2],allow_pickle=False) as g:
    assert len(g.files)==10
    for name in ['gripper_base_1','gripper_finger_1_1','gripper_finger_2_1','gripper_finger_3_1','gripper_finger_4_1']:
        verts=g[name+'_world_vertices_m'];faces=g[name+'_triangle_indices']
        assert verts.ndim==2 and verts.shape[1]==3 and len(verts)>0
        assert faces.ndim==2 and faces.shape[1]==3 and len(faces)>0
        assert faces.min()>=0 and faces.max()<len(verts)
print('STAGE3C37_JSON_NPZ_VERIFIED=True')
PY
echo 'RESULT=STAGE3C37_COLLISION_TRIANGLES_EXPORTED_TABLE_GATE_RECORDED_NO_GRASP_PERMISSION'
