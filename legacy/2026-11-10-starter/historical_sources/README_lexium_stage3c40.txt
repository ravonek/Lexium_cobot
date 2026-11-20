Lexium Stage 3C.40 — Offline URDF IK & gripper-only discrete geometry gate

This ZIP contains:
  lexium_stage3c40_ik_reachability_audit.py
  run_lexium_stage3c40_ik_reachability_audit.sh
  stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json
  stage40_inputs/lexium_stage3c36_local_aperture_report.json
  stage40_inputs/lexium_stage3c39_offline_example_report.json
  stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz
  stage40_example_report.json  (generated on uploaded archival files, without Isaac Lab)

The script reads your EXISTING:
  ~/Desktop/adi/lexium_rl_workspace/lexium_pick_place/assets/lexium_cobot.urdf
  ~/Desktop/adi/lexium_rl_workspace/lexium_pick_place/robot_cfg.py
It NEVER overwrites either, runs Isaac Lab, moves joints, or changes USD.

Installation:
  PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
  unzip -o "$HOME/Downloads/lexium_stage3c40_offline_ik_bundle.zip" -d "$PROJECT"
  bash "$PROJECT/run_lexium_stage3c40_ik_reachability_audit.sh"

Result:
  /tmp/lexium_stage3c40_ik_reachability_report.json
To copy to Downloads:
  cp /tmp/lexium_stage3c40_ik_reachability_report.json "$HOME/Downloads/"

Options:
  STAGE40_BASELINE_LOG=/tmp/your_stage3c37_log.log bash run_...sh
  STAGE40_STAGE39_REPORT=/tmp/lexium_stage3c39_offline_screen.json bash run_...sh

IMPORTANT:
- Stage39 input bundled here is the example report computed from the uploaded
  Stage36/37 geometry. If you reran Stage39 on a DIFFERENT scene, replace it.
- The Stage35 measured finger hinge pivot geometry is used to reconstruct the
  physically held gripper_base_1 pose, which cross-checks recorded TCP position.
- If a stage37/stage35 log exists, the measured FINAL_ARM_Q_RAD is used to compare
  physical/URDF FK. If missing, the baseline q is inferred by solving IK, and
  'logged_q_to_measured_pose_calibrated' remains FALSE.
- 'ik_pose_match' means only numerically fitting an UNVALIDATED 6-DoF target.
- The discrete path checks are on the frozen authored gripper mesh ONLY, not
  other robot links, continuous collision, actual PhysX collision shapes, or
  self collisions. No physical movement or grasp is authorized.
