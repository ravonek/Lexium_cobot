Lexium Cobot — Stage 3C.51: Controlled PhysX Pre-Grasp Motion Trial
==================================================================

PURPOSE
Perform ONE fully simulated HOME -> selected Pre-Grasp (+100 mm, world XYZ
orientation adjustment [30,-15,30] degrees) trial in Isaac Lab. It uses the
Stage3C49 selected 6-DOF joint target without editing USD/URDF/robot_cfg.

PREREQUISITES
- Same Ubuntu project as Stage 3C.50 and working IsaacLab-3.0-EA.
- Stage3C49 report at /tmp/... or ~/Downloads/...
- Stage3C50 report at /tmp/... or ~/Downloads/...
- Local source scripts: lexium_stage3c22_urdf_gravity_torque_audit.py and
  lexium_stage3c25_corrected_gravity_route_audit.py
- These reports are REQUIRED to pass baseline checks before motion.

RUN
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
unzip -o "$HOME/Downloads/lexium_stage3c51_physx_pregrasp_motion_bundle.zip" -d "$PROJECT"
python3 -m py_compile "$PROJECT/lexium_stage3c51_physx_pregrasp_motion_trial.py"
bash -n "$PROJECT/run_lexium_stage3c51_physx_pregrasp_motion_trial.sh"
bash "$PROJECT/run_lexium_stage3c51_physx_pregrasp_motion_trial.sh"

RESULTS
- /tmp/lexium_stage3c51_physx_pregrasp_motion_trial.log
- /tmp/lexium_stage3c51_physx_pregrasp_motion_trial.json (if execution enters monitored phase)
- ~/Downloads/lexium_stage3c51_physx_pregrasp_motion_trial.json

SAFETY & INTERPRETATION
- SIMULATION ONLY. There is no connection to physical hardware.
- Performs 150 warmup, 1200 quintic trajectory, 180 hold physics steps.
- Gripper fingers remain at their initial default positions; no grasp.
- Aborts on external contact, cube displacement or lost cube-table support,
  actuator/velocity/pose/limit issues. Writes partial report if motion aborts.
- Uses gravity position compensation and motor parameters already tested in
  Stage3C37 and Stage3C50; simulation-specific assumptions must remain valid.
- PhysX self collisions are DISABLED in the source robot. Stage3C49 found five
  source-mesh convex-hull proxy overlaps in HOME. A successful external-contact
  trial does NOT validate self clearance or robot safety.
- Offline Stage3C49 was not a cooked-PhysX or continuous swept-collision check.
- The selected target came from Stage39 example-derived geometry, not an
  independently validated grasp pose. 40 mm finger aperture unverified.
- NEVER use these positions, commands, or conclusions on a real robot based
  solely on these results.

Off-machine validation performed: Python compile, bash -n, ZIP integrity and
JSON preflight metadata inspection. Actual Isaac Lab runtime has not been
run in the assistant environment.
