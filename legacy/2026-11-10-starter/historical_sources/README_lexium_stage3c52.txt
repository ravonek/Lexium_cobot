STAGE 3C.52 — Corrected frozen-gripper frame audit (OFFLINE)
=================================================================

PURPOSE
-------
Correct a real reference-frame bug in prior Stage 3C.41–3C.49 offline
collision screens and re-evaluate the already executed Stage 3C.51 motion.

Stage 3C.38 NPZ stores all 5 gripper world-space meshes at a measured pose.
Previous load_frozen_gripper() converts every mesh to gripper_base_1 local
coordinates. Stage 3C.41 through 3C.49 then transformed frozen finger
meshes with EACH FINGER LINK's FK, although these vertices were already in
gripper_base_1 frame. Stage 3C.47 used the correct rigid-base mapping to
verify these meshes against actual HOME USD geometry (max delta 0.035mm).

FIX IN THIS TEST
----------------
For each sampled arm position, use gripper_base_1 FK for all five frozen
meshes. Arm links still use their own URDF FK. Nothing is edited or
replaced in your original project. This test includes convex-hull proxy
checks (41 samples), compares old and new transforms in HOME, and measures
where the actual Stage 3C.51 final finger meshes are relative to the cube.

REQUIREMENTS
------------
Installed prior Stage 3C.40 / 3C.41 / 3C.49 modules, assets, Stage35 JSON,
Stage38 clean NPZ and Stage47, Stage49, Stage51 downloaded reports.
Python packages: numpy, scipy.

INSTALL
-------
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
unzip -o "$HOME/Downloads/lexium_stage3c52_corrected_gripper_geometry_bundle.zip" -d "$PROJECT"
python3 -m py_compile "$PROJECT/lexium_stage3c52_corrected_gripper_geometry_audit.py"
bash -n "$PROJECT/run_lexium_stage3c52_corrected_gripper_geometry_audit.sh"

RUN
---
bash "$PROJECT/run_lexium_stage3c52_corrected_gripper_geometry_audit.sh"

OUTPUT
------
~/Downloads/lexium_stage3c52_corrected_gripper_geometry_report.json

Expected independent example run: three proxy overlaps in HOME, zero at
endpoint, one frame with a new pair (link1_1 / gripper_finger_2_1) at t=.175,
no table or cube proxy overlap in 41 samples. Stage3C51 final finger mesh
lowest Z approximately .140m; cube top Z approximately .040m, so frozen
finger geometry is ~100 mm above cube top. TCP is ~141mm displaced in Y
from cube, but TCP need not be physical grasp center.

LIMITATIONS
-----------
* These are authored mesh convex hulls, not PhysX cooked shapes.
* Motion is not re-run; Stage51 itself already completed independently.
* Self collisions were disabled in the Stage51 PhysX simulation.
* Frozen finger angles assume default finger pose during motion.
* 41 samples are not continuous swept collision checks.
* Neither grasp-center alignment, 40mm finger aperture, grip force nor
  safe approach/descent is validated.
* DO NOT command cube descent, finger closure, or hardware robot motion.
