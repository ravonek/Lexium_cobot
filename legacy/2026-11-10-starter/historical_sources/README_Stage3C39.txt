Lexium Cobot — Stage 3C.39
OFFLINE read-only opposing-finger contact hypotheses

Purpose: analyze whether finger groups (1+2) vs (3+4) can approach opposite
X/Y faces of the 40 mm cube at a static pose without authored triangle/box
penetrations. Not robot IK, not proof of contact force, not a grasp validation.

INPUTS (already created on the Ubuntu PC by previous stages):
  /tmp/lexium_stage3c36_local_aperture_report.json
  /tmp/lexium_stage3c38_offline_pose_screen.json
  /tmp/lexium_stage3c38_clean_collision_meshes.npz

INSTALL:
  PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
  unzip -o "$HOME/Downloads/lexium_stage3c39_opposing_contact_bundle.zip" -d "$PROJECT"

RUN:
  bash "$PROJECT/run_lexium_stage3c39_opposing_contact_screen.sh"

RESULT:
  /tmp/lexium_stage3c39_opposing_contact_screen.json

The offline script never starts Isaac Lab and never commands motion.
The target stage39 report intentionally does not grant permission to move
fingers, descend, or use a candidate pose on a robot.
