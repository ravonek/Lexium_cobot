Lexium Stage 3C.45 — OFFLINE lifted pre-grasp candidate screen

Inputs expected from prior successful stages:
  $PROJECT/assets/lexium_cobot.urdf
  $PROJECT/robot_cfg.py
  $PROJECT/assets/meshes/{base_link,link1_1,link2_1,link3_1,link4_1,LINK5_1}.stl
  $PROJECT/stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json
  $PROJECT/stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz
  $PROJECT/lexium_stage3c40_ik_reachability_audit.py
  $PROJECT/lexium_stage3c41_full_arm_collision_audit.py
  /tmp/lexium_stage3c40_ik_reachability_report.json
  /tmp/lexium_stage3c44_explicit_home_fk_report.json

Usage:
  PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
  unzip -o "$HOME/Downloads/lexium_stage3c45_offline_pregrasp_bundle.zip" -d "$PROJECT"
  python3 -m py_compile "$PROJECT/lexium_stage3c45_offline_pregrasp_lift_screen.py"
  bash -n "$PROJECT/run_lexium_stage3c45_offline_pregrasp_lift_screen.sh"
  bash "$PROJECT/run_lexium_stage3c45_offline_pregrasp_lift_screen.sh"

Output:
  /tmp/lexium_stage3c45_offline_pregrasp_report.json

VERY IMPORTANT: This stage is offline, no Isaac Lab, no joint commands, no
asset edits. It uses the Stage40 candidate-5 reference from an EXAMPLE Stage39
shortlist, not an independently proved grasp geometry. It tries to lift that
same gripper orientation above the cube by 60,70,80,90,100,120 mm, solve
URDF IK, then screen 41 static configurations on direct joint interpolation
from explicit HOME. Geometry comes from URDF binary STL collision meshes and
an earlier PhysX-exported set of frozen gripper meshes. Table/cube tests
include sampled vertices and authored mesh triangles. The screening DOES NOT
validate PhysX collision shape convex decomposition, continuous swept paths,
robot self-collisions (AABB overlaps remain UNRESOLVED), grasp opening, finger
motion, contact dynamics, or safe control. DO NOT send listed q vectors to
Isaac Lab or to any physical robot. They are diagnostic data only.

File versions: prepared with the Stage44 report for the explicit HOME test.
Python/Bash syntax and offline standalone execution tested. NOT tested inside
the user's Isaac Lab, which is intentionally not launched for this stage.
