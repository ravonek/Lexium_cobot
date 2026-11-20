Stage 3C.46 — OFFLINE full-arm self-collision authored-mesh surface audit

Inputs:
 - Current project's URDF, robot_cfg.py and six STL arm meshes
 - Stage40 and Stage41 Python modules already installed for Stage45
 - stage40_inputs/lexium_stage3c35_bottleneck_axis_audit.json
 - stage40_inputs/lexium_stage3c38_clean_collision_meshes.npz
 - Stage45 report /tmp/lexium_stage3c45_offline_pregrasp_report.json
   or ~/Downloads/lexium_stage3c45_offline_pregrasp_report.json

Install this ZIP in the existing project directory (do not extract into Isaac Lab root).
Run bash run_lexium_stage3c46_self_mesh_narrowphase.sh
Output: /tmp/lexium_stage3c46_self_mesh_narrowphase_report.json

The audit checks 41 discrete configurations on HOME -> hypothetical pre-grasp +100mm.
It tests triangle surface intersection for all nonadjacent URDF link pairs using
body-oriented bounding volume hierarchy (BVH) and triangle-vs-triangle SAT,
including coplanar edge axes. Authoring meshes only, finger joints frozen to
the exported Stage38 state, not validated against Stage44 actual finger state.
Stage45 pre-grasp reference comes from example Stage39 source.

WARNING: This does NOT check closed volume containment, 5mm self margins,
real PhysX collision shapes, continuous/swept collisions, dynamic torques,
40mm gripper opening or grasp. Never send audit joint vectors to a robot.
AABB overlaps alone are NOT evidence of intersecting surface meshes.
SURFACE_INTERSECTION is evidence of triangle surface overlap in authored data.
At no point does this tool launch Isaac Lab, move joints or edit robot assets.
