Stage 3C.49 — offline conservative authored-mesh convex-hull proxy screen
=======================================================================

Files:
- lexium_stage3c49_convex_hull_proxy_audit.py
- run_lexium_stage3c49_convex_hull_proxy_audit.sh
- lexium_stage3c49_offline_example_report.json (illustrative run on uploaded data)

Copy the ZIP members into:
~/Desktop/adi/lexium_rl_workspace/lexium_pick_place

Run:
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
bash "$PROJECT/run_lexium_stage3c49_convex_hull_proxy_audit.sh"

Output:
/tmp/lexium_stage3c49_convex_hull_proxy_report.json
~/Downloads/lexium_stage3c49_convex_hull_proxy_report.json

Dependencies: Stage40, Stage41 scripts, Stage35 JSON, Stage38 cleaned collision
NPZ (stage40_inputs), Stage45/47/48 user run reports. Python numpy and scipy.
No Isaac Lab launched. No motor commands. No source asset changes.

What this actually does:
Takes the full source collision-mesh vertices of 11 rigid bodies, builds exact
mathematical convex hulls in their local frame, then at each sampled FK pose
checks whether the mathematically enclosed convex polyhedra intersect. Hulls
are conservative relative to triangle surfaces and can yield false positives.

What this does NOT do:
- It does NOT read the PhysX cooked convex hulls, which may have fewer vertices;
  it does NOT certify convex approximation or contact filtering at runtime.
- It does NOT prove absence of near collisions (clearance), continuous/swept
  collisions, mechanical clearance, dynamic stability, grasp accessibility,
  finger 40mm opening, forces, or safe motion.
- HOME already shows mesh/hull overlap. The USD articulation has self collisions
  disabled. Zero new pair names relative to HOME is NOT a safety certificate.

The preferred pregrasp pose is derived in part from a Stage39 example report.
No simulated/real physical trajectory is authorized by this audit.
