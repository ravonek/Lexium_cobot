# Recovery of the actual Ubuntu workspace

The latest simulation code was reported at:

```text
/home/aybergen/Desktop/adi/lexium_rl_workspace/lexium_pick_place/
```

The historical 10 Nov archive explicitly says it is incomplete, and the later Phase 10 log refers to files missing from the chat handoff. **Codex must prioritize an exact snapshot of the user's current Ubuntu workspace.**

## Run on the user's Ubuntu machine

After copying this ZIP to Ubuntu and unpacking it:

```bash
python3 scripts/collect_workspace.py   --workspace "$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"   --output "$HOME/Downloads/LEXIUM_CURRENT_WORKSPACE_CODEX.zip"
```

The collector preserves source/config/report files, excludes common caches, Git internals and secrets, and emits a manifest of copied/skipped files. If the project has large required USD/textures/mesh files, run with `--include-large` after reviewing the manifest. This script **does not execute or modify the robot**.

## Reconciliation process

1. **Back up** the original Ubuntu workspace before copying any refactored files into it.
2. Unpack collection into `imports/current_workspace_snapshot/` (tracked selectively/private as appropriate).
3. Check actual `robot_cfg.py`, articulated USD, URDF/XRDF and mesh files for revision, geometry and frame naming.
4. Compare Stage 3C.73–75 source and logged `ROBOT_HOME_DRIFT` with later source; retain the failure history if later code solves it.
5. Locate Stage 6 scripts, Phase 10 validation suite, raw JSON in `results/`, 20 original unit tests, plots and 8 screenshot artifacts. Integrate them without changing baseline output.
6. Inspect Isaac Sim and Isaac Lab package versions and check the installed API before running.
7. Identify whether actual RL training and policy checkpoint exist. Document outcome, never infer from videos.
8. Run harmless offline tests first. Then only scripted **simulation** tests, after review of environment paths / hardware adapter status.
9. Record all commands and validation outputs in `docs/IMPLEMENTATION_LOG.md` with timestamps, seed and git SHA.

## Critical older source constraints

Historical archive says gripper meshes were missing, USD partly incomplete, base anchoring was previously problematic, and Stage 3C.75 failed. Additional XRDF and CAD references in this ZIP come from earlier standalone user files, not the latest workspace. Treat them as **reference**. Do not replace the current robot with the old URDF/XRDF merely because it parses.
