# Validation report — 20 November 2026

## Scope

This report covers only the repository handoff and dependency-light offline code
executed on macOS. No Isaac Sim, Isaac Lab, ROS 2, RGB-D camera, Schneider
controller, gripper actuator, or mobile base was started or contacted.

## Actually executed

| Check | Command | Result |
|---|---|---|
| Handoff integrity | `.venv/bin/python scripts/verify_handoff.py` | valid; 214/214 declared files present; 0 changed preserved source artifacts; 4/4 nested ZIPs valid |
| Offline tests | `.venv/bin/python -m pytest -q` | 30 passed |
| Lint | `.venv/bin/python -m ruff check src tests scripts` | all checks passed |
| Static typing | `.venv/bin/python -m mypy src/lexium_agro` | success; 18 source files |
| Byte compilation | `.venv/bin/python -m compileall -q src tests scripts` | exit 0, no output |
| Safe example | `.venv/bin/lexium-agro-dry-run` | synthetic-only JSON; `mode=offline_dry_run`; physical motion denied |

The 30 tests above are new scaffold tests. They are not the missing historical
Phase 10 suite reported as 20/20.

## Acceptance status

| Milestone | Status | Evidence / blocker |
|---|---|---|
| M0 Inventory | PASS for this handoff | Hash manifest, nested archive tests, local Git history, offline checks |
| M1 Model | BLOCKED | Latest USD/URDF/meshes and installed Isaac versions not supplied |
| M2 Perception | BLOCKED | Camera data, intrinsics, depth alignment, model weights, and hand-eye calibration absent |
| M3 Manipulation | BLOCKED | Phase 10 source, raw JSON, plots, and original tests absent |
| M4 Sorting | BLOCKED | Stage 6 controller/raw report and repeated seeded trials absent |
| M5 RL | NOT EVIDENCED | No verified checkpoint, training config, curves, or learned-policy evaluation |
| M6 Hardware pilot | NOT AUTHORIZED | Independent safety review and controller specifications absent |
| M7 Mobile/greenhouse | PLANNED | Driver, localization, TF, base stabilization, and field evidence absent |

## Evidence separation

- Stage 6 screenshot: one simulated scene with 5 targets and 5 sorted; no raw
  repeat denominator is available.
- Phase 10 transcription: five reported scenarios, including a separate
  three-tomato case; it was not rerun here.
- Physical evidence: assembly photographs and prototype media only. There is no
  verified autonomous physical harvesting run in this repository.
- The offline pinhole/transform/state/metrics tests validate mathematical and
  fail-closed software contracts only; they do not validate a simulated or real
  robot.
