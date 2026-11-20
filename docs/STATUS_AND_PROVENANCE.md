# Status and evidence provenance — 20 Nov 2026

| Component | Evidence in handoff | Classification | Remaining work |
|---|---|---|---|
| Physical mobile platform and mounted Lexium | User's 2026-11 photos and source progress slides | **Physically assembled / photographed** | Controller/mobile motion, wiring/E-stop audit, stability |
| Four-finger gripper | CAD, wrist mounted photo, older manual bottle-grasp pictures | **Geometry / prototype demonstrated** | Instrumented physical grasp force, repeats, food handling |
| Stage 6 tomato sorting | Final screenshot and benchmark force chart | **Simulated one scene, 5/5** | Raw JSON & code, multiple varied trials, seed data |
| Phase 10 tests A–E | User-pasted terminal log dated 2026-11-17 | **Simulation/software terminal-reported 5/5** | Actual output JSON, scripts, rerun |
| Phase 10 unit tests | Pasted `UNIT_TEST_SUMMARY passed=20 failed=0` | **Historical terminal-reported** | Recover those exact 20 files/tests and rerun |
| Stage 3C older experiments | 10 Nov starter source/log/report files | **Historical incomplete archive** | Review drift bug, import current local fixes, model revision |
| RGB-D / HSV cubes | Screenshots in prior deck | **Bench demo for colored cubes** | On-robot eye-in-hand calibration and fruit dataset |
| TinyUNet | Terminal Test B label | **Mentioned in successful test log** | Model architecture/weights/dataset/camera pipeline |
| RL in Isaac Lab | Demo clip / application screenshot only | **Unverified learning result** | Training config, checkpoint, curves, trained rollout metric |
| Navigation / ROS 2 | Architectural design, one Jazzy bridge unit test PASS in pasted log | **Partial software demonstration** | Actual mobile navigation, TF integration, live controller driver |
| Thesis | Prepared Overleaf ZIP | **Abstract + Chapter 1 draft** | Other chapters, supervisor review, signatures & RU/KZ abstracts |

## Sources

- `evidence/screenshots/`: user-provided exact graphs / robot images from 17–18 Nov, with native labels.
- `evidence/measurements/stage6_transcribed.json`: data manually transcribed from graphics, **not** the original simulator JSON.
- `evidence/measurements/phase10_transcribed.json`: data transcribed from user's 2026-11-17 pasted console log, **not** original `results/*.json`.
- `legacy/2026-11-10-starter/`: source archival record; may not be compatible with newer simulator.
- `legacy/original_isaaclab_prototype/`: earlier Gemini-source archive; its own README warns it may not run.
- `legacy/extra_sources/`: individual earlier user-owned file copies; do not merge without diffing.
- `media/documents/Project_Summary_Lexium_Agro.docx`: objectives and system description.

## Never combine inconsistent denominators

The sorting screenshot says `5 targets / 5 sorted`. The validation suite reports `5 tests / 5 PASS`, with Test C handling `3 tomatoes / 3`. These are **three different denominators**. Report each with its scope.

## Raw results that are missing from this bundle

`results/hardware_constrained_autonomous_validation.json`, `results/hardware_fidelity_audit.json`, `results/real_robot_compatibility_matrix.json`, `results/deliverables_manifest.json`, 6 motion plots, 8 screenshots per log, current phase10 scripts, current Stage6 script(s), latest `robot_cfg.py`, all current Robot USD/URDF meshes, RL config/checkpoints/training curves, and original `test_*` source files. They must be collected on Ubuntu, not guessed.
