# Lexium Agro — Autonomous Mobile Manipulator for Hydroponic Greenhouses

> **Status as of 20 November 2026:** repository handoff/starter assembled from project documents, videos, images, historical code archives and reported simulation results. **Not** a verified executable replica of the latest Isaac Sim/Isaac Lab workspace. The real hardware has **not** been validated for autonomous fruit harvesting.

This project researches the control architecture for an autonomous mobile robot for hydroponic greenhouses. A six-axis Schneider Electric Lexium Cobot is mounted on an eight-omni-wheel mobile platform. The target pipeline is RGB-D perception → tomato detection / classification → depth-based 3D localization → hand–eye calibration → collision-aware motion planning → four-finger compliant grasp → target-container sorting → logging and later crop statistics / mobility integration.

## Start here

1. **For Codex:** read [`CODEX_IMPLEMENTATION_PROMPT.md`](CODEX_IMPLEMENTATION_PROMPT.md), then [`PROJECT_SPEC.md`](PROJECT_SPEC.md), [`docs/STATUS_AND_PROVENANCE.md`](docs/STATUS_AND_PROVENANCE.md), and [`docs/SAFETY.md`](docs/SAFETY.md).
2. Run the **offline scaffold only**. Its hardware backend always raises and cannot connect:

   ```bash
   python3 -m venv .venv
   .venv/bin/python -m pip install -e '.[dev]'
   .venv/bin/python scripts/verify_handoff.py
   .venv/bin/python -m ruff check src tests scripts
   .venv/bin/python -m mypy src/lexium_agro
   .venv/bin/python -m pytest -q
   .venv/bin/lexium-agro-dry-run
   ```
3. Inspect `legacy/2026-11-10-starter/`, `legacy/original_isaaclab_prototype/`, `legacy/extra_sources/`, and `assets/robot/references/` before migration; these are historical, not certified/current.
4. On the actual Ubuntu workstation, run [`scripts/collect_workspace.py`](scripts/collect_workspace.py) to collect current source files and raw JSON reports, then merge them without overwriting proven local work.
5. The two embedded presentation clips and the earlier raw video records are under `media/videos/`; the formatted thesis is in `thesis/`.

## Repository map

| Directory | Contents |
|---|---|
| `src/lexium_agro/` | Dependency-light, **offline-only** domain, RGB-D/TF contracts, metrics, config validation and permanently disabled hardware backend |
| `tests/` | Unit tests for pure sorting/safety logic (not reproduction of the recorded 20 tests) |
| `ros2/`, `isaac_sim/`, `isaac_lab/` | Detailed integration guidance/placeholders, not live drivers |
| `configs/` | Reference simulation/safety configuration; hardware explicitly disabled |
| `assets/robot/` | Older URDF/XRDF and CAD references; validate revisions, meshes and origins |
| `legacy/` | Original archives and extracted historical sources, preserved read-only |
| `evidence/` | Screenshots and **transcribed** terminal/benchmark data with provenance |
| `thesis/` | Overleaf-ready LaTeX thesis draft (Abstract + Chapter 1) |
| `media/` | Project summary, progress presentations, photographs, demo clips |
| `docs/` | Architecture, migration, tests, limits and risk checklist |

## Truthfulness rules

- A screenshot of **5/5 simulated tomato sorting** is one demonstration, not a general success-rate estimate.
- Phase 10's **5/5 validation scenarios** are different tests from the Stage 6 five-tomato scene.
- Model-estimated hardware torque is **not a torque measurement on a physical robot**.
- `Isaac Lab` running or a moving arm does **not** prove RL training. An RL checkpoint, training config, logs and rollout evaluation must be obtained before that claim.
- The **real robot may not be commanded** by this starter. Hardware requires independent enablement, verified manual, safety assessment, E-stop, controller limits and supervised tests.

## Documentation

- [Full scientific/engineering specification](PROJECT_SPEC.md)
- [Codex prompt / task contract](CODEX_IMPLEMENTATION_PROMPT.md)
- [Provenance and status](docs/STATUS_AND_PROVENANCE.md)
- [Verification matrix](docs/VERIFICATION_MATRIX.md)
- [Hardware safety policy](docs/SAFETY.md)
- [Migration and current workspace capture](docs/MIGRATION.md)
- [Architecture and interfaces](docs/ARCHITECTURE.md)
- [Sources and inventory](docs/FILES_AND_SOURCES.md)
- [Phase 0 hash inventory](docs/INVENTORY.md)
- [Actually executed validation and blockers](docs/VALIDATION_REPORT.md)

## Project participants

Sestrevatovskiy Ratmir; Kaldybekov Adilkhan; Maytan Yerassyl; Jumashev Tair. Supervisor: Tolengutov Rasul. University: KBTU. Check official spelling and thesis administrative records before final submission.

## License

No new license is assigned here to user-owned or third-party materials. **Do not publish the historical archives, photographs, CAD files, hardware manuals or thesis publicly without checking permission and licensing.** This scaffold is a packaging/handoff artifact.
