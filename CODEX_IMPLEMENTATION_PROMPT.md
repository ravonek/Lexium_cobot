# COPY THIS PROMPT TO CODEX — Lexium Agro repository implementation

You are a senior robotics software engineer and repository maintainer. Build a **reproducible, auditable, testable Git repository** from the files in this ZIP without inventing experiments or passing results. Be conservative about the physical robot. **Read all linked instructions and provenance first.**

## Primary objective

Turn the contents of this folder into an organized Git repository for the diploma project **“Development of a Control System for an Autonomous Mobile Manipulator for Automating Agricultural Operations in Hydroponic Greenhouses”**. Preserve the real historical source scripts and reports, reconcile their versions, and define a path to a working Isaac Sim + Isaac Lab + ROS 2 + RGB-D + gripper system. Start from the current user's workspace when it becomes available; do **not** rewrite a proven controller from scratch unnecessarily.

## Required context documents (read in this order)

1. `PROJECT_SPEC.md` — scope, detailed functions, algorithms, physical system.
2. `docs/STATUS_AND_PROVENANCE.md` — confirmed vs unconfirmed results.
3. `docs/SAFETY.md` — strict simulation/hardware boundary.
4. `docs/ARCHITECTURE.md` — modular interfaces and schemas.
5. `docs/VERIFICATION_MATRIX.md` — acceptance criteria and metrics.
6. `docs/MIGRATION.md` and `docs/FILES_AND_SOURCES.md` — archive import plan, missing authoritative files.
7. `thesis/README_PROGRESS.md` and legacy `README_FIRST.md` / `CURRENT_STATUS_AND_MISSING_FILES.md`.

## Mandatory implementation sequence

**Phase 0: Inventory and freeze.** Enumerate all files and calculate hashes, distinguish originals vs generated/scaffold files. Inspect `legacy/archives/LEXIUM_GEMINI_STARTER_2026-11-10.zip` and extracted historical scripts. Run offline baseline tests; record results in `docs/IMPLEMENTATION_LOG.md`. Do NOT call these unit tests “the original 20 tests.” Check git-ignore and sensitive data before any push.

**Phase 1: Current-source import.** Ask the user to run `python scripts/collect_workspace.py --workspace ~/Desktop/adi/lexium_rl_workspace/lexium_pick_place --output current_workspace.zip` on the original Ubuntu host; import its current URDF/USD/meshes/XRDF, `robot_cfg.py`, Stage 6/Phase 10 scripts, real raw JSON reports, tests, configs and (if present) RL checkpoint metadata. Ensure file paths, scales, units, joint axes, handedness and robot-base orientation match the physical upright mounting. Never substitute older archive geometry silently.

**Phase 2: Clean architecture.** Establish coherent packages for domain/state-machine, RGB-D perception, camera calibration / TF, kinematics / motion planning, adaptive gripping, sorting, simulations, ROS 2 adapters, benchmarks, and optional Isaac Lab learning. Separate *pure logic* from Isaac / ROS imports so it can be unit-tested without GPU. Add typed interfaces, structured logging, seed handling and config validation.

**Phase 3: Simulator integration.** Reuse the actual latest USD/URDF rather than a placeholder robot. Build a deterministic Isaac Sim scene with camera, trays and contact sensors; evaluate collision geometry, constraints, physics stability and known test cases. Preserve existing validated scripts before refactoring and provide old-vs-new regression comparisons. Expose a headless run command **only after** the local environment is inspected. Do not assume APIs for Isaac Lab 3.0-EA / Isaac Sim 6.1.0 without validating against the installed versions.

**Phase 4: Perception.** Implement baseline HSV segmentation and/or existing TinyUNet code if actually present. Use camera intrinsics and depth validity checks to estimate 3D position; distinguish synthetic ground truth from measured RGB-D observations. Implement eye-in-hand calibration; treat current Lexium-specific camera-to-flange calibration as pending.

**Phase 5: Manipulation/sorting.** Integrate constrained IK, path planning, approach/lift/place, per-finger contact sensing, force/slip monitoring, object color and size classification, bin assignment, retry and safe rejection/recovery. Capture structured per-object metrics with clear error definitions. Do not treat raw gripper-contact simulation ceilings as certified robot force limits.

**Phase 6: RL (conditional).** Audit what actually exists. If training scripts/checkpoints are missing, create an **explicit experimental** Isaac Lab task, observation/action/reward config, training/playing commands and seed-controlled evaluation harness; report that training is not yet evidenced. Never label scripted Isaac Lab motion as learned RL control. Save training curves, checkpoints, config snapshot and policy-vs-scripted baseline if training is actually performed.

**Phase 7: ROS 2 / mobile integration (planned).** Use ROS 2 Jazzy where the environment supports it; introduce adapters for robot joint states, velocity/pose commands, RGB-D, `/tf`, gripper and mobile base. Validate frames and units. Physical Schneider controller interface and omnidirectional base driver remain unverified; implement interfaces/mock drivers first. Do not invent vendor protocol endpoints.

**Phase 8: Evidence, release.** Provide command-level reproducibility, CI for offline tests, a validation report with denominators and separate simulation/physical sections, diagrams, demonstration instructions and a clean README. Preserve thesis/presentation files as documentation artifacts (not source-of-truth for tested code).

## Deliverable tree / code quality

- Use `src/lexium_agro`, `tests`, `configs`, `ros2`, `isaac_sim`, `isaac_lab`, `assets/robot`, `docs`, `evidence`, `scripts`, `thesis`, `media`.
- Python packaging via `pyproject.toml`, lint/typing config, `pytest`, logging and minimal CI.
- Provide runnable commands; fail loudly for unsupported Isaac/ROS packages; never silently mock physics in a validation run.
- Small, reviewable commits. Do not delete user files or original results. Existing legacy source remains unchanged; import into active code via tracked diffs.
- Record every blocked item, needed artifact and proposed resolution. Do not claim a run succeeded unless its process was actually invoked and results inspected.
- Ensure coordinate conventions (m, rad, N, N·m), quaternions, frame names, timestamps and transforms are explicit.
- Ensure derived percent metrics specify numerator, denominator, scenarios, seeds and what “success” means.

## Zero-ambiguity hardware rule

**NO real-arm movement and NO live mobile robot movement are authorized.** Keep a hard-disabled hardware backend until the user explicitly authorizes a safety-reviewed controlled test and provides actual hardware/controller interface specifications, E-stop/physical guards, rated joint/torque/velocity limits and independent safeguards. Any real actuation must be opt-in and not reachable through default tests or demos.

## How to conclude your work

Give a concise report containing (1) what changed, (2) tests actually executed with commands and outputs, (3) sources still missing, (4) simulation vs real hardware distinctions, (5) next exact command on Ubuntu, and (6) new repository URL **only if** actually created with user's authorization and access. The repository must be truthful even if only the offline scaffold can be run initially.
