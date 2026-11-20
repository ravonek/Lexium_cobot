# LEXIUM AGRO — FULL PROJECT SPECIFICATION

**Version:** handoff 1.0, 20 November 2026. **Audience:** Codex agent, robotics engineers, thesis team. **Purpose:** faithful repository construction, not a claim of completed field deployment.

## 1. Official project identity

- **English project title:** Development of a Control System for an Autonomous Mobile Manipulator for Automating Agricultural Operations in Hydroponic Greenhouses.
- **Short name:** Lexium Agro / Autonomous Mobile Manipulator for Hydroponic Greenhouses.
- **University:** Kazakh-British Technical University (KBTU).
- **Team:** Sestrevatovskiy Ratmir Aleksandrovich; Jumashev Tair Salavatovich; Kaldybekov Adilkhan Yerbolatuly; Maytan Yerassyl Melsuly.
- **Supervisor:** Tolengutov Rasul (full formal credentials to be confirmed).
- **Focus:** fruit condition assessment and selective harvesting in hydroponic greenhouses, initially tomatoes and a four-finger adaptive grasp.

## 2. Scientific problem and motivation

Hydroponic greenhouses support year-round crop production under controlled conditions but inspection, quality assessment and selective harvesting still demand careful manual work. Fruit size, illumination, occlusion, variable geometry and mechanical fragility challenge automatic harvesting. A fixed arm cannot cover entire greenhouse rows; a mobile manipulator is intended to combine navigation, RGB-D perception, safe manipulation, a compliant end effector and data logging. The research question is how to construct and validate this multi-subsystem control pipeline while minimizing fruit damage and respecting robot kinematics, collision and contact constraints.

### Overall aim

Design, implement and validate **in simulation first**, then experimentally on real hardware under an independent safety procedure, a control system capable of detecting, localizing, approaching, gently gripping, transferring and sorting produce; eventually extend it to row-to-row navigation and crop monitoring.

### Specific objectives

1. Identify manipulator, gripper, chassis, onboard compute, RGB-D and sensor requirements.
2. Develop/reconcile CAD, URDF, USD and (where appropriate) XRDF model assets; check base anchoring and transforms.
3. Define PhysX collision, mass/inertia, joint limit, drive dynamics, torque / velocity / acceleration / jerk, grasp/contact models.
4. Detect/classify targets using color, size, shape/ripeness indicators and reliable depth-based 3D localization; use measured conditions rather than ideal labels in future vision tests.
5. Calibrate camera intrinsics, depth alignment and camera-to-tool transform (eye-in-hand); record calibration residuals.
6. Implement constrained workspace IK and collision-aware planning; safe approaches, grasp, lift, transport, place and retreat.
7. Design per-finger grip/contact/slip monitoring and rejection/recovery logic.
8. Develop a sorting controller and objective, replayable benchmark metrics.
9. Evaluate reinforcement learning in Isaac Lab **only with verifiable training artifacts** (environment, configuration, checkpoint and rollout metrics).
10. Integrate ROS 2 and the mobile base after subsystem validation; eventually support plant-by-plant statistics and greenhouses.

## 3. Physical and software platform

| Subsystem | Known from provided materials | Verify / caution |
|---|---|---|
| Mobile platform | Aluminum-profile support frame; eight omni-directional wheels; Lexium Cobot mounted upright on frame in later photographs | Base drivers, odometry, battery management, payload balance, emergency stop, measured dimensions not supplied |
| Manipulator | Schneider Electric Lexium 6-DOF collaborative arm; historical handoff mentions model `LXMRL03S0000` | Exact controller / joint type / manufacturer mechanical limits must be checked from real hardware documentation |
| Gripper | Four adaptive fingers, red/cyan geometry, physically mounted in a recent photograph; historical handoff mentions `FAE4MR86M-D` | Verify actuator, wiring, control interface, finger collision meshes and limits; no certified physical force measurements |
| Camera | Intel RealSense RGB-D D435-family, planned eye-in-hand | D435 vs D435i naming differs between supplied documents; check device identifier. Lexium-specific calibration not established |
| Compute | Historical presentation mentions Jetson and LiDAR for navigation; Ubuntu development workstation with NVIDIA GPU | Exact Jetson/LiDAR/ROS interface and deployed onboard configuration remain to be verified |
| Simulation | NVIDIA Isaac Sim 6.1.0 seen in screenshots; PhysX; Isaac Lab 3.0-EA from historical handoff | Reconfirm current local versions and correct launcher APIs; do not assume simulator is interchangeable with a physical controller |
| Robot middleware | ROS 2 Jazzy bridge Pub/Sub appears in a passing unit-test log | Bridge scope, node graph, live synchronization and physical driver not established |
| Perception | HSV color-segmentation / RGB-D colored cube experiments; Phase 10 terminal log identifies TinyUNet in Test B | Exact TinyUNet weights, input normalization, dataset and live camera test reports not in present handoff |
| Motion | Constrained planning / contact / torque audits, Stage 6 sorting, Phase 10 tests | Original scripts + raw reports required to reproduce; model torque ≠ physical measured torque |

## 4. Target operating cycle

```text
Mobile navigation (future ROS 2 / LiDAR / omni base)
           ↓ reach plant station, stop and stabilize
RGB-D acquisition + timestamp + quality check
           ↓
Tomato segmentation / detection / class / size / candidate score
           ↓ calibrated depth + camera intrinsics
3D target in camera coordinates
           ↓ eye-in-hand T_base_tool × T_tool_camera
Target pose in robot-base coordinates
           ↓ IK, self + environment collision and workspace constraints
Safe pre-grasp pose → controlled approach
           ↓
Adaptive four-finger closure + per-finger force/contact monitoring
           ↓ contact valid, no excessive force/slip
Grasp → lift → transit → bin selection (color/size/ripeness)
           ↓
Place → release → retreat → verify completion → record metrics
           ↘ if invalid: reject / timeout / recover to safe pose
```

The **fully autonomous physical cycle is an objective, not a verified completed functionality**. Current recordings are simulation demos and real hardware setup photographs.

## 5. Coordinate and unit requirements

- SI base units: positions **m**, orientation **rad** (or normalized quaternion with documented `[w,x,y,z]` vs `[x,y,z,w]` convention), forces **N**, torque **N·m**, time **s**.
- ROS frames to formalize: `map`, `odom`, `base_link` (mobile), `arm_base`, `tool0` / `tcp`, `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame`, `target_<id>`.
- In the app, declare a unique source/target convention, `T_A_B` meaning transform of B coordinates into A, and verify round-trip tests. Do not confuse mobile base with arm base.
- For a valid pixel `(u,v)` with intrinsics `(fx,fy,cx,cy)` and metric depth `z`:
  `x = (u-cx)*z/fx`, `y = (v-cy)*z/fy`; transform the 3D point into arm base via calibrated extrinsics. Reject invalid depth, out-of-range poses, stale timestamps and insufficient point support.
- Solve eye-in-hand calibration using multiple poses; ChArUco/solvePnP trial in older work does not establish a valid final Lexium transform.
- Verify upright physical arm base reference. A historical USD had `fixed base: False` and orientation concerns; validate newest files rather than force-setting transforms.

## 6. Main software components, interfaces and contracts

### 6.1 `robot_model`
Inputs: URDF, STL/meshes, USD and optional XRDF. Outputs: articulated simulated model and validated frame tree. Validate all six arm joints, gripper joints, masses/inertia, collision proxy, contact sensor bodies, limits and Jacobian convention. Revision artifacts should be hashed. Do not modify original CAD files in place.

### 6.2 `perception`
Inputs: RGB, registered depth image, intrinsics/extrinsics, acquisition timestamp; optional model weights. Outputs: candidate `{object_id, class, color, estimated_diameter_mm, mask/bbox, pose_camera, confidence, covariance/quality}`. Baseline: HSV segmentation + morphology/watershed; research path: TinyUNet or detection/segmentation network. Test occlusion, multiple fruits, lighting variation, invalid depth and repeated detections. Do not use ground-truth object positions as if estimated by camera.

### 6.3 `calibration` and `tf`
Camera intrinsics; color-depth alignment; hand-eye calibration; transform tree. Calibration includes source image/pose pairs, matrix, date, reprojection/translation errors and serialization format. Require test for inverse consistency, quaternion normalization, timestamp alignment and `camera -> tool -> arm_base` convention.

### 6.4 `planning`
IK with branch selection; joint and Cartesian constraints, collision checking (self, robot-platform, neighboring produce, bins), smooth time parameterization and safety margins. Historical project explored septimic trajectories with boundary derivatives, torque and gravity screening. Treat 10,717 trajectory samples / 0.593 rad/s / 29.98° as **one historical audit**, not universal constraints. Required report: command pose, joint path, sampled max v/a/j, torque model choice, clearance, achieved TCP error and rejection reason.

### 6.5 `gripper` and `force`
Four finger channels F1–F4; contact detection, grip closure logic, slip/weight transfer, uncertain contacts, release verification and actuator limits. The Stage 6 plot shows **2.80 N per-finger simulation ceiling**, with contact forces displayed below that line. This threshold is **not** an approved real-world hardware safety value. A physical force calibration and control strategy are still required.

### 6.6 `sorting` / orchestration
States: `SAFE_START`, `READY`, `ACQUIRE`, `LOCALIZE`, `PLAN`, `APPROACH`, `GRASP`, `LIFT`, `TRANSPORT`, `PLACE`, `VERIFY`, `COMPLETE`, `REJECT`, `RECOVER`, `FAULT`, `ESTOP`. IDs and transitions should be timestamped. Candidate selection based on classification and graspability, then deterministic bin mapping for initial red/green demos. Support retry/replan, avoid double counting, error reporting and safe rejection. Stage 6 scene of five tomatoes is evidence of a simulated scenario, not guaranteed physical sorting.

### 6.7 `isaac_sim`
Stage/scene construction, robot asset loading, PhysX + articulated model, targets and bins, deterministic replay, headless or GUI options, simulated RGB-D and contact sensors, logs. Do not make API calls from a generic Python environment without Isaac installed.

### 6.8 `isaac_lab` (RL experimental path)
Manager-based/direct task as appropriate to installed version; define observations, actions, reward functions (success, gentle grasp, object placement, limit adherence, collision penalties), termination and reset logic, domain randomization, curriculum and seed-controlled evaluation. **Training status is unknown from supplied materials**; require checkpoint/logs and comparative scripted baseline before asserting a learned policy.

### 6.9 `ros2` and hardware (future)
ROS 2 Jazzy bridge / nodes for perception, joint state, TF and coordination; mobile base and controller adapters behind explicit interfaces. No live connection string/protocol or physical controller motion command is established by this ZIP; default hardware mode MUST be disabled and fail closed. Controller link to EcoStruxure Cobot Expert requires vendor verification and safety signoff.

### 6.10 Metrics / reproducibility
Every trial needs `run_id`, scene version, robot asset hash, software versions, seed, target positions, start/end timestamps, number attempted, number detected, grasped, placed, rejected, accuracy definitions, localization/placement error and max force/torque with source (`sim`, `model`, `sensor`). Provide independent physical vs simulation metrics, plus median/P95, standard deviations and confidence bounds when sample sizes permit.

## 7. Evidence available now (limited recorded tests)

### 7.1 Stage 6 — one autonomous multi-tomato sorting scene

The screenshot `evidence/screenshots/stage6_final_sorted.png` displays `Total 5`, `Detected 5`, `Sorted 5`, `Remaining 0`, `State COMPLETE`, `Contact VALID`, `Slip State STABLE_GRASP` and `PeakF 1.91 N`. The benchmark graph has 100% shown in five categories (Stage 5C regression, color classification, size classification, per-tomato sorting, complete scenes) with different acceptance targets. There is no raw dataset or repeated-run denominator in supplied images; **do not generalize “100% accuracy.”** The per-finger force trace is from the simulation with a **2.80 N ceiling**. These Stage 6 numbers must remain separate from Phase 10.

### 7.2 Phase 10 — 17 November 2026 terminal-log suite

Five validation tasks show success (reported by user-supplied terminal output):

| Test | Outcome / logged key metrics |
|---|---|
| A Startup | PASS, 4.53 s, URDF-model torque 47.58 N·m, hardware-oriented model 20.45 N·m |
| B Single Tomato (`TinyUNet + 4F`) | PASS, localization 1.97 mm, placement 12.81 mm, URDF 50.08 N·m, hardware-oriented 22.90 N·m |
| C Three Tomatoes | PASS, 3/3 handled, localization 3.10 mm, URDF 62.72 N·m, hardware-oriented 22.96 N·m, simulated contact 2.32 N |
| D Randomized Reachable | PASS, placement errors 9.65 and 17.64 mm, URDF 53.30 N·m |
| E Safe Rejection/Recovery | PASS, `rejected=True`, recovery error 0.0020 rad |

The same terminal output gives `PHASE10_VALIDATION_SUITE overall_pass=True`, `MOTION_VALIDATE_COMPLETE valid=True`, 10,717 trajectory samples, maximum joint velocity 0.593 rad/s, maximum URDF-estimated torque 62.71 N·m, maximum hardware-oriented *estimated* torque 22.96 N·m and minimum angular joint margin 29.98°. It also reports 20/20 unit tests passed and 21/21 checked deliverables. **Those test scripts and raw JSON files are not in this handoff ZIP** and their results cannot be freshly reproduced here. The different 62.71 vs 62.72 logged torque peaks must be preserved as separate values (different reported runs/sampling/rounding); do not silently normalize them.

### 7.3 Physical hardware

Photographs demonstrate the robot mounted upright on an aluminum profile/omni-wheel platform and a four-finger end effector fitted to the Lexium wrist; prior manual bottle grasp demos and vision calibration fixtures exist. No evidence here of completed physical autonomous tomato manipulation, accurate hand-eye calibration on Lexium or quantified physical contact force/damage tests.

## 8. Critical gaps and known hazards

- Latest source code and raw results absent: Stage 6/Phase 10 scripts, tests, report JSON, model assets, Isaac Lab experiment configs, screenshots manifests.
- Old Nov 10 starter ZIP was already identified as **incomplete**, specifically missing Stage 3C.68–75 current scripts and gripper mesh set; one older Stage 3C.75 attempt aborted due `ROBOT_HOME_DRIFT`. Do not conflate an older failing historical state with later test suite passing, or assume it has been resolved in all configurations.
- RealSense camera exact model, extrinsic calibration and full acquisition chain unconfirmed.
- Schneider official joint/torque/velocity limits and physical gripper force limits must be verified from actual robot model/controller, not inferred from physics logs.
- RL training/evaluation evidence absent; Isaac Lab footage alone cannot prove policy learning.
- Mobile base localization/navigation and physical controller synchronization not demonstrated.
- Thesis only has draft Abstract/Introduction; literature/experiments/conclusions and administrative pages pending.

## 9. Acceptance milestones (evidence-gated)

- **M0 Inventory:** preserve files/hashes, source-vs-scaffold separation, offline unit tests pass.
- **M1 Model:** reproduce correct USD/articulation/IK/physics for at least a fixed pose, with validated URDF joint map/meshes.
- **M2 Perception:** quantify 3D error from RGB-D across independent varied scenes and invalid-depth rejection.
- **M3 Manipulation:** re-run Phase 10 five cases from actual sources and produce raw logs with comparable metrics; no claims without rerun.
- **M4 Sorting:** reproducible multi-tomato benchmark across seeds, layout/lighting/object variations, bins and contact constraints.
- **M5 RL:** train/play and evaluate learned policy only if checkpoints and learning curves exist, compare baseline.
- **M6 Hardware pilot:** separate reviewed safety authorization, valid stop chain, calibrated robot/tool, limited monitored motion, independent safe observer.
- **M7 Mobile/greenhouse:** integrated movement with localization, stabilized base and real target collection; metrics for grasp damage, navigation and end-to-end cycle.

## 10. Deliverables for Codex

Versioned Git repo; reproducible setup and scripts; ordered migration from historical to current source; test suite with honest reporting; ROS2/Isaac interfaces; checked reference data; an architecture diagram; train/play/evaluate interfaces (if RL genuinely implemented); clear current-status README; no secrets; generated manifest; and a documented list of missing original files.

## 11. Materials in this ZIP

- Exact original instructor thesis template ZIP and adapted Overleaf draft.
- Original older source archives including 10 Nov historical Python and URDF partial assets, plus optional separate XRDF/CAD references.
- Project Summary docx, final presentation with 2× videos, raw/speeded simulation clips, photographs and benchmark graphs.
- New repository scaffold with safe offline runnable sorting logic and tests.
- Source provenance registry `evidence/measurements/*.json`, not fabricated original raw results.

**Codex instruction:** prioritize faithful reproduction and data provenance over impressive but unjustified completion claims.
