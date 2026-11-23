<div align="center">

# 🍅 LEXIUM AGRO

### Autonomous Mobile Manipulation for Hydroponic Greenhouses

**See · understand · reach · grasp · sort**

<br />

<a href="media/videos/Isaac_Sim_2x.mp4">
  <img src="media/previews/isaac_sim_motion.gif" alt="Lexium Agro motion in Isaac Sim" width="900" />
</a>

<br />

<sub>Recorded project footage · click the animation to open the full Isaac Sim video</sub>

<br /><br />

[![Offline validation](https://github.com/ravonek/Lexium_cobot/actions/workflows/offline.yml/badge.svg)](https://github.com/ravonek/Lexium_cobot/actions/workflows/offline.yml)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![NVIDIA Isaac](https://img.shields.io/badge/NVIDIA-Isaac%20Sim%20%2F%20Lab-76B900?logo=nvidia&logoColor=white)](isaac_sim/README.md)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20planned-22314E?logo=ros&logoColor=white)](ros2/README.md)
[![Hardware](https://img.shields.io/badge/live%20hardware-disabled-DC3545)](docs/SAFETY.md)
[![Status](https://img.shields.io/badge/status-research%20prototype-F59E0B)](docs/STATUS_AND_PROVENANCE.md)

[**Demos**](#-demos) ·
[**Platform**](#-physical-platform) ·
[**Results**](#-recorded-evidence) ·
[**Architecture**](#-system-architecture) ·
[**Quick start**](#-quick-start) ·
[**Roadmap**](#-roadmap)

</div>

---

## The project in one minute

Lexium Agro is a KBTU diploma research project for selective harvesting and
produce sorting in hydroponic greenhouses. The platform combines a **6-axis
Schneider Electric Lexium Cobot**, an **8-wheel omnidirectional mobile base**,
an **RGB-D perception pipeline**, and a **four-finger adaptive gripper**.

The long-term goal is an evidence-driven autonomous cycle:

```text
observe → detect → localize in 3D → plan → approach → grasp → transfer → sort → log
```

<table>
  <tr>
    <td align="center"><strong>6 DOF</strong><br /><sub>Lexium arm</sub></td>
    <td align="center"><strong>8 wheels</strong><br /><sub>omnidirectional base</sub></td>
    <td align="center"><strong>4 fingers</strong><br /><sub>adaptive gripper</sub></td>
    <td align="center"><strong>30 tests</strong><br /><sub>offline suite</sub></td>
    <td align="center"><strong>5 / 5</strong><br /><sub>one recorded Stage 6 scene</sub></td>
  </tr>
</table>

> [!IMPORTANT]
> The repository separates verified offline checks, historical simulation evidence,
> and physical prototype media. A video is not treated as proof of autonomous field
> operation, and an Isaac Lab recording is not treated as proof of a trained policy.

## 🎬 Demos

The previews below are generated directly from the supplied project recordings.
Click any preview to open the original video.

<table>
  <tr>
    <th width="50%">Isaac Sim — scripted manipulation scene</th>
    <th width="50%">Isaac Lab — application recording</th>
  </tr>
  <tr>
    <td align="center">
      <a href="media/videos/Isaac_Sim_2x.mp4">
        <img src="media/previews/isaac_sim_motion.gif" alt="Isaac Sim demonstration" width="100%" />
      </a>
    </td>
    <td align="center">
      <a href="media/videos/Isaac_Lab_2x.mp4">
        <img src="media/previews/isaac_lab_motion.gif" alt="Isaac Lab demonstration" width="100%" />
      </a>
    </td>
  </tr>
  <tr>
    <td align="center"><a href="media/videos/Isaac_Sim_original_2026-11-18.mp4">▶ Original-speed video</a></td>
    <td align="center"><a href="media/videos/Isaac_Lab_original_2026-11-18.mp4">▶ Original-speed video</a></td>
  </tr>
</table>

<table>
  <tr>
    <th width="38%">Physical prototype recording</th>
    <th width="62%">Stage 6 recorded final scene</th>
  </tr>
  <tr>
    <td align="center">
      <a href="media/videos/real_robot_phone_video.mp4">
        <img src="media/previews/real_robot_motion.gif" alt="Physical Lexium prototype demonstration" height="430" />
      </a>
    </td>
    <td align="center">
      <img src="evidence/screenshots/stage6_final_sorted.png" alt="Stage 6 autonomous multi-tomato sorting final scene" width="100%" />
    </td>
  </tr>
  <tr>
    <td align="center"><a href="media/videos/real_robot_phone_video.mp4">▶ Open phone recording</a><br /><sub>Prototype bench demo; not autonomous harvesting evidence</sub></td>
    <td align="center"><sub>Supplied simulation screenshot: 5 detected, 5 sorted, 0 remaining</sub></td>
  </tr>
</table>

More supplied recordings are available in [`media/videos`](media/videos/README.md).

## 🤖 Physical platform

<table>
  <tr>
    <td align="center" width="50%">
      <img src="media/photos/upright_robot_mobile_base.jpeg" alt="Lexium Cobot mounted upright on the eight-wheel base" width="100%" />
      <br /><sub><b>Mobile manipulator prototype</b><br />Upright arm on an aluminium-profile base</sub>
    </td>
    <td align="center" width="50%">
      <img src="media/photos/real_lexium_gripper.jpeg" alt="Four-finger adaptive gripper installed on the Lexium Cobot" width="100%" />
      <br /><sub><b>Four-finger end effector</b><br />Physical gripper integration on the wrist</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="media/photos/robot_mount_view.jpeg" alt="Lexium Cobot mounting view on the mobile platform" width="100%" />
      <br /><sub><b>Mechanical integration</b><br />Arm mounting, frame, cabling, and omni wheels</sub>
    </td>
    <td align="center" width="50%">
      <img src="media/photos/sim_display_workstation.jpeg" alt="Lexium simulation development workstation" width="100%" />
      <br /><sub><b>Simulation workstation</b><br />Development and recorded simulator environment</sub>
    </td>
  </tr>
</table>

| Subsystem | Current evidence | Next validation gate |
|---|---|---|
| Lexium arm + mobile base | Physically assembled and photographed | Controller, limits, stability, and E-stop review |
| Four-finger gripper | CAD, installed prototype, bench media | Calibrated physical force and repeatability study |
| RGB-D perception | Earlier HSV/cube experiments and historical TinyUNet mention | Camera intrinsics, aligned depth, fruit dataset |
| Eye-in-hand calibration | Workflow defined | Lexium-specific transform and held-out residuals |
| Mobile navigation | Platform and architecture exist | Drivers, odometry, localization, and safe stop |

## 📊 Recorded evidence

### Stage 6 — one supplied simulation scene

<table>
  <tr>
    <td align="center" width="50%">
      <img src="evidence/screenshots/stage6_benchmark.png" alt="Stage 6 quantitative benchmark chart" width="100%" />
    </td>
    <td align="center" width="50%">
      <img src="evidence/screenshots/stage6_contact_forces.png" alt="Stage 6 per-finger contact forces" width="100%" />
    </td>
  </tr>
  <tr>
    <td align="center"><sub>Displayed benchmark categories and acceptance targets</sub></td>
    <td align="center"><sub>Simulated per-finger force traces and 2.80 N simulation ceiling</sub></td>
  </tr>
</table>

| Recorded overlay | Value | Evidence boundary |
|---|---:|---|
| Objects detected / sorted | **5 / 5** | One supplied simulation scene |
| Remaining objects | **0** | Screenshot transcription |
| Peak force shown | **1.91 N** | Simulated contact overlay |
| Contact / slip state | **VALID / STABLE_GRASP** | Simulated state display |

### Phase 10 — historical terminal transcription

<table>
  <tr>
    <td align="center"><strong>5 / 5</strong><br /><sub>reported validation scenarios</sub></td>
    <td align="center"><strong>10,717</strong><br /><sub>reported trajectory samples</sub></td>
    <td align="center"><strong>0.593 rad/s</strong><br /><sub>reported maximum joint velocity</sub></td>
    <td align="center"><strong>29.98°</strong><br /><sub>reported minimum joint margin</sub></td>
  </tr>
</table>

The Phase 10 source scripts and original raw JSON were not included in the handoff,
so those numbers remain **historical terminal-reported values**, not a fresh rerun.
See [status and provenance](docs/STATUS_AND_PROVENANCE.md) and the
[validation report](docs/VALIDATION_REPORT.md) for the exact distinctions.

## 🧠 System architecture

```mermaid
flowchart LR
    CAM[RGB-D camera] -->|RGB + aligned depth| PER[Perception]
    PER -->|class, mask, pose, confidence| TF[Calibration + TF]
    TF -->|target in arm base| PLAN[Constrained planning]
    PLAN -->|approach + transfer path| GRIP[Gripper supervisor]
    GRIP -->|contact + slip state| FSM[Sorting state machine]
    FSM -->|red / green / reject| BIN[Destination bin]

    SAFE[Safety monitor] -. gates .-> TF
    SAFE -. gates .-> PLAN
    SAFE -. gates .-> GRIP
    SAFE -. gates .-> FSM

    SIM[Isaac Sim / PhysX] <--> PLAN
    SIM <--> GRIP
    LAB[Isaac Lab experiment] -. optional .-> SIM
    ROS[ROS 2 simulation adapters] <--> FSM
    HW[Physical backend: disabled]:::blocked
    FSM -. no default route .-> HW

    classDef blocked fill:#4a1111,stroke:#ef4444,color:#ffffff;
```

```mermaid
stateDiagram-v2
    [*] --> SAFE_START
    SAFE_START --> READY
    READY --> ACQUIRE
    ACQUIRE --> LOCALIZE
    LOCALIZE --> PLAN
    PLAN --> APPROACH
    APPROACH --> GRASP
    GRASP --> LIFT
    LIFT --> TRANSPORT
    TRANSPORT --> PLACE
    PLACE --> VERIFY
    VERIFY --> COMPLETE
    ACQUIRE --> REJECT
    LOCALIZE --> REJECT
    PLAN --> REJECT
    REJECT --> RECOVER
    RECOVER --> READY
    COMPLETE --> READY
    SAFE_START --> FAULT
    state ESTOP <<choice>>
```

The implemented dependency-light layer contains fail-closed configuration, sorting
decisions, state transitions, RGB-D pinhole geometry, rigid transforms, metrics,
deterministic seeds, structured logging, and a permanently disabled hardware backend.

## 🚀 Quick start

The following workflow is offline-only. It does not import Isaac, ROS, a camera
driver, or a physical robot controller.

```bash
git clone https://github.com/ravonek/Lexium_cobot.git
cd Lexium_cobot

python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'

.venv/bin/python scripts/verify_handoff.py
.venv/bin/python -m ruff check src tests scripts
.venv/bin/python -m mypy src/lexium_agro
.venv/bin/python -m pytest -q
.venv/bin/lexium-agro-dry-run
```

Expected test result at the current revision:

```text
30 passed
```

> These are the new offline repository tests, not the missing historical suite
> described as 20/20 in the Phase 10 transcription.

## 🗺️ Roadmap

| Milestone | State |
|---|---|
| Freeze and hash the supplied handoff | ✅ Complete |
| Offline package, safety gates, tests, typing, and CI | ✅ Complete |
| Import current Ubuntu Stage 6 / Phase 10 workspace | ⏳ Awaiting snapshot |
| Validate latest USD/URDF, meshes, frames, and mounting | ⏳ Pending |
| Reproduce deterministic Isaac Sim baselines | ⏳ Pending |
| Validate RGB-D and eye-in-hand calibration | ⏳ Pending |
| Multi-seed manipulation and sorting benchmark | ⏳ Pending |
| Audit or train a reproducible learned policy | ⏳ Conditional |
| Read-only ROS 2 simulation integration | ⏳ Planned |
| Safety-reviewed physical pilot | 🔒 Separate authorization required |

<details>
<summary><strong>📁 Repository map</strong></summary>

| Path | Purpose |
|---|---|
| [`src/lexium_agro`](src/lexium_agro) | Offline-safe domain layer and typed contracts |
| [`tests`](tests) | Tests runnable without GPU, Isaac, or ROS |
| [`configs`](configs) | Fail-closed defaults; live hardware disabled |
| [`assets/robot`](assets/robot) | Historical robot references awaiting revision validation |
| [`isaac_sim`](isaac_sim) | Simulator integration boundary |
| [`isaac_lab`](isaac_lab) | Experimental learning path; no verified policy claim |
| [`ros2`](ros2) | Planned ROS 2 Jazzy simulation adapters |
| [`legacy`](legacy) | Preserved historical sources, reports, and archives |
| [`evidence`](evidence) | Screenshots and labelled transcriptions |
| [`media`](media) | Photographs, videos, presentations, and documents |
| [`thesis`](thesis) | KBTU thesis draft and figures |
| [`docs`](docs) | Architecture, migration, safety, provenance, and validation |

</details>

<details>
<summary><strong>📦 Recover the current Ubuntu workspace</strong></summary>

The latest Stage 6/Phase 10 sources and raw reports were not present in the handoff.
On the original Ubuntu workstation, run from this repository:

```bash
python3 scripts/collect_workspace.py \
  --workspace "$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place" \
  --output "$HOME/Downloads/LEXIUM_CURRENT_WORKSPACE_CODEX.zip"
```

Review `COLLECTION_MANIFEST.json` before sharing. Current robot geometry must never
be silently replaced by older reference assets.

</details>

## 🛡️ Safety boundary

> [!CAUTION]
> This repository is **not approved for live arm or mobile-base control**. Its
> hardware backend intentionally rejects connection and motion commands.

A physical trial requires a separate reviewed procedure, verified robot/tool limits,
validated frames, a guarded workspace, independent emergency stop, competent
supervision, and explicit authorization. Read the complete
[hardware safety policy](docs/SAFETY.md).

## Team

**Sestrevatovskiy Ratmir · Jumashev Tair · Kaldybekov Adilkhan · Maytan Yerassyl**

Supervisor: **Tolengutov Rasul** · Kazakh-British Technical University (KBTU)

## Documentation

[Project specification](PROJECT_SPEC.md) ·
[Architecture](docs/ARCHITECTURE.md) ·
[Provenance](docs/STATUS_AND_PROVENANCE.md) ·
[Validation](docs/VALIDATION_REPORT.md) ·
[Contributing](CONTRIBUTING.md) ·
[Citation](CITATION.cff)

## Licensing notice

No blanket open-source license is granted for bundled CAD, photographs, videos,
historical archives, third-party templates, or vendor-related materials. Review
[LICENSE-NOTICE.md](LICENSE-NOTICE.md) before redistributing project assets.

---

<div align="center">

### Simulation first · evidence before claims · safety before motion

<sub>Lexium Agro — KBTU autonomous mobile manipulation research</sub>

</div>
