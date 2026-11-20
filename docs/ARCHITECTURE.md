# Architecture and API boundaries

```text
                        ┌──────────────────────┐
                        │ Supervisor / Mission │
                        └──────────┬───────────┘
                                   │
                   ┌───────────────▼────────────────┐
                   │ State machine + safety monitor │
                   └───────┬──────────┬─────────────┘
                           │          │
                    RGB-D + TF        ▼
                      ┌────▼──────────┐  grasp target / bin
                      │ Perception    ├──────────────────┐
                      └────┬──────────┘                  │
                           │ camera pose                 ▼
                   ┌───────▼────────┐          ┌─────────────────────┐
                   │ Calibration TF │─────────►│ Constrained planner │
                   └────────────────┘          └─────────┬───────────┘
                                                        │
                                               ┌────────▼──────────┐
                                               │ Gripper supervisor│
                                               │ / contact / slip  │
                                               └────────┬──────────┘
                                                        │
                 ┌─────────────────┬────────────────────▼─────┐
                 │ Isaac Sim       │ ROS 2 simulation adapter│
                 │ physics/sensors │ (mobile/vision/arm/mock)│
                 └─────────────────┴──────────────────────────┘

                   [Hardware adapter: DISABLED/NOT IMPLEMENTED]
                   [Isaac Lab RL: separate experimental pipeline]
```

**Pure domain contract** (`src/lexium_agro/domain/`): immutable observations, sortable decisions, event logs, a safety-state transition engine, no Isaac/ROS imports. Production data must carry source confidence + calibrated pose.

**Simulation adapter contract** (future): create/advance/reset scene; read state/observations, send validated bounded arm/gripper targets, sample forces, report version/asset hashes. Do not assume simulator can send physical commands.

**ROS 2 adapter contract** (future): subscribers for RGB/depth + camera_info, robot joint states, tf transforms, odometry, targets; publishers/actions behind validated message schemas and E-stop interlocks. Start read-only.

**RL adapter contract** (future): normalized observations/actions and safety wrapper; no checkpoint => no learned-policy demo, fallback scripted baseline explicitly labelled.

## Metrics schema

Store `run_id`, `stage`, `source_type` (`terminal_transcription`, `raw_sim_report`, `real_sensor`, etc.), code git SHA, asset revision SHA, simulator version, seed, targets, detected, handled, placed, retries, failures, type and units of pose/force/torque. Different torque models need **distinct** keys.

## Optical projection

Camera point `(x,y,z)` computed using aligned RGB/depth and valid intrinsics in **metres**. Convert using `T_arm_base_tool × T_tool_camera × p_camera` if optical frame is attached to the tool. The hand-eye matrix is absent in this handoff and must not be guessed.
