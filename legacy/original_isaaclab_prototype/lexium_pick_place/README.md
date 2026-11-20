# Schneider Lexium Cobot RL Pick-and-Place (Isaac Lab 3.0)

Reinforcement learning pick-and-place implementation for the 6-DoF Schneider Electric Lexium Cobot arm equipped with a coupled 4-finger gripper. Built on NVIDIA Isaac Sim 6.1.0, Isaac Lab v3.0.0-EA, and RSL-RL 5.4.1 using the manager-based workflow and PPO.

---

## 1. URDF Joint Limits Reference Table

The joint limits below are directly extracted from `lexium_cobot.urdf` and enforced in simulation[cite: 2]:

| Joint Name | Parent Link | Child Link | Axis (URDF) | Pos Min (rad) | Pos Max (rad) | Max Vel (rad/s) | Max Effort (Nm) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `joint_1` | `base_link` | `link1_1` | `[0.0, -1.0, 0.0]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `joint_2` | `link1_1` | `link2_1` | `[-0.674302, 0.0, -0.738455]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `joint_3` | `link2_1` | `link3_1` | `[-0.674302, 0.0, -0.738455]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `joint_4` | `link3_1` | `link4_1` | `[0.674302, -0.0, 0.738455]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `joint_5` | `link4_1` | `LINK5_1` | `[-0.577921, -0.622515, 0.527715]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `joint_6_gripper_rotation` | `LINK5_1` | `gripper_base_1` | `[0.13773, 0.562961, 0.814926]`[cite: 2] | `-3.141593`[cite: 2] | `+3.141593`[cite: 2] | `1.0`[cite: 2] | `100.0`[cite: 2] |
| `gripper_Revolute_1` | `gripper_base_1` | `gripper_finger_1_1` | `[-0.577921, -0.622515, 0.527715]`[cite: 2] | `-1.780236`[cite: 2] | `+0.130900`[cite: 2] | `100.0`[cite: 2] | `100.0`[cite: 2] |
| `gripper_Revolute_2` | `gripper_base_1` | `gripper_finger_2_1` | `[-0.577921, -0.622515, 0.527715]`[cite: 2] | `-1.570796`[cite: 2] | `+0.340339`[cite: 2] | `100.0`[cite: 2] | `100.0`[cite: 2] |
| `gripper_Revolute_5` | `gripper_base_1` | `gripper_finger_3_1` | `[0.577921, 0.622515, -0.527715]`[cite: 2] | `-0.349066`[cite: 2] | `+1.570796`[cite: 2] | `100.0`[cite: 2] | `100.0`[cite: 2] |
| `gripper_Revolute_6` | `gripper_base_1` | `gripper_finger_4_1` | `[0.577921, 0.622515, -0.527715]`[cite: 2] | `-0.340339`[cite: 2] | `+1.570796`[cite: 2] | `100.0`[cite: 2] | `100.0`[cite: 2] |

---

## 2. Setup & Asset Organization

Ensure the mesh files are placed in `lexium_pick_place/assets/meshes/` relative to `lexium_cobot.urdf`:

```text
lexium_pick_place/
├── assets/
│   ├── lexium_cobot.urdf
│   ├── lexium_cobot.xrdf
│   ├── meshes/
│   │   ├── base_link.stl
│   │   ├── link1_1.stl
│   │   ├── link2_1.stl
│   │   ├── link3_1.stl
│   │   ├── link4_1.stl
│   │   ├── LINK5_1.stl
│   │   ├── gripper_base_1.stl
│   │   ├── gripper_finger_1_1.stl
│   │   ├── gripper_finger_2_1.stl
│   │   ├── gripper_finger_3_1.stl
│   │   └── gripper_finger_4_1.stl
│   └── usd/

```

---

## 3. Execution Commands (Isaac Lab 3.0 Workflow)

### A. Robot Articulation & Orientation Inspection

Spawns the standalone robot to verify link names, joint limits, mounting orientation (+90° about X), and TCP position:

```bash
# Using Isaac Lab CLI (uv run)
isaaclab -p lexium_pick_place/check_robot.py

# Or via bash launcher script
./isaaclab.sh -p lexium_pick_place/check_robot.py

# Windows Command Prompt
isaaclab.bat -p lexium_pick_place/check_robot.py

```

### B. Fast Debug Training (64 Environments)

```bash
isaaclab -p lexium_pick_place/train.py --debug --num_envs 64 --headless

```

### C. Full Production Training (4096 Environments)

```bash
isaaclab -p lexium_pick_place/train.py --num_envs 4096 --headless

```

### D. Policy Evaluation & Playback

```bash
isaaclab -p lexium_pick_place/play.py --num_envs 16 --checkpoint /path/to/model.pt

```

---

## 4. Key Tunable Parameters

1. **Gripper Control (`mdp/actions.py`):**
* `binary_mode`: Set to `True` for discrete open/close switching or `False` for continuous scalar interpolation.


2. **Physics Material (`robot_cfg.py`):**
* `static_friction`: `1.2`, `dynamic_friction`: `1.1` (applied to fingertips and manipuland cube).


3. **Solver Iterations (`pick_place_env_cfg.py`):**
* `solver_position_iteration_count`: `16`, `solver_velocity_iteration_count`: `2`.


4. **Reward Weights (`pick_place_env_cfg.py`):**
* `reach`: `2.0`
* `grasp`: `4.0`
* `lift`: `10.0`
* `transport`: `15.0`
* `place`: `25.0`
* `joint_limit_proximity`: `-1.0`
* `collision_penalty`: `-5.0`
* `action_rate`: `-1e-4` $\to$ `-1e-2` (curriculum)
* `joint_vel`: `-1e-4` $\to$ `-1e-2` (curriculum)
