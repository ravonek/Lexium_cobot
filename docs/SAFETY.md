# Safety and deployment gates (binding)

This repo is for **simulation and offline code development**. It is **NOT approved for live robot or mobile platform control**. Examples never connect to real Schneider controllers or drive wheels.

## Rules

1. `configs/default.yaml` always has `hardware.enabled: false` and `allow_motion: false`. Never switch this via environment-variable fallback or unreviewed CLI flag.
2. No executable path in the default package imports a physical driver or sends live motion commands.
3. Use a deliberate, separate, supervised hardware development process requiring approved risk assessment, manual/model check, collision/limit verification, independent emergency stop, guarded region, competent supervisor, manual jogging/teach testing and safe stop confirmation.
4. Force ceiling **2.80 N per finger** in the Stage 6 chart is only a simulation configuration. It is not a certified fruit-damage threshold, controller parameter, or physical force maximum.
5. Logged URDF torques up to **62.71/62.72 N·m** and hardware-oriented estimated torque **22.96 N·m** do not establish safe hardware ratings; obtain official Schneider documentation, confirmed model, payload, mounting and dynamic specs.
6. No direct transfer of simulated joint commands to real robot without unit/axis/frame testing and hardware limit enforcement on the physical controller.
7. Fail-closed on unknown transform, stale camera timestamp, high uncertainty, collision risk, singularity, unsupported controller, missing force feedback, over-limit sensor reading, object slip, unavailable E-stop and loss of network connection.
8. Physical fruit work needs an instrumented damage/force study. The current physical photos show a prototype, not completed safety validation.
9. Credentials, wifi passwords, API tokens and absolute personal paths must be absent from committed code and shared ZIPs (the original user archives are historical and should be privacy-reviewed before public push).

## Safety enablement checklist (no automatic activation)

- [ ] Controller firmware/model/official rated joint limits reviewed.
- [ ] Tool payload, center of mass, mounting orientation and physical workspace defined.
- [ ] Mechanical stops, collision zones, protective stop and independent E-stop tested.
- [ ] Joint coordinate mapping and forward/inverse kinematics verified at rest.
- [ ] Camera/tool calibration verified; target limits and keepouts approved.
- [ ] Gripper actuator limits and sensor calibration verified.
- [ ] Mobile platform can lock/stabilize and safely stop.
- [ ] Explicit authorized, supervised physical test plan signed off.
