# Verification & acceptance matrix

| ID | Scope | Procedure/evidence | Acceptance |
|---|---|---|---|
| OFF-01 | Offline package | `python -m pip install -e '.[dev]'`, `pytest -q` | All NEW scaffold tests pass; clear test count |
| OFF-02 | Formatting/typing | import check / optional lint | No missing optional Isaac/ROS imports during offline tests |
| DATA-01 | Provenance | Manifest hashes and declared raw/transcribed source | Source, version and uncertainty recorded |
| MOD-01 | Asset mapping | Joint list, forward kinematics, check URDF/STL/USD/XRDF | Link meshes and base orientation verified against actual latest model |
| MOD-02 | Mechanics | Kinematic workspace, collision & limits + time-parameterized motion | Run produces raw logs with units and thresholds defined by authentic model |
| PER-01 | RGB-D | Independent scenes with ground truth | Error distribution; reject invalid depth, object overlap/occlusion |
| CAL-01 | Eye-in-hand | Multiple calibration poses and independent held-out targets | Matrix and residuals; no hard-coded guess |
| GRP-01 | Contact | Four finger force traces + sensor calibration | Contact states and per-sensor range reported, compare to separate simulation limits |
| SORT-01 | Stage 6 | Recover source and rerun five-object scene | Full attempt/detection/pick/place counts; success definition documented |
| VAL-01 | Phase 10 A-E | Recover source/tests/raw JSON, reproduce on same config | Compare 5 case outcomes and per-metric tolerances justified |
| RL-01 | Isaac Lab | Inspect train config + checkpoint + rollouts | RL only reported after reproducible training and evaluation |
| ROS-01 | Bridge | Read-only pub/sub, TF, timestamps, simulation command adapter | No live controller connections; repeatable integration tests |
| HW-01 | Hardware | Separate manual risk-approved protocol | Not allowed in this starter; must complete independent checklist |

## Original reported Phase 10 summary (historical, not rerun here)

- `overall_pass=True`, 5/5 cases.
- `MOTION_VALIDATE_COMPLETE valid=True samples=10717 max_vel=0.593rad/s max_urdf_tq=62.71Nm max_hw_tq=22.96Nm min_marg=29.98deg plots=6`
- `UNIT_TEST_SUMMARY passed=20 failed=0`
- `DELIVERABLES_VERIFY verified_pass=True sha256=21/21 missing=0 shots=8`

Do not replace this line with your new offline scaffold `pytest` result; the latter is a separate suite.
