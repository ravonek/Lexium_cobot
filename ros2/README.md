# ROS 2 — planned hardware-agnostic integration

Target distribution inferred from log: ROS 2 Jazzy. The supplied log contains a successful publisher/subscriber roundtrip test; no live full pipeline is provided here. Codex should create ament-compatible packages only after checking the Ubuntu ROS environment. Start with simulator/read-only adapters for `sensor_msgs/Image`, `CameraInfo`, `JointState`, `tf2`, gripper state and mobile `/odom`; do not implement physical command publishers by default.
