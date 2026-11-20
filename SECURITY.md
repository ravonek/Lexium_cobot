# Security policy

## Supported scope

The current repository supports offline-safe code and documentation. Isaac, ROS,
camera, network, and physical-controller integrations are not treated as production
or safety-certified interfaces.

## Reporting a vulnerability

Please do not publish credentials, controller addresses, unsafe motion paths, or
details that could expose a connected lab system in a public issue. Contact the
repository owner privately through their GitHub profile and include:

- the affected revision and file;
- reproduction steps that do not command physical hardware;
- expected and observed behavior;
- potential safety or data impact.

## Operational rules

- Never commit API keys, Wi-Fi credentials, SSH keys, controller secrets, or tokens.
- Keep local hardware configuration in ignored `configs/hardware_local*.yaml` files.
- Do not connect this repository to a physical controller by default.
- Treat unknown transforms, limits, timestamps, or E-stop state as a hard failure.
- Rotate any secret immediately if it is accidentally exposed.
