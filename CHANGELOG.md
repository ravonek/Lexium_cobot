# Changelog

All notable repository-level changes are documented here. Historical experiment
results remain in their original provenance files and are not rewritten as releases.

## [Unreleased]

### Pending

- Import of the current Ubuntu Stage 6/Phase 10 workspace and raw reports.
- Reproduction against the installed Isaac Sim and Isaac Lab versions.

## [0.1.0] — 2026-11-20

### Added

- Verified handoff inventory and nested archive integrity checks.
- Dependency-light offline package with fail-closed configuration.
- Sorting decisions, state machine, metrics, RGB-D geometry, rigid transforms,
  deterministic seed handling, and structured logging.
- Permanently disabled physical hardware backend.
- Offline CI for Python 3.10 and 3.11.
- Documentation for architecture, provenance, safety, migration, and validation.
- Preserved historical code, reports, media, and thesis artifacts.

### Verified

- 214/214 declared handoff files present before repository changes.
- Four bundled source archives passed integrity checks.
- 30 offline tests passed; Ruff and strict mypy checks passed.

### Not claimed

- No Isaac Sim, Isaac Lab, ROS 2, camera, or physical-robot process was run as part
  of this release.
- No verified learned policy or autonomous physical harvesting result is claimed.
