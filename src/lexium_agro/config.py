"""Fail-closed configuration loader for the offline package."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a configuration is unsafe or unsupported."""


def _require_bool(mapping: Mapping[str, Any], key: str) -> bool:
    value = mapping.get(key)
    if type(value) is not bool:
        raise ConfigError(f"{key} must be a boolean")
    return value


@dataclass(frozen=True)
class HardwareConfig:
    enabled: bool
    allow_motion: bool
    stop_on_unknown_state: bool
    controller_endpoint: str | None

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> HardwareConfig:
        endpoint = value.get("controller_endpoint")
        if endpoint is not None and not isinstance(endpoint, str):
            raise ConfigError("hardware.controller_endpoint must be null or a string")
        result = cls(
            enabled=_require_bool(value, "enabled"),
            allow_motion=_require_bool(value, "allow_motion"),
            stop_on_unknown_state=_require_bool(value, "stop_on_unknown_state"),
            controller_endpoint=endpoint,
        )
        if result.enabled or result.allow_motion:
            raise ConfigError("this repository cannot enable or authorize physical motion")
        if not result.stop_on_unknown_state:
            raise ConfigError("hardware.stop_on_unknown_state must remain true")
        return result


@dataclass(frozen=True)
class OfflineConfig:
    project: str
    mode: str
    simulator_backend: str
    hardware: HardwareConfig


def load_offline_config(path: Path) -> OfflineConfig:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot read configuration {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("configuration root must be a mapping")

    project = raw.get("project")
    mode = raw.get("mode")
    simulator = raw.get("simulator")
    hardware = raw.get("hardware")
    if project != "lexium_agro":
        raise ConfigError("project must be lexium_agro")
    if mode != "offline_dry_run":
        raise ConfigError("offline CLI supports only mode=offline_dry_run")
    if not isinstance(simulator, dict) or simulator.get("backend") != "none":
        raise ConfigError("offline CLI requires simulator.backend=none")
    if not isinstance(hardware, dict):
        raise ConfigError("hardware must be a mapping")
    return OfflineConfig(project, mode, "none", HardwareConfig.from_mapping(hardware))
