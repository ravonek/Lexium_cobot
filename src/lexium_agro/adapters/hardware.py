"""Permanent fail-closed backend for this repository stage."""

from __future__ import annotations

from typing import NoReturn


class HardwareAccessDenied(RuntimeError):
    """Raised for every attempted physical command."""


class DisabledHardwareBackend:
    """A deliberate non-driver: connection and motion are both unreachable."""

    name = "disabled_unimplemented_hardware_backend"

    @property
    def connected(self) -> bool:
        return False

    def connect(self) -> NoReturn:
        self._deny("connect")

    def command_arm(self, _joint_targets_rad: tuple[float, ...]) -> NoReturn:
        self._deny("command_arm")

    def command_mobile_base(self, _velocity_m_s: tuple[float, float, float]) -> NoReturn:
        self._deny("command_mobile_base")

    @staticmethod
    def _deny(operation: str) -> NoReturn:
        raise HardwareAccessDenied(
            f"{operation} denied: no verified hardware interface or safety authorization"
        )
