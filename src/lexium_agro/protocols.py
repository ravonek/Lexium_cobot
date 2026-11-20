"""Typed integration boundaries; implementations live outside the pure domain."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class SimulationSnapshot:
    simulation_time_s: float
    joint_positions_rad: tuple[float, ...]
    contact_forces_n: tuple[float, ...]
    seed: int


class SimulationAdapter(Protocol):
    """Simulation-only interface. It is not compatible with a robot driver."""

    @property
    def backend_version(self) -> str: ...

    def reset(self, *, seed: int) -> SimulationSnapshot: ...

    def step(self, joint_targets_rad: tuple[float, ...]) -> SimulationSnapshot: ...


class ReadOnlyRgbdSource(Protocol):
    @property
    def frame_id(self) -> str: ...

    def latest_timestamp_s(self) -> float: ...
