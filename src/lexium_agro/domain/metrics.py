"""Metrics with explicit denominators and evidence source labels."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class EvidenceSource(str, Enum):
    SYNTHETIC_TEST = "synthetic_test"
    RAW_SIM_REPORT = "raw_sim_report"
    TERMINAL_TRANSCRIPTION = "terminal_transcription"
    REAL_SENSOR = "real_sensor"


@dataclass(frozen=True)
class Rate:
    name: str
    numerator: int
    denominator: int

    def __post_init__(self) -> None:
        if self.denominator <= 0:
            raise ValueError("rate denominator must be positive")
        if not 0 <= self.numerator <= self.denominator:
            raise ValueError("rate numerator must be within [0, denominator]")

    @property
    def fraction(self) -> float:
        return self.numerator / self.denominator

    @property
    def percent(self) -> float:
        return 100.0 * self.fraction


@dataclass(frozen=True)
class RunCounts:
    run_id: str
    source: EvidenceSource
    seed: int
    attempted: int
    detected: int
    handled: int
    placed: int
    rejected: int

    def __post_init__(self) -> None:
        if not self.run_id.strip():
            raise ValueError("run_id must be non-empty")
        counts = (self.attempted, self.detected, self.handled, self.placed, self.rejected)
        if any(value < 0 for value in counts):
            raise ValueError("counts must be non-negative")
        if self.detected > self.attempted:
            raise ValueError("detected cannot exceed attempted")
        if self.handled > self.detected:
            raise ValueError("handled cannot exceed detected")
        if self.placed > self.handled:
            raise ValueError("placed cannot exceed handled")
        if self.rejected > self.attempted:
            raise ValueError("rejected cannot exceed attempted")

    def detection_rate(self) -> Rate | None:
        return None if self.attempted == 0 else Rate("detection", self.detected, self.attempted)

    def placement_rate_per_attempt(self) -> Rate | None:
        return None if self.attempted == 0 else Rate("placement", self.placed, self.attempted)
