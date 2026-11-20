"""Domain objects: synthetic and observed targets are explicitly differentiated."""
from dataclasses import dataclass
from enum import Enum


class TargetColor(str, Enum):
    RED = "red"
    GREEN = "green"
    UNKNOWN = "unknown"


class Bin(str, Enum):
    RED = "bin_red"
    GREEN = "bin_green"


@dataclass(frozen=True)
class TomatoObservation:
    object_id: str
    color: TargetColor
    confidence: float
    source: str = "synthetic_test"

    def __post_init__(self) -> None:
        if not self.object_id.strip():
            raise ValueError("object_id must be a non-empty string")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if self.source not in {"synthetic_test", "rgbd", "simulated_sensor"}:
            raise ValueError("source must be declared explicitly")


@dataclass(frozen=True)
class SortDecision:
    object_id: str
    accepted: bool
    target_bin: Bin | None
    reason: str
