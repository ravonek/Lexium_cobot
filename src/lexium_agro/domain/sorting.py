"""Conservative decisions only: not an actuator or a real autonomous sorter."""
from .models import Bin, SortDecision, TargetColor, TomatoObservation


def choose_bin(observation: TomatoObservation, min_confidence: float = 0.90) -> SortDecision:
    if not 0 <= min_confidence <= 1:
        raise ValueError("min_confidence must be in [0, 1]")
    if observation.confidence < min_confidence:
        return SortDecision(observation.object_id, False, None, "low_confidence")
    if observation.color == TargetColor.RED:
        return SortDecision(observation.object_id, True, Bin.RED, "classified_red")
    if observation.color == TargetColor.GREEN:
        return SortDecision(observation.object_id, True, Bin.GREEN, "classified_green")
    return SortDecision(observation.object_id, False, None, "unknown_color")


def decide_batch(observations: list[TomatoObservation], min_confidence: float = 0.90) -> list[SortDecision]:
    seen: set[str] = set()
    decisions: list[SortDecision] = []
    for obs in observations:
        if obs.object_id in seen:
            decisions.append(SortDecision(obs.object_id, False, None, "duplicate_id"))
        else:
            decisions.append(choose_bin(obs, min_confidence))
            seen.add(obs.object_id)
    return decisions
