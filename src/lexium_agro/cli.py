"""Synthetic offline example, deliberately *not* Isaac Sim / Isaac Lab or hardware."""
import json
from .domain.models import TargetColor, TomatoObservation
from .domain.sorting import decide_batch
from .domain.safety import SafetyGate


def main():
    synthetic = [
        TomatoObservation("toy-01", TargetColor.RED, 0.99),
        TomatoObservation("toy-02", TargetColor.GREEN, 0.97),
        TomatoObservation("toy-03", TargetColor.UNKNOWN, 0.91),
    ]
    output = {
        "source": "SYNTHETIC_OFFLINE_EXAMPLE_NOT_EXPERIMENT",
        "hardware": SafetyGate().report(),
        "decisions": [d.__dict__ for d in decide_batch(synthetic)],
    }
    print(json.dumps(output, indent=2, default=lambda v: getattr(v, "value", str(v))))


if __name__ == "__main__":
    main()
