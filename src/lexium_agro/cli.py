"""Synthetic offline example, deliberately *not* Isaac Sim / Isaac Lab or hardware."""

import argparse
import json
from pathlib import Path

from .config import load_offline_config
from .domain.models import TargetColor, TomatoObservation
from .domain.safety import SafetyGate
from .domain.sorting import decide_batch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "configs" / "default.yaml",
    )
    args = parser.parse_args()
    config = load_offline_config(args.config)
    synthetic = [
        TomatoObservation("toy-01", TargetColor.RED, 0.99),
        TomatoObservation("toy-02", TargetColor.GREEN, 0.97),
        TomatoObservation("toy-03", TargetColor.UNKNOWN, 0.91),
    ]
    output = {
        "source": "SYNTHETIC_OFFLINE_EXAMPLE_NOT_EXPERIMENT",
        "mode": config.mode,
        "hardware": SafetyGate().report(),
        "decisions": [d.__dict__ for d in decide_batch(synthetic)],
    }
    print(json.dumps(output, indent=2, default=lambda v: getattr(v, "value", str(v))))


if __name__ == "__main__":
    main()
