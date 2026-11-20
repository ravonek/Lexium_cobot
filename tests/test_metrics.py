import pytest

from lexium_agro.domain.metrics import EvidenceSource, Rate, RunCounts


def test_rates_keep_numerator_and_denominator() -> None:
    counts = RunCounts("sim-seed-7", EvidenceSource.RAW_SIM_REPORT, 7, 5, 4, 3, 2, 1)
    assert counts.detection_rate() == Rate("detection", 4, 5)
    assert counts.placement_rate_per_attempt() == Rate("placement", 2, 5)
    assert counts.placement_rate_per_attempt().percent == 40.0  # type: ignore[union-attr]


def test_zero_attempts_do_not_create_misleading_percent() -> None:
    counts = RunCounts("empty", EvidenceSource.SYNTHETIC_TEST, 0, 0, 0, 0, 0, 0)
    assert counts.detection_rate() is None


def test_inconsistent_counts_are_rejected() -> None:
    with pytest.raises(ValueError, match="detected cannot exceed attempted"):
        RunCounts("bad", EvidenceSource.RAW_SIM_REPORT, 1, 1, 2, 0, 0, 0)
