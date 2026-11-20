import pytest

from lexium_agro.domain.models import Bin, TargetColor, TomatoObservation
from lexium_agro.domain.sorting import choose_bin, decide_batch


def test_red_green_route_to_distinct_bins():
    red = choose_bin(TomatoObservation("r", TargetColor.RED, 0.95))
    green = choose_bin(TomatoObservation("g", TargetColor.GREEN, 0.96))
    assert (red.accepted, red.target_bin) == (True, Bin.RED)
    assert (green.accepted, green.target_bin) == (True, Bin.GREEN)


def test_unknown_color_rejected():
    result = choose_bin(TomatoObservation("x", TargetColor.UNKNOWN, 0.99))
    assert not result.accepted and result.target_bin is None


def test_low_confidence_rejected():
    result = choose_bin(TomatoObservation("r", TargetColor.RED, 0.10))
    assert result.reason == "low_confidence"


def test_duplicate_id_not_counted_twice():
    obs = TomatoObservation("same", TargetColor.RED, 1.0)
    decisions = decide_batch([obs, obs])
    assert [d.accepted for d in decisions] == [True, False]
    assert decisions[-1].reason == "duplicate_id"


def test_invalid_confidence_rejected():
    with pytest.raises(ValueError):
        TomatoObservation("bad", TargetColor.RED, 1.5)
