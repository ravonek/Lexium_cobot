import pytest

from lexium_agro.domain.state_machine import InvalidTransition, OperationState, StateMachine


def test_nominal_prefix_records_ordered_events() -> None:
    machine = StateMachine()
    event = machine.transition(OperationState.READY, timestamp_s=1.0, reason="offline ready")
    machine.transition(OperationState.ACQUIRE, timestamp_s=2.0, reason="synthetic input")
    assert event.sequence == 1
    assert machine.state is OperationState.ACQUIRE
    assert [item.sequence for item in machine.events] == [1, 2]


def test_invalid_transition_has_no_side_effect() -> None:
    machine = StateMachine()
    with pytest.raises(InvalidTransition):
        machine.transition(OperationState.GRASP, timestamp_s=1.0, reason="skip gates")
    assert machine.state is OperationState.SAFE_START
    assert machine.events == ()


def test_estop_is_reachable_and_latched() -> None:
    machine = StateMachine()
    machine.transition(OperationState.ESTOP, timestamp_s=1.0, reason="external stop")
    with pytest.raises(InvalidTransition):
        machine.transition(OperationState.SAFE_START, timestamp_s=2.0, reason="unsafe reset")
