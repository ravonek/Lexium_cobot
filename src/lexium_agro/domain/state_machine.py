"""Deterministic orchestration state machine with no actuator side effects."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class OperationState(str, Enum):
    SAFE_START = "SAFE_START"
    READY = "READY"
    ACQUIRE = "ACQUIRE"
    LOCALIZE = "LOCALIZE"
    PLAN = "PLAN"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    PLACE = "PLACE"
    VERIFY = "VERIFY"
    COMPLETE = "COMPLETE"
    REJECT = "REJECT"
    RECOVER = "RECOVER"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


_ALLOWED: dict[OperationState, frozenset[OperationState]] = {
    OperationState.SAFE_START: frozenset({OperationState.READY, OperationState.FAULT}),
    OperationState.READY: frozenset({OperationState.ACQUIRE, OperationState.FAULT}),
    OperationState.ACQUIRE: frozenset(
        {OperationState.LOCALIZE, OperationState.REJECT, OperationState.FAULT}
    ),
    OperationState.LOCALIZE: frozenset(
        {OperationState.PLAN, OperationState.REJECT, OperationState.FAULT}
    ),
    OperationState.PLAN: frozenset(
        {OperationState.APPROACH, OperationState.REJECT, OperationState.FAULT}
    ),
    OperationState.APPROACH: frozenset(
        {OperationState.GRASP, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.GRASP: frozenset(
        {OperationState.LIFT, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.LIFT: frozenset(
        {OperationState.TRANSPORT, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.TRANSPORT: frozenset(
        {OperationState.PLACE, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.PLACE: frozenset(
        {OperationState.VERIFY, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.VERIFY: frozenset(
        {OperationState.COMPLETE, OperationState.RECOVER, OperationState.FAULT}
    ),
    OperationState.COMPLETE: frozenset({OperationState.READY}),
    OperationState.REJECT: frozenset({OperationState.RECOVER, OperationState.FAULT}),
    OperationState.RECOVER: frozenset({OperationState.READY, OperationState.FAULT}),
    OperationState.FAULT: frozenset({OperationState.SAFE_START}),
    OperationState.ESTOP: frozenset(),
}


class InvalidTransition(ValueError):
    """Raised before state changes when a transition is not declared."""


@dataclass(frozen=True)
class TransitionEvent:
    sequence: int
    timestamp_s: float
    previous: OperationState
    current: OperationState
    reason: str


class StateMachine:
    def __init__(self) -> None:
        self._state = OperationState.SAFE_START
        self._events: list[TransitionEvent] = []

    @property
    def state(self) -> OperationState:
        return self._state

    @property
    def events(self) -> tuple[TransitionEvent, ...]:
        return tuple(self._events)

    def transition(
        self, target: OperationState, *, timestamp_s: float, reason: str
    ) -> TransitionEvent:
        if not math.isfinite(timestamp_s) or timestamp_s < 0:
            raise ValueError("timestamp_s must be finite and non-negative")
        if not reason.strip():
            raise ValueError("transition reason must be non-empty")
        if target is not OperationState.ESTOP and target not in _ALLOWED[self._state]:
            raise InvalidTransition(f"{self._state.value} -> {target.value} is not allowed")
        previous = self._state
        event = TransitionEvent(len(self._events) + 1, timestamp_s, previous, target, reason)
        self._events.append(event)
        self._state = target
        return event
