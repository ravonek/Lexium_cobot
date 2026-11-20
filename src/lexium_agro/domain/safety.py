"""Pure safety gating. Does not enable any physical actuator."""
from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyGate:
    hardware_enabled: bool = False
    motion_authorized: bool = False
    emergency_stop_clear: bool = False
    transforms_verified: bool = False
    controller_limits_verified: bool = False

    def can_issue_real_motion(self) -> bool:
        # A programmatic gate is insufficient for real hardware: this starter ALWAYS rejects.
        return False

    def report(self) -> dict:
        return {
            "hardware_enabled": self.hardware_enabled,
            "motion_authorized": self.motion_authorized,
            "can_issue_real_motion": False,
            "reason": "starter_has_no_verified_physical_driver_or_safety_approval",
        }
