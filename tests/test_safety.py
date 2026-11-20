from lexium_agro.domain.safety import SafetyGate


def test_default_hardware_always_disabled():
    gate = SafetyGate()
    assert not gate.can_issue_real_motion()
    assert gate.report()["hardware_enabled"] is False


def test_even_flags_cannot_enable_hardware_in_starter():
    gate = SafetyGate(True, True, True, True, True)
    assert not gate.can_issue_real_motion()
