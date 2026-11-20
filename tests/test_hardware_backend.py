import pytest

from lexium_agro.adapters.hardware import DisabledHardwareBackend, HardwareAccessDenied


def test_hardware_backend_denies_every_command() -> None:
    backend = DisabledHardwareBackend()
    assert backend.connected is False
    with pytest.raises(HardwareAccessDenied):
        backend.connect()
    with pytest.raises(HardwareAccessDenied):
        backend.command_arm((0.0,) * 6)
    with pytest.raises(HardwareAccessDenied):
        backend.command_mobile_base((0.0, 0.0, 0.0))
