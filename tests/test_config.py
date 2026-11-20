from pathlib import Path

import pytest

from lexium_agro.config import ConfigError, load_offline_config


def test_default_configuration_is_offline_and_disabled() -> None:
    config = load_offline_config(Path("configs/default.yaml"))
    assert config.mode == "offline_dry_run"
    assert config.simulator_backend == "none"
    assert config.hardware.enabled is False
    assert config.hardware.allow_motion is False


@pytest.mark.parametrize("field", ["enabled", "allow_motion"])
def test_configuration_cannot_enable_hardware(tmp_path: Path, field: str) -> None:
    content = Path("configs/default.yaml").read_text(encoding="utf-8")
    content = content.replace(f"{field}: false", f"{field}: true")
    path = tmp_path / "unsafe.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(ConfigError, match="cannot enable"):
        load_offline_config(path)
