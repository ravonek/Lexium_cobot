import pytest

from lexium_agro.perception.rgbd import (
    CameraIntrinsics,
    InvalidDepth,
    deproject_pixel,
    validate_timestamp,
)

INTRINSICS = CameraIntrinsics(640, 480, 600.0, 600.0, 320.0, 240.0)


def test_principal_point_deprojects_along_optical_axis() -> None:
    point = deproject_pixel(320.0, 240.0, 1.2, INTRINSICS)
    assert (point.x_m, point.y_m, point.z_m) == (0.0, 0.0, 1.2)


@pytest.mark.parametrize("depth", [0.0, float("nan"), 4.0])
def test_invalid_depth_is_rejected(depth: float) -> None:
    with pytest.raises(InvalidDepth):
        deproject_pixel(320.0, 240.0, depth, INTRINSICS)


def test_stale_timestamp_is_rejected() -> None:
    with pytest.raises(ValueError, match="stale"):
        validate_timestamp(acquired_s=1.0, now_s=2.0, max_age_s=0.5)
