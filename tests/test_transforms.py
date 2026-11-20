import math

import pytest

from lexium_agro.calibration.transforms import QuaternionXYZW, RigidTransform

IDENTITY = QuaternionXYZW(0.0, 0.0, 0.0, 1.0)


def test_transform_round_trip() -> None:
    transform = RigidTransform("arm_base", "camera", (1.0, -2.0, 0.5), IDENTITY)
    point = (0.2, 0.3, 1.0)
    recovered = transform.inverse().apply(transform.apply(point))
    assert recovered == pytest.approx(point)


def test_composition_checks_frame_chain() -> None:
    arm_tool = RigidTransform("arm_base", "tool0", (1.0, 0.0, 0.0), IDENTITY)
    tool_camera = RigidTransform("tool0", "camera", (0.0, 2.0, 0.0), IDENTITY)
    arm_camera = arm_tool.compose(tool_camera)
    assert arm_camera.translation_m == pytest.approx((1.0, 2.0, 0.0))
    assert arm_camera.child_frame == "camera"


def test_rotation_uses_xyzw_convention() -> None:
    half = math.sqrt(0.5)
    rotation = QuaternionXYZW(0.0, 0.0, half, half)
    assert rotation.rotate((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))


def test_non_normalized_quaternion_is_rejected() -> None:
    with pytest.raises(ValueError, match="normalized"):
        QuaternionXYZW(0.0, 0.0, 0.0, 2.0)
