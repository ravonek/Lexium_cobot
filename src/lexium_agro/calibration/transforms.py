"""Rigid transforms using an explicit quaternion [x, y, z, w] convention."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class QuaternionXYZW:
    """Quaternion fields in [x, y, z, w] order despite the class name."""

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        values = (self.x, self.y, self.z, self.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("quaternion must be finite")
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("quaternion must be normalized")

    def conjugate(self) -> QuaternionXYZW:
        return QuaternionXYZW(-self.x, -self.y, -self.z, self.w)

    def multiply(self, other: QuaternionXYZW) -> QuaternionXYZW:
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        return QuaternionXYZW(x / norm, y / norm, z / norm, w / norm)

    def rotate(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        px, py, pz = point
        q_point = (self.x, self.y, self.z)
        dot = q_point[0] * px + q_point[1] * py + q_point[2] * pz
        cross = (
            q_point[1] * pz - q_point[2] * py,
            q_point[2] * px - q_point[0] * pz,
            q_point[0] * py - q_point[1] * px,
        )
        scale = self.w * self.w - sum(value * value for value in q_point)
        return (
            scale * px + 2.0 * dot * self.x + 2.0 * self.w * cross[0],
            scale * py + 2.0 * dot * self.y + 2.0 * self.w * cross[1],
            scale * pz + 2.0 * dot * self.z + 2.0 * self.w * cross[2],
        )


@dataclass(frozen=True)
class RigidTransform:
    """T_parent_child maps child-frame coordinates into the parent frame."""

    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    rotation_xyzw: QuaternionXYZW

    def __post_init__(self) -> None:
        if not self.parent_frame.strip() or not self.child_frame.strip():
            raise ValueError("frame names must be non-empty")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent and child frames must differ")
        if len(self.translation_m) != 3 or not all(
            math.isfinite(value) for value in self.translation_m
        ):
            raise ValueError("translation must contain three finite metre values")

    def apply(self, point_child_m: tuple[float, float, float]) -> tuple[float, float, float]:
        rotated = self.rotation_xyzw.rotate(point_child_m)
        return (
            rotated[0] + self.translation_m[0],
            rotated[1] + self.translation_m[1],
            rotated[2] + self.translation_m[2],
        )

    def inverse(self) -> RigidTransform:
        inverse_rotation = self.rotation_xyzw.conjugate()
        inverse_translation = inverse_rotation.rotate(
            (-self.translation_m[0], -self.translation_m[1], -self.translation_m[2])
        )
        return RigidTransform(
            self.child_frame,
            self.parent_frame,
            inverse_translation,
            inverse_rotation,
        )

    def compose(self, child: RigidTransform) -> RigidTransform:
        if self.child_frame != child.parent_frame:
            raise ValueError(
                f"frame mismatch: {self.child_frame} != {child.parent_frame}"
            )
        return RigidTransform(
            parent_frame=self.parent_frame,
            child_frame=child.child_frame,
            translation_m=self.apply(child.translation_m),
            rotation_xyzw=self.rotation_xyzw.multiply(child.rotation_xyzw),
        )
