"""Pinhole RGB-D geometry with explicit metre units and rejection checks."""

from __future__ import annotations

import math
from dataclasses import dataclass


class InvalidDepth(ValueError):
    """Raised for unusable or out-of-range depth observations."""


@dataclass(frozen=True)
class CameraIntrinsics:
    width_px: int
    height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px)
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("image dimensions must be positive")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("intrinsics must be finite")
        if self.fx_px <= 0 or self.fy_px <= 0:
            raise ValueError("focal lengths must be positive")


@dataclass(frozen=True)
class Point3D:
    x_m: float
    y_m: float
    z_m: float
    frame: str


def deproject_pixel(
    u_px: float,
    v_px: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    *,
    min_depth_m: float = 0.1,
    max_depth_m: float = 3.0,
    frame: str = "camera_depth_optical_frame",
) -> Point3D:
    if not all(math.isfinite(value) for value in (u_px, v_px, depth_m)):
        raise InvalidDepth("pixel and depth values must be finite")
    if not 0 <= u_px < intrinsics.width_px or not 0 <= v_px < intrinsics.height_px:
        raise InvalidDepth("pixel is outside the calibrated image")
    if min_depth_m <= 0 or max_depth_m <= min_depth_m:
        raise ValueError("depth range must satisfy 0 < min < max")
    if not min_depth_m <= depth_m <= max_depth_m:
        raise InvalidDepth("depth is outside the configured metric range")
    if not frame.strip():
        raise ValueError("frame must be non-empty")
    return Point3D(
        x_m=(u_px - intrinsics.cx_px) * depth_m / intrinsics.fx_px,
        y_m=(v_px - intrinsics.cy_px) * depth_m / intrinsics.fy_px,
        z_m=depth_m,
        frame=frame,
    )


def validate_timestamp(*, acquired_s: float, now_s: float, max_age_s: float) -> None:
    if not all(math.isfinite(value) for value in (acquired_s, now_s, max_age_s)):
        raise ValueError("timestamps must be finite")
    if max_age_s <= 0:
        raise ValueError("max_age_s must be positive")
    age = now_s - acquired_s
    if age < 0:
        raise ValueError("acquisition timestamp is in the future")
    if age > max_age_s:
        raise ValueError(f"RGB-D observation is stale: age={age:.6f}s")
