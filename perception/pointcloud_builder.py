"""Utilities for constructing point clouds from DepthAnything depth maps.

This module converts single-frame depth estimates into metric point clouds
expressed in the camera frame (and optionally into another target frame,
such as the robot base).  It handles confidence gating, arbitrary regions of
interest, and extrinsic transformations.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics.

    Attributes:
        fx: Focal length along the x-axis, in pixels.
        fy: Focal length along the y-axis, in pixels.
        cx: Principal point x-coordinate, in pixels.
        cy: Principal point y-coordinate, in pixels.
        width: Image width, in pixels.
        height: Image height, in pixels.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def validate_shape(self, shape: Tuple[int, int]) -> None:
        """Ensure the incoming image/depth shape matches this model."""
        if shape != (self.height, self.width):
            raise ValueError(
                f"Depth shape {shape} does not match intrinsics "
                f"({self.height}, {self.width})"
            )


@dataclass(frozen=True)
class PointCloudResult:
    """Container for per-frame point cloud outputs."""

    points_camera: np.ndarray  # shape: (N, 3)
    points_target: Optional[np.ndarray]  # shape: (N, 3)
    pixel_x: np.ndarray  # integer pixel x positions, shape: (N,)
    pixel_y: np.ndarray  # integer pixel y positions, shape: (N,)
    depth: np.ndarray  # metric depth values (after scaling), shape: (N,)
    confidence: Optional[np.ndarray]  # per-point confidence in [0, 1]

    @property
    def count(self) -> int:
        return int(self.points_camera.shape[0])

    def empty(self) -> bool:
        return self.count == 0


def build_pointcloud(
    depth_map: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_scale: float = 1.0,
    roi: Optional[Tuple[int, int, int, int]] = None,
    mask: Optional[np.ndarray] = None,
    confidence: Optional[np.ndarray] = None,
    min_confidence: float = 0.0,
    extrinsics: Optional[np.ndarray] = None,
) -> PointCloudResult:
    """Lift a depth map into a point cloud.

    Args:
        depth_map: Floating-point depth array with shape (H, W).  Units are
            arbitrary; `depth_scale` converts them into metres (or any metric
            unit required by the caller).
        intrinsics: Pinhole camera intrinsics describing the input frame.
        depth_scale: Conversion multiplier applied to raw depth values.
        roi: Optional region of interest `(x_min, y_min, x_max, y_max)` in pixel
            coordinates.  Pixels outside the ROI are discarded.
        mask: Optional boolean mask aligned with the depth map.  Only pixels
            where the mask is true are retained.
        confidence: Optional float array matching `depth_map` that denotes model
            confidence in `[0, 1]`.  Points below `min_confidence` are dropped.
        min_confidence: Threshold applied to the `confidence` map.
        extrinsics: Optional homogeneous transform (4×4) that maps camera-frame
            points into another target frame (e.g., robot base).  If provided,
            the result additionally contains `points_target`.

    Returns:
        `PointCloudResult` containing arrays of valid points.
    """

    if depth_map.ndim != 2:
        raise ValueError(f"Expected 2D depth map, received shape {depth_map.shape}")

    depth = np.asarray(depth_map, dtype=np.float32)
    intrinsics.validate_shape(depth.shape)

    if roi is not None:
        x_min, y_min, x_max, y_max = _canonicalise_roi(roi, intrinsics.width, intrinsics.height)
    else:
        x_min, y_min, x_max, y_max = 0, 0, intrinsics.width, intrinsics.height

    base_mask = np.isfinite(depth)
    base_mask &= depth > 0.0

    # Apply ROI by zeroing pixels outside the bounding box.
    if (x_min, y_min) != (0, 0) or (x_max, y_max) != (intrinsics.width, intrinsics.height):
        roi_mask = np.zeros_like(base_mask, dtype=bool)
        roi_mask[y_min:y_max, x_min:x_max] = True
        base_mask &= roi_mask

    if mask is not None:
        if mask.shape != depth.shape:
            raise ValueError(f"Mask shape {mask.shape} does not match depth shape {depth.shape}")
        base_mask &= mask.astype(bool)

    if confidence is not None:
        if confidence.shape != depth.shape:
            raise ValueError(
                f"Confidence shape {confidence.shape} does not match depth shape {depth.shape}"
            )
        base_mask &= confidence >= float(min_confidence)

    valid_indices = np.nonzero(base_mask)
    if valid_indices[0].size == 0:
        return PointCloudResult(
            points_camera=np.zeros((0, 3), dtype=np.float32),
            points_target=None if extrinsics is None else np.zeros((0, 3), dtype=np.float32),
            pixel_x=np.zeros((0,), dtype=np.int32),
            pixel_y=np.zeros((0,), dtype=np.int32),
            depth=np.zeros((0,), dtype=np.float32),
            confidence=None if confidence is None else np.zeros((0,), dtype=np.float32),
        )

    ys, xs = valid_indices
    depth_values = depth[ys, xs] * float(depth_scale)

    x_norm = (xs.astype(np.float32) - float(intrinsics.cx)) / float(intrinsics.fx)
    y_norm = (ys.astype(np.float32) - float(intrinsics.cy)) / float(intrinsics.fy)

    points_camera = np.column_stack(
        (
            x_norm * depth_values,
            y_norm * depth_values,
            depth_values,
        )
    ).astype(np.float32, copy=False)

    points_target: Optional[np.ndarray] = None
    if extrinsics is not None:
        extrinsics = np.asarray(extrinsics, dtype=np.float32)
        if extrinsics.shape != (4, 4):
            raise ValueError(f"Extrinsics must be 4x4 homogeneous matrix, received {extrinsics.shape}")
        ones = np.ones((points_camera.shape[0], 1), dtype=np.float32)
        homogeneous = np.hstack((points_camera, ones))
        transformed = (extrinsics @ homogeneous.T).T
        points_target = transformed[:, :3].astype(np.float32, copy=False)

    confidence_values = None
    if confidence is not None:
        confidence_values = confidence[ys, xs].astype(np.float32, copy=False)

    return PointCloudResult(
        points_camera=points_camera,
        points_target=points_target,
        pixel_x=xs.astype(np.int32, copy=False),
        pixel_y=ys.astype(np.int32, copy=False),
        depth=depth_values.astype(np.float32, copy=False),
        confidence=confidence_values,
    )


def _canonicalise_roi(
    roi: Iterable[int],
    width: int,
    height: int,
) -> Tuple[int, int, int, int]:
    """Clamp and order ROI coordinates."""

    coords = tuple(roi)
    if len(coords) != 4:
        raise ValueError(f"ROI must contain four integers, received {coords}")
    x_min, y_min, x_max, y_max = map(int, coords)
    if x_min > x_max:
        x_min, x_max = x_max, x_min
    if y_min > y_max:
        y_min, y_max = y_max, y_min

    x_min = int(np.clip(x_min, 0, width))
    x_max = int(np.clip(x_max, 0, width))
    y_min = int(np.clip(y_min, 0, height))
    y_max = int(np.clip(y_max, 0, height))

    if x_min == x_max or y_min == y_max:
        raise ValueError(f"ROI {coords} collapses to zero area after clamping")

    return x_min, y_min, x_max, y_max


__all__ = ["CameraIntrinsics", "PointCloudResult", "build_pointcloud"]
