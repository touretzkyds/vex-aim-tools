"""Camera calibration helpers shared across the DepthAnything pipeline.

The current defaults encode the Celeste robot geometry derived from
``aim_fsm/aim_kin.py`` and ``aim_fsm/camera.py``.  These can be overridden by
supplying a structured calibration file (JSON or YAML) via the environment
variable ``AIM_CAMERA_CALIBRATION``.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from .pointcloud_builder import CameraIntrinsics

_LOGGER = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default calibration constants (metres, camera frame → robot base frame).

_DEFAULT_CAMERA_TO_BASE = np.array(
    [
        [6.12323400e-17, -3.90731128e-01, 9.20504853e-01, 2.70000000e-02],
        [-1.00000000e00, -2.39253813e-17, 5.63646661e-17, 0.00000000e00],
        [0.00000000e00, -9.20504853e-01, -3.90731128e-01, 4.34700000e-02],
        [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
    ],
    dtype=np.float32,
)

_DEFAULT_GRAVITY_CAMERA = np.array(
    [0.0, -9.20504853e-01, -3.90731128e-01],
    dtype=np.float32,
)

# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CliffCalibration:
    """Bundle of intrinsics/extrinsics for cliff detection."""

    intrinsics: CameraIntrinsics
    """Camera intrinsics describing the input frame."""

    gravity_camera: np.ndarray
    """Gravity vector expressed in the camera frame (unit length)."""

    camera_to_base: np.ndarray
    """4x4 homogeneous transform mapping camera coordinates to the robot base frame."""


def load_cliff_calibration(camera: Optional[Any] = None) -> CliffCalibration:
    """Resolve calibration parameters for cliff detection.

    Args:
        camera: Optional camera object providing ``resolution``, ``focal_length``
            and ``center`` attributes.  When omitted, defaults to 640×480 with
            Fx/Fy=400 and principal point at the image centre.

    Returns:
        :class:`CliffCalibration` instance using either overrides supplied via
        ``AIM_CAMERA_CALIBRATION`` or the baked-in Celeste defaults.
    """

    data = _load_calibration_blob()

    intrinsics = _resolve_intrinsics(camera, data)

    gravity = _resolve_gravity(data)

    extrinsics = _resolve_extrinsics(data)

    return CliffCalibration(
        intrinsics=intrinsics,
        gravity_camera=gravity,
        camera_to_base=extrinsics,
    )


def _load_calibration_blob() -> Optional[dict[str, Any]]:
    """Read calibration overrides from ``AIM_CAMERA_CALIBRATION``."""

    env_value = os.environ.get("AIM_CAMERA_CALIBRATION")
    if not env_value:
        return None

    path = Path(env_value).expanduser()
    if not path.exists():
        _LOGGER.warning("Calibration file %s not found; falling back to defaults", path)
        return None

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        _LOGGER.warning("Unable to read calibration file %s: %s", path, exc)
        return None

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore
        except ImportError:
            _LOGGER.error(
                "Calibration file %s is not valid JSON and PyYAML is unavailable", path
            )
            return None
        try:
            data = yaml.safe_load(text)
        except Exception as exc:  # pragma: no cover - defensive
            _LOGGER.warning("Failed to parse calibration file %s: %s", path, exc)
            return None
        if not isinstance(data, dict):
            _LOGGER.warning("Calibration file %s did not contain a mapping", path)
            return None
        return data


def _resolve_intrinsics(camera: Optional[Any], data: Optional[dict[str, Any]]) -> CameraIntrinsics:
    if data and "intrinsics" in data:
        intr = data["intrinsics"]
        try:
            return CameraIntrinsics(
                fx=float(intr["fx"]),
                fy=float(intr["fy"]),
                cx=float(intr["cx"]),
                cy=float(intr["cy"]),
                width=int(intr["width"]),
                height=int(intr["height"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            _LOGGER.warning("Invalid intrinsics override; using defaults: %s", exc)

    if camera is not None:
        width, height = map(int, camera.resolution)
        fx, fy = map(float, camera.focal_length)
        cx, cy = map(float, camera.center)
        return CameraIntrinsics(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=width,
            height=height,
        )

    # Conservative fallback matching aim_fsm.camera defaults.
    return CameraIntrinsics(
        fx=400.0,
        fy=400.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
    )


def _resolve_gravity(data: Optional[dict[str, Any]]) -> np.ndarray:
    if data and "gravity_camera" in data:
        arr = np.asarray(data["gravity_camera"], dtype=np.float32)
        if arr.shape == (3,):
            norm = float(np.linalg.norm(arr))
            if norm > 0.0:
                return arr / norm
        _LOGGER.warning("Invalid gravity override; expected 3-vector, received %s", arr.shape)
    return _DEFAULT_GRAVITY_CAMERA.copy()


def _resolve_extrinsics(data: Optional[dict[str, Any]]) -> np.ndarray:
    if data and "camera_to_base" in data:
        arr = np.asarray(data["camera_to_base"], dtype=np.float32)
        if arr.shape == (4, 4):
            return arr
        _LOGGER.warning(
            "Invalid camera_to_base override; expected 4x4 matrix, received %s", arr.shape
        )
    return _DEFAULT_CAMERA_TO_BASE.copy()


__all__ = ["CliffCalibration", "load_cliff_calibration"]
