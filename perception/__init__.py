"""Perception subpackage for DepthAnything-based geometry processing."""

from .depth_anything_provider import DepthAnythingProvider, DepthResult
from .pointcloud_builder import CameraIntrinsics, PointCloudResult, build_pointcloud
from .plane_fit import PlaneModel, PlaneFitResult, fit_horizontal_planes
from .cliff_edges import CliffSegment, CliffDetectionResult, CliffDebugStats, detect_cliff_segments
from .cliff_detector import (
    DetectorConfig,
    CliffDetector,
    CliffDetectorOutput,
    PlaneDebugInfo,
)
from .calibration import CliffCalibration, load_cliff_calibration

__all__ = [
    "DepthAnythingProvider",
    "DepthResult",
    "CameraIntrinsics",
    "PointCloudResult",
    "build_pointcloud",
    "PlaneModel",
    "PlaneFitResult",
    "fit_horizontal_planes",
    "CliffSegment",
    "CliffDetectionResult",
    "CliffDebugStats",
    "detect_cliff_segments",
    "DetectorConfig",
    "CliffDetector",
    "CliffDetectorOutput",
    "PlaneDebugInfo",
    "CliffCalibration",
    "load_cliff_calibration",
]
