"""Lazy import surface for the PyQt6 viewer stack."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "CameraImageProvider",
    "CamViewer",
    "DepthViewer",
    "CliffOverlay",
    "CliffOverlayConfig",
    "LandmarkModel",
    "ParticleLayerModel",
    "ParticleSummary",
    "ParticleViewer",
    "ParticleViewState",
    "PathViewer",
    "PathViewState",
    "QtViewerApp",
    "SnapshotService",
    "WorldMapModel",
    "WorldMapViewer",
    # PyQt6 imshow() API
    "namedWindow",
    "imshow",
    "destroyWindow",
    "destroyAllWindows",
    "ImshowImageProvider",
    "ImshowWindow",
    "WindowManager",
]


_MODULE_MAP = {
    "CameraImageProvider": ("viewer.camera_provider", "CameraImageProvider"),
    "CliffOverlay": ("viewer.cliff_overlay", "CliffOverlay"),
    "CliffOverlayConfig": ("viewer.cliff_overlay", "CliffOverlayConfig"),
    "CamViewer": ("viewer.cam_viewer", "CamViewer"),
    "DepthViewer": ("viewer.depth_viewer", "DepthViewer"),
    "LandmarkModel": ("viewer.particle_model", "LandmarkModel"),
    "ParticleLayerModel": ("viewer.particle_model", "ParticleLayerModel"),
    "ParticleSummary": ("viewer.particle_model", "ParticleSummary"),
    "ParticleViewer": ("viewer.particle_viewer", "ParticleViewer"),
    "ParticleViewState": ("viewer.particle_viewer", "ParticleViewState"),
    "PathViewer": ("viewer.path_viewer", "PathViewer"),
    "PathViewState": ("viewer.path_viewer", "PathViewState"),
    "QtViewerApp": ("viewer.qt_app", "QtViewerApp"),
    "SnapshotService": ("viewer.snapshot_service", "SnapshotService"),
    "WorldMapModel": ("viewer.worldmap_model", "WorldMapModel"),
    "WorldMapViewer": ("viewer.worldmap_viewer", "WorldMapViewer"),
    # PyQt6 imshow() API
    "namedWindow": ("viewer.imshow_manager", "namedWindow"),
    "imshow": ("viewer.imshow_manager", "imshow"),
    "destroyWindow": ("viewer.imshow_manager", "destroyWindow"),
    "destroyAllWindows": ("viewer.imshow_manager", "destroyAllWindows"),
    "ImshowImageProvider": ("viewer.imshow_provider", "ImshowImageProvider"),
    "ImshowWindow": ("viewer.imshow_window", "ImshowWindow"),
    "WindowManager": ("viewer.imshow_manager", "WindowManager"),
}


def __getattr__(name: str) -> Any:
    if name not in _MODULE_MAP:
        raise AttributeError(f"module 'viewer' has no attribute '{name}'")
    module_name, attr = _MODULE_MAP[name]
    module = import_module(module_name)
    return getattr(module, attr)
