"""Lazy import surface for the PyQt6 viewer stack."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "CameraImageProvider",
    "CamViewer",
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
]


_MODULE_MAP = {
    "CameraImageProvider": ("viewer.camera_provider", "CameraImageProvider"),
    "CliffOverlay": ("viewer.cliff_overlay", "CliffOverlay"),
    "CliffOverlayConfig": ("viewer.cliff_overlay", "CliffOverlayConfig"),
    "CamViewer": ("viewer.cam_viewer", "CamViewer"),
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
}


def __getattr__(name: str) -> Any:
    if name not in _MODULE_MAP:
        raise AttributeError(f"module 'viewer' has no attribute '{name}'")
    module_name, attr = _MODULE_MAP[name]
    module = import_module(module_name)
    return getattr(module, attr)
