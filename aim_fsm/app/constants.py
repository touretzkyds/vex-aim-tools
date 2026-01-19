"""Read-only application constants shared across the Qt bootstrap stack."""

from __future__ import annotations

from types import MappingProxyType
from typing import Mapping

# Scaling / camera constants are measured in GL units where 1 GL == 50 mm.
AIVISION_RESOLUTION_SCALE: int = 1
WSCALE: float = 0.02
FOV_DEG: float = 50.0
NEAR_GL: float = 5.0
FAR_GL: float = 600.0
INITIAL_DIST_GL: float = 500.0

# Snapshot file handling configuration.
SNAPSHOT_DIR: str = "snapshots/"
SNAPSHOT_NAME_TEMPLATE: str = "{path}{name}{snapno}.png"
SNAP_SUFFIX_RAW: str = "_snap"
SNAP_SUFFIX_ANN: str = "_asnap"

# Physical object sizes (millimetres).
SPORTS_BALL_DIAM_MM: int = 25
BARREL_DIAM_MM: int = 22
BARREL_HEIGHT_MM: int = 25
APRILTAG_SIZE_MM: int = 38

# Detection thresholds and heuristics.
MAX_DISTANCE_MM: int = 300
BEARING_THRESHOLD_DEG: int = 30

_constants = MappingProxyType(
    {
        "AIVISION_RESOLUTION_SCALE": AIVISION_RESOLUTION_SCALE,
        "WSCALE": WSCALE,
        "FOV_DEG": FOV_DEG,
        "NEAR_GL": NEAR_GL,
        "FAR_GL": FAR_GL,
        "INITIAL_DIST_GL": INITIAL_DIST_GL,
        "SNAPSHOT_DIR": SNAPSHOT_DIR,
        "SNAPSHOT_NAME_TEMPLATE": SNAPSHOT_NAME_TEMPLATE,
        "SNAP_SUFFIX_RAW": SNAP_SUFFIX_RAW,
        "SNAP_SUFFIX_ANN": SNAP_SUFFIX_ANN,
        "SPORTS_BALL_DIAM_MM": SPORTS_BALL_DIAM_MM,
        "BARREL_DIAM_MM": BARREL_DIAM_MM,
        "BARREL_HEIGHT_MM": BARREL_HEIGHT_MM,
        "APRILTAG_SIZE_MM": APRILTAG_SIZE_MM,
        "MAX_DISTANCE_MM": MAX_DISTANCE_MM,
        "BEARING_THRESHOLD_DEG": BEARING_THRESHOLD_DEG,
    }
)

constants: Mapping[str, object] = _constants

__all__ = [
    "AIVISION_RESOLUTION_SCALE",
    "WSCALE",
    "FOV_DEG",
    "NEAR_GL",
    "FAR_GL",
    "INITIAL_DIST_GL",
    "SNAPSHOT_DIR",
    "SNAPSHOT_NAME_TEMPLATE",
    "SNAP_SUFFIX_RAW",
    "SNAP_SUFFIX_ANN",
    "SPORTS_BALL_DIAM_MM",
    "BARREL_DIAM_MM",
    "BARREL_HEIGHT_MM",
    "APRILTAG_SIZE_MM",
    "MAX_DISTANCE_MM",
    "BEARING_THRESHOLD_DEG",
    "constants",
]
