"""Qt list model projecting `aim_fsm.worldmap` objects for QML."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any, Dict, Optional

from PyQt6.QtCore import QAbstractListModel, QModelIndex, Qt, pyqtProperty, pyqtSignal, pyqtSlot

from aim_fsm.worldmap import (
    AprilTagObj,
    ArucoMarkerObj,
    BarrelObj,
    BlueBarrelObj,
    OrangeBarrelObj,
    SportsBallObj,
    WallObj,
)

RoleMap = Dict[int, bytes]
Item = Dict[str, Any]


def _role(index: int) -> int:
    return int(Qt.ItemDataRole.UserRole) + index


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _pose_attr(obj: Any, attr: str, default: float = 0.0) -> float:
    pose = getattr(obj, "pose", None)
    if pose is None and isinstance(obj, Mapping):
        pose = obj.get("pose")
    if pose is None:
        return float(default)

    value = getattr(pose, attr, None)
    if value is None and isinstance(pose, Mapping):
        value = pose.get(attr)
    if value is None and hasattr(pose, "__getitem__"):
        try:
            value = pose[attr]
        except Exception:  # pragma: no cover - non indexable pose
            value = None
    return _to_float(value, default)


def _theta_attr(obj: Any) -> float:
    theta = _pose_attr(obj, "theta", 0.0)
    if theta is None:
        return 0.0
    return _to_float(theta, 0.0)


class WorldMapModel(QAbstractListModel):
    """`QAbstractListModel` exposing canonical world objects to QML."""

    countChanged = pyqtSignal()

    ROLE_NAMES: tuple[str, ...] = (
        "id",
        "type",
        "x",
        "y",
        "z",
        "theta",
        "visible",
        "missing",
        "diameter_mm",
        "height_mm",
        "length_mm",
        "thickness_mm",
        "size_mm",
        "marker_id",
        "holding",
    )

    _ROLE_MAP: RoleMap = {
        _role(i + 1): name.encode("utf-8") for i, name in enumerate(ROLE_NAMES)
    }

    def __init__(self, parent: Optional[Any] = None) -> None:
        super().__init__(parent)
        self._items: list[Item] = []

    @pyqtProperty(int, notify=countChanged)
    def count(self) -> int:
        return len(self._items)

    # Qt overrides --------------------------------------------------

    def roleNames(self) -> RoleMap:  # type: ignore[override]
        return self._ROLE_MAP

    def rowCount(self, parent: QModelIndex | None = None) -> int:  # type: ignore[override]
        if parent is not None and parent.isValid():
            return 0
        return len(self._items)

    def data(self, index: QModelIndex, role: int = 0) -> Any:  # type: ignore[override]
        if not index.isValid():
            return None
        row = index.row()
        if row < 0 or row >= len(self._items):
            return None
        name = self._ROLE_MAP.get(role)
        if name is None:
            return None
        key = name.decode("utf-8")
        return self._items[row].get(key)

    # Public API ---------------------------------------------------

    def sync_from(self, robot: Any, objects: Mapping[str, Any] | Iterable[tuple[str, Any]]) -> None:
        """Rebuild the model from the provided robot and world objects."""

        entries: list[Item] = []

        robot_entry = self._build_robot(robot)
        if robot_entry is not None:
            entries.append(robot_entry)

        if hasattr(objects, "items"):
            iterator = getattr(objects, "items")()
        else:
            iterator = objects  # assume iterable of pairs

        for key, obj in sorted(iterator, key=lambda pair: str(pair[0])):  # type: ignore[arg-type]
            item = self._build_object(str(key), obj)
            if item is not None:
                entries.append(item)

        # Add cliff edges from cliff detection results
        cliff_items = self._build_cliff_edges(robot)
        entries.extend(cliff_items)

        # Debug: print model update info (only on changes)
        cliff_sessions = getattr(robot, "cliff_sessions", None)
        num_sessions = len(cliff_sessions) if cliff_sessions else 0
        last_num_sessions = getattr(self, "_last_num_sessions", 0)
        cliff_count = len([e for e in entries if 'cliff_' in e.get('id', '')])

        if cliff_count > 0 and num_sessions != last_num_sessions:
            self._last_num_sessions = num_sessions
            print(f"[WorldMapModel] Total entries: {len(entries)}, cliff segments: {cliff_count} ({num_sessions} sessions)")
            import sys
            sys.stdout.flush()

        self.beginResetModel()
        self._items = entries
        self.endResetModel()
        self.countChanged.emit()

    @pyqtSlot(int, result="QVariant")
    def get(self, row: int) -> Optional[Item]:
        """Expose model rows to QML callers via an index-based lookup."""
        if row < 0 or row >= len(self._items):
            return None
        return dict(self._items[row])

    # Internal helpers ---------------------------------------------

    def _build_robot(self, robot: Any) -> Optional[Item]:
        if robot is None:
            return None
        entry: Item = {
            "id": "robot#1",
            "type": "robot",
            "x": _pose_attr(robot, "x", 0.0),
            "y": _pose_attr(robot, "y", 0.0),
            "z": _pose_attr(robot, "z", 0.0),
            "theta": _theta_attr(robot),
            "visible": True,
            "missing": False,
            "diameter_mm": 64.0,
            "height_mm": 72.0,
            "length_mm": None,
            "thickness_mm": None,
            "size_mm": None,
            "marker_id": None,
            "holding": bool(getattr(robot, "holding", False)),
        }
        return entry

    def _build_object(self, object_id: str, obj: Any) -> Optional[Item]:
        if obj is None:
            return None

        type_name = self._resolve_type(obj)
        if type_name is None:
            return None
        
        # Skip robot objects - robot is added separately via _build_robot
        if type_name == "robot":
            return None

        entry: Item = {
            "id": object_id,
            "type": type_name,
            "x": _pose_attr(obj, "x", 0.0),
            "y": _pose_attr(obj, "y", 0.0),
            "z": _pose_attr(obj, "z", 0.0),
            "theta": _theta_attr(obj),
            "visible": bool(getattr(obj, "is_visible", False)),
            "missing": bool(getattr(obj, "is_missing", False)),
            "diameter_mm": None,
            "height_mm": None,
            "length_mm": None,
            "thickness_mm": None,
            "size_mm": None,
            "marker_id": None,
            "holding": None,
        }

        if type_name == "sports_ball":
            diameter = _to_float(getattr(obj, "diameter", getattr(obj, "diameter_mm", None)), 0.0)
            entry["diameter_mm"] = diameter
            entry["z"] = diameter / 2.0 if diameter else entry["z"]
        elif type_name in ("barrel", "barrel_orange", "barrel_blue"):
            diameter = _to_float(getattr(obj, "diameter", getattr(obj, "diameter_mm", None)), 0.0)
            height = _to_float(getattr(obj, "height", getattr(obj, "height_mm", None)), 0.0)
            entry["diameter_mm"] = diameter
            entry["height_mm"] = height
            entry["z"] = height / 2.0 if height else entry["z"]
        elif type_name == "apriltag":
            # Legacy: width=38mm (Y-axis), height=48mm (Z-axis), thickness=2mm (X-axis)
            # tag_size = (2, 38, 48) in legacy worldmap_viewer.py line 853
            width = 38.0   # Horizontal width (Y-axis in world coords)
            height = 48.0  # Vertical height (Z-axis), ~2/3 robot height (72mm)
            thickness = 2.0  # Depth (X-axis)
            entry["width_mm"] = width
            entry["height_mm"] = height
            entry["thickness_mm"] = thickness
            # Ensure marker_id is converted to string for QML
            tag_id = getattr(obj, "tag_id", None)
            entry["marker_id"] = str(tag_id) if tag_id is not None else None
            # Center should be at half height so bottom sits on ground
            entry["z"] = height / 2.0  # 24mm for height=48
        elif type_name == "aruco":
            marker = getattr(obj, "marker", None)
            size_candidate = None
            if marker is not None:
                parent = getattr(marker, "aruco_parent", None)
                if parent is not None:
                    size_candidate = getattr(parent, "marker_size", None)
                if size_candidate is None:
                    size_candidate = getattr(marker, "marker_size", getattr(marker, "size", None))
            size = _to_float(size_candidate or getattr(obj, "size", None), 38.0)
            entry["size_mm"] = size
            entry["marker_id"] = getattr(obj, "marker_id", None)
            entry["thickness_mm"] = 4.0
            entry["z"] = 2.0
        elif type_name == "wall":
            length = _to_float(getattr(obj, "length", getattr(obj, "length_mm", None)), 0.0)
            height = _to_float(getattr(obj, "height", getattr(obj, "height_mm", None)), 0.0)
            entry["length_mm"] = length
            entry["height_mm"] = height
            entry["thickness_mm"] = 4.0
            entry["z"] = height / 2.0 if height else entry["z"]

        return entry

    def _build_cliff_edges(self, robot: Any) -> list[Item]:
        """Build wall-like items from cliff detection results stored in robot.

        Now supports multiple detection sessions (accumulated history).
        Each session is rendered separately with its own segments.
        """
        cliff_items: list[Item] = []

        # NEW: Support multiple detection sessions (accumulated)
        cliff_sessions = getattr(robot, "cliff_sessions", None)
        if cliff_sessions:
            # Render all sessions
            for session_idx, session in enumerate(cliff_sessions):
                session_id = session.get("id", f"session_{session_idx}")
                cliff_results = session.get("results", [])
                session_items = self._build_session_cliff_edges(
                    cliff_results, session_id, session_idx
                )
                cliff_items.extend(session_items)
            return cliff_items

        # Fallback: single session mode (backward compatibility)
        cliff_results = getattr(robot, "cliff_results", None)
        if not cliff_results:
            return cliff_items

        cliff_items = self._build_session_cliff_edges(cliff_results, "cliff_0", 0)
        return cliff_items

    def _build_session_cliff_edges(
        self, cliff_results: list, session_id: str, session_idx: int
    ) -> list[Item]:
        """Build cliff edges for a single detection session."""
        cliff_items: list[Item] = []

        if not cliff_results:
            return cliff_items

        import numpy as np
        import math

        # Check if this session was already logged (avoid spam on every refresh)
        if not hasattr(self, "_logged_sessions"):
            self._logged_sessions = set()
        is_new_session = session_id not in self._logged_sessions

        # Find the best frame (highest confidence segment)
        best_frame = None
        best_confidence = -1.0
        for result in cliff_results:
            if hasattr(result, "segments") and result.segments:
                for segment in result.segments:
                    if segment.confidence > best_confidence:
                        best_confidence = segment.confidence
                        best_frame = result

        if best_frame is None:
            return cliff_items

        # Only process the best frame to avoid rendering thousands of objects
        for seg_idx, segment in enumerate(best_frame.segments):
            # Skip if no world coordinates
            if segment.polyline_world is None:
                continue

            # Convert polyline from meters to millimeters
            polyline_m = segment.polyline_world
            polyline_mm = polyline_m * 1000.0  # Convert to mm

            # Debug: print polyline stats ONLY for new sessions (not on every refresh)
            if is_new_session and seg_idx == 0:
                total_length = 0.0
                for i in range(len(polyline_mm) - 1):
                    dx = polyline_mm[i+1, 0] - polyline_mm[i, 0]
                    dy = polyline_mm[i+1, 1] - polyline_mm[i, 1]
                    dz = polyline_mm[i+1, 2] - polyline_mm[i, 2]
                    total_length += math.sqrt(dx**2 + dy**2 + dz**2)
                avg_step = total_length / (len(polyline_mm) - 1) if len(polyline_mm) > 1 else 0
                print(f"  Session {session_idx} (NEW): Polyline {len(polyline_mm)} points, length={total_length:.1f}mm, avg step={avg_step:.2f}mm")

            # Flat red line rendering - thin horizontal segments
            # Sample every Nth point to create line segments
            # Aim for ~20-30 segments for smooth line without too many objects
            sample_step = max(20, len(polyline_mm) // 25)  # At most 25 segments
            sampled_indices = list(range(0, len(polyline_mm), sample_step))
            if sampled_indices[-1] != len(polyline_mm) - 1:
                sampled_indices.append(len(polyline_mm) - 1)  # Always include last point

            # Build flat line segments between sampled points
            for i in range(len(sampled_indices) - 1):
                idx1 = sampled_indices[i]
                idx2 = sampled_indices[i + 1]

                p1 = polyline_mm[idx1]
                p2 = polyline_mm[idx2]

                # Calculate segment midpoint (x, y, z)
                # CRITICAL: Convert to Python float, not numpy float!
                mid_x = float((p1[0] + p2[0]) / 2.0)
                mid_y = float((p1[1] + p2[1]) / 2.0)
                mid_z = float((p1[2] + p2[2]) / 2.0)

                # Calculate segment length
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dz = p2[2] - p1[2]
                length_mm = float(math.sqrt(dx**2 + dy**2 + dz**2))

                # Skip degenerate segments
                if length_mm < 0.1:
                    continue

                # Calculate rotation angle (theta) in xy plane
                theta_rad = float(math.atan2(dy, dx))

                # Ultra-flat red line parameters
                height_mm = 1.5   # 1.5mm height - VERY thin, almost flat
                thickness_mm = 15.0  # 15mm width - visible but not too thick

                # Cube is centered, lift by height/2 to sit on ground
                z_position = mid_z + height_mm / 2.0

                # Create a cliff line segment
                cliff_id = f"{session_id}_seg{seg_idx}_{i}"
                item: Item = {
                    "id": cliff_id,
                    "type": "wall",  # Use wall type (cube-based)
                    "x": mid_x,
                    "y": mid_y,
                    "z": z_position,
                    "theta": theta_rad,
                    "visible": True,
                    "missing": False,
                    "diameter_mm": None,
                    "height_mm": height_mm,
                    "length_mm": length_mm,
                    "thickness_mm": thickness_mm,
                    "size_mm": None,
                    "marker_id": None,
                    "holding": None,
                }

                cliff_items.append(item)

        # Debug: print session summary ONLY for new sessions
        if is_new_session and cliff_items:
            first = cliff_items[0]
            print(f"  Session {session_idx} (NEW): Created {len(cliff_items)} flat line segments (best conf: {best_confidence:.2f})")
            print(f"    First segment: id='{first['id']}', x={first['x']:.1f}mm, y={first['y']:.1f}mm, "
                  f"size={first['length_mm']:.1f}×{first['thickness_mm']:.1f}×{first['height_mm']:.1f}mm")
            import sys
            sys.stdout.flush()
            # Mark this session as logged
            self._logged_sessions.add(session_id)

        return cliff_items

    @staticmethod
    def _resolve_type(obj: Any) -> Optional[str]:
        if isinstance(obj, SportsBallObj):
            return "sports_ball"
        # Check barrel subclasses first (before base class)
        if isinstance(obj, OrangeBarrelObj):
            return "barrel_orange"
        if isinstance(obj, BlueBarrelObj):
            return "barrel_blue"
        if isinstance(obj, BarrelObj):
            return "barrel"  # fallback for generic barrels
        if isinstance(obj, AprilTagObj):
            return "apriltag"
        if isinstance(obj, ArucoMarkerObj):
            return "aruco"
        if isinstance(obj, WallObj):
            return "wall"
        name = getattr(obj, "name", "")
        if isinstance(name, str):
            lower = name.lower()
            for candidate in ("sports_ball", "barrel", "apriltag", "aruco", "wall", "robot"):
                if candidate in lower:
                    return candidate
        return None


__all__ = ["WorldMapModel"]
