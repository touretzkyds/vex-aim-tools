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
    OpenVocabObj,
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
        "width_mm",
        "thickness_mm",
        "size_mm",
        "marker_id",
        "label",
        "color",
        "doorways",
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
            "width_mm": None,
            "thickness_mm": None,
            "size_mm": None,
            "marker_id": None,
            "doorways": [],
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
            "width_mm": None,
            "thickness_mm": None,
            "size_mm": None,
            "marker_id": None,
            "doorways": [],
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
        elif type_name == "openvocab":
            diameter = _to_float(getattr(obj, "diameter", None), 40.0)
            height = _to_float(getattr(obj, "height", None), diameter)
            entry["diameter_mm"] = diameter
            entry["height_mm"] = height
            entry["label"] = getattr(obj, "label", "")
            entry["color"] = getattr(obj, "color", None) or "#35d07f"
            entry["z"] = height / 2.0
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
            entry["width_mm"] = size
            entry["height_mm"] = size
            entry["marker_id"] = getattr(obj, "marker_id", None)
            entry["thickness_mm"] = 2.0
            entry["z"] = size / 2.0
        elif type_name == "wall":
            length = _to_float(getattr(obj, "length", getattr(obj, "length_mm", None)), 0.0)
            height = _to_float(getattr(obj, "height", getattr(obj, "height_mm", None)), 0.0)
            entry["length_mm"] = length
            entry["height_mm"] = height
            entry["thickness_mm"] = 4.0
            entry["z"] = height / 2.0 if height else entry["z"]
            wall_spec = getattr(obj, "wall_spec", None)
            door_specs = getattr(wall_spec, "doorways", None)
            doorways: list[Item] = []
            if isinstance(door_specs, Mapping):
                sorted_specs = sorted(door_specs.items(), key=lambda pair: int(pair[0]))
                for index, spec in sorted_specs:
                    if not isinstance(spec, Mapping):
                        continue
                    doorways.append(
                        {
                            "index": int(index),
                            "x": _to_float(spec.get("x"), 0.0),
                            "width": max(0.0, _to_float(spec.get("width"), 0.0)),
                            "height": max(0.0, _to_float(spec.get("height"), 0.0)),
                        }
                    )
            entry["doorways"] = doorways

        return entry

    @staticmethod
    def _resolve_type(obj: Any) -> Optional[str]:
        if isinstance(obj, OpenVocabObj):
            return "openvocab"
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
