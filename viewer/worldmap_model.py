"""Qt list model projecting `aim_fsm.worldmap` objects for QML."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from typing import Any, Dict, Optional

from PyQt6.QtCore import QAbstractListModel, QModelIndex, Qt, pyqtProperty, pyqtSignal, pyqtSlot

from aim_fsm.worldmap import DominoObj

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
    return _to_float(value, default)


class WorldMapModel(QAbstractListModel):
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
        "doorways",
        "holding",
        "face_label",
        "domino_halves",
        "is_fallen",
        "orientation",
        "flipped",
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

    def roleNames(self) -> RoleMap:
        return self._ROLE_MAP

    def rowCount(self, parent: QModelIndex | None = None) -> int:
        if parent is not None and parent.isValid():
            return 0
        return len(self._items)

    def data(self, index: QModelIndex, role: int = 0) -> Any:
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

    def sync_from(self, robot: Any, objects: Mapping[str, Any] | Iterable[tuple[str, Any]]) -> None:
        entries: list[Item] = []

        robot_entry = self._build_robot(robot)
        if robot_entry is not None:
            entries.append(robot_entry)

        if hasattr(objects, "items"):
            iterator = getattr(objects, "items")()
        else:
            iterator = objects

        for key, obj in sorted(iterator, key=lambda pair: str(pair[0])):
            item = self._build_object(str(key), obj)
            if item is not None:
                entries.append(item)

        self.beginResetModel()
        self._items = entries
        self.endResetModel()
        self.countChanged.emit()

    def _build_robot(self, robot: Any) -> Optional[Item]:
        if robot is None:
            return None
        return {
            "id": "robot#1",
            "type": "robot",
            "x": _pose_attr(robot, "x", 0.0),
            "y": _pose_attr(robot, "y", 0.0),
            "z": _pose_attr(robot, "z", 0.0),
            "theta": _pose_attr(robot, "theta", 0.0),
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
            "holding": False,
            "face_label": None,
            "domino_halves": [],
            "is_fallen": False,
            "orientation": None,
            "flipped": None,
        }

    def _build_object(self, object_id: str, obj: Any) -> Optional[Item]:
        if obj is None or not isinstance(obj, DominoObj):
            return None

        is_fallen = bool(getattr(obj, "is_fallen", False))
        is_fixed = bool(getattr(obj, "is_fixed", False))
        is_visible = bool(getattr(obj, "is_visible", False)) or is_fixed

        length = 48.0
        width = 24.0
        thickness = 8.0

        entry: Item = {
            "id": object_id,
            "type": "domino",
            "x": _pose_attr(obj, "x", 0.0),
            "y": _pose_attr(obj, "y", 0.0),
            "z": 4.0 if is_fallen else 12.0,
            "theta": _pose_attr(obj, "theta", 0.0),
            "visible": is_visible,
            "missing": False if is_fixed else bool(getattr(obj, "is_missing", False)),
            "diameter_mm": None,
            "height_mm": width if is_fallen else width,
            "length_mm": thickness,
            "width_mm": length,
            "thickness_mm": thickness,
            "size_mm": None,
            "marker_id": None,
            "doorways": [],
            "holding": None,
            "face_label": getattr(obj, "face_label", None),
            "domino_halves": self._build_domino_halves(obj, length),
            "is_fallen": is_fallen,
            "orientation": getattr(obj, "orientation", None) or ("flat" if is_fallen else "edgewise"),
            "flipped": getattr(obj, "flipped", None),
        }
        return entry

    def _build_domino_halves(self, obj: Any, length: float) -> list[Item]:
        half_span = length / 4.0
        counts = getattr(obj, "half_counts", None)
        face_label = getattr(obj, "face_label", None)

        if not counts and face_label and "-" in face_label:
            try:
                parts = face_label.split("-")
                counts = (int(parts[0]), int(parts[1]))
            except ValueError:
                pass

        if not counts or len(counts) != 2 or any(c is None for c in counts):
            return [
                {"count": None, "local_y": -half_span},
                {"count": None, "local_y": half_span},
            ]

        return [
            {"count": int(counts[0]), "local_y": -half_span},
            {"count": int(counts[1]), "local_y": half_span},
        ]
