"""Data providers and geometry conversion helpers for the Qt-based path viewer.

This module mirrors the responsibilities of :mod:`viewer.particle_model`, but
for the RRT / Wavefront tooling.  It focuses on translating the legacy data
structures (``aim_fsm.rrt.RRT``, ``aim_fsm.wavefront.WaveFront`` and the
robot's obstacle definitions) into immutable snapshots that are convenient for
Qt / QML consumers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi, sin
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from PyQt6.QtCore import (
    QAbstractListModel,
    QModelIndex,
    QObject,
    Qt,
    QSize,
    pyqtProperty,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import QColor, QImage
from PyQt6.QtQuick import QQuickImageProvider

from aim_fsm import geometry
from aim_fsm.geometry import wrap_angle
from aim_fsm.rrt import RRT, RRTNode
from aim_fsm.rrt_shapes import Circle, Rectangle, Shape
from aim_fsm.wavefront import WaveFront

__all__ = [
    "Bounds",
    "NodeMarker",
    "CircleItem",
    "PathSceneProvider",
    "PathSceneSnapshot",
    "Polyline",
    "RectangleItem",
    "RobotOutline",
    "PathNodeModel",
    "PathObstacleModel",
    "PathPolylineModel",
    "WavefrontTextureProvider",
    "WavefrontFrame",
]


# ---------------------------------------------------------------------------
# Dataclasses describing the geometry the Qt layer will consume


@dataclass(frozen=True)
class Polyline:
    """Polyline encoded as a flat list of xy points measured in millimetres."""

    points: Tuple[float, ...]
    color_rgba: Tuple[float, float, float, float]
    width_mm: float = 1.0
    tag: Optional[str] = None


@dataclass(frozen=True)
class NodeMarker:
    """Square marker representing a single RRT node."""

    x: float
    y: float
    size_mm: float
    color_rgba: Tuple[float, float, float, float]
    tag: Optional[str] = None


@dataclass(frozen=True)
class CircleItem:
    """Renderable circle – used for both obstacles and robot parts."""

    x: float
    y: float
    radius_mm: float
    color_rgba: Tuple[float, float, float, float]
    filled: bool
    tag: Optional[str] = None


@dataclass(frozen=True)
class RectangleItem:
    """Renderable rectangle – used for obstacles and robot parts."""

    x: float
    y: float
    width_mm: float
    height_mm: float
    rotation_deg: float
    color_rgba: Tuple[float, float, float, float]
    filled: bool
    tag: Optional[str] = None


@dataclass(frozen=True)
class RobotOutline:
    """Robot outline decomposed into renderable primitives."""

    parts: Tuple[CircleItem | RectangleItem, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class WavefrontFrame:
    """Wavefront grid snapshot ready for rasterisation into a texture."""

    grid: np.ndarray
    square_size_mm: float
    goal_marker: int
    max_value: int

    def copy(self) -> "WavefrontFrame":
        return WavefrontFrame(
            grid=self.grid.copy(),
            square_size_mm=self.square_size_mm,
            goal_marker=self.goal_marker,
            max_value=self.max_value,
        )


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned bounds covering all rendered elements."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @classmethod
    def from_points(cls, xs: Sequence[float], ys: Sequence[float]) -> Optional["Bounds"]:
        if not xs or not ys:
            return None
        return cls(min(xs), min(ys), max(xs), max(ys))

    def expanded(self, padding_mm: float) -> "Bounds":
        return Bounds(
            self.min_x - padding_mm,
            self.min_y - padding_mm,
            self.max_x + padding_mm,
            self.max_y + padding_mm,
        )


@dataclass(frozen=True)
class PathSceneSnapshot:
    """Immutable snapshot exposing everything the Qt layer needs to render."""

    revision: int
    tree_markers: Tuple[NodeMarker, ...]
    tree_edges: Tuple[Polyline, ...]
    path_overlays: Tuple[Polyline, ...]
    obstacles: Tuple[CircleItem | RectangleItem, ...]
    goal_region: Optional[CircleItem | RectangleItem]
    robot_outline: Optional[RobotOutline]
    wavefront: Optional[WavefrontFrame]
    bounds: Optional[Bounds]
    status_text: Optional[str]

    @classmethod
    def empty(cls) -> "PathSceneSnapshot":
        return PathSceneSnapshot(
            revision=0,
            tree_markers=tuple(),
            tree_edges=tuple(),
            path_overlays=tuple(),
            obstacles=tuple(),
            goal_region=None,
            robot_outline=None,
            wavefront=None,
            bounds=None,
            status_text=None,
        )


# ---------------------------------------------------------------------------
# Helper functions


RoleMap = dict[int, bytes]


def _role(index: int) -> int:
    return int(Qt.ItemDataRole.UserRole) + index


def _color_as_list(color: Tuple[float, float, float, float]) -> list[float]:
    r, g, b, a = color
    return [
        float(max(0.0, min(1.0, r))),
        float(max(0.0, min(1.0, g))),
        float(max(0.0, min(1.0, b))),
        float(max(0.0, min(1.0, a))),
    ]


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


class _BaseListModel(QAbstractListModel):
    """Shared helpers for path viewer list models."""

    countChanged = pyqtSignal()
    revisionChanged = pyqtSignal()

    def __init__(self, role_names: Sequence[str], parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._items: list[dict[str, Any]] = []
        self._revision: int = 0
        self._role_map: RoleMap = {
            _role(i + 1): name.encode("utf-8") for i, name in enumerate(role_names)
        }

    @pyqtProperty(int, notify=countChanged)
    def count(self) -> int:
        return len(self._items)

    @pyqtProperty(int, notify=revisionChanged)
    def revision(self) -> int:
        return self._revision

    # Qt overrides --------------------------------------------------

    def roleNames(self) -> RoleMap:  # type: ignore[override]
        return self._role_map

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
        name = self._role_map.get(role)
        if name is None:
            return None
        key = name.decode("utf-8")
        return self._items[row].get(key)

    # Public API ----------------------------------------------------

    @pyqtSlot(int, result="QVariant")
    def get(self, row: int) -> Optional[dict[str, Any]]:
        if row < 0 or row >= len(self._items):
            return None
        return dict(self._items[row])

    def _reset(self, entries: Sequence[dict[str, Any]]) -> None:
        self.beginResetModel()
        self._items = list(entries)
        self._revision += 1
        self.endResetModel()
        self.countChanged.emit()
        self.revisionChanged.emit()


class PathNodeModel(_BaseListModel):
    """Model exposing RRT node markers for Canvas rendering."""

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(("x", "y", "size", "color", "tag", "treeId"), parent=parent)

    def sync_from(self, markers: Sequence[NodeMarker]) -> None:
        entries: list[dict[str, Any]] = []
        for marker in markers:
            color = marker.color_rgba or (0.0, 1.0, 0.0, 1.0)
            tag = marker.tag or ""
            entries.append(
                {
                    "x": float(marker.x),
                    "y": float(marker.y),
                    "size": float(marker.size_mm),
                    "color": _color_as_list(color),
                    "tag": tag,
                    "treeId": tag,
                }
            )
        self._reset(entries)


class PathPolylineModel(_BaseListModel):
    """Model exposing polyline segments (tree edges or path overlays)."""

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(("points", "color", "width", "tag"), parent=parent)

    def sync_from(self, polylines: Sequence[Polyline]) -> None:
        entries: list[dict[str, Any]] = []
        for poly in polylines:
            entries.append(
                {
                    "points": list(map(float, poly.points)),
                    "color": _color_as_list(poly.color_rgba),
                    "width": float(poly.width_mm),
                    "tag": poly.tag or "",
                }
            )
        self._reset(entries)


class PathObstacleModel(_BaseListModel):
    """Model exposing circular/rectangular obstacles (including goal or robot parts)."""

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(
            ("type", "x", "y", "radius", "width", "height", "rotation", "color", "filled", "tag"),
            parent=parent,
        )

    def sync_from(
        self,
        shapes: Optional[Sequence[CircleItem | RectangleItem]],
        extra: Optional[Sequence[CircleItem | RectangleItem]] = None,
    ) -> None:
        entries: list[dict[str, Any]] = []

        def append_shape(item: CircleItem | RectangleItem) -> None:
            tag = item.tag or ""
            if isinstance(item, CircleItem):
                entries.append(
                    {
                        "type": "circle",
                        "x": float(item.x),
                        "y": float(item.y),
                        "radius": float(item.radius_mm),
                        "width": 0.0,
                        "height": 0.0,
                        "rotation": 0.0,
                        "color": _color_as_list(item.color_rgba),
                        "filled": bool(item.filled),
                        "tag": tag,
                    }
                )
            elif isinstance(item, RectangleItem):
                entries.append(
                    {
                        "type": "rectangle",
                        "x": float(item.x),
                        "y": float(item.y),
                        "radius": 0.0,
                        "width": float(item.width_mm),
                        "height": float(item.height_mm),
                        "rotation": float(item.rotation_deg),
                        "color": _color_as_list(item.color_rgba),
                        "filled": bool(item.filled),
                        "tag": tag,
                    }
                )

        if shapes:
            for shape in shapes:
                append_shape(shape)
        if extra:
            for shape in extra:
                append_shape(shape)

        self._reset(entries)


class WavefrontTextureProvider(QQuickImageProvider):
    """Image provider that rasterises wavefront grids to ``QImage`` textures."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._image: QImage = QImage()
        self._revision: int = 0
        self._cache_key: str = "wavefront/0"
        self._square_size_mm: float = 5.0
        self._image_size = QSize()

    def update_frame(self, frame: Optional[WavefrontFrame]) -> tuple[str, float]:
        """Convert the supplied frame to a cached ``QImage``.

        Returns a tuple of (image_source, square_size_mm) for QML bindings.
        """

        self._revision += 1

        if frame is None or frame.grid is None or frame.grid.size == 0:
            self._image = QImage()
            self._square_size_mm = 5.0
            self._cache_key = ""
            self._image_size = QSize()
            return self._cache_key, self._square_size_mm

        self._cache_key = f"wavefront/{self._revision}"

        grid = np.asarray(frame.grid)
        cols, rows = grid.shape[0], grid.shape[1]
        if cols <= 0 or rows <= 0:
            self._image = QImage()
            self._square_size_mm = float(getattr(frame, "square_size_mm", 5.0))
            self._image_size = QSize()
            return self._cache_key, self._square_size_mm

        self._image = QImage(cols, rows, QImage.Format.Format_RGBA8888)
        goal_marker = int(frame.goal_marker)
        max_value = int(frame.max_value) if frame.max_value > 0 else 0
        self._square_size_mm = float(frame.square_size_mm)

        for x in range(cols):
            column = grid[x]
            for y in range(rows):
                value = int(column[y])
                color = self._wavefront_color(value, goal_marker, max_value)
                # Flip Y so row 0 is at bottom (matching legacy viewer orientation).
                self._image.setPixelColor(x, rows - y - 1, color)

        self._image_size = QSize(self._image.width(), self._image.height())
        return self._cache_key, self._square_size_mm

    def requestImage(  # type: ignore[override]
        self,
        identifier: str,
        requestedSize: QSize,
    ):
        if not self._image.isNull():
            if requestedSize is not None and not requestedSize.isEmpty():
                return self._image.scaled(requestedSize), requestedSize
            return self._image, QSize(self._image.width(), self._image.height())
        return QImage(), QSize()

    @staticmethod
    def _wavefront_color(value: int, goal_marker: int, max_value: int) -> QColor:
        if value == goal_marker:
            return QColor.fromRgbF(0.24, 0.95, 0.35, 0.85)
        if value == 1:
            return QColor.fromRgbF(1.0, 0.94, 0.35, 0.85)
        if value < 0:
            return QColor.fromRgbF(0.95, 0.31, 0.31, 0.85)
        if max_value <= 0:
            return QColor.fromRgbF(0.31, 0.47, 0.62, 0.6)
        ratio = max(0.0, min(1.0, value / max_value))
        intensity = 40 + ratio * 180
        comp = intensity / 255.0
        return QColor.fromRgbF(comp, comp, comp, 0.75)


def _normalise_rgba(color: Sequence[float]) -> Tuple[float, float, float, float]:
    if len(color) == 3:
        r, g, b = color
        a = 1.0
    elif len(color) == 4:
        r, g, b, a = color
    else:
        raise ValueError(f"Expected RGB or RGBA tuple, got {color!r}")
    return (float(r), float(g), float(b), float(a))


def _line_segment(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    color: Tuple[float, float, float, float],
    *,
    width: float = 1.0,
    tag: Optional[str] = None,
) -> Polyline:
    return Polyline(points=(x0, y0, x1, y1), color_rgba=color, width_mm=width, tag=tag)


def _arc_polyline(parent: RRTNode, node: RRTNode, color: Tuple[float, float, float, float]) -> Polyline:
    """Sample the circular arc between a parent node and its child."""

    radius = float(node.radius or 0.0)
    if radius == 0.0:
        return _line_segment(parent.x, parent.y, node.x, node.y, color)

    direction = 1.0 if radius >= 0.0 else -1.0
    r = abs(radius)
    # Determine the arc centre using the same math as the legacy viewer.
    centre = geometry.translate(parent.x, parent.y).dot(
        geometry.aboutZ(parent.q + direction * pi / 2.0).dot(geometry.point(r))
    )
    theta = wrap_angle(parent.q - direction * pi / 2.0)
    target_theta = wrap_angle((node.q or 0.0) - direction * pi / 2.0)
    step = 0.05  # radians – matches legacy viewer fidelity

    points: List[float] = [float(parent.x), float(parent.y)]
    current_theta = theta
    current_x = float(parent.x)
    current_y = float(parent.y)
    while abs(current_theta - target_theta) > step:
        current_theta = wrap_angle(current_theta + direction * step)
        current_x = float(centre[0, 0] + r * cos(current_theta))
        current_y = float(centre[1, 0] + r * sin(current_theta))
        points.append(current_x)
        points.append(current_y)
    points.extend((float(node.x), float(node.y)))

    return Polyline(points=tuple(points), color_rgba=color, width_mm=1.0, tag="arc")


def _collect_robot_outline(rrt: RRT) -> Optional[RobotOutline]:
    node = rrt.start
    robot = getattr(rrt, "robot", None)
    robot_parts_to_node = getattr(rrt, "robot_parts_to_node", None)
    if node is None:
        robot_pose = getattr(robot, "pose", None)
        if robot_pose is None or not callable(robot_parts_to_node):
            return None
        node = RRTNode(
            x=float(getattr(robot_pose, "x", 0.0)),
            y=float(getattr(robot_pose, "y", 0.0)),
            q=float(getattr(robot_pose, "theta", 0.0)),
        )
    if not callable(robot_parts_to_node):
        return None
    parts = []
    for part in robot_parts_to_node(node):
        if isinstance(part, Circle):
            parts.append(
                CircleItem(
                    x=float(part.center[0, 0]),
                    y=float(part.center[1, 0]),
                    radius_mm=float(part.radius),
                    color_rgba=(1.0, 1.0, 0.0, 0.7),
                    filled=False,
                    tag="robot",
                )
            )
        elif isinstance(part, Rectangle):
            width = float(part.max_Ex - part.min_Ex)
            height = float(part.max_Ey - part.min_Ey)
            rotation = float(part.orient * 180.0 / pi)
            parts.append(
                RectangleItem(
                    x=float(part.center[0, 0]),
                    y=float(part.center[1, 0]),
                    width_mm=width,
                    height_mm=height,
                    rotation_deg=rotation,
                    color_rgba=(1.0, 1.0, 0.0, 0.7),
                    filled=False,
                    tag="robot",
                )
            )
    if not parts:
        return None
    return RobotOutline(parts=tuple(parts))


def _rectangle_from_shape(rect: Rectangle, color: Tuple[float, float, float, float], *, filled: bool, tag: Optional[str]) -> RectangleItem:
    width = float(rect.max_Ex - rect.min_Ex)
    height = float(rect.max_Ey - rect.min_Ey)
    rotation = float(rect.orient * 180.0 / pi)
    return RectangleItem(
        x=float(rect.center[0, 0]),
        y=float(rect.center[1, 0]),
        width_mm=width,
        height_mm=height,
        rotation_deg=rotation,
        color_rgba=color,
        filled=filled,
        tag=tag,
    )


def _circle_from_shape(circle: Circle, color: Tuple[float, float, float, float], *, filled: bool, tag: Optional[str]) -> CircleItem:
    return CircleItem(
        x=float(circle.center[0, 0]),
        y=float(circle.center[1, 0]),
        radius_mm=float(circle.radius),
        color_rgba=color,
        filled=filled,
        tag=tag,
    )


def _shape_to_item(
    shape: Shape,
    color: Tuple[float, float, float, float],
    *,
    filled: bool,
    tag: Optional[str],
) -> Optional[CircleItem | RectangleItem]:
    if isinstance(shape, Circle):
        return _circle_from_shape(shape, color, filled=filled, tag=tag)
    if isinstance(shape, Rectangle):
        return _rectangle_from_shape(shape, color, filled=filled, tag=tag)
    return None


# ---------------------------------------------------------------------------
# Public provider


class PathSceneProvider:
    """Transform ``robot.rrt`` and ``WaveFront`` state into renderable snapshots."""

    TREE_A_COLOR = _normalise_rgba((0.0, 1.0, 0.0, 1.0))
    TREE_B_COLOR = _normalise_rgba((0.0, 0.0, 1.0, 1.0))
    ARC_COLOR = _normalise_rgba((1.0, 1.0, 0.5, 1.0))
    PATH_COLOR = _normalise_rgba((1.0, 0.7, 0.0, 1.0))
    EXTRA_TREE_WIDTH_MM = 1.5

    def __init__(self) -> None:
        self._extra_trees: List[Tuple[Sequence[RRTNode], Tuple[float, float, float, float]]] = []
        self._revision = 0

    # Legacy compatibility -------------------------------------------------

    def clear_extra_trees(self) -> None:
        self._extra_trees.clear()

    def add_extra_tree(self, tree: Iterable[RRTNode], color: Sequence[float]) -> None:
        self._extra_trees.append((tuple(tree), _normalise_rgba(color)))

    # Snapshot generation --------------------------------------------------

    def build_snapshot(self, robot: object) -> PathSceneSnapshot:
        rrt = getattr(robot, "rrt", None)
        if rrt is None:
            self._revision += 1
            return PathSceneSnapshot.empty()

        markers: List[NodeMarker] = []
        edges: List[Polyline] = []
        bounds_x: List[float] = []
        bounds_y: List[float] = []

        def accumulate_tree(tree_nodes: Sequence[RRTNode], color: Tuple[float, float, float, float], tag: str) -> None:
            for node in tree_nodes:
                markers.append(
                    NodeMarker(
                        x=float(node.x),
                        y=float(node.y),
                        size_mm=4.0,
                        color_rgba=color,
                        tag=tag,
                    )
                )
                bounds_x.append(float(node.x))
                bounds_y.append(float(node.y))
                if node.parent:
                    if node.radius:
                        edges.append(_arc_polyline(node.parent, node, self.ARC_COLOR))
                    else:
                        edges.append(
                            _line_segment(
                                float(node.parent.x),
                                float(node.parent.y),
                                float(node.x),
                                float(node.y),
                                color=color,
                                width=1.0,
                                tag=tag,
                            )
                        )

        accumulate_tree(rrt.treeA or tuple(), self.TREE_A_COLOR, "treeA")
        accumulate_tree(rrt.treeB or tuple(), self.TREE_B_COLOR, "treeB")
        for index, (tree_nodes, color) in enumerate(self._extra_trees):
            accumulate_tree(tree_nodes, color, f"extra#{index}")

        # Path overlay (e.g. wavefront output)
        path_overlays: List[Polyline] = []
        draw_path = getattr(rrt, "draw_path", None)
        if draw_path:
            points: List[float] = []
            for pt in draw_path:
                if isinstance(pt, RRTNode):
                    x_val = float(pt.x)
                    y_val = float(pt.y)
                else:
                    x_val = float(pt[0])
                    y_val = float(pt[1])
                points.extend((x_val, y_val))
                bounds_x.append(x_val)
                bounds_y.append(y_val)
            if len(points) >= 4:
                path_overlays.append(Polyline(points=tuple(points), color_rgba=self.PATH_COLOR, width_mm=1.8, tag="draw_path"))

        # Obstacles and goal region
        obstacles: List[CircleItem | RectangleItem] = []
        for obstacle in getattr(rrt, "obstacles", []) or []:
            if isinstance(obstacle, Rectangle):
                width = float(obstacle.max_Ex - obstacle.min_Ex)
                height = float(obstacle.max_Ey - obstacle.min_Ey)
                color = (1.0, 1.0, 0.0, 0.5) if width > 10.0 * height else (1.0, 0.0, 0.0, 0.5)
            else:
                color = (1.0, 0.0, 0.0, 0.5)
            item = _shape_to_item(obstacle, _normalise_rgba(color), filled=True, tag="obstacle")
            if item is not None:
                obstacles.append(item)
                bounds_x.append(item.x)
                bounds_y.append(item.y)

        goal_region = getattr(rrt, "goal_obstacle", None)
        goal_item = None
        if goal_region is not None:
            goal_item = _shape_to_item(goal_region, _normalise_rgba((0.0, 1.0, 0.0, 0.5)), filled=True, tag="goal")
            if goal_item is not None:
                bounds_x.append(goal_item.x)
                bounds_y.append(goal_item.y)

        # Robot outline
        outline = _collect_robot_outline(rrt)
        if outline is not None:
            for part in outline.parts:
                bounds_x.append(part.x)
                bounds_y.append(part.y)

        # Wavefront grid
        wf = getattr(getattr(robot, "path_planner", None), "wf", None)
        wavefront_frame: Optional[WavefrontFrame] = None
        if isinstance(wf, WaveFront) and getattr(wf, "grid", None) is not None:
            grid = np.array(wf.grid, copy=True)
            unique_vals = np.unique(grid)
            max_value = int(unique_vals[-2]) if unique_vals.size >= 2 else int(unique_vals[-1]) if unique_vals.size else 0
            wavefront_frame = WavefrontFrame(
                grid=grid,
                square_size_mm=float(getattr(wf, "square_size", 5.0)),
                goal_marker=int(getattr(WaveFront, "goal_marker", 2**31 - 1)),
                max_value=max_value,
            )

        bounds = Bounds.from_points(bounds_x, bounds_y)

        self._revision += 1
        return PathSceneSnapshot(
            revision=self._revision,
            tree_markers=tuple(markers),
            tree_edges=tuple(edges),
            path_overlays=tuple(path_overlays),
            obstacles=tuple(obstacles),
            goal_region=goal_item,
            robot_outline=outline,
            wavefront=wavefront_frame,
            bounds=bounds,
            status_text=getattr(rrt, "text", None),
        )
