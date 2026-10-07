"""QtQuick-based replacement for the legacy OpenGL path + wavefront viewer."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtProperty, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQuick import QQuickView

from aim_fsm.rrt import RRT

from .help_texts import PATH_HELP_TEXT
from .lifecycle import stop_timer_if_view_hidden
from .path_model import (
    CircleItem,
    PathNodeModel,
    PathObstacleModel,
    PathPolylineModel,
    PathSceneProvider,
    PathSceneSnapshot,
    RectangleItem,
    WavefrontTextureProvider,
)


@dataclass
class _ViewState:
    """Simple struct tracking the shared centre / zoom for both sub-views."""

    center_x: float = 0.0
    center_y: float = 0.0
    zoom: float = 0.64


class PathViewState(QObject):
    """Expose view bounds + zoom to QML, mirroring the particle viewer."""

    changed = pyqtSignal()

    def __init__(self, *, center_x: float = 0.0, center_y: float = 0.0, zoom: float = 0.64, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._state = _ViewState(center_x=float(center_x), center_y=float(center_y), zoom=float(zoom))

    @pyqtProperty(float, notify=changed)
    def centerX(self) -> float:  # pragma: no cover - accessor
        return self._state.center_x

    @pyqtProperty(float, notify=changed)
    def centerY(self) -> float:  # pragma: no cover - accessor
        return self._state.center_y

    @pyqtProperty(float, notify=changed)
    def zoom(self) -> float:  # pragma: no cover - accessor
        return self._state.zoom

    @pyqtSlot(float, float)
    def setCenter(self, x: float, y: float) -> None:
        updated = False
        if float(x) != self._state.center_x:
            self._state.center_x = float(x)
            updated = True
        if float(y) != self._state.center_y:
            self._state.center_y = float(y)
            updated = True
        if updated:
            self.changed.emit()

    @pyqtSlot(float)
    def setZoom(self, zoom: float) -> None:
        zoom = max(0.01, float(zoom))
        if zoom != self._state.zoom:
            self._state.zoom = zoom
            self.changed.emit()


class PathViewer(QObject):
    """PyQt6 Path + Wavefront viewer preserving the legacy interface."""

    sceneChanged = pyqtSignal()

    def __init__(
        self,
        robot: Any,
        rrt: Optional[RRT] = None,
        *,
        width: int = 640,
        height: int = 640,
        windowName: str = "Path Viewer",
        update_interval_ms: int = 100,
    ) -> None:
        super().__init__(parent=None)
        if robot is None:
            raise ValueError("robot instance is required")

        self._robot = robot
        self._rrt = rrt or getattr(robot, "rrt", None)
        self._window_name = windowName
        self._width = int(width)
        self._height = int(height)
        self._auto_redisplay = update_interval_ms > 0

        self._app = QGuiApplication.instance() or QGuiApplication([])

        self._scene_provider = PathSceneProvider()
        self._node_model = PathNodeModel()
        self._edge_model = PathPolylineModel()
        self._path_model = PathPolylineModel()
        self._obstacle_model = PathObstacleModel()
        self._robot_model = PathObstacleModel()
        self._wavefront_provider = WavefrontTextureProvider()
        self._wavefront_source = ""
        self._wavefront_square_size = 5.0
        self._wavefront_origin_x = 0.0
        self._wavefront_origin_y = 0.0
        self._scene_snapshot: PathSceneSnapshot = PathSceneSnapshot.empty()
        self._view_state = PathViewState()
        self._help_text = PATH_HELP_TEXT

        self._timer = QTimer(self)
        self._timer.setInterval(max(0, int(update_interval_ms)))
        self._timer.timeout.connect(self.refresh)

        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self._bind_visibility_handler()

        self._context = self._initialise_qml_context()
        self.refresh()

    # ------------------------------------------------------------------
    # Public properties exposed to QML

    @pyqtProperty(PathViewState, constant=True)
    def viewState(self) -> PathViewState:
        return self._view_state

    @pyqtProperty(str, constant=True)
    def helpText(self) -> str:
        return self._help_text

    @pyqtProperty(str, notify=sceneChanged)
    def wavefrontSource(self) -> str:
        return self._wavefront_source

    @pyqtProperty(float, notify=sceneChanged)
    def wavefrontSquareSize(self) -> float:
        return self._wavefront_square_size

    @pyqtProperty(float, notify=sceneChanged)
    def wavefrontOriginX(self) -> float:
        return self._wavefront_origin_x

    @pyqtProperty(float, notify=sceneChanged)
    def wavefrontOriginY(self) -> float:
        return self._wavefront_origin_y

    @pyqtProperty(str, notify=sceneChanged)
    def statusText(self) -> str:
        return self._scene_snapshot.status_text or ""

    # ------------------------------------------------------------------
    # Legacy-compatible surface

    def start(self) -> None:
        import threading
        if self._auto_redisplay and not self._timer.isActive() and self._timer.interval() > 0:
            self._timer.start()
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        self._view.show()
        self._focus_root()
        #print(self._help_text, end="")

    def stop(self) -> None:
        self._timer.stop()
        self._view.close()

    def is_visible(self) -> bool:
        return self._view.isVisible()

    def clear(self) -> None:
        self._scene_provider.clear_extra_trees()
        rrt = self._rrt or getattr(self._robot, "rrt", None)
        if rrt is not None:
            rrt.treeA = []
            rrt.treeB = []
            rrt.path = []
            rrt.draw_path = []
            rrt.obstacles = []
            rrt.goal_obstacle = None
            rrt.grid_display = None
            rrt.text = None
        path_planner = getattr(self._robot, "path_planner", None)
        if path_planner is not None:
            try:
                path_planner.wf = None
            except Exception:
                pass
        self.refresh()

    def add_tree(self, tree, color) -> None:
        self._scene_provider.add_extra_tree(tree, color)
        self.refresh()

    def refresh(self) -> None:
        self._scene_snapshot = self._scene_provider.build_snapshot(self._robot)
        snapshot = self._scene_snapshot

        self._node_model.sync_from(snapshot.tree_markers)
        self._edge_model.sync_from(snapshot.tree_edges)
        self._path_model.sync_from(snapshot.path_overlays)

        goal_extra: Optional[tuple[CircleItem | RectangleItem, ...]]
        if snapshot.goal_region is not None:
            goal_extra = (snapshot.goal_region,)
        else:
            goal_extra = None
        self._obstacle_model.sync_from(snapshot.obstacles, goal_extra)

        robot_parts: tuple[CircleItem | RectangleItem, ...] = ()
        if snapshot.robot_outline is not None:
            robot_parts = snapshot.robot_outline.parts
        self._robot_model.sync_from(robot_parts)

        source_key, square_size = self._wavefront_provider.update_frame(snapshot.wavefront)
        self._wavefront_source = f"image://wavefront/{source_key}" if source_key else ""
        self._wavefront_square_size = square_size
        if snapshot.wavefront is not None:
            self._wavefront_origin_x = float(snapshot.wavefront.origin_x)
            self._wavefront_origin_y = float(snapshot.wavefront.origin_y)
        else:
            self._wavefront_origin_x = 0.0
            self._wavefront_origin_y = 0.0

        self.sceneChanged.emit()

    # ------------------------------------------------------------------
    # Slots exposed to QML (placeholders until we wire shortcuts)

    @pyqtSlot()
    def printHelp(self) -> None:
        print(self._help_text, end="")

    @pyqtSlot()
    def toggleRedisplay(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
            self._auto_redisplay = False
            print("[PathViewer] Redisplay off.")
        else:
            if self._timer.interval() > 0:
                self._timer.start()
                self._auto_redisplay = True
                print("[PathViewer] Redisplay on.")
            else:
                print("[PathViewer] Update interval is 0; nothing to toggle.")

    @pyqtSlot()
    def requestRefresh(self) -> None:
        self.refresh()

    @pyqtSlot()
    def requestQuit(self) -> None:
        self.stop()

    @pyqtSlot()
    def centerView(self) -> None:
        self._view_state.setCenter(0.0, 0.0)

    @pyqtSlot()
    def showObjects(self) -> None:
        world_map = getattr(self._robot, "world_map", None)
        if world_map is None:
            print("[PathViewer] world_map unavailable; cannot show objects")
            return
        show_objects = getattr(world_map, "show_objects", None)
        if callable(show_objects):
            show_objects()

    @pyqtSlot()
    def showObstacles(self) -> None:
        rrt = getattr(self._robot, "rrt", None)
        if rrt is None:
            print("[PathViewer] RRT unavailable; cannot list obstacles.")
            return
        obstacles = getattr(rrt, "obstacles", None)
        if not obstacles:
            print("[PathViewer] No obstacles to display.")
            return
        print(f"RRT has {len(obstacles)} obstacles.")
        for obstacle in obstacles:
            print("  ", obstacle)
        print()

    @pyqtSlot()
    def showPose(self) -> None:
        robot = getattr(self._robot, "robot", self._robot)
        show_pose = getattr(robot, "show_pose", None)
        if callable(show_pose):
            show_pose()
        else:
            print("[PathViewer] show_pose() unavailable on robot.")

    # ------------------------------------------------------------------
    # Internal helpers

    def _initialise_qml_context(self):
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"
        qml_path = (qml_dir / "PathViewer.qml").resolve()

        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))
        engine.addImageProvider("wavefront", self._wavefront_provider)

        context = self._view.rootContext()
        context.setContextProperty("viewerApp", self)
        context.setContextProperty("viewState", self._view_state)
        context.setContextProperty("pathNodeModel", self._node_model)
        context.setContextProperty("pathEdgeModel", self._edge_model)
        context.setContextProperty("pathOverlayModel", self._path_model)
        context.setContextProperty("pathObstacleModel", self._obstacle_model)
        context.setContextProperty("pathRobotModel", self._robot_model)

        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))
        if self._view.status() == QQuickView.Status.Error:
            errors = "\n".join(error.toString() for error in self._view.errors())
            raise RuntimeError(f"Failed to load PathViewer.qml:\n{errors}")
        return context

    def _bind_visibility_handler(self) -> None:
        signal = getattr(self._view, "visibleChanged", None)
        if signal is None:
            signal = getattr(self._view, "visibilityChanged", None)
        if signal is not None:
            signal.connect(self._handle_visibility_changed)

    def _handle_visibility_changed(self, *args) -> None:
        stop_timer_if_view_hidden(self._view, self._timer)

    def _focus_root(self) -> None:
        try:
            root = self._view.rootObject()
        except Exception:
            return
        if root is not None and hasattr(root, "forceActiveFocus"):
            root.forceActiveFocus()

    @staticmethod
    def _shape_dict(item: CircleItem | RectangleItem) -> dict:
        if isinstance(item, CircleItem):
            return {
                "type": "circle",
                "x": float(item.x),
                "y": float(item.y),
                "radius": float(item.radius_mm),
                "width": 0.0,
                "height": 0.0,
                "rotation": 0.0,
                "color": list(item.color_rgba),
                "filled": bool(item.filled),
                "tag": item.tag or "",
            }
        if isinstance(item, RectangleItem):
            return {
                "type": "rectangle",
                "x": float(item.x),
                "y": float(item.y),
                "radius": 0.0,
                "width": float(item.width_mm),
                "height": float(item.height_mm),
                "rotation": float(item.rotation_deg),
                "color": list(item.color_rgba),
                "filled": bool(item.filled),
                "tag": item.tag or "",
            }
        return {}

    # ------------------------------------------------------------------
    # Data access for QML (temporary getters until models exist)

    @pyqtSlot(result=float)
    def sceneRevision(self) -> float:
        return float(self._scene_snapshot.revision)

    @pyqtSlot(result="QVariantList")
    def treeMarkers(self):
        return [
            {"x": marker.x, "y": marker.y, "size": marker.size_mm, "color": marker.color_rgba, "tag": marker.tag}
            for marker in self._scene_snapshot.tree_markers
        ]

    @pyqtSlot(result="QVariantList")
    def treeEdges(self):
        return [
            {"points": edge.points, "color": edge.color_rgba, "width": edge.width_mm, "tag": edge.tag}
            for edge in self._scene_snapshot.tree_edges
        ]

    @pyqtSlot(result="QVariantList")
    def pathOverlays(self):
        return [
            {"points": overlay.points, "color": overlay.color_rgba, "width": overlay.width_mm, "tag": overlay.tag}
            for overlay in self._scene_snapshot.path_overlays
        ]

    @pyqtSlot(result="QVariantList")
    def obstacles(self):
        return [self._shape_dict(item) for item in self._scene_snapshot.obstacles]

    @pyqtSlot(result="QVariant")
    def goalRegion(self):
        goal = self._scene_snapshot.goal_region
        if goal is None:
            return None
        return self._shape_dict(goal)

    @pyqtSlot(result="QVariant")
    def wavefront(self):
        wf = self._scene_snapshot.wavefront
        if wf is None:
            return None
        return {
            "squareSize": wf.square_size_mm,
            "goalMarker": wf.goal_marker,
            "maxValue": wf.max_value,
            "grid": wf.grid.tolist(),
        }

    @pyqtSlot(result="QVariant")
    def bounds(self):
        bounds = self._scene_snapshot.bounds
        if bounds is None:
            return None
        return {"minX": bounds.min_x, "minY": bounds.min_y, "maxX": bounds.max_x, "maxY": bounds.max_y}
