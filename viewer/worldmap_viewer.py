"""QtQuick-based replacement for the legacy OpenGL world map viewer."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Optional

from PyQt6.QtCore import QObject, QSize, Qt, QTimer, QUrl, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter, QVector3D
from PyQt6.QtQuick import QQuickImageProvider, QQuickView

from aim_fsm.camera import AIVISION_RESOLUTION_SCALE  # legacy code expects this in scope

from .help_texts import WORLD_HELP_TEXT
from .worldmap_model import WorldMapModel
from .grid_provider import OccupancyGridProvider


class TagTextureProvider(QQuickImageProvider):
    """Image provider that renders AprilTag ID numbers as textures."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._cache: dict[str, QImage] = {}
        # Texture dimensions: After text panel rotation, texture U maps to height, V maps to width
        # So texture width:height should match AprilTag height:width = 48:38 = 24:19
        self._texture_width = 96   # 24×4, maps to AprilTag height (48mm) after rotation
        self._texture_height = 76  # 19×4, maps to AprilTag width (38mm) after rotation

    def requestImage(self, id: str, requestedSize: QSize):  # type: ignore[override]
        """Generate or retrieve cached texture for the given tag ID.
        
        Args:
            id: The tag ID string (e.g., "5", "42")
            requestedSize: Requested size (QSize)
            
        Returns:
            Tuple of (QImage, QSize)
        """
        result_size = QSize(self._texture_width, self._texture_height)

        if not id or id == "null" or id == "None":
            # Return transparent image for missing IDs
            img = QImage(self._texture_width, self._texture_height, QImage.Format.Format_RGBA8888)
            img.fill(QColor(0, 0, 0, 0))
            return img, result_size

        # Check cache
        if id in self._cache:
            return self._cache[id], result_size

        # Generate new texture with correct aspect ratio
        img = QImage(self._texture_width, self._texture_height, QImage.Format.Format_RGBA8888)
        img.fill(QColor(128, 76, 230, 255))  # Purple background matching AprilTag color

        painter = QPainter(img)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        # White text, bold font
        painter.setPen(QColor(255, 255, 255, 255))
        font = QFont("Arial", 72, QFont.Weight.Bold)
        painter.setFont(font)

        # Center the text
        rect = img.rect()
        painter.drawText(rect, 0x84, id)  # 0x84 = AlignCenter | AlignVCenter

        painter.end()

        # Cache the result
        self._cache[id] = img
        return img, result_size


class WorldMapViewer(QObject):
    """Drop-in replacement for :mod:`aim_fsm.legacy.worldmap_viewer`."""

    WSCALE = 0.02
    gridFrameBumpRequested = pyqtSignal()

    def __init__(
        self,
        robot: Any,
        width: int = 640,
        height: int = 640,
        windowName: str = "VEX AIM's World",
        update_interval_ms: int = 66,
    ) -> None:
        super().__init__(parent=None)
        if robot is None:
            raise ValueError("robot instance is required")

        worldmap = getattr(robot, "world_map", None)
        if worldmap is None:
            raise ValueError("robot.world_map is required for WorldMapViewer")
        print("[WorldMapViewer] __init__")

        self._robot = robot
        self._worldmap = worldmap
        self._width = int(width)
        self._height = int(height)
        self._window_name = windowName

        self._app = QGuiApplication.instance() or QGuiApplication([])
        self._model = WorldMapModel()
        self._tag_texture_provider = TagTextureProvider()
        self._grid_provider = OccupancyGridProvider()
        self._grid_frame_counter = 0
        self._grid_bump_pending = False

        # Always deliver grid frame bumps on the Qt (GUI) thread.
        self.gridFrameBumpRequested.connect(self._increment_grid_frame, Qt.ConnectionType.QueuedConnection)

        # Connect grid provider
        print("[WorldMapViewer] Connecting grid provider...")
        if self._worldmap and hasattr(self._worldmap, 'occupancy_grid'):
            grid = self._worldmap.occupancy_grid
            print(f"[WorldMapViewer] Found grid, ID={id(grid)}")
            self._grid_provider.set_grid(grid)

            # Register notifier for grid updates (thread-safe)
            original_on_update = grid.on_update

            def on_grid_update():
                original_on_update()  # GridProvider._mark_dirty
                self._queue_grid_frame_bump()  # Increment counter (thread-safe)

            grid.on_update = on_grid_update
            print("[WorldMapViewer] Grid callback registered")

        print("[WorldMapViewer] Creating timer...")
        self._timer = QTimer(self)
        self._timer.setInterval(int(update_interval_ms))
        self._timer.timeout.connect(self.refresh)

        print("[WorldMapViewer] Creating QQuickView...")
        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)

        print("[WorldMapViewer] Initializing QML context...")
        self._context = self._initialise_qml_context()
        print("[WorldMapViewer] Calling first refresh...")
        self.refresh()
        print("[WorldMapViewer] __init__ DONE")

    # ------------------------------------------------------------------
    # Legacy-compatible surface

    def start(self) -> None:
        print("[WorldMapViewer] start() called")
        if not self._timer.isActive() and self._timer.interval() > 0:
            self._timer.start()
            print(f"[WorldMapViewer] Timer started, interval={self._timer.interval()}ms")
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        print(f"[WorldMapViewer] View size set to {self._width}x{self._height}")
        self._view.show()
        print("[WorldMapViewer] View shown")
        self._focus_root()
        self.frame_scene()
        print(WORLD_HELP_TEXT, end="")
        print("[WorldMapViewer] start() DONE")

    def stop(self) -> None:
        self._timer.stop()
        self._view.close()

    def refresh(self) -> None:
        try:
            update = getattr(self._worldmap, "update", None)
            if callable(update):
                update()
        except Exception:
            pass

        # Dynamically ensure grid provider follows the current grid instance
        if self._worldmap and hasattr(self._worldmap, 'occupancy_grid'):
            self._grid_provider.set_grid(self._worldmap.occupancy_grid)

        # Always use worldmap.objects (shared_map is Cozmo holdover code, not implemented for VEX AIM)
        objects = getattr(self._worldmap, "objects", {}) or {}
        self._model.sync_from(self._robot, objects)

    def grab_window(self) -> QImage:
        return self._view.grabWindow()

    @property
    def model(self) -> WorldMapModel:
        return self._model

    @property
    def view(self) -> QQuickView:
        return self._view

    def root_object(self):
        try:
            return self._view.rootObject()
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Slots exposed to QML

    @pyqtSlot()
    def printHelp(self) -> None:
        print(WORLD_HELP_TEXT, end="")

    def _queue_grid_frame_bump(self) -> None:
        """Coalesce grid update notifications into a single Qt-queued bump."""
        if self._grid_bump_pending:
            return
        self._grid_bump_pending = True
        try:
            self.gridFrameBumpRequested.emit()
        except Exception:
            self._grid_bump_pending = False

    @pyqtSlot()
    def _increment_grid_frame(self) -> None:
        """Increment grid frame counter and notify QML (Qt main thread only)."""
        self._grid_bump_pending = False
        self._grid_frame_counter += 1

        # Call QML function instead of setContextProperty (breaks binding loop)
        root = self.root_object()
        if root and hasattr(root, 'updateGridFrame'):
            root.updateGridFrame(self._grid_frame_counter)

    @pyqtSlot()
    def frame_scene(self) -> None:
        # Auto framing removed - user controls camera manually
        pass

    # ------------------------------------------------------------------
    # Internal helpers

    def _initialise_qml_context(self):
        print("[WorldMapViewer] _initialise_qml_context START")
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"
        qml_path = (qml_dir / "WorldMapView.qml").resolve()
        print(f"[WorldMapViewer] QML path: {qml_path}")

        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))
        print("[WorldMapViewer] Adding image providers...")
        engine.addImageProvider("tagtexture", self._tag_texture_provider)
        engine.addImageProvider("grid", self._grid_provider)
        print("[WorldMapViewer] Image providers added")

        context = self._view.rootContext()
        print("[WorldMapViewer] Got root context")
        
        # Expose Grid Properties
        grid = getattr(self._worldmap, 'occupancy_grid', None)
        if grid:
            context.setContextProperty("GRID_X_MIN", float(grid.x_min))
            context.setContextProperty("GRID_Y_MIN", float(grid.y_min))
            context.setContextProperty("GRID_WIDTH_MM", float(grid.x_max - grid.x_min))
            context.setContextProperty("GRID_HEIGHT_MM", float(grid.y_max - grid.y_min))
        else:
            # Defaults matching OccupancyGrid defaults
            context.setContextProperty("GRID_X_MIN", -2500.0)
            context.setContextProperty("GRID_Y_MIN", -2500.0)
            context.setContextProperty("GRID_WIDTH_MM", 5000.0)
            context.setContextProperty("GRID_HEIGHT_MM", 5000.0)

        print("[WorldMapViewer] Setting context properties...")
        context.setContextProperty("worldModel", self._model)
        context.setContextProperty("WSCALE", float(self.WSCALE))
        context.setContextProperty("AIVISION_RESOLUTION_SCALE", float(AIVISION_RESOLUTION_SCALE))
        context.setContextProperty("gridFrameId", self._grid_frame_counter)
        context.setContextProperty("viewerApp", self)
        print(f"[WorldMapViewer] Context properties set, gridFrameId={self._grid_frame_counter}")

        print(f"[WorldMapViewer] Loading QML from: {qml_path}")
        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))
        print(f"[WorldMapViewer] QML loaded, status={self._view.status()}")
        if self._view.status() == QQuickView.Status.Error:
            errors = "\n".join(error.toString() for error in self._view.errors())
            raise RuntimeError(f"Failed to load WorldMapView.qml:\n{errors}")
        print("[WorldMapViewer] _initialise_qml_context DONE")
        return context

    def _focus_root(self) -> None:
        try:
            root = self._view.rootObject()
        except Exception:
            return
        if root is not None and hasattr(root, "forceActiveFocus"):
            root.forceActiveFocus()


__all__ = ["WorldMapViewer"]
