"""QtQuick-based replacement for the legacy OpenGL world map viewer."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Optional

from PyQt6.QtCore import QObject, QSize, QTimer, QUrl, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter, QVector3D
from PyQt6.QtQuick import QQuickImageProvider, QQuickView

from aim_fsm.camera import AIVISION_RESOLUTION_SCALE  # legacy code expects this in scope

from .help_texts import WORLD_HELP_TEXT
from .lifecycle import stop_timer_if_view_hidden
from .worldmap_model import WorldMapModel


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
            id: The tag ID string (e.g., "5", "42", "aruco-5")
            requestedSize: Requested size (QSize)
            
        Returns:
            Tuple of (QImage, QSize)
        """
        result_size = QSize(self._texture_width, self._texture_height)
        cache_key = id or ""

        if not cache_key or cache_key == "null" or cache_key == "None":
            # Return transparent image for missing IDs
            img = QImage(self._texture_width, self._texture_height, QImage.Format.Format_RGBA8888)
            img.fill(QColor(0, 0, 0, 0))
            return img, result_size

        # Check cache
        if cache_key in self._cache:
            return self._cache[cache_key], result_size

        texture_id = cache_key
        reverse_text = False
        if texture_id.startswith("back-"):
            reverse_text = True
            texture_id = texture_id[len("back-") :]

        label = texture_id
        background = QColor(128, 76, 230, 255)  # Purple background matching AprilTag color
        text_color = QColor(255, 255, 255, 255)
        if texture_id.startswith("aruco-"):
            label = texture_id.split("-", 1)[1]
            background = QColor(0, 255, 0, 255)  # Bright green for ArUco
            text_color = QColor(0, 0, 0, 255)

        # Generate new texture with correct aspect ratio
        img = QImage(self._texture_width, self._texture_height, QImage.Format.Format_RGBA8888)
        img.fill(background)

        painter = QPainter(img)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        # Bold font
        painter.setPen(text_color)
        font = QFont("Arial", 72, QFont.Weight.Bold)
        painter.setFont(font)

        # Center the text
        rect = img.rect()
        painter.drawText(rect, 0x84, label)  # 0x84 = AlignCenter | AlignVCenter

        painter.end()

        if reverse_text:
            img = img.mirrored(True, False)

        # Cache the result
        self._cache[cache_key] = img
        return img, result_size


class WorldMapViewer(QObject):
    """Drop-in replacement for :mod:`aim_fsm.legacy.worldmap_viewer`."""

    WSCALE = 0.02

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

        self._robot = robot
        self._worldmap = worldmap
        self._width = int(width)
        self._height = int(height)
        self._window_name = windowName
        self._last_refresh_error_ts = 0.0

        self._app = QGuiApplication.instance() or QGuiApplication([])
        self._model = WorldMapModel()
        self._tag_texture_provider = TagTextureProvider()

        self._timer = QTimer(self)
        self._timer.setInterval(int(update_interval_ms))
        self._timer.timeout.connect(self.refresh)

        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self._bind_visibility_handler()

        self._context = self._initialise_qml_context()
        self.refresh()

    # ------------------------------------------------------------------
    # Legacy-compatible surface

    def start(self) -> None:
        if not self._timer.isActive() and self._timer.interval() > 0:
            self._timer.start()
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        self._view.show()
        self._focus_root()
        self.frame_scene()
        #print(WORLD_HELP_TEXT, end="")

    def stop(self) -> None:
        self._timer.stop()
        self._view.close()

    def is_visible(self) -> bool:
        return self._view.isVisible()

    def refresh(self) -> None:
        snapshot = getattr(self._worldmap, "snapshot_objects", None)
        try:
            if callable(snapshot):
                objects = snapshot() or {}
            else:
                objects = dict(getattr(self._worldmap, "objects", {}) or {})
        except Exception as exc:
            self._log_refresh_error(exc)
            objects = {}
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

    @pyqtSlot()
    def frame_scene(self) -> None:
        # Auto framing removed - user controls camera manually
        pass

    # ------------------------------------------------------------------
    # Internal helpers

    def _initialise_qml_context(self):
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"
        qml_path = (qml_dir / "WorldMapView.qml").resolve()

        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))
        engine.addImageProvider("tagtexture", self._tag_texture_provider)

        context = self._view.rootContext()
        context.setContextProperty("worldModel", self._model)
        context.setContextProperty("WSCALE", float(self.WSCALE))
        context.setContextProperty("AIVISION_RESOLUTION_SCALE", float(AIVISION_RESOLUTION_SCALE))
        context.setContextProperty("viewerApp", self)

        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))
        if self._view.status() == QQuickView.Status.Error:
            errors = "\n".join(error.toString() for error in self._view.errors())
            raise RuntimeError(f"Failed to load WorldMapView.qml:\n{errors}")
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

    def _log_refresh_error(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self._last_refresh_error_ts >= 1.0:
            print(f"[WorldMapViewer] refresh failed: {exc}")
            self._last_refresh_error_ts = now


__all__ = ["WorldMapViewer"]
