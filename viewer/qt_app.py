"""QtQuick runner wiring worldmap state and camera feeds into QML."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtSlot
from PyQt6.QtGui import QGuiApplication, QImage
from PyQt6.QtQuick import QQuickView
from PyQt6.QtQml import QQmlContext

from aim_fsm.camera import AIVISION_RESOLUTION_SCALE

from .camera_provider import CameraImageProvider
from .grid_provider import OccupancyGridProvider
from .snapshot_service import SnapshotService
from .worldmap_model import WorldMapModel
from .help_texts import CAMERA_HELP_TEXT


class QtViewerApp(QObject):
    """Bootstrap a ``QQuickView`` backed by ``WorldMapModel`` and camera feeds."""

    def __init__(
        self,
        robot: Any,
        worldmap: Optional[Any] = None,
        *,
        model: Optional[WorldMapModel] = None,
        camera_provider: Optional[CameraImageProvider] = None,
        wscale: float = 0.02,
        update_interval_ms: int = 33,
        autostart: bool = True,
        show: bool = False,
        snapshot_service: Optional[SnapshotService] = None,
        snapshots_enabled: bool = True,
    ) -> None:
        if robot is None:
            raise ValueError("robot instance is required")

        super().__init__(parent=None)
        self._robot = robot
        self._worldmap = worldmap or getattr(robot, "world_map", None)
        if self._worldmap is None:
            raise ValueError("worldmap instance is required")

        self._app = QGuiApplication.instance() or QGuiApplication([])
        self._model = model or WorldMapModel()
        self._camera_provider = camera_provider or CameraImageProvider()
        self._grid_provider = OccupancyGridProvider()
        
        # Connect grid provider
        if self._worldmap and hasattr(self._worldmap, 'occupancy_grid'):
            self._grid_provider.set_grid(self._worldmap.occupancy_grid)

        self._wscale = float(wscale)
        self._snapshot_service = snapshot_service or SnapshotService()
        self._snapshot_service.enable_disk_write(snapshots_enabled)
        self._camera_provider.set_robot_ref(self._robot)

        self._frame_counter = 0
        self._timer = QTimer(self)
        self._timer.setInterval(int(update_interval_ms))
        self._timer.timeout.connect(self.refresh)

        self._view = QQuickView()
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self._context = self._initialise_qml_context()

        self.refresh()
        if autostart and update_interval_ms > 0:
            self._timer.start()
        if show:
            self._view.show()

    # Public API ---------------------------------------------------

    @property
    def model(self) -> WorldMapModel:
        return self._model

    @property
    def camera_provider(self) -> CameraImageProvider:
        return self._camera_provider

    @property
    def view(self) -> QQuickView:
        return self._view

    def refresh(self) -> None:
        try:
            update = getattr(self._worldmap, "update", None)
            if callable(update):
                update()
        except Exception:
            # Debug output belongs to the caller; swallow to keep viewer resilient.
            pass

        if self._worldmap and hasattr(self._worldmap, 'occupancy_grid'):
            self._grid_provider.set_grid(self._worldmap.occupancy_grid)

        objects = getattr(self._worldmap, "objects", {}) or {}
        self._model.sync_from(self._robot, objects)

    def start(self) -> None:
        if not self._timer.isActive() and self._timer.interval() > 0:
            self._timer.start()

    def stop(self) -> None:
        self._timer.stop()

    def exec(self) -> int:
        return self._app.exec()

    def grab_window(self) -> QImage:
        return self._view.grabWindow()

    @pyqtSlot(result=str)
    def captureRawSnapshot(self) -> str:
        """Persist the most recent live frame and return the output path."""

        return self._capture_snapshot(annotated=False) or ""

    @pyqtSlot(result=str)
    def captureAnnotatedSnapshot(self) -> str:
        """Persist the most recent annotated frame (or fall back to live)."""

        return self._capture_snapshot(annotated=True) or ""

    @pyqtSlot(bool)
    def setCrosshairEnabled(self, enabled: bool) -> None:
        """Propagate crosshair state to the camera provider."""

        self._camera_provider.set_crosshair_enabled(bool(enabled))

    @pyqtSlot()
    def requestQuit(self) -> None:
        """Request termination of the Qt event loop."""

        self._app.quit()

    # Internal helpers --------------------------------------------

    def _initialise_qml_context(self) -> QQmlContext:
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"
        source = (qml_dir / "WorldMapView.qml").resolve()

        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))
        engine.addImageProvider("camera", self._camera_provider)
        engine.addImageProvider("grid", self._grid_provider)

        context = self._view.rootContext()
        
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

        context.setContextProperty("worldModel", self._model)
        context.setContextProperty("WSCALE", self._wscale)
        context.setContextProperty("AIVISION_RESOLUTION_SCALE", float(AIVISION_RESOLUTION_SCALE))
        context.setContextProperty("cameraFrameId", self._frame_counter)
        context.setContextProperty("viewerApp", self)
        context.setContextProperty("cameraHelpText", CAMERA_HELP_TEXT)

        self._camera_provider.register_notifier(self._queue_frame_bump)

        self._view.setSource(QUrl.fromLocalFile(str(source)))
        if self._view.status() == QQuickView.Status.Error:
            for error in self._view.errors() or []:
                print(error.toString())
            raise RuntimeError(f"Failed to load QML source: {source}")

        return context

    def _queue_frame_bump(self) -> None:
        if self.thread() == QGuiApplication.instance().thread():
            self._increment_frame()
        else:
            QTimer.singleShot(0, self._increment_frame)

    def _increment_frame(self) -> None:
        self._frame_counter += 1
        self._context.setContextProperty("cameraFrameId", self._frame_counter)

    def _capture_snapshot(self, *, annotated: bool) -> Optional[str]:
        if annotated:
            image = self._camera_provider.current_image("annotated")
            if image is None:
                image = self._camera_provider.current_image("live")
        else:
            image = self._camera_provider.current_image("live")

        path = self._snapshot_service.capture(image, annotated=annotated)
        if path is None:
            print("[QtViewerApp] Snapshot failed: no frame available" if image is None else
                  "[QtViewerApp] Snapshot failed: could not persist frame")
            return None

        print(f"[QtViewerApp] Snapshot saved to {path}")
        return str(path)


__all__ = ["QtViewerApp"]
