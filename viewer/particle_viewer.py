"""QtQuick-based particle viewer replacing the legacy OpenGL implementation."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Optional

import numpy as np
from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtProperty, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQuick import QQuickView

try:
    from aim_fsm.camera import AIVISION_RESOLUTION_SCALE  # legacy API parity
except ImportError:  # pragma: no cover - optional during standalone tools
    AIVISION_RESOLUTION_SCALE = 1.0

from .help_texts import PARTICLE_HELP_TEXT
from .lifecycle import stop_timer_if_view_hidden
from .particle_model import LandmarkModel, ParticleLayerModel, ParticleSummary


class ParticleViewState(QObject):
    """Shared view bounds/zoom state exposed to QML."""

    changed = pyqtSignal()

    def __init__(
        self,
        center_x: float = 0.0,
        center_y: float = 0.0,
        zoom: float = 1.0,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._center_x = float(center_x)
        self._center_y = float(center_y)
        self._zoom = float(zoom)

    @pyqtProperty(float, notify=changed)
    def centerX(self) -> float:  # pragma: no cover - trivial accessor
        return self._center_x

    @pyqtProperty(float, notify=changed)
    def centerY(self) -> float:  # pragma: no cover - trivial accessor
        return self._center_y

    @pyqtProperty(float, notify=changed)
    def zoom(self) -> float:  # pragma: no cover - trivial accessor
        return self._zoom

    @pyqtSlot(float, float)
    def setCenter(self, x: float, y: float) -> None:
        updated = False
        if x != self._center_x:
            self._center_x = float(x)
            updated = True
        if y != self._center_y:
            self._center_y = float(y)
            updated = True
        if updated:
            self.changed.emit()

    @pyqtSlot(float)
    def setZoom(self, zoom: float) -> None:
        zoom = max(0.01, float(zoom))
        if zoom != self._zoom:
            self._zoom = zoom
            self.changed.emit()


class ParticleViewer(QObject):
    """Drop-in replacement for :mod:`aim_fsm.legacy.particle_viewer`."""

    def __init__(
        self,
        robot: Any,
        width: int = 640,
        height: int = 640,
        scale: float = 0.64,  # legacy compatibility; now used as initial zoom hint
        windowName: str = "Particle Viewer",
        bgcolor: tuple[float, float, float] | tuple[int, int, int] = (0, 0, 0),  # legacy compatibility
        update_interval_ms: int = 50,
    ) -> None:
        super().__init__(parent=None)
        if robot is None:
            raise ValueError("robot instance is required")

        self._robot = robot
        self._width = int(width)
        self._height = int(height)
        self._window_name = windowName

        self._app = QGuiApplication.instance() or QGuiApplication([])

        self._particle_model = ParticleLayerModel()
        self._landmark_model = LandmarkModel()
        self._summary = ParticleSummary()
        initial_zoom = float(scale)
        if math.isclose(initial_zoom, 0.64, rel_tol=0.0, abs_tol=1e-9):
            initial_zoom = 1.0
        self._view_state = ParticleViewState(zoom=initial_zoom)
        self._auto_center = False
        self._verbose = False
        self._update_interval_ms = max(0, int(update_interval_ms))
        self._redisplay_enabled = self._update_interval_ms > 0

        self._timer = QTimer(self)
        self._timer.setInterval(self._update_interval_ms)
        self._timer.timeout.connect(self.refresh)

        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self._bind_visibility_handler()

        self._context = self._initialise_qml_context()
        self.refresh()

    # ------------------------------------------------------------------
    # Legacy-compatible interface

    def start(self) -> None:
        if self._redisplay_enabled and not self._timer.isActive() and self._timer.interval() > 0:
            self._timer.start()
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        self._view.show()
        self._focus_root()
        #print(PARTICLE_HELP_TEXT, end="")

    def stop(self) -> None:
        self._timer.stop()
        self._view.close()

    def is_visible(self) -> bool:
        return self._view.isVisible()

    def refresh(self) -> None:
        particle_filter = getattr(self._robot, "particle_filter", None)
        world_map = getattr(self._robot, "world_map", None)

        self._particle_model.sync_from(particle_filter)
        self._landmark_model.sync_from(particle_filter, world_map)
        self._summary.sync_from(particle_filter)

        if self._auto_center and self._summary.isValid:
            self._view_state.setCenter(self._summary.poseX, self._summary.poseY)
        if self._verbose:
            self._print_pose()

    @property
    def particle_model(self) -> ParticleLayerModel:
        return self._particle_model

    @property
    def landmark_model(self) -> LandmarkModel:
        return self._landmark_model

    @property
    def summary(self) -> ParticleSummary:
        return self._summary

    # ------------------------------------------------------------------
    # Slots exposed to QML

    @pyqtSlot()
    def printHelp(self) -> None:
        print(PARTICLE_HELP_TEXT, end="")

    @pyqtSlot()
    def toggleAutoCenter(self) -> None:
        self._auto_center = not self._auto_center
        state = "on" if self._auto_center else "off"
        print(f"[ParticleViewer] Auto-center {state}.")

    @pyqtSlot()
    def requestRefresh(self) -> None:
        self.refresh()

    @pyqtSlot()
    def requestQuit(self) -> None:
        self.stop()

    @pyqtSlot()
    def toggleRedisplay(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
            self._redisplay_enabled = False
            print("[ParticleViewer] Redisplay off.")
        else:
            if self._update_interval_ms > 0:
                self._timer.start()
                self._redisplay_enabled = True
                print("[ParticleViewer] Redisplay on.")
            else:
                print("[ParticleViewer] Redisplay interval is 0; nothing to toggle.")

    @pyqtSlot(float)
    def driveForward(self, distance_mm: float) -> None:
        self._drive_command("forward", float(distance_mm))
        self.refresh()

    @pyqtSlot(float)
    def strafe(self, distance_mm: float) -> None:
        self._drive_command("sideways", float(distance_mm))
        self.refresh()

    @pyqtSlot(float)
    def turnDegrees(self, angle_deg: float) -> None:
        radians = float(angle_deg) * math.pi / 180.0
        self._drive_command("turn", radians)
        self.refresh()

    @pyqtSlot()
    def drop(self) -> None:
        self._drop_command()
        self.refresh()

    @pyqtSlot()
    def evaluateParticles(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        sensor_model = getattr(pf, "sensor_model", None)
        if sensor_model is None:
            print("[ParticleViewer] sensor model unavailable; cannot evaluate particles")
            return
        try:
            sensor_model.evaluate(pf.particles, force=True)
            pf.update_weights()
        except Exception as exc:  # pragma: no cover - defensive logging
            print(f"[ParticleViewer] evaluate failed: {exc}")
        self.refresh()

    @pyqtSlot()
    def resampleParticles(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        sensor_model = getattr(pf, "sensor_model", None)
        if sensor_model is not None:
            try:
                sensor_model.evaluate(pf.particles, force=True)
                pf.update_weights()
            except Exception as exc:  # pragma: no cover
                print(f"[ParticleViewer] evaluate prior to resample failed: {exc}")
        try:
            pf.resample()
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] resample failed: {exc}")
        self.refresh()

    @pyqtSlot()
    def resetParticles(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        try:
            pf.delocalize()
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] delocalize failed: {exc}")
        self.refresh()

    @pyqtSlot()
    def jitterParticles(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        try:
            pf.increase_variance()
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] increase_variance failed: {exc}")
        self.refresh()

    @pyqtSlot()
    def clearLandmarks(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        try:
            pf.clear_landmarks()
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] clear_landmarks failed: {exc}")
        self.refresh()

    @pyqtSlot()
    def showLandmarks(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        try:
            pf.show_landmarks()
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] show_landmarks failed: {exc}")

    @pyqtSlot()
    def showObjects(self) -> None:
        world_map = getattr(self._robot, "world_map", None)
        if world_map is None:
            print("[ParticleViewer] world_map unavailable; cannot show objects")
            return
        show_objects = getattr(world_map, "show_objects", None)
        if callable(show_objects):
            show_objects()

    @pyqtSlot()
    def showPose(self) -> None:
        show_pose = getattr(self._robot, "show_pose", None)
        if callable(show_pose):
            show_pose()

    @pyqtSlot()
    def showBestParticle(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        show_particle = getattr(pf, "show_particle", None)
        if callable(show_particle):
            show_particle([])

    @pyqtSlot()
    def reportVariance(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        particles = getattr(pf, "particles", [])
        if not particles:
            print("[ParticleViewer] no particles available for variance report")
            return
        weights = np.array([getattr(p, "weight", 0.0) for p in particles], dtype=float)
        weights.sort()
        variance = float(np.var(weights))
        mid_index = len(weights) // 2
        print(
            "weights:  min = {0:3.3e}  max = {1:3.3e} med = {2:3.3e}  variance = {3:3.3e}".format(
                weights[0], weights[-1], weights[mid_index], variance
            )
        )
        xy_var, theta_var = getattr(pf, "variance", (None, None))
        print("xy_var=", xy_var, "  theta_var=", theta_var)

    @pyqtSlot()
    def toggleVerbose(self) -> None:
        self._verbose = not self._verbose
        state = "on" if self._verbose else "off"
        print(f"[ParticleViewer] verbose mode {state}")
        if self._verbose:
            self._print_pose()

    # ------------------------------------------------------------------
    # Internal helpers

    def _initialise_qml_context(self):
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"

        qml_path = (qml_dir / "ParticleView.qml").resolve()
        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))

        context = self._view.rootContext()
        context.setContextProperty("particleModel", self._particle_model)
        context.setContextProperty("landmarkModel", self._landmark_model)
        context.setContextProperty("particleSummary", self._summary)
        context.setContextProperty("viewState", self._view_state)
        context.setContextProperty("AIVISION_RESOLUTION_SCALE", float(AIVISION_RESOLUTION_SCALE))
        context.setContextProperty("viewerApp", self)

        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))
        if self._view.status() == QQuickView.Status.Error:
            errors = "\n".join(error.toString() for error in self._view.errors())
            raise RuntimeError(f"Failed to load ParticleView.qml:\n{errors}")
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

    def _particle_filter(self):
        pf = getattr(self._robot, "particle_filter", None)
        if pf is None:
            print("[ParticleViewer] particle_filter unavailable")
        return pf

    def _drive_command(self, method: str, *args: float) -> None:
        actuators = getattr(self._robot, "actuators", None)
        drive = None
        if isinstance(actuators, dict):
            drive = actuators.get("drive")
        elif actuators is not None and hasattr(actuators, "get"):
            try:
                drive = actuators.get("drive")
            except Exception:  # pragma: no cover
                drive = None
        if drive is None:
            drive = getattr(self._robot, "drive", None)
        fn = getattr(drive, method, None) if drive is not None else None
        if not callable(fn):
            print(f"[ParticleViewer] drive.{method} unavailable")
            return
        try:
            fn(None, *args)
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] drive.{method} failed: {exc}")

    def _drop_command(self) -> None:
        actuators = getattr(self._robot, "actuators", None)
        kick = None
        if isinstance(actuators, dict):
            kick = actuators.get("kick")
        elif actuators is not None and hasattr(actuators, "get"):
            try:
                kick = actuators.get("kick")
            except Exception:  # pragma: no cover
                kick = None
        if kick is None:
            kick = getattr(self._robot, "kick", None)
        fn = getattr(kick, 'place', None) if kick is not None else None
        if not callable(fn):
            print(f"[ParticleViewer] kick.place unavailable")
            return
        try:
            fn(None)
        except Exception as exc:  # pragma: no cover
            print(f"[ParticleViewer] kick.place failed: {exc}")

    def _print_pose(self) -> None:
        pf = self._particle_filter()
        if pf is None:
            return
        pose = getattr(pf, "pose", None)
        if pose is None:
            update = getattr(pf, "update_pose_estimate", None)
            if callable(update):
                try:
                    pose = update()
                except Exception:  # pragma: no cover
                    pose = None
        if pose is None:
            return
        heading = math.degrees(getattr(pose, "theta", 0.0))
        print("Pose = ({0:5.1f}, {1:5.1f}) @ {2:5.1f} deg.".format(pose.x, pose.y, heading))


__all__ = ["ParticleViewer", "ParticleViewState"]
