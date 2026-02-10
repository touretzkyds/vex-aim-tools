"""QtQuick-based particle viewer replacing the legacy OpenGL implementation."""

from __future__ import annotations

import concurrent.futures
import math
import os
import time
from collections import deque
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
from .particle_model import LandmarkModel, ParticleLayerModel, ParticleSummary


class ParticleViewState(QObject):
    """Shared view bounds/zoom state exposed to QML."""

    changed = pyqtSignal()

    def __init__(
        self,
        center_x: float = 0.0,
        center_y: float = 0.0,
        zoom: float = 0.4,
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
        scale: float = 0.64,  # legacy compatibility (unused)
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
        self._view_state = ParticleViewState()
        self._auto_center = True
        self._verbose = False
        self._update_interval_ms = max(0, int(update_interval_ms))
        self._redisplay_enabled = self._update_interval_ms > 0

        self._grid_detector = None
        self._grid_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._grid_future: Optional[concurrent.futures.Future] = None
        self._grid_requests: deque[tuple[int, np.ndarray, tuple[float, float, float]]] = deque()
        self._grid_request_counter = 0
        self._active_grid_request_id: Optional[int] = None

        self._timer = QTimer(self)
        self._timer.setInterval(self._update_interval_ms)
        self._timer.timeout.connect(self.refresh)

        self._grid_poll_timer = QTimer(self)
        self._grid_poll_timer.setInterval(50)
        self._grid_poll_timer.timeout.connect(self._poll_grid_update_worker)

        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)

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
        print(PARTICLE_HELP_TEXT, end="")

    def stop(self) -> None:
        self._timer.stop()
        self._shutdown_grid_update_worker()
        self._view.close()

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

    @pyqtSlot()
    def updateOccupancyGridFromCurrentFrame(self) -> None:
        world_map = getattr(self._robot, "world_map", None)
        if world_map is None:
            print("[ParticleViewer] world_map unavailable; cannot update occupancy grid")
            return
        if getattr(world_map, "occupancy_grid", None) is None:
            print("[ParticleViewer] occupancy_grid unavailable; cannot update occupancy grid")
            return

        image = getattr(self._robot, "camera_image", None)
        if image is None:
            print("[ParticleViewer] camera_image unavailable; cannot update occupancy grid")
            return

        pose = getattr(self._robot, "pose", None)
        if pose is None:
            print("[ParticleViewer] robot pose unavailable; cannot update occupancy grid")
            return

        try:
            pose_snapshot = (float(pose.x), float(pose.y), float(pose.theta))
        except Exception:
            print("[ParticleViewer] robot pose invalid; cannot update occupancy grid")
            return

        frame = np.array(image, copy=True)
        self._grid_request_counter += 1
        request_id = self._grid_request_counter
        self._grid_requests.append((request_id, frame, pose_snapshot))

        pending = len(self._grid_requests) + (1 if self._grid_future is not None else 0)
        print(f"[ParticleViewer] queued occupancy grid update #{request_id} (pending={pending})")
        self._start_next_grid_update()

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

    def _start_next_grid_update(self) -> None:
        if self._grid_future is not None or not self._grid_requests:
            return

        if self._grid_executor is None:
            self._grid_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        request_id, frame, pose = self._grid_requests.popleft()
        self._active_grid_request_id = request_id
        self._grid_future = self._grid_executor.submit(
            self._process_grid_update_worker,
            frame,
            pose,
        )
        if not self._grid_poll_timer.isActive():
            self._grid_poll_timer.start()
        print(f"[ParticleViewer] processing occupancy grid update #{request_id}")

    @pyqtSlot()
    def _poll_grid_update_worker(self) -> None:
        future = self._grid_future
        if future is None:
            if self._grid_requests:
                self._start_next_grid_update()
            else:
                self._grid_poll_timer.stop()
            return

        if not future.done():
            return

        request_id = self._active_grid_request_id
        try:
            result = future.result(timeout=0.1)
        except Exception as exc:  # pragma: no cover - defensive
            result = {
                "success": False,
                "status": "error",
                "error": f"worker_error: {exc}",
                "ground": 0,
                "obstacle": 0,
                "elapsed_s": 0.0,
            }
        finally:
            self._grid_future = None
            self._active_grid_request_id = None

        status = str(result.get("status", "error"))
        error_msg = result.get("error")
        elapsed = float(result.get("elapsed_s", 0.0))
        ground = int(result.get("ground", 0))
        obstacle = int(result.get("obstacle", 0))

        if error_msg:
            print(f"[ParticleViewer] occupancy grid update #{request_id} failed: {error_msg}")
        elif status == "updated":
            print(
                "[ParticleViewer] occupancy grid update #{0}: ground={1} obstacle={2} ({3:.2f}s)".format(
                    request_id,
                    ground,
                    obstacle,
                    elapsed,
                )
            )
        else:
            print(f"[ParticleViewer] occupancy grid update #{request_id} skipped ({elapsed:.2f}s)")

        self._start_next_grid_update()
        if self._grid_future is None and not self._grid_requests:
            self._grid_poll_timer.stop()

    def _process_grid_update_worker(
        self,
        frame: np.ndarray,
        pose: tuple[float, float, float],
    ) -> dict[str, Any]:
        start = time.perf_counter()
        try:
            detector = self._grid_detector
            if detector is None:
                detector = self._build_grid_detector()
                self._grid_detector = detector

            result = detector.process(frame)
            depth_map = result.depth.depth
            scale_hint = getattr(result.depth, "scale_hint", None)
            if scale_hint is not None and np.isfinite(scale_hint) and float(scale_hint) > 0.0:
                depth_map = depth_map * float(scale_hint)

            world_map = getattr(self._robot, "world_map", None)
            if world_map is None:
                return {
                    "success": False,
                    "status": "error",
                    "error": "world_map unavailable",
                    "ground": 0,
                    "obstacle": 0,
                    "elapsed_s": time.perf_counter() - start,
                }

            success = bool(
                world_map.update_grid_from_depth(
                    depth_map,
                    detector.intrinsics,
                    detector.extrinsics,
                    robot_pose=pose,
                )
            )

            grid = getattr(world_map, "occupancy_grid", None)
            stats = getattr(grid, "last_update_counts", {}) if grid is not None else {}
            return {
                "success": success,
                "status": "updated" if success else "skipped",
                "error": None,
                "ground": int(stats.get("ground", 0)),
                "obstacle": int(stats.get("obstacle", 0)),
                "elapsed_s": time.perf_counter() - start,
            }
        except Exception as exc:
            return {
                "success": False,
                "status": "error",
                "error": str(exc),
                "ground": 0,
                "obstacle": 0,
                "elapsed_s": time.perf_counter() - start,
            }

    def _build_grid_detector(self):
        try:
            from perception import CliffDetector, DepthAnythingProvider, load_cliff_calibration
        except Exception as exc:  # pragma: no cover - optional dependency path
            raise RuntimeError("Perception modules unavailable") from exc

        provider_mode = os.environ.get("DEPTHANYTHING_PROVIDER", "dummy").lower()
        if provider_mode == "dummy":
            print("[ParticleViewer] using dummy DepthAnything provider")
            provider = DepthAnythingProvider.build_dummy()
        elif provider_mode == "torch":
            weights_env = os.environ.get("DEPTHANYTHING_WEIGHTS")
            if not weights_env:
                raise RuntimeError("DEPTHANYTHING_WEIGHTS must be set for torch provider")

            provider = DepthAnythingProvider.from_torch(
                weights_path=Path(weights_env),
                model_type=os.environ.get("DEPTHANYTHING_MODEL", "depthanything-v2-small"),
                device=os.environ.get("DEPTHANYTHING_DEVICE", "cpu"),
            )
        else:
            raise RuntimeError(f"Unsupported DEPTHANYTHING_PROVIDER={provider_mode}")

        camera = getattr(self._robot, "camera", None)
        calibration = load_cliff_calibration(camera)
        return CliffDetector(
            depth_provider=provider,
            intrinsics=calibration.intrinsics,
            gravity_camera=calibration.gravity_camera,
            extrinsics=calibration.camera_to_base,
        )

    def _shutdown_grid_update_worker(self) -> None:
        self._grid_requests.clear()
        self._active_grid_request_id = None
        self._grid_poll_timer.stop()

        future = self._grid_future
        self._grid_future = None
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass

        executor = self._grid_executor
        self._grid_executor = None
        if executor is not None:
            try:
                executor.shutdown(wait=False)
            except Exception:
                pass

        self._grid_detector = None

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
