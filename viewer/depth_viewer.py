"""QtQuick-based depth viewer showing depth and gradient maps."""

from __future__ import annotations

import concurrent.futures
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtProperty, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QGuiApplication, QImage
from PyQt6.QtQuick import QQuickImageProvider, QQuickView

try:  # Optional acceleration for colormap rendering.
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore

from .help_texts import DEPTH_HELP_TEXT


class DepthImageProvider(QQuickImageProvider):
    """Thread-safe image provider for depth/gradient textures."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._lock = threading.Lock()
        self._depth: Optional[QImage] = None
        self._gradient: Optional[QImage] = None

    def set_images(
        self,
        depth_map: np.ndarray,
        depth_valid: np.ndarray,
        gradient_map: np.ndarray,
        gradient_valid: np.ndarray,
    ) -> None:
        depth = self._map_to_qimage(depth_map, depth_valid, map_type="depth")
        gradient = self._map_to_qimage(gradient_map, gradient_valid, map_type="gradient")
        with self._lock:
            self._depth = depth
            self._gradient = gradient

    def requestImage(self, image_id: str, *args):  # type: ignore[override]
        route = image_id.split("?", 1)[0] if image_id else ""
        with self._lock:
            source = self._gradient if route == "gradient" else self._depth
            image = source.copy() if source is not None and not source.isNull() else self._placeholder()

        if len(args) == 1:
            size_out = args[0]
            _update_size(size_out, image)
            return image, image.size()
        if len(args) == 2:
            size_out, _requested = args
            _update_size(size_out, image)
            return image
        return image

    @staticmethod
    def _placeholder() -> QImage:
        image = QImage(1, 1, QImage.Format.Format_RGB888)
        image.fill(0)
        return image

    @staticmethod
    def _normalise_to_u8(data: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
        out = np.zeros(data.shape, dtype=np.uint8)
        if data.size == 0 or not np.any(valid_mask):
            return out

        values = data[valid_mask]
        values = values[np.isfinite(values)]
        if values.size == 0:
            return out

        lo = float(np.percentile(values, 1.0))
        hi = float(np.percentile(values, 99.0))
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            lo = float(values.min())
            hi = float(values.max())
        if hi <= lo:
            out[valid_mask] = 128
            return out

        scaled = (np.clip((data - lo) / (hi - lo), 0.0, 1.0) * 255.0).astype(np.uint8)
        out[valid_mask] = scaled[valid_mask]
        return out

    def _map_to_qimage(self, data: np.ndarray, valid_mask: np.ndarray, *, map_type: str) -> QImage:
        data = np.asarray(data, dtype=np.float32)
        valid = np.asarray(valid_mask, dtype=bool)
        gray = self._normalise_to_u8(data, valid)

        if cv2 is not None:
            cmap = cv2.COLORMAP_TURBO if map_type == "depth" else cv2.COLORMAP_INFERNO
            color_bgr = cv2.applyColorMap(gray, cmap)
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        else:  # pragma: no cover - fallback path
            color_rgb = np.repeat(gray[:, :, None], 3, axis=2)

        color_rgb[~valid] = 0
        return _qimage_from_rgb(color_rgb)


class DepthViewer(QObject):
    """Standalone depth viewer rendering live depth + gradient maps."""

    statusChanged = pyqtSignal()
    liveChanged = pyqtSignal()
    gradientSourceChanged = pyqtSignal()

    def __init__(
        self,
        robot: Any,
        width: int = 960,
        height: int = 540,
        windowName: str = "Depth Viewer",
        update_interval_ms: int = 333,
        poll_interval_ms: int = 50,
    ) -> None:
        super().__init__(parent=None)
        if robot is None:
            raise ValueError("robot instance is required")

        self._robot = robot
        self._width = int(width)
        self._height = int(height)
        self._window_name = windowName
        self._update_interval_ms = max(1, int(update_interval_ms))
        self._poll_interval_ms = max(10, int(poll_interval_ms))

        self._app = QGuiApplication.instance() or QGuiApplication([])
        self._image_provider = DepthImageProvider()

        self._detector = None
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._future: Optional[concurrent.futures.Future] = None
        self._manual_refresh_pending = False
        self._live_enabled = True
        self._status_text = "Waiting for camera frames..."
        self._gradient_source = "camera"
        self._frame_counter = 0
        self._last_processed_frame_id: Optional[int] = None

        self._update_timer = QTimer(self)
        self._update_timer.setInterval(self._update_interval_ms)
        self._update_timer.timeout.connect(self._on_update_tick)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(self._poll_interval_ms)
        self._poll_timer.timeout.connect(self._poll_worker)

        self._view = QQuickView()
        self._view.setTitle(self._window_name)
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)

        self._context = self._initialise_qml_context()

    @pyqtProperty(str, notify=statusChanged)
    def statusText(self) -> str:
        return self._status_text

    @pyqtProperty(bool, notify=liveChanged)
    def liveEnabled(self) -> bool:
        return self._live_enabled

    @pyqtProperty(str, notify=gradientSourceChanged)
    def gradientSource(self) -> str:
        return self._gradient_source

    def start(self) -> None:
        if self._live_enabled and not self._update_timer.isActive():
            self._update_timer.start()
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        self._view.show()
        self._focus_root()
        print(DEPTH_HELP_TEXT, end="")
        self.requestSingleRefresh()

    def stop(self) -> None:
        self._update_timer.stop()
        self._poll_timer.stop()
        self._manual_refresh_pending = False

        future = self._future
        self._future = None
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass

        executor = self._executor
        self._executor = None
        if executor is not None:
            try:
                executor.shutdown(wait=False)
            except Exception:
                pass

        self._detector = None
        self._view.close()

    @pyqtSlot()
    def requestQuit(self) -> None:
        self.stop()

    @pyqtSlot()
    def printHelp(self) -> None:
        print(DEPTH_HELP_TEXT, end="")

    @pyqtSlot()
    def toggleLiveRefresh(self) -> None:
        self._live_enabled = not self._live_enabled
        self.liveChanged.emit()
        if self._live_enabled:
            if not self._update_timer.isActive():
                self._update_timer.start()
            self._set_status("Live refresh enabled.")
            self.requestSingleRefresh()
        else:
            self._update_timer.stop()
            self._set_status("Live refresh paused. Press 'r' for one-frame update.")

    @pyqtSlot()
    def toggleGradientSource(self) -> None:
        self._gradient_source = "depth" if self._gradient_source == "camera" else "camera"
        self.gradientSourceChanged.emit()
        self._set_status(f"Gradient source switched: grad_source={self._gradient_source}")
        self.requestSingleRefresh()

    @pyqtSlot()
    def requestSingleRefresh(self) -> None:
        self._manual_refresh_pending = True
        self._try_schedule_update(force=True)

    @pyqtSlot()
    def _on_update_tick(self) -> None:
        if self._live_enabled:
            self._try_schedule_update(force=False)

    @pyqtSlot()
    def _poll_worker(self) -> None:
        future = self._future
        if future is None:
            if self._manual_refresh_pending:
                self._try_schedule_update(force=True)
            else:
                self._poll_timer.stop()
            return

        if not future.done():
            return

        try:
            result = future.result(timeout=0.1)
        except Exception as exc:  # pragma: no cover - defensive logging
            result = {
                "ok": False,
                "error": f"worker_error: {exc}",
                "elapsed_ms": 0.0,
                "runtime_ms": 0.0,
                "frame_id": None,
            }
        finally:
            self._future = None

        if result.get("ok"):
            depth = result["depth"]
            grad = result["gradient"]
            depth_valid = result["depth_valid"]
            grad_valid = result["gradient_valid"]
            self._image_provider.set_images(depth, depth_valid, grad, grad_valid)
            self._frame_counter += 1
            self._context.setContextProperty("depthFrameId", self._frame_counter)
            frame_id = result.get("frame_id")
            if frame_id is not None and frame_id >= 0:
                self._last_processed_frame_id = int(frame_id)

            status = (
                "grad_source={grad_source} model={model} device={device} infer={runtime_ms:.1f}ms total={elapsed_ms:.1f}ms "
                "frame={frame}"
            ).format(
                grad_source=result.get("gradient_source", self._gradient_source),
                model=result.get("model_name", "?"),
                device=result.get("device", "?"),
                runtime_ms=float(result.get("runtime_ms", 0.0)),
                elapsed_ms=float(result.get("elapsed_ms", 0.0)),
                frame=frame_id if frame_id is not None and frame_id >= 0 else "?",
            )
            self._set_status(status)
        else:
            self._set_status(f"Depth update failed: {result.get('error', 'unknown error')}")

        if self._manual_refresh_pending:
            self._try_schedule_update(force=True)
        elif self._future is None:
            self._poll_timer.stop()

    def _try_schedule_update(self, *, force: bool) -> None:
        if self._future is not None:
            return

        image = getattr(self._robot, "camera_image", None)
        frame_id = getattr(self._robot, "frame_count", None)
        if image is None:
            self._set_status("Waiting for camera frames...")
            return

        if (
            not force
            and frame_id is not None
            and self._last_processed_frame_id is not None
            and frame_id == self._last_processed_frame_id
        ):
            return

        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        frame = np.array(image, copy=True)
        frame_id_snapshot = int(frame_id) if frame_id is not None else -1
        gradient_source_snapshot = str(self._gradient_source)
        self._manual_refresh_pending = False
        self._future = self._executor.submit(
            self._process_frame_worker, frame, frame_id_snapshot, gradient_source_snapshot
        )
        if not self._poll_timer.isActive():
            self._poll_timer.start()
        self._set_status(
            "Processing frame {frame} (grad_source={source})...".format(
                frame=frame_id_snapshot if frame_id_snapshot >= 0 else "?",
                source=gradient_source_snapshot,
            )
        )

    def _process_frame_worker(
        self, frame: np.ndarray, frame_id: int, gradient_source: str
    ) -> dict[str, Any]:
        start = time.perf_counter()
        try:
            detector = self._detector
            if detector is None:
                detector = self._build_detector()
                self._detector = detector

            result = detector.process(frame)
            depth = result.depth.depth.astype(np.float32, copy=False)
            scale_hint = getattr(result.depth, "scale_hint", None)
            if scale_hint is not None and np.isfinite(scale_hint) and float(scale_hint) > 0.0:
                depth = depth * float(scale_hint)

            depth_valid = np.isfinite(depth) & (depth > 0.0)
            depth_safe = np.where(depth_valid, depth, 0.0).astype(np.float32, copy=False)

            if gradient_source == "depth":
                gradient, gradient_valid = self._compute_depth_gradient(depth_safe, depth_valid)
            else:
                gradient, gradient_valid = self._compute_camera_gradient(frame)
                gradient_source = "camera"

            elapsed_ms = (time.perf_counter() - start) * 1000.0
            return {
                "ok": True,
                "depth": depth_safe,
                "depth_valid": depth_valid,
                "gradient": gradient,
                "gradient_valid": gradient_valid,
                "gradient_source": gradient_source,
                "runtime_ms": float(getattr(result.depth, "runtime_ms", 0.0)),
                "elapsed_ms": elapsed_ms,
                "model_name": str(getattr(result.depth, "model_name", "?")),
                "device": str(getattr(result.depth, "device", "?")),
                "frame_id": frame_id,
            }
        except Exception as exc:
            return {
                "ok": False,
                "error": str(exc),
                "elapsed_ms": (time.perf_counter() - start) * 1000.0,
                "runtime_ms": 0.0,
                "frame_id": frame_id,
                "gradient_source": gradient_source,
            }

    @staticmethod
    def _compute_depth_gradient(
        depth_map: np.ndarray, depth_valid: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        depth_r = np.zeros_like(depth_map, dtype=np.float32)
        depth_r[:, 1:] = depth_map[:, :-1]
        depth_d = np.zeros_like(depth_map, dtype=np.float32)
        depth_d[1:, :] = depth_map[:-1, :]

        valid_r = np.zeros_like(depth_valid, dtype=bool)
        valid_r[:, 1:] = depth_valid[:, :-1]
        valid_d = np.zeros_like(depth_valid, dtype=bool)
        valid_d[1:, :] = depth_valid[:-1, :]

        dx = depth_map - depth_r
        dy = depth_map - depth_d
        gradient = np.sqrt(dx * dx + dy * dy).astype(np.float32, copy=False)
        gradient_valid = depth_valid & valid_r & valid_d
        gradient = np.where(gradient_valid, gradient, 0.0).astype(np.float32, copy=False)
        return gradient, gradient_valid

    @staticmethod
    def _compute_camera_gradient(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        camera = np.asarray(frame, dtype=np.float32)
        if camera.ndim < 2:
            raise RuntimeError("Camera frame has unsupported shape")

        camera_r = np.zeros_like(camera, dtype=np.float32)
        camera_d = np.zeros_like(camera, dtype=np.float32)
        camera_r[:, 1:, ...] = camera[:, :-1, ...]
        camera_d[1:, :, ...] = camera[:-1, :, ...]

        dx = camera - camera_r
        dy = camera - camera_d

        if camera.ndim == 2:
            gradient = np.sqrt(dx * dx + dy * dy).astype(np.float32, copy=False)
            finite = np.isfinite(camera)
        else:
            delta_sq = dx * dx + dy * dy
            gradient = np.sqrt(np.sum(delta_sq, axis=2, dtype=np.float32)).astype(
                np.float32, copy=False
            )
            finite = np.all(np.isfinite(camera), axis=2)

        valid_r = np.zeros_like(finite, dtype=bool)
        valid_r[:, 1:] = finite[:, :-1]
        valid_d = np.zeros_like(finite, dtype=bool)
        valid_d[1:, :] = finite[:-1, :]
        gradient_valid = finite & valid_r & valid_d
        gradient = np.where(gradient_valid, gradient, 0.0).astype(np.float32, copy=False)
        return gradient, gradient_valid

    def _build_detector(self):
        try:
            from perception import CliffDetector, DepthAnythingProvider, load_cliff_calibration
        except Exception as exc:  # pragma: no cover
            raise RuntimeError("Perception modules unavailable") from exc

        provider_mode = os.environ.get("DEPTHANYTHING_PROVIDER", "dummy").lower()
        if provider_mode == "dummy":
            print("[DepthViewer] using dummy DepthAnything provider")
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

    def _set_status(self, text: str) -> None:
        text = str(text)
        if text != self._status_text:
            self._status_text = text
            self.statusChanged.emit()

    def _initialise_qml_context(self):
        repo_root = Path(__file__).resolve().parents[1]
        qml_dir = repo_root / "qml"
        qml_path = (qml_dir / "DepthView.qml").resolve()

        engine = self._view.engine()
        engine.addImportPath(str(qml_dir))
        engine.addImageProvider("depthviz", self._image_provider)

        context = self._view.rootContext()
        context.setContextProperty("viewerApp", self)
        context.setContextProperty("depthFrameId", self._frame_counter)
        context.setContextProperty("depthHelpText", DEPTH_HELP_TEXT)

        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))
        if self._view.status() == QQuickView.Status.Error:
            errors = "\n".join(error.toString() for error in self._view.errors())
            raise RuntimeError(f"Failed to load DepthView.qml:\n{errors}")
        return context

    def _focus_root(self) -> None:
        try:
            root = self._view.rootObject()
        except Exception:
            return
        if root is not None and hasattr(root, "forceActiveFocus"):
            root.forceActiveFocus()


def _qimage_from_rgb(rgb: np.ndarray) -> QImage:
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("Expected RGB array with shape (H, W, 3)")
    if not rgb.flags.c_contiguous:
        rgb = np.ascontiguousarray(rgb)
    height, width = rgb.shape[:2]
    image = QImage(rgb.data, width, height, width * 3, QImage.Format.Format_RGB888)
    return image.copy()


def _update_size(target, image: QImage) -> None:
    if target is None:
        return
    try:
        target.setWidth(image.width())
        target.setHeight(image.height())
    except Exception:  # pragma: no cover
        pass


__all__ = ["DepthViewer"]
