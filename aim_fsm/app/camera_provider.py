"""Camera image provider bridging worker updates into QML via Qt Quick."""

from __future__ import annotations

import threading
from typing import Any, Callable, Optional, Tuple

try:  # Optional dependency used for ndarray conversion during tests.
    import numpy as _np  # type: ignore
except Exception:  # pragma: no cover - numpy not available
    _np = None  # type: ignore

from PyQt6.QtGui import QGuiApplication
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QImage
from PyQt6.QtQuick import QQuickImageProvider

from aim_fsm.app import constants
from aim_fsm.app.camera_overlay import apply_overlays


RGB888 = QImage.Format.Format_RGB888


class CameraImageProvider(QQuickImageProvider):
    """Thread-safe image provider supporting live and annotated camera feeds."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._lock = threading.Lock()
        self._live: Optional[QImage] = None
        self._annotated: Optional[QImage] = None
        self._live_buffer: Optional[Any] = None
        self._annotated_buffer: Optional[Any] = None
        self._frame_callback: Optional[Callable[[], None]] = None

        self._status: Optional[dict] = None
        self._aruco_detector: Optional[Any] = None
        self._user_hook: Optional[Callable[[Any], Any]] = None
        self._robot_ref: Optional[Any] = None

    # ------------------------------------------------------------------
    # Public API consumed by ingestion threads

    def register_frame_notifier(self, callback: Callable[[], None]) -> None:
        """Register a callback invoked whenever frame data changes."""

        self._frame_callback = callback

    def set_status(self, status: Optional[dict]) -> None:
        self._status = status

    def set_aruco_detector(self, detector: Optional[Any]) -> None:
        self._aruco_detector = detector

    def set_user_annotate(self, hook: Optional[Callable[[Any], Any]]) -> None:
        self._user_hook = hook

    def set_robot_ref(self, robot: Optional[Any]) -> None:
        self._robot_ref = robot

    def update_live_frame(self, rgb: Any) -> None:
        """Ingest the latest RGB frame and synthesize overlays if configured."""

        if _np is None:
            raise RuntimeError("NumPy is required for update_live_frame")

        rgb_array = self._ensure_rgb_array(rgb)
        live_qimage, live_buffer = self._qimage_from_rgb(rgb_array)

        annotated_qimage = self._empty_qimage_1x1_rgb()
        annotated_buffer: Optional[Any] = None
        annotated_np: Optional[Any] = None

        if self._status or self._aruco_detector or self._user_hook:
            try:
                annotated_np = apply_overlays(
                    image_rgb=rgb_array,
                    status=self._status,
                    scale=constants.AIVISION_RESOLUTION_SCALE,
                    aruco_detector=self._aruco_detector,
                    user_hook=self._user_hook,
                )
                if isinstance(annotated_np, _np.ndarray):
                    annotated_np = self._ensure_rgb_array(annotated_np)
                else:
                    annotated_np = None
            except Exception:
                annotated_np = None

            if annotated_np is not None:
                annotated_qimage, annotated_buffer = self._qimage_from_rgb(annotated_np)

        with self._lock:
            self._live = live_qimage
            self._live_buffer = live_buffer
            self._annotated = annotated_qimage
            self._annotated_buffer = annotated_buffer

        self._backfill_robot(annotated_np)
        self._queue_frame_bump()

    def set_image(self, image: Any) -> None:
        qimage, buffer = self._to_qimage(image)
        with self._lock:
            self._live = qimage
            self._live_buffer = buffer
        self._queue_frame_bump()

    def set_annotated_image(self, image: Any) -> None:
        qimage, buffer = self._to_qimage(image)
        with self._lock:
            self._annotated = qimage
            self._annotated_buffer = buffer
        self._backfill_robot(None)
        self._queue_frame_bump()

    def get_image(self) -> Optional[QImage]:
        with self._lock:
            return self._clone_or_none(self._live)

    def get_annotated_image(self) -> Optional[QImage]:
        with self._lock:
            return self._clone_or_none(self._annotated)

    # ------------------------------------------------------------------
    # QQuickImageProvider implementation

    def requestImage(  # type: ignore[override]
        self,
        image_id: str,
        size: Any,
        requested_size: Any,
    ) -> QImage:
        with self._lock:
            if image_id == "annotated":
                image = self._annotated
            else:
                image = self._live
            image = self._clone_or_none(image)

        if image is None or getattr(image, "isNull", lambda: False)():
            image = self._empty_qimage_1x1_rgb()
        if size is not None:
            try:
                size.setWidth(image.width())  # type: ignore[attr-defined]
                size.setHeight(image.height())  # type: ignore[attr-defined]
            except AttributeError:
                pass
        return image

    # ------------------------------------------------------------------
    # Internal helpers

    def _queue_frame_bump(self) -> None:
        if self._frame_callback is None:
            return
        if QGuiApplication.instance() is None:
            self._frame_callback()
        else:
            QTimer.singleShot(0, self._frame_callback)

    @staticmethod
    def _clone_or_none(image: Optional[QImage]) -> Optional[QImage]:
        if image is None:
            return None
        copy_method = getattr(image, "copy", None)
        if callable(copy_method):
            return copy_method()
        return image

    def _ensure_rgb_array(self, image: Any) -> _np.ndarray:
        if _np is None:
            raise RuntimeError("NumPy is required for update_live_frame")
        array = _np.asarray(image)
        if array.ndim != 3 or array.shape[2] < 3:
            raise ValueError("Expected HxWx3 array for image data")
        return _np.array(array[..., :3], dtype=_np.uint8, copy=True, order="C")

    def _qimage_from_rgb(self, rgb: _np.ndarray) -> Tuple[QImage, _np.ndarray]:
        bgr = rgb[:, :, ::-1].copy(order="C")
        height, width, _ = bgr.shape
        bytes_per_line = bgr.strides[0]
        qimage = QImage(bgr.data, width, height, bytes_per_line, RGB888)  # type: ignore[arg-type]
        return qimage, bgr

    def _to_qimage(self, image: Any) -> tuple[QImage, Optional[Any]]:
        if isinstance(image, QImage):
            converted = image.convertToFormat(RGB888)
            return converted.copy(), None

        if _np is not None and isinstance(image, _np.ndarray):
            arr = image
            if arr.ndim != 3 or arr.shape[2] < 3:
                raise ValueError("Expected HxWx3 array for image data")
            if arr.dtype != _np.uint8:
                arr = arr.astype(_np.uint8)
            bgr = arr[..., :3][:, :, ::-1].copy(order="C")
            height, width, _ = bgr.shape
            bytes_per_line = bgr.strides[0]
            qimage = QImage(bgr.data, width, height, bytes_per_line, RGB888)  # type: ignore[arg-type]
            return qimage, bgr

        raise TypeError("Unsupported image type for CameraImageProvider")

    def _empty_qimage_1x1_rgb(self) -> QImage:
        image = QImage(1, 1, RGB888)
        image.fill(0)
        return image

    def _backfill_robot(self, annotated_np: Optional[Any]) -> None:
        if self._robot_ref is None:
            return
        try:
            self._robot_ref.annotated_image = annotated_np if annotated_np is not None else None
        except Exception:
            pass


__all__ = ["CameraImageProvider"]
