"""Qt Quick image provider that decorates camera frames with overlays."""

from __future__ import annotations

import threading
from typing import Any, Callable, Optional

import numpy as _np
from PyQt6.QtGui import QImage
from PyQt6.QtQuick import QQuickImageProvider

from aim_fsm.camera import AIVISION_RESOLUTION_SCALE

from .camera_overlay import apply_overlays


class CameraImageProvider(QQuickImageProvider):
    """Thread-safe provider serving live and annotated frames to Qt Quick."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._lock = threading.Lock()
        self._live: Optional[QImage] = None
        self._annotated: Optional[QImage] = None
        self._notifier: Optional[Callable[[], None]] = None

        self._live_rgb: Optional[_np.ndarray] = None
        self._annotated_rgb: Optional[_np.ndarray] = None

        self._status: Optional[dict] = None
        self._aruco_detector: Optional[Any] = None
        self._user_hook: Optional[Callable[[Any], Any]] = None
        self._robot_ref: Optional[Any] = None
        self._crosshair_enabled = False

    # Configuration ------------------------------------------------

    def register_notifier(self, callback: Optional[Callable[[], None]]) -> None:
        self._notifier = callback

    def set_status(self, status: Optional[dict]) -> None:
        self._status = status
        self._refresh_annotation()

    def set_aruco_detector(self, detector: Optional[Any]) -> None:
        self._aruco_detector = detector
        self._refresh_annotation()

    def set_user_hook(self, hook: Optional[Callable[[Any], Any]]) -> None:
        self._user_hook = hook
        self._refresh_annotation()

    def set_robot_ref(self, robot: Optional[Any]) -> None:
        self._robot_ref = robot

    def set_crosshair_enabled(self, enabled: bool) -> None:
        if self._crosshair_enabled == bool(enabled):
            return
        self._crosshair_enabled = bool(enabled)
        self._refresh_annotation()

    # Frame ingestion ----------------------------------------------

    def update_live_frame(self, image: Any) -> None:
        rgb = self._ensure_rgb_array(image)
        live_qimage = self._qimage_from_rgb(rgb)
        annotated_rgb = self._build_annotated(rgb)
        annotated_qimage = self._qimage_from_rgb(annotated_rgb) if annotated_rgb is not None else None

        with self._lock:
            self._live = live_qimage
            self._annotated = annotated_qimage
            self._live_rgb = rgb
            self._annotated_rgb = annotated_rgb

        self._backfill_robot(annotated_rgb)
        self._notify()

    def update_annotated_frame(self, image: Any) -> None:
        rgb = self._ensure_rgb_array(image)
        qimage = self._qimage_from_rgb(rgb)
        with self._lock:
            self._annotated = qimage
            self._annotated_rgb = rgb
        self._backfill_robot(rgb)
        self._notify()

    def clear(self) -> None:
        with self._lock:
            self._live = None
            self._annotated = None
            self._live_rgb = None
            self._annotated_rgb = None
        self._notify()

    # Accessors ----------------------------------------------------

    def latest_live_image(self) -> Optional[QImage]:
        with self._lock:
            return self._clone_or_none(self._live)

    def latest_annotated_image(self) -> Optional[QImage]:
        with self._lock:
            return self._clone_or_none(self._annotated)

    def current_image(self, route: str) -> Optional[QImage]:
        selected = "annotated" if route == "annotated" else "live"
        with self._lock:
            source = self._annotated if selected == "annotated" else self._live
            return self._clone_or_none(source)

    # QQuickImageProvider -----------------------------------------

    def requestImage(self, image_id: str, *args):  # type: ignore[override]
        base = image_id.split("?", 1)[0] if image_id else ""
        with self._lock:
            source = self._annotated if base == "annotated" else self._live
            image = self._clone_or_none(source)
        if image is None or image.isNull():
            image = self._placeholder()

        if len(args) == 1:
            size_out = args[0]
            _update_size(size_out, image)
            return image, image.size()
        if len(args) == 2:
            size_out, _requested = args
            _update_size(size_out, image)
            return image
        return image

    # Internal helpers --------------------------------------------

    def _refresh_annotation(self) -> None:
        with self._lock:
            if self._live_rgb is None:
                return
            rgb = self._live_rgb.copy()

        annotated_rgb = self._build_annotated(rgb)
        annotated_qimage = self._qimage_from_rgb(annotated_rgb) if annotated_rgb is not None else None

        with self._lock:
            self._annotated = annotated_qimage
            self._annotated_rgb = annotated_rgb

        self._backfill_robot(annotated_rgb)
        self._notify()

    def _build_annotated(self, rgb: _np.ndarray) -> Optional[_np.ndarray]:
        annotated = rgb.copy()

        status = self._resolve_status()

        overlays_requested = bool((status or {}).get("aivision") or self._aruco_detector)
        if overlays_requested:
            maybe = apply_overlays(
                annotated,
                status,
                int(AIVISION_RESOLUTION_SCALE) or 1,
                self._aruco_detector,
            )
            if isinstance(maybe, _np.ndarray) and maybe.ndim == 3:
                annotated = maybe.copy()

        if self._crosshair_enabled:
            self._apply_crosshair(annotated)

        if self._user_hook is not None:
            try:
                maybe = self._user_hook(annotated)
                if isinstance(maybe, _np.ndarray) and maybe.ndim == 3:
                    annotated = maybe.copy()
            except Exception:
                pass

        return annotated

    def _apply_crosshair(self, arr: _np.ndarray) -> None:
        if arr is None or arr.ndim != 3:
            return
        h, w = arr.shape[:2]
        cx = w // 2
        cy = h // 2
        color = _np.array([255, 255, 0], dtype=_np.uint8)
        arr[cy, :] = color
        arr[:, cx] = color

    def _resolve_status(self) -> Optional[Any]:
        status = self._status
        if self._status_has_aivision(status):
            return status

        robot = self._robot_ref
        if robot is not None:
            candidate = self._safe_status_lookup(robot)
            if self._status_has_aivision(candidate):
                return candidate

            robot0 = getattr(robot, "robot0", None)
            if robot0 is not None:
                candidate = self._safe_status_lookup(robot0)
                if self._status_has_aivision(candidate):
                    return candidate

        return status

    @staticmethod
    def _status_has_aivision(status: Any) -> bool:
        try:
            return status is not None and "aivision" in status
        except TypeError:
            return False

    @staticmethod
    def _safe_status_lookup(obj: Any) -> Optional[Any]:
        try:
            return getattr(obj, "status", None)
        except Exception:
            return None

    def _ensure_rgb_array(self, image: Any) -> _np.ndarray:
        if isinstance(image, QImage):
            return self._numpy_from_qimage(image)
        array = _np.asarray(image, dtype=_np.uint8)
        if array.ndim != 3 or array.shape[2] < 3:
            raise ValueError("Expected HxWx3 array for image data")
        rgb = array[..., :3]
        if not rgb.flags.c_contiguous:
            rgb = _np.ascontiguousarray(rgb)
        return rgb.copy()

    def _numpy_from_qimage(self, image: QImage) -> _np.ndarray:
        converted = image.convertToFormat(QImage.Format.Format_RGB888)
        ptr = converted.bits()
        ptr.setsize(converted.bytesPerLine() * converted.height())
        arr = _np.frombuffer(ptr, dtype=_np.uint8)
        arr = arr.reshape((converted.height(), converted.bytesPerLine()))
        arr = arr[:, : converted.width() * 3]
        arr = arr.reshape((converted.height(), converted.width(), 3))
        return _np.array(arr, copy=True)

    def _qimage_from_rgb(self, rgb: _np.ndarray) -> QImage:
        if rgb is None or rgb.ndim != 3:
            raise ValueError("Expected RGB array for conversion")
        height, width = rgb.shape[:2]
        if not rgb.flags.c_contiguous:
            rgb = _np.ascontiguousarray(rgb)
        bytes_per_line = width * 3
        image = QImage(rgb.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
        return image.copy()

    def _backfill_robot(self, annotated_rgb: Optional[_np.ndarray]) -> None:
        robot = self._robot_ref
        if robot is None:
            return
        try:
            robot.annotated_image = None if annotated_rgb is None else annotated_rgb.copy()
        except Exception:
            pass

    def _notify(self) -> None:
        callback = self._notifier
        if callback is None:
            return
        try:
            callback()
        except Exception:  # pragma: no cover - defensive logging omitted
            pass

    @staticmethod
    def _clone_or_none(image: Optional[QImage]) -> Optional[QImage]:
        if image is None or image.isNull():
            return None
        return image.copy()

    @staticmethod
    def _placeholder() -> QImage:
        placeholder = QImage(1, 1, QImage.Format.Format_RGB888)
        placeholder.fill(0)
        return placeholder


def _update_size(target, image: QImage) -> None:
    if target is None:
        return
    try:
        target.setWidth(image.width())
        target.setHeight(image.height())
    except Exception:  # pragma: no cover - e.g. mocked QSize
        pass


__all__ = ["CameraImageProvider"]
