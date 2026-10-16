"""Qt Quick image provider for displaying RGB numpy images via imshow()."""

from __future__ import annotations

import threading
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QSize
from PyQt6.QtGui import QImage
from PyQt6.QtQuick import QQuickImageProvider


class ImshowImageProvider(QQuickImageProvider):
    """Thread-safe provider for displaying numpy arrays in PyQt6 windows.

    Simplified version of CameraImageProvider focused solely on displaying
    static images without overlays, aruco, or robot integration.
    """

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._lock = threading.Lock()
        self._image: Optional[QImage] = None
        self._notifier: Optional[Callable[[], None]] = None

    def register_notifier(self, callback: Optional[Callable[[], None]]) -> None:
        """Register callback to be notified when new image is available."""
        self._notifier = callback

    def update_image(self, image: np.ndarray) -> None:
        """Update the displayed image.

        Args:
            image: HxW grayscale, HxWx3 RGB, or HxWx4 RGBA numpy array (uint8).
                   RGB color order matches the rest of vex-aim-tools.
        """
        if image is None or image.size == 0:
            return  # Silently ignore like cv2

        rgb = self._ensure_rgb(image)
        qimage = self._qimage_from_rgb(rgb)

        with self._lock:
            self._image = qimage

        self._notify()

    def requestImage(self, image_id: str, *args):
        """QQuickImageProvider interface - called by QML Image component."""
        with self._lock:
            image = self._clone_or_placeholder(self._image)

        # Handle PyQt6 overload signatures
        if len(args) == 1:  # size_out
            size_out = args[0]
            _update_size(size_out, image)
            return image, image.size()
        elif len(args) == 2:  # size_out, requested_size
            size_out, _ = args
            _update_size(size_out, image)
            return image
        return image

    def _ensure_rgb(self, arr: np.ndarray) -> np.ndarray:
        """Convert HxW grayscale or HxWx3 RGB to HxWx3 RGB.

        NOTE: Assumes input is RGB.

        Args:
            arr: Input numpy array

        Returns:
            HxWx3 RGB array (uint8, C-contiguous)

        Raises:
            ValueError: If array shape is not supported
        """
        arr = np.asarray(arr, dtype=np.uint8)

        if arr.ndim == 2:
            # Grayscale HxW -> RGB HxWx3
            rgb = np.stack([arr, arr, arr], axis=-1)
        elif arr.ndim == 3 and arr.shape[2] >= 3:
            # RGB (or RGBA -> RGB by dropping alpha)
            rgb = arr[..., :3]
        else:
            raise ValueError(
                f"Expected HxW grayscale or HxWx3/HxWx4 RGB array, "
                f"got shape {arr.shape}"
            )

        if not rgb.flags.c_contiguous:
            rgb = np.ascontiguousarray(rgb)

        return rgb.copy()

    def _qimage_from_rgb(self, rgb: np.ndarray) -> QImage:
        """Convert RGB numpy array to QImage.

        CRITICAL: Must call image.copy() to detach from numpy buffer!
        Otherwise QImage becomes invalid when numpy array is freed.

        Args:
            rgb: HxWx3 RGB array (uint8, C-contiguous)

        Returns:
            QImage with Format_RGB888
        """
        if rgb is None or rgb.ndim != 3:
            raise ValueError("Expected RGB array for conversion")

        height, width = rgb.shape[:2]
        bytes_per_line = width * 3
        image = QImage(
            rgb.data, width, height, bytes_per_line, QImage.Format.Format_RGB888
        )
        return image.copy()  # CRITICAL: detach from numpy buffer

    @staticmethod
    def _clone_or_placeholder(image: Optional[QImage]) -> QImage:
        """Return a copy of the image or a placeholder if None."""
        if image is None or image.isNull():
            # Return 1x1 black image as placeholder
            placeholder = QImage(1, 1, QImage.Format.Format_RGB888)
            placeholder.fill(0)
            return placeholder
        return image.copy()

    def _notify(self) -> None:
        """Trigger notifier callback if registered."""
        callback = self._notifier
        if callback is None:
            return
        try:
            callback()
        except Exception:
            pass  # Silently ignore callback errors


def _update_size(size_out: QSize, image: QImage) -> None:
    """Update QSize with image dimensions."""
    if size_out is not None:
        size_out.setWidth(image.width())
        size_out.setHeight(image.height())
