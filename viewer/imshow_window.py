"""QtQuick-based window for displaying RGB numpy images via imshow()."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PyQt6.QtCore import QMetaObject, QObject, Qt, QUrl, pyqtSlot
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQml import QQmlContext
from PyQt6.QtQuick import QQuickView

from .imshow_provider import ImshowImageProvider


class ImshowWindow(QObject):
    """Window wrapper for displaying numpy arrays using PyQt6.

    Simplified version of CamViewer focused on displaying static images
    without polling, snapshots, or complex state management.
    """

    def __init__(self, window_name: str, width: int = 640, height: int = 480) -> None:
        """Initialize imshow window.

        Args:
            window_name: Name displayed in window title bar
            width: Initial window width
            height: Initial window height
        """
        super().__init__(parent=None)

        self._window_name = window_name
        self._width = width
        self._height = height

        # Get or create QGuiApplication singleton
        self._app = QGuiApplication.instance() or QGuiApplication([])

        self._provider = ImshowImageProvider()
        self._provider.register_notifier(self._queue_frame_bump)

        self._frame_counter = 0

        # Setup QQuickView
        self._view = QQuickView()
        self._view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self._view.setTitle(window_name)

        self._context = self._initialize_qml_context()

    def _initialize_qml_context(self) -> QQmlContext:
        """Initialize QML context with image provider and properties."""
        repo_root = Path(__file__).resolve().parents[1]
        qml_path = (repo_root / "qml" / "SimpleImshow.qml").resolve()

        engine = self._view.engine()
        engine.addImportPath(str(repo_root / "qml"))
        engine.addImageProvider("imshow", self._provider)

        context = self._view.rootContext()
        context.setContextProperty("frameId", self._frame_counter)

        self._view.setSource(QUrl.fromLocalFile(str(qml_path)))

        return context

    def show(self) -> None:
        """Display the window."""
        self._view.setWidth(self._width)
        self._view.setHeight(self._height)
        self._view.show()

    def close(self) -> None:
        """Close the window."""
        self._view.close()

    def display(self, image: np.ndarray) -> None:
        """Update displayed image.

        Args:
            image: HxW grayscale, HxWx3 RGB, or HxWx4 RGBA numpy array (uint8)
        """
        self._provider.update_image(image)

    def is_visible(self) -> bool:
        """Check if window is currently visible."""
        return self._view.isVisible()

    def _queue_frame_bump(self) -> None:
        """Schedule frame counter increment (thread-safe).

        Uses QMetaObject.invokeMethod with QueuedConnection to explicitly
        guarantee execution on the Qt GUI thread, regardless of which thread
        calls this method.
        """
        if QGuiApplication.instance() is None:
            self._increment_frame()
        else:
            QMetaObject.invokeMethod(
                self, "_increment_frame", Qt.ConnectionType.QueuedConnection
            )

    @pyqtSlot()
    def _increment_frame(self) -> None:
        """Increment frame counter to trigger QML refresh.

        This causes QML to re-request the image from the provider.
        """
        self._frame_counter += 1
        if self._context is not None:
            self._context.setContextProperty("frameId", self._frame_counter)
