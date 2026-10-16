"""Global window manager and cv2-style API for PyQt6 imshow()."""

from __future__ import annotations

import threading
import time
from queue import Empty, SimpleQueue
from typing import Dict, Optional

import numpy as np
from PyQt6.QtCore import QCoreApplication, QEventLoop, QThread

from .imshow_window import ImshowWindow


class WindowManager:
    """Global registry for imshow windows.

    Manages multiple named windows, mimicking cv2's global window management.
    Thread-safe singleton pattern using module-level instance.
    """

    def __init__(self) -> None:
        self._windows: Dict[str, ImshowWindow] = {}
        self._lock = threading.Lock()
        self._pending_ops: SimpleQueue[tuple[str, str | None]] = SimpleQueue()
        self._pending_images: Dict[str, np.ndarray] = {}
        self._pending_lock = threading.Lock()
        self._processing = False

    @staticmethod
    def _on_main_thread() -> bool:
        return threading.current_thread() is threading.main_thread()

    def create_window(self, window_name: str) -> ImshowWindow:
        """Create or retrieve window by name.

        Args:
            window_name: Name of the window

        Returns:
            ImshowWindow instance (creates new if doesn't exist)

        Raises:
            ValueError: If window_name is empty
        """
        if not isinstance(window_name, str) or not window_name:
            raise ValueError("window_name must be a non-empty string")

        with self._lock:
            if window_name not in self._windows:
                self._windows[window_name] = ImshowWindow(window_name)
            return self._windows[window_name]

    def get_window(self, window_name: str) -> Optional[ImshowWindow]:
        """Get existing window or None.

        Args:
            window_name: Name of the window

        Returns:
            ImshowWindow instance or None if not found
        """
        with self._lock:
            return self._windows.get(window_name)

    def destroy_window(self, window_name: str) -> None:
        """Close and remove window.

        Args:
            window_name: Name of the window to destroy
        """
        if not self._on_main_thread():
            self._pending_ops.put(("destroyWindow", window_name))
            return
        self._destroy_window_main_thread(window_name)

    def destroy_all_windows(self) -> None:
        """Close and remove all windows."""
        if not self._on_main_thread():
            self._pending_ops.put(("destroyAllWindows", None))
            return
        self._destroy_all_windows_main_thread()

    def imshow(self, window_name: str, image: np.ndarray) -> None:
        """Display image in named window (creates if needed).

        Args:
            window_name: Name of the window
            image: HxW grayscale, HxWx3 RGB, or HxWx4 RGBA numpy array (uint8)
        """
        if not self._on_main_thread():
            window = self.get_window(window_name)
            if window is not None and window.is_visible():
                window.display(image)
                return
            with self._pending_lock:
                self._pending_images[window_name] = image
            self._pending_ops.put(("namedWindow", window_name))
            return
        self._imshow_main_thread(window_name, image)
        self._process_events()

    def process_pending(self, process_events: bool = True) -> bool:
        if not self._on_main_thread():
            return False
        if self._processing:
            return False
        self._processing = True
        try:
            pending_ops = []
            while True:
                try:
                    pending_ops.append(self._pending_ops.get_nowait())
                except Empty:
                    break

            for action, window_name in pending_ops:
                if action == "namedWindow" and window_name is not None:
                    self._named_window_main_thread(window_name)
                elif action == "destroyWindow" and window_name is not None:
                    self._destroy_window_main_thread(window_name)
                elif action == "destroyAllWindows":
                    self._destroy_all_windows_main_thread()

            with self._pending_lock:
                pending_images = dict(self._pending_images)
                self._pending_images.clear()

            for window_name, image in pending_images.items():
                self._imshow_main_thread(window_name, image)

            processed = bool(pending_ops or pending_images)
            if process_events and processed:
                self._process_events()
            return processed
        finally:
            self._processing = False

    def _imshow_main_thread(self, window_name: str, image: np.ndarray) -> None:
        window = self.get_window(window_name)
        if window is None:
            window = self.create_window(window_name)
        if not window.is_visible():
            window.show()

        window.display(image)

    def _named_window_main_thread(self, window_name: str) -> None:
        window = self.create_window(window_name)
        if not window.is_visible():
            window.show()

    def _destroy_window_main_thread(self, window_name: str) -> None:
        with self._lock:
            window = self._windows.pop(window_name, None)
            if window is not None:
                window.close()

    def _destroy_all_windows_main_thread(self) -> None:
        with self._lock:
            for window in self._windows.values():
                window.close()
            self._windows.clear()

    def _process_events(self, delay_ms: int = 0) -> None:
        app = QCoreApplication.instance()
        if app is None or QThread.currentThread() != app.thread():
            return

        if delay_ms <= 0:
            # Single event processing pass
            app.processEvents(QEventLoop.ProcessEventsFlag.AllEvents)
            return

        # Process events for specified duration
        start = time.perf_counter()
        while (time.perf_counter() - start) * 1000 < delay_ms:
            app.processEvents(QEventLoop.ProcessEventsFlag.AllEvents)
            time.sleep(0.001)  # Minimal sleep to prevent CPU spinning


# Module-level singleton
_manager = WindowManager()


# cv2-style API functions
def namedWindow(window_name: str, flags: int = 0) -> None:
    """Create a named window (cv2-compatible API).

    Args:
        window_name: Name of the window
        flags: Ignored (for cv2 compatibility)
    """
    if _manager._on_main_thread():
        _manager._named_window_main_thread(window_name)
        _manager._process_events()
    else:
        _manager._pending_ops.put(("namedWindow", window_name))


def imshow(window_name: str, image: np.ndarray) -> None:
    """Display image in named window (cv2-compatible API).

    Args:
        window_name: Name of the window
        image: HxW grayscale, HxWx3 RGB, or HxWx4 RGBA numpy array (uint8)
    """
    _manager.imshow(window_name, image)


def destroyWindow(window_name: str) -> None:
    """Close and destroy named window (cv2-compatible API).

    Args:
        window_name: Name of the window to destroy
    """
    _manager.destroy_window(window_name)


def destroyAllWindows() -> None:
    """Close and destroy all windows (cv2-compatible API)."""
    _manager.destroy_all_windows()


def imshow_pump() -> bool:
    """Process queued imshow window operations on the main thread."""
    return _manager.process_pending()


__all__ = [
    "namedWindow",
    "imshow",
    "destroyWindow",
    "destroyAllWindows",
    "imshow_pump",
    "WindowManager",
]
