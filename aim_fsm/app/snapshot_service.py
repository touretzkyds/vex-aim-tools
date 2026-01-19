"""Snapshot filename management for camera captures."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from aim_fsm.app.constants import (
    SNAPSHOT_DIR,
    SNAPSHOT_NAME_TEMPLATE,
    SNAP_SUFFIX_ANN,
    SNAP_SUFFIX_RAW,
)


class SnapshotService:
    """Generate deterministic snapshot file names without touching the filesystem."""

    def __init__(self, name: str = "robot", out_dir: str = SNAPSHOT_DIR) -> None:
        self._name = name
        self._out_dir = out_dir
        self._snapno = 0
        self._disk_write_enabled = False

    def next_path_raw(self) -> str:
        """Return the next raw snapshot path."""
        path = self._format_path(SNAP_SUFFIX_RAW)
        self._snapno += 1
        return path

    def next_path_annotated(self) -> str:
        """Return the next annotated snapshot path."""
        path = self._format_path(SNAP_SUFFIX_ANN)
        self._snapno += 1
        return path

    def enable_disk_write(self, enabled: bool) -> None:
        """Toggle optional snapshot persistence to disk."""

        self._disk_write_enabled = bool(enabled)

    def save_rgb_png(self, path: str, image_rgb: Any) -> None:
        """Persist an RGB image to ``path`` if disk writes are enabled."""

        if not self._disk_write_enabled:
            return
        if not isinstance(path, str):
            raise TypeError("Snapshot path must be a string")
        if image_rgb is None:
            raise ValueError("Image data is required for snapshot persistence")

        arr = np.asarray(image_rgb)
        if arr.ndim != 3 or arr.shape[2] < 3:
            raise ValueError("Expected HxWx3 RGB array for snapshot data")
        rgb = np.array(arr[..., :3], dtype=np.uint8, copy=True, order="C")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        target_path = Path(path)
        try:
            target_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        cv2.imwrite(str(target_path), bgr)

    def _format_path(self, suffix: str) -> str:
        name_with_suffix = f"{self._name}{suffix}"
        return SNAPSHOT_NAME_TEMPLATE.format(
            path=self._out_dir,
            name=name_with_suffix,
            snapno=self._snapno,
        )


__all__ = ["SnapshotService"]
