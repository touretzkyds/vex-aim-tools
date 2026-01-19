"""Qt Quick image provider for Occupancy Grid visualization."""

from __future__ import annotations

import threading
from typing import Any, Optional

import numpy as np
from PyQt6.QtGui import QImage
from PyQt6.QtQuick import QQuickImageProvider, QQuickTextureFactory

from perception.occupancy_grid import OccupancyGrid, CellState


class OccupancyGridProvider(QQuickImageProvider):
    """Provider serving occupancy grid as a heatmap/mask image."""

    def __init__(self) -> None:
        # Use Image; we still implement requestTexture for Quick 3D consumers.
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._verbose = False
        self._lock = threading.Lock()
        self._grid: Optional[OccupancyGrid] = None
        self._cached_image: Optional[QImage] = None
        self._dirty = False
        self._grid_version: int = -1
        self._debug_prints = 0

    def set_grid(self, grid: Optional[OccupancyGrid]) -> None:
        with self._lock:
            # Avoid redundant work if same object
            if self._grid is grid:
                return
            if self._verbose:
                print(f"[GridProvider] CONNECTING NEW GRID! ID={id(grid) if grid else 'None'}")
            self._grid = grid
            self._grid_version = -1  # force redraw
            self._dirty = True
            if grid is not None:
                grid.on_update = self._mark_dirty

    def requestImage(self, image_id: str, *args):  # type: ignore[override]
        # QQuickImageProvider can be called from multiple threads (e.g. when QML
        # Image.asynchronous is enabled). Protect cached QImage + version state.
        with self._lock:
            if self._verbose:
                print(
                    f"[GridProvider] READING from Grid ID: {id(self._grid) if self._grid else 'None'} "
                    f"image_id={image_id}"
                )
            if self._grid:
                current_version = getattr(self._grid, "version", -1)
                if current_version != self._grid_version:
                    self._grid_version = current_version
                    self._dirty = True
            if self._dirty and self._grid:
                self._update_cache()

            if self._cached_image:
                image = self._cached_image.copy()
            else:
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

    def requestTexture(self, image_id: str, size, options):  # type: ignore[override]
        """Provide texture for Qt Quick / Quick3D."""
        if self._verbose:
            print(f"[GridProvider] requestTexture id={image_id}")
        img = self.requestImage(image_id)
        if isinstance(img, tuple):
            img = img[0]
        return QQuickTextureFactory.textureFactoryForImage(img)

    def _mark_dirty(self) -> None:
        # Boolean assignment is atomic in Python, no lock needed
        self._dirty = True

    def _update_cache(self) -> None:
        if not self._grid:
            return

        # Lock-free snapshot read (no UI blocking!)
        log_odds, cliff_mask, observed, version = self._grid.get_snapshot()

        # ✅ FIX Bug #3: Always process if dirty, version just for tracking
        # Removed early return that could skip updates
        self._grid_version = version
        height, width = log_odds.shape
        
        # Create RGBA buffer
        # Shape: (H, W, 4)
        rgba = np.zeros((height, width, 4), dtype=np.uint8)
        
        # Colors (R, G, B, A)
        COLOR_FREE = np.array([0, 255, 0, 100], dtype=np.uint8)      # Transparent Green
        COLOR_OCCUPIED = np.array([255, 0, 0, 200], dtype=np.uint8)  # Red
        COLOR_CLIFF = np.array([255, 255, 0, 255], dtype=np.uint8)   # Yellow
        COLOR_UNKNOWN = np.array([50, 50, 50, 50], dtype=np.uint8)   # Light gray for debugging
        
        # Masks
        mask_cliff = cliff_mask
        occ_thresh = float(getattr(self._grid, "LO_THRESH_OCC", 0.5))
        free_thresh = float(getattr(self._grid, "LO_THRESH_FREE", -0.5))
        mask_occupied = (log_odds > occ_thresh)
        mask_free = (log_odds < free_thresh)
        mask_unknown = ~(mask_occupied | mask_free)
        mask_seen_unknown = mask_unknown & observed
        mask_unseen_unknown = mask_unknown & ~observed
        
        # Apply colors (Order matters: Cliff > Occupied > Free > Unknown)
        # Write into a flipped view to avoid an extra np.flipud copy.
        rgba_view = rgba[::-1]  # reversed rows, so row 0 maps to y_min
        COLOR_UNKNOWN_SEEN = np.array([80, 80, 80, 80], dtype=np.uint8)   # Seen but uncertain
        COLOR_UNKNOWN_UNSEEN = np.array([30, 30, 30, 40], dtype=np.uint8) # Never observed

        rgba_view[:] = COLOR_UNKNOWN_UNSEEN
        rgba_view[mask_seen_unknown] = COLOR_UNKNOWN_SEEN
        rgba_view[mask_free] = COLOR_FREE
        rgba_view[mask_occupied] = COLOR_OCCUPIED
        rgba_view[mask_cliff] = COLOR_CLIFF
        if self._verbose and self._debug_prints < 5:
            print(f"[GridProvider] updated: occ={int(mask_occupied.sum())} free={int(mask_free.sum())} cliff={int(mask_cliff.sum())}")
            self._debug_prints += 1
        
        # Convert to QImage
        # QImage needs data to be C-contiguous
        if not rgba.flags.c_contiguous:
            rgba = np.ascontiguousarray(rgba)
            
        self._rgba_buffer = rgba # Keep reference to avoid GC
        
        # Create QImage from buffer
        img = QImage(rgba.data, width, height, width * 4, QImage.Format.Format_RGBA8888)
        self._cached_image = img.copy() # Copy to detach from numpy buffer
        self._dirty = False

    @staticmethod
    def _placeholder() -> QImage:
        return QImage(1, 1, QImage.Format.Format_RGBA8888)

def _update_size(target, image: QImage) -> None:
    if target is None:
        return
    try:
        target.setWidth(image.width())
        target.setHeight(image.height())
    except Exception:
        pass
