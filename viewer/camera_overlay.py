"""Helpers for drawing camera overlays on RGB frames."""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as _np

try:  # OpenCV provides fast drawing primitives when available.
    import cv2 as _cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    _cv2 = None  # type: ignore


_AI_NAME_COLORS = {
    "SportsBall": (255, 255, 0),      # Legacy: yellow (255, 255, 0)
    "OrangeBarrel": (255, 50, 50),    # Legacy: red (255, 50, 50)
    "BlueBarrel": (50, 100, 255),     # Legacy: (50, 100, 255)
    "Robot": (255, 255, 255),         # Legacy: white (255, 255, 255)
}
_DEFAULT_COLOR = (255, 0, 0)


def _clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def _draw_rect_numpy(img: _np.ndarray, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    x0 = _clamp(x0, 0, w - 1)
    x1 = _clamp(x1, 0, w - 1)
    y0 = _clamp(y0, 0, h - 1)
    y1 = _clamp(y1, 0, h - 1)
    if x0 > x1:
        x0, x1 = x1, x0
    if y0 > y1:
        y0, y1 = y1, y0
    img[y0, x0:x1 + 1] = color
    img[y1, x0:x1 + 1] = color
    img[y0:y1 + 1, x0] = color
    img[y0:y1 + 1, x1] = color


def _draw_line_numpy(img: _np.ndarray, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    x0 = _clamp(x0, 0, w - 1)
    x1 = _clamp(x1, 0, w - 1)
    y0 = _clamp(y0, 0, h - 1)
    y1 = _clamp(y1, 0, h - 1)
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        img[y, x] = color
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def draw_aiobj_boxes(img_rgb: _np.ndarray, items: Iterable[dict], scale: int) -> None:
    if img_rgb is None or img_rgb.ndim != 3:
        return
    for item in items:
        try:
            x = int(round(item.get("originx", 0) * scale))
            y = int(round(item.get("originy", 0) * scale))
            w = int(round(item.get("width", 0) * scale))
            h = int(round(item.get("height", 0) * scale))
            name = str(item.get("name", ""))
        except Exception:  # pragma: no cover - corrupted metadata
            continue
        color = _AI_NAME_COLORS.get(name, _DEFAULT_COLOR)
        x1 = x + max(0, w - 1)
        y1 = y + max(0, h - 1)
        if _cv2 is not None:  # pragma: no cover - exercised when OpenCV present
            try:
                _cv2.rectangle(img_rgb, (x, y), (x1, y1), color, thickness=4, lineType=_cv2.LINE_8)
                continue
            except Exception:
                pass
        _draw_rect_numpy(img_rgb, x, y, x1, y1, color)


def draw_tag_quads(img_rgb: _np.ndarray, items: Iterable[dict], scale: int) -> None:
    if img_rgb is None or img_rgb.ndim != 3:
        return
    for item in items:
        try:
            pts = [
                (int(round(item.get("x0", 0) * scale)), int(round(item.get("y0", 0) * scale))),
                (int(round(item.get("x1", 0) * scale)), int(round(item.get("y1", 0) * scale))),
                (int(round(item.get("x2", 0) * scale)), int(round(item.get("y2", 0) * scale))),
                (int(round(item.get("x3", 0) * scale)), int(round(item.get("y3", 0) * scale))),
            ]
        except Exception:
            continue
        color = (0, 255, 255)
        if _cv2 is not None:  # pragma: no cover - exercised when OpenCV present
            try:
                _cv2.polylines(img_rgb, [_np.asarray(pts, dtype=_np.int32)], True, color, thickness=2, lineType=_cv2.LINE_8)
                continue
            except Exception:
                pass
        for i in range(len(pts)):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % len(pts)]
            _draw_line_numpy(img_rgb, x0, y0, x1, y1, color)


def apply_overlays(
    image_rgb: _np.ndarray,
    status: Optional[dict],
    scale: int,
    aruco_detector: Optional[object],
) -> _np.ndarray:
    if image_rgb is None or image_rgb.ndim != 3:
        return image_rgb

    out = image_rgb.copy()

    items = []
    try:
        items = ((((status or {}).get("aivision") or {}).get("objects") or {}).get("items") or [])
    except Exception:
        items = []

    try:
        ai_items = [item for item in items if str(item.get("type_str")) == "aiobj"]
    except Exception:
        ai_items = []
    if ai_items:
        try:
            draw_aiobj_boxes(out, ai_items, scale)
        except Exception:
            pass

    try:
        tag_items = [item for item in items if str(item.get("type_str")) == "tag"]
    except Exception:
        tag_items = []
    if tag_items:
        try:
            draw_tag_quads(out, tag_items, scale)
        except Exception:
            pass

    if aruco_detector is not None:
        try:
            if getattr(aruco_detector, "seen_marker_ids", None):
                maybe = aruco_detector.annotate(out, scale)
                if isinstance(maybe, _np.ndarray) and maybe.ndim == 3:
                    out = maybe
        except Exception:
            pass

    return out


__all__ = ["apply_overlays", "draw_aiobj_boxes", "draw_tag_quads"]
