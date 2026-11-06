"""Save the cliff log-odds masks the same way the depth pipeline is dumped.

Each ``detect_cliff`` writes the frames under
``cliff_frames/cliff_grid_debug/<time>/``. A second ``detect_cliff`` after a
turn writes ``pass2`` into the same folder.
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

_ROOT = None
_PASS = 0


def _root() -> Path:
    global _ROOT
    if _ROOT is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        _ROOT = Path(__file__).resolve().parents[1] / "cliff_frames" / "cliff_grid_debug" / stamp
        _ROOT.mkdir(parents=True, exist_ok=True)
        print(f"Cliff grid images: {_ROOT}")
    return _ROOT


def _next_pass() -> int:
    global _PASS
    _PASS += 1
    print(f"Cliff grid debug pass {_PASS}")
    return _PASS


class CliffGridDebug:
    """One of these is attached for a single ``detect_cliff`` command."""

    def __init__(self):
        self.pass_index = _next_pass()
        self.seg_index = 0
        self.frame_index = 0
        self.root = _root()
        self.pending = []

    def __call__(self, grid, stage, payload):
        if stage == "paint":
            images = _paint_images(grid, payload, self.pass_index, self.seg_index)
            self.pending.append(images)
            tag = f"pass{self.pass_index}_seg{self.seg_index:02d}"
            _publish(self.root, tag, images)
            _append_summary(self.root, _paint_summary(grid, payload, self.pass_index, self.seg_index))
            self.seg_index += 1
            return
        images = _extract_images(grid, payload, self.pass_index, self.frame_index)
        tag = f"pass{self.pass_index}_frame{self.frame_index:02d}"
        _publish(self.root, tag, images)
        _append_summary(self.root, _extract_summary(payload, self.pass_index, self.frame_index))
        sheet_images = []
        for painted in self.pending:
            sheet_images.extend(painted.values())
        sheet_images.extend(images.values())
        sheet = _sheet(sheet_images)
        if sheet is not None:
            _save(self.root / f"pass{self.pass_index}_frame{self.frame_index:02d}_sheet.png", sheet)
        self.pending = []
        self.frame_index += 1


def _paint_images(grid, payload, pass_index, seg_index):
    safe = payload["safe"]
    drop = payload["drop"]
    weights = np.asarray(payload["weights"], dtype=np.float32)
    rays = payload["rays"]
    robot_xy = payload["robot_xy"]
    score = np.array(grid.cliff_log_odds, copy=True)
    ray_cells = _ray_cells(grid, rays, safe.shape)
    box = _crop_box(grid, (safe, drop, ray_cells, weights > 0), score, robot_xy)
    prefix = f"pass {pass_index}  seg {seg_index}"
    labels = np.zeros(safe.shape, np.uint8)
    labels[safe & ~drop] = 1
    labels[drop] = 2
    ray_weights = [ray[-1] for ray in rays]
    if ray_weights:
        weight_note = f"{min(ray_weights):.2f}-{max(ray_weights):.2f}"
    else:
        weight_note = "none"
    return {
        "01_rays": _render_rays(
            grid, rays, box, robot_xy,
            f"{prefix}  {len(rays)} rays to the edge  weight {weight_note}",
        ),
        "02_ray_cells": _render_labels(
            grid, labels, box, {1: (40, 190, 70), 2: (220, 50, 50)}, robot_xy,
            f"{prefix}  cells along rays  safe {int((safe & ~drop).sum())}  drop {int(drop.sum())}",
        ),
        "03_weights": _render_weights(
            grid, weights, box, robot_xy,
            f"{prefix}  confidence weight  conf {float(payload['weight']):.2f}  {weight_note}",
        ),
        "04_log_odds": _render_score(
            grid, score, box, robot_xy,
            f"{prefix}  log odds after vote  [{score.min():+.2f}, {score.max():+.2f}]",
        ),
    }


def _extract_images(grid, payload, pass_index, frame_index):
    score = np.array(grid.cliff_log_odds, copy=True)
    edge = payload["edge"]
    robot_xy = None
    box = _crop_box(grid, (score < 0, score > 0, edge), score, robot_xy)
    prefix = f"pass {pass_index}  frame {frame_index}"
    epsilon = float(payload.get("epsilon_mm", grid.cliff_resolution))
    simplified = payload["simplified"]
    return {
        "05_boundary": _render_mask(
            grid, edge, box, (255, 220, 40), robot_xy,
            f"{prefix}  boundary  {int(edge.sum())} cells",
        ),
        "06_chain": _draw_world(
            grid, box, score, payload["chains"], (80, 220, 255), robot_xy,
            f"{prefix}  contour  {len(payload['chains'])} chains",
        ),
        "07_simplified": _draw_world(
            grid, box, score, simplified, (255, 220, 0), robot_xy,
            f"{prefix}  Douglas-Peucker  eps {epsilon:.0f} mm  {len(simplified)} lines",
        ),
    }


def _crop_box(grid, masks, score, robot_xy):
    ys = []
    xs = []
    for mask in masks:
        if mask is None:
            continue
        yy, xx = np.nonzero(mask)
        if len(yy):
            ys.append(yy)
            xs.append(xx)
    if score is not None:
        yy, xx = np.nonzero(np.abs(score) > 1e-6)
        if len(yy):
            ys.append(yy)
            xs.append(xx)
    if robot_xy is not None:
        gx, gy = grid.cliff_world_to_grid(float(robot_xy[0]), float(robot_xy[1]))
        if gx >= 0:
            ys.append(np.array([gy], dtype=int))
            xs.append(np.array([gx], dtype=int))
    height = grid.cliff_height
    width = grid.cliff_width
    for mask in masks:
        if mask is not None and getattr(mask, "ndim", 0) == 2:
            height, width = mask.shape
            break
    if not ys:
        return 0, min(height, 40), 0, min(width, 40)
    pad = 12
    y0 = max(int(min(v.min() for v in ys)) - pad, 0)
    x0 = max(int(min(v.min() for v in xs)) - pad, 0)
    y1 = min(int(max(v.max() for v in ys)) + pad + 1, height)
    x1 = min(int(max(v.max() for v in xs)) + pad + 1, width)
    return y0, y1, x0, x1


def _view_size(height, width):
    long_side = max(height, width, 1)
    if long_side < 120:
        scale = min(6.0, 480.0 / long_side)
    else:
        scale = min(2.0, 720.0 / long_side)
    out_w = max(int(round(width * scale)), 1)
    out_h = max(int(round(height * scale)), 1)
    return out_w, out_h


def _blank(box):
    y0, y1, x0, x1 = box
    return np.full((y1 - y0, x1 - x0, 3), 24, np.uint8)


def _finish(cell_rgb, box, title):
    y0, y1, x0, x1 = box
    flipped = np.flipud(cell_rgb)
    out_w, out_h = _view_size(y1 - y0, x1 - x0)
    view = cv2.resize(flipped, (out_w, out_h), interpolation=cv2.INTER_NEAREST)
    if view.shape[1] < 420:
        pad = 420 - view.shape[1]
        view = np.pad(view, ((0, 0), (0, pad), (0, 0)), constant_values=24)
    bar_h = 36 if len(title) > 70 else 26
    bar = np.full((bar_h, view.shape[1], 3), 0, np.uint8)
    cv2.putText(
        bar, title[:110], (6, bar_h - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return np.vstack((bar, view)), (y0, y1, x0, x1, out_w, out_h, bar_h)


def _render_mask(grid, mask, box, color, robot_xy, title):
    img = _blank(box)
    y0, y1, x0, x1 = box
    sub = np.asarray(mask[y0:y1, x0:x1], dtype=bool)
    img[sub] = color
    view, geom = _finish(img, box, title)
    _dot(view, grid, robot_xy, geom)
    return view


def _render_labels(grid, labels, box, colors, robot_xy, title):
    img = _blank(box)
    y0, y1, x0, x1 = box
    sub = labels[y0:y1, x0:x1]
    for value, color in colors.items():
        img[sub == value] = color
    view, geom = _finish(img, box, title)
    _dot(view, grid, robot_xy, geom)
    return view


def _render_weights(grid, weights, box, robot_xy, title):
    img = _blank(box)
    y0, y1, x0, x1 = box
    sub = weights[y0:y1, x0:x1]
    peak = float(sub.max()) if sub.size else 0.0
    scale = sub / peak if peak > 0.0 else sub
    active = sub > 0.0
    img[active, 0] = (40 + scale[active] * 215).astype(np.uint8)
    img[active, 1] = (80 + scale[active] * 140).astype(np.uint8)
    img[active, 2] = (255 - scale[active] * 180).astype(np.uint8)
    view, geom = _finish(img, box, title)
    _dot(view, grid, robot_xy, geom)
    return view


def _render_rays(grid, rays, box, robot_xy, title):
    img = _blank(box)
    view, geom = _finish(img, box, title)
    for rx, ry, ex, ey, dx, dy, sample_weight in rays:
        color = _weight_color(sample_weight)
        _polylines(view, grid, box, [np.array([[rx, ry], [ex, ey]], dtype=float)], color, 1)
        _polylines(view, grid, box, [np.array([[ex, ey], [dx, dy]], dtype=float)], (220, 50, 50), 1)
    _dot(view, grid, robot_xy, geom)
    return view


def _weight_color(sample_weight):
    w = float(np.clip(sample_weight, 0.0, 1.0))
    return (int(40 + 215 * w), int(80 + 140 * w), int(255 - 180 * w))


def _ray_cells(grid, rays, shape):
    mask = np.zeros(shape, dtype=bool)
    for ray in rays:
        for x_mm, y_mm in ((ray[0], ray[1]), (ray[2], ray[3]), (ray[4], ray[5])):
            gx, gy = grid.cliff_world_to_grid(float(x_mm), float(y_mm))
            if gx >= 0:
                mask[gy, gx] = True
    return mask


def _render_score(grid, score, box, robot_xy, title):
    img = _blank(box)
    y0, y1, x0, x1 = box
    sub = score[y0:y1, x0:x1]
    mag = np.clip(np.abs(sub) / grid.LO_MAX, 0.0, 1.0)
    neg = sub < 0
    pos = sub > 0
    img[neg, 2] = (50 + mag[neg] * 205).astype(np.uint8)
    img[pos, 0] = (50 + mag[pos] * 205).astype(np.uint8)
    view, geom = _finish(img, box, title)
    _dot(view, grid, robot_xy, geom)
    return view


def _draw_world(grid, box, score, polylines, color, robot_xy, title):
    base = _render_score(grid, score, box, robot_xy, title)
    _polylines(base, grid, box, polylines, color, 2)
    return base


def _polylines(view, grid, box, polylines, color, thickness):
    y0, y1, x0, x1 = box
    out_w, out_h = _view_size(y1 - y0, x1 - x0)
    bar_h = view.shape[0] - out_h
    for poly in polylines:
        pts = np.asarray(poly, dtype=float)
        if pts.ndim != 2 or len(pts) < 2:
            continue
        px = []
        for x_mm, y_mm in pts[:, :2]:
            gx = (x_mm - grid.x_min) / grid.cliff_resolution - 0.5
            gy = (y_mm - grid.y_min) / grid.cliff_resolution - 0.5
            col = int(round((gx - x0 + 0.5) / max(x1 - x0, 1) * out_w))
            row = int(round(out_h - (gy - y0 + 0.5) / max(y1 - y0, 1) * out_h))
            px.append((col, row + bar_h))
        cv2.polylines(view, [np.array(px, np.int32).reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)


def _dot(view, grid, robot_xy, geom):
    if robot_xy is None:
        return
    y0, y1, x0, x1, out_w, out_h, bar_h = geom
    gx, gy = grid.cliff_world_to_grid(float(robot_xy[0]), float(robot_xy[1]))
    if gx < 0:
        return
    col = int(round((gx - x0 + 0.5) / max(x1 - x0, 1) * out_w))
    row = int(round(out_h - (gy - y0 + 0.5) / max(y1 - y0, 1) * out_h)) + bar_h
    cv2.circle(view, (col, row), 5, (0, 220, 255), -1)


def _publish(root, tag, images):
    for key, image in images.items():
        _save(root / f"{tag}_{key}.png", image)


def _save(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        print(f"failed to write {path}")


def _sheet(images):
    if not images:
        return None
    height = max(im.shape[0] for im in images)
    padded = []
    for im in images:
        if im.shape[0] < height:
            extra = np.full((height - im.shape[0], im.shape[1], 3), 24, np.uint8)
            im = np.vstack((im, extra))
        padded.append(im)
    mid = (len(padded) + 1) // 2
    rows = []
    for group in (padded[:mid], padded[mid:]):
        if group:
            rows.append(np.hstack(group))
    width = max(row.shape[1] for row in rows)
    matched = []
    for row in rows:
        if row.shape[1] < width:
            extra = np.full((row.shape[0], width - row.shape[1], 3), 24, np.uint8)
            row = np.hstack((row, extra))
        matched.append(row)
    sheet = np.vstack(matched)
    if sheet.shape[1] > 1600:
        scale = 1600 / sheet.shape[1]
        sheet = cv2.resize(
            sheet,
            (1600, max(int(sheet.shape[0] * scale), 1)),
            interpolation=cv2.INTER_AREA,
        )
    return sheet


def _append_summary(root, text):
    path = root / "summary.txt"
    with path.open("a", encoding="utf-8") as handle:
        handle.write(text)
        if not text.endswith("\n"):
            handle.write("\n")


def _paint_summary(grid, payload, pass_index, seg_index):
    score = grid.cliff_log_odds
    weights = np.asarray(payload["weights"], dtype=float)
    used = weights[weights > 0]
    if used.size:
        weight_note = f"{float(used.min()):.3f} to {float(used.max()):.3f}"
    else:
        weight_note = "none"
    return (
        f"pass {pass_index} segment {seg_index}\n"
        f"  robot_xy_mm {payload['robot_xy']}\n"
        f"  segment confidence {float(payload['weight']):.3f}\n"
        f"  rays {len(payload['rays'])}  cell weight {weight_note}\n"
        f"  safe cells {int((payload['safe'] & ~payload['drop']).sum())}\n"
        f"  drop cells {int(payload['drop'].sum())}\n"
        f"  score now [{float(score.min()):+.3f}, {float(score.max()):+.3f}]\n"
    )


def _extract_summary(payload, pass_index, frame_index):
    polyline = payload["polyline_mm"]
    if polyline is None:
        length = 0.0
        npts = 0
    else:
        step = np.diff(np.asarray(polyline, dtype=float), axis=0)
        length = float(np.hypot(step[:, 0], step[:, 1]).sum()) if len(step) else 0.0
        npts = len(polyline)
    return (
        f"pass {pass_index} frame {frame_index}\n"
        f"  boundary cells {int(np.asarray(payload['edge']).sum())}\n"
        f"  chains {len(payload['chains'])}  simplified {len(payload['simplified'])}\n"
        f"  epsilon {float(payload.get('epsilon_mm', 0.0)):.1f} mm\n"
        f"  polyline points {npts}  length {length:.1f} mm\n"
    )
