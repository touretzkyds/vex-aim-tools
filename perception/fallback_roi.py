"""Fallback region-of-interest generator for cliff detection."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class FallbackBand:
    """Conservative no-go band represented by two endpoints in pixel space."""

    start: Tuple[float, float]
    end: Tuple[float, float]
    thickness_px: float
    confidence: float

    @property
    def length(self) -> float:
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        return float(math.hypot(dx, dy))


def generate_fallback_bands(
    depth_map: np.ndarray,
    *,
    valid_mask: Optional[np.ndarray] = None,
    min_fraction_invalid: float = 0.4,
    magnitude_thresh: float = 0.08,
    accumulator_threshold: int = 30,
    min_length_px: float = 40.0,
    line_thickness_px: float = 8.0,
    max_candidates: int = 2,
) -> Sequence[FallbackBand]:
    """Produce conservative no-go bands using gradient + Hough heuristics.

    The algorithm follows a weak-signal philosophy: only when the depth/validity
    map is too sparse do we accept these bands, and they are intentionally broad
    to keep the robot safe until the primary detector recovers.
    """

    depth = np.asarray(depth_map, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"Expected 2D depth map, received {depth.shape}")

    h, w = depth.shape
    total_pixels = h * w

    if valid_mask is not None:
        valid = valid_mask.astype(bool)
        invalid_fraction = 1.0 - float(np.count_nonzero(valid)) / max(total_pixels, 1)
    else:
        valid = np.isfinite(depth) & (depth > 0.0)
        invalid_fraction = float(np.count_nonzero(~valid)) / max(total_pixels, 1)

    if invalid_fraction < min_fraction_invalid:
        return []

    # Compute normalized gradients on the valid subset.
    grad_y, grad_x = np.gradient(np.where(valid, depth, 0.0))
    magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)

    # Focus on the strongest gradients (potential edges).
    edge_mask = magnitude > magnitude_thresh
    if not edge_mask.any():
        return []

    votes = _hough_lines(edge_mask, max_theta=180, theta_step=2)
    if not votes:
        return []

    bands: List[FallbackBand] = []
    for rho, theta, vote_count in votes[:max_candidates]:
        if vote_count < accumulator_threshold:
            continue
        start, end = _clip_line_to_bounds(rho, theta, w, h)
        if start is None or end is None:
            continue
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        if segment_length < min_length_px:
            continue
        confidence = min(0.5, vote_count / (total_pixels * 0.25 + 1e-6))
        bands.append(
            FallbackBand(
                start=start,
                end=end,
                thickness_px=line_thickness_px,
                confidence=float(confidence),
            )
        )

    return bands


def _hough_lines(edge_mask: np.ndarray, max_theta: int, theta_step: int) -> List[Tuple[float, float, int]]:
    h, w = edge_mask.shape
    diag_len = int(round(math.hypot(h, w)))
    rhos = np.linspace(-diag_len, diag_len, diag_len * 2 + 1)
    thetas = np.deg2rad(np.arange(0, max_theta, theta_step))

    accumulator = np.zeros((len(rhos), len(thetas)), dtype=np.int32)
    ys, xs = np.nonzero(edge_mask)
    if ys.size == 0:
        return []

    sin_theta = np.sin(thetas)
    cos_theta = np.cos(thetas)

    for y, x in zip(ys, xs):
        r = x * cos_theta + y * sin_theta
        rho_indices = np.round(r + diag_len).astype(int)
        for idx, theta_idx in enumerate(range(len(thetas))):
            rho_idx = rho_indices[idx]
            if 0 <= rho_idx < accumulator.shape[0]:
                accumulator[rho_idx, theta_idx] += 1

    candidates: List[Tuple[float, float, int]] = []
    for rho_idx in range(accumulator.shape[0]):
        for theta_idx in range(accumulator.shape[1]):
            votes = accumulator[rho_idx, theta_idx]
            if votes <= 0:
                continue
            rho = rhos[rho_idx]
            theta = thetas[theta_idx]
            candidates.append((rho, float(theta), int(votes)))

    candidates.sort(key=lambda item: item[2], reverse=True)
    return candidates


def _clip_line_to_bounds(
    rho: float,
    theta: float,
    width: int,
    height: int,
) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
    """Clip a (rho, theta) line to the image bounds."""

    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    if abs(sin_t) < 1e-6 and abs(cos_t) < 1e-6:
        return None, None

    points: List[Tuple[float, float]] = []
    boundaries = (
        (0, lambda y: (rho - y * sin_t) / cos_t if abs(cos_t) > 1e-6 else None),
        (width, lambda y: (rho - y * sin_t) / cos_t if abs(cos_t) > 1e-6 else None),
    )

    # x = constant boundaries.
    for x_const, func in boundaries:
        y = (rho - x_const * cos_t) / sin_t if abs(sin_t) > 1e-6 else None
        if y is not None and 0 <= y <= height:
            points.append((float(x_const), float(y)))

    # y = constant boundaries.
    for y_const in (0, height):
        x = (rho - y_const * sin_t) / cos_t if abs(cos_t) > 1e-6 else None
        if x is not None and 0 <= x <= width:
            points.append((float(x), float(y_const)))

    if len(points) < 2:
        return None, None

    # Choose the pair farthest apart.
    best_pair = (points[0], points[1])
    best_dist = 0.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            dist = dx * dx + dy * dy
            if dist > best_dist:
                best_dist = dist
                best_pair = (points[i], points[j])
    return best_pair


__all__ = ["FallbackBand", "generate_fallback_bands"]
