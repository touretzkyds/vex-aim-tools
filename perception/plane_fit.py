"""Robust plane fitting utilities tailored for DepthAnything point clouds."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class PlaneModel:
    """Representation of a plane in 3D."""

    normal: np.ndarray  # Unit normal vector pointing away from the plane.
    offset: float  # Plane offset `d` in the equation `n·x + d = 0`.

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Compute signed distances from points to the plane."""
        return points @ self.normal + self.offset

    def angle_to(self, direction: np.ndarray) -> float:
        """Return the angle (in radians) between the plane normal and a vector."""
        direction = np.asarray(direction, dtype=np.float32)
        if direction.shape != (3,):
            raise ValueError("direction must be a 3-element vector")
        dot = float(np.dot(self.normal, direction) / (np.linalg.norm(direction) + 1e-12))
        dot = max(-1.0, min(1.0, dot))
        return math.acos(dot)


@dataclass(frozen=True)
class PlaneFitResult:
    """Outcome of a single plane estimation."""

    plane: PlaneModel
    inlier_mask: np.ndarray  # Boolean mask over the original points.
    residuals: np.ndarray  # Absolute point-to-plane distances for inliers.
    mad: float  # Median absolute deviation of residuals.
    rms: float  # Root-mean-square residual.
    support: int  # Number of inliers.
    score: float  # MSAC score (lower is better).
    angle_to_gravity_deg: float  # Angle between plane normal and gravity (deg).

    @property
    def confidence(self) -> float:
        """Heuristic confidence based on residual spread and support."""
        if self.support <= 0:
            return 0.0
        denom = (self.mad + 1e-6) * math.sqrt(self.support)
        raw = 1.0 / (1.0 + denom)
        return float(max(0.0, min(1.0, raw)))


def fit_horizontal_planes(
    points: np.ndarray,
    *,
    gravity: np.ndarray,
    sample_count: int = 256,
    max_planes: int = 2,
    angle_threshold_deg: float = 15.0,
    initial_threshold: float = 0.0,
    mad_scale: float = 2.5,
    seed: Optional[int] = None,
) -> List[PlaneFitResult]:
    """Find horizontal planes using MSAC with adaptive thresholds.

    Args:
        points: Nx3 array of candidate points in camera coordinates.
        gravity: 3-vector representing the gravity direction (camera frame).
        sample_count: Number of random MSAC hypotheses to evaluate.
        max_planes: Extract up to this many planes (non-overlapping inliers).
        angle_threshold_deg: Max angle between plane normal and gravity vector.
        initial_threshold: Initial absolute distance threshold (metres) to clamp
            MSAC residuals before the MAD-based refinement.
        mad_scale: Multiplier applied to `sigma = 1.4826 * MAD` to form the final
            inlier threshold.
        seed: Optional random seed for determinism.

    Returns:
        List of :class:`PlaneFitResult` sorted by descending support.
    """

    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"Expected point array of shape (N, 3), received {pts.shape}")
    if pts.shape[0] < 3:
        return []

    gravity_vec = _normalise(np.asarray(gravity, dtype=np.float32))
    if np.linalg.norm(gravity_vec) < 1e-6:
        raise ValueError("Gravity vector must be non-zero")

    rng = random.Random(seed)
    remaining_mask = np.ones(pts.shape[0], dtype=bool)
    planes: List[PlaneFitResult] = []

    for plane_index in range(max_planes):
        candidates = pts[remaining_mask]
        if candidates.shape[0] < 3:
            break

        best = _run_msac(
            candidates,
            gravity_vec,
            sample_count=sample_count,
            angle_threshold_rad=math.radians(angle_threshold_deg),
            initial_threshold=initial_threshold,
            mad_scale=mad_scale,
            rng=rng,
        )

        if best is None or best.support < 3:
            break

        # Map inlier mask back to the original array.
        full_inliers = np.zeros_like(remaining_mask)
        full_inliers_indices = np.nonzero(remaining_mask)[0][best.inlier_mask]
        full_inliers[full_inliers_indices] = True

        result = PlaneFitResult(
            plane=best.plane,
            inlier_mask=full_inliers,
            residuals=best.residuals,
            mad=best.mad,
            rms=best.rms,
            support=best.support,
            score=best.score,
            angle_to_gravity_deg=best.angle_to_gravity_deg,
        )
        planes.append(result)

        # Remove inliers when searching for subsequent planes.
        remaining_mask &= ~full_inliers

    # Sort by support (descending), then by RMS (ascending).
    planes.sort(key=lambda res: (-res.support, res.rms))
    return planes


def _run_msac(
    points: np.ndarray,
    gravity: np.ndarray,
    *,
    sample_count: int,
    angle_threshold_rad: float,
    initial_threshold: float,
    mad_scale: float,
    rng: random.Random,
) -> Optional[PlaneFitResult]:
    """Perform MSAC on a set of candidate points."""

    best_score = float("inf")
    best_model: Optional[PlaneModel] = None
    best_inliers: Optional[np.ndarray] = None
    best_residuals: Optional[np.ndarray] = None
    best_mad = 0.0
    best_rms = 0.0
    best_angle_deg = 0.0

# We'll evaluate initial residual threshold squared once to avoid recomputation.
    threshold_sq = initial_threshold ** 2

    total_points = points.shape[0]
    idx_range = list(range(total_points))

    for _ in range(sample_count):
        sample_indices = rng.sample(idx_range, 3)
        p0, p1, p2 = points[sample_indices]
        model = _plane_from_points(p0, p1, p2)
        if model is None:
            continue
        dot = float(np.dot(model.normal, gravity))
        if dot < 0.0:
            model = PlaneModel(normal=-model.normal, offset=-model.offset)

        angle = model.angle_to(gravity)
        # For horizontal surfaces we expect normals roughly aligned with gravity
        # (either direction). Use the smaller angle to either +/- gravity.
        angle = min(angle, math.pi - angle)
        if angle > angle_threshold_rad:
            continue

        residuals_full = np.abs(model.distance(points))

        tau = initial_threshold
        if tau <= 0.0:
            percentile = float(np.percentile(residuals_full, 30)) if residuals_full.size else 0.0
            tau = max(percentile, 1e-3)
        threshold_sq = tau ** 2

        # MSAC score truncates residuals at tau.
        clipped = np.minimum(residuals_full ** 2, threshold_sq)
        score = float(np.sum(clipped))
        if not math.isfinite(score):
            continue

        # Count provisional inliers using the same initial threshold.
        provisional_inliers = residuals_full <= tau
        support = int(np.count_nonzero(provisional_inliers))
        if support < 3:
            continue

        if score < best_score:
            best_score = score
            best_model = model

            inlier_residuals = residuals_full[provisional_inliers]
            mad = _mad(inlier_residuals)
            sigma = 1.4826 * mad
            adaptive_tau = max(tau * 0.5, mad_scale_factor(sigma, mad_scale))

            refined_inliers = residuals_full <= adaptive_tau
            refined_residuals = residuals_full[refined_inliers]
            if refined_residuals.size == 0:
                continue
            best_inliers = refined_inliers
            best_residuals = refined_residuals
            best_mad = _mad(refined_residuals)
            best_rms = float(np.sqrt(np.mean(refined_residuals ** 2)))
            best_angle_deg = math.degrees(angle)

    if best_model is None or best_inliers is None or best_residuals is None:
        return None

    return PlaneFitResult(
        plane=best_model,
        inlier_mask=best_inliers,
        residuals=best_residuals,
        mad=best_mad,
        rms=best_rms,
        support=int(best_inliers.sum()),
        score=best_score,
        angle_to_gravity_deg=best_angle_deg,
    )


def _plane_from_points(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> Optional[PlaneModel]:
    """Construct a plane from three points (returns None if degenerate)."""

    v1 = p1 - p0
    v2 = p2 - p0
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-6:
        return None
    normal /= norm
    offset = -float(np.dot(normal, p0))
    return PlaneModel(normal=normal.astype(np.float32), offset=offset)


def _normalise(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return vec.copy()
    return vec / norm


def _mad(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    median = float(np.median(values))
    deviations = np.abs(values - median)
    return float(np.median(deviations))


def mad_scale_factor(sigma: float, scale: float) -> float:
    """Helper returning an adaptive threshold from a sigma estimate."""
    if not math.isfinite(sigma) or sigma <= 0.0:
        return 0.01
    return float(max(0.004, scale * sigma))


__all__ = ["PlaneModel", "PlaneFitResult", "fit_horizontal_planes"]
