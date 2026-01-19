"""High-level cliff detection pipeline built on DepthAnything primitives."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, replace
from typing import Optional, Sequence

import numpy as np

from .cliff_continuity import (
    GradientScanConfig,
    GradientScanResult,
    compute_depth_statistics,
    detect_cliff_by_gradient_scan,
)
from .cliff_edges import CliffDetectionResult, CliffSegment, detect_cliff_segments
from .depth_anything_provider import DepthAnythingProvider, DepthResult
from .plane_fit import PlaneFitResult, PlaneModel, fit_horizontal_planes
from .pointcloud_builder import CameraIntrinsics, PointCloudResult, build_pointcloud


_LOGGER = logging.getLogger(__name__)


def _binary_dilation(mask: np.ndarray, radius_y: int, radius_x: int) -> np.ndarray:
    """Simple binary dilation with a rectangular kernel."""

    radius_y = max(0, int(radius_y))
    radius_x = max(0, int(radius_x))
    if radius_y == 0 and radius_x == 0:
        return mask.copy()

    padded = np.pad(mask, ((radius_y, radius_y), (radius_x, radius_x)), mode="constant", constant_values=False)
    height, width = mask.shape
    result = np.zeros_like(mask, dtype=bool)
    for dy in range(0, 2 * radius_y + 1):
        row_slice = slice(dy, dy + height)
        for dx in range(0, 2 * radius_x + 1):
            col_slice = slice(dx, dx + width)
            result |= padded[row_slice, col_slice]
    return result


def _binary_erosion(mask: np.ndarray, radius_y: int, radius_x: int) -> np.ndarray:
    """Simple binary erosion with a rectangular kernel."""

    radius_y = max(0, int(radius_y))
    radius_x = max(0, int(radius_x))
    if radius_y == 0 and radius_x == 0:
        return mask.copy()

    padded = np.pad(mask, ((radius_y, radius_y), (radius_x, radius_x)), mode="constant", constant_values=False)
    height, width = mask.shape
    result = np.ones_like(mask, dtype=bool)
    for dy in range(0, 2 * radius_y + 1):
        row_slice = slice(dy, dy + height)
        for dx in range(0, 2 * radius_x + 1):
            col_slice = slice(dx, dx + width)
            result &= padded[row_slice, col_slice]
    return result


def _binary_closing(mask: np.ndarray, kernel: int) -> np.ndarray:
    """Binary closing (dilation+erosion) with an odd kernel size."""

    kernel = max(0, int(kernel))
    if kernel <= 1:
        return mask.copy()
    radius = kernel // 2
    dilated = _binary_dilation(mask, radius, radius)
    return _binary_erosion(dilated, radius, radius)


def _binary_closing_rect(mask: np.ndarray, radius_y: int, radius_x: int) -> np.ndarray:
    """Binary closing with independent radii."""

    radius_y = max(0, int(radius_y))
    radius_x = max(0, int(radius_x))
    if radius_y == 0 and radius_x == 0:
        return mask.copy()
    dilated = _binary_dilation(mask, radius_y, radius_x)
    return _binary_erosion(dilated, radius_y, radius_x)


def _largest_component(mask: np.ndarray) -> np.ndarray:
    """Keep only the largest connected component (4-neighborhood)."""

    if not mask.any():
        return mask.copy()

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best_size = 0
    best_indices: list[tuple[int, int]] = []

    for y, x in zip(*np.where(mask)):
        if visited[y, x]:
            continue
        stack = [(y, x)]
        visited[y, x] = True
        current: list[tuple[int, int]] = []
        while stack:
            cy, cx = stack.pop()
            current.append((cy, cx))
            for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx] and mask[ny, nx]:
                    visited[ny, nx] = True
                    stack.append((ny, nx))
        if len(current) > best_size:
            best_size = len(current)
            best_indices = current

    result = np.zeros_like(mask, dtype=bool)
    if best_indices:
        ys, xs = zip(*best_indices)
        result[ys, xs] = True
    return result


def _bottom_connected_component(mask: np.ndarray, slack: int = 0) -> np.ndarray:
    """Return the subset of the mask connected to the bottom rows (±slack)."""

    if not mask.any():
        return mask.copy()

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    queue: list[tuple[int, int]] = []

    slack = max(0, int(slack))
    start_row = max(0, height - 1 - slack)
    bottom_rows = mask[start_row:, :]
    rows, cols = np.where(bottom_rows)
    for offset_row, col in zip(rows, cols):
        row = start_row + offset_row
        visited[row, col] = True
        queue.append((row, col))

    if not queue:
        return np.zeros_like(mask, dtype=bool)

    while queue:
        y, x = queue.pop()
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx] and mask[ny, nx]:
                visited[ny, nx] = True
                queue.append((ny, nx))

    return visited


@dataclass(frozen=True)
class DetectorConfig:
    """Configuration values controlling cliff detection heuristics."""

    min_plane_support: int = 250
    plane_support_floor: int = 300
    # DISABLED (2025-01-13): Plane-based cliff detection disabled in favor of continuity-based approach
    # The plane fitting method consistently failed (fit to walls instead of ground/table)
    # See CLAUDE.md for detailed analysis. Set to True to re-enable for testing.
    use_plane: bool = False
    plane_roi_top: float = 0.0
    min_confidence: float = 0.1
    max_plane_angle_deg: float = 25.0
    min_segment_length_px: float = 40.0
    max_planes: int = 2
    msac_samples: int = 2048
    msac_seed: int = 0
    invalid_fraction_for_gradient: float = 0.3
    gradient_rows: int = 0  # Legacy pixel-based ROI depth; use gradient_rows_frac when zero
    # UPDATED (Phase 5): Increased from 0.45 to 0.65 to support distant cliff detection
    gradient_rows_frac: float = 0.65
    gradient_min_rows: int = 32
    gradient_min_columns: int = 6
    gradient_relative_ratio: float = 0.55  # Relative jump (fractional increase) for scale-invariant gradient test
    gradient_min_score: float = 0.45  # Fraction of columns within a segment that must exceed threshold
    gradient_morph_kernel: int = 3
    gradient_blur_kernel: int = 3
    gradient_window_size: int = 5
    gradient_max_gap: int = 6
    gradient_bridge_gap: int = 4
    gradient_bridge_valid_ratio: float = 0.35
    gradient_bridge_rows: int = 24
    occlusion_min_invalid_run: int = 4
    occlusion_near_quantile: float = 0.3
    occlusion_border_px: int = 4
    plane_gap_ratio: float = 0.10
    plane_gap_abs: float = 0.0
    plane_min_alignment: float = 0.90
    plane_bottom_coverage_min: float = 0.10
    plane_score_threshold: float = 0.4
    plane_max_candidates: int = 8
    plane_use_horizon_for_roi: bool = True
    roi_plane_margin_px: int = 20
    plane_bottom_region_frac: float = 0.15
    plane_near_region_frac: float = 0.5
    plane_median_ratio_min: float = 0.3
    plane_median_ratio_max: float = 4.5
    plane_p95_ratio_max: float = 6.0
    plane_gap_fraction_max: float = 1.0
    plane_min_ratio_pixels: int = 500
    plane_gap_fuse_ratio: float = 0.95
    plane_scale_min: float = 0.25
    plane_scale_max: float = 8.0
    plane_bias_abs_max_frac: float = 0.3
    plane_mape_max: float = 2.8
    plane_outlier_fraction_max: float = 0.95
    plane_consistency_min_samples: int = 200
    plane_match_error_thresh: float = 0.35
    plane_match_morph_kernel: int = 5
    plane_boundary_band_px: int = 24
    plane_boundary_lateral_px: int = 4
    plane_band_min_hit_ratio: float = 0.01
    plane_band_max_ratio: float = 0.15
    plane_band_min_area_ratio: float = 0.01
    plane_gap_weak_weight: float = 0.35
    plane_table_area_min: float = 0.004
    plane_table_thin_ratio: float = 0.004
    plane_align_band_px: int = 12
    plane_align_percentile: float = 0.9
    plane_align_fallback_rows: int = 30
    plane_inlier_ratio_tau: float = 0.5
    plane_inlier_abs_tau: float = 0.0
    plane_bottom_touch_delta: int = 4
    plane_table_fallback_rows: int = 24
    plane_msac_bottom_frac: float = 0.6
    plane_support_dilate_px: int = 2
    plane_fit_depth_max: float = 0.0

    # Gradient-scan detection parameters (NEW - 2025-01-13, Rewrite v2 - Hysteresis)
    use_gradient_scan: bool = True  # Enable gradient-scan cliff detection
    gradient_scan_strong_threshold: float = 1.0  # Strong edge threshold (definite cliff, e.g. 8→3)
    gradient_scan_weak_threshold: float = 0.2  # Weak edge threshold (possible cliff, e.g. 2→1) - LOWERED
    gradient_scan_weak_morph_closing: int = 3  # Morphological closing on weak edges (fills gaps)
    gradient_scan_gap_tolerance: int = 2  # Gap tolerance for hysteresis linking (can jump 2px gaps)
    gradient_scan_table_rows: int = 30  # Rows above cliff to check for table context
    gradient_scan_above_valid: float = 0.6  # Min valid ratio in above-region
    # UPDATED (Phase 5): Lowered from 0.5 to 0.25 for distant/weak gradient detection
    gradient_scan_depth_drop: float = 0.25  # Min depth drop (above - below)
    # UPDATED (Phase 5): Lowered from 0.4 to 0.15 to support distant cliff detection (20-30cm)
    gradient_scan_position_min: float = 0.15  # Cliff must be in bottom 85% of image
    gradient_scan_min_contour_length: int = 15  # Minimum contour length (pixels) - lowered to handle fragmented edges
    gradient_scan_min_horizontal_span: float = 0.03  # Min horizontal span (fraction of width) - lowered for fragmented edges
    gradient_scan_smoothing: int = 5  # Boundary smoothing window

    def gradient_roi_bounds(self, image_height: int, horizon_row: Optional[int] = None) -> tuple[int, int]:
        """Return `(roi_start, roi_rows)` for the gradient ROI."""

        if image_height <= 0:
            return 0, 0

        if self.gradient_rows > 0:
            roi_rows = int(self.gradient_rows)
        else:
            frac = float(np.clip(self.gradient_rows_frac, 0.0, 1.0))
            roi_rows = int(round(frac * image_height))

        min_rows = max(2, int(self.gradient_min_rows))
        roi_rows = int(np.clip(roi_rows, min_rows, image_height))
        roi_start = max(0, image_height - roi_rows)

        if horizon_row is not None and self.plane_use_horizon_for_roi:
            # Only use horizon to tighten ROI if it's very high in the image (sky/ceiling scenario)
            # For downward-facing planes (table tops), horizon is the near edge, not far edge
            high_horizon_threshold = int(image_height * 0.3)  # Top 30% of image
            if horizon_row < high_horizon_threshold:
                margin = max(0, int(self.roi_plane_margin_px))
                adjusted_start = int(np.clip(horizon_row + margin, 0, image_height - min_rows))
                roi_start = min(roi_start, adjusted_start)
                roi_rows = max(min_rows, image_height - roi_start)

        return roi_start, roi_rows

    def gradient_roi_rows(self, image_height: int) -> int:
        """Backward-compatible helper returning just the ROI height."""

        _, roi_rows = self.gradient_roi_bounds(image_height)
        return roi_rows


@dataclass(frozen=True)
class CliffDetectorOutput:
    """Aggregated output for a single RGB frame."""

    depth: DepthResult
    pointcloud: Optional[PointCloudResult]
    plane: Optional[PlaneFitResult]
    plane_debug: Optional["PlaneDebugInfo"]
    cliff: Optional[CliffDetectionResult]
    segments: Sequence[CliffSegment]
    status: str

    @property
    def has_confident_segment(self) -> bool:
        return any(seg.confidence > 0.0 for seg in self.segments)


@dataclass(frozen=True)
class PlaneEvidence:
    """Validated plane information used purely as geometric support."""

    result: PlaneFitResult
    depth_map: np.ndarray
    score: float
    alignment: float
    bottom_coverage: float
    far_score: float
    support_ratio: float
    horizon_row: Optional[int]
    raw_ratio: Optional[float]
    support_mask: Optional[np.ndarray]


@dataclass(frozen=True)
class PlaneDebugInfo:
    """Lightweight summary of the validated plane for visualization/logging."""

    score: float
    alignment: float
    bottom_coverage: float
    far_score: float
    support_ratio: float
    horizon_row: Optional[int]
    angle_to_gravity_deg: Optional[float]
    consistency_ok: bool
    consistency_samples: Optional[int]
    scale: Optional[float]
    bias: Optional[float]
    mape: Optional[float]
    outlier_fraction: Optional[float]
    median_ratio: Optional[float]
    p95_ratio: Optional[float]
    gap_fraction: Optional[float]
    raw_ratio: Optional[float]


@dataclass(frozen=True)
class _RatioStats:
    median: float
    p95: float
    gap_fraction: float


class CliffDetector:
    """Pipeline orchestrating depth inference, plane fitting, and edge tests."""

    def __init__(
        self,
        *,
        depth_provider: DepthAnythingProvider,
        intrinsics: CameraIntrinsics,
        gravity_camera: np.ndarray,
        extrinsics: Optional[np.ndarray] = None,
        config: Optional[DetectorConfig] = None,
    ) -> None:
        self._depth_provider = depth_provider
        self._intrinsics = intrinsics
        self._gravity_cam = np.asarray(gravity_camera, dtype=np.float32)
        if self._gravity_cam.shape != (3,):
            raise ValueError("gravity_camera must be a 3-element vector")
        gravity_norm = np.linalg.norm(self._gravity_cam)
        if gravity_norm < 1e-6:
            raise ValueError("gravity_camera must be non-zero")
        self._gravity_cam = self._gravity_cam / gravity_norm
        self._extrinsics = np.asarray(extrinsics, dtype=np.float32) if extrinsics is not None else None
        if self._extrinsics is not None and self._extrinsics.shape != (4, 4):
            raise ValueError("extrinsics must be 4x4 when provided")
        self._config = config or DetectorConfig()
        self._ray_directions = self._precompute_rays()

    @property
    def intrinsics(self) -> CameraIntrinsics:
        return self._intrinsics

    @property
    def extrinsics(self) -> Optional[np.ndarray]:
        return self._extrinsics

    def process(self, rgb_image: np.ndarray) -> CliffDetectorOutput:
        """Run the detection stack on an RGB frame."""

        depth_result = self._depth_provider.infer(rgb_image)
        depth_for_edges = depth_result.depth.astype(np.float32, copy=False)
        scale_hint = float(depth_result.scale_hint)
        if np.isfinite(scale_hint) and scale_hint > 0.0:
            depth_for_edges = depth_for_edges * scale_hint

        valid_mask_edges = depth_result.valid_mask.astype(bool)
        if depth_result.confidence is not None and self._config.min_confidence > 0.0:
            valid_mask_edges &= depth_result.confidence >= float(self._config.min_confidence)

        pointcloud: Optional[PointCloudResult] = None
        plane_evidence: Optional[PlaneEvidence] = None
        if self._config.use_plane:
            pointcloud = self._build_pointcloud(depth_result)
            roi_top_px = int(
                np.clip(self._config.plane_roi_top * self._intrinsics.height, 0, self._intrinsics.height - 1)
            ) if self._config.plane_roi_top > 0.0 else 0
            min_support = self._min_plane_support_threshold(self._intrinsics.width, self._intrinsics.height)
            _LOGGER.debug(
                "plane setup: use_plane=%s roi_top_px=%d min_conf=%.2f pointcloud=%d min_support=%d "
                "msac_samples=%d max_planes=%d max_angle=%.1f",
                self._config.use_plane,
                roi_top_px,
                self._config.min_confidence,
                pointcloud.count if pointcloud is not None else 0,
                min_support,
                self._config.msac_samples,
                self._config.max_planes,
                self._config.max_plane_angle_deg,
            )
            plane_evidence = self._select_plane_evidence(
                pointcloud,
                depth_for_edges,
                valid_mask_edges,
            )

        plane_result: Optional[PlaneFitResult] = None
        plane_depth_map: Optional[np.ndarray] = plane_evidence.depth_map if plane_evidence is not None else None
        plane_table_mask: Optional[np.ndarray] = None
        plane_debug: Optional[PlaneDebugInfo] = None
        horizon_row: Optional[int] = None
        plane_ok = False
        if plane_evidence is not None:
            plane_ok, aligned_plane, plane_debug, plane_table_mask = self._apply_plane_consistency_gate(
                plane_evidence,
                depth_for_edges,
                depth_result.valid_mask,
            )
            if aligned_plane is not None:
                plane_depth_map = aligned_plane
            if plane_ok:
                plane_result = plane_evidence.result
                horizon_row = plane_evidence.horizon_row

        # ============================================================================
        # GRADIENT-SCAN DETECTION (NEW - 2025-01-13, Rewrite)
        # ============================================================================
        # Detect cliff edges using direct gradient analysis with context validation
        gradient_scan_result: Optional[GradientScanResult] = None

        if self._config.use_gradient_scan:
            # Build gradient scan config from detector config
            grad_config = GradientScanConfig(
                gradient_strong_threshold=self._config.gradient_scan_strong_threshold,
                gradient_weak_threshold=self._config.gradient_scan_weak_threshold,
                weak_edge_morph_closing=self._config.gradient_scan_weak_morph_closing,
                hysteresis_gap_tolerance=self._config.gradient_scan_gap_tolerance,
                table_context_rows=self._config.gradient_scan_table_rows,
                above_valid_ratio_min=self._config.gradient_scan_above_valid,
                depth_drop_min=self._config.gradient_scan_depth_drop,
                cliff_position_min=self._config.gradient_scan_position_min,
                min_contour_length=self._config.gradient_scan_min_contour_length,
                min_horizontal_span=self._config.gradient_scan_min_horizontal_span,
                boundary_smoothing_window=self._config.gradient_scan_smoothing,
            )

            # Run gradient-scan detection
            # TODO: Add debug_output_path support if needed for gradient visualization
            gradient_scan_result = detect_cliff_by_gradient_scan(
                depth=depth_for_edges,
                valid_mask=valid_mask_edges,
                config=grad_config,
                debug_output_path=None,  # Set to path string to save gradient visualization
            )

        cliff_result = detect_cliff_segments(
            depth_map=depth_for_edges,
            valid_mask=valid_mask_edges,
            config=self._config,
            plane_depth_map=plane_depth_map,
            plane_ok=plane_ok,
            horizon_row=horizon_row,
            plane_table_mask=plane_table_mask,
        )

        # Merge gradient-scan and fallback gradient-based segments
        segments: Sequence[CliffSegment] = []
        status = "none"

        if gradient_scan_result is not None and gradient_scan_result.detected:
            # PHASE 5: Process ALL detected segments (not just best)
            if gradient_scan_result.all_segments:
                # Use all_segments (multiple cliff fragments)
                grad_segments = []
                depth_means = []  # PHASE 6A: Collect depth info for filtering

                for boundary_coords, conf, reason in gradient_scan_result.all_segments:
                    seg = self._boundary_to_cliff_segment(
                        boundary_coords,
                        depth_for_edges,
                        confidence=conf,
                        valid_mask=valid_mask_edges,  # PHASE 5: For depth stats
                    )
                    if seg is not None:
                        grad_segments.append(seg)
                        depth_means.append(seg.mean_depth)  # PHASE 6A: Store depth

                if grad_segments:
                    # PHASE 6A: Depth consistency filtering (reduce false positives)
                    from .cliff_continuity import match_segments_by_depth_consistency

                    # Construct segment tuples for matching function
                    segment_tuples = [
                        (seg.polyline_px, seg.confidence, "")
                        for seg in grad_segments
                    ]

                    # Group segments by depth similarity
                    groups = match_segments_by_depth_consistency(
                        segment_tuples,
                        depth_means,
                        depth_similarity_threshold=0.15  # 15% tolerance
                    )

                    # Select best group (highest average confidence)
                    if groups:
                        best_group_idx = max(
                            range(len(groups)),
                            key=lambda i: np.mean([grad_segments[j].confidence for j in groups[i]])
                        )
                        segments = [grad_segments[j] for j in groups[best_group_idx]]

                        _LOGGER.debug(
                            "gradient_scan: depth filter kept %d/%d segments (best group from %d groups, avg_conf=%.2f)",
                            len(segments),
                            len(grad_segments),
                            len(groups),
                            np.mean([s.confidence for s in segments])
                        )
                    else:
                        # Fallback: if grouping fails (no valid depths), keep all segments
                        segments = grad_segments
                        _LOGGER.warning(
                            "gradient_scan: depth filtering skipped (no valid depths), keeping all %d segments",
                            len(grad_segments)
                        )

                    status = "gradient_scan"
                    _LOGGER.debug(
                        "gradient_scan: %d cliff segment(s) after filtering (strong=%d, weak=%d, linked=%d, contours=%d, best_conf=%.2f)",
                        len(segments),
                        gradient_scan_result.num_strong_edges,
                        gradient_scan_result.num_weak_edges,
                        gradient_scan_result.num_linked_edges,
                        gradient_scan_result.num_contours,
                        max((s.confidence for s in segments), default=0.0),
                    )
            else:
                # Fallback: use single best segment (backward compatibility)
                grad_segment = self._boundary_to_cliff_segment(
                    gradient_scan_result.boundary_coords,
                    depth_for_edges,
                    confidence=gradient_scan_result.confidence,
                    valid_mask=valid_mask_edges,  # PHASE 5: For depth stats
                )
                if grad_segment is not None:
                    segments = [grad_segment]
                    status = "gradient_scan"
                    _LOGGER.debug(
                        "gradient_scan: cliff segment created (length=%d px, conf=%.2f)",
                        gradient_scan_result.selected_contour_length,
                        gradient_scan_result.confidence,
                    )

        # Fallback to old gradient-based segments if gradient-scan didn't detect
        if not segments and cliff_result.segments:
            segments = cliff_result.segments
            status = "gradient_legacy"

        # Keep original status logic for no_depth cases
        if not segments:
            roi_start, roi_rows = self._config.gradient_roi_bounds(
                depth_result.valid_mask.shape[0],
                horizon_row if plane_ok else None,
            )
            roi_mask = depth_result.valid_mask[roi_start:, :] if roi_rows > 0 else depth_result.valid_mask
            valid_fraction = float(roi_mask.mean()) if roi_mask.size else 0.0
            invalid_fraction = 1.0 - valid_fraction
            threshold = float(np.clip(self._config.invalid_fraction_for_gradient, 0.0, 1.0))
            if invalid_fraction > threshold:
                status = "no_depth"

        return CliffDetectorOutput(
            depth=depth_result,
            pointcloud=pointcloud,
            plane=plane_result,
            plane_debug=plane_debug,
            cliff=cliff_result,
            segments=segments,
            status=status,
        )

    # ------------------------------------------------------------------
    # Helpers

    def _build_pointcloud(
        self,
        depth_result: DepthResult,
        mask_override: Optional[np.ndarray] = None,
    ) -> PointCloudResult:
        valid_mask = mask_override if mask_override is not None else depth_result.valid_mask.astype(bool)
        confidence = depth_result.confidence

        height = self._intrinsics.height
        width = self._intrinsics.width
        roi = None
        if self._config.plane_roi_top > 0.0:
            roi_top = int(np.clip(self._config.plane_roi_top * height, 0, height - 1))
            roi = (0, roi_top, width, height)

        return build_pointcloud(
            depth_result.depth,
            self._intrinsics,
            depth_scale=depth_result.scale_hint,
            mask=valid_mask,
            confidence=confidence,
            min_confidence=max(0.0, self._config.min_confidence),
            roi=roi,
            extrinsics=self._extrinsics,
        )

    def _min_plane_support_threshold(self, width: int, height: int) -> int:
        """Return the sample count required before running MSAC."""

        frac = int(round(0.005 * width * height))
        base = max(int(self._config.min_plane_support), frac)
        floor = max(1, int(self._config.plane_support_floor))
        return max(base, floor)

    # ============================================================================
    # PLANE-BASED DETECTION (DISABLED - see DetectorConfig.use_plane)
    # ============================================================================
    # The following methods implement plane fitting-based cliff detection.
    # This approach has been disabled (2025-01-13) due to consistent failures:
    # - MSAC fits to walls instead of ground/table surface
    # - Heuristic ground plane assumes fixed geometry that doesn't match DepthAnything's relative depth
    # - False positives on flat tables, false negatives on actual cliffs
    #
    # Code is preserved for potential future fixes or comparison with new approach.
    # See CLAUDE.md for detailed failure analysis and design decisions.
    # ============================================================================

    def _select_plane_evidence(
        self,
        pointcloud: Optional[PointCloudResult],
        depth_map: np.ndarray,
        valid_mask: np.ndarray,
    ) -> Optional[PlaneEvidence]:
        if pointcloud is None:
            return None

        width = int(self._intrinsics.width)
        height = int(self._intrinsics.height)
        min_support = self._min_plane_support_threshold(width, height)
        if pointcloud.count < min_support:
            _LOGGER.debug("plane: insufficient support (%d < %d)", pointcloud.count, min_support)
            return None

        all_indices = np.arange(pointcloud.count)
        msac_indices = all_indices
        msac_points = pointcloud.points_camera
        msac_frac = float(np.clip(self._config.plane_msac_bottom_frac, 0.0, 1.0))
        if 0.0 < msac_frac < 1.0:
            rows_needed = int(round(msac_frac * height))
            bottom_start = max(0, height - rows_needed)
            subset_idx = np.where(pointcloud.pixel_y >= bottom_start)[0]
            if subset_idx.size >= min_support:
                msac_indices = subset_idx
                msac_points = msac_points[msac_indices]
        msac_pixels_x = pointcloud.pixel_x[msac_indices]
        msac_pixels_y = pointcloud.pixel_y[msac_indices]
        msac_total = msac_points.shape[0]
        if msac_total < min_support:
            _LOGGER.debug("plane: insufficient support in msac roi (%d < %d)", msac_total, min_support)
            return None

        candidates = fit_horizontal_planes(
            msac_points,
            gravity=self._gravity_cam,
            sample_count=self._config.msac_samples,
            max_planes=self._config.max_planes,
            angle_threshold_deg=self._config.max_plane_angle_deg,
            seed=self._config.msac_seed,
        )
        candidate_count = len(candidates)
        _LOGGER.debug(
            "plane: MSAC candidates=%d (points=%d min_support=%d samples=%d)",
            candidate_count,
            msac_total,
            min_support,
            self._config.msac_samples,
        )
        if not candidates:
            _LOGGER.debug("plane: no MSAC candidates")
            return None

        total_points = max(1, msac_total)
        max_candidates = max(1, int(self._config.plane_max_candidates))
        best: Optional[PlaneEvidence] = None
        candidate_logs: list[str] = []

        for idx, candidate in enumerate(candidates[:max_candidates]):
            plane_model = candidate.plane
            dot = float(np.dot(plane_model.normal, self._gravity_cam))
            if dot < 0.0:
                plane_model = PlaneModel(normal=-plane_model.normal, offset=-plane_model.offset)
                candidate = replace(candidate, plane=plane_model)
                dot = -dot
            alignment = abs(dot)

            plane_depth = self._project_plane_depth_map(plane_model)
            support_ratio = min(1.0, candidate.support / float(total_points))

            # Build support_mask first, before evaluation
            support_mask = None
            if candidate.inlier_mask is not None and candidate.inlier_mask.shape[0] == msac_total:
                support_idx = np.where(candidate.inlier_mask)[0]
                if support_idx.size:
                    support_mask = np.zeros((height, width), dtype=bool)
                    support_mask[msac_pixels_y[support_idx], msac_pixels_x[support_idx]] = True

            metrics = self._evaluate_plane_candidate(
                plane_depth=plane_depth,
                alignment=alignment,
                support_ratio=support_ratio,
                support_mask=support_mask,
            )
            if metrics is None:
                continue

            score_base, bottom_coverage, far_score, horizon_row = metrics
            ratio_stats = self._compute_ratio_stats(plane_depth, depth_map, valid_mask, support_mask)
            raw_ratio = ratio_stats.median if ratio_stats is not None else None
            penalty = 1.0
            if raw_ratio is not None and math.isfinite(raw_ratio):
                penalty = float(np.exp(-(((raw_ratio - 1.0) / 0.5) ** 2)))
            score = score_base * penalty

            candidate_logs.append(
                "cand#{idx}:sup={sup} align={align:.3f} d={offset:.3f} bottom={bottom:.2f} far={far:.2f} "
                "score={score:.3f} raw_ratio={ratio}".format(
                    idx=idx,
                    sup=candidate.support,
                    align=alignment,
                    offset=candidate.plane.offset,
                    bottom=bottom_coverage,
                    far=far_score,
                    score=score,
                    ratio="NA" if raw_ratio is None else f"{raw_ratio:.2f}",
                )
            )

            if best is None or score > best.score:
                best = PlaneEvidence(
                    result=candidate,
                    depth_map=plane_depth,
                    score=score,
                    alignment=alignment,
                    bottom_coverage=bottom_coverage,
                    far_score=far_score,
                    support_ratio=support_ratio,
                    horizon_row=horizon_row,
                    raw_ratio=raw_ratio,
                    support_mask=support_mask,
                )

        if candidate_logs:
            _LOGGER.debug("plane candidates (%d): %s", len(candidate_logs), " | ".join(candidate_logs))

        # Fallback: if MSAC failed or found low-quality plane, try heuristic ground plane
        use_heuristic = False
        if best is None:
            use_heuristic = True
        elif best.score <= 0.01:  # Very low score, likely wrong plane (e.g., wall)
            _LOGGER.debug("plane: MSAC plane has low score (%.3f), trying heuristic", best.score)
            use_heuristic = True

        if use_heuristic and self._extrinsics is not None:
            heuristic = self._create_heuristic_ground_plane(depth_map, valid_mask, height, width)
            if heuristic is not None:
                # Use heuristic if it's better than MSAC result (or if MSAC failed)
                if best is None or heuristic.score > best.score:
                    _LOGGER.debug("plane: using heuristic ground plane (score=%.3f)", heuristic.score)
                    best = heuristic

        return best

    def _create_heuristic_ground_plane(
        self,
        depth_map: np.ndarray,
        valid_mask: np.ndarray,
        height: int,
        width: int,
    ) -> Optional[PlaneEvidence]:
        """Create a heuristic horizontal ground plane based on camera calibration."""
        from .plane_fit import PlaneModel, PlaneFitResult

        # Extract camera height from extrinsics (z-component of translation)
        camera_height = float(self._extrinsics[2, 3])  # meters

        # Create horizontal ground plane (normal points up in camera frame)
        # The plane equation is: normal · (point - origin) = 0
        # For ground: normal = -gravity_camera (points up), offset = camera_height
        normal = -self._gravity_cam  # Flip gravity to point upward
        offset = -camera_height  # Ground is below camera

        ground_plane = PlaneModel(normal=normal, offset=offset)

        # Project plane to depth map
        plane_depth = self._project_plane_depth_map(ground_plane)

        # Create a synthetic support mask from the projected plane
        # Mark pixels where projection is valid and close to observed depth
        finite_mask = np.isfinite(plane_depth) & (plane_depth > 0.0)
        if not finite_mask.any():
            _LOGGER.debug("plane: heuristic plane has no valid projection")
            return None

        # Compute alignment (should be ~1.0 for horizontal plane)
        alignment = abs(float(np.dot(normal, self._gravity_cam)))

        # Evaluate the heuristic plane using existing logic
        support_mask = finite_mask.copy()  # Use projection as proxy for support
        metrics = self._evaluate_plane_candidate(
            plane_depth=plane_depth,
            alignment=alignment,
            support_ratio=0.5,  # Assume moderate support
            support_mask=support_mask,
        )

        if metrics is None:
            _LOGGER.debug("plane: heuristic plane failed evaluation")
            return None

        score_base, bottom_coverage, far_score, horizon_row = metrics

        # Compute initial ratio stats
        ratio_stats = self._compute_ratio_stats(plane_depth, depth_map, valid_mask, support_mask)
        raw_ratio = ratio_stats.median if ratio_stats is not None else None

        # Apply scale/bias alignment to heuristic plane
        # Since DepthAnything depth is relative, we need to align it
        aligned_plane_depth = plane_depth
        scale = 1.0
        bias = 0.0
        if support_mask.any() and valid_mask.any():
            # Use bottom region for scale/bias fitting
            bottom_rows = max(1, int(height * 0.4))
            fit_region = support_mask[-bottom_rows:, :] & valid_mask[-bottom_rows:, :]
            if fit_region.sum() >= 100:  # Need enough samples
                plane_vals = plane_depth[-bottom_rows:, :][fit_region]
                meas_vals = depth_map[-bottom_rows:, :][fit_region]
                scale, bias = self._fit_plane_scale_bias(plane_vals, meas_vals)
                aligned_plane_depth = plane_depth * scale + bias
                # Recompute ratio stats with aligned plane
                ratio_stats = self._compute_ratio_stats(aligned_plane_depth, depth_map, valid_mask, support_mask)
                raw_ratio = ratio_stats.median if ratio_stats is not None else None

        # Create synthetic PlaneFitResult with dummy values
        support_count = int(support_mask.sum())
        angle_deg = math.degrees(math.acos(float(np.clip(alignment, -1.0, 1.0))))

        plane_result = PlaneFitResult(
            plane=ground_plane,
            inlier_mask=np.zeros(0, dtype=bool),  # Empty mask for heuristic plane
            residuals=np.zeros(0, dtype=np.float32),  # No residuals
            mad=0.0,  # No residuals to compute MAD
            rms=0.0,  # No residuals to compute RMS
            support=support_count,
            score=0.0,  # Heuristic plane has no MSAC score
            angle_to_gravity_deg=angle_deg,
        )

        # Compute score
        penalty = 1.0
        if raw_ratio is not None and math.isfinite(raw_ratio):
            penalty = float(np.exp(-(((raw_ratio - 1.0) / 0.5) ** 2)))
        score = score_base * penalty

        _LOGGER.debug(
            "plane: heuristic ground plane: height=%.3fm align=%.3f bottom=%.2f scale=%.2f score=%.3f ratio=%s",
            camera_height, alignment, bottom_coverage, scale, score,
            "NA" if raw_ratio is None else f"{raw_ratio:.2f}"
        )

        return PlaneEvidence(
            result=plane_result,
            depth_map=aligned_plane_depth,
            score=score,
            alignment=alignment,
            bottom_coverage=bottom_coverage,
            far_score=far_score,
            support_ratio=0.5,
            horizon_row=horizon_row,
            raw_ratio=raw_ratio,
            support_mask=support_mask,
        )

    def _evaluate_plane_candidate(
        self,
        *,
        plane_depth: np.ndarray,
        alignment: float,
        support_ratio: float,
        support_mask: Optional[np.ndarray] = None,
    ) -> Optional[tuple[float, float, float, Optional[int]]]:
        height, width = plane_depth.shape
        finite_mask = np.isfinite(plane_depth) & (plane_depth > 0.0)
        if not finite_mask.any():
            _LOGGER.debug("plane: reject candidate (no finite projection)")
            return None

        horizon_rows = np.where(np.any(finite_mask, axis=1))[0]
        if horizon_rows.size == 0:
            _LOGGER.debug("plane: reject candidate (no horizon rows)")
            return None
        horizon_row = int(horizon_rows[0])

        foot_depth = plane_depth[-1, width // 2]
        if not (np.isfinite(foot_depth) and foot_depth > 0.0):
            bottom_row = plane_depth[-1, :]
            finite_bottom = bottom_row[np.isfinite(bottom_row) & (bottom_row > 0.0)]
            if finite_bottom.size:
                foot_depth = float(np.median(finite_bottom))
            else:
                foot_depth = float("inf")
                _LOGGER.debug("plane: candidate bottom depth unavailable")

        # Use support_mask for bottom coverage if available (more robust than projection)
        bottom_frac = float(np.clip(self._config.plane_bottom_region_frac, 0.05, 1.0))
        bottom_rows = max(1, int(round(bottom_frac * height)))
        if support_mask is not None and support_mask.shape == (height, width):
            bottom_slice = support_mask[-bottom_rows:, :]
            # Additional check: ensure support exists in the very bottom rows (bottom 5%)
            very_bottom_rows = max(1, int(height * 0.05))
            very_bottom_slice = support_mask[-very_bottom_rows:, :]
            if not very_bottom_slice.any():
                _LOGGER.debug("plane: candidate has no support in very bottom rows")
        else:
            bottom_slice = finite_mask[-bottom_rows:, :]
        bottom_coverage = float(np.mean(bottom_slice)) if bottom_slice.size else 0.0
        bottom_thresh = float(np.clip(self._config.plane_bottom_coverage_min, 0.0, 1.0))
        bottom_penalty = 1.0
        if bottom_thresh > 0.0:
            bottom_penalty = float(np.clip(bottom_coverage / max(bottom_thresh, 1e-3), 0.0, 1.0))

        # Use support_mask for near-field check if available
        near_frac = float(np.clip(self._config.plane_near_region_frac, 0.05, 1.0))
        near_rows = max(1, int(round(near_frac * height)))
        near_start = max(0, height - near_rows)
        if support_mask is not None and support_mask.shape == (height, width):
            near_slice_mask = support_mask[near_start:, :]
        else:
            near_slice = plane_depth[near_start:, :]
            near_slice_mask = np.isfinite(near_slice) & (near_slice > 0.0)
        if not near_slice_mask.any():
            _LOGGER.debug("plane: candidate lacks near-field support")
        near_fraction = float(np.mean(near_slice_mask)) if near_slice_mask.size else 0.0

        # For far_score, still use plane_depth
        near_slice = plane_depth[near_start:, :]
        near_mask = np.isfinite(near_slice) & (near_slice > 0.0)

        median_depth = float(np.median(near_slice[near_mask])) if near_mask.any() else float("inf")
        far_score = float(1.0 - np.exp(-median_depth)) if np.isfinite(median_depth) and median_depth > 0.0 else 1.0

        score = (
            0.50 * alignment
            + 3.0 * bottom_coverage
            + 1.0 * near_fraction
            - 1.0 * far_score
            + 0.50 * support_ratio
        )
        score = max(score, 0.0) * bottom_penalty
        return score, bottom_coverage, far_score, horizon_row

    def _compute_ratio_stats(
        self,
        plane_depth: np.ndarray,
        depth_map: np.ndarray,
        valid_mask: np.ndarray,
        support_mask: Optional[np.ndarray] = None,
    ) -> Optional[_RatioStats]:
        mask = valid_mask & np.isfinite(plane_depth) & (plane_depth > 0.0)
        depth_cap = float(getattr(self._config, "plane_fit_depth_max", 0.0))
        if depth_cap > 0.0:
            mask &= plane_depth <= depth_cap
        if support_mask is not None and support_mask.shape == mask.shape:
            support_window = support_mask & mask
            if support_window.any():
                mask = support_window
        if not mask.any():
            return None

        ratios = depth_map[mask] / np.maximum(plane_depth[mask], 1e-6)
        if ratios.size == 0:
            return None

        gap_threshold = 1.0 + max(0.0, self._config.plane_gap_ratio)
        return _RatioStats(
            median=float(np.median(ratios)),
            p95=float(np.percentile(ratios, 95)),
            gap_fraction=float(np.mean(ratios >= gap_threshold)),
        )

    def _apply_plane_consistency_gate(
        self,
        plane: PlaneEvidence,
        depth_map: np.ndarray,
        valid_mask: np.ndarray,
    ) -> tuple[bool, Optional[np.ndarray], PlaneDebugInfo, Optional[np.ndarray]]:
        config = self._config
        height, _ = depth_map.shape
        roi_start, _ = config.gradient_roi_bounds(height, None)
        depth_roi = depth_map[roi_start:]
        valid_roi = valid_mask[roi_start:]
        plane_roi = plane.depth_map[roi_start:]

        min_samples = max(1, int(config.plane_consistency_min_samples))
        align_rows = max(1, int(round(config.plane_near_region_frac * depth_roi.shape[0])))
        region_mask = np.zeros_like(valid_roi, dtype=bool)
        region_mask[-align_rows:, :] = True

        support_roi = None
        if plane.support_mask is not None and plane.support_mask.shape == depth_map.shape:
            support_roi = plane.support_mask[roi_start:]

        # Phase 1: Build fit_mask for scale/bias estimation using support inliers
        # (not constrained by depth_cap to avoid circular dependency)
        fit_mask = valid_roi & np.isfinite(plane_roi) & (plane_roi > 0.0)
        if support_roi is not None and support_roi.any():
            # Prefer MSAC inliers (already vetted as plane points)
            support_band = support_roi
            dilate_px = max(0, int(getattr(config, "plane_support_dilate_px", 0)))
            if dilate_px > 0:
                support_band = _binary_dilation(support_band, dilate_px, dilate_px)
            fit_mask &= support_band
        else:
            # Fallback to near-field region if no support mask
            fit_mask &= region_mask

        aligned_plane = None
        aligned_plane_full = plane.depth_map
        scale = None
        bias = None
        med_ratio = None
        p95_ratio = None
        gap_fraction = None
        mape = None
        outlier_fraction = None
        consistency_ok = False
        sample_count = 0
        table_mask: Optional[np.ndarray] = None

        fit_count = int(fit_mask.sum())
        _LOGGER.debug(
            "plane consistency fit_mask: support=%d region=%d final=%d depth_cap=%.2f",
            support_roi.sum() if support_roi is not None else 0,
            region_mask.sum(),
            fit_count,
            float(getattr(config, "plane_fit_depth_max", 0.0))
        )
        if fit_count >= 2:
            plane_vals = plane_roi[fit_mask]
            meas_vals = depth_roi[fit_mask]
            scale, bias = self._fit_plane_scale_bias(plane_vals, meas_vals)
            aligned_plane_full = plane.depth_map * scale + bias
        else:
            _LOGGER.debug("plane consistency: insufficient samples (%d < 2) for scale/bias fit", fit_count)

        # Phase 2: Use aligned plane for consistency verification
        aligned_roi = aligned_plane_full[roi_start:]
        align_mask = valid_roi & np.isfinite(aligned_roi) & (aligned_roi > 0.0) & fit_mask

        table_seed = np.zeros_like(align_mask, dtype=bool)
        if align_mask.any():
            ratio_tau = float(np.clip(config.plane_inlier_ratio_tau, 0.0, 1.0))
            ratio_low = max(0.0, 1.0 - ratio_tau)
            ratio_high = 1.0 + ratio_tau
            abs_tau = max(0.0, float(config.plane_inlier_abs_tau))
            denom = np.maximum(np.abs(aligned_roi), 1e-6)
            ratio_vals = np.zeros_like(depth_roi, dtype=np.float32)
            ratio_vals[align_mask] = depth_roi[align_mask] / np.maximum(aligned_roi[align_mask], 1e-6)
            ratio_mask = (ratio_vals >= ratio_low) & (ratio_vals <= ratio_high)
            abs_mask = np.abs(depth_roi - aligned_roi) <= abs_tau
            table_seed = align_mask & (ratio_mask | abs_mask)
            if table_seed.any():
                table_seed = _binary_closing_rect(table_seed, 1, 4)
                table_seed = _largest_component(table_seed)
                rows = np.where(table_seed)[0]
                if rows.size:
                    centroid = float(np.mean(rows))
                    if centroid < 0.4 * float(align_mask.shape[0]):
                        table_seed = _bottom_connected_component(table_seed, slack=config.plane_bottom_touch_delta)
                else:
                    table_seed = _bottom_connected_component(table_seed, slack=config.plane_bottom_touch_delta)

        roi_rows = depth_roi.shape[0]
        roi_area = max(1, align_mask.size)
        table_pixels = int(table_seed.sum())
        table_area_ratio = float(table_pixels / roi_area)
        min_table_ratio = float(np.clip(config.plane_table_area_min, 0.0, 1.0))
        if table_area_ratio < min_table_ratio or not table_seed.any():
            fallback_rows = max(1, min(int(config.plane_table_fallback_rows), roi_rows))
            table_seed = np.zeros_like(table_seed, dtype=bool)
            table_seed[-fallback_rows:, :] = True
            table_pixels = int(table_seed.sum())
            table_area_ratio = float(table_pixels / roi_area)

        table_rows = np.where(table_seed)[0]
        align_band = np.zeros_like(align_mask, dtype=bool)
        if table_rows.size > 0:
            percentile = float(np.clip(config.plane_align_percentile, 0.5, 0.98))
            target_row = int(np.clip(np.percentile(table_rows, percentile * 100.0), 0, max(roi_rows - 1, 0)))
            band_half = max(1, int(config.plane_align_band_px))
            row_start = max(0, target_row - band_half)
            row_end = min(roi_rows, target_row + band_half + 1)
            align_band[row_start:row_end, :] = True
        if not align_band.any() or table_area_ratio < float(np.clip(config.plane_table_thin_ratio, 0.0, 1.0)):
            fallback_limit = max(2, int(round(0.08 * roi_rows))) if roi_rows > 0 else 2
            fallback_rows = max(2, min(int(config.plane_align_fallback_rows), fallback_limit))
            fallback_rows = min(fallback_rows, roi_rows)
            align_band = np.zeros_like(align_mask, dtype=bool)
            if fallback_rows > 0:
                align_band[-fallback_rows:, :] = True
        align_mask &= align_band
        if table_seed.any():
            table_seed &= align_band
        else:
            table_seed = align_band.copy()

        if align_mask.sum() < min_samples:
            align_mask = fit_mask.copy()
            table_seed = align_mask.copy()

        table_mask = np.zeros_like(valid_mask, dtype=bool)
        table_roi = table_mask[roi_start:]
        table_roi[:] = False
        table_roi[table_seed] = True
        table_area_ratio = float(table_roi.mean()) if table_roi.size else 0.0

        if align_mask.sum() >= min_samples:
            error_map = np.zeros_like(depth_roi, dtype=np.float32)
            denom = np.maximum(np.abs(aligned_roi), 1e-6)
            diff = np.abs(depth_roi - aligned_roi)
            error_map[align_mask] = diff[align_mask] / denom[align_mask]
            threshold = max(1e-5, float(config.plane_match_error_thresh))
            match_mask = align_mask & (error_map <= threshold)
            kernel = max(1, int(config.plane_match_morph_kernel))
            if kernel > 1:
                match_mask = _binary_closing(match_mask, kernel)
            match_mask &= align_mask
            match_mask = _largest_component(match_mask)
            sample_count = int(match_mask.sum())
            if table_mask is not None and match_mask.any():
                table_roi = table_mask[roi_start:]
                table_roi[:] = False
                table_roi[match_mask] = True
            if sample_count >= min_samples:
                match_errors = error_map[match_mask]
                meas_vals = depth_roi[match_mask]
                plane_vals_aligned = aligned_roi[match_mask]
                ratios = meas_vals / np.maximum(plane_vals_aligned, 1e-6)
                mape = float(np.median(match_errors))
                outlier_fraction = float(np.mean(match_errors > 0.5))
                med_ratio = float(np.median(ratios))
                p95_ratio = float(np.percentile(ratios, 95))
                gap_threshold = 1.0 + max(0.0, config.plane_gap_ratio)
                gap_fraction = float(np.mean(ratios >= gap_threshold))
                consistency_ok = (
                    plane.alignment >= config.plane_min_alignment
                    and plane.bottom_coverage >= config.plane_bottom_coverage_min
                    and mape <= config.plane_mape_max
                    and outlier_fraction <= config.plane_outlier_fraction_max
                    and config.plane_median_ratio_min <= med_ratio <= config.plane_median_ratio_max
                    and p95_ratio <= config.plane_p95_ratio_max
                    and gap_fraction <= config.plane_gap_fraction_max
                )
                if consistency_ok:
                    aligned_plane = aligned_plane_full

        angle = math.degrees(math.acos(float(np.clip(plane.alignment, -1.0, 1.0))))
        debug = PlaneDebugInfo(
            score=plane.score,
            alignment=plane.alignment,
            bottom_coverage=plane.bottom_coverage,
            far_score=plane.far_score,
            support_ratio=plane.support_ratio,
            horizon_row=plane.horizon_row,
            angle_to_gravity_deg=angle,
            consistency_ok=consistency_ok,
            consistency_samples=sample_count,
            scale=scale,
            bias=bias,
            mape=mape,
            outlier_fraction=outlier_fraction,
            median_ratio=med_ratio,
            p95_ratio=p95_ratio,
            gap_fraction=gap_fraction,
            raw_ratio=plane.raw_ratio,
        )
        if not consistency_ok or aligned_plane is None:
            return False, None, debug, table_mask

        return True, aligned_plane, debug, table_mask

    def _fit_plane_scale_bias(self, plane_vals: np.ndarray, meas_vals: np.ndarray) -> tuple[float, float]:
        config = self._config
        rng = np.random.default_rng(self._config.msac_seed or None)
        x = plane_vals.astype(np.float32, copy=False)
        y = meas_vals.astype(np.float32, copy=False)
        valid = np.isfinite(x) & np.isfinite(y) & (x > 0.0)
        depth_cap = float(getattr(config, "plane_fit_depth_max", 0.0))
        if depth_cap > 0.0:
            valid &= x <= depth_cap
        x = x[valid]
        y = y[valid]
        n = x.size
        if n < 2:
            return 1.0, 0.0

        max_subset = min(n, 4096)
        if n > max_subset:
            idx = rng.choice(n, size=max_subset, replace=False)
            x_sample = x[idx]
            y_sample = y[idx]
        else:
            x_sample = x
            y_sample = y

        ratios = y_sample / np.maximum(x_sample, 1e-6)
        raw_scale = float(np.median(ratios))
        residuals = y_sample - raw_scale * x_sample
        raw_bias = float(np.median(residuals))

        scale = float(np.clip(raw_scale, config.plane_scale_min, config.plane_scale_max))
        median_depth = float(np.median(y_sample))
        bias_limit = float(config.plane_bias_abs_max_frac * max(median_depth, 1e-3))
        bias = float(np.clip(raw_bias, -bias_limit, bias_limit))

        _LOGGER.debug(
            "scale/bias fit: n=%d raw_scale=%.3f→%.3f bias=%.4f→%.4f z_range=(%.2f,%.2f)",
            n, raw_scale, scale, raw_bias, bias, x.min(), x.max()
        )
        if abs(scale - config.plane_scale_max) < 0.01:
            _LOGGER.warning(
                "Plane scale clipped to maximum %.2f (raw=%.2f) - consider increasing plane_scale_max",
                config.plane_scale_max, raw_scale
            )
        if abs(scale - config.plane_scale_min) < 0.01:
            _LOGGER.warning(
                "Plane scale clipped to minimum %.2f (raw=%.2f) - consider decreasing plane_scale_min",
                config.plane_scale_min, raw_scale
            )
        return scale, bias

    def _precompute_rays(self) -> np.ndarray:
        height = self._intrinsics.height
        width = self._intrinsics.width
        yy, xx = np.indices((height, width), dtype=np.float32)
        dir_x = (xx - float(self._intrinsics.cx)) / float(self._intrinsics.fx)
        dir_y = (yy - float(self._intrinsics.cy)) / float(self._intrinsics.fy)
        ones = np.ones_like(dir_x, dtype=np.float32)
        return np.stack((dir_x, dir_y, ones), axis=-1)

    def _project_plane_depth_map(self, plane: PlaneModel) -> np.ndarray:
        normal = plane.normal.astype(np.float32)
        offset = float(plane.offset)
        dirs = self._ray_directions
        denom = dirs[..., 0] * normal[0] + dirs[..., 1] * normal[1] + dirs[..., 2] * normal[2]
        depth = np.full(dirs.shape[:2], np.inf, dtype=np.float32)
        valid = np.abs(denom) > 1e-6
        t = np.zeros_like(depth)
        t[valid] = -offset / denom[valid]
        depth[valid] = t[valid]
        depth[(t <= 0) | (~np.isfinite(depth))] = np.inf
        return depth

    # ============================================================================
    # GRADIENT-SCAN HELPERS (NEW - 2025-01-13, Rewrite)
    # ============================================================================

    def _project_pixel_to_ground(self, pixel_x: float, pixel_y: float) -> Optional[np.ndarray]:
        """Project a pixel coordinate to 3D world coordinates on the ground plane (z=0).

        Args:
            pixel_x: X coordinate in image (column)
            pixel_y: Y coordinate in image (row)

        Returns:
            3D point [x, y, z] in base frame (meters), or None if projection fails
        """
        if self._extrinsics is None:
            return None

        # Convert pixel to normalized camera coordinates
        x_norm = (pixel_x - self._intrinsics.cx) / self._intrinsics.fx
        y_norm = (pixel_y - self._intrinsics.cy) / self._intrinsics.fy

        # Ray direction in camera frame (normalized)
        ray_camera = np.array([x_norm, y_norm, 1.0], dtype=np.float32)

        # Transform ray to base frame using extrinsics
        rotation = self._extrinsics[:3, :3]
        translation = self._extrinsics[:3, 3]
        ray_base = rotation @ ray_camera  # Ray direction in base frame
        camera_origin_base = translation  # Camera position in base frame

        # Intersect ray with ground plane (z = 0)
        # Ray equation: point = camera_origin + t * ray_base
        # Ground plane: z = 0
        # Solve: camera_origin.z + t * ray_base.z = 0
        if abs(ray_base[2]) < 1e-6:
            # Ray is parallel to ground plane, no intersection
            return None

        t = -camera_origin_base[2] / ray_base[2]

        if t <= 0:
            # Intersection is behind camera
            return None

        # Compute intersection point
        ground_point = camera_origin_base + t * ray_base

        return ground_point

    def _project_polyline_to_world(self, polyline_px: np.ndarray) -> tuple[Optional[np.ndarray], Optional[float]]:
        """Project a polyline from image space to world coordinates.

        Args:
            polyline_px: (N, 2) array of pixel coordinates (x, y)

        Returns:
            Tuple of:
            - polyline_world: (N, 3) array of 3D points in base frame (meters), or None if projection fails
            - distance_to_robot: Minimum horizontal distance to robot (meters), or None
        """
        if self._extrinsics is None:
            return None, None

        world_points = []
        for pixel in polyline_px:
            pixel_x, pixel_y = float(pixel[0]), float(pixel[1])
            world_point = self._project_pixel_to_ground(pixel_x, pixel_y)
            if world_point is not None:
                world_points.append(world_point)

        if not world_points:
            return None, None

        polyline_world = np.array(world_points, dtype=np.float32)

        # Apply empirical scale factor to correct systematic bias
        # Calibrated from real-world measurements: calculated distances were ~29% too large
        # Scale factor = 1/1.29 ≈ 0.775
        DISTANCE_SCALE_FACTOR = 0.775

        polyline_world = polyline_world * DISTANCE_SCALE_FACTOR

        # Calculate horizontal distance to robot (x-y plane distance)
        # Robot is at origin in base frame
        distances = np.sqrt(polyline_world[:, 0]**2 + polyline_world[:, 1]**2)
        min_distance = float(np.min(distances))

        return polyline_world, min_distance

    def _boundary_to_cliff_segment(
        self,
        boundary_coords: np.ndarray,
        depth_map: np.ndarray,
        confidence: Optional[float] = None,
        valid_mask: Optional[np.ndarray] = None,
    ) -> Optional[CliffSegment]:
        """Convert boundary coordinates to CliffSegment.

        Args:
            boundary_coords: (N, 2) array of (x, y) pixel coordinates
            depth_map: Depth map for computing segment depth
            confidence: Optional confidence score (0-1), defaults to 0.9
            valid_mask: Optional boolean mask of valid depth pixels (PHASE 5)

        Returns:
            CliffSegment or None if conversion fails
        """
        if boundary_coords.size == 0:
            return None

        # CliffSegment expects polyline in (x, y) format
        polyline_px = boundary_coords.astype(np.float32)

        # Compute segment length
        if len(polyline_px) > 1:
            diffs = np.diff(polyline_px, axis=0)
            segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
            length_px = float(np.sum(segment_lengths))
        else:
            length_px = 0.0

        # Use provided confidence or default to high confidence
        if confidence is None:
            confidence = 0.9

        # Unsupported ratio is not available from gradient scan, set to 0
        unsupported_ratio = 0.0

        # Project polyline to world coordinates (if extrinsics available)
        polyline_world, distance_to_robot = self._project_polyline_to_world(polyline_px)

        # PHASE 5: Compute depth statistics for segment matching
        mean_depth, depth_std = None, None
        if valid_mask is not None:
            mean_depth, depth_std = compute_depth_statistics(
                polyline_px, depth_map, valid_mask
            )

        # Create segment
        return CliffSegment(
            polyline_px=polyline_px,
            confidence=confidence,
            unsupported_ratio=unsupported_ratio,
            length_px=length_px,
            polyline_world=polyline_world,
            distance_to_robot=distance_to_robot,
            mean_depth=mean_depth,
            depth_std=depth_std,
        )


__all__ = ["DetectorConfig", "CliffDetector", "CliffDetectorOutput", "PlaneEvidence", "PlaneDebugInfo"]
