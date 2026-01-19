"""Cliff edge detection using depth gradients."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:  # Optional dependency for morphology
    import cv2  # type: ignore
except Exception:  # pragma: no cover - OpenCV optional
    cv2 = None


if False:  # pragma: no cover - typing aid without runtime import
    from .cliff_detector import DetectorConfig


@dataclass(frozen=True)
class CliffSegment:
    """Detected cliff segment represented as a polyline in image space."""

    polyline_px: np.ndarray  # shape (M, 2) with columns (x, y)
    confidence: float
    unsupported_ratio: float
    length_px: float
    polyline_world: Optional[np.ndarray] = None  # shape (M, 3) with columns (x, y, z) in meters (base frame)
    distance_to_robot: Optional[float] = None  # Minimum distance to robot in meters

    # PHASE 5: Depth statistics for segment matching
    mean_depth: Optional[float] = None  # Average depth along cliff edge
    depth_std: Optional[float] = None   # Depth standard deviation


@dataclass(frozen=True)
class CliffDetectionResult:
    """Aggregate result for a single frame."""

    segments: Sequence[CliffSegment]
    table_mask: np.ndarray  # bool array (H, W)
    boundary_mask: np.ndarray  # bool array (H, W)
    debug: "CliffDebugStats"

    def has_cliff(self) -> bool:
        return any(seg.confidence > 0.0 for seg in self.segments)


@dataclass(frozen=True)
class CliffDebugStats:
    """Instrumentation to aid tuning of the gradient/occlusion/plane cues."""

    roi_start: int
    roi_rows: int
    gradient_pixels: int
    occlusion_pixels: int
    plane_gap_pixels: int
    union_pixels: int
    gradient_ratio: float
    occlusion_ratio: float
    plane_gap_ratio: float
    band_hit_ratio: float
    table_area_ratio: float
    band_area_ratio: float
    union_ratio: float
    plane_gap_fused: bool


def detect_cliff_segments(
    *,
    depth_map: np.ndarray,
    valid_mask: np.ndarray,
    config: "DetectorConfig",
    plane_depth_map: Optional[np.ndarray] = None,
    plane_ok: bool = False,
    horizon_row: Optional[int] = None,
    plane_table_mask: Optional[np.ndarray] = None,
) -> CliffDetectionResult:
    """Extract cliff segments from a single DepthAnything frame.

    Args:
        depth_map: Depth map aligned with the RGB frame (H, W).
        valid_mask: Boolean mask indicating pixels with reliable depth.
        config: Detector configuration controlling ROI and thresholds.
        plane_depth_map: Optional per-pixel depth predicted by the validated plane.
        plane_ok: Whether the supplied plane depth map passed validation.
        horizon_row: Optional row index of the plane horizon used for ROI trimming.
        plane_table_mask: Optional binary mask of the validated tabletop region.

    Returns:
        CliffDetectionResult containing candidate segments and masks.
    """

    depth = np.asarray(depth_map, dtype=np.float32)
    mask = np.asarray(valid_mask, dtype=bool)
    if depth.shape != mask.shape:
        raise ValueError("depth_map and valid_mask must share the same shape")
    plane_depth = None
    if plane_depth_map is not None:
        plane_depth = np.asarray(plane_depth_map, dtype=np.float32)
        if plane_depth.shape != depth.shape:
            raise ValueError("plane_depth_map must match depth_map shape")

    height, width = depth.shape
    roi_start, roi_rows = config.gradient_roi_bounds(height, horizon_row if plane_ok else None)
    roi_start = max(0, roi_start)
    depth_roi = depth[roi_start:]
    valid_roi = mask[roi_start:]
    plane_roi = plane_depth[roi_start:] if plane_depth is not None else None

    plane_table_roi: Optional[np.ndarray] = None
    if plane_table_mask is not None:
        plane_mask = np.asarray(plane_table_mask, dtype=bool)
        if plane_mask.shape != mask.shape:
            raise ValueError("plane_table_mask must match depth_map shape")
        plane_table_roi = plane_mask[roi_start:]

    table_area_ratio = 0.0
    table_source: Optional[np.ndarray] = None
    if plane_table_roi is not None:
        table_source, table_area_ratio = _prepare_table_mask(
            plane_table_roi,
            float(np.clip(getattr(config, "plane_table_area_min", 0.0), 0.0, 1.0)),
        )

    if table_source is None:
        fallback_rows = max(
            1,
            min(int(getattr(config, "plane_table_fallback_rows", 24)), roi_rows),
        )
        table_source = np.zeros_like(valid_roi, dtype=bool)
        if fallback_rows > 0:
            table_source[-fallback_rows:, :] = True
        table_area_ratio = float(np.mean(table_source)) if table_source.size else 0.0

    table_mask = np.zeros_like(mask, dtype=bool)
    table_mask[roi_start:] = table_source

    boundary_mask = np.zeros_like(mask)
    segments: List[CliffSegment] = []

    empty_debug = CliffDebugStats(
        roi_start=int(roi_start),
        roi_rows=int(roi_rows),
        gradient_pixels=0,
        occlusion_pixels=0,
        plane_gap_pixels=0,
        union_pixels=0,
        gradient_ratio=0.0,
        occlusion_ratio=0.0,
        plane_gap_ratio=0.0,
        band_hit_ratio=0.0,
        table_area_ratio=table_area_ratio,
        band_area_ratio=0.0,
        union_ratio=0.0,
        plane_gap_fused=False,
    )

    if roi_rows < 2 or not valid_roi.any():
        return CliffDetectionResult(
            segments=segments,
            table_mask=table_mask,
            boundary_mask=boundary_mask,
            debug=empty_debug,
        )

    depth_proc = depth_roi.copy()
    if cv2 is not None and config.gradient_blur_kernel > 1:
        kernel_size = int(max(1, config.gradient_blur_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1
        fill_value = float(np.nanmedian(depth_roi[valid_roi])) if valid_roi.any() else 0.0
        temp = np.where(valid_roi, depth_roi, fill_value).astype(np.float32)
        depth_proc = cv2.GaussianBlur(temp, (kernel_size, kernel_size), 0)

    if depth_proc.shape[0] < 2:
        return CliffDetectionResult(
            segments=segments,
            table_mask=table_mask,
            boundary_mask=boundary_mask,
            debug=empty_debug,
        )

    valid_depth_values = depth_proc[valid_roi]
    z_up = np.clip(depth_proc[:-1, :], 1e-6, None)
    z_down = np.clip(depth_proc[1:, :], 1e-6, None)
    valid_pair = valid_roi[1:, :] & valid_roi[:-1, :]
    invalid_transition = valid_roi[:-1, :] & ~valid_roi[1:, :]

    ratio_threshold = 1.0 + max(float(config.gradient_relative_ratio), 0.0)
    grad_ratio = (z_down / z_up) > ratio_threshold

    grad_mask = grad_ratio & valid_pair
    grad_mask_raw = grad_mask.copy()

    k_run = max(0, int(getattr(config, "occlusion_min_invalid_run", 0)))
    occl_mask = np.zeros_like(grad_mask, dtype=bool)
    if k_run > 0 and invalid_transition.any():
        invalid = ~valid_roi
        run = np.zeros_like(invalid, dtype=np.int32)
        for row in range(invalid.shape[0] - 2, -1, -1):
            run[row, :] = np.where(invalid[row, :], run[row + 1, :] + 1, 0)

        run_down = run[1:, :]
        occl_mask = invalid_transition & (run_down >= k_run)

        border = max(0, int(getattr(config, "occlusion_border_px", 0)))
        if border > 0:
            occl_mask[:border, :] = False
            if occl_mask.shape[0] > border:
                occl_mask[-border:, :] = False

        near_quantile = float(np.clip(getattr(config, "occlusion_near_quantile", 0.3), 0.0, 1.0))
        if valid_depth_values.size and near_quantile < 1.0:
            z_thresh = float(np.quantile(valid_depth_values, near_quantile))
            occl_mask &= depth_proc[:-1, :] <= z_thresh
        elif not valid_depth_values.size:
            occl_mask[:] = False
    occl_mask_raw = occl_mask.copy()

    plane_gap_ratio_cfg = max(0.0, float(getattr(config, "plane_gap_ratio", 0.0)))
    plane_gap_abs = max(0.0, float(getattr(config, "plane_gap_abs", 0.0)))
    plane_mask_raw = np.zeros_like(grad_mask, dtype=bool)
    plane_gap_pixels = np.zeros_like(valid_roi, dtype=bool)
    band_hit_ratio = 0.0
    band_area_ratio = 0.0
    gap_weight = 1.0 if plane_ok else float(np.clip(getattr(config, "plane_gap_weak_weight", 0.35), 0.0, 1.0))
    if plane_roi is not None and table_source is not None:
        plane_valid = np.isfinite(plane_roi) & (plane_roi > 0.0)
        band_px = max(0, int(getattr(config, "plane_boundary_band_px", 0)))
        lateral_px = max(0, int(getattr(config, "plane_boundary_lateral_px", 0)))
        # Only use bottom boundary of table footprint (where cliff is likely)
        boundary_band = _build_bottom_boundary_band(table_source, band_px, lateral_px) if band_px > 0 else None
        if boundary_band is not None and boundary_band.any():
            band_area_ratio = float(np.clip(np.mean(boundary_band), 0.0, 1.0))
            max_band_ratio = float(np.clip(getattr(config, "plane_band_max_ratio", 0.15), 0.0, 1.0))
            min_band_ratio = float(np.clip(getattr(config, "plane_band_min_area_ratio", 0.05), 0.0, 1.0))
            if band_area_ratio < min_band_ratio or band_area_ratio > max_band_ratio:
                boundary_band = None
        if boundary_band is None or not boundary_band.any():
            boundary_band = None
        if boundary_band is not None:
            ratio_mask = np.zeros_like(boundary_band, dtype=bool)
            abs_mask = np.zeros_like(boundary_band, dtype=bool)
            denom = np.maximum(plane_roi, 1e-6)
            if plane_gap_ratio_cfg > 0.0:
                ratio_mask = (
                    boundary_band
                    & plane_valid
                    & valid_roi
                    & ((depth_proc / denom) >= (1.0 + plane_gap_ratio_cfg))
                )
            if plane_gap_abs > 0.0:
                abs_mask = boundary_band & plane_valid & valid_roi & ((depth_proc - plane_roi) >= plane_gap_abs)
            if plane_gap_ratio_cfg > 0.0 and plane_gap_abs > 0.0:
                gap_mask = ratio_mask & abs_mask
            elif plane_gap_ratio_cfg > 0.0:
                gap_mask = ratio_mask
            elif plane_gap_abs > 0.0:
                gap_mask = abs_mask
            else:
                gap_mask = boundary_band & plane_valid & valid_roi
            missing_mask = boundary_band & ~valid_roi
            unsupported_mask = _compute_outer_unsupported(boundary_band, valid_roi, max(k_run, 1))
            plane_gap_pixels = gap_mask | missing_mask | unsupported_mask
            band_columns = boundary_band.any(axis=0)
            total_band_cols = int(band_columns.sum())
            if total_band_cols > 0:
                gap_cols = plane_gap_pixels.any(axis=0)
                band_hit_ratio = float(np.clip(gap_cols.sum() / max(total_band_cols, 1), 0.0, 1.0))
                min_hit = float(np.clip(getattr(config, "plane_band_min_hit_ratio", 0.15), 0.0, 1.0))
                if band_hit_ratio < min_hit:
                    plane_gap_pixels[:] = False
            if plane_gap_pixels.shape[0] > 1:
                plane_mask_raw = plane_gap_pixels[:-1, :] | plane_gap_pixels[1:, :]

    area = max(1, grad_mask_raw.size)
    plane_gap_pixels_count = int(plane_mask_raw.sum())
    plane_gap_ratio_val = float(plane_gap_pixels_count / area)
    fuse_threshold = float(np.clip(getattr(config, "plane_gap_fuse_ratio", 0.0), 0.0, 1.0))
    plane_gap_fused = False
    plane_mask_active = plane_mask_raw.copy()
    if fuse_threshold > 0.0 and plane_gap_ratio_val > fuse_threshold:
        plane_mask_active[:] = False
        plane_gap_fused = True

    grad_mask = grad_mask | occl_mask | plane_mask_active

    gradient_pixels = int(grad_mask_raw.sum())
    occlusion_pixels = int(occl_mask_raw.sum())
    plane_pixels = plane_gap_pixels_count
    union_mask = grad_mask_raw | occl_mask_raw | plane_mask_active
    union_pixels = int(union_mask.sum())
    plane_conf_mask = plane_mask_raw if not plane_gap_fused else np.zeros_like(plane_mask_raw, dtype=bool)

    if cv2 is not None and config.gradient_morph_kernel > 1:
        kernel_size = int(max(1, config.gradient_morph_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        grad_mask_uint8 = grad_mask.astype(np.uint8) * 255
        grad_mask_uint8 = cv2.morphologyEx(grad_mask_uint8, cv2.MORPH_CLOSE, kernel)
        grad_mask = grad_mask_uint8.astype(bool)

    column_hits = grad_mask.any(axis=0)
    min_columns = max(1, int(config.gradient_min_columns))
    bridge_gap = max(0, int(config.gradient_bridge_gap))
    max_gap = max(0, int(getattr(config, "gradient_max_gap", 0)))
    extension_limit = bridge_gap
    bridge_rows = int(np.clip(config.gradient_bridge_rows, 1, roi_rows))
    bridge_rows = max(1, min(roi_rows, bridge_rows))
    bridge_slice = valid_roi[-bridge_rows:, :] if bridge_rows > 0 else valid_roi
    bridge_valid_ratio = (
        bridge_slice.mean(axis=0) if bridge_slice.size else np.zeros(width, dtype=np.float32)
    )
    bridge_threshold = float(np.clip(config.gradient_bridge_valid_ratio, 0.0, 1.0))

    col = 0
    while col < width:
        if not column_hits[col]:
            col += 1
            continue

        start_col = col
        col_values: dict[int, int] = {}

        while col < width and column_hits[col]:
            rows = np.where(grad_mask[:, col])[0]
            if rows.size > 0:
                row_idx = int(rows[0])
                y_pix = int(np.clip(roi_start + row_idx + 1, 0, height - 1))
                col_values[col] = y_pix
            col += 1

        end_col = col
        num_columns = end_col - start_col
        if num_columns < min_columns or not col_values:
            continue

        base_cols = sorted(col_values.keys())
        blocks: List[List[int]] = []
        current_block: List[int] = [base_cols[0]]
        for current_col in base_cols[1:]:
            if max_gap > 0 and current_col - current_block[-1] > max_gap + 1:
                blocks.append(current_block)
                current_block = [current_col]
            else:
                current_block.append(current_col)
        blocks.append(current_block)

        for block_cols in blocks:
            hit_columns = len(block_cols)
            if hit_columns < min_columns:
                continue

            ratio = hit_columns / max(block_cols[-1] - block_cols[0] + 1, 1)
            if ratio < float(np.clip(config.gradient_min_score, 0.0, 1.0)):
                continue

            left = block_cols[0]
            right = block_cols[-1]
            left_ext = left
            right_ext = right
            if extension_limit > 0:
                while (
                    left_ext > 0
                    and (left - (left_ext - 1)) <= extension_limit
                    and not column_hits[left_ext - 1]
                    and bridge_valid_ratio[left_ext - 1] <= bridge_threshold
                ):
                    left_ext -= 1
                while (
                    right_ext + 1 < width
                    and ((right_ext + 1) - right) <= extension_limit
                    and not column_hits[right_ext + 1]
                    and bridge_valid_ratio[right_ext + 1] <= bridge_threshold
                ):
                    right_ext += 1

            cols_block = list(range(left_ext, right_ext + 1))
            span_ext = len(cols_block)
            if span_ext < min_columns:
                continue

            coverage = hit_columns / max(span_ext, 1)
            if coverage < float(np.clip(config.gradient_min_score, 0.0, 1.0)):
                continue

            median_row = int(np.clip(np.median([col_values[c] for c in block_cols]), roi_start, height - 1))
            xs: list[float] = []
            ys: list[float] = []
            for column in cols_block:
                xs.append(float(column))
                row_value = int(np.clip(col_values.get(column, median_row), roi_start, height - 1))
                ys.append(float(row_value))
                boundary_mask[row_value, column] = True
                if row_value + 1 < height:
                    table_mask[row_value + 1 :, column] = False

            polyline = np.column_stack((np.array(xs, dtype=np.float32), np.array(ys, dtype=np.float32)))
            if polyline.shape[0] < 2:
                continue

            window = int(max(1, getattr(config, "gradient_window_size", 1)))
            if window > 1:
                if window % 2 == 0:
                    window += 1
                ys_smooth = ys.copy()
                half = window // 2
                for idx_pt in range(len(ys)):
                    left_idx = max(0, idx_pt - half)
                    right_idx = min(len(ys), idx_pt + half + 1)
                    ys_smooth[idx_pt] = float(np.median(ys[left_idx:right_idx]))
                polyline[:, 1] = np.array(ys_smooth, dtype=np.float32)

            length = _polyline_length(polyline)
            min_length = max(0.0, float(getattr(config, "min_segment_length_px", 0.0)))
            if length < min_length:
                continue

            bridge_deficit = float(
                np.clip(
                    1.0 - float(np.mean(bridge_valid_ratio[left_ext : right_ext + 1]))
                    if right_ext >= left_ext
                    else 0.0,
                    0.0,
                    1.0,
                )
            )

            occlusion_strength = 0.0
            if occl_mask_raw.any() and right_ext >= left_ext:
                col_slice = slice(left_ext, right_ext + 1)
                occlusion_strength = float(np.mean(occl_mask_raw[:, col_slice].any(axis=0)))

            gap_strength = 0.0
            if plane_conf_mask.any() and right_ext >= left_ext:
                col_slice = slice(left_ext, right_ext + 1)
                gap_strength = float(np.mean(plane_conf_mask[:, col_slice].any(axis=0))) * gap_weight

            support_strength = float(np.clip(max(bridge_deficit, occlusion_strength, gap_strength), 0.0, 1.0))
            boost = np.clip(0.5 + 0.3 * gap_strength + 0.2 * occlusion_strength, 0.0, 1.0)
            confidence = float(np.clip(coverage * boost, 0.0, 1.0))

            segments.append(
                CliffSegment(
                    polyline_px=polyline,
                    confidence=confidence,
                    unsupported_ratio=support_strength,
                    length_px=length,
                )
            )

    debug_stats = CliffDebugStats(
        roi_start=int(roi_start),
        roi_rows=int(roi_rows),
        gradient_pixels=gradient_pixels,
        occlusion_pixels=occlusion_pixels,
        plane_gap_pixels=plane_pixels,
        union_pixels=union_pixels,
        gradient_ratio=float(gradient_pixels / area),
        occlusion_ratio=float(occlusion_pixels / area),
        plane_gap_ratio=plane_gap_ratio_val,
        band_hit_ratio=band_hit_ratio,
        table_area_ratio=table_area_ratio,
        band_area_ratio=band_area_ratio,
        union_ratio=float(union_pixels / area),
        plane_gap_fused=plane_gap_fused,
    )

    return CliffDetectionResult(
        segments=segments,
        table_mask=table_mask,
        boundary_mask=boundary_mask,
        debug=debug_stats,
    )


def _polyline_length(polyline: np.ndarray) -> float:
    if polyline.shape[0] < 2:
        return 0.0
    diffs = np.diff(polyline, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def _build_bottom_boundary_band(table_roi: np.ndarray, vertical_px: int, lateral_px: int) -> np.ndarray:
    """Extract only the bottom boundary of table footprint (where cliff is likely)."""
    table_roi = np.asarray(table_roi, dtype=bool)
    if not table_roi.any():
        return np.zeros_like(table_roi, dtype=bool)

    vertical_px = max(1, int(vertical_px))
    lateral_px = max(0, int(lateral_px))
    height, width = table_roi.shape

    # Find the bottom edge: for each column, find the last (bottommost) True pixel
    bottom_edge = np.zeros_like(table_roi, dtype=bool)
    for col in range(width):
        column_mask = table_roi[:, col]
        if column_mask.any():
            # Find the last True index in this column
            last_true_idx = np.where(column_mask)[0][-1]
            bottom_edge[last_true_idx, col] = True

    if not bottom_edge.any():
        return np.zeros_like(table_roi, dtype=bool)

    # Expand the bottom edge upward by vertical_px rows
    band = np.zeros_like(table_roi, dtype=bool)
    max_offset = min(vertical_px, max(1, height - 1))
    for offset in range(1, max_offset + 1):
        shifted = np.zeros_like(table_roi, dtype=bool)
        shifted[:-offset, :] = bottom_edge[offset:, :]
        band |= shifted

    # Horizontal dilation
    if lateral_px > 0 and band.any():
        band = _dilate_horizontal(band, lateral_px)

    # Exclude pixels inside table
    band &= ~table_roi
    return band


def _build_boundary_band(table_roi: np.ndarray, vertical_px: int, lateral_px: int) -> np.ndarray:
    table_roi = np.asarray(table_roi, dtype=bool)
    if not table_roi.any():
        return np.zeros_like(table_roi, dtype=bool)

    vertical_px = max(1, int(vertical_px))
    lateral_px = max(0, int(lateral_px))
    height, _ = table_roi.shape

    edge = np.zeros_like(table_roi, dtype=bool)
    edge[0, :] = table_roi[0, :]
    edge[1:, :] = table_roi[1:, :] & ~table_roi[:-1, :]
    if not edge.any():
        return np.zeros_like(table_roi, dtype=bool)

    band = np.zeros_like(table_roi, dtype=bool)
    max_offset = min(vertical_px, max(1, height - 1))
    for offset in range(1, max_offset + 1):
        shifted = np.zeros_like(table_roi, dtype=bool)
        shifted[:-offset, :] = edge[offset:, :]
        band |= shifted

    if lateral_px > 0 and band.any():
        band = _dilate_horizontal(band, lateral_px)

    band &= ~table_roi
    return band


def _dilate_horizontal(mask: np.ndarray, radius: int) -> np.ndarray:
    radius = max(0, int(radius))
    mask = np.asarray(mask, dtype=bool)
    if radius == 0 or not mask.any():
        return mask.copy()
    padded = np.pad(mask, ((0, 0), (radius, radius)), mode="constant", constant_values=False)
    height, width = mask.shape
    result = np.zeros_like(mask, dtype=bool)
    for dx in range(0, 2 * radius + 1):
        result |= padded[:, dx : dx + width]
    return result


def _compute_outer_unsupported(band_mask: np.ndarray, valid_roi: np.ndarray, min_run: int) -> np.ndarray:
    band_mask = np.asarray(band_mask, dtype=bool)
    if not band_mask.any():
        return np.zeros_like(band_mask, dtype=bool)
    min_run = max(0, int(min_run))
    if min_run == 0:
        return np.zeros_like(band_mask, dtype=bool)

    invalid = ~valid_roi
    run = np.zeros_like(invalid, dtype=np.int32)
    for row in range(1, invalid.shape[0]):
        prev_invalid = invalid[row - 1, :]
        run[row, :] = np.where(prev_invalid, run[row - 1, :] + 1, 0)

    result = np.zeros_like(band_mask, dtype=bool)
    rows, cols = np.where(band_mask)
    if rows.size == 0:
        return result

    unsupported = (run[rows, cols] >= min_run) | (rows < min_run)
    result[rows, cols] = unsupported
    return result


def _prepare_table_mask(mask: np.ndarray, min_area_ratio: float) -> Tuple[Optional[np.ndarray], float]:
    table = np.asarray(mask, dtype=bool)
    if table.size == 0 or not table.any():
        return None, 0.0

    if cv2 is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))
        table_u8 = cv2.morphologyEx((table.astype(np.uint8) * 255), cv2.MORPH_CLOSE, kernel)
        table = table_u8 > 0
    else:
        table = _largest_component_mask(table)

    table = _largest_component_mask(table)
    rows = np.where(table)[0]
    if rows.size:
        centroid = float(np.mean(rows))
        if centroid < 0.4 * float(table.shape[0]):
            table = _bottom_connected(table)
    else:
        table = _bottom_connected(table)

    area_ratio = float(np.clip(np.mean(table), 0.0, 1.0)) if table.any() else 0.0
    if area_ratio < max(0.0, min_area_ratio):
        return None, area_ratio
    return table, area_ratio


def _largest_component_mask(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return mask.copy()

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best_size = 0
    best_coords: list[tuple[int, int]] = []

    ys, xs = np.where(mask)
    for y, x in zip(ys, xs):
        if visited[y, x]:
            continue
        stack = [(y, x)]
        visited[y, x] = True
        component: list[tuple[int, int]] = []
        while stack:
            cy, cx = stack.pop()
            component.append((cy, cx))
            for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx] and mask[ny, nx]:
                    visited[ny, nx] = True
                    stack.append((ny, nx))
        if len(component) > best_size:
            best_size = len(component)
            best_coords = component

    if best_size == 0:
        return np.zeros_like(mask, dtype=bool)

    result = np.zeros_like(mask, dtype=bool)
    for y, x in best_coords:
        result[y, x] = True
    return result


def _bottom_connected(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return mask.copy()

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    queue: list[tuple[int, int]] = []

    cols = np.where(mask[-1, :])[0]
    for col in cols:
        visited[-1, col] = True
        queue.append((height - 1, col))

    if not queue:
        return np.zeros_like(mask, dtype=bool)

    while queue:
        y, x = queue.pop()
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx] and mask[ny, nx]:
                visited[ny, nx] = True
                queue.append((ny, nx))

    return visited


__all__ = ["CliffSegment", "CliffDetectionResult", "CliffDebugStats", "detect_cliff_segments"]
