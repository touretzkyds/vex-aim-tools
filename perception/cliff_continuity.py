"""Gradient-based cliff detection with hysteresis edge linking.

This module implements cliff detection using Canny-style hysteresis thresholding
to connect edges of varying gradient strength into continuous contours.

Key insight: A cliff edge may have varying gradient magnitudes across its length
(e.g., 8→3 at one end, 2.5→1.0 at the other). Using dual thresholds and edge
linking allows us to detect the entire cliff boundary, not just the strongest parts.

Algorithm stages:
1. Compute gradient map with dual thresholds (strong/weak edges)
2. Hysteresis edge linking: connect weak edges to strong edges
3. Extract continuous contours from linked edges
4. Validate contours using context (table above, position, depth drop)
5. Select best cliff contour

Author: Claude Code
Date: 2025-01-13 (Rewrite v2 - Hysteresis)
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class GradientScanConfig:
    """Configuration for gradient-based cliff detection with hysteresis."""

    # === Dual Gradient Thresholds (Hysteresis) ===
    # Strong threshold: definite cliff edges (large jumps like 8→3)
    gradient_strong_threshold: float = 1.0

    # Weak threshold: possible cliff edges (smaller jumps like 2.5→1.0)
    # These are only included if connected to strong edges
    # PHASE 6A: Raised from 0.3 to 0.4 to reduce obstacle edge linking
    gradient_weak_threshold: float = 0.4

    # Edge linking parameters
    # Apply morphological closing to weak edges before linking (fills small gaps)
    weak_edge_morph_closing: int = 3  # Kernel size for closing (0 = disabled)

    # Gap tolerance: max distance to search for weak edges when linking
    # PHASE 6A: Reduced from 2 to 1 to reduce linking obstacle edges
    hysteresis_gap_tolerance: int = 1  # Can jump 1 pixel gap

    # === Context Validation ===
    # Number of rows above cliff to check for table presence
    table_context_rows: int = 30

    # Minimum fraction of valid pixels in above-region (continuity check)
    above_valid_ratio_min: float = 0.6

    # Minimum depth drop from above-region to below-region
    # UPDATED (Phase 5): Lowered from 0.5 to 0.25 for distant cliff detection
    depth_drop_min: float = 0.25

    # === Positional Prior ===
    # Cliff must be in bottom portion of image (robot's forward view)
    # UPDATED (Phase 5): Lowered from 0.4 to 0.15 to support 20-30cm distance
    cliff_position_min: float = 0.15

    # === Contour Filtering ===
    # Minimum contour length (pixels)
    # Lowered from 50 to 15 because morphological closing can't always merge all fragments
    min_contour_length: int = 15

    # Minimum horizontal span (fraction of image width)
    # Lowered from 0.15 to 0.03 to handle fragmented edges
    min_horizontal_span: float = 0.03

    # === Refinement ===
    # Smoothing window for boundary polyline
    boundary_smoothing_window: int = 5


@dataclass(frozen=True)
class GradientScanResult:
    """Result of gradient-based cliff detection."""

    # Cliff detected or not
    detected: bool

    # Cliff boundary coordinates (N, 2) array of (x, y) pixels
    # None if not detected
    # NOTE: This is the BEST segment for backward compatibility
    boundary_coords: Optional[np.ndarray] = None

    # Confidence score [0, 1] of the best segment
    confidence: float = 0.0

    # PHASE 5: All detected segments (polyline, confidence, reason)
    # List of tuples: (boundary_coords, confidence, validation_reason)
    # Sorted by confidence (descending)
    all_segments: Optional[List[Tuple[np.ndarray, float, str]]] = None

    # Diagnostic information
    num_strong_edges: int = 0
    num_weak_edges: int = 0
    num_linked_edges: int = 0
    num_contours: int = 0
    selected_contour_length: int = 0
    above_continuity: float = 0.0
    depth_drop: float = 0.0
    validation_reason: str = ""


def compute_depth_statistics(
    polyline: np.ndarray,  # (N, 2) pixel coords
    depth: np.ndarray,
    valid_mask: np.ndarray
) -> Tuple[Optional[float], Optional[float]]:
    """
    Compute depth statistics along a cliff edge polyline.

    PHASE 5: Used for segment matching by depth consistency.

    Args:
        polyline: (N, 2) array of (x, y) pixel coordinates
        depth: Depth map (H, W)
        valid_mask: Boolean mask of valid pixels

    Returns:
        (mean_depth, depth_std): Average depth and standard deviation,
                                  or (None, None) if no valid depths
    """
    depth_values = []

    for x, y in polyline:
        x, y = int(x), int(y)
        if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
            if valid_mask[y, x]:
                depth_values.append(depth[y, x])

    if not depth_values:
        return None, None

    depth_array = np.array(depth_values)
    return float(depth_array.mean()), float(depth_array.std())


def match_segments_by_depth_consistency(
    segments: List[Tuple[np.ndarray, float, str]],  # (polyline, confidence, reason)
    depth_mean_list: List[Optional[float]],
    depth_similarity_threshold: float = 0.15  # 15% tolerance
) -> List[List[int]]:
    """
    Group cliff segments by depth similarity.

    PHASE 5: Segments with similar depth belong to the same physical cliff edge.
    This helps merge left/right cliff segments when obstacle blocks center.

    Args:
        segments: List of (polyline, confidence, reason) tuples
        depth_mean_list: List of mean depths (same length as segments)
        depth_similarity_threshold: Relative depth difference threshold (0.15 = 15%)

    Returns:
        List of groups, where each group is a list of segment indices
    """
    if not segments:
        return []

    n = len(segments)

    # Filter valid segments (those with depth information)
    valid_indices = [i for i in range(n) if depth_mean_list[i] is not None]

    if not valid_indices:
        # No depth info, return each segment as separate group
        return [[i] for i in range(n)]

    groups = []
    remaining = set(valid_indices)

    while remaining:
        # Select seed: highest confidence among remaining
        seed_idx = max(remaining, key=lambda i: segments[i][1])  # segments[i][1] = confidence
        seed_depth = depth_mean_list[seed_idx]
        current_group = [seed_idx]
        remaining.remove(seed_idx)

        # Find segments with similar depth
        to_remove = []
        for idx in remaining:
            seg_depth = depth_mean_list[idx]

            # Depth similarity test
            depth_ratio = seg_depth / seed_depth
            relative_diff = abs(1.0 - depth_ratio)

            if relative_diff <= depth_similarity_threshold:
                # Belongs to same cliff
                current_group.append(idx)
                to_remove.append(idx)

        for idx in to_remove:
            remaining.remove(idx)

        groups.append(current_group)

    # Add segments without depth info as separate groups
    for i in range(n):
        if depth_mean_list[i] is None:
            groups.append([i])

    return groups


def compute_gradient_map_dual(
    depth: np.ndarray,
    valid_mask: np.ndarray,
    config: GradientScanConfig,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute vertical gradient map with dual thresholding.

    Args:
        depth: Depth map (H, W)
        valid_mask: Boolean mask of valid pixels
        config: Configuration

    Returns:
        gradient_map: (H-1, W) array of absolute gradient magnitudes
        strong_edge_mask: (H-1, W) boolean mask of strong edges
        weak_edge_mask: (H-1, W) boolean mask of weak edges
    """
    h, w = depth.shape

    # Compute vertical gradients (row-to-row)
    depth_below = depth[1:, :]
    depth_above = depth[:-1, :]
    gradient_map = np.abs(depth_below - depth_above)

    # Valid gradient computation (both pixels must be valid)
    valid_above = valid_mask[:-1, :]
    valid_below = valid_mask[1:, :]
    valid_gradient = valid_above & valid_below

    # Dual thresholding
    strong_threshold = config.gradient_strong_threshold
    weak_threshold = config.gradient_weak_threshold

    strong_edge_mask = (gradient_map > strong_threshold) & valid_gradient
    weak_edge_mask = (gradient_map > weak_threshold) & valid_gradient

    # Invalid gradients set to zero
    gradient_map = np.where(valid_gradient, gradient_map, 0.0)

    return gradient_map, strong_edge_mask, weak_edge_mask


def morphological_closing(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    """Apply morphological closing to fill small gaps in edge mask.

    Args:
        mask: Boolean mask
        kernel_size: Size of structuring element (must be odd)

    Returns:
        Closed mask
    """
    if kernel_size <= 1:
        return mask

    try:
        import cv2
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        closed = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        return closed.astype(bool)
    except ImportError:
        # Fallback: simple dilation + erosion
        radius = kernel_size // 2
        h, w = mask.shape

        # Dilation
        dilated = np.copy(mask)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dy == 0 and dx == 0:
                    continue
                shifted = np.zeros_like(mask, dtype=bool)
                y_start = max(0, dy)
                y_end = min(h, h + dy)
                x_start = max(0, dx)
                x_end = min(w, w + dx)

                shifted[y_start:y_end, x_start:x_end] = mask[max(0, -dy):min(h, h - dy), max(0, -dx):min(w, w - dx)]
                dilated |= shifted

        # Erosion
        eroded = np.ones_like(dilated, dtype=bool)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dy == 0 and dx == 0:
                    continue
                shifted = np.ones_like(dilated, dtype=bool)
                y_start = max(0, dy)
                y_end = min(h, h + dy)
                x_start = max(0, dx)
                x_end = min(w, w + dx)

                shifted[y_start:y_end, x_start:x_end] = dilated[max(0, -dy):min(h, h - dy), max(0, -dx):min(w, w - dx)]
                eroded &= shifted

        return eroded


def link_edges_hysteresis(
    strong_edges: np.ndarray,
    weak_edges: np.ndarray,
    gap_tolerance: int = 0,
) -> np.ndarray:
    """Link weak edges to strong edges using hysteresis with gap tolerance.

    Starting from strong edge pixels, recursively explore neighbors (with gap tolerance)
    to link any weak edge pixels that form continuous contours.

    Args:
        strong_edges: (H, W) boolean mask of strong edges
        weak_edges: (H, W) boolean mask of weak edges
        gap_tolerance: Maximum distance to search for weak edges (0 = adjacent only)

    Returns:
        linked_edges: (H, W) boolean mask of all linked edges (strong + connected weak)
    """
    h, w = strong_edges.shape
    linked = np.copy(strong_edges)
    visited = np.zeros((h, w), dtype=bool)

    # Generate neighbor offsets based on gap tolerance
    offsets = []
    search_radius = 1 + gap_tolerance
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            if dy == 0 and dx == 0:
                continue
            offsets.append((dy, dx))

    # BFS from each strong edge pixel
    queue = deque()

    # Initialize queue with all strong edge pixels
    strong_coords = np.argwhere(strong_edges)
    for y, x in strong_coords:
        queue.append((y, x))
        visited[y, x] = True

    # Traverse and link
    while queue:
        cy, cx = queue.popleft()

        for dy, dx in offsets:
            ny, nx = cy + dy, cx + dx

            # Check bounds
            if not (0 <= ny < h and 0 <= nx < w):
                continue

            # Skip if already visited
            if visited[ny, nx]:
                continue

            # If neighbor is a weak edge, link it
            if weak_edges[ny, nx]:
                linked[ny, nx] = True
                visited[ny, nx] = True
                queue.append((ny, nx))

    return linked


def extract_contours_from_edges(
    edge_mask: np.ndarray,
) -> list[np.ndarray]:
    """Extract continuous contours from edge mask.

    Groups connected edge pixels into separate contours using connected components.

    Args:
        edge_mask: (H, W) boolean mask of edge pixels

    Returns:
        List of contours, each is an (N, 2) array of (x, y) coordinates
    """
    if not edge_mask.any():
        return []

    h, w = edge_mask.shape
    visited = np.zeros_like(edge_mask, dtype=bool)
    contours = []

    # 8-connectivity offsets
    neighbors = [(-1, -1), (-1, 0), (-1, 1),
                 (0, -1),           (0, 1),
                 (1, -1),  (1, 0),  (1, 1)]

    # Find all connected components
    for start_y, start_x in zip(*np.where(edge_mask)):
        if visited[start_y, start_x]:
            continue

        # BFS to extract this contour
        contour_pixels = []
        queue = deque([(start_y, start_x)])
        visited[start_y, start_x] = True

        while queue:
            cy, cx = queue.popleft()
            contour_pixels.append((cx, cy))  # Note: (x, y) format

            for dy, dx in neighbors:
                ny, nx = cy + dy, cx + dx

                if not (0 <= ny < h and 0 <= nx < w):
                    continue

                if visited[ny, nx]:
                    continue

                if edge_mask[ny, nx]:
                    visited[ny, nx] = True
                    queue.append((ny, nx))

        if contour_pixels:
            contours.append(np.array(contour_pixels, dtype=np.int32))

    return contours


def build_contour_polyline(
    contour: np.ndarray,
    config: GradientScanConfig,
) -> np.ndarray:
    """Build a smooth polyline from scattered contour pixels.

    For cliff detection, we want a polyline that represents the cliff edge.
    Strategy: for each x-coordinate, find the bottommost y-coordinate in the contour.

    Args:
        contour: (N, 2) array of (x, y) pixel coordinates
        config: Configuration

    Returns:
        Polyline (M, 2) array of (x, y) coordinates, sorted by x
    """
    if contour.size == 0:
        return contour

    # Group by x-coordinate, take max y (bottommost) for each x
    x_coords = contour[:, 0]
    y_coords = contour[:, 1]

    min_x = int(x_coords.min())
    max_x = int(x_coords.max())

    polyline_points = []

    for x in range(min_x, max_x + 1):
        # Find all y values for this x
        mask = (x_coords == x)
        if not mask.any():
            continue

        y_values = y_coords[mask]
        # Take bottommost (max y) as the cliff edge for this column
        y_bottom = int(y_values.max())
        polyline_points.append((x, y_bottom))

    if not polyline_points:
        return np.array([], dtype=np.int32).reshape(0, 2)

    polyline = np.array(polyline_points, dtype=np.int32)

    # Optional smoothing
    if config.boundary_smoothing_window > 1 and len(polyline) > config.boundary_smoothing_window:
        smoothed_y = smooth_coordinates(polyline[:, 1], config.boundary_smoothing_window)
        polyline[:, 1] = smoothed_y

    return polyline


def split_polyline_by_depth_jumps(
    polyline: np.ndarray,
    depth: np.ndarray,
    valid_mask: np.ndarray,
    max_depth_jump_ratio: float = 0.3,  # 30% jump threshold
    min_segment_length: int = 15,
) -> list[np.ndarray]:
    """Split polyline at depth discontinuities (e.g., obstacle edges).

    PHASE 6A: Obstacle edges mixed into cliff contours cause false positives.
    By detecting depth jumps along the polyline, we can split off obstacle edges.

    Args:
        polyline: (N, 2) array of (x, y) pixel coordinates
        depth: Depth map (H, W)
        valid_mask: Valid pixel mask
        max_depth_jump_ratio: Maximum relative depth change between adjacent pixels
                               (e.g., 0.3 = 30% jump)
        min_segment_length: Minimum length of split segments to keep

    Returns:
        List of polyline segments (each is np.ndarray)
    """
    if polyline.size == 0 or len(polyline) < 2:
        return [polyline] if polyline.size > 0 else []

    # Extract depths along polyline
    depths = []
    valid_indices = []

    for i, (x, y) in enumerate(polyline):
        x, y = int(x), int(y)
        if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
            if valid_mask[y, x]:
                depths.append(depth[y, x])
                valid_indices.append(i)

    if len(depths) < 2:
        return [polyline]  # No valid depths, keep original

    # Detect jump points
    split_indices = []

    for i in range(1, len(depths)):
        prev_depth = depths[i - 1]
        curr_depth = depths[i]

        # Relative depth change
        if prev_depth > 0:  # Avoid division by zero
            relative_change = abs(curr_depth - prev_depth) / prev_depth

            if relative_change > max_depth_jump_ratio:
                # Found a depth jump, split at this polyline index
                polyline_idx = valid_indices[i]
                split_indices.append(polyline_idx)

    # Split polyline at jump points
    if not split_indices:
        return [polyline]  # No jumps, keep original

    segments = []
    start_idx = 0

    for split_idx in split_indices:
        segment = polyline[start_idx:split_idx]
        if len(segment) >= min_segment_length:
            segments.append(segment)
        start_idx = split_idx

    # Add last segment
    last_segment = polyline[start_idx:]
    if len(last_segment) >= min_segment_length:
        segments.append(last_segment)

    # If all segments were too short, keep original
    if not segments:
        return [polyline]

    return segments


def smooth_coordinates(y_coords: np.ndarray, window: int) -> np.ndarray:
    """Apply moving average smoothing to y-coordinates."""
    if window <= 1:
        return y_coords

    radius = window // 2
    smoothed = np.copy(y_coords).astype(np.float32)

    for i in range(len(y_coords)):
        left = max(0, i - radius)
        right = min(len(y_coords), i + radius + 1)
        smoothed[i] = np.mean(y_coords[left:right])

    return np.round(smoothed).astype(np.int32)


def validate_contour(
    polyline: np.ndarray,
    depth: np.ndarray,
    valid_mask: np.ndarray,
    config: GradientScanConfig,
) -> Tuple[bool, float, str]:
    """Validate whether a contour represents a true cliff edge.

    Args:
        polyline: (N, 2) array of (x, y) coordinates
        depth: Depth map
        valid_mask: Valid pixel mask
        config: Configuration

    Returns:
        (is_valid, confidence_score, reason_string)
    """
    if polyline.size == 0:
        return False, 0.0, "Empty polyline"

    h, w = depth.shape

    # === Check 1: Contour length ===
    if len(polyline) < config.min_contour_length:
        return False, 0.0, f"Too short ({len(polyline)} < {config.min_contour_length} px)"

    # === Check 2: Horizontal span ===
    x_coords = polyline[:, 0]
    x_span = x_coords.max() - x_coords.min() + 1
    horizontal_span_ratio = x_span / w

    if horizontal_span_ratio < config.min_horizontal_span:
        return False, 0.0, f"Insufficient horizontal span ({horizontal_span_ratio:.1%} < {config.min_horizontal_span:.1%})"

    # === Check 3: Positional prior ===
    y_coords = polyline[:, 1]
    mean_y = float(y_coords.mean())
    relative_position = mean_y / h

    if relative_position < config.cliff_position_min:
        return False, 0.0, f"Too high in image (mean_y={mean_y:.0f}/{h}, {relative_position:.1%})"

    # === Check 4: Table context above ===
    # Use mean_y as representative cliff position
    cliff_y = int(mean_y)
    table_check_rows = min(config.table_context_rows, cliff_y)

    if table_check_rows < 10:
        return False, 0.0, "Too close to top edge"

    above_start = cliff_y - table_check_rows
    above_region_mask = valid_mask[above_start:cliff_y, :]
    above_valid_ratio = float(above_region_mask.mean())

    if above_valid_ratio < config.above_valid_ratio_min:
        return False, 0.0, f"Above region not continuous ({above_valid_ratio:.1%} < {config.above_valid_ratio_min:.1%})"

    # === Check 5: Depth drop ===
    above_region_depth = depth[above_start:cliff_y, :]
    above_depths = above_region_depth[above_region_mask]

    if above_depths.size == 0:
        return False, 0.0, "No valid depths above"

    above_mean = float(above_depths.mean())

    # Check below region
    below_check_rows = min(20, h - cliff_y - 1)
    depth_drop = 0.0

    if below_check_rows > 0:
        below_end = cliff_y + 1 + below_check_rows
        below_region_depth = depth[cliff_y + 1:below_end, :]
        below_region_mask = valid_mask[cliff_y + 1:below_end, :]

        if below_region_mask.any():
            below_depths = below_region_depth[below_region_mask]
            below_mean = float(below_depths.mean())
            depth_drop = above_mean - below_mean

            # Accept both positive and negative depth drops (abs value)
            # Positive: table → drop-off (above > below)
            # Negative: background → table (above < below)
            if abs(depth_drop) < config.depth_drop_min:
                return False, 0.0, f"Insufficient depth change (|{depth_drop:.2f}| < {config.depth_drop_min})"

    # === All checks passed ===
    # Compute confidence
    confidence = min(1.0, (
        horizontal_span_ratio * 0.4 +
        above_valid_ratio * 0.3 +
        min(abs(depth_drop) / 2.0, 1.0) * 0.3
    ))

    reason = (
        f"Valid cliff: length={len(polyline)}px, span={horizontal_span_ratio:.1%}, "
        f"y={mean_y:.0f}, above_continuity={above_valid_ratio:.1%}, drop={depth_drop:.2f}, conf={confidence:.2f}"
    )

    return True, confidence, reason


def save_gradient_visualization(
    gradient_map: np.ndarray,
    strong_edges: np.ndarray,
    weak_edges: np.ndarray,
    linked_edges: np.ndarray,
    output_path: str,
) -> None:
    """Save visualization of gradient detection stages for debugging.

    Creates a composite image showing:
    - Top: Gradient magnitude heatmap
    - Middle: Strong edges (red), weak edges (yellow)
    - Bottom: Linked edges (green)

    Args:
        gradient_map: (H, W) gradient magnitudes
        strong_edges: (H, W) strong edge mask
        weak_edges: (H, W) weak edge mask
        linked_edges: (H, W) linked edge mask
        output_path: Path to save visualization
    """
    try:
        import cv2
    except ImportError:
        _LOGGER.warning("OpenCV not available, skipping gradient visualization")
        return

    h, w = gradient_map.shape

    # Pad to full image size (gradient_map is H-1)
    gradient_full = np.zeros((h + 1, w), dtype=np.float32)
    gradient_full[:-1, :] = gradient_map

    strong_full = np.zeros((h + 1, w), dtype=bool)
    strong_full[:-1, :] = strong_edges

    weak_full = np.zeros((h + 1, w), dtype=bool)
    weak_full[:-1, :] = weak_edges

    linked_full = np.zeros((h + 1, w), dtype=bool)
    linked_full[:-1, :] = linked_edges

    # Normalize gradient for visualization
    grad_vis = gradient_full.copy()
    if grad_vis.max() > 0:
        grad_vis = np.clip(grad_vis / 3.0, 0, 1) * 255  # Cap at 3.0 for visibility
    grad_vis_color = cv2.applyColorMap(grad_vis.astype(np.uint8), cv2.COLORMAP_JET)

    # Create edge overlays
    edges_vis = np.zeros((h + 1, w, 3), dtype=np.uint8)
    edges_vis[strong_full] = [0, 0, 255]  # Strong = Red
    edges_vis[weak_full & ~strong_full] = [0, 255, 255]  # Weak only = Yellow

    linked_vis = np.zeros((h + 1, w, 3), dtype=np.uint8)
    linked_vis[linked_full] = [0, 255, 0]  # Linked = Green

    # Stack vertically
    combined = np.vstack([grad_vis_color, edges_vis, linked_vis])

    cv2.imwrite(output_path, combined)
    _LOGGER.debug("Saved gradient visualization to %s", output_path)


def detect_cliff_by_gradient_scan(
    depth: np.ndarray,
    valid_mask: np.ndarray,
    config: GradientScanConfig,
    debug_output_path: Optional[str] = None,
) -> GradientScanResult:
    """Detect cliff edge using gradient-based scanning with hysteresis edge linking.

    Algorithm stages:
    1. Compute gradient map with dual thresholds (strong/weak)
    2. Link weak edges to strong edges (hysteresis)
    3. Extract continuous contours from linked edges
    4. Validate each contour (table context, position, depth drop)
    5. Select best contour by confidence

    Args:
        depth: Depth map (H, W) - inverse depth from DepthAnything V2
        valid_mask: Boolean mask of valid depth pixels
        config: Configuration parameters

    Returns:
        GradientScanResult with detected cliff boundary and diagnostics
    """
    h, w = depth.shape

    # Stage 1: Compute gradient map with dual thresholds
    gradient_map, strong_edges, weak_edges = compute_gradient_map_dual(
        depth, valid_mask, config
    )

    num_strong = int(strong_edges.sum())
    num_weak = int(weak_edges.sum())

    _LOGGER.debug(
        "gradient_scan: strong_edges=%d (%.2f%%), weak_edges=%d (%.2f%%)",
        num_strong,
        100.0 * num_strong / strong_edges.size,
        num_weak,
        100.0 * num_weak / weak_edges.size,
    )

    if num_strong == 0:
        return GradientScanResult(
            detected=False,
            num_strong_edges=0,
            num_weak_edges=num_weak,
            validation_reason="No strong edges found",
        )

    # Stage 2a: Apply morphological closing to weak edges (fills gaps)
    weak_edges_closed = weak_edges
    if config.weak_edge_morph_closing > 0:
        weak_edges_closed = morphological_closing(weak_edges, config.weak_edge_morph_closing)
        num_weak_closed = int(weak_edges_closed.sum())
        _LOGGER.debug(
            "gradient_scan: morphological closing added %d weak edge pixels",
            num_weak_closed - num_weak,
        )

    # Stage 2b: Hysteresis edge linking with gap tolerance
    linked_edges = link_edges_hysteresis(
        strong_edges, weak_edges_closed, gap_tolerance=config.hysteresis_gap_tolerance
    )
    num_linked = int(linked_edges.sum())

    _LOGGER.debug(
        "gradient_scan: linked_edges=%d (added %d weak edges)",
        num_linked,
        num_linked - num_strong,
    )

    # Stage 2c: Apply morphological closing to linked edges to bridge final gaps
    # This ensures pixels that are close but not 8-connected get merged
    # Use a wide horizontal rectangular kernel to connect horizontally-aligned edge pixels
    # (cliff edges are mostly horizontal, so use much wider kernel in X direction)
    import cv2
    horiz_kernel = np.ones((5, 51), dtype=np.uint8)  # 5 rows x 51 cols (~8% of 640px width)
    linked_edges_closed = cv2.morphologyEx(
        linked_edges.astype(np.uint8), cv2.MORPH_CLOSE, horiz_kernel
    ).astype(bool)
    num_linked_closed = int(linked_edges_closed.sum())

    if num_linked_closed > num_linked:
        _LOGGER.debug(
            "gradient_scan: morphological closing on linked edges added %d pixels",
            num_linked_closed - num_linked,
        )

    # Save visualization if debug path provided
    if debug_output_path:
        save_gradient_visualization(
            gradient_map, strong_edges, weak_edges, linked_edges_closed, debug_output_path
        )
        # Save the linked edges mask for detailed inspection
        linked_mask_path = debug_output_path.replace('.png', '_linked_mask.png')
        try:
            import cv2
            linked_vis = (linked_edges_closed.astype(np.uint8) * 255)
            cv2.imwrite(linked_mask_path, linked_vis)
            _LOGGER.debug("Saved linked edges mask to %s", linked_mask_path)
        except Exception as e:
            _LOGGER.warning("Failed to save linked edges mask: %s", e)

    # Stage 3: Extract contours
    contours = extract_contours_from_edges(linked_edges_closed)
    num_contours = len(contours)

    _LOGGER.debug("gradient_scan: extracted %d contours", num_contours)

    if num_contours == 0:
        return GradientScanResult(
            detected=False,
            num_strong_edges=num_strong,
            num_weak_edges=num_weak,
            num_linked_edges=num_linked_closed,
            num_contours=0,
            validation_reason="No contours extracted",
        )

    # Stage 4: Validate contours
    validated_contours = []

    for idx, contour in enumerate(contours):
        # Build polyline from contour
        polyline = build_contour_polyline(contour, config)

        if polyline.size == 0:
            continue

        # Validate
        is_valid, confidence, reason = validate_contour(
            polyline, depth, valid_mask, config
        )

        _LOGGER.debug("  Contour #%d: %s", idx, reason)

        if is_valid:
            validated_contours.append((polyline, confidence, reason))

    if not validated_contours:
        return GradientScanResult(
            detected=False,
            num_strong_edges=num_strong,
            num_weak_edges=num_weak,
            num_linked_edges=num_linked_closed,
            num_contours=num_contours,
            validation_reason="All contours failed validation",
        )

    # Stage 5: Select best contour & collect all high-confidence segments
    # PHASE 5: Return ALL validated segments (not just best)
    # UPDATED: Lowered from 0.3 to 0.2 to capture weaker cliff segments (e.g., partial occlusion)
    MIN_CONFIDENCE_THRESHOLD = 0.2

    # Sort by confidence (descending)
    validated_contours.sort(key=lambda x: x[1], reverse=True)

    # Best segment (for backward compatibility)
    best_polyline, best_confidence, best_reason = validated_contours[0]

    # All segments above threshold
    all_valid_segments = [
        (polyline, conf, reason)
        for polyline, conf, reason in validated_contours
        if conf >= MIN_CONFIDENCE_THRESHOLD
    ]

    _LOGGER.debug(
        "gradient_scan: selected best contour (length=%d, conf=%.2f), total valid segments=%d",
        len(best_polyline),
        best_confidence,
        len(all_valid_segments),
    )

    # Compute final diagnostics
    mean_y = float(best_polyline[:, 1].mean())
    table_rows = min(config.table_context_rows, int(mean_y))

    if table_rows > 0:
        above_start = int(mean_y) - table_rows
        above_mask = valid_mask[above_start:int(mean_y), :]
        above_continuity = float(above_mask.mean())
    else:
        above_continuity = 0.0

    # Compute depth drop
    cliff_y = int(mean_y)
    above_start = max(0, cliff_y - table_rows)
    above_depths = depth[above_start:cliff_y, :][valid_mask[above_start:cliff_y, :]]

    below_rows = min(20, h - cliff_y - 1)
    if below_rows > 0 and above_depths.size > 0:
        below_end = cliff_y + 1 + below_rows
        below_depths = depth[cliff_y + 1:below_end, :][valid_mask[cliff_y + 1:below_end, :]]
        if below_depths.size > 0:
            depth_drop = float(above_depths.mean() - below_depths.mean())
        else:
            depth_drop = 0.0
    else:
        depth_drop = 0.0

    return GradientScanResult(
        detected=True,
        boundary_coords=best_polyline,
        confidence=best_confidence,
        all_segments=all_valid_segments,  # PHASE 5: Include all segments
        num_strong_edges=num_strong,
        num_weak_edges=num_weak,
        num_linked_edges=num_linked_closed,
        num_contours=num_contours,
        selected_contour_length=len(best_polyline),
        above_continuity=above_continuity,
        depth_drop=depth_drop,
        validation_reason=best_reason,
    )
