import logging
import math
import threading
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

_LOGGER = logging.getLogger(__name__)

# Try to import CameraIntrinsics, handle if not available (e.g. circular import issues)
try:
    from perception.pointcloud_builder import CameraIntrinsics
except ImportError:
    # Fallback definition if import fails
    from dataclasses import dataclass
    @dataclass(frozen=True)
    class CameraIntrinsics:
        fx: float
        fy: float
        cx: float
        cy: float
        width: int
        height: int
        def validate_shape(self, shape): pass

class CellState(IntEnum):
    UNKNOWN = 0      # Never observed (log odds = 0)
    FREE = 1         # Observed as free space (log odds < threshold)
    OCCUPIED = 2     # Observed as obstacle (log odds > threshold)
    CLIFF = 3        # Marked as cliff (special status, overrides occupancy)

class OccupancyGrid:
    """
    Probabilistic Occupancy Grid using Log-Odds representation.
    
    Coordinate System:
        - World Frame (mm): Aligned with the robot's world frame (WorldMap).
        - Grid Frame (indices): (row, col) indices into the numpy array.
    
    The grid covers a fixed rectangular area in the world frame.
    """
    
    # Log-odds constants
    # p=0.5 -> log_odds=0
    # p=0.9 -> log_odds ≈ 2.2
    # p=0.1 -> log_odds ≈ -2.2
    # NOTE: We intentionally make FREE updates weaker than OCC updates so
    # obstacles "persist until re-observed as free" (no time decay), matching
    # the project requirements for ghost elimination by re-observation.
    LO_OCCUPIED = 1.00  # Increment for occupied observation
    LO_FREE = -0.35     # Decrement for free observation
    LO_MAX = 4.0        # Clamping max
    LO_MIN = -4.0       # Clamping min
    LO_THRESH_OCC = 0.7   # Threshold to consider cell OCCUPIED
    LO_THRESH_FREE = -0.7 # Threshold to consider cell FREE

    # Depth-to-ground classification parameters (meters)
    GROUND_TOL_BASE_M = 0.05
    GROUND_TOL_FRAC = 0.02
    OBSTACLE_HEIGHT_M = 0.05

    # Optional depth scale correction (robust median + EMA)
    SCALE_CORRECTION_ENABLED = False
    SCALE_RATIO_MIN = 0.05
    SCALE_RATIO_MAX = 20.0
    SCALE_INLIER_BAND = 0.4
    SCALE_MIN_INLIERS = 20
    SCALE_EMA_ALPHA = 0.2
    INVERT_DEPTH_ON_FAIL = True

    # Default sampling parameters (Method 1): fixed-budget rays with uniform
    # horizontal coverage and ground-focused vertical ROI.
    DEFAULT_SAMPLING = "bearing_bins"  # or "stride"
    DEFAULT_BEARING_BINS = 160
    DEFAULT_V_FRACTIONS = (0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90)

    # Endpoint splat (in cells)
    DEFAULT_SPLAT_RADIUS_OCC = 2
    DEFAULT_SPLAT_RADIUS_FREE = 1

    def __init__(self, 
                 x_range: tuple[float, float] = (-2500, 2500), 
                 y_range: tuple[float, float] = (-2500, 2500), 
                 resolution: float = 10.0):
        """
        Initialize the occupancy grid.
        
        Args:
            x_range: (min_x, max_x) in mm
            y_range: (min_y, max_y) in mm
            resolution: grid cell size in mm
        """
        self.resolution = float(resolution)
        self.x_min = float(x_range[0])
        self.x_max = float(x_range[1])
        self.y_min = float(y_range[0])
        self.y_max = float(y_range[1])
        
        self.width = int(np.ceil((self.x_max - self.x_min) / self.resolution))
        self.height = int(np.ceil((self.y_max - self.y_min) / self.resolution))
        
        # Initialize grid with 0 (probability 0.5 = unknown)
        # Shape is (height, width) -> (y, x)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        # Observed mask: True once a cell has been traversed/updated by a ray.
        # This distinguishes "unseen unknown" from "seen but uncertain".
        self.observed = np.zeros((self.height, self.width), dtype=bool)
        
        # Cliff mask (boolean), True means cliff
        self.cliff_mask = np.zeros((self.height, self.width), dtype=bool)

        # Guard concurrent access from viewer/provider threads
        self._lock = threading.RLock()

        # Double-buffer for lock-free reads (snapshot mechanism)
        # UI thread reads from snapshots, mapping thread updates main buffers
        self._snapshot_log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self._snapshot_cliff_mask = np.zeros((self.height, self.width), dtype=bool)
        self._snapshot_observed = np.zeros((self.height, self.width), dtype=bool)
        self._snapshot_version = 0
        self._snapshot_lock = threading.Lock()  # Fast lock for snapshot swap only

        # Change notification for viewers
        self.version = 0
        self.on_update = None
        self._depth_scale_ema = 1.0

        print(f"Occupancy Grid Initialized: {self.width}x{self.height} cells, "
              f"Resolution: {self.resolution}mm, "
              f"Range: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}]")

    def world_to_grid(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """
        Convert world coordinates (mm) to grid indices (x_idx, y_idx).
        Note: Returns (col, row).
        Returns (-1, -1) if out of bounds.
        """
        if not (self.x_min <= x_mm < self.x_max and self.y_min <= y_mm < self.y_max):
            return -1, -1
            
        gx = int((x_mm - self.x_min) / self.resolution)
        gy = int((y_mm - self.y_min) / self.resolution)
        
        # Double check bounds (should be handled by if check, but for safety)
        gx = min(max(gx, 0), self.width - 1)
        gy = min(max(gy, 0), self.height - 1)
        
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        """
        Convert grid indices (x_idx, y_idx) to world coordinates (center of cell).
        """
        x_mm = self.x_min + (gx + 0.5) * self.resolution
        y_mm = self.y_min + (gy + 0.5) * self.resolution
        return x_mm, y_mm

    def is_in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_cell(self, gx: int, gy: int, state: CellState):
        """
        Update a single cell's log-odds based on observation.
        """
        with self._lock:
            if not self.is_in_bounds(gx, gy):
                return

            if state == CellState.FREE:
                self.log_odds[gy, gx] += self.LO_FREE
                self.observed[gy, gx] = True
            elif state == CellState.OCCUPIED:
                self.log_odds[gy, gx] += self.LO_OCCUPIED
                self.observed[gy, gx] = True
            
            # Clamp values
            self.log_odds[gy, gx] = max(min(self.log_odds[gy, gx], self.LO_MAX), self.LO_MIN)
        # Notify outside lock to avoid deadlocks with viewers
        self._notify_update()

    def mark_cliff(self, gx: int, gy: int, is_cliff: bool = True):
        """Mark a cell as cliff (dangerous)."""
        updated = False
        with self._lock:
            if not self.is_in_bounds(gx, gy):
                return
            self.cliff_mask[gy, gx] = is_cliff
            updated = True
        if updated:
            self._notify_update()

    def mark_cliff_polyline(self, polyline_world: list[tuple[float, float]], is_cliff: bool = True):
        """
        Rasterize a world-space polyline onto the cliff mask.
        Args:
            polyline_world: List of (x, y) points in mm.
            is_cliff: True to mark as cliff, False to clear.
        """
        updated = False
        with self._lock:
            if len(polyline_world) < 2:
                return

            for i in range(len(polyline_world) - 1):
                p1 = polyline_world[i]
                p2 = polyline_world[i+1]
                
                x1, y1 = p1
                x2, y2 = p2
                
                dist = math.hypot(x2 - x1, y2 - y1)
                if dist < 1e-3:
                    continue
                    
                # Interpolate points every 0.5 * resolution to ensure no gaps
                steps = int(np.ceil(dist / (self.resolution * 0.5)))
                
                xs = np.linspace(x1, x2, steps)
                ys = np.linspace(y1, y2, steps)
                
                for x, y in zip(xs, ys):
                    gx, gy = self.world_to_grid(x, y)
                    if gx != -1:
                        self.cliff_mask[gy, gx] = is_cliff
                        updated = True
        if updated:
            self._notify_update()

    def get_state(self, gx: int, gy: int) -> CellState:
        """Get the categorical state of a cell."""
        if not self.is_in_bounds(gx, gy):
            return CellState.UNKNOWN
            
        if self.cliff_mask[gy, gx]:
            return CellState.CLIFF
            
        val = self.log_odds[gy, gx]
        if val > self.LO_THRESH_OCC:
            return CellState.OCCUPIED
        elif val < self.LO_THRESH_FREE:
            return CellState.FREE
        else:
            return CellState.UNKNOWN

    def is_navigable(self, x_mm: float, y_mm: float) -> bool:
        """
        Check if a world position is safe for navigation.
        Returns True if FREE or UNKNOWN (optimistic), False if OCCUPIED or CLIFF.
        To be strict (only FREE), change logic.
        """
        gx, gy = self.world_to_grid(x_mm, y_mm)
        if gx == -1:
            return False # Out of bounds treated as non-navigable
            
        state = self.get_state(gx, gy)
        return state != CellState.OCCUPIED and state != CellState.CLIFF

    def check_collision_circle(self, x_mm: float, y_mm: float, radius_mm: float) -> bool:
        """
        Check if a circle centered at (x, y) collides with obstacles or cliffs.
        Returns True if collision detected.
        """
        # 1. Bounding box in grid coordinates
        r_cells = int(np.ceil(radius_mm / self.resolution))
        gx, gy = self.world_to_grid(x_mm, y_mm)
        
        if gx == -1: # Center out of bounds
            return True
            
        x0 = max(0, gx - r_cells)
        x1 = min(self.width, gx + r_cells + 1)
        y0 = max(0, gy - r_cells)
        y1 = min(self.height, gy + r_cells + 1)
        
        if x0 >= x1 or y0 >= y1:
            return False
            
        # 2. Extract subgrid
        sub_log_odds = self.log_odds[y0:y1, x0:x1]
        sub_cliff = self.cliff_mask[y0:y1, x0:x1]
        
        # 3. Check simply if any cell in box is occupied (Fast rejection)
        # If bounding box is clear, circle is definitely clear
        box_collision = np.any(sub_log_odds > self.LO_THRESH_OCC) or np.any(sub_cliff)
        if not box_collision:
            return False
            
        # 4. Detailed circle check
        # Generate coordinate grid relative to center
        # Use indices relative to (gx, gy)
        y_idx, x_idx = np.ogrid[y0:y1, x0:x1]
        dist_sq = (x_idx - gx)**2 + (y_idx - gy)**2
        mask = dist_sq <= r_cells**2
        
        # Check collision within mask
        if np.any((sub_log_odds[mask] > self.LO_THRESH_OCC) | sub_cliff[mask]):
            return True
            
        return False

    def is_area_free(self, x_mm: float, y_mm: float, radius_mm: float, ratio: float = 0.7) -> bool:
        """
        Check if an area is observed as free (ground).
        Returns True if >ratio of cells in the area are FREE.
        """
        r_cells = int(np.ceil(radius_mm / self.resolution))
        gx, gy = self.world_to_grid(x_mm, y_mm)
        
        if gx == -1:
            return False
            
        x0 = max(0, gx - r_cells)
        x1 = min(self.width, gx + r_cells + 1)
        y0 = max(0, gy - r_cells)
        y1 = min(self.height, gy + r_cells + 1)
        
        if x0 >= x1 or y0 >= y1:
            return False
            
        sub_log_odds = self.log_odds[y0:y1, x0:x1]
        
        y_idx, x_idx = np.ogrid[y0:y1, x0:x1]
        dist_sq = (x_idx - gx)**2 + (y_idx - gy)**2
        mask = dist_sq <= r_cells**2
        
        total_cells = np.count_nonzero(mask)
        if total_cells == 0:
            return False
            
        free_cells = np.count_nonzero((sub_log_odds[mask] < self.LO_THRESH_FREE))
        
        return (free_cells / total_cells) > ratio

    def clear(self):
        """Reset the grid to unknown state."""
        with self._lock:
            self.log_odds.fill(0)
            self.observed.fill(False)
            self.cliff_mask.fill(False)
        self._notify_update()

    def update_from_depth(self, 
                          depth_map: np.ndarray, 
                          intrinsics: CameraIntrinsics, 
                          extrinsics: np.ndarray, 
                          robot_pose: Tuple[float, float, float],
                          stride: int = 8,
                          *,
                          sampling: str = DEFAULT_SAMPLING,
                          bearing_bins: int = DEFAULT_BEARING_BINS,
                          v_fractions: Tuple[float, ...] = DEFAULT_V_FRACTIONS,
                          splat_radius_occ: int = DEFAULT_SPLAT_RADIUS_OCC,
                          splat_radius_free: int = DEFAULT_SPLAT_RADIUS_FREE):
        """Thread-safe wrapper for grid updates."""
        with self._lock:
            self._update_from_depth_unlocked(
                depth_map,
                intrinsics,
                extrinsics,
                robot_pose,
                stride,
                sampling=sampling,
                bearing_bins=bearing_bins,
                v_fractions=v_fractions,
                splat_radius_occ=splat_radius_occ,
                splat_radius_free=splat_radius_free,
            )
        self._notify_update()

    def _update_from_depth_unlocked(self, 
                          depth_map: np.ndarray, 
                          intrinsics: CameraIntrinsics, 
                          extrinsics: np.ndarray, 
                          robot_pose: Tuple[float, float, float],
                          stride: int = 8,
                          *,
                          sampling: str = DEFAULT_SAMPLING,
                          bearing_bins: int = DEFAULT_BEARING_BINS,
                          v_fractions: Tuple[float, ...] = DEFAULT_V_FRACTIONS,
                          splat_radius_occ: int = DEFAULT_SPLAT_RADIUS_OCC,
                          splat_radius_free: int = DEFAULT_SPLAT_RADIUS_FREE):
        """
        Update grid based on depth map observation.
        
        Args:
            depth_map: HxW array of depth values (meters, Z-depth).
            intrinsics: Camera parameters.
            extrinsics: 4x4 matrix (Camera to Base Frame).
            robot_pose: (x, y, theta) of robot in World Frame.
            stride: Downsampling factor for performance (default 8).
            sampling: Pixel sampling strategy: "bearing_bins" or "stride".
            bearing_bins: Number of horizontal bins (bearing_bins * len(v_fractions) rays).
            v_fractions: Vertical sample positions (fractions of image height).
            splat_radius_occ: Obstacle endpoint splat radius (cells).
            splat_radius_free: Ground endpoint splat radius (cells).
        """
            # === DEBUG: print depth statistics ===
        valid_depth = depth_map[(depth_map > 0) & np.isfinite(depth_map)]
        if valid_depth.size > 0:
            print(f"[DEBUG] depth_map stats: min={valid_depth.min():.4f}, "
                f"max={valid_depth.max():.4f}, mean={valid_depth.mean():.4f}, "
                f"median={np.median(valid_depth):.4f}")
    # === END DEBUG ===
        height, width = depth_map.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.cx, intrinsics.cy
        
    # ============================================================
        # GPR (Ground Plane Reference) depth correction
        # Dynamically estimate scale and shift using the bottom image region
        # Model: d_pred = scale * (1/Z_geo) + shift
        # ============================================================
        DISPARITY_MIN = 0.1
        DISPARITY_MAX = 20.0
        
        # Keep original disparity for gradient computation and calibration
        disparity_original = depth_map.copy()
        disparity_clipped = np.clip(depth_map, DISPARITY_MIN, DISPARITY_MAX)
        
        # GPR correction
        R_cb = extrinsics[:3, :3]
        t_cb = extrinsics[:3, 3]
        cam_height = abs(t_cb[2])
        
        gpr_scale, gpr_shift = self._calibrate_gpr(
            disparity_clipped, intrinsics, R_cb, t_cb, cam_height
        )
        
        if gpr_scale is not None:
            depth_map = gpr_scale / np.maximum(disparity_clipped - gpr_shift, 0.1)
            print(f"[DEBUG] GPR: scale={gpr_scale:.3f}, shift={gpr_shift:.3f}")
        else:
            # Use a conservative default value
            depth_map = 0.5 / disparity_clipped
            print(f"[DEBUG] GPR failed, using default scale=0.5")
        # ============================================================
        
        # ============================================================
        # Depth gradient filter: detect depth-unstable regions like cliff edges
        # ============================================================
        def compute_local_depth_gradient(depth, u_coords, v_coords, window=5):
            """Compute local depth gradient at samples (max-min)."""
            h, w = depth.shape
            half_w = window // 2
            n = len(u_coords)
            gradients = np.zeros(n, dtype=np.float32)

            for i in range(n):
                u, v = int(u_coords[i]), int(v_coords[i])
                v0, v1 = max(0, v - half_w), min(h, v + half_w + 1)
                u0, u1 = max(0, u - half_w), min(w, u + half_w + 1)
                patch = depth[v0:v1, u0:u1]

                if patch.size > 1:
                    valid_patch = patch[patch > 0.01]  # Ignore invalid depth
                    if valid_patch.size > 1:
                        gradients[i] = valid_patch.max() - valid_patch.min()

            return gradients

        # ------------------------------------------------------------------
        # 1) Select a fixed budget of rays (pixel samples)

        if sampling == "stride":
            v_coords, u_coords = np.mgrid[0:height:stride, 0:width:stride]
            u_flat = u_coords.reshape(-1)
            v_flat = v_coords.reshape(-1)
        else:
            # "bearing_bins": uniform horizontal bearings + ground-focused rows.
            # Choose a few rows in the lower part of the image (more likely ground).
            v_samples = np.rint((height - 1) * np.asarray(v_fractions, dtype=np.float32)).astype(int)
            v_samples = np.clip(v_samples, 0, height - 1)
            v_samples = np.unique(v_samples)
            if len(v_samples) == 0:
                return

            bins = int(bearing_bins)
            if bins <= 0:
                return
            bins = min(bins, width)

            # Horizontal bearing range from left to right edge.
            phi_min = math.atan((0.0 - cx) / fx)
            phi_max = math.atan(((width - 1) - cx) / fx)
            phis = phi_min + (np.arange(bins, dtype=np.float32) + 0.5) * (phi_max - phi_min) / bins
            u_samples = np.rint(fx * np.tan(phis) + cx).astype(int)
            u_samples = np.clip(u_samples, 0, width - 1)
            u_samples = np.unique(u_samples)
            if len(u_samples) == 0:
                return

            u_grid, v_grid = np.meshgrid(u_samples, v_samples, indexing="xy")
            u_flat = u_grid.reshape(-1)
            v_flat = v_grid.reshape(-1)

        z_obs = depth_map[v_flat, u_flat]

        # Compute gradients on original disparity (better reflects depth jumps)
        depth_gradients_all = compute_local_depth_gradient(disparity_original, u_flat, v_flat, window=5)

        valid_mask = (z_obs > 0.03) & (z_obs < 10.0) & np.isfinite(z_obs)
        if not np.any(valid_mask):
            self.last_update_counts = {
                "ground": 0,
                "obstacle": 0,
                "valid": 0,
                "ignored": 0,
                "scale_ratio": 1.0,
                "scale_inliers": 0,
            }
            _LOGGER.warning("occupancy_grid: no valid depth samples this frame")
            return

        u_valid = u_flat[valid_mask]
        v_valid = v_flat[valid_mask]
        z_obs_valid = z_obs[valid_mask]
        depth_gradients_valid = depth_gradients_all[valid_mask]

        # 2. Calculate expected ground depth (geometry only)
        R_cb = extrinsics[:3, :3]
        t_cb = extrinsics[:3, 3]

        x_norm = (u_valid - cx) / fx
        y_norm = (v_valid - cy) / fy
        Ray_cam = np.stack([x_norm, y_norm, np.ones_like(x_norm)], axis=1)
        Ray_base = (R_cb @ Ray_cam.T).T  # Direction in base frame

        Cam_origin_base = t_cb  # Camera position in base frame

        with np.errstate(divide='ignore', invalid='ignore'):
            t_ground = -Cam_origin_base[2] / Ray_base[:, 2]

        valid_ground_proj = (t_ground > 0) & (Ray_base[:, 2] < -0.1)
        z_expected = t_ground  # Expected Z-depth in camera frame (meters)

        # Optional scale correction (robust median + EMA).
        scale_ratio = 1.0
        scale_inliers = 0
        scale_mode = "none"
        if self.SCALE_CORRECTION_ENABLED and np.any(valid_ground_proj):
            ratio = t_ground[valid_ground_proj] / z_obs_valid[valid_ground_proj]
            ratio = ratio[np.isfinite(ratio)]
            ratio = ratio[(ratio > self.SCALE_RATIO_MIN) & (ratio < self.SCALE_RATIO_MAX)]
            if ratio.size >= self.SCALE_MIN_INLIERS:
                median_ratio = np.median(ratio)
                inliers = np.abs(ratio - median_ratio) <= self.SCALE_INLIER_BAND
                inlier_count = int(np.count_nonzero(inliers))
                if inlier_count >= self.SCALE_MIN_INLIERS:
                    candidate = float(np.median(ratio[inliers]))
                    alpha = float(self.SCALE_EMA_ALPHA)
                    self._depth_scale_ema = (1.0 - alpha) * self._depth_scale_ema + alpha * candidate
                    scale_ratio = self._depth_scale_ema
                    z_obs_valid = z_obs_valid * scale_ratio
                    scale_inliers = inlier_count
                    scale_mode = "raw"

            if scale_inliers == 0 and self.INVERT_DEPTH_ON_FAIL:
                z_inv = 1.0 / np.clip(z_obs_valid, 1e-3, None)
                ratio = t_ground[valid_ground_proj] / z_inv[valid_ground_proj]
                ratio = ratio[np.isfinite(ratio)]
                ratio = ratio[(ratio > self.SCALE_RATIO_MIN) & (ratio < self.SCALE_RATIO_MAX)]
                if ratio.size >= self.SCALE_MIN_INLIERS:
                    median_ratio = np.median(ratio)
                    inliers = np.abs(ratio - median_ratio) <= self.SCALE_INLIER_BAND
                    inlier_count = int(np.count_nonzero(inliers))
                    if inlier_count >= self.SCALE_MIN_INLIERS:
                        candidate = float(np.median(ratio[inliers]))
                        alpha = float(self.SCALE_EMA_ALPHA)
                        self._depth_scale_ema = (1.0 - alpha) * self._depth_scale_ema + alpha * candidate
                        scale_ratio = self._depth_scale_ema
                        z_obs_valid = z_inv * scale_ratio
                        scale_inliers = inlier_count
                        scale_mode = "inv"

        # 3. Back-project to Camera Frame (use corrected depth if applied)
        x_cam = (u_valid - cx) * z_obs_valid / fx
        y_cam = (v_valid - cy) * z_obs_valid / fy
        P_cam = np.stack([x_cam, y_cam, z_obs_valid], axis=1)

        # 4. Transform to Base Frame
        P_base = (R_cb @ P_cam.T).T + t_cb
        
        # ------------------------------------------------------------------
        # 5) Classification (ground vs obstacle vs ignore)

        height_above_ground = P_base[:, 2]
        distance_xy = np.sqrt(P_base[:, 0]**2 + P_base[:, 1]**2)

        # === Depth consistency analysis ===
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_ratio = z_obs_valid / np.maximum(z_expected, 0.01)

        # Depth consistency thresholds (temporarily relaxed; tighten after GPR works)
        DEPTH_RATIO_MIN = 0.5
        DEPTH_RATIO_MAX = 2.0

        is_depth_consistent = (
            (depth_ratio >= DEPTH_RATIO_MIN) &
            (depth_ratio <= DEPTH_RATIO_MAX) &
            np.isfinite(depth_ratio) &
            valid_ground_proj
        )

        # Closer than expected = may hit obstacle surface
        is_closer_than_expected = (depth_ratio < DEPTH_RATIO_MIN) & np.isfinite(depth_ratio) & valid_ground_proj

        # === Reliability filtering ===
        RELIABLE_HEIGHT_MIN = -0.05   # Must not be more than 5cm below ground
        RELIABLE_HEIGHT_MAX = 0.40    # Must not exceed 40cm (typical tabletop obstacle range)
        MAX_OBSTACLE_DISTANCE = 0.80  # Only care about points within 80cm
        DEPTH_GRADIENT_THRESHOLD = 1.5  # Disparity local jump threshold; needs tuning

        is_stable_depth = depth_gradients_valid < DEPTH_GRADIENT_THRESHOLD

        is_reliable = (
            (height_above_ground >= RELIABLE_HEIGHT_MIN) &
            (height_above_ground <= RELIABLE_HEIGHT_MAX) &
            (distance_xy < MAX_OBSTACLE_DISTANCE) &
            is_stable_depth  # New: filter depth-unstable regions
        )

        # === Classification ===
        # Height thresholds (temporarily relaxed)
        GROUND_HEIGHT_THRESHOLD = 0.020   # 20mm
        OBSTACLE_HEIGHT_THRESHOLD = 0.010  # 10mm, slightly lower to detect more obstacles

        # Ground: must satisfy multiple strict conditions
        is_ground = (
            is_reliable &
            valid_ground_proj &
            is_depth_consistent &
            (np.abs(height_above_ground) <= GROUND_HEIGHT_THRESHOLD)
        )

        # Obstacle:
        # 1. Height clearly above ground
        # 2. Closer than expected (hits obstacle surface, e.g., a laptop)
        is_obstacle = is_reliable & (
            (height_above_ground > OBSTACLE_HEIGHT_THRESHOLD) |
            is_closer_than_expected  # Change: only closer-than-expected counts as obstacle
        )

        # Debug
        print(f"[DEBUG] depth_ratio: min={depth_ratio[np.isfinite(depth_ratio)].min():.3f}, "
              f"max={depth_ratio[np.isfinite(depth_ratio)].max():.3f}, "
              f"median={np.median(depth_ratio[np.isfinite(depth_ratio)]):.3f}")
        print(f"[DEBUG] height: min={height_above_ground.min():.4f}, "
              f"max={height_above_ground.max():.4f}, median={np.median(height_above_ground):.4f}")
        print(f"[DEBUG] stable_depth: {np.count_nonzero(is_stable_depth)}/{len(is_stable_depth)}")
        print(f"[DEBUG] depth_consistent: {np.count_nonzero(is_depth_consistent)}/{len(is_depth_consistent)}")
        print(f"[DEBUG] reliable: {np.count_nonzero(is_reliable)}/{len(is_reliable)}")
        print(f"[DEBUG] is_ground: {np.count_nonzero(is_ground)}, is_obstacle: {np.count_nonzero(is_obstacle)}")
        # Debug - view distance distribution of obstacle points
        if np.any(is_obstacle):
            obs_distances = distance_xy[is_obstacle]
            obs_heights = height_above_ground[is_obstacle]
            print(f"[DEBUG] obstacle distance: min={obs_distances.min():.4f}, "
                  f"max={obs_distances.max():.4f}, median={np.median(obs_distances):.4f}")
            print(f"[DEBUG] obstacle height: min={obs_heights.min():.4f}, "
                  f"max={obs_heights.max():.4f}, median={np.median(obs_heights):.4f}")

        valid_count = int(z_obs_valid.size)
        ground_raw = int(np.count_nonzero(is_ground))
        obstacle_raw = int(np.count_nonzero(is_obstacle))
        ignored_count = max(0, valid_count - ground_raw - obstacle_raw)
        
        # 6. Prepare updates in World Frame
        rx, ry, rtheta = robot_pose
        c_theta = np.cos(rtheta)
        s_theta = np.sin(rtheta)
        
        # P_base contains the measured 3D points in base frame
        # For ground points, we prefer using the P_ground_base (perfect plane) to clear noise
        # P_ground_base = Cam + t * Ray
        P_ground_base_x = Cam_origin_base[0] + t_ground[is_ground] * Ray_base[is_ground, 0]
        P_ground_base_y = Cam_origin_base[1] + t_ground[is_ground] * Ray_base[is_ground, 1]
        
        # Transform to World (meters → millimeters before mixing with robot pose)
        P_ground_base_x_mm = P_ground_base_x * 1000.0
        P_ground_base_y_mm = P_ground_base_y * 1000.0

        # x_w = x_b * cos - y_b * sin + rx
        # y_w = x_b * sin + y_b * cos + ry
        ground_x_w = P_ground_base_x_mm * c_theta - P_ground_base_y_mm * s_theta + rx
        ground_y_w = P_ground_base_x_mm * s_theta + P_ground_base_y_mm * c_theta + ry
        
        # For obstacles, use measured points
        obs_x_b = P_base[is_obstacle, 0] * 1000.0
        obs_y_b = P_base[is_obstacle, 1] * 1000.0
        
        obs_x_w = obs_x_b * c_theta - obs_y_b * s_theta + rx
        obs_y_w = obs_x_b * s_theta + obs_y_b * c_theta + ry
        
        # ------------------------------------------------------------------
        # 7) Ray carving + endpoint splat (inverse sensor model)

        # Ray origin: camera position projected onto the ground plane (world XY).
        cam_x_b_mm = float(Cam_origin_base[0]) * 1000.0
        cam_y_b_mm = float(Cam_origin_base[1]) * 1000.0
        cam_x_w = cam_x_b_mm * c_theta - cam_y_b_mm * s_theta + rx
        cam_y_w = cam_x_b_mm * s_theta + cam_y_b_mm * c_theta + ry
        gx_start, gy_start = self.world_to_grid(cam_x_w, cam_y_w)

        if gx_start == -1:
            # Camera origin out of bounds; fall back to robot base position.
            gx_start, gy_start = self.world_to_grid(rx, ry)

        gxs_ground = np.array([], dtype=int)
        gys_ground = np.array([], dtype=int)
        gxs_obs = np.array([], dtype=int)
        gys_obs = np.array([], dtype=int)

        if gx_start != -1:
            gxs_ground, gys_ground = self._world_to_grid_vec(ground_x_w, ground_y_w)
            gxs_obs, gys_obs = self._world_to_grid_vec(obs_x_w, obs_y_w)

            if len(gxs_ground):
                for gx1, gy1 in zip(gxs_ground.tolist(), gys_ground.tolist()):
                    self._carve_line(gx_start, gy_start, gx1, gy1, include_end=True)
                    if splat_radius_free > 0:
                        self._apply_splat(gx1, gy1, delta=self.LO_FREE, radius=int(splat_radius_free))

            if len(gxs_obs):
                for gx1, gy1 in zip(gxs_obs.tolist(), gys_obs.tolist()):
                    self._carve_line(gx_start, gy_start, gx1, gy1, include_end=False)
                    if splat_radius_occ > 0:
                        self._apply_splat(gx1, gy1, delta=self.LO_OCCUPIED, radius=int(splat_radius_occ))
                    else:
                        if self.is_in_bounds(gx1, gy1):
                            self.log_odds[gy1, gx1] += self.LO_OCCUPIED
                            self.observed[gy1, gx1] = True

        # Clamp globally (faster than per-cell)
        np.clip(self.log_odds, self.LO_MIN, self.LO_MAX, out=self.log_odds)

        # Expose quick stats for debugging/telemetry
        self.last_update_counts = {
            "ground": int(len(gxs_ground)),
            "obstacle": int(len(gxs_obs)),
            "valid": valid_count,
            "ignored": ignored_count,
            "ground_raw": ground_raw,
            "obstacle_raw": obstacle_raw,
            "scale_ratio": float(scale_ratio),
            "scale_inliers": int(scale_inliers),
            "scale_mode": scale_mode,
            "ground_proj": int(np.count_nonzero(valid_ground_proj)),
        }
        # ✅ FIX Bug #2: Remove duplicate notification (wrapper calls it)

    def _notify_update(self):
        """Update version and atomically swap snapshot for viewers."""
        callback = None
        with self._lock:
            self.version += 1
            callback = self.on_update

            # ✅ FIX Bug #1: Copy snapshot WHILE holding main lock
            # This ensures log_odds/cliff_mask cannot change during copy
            with self._snapshot_lock:
                np.copyto(self._snapshot_log_odds, self.log_odds)
                np.copyto(self._snapshot_cliff_mask, self.cliff_mask)
                np.copyto(self._snapshot_observed, self.observed)
                self._snapshot_version = self.version

        # Call callback outside locks to avoid deadlock with viewer
        if callback:
            try:
                callback()
            except Exception:
                pass

    def get_snapshot(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        """
        Get thread-safe snapshot of grid data WITHOUT holding main lock.

        This method enables lock-free reads for UI rendering. The snapshot
        is updated after each grid update via _notify_update().

        Returns:
            Tuple of (log_odds_copy, cliff_mask_copy, observed_copy, version)
        """
        with self._snapshot_lock:
            log_odds = self._snapshot_log_odds.copy()
            cliff_mask = self._snapshot_cliff_mask.copy()
            observed = self._snapshot_observed.copy()
            version = self._snapshot_version
        return log_odds, cliff_mask, observed, version

    def _world_to_grid_vec(self, x_mm: np.ndarray, y_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized world->grid conversion with bounds filtering."""
        if x_mm.size == 0:
            return np.array([], dtype=int), np.array([], dtype=int)
        gxs = ((x_mm - self.x_min) / self.resolution).astype(int)
        gys = ((y_mm - self.y_min) / self.resolution).astype(int)
        valid = (gxs >= 0) & (gxs < self.width) & (gys >= 0) & (gys < self.height)
        return gxs[valid], gys[valid]

    def _apply_splat(self, gx: int, gy: int, *, delta: float, radius: int) -> None:
        """Apply a small disk kernel update around (gx, gy)."""
        if radius <= 0:
            if self.is_in_bounds(gx, gy):
                self.log_odds[gy, gx] += float(delta)
                self.observed[gy, gx] = True
            return

        key = int(radius)
        cache = getattr(self, "_splat_cache", None)
        if cache is None:
            cache = {}
            setattr(self, "_splat_cache", cache)
        kernel = cache.get(key)
        if kernel is None:
            offsets = []
            weights = []
            sigma = max(0.5, key / 2.0)
            for dy in range(-key, key + 1):
                for dx in range(-key, key + 1):
                    if dx * dx + dy * dy <= key * key:
                        offsets.append((dx, dy))
                        w = math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
                        weights.append(w)
            dxs = np.array([o[0] for o in offsets], dtype=int)
            dys = np.array([o[1] for o in offsets], dtype=int)
            wts = np.array(weights, dtype=np.float32)
            # Normalize peak to 1.0 at center (center exists in kernel).
            wts /= float(wts.max() if wts.size else 1.0)
            kernel = (dxs, dys, wts)
            cache[key] = kernel

        dxs, dys, wts = kernel
        xs = gx + dxs
        ys = gy + dys
        inb = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
        if not np.any(inb):
            return
        xs = xs[inb]
        ys = ys[inb]
        wts = wts[inb]
        self.log_odds[ys, xs] += (float(delta) * wts)
        self.observed[ys, xs] = True

    def _carve_line(self, x0: int, y0: int, x1: int, y1: int, *, include_end: bool) -> None:
        """Bresenham line; applies LO_FREE along the segment."""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            if not include_end and x == x1 and y == y1:
                break
            if self.is_in_bounds(x, y):
                self.log_odds[y, x] += self.LO_FREE
                self.observed[y, x] = True
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _calibrate_gpr(self, disparity_map: np.ndarray, intrinsics, 
                       R_cb: np.ndarray, t_cb: np.ndarray, cam_height: float
                       ) -> Tuple[Optional[float], Optional[float]]:
        """
        Ground Plane Reference (GPR) correction - stable version
        """
        height, width = disparity_map.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.cx, intrinsics.cy
        
        # 1. Select the middle region of the bottom 20% (more conservative, less obstacle contamination)
        roi_top = int(height * 0.80)
        roi_left = int(width * 0.25)
        roi_right = int(width * 0.75)
        
        v_roi = np.arange(roi_top, height, 2)
        u_roi = np.arange(roi_left, roi_right, 4)
        u_grid, v_grid = np.meshgrid(u_roi, v_roi)
        u_flat = u_grid.flatten()
        v_flat = v_grid.flatten()
        
        # 2. Get disparity at sample points
        d_samples = disparity_map[v_flat, u_flat]
        valid = (d_samples > 0.5) & (d_samples < 12.0) & np.isfinite(d_samples)
        
        if np.sum(valid) < 50:
            return self._get_smoothed_gpr(None, None)
        
        u_valid = u_flat[valid]
        v_valid = v_flat[valid]
        d_valid = d_samples[valid]
        
        # 3. Compute geometric expected depth Z_geo
        x_norm = (u_valid - cx) / fx
        y_norm = (v_valid - cy) / fy
        Ray_cam = np.stack([x_norm, y_norm, np.ones_like(x_norm)], axis=1)
        Ray_base = (R_cb @ Ray_cam.T).T
        
        with np.errstate(divide='ignore', invalid='ignore'):
            z_geo = cam_height / (-Ray_base[:, 2])
        
        valid_proj = (z_geo > 0.02) & (z_geo < 0.5) & (Ray_base[:, 2] < -0.1) & np.isfinite(z_geo)
        
        if np.sum(valid_proj) < 30:
            return self._get_smoothed_gpr(None, None)
        
        z_geo = z_geo[valid_proj]
        d_geo = d_valid[valid_proj]
        inv_z = 1.0 / z_geo
        
        # 4. RANSAC linear regression (reject outliers)
        try:
            # Simple RANSAC implementation
            best_scale, best_shift, best_inliers = None, None, 0
            n_samples = len(inv_z)
            
            for _ in range(50):  # 50 iterations
                # Randomly choose 2 points
                idx = np.random.choice(n_samples, size=2, replace=False)
                x1, x2 = inv_z[idx]
                y1, y2 = d_geo[idx]
                
                if abs(x2 - x1) < 1e-6:
                    continue
                
                # Fit line d = scale * (1/Z) + shift
                scale = (y2 - y1) / (x2 - x1)
                shift = y1 - scale * x1
                
                # Compute residuals for all points
                d_pred = scale * inv_z + shift
                residuals = np.abs(d_geo - d_pred)
                inliers = residuals < 0.5  # Residual threshold
                n_inliers = np.sum(inliers)
                
                if n_inliers > best_inliers:
                    best_scale = scale
                    best_shift = shift
                    best_inliers = n_inliers
            
            if best_inliers < 20:
                return self._get_smoothed_gpr(None, None)
            
            scale, shift = best_scale, best_shift
            
            # Refit using inliers
            inlier_mask = np.abs(d_geo - (scale * inv_z + shift)) < 0.5
            if np.sum(inlier_mask) >= 20:
                A = np.vstack([inv_z[inlier_mask], np.ones(np.sum(inlier_mask))]).T
                result = np.linalg.lstsq(A, d_geo[inlier_mask], rcond=None)
                scale, shift = float(result[0][0]), float(result[0][1])
            
            # 5. Sanity check (key: shift cannot be positive or too negative)
            if scale < 0.2 or scale > 1.0:
                print(f"[GPR DEBUG] scale={scale:.3f} out of range [0.2, 1.0], rejecting")
                return self._get_smoothed_gpr(None, None)
            
            if shift > 0.5:  # Shift should not be positive
                print(f"[GPR DEBUG] shift={shift:.3f} is positive, rejecting")
                return self._get_smoothed_gpr(None, None)
            
            if shift < -5.0:  # Shift should not be too negative
                print(f"[GPR DEBUG] shift={shift:.3f} too negative, rejecting")
                return self._get_smoothed_gpr(None, None)
            
            print(f"[GPR DEBUG] raw fit: scale={scale:.4f}, shift={shift:.4f}, inliers={best_inliers}")
            return self._get_smoothed_gpr(scale, shift)
            
        except Exception as e:
            print(f"[GPR DEBUG] regression failed: {e}")
            return self._get_smoothed_gpr(None, None)
    
    def _get_smoothed_gpr(self, scale: Optional[float], shift: Optional[float]
                          ) -> Tuple[Optional[float], Optional[float]]:
        """
        Use EMA to smooth GPR parameters and return history when current frame fails.
        """
        # Initialize EMA state (if missing)
        if not hasattr(self, '_gpr_scale_ema'):
            self._gpr_scale_ema = None
            self._gpr_shift_ema = None
        
        if scale is None or shift is None:
            # Current frame calibration failed; return history if available
            if self._gpr_scale_ema is not None:
                print(f"[GPR DEBUG] using cached: scale={self._gpr_scale_ema:.4f}, shift={self._gpr_shift_ema:.4f}")
                return self._gpr_scale_ema, self._gpr_shift_ema
            else:
                return None, None
        
        # Update EMA
        alpha = 0.3  # Smoothing factor
        if self._gpr_scale_ema is None:
            self._gpr_scale_ema = scale
            self._gpr_shift_ema = shift
        else:
            self._gpr_scale_ema = (1 - alpha) * self._gpr_scale_ema + alpha * scale
            self._gpr_shift_ema = (1 - alpha) * self._gpr_shift_ema + alpha * shift
        
        print(f"[GPR DEBUG] smoothed: scale={self._gpr_scale_ema:.4f}, shift={self._gpr_shift_ema:.4f}")
        return self._gpr_scale_ema, self._gpr_shift_ema
