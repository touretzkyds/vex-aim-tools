"""Occupancy grid mapping behavior for VEX AIM robot.

This module provides FSM behaviors for incrementally building and updating
the occupancy grid from camera observations as the robot moves.

The UpdateOccupancyGrid behavior can be used:
1. In parallel with Wander() for autonomous exploration and mapping
2. As a standalone behavior for stationary 360-degree scanning
3. Combined with other behaviors that require continuous mapping

Usage examples:
    # Autonomous exploration with mapping
    Parallel(Wander(), UpdateOccupancyGrid())

    # Stationary scan (rotate in place while mapping)
    Sequential(Turn(360), UpdateOccupancyGrid(duration=10.0))

    # Continuous mapping during navigation
    Parallel(GoToGoal(x, y), UpdateOccupancyGrid())
"""

from __future__ import annotations

import concurrent.futures
import logging
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .base import StateNode

try:
    from perception import CliffDetector, DepthAnythingProvider, load_cliff_calibration
    PERCEPTION_AVAILABLE = True
except ImportError:
    PERCEPTION_AVAILABLE = False

_LOGGER = logging.getLogger(__name__)


class UpdateOccupancyGrid(StateNode):
    """FSM behavior that continuously updates the occupancy grid from camera.

    This behavior captures camera frames and updates the occupancy grid with:
    - Free space (ground) detection from depth maps
    - Occupied space (obstacles) from depth discontinuities
    - Cliff edges from cliff detection results

    The behavior runs continuously until stopped or duration expires.
    It only updates the grid when the robot is localized.

    Args:
        frame_interval: Minimum seconds between grid updates (default: 0.5)
        duration: Maximum runtime in seconds. None = run forever (default: None)
        update_cliffs: If True, also mark cliff edges on grid (default: True)
    """

    def __init__(
        self,
        frame_interval: float = 0.5,
        duration: Optional[float] = None,
        update_cliffs: bool = True
    ):
        super().__init__()
        self.frame_interval = frame_interval
        self.duration = duration
        self.update_cliffs = update_cliffs

        # State
        self.detector: Optional[CliffDetector] = None
        self.last_update_time = 0.0
        self.start_time = None
        self.frame_count = 0
        self._last_frame_id: Optional[int] = None

        # Background processing (avoid blocking the asyncio event loop thread).
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._processing_future: Optional[concurrent.futures.Future] = None
        self._stop_requested = False

    def start(self, event=None):
        """Start mapping updates.

        Important: heavy model initialization should happen outside the asyncio
        event loop thread when possible (e.g. pre-init in the CLI thread and
        assign to ``self.detector`` before calling ``start()``).
        """
        if self.running:
            return self.running

        super().start(event)

        self._stop_requested = False
        self._processing_future = None
        self._last_frame_id = None

        if not PERCEPTION_AVAILABLE:
            _LOGGER.error("Perception modules not available - cannot update occupancy grid")
            self.post_failure(details="perception_module_missing")
            return self.running

        if not hasattr(self.robot.world_map, "occupancy_grid") or self.robot.world_map.occupancy_grid is None:
            _LOGGER.warning("No occupancy grid initialized - UpdateOccupancyGrid has nothing to do")
            self.post_failure(details="grid_missing")
            return self.running

        # Create worker thread (single worker to avoid detector concurrency).
        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        # Initialize depth provider and cliff detector (if not pre-initialized).
        try:
            if self.detector is None:
                _LOGGER.info("UpdateOccupancyGrid: initializing detector (may take a while)...")

                provider_mode = os.environ.get("DEPTHANYTHING_PROVIDER", "dummy").lower()
                if provider_mode == "dummy":
                    _LOGGER.warning("Using dummy DepthAnything provider (no real detection)")
                    provider = DepthAnythingProvider.build_dummy()
                elif provider_mode == "torch":
                    weights_env = os.environ.get("DEPTHANYTHING_WEIGHTS")
                    if not weights_env:
                        raise RuntimeError("DEPTHANYTHING_WEIGHTS must be set for torch provider")

                    provider = DepthAnythingProvider.from_torch(
                        weights_path=Path(weights_env),
                        model_type=os.environ.get("DEPTHANYTHING_MODEL", "depthanything-v2-small"),
                        device=os.environ.get("DEPTHANYTHING_DEVICE", "cpu"),
                    )
                else:
                    raise RuntimeError(f"Unsupported DEPTHANYTHING_PROVIDER={provider_mode}")

                camera = getattr(self.robot, "camera", None)
                calibration = load_cliff_calibration(camera)
                self.detector = CliffDetector(
                    depth_provider=provider,
                    intrinsics=calibration.intrinsics,
                    gravity_camera=calibration.gravity_camera,
                    extrinsics=calibration.camera_to_base,
                )

            self.start_time = time.time()
            self.last_update_time = 0.0
            self.frame_count = 0
            # Poll at a steady cadence; frame_interval gates work scheduling.
            poll_interval = min(0.1, max(0.01, float(self.frame_interval)))
            self.set_polling_interval(poll_interval)

            _LOGGER.info(f"UpdateOccupancyGrid started (interval={self.frame_interval}s, "
                        f"duration={self.duration}s, update_cliffs={self.update_cliffs})")
            print("[UpdateOccupancyGrid] started, polling enabled")
            return self.running

        except Exception as e:
            _LOGGER.error(f"Failed to initialize UpdateOccupancyGrid: {e}")
            import traceback
            traceback.print_exc()
            self.post_failure(details=f"init_failed: {e}")
            self._shutdown_worker()
            return self.running

    def poll(self):
        """Poll for new camera frames and update grid when interval expires."""
        if self._stop_requested:
            return self.running

        now = time.time()

        # If a worker task is running, avoid blocking the event loop.
        if self._processing_future is not None:
            if not self._processing_future.done():
                return self.running

            success = False
            ground = obstacle = seg_count = 0
            elapsed_s = 0.0
            error_msg: Optional[str] = None
            try:
                success, ground, obstacle, seg_count, elapsed_s, error_msg = self._processing_future.result(timeout=0.1)
            except Exception as exc:
                error_msg = f"worker_error: {exc}"
            finally:
                self._processing_future = None

            if error_msg:
                _LOGGER.error("UpdateOccupancyGrid worker failed: %s", error_msg)
            elif success:
                self.frame_count += 1
                msg = f"[UpdateOccupancyGrid] frame {self.frame_count}: ground={ground} obstacle={obstacle} cliffs={seg_count} ({elapsed_s:.2f}s)"
                print(msg)
            else:
                _LOGGER.debug("Grid update skipped/failed (success=False)")

        # Check duration limit (only complete once any in-flight work is done).
        if self.duration is not None and self.start_time is not None:
            elapsed = now - self.start_time
            if elapsed >= self.duration and self._processing_future is None:
                _LOGGER.info(f"UpdateOccupancyGrid completed: {self.frame_count} frames in {elapsed:.1f}s")
                self.set_polling_interval(None)
                # Emit both for compatibility with compositions that watch CompletionEvent.
                self.post_success(details={"frames": self.frame_count, "elapsed_s": elapsed})
                self.post_completion()
                self._shutdown_worker()
                return self.running

        # If duration has elapsed, do not start new work; wait for completion path above.
        if self.duration is not None and self.start_time is not None:
            if (now - self.start_time) >= self.duration:
                return self.running

        # Check if enough time has passed since last scheduled update.
        if now - self.last_update_time < self.frame_interval:
            return self.running

        # Get camera image
        image = getattr(self.robot, "camera_image", None)
        if image is None:
            return self.running  # Wait for camera
        frame_id = getattr(self.robot, "frame_count", None)
        if frame_id is not None and frame_id == self._last_frame_id:
            return self.running

        # Check if robot is localized (WorldMap handles this internally)
        pf = getattr(self.robot, "particle_filter", None)
        if not pf or pf.state != pf.LOCALIZED:
            return self.running  # Don't update when not localized

        if self.detector is None:
            _LOGGER.error("UpdateOccupancyGrid detector is not initialized")
            return self.running

        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        # Snapshot pose at capture time to keep mapping consistent even if pose changes mid-processing.
        pose = (float(self.robot.pose.x), float(self.robot.pose.y), float(self.robot.pose.theta))
        frame = np.array(image)

        # Record scheduling time + frame id (avoid reprocessing same camera image).
        self.last_update_time = now
        self._last_frame_id = frame_id

        self._processing_future = self._executor.submit(self._process_frame_worker, frame, pose)

        return self.running

    def stop(self):
        self._stop_requested = True
        self._shutdown_worker()
        super().stop()

    def _shutdown_worker(self) -> None:
        future = self._processing_future
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass
        self._processing_future = None

        executor = self._executor
        if executor is not None:
            try:
                executor.shutdown(wait=False)
            except Exception:
                pass
        self._executor = None

    def _process_frame_worker(self, frame: np.ndarray, pose: tuple[float, float, float]):
        """Run depth + grid update off the asyncio event loop thread."""
        if self._stop_requested:
            return (False, 0, 0, 0, 0.0, "stopped")
        detector = self.detector
        if detector is None:
            return (False, 0, 0, 0, 0.0, "detector_not_initialized")

        start = time.perf_counter()
        try:
            result = detector.process(frame)
            depth_map = result.depth.depth
            scale_hint = getattr(result.depth, "scale_hint", None)
            if scale_hint is not None and np.isfinite(scale_hint) and float(scale_hint) > 0.0:
                depth_map = depth_map * float(scale_hint)

            world_map = getattr(self.robot, "world_map", None)
            if world_map is None:
                return (False, 0, 0, 0, time.perf_counter() - start, "world_map_missing")

            success = bool(
                world_map.update_grid_from_depth(
                    depth_map,
                    detector.intrinsics,
                    detector.extrinsics,
                    robot_pose=pose,
                )
            )

            seg_count = 0
            segments = getattr(result, "segments", None) or []
            if self.update_cliffs and segments:
                seg_count = int(len(segments))
                try:
                    world_map.mark_cliff_on_grid(segments, robot_pose=pose)
                except Exception:
                    pass

            grid = getattr(world_map, "occupancy_grid", None)
            stats = getattr(grid, "last_update_counts", {}) if grid else {}
            ground = int(stats.get("ground", 0))
            obstacle = int(stats.get("obstacle", 0))

            return (success, ground, obstacle, seg_count, time.perf_counter() - start, None)

        except Exception as exc:
            return (False, 0, 0, 0, time.perf_counter() - start, f"{exc}")


class ScanAndMap(StateNode):
    """Convenience behavior: Rotate in place while updating occupancy grid.

    This behavior combines turning with grid updates to perform a 360-degree scan.

    Args:
        angle: Total rotation angle in degrees (default: 360)
        speed: Angular speed in degrees/second (default: 30)
        frame_interval: Seconds between grid updates (default: 0.3)
    """

    def __init__(
        self,
        angle: float = 360,
        speed: float = 30,
        frame_interval: float = 0.3,
        detector: Optional[CliffDetector] = None,
        update_cliffs: bool = True,
    ):
        from .parallel import Parallel
        from .nodes import Turn

        duration = abs(angle) / speed  # Time needed for rotation

        # Run Turn and UpdateOccupancyGrid in parallel
        super().__init__()
        grid_updater = UpdateOccupancyGrid(
            frame_interval=frame_interval,
            duration=duration,
            update_cliffs=update_cliffs,
        )
        if detector is not None:
            grid_updater.detector = detector
        self.behavior = Parallel(
            Turn(angle, speed),
            grid_updater,
        )

    def start(self, event=None):
        self.behavior.set_robot(self.robot)
        return self.behavior.start(event)

    def poll(self):
        return self.behavior.poll()
