"""Cliff detection FSM behavior for VEX AIM robot.

This module provides a standalone FSM behavior that captures multiple camera
frames and runs cliff detection, storing results for worldmap visualization.

Usage in simple_cli:
    detect_cliff 5        # Capture 5 frames and detect cliffs
    detect_cliff          # Use default (5 frames)
"""

from __future__ import annotations

import concurrent.futures
import dataclasses
import logging
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .base import StateNode

try:
    from perception import CliffDetector, DepthAnythingProvider, load_cliff_calibration
    CLIFF_DETECTION_AVAILABLE = True
except ImportError:
    CLIFF_DETECTION_AVAILABLE = False

_LOGGER = logging.getLogger(__name__)


def _transform_polyline_to_world(robot, polyline_base_m: np.ndarray) -> np.ndarray:
    """Transform polyline from robot base frame to world frame.

    This function converts cliff edge coordinates from the robot's local reference
    frame to global world coordinates, accounting for the robot's current position
    and heading. This ensures cliff edges remain at fixed world positions even
    when the robot moves.

    Args:
        robot: Robot instance with pose (x, y in mm, theta in radians)
        polyline_base_m: (N, 3) array of points in robot base frame (meters)

    Returns:
        (N, 3) array of points in world frame (meters)
    """
    from aim_fsm.geometry import aboutZ, point

    # Get robot pose (in mm and radians)
    rotation = aboutZ(robot.pose.theta)
    robot_pos = point(robot.pose.x, robot.pose.y)

    world_points = []
    for pt in polyline_base_m:
        # Convert point from meters to mm
        pt_mm = point(pt[0] * 1000, pt[1] * 1000)

        # Transform: rotate + translate
        world_xy = rotation.dot(pt_mm) + robot_pos

        # Convert back to meters and store
        world_points.append([
            float(world_xy[0][0]) / 1000.0,  # x in meters
            float(world_xy[1][0]) / 1000.0,  # y in meters
            float(pt[2])                      # z in meters (unchanged)
        ])

    return np.array(world_points)


class DetectCliff(StateNode):
    """FSM behavior that captures N camera frames and runs cliff detection.

    This behavior:
    1. Captures multiple camera frames over ~1-2 seconds
    2. Runs cliff detection on each frame
    3. Stores results in robot.cliff_results for worldmap visualization
    4. Reports summary of detected cliffs

    Args:
        num_frames: Number of frames to capture (default: 5)
        save_frames: If True, save captured frames to disk (default: False)
        frame_interval: Minimum seconds between frame captures (default: 0.2)
    """

    def __init__(self, num_frames: int = 5, save_frames: bool = False, frame_interval: float = 0.2):
        super().__init__()
        self.num_frames = max(1, int(num_frames))
        self.save_frames = save_frames
        self.frame_interval = frame_interval

        # State
        self.frames = []
        self.last_frame_id = None
        self.last_capture_time = 0.0
        self.detector: Optional[CliffDetector] = None
        self.start_time = None

        # Background processing
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._processing_future: Optional[concurrent.futures.Future] = None

    def start(self, event=None):
        """Start cliff detection by initializing detector and beginning frame capture."""
        print("[DetectCliff.start] CALLED!")  # Critical debug - always visible
        sys.stdout.flush()
        _LOGGER.debug("start() called")
        super().start(event)

        if not CLIFF_DETECTION_AVAILABLE:
            _LOGGER.error("Cliff detection not available (perception module missing)")
            sys.stdout.flush()
            self.post_failure(details="perception_module_missing")
            return

        # Check if detector is already initialized (should be done in CLI thread)
        if self.detector is None:
            _LOGGER.error("Detector not initialized!")
            sys.stdout.flush()
            self.post_failure(details="detector_not_initialized")
            return

        _LOGGER.info(f"Starting cliff detection (capturing {self.num_frames} frames)")
        sys.stdout.flush()

        # Reset state
        self.frames = []
        self.last_frame_id = None
        self.last_capture_time = 0.0
        self.start_time = time.time()

        # Initialize thread pool for background processing
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._processing_future = None

        # Debug: check robot attributes
        has_frame_count = hasattr(self.robot, "frame_count")
        has_camera_image = hasattr(self.robot, "camera_image")
        _LOGGER.debug(f"Robot attributes: frame_count={has_frame_count}, camera_image={has_camera_image}")

        if has_frame_count:
            _LOGGER.debug(f"Current frame_count: {self.robot.frame_count}")

        if has_camera_image:
            img = self.robot.camera_image
            if img is not None:
                _LOGGER.debug(f"Camera image available: shape={getattr(img, 'shape', 'unknown')}")
            else:
                _LOGGER.debug("camera_image is None")

        # Start polling for frames at 10Hz
        _LOGGER.debug("Setting polling interval to 0.1s (10Hz)")
        self.set_polling_interval(0.1)
        _LOGGER.debug("Polling started, waiting for frames...")
        sys.stdout.flush()

    def poll(self):
        """Poll for new camera frames and capture them."""
        # Only print first few times to avoid spam
        if len(self.frames) == 0 and not hasattr(self, '_poll_count'):
            print(f"[DetectCliff.poll] First poll! Looking for frames...")
            sys.stdout.flush()
            self._poll_count = 0

        try:
            current_time = time.time()

            # Check if we have a new frame
            frame_id = getattr(self.robot, "frame_count", None)
            if frame_id is None:
                if not hasattr(self, '_warned_no_frame_count'):
                    print("[DetectCliff.poll] WARNING: robot.frame_count is None!")
                    sys.stdout.flush()
                    self._warned_no_frame_count = True
                _LOGGER.debug("poll(): frame_count is None")
                return

            # Skip if same frame as before
            if frame_id == self.last_frame_id:
                return

            # Check if enough time has elapsed since last capture
            if current_time - self.last_capture_time < self.frame_interval:
                return

            # Capture frame
            image = getattr(self.robot, "camera_image", None)
            if image is None:
                _LOGGER.debug(f"poll(): camera_image is None (frame_id={frame_id})")
                return

            # Store frame and pose (make a copy to avoid reference issues)
            # Capture pose at the moment of frame capture
            pose = (self.robot.pose.x, self.robot.pose.y, self.robot.pose.theta)
            self.frames.append((np.array(image), pose))
            
            self.last_frame_id = frame_id
            self.last_capture_time = current_time

            elapsed = current_time - self.start_time
            _LOGGER.info(f"Captured frame {len(self.frames)}/{self.num_frames} (elapsed: {elapsed:.1f}s)")
            sys.stdout.flush()

            # Check if we have enough frames
            if len(self.frames) >= self.num_frames:
                self.set_polling_interval(None)  # Stop polling
                self.process_frames()
        except Exception as exc:
            _LOGGER.error(f"ERROR in poll(): {exc}")
            import traceback
            traceback.print_exc()
            sys.stdout.flush()
            self.set_polling_interval(None)
            self.post_failure(details=f"poll_error: {exc}")

    def process_frames(self):
        """Schedule frame processing in background thread (non-blocking)."""
        _LOGGER.info(f"\nProcessing {len(self.frames)} frames for cliff detection...")
        sys.stdout.flush()

        # Submit processing to background thread
        self._processing_future = self._executor.submit(self._process_frames_worker)

        # Schedule completion check (poll every 0.1s)
        self.robot.loop.call_later(0.1, self._check_processing_done)

    def _process_frames_worker(self):
        """Worker method that runs in background thread to process frames.

        Returns:
            Tuple of (results, total_segments, error_msg)
        """
        try:
            results = []
            total_segments = 0
            processing_start = time.time()

            for i, (frame, pose) in enumerate(self.frames):
                frame_start = time.time()

                # Log progress (thread-safe call back to event loop)
                self.robot.loop.call_soon_threadsafe(
                    self._log_progress,
                    f"Processing frame {i+1}/{len(self.frames)}..."
                )

                try:
                    result = self.detector.process(frame)
                    frame_elapsed = time.time() - frame_start

                    results.append(result)
                    num_segments = len(result.segments)
                    total_segments += num_segments

                    # Log frame result
                    if num_segments > 0:
                        self.robot.loop.call_soon_threadsafe(
                            _LOGGER.info,
                            f"  Frame {i+1}: {num_segments} segment(s) in {frame_elapsed:.2f}s"
                        )
                        for seg in result.segments:
                            if seg.distance_to_robot is not None:
                                self.robot.loop.call_soon_threadsafe(
                                    _LOGGER.info,
                                    f"    - Confidence: {seg.confidence:.2f}, "
                                    f"Distance: {seg.distance_to_robot:.3f}m, "
                                    f"Points: {len(seg.polyline_px)}"
                                )
                            else:
                                self.robot.loop.call_soon_threadsafe(
                                    _LOGGER.info,
                                    f"    - Confidence: {seg.confidence:.2f}, "
                                    f"Points: {len(seg.polyline_px)} (no distance)"
                                )
                    else:
                        self.robot.loop.call_soon_threadsafe(
                            _LOGGER.info,
                            f"  Frame {i+1}: No cliff detected ({frame_elapsed:.2f}s)"
                        )

                except Exception as exc:
                    self.robot.loop.call_soon_threadsafe(
                        _LOGGER.error,
                        f"  Frame {i+1}: Detection failed: {exc}"
                    )
                    import traceback
                    traceback.print_exc()

                # Optionally save frame to disk
                if self.save_frames:
                    self._save_frame(frame, i)

            total_elapsed = time.time() - processing_start
            return (results, total_segments, total_elapsed, None)

        except Exception as exc:
            # Fatal error in worker
            import traceback
            error_msg = f"Fatal error in background processing: {exc}\n{traceback.format_exc()}"
            return ([], 0, 0, error_msg)

    def _check_processing_done(self):
        """Check if background processing is complete (called from event loop)."""
        if self._processing_future is None:
            _LOGGER.error("_check_processing_done called but no future exists!")
            sys.stdout.flush()
            return

        if not self._processing_future.done():
            # Not done yet, check again in 0.1s
            self.robot.loop.call_later(0.1, self._check_processing_done)
            return

        # Processing complete! Get results
        try:
            results, total_segments, processing_time, error_msg = self._processing_future.result(timeout=0.1)

            if error_msg:
                # Fatal error occurred
                _LOGGER.error(error_msg)
                sys.stdout.flush()
                self.post_failure(details="processing_error")
                return

            # Store results in robot for worldmap viewer
            # NEW: Accumulate multiple detection sessions instead of overwriting
            timestamp = time.time()

            # Transform all polylines from base frame to world frame
            # This ensures cliffs stay at fixed world positions when robot moves
            # Note: CliffSegment and CliffDetectionResult are frozen dataclasses,
            # so we need to create new instances
            transformed_results = []
            for result in results:
                transformed_segments = []
                for seg in result.segments:
                    if seg.polyline_world is not None:
                        # Create new segment with transformed coordinates
                        transformed_polyline = _transform_polyline_to_world(
                            self.robot, seg.polyline_world
                        )
                        new_seg = dataclasses.replace(seg, polyline_world=transformed_polyline)
                        transformed_segments.append(new_seg)
                    else:
                        transformed_segments.append(seg)
                # Create new result with transformed segments
                new_result = dataclasses.replace(result, segments=transformed_segments)
                transformed_results.append(new_result)

            # Use transformed results for storage
            results = transformed_results

            session = {
                "id": f"cliff_{int(timestamp * 1000)}",  # Unique ID based on timestamp
                "timestamp": timestamp,
                "robot_pose": {  # Save robot pose at detection time
                    "x": float(self.robot.pose.x),
                    "y": float(self.robot.pose.y),
                    "theta": float(self.robot.pose.theta),
                },
                "results": results,
                "num_frames": len(results),
                "total_segments": total_segments,
            }

            # Initialize cliff history if needed
            if not hasattr(self.robot, "cliff_sessions"):
                self.robot.cliff_sessions = []

            # Append new session (accumulate, don't overwrite)
            self.robot.cliff_sessions.append(session)

            # Also keep backward compatibility with old single-session format
            self.robot.cliff_results = results
            self.robot.cliff_timestamp = timestamp
            self.robot.cliff_num_frames = len(results)

            total_elapsed = time.time() - self.start_time
            _LOGGER.info(f"\nCliff detection complete! ({total_elapsed:.1f}s total, {processing_time:.1f}s processing)")
            _LOGGER.info(f"Total segments detected: {total_segments} across {len(results)} frames")
            sys.stdout.flush()

            if total_segments > 0:
                self.post_success(details={"num_segments": total_segments, "num_frames": len(results)})
            else:
                self.post_completion()

        except concurrent.futures.TimeoutError:
            _LOGGER.error("Timeout waiting for processing results!")
            sys.stdout.flush()
            self.post_failure(details="processing_timeout")
        except Exception as exc:
            _LOGGER.error(f"Error retrieving processing results: {exc}")
            import traceback
            traceback.print_exc()
            sys.stdout.flush()
            self.post_failure(details=f"result_error: {exc}")
        finally:
            # Cleanup thread pool
            if self._executor:
                self._executor.shutdown(wait=False)
                self._executor = None

    def _log_progress(self, message: str):
        """Helper to log progress with flush (called from background thread)."""
        _LOGGER.info(message)
        sys.stdout.flush()

    def _initialize_detector(self) -> CliffDetector:
        """Initialize the CliffDetector with appropriate configuration."""
        # Get DepthAnything provider from environment
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

        # Get calibration from robot camera
        camera = getattr(self.robot, "camera", None)
        calibration = load_cliff_calibration(camera)

        # Create detector
        detector = CliffDetector(
            depth_provider=provider,
            intrinsics=calibration.intrinsics,
            gravity_camera=calibration.gravity_camera,
            extrinsics=calibration.camera_to_base,
        )

        return detector

    def _save_frame(self, frame: np.ndarray, index: int):
        """Save a captured frame to disk for debugging."""
        try:
            import cv2
            output_dir = Path("cliff_frames")
            output_dir.mkdir(exist_ok=True)

            fname = output_dir / f"cliff_frame_{int(time.time())}_{index}.png"
            cv2.imwrite(str(fname), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            _LOGGER.debug(f"Saved frame to {fname}")
        except Exception as exc:
            _LOGGER.warning(f"Failed to save frame {index}: {exc}")


__all__ = ["DetectCliff"]
