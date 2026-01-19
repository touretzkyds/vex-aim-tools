"""Camera overlay hook that renders cliff detection results."""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

try:  # Optional acceleration when OpenCV is available.
    import cv2  # type: ignore
except Exception:  # pragma: no cover - OpenCV optional
    cv2 = None

# Note: Camera imported locally in build_default() to avoid circular import

from perception import (
    CliffDetector,
    CliffDetectorOutput,
    CliffSegment,
    DepthAnythingProvider,
    DetectorConfig,
    load_cliff_calibration,
)

from .camera_overlay import _draw_line_numpy  # reuse existing rasteriser

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class CliffOverlayConfig:
    """Runtime configuration for the cliff overlay hook."""

    frame_stride: int = 10  # Process every 10th frame (~3Hz) to avoid blocking Qt main thread
    strong_confidence: float = 0.5
    weak_confidence: float = 0.2
    segment_thickness: int = 2
    strong_color: tuple[int, int, int] = (255, 64, 64)   # Solid red
    weak_color: tuple[int, int, int] = (255, 180, 80)    # Amber
    status_color: tuple[int, int, int] = (255, 214, 0)
    status_shadow: tuple[int, int, int] = (0, 0, 0)
    status_font_scale: float = 0.6
    status_thickness: int = 1


class CliffOverlay:
    """Callable overlay that annotates RGB frames with cliff detections."""

    def __init__(
        self,
        *,
        detector: Optional[CliffDetector],
        config: Optional[CliffOverlayConfig] = None,
        robot=None,  # NEW: robot reference for reading cached results
    ) -> None:
        self._detector = detector
        self._config = config or CliffOverlayConfig()
        self._robot = robot
        self._use_cached_results = robot is not None and detector is None
        self.enabled = detector is not None or self._use_cached_results
        self._frame_stride = max(1, int(self._config.frame_stride))
        self._frame_counter = 0
        self._last_output: Optional[CliffDetectorOutput] = None
        self._cached_timestamp: Optional[float] = None
        self._cache_expiry_seconds = 5.0  # Show cliff for 5 seconds after detection

    # ------------------------------------------------------------------
    # Factories

    @classmethod
    def build_default(
        cls,
        *,
        config: Optional[CliffOverlayConfig] = None,
        detector_config: Optional[DetectorConfig] = None,
    ) -> "CliffOverlay":
        """Best-effort factory using the repository defaults."""

        provider_mode = os.environ.get("DEPTHANYTHING_PROVIDER", "dummy").lower()
        try:
            if provider_mode == "dummy":
                provider = DepthAnythingProvider.build_dummy()
            elif provider_mode == "torch":
                weights_env = os.environ.get("DEPTHANYTHING_WEIGHTS")
                if not weights_env:
                    raise RuntimeError("DEPTHANYTHING_WEIGHTS must be set for torch provider")
                provider = DepthAnythingProvider.from_torch(
                    weights_path=Path(weights_env),
                    model_type=os.environ.get("DEPTHANYTHING_MODEL", "depthanything-small"),
                    device=os.environ.get("DEPTHANYTHING_DEVICE", "cpu"),
                )
            else:
                raise RuntimeError(f"Unsupported DEPTHANYTHING_PROVIDER={provider_mode!r}")
        except NotImplementedError as exc:
            _LOGGER.warning("DepthAnythingProvider.from_torch not implemented: %s", exc)
            return cls(detector=None, config=config)
        except Exception as exc:
            _LOGGER.warning("Failed to initialise DepthAnything provider: %s", exc)
            return cls(detector=None, config=config)

        # Import here to avoid circular import (viewer.cliff_overlay ↔ aim_fsm)
        from aim_fsm.camera import Camera
        
        camera = Camera()
        calibration = load_cliff_calibration(camera)
        intrinsics = calibration.intrinsics

        detector = CliffDetector(
            depth_provider=provider,
            intrinsics=intrinsics,
            gravity_camera=calibration.gravity_camera,
            extrinsics=calibration.camera_to_base,
            config=detector_config,
        )

        return cls(detector=detector, config=config)

    # ------------------------------------------------------------------
    # Callable API

    def __call__(self, image: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return image

        if image is None or image.ndim != 3:
            return image

        # NEW: Cached results mode (read from robot.cliff_results)
        if self._use_cached_results and self._robot is not None:
            import time
            cliff_results = getattr(self._robot, "cliff_results", None)
            cliff_timestamp = getattr(self._robot, "cliff_timestamp", None)

            # Check if we have new results
            if cliff_results and cliff_timestamp != self._cached_timestamp:
                self._cached_timestamp = cliff_timestamp
                # Find best result (highest confidence)
                best_output = None
                best_conf = -1.0
                for result in cliff_results:
                    if hasattr(result, "segments") and result.segments:
                        for seg in result.segments:
                            if seg.confidence > best_conf:
                                best_conf = seg.confidence
                                best_output = result
                self._last_output = best_output

            # Check if results expired (5 seconds)
            if cliff_timestamp and (time.time() - cliff_timestamp) > self._cache_expiry_seconds:
                self._last_output = None

        # Original detector mode
        elif self._detector is not None:
            should_run = self._last_output is None or (self._frame_counter % self._frame_stride == 0)
            if should_run:
                try:
                    output = self._detector.process(image)
                except Exception as exc:  # pragma: no cover - defensive
                    _LOGGER.warning("Cliff detection failed; disabling overlay: %s", exc, exc_info=True)
                    self.enabled = False
                    return image
                self._last_output = output
            self._frame_counter += 1

        if self._last_output is None:
            return image

        annotated = image.copy()
        output = self._last_output
        self._render_segments(annotated, output.segments)
        self._render_status(annotated, output.status)
        return annotated

    @property
    def last_output(self) -> Optional[CliffDetectorOutput]:
        """Return the most recent detector output (if available)."""

        return self._last_output

    # ------------------------------------------------------------------
    # Rendering helpers

    def _render_segments(self, image: np.ndarray, segments: Sequence[CliffSegment]) -> None:
        if not segments:
            return
        for segment in segments:
            pts = segment.polyline_px
            if pts.shape[0] < 2:
                continue
            confidence = float(segment.confidence)
            if confidence >= self._config.strong_confidence:
                color = self._config.strong_color
            elif confidence >= self._config.weak_confidence:
                color = self._config.weak_color
            else:
                color = tuple(min(255, c + 40) for c in self._config.weak_color)
            self._draw_polyline(image, pts, color, self._config.segment_thickness)

    def _render_status(self, image: np.ndarray, status: str) -> None:
        if not status or status == "none":
            return
        text_map = {
            "plane": "Cliff detected",
            "gradient": "Cliff (edge discontinuity)",
            "no_depth": "Depth unavailable",
        }
        message = text_map.get(status, status)
        if cv2 is None:  # pragma: no cover - fallback draw
            return
        h, w = image.shape[:2]
        org = (int(w * 0.05), int(h * 0.92))
        font = cv2.FONT_HERSHEY_SIMPLEX
        shadow_org = (org[0] + 1, org[1] + 1)
        cv2.putText(
            image,
            message,
            shadow_org,
            font,
            self._config.status_font_scale,
            self._config.status_shadow,
            thickness=self._config.status_thickness + 1,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            image,
            message,
            org,
            font,
            self._config.status_font_scale,
            self._config.status_color,
            thickness=self._config.status_thickness,
            lineType=cv2.LINE_AA,
        )

    # ------------------------------------------------------------------
    # Low-level drawing primitives

    def _draw_polyline(self, image: np.ndarray, pts: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
        points = np.round(pts).astype(np.int32)
        if cv2 is not None:  # pragma: no cover - exercised when OpenCV present
            cv2.polylines(image, [points.reshape((-1, 1, 2))], False, color, thickness, lineType=cv2.LINE_AA)
            return
        for idx in range(len(points) - 1):
            x0, y0 = map(int, points[idx])
            x1, y1 = map(int, points[idx + 1])
            _draw_line_numpy(image, x0, y0, x1, y1, color)

    def _draw_dashed_line(
        self,
        image: np.ndarray,
        start: np.ndarray,
        end: np.ndarray,
        color: tuple[int, int, int],
        thickness: int,
        *,
        dash: float,
        gap: float,
    ) -> None:
        vector = end - start
        length = float(np.linalg.norm(vector))
        if length < 1e-3:
            return
        direction = vector / length
        step = max(1.0, dash + gap)
        num_segments = max(int(length / step) + 1, 1)
        cursor = 0.0

        for idx in range(num_segments):
            t0 = cursor / length
            t1 = min((cursor + dash) / length, 1.0)
            if t0 >= 1.0:
                break
            p0 = start + direction * (t0 * length)
            p1 = start + direction * (t1 * length)
            self._draw_polyline(
                image,
                np.stack([p0, p1], axis=0),
                color,
                thickness,
            )
            cursor += step


__all__ = ["CliffOverlay", "CliffOverlayConfig"]
