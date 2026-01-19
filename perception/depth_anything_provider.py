"""DepthAnything runtime wrapper and CLI smoke test utilities.

The real DepthAnything model is not vendored in this repository yet, but this
module provides the scaffolding required to integrate it cleanly with the
surrounding perception stack.  Users can plug in their own inference callable
immediately (for example, a PyTorch `nn.Module`), while enjoying consistent
pre/post-processing, timing, and result packaging.

The CLI can run in a lightweight `--dummy` mode to validate the pipeline without
needing the heavyweight model weights.  When a proper model is available, the
`DepthAnythingProvider.from_torch()` factory can be extended to build it.
"""

from __future__ import annotations

import argparse
import time
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol

import numpy as np
try:  # Optional dependency; only required for CLI usage.
    from PIL import Image
except ImportError:  # pragma: no cover - exercised when Pillow not installed
    Image = None  # type: ignore


ArrayLike = np.ndarray


class DepthModel(Protocol):
    """Protocol describing the callable expected by :class:`DepthAnythingProvider`."""

    def __call__(self, image: ArrayLike) -> ArrayLike:
        """Return a depth map for the given RGB image."""


class ConfidenceModel(Protocol):
    """Protocol for optional confidence predictors."""

    def __call__(self, image: ArrayLike) -> ArrayLike:
        """Return a confidence map aligned with the given RGB image."""


@dataclass(frozen=True)
class DepthResult:
    """Structured output from a single DepthAnything inference call."""

    depth: ArrayLike
    """Depth map as a 2D float array. Units are arbitrary prior to scaling."""

    confidence: Optional[ArrayLike]
    """Optional confidence map in `[0, 1]`. Matches the depth shape."""

    valid_mask: ArrayLike
    """Boolean mask indicating pixels considered valid by the model."""

    scale_hint: float
    """Best-effort multiplicative factor to convert raw depth into metres."""

    runtime_ms: float
    """End-to-end inference time (including preprocessing), in milliseconds."""

    model_name: str
    """Identifier for the underlying DepthAnything variant."""

    device: str
    """Device string (e.g., `cpu`, `cuda:0`, `mps`)."""

    @property
    def shape(self) -> tuple[int, int]:
        return tuple(self.depth.shape)  # type: ignore[return-value]


class DepthAnythingProvider:
    """High-level convenience wrapper around a DepthAnything model."""

    def __init__(
        self,
        *,
        depth_model: Callable[[ArrayLike], ArrayLike],
        confidence_model: Optional[Callable[[ArrayLike], ArrayLike]] = None,
        scale_hint: float = 1.0,
        model_name: str = "DepthAnything",
        device: str = "cpu",
        preprocessor: Optional[Callable[[ArrayLike], ArrayLike]] = None,
        postprocess: Optional[Callable[[ArrayLike], ArrayLike]] = None,
    ) -> None:
        """Construct the provider with arbitrary depth/confidence callables.

        Args:
            depth_model: Callable that maps a float RGB array of shape (H, W, 3)
                (values in [0, 1]) to a depth array of shape (H, W).
            confidence_model: Optional callable returning a confidence map.
            scale_hint: Default multiplicative factor applied to the raw depth.
            model_name: Friendly name for logging/telemetry.
            device: Device string for introspection.
            preprocessor: Optional transform applied before calling `depth_model`.
            postprocess: Optional transform applied to the depth output.
        """

        self._depth_model = depth_model
        self._confidence_model = confidence_model
        self._scale_hint = float(scale_hint)
        self._model_name = str(model_name)
        self._device = str(device)
        self._preprocessor = preprocessor
        self._postprocess = postprocess

    @property
    def model_name(self) -> str:
        return self._model_name

    @property
    def device(self) -> str:
        return self._device

    def infer(self, rgb_image: ArrayLike) -> DepthResult:
        """Run a single DepthAnything inference on an RGB image.

        Args:
            rgb_image: NumPy array of shape (H, W, 3) in uint8 or float format.

        Returns:
            :class:`DepthResult` with depth/confidence/validity metadata.

        Raises:
            ValueError: if the input shape is invalid.
        """

        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError(
                f"Expected RGB image with shape (H, W, 3), received {rgb_image.shape}"
            )

        image_f32 = self._prepare_input(rgb_image)

        start = time.perf_counter()
        processed = self._preprocessor(image_f32) if self._preprocessor else image_f32
        depth = self._depth_model(processed)
        if depth.ndim != 2:
            raise ValueError(
                f"Depth model returned array with shape {depth.shape}; expected 2D map"
            )
        depth = depth.astype(np.float32, copy=False)

        if self._postprocess is not None:
            depth = self._postprocess(depth)

        confidence = None
        if self._confidence_model is not None:
            confidence = self._confidence_model(processed)
            if confidence.shape != depth.shape:
                raise ValueError(
                    "Confidence map shape "
                    f"{confidence.shape} does not match depth shape {depth.shape}"
                )
            confidence = np.clip(confidence.astype(np.float32, copy=False), 0.0, 1.0)

        valid_mask = np.isfinite(depth) & (depth > 0.0)
        runtime_ms = (time.perf_counter() - start) * 1e3

        return DepthResult(
            depth=depth,
            confidence=confidence,
            valid_mask=valid_mask,
            scale_hint=self._scale_hint,
            runtime_ms=runtime_ms,
            model_name=self._model_name,
            device=self._device,
        )

    # ------------------------------------------------------------------
    # Input helpers

    @staticmethod
    def _prepare_input(rgb_image: ArrayLike) -> ArrayLike:
        if rgb_image.dtype == np.uint8:
            image_f32 = rgb_image.astype(np.float32) / 255.0
        else:
            image_f32 = rgb_image.astype(np.float32, copy=False)
        return np.clip(image_f32, 0.0, 1.0)

    # ------------------------------------------------------------------
    # Factories & CLI support

    @classmethod
    def from_torch(
        cls,
        *,
        weights_path: Path,
        model_type: str = "depthanything-small",
        device: str = "cpu",
    ) -> "DepthAnythingProvider":
        """Factory expecting the official DepthAnything PyTorch implementation.

        Args:
            weights_path: Path to the `.pth` checkpoint.
            model_type: DepthAnything variant (e.g., ``depthanything-v2-small``).
            device: Target device string (``cpu``, ``cuda:0``, ``mps`` ...).
        """

        try:
            import torch  # type: ignore
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise RuntimeError(
                "PyTorch is required for DepthAnythingProvider.from_torch"
            ) from exc

        try:  # torchvision is required for Compose
            from torchvision.transforms import Compose  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "torchvision is required for DepthAnythingProvider.from_torch"
            ) from exc

        try:
            import cv2  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "OpenCV (cv2) is required for DepthAnythingProvider.from_torch"
            ) from exc

        repo_root = Path(__file__).resolve().parents[1] / "third_party" / "Depth-Anything-V2"
        if not repo_root.exists():
            raise FileNotFoundError(
                "Depth-Anything-V2 repository not found under third_party."
                " Clone/download it to third_party/Depth-Anything-V2 first."
            )
        if str(repo_root) not in sys.path:
            sys.path.insert(0, str(repo_root))

        try:
            from depth_anything_v2.dpt import DepthAnythingV2  # type: ignore
            from depth_anything_v2.util.transform import (  # type: ignore
                NormalizeImage,
                PrepareForNet,
                Resize,
            )
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "Unable to import Depth-Anything-V2 modules. Verify third_party/Depth-Anything-V2 is intact."
            ) from exc

        # Normalise model_type aliases.
        alias = model_type.lower().replace("_", "-")
        type_map = {
            "depthanything-small": "vits",
            "depthanything-v2-small": "vits",
            "depthanything-v2-vits": "vits",
            "vits": "vits",
            "depthanything-base": "vitb",
            "depthanything-v2-base": "vitb",
            "vitb": "vitb",
            "depthanything-large": "vitl",
            "depthanything-v2-large": "vitl",
            "vitl": "vitl",
            "vitg": "vitg",
        }
        if alias not in type_map:
            raise ValueError(f"Unsupported DepthAnything model type: {model_type}")
        encoder_key = type_map[alias]

        model_configs = {
            "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384], "input_size": 518},
            "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768], "input_size": 518},
            "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024], "input_size": 518},
            "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536], "input_size": 518},
        }

        config = dict(model_configs[encoder_key])
        input_size = int(config.pop("input_size"))

        model = DepthAnythingV2(**config)
        state_dict = torch.load(weights_path, map_location="cpu")
        model.load_state_dict(state_dict)

        torch_device = torch.device(device)
        model = model.to(torch_device)
        model.eval()

        transform = Compose(
            [
                Resize(
                    width=input_size,
                    height=input_size,
                    resize_target=False,
                    keep_aspect_ratio=True,
                    ensure_multiple_of=14,
                    resize_method="lower_bound",
                    image_interpolation_method=cv2.INTER_CUBIC,
                ),
                NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
                PrepareForNet(),
            ]
        )

        def _infer(image: ArrayLike) -> ArrayLike:
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"Expected RGB image with shape (H, W, 3); got {image.shape}")

            rgb = np.clip(image, 0.0, 1.0).astype(np.float32)
            h, w = rgb.shape[:2]

            sample = {"image": rgb}
            sample = transform(sample)
            tensor = torch.from_numpy(sample["image"]).unsqueeze(0).to(torch_device)

            with torch.no_grad():
                depth = model(tensor)
                depth = torch.nn.functional.interpolate(
                    depth[:, None],
                    (h, w),
                    mode="bilinear",
                    align_corners=True,
                )[0, 0]

            return depth.detach().cpu().numpy().astype(np.float32)

        return cls(
            depth_model=_infer,
            confidence_model=None,
            scale_hint=1.0,
            model_name=f"DepthAnythingV2-{encoder_key}",
            device=str(torch_device),
        )

    # ------------------------------------------------------------------
    # CLI utilities

    @staticmethod
    def _dummy_model(image: ArrayLike) -> ArrayLike:
        """Simple depth heuristic used by the CLI for quick validation."""

        h, w = image.shape[:2]
        yy, xx = np.meshgrid(np.linspace(0, 1, h), np.linspace(0, 1, w), indexing="ij")
        depth = (0.6 * yy + 0.4 * xx).astype(np.float32)
        return depth

    @staticmethod
    def _dummy_confidence(image: ArrayLike) -> ArrayLike:
        h, w = image.shape[:2]
        center = np.array([0.5, 0.5], dtype=np.float32)
        yy, xx = np.meshgrid(np.linspace(0, 1, h), np.linspace(0, 1, w), indexing="ij")
        dist = np.sqrt((yy - center[0]) ** 2 + (xx - center[1]) ** 2)
        conf = 1.0 - np.clip(dist / 0.75, 0.0, 1.0)
        return conf.astype(np.float32)

    @classmethod
    def build_dummy(cls) -> "DepthAnythingProvider":
        """Return a provider using lightweight analytic depth/confidence maps."""

        return cls(
            depth_model=cls._dummy_model,
            confidence_model=cls._dummy_confidence,
            scale_hint=1.0,
            model_name="DepthAnything-dummy",
            device="cpu",
        )


# ----------------------------------------------------------------------
# Command-line interface


def _load_rgb(path: Path) -> ArrayLike:
    if Image is None:
        raise RuntimeError(
            "Pillow is required to load images. Install `Pillow` or run the "
            "DepthAnythingProvider CLI with an already loaded numpy array."
        )
    with Image.open(path) as img:
        return np.array(img.convert("RGB"))


def _write_depth(path: Path, depth: ArrayLike) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, depth)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="DepthAnything smoke test CLI")
    parser.add_argument("image", type=Path, help="Input RGB image path")
    parser.add_argument(
        "--out",
        type=Path,
        help="Optional output .npy file to store the depth map",
    )
    parser.add_argument(
        "--dummy",
        action="store_true",
        help="Use an analytic dummy model instead of a real DepthAnything network",
    )
    args = parser.parse_args(argv)

    if args.dummy:
        provider = DepthAnythingProvider.build_dummy()
    else:
        raise RuntimeError(
            "Real DepthAnything inference is not wired in yet. "
            "Run the CLI with --dummy or extend DepthAnythingProvider.from_torch."
        )

    rgb = _load_rgb(args.image)
    result = provider.infer(rgb)

    print(
        f"[depth-anything] model={result.model_name} device={result.device} "
        f"runtime_ms={result.runtime_ms:.2f} shape={result.shape}"
    )

    if args.out is not None:
        _write_depth(args.out, result.depth * result.scale_hint)
        print(f"[depth-anything] depth saved to {args.out}")

    return 0


if __name__ == "__main__":  # pragma: no cover - CLI exercised manually
    raise SystemExit(main())
