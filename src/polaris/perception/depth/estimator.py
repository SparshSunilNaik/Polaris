from __future__ import annotations

from dataclasses import dataclass

from polaris.perception.dependencies import add_homebrew_python_site

add_homebrew_python_site()

import numpy as np  # noqa: E402

from .models import DepthConfig


@dataclass(frozen=True)
class DepthInference:
    raw_depth: np.ndarray
    input_width: int
    input_height: int
    output_width: int
    output_height: int


def resolve_device(requested: str) -> str:
    import torch

    if requested == "cpu":
        return "cpu"
    if requested == "mps":
        if not torch.backends.mps.is_available():
            raise RuntimeError("MPS was requested but is not available")
        return "mps"
    if requested != "auto":
        raise ValueError("device must be auto, mps, or cpu")
    return "mps" if torch.backends.mps.is_available() else "cpu"


class DepthAnythingEstimator:
    """Official Hugging Face Depth Anything V2 Small inference wrapper."""

    def __init__(self, config: DepthConfig) -> None:
        self.config = config
        self.device = resolve_device(config.device)
        self.model = None
        self.image_processor = None

    def load(self) -> None:
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        self.image_processor = AutoImageProcessor.from_pretrained(
            self.config.model_identifier,
            revision=self.config.model_revision,
            use_fast=False,
        )
        self.model = AutoModelForDepthEstimation.from_pretrained(
            self.config.model_identifier,
            revision=self.config.model_revision,
            dtype=torch.float32,
        )
        self.model.to(self.device)
        self.model.eval()

    def infer(self, image_bgr: np.ndarray) -> DepthInference:
        import torch
        import torch.nn.functional as functional

        if self.model is None or self.image_processor is None:
            raise RuntimeError("Depth estimator has not been loaded")
        image_rgb = np.ascontiguousarray(image_bgr[:, :, ::-1])
        inputs = self.image_processor(
            images=image_rgb,
            return_tensors="pt",
            size={
                "height": self.config.input_height,
                "width": self.config.input_width,
            },
            keep_aspect_ratio=False,
            ensure_multiple_of=14,
        )
        pixel_values = inputs["pixel_values"].to(
            device=self.device,
            dtype=torch.float32,
        )
        input_height, input_width = pixel_values.shape[-2:]
        if self.device == "mps":
            torch.mps.synchronize()
        with torch.inference_mode():
            predicted = self.model(pixel_values=pixel_values).predicted_depth
            resized = functional.interpolate(
                predicted.unsqueeze(1),
                size=(input_height, input_width),
                mode="bicubic",
                align_corners=False,
            ).squeeze(0).squeeze(0)
        if self.device == "mps":
            torch.mps.synchronize()
        raw_depth = resized.detach().to("cpu", dtype=torch.float32).numpy()
        return DepthInference(
            raw_depth=raw_depth,
            input_width=int(input_width),
            input_height=int(input_height),
            output_width=int(raw_depth.shape[1]),
            output_height=int(raw_depth.shape[0]),
        )
