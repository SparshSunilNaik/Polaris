from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any

from .dependencies import add_homebrew_python_site

add_homebrew_python_site()

import numpy as np  # noqa: E402


class VisualCondition(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True)
class CameraFrame:
    image_bgr: np.ndarray
    timestamp_monotonic: float
    source_timestamp: float | None
    sequence: int
    width: int
    height: int
    pixel_format: str


@dataclass(frozen=True)
class VisualHealthReport:
    timestamp: float
    frame_age_ms: float
    stale: bool
    fps: float
    width: int
    height: int
    brightness: float
    contrast: float
    blur_score: float
    texture_score: float
    feature_count: int
    feature_coverage: float
    overexposed_ratio: float
    underexposed_ratio: float
    health_score: float
    condition: VisualCondition
    reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["condition"] = self.condition.value
        return result
