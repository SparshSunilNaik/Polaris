from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields, replace
from enum import Enum
from pathlib import Path
from time import monotonic
from typing import Any


REGION_NAMES = (
    "UP_LEFT",
    "UP",
    "UP_RIGHT",
    "LEFT",
    "CENTRE",
    "RIGHT",
    "DOWN_LEFT",
    "DOWN",
    "DOWN_RIGHT",
)


class DepthCondition(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True)
class DepthConfig:
    model_identifier: str = "depth-anything/Depth-Anything-V2-Small-hf"
    model_revision: str = "5426e4f0f36572d16453bbda7a8389317b1bef99"
    device: str = "auto"
    target_fps: float = 3.0
    input_width: int = 392
    input_height: int = 294
    temporal_alpha: float = 0.45
    stale_after_ms: float = 1200.0
    source_stale_after_ms: float = 500.0
    far_percentile: float = 5.0
    near_percentile: float = 95.0
    regional_risk_percentile: float = 75.0
    risk_threshold: float = 0.70
    immediate_centre_risk: float = 0.90
    persistence_count: int = 2
    min_dynamic_range_ratio: float = 0.05
    stability_healthy: float = 0.85
    stability_critical: float = 0.35
    healthy_score: float = 0.70
    critical_score: float = 0.35
    source_texture_healthy: float = 4.0
    depth_required_for_motion: bool = False
    raw_larger_is_near: bool = True

    @classmethod
    def from_json(cls, path: str | Path) -> DepthConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown depth settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        if self.device not in {"auto", "mps", "cpu"}:
            raise ValueError("device must be auto, mps, or cpu")
        numeric = (
            self.target_fps,
            self.temporal_alpha,
            self.stale_after_ms,
            self.source_stale_after_ms,
            self.far_percentile,
            self.near_percentile,
            self.regional_risk_percentile,
            self.risk_threshold,
            self.immediate_centre_risk,
            self.min_dynamic_range_ratio,
            self.stability_healthy,
            self.stability_critical,
            self.healthy_score,
            self.critical_score,
            self.source_texture_healthy,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Depth numeric settings must be finite")
        if self.target_fps <= 0 or self.input_width <= 0 or self.input_height <= 0:
            raise ValueError("target_fps and input dimensions must be positive")
        if self.stale_after_ms <= 0 or self.source_stale_after_ms <= 0:
            raise ValueError("Depth stale timeouts must be positive")
        if not 0 < self.temporal_alpha <= 1:
            raise ValueError("temporal_alpha must be in (0, 1]")
        if not 0 <= self.far_percentile < self.near_percentile <= 100:
            raise ValueError("far_percentile must be below near_percentile")
        if not 0 <= self.regional_risk_percentile <= 100:
            raise ValueError("regional_risk_percentile must be in [0, 100]")
        unit_values = (
            self.risk_threshold,
            self.immediate_centre_risk,
            self.stability_healthy,
            self.stability_critical,
            self.healthy_score,
            self.critical_score,
        )
        if any(value < 0 or value > 1 for value in unit_values):
            raise ValueError("Depth risk, stability, and health thresholds must be in [0, 1]")
        if self.stability_critical >= self.stability_healthy:
            raise ValueError("stability_critical must be below stability_healthy")
        if self.critical_score >= self.healthy_score:
            raise ValueError("critical_score must be below healthy_score")
        if self.min_dynamic_range_ratio < 0 or self.source_texture_healthy < 0:
            raise ValueError("Dynamic range and texture thresholds cannot be negative")
        if self.persistence_count < 1:
            raise ValueError("persistence_count must be at least one")


@dataclass(frozen=True)
class DepthSourceHealth:
    visual_health_score: float
    visual_condition: str
    texture_score: float
    overexposed_ratio: float
    underexposed_ratio: float
    reasons: tuple[str, ...]


EMPTY_GRID = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
)


@dataclass(frozen=True)
class DepthState:
    available: bool = False
    stale: bool = True
    source_frame_sequence: int = 0
    source_frame_age_ms: float = -1.0
    inference_timestamp_monotonic: float = 0.0
    inference_latency_ms: float = 0.0
    warmup_latency_ms: float = 0.0
    inference_fps: float = 0.0
    model_name: str = ""
    model_revision: str = ""
    device: str = ""
    input_width: int = 0
    input_height: int = 0
    output_width: int = 0
    output_height: int = 0
    temporal_stability: float = 0.0
    depth_health_score: float = 0.0
    condition: DepthCondition = DepthCondition.CRITICAL
    reasons: tuple[str, ...] = ("Depth model is not available",)
    risk_grid: tuple[tuple[float, float, float], ...] = EMPTY_GRID
    nearest_region: str = "NONE"
    safest_region: str = "NONE"
    centre_risk: float = 0.0
    left_risk: float = 0.0
    right_risk: float = 0.0
    upper_risk: float = 0.0
    lower_risk: float = 0.0
    global_near_percentile: float = 0.0
    global_median: float = 0.0
    global_far_percentile: float = 0.0
    raw_dynamic_range: float = 0.0
    relative_dynamic_range: float = 0.0
    valid_pixel_ratio: float = 0.0
    dropped_frames: int = 0
    consecutive_failures: int = 0

    def refreshed(
        self,
        stale_after_ms: float,
        now_monotonic: float | None = None,
    ) -> DepthState:
        now = monotonic() if now_monotonic is None else now_monotonic
        if self.inference_timestamp_monotonic == 0.0:
            return self
        inference_age_ms = (now - self.inference_timestamp_monotonic) * 1000.0
        if inference_age_ms <= stale_after_ms:
            return self
        reasons = tuple(dict.fromkeys((*self.reasons, "Depth inference is stale")))
        return replace(
            self,
            stale=True,
            depth_health_score=0.0,
            condition=DepthCondition.CRITICAL,
            reasons=reasons,
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["condition"] = self.condition.value
        result["risk_grid"] = [list(row) for row in self.risk_grid]
        return result
