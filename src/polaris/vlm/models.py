from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields, replace
from enum import Enum
from pathlib import Path
from urllib.parse import urlparse


class VLMStatus(str, Enum):
    PENDING = "PENDING"
    SUCCESS = "SUCCESS"
    UNAVAILABLE = "UNAVAILABLE"
    MODEL_UNAVAILABLE = "MODEL_UNAVAILABLE"
    TIMEOUT = "TIMEOUT"
    INVALID_RESPONSE = "INVALID_RESPONSE"
    STALE = "STALE"


@dataclass(frozen=True, slots=True)
class VLMConfig:
    endpoint: str = "http://localhost:11434"
    model: str = "qwen2.5vl:7b"
    inference_timeout_s: float = 60.0
    health_timeout_s: float = 2.0
    maximum_retries: int = 1
    retry_delay_s: float = 0.25
    cache_capacity: int = 8
    cache_ttl_s: float = 5.0
    inference_interval_s: float = 4.0
    result_stale_after_s: float = 12.0
    ollama_num_gpu: int | None = None
    fusion_window_size: int = 5
    fusion_maximum_observation_age_s: float = 20.0
    fusion_minimum_evidence_count: int = 3
    fusion_target_confirmation_threshold: int = 3
    fusion_target_rejection_threshold: int = 3
    fusion_minimum_accepted_confidence: float = 0.5
    fusion_landmark_support_count: int = 2

    @property
    def inference_budget_s(self) -> float:
        attempts = self.maximum_retries + 1
        return attempts * self.inference_timeout_s + self.maximum_retries * self.retry_delay_s

    @classmethod
    def from_json(cls, path: str | Path) -> VLMConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown VLM settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        parsed = urlparse(self.endpoint)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("endpoint must be an HTTP or HTTPS URL")
        if not self.model:
            raise ValueError("model must not be empty")
        numeric = (
            self.inference_timeout_s,
            self.health_timeout_s,
            self.retry_delay_s,
            self.cache_ttl_s,
            self.inference_interval_s,
            self.result_stale_after_s,
            self.fusion_maximum_observation_age_s,
            self.fusion_minimum_accepted_confidence,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("VLM timing settings must be finite")
        if self.inference_timeout_s <= 0 or self.health_timeout_s <= 0:
            raise ValueError("VLM timeouts must be positive")
        if self.maximum_retries < 0 or self.retry_delay_s < 0:
            raise ValueError("VLM retry settings must be non-negative")
        if self.cache_capacity < 0 or self.cache_ttl_s < 0:
            raise ValueError("VLM cache settings must be non-negative")
        if self.inference_interval_s <= 0 or self.result_stale_after_s <= 0:
            raise ValueError("VLM inference interval and stale timeout must be positive")
        if self.ollama_num_gpu is not None and self.ollama_num_gpu < 0:
            raise ValueError("ollama_num_gpu must be non-negative when configured")
        fusion_counts = (
            self.fusion_window_size,
            self.fusion_minimum_evidence_count,
            self.fusion_target_confirmation_threshold,
            self.fusion_target_rejection_threshold,
            self.fusion_landmark_support_count,
        )
        if any(value < 1 for value in fusion_counts):
            raise ValueError("VLM fusion counts must be positive")
        if any(value > self.fusion_window_size for value in fusion_counts[1:]):
            raise ValueError("VLM fusion thresholds cannot exceed the window size")
        if self.fusion_maximum_observation_age_s <= 0:
            raise ValueError("VLM fusion maximum age must be positive")
        if not 0 <= self.fusion_minimum_accepted_confidence <= 1:
            raise ValueError("VLM fusion minimum confidence must be in [0,1]")


@dataclass(frozen=True, slots=True)
class VLMHealth:
    reachable: bool
    model_available: bool
    model: str
    installed_models: tuple[str, ...] = ()

    @property
    def available(self) -> bool:
        return self.reachable and self.model_available


@dataclass(frozen=True, slots=True)
class VLMResult:
    status: VLMStatus
    terrain_type: str
    visible_landmarks: tuple[str, ...]
    target_visible: bool
    scene_confidence: float
    texture_quality: float
    scene_summary: str
    model: str

    def __post_init__(self) -> None:
        if not self.terrain_type or len(self.terrain_type) > 100:
            raise ValueError("terrain_type must contain 1 to 100 characters")
        landmarks = tuple(self.visible_landmarks)
        if any(not isinstance(item, str) or not item or len(item) > 100 for item in landmarks):
            raise ValueError("visible_landmarks must contain non-empty strings")
        if len(landmarks) > 32:
            raise ValueError("visible_landmarks cannot contain more than 32 items")
        if not isinstance(self.target_visible, bool):
            raise TypeError("target_visible must be a boolean")
        for value, name in (
            (self.scene_confidence, "scene_confidence"),
            (self.texture_quality, "texture_quality"),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be numeric")
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0,1]")
        if not self.scene_summary or len(self.scene_summary) > 1000:
            raise ValueError("scene_summary must contain 1 to 1000 characters")
        if not self.model:
            raise ValueError("model must not be empty")
        object.__setattr__(self, "visible_landmarks", landmarks)

    @classmethod
    def degraded(cls, status: VLMStatus, model: str) -> VLMResult:
        if status is VLMStatus.SUCCESS:
            raise ValueError("A degraded result cannot have SUCCESS status")
        return cls(
            status=status,
            terrain_type="UNKNOWN",
            visible_landmarks=(),
            target_visible=False,
            scene_confidence=0.0,
            texture_quality=0.0,
            scene_summary="Scene understanding unavailable",
            model=model,
        )


@dataclass(frozen=True, slots=True)
class VLMSceneState:
    status: VLMStatus = VLMStatus.PENDING
    result: VLMResult | None = None
    model: str = "NONE"
    source_frame_sequence: int = 0
    source_frame_timestamp_monotonic: float = 0.0
    inference_timestamp_monotonic: float = 0.0
    inference_latency_ms: float = 0.0
    dropped_frames: int = 0

    def __post_init__(self) -> None:
        if self.source_frame_sequence < 0 or self.dropped_frames < 0:
            raise ValueError("VLM frame counters must be non-negative")
        timing = (
            self.source_frame_timestamp_monotonic,
            self.inference_timestamp_monotonic,
            self.inference_latency_ms,
        )
        if any(not math.isfinite(value) or value < 0 for value in timing):
            raise ValueError("VLM timestamps and latency must be finite and non-negative")
        if self.status is VLMStatus.SUCCESS:
            if self.result is None or self.result.status is not VLMStatus.SUCCESS:
                raise ValueError("Successful VLM scene state requires a successful result")
        elif self.result is not None:
            raise ValueError("Non-successful VLM scene state cannot publish a result")

    @property
    def fresh(self) -> bool:
        return self.status is VLMStatus.SUCCESS and self.result is not None

    def refreshed(
        self,
        stale_after_s: float,
        now_monotonic: float,
    ) -> VLMSceneState:
        if not self.fresh:
            return self
        if now_monotonic - self.inference_timestamp_monotonic <= stale_after_s:
            return self
        return replace(self, status=VLMStatus.STALE, result=None)
