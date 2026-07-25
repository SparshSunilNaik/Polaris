from __future__ import annotations

import math
import re
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from enum import Enum

from .models import VLMConfig, VLMSceneState, VLMStatus


class FusedSceneStatus(str, Enum):
    INSUFFICIENT_EVIDENCE = "INSUFFICIENT_EVIDENCE"
    STABLE = "STABLE"
    CONFLICTED = "CONFLICTED"
    STALE = "STALE"


@dataclass(frozen=True, slots=True)
class SemanticFusionConfig:
    window_size: int = 5
    maximum_observation_age_s: float = 20.0
    minimum_evidence_count: int = 3
    target_confirmation_threshold: int = 3
    target_rejection_threshold: int = 3
    minimum_accepted_confidence: float = 0.5
    landmark_support_count: int = 2

    @classmethod
    def from_vlm_config(cls, config: VLMConfig) -> SemanticFusionConfig:
        return cls(
            window_size=config.fusion_window_size,
            maximum_observation_age_s=config.fusion_maximum_observation_age_s,
            minimum_evidence_count=config.fusion_minimum_evidence_count,
            target_confirmation_threshold=config.fusion_target_confirmation_threshold,
            target_rejection_threshold=config.fusion_target_rejection_threshold,
            minimum_accepted_confidence=config.fusion_minimum_accepted_confidence,
            landmark_support_count=config.fusion_landmark_support_count,
        )

    def validate(self) -> None:
        counts = (
            self.window_size,
            self.minimum_evidence_count,
            self.target_confirmation_threshold,
            self.target_rejection_threshold,
            self.landmark_support_count,
        )
        if any(value < 1 for value in counts):
            raise ValueError("Semantic fusion counts must be positive")
        if self.minimum_evidence_count > self.window_size:
            raise ValueError("minimum_evidence_count cannot exceed window_size")
        if self.target_confirmation_threshold > self.window_size:
            raise ValueError("target confirmation threshold cannot exceed window_size")
        if self.target_rejection_threshold > self.window_size:
            raise ValueError("target rejection threshold cannot exceed window_size")
        if self.landmark_support_count > self.window_size:
            raise ValueError("landmark support count cannot exceed window_size")
        if (
            not math.isfinite(self.maximum_observation_age_s)
            or self.maximum_observation_age_s <= 0
        ):
            raise ValueError("maximum_observation_age_s must be finite and positive")
        if (
            not math.isfinite(self.minimum_accepted_confidence)
            or not 0 <= self.minimum_accepted_confidence <= 1
        ):
            raise ValueError("minimum_accepted_confidence must be in [0,1]")


@dataclass(frozen=True, slots=True)
class FusedSceneContext:
    status: FusedSceneStatus = FusedSceneStatus.INSUFFICIENT_EVIDENCE
    terrain_type: str = "UNKNOWN"
    visible_landmarks: tuple[str, ...] = ()
    target_visible: bool | None = None
    texture_quality: float = 0.0
    fused_confidence: float = 0.0
    evidence_count: int = 0
    positive_target_count: int = 0
    negative_target_count: int = 0
    source_frame_range: tuple[int, int] | None = None
    first_observation_timestamp_monotonic: float = 0.0
    last_observation_timestamp_monotonic: float = 0.0
    scene_summary: str = "Insufficient semantic evidence"
    rationale: str = "No accepted VLM observations"

    def __post_init__(self) -> None:
        if not self.terrain_type or not self.scene_summary or not self.rationale:
            raise ValueError("Fused scene labels must not be empty")
        landmarks = tuple(self.visible_landmarks)
        if any(not isinstance(item, str) or not item for item in landmarks):
            raise ValueError("Fused landmarks must contain non-empty strings")
        if self.target_visible is not None and not isinstance(self.target_visible, bool):
            raise TypeError("target_visible must be bool or None")
        for value, name in (
            (self.texture_quality, "texture_quality"),
            (self.fused_confidence, "fused_confidence"),
        ):
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f"{name} must be finite and in [0,1]")
        counts = (
            self.evidence_count,
            self.positive_target_count,
            self.negative_target_count,
        )
        if any(value < 0 for value in counts):
            raise ValueError("Fused evidence counts must be non-negative")
        timestamps = (
            self.first_observation_timestamp_monotonic,
            self.last_observation_timestamp_monotonic,
        )
        if any(not math.isfinite(value) or value < 0 for value in timestamps):
            raise ValueError("Fused observation timestamps must be non-negative")
        if self.source_frame_range is not None:
            first, last = self.source_frame_range
            if first < 0 or last < first:
                raise ValueError("source_frame_range must be ordered and non-negative")
        object.__setattr__(self, "visible_landmarks", landmarks)

    @property
    def meaningful_signature(self) -> tuple[object, ...]:
        return (
            self.status.value,
            _normalize(self.terrain_type),
            tuple(sorted(_normalize(item) for item in self.visible_landmarks)),
            self.target_visible,
        )


@dataclass(frozen=True, slots=True)
class _Observation:
    sequence: int
    source_timestamp: float
    terrain_type: str
    landmarks: tuple[str, ...]
    target_visible: bool
    confidence: float
    texture_quality: float
    summary: str


class SemanticFusion:
    """Synchronous bounded fusion of immutable successful VLM observations."""

    def __init__(self, config: SemanticFusionConfig | None = None) -> None:
        self.config = config or SemanticFusionConfig()
        self.config.validate()
        self._observations: deque[_Observation] = deque()
        self.reset()

    def reset(self, mission_epoch_monotonic: float = 0.0) -> FusedSceneContext:
        if not math.isfinite(mission_epoch_monotonic) or mission_epoch_monotonic < 0:
            raise ValueError("mission_epoch_monotonic must be finite and non-negative")
        self._observations.clear()
        self._mission_epoch = mission_epoch_monotonic
        self._newest_sequence = -1
        self._newest_source_timestamp = mission_epoch_monotonic
        self._ever_had_evidence = False
        self._context = FusedSceneContext()
        return self._context

    def update(
        self,
        state: VLMSceneState | None,
        now_monotonic: float,
    ) -> FusedSceneContext:
        if not math.isfinite(now_monotonic) or now_monotonic < 0:
            raise ValueError("now_monotonic must be finite and non-negative")
        self._evict_old(now_monotonic)
        if self._accepts(state, now_monotonic):
            assert state is not None and state.result is not None
            result = state.result
            self._observations.append(
                _Observation(
                    sequence=state.source_frame_sequence,
                    source_timestamp=state.source_frame_timestamp_monotonic,
                    terrain_type=result.terrain_type,
                    landmarks=result.visible_landmarks,
                    target_visible=result.target_visible,
                    confidence=result.scene_confidence,
                    texture_quality=result.texture_quality,
                    summary=result.scene_summary,
                )
            )
            self._newest_sequence = state.source_frame_sequence
            self._newest_source_timestamp = state.source_frame_timestamp_monotonic
            self._ever_had_evidence = True
            while len(self._observations) > self.config.window_size:
                self._observations.popleft()
        self._context = self._fuse()
        return self._context

    def snapshot(self, now_monotonic: float | None = None) -> FusedSceneContext:
        if now_monotonic is not None:
            self._evict_old(now_monotonic)
            self._context = self._fuse()
        return self._context

    @property
    def observation_count(self) -> int:
        return len(self._observations)

    def _accepts(
        self,
        state: VLMSceneState | None,
        now_monotonic: float,
    ) -> bool:
        if (
            state is None
            or state.status is not VLMStatus.SUCCESS
            or state.result is None
            or state.result.scene_confidence < self.config.minimum_accepted_confidence
        ):
            return False
        source_time = state.source_frame_timestamp_monotonic
        if source_time < self._mission_epoch:
            return False
        if now_monotonic - source_time > self.config.maximum_observation_age_s:
            return False
        if state.source_frame_sequence <= self._newest_sequence:
            return False
        if source_time < self._newest_source_timestamp:
            return False
        return True

    def _evict_old(self, now_monotonic: float) -> None:
        cutoff = now_monotonic - self.config.maximum_observation_age_s
        while self._observations and self._observations[0].source_timestamp < cutoff:
            self._observations.popleft()

    def _fuse(self) -> FusedSceneContext:
        observations = tuple(self._observations)
        if not observations:
            if self._ever_had_evidence:
                return FusedSceneContext(
                    status=FusedSceneStatus.STALE,
                    scene_summary="Fused semantic evidence expired",
                    rationale="All accepted VLM observations exceeded the age bound",
                )
            return FusedSceneContext()

        evidence_count = len(observations)
        positive = sum(item.target_visible for item in observations)
        negative = evidence_count - positive
        if evidence_count < self.config.minimum_evidence_count:
            status = FusedSceneStatus.INSUFFICIENT_EVIDENCE
            target_visible = None
            rationale = (
                f"{evidence_count}/{self.config.minimum_evidence_count} observations available"
            )
        elif positive and negative:
            status = FusedSceneStatus.CONFLICTED
            target_visible = None
            rationale = f"Conflicting target evidence: {positive} positive, {negative} negative"
        elif positive >= self.config.target_confirmation_threshold:
            status = FusedSceneStatus.STABLE
            target_visible = True
            rationale = f"Target confirmed by {positive} observations"
        elif negative >= self.config.target_rejection_threshold:
            status = FusedSceneStatus.STABLE
            target_visible = False
            rationale = f"Target rejected by {negative} observations"
        else:
            status = FusedSceneStatus.INSUFFICIENT_EVIDENCE
            target_visible = None
            rationale = "Target evidence has not reached a confirmation threshold"

        terrain_weights: dict[str, float] = defaultdict(float)
        terrain_counts: Counter[str] = Counter()
        terrain_labels: dict[str, str] = {}
        for item in observations:
            key = _normalize(item.terrain_type)
            terrain_weights[key] += item.confidence
            terrain_counts[key] += 1
            terrain_labels[key] = item.terrain_type
        terrain_key = max(
            terrain_weights,
            key=lambda key: (terrain_weights[key], terrain_counts[key], key),
        )
        terrain_type = terrain_labels[terrain_key]
        total_weight = sum(item.confidence for item in observations)
        terrain_support = terrain_weights[terrain_key] / total_weight

        landmark_counts: Counter[str] = Counter()
        landmark_labels: dict[str, str] = {}
        for item in observations:
            for landmark in {_normalize(value): value for value in item.landmarks}.items():
                key, label = landmark
                landmark_counts[key] += 1
                landmark_labels[key] = label
        landmarks = tuple(
            landmark_labels[key]
            for key in sorted(landmark_counts)
            if landmark_counts[key] >= self.config.landmark_support_count
        )

        texture_quality = sum(
            item.texture_quality * item.confidence for item in observations
        ) / total_weight
        target_support = max(positive, negative) / evidence_count
        fused_confidence = min(item.confidence for item in observations) * min(
            terrain_support,
            target_support,
        )
        summary = observations[-1].summary
        for item in reversed(observations):
            if _normalize(item.terrain_type) != terrain_key:
                continue
            if target_visible is None or item.target_visible is target_visible:
                summary = item.summary
                break

        return FusedSceneContext(
            status=status,
            terrain_type=terrain_type,
            visible_landmarks=landmarks,
            target_visible=target_visible,
            texture_quality=texture_quality,
            fused_confidence=fused_confidence,
            evidence_count=evidence_count,
            positive_target_count=positive,
            negative_target_count=negative,
            source_frame_range=(observations[0].sequence, observations[-1].sequence),
            first_observation_timestamp_monotonic=observations[0].source_timestamp,
            last_observation_timestamp_monotonic=observations[-1].source_timestamp,
            scene_summary=summary,
            rationale=rationale,
        )


def _normalize(value: str) -> str:
    return " ".join(re.sub(r"[\W_]+", " ", value.casefold()).split())
