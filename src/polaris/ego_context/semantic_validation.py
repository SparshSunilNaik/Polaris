from __future__ import annotations

import math
import re
from dataclasses import dataclass
from enum import Enum

from polaris.vlm import (
    FusedSceneContext,
    FusedSceneStatus,
    VLMSceneState,
    VLMStatus,
)


class SemanticValidationClassification(str, Enum):
    AGREEMENT = "AGREEMENT"
    DISAGREEMENT = "DISAGREEMENT"
    NO_EVIDENCE = "NO_EVIDENCE"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    STALE_EVIDENCE = "STALE_EVIDENCE"
    TARGET_UNSPECIFIED = "TARGET_UNSPECIFIED"


class DeterministicObservationOutcome(str, Enum):
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class SemanticEvidenceFreshness(str, Enum):
    FRESH = "FRESH"
    STALE = "STALE"
    UNAVAILABLE = "UNAVAILABLE"


class SemanticEvidenceSource(str, Enum):
    FUSED = "FUSED"
    SINGLE_FRAME = "SINGLE_FRAME"


@dataclass(frozen=True, slots=True)
class SemanticValidationRecord:
    waypoint_id: str
    expected_target_label: str | None
    deterministic_outcome: DeterministicObservationOutcome
    vlm_target_visible: bool | None
    visible_landmarks: tuple[str, ...]
    scene_confidence: float | None
    result_freshness: SemanticEvidenceFreshness
    classification: SemanticValidationClassification
    rationale: str
    timestamp_monotonic: float
    evidence_source: SemanticEvidenceSource = SemanticEvidenceSource.SINGLE_FRAME

    def __post_init__(self) -> None:
        if not self.waypoint_id or not self.rationale:
            raise ValueError("Semantic validation labels must not be empty")
        if self.expected_target_label is not None and not self.expected_target_label:
            raise ValueError("Expected target label must be non-empty when present")
        if self.vlm_target_visible is not None and not isinstance(
            self.vlm_target_visible, bool
        ):
            raise TypeError("vlm_target_visible must be a boolean when present")
        landmarks = tuple(self.visible_landmarks)
        if any(not isinstance(item, str) or not item for item in landmarks):
            raise ValueError("Visible landmarks must contain non-empty strings")
        if self.scene_confidence is not None:
            if (
                not math.isfinite(self.scene_confidence)
                or not 0 <= self.scene_confidence <= 1
            ):
                raise ValueError("scene_confidence must be finite and in [0,1]")
        if not math.isfinite(self.timestamp_monotonic) or self.timestamp_monotonic < 0:
            raise ValueError("timestamp_monotonic must be finite and non-negative")
        object.__setattr__(self, "visible_landmarks", landmarks)


def normalize_target_label(label: str) -> str:
    return " ".join(re.sub(r"[\W_]+", " ", label.casefold()).split())


class SemanticObservationValidator:
    """Compares outcomes with VLM evidence without producing mission actions."""

    def __init__(self, minimum_scene_confidence: float = 0.5) -> None:
        if (
            not math.isfinite(minimum_scene_confidence)
            or not 0 <= minimum_scene_confidence <= 1
        ):
            raise ValueError("minimum_scene_confidence must be finite and in [0,1]")
        self.minimum_scene_confidence = minimum_scene_confidence

    def validate(
        self,
        *,
        waypoint_id: str,
        expected_target_label: str | None,
        deterministic_outcome: DeterministicObservationOutcome,
        vlm: VLMSceneState | None,
        timestamp_monotonic: float,
        fused_scene: FusedSceneContext | None = None,
    ) -> SemanticValidationRecord:
        source = SemanticEvidenceSource.SINGLE_FRAME
        freshness = self._evidence_freshness(vlm)
        target_visible = None
        landmarks: tuple[str, ...] = ()
        confidence = None
        unavailable_rationale = "No fresh successful VLM evidence is available"
        if fused_scene is not None and fused_scene.status is FusedSceneStatus.STABLE:
            source = SemanticEvidenceSource.FUSED
            freshness = SemanticEvidenceFreshness.FRESH
            target_visible = fused_scene.target_visible
            landmarks = fused_scene.visible_landmarks
            confidence = fused_scene.fused_confidence
        elif fused_scene is not None and fused_scene.status is FusedSceneStatus.CONFLICTED:
            source = SemanticEvidenceSource.FUSED
            freshness = SemanticEvidenceFreshness.FRESH
            landmarks = fused_scene.visible_landmarks
            confidence = fused_scene.fused_confidence
            unavailable_rationale = "Fused semantic evidence is conflicted"
        elif fused_scene is not None and fused_scene.status is FusedSceneStatus.STALE:
            source = SemanticEvidenceSource.FUSED
            freshness = SemanticEvidenceFreshness.STALE
            unavailable_rationale = "Fused semantic evidence is stale"
        elif vlm is not None and vlm.status is VLMStatus.SUCCESS and vlm.result is not None:
            target_visible = vlm.result.target_visible
            landmarks = vlm.result.visible_landmarks
            confidence = vlm.result.scene_confidence

        expected = None
        if isinstance(expected_target_label, str):
            expected = expected_target_label.strip() or None
        if expected is None:
            return self._record(
                waypoint_id,
                None,
                deterministic_outcome,
                target_visible,
                landmarks,
                confidence,
                freshness,
                source,
                SemanticValidationClassification.TARGET_UNSPECIFIED,
                "Observation waypoint has no expected target label",
                timestamp_monotonic,
            )
        if freshness is SemanticEvidenceFreshness.STALE:
            return self._record(
                waypoint_id,
                expected,
                deterministic_outcome,
                target_visible,
                landmarks,
                confidence,
                SemanticEvidenceFreshness.STALE,
                source,
                SemanticValidationClassification.STALE_EVIDENCE,
                "Latest VLM evidence is stale and was not used",
                timestamp_monotonic,
            )
        evidence_available = target_visible is not None and confidence is not None
        if not evidence_available:
            return self._record(
                waypoint_id,
                expected,
                deterministic_outcome,
                target_visible,
                landmarks,
                confidence,
                freshness,
                source,
                SemanticValidationClassification.NO_EVIDENCE,
                unavailable_rationale,
                timestamp_monotonic,
            )

        assert confidence is not None
        if confidence < self.minimum_scene_confidence:
            return self._record(
                waypoint_id,
                expected,
                deterministic_outcome,
                target_visible,
                landmarks,
                confidence,
                SemanticEvidenceFreshness.FRESH,
                source,
                SemanticValidationClassification.LOW_CONFIDENCE,
                "Fresh VLM evidence is below the confidence threshold",
                timestamp_monotonic,
            )

        expected_normalized = normalize_target_label(expected)
        normalized_landmarks = {
            normalize_target_label(item) for item in landmarks
        }
        label_matched = expected_normalized in normalized_landmarks
        semantic_visible = bool(target_visible) and label_matched
        deterministic_visible = (
            deterministic_outcome is DeterministicObservationOutcome.COMPLETED
        )
        classification = (
            SemanticValidationClassification.AGREEMENT
            if semantic_visible == deterministic_visible
            else SemanticValidationClassification.DISAGREEMENT
        )
        relation = (
            "agrees with"
            if classification is SemanticValidationClassification.AGREEMENT
            else "disagrees with"
        )
        evidence = (
            "expected target confirmed visible"
            if semantic_visible
            else "expected target not confirmed visible"
        )
        rationale = (
            f"VLM {evidence} and {relation} deterministic "
            f"{deterministic_outcome.value.lower()} outcome"
        )
        return self._record(
            waypoint_id,
            expected,
            deterministic_outcome,
            target_visible,
            landmarks,
            confidence,
            SemanticEvidenceFreshness.FRESH,
            source,
            classification,
            rationale,
            timestamp_monotonic,
        )

    @staticmethod
    def _evidence_freshness(vlm: VLMSceneState | None) -> SemanticEvidenceFreshness:
        if vlm is not None and vlm.status is VLMStatus.STALE:
            return SemanticEvidenceFreshness.STALE
        if vlm is not None and vlm.status is VLMStatus.SUCCESS and vlm.result is not None:
            return SemanticEvidenceFreshness.FRESH
        return SemanticEvidenceFreshness.UNAVAILABLE

    @staticmethod
    def _record(
        waypoint_id: str,
        expected_target_label: str | None,
        deterministic_outcome: DeterministicObservationOutcome,
        vlm_target_visible: bool | None,
        visible_landmarks: tuple[str, ...],
        scene_confidence: float | None,
        freshness: SemanticEvidenceFreshness,
        evidence_source: SemanticEvidenceSource,
        classification: SemanticValidationClassification,
        rationale: str,
        timestamp_monotonic: float,
    ) -> SemanticValidationRecord:
        return SemanticValidationRecord(
            waypoint_id=waypoint_id,
            expected_target_label=expected_target_label,
            deterministic_outcome=deterministic_outcome,
            vlm_target_visible=vlm_target_visible,
            visible_landmarks=visible_landmarks,
            scene_confidence=scene_confidence,
            result_freshness=freshness,
            evidence_source=evidence_source,
            classification=classification,
            rationale=rationale,
            timestamp_monotonic=timestamp_monotonic,
        )
