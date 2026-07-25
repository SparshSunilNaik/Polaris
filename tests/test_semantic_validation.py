from __future__ import annotations

import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.ego_context import (  # noqa: E402
    DeterministicObservationOutcome,
    SemanticEvidenceSource,
    SemanticEvidenceFreshness,
    SemanticObservationValidator,
    SemanticValidationClassification,
    normalize_target_label,
)
from polaris.vlm import (  # noqa: E402
    FusedSceneContext,
    FusedSceneStatus,
    VLMResult,
    VLMSceneState,
    VLMStatus,
)


def scene(
    *,
    target_visible: bool,
    landmarks: tuple[str, ...],
    confidence: float = 0.9,
) -> VLMSceneState:
    result = VLMResult(
        status=VLMStatus.SUCCESS,
        terrain_type="snowfield",
        visible_landmarks=landmarks,
        target_visible=target_visible,
        scene_confidence=confidence,
        texture_quality=0.7,
        scene_summary="Structured scene evidence",
        model="qwen2.5vl:7b",
    )
    return VLMSceneState(
        status=VLMStatus.SUCCESS,
        result=result,
        model=result.model,
        source_frame_sequence=4,
        source_frame_timestamp_monotonic=1.9,
        inference_timestamp_monotonic=2.0,
        inference_latency_ms=100.0,
    )


class SemanticObservationValidatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.validator = SemanticObservationValidator(minimum_scene_confidence=0.5)

    def validate(self, outcome, vlm, expected="rock"):
        return self.validator.validate(
            waypoint_id="rock-observation",
            expected_target_label=expected,
            deterministic_outcome=outcome,
            vlm=vlm,
            timestamp_monotonic=3.0,
        )

    def test_completion_and_visible_target_agree(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=True, landmarks=("rock",)),
        )
        self.assertEqual(record.classification, SemanticValidationClassification.AGREEMENT)

    def test_completion_and_absent_target_disagree(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=False, landmarks=()),
        )
        self.assertEqual(record.classification, SemanticValidationClassification.DISAGREEMENT)

    def test_failed_observation_and_absent_target_agree(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.FAILED,
            scene(target_visible=False, landmarks=("station",)),
        )
        self.assertEqual(record.classification, SemanticValidationClassification.AGREEMENT)

    def test_failed_observation_and_visible_target_disagree(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.FAILED,
            scene(target_visible=True, landmarks=("rock",)),
        )
        self.assertEqual(record.classification, SemanticValidationClassification.DISAGREEMENT)

    def test_unavailable_vlm_has_no_evidence(self) -> None:
        unavailable = VLMSceneState(
            status=VLMStatus.UNAVAILABLE,
            model="qwen2.5vl:7b",
            inference_timestamp_monotonic=2.0,
        )
        record = self.validate(DeterministicObservationOutcome.COMPLETED, unavailable)
        self.assertEqual(record.classification, SemanticValidationClassification.NO_EVIDENCE)
        self.assertEqual(record.result_freshness, SemanticEvidenceFreshness.UNAVAILABLE)

    def test_stale_vlm_is_rejected(self) -> None:
        stale = VLMSceneState(
            status=VLMStatus.STALE,
            model="qwen2.5vl:7b",
            inference_timestamp_monotonic=2.0,
        )
        record = self.validate(DeterministicObservationOutcome.COMPLETED, stale)
        self.assertEqual(record.classification, SemanticValidationClassification.STALE_EVIDENCE)
        self.assertEqual(record.result_freshness, SemanticEvidenceFreshness.STALE)

    def test_low_confidence_vlm_is_not_compared(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=True, landmarks=("rock",), confidence=0.49),
        )
        self.assertEqual(record.classification, SemanticValidationClassification.LOW_CONFIDENCE)

    def test_unspecified_target_is_classified_explicitly(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=True, landmarks=("rock",)),
            expected=None,
        )
        self.assertEqual(record.classification, SemanticValidationClassification.TARGET_UNSPECIFIED)

    def test_landmark_normalization_is_exact_after_case_and_punctuation(self) -> None:
        self.assertEqual(normalize_target_label(" Research-Station "), "research station")
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=True, landmarks=("RESEARCH station",)),
            expected="research-station",
        )
        self.assertEqual(record.classification, SemanticValidationClassification.AGREEMENT)

    def test_validation_record_is_immutable(self) -> None:
        record = self.validate(
            DeterministicObservationOutcome.COMPLETED,
            scene(target_visible=True, landmarks=("rock",)),
        )
        with self.assertRaises(FrozenInstanceError):
            record.rationale = "changed"

    def test_stable_fused_evidence_is_preferred(self) -> None:
        fused = FusedSceneContext(
            status=FusedSceneStatus.STABLE,
            terrain_type="snowfield",
            visible_landmarks=("rock",),
            target_visible=True,
            texture_quality=0.7,
            fused_confidence=0.8,
            evidence_count=3,
            positive_target_count=3,
            source_frame_range=(1, 3),
            first_observation_timestamp_monotonic=1.0,
            last_observation_timestamp_monotonic=3.0,
            scene_summary="Stable rock evidence",
            rationale="Target confirmed by 3 observations",
        )
        record = self.validator.validate(
            waypoint_id="rock-observation",
            expected_target_label="rock",
            deterministic_outcome=DeterministicObservationOutcome.COMPLETED,
            vlm=scene(target_visible=False, landmarks=()),
            fused_scene=fused,
            timestamp_monotonic=4.0,
        )
        self.assertEqual(record.classification, SemanticValidationClassification.AGREEMENT)
        self.assertEqual(record.evidence_source, SemanticEvidenceSource.FUSED)

    def test_insufficient_fusion_falls_back_to_single_frame(self) -> None:
        record = self.validator.validate(
            waypoint_id="rock-observation",
            expected_target_label="rock",
            deterministic_outcome=DeterministicObservationOutcome.COMPLETED,
            vlm=scene(target_visible=True, landmarks=("rock",)),
            fused_scene=FusedSceneContext(evidence_count=1),
            timestamp_monotonic=4.0,
        )
        self.assertEqual(record.classification, SemanticValidationClassification.AGREEMENT)
        self.assertEqual(record.evidence_source, SemanticEvidenceSource.SINGLE_FRAME)


if __name__ == "__main__":
    unittest.main()
