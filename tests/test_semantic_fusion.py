from __future__ import annotations

import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.vlm import (  # noqa: E402
    FusedSceneStatus,
    SemanticFusion,
    SemanticFusionConfig,
    VLMResult,
    VLMSceneState,
    VLMStatus,
)


def observation(
    sequence: int,
    timestamp: float,
    *,
    terrain: str = "snowfield",
    landmarks: tuple[str, ...] = ("rock",),
    target_visible: bool = True,
    confidence: float = 0.9,
    texture: float = 0.7,
    summary: str | None = None,
) -> VLMSceneState:
    result = VLMResult(
        status=VLMStatus.SUCCESS,
        terrain_type=terrain,
        visible_landmarks=landmarks,
        target_visible=target_visible,
        scene_confidence=confidence,
        texture_quality=texture,
        scene_summary=summary or f"Scene from frame {sequence}",
        model="qwen2.5vl:7b",
    )
    return VLMSceneState(
        status=VLMStatus.SUCCESS,
        result=result,
        model=result.model,
        source_frame_sequence=sequence,
        source_frame_timestamp_monotonic=timestamp,
        inference_timestamp_monotonic=timestamp + 0.1,
        inference_latency_ms=100.0,
    )


class SemanticFusionTests(unittest.TestCase):
    def test_single_frame_is_insufficient_evidence(self) -> None:
        fused = SemanticFusion().update(observation(1, 1.0), 1.1)
        self.assertEqual(fused.status, FusedSceneStatus.INSUFFICIENT_EVIDENCE)
        self.assertIsNone(fused.target_visible)
        self.assertEqual(fused.evidence_count, 1)

    def test_three_positive_observations_confirm_target(self) -> None:
        fusion = SemanticFusion()
        for sequence in range(1, 4):
            fused = fusion.update(observation(sequence, float(sequence)), sequence + 0.1)
        self.assertEqual(fused.status, FusedSceneStatus.STABLE)
        self.assertTrue(fused.target_visible)
        self.assertEqual(fused.positive_target_count, 3)

    def test_three_negative_observations_reject_target(self) -> None:
        fusion = SemanticFusion()
        for sequence in range(1, 4):
            fused = fusion.update(
                observation(sequence, float(sequence), target_visible=False),
                sequence + 0.1,
            )
        self.assertEqual(fused.status, FusedSceneStatus.STABLE)
        self.assertFalse(fused.target_visible)
        self.assertEqual(fused.negative_target_count, 3)

    def test_contradictory_target_observations_are_conflicted(self) -> None:
        fusion = SemanticFusion()
        for sequence, visible in enumerate((True, False, True), start=1):
            fused = fusion.update(
                observation(sequence, float(sequence), target_visible=visible),
                sequence + 0.1,
            )
        self.assertEqual(fused.status, FusedSceneStatus.CONFLICTED)
        self.assertIsNone(fused.target_visible)

    def test_terrain_uses_confidence_weighted_vote(self) -> None:
        config = SemanticFusionConfig(
            minimum_evidence_count=2,
            target_confirmation_threshold=2,
            target_rejection_threshold=2,
        )
        fusion = SemanticFusion(config)
        fusion.update(observation(1, 1.0, terrain="snow", confidence=0.95), 1.1)
        fused = fusion.update(
            observation(2, 2.0, terrain="rock", confidence=0.55),
            2.1,
        )
        self.assertEqual(fused.terrain_type, "snow")

    def test_landmarks_require_repeated_normalized_evidence(self) -> None:
        fusion = SemanticFusion()
        fusion.update(observation(1, 1.0, landmarks=("Research-Station",)), 1.1)
        fusion.update(observation(2, 2.0, landmarks=("research station",)), 2.1)
        fused = fusion.update(observation(3, 3.0, landmarks=("ridge",)), 3.1)
        self.assertEqual(fused.visible_landmarks, ("research station",))

    def test_low_confidence_observation_is_excluded(self) -> None:
        fusion = SemanticFusion()
        fused = fusion.update(observation(1, 1.0, confidence=0.49), 1.1)
        self.assertEqual(fused.evidence_count, 0)

    def test_old_observations_are_evicted_as_stale(self) -> None:
        fusion = SemanticFusion(
            SemanticFusionConfig(maximum_observation_age_s=5.0)
        )
        fusion.update(observation(1, 1.0), 1.1)
        fused = fusion.update(None, 7.0)
        self.assertEqual(fused.status, FusedSceneStatus.STALE)
        self.assertEqual(fused.evidence_count, 0)

    def test_window_capacity_evicts_oldest_observation(self) -> None:
        config = SemanticFusionConfig(
            window_size=3,
            minimum_evidence_count=3,
            target_confirmation_threshold=3,
            target_rejection_threshold=3,
            landmark_support_count=2,
        )
        fusion = SemanticFusion(config)
        for sequence in range(1, 5):
            fused = fusion.update(observation(sequence, float(sequence)), sequence + 0.1)
        self.assertEqual(fusion.observation_count, 3)
        self.assertEqual(fused.source_frame_range, (2, 4))

    def test_duplicate_frame_sequence_is_ignored(self) -> None:
        fusion = SemanticFusion()
        first = fusion.update(observation(1, 1.0), 1.1)
        duplicate = fusion.update(
            observation(1, 1.2, terrain="rock field"),
            1.3,
        )
        self.assertEqual(duplicate, first)
        self.assertEqual(fusion.observation_count, 1)

    def test_out_of_order_results_do_not_regress_state(self) -> None:
        fusion = SemanticFusion()
        first = fusion.update(observation(3, 3.0), 3.1)
        older_sequence = fusion.update(observation(2, 4.0), 4.1)
        older_timestamp = fusion.update(observation(4, 2.0), 4.2)
        self.assertEqual(older_sequence, first)
        self.assertEqual(older_timestamp, first)

    def test_reset_clears_window_and_rejects_pre_mission_result(self) -> None:
        fusion = SemanticFusion()
        fusion.update(observation(8, 8.0), 8.1)
        reset = fusion.reset(10.0)
        rejected = fusion.update(observation(9, 9.0), 10.1)
        accepted = fusion.update(observation(1, 11.0), 11.1)
        self.assertEqual(reset.evidence_count, 0)
        self.assertEqual(rejected.evidence_count, 0)
        self.assertEqual(accepted.evidence_count, 1)

    def test_confidence_only_change_keeps_meaningful_signature(self) -> None:
        config = SemanticFusionConfig(landmark_support_count=3)
        fusion = SemanticFusion(config)
        first = fusion.update(observation(1, 1.0, landmarks=()), 1.1)
        second = fusion.update(
            observation(2, 2.0, landmarks=(), confidence=0.7, texture=0.2),
            2.1,
        )
        self.assertEqual(first.meaningful_signature, second.meaningful_signature)
        self.assertNotEqual(first.fused_confidence, second.fused_confidence)

    def test_fused_context_is_immutable_and_reset_is_clean(self) -> None:
        fusion = SemanticFusion()
        context = fusion.update(observation(1, 1.0), 1.1)
        with self.assertRaises(FrozenInstanceError):
            context.evidence_count = 4
        self.assertEqual(fusion.reset().evidence_count, 0)


if __name__ == "__main__":
    unittest.main()
