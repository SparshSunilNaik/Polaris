from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from polaris.perception.depth.models import (  # noqa: E402
    DepthCondition,
    DepthConfig,
    DepthSourceHealth,
)
from polaris.perception.depth.processor import DepthProcessor  # noqa: E402


HEALTHY_SOURCE = DepthSourceHealth(
    visual_health_score=0.9,
    visual_condition="HEALTHY",
    texture_score=10.0,
    overexposed_ratio=0.0,
    underexposed_ratio=0.0,
    reasons=(),
)


def process(processor: DepthProcessor, depth: np.ndarray, sequence: int = 1):
    return processor.process(
        depth,
        source_frame_sequence=sequence,
        source_frame_age_ms=20.0,
        inference_timestamp_monotonic=10.0 + sequence,
        inference_latency_ms=50.0,
        warmup_latency_ms=100.0,
        inference_fps=3.0,
        model_name="test-model",
        model_revision="test-revision",
        device="cpu",
        input_width=90,
        input_height=90,
        source_health=HEALTHY_SOURCE,
    )


class DepthProcessingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = replace(DepthConfig(), persistence_count=1)
        self.processor = DepthProcessor(self.config)

    def scene(self, region: tuple[slice, slice] | None = None) -> np.ndarray:
        depth = np.ones((90, 90), dtype=np.float32)
        if region is not None:
            depth[region] = 10.0
        return depth

    def test_uniform_flat_output_is_critical(self) -> None:
        result = process(self.processor, self.scene())
        self.assertEqual(result.state.condition, DepthCondition.CRITICAL)
        self.assertIn("dynamic range", result.state.reasons[0])

    def test_centre_obstacle(self) -> None:
        result = process(
            self.processor,
            self.scene((slice(30, 60), slice(30, 60))),
        )
        self.assertEqual(result.state.nearest_region, "CENTRE")
        self.assertGreater(result.state.centre_risk, 0.95)
        self.assertLessEqual(result.state.relative_dynamic_range, 2.0)

    def test_directional_obstacles(self) -> None:
        cases = {
            "LEFT": (slice(None), slice(0, 30)),
            "RIGHT": (slice(None), slice(60, 90)),
            "UP": (slice(0, 30), slice(None)),
            "DOWN": (slice(60, 90), slice(None)),
        }
        for expected, region in cases.items():
            with self.subTest(expected=expected):
                result = process(DepthProcessor(self.config), self.scene(region))
                self.assertIn(expected, result.state.nearest_region)

    def test_isolated_outlier_does_not_create_maximum_region_risk(self) -> None:
        depth = np.tile(np.linspace(1.0, 2.0, 90), (90, 1)).astype(np.float32)
        baseline = process(DepthProcessor(self.config), depth).state.risk_grid
        depth[45, 45] = 1000.0
        outlier = process(DepthProcessor(self.config), depth).state.risk_grid
        self.assertLess(abs(outlier[1][1] - baseline[1][1]), 0.05)

    def test_nan_and_inf_are_critical(self) -> None:
        for value in (np.nan, np.inf):
            depth = np.tile(np.linspace(1.0, 2.0, 90), (90, 1)).astype(np.float32)
            depth[0, 0] = value
            result = process(DepthProcessor(self.config), depth)
            self.assertEqual(result.state.condition, DepthCondition.CRITICAL)
            self.assertFalse(result.state.available)
            self.assertEqual(result.state.model_name, "test-model")

    def test_temporal_smoothing_and_stability(self) -> None:
        processor = DepthProcessor(replace(self.config, temporal_alpha=0.5))
        first = process(
            processor,
            self.scene((slice(30, 60), slice(30, 60))),
            1,
        )
        stable = process(
            processor,
            self.scene((slice(30, 60), slice(30, 60))),
            2,
        )
        unstable = process(
            processor,
            self.scene((slice(None), slice(0, 30))),
            3,
        )
        self.assertEqual(first.state.temporal_stability, 1.0)
        self.assertGreater(stable.state.temporal_stability, 0.99)
        self.assertLess(unstable.state.temporal_stability, 0.8)
        self.assertGreater(unstable.state.centre_risk, 0.0)

    def test_stale_depth_state_becomes_critical(self) -> None:
        state = process(
            self.processor,
            self.scene((slice(30, 60), slice(30, 60))),
        ).state
        stale = state.refreshed(100.0, now_monotonic=12.0)
        self.assertTrue(stale.stale)
        self.assertEqual(stale.condition, DepthCondition.CRITICAL)

    def test_degraded_source_degrades_depth_health(self) -> None:
        source = replace(
            HEALTHY_SOURCE,
            visual_health_score=0.55,
            visual_condition="DEGRADED",
        )
        result = self.processor.process(
            self.scene((slice(30, 60), slice(30, 60))),
            source_frame_sequence=1,
            source_frame_age_ms=20.0,
            inference_timestamp_monotonic=1.0,
            inference_latency_ms=50.0,
            warmup_latency_ms=100.0,
            inference_fps=3.0,
            model_name="test",
            model_revision="test",
            device="cpu",
            input_width=90,
            input_height=90,
            source_health=source,
        )
        self.assertEqual(result.state.condition, DepthCondition.DEGRADED)


if __name__ == "__main__":
    unittest.main()
