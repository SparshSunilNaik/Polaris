from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.ego_state.confidence import ConfidenceReport, SafetyMode  # noqa: E402
from polaris.ego_state.models import NavigationState, VisualState  # noqa: E402
from polaris.evaluation.viewpoint import (  # noqa: E402
    BenchmarkRun,
    ViewpointSample,
    ViewpointSpec,
    abort_reason,
    aggregate_viewpoint,
    sample_from_state,
    write_outputs,
)
from polaris.perception.models import VisualCondition  # noqa: E402
from polaris.perception.depth.models import DepthState  # noqa: E402


def confidence(mode: SafetyMode = SafetyMode.NORMAL) -> ConfidenceReport:
    return ConfidenceReport(
        visual_health=0.9,
        camera_freshness=1.0,
        exposure_quality=1.0,
        texture_feature_quality=0.9,
        localization_health=1.0,
        telemetry_health=1.0,
        battery_health=1.0,
        motion_health=1.0,
        overall_health=0.9,
        mode=mode,
        reasons=[],
        critical_overrides=[],
    )


def sample(
    health: float,
    condition: VisualCondition = VisualCondition.HEALTHY,
    update: float = 1.0,
) -> ViewpointSample:
    return ViewpointSample(
        measured_yaw_deg=89.0,
        north_m=0.1,
        east_m=-0.1,
        down_m=-2.5,
        altitude_m=2.5,
        horizontal_drift_m=0.14,
        fps=30.0,
        frame_age_ms=20.0,
        brightness=150.0,
        contrast=50.0,
        blur_score=100.0,
        texture_score=10.0,
        feature_count=200,
        feature_coverage=0.75,
        overexposed_ratio=0.02,
        underexposed_ratio=0.01,
        visual_health_score=health,
        visual_condition=condition,
        reasons=("example",),
        localization_health=1.0,
        telemetry_health=1.0,
        safety_mode=SafetyMode.NORMAL,
        visual_update_monotonic=update,
    )


class ViewpointEvaluationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = ViewpointSpec("east", 0.0, 90.0)

    def test_aggregation_uses_median_and_worst_score(self) -> None:
        samples = [sample(0.8), sample(0.82), sample(0.1)]
        result = aggregate_viewpoint(self.spec, samples)
        self.assertEqual(result.median_visual_health_score, 0.8)
        self.assertEqual(result.worst_visual_health_score, 0.1)
        self.assertEqual(result.sample_count, 3)
        self.assertAlmostEqual(result.measured_yaw_deg, 89.0)

    def test_dominant_condition_prefers_worse_condition_on_tie(self) -> None:
        result = aggregate_viewpoint(
            self.spec,
            [
                sample(0.8, VisualCondition.HEALTHY),
                sample(0.5, VisualCondition.DEGRADED),
            ],
        )
        self.assertEqual(
            result.dominant_visual_condition,
            VisualCondition.DEGRADED.value,
        )

    def test_stale_frame_is_rejected(self) -> None:
        now = monotonic()
        state = NavigationState(
            visual=VisualState(
                camera_connected=False,
                stale=True,
                last_visual_update_monotonic=now,
            ),
            local_position_ok=True,
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
        )
        self.assertIsNone(sample_from_state(state, confidence(), 0.0, 0.0))

    def test_emergency_abort_decision(self) -> None:
        now = monotonic()
        state = NavigationState(
            visual=VisualState(
                camera_connected=True,
                stale=False,
                last_visual_update_monotonic=now,
            ),
            local_position_ok=True,
            in_air=True,
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
        )
        self.assertTrue(
            abort_reason(state, confidence(SafetyMode.EMERGENCY), True).startswith(
                "Confidence Engine entered EMERGENCY:"
            )
        )
        self.assertEqual(
            abort_reason(state, confidence(), False),
            "PX4 disconnected",
        )

    def test_required_depth_failure_aborts_when_armed_before_takeoff(self) -> None:
        now = monotonic()
        state = NavigationState(
            visual=VisualState(
                camera_connected=True,
                stale=False,
                last_visual_update_monotonic=now,
            ),
            depth=DepthState(model_name="test", available=False, stale=True),
            depth_required_for_motion=True,
            local_position_ok=True,
            armed=True,
            in_air=False,
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
        )
        self.assertEqual(
            abort_reason(state, confidence(), True),
            "Required depth failed while vehicle is armed",
        )

    def test_json_and_csv_serialization(self) -> None:
        result = aggregate_viewpoint(self.spec, [sample(0.8), sample(0.9)])
        run = BenchmarkRun(
            started_at_utc="2026-01-01T00:00:00+00:00",
            completed_at_utc="2026-01-01T00:01:00+00:00",
            hover_reference_ned_m={"north": 0.0, "east": 0.0, "down": -2.5},
            max_horizontal_drift_m=0.14,
            viewpoints=(result,),
        )
        with tempfile.TemporaryDirectory() as directory:
            json_path, csv_path = write_outputs(run, directory, "test")
            payload = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["viewpoints"][0]["name"], "east")
            self.assertIsNotNone(csv_path)
            with csv_path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["name"], "east")
            self.assertEqual(rows[0]["position_down_m"], "-2.5")


if __name__ == "__main__":
    unittest.main()
