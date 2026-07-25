from __future__ import annotations

import sys
import unittest
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.ego_state.confidence import ConfidenceEngine, SafetyMode  # noqa: E402
from polaris.ego_state.models import NavigationState, VisualState  # noqa: E402
from polaris.perception.models import VisualCondition  # noqa: E402
from polaris.perception.depth.models import DepthCondition, DepthState  # noqa: E402


def healthy_visual(**changes: object) -> VisualState:
    values: dict[str, object] = {
        "camera_connected": True,
        "frame_age_ms": 20.0,
        "fps": 30.0,
        "visual_health_score": 0.95,
        "visual_condition": VisualCondition.HEALTHY,
        "brightness": 128.0,
        "contrast": 50.0,
        "blur_score": 150.0,
        "texture_score": 20.0,
        "feature_count": 250,
        "feature_coverage": 0.75,
        "overexposed_ratio": 0.01,
        "underexposed_ratio": 0.01,
        "stale": False,
        "reasons": (),
        "last_visual_update_monotonic": monotonic(),
    }
    values.update(changes)
    return VisualState(**values)


def healthy_state(in_air: bool = True) -> NavigationState:
    now = monotonic()
    return NavigationState(
        visual=healthy_visual(),
        local_position_ok=True,
        global_position_ok=True,
        home_position_ok=True,
        armable=True,
        armed=in_air,
        in_air=in_air,
        battery_remaining=0.8,
        last_position_update_monotonic=now,
        last_attitude_update_monotonic=now,
    )


class ConfidenceEngineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = ConfidenceEngine()

    def test_healthy_telemetry_and_vision_is_normal(self) -> None:
        report = self.engine.evaluate(healthy_state())
        self.assertEqual(report.mode, SafetyMode.NORMAL)
        self.assertEqual(report.critical_overrides, [])

    def test_degraded_airborne_vision_is_at_least_cautious(self) -> None:
        state = healthy_state()
        state.visual = healthy_visual(
            visual_health_score=0.55,
            visual_condition=VisualCondition.DEGRADED,
            reasons=("Low contrast",),
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.CAUTIOUS)

    def test_airborne_stale_camera_is_emergency(self) -> None:
        state = healthy_state()
        state.visual = healthy_visual(
            camera_connected=False,
            stale=True,
            frame_age_ms=800.0,
            visual_condition=VisualCondition.CRITICAL,
            reasons=("Camera frame is stale",),
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertIn("Airborne camera stale or disconnected", report.critical_overrides)

    def test_ground_stale_camera_is_not_airborne_emergency(self) -> None:
        state = healthy_state(in_air=False)
        state.visual = VisualState()
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.NORMAL)
        self.assertEqual(report.critical_overrides, [])

    def test_airborne_missing_camera_is_emergency(self) -> None:
        state = healthy_state()
        state.visual = VisualState()
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertIn("Airborne camera missing", report.critical_overrides)

    def test_airborne_local_position_failure_is_emergency(self) -> None:
        state = healthy_state()
        state.local_position_ok = False
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertIn("Airborne local position failure", report.critical_overrides)

    def test_low_battery_with_healthy_vision_is_cautious(self) -> None:
        state = healthy_state()
        state.battery_remaining = 0.20
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.CAUTIOUS)
        self.assertEqual(report.battery_health, 0.5)

    def test_critical_exposure_cannot_be_hidden_by_high_score(self) -> None:
        state = healthy_state()
        state.visual = healthy_visual(
            visual_health_score=0.99,
            overexposed_ratio=0.90,
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertGreater(report.overall_health, 0.9)
        self.assertIn("Airborne camera exposure unusable", report.critical_overrides)

    def test_critical_vision_has_degraded_and_emergency_severity(self) -> None:
        state = healthy_state()
        state.visual = healthy_visual(
            visual_health_score=0.30,
            visual_condition=VisualCondition.CRITICAL,
            reasons=("Low texture",),
        )
        self.assertEqual(self.engine.evaluate(state).mode, SafetyMode.DEGRADED)

        state.visual = healthy_visual(
            visual_health_score=0.10,
            visual_condition=VisualCondition.CRITICAL,
            reasons=("No usable visual features",),
        )
        self.assertEqual(self.engine.evaluate(state).mode, SafetyMode.EMERGENCY)

    def test_required_airborne_depth_failure_is_emergency(self) -> None:
        state = healthy_state()
        state.depth_required_for_motion = True
        state.depth = DepthState(
            model_name="test-depth",
            available=False,
            stale=True,
            reasons=("Depth inference is stale",),
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertIn(
            "Required airborne depth unavailable or stale",
            report.critical_overrides,
        )

    def test_monitoring_only_depth_failure_does_not_override_flight(self) -> None:
        state = healthy_state()
        state.depth = DepthState(
            model_name="test-depth",
            available=False,
            stale=True,
            condition=DepthCondition.CRITICAL,
            reasons=("Depth model failed",),
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.NORMAL)
        self.assertEqual(report.critical_overrides, [])
        self.assertIn("Depth: Depth model failed", report.reasons)

    def test_required_critical_depth_is_airborne_emergency(self) -> None:
        state = healthy_state()
        state.depth_required_for_motion = True
        state.depth = DepthState(
            model_name="test-depth",
            available=True,
            stale=False,
            depth_health_score=0.2,
            condition=DepthCondition.CRITICAL,
            reasons=("Depth grid unusable",),
        )
        report = self.engine.evaluate(state)
        self.assertEqual(report.mode, SafetyMode.EMERGENCY)
        self.assertIn(
            "Required airborne depth is critical",
            report.critical_overrides,
        )


if __name__ == "__main__":
    unittest.main()
