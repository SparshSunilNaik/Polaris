from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    DesiredMotion,
    EmergencyConfirmationState,
    ReactiveAvoidanceController,
    SafetySupervisor,
    SupervisorMode,
)
from polaris.ego_state.confidence import ConfidenceEngine, SafetyMode  # noqa: E402
from test_reactive_avoidance import CLEAR, navigation  # noqa: E402


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = AvoidanceConfig()
        self.supervisor = SafetySupervisor(self.config)
        self.now = monotonic()

    def inputs(self):
        state = navigation(CLEAR, 1)
        state.armed = True
        state.in_air = True
        state.flight_mode = "OFFBOARD"
        state.depth_required_for_motion = True
        command = DesiredMotion(forward_m_s=0.4, timestamp_monotonic=self.now)
        decision = ReactiveAvoidanceController(self.config).update(
            command,
            state,
            "NORMAL",
            self.now,
        )
        report = ConfidenceEngine().evaluate(state)
        return state, decision, report

    def supervise(self, state, decision, report, **changes):
        values = {
            "px4_connected": True,
            "autonomy_enabled": True,
            "offboard_expected": True,
            "displacement_from_start_m": 0.0,
            "now_monotonic": self.now,
        }
        values.update(changes)
        self.supervisor.reset_control_timer()
        return self.supervisor.evaluate(decision, state, report, **values)

    def test_clear_command_passes(self) -> None:
        state, decision, report = self.inputs()
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.accepted)
        self.assertEqual(safe.supervisor_mode, SupervisorMode.PASS_THROUGH)
        self.assertEqual(safe.forward_m_s, 0.4)

    def test_stale_depth_required_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        state.depth = replace(state.depth, stale=True)
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.supervisor_mode, SupervisorMode.ABORT)
        self.assertEqual(safe.forward_m_s, 0.0)

    def test_stale_rgb_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        state.visual = replace(state.visual, stale=True, camera_connected=False)
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertIn("RGB unavailable or stale", safe.reasons)

    def test_failed_localization_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        state.local_position_ok = False
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertIn("Local position unavailable or stale", safe.reasons)

    def test_emergency_confidence_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        report = replace(report, mode=SafetyMode.EMERGENCY)
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertIn("Confidence Engine is EMERGENCY", safe.reasons)

    def test_offboard_loss_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        state.flight_mode = "HOLD"
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertIn("Offboard mode was lost", safe.reasons)

    def test_disconnect_is_hard_abort(self) -> None:
        state, decision, report = self.inputs()
        safe = self.supervise(state, decision, report, px4_connected=False)
        self.assertTrue(safe.hard_override)
        self.assertIn("PX4 disconnected", safe.reasons)

    def test_monitoring_only_disabled_autonomy_hovers(self) -> None:
        state, decision, report = self.inputs()
        safe = self.supervise(state, decision, report, autonomy_enabled=False)
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.supervisor_mode, SupervisorMode.HOVER)

    def test_emergency_centre_risk_is_hard_abort(self) -> None:
        state, _, report = self.inputs()
        grid = ((0.1, 0.1, 0.1), (0.1, 0.95, 0.1), (1.0, 1.0, 1.0))
        state.depth = replace(state.depth, risk_grid=grid, source_frame_sequence=2)
        command = DesiredMotion(forward_m_s=0.4, timestamp_monotonic=self.now)
        decision = ReactiveAvoidanceController(self.config).update(
            command,
            state,
            report.mode.name,
            self.now,
        )
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.supervisor_mode, SupervisorMode.ABORT)

    def test_active_perception_single_emergency_frame_is_suspected(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        safe = self.supervise(state, decision, report)
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.SUSPECTED)
        self.assertEqual(safe.emergency_confirmation.observed_frames, 1)

    def test_suspected_emergency_holds_translation_at_zero(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        safe = self.supervise(state, decision, report)
        self.assertEqual((safe.forward_m_s, safe.right_m_s, safe.down_m_s), (0.0, 0.0, 0.0))

    def test_consistent_fresh_emergency_frames_confirm(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        state, decision, report = self.emergency_inputs(sequence=3, now=self.now + 0.3)
        safe = self.supervise(state, decision, report, now_monotonic=self.now + 0.3)
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CONFIRMED)

    def test_cleared_emergency_returns_to_normal_processing(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        state, decision, report = self.emergency_inputs(
            sequence=3,
            risk=0.2,
            now=self.now + 0.3,
        )
        safe = self.supervise(state, decision, report, now_monotonic=self.now + 0.3)
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CLEAR)

    def test_stale_emergency_observation_does_not_count(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        state.depth = replace(state.depth, stale=True)
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CLEAR)

    def test_duplicate_emergency_frame_does_not_count_twice(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        safe = self.supervise(
            state,
            decision,
            report,
            now_monotonic=self.now + 0.2,
        )
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.observed_frames, 1)

    def test_duplicate_inference_timestamp_does_not_count_twice(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        state, decision, report = self.emergency_inputs(sequence=3)
        safe = self.supervise(
            state,
            decision,
            report,
            now_monotonic=self.now + 0.2,
        )
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.observed_frames, 1)

    def test_collision_bypasses_emergency_confirmation(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        safe = self.supervise(state, decision, report, collision_detected=True)
        self.assertTrue(safe.hard_override)
        self.assertIn("Airborne collision detected", safe.reasons)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CLEAR)

    def test_forward_motion_emergency_remains_immediate(self) -> None:
        state, decision, report = self.emergency_inputs(
            sequence=2,
            source="local-goal-advance",
            forward_m_s=0.3,
        )
        safe = self.supervise(state, decision, report)
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CONFIRMED)

    def test_active_perception_yaw_continues_while_suspected(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2, yaw_rate_deg_s=10.0)
        safe = self.supervise(state, decision, report)
        self.assertEqual(safe.yaw_rate_deg_s, 10.0)
        self.assertEqual((safe.forward_m_s, safe.right_m_s, safe.down_m_s), (0.0, 0.0, 0.0))

    def test_confirmation_timeout_fails_closed(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        safe = self.supervise(
            state,
            decision,
            report,
            now_monotonic=self.now + self.config.emergency_confirmation_timeout_s + 0.01,
        )
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.state, EmergencyConfirmationState.CONFIRMED)

    def test_confirmation_reset_isolates_missions(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2)
        self.supervise(state, decision, report)
        self.supervisor.reset()
        safe = self.supervise(state, decision, report, now_monotonic=self.now + 0.1)
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.emergency_confirmation.observed_frames, 1)

    def test_active_perception_below_emergency_preserves_prior_behavior(self) -> None:
        state, decision, report = self.emergency_inputs(sequence=2, risk=0.7, yaw_rate_deg_s=8.0)
        safe = self.supervise(state, decision, report)
        self.assertFalse(safe.hard_override)
        self.assertEqual(safe.yaw_rate_deg_s, 8.0)

    def test_observation_collision_remains_immediate(self) -> None:
        state, decision, report = self.emergency_inputs(
            sequence=2,
            risk=0.2,
            source="active-perception-observation",
        )
        safe = self.supervise(state, decision, report, collision_detected=True)
        self.assertTrue(safe.hard_override)
        self.assertIn("Airborne collision detected", safe.reasons)

    def test_confirmed_emergency_overrides_observation(self) -> None:
        state, decision, report = self.emergency_inputs(
            sequence=2,
            source="active-perception-observation",
        )
        first = self.supervise(state, decision, report)
        self.assertFalse(first.hard_override)
        state, decision, report = self.emergency_inputs(
            sequence=3,
            source="active-perception-observation",
            now=self.now + 0.3,
        )
        second = self.supervise(
            state,
            decision,
            report,
            now_monotonic=self.now + 0.3,
        )
        self.assertTrue(second.hard_override)

    def emergency_inputs(
        self,
        *,
        sequence: int,
        risk: float = 0.95,
        now: float | None = None,
        source: str = "active-perception-scan",
        forward_m_s: float = 0.0,
        yaw_rate_deg_s: float = 5.0,
    ):
        timestamp = self.now if now is None else now
        state, _, report = self.inputs()
        grid = ((0.1, 0.1, 0.1), (0.1, risk, 0.1), (1.0, 1.0, 1.0))
        state.depth = replace(
            state.depth,
            risk_grid=grid,
            centre_risk=risk,
            source_frame_sequence=sequence,
            inference_timestamp_monotonic=timestamp,
        )
        command = DesiredMotion(
            forward_m_s=forward_m_s,
            yaw_rate_deg_s=yaw_rate_deg_s,
            source=source,
            timestamp_monotonic=timestamp,
        )
        decision = ReactiveAvoidanceController(self.config).update(
            command,
            state,
            report.mode.name,
            timestamp,
        )
        return state, decision, report


if __name__ == "__main__":
    unittest.main()
