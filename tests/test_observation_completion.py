from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.mission import (  # noqa: E402
    ObservationCompletionConfig,
    ObservationEventType,
    ObservationState,
    ObservationTracker,
)
from polaris.navigation import ActivePerceptionConfig, ActivePerceptionManager  # noqa: E402
from polaris.navigation.models import RecoveryState  # noqa: E402
from test_reactive_avoidance import navigation  # noqa: E402


class ObservationCompletionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ObservationCompletionConfig(
            minimum_goal_distance_m=2.5,
            maximum_goal_distance_m=3.0,
            maximum_heading_error_deg=12.0,
            minimum_visual_health=0.35,
            minimum_depth_health=0.4,
            minimum_depth_stability=0.75,
            maximum_centre_risk=0.45,
            maximum_horizontal_speed_m_s=0.2,
            required_stable_s=1.0,
            minimum_fresh_observations=3,
            maximum_wait_s=6.0,
        )
        self.tracker = ObservationTracker("rock-observation", self.config)

    def update(
        self,
        now: float,
        sequence: int,
        *,
        distance: float = 2.8,
        heading_error: float = 5.0,
        centre_risk: float = 0.2,
        visual_health: float = 0.9,
        depth_health: float = 0.9,
        stability: float = 0.95,
        visual_stale: bool = False,
        depth_stale: bool = False,
        collision: bool = False,
        recovery_state: RecoveryState = RecoveryState.IDLE,
        inference_timestamp: float | None = None,
    ):
        state = navigation(sequence=sequence, stability=stability)
        state.visual = replace(
            state.visual,
            stale=visual_stale,
            visual_health_score=visual_health,
            last_visual_update_monotonic=now,
        )
        state.depth = replace(
            state.depth,
            stale=depth_stale,
            depth_health_score=depth_health,
            inference_timestamp_monotonic=(
                now if inference_timestamp is None else inference_timestamp
            ),
        )
        return self.tracker.update(
            state,
            goal_distance_m=distance,
            heading_error_deg=heading_error,
            centre_risk=centre_risk,
            collision_detected=collision,
            active_perception_state=recovery_state,
            now_monotonic=now,
        )

    def test_observation_does_not_complete_outside_distance_band(self) -> None:
        decision = self.update(1.0, 1, distance=3.1)
        self.assertEqual(decision.state, ObservationState.APPROACHING)
        self.assertFalse(decision.completed)

    def test_stale_perception_cannot_qualify(self) -> None:
        decision = self.update(1.0, 1, visual_stale=True, depth_stale=True)
        self.assertFalse(decision.hold_required)
        self.assertIn("stale", decision.reason.lower())

    def test_unhealthy_visual_state_cannot_qualify(self) -> None:
        decision = self.update(1.0, 1, visual_health=0.34)
        self.assertFalse(decision.hold_required)
        self.assertIn("visual", decision.reason.lower())

    def test_unhealthy_depth_state_cannot_qualify(self) -> None:
        decision = self.update(1.0, 1, depth_health=0.39)
        self.assertFalse(decision.hold_required)
        self.assertIn("depth", decision.reason.lower())

    def test_excessive_centre_risk_cannot_qualify(self) -> None:
        decision = self.update(1.0, 1, centre_risk=0.46)
        self.assertFalse(decision.hold_required)
        self.assertIn("risk", decision.reason.lower())

    def test_dwell_advances_only_on_distinct_fresh_observations(self) -> None:
        first = self.update(1.0, 1)
        duplicate = self.update(1.8, 1, inference_timestamp=1.0)
        fresh = self.update(1.9, 2)
        self.assertEqual(first.dwell_elapsed_s, 0.0)
        self.assertEqual(duplicate.dwell_elapsed_s, 0.0)
        self.assertAlmostEqual(fresh.dwell_elapsed_s, 0.9)
        self.assertFalse(fresh.completed)

    def test_invalid_condition_resets_dwell(self) -> None:
        self.update(1.0, 1)
        decision = self.update(1.5, 2, visual_health=0.2)
        self.assertEqual(decision.state, ObservationState.APPROACHING)
        self.assertEqual(decision.dwell_elapsed_s, 0.0)
        self.assertEqual(self.tracker.events[-1].event_type, ObservationEventType.DWELL_RESET)
        duplicate = self.update(1.6, 2, inference_timestamp=1.5)
        self.assertEqual(duplicate.state, ObservationState.APPROACHING)
        fresh = self.update(1.9, 3)
        self.assertEqual(fresh.state, ObservationState.OBSERVING)

    def test_stable_valid_observation_completes(self) -> None:
        self.update(1.0, 1)
        self.update(1.5, 2)
        ready = self.update(2.0, 3)
        self.assertTrue(ready.completion_ready)
        self.assertFalse(ready.completed)
        state = navigation(sequence=3, stability=0.95)
        state.depth = replace(state.depth, inference_timestamp_monotonic=2.0)
        decision = self.tracker.accept_completion(ready, state, 2.0)
        self.assertTrue(decision.completed)
        self.assertEqual(decision.state, ObservationState.OBSERVATION_COMPLETE)
        self.assertEqual(self.tracker.events[-1].event_type, ObservationEventType.COMPLETED)

    def test_elapsed_dwell_still_requires_minimum_fresh_observations(self) -> None:
        self.update(1.0, 1)
        decision = self.update(2.2, 2)
        self.assertFalse(decision.completion_ready)
        self.assertEqual(decision.fresh_observation_count, 2)

    def test_translation_is_zero_during_observation_dwell(self) -> None:
        manager = ActivePerceptionManager(ActivePerceptionConfig())
        command = manager.observation_hold_command(0.0, 1.0)
        self.assertEqual(
            (command.forward_m_s, command.right_m_s, command.down_m_s),
            (0.0, 0.0, 0.0),
        )

    def test_authorized_yaw_may_continue_during_dwell(self) -> None:
        manager = ActivePerceptionManager(ActivePerceptionConfig())
        command = manager.observation_hold_command(8.0, 1.0)
        self.assertEqual(command.yaw_rate_deg_s, 8.0)
        self.assertEqual(command.source, "active-perception-observation")

    def test_collision_invalidates_observation_immediately(self) -> None:
        decision = self.update(1.0, 1, collision=True)
        self.assertFalse(decision.completed)
        self.assertFalse(decision.hold_required)
        self.assertIn("collision", decision.reason.lower())

    def test_failed_active_perception_cannot_qualify(self) -> None:
        decision = self.update(1.0, 1, recovery_state=RecoveryState.FAILED)
        self.assertFalse(decision.hold_required)
        self.assertIn("active perception", decision.reason.lower())

    def test_observation_wait_is_bounded(self) -> None:
        self.update(1.0, 1, visual_health=0.2)
        decision = self.update(7.1, 2, visual_health=0.2)
        self.assertTrue(decision.timed_out)
        self.assertFalse(decision.hold_required)
        self.assertEqual(self.tracker.events[-1].event_type, ObservationEventType.TIMED_OUT)

    def test_reset_and_new_tracker_do_not_leak_state(self) -> None:
        self.update(1.0, 1)
        self.tracker.reset()
        other = ObservationTracker("other", self.config)
        self.assertEqual(self.tracker.state, ObservationState.APPROACHING)
        self.assertEqual(other.state, ObservationState.APPROACHING)
        self.assertEqual(self.tracker.events, [])
        self.assertEqual(other.events, [])


if __name__ == "__main__":
    unittest.main()
