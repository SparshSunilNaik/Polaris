from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.navigation import (  # noqa: E402
    ActivePerceptionConfig,
    ActivePerceptionManager,
    GoalDecision,
    GoalMetrics,
    LocalGoal,
    NavigatorState,
    RecoveryState,
    ViewpointCandidate,
    ViewpointScore,
)
from polaris.control import DesiredMotion  # noqa: E402
from test_reactive_avoidance import CLEAR, navigation  # noqa: E402


def goal_decision(recovery_requested=False) -> GoalDecision:
    goal = LocalGoal("test", 5.0, 0.0, -2.5, timestamp_monotonic=1.0)
    metrics = GoalMetrics(5, 5, 0, 0, 0, 5, 0, 0, 0, 0, 5, 0, 1, 0, 0)
    return GoalDecision(
        NavigatorState.ADVANCE,
        goal,
        DesiredMotion(forward_m_s=0.3, timestamp_monotonic=1.0),
        metrics,
        recovery_requested,
        (),
        1.0,
    )


class ActivePerceptionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ActivePerceptionConfig()
        self.manager = ActivePerceptionManager(self.config)

    def test_poor_visual_health_triggers_recovery(self) -> None:
        state = navigation(CLEAR, 1)
        state.visual = replace(state.visual, visual_health_score=0.4)
        reason = self.manager.recovery_trigger(state, goal_decision(), None)
        self.assertIn("Visual health", reason)

    def test_unstable_depth_triggers_recovery(self) -> None:
        state = navigation(CLEAR, 1, stability=0.5)
        reason = self.manager.recovery_trigger(state, goal_decision(), None)
        self.assertIn("stability", reason)

    def test_healthy_view_does_not_scan(self) -> None:
        state = navigation(CLEAR, 1)
        self.assertIsNone(self.manager.recovery_trigger(state, goal_decision(), None))

    def test_no_progress_triggers_recovery(self) -> None:
        state = navigation(CLEAR, 1)
        reason = self.manager.recovery_trigger(
            state,
            goal_decision(recovery_requested=True),
            None,
        )
        self.assertIn("progress", reason)

    def test_recovery_attempt_limit(self) -> None:
        manager = ActivePerceptionManager(
            replace(self.config, maximum_recovery_attempts=1, recovery_cooldown_s=0.1)
        )
        state = navigation(CLEAR, 1)
        self.assertTrue(manager.trigger("test", state, 0.0, 1.0))
        manager.state = RecoveryState.IDLE
        manager.last_completed_at = 1.0
        self.assertFalse(manager.trigger("test", state, 0.0, 2.0))
        self.assertEqual(manager.state, RecoveryState.FAILED)

    def test_recovery_cooldown(self) -> None:
        state = navigation(CLEAR, 1)
        self.manager.last_completed_at = 10.0
        self.assertFalse(self.manager.trigger("test", state, 0.0, 11.0))
        self.assertEqual(self.manager.state, RecoveryState.IDLE)

    def test_no_acceptable_viewpoint_aborts(self) -> None:
        self.manager.state = RecoveryState.SELECTING
        self.manager.trigger_reason = "test"
        self.manager.candidates = [
            ViewpointCandidate(0, 0.2, False, 2, 0.2, 0.9, 0.9, 0.1, ("bad",))
        ]
        decision = self.manager.update(navigation(CLEAR, 1), 1.0)
        self.assertEqual(decision.state, RecoveryState.FAILED)

    def test_settle_interval_resets_when_heading_moves(self) -> None:
        state = navigation(CLEAR, 1)
        state.orientation.yaw_deg = 0.0
        self.manager.trigger("test", state, 0.0, 1.0)
        state.orientation.yaw_deg = -30.0
        self.manager.update(state, 2.0)
        self.assertGreater(self.manager.settle_started_at, 0.0)
        state.orientation.yaw_deg = -20.0
        decision = self.manager.update(state, 2.6)
        self.assertEqual(self.manager.settle_started_at, 0.0)
        self.assertNotEqual(decision.command.yaw_rate_deg_s, 0.0)

    def test_zero_degree_candidate_heading_is_preserved(self) -> None:
        self.manager.target_heading_deg = 0.0
        self.manager.base_heading_deg = 45.0
        self.manager.samples = [
            ViewpointScore(0, 0.8, True, 0.9, 0.9, 0.9, 0.9, 0.8, 0.9, 1.0, 0.0, 0.0, ()),
            ViewpointScore(0, 0.82, True, 0.9, 0.9, 0.9, 0.9, 0.8, 0.9, 1.0, 0.0, 0.0, ()),
        ]
        self.manager._finish_candidate()
        self.assertEqual(self.manager.candidates[0].heading_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
