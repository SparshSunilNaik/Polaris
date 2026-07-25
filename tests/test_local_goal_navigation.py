from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    AvoidanceMode,
    ReactiveAvoidanceController,
)
from polaris.ego_state.models import NavigationState, Orientation, Vector3  # noqa: E402
from polaris.navigation import (  # noqa: E402
    LocalGoal,
    LocalGoalNavigator,
    LocalNavigationConfig,
    NavigatorState,
    ned_error_to_body,
)
from test_reactive_avoidance import CLEAR, navigation as perception_state  # noqa: E402


def config(**changes) -> LocalNavigationConfig:
    base = LocalNavigationConfig(goals={"test": {"north_offset_m": 5, "east_offset_m": 0}})
    return replace(base, **changes)


def state(north=0.0, east=0.0, down=-2.5, yaw=0.0, update=1.0) -> NavigationState:
    return NavigationState(
        position_ned_m=Vector3(north, east, down),
        velocity_ned_m_s=Vector3(),
        orientation=Orientation(yaw_deg=yaw),
        last_position_update_monotonic=update,
        last_attitude_update_monotonic=update,
        local_position_ok=True,
    )


def goal(**changes) -> LocalGoal:
    base = LocalGoal(
        goal_id="test",
        target_north_m=5.0,
        target_east_m=0.0,
        target_down_m=-2.5,
        timestamp_monotonic=1.0,
    )
    return replace(base, **changes)


class LocalGoalNavigationTests(unittest.TestCase):
    def test_automatic_landing_on_failure_cannot_be_disabled(self) -> None:
        unsafe = config(automatic_landing_on_failure=False)
        with self.assertRaises(ValueError):
            unsafe.validate()

    def test_collision_hard_override_cannot_be_disabled(self) -> None:
        unsafe = config(collision_hard_override_enabled=False)
        with self.assertRaises(ValueError):
            unsafe.validate()

    def test_goal_ahead_produces_forward_command(self) -> None:
        navigator = LocalGoalNavigator(config())
        vehicle = state()
        navigator.start(goal(), vehicle, 1.0)
        decision = navigator.update(vehicle, now_monotonic=1.1)
        self.assertEqual(decision.state, NavigatorState.ADVANCE)
        self.assertGreater(decision.requested_motion.forward_m_s, 0.3)

    def test_goal_to_right_has_positive_body_right_error(self) -> None:
        forward, right = ned_error_to_body(0.0, 5.0, 0.0)
        self.assertAlmostEqual(forward, 0.0)
        self.assertGreater(right, 0.0)
        navigator = LocalGoalNavigator(config(alignment_motion_limit_deg=100.0))
        vehicle = state(yaw=0.0)
        navigator.start(goal(target_north_m=0.0, target_east_m=5.0), vehicle, 1.0)
        decision = navigator.update(vehicle, now_monotonic=1.1)
        self.assertGreater(decision.requested_motion.right_m_s, 0.0)

    def test_approach_slowdown(self) -> None:
        navigator = LocalGoalNavigator(config())
        vehicle = state()
        navigator.start(goal(target_north_m=1.0), vehicle, 1.0)
        decision = navigator.update(vehicle, now_monotonic=1.1)
        self.assertLess(decision.requested_motion.forward_m_s, config().cruise_speed_m_s)
        self.assertGreaterEqual(decision.requested_motion.forward_m_s, config().approach_speed_m_s)

    def test_completion_requires_hold_persistence(self) -> None:
        navigator = LocalGoalNavigator(config())
        vehicle = state(north=5.0)
        navigator.start(goal(), state(), 1.0)
        first = navigator.update(vehicle, now_monotonic=2.0)
        final = navigator.update(vehicle, now_monotonic=3.6)
        self.assertEqual(first.state, NavigatorState.VERIFY_PROGRESS)
        self.assertTrue(final.goal_reached)

    def test_terminal_yaw_is_required_before_completion(self) -> None:
        navigator = LocalGoalNavigator(config())
        navigator.start(goal(target_yaw_deg=90.0), state(), 1.0)
        position_only = state(north=5.0, yaw=0.0)
        aligning = navigator.update(position_only, now_monotonic=2.0)
        self.assertEqual(aligning.state, NavigatorState.ALIGN)
        self.assertNotEqual(aligning.requested_motion.yaw_rate_deg_s, 0.0)
        aligned = state(north=5.0, yaw=90.0)
        navigator.update(aligned, now_monotonic=3.0)
        reached = navigator.update(aligned, now_monotonic=4.6)
        self.assertTrue(reached.goal_reached)

    def test_goal_timeout(self) -> None:
        navigator = LocalGoalNavigator(config())
        vehicle = state()
        navigator.start(goal(timeout_s=2.0), vehicle, 1.0)
        decision = navigator.update(vehicle, now_monotonic=3.1)
        self.assertTrue(decision.failed)

    def test_reached_goal_remains_terminal_during_verification(self) -> None:
        navigator = LocalGoalNavigator(config())
        vehicle = state(north=5.0)
        navigator.start(goal(timeout_s=2.0), state(), 1.0)
        navigator.update(vehicle, now_monotonic=1.1)
        reached = navigator.update(vehicle, now_monotonic=2.7)
        verified = navigator.update(vehicle, now_monotonic=3.1)
        self.assertTrue(reached.goal_reached)
        self.assertTrue(verified.goal_reached)
        self.assertFalse(verified.failed)

    def test_path_length_limit(self) -> None:
        navigator = LocalGoalNavigator(config())
        navigator.start(goal(maximum_path_length_m=1.0), state(update=1.0), 1.0)
        navigator.update(state(north=0.6, update=2.0), now_monotonic=2.0)
        decision = navigator.update(state(north=1.2, update=3.0), now_monotonic=3.0)
        self.assertTrue(decision.failed)
        self.assertGreater(decision.metrics.path_length_proxy_m, 1.0)

    def test_no_progress_requests_recovery(self) -> None:
        navigator = LocalGoalNavigator(config(no_progress_timeout_s=2.0))
        vehicle = state()
        navigator.start(goal(), vehicle, 1.0)
        navigator.update(vehicle, now_monotonic=1.1)
        decision = navigator.update(vehicle, now_monotonic=3.1)
        self.assertTrue(decision.recovery_requested)
        self.assertEqual(decision.state, NavigatorState.RECOVER_VIEW)

    def test_yaw_alignment_does_not_count_as_no_progress(self) -> None:
        navigator = LocalGoalNavigator(config(no_progress_timeout_s=2.0))
        vehicle = state(yaw=180.0)
        navigator.start(goal(), vehicle, 1.0)
        decision = navigator.update(vehicle, now_monotonic=4.0)
        self.assertEqual(decision.state, NavigatorState.ALIGN)
        self.assertFalse(decision.recovery_requested)

    def test_post_recovery_progress_deadline_is_enforced(self) -> None:
        navigator = LocalGoalNavigator(
            config(progress_timeout_after_recovery_s=2.0)
        )
        vehicle = state()
        navigator.start(goal(), vehicle, 1.0)
        navigator.update(vehicle, now_monotonic=1.1)
        navigator.complete_recovery(True, 2.0, 0.0)
        decision = navigator.update(vehicle, now_monotonic=4.1)
        self.assertTrue(decision.recovery_requested)
        self.assertIn("after recovery", decision.reasons[0])

    def test_metrics_distinguish_displacement_path_and_hold_error(self) -> None:
        navigator = LocalGoalNavigator(config())
        navigator.start(goal(), state(update=1.0), 1.0)
        navigator.update(state(north=1.0, update=2.0), now_monotonic=2.0)
        navigator.update(state(north=1.0, east=1.0, update=3.0), now_monotonic=3.0)
        decision = navigator.update(state(north=2.0, east=1.0, update=4.0), now_monotonic=4.0)
        metrics = decision.metrics
        self.assertAlmostEqual(metrics.path_length_proxy_m, 3.0)
        self.assertAlmostEqual(metrics.displacement_from_start_m, 5 ** 0.5)
        self.assertNotEqual(metrics.hold_position_error_m, metrics.path_length_proxy_m)
        self.assertNotIn("drift", metrics.__dataclass_fields__)

    def test_reactive_avoidance_remains_authoritative(self) -> None:
        nav_config = config()
        navigator = LocalGoalNavigator(nav_config)
        vehicle = state()
        navigator.start(goal(), vehicle, 1.0)
        requested = navigator.update(vehicle, now_monotonic=1.1).requested_motion
        obstacle = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        controller = ReactiveAvoidanceController(AvoidanceConfig())
        perception = perception_state(obstacle, 1)
        controller.update(requested, perception, "NORMAL", 1.1)
        perception.depth = replace(perception.depth, source_frame_sequence=2)
        decision = controller.update(requested, perception, "NORMAL", 1.5)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.0)


if __name__ == "__main__":
    unittest.main()
