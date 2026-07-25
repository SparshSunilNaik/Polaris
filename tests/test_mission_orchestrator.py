from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.mission import (  # noqa: E402
    FailureAction,
    FailurePolicy,
    MissionAction,
    MissionOrchestrator,
    MissionOrchestratorConfig,
    MissionPhase,
    MissionPlan,
    MissionResult,
    MissionWaypoint,
    ReturnOrLandPolicy,
    WaypointCompletionMode,
)
from polaris.navigation import LocalGoal  # noqa: E402


def waypoint(identifier: str, east: float, **changes) -> MissionWaypoint:
    base = MissionWaypoint(
        waypoint_id=identifier,
        goal=LocalGoal(identifier, 0.0, east, 0.0),
        required=True,
        skippable=False,
        maximum_attempts=1,
        verification_hold_s=1.0,
        failure_action=FailureAction.ABORT_MISSION,
    )
    return replace(base, **changes)


def plan(*waypoints: MissionWaypoint, policy=FailurePolicy.ABORT_MISSION) -> MissionPlan:
    return MissionPlan(
        "test-mission",
        "Test Mission",
        "tests",
        tuple(waypoints),
        policy,
        60.0,
        12.0,
        1,
        ReturnOrLandPolicy.LAND_AT_CURRENT_POSITION,
        "test",
    )


class MissionOrchestratorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MissionOrchestratorConfig()

    def _started(self, mission: MissionPlan) -> MissionOrchestrator:
        orchestrator = MissionOrchestrator(self.config)
        orchestrator.start(mission, 1.0)
        orchestrator.mark_takeoff(2.0)
        orchestrator.mark_mission_start(3.0)
        orchestrator.begin_waypoint(3.1)
        return orchestrator

    def test_successful_waypoint_progression_and_completion(self) -> None:
        orchestrator = self._started(plan(waypoint("one", 2), waypoint("two", 4)))
        orchestrator.begin_verification(4.0)
        decision = orchestrator.verify_waypoint(
            5.1, inside_tolerance=True, visual_health=1, depth_health=1
        )
        self.assertEqual(decision.action, MissionAction.START_WAYPOINT)
        orchestrator.prepare_waypoint(6.1)
        orchestrator.begin_waypoint(6.2)
        orchestrator.begin_verification(7.0)
        decision = orchestrator.verify_waypoint(
            8.1, inside_tolerance=True, visual_health=1, depth_health=1
        )
        self.assertEqual(decision.action, MissionAction.COMPLETE_MISSION)
        self.assertEqual(orchestrator.state.result, MissionResult.COMPLETED)
        self.assertEqual(
            orchestrator.state.waypoint_completion_modes,
            {"one": "POSITION", "two": "POSITION"},
        )

    def test_observation_completion_mode_is_recorded(self) -> None:
        orchestrator = self._started(plan(waypoint("rock-observation", 7)))
        decision = orchestrator.complete_waypoint(
            4.0,
            completion_mode=WaypointCompletionMode.OBSERVATION,
            reason="Stable observation complete",
        )
        self.assertEqual(decision.action, MissionAction.COMPLETE_MISSION)
        self.assertEqual(
            orchestrator.state.waypoint_completion_modes["rock-observation"],
            "OBSERVATION",
        )

    def test_mission_start_clears_completion_modes(self) -> None:
        mission = plan(waypoint("one", 2))
        orchestrator = self._started(mission)
        orchestrator.complete_waypoint(4.0)
        orchestrator.start(mission, 5.0)
        self.assertEqual(orchestrator.state.waypoint_completion_modes, {})

    def test_policy_is_preserved_after_observation_timeout(self) -> None:
        observation = waypoint(
            "rock-observation",
            7,
            failure_action=FailureAction.REPLAN_AROUND_WAYPOINT,
        )
        orchestrator = self._started(plan(observation))
        decision = orchestrator.fail_waypoint("Observation and position timed out", 4.0)
        self.assertEqual(decision.action, MissionAction.REQUEST_REPLAN)

    def test_verification_hold_is_required(self) -> None:
        orchestrator = self._started(plan(waypoint("one", 2)))
        orchestrator.begin_verification(4.0)
        decision = orchestrator.verify_waypoint(
            4.5, inside_tolerance=True, visual_health=1, depth_health=1
        )
        self.assertEqual(decision.action, MissionAction.NONE)
        self.assertEqual(orchestrator.state.state, MissionPhase.WAYPOINT_VERIFY)

    def test_optional_waypoint_can_be_skipped(self) -> None:
        optional = waypoint(
            "optional",
            2,
            required=False,
            skippable=True,
            failure_action=FailureAction.SKIP_OPTIONAL,
        )
        orchestrator = self._started(plan(optional, waypoint("required", 4)))
        decision = orchestrator.fail_waypoint("controlled", 4.0)
        self.assertEqual(decision.action, MissionAction.SKIP_WAYPOINT)
        self.assertEqual(orchestrator.state.skipped_waypoints, ["optional"])

    def test_completion_with_skips(self) -> None:
        optional = waypoint(
            "optional",
            2,
            required=False,
            skippable=True,
            failure_action=FailureAction.SKIP_OPTIONAL,
        )
        orchestrator = self._started(plan(optional))
        decision = orchestrator.fail_waypoint("controlled", 4.0)
        self.assertEqual(decision.action, MissionAction.COMPLETE_MISSION)
        self.assertEqual(orchestrator.state.result, MissionResult.COMPLETED_WITH_SKIPS)

    def test_required_failure_aborts(self) -> None:
        orchestrator = self._started(plan(waypoint("required", 2)))
        decision = orchestrator.fail_waypoint("blocked", 4.0)
        self.assertEqual(decision.action, MissionAction.ABORT_MISSION)
        self.assertEqual(orchestrator.state.result, MissionResult.ABORTED)

    def test_retry_limit(self) -> None:
        retry = waypoint(
            "retry",
            2,
            maximum_attempts=2,
            failure_action=FailureAction.RETRY_WAYPOINT,
        )
        orchestrator = self._started(plan(retry, policy=FailurePolicy.RETRY_WAYPOINT))
        first = orchestrator.fail_waypoint("blocked", 4.0)
        self.assertEqual(first.action, MissionAction.RETRY_WAYPOINT)
        orchestrator.begin_waypoint(4.1)
        second = orchestrator.fail_waypoint("blocked again", 5.0)
        self.assertEqual(second.action, MissionAction.ABORT_MISSION)

    def test_mission_timeout(self) -> None:
        orchestrator = self._started(plan(waypoint("one", 2)))
        decision = orchestrator.update_metrics(
            63.1,
            path_length_proxy_m=1.0,
            displacement_from_start_m=1.0,
            current_goal_distance_m=1.0,
        )
        self.assertEqual(decision.action, MissionAction.ABORT_MISSION)

    def test_path_length_limit(self) -> None:
        orchestrator = self._started(plan(waypoint("one", 2)))
        decision = orchestrator.update_metrics(
            4.0,
            path_length_proxy_m=12.1,
            displacement_from_start_m=2.0,
            current_goal_distance_m=1.0,
        )
        self.assertEqual(decision.action, MissionAction.ABORT_MISSION)

    def test_collision_and_confidence_emergency_abort(self) -> None:
        for reason in ("Airborne collision detected", "Confidence Engine is EMERGENCY"):
            orchestrator = self._started(plan(waypoint("one", 2)))
            decision = orchestrator.fail_waypoint(reason, 4.0, hard_failure=True)
            self.assertEqual(decision.action, MissionAction.ABORT_MISSION)

    def test_ctrl_c_cancellation_path(self) -> None:
        orchestrator = self._started(plan(waypoint("one", 2)))
        decision = orchestrator.abort("Operator interrupted with Ctrl+C", 4.0)
        self.assertEqual(decision.action, MissionAction.ABORT_MISSION)
        self.assertIn("Ctrl+C", orchestrator.state.abort_reason)

    def test_stop_after_waypoint_finishes_early(self) -> None:
        orchestrator = MissionOrchestrator(self.config)
        orchestrator.start(
            plan(waypoint("one", 2), waypoint("two", 4)),
            1.0,
            stop_after_waypoint="one",
        )
        orchestrator.mark_mission_start(2.0)
        orchestrator.begin_waypoint(2.1)
        orchestrator.begin_verification(3.0)
        decision = orchestrator.verify_waypoint(
            4.1, inside_tolerance=True, visual_health=1, depth_health=1
        )
        self.assertEqual(decision.action, MissionAction.COMPLETE_MISSION)


if __name__ == "__main__":
    unittest.main()
