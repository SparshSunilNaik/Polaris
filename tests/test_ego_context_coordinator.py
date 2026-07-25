from __future__ import annotations

import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    DesiredMotion,
    EmergencyConfirmation,
    EmergencyConfirmationState,
    ReactiveAvoidanceController,
    SafeMotion,
    SupervisorMode,
)
from polaris.ego_context import (  # noqa: E402
    EgoContextCoordinator,
    EgoContextMemory,
    RuntimeContextProjection,
)
from polaris.ego_state.confidence import ConfidenceEngine  # noqa: E402
from polaris.mission import (  # noqa: E402
    FailureAction,
    FailurePolicy,
    MissionPhase,
    MissionPlan,
    MissionResult,
    MissionState,
    MissionWaypoint,
    ObservationCompletionConfig,
    ReturnOrLandPolicy,
    WaypointType,
)
from polaris.navigation import (  # noqa: E402
    LocalGoal,
    LocalGoalNavigator,
    LocalNavigationConfig,
    RecoveryAction,
    RecoveryDecision,
    RecoveryState,
)
from polaris.perception.depth.models import DepthCondition  # noqa: E402
from polaris.perception.models import VisualCondition  # noqa: E402
from polaris.simulation.collision_monitor import CollisionState  # noqa: E402
from polaris.vlm import VLMResult, VLMSceneState, VLMStatus  # noqa: E402
from test_reactive_avoidance import CLEAR, navigation  # noqa: E402


class EgoContextCoordinatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.memory = EgoContextMemory(event_capacity=64)
        self.coordinator = EgoContextCoordinator(self.memory)
        self.plan = MissionPlan(
            mission_id="demo",
            name="Demo Mission",
            description="tests",
            waypoints=(self.waypoint(),),
            failure_policy=FailurePolicy.ABORT_MISSION,
            maximum_duration_s=60.0,
            maximum_path_length_m=12.0,
            maximum_replans=1,
            return_or_land_policy=ReturnOrLandPolicy.LAND_AT_CURRENT_POSITION,
            source="test",
        )
        self.state = MissionState(
            mission_id="demo",
            mission_name="Demo Mission",
            state=MissionPhase.PREFLIGHT,
            current_waypoint_index=0,
            current_waypoint_id="one",
            total_waypoints=1,
        )

    @staticmethod
    def waypoint(
        waypoint_type: WaypointType = WaypointType.POSITION,
    ) -> MissionWaypoint:
        return MissionWaypoint(
            waypoint_id="one",
            goal=LocalGoal("one", 0.0, 2.0, 0.0),
            failure_action=FailureAction.ABORT_MISSION,
            waypoint_type=waypoint_type,
        )

    def projection(self, now: float, **changes) -> RuntimeContextProjection:
        ego = navigation(CLEAR, int(now * 10) + 1)
        ego.armed = True
        ego.in_air = True
        ego.flight_mode = "OFFBOARD"
        navigator = LocalGoalNavigator(
            LocalNavigationConfig(goals={"one": {"north_offset_m": 0, "east_offset_m": 2}})
        )
        navigator.start(LocalGoal("one", 0.0, 2.0, 0.0), ego, now)
        goal = navigator.update(ego, now_monotonic=now)
        report = ConfidenceEngine().evaluate(ego)
        safe = SafeMotion.hover(
            now,
            SupervisorMode.HOVER,
            ("Holding",),
            False,
            10.0,
        )
        values = {
            "timestamp_monotonic": now,
            "mission_state": self.state,
            "current_waypoint": self.plan.waypoints[0],
            "ego_state": ego,
            "confidence": report,
            "goal": goal,
            "recovery": None,
            "recovery_state": RecoveryState.IDLE,
            "avoidance": None,
            "observation": None,
            "safe_motion": safe,
            "collision": CollisionState(),
            "mission_transitions": (),
            "observation_events": (),
        }
        values.update(changes)
        return RuntimeContextProjection(**values)

    @staticmethod
    def kinds(snapshot) -> list[str]:
        return [event.metadata.get("kind", "") for event in snapshot.recent_events]

    def start(self):
        return self.coordinator.start_mission(self.plan, self.state, 1.0)

    @staticmethod
    def vlm_state(
        now: float,
        *,
        terrain: str = "snowfield",
        landmarks: tuple[str, ...] = ("station",),
        target_visible: bool = False,
        summary: str = "Snowfield with station ahead",
    ) -> VLMSceneState:
        result = VLMResult(
            status=VLMStatus.SUCCESS,
            terrain_type=terrain,
            visible_landmarks=landmarks,
            target_visible=target_visible,
            scene_confidence=0.9,
            texture_quality=0.7,
            scene_summary=summary,
            model="qwen2.5vl:7b",
        )
        return VLMSceneState(
            status=VLMStatus.SUCCESS,
            result=result,
            model=result.model,
            source_frame_sequence=int(now * 10),
            source_frame_timestamp_monotonic=now - 0.1,
            inference_timestamp_monotonic=now,
            inference_latency_ms=100.0,
        )

    def test_coordinator_initializes_and_resets_memory(self) -> None:
        self.memory.update_mission(timestamp_monotonic=0.5, mission_id="old")
        snapshot = self.start()
        self.assertEqual(snapshot.mission.mission_id, "demo")
        self.assertEqual(self.kinds(snapshot), ["MISSION_STARTED"])

    def test_mission_start_projection(self) -> None:
        snapshot = self.start()
        self.assertEqual(snapshot.mission.current_waypoint_id, "one")
        self.assertEqual(snapshot.mission.current_waypoint_index, 0)
        self.assertEqual(snapshot.mission.total_waypoints, 1)

    def test_waypoint_transition_projection(self) -> None:
        self.start()
        state = replace(self.state, state=MissionPhase.WAYPOINT_EXECUTE, current_attempt=1)
        transition = {
            "timestamp_monotonic": 2.0,
            "state": "WAYPOINT_EXECUTE",
            "waypoint_id": "one",
            "reason": "Waypoint execution started",
        }
        snapshot = self.coordinator.project(
            self.projection(2.0, mission_state=state, mission_transitions=(transition,))
        )
        self.assertIn("WAYPOINT_STARTED", self.kinds(snapshot))

    def test_positional_completion_event(self) -> None:
        self.start()
        state = replace(
            self.state,
            completed_waypoints=["one"],
            waypoint_completion_modes={"one": "POSITION"},
        )
        snapshot = self.coordinator.project(self.projection(2.0, mission_state=state))
        event = snapshot.recent_events[-1]
        self.assertEqual(event.metadata["kind"], "WAYPOINT_COMPLETED")
        self.assertEqual(event.metadata["completion_mode"], "POSITION")

    def test_observation_completion_event(self) -> None:
        self.plan = replace(self.plan, waypoints=(self.waypoint(WaypointType.OBSERVATION),))
        self.start()
        state = replace(
            self.state,
            completed_waypoints=["one"],
            waypoint_completion_modes={"one": "OBSERVATION"},
        )
        snapshot = self.coordinator.project(
            self.projection(
                2.0,
                mission_state=state,
                current_waypoint=self.plan.waypoints[0],
            )
        )
        event = snapshot.recent_events[-1]
        self.assertEqual(event.metadata["completion_mode"], "OBSERVATION")
        self.assertEqual(snapshot.summary.observations_completed, 1)

    def test_optional_skip_event(self) -> None:
        self.start()
        state = replace(self.state, skipped_waypoints=["one"])
        snapshot = self.coordinator.project(self.projection(2.0, mission_state=state))
        self.assertIn("WAYPOINT_SKIPPED", self.kinds(snapshot))

    def test_mission_abort_event(self) -> None:
        self.start()
        state = replace(
            self.state,
            state=MissionPhase.ABORT,
            result=MissionResult.ABORTED,
            abort_reason="Hard safety override",
        )
        snapshot = self.coordinator.project(self.projection(2.0, mission_state=state))
        self.assertIn("MISSION_ABORTED", self.kinds(snapshot))

    def test_navigation_state_projection(self) -> None:
        self.start()
        projection = self.projection(2.0)
        snapshot = self.coordinator.project(projection)
        self.assertEqual(snapshot.navigation.navigator_state, projection.goal.state.value)
        self.assertEqual(snapshot.navigation.strategy, "LOCAL_GOAL")
        self.assertAlmostEqual(
            snapshot.navigation.goal_distance_m,
            projection.goal.metrics.goal_distance_m,
        )

    def test_avoidance_transition_edge_detection(self) -> None:
        self.start()
        controller = ReactiveAvoidanceController(AvoidanceConfig())
        ego = navigation(((0.9, 0.1, 0.1), (0.9, 0.2, 0.1), (1, 1, 1)), 1)
        command = DesiredMotion(forward_m_s=0.3, timestamp_monotonic=2.0)
        controller.update(command, ego, "NORMAL", 2.0)
        avoidance = controller.update(command, replace(ego, depth=replace(ego.depth, source_frame_sequence=2)), "NORMAL", 2.4)
        first = self.coordinator.project(self.projection(2.4, avoidance=avoidance))
        count = len(first.recent_events)
        second = self.coordinator.project(self.projection(2.5, avoidance=avoidance))
        self.assertEqual(len(second.recent_events), count)
        self.assertEqual(second.summary.avoidance_count, 1)

    def test_recovery_and_active_perception_edges(self) -> None:
        self.start()
        recovery = RecoveryDecision(
            RecoveryState.SCANNING,
            RecoveryAction.YAW_SCAN_LEFT,
            DesiredMotion.hover(2.0, "active-perception-scan"),
            "Low texture",
            -15.0,
            None,
            (),
            1,
            10.0,
            ("Scanning",),
            2.0,
        )
        started = self.coordinator.project(
            self.projection(2.0, recovery=recovery, recovery_state=RecoveryState.SCANNING)
        )
        self.assertIn("RECOVERY_STARTED", self.kinds(started))
        self.assertIn("ACTIVE_PERCEPTION_STARTED", self.kinds(started))
        complete = replace(recovery, state=RecoveryState.COMPLETE, timestamp_monotonic=2.5)
        ended = self.coordinator.project(
            self.projection(2.5, recovery=complete, recovery_state=RecoveryState.COMPLETE)
        )
        self.assertIn("RECOVERY_COMPLETED", self.kinds(ended))
        self.assertIn("ACTIVE_PERCEPTION_COMPLETED", self.kinds(ended))

    def test_perception_health_and_stale_depth_transitions(self) -> None:
        self.start()
        baseline = self.projection(2.0)
        self.coordinator.project(baseline)
        ego = baseline.ego_state
        ego.visual = replace(
            ego.visual,
            visual_condition=VisualCondition.CRITICAL,
            visual_health_score=0.1,
        )
        ego.depth = replace(ego.depth, stale=True, condition=DepthCondition.CRITICAL)
        changed = self.coordinator.project(self.projection(2.5, ego_state=ego))
        self.assertIn("PERCEPTION_UNHEALTHY", self.kinds(changed))
        self.assertIn("DEPTH_STALE", self.kinds(changed))

    def test_emergency_suspected_cleared_confirmed_transitions(self) -> None:
        self.start()
        self.coordinator.project(self.projection(2.0))
        snapshots = []
        for index, state in enumerate(
            (
                EmergencyConfirmationState.SUSPECTED,
                EmergencyConfirmationState.CLEAR,
                EmergencyConfirmationState.CONFIRMED,
            ),
            start=1,
        ):
            confirmation = EmergencyConfirmation(state=state, reason=state.value)
            safe = SafeMotion.hover(
                2.0 + index,
                SupervisorMode.HOVER,
                (state.value,),
                state is EmergencyConfirmationState.CONFIRMED,
                10.0,
                emergency_confirmation=confirmation,
            )
            snapshots.append(
                self.coordinator.project(
                    self.projection(2.0 + index, safe_motion=safe)
                )
            )
        kinds = self.kinds(snapshots[-1])
        self.assertIn("EMERGENCY_SUSPECTED", kinds)
        self.assertIn("EMERGENCY_CLEARED", kinds)
        self.assertIn("EMERGENCY_CONFIRMED", kinds)

    def test_collision_and_hard_abort_projection(self) -> None:
        self.start()
        collision = CollisionState(
            available=True,
            collision_detected=True,
            collision_count=1,
            latest_contacted_entity="fixture",
        )
        safe = SafeMotion.hover(
            2.0,
            SupervisorMode.ABORT,
            ("Airborne collision detected",),
            True,
            10.0,
        )
        snapshot = self.coordinator.project(
            self.projection(2.0, collision=collision, safe_motion=safe)
        )
        self.assertTrue(snapshot.safety.collision_detected)
        self.assertIn("COLLISION_DETECTED", self.kinds(snapshot))
        self.assertIn("HARD_ABORT", self.kinds(snapshot))

    def test_landing_and_disarm_transitions(self) -> None:
        self.start()
        landing = replace(
            self.state,
            state=MissionPhase.RETURN_OR_LAND,
            landing_result="NOT_ATTEMPTED",
        )
        self.coordinator.project(self.projection(2.0, mission_state=landing))
        ego = navigation(CLEAR, 30)
        ego.armed = False
        ego.in_air = False
        landed = replace(
            landing,
            state=MissionPhase.FINISHED,
            landing_result="CONFIRMED",
            result=MissionResult.COMPLETED,
        )
        snapshot = self.coordinator.finish(
            self.projection(3.0, mission_state=landed, ego_state=ego)
        )
        self.assertIn("LANDING_STARTED", self.kinds(snapshot))
        self.assertIn("LANDING_CONFIRMED", self.kinds(snapshot))
        self.assertIn("DISARM_CONFIRMED", self.kinds(snapshot))

    def test_aggregate_summary_updates(self) -> None:
        self.start()
        state = replace(
            self.state,
            completed_waypoints=["one"],
            skipped_waypoints=["optional"],
            replan_count=1,
            mission_elapsed_s=12.0,
        )
        snapshot = self.coordinator.project(self.projection(2.0, mission_state=state))
        self.assertEqual(snapshot.summary.completed_waypoint_count, 1)
        self.assertEqual(snapshot.summary.skipped_waypoint_count, 1)
        self.assertEqual(snapshot.summary.replan_count, 1)
        self.assertEqual(snapshot.summary.elapsed_mission_s, 12.0)

    def test_unchanged_state_does_not_duplicate_events(self) -> None:
        self.start()
        projection = self.projection(2.0)
        first = self.coordinator.project(projection)
        second = self.coordinator.project(replace(projection, timestamp_monotonic=2.1))
        self.assertEqual(first.recent_events, second.recent_events)

    def test_event_order_is_chronological(self) -> None:
        self.start()
        self.coordinator.project(self.projection(2.0))
        self.coordinator.project(self.projection(3.0))
        timestamps = [event.timestamp_monotonic for event in self.coordinator.snapshot().recent_events]
        self.assertEqual(timestamps, sorted(timestamps))

    def test_runtime_reset_and_consecutive_missions_do_not_leak(self) -> None:
        self.start()
        self.coordinator.project(self.projection(2.0))
        self.coordinator.reset()
        self.plan = replace(self.plan, mission_id="second", name="Second Mission")
        self.state = replace(self.state, mission_id="second", mission_name="Second Mission")
        snapshot = self.start()
        self.assertEqual(snapshot.mission.mission_id, "second")
        self.assertEqual(len(snapshot.recent_events), 1)
        self.assertNotIn("demo", snapshot.recent_events[0].message)

    def test_snapshot_is_immutable_for_operator_view(self) -> None:
        snapshot = self.start()
        with self.assertRaises(FrozenInstanceError):
            snapshot.revision = 99

    def test_coordinator_cannot_produce_commands(self) -> None:
        self.start()
        result = self.coordinator.project(self.projection(2.0))
        self.assertFalse(hasattr(self.coordinator, "send"))
        self.assertFalse(hasattr(result, "forward_m_s"))

    def test_vlm_disabled_mode_is_explicit(self) -> None:
        self.start()
        snapshot = self.coordinator.project(self.projection(2.0, vlm=None))
        self.assertEqual(snapshot.perception.vlm_status, "DISABLED")
        self.assertEqual(snapshot.perception.terrain_type, "UNKNOWN")

    def test_successful_fresh_vlm_result_is_projected(self) -> None:
        self.start()
        snapshot = self.coordinator.project(
            self.projection(2.0, vlm=self.vlm_state(2.0, target_visible=True))
        )
        self.assertEqual(snapshot.perception.vlm_status, "SUCCESS")
        self.assertEqual(snapshot.perception.terrain_type, "snowfield")
        self.assertEqual(snapshot.perception.visible_landmarks, ("station",))
        self.assertTrue(snapshot.perception.target_visible)
        self.assertEqual(snapshot.perception.fused_scene.evidence_count, 1)
        self.assertEqual(
            snapshot.perception.fused_scene.status.value,
            "INSUFFICIENT_EVIDENCE",
        )

    def test_vlm_result_deduplicates_equivalent_scene_meaning(self) -> None:
        self.start()
        snapshots = []
        for now in (2.0, 3.0, 4.0, 5.0):
            snapshots.append(
                self.coordinator.project(
                    self.projection(
                        now,
                        vlm=self.vlm_state(
                            now,
                            target_visible=True,
                            summary=f"Wording {now}",
                        ),
                    )
                )
            )
        events = [
            event
            for event in snapshots[-1].recent_events
            if event.category == "SCENE_FUSION"
        ]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].metadata["kind"], "FUSED_SCENE_STABLE")

    def test_significant_vlm_semantic_change_emits_event(self) -> None:
        self.start()
        for now in (2.0, 3.0, 4.0):
            self.coordinator.project(
                self.projection(
                    now,
                    vlm=self.vlm_state(now, target_visible=True),
                )
            )
        changed = self.coordinator.project(
            self.projection(
                5.0,
                vlm=self.vlm_state(
                    5.0,
                    terrain="rocky ridge",
                    landmarks=("station", "ridge"),
                    target_visible=False,
                    summary="Conflicting target evidence beside ridge",
                ),
            )
        )
        self.assertIn("FUSED_SCENE_STABLE", self.kinds(changed))
        self.assertIn("FUSED_SCENE_CONFLICTED", self.kinds(changed))

    def test_vlm_has_zero_control_or_observation_authority(self) -> None:
        self.start()
        projection = self.projection(
            2.0,
            vlm=self.vlm_state(2.0, target_visible=True),
        )
        requested = projection.goal.requested_motion
        snapshot = self.coordinator.project(projection)
        self.assertEqual(snapshot.safety.current_ownership, "LOCAL_GOAL")
        self.assertEqual(projection.goal.requested_motion, requested)
        self.assertEqual(snapshot.mission.completed_waypoints, ())
        self.assertFalse(hasattr(projection.ego_state, "vlm"))

    def test_semantic_validation_records_once_without_changing_outcome(self) -> None:
        observation_waypoint = replace(
            self.waypoint(WaypointType.OBSERVATION),
            observation=ObservationCompletionConfig(),
            metadata={"expected_target_label": "rock"},
        )
        self.plan = replace(self.plan, waypoints=(observation_waypoint,))
        self.start()
        state = replace(
            self.state,
            completed_waypoints=["one"],
            waypoint_completion_modes={"one": "OBSERVATION"},
        )
        projection = self.projection(
            2.0,
            mission_state=state,
            current_waypoint=observation_waypoint,
            vlm=self.vlm_state(
                2.0,
                landmarks=("rock",),
                target_visible=True,
            ),
            semantic_validation_enabled=True,
        )
        requested = projection.goal.requested_motion
        first = self.coordinator.project(projection)
        second = self.coordinator.project(
            replace(
                projection,
                timestamp_monotonic=3.0,
                vlm=self.vlm_state(
                    3.0,
                    landmarks=("rock",),
                    target_visible=True,
                ),
            )
        )
        events = [
            event
            for event in second.recent_events
            if event.metadata.get("kind") == "OBSERVATION_SEMANTIC_VALIDATION"
        ]
        record = second.perception.latest_semantic_validation
        self.assertEqual(len(events), 1)
        self.assertEqual(record.classification.value, "AGREEMENT")
        self.assertEqual(record.evidence_source.value, "SINGLE_FRAME")
        self.assertEqual(second.mission.completed_waypoints, ("one",))
        self.assertEqual(projection.goal.requested_motion, requested)
        self.assertEqual(first.safety.current_ownership, second.safety.current_ownership)


if __name__ == "__main__":
    unittest.main()
