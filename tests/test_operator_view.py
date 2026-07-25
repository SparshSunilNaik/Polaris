from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    DesiredMotion,
    EmergencyConfirmation,
    EmergencyConfirmationState,
    ReactiveAvoidanceController,
    RecommendedAction,
    SafeMotion,
    SupervisorMode,
)
from polaris.ego_state.confidence import ConfidenceEngine  # noqa: E402
from polaris.ego_context import (  # noqa: E402
    DeterministicObservationOutcome,
    EgoContextMemory,
    SemanticEvidenceFreshness,
    SemanticValidationClassification,
    SemanticValidationRecord,
)
from polaris.vlm import FusedSceneContext, FusedSceneStatus  # noqa: E402
from polaris.ego_state.models import NavigationState, Orientation, Vector3  # noqa: E402
from polaris.navigation import (  # noqa: E402
    LocalGoal,
    LocalGoalNavigator,
    LocalNavigationConfig,
    RecoveryState,
)
from polaris.mission import (  # noqa: E402
    MissionState,
    ObservationCompletionConfig,
    ObservationTracker,
)
from polaris.operator_view import (  # noqa: E402
    OperatorRenderer,
    OperatorViewConfig,
    format_avoidance_lines,
    format_navigation_lines,
    format_mission_lines,
    avoidance_hard_override_banner,
    format_ego_lines,
    format_ego_context_lines,
    format_safety_lines,
    format_visual_lines,
    is_quit_key,
    render_risk_panel,
    risk_color,
    screenshot_path,
)
from polaris.perception.depth.models import (  # noqa: E402
    DepthCondition,
    DepthState,
)
from polaris.simulation import CollisionMonitor  # noqa: E402


def depth_state(grid) -> DepthState:
    flat = [value for row in grid for value in row]
    names = (
        "UP_LEFT",
        "UP",
        "UP_RIGHT",
        "LEFT",
        "CENTRE",
        "RIGHT",
        "DOWN_LEFT",
        "DOWN",
        "DOWN_RIGHT",
    )
    return DepthState(
        available=True,
        stale=False,
        inference_timestamp_monotonic=monotonic(),
        depth_health_score=0.9,
        condition=DepthCondition.HEALTHY,
        risk_grid=grid,
        nearest_region=names[int(np.argmax(flat))],
        safest_region=names[int(np.argmin(flat))],
    )


class OperatorViewTests(unittest.TestCase):
    def setUp(self) -> None:
        self.width = 540
        self.height = 200

    def test_all_clear_risk_grid_renders(self) -> None:
        state = NavigationState(
            depth=depth_state(((0.0, 0.0, 0.0),) * 3)
        )
        panel = render_risk_panel(self.width, self.height, state)
        self.assertEqual(panel.shape, (self.height, self.width, 3))
        clear_pixels = np.all(panel == risk_color(0.0), axis=2)
        self.assertGreater(int(np.sum(clear_pixels)), 5000)

    def test_centre_obstacle_is_rendered_high_risk(self) -> None:
        grid = ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0))
        panel = render_risk_panel(
            self.width,
            self.height,
            NavigationState(depth=depth_state(grid)),
        )
        danger_pixels = np.all(panel == risk_color(1.0), axis=2)
        self.assertGreater(int(np.sum(danger_pixels)), 3000)

    def test_left_obstacle_renders_three_high_risk_cells(self) -> None:
        grid = ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        panel = render_risk_panel(
            self.width,
            self.height,
            NavigationState(depth=depth_state(grid)),
        )
        danger_pixels = np.all(panel == risk_color(1.0), axis=2)
        self.assertGreater(int(np.sum(danger_pixels)), 9000)

    def test_compact_risk_grid_keeps_bottom_row_visible(self) -> None:
        grid = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        panel = render_risk_panel(
            340,
            141,
            NavigationState(depth=depth_state(grid)),
        )
        danger_pixels = np.all(panel[-30:] == risk_color(1.0), axis=2)
        self.assertGreater(int(np.sum(danger_pixels)), 500)

    def test_stale_and_missing_depth_render_placeholders(self) -> None:
        stale = DepthState(
            model_name="test",
            available=True,
            stale=True,
            inference_timestamp_monotonic=1.0,
            reasons=("Depth inference is stale",),
        )
        for depth in (stale, DepthState()):
            with self.subTest(depth=depth):
                panel = render_risk_panel(
                    self.width,
                    self.height,
                    NavigationState(depth=depth),
                )
                self.assertEqual(panel.shape, (self.height, self.width, 3))
                self.assertGreater(int(np.sum(panel[:, :, 2] > 180)), 10)

    def test_status_formatting_contains_navigation_and_health(self) -> None:
        now = monotonic()
        state = NavigationState(
            position_ned_m=Vector3(1.0, -2.0, -3.0),
            velocity_ned_m_s=Vector3(0.1, 0.2, -0.3),
            orientation=Orientation(1.0, 2.0, 90.0),
            local_position_ok=True,
            armed=True,
            in_air=True,
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
        )
        ego = "\n".join(format_ego_lines(state))
        visual = "\n".join(format_visual_lines(state))
        self.assertIn("N   +1.00", ego)
        self.assertIn("Altitude   3.00 m", ego)
        self.assertIn("Armed YES", ego)
        self.assertIn("Condition CRITICAL", visual)
        self.assertIn("unknown / unknown", ego)

    def test_no_depth_safety_format_is_explicitly_disabled(self) -> None:
        report = ConfidenceEngine().evaluate(NavigationState())
        safety = "\n".join(format_safety_lines(report, depth_enabled=False))
        self.assertIn("Depth DISABLED", safety)
        self.assertIn("Depth fresh DISABLED", safety)

    def test_screenshot_path_is_timestamped(self) -> None:
        timestamp = datetime(2026, 7, 23, 12, 34, 56, 789000, tzinfo=timezone.utc)
        path = screenshot_path("artifacts/operator_screenshots", timestamp)
        self.assertEqual(
            path,
            Path("artifacts/operator_screenshots/20260723T123456_789_operator.png"),
        )

    def test_q_and_escape_are_quit_keys(self) -> None:
        self.assertTrue(is_quit_key(ord("q")))
        self.assertTrue(is_quit_key(27))
        self.assertFalse(is_quit_key(ord("p")))

    def test_complete_missing_data_composition_has_requested_size(self) -> None:
        config = OperatorViewConfig(window_width=1200, window_height=700)
        state = NavigationState()
        image = OperatorRenderer(config).compose(
            state,
            ConfidenceEngine().evaluate(state),
            None,
            None,
            display_fps=0.0,
            depth_enabled=False,
            grid_overlay=True,
            paused=False,
            help_visible=True,
        )
        self.assertEqual(image.shape, (700, 1200, 3))

    def test_avoidance_status_composes_at_minimum_size(self) -> None:
        now = monotonic()
        state = NavigationState(depth=depth_state(((0.1, 0.1, 0.1),) * 3))
        state.depth = replace(
            state.depth,
            source_frame_sequence=1,
            temporal_stability=0.9,
        )
        state.visual.visual_health_score = 0.9
        decision = ReactiveAvoidanceController(AvoidanceConfig()).update(
            DesiredMotion(forward_m_s=0.4, timestamp_monotonic=now),
            state,
            "NORMAL",
            now,
        )
        lines = "\n".join(format_avoidance_lines(decision))
        self.assertIn("Enabled YES", lines)
        self.assertIn("Requested F +0.40", lines)
        self.assertIn("Thresholds C 0.45", lines)
        image = OperatorRenderer(
            OperatorViewConfig(window_width=1000, window_height=650)
        ).compose(
            state,
            ConfidenceEngine().evaluate(state),
            None,
            None,
            display_fps=10.0,
            depth_enabled=True,
            grid_overlay=True,
            paused=False,
            help_visible=False,
            avoidance=decision,
        )
        self.assertEqual(image.shape, (650, 1000, 3))

    def test_supervisor_hard_override_has_dominant_banner(self) -> None:
        now = monotonic()
        state = NavigationState(depth=depth_state(((0.1, 0.1, 0.1),) * 3))
        state.visual.visual_health_score = 0.9
        decision = ReactiveAvoidanceController(AvoidanceConfig()).update(
            DesiredMotion(forward_m_s=0.4, timestamp_monotonic=now),
            state,
            "NORMAL",
            now,
        )
        safe = SafeMotion.hover(
            now,
            SupervisorMode.ABORT,
            ("Test hard override",),
            True,
            10.0,
            RecommendedAction.LAND,
        )
        decision = replace(decision, safe_command=safe)
        self.assertEqual(
            avoidance_hard_override_banner(decision),
            "SAFETY SUPERVISOR HARD OVERRIDE - ABORT",
        )

    def test_emergency_confirmation_is_visible(self) -> None:
        now = monotonic()
        state = NavigationState(depth=depth_state(((0.1, 0.1, 0.1),) * 3))
        state.visual.visual_health_score = 0.9
        decision = ReactiveAvoidanceController(AvoidanceConfig()).update(
            DesiredMotion.hover(now, "active-perception-scan"),
            state,
            "NORMAL",
            now,
        )
        confirmation = EmergencyConfirmation(
            state=EmergencyConfirmationState.SUSPECTED,
            observed_frames=1,
            required_frames=2,
            age_s=0.2,
            centre_risk=0.95,
            reason="Emergency centre proximity suspected",
        )
        safe = SafeMotion.hover(
            now,
            SupervisorMode.HOVER,
            (confirmation.reason,),
            False,
            10.0,
            emergency_confirmation=confirmation,
        )
        decision = replace(decision, safe_command=safe)
        lines = "\n".join(format_avoidance_lines(decision))
        self.assertIn("Emergency SUSPECTED 1/2", lines)
        self.assertIn("Age 0.2 s", lines)
        self.assertIn("Centre 0.950", lines)

    def test_navigation_and_collision_status_compose_at_minimum_size(self) -> None:
        now = monotonic()
        state = NavigationState(
            depth=depth_state(((0.1, 0.1, 0.1),) * 3),
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
            local_position_ok=True,
        )
        navigator = LocalGoalNavigator(
            LocalNavigationConfig(
                goals={"test": {"north_offset_m": 5.0, "east_offset_m": 0.0}}
            )
        )
        goal = LocalGoal("test", 5.0, 0.0, 0.0, timestamp_monotonic=now)
        navigator.start(goal, state, now)
        goal_decision = navigator.update(state, now_monotonic=now + 0.1)
        collision_monitor = CollisionMonitor()
        collision_monitor.inject_collision("test_fixture", now)
        collision = collision_monitor.latest_state()
        lines = "\n".join(
            format_navigation_lines(goal_decision, None, collision, None)
        )
        self.assertIn("Goal test", lines)
        self.assertIn("Collision YES count 1", lines)
        image = OperatorRenderer(
            OperatorViewConfig(window_width=1000, window_height=650)
        ).compose(
            state,
            ConfidenceEngine().evaluate(state),
            None,
            None,
            display_fps=10.0,
            depth_enabled=True,
            grid_overlay=True,
            paused=False,
            help_visible=False,
            navigation=goal_decision,
            collision=collision,
        )
        self.assertEqual(image.shape, (650, 1000, 3))

    def test_mission_status_composes_with_local_navigation(self) -> None:
        now = monotonic()
        state = NavigationState(
            depth=depth_state(((0.1, 0.1, 0.1),) * 3),
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
        )
        navigator = LocalGoalNavigator(
            LocalNavigationConfig(
                goals={"test": {"north_offset_m": 2.0, "east_offset_m": 0.0}}
            )
        )
        goal = LocalGoal("test", 2.0, 0.0, 0.0, timestamp_monotonic=now)
        navigator.start(goal, state, now)
        goal_decision = navigator.update(state, now_monotonic=now + 0.1)
        mission = MissionState(
            mission_id="test-mission",
            mission_name="Test Mission",
            current_waypoint_index=0,
            current_waypoint_id="test",
            total_waypoints=2,
        )
        memory = EgoContextMemory()
        memory.update_perception(
            timestamp_monotonic=now,
            vlm_status="SUCCESS",
            terrain_type="snowfield",
            visible_landmarks=("rock",),
            scene_confidence=0.9,
            texture_quality=0.7,
            scene_summary="Rock visible across snowfield",
            vlm_model="qwen2.5vl:7b",
            vlm_timestamp_monotonic=now,
            vlm_source_frame_sequence=4,
            fused_scene=FusedSceneContext(
                status=FusedSceneStatus.STABLE,
                terrain_type="snowfield",
                visible_landmarks=("rock",),
                target_visible=True,
                texture_quality=0.7,
                fused_confidence=0.8,
                evidence_count=3,
                positive_target_count=3,
                source_frame_range=(2, 4),
                first_observation_timestamp_monotonic=now - 2.0,
                last_observation_timestamp_monotonic=now,
                scene_summary="Stable rock evidence",
                rationale="Target confirmed by 3 observations",
            ),
            latest_semantic_validation=SemanticValidationRecord(
                waypoint_id="rock-observation",
                expected_target_label="rock",
                deterministic_outcome=DeterministicObservationOutcome.COMPLETED,
                vlm_target_visible=True,
                visible_landmarks=("rock",),
                scene_confidence=0.9,
                result_freshness=SemanticEvidenceFreshness.FRESH,
                classification=SemanticValidationClassification.AGREEMENT,
                rationale="Expected target confirmed",
                timestamp_monotonic=now,
            ),
        )
        ego_context = memory.snapshot()
        lines = "\n".join(
            format_mission_lines(
                mission,
                goal_decision,
                None,
                None,
                None,
                ego_context=ego_context,
            )
        )
        self.assertIn("Test Mission", lines)
        self.assertIn("Waypoint 1/2 test", lines)
        self.assertIn("Navigator", lines)
        self.assertIn("Avoidance NONE", lines)
        self.assertIn("Recovery IDLE", lines)
        self.assertIn("Supervisor PENDING", lines)
        self.assertIn("Command F", lines)
        self.assertIn("Safety reason", lines)
        self.assertIn("VAL AGREEMENT rock", lines)
        self.assertIn("FUSE STABLE", lines)
        self.assertIn("TYES E3", lines)
        image = OperatorRenderer(
            OperatorViewConfig(window_width=1000, window_height=650)
        ).compose(
            state,
            ConfidenceEngine().evaluate(state),
            None,
            None,
            display_fps=10.0,
            depth_enabled=True,
            grid_overlay=True,
            paused=False,
            help_visible=False,
            navigation=goal_decision,
            mission=mission,
            ego_context=ego_context,
        )
        self.assertEqual(image.shape, (650, 1000, 3))

    def test_mission_status_renders_observation_state(self) -> None:
        now = monotonic()
        state = NavigationState(
            depth=replace(
                depth_state(((0.1, 0.1, 0.1),) * 3),
                source_frame_sequence=1,
                temporal_stability=0.95,
                inference_timestamp_monotonic=now,
            ),
            last_position_update_monotonic=now,
            last_attitude_update_monotonic=now,
            local_position_ok=True,
        )
        state.visual.camera_connected = True
        state.visual.stale = False
        state.visual.visual_health_score = 0.9
        state.visual.last_visual_update_monotonic = now
        navigator = LocalGoalNavigator(
            LocalNavigationConfig(
                goals={"test": {"north_offset_m": 3.0, "east_offset_m": 0.0}}
            )
        )
        goal = LocalGoal("rock-observation", 3.0, 0.0, 0.0, timestamp_monotonic=now)
        navigator.start(goal, state, now)
        goal_decision = navigator.update(state, now_monotonic=now + 0.1)
        tracker = ObservationTracker("rock-observation", ObservationCompletionConfig())
        observation = tracker.update(
            state,
            goal_distance_m=2.8,
            heading_error_deg=5.0,
            centre_risk=0.2,
            collision_detected=False,
            active_perception_state=RecoveryState.IDLE,
            now_monotonic=now,
        )
        mission = MissionState(
            mission_id="test-mission",
            mission_name="Test Mission",
            current_waypoint_index=0,
            current_waypoint_id="rock-observation",
            total_waypoints=1,
        )
        lines = "\n".join(
            format_mission_lines(
                mission,
                goal_decision,
                None,
                None,
                None,
                observation=observation,
            )
        )
        self.assertIn("Type OBSERVATION", lines)
        self.assertIn("Observation OBSERVING", lines)
        self.assertIn("Band 2.5-3.0 m", lines)
        self.assertIn("Dwell 0.0/1.0 s", lines)

    def test_operator_view_renders_immutable_ego_context(self) -> None:
        memory = EgoContextMemory(event_capacity=4)
        memory.update_mission(
            timestamp_monotonic=1.0,
            mission_id="demo",
            current_objective="Observe rock",
            current_waypoint_id="rock-observation",
            completion_mode="OBSERVATION",
        )
        memory.update_navigation(
            timestamp_monotonic=1.1,
            strategy="ACTIVE_PERCEPTION_OBSERVATION",
        )
        memory.update_safety(
            timestamp_monotonic=1.2,
            supervisor_state="HOVER",
        )
        memory.update_perception(
            timestamp_monotonic=1.3,
            vlm_status="SUCCESS",
            terrain_type="snowfield",
            visible_landmarks=("rock", "station"),
            target_visible=True,
            scene_confidence=0.9,
            texture_quality=0.7,
            scene_summary="Rock and station visible across snowfield",
            vlm_model="qwen2.5vl:7b",
            vlm_timestamp_monotonic=1.3,
            vlm_source_frame_sequence=4,
        )
        memory.record_event(1.4, "OBSERVATION", "Stable observation completed")
        snapshot = memory.snapshot()
        lines = "\n".join(format_ego_context_lines(snapshot))
        self.assertIn("Context r5", lines)
        self.assertIn("Observe rock", lines)
        self.assertIn("ACTIVE_PERCEPTION_OBSERVATION", lines)
        self.assertIn("VLM SUCCESS", lines)
        self.assertIn("Rock and station visible", lines)
        self.assertIn("Stable observation completed", lines)

    def test_commands_cheat_sheet_references_existing_files(self) -> None:
        commands = ROOT / "commands.txt"
        self.assertTrue(commands.is_file())
        content = commands.read_text(encoding="utf-8")
        project_files = (
            "scripts/operator_view.py",
            "scripts/camera_health_monitor.py",
            "scripts/depth_monitor.py",
            "ego_state_monitor.py",
            "src/first_flight.py",
            "src/offboard_movement.py",
            "scripts/viewpoint_benchmark.py",
            "scripts/depth_viewpoint_benchmark.py",
            "scripts/reactive_avoidance_demo.py",
            "config/reactive_avoidance.json",
            "scripts/local_goal_demo.py",
            "config/local_navigation.json",
            "config/active_perception.json",
            "simulation/worlds/polaris_antarctica.sdf",
            "requirements-depth.txt",
        )
        px4_files = (
            "scripts/start_polaris_sim.sh",
            "scripts/stop_polaris_sim.sh",
        )
        for relative in project_files:
            self.assertIn(relative, content)
            self.assertTrue((ROOT / relative).exists(), relative)
        px4_root = ROOT / "PX4-Autopilot"
        for relative in px4_files:
            self.assertIn(relative, content)
            self.assertTrue((px4_root / relative).exists(), relative)
        for section in range(18):
            self.assertIn(f"{section}.", content)


if __name__ == "__main__":
    unittest.main()
