from __future__ import annotations

import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    AvoidanceMode,
    DesiredMotion,
    EscapeDirection,
    ReactiveAvoidanceController,
    write_avoidance_artifact,
)
from polaris.ego_state.models import NavigationState, VisualState  # noqa: E402
from polaris.perception.depth.models import (  # noqa: E402
    DepthCondition,
    DepthState,
)
from polaris.perception.models import VisualCondition  # noqa: E402


CLEAR = ((0.1, 0.1, 0.1), (0.1, 0.2, 0.1), (0.95, 0.95, 0.95))


def navigation(grid=CLEAR, sequence: int = 1, stability: float = 0.95) -> NavigationState:
    now = monotonic()
    return NavigationState(
        visual=VisualState(
            camera_connected=True,
            frame_age_ms=20.0,
            fps=30.0,
            visual_health_score=0.9,
            visual_condition=VisualCondition.HEALTHY,
            texture_score=10.0,
            stale=False,
            reasons=(),
            last_visual_update_monotonic=now,
        ),
        depth=DepthState(
            available=True,
            stale=False,
            source_frame_sequence=sequence,
            inference_timestamp_monotonic=now,
            depth_health_score=0.9,
            temporal_stability=stability,
            condition=DepthCondition.HEALTHY,
            risk_grid=grid,
        ),
        local_position_ok=True,
        last_position_update_monotonic=now,
        last_attitude_update_monotonic=now,
    )


def desired(now: float = 1.0) -> DesiredMotion:
    return DesiredMotion(forward_m_s=0.4, timestamp_monotonic=now)


class ReactiveAvoidanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = AvoidanceConfig()
        self.controller = ReactiveAvoidanceController(self.config)

    def update(self, grid, sequence, now):
        return self.controller.update(desired(now), navigation(grid, sequence), "NORMAL", now)

    def persistent(self, grid, start_sequence=1, start_time=1.0):
        decision = None
        for offset in range(self.config.blocked_persistence_updates):
            decision = self.update(grid, start_sequence + offset, start_time + offset * 0.4)
        return decision

    def test_clear_path_passes_forward_command(self) -> None:
        decision = self.update(CLEAR, 1, 1.0)
        self.assertEqual(decision.mode, AvoidanceMode.CLEAR)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.4)

    def test_caution_risk_reduces_speed(self) -> None:
        grid = ((0.1, 0.1, 0.1), (0.1, 0.5, 0.1), (1.0, 1.0, 1.0))
        decision = self.update(grid, 1, 1.0)
        self.assertEqual(decision.mode, AvoidanceMode.CAUTION)
        self.assertEqual(decision.proposed_command.forward_m_s, self.config.cautious_speed_m_s)

    def test_low_visual_health_refuses_motion(self) -> None:
        state = navigation(CLEAR, 1)
        state.visual.visual_health_score = self.config.minimum_visual_health - 0.05
        state.visual.visual_condition = VisualCondition.DEGRADED
        decision = self.controller.update(desired(1.0), state, "DEGRADED", 1.0)
        self.assertEqual(decision.mode, AvoidanceMode.HOVER)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.0)

    def test_centre_obstacle_stops_and_chooses_left(self) -> None:
        grid = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        decision = self.persistent(grid)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.0)
        self.assertLess(decision.proposed_command.right_m_s, 0.0)

    def test_centre_obstacle_chooses_right(self) -> None:
        grid = ((0.8, 0.1, 0.1), (0.8, 0.7, 0.2), (1.0, 1.0, 1.0))
        decision = self.persistent(grid)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_RIGHT)
        self.assertGreater(decision.proposed_command.right_m_s, 0.0)

    def test_left_blocked_biases_right(self) -> None:
        grid = ((0.8, 0.1, 0.1), (0.8, 0.2, 0.1), (1.0, 1.0, 1.0))
        decision = self.persistent(grid)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_RIGHT)

    def test_right_blocked_biases_left(self) -> None:
        grid = ((0.1, 0.1, 0.8), (0.1, 0.2, 0.8), (1.0, 1.0, 1.0))
        decision = self.persistent(grid)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)

    def test_both_sides_blocked_retreats(self) -> None:
        grid = ((0.8, 0.1, 0.8), (0.8, 0.7, 0.8), (1.0, 1.0, 1.0))
        decision = None
        for sequence in range(1, self.config.retreat_persistence_updates + 1):
            decision = self.update(grid, sequence, float(sequence))
        self.assertIsNotNone(decision)
        self.assertEqual(decision.mode, AvoidanceMode.RETREAT)
        self.assertLess(decision.proposed_command.forward_m_s, 0.0)

    def test_both_sides_blocked_with_clear_centre_never_sidesteps(self) -> None:
        grid = ((0.8, 0.1, 0.8), (0.8, 0.2, 0.8), (1.0, 1.0, 1.0))
        decision = None
        for sequence in range(1, self.config.retreat_persistence_updates + 2):
            decision = self.update(grid, sequence, float(sequence))
        self.assertEqual(decision.mode, AvoidanceMode.CAUTION)
        self.assertEqual(
            decision.proposed_command.forward_m_s,
            self.config.cautious_speed_m_s,
        )
        self.assertEqual(decision.proposed_command.right_m_s, 0.0)

    def test_down_ground_risk_alone_is_ignored(self) -> None:
        decision = self.update(CLEAR, 1, 1.0)
        self.assertGreater(decision.risk.lower, 0.9)
        self.assertEqual(decision.mode, AvoidanceMode.CLEAR)

    def test_one_frame_obstacle_does_not_trigger_avoidance(self) -> None:
        blocked = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        first = self.update(blocked, 1, 1.0)
        clear = self.update(CLEAR, 2, 1.4)
        self.assertNotIn(first.mode, {AvoidanceMode.AVOID_LEFT, AvoidanceMode.AVOID_RIGHT})
        self.assertEqual(clear.mode, AvoidanceMode.CLEAR)

    def test_hysteresis_prevents_immediate_resume(self) -> None:
        blocked = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        decision = self.persistent(blocked)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)
        decision = self.update(CLEAR, 3, 2.0)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.0)

    def test_direction_hold_prevents_left_right_flip(self) -> None:
        left_escape = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        decision = self.persistent(left_escape)
        opposite = ((0.5, 0.1, 0.2), (0.5, 0.7, 0.2), (1.0, 1.0, 1.0))
        decision = self.update(opposite, 3, 1.9)
        self.assertEqual(decision.chosen_direction, EscapeDirection.LEFT)
        self.assertEqual(decision.mode, AvoidanceMode.AVOID_LEFT)

    def test_blocked_chosen_escape_sector_hovers_immediately(self) -> None:
        left_escape = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        self.persistent(left_escape)
        left_becomes_blocked = (
            (0.8, 0.1, 0.1),
            (0.8, 0.7, 0.2),
            (1.0, 1.0, 1.0),
        )
        decision = self.update(left_becomes_blocked, 3, 1.9)
        self.assertEqual(decision.mode, AvoidanceMode.HOVER)
        self.assertEqual(decision.proposed_command.right_m_s, 0.0)

    def test_hover_does_not_reset_avoidance_episode_timeout(self) -> None:
        config = replace(self.config, maximum_avoidance_duration_s=1.0)
        controller = ReactiveAvoidanceController(config)
        left_escape = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        controller.update(desired(1.0), navigation(left_escape, 1), "NORMAL", 1.0)
        controller.update(desired(1.4), navigation(left_escape, 2), "NORMAL", 1.4)
        left_blocked = ((0.8, 0.1, 0.1), (0.8, 0.7, 0.2), (1.0, 1.0, 1.0))
        hover = controller.update(
            desired(1.8),
            navigation(left_blocked, 3),
            "NORMAL",
            1.8,
        )
        self.assertEqual(hover.mode, AvoidanceMode.HOVER)
        abort = controller.update(
            desired(2.5),
            navigation(left_blocked, 4),
            "NORMAL",
            2.5,
        )
        self.assertEqual(abort.mode, AvoidanceMode.ABORT)

    def test_active_perception_starts_a_fresh_avoidance_episode(self) -> None:
        config = replace(
            self.config,
            blocked_persistence_updates=1,
            maximum_avoidance_duration_s=1.0,
        )
        controller = ReactiveAvoidanceController(config)
        left_escape = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        controller.update(desired(1.0), navigation(left_escape, 1), "NORMAL", 1.0)
        observation = DesiredMotion(
            yaw_rate_deg_s=15.0,
            source="active-perception-scan",
            timestamp_monotonic=2.5,
        )
        scanning = controller.update(
            observation,
            navigation(left_escape, 2),
            "NORMAL",
            2.5,
        )
        self.assertEqual(scanning.mode, AvoidanceMode.CAUTION)
        self.assertEqual(scanning.elapsed_avoidance_s, 0.0)
        resumed = controller.update(
            desired(2.6),
            navigation(left_escape, 3),
            "NORMAL",
            2.6,
        )
        self.assertIn(
            resumed.mode,
            {AvoidanceMode.AVOID_LEFT, AvoidanceMode.AVOID_RIGHT},
        )
        abort = controller.update(
            desired(3.7),
            navigation(left_escape, 4),
            "NORMAL",
            3.7,
        )
        self.assertEqual(abort.mode, AvoidanceMode.ABORT)

    def test_maximum_reversal_count_aborts(self) -> None:
        config = replace(
            self.config,
            blocked_persistence_updates=1,
            clear_persistence_updates=1,
            preferred_direction_hold_s=0.0,
            sidestep_cooldown_s=0.0,
            maximum_reversals=0,
        )
        controller = ReactiveAvoidanceController(config)
        left_escape = ((0.1, 0.1, 0.8), (0.2, 0.7, 0.8), (1.0, 1.0, 1.0))
        right_escape = ((0.8, 0.1, 0.1), (0.8, 0.7, 0.2), (1.0, 1.0, 1.0))
        controller.update(desired(1), navigation(left_escape, 1), "NORMAL", 1)
        controller.update(desired(2), navigation(CLEAR, 2), "NORMAL", 2)
        decision = controller.update(desired(3), navigation(right_escape, 3), "NORMAL", 3)
        self.assertEqual(decision.mode, AvoidanceMode.ABORT)

    def test_invalid_grid_aborts(self) -> None:
        grid = ((0.1, 0.1, 0.1), (0.1, float("nan"), 0.1), (1.0, 1.0, 1.0))
        decision = self.update(grid, 1, 1.0)
        self.assertEqual(decision.mode, AvoidanceMode.ABORT)
        self.assertEqual(decision.proposed_command.forward_m_s, 0.0)

    def test_shutdown_command_is_zero_velocity(self) -> None:
        command = self.controller.shutdown_command(5.0)
        self.assertEqual(
            (command.forward_m_s, command.right_m_s, command.down_m_s, command.yaw_rate_deg_s),
            (0.0, 0.0, 0.0, 0.0),
        )

    def test_stationary_hold_does_not_create_sidestep(self) -> None:
        grid = ((0.8, 0.1, 0.8), (0.8, 0.7, 0.8), (1.0, 1.0, 1.0))
        decision = self.controller.update(
            DesiredMotion.hover(1.0, "goal-hold"),
            navigation(grid, 1),
            "NORMAL",
            1.0,
        )
        self.assertEqual(decision.mode, AvoidanceMode.CLEAR)
        self.assertEqual(decision.proposed_command.right_m_s, 0.0)

    def test_decision_json_serialization(self) -> None:
        decision = self.update(CLEAR, 1, 1.0)
        payload = json.dumps(decision.to_dict(), allow_nan=False)
        self.assertIn('"mode": "CLEAR"', payload)

    def test_json_artifact_writer(self) -> None:
        decision = self.update(CLEAR, 1, 1.0)
        with TemporaryDirectory() as directory:
            path = write_avoidance_artifact(
                {"scenario": "synthetic", "records": [decision.to_dict()]},
                directory,
                "test",
            )
            loaded = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(loaded["records"][0]["mode"], "CLEAR")


if __name__ == "__main__":
    unittest.main()
