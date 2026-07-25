from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.control import RiskSummary  # noqa: E402
from polaris.mission import (  # noqa: E402
    BoundedMissionReplanner,
    FailureAction,
    MissionOrchestratorConfig,
    MissionWaypoint,
)
from polaris.navigation import LocalGoal  # noqa: E402


def waypoint(identifier="blocked", north=0.0, east=4.0) -> MissionWaypoint:
    return MissionWaypoint(
        identifier,
        LocalGoal(identifier, north, east, 0.0),
        True,
        False,
        failure_action=FailureAction.REPLAN_AROUND_WAYPOINT,
    )


class MissionReplannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MissionOrchestratorConfig()
        self.replanner = BoundedMissionReplanner(self.config)
        self.risk = RiskSummary(0.7, 0.2, 0.8, 0.2, 0.0, 0.7, True)

    def test_lateral_staging_waypoint_generation(self) -> None:
        decision = self.replanner.generate_lateral_staging(
            waypoint(),
            current_north_offset_m=0,
            current_east_offset_m=2,
            current_yaw_deg=90,
            risk=self.risk,
            replan_index=0,
            existing_ids={"blocked"},
        )
        self.assertTrue(decision.accepted)
        self.assertTrue(decision.generated_waypoint.generated)
        self.assertIn("generated-blocked-r1", decision.generated_waypoint.waypoint_id)

    def test_world_bounds_enforcement(self) -> None:
        decision = self.replanner.generate_lateral_staging(
            waypoint(east=13),
            current_north_offset_m=-18,
            current_east_offset_m=12.9,
            current_yaw_deg=0,
            risk=RiskSummary(0.7, 0.8, 0.2, 0.2, 0, 0.7, True),
            replan_index=0,
            existing_ids=set(),
        )
        self.assertFalse(decision.accepted)

    def test_maximum_offset_enforcement(self) -> None:
        config = replace(
            self.config,
            maximum_generated_waypoint_offset_m=0.5,
            lateral_replan_offset_m=0.5,
        )
        replanner = BoundedMissionReplanner(config)
        decision = replanner.generate_lateral_staging(
            waypoint(),
            current_north_offset_m=0,
            current_east_offset_m=2,
            current_yaw_deg=90,
            risk=self.risk,
            replan_index=0,
            existing_ids=set(),
        )
        self.assertTrue(decision.accepted)
        self.assertLessEqual(abs(decision.generated_waypoint.goal.target_north_m), 0.5)

    def test_replan_limit(self) -> None:
        decision = self.replanner.generate_lateral_staging(
            waypoint(),
            current_north_offset_m=0,
            current_east_offset_m=2,
            current_yaw_deg=90,
            risk=self.risk,
            replan_index=1,
            existing_ids=set(),
        )
        self.assertFalse(decision.accepted)

    def test_generated_id_must_be_unique(self) -> None:
        decision = self.replanner.generate_lateral_staging(
            waypoint(),
            current_north_offset_m=0,
            current_east_offset_m=2,
            current_yaw_deg=90,
            risk=self.risk,
            replan_index=0,
            existing_ids={"generated-blocked-r1"},
        )
        self.assertFalse(decision.accepted)

    def test_no_valid_replan_for_invalid_risk(self) -> None:
        decision = self.replanner.generate_lateral_staging(
            waypoint(),
            current_north_offset_m=0,
            current_east_offset_m=2,
            current_yaw_deg=90,
            risk=replace(self.risk, valid=False),
            replan_index=0,
            existing_ids=set(),
        )
        self.assertFalse(decision.accepted)

    def test_previous_waypoint_retreat(self) -> None:
        decision = self.replanner.generate_previous_waypoint_retreat(
            waypoint("previous", 0, 2),
            waypoint(),
            replan_index=0,
            existing_ids={"blocked"},
        )
        self.assertTrue(decision.accepted)
        self.assertIn("generated-retreat", decision.generated_waypoint.waypoint_id)


if __name__ == "__main__":
    unittest.main()
