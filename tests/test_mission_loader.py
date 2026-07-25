from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.mission import (  # noqa: E402
    MissionOrchestratorConfig,
    WaypointType,
    load_mission,
    load_mission_file,
)


class MissionLoaderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MissionOrchestratorConfig.from_json(
            ROOT / "config/mission_orchestrator.json"
        )

    def test_all_configured_missions_are_valid(self) -> None:
        mission_ids = (
            "two-point-demo",
            "polaris-demo",
            "recovery-demo",
            "failure-demo",
            "required-failure-demo",
            "collision-demo",
        )
        plans = [
            load_mission(item, ROOT / "config/missions", self.config)
            for item in mission_ids
        ]
        self.assertEqual(plans[0].mission_id, "two-point-demo")
        self.assertTrue(all(plan.waypoints for plan in plans))

    def test_duplicate_waypoint_ids_are_rejected(self) -> None:
        values = self._valid_values()
        values["waypoints"].append(dict(values["waypoints"][0]))
        with self.assertRaisesRegex(ValueError, "unique"):
            self._load(values)

    def test_empty_mission_is_rejected(self) -> None:
        values = self._valid_values()
        values["waypoints"] = []
        with self.assertRaisesRegex(ValueError, "at least one"):
            self._load(values)

    def test_invalid_coordinates_are_rejected(self) -> None:
        values = self._valid_values()
        values["waypoints"][0]["goal"]["east_offset_m"] = 99.0
        with self.assertRaisesRegex(ValueError, "outside"):
            self._load(values)

    def test_required_skippable_contradiction_is_rejected(self) -> None:
        values = self._valid_values()
        values["waypoints"][0]["skippable"] = True
        with self.assertRaisesRegex(ValueError, "required waypoint"):
            self._load(values)

    def test_invalid_failure_policy_is_rejected(self) -> None:
        values = self._valid_values()
        values["failure_policy"] = "FLY_ANYWHERE"
        with self.assertRaises(ValueError):
            self._load(values)

    def test_existing_waypoint_defaults_to_position_completion(self) -> None:
        plan = self._load(self._valid_values())
        self.assertEqual(plan.waypoints[0].waypoint_type, WaypointType.POSITION)
        self.assertIsNone(plan.waypoints[0].observation)

    def test_observation_waypoint_configuration_is_loaded(self) -> None:
        values = self._valid_values()
        values["waypoints"][0].update(
            {
                "type": "OBSERVATION",
                "observation": {
                    "minimum_goal_distance_m": 2.5,
                    "maximum_goal_distance_m": 3.0,
                    "maximum_heading_error_deg": 12.0,
                    "minimum_visual_health": 0.35,
                    "minimum_depth_health": 0.4,
                    "minimum_depth_stability": 0.75,
                    "maximum_centre_risk": 0.45,
                    "maximum_horizontal_speed_m_s": 0.2,
                    "required_stable_s": 1.0,
                    "minimum_fresh_observations": 3,
                    "maximum_wait_s": 6.0,
                },
            }
        )
        waypoint = self._load(values).waypoints[0]
        self.assertEqual(waypoint.waypoint_type, WaypointType.OBSERVATION)
        self.assertEqual(waypoint.observation.maximum_centre_risk, 0.45)

    def _load(self, values):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(values), encoding="utf-8")
            return load_mission_file(path, self.config)

    @staticmethod
    def _valid_values() -> dict:
        return {
            "mission_id": "test",
            "name": "Test",
            "description": "Test mission",
            "failure_policy": "ABORT_MISSION",
            "maximum_duration_s": 30,
            "maximum_path_length_m": 5,
            "maximum_replans": 0,
            "return_or_land_policy": "LAND_AT_CURRENT_POSITION",
            "waypoints": [
                {
                    "waypoint_id": "one",
                    "goal": {"north_offset_m": 0, "east_offset_m": 2},
                    "required": True,
                    "skippable": False,
                }
            ],
        }


if __name__ == "__main__":
    unittest.main()
