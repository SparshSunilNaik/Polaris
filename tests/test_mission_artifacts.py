from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.control import (  # noqa: E402
    EmergencyConfirmation,
    EmergencyConfirmationState,
    SafeMotion,
    SupervisorMode,
)
from polaris.mission import append_supervisor_transition, write_mission_artifact  # noqa: E402


class MissionArtifactTests(unittest.TestCase):
    def test_original_generated_attempts_outcomes_and_landing_serialize(self) -> None:
        payload = {
            "original_waypoints": [{"waypoint_id": "one"}],
            "generated_waypoints": [{"waypoint_id": "generated-one-r1"}],
            "waypoint_results": [
                {"waypoint_id": "one", "attempt": 2, "outcome": "GOAL_REACHED"}
            ],
            "landing_result": "CONFIRMED",
            "mission_result": "COMPLETED",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = write_mission_artifact(payload, directory, "test")
            restored = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(restored, payload)

    def test_writer_rejects_non_finite_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                write_mission_artifact({"bad": float("nan")}, directory, "bad")

    def test_artifact_records_confirmation_transitions_without_flooding(self) -> None:
        transitions = []
        previous = None
        for state, reason in (
            (EmergencyConfirmationState.SUSPECTED, "Emergency evidence appeared"),
            (EmergencyConfirmationState.SUSPECTED, "Emergency evidence appeared"),
            (EmergencyConfirmationState.CLEAR, "Emergency evidence cleared"),
            (EmergencyConfirmationState.CONFIRMED, "Emergency evidence persisted"),
        ):
            confirmation = EmergencyConfirmation(state=state, reason=reason)
            safe = SafeMotion.hover(
                10.0,
                SupervisorMode.HOVER,
                (reason,),
                state is EmergencyConfirmationState.CONFIRMED,
                20.0,
                emergency_confirmation=confirmation,
            )
            previous = append_supervisor_transition(
                transitions,
                previous,
                safe,
                "one",
                10.0,
            )
        self.assertEqual(
            [item["emergency_confirmation"]["state"] for item in transitions],
            ["SUSPECTED", "CLEAR", "CONFIRMED"],
        )

    def test_artifact_records_observation_events_and_completion_mode(self) -> None:
        payload = {
            "observation_events": [
                {
                    "event_type": "CANDIDATE_ENTERED",
                    "waypoint_id": "rock-observation",
                },
                {
                    "event_type": "DWELL_STARTED",
                    "waypoint_id": "rock-observation",
                },
                {
                    "event_type": "DWELL_RESET",
                    "waypoint_id": "rock-observation",
                    "reason": "Centre risk exceeded observation maximum",
                },
                {
                    "event_type": "COMPLETED",
                    "waypoint_id": "rock-observation",
                },
            ],
            "waypoint_results": [
                {
                    "waypoint_id": "rock-observation",
                    "completion_mode": "OBSERVATION",
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = write_mission_artifact(payload, directory, "observation")
            restored = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(restored, payload)


if __name__ == "__main__":
    unittest.main()
