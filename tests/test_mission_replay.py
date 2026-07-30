from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.mission import format_mission_replay  # noqa: E402


class MissionReplayTests(unittest.TestCase):
    def test_summary_uses_existing_artifact_fields(self) -> None:
        report = format_mission_replay(self.payload(), "artifact.json")
        self.assertIn("Mission: Replay Test", report)
        self.assertIn("Outcome: ABORTED", report)
        self.assertIn("Duration: 12.3 s", report)
        self.assertIn("Completed waypoints: one", report)
        self.assertIn("Failed waypoint: two", report)
        self.assertIn("Abort reason: bounded failure", report)
        self.assertIn("Recovery count: 1", report)
        self.assertIn("Avoidance count: 3", report)
        self.assertIn("Collision count: 0", report)
        self.assertIn("Landing: CONFIRMED", report)
        self.assertIn("Disarm: CONFIRMED", report)
        self.assertIn("Artifact: artifact.json", report)

    def test_timeline_is_chronological_across_transition_types(self) -> None:
        report = format_mission_replay(self.payload(), "artifact.json")
        mission = report.index("MISSION WAYPOINT_EXECUTE")
        avoidance = report.index("AVOIDANCE AVOID_RIGHT")
        recovery = report.index("RECOVERY COMPLETE")
        supervisor = report.index("SUPERVISOR ABORT")
        self.assertLess(mission, avoidance)
        self.assertLess(avoidance, recovery)
        self.assertLess(recovery, supervisor)

    def test_no_flight_reports_disarm_not_required(self) -> None:
        payload = self.payload()
        payload["final_result"] = "NO_FLIGHT_COMPLETED"
        payload["landing_result"] = "NOT_ATTEMPTED"
        report = format_mission_replay(payload, "artifact.json")
        self.assertIn("Disarm: NOT_REQUIRED", report)

    def test_replay_accepts_optional_ordered_events(self) -> None:
        payload = self.payload()
        payload["schema_version"] = 1
        payload["events"] = [
            {
                "sequence": 1,
                "timestamp_utc": "2026-07-30T00:00:00Z",
                "elapsed_s": 0.0,
                "event_type": "MISSION_STARTED",
                "source": "test",
                "severity": "INFO",
                "message": "started",
                "data": {},
            }
        ]
        from polaris.mission import validate_mission_replay_payload

        validate_mission_replay_payload(payload)

    def test_outcome_preserves_completed_with_skips(self) -> None:
        payload = self.payload()
        payload["mission_result"] = "COMPLETED_WITH_SKIPS"
        payload["final_result"] = "COMPLETED"
        report = format_mission_replay(payload, "artifact.json")
        self.assertIn("Outcome: COMPLETED_WITH_SKIPS", report)

    def test_timeline_omits_routine_supervisor_noise(self) -> None:
        payload = self.payload()
        payload["supervisor_transitions"].insert(
            0,
            {
                "timestamp_monotonic": 11.5,
                "mode": "SLOW",
                "waypoint_id": "one",
                "reasons": ["Confidence constrained speed"],
            },
        )
        report = format_mission_replay(payload, "artifact.json")
        self.assertNotIn("SUPERVISOR SLOW", report)
        self.assertIn("SUPERVISOR ABORT", report)

    def test_timeline_renders_emergency_confirmation_transitions(self) -> None:
        payload = self.payload()
        payload["supervisor_transitions"] = [
            {
                "timestamp_monotonic": 10.5,
                "mode": "HOVER",
                "waypoint_id": "one",
                "reasons": ["Emergency centre proximity suspected"],
                "emergency_confirmation": {
                    "state": "SUSPECTED",
                    "observed_frames": 1,
                    "required_frames": 2,
                    "age_s": 0.0,
                    "reason": "Emergency centre proximity suspected",
                },
            },
            {
                "timestamp_monotonic": 10.8,
                "mode": "PASS_THROUGH",
                "waypoint_id": "one",
                "reasons": ["Emergency centre proximity risk cleared"],
                "emergency_confirmation": {
                    "state": "CLEAR",
                    "observed_frames": 0,
                    "required_frames": 2,
                    "age_s": 0.0,
                    "reason": "Emergency centre proximity risk cleared",
                },
            },
            {
                "timestamp_monotonic": 11.1,
                "mode": "ABORT",
                "waypoint_id": "one",
                "reasons": ["Emergency centre proximity confirmed"],
                "emergency_confirmation": {
                    "state": "CONFIRMED",
                    "observed_frames": 2,
                    "required_frames": 2,
                    "age_s": 0.3,
                    "reason": "Emergency centre proximity confirmed",
                },
            },
        ]
        report = format_mission_replay(payload, "artifact.json")
        self.assertIn("EMERGENCY SUSPECTED 1/2", report)
        self.assertIn("EMERGENCY CLEAR 0/2", report)
        self.assertIn("EMERGENCY CONFIRMED 2/2", report)

    def test_hard_fault_confirmation_reset_remains_supervisor_abort(self) -> None:
        payload = self.payload()
        payload["supervisor_transitions"] = [
            {
                "timestamp_monotonic": 10.5,
                "mode": "ABORT",
                "waypoint_id": "one",
                "reasons": ["Airborne collision detected"],
                "emergency_confirmation": {
                    "state": "CLEAR",
                    "observed_frames": 0,
                    "required_frames": 2,
                    "age_s": 0.0,
                    "reason": "Bypassed by immediate hard safety condition",
                },
            }
        ]
        report = format_mission_replay(payload, "artifact.json")
        self.assertIn("SUPERVISOR ABORT", report)
        self.assertNotIn("EMERGENCY CLEAR", report)

    def test_replay_reports_observation_completion(self) -> None:
        payload = self.payload()
        payload["waypoint_results"][0].update(
            {
                "waypoint_id": "rock-observation",
                "completion_mode": "OBSERVATION",
                "observation": {
                    "completion_distance_m": 2.8,
                    "dwell_duration_s": 1.0,
                },
            }
        )
        payload["observation_events"] = [
            {
                "timestamp_monotonic": 12.5,
                "waypoint_id": "rock-observation",
                "event_type": "COMPLETED",
                "goal_distance_m": 2.8,
                "dwell_elapsed_s": 1.0,
                "reason": "Stable observation complete",
            }
        ]
        report = format_mission_replay(payload, "artifact.json")
        self.assertIn(
            "rock-observation completed by stable observation at 2.80 metres",
            report,
        )
        self.assertIn("OBSERVATION COMPLETED [rock-observation]", report)

    @staticmethod
    def payload() -> dict:
        return {
            "mission_plan": {"name": "Replay Test"},
            "mission_state": {
                "mission_name": "Replay Test",
                "mission_elapsed_s": 12.34,
                "completed_waypoints": ["one"],
                "failed_waypoints": ["two"],
                "abort_reason": "bounded failure",
            },
            "waypoint_results": [
                {
                    "recovery_count": 1,
                    "final_metrics": {"avoidance_count": 3},
                }
            ],
            "mission_transitions": [
                {
                    "timestamp_monotonic": 10.0,
                    "state": "WAYPOINT_EXECUTE",
                    "waypoint_id": "one",
                    "reason": "Started",
                }
            ],
            "avoidance_transitions": [
                {
                    "timestamp_monotonic": 11.0,
                    "mode": "AVOID_RIGHT",
                    "waypoint_id": "one",
                }
            ],
            "active_perception_recoveries": [
                {
                    "waypoint_id": "one",
                    "decision": {
                        "timestamp_monotonic": 12.0,
                        "state": "COMPLETE",
                        "trigger": "Low visual health",
                        "selected_heading_deg": 15.0,
                    },
                }
            ],
            "supervisor_transitions": [
                {
                    "timestamp_monotonic": 13.0,
                    "mode": "ABORT",
                    "waypoint_id": "two",
                    "reasons": ["bounded failure"],
                }
            ],
            "navigation_transitions": [],
            "replanning_decisions": [],
            "collision_count": 0,
            "abort_reason": "bounded failure",
            "landing_result": "CONFIRMED",
            "mission_result": "ABORTED",
            "final_result": "ABORTED",
        }


if __name__ == "__main__":
    unittest.main()
