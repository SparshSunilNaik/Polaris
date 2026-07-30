from __future__ import annotations

import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.observability import MissionEventRecorder, MissionEventType  # noqa: E402


class MissionEventRecorderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 10.0
        self.recorder = MissionEventRecorder(
            "run-1",
            monotonic_clock=lambda: self.now,
            utc_now=lambda: datetime(2026, 7, 30, tzinfo=timezone.utc),
        )

    def test_sequence_elapsed_and_default_data(self) -> None:
        self.now = 11.25
        event = self.recorder.record(MissionEventType.MISSION_STARTED, "test", "started")
        self.assertEqual([item["sequence"] for item in self.recorder.snapshot()], [1, 2])
        self.assertEqual(event["elapsed_s"], 1.25)
        self.assertEqual(event["timestamp_utc"], "2026-07-30T00:00:00Z")
        self.assertEqual(event["data"], {})

    def test_snapshot_is_isolated_and_finalize_is_idempotent(self) -> None:
        snapshot = self.recorder.snapshot()
        snapshot[0]["data"]["changed"] = True
        self.assertNotIn("changed", self.recorder.snapshot()[0]["data"])
        self.assertEqual(self.recorder.finalize(), self.recorder.finalize())

    def test_rejects_non_object_or_non_json_data(self) -> None:
        with self.assertRaises(ValueError):
            self.recorder.record("TEST", "test", "bad", data=[])
        with self.assertRaises(ValueError):
            self.recorder.record("TEST", "test", "bad", data={"value": float("nan")})

    def test_mode_transitions_emit_offboard_once(self) -> None:
        self.recorder.observe_telemetry(armed=False, in_air=False, flight_mode="HOLD", mission_state="PREFLIGHT", initiated_by_polaris=False)
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="OFFBOARD", mission_state="TAKEOFF", initiated_by_polaris=False)
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="OFFBOARD", mission_state="TAKEOFF", initiated_by_polaris=False)
        types = [event["event_type"] for event in self.recorder.snapshot()]
        self.assertEqual(types.count("OFFBOARD_ENTERED"), 1)
        self.assertEqual(types.count("ARMED"), 1)
        self.assertEqual(types.count("IN_AIR"), 1)

    def test_external_offboard_exit_is_observational(self) -> None:
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="OFFBOARD", mission_state="WAYPOINT_EXECUTE", initiated_by_polaris=False)
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="HOLD", mission_state="WAYPOINT_EXECUTE", initiated_by_polaris=False)
        events = self.recorder.snapshot()
        interrupted = [item for item in events if item["event_type"] == "MISSION_EXTERNALLY_INTERRUPTED"]
        self.assertEqual(len(interrupted), 1)
        self.assertEqual(interrupted[0]["data"]["classification"], "OBSERVED_EXTERNAL_MODE_CHANGE")
        self.assertNotIn("QGC", interrupted[0]["message"])

    def test_expected_landing_exit_is_not_external(self) -> None:
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="OFFBOARD", mission_state="WAYPOINT_EXECUTE", initiated_by_polaris=False)
        self.recorder.observe_telemetry(armed=True, in_air=True, flight_mode="LAND", mission_state="LAND", initiated_by_polaris=True)
        self.recorder.observe_telemetry(armed=False, in_air=False, flight_mode="LAND", mission_state="LAND", initiated_by_polaris=True)
        types = [event["event_type"] for event in self.recorder.snapshot()]
        self.assertIn("OFFBOARD_EXITED", types)
        self.assertNotIn("MISSION_EXTERNALLY_INTERRUPTED", types)
        self.assertEqual(types.count("LANDED"), 1)
        self.assertEqual(types.count("DISARMED"), 1)


if __name__ == "__main__":
    unittest.main()
