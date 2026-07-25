from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/mission_replay.py"


class MissionReplayCliTests(unittest.TestCase):
    def test_valid_completed_artifact(self) -> None:
        result = self.run_artifact(self.payload("COMPLETED", "COMPLETED"))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Mission: CLI Replay Test", result.stdout)
        self.assertIn("Outcome: COMPLETED", result.stdout)
        self.assertIn("MISSION MISSION_COMPLETE", result.stdout)

    def test_valid_aborted_artifact(self) -> None:
        payload = self.payload("ABORTED", "ABORTED")
        payload["abort_reason"] = "Emergency centre proximity risk"
        payload["mission_state"]["abort_reason"] = payload["abort_reason"]
        payload["mission_state"]["failed_waypoints"] = ["two"]
        result = self.run_artifact(payload)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Outcome: ABORTED", result.stdout)
        self.assertIn("Failed waypoint: two", result.stdout)
        self.assertIn("Abort reason: Emergency centre proximity risk", result.stdout)

    def test_missing_argument(self) -> None:
        result = self.run_cli()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("artifact", result.stderr)

    def test_missing_file(self) -> None:
        result = self.run_cli("does-not-exist.json")
        self.assertEqual(result.returncode, 2)
        self.assertIn("file does not exist", result.stderr)

    def test_invalid_json(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text("{not-json", encoding="utf-8")
            result = self.run_cli(str(path))
        self.assertEqual(result.returncode, 3)
        self.assertIn("invalid JSON", result.stderr)

    def test_malformed_artifact(self) -> None:
        result = self.run_artifact({"schema_version": 1})
        self.assertEqual(result.returncode, 4)
        self.assertIn("malformed artifact", result.stderr)

    def test_unsupported_schema(self) -> None:
        payload = self.payload("COMPLETED", "COMPLETED")
        payload["schema_version"] = 2
        result = self.run_artifact(payload)
        self.assertEqual(result.returncode, 4)
        self.assertIn("Unsupported mission artifact schema version", result.stderr)

    def test_help(self) -> None:
        result = self.run_cli("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("usage:", result.stdout)
        self.assertIn("artifact", result.stdout)

    def test_source_artifact_remains_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(self.payload("COMPLETED", "COMPLETED")), encoding="utf-8")
            before = path.read_bytes()
            result = self.run_cli(str(path))
            after = path.read_bytes()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(after, before)

    def run_artifact(self, payload: dict) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            return self.run_cli(str(path))

    @staticmethod
    def run_cli(*arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(SCRIPT), *arguments],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

    @staticmethod
    def payload(mission_result: str, final_result: str) -> dict:
        return {
            "schema_version": 1,
            "mission_plan": {"name": "CLI Replay Test"},
            "mission_state": {
                "mission_name": "CLI Replay Test",
                "mission_elapsed_s": 8.5,
                "completed_waypoints": ["one"],
                "failed_waypoints": [],
                "abort_reason": None,
            },
            "waypoint_results": [
                {
                    "recovery_count": 0,
                    "final_metrics": {"avoidance_count": 1},
                }
            ],
            "mission_transitions": [
                {
                    "timestamp_monotonic": 1.0,
                    "state": "MISSION_COMPLETE",
                    "waypoint_id": "one",
                    "reason": "All waypoints completed",
                }
            ],
            "navigation_transitions": [],
            "avoidance_transitions": [],
            "supervisor_transitions": [],
            "active_perception_recoveries": [],
            "replanning_decisions": [],
            "collision_count": 0,
            "abort_reason": None,
            "landing_result": "CONFIRMED",
            "mission_result": mission_result,
            "final_result": final_result,
        }


if __name__ == "__main__":
    unittest.main()
