from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.ego_state.telemetry import normalize_battery_remaining  # noqa: E402


class TelemetryTests(unittest.TestCase):
    def test_normalizes_fraction_and_percentage_backends(self) -> None:
        self.assertEqual(normalize_battery_remaining(0.75), 0.75)
        self.assertEqual(normalize_battery_remaining(75.0), 0.75)
        self.assertEqual(normalize_battery_remaining(100.0), 1.0)

    def test_clamps_invalid_battery_values(self) -> None:
        self.assertEqual(normalize_battery_remaining(-1.0), 0.0)
        self.assertEqual(normalize_battery_remaining(150.0), 1.0)


if __name__ == "__main__":
    unittest.main()
