from __future__ import annotations

import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.ego_context import EgoContextMemory  # noqa: E402


class EgoContextMemoryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.memory = EgoContextMemory(event_capacity=3)

    def test_default_initialization(self) -> None:
        snapshot = self.memory.snapshot()
        self.assertEqual(snapshot.revision, 0)
        self.assertEqual(snapshot.mission.mission_id, "NONE")
        self.assertEqual(snapshot.mission.phase, "IDLE")
        self.assertEqual(snapshot.navigation.navigator_state, "IDLE")
        self.assertEqual(snapshot.safety.emergency_state, "CLEAR")
        self.assertEqual(snapshot.recent_events, ())

    def test_snapshot_is_deeply_immutable_and_stable(self) -> None:
        metadata = {"waypoints": ["one"], "details": {"attempt": 1}}
        self.memory.record_event(1.0, "MISSION", "Started", metadata)
        snapshot = self.memory.snapshot()
        metadata["waypoints"].append("two")
        metadata["details"]["attempt"] = 2
        with self.assertRaises(FrozenInstanceError):
            snapshot.revision = 9
        with self.assertRaises(TypeError):
            snapshot.recent_events[0].metadata["new"] = "value"
        self.assertEqual(snapshot.recent_events[0].metadata["waypoints"], ("one",))
        self.assertEqual(snapshot.recent_events[0].metadata["details"]["attempt"], 1)
        self.memory.update_mission(timestamp_monotonic=2.0, phase="TAKEOFF")
        self.assertEqual(snapshot.mission.phase, "IDLE")

    def test_mission_updates(self) -> None:
        snapshot = self.memory.update_mission(
            timestamp_monotonic=1.0,
            mission_id="demo",
            mission_name="Demo Mission",
            phase="WAYPOINT_EXECUTE",
            result="PENDING",
            current_objective="Observe rock",
            current_waypoint_id="rock-observation",
            waypoint_type="OBSERVATION",
            completion_mode="NONE",
            completed_waypoints=("corridor-staging",),
            observation_status="OBSERVING",
        )
        self.assertEqual(snapshot.mission.current_objective, "Observe rock")
        self.assertEqual(snapshot.mission.completed_waypoints, ("corridor-staging",))
        self.assertEqual(snapshot.revision, 1)

    def test_navigation_updates(self) -> None:
        snapshot = self.memory.update_navigation(
            timestamp_monotonic=1.0,
            navigator_state="RECOVER_VIEW",
            recovery_state="SCANNING",
            active_perception_owns_motion=True,
            strategy="ACTIVE_PERCEPTION",
            retry_count=1,
            recent_recoveries=("Low texture scan",),
        )
        self.assertTrue(snapshot.navigation.active_perception_owns_motion)
        self.assertEqual(snapshot.navigation.recent_recoveries, ("Low texture scan",))

    def test_perception_updates(self) -> None:
        snapshot = self.memory.update_perception(
            timestamp_monotonic=1.0,
            visual_health=0.8,
            depth_health=0.7,
            depth_stability=0.9,
            condition="HEALTHY",
        )
        self.assertEqual(snapshot.perception.visual_health, 0.8)
        self.assertEqual(snapshot.perception.condition, "HEALTHY")

    def test_safety_updates(self) -> None:
        snapshot = self.memory.update_safety(
            timestamp_monotonic=1.0,
            supervisor_state="HOVER",
            emergency_state="SUSPECTED",
            collision_detected=False,
            hard_override=False,
            current_ownership="ACTIVE_PERCEPTION",
        )
        self.assertEqual(snapshot.safety.emergency_state, "SUSPECTED")
        self.assertEqual(snapshot.safety.current_ownership, "ACTIVE_PERCEPTION")

    def test_mission_summary_updates(self) -> None:
        snapshot = self.memory.update_summary(
            timestamp_monotonic=1.0,
            recovery_count=2,
            avoidance_count=4,
            observations_completed=1,
            mission_confidence=0.85,
            elapsed_mission_s=42.0,
        )
        self.assertEqual(snapshot.summary.avoidance_count, 4)
        self.assertEqual(snapshot.summary.mission_confidence, 0.85)

    def test_event_insertion(self) -> None:
        event = self.memory.record_event(
            1.0,
            "OBSERVATION",
            "Stable observation completed",
            {"waypoint_id": "rock-observation"},
        )
        self.assertEqual(event.sequence, 1)
        self.assertEqual(event.category, "OBSERVATION")
        self.assertEqual(self.memory.recent_events(), (event,))

    def test_event_capacity_discards_oldest(self) -> None:
        for index in range(5):
            self.memory.record_event(float(index), "MISSION", f"event-{index}")
        events = self.memory.recent_events()
        self.assertEqual(len(events), 3)
        self.assertEqual([event.message for event in events], ["event-2", "event-3", "event-4"])

    def test_event_ordering_is_chronological(self) -> None:
        first = self.memory.record_event(1.0, "MISSION", "first")
        second = self.memory.record_event(1.0, "SAFETY", "second")
        self.assertEqual(self.memory.recent_events(), (first, second))
        self.assertEqual((first.sequence, second.sequence), (1, 2))
        with self.assertRaises(ValueError):
            self.memory.record_event(0.9, "MISSION", "out-of-order")

    def test_snapshot_reflects_all_updates(self) -> None:
        self.memory.update_mission(timestamp_monotonic=1.0, mission_id="demo")
        self.memory.update_navigation(timestamp_monotonic=2.0, strategy="DIRECT")
        self.memory.update_perception(timestamp_monotonic=3.0, condition="DEGRADED")
        self.memory.update_safety(timestamp_monotonic=4.0, supervisor_state="SLOW")
        self.memory.update_summary(timestamp_monotonic=5.0, elapsed_mission_s=4.0)
        self.memory.record_event(6.0, "MISSION", "Context assembled")
        snapshot = self.memory.snapshot()
        self.assertEqual(snapshot.revision, 6)
        self.assertEqual(snapshot.updated_at_monotonic, 6.0)
        self.assertEqual(snapshot.mission.mission_id, "demo")
        self.assertEqual(snapshot.navigation.strategy, "DIRECT")
        self.assertEqual(snapshot.perception.condition, "DEGRADED")
        self.assertEqual(snapshot.safety.supervisor_state, "SLOW")
        self.assertEqual(snapshot.summary.elapsed_mission_s, 4.0)

    def test_reset_clears_state_events_and_sequences(self) -> None:
        self.memory.update_mission(timestamp_monotonic=1.0, mission_id="demo")
        self.memory.record_event(2.0, "MISSION", "Started")
        snapshot = self.memory.reset()
        self.assertEqual(snapshot.revision, 0)
        self.assertEqual(snapshot.updated_at_monotonic, 0.0)
        self.assertEqual(snapshot.mission.mission_id, "NONE")
        self.assertEqual(snapshot.recent_events, ())
        event = self.memory.record_event(3.0, "MISSION", "Restarted")
        self.assertEqual(event.sequence, 1)


if __name__ == "__main__":
    unittest.main()
