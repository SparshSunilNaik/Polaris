from __future__ import annotations

import json
import sys
import unittest
from dataclasses import asdict
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from gz.msgs10.contacts_pb2 import Contacts  # noqa: E402

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    DesiredMotion,
    ReactiveAvoidanceController,
    SafetySupervisor,
    SupervisorMode,
)
from polaris.ego_state.confidence import ConfidenceEngine  # noqa: E402
from polaris.simulation import (  # noqa: E402
    CollisionMonitor,
    parse_contact_events,
)
from test_reactive_avoidance import CLEAR, navigation  # noqa: E402


def contacts(other: str) -> Contacts:
    message = Contacts()
    contact = message.contact.add()
    contact.collision1.name = "x500_mono_cam_0::base_link::base_link_collision_0"
    contact.collision2.name = other
    contact.position.add(x=1.0, y=2.0, z=3.0)
    contact.normal.add(x=0.0, y=0.0, z=1.0)
    contact.depth.append(0.01)
    return message


class CollisionMonitorTests(unittest.TestCase):
    def test_collision_event_parsing(self) -> None:
        events = parse_contact_events(
            contacts("physical_centred_obstacle::obstacle_link::obstacle_collision"),
            now_monotonic=10.0,
            ground_contact_allowed=False,
        )
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].contact_position, (1.0, 2.0, 3.0))
        self.assertFalse(events[0].expected_contact)

    def test_expected_ground_contact_is_not_collision(self) -> None:
        events = parse_contact_events(
            contacts("snow_ground::ground_link::ground_collision"),
            now_monotonic=10.0,
            ground_contact_allowed=True,
        )
        self.assertTrue(events[0].expected_contact)

    def test_fast_ground_contact_is_not_expected_touchdown(self) -> None:
        events = parse_contact_events(
            contacts("snow_ground::ground_link::ground_collision"),
            now_monotonic=10.0,
            ground_contact_allowed=True,
            vertical_speed_m_s=2.0,
        )
        self.assertFalse(events[0].expected_contact)

    def test_ground_contact_without_depth_fails_closed(self) -> None:
        message = contacts("snow_ground::ground_link::ground_collision")
        del message.contact[0].depth[:]
        events = parse_contact_events(
            message,
            now_monotonic=10.0,
            ground_contact_allowed=True,
        )
        self.assertFalse(events[0].expected_contact)

    def test_expected_contact_can_escalate_inside_cooldown(self) -> None:
        monitor = CollisionMonitor(event_cooldown_s=10.0)
        monitor.set_ground_contact_allowed(True)
        monitor.update_vertical_speed(0.1)
        monitor._on_contacts(contacts("snow_ground::ground_link::ground_collision"))
        self.assertFalse(monitor.latest_state().collision_detected)
        monitor.update_vertical_speed(2.0)
        monitor._on_contacts(contacts("snow_ground::ground_link::ground_collision"))
        self.assertTrue(monitor.latest_state().collision_detected)

    def test_non_drone_contact_is_ignored(self) -> None:
        message = Contacts()
        contact = message.contact.add()
        contact.collision1.name = "rock::a"
        contact.collision2.name = "ground::b"
        self.assertEqual(
            parse_contact_events(
                message,
                now_monotonic=10.0,
                ground_contact_allowed=False,
            ),
            (),
        )

    def test_collision_hard_override(self) -> None:
        now = monotonic()
        state = navigation(CLEAR, 1)
        state.armed = True
        state.in_air = True
        state.flight_mode = "OFFBOARD"
        state.depth_required_for_motion = True
        command = DesiredMotion(forward_m_s=0.3, timestamp_monotonic=now)
        decision = ReactiveAvoidanceController(AvoidanceConfig()).update(
            command,
            state,
            "NORMAL",
            now,
        )
        safe = SafetySupervisor(AvoidanceConfig()).evaluate(
            decision,
            state,
            ConfidenceEngine().evaluate(state),
            px4_connected=True,
            autonomy_enabled=True,
            offboard_expected=True,
            displacement_from_start_m=0.0,
            collision_detected=True,
            now_monotonic=now,
        )
        self.assertTrue(safe.hard_override)
        self.assertEqual(safe.supervisor_mode, SupervisorMode.ABORT)
        self.assertIn("Airborne collision detected", safe.reasons)

    def test_injected_collision_serializes(self) -> None:
        monitor = CollisionMonitor()
        event = monitor.inject_collision("fixture", 10.0)
        state = monitor.latest_state()
        payload = json.dumps(asdict(event), allow_nan=False)
        self.assertTrue(state.collision_detected)
        self.assertEqual(state.collision_count, 1)
        self.assertTrue(state.injected_collision_detected)
        self.assertFalse(state.native_collision_detected)
        self.assertIn("fixture", payload)


if __name__ == "__main__":
    unittest.main()
