from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, replace
from threading import Lock
from time import monotonic

from polaris.perception.dependencies import add_homebrew_python_site

add_homebrew_python_site()

from gz.msgs10.contacts_pb2 import Contacts  # noqa: E402
from gz.transport13 import Node  # noqa: E402


CONTACT_TOPIC = "/polaris/world/contacts"
DRONE_COLLISION_PATTERN = re.compile(r"(?:^|::)x500_mono_cam_[0-9]+::")


@dataclass(frozen=True)
class CollisionEvent:
    timestamp_monotonic: float
    drone_collision: str
    contacted_entity: str
    collision_pair: tuple[str, str]
    contact_position: tuple[float, float, float] | None
    contact_normal: tuple[float, float, float] | None
    maximum_penetration_depth: float | None
    ground_contact: bool
    expected_contact: bool
    injected: bool = False


@dataclass(frozen=True)
class CollisionState:
    available: bool = False
    collision_detected: bool = False
    collision_count: int = 0
    native_collision_detected: bool = False
    injected_collision_detected: bool = False
    latest_collision_timestamp_monotonic: float = 0.0
    latest_contacted_entity: str = "NONE"
    latest_drone_collision: str = "NONE"
    contact_position: tuple[float, float, float] | None = None
    contact_normal: tuple[float, float, float] | None = None
    expected_ground_contact_count: int = 0
    last_message_timestamp_monotonic: float = 0.0
    reasons: tuple[str, ...] = ()
    events: tuple[CollisionEvent, ...] = ()


def parse_contact_events(
    message: Contacts,
    *,
    now_monotonic: float,
    ground_contact_allowed: bool,
    vertical_speed_m_s: float = 0.0,
    maximum_touchdown_speed_m_s: float = 1.0,
    maximum_touchdown_depth_m: float = 0.01,
) -> tuple[CollisionEvent, ...]:
    events: list[CollisionEvent] = []
    for contact in message.contact:
        first = contact.collision1.name
        second = contact.collision2.name
        first_is_drone = DRONE_COLLISION_PATTERN.search(first) is not None
        second_is_drone = DRONE_COLLISION_PATTERN.search(second) is not None
        if first_is_drone == second_is_drone:
            continue
        drone_collision = first if first_is_drone else second
        contacted_entity = second if first_is_drone else first
        ground = "snow_ground::" in contacted_entity
        position = None
        normal = None
        if contact.position:
            point = contact.position[0]
            position = (point.x, point.y, point.z)
        if contact.normal:
            vector = contact.normal[0]
            normal = (vector.x, vector.y, vector.z)
        depth = max(contact.depth) if contact.depth else None
        expected_ground = (
            ground
            and ground_contact_allowed
            and abs(vertical_speed_m_s) <= maximum_touchdown_speed_m_s
            and depth is not None
            and depth <= maximum_touchdown_depth_m
        )
        events.append(
            CollisionEvent(
                timestamp_monotonic=now_monotonic,
                drone_collision=drone_collision,
                contacted_entity=contacted_entity,
                collision_pair=(first, second),
                contact_position=position,
                contact_normal=normal,
                maximum_penetration_depth=depth,
                ground_contact=ground,
                expected_contact=expected_ground,
            )
        )
    return tuple(events)


class CollisionMonitor:
    def __init__(
        self,
        topic: str = CONTACT_TOPIC,
        event_cooldown_s: float = 0.5,
        maximum_events: int = 64,
        maximum_touchdown_speed_m_s: float = 1.0,
        maximum_touchdown_depth_m: float = 0.01,
    ) -> None:
        self.topic = topic
        self.event_cooldown_s = event_cooldown_s
        self.maximum_touchdown_speed_m_s = maximum_touchdown_speed_m_s
        self.maximum_touchdown_depth_m = maximum_touchdown_depth_m
        self._node = Node()
        self._lock = Lock()
        self._state = CollisionState()
        self._events: deque[CollisionEvent] = deque(maxlen=maximum_events)
        self._last_pair_at: dict[tuple[str, str], float] = {}
        self._last_pair_expected: dict[tuple[str, str], bool] = {}
        self._ground_contact_allowed = True
        self._vertical_speed_m_s = 0.0
        self._started = False
        self._last_availability_check = 0.0

    def start(self) -> None:
        if self._started:
            return
        self._node.subscribe(Contacts, self.topic, self._on_contacts)
        self._started = True
        self._refresh_available(force=True)

    def close(self) -> None:
        if self._started:
            self._node.unsubscribe(self.topic)
        self._started = False

    def set_ground_contact_allowed(self, allowed: bool) -> None:
        with self._lock:
            self._ground_contact_allowed = allowed

    def update_vertical_speed(self, down_m_s: float) -> None:
        with self._lock:
            self._vertical_speed_m_s = down_m_s

    def clear_for_run(self) -> None:
        with self._lock:
            self._events.clear()
            self._last_pair_at.clear()
            self._last_pair_expected.clear()
            self._state = CollisionState(available=self._state.available)
            self._ground_contact_allowed = True

    def latest_state(self) -> CollisionState:
        self._refresh_available()
        with self._lock:
            return replace(self._state, events=tuple(self._events))

    def inject_collision(
        self,
        contacted_entity: str = "synthetic_collision_fixture",
        now_monotonic: float | None = None,
    ) -> CollisionEvent:
        now = monotonic() if now_monotonic is None else now_monotonic
        event = CollisionEvent(
            timestamp_monotonic=now,
            drone_collision="x500_mono_cam_0::injected_collision",
            contacted_entity=contacted_entity,
            collision_pair=("x500_mono_cam_0::injected_collision", contacted_entity),
            contact_position=None,
            contact_normal=None,
            maximum_penetration_depth=None,
            ground_contact=False,
            expected_contact=False,
            injected=True,
        )
        with self._lock:
            self._record(event)
        return event

    def _on_contacts(self, message: Contacts) -> None:
        now = monotonic()
        with self._lock:
            allowed = self._ground_contact_allowed
            vertical_speed = self._vertical_speed_m_s
        events = parse_contact_events(
            message,
            now_monotonic=now,
            ground_contact_allowed=allowed,
            vertical_speed_m_s=vertical_speed,
            maximum_touchdown_speed_m_s=self.maximum_touchdown_speed_m_s,
            maximum_touchdown_depth_m=self.maximum_touchdown_depth_m,
        )
        with self._lock:
            self._state = replace(
                self._state,
                available=True,
                last_message_timestamp_monotonic=now,
            )
            for event in events:
                pair = tuple(sorted(event.collision_pair))
                previous = self._last_pair_at.get(pair, -float("inf"))
                previous_expected = self._last_pair_expected.get(pair, False)
                escalation = previous_expected and not event.expected_contact
                if now - previous < self.event_cooldown_s and not escalation:
                    continue
                self._last_pair_at[pair] = now
                self._last_pair_expected[pair] = event.expected_contact
                self._record(event)

    def _record(self, event: CollisionEvent) -> None:
        self._events.append(event)
        if event.expected_contact:
            self._state = replace(
                self._state,
                expected_ground_contact_count=(
                    self._state.expected_ground_contact_count + 1
                ),
                events=tuple(self._events),
            )
            return
        self._state = CollisionState(
            available=True,
            collision_detected=True,
            collision_count=self._state.collision_count + 1,
            native_collision_detected=(
                self._state.native_collision_detected or not event.injected
            ),
            injected_collision_detected=(
                self._state.injected_collision_detected or event.injected
            ),
            latest_collision_timestamp_monotonic=event.timestamp_monotonic,
            latest_contacted_entity=event.contacted_entity,
            latest_drone_collision=event.drone_collision,
            contact_position=event.contact_position,
            contact_normal=event.contact_normal,
            expected_ground_contact_count=self._state.expected_ground_contact_count,
            last_message_timestamp_monotonic=event.timestamp_monotonic,
            reasons=(f"Contact with {event.contacted_entity}",),
            events=tuple(self._events),
        )

    def _refresh_available(self, force: bool = False) -> None:
        if not self._started:
            return
        now = monotonic()
        if not force and now - self._last_availability_check < 1.0:
            return
        self._last_availability_check = now
        available = self.topic in self._node.topic_list()
        with self._lock:
            if available != self._state.available:
                self._state = replace(self._state, available=available)
