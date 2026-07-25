from __future__ import annotations

import math
from collections import deque
from dataclasses import replace
from threading import RLock
from time import monotonic
from typing import Any, Mapping

from .models import (
    EgoContextEvent,
    EgoContextSnapshot,
    MissionContext,
    MissionSummary,
    NavigationContext,
    PerceptionContext,
    SafetyContext,
    freeze_metadata,
)


class EgoContextMemory:
    """Thread-safe semantic mission memory with immutable read snapshots."""

    DEFAULT_EVENT_CAPACITY = 256

    def __init__(self, event_capacity: int = DEFAULT_EVENT_CAPACITY) -> None:
        if event_capacity < 1:
            raise ValueError("event_capacity must be positive")
        self._event_capacity = event_capacity
        self._lock = RLock()
        self._events: deque[EgoContextEvent] = deque(maxlen=event_capacity)
        self._reset_unlocked()

    @property
    def event_capacity(self) -> int:
        return self._event_capacity

    def snapshot(self) -> EgoContextSnapshot:
        with self._lock:
            return self._snapshot_unlocked()

    def recent_events(self) -> tuple[EgoContextEvent, ...]:
        with self._lock:
            return tuple(self._events)

    def update_mission(
        self,
        *,
        timestamp_monotonic: float | None = None,
        **changes: Any,
    ) -> EgoContextSnapshot:
        return self._update("_mission", timestamp_monotonic, changes)

    def update_navigation(
        self,
        *,
        timestamp_monotonic: float | None = None,
        **changes: Any,
    ) -> EgoContextSnapshot:
        return self._update("_navigation", timestamp_monotonic, changes)

    def update_perception(
        self,
        *,
        timestamp_monotonic: float | None = None,
        **changes: Any,
    ) -> EgoContextSnapshot:
        return self._update("_perception", timestamp_monotonic, changes)

    def update_safety(
        self,
        *,
        timestamp_monotonic: float | None = None,
        **changes: Any,
    ) -> EgoContextSnapshot:
        return self._update("_safety", timestamp_monotonic, changes)

    def update_summary(
        self,
        *,
        timestamp_monotonic: float | None = None,
        **changes: Any,
    ) -> EgoContextSnapshot:
        return self._update("_summary", timestamp_monotonic, changes)

    def record_event(
        self,
        timestamp_monotonic: float,
        category: str,
        message: str,
        metadata: Mapping[str, Any] | None = None,
    ) -> EgoContextEvent:
        with self._lock:
            timestamp = self._timestamp(timestamp_monotonic)
            if self._events and timestamp < self._events[-1].timestamp_monotonic:
                raise ValueError("Ego-context events must be inserted chronologically")
            if not category or not message:
                raise ValueError("Event category and message must not be empty")
            self._event_sequence += 1
            event = EgoContextEvent(
                sequence=self._event_sequence,
                timestamp_monotonic=timestamp,
                category=category,
                message=message,
                metadata=freeze_metadata(metadata),
            )
            self._events.append(event)
            self._touch(timestamp)
            return event

    def reset(self) -> EgoContextSnapshot:
        with self._lock:
            self._reset_unlocked()
            return self._snapshot_unlocked()

    def _update(
        self,
        attribute: str,
        timestamp_monotonic: float | None,
        changes: dict[str, Any],
    ) -> EgoContextSnapshot:
        with self._lock:
            timestamp = self._timestamp(timestamp_monotonic)
            current = getattr(self, attribute)
            setattr(self, attribute, replace(current, **changes))
            self._touch(timestamp)
            return self._snapshot_unlocked()

    def _timestamp(self, value: float | None) -> float:
        timestamp = monotonic() if value is None else float(value)
        if not math.isfinite(timestamp) or timestamp < 0:
            raise ValueError("Context timestamps must be finite and non-negative")
        if timestamp < self._updated_at_monotonic:
            raise ValueError("Context updates must be chronological")
        return timestamp

    def _touch(self, timestamp: float) -> None:
        self._revision += 1
        self._updated_at_monotonic = timestamp

    def _snapshot_unlocked(self) -> EgoContextSnapshot:
        return EgoContextSnapshot(
            revision=self._revision,
            updated_at_monotonic=self._updated_at_monotonic,
            mission=self._mission,
            navigation=self._navigation,
            perception=self._perception,
            safety=self._safety,
            summary=self._summary,
            recent_events=tuple(self._events),
        )

    def _reset_unlocked(self) -> None:
        self._mission = MissionContext()
        self._navigation = NavigationContext()
        self._perception = PerceptionContext()
        self._safety = SafetyContext()
        self._summary = MissionSummary()
        self._events.clear()
        self._event_sequence = 0
        self._revision = 0
        self._updated_at_monotonic = 0.0
