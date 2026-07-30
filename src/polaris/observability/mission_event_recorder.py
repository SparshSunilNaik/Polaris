from __future__ import annotations

import copy
import json
from datetime import datetime, timezone
from enum import Enum
from threading import Lock
from time import monotonic
from typing import Any, Callable


class MissionEventType(str, Enum):
    MISSION_RECORDER_STARTED = "MISSION_RECORDER_STARTED"
    PX4_CONNECTED = "PX4_CONNECTED"
    PX4_DISCONNECTED = "PX4_DISCONNECTED"
    MISSION_STARTED = "MISSION_STARTED"
    MISSION_READY = "MISSION_READY"
    ARM_REQUESTED = "ARM_REQUESTED"
    ARMED = "ARMED"
    TAKEOFF_REQUESTED = "TAKEOFF_REQUESTED"
    IN_AIR = "IN_AIR"
    OFFBOARD_START_REQUESTED = "OFFBOARD_START_REQUESTED"
    OFFBOARD_ENTERED = "OFFBOARD_ENTERED"
    OFFBOARD_EXITED = "OFFBOARD_EXITED"
    FLIGHT_MODE_CHANGED = "FLIGHT_MODE_CHANGED"
    WAYPOINT_STARTED = "WAYPOINT_STARTED"
    WAYPOINT_REACHED = "WAYPOINT_REACHED"
    WAYPOINT_FAILED = "WAYPOINT_FAILED"
    AVOIDANCE_ENTERED = "AVOIDANCE_ENTERED"
    AVOIDANCE_EXITED = "AVOIDANCE_EXITED"
    RECOVERY_STARTED = "RECOVERY_STARTED"
    RECOVERY_COMPLETED = "RECOVERY_COMPLETED"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    SAFETY_WARNING = "SAFETY_WARNING"
    SAFETY_HARD_OVERRIDE = "SAFETY_HARD_OVERRIDE"
    MISSION_TIMEOUT = "MISSION_TIMEOUT"
    MISSION_EXTERNALLY_INTERRUPTED = "MISSION_EXTERNALLY_INTERRUPTED"
    LAND_REQUESTED = "LAND_REQUESTED"
    LANDING_STARTED = "LANDING_STARTED"
    LANDED = "LANDED"
    DISARMED = "DISARMED"
    MISSION_COMPLETED = "MISSION_COMPLETED"
    MISSION_FAILED = "MISSION_FAILED"
    MISSION_ABORTED = "MISSION_ABORTED"
    SHUTDOWN_STARTED = "SHUTDOWN_STARTED"


class MissionEventRecorder:
    """Thread-safe, transition-only mission evidence recorder."""

    def __init__(
        self,
        run_id: str,
        *,
        monotonic_clock: Callable[[], float] = monotonic,
        utc_now: Callable[[], datetime] | None = None,
    ) -> None:
        self.run_id = run_id
        self._monotonic = monotonic_clock
        self._utc_now = utc_now or (lambda: datetime.now(timezone.utc))
        self._started_at = self._monotonic()
        self._events: list[dict[str, Any]] = []
        self._warnings: list[str] = []
        self._lock = Lock()
        self._finalized = False
        self._connected: bool | None = None
        self._armed: bool | None = None
        self._in_air: bool | None = None
        self._flight_mode: str | None = None
        self._collision_detected = False
        self.record(
            MissionEventType.MISSION_RECORDER_STARTED,
            "mission_event_recorder",
            f"Mission event recorder started for {run_id}",
            data={"run_id": run_id},
        )

    def record(
        self,
        event_type: MissionEventType | str,
        source: str,
        message: str,
        *,
        severity: str = "INFO",
        data: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not isinstance(data, dict) and data is not None:
            raise ValueError("Mission event data must be a JSON object")
        payload = {} if data is None else copy.deepcopy(data)
        try:
            json.dumps(payload, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Mission event data is not JSON serializable: {exc}") from exc
        label = event_type.value if isinstance(event_type, MissionEventType) else str(event_type)
        if not label or not label.replace("_", "").isupper():
            raise ValueError("Mission event type must be uppercase snake case")
        now = self._monotonic()
        timestamp = self._utc_now().astimezone(timezone.utc).isoformat().replace("+00:00", "Z")
        with self._lock:
            event = {
                "sequence": len(self._events) + 1,
                "timestamp_utc": timestamp,
                "elapsed_s": max(0.0, now - self._started_at),
                "event_type": label,
                "source": source,
                "severity": severity,
                "message": message,
                "data": payload,
            }
            self._events.append(event)
            return copy.deepcopy(event)

    def observe_connection(self, connected: bool) -> None:
        if self._connected is connected:
            return
        self._connected = connected
        event = MissionEventType.PX4_CONNECTED if connected else MissionEventType.PX4_DISCONNECTED
        self.record(event, "mavsdk_telemetry", "PX4 connected" if connected else "PX4 disconnected")

    def observe_telemetry(
        self,
        *,
        armed: bool,
        in_air: bool,
        flight_mode: str,
        mission_state: str,
        initiated_by_polaris: bool,
    ) -> None:
        if self._armed is None:
            self._armed = armed
        elif armed != self._armed:
            self._armed = armed
            self.record(
                MissionEventType.ARMED if armed else MissionEventType.DISARMED,
                "mavsdk_telemetry",
                "Vehicle armed" if armed else "Vehicle disarmed",
                data={"mission_state": mission_state},
            )
        if self._in_air is None:
            self._in_air = in_air
        elif in_air != self._in_air:
            self._in_air = in_air
            self.record(
                MissionEventType.IN_AIR if in_air else MissionEventType.LANDED,
                "mavsdk_telemetry",
                "Vehicle in air" if in_air else "Vehicle landed",
                data={"mission_state": mission_state},
            )
        mode = flight_mode or "UNKNOWN"
        if self._flight_mode is None:
            self._flight_mode = mode
            self.record(
                MissionEventType.FLIGHT_MODE_CHANGED,
                "mavsdk_telemetry",
                f"Initial flight mode {mode}",
                data={"previous_mode": None, "current_mode": mode, "mission_state": mission_state, "expected": True},
            )
            if mode == "OFFBOARD":
                self.record(MissionEventType.OFFBOARD_ENTERED, "mavsdk_telemetry", "Offboard mode confirmed")
            return
        if mode == self._flight_mode:
            return
        previous = self._flight_mode
        self._flight_mode = mode
        self.record(
            MissionEventType.FLIGHT_MODE_CHANGED,
            "mavsdk_telemetry",
            f"Flight mode changed {previous} -> {mode}",
            data={"previous_mode": previous, "current_mode": mode, "mission_state": mission_state, "expected": initiated_by_polaris},
        )
        if previous == "OFFBOARD" and mode != "OFFBOARD":
            self.record(
                MissionEventType.OFFBOARD_EXITED,
                "mavsdk_telemetry",
                f"Offboard mode exited to {mode}",
                data={"previous_mode": previous, "current_mode": mode, "mission_state": mission_state, "expected": initiated_by_polaris},
            )
            if not initiated_by_polaris:
                self.record(
                    MissionEventType.MISSION_EXTERNALLY_INTERRUPTED,
                    "mavsdk_telemetry",
                    f"Externally observed mode change from OFFBOARD to {mode}",
                    severity="WARNING",
                    data={"previous_mode": previous, "current_mode": mode, "initiated_by_polaris": False, "classification": "OBSERVED_EXTERNAL_MODE_CHANGE"},
                )
        elif previous != "OFFBOARD" and mode == "OFFBOARD":
            self.record(MissionEventType.OFFBOARD_ENTERED, "mavsdk_telemetry", "Offboard mode confirmed")

    def observe_collision(self, detected: bool, *, data: dict[str, Any]) -> None:
        if detected and not self._collision_detected:
            self.record(MissionEventType.COLLISION_DETECTED, "collision_monitor", "Collision detected", severity="ERROR", data=data)
        self._collision_detected = detected

    def warn(self, message: str) -> None:
        with self._lock:
            if len(self._warnings) < 32:
                self._warnings.append(message)

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._events)

    def warnings(self) -> list[str]:
        with self._lock:
            return list(self._warnings)

    def finalize(self) -> list[dict[str, Any]]:
        with self._lock:
            self._finalized = True
        return self.snapshot()
