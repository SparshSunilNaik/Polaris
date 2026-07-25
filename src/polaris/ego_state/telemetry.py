from __future__ import annotations

import asyncio
from contextlib import suppress
from time import monotonic

from mavsdk import System

from .models import NavigationState, Orientation, Vector3


def normalize_battery_remaining(value: float) -> float:
    """Normalize MAVSDK backends that report either 0-1 or 0-100."""
    normalized = value / 100.0 if value > 1.0 else value
    return max(0.0, min(1.0, normalized))


class TelemetryCollector:
    def __init__(self, drone: System, state: NavigationState | None = None) -> None:
        self.drone = drone
        self.state = state or NavigationState()
        self._tasks: list[asyncio.Task[None]] = []

    async def start(self) -> None:
        if self._tasks:
            return

        self._tasks = [
            asyncio.create_task(self._watch_position_velocity()),
            asyncio.create_task(self._watch_attitude()),
            asyncio.create_task(self._watch_health()),
            asyncio.create_task(self._watch_battery()),
            asyncio.create_task(self._watch_armed()),
            asyncio.create_task(self._watch_in_air()),
            asyncio.create_task(self._watch_flight_mode()),
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()

        for task in self._tasks:
            with suppress(asyncio.CancelledError):
                await task

        self._tasks.clear()

    async def _watch_position_velocity(self) -> None:
        async for sample in self.drone.telemetry.position_velocity_ned():
            position = sample.position
            velocity = sample.velocity

            self.state.position_ned_m = Vector3(
                x=position.north_m,
                y=position.east_m,
                z=position.down_m,
            )
            self.state.velocity_ned_m_s = Vector3(
                x=velocity.north_m_s,
                y=velocity.east_m_s,
                z=velocity.down_m_s,
            )
            self.state.last_position_update_monotonic = monotonic()
            self.state.touch()

    async def _watch_attitude(self) -> None:
        async for attitude in self.drone.telemetry.attitude_euler():
            self.state.orientation = Orientation(
                roll_deg=attitude.roll_deg,
                pitch_deg=attitude.pitch_deg,
                yaw_deg=attitude.yaw_deg,
            )
            self.state.last_attitude_update_monotonic = monotonic()
            self.state.touch()

    async def _watch_health(self) -> None:
        async for health in self.drone.telemetry.health():
            self.state.local_position_ok = health.is_local_position_ok
            self.state.global_position_ok = health.is_global_position_ok
            self.state.home_position_ok = health.is_home_position_ok
            self.state.armable = health.is_armable
            self.state.touch()

    async def _watch_battery(self) -> None:
        async for battery in self.drone.telemetry.battery():
            self.state.battery_remaining = normalize_battery_remaining(
                battery.remaining_percent
            )
            self.state.battery_voltage_v = battery.voltage_v
            self.state.touch()

    async def _watch_armed(self) -> None:
        async for armed in self.drone.telemetry.armed():
            self.state.armed = armed
            self.state.last_armed_update_monotonic = monotonic()
            self.state.touch()

    async def _watch_in_air(self) -> None:
        async for in_air in self.drone.telemetry.in_air():
            self.state.in_air = in_air
            self.state.last_in_air_update_monotonic = monotonic()
            self.state.touch()

    async def _watch_flight_mode(self) -> None:
        async for mode in self.drone.telemetry.flight_mode():
            self.state.flight_mode = mode.name
            self.state.last_flight_mode_update_monotonic = monotonic()
            self.state.touch()
