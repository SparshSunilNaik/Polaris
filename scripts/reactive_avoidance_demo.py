#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import copy
import json
import math
import os
import sys
from contextlib import suppress
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    AvoidanceDecision,
    AvoidanceMode,
    DesiredMotion,
    OwnedMavsdkSystem,
    ReactiveAvoidanceController,
    SafetySupervisor,
    write_avoidance_artifact,
)
from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.ego_state.models import NavigationState, VisualState  # noqa: E402
from polaris.operator_view import (  # noqa: E402
    WINDOW_TITLE,
    OperatorRenderer,
    OperatorViewConfig,
    is_quit_key,
    screenshot_path,
)
from polaris.perception.dependencies import add_homebrew_python_site  # noqa: E402
from polaris.perception.depth import DepthConfig, DepthState  # noqa: E402
from polaris.perception.depth.models import DepthCondition  # noqa: E402
from polaris.perception.models import VisualCondition  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402

add_homebrew_python_site()
import cv2  # noqa: E402


SCENARIOS = (
    "corridor",
    "centre-obstacle",
    "hover-only",
    "stale-depth-test",
    "low-visual-health-test",
    "synthetic-grid-test",
)


class DemoAbort(RuntimeError):
    pass


class ConnectionMonitor:
    def __init__(self, drone: System) -> None:
        self.drone = drone
        self.connected = False
        self._event = asyncio.Event()
        self._task: asyncio.Task[None] | None = None

    def start(self) -> None:
        self._task = asyncio.create_task(self._watch())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def wait(self, timeout_s: float = 20.0) -> None:
        await asyncio.wait_for(self._event.wait(), timeout_s)

    async def _watch(self) -> None:
        async for state in self.drone.core.connection_state():
            self.connected = state.is_connected
            if self.connected:
                self._event.set()


class IntegratedDisplay:
    def __init__(self, enabled: bool, config: OperatorViewConfig) -> None:
        self.enabled = enabled
        self.config = config
        self.renderer = OperatorRenderer(config)
        self.started = False
        self.stop_requested = False
        self.grid_overlay = config.rgb_grid_overlay
        self.canvas = None

    def start(self) -> None:
        if not self.enabled or self.started:
            return
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_TITLE, self.config.window_width, self.config.window_height)
        self.started = True

    async def render(
        self,
        coordinator: EgoStateCoordinator,
        state: NavigationState,
        report,
        proximity,
        decision: AvoidanceDecision | None,
        navigation_decision=None,
        recovery_decision=None,
        collision_state=None,
        mission_state=None,
        observation_decision=None,
        ego_context_snapshot=None,
    ) -> None:
        if not self.enabled:
            return
        self.start()
        frame = coordinator.camera.latest_frame()
        self.canvas = await asyncio.to_thread(
            self.renderer.compose,
            state,
            report,
            frame,
            proximity,
            display_fps=0.0,
            depth_enabled=True,
            grid_overlay=self.grid_overlay,
            paused=False,
            help_visible=False,
            avoidance=decision,
            navigation=navigation_decision,
            recovery=recovery_decision,
            collision=collision_state,
            mission=mission_state,
            observation=observation_decision,
            ego_context=ego_context_snapshot,
        )
        cv2.imshow(WINDOW_TITLE, self.canvas)
        key = cv2.waitKey(1) & 0xFF
        if is_quit_key(key):
            self.stop_requested = True
        elif key == ord("g"):
            self.grid_overlay = not self.grid_overlay
        elif key == ord("s") and self.canvas is not None:
            directory = PROJECT_ROOT / "artifacts/operator_screenshots"
            directory.mkdir(parents=True, exist_ok=True)
            path = screenshot_path(directory)
            if await asyncio.to_thread(cv2.imwrite, str(path), self.canvas):
                print(f"Screenshot saved: {path}")
        try:
            if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                self.stop_requested = True
        except cv2.error:
            self.stop_requested = True

    def close(self) -> None:
        if self.started:
            cv2.destroyAllWindows()
            self.started = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run deterministic reactive avoidance")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/reactive_avoidance.json"))
    parser.add_argument("--duration", type=float)
    parser.add_argument("--forward-speed", type=float)
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-depth-fps", type=float)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-flight", action="store_true")
    parser.add_argument("--scenario", choices=SCENARIOS, default="corridor")
    parser.add_argument("--start-delay", type=float)
    parser.add_argument("--auto-land", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--json-log", action="store_true")
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--display", dest="display", action="store_true")
    display.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=True)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "artifacts/avoidance_runs"))
    parser.add_argument("--system-address", default="udp://:14540")
    return parser.parse_args()


def snapshot(coordinator: EgoStateCoordinator):
    state = copy.deepcopy(coordinator.state)
    proximity = None
    if coordinator.depth_worker is not None:
        depth, proximity = coordinator.depth_worker.latest_snapshot()
        state.depth = depth
    return state, proximity


def command_record(
    decision: AvoidanceDecision,
    state: NavigationState,
    loop_elapsed_ms: float,
    displacement_m: float,
    transmitted: bool,
) -> dict:
    supervised = None
    if decision.safe_command is not None:
        supervised = asdict(decision.safe_command)
        supervised["supervisor_mode"] = decision.safe_command.supervisor_mode.value
        supervised["recommended_action"] = decision.safe_command.recommended_action.value
    return {
        "timestamp_monotonic": decision.timestamp_monotonic,
        "loop_elapsed_ms": loop_elapsed_ms,
        "mode": decision.mode.value,
        "chosen_direction": decision.chosen_direction.value,
        "blocked_count": decision.blocked_count,
        "clear_count": decision.clear_count,
        "reversal_count": decision.reversal_count,
        "risk": asdict(decision.risk),
        "risk_grid": [list(row) for row in state.depth.risk_grid],
        "requested": asdict(decision.requested_command),
        "proposed": asdict(decision.proposed_command),
        "supervised": supervised,
        "issued": supervised if transmitted else None,
        "command_transmitted": transmitted,
        "visual_health": state.visual.visual_health_score,
        "visual_condition": state.visual.visual_condition.value,
        "depth_health": state.depth.depth_health_score,
        "depth_condition": state.depth.condition.value,
        "depth_stability": state.depth.temporal_stability,
        "confidence_mode": decision.confidence_mode,
        "position_ned_m": asdict(state.position_ned_m),
        "velocity_ned_m_s": asdict(state.velocity_ned_m_s),
        "displacement_from_start_m": displacement_m,
        "horizontal_speed_m_s": state.horizontal_speed_m_s,
        "flight_mode": state.flight_mode,
        "reasons": list(decision.reasons),
    }


def synthetic_state(grid, sequence: int, now: float) -> NavigationState:
    return NavigationState(
        visual=VisualState(
            camera_connected=True,
            frame_age_ms=20.0,
            visual_health_score=0.9,
            visual_condition=VisualCondition.HEALTHY,
            stale=False,
            reasons=(),
            last_visual_update_monotonic=now,
        ),
        depth=DepthState(
            available=True,
            stale=False,
            source_frame_sequence=sequence,
            inference_timestamp_monotonic=now,
            depth_health_score=0.9,
            condition=DepthCondition.HEALTHY,
            temporal_stability=0.95,
            risk_grid=grid,
        ),
        local_position_ok=True,
        last_position_update_monotonic=now,
        last_attitude_update_monotonic=now,
    )


def run_synthetic(config: AvoidanceConfig, args: argparse.Namespace) -> dict:
    controller = ReactiveAvoidanceController(config)
    clear = ((0.1, 0.1, 0.1), (0.1, 0.2, 0.1), (0.95, 0.95, 0.95))
    caution = ((0.1, 0.1, 0.1), (0.1, 0.52, 0.1), (0.95, 0.95, 0.95))
    centre = ((0.1, 0.1, 0.8), (0.2, 0.72, 0.8), (0.95, 0.95, 0.95))
    blocked = ((0.8, 0.1, 0.8), (0.8, 0.75, 0.8), (0.95, 0.95, 0.95))
    grids = [clear, caution, centre, centre, clear, clear, clear, blocked, blocked, blocked]
    records = []
    transitions = []
    previous = None
    for index, grid in enumerate(grids, start=1):
        now = float(index)
        state = synthetic_state(grid, index, now)
        requested = DesiredMotion(config.forward_speed_m_s, source="synthetic", timestamp_monotonic=now)
        decision = controller.update(requested, state, "NORMAL", now)
        if decision.mode.value != previous:
            transitions.append({"sequence": index, "mode": decision.mode.value})
            previous = decision.mode.value
        records.append(decision.to_dict())
        print(
            f"{index:02d} {decision.mode.value:12s} centre={decision.risk.centre:.2f} "
            f"left={decision.risk.left:.2f} right={decision.risk.right:.2f} "
            f"F={decision.proposed_command.forward_m_s:+.2f} "
            f"R={decision.proposed_command.right_m_s:+.2f}"
        )
    return {
        "scenario": "synthetic-grid-test",
        "configuration": asdict(config),
        "records": records,
        "transitions": transitions,
        "result": "COMPLETED",
        "landing_result": "NOT_APPLICABLE",
    }


class ReactiveAvoidanceDemo:
    def __init__(
        self,
        drone: System,
        coordinator: EgoStateCoordinator,
        connection: ConnectionMonitor,
        config: AvoidanceConfig,
        confidence: ConfidenceEngine,
        display: IntegratedDisplay,
        args: argparse.Namespace,
    ) -> None:
        self.drone = drone
        self.coordinator = coordinator
        self.connection = connection
        self.config = config
        self.confidence = confidence
        self.display = display
        self.args = args
        self.controller = ReactiveAvoidanceController(config)
        self.supervisor = SafetySupervisor(config)
        self.offboard_started = False
        self.origin_north_m = 0.0
        self.origin_east_m = 0.0
        self.origin_down_m = 0.0
        self.records: list[dict] = []
        self.transitions: list[dict] = []
        self.confidence_transitions: list[dict] = []
        self.abort_reason: str | None = None
        self.landing_result = "NOT_ATTEMPTED"
        self.max_displacement_m = 0.0
        self.max_speed_m_s = 0.0
        self.max_risk = {"centre": 0.0, "left": 0.0, "right": 0.0}
        self.forward_reduced = False
        self.avoidance_issued = False
        self.previous_mode: str | None = None
        self.loop_deadline_misses = 0
        self.takeoff_completed = False
        self.arming_attempted = False
        self.arm_accepted = False
        self.transmission_count = 0

    async def run(self) -> None:
        self.supervisor.reset()
        await self._wait_ready()
        if self.args.no_flight:
            await self._observe_without_flight()
            return
        try:
            await self._takeoff()
            if self.args.scenario == "hover-only":
                await self._hover_only()
            else:
                await self._control_loop()
        finally:
            await self._shielded_hover_and_land()

    async def _shielded_hover_and_land(self) -> None:
        landing = asyncio.create_task(self._hover_and_land())
        try:
            await asyncio.shield(landing)
        except asyncio.CancelledError:
            # asyncio.run cancels the main task on Ctrl+C. Keep the coordinator
            # alive until the zero-velocity hover and landing sequence finishes.
            await landing
            raise

    async def _wait_ready(self, timeout_s: float = 90.0) -> None:
        print("Waiting for PX4, RGB, localization, and persistent healthy depth...")
        deadline = monotonic() + timeout_s
        samples = 0
        last_sequence = 0
        while monotonic() < deadline:
            state, proximity = snapshot(self.coordinator)
            await self._takeoff_safety_hook(state)
            report = self.confidence.evaluate(state)
            await self.display.render(self.coordinator, state, report, proximity, None)
            if self.display.stop_requested:
                raise DemoAbort("Operator requested stop during readiness")
            ready = (
                self.connection.connected
                and state.local_position_ok
                and state.armable
                and state.visual.camera_connected
                and not state.visual.stale
                and state.depth.available
                and not state.depth.stale
                and state.depth.condition is not DepthCondition.CRITICAL
                and state.depth.depth_health_score >= self.config.minimum_depth_health
                and state.depth.temporal_stability >= self.config.minimum_temporal_stability
            )
            if ready and state.depth.source_frame_sequence != last_sequence:
                samples += 1
                last_sequence = state.depth.source_frame_sequence
            elif not ready:
                samples = 0
            if samples >= self.config.readiness_depth_samples:
                print(f"Ready with {samples} consecutive healthy depth updates.")
                return
            await asyncio.sleep(0.05)
        raise DemoAbort("Readiness timeout")

    async def _takeoff(self) -> None:
        state, _ = snapshot(self.coordinator)
        self.origin_north_m = state.position_ned_m.x
        self.origin_east_m = state.position_ned_m.y
        self.origin_down_m = state.position_ned_m.z
        target_down = self.origin_down_m - self.config.takeoff_altitude_m
        yaw = state.orientation.yaw_deg
        print(f"Arming and taking off to {self.config.takeoff_altitude_m:.1f} m in fixed-position Offboard...")
        self.arming_attempted = True
        await self.drone.action.arm()
        self.arm_accepted = True
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.origin_north_m, self.origin_east_m, target_down, yaw)
        )
        await self.drone.offboard.start()
        self.offboard_started = True
        deadline = monotonic() + 30.0
        while monotonic() < deadline:
            state, proximity = snapshot(self.coordinator)
            await self._takeoff_safety_hook(state)
            report = self.confidence.evaluate(state)
            await self.display.render(self.coordinator, state, report, proximity, None)
            if report.mode.name == "EMERGENCY":
                raise DemoAbort("Confidence Engine became EMERGENCY during takeoff")
            if self.display.stop_requested:
                raise DemoAbort("Operator requested stop during takeoff")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(self.origin_north_m, self.origin_east_m, target_down, yaw)
            )
            if state.in_air and state.altitude_m >= self.config.takeoff_altitude_m - 0.35:
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
                await self._wait_offboard_mode()
                self.takeoff_completed = True
                print("Takeoff and Offboard hover confirmed.")
                return
            await asyncio.sleep(0.1)
        raise DemoAbort("Takeoff altitude timeout")

    async def _takeoff_safety_hook(self, state: NavigationState) -> None:
        return

    async def _wait_offboard_mode(self) -> None:
        deadline = monotonic() + 5.0
        while monotonic() < deadline:
            if self.coordinator.state.flight_mode == "OFFBOARD":
                self.supervisor.reset_control_timer()
                return
            await asyncio.sleep(0.05)
        raise DemoAbort("PX4 did not report OFFBOARD mode")

    async def _hover_only(self) -> None:
        deadline = monotonic() + min(5.0, self._duration())
        while monotonic() < deadline:
            await self._send_hover()
            await asyncio.sleep(0.1)

    async def _observe_without_flight(self) -> None:
        print("No-flight mode: evaluating supervisor with autonomy disabled.")
        deadline = monotonic() + min(self._duration(), 10.0)
        while monotonic() < deadline:
            state, proximity = snapshot(self.coordinator)
            report = self.confidence.evaluate(state)
            now = monotonic()
            requested = DesiredMotion(self.config.forward_speed_m_s, source="no-flight", timestamp_monotonic=now)
            decision = self.controller.update(requested, state, report.mode.name, now)
            safe = self.supervisor.evaluate(
                decision,
                state,
                report,
                px4_connected=self.connection.connected,
                autonomy_enabled=False,
                offboard_expected=False,
                displacement_from_start_m=0.0,
                now_monotonic=now,
            )
            decision = replace(decision, safe_command=safe, autonomy_enabled=False)
            await self.display.render(self.coordinator, state, report, proximity, decision)
            self._record(decision, state, 0.0, 0.0, transmitted=False)
            if self.display.stop_requested:
                break
            await asyncio.sleep(1.0 / self.config.control_loop_hz)

    async def _control_loop(self) -> None:
        await self._delay_with_hover(self._start_delay())
        print(
            f"Reactive motion enabled: forward request {self.config.forward_speed_m_s:.2f} m/s, "
            f"maximum duration {self._duration():.1f} s"
        )
        started = monotonic()
        next_tick = started
        while monotonic() - started < self._duration():
            loop_started = monotonic()
            state, proximity = snapshot(self.coordinator)
            if self.args.scenario == "stale-depth-test" and loop_started - started >= 2.0:
                state.depth = replace(
                    state.depth,
                    available=False,
                    stale=True,
                    condition=DepthCondition.CRITICAL,
                    reasons=("Injected stale depth for safety validation",),
                )
            elif (
                self.args.scenario == "low-visual-health-test"
                and loop_started - started >= 2.0
            ):
                state.visual = replace(
                    state.visual,
                    visual_health_score=max(
                        0.0,
                        self.config.minimum_visual_health - 0.05,
                    ),
                    visual_condition=VisualCondition.DEGRADED,
                    reasons=("Injected low visual health for policy validation",),
                )
            report = self.confidence.evaluate(state)
            drift = self._drift(state)
            requested = DesiredMotion(
                self.config.forward_speed_m_s,
                source=self.args.scenario,
                timestamp_monotonic=loop_started,
            )
            decision = self.controller.update(requested, state, report.mode.name, loop_started)
            safe = self.supervisor.evaluate(
                decision,
                state,
                report,
                px4_connected=self.connection.connected,
                autonomy_enabled=True,
                offboard_expected=True,
                displacement_from_start_m=drift,
                now_monotonic=loop_started,
            )
            decision = replace(decision, safe_command=safe)
            await self._send(safe)
            self.transmission_count += 1
            await self.display.render(self.coordinator, state, report, proximity, decision)
            elapsed_ms = (monotonic() - loop_started) * 1000.0
            self._record(decision, state, elapsed_ms, drift, transmitted=True)
            if self.args.json_log:
                print(json.dumps(self.records[-1], separators=(",", ":"), allow_nan=False))
            elif self.args.verbose and decision.mode.value != self.previous_mode:
                print(
                    f"{decision.mode.value}: centre={decision.risk.centre:.2f} "
                    f"left={decision.risk.left:.2f} right={decision.risk.right:.2f} "
                    f"issued F={safe.forward_m_s:+.2f} R={safe.right_m_s:+.2f}"
                )
            self.previous_mode = decision.mode.value
            if safe.hard_override or decision.mode is AvoidanceMode.ABORT:
                self.abort_reason = "; ".join(safe.reasons or decision.reasons)
                print(f"Safety abort: {self.abort_reason}")
                await self._failure_hover()
                return
            if self.display.stop_requested:
                self.abort_reason = "Operator requested stop"
                return
            if drift >= self.config.maximum_forward_travel_m:
                print("Forward travel proxy limit reached.")
                return
            next_tick += 1.0 / self.config.control_loop_hz
            sleep_time = next_tick - monotonic()
            if sleep_time < 0:
                self.loop_deadline_misses += 1
                next_tick = monotonic()
                sleep_time = 0
            await asyncio.sleep(sleep_time)

    async def _send(self, safe) -> None:
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                safe.forward_m_s,
                safe.right_m_s,
                safe.down_m_s,
                safe.yaw_rate_deg_s,
            )
        )

    async def _send_hover(self) -> None:
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    async def _failure_hover(self) -> None:
        deadline = monotonic() + self.config.failure_hover_s
        while monotonic() < deadline:
            with suppress(Exception):
                await self._send_hover()
            await asyncio.sleep(0.1)

    async def _delay_with_hover(self, duration_s: float) -> None:
        deadline = monotonic() + duration_s
        while monotonic() < deadline:
            await self._send_hover()
            await asyncio.sleep(0.1)

    async def _hover_and_land(self) -> None:
        self.supervisor.reset_emergency_confirmation("Landing sequence started")
        if self.landing_result != "NOT_ATTEMPTED":
            return
        if (
            not self.arming_attempted
            and not self.coordinator.state.armed
            and not self.coordinator.state.in_air
        ):
            self.landing_result = "NOT_REQUIRED"
            return
        print("Final zero-velocity hover...")
        with suppress(Exception):
            await self._delay_with_hover(self.config.final_hover_s)
        if self.offboard_started:
            with suppress(OffboardError):
                await self.drone.offboard.stop()
            self.offboard_started = False
        print("Landing...")
        try:
            landing_command_time = monotonic()
            await self.drone.action.land()
            await self._after_landing_command()
            deadline = monotonic() + 45.0
            landed_with_fresh_state = False
            while monotonic() < deadline:
                state = self.coordinator.state
                await self._landing_progress(state)
                if (
                    state.last_in_air_update_monotonic >= landing_command_time
                    and not state.in_air
                ):
                    with suppress(Exception):
                        await self.drone.action.disarm()
                    disarm_deadline = min(deadline, monotonic() + 5.0)
                    while monotonic() < disarm_deadline:
                        state = self.coordinator.state
                        if (
                            state.last_armed_update_monotonic >= landing_command_time
                            and not state.armed
                        ):
                            landed_with_fresh_state = True
                            self.supervisor.reset_emergency_confirmation(
                                "Vehicle disarmed"
                            )
                            break
                        await asyncio.sleep(0.1)
                    break
                await asyncio.sleep(0.2)
            self.landing_result = "CONFIRMED" if landed_with_fresh_state else "TIMEOUT"
            print(f"Landing result: {self.landing_result}")
        except Exception as exc:
            if self.arming_attempted:
                with suppress(Exception):
                    await self.drone.action.disarm()
            self.supervisor.reset_emergency_confirmation("Landing command failed")
            self.landing_result = f"COMMAND_FAILED: {exc}"

    async def _after_landing_command(self) -> None:
        return

    async def _landing_progress(self, state: NavigationState) -> None:
        return

    def _record(self, decision, state, elapsed_ms, drift, *, transmitted: bool) -> None:
        record = command_record(decision, state, elapsed_ms, drift, transmitted)
        self.records.append(record)
        mode = decision.safe_command.supervisor_mode.value if decision.safe_command else decision.mode.value
        if not self.transitions or self.transitions[-1]["mode"] != mode:
            supervised_reasons = (
                record["supervised"]["reasons"] if record["supervised"] else []
            )
            self.transitions.append(
                {
                    "timestamp_monotonic": decision.timestamp_monotonic,
                    "mode": mode,
                    "reason": (
                        supervised_reasons[-1]
                        if supervised_reasons
                        else decision.reasons[0]
                    ),
                }
            )
        if (
            not self.confidence_transitions
            or self.confidence_transitions[-1]["mode"] != decision.confidence_mode
        ):
            self.confidence_transitions.append(
                {
                    "timestamp_monotonic": decision.timestamp_monotonic,
                    "mode": decision.confidence_mode,
                }
            )
        self.max_displacement_m = max(self.max_displacement_m, drift)
        self.max_speed_m_s = max(self.max_speed_m_s, state.horizontal_speed_m_s)
        self.max_risk["centre"] = max(self.max_risk["centre"], decision.risk.centre)
        self.max_risk["left"] = max(self.max_risk["left"], decision.risk.left)
        self.max_risk["right"] = max(self.max_risk["right"], decision.risk.right)
        if decision.safe_command and transmitted:
            self.forward_reduced |= (
                decision.safe_command.forward_m_s
                < decision.requested_command.forward_m_s - 1e-6
            )
            self.avoidance_issued |= decision.safe_command.supervisor_mode.value in {
                "AVOID_LEFT",
                "AVOID_RIGHT",
                "RETREAT",
                "HOVER",
            }

    def artifact(self, started_at: str, completed_at: str) -> dict:
        if self.abort_reason:
            final_result = "ABORTED"
        elif self.takeoff_completed and self.landing_result != "CONFIRMED":
            final_result = "FAILED"
        else:
            final_result = "COMPLETED"
        return {
            "started_at_utc": started_at,
            "completed_at_utc": completed_at,
            "scenario": self.args.scenario,
            "configuration": asdict(self.config),
            "takeoff_status": (
                "COMPLETED"
                if self.takeoff_completed
                else "NOT_ATTEMPTED" if self.args.no_flight else "NOT_COMPLETED"
            ),
            "records": self.records,
            "supervisor_transitions": self.transitions,
            "confidence_transitions": self.confidence_transitions,
            "abort_reason": self.abort_reason,
            "landing_result": self.landing_result,
            "maximum_displacement_from_start_m": self.max_displacement_m,
            "maximum_horizontal_speed_m_s": self.max_speed_m_s,
            "maximum_observed_risk": self.max_risk,
            "forward_motion_reduced": self.forward_reduced,
            "avoidance_command_issued": self.avoidance_issued,
            "reversal_count": self.controller.state.reversal_count,
            "loop_deadline_misses": self.loop_deadline_misses,
            "transmitted_command_count": self.transmission_count,
            "collision_status": "NOT_INSTRUMENTED",
            "final_result": final_result,
        }

    def _drift(self, state: NavigationState) -> float:
        return math.hypot(
            state.position_ned_m.x - self.origin_north_m,
            state.position_ned_m.y - self.origin_east_m,
        )

    def _duration(self) -> float:
        return self.config.maximum_run_duration_s if self.args.duration is None else self.args.duration

    def _start_delay(self) -> float:
        return self.config.start_delay_s if self.args.start_delay is None else self.args.start_delay


async def run_live(args: argparse.Namespace, config: AvoidanceConfig) -> dict:
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    depth_config = DepthConfig.from_json(PROJECT_ROOT / "config/depth_anything.json")
    depth_config = replace(
        depth_config,
        device=args.device or depth_config.device,
        target_fps=(
            depth_config.target_fps
            if args.target_depth_fps is None
            else args.target_depth_fps
        ),
        depth_required_for_motion=config.depth_required_for_motion,
    )
    drone = OwnedMavsdkSystem()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(PROJECT_ROOT / "config/visual_health.json"),
        analysis_rate_hz=max(10.0, depth_config.target_fps * 2.0),
        depth_config=depth_config,
    )
    confidence = ConfidenceEngine(
        ConfidenceConfig.from_json(PROJECT_ROOT / "config/confidence.json")
    )
    operator_config = OperatorViewConfig.from_json(PROJECT_ROOT / "config/operator_view.json")
    display = IntegratedDisplay(args.display, operator_config)
    connection = ConnectionMonitor(drone)
    connection.start()
    started = datetime.now(timezone.utc)
    demo = ReactiveAvoidanceDemo(
        drone,
        coordinator,
        connection,
        config,
        confidence,
        display,
        args,
    )
    try:
        await coordinator.start()
        await connection.wait()
        await demo.run()
    except asyncio.CancelledError:
        demo.abort_reason = "Operator interrupted with Ctrl+C"
        print("Reactive avoidance interrupted after confirmed safe-landing attempt.")
    except DemoAbort as exc:
        demo.abort_reason = str(exc)
        print(f"Demo aborted: {exc}")
        if demo.landing_result == "NOT_ATTEMPTED":
            await asyncio.shield(demo._hover_and_land())
    finally:
        display.close()
        try:
            await coordinator.stop()
        finally:
            try:
                await connection.stop()
            finally:
                drone.close()
    completed = datetime.now(timezone.utc)
    return demo.artifact(started.isoformat(), completed.isoformat())


def main() -> int:
    args = parse_args()
    config = AvoidanceConfig.from_json(args.config)
    overrides = {}
    if args.duration is not None:
        overrides["maximum_run_duration_s"] = args.duration
    if args.forward_speed is not None:
        overrides["forward_speed_m_s"] = args.forward_speed
    if args.start_delay is not None:
        overrides["start_delay_s"] = args.start_delay
    if args.auto_land:
        overrides["automatic_landing_on_abort"] = True
    if overrides:
        config = replace(config, **overrides)
    config.validate()
    print(json.dumps(asdict(config), indent=2))
    if args.dry_run:
        return 0

    started = datetime.now(timezone.utc)
    try:
        if args.scenario == "synthetic-grid-test":
            payload = run_synthetic(config, args)
            payload["started_at_utc"] = started.isoformat()
            payload["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        else:
            payload = asyncio.run(run_live(args, config))
    except KeyboardInterrupt:
        print("Reactive avoidance interrupted; hover and landing were requested.")
        return 130
    stem = started.strftime("%Y%m%dT%H%M%SZ")
    path = write_avoidance_artifact(payload, args.output_dir, stem)
    print(f"Avoidance artifact: {path}")
    return 0 if payload.get("final_result", payload.get("result")) == "COMPLETED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
