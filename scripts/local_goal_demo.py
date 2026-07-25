#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
from contextlib import suppress
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from reactive_avoidance_demo import (  # noqa: E402
    ConnectionMonitor,
    IntegratedDisplay,
    ReactiveAvoidanceDemo,
    snapshot,
)
from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    AvoidanceMode,
    DesiredMotion,
    OwnedMavsdkSystem,
)
from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.navigation import (  # noqa: E402
    ActivePerceptionConfig,
    ActivePerceptionManager,
    LocalGoalNavigator,
    LocalNavigationConfig,
    RecoveryState,
    write_navigation_artifact,
)
from polaris.operator_view import OperatorViewConfig  # noqa: E402
from polaris.perception.depth import DepthConfig  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402
from polaris.simulation import CollisionMonitor  # noqa: E402


SCENARIOS = (
    "bounded-goal",
    "centred-obstacle",
    "low-texture-recovery",
    "glare-recovery",
    "no-acceptable-viewpoint",
    "no-progress-test",
    "collision-test",
    "hover-only",
)

SCENARIO_GOALS = {
    "bounded-goal": "corridor-staging",
    "centred-obstacle": "centred-obstacle",
    "low-texture-recovery": "snowfield-view",
    "glare-recovery": "glare-view",
    "no-acceptable-viewpoint": "corridor-staging",
    "no-progress-test": "corridor-staging",
    "collision-test": "corridor-staging",
    "hover-only": "corridor-staging",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run bounded local-goal navigation with active perception"
    )
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/local_navigation.json"))
    parser.add_argument(
        "--active-perception-config",
        default=str(PROJECT_ROOT / "config/active_perception.json"),
    )
    parser.add_argument(
        "--avoidance-config",
        default=str(PROJECT_ROOT / "config/reactive_avoidance.json"),
    )
    parser.add_argument("--goal")
    parser.add_argument("--scenario", choices=SCENARIOS, default="bounded-goal")
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--display", dest="display", action="store_true")
    display.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=True)
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-depth-fps", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--start-delay", type=float, default=2.0)
    parser.add_argument("--auto-land", action="store_true")
    parser.add_argument("--json-log", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-flight", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "artifacts/navigation_runs"),
    )
    parser.add_argument("--system-address", default="udp://:14540")
    return parser.parse_args()


def selected_goal(args: argparse.Namespace, config: LocalNavigationConfig) -> str:
    goal_id = args.goal or SCENARIO_GOALS[args.scenario]
    if config.goals is None or goal_id not in config.goals:
        raise ValueError(f"Unknown local goal: {goal_id}")
    return goal_id


def recovery_dict(decision) -> dict | None:
    if decision is None:
        return None
    result = asdict(decision)
    result["state"] = decision.state.value
    result["action"] = decision.action.value
    return result


class LocalGoalDemo(ReactiveAvoidanceDemo):
    def __init__(
        self,
        *args,
        navigation_config: LocalNavigationConfig,
        active_config: ActivePerceptionConfig,
        collision_monitor: CollisionMonitor,
        goal_id: str,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.navigation_config = navigation_config
        self.active_config = active_config
        self.collision_monitor = collision_monitor
        self.goal_id = goal_id
        self.navigator = LocalGoalNavigator(navigation_config)
        self.recovery = ActivePerceptionManager(active_config)
        self.navigation_records: list[dict] = []
        self.navigation_transitions: list[dict] = []
        self.recovery_events: list[dict] = []
        self.goal_reached = False
        self.goal_failed = False
        self.last_goal_decision = None
        self.last_recovery_decision = None
        self.last_avoidance_mode = AvoidanceMode.CLEAR
        self.last_avoidance_decision = None
        self.maximum_command_m_s = 0.0
        self.maximum_loop_elapsed_ms = 0.0
        self.initial_position = None
        self.final_position = None
        self.hover_only_completed = False

    async def run(self) -> None:
        await self._wait_ready_with_collision()
        if self.args.no_flight:
            await self._observe_goal_without_flight()
            return
        try:
            self.collision_monitor.clear_for_run()
            await self._takeoff()
            self.collision_monitor.set_ground_contact_allowed(False)
            if self.args.scenario == "hover-only":
                state, _ = snapshot(self.coordinator)
                self._start_goal(state, monotonic())
                await self._hover_only()
                self.hover_only_completed = True
            else:
                await self._local_goal_loop()
        finally:
            await self._shielded_hover_and_land()

    async def _wait_ready_with_collision(self) -> None:
        await self._wait_ready()
        deadline = monotonic() + 5.0
        while monotonic() < deadline:
            collision = self.collision_monitor.latest_state()
            if collision.available:
                print(
                    "Collision monitor ready: "
                    f"{collision.expected_ground_contact_count} expected ground events"
                )
                return
            await asyncio.sleep(0.1)
        raise RuntimeError("Gazebo collision instrumentation topic is unavailable")

    def _start_goal(self, state, now: float) -> None:
        goal = self.navigation_config.build_goal(
            self.goal_id,
            state.position_ned_m.x,
            state.position_ned_m.y,
            state.position_ned_m.z,
            now,
        )
        if self.args.duration is not None:
            goal = replace(goal, timeout_s=self.args.duration)
        self.navigator.start(goal, state, now)
        self.initial_position = asdict(state.position_ned_m)
        print(
            f"Local goal {goal.goal_id}: N={goal.target_north_m:+.2f}, "
            f"E={goal.target_east_m:+.2f}, D={goal.target_down_m:+.2f} m"
        )

    async def _observe_goal_without_flight(self) -> None:
        state, proximity = snapshot(self.coordinator)
        now = monotonic()
        self._start_goal(state, now)
        deadline = now + min(5.0, self.args.duration or 5.0)
        while monotonic() < deadline:
            cycle_started = monotonic()
            state, proximity = snapshot(self.coordinator)
            report = self.confidence.evaluate(state)
            goal = self.navigator.update(state, now_monotonic=cycle_started)
            avoidance = self.controller.update(
                goal.requested_motion,
                state,
                report.mode.name,
                cycle_started,
            )
            safe = self.supervisor.evaluate(
                avoidance,
                state,
                report,
                px4_connected=self.connection.connected,
                autonomy_enabled=False,
                offboard_expected=False,
                displacement_from_start_m=goal.metrics.displacement_from_start_m,
                collision_detected=False,
                now_monotonic=cycle_started,
            )
            avoidance = replace(avoidance, safe_command=safe, autonomy_enabled=False)
            self.collision_monitor.update_vertical_speed(state.velocity_ned_m_s.z)
            collision = self.collision_monitor.latest_state()
            await self.display.render(
                self.coordinator,
                state,
                report,
                proximity,
                avoidance,
                goal,
                None,
                collision,
            )
            self._record_navigation(
                goal,
                None,
                avoidance,
                collision,
                state,
                report.mode.name,
                0.0,
                transmitted=False,
            )
            self.last_goal_decision = goal
            if self.display.stop_requested:
                self.abort_reason = "Operator requested stop during no-flight observation"
                break
            await asyncio.sleep(1.0 / self.navigation_config.control_loop_hz)
        self.final_position = asdict(self.coordinator.state.position_ned_m)

    async def _local_goal_loop(self) -> None:
        await self._delay_with_hover(self.args.start_delay)
        state, _ = snapshot(self.coordinator)
        started = monotonic()
        self._start_goal(state, started)
        next_tick = started
        while True:
            cycle_started = monotonic()
            state, proximity = snapshot(self.coordinator)
            self._inject_controlled_state(state, cycle_started - started)
            report = self.confidence.evaluate(state)
            self.collision_monitor.update_vertical_speed(state.velocity_ned_m_s.z)
            collision = self.collision_monitor.latest_state()

            goal = self.navigator.update(
                state,
                now_monotonic=cycle_started,
                avoidance_mode=self.last_avoidance_mode,
                recovery_active=self.recovery.active,
            )
            trigger = (
                None
                if self.recovery.active or goal.goal_reached or goal.failed
                else self.recovery.recovery_trigger(
                    state,
                    goal,
                    self.last_avoidance_decision,
                )
            )
            if trigger is not None:
                self.recovery.trigger(
                    trigger,
                    state,
                    goal.metrics.goal_bearing_deg,
                    cycle_started,
                )

            recovery = None
            requested = goal.requested_motion
            if self.recovery.active or self.recovery.state in {
                RecoveryState.COMPLETE,
                RecoveryState.FAILED,
            }:
                recovery = self.recovery.update(state, cycle_started)
                requested = recovery.command
                if recovery.state is RecoveryState.COMPLETE:
                    self.navigator.complete_recovery(
                        True,
                        cycle_started,
                        recovery.selected_heading_deg,
                    )
                    self.recovery_events.append(recovery_dict(recovery))
                    self.recovery.reset_after_completion()
                elif recovery.state is RecoveryState.FAILED:
                    self.navigator.complete_recovery(False, cycle_started)
                    self.goal_failed = True
                    self.abort_reason = "; ".join(recovery.reasons)

            if self.args.scenario == "no-progress-test" and recovery is None:
                requested = DesiredMotion.hover(cycle_started, "controlled-no-progress")

            avoidance = self.controller.update(
                requested,
                state,
                report.mode.name,
                cycle_started,
            )
            safe = self.supervisor.evaluate(
                avoidance,
                state,
                report,
                px4_connected=self.connection.connected,
                autonomy_enabled=True,
                offboard_expected=True,
                displacement_from_start_m=goal.metrics.displacement_from_start_m,
                collision_detected=collision.collision_detected,
                now_monotonic=cycle_started,
            )
            avoidance = replace(avoidance, safe_command=safe)
            try:
                await self._send(safe)
                transmitted = True
            except Exception as exc:
                transmitted = False
                self.abort_reason = f"Safe command transmission failed: {exc}"
            self.maximum_command_m_s = max(
                self.maximum_command_m_s,
                math.hypot(safe.forward_m_s, safe.right_m_s),
            )
            self.last_goal_decision = goal
            self.last_recovery_decision = recovery
            self.last_avoidance_mode = avoidance.mode
            self.last_avoidance_decision = avoidance
            await self.display.render(
                self.coordinator,
                state,
                report,
                proximity,
                avoidance,
                goal,
                recovery,
                collision,
            )
            elapsed_ms = (monotonic() - cycle_started) * 1000.0
            self.maximum_loop_elapsed_ms = max(self.maximum_loop_elapsed_ms, elapsed_ms)
            transition_changed = (
                not self.navigation_transitions
                or self.navigation_transitions[-1]["state"] != goal.state.value
            )
            self._record_navigation(
                goal,
                recovery,
                avoidance,
                collision,
                state,
                report.mode.name,
                elapsed_ms,
                transmitted=transmitted,
            )

            if self.args.json_log:
                print(json.dumps(self.navigation_records[-1], separators=(",", ":")))
            if self.args.verbose and transition_changed:
                print(
                    f"{goal.state.value}: distance={goal.metrics.goal_distance_m:.2f} m "
                    f"path={goal.metrics.path_length_proxy_m:.2f} m "
                    f"avoidance={avoidance.mode.value} supervisor={safe.supervisor_mode.value}"
                )

            if safe.hard_override:
                self.abort_reason = "; ".join(safe.reasons)
                self.goal_failed = True
                break
            if not transmitted or self.goal_failed:
                break
            if goal.goal_reached:
                self.goal_reached = True
                break
            if goal.failed:
                self.goal_failed = True
                self.abort_reason = "; ".join(goal.reasons)
                break
            if self.display.stop_requested:
                self.abort_reason = "Operator requested stop"
                break

            next_tick += 1.0 / self.navigation_config.control_loop_hz
            sleep_time = next_tick - monotonic()
            if sleep_time < 0:
                self.loop_deadline_misses += 1
                next_tick = monotonic()
                sleep_time = 0.0
            await asyncio.sleep(sleep_time)
        self.final_position = asdict(self.coordinator.state.position_ned_m)

    def _inject_controlled_state(self, state, elapsed_s: float) -> None:
        if self.args.scenario == "collision-test" and elapsed_s >= 2.0:
            if not self.collision_monitor.latest_state().collision_detected:
                self.collision_monitor.inject_collision(
                    "controlled_low_speed_collision_fixture"
                )
        if self.args.scenario == "no-acceptable-viewpoint" and elapsed_s >= 2.0:
            state.visual = replace(
                state.visual,
                visual_health_score=0.45,
                reasons=("Controlled no-acceptable-viewpoint state",),
            )
            state.depth = replace(
                state.depth,
                depth_health_score=0.42,
                temporal_stability=0.68,
            )

    async def _after_landing_command(self) -> None:
        self.collision_monitor.set_ground_contact_allowed(True)

    async def _takeoff_safety_hook(self, state) -> None:
        self.collision_monitor.update_vertical_speed(state.velocity_ned_m_s.z)
        vertical_clearance_m = self.origin_down_m - state.position_ned_m.z
        if state.in_air and vertical_clearance_m >= 0.35:
            self.collision_monitor.set_ground_contact_allowed(False)
        collision = self.collision_monitor.latest_state()
        if collision.collision_detected:
            raise RuntimeError(
                "Collision detected during takeoff: "
                f"{collision.latest_contacted_entity}"
            )

    async def _landing_progress(self, state) -> None:
        self.collision_monitor.update_vertical_speed(state.velocity_ned_m_s.z)

    def _record_navigation(
        self,
        goal,
        recovery,
        avoidance,
        collision,
        state,
        confidence_mode: str,
        loop_elapsed_ms: float,
        *,
        transmitted: bool,
    ) -> None:
        record = {
            "timestamp_monotonic": goal.timestamp_monotonic,
            "navigator_state": goal.state.value,
            "goal": asdict(goal.goal),
            "metrics": asdict(goal.metrics),
            "requested_motion": asdict(goal.requested_motion),
            "recovery": recovery_dict(recovery),
            "avoidance": avoidance.to_dict(),
            "command_transmitted": transmitted,
            "collision": asdict(collision),
            "confidence_mode": confidence_mode,
            "position_ned_m": asdict(state.position_ned_m),
            "velocity_ned_m_s": asdict(state.velocity_ned_m_s),
            "visual_health": state.visual.visual_health_score,
            "depth_health": state.depth.depth_health_score,
            "depth_stability": state.depth.temporal_stability,
            "loop_elapsed_ms": loop_elapsed_ms,
        }
        self.navigation_records.append(record)
        if (
            not self.navigation_transitions
            or self.navigation_transitions[-1]["state"] != goal.state.value
        ):
            self.navigation_transitions.append(
                {
                    "timestamp_monotonic": goal.timestamp_monotonic,
                    "state": goal.state.value,
                    "reason": goal.reasons[0] if goal.reasons else "None",
                }
            )

    def navigation_artifact(self, started_at: str, completed_at: str) -> dict:
        final_metrics = (
            None if self.last_goal_decision is None else asdict(self.last_goal_decision.metrics)
        )
        collision = self.collision_monitor.latest_state()
        has_injected_collision = collision.injected_collision_detected
        has_native_collision = collision.native_collision_detected
        if has_injected_collision and has_native_collision:
            evidence_mode = "MIXED"
        elif has_native_collision:
            evidence_mode = "NATIVE"
        elif has_injected_collision:
            evidence_mode = "INJECTED"
        else:
            evidence_mode = "NONE"
        if collision.collision_detected:
            outcome = f"{evidence_mode}_COLLISION_ABORT"
        elif self.args.no_flight:
            outcome = "NO_FLIGHT_COMPLETED"
        elif self.hover_only_completed:
            outcome = "HOVER_ONLY_COMPLETED"
        elif self.goal_reached:
            outcome = "GOAL_REACHED"
        else:
            outcome = "GOAL_FAILED"
        return {
            "schema_version": 1,
            "started_at_utc": started_at,
            "completed_at_utc": completed_at,
            "scenario": self.args.scenario,
            "goal_id": self.goal_id,
            "navigation_configuration": asdict(self.navigation_config),
            "active_perception_configuration": asdict(self.active_config),
            "avoidance_configuration": asdict(self.config),
            "initial_position_ned_m": self.initial_position,
            "final_position_ned_m": self.final_position,
            "final_metrics": final_metrics,
            "navigation_transitions": self.navigation_transitions,
            "recovery_events": self.recovery_events,
            "records": self.navigation_records,
            "collision_monitor_available": collision.available,
            "collision_detected": collision.collision_detected,
            "collision_count": collision.collision_count,
            "collision_evidence_mode": evidence_mode,
            "collision_events": [asdict(event) for event in collision.events],
            "goal_outcome": outcome,
            "abort_reason": self.abort_reason,
            "landing_result": self.landing_result,
            "maximum_speed_m_s": 0.0 if not self.navigation_records else max(
                math.hypot(
                    record["velocity_ned_m_s"]["x"],
                    record["velocity_ned_m_s"]["y"],
                )
                for record in self.navigation_records
            ),
            "maximum_command_m_s": self.maximum_command_m_s,
            "maximum_loop_elapsed_ms": self.maximum_loop_elapsed_ms,
            "loop_deadline_misses": self.loop_deadline_misses,
            "final_result": (
                "COMPLETED"
                if self.abort_reason is None
                and not collision.collision_detected
                and outcome in {
                    "GOAL_REACHED",
                    "NO_FLIGHT_COMPLETED",
                    "HOVER_ONLY_COMPLETED",
                }
                and (
                    self.args.no_flight
                    or self.landing_result == "CONFIRMED"
                )
                else "ABORTED"
            ),
        }


async def run_live(
    args: argparse.Namespace,
    nav_config: LocalNavigationConfig,
    active_config: ActivePerceptionConfig,
    avoidance_config: AvoidanceConfig,
    goal_id: str,
) -> dict:
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
        depth_required_for_motion=True,
    )
    drone = OwnedMavsdkSystem()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(
            PROJECT_ROOT / "config/visual_health.json"
        ),
        analysis_rate_hz=max(10.0, depth_config.target_fps * 2.0),
        depth_config=depth_config,
    )
    confidence = ConfidenceEngine(
        ConfidenceConfig.from_json(PROJECT_ROOT / "config/confidence.json")
    )
    display = IntegratedDisplay(
        args.display,
        OperatorViewConfig.from_json(PROJECT_ROOT / "config/operator_view.json"),
    )
    connection = ConnectionMonitor(drone)
    collision = CollisionMonitor(
        maximum_touchdown_speed_m_s=nav_config.maximum_touchdown_speed_m_s,
        maximum_touchdown_depth_m=nav_config.maximum_touchdown_depth_m,
    )
    connection.start()
    collision.start()
    started = datetime.now(timezone.utc)
    demo = LocalGoalDemo(
        drone,
        coordinator,
        connection,
        avoidance_config,
        confidence,
        display,
        args,
        navigation_config=nav_config,
        active_config=active_config,
        collision_monitor=collision,
        goal_id=goal_id,
    )
    try:
        await coordinator.start()
        await connection.wait()
        await demo.run()
    except asyncio.CancelledError:
        demo.abort_reason = "Operator interrupted with Ctrl+C"
    except Exception as exc:
        demo.abort_reason = str(exc)
        if demo.landing_result == "NOT_ATTEMPTED":
            await asyncio.shield(demo._hover_and_land())
    finally:
        display.close()
        collision.close()
        try:
            await asyncio.wait_for(coordinator.stop(), timeout=10.0)
        except Exception as exc:
            if demo.abort_reason is None:
                demo.abort_reason = f"Coordinator shutdown failed: {exc}"
        finally:
            try:
                await asyncio.wait_for(connection.stop(), timeout=5.0)
            except Exception as exc:
                if demo.abort_reason is None:
                    demo.abort_reason = f"Connection monitor shutdown failed: {exc}"
            finally:
                drone.close()
    return demo.navigation_artifact(
        started.isoformat(),
        datetime.now(timezone.utc).isoformat(),
    )


def main() -> int:
    args = parse_args()
    nav_config = LocalNavigationConfig.from_json(args.config)
    active_config = ActivePerceptionConfig.from_json(args.active_perception_config)
    avoidance_config = AvoidanceConfig.from_json(args.avoidance_config)
    goal_id = selected_goal(args, nav_config)
    if args.duration is not None:
        if args.duration <= 0:
            raise ValueError("--duration must be positive")
        nav_config = replace(nav_config, maximum_goal_duration_s=args.duration)
    avoidance_config = replace(
        avoidance_config,
        takeoff_altitude_m=nav_config.takeoff_altitude_m,
        final_hover_s=nav_config.final_hover_s,
        automatic_landing_on_abort=nav_config.automatic_landing_on_failure,
    )
    print(
        json.dumps(
            {
                "goal": goal_id,
                "scenario": args.scenario,
                "navigation": asdict(nav_config),
                "active_perception": asdict(active_config),
            },
            indent=2,
        )
    )
    if args.dry_run:
        return 0
    started = datetime.now(timezone.utc)
    payload = asyncio.run(
        run_live(args, nav_config, active_config, avoidance_config, goal_id)
    )
    path = write_navigation_artifact(
        payload,
        args.output_dir,
        started.strftime("%Y%m%dT%H%M%SZ"),
    )
    print(f"Navigation artifact: {path}")
    return 0 if payload["final_result"] == "COMPLETED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
