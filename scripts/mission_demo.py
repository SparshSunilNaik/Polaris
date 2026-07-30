#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
from dataclasses import asdict, replace
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from time import monotonic

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from local_goal_demo import (  # noqa: E402
    ConnectionMonitor,
    IntegratedDisplay,
    LocalGoalDemo,
    snapshot,
)
from polaris.control import (  # noqa: E402
    AvoidanceConfig,
    DesiredMotion,
    OwnedMavsdkSystem,
    RiskSummary,
)
from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.ego_context import (  # noqa: E402
    EgoContextCoordinator,
    EgoContextMemory,
    RuntimeContextProjection,
)
from polaris.mission import (  # noqa: E402
    BoundedMissionReplanner,
    MissionAction,
    MissionOrchestrator,
    MissionOrchestratorConfig,
    MissionPhase,
    MissionPlan,
    MissionResult,
    MissionWaypoint,
    ObservationTracker,
    WaypointCompletionMode,
    append_supervisor_transition,
    format_mission_replay,
    load_mission,
    write_mission_artifact,
)
from polaris.navigation import (  # noqa: E402
    ActivePerceptionConfig,
    ActivePerceptionManager,
    LocalGoal,
    LocalNavigationConfig,
    RecoveryState,
)
from polaris.operator_view import OperatorViewConfig  # noqa: E402
from polaris.observability import MissionEventRecorder, MissionEventType  # noqa: E402
from polaris.perception.depth import DepthConfig  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402
from polaris.simulation import CollisionMonitor  # noqa: E402
from polaris.vlm import (  # noqa: E402
    FusedSceneStatus,
    SemanticFusionConfig,
    VLMConfig,
    VLMService,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run deterministic short multi-waypoint Polaris missions"
    )
    parser.add_argument("--mission", default="two-point-demo")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config/mission_orchestrator.json"),
    )
    parser.add_argument(
        "--local-navigation-config",
        default=str(PROJECT_ROOT / "config/local_navigation.json"),
    )
    parser.add_argument(
        "--active-perception-config",
        default=str(PROJECT_ROOT / "config/active_perception.json"),
    )
    parser.add_argument(
        "--avoidance-config",
        default=str(PROJECT_ROOT / "config/reactive_avoidance.json"),
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--display", dest="display", action="store_true")
    display.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=True)
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-depth-fps", type=float)
    vlm = parser.add_mutually_exclusive_group()
    vlm.add_argument("--vlm", dest="vlm", action="store_true")
    vlm.add_argument("--no-vlm", dest="vlm", action="store_false")
    parser.set_defaults(vlm=False)
    parser.add_argument(
        "--vlm-config",
        default=str(PROJECT_ROOT / "config/vlm.json"),
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument("--auto-land", action="store_true")
    parser.add_argument("--json-log", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-flight", action="store_true")
    parser.add_argument("--start-waypoint")
    parser.add_argument("--stop-after-waypoint")
    parser.add_argument("--start-delay", type=float, default=2.0)
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "artifacts/mission_runs"),
    )
    parser.add_argument("--system-address", default="udp://:14540")
    return parser.parse_args()


def jsonable(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


class MissionDemo(LocalGoalDemo):
    def __init__(
        self,
        *args,
        mission_plan: MissionPlan,
        mission_config: MissionOrchestratorConfig,
        ego_context: EgoContextCoordinator | None = None,
        **kwargs,
    ) -> None:
        super().__init__(*args, goal_id=mission_plan.waypoints[0].waypoint_id, **kwargs)
        self.mission_plan = mission_plan
        self.mission_config = mission_config
        self.ego_context = ego_context
        self.orchestrator = MissionOrchestrator(mission_config)
        self.replanner = BoundedMissionReplanner(mission_config)
        self.mission_origin_ned: tuple[float, float, float] | None = None
        self.mission_path_length_m = 0.0
        self._last_mission_position: tuple[float, float] | None = None
        self._last_mission_position_update = 0.0
        self._next_artifact_sample = 0.0
        self.waypoint_results: list[dict] = []
        self.replanning_decisions: list[dict] = []
        self.avoidance_transitions: list[dict] = []
        self.supervisor_transitions: list[dict] = []
        self.observation_events: list[dict] = []
        self._last_recorded_avoidance = None
        self._last_recorded_supervisor = None
        self.maximum_mission_speed_m_s = 0.0
        self.last_selected_heading_deg: float | None = None
        self.last_observation_decision = None
        self.no_flight_completed = False
        self.event_recorder = MissionEventRecorder(mission_plan.mission_id)
        self._polaris_terminal_landing = False

    async def run(self) -> None:
        self.supervisor.reset()
        started = monotonic()
        self.orchestrator.start(
            self.mission_plan,
            started,
            start_waypoint=self.args.start_waypoint,
            stop_after_waypoint=self.args.stop_after_waypoint,
        )
        self._record_event(
            MissionEventType.MISSION_STARTED,
            "mission_orchestrator",
            f"Mission {self.mission_plan.mission_id} started",
        )
        if self.ego_context is not None:
            self.ego_context.start_mission(
                self.mission_plan,
                self.orchestrator.state,
                started,
            )
        await self._wait_ready_with_collision()
        self._record_event(MissionEventType.MISSION_READY, "mission_runtime", "Mission readiness confirmed")
        if self.args.no_flight:
            await self._observe_mission_without_flight()
            return
        try:
            self.collision_monitor.clear_for_run()
            self.orchestrator.mark_takeoff(monotonic())
            await self._takeoff()
            self.collision_monitor.set_ground_contact_allowed(False)
            state, _ = snapshot(self.coordinator)
            self.mission_origin_ned = (
                state.position_ned_m.x,
                state.position_ned_m.y,
                state.position_ned_m.z,
            )
            self.initial_position = asdict(state.position_ned_m)
            self._last_mission_position = (
                state.position_ned_m.x,
                state.position_ned_m.y,
            )
            self._last_mission_position_update = state.last_position_update_monotonic
            self.orchestrator.mark_mission_start(monotonic())
            await self._delay_with_hover(self.args.start_delay)
            self.coordinator.set_vlm_observation_enabled(True)
            await self._mission_loop()
        finally:
            self._record_event(MissionEventType.SHUTDOWN_STARTED, "mission_runtime", "Mission shutdown started")
            self.coordinator.set_vlm_observation_enabled(False)
            self.orchestrator.begin_return_or_land(monotonic())
            await self._shielded_hover_and_land()
            self.orchestrator.mark_landing(self.landing_result, monotonic())
            self.orchestrator.finish(self.landing_result, monotonic())
            result = self.orchestrator.state.result
            event = (
                MissionEventType.MISSION_COMPLETED
                if result in {MissionResult.COMPLETED, MissionResult.COMPLETED_WITH_SKIPS}
                else MissionEventType.MISSION_ABORTED
                if result is MissionResult.ABORTED
                else MissionEventType.MISSION_FAILED
            )
            self._record_event(event, "mission_orchestrator", f"Mission finished with {result.value}")
            self._project_terminal_context(finish=True)

    async def _observe_mission_without_flight(self) -> None:
        self.coordinator.set_vlm_observation_enabled(True)
        await self._wait_for_no_flight_vlm_result()
        state, proximity = snapshot(self.coordinator)
        self._observe_runtime_state(state, self.collision_monitor.latest_state())
        self.mission_origin_ned = (
            state.position_ned_m.x,
            state.position_ned_m.y,
            state.position_ned_m.z,
        )
        waypoint = self.orchestrator.current_waypoint
        assert waypoint is not None
        goal = self._resolve_goal(waypoint, state, monotonic())
        self.navigator.start(goal, state, monotonic())
        decision = self.navigator.update(state, now_monotonic=monotonic())
        report = self.confidence.evaluate(state)
        avoidance = self.controller.update(
            decision.requested_motion,
            state,
            report.mode.name,
            monotonic(),
        )
        safe = self.supervisor.evaluate(
            avoidance,
            state,
            report,
            px4_connected=self.connection.connected,
            autonomy_enabled=False,
            offboard_expected=False,
            displacement_from_start_m=0.0,
            collision_detected=False,
            now_monotonic=monotonic(),
        )
        avoidance = replace(avoidance, safe_command=safe, autonomy_enabled=False)
        collision = self.collision_monitor.latest_state()
        ego_context = self._project_ego_context(
            state=state,
            report=report,
            goal=decision,
            avoidance=avoidance,
            collision=collision,
        )
        await self.display.render(
            self.coordinator,
            state,
            report,
            proximity,
            avoidance,
            decision,
            None,
            collision,
            self.orchestrator.state,
            None,
            ego_context,
        )
        self._record_cycle(
            decision,
            None,
            avoidance,
            collision,
            state,
            report.mode.name,
            0.0,
            transmitted=False,
            force=True,
        )
        self.no_flight_completed = True
        self.final_position = asdict(state.position_ned_m)

    async def _wait_for_no_flight_vlm_result(self) -> None:
        worker = self.coordinator.vlm_worker
        if worker is None:
            return
        fusion = SemanticFusionConfig.from_vlm_config(worker.config)
        wait_budget = (
            worker.config.inference_budget_s
            + fusion.minimum_evidence_count * worker.config.inference_interval_s
            + 10.0
        )
        deadline = monotonic() + min(45.0, max(20.0, wait_budget))
        while monotonic() < deadline:
            state, _ = snapshot(self.coordinator)
            self._project_ego_context(
                state=state,
                report=self.confidence.evaluate(state),
            )
            fused = self.ego_context.snapshot().perception.fused_scene
            if fused.status in {FusedSceneStatus.STABLE, FusedSceneStatus.CONFLICTED}:
                return
            await asyncio.sleep(0.1)

    async def _mission_loop(self) -> None:
        while self.orchestrator.state.state not in {
            MissionPhase.MISSION_COMPLETE,
            MissionPhase.ABORT,
        }:
            waypoint = self.orchestrator.current_waypoint
            if waypoint is None:
                self.orchestrator.abort("Mission has no current waypoint", monotonic())
                break
            if monotonic() < self.orchestrator.pause_until:
                if not await self._supervised_pause(self.orchestrator.pause_until):
                    break
            self.orchestrator.prepare_waypoint(monotonic())
            outcome = await self._execute_waypoint(waypoint)
            if outcome["completed"]:
                continue
            decision = self.orchestrator.fail_waypoint(
                outcome["reason"],
                monotonic(),
                hard_failure=outcome["hard_failure"],
            )
            outcome["policy_action"] = decision.action.value
            if self.waypoint_results:
                self.waypoint_results[-1]["policy_action"] = decision.action.value
            if decision.action is MissionAction.RETRY_WAYPOINT:
                continue
            if decision.action is MissionAction.SKIP_WAYPOINT:
                continue
            if decision.action is MissionAction.REQUEST_REPLAN:
                if not self._apply_bounded_replan(waypoint):
                    self.orchestrator.abort("No valid bounded replan available", monotonic())
                    break
                continue
            if decision.action in {MissionAction.ABORT_MISSION, MissionAction.LAND}:
                break
        self.abort_reason = self.orchestrator.state.abort_reason
        self.final_position = asdict(self.coordinator.state.position_ned_m)

    async def _execute_waypoint(self, waypoint: MissionWaypoint) -> dict:
        state, _ = snapshot(self.coordinator)
        now = monotonic()
        goal = self._resolve_goal(waypoint, state, now)
        self.goal_id = waypoint.waypoint_id
        self.navigator.start(goal, state, now)
        self.recovery = ActivePerceptionManager(self.active_config)
        self.goal_reached = False
        self.goal_failed = False
        self.last_goal_decision = None
        self.last_recovery_decision = None
        self.last_observation_decision = None
        self.orchestrator.begin_waypoint(now)
        attempt = self.orchestrator.state.current_attempt
        self._record_event(
            MissionEventType.WAYPOINT_STARTED,
            "mission_orchestrator",
            f"Waypoint {waypoint.waypoint_id} started",
            data={"waypoint_id": waypoint.waypoint_id, "attempt": attempt},
        )
        record_start = len(self.navigation_records)
        transition_start = len(self.navigation_transitions)
        recovery_start = len(self.recovery_events)
        observation_start = len(self.observation_events)
        observation_tracker = (
            None
            if waypoint.observation is None
            else ObservationTracker(waypoint.waypoint_id, waypoint.observation)
        )
        observation_event_cursor = 0
        start_position = asdict(state.position_ned_m)
        started = now
        verification_active = False
        reason = "Unknown waypoint failure"
        hard_failure = False
        next_tick = now

        while True:
            cycle_started = monotonic()
            state, proximity = snapshot(self.coordinator)
            elapsed = cycle_started - started
            self._update_mission_path(state)
            self._apply_waypoint_injection(waypoint, elapsed)
            report = self.confidence.evaluate(state)
            self.collision_monitor.update_vertical_speed(state.velocity_ned_m_s.z)
            collision = self.collision_monitor.latest_state()
            self._observe_runtime_state(state, collision)
            mission_displacement = self._mission_displacement(state)

            goal_decision = self.navigator.update(
                state,
                now_monotonic=cycle_started,
                avoidance_mode=self.last_avoidance_mode,
                recovery_active=self.recovery.active,
            )
            bound_failure = self.orchestrator.update_metrics(
                cycle_started,
                path_length_proxy_m=self.mission_path_length_m,
                displacement_from_start_m=mission_displacement,
                current_goal_distance_m=goal_decision.metrics.goal_distance_m,
            )

            trigger = None if self.recovery.active or goal_decision.failed else self.recovery.recovery_trigger(
                state,
                goal_decision,
                self.last_avoidance_decision,
            )
            if trigger is not None:
                self.recovery.trigger(
                    trigger,
                    state,
                    goal_decision.metrics.goal_bearing_deg,
                    cycle_started,
                )
            recovery = None
            requested = goal_decision.requested_motion
            recovery_failed = False
            if self.recovery.active or self.recovery.state in {
                RecoveryState.COMPLETE,
                RecoveryState.FAILED,
            }:
                recovery = self.recovery.update(state, cycle_started)
                requested = recovery.command
                if recovery.state is RecoveryState.COMPLETE:
                    self.last_selected_heading_deg = recovery.selected_heading_deg
                    self.navigator.complete_recovery(
                        True,
                        cycle_started,
                        recovery.selected_heading_deg,
                    )
                    self.recovery_events.append(
                        {
                            "waypoint_id": waypoint.waypoint_id,
                            "attempt": attempt,
                            "decision": jsonable(asdict(recovery)),
                        }
                    )
                    self.recovery.reset_after_completion()
                    if verification_active:
                        self.orchestrator.begin_verification(cycle_started)
                elif recovery.state is RecoveryState.FAILED:
                    requested = DesiredMotion.hover(cycle_started, "mission-recovery-failed")
                    recovery_failed = True

            controlled_failure = waypoint.metadata.get("controlled_soft_failure_after_s")
            soft_failure = controlled_failure is not None and elapsed >= float(controlled_failure)
            if soft_failure or goal_decision.failed or recovery_failed or bound_failure is not None:
                requested = DesiredMotion.hover(cycle_started, "mission-waypoint-failure")

            observation = None
            if (
                observation_tracker is not None
                and not goal_decision.goal_reached
                and not soft_failure
                and not goal_decision.failed
                and not recovery_failed
                and bound_failure is None
            ):
                risk = RiskSummary.from_grid(state.depth.risk_grid)
                observation = observation_tracker.update(
                    state,
                    goal_distance_m=goal_decision.metrics.goal_distance_m,
                    heading_error_deg=goal_decision.metrics.yaw_error_deg,
                    centre_risk=risk.centre,
                    collision_detected=collision.collision_detected,
                    active_perception_state=self.recovery.state,
                    now_monotonic=cycle_started,
                )
                for event in observation_tracker.events[observation_event_cursor:]:
                    payload = jsonable(asdict(event))
                    payload["attempt"] = attempt
                    self.observation_events.append(payload)
                observation_event_cursor = len(observation_tracker.events)
                if observation.hold_required:
                    # Active Perception remains the sole owner of bounded dwell yaw.
                    requested = self.recovery.observation_hold_command(
                        requested.yaw_rate_deg_s,
                        cycle_started,
                    )
            if observation is not None:
                self.last_observation_decision = observation

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
                displacement_from_start_m=mission_displacement,
                collision_detected=collision.collision_detected,
                now_monotonic=cycle_started,
            )
            avoidance = replace(avoidance, safe_command=safe)
            try:
                await self._send(safe)
                transmitted = True
            except Exception as exc:
                transmitted = False
                reason = f"Safe command transmission failed: {exc}"
                hard_failure = True

            if (
                observation is not None
                and observation.completion_ready
                and transmitted
                and not safe.hard_override
            ):
                observation = observation_tracker.accept_completion(
                    observation,
                    state,
                    cycle_started,
                )
                for event in observation_tracker.events[observation_event_cursor:]:
                    payload = jsonable(asdict(event))
                    payload["attempt"] = attempt
                    self.observation_events.append(payload)
                observation_event_cursor = len(observation_tracker.events)
                self.last_observation_decision = observation

            self.maximum_command_m_s = max(
                self.maximum_command_m_s,
                math.hypot(safe.forward_m_s, safe.right_m_s),
            )
            self.maximum_mission_speed_m_s = max(
                self.maximum_mission_speed_m_s,
                state.horizontal_speed_m_s,
            )
            self.last_goal_decision = goal_decision
            self.last_recovery_decision = recovery
            self.last_avoidance_mode = avoidance.mode
            self.last_avoidance_decision = avoidance
            ego_context = self._project_ego_context(
                timestamp=cycle_started,
                state=state,
                report=report,
                goal=goal_decision,
                recovery=recovery,
                avoidance=avoidance,
                observation=observation,
                collision=collision,
            )
            await self.display.render(
                self.coordinator,
                state,
                report,
                proximity,
                avoidance,
                goal_decision,
                recovery,
                collision,
                self.orchestrator.state,
                observation,
                ego_context,
            )
            elapsed_ms = (monotonic() - cycle_started) * 1000.0
            self.maximum_loop_elapsed_ms = max(self.maximum_loop_elapsed_ms, elapsed_ms)
            self._record_control_transitions(avoidance, cycle_started)
            self._record_cycle(
                goal_decision,
                recovery,
                avoidance,
                collision,
                state,
                report.mode.name,
                elapsed_ms,
                transmitted=transmitted,
                observation=observation,
                force=(safe.hard_override or soft_failure or goal_decision.failed or recovery_failed),
            )
            if self.args.json_log and self.navigation_records:
                print(json.dumps(self.navigation_records[-1], separators=(",", ":")))

            if safe.hard_override:
                reason = "; ".join(safe.reasons)
                hard_failure = True
                break
            if not transmitted:
                break
            if bound_failure is not None:
                reason = bound_failure.reason
                hard_failure = True
                break
            if soft_failure:
                reason = "Controlled soft waypoint failure"
                break
            if recovery_failed:
                reason = "; ".join(recovery.reasons)
                break
            if goal_decision.failed:
                reason = "; ".join(goal_decision.reasons)
                break
            if self.display.stop_requested:
                reason = "Operator requested mission stop"
                hard_failure = True
                break

            if observation is not None and observation.completed:
                completion = self.orchestrator.complete_waypoint(
                    cycle_started,
                    completion_mode=WaypointCompletionMode.OBSERVATION,
                    reason=(
                        "Waypoint completed by stable observation at "
                        f"{observation.goal_distance_m:.2f} metres"
                    ),
                )
                summary = self._waypoint_summary(
                    waypoint,
                    attempt,
                    started,
                    cycle_started,
                    start_position,
                    state,
                    goal_decision,
                    "COMPLETED",
                    None,
                    record_start,
                    transition_start,
                    recovery_start,
                    completion_mode=WaypointCompletionMode.OBSERVATION,
                    observation=observation,
                    observation_start=observation_start,
                )
                self.waypoint_results.append(summary)
                self._record_event(MissionEventType.WAYPOINT_REACHED, "mission_orchestrator", f"Waypoint {waypoint.waypoint_id} reached", data={"waypoint_id": waypoint.waypoint_id, "completion_mode": "OBSERVATION"})
                return {
                    "completed": completion.action
                    in {MissionAction.START_WAYPOINT, MissionAction.COMPLETE_MISSION},
                    "reason": "Waypoint completed by stable observation",
                    "hard_failure": False,
                }

            if goal_decision.goal_reached and not verification_active:
                verification_active = True
                self.orchestrator.begin_verification(cycle_started)
            if verification_active and not self.recovery.active:
                verification = self.orchestrator.verify_waypoint(
                    cycle_started,
                    inside_tolerance=goal_decision.goal_reached,
                    visual_health=state.visual.visual_health_score,
                    depth_health=state.depth.depth_health_score,
                )
                if verification.action in {
                    MissionAction.START_WAYPOINT,
                    MissionAction.COMPLETE_MISSION,
                }:
                    summary = self._waypoint_summary(
                        waypoint,
                        attempt,
                        started,
                        cycle_started,
                        start_position,
                        state,
                        goal_decision,
                        "COMPLETED",
                        None,
                        record_start,
                        transition_start,
                        recovery_start,
                        completion_mode=WaypointCompletionMode.POSITION,
                        observation=observation,
                        observation_start=observation_start,
                    )
                    self.waypoint_results.append(summary)
                    self._record_event(MissionEventType.WAYPOINT_REACHED, "mission_orchestrator", f"Waypoint {waypoint.waypoint_id} reached", data={"waypoint_id": waypoint.waypoint_id, "completion_mode": "POSITION"})
                    return {"completed": True, "reason": "Waypoint verified", "hard_failure": False}

            next_tick += 1.0 / self.navigation_config.control_loop_hz
            sleep_time = next_tick - monotonic()
            if sleep_time < 0:
                self.loop_deadline_misses += 1
                next_tick = monotonic()
                sleep_time = 0.0
            await asyncio.sleep(sleep_time)

        summary = self._waypoint_summary(
            waypoint,
            attempt,
            started,
            monotonic(),
            start_position,
            state,
            goal_decision,
            "ABORTED" if hard_failure else "FAILED",
            reason,
            record_start,
            transition_start,
            recovery_start,
            observation=self.last_observation_decision,
            observation_start=observation_start,
        )
        self.waypoint_results.append(summary)
        event = MissionEventType.MISSION_TIMEOUT if "timeout" in reason.lower() else MissionEventType.WAYPOINT_FAILED
        self._record_event(event, "mission_orchestrator", f"Waypoint {waypoint.waypoint_id} failed: {reason}", severity="ERROR" if hard_failure else "WARNING", data={"waypoint_id": waypoint.waypoint_id, "reason": reason, "hard_failure": hard_failure})
        return {"completed": False, "reason": reason, "hard_failure": hard_failure}

    async def _supervised_pause(self, deadline: float) -> bool:
        while monotonic() < deadline:
            now = monotonic()
            state, proximity = snapshot(self.coordinator)
            self._update_mission_path(state)
            report = self.confidence.evaluate(state)
            collision = self.collision_monitor.latest_state()
            requested = DesiredMotion.hover(now, "mission-inter-waypoint-pause")
            avoidance = self.controller.update(
                requested,
                state,
                report.mode.name,
                now,
            )
            safe = self.supervisor.evaluate(
                avoidance,
                state,
                report,
                px4_connected=self.connection.connected,
                autonomy_enabled=True,
                offboard_expected=True,
                displacement_from_start_m=self._mission_displacement(state),
                collision_detected=collision.collision_detected,
                now_monotonic=now,
            )
            avoidance = replace(avoidance, safe_command=safe)
            await self._send(safe)
            self._project_ego_context(
                timestamp=now,
                state=state,
                report=report,
                avoidance=avoidance,
                collision=collision,
            )
            if safe.hard_override:
                self.orchestrator.abort("; ".join(safe.reasons), now)
                return False
            await asyncio.sleep(1.0 / self.navigation_config.control_loop_hz)
        return True

    def _project_ego_context(
        self,
        *,
        timestamp: float | None = None,
        state=None,
        report=None,
        goal=None,
        recovery=None,
        avoidance=None,
        observation=None,
        collision=None,
        finish: bool = False,
    ):
        if self.ego_context is None:
            return None
        if state is None:
            state, _ = snapshot(self.coordinator)
        if report is None:
            report = self.confidence.evaluate(state)
        if goal is None:
            goal = self.last_goal_decision
        if recovery is None:
            recovery = self.last_recovery_decision
        if avoidance is None:
            avoidance = self.last_avoidance_decision
        if observation is None:
            observation = self.last_observation_decision
        if collision is None:
            collision = self.collision_monitor.latest_state()
        projection = RuntimeContextProjection(
            timestamp_monotonic=monotonic() if timestamp is None else timestamp,
            mission_state=self.orchestrator.state,
            current_waypoint=self.orchestrator.current_waypoint,
            ego_state=state,
            confidence=report,
            goal=goal,
            recovery=recovery,
            recovery_state=self.recovery.state,
            avoidance=avoidance,
            observation=observation,
            safe_motion=None if avoidance is None else avoidance.safe_command,
            collision=collision,
            mission_transitions=tuple(self.orchestrator.transitions),
            observation_events=tuple(self.observation_events),
            vlm=self.coordinator.latest_vlm_state(),
            semantic_validation_enabled=self.coordinator.vlm_worker is not None,
        )
        if finish:
            return self.ego_context.finish(projection)
        return self.ego_context.project(projection)

    def _project_terminal_context(self, *, finish: bool) -> None:
        if self.ego_context is not None:
            self._project_ego_context(finish=finish)

    def ego_context_report(self) -> dict | None:
        if self.ego_context is None:
            return None
        context = self.ego_context.snapshot()
        metrics = self.ego_context.metrics()
        validation = context.perception.latest_semantic_validation
        fused = context.perception.fused_scene
        return {
            "instance_count": 1,
            "revision": context.revision,
            "projection_count": metrics.projection_count,
            "event_count": metrics.emitted_event_count,
            "maximum_event_buffer_usage": metrics.maximum_event_buffer_usage,
            "event_categories": dict(metrics.event_categories),
            "duplicate_event_count": metrics.duplicate_event_count,
            "mission_result": context.summary.mission_result,
            "completed_waypoint_count": context.summary.completed_waypoint_count,
            "skipped_waypoint_count": context.summary.skipped_waypoint_count,
            "recovery_count": context.summary.recovery_count,
            "avoidance_count": context.summary.avoidance_count,
            "emergency_confirmed_count": context.summary.emergency_confirmed_count,
            "collision_count": context.summary.collision_count,
            "vlm_status": context.perception.vlm_status,
            "terrain_type": context.perception.terrain_type,
            "target_visible": context.perception.target_visible,
            "scene_confidence": context.perception.scene_confidence,
            "fused_scene": {
                "status": fused.status.value,
                "terrain_type": fused.terrain_type,
                "visible_landmarks": fused.visible_landmarks,
                "target_visible": fused.target_visible,
                "texture_quality": fused.texture_quality,
                "fused_confidence": fused.fused_confidence,
                "evidence_count": fused.evidence_count,
                "positive_target_count": fused.positive_target_count,
                "negative_target_count": fused.negative_target_count,
                "source_frame_range": fused.source_frame_range,
                "first_observation_timestamp_monotonic": (
                    fused.first_observation_timestamp_monotonic
                ),
                "last_observation_timestamp_monotonic": (
                    fused.last_observation_timestamp_monotonic
                ),
                "rationale": fused.rationale,
            },
            "semantic_validation": (
                None
                if validation is None
                else {
                    "waypoint_id": validation.waypoint_id,
                    "expected_target_label": validation.expected_target_label,
                    "deterministic_outcome": validation.deterministic_outcome.value,
                    "vlm_target_visible": validation.vlm_target_visible,
                    "visible_landmarks": validation.visible_landmarks,
                    "scene_confidence": validation.scene_confidence,
                    "result_freshness": validation.result_freshness.value,
                    "evidence_source": validation.evidence_source.value,
                    "classification": validation.classification.value,
                    "rationale": validation.rationale,
                    "timestamp_monotonic": validation.timestamp_monotonic,
                }
            ),
            "landing_state": context.safety.landing_state,
            "disarm_state": context.safety.disarm_state,
            "recent_event_kinds": [
                event.metadata.get("kind", "") for event in context.recent_events[-12:]
            ],
        }

    def _resolve_goal(self, waypoint: MissionWaypoint, state, now: float) -> LocalGoal:
        assert self.mission_origin_ned is not None
        origin_n, origin_e, origin_d = self.mission_origin_ned
        template = waypoint.goal
        return replace(
            template,
            target_north_m=origin_n + template.target_north_m,
            target_east_m=origin_e + template.target_east_m,
            target_down_m=origin_d + template.target_down_m,
            target_yaw_deg=(
                waypoint.preferred_terminal_yaw_deg
                if waypoint.preferred_terminal_yaw_deg is not None
                else template.target_yaw_deg
            ),
            timestamp_monotonic=now,
            source=template.source.replace("MISSION_ORIGIN_NED_OFFSET", "RESOLVED_NED"),
        )

    def _update_mission_path(self, state) -> None:
        if state.last_position_update_monotonic == self._last_mission_position_update:
            return
        current = (state.position_ned_m.x, state.position_ned_m.y)
        if self._last_mission_position is not None:
            self.mission_path_length_m += math.hypot(
                current[0] - self._last_mission_position[0],
                current[1] - self._last_mission_position[1],
            )
        self._last_mission_position = current
        self._last_mission_position_update = state.last_position_update_monotonic

    def _mission_displacement(self, state) -> float:
        assert self.mission_origin_ned is not None
        return math.hypot(
            state.position_ned_m.x - self.mission_origin_ned[0],
            state.position_ned_m.y - self.mission_origin_ned[1],
        )

    def _apply_waypoint_injection(self, waypoint: MissionWaypoint, elapsed: float) -> None:
        collision_after = waypoint.metadata.get("inject_collision_after_s")
        if collision_after is not None and elapsed >= float(collision_after):
            if not self.collision_monitor.latest_state().collision_detected:
                self.collision_monitor.inject_collision("controlled_mission_collision_fixture")

    def _apply_bounded_replan(self, failed: MissionWaypoint) -> bool:
        state, _ = snapshot(self.coordinator)
        assert self.mission_origin_ned is not None
        existing = {item.waypoint_id for item in self.orchestrator.execution_waypoints}
        current_n = state.position_ned_m.x - self.mission_origin_ned[0]
        current_e = state.position_ned_m.y - self.mission_origin_ned[1]
        decision = self.replanner.generate_healthier_viewpoint_staging(
            failed,
            current_north_offset_m=current_n,
            current_east_offset_m=current_e,
            selected_heading_deg=self.last_selected_heading_deg,
            replan_index=self.orchestrator.state.replan_count,
            existing_ids=existing,
        )
        if not decision.accepted and self.last_avoidance_decision is not None:
            decision = self.replanner.generate_lateral_staging(
                failed,
                current_north_offset_m=current_n,
                current_east_offset_m=current_e,
                current_yaw_deg=state.orientation.yaw_deg,
                risk=self.last_avoidance_decision.risk,
                replan_index=self.orchestrator.state.replan_count,
                existing_ids=existing,
            )
        self.replanning_decisions.append(jsonable(asdict(decision)))
        if not decision.accepted or decision.generated_waypoint is None:
            return False
        self.orchestrator.apply_replan(decision.generated_waypoint, monotonic())
        return True

    def _record_cycle(
        self,
        goal,
        recovery,
        avoidance,
        collision,
        state,
        confidence_mode,
        loop_elapsed_ms,
        *,
        transmitted,
        observation=None,
        force=False,
    ) -> None:
        now = goal.timestamp_monotonic
        if not force and now < self._next_artifact_sample:
            return
        self._next_artifact_sample = now + 1.0 / self.mission_config.artifact_sampling_hz
        super()._record_navigation(
            goal,
            recovery,
            avoidance,
            collision,
            state,
            confidence_mode,
            loop_elapsed_ms,
            transmitted=transmitted,
        )
        self.navigation_records[-1].update(
            {
                "mission": self.orchestrator.snapshot(),
                "waypoint_id": self.orchestrator.state.current_waypoint_id,
                "waypoint_index": self.orchestrator.state.current_waypoint_index,
                "waypoint_count": self.orchestrator.state.total_waypoints,
                "observation": (
                    None if observation is None else jsonable(asdict(observation))
                ),
            }
        )

    def _record_control_transitions(self, avoidance, now: float) -> None:
        if avoidance.mode.value != self._last_recorded_avoidance:
            previous = self._last_recorded_avoidance
            self._last_recorded_avoidance = avoidance.mode.value
            self.avoidance_transitions.append(
                {
                    "timestamp_monotonic": now,
                    "waypoint_id": self.orchestrator.state.current_waypoint_id,
                    "mode": avoidance.mode.value,
                }
            )
            if avoidance.mode.value == "CLEAR" and previous is not None:
                self._record_event(MissionEventType.AVOIDANCE_EXITED, "reactive_avoidance", "Avoidance returned to clear")
            elif avoidance.mode.value != "CLEAR":
                self._record_event(MissionEventType.AVOIDANCE_ENTERED, "reactive_avoidance", f"Avoidance entered {avoidance.mode.value}", data={"mode": avoidance.mode.value, "waypoint_id": self.orchestrator.state.current_waypoint_id, "centre_risk": avoidance.risk.centre})
        if avoidance.safe_command.hard_override:
            self._record_event(MissionEventType.SAFETY_HARD_OVERRIDE, "safety_supervisor", "SafetySupervisor issued hard override", severity="ERROR", data={"reasons": list(avoidance.safe_command.reasons), "recommended_action": avoidance.safe_command.recommended_action.value})
        self._last_recorded_supervisor = append_supervisor_transition(
            self.supervisor_transitions,
            self._last_recorded_supervisor,
            avoidance.safe_command,
            self.orchestrator.state.current_waypoint_id,
            now,
        )

    def _waypoint_summary(
        self,
        waypoint,
        attempt,
        started,
        ended,
        start_position,
        state,
        goal,
        outcome,
        failure_reason,
        record_start,
        transition_start,
        recovery_start,
        *,
        completion_mode=None,
        observation=None,
        observation_start=0,
    ) -> dict:
        observation_summary = None
        if observation is not None:
            observation_summary = {
                "state": observation.state.value,
                "completion_distance_m": observation.goal_distance_m,
                "heading_error_deg": observation.heading_error_deg,
                "visual_health": observation.visual_health,
                "depth_health": observation.depth_health,
                "depth_stability": observation.depth_stability,
                "centre_risk": observation.centre_risk,
                "horizontal_speed_m_s": observation.horizontal_speed_m_s,
                "dwell_duration_s": observation.dwell_elapsed_s,
                "fresh_observation_count": observation.fresh_observation_count,
                "wait_elapsed_s": observation.wait_elapsed_s,
                "depth_frame_sequence": observation.depth_frame_sequence,
                "depth_inference_timestamp_monotonic": (
                    observation.depth_inference_timestamp_monotonic
                ),
            }
        return {
            "waypoint_id": waypoint.waypoint_id,
            "generated": waypoint.generated,
            "attempt": attempt,
            "started_at_monotonic": started,
            "ended_at_monotonic": ended,
            "duration_s": ended - started,
            "start_position_ned_m": start_position,
            "end_position_ned_m": asdict(state.position_ned_m),
            "resolved_goal": asdict(goal.goal),
            "final_metrics": asdict(goal.metrics),
            "outcome": outcome,
            "failure_reason": failure_reason,
            "waypoint_type": waypoint.waypoint_type.value,
            "completion_mode": (
                None if completion_mode is None else completion_mode.value
            ),
            "observation": observation_summary,
            "observation_event_count": (
                len(self.observation_events) - observation_start
            ),
            "record_count": len(self.navigation_records) - record_start,
            "transition_count": len(self.navigation_transitions) - transition_start,
            "recovery_count": len(self.recovery_events) - recovery_start,
            "policy_action": "NONE",
        }

    def mission_artifact(self, started_at: str, completed_at: str) -> dict:
        collision = self.collision_monitor.latest_state()
        has_injected = collision.injected_collision_detected
        has_native = collision.native_collision_detected
        evidence = "MIXED" if has_injected and has_native else "INJECTED" if has_injected else "NATIVE" if has_native else "NONE"
        state = self.orchestrator.state
        completed = state.result in {
            MissionResult.COMPLETED,
            MissionResult.COMPLETED_WITH_SKIPS,
        }
        final_result = (
            "NO_FLIGHT_COMPLETED"
            if self.no_flight_completed and self.abort_reason is None
            else "COMPLETED"
            if completed
            and self.landing_result == "CONFIRMED"
            and self.abort_reason is None
            and not collision.collision_detected
            else "ABORTED"
        )
        generated = [
            waypoint
            for waypoint in self.orchestrator.execution_waypoints
            if waypoint.generated
        ]
        return {
            "schema_version": 1,
            "started_at_utc": started_at,
            "completed_at_utc": completed_at,
            "mission_plan": jsonable(asdict(self.mission_plan)),
            "mission_configuration": jsonable(asdict(self.mission_config)),
            "local_navigation_configuration": jsonable(asdict(self.navigation_config)),
            "active_perception_configuration": jsonable(asdict(self.active_config)),
            "avoidance_configuration": jsonable(asdict(self.config)),
            "mission_state": self.orchestrator.snapshot(),
            "mission_transitions": self.orchestrator.transitions,
            "original_waypoints": jsonable([asdict(item) for item in self.mission_plan.waypoints]),
            "generated_waypoints": jsonable([asdict(item) for item in generated]),
            "waypoint_results": self.waypoint_results,
            "replanning_decisions": self.replanning_decisions,
            "navigation_transitions": self.navigation_transitions,
            "avoidance_transitions": self.avoidance_transitions,
            "supervisor_transitions": self.supervisor_transitions,
            "active_perception_recoveries": self.recovery_events,
            "observation_events": self.observation_events,
            "records": self.navigation_records,
            "initial_position_ned_m": self.initial_position,
            "final_position_ned_m": self.final_position,
            "path_length_proxy_m": self.mission_path_length_m,
            "displacement_from_start_m": state.displacement_from_start_m,
            "maximum_speed_m_s": self.maximum_mission_speed_m_s,
            "maximum_command_m_s": self.maximum_command_m_s,
            "maximum_loop_elapsed_ms": self.maximum_loop_elapsed_ms,
            "loop_deadline_misses": self.loop_deadline_misses,
            "collision_monitor_available": collision.available,
            "collision_detected": collision.collision_detected,
            "collision_count": collision.collision_count,
            "collision_evidence_mode": evidence,
            "collision_events": jsonable([asdict(item) for item in collision.events]),
            "mission_result": state.result.value,
            "abort_reason": state.abort_reason or self.abort_reason,
            "landing_result": self.landing_result,
            "final_result": final_result,
            "events": self.event_recorder.finalize(),
            "event_recorder_warnings": self.event_recorder.warnings(),
        }

    def record_event(self, event_type, source: str, message: str, **kwargs) -> None:
        self._record_event(event_type, source, message, **kwargs)

    def _record_event(self, event_type, source: str, message: str, **kwargs) -> None:
        try:
            self.event_recorder.record(event_type, source, message, **kwargs)
        except Exception as exc:
            warning = f"Mission event recorder failed for {event_type}: {exc}"
            self.event_recorder.warn(warning)
            print(warning, file=sys.stderr)

    def _observe_runtime_state(self, state, collision=None) -> None:
        try:
            self.event_recorder.observe_connection(self.connection.connected)
            self.event_recorder.observe_telemetry(
                armed=state.armed,
                in_air=state.in_air,
                flight_mode=state.flight_mode,
                mission_state=self.orchestrator.state.state.value,
                initiated_by_polaris=self._polaris_terminal_landing,
            )
            if collision is not None:
                self.event_recorder.observe_collision(
                    collision.collision_detected,
                    data={"count": collision.collision_count, "entity": collision.latest_contacted_entity},
                )
        except Exception as exc:
            warning = f"Mission event recorder telemetry observation failed: {exc}"
            self.event_recorder.warn(warning)
            print(warning, file=sys.stderr)

    def _mark_terminal_landing(self) -> None:
        self._polaris_terminal_landing = True


async def run_mission_live(
    args,
    mission_plan,
    mission_config,
    navigation_config,
    active_config,
    avoidance_config,
    vlm_config,
) -> dict:
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    depth_config = DepthConfig.from_json(PROJECT_ROOT / "config/depth_anything.json")
    depth_config = replace(
        depth_config,
        device=args.device or depth_config.device,
        target_fps=depth_config.target_fps if args.target_depth_fps is None else args.target_depth_fps,
        depth_required_for_motion=True,
    )
    drone = OwnedMavsdkSystem()
    await drone.connect(system_address=args.system_address)
    service_config = (
        replace(vlm_config, ollama_num_gpu=None)
        if vlm_config is not None and args.no_flight
        else vlm_config
    )
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(PROJECT_ROOT / "config/visual_health.json"),
        analysis_rate_hz=max(10.0, depth_config.target_fps * 2.0),
        depth_config=depth_config,
        vlm_service=None if service_config is None else VLMService(service_config),
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
        maximum_touchdown_speed_m_s=navigation_config.maximum_touchdown_speed_m_s,
        maximum_touchdown_depth_m=navigation_config.maximum_touchdown_depth_m,
    )
    connection.start()
    collision.start()
    started = datetime.now(timezone.utc)
    ego_context_memory = EgoContextMemory()
    ego_context = EgoContextCoordinator(
        ego_context_memory,
        fusion_config=(
            None
            if vlm_config is None
            else SemanticFusionConfig.from_vlm_config(vlm_config)
        ),
    )
    demo = MissionDemo(
        drone,
        coordinator,
        connection,
        avoidance_config,
        confidence,
        display,
        args,
        navigation_config=navigation_config,
        active_config=active_config,
        collision_monitor=collision,
        mission_plan=mission_plan,
        mission_config=mission_config,
        ego_context=ego_context,
    )
    try:
        await coordinator.start()
        await connection.wait()
        demo.record_event(MissionEventType.PX4_CONNECTED, "mavsdk_connection", "PX4 connection confirmed")
        await demo.run()
    except asyncio.CancelledError:
        demo.abort_reason = "Operator interrupted with Ctrl+C"
        demo.orchestrator.abort(demo.abort_reason, monotonic())
        demo._project_terminal_context(finish=True)
    except Exception as exc:
        demo.abort_reason = str(exc)
        demo.orchestrator.abort(demo.abort_reason, monotonic())
        if demo.landing_result == "NOT_ATTEMPTED":
            await asyncio.shield(demo._hover_and_land())
            demo.orchestrator.finish(demo.landing_result, monotonic())
        demo._project_terminal_context(finish=True)
    finally:
        display.close()
        collision.close()
        try:
            stop_timeout = (
                10.0
                if vlm_config is None
                else max(10.0, vlm_config.inference_budget_s + 10.0)
            )
            await asyncio.wait_for(coordinator.stop(), timeout=stop_timeout)
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
    report = demo.ego_context_report()
    if report is not None:
        print("Ego-Context runtime: " + json.dumps(report, separators=(",", ":")))
    return demo.mission_artifact(
        started.isoformat(),
        datetime.now(timezone.utc).isoformat(),
    )


def main() -> int:
    args = parse_args()
    mission_config = MissionOrchestratorConfig.from_json(args.config)
    navigation_config = LocalNavigationConfig.from_json(args.local_navigation_config)
    active_config = ActivePerceptionConfig.from_json(args.active_perception_config)
    avoidance_config = AvoidanceConfig.from_json(args.avoidance_config)
    vlm_config = VLMConfig.from_json(args.vlm_config) if args.vlm else None
    mission_plan = load_mission(
        args.mission,
        PROJECT_ROOT / "config/missions",
        mission_config,
    )
    if args.duration is not None:
        if args.duration <= 0 or args.duration > mission_config.maximum_mission_duration_s:
            raise ValueError("--duration must be positive and within mission configuration")
        mission_plan = replace(mission_plan, maximum_duration_s=args.duration)
    avoidance_config = replace(
        avoidance_config,
        takeoff_altitude_m=mission_config.takeoff_altitude_m,
        final_hover_s=navigation_config.final_hover_s,
        automatic_landing_on_abort=mission_config.automatic_landing,
    )
    print(
        json.dumps(
            {
                "mission": jsonable(asdict(mission_plan)),
                "mission_configuration": jsonable(asdict(mission_config)),
                "inherited_navigation": {
                    "cruise_speed_m_s": navigation_config.cruise_speed_m_s,
                    "horizontal_tolerance_m": navigation_config.horizontal_tolerance_m,
                    "vertical_tolerance_m": navigation_config.vertical_tolerance_m,
                },
                "inherited_recovery": {
                    "yaw_scan_offsets_deg": active_config.yaw_scan_offsets_deg,
                    "maximum_recovery_attempts": active_config.maximum_recovery_attempts,
                    "maximum_total_recovery_yaw_deg": active_config.maximum_total_recovery_yaw_deg,
                },
                "vlm": {
                    "enabled": vlm_config is not None,
                    "model": "DISABLED" if vlm_config is None else vlm_config.model,
                    "inference_interval_s": (
                        None if vlm_config is None else vlm_config.inference_interval_s
                    ),
                },
            },
            indent=2,
        )
    )
    if args.dry_run:
        return 0
    started = datetime.now(timezone.utc)
    payload = asyncio.run(
        run_mission_live(
            args,
            mission_plan,
            mission_config,
            navigation_config,
            active_config,
            avoidance_config,
            vlm_config,
        )
    )
    path = write_mission_artifact(
        payload,
        args.output_dir,
        started.strftime("%Y%m%dT%H%M%SZ"),
    )
    print(f"Mission artifact: {path}")
    print(format_mission_replay(payload, path))
    return 0 if payload["final_result"] in {"COMPLETED", "NO_FLIGHT_COMPLETED"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
