from __future__ import annotations

from dataclasses import asdict

from .models import (
    FailureAction,
    FailurePolicy,
    MissionAction,
    MissionOrchestratorConfig,
    MissionPhase,
    MissionPlan,
    MissionResult,
    MissionState,
    MissionWaypoint,
    WaypointCompletionMode,
    WaypointType,
    OrchestratorDecision,
)


class MissionOrchestrator:
    def __init__(self, config: MissionOrchestratorConfig) -> None:
        self.config = config
        self.plan: MissionPlan | None = None
        self.state = MissionState()
        self.execution_waypoints: list[MissionWaypoint] = []
        self.transitions: list[dict] = []
        self.started_at = 0.0
        self.verification_started_at = 0.0
        self.pause_until = 0.0
        self.stop_after_waypoint: str | None = None

    @property
    def current_waypoint(self) -> MissionWaypoint | None:
        index = self.state.current_waypoint_index
        if 0 <= index < len(self.execution_waypoints):
            return self.execution_waypoints[index]
        return None

    def start(
        self,
        plan: MissionPlan,
        now_monotonic: float,
        *,
        start_waypoint: str | None = None,
        stop_after_waypoint: str | None = None,
    ) -> None:
        plan.validate(self.config)
        self.plan = plan
        self.execution_waypoints = list(plan.waypoints)
        start_index = 0
        if start_waypoint is not None:
            matches = [
                index
                for index, waypoint in enumerate(self.execution_waypoints)
                if waypoint.waypoint_id == start_waypoint
            ]
            if not matches:
                raise ValueError(f"Unknown start waypoint: {start_waypoint}")
            start_index = matches[0]
            self.execution_waypoints = self.execution_waypoints[start_index:]
        if stop_after_waypoint is not None and not any(
            waypoint.waypoint_id == stop_after_waypoint
            for waypoint in self.execution_waypoints
        ):
            raise ValueError(f"Unknown stop waypoint: {stop_after_waypoint}")
        self.stop_after_waypoint = stop_after_waypoint
        self.state = MissionState(
            mission_id=plan.mission_id,
            mission_name=plan.name,
            state=MissionPhase.PREFLIGHT,
            current_waypoint_index=0,
            current_waypoint_id=self.execution_waypoints[0].waypoint_id,
            total_waypoints=len(self.execution_waypoints),
            active_failure_policy=plan.failure_policy.value,
        )
        self.started_at = now_monotonic
        self.transitions.clear()
        self._transition(MissionPhase.PREFLIGHT, "Mission loaded", now_monotonic)

    def mark_takeoff(self, now_monotonic: float) -> None:
        self._transition(MissionPhase.TAKEOFF, "Preflight passed", now_monotonic)

    def mark_mission_start(self, now_monotonic: float) -> OrchestratorDecision:
        self.started_at = now_monotonic
        self._transition(MissionPhase.MISSION_START, "Takeoff confirmed", now_monotonic)
        return self.prepare_waypoint(now_monotonic)

    def begin_return_or_land(self, now_monotonic: float) -> None:
        self._transition(
            MissionPhase.RETURN_OR_LAND,
            "Mission execution ended; applying terminal policy",
            now_monotonic,
        )

    def prepare_waypoint(self, now_monotonic: float) -> OrchestratorDecision:
        waypoint = self.current_waypoint
        if waypoint is None:
            return self._complete(now_monotonic, "All waypoints resolved")
        self.state.current_waypoint_id = waypoint.waypoint_id
        self.state.current_waypoint_type = waypoint.waypoint_type.value
        self.state.current_completion_mode = "NONE"
        self.state.active_failure_policy = self._failure_action(waypoint).value
        self._transition(MissionPhase.WAYPOINT_PREPARE, "Preparing waypoint", now_monotonic)
        return OrchestratorDecision(
            MissionAction.START_WAYPOINT,
            self.state.state,
            "Waypoint ready",
            waypoint,
        )

    def begin_waypoint(self, now_monotonic: float) -> OrchestratorDecision:
        waypoint = self.current_waypoint
        if waypoint is None:
            return self._complete(now_monotonic, "No waypoint remains")
        attempts = self.state.waypoint_attempts.get(waypoint.waypoint_id, 0) + 1
        self.state.waypoint_attempts[waypoint.waypoint_id] = attempts
        self.state.current_attempt = attempts
        self._transition(MissionPhase.WAYPOINT_EXECUTE, "Waypoint execution started", now_monotonic)
        return OrchestratorDecision(
            MissionAction.NONE,
            self.state.state,
            "Executing waypoint",
            waypoint,
        )

    def begin_verification(self, now_monotonic: float) -> None:
        self.verification_started_at = now_monotonic
        self._transition(MissionPhase.WAYPOINT_VERIFY, "Local goal reached", now_monotonic)

    def verify_waypoint(
        self,
        now_monotonic: float,
        *,
        inside_tolerance: bool,
        visual_health: float,
        depth_health: float,
    ) -> OrchestratorDecision:
        waypoint = self.current_waypoint
        if waypoint is None:
            return self._abort("Verification has no current waypoint", now_monotonic)
        if not inside_tolerance:
            self.verification_started_at = now_monotonic
            return OrchestratorDecision(
                MissionAction.NONE,
                self.state.state,
                "Waypoint left tolerance during verification",
                waypoint,
            )
        if visual_health < waypoint.minimum_visual_health:
            return OrchestratorDecision(
                MissionAction.NONE,
                self.state.state,
                "Waypoint visual-health requirement not met",
                waypoint,
            )
        if depth_health < waypoint.minimum_depth_health:
            return OrchestratorDecision(
                MissionAction.NONE,
                self.state.state,
                "Waypoint depth-health requirement not met",
                waypoint,
            )
        if now_monotonic - self.verification_started_at < waypoint.verification_hold_s:
            return OrchestratorDecision(
                MissionAction.NONE,
                self.state.state,
                "Holding waypoint verification",
                waypoint,
            )
        return self.complete_waypoint(now_monotonic)

    def complete_waypoint(
        self,
        now_monotonic: float,
        *,
        completion_mode: WaypointCompletionMode = WaypointCompletionMode.POSITION,
        reason: str = "Waypoint verified",
    ) -> OrchestratorDecision:
        waypoint = self.current_waypoint
        if waypoint is None:
            return self._abort("Completion has no current waypoint", now_monotonic)
        if waypoint.waypoint_id not in self.state.completed_waypoints:
            self.state.completed_waypoints.append(waypoint.waypoint_id)
        self.state.waypoint_completion_modes[waypoint.waypoint_id] = completion_mode.value
        self.state.current_completion_mode = completion_mode.value
        self._transition(MissionPhase.WAYPOINT_COMPLETE, reason, now_monotonic)
        if self.stop_after_waypoint == waypoint.waypoint_id:
            return self._complete(now_monotonic, "Configured stop waypoint completed")
        self.state.current_waypoint_index += 1
        if self.state.current_waypoint_index >= len(self.execution_waypoints):
            return self._complete(now_monotonic, "All waypoints completed")
        self.pause_until = now_monotonic + self.config.pause_between_waypoints_s
        self.state.current_waypoint_id = self.execution_waypoints[
            self.state.current_waypoint_index
        ].waypoint_id
        self.state.current_waypoint_type = self.current_waypoint.waypoint_type.value
        self.state.current_completion_mode = "NONE"
        return OrchestratorDecision(
            MissionAction.START_WAYPOINT,
            self.state.state,
            "Advance after inter-waypoint pause",
            self.current_waypoint,
        )

    def fail_waypoint(
        self,
        reason: str,
        now_monotonic: float,
        *,
        hard_failure: bool = False,
    ) -> OrchestratorDecision:
        waypoint = self.current_waypoint
        if waypoint is None:
            return self._abort(reason, now_monotonic)
        if waypoint.waypoint_id not in self.state.failed_waypoints:
            self.state.failed_waypoints.append(waypoint.waypoint_id)
        self._transition(MissionPhase.WAYPOINT_FAILED, reason, now_monotonic)
        if hard_failure:
            return self._abort(reason, now_monotonic)
        action = self._failure_action(waypoint)
        attempts = self.state.waypoint_attempts.get(waypoint.waypoint_id, 0)
        if action is FailureAction.RETRY_WAYPOINT and attempts < waypoint.maximum_attempts:
            self._transition(MissionPhase.WAYPOINT_PREPARE, "Retrying waypoint", now_monotonic)
            return OrchestratorDecision(
                MissionAction.RETRY_WAYPOINT,
                self.state.state,
                reason,
                waypoint,
            )
        if action is FailureAction.SKIP_OPTIONAL and waypoint.skippable and not waypoint.required:
            self.state.skipped_waypoints.append(waypoint.waypoint_id)
            self.state.current_waypoint_index += 1
            if self.state.current_waypoint_index >= len(self.execution_waypoints):
                return self._complete(now_monotonic, "Optional waypoint skipped")
            self.state.current_waypoint_id = self.current_waypoint.waypoint_id
            return OrchestratorDecision(
                MissionAction.SKIP_WAYPOINT,
                self.state.state,
                reason,
                self.current_waypoint,
            )
        if action is FailureAction.REPLAN_AROUND_WAYPOINT:
            assert self.plan is not None
            if self.state.replan_count < min(self.plan.maximum_replans, self.config.maximum_replans):
                self._transition(MissionPhase.REPLAN, "Bounded replan requested", now_monotonic)
                return OrchestratorDecision(
                    MissionAction.REQUEST_REPLAN,
                    self.state.state,
                    reason,
                    waypoint,
                )
        return self._abort(reason, now_monotonic)

    def apply_replan(
        self,
        generated: MissionWaypoint,
        now_monotonic: float,
    ) -> OrchestratorDecision:
        generated.validate(self.config.world_bounds)
        existing = {waypoint.waypoint_id for waypoint in self.execution_waypoints}
        if generated.waypoint_id in existing:
            return self._abort("Generated waypoint ID is not unique", now_monotonic)
        self.execution_waypoints.insert(self.state.current_waypoint_index, generated)
        self.state.total_waypoints += 1
        self.state.replan_count += 1
        self.state.generated_waypoints.append(generated.waypoint_id)
        self.state.generated_replan_waypoint = generated.waypoint_id
        self.state.current_waypoint_id = generated.waypoint_id
        self._transition(MissionPhase.WAYPOINT_PREPARE, "Generated waypoint inserted", now_monotonic)
        return OrchestratorDecision(
            MissionAction.START_WAYPOINT,
            self.state.state,
            "Execute generated staging waypoint",
            generated,
        )

    def update_metrics(
        self,
        now_monotonic: float,
        *,
        path_length_proxy_m: float,
        displacement_from_start_m: float,
        current_goal_distance_m: float,
    ) -> OrchestratorDecision | None:
        self.state.mission_elapsed_s = max(0.0, now_monotonic - self.started_at)
        self.state.total_path_length_proxy_m = path_length_proxy_m
        self.state.displacement_from_start_m = displacement_from_start_m
        self.state.current_goal_distance_m = current_goal_distance_m
        assert self.plan is not None
        if self.state.mission_elapsed_s > self.plan.maximum_duration_s:
            return self._abort("Mission duration limit exceeded", now_monotonic)
        if path_length_proxy_m > self.plan.maximum_path_length_m:
            return self._abort("Mission path-length limit exceeded", now_monotonic)
        return None

    def abort(self, reason: str, now_monotonic: float) -> OrchestratorDecision:
        return self._abort(reason, now_monotonic)

    def mark_landing(self, result: str, now_monotonic: float) -> None:
        self.state.landing_result = result
        self._transition(MissionPhase.LAND, "Landing attempted", now_monotonic)

    def finish(self, landing_result: str, now_monotonic: float) -> None:
        self.state.landing_result = landing_result
        if landing_result == "CONFIRMED" and self.state.result in {
            MissionResult.COMPLETED,
            MissionResult.COMPLETED_WITH_SKIPS,
        }:
            pass
        elif landing_result == "CONFIRMED" and self.state.result is MissionResult.PENDING:
            self.state.result = MissionResult.LANDED
        elif self.state.result not in {MissionResult.ABORTED, MissionResult.FAILED}:
            self.state.result = MissionResult.FAILED
        self._transition(MissionPhase.FINISHED, "Mission runtime finished", now_monotonic)

    def snapshot(self) -> dict:
        result = asdict(self.state)
        result["state"] = self.state.state.value
        result["result"] = self.state.result.value
        result["progress_percent"] = self.state.progress_percent
        return result

    def _failure_action(self, waypoint: MissionWaypoint) -> FailureAction:
        if waypoint.failure_action is not FailureAction.USE_MISSION_POLICY:
            return waypoint.failure_action
        assert self.plan is not None
        return FailureAction(self.plan.failure_policy.value)

    def _complete(self, now: float, reason: str) -> OrchestratorDecision:
        self.state.result = (
            MissionResult.COMPLETED_WITH_SKIPS
            if self.state.skipped_waypoints
            else MissionResult.COMPLETED
        )
        self._transition(MissionPhase.MISSION_COMPLETE, reason, now)
        return OrchestratorDecision(
            MissionAction.COMPLETE_MISSION,
            self.state.state,
            reason,
        )

    def _abort(self, reason: str, now: float) -> OrchestratorDecision:
        self.state.abort_reason = reason
        self.state.result = MissionResult.ABORTED
        self._transition(MissionPhase.ABORT, reason, now)
        return OrchestratorDecision(
            MissionAction.ABORT_MISSION,
            self.state.state,
            reason,
            self.current_waypoint,
        )

    def _transition(self, state: MissionPhase, reason: str, now: float) -> None:
        self.state.state = state
        self.transitions.append(
            {
                "timestamp_monotonic": now,
                "state": state.value,
                "waypoint_id": self.state.current_waypoint_id,
                "reason": reason,
            }
        )
