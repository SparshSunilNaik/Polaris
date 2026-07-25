from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from typing import Any, Mapping

from polaris.control.models import AvoidanceDecision, SafeMotion
from polaris.ego_state.confidence import ConfidenceReport
from polaris.ego_state.models import NavigationState
from polaris.mission.models import (
    MissionPlan,
    MissionState,
    MissionWaypoint,
    ObservationDecision,
)
from polaris.navigation.models import GoalDecision, RecoveryDecision, RecoveryState
from polaris.simulation.collision_monitor import CollisionState
from polaris.vlm import (
    FusedSceneContext,
    FusedSceneStatus,
    SemanticFusion,
    SemanticFusionConfig,
    VLMSceneState,
)

from .memory import EgoContextMemory
from .models import (
    EgoContextSnapshot,
    MissionContext,
    MissionSummary,
    NavigationContext,
    PerceptionContext,
    SafetyContext,
)
from .semantic_validation import (
    DeterministicObservationOutcome,
    SemanticObservationValidator,
)


@dataclass(frozen=True, slots=True)
class RuntimeContextProjection:
    timestamp_monotonic: float
    mission_state: MissionState
    ego_state: NavigationState
    confidence: ConfidenceReport
    current_waypoint: MissionWaypoint | None = None
    goal: GoalDecision | None = None
    recovery: RecoveryDecision | None = None
    recovery_state: RecoveryState = RecoveryState.IDLE
    avoidance: AvoidanceDecision | None = None
    observation: ObservationDecision | None = None
    safe_motion: SafeMotion | None = None
    collision: CollisionState | None = None
    mission_transitions: tuple[Mapping[str, Any], ...] = ()
    observation_events: tuple[Mapping[str, Any], ...] = ()
    vlm: VLMSceneState | None = None
    semantic_validation_enabled: bool = False


@dataclass(frozen=True, slots=True)
class EgoContextCoordinatorMetrics:
    projection_count: int
    emitted_event_count: int
    maximum_event_buffer_usage: int
    event_categories: tuple[tuple[str, int], ...]
    duplicate_event_count: int


class EgoContextCoordinator:
    """One-way semantic projection from authoritative runtime decisions."""

    _ACTIVE_AVOIDANCE = {"AVOID_LEFT", "AVOID_RIGHT", "RETREAT", "HOVER"}
    _ACTIVE_RECOVERY = {"TRIGGERED", "SCANNING", "SAMPLING", "SELECTING", "RETURNING"}

    def __init__(
        self,
        memory: EgoContextMemory | None = None,
        fusion_config: SemanticFusionConfig | None = None,
    ) -> None:
        self.memory = memory or EgoContextMemory()
        self._semantic_validator = SemanticObservationValidator()
        self._fusion = SemanticFusion(fusion_config)
        self._fused_scene = self._fusion.snapshot()
        self._reset_edges()

    def snapshot(self) -> EgoContextSnapshot:
        return self.memory.snapshot()

    def metrics(self) -> EgoContextCoordinatorMetrics:
        return EgoContextCoordinatorMetrics(
            projection_count=self._projection_count,
            emitted_event_count=self._emitted_event_count,
            maximum_event_buffer_usage=self._maximum_event_buffer_usage,
            event_categories=tuple(sorted(self._event_categories.items())),
            duplicate_event_count=0,
        )

    def reset(self) -> EgoContextSnapshot:
        self._fused_scene = self._fusion.reset()
        self._reset_edges()
        return self.memory.reset()

    def start_mission(
        self,
        plan: MissionPlan,
        state: MissionState,
        timestamp_monotonic: float,
    ) -> EgoContextSnapshot:
        self.reset()
        self._fused_scene = self._fusion.reset(timestamp_monotonic)
        self._plan = plan
        self._known_completed = set(state.completed_waypoints)
        self._known_skipped = set(state.skipped_waypoints)
        self._known_failed = set(state.failed_waypoints)
        self._previous_phase = state.state.value
        self._previous_result = state.result.value
        waypoint = plan.waypoints[0] if plan.waypoints else None
        self.memory.update_mission(
            timestamp_monotonic=timestamp_monotonic,
            **self._mission_values(state, waypoint, None),
        )
        self.memory.update_summary(
            timestamp_monotonic=timestamp_monotonic,
            mission_result=state.result.value,
        )
        self._emit(
            timestamp_monotonic,
            "MISSION",
            "MISSION_STARTED",
            f"Mission {plan.name} started",
            {"mission_id": plan.mission_id},
        )
        return self.snapshot()

    def project(self, projection: RuntimeContextProjection) -> EgoContextSnapshot:
        timestamp = projection.timestamp_monotonic
        self._projection_count += 1
        self._fused_scene = self._fusion.update(projection.vlm, timestamp)
        self._consume_explicit_events(projection)
        self._detect_edges(projection)

        owner, strategy = self._ownership(projection)
        mission = MissionContext(**self._mission_values(
            projection.mission_state,
            projection.current_waypoint,
            projection.observation,
        ))
        navigation = self._navigation_context(projection, owner, strategy)
        perception = self._perception_context(projection)
        safety = self._safety_context(projection, owner)
        summary = MissionSummary(
            recovery_count=self._recovery_count,
            avoidance_count=self._avoidance_count,
            observations_completed=self._observations_completed,
            mission_confidence=projection.confidence.overall_health,
            elapsed_mission_s=projection.mission_state.mission_elapsed_s,
            completed_waypoint_count=len(projection.mission_state.completed_waypoints),
            skipped_waypoint_count=len(projection.mission_state.skipped_waypoints),
            emergency_suspected_count=self._emergency_suspected_count,
            emergency_cleared_count=self._emergency_cleared_count,
            emergency_confirmed_count=self._emergency_confirmed_count,
            collision_count=(
                0 if projection.collision is None else projection.collision.collision_count
            ),
            replan_count=projection.mission_state.replan_count,
            mission_result=projection.mission_state.result.value,
        )
        self.memory.update_mission(timestamp_monotonic=timestamp, **asdict(mission))
        self.memory.update_navigation(timestamp_monotonic=timestamp, **asdict(navigation))
        self.memory.update_perception(
            timestamp_monotonic=timestamp,
            **{item.name: getattr(perception, item.name) for item in fields(perception)},
        )
        self.memory.update_safety(timestamp_monotonic=timestamp, **asdict(safety))
        self.memory.update_summary(timestamp_monotonic=timestamp, **asdict(summary))
        return self.snapshot()

    def finish(self, projection: RuntimeContextProjection) -> EgoContextSnapshot:
        snapshot = self.project(projection)
        if not self._mission_finished_recorded:
            self._emit(
                projection.timestamp_monotonic,
                "MISSION",
                "MISSION_FINISHED",
                f"Mission finished with {projection.mission_state.result.value}",
                {"result": projection.mission_state.result.value},
            )
            self._mission_finished_recorded = True
            snapshot = self.snapshot()
        return snapshot

    def _mission_values(
        self,
        state: MissionState,
        waypoint: MissionWaypoint | None,
        observation: ObservationDecision | None,
    ) -> dict[str, Any]:
        waypoint_type = state.current_waypoint_type
        if waypoint is not None:
            waypoint_type = waypoint.waypoint_type.value
        objective = (
            f"Observe {state.current_waypoint_id}"
            if waypoint_type == "OBSERVATION"
            else f"Reach {state.current_waypoint_id}"
            if state.current_waypoint_id != "NONE"
            else "NONE"
        )
        return {
            "mission_id": state.mission_id,
            "mission_name": state.mission_name,
            "phase": state.state.value,
            "result": state.result.value,
            "current_objective": objective,
            "current_waypoint_id": state.current_waypoint_id,
            "current_waypoint_index": state.current_waypoint_index,
            "total_waypoints": state.total_waypoints,
            "current_attempt": state.current_attempt,
            "waypoint_type": waypoint_type,
            "completion_mode": state.current_completion_mode,
            "completed_waypoints": tuple(state.completed_waypoints),
            "skipped_waypoints": tuple(state.skipped_waypoints),
            "failed_waypoints": tuple(state.failed_waypoints),
            "observation_status": (
                "NOT_APPLICABLE" if observation is None else observation.state.value
            ),
        }

    def _navigation_context(
        self,
        projection: RuntimeContextProjection,
        owner: str,
        strategy: str,
    ) -> NavigationContext:
        goal = projection.goal
        recovery_state = (
            projection.recovery.state.value
            if projection.recovery is not None
            else projection.recovery_state.value
        )
        avoidance_state = (
            "CLEAR" if projection.avoidance is None else projection.avoidance.mode.value
        )
        return NavigationContext(
            navigator_state="IDLE" if goal is None else goal.state.value,
            recovery_state=recovery_state,
            active_perception_owns_motion=owner.startswith("ACTIVE_PERCEPTION"),
            strategy=strategy,
            goal_distance_m=None if goal is None else goal.metrics.goal_distance_m,
            heading_error_deg=None if goal is None else goal.metrics.yaw_error_deg,
            avoidance_state=avoidance_state,
            avoidance_episode_count=self._avoidance_count,
            recovery_count=self._recovery_count,
            no_progress=False if goal is None else goal.recovery_requested,
            retry_count=max(0, projection.mission_state.current_attempt - 1),
            recent_recoveries=tuple(self._recent_recoveries),
        )

    def _perception_context(
        self,
        projection: RuntimeContextProjection,
    ) -> PerceptionContext:
        visual = projection.ego_state.visual
        depth = projection.ego_state.depth
        conditions = [visual.visual_condition.value]
        if depth.available or projection.ego_state.depth_required_for_motion:
            conditions.append(depth.condition.value)
        severity = {"UNKNOWN": 0, "HEALTHY": 1, "DEGRADED": 2, "CRITICAL": 3}
        condition = max(conditions, key=lambda item: severity.get(item, 3))
        freshness = (
            "STALE"
            if visual.stale or (projection.ego_state.depth_required_for_motion and depth.stale)
            else "FRESH"
        )
        centre_risk = (
            0.0 if projection.avoidance is None else projection.avoidance.risk.centre
        )
        vlm = projection.vlm
        vlm_result = None if vlm is None or not vlm.fresh else vlm.result
        return PerceptionContext(
            visual_health=visual.visual_health_score,
            depth_health=depth.depth_health_score,
            depth_stability=depth.temporal_stability,
            condition=condition,
            visual_condition=visual.visual_condition.value,
            depth_condition=depth.condition.value,
            freshness_state=freshness,
            centre_risk=centre_risk,
            vlm_status="DISABLED" if vlm is None else vlm.status.value,
            terrain_type="UNKNOWN" if vlm_result is None else vlm_result.terrain_type,
            visible_landmarks=() if vlm_result is None else vlm_result.visible_landmarks,
            target_visible=False if vlm_result is None else vlm_result.target_visible,
            scene_confidence=0.0 if vlm_result is None else vlm_result.scene_confidence,
            texture_quality=0.0 if vlm_result is None else vlm_result.texture_quality,
            scene_summary=(
                "Scene understanding disabled"
                if vlm is None
                else "Fresh scene understanding unavailable"
                if vlm_result is None
                else vlm_result.scene_summary
            ),
            vlm_model="NONE" if vlm is None else vlm.model,
            vlm_timestamp_monotonic=(
                0.0 if vlm is None else vlm.inference_timestamp_monotonic
            ),
            vlm_source_frame_sequence=(
                0 if vlm is None else vlm.source_frame_sequence
            ),
            fused_scene=self._fused_scene,
            latest_semantic_validation=self._latest_semantic_validation,
        )

    def _safety_context(
        self,
        projection: RuntimeContextProjection,
        owner: str,
    ) -> SafetyContext:
        safe = projection.safe_motion
        collision = projection.collision
        reasons = () if safe is None else safe.reasons
        landing = projection.mission_state.landing_result
        return SafetyContext(
            supervisor_state="PENDING" if safe is None else safe.supervisor_mode.value,
            emergency_state=(
                "CLEAR"
                if safe is None
                else safe.emergency_confirmation.state.value
            ),
            collision_detected=False if collision is None else collision.collision_detected,
            hard_override=False if safe is None else safe.hard_override,
            current_ownership=owner,
            collision_count=0 if collision is None else collision.collision_count,
            last_safety_reason=reasons[0] if reasons else "NONE",
            landing_state=landing,
            disarm_state=(
                "CONFIRMED"
                if landing == "CONFIRMED" and not projection.ego_state.armed
                else "ARMED"
                if projection.ego_state.armed
                else "UNKNOWN"
            ),
        )

    def _ownership(self, projection: RuntimeContextProjection) -> tuple[str, str]:
        if projection.safe_motion is not None and projection.safe_motion.hard_override:
            return "SAFETY_SUPERVISOR", "SAFETY_OVERRIDE"
        if projection.observation is not None and projection.observation.hold_required:
            return "ACTIVE_PERCEPTION_OBSERVATION", "ACTIVE_PERCEPTION_OBSERVATION"
        recovery_state = (
            projection.recovery.state.value
            if projection.recovery is not None
            else projection.recovery_state.value
        )
        if recovery_state in self._ACTIVE_RECOVERY:
            return "ACTIVE_PERCEPTION_RECOVERY", "ACTIVE_PERCEPTION_RECOVERY"
        if (
            projection.avoidance is not None
            and projection.avoidance.mode.value in self._ACTIVE_AVOIDANCE
        ):
            return "REACTIVE_AVOIDANCE", "REACTIVE_AVOIDANCE"
        if projection.goal is not None:
            return "LOCAL_GOAL", "LOCAL_GOAL"
        return "MISSION", "MISSION_HOLD"

    def _consume_explicit_events(self, projection: RuntimeContextProjection) -> None:
        timestamp = projection.timestamp_monotonic
        transitions = projection.mission_transitions
        for item in transitions[self._mission_transition_cursor :]:
            phase = str(item.get("state", "UNKNOWN"))
            waypoint = str(item.get("waypoint_id", "NONE"))
            reason = str(item.get("reason", ""))
            if phase == "WAYPOINT_EXECUTE":
                self._emit(timestamp, "WAYPOINT", "WAYPOINT_STARTED", f"Waypoint {waypoint} started", {"waypoint_id": waypoint})
            elif phase == "WAYPOINT_PREPARE" and "retry" in reason.lower():
                self._emit(timestamp, "WAYPOINT", "WAYPOINT_RETRY", f"Waypoint {waypoint} retry", {"waypoint_id": waypoint})
            elif phase == "REPLAN":
                self._emit(timestamp, "MISSION", "REPLAN_REQUESTED", f"Replan requested for {waypoint}", {"waypoint_id": waypoint})
            elif phase == "RETURN_OR_LAND" and not self._landing_started:
                self._landing_started = True
                self._emit(timestamp, "SAFETY", "LANDING_STARTED", "Controlled landing started")
        self._mission_transition_cursor = len(transitions)

        events = projection.observation_events
        for item in events[self._observation_event_cursor :]:
            kind = str(item.get("event_type", "UNKNOWN"))
            waypoint = str(item.get("waypoint_id", "NONE"))
            self._emit(
                timestamp,
                "OBSERVATION",
                f"OBSERVATION_{kind}",
                str(item.get("reason", f"Observation {kind.lower()}")),
                {"waypoint_id": waypoint},
            )
        self._observation_event_cursor = len(events)

    def _detect_edges(self, projection: RuntimeContextProjection) -> None:
        timestamp = projection.timestamp_monotonic
        state = projection.mission_state

        for waypoint in state.completed_waypoints:
            if waypoint not in self._known_completed:
                mode = state.waypoint_completion_modes.get(waypoint, "POSITION")
                self._emit(
                    timestamp,
                    "WAYPOINT",
                    "WAYPOINT_COMPLETED",
                    f"Waypoint {waypoint} completed by {mode}",
                    {"waypoint_id": waypoint, "completion_mode": mode},
                )
                if mode == "OBSERVATION":
                    self._observations_completed += 1
                    self._record_semantic_validation(
                        projection,
                        waypoint,
                        DeterministicObservationOutcome.COMPLETED,
                    )
        self._known_completed.update(state.completed_waypoints)
        for waypoint in state.skipped_waypoints:
            if waypoint not in self._known_skipped:
                self._emit(timestamp, "WAYPOINT", "WAYPOINT_SKIPPED", f"Optional waypoint {waypoint} skipped", {"waypoint_id": waypoint})
        self._known_skipped.update(state.skipped_waypoints)
        for waypoint in state.failed_waypoints:
            if waypoint not in self._known_failed:
                self._emit(timestamp, "WAYPOINT", "WAYPOINT_FAILED", f"Waypoint {waypoint} failed", {"waypoint_id": waypoint})
                self._record_semantic_validation(
                    projection,
                    waypoint,
                    DeterministicObservationOutcome.FAILED,
                )
        self._known_failed.update(state.failed_waypoints)

        phase = state.state.value
        if phase != self._previous_phase:
            if phase == "ABORT":
                self._emit(timestamp, "MISSION", "MISSION_ABORTED", f"Mission aborted: {state.abort_reason or 'Unknown reason'}")
            elif phase == "MISSION_COMPLETE":
                self._emit(timestamp, "MISSION", "MISSION_COMPLETED", "Mission objectives completed")
            elif phase == "RETURN_OR_LAND" and not self._landing_started:
                self._landing_started = True
                self._emit(timestamp, "SAFETY", "LANDING_STARTED", "Controlled landing started")
        self._previous_phase = phase
        self._previous_result = state.result.value

        nav_state = None if projection.goal is None else projection.goal.state.value
        if self._previous_navigator_state is not None and nav_state != self._previous_navigator_state:
            self._emit(timestamp, "NAVIGATION", "NAVIGATOR_CHANGED", f"Navigator entered {nav_state}")
        self._previous_navigator_state = nav_state

        avoidance_mode = None if projection.avoidance is None else projection.avoidance.mode.value
        if avoidance_mode != self._previous_avoidance_mode:
            if avoidance_mode in self._ACTIVE_AVOIDANCE:
                self._avoidance_count += 1
                self._emit(timestamp, "AVOIDANCE", "AVOIDANCE_ENTERED", f"Avoidance entered {avoidance_mode}")
            elif self._previous_avoidance_mode in self._ACTIVE_AVOIDANCE:
                self._emit(timestamp, "AVOIDANCE", "AVOIDANCE_CLEARED", "Avoidance cleared")
        self._previous_avoidance_mode = avoidance_mode

        recovery_state = (
            projection.recovery.state.value
            if projection.recovery is not None
            else projection.recovery_state.value
        )
        previous_active = self._previous_recovery_state in self._ACTIVE_RECOVERY
        active = recovery_state in self._ACTIVE_RECOVERY
        if active and not previous_active:
            self._recovery_count += 1
            self._emit(timestamp, "RECOVERY", "RECOVERY_STARTED", "Recovery started")
            self._emit(timestamp, "ACTIVE_PERCEPTION", "ACTIVE_PERCEPTION_STARTED", "Active perception started")
        if recovery_state in {"COMPLETE", "FAILED", "ABORT"} and recovery_state != self._previous_recovery_state:
            suffix = "COMPLETED" if recovery_state == "COMPLETE" else "FAILED"
            self._emit(timestamp, "RECOVERY", f"RECOVERY_{suffix}", f"Recovery {recovery_state.lower()}")
            self._emit(timestamp, "ACTIVE_PERCEPTION", f"ACTIVE_PERCEPTION_{suffix}", f"Active perception {recovery_state.lower()}")
            self._recent_recoveries.append(recovery_state)
            self._recent_recoveries = self._recent_recoveries[-5:]
        self._previous_recovery_state = recovery_state

        visual_condition = projection.ego_state.visual.visual_condition.value
        depth_condition = projection.ego_state.depth.condition.value
        depth_stale = projection.ego_state.depth.stale
        if self._previous_visual_condition is not None and visual_condition != self._previous_visual_condition:
            kind = "PERCEPTION_RECOVERED" if visual_condition == "HEALTHY" else "PERCEPTION_UNHEALTHY"
            self._emit(timestamp, "PERCEPTION", kind, f"Visual condition became {visual_condition}")
        if self._previous_depth_condition is not None and depth_condition != self._previous_depth_condition:
            kind = "PERCEPTION_RECOVERED" if depth_condition == "HEALTHY" else "PERCEPTION_UNHEALTHY"
            self._emit(timestamp, "PERCEPTION", kind, f"Depth condition became {depth_condition}")
        if self._previous_depth_stale is not None and depth_stale != self._previous_depth_stale:
            kind = "DEPTH_STALE" if depth_stale else "DEPTH_RECOVERED"
            self._emit(timestamp, "PERCEPTION", kind, "Depth became stale" if depth_stale else "Depth recovered")
        self._previous_visual_condition = visual_condition
        self._previous_depth_condition = depth_condition
        self._previous_depth_stale = depth_stale

        fused_signature = self._fused_scene.meaningful_signature
        if (
            self._fused_scene.status
            in {FusedSceneStatus.STABLE, FusedSceneStatus.CONFLICTED, FusedSceneStatus.STALE}
            and fused_signature != self._previous_fused_scene_signature
        ):
            self._emit(
                timestamp,
                "SCENE_FUSION",
                f"FUSED_SCENE_{self._fused_scene.status.value}",
                self._fused_scene.rationale,
                {
                    "status": self._fused_scene.status.value,
                    "terrain_type": self._fused_scene.terrain_type,
                    "visible_landmarks": self._fused_scene.visible_landmarks,
                    "target_visible": self._fused_scene.target_visible,
                    "evidence_count": self._fused_scene.evidence_count,
                    "source_frame_range": self._fused_scene.source_frame_range,
                },
            )
            self._previous_fused_scene_signature = fused_signature

        safe = projection.safe_motion
        emergency = "CLEAR" if safe is None else safe.emergency_confirmation.state.value
        if self._previous_emergency_state is not None and emergency != self._previous_emergency_state:
            kind = (
                "EMERGENCY_CLEARED"
                if emergency == "CLEAR"
                else f"EMERGENCY_{emergency}"
            )
            self._emit(timestamp, "SAFETY", kind, f"Emergency confirmation became {emergency}")
            if emergency == "SUSPECTED":
                self._emergency_suspected_count += 1
            elif emergency == "CLEAR":
                self._emergency_cleared_count += 1
            elif emergency == "CONFIRMED":
                self._emergency_confirmed_count += 1
        self._previous_emergency_state = emergency

        supervisor_mode = None if safe is None else safe.supervisor_mode.value
        if self._previous_supervisor_mode is not None and supervisor_mode != self._previous_supervisor_mode:
            self._emit(timestamp, "SAFETY", "SAFETY_CHANGED", f"Safety supervisor entered {supervisor_mode}")
        self._previous_supervisor_mode = supervisor_mode
        hard = False if safe is None else safe.hard_override
        if hard and not self._previous_hard_override:
            self._emit(timestamp, "SAFETY", "HARD_ABORT", f"Hard safety override: {'; '.join(safe.reasons)}")
        self._previous_hard_override = hard

        collision_count = 0 if projection.collision is None else projection.collision.collision_count
        if collision_count > self._previous_collision_count:
            entity = "UNKNOWN" if projection.collision is None else projection.collision.latest_contacted_entity
            self._emit(timestamp, "COLLISION", "COLLISION_DETECTED", f"Collision detected with {entity}", {"collision_count": collision_count})
        self._previous_collision_count = collision_count

        owner, _ = self._ownership(projection)
        if self._previous_owner is not None and owner != self._previous_owner:
            self._emit(timestamp, "OWNERSHIP", "OWNERSHIP_CHANGED", f"Motion ownership changed from {self._previous_owner} to {owner}")
        self._previous_owner = owner

        if state.landing_result == "CONFIRMED" and not self._landing_confirmed:
            self._landing_confirmed = True
            self._emit(timestamp, "SAFETY", "LANDING_CONFIRMED", "Landing confirmed")
        if self._landing_started and not projection.ego_state.armed and state.landing_result == "CONFIRMED" and not self._disarm_confirmed:
            self._disarm_confirmed = True
            self._emit(timestamp, "SAFETY", "DISARM_CONFIRMED", "Disarm confirmed")

    def _record_semantic_validation(
        self,
        projection: RuntimeContextProjection,
        waypoint_id: str,
        outcome: DeterministicObservationOutcome,
    ) -> None:
        if (
            not projection.semantic_validation_enabled
            or waypoint_id in self._semantic_validation_waypoints
            or self._plan is None
        ):
            return
        waypoint = next(
            (
                item
                for item in self._plan.waypoints
                if item.waypoint_id == waypoint_id and item.waypoint_type.value == "OBSERVATION"
            ),
            None,
        )
        if waypoint is None:
            return
        expected = waypoint.metadata.get("expected_target_label")
        record = self._semantic_validator.validate(
            waypoint_id=waypoint_id,
            expected_target_label=expected if isinstance(expected, str) else None,
            deterministic_outcome=outcome,
            vlm=projection.vlm,
            timestamp_monotonic=projection.timestamp_monotonic,
            fused_scene=self._fused_scene,
        )
        self._latest_semantic_validation = record
        self._semantic_validation_waypoints.add(waypoint_id)
        self._emit(
            projection.timestamp_monotonic,
            "SEMANTIC_VALIDATION",
            "OBSERVATION_SEMANTIC_VALIDATION",
            f"{waypoint_id}: {record.classification.value} - {record.rationale}",
            {
                "waypoint_id": waypoint_id,
                "expected_target_label": record.expected_target_label,
                "deterministic_outcome": record.deterministic_outcome.value,
                "vlm_target_visible": record.vlm_target_visible,
                "visible_landmarks": record.visible_landmarks,
                "scene_confidence": record.scene_confidence,
                "result_freshness": record.result_freshness.value,
                "evidence_source": record.evidence_source.value,
                "classification": record.classification.value,
            },
        )

    def _emit(
        self,
        timestamp: float,
        category: str,
        kind: str,
        message: str,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        values = dict(metadata or {})
        values["kind"] = kind
        self.memory.record_event(timestamp, category, message, values)
        self._emitted_event_count += 1
        self._event_categories[category] = self._event_categories.get(category, 0) + 1
        self._maximum_event_buffer_usage = max(
            self._maximum_event_buffer_usage,
            len(self.memory.recent_events()),
        )

    def _reset_edges(self) -> None:
        self._plan: MissionPlan | None = None
        self._mission_transition_cursor = 0
        self._observation_event_cursor = 0
        self._known_completed: set[str] = set()
        self._known_skipped: set[str] = set()
        self._known_failed: set[str] = set()
        self._previous_phase: str | None = None
        self._previous_result: str | None = None
        self._previous_navigator_state: str | None = None
        self._previous_avoidance_mode: str | None = None
        self._previous_recovery_state: str | None = None
        self._previous_visual_condition: str | None = None
        self._previous_depth_condition: str | None = None
        self._previous_depth_stale: bool | None = None
        self._previous_fused_scene_signature: tuple[object, ...] | None = None
        self._semantic_validation_waypoints: set[str] = set()
        self._latest_semantic_validation = None
        self._previous_emergency_state: str | None = None
        self._previous_supervisor_mode: str | None = None
        self._previous_hard_override = False
        self._previous_collision_count = 0
        self._previous_owner: str | None = None
        self._landing_started = False
        self._landing_confirmed = False
        self._disarm_confirmed = False
        self._mission_finished_recorded = False
        self._avoidance_count = 0
        self._recovery_count = 0
        self._observations_completed = 0
        self._emergency_suspected_count = 0
        self._emergency_cleared_count = 0
        self._emergency_confirmed_count = 0
        self._recent_recoveries: list[str] = []
        self._projection_count = 0
        self._emitted_event_count = 0
        self._maximum_event_buffer_usage = 0
        self._event_categories: dict[str, int] = {}
