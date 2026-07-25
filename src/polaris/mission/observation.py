from __future__ import annotations

from dataclasses import replace

from polaris.ego_state.models import NavigationState
from polaris.navigation.models import RecoveryState
from polaris.perception.depth.models import DepthCondition

from .models import (
    ObservationCompletionConfig,
    ObservationDecision,
    ObservationEvent,
    ObservationEventType,
    ObservationState,
    WaypointCompletionMode,
    WaypointType,
)


class ObservationTracker:
    """Bounded mission-level evidence tracker; it never transmits commands."""

    def __init__(self, waypoint_id: str, config: ObservationCompletionConfig) -> None:
        self.waypoint_id = waypoint_id
        self.config = config
        self.config.validate()
        self.events: list[ObservationEvent] = []
        self.reset()

    def reset(self) -> None:
        self.state = ObservationState.APPROACHING
        self.events.clear()
        self._wait_started_at: float | None = None
        self._dwell_started_inference: float | None = None
        self._dwell_elapsed_s = 0.0
        self._fresh_observation_count = 0
        self._last_depth_sequence = 0
        self._last_depth_inference = 0.0
        self._timed_out = False
        self._completion_ready = False

    def update(
        self,
        navigation: NavigationState,
        *,
        goal_distance_m: float,
        heading_error_deg: float,
        centre_risk: float,
        collision_detected: bool,
        active_perception_state: RecoveryState,
        now_monotonic: float,
    ) -> ObservationDecision:
        inside_band = (
            self.config.minimum_goal_distance_m
            <= goal_distance_m
            <= self.config.maximum_goal_distance_m
        )
        if inside_band and self._wait_started_at is None:
            self._wait_started_at = now_monotonic
        wait_elapsed = (
            0.0
            if self._wait_started_at is None
            else max(0.0, now_monotonic - self._wait_started_at)
        )
        if (
            not self._timed_out
            and self._wait_started_at is not None
            and wait_elapsed >= self.config.maximum_wait_s
        ):
            self._reset_dwell(
                "Observation opportunity reached its bounded wait",
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                now_monotonic,
                emit_reset=False,
            )
            self._timed_out = True
            self._event(
                ObservationEventType.TIMED_OUT,
                "Observation opportunity timed out; resuming positional handling",
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                now_monotonic,
            )
        if self._timed_out:
            return self._decision(
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                wait_elapsed,
                "Observation wait expired; ordinary waypoint handling active",
            )

        sequence = navigation.depth.source_frame_sequence
        inference = navigation.depth.inference_timestamp_monotonic
        fresh = (
            sequence > 0
            and sequence != self._last_depth_sequence
            and inference > self._last_depth_inference
        )
        if fresh:
            self._last_depth_sequence = sequence
            self._last_depth_inference = inference

        invalid = self._invalid_reason(
            navigation,
            inside_band,
            heading_error_deg,
            centre_risk,
            collision_detected,
            active_perception_state,
        )
        if invalid is not None:
            if self.state in {
                ObservationState.OBSERVATION_CANDIDATE,
                ObservationState.OBSERVING,
            }:
                self._reset_dwell(
                    invalid,
                    navigation,
                    goal_distance_m,
                    heading_error_deg,
                    centre_risk,
                    now_monotonic,
                    emit_reset=True,
                )
            return self._decision(
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                wait_elapsed,
                invalid,
            )

        if not fresh:
            return self._decision(
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                wait_elapsed,
                "Waiting for a distinct fresh depth observation",
                hold_required=self.state is ObservationState.OBSERVING,
            )

        self._fresh_observation_count += 1

        if self.state is ObservationState.APPROACHING:
            self.state = ObservationState.OBSERVATION_CANDIDATE
            self._event(
                ObservationEventType.CANDIDATE_ENTERED,
                "Observation geometry and quality qualified",
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                now_monotonic,
            )
            self._dwell_started_inference = inference
            self._dwell_elapsed_s = 0.0
            self.state = ObservationState.OBSERVING
            self._event(
                ObservationEventType.DWELL_STARTED,
                "Stable observation dwell started",
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                now_monotonic,
            )
        else:
            assert self._dwell_started_inference is not None
            self._dwell_elapsed_s = max(
                0.0,
                inference - self._dwell_started_inference,
            )

        if (
            self._dwell_elapsed_s >= self.config.required_stable_s
            and self._fresh_observation_count
            >= self.config.minimum_fresh_observations
        ):
            self._completion_ready = True
        return self._decision(
            navigation,
            goal_distance_m,
            heading_error_deg,
            centre_risk,
            wait_elapsed,
            (
                "Stable observation ready for supervised completion"
                if self._completion_ready
                else "Accumulating stable fresh observation dwell"
            ),
            hold_required=True,
        )

    def accept_completion(
        self,
        decision: ObservationDecision,
        navigation: NavigationState,
        now_monotonic: float,
    ) -> ObservationDecision:
        if not self._completion_ready or decision.timed_out:
            raise RuntimeError("Observation completion is not ready")
        self.state = ObservationState.OBSERVATION_COMPLETE
        self._completion_ready = False
        self._event(
            ObservationEventType.COMPLETED,
            "Stable observation complete after safety supervision",
            navigation,
            decision.goal_distance_m,
            decision.heading_error_deg,
            decision.centre_risk,
            now_monotonic,
        )
        return replace(
            decision,
            state=ObservationState.OBSERVATION_COMPLETE,
            completion_mode=WaypointCompletionMode.OBSERVATION,
            completion_ready=False,
            completed=True,
            reason="Stable observation complete after safety supervision",
        )

    def _invalid_reason(
        self,
        navigation: NavigationState,
        inside_band: bool,
        heading_error_deg: float,
        centre_risk: float,
        collision_detected: bool,
        active_perception_state: RecoveryState,
    ) -> str | None:
        if collision_detected:
            return "Collision prevents observation completion"
        if not inside_band:
            return "Outside configured observation distance band"
        if not navigation.local_position_ok or navigation.position_is_stale():
            return "Localization is unavailable or stale"
        if abs(heading_error_deg) > self.config.maximum_heading_error_deg:
            return "Observation heading error exceeds configured bound"
        if (
            not navigation.visual.camera_connected
            or navigation.visual.stale
            or not navigation.depth.available
            or navigation.depth.stale
        ):
            return "Visual or depth perception is stale"
        if navigation.visual.visual_health_score < self.config.minimum_visual_health:
            return "Visual health is below observation minimum"
        if (
            navigation.depth.condition is DepthCondition.CRITICAL
            or navigation.depth.depth_health_score < self.config.minimum_depth_health
        ):
            return "Depth health is below observation minimum"
        if navigation.depth.temporal_stability < self.config.minimum_depth_stability:
            return "Depth stability is below observation minimum"
        if centre_risk > self.config.maximum_centre_risk:
            return "Centre risk exceeds safe observation maximum"
        if navigation.horizontal_speed_m_s > self.config.maximum_horizontal_speed_m_s:
            return "Horizontal speed exceeds stable observation maximum"
        if active_perception_state in {RecoveryState.FAILED, RecoveryState.ABORT}:
            return "Active perception is in a failed state"
        return None

    def _reset_dwell(
        self,
        reason: str,
        navigation: NavigationState,
        goal_distance_m: float,
        heading_error_deg: float,
        centre_risk: float,
        now_monotonic: float,
        *,
        emit_reset: bool,
    ) -> None:
        if emit_reset:
            self._event(
                ObservationEventType.DWELL_RESET,
                reason,
                navigation,
                goal_distance_m,
                heading_error_deg,
                centre_risk,
                now_monotonic,
            )
        self.state = ObservationState.APPROACHING
        self._dwell_started_inference = None
        self._dwell_elapsed_s = 0.0
        self._completion_ready = False
        self._fresh_observation_count = 0

    def _event(
        self,
        event_type: ObservationEventType,
        reason: str,
        navigation: NavigationState,
        goal_distance_m: float,
        heading_error_deg: float,
        centre_risk: float,
        now_monotonic: float,
    ) -> None:
        self.events.append(
            ObservationEvent(
                event_type=event_type,
                timestamp_monotonic=now_monotonic,
                waypoint_id=self.waypoint_id,
                reason=reason,
                goal_distance_m=goal_distance_m,
                heading_error_deg=heading_error_deg,
                visual_health=navigation.visual.visual_health_score,
                depth_health=navigation.depth.depth_health_score,
                depth_stability=navigation.depth.temporal_stability,
                centre_risk=centre_risk,
                horizontal_speed_m_s=navigation.horizontal_speed_m_s,
                dwell_elapsed_s=self._dwell_elapsed_s,
                fresh_observation_count=self._fresh_observation_count,
                depth_frame_sequence=navigation.depth.source_frame_sequence,
                depth_inference_timestamp_monotonic=(
                    navigation.depth.inference_timestamp_monotonic
                ),
            )
        )

    def _decision(
        self,
        navigation: NavigationState,
        goal_distance_m: float,
        heading_error_deg: float,
        centre_risk: float,
        wait_elapsed_s: float,
        reason: str,
        *,
        hold_required: bool = False,
    ) -> ObservationDecision:
        completed = self.state is ObservationState.OBSERVATION_COMPLETE
        return ObservationDecision(
            state=self.state,
            waypoint_type=WaypointType.OBSERVATION,
            completion_mode=(
                WaypointCompletionMode.OBSERVATION if completed else None
            ),
            hold_required=hold_required or completed,
            completion_ready=self._completion_ready,
            completed=completed,
            timed_out=self._timed_out,
            reason=reason,
            goal_distance_m=goal_distance_m,
            heading_error_deg=heading_error_deg,
            visual_health=navigation.visual.visual_health_score,
            depth_health=navigation.depth.depth_health_score,
            depth_stability=navigation.depth.temporal_stability,
            centre_risk=centre_risk,
            horizontal_speed_m_s=navigation.horizontal_speed_m_s,
            dwell_elapsed_s=self._dwell_elapsed_s,
            fresh_observation_count=self._fresh_observation_count,
            wait_elapsed_s=wait_elapsed_s,
            depth_frame_sequence=navigation.depth.source_frame_sequence,
            depth_inference_timestamp_monotonic=(
                navigation.depth.inference_timestamp_monotonic
            ),
            config=self.config,
        )
