from __future__ import annotations

import math
from time import monotonic

from polaris.ego_state.confidence import ConfidenceReport, SafetyMode
from polaris.ego_state.models import NavigationState
from polaris.perception.depth.models import DepthCondition

from .models import (
    AvoidanceConfig,
    AvoidanceDecision,
    AvoidanceMode,
    DesiredMotion,
    EmergencyConfirmation,
    EmergencyConfirmationState,
    RecommendedAction,
    SafeMotion,
    SupervisorMode,
)


class SafetySupervisor:
    """Final deterministic gate between reactive policy and PX4 Offboard."""

    def __init__(self, config: AvoidanceConfig) -> None:
        self.config = config
        self._last_evaluation_monotonic = 0.0
        self._confirmation = EmergencyConfirmation(
            required_frames=config.emergency_confirmation_frames,
            timeout_s=config.emergency_confirmation_timeout_s,
        )
        self._confirmation_started_at: float | None = None
        self._confirmation_last_sequence = 0
        self._confirmation_last_inference = 0.0
        self._confirmation_risks: list[float] = []

    def evaluate(
        self,
        decision: AvoidanceDecision,
        navigation: NavigationState,
        confidence: ConfidenceReport,
        *,
        px4_connected: bool,
        autonomy_enabled: bool,
        offboard_expected: bool,
        displacement_from_start_m: float,
        collision_detected: bool = False,
        now_monotonic: float | None = None,
    ) -> SafeMotion:
        now = monotonic() if now_monotonic is None else now_monotonic
        state_age_ms = self._source_state_age_ms(navigation, now)
        loop_gap = (
            0.0
            if self._last_evaluation_monotonic == 0.0
            else now - self._last_evaluation_monotonic
        )
        self._last_evaluation_monotonic = now

        hard_reasons: list[str] = []
        if not px4_connected:
            hard_reasons.append("PX4 disconnected")
        if not decision.risk.valid:
            hard_reasons.append("Risk grid contains invalid or non-finite values")
        if not decision.requested_command.finite() or not decision.proposed_command.finite():
            hard_reasons.append("Motion command contains invalid or non-finite values")
        if navigation.in_air or navigation.armed:
            if collision_detected:
                hard_reasons.append("Airborne collision detected")
            if not navigation.local_position_ok or navigation.position_is_stale():
                hard_reasons.append("Local position unavailable or stale")
            if navigation.visual.stale or not navigation.visual.camera_connected:
                hard_reasons.append("RGB unavailable or stale")
            elif navigation.visual.frame_age_ms > self.config.stale_timeout_ms:
                hard_reasons.append("RGB exceeded configured stale timeout")
            if self.config.depth_required_for_motion and (
                not navigation.depth.available or navigation.depth.stale
            ):
                hard_reasons.append("Required depth unavailable or stale")
            elif (
                self.config.depth_required_for_motion
                and navigation.depth.inference_timestamp_monotonic > 0
                and (now - navigation.depth.inference_timestamp_monotonic) * 1000
                > self.config.stale_timeout_ms
            ):
                hard_reasons.append("Depth exceeded configured stale timeout")
            if navigation.depth.condition is DepthCondition.CRITICAL:
                hard_reasons.append("Depth condition is CRITICAL")
            if confidence.mode is SafetyMode.EMERGENCY:
                hard_reasons.append("Confidence Engine is EMERGENCY")
            if state_age_ms > self.config.maximum_state_age_ms:
                hard_reasons.append("Ego State is too old")
            if offboard_expected and navigation.flight_mode != "OFFBOARD":
                hard_reasons.append("Offboard mode was lost")
            if loop_gap > self.config.control_loop_timeout_s:
                hard_reasons.append("Control loop update deadline was missed")
            if displacement_from_start_m > self.config.maximum_displacement_m:
                hard_reasons.append("Displacement-from-start limit exceeded")

        if hard_reasons:
            self.reset_emergency_confirmation("Bypassed by immediate hard safety condition")
            return SafeMotion.hover(
                now,
                SupervisorMode.ABORT,
                tuple(hard_reasons),
                True,
                state_age_ms,
                RecommendedAction.LAND,
                self._confirmation,
            )
        if not autonomy_enabled:
            self.reset_emergency_confirmation("Reactive autonomy is disabled")
            return SafeMotion.hover(
                now,
                SupervisorMode.HOVER,
                ("Reactive autonomy is disabled",),
                False,
                state_age_ms,
                emergency_confirmation=self._confirmation,
            )
        if decision.mode is AvoidanceMode.ABORT:
            self.reset_emergency_confirmation("Controller requested immediate abort")
            return SafeMotion.hover(
                now,
                SupervisorMode.ABORT,
                decision.reasons,
                True,
                state_age_ms,
                RecommendedAction.LAND,
                self._confirmation,
            )
        if decision.risk.centre >= self.config.emergency_threshold:
            if self._active_perception_owns_motion(decision):
                return self._evaluate_emergency_confirmation(
                    decision,
                    navigation,
                    now,
                    state_age_ms,
                )
            self._confirmation = EmergencyConfirmation(
                state=EmergencyConfirmationState.CONFIRMED,
                observed_frames=1,
                required_frames=1,
                age_s=0.0,
                timeout_s=self.config.emergency_confirmation_timeout_s,
                centre_risk=decision.risk.centre,
                variance=0.0,
                neighbour_agreement=self._neighbour_agreement(decision),
                reason="Emergency centre proximity risk during ordinary motion",
            )
            return SafeMotion.hover(
                now,
                SupervisorMode.ABORT,
                ("Emergency centre proximity risk",),
                True,
                state_age_ms,
                RecommendedAction.LAND,
                self._confirmation,
            )

        if self._confirmation.state is not EmergencyConfirmationState.CLEAR:
            reason = (
                "Emergency centre proximity risk cleared"
                if self._active_perception_owns_motion(decision)
                else "Active-perception ownership ended"
            )
            self.reset_emergency_confirmation(reason)

        proposed = decision.proposed_command
        forward = proposed.forward_m_s
        right = proposed.right_m_s
        down = proposed.down_m_s
        yaw = proposed.yaw_rate_deg_s
        reasons = list(decision.reasons)
        mode = self._mode_for_decision(decision.mode)
        if confidence.mode in {SafetyMode.CAUTIOUS, SafetyMode.DEGRADED}:
            forward = math.copysign(
                min(abs(forward), self.config.cautious_speed_m_s),
                forward,
            ) if forward else 0.0
            right = math.copysign(
                min(abs(right), self.config.sidestep_speed_m_s),
                right,
            ) if right else 0.0
            reasons.append(f"Confidence mode {confidence.mode.name} constrained speed")
            if mode is SupervisorMode.PASS_THROUGH:
                mode = SupervisorMode.SLOW

        horizontal = math.hypot(forward, right)
        if horizontal > self.config.maximum_speed_m_s:
            scale = self.config.maximum_speed_m_s / horizontal
            forward *= scale
            right *= scale
            reasons.append("Horizontal command clamped to configured maximum")
        down = max(-self.config.climb_speed_m_s, min(self.config.climb_speed_m_s, down))
        yaw = max(-self.config.yaw_rate_deg_s, min(self.config.yaw_rate_deg_s, yaw))
        if navigation.horizontal_speed_m_s > self.config.maximum_speed_m_s * 1.5:
            return SafeMotion.hover(
                now,
                SupervisorMode.HOVER,
                ("Measured horizontal speed exceeds safety envelope",),
                False,
                state_age_ms,
                emergency_confirmation=self._confirmation,
            )

        modified = any(
            not math.isclose(actual, requested, abs_tol=1e-6)
            for actual, requested in (
                (forward, decision.requested_command.forward_m_s),
                (right, decision.requested_command.right_m_s),
                (down, decision.requested_command.down_m_s),
                (yaw, decision.requested_command.yaw_rate_deg_s),
            )
        )
        return SafeMotion(
            forward,
            right,
            down,
            yaw,
            accepted=True,
            modified=modified,
            supervisor_mode=mode,
            reasons=tuple(dict.fromkeys(reasons)),
            hard_override=False,
            recommended_action=self._action_for_mode(mode),
            timestamp_monotonic=now,
            source_state_age_ms=state_age_ms,
            emergency_confirmation=self._confirmation,
        )

    def reset_control_timer(self) -> None:
        self._last_evaluation_monotonic = 0.0

    def reset(self) -> None:
        self.reset_control_timer()
        self.reset_emergency_confirmation("Confirmation reset")

    def reset_emergency_confirmation(
        self,
        reason: str = "No emergency proximity evidence",
    ) -> None:
        self._confirmation_started_at = None
        self._confirmation_last_sequence = 0
        self._confirmation_last_inference = 0.0
        self._confirmation_risks.clear()
        self._confirmation = EmergencyConfirmation(
            required_frames=self.config.emergency_confirmation_frames,
            timeout_s=self.config.emergency_confirmation_timeout_s,
            reason=reason,
        )

    def _evaluate_emergency_confirmation(
        self,
        decision: AvoidanceDecision,
        navigation: NavigationState,
        now: float,
        state_age_ms: float,
    ) -> SafeMotion:
        sequence = navigation.depth.source_frame_sequence
        inference = navigation.depth.inference_timestamp_monotonic
        if sequence < self._confirmation_last_sequence:
            self.reset_emergency_confirmation("Depth sequence restarted")
        if self._confirmation_started_at is None:
            self._confirmation_started_at = now

        fresh = (
            sequence > 0
            and sequence != self._confirmation_last_sequence
            and inference > self._confirmation_last_inference
        )
        if fresh:
            self._confirmation_last_sequence = sequence
            self._confirmation_last_inference = inference
            self._confirmation_risks.append(decision.risk.centre)
            self._confirmation_risks = self._confirmation_risks[
                -self.config.emergency_confirmation_frames :
            ]

        age = max(0.0, now - self._confirmation_started_at)
        variance = self._risk_variance()
        observed = len(self._confirmation_risks)
        confirmed_by_frames = (
            observed >= self.config.emergency_confirmation_frames
            and variance <= self.config.emergency_confirmation_max_variance
        )
        timed_out = age >= self.config.emergency_confirmation_timeout_s
        if confirmed_by_frames or timed_out:
            reason = (
                "Emergency centre proximity risk confirmed across fresh depth frames"
                if confirmed_by_frames
                else "Emergency centre proximity confirmation timed out"
            )
            self._confirmation = self._confirmation_snapshot(
                EmergencyConfirmationState.CONFIRMED,
                decision,
                age,
                variance,
                reason,
            )
            return SafeMotion.hover(
                now,
                SupervisorMode.ABORT,
                (reason,),
                True,
                state_age_ms,
                RecommendedAction.LAND,
                self._confirmation,
            )

        reason = "Emergency centre proximity suspected; confirming fresh depth"
        self._confirmation = self._confirmation_snapshot(
            EmergencyConfirmationState.SUSPECTED,
            decision,
            age,
            variance,
            reason,
        )
        requested = decision.requested_command
        yaw = max(
            -self.config.yaw_rate_deg_s,
            min(self.config.yaw_rate_deg_s, requested.yaw_rate_deg_s),
        )
        # Translation remains zero while bounded active-perception yaw observes.
        return SafeMotion(
            0.0,
            0.0,
            0.0,
            yaw,
            accepted=True,
            modified=True,
            supervisor_mode=SupervisorMode.HOVER,
            reasons=(reason,),
            hard_override=False,
            recommended_action=RecommendedAction.HOVER,
            timestamp_monotonic=now,
            source_state_age_ms=state_age_ms,
            emergency_confirmation=self._confirmation,
        )

    def _confirmation_snapshot(
        self,
        state: EmergencyConfirmationState,
        decision: AvoidanceDecision,
        age: float,
        variance: float,
        reason: str,
    ) -> EmergencyConfirmation:
        return EmergencyConfirmation(
            state=state,
            observed_frames=len(self._confirmation_risks),
            required_frames=self.config.emergency_confirmation_frames,
            age_s=age,
            timeout_s=self.config.emergency_confirmation_timeout_s,
            centre_risk=decision.risk.centre,
            variance=variance,
            neighbour_agreement=self._neighbour_agreement(decision),
            reason=reason,
        )

    def _risk_variance(self) -> float:
        if len(self._confirmation_risks) < 2:
            return 0.0
        mean = sum(self._confirmation_risks) / len(self._confirmation_risks)
        return sum((value - mean) ** 2 for value in self._confirmation_risks) / len(
            self._confirmation_risks
        )

    def _neighbour_agreement(self, decision: AvoidanceDecision) -> bool:
        return max(
            decision.risk.upper,
            decision.risk.left,
            decision.risk.right,
        ) >= self.config.stop_threshold

    @staticmethod
    def _active_perception_owns_motion(decision: AvoidanceDecision) -> bool:
        requested = decision.requested_command
        sources = {
            "active-perception",
            "active-perception-scan",
            "active-perception-settle",
            "active-perception-resettle",
            "active-perception-sample",
            "active-perception-return",
            "active-perception-hold",
            "active-perception-observation",
        }
        return (
            requested.source in sources
            and abs(requested.forward_m_s) < 1e-6
            and abs(requested.right_m_s) < 1e-6
            and abs(requested.down_m_s) < 1e-6
        )

    def _source_state_age_ms(self, state: NavigationState, now: float) -> float:
        ages = []
        for timestamp in (
            state.last_position_update_monotonic,
            state.last_attitude_update_monotonic,
        ):
            ages.append(float("inf") if timestamp == 0.0 else max(0.0, now - timestamp) * 1000)
        ages.append(state.visual.frame_age_ms)
        if self.config.depth_required_for_motion:
            ages.append(
                float("inf")
                if state.depth.inference_timestamp_monotonic == 0.0
                else max(0.0, now - state.depth.inference_timestamp_monotonic) * 1000
            )
        return max(ages)

    @staticmethod
    def _mode_for_decision(mode: AvoidanceMode) -> SupervisorMode:
        return {
            AvoidanceMode.CLEAR: SupervisorMode.PASS_THROUGH,
            AvoidanceMode.CAUTION: SupervisorMode.SLOW,
            AvoidanceMode.AVOID_LEFT: SupervisorMode.AVOID_LEFT,
            AvoidanceMode.AVOID_RIGHT: SupervisorMode.AVOID_RIGHT,
            AvoidanceMode.RETREAT: SupervisorMode.RETREAT,
            AvoidanceMode.HOVER: SupervisorMode.HOVER,
            AvoidanceMode.ABORT: SupervisorMode.ABORT,
            AvoidanceMode.LAND: SupervisorMode.LAND,
        }[mode]

    @staticmethod
    def _action_for_mode(mode: SupervisorMode) -> RecommendedAction:
        return {
            SupervisorMode.PASS_THROUGH: RecommendedAction.CONTINUE,
            SupervisorMode.SLOW: RecommendedAction.SLOW,
            SupervisorMode.AVOID_LEFT: RecommendedAction.SIDESTEP,
            SupervisorMode.AVOID_RIGHT: RecommendedAction.SIDESTEP,
            SupervisorMode.RETREAT: RecommendedAction.RETREAT,
            SupervisorMode.HOVER: RecommendedAction.HOVER,
            SupervisorMode.ABORT: RecommendedAction.ABORT,
            SupervisorMode.LAND: RecommendedAction.LAND,
        }[mode]
