from __future__ import annotations

from time import monotonic

from polaris.ego_state.models import NavigationState

from .models import (
    AvoidanceConfig,
    AvoidanceDecision,
    AvoidanceMode,
    AvoidanceState,
    DesiredMotion,
    EscapeDirection,
    RiskSummary,
)


class ReactiveAvoidanceController:
    """Deterministic body-frame policy over the compressed proximity grid."""

    def __init__(self, config: AvoidanceConfig) -> None:
        self.config = config
        self.state = AvoidanceState()

    def update(
        self,
        requested: DesiredMotion,
        navigation: NavigationState,
        confidence_mode: str,
        now_monotonic: float | None = None,
    ) -> AvoidanceDecision:
        now = monotonic() if now_monotonic is None else now_monotonic
        risk = RiskSummary.from_grid(navigation.depth.risk_grid)
        reasons: list[str] = []
        observation_only = (
            requested.source.startswith("active-perception")
            and requested.forward_m_s == 0.0
            and requested.right_m_s == 0.0
            and requested.down_m_s == 0.0
        )

        if not requested.finite() or not risk.valid:
            return self._decision(
                AvoidanceMode.ABORT,
                requested,
                DesiredMotion.hover(now, "invalid-risk"),
                risk,
                ("Requested motion or risk grid is invalid",),
                confidence_mode,
                now,
            )

        self._update_persistence(navigation, risk)
        if observation_only:
            self.state.avoidance_episode_started_at = 0.0
        elapsed = self._avoidance_elapsed(now)
        if (
            self.state.avoidance_episode_started_at > 0
            and elapsed > self.config.maximum_avoidance_duration_s
        ):
            return self._transition_decision(
                AvoidanceMode.ABORT,
                requested,
                DesiredMotion.hover(now, "avoidance-timeout"),
                risk,
                "Maximum avoidance duration exceeded",
                confidence_mode,
                now,
            )

        if navigation.visual.visual_health_score < self.config.minimum_visual_health:
            if observation_only:
                return self._transition_decision(
                    AvoidanceMode.CAUTION,
                    requested,
                    requested,
                    risk,
                    "Allowing bounded yaw-only perception recovery",
                    confidence_mode,
                    now,
                )
            return self._transition_decision(
                AvoidanceMode.HOVER,
                requested,
                DesiredMotion.hover(now, "low-visual-health"),
                risk,
                "Visual health is below motion minimum",
                confidence_mode,
                now,
            )
        if navigation.depth.depth_health_score < self.config.minimum_depth_health:
            if observation_only and navigation.depth.condition.value != "CRITICAL":
                return self._transition_decision(
                    AvoidanceMode.CAUTION,
                    requested,
                    requested,
                    risk,
                    "Allowing yaw-only recovery with degraded depth",
                    confidence_mode,
                    now,
                )
            return self._transition_decision(
                AvoidanceMode.HOVER,
                requested,
                DesiredMotion.hover(now, "low-depth-health"),
                risk,
                "Depth health is below motion minimum",
                confidence_mode,
                now,
            )
        if navigation.depth.temporal_stability < self.config.critical_temporal_stability:
            return self._transition_decision(
                AvoidanceMode.HOVER,
                requested,
                DesiredMotion.hover(now, "unstable-depth"),
                risk,
                "Depth stability is critically low",
                confidence_mode,
                now,
            )

        if risk.centre >= self.config.emergency_threshold:
            return self._transition_decision(
                AvoidanceMode.RETREAT,
                requested,
                self._retreat(now),
                risk,
                "Emergency centre risk",
                confidence_mode,
                now,
            )

        if observation_only:
            return self._transition_decision(
                AvoidanceMode.CAUTION,
                requested,
                requested,
                risk,
                "Bounded yaw-only active-perception command",
                confidence_mode,
                now,
            )

        if (
            abs(requested.forward_m_s) < 1e-6
            and abs(requested.right_m_s) < 1e-6
        ):
            self._transition(AvoidanceMode.CLEAR, "No horizontal motion requested", now)
            return self._decision(
                AvoidanceMode.CLEAR,
                requested,
                requested,
                risk,
                ("No horizontal motion requested",),
                confidence_mode,
                now,
            )

        if self.state.mode in {
            AvoidanceMode.AVOID_LEFT,
            AvoidanceMode.AVOID_RIGHT,
            AvoidanceMode.RETREAT,
        }:
            chosen_risk = (
                risk.left
                if self.state.mode is AvoidanceMode.AVOID_LEFT
                else risk.right if self.state.mode is AvoidanceMode.AVOID_RIGHT else 0.0
            )
            if chosen_risk >= self.config.stop_threshold:
                return self._transition_decision(
                    AvoidanceMode.HOVER,
                    requested,
                    DesiredMotion.hover(now, "blocked-escape-sector"),
                    risk,
                    "Chosen escape sector became blocked",
                    confidence_mode,
                    now,
                )
            if (
                self.state.clear_count >= self.config.clear_persistence_updates
                and now - self.state.direction_chosen_at
                >= self.config.preferred_direction_hold_s
                and now - self.state.mode_entered_at >= self.config.sidestep_cooldown_s
            ):
                self._transition(AvoidanceMode.CLEAR, "Clear persistence satisfied", now)
            else:
                return self._decision(
                    self.state.mode,
                    requested,
                    self._command_for_mode(self.state.mode, now),
                    risk,
                    ("Holding avoidance direction until clear persistence",),
                    confidence_mode,
                    now,
                )

        both_sides_persistent = (
            self.state.left_blocked_count >= self.config.blocked_persistence_updates
            and self.state.right_blocked_count >= self.config.blocked_persistence_updates
        )
        if both_sides_persistent:
            if self.state.blocked_count < self.config.blocked_persistence_updates:
                proposed = DesiredMotion(
                    forward_m_s=min(
                        max(0.0, requested.forward_m_s),
                        self.config.cautious_speed_m_s,
                    ),
                    source="constrained-corridor",
                    timestamp_monotonic=now,
                )
                return self._transition_decision(
                    AvoidanceMode.CAUTION,
                    requested,
                    proposed,
                    risk,
                    "Both sides blocked but centre remains clear",
                    confidence_mode,
                    now,
                )
            self._start_avoidance_episode(now)
            if self.state.retreat_count < self.config.retreat_persistence_updates:
                return self._transition_decision(
                    AvoidanceMode.HOVER,
                    requested,
                    DesiredMotion.hover(now, "confirm-no-safe-path"),
                    risk,
                    "Hovering while both-side blockage is confirmed",
                    confidence_mode,
                    now,
                )
            return self._transition_decision(
                AvoidanceMode.RETREAT,
                requested,
                self._retreat(now),
                risk,
                "Both side sectors are persistently blocked",
                confidence_mode,
                now,
            )

        if self.state.blocked_count >= self.config.blocked_persistence_updates:
            direction = self._safer_direction(risk)
            if direction is None:
                return self._transition_decision(
                    AvoidanceMode.RETREAT,
                    requested,
                    self._retreat(now),
                    risk,
                    "No side has a reliable clearance margin",
                    confidence_mode,
                    now,
                )
            return self._choose_escape(direction, requested, risk, confidence_mode, now)

        if self.state.left_blocked_count >= self.config.blocked_persistence_updates:
            return self._choose_escape(
                EscapeDirection.RIGHT,
                requested,
                risk,
                confidence_mode,
                now,
                "Persistent left-sector risk",
            )
        if self.state.right_blocked_count >= self.config.blocked_persistence_updates:
            return self._choose_escape(
                EscapeDirection.LEFT,
                requested,
                risk,
                confidence_mode,
                now,
                "Persistent right-sector risk",
            )

        cautious = (
            risk.forward >= self.config.caution_threshold
            or navigation.depth.temporal_stability < self.config.minimum_temporal_stability
            or navigation.visual.visual_condition.value != "HEALTHY"
        )
        if cautious and requested.forward_m_s > 0:
            reasons.append("Forward risk or perception quality requires reduced speed")
            proposed = DesiredMotion(
                forward_m_s=min(requested.forward_m_s, self.config.cautious_speed_m_s),
                right_m_s=requested.right_m_s,
                down_m_s=0.0,
                yaw_rate_deg_s=0.0,
                source="reactive-caution",
                timestamp_monotonic=now,
            )
            self._transition(AvoidanceMode.CAUTION, reasons[0], now)
            return self._decision(
                AvoidanceMode.CAUTION,
                requested,
                proposed,
                risk,
                tuple(reasons),
                confidence_mode,
                now,
            )

        self._transition(AvoidanceMode.CLEAR, "Forward sector clear", now)
        proposed = DesiredMotion(
            requested.forward_m_s,
            requested.right_m_s,
            requested.down_m_s,
            requested.yaw_rate_deg_s,
            source="reactive-clear",
            timestamp_monotonic=now,
        )
        return self._decision(
            AvoidanceMode.CLEAR,
            requested,
            proposed,
            risk,
            ("Forward sector clear",),
            confidence_mode,
            now,
        )

    def shutdown_command(self, now_monotonic: float | None = None) -> DesiredMotion:
        now = monotonic() if now_monotonic is None else now_monotonic
        return DesiredMotion.hover(now, "controller-shutdown")

    def _update_persistence(self, navigation: NavigationState, risk: RiskSummary) -> None:
        sequence = navigation.depth.source_frame_sequence
        if sequence == 0 or sequence == self.state.last_depth_sequence:
            return
        self.state.last_depth_sequence = sequence
        self.state.last_risk_grid = navigation.depth.risk_grid
        self.state.blocked_count = (
            self.state.blocked_count + 1
            if risk.centre >= self.config.stop_threshold
            else 0
        )
        self.state.left_blocked_count = (
            self.state.left_blocked_count + 1
            if risk.left >= self.config.stop_threshold
            else 0
        )
        self.state.right_blocked_count = (
            self.state.right_blocked_count + 1
            if risk.right >= self.config.stop_threshold
            else 0
        )
        clear = (
            risk.centre <= self.config.clear_threshold
            and risk.left < self.config.stop_threshold
            and risk.right < self.config.stop_threshold
        )
        self.state.clear_count = self.state.clear_count + 1 if clear else 0
        self.state.retreat_count = (
            self.state.retreat_count + 1
            if risk.centre >= self.config.stop_threshold
            and risk.left >= self.config.stop_threshold
            and risk.right >= self.config.stop_threshold
            else 0
        )

    def _safer_direction(self, risk: RiskSummary) -> EscapeDirection | None:
        difference = abs(risk.left - risk.right)
        if difference < self.config.minimum_escape_margin:
            if max(risk.left, risk.right) >= self.config.stop_threshold:
                return None
            return EscapeDirection(self.config.preferred_turn_direction)
        return EscapeDirection.LEFT if risk.left < risk.right else EscapeDirection.RIGHT

    def _choose_escape(
        self,
        direction: EscapeDirection,
        requested: DesiredMotion,
        risk: RiskSummary,
        confidence_mode: str,
        now: float,
        reason: str = "Persistent centre obstacle",
    ) -> AvoidanceDecision:
        destination_risk = (
            risk.left if direction is EscapeDirection.LEFT else risk.right
        )
        if destination_risk >= self.config.stop_threshold:
            self._start_avoidance_episode(now)
            return self._transition_decision(
                AvoidanceMode.HOVER,
                requested,
                DesiredMotion.hover(now, "blocked-destination-sector"),
                risk,
                "Selected escape sector is blocked",
                confidence_mode,
                now,
            )
        current = self.state.chosen_direction
        if current is not EscapeDirection.NONE and current is not direction:
            current_risk = risk.left if current is EscapeDirection.LEFT else risk.right
            if (
                current_risk < self.config.stop_threshold
                and now - self.state.direction_chosen_at
                < self.config.preferred_direction_hold_s
            ):
                direction = current
                reason = "Preferred direction hold prevented reversal"
            else:
                self.state.reversal_count += 1
                if self.state.reversal_count > self.config.maximum_reversals:
                    return self._transition_decision(
                        AvoidanceMode.ABORT,
                        requested,
                        DesiredMotion.hover(now, "reversal-limit"),
                        risk,
                        "Maximum avoidance reversal count exceeded",
                        confidence_mode,
                        now,
                    )
        if self.state.chosen_direction is not direction:
            self.state.chosen_direction = direction
            self.state.direction_chosen_at = now
        mode = (
            AvoidanceMode.AVOID_LEFT
            if direction is EscapeDirection.LEFT
            else AvoidanceMode.AVOID_RIGHT
        )
        proposed = self._sidestep(direction, now)
        return self._transition_decision(
            mode,
            requested,
            proposed,
            risk,
            reason,
            confidence_mode,
            now,
        )

    def _sidestep(self, direction: EscapeDirection, now: float) -> DesiredMotion:
        right = (
            -self.config.sidestep_speed_m_s
            if direction is EscapeDirection.LEFT
            else self.config.sidestep_speed_m_s
        )
        return DesiredMotion(
            forward_m_s=0.0,
            right_m_s=right,
            source=f"avoid-{direction.value.lower()}",
            timestamp_monotonic=now,
        )

    def _retreat(self, now: float) -> DesiredMotion:
        return DesiredMotion(
            forward_m_s=-self.config.retreat_speed_m_s,
            source="reactive-retreat",
            timestamp_monotonic=now,
        )

    def _command_for_mode(self, mode: AvoidanceMode, now: float) -> DesiredMotion:
        if mode is AvoidanceMode.AVOID_LEFT:
            return self._sidestep(EscapeDirection.LEFT, now)
        if mode is AvoidanceMode.AVOID_RIGHT:
            return self._sidestep(EscapeDirection.RIGHT, now)
        if mode is AvoidanceMode.RETREAT:
            return self._retreat(now)
        return DesiredMotion.hover(now)

    def _transition_decision(
        self,
        mode: AvoidanceMode,
        requested: DesiredMotion,
        proposed: DesiredMotion,
        risk: RiskSummary,
        reason: str,
        confidence_mode: str,
        now: float,
    ) -> AvoidanceDecision:
        self._transition(mode, reason, now)
        return self._decision(
            mode,
            requested,
            proposed,
            risk,
            (reason,),
            confidence_mode,
            now,
        )

    def _transition(self, mode: AvoidanceMode, reason: str, now: float) -> None:
        if mode in {
            AvoidanceMode.AVOID_LEFT,
            AvoidanceMode.AVOID_RIGHT,
            AvoidanceMode.RETREAT,
        }:
            self._start_avoidance_episode(now)
        elif mode is AvoidanceMode.CLEAR:
            self.state.avoidance_episode_started_at = 0.0
        if self.state.mode is not mode:
            self.state.mode = mode
            self.state.mode_entered_at = now
        self.state.last_transition_reason = reason

    def _avoidance_elapsed(self, now: float) -> float:
        if self.state.avoidance_episode_started_at == 0.0:
            return 0.0
        return max(0.0, now - self.state.avoidance_episode_started_at)

    def _start_avoidance_episode(self, now: float) -> None:
        if self.state.avoidance_episode_started_at == 0.0:
            self.state.avoidance_episode_started_at = now

    def _decision(
        self,
        mode: AvoidanceMode,
        requested: DesiredMotion,
        proposed: DesiredMotion,
        risk: RiskSummary,
        reasons: tuple[str, ...],
        confidence_mode: str,
        now: float,
    ) -> AvoidanceDecision:
        self.state.last_command = proposed
        return AvoidanceDecision(
            mode=mode,
            requested_command=requested,
            proposed_command=proposed,
            risk=risk,
            reasons=reasons,
            confidence_mode=confidence_mode,
            timestamp_monotonic=now,
            chosen_direction=self.state.chosen_direction,
            blocked_count=self.state.blocked_count,
            clear_count=self.state.clear_count,
            reversal_count=self.state.reversal_count,
            elapsed_avoidance_s=self._avoidance_elapsed(now),
            caution_threshold=self.config.caution_threshold,
            stop_threshold=self.config.stop_threshold,
            emergency_threshold=self.config.emergency_threshold,
        )
