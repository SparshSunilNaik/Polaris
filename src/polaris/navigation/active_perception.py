from __future__ import annotations

import statistics
from time import monotonic

from polaris.control.models import AvoidanceDecision, AvoidanceMode, DesiredMotion
from polaris.ego_state.models import NavigationState
from polaris.evaluation.viewpoint import normalize_yaw, shortest_yaw_delta

from .models import (
    ActivePerceptionConfig,
    GoalDecision,
    RecoveryAction,
    RecoveryDecision,
    RecoveryState,
    ViewpointCandidate,
    ViewpointScore,
)
from .viewpoint_scoring import ViewpointScorer


class ActivePerceptionManager:
    def __init__(
        self,
        config: ActivePerceptionConfig,
        scorer: ViewpointScorer | None = None,
    ) -> None:
        self.config = config
        self.scorer = scorer or ViewpointScorer(config)
        self.state = RecoveryState.IDLE
        self.trigger_reason = ""
        self.base_heading_deg = 0.0
        self.goal_heading_deg = 0.0
        self.selected_heading_deg: float | None = None
        self.target_heading_deg: float | None = None
        self.attempt_count = 0
        self.scan_count = 0
        self.total_yaw_travelled_deg = 0.0
        self.last_heading_deg: float | None = None
        self.last_completed_at = -float("inf")
        self.state_entered_at = 0.0
        self.scan_index = 0
        self.settle_started_at = 0.0
        self.sample_started_at = 0.0
        self.heading_hold_started_at = 0.0
        self.last_depth_sequence = 0
        self.last_visual_update = 0.0
        self.baseline_score = 0.0
        self.samples: list[ViewpointScore] = []
        self.candidates: list[ViewpointCandidate] = []
        self.failed_headings: list[float] = []

    @property
    def active(self) -> bool:
        return self.state in {
            RecoveryState.TRIGGERED,
            RecoveryState.SCANNING,
            RecoveryState.SAMPLING,
            RecoveryState.SELECTING,
            RecoveryState.RETURNING,
        }

    def recovery_trigger(
        self,
        navigation: NavigationState,
        goal: GoalDecision,
        avoidance: AvoidanceDecision | None,
    ) -> str | None:
        if goal.recovery_requested:
            return "No meaningful local-goal progress"
        if navigation.visual.visual_health_score < self.config.minimum_visual_health:
            return "Visual health below active-perception minimum"
        if navigation.depth.depth_health_score < self.config.minimum_depth_health:
            return "Depth health below active-perception minimum"
        if navigation.depth.temporal_stability < self.config.minimum_temporal_stability:
            return "Pseudo-depth temporal stability is low"
        if avoidance is not None:
            ambiguous = (
                avoidance.risk.centre >= avoidance.stop_threshold
                and abs(avoidance.risk.left - avoidance.risk.right) < 0.08
            )
            if ambiguous:
                return "Blocked centre has ambiguous side clearance"
            if avoidance.mode is AvoidanceMode.HOVER:
                return "Reactive avoidance has no immediate motion"
        return None

    def trigger(
        self,
        reason: str,
        navigation: NavigationState,
        goal_heading_deg: float,
        now_monotonic: float | None = None,
    ) -> bool:
        now = monotonic() if now_monotonic is None else now_monotonic
        if self.active:
            return True
        if now - self.last_completed_at < self.config.recovery_cooldown_s:
            return False
        if (
            self.attempt_count >= self.config.maximum_recovery_attempts
            or self.scan_count >= self.config.maximum_scans_per_goal
        ):
            self.state = RecoveryState.FAILED
            self.trigger_reason = "Recovery attempt limit exhausted"
            self.state_entered_at = now
            return False
        self.attempt_count += 1
        self.scan_count += 1
        self.trigger_reason = reason
        self.base_heading_deg = navigation.orientation.yaw_deg
        self.goal_heading_deg = goal_heading_deg
        self.selected_heading_deg = None
        self.target_heading_deg = None
        self.total_yaw_travelled_deg = 0.0
        self.last_heading_deg = navigation.orientation.yaw_deg
        self.scan_index = 0
        self.samples.clear()
        self.candidates.clear()
        self.settle_started_at = 0.0
        self.sample_started_at = 0.0
        self.heading_hold_started_at = 0.0
        self.last_depth_sequence = 0
        self.last_visual_update = 0.0
        baseline = self.scorer.score(
            navigation,
            self.base_heading_deg,
            self.goal_heading_deg,
            self.base_heading_deg,
            tuple(self.failed_headings),
        )
        self.baseline_score = baseline.score
        self.state = RecoveryState.TRIGGERED
        self.state_entered_at = now
        return True

    def update(
        self,
        navigation: NavigationState,
        now_monotonic: float | None = None,
    ) -> RecoveryDecision:
        now = monotonic() if now_monotonic is None else now_monotonic
        self._track_yaw(navigation.orientation.yaw_deg)
        if self.total_yaw_travelled_deg > self.config.maximum_total_recovery_yaw_deg:
            self._fail("Maximum total recovery yaw exceeded", now)
        if self.state is RecoveryState.TRIGGERED:
            self.state = RecoveryState.SCANNING
            self.state_entered_at = now

        if self.state is RecoveryState.SCANNING:
            if self.scan_index >= len(self.config.yaw_scan_offsets_deg):
                self.state = RecoveryState.SELECTING
                self.state_entered_at = now
            else:
                offset = self.config.yaw_scan_offsets_deg[self.scan_index]
                self.target_heading_deg = normalize_yaw(self.base_heading_deg + offset)
                error = shortest_yaw_delta(
                    self.target_heading_deg,
                    navigation.orientation.yaw_deg,
                )
                if abs(error) > self.config.heading_tolerance_deg:
                    self.settle_started_at = 0.0
                    return self._decision(
                        self._scan_action(offset),
                        self._yaw_command(error, now, "active-perception-scan"),
                        now,
                        ("Yawing to bounded scan heading",),
                    )
                if self.settle_started_at == 0.0:
                    self.settle_started_at = now
                if now - self.settle_started_at < self.config.settle_duration_s:
                    return self._decision(
                        RecoveryAction.HOVER_AND_OBSERVE,
                        DesiredMotion.hover(now, "active-perception-settle"),
                        now,
                        ("Settling at scan heading",),
                    )
                self.state = RecoveryState.SAMPLING
                self.state_entered_at = now
                self.sample_started_at = now
                self.samples.clear()
                self.last_depth_sequence = 0
                self.last_visual_update = 0.0

        if self.state is RecoveryState.SAMPLING:
            assert self.target_heading_deg is not None
            sampling_error = shortest_yaw_delta(
                self.target_heading_deg,
                navigation.orientation.yaw_deg,
            )
            if abs(sampling_error) > self.config.heading_tolerance_deg:
                self.state = RecoveryState.SCANNING
                self.settle_started_at = 0.0
                self.sample_started_at = 0.0
                self.samples.clear()
                return self._decision(
                    self._scan_action(
                        shortest_yaw_delta(
                            self.target_heading_deg,
                            self.base_heading_deg,
                        )
                    ),
                    self._yaw_command(
                        sampling_error,
                        now,
                        "active-perception-resettle",
                    ),
                    now,
                    ("Scan heading moved outside tolerance; resettling",),
                )
            self._sample(navigation)
            if now - self.sample_started_at >= self.config.sample_duration_s:
                self._finish_candidate()
                self.scan_index += 1
                self.settle_started_at = 0.0
                self.sample_started_at = 0.0
                self.state = RecoveryState.SCANNING
                self.state_entered_at = now
            return self._decision(
                RecoveryAction.HOVER_AND_OBSERVE,
                DesiredMotion.hover(now, "active-perception-sample"),
                now,
                ("Collecting fresh visual and depth samples",),
            )

        if self.state is RecoveryState.SELECTING:
            acceptable = [candidate for candidate in self.candidates if candidate.acceptable]
            if not acceptable:
                self.failed_headings.append(self.base_heading_deg)
                self._fail("No acceptable recovery viewpoint", now)
            else:
                best = max(acceptable, key=lambda candidate: candidate.median_score)
                progress_trigger = "progress" in self.trigger_reason.lower()
                if (
                    not progress_trigger
                    and best.median_score
                    < self.baseline_score + self.config.minimum_score_improvement
                ):
                    self.failed_headings.append(best.heading_deg)
                    self._fail("No viewpoint met minimum score improvement", now)
                else:
                    self.selected_heading_deg = best.heading_deg
                    self.target_heading_deg = best.heading_deg
                    self.state = RecoveryState.RETURNING
                    self.state_entered_at = now

        if self.state is RecoveryState.RETURNING:
            assert self.selected_heading_deg is not None
            error = shortest_yaw_delta(
                self.selected_heading_deg,
                navigation.orientation.yaw_deg,
            )
            if abs(error) > self.config.heading_tolerance_deg:
                self.heading_hold_started_at = 0.0
                return self._decision(
                    RecoveryAction.RETURN_TO_BEST_HEADING,
                    self._yaw_command(error, now, "active-perception-return"),
                    now,
                    ("Returning to selected viewpoint",),
                )
            if self.heading_hold_started_at == 0.0:
                self.heading_hold_started_at = now
            if now - self.heading_hold_started_at >= self.config.heading_hold_duration_s:
                self.state = RecoveryState.COMPLETE
                self.state_entered_at = now
                self.last_completed_at = now
            return self._decision(
                RecoveryAction.HOVER_AND_OBSERVE,
                DesiredMotion.hover(now, "active-perception-hold"),
                now,
                ("Holding selected recovery heading",),
            )

        if self.state in {RecoveryState.FAILED, RecoveryState.ABORT}:
            return self._decision(
                RecoveryAction.ABORT,
                DesiredMotion.hover(now, "active-perception-failed"),
                now,
                (self.trigger_reason or "Active-perception recovery failed",),
            )
        if self.state is RecoveryState.COMPLETE:
            return self._decision(
                RecoveryAction.NONE,
                DesiredMotion.hover(now, "active-perception-complete"),
                now,
                ("Selected viewpoint accepted",),
            )
        return self._decision(
            RecoveryAction.NONE,
            DesiredMotion.hover(now, "active-perception-idle"),
            now,
            ("Active perception idle",),
        )

    def reset_after_completion(self) -> None:
        if self.state is RecoveryState.COMPLETE:
            self.state = RecoveryState.IDLE

    def observation_hold_command(
        self,
        yaw_rate_deg_s: float,
        now_monotonic: float,
    ) -> DesiredMotion:
        """Own stationary mission observation through the existing yaw path."""
        yaw = max(
            -self.config.yaw_rate_deg_s,
            min(self.config.yaw_rate_deg_s, yaw_rate_deg_s),
        )
        return DesiredMotion(
            yaw_rate_deg_s=yaw,
            source="active-perception-observation",
            timestamp_monotonic=now_monotonic,
        )

    def _sample(self, navigation: NavigationState) -> None:
        depth_sequence = navigation.depth.source_frame_sequence
        visual_update = navigation.visual.last_visual_update_monotonic
        if (
            depth_sequence == 0
            or depth_sequence == self.last_depth_sequence
            or visual_update == 0.0
            or visual_update == self.last_visual_update
        ):
            return
        self.last_depth_sequence = depth_sequence
        self.last_visual_update = visual_update
        self.samples.append(
            self.scorer.score(
                navigation,
                navigation.orientation.yaw_deg,
                self.goal_heading_deg,
                self.base_heading_deg,
                tuple(self.failed_headings),
            )
        )

    def _finish_candidate(self) -> None:
        if not self.samples:
            self.candidates.append(
                ViewpointCandidate(
                    heading_deg=(
                        self.base_heading_deg
                        if self.target_heading_deg is None
                        else self.target_heading_deg
                    ),
                    median_score=0.0,
                    acceptable=False,
                    sample_count=0,
                    visual_health=0.0,
                    depth_health=0.0,
                    temporal_stability=0.0,
                    forward_risk=1.0,
                    reasons=("No fresh samples at heading",),
                )
            )
            return
        median_score = statistics.median(sample.score for sample in self.samples)
        acceptable = len(self.samples) >= 2 and all(sample.acceptable for sample in self.samples)
        reasons = tuple(dict.fromkeys(reason for sample in self.samples for reason in sample.reasons))
        self.candidates.append(
            ViewpointCandidate(
                heading_deg=(
                    self.base_heading_deg
                    if self.target_heading_deg is None
                    else self.target_heading_deg
                ),
                median_score=median_score,
                acceptable=acceptable,
                sample_count=len(self.samples),
                visual_health=statistics.median(sample.visual_health for sample in self.samples),
                depth_health=statistics.median(sample.depth_health for sample in self.samples),
                temporal_stability=statistics.median(
                    sample.temporal_stability for sample in self.samples
                ),
                forward_risk=1.0
                - statistics.median(sample.forward_clearance for sample in self.samples),
                reasons=reasons,
            )
        )

    def _track_yaw(self, heading_deg: float) -> None:
        if self.last_heading_deg is not None:
            self.total_yaw_travelled_deg += abs(
                shortest_yaw_delta(heading_deg, self.last_heading_deg)
            )
        self.last_heading_deg = heading_deg

    def _fail(self, reason: str, now: float) -> None:
        self.state = RecoveryState.FAILED
        self.trigger_reason = reason
        self.state_entered_at = now
        self.last_completed_at = now

    def _yaw_command(self, error: float, now: float, source: str) -> DesiredMotion:
        yaw_rate = max(
            -self.config.yaw_rate_deg_s,
            min(self.config.yaw_rate_deg_s, error),
        )
        return DesiredMotion(
            yaw_rate_deg_s=yaw_rate,
            source=source,
            timestamp_monotonic=now,
        )

    @staticmethod
    def _scan_action(offset: float) -> RecoveryAction:
        if offset < 0:
            return RecoveryAction.YAW_SCAN_LEFT
        if offset > 0:
            return RecoveryAction.YAW_SCAN_RIGHT
        return RecoveryAction.HOVER_AND_OBSERVE

    def _decision(
        self,
        action: RecoveryAction,
        command: DesiredMotion,
        now: float,
        reasons: tuple[str, ...],
    ) -> RecoveryDecision:
        return RecoveryDecision(
            state=self.state,
            action=action,
            command=command,
            trigger=self.trigger_reason,
            target_heading_deg=self.target_heading_deg,
            selected_heading_deg=self.selected_heading_deg,
            candidates=tuple(self.candidates),
            attempt_count=self.attempt_count,
            total_yaw_travelled_deg=self.total_yaw_travelled_deg,
            reasons=reasons,
            timestamp_monotonic=now,
        )
