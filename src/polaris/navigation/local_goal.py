from __future__ import annotations

import math
from collections import deque
from time import monotonic

from polaris.control.models import AvoidanceMode, DesiredMotion
from polaris.ego_state.models import NavigationState
from polaris.evaluation.viewpoint import normalize_yaw, shortest_yaw_delta

from .models import (
    GoalDecision,
    GoalMetrics,
    LocalGoal,
    LocalNavigationConfig,
    NavigatorState,
)


def ned_error_to_body(
    north_error_m: float,
    east_error_m: float,
    yaw_deg: float,
) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    forward = math.cos(yaw) * north_error_m + math.sin(yaw) * east_error_m
    right = -math.sin(yaw) * north_error_m + math.cos(yaw) * east_error_m
    return forward, right


class LocalGoalNavigator:
    def __init__(self, config: LocalNavigationConfig) -> None:
        self.config = config
        self.state = NavigatorState.IDLE
        self.goal: LocalGoal | None = None
        self.started_at = 0.0
        self.start_north_m = 0.0
        self.start_east_m = 0.0
        self.start_down_m = 0.0
        self.initial_goal_distance_m = 0.0
        self.best_goal_distance_m = float("inf")
        self.progress_reference_distance_m = float("inf")
        self.last_progress_at = 0.0
        self.path_length_proxy_m = 0.0
        self.last_position: tuple[float, float] | None = None
        self.last_position_update = 0.0
        self.hold_started_at = 0.0
        self.avoidance_count = 0
        self.recovery_count = 0
        self._last_avoidance_mode = AvoidanceMode.CLEAR
        self._progress_samples: deque[tuple[float, float]] = deque()
        self.preferred_heading_deg: float | None = None
        self.recovery_baseline_distance_m = 0.0
        self.recovery_progress_deadline = 0.0
        self.awaiting_recovery_progress = False

    def start(
        self,
        goal: LocalGoal,
        navigation: NavigationState,
        now_monotonic: float | None = None,
    ) -> None:
        now = monotonic() if now_monotonic is None else now_monotonic
        goal.validate()
        self.goal = goal
        self.started_at = now
        self.start_north_m = navigation.position_ned_m.x
        self.start_east_m = navigation.position_ned_m.y
        self.start_down_m = navigation.position_ned_m.z
        self.initial_goal_distance_m = math.hypot(
            goal.target_north_m - self.start_north_m,
            goal.target_east_m - self.start_east_m,
        )
        self.best_goal_distance_m = self.initial_goal_distance_m
        self.progress_reference_distance_m = self.initial_goal_distance_m
        self.last_progress_at = now
        self.path_length_proxy_m = 0.0
        self.last_position = (self.start_north_m, self.start_east_m)
        self.last_position_update = navigation.last_position_update_monotonic
        self.hold_started_at = 0.0
        self.avoidance_count = 0
        self.recovery_count = 0
        self._last_avoidance_mode = AvoidanceMode.CLEAR
        self._progress_samples.clear()
        self.preferred_heading_deg = None
        self.awaiting_recovery_progress = False
        self.state = NavigatorState.ALIGN

    def update(
        self,
        navigation: NavigationState,
        *,
        now_monotonic: float | None = None,
        avoidance_mode: AvoidanceMode = AvoidanceMode.CLEAR,
        recovery_active: bool = False,
    ) -> GoalDecision:
        now = monotonic() if now_monotonic is None else now_monotonic
        if self.goal is None:
            raise RuntimeError("LocalGoalNavigator.start() must be called first")
        self._update_path(navigation)
        self._track_avoidance(avoidance_mode)
        metrics = self._metrics(navigation, now)

        failure = None if self.state is NavigatorState.GOAL_REACHED else self._bounded_failure(metrics)
        if failure is not None:
            self.state = NavigatorState.FAILED
            return self._decision(
                DesiredMotion.hover(now, "local-goal-failed"),
                metrics,
                now,
                (failure,),
                failed=True,
            )

        position_inside = (
            metrics.goal_distance_m <= self.goal.horizontal_tolerance_m
            and abs(metrics.vertical_error_m) <= self.goal.vertical_tolerance_m
            and navigation.horizontal_speed_m_s <= self.config.completion_speed_m_s
        )
        terminal_yaw_ok = (
            self.goal.target_yaw_deg is None
            or abs(metrics.yaw_error_deg) <= self.config.alignment_tolerance_deg
        )
        inside = position_inside and terminal_yaw_ok
        if position_inside and not terminal_yaw_ok:
            self.hold_started_at = 0.0
            self.state = NavigatorState.ALIGN
            return self._decision(
                DesiredMotion(
                    yaw_rate_deg_s=max(
                        -self.config.yaw_rate_deg_s,
                        min(self.config.yaw_rate_deg_s, metrics.yaw_error_deg),
                    ),
                    source="local-goal-terminal-yaw",
                    timestamp_monotonic=now,
                ),
                metrics,
                now,
                ("Holding position while aligning terminal yaw",),
            )
        if inside:
            if self.hold_started_at == 0.0:
                self.hold_started_at = now
            self.state = NavigatorState.VERIFY_PROGRESS
            if now - self.hold_started_at >= self.goal.completion_hold_s:
                self.state = NavigatorState.GOAL_REACHED
                return self._decision(
                    DesiredMotion.hover(now, "goal-reached"),
                    metrics,
                    now,
                    ("Goal tolerance and hold duration satisfied",),
                    goal_reached=True,
                )
            return self._decision(
                DesiredMotion.hover(now, "goal-hold"),
                metrics,
                now,
                ("Holding inside goal tolerance",),
            )
        self.hold_started_at = 0.0

        if self.awaiting_recovery_progress:
            progress = self.recovery_baseline_distance_m - metrics.best_goal_distance_m
            if progress >= self.config.minimum_progress_m:
                self.awaiting_recovery_progress = False
            elif now >= self.recovery_progress_deadline:
                self.state = NavigatorState.RECOVER_VIEW
                return self._decision(
                    DesiredMotion.hover(now, "post-recovery-no-progress"),
                    metrics,
                    now,
                    ("No meaningful progress after recovery",),
                    recovery_requested=True,
                )

        if recovery_active:
            self.state = NavigatorState.RECOVER_VIEW
            return self._decision(
                DesiredMotion.hover(now, "active-perception"),
                metrics,
                now,
                ("Active-perception recovery owns motion",),
            )

        if (
            metrics.time_without_progress_s >= self.config.no_progress_timeout_s
            and self.state in {NavigatorState.ADVANCE, NavigatorState.AVOIDING}
        ):
            self.state = NavigatorState.RECOVER_VIEW
            return self._decision(
                DesiredMotion.hover(now, "no-progress"),
                metrics,
                now,
                ("No meaningful goal progress",),
                recovery_requested=True,
            )

        if avoidance_mode not in {AvoidanceMode.CLEAR, AvoidanceMode.CAUTION}:
            self.state = NavigatorState.AVOIDING
        else:
            yaw_error = metrics.yaw_error_deg
            self.state = (
                NavigatorState.ALIGN
                if abs(yaw_error) > self.config.alignment_motion_limit_deg
                else NavigatorState.ADVANCE
            )

        command = self._motion(metrics, navigation.orientation.yaw_deg, now)
        return self._decision(
            command,
            metrics,
            now,
            ("Bounded goal command generated",),
        )

    def complete_recovery(
        self,
        success: bool,
        now_monotonic: float,
        selected_heading_deg: float | None = None,
    ) -> None:
        self.recovery_count += 1
        if success:
            self.last_progress_at = now_monotonic
            self.progress_reference_distance_m = self.best_goal_distance_m
            self.preferred_heading_deg = selected_heading_deg
            self.recovery_baseline_distance_m = self.best_goal_distance_m
            self.recovery_progress_deadline = (
                now_monotonic + self.config.progress_timeout_after_recovery_s
            )
            self.awaiting_recovery_progress = True
            self.state = NavigatorState.VERIFY_PROGRESS
        else:
            self.state = NavigatorState.FAILED

    def abort(self) -> None:
        self.state = NavigatorState.ABORT

    def _motion(
        self,
        metrics: GoalMetrics,
        current_yaw_deg: float,
        now: float,
    ) -> DesiredMotion:
        yaw_error = metrics.yaw_error_deg
        yaw_rate = max(
            -self.config.yaw_rate_deg_s,
            min(self.config.yaw_rate_deg_s, yaw_error),
        )
        if abs(yaw_error) > self.config.alignment_motion_limit_deg:
            return DesiredMotion(
                yaw_rate_deg_s=yaw_rate,
                source="local-goal-align",
                timestamp_monotonic=now,
            )
        speed_fraction = min(
            1.0,
            metrics.goal_distance_m / self.config.approach_slowdown_distance_m,
        )
        speed = self.config.approach_speed_m_s + (
            self.config.cruise_speed_m_s - self.config.approach_speed_m_s
        ) * speed_fraction
        heading = math.radians(metrics.goal_bearing_deg)
        north_velocity = math.cos(heading) * speed
        east_velocity = math.sin(heading) * speed
        forward, right = ned_error_to_body(
            north_velocity,
            east_velocity,
            current_yaw_deg,
        )
        right = max(
            -self.config.maximum_lateral_speed_m_s,
            min(self.config.maximum_lateral_speed_m_s, right),
        )
        down = max(
            -self.config.maximum_vertical_speed_m_s,
            min(
                self.config.maximum_vertical_speed_m_s,
                metrics.vertical_error_m,
            ),
        )
        return DesiredMotion(
            forward_m_s=max(0.0, forward),
            right_m_s=right,
            down_m_s=down,
            yaw_rate_deg_s=yaw_rate,
            source="local-goal-advance",
            timestamp_monotonic=now,
        )

    def _metrics(self, navigation: NavigationState, now: float) -> GoalMetrics:
        assert self.goal is not None
        north = navigation.position_ned_m.x
        east = navigation.position_ned_m.y
        north_error = self.goal.target_north_m - north
        east_error = self.goal.target_east_m - east
        distance = math.hypot(north_error, east_error)
        bearing = normalize_yaw(math.degrees(math.atan2(east_error, north_error)))
        yaw_target = (
            self.goal.target_yaw_deg
            if distance <= self.goal.horizontal_tolerance_m
            and self.goal.target_yaw_deg is not None
            else (
                self.preferred_heading_deg
                if self.preferred_heading_deg is not None
                else bearing
            )
        )
        yaw_error = shortest_yaw_delta(yaw_target, navigation.orientation.yaw_deg)
        vertical_error = self.goal.target_down_m - navigation.position_ned_m.z
        self.best_goal_distance_m = min(self.best_goal_distance_m, distance)
        if distance <= self.progress_reference_distance_m - self.config.minimum_progress_m:
            self.progress_reference_distance_m = distance
            self.last_progress_at = now
        self._progress_samples.append((now, distance))
        while self._progress_samples and now - self._progress_samples[0][0] > self.config.progress_window_s:
            self._progress_samples.popleft()
        progress = (
            0.0
            if len(self._progress_samples) < 2
            else self._progress_samples[0][1] - self._progress_samples[-1][1]
        )
        displacement = math.hypot(north - self.start_north_m, east - self.start_east_m)
        cross_track = self._cross_track(north, east)
        return GoalMetrics(
            initial_goal_distance_m=self.initial_goal_distance_m,
            goal_distance_m=distance,
            vertical_error_m=vertical_error,
            goal_bearing_deg=bearing,
            yaw_error_deg=yaw_error,
            best_goal_distance_m=self.best_goal_distance_m,
            progress_over_window_m=progress,
            time_without_progress_s=max(0.0, now - self.last_progress_at),
            path_length_proxy_m=self.path_length_proxy_m,
            displacement_from_start_m=displacement,
            hold_position_error_m=math.hypot(distance, vertical_error),
            cross_track_error_m=cross_track,
            elapsed_s=max(0.0, now - self.started_at),
            avoidance_count=self.avoidance_count,
            recovery_count=self.recovery_count,
        )

    def _update_path(self, navigation: NavigationState) -> None:
        if navigation.last_position_update_monotonic == self.last_position_update:
            return
        current = (navigation.position_ned_m.x, navigation.position_ned_m.y)
        if self.last_position is not None:
            self.path_length_proxy_m += math.hypot(
                current[0] - self.last_position[0],
                current[1] - self.last_position[1],
            )
        self.last_position = current
        self.last_position_update = navigation.last_position_update_monotonic

    def _cross_track(self, north: float, east: float) -> float:
        assert self.goal is not None
        line_n = self.goal.target_north_m - self.start_north_m
        line_e = self.goal.target_east_m - self.start_east_m
        length = math.hypot(line_n, line_e)
        if length == 0:
            return 0.0
        point_n = north - self.start_north_m
        point_e = east - self.start_east_m
        return abs(line_e * point_n - line_n * point_e) / length

    def _bounded_failure(self, metrics: GoalMetrics) -> str | None:
        assert self.goal is not None
        if metrics.elapsed_s > self.goal.timeout_s:
            return "Local goal timeout exceeded"
        if metrics.path_length_proxy_m > self.goal.maximum_path_length_m:
            return "Path-length proxy exceeded goal bound"
        if metrics.displacement_from_start_m > self.goal.maximum_displacement_m:
            return "Displacement from start exceeded goal bound"
        if metrics.cross_track_error_m > self.config.maximum_cross_track_error_m:
            return "Cross-track error exceeded configured bound"
        return None

    def _track_avoidance(self, mode: AvoidanceMode) -> None:
        if (
            mode not in {AvoidanceMode.CLEAR, AvoidanceMode.CAUTION}
            and self._last_avoidance_mode in {AvoidanceMode.CLEAR, AvoidanceMode.CAUTION}
        ):
            self.avoidance_count += 1
        self._last_avoidance_mode = mode

    def _decision(
        self,
        command: DesiredMotion,
        metrics: GoalMetrics,
        now: float,
        reasons: tuple[str, ...],
        *,
        recovery_requested: bool = False,
        goal_reached: bool = False,
        failed: bool = False,
    ) -> GoalDecision:
        assert self.goal is not None
        return GoalDecision(
            state=self.state,
            goal=self.goal,
            requested_motion=command,
            metrics=metrics,
            recovery_requested=recovery_requested,
            reasons=reasons,
            timestamp_monotonic=now,
            goal_reached=goal_reached,
            failed=failed,
        )
