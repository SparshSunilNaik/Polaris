from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields
from enum import Enum
from pathlib import Path
from typing import Any

from polaris.control.models import DesiredMotion


class NavigatorState(str, Enum):
    IDLE = "IDLE"
    ALIGN = "ALIGN"
    ADVANCE = "ADVANCE"
    AVOIDING = "AVOIDING"
    RECOVER_VIEW = "RECOVER_VIEW"
    VERIFY_PROGRESS = "VERIFY_PROGRESS"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"
    ABORT = "ABORT"
    LAND = "LAND"


class RecoveryState(str, Enum):
    IDLE = "IDLE"
    TRIGGERED = "TRIGGERED"
    SCANNING = "SCANNING"
    SAMPLING = "SAMPLING"
    SELECTING = "SELECTING"
    RETURNING = "RETURNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    ABORT = "ABORT"


class RecoveryAction(str, Enum):
    NONE = "NONE"
    HOVER_AND_OBSERVE = "HOVER_AND_OBSERVE"
    YAW_SCAN_LEFT = "YAW_SCAN_LEFT"
    YAW_SCAN_RIGHT = "YAW_SCAN_RIGHT"
    YAW_SCAN_BOTH = "YAW_SCAN_BOTH"
    SMALL_SIDESTEP_LEFT = "SMALL_SIDESTEP_LEFT"
    SMALL_SIDESTEP_RIGHT = "SMALL_SIDESTEP_RIGHT"
    RETURN_TO_BEST_HEADING = "RETURN_TO_BEST_HEADING"
    ABORT = "ABORT"


@dataclass(frozen=True)
class LocalGoal:
    goal_id: str
    target_north_m: float
    target_east_m: float
    target_down_m: float
    target_yaw_deg: float | None = None
    horizontal_tolerance_m: float = 0.5
    vertical_tolerance_m: float = 0.35
    timeout_s: float = 60.0
    maximum_path_length_m: float = 15.0
    maximum_displacement_m: float = 13.0
    completion_hold_s: float = 1.5
    source: str = "config"
    timestamp_monotonic: float = 0.0

    def validate(self) -> None:
        values = (
            self.target_north_m,
            self.target_east_m,
            self.target_down_m,
            self.horizontal_tolerance_m,
            self.vertical_tolerance_m,
            self.timeout_s,
            self.maximum_path_length_m,
            self.maximum_displacement_m,
            self.completion_hold_s,
            self.timestamp_monotonic,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Local goal values must be finite")
        if self.target_yaw_deg is not None and not math.isfinite(self.target_yaw_deg):
            raise ValueError("Local goal yaw must be finite")
        if any(
            value <= 0
            for value in (
                self.horizontal_tolerance_m,
                self.vertical_tolerance_m,
                self.timeout_s,
                self.maximum_path_length_m,
                self.maximum_displacement_m,
                self.completion_hold_s,
            )
        ):
            raise ValueError("Local goal tolerances and bounds must be positive")


@dataclass(frozen=True)
class LocalNavigationConfig:
    control_loop_hz: float = 10.0
    cruise_speed_m_s: float = 0.35
    approach_speed_m_s: float = 0.12
    maximum_lateral_speed_m_s: float = 0.18
    maximum_vertical_speed_m_s: float = 0.12
    approach_slowdown_distance_m: float = 2.0
    alignment_tolerance_deg: float = 12.0
    alignment_motion_limit_deg: float = 25.0
    yaw_rate_deg_s: float = 15.0
    completion_speed_m_s: float = 0.12
    horizontal_tolerance_m: float = 0.5
    vertical_tolerance_m: float = 0.35
    completion_hold_s: float = 1.5
    maximum_goal_duration_s: float = 60.0
    maximum_path_length_m: float = 15.0
    maximum_displacement_m: float = 13.0
    maximum_cross_track_error_m: float = 4.0
    progress_window_s: float = 4.0
    minimum_progress_m: float = 0.25
    no_progress_timeout_s: float = 6.0
    progress_timeout_after_recovery_s: float = 5.0
    takeoff_altitude_m: float = 2.5
    final_hover_s: float = 2.0
    maximum_touchdown_speed_m_s: float = 1.0
    maximum_touchdown_depth_m: float = 0.01
    collision_hard_override_enabled: bool = True
    automatic_landing_on_failure: bool = True
    goals: dict[str, dict[str, Any]] | None = None

    @classmethod
    def from_json(cls, path: str | Path) -> LocalNavigationConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown local-navigation settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        numeric = [
            value
            for item in fields(self)
            if isinstance((value := getattr(self, item.name)), (int, float))
            and not isinstance(value, bool)
        ]
        if not all(math.isfinite(float(value)) and value > 0 for value in numeric):
            raise ValueError("Local-navigation numeric settings must be finite and positive")
        if self.approach_speed_m_s > self.cruise_speed_m_s:
            raise ValueError("Approach speed must not exceed cruise speed")
        if self.alignment_tolerance_deg >= self.alignment_motion_limit_deg:
            raise ValueError("Alignment tolerance must be below motion limit")
        if not self.goals:
            raise ValueError("At least one bounded local goal is required")
        if not self.automatic_landing_on_failure:
            raise ValueError(
                "automatic_landing_on_failure must remain enabled for simulator safety"
            )
        if not self.collision_hard_override_enabled:
            raise ValueError(
                "collision_hard_override_enabled must remain enabled"
            )
        for goal_id, spec in self.goals.items():
            if not goal_id or not isinstance(spec, dict):
                raise ValueError("Goal specifications must be named objects")
            required = {"north_offset_m", "east_offset_m"}
            if not required.issubset(spec):
                raise ValueError(f"Goal {goal_id} is missing {sorted(required - set(spec))}")

    def build_goal(
        self,
        goal_id: str,
        origin_north_m: float,
        origin_east_m: float,
        target_down_m: float,
        now_monotonic: float,
    ) -> LocalGoal:
        if self.goals is None or goal_id not in self.goals:
            raise KeyError(f"Unknown local goal: {goal_id}")
        spec = self.goals[goal_id]
        goal = LocalGoal(
            goal_id=goal_id,
            target_north_m=origin_north_m + float(spec["north_offset_m"]),
            target_east_m=origin_east_m + float(spec["east_offset_m"]),
            target_down_m=target_down_m + float(spec.get("down_offset_m", 0.0)),
            target_yaw_deg=(
                None
                if spec.get("target_yaw_deg") is None
                else float(spec["target_yaw_deg"])
            ),
            horizontal_tolerance_m=float(
                spec.get("horizontal_tolerance_m", self.horizontal_tolerance_m)
            ),
            vertical_tolerance_m=float(
                spec.get("vertical_tolerance_m", self.vertical_tolerance_m)
            ),
            timeout_s=float(spec.get("timeout_s", self.maximum_goal_duration_s)),
            maximum_path_length_m=float(
                spec.get("maximum_path_length_m", self.maximum_path_length_m)
            ),
            maximum_displacement_m=float(
                spec.get("maximum_displacement_m", self.maximum_displacement_m)
            ),
            completion_hold_s=float(
                spec.get("completion_hold_s", self.completion_hold_s)
            ),
            source="local_navigation.json",
            timestamp_monotonic=now_monotonic,
        )
        goal.validate()
        return goal


@dataclass(frozen=True)
class GoalMetrics:
    initial_goal_distance_m: float
    goal_distance_m: float
    vertical_error_m: float
    goal_bearing_deg: float
    yaw_error_deg: float
    best_goal_distance_m: float
    progress_over_window_m: float
    time_without_progress_s: float
    path_length_proxy_m: float
    displacement_from_start_m: float
    hold_position_error_m: float
    cross_track_error_m: float
    elapsed_s: float
    avoidance_count: int
    recovery_count: int


@dataclass(frozen=True)
class GoalDecision:
    state: NavigatorState
    goal: LocalGoal
    requested_motion: DesiredMotion
    metrics: GoalMetrics
    recovery_requested: bool
    reasons: tuple[str, ...]
    timestamp_monotonic: float
    goal_reached: bool = False
    failed: bool = False


@dataclass(frozen=True)
class ActivePerceptionConfig:
    yaw_scan_offsets_deg: tuple[float, ...] = (-30.0, -15.0, 0.0, 15.0, 30.0)
    yaw_rate_deg_s: float = 15.0
    heading_tolerance_deg: float = 4.0
    heading_hold_duration_s: float = 0.4
    settle_duration_s: float = 0.5
    sample_duration_s: float = 0.8
    maximum_recovery_attempts: int = 2
    maximum_scans_per_goal: int = 2
    recovery_cooldown_s: float = 4.0
    maximum_total_recovery_yaw_deg: float = 180.0
    minimum_score_improvement: float = 0.04
    minimum_acceptable_score: float = 0.52
    minimum_visual_health: float = 0.50
    minimum_depth_health: float = 0.45
    minimum_temporal_stability: float = 0.72
    feature_count_reference: float = 120.0
    failure_heading_radius_deg: float = 12.0
    visual_weight: float = 0.24
    depth_weight: float = 0.15
    stability_weight: float = 0.14
    clearance_weight: float = 0.19
    feature_weight: float = 0.10
    exposure_weight: float = 0.06
    goal_alignment_weight: float = 0.09
    yaw_change_weight: float = 0.03
    failure_history_penalty: float = 0.12

    @classmethod
    def from_json(cls, path: str | Path) -> ActivePerceptionConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        if "yaw_scan_offsets_deg" in values:
            values["yaw_scan_offsets_deg"] = tuple(values["yaw_scan_offsets_deg"])
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown active-perception settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        if not self.yaw_scan_offsets_deg or 0.0 not in self.yaw_scan_offsets_deg:
            raise ValueError("Yaw scan offsets must include the current heading")
        if len(set(self.yaw_scan_offsets_deg)) != len(self.yaw_scan_offsets_deg):
            raise ValueError("Yaw scan offsets must be unique")
        bounded = (
            self.minimum_score_improvement,
            self.minimum_acceptable_score,
            self.minimum_visual_health,
            self.minimum_depth_health,
            self.minimum_temporal_stability,
        )
        if any(value < 0 or value > 1 for value in bounded):
            raise ValueError("Active-perception thresholds must be in [0,1]")
        if self.maximum_recovery_attempts < 1 or self.maximum_scans_per_goal < 1:
            raise ValueError("Recovery attempt limits must be positive")
        positive = (
            self.yaw_rate_deg_s,
            self.heading_tolerance_deg,
            self.heading_hold_duration_s,
            self.settle_duration_s,
            self.sample_duration_s,
            self.recovery_cooldown_s,
            self.maximum_total_recovery_yaw_deg,
            self.feature_count_reference,
        )
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("Recovery timing and motion limits must be positive")
        weights = (
            self.visual_weight,
            self.depth_weight,
            self.stability_weight,
            self.clearance_weight,
            self.feature_weight,
            self.exposure_weight,
            self.goal_alignment_weight,
            self.yaw_change_weight,
        )
        if not math.isclose(sum(weights), 1.0, abs_tol=1e-6):
            raise ValueError("Viewpoint score weights must sum to 1")


@dataclass(frozen=True)
class ViewpointScore:
    heading_deg: float
    score: float
    acceptable: bool
    visual_health: float
    depth_health: float
    temporal_stability: float
    forward_clearance: float
    feature_quality: float
    exposure_quality: float
    goal_alignment: float
    yaw_change_cost: float
    failure_penalty: float
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ViewpointCandidate:
    heading_deg: float
    median_score: float
    acceptable: bool
    sample_count: int
    visual_health: float
    depth_health: float
    temporal_stability: float
    forward_risk: float
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class RecoveryDecision:
    state: RecoveryState
    action: RecoveryAction
    command: DesiredMotion
    trigger: str
    target_heading_deg: float | None
    selected_heading_deg: float | None
    candidates: tuple[ViewpointCandidate, ...]
    attempt_count: int
    total_yaw_travelled_deg: float
    reasons: tuple[str, ...]
    timestamp_monotonic: float
