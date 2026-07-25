from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields
from enum import Enum
from pathlib import Path
from typing import Any

from polaris.navigation.models import LocalGoal


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    MISSION_START = "MISSION_START"
    WAYPOINT_PREPARE = "WAYPOINT_PREPARE"
    WAYPOINT_EXECUTE = "WAYPOINT_EXECUTE"
    WAYPOINT_VERIFY = "WAYPOINT_VERIFY"
    WAYPOINT_COMPLETE = "WAYPOINT_COMPLETE"
    WAYPOINT_FAILED = "WAYPOINT_FAILED"
    REPLAN = "REPLAN"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    RETURN_OR_LAND = "RETURN_OR_LAND"
    ABORT = "ABORT"
    LAND = "LAND"
    FINISHED = "FINISHED"


class MissionResult(str, Enum):
    PENDING = "PENDING"
    COMPLETED = "COMPLETED"
    COMPLETED_WITH_SKIPS = "COMPLETED_WITH_SKIPS"
    FAILED = "FAILED"
    ABORTED = "ABORTED"
    LANDED = "LANDED"


class WaypointType(str, Enum):
    POSITION = "POSITION"
    OBSERVATION = "OBSERVATION"


class WaypointCompletionMode(str, Enum):
    POSITION = "POSITION"
    OBSERVATION = "OBSERVATION"


class ObservationState(str, Enum):
    APPROACHING = "APPROACHING"
    OBSERVATION_CANDIDATE = "OBSERVATION_CANDIDATE"
    OBSERVING = "OBSERVING"
    OBSERVATION_COMPLETE = "OBSERVATION_COMPLETE"


class ObservationEventType(str, Enum):
    CANDIDATE_ENTERED = "CANDIDATE_ENTERED"
    DWELL_STARTED = "DWELL_STARTED"
    DWELL_RESET = "DWELL_RESET"
    COMPLETED = "COMPLETED"
    TIMED_OUT = "TIMED_OUT"


class FailurePolicy(str, Enum):
    ABORT_MISSION = "ABORT_MISSION"
    RETRY_WAYPOINT = "RETRY_WAYPOINT"
    SKIP_OPTIONAL = "SKIP_OPTIONAL"
    REPLAN_AROUND_WAYPOINT = "REPLAN_AROUND_WAYPOINT"
    LAND_IMMEDIATELY = "LAND_IMMEDIATELY"


class FailureAction(str, Enum):
    USE_MISSION_POLICY = "USE_MISSION_POLICY"
    ABORT_MISSION = "ABORT_MISSION"
    RETRY_WAYPOINT = "RETRY_WAYPOINT"
    SKIP_OPTIONAL = "SKIP_OPTIONAL"
    REPLAN_AROUND_WAYPOINT = "REPLAN_AROUND_WAYPOINT"
    LAND_IMMEDIATELY = "LAND_IMMEDIATELY"


class ReturnOrLandPolicy(str, Enum):
    LAND_AT_CURRENT_POSITION = "LAND_AT_CURRENT_POSITION"
    RETURN_TO_START_THEN_LAND = "RETURN_TO_START_THEN_LAND"


class MissionAction(str, Enum):
    NONE = "NONE"
    START_WAYPOINT = "START_WAYPOINT"
    RETRY_WAYPOINT = "RETRY_WAYPOINT"
    SKIP_WAYPOINT = "SKIP_WAYPOINT"
    REQUEST_REPLAN = "REQUEST_REPLAN"
    COMPLETE_MISSION = "COMPLETE_MISSION"
    ABORT_MISSION = "ABORT_MISSION"
    LAND = "LAND"


@dataclass(frozen=True)
class WorldBounds:
    north_min_m: float
    north_max_m: float
    east_min_m: float
    east_max_m: float
    down_offset_min_m: float
    down_offset_max_m: float

    def contains(self, north_m: float, east_m: float, down_offset_m: float) -> bool:
        return (
            self.north_min_m <= north_m <= self.north_max_m
            and self.east_min_m <= east_m <= self.east_max_m
            and self.down_offset_min_m <= down_offset_m <= self.down_offset_max_m
        )


@dataclass(frozen=True)
class MissionOrchestratorConfig:
    preflight_timeout_s: float = 90.0
    takeoff_altitude_m: float = 2.5
    waypoint_verification_duration_s: float = 1.0
    maximum_mission_duration_s: float = 150.0
    maximum_mission_path_length_m: float = 18.0
    maximum_waypoint_attempts: int = 2
    maximum_replans: int = 1
    maximum_generated_waypoint_offset_m: float = 2.0
    generated_waypoint_timeout_s: float = 20.0
    lateral_replan_offset_m: float = 1.5
    pause_between_waypoints_s: float = 1.0
    return_or_land_policy: str = ReturnOrLandPolicy.LAND_AT_CURRENT_POSITION.value
    automatic_landing: bool = True
    artifact_sampling_hz: float = 5.0
    world_bounds: WorldBounds = field(
        default_factory=lambda: WorldBounds(-18.0, 18.0, -8.0, 13.0, -0.5, 0.5)
    )

    @classmethod
    def from_json(cls, path: str | Path) -> MissionOrchestratorConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        if "world_bounds" in values:
            values["world_bounds"] = WorldBounds(**values["world_bounds"])
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown mission-orchestrator settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        positive = (
            self.preflight_timeout_s,
            self.takeoff_altitude_m,
            self.waypoint_verification_duration_s,
            self.maximum_mission_duration_s,
            self.maximum_mission_path_length_m,
            self.maximum_generated_waypoint_offset_m,
            self.generated_waypoint_timeout_s,
            self.lateral_replan_offset_m,
            self.pause_between_waypoints_s,
            self.artifact_sampling_hz,
        )
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("Mission timing and bounds must be finite and positive")
        if self.maximum_waypoint_attempts < 1 or self.maximum_replans < 0:
            raise ValueError("Mission attempt and replan limits are invalid")
        if self.lateral_replan_offset_m > self.maximum_generated_waypoint_offset_m:
            raise ValueError("Lateral replan offset exceeds generated-waypoint bound")
        ReturnOrLandPolicy(self.return_or_land_policy)
        if not self.automatic_landing:
            raise ValueError("automatic_landing must remain enabled for simulator safety")
        bounds = self.world_bounds
        if (
            bounds.north_min_m >= bounds.north_max_m
            or bounds.east_min_m >= bounds.east_max_m
            or bounds.down_offset_min_m >= bounds.down_offset_max_m
        ):
            raise ValueError("Mission world bounds are invalid")


@dataclass(frozen=True)
class ObservationCompletionConfig:
    minimum_goal_distance_m: float = 2.5
    maximum_goal_distance_m: float = 3.0
    maximum_heading_error_deg: float = 12.0
    minimum_visual_health: float = 0.35
    minimum_depth_health: float = 0.40
    minimum_depth_stability: float = 0.75
    maximum_centre_risk: float = 0.45
    maximum_horizontal_speed_m_s: float = 0.20
    required_stable_s: float = 1.0
    minimum_fresh_observations: int = 3
    maximum_wait_s: float = 6.0

    def validate(self) -> None:
        values = tuple(getattr(self, item.name) for item in fields(self))
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise ValueError("Observation completion values must be finite and non-negative")
        if self.minimum_goal_distance_m >= self.maximum_goal_distance_m:
            raise ValueError("Observation distance band must have increasing bounds")
        if self.maximum_heading_error_deg <= 0:
            raise ValueError("Observation heading bound must be positive")
        health = (
            self.minimum_visual_health,
            self.minimum_depth_health,
            self.minimum_depth_stability,
            self.maximum_centre_risk,
        )
        if any(value < 0 or value > 1 for value in health):
            raise ValueError("Observation health, stability, and risk must be in [0,1]")
        if (
            self.maximum_horizontal_speed_m_s <= 0
            or self.required_stable_s <= 0
            or self.maximum_wait_s <= 0
        ):
            raise ValueError("Observation speed and timing bounds must be positive")
        if self.minimum_fresh_observations < 2:
            raise ValueError("Observation completion requires at least two fresh frames")


@dataclass(frozen=True)
class ObservationEvent:
    event_type: ObservationEventType
    timestamp_monotonic: float
    waypoint_id: str
    reason: str
    goal_distance_m: float
    heading_error_deg: float
    visual_health: float
    depth_health: float
    depth_stability: float
    centre_risk: float
    horizontal_speed_m_s: float
    dwell_elapsed_s: float
    fresh_observation_count: int
    depth_frame_sequence: int
    depth_inference_timestamp_monotonic: float


@dataclass(frozen=True)
class ObservationDecision:
    state: ObservationState
    waypoint_type: WaypointType
    completion_mode: WaypointCompletionMode | None
    hold_required: bool
    completion_ready: bool
    completed: bool
    timed_out: bool
    reason: str
    goal_distance_m: float
    heading_error_deg: float
    visual_health: float
    depth_health: float
    depth_stability: float
    centre_risk: float
    horizontal_speed_m_s: float
    dwell_elapsed_s: float
    fresh_observation_count: int
    wait_elapsed_s: float
    depth_frame_sequence: int
    depth_inference_timestamp_monotonic: float
    config: ObservationCompletionConfig


@dataclass(frozen=True)
class MissionWaypoint:
    waypoint_id: str
    goal: LocalGoal
    required: bool = True
    skippable: bool = False
    maximum_attempts: int = 1
    verification_hold_s: float = 1.0
    minimum_visual_health: float = 0.35
    minimum_depth_health: float = 0.40
    preferred_terminal_yaw_deg: float | None = None
    failure_action: FailureAction = FailureAction.USE_MISSION_POLICY
    metadata: dict[str, Any] = field(default_factory=dict)
    generated: bool = False
    waypoint_type: WaypointType = WaypointType.POSITION
    observation: ObservationCompletionConfig | None = None

    def validate(self, bounds: WorldBounds) -> None:
        if not self.waypoint_id:
            raise ValueError("Waypoint ID must not be empty")
        self.goal.validate()
        if self.waypoint_type is WaypointType.OBSERVATION:
            if self.observation is None:
                raise ValueError("Observation waypoint requires observation configuration")
            self.observation.validate()
        elif self.observation is not None:
            raise ValueError("Position waypoint cannot define observation configuration")
        if self.required and self.skippable:
            raise ValueError("A required waypoint cannot be skippable")
        if not self.required and not self.skippable:
            raise ValueError("An optional waypoint must explicitly be skippable")
        if self.maximum_attempts < 1 or self.verification_hold_s <= 0:
            raise ValueError("Waypoint attempts and verification hold must be positive")
        health = (self.minimum_visual_health, self.minimum_depth_health)
        if any(value < 0 or value > 1 for value in health):
            raise ValueError("Waypoint health thresholds must be in [0,1]")
        if self.preferred_terminal_yaw_deg is not None and not math.isfinite(
            self.preferred_terminal_yaw_deg
        ):
            raise ValueError("Preferred terminal yaw must be finite")
        down_offset = self.goal.target_down_m
        if not bounds.contains(
            self.goal.target_north_m,
            self.goal.target_east_m,
            down_offset,
        ):
            raise ValueError(f"Waypoint {self.waypoint_id} lies outside mission bounds")


@dataclass(frozen=True)
class MissionPlan:
    mission_id: str
    name: str
    description: str
    waypoints: tuple[MissionWaypoint, ...]
    failure_policy: FailurePolicy
    maximum_duration_s: float
    maximum_path_length_m: float
    maximum_replans: int
    return_or_land_policy: ReturnOrLandPolicy
    source: str
    timestamp_monotonic: float = 0.0

    def validate(self, config: MissionOrchestratorConfig) -> None:
        if not self.mission_id or not self.name:
            raise ValueError("Mission ID and name are required")
        if not self.waypoints:
            raise ValueError("Mission must contain at least one waypoint")
        ids = [waypoint.waypoint_id for waypoint in self.waypoints]
        if len(ids) != len(set(ids)):
            raise ValueError("Mission waypoint IDs must be unique")
        if (
            not math.isfinite(self.maximum_duration_s)
            or self.maximum_duration_s <= 0
            or self.maximum_duration_s > config.maximum_mission_duration_s
        ):
            raise ValueError("Mission duration exceeds configured bound")
        if (
            not math.isfinite(self.maximum_path_length_m)
            or self.maximum_path_length_m <= 0
            or self.maximum_path_length_m > config.maximum_mission_path_length_m
        ):
            raise ValueError("Mission path-length limit exceeds configured bound")
        if self.maximum_replans < 0 or self.maximum_replans > config.maximum_replans:
            raise ValueError("Mission replan limit exceeds configured bound")
        if self.return_or_land_policy is not ReturnOrLandPolicy.LAND_AT_CURRENT_POSITION:
            raise ValueError("Only LAND_AT_CURRENT_POSITION is implemented in this milestone")
        for waypoint in self.waypoints:
            waypoint.validate(config.world_bounds)


@dataclass
class MissionState:
    mission_id: str = "NONE"
    mission_name: str = "NONE"
    state: MissionPhase = MissionPhase.IDLE
    current_waypoint_index: int = -1
    current_waypoint_id: str = "NONE"
    total_waypoints: int = 0
    completed_waypoints: list[str] = field(default_factory=list)
    skipped_waypoints: list[str] = field(default_factory=list)
    failed_waypoints: list[str] = field(default_factory=list)
    generated_waypoints: list[str] = field(default_factory=list)
    waypoint_attempts: dict[str, int] = field(default_factory=dict)
    waypoint_completion_modes: dict[str, str] = field(default_factory=dict)
    current_waypoint_type: str = WaypointType.POSITION.value
    current_completion_mode: str = "NONE"
    current_attempt: int = 0
    replan_count: int = 0
    mission_elapsed_s: float = 0.0
    total_path_length_proxy_m: float = 0.0
    displacement_from_start_m: float = 0.0
    current_goal_distance_m: float = 0.0
    active_failure_policy: str = "NONE"
    generated_replan_waypoint: str = "NONE"
    abort_reason: str | None = None
    landing_result: str = "NOT_ATTEMPTED"
    result: MissionResult = MissionResult.PENDING

    @property
    def progress_percent(self) -> float:
        if self.total_waypoints <= 0:
            return 0.0
        resolved = len(self.completed_waypoints) + len(self.skipped_waypoints)
        return min(100.0, 100.0 * resolved / self.total_waypoints)


@dataclass(frozen=True)
class OrchestratorDecision:
    action: MissionAction
    state: MissionPhase
    reason: str
    waypoint: MissionWaypoint | None = None


@dataclass(frozen=True)
class ReplanDecision:
    accepted: bool
    reason: str
    generated_waypoint: MissionWaypoint | None
    strategy: str
