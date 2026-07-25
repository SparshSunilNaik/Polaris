from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping

from polaris.vlm import FusedSceneContext

from .semantic_validation import SemanticValidationRecord


def _tuple_of_strings(values: tuple[str, ...] | list[str]) -> tuple[str, ...]:
    result = tuple(str(value) for value in values)
    if any(not value for value in result):
        raise ValueError("Context labels must not be empty")
    return result


def _unit_interval(value: float, name: str) -> None:
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be finite and in [0,1]")


@dataclass(frozen=True, slots=True)
class MissionContext:
    mission_id: str = "NONE"
    mission_name: str = "NONE"
    phase: str = "IDLE"
    result: str = "PENDING"
    current_objective: str = "NONE"
    current_waypoint_id: str = "NONE"
    current_waypoint_index: int = -1
    total_waypoints: int = 0
    current_attempt: int = 0
    waypoint_type: str = "POSITION"
    completion_mode: str = "NONE"
    completed_waypoints: tuple[str, ...] = ()
    skipped_waypoints: tuple[str, ...] = ()
    failed_waypoints: tuple[str, ...] = ()
    observation_status: str = "NOT_APPLICABLE"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "completed_waypoints",
            _tuple_of_strings(self.completed_waypoints),
        )
        object.__setattr__(
            self,
            "skipped_waypoints",
            _tuple_of_strings(self.skipped_waypoints),
        )
        object.__setattr__(
            self,
            "failed_waypoints",
            _tuple_of_strings(self.failed_waypoints),
        )


@dataclass(frozen=True, slots=True)
class NavigationContext:
    navigator_state: str = "IDLE"
    recovery_state: str = "IDLE"
    active_perception_owns_motion: bool = False
    strategy: str = "NONE"
    goal_distance_m: float | None = None
    heading_error_deg: float | None = None
    avoidance_state: str = "CLEAR"
    avoidance_episode_count: int = 0
    recovery_count: int = 0
    no_progress: bool = False
    retry_count: int = 0
    recent_recoveries: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.retry_count < 0:
            raise ValueError("retry_count must be non-negative")
        if self.avoidance_episode_count < 0 or self.recovery_count < 0:
            raise ValueError("Navigation episode counts must be non-negative")
        for value, name in (
            (self.goal_distance_m, "goal_distance_m"),
            (self.heading_error_deg, "heading_error_deg"),
        ):
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{name} must be finite when present")
        object.__setattr__(
            self,
            "recent_recoveries",
            _tuple_of_strings(self.recent_recoveries),
        )


@dataclass(frozen=True, slots=True)
class PerceptionContext:
    visual_health: float = 0.0
    depth_health: float = 0.0
    depth_stability: float = 0.0
    condition: str = "UNKNOWN"
    visual_condition: str = "UNKNOWN"
    depth_condition: str = "UNKNOWN"
    freshness_state: str = "UNKNOWN"
    centre_risk: float = 0.0
    vlm_status: str = "DISABLED"
    terrain_type: str = "UNKNOWN"
    visible_landmarks: tuple[str, ...] = ()
    target_visible: bool = False
    scene_confidence: float = 0.0
    texture_quality: float = 0.0
    scene_summary: str = "Scene understanding disabled"
    vlm_model: str = "NONE"
    vlm_timestamp_monotonic: float = 0.0
    vlm_source_frame_sequence: int = 0
    fused_scene: FusedSceneContext = field(default_factory=FusedSceneContext)
    latest_semantic_validation: SemanticValidationRecord | None = None

    def __post_init__(self) -> None:
        _unit_interval(self.visual_health, "visual_health")
        _unit_interval(self.depth_health, "depth_health")
        _unit_interval(self.depth_stability, "depth_stability")
        _unit_interval(self.centre_risk, "centre_risk")
        _unit_interval(self.scene_confidence, "scene_confidence")
        _unit_interval(self.texture_quality, "texture_quality")
        if not math.isfinite(self.vlm_timestamp_monotonic) or self.vlm_timestamp_monotonic < 0:
            raise ValueError("vlm_timestamp_monotonic must be finite and non-negative")
        if self.vlm_source_frame_sequence < 0:
            raise ValueError("vlm_source_frame_sequence must be non-negative")
        if not self.vlm_status or not self.terrain_type or not self.scene_summary:
            raise ValueError("VLM perception labels must not be empty")
        object.__setattr__(
            self,
            "visible_landmarks",
            _tuple_of_strings(self.visible_landmarks),
        )
        if self.latest_semantic_validation is not None and not isinstance(
            self.latest_semantic_validation,
            SemanticValidationRecord,
        ):
            raise TypeError("latest_semantic_validation must be a typed record")
        if not isinstance(self.fused_scene, FusedSceneContext):
            raise TypeError("fused_scene must be a typed immutable context")


@dataclass(frozen=True, slots=True)
class SafetyContext:
    supervisor_state: str = "PENDING"
    emergency_state: str = "CLEAR"
    collision_detected: bool = False
    hard_override: bool = False
    current_ownership: str = "NONE"
    collision_count: int = 0
    last_safety_reason: str = "NONE"
    landing_state: str = "NOT_ATTEMPTED"
    disarm_state: str = "UNKNOWN"

    def __post_init__(self) -> None:
        if self.collision_count < 0:
            raise ValueError("collision_count must be non-negative")


@dataclass(frozen=True, slots=True)
class MissionSummary:
    recovery_count: int = 0
    avoidance_count: int = 0
    observations_completed: int = 0
    mission_confidence: float = 0.0
    elapsed_mission_s: float = 0.0
    completed_waypoint_count: int = 0
    skipped_waypoint_count: int = 0
    emergency_suspected_count: int = 0
    emergency_cleared_count: int = 0
    emergency_confirmed_count: int = 0
    collision_count: int = 0
    replan_count: int = 0
    mission_result: str = "PENDING"

    def __post_init__(self) -> None:
        if min(
            self.recovery_count,
            self.avoidance_count,
            self.observations_completed,
            self.completed_waypoint_count,
            self.skipped_waypoint_count,
            self.emergency_suspected_count,
            self.emergency_cleared_count,
            self.emergency_confirmed_count,
            self.collision_count,
            self.replan_count,
        ) < 0:
            raise ValueError("Mission summary counts must be non-negative")
        _unit_interval(self.mission_confidence, "mission_confidence")
        if not math.isfinite(self.elapsed_mission_s) or self.elapsed_mission_s < 0:
            raise ValueError("elapsed_mission_s must be finite and non-negative")


def freeze_metadata(metadata: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if metadata is None:
        return MappingProxyType({})
    if any(not isinstance(key, str) or not key for key in metadata):
        raise ValueError("Event metadata keys must be non-empty strings")
    return MappingProxyType({key: _freeze(value) for key, value in metadata.items()})


def _freeze(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("Event metadata floats must be finite")
        return value
    if isinstance(value, Enum):
        return _freeze(value.value)
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) or not key for key in value):
            raise ValueError("Event metadata keys must be non-empty strings")
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return tuple(sorted((_freeze(item) for item in value), key=repr))
    raise TypeError(f"Unsupported event metadata value: {type(value).__name__}")


@dataclass(frozen=True, slots=True)
class EgoContextEvent:
    sequence: int
    timestamp_monotonic: float
    category: str
    message: str
    metadata: Mapping[str, Any] = field(
        default_factory=lambda: MappingProxyType({})
    )


@dataclass(frozen=True, slots=True)
class EgoContextSnapshot:
    revision: int
    updated_at_monotonic: float
    mission: MissionContext
    navigation: NavigationContext
    perception: PerceptionContext
    safety: SafetyContext
    summary: MissionSummary
    recent_events: tuple[EgoContextEvent, ...]
