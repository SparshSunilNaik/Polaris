from __future__ import annotations

import json
from pathlib import Path
from time import monotonic

from polaris.navigation.models import LocalGoal

from .models import (
    FailureAction,
    FailurePolicy,
    MissionOrchestratorConfig,
    MissionPlan,
    MissionWaypoint,
    ObservationCompletionConfig,
    ReturnOrLandPolicy,
    WaypointType,
)


def load_mission(
    mission_id: str,
    mission_dir: str | Path,
    config: MissionOrchestratorConfig,
) -> MissionPlan:
    directory = Path(mission_dir)
    filename = mission_id.replace("-", "_") + ".json"
    direct = directory / filename
    if direct.exists():
        return load_mission_file(direct, config)
    matches = [
        plan
        for path in sorted(directory.glob("*.json"))
        if (plan := load_mission_file(path, config)).mission_id == mission_id
    ]
    if len(matches) != 1:
        raise FileNotFoundError(f"Mission {mission_id!r} was not found uniquely")
    return matches[0]


def load_mission_file(
    path: str | Path,
    config: MissionOrchestratorConfig,
) -> MissionPlan:
    source = Path(path)
    values = json.loads(source.read_text(encoding="utf-8"))
    required = {
        "mission_id",
        "name",
        "description",
        "failure_policy",
        "maximum_duration_s",
        "maximum_path_length_m",
        "maximum_replans",
        "return_or_land_policy",
        "waypoints",
    }
    missing = required - set(values)
    if missing:
        raise ValueError(f"Mission is missing fields: {sorted(missing)}")
    unknown = set(values) - required
    if unknown:
        raise ValueError(f"Mission contains unknown fields: {sorted(unknown)}")
    waypoints = tuple(
        _load_waypoint(item, source.name, config)
        for item in values["waypoints"]
    )
    plan = MissionPlan(
        mission_id=str(values["mission_id"]),
        name=str(values["name"]),
        description=str(values["description"]),
        waypoints=waypoints,
        failure_policy=FailurePolicy(values["failure_policy"]),
        maximum_duration_s=float(values["maximum_duration_s"]),
        maximum_path_length_m=float(values["maximum_path_length_m"]),
        maximum_replans=int(values["maximum_replans"]),
        return_or_land_policy=ReturnOrLandPolicy(values["return_or_land_policy"]),
        source=str(source),
        timestamp_monotonic=monotonic(),
    )
    plan.validate(config)
    return plan


def _load_waypoint(
    values: dict,
    source: str,
    config: MissionOrchestratorConfig,
) -> MissionWaypoint:
    required = {"waypoint_id", "goal", "required", "skippable"}
    missing = required - set(values)
    if missing:
        raise ValueError(f"Waypoint is missing fields: {sorted(missing)}")
    allowed = required | {
        "maximum_attempts",
        "verification_hold_s",
        "minimum_visual_health",
        "minimum_depth_health",
        "preferred_terminal_yaw_deg",
        "failure_action",
        "metadata",
        "type",
        "observation",
    }
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"Waypoint contains unknown fields: {sorted(unknown)}")
    goal_values = values["goal"]
    goal_required = {"north_offset_m", "east_offset_m"}
    if not goal_required.issubset(goal_values):
        raise ValueError("Waypoint goal requires north_offset_m and east_offset_m")
    goal = LocalGoal(
        goal_id=str(values["waypoint_id"]),
        target_north_m=float(goal_values["north_offset_m"]),
        target_east_m=float(goal_values["east_offset_m"]),
        target_down_m=float(goal_values.get("down_offset_m", 0.0)),
        target_yaw_deg=(
            None
            if goal_values.get("target_yaw_deg") is None
            else float(goal_values["target_yaw_deg"])
        ),
        horizontal_tolerance_m=float(goal_values.get("horizontal_tolerance_m", 0.5)),
        vertical_tolerance_m=float(goal_values.get("vertical_tolerance_m", 0.35)),
        timeout_s=float(goal_values.get("timeout_s", 60.0)),
        maximum_path_length_m=float(goal_values.get("maximum_path_length_m", 15.0)),
        maximum_displacement_m=float(goal_values.get("maximum_displacement_m", 13.0)),
        completion_hold_s=float(goal_values.get("completion_hold_s", 1.5)),
        source=f"mission:{source}:MISSION_ORIGIN_NED_OFFSET",
        timestamp_monotonic=0.0,
    )
    waypoint_type = WaypointType(values.get("type", WaypointType.POSITION.value))
    observation_values = values.get("observation")
    observation = None
    if observation_values is not None:
        allowed_observation = {
            "minimum_goal_distance_m",
            "maximum_goal_distance_m",
            "maximum_heading_error_deg",
            "minimum_visual_health",
            "minimum_depth_health",
            "minimum_depth_stability",
            "maximum_centre_risk",
            "maximum_horizontal_speed_m_s",
            "required_stable_s",
            "minimum_fresh_observations",
            "maximum_wait_s",
        }
        unknown_observation = set(observation_values) - allowed_observation
        if unknown_observation:
            raise ValueError(
                "Observation contains unknown fields: "
                f"{sorted(unknown_observation)}"
            )
        missing_observation = allowed_observation - set(observation_values)
        if missing_observation:
            raise ValueError(
                "Observation is missing fields: "
                f"{sorted(missing_observation)}"
            )
        parsed_observation = {
            name: float(observation_values[name]) for name in allowed_observation
        }
        parsed_observation["minimum_fresh_observations"] = int(
            observation_values["minimum_fresh_observations"]
        )
        observation = ObservationCompletionConfig(**parsed_observation)
    waypoint = MissionWaypoint(
        waypoint_id=str(values["waypoint_id"]),
        goal=goal,
        waypoint_type=waypoint_type,
        observation=observation,
        required=bool(values["required"]),
        skippable=bool(values["skippable"]),
        maximum_attempts=int(
            values.get("maximum_attempts", config.maximum_waypoint_attempts)
        ),
        verification_hold_s=float(
            values.get(
                "verification_hold_s",
                config.waypoint_verification_duration_s,
            )
        ),
        minimum_visual_health=float(values.get("minimum_visual_health", 0.35)),
        minimum_depth_health=float(values.get("minimum_depth_health", 0.40)),
        preferred_terminal_yaw_deg=(
            None
            if values.get("preferred_terminal_yaw_deg") is None
            else float(values["preferred_terminal_yaw_deg"])
        ),
        failure_action=FailureAction(
            values.get("failure_action", FailureAction.USE_MISSION_POLICY.value)
        ),
        metadata=dict(values.get("metadata", {})),
    )
    waypoint.validate(config.world_bounds)
    if waypoint.maximum_attempts > config.maximum_waypoint_attempts:
        raise ValueError("Waypoint attempt limit exceeds orchestrator configuration")
    return waypoint
