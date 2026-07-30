from __future__ import annotations

import math
from pathlib import Path
from typing import Any


def validate_mission_replay_payload(payload: Any) -> None:
    if not isinstance(payload, dict):
        raise ValueError("Artifact root must be a JSON object")
    if payload.get("schema_version") != 1:
        raise ValueError(
            f"Unsupported mission artifact schema version: {payload.get('schema_version')!r}"
        )

    plan = _mapping(payload, "mission_plan")
    state = _mapping(payload, "mission_state")
    if not isinstance(plan.get("name"), str) or not plan["name"].strip():
        raise ValueError("mission_plan.name must be a non-empty string")
    duration = state.get("mission_elapsed_s")
    if (
        isinstance(duration, bool)
        or not isinstance(duration, (int, float))
        or not math.isfinite(duration)
        or duration < 0
    ):
        raise ValueError("mission_state.mission_elapsed_s must be finite and non-negative")
    for key in ("completed_waypoints", "failed_waypoints"):
        values = state.get(key)
        if not isinstance(values, list) or not all(isinstance(item, str) for item in values):
            raise ValueError(f"mission_state.{key} must be a list of strings")

    waypoint_results = _list(payload, "waypoint_results")
    for index, item in enumerate(waypoint_results):
        if not isinstance(item, dict):
            raise ValueError(f"waypoint_results[{index}] must be an object")
        if not isinstance(item.get("final_metrics", {}), dict):
            raise ValueError(f"waypoint_results[{index}].final_metrics must be an object")

    for key in (
        "mission_transitions",
        "navigation_transitions",
        "avoidance_transitions",
        "supervisor_transitions",
        "active_perception_recoveries",
        "replanning_decisions",
    ):
        values = _list(payload, key)
        if not all(isinstance(item, dict) for item in values):
            raise ValueError(f"{key} must contain only objects")

    collision_count = payload.get("collision_count")
    if (
        isinstance(collision_count, bool)
        or not isinstance(collision_count, int)
        or collision_count < 0
    ):
        raise ValueError("collision_count must be a non-negative integer")
    for key in ("landing_result", "mission_result", "final_result"):
        if not isinstance(payload.get(key), str) or not payload[key]:
            raise ValueError(f"{key} must be a non-empty string")
    abort_reason = payload.get("abort_reason")
    if abort_reason is not None and not isinstance(abort_reason, str):
        raise ValueError("abort_reason must be a string or null")
    events = payload.get("events")
    if events is not None:
        if not isinstance(events, list):
            raise ValueError("events must be a list when present")
        previous_sequence = 0
        for index, event in enumerate(events):
            if not isinstance(event, dict):
                raise ValueError(f"events[{index}] must be an object")
            if event.get("sequence") != previous_sequence + 1:
                raise ValueError("events sequence must be strictly increasing from 1")
            previous_sequence += 1
            if not isinstance(event.get("data"), dict):
                raise ValueError(f"events[{index}].data must be an object")


def format_mission_replay(payload: dict[str, Any], artifact_path: str | Path) -> str:
    plan = payload.get("mission_plan", {})
    state = payload.get("mission_state", {})
    waypoint_results = payload.get("waypoint_results", [])
    completed = state.get("completed_waypoints", [])
    failed = state.get("failed_waypoints", [])
    landing = str(payload.get("landing_result", "UNKNOWN"))
    final_result = str(payload.get("final_result", "UNKNOWN"))
    outcome = (
        final_result
        if final_result == "NO_FLIGHT_COMPLETED"
        else str(payload.get("mission_result", final_result))
    )
    disarm = (
        "CONFIRMED"
        if landing == "CONFIRMED"
        else "NOT_REQUIRED"
        if final_result == "NO_FLIGHT_COMPLETED" or landing == "NOT_REQUIRED"
        else "UNCONFIRMED"
    )
    recoveries = sum(int(item.get("recovery_count", 0)) for item in waypoint_results)
    avoidances = sum(
        int(item.get("final_metrics", {}).get("avoidance_count", 0))
        for item in waypoint_results
    )
    timeline = _timeline(payload)
    lines = [
        "========== Mission Replay ==========",
        f"Mission: {plan.get('name', state.get('mission_name', 'UNKNOWN'))}",
        f"Outcome: {outcome}",
        f"Duration: {float(state.get('mission_elapsed_s', 0.0)):.1f} s",
        f"Completed waypoints: {_items(completed)}",
        f"Failed waypoint: {failed[-1] if failed else 'NONE'}",
        f"Abort reason: {payload.get('abort_reason') or state.get('abort_reason') or 'NONE'}",
        f"Recovery count: {recoveries}",
        f"Avoidance count: {avoidances}",
        f"Collision count: {int(payload.get('collision_count', 0))}",
        f"Landing: {landing}",
        f"Disarm: {disarm}",
        f"Artifact: {artifact_path}",
    ]
    for result in waypoint_results:
        if result.get("completion_mode") != "OBSERVATION":
            continue
        observation = result.get("observation") or {}
        distance = float(observation.get("completion_distance_m", 0.0))
        lines.append(
            f"Observation: {result.get('waypoint_id', 'UNKNOWN')} completed by "
            f"stable observation at {distance:.2f} metres"
        )
    lines.append("Timeline:")
    lines.extend(f"  {item}" for item in timeline)
    if not timeline:
        lines.append("  NONE")
    lines.append("====================================")
    return "\n".join(lines)


def _timeline(payload: dict[str, Any]) -> list[str]:
    events: list[tuple[float, int, str]] = []
    order = 0

    def add(timestamp: Any, label: str) -> None:
        nonlocal order
        if isinstance(timestamp, (int, float)):
            events.append((float(timestamp), order, label))
            order += 1

    for item in payload.get("mission_transitions", []):
        add(
            item.get("timestamp_monotonic"),
            _event("MISSION", item.get("state"), item.get("waypoint_id"), item.get("reason")),
        )
    for item in payload.get("navigation_transitions", []):
        add(
            item.get("timestamp_monotonic"),
            _event("NAVIGATOR", item.get("state"), None, item.get("reason")),
        )
    for item in payload.get("avoidance_transitions", []):
        add(
            item.get("timestamp_monotonic"),
            _event("AVOIDANCE", item.get("mode"), item.get("waypoint_id"), None),
        )
    for item in payload.get("supervisor_transitions", []):
        confirmation = item.get("emergency_confirmation") or {}
        confirmation_state = confirmation.get("state")
        confirmation_reason = confirmation.get("reason", "")
        clear_transition = confirmation_state == "CLEAR" and (
            "cleared" in confirmation_reason.lower()
            or "ownership ended" in confirmation_reason.lower()
        )
        if confirmation_state and (
            confirmation_state != "CLEAR" or clear_transition
        ):
            observed = int(confirmation.get("observed_frames", 0))
            required = int(confirmation.get("required_frames", 0))
            age = float(confirmation.get("age_s", 0.0))
            centre = float(confirmation.get("centre_risk", 0.0))
            add(
                item.get("timestamp_monotonic"),
                f"EMERGENCY {confirmation_state} {observed}/{required} "
                f"age {age:.1f}s centre {centre:.3f}: {confirmation_reason}",
            )
            continue
        if item.get("mode") in {"PASS_THROUGH", "SLOW"}:
            continue
        reasons = "; ".join(str(reason) for reason in item.get("reasons", []))
        add(
            item.get("timestamp_monotonic"),
            _event("SUPERVISOR", item.get("mode"), item.get("waypoint_id"), reasons),
        )
    for item in payload.get("observation_events", []):
        event_type = item.get("event_type", "UNKNOWN")
        detail = str(item.get("reason", ""))
        if event_type == "COMPLETED":
            detail += (
                f"; distance {float(item.get('goal_distance_m', 0.0)):.2f}m"
                f"; dwell {float(item.get('dwell_elapsed_s', 0.0)):.1f}s"
            )
        add(
            item.get("timestamp_monotonic"),
            _event(
                "OBSERVATION",
                event_type,
                item.get("waypoint_id"),
                detail,
            ),
        )
    for item in payload.get("active_perception_recoveries", []):
        decision = item.get("decision", {})
        selected = decision.get("selected_heading_deg")
        detail = str(decision.get("trigger") or "Recovery decision")
        if selected is not None:
            detail += f"; selected {float(selected):+.1f} deg"
        add(
            decision.get("timestamp_monotonic"),
            _event("RECOVERY", decision.get("state"), item.get("waypoint_id"), detail),
        )
    for item in payload.get("replanning_decisions", []):
        generated = item.get("generated_waypoint") or {}
        detail = str(item.get("reason") or "Replan decision")
        if generated.get("waypoint_id"):
            detail += f"; generated {generated['waypoint_id']}"
        add(
            item.get("timestamp_monotonic"),
            _event("REPLAN", item.get("strategy"), generated.get("waypoint_id"), detail),
        )

    if not events:
        return []
    events.sort(key=lambda event: (event[0], event[1]))
    started = events[0][0]
    return [f"+{timestamp - started:06.1f}s {label}" for timestamp, _, label in events]


def _event(category: str, state: Any, waypoint: Any, reason: Any) -> str:
    label = f"{category} {state or 'UNKNOWN'}"
    if waypoint and waypoint != "NONE":
        label += f" [{waypoint}]"
    if reason:
        label += f": {reason}"
    return label


def _items(values: list[Any]) -> str:
    return ", ".join(str(value) for value in values) if values else "NONE"


def _mapping(payload: dict[str, Any], key: str) -> dict[str, Any]:
    value = payload.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be an object")
    return value


def _list(payload: dict[str, Any], key: str) -> list[Any]:
    value = payload.get(key)
    if not isinstance(value, list):
        raise ValueError(f"{key} must be a list")
    return value
