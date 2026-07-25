from __future__ import annotations

import json
from dataclasses import dataclass, fields
from enum import Enum, auto
from pathlib import Path

from polaris.perception.models import VisualCondition
from polaris.perception.depth.models import DepthCondition

from .models import NavigationState


class SafetyMode(Enum):
    NORMAL = auto()
    CAUTIOUS = auto()
    DEGRADED = auto()
    EMERGENCY = auto()


@dataclass(frozen=True)
class ConfidenceConfig:
    normal_health_min: float = 0.75
    cautious_health_min: float = 0.50
    degraded_health_min: float = 0.25
    battery_critical: float = 0.15
    battery_low: float = 0.30
    high_horizontal_speed_m_s: float = 3.0
    visual_emergency_score: float = 0.15
    exposure_critical_ratio: float = 0.80
    texture_healthy: float = 12.0
    features_healthy: int = 150
    coverage_healthy: float = 0.50

    @classmethod
    def from_json(cls, path: str | Path) -> ConfidenceConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {field.name for field in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown confidence settings: {sorted(unknown)}")
        return cls(**values)


@dataclass
class ConfidenceReport:
    visual_health: float
    camera_freshness: float
    exposure_quality: float
    texture_feature_quality: float
    localization_health: float
    telemetry_health: float
    battery_health: float
    motion_health: float
    overall_health: float
    mode: SafetyMode
    reasons: list[str]
    critical_overrides: list[str]
    depth_health: float = 0.0
    depth_freshness: float = 0.0
    depth_required: bool = False


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def _at_least(mode: SafetyMode, minimum: SafetyMode) -> SafetyMode:
    return mode if mode.value >= minimum.value else minimum


class ConfidenceEngine:
    def __init__(self, config: ConfidenceConfig | None = None) -> None:
        self.config = config or ConfidenceConfig()

    def evaluate(self, state: NavigationState) -> ConfidenceReport:
        reasons: list[str] = []
        critical_overrides: list[str] = []

        localization_health = 1.0
        if not state.local_position_ok:
            localization_health = 0.2
            reasons.append("Local position unavailable")
        position_stale = state.position_is_stale()
        if position_stale:
            localization_health = min(localization_health, 0.1)
            reasons.append("Position telemetry stale")

        telemetry_health = 1.0
        if state.attitude_is_stale():
            telemetry_health = 0.1
            reasons.append("Attitude telemetry stale")

        battery_health = 1.0
        if state.battery_remaining is not None:
            if state.battery_remaining < self.config.battery_critical:
                battery_health = 0.1
                reasons.append("Battery critical")
            elif state.battery_remaining < self.config.battery_low:
                battery_health = 0.5
                reasons.append("Battery low")

        motion_health = 1.0
        if state.horizontal_speed_m_s > self.config.high_horizontal_speed_m_s:
            motion_health = 0.5
            reasons.append("Horizontal speed high")

        depth = state.depth
        depth_enabled = bool(depth.model_name) or state.depth_required_for_motion
        depth_freshness = float(depth.available and not depth.stale)
        depth_health = _clamp(depth.depth_health_score)
        if depth_enabled and depth.condition is not DepthCondition.HEALTHY:
            reasons.extend(f"Depth: {reason}" for reason in depth.reasons)

        visual = state.visual
        visual_health = _clamp(visual.visual_health_score)
        camera_missing = visual.last_visual_update_monotonic == 0.0
        camera_freshness = float(
            visual.camera_connected and not visual.stale and not camera_missing
        )
        exposure_ratio = max(visual.overexposed_ratio, visual.underexposed_ratio)
        exposure_quality = _clamp(
            1.0 - exposure_ratio / self.config.exposure_critical_ratio
        )
        texture_feature_quality = min(
            _clamp(visual.texture_score / self.config.texture_healthy),
            _clamp(visual.feature_count / self.config.features_healthy),
            _clamp(visual.feature_coverage / self.config.coverage_healthy),
        )

        if visual.visual_condition is not VisualCondition.HEALTHY:
            reasons.extend(f"Visual: {reason}" for reason in visual.reasons)

        # A missing/stale camera is tolerated on the ground, but its factual
        # health remains visible in the report. Fresh degraded vision still
        # constrains ground and airborne confidence.
        effective_visual_health = (
            1.0 if not state.in_air and camera_freshness == 0.0 else visual_health
        )
        overall_health = min(
            localization_health,
            telemetry_health,
            battery_health,
            motion_health,
            effective_visual_health,
            depth_health if state.depth_required_for_motion else 1.0,
        )
        mode = self._mode_for_health(overall_health)

        if state.in_air and (not state.local_position_ok or position_stale):
            critical_overrides.append("Airborne local position failure")
            mode = SafetyMode.EMERGENCY

        if state.in_air and camera_missing:
            critical_overrides.append("Airborne camera missing")
            mode = SafetyMode.EMERGENCY
        elif state.in_air and (visual.stale or not visual.camera_connected):
            critical_overrides.append("Airborne camera stale or disconnected")
            mode = SafetyMode.EMERGENCY

        if state.in_air and exposure_ratio >= self.config.exposure_critical_ratio:
            critical_overrides.append("Airborne camera exposure unusable")
            mode = SafetyMode.EMERGENCY

        if state.in_air and visual.visual_condition is VisualCondition.CRITICAL:
            if visual_health <= self.config.visual_emergency_score:
                critical_overrides.append("Airborne visual health critically low")
                mode = SafetyMode.EMERGENCY
            else:
                mode = _at_least(mode, SafetyMode.DEGRADED)
        elif state.in_air and visual.visual_condition is VisualCondition.DEGRADED:
            mode = _at_least(mode, SafetyMode.CAUTIOUS)

        if state.depth_required_for_motion:
            if not depth.available or depth.stale:
                if state.in_air:
                    critical_overrides.append("Required airborne depth unavailable or stale")
                    mode = SafetyMode.EMERGENCY
                else:
                    mode = _at_least(mode, SafetyMode.DEGRADED)
            elif depth.condition is DepthCondition.CRITICAL:
                if state.in_air:
                    critical_overrides.append("Required airborne depth is critical")
                    mode = SafetyMode.EMERGENCY
                else:
                    mode = _at_least(mode, SafetyMode.DEGRADED)
            elif depth.condition is DepthCondition.DEGRADED:
                mode = _at_least(mode, SafetyMode.DEGRADED)

        reasons.extend(critical_overrides)
        return ConfidenceReport(
            visual_health=visual_health,
            camera_freshness=camera_freshness,
            exposure_quality=exposure_quality,
            texture_feature_quality=texture_feature_quality,
            localization_health=localization_health,
            telemetry_health=telemetry_health,
            battery_health=battery_health,
            motion_health=motion_health,
            overall_health=overall_health,
            mode=mode,
            reasons=reasons,
            critical_overrides=critical_overrides,
            depth_health=depth_health,
            depth_freshness=depth_freshness,
            depth_required=state.depth_required_for_motion,
        )

    def _mode_for_health(self, health: float) -> SafetyMode:
        if health >= self.config.normal_health_min:
            return SafetyMode.NORMAL
        if health >= self.config.cautious_health_min:
            return SafetyMode.CAUTIOUS
        if health >= self.config.degraded_health_min:
            return SafetyMode.DEGRADED
        return SafetyMode.EMERGENCY
