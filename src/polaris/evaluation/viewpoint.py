from __future__ import annotations

import csv
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import median
from typing import Any

from polaris.ego_state.confidence import ConfidenceReport, SafetyMode
from polaris.ego_state.models import NavigationState
from polaris.perception.models import VisualCondition


@dataclass(frozen=True)
class ViewpointSpec:
    name: str
    gazebo_yaw_deg: float
    px4_yaw_deg: float


@dataclass(frozen=True)
class BenchmarkConfig:
    takeoff_altitude_m: float
    yaw_rate_deg_s: float
    yaw_tolerance_deg: float
    max_horizontal_drift_m: float
    settle_duration_s: float
    sample_duration_s: float
    analysis_rate_hz: float
    viewpoints: tuple[ViewpointSpec, ...]

    @classmethod
    def from_json(cls, path: str | Path) -> BenchmarkConfig:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        viewpoints = tuple(ViewpointSpec(**item) for item in data.pop("viewpoints"))
        config = cls(viewpoints=viewpoints, **data)
        if not config.viewpoints:
            raise ValueError("At least one viewpoint is required")
        if min(
            config.takeoff_altitude_m,
            config.yaw_rate_deg_s,
            config.yaw_tolerance_deg,
            config.max_horizontal_drift_m,
            config.sample_duration_s,
            config.analysis_rate_hz,
        ) <= 0:
            raise ValueError("Benchmark numeric settings must be positive")
        if config.settle_duration_s < 0:
            raise ValueError("settle_duration_s cannot be negative")
        names = [viewpoint.name for viewpoint in config.viewpoints]
        if len(names) != len(set(names)):
            raise ValueError("Viewpoint names must be unique")
        return config


@dataclass(frozen=True)
class ViewpointSample:
    measured_yaw_deg: float
    north_m: float
    east_m: float
    down_m: float
    altitude_m: float
    horizontal_drift_m: float
    fps: float
    frame_age_ms: float
    brightness: float
    contrast: float
    blur_score: float
    texture_score: float
    feature_count: int
    feature_coverage: float
    overexposed_ratio: float
    underexposed_ratio: float
    visual_health_score: float
    visual_condition: VisualCondition
    reasons: tuple[str, ...]
    localization_health: float
    telemetry_health: float
    safety_mode: SafetyMode
    visual_update_monotonic: float


@dataclass(frozen=True)
class ViewpointResult:
    name: str
    commanded_yaw_deg: float
    expected_gazebo_yaw_deg: float
    measured_yaw_deg: float
    position_ned_m: dict[str, float]
    altitude_m: float
    sample_count: int
    median_fps: float
    median_frame_age_ms: float
    median_brightness: float
    median_contrast: float
    median_blur_score: float
    median_texture_score: float
    median_feature_count: float
    median_feature_coverage: float
    median_overexposed_ratio: float
    median_underexposed_ratio: float
    median_visual_health_score: float
    worst_visual_health_score: float
    dominant_visual_condition: str
    accumulated_reasons: tuple[str, ...]
    localization_health: float
    telemetry_health: float
    final_safety_mode: str
    max_horizontal_drift_m: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_csv_row(self) -> dict[str, Any]:
        row = self.to_dict()
        position = row.pop("position_ned_m")
        row["position_north_m"] = position["north"]
        row["position_east_m"] = position["east"]
        row["position_down_m"] = position["down"]
        row["accumulated_reasons"] = " | ".join(self.accumulated_reasons)
        return row


@dataclass(frozen=True)
class BenchmarkRun:
    started_at_utc: str
    completed_at_utc: str
    hover_reference_ned_m: dict[str, float]
    max_horizontal_drift_m: float
    viewpoints: tuple[ViewpointResult, ...]
    yaw_convention: str = (
        "PX4 NED yaw is clockwise from north; Gazebo ENU yaw is "
        "counter-clockwise from east; px4_yaw = 90 - gazebo_yaw"
    )
    schema_version: int = 1

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def normalize_yaw(yaw_deg: float) -> float:
    wrapped = (yaw_deg + 180.0) % 360.0 - 180.0
    return 180.0 if math.isclose(wrapped, -180.0) else wrapped


def shortest_yaw_delta(target_deg: float, current_deg: float) -> float:
    return normalize_yaw(target_deg - current_deg)


def sample_from_state(
    state: NavigationState,
    confidence: ConfidenceReport,
    hover_north_m: float,
    hover_east_m: float,
) -> ViewpointSample | None:
    visual = state.visual
    if (
        visual.stale
        or not visual.camera_connected
        or visual.last_visual_update_monotonic == 0.0
    ):
        return None
    north = state.position_ned_m.x
    east = state.position_ned_m.y
    return ViewpointSample(
        measured_yaw_deg=state.orientation.yaw_deg,
        north_m=north,
        east_m=east,
        down_m=state.position_ned_m.z,
        altitude_m=state.altitude_m,
        horizontal_drift_m=math.hypot(
            north - hover_north_m,
            east - hover_east_m,
        ),
        fps=visual.fps,
        frame_age_ms=visual.frame_age_ms,
        brightness=visual.brightness,
        contrast=visual.contrast,
        blur_score=visual.blur_score,
        texture_score=visual.texture_score,
        feature_count=visual.feature_count,
        feature_coverage=visual.feature_coverage,
        overexposed_ratio=visual.overexposed_ratio,
        underexposed_ratio=visual.underexposed_ratio,
        visual_health_score=visual.visual_health_score,
        visual_condition=visual.visual_condition,
        reasons=visual.reasons,
        localization_health=confidence.localization_health,
        telemetry_health=confidence.telemetry_health,
        safety_mode=confidence.mode,
        visual_update_monotonic=visual.last_visual_update_monotonic,
    )


def abort_reason(
    state: NavigationState,
    confidence: ConfidenceReport,
    px4_connected: bool,
) -> str | None:
    if not px4_connected:
        return "PX4 disconnected"
    if state.armed and state.depth_required_for_motion and (
        not state.depth.available
        or state.depth.stale
        or state.depth.condition.value == "CRITICAL"
    ):
        return "Required depth failed while vehicle is armed"
    if not state.in_air:
        return None
    if not state.local_position_ok or state.position_is_stale():
        return "Airborne local position unavailable or stale"
    if (
        state.visual.stale
        or not state.visual.camera_connected
        or state.visual.last_visual_update_monotonic == 0.0
    ):
        return "Airborne camera unavailable or stale"
    if confidence.mode is SafetyMode.EMERGENCY:
        details = "; ".join(confidence.reasons) or "no reasons reported"
        return f"Confidence Engine entered EMERGENCY: {details}"
    return None


def aggregate_viewpoint(
    spec: ViewpointSpec,
    samples: list[ViewpointSample],
) -> ViewpointResult:
    if not samples:
        raise ValueError(f"No valid samples for viewpoint {spec.name}")

    def med(name: str) -> float:
        return float(median(getattr(sample, name) for sample in samples))

    yaw_delta = median(
        shortest_yaw_delta(sample.measured_yaw_deg, spec.px4_yaw_deg)
        for sample in samples
    )
    condition_counts = Counter(sample.visual_condition for sample in samples)
    condition = max(
        condition_counts,
        key=lambda item: (condition_counts[item], _condition_severity(item)),
    )
    accumulated_reasons = tuple(
        sorted({reason for sample in samples for reason in sample.reasons})
    )
    return ViewpointResult(
        name=spec.name,
        commanded_yaw_deg=spec.px4_yaw_deg,
        expected_gazebo_yaw_deg=spec.gazebo_yaw_deg,
        measured_yaw_deg=normalize_yaw(spec.px4_yaw_deg + yaw_delta),
        position_ned_m={
            "north": med("north_m"),
            "east": med("east_m"),
            "down": med("down_m"),
        },
        altitude_m=med("altitude_m"),
        sample_count=len(samples),
        median_fps=med("fps"),
        median_frame_age_ms=med("frame_age_ms"),
        median_brightness=med("brightness"),
        median_contrast=med("contrast"),
        median_blur_score=med("blur_score"),
        median_texture_score=med("texture_score"),
        median_feature_count=med("feature_count"),
        median_feature_coverage=med("feature_coverage"),
        median_overexposed_ratio=med("overexposed_ratio"),
        median_underexposed_ratio=med("underexposed_ratio"),
        median_visual_health_score=med("visual_health_score"),
        worst_visual_health_score=min(
            sample.visual_health_score for sample in samples
        ),
        dominant_visual_condition=condition.value,
        accumulated_reasons=accumulated_reasons,
        localization_health=med("localization_health"),
        telemetry_health=med("telemetry_health"),
        final_safety_mode=samples[-1].safety_mode.name,
        max_horizontal_drift_m=max(sample.horizontal_drift_m for sample in samples),
    )


def write_outputs(
    run: BenchmarkRun,
    output_dir: str | Path,
    stem: str,
    json_only: bool = False,
) -> tuple[Path, Path | None]:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / f"{stem}_viewpoints.json"
    json_path.write_text(
        json.dumps(run.to_dict(), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    csv_path: Path | None = None
    if not json_only:
        csv_path = directory / f"{stem}_viewpoints.csv"
        rows = [viewpoint.to_csv_row() for viewpoint in run.viewpoints]
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return json_path, csv_path


def _condition_severity(condition: VisualCondition) -> int:
    return {
        VisualCondition.HEALTHY: 0,
        VisualCondition.DEGRADED: 1,
        VisualCondition.CRITICAL: 2,
    }[condition]
