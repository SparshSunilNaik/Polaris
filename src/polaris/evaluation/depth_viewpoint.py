from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import median
from typing import Any

from polaris.evaluation.viewpoint import ViewpointResult
from polaris.perception.depth.models import DepthCondition, DepthState, REGION_NAMES


@dataclass(frozen=True)
class DepthViewpointResult:
    name: str
    visual: ViewpointResult
    baseline_visual_health_score: float | None
    visual_health_delta: float | None
    depth_sample_count: int
    median_depth_latency_ms: float
    median_depth_fps: float
    median_source_frame_age_ms: float
    median_depth_health_score: float
    worst_depth_health_score: float
    dominant_depth_condition: str
    median_temporal_stability: float
    median_dynamic_range: float
    median_relative_dynamic_range: float
    risk_grid: tuple[tuple[float, float, float], ...]
    nearest_region: str
    safest_region: str
    centre_risk: float
    left_risk: float
    right_risk: float
    upper_risk: float
    lower_risk: float
    accumulated_depth_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["risk_grid"] = [list(row) for row in self.risk_grid]
        return result

    def to_csv_row(self) -> dict[str, Any]:
        row: dict[str, Any] = {
            key: value
            for key, value in self.to_dict().items()
            if key not in {"visual", "risk_grid", "accumulated_depth_reasons"}
        }
        visual = self.visual.to_csv_row()
        row.update({f"visual_{key}": value for key, value in visual.items()})
        for index, name in enumerate(REGION_NAMES):
            row[f"risk_{name.lower()}"] = self.risk_grid[index // 3][index % 3]
        row["accumulated_depth_reasons"] = " | ".join(
            self.accumulated_depth_reasons
        )
        return row


@dataclass(frozen=True)
class DepthBenchmarkRun:
    started_at_utc: str
    completed_at_utc: str
    model_identifier: str
    model_revision: str
    device: str
    normalization: str
    baseline_artifact: str
    warmup_latency_ms: float
    max_horizontal_drift_m: float
    viewpoints: tuple[DepthViewpointResult, ...]
    schema_version: int = 1

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def aggregate_depth_viewpoint(
    visual: ViewpointResult,
    depth_samples: list[DepthState],
    baseline_visual_health_score: float | None,
) -> DepthViewpointResult:
    if not depth_samples:
        raise ValueError(f"No depth samples for viewpoint {visual.name}")

    def med(name: str) -> float:
        return float(median(getattr(sample, name) for sample in depth_samples))

    conditions = Counter(sample.condition for sample in depth_samples)
    condition = max(
        conditions,
        key=lambda item: (conditions[item], _severity(item)),
    )
    grids = [sample.risk_grid for sample in depth_samples]
    grid = tuple(
        tuple(
            float(median(frame[row][column] for frame in grids))
            for column in range(3)
        )
        for row in range(3)
    )
    flat_grid = [value for row in grid for value in row]
    nearest = REGION_NAMES[max(range(9), key=flat_grid.__getitem__)]
    safest = REGION_NAMES[min(range(9), key=flat_grid.__getitem__)]
    reasons = tuple(
        sorted({reason for sample in depth_samples for reason in sample.reasons})
    )
    health_delta = (
        None
        if baseline_visual_health_score is None
        else visual.median_visual_health_score - baseline_visual_health_score
    )
    return DepthViewpointResult(
        name=visual.name,
        visual=visual,
        baseline_visual_health_score=baseline_visual_health_score,
        visual_health_delta=health_delta,
        depth_sample_count=len(depth_samples),
        median_depth_latency_ms=med("inference_latency_ms"),
        median_depth_fps=med("inference_fps"),
        median_source_frame_age_ms=med("source_frame_age_ms"),
        median_depth_health_score=med("depth_health_score"),
        worst_depth_health_score=min(
            sample.depth_health_score for sample in depth_samples
        ),
        dominant_depth_condition=condition.value,
        median_temporal_stability=med("temporal_stability"),
        median_dynamic_range=med("raw_dynamic_range"),
        median_relative_dynamic_range=med("relative_dynamic_range"),
        risk_grid=grid,
        nearest_region=nearest,
        safest_region=safest,
        centre_risk=med("centre_risk"),
        left_risk=med("left_risk"),
        right_risk=med("right_risk"),
        upper_risk=med("upper_risk"),
        lower_risk=med("lower_risk"),
        accumulated_depth_reasons=reasons,
    )


def load_visual_baseline(path: str | Path) -> dict[str, float]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        item["name"]: float(item["median_visual_health_score"])
        for item in payload["viewpoints"]
    }


def write_depth_outputs(
    run: DepthBenchmarkRun,
    output_dir: str | Path,
    stem: str,
) -> tuple[Path, Path]:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / f"{stem}_depth_viewpoints.json"
    csv_path = directory / f"{stem}_depth_viewpoints.csv"
    json_path.write_text(
        json.dumps(run.to_dict(), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    rows = [viewpoint.to_csv_row() for viewpoint in run.viewpoints]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return json_path, csv_path


def _severity(condition: DepthCondition) -> int:
    return {
        DepthCondition.HEALTHY: 0,
        DepthCondition.DEGRADED: 1,
        DepthCondition.CRITICAL: 2,
    }[condition]
