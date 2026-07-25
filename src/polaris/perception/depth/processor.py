from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from polaris.perception.dependencies import add_homebrew_python_site

add_homebrew_python_site()

import numpy as np  # noqa: E402

from .models import (
    REGION_NAMES,
    DepthCondition,
    DepthConfig,
    DepthSourceHealth,
    DepthState,
)


@dataclass(frozen=True)
class DepthProcessorResult:
    state: DepthState
    normalized_proximity: np.ndarray | None


class DepthProcessor:
    """Converts raw relative depth to robust risk=proximity summaries."""

    def __init__(self, config: DepthConfig) -> None:
        self.config = config
        self._previous_grid: np.ndarray | None = None
        self._persistence = np.zeros(9, dtype=np.int32)

    def process(
        self,
        raw_depth: np.ndarray,
        *,
        source_frame_sequence: int,
        source_frame_age_ms: float,
        inference_timestamp_monotonic: float | None,
        inference_latency_ms: float,
        warmup_latency_ms: float,
        inference_fps: float,
        model_name: str,
        model_revision: str,
        device: str,
        input_width: int,
        input_height: int,
        source_health: DepthSourceHealth,
        dropped_frames: int = 0,
    ) -> DepthProcessorResult:
        now = monotonic() if inference_timestamp_monotonic is None else inference_timestamp_monotonic
        depth = np.asarray(raw_depth, dtype=np.float32)
        metadata = {
            "model_name": model_name,
            "model_revision": model_revision,
            "device": device,
            "source_frame_sequence": source_frame_sequence,
            "source_frame_age_ms": source_frame_age_ms,
            "inference_latency_ms": inference_latency_ms,
            "warmup_latency_ms": warmup_latency_ms,
            "inference_fps": inference_fps,
            "input_width": input_width,
            "input_height": input_height,
            "dropped_frames": dropped_frames,
        }
        if depth.ndim != 2 or depth.size == 0:
            return self._critical("Depth output has invalid dimensions", now, **metadata)

        finite = np.isfinite(depth)
        valid_ratio = float(np.mean(finite))
        if not np.all(finite):
            return self._critical(
                "Depth output contains NaN or Inf",
                now,
                valid_pixel_ratio=valid_ratio,
                **metadata,
            )

        raw_far = float(np.percentile(depth, self.config.far_percentile))
        raw_near = float(np.percentile(depth, self.config.near_percentile))
        raw_range = raw_near - raw_far
        relative_range = raw_range / max(abs(raw_near), abs(raw_far), 1e-6)
        if raw_range <= 1e-8 or relative_range < self.config.min_dynamic_range_ratio:
            return self._critical(
                "Depth output dynamic range is unusable",
                now,
                raw_dynamic_range=max(0.0, raw_range),
                relative_dynamic_range=max(0.0, relative_range),
                valid_pixel_ratio=valid_ratio,
                **metadata,
            )

        proximity = np.clip((depth - raw_far) / raw_range, 0.0, 1.0)
        if not self.config.raw_larger_is_near:
            proximity = 1.0 - proximity

        if source_frame_age_ms > self.config.source_stale_after_ms:
            return self._critical(
                "Source RGB frame is stale",
                now,
                valid_pixel_ratio=valid_ratio,
                **metadata,
            )
        exposure = max(
            source_health.overexposed_ratio,
            source_health.underexposed_ratio,
        )
        if exposure >= 0.80:
            return self._critical(
                "Source RGB exposure is unusable",
                now,
                valid_pixel_ratio=valid_ratio,
                **metadata,
            )

        current_grid = self._extract_grid(proximity)
        if self._previous_grid is None:
            stability = 1.0
            smoothed = current_grid
        else:
            stability = float(1.0 - np.mean(np.abs(current_grid - self._previous_grid)))
            smoothed = (
                self.config.temporal_alpha * current_grid
                + (1.0 - self.config.temporal_alpha) * self._previous_grid
            )
        smoothed = self._apply_persistence(current_grid, smoothed)
        self._previous_grid = smoothed.copy()

        reasons: list[str] = []
        health = 1.0
        visual_condition = source_health.visual_condition
        if visual_condition == "CRITICAL":
            reasons.append("Source RGB visual health is critical")
            health = min(health, max(0.40, source_health.visual_health_score))
        elif visual_condition == "DEGRADED":
            reasons.append("Source RGB visual health is degraded")
            health = min(health, 0.65, source_health.visual_health_score)
        if source_health.texture_score < self.config.source_texture_healthy:
            reasons.append("Source RGB texture is low")
            health = min(health, 0.65)
        if exposure >= 0.15:
            reasons.append("Source RGB exposure is degraded")
            health = min(health, 0.60)
        if stability < self.config.stability_healthy:
            reasons.append("Depth grid is temporally unstable")
            stability_health = np.clip(
                (stability - self.config.stability_critical)
                / (self.config.stability_healthy - self.config.stability_critical),
                0.0,
                1.0,
            )
            health = min(health, float(stability_health))

        condition = self._condition(health)
        flat_grid = smoothed.reshape(-1)
        nearest_index = int(np.argmax(flat_grid))
        safest_index = int(np.argmin(flat_grid))
        grid_tuple = tuple(
            tuple(float(value) for value in row) for row in smoothed.reshape(3, 3)
        )
        state = DepthState(
            available=True,
            stale=False,
            source_frame_sequence=source_frame_sequence,
            source_frame_age_ms=source_frame_age_ms,
            inference_timestamp_monotonic=now,
            inference_latency_ms=inference_latency_ms,
            warmup_latency_ms=warmup_latency_ms,
            inference_fps=inference_fps,
            model_name=model_name,
            model_revision=model_revision,
            device=device,
            input_width=input_width,
            input_height=input_height,
            output_width=int(depth.shape[1]),
            output_height=int(depth.shape[0]),
            temporal_stability=stability,
            depth_health_score=health,
            condition=condition,
            reasons=tuple(reasons),
            risk_grid=grid_tuple,
            nearest_region=REGION_NAMES[nearest_index],
            safest_region=REGION_NAMES[safest_index],
            centre_risk=float(smoothed[4]),
            left_risk=float(np.median(smoothed[[0, 3, 6]])),
            right_risk=float(np.median(smoothed[[2, 5, 8]])),
            upper_risk=float(np.median(smoothed[[0, 1, 2]])),
            lower_risk=float(np.median(smoothed[[6, 7, 8]])),
            global_near_percentile=float(
                np.percentile(proximity, self.config.near_percentile)
            ),
            global_median=float(np.median(proximity)),
            global_far_percentile=float(
                np.percentile(proximity, self.config.far_percentile)
            ),
            raw_dynamic_range=raw_range,
            relative_dynamic_range=relative_range,
            valid_pixel_ratio=valid_ratio,
            dropped_frames=dropped_frames,
        )
        return DepthProcessorResult(state=state, normalized_proximity=proximity)

    def failure_state(
        self,
        reason: str,
        *,
        model_name: str,
        model_revision: str,
        device: str,
        consecutive_failures: int,
        dropped_frames: int,
    ) -> DepthState:
        return DepthState(
            available=False,
            stale=True,
            inference_timestamp_monotonic=monotonic(),
            model_name=model_name,
            model_revision=model_revision,
            device=device,
            reasons=(reason,),
            consecutive_failures=consecutive_failures,
            dropped_frames=dropped_frames,
        )

    def _extract_grid(self, proximity: np.ndarray) -> np.ndarray:
        rows = np.array_split(proximity, 3, axis=0)
        values: list[float] = []
        for row in rows:
            for region in np.array_split(row, 3, axis=1):
                values.append(
                    float(
                        np.percentile(
                            region,
                            self.config.regional_risk_percentile,
                        )
                    )
                )
        return np.asarray(values, dtype=np.float32)

    def _apply_persistence(
        self,
        current: np.ndarray,
        smoothed: np.ndarray,
    ) -> np.ndarray:
        result = smoothed.copy()
        for index, risk in enumerate(current):
            if risk >= self.config.risk_threshold:
                self._persistence[index] += 1
            else:
                self._persistence[index] = 0
            immediate = index == 4 and risk >= self.config.immediate_centre_risk
            if self._persistence[index] < self.config.persistence_count and not immediate:
                result[index] = min(result[index], self.config.risk_threshold)
            if immediate:
                result[index] = max(result[index], risk)
        return result

    def _condition(self, health: float) -> DepthCondition:
        if health < self.config.critical_score:
            return DepthCondition.CRITICAL
        if health < self.config.healthy_score:
            return DepthCondition.DEGRADED
        return DepthCondition.HEALTHY

    def _critical(
        self,
        reason: str,
        timestamp: float,
        **values,
    ) -> DepthProcessorResult:
        state = DepthState(
            available=False,
            stale=False,
            inference_timestamp_monotonic=timestamp,
            condition=DepthCondition.CRITICAL,
            reasons=(reason,),
            **values,
        )
        return DepthProcessorResult(state=state, normalized_proximity=None)
