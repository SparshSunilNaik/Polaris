from __future__ import annotations

import json
from dataclasses import dataclass, fields
from pathlib import Path
from time import monotonic, time

from .dependencies import add_homebrew_python_site
from .models import CameraFrame, VisualCondition, VisualHealthReport

add_homebrew_python_site()

import cv2  # noqa: E402
import numpy as np  # noqa: E402


@dataclass(frozen=True)
class VisualHealthConfig:
    stale_after_ms: float = 500.0
    brightness_critical_low: float = 15.0
    brightness_healthy_low: float = 55.0
    brightness_healthy_high: float = 210.0
    brightness_critical_high: float = 245.0
    contrast_critical: float = 8.0
    contrast_healthy: float = 35.0
    exposure_pixel_low: int = 5
    exposure_pixel_high: int = 250
    exposure_degraded_ratio: float = 0.15
    exposure_critical_ratio: float = 0.80
    blur_critical: float = 20.0
    blur_healthy: float = 100.0
    texture_critical: float = 2.0
    texture_healthy: float = 12.0
    features_critical: int = 20
    features_healthy: int = 150
    coverage_critical: float = 0.125
    coverage_healthy: float = 0.50
    healthy_score: float = 0.70
    critical_score: float = 0.35
    feature_max_count: int = 500
    feature_quality_level: float = 0.01
    feature_min_distance: float = 8.0
    coverage_grid_size: int = 4
    weight_brightness: float = 0.15
    weight_contrast: float = 0.15
    weight_exposure: float = 0.15
    weight_blur: float = 0.20
    weight_texture: float = 0.15
    weight_features: float = 0.20

    @classmethod
    def from_json(cls, path: str | Path) -> VisualHealthConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {field.name for field in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown visual-health settings: {sorted(unknown)}")
        return cls(**values)


def _ramp(value: float, critical: float, healthy: float) -> float:
    if healthy == critical:
        return float(value >= healthy)
    return float(np.clip((value - critical) / (healthy - critical), 0.0, 1.0))


class VisualHealthAnalyzer:
    def __init__(self, config: VisualHealthConfig | None = None) -> None:
        self.config = config or VisualHealthConfig()

    def evaluate(
        self,
        frame: CameraFrame | None,
        fps: float,
        now_monotonic: float | None = None,
    ) -> VisualHealthReport:
        now_monotonic = monotonic() if now_monotonic is None else now_monotonic
        if frame is None:
            return VisualHealthReport(
                timestamp=time(),
                frame_age_ms=float("inf"),
                stale=True,
                fps=fps,
                width=0,
                height=0,
                brightness=0.0,
                contrast=0.0,
                blur_score=0.0,
                texture_score=0.0,
                feature_count=0,
                feature_coverage=0.0,
                overexposed_ratio=0.0,
                underexposed_ratio=0.0,
                health_score=0.0,
                condition=VisualCondition.CRITICAL,
                reasons=("No camera frame received",),
            )

        frame_age_ms = max(0.0, (now_monotonic - frame.timestamp_monotonic) * 1000)
        stale = frame_age_ms > self.config.stale_after_ms
        gray = cv2.cvtColor(frame.image_bgr, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray))
        contrast = float(np.std(gray))
        overexposed_ratio = float(np.mean(gray >= self.config.exposure_pixel_high))
        underexposed_ratio = float(np.mean(gray <= self.config.exposure_pixel_low))
        blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        gradient_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gradient_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        texture_score = float(np.mean(cv2.magnitude(gradient_x, gradient_y)))

        corners = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.config.feature_max_count,
            qualityLevel=self.config.feature_quality_level,
            minDistance=self.config.feature_min_distance,
        )
        feature_count = 0 if corners is None else len(corners)
        feature_coverage = self._feature_coverage(corners, frame.width, frame.height)

        brightness_score = self._brightness_score(brightness)
        contrast_score = _ramp(
            contrast, self.config.contrast_critical, self.config.contrast_healthy
        )
        exposure_ratio = max(overexposed_ratio, underexposed_ratio)
        exposure_score = 1.0 - _ramp(
            exposure_ratio,
            self.config.exposure_degraded_ratio,
            self.config.exposure_critical_ratio,
        )
        blur_health = _ramp(
            blur_score, self.config.blur_critical, self.config.blur_healthy
        )
        texture_health = _ramp(
            texture_score, self.config.texture_critical, self.config.texture_healthy
        )
        count_health = _ramp(
            feature_count, self.config.features_critical, self.config.features_healthy
        )
        coverage_health = _ramp(
            feature_coverage,
            self.config.coverage_critical,
            self.config.coverage_healthy,
        )
        features_health = min(count_health, coverage_health)

        weighted = (
            self.config.weight_brightness * brightness_score
            + self.config.weight_contrast * contrast_score
            + self.config.weight_exposure * exposure_score
            + self.config.weight_blur * blur_health
            + self.config.weight_texture * texture_health
            + self.config.weight_features * features_health
        )
        weight_sum = (
            self.config.weight_brightness
            + self.config.weight_contrast
            + self.config.weight_exposure
            + self.config.weight_blur
            + self.config.weight_texture
            + self.config.weight_features
        )
        health_score = float(np.clip(weighted / weight_sum, 0.0, 1.0))
        reasons = self._reasons(
            stale,
            brightness,
            contrast,
            blur_score,
            texture_score,
            feature_count,
            feature_coverage,
            overexposed_ratio,
            underexposed_ratio,
        )

        critical_override = stale or exposure_ratio >= self.config.exposure_critical_ratio
        if critical_override or health_score < self.config.critical_score:
            condition = VisualCondition.CRITICAL
        elif health_score < self.config.healthy_score:
            condition = VisualCondition.DEGRADED
        else:
            condition = VisualCondition.HEALTHY

        return VisualHealthReport(
            timestamp=time(),
            frame_age_ms=frame_age_ms,
            stale=stale,
            fps=fps,
            width=frame.width,
            height=frame.height,
            brightness=brightness,
            contrast=contrast,
            blur_score=blur_score,
            texture_score=texture_score,
            feature_count=feature_count,
            feature_coverage=feature_coverage,
            overexposed_ratio=overexposed_ratio,
            underexposed_ratio=underexposed_ratio,
            health_score=health_score,
            condition=condition,
            reasons=tuple(reasons),
        )

    def _brightness_score(self, brightness: float) -> float:
        if brightness < self.config.brightness_healthy_low:
            return _ramp(
                brightness,
                self.config.brightness_critical_low,
                self.config.brightness_healthy_low,
            )
        if brightness > self.config.brightness_healthy_high:
            return 1.0 - _ramp(
                brightness,
                self.config.brightness_healthy_high,
                self.config.brightness_critical_high,
            )
        return 1.0

    def _feature_coverage(
        self, corners: np.ndarray | None, width: int, height: int
    ) -> float:
        if corners is None or width <= 0 or height <= 0:
            return 0.0
        grid = self.config.coverage_grid_size
        occupied: set[tuple[int, int]] = set()
        for x, y in corners.reshape(-1, 2):
            cell_x = min(int(x * grid / width), grid - 1)
            cell_y = min(int(y * grid / height), grid - 1)
            occupied.add((cell_x, cell_y))
        return len(occupied) / (grid * grid)

    def _reasons(
        self,
        stale: bool,
        brightness: float,
        contrast: float,
        blur_score: float,
        texture_score: float,
        feature_count: int,
        feature_coverage: float,
        overexposed_ratio: float,
        underexposed_ratio: float,
    ) -> list[str]:
        reasons: list[str] = []
        if stale:
            reasons.append("Camera frame is stale")
        if brightness < self.config.brightness_healthy_low:
            reasons.append("Image is dark")
        elif brightness > self.config.brightness_healthy_high:
            reasons.append("Image is bright")
        if contrast < self.config.contrast_healthy:
            reasons.append("Low contrast")
        if overexposed_ratio > self.config.exposure_degraded_ratio:
            reasons.append("High overexposed-pixel ratio")
        if underexposed_ratio > self.config.exposure_degraded_ratio:
            reasons.append("High underexposed-pixel ratio")
        if blur_score < self.config.blur_healthy:
            reasons.append("Possible blur")
        if texture_score < self.config.texture_healthy:
            reasons.append("Low texture")
        if feature_count < self.config.features_healthy:
            reasons.append("Few visual features")
        if feature_coverage < self.config.coverage_healthy:
            reasons.append("Poor feature distribution")
        return reasons
