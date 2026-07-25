from __future__ import annotations

import asyncio
from contextlib import suppress

from mavsdk import System

from polaris.perception.camera import GazeboCameraSubscriber
from polaris.perception.depth.models import DepthConfig, DepthSourceHealth
from polaris.perception.depth.worker import DepthWorker
from polaris.perception.visual_health import VisualHealthAnalyzer, VisualHealthConfig
from polaris.vlm import VLMSceneState, VLMService, VLMWorker

from .models import NavigationState, VisualState
from .telemetry import TelemetryCollector


class EgoStateCoordinator:
    """Own the single telemetry connection and transient camera pipeline."""

    def __init__(
        self,
        drone: System,
        visual_config: VisualHealthConfig | None = None,
        camera_topic: str | None = None,
        analysis_rate_hz: float = 5.0,
        depth_config: DepthConfig | None = None,
        vlm_service: VLMService | None = None,
    ) -> None:
        if analysis_rate_hz <= 0:
            raise ValueError("analysis_rate_hz must be greater than zero")
        self.state = NavigationState()
        self.telemetry = TelemetryCollector(drone, self.state)
        self.camera = GazeboCameraSubscriber(camera_topic)
        self.analyzer = VisualHealthAnalyzer(visual_config)
        self.analysis_rate_hz = analysis_rate_hz
        self.depth_worker = DepthWorker(depth_config) if depth_config is not None else None
        self.vlm_worker = VLMWorker(vlm_service) if vlm_service is not None else None
        self.state.depth_required_for_motion = bool(
            depth_config and depth_config.depth_required_for_motion
        )
        self._visual_task: asyncio.Task[None] | None = None
        self._last_sequence = 0
        self._vlm_observation_enabled = False

    async def start(self) -> None:
        if self._visual_task is not None:
            return
        self.camera.start()
        try:
            await self.telemetry.start()
            if self.depth_worker is not None:
                self.depth_worker.start()
            if self.vlm_worker is not None:
                self.vlm_worker.start()
            self._visual_task = asyncio.create_task(self._run_visual_analysis())
        except BaseException:
            if self.vlm_worker is not None:
                with suppress(Exception):
                    await asyncio.to_thread(self.vlm_worker.stop)
            if self.depth_worker is not None:
                with suppress(Exception):
                    await asyncio.to_thread(self.depth_worker.stop)
            self.camera.close()
            await self.telemetry.stop()
            raise

    async def stop(self) -> None:
        if self._visual_task is not None:
            self._visual_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._visual_task
            self._visual_task = None
        worker_error: Exception | None = None
        if self.vlm_worker is not None:
            try:
                await asyncio.to_thread(self.vlm_worker.stop)
            except Exception as exc:
                worker_error = exc
        if self.depth_worker is not None:
            try:
                await asyncio.to_thread(self.depth_worker.stop)
            except Exception as exc:
                if worker_error is None:
                    worker_error = exc
        try:
            self.camera.close()
        finally:
            await self.telemetry.stop()
        if worker_error is not None:
            raise worker_error

    def latest_vlm_state(self) -> VLMSceneState | None:
        if self.vlm_worker is None:
            return None
        return self.vlm_worker.latest_state()

    def set_vlm_observation_enabled(self, enabled: bool) -> None:
        self._vlm_observation_enabled = bool(enabled) and self.vlm_worker is not None

    async def _run_visual_analysis(self) -> None:
        interval = 1.0 / self.analysis_rate_hz
        while True:
            frame = self.camera.latest_frame()
            report = await asyncio.to_thread(
                self.analyzer.evaluate,
                frame,
                self.camera.fps,
            )
            last_update = self.state.visual.last_visual_update_monotonic
            new_frame = frame is not None and frame.sequence != self._last_sequence
            if new_frame:
                self._last_sequence = frame.sequence
                last_update = frame.timestamp_monotonic
            self.state.visual = VisualState(
                camera_connected=frame is not None and not report.stale,
                frame_age_ms=report.frame_age_ms,
                fps=report.fps,
                visual_health_score=report.health_score,
                visual_condition=report.condition,
                brightness=report.brightness,
                contrast=report.contrast,
                blur_score=report.blur_score,
                texture_score=report.texture_score,
                feature_count=report.feature_count,
                feature_coverage=report.feature_coverage,
                overexposed_ratio=report.overexposed_ratio,
                underexposed_ratio=report.underexposed_ratio,
                stale=report.stale,
                reasons=report.reasons,
                last_visual_update_monotonic=last_update,
            )
            if self.depth_worker is not None:
                if new_frame and frame is not None:
                    visual = self.state.visual
                    self.depth_worker.submit(
                        frame,
                        DepthSourceHealth(
                            visual_health_score=visual.visual_health_score,
                            visual_condition=visual.visual_condition.value,
                            texture_score=visual.texture_score,
                            overexposed_ratio=visual.overexposed_ratio,
                            underexposed_ratio=visual.underexposed_ratio,
                            reasons=visual.reasons,
                        ),
                    )
                self.state.depth = self.depth_worker.latest_state()
            if (
                new_frame
                and frame is not None
                and self.vlm_worker is not None
                and self._vlm_observation_enabled
            ):
                self.vlm_worker.submit(frame)
            self.state.touch()
            await asyncio.sleep(interval)
