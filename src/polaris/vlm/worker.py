from __future__ import annotations

from dataclasses import dataclass
from threading import Condition, Lock, Thread
from time import monotonic

from polaris.perception.dependencies import add_homebrew_python_site
from polaris.perception.models import CameraFrame

from .models import VLMResult, VLMSceneState, VLMStatus
from .service import VLMService

add_homebrew_python_site()

import cv2  # noqa: E402


@dataclass(frozen=True, slots=True)
class _WorkItem:
    frame: CameraFrame


class VLMWorker:
    """Low-frequency latest-frame observer with no control authority."""

    def __init__(self, service: VLMService) -> None:
        self.service = service
        self.config = service.config
        self._condition = Condition()
        self._state_lock = Lock()
        self._pending: _WorkItem | None = None
        self._state = VLMSceneState(model=self.config.model)
        self._thread: Thread | None = None
        self._stop = False
        self._active = False
        self._dropped_frames = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop = False
        self._thread = Thread(target=self._run, name="polaris-vlm", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float | None = None) -> None:
        timeout = self.config.inference_budget_s + 5.0 if timeout_s is None else timeout_s
        with self._condition:
            self._stop = True
            self._pending = None
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise RuntimeError("VLM worker did not stop within timeout")
            self._thread = None

    def submit(self, frame: CameraFrame) -> None:
        with self._condition:
            if self._pending is not None:
                self._dropped_frames += 1
            self._pending = _WorkItem(frame)
            self._condition.notify()

    def latest_state(self, now_monotonic: float | None = None) -> VLMSceneState:
        now = monotonic() if now_monotonic is None else now_monotonic
        with self._state_lock:
            return self._state.refreshed(self.config.result_stale_after_s, now)

    @property
    def pending_sequence(self) -> int | None:
        with self._condition:
            return None if self._pending is None else self._pending.frame.sequence

    @property
    def inference_active(self) -> bool:
        with self._condition:
            return self._active

    @property
    def dropped_frames(self) -> int:
        with self._condition:
            return self._dropped_frames

    def _run(self) -> None:
        next_inference = 0.0
        while True:
            with self._condition:
                while not self._stop and self._pending is None:
                    self._condition.wait()
                if self._stop:
                    return
                delay = max(0.0, next_inference - monotonic())
                if delay > 0:
                    self._condition.wait(delay)
                    if self._stop:
                        return
                    if monotonic() < next_inference:
                        continue
                item = self._pending
                self._pending = None
                self._active = True
            if item is None:
                continue

            started = monotonic()
            try:
                encoded = self._encode(item.frame)
                result = self.service.analyze(encoded)
                completed = monotonic()
                state = self._scene_state(item.frame, result, started, completed)
            except Exception:
                completed = monotonic()
                state = VLMSceneState(
                    status=VLMStatus.INVALID_RESPONSE,
                    model=self.config.model,
                    source_frame_sequence=item.frame.sequence,
                    source_frame_timestamp_monotonic=item.frame.timestamp_monotonic,
                    inference_timestamp_monotonic=completed,
                    inference_latency_ms=(completed - started) * 1000.0,
                    dropped_frames=self.dropped_frames,
                )
            finally:
                with self._condition:
                    self._active = False
                    self._condition.notify_all()
            with self._state_lock:
                self._state = state
            next_inference = max(
                started + self.config.inference_interval_s,
                monotonic(),
            )

    def _scene_state(
        self,
        frame: CameraFrame,
        result: VLMResult,
        started: float,
        completed: float,
    ) -> VLMSceneState:
        fresh = (
            result.status is VLMStatus.SUCCESS
            and completed - frame.timestamp_monotonic <= self.config.result_stale_after_s
        )
        return VLMSceneState(
            status=result.status if fresh else VLMStatus.STALE if result.status is VLMStatus.SUCCESS else result.status,
            result=result if fresh else None,
            model=result.model,
            source_frame_sequence=frame.sequence,
            source_frame_timestamp_monotonic=frame.timestamp_monotonic,
            inference_timestamp_monotonic=completed,
            inference_latency_ms=(completed - started) * 1000.0,
            dropped_frames=self.dropped_frames,
        )

    @staticmethod
    def _encode(frame: CameraFrame) -> bytes:
        success, encoded = cv2.imencode(
            ".jpg",
            frame.image_bgr,
            (cv2.IMWRITE_JPEG_QUALITY, 85),
        )
        if not success:
            raise ValueError("VLM frame encoding failed")
        return encoded.tobytes()
