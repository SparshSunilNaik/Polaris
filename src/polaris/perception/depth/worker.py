from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Condition, Lock, Thread
from time import monotonic

from polaris.perception.models import CameraFrame

from .estimator import DepthAnythingEstimator
from .models import DepthConfig, DepthSourceHealth, DepthState
from .processor import DepthProcessor


@dataclass(frozen=True)
class _WorkItem:
    frame: CameraFrame
    source_health: DepthSourceHealth


class DepthWorker:
    """Dedicated latest-frame worker with a one-item replacement queue."""

    def __init__(
        self,
        config: DepthConfig,
        estimator: DepthAnythingEstimator | None = None,
        processor: DepthProcessor | None = None,
    ) -> None:
        self.config = config
        self.estimator = estimator or DepthAnythingEstimator(config)
        self.processor = processor or DepthProcessor(config)
        self._condition = Condition()
        self._state_lock = Lock()
        self._pending: _WorkItem | None = None
        self._state = DepthState(
            model_name=config.model_identifier,
            model_revision=config.model_revision,
            device=self.estimator.device,
            reasons=("Depth model is loading",),
        )
        self._latest_map = None
        self._thread: Thread | None = None
        self._stop = False
        self._dropped_frames = 0
        self._consecutive_failures = 0
        self._completion_times: deque[float] = deque(maxlen=30)
        self._warmup_latency_ms = 0.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop = False
        self._thread = Thread(target=self._run, name="polaris-depth", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 30.0) -> None:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout_s)
            if self._thread.is_alive():
                raise RuntimeError("Depth worker did not stop within timeout")
            self._thread = None

    def submit(self, frame: CameraFrame, source_health: DepthSourceHealth) -> None:
        with self._condition:
            if self._pending is not None:
                self._dropped_frames += 1
            self._pending = _WorkItem(frame, source_health)
            self._condition.notify()

    def latest_state(self) -> DepthState:
        with self._state_lock:
            return self._state.refreshed(self.config.stale_after_ms)

    def latest_depth_map(self):
        with self._state_lock:
            return self._latest_map

    def latest_snapshot(self):
        """Return matching compressed state and transient map under one lock."""
        with self._state_lock:
            return (
                self._state.refreshed(self.config.stale_after_ms),
                self._latest_map,
            )

    @property
    def dropped_frames(self) -> int:
        return self._dropped_frames

    @property
    def pending_sequence(self) -> int | None:
        with self._condition:
            return None if self._pending is None else self._pending.frame.sequence

    def _run(self) -> None:
        try:
            self.estimator.load()
        except Exception as exc:
            self._record_failure(f"Depth model load failed: {exc}")
            return

        interval = 1.0 / self.config.target_fps
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
            if item is None:
                continue

            started = monotonic()
            try:
                inference = self.estimator.infer(item.frame.image_bgr)
                completed = monotonic()
                latency_ms = (completed - started) * 1000.0
                if self._warmup_latency_ms == 0.0:
                    self._warmup_latency_ms = latency_ms
                self._completion_times.append(completed)
                fps = self._inference_fps()
                result = self.processor.process(
                    inference.raw_depth,
                    source_frame_sequence=item.frame.sequence,
                    source_frame_age_ms=(completed - item.frame.timestamp_monotonic) * 1000.0,
                    inference_timestamp_monotonic=completed,
                    inference_latency_ms=latency_ms,
                    warmup_latency_ms=self._warmup_latency_ms,
                    inference_fps=fps,
                    model_name=self.config.model_identifier,
                    model_revision=self.config.model_revision,
                    device=self.estimator.device,
                    input_width=inference.input_width,
                    input_height=inference.input_height,
                    source_health=item.source_health,
                    dropped_frames=self._dropped_frames,
                )
                self._consecutive_failures = 0
                with self._state_lock:
                    self._state = result.state
                    self._latest_map = result.normalized_proximity
            except Exception as exc:
                self._record_failure(f"Depth inference failed: {exc}")
            next_inference = max(started + interval, monotonic())

    def _record_failure(self, reason: str) -> None:
        self._consecutive_failures += 1
        state = self.processor.failure_state(
            reason,
            model_name=self.config.model_identifier,
            model_revision=self.config.model_revision,
            device=self.estimator.device,
            consecutive_failures=self._consecutive_failures,
            dropped_frames=self._dropped_frames,
        )
        with self._state_lock:
            self._state = state
            self._latest_map = None

    def _inference_fps(self) -> float:
        if len(self._completion_times) < 2:
            return 0.0
        elapsed = self._completion_times[-1] - self._completion_times[0]
        return (len(self._completion_times) - 1) / elapsed if elapsed > 0 else 0.0
