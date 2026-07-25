from __future__ import annotations

import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from threading import Event, Lock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from polaris.perception.models import CameraFrame  # noqa: E402
from polaris.vlm import (  # noqa: E402
    VLMConfig,
    VLMResult,
    VLMStatus,
    VLMWorker,
)


def successful_result(summary: str = "Snowfield with station ahead") -> VLMResult:
    return VLMResult(
        status=VLMStatus.SUCCESS,
        terrain_type="snowfield",
        visible_landmarks=("station",),
        target_visible=True,
        scene_confidence=0.9,
        texture_quality=0.6,
        scene_summary=summary,
        model="qwen2.5vl:7b",
    )


def frame(sequence: int, *, age_s: float = 0.0) -> CameraFrame:
    return CameraFrame(
        image_bgr=np.zeros((24, 24, 3), dtype=np.uint8),
        timestamp_monotonic=time.monotonic() - age_s,
        source_timestamp=None,
        sequence=sequence,
        width=24,
        height=24,
        pixel_format="BGR_INT8",
    )


class FakeService:
    def __init__(self, result: VLMResult, config: VLMConfig | None = None) -> None:
        self.config = config or replace(
            VLMConfig(),
            inference_interval_s=0.01,
            result_stale_after_s=1.0,
            inference_timeout_s=0.2,
        )
        self.result = result
        self.calls = 0

    def analyze(self, image: bytes) -> VLMResult:
        self.calls += 1
        return self.result


class BlockingService(FakeService):
    def __init__(self) -> None:
        super().__init__(successful_result())
        self.started = Event()
        self.release = Event()
        self.lock = Lock()
        self.active = 0
        self.maximum_active = 0

    def analyze(self, image: bytes) -> VLMResult:
        with self.lock:
            self.calls += 1
            call = self.calls
            self.active += 1
            self.maximum_active = max(self.maximum_active, self.active)
        if call == 1:
            self.started.set()
            self.release.wait(1.0)
        with self.lock:
            self.active -= 1
        return self.result


class VLMWorkerTests(unittest.TestCase):
    @staticmethod
    def wait_for(predicate, timeout_s: float = 1.0) -> None:
        deadline = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError("Timed out waiting for VLM worker")
            time.sleep(0.005)

    def test_successful_result_is_published_with_frame_metadata(self) -> None:
        worker = VLMWorker(FakeService(successful_result()))
        worker.start()
        try:
            worker.submit(frame(7))
            self.wait_for(lambda: worker.latest_state().source_frame_sequence == 7)
            state = worker.latest_state()
            self.assertEqual(state.status, VLMStatus.SUCCESS)
            self.assertTrue(state.fresh)
            self.assertEqual(state.result.terrain_type, "snowfield")
            self.assertGreater(state.inference_timestamp_monotonic, 0.0)
        finally:
            worker.stop()

    def test_unavailable_ollama_degrades_silently(self) -> None:
        service = FakeService(
            VLMResult.degraded(VLMStatus.UNAVAILABLE, "qwen2.5vl:7b")
        )
        worker = VLMWorker(service)
        worker.start()
        try:
            worker.submit(frame(1))
            self.wait_for(lambda: worker.latest_state().source_frame_sequence == 1)
            self.assertEqual(worker.latest_state().status, VLMStatus.UNAVAILABLE)
            self.assertIsNone(worker.latest_state().result)
        finally:
            worker.stop()

    def test_timeout_degrades_silently(self) -> None:
        service = FakeService(VLMResult.degraded(VLMStatus.TIMEOUT, "qwen2.5vl:7b"))
        worker = VLMWorker(service)
        worker.start()
        try:
            worker.submit(frame(2))
            self.wait_for(lambda: worker.latest_state().source_frame_sequence == 2)
            self.assertEqual(worker.latest_state().status, VLMStatus.TIMEOUT)
        finally:
            worker.stop()

    def test_stale_source_frame_is_not_published(self) -> None:
        config = replace(
            VLMConfig(),
            inference_interval_s=0.01,
            result_stale_after_s=0.1,
            inference_timeout_s=0.2,
        )
        worker = VLMWorker(FakeService(successful_result(), config))
        worker.start()
        try:
            worker.submit(frame(3, age_s=1.0))
            self.wait_for(lambda: worker.latest_state().source_frame_sequence == 3)
            self.assertEqual(worker.latest_state().status, VLMStatus.STALE)
            self.assertIsNone(worker.latest_state().result)
        finally:
            worker.stop()

    def test_running_inference_keeps_only_latest_pending_frame(self) -> None:
        service = BlockingService()
        worker = VLMWorker(service)
        worker.start()
        try:
            worker.submit(frame(1))
            self.assertTrue(service.started.wait(1.0))
            worker.submit(frame(2))
            worker.submit(frame(3))
            time.sleep(0.02)
            self.assertEqual(service.calls, 1)
            self.assertEqual(service.maximum_active, 1)
            self.assertEqual(worker.pending_sequence, 3)
            service.release.set()
            self.wait_for(lambda: worker.latest_state().source_frame_sequence == 3)
            self.assertEqual(service.calls, 2)
            self.assertEqual(service.maximum_active, 1)
            self.assertGreaterEqual(worker.dropped_frames, 1)
        finally:
            service.release.set()
            worker.stop()

    def test_clean_shutdown_is_idempotent(self) -> None:
        worker = VLMWorker(FakeService(successful_result()))
        worker.start()
        worker.submit(frame(1))
        self.wait_for(lambda: worker.latest_state().source_frame_sequence == 1)
        worker.stop()
        worker.stop()
        self.assertFalse(worker.inference_active)


if __name__ == "__main__":
    unittest.main()
