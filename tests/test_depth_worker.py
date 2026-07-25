from __future__ import annotations

import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from threading import Event

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from polaris.perception.depth.estimator import DepthInference  # noqa: E402
from polaris.perception.depth.models import (  # noqa: E402
    DepthConfig,
    DepthSourceHealth,
)
from polaris.perception.depth.worker import DepthWorker  # noqa: E402
from polaris.perception.models import CameraFrame  # noqa: E402


class SlowEstimator:
    device = "cpu"

    def __init__(self) -> None:
        self.started = Event()

    def load(self) -> None:
        pass

    def infer(self, image: np.ndarray) -> DepthInference:
        self.started.set()
        time.sleep(0.05)
        depth = np.tile(np.linspace(1.0, 2.0, 30), (30, 1)).astype(np.float32)
        return DepthInference(depth, 30, 30, 30, 30)


def frame(sequence: int) -> CameraFrame:
    return CameraFrame(
        image_bgr=np.zeros((30, 30, 3), dtype=np.uint8),
        timestamp_monotonic=time.monotonic(),
        source_timestamp=None,
        sequence=sequence,
        width=30,
        height=30,
        pixel_format="BGR_INT8",
    )


SOURCE = DepthSourceHealth(0.9, "HEALTHY", 10.0, 0.0, 0.0, ())


class DepthWorkerTests(unittest.TestCase):
    def test_latest_frame_queue_replaces_old_pending_frame(self) -> None:
        estimator = SlowEstimator()
        config = replace(DepthConfig(), target_fps=100.0, persistence_count=1)
        worker = DepthWorker(config, estimator=estimator)
        worker.start()
        try:
            worker.submit(frame(1), SOURCE)
            self.assertTrue(estimator.started.wait(1.0))
            worker.submit(frame(2), SOURCE)
            worker.submit(frame(3), SOURCE)
            deadline = time.monotonic() + 2.0
            while worker.latest_state().source_frame_sequence != 3:
                if time.monotonic() > deadline:
                    self.fail("Latest frame was not processed")
                time.sleep(0.01)
            self.assertGreaterEqual(worker.dropped_frames, 1)
            self.assertEqual(worker.latest_state().source_frame_sequence, 3)
            state, depth_map = worker.latest_snapshot()
            self.assertEqual(state.source_frame_sequence, 3)
            self.assertIsNotNone(depth_map)
        finally:
            worker.stop()


if __name__ == "__main__":
    unittest.main()
