from __future__ import annotations

import sys
import unittest
from pathlib import Path
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.perception.dependencies import add_homebrew_python_site  # noqa: E402

add_homebrew_python_site()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from polaris.perception.models import CameraFrame, VisualCondition  # noqa: E402
from polaris.perception.camera import decode_image_message  # noqa: E402
from polaris.perception.visual_health import VisualHealthAnalyzer  # noqa: E402
from gz.msgs10.image_pb2 import Image, RGB_INT8  # noqa: E402


def frame(image: np.ndarray, age_s: float = 0.0) -> CameraFrame:
    height, width = image.shape[:2]
    return CameraFrame(
        image_bgr=image,
        timestamp_monotonic=monotonic() - age_s,
        source_timestamp=1.0,
        sequence=1,
        width=width,
        height=height,
        pixel_format="BGR_INT8",
    )


def textured_image() -> np.ndarray:
    rng = np.random.default_rng(7)
    image = rng.integers(45, 211, (480, 640, 3), dtype=np.uint8)
    for y in range(40, 440, 80):
        for x in range(40, 600, 80):
            color = (230, 90, 30) if (x + y) // 80 % 2 else (30, 190, 230)
            cv2.rectangle(image, (x, y), (x + 35, y + 35), color, -1)
    return image


class VisualHealthTests(unittest.TestCase):
    def setUp(self) -> None:
        self.analyzer = VisualHealthAnalyzer()

    def test_normal_textured_image_is_healthy(self) -> None:
        report = self.analyzer.evaluate(frame(textured_image()), fps=30.0)
        self.assertEqual(report.condition, VisualCondition.HEALTHY)
        self.assertGreaterEqual(report.health_score, 0.7)
        self.assertGreater(report.feature_count, 150)
        self.assertGreaterEqual(report.feature_coverage, 0.5)

    def test_white_image_is_critical_and_overexposed(self) -> None:
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        report = self.analyzer.evaluate(frame(image), fps=30.0)
        self.assertEqual(report.condition, VisualCondition.CRITICAL)
        self.assertEqual(report.overexposed_ratio, 1.0)

    def test_black_image_is_critical_and_underexposed(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        report = self.analyzer.evaluate(frame(image), fps=30.0)
        self.assertEqual(report.condition, VisualCondition.CRITICAL)
        self.assertEqual(report.underexposed_ratio, 1.0)

    def test_blur_reduces_health(self) -> None:
        sharp = self.analyzer.evaluate(frame(textured_image()), fps=30.0)
        blurred_image = cv2.GaussianBlur(textured_image(), (31, 31), 9)
        blurred = self.analyzer.evaluate(frame(blurred_image), fps=30.0)
        self.assertLess(blurred.blur_score, sharp.blur_score)
        self.assertLess(blurred.health_score, sharp.health_score)
        self.assertIn("Possible blur", blurred.reasons)

    def test_low_texture_gray_image_is_critical(self) -> None:
        image = np.full((480, 640, 3), 128, dtype=np.uint8)
        report = self.analyzer.evaluate(frame(image), fps=30.0)
        self.assertEqual(report.condition, VisualCondition.CRITICAL)
        self.assertEqual(report.feature_count, 0)
        self.assertEqual(report.feature_coverage, 0.0)

    def test_stale_frame_is_critical(self) -> None:
        report = self.analyzer.evaluate(frame(textured_image(), age_s=1.0), fps=0.0)
        self.assertTrue(report.stale)
        self.assertEqual(report.condition, VisualCondition.CRITICAL)

    def test_no_frame_is_critical(self) -> None:
        report = self.analyzer.evaluate(None, fps=0.0)
        self.assertTrue(report.stale)
        self.assertEqual(report.condition, VisualCondition.CRITICAL)

    def test_rgb_message_is_converted_to_bgr(self) -> None:
        message = Image(
            width=2,
            height=1,
            step=6,
            pixel_format_type=RGB_INT8,
            data=bytes([255, 0, 0, 0, 255, 0]),
        )
        decoded = decode_image_message(message)
        np.testing.assert_array_equal(
            decoded.image_bgr,
            np.array([[[0, 0, 255], [0, 255, 0]]], dtype=np.uint8),
        )

    def test_decoder_honors_padded_row_step(self) -> None:
        message = Image(
            width=1,
            height=2,
            step=5,
            pixel_format_type=RGB_INT8,
            data=bytes([1, 2, 3, 99, 99, 4, 5, 6, 88, 88]),
        )
        decoded = decode_image_message(message)
        np.testing.assert_array_equal(
            decoded.image_bgr,
            np.array([[[3, 2, 1]], [[6, 5, 4]]], dtype=np.uint8),
        )


if __name__ == "__main__":
    unittest.main()
