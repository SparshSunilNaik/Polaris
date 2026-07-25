#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from time import monotonic, sleep

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.perception.camera import (  # noqa: E402
    DEFAULT_CAMERA_TOPIC,
    GazeboCameraSubscriber,
)
from polaris.perception.dependencies import add_homebrew_python_site  # noqa: E402
from polaris.perception.visual_health import (  # noqa: E402
    VisualHealthAnalyzer,
    VisualHealthConfig,
)

add_homebrew_python_site()

import cv2  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor Gazebo RGB visual health")
    parser.add_argument(
        "--topic",
        default=None,
        help=f"Gazebo image topic (default: auto-discover; standard: {DEFAULT_CAMERA_TOPIC})",
    )
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/visual_health.json"))
    parser.add_argument("--rate", type=float, default=5.0, help="Report rate in Hz")
    parser.add_argument("--duration", type=float, help="Stop after this many seconds")
    parser.add_argument("--json", action="store_true", help="Emit JSON lines")
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--display", action="store_true")
    display.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def terminal_report(report: object, transport_error: str | None) -> str:
    reasons = ", ".join(report.reasons) if report.reasons else "None"
    if transport_error:
        reasons = f"{reasons}; transport: {transport_error}"
    age = "never" if report.frame_age_ms == float("inf") else f"{report.frame_age_ms:.0f} ms"
    return (
        "\033[2J\033[H"
        "POLARIS RGB VISUAL HEALTH\n"
        "=========================\n"
        f"Resolution       : {report.width}x{report.height}\n"
        f"FPS / frame age  : {report.fps:5.1f} / {age} (stale={report.stale})\n"
        f"Condition / score: {report.condition.value} / {report.health_score:.2f}\n"
        f"Brightness       : {report.brightness:7.2f}\n"
        f"Contrast         : {report.contrast:7.2f}\n"
        f"Blur / texture   : {report.blur_score:7.2f} / {report.texture_score:7.2f}\n"
        f"Features         : {report.feature_count:4d} / {report.feature_coverage:.2f} coverage\n"
        f"Over / under exp.: {report.overexposed_ratio:.3f} / {report.underexposed_ratio:.3f}\n"
        f"Reasons          : {reasons}\n"
        "\nPress Ctrl+C to stop."
    )


def overlay(frame: object, report: object) -> object:
    image = frame.image_bgr.copy()
    color = {
        "HEALTHY": (40, 200, 40),
        "DEGRADED": (0, 180, 255),
        "CRITICAL": (20, 20, 230),
    }[report.condition.value]
    lines = [
        f"{report.condition.value} {report.health_score:.2f}",
        f"FPS {report.fps:.1f} age {report.frame_age_ms:.0f}ms",
        f"B {report.brightness:.0f} C {report.contrast:.0f} blur {report.blur_score:.0f}",
        f"features {report.feature_count} coverage {report.feature_coverage:.2f}",
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            image,
            text,
            (20, 35 + index * 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )
    return image


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be greater than zero")
    os.environ.setdefault("GZ_IP", "127.0.0.1")

    config = VisualHealthConfig.from_json(args.config)
    analyzer = VisualHealthAnalyzer(config)
    camera = GazeboCameraSubscriber(args.topic)
    camera.start()
    start = monotonic()

    try:
        while args.duration is None or monotonic() - start < args.duration:
            frame = camera.latest_frame()
            report = analyzer.evaluate(frame, camera.fps)
            if args.json:
                payload = report.to_dict()
                if not math.isfinite(payload["frame_age_ms"]):
                    payload["frame_age_ms"] = None
                if camera.last_error:
                    payload["transport_error"] = camera.last_error
                print(json.dumps(payload, allow_nan=False), flush=True)
            else:
                print(terminal_report(report, camera.last_error), end="", flush=True)

            if args.display and frame is not None:
                cv2.imshow("Polaris RGB Camera", overlay(frame, report))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            sleep(1.0 / args.rate)
    finally:
        camera.close()
        if args.display:
            cv2.destroyAllWindows()
        if not args.json:
            print("\nCamera monitor stopped.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130) from None
