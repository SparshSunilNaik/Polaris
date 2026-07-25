#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from dataclasses import replace
from pathlib import Path
from time import monotonic

from mavsdk import System

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.perception.dependencies import add_homebrew_python_site  # noqa: E402
from polaris.perception.depth import DepthConfig  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402

add_homebrew_python_site()
import cv2  # noqa: E402
import numpy as np  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor relative pseudo-depth")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/depth_anything.json"))
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-fps", type=float)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--system-address", default="udp://:14540")
    return parser.parse_args()


async def wait_connected(drone: System, timeout_s: float = 20.0) -> None:
    async def wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(wait(), timeout_s)


def print_state(state, rgb_fps: float, as_json: bool) -> None:
    depth = state.depth
    if as_json:
        payload = depth.to_dict()
        payload["rgb_fps"] = rgb_fps
        print(json.dumps(payload, separators=(",", ":")))
        return
    grid = depth.risk_grid
    print(
        f"model={depth.model_name or 'loading'} device={depth.device or '-'} "
        f"input={depth.input_width}x{depth.input_height} "
        f"output={depth.output_width}x{depth.output_height}"
    )
    print(
        f"latency={depth.inference_latency_ms:6.1f} ms "
        f"warmup={depth.warmup_latency_ms:6.1f} ms "
        f"depth_fps={depth.inference_fps:4.2f} rgb_fps={rgb_fps:4.1f} "
        f"source_age={depth.source_frame_age_ms:6.1f} ms stale={depth.stale}"
    )
    print(
        f"condition={depth.condition.value} health={depth.depth_health_score:.3f} "
        f"stability={depth.temporal_stability:.3f} "
        f"nearest={depth.nearest_region} safest={depth.safest_region}"
    )
    print("RELATIVE PSEUDO-DEPTH RISK (1=near concern, 0=clear)")
    for row in grid:
        print("  " + " ".join(f"{value:5.2f}" for value in row))
    print("reasons=" + ("; ".join(depth.reasons) if depth.reasons else "None"))
    print()


def render(coordinator: EgoStateCoordinator) -> bool:
    worker = coordinator.depth_worker
    frame = coordinator.camera.latest_frame()
    proximity = None if worker is None else worker.latest_depth_map()
    if frame is None or proximity is None:
        return True
    rgb = cv2.resize(frame.image_bgr, (640, 480), interpolation=cv2.INTER_AREA)
    risk = cv2.resize(proximity, (640, 480), interpolation=cv2.INTER_CUBIC)
    color = cv2.applyColorMap(np.uint8(np.clip(risk, 0, 1) * 255), cv2.COLORMAP_TURBO)
    depth = coordinator.state.depth
    cv2.putText(
        color,
        "RELATIVE PSEUDO-DEPTH RISK - NOT METRES",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        color,
        f"{depth.condition.value} {depth.inference_latency_ms:.0f} ms  red=near concern",
        (12, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("Polaris RGB | Relative Pseudo-Depth", np.hstack((rgb, color)))
    return cv2.waitKey(1) & 0xFF != ord("q")


async def run(args: argparse.Namespace) -> None:
    config = DepthConfig.from_json(args.config)
    if args.device:
        config = replace(config, device=args.device)
    if args.target_fps:
        config = replace(config, target_fps=args.target_fps)
    config.validate()
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    drone = System()
    await drone.connect(system_address=args.system_address)
    await wait_connected(drone)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(
            PROJECT_ROOT / "config/visual_health.json"
        ),
        analysis_rate_hz=max(5.0, config.target_fps * 2.0),
        depth_config=config,
    )
    await coordinator.start()
    started = monotonic()
    next_print = 0.0
    display = args.display and not args.no_display
    try:
        while args.duration <= 0 or monotonic() - started < args.duration:
            now = monotonic()
            if now >= next_print:
                print_state(coordinator.state, coordinator.camera.fps, args.json)
                next_print = now + 1.0
            if display and not render(coordinator):
                break
            await asyncio.sleep(0.02)
    finally:
        await coordinator.stop()
        if display:
            cv2.destroyAllWindows()


def main() -> int:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
