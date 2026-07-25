#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import copy
import os
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path
from time import monotonic

from mavsdk import System

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.operator_view import (  # noqa: E402
    WINDOW_TITLE,
    OperatorRenderer,
    OperatorViewConfig,
    is_quit_key,
    screenshot_path,
)
from polaris.perception.dependencies import add_homebrew_python_site  # noqa: E402
from polaris.perception.depth import DepthConfig  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402

add_homebrew_python_site()
import cv2  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the unified Polaris Operator View")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/operator_view.json"))
    parser.add_argument("--topic", help="Gazebo RGB topic (default: auto-discover X500 camera)")
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-depth-fps", type=float)
    parser.add_argument("--window-width", type=int)
    parser.add_argument("--window-height", type=int)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument(
        "--save-screenshots",
        action="store_true",
        help="Save one screenshot automatically after all enabled panels are live",
    )
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--system-address", default="udp://:14540")
    return parser.parse_args()


def project_path(path: str) -> Path:
    candidate = Path(path).expanduser()
    return candidate if candidate.is_absolute() else PROJECT_ROOT / candidate


async def wait_connected(drone: System, timeout_s: float = 20.0) -> None:
    async def wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(wait(), timeout_s)


def display_fps(timestamps: deque[float]) -> float:
    if len(timestamps) < 2:
        return 0.0
    elapsed = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / elapsed if elapsed > 0 else 0.0


async def save_screenshot(canvas, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = screenshot_path(output_dir)
    saved = await asyncio.to_thread(cv2.imwrite, str(path), canvas)
    if not saved:
        raise RuntimeError(f"OpenCV could not save screenshot: {path}")
    return path


async def run(args: argparse.Namespace) -> int:
    config = OperatorViewConfig.from_json(args.config)
    overrides = {}
    if args.device:
        overrides["depth_device"] = args.device
    if args.target_depth_fps is not None:
        overrides["target_depth_fps"] = args.target_depth_fps
    if args.window_width is not None:
        overrides["window_width"] = args.window_width
    if args.window_height is not None:
        overrides["window_height"] = args.window_height
    if overrides:
        config = replace(config, **overrides)
    config.validate()

    visual_config = VisualHealthConfig.from_json(project_path(config.visual_config))
    confidence = ConfidenceEngine(
        ConfidenceConfig.from_json(project_path(config.confidence_config))
    )
    depth_config = None
    if not args.no_depth:
        depth_config = replace(
            DepthConfig.from_json(project_path(config.depth_config)),
            device=config.depth_device,
            target_fps=config.target_depth_fps,
        )
        depth_config.validate()

    os.environ.setdefault("GZ_IP", "127.0.0.1")
    drone = System()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=visual_config,
        camera_topic=args.topic,
        analysis_rate_hz=max(5.0, config.target_depth_fps * 2.0),
        depth_config=depth_config,
    )
    renderer = OperatorRenderer(config)
    timestamps: deque[float] = deque(maxlen=60)
    grid_overlay = config.rgb_grid_overlay
    paused = False
    help_visible = False
    dirty = True
    automatic_screenshot_saved = False
    canvas = None
    rendered_ready = False
    started = monotonic()
    last_verbose = 0.0
    interval = 1.0 / config.target_display_fps

    print("Waiting for PX4 connection...")

    try:
        await coordinator.start()
        await wait_connected(drone)
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_TITLE, config.window_width, config.window_height)
        print("Operator View connected. Press h for controls; q or Escape quits.")
        while args.duration <= 0 or monotonic() - started < args.duration:
            iteration_started = monotonic()
            state = copy.deepcopy(coordinator.state)
            proximity = None
            if coordinator.depth_worker is not None:
                depth_state, proximity = coordinator.depth_worker.latest_snapshot()
                state.depth = depth_state
            report = confidence.evaluate(state)
            live_ready = state.visual.camera_connected and (
                args.no_depth
                or (
                    state.depth.available
                    and not state.depth.stale
                    and state.depth.inference_fps > 0
                )
            )
            if args.save_screenshots and not automatic_screenshot_saved and live_ready and not rendered_ready:
                dirty = True
            if not paused or dirty or canvas is None:
                frame = coordinator.camera.latest_frame()
                canvas = await asyncio.to_thread(
                    renderer.compose,
                    state,
                    report,
                    frame,
                    proximity,
                    display_fps=display_fps(timestamps),
                    depth_enabled=depth_config is not None,
                    grid_overlay=grid_overlay,
                    paused=paused,
                    help_visible=help_visible,
                )
                timestamps.append(monotonic())
                rendered_ready = live_ready
                dirty = False

            cv2.imshow(WINDOW_TITLE, canvas)
            key = cv2.waitKey(1) & 0xFF
            if is_quit_key(key):
                break
            if key == ord("p"):
                paused = not paused
                dirty = True
            elif key == ord("h"):
                help_visible = not help_visible
                dirty = True
            elif key == ord("g"):
                grid_overlay = not grid_overlay
                dirty = True
            elif key == ord("s"):
                path = await save_screenshot(
                    canvas,
                    project_path(config.screenshot_output_dir),
                )
                print(f"Screenshot saved: {path}")

            try:
                if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break

            if args.save_screenshots and not automatic_screenshot_saved:
                if rendered_ready:
                    path = await save_screenshot(
                        canvas,
                        project_path(config.screenshot_output_dir),
                    )
                    automatic_screenshot_saved = True
                    print(f"Screenshot saved: {path}")

            now = monotonic()
            if args.verbose and now - last_verbose >= 1.0:
                depth = state.depth
                print(
                    f"view={display_fps(timestamps):.1f} fps "
                    f"rgb={coordinator.camera.fps:.1f} fps "
                    f"depth={depth.inference_fps:.2f} fps "
                    f"latency={depth.inference_latency_ms:.1f} ms "
                    f"armed={state.armed} "
                    f"in_air={state.in_air} "
                    f"alt={state.altitude_m:.2f} m "
                    f"yaw={state.orientation.yaw_deg:+.1f} deg "
                    f"visual={state.visual.visual_health_score:.3f} "
                    f"nearest={depth.nearest_region} "
                    f"rgb_stale={state.visual.stale} "
                    f"depth_stale={depth.stale} "
                    f"safety={report.mode.name}"
                )
                last_verbose = now

            await asyncio.sleep(max(0.0, interval - (monotonic() - iteration_started)))
    finally:
        try:
            await coordinator.stop()
        finally:
            cv2.destroyAllWindows()

    print(
        f"Operator View stopped. Display {display_fps(timestamps):.1f} FPS; "
        f"RGB {coordinator.camera.fps:.1f} FPS."
    )
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nOperator View interrupted; shared pipelines stopped.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
