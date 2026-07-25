from __future__ import annotations

import argparse
import asyncio
import os
import sys
from pathlib import Path

from mavsdk import System


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_ROOT = PROJECT_ROOT / "src"
sys.path.insert(0, str(SRC_ROOT))

from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
    ConfidenceReport,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.ego_state.models import NavigationState  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the integrated Polaris Ego State")
    parser.add_argument("--system-address", default="udp://:14540")
    parser.add_argument(
        "--camera-topic",
        default=None,
        help="Gazebo image topic (default: auto-discover X500 mono camera)",
    )
    parser.add_argument(
        "--visual-config",
        default=str(PROJECT_ROOT / "config/visual_health.json"),
    )
    parser.add_argument(
        "--confidence-config",
        default=str(PROJECT_ROOT / "config/confidence.json"),
    )
    parser.add_argument("--rate", type=float, default=5.0, help="Display rate in Hz")
    parser.add_argument("--duration", type=float, help="Stop after this many seconds")
    return parser.parse_args()


async def wait_for_connection(drone: System) -> None:
    print("Waiting for PX4...")
    async for connection in drone.core.connection_state():
        if connection.is_connected:
            print("Connected to PX4.\n")
            return


def print_state(state: NavigationState, report: ConfidenceReport) -> None:
    position = state.position_ned_m
    velocity = state.velocity_ned_m_s
    attitude = state.orientation
    visual = state.visual

    battery = (
        f"{state.battery_remaining * 100:.1f}%"
        if state.battery_remaining is not None
        else "unknown"
    )
    frame_age = (
        "never" if visual.frame_age_ms == float("inf") else f"{visual.frame_age_ms:.0f} ms"
    )
    reasons = ", ".join(report.reasons) if report.reasons else "None"
    overrides = (
        ", ".join(report.critical_overrides)
        if report.critical_overrides
        else "None"
    )

    print(
        "\033[2J\033[H"
        "PROJECT POLARIS - INTEGRATED EGO STATE v0.3\n"
        "==========================================\n"
        "NAVIGATION\n"
        f"Position NED [m] : N={position.x:7.2f}  E={position.y:7.2f}  D={position.z:7.2f}\n"
        f"Altitude [m]     : {state.altitude_m:7.2f}\n"
        f"Velocity NED     : N={velocity.x:7.2f}  E={velocity.y:7.2f}  D={velocity.z:7.2f}\n"
        f"Horizontal speed : {state.horizontal_speed_m_s:7.2f} m/s\n"
        f"Attitude [deg]   : R={attitude.roll_deg:7.2f}  P={attitude.pitch_deg:7.2f}  Y={attitude.yaw_deg:7.2f}\n"
        f"Armed / in air   : {state.armed} / {state.in_air}\n"
        f"Position health  : local={state.local_position_ok} global={state.global_position_ok} home={state.home_position_ok}\n"
        f"Position stale   : {state.position_is_stale()}\n"
        f"Attitude stale   : {state.attitude_is_stale()}\n"
        f"Battery          : {battery}\n"
        "\nVISUAL PERCEPTION\n"
        f"Camera connected : {visual.camera_connected}\n"
        f"Frame age / FPS  : {frame_age} / {visual.fps:.1f} (stale={visual.stale})\n"
        f"Condition / score: {visual.visual_condition.value} / {visual.visual_health_score:.2f}\n"
        f"Brightness       : {visual.brightness:7.2f}\n"
        f"Contrast         : {visual.contrast:7.2f}\n"
        f"Blur / texture   : {visual.blur_score:7.2f} / {visual.texture_score:7.2f}\n"
        f"Features         : {visual.feature_count:4d} / {visual.feature_coverage:.2f} coverage\n"
        f"Over / under exp.: {visual.overexposed_ratio:.3f} / {visual.underexposed_ratio:.3f}\n"
        "\nCONFIDENCE ENGINE\n"
        f"Safety mode      : {report.mode.name}\n"
        f"Overall health   : {report.overall_health:.2f}\n"
        f"Visual health    : {report.visual_health:.2f}\n"
        f"Camera freshness : {report.camera_freshness:.2f}\n"
        f"Exposure quality : {report.exposure_quality:.2f}\n"
        f"Texture/features : {report.texture_feature_quality:.2f}\n"
        f"Localization     : {report.localization_health:.2f}\n"
        f"Telemetry        : {report.telemetry_health:.2f}\n"
        f"Battery health   : {report.battery_health:.2f}\n"
        f"Motion health    : {report.motion_health:.2f}\n"
        f"Hard overrides   : {overrides}\n"
        f"Reasons          : {reasons}\n"
        "\nPress Ctrl+C to stop."
    )


async def run(args: argparse.Namespace) -> None:
    if args.rate <= 0:
        raise ValueError("--rate must be greater than zero")
    os.environ.setdefault("GZ_IP", "127.0.0.1")

    drone = System()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(args.visual_config),
        camera_topic=args.camera_topic,
        analysis_rate_hz=args.rate,
    )
    confidence_engine = ConfidenceEngine(
        ConfidenceConfig.from_json(args.confidence_config)
    )
    await coordinator.start()
    loop = asyncio.get_running_loop()
    started_at = loop.time()

    try:
        await wait_for_connection(drone)
        while args.duration is None or loop.time() - started_at < args.duration:
            report = confidence_engine.evaluate(coordinator.state)
            print_state(coordinator.state, report)
            await asyncio.sleep(1.0 / args.rate)
    finally:
        await coordinator.stop()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nEgo-state monitor stopped.")


if __name__ == "__main__":
    main()
