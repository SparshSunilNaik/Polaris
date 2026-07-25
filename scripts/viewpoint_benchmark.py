#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
from contextlib import suppress
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
    ConfidenceReport,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.evaluation.viewpoint import (  # noqa: E402
    BenchmarkConfig,
    BenchmarkRun,
    ViewpointResult,
    ViewpointSample,
    abort_reason,
    aggregate_viewpoint,
    normalize_yaw,
    sample_from_state,
    shortest_yaw_delta,
    write_outputs,
)
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402


class BenchmarkAbort(RuntimeError):
    pass


class ConnectionMonitor:
    def __init__(self, drone: System) -> None:
        self.drone = drone
        self.connected = False
        self._connected_event = asyncio.Event()
        self._task: asyncio.Task[None] | None = None

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._watch())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def wait_connected(self, timeout_s: float = 20.0) -> None:
        await asyncio.wait_for(self._connected_event.wait(), timeout_s)

    async def _watch(self) -> None:
        async for state in self.drone.core.connection_state():
            self.connected = state.is_connected
            if self.connected:
                self._connected_event.set()


class FlightBenchmark:
    def __init__(
        self,
        drone: System,
        coordinator: EgoStateCoordinator,
        confidence: ConfidenceEngine,
        connection: ConnectionMonitor,
        config: BenchmarkConfig,
    ) -> None:
        self.drone = drone
        self.coordinator = coordinator
        self.confidence = confidence
        self.connection = connection
        self.config = config
        self.offboard_started = False
        self.hover_north_m = 0.0
        self.hover_east_m = 0.0
        self.hover_down_m = 0.0
        self.max_horizontal_drift_m = 0.0

    async def run(self) -> tuple[ViewpointResult, ...]:
        await self._wait_ready()
        results: list[ViewpointResult] = []
        try:
            await self._takeoff()
            for index, spec in enumerate(self.config.viewpoints, start=1):
                print(
                    f"[{index}/{len(self.config.viewpoints)}] {spec.name}: "
                    f"PX4 yaw {spec.px4_yaw_deg:+.0f} deg "
                    f"(Gazebo {spec.gazebo_yaw_deg:+.0f} deg)"
                )
                await self._rotate_to(spec.px4_yaw_deg)
                await self._hold(
                    spec.px4_yaw_deg,
                    self.config.settle_duration_s,
                )
                samples = await self._collect_samples(
                    self.config.sample_duration_s
                )
                result = aggregate_viewpoint(spec, samples)
                results.append(result)
                print(
                    f"  yaw={result.measured_yaw_deg:+.1f} deg "
                    f"health={result.median_visual_health_score:.3f} "
                    f"fps={result.median_fps:.1f} samples={result.sample_count}"
                )
        finally:
            await asyncio.shield(self._land_safely())
        return tuple(results)

    async def _wait_ready(self, timeout_s: float = 45.0) -> None:
        print("Waiting for PX4, local position, and fresh RGB...")
        deadline = asyncio.get_running_loop().time() + timeout_s
        while asyncio.get_running_loop().time() < deadline:
            state = self.coordinator.state
            if (
                self.connection.connected
                and state.local_position_ok
                and state.armable
                and state.visual.camera_connected
                and not state.visual.stale
            ):
                print("Vehicle and perception pipeline are ready.")
                return
            await asyncio.sleep(0.2)
        raise BenchmarkAbort("Timed out waiting for vehicle and camera readiness")

    async def _takeoff(self) -> None:
        state = self.coordinator.state
        if state.depth_required_for_motion and (
            not state.depth.available
            or state.depth.stale
            or state.depth.condition.value == "CRITICAL"
        ):
            raise BenchmarkAbort("Required depth is not healthy enough to arm")
        self.hover_north_m = state.position_ned_m.x
        self.hover_east_m = state.position_ned_m.y
        ground_down_m = state.position_ned_m.z
        self.hover_down_m = ground_down_m - self.config.takeoff_altitude_m
        takeoff_yaw = state.orientation.yaw_deg
        print(
            f"Offboard takeoff to {self.config.takeoff_altitude_m:.1f} m at fixed "
            f"N={self.hover_north_m:.2f}, E={self.hover_east_m:.2f}..."
        )
        await self.drone.action.arm()
        await self._send_setpoint(takeoff_yaw)
        await self.drone.offboard.start()
        self.offboard_started = True

        deadline = asyncio.get_running_loop().time() + 30.0
        while asyncio.get_running_loop().time() < deadline:
            self._check_safety()
            await self._send_setpoint(takeoff_yaw)
            state = self.coordinator.state
            if state.in_air and state.altitude_m >= self.config.takeoff_altitude_m - 0.35:
                await asyncio.sleep(1.0)
                print(
                    "Fixed hover target NED: "
                    f"N={self.hover_north_m:.2f}, "
                    f"E={self.hover_east_m:.2f}, "
                    f"D={self.hover_down_m:.2f}"
                )
                return
            await asyncio.sleep(0.1)
        raise BenchmarkAbort("Takeoff altitude was not reached")

    async def _rotate_to(self, target_yaw_deg: float) -> None:
        current = self.coordinator.state.orientation.yaw_deg
        delta = shortest_yaw_delta(target_yaw_deg, current)
        duration = abs(delta) / self.config.yaw_rate_deg_s
        steps = max(1, math.ceil(duration / 0.1))
        step_duration = duration / steps if duration > 0 else 0.1

        for step in range(1, steps + 1):
            self._check_safety()
            intermediate = normalize_yaw(current + delta * step / steps)
            await self._send_setpoint(intermediate)
            await asyncio.sleep(step_duration)

        deadline = asyncio.get_running_loop().time() + 12.0
        stable_count = 0
        while asyncio.get_running_loop().time() < deadline:
            self._check_safety()
            await self._send_setpoint(target_yaw_deg)
            error = abs(
                shortest_yaw_delta(
                    target_yaw_deg,
                    self.coordinator.state.orientation.yaw_deg,
                )
            )
            stable_count = stable_count + 1 if error <= self.config.yaw_tolerance_deg else 0
            if stable_count >= 5:
                return
            await asyncio.sleep(0.1)
        raise BenchmarkAbort(
            f"Yaw failed to settle at {target_yaw_deg:+.1f} degrees"
        )

    async def _hold(self, yaw_deg: float, duration_s: float) -> None:
        deadline = asyncio.get_running_loop().time() + duration_s
        while asyncio.get_running_loop().time() < deadline:
            self._check_safety()
            await self._send_setpoint(yaw_deg)
            await asyncio.sleep(0.1)

    async def _collect_samples(self, duration_s: float) -> list[ViewpointSample]:
        samples: list[ViewpointSample] = []
        last_visual_update = 0.0
        deadline = asyncio.get_running_loop().time() + duration_s
        while asyncio.get_running_loop().time() < deadline:
            confidence = self._check_safety()
            sample = sample_from_state(
                self.coordinator.state,
                confidence,
                self.hover_north_m,
                self.hover_east_m,
            )
            if (
                sample is not None
                and sample.visual_update_monotonic > last_visual_update
            ):
                samples.append(sample)
                last_visual_update = sample.visual_update_monotonic
            await asyncio.sleep(0.02)

        minimum_samples = max(5, int(duration_s * self.config.analysis_rate_hz * 0.4))
        if len(samples) < minimum_samples:
            raise BenchmarkAbort(
                f"Only {len(samples)} fresh visual samples received; "
                f"required at least {minimum_samples}"
            )
        return samples

    def _check_safety(self) -> ConfidenceReport:
        state = self.coordinator.state
        confidence = self.confidence.evaluate(state)
        reason = abort_reason(state, confidence, self.connection.connected)
        if reason is not None:
            raise BenchmarkAbort(reason)
        drift = math.hypot(
            state.position_ned_m.x - self.hover_north_m,
            state.position_ned_m.y - self.hover_east_m,
        )
        if self.hover_down_m != 0.0:
            self.max_horizontal_drift_m = max(
                self.max_horizontal_drift_m,
                drift,
            )
            if drift > self.config.max_horizontal_drift_m:
                raise BenchmarkAbort(
                    f"Horizontal drift {drift:.2f} m exceeded "
                    f"{self.config.max_horizontal_drift_m:.2f} m safety limit"
                )
        return confidence

    async def _send_setpoint(self, yaw_deg: float) -> None:
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                self.hover_north_m,
                self.hover_east_m,
                self.hover_down_m,
                normalize_yaw(yaw_deg),
            )
        )

    async def _land_safely(self) -> None:
        if self.offboard_started:
            with suppress(OffboardError):
                await self.drone.offboard.stop()
            self.offboard_started = False

        state = self.coordinator.state
        if not state.armed and not state.in_air:
            return
        print("Landing...")
        try:
            await self.drone.action.land()
            deadline = asyncio.get_running_loop().time() + 45.0
            while self.coordinator.state.in_air and asyncio.get_running_loop().time() < deadline:
                await asyncio.sleep(0.2)
            if self.coordinator.state.in_air:
                print("WARNING: landing was commanded but in_air did not clear")
            else:
                print("Landing confirmed.")
        except Exception as exc:
            print(f"WARNING: best-effort landing command failed: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run deterministic Polaris viewpoints")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/viewpoints.json"))
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "artifacts/viewpoint_runs"),
    )
    parser.add_argument("--sample-duration", type=float)
    parser.add_argument("--settle-duration", type=float)
    parser.add_argument("--system-address", default="udp://:14540")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--no-flight",
        action="store_true",
        help="Validate live PX4/RGB readiness without arming or writing artifacts",
    )
    parser.add_argument("--json-only", action="store_true")
    return parser.parse_args()


def print_mapping(config: BenchmarkConfig) -> None:
    print("Viewpoint yaw mapping (Gazebo ENU -> PX4 NED):")
    for viewpoint in config.viewpoints:
        print(
            f"  {viewpoint.name:20s} "
            f"Gazebo={viewpoint.gazebo_yaw_deg:+6.0f} deg "
            f"PX4={viewpoint.px4_yaw_deg:+6.0f} deg"
        )


def print_results(results: tuple[ViewpointResult, ...]) -> None:
    print("\nViewpoints ranked by median visual health:")
    print("name                   cmd    measured  health  worst   fps  feat  cov  condition  mode")
    print("-" * 101)
    for result in sorted(
        results,
        key=lambda item: item.median_visual_health_score,
        reverse=True,
    ):
        print(
            f"{result.name:22s} "
            f"{result.commanded_yaw_deg:6.0f} "
            f"{result.measured_yaw_deg:9.1f} "
            f"{result.median_visual_health_score:7.3f} "
            f"{result.worst_visual_health_score:6.3f} "
            f"{result.median_fps:5.1f} "
            f"{result.median_feature_count:5.0f} "
            f"{result.median_feature_coverage:4.2f} "
            f"{result.dominant_visual_condition:10s} "
            f"{result.final_safety_mode}"
        )


async def run_live(args: argparse.Namespace, config: BenchmarkConfig) -> int:
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    drone = System()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(
            PROJECT_ROOT / "config/visual_health.json"
        ),
        analysis_rate_hz=config.analysis_rate_hz,
    )
    confidence = ConfidenceEngine(
        ConfidenceConfig.from_json(PROJECT_ROOT / "config/confidence.json")
    )
    connection = ConnectionMonitor(drone)
    connection.start()
    await coordinator.start()

    try:
        await connection.wait_connected()
        if args.no_flight:
            runner = FlightBenchmark(drone, coordinator, confidence, connection, config)
            await runner._wait_ready()
            print("Live readiness validated; --no-flight requested, exiting without arming.")
            return 0

        started = datetime.now(timezone.utc)
        benchmark = FlightBenchmark(drone, coordinator, confidence, connection, config)
        results = await benchmark.run()
        completed = datetime.now(timezone.utc)
        run = BenchmarkRun(
            started_at_utc=started.isoformat(),
            completed_at_utc=completed.isoformat(),
            hover_reference_ned_m={
                "north": benchmark.hover_north_m,
                "east": benchmark.hover_east_m,
                "down": benchmark.hover_down_m,
            },
            max_horizontal_drift_m=benchmark.max_horizontal_drift_m,
            viewpoints=results,
        )
        stem = started.strftime("%Y%m%dT%H%M%SZ")
        json_path, csv_path = write_outputs(
            run,
            args.output_dir,
            stem,
            json_only=args.json_only,
        )
        print_results(results)
        print(f"\nMaximum horizontal drift: {run.max_horizontal_drift_m:.3f} m")
        print(f"JSON: {json_path}")
        if csv_path is not None:
            print(f"CSV : {csv_path}")
        return 0
    finally:
        try:
            await coordinator.stop()
        finally:
            await connection.stop()


def main() -> int:
    args = parse_args()
    config = BenchmarkConfig.from_json(args.config)
    if args.sample_duration is not None:
        config = replace(config, sample_duration_s=args.sample_duration)
    if args.settle_duration is not None:
        config = replace(config, settle_duration_s=args.settle_duration)
    print_mapping(config)
    if args.dry_run:
        return 0
    try:
        return asyncio.run(run_live(args, config))
    except BenchmarkAbort as exc:
        print(f"BENCHMARK ABORTED: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Benchmark interrupted; landing was requested.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
