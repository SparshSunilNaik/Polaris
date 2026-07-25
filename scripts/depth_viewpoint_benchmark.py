#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import os
import sys
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic

from mavsdk import System

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from viewpoint_benchmark import (  # noqa: E402
    BenchmarkAbort,
    ConnectionMonitor,
    FlightBenchmark,
    print_mapping,
)

from polaris.ego_state.confidence import (  # noqa: E402
    ConfidenceConfig,
    ConfidenceEngine,
)
from polaris.ego_state.coordinator import EgoStateCoordinator  # noqa: E402
from polaris.evaluation.depth_viewpoint import (  # noqa: E402
    DepthBenchmarkRun,
    aggregate_depth_viewpoint,
    load_visual_baseline,
    write_depth_outputs,
)
from polaris.evaluation.viewpoint import (  # noqa: E402
    BenchmarkConfig,
    ViewpointSample,
    aggregate_viewpoint,
    sample_from_state,
)
from polaris.perception.depth import DepthConfig, DepthState  # noqa: E402
from polaris.perception.visual_health import VisualHealthConfig  # noqa: E402


DEFAULT_BASELINE = (
    PROJECT_ROOT / "artifacts/viewpoint_runs/20260723T141537Z_viewpoints.json"
)

SAFE_DEPTH_ORDER = (
    "west_snowfield",
    "southwest_snow",
    "south_station",
    "southeast_corridor",
    "east_corridor",
    "northeast_rocks",
    "north_rocks",
    "northwest_glare",
)


class DepthFlightBenchmark(FlightBenchmark):
    def __init__(self, *args, depth_config: DepthConfig, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.depth_config = depth_config

    async def run_depth(self, baseline: dict[str, float]):
        await self._wait_ready()
        await self._wait_depth_ready()
        results = []
        try:
            await self._takeoff()
            for index, spec in enumerate(self.config.viewpoints, start=1):
                print(
                    f"[{index}/{len(self.config.viewpoints)}] {spec.name}: "
                    f"PX4 yaw {spec.px4_yaw_deg:+.0f} deg"
                )
                await self._rotate_to(spec.px4_yaw_deg)
                await self._hold(spec.px4_yaw_deg, self.config.settle_duration_s)
                visual_samples, depth_samples = await self._collect_depth_samples(
                    self.config.sample_duration_s
                )
                visual_result = aggregate_viewpoint(spec, visual_samples)
                result = aggregate_depth_viewpoint(
                    visual_result,
                    depth_samples,
                    baseline.get(spec.name),
                )
                results.append(result)
                print(
                    f"  visual={visual_result.median_visual_health_score:.3f} "
                    f"depth={result.median_depth_health_score:.3f} "
                    f"latency={result.median_depth_latency_ms:.1f} ms "
                    f"depth_fps={result.median_depth_fps:.2f} "
                    f"nearest={result.nearest_region}"
                )
        finally:
            await asyncio.shield(self._land_safely())
        return tuple(results)

    async def _wait_depth_ready(self, timeout_s: float = 90.0) -> None:
        print("Waiting for the pinned Depth Anything model and first valid inference...")
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            depth = self.coordinator.state.depth
            if depth.available and not depth.stale:
                print(
                    f"Depth ready on {depth.device}: warmup "
                    f"{depth.warmup_latency_ms:.1f} ms"
                )
                return
            if depth.consecutive_failures > 0:
                raise BenchmarkAbort("; ".join(depth.reasons))
            await asyncio.sleep(0.1)
        raise BenchmarkAbort("Timed out waiting for fresh depth inference")

    async def _collect_depth_samples(
        self,
        duration_s: float,
    ) -> tuple[list[ViewpointSample], list[DepthState]]:
        visual_samples: list[ViewpointSample] = []
        depth_samples: list[DepthState] = []
        last_visual_update = 0.0
        last_depth_update = 0.0
        deadline = monotonic() + duration_s
        while monotonic() < deadline:
            confidence = self._check_safety()
            state = self.coordinator.state
            visual = sample_from_state(
                state,
                confidence,
                self.hover_north_m,
                self.hover_east_m,
            )
            if visual is not None and visual.visual_update_monotonic > last_visual_update:
                visual_samples.append(visual)
                last_visual_update = visual.visual_update_monotonic
            depth = state.depth
            if depth.consecutive_failures > 0:
                raise BenchmarkAbort("; ".join(depth.reasons))
            if depth.inference_timestamp_monotonic > 0 and depth.stale:
                raise BenchmarkAbort("Depth became stale during viewpoint sampling")
            if (
                depth.available
                and not depth.stale
                and depth.inference_timestamp_monotonic > last_depth_update
            ):
                depth_samples.append(depth)
                last_depth_update = depth.inference_timestamp_monotonic
            await asyncio.sleep(0.02)

        if len(visual_samples) < 5:
            raise BenchmarkAbort("Insufficient fresh visual samples")
        minimum_depth = max(5, int(duration_s * self.depth_config.target_fps * 0.7))
        if len(depth_samples) < minimum_depth:
            raise BenchmarkAbort(
                f"Only {len(depth_samples)} fresh depth samples; "
                f"required at least {minimum_depth}"
            )
        return visual_samples, depth_samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark depth at Polaris viewpoints")
    parser.add_argument("--viewpoint-config", default=str(PROJECT_ROOT / "config/viewpoints.json"))
    parser.add_argument("--depth-config", default=str(PROJECT_ROOT / "config/depth_anything.json"))
    parser.add_argument("--baseline", default=str(DEFAULT_BASELINE))
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "artifacts/depth_runs"))
    parser.add_argument("--device", choices=("auto", "mps", "cpu"))
    parser.add_argument("--target-fps", type=float)
    parser.add_argument("--sample-duration", type=float, default=3.0)
    parser.add_argument("--system-address", default="udp://:14540")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def print_results(results) -> None:
    print("\nDepth viewpoints ranked by median depth health:")
    print("name                   depth  stable latency   fps nearest      centre visual")
    print("-" * 88)
    for result in sorted(
        results,
        key=lambda item: item.median_depth_health_score,
        reverse=True,
    ):
        print(
            f"{result.name:22s} {result.median_depth_health_score:6.3f} "
            f"{result.median_temporal_stability:6.3f} "
            f"{result.median_depth_latency_ms:7.1f} "
            f"{result.median_depth_fps:5.2f} "
            f"{result.nearest_region:12s} "
            f"{result.centre_risk:6.3f} "
            f"{result.visual.median_visual_health_score:6.3f}"
        )


async def run_live(args, viewpoint_config, depth_config) -> int:
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    baseline = load_visual_baseline(args.baseline)
    drone = System()
    await drone.connect(system_address=args.system_address)
    coordinator = EgoStateCoordinator(
        drone,
        visual_config=VisualHealthConfig.from_json(
            PROJECT_ROOT / "config/visual_health.json"
        ),
        analysis_rate_hz=max(
            viewpoint_config.analysis_rate_hz,
            depth_config.target_fps * 2.0,
        ),
        depth_config=depth_config,
    )
    confidence = ConfidenceEngine(
        ConfidenceConfig.from_json(PROJECT_ROOT / "config/confidence.json")
    )
    connection = ConnectionMonitor(drone)
    connection.start()
    await coordinator.start()
    try:
        await connection.wait_connected()
        started = datetime.now(timezone.utc)
        benchmark = DepthFlightBenchmark(
            drone,
            coordinator,
            confidence,
            connection,
            viewpoint_config,
            depth_config=depth_config,
        )
        results = await benchmark.run_depth(baseline)
        completed = datetime.now(timezone.utc)
        depth = coordinator.state.depth
        run = DepthBenchmarkRun(
            started_at_utc=started.isoformat(),
            completed_at_utc=completed.isoformat(),
            model_identifier=depth_config.model_identifier,
            model_revision=depth_config.model_revision,
            device=depth.device,
            normalization=(
                "normalized_proximity in [0,1]: 1=near/high concern, "
                "0=far/clear; robust per-frame percentile normalization"
            ),
            baseline_artifact=str(Path(args.baseline).resolve()),
            warmup_latency_ms=depth.warmup_latency_ms,
            max_horizontal_drift_m=benchmark.max_horizontal_drift_m,
            viewpoints=results,
        )
        stem = started.strftime("%Y%m%dT%H%M%SZ")
        json_path, csv_path = write_depth_outputs(run, args.output_dir, stem)
        print_results(results)
        print(f"\nMaximum horizontal drift: {run.max_horizontal_drift_m:.3f} m")
        print(f"JSON: {json_path}")
        print(f"CSV : {csv_path}")
        return 0
    finally:
        try:
            await coordinator.stop()
        finally:
            await connection.stop()


def main() -> int:
    args = parse_args()
    viewpoints = BenchmarkConfig.from_json(args.viewpoint_config)
    by_name = {viewpoint.name: viewpoint for viewpoint in viewpoints.viewpoints}
    if set(by_name) != set(SAFE_DEPTH_ORDER):
        raise ValueError("Depth safe-sweep order does not match configured viewpoints")
    viewpoints = replace(
        viewpoints,
        sample_duration_s=args.sample_duration,
        viewpoints=tuple(by_name[name] for name in SAFE_DEPTH_ORDER),
    )
    depth = DepthConfig.from_json(args.depth_config)
    if args.device:
        depth = replace(depth, device=args.device)
    if args.target_fps:
        depth = replace(depth, target_fps=args.target_fps)
    depth.validate()
    print_mapping(viewpoints)
    print(
        f"Depth: {depth.model_identifier}@{depth.model_revision} "
        f"device={depth.device} target={depth.target_fps:.1f} FPS"
    )
    if args.dry_run:
        return 0
    try:
        return asyncio.run(run_live(args, viewpoints, depth))
    except BenchmarkAbort as exc:
        print(f"DEPTH BENCHMARK ABORTED: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Depth benchmark interrupted; landing was requested.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
