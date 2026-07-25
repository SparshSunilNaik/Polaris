# Polaris Operator View

## Reactive avoidance integration

`scripts/reactive_avoidance_demo.py --display` embeds this compositor in the same process as the single MAVSDK flight controller and shared `EgoStateCoordinator`. When an avoidance decision is supplied, the lower row becomes three panels for visual health, depth health, and reactive-avoidance status. The avoidance panel shows requested and issued body motion, controller and supervisor modes, escape direction, persistence counters, thresholds, reversal count, elapsed time, reasons, and hard overrides.

This integration does not add keyboard flight controls or start a second camera, depth worker, telemetry collector, or MAVSDK client. See `docs/REACTIVE_AVOIDANCE.md` for the controller and supervisor design.

## Local navigation integration

`scripts/local_goal_demo.py --display` adds local-goal, progress, recovery, and collision status without creating another process. The lower navigation panel shows goal ID and NED coordinates, navigator state/result, distance, bearing, progress, path-length proxy, displacement from start, hold/cross-track errors, recovery action and candidates, selected heading, avoidance/supervisor mode, collision count, and contacted entity.

A latched collision always displays a dominant red collision banner. The integrated view remains observation-only; it adds no keyboard flight controls.

## Purpose

The Operator View is a local OpenCV dashboard for demonstrations and debugging. It combines the forward RGB image, relative pseudo-depth, 3x3 proximity risk, vehicle telemetry, perception health, and Confidence Engine output in one observation-only window. It is not a ground-control station and provides no flight controls.

`commands.txt` at the Polaris project root is the canonical quick-reference command file for demonstrations. Update it whenever a user-facing command, flag, or script location changes.

## Architecture

The process creates exactly one MAVSDK `System` and one `EgoStateCoordinator`. The coordinator owns:

- one Gazebo RGB subscription
- one telemetry collector
- one visual-health analyzer
- zero or one `DepthWorker`
- one shared `NavigationState`

The Operator View does not launch or combine existing monitor subprocesses. It reads the coordinator's latest immutable camera frame reference and the worker's lock-protected transient depth map. Full RGB and depth images remain transient unless the operator explicitly saves a screenshot.

Composition runs through `asyncio.to_thread` one frame at a time. There is no display queue. OpenCV Cocoa window and keyboard calls remain on the main thread.

## Run

Start Antarctic SITL:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
```

Launch the unified view:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/operator_view.py
```

Run without loading Depth Anything:

```sh
./.venv/bin/python scripts/operator_view.py --no-depth
```

Useful overrides:

```sh
./.venv/bin/python scripts/operator_view.py \
  --device cpu \
  --target-depth-fps 2 \
  --window-width 1400 \
  --window-height 800 \
  --verbose
```

## Layout

The top half contains forward RGB and relative pseudo-depth. The lower area contains:

- 3x3 risk grid
- NED position and velocity
- altitude, attitude, arm state, battery, and telemetry freshness
- visual-health measurements
- depth-health and spatial measurements
- Confidence Engine mode, subsystem health, reasons, and hard overrides

All state colors include text labels. `EMERGENCY` uses a high-contrast red safety header and is never hidden by depth or image panels.

## Keyboard

| Key | Behavior |
|---|---|
| `q` or Escape | Quit and stop shared pipelines |
| `p` | Pause display rendering only |
| `s` | Save one timestamped composite screenshot |
| `h` | Toggle keyboard help |
| `g` | Toggle 3x3 guide lines over RGB |

Pause does not stop telemetry, RGB ingestion, depth inference, visual analysis, or safety evaluation.

## Interpretation

The RGB panel preserves the decoded OpenCV BGR image and overlays the measured frame rate and age. Its guide lines match the depth regions.

The depth panel is always labelled:

```text
RELATIVE PSEUDO-DEPTH - NOT METRIC DISTANCE
```

Depth risk uses:

```text
0 = relatively far / clear
1 = relatively near / higher concern
```

Blue indicates clearer relative content and red indicates nearer relative content. Numeric values and region names accompany colors, so color is not the sole indication.

## Stale Data

The view reuses stale thresholds from `config/visual_health.json` and `config/depth_anything.json`; it does not define competing safety thresholds.

- Stale RGB remains visible but is dimmed under a red `RGB STALE / DISCONNECTED` warning.
- Stale depth is hidden and replaced by `DEPTH STALE - MAP HIDDEN`.
- Missing depth shows a loading or unavailable placeholder.
- Safety reasons and hard overrides continue to come directly from the Confidence Engine.

## Screenshots

Press `s` to save one composite PNG under:

```text
artifacts/operator_screenshots/<timestamp>_operator.png
```

`--save-screenshots` saves one screenshot automatically after all enabled panels become live. No continuous recording is implemented.

## Demonstrations

Keep the simulator terminal running, then launch the Operator View in a second terminal. No third terminal or standalone MAVSDK process is required for the observation demo.

Each current Python application starts its own MAVSDK server. Avoid running another MAVSDK-based flight or monitor script against the same UDP endpoint while the Operator View is active. Quit the view before running `src/first_flight.py`, `src/offboard_movement.py`, or a viewpoint benchmark.

## Limitations

- OpenCV text uses the built-in Hershey font rather than native macOS typography.
- Pseudo-depth is relative and cannot report metres or physical clearance.
- The display is optimized for approximately 1600x900 and enforces a 1000x650 minimum.
- The dashboard does not issue flight commands.
- Window-close detection depends on OpenCV Cocoa's `WND_PROP_VISIBLE` support; `q`, Escape, and Ctrl+C remain reliable.
