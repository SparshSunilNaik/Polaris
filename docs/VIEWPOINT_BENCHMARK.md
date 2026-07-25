# Deterministic Viewpoint Benchmark

## Purpose

The viewpoint benchmark flies the X500 to a fixed 2.5 m hover near the Antarctic takeoff marker, rotates through named headings, collects fresh visual-health samples, and writes robust JSON/CSV summaries. It uses one MAVSDK `System`, one `EgoStateCoordinator`, and the coordinator's existing camera subscription. No image arrays enter benchmark artifacts.

## Yaw Convention

Live validation showed the camera pointing along Gazebo `+X` while PX4 reported approximately `+90 deg`. Gazebo ENU yaw is counter-clockwise from east; PX4 NED yaw is clockwise from north:

```text
PX4 yaw = 90 deg - Gazebo yaw
```

The configured mapping is:

| Viewpoint | Gazebo ENU yaw | Commanded PX4 NED yaw |
|---|---:|---:|
| `east_corridor` | 0 | 90 |
| `northeast_rocks` | 45 | 45 |
| `north_rocks` | 90 | 0 |
| `northwest_glare` | 135 | -45 |
| `west_snowfield` | 180 | -90 |
| `southwest_snow` | -135 | -135 |
| `south_station` | -90 | 180 |
| `southeast_corridor` | -45 | 135 |

Both values are retained in configuration and output to prevent frame-convention ambiguity.

The Antarctic-style world retains PX4's default geodetic and magnetic reference so estimator heading remains stable during this conversion.

## Running

Terminal 1:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
```

Terminal 2:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/viewpoint_benchmark.py
```

Validate configuration without connecting:

```sh
./.venv/bin/python scripts/viewpoint_benchmark.py --dry-run
```

Validate live PX4, local position, and RGB readiness without arming:

```sh
./.venv/bin/python scripts/viewpoint_benchmark.py --no-flight
```

Useful overrides:

```sh
./.venv/bin/python scripts/viewpoint_benchmark.py \
  --sample-duration 3 \
  --settle-duration 2 \
  --output-dir artifacts/viewpoint_runs \
  --json-only
```

## Flight Behavior

1. Wait for PX4 connection, armability, local position, and fresh camera data.
2. Capture the ground NED reference and start an Offboard `PositionNedYaw` climb to 2.5 m at exactly that horizontal target.
3. Keep the same fixed NED position for the complete run and abort if horizontal drift exceeds 1.5 m, remaining well inside the 4 m takeoff-zone radius.
4. Step each yaw setpoint along the shortest rotation at a configured 20 deg/s.
5. Require measured yaw to remain within 5 deg for five checks.
6. Hold for 2 seconds, then gather unique fresh samples for 2.5 seconds.
7. Stop Offboard and land.

The runner attempts landing in `finally`, including benchmark aborts and Ctrl+C cancellation.

## Safety Aborts

The flight aborts and requests landing if any of these occur in air:

- PX4 disconnects
- local position is unavailable or stale
- camera is unavailable or stale
- Confidence Engine reaches `EMERGENCY`

Stale visual states are never added to the sample set. A minimum number of unique visual updates is required for every viewpoint.

## Statistics and Artifacts

Continuous metrics use medians. Visual-health output additionally records the worst score. Visual condition uses the most frequent condition, choosing the more severe condition on a tie. Reasons are accumulated as a unique sorted set. Measured yaw is aggregated relative to the commanded heading so wrapping around `+/-180 deg` remains valid.

Outputs are timestamped:

```text
artifacts/viewpoint_runs/<timestamp>_viewpoints.json
artifacts/viewpoint_runs/<timestamp>_viewpoints.csv
```

The JSON includes schema version, UTC start/completion timestamps, hover reference, maximum horizontal drift, yaw convention, and all viewpoint summaries. CSV contains one flattened row per viewpoint for later pseudo-depth comparisons.

## Limitations

- The benchmark measures visual health, not depth accuracy.
- PX4 position and yaw controller tuning determines actual settling time.
- No wind or moving illumination is used in the current deterministic world.
- A single run does not establish statistical repeatability; preserve artifacts from repeated runs for comparison.
