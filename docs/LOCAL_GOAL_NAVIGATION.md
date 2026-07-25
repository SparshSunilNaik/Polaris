# Bounded Local Goal Navigation

## Scope

Local goal navigation generates conservative body-frame velocity requests for one nearby simulator-relative NED goal. Every request still passes through `ReactiveAvoidanceController` and `SafetySupervisor`. This is not a global planner, mission planner, SLAM, VIO, semantic system, or real-hardware controller.

Relative pseudo-depth remains non-metric. Metre-valued goal errors come from PX4 local NED telemetry, not from Depth Anything.

## Architecture

```text
LocalGoal
  -> LocalGoalNavigator
  -> DesiredMotion
  -> ReactiveAvoidanceController
  -> SafetySupervisor
  -> SafeMotion
  -> MAVSDK body velocity
  -> PX4 Offboard
```

The demo owns one `OwnedMavsdkSystem`, one `EgoStateCoordinator`, one camera subscriber, one telemetry collector, one DepthWorker, one collision monitor, and one flight controller.

## States

- `IDLE`: no goal.
- `ALIGN`: bounded yaw alignment while translation is held.
- `ADVANCE`: conservative velocity toward the NED goal.
- `AVOIDING`: reactive avoidance currently modifies motion.
- `RECOVER_VIEW`: active perception owns the requested motion.
- `VERIFY_PROGRESS`: verifies post-recovery progress or goal hold.
- `GOAL_REACHED`: position, speed, optional yaw, and hold duration passed.
- `FAILED`: a duration, path, displacement, cross-track, or recovery bound failed.
- `ABORT`: external safety abort.
- `LAND`: final landing state.

## Coordinate conversion

Goal errors are calculated in NED. For PX4 yaw clockwise from north:

```text
body_forward = cos(yaw) * north + sin(yaw) * east
body_right   = -sin(yaw) * north + cos(yaw) * east
```

Goal motion never bypasses avoidance or supervision. Position setpoints are used only for fixed-position takeoff.

## Metrics

- `goal_distance_m`: current horizontal distance to target.
- `path_length_proxy_m`: sum of successive horizontal NED position changes.
- `displacement_from_start_m`: straight-line distance from the goal-run origin.
- `hold_position_error_m`: combined horizontal and vertical goal error.
- `cross_track_error_m`: perpendicular distance from the start-to-goal line.
- `progress_over_window_m`: goal-distance decrease over the progress window.
- `time_without_progress_s`: time since the configured meaningful improvement.

Total displacement is not called drift.

## Default limits

| Limit | Value |
|---|---:|
| Control loop | 10 Hz |
| Cruise speed | 0.35 m/s |
| Approach speed | 0.12 m/s |
| Maximum lateral speed | 0.18 m/s |
| Maximum vertical speed | 0.12 m/s |
| Yaw rate | 15 deg/s |
| Slowdown distance | 2.0 m |
| Horizontal tolerance | 0.50 m |
| Vertical tolerance | 0.35 m |
| Completion speed | 0.12 m/s |
| Completion hold | 1.5 s |
| Goal duration | 60 s |
| Path-length proxy | 15 m |
| Displacement | 13 m |
| Cross-track error | 4 m |
| Progress window | 4 s |
| Meaningful progress | 0.25 m |
| No-progress timeout | 6 s |
| Post-recovery progress timeout | 5 s |
| Maximum expected touchdown speed | 1.0 m/s |
| Maximum expected touchdown depth | 0.01 m |

`collision_hard_override_enabled` and `automatic_landing_on_failure` are required to remain enabled in this simulator milestone.

## Goals

Configured in `config/local_navigation.json`:

- `corridor-staging`
- `pre-obstacle`
- `centred-obstacle`
- `snowfield-view`
- `glare-view`
- `landing-staging`

Offsets are converted to absolute NED coordinates only after fresh post-takeoff local position is available.

## Commands

```sh
./.venv/bin/python scripts/local_goal_demo.py --display
./.venv/bin/python scripts/local_goal_demo.py --goal corridor-staging --display
./.venv/bin/python scripts/local_goal_demo.py --scenario centred-obstacle --display
./.venv/bin/python scripts/local_goal_demo.py --no-flight --no-display
./.venv/bin/python scripts/local_goal_demo.py --dry-run --no-display
```

## Artifact

Artifacts are written to `artifacts/navigation_runs/`. They include configuration snapshots, goal, initial/final position, all distinct metrics, navigation/avoidance/supervisor transitions, recovery candidates and selections, collision evidence, outcome, landing result, speed, command, and loop timing.

No RGB, full pseudo-depth, or video stream is stored.

## Limitations

- One local goal only.
- No persistent map or replanning graph.
- Retreat remains rear-blind.
- No climb escape is enabled.
- Goal completion does not prove obstacle clearance; collision sensing is separate.
- Simulator only and unsafe for real hardware.
