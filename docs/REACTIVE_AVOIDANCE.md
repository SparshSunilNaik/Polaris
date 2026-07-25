# Reactive Obstacle Avoidance

## Scope and warning

This milestone is deterministic reactive local avoidance for PX4 SITL only. It is not a global planner, SLAM system, VIO system, semantic detector, learned policy, or autonomous mission planner.

**The Depth Anything output is relative pseudo-depth. It is not metric distance.** Risk thresholds compare persistent normalized scene-relative proximity. They must not be interpreted as metres or used for real-hardware separation guarantees.

This system is not safe for real hardware. It has not been validated against physical sensor timing, actuator faults, wind, unmodelled dynamics, real collision geometry, or a certified safety process.

## Architecture

```text
DesiredMotion (body frame)
        |
        v
ReactiveAvoidanceController
        |
        v
SafetySupervisor
        |
        v
SafeMotion
        |
        v
MAVSDK VelocityBodyYawspeed -> PX4 Offboard

Gazebo RGB + PX4 telemetry
        |
        v
EgoStateCoordinator
        |
        +-- one camera subscriber
        +-- one DepthWorker
        +-- one telemetry collector
        |
        v
NavigationState + ConfidenceReport
```

`scripts/reactive_avoidance_demo.py` creates one MAVSDK `System` and one `EgoStateCoordinator`. The integrated display reads the same latest state and atomic depth snapshot. It does not start another MAVSDK client, camera subscriber, worker, or flight controller.

The control loop is explicit at 10 Hz. Full RGB and depth arrays remain transient. JSON artifacts contain only compressed state, risk, commands, timing, and transitions.

## Body-frame motion

The forward camera aligns naturally with MAVSDK `VelocityBodyYawspeed`:

- Positive `forward_m_s`: camera-forward motion.
- Positive `right_m_s`: body-right sidestep.
- Positive `down_m_s`: downward motion in NED convention.
- Positive `yaw_rate_deg_s`: body yaw rate.

Avoidance uses sidestep-only or retreat-only commands. It does not combine aggressive yaw, climb, and lateral motion. Zero-velocity hover commands are transmitted continuously before landing.

## 3x3 interpretation

```text
UP_LEFT      UP       UP_RIGHT
LEFT         CENTRE   RIGHT
DOWN_LEFT    DOWN     DOWN_RIGHT
```

The controller computes:

- Centre from `CENTRE`.
- Left from the maximum of `UP_LEFT` and `LEFT`.
- Right from the maximum of `UP_RIGHT` and `RIGHT`.
- Upper from the upper row.
- Lower from the lower row for reporting only.

The lower row frequently sees nearby ground during level flight. High lower-row risk alone therefore does not trigger frontal avoidance. It remains available for future attitude-aware ground and landing logic.

## Avoidance state machine

- `CLEAR`: pass the bounded requested command.
- `CAUTION`: cap forward velocity when risk rises, depth stability drops, or visual condition degrades.
- `AVOID_LEFT`: stop forward motion and sidestep left.
- `AVOID_RIGHT`: stop forward motion and sidestep right.
- `HOVER`: hold zero velocity while confirming no safe path or perception quality is insufficient.
- `RETREAT`: command slow reverse motion after persistent centre-and-both-side blockage.
- `ABORT`: zero velocity, terminate the run, and land.
- `LAND`: reserved typed state for final landing supervision.

Only new depth source sequences increment blocked and clear counters. One-frame obstacles cannot trigger a sidestep. An active escape direction is held for at least 2 seconds, and forward motion resumes only after three clear depth updates. Direction changes increment a reversal counter. Excess reversals or more than 8 seconds in one avoidance episode abort the run.

No climb mode is issued in this first version. When both sides are blocked, the simpler conservative policy hovers for confirmation and retreats rather than assuming that a clear upper image implies safe metric vertical clearance.

## Safety Supervisor

Every controller proposal passes through `SafetySupervisor.evaluate()` before MAVSDK receives it. The supervisor checks:

- Confidence mode and hard overrides.
- PX4 connection and reported Offboard mode.
- Local-position health and freshness.
- RGB availability and freshness.
- Required-depth availability, freshness, condition, and source age.
- Ego State age and control-loop deadline.
- Finite command and 3x3 values.
- Armed/in-air state.
- Measured horizontal speed and drift.
- Autonomy enable state.
- Controller mode, risk summary, and selected escape command.

An airborne critical failure immediately outputs a zero-velocity hard override with `ABORT` and a landing recommendation. The runtime continues hover commands for the configured failure interval, stops Offboard when possible, commands PX4 landing, and waits for `in_air` to clear. PX4 safety checks are not modified.

The only narrow exception is perception-derived centre risk while bounded active perception already owns a zero-translation command. The first fresh emergency observation enters `SUSPECTED`: translation remains zero while the already-authorized yaw command may continue. A second distinct, newer depth inference confirms the emergency when the short-window variance is within its configured bound. If evidence does not clear or confirm within 0.8 seconds, confirmation fails closed to `CONFIRMED` and the existing abort/landing path. Ordinary motion, collision, stale perception, localization failure, connection loss, Confidence Engine emergency, Offboard loss, invalid state, and operator interruption remain immediate.

## Default limits

| Setting | Default |
|---|---:|
| Control loop | 10 Hz |
| Requested forward speed | 0.40 m/s |
| Cautious speed | 0.16 m/s |
| Sidestep speed | 0.25 m/s |
| Retreat speed | 0.20 m/s |
| Configured climb bound | 0.18 m/s |
| Yaw-rate bound | 15 deg/s |
| Maximum horizontal command | 0.60 m/s |
| Clear threshold | 0.35 relative risk |
| Caution threshold | 0.45 relative risk |
| Stop threshold | 0.62 relative risk |
| Emergency threshold | 0.90 relative risk |
| Active-perception emergency confirmation | 2 fresh depth frames |
| Emergency confirmation deadline | 0.8 s, fail closed |
| Emergency short-window variance | 0.0016 maximum |
| Blocked confirmation | 2 new depth updates |
| Clear confirmation | 3 new depth updates |
| No-safe-path confirmation | 3 new depth updates |
| Direction hold | 2.0 s |
| Maximum avoidance episode | 8.0 s |
| Maximum reversals | 2 |
| Maximum run duration | 60.0 s |
| Maximum displacement from start | 14.0 m |
| Default travel proxy limit | 12.0 m |

All settings are in `config/reactive_avoidance.json`.

The configuration snapshot in every new avoidance and navigation artifact records these exact numerical values. When no horizontal motion is requested for goal hold or yaw alignment, non-emergency side risk does not invent a sidestep. Emergency centre risk remains immediate during ordinary motion; only stationary active-perception ownership uses the bounded confirmation described above. Every non-perception hard safety override remains immediate.

## Commands

Start the Antarctic simulator:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
```

Configuration validation and synthetic scenario:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/reactive_avoidance_demo.py --dry-run --no-display
./.venv/bin/python scripts/reactive_avoidance_demo.py --scenario synthetic-grid-test --no-display
```

Live readiness without arming:

```sh
./.venv/bin/python scripts/reactive_avoidance_demo.py --no-flight --duration 10 --display
```

Default bounded corridor run with integrated Operator View:

```sh
./.venv/bin/python scripts/reactive_avoidance_demo.py --scenario corridor --display
```

Airborne injected stale-depth safety validation:

```sh
./.venv/bin/python scripts/reactive_avoidance_demo.py --scenario stale-depth-test --duration 10 --display
```

Airborne injected low-visual-health policy validation:

```sh
./.venv/bin/python scripts/reactive_avoidance_demo.py --scenario low-visual-health-test --duration 8 --display
```

Both safety scenarios inject compressed state conditions; they do not modify or store RGB/depth frames. The low-visual test validates policy response and must not be represented as a measured snow/glare run.

Stop the simulator only after the demo has landed and exited:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
./scripts/stop_polaris_sim.sh
```

## Integrated Operator View

`--display` uses the existing OpenCV compositor in the same process. Its reactive-avoidance panel displays:

- Autonomy enable state.
- Controller and supervisor modes.
- Requested and issued body motion.
- Escape direction.
- Blocked and clear counters.
- Reversal count.
- Caution, stop, and emergency thresholds.
- Hard-override state.
- Avoidance elapsed time and reason.

There are no keyboard flight controls. `q` or Escape requests a safe stop, `s` saves an explicit screenshot, and `g` toggles RGB grid guides.

## Artifacts

Each completed run writes `artifacts/avoidance_runs/<timestamp>_avoidance.json` containing:

- Configuration and UTC start/end times.
- Scenario and takeoff status.
- Requested, proposed, and issued commands.
- Supervised commands even in no-flight mode, plus whether each command was actually transmitted.
- Compressed 3x3 risk grids and summaries.
- Supervisor and Confidence Engine transitions.
- Visual and depth health.
- NED position, velocity, speed, drift, and loop timing.
- Maximum observed risks, speed, drift, and reversal count.
- Forward-reduction and avoidance-command flags.
- Abort reason, landing result, collision instrumentation status, and final result.

No RGB image, full pseudo-depth frame, or video is stored.

## Validation interpretation

Synthetic tests cover centred obstacles, left/right selection, both-side blockage, lower-row rejection, persistence, hysteresis, direction hold, reversal limits, invalid grids, and hard safety overrides.

The live Antarctic corridor contains an offset obstacle. A valid live run may therefore trigger a side-sector avoidance while centre risk remains below the stop threshold. This is not evidence of successful centred-obstacle avoidance. Collision status is recorded as `NOT_INSTRUMENTED`; completion alone must never be presented as proof that a collision did not occur.

## Limitations

- Relative pseudo-depth does not provide physical clearance.
- No collision/contact sensor is currently logged.
- No attitude-aware lower-grid interpretation is implemented.
- Side-sector corridor walls can conservatively trigger sidesteps.
- The controller has no local map or bounded goal-position recovery.
- It does not reason about moving obstacles.
- It does not climb in response to upper-sector clearance.
- Independent MAVSDK applications should not run concurrently because UDP bind warnings can occur.
- `automatic_landing_on_abort` cannot suppress the demo's mandatory landing after an armed run; hard safety behavior takes precedence.
- Only simulator operation is allowed.
