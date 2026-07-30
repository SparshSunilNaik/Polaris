# Emergency Command Ownership Test Report

## Executive Summary

The normal QGC-observed Polaris mission remains safe and repeatable. One QGC
Hold/Pause UI action was attempted in isolated SITL, but did not produce a
verifiable PX4 mode transition, QGC command/ACK capture, or Offboard-loss
artifact event. The run must be classified as inconclusive, not as a successful
override test. No higher-risk QGC commands were executed.

## Current Control Architecture

- PX4 GCS link: UDP `18570 -> 14550`; QGC listens on UDP 14550.
- PX4 onboard link: UDP `14580 -> 14540`; one Polaris-owned MAVSDK server uses
  that path and local gRPC 50051.
- Polaris enters Offboard in `ReactiveAvoidanceDemo._takeoff()` after arming and
  an initial position setpoint. It sends velocity SafeMotion through
  `offboard.set_velocity_body()` at the 10 Hz control rate.
- `TelemetryCollector` updates flight mode, armed, in-air, local/global/home
  health, position, attitude, and battery state.
- `SafetySupervisor` treats flight mode other than `OFFBOARD` while armed or
  airborne and Offboard is expected as a hard abort.
- `MissionDemo.run()` has a `finally` path that begins return/land, attempts
  shielded hover/land cleanup, waits for fresh landing telemetry, then disarm.
- No automatic `offboard.start()` re-entry exists after takeoff.

## Baseline

Precondition: one clean SITL, one QGC process monitoring only, one Polaris
mission process, VLM disabled.

Command:

```sh
./.venv/bin/python scripts/mission_demo.py --mission two-point-demo --display --no-vlm
```

Observed outcome: PASS. Both waypoints completed; duration 46.0 s; recovery 0;
avoidance 0; collision 0; hard overrides 0; landing/disarm confirmed. QGC
remained connected and monitoring-only.

Artifact: `artifacts/mission_runs/20260730T180403Z_mission.json`.

## Hold / Pause Test

### Preconditions

- Fresh SITL, QGC auto-connected on UDP 14550, no Polaris process beforehand.
- Camera/collision transport available; no collision/recovery active at takeoff.
- Polaris launched with existing `--start-delay 30` to create a post-takeoff
  hover window. This did not alter mission configuration or safety thresholds.

### Command

QGC Hold/Pause UI action issued once during the hover window. No other QGC
action was issued.

### Expected Outcome and Abort Conditions

The test expected a visible PX4 mode transition and QGC command/ACK evidence.
It would stop on instability, oscillation, unexpected altitude change,
collision, log growth, duplicate vehicle, or ambiguous ownership.

### Observed Outcome

- User observed QGC "Hold then landing," but QGC MAVLink Inspector command/ACK
  details were not captured.
- Polaris artifact contains no Offboard-loss abort, external mode transition, or
  external-command field.
- PX4 log contains no Hold/mode-command acknowledgment.
- Instead, Polaris completed the 30-second delay, attempted normal navigation,
  then eventually aborted for `Local goal timeout exceeded` and performed its
  normal controlled landing/disarm cleanup.
- Therefore the QGC Hold UI action did not produce evidence of an accepted PX4
  ownership transfer. The test is INCONCLUSIVE.

### Recovery

Polaris completed its existing terminal path. Landing and disarm were
confirmed. No follow-up emergency command was issued.

Artifact: `artifacts/mission_runs/20260730T180618Z_mission.json`.

## Land, RTL, Mode Change, and Disarm

Not executed. The Hold ownership transfer was unproven, and the risk matrix
defaults uncertain commands to DO NOT TEST. RTL is additionally unsuitable for
assumption-based testing in the GNSS-denied scenario. Airborne disarm and kill
remain explicitly untested.

## 2026-07-30 Recorder-Assisted Hold Attempt

This attempt used one clean SITL stack, one QGC process, one Polaris process,
and one MAVSDK server. QGC was listening on UDP 14550. Polaris produced
`artifacts/mission_runs/20260730T183130Z_mission.json`.

- The artifact shows normal Offboard entry, both waypoints completed, terminal
  Polaris `OFFBOARD -> LAND`, landing, and disarm. It has no Hold transition,
  `MISSION_EXTERNALLY_INTERRUPTED`, or unexpected Offboard exit.
- PX4's bounded console log has only the Polaris arm, takeoff, land, and disarm
  messages; it has no Hold/commander mode-change entry.
- The operator observed QGC showing `Offboard -> Hold` at approximately
  `2026-07-30T18:34:00Z`, but could not capture command, target, or ACK fields.
- The mission console and artifact had already completed terminal landing and
  disarm at approximately `18:32Z`. The reported QGC observation is therefore
  outside the airborne mission interval and cannot evidence an airborne Hold.

Classification: INCONCLUSIVE. The vehicle was stable and landed/disarmed, but
the action timing, QGC command/ACK, PX4 mode evidence, and recorder evidence
do not establish ownership transfer.

## Observed MAVLink Commands and ACKs

- Normal baseline: PX4/MAVSDK logged arm, takeoff, land, and disarm events.
- QGC Hold: no command identifier or COMMAND_ACK was captured. Do not infer a
  MAVLink command from the QGC button label.

## Ownership Violations and Code Changes

No runtime ownership violation was proven because no external transition was
observed. Static analysis identifies a potential policy mismatch: a proven
non-Offboard external override would trigger the current Polaris hard-abort
landing cleanup rather than passive external-command ownership. No code change
was made to ownership policy because the precondition for evidence-backed
modification was not met.

A lightweight mission event recorder was subsequently added. It records
Polaris lifecycle requests and observed PX4 telemetry transitions in each new
artifact, including a clearly classified observed transition out of Offboard
when it was not initiated by Polaris. It does not capture MAVLink command
source or COMMAND_ACK, so it cannot by itself prove QGC command ownership.

## Test Results and Regression

- Existing Polaris suite before testing: `285 passed, 6 subtests passed`.
- Recorder validation: `292 passed, 6 subtests passed`; a no-flight artifact
  and a controlled terminal landing/disarm run validated ordered event output.
- The controlled run aborted on the existing local-goal timeout, so it is not a
  replacement for the successful normal-autonomy baseline.
- Baseline normal autonomy: PASS.
- Hold test landing/disarm: PASS, but command ownership: INCONCLUSIVE.
- Recorder-assisted Hold attempt: stable terminal result, but invalid as an
  airborne ownership test because the reported Hold observation followed the
  artifact's confirmed landing/disarm.

## Recommended Emergency Hierarchy

Follow [OPERATOR_EMERGENCY_POLICY.md](OPERATOR_EMERGENCY_POLICY.md): Polaris
owns normal autonomy. Hold, QGC Land, and RTL remain disabled/not used during
Polaris autonomy until command capture and policy-preserving cancellation are
implemented and validated. Kill remains a separate, hardware-dependent last
resort.

## Hardware Validation Still Required

All external emergency-command authority, radio priority, airborne disarm/kill,
RTL without GNSS, vehicle motion stability, and physical operator procedures.

## Exact Next Milestone

Create a non-destructive, observable QGC command-capture procedure that records
the selected QGC action, MAVLink Inspector command/ACK, PX4 mode transition,
and Polaris event timeline in one timestamped test. Only if it proves an
external override does Polaris fight should a minimal cancellation change be
designed.
