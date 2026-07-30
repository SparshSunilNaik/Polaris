# QGC Operator Guide

## Validation Status

Stock QGroundControl was not found in `/Applications` or Spotlight on
2026-07-30. The following is an operator checklist, not a claim that these QGC
screens have been validated against this SITL.

## Installation Gate

Official stable source: <https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html>

Official macOS steps:

1. Download the stable `QGroundControl.dmg` from the stable official QGC page.
2. Open the DMG and drag `QGroundControl.app` into `/Applications`.
3. If macOS blocks the unsigned application, grant permission through the
   normal macOS security prompt. Do not bypass platform security controls by
   changing global networking or downloading an unverified binary.
4. Record the installed application path and QGC version before connecting.

Do not install a daily build, build from source, flash firmware, upload a plan,
change parameters, arm, or take off as part of first connection.

## Screen Checklist

| Screen | Purpose | Current SITL status | Polaris need | Product role | Safety note |
|---|---|---|---|---|---|
| Fly View | Vehicle telemetry, mode, map, action controls | Not verified: QGC absent | Monitoring only | Operator | Do not press flight actions in Polaris mode |
| Plan View | Create/upload MAVLink missions | Not verified | Not needed for current JSON mission | Engineering/future operator | Do not upload while Polaris owns mission |
| Vehicle Setup | PX4 configuration and calibration | Not verified | Not needed for validated SITL | Engineering | Disarmed, Polaris stopped |
| Application Settings | Comm Links, UI, video, logging settings | Not verified | Required to inspect QGC link | Engineering | Record settings; do not guess ports |
| Analyze / MAVLink Inspector | Inspect heartbeat, telemetry, IDs | Not verified | Useful diagnostic | Engineering | Read-only observation |
| Analyze / Log Download | Download PX4 ULog | Not verified | Optional evidence | Engineering | May require link support and real hardware validation |
| Parameter Editor | Read/write PX4 parameters | Not verified | Read-only diagnostic initially | Engineering | Never write during Polaris mode |
| Firmware | Flash PX4 firmware | Not verified | Not needed | Engineering | Never flash SITL/hardware in this phase |
| Video Settings | Configure QGC video input | Not verified | No current feed: Polaris uses direct Gazebo RGB | Future operator | Requires explicit streaming integration/hardware validation |

## First Safe Connection Procedure

1. Start one PX4/Gazebo stack; do not start Polaris.
2. Start stock QGC and wait for auto-discovery.
3. If no vehicle appears, inspect QGC Application Settings > Comm Links and
   record its configured UDP receive port. Only then compare it to the PX4 GCS
   endpoint map.
4. Verify one vehicle, heartbeat, attitude, local/global position where
   available, flight mode, armed state, parameter download, and MAVLink
   Inspector data.
5. Treat absent GPS/global-position warnings as potentially expected in this
   GNSS-denied simulated scenario. Record them; do not suppress them.
6. Do not flash, upload, arm, take off, land, return, pause, or change modes.

## Polaris Autonomous Mode

During a Polaris mission, QGC is telemetry and diagnosis only. Do not use arm,
takeoff, land, pause, return, mission, mode, parameter, or calibration controls.
Only observe vehicle state, mode, link health, and telemetry.

## Validated Launch Order

1. In Terminal 1, start one interactive or pseudo-terminal simulator stack:

   ```sh
   cd /Users/sparshsunilnaik/Documents/Polaris/PX4-Autopilot
   POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
   ```

2. Start stock QGC with `open -a QGroundControl` and wait for one connected
   vehicle. QGC v5.0.8 auto-connected on its UDP 14550 listener in this SITL.
3. For Polaris autonomy, start exactly one mission process with `--no-vlm`.
4. Use QGC Fly View and MAVLink Inspector only as monitoring surfaces. The
   validated visual indicators are connection, arm state, takeoff/movement,
   mode, landing, and final disarm.
5. Wait for Polaris to print confirmed landing and disarm, then close QGC
   normally and stop the simulator with `./scripts/stop_polaris_sim.sh`.

Validated stock-QGC screens: Fly View telemetry, Plan View controls, parameter
download, Airframe, Sensors, Actuators, Safety/Setup, and MAVLink Inspector.
Firmware and Video pages were not inspected. The Gazebo RGB camera has not been
validated as a QGC video source; Polaris consumes it directly via Gazebo
Transport.
