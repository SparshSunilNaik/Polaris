# QGC Integration Report

## Executive Summary

Polaris and PX4 expose separate GCS and MAVSDK/Offboard UDP paths. The live
SITL transport map is compatible in principle with QGC monitoring on the GCS
path and Polaris on the onboard path. Stock QGroundControl is not installed on
this Mac, so QGC discovery, telemetry, parameters, UI, and coexistence were not
executed or claimed. No implementation change is evidence-backed.

## Environment

- Host: Apple Silicon macOS, PX4 SITL with Gazebo Harmonic, MAVSDK Python, no ROS.
- Polaris: `/Users/sparshsunilnaik/Documents/Polaris`.
- PX4: `/Users/sparshsunilnaik/Documents/Polaris/PX4-Autopilot`.
- Python: 3.14.6; `pip check` passed.
- Polaris tests: `285 passed, 6 subtests passed in 4.06s`.
- VLM was not started.

## QGC Version and Path

Not installed. `/Applications` and Spotlight returned no `QGroundControl.app`.
No version is available.

## PX4 SITL Connection Method

Simulator command, run from a real interactive terminal or pseudo-terminal:

```sh
cd /Users/sparshsunilnaik/Documents/Polaris/PX4-Autopilot
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
```

Do not use ordinary closed-stdin `nohup`: PX4 then emits repeated `pxh>` prompts
and grows `px4.log`. Runtime logs are in `.polaris-sim/`.

## MAVLink Endpoint Map

See [MAVLINK_ENDPOINT_MAP.md](MAVLINK_ENDPOINT_MAP.md). Live PX4 startup and
socket inspection verified GCS `18570 -> 14550` and API/Offboard
`14580 -> 14540`. The separate Polaris MAVSDK server uses gRPC 50051 locally.

## Stock QGC Functionality and Coexistence

Not verified: application absent. The following remain untested: QGC vehicle
discovery, heartbeat, system/component IDs, vehicle/autopilot type, telemetry,
local position, mode, arm state, battery, parameter count/download, setup,
plan, Analyze, MAVLink Inspector, log transfer, video, QGC + Polaris no-flight,
and QGC + Polaris autonomous flight.

## Flight Ownership Model

See [QGC_ARCHITECTURE_NOTES.md](QGC_ARCHITECTURE_NOTES.md). Polaris owns
Offboard only in Mode C. QGC is monitoring-only in that mode.

## Observed Limitations and Hardware Dependencies

- QGC is missing, blocking all QGC GUI validation.
- PX4 reports the GCS link as localhost-only absent `PX4_NET_INTERFACE`; LAN
  use has not been validated.
- Real hardware flashing, calibration, actuator setup, GPS behavior, battery
  telemetry, video transport, and emergency-command policy require separate
  hardware safety validation.

## Recommended Product Direction

Use stock QGC plus the Polaris dashboard now. Do not fork or rebuild QGC.
Evaluate a supported QGC custom build/plugin only after an operator workflow
requires Polaris-specific data in the QGC UI and a stable protocol exists.

## Exact Next Milestone

Manually install the official stable QGC macOS application, record its version,
then complete an observer-only connection test with Polaris stopped. Only after
that passes, run a Polaris `--no-flight` coexistence test; then one flight with
QGC controls untouched.

## Commands Used

```sh
cd /Users/sparshsunilnaik/Documents/Polaris
./.venv/bin/python -m pytest -q tests

cd /Users/sparshsunilnaik/Documents/Polaris/PX4-Autopilot
./scripts/stop_polaris_sim.sh
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
lsof -nP -iUDP
tail -n 120 .polaris-sim/px4.log
```

## Files Changed

- `docs/qgc_integration/QGC_INTEGRATION_REPORT.md`
- `docs/qgc_integration/MAVLINK_ENDPOINT_MAP.md`
- `docs/qgc_integration/QGC_OPERATOR_GUIDE.md`
- `docs/qgc_integration/QGC_ARCHITECTURE_NOTES.md`
- `docs/qgc_integration/QGC_REQUIREMENTS_DRAFT.md`

QGC INSTALLED: NO
QGC VERSION: N/A
PX4 SITL CONNECTED: YES
TELEMETRY VERIFIED: NO — QGC absent
PARAMETERS VERIFIED: NO — QGC absent
QGC + POLARIS NO-FLIGHT: NOT RUN — QGC absent
QGC + POLARIS FLIGHT: NOT RUN — QGC absent
MISSION ARTIFACT: N/A for this QGC integration phase
PORT CONFLICTS: NONE OBSERVED; QGC not present to test coexistence
FILES CHANGED: five documentation files under docs/qgc_integration/
RECOMMENDED APPROACH: Stock QGC plus separate Polaris dashboard
NEXT MILESTONE: Install stock QGC and validate observer-only connection on PX4 GCS path
BLOCKERS: Stock QGroundControl is not installed

STATUS: PARTIALLY VERIFIED — stock QGroundControl is not installed

---

## Completion Update: 2026-07-30

This update supersedes the earlier installation and coexistence status above.

### Official Release and Installation

- Official upstream release: [QGroundControl v5.0.8 - Stable](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.0.8).
- Release asset: `QGroundControl.dmg`, 333,193,813 bytes upstream-reported.
- Downloaded from the official upstream GitHub release to
  `/tmp/qgroundcontrol-install/QGroundControl-5.0.8.dmg`.
- Computed SHA-256:
  `1f5fc88f331302adcb4d00c3b92f05ef83bad748196a2646497c50800ab74822`.
- This exactly matched the upstream asset digest.
- Installed path: `/Applications/QGroundControl.app`.
- Bundle version: `5.0`; upstream release version: `5.0.8`.
- Bundle ID: `org.qgroundcontrol.QGroundControl`.
- Architecture: universal Mach-O `arm64` and `x86_64`; Rosetta not required.
- Signing: Developer ID Application: Donald Gagne (`BDJKVNW2ZQ`), notarization
  ticket stapled; `spctl` accepted it as `Notarized Developer ID`.
- QGC launched successfully and remained alive with no immediate crash report.

### SITL and QGC Connection

- One Antarctica SITL was started through the checked-in launcher under a
  pseudo-terminal. PX4, Gazebo server, GUI, X500 mono camera, RGB topic, and
  collision topic were present; px4.log was bounded over ten seconds.
- QGC auto-connected without a manual Comm Link.
- Live sockets: QGC `UDP *:14550`; PX4 GCS `UDP *:18570`; PX4 onboard
  `UDP *:14580`; PX4 logged GCS partner `127.0.0.1`.
- No QGC process, MAVSDK process, or socket shared the other client's receive
  port.
- QGC showed exactly one PX4 vehicle with live telemetry. It displayed arm,
  takeoff/movement, landing, and final disarm during Polaris autonomous flight.
- QGC Fly View, Plan View, Vehicle Setup, and MAVLink Inspector were reachable.
  Parameters downloaded successfully; Airframe, Sensors, Actuators, and
  Safety/Setup loaded. No values were changed.
- MAVLink Inspector showed updating messages. Firmware and Video pages were
  not inspected. Direct Gazebo RGB-to-QGC video has not been tested.

### QGC + Polaris Coexistence

No-flight command:

```sh
./.venv/bin/python scripts/mission_demo.py --no-flight --mission two-point-demo --display --no-vlm
```

- Result: `NO_FLIGHT_COMPLETED`; QGC remained one connected vehicle with live
  telemetry and no unexpected mode change.
- Artifact: `artifacts/mission_runs/20260730T174747Z_mission.json`.
- Polaris passed readiness from five distinct healthy depth updates, sent no
  command, created no duplicate MAVSDK server, and recorded no collision.

Flight command:

```sh
./.venv/bin/python scripts/mission_demo.py --mission two-point-demo --display --no-vlm
```

- Result: `COMPLETED`; duration 44.1 s; both waypoints complete.
- Artifact: `artifacts/mission_runs/20260730T174822Z_mission.json`.
- Recovery count 0; avoidance count 0; collision count 0; hard overrides 0.
- Landing and disarm: `CONFIRMED`.
- QGC remained monitoring-only, showed one vehicle throughout, and issued no
  flight command. There was no unexpected PX4 mode change or port conflict.
- Replay command matched the live artifact result:

```sh
PYTHONDONTWRITEBYTECODE=1 ./.venv/bin/python scripts/mission_replay.py \
  artifacts/mission_runs/20260730T174822Z_mission.json
```

### Updated Recommendation and Remaining Work

Use stock QGC v5.0.8 for engineering/setup and monitoring, with the Polaris
Operator View for autonomy-specific state. Do not fork or custom-build QGC now.
The next implementation milestone is to define, then test, a strict emergency
intervention policy before exposing any QGC flight controls during Polaris
autonomy. Firmware/Video UI inspection, log download, and all physical-hardware
validation remain separate work.

QGC INSTALLED: YES
QGC VERSION: v5.0.8 upstream release; bundle CFBundleShortVersionString 5.0
QGC PATH: /Applications/QGroundControl.app
QGC ARCHITECTURE: Universal arm64/x86_64
QGC SHA256: 1f5fc88f331302adcb4d00c3b92f05ef83bad748196a2646497c50800ab74822
SIGNING/GATEKEEPER: Notarized Developer ID accepted; stapled ticket
PX4 SITL CONNECTED: YES
AUTO-CONNECT: YES, QGC UDP 14550
QGC UDP ENDPOINT: UDP *:14550
TELEMETRY VERIFIED: YES
PARAMETERS VERIFIED: YES, download and setup pages loaded read-only
MAVLINK INSPECTOR VERIFIED: YES, updating messages observed
PLAN VIEW VERIFIED: YES, controls visible; no mission uploaded
FIRMWARE PAGE INSPECTED: NO
VIDEO STATUS: NOT INSPECTED; no Gazebo-to-QGC video path validated
QGC + POLARIS NO-FLIGHT: PASS
NO-FLIGHT ARTIFACT: artifacts/mission_runs/20260730T174747Z_mission.json
QGC + POLARIS FLIGHT: PASS
FLIGHT ARTIFACT: artifacts/mission_runs/20260730T174822Z_mission.json
REPLAY: PASS, matched live flight artifact
PORT CONFLICTS: NONE OBSERVED
DUPLICATE VEHICLES: NONE OBSERVED
UNEXPECTED MODE CHANGES: NONE OBSERVED
LANDING: CONFIRMED
DISARM: CONFIRMED
FILES CHANGED: all five docs/qgc_integration files updated
MANUAL USER ACTION REQUIRED: NONE
BLOCKERS: NONE for stock-QGC SITL monitoring compatibility
NEXT MILESTONE: Define and validate emergency-command ownership before allowing QGC flight actions during Polaris autonomy

STATUS: QGC COMPATIBILITY VERIFIED
