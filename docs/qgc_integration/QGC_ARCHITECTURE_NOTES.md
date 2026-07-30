# QGC Architecture Notes

## Current Architecture

PX4 SITL exposes separate GCS and API/Offboard MAVLink instances. Polaris owns
one MAVSDK connection and is the sole application-side Offboard command owner.
The SafetySupervisor remains Polaris's final command gate. QGC is not currently
installed or validated; no custom QGC integration exists.

## Operating Modes

### Mode A: Engineering / Setup

- QGC may be used for firmware, airframe, sensor, actuator, calibration, and
  parameter work only with Polaris autonomy stopped.
- Keep the vehicle disarmed where required.
- This phase did not validate any setup action.

### Mode B: Manual / QGC Flight

- QGC or a pilot owns flight operation.
- Polaris mission processes and all other command-capable MAVSDK applications
  remain stopped.
- No manual-flight procedure is introduced by this document.

### Mode C: Polaris Autonomous Flight

- One `mission_demo.py` process owns MAVSDK and sends the SafetySupervisor's
  SafeMotion to PX4 Offboard.
- QGC may observe telemetry only through the distinct GCS MAVLink path.
- QGC must not send arm, takeoff, land, return, hold, mode, mission, or
  parameter commands.
- VLM remains disabled and has no control authority.

### Mode D: Emergency Intervention

- Current Polaris behavior: on armed mission termination or hard safety abort,
  Polaris performs final hover, stops Offboard where possible, calls PX4 land,
  and waits for confirmed landing and disarm.
- Current QGC emergency-command precedence during simultaneous Polaris control
  has not been validated and must not be invented. Do not command QGC emergency
  actions during a Polaris-owned flight until a written operating policy and
  test prove the interaction.
- Operator emergency policy for real hardware remains an unresolved,
  hardware-dependent safety decision.

## Product Direction Decision

| Option | Time to prototype | PX4 setup/calibration | Safety/maintenance | UI flexibility | Assessment |
|---|---|---|---|---|---|
| Stock QGC plus Polaris dashboard | Lowest | Native QGC | Lowest added risk | Separate UI | Recommended now |
| QGC custom build/plugins | Medium | Retains QGC functions | Upstream update burden | Moderate | Later only for proven operator workflow |
| Full QGC fork | High | Retains functions initially | High merge/test burden | High | Not justified now |
| Ground station from scratch | Highest | Must recreate critical functions | Highest risk | Highest | Not suitable for first product stage |

Recommendation: use stock QGC for engineering/setup and diagnostics, retain
the Polaris Operator View for autonomy-specific status, and validate the
observer-only coexistence path before considering a supported QGC custom build.
No fork decision is justified by the evidence collected so far.

## Validation Update: 2026-07-30

The stock QGC observer role was validated in one no-flight run and one
two-point autonomous run. QGC auto-connected through the GCS endpoint and
showed one vehicle's telemetry, arm, takeoff, motion, landing, and disarm while
Polaris alone owned MAVSDK Offboard. No QGC command was issued. The current
emergency-intervention ownership question remains unresolved: QGC controls must
not be used during Polaris-owned Offboard flight until an explicit policy is
defined and tested.
