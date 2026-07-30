# Operator Emergency Policy

## Normal

- Polaris owns Offboard flight, navigation, and terminal landing.
- QGC is telemetry and diagnostics only.
- Do not use QGC arm, takeoff, Hold, mode, Land, RTL, mission, parameter, or
  calibration controls during a Polaris autonomous mission.

## Level 1: Pause / Hold

- Use only after an external-override implementation and SITL test prove that
  Polaris stops setpoints and does not auto-land over the Hold.
- Do not use in the current product behavior: ownership transfer is not
  validated.
- The 2026-07-30 recorder-assisted attempt did not validate Hold because the
  reported QGC observation followed confirmed landing/disarm.
- Future expected result: stable Hold, autonomy marked externally interrupted,
  explicit operator decision required.
- First product version: not enabled during Polaris autonomy.

## Level 2: Land

- Intended use: terminate an otherwise stable flight with controlled descent.
- Do not use through QGC during current Polaris autonomy: the current mission
  cleanup may also issue Land after Offboard loss, so ownership is ambiguous.
- Current supported terminal action: let Polaris SafetySupervisor/mission
  cleanup perform landing after its own abort or normal completion.
- First product version: Polaris-controlled terminal landing only.

## Level 3: RTL

- Intended use: return only where home/global navigation prerequisites are
  verified.
- Do not rely on RTL in the GNSS-denied Antarctic scenario.
- First product version: unavailable as a Polaris-autonomy emergency action.

## Level 4: Emergency Stop / Kill

- Last resort; potentially destructive and distinct from ordinary disarm.
- Not tested in SITL or hardware in this phase.
- Must require separate guarded UI, confirmation, policy, audit logging, and
  hardware validation.

## Mandatory Operator Indication

After any future accepted external override, the operator UI must show the
command source, requested action, PX4 acknowledgment, current flight mode,
autonomy-cancelled status, landing/disarm state, and an audit timestamp.
