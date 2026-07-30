# Control Ownership State Machine

## Current Implementation

```text
DISCONNECTED -> CONNECTED_DISARMED -> POLARIS_ARMING
POLARIS_ARMING -> POLARIS_OFFBOARD_ACTIVE -> LANDING -> LANDED_DISARMED
POLARIS_OFFBOARD_ACTIVE --Safety failure or Ctrl+C--> AUTONOMY_CANCELLED
AUTONOMY_CANCELLED -> LANDING -> LANDED_DISARMED
```

| State | Current command owner | QGC actions | Polaris actions | PX4 mode | Entry / exit |
|---|---|---|---|---|---|
| DISCONNECTED | None | None | None | Unknown | Enter on missing transport; exit after PX4 connection |
| CONNECTED_DISARMED | None | Monitoring only in validated integration | Readiness telemetry only | Disarmed standby | Exit when Polaris arms |
| POLARIS_ARMING | Polaris | Monitoring only | Arm, position setpoint, start Offboard | Transitioning to Offboard | Exit when fresh telemetry reports OFFBOARD |
| POLARIS_OFFBOARD_ACTIVE | Polaris | Monitoring only | 10 Hz SafeMotion through MAVSDK Offboard | OFFBOARD | Exit on normal terminal handling or hard failure |
| AUTONOMY_CANCELLED | Polaris cleanup still acts | No validated QGC override path | Current code begins return/land cleanup | Depends on PX4 | Enter on hard failure/Ctrl+C; exit after landing/disarm |
| LANDING | Polaris during normal/current abort cleanup | Monitoring only | Stop Offboard where possible, action.land(), wait and disarm | PX4-selected land mode | Exit on fresh landed and disarmed telemetry |
| LANDED_DISARMED | None | Setup/manual policy only | Process exits | Disarmed | Terminal |
| FAULT | Undefined | Stop command interaction | Current code attempts landing if armed | Unknown | Requires log review |

## Recommended Future Policy, Not Implemented

```text
POLARIS_OFFBOARD_ACTIVE --valid external command--> QGC_HOLD_OVERRIDE
POLARIS_OFFBOARD_ACTIVE --valid external Land--> QGC_LAND_OVERRIDE
POLARIS_OFFBOARD_ACTIVE --valid external RTL--> QGC_RTL_OVERRIDE
override state -> AUTONOMY_CANCELLED -> explicit operator decision
```

| Recommended state | Owner | Polaris behavior | QGC behavior | Exit / timeout |
|---|---|---|---|---|
| QGC_HOLD_OVERRIDE | QGC/PX4 | Stop transmitting Offboard setpoints; stop mission progression; do not call Land automatically | Hold only, then operator chooses land or a separately authorized resume | Explicit operator action; no automatic resume |
| QGC_LAND_OVERRIDE | QGC/PX4 | Passive telemetry/artifact recording; never countermand Land | Observe controlled descent | Fresh landed/disarmed state |
| QGC_RTL_OVERRIDE | QGC/PX4 | Passive only | Valid only with proven navigation prerequisites | PX4 terminal outcome or explicit operator decision |
| EMERGENCY_STOP | PX4/operator policy | No command competition | Separate guarded action | Hardware-specific recovery |

The recommended model requires an explicit external-override signal and tests.
It must not be inferred from the existing `flight_mode` field alone without
preserving an auditable reason and suppressing terminal countercommands.

The recorder-assisted QGC Hold attempt on 2026-07-30 did not enter an override
state. Its artifact records normal Polaris completion and terminal landing;
the reported QGC Hold observation was after the artifact's landing/disarm.
