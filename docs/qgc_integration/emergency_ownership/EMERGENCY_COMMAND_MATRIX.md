# Emergency Command Matrix

## Evidence Basis

This matrix combines static Polaris/PX4 inspection with the 2026-07-30 QGC
SITL session. A QGC Hold UI action was attempted, but no mode transition or
command acknowledgment was captured. It is therefore not evidence that Hold
overrides Polaris.

| Command | Intended use | Risk level | Allowed during autonomy? | Expected PX4 response | Expected Polaris response | Test decision |
|---|---|---|---|---|---|---|
| Pause / Hold | Arrest motion for operator assessment | Medium | Policy unresolved | Must be observed with QGC command/ACK and flight-mode transition | Static path would hard-abort on non-OFFBOARD then run terminal landing cleanup | TEST WITH RESTRICTIONS; two attempts inconclusive, latest observation was after landing |
| Land | Controlled terminal descent | Medium | Not until Hold ownership is proven | PX4 Land mode/land command acceptance | Current static cleanup also issues Land after Offboard loss | DO NOT TEST in this phase |
| Return / RTL | Return to valid home | High in GNSS-denied scenario | No | Depends on valid global/home state | Static path would hard-abort on mode loss and start landing cleanup | DO NOT TEST; navigation prerequisites unverified |
| Mode switch to Position | Exit Offboard into assisted hold | High | No | Requires estimator validity; must be observed | Static path treats non-OFFBOARD as hard abort then lands | DO NOT TEST |
| Mode switch to Altitude | Exit Offboard into altitude-assisted control | High | No | Requires valid altitude estimate | Same static path as above | DO NOT TEST |
| Manual / Stabilized | Pilot-input-dependent control | Critical | No | Requires a manual-control source | No supported Polaris handoff | DO NOT TEST |
| Arm | Arm before flight | High | No | PX4 arming checks | Polaris owns arming in autonomous mode | DO NOT TEST |
| Disarm while landed | Confirm a terminal vehicle state | Low | Only after landing and Polaris exit | PX4 accepts/rejects based on state | Polaris already requests disarm after its own landing | POLICY UNRESOLVED; not separately tested |
| Disarm while airborne | Remove motor authority | Critical | No | PX4 rejection/confirmation is configuration dependent | No safe Polaris handoff | DO NOT TEST |
| Mission upload/start | Start a QGC MAVLink mission | High | No | Changes PX4 mission execution | Conflicts with JSON mission orchestration | DO NOT TEST |
| Takeoff | Initiate flight | High | No | PX4 takeoff | Conflicts with Polaris takeoff path | DO NOT TEST |
| Emergency Stop / Kill | Last-resort termination | Critical | No | Potentially destructive | No current policy or artifact semantics | DO NOT TEST; HARDWARE-DEPENDENT |

## Current Implementation Constraint

`TelemetryCollector` records `flight_mode` but does not publish an
autonomy-cancellation event. `SafetySupervisor.evaluate()` treats any armed or
airborne `flight_mode != OFFBOARD` while Offboard is expected as an immediate
hard safety condition. `MissionDemo` then follows its normal `finally` path:
terminal hover attempt, `offboard.stop()`, `action.land()`, and disarm wait.

`offboard.start()` is called only during takeoff; no automatic Offboard re-entry
path was found. However, current code also does not preserve external Hold,
Land, or RTL ownership. No code was changed because this limitation has not
been reproduced with a verifiable PX4 mode transition.

The recorder-assisted attempt in
`artifacts/mission_runs/20260730T183130Z_mission.json` is not such a
reproduction: it records a normal Polaris terminal `OFFBOARD -> LAND` after
both waypoints completed, while the reported QGC Hold observation was after
landing/disarm.
