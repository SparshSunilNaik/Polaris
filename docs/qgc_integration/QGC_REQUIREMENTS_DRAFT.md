# QGC Requirements Draft

Status values: `Stock QGC` means expected stock capability pending host
validation; `Polaris integration` means current Polaris data is not exposed to
QGC; `Custom MAVLink` means an explicit message/protocol decision would be
needed.

## A. Must Have: First Useful Prototype

| Feature | Delivery path | Status |
|---|---|---|
| Vehicle connection/status | Stock QGC | Not validated: QGC absent |
| Telemetry, attitude, battery, link health, flight mode, arm state | Stock QGC | Not validated: QGC absent |
| Polaris autonomy status | Polaris integration | Current Operator View only |
| Mission progress | Polaris integration | Current JSON mission/artifact only |
| Landing/disarm status | Stock QGC plus Polaris integration | QGC unvalidated; Polaris validated separately |
| Logs | Stock QGC and Polaris artifacts | QGC unvalidated; Polaris artifacts available |

## B. Engineering Features

| Feature | Delivery path | Status |
|---|---|---|
| Firmware flashing | Stock QGC | Real hardware validation required |
| Airframe setup | Stock QGC | Real hardware validation required |
| Calibration | Stock QGC | Real hardware validation required |
| Actuator setup | Stock QGC | Real hardware validation required |
| Parameter editor | Stock QGC | Not validated: QGC absent |
| MAVLink Inspector | Stock QGC | Not validated: QGC absent |
| Log download | Stock QGC | Not validated; hardware/link dependent |

## C. Operator Features

| Feature | Delivery path | Status |
|---|---|---|
| Map | Stock QGC | Not validated: QGC absent |
| Video | Stock QGC configuration plus streaming source | Requires Polaris/video integration and hardware validation |
| Mission/waypoint progress | Polaris integration | Current Operator View only |
| Safety/avoidance state | Polaris integration | Current Operator View only |
| Emergency actions | Stock QGC commands plus policy | Not yet decided; must be safety-tested |

## D. Future Features

| Feature | Delivery path | Status |
|---|---|---|
| Mission authoring | Stock QGC or Polaris integration | Not yet decided; current mission JSON is separate |
| Multi-drone | Custom MAVLink and Polaris integration | Not yet decided |
| Custom MAVLink messages | QGC custom build/plugins plus protocol | Not yet decided |
| Polaris configuration UI | QGC custom build/plugins or separate dashboard | Not yet decided |
| Remote updates | Separate product service | Requires real hardware validation |
| User roles/audit logs | Separate product service | Not yet decided |

No requirement here authorizes QGC to command a vehicle while Polaris owns
Offboard control.

## Validation Update: 2026-07-30

Verified in stock QGC v5.0.8 on this SITL: vehicle connection/status, live
telemetry/attitude/position/mode/arm state, parameter download, Airframe,
Sensors, Actuators, Safety/Setup pages, Plan View loading, and MAVLink
Inspector availability with updating messages. QGC showed arm, takeoff,
movement, landing, and disarm while Polaris retained Offboard ownership.

Not yet inspected: Firmware page, Video settings, QGC log download, and QGC
video transport. All firmware flashing, calibration, actuator configuration,
and physical-hardware claims remain hardware-dependent and unvalidated.
