# MAVLink Endpoint Map

## Evidence

Validated on 2026-07-30 with one `gz_x500_mono_cam` Antarctica SITL stack.
Sources: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`,
`src/polaris/control/mavsdk_runtime.py`, `scripts/mission_demo.py`, live PX4
startup output, and `lsof`.

| Purpose | Process | Bind IP | Local Port | Remote IP | Remote Port |
|---|---|---:|---:|---:|---:|
| PX4 GCS MAVLink | PX4 | `*` | 18570 | PX4 default peer | 14550 |
| Polaris API/Offboard MAVLink | PX4 | `*` | 14580 | PX4 default peer | 14540 |
| Polaris MAVSDK input | owned `mavsdk_server` | MAVSDK `udp://:14540` receiver | 14540 | PX4 | 14580 |
| Polaris MAVSDK gRPC | owned `mavsdk_server` | `localhost` | 50051/TCP | Polaris mission process | 50051/TCP |
| PX4 onboard payload link | PX4 | `*` | 14280 | PX4 default peer | 14030 |
| PX4 gimbal link | PX4 | `*` | 13030 | PX4 default peer | 13280 |
| Gazebo RGB camera | Gazebo Transport | Gazebo transport-managed | N/A | subscribers | N/A |
| Collision instrumentation | Gazebo Transport | topic `/polaris/world/contacts` | N/A | subscribers | N/A |

`*` is the live `lsof` UDP bind result. It does not establish LAN broadcast.
PX4 startup also prints `MAVLink only on localhost`; its POSIX startup script
only adds `-p` broadcast mode when `PX4_NET_INTERFACE` is configured. No such
configuration was observed for this validation.

## Live Socket Evidence

Live `lsof -nP -iUDP` showed PX4 listening on UDP `18570`, `14580`, `14280`,
and `13030`. PX4 startup reported:

```text
mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
```

## Ownership

- PX4 owns MAVLink instance creation and flight stabilization.
- `mission_demo.py` creates exactly one `OwnedMavsdkSystem`; it starts one
  packaged `mavsdk_server` with MAVSDK system ID 245, component ID 190, and
  gRPC port 50051.
- The mission process connects MAVSDK to `udp://:14540`; PX4's paired onboard
  instance is `14580 -> 14540`.
- QGroundControl must use the distinct PX4 GCS path, not the MAVSDK port.
- QGC was not installed during this validation. Its actual bind port, system
  discovery, and multi-client message distribution remain unverified.

## QGC Connection Prerequisite

After stock QGC is installed, launch it with Polaris stopped and inspect its
active Comm Link. Confirm QGC has bound its configured UDP receive port and
that PX4 reports the expected GCS partner. Do not create a manual link or
change PX4 ports until the QGC UI identifies the selected local port. The PX4
GCS instance's configured remote port is 14550, but QGC behavior has not been
validated on this host.

## QGC Validation Update: 2026-07-30

Stock QGroundControl v5.0.8 was installed and tested. Its process bound
`UDP *:14550`; PX4 logged GCS partner `127.0.0.1` on the normal GCS instance.
QGC auto-connected without a manual Comm Link. The QGC listener, PX4 GCS
listener, PX4 onboard listener, and Polaris MAVSDK server were all distinct.

During QGC + Polaris no-flight and autonomous-flight tests, QGC remained on
UDP 14550 and Polaris used only `udp://:14540` plus local gRPC 50051. No port
conflict, duplicate vehicle, or command-ownership conflict was observed.
