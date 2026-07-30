# Observability Limitations

The mission recorder observes Polaris requests and MAVSDK telemetry. It does
not capture QGC's transmitted MAVLink command, `COMMAND_ACK`, originating
client, or an authoritative operator-action timestamp.

QGC UI mode displays alone are insufficient to attribute an in-flight command.
Correlate a future Hold test with a captured QGC command and ACK, PX4 mode
evidence, and the artifact's timestamped events. The 2026-07-30 recorder-
assisted attempt showed why: the QGC Hold observation was reported after the
artifact had already recorded landing and disarm.
