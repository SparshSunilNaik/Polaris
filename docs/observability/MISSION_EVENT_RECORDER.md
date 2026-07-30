# Mission Event Recorder

Mission artifacts may include an optional top-level `events` array. It is an
append-only, ordered account of Polaris lifecycle and observed PX4 telemetry
transitions. Artifacts without this field remain valid schema-version 1
artifacts.

Each event has the following fields:

```json
{
  "sequence": 1,
  "timestamp_utc": "2026-07-30T18:24:46.000000Z",
  "elapsed_s": 0.0,
  "event_type": "MISSION_RECORDER_STARTED",
  "source": "mission_event_recorder",
  "severity": "INFO",
  "message": "Mission event recorder started for two-point-demo",
  "data": {"run_id": "two-point-demo"}
}
```

`sequence` starts at 1 and increases by one for every event. `elapsed_s` is
monotonic and should be used to order events within a run; `timestamp_utc` is
for correlation with external logs.

The recorder captures Polaris requests, mission/waypoint outcomes, avoidance
and safety transitions, collision detections, and observed connection, armed,
in-air, and flight-mode changes. A transition out of Offboard not initiated by
Polaris is recorded as `MISSION_EXTERNALLY_INTERRUPTED` with
`classification: OBSERVED_EXTERNAL_MODE_CHANGE`.

This is evidence of observed telemetry, not proof of the originating MAVLink
command or operator. QGC command and `COMMAND_ACK` capture remain required to
attribute an external action.

Inspect an artifact without mutating it:

```sh
./.venv/bin/python scripts/mission_timeline.py artifacts/mission_runs/<run>_mission.json
./.venv/bin/python scripts/mission_replay.py artifacts/mission_runs/<run>_mission.json
```
