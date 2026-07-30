# Changelog

## v1.0.0-polaris-core

- Stabilized the PX4 SITL and Gazebo Harmonic launch workflow, including the
  Antarctica simulation environment.
- Added reproducible mission artifacts and mission replay support.
- Verified stock QGroundControl coexistence on its dedicated GCS endpoint.
- Added the Mission Event Recorder with optional, ordered artifact events.
- Added the read-only `scripts/mission_timeline.py` timeline utility.
- Documented QGC emergency-command ownership, operator policy, and known
  evidence limitations.
- Added observability documentation and replay validation compatible with
  legacy schema-version 1 artifacts.
- Validated the baseline with 292 passing tests and 6 passing subtests.
