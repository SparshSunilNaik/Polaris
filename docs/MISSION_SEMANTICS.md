# Mission Semantics

Mission waypoints default to `POSITION`. Position waypoints retain the local-goal contract: enter horizontal and vertical tolerance, satisfy optional terminal yaw, hold at low speed, then pass mission verification.

An explicitly configured `OBSERVATION` waypoint may instead complete inside a bounded stand-off band after stable observation evidence. This is mission-level completion semantics, not object detection. The configured goal distance is local-NED geometry to the waypoint coordinate. Relative pseudo-depth is used only as a safety and quality gate; it is not treated as metric target range.

## Observation state

- `APPROACHING`: the existing local-goal navigator owns approach motion.
- `OBSERVATION_CANDIDATE`: a distinct fresh depth observation satisfies geometry, freshness, health, stability, risk, velocity, collision, and recovery-state gates.
- `OBSERVING`: Active Perception owns a zero-translation command and may continue bounded yaw. Dwell advances only on distinct depth sequences with newer inference timestamps.
- `OBSERVATION_COMPLETE`: the stable dwell is ready and the current command has passed the Safety Supervisor without a hard override.

Invalid conditions reset dwell. The observation opportunity has a fixed maximum wait; after timeout, observation completion is disabled for that waypoint attempt and ordinary positional navigation and failure policy continue. Difficulty moving forward is never completion evidence.

Collision, stale state, localization failure, connection loss, Offboard loss, confidence emergency, invalid state, controller abort, confirmed emergency proximity, and operator interruption retain their existing precedence. The Safety Supervisor remains the final command authority.

## Rock observation

`rock-observation` uses the preserved-run evidence band and existing system thresholds:

| Setting | Value | Basis |
|---|---:|---|
| Goal-distance band | 2.5-3.0 m | Safe pre-abort local-NED windows |
| Heading error | 12 deg | Existing navigation alignment tolerance |
| Visual health | 0.35 | Existing waypoint minimum |
| Depth health | 0.40 | Existing waypoint minimum |
| Depth stability | 0.75 | Existing reactive stability requirement |
| Centre risk | 0.45 maximum | Existing caution threshold |
| Horizontal speed | 0.20 m/s maximum | Preserved stable windows and retreat bound |
| Stable dwell | 0.6 s and 3 fresh frames | Two live safe windows reached 0.676-0.679 s at 3 FPS |
| Maximum wait | 6.0 s | Existing no-progress timeout |

Existing waypoints and generated replanning waypoints remain positional unless they explicitly declare `type: OBSERVATION` and a complete observation block.
