# Unresolved Safety Questions

## Software Questions

- Can autonomy resume after Hold, and who explicitly authorizes it?
- How is an external Hold/Land/RTL command distinguished from transient
  telemetry or Offboard failure?
- How are command source and ACK captured alongside the recorder's observed
  mode transition and override reason?
- What passive monitoring behavior remains after a valid override?
- What happens if QGC disconnects after Hold?
- What happens if Polaris crashes while Offboard is active?
- Which operator status is authoritative after an override?
- How is an airborne QGC action time-correlated with mission events so a
  post-landing mode display cannot be mistaken for an in-flight override?

## PX4 Configuration Questions

- What are the active Offboard-loss action and timeout parameters in this SITL?
- Which QGC Hold action and MAVLink command/ACK occur on this PX4/QGC version?
- Does Position mode meet estimator prerequisites in the Antarctic world?
- Is global position/home valid enough for RTL, and what does PX4 reject when it
  is not?
- What confirmation and rejection policy applies to airborne disarm?

## Real-Hardware Questions

- Should RC input outrank QGC and Polaris?
- Who owns emergency disarm or kill with real motors/propellers?
- How do radio loss, telemetry loss, power state, and physical surroundings
  alter emergency authority?
- What landing and RTL behavior is acceptable without GNSS?

## Startup Product-Policy Questions

- Should commands require user roles or authentication?
- How are multiple GCS clients prevented or audited?
- How are emergency commands retained in an audit trail?
- Which role may authorize resume after an override?
- Should Land always override every autonomy subsystem?
