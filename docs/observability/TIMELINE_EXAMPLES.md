# Timeline Examples

`scripts/mission_timeline.py` prints mission-recorder events in their recorded
sequence order. The timeline for
`artifacts/mission_runs/20260730T183130Z_mission.json` includes:

```text
+004.647 FLIGHT_MODE_CHANGED  Flight mode changed HOLD -> OFFBOARD
+004.647 OFFBOARD_ENTERED     Offboard mode confirmed
+058.690 FLIGHT_MODE_CHANGED  Flight mode changed OFFBOARD -> LAND
+058.690 OFFBOARD_EXITED      Offboard mode exited to LAND
+062.698 LANDED               Vehicle landed
+063.660 DISARMED             Vehicle disarmed
+063.660 MISSION_COMPLETED    Mission finished with COMPLETED
```

This is Polaris's expected terminal transition, not external-interruption
evidence. An externally observed exit before Polaris terminal landing would
also contain `MISSION_EXTERNALLY_INTERRUPTED`.
