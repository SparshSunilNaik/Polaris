# Collision Instrumentation

## Method

The project-owned Antarctic SDF attaches native Gazebo `contact` sensors to all physical world collisions:

- Ground
- Snow ridges
- Rock landmarks
- Corridor walls and offset obstacle
- Navigation gate
- Research station
- Glare panel
- Physical centred obstacle

All sensors publish `gz.msgs.Contacts` on:

```text
/polaris/world/contacts
```

No PX4 source or upstream X500 model is modified. The monitor filters contact pairs for `x500_mono_cam_<n>` collision names.

## State

`CollisionState` exposes:

- Availability
- Latched collision detection
- Collision episode count
- Cumulative native/injected evidence flags
- Latest timestamp
- Drone and contacted entity names
- Contact position and normal
- Expected ground-contact count
- Bounded event history

## Ground classification

Ground contact is expected only after the PX4 landing command has been accepted. It must also have:

- Known penetration depth no greater than `0.01 m`.
- Last measured vertical speed no greater than `1.0 m/s` in magnitude.

Missing depth, early contact, excessive depth, or excessive vertical speed fails closed as a collision. An unexpected event can escalate a previously expected pair inside the de-duplication cooldown.

## Safety behavior

Any latched airborne collision passed to `SafetySupervisor` produces:

```text
ABORT
zero body velocity
hard_override = true
recommended_action = LAND
```

Collision state receives a dominant red Operator View banner and is stored in the navigation artifact.

## Validation

Native instrumentation availability and actual parked/landing ground messages were validated. Physical centred-obstacle navigation recorded zero contact events.

The isolated collision hard-override scenario uses an explicit software-injected event, labelled:

```text
goal_outcome = INJECTED_COLLISION_ABORT
collision_evidence_mode = INJECTED
```

It does not claim a physical airborne impact. This avoids intentionally striking the vehicle while still validating latching, supervisor override, artifact recording, hover, and landing.

## Commands

```sh
./.venv/bin/python scripts/local_goal_demo.py --scenario collision-test --display
```

Inspect the native topic:

```sh
gz topic -i -t /polaris/world/contacts
```

## Limitations

- Only collisions instrumented in the Antarctic project world are covered.
- The default PX4 world is unchanged and does not expose this topic.
- Contact depth and solver values are simulation-engine outputs, not calibrated impact severity.
- The event history is bounded, while cumulative evidence flags remain latched.
- Simulator only.
