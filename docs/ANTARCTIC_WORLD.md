# Polaris Antarctic Evaluation World

## Purpose

`simulation/worlds/polaris_antarctica.sdf` is a deterministic, primitive-only Gazebo Harmonic evaluation world for local RGB navigation work. It deliberately combines low-texture snow, high-contrast landmarks, safe open areas, a forgiving obstacle corridor, a gate, a semantic target surrogate, and one controlled glare direction. It is not a photorealistic or scientifically calibrated Antarctic model.

The world does not change PX4's upstream default world, camera resolution, or sensor model. Shadows are disabled and all geometry uses boxes, cylinders, and planes to preserve local performance on Apple Silicon.

## Coordinates

Gazebo uses ENU: `+X` east, `+Y` north, and `+Z` up. PX4 local telemetry uses NED: north conceptually corresponds to Gazebo `+Y`, east to Gazebo `+X`, and down is the negative of Gazebo `+Z`. Do not directly copy coordinate tuples between the frames without conversion.

The X500 spawns at Gazebo `(0, 0, 0)` with yaw zero. Its RGB camera points along Gazebo `+X`, toward the obstacle corridor and distant gate.

The visual environment is Antarctic-style, but the SDF intentionally retains PX4's default Zurich geodetic origin. PX4's supplied fixed simulation magnetic field is consistent with that origin; using an Antarctic latitude without a corresponding magnetic-field model caused estimator heading resets and unsafe horizontal drift during yaw. The geodetic origin is therefore a simulator compatibility reference, not a claim about the depicted location.

| Zone | Approximate center / bounds in Gazebo ENU | Main dimensions |
|---|---|---|
| Safe takeoff | center `(0, 0)`, clear bounds `x=-4..4`, `y=-4..4` | 8x8 m minimum; 3x3 m blue cross marker |
| Low-texture snowfield | center `(-15, 0)`, bounds `x=-24..-6`, `y=-9..9` | 18x18 m bright patch; two low ridges |
| Rock landmarks | centers `(8,10)`, `(12,13)`, `(16,9)`, `(18,15)`, `(11,17)` | 1.1-3.4 m tall dark primitives |
| Obstacle corridor | centerline `y=0`, walls from `x=5..23` at `y=+/-4.5` | 8 m clear interior; walls 2.5 m tall |
| Offset obstacle | center `(15, 1.8)` | 1.6x1.6x1.8 m orange block; pass south of it |
| Navigation gate | center `(24, 0)` | 5.2 m opening, about 3.1 m vertical clearance |
| Research station | center `(12, -14)` | 7x5x3 m orange body, white roof, cyan door/windows, mast |
| Glare test | panel center `(-7, 19)` with light near `(-7,15,3)` | 10x5.5 m emissive white panel, local point light |
| Safe landing | center `(28, 0)` | 4x4 m green `H` marker, hazard-free ground |

All important geometry lies approximately within `x=-24..30`, `y=-18..20`.

## Top-Down Map

```text
                         NORTH (+Y)
                            ^
          glare panel       |       rock landmark zone
             [GLARE]        |      r  r   r
                 * light    |        r  r
                            |
   low-texture snowfield    |                    north wall
   +------------------+     |     +==================+  GATE   LAND
   |                  |     |     |                  |  |  |   [ H ]
   |       SNOW       |-----+--[ TAKEOFF ]----->  []|--|  |-----> +X
   |                  |     |     | offset obstacle  |
   +------------------+     |     +==================+
                            |                    south wall
                            |
                            |       [ RESEARCH STATION ]
                            +--------------------------------> EAST (+X)
```

## Running

Default-world behavior is unchanged:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
./scripts/start_polaris_sim.sh
```

Launch the Antarctic world:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
POLARIS_WORLD=antarctica ./scripts/start_polaris_sim.sh
```

Headless mode remains available:

```sh
POLARIS_WORLD=antarctica POLARIS_SIM_HEADLESS=1 ./scripts/start_polaris_sim.sh
```

Camera health:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/camera_health_monitor.py --display
```

Integrated Ego State:

```sh
./.venv/bin/python ego_state_monitor.py
```

Stop an orphaned terminal session with:

```sh
~/Documents/Polaris/PX4-Autopilot/scripts/stop_polaris_sim.sh
```

## Expected Observations

- Initial / east: blue corridor walls, orange offset obstacle, and yellow gate provide depth edges after takeoff.
- West / yaw about 180 degrees: the bright snowfield occupies most of the image and should reduce contrast, texture, corner distribution, and visual-health score.
- Northeast / yaw roughly 35-60 degrees: multiple dark landmarks at different ranges should improve contrast and geometric feature distribution.
- Southeast / yaw roughly -50 degrees: the orange station, cyan openings, roof edge, and mast provide a strong target signature.
- Northwest / yaw roughly 110-130 degrees: the emissive panel and local light should increase brightness and overexposure without affecting every direction.

The corridor starts 5 m east of spawn, leaving the required takeoff clearance. Its 8 m interior and 5.2 m gate opening are intentionally conservative for future closed-loop work. The offset obstacle requires a directional correction but leaves more than 5 m of lateral corridor space.

## Performance and Visual Health

Measurements on the target M4 MacBook Air retained 1280x960 `RGB_INT8` at approximately 30.3 FPS with the GUI open:

| View from takeoff | Brightness | Contrast | Blur | Features / coverage | Overexposed | Health |
|---|---:|---:|---:|---:|---:|---:|
| East corridor / initial yaw | 171.9 | 59.7 | 107.4 | 187 / 0.81 | 11.6% | 0.89 `HEALTHY` |
| West snowfield / yaw 180 deg | 198.1 | 40.5 | 56.5 | 70 / 0.44 | 12.5% | 0.63 `DEGRADED` |
| Northeast rocks / yaw 45 deg | 187.5 | 48.7 | 69.1 | 231 / 0.56 | 10.3% | 0.80 `HEALTHY` |
| Northwest glare / yaw 110 deg | 200.5 | 41.4 | 39.8 | 171 / 0.50 | 16.0% | 0.70 `HEALTHY` with exposure warning |

These values demonstrate directional variation while maintaining camera rate. They are evidence for simulation tuning, not environmental reliability claims.

Takeoff to 2.5 m, an 8-second hover, and landing completed successfully. A separate conservative Offboard test completed forward/back/right/left movement, climb/descent, clockwise/counter-clockwise yaw, and landing. The horizontal test displacement remained within the clear takeoff zone; gate traversal was not attempted.

The Antarctic camera topic is:

```text
/world/antarctica/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

Camera clients auto-discover the selected world's X500 topic, while `--topic` remains available for explicit selection.

## Limitations

- Rock shapes are low-poly surrogates, not terrain meshes.
- Lighting is deterministic and static; there is no weather, blowing snow, or sun motion.
- The glare panel approximates directional visual degradation rather than a physical solar model.
- Uniform primitive materials provide edges and color contrast but no realistic snow microtexture.
- Gate clearance is suitable for careful flight, not yet validated for autonomous passage.

## Future Use

The separated ranges, silhouettes, occlusions, and corridor geometry provide controlled inputs for later pseudo-depth evaluation. Named zones also support active-perception tests that compare visual health before and after deliberate yaw changes. No pseudo-depth, avoidance, semantic detection, or mission control is implemented by this world milestone.
