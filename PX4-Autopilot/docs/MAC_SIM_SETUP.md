# macOS PX4 Gazebo Harmonic

`~/Documents/Polaris/commands.txt` is the canonical quick-reference command file for Polaris demonstrations and development. Update it whenever a user-facing command, flag, or script location changes.

Run one PX4 SITL and Gazebo Harmonic instance from the repository root:

```sh
./scripts/start_polaris_sim.sh
```

The script binds Gazebo transport to `127.0.0.1`, matching PX4's `gz_x500` target. It starts the Gazebo server, waits for `/world/default/scene/info`, starts a separate GUI client (required by Gazebo Harmonic on macOS), and then opens the PX4 `pxh>` shell. Logs are retained in `.polaris-sim/`.

For headless operation:

```sh
POLARIS_SIM_HEADLESS=1 ./scripts/start_polaris_sim.sh
```

Press `Ctrl+C` in the PX4 terminal to stop the simulation. If that terminal was closed unexpectedly, run:

```sh
./scripts/stop_polaris_sim.sh
```

Do not set `GZ_TRANSPORT_LOCALHOST_ONLY`, `IGN_TRANSPORT_DISABLE_MULTICAST`, or `IGN_IP`. The launcher removes those incompatible legacy overrides and consistently uses `GZ_IP=127.0.0.1` for every Gazebo process.

At `pxh>`, verify the active sensor topics:

```sh
listener sensor_accel -n 1
listener sensor_gyro -n 1
listener sensor_baro -n 1
listener sensor_mag -n 1
listener vehicle_local_position -n 1
commander check
```

Current PX4 no longer publishes the deprecated `sensor_combined` topic. Standard `gz_x500` publishes `vehicle_local_position_groundtruth`; it does not configure the optional visual-odometry publisher, so `vehicle_odometry` is not expected for this model.

`commander arm`, `commander takeoff`, and `commander land` require a valid RC or GCS connection because the default `NAV_DLL_ACT=2` safety policy is retained. Connect QGroundControl to the SITL MAVLink endpoint before arming; no preflight or sensor checks are disabled by this setup.
