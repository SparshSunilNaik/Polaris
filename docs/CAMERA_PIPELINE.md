# RGB Camera and Visual Health Pipeline

## Architecture

Polaris subscribes directly to Gazebo Harmonic Transport from Python. No ROS, network-video bridge, frame files, or C++ intermediary is used.

1. `GazeboCameraSubscriber` receives `gz.msgs.Image` through `gz.transport13.Node`.
2. `decode_image_message` validates the format, dimensions, stride, and data size, then converts to an owned OpenCV BGR array.
3. The subscriber retains only the latest frame plus recent arrival times for FPS measurement.
4. `VisualHealthAnalyzer` computes deterministic metrics and returns a typed `VisualHealthReport`.
5. `EgoStateCoordinator` copies only report scalars into `NavigationState.visual`; image arrays remain transient and never enter long-term Ego State.

Gazebo invokes the subscription callback on a transport thread. Access to the latest frame, FPS window, sequence, and errors is protected by a lock. Temporary topic loss does not terminate the monitor; frame age continues increasing until the report becomes `CRITICAL` and stale.

## Dependencies

The tested Apple Silicon installation already supplies all native components:

- Python 3.14 arm64 in `./.venv`
- Homebrew `gz-transport13` and `gz-msgs10`
- Homebrew OpenCV Python module (`cv2` 4.13.0 at validation time)
- Homebrew NumPy
- Project-venv protobuf 7.35.1

Homebrew installs native Python modules under `/opt/homebrew/lib/python3.14/site-packages`, which a normal venv does not expose. `polaris.perception.dependencies` adds that directory at runtime without modifying `.zshrc`, system files, or the venv. Override it when needed:

```sh
export POLARIS_HOMEBREW_PYTHON_SITE=/path/to/site-packages
```

No additional installation was required. Confirm imports with:

```sh
./.venv/bin/python -c "import sys; sys.path.insert(0, 'src'); from polaris.perception.dependencies import add_homebrew_python_site; add_homebrew_python_site(); import cv2, gz.transport13, gz.msgs10.image_pb2; print(cv2.__version__)"
```

## Running

Terminal 1:

```sh
cd ~/Documents/Polaris/PX4-Autopilot
./scripts/start_polaris_sim.sh
```

Terminal 2:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/camera_health_monitor.py --no-display
```

Enable the native Cocoa OpenCV window:

```sh
./.venv/bin/python scripts/camera_health_monitor.py --display
```

Press `q` in the image window or `Ctrl+C` in the terminal to stop. Other useful options are `--topic`, `--rate`, `--duration`, `--config`, and `--json`.

Run the integrated MAVSDK, camera, Ego State, and Confidence monitor with one `System` instance:

```sh
./.venv/bin/python ego_state_monitor.py
```

The default topic is:

```text
/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

## Message Layout

The live Gazebo message was inspected before implementation:

- width: 1280
- height: 960
- pixel format: `RGB_INT8`
- channels: 3
- step: 3840 bytes
- data size: 3,686,400 bytes

The decoder honors `step`, removes any row padding, reshapes only after validation, and converts `RGB_INT8` to OpenCV's BGR order. It also explicitly supports `BGR_INT8`, `RGBA_INT8`, `BGRA_INT8`, and `L_INT8`; unsupported formats produce a visible transport error instead of guessed decoding. Gazebo's source timestamp is retained separately because simulation time is not comparable to the host monotonic clock used for frame age.

## Metrics

- **Frame age:** Host monotonic time since callback receipt. Above `stale_after_ms` is stale and `CRITICAL`.
- **FPS:** Arrival rate over the most recent 120 frames.
- **Brightness:** Mean grayscale intensity from 0 to 255.
- **Contrast:** Grayscale standard deviation.
- **Exposure ratios:** Fraction of pixels at or below 5 and at or above 250.
- **Blur:** Variance of the grayscale Laplacian. Lower values indicate fewer sharp edges.
- **Texture:** Mean Sobel gradient magnitude.
- **Feature count:** Shi-Tomasi corners from `goodFeaturesToTrack`.
- **Feature coverage:** Fraction of occupied cells in a 4x4 image grid.

## Prototype Thresholds

All initial values live in `config/visual_health.json`. They are transparent simulation-tuning defaults, not calibrated Antarctic reliability limits.

- Frame stale: 500 ms
- Healthy brightness: 55 to 210; critical endpoints: 15 and 245
- Healthy contrast: 35; critical: 8
- Exposure degradation starts at 15%; critical override at 80%
- Healthy Laplacian variance: 100; critical: 20
- Healthy mean gradient: 12; critical: 2
- Healthy corners: 150; critical: 20
- Healthy grid coverage: 50%; critical: 12.5%
- Healthy aggregate score: at least 0.70
- Critical aggregate score: below 0.35

The score is a normalized weighted average: brightness 15%, contrast 15%, exposure 15%, blur 20%, texture 15%, and combined feature count/coverage 20%. Stale data and extreme exposure override the average as `CRITICAL`. Scores between 0.35 and 0.70 are `DEGRADED`.

## Tests

```sh
./.venv/bin/python -m unittest discover -s tests -v
```

Tests cover textured, white, black, blurred, low-texture gray, stale, and unavailable frames.

## Confidence Integration

`config/confidence.json` contains the prototype confidence thresholds. The engine uses the minimum applicable subsystem health, never a weighted average. It also applies explicit airborne hard overrides:

- missing or stale camera: `EMERGENCY`
- failed or stale local position: `EMERGENCY`
- unusable camera exposure: `EMERGENCY`
- critically low visual score: `EMERGENCY`
- other critical visual state: at least `DEGRADED`
- degraded visual state: at least `CAUTIOUS`

Missing or stale camera data on the ground remains visible as factual visual health `0.0`, but is gated out of flight-critical overall health and does not cause an airborne emergency mode.

## Limitations and Next Integration

- Thresholds need tuning across representative simulation lighting, terrain, weather, motion, and camera poses.
- Laplacian variance and corner count vary with resolution and scene scale.
- A visually healthy frame does not prove geometric localization or obstacle clearance.
- The default world has limited visual texture, so a stationary ground-facing or sky-facing camera may legitimately report degraded health.
- This milestone does not estimate depth or perform avoidance.

The next integration should add timestamped confidence history and transition debouncing for non-critical mode changes. Hard safety overrides must remain immediate. A later milestone can consume the same transient BGR frame for pseudo-depth and active-perception experiments without retaining images in long-term state.
