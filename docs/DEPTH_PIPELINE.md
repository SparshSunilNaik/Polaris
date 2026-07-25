# Relative Pseudo-Depth Pipeline

## Model and Licence

Polaris uses the official Hugging Face Transformers conversion of Depth Anything V2 Small:

```text
Identifier: depth-anything/Depth-Anything-V2-Small-hf
Revision:   5426e4f0f36572d16453bbda7a8389317b1bef99
Parameters: 24.8 million, float32
Source:     https://huggingface.co/depth-anything/Depth-Anything-V2-Small-hf
Licence:    Apache-2.0
```

Only the Small checkpoint is used. Base, Large, Giant, metric-depth variants, Video Depth Anything, and fine-tuning are not part of this pipeline.

Transformers stores configuration and the approximately 99 MB safetensors checkpoint in the standard Hugging Face user cache, normally `~/.cache/huggingface/hub`. Polaris does not commit model weights or hardcode the resolved cache directory.

## Semantics

Depth Anything V2 Small emits relative, non-metric depth-like values. Official and live checks show that larger raw values correspond to nearer scene content for this checkpoint. Polaris converts them to:

```text
normalized_proximity in [0, 1]
0 = relatively far / clear
1 = relatively near / higher collision concern
```

Raw output is never labelled as distance or metres. Each frame uses robust 5th and 95th raw percentiles as far and near anchors. Values are clipped to `[0,1]`. This gives useful within-frame spatial ordering but does not make scores comparable to physical distances.

Each of nine image regions stores its 75th percentile proximity, avoiding sensitivity to a single predicted pixel:

```text
UP_LEFT     UP        UP_RIGHT
LEFT        CENTRE    RIGHT
DOWN_LEFT   DOWN      DOWN_RIGHT
```

## Architecture

`EgoStateCoordinator` remains the only Gazebo camera subscriber. Its visual-analysis loop submits references to newly decoded `CameraFrame` objects to `DepthWorker`.

`DepthWorker` is a dedicated thread with one replaceable pending slot:

- It always processes the newest pending frame.
- A newer frame replaces an older pending frame.
- It never builds an unbounded queue.
- Camera ingestion and MAVSDK asyncio tasks do not wait for model inference.
- The default scheduler targets 3 FPS.

The worker owns the latest normalized full map only for transient display. `NavigationState` receives `DepthState`, which contains scalar health, timing, dimensions, and the 3x3 grid. No RGB frame or full depth map is stored in long-term Ego State or benchmark artifacts.

## Temporal Processing

Regional risks use an exponential moving average:

```text
smoothed = alpha * current + (1 - alpha) * previous
```

The default `alpha` is `0.45`. Temporal stability is `1 - mean(abs(current - previous))`. Regional concern must persist for two updates before crossing the configured risk threshold, except a centre prediction above `0.90`, which receives an immediate conservative override.

## Health

Depth health considers:

- model availability and inference exceptions
- source-frame and inference freshness
- NaN/Inf output
- robust raw dynamic range
- temporal grid stability
- source visual-health condition
- source texture and exposure

Unavailable, stale, non-finite, or unusably flat output is `CRITICAL`. Fresh output can still be `DEGRADED` because poor RGB evidence makes neural predictions less trustworthy.

Depth defaults to monitoring-only:

```json
"depth_required_for_motion": false
```

Monitoring failures remain visible in Confidence Engine reasons but do not ground an otherwise healthy vehicle. If depth is configured as required for motion, unavailable, stale, or critical airborne depth is an `EMERGENCY` hard override. Existing camera and localization overrides remain unchanged.

## Device Selection

Supported values are:

- `auto`: use MPS when `torch.backends.mps.is_available()`, otherwise CPU
- `mps`: require MPS and fail clearly if unavailable
- `cpu`: force CPU

Inference uses `torch.inference_mode()` and float32. The checkpoint itself is float32. A controlled MPS float16 check remained finite but changed normalized output by about 26% and compressed the synthetic raw range from `1.93` to `0.67`; float16 is therefore rejected for this safety baseline.

## Installation

Dependencies are project-local:

```sh
cd ~/Documents/Polaris
./.venv/bin/pip install -r requirements-depth.txt
```

## Live Monitor

Start Antarctic SITL, then run:

```sh
cd ~/Documents/Polaris
./.venv/bin/python scripts/depth_monitor.py --display
```

Headless timed sample:

```sh
./.venv/bin/python scripts/depth_monitor.py --no-display --duration 30
```

Device and scheduler overrides:

```sh
./.venv/bin/python scripts/depth_monitor.py --device cpu --target-fps 2
```

Display output is explicitly labelled `RELATIVE PSEUDO-DEPTH RISK - NOT METRES`.

## Viewpoint Benchmark

```sh
./.venv/bin/python scripts/depth_viewpoint_benchmark.py
```

Configuration-only validation:

```sh
./.venv/bin/python scripts/depth_viewpoint_benchmark.py --dry-run
```

Outputs are written without replacing visual baselines:

```text
artifacts/depth_runs/<timestamp>_depth_viewpoints.json
artifacts/depth_runs/<timestamp>_depth_viewpoints.csv
```

The benchmark loads `artifacts/viewpoint_runs/20260723T141537Z_viewpoints.json`, records the current visual report and depth report for every named heading, and includes the visual-health delta from that baseline.

The depth flight uses a deterministic safe sweep from west through south and east to north, with `northwest_glare` last. Leaving the glare heading toward west produced transient visual `EMERGENCY` states in live validation. Making glare the final sample allows the existing Safety Supervisor behavior to remain unchanged and lands immediately afterward instead of repeatedly crossing that boundary.

## Limitations

- Relative monocular output has unknown scale and shift and cannot establish physical clearance.
- Per-frame percentile normalization emphasizes spatial ordering, not cross-run metric comparability.
- A 3x3 grid discards fine obstacle shape and cannot yet command motion.
- The Small model can flicker across lighting changes despite temporal smoothing.
- MPS and CPU can have small numerical differences.
- No obstacle avoidance or autonomous traversal consumes this state yet.
