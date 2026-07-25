# Active Perception Recovery

## Purpose

Active perception performs bounded yaw-only observation when visual/depth quality or goal progress is inadequate. It does not spin continuously or explore without bounds.

## State machine

```text
IDLE
  -> TRIGGERED
  -> SCANNING
  -> SAMPLING
  -> SELECTING
  -> RETURNING
  -> COMPLETE

Any bound or acceptance failure -> FAILED -> hover / goal abort / landing
```

Every scan uses offsets `[-30, -15, 0, +15, +30]` degrees relative to the trigger heading. Each heading must remain continuously within 4 degrees through a 0.5-second settle, then receive at least two matching fresh visual/depth samples during 0.8 seconds.

## Triggers

- Visual health below `0.50`.
- Depth health below `0.45`.
- Temporal stability below `0.72`.
- No meaningful local-goal progress.
- Ambiguous blocked-centre side clearance.
- Reactive hover with no immediate motion.

Confidence `EMERGENCY`, stale required perception, localization failure, collision, and other hard overrides abort immediately instead of scanning.

## Viewpoint score

Prototype score weights in `config/active_perception.json`:

| Component | Weight |
|---|---:|
| Visual health | 0.24 |
| Depth health | 0.15 |
| Temporal stability | 0.14 |
| Forward clearance | 0.19 |
| Feature quality | 0.10 |
| Exposure quality | 0.06 |
| Goal alignment | 0.09 |
| Small yaw change | 0.03 |

Recent failed headings receive a `0.12` penalty. A clear pseudo-depth view cannot be accepted if visual health, depth health, or stability is below its hard viewpoint minimum.

## Bounds

- Minimum acceptable score: `0.52`.
- Minimum score improvement: `0.04`.
- Maximum recovery attempts: `2`.
- Maximum scans per goal: `2`.
- Recovery cooldown: `4 s`.
- Maximum total recovery yaw per scan: `180 deg`.
- Selected-heading hold: `0.4 s`.
- Post-recovery progress deadline: `5 s`.

## Commands

```sh
./.venv/bin/python scripts/local_goal_demo.py --scenario low-texture-recovery --display
./.venv/bin/python scripts/local_goal_demo.py --scenario glare-recovery --display
./.venv/bin/python scripts/local_goal_demo.py --scenario no-acceptable-viewpoint --display
./.venv/bin/python scripts/local_goal_demo.py --scenario no-progress-test --display
```

## Interpretation

The physical snowfield and glare runs demonstrated trigger, bounded scan, deterministic candidate rejection, healthier-heading selection, and resumed movement. Their local goals later failed safely within configured bounds. They must not be reported as goal-completion successes.

This is prototype viewpoint selection, not active SLAM or semantic exploration.
