# Spielberg key-turn obstacle map

This directory contains a reproducible nine-obstacle Spielberg map generated
from `obstacle_specs/spielberg_key_turns.yaml`. Five obstacles are added at
major turns. Four retain the earlier anchor locations; two of those are moved
to the side occupied by the active raceline because their previous placement
did not actually require avoidance.

Every obstacle passed the following generation gates:

- square size at or below `0.50 m`;
- at least `0.50 m` requested clearance to both track boundaries, within the
  map raster tolerance;
- no overlap with another added obstacle;
- intersection with the active raceline's `0.24 m` physical vehicle envelope.

Regenerate from `/sim_ws/src/centerline_tools`:

```bash
python3 obstacle_creation_tool.py Spielberg_map.png Spielberg_map.yaml \
  --centerline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/centerline_points_smooth.csv \
  --raceline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv \
  --raceline-blocking-clearance-m 0.24 \
  --batch-spec obstacle_specs/spielberg_key_turns.yaml \
  --output-dir obstacle_output/spielberg_key_turns
```

## Three-lap simulator result

Tested on 2026-08-17 with:

```bash
/sim_ws/src/path_following_v2/tools/run_multi_lap_test.sh \
  --laps 3 --timeout 600
```

The vehicle completed all nine obstacle encounters on all three laps:

| Result | Value |
|---|---:|
| Completed laps | 3 / 3 |
| Lap length | 341.86 m |
| Lap times | 96.57 s, 92.29 s, 94.88 s |
| Total test time | 284.01 s |
| Mean cross-track error | 0.166 m |
| 95th-percentile cross-track error | 0.717 m |
| Maximum cross-track error | 1.050 m |
| RACELINE arbitration samples | 5,330 / 5,674 |
| REACTIVE arbitration samples | 344 / 5,674 |

The harder map exposed three confirmed no-safe-path intervals: obstacle 6 on
lap 1, obstacle 8 on lap 3, and obstacle 9 on lap 3. Reactive control safely
completed those passes and normal raceline ownership recovered afterward. A
separate short lower-level time-to-collision intervention occurred near
obstacle 7 on lap 2. These are recorded as test findings rather than hidden by
loosening safety thresholds.

Raw CSV, event JSONL, and summary JSON are written to the ignored
`path_following_v2/trial_logs/multi_lap/` directory.
