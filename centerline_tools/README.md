# centerline_tools ROS 2 Foxy package

This package contains three parts:

1. an offline Python generator, `generate_centerline.py`, which extracts a smoothed centerline and then generates a final smoothed raceline;
2. a headless global optimizer, `optimize_global_raceline.py`, which generates a map-bounded minimum-curvature raceline without corner keypoints;
3. a ROS 2 Foxy publisher, `raceline_publisher`, which loads and publishes the selected raceline.

The package name is still `centerline_tools` because the offline pipeline still contains both centerline and raceline generation. The ROS-facing API now uses `raceline` names for the active published output.

## Main outputs

After running the offline generator, the important files are in `centerline_output/`:

- `centerline_points_smooth.csv`: final smoothed centerline with geometry fields
- `raceline_points_smooth.csv`: final smoothed raceline with the same geometry fields
- `raceline_offset_field.csv`: legacy x/y-only raceline export kept for older debug scripts
- `debug_raceline_offset_field.png`: raceline overlay debug image
- `debug_raceline_curvature.png`: raceline curvature debug plot

Both `centerline_points_smooth.csv` and `raceline_points_smooth.csv` use this format:

```text
index,x,y,yaw,curvature,curvature_abs
```

## Package placement

Recommended simulator workspace location:

```bash
/sim_ws/src/centerline_tools
```

## Offline generation

From the package root:

```bash
cd /sim_ws/src/centerline_tools
python3 generate_centerline.py Spielberg_map.png Spielberg_map.yaml
```

For another map, replace the image and YAML names:

```bash
python3 generate_centerline.py <map_image.pgm-or-png> <map.yaml>
```

Expected final raceline output:

```bash
centerline_output/raceline_points_smooth.csv
```

Quick check:

```bash
head centerline_output/raceline_points_smooth.csv
```

The header should be:

```text
index,x,y,yaw,curvature,curvature_abs
```

## Global raceline optimization

The global optimizer consumes the Phase-9 centerline, selected drivable-region
mask, and ROS map metadata. It optimizes one lateral offset at each uniformly
sampled centerline station. The objective is solved in three deterministic
stages: segment/length initialization, minimum curvature, and curvature-rate
smoothing. The default length and lateral-transition weights balance minimum
curvature with an outside-inside-outside corner line; the offset-to-center term
remains small. All objective terms wrap across the lap seam.

Legal lateral offsets are ray-cast from the centerline through an eroded
drivable-region mask. Before saving, the tool densely validates the complete
loop for corridor clearance, self-intersection, direction, finite geometry,
and maximum curvature. A failed solve or validation does not replace the final
CSV/report pair.

Run the optimizer against the active Spielberg artifact set:

```bash
cd /sim_ws/src/centerline_tools
python3 optimize_global_raceline.py \
  --centerline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/centerline_points_smooth.csv \
  --drivable-mask output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/drivable_region.npy \
  --map-yaml Spielberg_map.yaml \
  --config config/global_raceline_optimizer.yaml \
  --baseline-raceline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv \
  --output output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_optimized.csv \
  --report output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_optimized_validation.yaml
```

`--baseline-raceline` records a common-spacing comparison with the manually
tuned line; it is not used as an optimization input. For another map, keep the
centerline, mask, and map YAML from the same generation run.

To also render the optional comparison plot, install the package dependencies
with `rosdep` and add:

```bash
--debug-plot output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/debug_global_raceline_optimization.png
```

The default output name is `raceline_points_optimized.csv`. The optimizer never
replaces its source centerline or a comparison raceline. This keeps the active
vehicle path unchanged until the optimized CSV is explicitly selected.

Generate a fresh centerline and optimized raceline in one command:

```bash
python3 run_centerline_and_optimized_raceline.py Spielberg_map.png Spielberg_map.yaml
```

The validation YAML records input hashes, resolved parameters, each solver
stage, numerical-library versions, common-spacing before/after metrics, and
dense safety results. Its clearance metric is a conservative nearest-grid-cell
sample, not an exact Euclidean distance to the continuous wall boundary. The
debug plot overlays the centerline, optional baseline, and optimized result.
The shipped `1.082 1/m` curvature gate matches the runtime 20.6-degree steering
limit, 0.33 m wheelbase, and 0.95 safety factor; change it only together with
the vehicle/controller envelope.

After reviewing the CSV, report, and plot, test it in the simulator with an
explicit launch override:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  raceline_csv_path:=/sim_ws/src/centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_optimized.csv
```

This selection changes the global reference path used by the running stack;
generating the file alone does not change runtime vehicle behavior.

The current Phase-9 centerline generator assumes a zero map-origin yaw. The
optimizer's coordinate conversion supports rotated maps, but a rotated-map
centerline must come from a generator that applies the same origin rotation.

## Reproducible obstacle maps

`obstacle_creation_tool.py` supports both its graphical editor and a batch
workflow. Batch mode applies the same track-boundary, obstacle-size, raster,
overlap, and global-clearance checks as the editor. It additionally requires a
raceline and rejects any obstacle that does not block the configured physical
vehicle envelope.

Generate the nine-obstacle Spielberg key-turn map from the same
centerline/raceline pair used by the simulator autonomy launch:

```bash
cd /sim_ws/src/centerline_tools
python3 obstacle_creation_tool.py Spielberg_map.png Spielberg_map.yaml \
  --centerline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/centerline_points_smooth.csv \
  --raceline output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv \
  --raceline-blocking-clearance-m 0.24 \
  --batch-spec obstacle_specs/spielberg_key_turns.yaml \
  --output-dir obstacle_output/spielberg_key_turns
```

The output includes the ROS map image and YAML, a clearance CSV, and a debug
overlay showing the centerline, active raceline, obstacle labels, boundary
gaps, and obstacle-to-raceline clearance. The batch command fails instead of
silently saving if any placement is invalid or does not block the raceline
envelope.

To place obstacles manually, omit `--batch-spec` and `--raceline`; the existing
graphical workflow opens and the original input map remains unchanged.

## Build

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select centerline_tools
source install/setup.bash
```

Because `centerline_output/*` is installed into the package share directory, rebuild after regenerating the offline raceline if you want the installed launch default to use the newest CSV.

## Basic ROS run: publish final smoothed raceline

Recommended launch command:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py
```

A compatibility launch file also remains available:

```bash
ros2 launch centerline_tools centerline_publisher.launch.py
```

Both launch files start the same ROS node:

```text
/raceline_publisher
```

By default, the launch file loads:

```bash
centerline_output/raceline_points_smooth.csv
```

relative to the installed package share directory.

Published outputs:

- `nav_msgs/Path` on `/raceline_path`
- `visualization_msgs/MarkerArray` on `/raceline_markers`
- `std_msgs/Float64MultiArray` on `/raceline_waypoints`

## Run directly from source-tree CSV

Use this if you regenerated the CSV in the source tree but have not rebuilt yet:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py \
  csv_path:=/sim_ws/src/centerline_tools/centerline_output/raceline_points_smooth.csv \
  frame_id:=map \
  use_sim_time:=true
```

## Publish the centerline instead, if needed

The same publisher can still publish the centerline CSV when explicitly requested:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py \
  csv_path:=centerline_output/centerline_points_smooth.csv
```

If you do this, the topic names still remain `/raceline_*` unless you override them manually.

## Reverse the published direction

If the arrows point opposite to the desired driving direction:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py \
  direction:=reverse
```

Valid direction values:

- `csv` / `normal`: use the CSV order
- `reverse`: reverse the loaded path before publishing

The node recomputes yaw and curvature after the final direction choice, so the Path orientation, direction arrows, and waypoint rows stay consistent.

## RViz2 setup

In RViz2:

1. Set `Fixed Frame` to `map`.
2. Add a `Path` display:
   - Topic: `/raceline_path`
3. Add a `MarkerArray` display:
   - Topic: `/raceline_markers`
   - Reliability Policy: `Reliable`
   - Durability Policy: `Transient Local`
   - History Policy: `Keep Last`
   - Depth: `1` or higher

Marker meaning:

- green `LINE_STRIP`: published raceline shape
- red sphere: start point / first CSV point
- yellow arrows: sampled published direction
- optional blue spheres: sampled waypoint points, disabled by default

## Useful launch parameters

```bash
csv_path:=centerline_output/raceline_points_smooth.csv
frame_id:=map
path_topic:=/raceline_path
marker_topic:=/raceline_markers
waypoints_topic:=/raceline_waypoints
direction:=csv
publish_rate_hz:=1.0
publish_start_marker:=true
publish_direction_arrows:=true
direction_arrow_stride:=40
direction_arrow_length:=0.35
publish_point_markers:=false
point_marker_stride:=10
use_sim_time:=true
```

For denser direction arrows:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py \
  direction_arrow_stride:=20 \
  direction_arrow_length:=0.25
```

For a cleaner display with no arrows:

```bash
ros2 launch centerline_tools raceline_publisher.launch.py \
  publish_direction_arrows:=false
```

## Useful checks

```bash
ros2 node list | grep raceline
ros2 topic list | grep raceline
ros2 topic echo /raceline_path --once
ros2 topic echo /raceline_markers --once
ros2 topic echo /raceline_waypoints --once
ros2 param list /raceline_publisher
ros2 param get /raceline_publisher csv_path
```

Expected active ROS API:

```text
/raceline_publisher
/raceline_path
/raceline_markers
/raceline_waypoints
```

## CSV format consumed by ROS publisher

Required columns:

- `index`
- `x`
- `y`

Recommended geometry columns:

- `yaw`
- `curvature`
- `curvature_abs`

The publisher accepts missing geometry columns, but it recomputes `yaw`, `curvature`, and `curvature_abs` at startup anyway. This keeps the published Path orientation and waypoint rows consistent with the selected `direction` parameter.

## Notes

- The node loads the CSV once at startup, then republishes the static data at a low wall-time rate.
- Transient-local QoS is used so RViz can still receive markers if it starts after the publisher.
- If the loop is not closed within the configured tolerance, the node can append the first point to the end automatically.
- The default marker display is intentionally simple: line + start marker + direction arrows.
