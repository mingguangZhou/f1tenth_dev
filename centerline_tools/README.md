# centerline_tools ROS 2 Foxy package

This package contains two parts:

1. an offline Python generator, `generate_centerline.py`, which extracts a smoothed centerline and then generates a final smoothed raceline;
2. a ROS 2 Foxy publisher, `raceline_publisher`, which loads and publishes the final smoothed raceline by default.

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
