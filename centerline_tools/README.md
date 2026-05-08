# centerline_tools ROS 2 Foxy package

This package publishes an offline-generated centerline CSV into ROS 2 for path following and RViz debugging.

Runtime outputs:

- `nav_msgs/Path` on `/centerline_path`
- `visualization_msgs/MarkerArray` on `/centerline_markers`
- `std_msgs/Float64MultiArray` on `/centerline_waypoints`

The RViz marker output now contains a simple direction visualization:

- green `LINE_STRIP`: centerline shape
- red sphere: start point / first CSV point
- yellow arrows: sampled driving direction along the centerline
- optional blue spheres: sampled waypoint points, disabled by default

## Package placement

Recommended location in the simulator workspace:

```bash
/sim_ws/src/centerline_tools
```

## Build

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select centerline_tools
source install/setup.bash
```

## Basic run

```bash
ros2 launch centerline_tools centerline_publisher.launch.py
```

By default, the launch file loads:

```bash
centerline_output/centerline_points_smooth.csv
```

relative to the installed package share directory.

## Run with explicit CSV path

Use this when testing directly from the source tree:

```bash
ros2 launch centerline_tools centerline_publisher.launch.py \
  csv_path:=/sim_ws/src/centerline_tools/centerline_output/centerline_points_smooth.csv \
  frame_id:=map \
  use_sim_time:=true
```

## Reverse the published direction

If the arrows point opposite to the desired driving direction, run:

```bash
ros2 launch centerline_tools centerline_publisher.launch.py \
  direction:=reverse
```

Valid direction values are:

- `csv` / `normal`: use the CSV order
- `reverse`: reverse the centerline order before publishing

The node recomputes yaw and curvature after the final direction choice, so the Path orientation, direction arrows, and waypoint rows stay consistent.

## RViz2 setup

In RViz2:

1. Set `Fixed Frame` to `map`.
2. Add a `Path` display:
   - Topic: `/centerline_path`
3. Add a `MarkerArray` display:
   - Topic: `/centerline_markers`
   - Reliability Policy: `Reliable`
   - Durability Policy: `Transient Local`
   - History Policy: `Keep Last`
   - Depth: `1` or higher

The yellow arrows in `/centerline_markers` show the current published centerline direction. The red sphere marks the first point of the published path.

## Useful launch parameters

```bash
csv_path:=centerline_output/centerline_points_smooth.csv
frame_id:=map
path_topic:=/centerline_path
marker_topic:=/centerline_markers
waypoints_topic:=/centerline_waypoints
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

For a denser direction display:

```bash
ros2 launch centerline_tools centerline_publisher.launch.py \
  direction_arrow_stride:=20 \
  direction_arrow_length:=0.25
```

For a cleaner display with no arrows:

```bash
ros2 launch centerline_tools centerline_publisher.launch.py \
  publish_direction_arrows:=false
```

## Useful checks

```bash
ros2 topic list | grep centerline
ros2 topic echo /centerline_path --once
ros2 topic echo /centerline_markers --once
ros2 topic echo /centerline_waypoints --once
ros2 node list
ros2 param list /centerline_publisher
```

## CSV format

Required columns:

- `index`
- `x`
- `y`

Optional columns such as `yaw`, `curvature`, and `curvature_abs` are accepted. The publisher recomputes directional fields at startup so they remain consistent with the chosen `direction` parameter.

## Notes

- The node loads the CSV once at startup, then republishes the static data at a low wall-time rate.
- Transient-local QoS is used so RViz can still receive markers if it starts after the publisher.
- If the loop is not closed within the configured tolerance, the node can append the first point to the end automatically.
- The default marker display is intentionally simple: line + start marker + direction arrows.
