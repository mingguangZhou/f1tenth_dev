# centerline_tools ROS 2 Foxy package

This package loads an offline-generated `centerline_points_smooth.csv` and publishes it into ROS 2 as:

- `nav_msgs/Path` on `/centerline_path`
- `visualization_msgs/MarkerArray` on `/centerline_markers`

It is designed to stay lightweight at runtime while remaining forward-compatible with future extra CSV columns such as curvature, heading, width, or speed hints.

## Expected CSV format

Required columns:

- `index`
- `x`
- `y`

Optional extra columns are detected automatically and preserved in memory for future downstream use.

## Recommended folder placement in your container

```bash
/sim_ws/src/centerline_tools
```

## Build

From your ROS workspace root:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select centerline_tools
source install/setup.bash
```

## Run with launch file

```bash
ros2 launch centerline_tools centerline_publisher.launch.py
```

## Run with explicit CSV override

```bash
ros2 launch centerline_tools centerline_publisher.launch.py \
  csv_path:=/sim_ws/src/centerline_tools/centerline_output/centerline_points_smooth.csv \
  frame_id:=map \
  use_sim_time:=true
```

## Useful checks

```bash
ros2 topic list | grep centerline
ros2 topic echo /centerline_path --once
ros2 topic echo /centerline_markers --once
ros2 node list
ros2 param list /centerline_publisher
```

## RViz

Set fixed frame to `map`, then add:

- Path display for `/centerline_path`
- Marker display for `/centerline_markers`

## Notes

- The node loads the CSV once at startup, then republishes at a low rate.
- If the loop is not closed within the configured tolerance, it can append the first point to the end automatically.
- Future CSV add-ons are not discarded. They are parsed and retained in the loader layer so the package can grow cleanly later.
