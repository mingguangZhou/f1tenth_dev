# path_following_v2 dummy RL-flow package

This package now contains two nodes:

1. `path_generator_node`
   - subscribes to `/centerline_waypoints` as `std_msgs/Float64MultiArray`
   - expects rows of `[index, x, y, yaw, curvature, curvature_abs]`
   - publishes `/path_following_v2/generated_path` as `nav_msgs/Path`
   - publishes `/path_following_v2/target_speed` as `std_msgs/Float64`

2. `path_following_v2_node`
   - subscribes to the generated path
   - uses pure pursuit to compute steering
   - uses the external target speed if it is fresh; otherwise falls back to the old steering-based speed rule
   - publishes Ackermann commands to `/drive`

Launch:

```bash
ros2 launch path_following_v2 path_following_v2.launch.py
```

The current path generator is a deterministic dummy replacement for the future RL agent. It computes lateral offset and target speed from simple rules configured in `config/path_following_v2.yaml`.
