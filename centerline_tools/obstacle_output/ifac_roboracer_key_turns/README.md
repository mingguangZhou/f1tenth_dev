# IFAC Roboracer key-turn fixture

This fixture adds three `0.40 m` square obstacles at separate corner complexes.
Every obstacle retains at least the requested `0.50 m` lateral boundary gap
within raster tolerance and intersects the active raceline's `0.24 m` vehicle
envelope.

The obstacle indices use the optimized raceline as their smooth placement
reference. Regenerate the map from the repository root:

```bash
python3 centerline_tools/obstacle_creation_tool.py \
  f1tenth_gym_ros/maps/ifac_roboracer.png \
  f1tenth_gym_ros/maps/ifac_roboracer.yaml \
  --centerline centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --raceline centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --raceline-blocking-clearance-m 0.24 \
  --batch-spec centerline_tools/obstacle_specs/ifac_roboracer_key_turns.yaml \
  --output-dir centerline_tools/obstacle_output/ifac_roboracer_key_turns
```

Regenerate the one-agent traffic route:

```bash
python3 centerline_tools/generate_moving_agent_path.py \
  --centerline centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --raceline centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --obstacles centerline_tools/obstacle_output/ifac_roboracer_key_turns/ifac_roboracer_obstacles_clearance.csv \
  --map-image centerline_tools/obstacle_output/ifac_roboracer_key_turns/ifac_roboracer_obstacles.png \
  --map-yaml centerline_tools/obstacle_output/ifac_roboracer_key_turns/ifac_roboracer_obstacles.yaml \
  --output-dir centerline_tools/moving_agent_output/ifac_roboracer_moving_agent \
  --output-name ifac_roboracer_moving_agent_path \
  --passing-offset-m 0.45 \
  --minimum-nudge-clearance-m 0.35 \
  --maximum-curvature-inv-m 1.10 \
  --spawn-index 600 \
  --spawn-count 1
```

The generated route has `0.384 m` minimum continuous occupancy clearance and
`0.921 1/m` maximum curvature. Its required steering is `0.295 rad`, below the
traffic controller's `0.36 rad` limit.
