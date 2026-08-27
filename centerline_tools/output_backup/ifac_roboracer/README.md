# IFAC Roboracer racing line

This directory contains the reviewed centerline and globally optimized racing
line for `f1tenth_gym_ros/maps/ifac_roboracer.yaml`.

The scanned map leaves a few short medial-axis branches after conservative
skeleton pruning. The selected traversal is a closed 50.05 m loop and covers
94.7% of that skeleton, so this generation used an explicit 0.94 coverage
threshold while retaining all closure and traversal-step checks:

```bash
mkdir -p /tmp/ifac_roboracer_generation
cd /tmp/ifac_roboracer_generation
CENTERLINE_HEADLESS=1 \
CENTERLINE_MIN_ORDERED_LOOP_COVERAGE_RATIO=0.94 \
  python3 /sim_ws/src/centerline_tools/centerline_reference_generator.py \
  /sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.png \
  /sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml
```

The map-specific optimizer configuration uses a `1.22` curvature weight, a
`0.30 m` safety margin, a `0.025` curvature-rate weight, and the numerical
solver tolerances needed for this short scanned circuit:

```bash
python3 /sim_ws/src/centerline_tools/optimize_global_raceline.py \
  --centerline /tmp/ifac_roboracer_generation/centerline_output/centerline_points_smooth.csv \
  --drivable-mask /tmp/ifac_roboracer_generation/centerline_output/drivable_region.npy \
  --map-yaml /sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml \
  --config /sim_ws/src/centerline_tools/config/global_raceline_optimizer_ifac_roboracer.yaml \
  --output /tmp/ifac_roboracer_generation/raceline_points_optimized.csv \
  --report /tmp/ifac_roboracer_generation/raceline_points_optimized_validation.yaml \
  --debug-plot /tmp/ifac_roboracer_generation/debug_global_raceline_optimization.png
```

The reviewed artifacts were generated from the copied map pair. The optimized
47.09 m loop has 0.329 1/m curvature RMS, 0.290 1/m^2 curvature-rate RMS at
the common comparison spacing, and 0.372 m minimum dense grid-cell clearance.
All validation flags in `raceline_points_optimized_validation.yaml` pass.

A headless ego-only simulation completed five laps on the final line with the
planner `READY` and arbitration in `RACELINE` for all 1,692 recorded samples.
No reactive fallback or no-safe-path event occurred. Mean lap time was
16.925 s, compared with 16.962 s over the three-lap pre-margin baseline.

For a manual ego-only check, run the simulator and autonomy stack in separate
container terminals:

```bash
f1 sim --no-obstacle
```

```bash
f1 auto \
  raceline_csv_path:=/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
```

That racing line supplies both the driven path and the planner's Frenet frame.
The smooth centerline is retained only as offline optimizer provenance.
