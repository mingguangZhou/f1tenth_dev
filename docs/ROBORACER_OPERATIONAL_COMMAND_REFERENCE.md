# RoboRacer operational command reference

## 1. How to use this document

This is the maintained manual command reference for the canonical ROS 2 Foxy
simulation environment. It covers simulator, localization, onboard bring-up,
regression checks, and the optional recorded localization baseline. Simulation
commands run inside the existing `f1tenth_gym_ros_rocker` container, using the
`f1tenth_gym_ros_localization_ready` image. Onboard commands are added here when
that workflow is verified.

The perfect-localization and PF-localization simulator workflows remain separate.
The PF + PnC closed-loop measurement runner is a third explicit workflow; it
starts all required processes itself and must not be combined with manual launch
terminals.

This is the source of truth for copyable human commands. Use the
[development environment](DEVELOPMENT_ENVIRONMENT.md) for container/workspace facts
and [localization simulation and shared evaluation](LOCALIZATION_SIMULATION.md) for
startup contracts, recorded signals, metric definitions, and evidence limits.

## 2. Open the canonical container

On the dev laptop host:

```bash
cd /home/mzhou/f1tenth_dev
./scripts/rr_container.sh status
./scripts/rr_container.sh shell
```

Inside each container terminal, source the ROS installation and workspace:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
source install/local_setup.bash
```

Open a second container shell with `./scripts/rr_container.sh shell` when a
workflow calls for another terminal. Stop a launch with Ctrl-C in its terminal;
do not stop the container.

## 3. Perfect-localization simulator and full stack

Use two sourced container terminals. Build each package group before launching.

Terminal 1 — simulator:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2 — existing full PnC stack:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select centerline_tools path_following_v2 reactive_control_v2 drive_arbitration_v2 oudtra_driver_bringup
source install/local_setup.bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

This is the perfect-localization reference workflow. It uses the normal simulator
profile and is not a PF localization evaluation.

## 4. PF localization simulator and manual full stack

Use the corrected Step 3.1 evaluation profile. The normal three-terminal sequence
without these overlays may leave the car stationary: PF TF is asynchronous, the
simulator publishes wall time, and lower safety needs map-frame PF odometry.
Open three sourced container terminals and start the simulator first.

Terminal 1 — simulator:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_slam_launch.py \
  config_file:=/sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml
```

Terminal 2 — PF localization with the evaluation overlay:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select particle_filter
source install/local_setup.bash
ros2 launch particle_filter localize_sim_launch.py \
  parameter_overlay:=/sim_ws/src/oudtra_driver_bringup/config/localization_eval.yaml
```

After PF is running, send one initial pose in RViz with `2D Pose Estimate`, or
publish the configured IFAC start pose once on `/initialpose`. The pose must be
in `map`; wait for fresh PF health and map-to-laser TF before starting PnC. For
the configured IFAC start pose, the one-shot command is:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -10.0514058817227, y: 5.35332819747966, z: 0.0}, orientation: {z: -0.9637981767743538, w: 0.26663284577566865}}}}"
```

Do not continuously publish simulator GT. The one-shot initial pose is only for
PF initialization; the evaluation overlay keeps the PF and PnC data flow
separate from `/simulator/agent_status`.

Terminal 3 — full stack with the evaluation profile:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select centerline_tools path_following_v2 reactive_control_v2 drive_arbitration_v2 oudtra_driver_bringup
source install/local_setup.bash
EVAL=/sim_ws/src/oudtra_driver_bringup/config/localization_eval.yaml
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  use_sim_time:=false \
  path_platform_config:=$EVAL \
  reactive_platform_config:=$EVAL \
  arbitration_platform_config:=$EVAL \
  integration_platform_config:=$EVAL
```

This overlay keeps the bounded transform wait, routes lower safety to
`/pf/pose/odom`, requires PF health in arbitration, and disables the simulator-GT
reverse gate. Confirm motion from `/drive` and `/ego_racecar/odom`; stop PnC
first, then PF and simulator. The dedicated runner below remains the preferred
repeatable path because it performs readiness and cleanup automatically.

## 5. Onboard/default command placeholder

Onboard operation uses the same full-stack launch family with `platform:=onboard`
and onboard configuration paths. Add verified vehicle-computer commands here when
that workflow is updated; do not copy simulator overlays or GT-isolation settings
into onboard instructions.

## 6. Essential regression checks

After rebuilding affected packages and sourcing again:

```bash
cd /sim_ws
python3 -m pytest -q \
  src/particle_filter/test/test_launch_time_contracts.py \
  src/oudtra_driver_bringup/test
```

The checks cover PF launch-time defaults, evaluation-profile isolation,
initialization inputs, metric alignment, missing data, and existing full-stack
launch contracts. All tests must pass; this command does not replace a runtime
simulation check.

## 7. Closed-loop localization smoke check

This command starts simulator, PF, and the explicit evaluation-profile PnC stack,
initializes PF once, checks physical motion, and cleans up its own processes:

```bash
cd /sim_ws
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --sim-config /sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml \
  --run-duration-sec 15 \
  --result-json /tmp/localization_smoke.json
```

PASS means exit 0 and JSON `"result": "PASS"`. Use an unused ROS domain with
`--ros-domain-id` when another experiment is active. This runner already owns
the launch sequence; do not start the manual terminals above in the same domain.

## 8. Record and analyze an evaluation run

From a **host terminal**, preserve repository revisions when opening the existing
container. The revisions identify source history; they do not prove installed
binaries are current.

```bash
cd /home/mzhou/f1tenth_dev
git status --short
docker exec -it \
  -e ROBORACER_MAIN_REV="$(git rev-parse HEAD)" \
  -e ROBORACER_PF_REV="$(git -C particle_filter rev-parse HEAD)" \
  f1tenth_gym_ros_rocker bash
```

Inside that container shell, build/source the analyzer package and use a descriptive
run ID. Set `--source-state` to `clean` or `dirty` from the host status when known;
`unknown` remains explicit.

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select oudtra_driver_bringup
source install/local_setup.bash
RUN_DIR="/sim_ws/src/oudtra_driver_bringup/runs/ifac_pf_closed_loop_localization_baseline_$(date -u +%Y%m%dT%H%M%SZ)"
```

Run and analyze it after all simulation processes stop:

```bash
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --sim-config /sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml \
  --run-duration-sec 15 \
  --record-dir "$RUN_DIR" \
  --source-revision "$ROBORACER_MAIN_REV" \
  --pf-revision "$ROBORACER_PF_REV" \
  --source-state unknown \
  --result-json /tmp/localization_smoke_recorded.json

ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN_DIR"
cat "$RUN_DIR/report.md"
cat "$RUN_DIR/metrics.json"
```

The run directory contains `metadata.yaml`, `smoke_result.json`, `rosbag/`,
`config/`, `logs/`, `metrics.json`, `report.md`, and `plots/`. The host-visible
equivalent is `/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/`.

For a simple performance matrix, repeat with one descriptive run directory per
scenario, such as `..._offset0`, `..._offset_x02m`, or `..._duration30s`.
Compare the generated `metrics.json` files offline. This is measurement
collection only; no thresholds or automated ranking are defined yet.

To verify reproducibility, analyze the same run twice and compare hashes:

```bash
sha256sum "$RUN_DIR/metrics.json" "$RUN_DIR/report.md"
ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN_DIR"
sha256sum "$RUN_DIR/metrics.json" "$RUN_DIR/report.md"
```

## 9. Open run deliverables

Open the human report and plots from the host after analysis:

```text
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/report.md
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/metrics.json
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/trajectory_xy.png
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/position_error.png
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/reference_trajectory.png
```

`report.md` is the concise human scorecard; `metrics.json` is the machine result;
`plots/` contains the figures; `rosbag/`, `metadata.yaml`, `config/`, and `logs/`
are the raw evidence and provenance. Run datasets are ignored by Git and should
be archived or shared explicitly when needed.

## 10. Compare two analyzed runs

After both run directories report analysis PASS, use a sourced canonical container;
no simulator or live ROS graph is needed:

```bash
ros2 run oudtra_driver_bringup compare_localization_runs.py \
  /sim_ws/src/oudtra_driver_bringup/runs/<sim-run> \
  /sim_ws/src/oudtra_driver_bringup/runs/<onboard-run> \
  --output-dir /sim_ws/src/oudtra_driver_bringup/runs/<descriptive-comparison-name>
```

Replace the angle-bracket paths with real analyzed runs. Comparison requires no
onboard recorder; that workflow remains future work. Unknown identity or incompatible
evidence suppresses deltas instead of suggesting equivalent measurements.

Open `report.md` beside `metrics.json` in each run; plots are linked under `plots/`.
Comparison writes `report.md` and `comparison.json` in its separate output directory.
Host equivalents replace `/sim_ws/src/` with `/home/mzhou/f1tenth_dev/`. Current
simulation and **synthetic no-GT validation** artifacts are listed in the
[deliverables section](LOCALIZATION_SIMULATION.md#56-plots-and-validation-artifacts).
