# RoboRacer operational command reference

## 1. How to use this document

**Purpose:** run, record and inspect supported RoboRacer workflows using exact
terminal commands. **Start here:** the human-confirmed activity is A → B → C below;
it is an operator handoff, not a development roadmap. Simulation uses the existing
Foxy `f1tenth_gym_ros_rocker` localization-ready container; onboard uses its own
Foxy workspace and requires human-qualified hardware/assets.

### Current operator workflow

| Activity / readiness | Entry point | Expected output | Boundary |
| --- | --- | --- | --- |
| **A. Repeat Tempo simulation — executable, explicitly experimental** | [Complete host/container procedure](#12-tempo-coffee-experimental-pfpnc-and-low-speed-comparison) | Recorded run, smoke result, offline scorecard and plots | Requires explicit local manifest selection; two 30 s straight-start passes, not production/hardware qualification |
| **B. Prepare onboard data — conditional / real-car recorder unverified** | [Qualification, Terminals 1–4, recording and transfer](#5-onboard-stack-and-passive-measurement) | Preflight evidence and preserved onboard bag/metadata | Tempo autonomy is **BLOCKED**: failed raceline gate, no hardware qualification, unresolved onboard PF map selection |
| **C. Analyze and compare — executable once datasets exist** | [Offline two-run procedure](#10-compare-two-analyzed-runs) | Two scorecards plus comparison Markdown/JSON | No real onboard comparison yet; unknown identity or incompatible evidence suppresses deltas |

**Other workflows:** [container access](#2-open-the-canonical-container) ·
[perfect-localization reference](#3-perfect-localization-simulator-and-full-stack) ·
[manual IFAC PF/PnC](#4-pf-localization-simulator-and-manual-full-stack) ·
[regression checks](#6-essential-regression-checks) ·
[IFAC smoke](#7-closed-loop-localization-smoke-check) ·
[IFAC recorded baseline](#8-record-and-analyze-an-evaluation-run) ·
[deliverable locations](#9-open-run-deliverables) ·
[raceline workbench](#11-offline-raceline-workbench).

The automated runner starts its own simulator, PF and PnC; do not combine it with
manual launch terminals in the same ROS domain. This document owns copyable
commands. [Environment](DEVELOPMENT_ENVIRONMENT.md) owns host/container facts,
[evaluation guide](LOCALIZATION_SIMULATION.md) owns evidence/metrics, and
[project record](PROJECT_RECORD.md) owns demonstrated status and limitations.

<!-- table-of-contents:start -->
**Table of contents**

- [1. How to use this document](#1-how-to-use-this-document)
  - [Current operator workflow](#current-operator-workflow)
- [2. Open the canonical container](#2-open-the-canonical-container)
- [3. Perfect-localization simulator and full stack](#3-perfect-localization-simulator-and-full-stack)
- [4. PF localization simulator and manual full stack](#4-pf-localization-simulator-and-manual-full-stack)
- [5. Onboard stack and passive measurement](#5-onboard-stack-and-passive-measurement)
  - [Goal and prerequisites](#goal-and-prerequisites)
  - [Onboard host / container checks](#onboard-host--container-checks)
  - [Human-qualified scenario: existing Terminals 1–3](#human-qualified-scenario-existing-terminals-13)
  - [Terminal 4: preflight, record, finalize](#terminal-4-preflight-record-finalize)
  - [Transfer to laptop and analyze](#transfer-to-laptop-and-analyze)
- [6. Essential regression checks](#6-essential-regression-checks)
- [7. Closed-loop localization smoke check](#7-closed-loop-localization-smoke-check)
- [8. Record and analyze an evaluation run](#8-record-and-analyze-an-evaluation-run)
- [9. Open run deliverables](#9-open-run-deliverables)
- [10. Compare two analyzed runs](#10-compare-two-analyzed-runs)
- [11. Offline raceline workbench](#11-offline-raceline-workbench)
  - [Open GUI and generate](#open-gui-and-generate)
  - [Reopen a session](#reopen-a-session)
  - [Headless generation and explicit export](#headless-generation-and-explicit-export)
  - [Verify selection and hand off to existing interfaces](#verify-selection-and-hand-off-to-existing-interfaces)
  - [Deliverables and essential checks](#deliverables-and-essential-checks)
- [12. Tempo Coffee experimental PF/PnC and low-speed comparison](#12-tempo-coffee-experimental-pfpnc-and-low-speed-comparison)
  - [Goal and prerequisites](#goal-and-prerequisites-1)
  - [Host terminal: verify and enter the existing container](#host-terminal-verify-and-enter-the-existing-container)
  - [Simulation container: build, choose assets and verify selection](#simulation-container-build-choose-assets-and-verify-selection)
  - [Simulation container: run, stop, analyze](#simulation-container-run-stop-analyze)
  - [Optional monitoring terminal](#optional-monitoring-terminal)
  - [Troubleshooting and evidence to retain](#troubleshooting-and-evidence-to-retain)
<!-- table-of-contents:end -->

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
  localize_config:=/sim_ws/src/particle_filter/config/localize_sim.yaml \
  map_yaml:=/sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml \
  parameter_overlay:=/sim_ws/src/oudtra_driver_bringup/config/localization_eval_sim.yaml
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
EVAL_SIM=/sim_ws/src/oudtra_driver_bringup/config/localization_eval_sim.yaml
RACELINE=/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  use_sim_time:=false \
  raceline_csv_path:=$RACELINE \
  raceline_direction:=csv \
  path_platform_config:=$EVAL_SIM \
  reactive_platform_config:=$EVAL_SIM \
  arbitration_platform_config:=$EVAL_SIM \
  integration_platform_config:=$EVAL_SIM
```

This overlay keeps the bounded transform wait, routes lower safety to
`/pf/pose/odom`, requires PF health in arbitration, and disables the simulator-GT
reverse gate. Confirm motion from `/drive` and `/ego_racecar/odom`; stop PnC
first, then PF and simulator. The dedicated runner below remains the preferred
repeatable path because it performs readiness and cleanup automatically.

## 5. Onboard stack and passive measurement

### Goal and prerequisites

Collect an onboard dataset in an additional passive Terminal 4, without changing
the vehicle team's operating process. **Conditional:** the recorder has test/mock
evidence, not physical-car validation. A human must confirm the qualified map,
raceline/direction, platform profile and operating authorization first.
**Tempo autonomy is BLOCKED / requires human qualification and implementation
verification.** Its current experimental path fails the production curvature gate.
Do not use simulation passes as hardware approval.

The pinned `localize_onboard_rviz_launch.py` includes `localize_launch.py`, which
reads the installed default `localize.yaml` map *before* resolving a custom
`localize_config`. A custom PF config alone therefore does not select a new map.
The current default names IFAC; this is implementation evidence, not proof IFAC is
qualified on your car. Do not work around this with simulator frames or a simulation
launch. For a new track, stop until explicit onboard map selection is verified.

### Onboard host / container checks

On the **onboard host**, check the deployed checkout and submodules:

```bash
cd ~/f1tenth_ws/src/f1tenth_dev
git branch --show-current
git rev-parse HEAD
git status --short
git submodule status
git -C particle_filter rev-parse HEAD
git -C ../f1tenth_system rev-parse HEAD
```

Compare with the intended deployment revision (this procedure was inspected at
`050d9ab`, PF `dfd07cb`) and record differences/dirty state. Enter the existing
onboard container using the vehicle's established access procedure; its name is
not established here. In **every onboard container terminal**:

```bash
source /opt/ros/foxy/setup.bash
cd /f1tenth_ws
source install/setup.bash
REPO=/f1tenth_ws/src/f1tenth_dev
ros2 pkg prefix particle_filter
ros2 pkg prefix oudtra_driver_bringup
```

Before vehicle operation, verify installed launch/config/script files against the
intended host revision. For example, these pairs should match:

```bash
PF_SHARE="$(ros2 pkg prefix particle_filter)/share/particle_filter"
BRINGUP_PREFIX="$(ros2 pkg prefix oudtra_driver_bringup)"
sha256sum "$REPO/particle_filter/launch/localize_launch.py" \
  "$PF_SHARE/launch/localize_launch.py"
sha256sum "$REPO/particle_filter/config/localize.yaml" \
  "$PF_SHARE/config/localize.yaml"
sha256sum "$REPO/oudtra_driver_bringup/scripts/record_onboard_localization.py" \
  "$BRINGUP_PREFIX/lib/oudtra_driver_bringup/record_onboard_localization.py"
```

A source revision alone does not prove C++ binaries are current. If deployment,
build/source state or hashes are inconsistent, **BLOCKED** until verified through
the established deployment process. Do not rebuild or change parameters on an
active vehicle merely to pass a check.

### Human-qualified scenario: existing Terminals 1–3

Only after qualification, use the established workflow:

```text
Terminal 1: ros2 launch f1tenth_stack bringup_launch.py
Terminal 2: ros2 launch particle_filter localize_onboard_rviz_launch.py
Terminal 3: ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

Initialize PF manually with RViz `2D Pose Estimate`; verify fresh localization
and TF under the normal operating process before enabling autonomy. The recorder
never initializes PF, enables autonomy, publishes drive, or changes parameters.

In **Terminal 4**, define *the actual human-qualified assets*. These IFAC paths
are an example for the current default, not authorization for that scenario:

```bash
MAP_YAML="$REPO/particle_filter/maps/ifac_roboracer.yaml"
MAP_IMAGE="$REPO/particle_filter/maps/ifac_roboracer.png"
RACELINE="$REPO/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv"
DIRECTION=csv
SPEED_PROFILE="$REPO/oudtra_driver_bringup/config/comparison_speed_limits.yaml"
sha256sum "$MAP_YAML" "$MAP_IMAGE" "$RACELINE" "$SPEED_PROFILE"
ros2 param get /map_server yaml_filename
timeout 5s ros2 topic echo /map nav_msgs/msg/OccupancyGrid --no-arr \
  --qos-reliability reliable --qos-durability transient_local
ros2 param get /raceline_publisher csv_path
ros2 param get /raceline_publisher direction
ros2 param get /particle_filter use_sim_time
ros2 param get /particle_filter base_frame
ros2 param get /particle_filter laser_frame
ros2 param get /lower_safety_controller enable_sim_reverse_swept_gate
ros2 param get /lower_safety_controller odom_topic
ros2 param get /drive_arbitrator require_pf_health
```

The bounded map echo displays metadata without the large occupancy array; its
normal timeout exit 124 is not a stack failure. Foxy does not support newer
`echo --once --field` flags. The loaded map must match `MAP_YAML`, image geometry/origin and the qualified track;
publisher path/direction must match the qualified CSV. Expect wall time, onboard
frames/odometry, simulator-GT gate false and PF-health gate true. A mismatch is
**BLOCKED**, not permission to change live parameters. Confirm the mux/actuator
command path under the existing operating procedure: `/drive` is autonomy output,
while the recorder's final plant-input role is mux output `/ackermann_cmd`.

If the human approves the shared 0.5–2.0 m/s comparison profile for this qualified
scenario, replace **Terminal 3**, rather than starting another stack, with:

```bash
# In sourced onboard Terminal 3, after human scenario/profile qualification:
REPO=/f1tenth_ws/src/f1tenth_dev
read -r -p 'Qualified raceline absolute CSV path: ' RACELINE
read -r -p 'Qualified direction (csv/normal/reverse): ' DIRECTION
SPEED_PROFILE="$REPO/oudtra_driver_bringup/config/comparison_speed_limits.yaml"
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py \
  raceline_csv_path:="$RACELINE" raceline_direction:="$DIRECTION" \
  path_platform_config:="$SPEED_PROFILE" \
  reactive_platform_config:="$SPEED_PROFILE" \
  integration_platform_config:="$SPEED_PROFILE"
```

This profile contains speed limits only; it does not qualify the raceline or set
onboard map/frames. Do not substitute `localization_eval_sim.yaml`. Confirm min/max
on `/path_generator`, `/local_trajectory_planner`, `/path_following_v2` and the
absolute cap on `/lower_safety_controller`. Stops/slower caps and reverse remain
possible; joystick/mux authority is a separate operating concern.

### Terminal 4: preflight, record, finalize

Run the passive preflight before the authorized measurement:

```bash
ros2 run oudtra_driver_bringup record_onboard_localization.py \
  --preflight-only --preflight-json /tmp/onboard_recorder_preflight.json
```

Exit 0 / preflight PASS requires observed messages with required types. Optional
missing evidence is reported. Failure means **BLOCKED** until the real interface
or deployed contract is understood; never silently record a different topic.

For the confirmed scenario/configuration, keep the exact file list. Include
`--config speed=...` only if that profile is actually loaded; otherwise retain the
actual approved platform overlays as additional `--config LABEL=PATH` arguments.

```bash
RUN_DIR="/f1tenth_ws/onboard_runs/qualified_pf_pnc_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run oudtra_driver_bringup record_onboard_localization.py "$RUN_DIR" \
  --map-yaml "$MAP_YAML" --map-image "$MAP_IMAGE" --raceline-csv "$RACELINE" \
  --config pf="$REPO/particle_filter/config/localize.yaml" \
  --config pnc="$REPO/oudtra_driver_bringup/config/full_stack.yaml" \
  --config path="$REPO/path_following_v2/config/path_following_v2.yaml" \
  --config reactive="$REPO/reactive_control_v2/config/reactive_control_v2.yaml" \
  --config arbitration="$REPO/drive_arbitration_v2/config/drive_arbitration_v2.yaml" \
  --config speed="$SPEED_PROFILE" \
  --config mux=/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/mux.yaml \
  --config vesc=/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
```

**Expected output:** startup evidence is recorded, then the tool waits for Enter.
Press Enter once to mark evaluation start at the human-authorized activity. Press
Ctrl-C once at its end; the recorder marks evaluation end, finalizes rosbag, retains
0.5 s post-roll and updates `metadata.yaml`. Exit 0 plus recording PASS is a recorder
result, not vehicle-performance acceptance. Existing directories are rejected.

**Stop/cleanup:** stopping this passive tool does **not** stop the car. Stop/disarm
the vehicle under the team's operating procedure, finalize the recorder, then stop
owned stack launches as appropriate. An interrupted preflight/run may leave failure
metadata; retain it, do not rename it into a successful dataset.

**Troubleshooting/evidence:** retain preflight JSON, map/publisher parameter output,
actual direction/profile, revisions, `metadata.yaml`, `rosbag/`, `config/`, `logs/`
and all referenced asset/config files. The onboard recorder hashes map/raceline/
configs but does not copy every supplied file or automatically declare scenario,
direction or comparison identity. Keep originals alongside the archived dataset
and a factual operator note; do not fabricate identity to unlock deltas.

After recorder finalization, keep actual assets and a direction/profile note with
this run (verify their hashes against `metadata.yaml` before archiving). For example,
when the declared low-speed profile was used:

```bash
mkdir -p "$RUN_DIR/operator_assets"
cp "$MAP_YAML" "$MAP_IMAGE" "$RACELINE" "$SPEED_PROFILE" "$RUN_DIR/operator_assets/"
printf 'raceline_direction=%s\nmap_yaml=%s\nraceline_csv=%s\nspeed_profile=%s\n' \
  "$DIRECTION" "$MAP_YAML" "$RACELINE" "$SPEED_PROFILE" > "$RUN_DIR/operator_notes.txt"
```

Also archive the exact PF/PnC/mux/VESC configuration files declared above; copied
files must still match the run's recorded hashes. These notes are evidence, not a
substitute for comparison identity or hardware qualification.

### Transfer to laptop and analyze

Confirm the onboard bind mount exposes `/f1tenth_ws/onboard_runs/` as
`~/f1tenth_ws/onboard_runs/`; otherwise use its actual host path. From the **dev host**:

```bash
# Replace these values with the actual vehicle account, hostname and run ID.
VEHICLE='<vehicle-user>@<vehicle-host>'
RUN_ID='<actual-run-directory-name>'
rsync -a "$VEHICLE:~/f1tenth_ws/onboard_runs/$RUN_ID/" \
  "/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/$RUN_ID/"
```

Transfer the complete directory and retained input assets/notes, not just plots or
`metrics.json`. Inside the **sourced simulation/analysis container**:

```bash
ONBOARD_RUN="/sim_ws/src/oudtra_driver_bringup/runs/$RUN_ID"
ros2 run oudtra_driver_bringup analyze_localization_run.py "$ONBOARD_RUN"
```

Set `RUN_ID` there explicitly (host shell variables are not inherited). Open the
host-visible `report.md`, `metrics.json`, `plots/`; absolute localization accuracy
without GT is NOT_APPLICABLE. Continue with [comparison](#10-compare-two-analyzed-runs).

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
SCENARIO_ID=ifac_roboracer
SIM_CONFIG=/sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml
MAP_YAML=/sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml
RACELINE=/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
LOCALIZE_CONFIG=/sim_ws/src/particle_filter/config/localize_sim.yaml
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --scenario-id "$SCENARIO_ID" \
  --sim-config "$SIM_CONFIG" \
  --pf-map-yaml "$MAP_YAML" \
  --raceline-csv "$RACELINE" \
  --raceline-direction csv \
  --localize-config "$LOCALIZE_CONFIG" \
  --run-duration-sec 15 \
  --result-json /tmp/localization_smoke.json
```

PASS means exit 0 and JSON `"result": "PASS"`. Use an unused ROS domain with
`--ros-domain-id` when another experiment is active. This runner already owns
the launch sequence; do not start the manual terminals above in the same domain.
It is headless by default and requires no display. Add `--with-rviz` only for
interactive observation; RViz appearance is not benchmark evidence.

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

Inside that container shell, build/source the runner and its dependency closure,
then use a descriptive run ID. Set `--source-state` to `clean` or `dirty` from the
host status when known; `unknown` remains explicit.

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-up-to oudtra_driver_bringup
source install/local_setup.bash
RUN_DIR="/sim_ws/src/oudtra_driver_bringup/runs/ifac_pf_closed_loop_localization_baseline_$(date -u +%Y%m%dT%H%M%SZ)"
SCENARIO_ID=ifac_roboracer
SIM_CONFIG=/sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml
MAP_YAML=/sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml
RACELINE=/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
LOCALIZE_CONFIG=/sim_ws/src/particle_filter/config/localize_sim.yaml
```

Run and analyze it after all simulation processes stop:

```bash
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --scenario-id "$SCENARIO_ID" \
  --sim-config "$SIM_CONFIG" \
  --pf-map-yaml "$MAP_YAML" \
  --raceline-csv "$RACELINE" \
  --raceline-direction csv \
  --localize-config "$LOCALIZE_CONFIG" \
  --run-duration-sec 15 \
  --record-dir "$RUN_DIR" \
  --source-revision "$ROBORACER_MAIN_REV" \
  --pf-revision "$ROBORACER_PF_REV" \
  --source-state unknown \
  --result-json /tmp/localization_smoke_recorded.json

ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN_DIR"
ros2 run oudtra_driver_bringup analyze_sim_timing.py "$RUN_DIR"
cat "$RUN_DIR/report.md"
cat "$RUN_DIR/metrics.json"
```

The run directory contains `metadata.yaml`, `smoke_result.json`, `rosbag/`,
`config/`, `logs/`, `metrics.json`, `timing.json`, `report.md`, and `plots/`. The
host-visible equivalent is
`/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/`.

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

To add a new track, copy/add its map image and YAML, generate and validate
`raceline_points_optimized.csv`, and create a simulator config with the track map
path and start pose. Set the five variables above to the new scenario and assets;
the runner preflight checks map agreement and raceline geometry before launch.
The evaluation integration overlay remains unchanged.

## 9. Open run deliverables

Open the human report and plots from the host after analysis:

```text
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/report.md
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/metrics.json
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/trajectory_xy.png
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/position_error.png
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/reference_trajectory.png
/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<descriptive-run-id>/plots/smoothness_oscillations.png
```

`report.md` is the concise human scorecard; `metrics.json` is the machine result;
`plots/` contains the figures; `rosbag/`, `metadata.yaml`, `config/`, and `logs/`
are the raw evidence and provenance. Run datasets are ignored by Git and should
be archived or shared explicitly when needed.

## 10. Compare two analyzed runs

**Goal:** inspect simulation and onboard results side by side, offline.
**Preconditions:** the two preserved directories exist, both have successful
schema-2 analysis, and the operator has checked scenario/assets/configuration.
A successful comparison command does not establish sim/onboard equivalence.

In the **sourced simulation/analysis container**, explicitly select real directories:

```bash
SIM_RUN='/sim_ws/src/oudtra_driver_bringup/runs/<actual-simulation-run>'
ONBOARD_RUN='/sim_ws/src/oudtra_driver_bringup/runs/<actual-copied-onboard-run>'
COMPARISON_DIR="/sim_ws/src/oudtra_driver_bringup/runs/sim_vs_onboard_$(date -u +%Y%m%dT%H%M%SZ)"
# Reanalysis is needed if metrics.json is absent, older-schema, or analyzer changed.
ros2 run oudtra_driver_bringup analyze_localization_run.py "$SIM_RUN"
ros2 run oudtra_driver_bringup analyze_localization_run.py "$ONBOARD_RUN"
ros2 run oudtra_driver_bringup compare_localization_runs.py \
  "$SIM_RUN" "$ONBOARD_RUN" --output-dir "$COMPARISON_DIR"
```

Replace placeholders before copying commands. Keep the comparison directory
separate from either input run; no simulator, replay or live ROS graph is needed.

**Expected output:** `report.md` and `comparison.json` in `COMPARISON_DIR`. On the
host, replace `/sim_ws/src/` with `/home/mzhou/f1tenth_dev/`. Read each run's report,
plots and `metrics.json` alongside the comparison, especially applicability and
per-row reasons for suppressed deltas. Retain the two complete run directories,
comparison output, configs/assets, hashes and operator notes together.

**Comparability checks:** inspect simulation `metadata.yaml` `track.assets` /
`selected_raceline` / `configs`, onboard `assets` / `configs` / repository provenance,
and both `metrics.json` identity/settings. Compare actual map YAML/image and CSV
hashes, direction, speed limits, PF configuration, timing/evaluation intervals and
analyzer definitions. Inspect speed profile snapshots and runtime parameter evidence,
not filenames alone. The tool checks declared identity/settings and metric evidence;
it does not automatically enforce every map/direction/speed qualification. Current
onboard metadata does not populate all comparison identity fields, so unknown
identity will suppress deltas. Do not edit metadata to manufacture comparability.

| Result family | Interpretation across platforms |
| --- | --- |
| Availability, continuity, command smoothness | Potentially common when method, evidence, settings, timing and coverage agree; inspect row reasons |
| Absolute position/yaw accuracy | Simulation GT only; onboard without independent GT is NOT_APPLICABLE, never zero |
| Tracking / pace | Sim GT and onboard estimated-pose evidence differ; side-by-side observations are not truth-equivalent deltas |
| Forward-speed / yaw-rate response | Qualified adapters required; measured onboard IMU yaw-rate support remains incomplete |
| Robustness | Collision absence needs qualified evidence; counts depend on observation coverage and interval |

**Stop/cleanup:** analysis is offline; do not launch a vehicle to run it. Preserve
original bags/metadata when regenerating outputs. **Troubleshooting:** missing/
malformed data may yield UNAVAILABLE_DATA or ANALYSIS_ERROR. Analysis PASS establishes
evidence processing, not successful driving. The pipeline has no matched-section
alignment, automatic hardware qualification or causal platform score. See the
[formal comparison contract](LOCALIZATION_SIMULATION.md#54-compare-completed-runs).

## 11. Offline raceline workbench

**Race-day:** open the existing container ([section 2](#2-open-the-canonical-container)),
then run the map pair. A working display opens the GUI; no special GUI flag is needed.
The workbench only generates/selects files: it never launches a simulator or vehicle.
[Package contract](../centerline_tools/README.md#2-workbench-contract) owns methods/limits.

### Open GUI and generate

```bash
cd /sim_ws/src/centerline_tools
python3 raceline_workbench.py \
  tempo_coffee_scanned_20261002_00.pgm tempo_coffee_scanned_20261002_00.yaml
```

Each invocation creates a fresh, descriptive session and shows the most recent
verified export for the same map hashes as a labelled preview. **Open Previous
Session** resumes it explicitly; preview alone never changes downstream selection.
Wheel-zoom at the map cursor; toolbar pan/zoom and **Fit Map** are available.
Click **Generate Raceline**,
then **Try More Settings** if needed; review candidates, try presets/advanced values
or **Edit Path Manually**. **Preset Values** shows exact settings for every choice.
Advanced numerical controls support dragging and typed values; **Apply Settings
& Close** stages trial settings, Cancel discards the draft. **Show Candidate** is
preview only; **Check File** validates without exporting. Explicit clearance/curvature
edits require confirmation and create a new session/profile with no inherited
validation or selection; generate/validate again. Auto-search never changes limits.
Custom-profile PASS does not establish real vehicle steering or onboard suitability.
Use **Finish / Exit** for saved-file/status confirmation; **Save Validated & Exit**
exports/selects a passing candidate and closes, or exit preserving current files.
Tempo is a difficult failure fixture, not an accepted track. For the known successful
IFAC fixture use its explicitly reviewed profile:

```bash
python3 raceline_workbench.py \
  /sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.png \
  /sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer.yaml \
  --config config/raceline_workbench_ifac.yaml
```

### Reopen a session

Use the exact SESSION directory printed by the console/displayed in the GUI.
A bare `--open-session` opens a file picker at the session home; choose its report.
For explicit reopening (also works headless):

```bash
WORKBENCH_DIR='/absolute/session/directory/printed/by/the/workbench'
python3 raceline_workbench.py --open-session "$WORKBENCH_DIR"
```

Maps/config are recovered and hash-checked; do not resupply them. A changed solver
profile or stale map/centerline requires a new session. Existing compatible saved
manual E/A/X edits remain resumable. Closing the editor without Save cannot select
an old edit or substitute the centerline.

### Headless generation and explicit export

```bash
python3 raceline_workbench.py \
  tempo_coffee_scanned_20261002_00.pgm tempo_coffee_scanned_20261002_00.yaml \
  --headless --auto-search
```

Exit `0` means a validated output or deliberate experimental export exists; it is
**not** a production-validity verdict. Exit `1` means no usable export, `2` an input/
action error. Search exhaustion means no valid candidate in the bounded schedule,
not physical infeasibility. Read the YAML/status and preview before selection.

In the GUI use **Select / Export Validated**, or **Export Experimental** followed by
its failed-check confirmation. Headless selection is equally explicit:

```bash
python3 raceline_workbench.py --open-session "$WORKBENCH_DIR" \
  --headless --select-validated best
# Only after reviewing that specific failed candidate's diagnostic report:
python3 raceline_workbench.py --open-session "$WORKBENCH_DIR" \
  --headless --export-experimental 008_geometry
```

Replace the example ID with the intended candidate. `--export-experimental` itself
is the explicit CLI acknowledgment. It writes exact candidate geometry and failed
checks to experimental files; it cannot create a validated output. Reference-only
previews are rejected. Export never starts or enables vehicle control.

### Verify selection and hand off to existing interfaces

```bash
SELECTED="$WORKBENCH_DIR/selected_raceline.yaml"
python3 resolve_selected_raceline.py "$SELECTED"
# For deliberate experimental simulation only:
python3 resolve_selected_raceline.py "$SELECTED" --allow-experimental
```

The helper prints the absolute CSV, status, candidate, direction and manifest.
Missing/changed CSV, map or validation evidence fails. Add `--map-yaml "$MAP_YAML"`
to verify caller-map identity. `--format json` is machine-readable; `--format pnc`
prints the existing `raceline_csv_path:=... raceline_direction:=...` arguments;
`--format publisher` prints `csv_path:=... direction:=...`. These print arguments
without launching anything. Use the **validated-only** resolver (no acknowledgment)
for the recommended onboard handoff to the existing [Terminal 3](#5-onboard-stack-and-passive-measurement).
Do not use an experimental path as an onboard reference. A custom offline vehicle
profile also needs independent physical-envelope review; changing an analysis
limit does not change hardware or authorize onboard use.

For simulation, build/source the affected packages first. Use the same explicit
scenario/simulator/map/PF variables from [section 7](#7-closed-loop-localization-smoke-check)
or [section 8](#8-record-and-analyze-an-evaluation-run). Substitute the manifest
for the CSV/direction pair; the runner still performs map/scenario preflight:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select centerline_tools oudtra_driver_bringup
source install/local_setup.bash
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --scenario-id "$SCENARIO_ID" --sim-config "$SIM_CONFIG" \
  --pf-map-yaml "$MAP_YAML" --localize-config "$LOCALIZE_CONFIG" \
  --selected-raceline "$SELECTED" \
  --run-duration-sec 15 --result-json /tmp/selected_raceline_smoke.json
```

Add `--allow-experimental` **only** for an intentionally selected experimental
simulation. The tool will reject it otherwise. Never change the explicit map to
silence a mismatch. Add the recording/provenance arguments from section 8 to
preserve a run; metadata retains the manifest hash, validation status and resolved
CSV/hash, and copies `selected_raceline.yaml` / `selected_raceline_validation.yaml`.
The existing config snapshot retains the actual reference CSV.

**Which CSV is active?** The resolver shows selection before launch; the runner's
result JSON records `selected_raceline` and `track.assets.raceline_csv`, and full-stack
startup logs print the path/direction. Inspect the running publisher if needed:

```bash
ros2 param get /raceline_publisher csv_path
ros2 param get /raceline_publisher direction
```

### Deliverables and essential checks

Container session home: `/sim_ws/src/centerline_tools/centerline_output/workbench/`.
Host home: `/home/mzhou/f1tenth_dev/centerline_tools/centerline_output/workbench/`.
The console prints the exact session directory. Open `preview.png` for map/curvature,
`raceline_workbench_report.yaml` for attempts/limits, and `selected_raceline.yaml`
for the active handoff. Validated files are `raceline_points_optimized.csv` and
`raceline_validation.yaml`; experimental files are `raceline_points_experimental.csv`
and `experimental_validation.yaml`. Raw candidate/manual files are never implicitly
selected. Detailed generation/manual logs and candidate diagnostics stay in session.

Local V2 examples are `ifac_race_day_v2_validated_20261011` and
`tempo_coffee_race_day_v2_search_20261011`. They are ignored offline validation data,
not guaranteed downloadable assets. The IFAC selection also passed a 10 s recorded
closed-loop handoff in `oudtra_driver_bringup/runs/ifac_selected_raceline_v2_handoff_20261011/`;
open its `report.md`, `smoke_result.json`, `metadata.yaml`, `selected_raceline.yaml`,
`selected_raceline_validation.yaml`, `config/reference_csv.csv` and `plots/`. Tempo
experimental export was not driven. Previous V1 examples remain intact. All source paths in manifests must be reachable in the
consuming environment; container paths refer to the host bind mounts.

```bash
cd /sim_ws/src/centerline_tools
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest test/ -q -p no:cacheprovider
```

Use `PYTHONDONTWRITEBYTECODE=1` to avoid modifying retained tracked bytecode.
GUI usability still needs human operator review; display-free operation uses
`--headless` and saved previews.


## 12. Tempo Coffee experimental PF/PnC and low-speed comparison

### Goal and prerequisites

Repeat the established bounded closed loop: simulator → PF → one-shot initialization
→ current planning/control → final command → simulator, with passive recording and
offline analysis. **Executable, simulation-only experimental workflow.** The selected
`008_geometry` CSV is corridor-safe but exceeds the 1.082 1/m production curvature
limit (peak≈1.460). Two 30 s straight-start runs passed; the first-row start failed
with wrong PF heading/reverse recovery. No hardware qualification follows.

Prerequisites: the expected host checkout, existing running localization-ready
container, current installed dependencies, explicit local selection manifest and
an unused ROS domain. The runner starts all stack processes automatically; do not
start additional simulator/PF/PnC nodes in its domain.

### Host terminal: verify and enter the existing container

```bash
cd /home/mzhou/f1tenth_dev
git branch --show-current
git rev-parse HEAD
git status --short
git submodule status
./scripts/rr_container.sh status
if [ -z "$(git status --porcelain)" ]; then RR_SOURCE_STATE=clean; else RR_SOURCE_STATE=dirty; fi
docker exec -it \
  -e ROBORACER_MAIN_REV="$(git rev-parse HEAD)" \
  -e ROBORACER_PF_REV="$(git -C particle_filter rev-parse HEAD)" \
  -e ROBORACER_SOURCE_STATE="$RR_SOURCE_STATE" \
  f1tenth_gym_ros_rocker bash
```

Expected branch: `mingguang_localization_robustness_ai_workspace`; this procedure
was inspected at `050d9ab`. If checkout/container differs, understand that difference
before running. This command enters the existing container; it does not create or
replace it. A display is needed only for the RViz choice below.

### Simulation container: build, choose assets and verify selection

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros particle_filter centerline_tools path_following_v2 reactive_control_v2 drive_arbitration_v2 oudtra_driver_bringup
source install/local_setup.bash
SCENARIO_ID=tempo_coffee_scanned_20261002_00
SIM_CONFIG=/sim_ws/src/f1tenth_gym_ros/config/sim_tempo_coffee.yaml
MAP_YAML=/sim_ws/src/centerline_tools/tempo_coffee_scanned_20261002_00.yaml
LOCALIZE_CONFIG=/sim_ws/src/particle_filter/config/localize_sim.yaml
SPEED_PROFILE=/sim_ws/src/oudtra_driver_bringup/config/comparison_speed_limits.yaml
```

Discover saved manifests for this map; listing does not select a hidden latest file:

```bash
python3 - <<'PYTHON'
from pathlib import Path
import yaml
expected = Path('/sim_ws/src/centerline_tools/tempo_coffee_scanned_20261002_00.yaml')
for manifest in sorted(Path('/sim_ws/src/centerline_tools/centerline_output/workbench').glob('*/selected_raceline.yaml')):
    selected = yaml.safe_load(manifest.read_text())['selection']
    if Path(selected['map_yaml']).resolve() == expected.resolve():
        print(selected['status'], selected['candidate_id'], selected['raceline_csv_sha256'])
        print(manifest)
PYTHON
read -r -p 'Paste the intended manifest absolute path: ' SELECTED
python3 /sim_ws/src/centerline_tools/resolve_selected_raceline.py \
  "$SELECTED" --allow-experimental --map-yaml "$MAP_YAML"
```

Confirm candidate, direction, status, physical limits and exact CSV path. The tested
candidate's CSV hash is
`0aab3968b64537b608ff8821ff8b1bd28ec3253d3c12719d542ad3ae6a71e88e`;
there may be several sessions exporting those same bytes. Use the intended one,
not an arbitrary directory. Missing/changed CSV, map or evidence fails verification.
No manifest available? Use the [workbench](#11-offline-raceline-workbench) to deliberately
generate/select/export; generated sessions are local and not included in every clone.
A different CSV or vehicle profile is a new experimental input, not reproduction
of the previously tested baseline.

The source simulator YAML selects the same Tempo YAML/PGM and straight-start pose
x=2.66927602204947, y=1.14374263398612, yaw=0.131970518672264 (CSV row 270 of the
above candidate). It must correspond to your intended geometry. Do not silently
reuse that pose for an unrelated CSV. PF receives the explicit map YAML through
`map_yaml`; its base config's legacy IFAC map name is not authoritative here.

### Simulation container: run, stop, analyze

```bash
RUN_DIR="/sim_ws/src/oudtra_driver_bringup/runs/tempo_coffee_experimental_pf_pnc_low_speed_$(date -u +%Y%m%dT%H%M%SZ)"
# RViz observation; remove --with-rviz for the maintained headless mode.
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --scenario-id "$SCENARIO_ID" --sim-config "$SIM_CONFIG" \
  --pf-map-yaml "$MAP_YAML" --localize-config "$LOCALIZE_CONFIG" \
  --selected-raceline "$SELECTED" --allow-experimental \
  --speed-profile "$SPEED_PROFILE" --with-rviz \
  --run-duration-sec 30 --ros-domain-id 95 \
  --record-dir "$RUN_DIR" --result-json /tmp/tempo_coffee_low_speed_smoke.json \
  --source-revision "$ROBORACER_MAIN_REV" --pf-revision "$ROBORACER_PF_REV" \
  --source-state "$ROBORACER_SOURCE_STATE"
# Check the smoke result before interpreting offline analysis.
cat "$RUN_DIR/smoke_result.json"
ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN_DIR"
```

**Expected output:** smoke exit 0, `result: PASS`, `recording_passed: true`, 30 s
motion, runtime Tempo map path and verified speed parameters, followed by analysis
PASS and generated scorecard. Smoke FAIL or nonzero exit means the bounded run
failed; retain its evidence. Analyzer PASS alone does not turn that failure into
a successful experiment. Success is short-run motion/recording evidence, not
accuracy acceptance, collision-free endurance, repeatability for every initial
pose or onboard readiness.

The opt-in profile sets shared nominal forward commands to 0.5–2.0 m/s and final
commands to ±2.0 m/s. Safety stops, slower caps and reverse remain possible;
measured speed is not forced into this range. No default PF/PnC/vehicle tuning is
changed. Profile and resolved integration overlay are copied into the run.

**Stop/cleanup:** the runner stops automatically after the interval and checks
stationary cleanup/owned-process exit. Ctrl-C requests the same cleanup but marks
an interrupted run incomplete/failed. Do not stop/remove the container. Closing
RViz is not the normal stop command. If cleanup fails, inspect the result and
`logs/` before another run; do not kill unrelated stacks.

### Optional monitoring terminal

From a second **host terminal**, enter the same container:

```bash
docker exec -it f1tenth_gym_ros_rocker bash
```

Inside that **monitoring container terminal**, while the runner is active:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=95
ros2 param get /map_server yaml_filename
timeout 5s ros2 topic echo /map nav_msgs/msg/OccupancyGrid --no-arr \
  --qos-reliability reliable --qos-durability transient_local
ros2 param get /raceline_publisher csv_path
ros2 param get /raceline_publisher direction
ros2 param get /path_generator command_speed_min_mps
ros2 param get /path_generator command_speed_max_mps
ros2 param get /path_following_v2 command_speed_max_mps
ros2 param get /lower_safety_controller absolute_speed_limit_mps
ros2 param get /lower_safety_controller enable_sim_reverse_swept_gate
ros2 param get /drive_arbitrator require_pf_health
ros2 topic info /pf/pose/odom
ros2 topic info /drive
```

The bounded map echo may finish with timeout exit 124 after displaying the latched
grid metadata; it is a passive observation, not a readiness delay.
PF loads `/map_server/map`; the planner consumes that server's `/map`. Confirm
Tempo map geometry and selected CSV, min/max 0.5/2.0, final cap 2.0, GT gate false
and health gate true. Graph presence alone does not prove fresh/accurate poses;
read the runner and recorded evidence. If changing the runner domain, change this
terminal's domain too.

### Troubleshooting and evidence to retain

| Symptom | Action / interpretation |
| --- | --- |
| Manifest missing or hash mismatch | Explicitly select a real session and run the resolver; no silent fallback or arbitrary copied CSV |
| Car stationary / smoke FAIL | Inspect `smoke_result.json`, PF and PnC logs; preserve the first failure, do not relax safety/PF gates |
| Wrong starting heading / reverse anomaly | First-row start is a known failed case; use the documented straight-start fixture only for this tested geometry |
| RViz graphics warning | Inspect simulator log; headless repeat is available. Rendering is not control/localization validation |
| Existing run directory | Choose a fresh descriptive directory; do not overwrite prior evidence |

Host run home: `/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<run-id>/`.
Retain the whole directory: `rosbag/`, `metadata.yaml`, `smoke_result.json`, selected
manifest/validation copies, `config/`, `logs/`, `metrics.json`, `report.md`, `plots/`.
Open `report.md` in Markdown preview and `plots/trajectory_xy.png`,
`reference_trajectory.png`, `position_error.png`, `smoothness_oscillations.png`.
The locally preserved headless and RViz examples are named
`tempo_coffee_experimental_pf_pnc_straight_start_0p5_to_2mps_20261011` and
`tempo_coffee_experimental_pf_pnc_rviz_repeat_0p5_to_2mps_20261011`; these are discovery
names, not guaranteed downloadable assets. The failed first-start run remains
separate and retains its original simulator config.

Changing map/image, CSV/direction, vehicle profile, start pose/offsets, PF/PnC config,
speed limits, duration or analysis definitions changes the comparison context.
Retain exact hashes and describe differences; do not infer comparability from run
names. For the onboard qualification/recording gate use [section 5](#5-onboard-stack-and-passive-measurement),
then follow [section 10](#10-compare-two-analyzed-runs). Tempo is still blocked for
recommended onboard autonomy; no new-track launch workaround is provided here.
