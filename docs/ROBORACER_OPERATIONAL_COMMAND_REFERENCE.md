# RoboRacer operational command reference

## 1. How to use this document

**Purpose:** run, record and inspect RoboRacer workflows using exact terminal
commands. **Start here:** choose A1 or A2, then the human-controlled B and offline C;
it is an operator handoff, not a development roadmap. Simulation uses the existing
Foxy `f1tenth_gym_ros_rocker` localization-ready container; onboard uses its own
Foxy workspace. The human authorizes current experimental Tempo geometry for a
controlled integration experiment; production validity and normal safety remain unchanged.

### Current operator workflow

| Activity / readiness | Entry point | Expected output | Boundary |
| --- | --- | --- | --- |
| **A1. Tempo interactive simulation — executable** | [Three persistent terminals](#13-a1--interactive-tempo-simulation-in-three-terminals) | Live PF/PnC and RViz; optional raw diagnostic bag | Human readiness/initialization/stop; no automatic evaluation recording |
| **A2. Tempo timed simulation — executable** | [Existing recorded runner](#12-tempo-coffee-experimental-pfpnc-and-low-speed-comparison) | Bounded smoke, bag, metrics/report/plots | Automatic one-shot initialization/cleanup; experimental path unchanged |
| **B. Tempo onboard — prepared controlled experiment** | [Four simple terminals](#5-onboard-stack-and-passive-measurement) | Human-controlled run and passive dataset | Assumes code/PF deployed and built; tracked map/raceline included; experimental geometry; physical operation remains human-controlled |
| **C. Transfer, analysis and comparison — executable when datasets exist** | [B5 transfer](#b5-dataset-transfer-and-offline-analysis), [offline comparison](#10-compare-two-analyzed-runs) | Two scorecards and comparison report/JSON | Prefer A2 for the simulation analysis contract; no matched-section or truth equivalence claim |

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
  - [B1. Terminal 1 — established vehicle bringup](#b1-terminal-1--established-vehicle-bringup)
  - [B2. Terminal 2 — explicitly selected Tempo PF and RViz](#b2-terminal-2--explicitly-selected-tempo-pf-and-rviz)
  - [B3. Terminal 3 — current experimental Tempo PnC](#b3-terminal-3--current-experimental-tempo-pnc)
  - [B4. Terminal 4 — simple passive logging](#b4-terminal-4--simple-passive-logging)
  - [B5. Dataset transfer and offline analysis](#b5-dataset-transfer-and-offline-analysis)
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
  - [Goal and prerequisites](#goal-and-prerequisites)
  - [Host terminal: verify and enter the existing container](#host-terminal-verify-and-enter-the-existing-container)
  - [Simulation container: build, choose assets and verify selection](#simulation-container-build-choose-assets-and-verify-selection)
  - [A2 — timed run, stop and offline analysis](#a2--timed-run-stop-and-offline-analysis)
  - [Optional monitoring terminal](#optional-monitoring-terminal)
  - [Troubleshooting and evidence to retain](#troubleshooting-and-evidence-to-retain)
- [13. A1 — interactive Tempo simulation in three terminals](#13-a1--interactive-tempo-simulation-in-three-terminals)
  - [Goal and preconditions](#goal-and-preconditions)
  - [A1 Terminal 1 — persistent Tempo simulator and RViz](#a1-terminal-1--persistent-tempo-simulator-and-rviz)
  - [A1 Terminal 2 — PF with the explicit Tempo map](#a1-terminal-2--pf-with-the-explicit-tempo-map)
  - [A1 Terminal 3 — persistent full PnC with the selected experimental raceline](#a1-terminal-3--persistent-full-pnc-with-the-selected-experimental-raceline)
  - [A1 verification and troubleshooting](#a1-verification-and-troubleshooting)
  - [A1 stop and cleanup](#a1-stop-and-cleanup)
  - [A1 data outputs and optional diagnostic recording](#a1-data-outputs-and-optional-diagnostic-recording)
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

Run these four terminals on the onboard computer. Assume the repository, PF
submodule and curated Tempo assets are already deployed and built, and the existing
bringup works. Use your normal vehicle control/stop procedure.

### B1. Terminal 1 — established vehicle bringup

On the **onboard host**, use your existing startup sequence unchanged:

```bash
~/start_gui_host.sh
~/f1tenth_ws/src/f1tenth_system/scripts/run_container_loc.sh
```

Then **inside the onboard container**:

```bash
source /opt/ros/foxy/setup.bash
source /f1tenth_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Leave running. This starts the normal sensor, odometry, hardware and mux path.

### B2. Terminal 2 — explicitly selected Tempo PF and RViz

Enter the container from another **onboard-host terminal**:

```bash
docker exec -it f1tenth_onboard /bin/bash
```

Then run **inside the container**:

```bash
source /opt/ros/foxy/setup.bash
source /f1tenth_ws/install/setup.bash
ros2 launch particle_filter localize_onboard_rviz_launch.py \
  map_yaml:=/f1tenth_ws/src/f1tenth_dev/centerline_tools/tempo_coffee_scanned_20261002_00.yaml
```

Use RViz **2D Pose Estimate** once at the car's actual position and heading on the
Tempo map. Leave PF/RViz running. The launch uses its installed onboard PF config;
`map_yaml` explicitly replaces the old implicit IFAC selection. A missing map/image
fails rather than choosing another track. No simulator parameters are involved.

### B3. Terminal 3 — current experimental Tempo PnC

Enter the container from another **onboard-host terminal**:

```bash
docker exec -it f1tenth_onboard /bin/bash
```

Then run **inside the container**:

```bash
source /opt/ros/foxy/setup.bash
source /f1tenth_ws/install/setup.bash
ros2 launch oudtra_driver_bringup full_stack_localization_onboard_launch.py \
  track:=tempo_coffee allow_experimental:=true
```

Leave running. Enable/control autonomy with your existing vehicle procedure after
PF initialization. The adapter applies the existing comparison-speed profile: **0.5–2.0 m/s**
ordinary forward-command range and **2.0 m/s absolute final-command cap** across
path following and safety. No configuration arguments or generated files need
operator management. The startup banner shows the exact installed CSV, map,
status, direction and speed limits. Safety can still slow/stop/reverse. Everything
else retains onboard defaults; the experimental path is not relabelled validated.
Autonomy `/drive` passes through the mux to `/ackermann_cmd` toward the VESC.

### B4. Terminal 4 — simple passive logging

Enter the container from another **onboard-host terminal**:

```bash
docker exec -it f1tenth_onboard /bin/bash
```

Then run **inside the container**:

```bash
source /opt/ros/foxy/setup.bash
source /f1tenth_ws/install/setup.bash
ros2 bag record -o "/f1tenth_ws/onboard_runs/tempo_raw_$(date -u +%Y%m%dT%H%M%SZ)" \
  /pf/pose/odom /pf/health /odom /scan \
  /raceline_path /drive /ackermann_cmd \
  /drive_arbitration_v2/status /drive_arbitration_v2/selected_mode \
  /reactive_control_v2/status /reactive_control_v2/lower_safety_status \
  /sensors/core /sensors/imu/raw /sensors/servo_position_command \
  /tf /tf_static /initialpose
```

This is an ordinary passive rosbag, extending your earlier status/drive recording
with pose, reference and sensor evidence. No separate preflight, interval marking
or metadata setup. It does not launch/control/reset anything. Start another bag
with the same command for the next attempt; the timestamp creates a new directory.
Only messages published during recording are captured; late-started recording
may miss an earlier initial-pose event or latched reference/static TF.

**Stop:** use your normal vehicle stop/manual override first. Ctrl-C T4 to finalize
the bag, then stop T3, T2 and T1 as usual. Stopping recording does not stop the car;
releasing the latched auto button alone is not a disarm.

**Output:** `/f1tenth_ws/onboard_runs/tempo_raw_<UTC>/` contains the bag's
`metadata.yaml` and database files. This is raw diagnostic evidence, **not** the
shared analyzer's run `metadata.yaml`/interval contract. Follow B5 to import this
raw bag before analysis. To capture an explicit evaluation interval and deployed
provenance at recording time, use the structured recorder **instead**:

<details>
<summary>Optional Terminal 4 alternative — structured dataset for offline analysis</summary>

In the same sourced onboard container shell:

```bash
REPO=/f1tenth_ws/src/f1tenth_dev
RACELINE="$REPO/centerline_tools/tracks/tempo_coffee/raceline_points_experimental.csv"
RUN_DIR="/f1tenth_ws/onboard_runs/tempo_structured_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run oudtra_driver_bringup record_onboard_localization.py "$RUN_DIR" \
  --map-yaml "$REPO/centerline_tools/tempo_coffee_scanned_20261002_00.yaml" \
  --map-image "$REPO/centerline_tools/tempo_coffee_scanned_20261002_00.pgm" \
  --raceline-csv "$RACELINE" \
  --config pf="$REPO/particle_filter/config/localize.yaml" \
  --config pnc="$REPO/oudtra_driver_bringup/config/full_stack.yaml" \
  --config path="$REPO/path_following_v2/config/path_following_v2.yaml" \
  --config reactive="$REPO/reactive_control_v2/config/reactive_control_v2.yaml" \
  --config arbitration="$REPO/drive_arbitration_v2/config/drive_arbitration_v2.yaml" \
  --config speed="$REPO/oudtra_driver_bringup/config/comparison_speed_limits.yaml"
```

No separate preflight command: the recorder checks its own required evidence on
startup and reports any missing signals. Press **Enter** when you want the evaluated
interval to begin; Ctrl-C ends/finalizes it after the human stops the car. The
recorder never enables autonomy. It produces run `metadata.yaml`, `rosbag/`,
`config/` and `logs/`, with source/asset/config hashes and the explicit interval.
Keep a short operator note of the activity, interventions and stop reason; direction
here is `csv`. Inputs named by `--config` are hashed, not all copied automatically.
Preserve the deployed files alongside the dataset when retaining comparison evidence.
Missing evidence remains missing; no physical-car recorder validation is claimed.

</details>

### B5. Dataset transfer and offline analysis

**After recording:** transfer → import raw bag → analyze → compare. These steps run
on the **development laptop**, not the vehicle. No ROS replay or live simulator is
needed for onboard analysis. A raw bag's `metadata.yaml` is bag indexing metadata,
not the evaluation run metadata; importing creates the missing run wrapper without
inventing the deployed source/configuration or successful completion.

**Host terminal — copy from the onboard computer:** replace the SSH stub once.
The package-local raw inbox is Git-ignored and bind-mounted into the analysis
container, so no additional Docker copy is needed after transfer.

```bash
cd /home/mzhou/f1tenth_dev
mkdir -p oudtra_driver_bringup/onboard_runs
VEHICLE='<onboard-user>@<onboard-host>'
rsync -a "$VEHICLE:~/f1tenth_ws/onboard_runs/" oudtra_driver_bringup/onboard_runs/
docker exec -it f1tenth_gym_ros_rocker /bin/bash
```

The SSH address is intentionally a placeholder. Current delivered bags have already
been copied to that inbox. Original root `onboard_runs/` files are preserved too.

**Inside the simulation/analysis container — build once after pulling updates:**

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select oudtra_driver_bringup
source install/local_setup.bash
```

**Import and analyze the selected third recording:** these exact offsets describe
this delivered bag's observed motion, not a general setting for every future run.
The timestamped output makes each manual import distinct.

```bash
BASE=/sim_ws/src/oudtra_driver_bringup
RAW="$BASE/onboard_runs/tempo_raw_20261105T202752Z"
RUN="$BASE/runs/tempo_onboard_session3_motion_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run oudtra_driver_bringup import_onboard_bag.py "$RAW" \
  --output-dir "$RUN" --start-offset-sec 7.2 --end-offset-sec 64.8 \
  --scenario-id tempo_coffee_scanned_20261002_00
ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN"
```

For a different raw bag, change `RAW`; omit both offset options to analyze its whole
recording, including stationary/startup time, or supply explicit reviewed bounds.
`--inspect-only` prints SQLite integrity, duration, topic/count and raw-file hashes
without creating output. Existing output directories are rejected. Do not point
`analyze_localization_run.py` directly at a raw bag or manufacture a smoke PASS.
Structured-recorder datasets already have `metadata.yaml` plus `rosbag/`: skip
import and pass that complete run directory directly to the analyzer.

**Compare with simulation:** use an existing successfully analyzed A2 run. For the
current local validation dataset:

```bash
SIM="$BASE/runs/tempo_sim_dependency_regression_20261107"
COMPARISON="$BASE/runs/tempo_sim_vs_onboard_session3_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run oudtra_driver_bringup compare_localization_runs.py \
  "$SIM" "$RUN" --output-dir "$COMPARISON"
```

That simulation dataset is local ignored evidence, not downloaded by Git. If absent,
run/analyze [A2](#12-tempo-coffee-experimental-pfpnc-and-low-speed-comparison) first
and assign its output directory to `SIM`. Keep the same map, selected reference,
direction and declared speed profile for the intended comparison. Matching filenames
alone do not establish what was deployed on the car.

**Open on the laptop host:** container prefix `/sim_ws/src` corresponds to
`/home/mzhou/f1tenth_dev`. Each imported run contains:

| Artifact | Purpose |
| --- | --- |
| `report.md`, `metrics.json`, `plots/` | Human scorecard, machine results and engineering figures |
| `metadata.yaml`, `import_audit.json`, `config/` | Explicit interval, roles/settings, unknown provenance and raw hash audit |
| `rosbag/` | Complete copied raw bag; original input remains untouched |
| Comparison `report.md`, `comparison.json` | Side-by-side results and reasons deltas are withheld |

The current completed examples are `runs/tempo_onboard_session3_motion_20261105/`
and `runs/tempo_sim_vs_onboard_session3_20261107/` under this package. Open their
`report.md`; the selected-run figures are in its `plots/` directory. Retain whole
run directories, not just JSON or images. Raw inboxes, run datasets and temporary
handoffs stay uncommitted.

**Interpretation:** 100% fresh-pose delivery is not 100% healthy localization.
Raw imports cannot recover missing deployed revisions, PF map/config hashes,
initialization events, task completion or collision evidence. No onboard GT means
absolute accuracy is NOT_APPLICABLE. The current split speed/IMU adapter is absent,
so vehicle-response smoothness and stall evidence remain unavailable. Comparison
PASS means the table was generated; unknown identity/evidence suppresses numerical
deltas. It does not establish matched sections or simulator/onboard equivalence.

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
without launching anything. Use the **validated-only** resolver for ordinary production handoff. The human-authorized
Tempo experimental exception uses the explicit hash-qualified bundle and onboard
procedure in [section 5](#5-onboard-stack-and-passive-measurement); no status is relabelled. A custom offline vehicle
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

The current Tempo selection is version-controlled; use it directly:

```bash
SELECTED=/sim_ws/src/centerline_tools/tracks/tempo_coffee/selected_raceline.yaml
```

The runner verifies the manifest, experimental acknowledgment, geometry/evidence
hashes and explicit map identity before startup. No session discovery, prompt or
separate transfer is needed. CSV SHA is
`0aab3968b64537b608ff8821ff8b1bd28ec3253d3c12719d542ad3ae6a71e88e`.
A deliberately changed deployment selection is a Git change; use the same commit
on both platforms. Workbench experiments remain local until explicitly promoted.

The source simulator YAML selects the same Tempo YAML/PGM and straight-start pose
x=2.66927602204947, y=1.14374263398612, yaw=0.131970518672264 (CSV row 270 of the
above candidate). It must correspond to your intended geometry. Do not silently
reuse that pose for an unrelated CSV. PF receives the explicit map YAML through
`map_yaml`; its base config's legacy IFAC map name is not authoritative here.

### A2 — timed run, stop and offline analysis

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
then follow [section 10](#10-compare-two-analyzed-runs). The onboard launch now requires
explicit map selection; actual stationary/deployment and physical driving checks
remain human-executed. The experimental reference is not production-validated.

## 13. A1 — interactive Tempo simulation in three terminals

### Goal and preconditions

Run Tempo persistently for RViz observation: **simulator → PF → PnC**. Paste each
terminal block in order; initialize PF in RViz before starting Terminal 3.
**No prompts, copied overlay paths, fixed duration or automatic recording.**
This procedure explicitly uses experimental `008_geometry`; it is not a
production-validated raceline. Do not run A1 and A2 together in one ROS domain.

**Build once, or after source changes:** enter the existing container from a
**dev-host shell**:

```bash
docker exec -it f1tenth_gym_ros_rocker bash
```

Then build in that **container shell** (wait for completion before starting T1–T3):

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros particle_filter centerline_tools path_following_v2 reactive_control_v2 drive_arbitration_v2 oudtra_driver_bringup
```

Open three **dev-host terminals**, enter the same existing container in each:

```bash
docker exec -it f1tenth_gym_ros_rocker bash
```

The following blocks run **inside the container**, all on domain 95. Source changes
must be built first; daily starts need only the source/launch commands below.

### A1 Terminal 1 — persistent Tempo simulator and RViz

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=95
ros2 launch f1tenth_gym_ros gym_bridge_slam_launch.py \
  config_file:=/sim_ws/src/f1tenth_gym_ros/config/sim_tempo_coffee.yaml \
  use_rviz:=true
```

Starts one car, scan/raw odometry, robot transforms and RViz. Map and localized
map→vehicle TF appear after Terminal 2 starts and PF initializes. Their initial
absence is expected. Keep the simulator running.

### A1 Terminal 2 — PF with the explicit Tempo map

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=95
ros2 launch particle_filter localize_sim_launch.py \
  localize_config:=/sim_ws/src/particle_filter/config/localize_sim.yaml \
  map_yaml:=/sim_ws/src/centerline_tools/tempo_coffee_scanned_20261002_00.yaml \
  parameter_overlay:=/sim_ws/src/oudtra_driver_bringup/config/localization_eval_sim.yaml
```

In RViz, use **2D Pose Estimate** once at the actual stationary start:
**x=2.669 m, y=1.144 m, heading=0.132 rad (7.56° from +X)**. Wait for the localized
scan/vehicle transform and usable PF health before starting PnC. Terminal 2 keeps
running; no generated path needs to be copied anywhere.

PF uses the existing integration config directly. Speed settings belong to PnC,
so PF does not need a merged speed overlay. No simulator truth is continuously
injected into PF. RViz is an observation aid, not proof of localization accuracy.

### A1 Terminal 3 — persistent full PnC with the selected experimental raceline

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=95
ros2 launch oudtra_driver_bringup full_stack_localization_sim_launch.py \
  track:=tempo_coffee allow_experimental:=true
```

`track:=tempo_coffee` explicitly selects the installed, tracked manifest. It resolves
the same CSV/direction/map as onboard; missing/changed evidence fails before PnC
starts, without a fallback. No timestamped session path is needed. Advanced use
can still provide `selected_raceline:=<absolute-manifest>` with explicit `map_yaml`
instead of `track`; supplying both selection mechanisms is rejected.

The launch verifies the manifest/hashes, prints the exact candidate/CSV/direction,
and starts **only PnC**, using existing shared launches. It internally applies the
same localization integration and comparison-speed merge as A2. No operator-managed
overlay is needed. Default ordinary forward demand is **0.5–2.0 m/s**, with a final
**2.0 m/s absolute command cap**; safety stops, slower caps and reverse remain
possible. PF health is required; simulator-GT reverse recovery is disabled.
Experimental selection acknowledges failed production validation; it disables no
safety gate and does not qualify the raceline for production.

### A1 verification and troubleshooting

The startup banner identifies candidate `008_geometry`, status `EXPERIMENTAL`,
direction `csv`, active CSV and speed profile. Expected CSV SHA is
`0aab3968b64537b608ff8821ff8b1bd28ec3253d3c12719d542ad3ae6a71e88e`.
PnC runs until Ctrl-C. Stopped/reactive operation must be interpreted from status
messages; do not bypass a stop merely to obtain motion.

If PF is not initialized or the vehicle remains stopped, use an optional fourth
**sourced container shell**:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
export ROS_DOMAIN_ID=95
ros2 param get /map_server yaml_filename
timeout 3s ros2 topic echo /pf/health
timeout 3s ros2 topic echo /pf/pose/odom --no-arr
timeout 3s ros2 run tf2_ros tf2_echo map ego_racecar/base_link
```

Bounded observations normally end with timeout exit 124. Check fresh usable health,
finite map-frame pose and the expected heading. The prior first-row start failed
despite initial usable health, so health alone does not establish correctness.
See [A2 monitoring](#optional-monitoring-terminal) for active-reference, speed,
GT-gate and arbitration checks.

If RViz reports an `indexed_8bit_image` GLSL/sampler error, distinguish that map
rendering issue from ROS map/PF message flow. It occurred in container validation
while closed-loop motion still worked; map appearance and the manual pose-click
experience require your visual check. Graphics configuration is unchanged here.

For repeatable initialization **instead of the RViz click**, publish once from
that fourth shell **before PnC starts and only at this configured stationary start**:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 2.66927602204947, y: 1.14374263398612}, orientation: {z: 0.065937386, w: 0.997823765}}}}'
```

This simulation-only command must be updated if the configured start changes.
Do not use it on the physical car or repeatedly reset localization while driving.

### A1 stop and cleanup

While the simulator still runs, Ctrl-C **Terminal 3** first. From that now-free
sourced terminal, command simulator stop and confirm stationary raw odometry:

```bash
ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {speed: 0.0, steering_angle: 0.0}}'
timeout 3s ros2 topic echo /ego_racecar/odom --no-arr
```

Then Ctrl-C **Terminal 2**, then **Terminal 1**. Do not stop/remove the container.
This is a simulator procedure, not an onboard stop procedure. If stationary cleanup
cannot be established, stop the simulator and retain diagnostics before restarting.
The wrapper removes its internal temporary YAML/provenance on normal shutdown.
A1 provides no automatic readiness, domain-exclusivity checks, motion watchdog or
owned process cleanup for the other terminals; the operator controls their lifecycle.

### A1 data outputs and optional diagnostic recording

**No run dataset/report is recorded automatically.** Console output identifies the
active assets. For reproducible scorecards/comparison, use **A2 timed recording**.

Optional **raw diagnostic** recording in a fourth sourced container shell:

```bash
export ROS_DOMAIN_ID=95
RAW_DIR="/sim_ws/src/oudtra_driver_bringup/runs/tempo_manual_raw_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 bag record -o "$RAW_DIR" /scan /ego_racecar/odom /pf/pose/odom /pf/health \
  /tf /simulator/agent_status /drive /reactive_control_v2/lower_safety_status /drive_arbitration_v2/status
```

Ctrl-C the recorder to finalize. The bag is host-visible under
`/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/`. Retain the exact manifest,
source/config revisions, initialization and interval notes with it. This raw bag
omits the latched reference/standard metadata and evaluation interval, so it is
**not a shared-analyzer run contract**; do not fabricate benchmark metadata for it.
