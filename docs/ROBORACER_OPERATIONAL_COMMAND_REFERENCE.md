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

These commands reflect the current onboard launch and recording contracts. The
recorder has been validated against tests and a controlled ROS graph, but this
four-terminal procedure still requires its first real-car verification. It does
not replace the vehicle team's normal safety checks or operating authority.
After deploying this repository revision, build `oudtra_driver_bringup` in
`/f1tenth_ws` once so Terminal 4 uses the installed recorder and contract.

In each terminal inside the existing onboard ROS 2 Foxy container:

```bash
source /opt/ros/foxy/setup.bash
cd /f1tenth_ws
source install/setup.bash
```

Use the existing workflow unchanged in Terminals 1–3:

```text
Terminal 1: ros2 launch f1tenth_stack bringup_launch.py
Terminal 2: ros2 launch particle_filter localize_onboard_rviz_launch.py
Terminal 3: ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

Initialize PF with RViz `2D Pose Estimate` as usual. The passive recorder does
not publish `/initialpose`, reset PF, enable autonomy, or launch any stack node.

In Terminal 4, first check the live contract without creating a run:

```bash
ros2 run oudtra_driver_bringup record_onboard_localization.py \
  --preflight-only
```

`PASS` requires every required topic to have the declared type and an observed
message. Missing optional `/initialpose` evidence is reported but does not fail
preflight. Start a descriptive run before enabling/driving autonomous motion:

```bash
RUN_DIR="/f1tenth_ws/onboard_runs/ifac_pf_onboard_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run oudtra_driver_bringup record_onboard_localization.py "$RUN_DIR" \
  --map-yaml /f1tenth_ws/src/f1tenth_dev/particle_filter/maps/ifac_roboracer.yaml \
  --map-image /f1tenth_ws/src/f1tenth_dev/particle_filter/maps/ifac_roboracer.png \
  --raceline-csv /f1tenth_ws/src/f1tenth_dev/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --config pf=/f1tenth_ws/src/f1tenth_dev/particle_filter/config/localize.yaml \
  --config pnc=/f1tenth_ws/src/f1tenth_dev/oudtra_driver_bringup/config/full_stack.yaml \
  --config path=/f1tenth_ws/src/f1tenth_dev/path_following_v2/config/path_following_v2.yaml \
  --config reactive=/f1tenth_ws/src/f1tenth_dev/reactive_control_v2/config/reactive_control_v2.yaml \
  --config arbitration=/f1tenth_ws/src/f1tenth_dev/drive_arbitration_v2/config/drive_arbitration_v2.yaml \
  --config mux=/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/mux.yaml \
  --config vesc=/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
```

The recorder immediately retains startup evidence, then waits. Press Enter once
to mark the evaluation start. Press Ctrl-C once at the intended end; it marks the
evaluation end, asks rosbag to finalize, validates the bag, and updates
`metadata.yaml`. A 0.5 s passive post-roll preserves samples just beyond the
evaluation boundary; it is not included in the evaluated interval. The tool rejects
an existing run directory rather than overwriting it.
An interrupted preflight/startup still leaves explicit incomplete/failure metadata
when a run directory was created.

The onboard host sees the run under `~/f1tenth_ws/onboard_runs/<run-id>/` when
the standard workspace bind mount is used. From the development laptop, copy the
whole directory without changing its contents:

```bash
rsync -a <vehicle-user>@<vehicle-host>:~/f1tenth_ws/onboard_runs/<run-id>/ \
  /home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<run-id>/
```

Then analyze it inside the sourced localization-ready development container:

```bash
ros2 run oudtra_driver_bringup analyze_localization_run.py \
  /sim_ws/src/oudtra_driver_bringup/runs/<run-id>
```

Open `report.md`, `metrics.json`, and `plots/` under the copied run directory.
Absolute GT accuracy is correctly reported as not applicable onboard.

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

After both run directories report analysis PASS, use a sourced canonical container;
no simulator or live ROS graph is needed:

```bash
ros2 run oudtra_driver_bringup compare_localization_runs.py \
  /sim_ws/src/oudtra_driver_bringup/runs/<sim-run> \
  /sim_ws/src/oudtra_driver_bringup/runs/<onboard-run> \
  --output-dir /sim_ws/src/oudtra_driver_bringup/runs/<descriptive-comparison-name>
```

Replace the angle-bracket paths with real analyzed runs. Comparison requires no
live ROS graph. Unknown identity or incompatible evidence suppresses deltas instead
of suggesting equivalent measurements.

Open `report.md` beside `metrics.json` in each run; plots are linked under `plots/`.
Comparison writes `report.md` and `comparison.json` in its separate output directory.
Host equivalents replace `/sim_ws/src/` with `/home/mzhou/f1tenth_dev/`. Current
simulation and **synthetic no-GT validation** artifacts are listed in the
[deliverables section](LOCALIZATION_SIMULATION.md#56-plots-and-validation-artifacts).

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

The selected Tempo Coffee `008_geometry` CSV is **experimental**, not production
validated: peak curvature exceeds 1.082 1/m. A straight-section start (CSV row 270)
was used for the bounded simulation; the first-row start failed during initialization
and wrong-way recovery. Short-run simulation success does not qualify onboard use.
The workbench does not launch this workflow automatically.

### Repeat the complete simulator workflow, optionally with RViz

Open the existing container from the host as in [section 8](#8-record-and-analyze-an-evaluation-run)
to pass `ROBORACER_MAIN_REV` and `ROBORACER_PF_REV`. Inside it:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros centerline_tools oudtra_driver_bringup
source install/local_setup.bash
SCENARIO_ID=tempo_coffee_scanned_20261002_00
SIM_CONFIG=/sim_ws/src/f1tenth_gym_ros/config/sim_tempo_coffee.yaml
MAP_YAML=/sim_ws/src/centerline_tools/tempo_coffee_scanned_20261002_00.yaml
SELECTED=/sim_ws/src/centerline_tools/centerline_output/workbench/tempo_coffee_scanned_20261002_00-20261011T194408Z-e62d290a5af6-a91105/selected_raceline.yaml
LOCALIZE_CONFIG=/sim_ws/src/particle_filter/config/localize_sim.yaml
SPEED_PROFILE=/sim_ws/src/oudtra_driver_bringup/config/comparison_speed_limits.yaml
RUN_DIR="/sim_ws/src/oudtra_driver_bringup/runs/tempo_coffee_experimental_pf_pnc_low_speed_$(date -u +%Y%m%dT%H%M%SZ)"
python3 /sim_ws/src/centerline_tools/resolve_selected_raceline.py \
  "$SELECTED" --allow-experimental --map-yaml "$MAP_YAML"
ros2 run oudtra_driver_bringup run_localization_smoke.py \
  --scenario-id "$SCENARIO_ID" --sim-config "$SIM_CONFIG" \
  --pf-map-yaml "$MAP_YAML" --localize-config "$LOCALIZE_CONFIG" \
  --selected-raceline "$SELECTED" --allow-experimental \
  --speed-profile "$SPEED_PROFILE" --with-rviz \
  --run-duration-sec 30 --ros-domain-id 95 \
  --record-dir "$RUN_DIR" --result-json /tmp/tempo_coffee_low_speed_smoke.json \
  --source-revision "${ROBORACER_MAIN_REV:-}" \
  --pf-revision "${ROBORACER_PF_REV:-}" --source-state unknown
ros2 run oudtra_driver_bringup analyze_localization_run.py "$RUN_DIR"
```

The exact session above is local/Git-ignored; substitute your explicitly saved
manifest if it is unavailable. There is no hidden latest-path fallback. The runner
owns simulator → PF → one-shot initialization → full PnC → bounded motion → stop.
Do not launch duplicate stacks in this ROS domain or reset PF during the run.
Omit `--with-rviz` for the headless repeat. Ctrl-C requests stopped cleanup; an
interrupted run is incomplete, not PASS. Read the smoke result before interpreting
the analyzer: analysis PASS validates recorded evidence, not driving success.

The opt-in speed profile sets the shared path generator/planner/follower command
range to **0.5–2.0 m/s** and lower safety's final command cap to **±2.0 m/s**.
Safety stops, slower safety/trajectory caps and reverse recovery remain possible;
measured speed is not forced into the command interval. It changes no default
profiles, PF tuning, steering envelope or vehicle dynamics. The runner merges only
these limits into its localization integration overlay, verifies effective values,
and snapshots both the source profile and resolved overlay in the recorded run.

For a live check in a second sourced terminal:

```bash
export ROS_DOMAIN_ID=95
ros2 param get /map_server yaml_filename
ros2 param get /raceline_publisher csv_path
ros2 param get /raceline_publisher direction
ros2 param get /path_generator command_speed_max_mps
ros2 param get /path_following_v2 command_speed_max_mps
ros2 param get /lower_safety_controller absolute_speed_limit_mps
```

Simulator and PF use the same explicit Tempo YAML/PGM; the planner subscribes to
that map server's `/map`, while PnC reference geometry comes from the selected CSV.
The legacy `map: ifac_roboracer` in the PF base config is overridden by `map_yaml`.
`smoke_result.json` retains the runtime map parameter, selected CSV and effective
limits. Open the run's `report.md`, `metrics.json`, `plots/` and `config/` on the host
under `/home/mzhou/f1tenth_dev/oudtra_driver_bringup/runs/<run-id>/`.

### Onboard comparison preparation — not a validated Tempo driving procedure

Use the existing [onboard Terminal 1–4 workflow](#5-onboard-stack-and-passive-measurement)
and passive recorder. **Do not deploy this experimental CSV through the recommended
onboard workflow.** A human-reviewed, physically qualified reference is required;
this simulation does not establish that qualification. No car was driven here.

There is also a pinned-launch limitation: `localize_onboard_rviz_launch.py` includes
`localize_launch.py`, which selects the map from the installed default `localize.yaml`
before resolving `localize_config`. Passing a custom config alone therefore does
**not** select Tempo. Confirm `/map_server yaml_filename` and the actual `/map`
against the intended YAML/image before enabling autonomy; mismatched map evidence
invalidates the comparison. A reliable explicit new-track onboard PF startup is
not established by this task.

For an already verified onboard map and human-qualified CSV, the existing Terminal 3
interface can apply the same speed limits without simulator frames/GT adapters:

```bash
SPEED_PROFILE=/f1tenth_ws/src/f1tenth_dev/oudtra_driver_bringup/config/comparison_speed_limits.yaml
# Set this to the reviewed CSV available on the car, not an experimental export.
RACELINE=/absolute/onboard/path/to/reviewed_raceline.csv
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py \
  raceline_csv_path:="$RACELINE" raceline_direction:=csv \
  path_platform_config:="$SPEED_PROFILE" \
  reactive_platform_config:="$SPEED_PROFILE" \
  integration_platform_config:="$SPEED_PROFILE"
```

This command replaces Terminal 3, not an additional stack. Do not overlay the
simulator localization config onboard. Verify the three path nodes' shared range
and lower-safety cap with the parameter checks above (using the onboard ROS domain).
Record the actual map YAML/image, exact CSV and `--config speed="$SPEED_PROFILE"`
with Terminal 4's existing asset/config arguments; begin evaluation only after
manual PF initialization and readiness. Copy the entire dataset and analyze as in
section 5, then use [section 10](#10-compare-two-analyzed-runs).

For meaningful comparison, retain matching map-image/YAML and CSV hashes, direction,
speed profile, algorithm configuration and evaluation context. Different local paths
are fine; different asset bytes are not the same scenario. The generic comparison
can report differences without matched-section alignment or asserting equivalence.
Onboard absolute localization accuracy remains unavailable without qualified GT.
The measured IMU yaw-rate adapter is still an analysis limitation; preserving the
raw evidence does not make unsupported metrics available.
