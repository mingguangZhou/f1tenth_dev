# RoboRacer Development Environment

This document describes the canonical local development environment and common development commands for the RoboRacer / F1TENTH workspace.

It is intended as a reference for both human developers and AI-assisted development tools.

For the development process and completion criteria, see [Engineering workflow](ENGINEERING_WORKFLOW.md).

## 1. Host repository

Primary repository:

```bash
cd /home/mzhou/f1tenth_dev
```

Current AI-assisted integration branch:

```text
mingguang_localization_robustness_ai_workspace
```

Check repository state:

```bash
git status
git branch --show-current
```

The host Git repository is the authoritative source tree.

Source code should be edited on the host rather than directly inside the Docker container.

---

## 2. Localization-ready simulation environment

The primary development simulation environment is the localization-ready F1TENTH Docker image:

```text
f1tenth_gym_ros_localization_ready:latest
```

The preferred GPU-enabled runtime uses Rocker.

From:

```bash
cd /home/mzhou/f1tenth_dev/f1tenth_gym_ros
```

start the localization-ready GPU environment with:

```bash
./f1tenth_rocker_loc_start.sh
```

The running container is expected to be:

```text
f1tenth_gym_ros_rocker
```

Check:

```bash
docker ps
```

The container uses ROS 2 Foxy and exposes the ROS workspace at:

```text
/sim_ws
```

The host packages are bind-mounted into:

```text
/sim_ws/src
```

---

## 3. Container helper

From the repository root:

```bash
cd /home/mzhou/f1tenth_dev
```

check the container:

```bash
./scripts/rr_container.sh status
```

Open an interactive container shell:

```bash
./scripts/rr_container.sh shell
```

Run a command in the container:

```bash
./scripts/rr_container.sh exec \
  'source /opt/ros/foxy/setup.bash && cd /sim_ws && colcon list'
```

The helper uses:

```text
f1tenth_gym_ros_rocker
```

by default.

The container name may be overridden with:

```bash
ROBORACER_CONTAINER=<container> ./scripts/rr_container.sh ...
```

The workspace may be overridden with:

```bash
ROBORACER_WS=<workspace> ./scripts/rr_container.sh ...
```

---

## 4. ROS package discovery

List packages visible in the ROS workspace:

```bash
./scripts/rr_container.sh exec \
  'source /opt/ros/foxy/setup.bash && cd /sim_ws && colcon list'
```

The localization-ready environment should include packages such as:

```text
f1tenth_gym_ros
particle_filter
path_following_v2
reactive_control_v2
drive_arbitration_v2
slam_toolbox
range_lib
```

---

## 5. Building packages

Prefer targeted builds during development.

Build one package:

```bash
./scripts/rr_container.sh exec \
  'source /opt/ros/foxy/setup.bash && cd /sim_ws && \
   colcon build --packages-select <package_name>'
```

Example:

```bash
./scripts/rr_container.sh exec \
  'source /opt/ros/foxy/setup.bash && cd /sim_ws && \
   colcon build --packages-select drive_arbitration_v2'
```

Build a package together with required workspace dependencies:

```bash
./scripts/rr_container.sh exec \
  'source /opt/ros/foxy/setup.bash && cd /sim_ws && \
   colcon build --packages-up-to <package_name>'
```

Avoid full-workspace builds during normal edit-test iterations unless they are needed for integration validation.

---

## 6. Entering a built ROS environment

After a normal build:

```bash
./scripts/rr_container.sh shell
```

Then inside the container:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
```

ROS commands and launches can then use the workspace installation.

---

## Canonical simulation workflows

All current post-IFAC simulation development uses the canonical
`f1tenth_gym_ros_localization_ready` image in `f1tenth_gym_ros_rocker`.
The older `f1tenth_gym_ros` image is legacy/reference infrastructure;
`f1tenth_gym_ros_localization_ready` is the canonical environment for current
localization work.

Run these two verified workflows separately. Each terminal below is a shell
inside the existing container (open with `./scripts/rr_container.sh shell`).

### Perfect-localization reference

Terminal 1:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select centerline_tools path_following_v2 reactive_control_v2 drive_arbitration_v2 oudtra_driver_bringup
source install/local_setup.bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

For dev-laptop commands to build, run and stop the complete PF + PnC loop,
inspect results, and repeat essential tests, see
[Localization simulation: manual commands](LOCALIZATION_SIMULATION.md#manual-commands-from-the-dev-laptop).
For preserved bags, offline metrics and plots, use the same document's
[measurement baseline commands](LOCALIZATION_SIMULATION.md#record-one-simulation-measurement-baseline).
The same document's [deliverables guide](LOCALIZATION_SIMULATION.md#finding-the-engineering-deliverables)
shows the exact host paths for the report, metrics JSON, plots, metadata, logs,
and rosbag produced by a run.

### PF-localization simulator

Terminal 1:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_slam_launch.py \
  config_file:=/sim_ws/src/f1tenth_gym_ros/config/sim_ifac_roboracer.yaml
```

Terminal 2:

```bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --packages-select particle_filter
source install/local_setup.bash
ros2 launch particle_filter localize_sim_launch.py
```

---

## 7. Codex CLI

Codex is installed on the host through the nvm-managed Node.js environment.

Start it from the repository root:

```bash
cd /home/mzhou/f1tenth_dev
codex
```

Codex should operate on the host Git checkout.

ROS builds and runtime commands should be executed through the existing Docker environment.

Codex should not normally be installed inside the simulation container.

Repository-wide AI operating rules are defined in:

```text
AGENTS.md
```

---

## 8. Git workflow

Before development:

```bash
git status
git branch --show-current
```

After AI-assisted or manual edits:

```bash
git diff
git status
```

Build and test the relevant packages before accepting a change.

Commits and pushes should remain explicit engineering checkpoints rather than automatic consequences of code generation.

---

## 9. Baseline

The starting post-IFAC AI-assisted development baseline is tagged:

```text
post_ifac_ai_baseline_v0
```

Inspect it with:

```bash
git show --no-patch --decorate post_ifac_ai_baseline_v0
```

This tag identifies the software state before the new AI-assisted development and validation workflow was introduced.

---

## 10. Documentation maintenance rule

When a development procedure becomes stable and repeatable:

* encode executable behavior in a dedicated script;
* keep configuration values in configuration files when practical;
* document the canonical workflow here or in a subsystem-specific document;
* update `AGENTS.md` only when repository-wide AI operating rules change.

Do not rely on chat history as the only record of a required development procedure.

This document should describe the supported workflow rather than every experimental command ever attempted.

### Maintain human-operable commands

Keep copyable manual commands for key workflows and essential repeatable tests
in the relevant operational document. Label host versus container commands,
prerequisites/source steps, startup and clean stop, and expected success/failure
results. Link to that single home from this environment guide rather than
maintaining duplicate command lists.

When code, launch arguments, configuration, paths, or tests change, update their
operating commands in the same change. Check commands against the current
implementation and validate affected commands when practical; state any unverified
steps. Keep guidance concise and organized by user action, not by development
phase or temporary report. Temporary reports are not the operating manual.

### Make deliverables easy to find

Every workflow that creates an engineering artifact must state the exact output
directory and distinguish host paths from container paths. Identify the human
summary, machine-readable result, plots, raw data, logs, and provenance files
individually. Keep a stable example path or discovery command in the owning
operational document, and update it when output names or locations change.
