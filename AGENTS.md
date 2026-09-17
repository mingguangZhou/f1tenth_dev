# RoboRacer Development Instructions

## Repository purpose

This repository is the main development workspace for the RoboRacer / F1TENTH software stack.

The current development baseline uses ROS 2 Foxy and a localization-in-the-loop simulation environment containing the F1TENTH Gym simulator, particle-filter localization, SLAM dependencies, path following, reactive control, drive arbitration, and related tools.

## Source of truth

The authoritative source tree is the host Git repository:

`/home/mzhou/f1tenth_dev`

Edit source files in the host repository only.

The ROS workspace inside Docker uses bind mounts from this repository. Files under `/sim_ws/src/...` generally refer to the same host source files, but source edits should still be made through the host repository to keep Git ownership and file permissions predictable.

## Primary simulation container

Use the running container:

`f1tenth_gym_ros_rocker`

for ROS 2 Foxy builds, ROS commands, simulation execution, and related tests unless a task explicitly specifies another environment.

The container is based on the localization-ready simulation image and includes the dependencies required for particle-filter localization and SLAM-related development.

ROS workspace inside the container:

`/sim_ws`

ROS distribution:

`foxy`

## Development workflow

Follow [Engineering workflow](docs/ENGINEERING_WORKFLOW.md), scaled to the task.

* Start from a concrete purpose, expected behavior, and explicit non-goals.
* Choose the simplest sufficient design; reuse existing contracts and tools before duplicating them.
* Favor readable code, explicit data flow, and clear ownership of state and side effects.
* Handle realistic failure modes without speculative abstractions or unnecessary complexity.
* Keep changes focused; do not introduce unrelated refactoring.

Before modifying code:

* inspect the relevant implementation and configuration;
* check the current Git branch and working-tree state;
* understand which packages are affected.

After modifying code:

* inspect the Git diff;
* run the smallest relevant build/test first;
* report build or test failures without hiding them;
* avoid unrelated cleanup or refactoring unless explicitly requested.

Prefer targeted builds such as:

`colcon build --packages-select <package>`

or:

`colcon build --packages-up-to <package>`

instead of rebuilding the entire workspace unless full-workspace validation is explicitly required.

## Docker usage

Prefer interacting with the existing localization container through explicit `docker exec` commands or repository helper scripts.

Do not install Codex, Node.js, development tools, or Git credentials inside the simulation container unless explicitly requested.

Do not rebuild, stop, remove, replace, or otherwise alter Docker containers or images unless the task explicitly requires it.

## Safety and approval boundaries

Do not perform the following without explicit user approval:

* `sudo`
* OS-level package installation or removal
* destructive Docker operations
* Docker image deletion or pruning
* destructive Git operations
* `git reset --hard`
* `git clean -fd`
* branch deletion
* force pushes
* commits
* pushes
* pull-request merges
* modification of Git history
* changes to host system configuration

Normal source edits, targeted builds, tests, and read-only inspection should remain scoped to the task being performed.

## Git policy

Do not commit or push changes unless explicitly requested.

Before presenting work as complete, report:

* files changed;
* relevant build/test results;
* remaining warnings or failures;
* current Git status.

Do not modify unrelated files merely to obtain a clean working tree.

## Compatibility

Preserve ROS 2 Foxy compatibility unless a task explicitly changes the target platform.

Avoid introducing dependencies that are unavailable in the established simulation or onboard environment without first discussing them.

## Validation philosophy

Prefer measurable validation over visual inspection alone.

When practical, changes should eventually be evaluated using reproducible simulation scenarios, logs, metrics, regression tests, or rosbag replay rather than relying only on manual driving observations.

Do not invent performance improvements or claim success without supporting evidence from the relevant test or measurement.

## Documentation structure

Keep this file focused on persistent repository-wide rules.

Engineering reports are temporary and uncommitted by default unless explicitly requested otherwise. Use the [report template](docs/templates/CODEX_ENGINEERING_REPORT_TEMPLATE.md) when a report is needed.

Put detailed architecture, operating procedures, simulator instructions, scenario definitions, evaluation methodology, and subsystem-specific information in dedicated documentation or scripts rather than continuously expanding this file.

More specific `AGENTS.md` files may later be added inside individual packages when package-specific instructions are needed.

## Definition of done

Work is complete when the objective is met within scope, the diff is reviewed, relevant checks support the claimed behavior, permanent documentation reflects changed contracts or usage, and limitations plus Git state are reported. Disclose failed or omitted validation and its impact; do not claim unverified behavior. Completion does not imply permission to commit or push.
