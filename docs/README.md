# RoboRacer engineering documentation

## 1. How to use this documentation

Start here when setting up the workspace, running a supported workflow, or
interpreting an evaluation result. Each subject has one permanent source of truth;
the links below avoid duplicating commands and metric definitions.

## 2. Document map

| Question | Source of truth |
| --- | --- |
| Which host, container, ROS distribution, and workspace should I use? | [Development environment](DEVELOPMENT_ENVIRONMENT.md) |
| What engineering process and validation standard applies? | [Engineering workflow](ENGINEERING_WORKFLOW.md) |
| Which commands run the simulator, PF/full stack, tests, recording, and offline analysis? | [Operational command reference](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md) |
| How does localization-enabled simulation start, record, and evaluate a run? | [Localization simulation and shared evaluation](LOCALIZATION_SIMULATION.md) |
| Where are metric definitions and sim/onboard evidence limits? | [Shared evaluation contract](LOCALIZATION_SIMULATION.md#5-shared-simulationonboard-evaluation) |
| What should a temporary engineering handoff contain? | [Engineering report template](templates/CODEX_ENGINEERING_REPORT_TEMPLATE.md) |

For package-specific architecture and behavior, use the owning package README and
source files. Repository-wide AI operating rules remain in [AGENTS.md](../AGENTS.md).

## 3. Reports and validation artifacts

These artifact classes have different lifetimes:

| Class | Location and purpose |
| --- | --- |
| Permanent documentation | Source-controlled files under `docs/`; supported environment, commands, contracts, and interpretation guidance |
| Preserved run artifacts | Git-ignored `oudtra_driver_bringup/runs/<descriptive-run-id>/`; raw bag, metadata, generated scorecard, JSON, plots, logs, and copied configuration |
| Temporary engineering handoff | Repository-root `PHASE*_CODEX_REPORT.md`; task evidence that remains uncommitted unless explicitly promoted |

The canonical local example is
`oudtra_driver_bringup/runs/ifac_pf_closed_loop_localization_baseline_final/`.
When present locally, open its [human scorecard](../oudtra_driver_bringup/runs/ifac_pf_closed_loop_localization_baseline_final/report.md)
and adjacent `metrics.json`. See [artifact discovery and regeneration](LOCALIZATION_SIMULATION.md#4-preserved-run-artifacts-and-deliverables)
for host/container paths, directory contents, and reanalysis commands. Bags and
generated datasets are not committed merely to make them discoverable.

## 4. Maintenance rule

Keep commands in the operational reference, environment facts in the environment
guide, engineering policy in the workflow, and metric/recording definitions in the
localization evaluation document. Generated reports should link to those sources
instead of copying their instructions. Update navigation, operating commands, and
artifact paths in the same change when their contracts move.
