# RoboRacer engineering decisions

## 1. How to use this record

These stable IDs capture significant constraints, not ordinary implementation
details. Reconstructed on **2026-10-09** at `9d4abc0`; dates tied to Git are
committer dates, not invented discussion/approval dates. **Accepted** means the
current supported approach or explicit task choice, not hardware certification.
**Historical**, **proposed** and **superseded** mark different states. No entry
below has an identified superseding decision; add one explicitly if that changes.

Verified means inspected code/commit/artifact; Reported means unreplicated past
execution; Inferred and Unknown require labels. Rationale below is supported by
current documentation or the change itself; undocumented historical motives and
rejected alternatives are not invented. See [status](PROJECT_STATUS.md),
[milestones](DEVELOPMENT_LOG.md), and the [workflow](ENGINEERING_WORKFLOW.md)
for maintenance and evidence policy. Sources describe mechanics; commands remain
in the operational reference.

## 2. Accepted architectural and engineering choices

### DEC-001 — Share behavior profiles across simulator and onboard

- **Date/status:** 2026-08-25 (`ebb8c79`); accepted.
- **Context/choice:** divergent runtime profiles can conceal behavioral differences.
  Common component YAMLs own behavior; sparse platform adapters own clocks,
  frames, paths and explicit simulator-only gates. Shared bringup composes them.
- **Reasoning/tradeoff:** preserve a common behavior baseline while allowing real
  interface differences. Parameter ordering must remain explicit; a simulator
  adapter can still intentionally enable behavior with platform-specific evidence.
- **Alternative actually replaced:** duplicated full simulator behavior profiles,
  evidenced by the commit diff. Future evaluation variants must not silently
  change normal/onboard defaults.
- **Evidence:** Verified [bringup](../oudtra_driver_bringup/README.md),
  [shared launch](../oudtra_driver_bringup/launch/full_stack_launch.py), and
  [onboard contracts](../oudtra_driver_bringup/test/test_onboard_profile_contracts.py).

### DEC-002 — Use one published static raceline as the planner reference

- **Date/status:** 2026-08-25 (`449e3aa`); accepted.
- **Context/choice:** the followed path and Frenet planning coordinates must agree.
  The selected raceline CSV supplies published geometry for path generation and
  the sole static Frenet reference; nominal/rejoin targets use `d=0`.
- **Reasoning/tradeoff:** avoids independently selected centerline/raceline frames.
  Static-reference deviation can include legitimate obstacle avoidance; the
  reference is not the instantaneous controller target.
- **Consequences:** apply direction once at the publisher; record the stable path
  for analysis. Offline centerline remains optimizer provenance, not a second
  runtime planner input. A separate centerline reference was replaced in the diff.
- **Evidence:** Verified [planner README](../path_following_v2/README.md),
  [publisher](../centerline_tools/centerline_tools/centerline_publisher_node.py),
  [tracking contract](LOCALIZATION_SIMULATION.md#53-scorecard-definitions).

### DEC-003 — Use the localization-ready Foxy container and host source tree

- **Date/status:** 2026-09-16 (`94a6c0f`, `ec52e52`, `7cd7968`); accepted.
- **Choice/reasoning:** `/home/mzhou/f1tenth_dev` owns source edits; the existing
  `f1tenth_gym_ros_rocker` binds them into `/sim_ws` using the localization-ready
  image. This reuses the teammate simulator/PnC infrastructure and established
  Foxy dependencies. Older image and root/package instructions remain historical
  references, not authorization to replace containers.
- **Consequences:** targeted builds/source steps and installed-source freshness
  matter. Perfect-localization, standalone PF and evaluated PF+PnC workflows stay
  distinguishable. Dependency/runtime migrations require a separate task.
- **Evidence:** Verified [environment](DEVELOPMENT_ENVIRONMENT.md) and
  [container helper](../scripts/rr_container.sh); historical mount/runtime checks
  are Reported in local reports, not newly repeated.

### DEC-004 — Isolate GT in localization evaluation

- **Date/status:** 2026-09-16 truth contract (`2bb59d5`), 2026-09-18 isolation
  profile (`8db4f22`); accepted.
- **Context/choice:** exposing `/simulator/agent_status` does not alone isolate
  control. Normal full-stack simulation enables the pre-existing reverse swept
  GT gate. Evaluation explicitly disables it and redirects the controller's
  GT topic, while retaining passive observer/recorder access to simulator truth.
- **Reasoning/tradeoff:** retain normal simulation/onboard behavior and safety
  logic while preventing truth from influencing evaluated PF/PnC decisions.
  Reverse recovery in the evaluation profile therefore lacks this simulator-only
  additional check; ordinary scan/authority/odometry checks remain.
- **Consequences:** runner verifies effective parameters and GT subscribers.
  This decision does not claim GT is globally passive in every profile.
- **Evidence:** Verified [evaluation overlay](../oudtra_driver_bringup/config/localization_eval_sim.yaml),
  [normal sim override](../oudtra_driver_bringup/config/full_stack_sim.yaml),
  [lower safety](../reactive_control_v2/src/lower_safety_controller.cpp), and
  [runner](../oudtra_driver_bringup/scripts/run_localization_smoke.py).

### DEC-005 — Release PnC through observable readiness and frame/clock contracts

- **Date/status:** 2026-09-18 (`8db4f22`, PF `b11aebe`); accepted.
- **Context/choice:** use one configured map-frame `/initialpose`, matched reader,
  subsequent estimates/health/TF, bounded timeouts and actual motion checks.
  The evaluation profile uses wall time, map-frame PF odometry for lower safety,
  and the existing bounded planner wait for asynchronous scan-time TF.
- **Reasoning/tradeoff:** local historical diagnosis reports a persistent
  odometry-versus-heading frame hold plus timing/clock defects. Current code
  corroborates the frame predicate. Fixes were tested together, so isolated
  causal attribution remains limited. Readiness is observable, not a PF reset
  acknowledgement or accuracy guarantee.
- **Consequences:** no fixed sleep as startup proof, repeated truth/reset input,
  latest-TF shortcut, or TF redesign was introduced by this workflow. Manual pose
  initialization remains possible; offsets belong to initialization input.
- **Evidence:** Verified [startup contract](LOCALIZATION_SIMULATION.md#22-readiness-and-acceptance),
  [runner tests](../oudtra_driver_bringup/test/test_localization_smoke.py), and
  [pinned PF launch](../particle_filter/launch/localize_sim_launch.py).

### DEC-006 — Share semantic offline analysis with explicit evidence applicability

- **Date/status:** 2026-09-18 recording (`297673b`), 2026-09-20 shared model
  (`85702a6`); accepted.
- **Choice/reasoning:** one preserved run contract, role-based analyzer and
  scorecard support sim and no-GT evidence. GT enables absolute accuracy;
  odometry consistency is correlated evidence, never substitute truth. Use
  AVAILABLE, NOT_APPLICABLE, UNAVAILABLE_DATA and ANALYSIS_ERROR rather than zero
  for missing results. Offline analysis feeds no live controller.
- **Tradeoffs/constraints:** direct Foxy SQLite/CDR decoding fits the established
  environment; compressed/foreign contracts need explicit support. Static tracking
  must label GT versus estimated pose. Comparison suppresses deltas unless
  identity, definitions, units, evidence, coverage and timing are compatible;
  no composite score or synthetic-onboard performance claim is justified.
- **Evidence:** Verified [analyzer](../oudtra_driver_bringup/scripts/analyze_localization_run.py),
  [comparison](../oudtra_driver_bringup/scripts/compare_localization_runs.py),
  [measurement tests](../oudtra_driver_bringup/test/test_localization_measurements.py),
  and [shared evaluation](LOCALIZATION_SIMULATION.md#5-shared-simulationonboard-evaluation).

### DEC-007 — Use spatial residuals for headline smoothness

- **Date/status:** 2026-09-23 (`c12c4ec`); accepted, replaces earlier headline
  steering-rate/physical-dynamics presentation in the shared scorecard.
- **Choice/reasoning:** speed/steering command and vehicle forward-speed/yaw-rate
  oscillation are measured around a local trend in raw-odometry driven-distance
  coordinates. Legacy wall-stamped simulation does not qualify physical timing
  for acceleration/jerk; these remain secondary unavailable diagnostics.
- **Tradeoff:** spatial evidence supports irregular publication sampling and
  separates local wiggle from intended maneuvers, but omits stationary chatter
  and proves neither tracking quality nor time-frequency vibration. Settings
  are analysis-only, with units and coverage visible.
- **Evidence:** Verified [metric definitions](LOCALIZATION_SIMULATION.md#53-scorecard-definitions),
  [analysis settings](../oudtra_driver_bringup/config/evaluation_analysis.yaml)
  and measurement tests; local simulation metrics corroborate four available rows.

### DEC-008 — Record onboard passively with actuator-path command evidence

- **Date/status:** 2026-09-23 (`6de72e7`); accepted; hardware verification pending.
- **Choice/reasoning:** an additional recorder terminal observes existing operation,
  marks an operator evaluation interval and preserves startup/provenance. It
  launches no vehicle stack, publishes no commands/initialpose, and changes no
  parameters. `/ackermann_cmd` is the shared final-command role; upstream `/drive`
  remains debugging evidence because the mux may choose another source.
- **Constraints/tradeoffs:** preserve VESC RPM-derived speed and measured IMU yaw
  separately. A split vehicle-state adapter is still required; model-derived
  odometry yaw cannot be called measured response. Servo-command echo is not
  steering feedback. Required preflight is conservative and must be checked on
  the actual car before claiming a usable hardware contract.
- **Evidence:** Verified [recorder](../oudtra_driver_bringup/scripts/record_onboard_localization.py),
  [onboard contract](../oudtra_driver_bringup/config/onboard_localization_recording.yaml),
  [tests](../oudtra_driver_bringup/test/test_onboard_recording.py); external source
  at `f1tenth_system` `5a46e19` corroborates mux/VESC topics. Mock results are
  Reported, not hardware performance.

### DEC-009 — Keep track assets separate from evaluation integration

- **Date/status:** 2026-09-25 (`98b24c8`, PF `dfd07cb`); accepted.
- **Choice/reasoning:** explicit scenario ID, simulator config, PF map,
  raceline/direction and PF base config flow through the existing runner.
  `localization_eval_sim.yaml` owns integration only; an explicit PF `map_yaml`
  overrides the legacy fallback resolved from the selected runtime config.
- **Consequences:** preflight verifies files/map agreement/reference geometry and
  records asset hashes. New-track source edits should normally be unnecessary;
  safe path generation is a separate gate that finite CSV preflight cannot prove.
  Backward-compatible IFAC defaults remain outside the maintained explicit path.
- **Evidence:** Verified [scenario contract](LOCALIZATION_SIMULATION.md#23-interfaces-and-evaluation-profile),
  runner, [PF launch tests](../particle_filter/test/test_launch_time_contracts.py),
  and smoke tests. Tempo map existence does not satisfy the raceline requirement.

### DEC-010 — Default benchmark to headless wall-time operation

- **Date/status:** 2026-09-27 (`1b9a5f4`); accepted.
- **Context/choice:** the runner inherited unconditional RViz. It now requests
  no RViz by default; manual bridge launch retains RViz and the runner can opt in.
  Gym rendering is absent from this bridge path. Record passive timing separately
  without changing control rates, physics or introducing `/clock`.
- **Reasoning/tradeoff:** measured historical visual/headless repeats motivate
  removing avoidable display overhead, not maximizing simulation speed. Bag/
  observer ages combine queue, processing and transport; no qualified physics
  clock means real-time factor stays unavailable. Some PnC marker work remains.
- **Evidence:** Verified [launch/test](../f1tenth_gym_ros/test/test_slam_launch_headless.py),
  [timing analyzer](../oudtra_driver_bringup/scripts/analyze_sim_timing.py), and
  [timing limits](LOCALIZATION_SIMULATION.md#25-headless-execution-and-timing-evidence).
  Local result files corroborate two headless PASS runs and one visual cleanup FAIL.

### DEC-011 — Persistent progress complements existing documentation

- **Date/status:** 2026-10-09; accepted by this task, documentation uncommitted.
- **Choice/reasoning:** status is a dated current assessment, the milestone log
  retains meaningful evolution, and stable decision IDs preserve significant
  rationale. They link existing sources of truth instead of duplicating operating
  procedures or accumulating task reports as permanent history.
- **Consequences:** maintenance accompanies meaningful task completion within
  scope. Add one milestone per meaningful outcome/investigation; update decisions
  only for significant changes; explicitly correct history and label evidence.
  Tiny edits require no entry, and maintenance never authorizes commits, pushes,
  or moving/deleting historical artifacts.
- **Evidence:** this task's explicit requirements and updated
  [AGENTS.md](../AGENTS.md), [workflow](ENGINEERING_WORKFLOW.md#persistent-engineering-records)
  and [documentation map](README.md). Future changes may supersede a decision
  with a new ID while keeping the original traceable.
