# RoboRacer engineering milestone log

## 1. Reading and maintenance

Reconstructed on **2026-10-09** against `9d4abc0` on
`mingguang_localization_robustness_ai_workspace`. Dates below are Git **committer
dates**, including their recorded local calendar date, unless explicitly labelled
as experiment or reconstruction dates. The baseline tag targets `fb945e2`
(2026-08-29) and was created on 2026-09-13; those are different events.

Evidence labels follow [project status](PROJECT_STATUS.md): Verified refers to
inspected code/commits/artifacts; Reported execution was not rerun here; Inferred
and Unknown are explicit. Commit IDs can be inspected with `git show <id>`.
Local merge messages identify PRs but their remote discussions/CI were not queried.
Read [decisions](DECISIONS.md) for lasting constraints and the existing technical
guides for usage. Append meaningful milestones; retain history and add dated
corrections when needed. This is not a commit-by-commit transcript.

## 2. Foundations before the post-IFAC boundary

### 2025-07-31 and 2025-11-25 — Legacy following and fallback integration

- **Packages/objective:** legacy path following, reactive safety and arbitration;
  combine primary following with fallback control.
- **Verified implementation:** PR #8 merge `1e374e4` adds path-following work;
  PR #12 record `8ca7dc4` changes the legacy planner/boundary/safety stack and
  adds drive arbitration. These precede current V2 packages.
- **Status:** historical implementation; runtime and hardware outcomes **Unknown**
  from inspected evidence. Do not transfer current V2 validation to these versions.

### 2025-12-04 and 2026-04-09 — In-tree simulator and localization environment

- **Objective:** develop ROS Gym and localization/mapping within the workspace.
- **Verified:** PR #14 merge `92aa154` records simulator integration/startup work;
  PR #15 merge `d1b70cd` adds the SLAM bridge/launch, localization-ready startup,
  PF, range library and SLAM submodule pointers. See [.gitmodules](../.gitmodules).
- **Status:** integration present; these merges alone establish no successful
  mapping experiment or hardware localization result.

### 2026-08-06 — V2 integrated autonomy stack

- **Objective/packages:** `oudtra_driver_bringup`, path/reactive/arbitration V2;
  compose primary following, guard, selection and final lower safety.
- **Verified:** `be93762` adds bringup and arbitration V2 and updates follower/
  reactive integration. Current [bringup README](../oudtra_driver_bringup/README.md)
  describes the chain and single final command gateway.
- **Status:** simulator testing is **Reported** in the commit title; no matching
  original runtime trace was independently identified. Hardware coverage Unknown.

### 2026-08-22–26 — Optimized raceline, shared profiles and persistent planning

- **Objective:** generate a smooth validated reference and reuse common behavior
  across simulator/onboard, with persistent avoidance/rejoin planning.
- **Verified:** `fcaa3e8` adds optimizer and tests; `449e3aa` selects the raceline
  as the sole Frenet reference; `ebb8c79` unifies launch/profile layering;
  `529bbf7`/`621741c` add and harden the static-obstacle fast path. See
  [planner](../path_following_v2/README.md), [geometry tools](../centerline_tools/README.md),
  [DEC-001/002](DECISIONS.md).
- **Evidence/status:** stored IFAC validation flags and current asset hashes
  agree. The IFAC asset README's five-lap/latency claims are **Reported**, not
  rerun or sufficient for current hardware performance. PR #16 merge `0734171`
  on 2026-08-27 brings this development into the inspected local `main` history.

### 2026-08-29 — IFAC localization fixture / baseline software state

- **Verified:** `fb945e2` selects the IFAC simulator/PF fixture; this is the target
  of `post_ifac_ai_baseline_v0`, subsequently tagged on 2026-09-13.
- **Status:** historical boundary, not a blanket certification. PF clock/health
  and onboard parameters already exist in the pinned submodule's earlier history;
  they were not all introduced by the later AI-assisted steps.

## 3. Post-baseline integration and evaluation

### 2026-09-16 — Canonical environment, config selection and simulator truth

- **Objective:** preserve teammate workflows while enabling explicit PF simulator
  configuration and independent true-state observation.
- **Verified:** `94a6c0f`/`ec52e52` add guardrails/environment guidance;
  `7cd7968` adds `config_file`; `2bb59d5` shares diagnostic serialization between
  bridges. See [environment](DEVELOPMENT_ENVIRONMENT.md),
  [agent-status code](../f1tenth_gym_ros/f1tenth_gym_ros/agent_status.py), and its
  [contract tests](../f1tenth_gym_ros/test/test_agent_status.py).
- **Status:** targeted builds and actual scan/odom/PF/GT messages are **Reported**
  in local Step 1/2 reports. Source contract is Verified. Global control-GT
  isolation was not established by simply adding the GT topic.

### 2026-09-17 — Engineering process and GT dependency understanding

- **Verified:** `c388235`, `b7653ab`, `5d48bd5` establish workflow/report, Mermaid
  compatibility and artifact triage guidance.
- **Investigation:** local Step 2.5 report describes lower safety's pre-existing
  GT-dependent reverse swept gate. Current code/config corroborate default-off
  base behavior and normal full-stack-sim override-on. The report's effective
  runtime parameter/graph check is **Reported**; forced recovery was not tested.
- **Status:** functional gate preserved. Evaluation isolation must be explicit
  ([DEC-004](DECISIONS.md#dec-004--isolate-gt-in-localization-evaluation)). Exact
  audit execution date is uncertain; 2026-09-17 dates the surrounding Git guidance,
  not a newly invented audit timestamp.

### 2026-09-18 — PF closed-loop startup and bounded motion

- **Problem:** local investigation reproduced zero final drive despite nonzero
  selected commands. The Reported direct blocker was lower-safety odometry frame
  versus map-heading geometry; asynchronous scan-time TF and mixed clocks also
  failed contracts. Their predicates/config are Verified in current source.
- **Implementation:** `8db4f22` and PF `b11aebe` add a separate evaluation overlay,
  map-frame PF odometry to lower safety, wall time, bounded TF wait, PF-health
  gating, one-shot initialization and readiness-driven runner. Commands are
  maintained via `2814336`/`5389fc1`.
- **Status:** 15-second motion and clean stop **Reported**; later preserved IFAC
  bags corroborate the combined workflow. Corrections were validated together;
  no independent causal experiment proves each correction necessary or sufficient.
  See [startup contract](LOCALIZATION_SIMULATION.md#2-closed-loop-localization-workflow)
  and [smoke tests](../oudtra_driver_bringup/test/test_localization_smoke.py).

### 2026-09-18 — Preserved simulation data and offline baseline

- **Objective:** extend that same runner with selected recording and offline metrics.
- **Verified:** `297673b` adds recording contract/helper, SQLite/CDR analyzer and
  [measurement tests](../oudtra_driver_bringup/test/test_localization_measurements.py).
  The preserved final baseline has 30,449 bag messages across nine topics;
  smoke, recording and analysis outcomes all record PASS for about 15 seconds.
- **Status:** historical simulation evidence corroborated; deterministic reanalysis
  and regression success **Reported**, not rerun for this assessment. No accuracy
  acceptance threshold or lap evaluation follows from pipeline PASS.

### 2026-09-20 — Shared no-GT scorecard, comparison and navigation

- **Verified:** despite its documentation-oriented title, `85702a6` also introduces
  the schema-2 shared analyzer, analysis settings and comparison script. Explicit
  applicability/evidence/coverage preserve unavailable versus inapplicable results.
- **Evidence:** local synthetic onboard-style bag/results and comparison outputs
  corroborate no-GT handling; local Step 3.2B report records 54 tests and unchanged
  legacy metrics. It is synthetic contract validation, not onboard performance.
- **Status:** shared analysis implemented; real matched sim/onboard comparison
  remains open. See [shared contract](LOCALIZATION_SIMULATION.md#5-shared-simulationonboard-evaluation)
  and [comparison tests](../oudtra_driver_bringup/test/test_evaluation_comparison.py).

### 2026-09-23 — Spatial smoothness and technical-report readability

- **Problem/choice:** wall publication timestamps do not qualify physical
  acceleration/jerk; use local-trend residuals in driven-distance coordinates for
  four command/vehicle oscillation signals.
- **Verified:** `c12c4ec` changes metrics/tests and generated report mechanics,
  terminology and human labels. Local smoothness run created on **2026-09-22**
  has 26,700 bag messages, PASS outcomes and four available oscillation metrics.
  That experiment date differs from the Git milestone date.
- **Status:** artifact-backed simulation baseline; 56 tests and deterministic
  regeneration are **Reported**. No new performance thresholds or physical-time
  dynamics evidence. See [metric definitions](LOCALIZATION_SIMULATION.md#53-scorecard-definitions).

### 2026-09-23 — Passive onboard measurement readiness

- **Verified:** `6de72e7` adds required/optional onboard contract, preflight,
  operator interval, provenance and clean finalization without control publishers.
  External `f1tenth_system` source at `5a46e19` corroborates `/ackermann_cmd`
  as mux-to-VESC input, distinct from upstream `/drive`.
- **Status:** local report records 67 tests, six CTest targets and mock recording/
  graph checks; mock telemetry used a substitute type because the simulation
  image lacks `vesc_msgs`. **Real-car validation still pending.** Split odometry
  speed/IMU yaw analysis adapter remains open; no measured steering feedback claim.
  See [onboard contract](LOCALIZATION_SIMULATION.md#55-onboard-recording-contract).

### 2026-09-25 — Track-explicit integration and provenance

- **Verified:** `98b24c8` renames `localization_eval_sim.yaml`; PF `dfd07cb`
  makes explicit `map_yaml` authoritative and resolves legacy fallback at launch
  time. Runner forwards scenario/map/raceline/direction with preflight and hashes.
- **Evidence/status:** preserved explicit IFAC run contains 24,750 messages,
  PASS outcomes and only observer/recorder GT readers. Local report records 79
  relevant tests and repeatable analysis. New-track geometry still requires its
  own validation; preflight alone checks no vehicle-envelope feasibility.

### 2026-09-27 — Headless evaluation and passive timing characterization

- **Problem/implementation:** `1b9a5f4` removes unconditional RViz from the
  automated path through an explicit launch switch, preserving manual default.
  Offline timing remains separate from the vehicle/localization scorecard.
- **Evidence:** two local headless 10-second bags/results record PASS and roughly
  20 Hz drive. One RViz repeat records stationary cleanup FAIL although recording
  and analysis passed. The local report records 100 targeted tests and broader
  Gym CLI/lint failures.
- **Status:** short historical runs corroborated; no isolated PF computation or
  physics real-time-factor measurement. Interrupted runs may lack scan observer
  timing. See [timing contract](LOCALIZATION_SIMULATION.md#25-headless-execution-and-timing-evidence).

## 4. New-track investigation and persistent tracking

### 2026-10-02 experiment; 2026-10-07 map commit — Tempo Coffee unresolved

- **Verified current state:** `9d4abc0` commits only the new PGM/YAML. No Tempo
  simulator config or generated raceline remains in the assessed tracked tree.
- **Local evidence:** archived metadata timestamps support the 2026-10-02 runs.
  The temporary handoff reports centerline success and optimizer rejection under
  unchanged corridor/curvature gates. Archived results corroborate a 10-second
  provisional centerline PASS (14.824 m, no collision, one emergency entry), and
  a requested 20-second run stopping after 7.59 seconds with collision.
- **Status:** experiment implementation rolled back; no validated optimized
  raceline. Geometry/control limitations warrant investigation; a causal link to
  a particular optimizer failure is **Inferred**, not isolated. Local artifacts
  and reports are uncommitted/temporary and are not linked as portable evidence.
  The unsuccessful result must remain visible before promoting any new workflow.

### 2026-10-09 reconstruction — Persistent state, milestones and decisions

- **Objective/result:** introduce these three tracking records and connect them to
  existing rules/navigation without replacing technical documentation or reports.
- **Verification:** Git/source/report inspection, read-only bag message counts,
  stored outcome inspection and IFAC asset-manifest hash checks. No runtime,
  build/test or hardware result was newly produced. Documentation checks passed:
  94 local link targets/anchors, sequential sections, 11 unique decision IDs,
  whitespace and `git diff --check`. Existing reports/data were preserved without
  reorganization.
- **Status:** documentation drafted for review, uncommitted. Future meaningful
  tasks maintain state/milestones/decisions within their authorized scope
  ([DEC-011](DECISIONS.md#dec-011--persistent-progress-complements-existing-documentation)).
