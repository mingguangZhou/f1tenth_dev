# RoboRacer project engineering record

## 1. Current engineering status

The repository implements short IFAC PF-localized closed-loop simulation,
recording and offline evaluation, with historical run artifacts supporting that
workflow. A passive onboard recorder exists; its physical-car validation is not
established. Tempo Coffee's committed track assets remain the map pair, without a validated
raceline or maintained simulation workflow. The offline workbench now records quantified
Tempo search failure without promoting an invalid candidate.

**Assessment:** 2026-10-10, `mingguangZhou/f1tenth_dev`, branch
`mingguang_localization_robustness_ai_workspace`, revision
[d28f615](https://github.com/mingguangZhou/f1tenth_dev/commit/d28f6159fb514be35679e27f3177144d1578360a)
plus the uncommitted offline-workbench changes described below.
Pinned PF: [dfd07cb](https://github.com/mingguangZhou/particle_filter/commit/dfd07cb895e854a7a5ca872339a0f567e24fa5ba).
Current workbench evidence includes a targeted Foxy package build, offline
regression tests and fresh IFAC/Tempo generation. No new vehicle simulation or
hardware test was run. Historical research was reconstructed on 2026-10-09 and
consolidated on 2026-10-10; prior artifacts do not certify current installed binaries.

**Human-confirmed current objective:** implement the offline raceline workbench:
Charlie baseline → bounded tuning → existing manual editor → common production
validator. No PF/PnC/vehicle tuning or automatic simulation launch. **Subsequent
objective, strategic priorities and development sequence: Not specified / awaiting
human direction.** Open issues below are observations, not assigned work.

| Capability | Demonstrated evidence and boundary |
| --- | --- |
| Foxy / localization-ready Rocker environment | Implemented host-source/mount conventions inspected; past container builds/runtime checks **Reported**, not rerun. |
| Normal and PF Gym bridges | Explicit config selection and shared simulator-truth diagnostic contract **Verified in source/tests**; past message-flow checks **Reported**. |
| IFAC geometry / V2 PnC | Stored IFAC validation hashes/flags corroborate assets; bounded closed-loop bags corroborate historical **Simulation/runtime validation**. Five-lap performance is **Reported**; full recovery coverage and hardware results **Unknown**. |
| Offline raceline workbench | **Build/test and offline fixture validated:** common automatic/manual gates, hash-qualified outputs, deterministic baseline/search, Matplotlib construction. Fresh explicit IFAC profile PASS; Tempo baseline + 16 trials exhausted with no validated CSV. Interactive race-day editing not yet human-validated. |
| PF initialization / closed-loop integration | Readiness, one-shot pose and health gating implemented; historical short IFAC motion corroborated. No reset acknowledgement or accuracy guarantee. |
| Recording / shared offline scorecard | Simulation and synthetic no-GT artifacts inspected; regression success **Reported**. Synthetic onboard fixtures are not hardware results or sim/onboard equivalence. |
| Passive onboard recorder | Passivity verified in source; **Build/test and mock/runtime validation Reported**. Hardware validation not established; measured speed/IMU adapter remains missing. |
| SLAM / range libraries | Dependencies and launch/config support implemented; successful current mapping experiment **Unknown**. |

**Evidence terms:** Verified means inspected code, commits, contracts or artifact
content; it does not imply execution. Reported means a past execution claim not
independently repeated in this assessment. Inferred means a reasoned but unconfirmed
conclusion. Unknown means insufficient evidence. Build/test, simulation and hardware
validation remain separate levels.

Use the [documentation map](README.md), [environment](DEVELOPMENT_ENVIRONMENT.md),
[operational commands](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md) and
[evaluation contracts](LOCALIZATION_SIMULATION.md) for technical details.

## 2. Development milestones

Dates are Git committer dates unless an experiment/reconstruction date is stated.
Merge records establish code inclusion, not CI, runtime or hardware success; remote
PR discussions/CI were not independently checked. Entries preserve meaningful
changes and failures rather than every commit.

### Foundations and historical boundary

| Date | Milestone, evidence and limitation |
| --- | --- |
| 2025-07-31; 2025-11-25 | **Legacy following and fallback integration.** PR #8 merge [1e374e4](https://github.com/mingguangZhou/f1tenth_dev/commit/1e374e4) and PR #12 record [8ca7dc4](https://github.com/mingguangZhou/f1tenth_dev/commit/8ca7dc4) add following/reactive/arbitration work. Historical implementation verified; runtime/hardware outcomes unknown. |
| 2025-12-04; 2026-04-09 | **Gym and localization environment.** PR #14 [92aa154](https://github.com/mingguangZhou/f1tenth_dev/commit/92aa154), PR #15 [d1b70cd](https://github.com/mingguangZhou/f1tenth_dev/commit/d1b70cd) integrate Gym, SLAM bridge and PF/range/SLAM dependencies. Source integration verified; no successful mapping/hardware experiment established by the merges. |
| 2026-08-06 | **V2 integrated autonomy.** [be93762](https://github.com/mingguangZhou/f1tenth_dev/commit/be93762) adds bringup/arbitration V2 and updates path/reactive integration. Simulator testing reported in commit title; original trace and hardware coverage unknown. See [bringup](../oudtra_driver_bringup/README.md). |
| 2026-08-22–26; merge 08-27 | **Optimized reference and persistent planning.** [fcaa3e8](https://github.com/mingguangZhou/f1tenth_dev/commit/fcaa3e8) optimizer; [449e3aa](https://github.com/mingguangZhou/f1tenth_dev/commit/449e3aa) sole raceline reference; [ebb8c79](https://github.com/mingguangZhou/f1tenth_dev/commit/ebb8c79) shared profiles; [529bbf7](https://github.com/mingguangZhou/f1tenth_dev/commit/529bbf7)/[621741c](https://github.com/mingguangZhou/f1tenth_dev/commit/621741c) static-obstacle fast path/hardening; PR #16 [0734171](https://github.com/mingguangZhou/f1tenth_dev/commit/0734171). Stored [IFAC asset](../centerline_tools/output_backup/ifac_roboracer/README.md) hashes match passing flags; five-lap/latency claims reported, not current hardware proof. |
| 2026-08-29; tag 09-13 | **Post-IFAC boundary.** [fb945e2](https://github.com/mingguangZhou/f1tenth_dev/commit/fb945e2) is the `post_ifac_ai_baseline_v0` target; tag creation is a separate event. PF clock/health/onboard configuration partly predates later AI integration. This is a software boundary, not certification. |

### Localization integration and measurement

| Date | Milestone, evidence and limitation |
| --- | --- |
| 2026-09-16 | **Canonical environment, config selection and truth interface.** [94a6c0f](https://github.com/mingguangZhou/f1tenth_dev/commit/94a6c0f)/[ec52e52](https://github.com/mingguangZhou/f1tenth_dev/commit/ec52e52) guardrails; [7cd7968](https://github.com/mingguangZhou/f1tenth_dev/commit/7cd7968) explicit simulator config; [2bb59d5](https://github.com/mingguangZhou/f1tenth_dev/commit/2bb59d5) shared [agent-status serializer](../f1tenth_gym_ros/f1tenth_gym_ros/agent_status.py). Contracts verified; builds and scan/odom/PF/GT message validation reported locally. Exposing GT does not isolate control. |
| 2026-09-17 guidance; audit date uncertain | **Process and GT dependency audit.** [c388235](https://github.com/mingguangZhou/f1tenth_dev/commit/c388235), [b7653ab](https://github.com/mingguangZhou/f1tenth_dev/commit/b7653ab), [5d48bd5](https://github.com/mingguangZhou/f1tenth_dev/commit/5d48bd5) establish reporting/Mermaid/artifact policy. Lower-safety GT reverse gate is default-off but normal-sim override-on, corroborated by code/config. Runtime parameter/graph check reported; forced recovery not tested. |
| 2026-09-18 | **Closed-loop startup and bounded motion.** [8db4f22](https://github.com/mingguangZhou/f1tenth_dev/commit/8db4f22), PF [b11aebe](https://github.com/mingguangZhou/particle_filter/commit/b11aebe), and operating guidance [2814336](https://github.com/mingguangZhou/f1tenth_dev/commit/2814336)/[5389fc1](https://github.com/mingguangZhou/f1tenth_dev/commit/5389fc1). Reported zero-drive blocker: lower-safety odometry frame versus map-heading geometry; asynchronous TF/mixed clocks also failed contracts. Frame predicate corroborated in source. Separate eval profile, PF map-frame safety odometry, wall time, bounded TF wait, health gating and readiness runner implemented. Fifteen-second motion/clean stop reported, later bags corroborate combined workflow; individual fixes were not causally isolated. See [startup contract](LOCALIZATION_SIMULATION.md#2-closed-loop-localization-workflow). |
| 2026-09-18 | **Recorded baseline and offline analysis.** [297673b](https://github.com/mingguangZhou/f1tenth_dev/commit/297673b) extends the same runner with recording, SQLite/CDR analysis and tests. Preserved baseline: 30,449 messages / nine topics / about 15 s; smoke, recording and analysis record PASS. Reanalysis/regression success reported. Pipeline PASS is not accuracy or lap acceptance. |
| 2026-09-20 | **Shared no-GT scorecard and comparison.** [85702a6](https://github.com/mingguangZhou/f1tenth_dev/commit/85702a6) includes schema-2 analyzer/settings/comparison despite a documentation-oriented title. Synthetic onboard-style artifacts corroborate no-GT applicability handling; 54 tests and unchanged legacy metrics reported. Not real onboard performance or matched-section comparison. |
| 2026-09-23; experiment 09-22 | **Spatial smoothness and readable reports.** [c12c4ec](https://github.com/mingguangZhou/f1tenth_dev/commit/c12c4ec) replaces headline physical derivatives with four spatial local-trend oscillation signals. Preserved 09-22 run: 26,700 messages, PASS outcomes, four available rows; 56 tests/deterministic regeneration reported. Physical-time dynamics remain unavailable. See [definitions](LOCALIZATION_SIMULATION.md#53-scorecard-definitions). |
| 2026-09-23 | **Passive onboard readiness.** [6de72e7](https://github.com/mingguangZhou/f1tenth_dev/commit/6de72e7) adds preflight, operator interval/provenance/finalization. External [f1tenth_system 5a46e19](https://github.com/mingguangZhou/f1tenth_system/commit/5a46e19) corroborates mux `/ackermann_cmd` versus autonomy `/drive`. Sixty-seven tests, six CTest targets and mock graph/recording checks reported; mock telemetry used a substitute type because the sim image lacks `vesc_msgs`. Real-car contract and split speed/IMU adapter remain unverified/incomplete. |
| 2026-09-25 | **Track-explicit evaluation.** [98b24c8](https://github.com/mingguangZhou/f1tenth_dev/commit/98b24c8), PF [dfd07cb](https://github.com/mingguangZhou/particle_filter/commit/dfd07cb) separate integration from assets, resolve explicit/runtime maps and record hashes. Preserved IFAC run: 24,750 messages, PASS outcomes, GT readers limited to observer/recorder; 79 relevant tests/reanalysis reported. Asset preflight is not vehicle-envelope feasibility validation. |
| 2026-09-27 | **Headless operation and passive timing.** [1b9a5f4](https://github.com/mingguangZhou/f1tenth_dev/commit/1b9a5f4) removes unconditional runner RViz, preserving manual opt-in/default. Two preserved 10 s headless runs PASS at roughly 20 Hz drive; one RViz repeat FAILs stationary cleanup although recording/analysis PASS. One hundred targeted tests and broader Gym CLI/lint failures reported. No qualified real-time factor or isolated PF computation measurement; interrupted scan timing can be absent. |

### New-track investigation and record maintenance

| Date | Milestone, evidence and limitation |
| --- | --- |
| Experiment 2026-10-02; commit 10-07 | **Tempo Coffee unresolved.** [9d4abc0](https://github.com/mingguangZhou/f1tenth_dev/commit/9d4abc0) commits PGM/YAML only. Local handoff reports optimizer corridor/curvature rejection; archived metadata/results corroborate provisional centerline 10 s PASS (14.824 m, no collision, one emergency entry) but a requested 20 s run stops at 7.59 s with collision. Experimental config/generated path/docs were rolled back. No validated optimized raceline; link between optimizer rejection and driving failure is inferred, not isolated. Artifacts/reports are local-only. |
| Reconstruction 2026-10-09; commit 10-10 | **Persistent engineering history.** [edc009b](https://github.com/mingguangZhou/f1tenth_dev/commit/edc009b) records source/Git/report inspection, bag counts and IFAC manifest/hash corroboration. Documentation checks reported passing; no new ROS/hardware validation. The prior entry's “uncommitted” status is corrected by this commit evidence. |
| 2026-10-10; this uncommitted task | **Human-led consolidated record.** Explicit human request replaces the three-record layout with this record, removes AI-assigned priorities and distinguishes implementation from approval. Documentation/reference/scope checks are the applicable validation; no functional change or ROS validation. |
| 2026-10-10; uncommitted workbench task | **Offline raceline workflow.** [Workbench](../centerline_tools/raceline_workbench.py) reuses Charlie and E/A/X, with shared production gates, isolated/hash-qualified outputs and 16 deterministic retries. Fresh IFAC fixture AUTO_PASS (max 0.699 1/m); Tempo AUTO_SEARCH_EXHAUSTED (best corridor-safe candidate 1.460 > 1.082 1/m). Tests/targeted Foxy build passed; no runtime/hardware acceptance inferred. See [contract](../centerline_tools/README.md#2-workbench-contract) and [commands](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md#11-offline-raceline-workbench). |

## 3. Engineering decisions and implementation rationale

Dates below identify implementing commits, not inferred approval dates. Unless
explicitly stated otherwise, these are **Implemented technical choices**: source
supports the choice, but independent evidence of formal human approval or original
historical intent is not established. Existing IDs are retained. Rationale is
limited to inspected code/docs; no undocumented rejected alternative is invented.

| ID / date / classification | Approach, rationale, constraints and evidence |
| --- | --- |
| **DEC-001** · 2026-08-25 · Implemented technical choice | Common behavior YAMLs plus sparse platform adapters, replacing duplicated simulator profiles ([ebb8c79](https://github.com/mingguangZhou/f1tenth_dev/commit/ebb8c79)). Explicit parameter order preserves shared behavior while accommodating clocks/frames/paths and simulator gates. See [bringup](../oudtra_driver_bringup/README.md); eval variants must preserve normal/onboard defaults. |
| **DEC-002** · 2026-08-25 · Implemented technical choice | Sole published static raceline supplies Frenet/reference geometry, replacing separate runtime centerline selection ([449e3aa](https://github.com/mingguangZhou/f1tenth_dev/commit/449e3aa)). Direction applied once; nominal/rejoin target `d=0`. Static deviation includes intentional avoidance, not just tracking error. See [planner](../path_following_v2/README.md). |
| **DEC-003** · implemented 2026-09-16 · Human-confirmed current constraint | Host source and existing localization-ready Foxy Rocker environment, explicitly required by the human-supplied [AGENTS.md](../AGENTS.md) instructions for this task. [94a6c0f](https://github.com/mingguangZhou/f1tenth_dev/commit/94a6c0f)/[ec52e52](https://github.com/mingguangZhou/f1tenth_dev/commit/ec52e52)/[7cd7968](https://github.com/mingguangZhou/f1tenth_dev/commit/7cd7968) document implementation, not historical approval. Reuse mounted infrastructure; separate perfect/PF workflows, targeted builds and source freshness remain constraints. |
| **DEC-004** · 2026-09-16/18 · Implemented technical choice | Passive GT evaluation interface plus explicit control isolation ([2bb59d5](https://github.com/mingguangZhou/f1tenth_dev/commit/2bb59d5), [8db4f22](https://github.com/mingguangZhou/f1tenth_dev/commit/8db4f22)). Normal sim reverse swept GT gate remains enabled; [eval overlay](../oudtra_driver_bringup/config/localization_eval_sim.yaml) disables/redirects it, runner checks parameters/subscribers. Preserves ordinary safety behavior but removes that additional simulator-only check from evaluated recovery. GT is not globally passive in every profile. |
| **DEC-005** · 2026-09-18 · Implemented technical choice | Observable readiness, one-shot map `/initialpose`, subsequent health/pose/TF and bounded motion/stop ([8db4f22](https://github.com/mingguangZhou/f1tenth_dev/commit/8db4f22), PF [b11aebe](https://github.com/mingguangZhou/particle_filter/commit/b11aebe)). Map-frame safety odometry, wall time and bounded asynchronous TF wait address reported startup contracts. No repeated truth input, latest-TF shortcut or TF redesign; no reset acknowledgement or accuracy guarantee. Fixes tested together. See [runner](../oudtra_driver_bringup/scripts/run_localization_smoke.py). |
| **DEC-006** · 2026-09-18/20 · Implemented technical choice | Shared offline semantic contract, optional GT and four applicability states ([297673b](https://github.com/mingguangZhou/f1tenth_dev/commit/297673b), [85702a6](https://github.com/mingguangZhou/f1tenth_dev/commit/85702a6)). Odometry consistency is correlated evidence, not truth; unavailable is not zero. SQLite/CDR scope is explicit. Comparison requires compatible identity/definitions/units/evidence/coverage/timing; no composite score. See [shared contract](LOCALIZATION_SIMULATION.md#5-shared-simulationonboard-evaluation). |
| **DEC-007** · 2026-09-23 · Implemented technical choice | Headline smoothness uses local-trend residuals in raw-odometry driven-distance coordinates ([c12c4ec](https://github.com/mingguangZhou/f1tenth_dev/commit/c12c4ec)), superseding headline steering-rate/physical derivatives. Irregular sampling supported, intended maneuvers separated from local wiggle; stationary chatter omitted. Unqualified physical derivatives remain secondary/unavailable. See [analysis settings](../oudtra_driver_bringup/config/evaluation_analysis.yaml). |
| **DEC-008** · 2026-09-23 · Implemented technical choice | Passive Terminal 4 recorder; mux `/ackermann_cmd` is actuator-path command role, `/drive` debug evidence ([6de72e7](https://github.com/mingguangZhou/f1tenth_dev/commit/6de72e7)). No stack launch, commands, pose reset or parameter changes. Preserve VESC speed/measured IMU separately; servo-command echo is not feedback. Split adapter and live preflight remain open. See [recorder](../oudtra_driver_bringup/scripts/record_onboard_localization.py) and [contract](LOCALIZATION_SIMULATION.md#55-onboard-recording-contract). |
| **DEC-009** · 2026-09-25 · Implemented technical choice | Explicit scenario/map/raceline/direction/config, separate from eval integration ([98b24c8](https://github.com/mingguangZhou/f1tenth_dev/commit/98b24c8), PF [dfd07cb](https://github.com/mingguangZhou/particle_filter/commit/dfd07cb)). Explicit map authoritative; fallback resolved from runtime config. Preflight/hash provenance avoids hidden IFAC ownership; does not certify geometry safety. See [scenario contract](LOCALIZATION_SIMULATION.md#23-interfaces-and-evaluation-profile). |
| **DEC-010** · 2026-09-27 · Implemented technical choice | Headless benchmark, optional manual RViz, unchanged wall-time model ([1b9a5f4](https://github.com/mingguangZhou/f1tenth_dev/commit/1b9a5f4)). Gym bridge path has no rendering; no physics/control-rate tuning or `/clock`. Measured ages combine queues/processing/transport; some PnC marker computation persists. See [timing limits](LOCALIZATION_SIMULATION.md#25-headless-execution-and-timing-evidence). |
| **DEC-011** · reconstructed 2026-10-09, committed 10-10 · Historical / superseded | Three-document progress layout recorded by [edc009b](https://github.com/mingguangZhou/f1tenth_dev/commit/edc009b). Its useful historical/evidence content is retained here; “accepted” labels did not establish human approval. Superseded by DEC-012 through this task's explicit human request. |
| **DEC-012** · 2026-10-10 · Human-confirmed decision | This task explicitly requires one factual project record and human ownership of objectives/priorities/major choices. Codex plans authorized tasks and maintains evidence; earlier entries are context, not authorization. AI proposals remain temporary unless explicitly requested for tracking, labelled **AI suggestion — not approved**. Meaningful updates only; no automatic commit/push. See [workflow](ENGINEERING_WORKFLOW.md#persistent-engineering-records). |
| **DEC-013** · 2026-10-10 · Human-confirmed workbench requirement | This task explicitly requires Charlie as preferred backend, one production validator for automatic/manual candidates, hard envelope never auto-relaxed, bounded deterministic search and no invalid promotion. Implemented using existing backend geometry/smoothness checks, with diagnostics retained on failure. Search exhaustion is not physical infeasibility; GUI preview cannot override final gates. See [workbench contract](../centerline_tools/README.md#2-workbench-contract). |

## 4. Open issues and validation gaps

These are unordered observations. Resolution, development sequence and major
alternatives require human direction; listing an issue does not authorize work.

- **Tempo geometry/workflow unresolved:** map files exist, but no validated optimized
  raceline/config survives rollback. The new offline default baseline fails clearance
  and curvature; 16 retries find no valid line. Best corridor-safe candidate has
  0.381 m clearance versus 0.285 m required, but curvature 1.460 > 1.082 1/m.
  Search exhaustion and earlier short motion are not track acceptance or physical
  infeasibility. Manual fallback exists; no alternative vehicle envelope is selected.
- **Hardware evidence absent:** onboard topic/type/frame preflight and passive
  recording need real-car verification. Split VESC forward-speed / measured IMU
  yaw adapter is missing; model-derived odometry yaw must not be presented as
  measured yaw. Vehicle-response smoothness/stall support is consequently limited.
- **Recovery and shutdown coverage incomplete:** normal simulation can consume GT
  in reverse recovery; eval profile isolates it. One visual timing repeat failed
  stationary cleanup. Full recovery behavior/hardware coverage remain unknown.
- **Timing evidence limited:** no qualified physics clock/step counter, real-time
  factor or physical derivative timing. PF pose publication stamp and input-stamped
  TF differ; message age is not isolated PF compute time. Interrupted runs can lack
  scan observer summaries, and some unsubscribed marker work persists.
- **Workbench evidence limits:** GUI construction is tested, not race-day operator
  interaction. IFAC needs its explicit reviewed solver/coverage profile; generic
  strict solver can reach iteration limit. Phase 9 rejects rotated origins/negated
  maps; synchronous solves can pause the GUI. No changed profile auto-relaxes gates.
- **Test gaps:** historical broad Gym checks report missing `f1` CLI and lint
  failures. Targeted passing tests do not erase those failures; no suites were
  rerun for this assessment.
- **Evaluation scope:** no real matched sim/onboard run, section alignment, lap/sector
  contract, absolute-accuracy acceptance thresholds or composite score established.
  Short-run PASS does not prove collision freedom or long-run robustness.
- **Provenance gaps:** remote CI/PR discussions, live installed-source freshness and
  real-car logs not independently checked. Some old run revisions are dirty/unknown;
  artifacts cannot silently be attributed to assessed HEAD. Root/package legacy
  commands are not the canonical [operational reference](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md).

Historical local evidence includes `ifac_pf_closed_loop_localization_baseline_final`,
`ifac_pf_closed_loop_smoothness_baseline_20260922T175959Z`,
`ifac_track_explicit_pf_eval_20260925T120000Z`, `ifac_timing_headless_01/02`,
and `synthetic_onboard_no_gt_contract_fixture` under the documented ignored run
home. The last is a synthetic fixture, not car performance. See
[artifact discovery](LOCALIZATION_SIMULATION.md#4-preserved-run-artifacts-and-deliverables)
for access/regeneration. These datasets, local temporary reports and the Tempo
`/tmp` archive are not promised downloadable evidence; none were moved or added
to version control by this consolidation. Older source records remain recoverable
at [edc009b](https://github.com/mingguangZhou/f1tenth_dev/commit/edc009b).
