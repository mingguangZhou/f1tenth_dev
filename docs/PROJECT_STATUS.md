# RoboRacer project status

## 1. Summary and assessment boundary

The workspace has a repeatable, short IFAC PF + PnC simulation/recording/analysis
path backed by inspected historical run artifacts. It also has a passive onboard
recorder and shared no-GT analysis contract. Physical validation of that recording/
evaluation workflow and a validated Tempo Coffee raceline remain open. Short-run success does not establish
collision freedom, long-run robustness, or sim/onboard equivalence.

| Assessment | Value |
| --- | --- |
| Date | 2026-10-09 |
| Repository / checkout | `mingguangZhou/f1tenth_dev` / `/home/mzhou/f1tenth_dev` |
| Branch | `mingguang_localization_robustness_ai_workspace` |
| Assessed HEAD | `9d4abc0bba62a7a2b2125f07a407719ae43f8cac` |
| Historical boundary | `post_ifac_ai_baseline_v0` → `fb945e2` (commit 2026-08-29; tag created 2026-09-13) |
| Pinned PF | `dfd07cb895e854a7a5ca872339a0f567e24fa5ba` |
| Initial working tree | No tracked changes; 12 pre-existing untracked engineering reports, preserved. |

**Evidence vocabulary:** Verified = directly inspected code, commit, test contract
or artifact content; Reported = a past execution claim not reproduced here;
Inferred = a conclusion drawn without direct confirmation; Unknown = insufficient
evidence. Historical runtime artifacts support prior outcomes, not a fresh test of
this HEAD. No build, simulation, hardware test, or performance benchmark was run
for this documentation-only assessment.

Use [milestones](DEVELOPMENT_LOG.md) for history, [decisions](DECISIONS.md) for
constraints, and the [documentation map](README.md) for commands and definitions.

## 2. Overall objective and subsystem maturity

The current technical objective is reproducible localization-enabled closed-loop
measurement, progressing from simulation to honest onboard evidence and reusable
track selection. This assessment adds persistent progress records; it does not
approve or implement the proposed next technical tasks.

| Subsystem | Demonstrated maturity and evidence boundary |
| --- | --- |
| Foxy / Docker development | **Historical runtime validation Reported:** canonical environment and mount scripts inspected; live container/build freshness not rechecked. See [environment](DEVELOPMENT_ENVIRONMENT.md). |
| Normal and PF Gym bridges | **Historical simulation/runtime validation Reported:** explicit configuration and shared true-state contract Verified in [launch](../f1tenth_gym_ros/launch/gym_bridge_slam_launch.py) and [helper](../f1tenth_gym_ros/f1tenth_gym_ros/agent_status.py). |
| Centerline / optimized raceline | **IFAC artifact validated:** CSV/mask/centerline hashes match stored passing validation. Five-lap performance is **Reported**. New-track generalization unresolved. See [IFAC assets](../centerline_tools/output_backup/ifac_roboracer/README.md). |
| Following / planning / safety / arbitration | **Historical bounded IFAC simulation/runtime validation corroborated by bags/results.** Recovery/platform tests exist; full recovery coverage and hardware performance **Unknown**. See [bringup](../oudtra_driver_bringup/README.md). |
| PF initialization/integration | **Historical simulation/runtime validation corroborated:** one-shot pose, readiness and health gating. No initialization acknowledgement or accuracy guarantee; onboard hardware validation **Unknown**. |
| Recording / metrics / comparison | **Historical simulation and synthetic-contract evidence inspected:** schema-2 outputs and optional GT. Regression success **Reported**; no real sim/onboard comparison identified. |
| Passive onboard recorder | **Build/test and mock/runtime validated, Reported:** source passivity Verified; external mux/VESC source inspected at `5a46e19`. **Hardware validation pending.** |
| SLAM / range libraries | **Implemented, validation Unknown:** pinned dependencies and launch/config support; no current mapping experiment corroborated. |

## 3. Recent demonstrated milestones and active focus

- Closed-loop startup runner (`8db4f22`): frame/clock corrections without PF
  numerical tuning; individual contributions were not experimentally isolated.
- Recorded evaluation (`297673b`, `85702a6`, `c12c4ec`): shared sim/no-GT
  scorecard and spatial oscillation; preserved artifacts corroborate outputs.
- Explicit track contract (`98b24c8`, PF `dfd07cb`): selected map/reference assets
  and hashes flow through one runner; IFAC remains the maintained fixture.
- Headless default (`1b9a5f4`): two preserved 10-second headless runs report PASS;
  final drive is about 20 Hz. One visual repeat failed stationary cleanup.
- Tempo Coffee map committed (`9d4abc0`): **assets only**. The experimental config,
  centerline and documentation were rolled back. No validated optimized raceline
  or maintained Tempo launch exists at assessed HEAD.

Active unresolved technical focus is how to validate the new scanned track before
promoting a workflow. The parallel onboard measurement path is prepared in code
but still needs a first real-car dataset and the split speed/IMU analysis adapter.

## 4. Risks and proposed next tasks

1. **New-track geometry first:** re-establish a reviewed Tempo Coffee generation
   plan. Local archived evidence shows optimizer corridor/curvature failures, a
   short provisional centerline run passing, and a repeat colliding/stopping.
   Do not treat the map commit or short PASS as track acceptance.
2. **Onboard measurement:** verify the live required topic/type/frame contract
   and record one controlled dataset using the passive Terminal 4 workflow.
   Vehicle-response smoothness/stalls remain unavailable until the adapter combines
   VESC-derived forward speed with measured IMU yaw rate; odometry yaw is not a
   measured substitute. Hardware execution requires a separate task.
3. **Reliability diagnostics:** characterize intermittent stationary cleanup;
   timing analysis currently lacks scan summaries for interrupted runs. Broad Gym
   lint/CLI checks have Reported failures (`f1` absent and package-wide style issues).
4. **Evidence-qualified comparison:** implement matched-section/alignment work
   only after compatible real data/assets exist. No lap/sector contract, accuracy
   thresholds, or composite performance score is established by this pipeline.

Normal full-stack simulator reverse recovery can use GT; the explicit evaluation
overlay disables and disconnects that gate. Legacy wall-stamped publications do
not qualify physics-time derivatives or real-time factor. PnC still computes some
visualization markers without subscribers. Package/root READMEs retain legacy
commands and some hardware-oriented wording; the operational reference is the
current command source. These limits constrain interpretation, not proof of a new
functional defect.

## 5. Evidence access and gaps

Use [evaluation contracts and artifact discovery](LOCALIZATION_SIMULATION.md)
and [operational commands](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md).
Locally inspected run IDs include
`ifac_pf_closed_loop_smoothness_baseline_20260922T175959Z`,
`ifac_track_explicit_pf_eval_20260925T120000Z`, `ifac_timing_headless_01/02`,
and `synthetic_onboard_no_gt_contract_fixture` under the documented ignored
`oudtra_driver_bringup/runs/` home. Bags contain messages and results report
recording/analysis success; the synthetic fixture is not vehicle performance.
These are local-only evidence, not promised downloadable repository artifacts.

Temporary Phase reports and the Tempo handoff were read but remain uncommitted.
The Tempo archive was inspected locally under `/tmp`; it is temporary and has no
permanent link here. Older PR merge records establish code inclusion, not hardware
success. Remote CI/PR discussions, current installed binaries and real-car logs
were not independently checked. Some legacy run revisions are unknown or dirty;
do not infer equivalence to assessed HEAD. Reassess when meaningful new evidence
changes the state; do not apply its findings to a different revision silently.
