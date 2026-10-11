# Engineering report: <task>

<!-- Save as temporary_reports/NNN_<meaningful_subject>.md using the next unused
     ordinal and updating local INDEX.md. Chronology is not phase or priority.
     Task reports are Git-ignored/uncommitted unless explicitly authorized.
     Rebase source-template relative links for the report location (../docs/...).
     Keep the clickable Table of contents after the summary; update it after
     omitting/renaming sections and verify all anchors before handoff.
     Scale detail to the task; remove these instructions and optional sections
     that add no value. Do not paste large logs or unrelated diffs.

     Write in three layers: (1) quick purpose/outcome/findings/limits, (2) system,
     evidence flow, mechanism and necessary terms, then (3) exact evidence,
     validation, provenance and reproduction links. A normal experiment/validation
     report is usually about 1,000–2,000 words excluding tables/code/captions, but
     correctness and interpretability take priority. -->

## 1. Summary

<Concise overview: objective, outcome, key decisions, and material limits. For fixes,
state the symptom, evidenced root cause, correction, and verified completion status.>

<!-- table-of-contents:start -->
**Table of contents**

- [1. Summary](#1-summary)
- [2. Objective and non-goals](#2-objective-and-non-goals)
- [3. System / experiment and evidence flow](#3-system--experiment-and-evidence-flow)
- [4. Existing behavior / observations](#4-existing-behavior--observations)
- [5. Engineering decision / mechanism](#5-engineering-decision--mechanism)
- [6. Debugging and fix closure (when applicable)](#6-debugging-and-fix-closure-when-applicable)
- [7. Files changed](#7-files-changed)
- [8. Expected behavior](#8-expected-behavior)
- [9. Diagram / data flow (optional; omit if covered above)](#9-diagram--data-flow-optional-omit-if-covered-above)
- [10. Detailed evidence and validation](#10-detailed-evidence-and-validation)
- [11. Limitations / follow-up](#11-limitations--follow-up)
- [12. Validation artifact triage](#12-validation-artifact-triage)
- [13. Git state](#13-git-state)
<!-- table-of-contents:end -->

## 2. Objective and non-goals

<Human-defined problem, intended observable outcome, and acceptance criteria.
If future project direction is unknown, say "Not specified / awaiting human direction".>

<Behavior and subsystems intentionally outside this task.>

## 3. System / experiment and evidence flow

<What was exercised; relevant data/control flow; what evidence was collected; and
how it produced the result. Define only terms needed by the reader. Link maintained
commands, formal definitions, source, logs, and machine artifacts instead of copying
them. Add a compact diagram only when it clarifies relationships.>

## 4. Existing behavior / observations

<Relevant implementation/contracts inspected and the gap being addressed.>

## 5. Engineering decision / mechanism

<Minimal chosen change, existing mechanisms reused, high-level operation, how to
interpret its outputs, and material tradeoffs. Distinguish implemented technical
choices from explicit human-confirmed decisions; a commit/merge is not approval.
Major choices outside the assigned task require human direction.>

## 6. Debugging and fix closure (when applicable)

<Explain symptom → evidence and isolation steps → cause → correction → validation.
Distinguish direct causes, contributing defects, and unrelated improvements.
Disclose hypotheses rejected and whether fixes were validated together or separately.>

| Status | Investigation / fix | Evidence and result / remaining work |
| --- | --- | --- |
| <Done / Attempted or rejected / Unresolved or deferred> | <action and reason> | <observable evidence; limits> |

<Mark completed work explicitly and reconcile earlier plans with the final outcome.
Do not mark a fix done merely because code was edited.>

## 7. Files changed

| File | Reason |
| --- | --- |
| <path> | <specific purpose> |

## 8. Expected behavior

<Resulting contract, defaults, inputs/outputs, and operational commands when relevant.
Link to permanent documentation rather than duplicating it.>

## 9. Diagram / data flow (optional; omit if covered above)

<Link to an existing diagram or include Mermaid when it clarifies relationships,
ownership, or boundaries. Label current versus proposed behavior.>

## 10. Detailed evidence and validation

Environment: <branch, starting HEAD, relevant container/image/platform and scenario>.

| Exact command or procedure | Result | Evidence / limits |
| --- | --- | --- |
| <build/test/runtime check> | <pass/fail/not run> | <observations; reason if omitted> |

<Include exact results, relevant failures/warnings, measurements, sampling conditions,
plots or artifact links, and cleanup. Explain larger/smaller values where applicable.
Distinguish verified evidence from inference.>

## 11. Limitations / follow-up

<Unresolved issues, unverified behavior, and observations outside scope.
State whether any acceptance criterion remains unmet. Optional AI suggestions
are not approved priorities or tasks; do not invent a next strategic plan. Track
suggestions permanently only if explicitly requested, labelled "AI suggestion —
not approved".>

### Operator handoff (when runtime/CLI/validation/setup changes)

| Area | Handoff |
| --- | --- |
| Implemented | <changed behavior, tests, unverified portions> |
| Operator readiness | <canonical command-guide section, prerequisites, executable/conditional/blocked status, outputs and stop/cleanup> |
| Human direction | <already stated next manual activity; essential unresolved map/raceline/profile/approval question, if any> |

<Ask about the next manual activity only if unknown; do not invent a goal. Unanswered
choices remain conditional. Noninteractive handoffs state questions without claiming
human confirmation occurred.>

## 12. Validation artifact triage

<Apply the promotion rule in `docs/ENGINEERING_WORKFLOW.md`, section 5:
"Will we care about this result again after today?" List temporary artifacts,
including /tmp probes/logs and this report, or state that none were created.
Recommend promotions before implementing them.>

| Artifact / path | Purpose and classification | Recommended disposition / home | Rationale |
| --- | --- | --- | --- |
| <path> | <disposable diagnostic / reusable diagnostic / regression or acceptance test> | <keep temporary / proposed tool, test, or documentation> | <future value or contract> |

## 13. Git state

<Identify pre-existing changes separately from this task. Include final output below;
ordinary diff statistics omit untracked files, so list new files under Files changed.>

```text
$ git diff --stat
<output>

$ git status --short
<output>
```

<State commit/push status and any temporary artifacts excluded from staging.>
