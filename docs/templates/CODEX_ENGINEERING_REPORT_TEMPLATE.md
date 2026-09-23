# Engineering report: <task>

<!-- Task reports are temporary/uncommitted unless explicitly requested otherwise.
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

## 2. Objective and non-goals

<Problem, intended observable outcome, and acceptance criteria.>

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
interpret its outputs, and material tradeoffs.>

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
State whether any acceptance criterion remains unmet.>

## 12. Validation artifact triage

<Apply [the promotion rule](../ENGINEERING_WORKFLOW.md#5-validation-artifact-triage):
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
