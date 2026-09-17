# Engineering report: <task>

<!-- Task reports are temporary/uncommitted unless explicitly requested otherwise.
     Scale detail to the task; remove these instructions and optional sections
     that add no value. Do not paste large logs or unrelated diffs. -->

## Objective

<Problem, intended observable outcome, and acceptance criteria.>

## Non-goals

<Behavior and subsystems intentionally outside this task.>

## Existing behavior

<Relevant implementation/contracts inspected and the gap being addressed.>

## Engineering decision

<Minimal chosen change, existing mechanisms reused, and material tradeoffs.>

## Files changed

| File | Reason |
| --- | --- |
| <path> | <specific purpose> |

## Expected behavior

<Resulting contract, defaults, inputs/outputs, and operational commands when relevant.
Link to permanent documentation rather than duplicating it.>

## Diagram / data flow (optional)

<Link to an existing diagram or include Mermaid when it clarifies relationships,
ownership, or boundaries. Label current versus proposed behavior.>

## Validation

Environment: <branch, starting HEAD, relevant container/image/platform and scenario>.

| Exact command or procedure | Result | Evidence / limits |
| --- | --- | --- |
| <build/test/runtime check> | <pass/fail/not run> | <observations; reason if omitted> |

<Include relevant failures and warnings, actual message checks or measurements,
sampling conditions, and cleanup. Distinguish evidence from inference.>

## Limitations / follow-up

<Unresolved issues, unverified behavior, and observations outside scope.
State whether any acceptance criterion remains unmet.>

## Git state

<Identify pre-existing changes separately from this task. Include final output below;
ordinary diff statistics omit untracked files, so list new files under Files changed.>

```text
$ git diff --stat
<output>

$ git status --short
<output>
```

<State commit/push status and any temporary artifacts excluded from staging.>
