# Engineering workflow

Use this workflow to deliver a focused, reviewable change with evidence for its
intended behavior. Scale the detail to the task: a small fix needs a short plan
and relevant checks, not a separate design document or approval ceremony.
[AGENTS.md](../AGENTS.md) defines repository rules and approval boundaries;
[Development environment](DEVELOPMENT_ENVIRONMENT.md) defines build and runtime procedures.

## Standard workflow

1. **Inspect.** Read applicable instructions, check branch/HEAD and working-tree
   state, and trace the relevant implementation, configuration, consumers, and
   existing tests. Preserve unrelated work.
2. **Define objective and non-goals.** State the concrete problem, expected
   observable behavior, acceptance criteria, and boundaries of this task.
3. **Plan the minimal change.** Identify affected packages and contracts, reuse
   existing mechanisms, and choose the smallest sufficient design and validation.
   Explain meaningful tradeoffs; avoid abstractions for hypothetical future needs.
4. **Implement.** Make focused host-source edits. Preserve compatibility and
   behavior outside scope. Handle realistic failures explicitly without hidden
   fallbacks, unnecessary configuration, or unrelated refactoring.
5. **Run targeted build/tests.** Start with the affected package and smallest
   relevant checks in the canonical environment. Fix in-scope failures; record
   unrelated failures without expanding the task silently.
6. **Validate runtime behavior when appropriate.** For changes to ROS interfaces,
   launch/config resolution, timing, or integration, exercise the relevant workflow
   and inspect actual messages or other measurable outputs. Stop only processes
   started for the validation and record the scenario and commands.
7. **Inspect the diff.** Check tracked changes and new files for scope, accidental
   artifacts, and readability; run `git diff --check`. Revalidate if subsequent
   edits affect behavior already tested.
8. **Document expected behavior.** Update the permanent home for changed usage or
   contracts, including defaults and important limits. Avoid duplicating content.
9. **Report limitations.** Summarize outcome, files changed, exact validation and
   results, remaining issues, and Git state. Separate verified behavior from
   assumptions. Triage temporary validation artifacts using the rule below.
   Do not mark an unmet objective complete or commit/push without
   explicit authorization.

## Readability and explicit data flow

Use names that express intent and include units or frames where ambiguity matters.
Keep responsibilities focused and make inputs, outputs, state ownership, and side
effects easy to trace. Comments should explain constraints or reasons that code
alone cannot show. Prefer familiar local patterns and straightforward control flow;
extract shared code when there is a concrete reuse need, not just resemblance.

## Tests and validation

Test observable contracts and meaningful failure cases, not copies of implementation
logic. Reuse existing fixtures and checks before adding infrastructure. A regression
test should catch the behavior being fixed; a diagnostic interface check should
inspect actual fields and values, not merely confirm that a topic exists.

Use reproducible commands, scenarios, and measurable evidence where practical.
Distinguish startup/message-flow checks from correctness, accuracy, and performance
claims. GUI appearance alone is insufficient for those claims. Report failed checks,
warnings, sampling limits, and checks not run with their reasons. Broaden validation
when the change or evidence warrants it; avoid repeatedly running unrelated suites.
Documentation-only changes normally need diff, link, and content review, not ROS
builds or new tests.

## Validation artifact triage

At task completion, ask: **"Will we care about this result again after today?"**
Apply this to tests, probes, analysis scripts, logs, and engineering reports;
classify by purpose rather than filename or where the artifact currently lives.

| Class | Practical action |
| --- | --- |
| Disposable diagnostic | A one-time investigation may stay temporary, for example under `/tmp`. When useful, preserve its objective, exact command/method, result, and limitations in the engineering report. |
| Reusable engineering diagnostic | If the same check is likely to be useful again, recommend promotion to a version-controlled script/tool. Give it a stable interface, documented inputs/outputs and invocation, and an appropriate home such as `scripts/`, `evaluation/`, or the owning package. Reuse an existing tool when possible. |
| Regression/acceptance test | If the behavior is a contract that must remain true, recommend an automated test in the owning test suite. It must assert the contract and fail when that behavior regresses, rather than merely print observations. |

A useful result does not require retaining every artifact that produced it.
Logs and reports usually supply evidence to summarize; promote the reusable check,
test, or lasting conclusion instead. Retain raw data or a report permanently only
when there is an explicit need and an agreed home. Keep permanent documentation
organized around outcomes, contracts, and operating procedures rather than an
ever-growing chronology of `PHASE*_CODEX_REPORT` files.

Recommend promotions and their scope before making them; follow existing task
authorization and do not silently expand the work. Reports remain uncommitted by
default. Triage does not authorize deletion, staging, committing, or pushing.

### Reusable end-of-task Codex prompt

```text
Before closing this task, triage every temporary test, probe, analysis script,
log, engineering report, and other validation artifact created or used for it,
including artifacts under /tmp. Ask: "Will we care about this result again after
today?" Classify each as a disposable diagnostic, reusable engineering diagnostic,
or regression/acceptance test using docs/ENGINEERING_WORKFLOW.md.

List each artifact's path, purpose, classification, and recommended disposition.
For proposed promotions, explain future value, the existing tool/test/doc to reuse
or proposed home, and the stable interface or regression assertion needed. For
logs and reports, distinguish evidence worth preserving from the reusable method
or lasting conclusion. Record useful objectives, commands/methods, results, and
limitations without copying large logs into permanent documentation.

Present recommendations before making promotions. This triage request is analysis
only: do not promote, delete, stage, commit, or push artifacts. Identify any next
action needing separate authorization. Keep reports temporary/uncommitted unless
explicitly requested otherwise; prefer outcome-oriented permanent documentation.
```

## Documentation and diagrams

Start engineering documents and reports with a concise summary of the objective,
outcome, key decisions, and material limits so readers can understand the whole
change at a high level before reading the details.
For debugging or fixes, include the symptom, evidenced root cause(s), the steps
used to isolate them, what changed, and why validation supports resolution. Separate
direct causes from contributing defects and unrelated improvements; state when
changes were only tested together. Mark actions **Done**, **Attempted / rejected**,
or **Unresolved / deferred**, with evidence for completed fixes. Update earlier
proposed plans to reflect final status; do not leave implemented work ambiguous.

Permanent documentation describes supported behavior, interfaces, operating commands,
and decisions needed for maintenance. Keep it concise, operational, and close to its
subject. Link to the source of truth rather than copying instructions between files.

Engineering reports capture task-specific evidence, temporary observations, and
handoff context. They are **temporary and uncommitted by default**, unless explicitly
requested otherwise. Use the [report template](templates/CODEX_ENGINEERING_REPORT_TEMPLATE.md)
when a report is useful or requested; omit optional sections that add no value.
Promote stable lessons into the appropriate permanent document rather than preserving
every report, transcript, or generated log. Keep temporary artifacts out of commits.

Include an architecture or data-flow diagram when relationships, ownership, or
boundaries are clearer visually—for example ROS producers/consumers, TF ownership,
or separation of validation and control paths. Prefer Mermaid in committed Markdown
for useful version-controlled diagrams. Label relevant interfaces and arrow direction,
reflect the implementation, and distinguish proposed designs from current behavior.
Update an existing diagram when its contract changes; skip diagrams for simple edits
already clear in prose. A diagram is an explanation, not validation evidence.

For Markdown preview compatibility, use basic Mermaid flowchart syntax, simple
alphanumeric node IDs, and double-quoted node and edge labels. Keep ROS topic names
inside quoted labels: write `DRIVE["/drive"]`, not `DRIVE[/drive]`, which starts
Mermaid shape syntax. Avoid renderer-specific features and HTML labels unless
needed and verified. Check parsing/rendering with an available Mermaid renderer
before handoff; state explicitly when the target preview was not verified.

## Completion

The objective is met within its non-goals; the diff is focused and reviewed; relevant
validation supports the stated outcome; changed contracts or usage are documented;
and limitations plus final Git state are disclosed. An unresolved validation blocker
must be reported as such. A report or successful build alone does not prove runtime
correctness, and completion does not require a commit.
