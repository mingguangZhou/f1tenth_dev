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
   assumptions. Do not mark an unmet objective complete or commit/push without
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

## Documentation and diagrams

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

## Completion

The objective is met within its non-goals; the diff is focused and reviewed; relevant
validation supports the stated outcome; changed contracts or usage are documented;
and limitations plus final Git state are disclosed. An unresolved validation blocker
must be reported as such. A report or successful build alone does not prove runtime
correctness, and completion does not require a commit.
