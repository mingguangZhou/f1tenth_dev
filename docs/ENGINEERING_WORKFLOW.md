# Engineering workflow

## 1. How to use this document

Use this workflow to deliver a focused, reviewable change with evidence for its
intended behavior. Scale the detail to the task: a small fix needs a short plan
and relevant checks, not a separate design document or approval ceremony.
[AGENTS.md](../AGENTS.md) defines repository rules and approval boundaries;
[Development environment](DEVELOPMENT_ENVIRONMENT.md) defines build/runtime facts;
the [operational command reference](ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md)
owns maintained commands.

**Start here:** follow the [task workflow](#2-standard-workflow), then use
[operator handoff](#7-operator-handoff) at completion when usage changes.
**Quick links:** [readability](#3-readability-and-explicit-data-flow) · [validation](#4-tests-and-validation) ·
[artifact triage](#5-validation-artifact-triage) · [reporting/documents](#6-documentation-and-diagrams) ·
[project record](#persistent-engineering-records) · [completion](#8-completion).

<!-- table-of-contents:start -->
**Table of contents**

- [1. How to use this document](#1-how-to-use-this-document)
- [2. Standard workflow](#2-standard-workflow)
- [3. Readability and explicit data flow](#3-readability-and-explicit-data-flow)
- [4. Tests and validation](#4-tests-and-validation)
- [5. Validation artifact triage](#5-validation-artifact-triage)
  - [Reusable end-of-task Codex prompt](#reusable-end-of-task-codex-prompt)
- [6. Documentation and diagrams](#6-documentation-and-diagrams)
  - [Technical report standard](#technical-report-standard)
  - [Persistent engineering records](#persistent-engineering-records)
- [7. Operator handoff](#7-operator-handoff)
- [8. Completion](#8-completion)
<!-- table-of-contents:end -->

## 2. Standard workflow

1. **Inspect.** Read applicable instructions, check branch/HEAD and working-tree
   state, and trace the relevant implementation, configuration, consumers, and
   existing tests. Preserve unrelated work.
2. **Define objective and non-goals.** Confirm the human-defined problem, expected
   observable behavior, acceptance criteria, and boundaries of this task.
3. **Plan the minimal change.** Identify affected packages and contracts, reuse
   existing mechanisms, and choose the smallest sufficient design and validation.
   Explain meaningful tradeoffs; avoid abstractions for hypothetical future needs.
   Resolve minor implementation details within authorization. If a major choice
   is not covered by the task, present alternatives and request human direction.
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

## 3. Readability and explicit data flow

Use names that express intent and include units or frames where ambiguity matters.
Keep responsibilities focused and make inputs, outputs, state ownership, and side
effects easy to trace. Comments should explain constraints or reasons that code
alone cannot show. Prefer familiar local patterns and straightforward control flow;
extract shared code when there is a concrete reuse need, not just resemblance.

## 4. Tests and validation

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

## 5. Validation artifact triage

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
organized around outcomes, contracts, and operating procedures rather than a
collection of detailed local task reports.

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

## 6. Documentation and diagrams

Start engineering documents and reports with a concise summary of the objective,
outcome, key decisions, and material limits so readers can understand the whole
change at a high level before reading the details.
Use numbered top-level sections for substantial permanent human documentation and
generated engineering reports. Near the start, include a short document map or
experiment explanation that tells readers what the document answers and where the
source-of-truth commands, environment, contracts, and artifacts live.
For substantial human-facing guides and technical reports, include an explicit
clickable **Table of contents** near the front, after the short purpose/summary and
before the main detail. List every major section in document order; add a compact
second level for useful subsections in long guides. A few quick links or a narrative
document map do not replace the whole-document contents list. Keep existing heading
names/anchors stable where practical. Update contents in the same change as heading
additions, removals or renames, including generated-report templates/renderers when
in scope. Use ordinary Markdown anchor links compatible with GitHub and VS Code;
ignore code/comment headings and account for duplicate heading suffixes. Check
contents coverage and links before handoff. Short indexes or simple notes may omit
the list when it would add no navigational value.

For debugging or fixes, include the symptom, evidenced root cause(s), the steps
used to isolate them, what changed, and why validation supports resolution. Separate
direct causes from contributing defects and unrelated improvements; state when
changes were only tested together. Mark actions **Done**, **Attempted / rejected**,
or **Unresolved / deferred**, with evidence for completed fixes. Update earlier
proposed plans to reflect final status; do not leave implemented work ambiguous.

### Technical report standard

Write experiment, validation, debugging, performance, integration, and subsystem
reports in three reading layers. Combine or omit sections when that reads better.

1. **Quick engineering understanding:** purpose, outcome, main findings, and the
   limitations that materially qualify them.
2. **Engineering mechanics:** the system or workflow exercised, relevant data/control
   flow, how evidence became results, and only the terminology needed to interpret
   those results.
3. **Detailed evidence:** metric tables, exact values, plots, validation evidence,
   provenance, and links for reproduction or deeper inspection.

Where applicable, a technically literate reader should be able to answer: What was
the purpose? What system or experiment ran? How did control and evidence flow? What
was collected, and how were the important results derived? What do the main terms,
numbers, and larger/smaller values mean? Which limitations affect interpretation?
What was verified versus inferred? Where can the reader reproduce or inspect it?
These are reader questions, not mandatory one-question-per-section headings.

Keep reports self-explanatory without turning them into textbooks. Start with a
short whole-result summary; define a term once and use it consistently; explain one
clear mechanism instead of repeating it; use compact tables for repeated facts; and
use diagrams only when relationships are clearer visually. Link to maintained
commands, formal definitions, source, machine schemas, and raw logs rather than
copying them. Omit sections that add no useful information. As a practical default,
a normal generated experiment or validation report should usually remain around
1,000–2,000 words, excluding tables, code blocks, and captions. This is guidance,
not a failure threshold: use less for small work and exceed it when correct
interpretation genuinely requires more detail. Never achieve brevity by removing
evidence, terminology, or limitations needed to understand the result.

Permanent documentation describes supported behavior, interfaces, operating commands,
and decisions needed for maintenance. Keep it concise, operational, and close to its
subject. Link to the source of truth rather than copying instructions between files.

Engineering reports capture task-specific evidence, temporary observations, and
handoff context. They are **temporary and uncommitted by default**, unless explicitly
requested otherwise. Use the [report template](templates/CODEX_ENGINEERING_REPORT_TEMPLATE.md)
when a report is useful or requested; omit optional sections that add no value.
Store reports in Git-ignored repository-root `temporary_reports/`, using the next
unused ordinal plus a meaningful topic (`NNN_<subject>.md`) and updating local
`INDEX.md`. Ordinals describe chronology, not engineering phase or priority.
Reserve a distinct filename; never overwrite an earlier report. The index records
original names/date evidence and uncertainty for reorganized historical reports.
Relative links in reports start from that directory, for example `../docs/...`.
Keep reports accessible locally, but do not promise them through permanent links
as if every clone contains them. Commit/push requires explicit authorization.
Promote stable lessons into the appropriate permanent document rather than preserving
every report, transcript, or generated log. Keep temporary artifacts out of commits.

### Persistent engineering records

[PROJECT_RECORD.md](PROJECT_RECORD.md) holds a dated factual status, concise
milestones, significant decisions/implementation rationale, and open evidence gaps.
It complements technical documentation and temporary detailed task reports; it
is not a roadmap or authorization to fix every listed issue.

The human engineer owns objectives, strategic priorities, development sequence,
scope, and major design approval. Codex plans the assigned task, investigates,
implements within authorization, validates, and records evidence. Suggestions do
not become official objectives, priorities or accepted decisions. Keep them in
conversation or temporary reports unless the human explicitly requests permanent
tracking; then label **AI suggestion — not approved**, separate from confirmed
direction. If direction is unknown, state **Not specified / awaiting human direction**.
A commit, merge or earlier AI report does not establish human approval. Report
safety risks immediately without treating them as authorization to broaden scope.

During inspection, read relevant record entries and technical sources, confirm the
human task, and check branch/HEAD/working tree. At completion, refresh materially
changed factual status, append one concise milestone for a meaningful result or
concluded investigation, and update significant decisions/constraints when warranted.
Do not invent next priorities or add entries for tiny edits/intermediate debugging.
Keep implemented, build/test, runtime, performance and hardware evidence distinct;
label reported, inferred and unknown outcomes. Decision entries distinguish explicit
human confirmation from implementation evidence and historical/unresolved choices.
Correct history explicitly with a dated correction and preserve its references;
link superseding choices rather than silently rewriting history. Link accessible
permanent evidence and identify local-only artifacts without promising availability.
Maintenance does not authorize moving/deleting reports or data, committing, or
pushing. Honor limited task scopes; report recommended tracking updates outside them.

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

## 7. Operator handoff

For changes to runtime workflows, CLI/launch/config contracts, validation procedures
or experiment setup, include a brief handoff alongside the engineering report:

| Area | Required information |
| --- | --- |
| Implemented | What changed, what was tested and what remains unverified |
| Operator readiness | Exact operational-guide entry, executable versus conditional/blocked steps, selected paths/profiles and runtime prerequisites |
| Human direction | The next manual activity already stated by the engineer, and essential unresolved choices/approvals |

Keep procedures in the operational guide, organized as **Goal → Preconditions →
Terminal commands → Expected output → Stop/cleanup → Troubleshooting → Evidence
to retain**. Identify host, simulation container and onboard terminals; do not make
the engineer reconstruct a sequence from historical reports. Update verified,
authorized instructions during the task; trivial documentation edits do not need
another approval conversation. Preserve conditional steps when prerequisites are
unresolved, and wait for human direction before presenting them as ready to run.

If the next manual activity is unknown, ask conversationally: “Which manual workflow
would you like to perform next, so I can prepare the exact terminal instructions?”
If it is already known, do not ask again: raise only essential missing parameters
or approval choices. In noninteractive work, report those questions/prerequisites
in the final handoff; do not claim a popup or confirmation occurred. An unanswered question
does not grant approval. This step supports human-owned activity, not an AI-generated roadmap.

## 8. Completion

The objective is met within its non-goals; the diff is focused and reviewed; relevant
validation supports the stated outcome; changed contracts or usage are documented;
and limitations plus final Git state are disclosed. An unresolved validation blocker
must be reported as such. A report or successful build alone does not prove runtime
correctness, and completion does not require a commit.
