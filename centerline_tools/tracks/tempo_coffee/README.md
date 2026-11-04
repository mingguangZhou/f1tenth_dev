# Tempo Coffee deployment selection

This directory contains the engineer-selected **experimental** `008_geometry`
raceline shared by simulation and onboard integration. It is a reviewed deployment
asset, not an automatically updated workbench output.

- CSV SHA-256: `0aab3968b64537b608ff8821ff8b1bd28ec3253d3c12719d542ad3ae6a71e88e`.
- Direction: `csv`; original geometry and validation report are copied byte-for-byte.
- Peak curvature: 1.460243 m⁻¹ versus the unchanged 1.082 m⁻¹ production limit.
- `selected_raceline.yaml` uses relative asset paths; the existing root Tempo map
  YAML/PGM remains the single map source. Package installation preserves this layout.
- `experimental_validation.yaml` is original generation-time evidence. Its absolute
  source-candidate paths and simulation-only wording are historical provenance,
  not runtime dependencies. The engineer subsequently authorized controlled onboard
  integration; that authorization does not establish production or onboard validation.

Changing the deployed selection is an explicit Git change: replace the chosen CSV
and matching evidence, update relative manifest paths/hashes, validate and review.
Never promote a failed path under the validated filename or automatically select
new workbench output. Session candidates, bags, logs and temporary reports stay
ignored. Ordinary code deployment carries these curated assets after a commit/pull;
no extra file transfer is needed.

Use the [operator command reference](../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md#current-operator-workflow)
for simulator and onboard commands.
