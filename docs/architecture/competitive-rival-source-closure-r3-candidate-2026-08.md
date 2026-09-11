# Competitive rival source closure r3 candidate (2026-08)

This is an additive, non-promoting source-closure candidate.  The checked-in
r2 profile, r2 selection, historical selection, active profile, execution r3
candidate, and README are unchanged.  The candidate is intentionally
`NOT_READY`, `benchmark_eligible: false`, and `claim_eligible: false`.

## What r3 binds

`competitive_rival_source_closure_2026-08-r3-candidate.json` embeds an exact
copy of the profile's r2 `evidence_gate_v2.rival_source_closure` object.  The
copy includes every pinned upstream revision, archive/tree hash, source and
component declaration, license row, local patch, recipe, active-selection
pointer, and supersession field.  The validator recomputes the r2 closure
identity and rejects any projection that is not byte-for-byte equal to that
current profile section.

The separate `current_runner_bindings` and `current_recipe_bindings` records
reopen local files and trees.  They bind the current GLIM runner
(`1cb5a1e7354c168ae2317ff96e9450fb9e7e93b1314db8690bc63177e04913a1`) and
FAST-LIVO2 runner
(`eda2373a1f04098851ffc48ccb0b1d8ba282b8b84c2a3f8102e18e3b45284352`) while
retaining each r2 declared hash as `r2_sha256`.  This resolves only the
reproducibility identity drift inside the candidate; it does not repair r2 or
claim that a benchmark was executed.  Other local recipe files and pinned
upstream archive descriptors are bound separately.  An archive-relative
descriptor is recorded as `PINNED_UPSTREAM_ARCHIVE_UNOBSERVED`, never guessed
from a same-named checkout file.

The candidate also binds the legal-capture contract producer and schema:

| contract | path | SHA-256 | meaning |
|---|---|---|---|
| producer | `scripts/capture_competitive_rival_legal_provenance.py` | `273c9354ca2b745c912a226262a9f23412e3497737f4d26589d463a962ec6316` | offline capture implementation identity |
| schema | `configs/slam_benchmark_profiles/competitive_rival_legal_provenance_capture_v1.schema.json` | `e3f28f61cc524318de76492b021c87dba116993fbadc6cab1fe3f5adf815e008` | capture contract identity |

These are contract hashes, not external legal review.  A capture packet is
never auto-promoted by this candidate.

## Legal status and handoff

The candidate retains these fail-closed blockers:

- GLIM ROS2's pinned tree has package metadata but no license text artifact.
- FAST-LIVO2 has an unresolved BSD declaration versus GPL-2.0 license conflict.
- rpg_vikit has mixed/incomplete component license artifacts.
- Sophus has no license artifact at its pinned legacy revision.
- An independent custodian review is required.

The companion selection candidate and
`prepare_competitive_rival_source_closure_r3_handoff.py` produce an unsigned,
immutable review envelope.  It binds candidate/selection bytes and the r2
lineage, but its legal packet is explicitly `NOT_PROVIDED`; an offline capture
does not become `REVIEWED_EXTERNAL`.  The pair is fresh, no-follow,
single-link, fsynced, mode `0444`, and cannot overwrite an existing output.

Promotion remains a separately reviewed operation requiring a READY external
artifact and a profile canonical reseal.  No r3 selection is active and no
SOTA or superiority claim is made.

## Evidence

The checked-in candidate identities are
`26280f37d7fb0f18574b90042f932409013ff0d9c3f5e8d40d8daaffe1089dfa` for the
closure candidate and
`d946cf3d862618b48d45a5f7143b17e31138de5a5da227cea412bd1e1b7670b8` for the
selection candidate.  The current file/sidecar SHA-256 values are:

| artifact | SHA-256 |
|---|---|
| closure candidate JSON | `32b1ea2601d71d582cfd2b4476abd00a12a41fc77c06614f25c6e9f8819fa6d6` |
| closure candidate sidecar | `701cb390ab73b061147d7c9166186a20cd4b31b3a4181b337603f07cdf984e98` |
| selection candidate JSON | `28829bf6470545dcaae30825591f02e69bbe5c9bf962eb7545f24d203552b661` |
| selection candidate sidecar | `746244403e9428531d2b95e7428ed577b47b4362de463dc95848af4f393597d6` |
| candidate validator | `7acf1effbde478a1d2da46d5e9fd8c7b538ca9d5c3346ccc77f390cc457c5b7e` |
| handoff utility | `4528e0cc7e41395abf9c3abc4fda59e9bd4ca85ab4ffe1e53e8d69a122465032` |
| candidate schema | `623d277c6f582c1e1d0d15637b6c6c48c9a208155b4be9d425c12af12bc2df8a` |
| selection schema | `59ccb120914d7ba09c2e188038331f2e2b77cc05bdbf4be807d82e2b62b58d68` |
| handoff schema | `8f757ab8ba0f3eea838785931b95dcd07c31734e4abb09defcb79eb802acb204` |

The latest offline packet used for validator testing was stored outside the
repository at `/tmp/competitive-rival-legal-parent-archive2.RYTIES/packet`.
Its capture bytes SHA-256 was
`892e4cd6f8a61faccfd79b7647e82a8f09b3654f5e8709c3880341ff066222a7` and its
sidecar SHA-256 was
`24a3d6aff9739e756439b324b5045102c5d3c12ff594570154546293905c1085`.
The packet remained `REVIEW_REQUIRED` / `UNSIGNED_REVIEW_REQUIRED` and was
not included in the checked-in candidate.

No network, Docker, image pull, dataset, bag, GT, scorer, or formal benchmark
was run for this candidate.  The current r2 checker still reports runner
drift and legal `NOT_READY`; those results are preserved rather than
rewritten.
