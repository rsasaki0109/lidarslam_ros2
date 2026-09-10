# Competitive claim publication contract (Phase 1)

Status: **NOT_READY**.  This document specifies a future publication boundary;
it does not add a current benchmark result or a README claim.  No production
campaign, dataset, ground-truth artifact, or scorer is opened by the publisher.

## Contract

`configs/slam_benchmark_profiles/competitive_claim_publication_v1.schema.json`
describes a deterministic publication request.  It binds a profile, the
scored-evidence document, a v2 suite receipt, an execution-selection receipt, a
fresh-holdout authorization document, and one canonical evidence-bundle
root/manifest.  The publisher reruns
`evaluate_evidence_v2(..., require_bundle=True)` over the sealed scored
evidence, then compares the supplied suite receipt with the complete
recomputed result and its profile/file/evidence SHA identity.  A status or
metric edited in the supplied receipt cannot override that result.  The scope
is derived from the profile and evaluated evidence, then the declared scope is
required to match its canonical SHA.  It contains the complete system/dataset
list, pinned revisions, at least three runs per system/dataset, metrics,
thresholds, and a fixed 95% confidence-interval method/seed.

The publisher in `scripts/publish_competitive_claim.py` reopens every declared
document and calls the existing v2 evaluator plus execution-selection,
fresh-holdout, rival source-closure, dataset source-closure, and evidence-bundle
verifiers.  It
rejects a status-only receipt, report-only or synthetic receipt, missing or
expired authorization, failed closure, changed bytes, invalid bundle
manifest, noncanonical revision identity, and any partial or duplicate
system/dataset/run binding.  It also requires the recomputed suite checks for
completion, RTF, RSS, map quality, aggregate APE improvement of at least 10%,
and positive fixed-seed 95% bootstrap superiority for every rival.

The sealed failure projection is the canonical concatenation of evaluator
errors/failed checks and every event in the authorization `failure_log`
(`complete: true` is mandatory).  The bundle must expose exactly one reopened
global `failure` artifact.  For a claim, that artifact is not an opaque file:
its bytes must be canonical UTF-8 JSON
`{"schema_version":1,"failure_log":<exact authorization failure_log>}`
with sorted keys, compact separators, and one trailing newline.  Duplicate
JSON keys, non-JSON bytes, extra fields, event/hash-only substitutes, and a
different event list are rejected.  The publisher reopens the file, checks
regular-file/size/SHA identity through the bundle verifier, and compares its
parsed content to the authorization mapping before rendering; the artifact
SHA and every failure event are included in the receipt and Markdown.  The
composer enforces the same producer contract whenever claim composition is
requested.  Candidate/NOT_READY bundles may remain opaque, but they cannot
cross the claim publication boundary.

The renderer emits only a deterministic `claim.md` fragment and a
`publication_receipt.json` into a new output directory.  It never writes
`README.md`, invokes a scorer, starts a process, or follows a GT path.  The
output is staged beside the requested root and atomically renamed; an existing
root is never overwritten.  The renderer reads aggregate APE (ours/best rival,
improvement), 95% CI bounds, completion, maximum RTF, RSS ratio/limit, and map
gate only from the recomputed result and emits them deterministically.  The
rendered sentence explicitly says the result is lower than the pinned best
rival only within the fixed scope and is not a universal/all-datasets claim.
receipt binds the rendered Markdown, scope, all input hashes, the complete
recomputed failure ledger, caveats, and reproduction command.  The receipt
identity deliberately excludes only its own identity and rendered Markdown
hash to avoid a circular hash; the rendered hash is checked separately.

## Current gate state

The authoritative competitive profile remains `NOT_READY` because its fresh
holdout authorization, dataset source closure, and rival source closure are
not all claim-ready.  A real request made from that profile must therefore
fail closed and produce no publication root.  Synthetic positive fixtures in
`graph_based_slam/test/test_publish_competitive_claim.py` monkeypatch the
verifier calls only to exercise deterministic formatting, full coverage,
tamper rejection, authorization rejection, and atomic/no-overwrite behavior;
they are not evidence and cannot satisfy the production gate.

## Phase 2 README publication guard

The dedicated marker in `README.md` is now guarded by
`scripts/verify_competitive_claim_readme.py` and the configuration/schema pair
`configs/slam_benchmark_profiles/competitive_claim_readme_publication_v1.{json,schema.json}`.
The current configuration is deliberately `NOT_READY`: the marker is empty,
the publisher is not invoked, and no claim text can be generated from the
current profile.  The guard still reopens the README, validates the immutable
legacy paragraph identities and hashes, and rejects new comprehensive,
universal, or all-dataset superiority wording outside the marker.  Existing
scoped comparison paragraphs are allowlisted by stable ID, exact heading, and
paragraph SHA, so unrelated paragraphs may be inserted above them without
changing their identity.  A changed paragraph, duplicate identity, or newly
added allowlist entry is rejected; revision 1 requires a versioned guard
migration for any intentional baseline change.

When a future reviewed configuration becomes `READY`, the guard must reopen
the publication spec and sealed root, invoke the Phase 1 publisher into a new
temporary root, and compare the generated Markdown and receipt identities with
the sealed receipt and marker bytes.  It rejects symlinks, traversal, duplicate
or nested markers, tampered configuration, manual marker edits, incomplete
receipts, publisher failure, and any mismatch.  The CI docs-and-release job
runs this read-only guard without network, dataset, GT, or scorer access.

Synthetic adversarial coverage is in
`graph_based_slam/test/test_verify_competitive_claim_readme.py`; it does not
create or upgrade a production claim.  The current gate remains PASS for the
guard and `claim_eligible: false` / `publication_status: NOT_READY`.
