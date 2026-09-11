# Competitive execution-selection r3 candidate (2026-08)

This document describes an additive handoff candidate. It does not replace the
active profile or the historical selection receipt.

## Status

The checked-in candidate is `NOT_READY`, `benchmark_eligible: false`, and
`claim_eligible: false`. Its current structural validation is:

```text
PYTHONDONTWRITEBYTECODE=1 python3 scripts/validate_competitive_execution_selection_r3.py
```

The candidate binds the current checkout bytes for every runner (ours, GLIM,
and FAST-LIVO2), the GT-blind driver, six scorer/result producers, and the
three process-RSS producer/gate files. These are source identities only; they
are not execution receipts. It also binds the exact seven-field CPU thread
policy (`cpu_affinity`, `max_threads`, `OMP_NUM_THREADS`,
`OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, `TBB_NUM_THREADS`, and
`accelerator_policy`) with canonical identity
`262d656a7c382cd27696b3150215ae563a014796de69718754858bcb814ba993`.

The selection lineage is explicit:

* current recipe selection: `competitive_execution_selection_2026-08-r2.yaml`,
  selection id `competitive-execution-selection-2026-08-r2`, closure id
  `competitive-rival-source-closure-2026-08-r2`, closure identity
  `e876339f843b28c1be5d835e8f6a9a062c52715498c85d93afbdb83c6dbae4e8`;
* historical `competitive_execution_selection_2026-08.yaml` remains in place
  and is recorded as rejected/superseded under the r2 closure revision;
* the active profile still points to the historical path. No profile switch or
  hash update is performed by this candidate.

The candidate reports, rather than infers away, the remaining blockers:

1. rival-source legal provenance is `NOT_READY` and image publication is
   blocked;
2. dataset source closure remains `NOT_READY` with no reviewed pin manifest;
3. fresh-holdout authorization lacks the independent custodian attestation;
4. machine identity, image inspection, and container toolchain evidence are
   external execution artifacts and are unresolved here; and
5. the shared checkout has pending changes, so source bytes require external
   review before promotion.

The dataset and holdout section identities are recorded from the current
profile, but no dataset, bag, ground-truth, or scorer content is opened.
Machine fields are intentionally null; no machine id, container digest, or
license identity is invented.

## Unsigned handoff

An unsigned request can be prepared without a signer:

```text
PYTHONDONTWRITEBYTECODE=1 python3 scripts/prepare_competitive_execution_selection_r3_handoff.py \
  prepare --output /tmp/competitive-execution-selection-r3-handoff.json
PYTHONDONTWRITEBYTECODE=1 python3 scripts/prepare_competitive_execution_selection_r3_handoff.py \
  validate --handoff /tmp/competitive-execution-selection-r3-handoff.json
```

The request is an atomic JSON/sidecar pair. The two output files are created
exclusively, fsynced, and sealed read-only (`0444`), with single-link regular
file and device/inode checks on every reopen. Existing output/sidecar paths,
hard links, symlinked parents, path traversal, writable files, partial pairs,
and changes during validation are fail-closed; a failed second write removes
only the inode owned by that write. With no external manifest it lists all seven
required artifact classes as missing. A separately reviewed external manifest
may provide exact regular-file SHA-256 bindings for the rival closure, dataset
closure, holdout authorization, machine, image inspection, toolchain, and
execution receipt. The manifest and each referenced artifact are likewise
bounded, read-only (`0444`), single-link files. The handoff binds the manifest
root and file device/inode/mode/size as well as its bytes, and reopens those
identities before accepting coverage. The manifest is defined by
`competitive_execution_selection_r3_external_artifacts_v1.schema.json`.
Coverage is exact; duplicate, extra, symlinked,
hard-linked, stale, or cross-selection artifacts are rejected. The handoff
contains no private key, signer, signature, or promotion result.

The checked-in candidate sidecar is a source-control binding and may retain
normal repository permissions. The stricter `0444` requirement applies to
fresh handoff and external evidence artifacts, not to that checked-in source
sidecar.

Promotion is a separate custodian workflow requiring a reviewed READY r3
execution receipt and a canonical profile reseal. The workflow is atomic at
the policy boundary: historical receipts remain immutable, and this candidate
cannot switch the active profile. A signed or self-rehashed JSON file is not
treated as a receipt merely because its status says `READY`.

The additive [`r3 external capture`](competitive-execution-selection-r3-external-capture-2026-08.md)
utility can observe the three local image identities and, only with explicit
opt-in, probe their fixed toolchains. It emits immutable but
`NOT_REVIEWED_EXTERNAL` image/toolchain artifacts; a complete observation still
cannot become a reviewed receipt or alter this candidate.

No Docker, network, SSD mount, dataset/GT access, scorer, benchmark replay,
real signing, or profile promotion was performed for this candidate.

## Acceptance preflight

The checked-in `competitive_execution_selection_r3_acceptance_manifest.json`
is a deliberately non-promoting preflight manifest.  The exact checker is:

```text
PYTHONDONTWRITEBYTECODE=1 python3 scripts/check_competitive_execution_r3_acceptance.py
```

It reopens the current candidate/profile bytes, verifies their fixed SHA and
canonical identities, and requires the exact seven closure names.  All seven
slots are currently `PENDING` with no path or hash, so the checker returns
`NOT_READY` (exit 3).  Any self-rehashed `READY` edit, candidate/profile drift,
missing closure, or invented extra closure is `INVALID` (exit 2); this slice
does not accept an in-place promotion.  Signed legal/machine/image/toolchain,
dataset, holdout, and source-review artifacts are not opened or fabricated;
the future aggregator must receive them through a separately reviewed input
contract.
## Source-worktree observation

The additive `source_worktree_review` closure now has a non-promoting local
observation path, `scripts/capture_competitive_execution_r3_source_worktree.py`.
It uses only fixed read-only Git probes and records the current HEAD, the
existing staged+unstaged binary-diff hash contract, sorted untracked regular
file paths/content hashes, and recursive submodule revisions.  Raw untracked
content is never copied into the artifact.  Symlinks, hardlinks, non-regular
files, traversal paths, races, unreadable files, and bounded-size violations
fail closed.  JSON and its 0444 sidecar are sealed with `O_EXCL`,
`O_NOFOLLOW`, descriptor identity checks, fsync, and a second immutable read.

The result is `NOT_REVIEWED_EXTERNAL`, with `benchmark_eligible=false`,
`claim_eligible=false`, and no active-profile switch.  Validation reopens the
candidate/profile/source-manifest bindings and recaptures the worktree; it is
not an external custodian decision and cannot satisfy the acceptance manifest
without a separate signed review.  No capture was promoted to the checked-in
acceptance manifest in this change.

The producer binding intentionally distinguishes the compatibility wrapper
`competitive_execution_r3_source_worktree_v1.schema.json` from the strict
target `competitive_execution_source_worktree_snapshot_v1.schema.json`; both
paths and file SHA-256 values are sealed.  The signed review verifier reopens
both bindings (plus the sidecar schema), so a wrapper-only or strict-schema
substitution is rejected.  `scripts/validate_competitive_execution_r3_source_worktree_review.py`
accepts only a separately supplied READY custodian policy; the checked-in
zero-key policy remains `NOT_READY`, and even an accepted response is
offline, candidate-review-only evidence requiring a later acceptance
aggregator and profile reseal.

## Signed acceptance aggregation boundary

The additive `scripts/validate_competitive_execution_r3_acceptance.py` is a
verifier only; it contains no signer or key-generation path.  It reopens all
seven closure slots through fixed authoritative adapters, then binds each
closure's immutable path/file SHA/canonical identity, validator source, trust
policy, candidate, profile, campaign, signature, and one-shot nonce.  The
selection handoff and dataset-source closure now have dedicated signed
custodian packet validation in
`scripts/validate_competitive_execution_r3_unsigned_closure_review.py` rather
than being accepted from status fields.  Their checked-in trust policies have
zero keys and remain `NOT_READY`.

An externally signed aggregate, even when structurally valid, is always
`READY_FOR_SEPARATE_PUBLICATION_REVIEW` with benchmark, claim, active-profile,
and promotion flags false.  It is offline review evidence only and cannot
replace the checked-in seven-slot `PENDING` preflight, alter the active
profile, or publish a README/SOTA claim.  Missing, partial, duplicate,
cross-kind, expired, replayed, self-rehashed, symlinked, hard-linked, or
validator/policy-drifted inputs fail closed.  No external key, signature,
review packet, Docker run, dataset, GT, scorer, or benchmark was created in
this change.

The aggregate's `execution_authorized=true` is deliberately narrower than
publication or competitive-claim authorization.  `publication_authorized`,
`sota_claim_authorized`, and active-profile/promotion remain false, and
`requires_separate_publication_review` remains true.  The machine closure is
accepted only through the dedicated runtime signed-review adapter after the
authoritative PoP validator performs a live-host check; an offline machine
artifact cannot be promoted by projection.  Fresh-holdout reviews likewise
pass through the dedicated adapter, which reopens the profile and requires
the authoritative fresh-holdout verifier's external-attestation signature and
one-shot replay result.  The checked-in policies have zero keys and the
checked-in holdout profile has no valid external attestation, so both runtime
slots remain `NOT_READY` until an independently reviewed policy and subject
are supplied.  Neither wrapper infers `signature_valid` or
`replay_protected` from status fields.
