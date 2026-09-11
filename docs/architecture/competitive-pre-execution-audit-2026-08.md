# Competitive SOTA pre-execution completion audit (2026-08)

Status: **NOT_READY**.  This is a static requirements-to-evidence audit; it
does not open datasets, ground truth, bags, scorers, Docker, or the evidence
volume.  The current profile is not promoted by this document.

| Objective requirement | Enforced boundary | Static result |
| --- | --- | --- |
| Pinned rivals and immutable recipes | rival source-closure verifier, profile closure identity, runner preflight | NOT_READY: legal provenance/closure blockers remain |
| Precommitted datasets and fresh holdout | dataset source closure and fresh-holdout authorization | NOT_READY: independent custodian attestation and acquired/revalidated bytes are absent |
| Same input/calibration/hardware/thread/Release | selection identity, machine/thread/resource receipts, suite identity checks | Contracted; no production run evidence is available |
| At least three complete runs for every system/sequence | v2 suite coverage plus canonical per-attempt receipt | Claim path requires one unique sealed receipt per slot and one common campaign ID |
| No manufactured repetitions | execution receipt canonical SHA plus exact `execution_receipt_file_sha256` bundle binding | PASS in synthetic adversarial coverage; cloned receipt/mixed campaign fails closed |
| 100% completion and no serious sequence failure | per-run completion/sequence gate and bundle coverage | Contracted; production evidence absent |
| APE improvement and fixed 95% CI | v2 aggregate/CI evaluator and publication renderer | NOT_READY until all closures and authorized scoring pass |
| RTF, RSS, and map quality | sequence, memory, and suite gates; resource identity | Contracted; functional-only/contaminated receipts rejected |
| Failure publication | canonical authorization failure log and global failure artifact | Contracted by publication/composer; no production claim exists |

## Attempt identity boundary

`scripts/run_competitive_gt_blind_benchmark.py` seals each attempt as
`competitive_execution_attempt_receipt_v1`.  The receipt's
`execution_receipt_sha256` is a canonical compact-JSON self-hash excluding
only that field.  The pretty `attempt.json` file has a separate
`execution_receipt_file_sha256`, recorded by the completion/run index after
sealing (the completion manifest keeps this in a separate
`execution_receipt_files` index); these identities must never be conflated.  The bundle composer copies
the exact file and its sidecar, while the verifier reopens and parses it,
checks system/sequence/repetition/campaign/completion/exit/GT-blind fields,
and compares both hashes.  Equal trajectory, map, score, or metric bytes are
allowed for deterministic repeated runs; reusing an execution receipt is not.
The receipt parser also preserves truthful failed/incomplete attempts for the
failure ledger (`require_success=False`); only the claim bundle path enables
`require_success=True`, so a failed receipt cannot be relabelled as a complete
run.

All claim-bound runs must use one campaign identity.  A row with a different
campaign, a missing file hash, a failed/timed-out receipt, duplicate receipt,
missing bundle execution artifact, or malformed/duplicate JSON keys fails
closed.  The field-level distinction is represented in
`competitive_execution_attempt_receipt_v1.schema.json` and the bundle,
composer, and handoff schemas.

## Remaining blockers

The authoritative campaign cannot be claim eligible: the profile's fresh
holdout authorization lacks independent external-custodian attestation, the
dataset bytes/calibration identities are not acquired and revalidated, and
the required rival source closure retains legal-provenance blockers.  No
metric, CI, RSS, map, completion, or failure result in this audit changes that
status.  A future execution must reopen all receipts and run the complete
required-system-by-sequence-by-repetition matrix before publication.
