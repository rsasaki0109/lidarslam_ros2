# Competitive execution r3 machine-identity proof of possession

This is an additive, opt-in boundary for the competitive execution r3
candidate.  It does not alter the active profile, r2 receipts, selection,
README claims, or the checked-in authorization policy.  The checked-in policy
has no authorized keys and remains `NOT_READY`; no benchmark or claim can be
promoted by this path.

## Contract

`capture_competitive_execution_machine_identity_pop.py` creates a challenge
for an externally provisioned Ed25519 public key.  It never reads, creates,
stores, or logs private key material.  The challenge binds:

* the candidate file and canonical identity, profile file and canonical
  identity, campaign/domain, and the exact seven-field thread policy;
* the machine-id only as a domain-separated SHA-256 value, plus non-secret
  public facts (architecture, CPU model, logical CPUs, memory, kernel, and
  observed affinity);
* the public-key descriptor, key/provisioning receipt identities, validity
  window, final output basename, absolute replay-ledger path, and fresh nonce;
  and
* the producer source/version and fixed trust-policy reference.

The raw machine-id and optional DMI/board values are used only inside the
capture provider.  They are never placed in JSON, exception text, logs, or
the detached proof.  A DMI-less host is still fail-closed if its machine-id
anchor is unavailable.  This path does not weaken the legacy two-distinct-ID
machine policy; the external key is an independently provisioned second
anchor for the additive method.

An external custodian signs the canonical challenge payload.  Finalization
requires the current host projection to match the signed challenge and checks
the actual verification time against both challenge and key validity.  A
successful final artifact is `LIVE_CAPTURED`, but remains
`NOT_REVIEWED_EXTERNAL`, `benchmark_eligible: false`, and
`claim_eligible: false`.  A future reviewed policy must select exactly one
machine method and bind the full artifact/proof/key/candidate/profile
projection; this implementation does not perform that promotion.

## Immutable evidence boundary

Challenge, key descriptor, proof, final artifact, sidecar, and nonce claim
files are fresh bounded regular single-link files.  They use
`O_EXCL|O_NOFOLLOW`, mode `0444`, bounded no-follow reads, full-write loops,
`fsync`, parent-directory `fsync`, and byte/size/device/inode reopens.  The
sealing code rejects a zero-progress write and fixes the parent directory
device/inode across every write, fsync, reopen, and owned cleanup.  Challenge
and final outputs must share one parent directory.  A nonce claim is created
with the same immutable sealing boundary and is the one-shot execution
claim.

The signed challenge is bound to its campaign, candidate, output basename,
absolute ledger path, and nonce.  Copying an immutable challenge directory to
an arbitrary path is not claimed to be universal replay protection; the
authorization executor's separately signed nonce claim is the one-shot
execution boundary.  Offline artifact validation reports
`HOST_MATCH_NOT_CHECKED`; only the initial finalization/live validation may
report a current-host match.  An external provisioning receipt is an input
identity binding only, not a trust decision.

## Status and evidence

The checked-in trust policy remains `NOT_READY` with `authorized_keys: []` and
`authorized_runtime: false`.  The challenge-only CLI therefore provides a
safe preflight only when a temporary test policy is explicitly supplied by
tests; the production CLI cannot enable a runner.  Synthetic Ed25519 keys and
proofs exist only under temporary test roots.  No real key, signature, host
capture, GPG, network, APT, Docker, bag, GT, scorer, or benchmark execution is
part of this candidate.

The focused test file
`graph_based_slam/test/test_competitive_execution_machine_identity_pop.py`
covers candidate/profile/thread binding, raw-identifier non-disclosure,
NOT_READY policy behavior, live/offline separation, validity boundaries,
signature/key/campaign/host tampering, nonce replay, sidecar immutability,
output collision, and strict schemas.  The artifact is an external review
input only; it does not update the candidate, active profile, handoff, or
README.

## External custodian handoff packet

`prepare_competitive_execution_machine_identity_pop_handoff.py` is the
non-promoting bridge for an external custodian.  It accepts an externally
supplied key descriptor, detached proof, and separate READY trust-policy file,
but rejects the checked-in policy path and requires the external policy's
immutable `0444` sidecar.  The policy must bind the current candidate/profile,
backend/source identity, exactly one ACTIVE key matching the descriptor, and
the challenge validity interval.  A proof is accepted only after the existing
PoP verifier reopens the challenge, key, proof, nonce claim, and
`LIVE_CAPTURED` artifact.

The packet additionally binds the candidate's seven-field thread policy and
the canonical projections of the required systems, release, runner/scorer/
memory source bindings, image/toolchain identities, dataset/holdout closures,
and legal status.  Challenge/key/proof/artifact/policy files must be bounded
single-link immutable files under a recorded asset-root identity; each source
and sidecar is reopened after verification to close TOCTOU replacement.  The
packet itself is an immutable `UNSIGNED_REVIEW_REQUIRED` pair and reports
`HOST_MATCH_NOT_CHECKED`, `benchmark_eligible: false`, and
`claim_eligible: false`.  It contains no private key and cannot switch the
profile or promote a README claim.  A separate custodian review and explicit
candidate/profile reseal remain mandatory.  A copied packet is not treated as
universal replay prevention; the signed PoP challenge and executor nonce
claim remain the one-shot boundary.
