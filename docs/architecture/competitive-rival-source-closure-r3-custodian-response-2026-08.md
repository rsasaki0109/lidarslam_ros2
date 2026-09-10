# Competitive rival-source closure r3 custodian response

Status: additive candidate only — `NOT_READY`, benchmark-eligible `false`,
claim-eligible `false`, and no active-profile switch.

This boundary validates a detached Ed25519 response from an external legal
custodian.  It is separate from the GLIM APT/GPG host-authorization path.  The
checked-in trust policy is intentionally `NOT_READY` with an empty key list;
no private key, public trust key, or signed production response is stored in
the repository.  Synthetic keys are used only by temporary tests.

The verifier reopens and binds:

* the immutable unsigned r3 handoff, current r3 candidate and r3 selection;
* the complete immutable r2 lineage and current candidate/selection identities;
* the legal-capture packet, its exact capture identity, closure/selection
  binding, producer SHA, and capture-schema SHA; and
* exactly four concrete legal blockers: GLIM ROS2 license text, FAST-LIVO2
  declaration conflict, rpg_vikit artifacts, and Sophus artifacts.

Every blocker decision is one of `ACCEPTED`, `REJECTED`, or
`NEEDS_CLARIFICATION` and carries immutable evidence, the exact upstream
repository/revision from the inherited closure, a license identity, build and
execute policy, redistribution policy, publication policy, and a bounded
review note.  An accepted decision requires an actual immutable `LICENSE_TEXT`
artifact; `package.xml`, `package.json`, and manifest declarations are rejected
as license text.  A response whose capture packet still says
`remote_policy.responses_status=NOT_RUN` is not treating that packet as legal
approval: acceptance relies on the four separately signed custodian decisions.

The response payload is domain-separated and signs key metadata, backend
identity, candidate/selection/capture bindings, all decisions, validity, and
nonce.  Review uses the verifier's current wall clock (`issued_at <= review
time < expires_at`); the response cannot choose review time.  The caller must
provide an existing replay ledger.  A nonce claim is created with
`O_EXCL|O_NOFOLLOW`, looped writes, `fchmod(0444)`, fsync, parent fsync, and
post-write identity/content reopening.  This is one-shot protection within
that fixed ledger, not a claim that copying artifacts to another ledger is
globally impossible.

Response and sidecar files are no-follow, single-link, bounded, immutable
`0444` artifacts.  `ACCEPTED` never automatically changes the active profile,
README, benchmark eligibility, or SOTA publication.  A separate reviewed
candidate/profile reseal and promotion procedure remains required.  Rejected
and clarification responses are retained as first-class, non-promoting review
results.

The checked-in policy and all production paths remain `NOT_READY`; no external
legal packet, signature, Docker execution, network access, dataset, GT, scorer,
or benchmark was used to produce this candidate.

## Sealed source identities

These hashes are recomputable with `sha256sum` from the repository root.  The
policy's internal identity is the canonical SHA-256 of its JSON object with
`policy_identity_sha256` omitted, using sorted compact JSON plus a final LF.

| Artifact | SHA-256 |
|---|---|
| `scripts/validate_competitive_rival_source_closure_r3_custodian_response.py` | `66772f039313ab823d4bc239aa6283b9c774e753409fb66d6028fc03dac8c3a5` |
| `graph_based_slam/test/test_competitive_rival_source_closure_r3_custodian_response.py` | `f8f1f03cc135e44a805a50ac631fd4f0aad8d37f12bc35124ef817fc804bea38` |
| response schema | `8d88ae0c96b757fc38dbd44488055c5a0b9d42dc47ad125c61892a8ed52c76d4` |
| response sidecar schema | `b6d8558137ab58b12e43c5697a46142cdddf385dc14ac289026172f77d3cb453` |
| trust-policy schema | `edbc5c81fac7cdd59b6c29a8d97624a76a1cedd48c88e64cfa8bc52023a5199b` |
| checked-in trust policy | `181192cb3be103361136efa2f1f2a5c9bbb1dfb67f9ffc62537b5d622b5ce4c6` |

The checked-in policy canonical identity is
`7d506d0bd8ea412836350047e9c57b411de3b1cc357fd89087e6a6e867117661`; its
status is `NOT_READY` and `authorized_keys` is the empty list.
