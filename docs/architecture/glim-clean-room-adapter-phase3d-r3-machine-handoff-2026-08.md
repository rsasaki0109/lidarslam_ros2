# GLIM clean-room Phase3d r3 machine identity and custodian handoff

This document records an opt-in, non-promoting handoff boundary for the
Phase3d r3 OpenPGP executor. It does not change the active r2 profile, create a
trust key, capture this host, run GnuPG, or make the candidate benchmark-ready.

## Machine identity

`apt_machine_identity.py` emits `glim_clean_room_r3_machine_identity_v1` and a
read-only `.sha256.json` sidecar. The capture boundary reads only stable private
machine inputs (machine-id, DMI UUID, and board serial); their raw bytes never
leave the capture function. At least two distinct, meaningful stable
identifiers are required: common DMI placeholder values (for example “to be
filled by o.e.m.”, “default string”, and “system serial number”) and duplicate
raw values do not satisfy that minimum. The artifact contains only
domain-separated SHA-256 projections and public architecture/CPU/memory/kernel/
toolchain fields. Hostname, username, and MAC addresses are not collected
because they are volatile and unnecessary.

The artifact binds campaign ID, producer module/source SHA/version, canonical
identity SHA, capture time, file bytes, and file SHA. It rejects symlinks,
hardlinks, parent symlinks, unknown fields, source drift, canonical drift, and
TOCTOU changes. The two output files are sealed as a fresh pair. Offline reopen
returns `HOST_MATCH_NOT_CHECKED` (`live_host_match: false`); only an explicit
live validation performs a current-host projection comparison. `LIVE_CAPTURED`
is required before an authorization policy can use the artifact; synthetic test
artifacts cannot enter a READY policy.

No repository machine artifact is checked in. A real capture remains a future
custodian operation and is not represented by the checked-in NOT_READY policy.

### DMI-less keyed fallback (additive, opt-in authorization method)

When a host exposes only one meaningful stable identifier, the v1 two-ID
contract still fails closed. The additive keyed path in
`apt_machine_identity.py` does not weaken that rule. An operator supplies an
external, read-only Ed25519 public-key descriptor with a nonempty provisioning
receipt identity. `prepare-key-challenge` then binds the campaign, keyed
domain, current machine-id hash, public host projection, key identity, source
identity, fixed candidate-manifest identity, intended output filename, and a
fresh random challenge nonce. A separate external signer
returns only a detached proof document; `finalize-keyed` verifies it and emits
the final artifact. No command in this repository reads, creates, stores, or
logs private-key material.

The challenge fixes the intended final output filename before signing;
finalization requires that fresh path. It is therefore bound to the signed
campaign/candidate/output-name/nonce tuple, but a copied immutable challenge
directory is not, by itself, a universal replay-prevention mechanism. The
authorization executor's separately signed nonce claim is the one-shot
execution boundary. A changed challenge, proof, key descriptor, campaign,
machine-id projection, or public projection fails closed.

The key descriptor, challenge, proof, final artifact, and their sidecars are
bounded regular single-link files. All source paths are same-directory safe
names, and every source is reopened by bytes, canonical hash, sidecar hash,
campaign, key, challenge, and signature identity. Finalization re-reads the
current machine-id and public projection, so a copied artifact cannot claim a
different host in live mode. Offline validation reports
`HOST_MATCH_NOT_CHECKED`; it is not a host-match claim. The external
provisioning receipt is an input binding, not an independent trust decision:
custodian policy must still review the key before any READY authorization.

The keyed method is an additive tagged-union branch in the authorization policy,
authorization document, executor receipt reopen path, and custodian handoff.
A future READY policy must select exactly one method (legacy v1 or keyed
`PROVISIONED_ED25519`) and bind its exact schema, file/canonical/identity
projections, public-key descriptor, challenge, and proof. Initial authorization
verification invokes keyed live validation; receipt reopen invokes keyed offline
validation and preserves the sealed `live_host_match: true` evidence rather than
claiming that the current host was checked. Handoff accepts a keyed
`LIVE_CAPTURED` artifact but records `OFFLINE_ARTIFACT_ONLY`.

The checked-in authorization policy remains empty `NOT_READY`; no authorized
key or keyed machine artifact is checked in. `provisioning_receipt_sha256` is
only a precommitted external receipt identity and is not cryptographically
verified by this repository. Custodian review and a separately signed
authorization remain required before any real runner can be enabled.

## Custodian request

`apt_release_signature_handoff.py` and
`apt_release_signature_handoff.schema.json` define an unsigned
`UNSIGNED_REVIEW_REQUIRED` request. `prepare-request` accepts only sealed JSON
files for the plan, repository, and deb-binding ledger; it never accepts an
inline trust root/key or generates a key/signature. The request reopens and
binds:

* fixed candidate and NOT_READY policy identities;
* executor, authorization verifier, cryptography backend/version/source hash;
* a LIVE_CAPTURED machine artifact, campaign identity, and offline-only
  validation status;
* a fully validated plan, trust root, Release/Packages/deb binding, and source
  bytes; and
* exact root/scope/safety values, validity window, nonce, and empty custodian
  key/signature fields.

The output and JSON sidecar are fresh, single-link, read-only, and reopened
before a successful result is returned. Tampering, replay across campaigns,
source or plan drift, symlink/hardlink replacement, scope drift, arbitrary
custodian key material, and an offline-as-live claim fail closed. A separate
external custodian must review this request and return policy/authorization
material; that process is outside this repository and remains NOT_READY.

## Custodian response review (implemented, non-promoting)

`apt_release_signature_handoff_response.py` and its response/sidecar schemas
validate a returned `READY_AUTHORIZATION_RESPONSE` as an offline review
artifact. The verifier reopens the immutable request and its request sidecar,
recomputes the complete request binding, then reopens the candidate, READY
policy, and signed authorization bytes. It checks the candidate and policy
file/canonical identities against the exact request template, the policy's
candidate and machine/backend projections, the authorization body against the
template (with only custodian validity and detached signature fields filled),
and the existing Ed25519 verifier against the reopened Release/Packages/deb
context. The verifier implementation, schemas, backend version, authorization
payload identity, and `live_host_match=false` offline result are also sealed in
the response binding. The immutable binding records `authorization_issued_at`,
not a verifier-selected review time. Operational `validate-response` obtains
its own integer wall clock and passes that review time to the existing
authorization verifier, requiring `issued_at <= review_time < expires_at`; the
response author cannot choose it. The returned `review_time`/current-status
projection is an ephemeral, non-promoting review result, not a sealed claim.
Historical structural reopening must not be described as current
authorization validity. Response and sidecar files must be fresh, immutable,
single-link regular files; path replacement, sidecar drift, mixed method
branches, request/policy/candidate swaps, expanded validity/scope, and
self-rehashed metadata fail closed.

The checked-in request producer intentionally pins the checked-in candidate and
`NOT_READY` policy. Therefore the current repository cannot accept a READY
response merely by placing a different policy beside the request: the exact
candidate/policy bytes must first be reviewed and resealed by the external
custodian workflow, followed by a new request bound to those bytes. Synthetic
tests exercise that future response path only; they do not add a key, generate
a signature, enable the executor, or promote the candidate. Offline response
review never claims that the response machine is the current host.

## Evidence status

| Boundary | Status | Promotion |
|---|---|---|
| machine identity schema/module | IMPLEMENTED, synthetic-tested | not captured on this host |
| live machine comparison | IMPLEMENTED, test-only provider seam | not run |
| custodian handoff request | IMPLEMENTED, synthetic-tested | `UNSIGNED_REVIEW_REQUIRED` |
| custodian READY response review | IMPLEMENTED, synthetic-tested | non-promoting, requires external reseal |
| GPG/network/apt/Docker execution | NOT_RUN | forbidden in this task |
| active r2 / README / production policy | unchanged | no claim |

The checked-in policy keeps `authorized_keys: []` and an empty NOT_READY machine
identity projection. This candidate therefore cannot authorize a real runner.
