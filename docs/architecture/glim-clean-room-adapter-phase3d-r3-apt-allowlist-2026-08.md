# GLIM clean-room Phase 3d r3 apt allowlist candidate

This is an additive, fixture-only candidate.  It does not change the active
r2 profile, selection, receipts, rival set, or production offline manifest.
Its status is `OPT_IN_NOT_READY`, `benchmark_eligible=false`, and
`network_execution=NOT_RUN`.  No Docker, apt, resolver, download, dataset,
bag, GT, scorer, or benchmark operation was executed.

## Fixed precommit and resolver boundary

`apt_package_requirements.schema.json` accepts only versionless, explicit
package names and architectures in sorted build/runtime lists.  Its canonical
SHA is the precommit identity.  `plan_glim_clean_room_r3_apt_allowlist.py`
emits a fixed, non-executing argv workflow for the pinned Jazzy image
`ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f`
(`linux/amd64`, pull `never`).  Build and runtime receive distinct fresh apt
archive/cache roots; each role has its own `apt-get --print-uris` and
download-only phases.  The plan has no arbitrary shell fragments and records
`PROPOSED_REVIEW_REQUIRED`, `benchmark_eligible=false`, and
`PROVISIONING_ONLY_NOT_RUN`.  The plan also carries an explicit
`host_executor.status=IMPLEMENTED_NOT_RUNTIME_VALIDATED` marker bound to the
executor source hash.  Its `host ...` entries remain an auditable future-
workflow contract; the executor is opt-in and cannot be used without an
independent host authorization, so no runtime command is selected implicitly.

The planned host sequence is explicit and ordered: inspect the image, reject
pre-existing output/container names, create and lstat a fresh `0700` output
directory, run the pinned image with that directory as the only writable bind
and all requirements/composer/schema inputs as read-only binds, then run
`apt-get update`, base `dpkg-query`, apt/Release and rosdep snapshots, and
separate build/runtime `--print-uris ... install <names>` followed by
download-only `install <names>` phases.  The container is disconnected,
inspected, stopped, removed, and absence-checked before host-only staging and
composition phases.  Host phases use absolute paths below the fresh output
root; `/workspace/capture/...` appears only in Docker argv.  The plan records
both mount identities and a versioned outer-receipt schema for command,
stdout/stderr, raw snapshot, cleanup, and network-phase binding.  Each
resolver phase also seals its exact inner argv to a host-side JSON path; the
host composer CLI consumes that argv plus the captured raw log rather than
accepting an arbitrary shell command.

The resolver input is the strict JSON equivalent of a captured
`apt-get --print-uris` result.  Every URI line must have exactly four fields:
shell-quoted HTTPS URL, safe `.deb` filename, positive decimal byte count, and
either `SHA256:<64 lowercase hex>` or the apt-2.x-compatible
`MD5Sum:<32 lowercase hex>` locator.  The URL's percent-decoded basename must
equal the filename.  An MD5 value is never an authenticity root: the source
snapshot must include the corresponding signed Packages representation and
decoded plain index, whose Release-bound path/URL/SHA, Filename, Size and
SHA256 are checked against the downloaded deb bytes.  A URI SHA256, when
present, must also equal that index SHA256.  Unknown digest labels, duplicates,
traversal, ambiguous query or
fragment URLs, shell fragments, missing transitive packages, and nonzero
resolver status fail closed.  A resolver package record binds the URI to
`dpkg-deb` Package/Version/Architecture, Essential/Pre-Depends/Multi-Arch,
repository URL, Release/InRelease path/SHA, and the preinstall base-status
SHA.  A package already present in the base status is explicitly marked
`base_installed` and has no acquired URI/deb; it is excluded from the new
allowlist/ledger.  Same name+architecture with different versions across
roles is rejected; distinct explicit architectures remain distinct identities.
Pre-Depends must resolve in the role or base set.

The signed Packages representation policy is fixed to the ordered candidates
`.xz`, `.gz`, then plain `Packages`.  The first available candidate must have
both its compressed (or plain) path and its decoded `Packages` path/bytes
bound by signed Release SHA-256/size entries.  APT's local `.lz4` cache,
`.bz2`, unsigned or stale cache, and any external-codec or network fallback
are rejected.  For compressed candidates, bounded streaming standard-library
decoding enforces 64 MiB compressed and decoded limits, a 256:1 expansion
limit, mandatory EOF, and rejection of truncation, decoder errors, trailing
bytes, and concatenated streams.  These artifact limits are independent of
the separate 16 GiB bound for signed Release numeric size declarations.  That
declaration-only parser bound accepts canonical ASCII unsigned decimal text
through `17179869184`; compressed, decoded, downloaded, and `.deb` limits do
not change.

## Host composer and review gate

`apt_allowlist_composer.py` is filesystem-only.  Its
`seal_base_status_snapshot` and `build_resolver_output` helpers consume the
host-captured raw `dpkg-query`/`--print-uris` logs, downloaded deb roots, and
source snapshot; they never run apt, dpkg, Docker, or a network client.  The
resolver documents are therefore runtime-generated inputs, not synthetic
prebuilt inputs accepted by the host composer.  It reopens the base status,
both resolver documents, the apt/Release/rosdep source snapshot, and separate
build/runtime downloaded-deb roots using no-follow, single-link, bounded
reads.  It computes each deb SHA/size and invokes only an injected fixture
`dpkg-deb` metadata reader; production execution is intentionally absent from
this non-executing candidate.  A future executor must provide the fixed
metadata command and bind its stdout/exit record in the outer receipt.
Release and Packages bytes must match the repository ledger.  The composer
strictly parses the Release/InRelease `SHA256:` section and requires exactly
one entry whose path (derived from the Packages URL) has the same Size and
SHA256 as the captured Packages bytes; duplicate, missing, or URL-ambiguous
entries fail closed.  This is a Release metadata binding, not an OpenPGP
signature verification.  Signature validation and its outer receipt remain a
future host/custodian gate and were not run.  Missing/extra/symlinked/
hard-linked debs, URI/deb/control/SHA/size drift, source drift, base mutation,
resolver self-hash drift, mixed role closure, or a non-fresh output root is a
hard failure.  The output contains a canonical collector-compatible
`package-allowlist.json` and `apt-acquisition-ledger.json`, plus the complete
role/base/resolver/source binding, but remains a
`PROPOSED_REVIEW_REQUIRED` proposal.

The proposal is not collector input.  `materialize_reviewed` requires a
separate `REVIEWED_FOR_COLLECTOR` review document with a non-empty custodian
signature/key identity bound to the exact proposal SHA *and* a referenced
per-repository signature receipt index.  The strong host receipt gate still
rejects the current fixture-only executor outputs, so no collector input can
be materialized.  No production manifest is edited automatically.  Candidate
tool/schema hashes are separately bound in `apt_allowlist_candidate.json`;
`validate_glim_clean_room_r3_apt_allowlist.py` rejects pending or drifted
hashes.

## OpenPGP signature candidate (NOT_RUN)

`apt_release_signature.py` and
`plan_glim_clean_room_r3_apt_signature.py` add an additive, non-promoting
signature contract. A future host must precommit a no-follow keyring byte
artifact, its primary fingerprint/key ID, signing-subkey fingerprints,
creation/expiry/revocation metadata, a fixed verification timestamp, and the
exact `gpg` binary identity. The fixture validator rejects unknown, bad,
expired, revoked, or otherwise untrusted status records; `gpgv` exit status
alone is explicitly insufficient because it does not establish expiry or
revocation state. The planned verifier uses `--no-default-keyring`, disables
automatic key retrieval/import/location and autostart, emits status-fd data,
and separately inventories the explicit keyring with
`--with-colons --with-fingerprint --with-subkey-fingerprint --show-keys`.
The `VALIDSIG` primary-key fingerprint is checked at its documented optional
field after the signature class, while `GOODSIG` may name an explicitly
precommitted signing subkey.

The receipt reopens and binds the status log, key-inventory log, exact argv,
tool identity, fixed-time key metadata, detached `Release.gpg` bytes when
applicable, and the existing Release SHA256 → Packages → downloaded-deb
binding identity. Its ordered phases include trust/release reopen, signature
and inventory verification, binding reopen, sealing, and cleanup. The
additive `apt_release_signature_executor.py` now provides a bounded host
executor candidate: it requires a full source-revalidated plan, an absolute
precommitted binary path/SHA/version, fresh 0700 output and GnuPG directories,
`shell=False`, a fixed environment, `stdin=DEVNULL`, timeout/resource limits,
exclusive bounded logs, inventory-before-signature ordering, and no-default-
keyring/no-auto-key/no-agent flags. It reopens every source and command
projection before validating the sealed result, rejects self-rehashed or
swapped output roots, and records cleanup identity/absence. Failed attempts
are sealed as non-promoting results; cleanup or phase failures cannot become
PASS. The default subprocess runner is deliberately disabled until an
independent host authorization receipt exists; tests inject only a synthetic
runner. Thus no keyring, GnuPG process, Docker, network, apt, or package
acquisition was run. The executor is
`IMPLEMENTED_NOT_RUNTIME_VALIDATED`, `signature_runtime=NOT_RUN`, and
`benchmark_eligible=false`; an independently reviewed trust root and a
host-owned runtime authorization/cleanup receipt are still required.

## Host runtime authorization (NOT_READY)

`apt_release_signature_authorization.py` is the separate Ed25519 gate for the
future `SubprocessRunner` path. It signs a domain-separated payload containing
the exact candidate and fixed policy identities, plan canonical/file identity,
executor source hash, trust-root/keyring and Release/Packages/signature/deb
bindings, GnuPG binary path/SHA/version, root/homedir/output paths, a
precommitted machine-identity artifact, validity interval, nonce, no-network/
no-shell policy, resource limits, and the complete expected workflow/argv.
Only the policy path bound by `apt_allowlist_candidate.json` is accepted;
caller-selected trust stores and self-declared public keys are rejected. The
policy also binds the cryptography implementation file/SHA and exact backend
version. Machine identity is a strict tagged union: a READY policy selects
exactly one legacy v1 artifact or keyed `PROVISIONED_ED25519` artifact and binds
its method/schema/file/canonical/identity plus keyed public-key, challenge, and
proof projections. Initial keyed authorization performs live validation;
receipt reopen performs offline validation and preserves the sealed live-host
evidence. The checked-in policy is `NOT_READY` with no authorized key and no
machine artifact, so it cannot authorize a process. A keyed
`provisioning_receipt_sha256` is only an external receipt identity here, not a
cryptographic verification of that receipt.

The executor obtains the host wall-clock time and verifies it is inside the
signed authorization interval before creating the output root or starting a
command. That actual execution time is sealed in both an atomic, persistent
nonce claim and the receipt; reopening uses that sealed time, so a receipt
that was valid before later expiry remains reproducible while an expired
authorization is rejected before its first output. Output-root, parent
identity, machine artifact, and all source/tool identities are reopened on
receipt validation. `runner=None` and
an explicit `SubprocessRunner` therefore fail closed without authorization;
injected fake runners remain a synthetic, non-promoting path and cannot be
paired with an authorization. A successful run is labelled
`HOST_AUTHORIZED_SUBPROCESS` / `RUNTIME_VERIFIED`; a failed authorized attempt
is labelled `RUNTIME_FAILED`, `benchmark_eligible=false`, and
`promotion=FORBIDDEN`. Only the successful state is accepted by the
per-repository promotion gate. No private key or signed PASS
receipt is checked in, and no GPG/network/apt/Docker execution was performed.

## Per-repository promotion binding (NOT_READY)

`apt_signature_promotion.py` is now the shared fail-closed boundary used by
allowlist materialization and the collector's explicit promotion mode.  A
review must reference the new `apt_signature_receipt_index.schema.json`; the
index contains exactly one receipt root, plan file/identity, input root, and
complete deb binding for each repository `(url, release_url)` in the
acquisition ledger.  The validator rejects missing, duplicate, extra, and
cross-repository replay entries, then reopens the executor's full
plan/source/trust-root/Release/Packages/deb projection and receipt sidecars.
Static validation, self-rehashed JSON, fixture/fake-runner output,
`INJECTED_FIXTURE_ONLY`, `NOT_RUNTIME`, and `promotion=FORBIDDEN` are never
promotion evidence.

`materialize_reviewed` cannot produce collector input without this index and a
successful host-authorized receipt for every repository.  `seal_to_capture`
retains its review-only direct path for synthetic capture evidence; when
`collector_input_allowed=true` is requested it requires the same index and
gate, and fails closed otherwise.  The current candidate has no authorized
GnuPG execution, so this gate intentionally rejects all current receipts.
Neither the active r2 profile/selection/receipts nor README claims are
changed or promoted.

## Evidence status

The focused synthetic suite exercises valid build/runtime composition and
review-only materialization, URI quoting/percent encoding, MD5 locator plus
Packages-SHA binding (and malformed digest rejection), duplicate URI rejection,
missing transitive packages, Release/Packages and base-status
drift, resolver status/self-hash errors, deb extra/symlink/control/byte drift,
Pre-Depends, and cross-role version conflicts.  These fixtures do not claim
that any package was acquired or that the pinned image contains the closure.
The candidate remains pending a future independently reviewed host capture of
the exact image, apt sources/Release bytes, rosdep bytes, all versioned deb
bytes, dpkg status, and license/copyright artifacts.
