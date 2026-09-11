# Registration-plugin dependency closure (2026-08)

The four release rows use a connected-provisioning plus disconnected-runtime
dependency contract.  While a pinned
Humble or Jazzy container is connected, an external provisioning worker may
capture the exact APT/rosdep resolution and the two optional source archives.
The capture packet must include the requested and final HTTPS URLs plus only
the profile-bound HTTP APT prefixes
`http://archive.ubuntu.com/ubuntu`,
`http://packages.ros.org/ros2/ubuntu`, and
`http://security.ubuntu.com/ubuntu`.  HTTP matching is host/path-boundary
exact, uses the default port only (`80`), and rejects credentials, queries,
and fragments; the raw requested URL is retained.  rosdep remains HTTPS-only.
The packet also includes distro and `linux/amd64` identity, resolver argv and
exit records, base `dpkg` status, APT source/Release/Packages bytes, downloaded
`.deb` bytes and SHA-256 values, rosdep source/cache identities, and bounded
host-owned logs.  URI MD5 values are locators only; a downloaded package is
accepted only after gpgv evidence, and only when its bytes and `dpkg-deb`
fields match the SHA-256/size entry in a signed Release-bound Packages index.
The chain is therefore signed Release SHA-256/size entry -> exact Packages
bytes and parsed entry -> downloaded deb SHA-256/size and package identity.
HTTPS and profile-allowed HTTP redirects are same-host-only, every command has
a bounded timeout, and no retry is permitted.

The rosdep prepare receipt carries two deliberately different profile
identities.  `discovery_profile_sha256` is the immutable profile hash recorded
by the sealed discovery-11 packet (`6f4faf87...`); it is never rewritten when
the consumer profile evolves.  `consumer_profile_sha256` is computed from the
current active profile at receipt time and is accompanied by an exact
`consumer_profile` path binding.  The active profile must explicitly
allowlist the discovery receipt path/bytes/canonical hash, root, image,
commit-addressed archive, package artifacts, and selected files.  A consumer
profile hash that differs from the historical discovery hash is expected;
missing, cross-discovery, drifted, or rewritten bindings are fail-closed.  This
identity separation does not promote the packet or switch the active profile.

`scripts/registration_plugin_dependency_closure.py` reopens those bytes and
composes a fresh immutable closure.  It does not run APT, rosdep, Docker, or a
network client.  Its output is always `REVIEW_REQUIRED` with
`benchmark_eligible=false`; the signature/key receipt is explicitly
`REQUIRED_NOT_PROVEN` until an independent custodian review exists.  The
current profile therefore remains `NOT_READY` and cannot be used to start a
formal leg.

The connected-row producer is
`scripts/capture_registration_plugin_dependency_closure.py`.  Its
`registration-plugin-dependency-capture-executor-v1` receipt is a strict,
sidecar-sealed record of the fixed 19-phase command plan, image and campaign
bindings, host-owned logs, and any captured source/dependency artifacts.  A
complete row and a truthful partial failure both remain
`REVIEW_REQUIRED`/`benchmark_eligible=false`; the companion schema is
`configs/slam_benchmark_profiles/registration_plugin_dependency_capture_executor_v1.schema.json`.
The composed closure carries an optional `capture_executor` reference that
binds the exact capture-input bytes and schema SHA without introducing a
circular receipt hash.  Runtime validation still rejects this unsigned review
state and requires a separate signed promotion receipt; a capture receipt is
evidence of provisioning, not runtime authorization.

The `apt_source_snapshot` phase has one responsibility: APT source files,
repository keyrings, signed Release metadata, Packages indexes, and their
gpgv evidence.  It does not scan `/etc/ros/rosdep`, `/usr/share/rosdep`, or
rosdep caches, and a base image with none of those paths is therefore a valid
APT-phase result.  Any legacy v2 source manifest that includes a
`rosdep_source` record is rejected.  Rosdep source and cache evidence is
created only by the sealed `rosdep_prepare` phase after
`network_disconnect`; a completed prepare receipt must carry non-empty source
bindings and cache entries.  A missing or partial prepare receipt remains a
truthful `REVIEW_REQUIRED` failure and cannot be inferred from the APT
snapshot.

The current plan orders `network_disconnect`, `rosdep_prepare`, and
`rosdep_resolution` explicitly.  The prepare phase consumes only the pinned
discovery-11 receipt, its four sealed deb inputs, and its eight selected
rosdistro files; it does not provision or resolve through a mutable URL.  Its
tool and schema hashes are separate contract bindings, and its projection is
required in both complete and partial capture receipts.  A missing, stale, or
tampered prepare projection is therefore a capture-contract failure, not an
implicit legacy 16-phase success.

The row distro is also an explicit prepare input.  The outer executor passes
exactly `--distro humble` or `--distro jazzy` from the immutable row identity,
and binds that argv in both the phase hash and prepare-script projection.  The
helper extracts the matching `humble/distribution.yaml` or
`jazzy/distribution.yaml` member from the pinned rosdistro commit archive,
checks its distro-specific byte count and SHA-256, emits a one-distribution
offline index, and invokes `rosdep resolve --rosdistro` with the same value.
Unknown or mismatched values fail closed.  This binding was added after the
sealed `rosdep-warning-prefix-v3` campaign truthfully exposed the former
Humble-only prepare path in Jazzy row 3: Humble rows 1 and 2 completed all 19
phases, while Jazzy/Noble rejected the Humble `libg2o` definition and row 4
was not started.  That partial campaign is diagnostic evidence only and is
never reused or promoted.

The subsequent sealed `distro-bound-v4` campaign proved that correction in a
real Jazzy container: the prepare receipt selected the exact pinned Jazzy
distribution (393061 bytes, SHA-256 `0c67bbab...e124f9a8`) and the Jazzy
`rosdep resolve` command exited zero.  It then exposed a separate Packages
grammar edge at phase 17.  Noble universe contains legitimate deb822 control
fields such as `X-Cargo-Built-Using:` whose first value is empty and whose
value begins on the next whitespace-prefixed continuation line.  The parser
now accepts that form only for a syntactically bounded optional field with an
immediate continuation.  Empty required identity fields, an empty optional
field without a continuation, malformed names, duplicates, and all existing
package identity/size/SHA checks remain fail-closed.  The v4 partial root is
also retained only as diagnostic evidence and is not reused.

The sealed `deb822-v5` campaign then completed all 19 Jazzy row-3 commands,
including the corrected Packages parser and a successful in-container
dependency capture.  Final composition exposed one more producer/consumer
contract mismatch: the Jazzy base image retains a comment-only legacy
`/etc/apt/sources.list` alongside its active deb822 files.  The v2 producer
correctly represents that inactive file as a sealed non-empty artifact with
`urls=[]`, while the composer formerly rejected every empty per-file URL set.
The composer now permits an empty URL projection only for a v2 `apt_source`
file.  It still requires canonical sorted/unique URL lists, non-empty active
`source_entries` for the packet, exact reconstruction of every entry, and
non-empty signed repository/Release/Packages evidence.  Empty URL sets for
legacy snapshots or any other role remain invalid.  The v5 partial campaign
is retained as diagnostic evidence only and is not reused.

The sealed `inactive-source-v6` campaign proved the inactive-source change
and again completed all 19 commands in Jazzy row 3.  Final composition then
exposed a second copy of the Packages grammar in the r3 allowlist composer:
the capture parser accepted Noble's valid empty-first-value continuation, but
the composer still required `: ` on every non-continuation line.  The composer
now applies the same narrow Debian-control rule: only a bounded optional field
with an immediate whitespace continuation may have an empty first value;
required fields, bare empty fields, malformed names, and duplicates still
fail closed.  The v6 root contains two complete Humble receipts and one
19-phase Jazzy partial receipt; Jazzy row 4 was not started.  It remains
diagnostic evidence only and is never reused or promoted.

The sealed `composer-parity-v7` campaign passed that composer grammar and
again completed every Jazzy row-3 command.  It exposed a distinct outer
closure bound: Noble universe's decoded signed Packages index had grown to
73,379,142 bytes, within the established 256 MiB decoded-Packages limit, but
the generic source-tree descriptor still applied its 64 MiB text-file default.
APT source-tree composition and readback now pass the existing 256 MiB decoded
Packages ceiling explicitly.  The generic text limit remains 64 MiB and the
downloaded-deb limit is unchanged.  The v7 root contains two complete Humble
receipts and one 19-phase Jazzy partial receipt; row 4 was not started, and the
root remains diagnostic-only evidence that cannot be reused or promoted.

Ubuntu 24.04's Python emits a fixed `pkg_resources` deprecation warning on
each of the three allowlisted rosdep commands.  That warning contains a
setuptools documentation URL but is not a network attempt.  The prepare
runner removes at most one byte-exact warning prefix only for
`rosdep --version`, `rosdep update`, and `rosdep resolve`, then applies the
normal forbidden-URL scan to every remaining stdout/stderr byte and seals the
original streams.  A changed prefix, an extra URL, a different command, or a
reported runner network attempt remains fail-closed; the container network is
already disconnected before any of these commands run.

The two APT download phases are each followed immediately by an explicit
owner-restore phase: `restore_build_partial_owner` and
`restore_runtime_partial_owner`.  Before container creation the host creates
the corresponding 0700 reference directories and records their device/inode,
owner, mode, and link-count descriptors.  Each restore command uses only its
ordered read-only reference bind and a fixed `chown --reference` argv; its
return code, bounded stdout/stderr, pre/post target descriptors, empty-child
proof, and phase position are sealed.  Missing, reordered, replaced, linked,
metadata-drifting, or writable references, and any post-hoc chmod/chown repair,
are rejected.

APT's sandbox exception is deliberately narrower than a general ownership
waiver.  Before restore, the target is checked with `lstat` only against the
pre-container device/inode/type/mode/link-count descriptor; no child scan is
attempted while the directory is unreadable.  The v2 policy selects exactly
one distro-, profile-, and image-digest-bound `_apt` identity: Humble uses
`uid=100,gid=0`, while the pinned Jazzy image uses `uid=42,gid=0`.  Both are
backed by the captured base-status/passwd evidence; an unknown distro, swapped
UID, profile drift, or image drift is rejected rather than falling back to a
global owner.  The restore command must then produce the original host
owner/mode on the same device/inode/link-count, allow a full no-follow scan,
and prove an empty child set.  Any other owner, identity drift, extra child, or
missing transient-identity binding fails closed.

Phase validation is boundary-aware. At every intermediate receipt boundary,
the observed phase list must be an exact ordered prefix of the fixed 19-phase
plan. Logs, sealed artifact descriptors, and non-empty host output
directories belonging to a future phase are rejected; evidence from a phase
already reached must be present and reopenable before the prefix is accepted.
The terminal validator alone requires the complete artifact set. In
particular, `apt_source_snapshot` is validated without requiring the future
`rosdep_prepare` receipt, while a prepare receipt or rosdep output planted
before that phase is itself a fail-closed error. The JSON schemas and runtime
readers both encode this prefix/order rule, so changing only a self-rehashed
receipt cannot bypass it.

After a separately reviewed closure is materialized, the release runner must
reopen the exact closure before creating a container.  Build and test then use
only the sealed archive/dependency bytes with Docker `--pull=never` and
`--network=none`; any missing, extra, mutable, symlinked, hard-linked,
unpinned, or changed artifact is fail-closed.  Provisioning network use is
recorded separately from disconnected build/test network state.  The present
repository has no captured registration-plugin APT closure, so no formal
campaign is promoted by this contract.

### Capture integration contract (v2)

The capture contract is bound independently into the profile, closure, capture
receipt, and non-promoting candidate manifests.  Its v2 source snapshot keeps
each legacy `sources.list` line and each deb822 stanza as an ordered record,
including every URI, type, suite, component, option, and provenance location.
`Types: deb deb-src` records are retained together while only the binary
Packages projection can satisfy the dependency closure.  Inline `Signed-By`
key material is a separate byte-and-fingerprint binding; the fixed Humble
`ros2.sources` symlink is likewise recorded separately from its resolved target
bytes.  The acceptance chain is signed Release -> Packages index -> downloaded
deb bytes/metadata, with no MD5 locator treated as a trust root.

For an inline `Signed-By` record, the raw ASCII-armored file, normalized armor,
and dearmored binary `.gpg` file are all retained and hash-bound.  Only the
sealed dearmored path may appear in the `gpgv` or isolated `gpg --import
--show-only` command; passing the `.asc`/armor evidence file is invalid even if
its contents look like a key.  Inventory primary and subkey fingerprints must
contain the sole `GOODSIG`/`VALIDSIG` signer (including the `VALIDSIG` primary
fingerprint), and the inline key fingerprint must match that signer.  Ubuntu
path-based keyrings remain binary path records and are not converted by this
inline-only rule.  Cross-repository keyring reuse, `NO_PUBKEY`/`ERRSIG`, key
dearmor drift, and multiple or unexpected signers remain fail-closed.

Signed Release size declarations use the fixed
`release_declaration_policy`: canonical unsigned decimal text, zero allowed,
with a finite maximum of `17179869184` bytes (16 GiB).  This bound applies only
to declarations in a signed Release; the materially smaller profile limits for
downloaded Packages, `.deb`, and archive artifacts remain independent.  The
parser requires canonical ASCII unsigned decimal text, rejects leading zeros,
signs, whitespace, non-ASCII digits, oversized digit strings, and compares the
bounded decimal text before integer conversion.  Raw
`InRelease` or `Release` plus detached `Release.gpg` bytes are sealed before
declaration parsing and are bound by source URL, relative evidence path, byte
count, and SHA-256.  A parse or signature failure therefore leaves
reopenable `REVIEW_REQUIRED` evidence instead of silently dropping the raw
input.

### Signed Packages representation policy

After the Release signature and its SHA-256 stanza have been verified, the
collector selects exactly one Packages representation in this fixed order:
`.xz`, then `.gz`, then plain `Packages`. Only the selected path that is
present in the signed Release is eligible. APT's local `.lz4` cache and
`.bz2`, unsigned, stale, or otherwise unbound cache files are rejected; there
is no external codec or network-capable fallback. The requested URL, final
URL, format, Release-relative path, signed Release SHA-256/size, and the raw
compressed bytes/size/SHA-256 are all retained in `package_evidence`.
When the signed Release declares `Acquire-By-Hash: yes`, a network fetch must
use the component's SHA256 by-hash URL derived from that signed compressed
digest; the mutable direct `Packages.*` path is forbidden. A missing flag or
an exact signed `no` retains direct-path behavior. Duplicate, malformed, or
ambiguous declarations fail closed, preventing mirror synchronization from
mixing a signed Release from one generation with Packages bytes from another.

For `.xz` and `.gz`, the helper uses bounded streaming standard-library
decoders and seals the decoded plain `Packages` bytes separately. Both
compressed and decoded projections must match their distinct signed Release
entries and the source-record URL/path binding. General text and compressed
Packages payloads remain limited to 64 MiB. Only a decoded, signed Packages
artifact may use the dedicated 256 MiB bound; the maximum expansion ratio
remains 256 and decoder EOF is mandatory. Individual deb artifacts retain
their separate 512 MiB bound. The pinned Jazzy/Noble universe currently
decodes to 73,379,142 bytes, so it is valid under the decoded-only bound but
would correctly exceed the general text bound. A self-labelled compressed or
unsigned artifact cannot select the larger limit. The profile-bound XZ
exception permits at most two
streams: the first must carry the index, and an optional second stream must be
exactly the 32-byte valid empty terminal stream emitted by the observed Ubuntu
producer. Per-stream compressed offset, length, SHA-256, and decoded length are
sealed. A data-bearing second stream, a third stream, non-32-byte terminal,
padding, junk, truncation, decompression error, or over-limit expansion fails
closed. Plain `Packages` carries an empty stream-descriptor list and both the
raw and decoded projections with the same Release binding. These limits govern
captured artifacts, not the separate 16-GiB bound for a numeric size declaration
in signed Release metadata.

An explicit signed-empty exception is narrower than the ordinary decoded-index
rule. It is enabled only when the signed Release contains both the selected
compressed entry and the plain `Packages` entry, with the plain entry declaring
zero bytes and SHA-256
`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`.
The compressed bytes must match their signed path, hash, and size and decode to
that exact empty digest. Only one XZ stream of exactly 32 bytes is accepted;
gzip, plain selection, a second stream, padding, trailing bytes, nonempty output,
or any path/repository/Release identity mismatch remains fail-closed. The
evidence records `signed_empty=true` explicitly and binds the policy projection
itself. A zero-length result without this complete signed pair is invalid; all
other decoded Packages artifacts still require at least one byte.

Each Packages record also carries a complete artifact identity, not merely the
decoded content hash.  The identity binds the repository URL, suite,
component, architecture, Release kind/identity/URL/record path/bytes/SHA-256,
the signed compressed and plain relative paths, selected format, Packages URL,
and both compressed and decoded bytes/SHA-256 values.  Its canonical
`registration-plugin-packages-artifact-identity-v1` digest is the key in the
per-capture registry.  Output names use only a bounded 24-hex prefix of that
digest; the registry retains the full digest and rejects a prefix collision
before writing.  Exact full identities are deduplicated once, while each
consumer is represented in an immutable reference map and count projection.
The map's `packages` values are artifact-identity digests rather than
content-only digests, so equal bytes from different suites or repositories
cannot alias.  Partial receipts retain any validated raw evidence and always
emit deterministic maps/counts (empty maps and zero counts when no evidence is
available); `null` is not a partial-evidence placeholder.

### gpgv status contract and partial evidence

The capture contract pins the gpgv status grammar to the reviewed
`registration-plugin-gpgv-status-policy-v1` document for GnuPG 2.2.27. The
proof-bearing sequence is exactly `NEWSIG`, `GOODSIG`, and `VALIDSIG`; the
fingerprint and primary-fingerprint relationship in those records must match
the repository-scoped keyring. `KEY_CONSIDERED <fingerprint> <flags>` and
`VERIFICATION_COMPLIANCE_MODE <mode>` are informational only. They are
accepted only as the documented both-or-neither pair, in the fixed status
order, with canonical decimal fields and the expected fingerprint. They
never replace `GOODSIG`/`VALIDSIG` proof, and duplicate, reordered, unknown,
or extra status records fail closed.

Release bytes and their source-record descriptor are sealed before gpgv
parsing. A gpgv/status failure may retain the already validated Release
prefix together with its diagnostics. If Packages selection, decoding, or
package binding fails, raw files already written for diagnosis are not
promoted into evidence: the inner and outer receipts project only the
independently validated Release records, with empty package/signature
references and deterministic zero package/signature counts. This prevents a
partial receipt from implying a package graph or signer scope that was never
proven. Every such receipt remains independently reopenable,
`REVIEW_REQUIRED`, `benchmark_eligible=false`, and unusable as an offline
runtime closure. The inner helper's self-report is never authority for image,
host, phase, or gpgv policy identity; those bindings are rechecked by the
outer receipt and the closure/audit/summary readers.

The inventory scratch parent is a host-owned bind directory named
`gpg-inventory-work`, created under the fresh capture root before container
creation with mode `0700` and the invoking host uid/gid.  The container may
create one fresh `glim-clean-room-r3-gpg-inventory-<24-hex>` child there and
must remove only that child.  The outer receipt binds the parent device, inode,
owner, mode, and link count before and after the container phase; the parent
must be empty at both observations.  The child's container-visible path is a
logical path and is never compared with host `/tmp` device or inode metadata.
No post-hoc ownership or mode repair is allowed.  Parent replacement,
symlink/hard-link traversal, residue (including sockets), metadata drift, or a
non-empty cleanup fails closed while retaining any already sealed diagnostics.

For the pinned GnuPG 2.2.27 show-only import, the only permitted pre-cleanup
regular metadata names are `pubring.kbx` and `trustdb.gpg`.  Each descriptor
binds type, mode `0600`, uid/gid, single-link count, size (at most 16 MiB),
device/inode, empty xattrs, and SHA-256, together with the exact stderr
creation evidence.  The descriptor is retained after cleanup; only the
matching owned inodes may be removed, followed by fsync and proof that the
child is absent and the host parent is empty.  An empty projection is allowed
only when GnuPG failed before creating metadata.  Unknown names, special
files, locks/sockets, hard links, metadata races, or a missing/tampered
projection are `REVIEW_REQUIRED` failures, never signer or promotion proof.

The host creates the capture bind/output directories before container creation
and records their device, inode, owner, mode, and link-count identity.  The
container may populate only those existing directories; replacement, symlink,
hard-link, ownership, mode, or identity drift is a failure and is never repaired
after the fact.  The exact four capture artifacts and their sidecars are
reopened with no-follow checks before a receipt is sealed.  A complete or
partial capture remains `REVIEW_REQUIRED` and non-benchmark; a separately
signed custodian receipt and later profile reseal are required before any
promotion.  Candidate manifests describe this contract only and cannot switch
the active profile or make a SOTA/publication claim.

The managed directory set includes the nested
`rosdep-prepare/work/rosdistro/files` mount target.  It is created by the host
alongside `work/rosdistro` before Docker creates any container.  The prepare
runner runs with `--precreated` and never calls a directory-creation fallback:
all managed paths must already be regular `0700` directories owned by the
invoking host uid/gid, with no symlink, hard-link, path escape, unexpected
child, or link-count drift.  The pre/post tree snapshots are checked before
the prepare receipt is sealed, including on partial failure.

APT's archive caches use two additional managed, empty bookkeeping
directories: `debs-build/partial` and `debs-runtime/partial`.  Both are
precreated before the container starts so APT cannot create them and change a
parent link count.  They are baseline directories only; their own device,
inode, owner, mode, and link-count identities are still checked before and
after the run, and any file, replacement, or unexpected child remains a
fail-closed output-layout violation.

The executor receipt's `plan.rosdep_prepare_mounts` is the authoritative
ordered eight-entry mount list.  Each entry binds the concrete host `source`,
container `target`, `read_only`, `type=bind`, and `noexec=false` values, plus
`pre`, `post`, and (when Docker inspect completed) `runtime` observations.  The
same receipt plan separately carries the four-file input-bundle identity, the
discovery receipt identity, and the fixed prepare `argv`/script SHA-256.  The
outer validator compares all of these to the static profile mount policy and
reopens every host source with no-follow checks; a production prepare phase
also requires Docker's inspect `Source`/`Destination`/`RW` projection to match
the ordered plan exactly.  Missing, duplicate, reordered, swapped, writable,
symlinked, hard-linked, escaped, or content/metadata-drifted bindings are
fail-closed.  The prepare helper receives only these explicit mounts; a
repository-wide bind or an unbound host path is not an accepted substitute.

### Four-row capture campaign aggregation

The capture command's `campaign` subcommand has its own strict aggregation
receipt. `row_states` is always the fixed Humble/Jazzy absent/present vector;
`rows` contains only children that completed with `PASS_REVIEW_REQUIRED`.
Once a child has started, any failure is represented exactly once by a
separately sealed child partial receipt and a `partial_rows` projection with
relative receipt/sidecar paths, file and canonical SHA-256 values, row
identity, and observed cardinalities. A partial child is never counted as a
completed row or PASS result, including when `rows` is empty. The outer
receipt reopens every referenced pair and rejects missing, extra, duplicate,
reordered, substituted, traversing, symlinked, or unreferenced child entries.
Both complete and partial campaign receipts stay `REVIEW_REQUIRED` with
`benchmark_eligible=false`, `claim_eligible=false`,
`active_profile_switch=false`, and `promotion_allowed=false`; a separate
signed review/acceptance receipt remains mandatory.
