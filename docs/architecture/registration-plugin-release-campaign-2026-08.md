# Registration-plugin release campaign orchestration

The additive `run_registration_plugin_release_campaign.py` host orchestrator
wraps the existing per-leg launcher and matrix summarizer. It is a fixed
four-row state machine in this exact order:

1. Humble / absent
2. Humble / present
3. Jazzy / absent
4. Jazzy / present

The order and campaign identity are taken only from the current profile's
sealed `campaign_set`; the command line has no row-selection or row-override
option. Every row receives a fresh child directory named by the authoritative
leg launcher, and its host receipt, inner receipt, profile hash, source
manifest, campaign set, container name, cleanup proof, and safety fields are
reopened before the row can be marked `PASS`.

The preflight revalidates the profile, exact campaign rows, current source
manifest, Git dirty-tree snapshot, fresh campaign/child paths, image digest
contracts, and container-name absence. Runtime mode additionally requires the
profile-bound evidence mount and read-only image/container probes. A source,
campaign, storage, image, or collision identity change before a later row
aborts the campaign without starting that row. A failed row is not retried;
the campaign seals a partial failure and marks all later rows
`NOT_STARTED`. The matrix summarizer is invoked only after all four host and
inner receipts pass. Otherwise the campaign receipt records
`summary.status=NOT_RUN`.

When a row has been entered but cannot produce its normal host/inner receipt,
the child is still referenced by one ordered `partial_rows` entry. The entry
binds the fixed row identity, root-relative child-receipt and sidecar paths,
file bytes/SHA-256, canonical receipt hash, failure status, and the campaign,
profile, source-manifest, Git, and four-row cardinality identities carried by
the sealed `registration-plugin-release-child-partial-v1` receipt. It is a
diagnostic failure artifact, never a completed row: it is not appended to the
normal `rows`, does not count toward `PASS`, and cannot trigger summarization
or promotion. Preflight and successful campaigns carry an empty `partial_rows`
list with count zero. Runtime validation requires the exact sorted set of
started failed rows, rejects missing, extra, duplicate, reordered,
path/hash/status/cardinality or row-double-membership projections, and reopens
every child receipt and sidecar. An existing partial receipt under any
expected or unexpected child which is absent from the projection is an
unreferenced started child and fails closed, including a forged report with an
empty normal-row projection.

Docker/build safety is recorded as a phase state, not inferred from a row's
result: each phase is `NOT_STARTED`, `ATTEMPTED`, or `CONFIRMED`. `ATTEMPTED`
is the conservative state when the launcher was entered but an authoritative
host receipt cannot prove whether the phase started; `CONFIRMED` requires the
reopened host receipt (`container.start_count=1` for Docker, and a PASS inner
runner binding for build/test). A failed leg therefore cannot be reported as
`NOT_STARTED` merely because it did not produce a PASS result.

The official ROS images contain one narrowly permitted apt-source exception:
`/etc/apt/sources.list.d/ros2.sources` may be a symlink only when its target is
the exact absolute path `/usr/share/ros-apt-source/ros2.sources`. Capture
records the symlink lstat descriptor separately from the resolved target file
descriptor. The target is opened with no-follow semantics after every parent
component is checked, and its root ownership, mode `0644`, single link, size,
descriptor identity, and two-pass SHA-256 are checked before and after the
read. The source link itself must be root-owned, mode `0777`, and single-link
in the production container. Relative, alternate, looping, parent-symlinked,
hard-linked, mutable, or otherwise unexpected source entries remain rejected;
ordinary source files (including Jazzy's `ubuntu.sources`) retain the regular
single-link contract. A synthetic filesystem-root injection exists only for
unit fixtures and is not a production identity proof.

The source snapshot also records isolated GnuPG key inventory evidence. The
inventory uses the pinned `/usr/bin/gpg` 2.2.27 contract with an exclusive,
fresh 0700 child under the host-owned `gpg-inventory-work` bind directory,
`--no-default-keyring`, disabled automatic key acquisition/import/location,
fixed locale, no shell, and no network. The host binds the parent
device/inode/uid/gid/mode/nlink before and after the container; the child is
created and removed only within that parent, and its container `/tmp` identity
is never compared with a host inode. The exact argv, version output, keyring
bytes, stdout/stderr, status, homedir identity, cleanup, and post-absence are
reopened before acceptance. A
`VERIFIED` inventory is required before signer scope is claimed. A
`GPG_KEY_INVENTORY_FAILED` descriptor may preserve gpgv and Release evidence
for diagnosis, but remains `REVIEW_REQUIRED`, does not assert a signer, and
cannot feed runtime or promotion. Inventory fields are repository-scoped; a
keyring or result from another source is not a fallback.

Once the runtime root is reserved, every later failure—including a source or
storage drift before row one, a probe failure, a summary failure, or a receipt
sealing failure—takes the fail-closed path. The owned root receives a
best-effort immutable campaign receipt with all four ordered row states and
the failure phase; rows after the first failed/blocked row are `NOT_STARTED`,
and the summary is never promoted from a partial campaign. The strict schema
and the independent runtime validator both reject extra fields, reordered or
duplicate rows, mixed preflight/runtime shapes, forged safety states, reversed
timestamps, and a summary attached to a partial campaign.

`--preflight-only` never invokes Docker and writes only one fresh immutable
preflight receipt below `/tmp` or the profile-bound evidence mount. It reports
`PREFLIGHT_NOT_RUNTIME_VALIDATED` when live image/storage probes were not
injected. Runtime execution is intentionally not claimed here; no Docker,
build, bag, GT, scorer, map, or formal replay was run while this contract was
added.

Campaign and preflight receipts use the additive
`registration-plugin-release-campaign-v1` schema and a canonical self-hash
plus `.sha256` sidecar. All receipt and sidecar writes are fresh, exclusive,
mode `0444`, single-link files. Existing leg evidence is never overwritten or
deleted. The checked-in SOTA/README and active benchmark claims are unchanged.
