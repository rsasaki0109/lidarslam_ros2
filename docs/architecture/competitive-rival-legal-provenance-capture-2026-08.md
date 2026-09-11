# Competitive rival legal provenance capture (2026-08)

This document describes an additive, non-promoting review packet for the
`rival_source_closure_not_ready_legal_provenance` blocker.  It does not modify
the r2 selection, the active profile, the historical execution receipt, or the
r3 candidate.  The checked-in status remains `NOT_READY`; no benchmark or
claim may consume this packet.

## What is fixed, and what is not proven

The current r2 closure fixes exact commit-addressed upstream identities and
local recipe bindings.  The relevant primary upstream references are:

| component | pinned revision | canonical primary source | observed provenance status |
|---|---|---|---|
| GLIM core | `faa264a1bce1bda406f73457e35511f56cdc2eaa` | [koide3/glim commit](https://github.com/koide3/glim/commit/faa264a1bce1bda406f73457e35511f56cdc2eaa) | MIT license artifact is present in the r2 audit; component is the only legal-ready core path |
| GLIM ROS2 | `4a9e7a4cb084967c8525a1be529ad3ba2a118ae7` | [koide3/glim_ros2 commit](https://github.com/koide3/glim_ros2/commit/4a9e7a4cb084967c8525a1be529ad3ba2a118ae7) | `package.xml` declares MIT, but the pinned tree has no license text artifact; metadata is not a license artifact |
| FAST-LIVO2 | `0d2c0346107b75b59934975adec9a6eeeb913c64` | [hku-mars/FAST-LIVO2 commit](https://github.com/hku-mars/FAST-LIVO2/commit/0d2c0346107b75b59934975adec9a6eeeb913c64) | root `LICENSE` is GPL-2.0 while `package.xml` declares BSD; unresolved authoritative conflict |
| rpg_vikit | `6c886c8e5d83997806e00294826d528cea3581dd` | [xuankuzcr/rpg_vikit commit](https://github.com/xuankuzcr/rpg_vikit/commit/6c886c8e5d83997806e00294826d528cea3581dd) | mixed component declarations and missing license text artifact |
| Sophus | `a621ff2e56c56c839a6c40418d42c3c254424b5c` | [strasdat/Sophus commit](https://github.com/strasdat/Sophus/commit/a621ff2e56c56c839a6c40418d42c3c254424b5c) | no license artifact at the pinned legacy tree |

`ours` is the repository's baseline, not an entry in the r2 rival source
closure.  Its phase-v2 execution contract, image/toolchain fields, and
`scripts/run_rko_lio_graph_benchmark.sh` runner are bound by the complete
profile SHA captured below (the declared/current runner SHA is
`d82df94521189d97fabd0b8f672b2b61a239a417f7e3220ddb4ab12a65ef450f`).  This
packet deliberately does not invent an upstream revision or a separate
license row for the baseline.  A future claim receipt still has to bind the
baseline image, recipe, runner, and per-run execution receipt explicitly; a
profile SHA alone is not execution evidence.

These are evidence classifications, not legal conclusions.  The GLIM ROS2
metadata claim can be removed from the active recipe only by the separate
clean-room adapter track; it cannot be silently converted into redistribution
permission.  FAST-LIVO2's BSD/GPL conflict and the rpg_vikit/Sophus missing or
mixed artifacts require upstream/custodian review or an approved replacement;
this packet does not choose a replacement rival.

The current local r2 recipe also has reproducibility drift.  The declared and
current runner hashes are:

| runner | r2 declared SHA-256 | current bytes SHA-256 | result |
|---|---|---|---|
| `scripts/run_glim_benchmark.py` | `2345b930ce8b679d92d315bae8ed372d99fde1c43fb744f5d8b7c7fc77588794` | `1cb5a1e7354c168ae2317ff96e9450fb9e7e93b1314db8690bc63177e04913a1` | drift; r2 checker fails closed |
| `scripts/run_fast_livo2_benchmark.py` | `b8a57115aba296e31ee53e380052d5a35eb8df4fbb72e8797ae65bab694dbd30` | `eda2373a1f04098851ffc48ccb0b1d8ba282b8b84c2a3f8102e18e3b45284352` | drift; r2 checker fails closed |

The historical selection is a recipe-selection revision, not an execution
receipt.  Passing it to the execution-receipt checker therefore remains
invalid and does not repair the missing per-run evidence.

## Capture contract

`scripts/capture_competitive_rival_legal_provenance.py` reopens the current
profile, closure identity, selection bytes, checker source, all local recipe
artifacts, exact commit/archive/license URLs, and source declarations.  It
uses no network call.  Every future upstream request is represented as an
explicit `NOT_RUN` observation with an exact HTTPS URL, expected hash (when
known), zero redirects, no final URL, no body, and no HTTP status.  If a
response is supplied to the pure validator, it must be HTTP 200, have zero
redirects, match the expected SHA-256, stay within 2 MiB, and contain only the
bounded response-header allowlist (`content-length`, `content-type`, `etag`,
`last-modified`).  Redirects, non-200 responses, unknown headers, control
characters, and oversized bodies fail closed.

The output is a fresh two-file pair:

```text
legal_provenance_capture.json
legal_provenance_capture.json.sha256
```

Both files are created exclusively, fsynced, mode `0444`, regular and
single-link.  The validator performs no-follow, bounded, before/after inode
and size checks, rejects symlink/hardlink/path traversal/collision/extra-file
conditions, and reopens the profile and recipe bytes before accepting the
packet.  A failed pair write rolls back only the inodes owned by that write.

Recipe descriptors with
`path_kind: pinned_upstream_archive_relative_path` are handled separately
from local checkout files.  The capture requires an exact one-to-one match
against the source's `archive_artifacts` entry (path, role, hash kind, SHA,
status, source revision, archive URL, and archive SHA), records
`PINNED_UPSTREAM_ARCHIVE_BOUND` and `NOT_LOCALLY_OBSERVED`, and never reads a
same-named host path.  Unknown path kinds or a missing, duplicate, or
mismatched archive entry fail closed.  Therefore the current FAST-LIVO2
`config/HILTI22.yaml` descriptor is not reported as a local missing file; its
recipe remains `DRIFT_OR_MISSING` only because the separately bound runner
bytes drift from the r2 declaration.

The schema is
`configs/slam_benchmark_profiles/competitive_rival_legal_provenance_capture_v1.schema.json`.
Regardless of local recipe matches, the packet is always:

```text
status: REVIEW_REQUIRED
review_status: UNSIGNED_REVIEW_REQUIRED
benchmark_eligible: false
claim_eligible: false
remote_policy.responses_status: NOT_RUN
```

No response packet is treated as an upstream license decision.  An external
custodian must independently review the exact pinned bytes and produce a
separate signed artifact before any future promotion workflow can be
considered.  That external review is intentionally not implemented here.

## Verification record

The offline checker was run against the current worktree and returned
`INVALID`, not `PASS`: both runner hashes above drift, and the legal rows remain
not ready.  The capture utility and synthetic adversarial tests are the only
new execution in this milestone; no Docker, image pull, network fetch,
benchmark, bag, GT, or scorer was run.  Existing r2/profile/candidate bytes
remain untouched.
