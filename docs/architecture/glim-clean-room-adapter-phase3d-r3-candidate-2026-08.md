# GLIM clean-room Phase 3d r3 candidate

This document describes an additive, opt-in candidate only.  It does not
change the active r2 closure, selection, profile, receipts, or required rival
set.  No r3 selection is issued, no benchmark evidence is promoted, and no
README claim is made.

## Boundary

`docker/benchmark_adapters/glim_clean_room/phase3d/r3/` supplies a separate
Jazzy recipe, entrypoint, and ROS 2 output collector.  The recipe consumes
only a host-prefetched, independently verified copy of the exact GLIM core,
GTSAM, gtsam_points, and Boost archives already recorded in the Phase 1
source closure.  It contains no upstream ROS transport source, patch,
include, link target, or symbol.  The existing Phase 3d node remains the
host-owned transport boundary and is copied into the candidate image; the
collector consumes only its `nav_msgs/Path`, map `PointCloud2`, and
post-publication diagnostic.

The candidate image is based on the same digest-pinned Jazzy base as the
retained r2 recipe:

```text
ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f
```

The candidate manifest binds the Phase 1 core-closure SHA, the r2 profile and
selection identities, the selected profile phase, and the existing closure
identity.  At execution time the host must reopen the profile, selection,
input manifest, input tree, calibration tree, and config tree.  Paths and
recorded tree hashes are compared exactly; stale, substituted, symlinked,
hard-linked, or missing inputs fail closed.  The selected profile's legacy
source metadata is not consumed by this path, so the candidate cannot
silently inherit bridge provenance from the old runner.

## Offline dependency and build boundary

`offline_dependency_manifest.json` is the additive source/dependency closure.
Each archive has an official immutable HTTPS provenance URL, exact byte count
and SHA-256, pinned commit where applicable, extracted-tree identity, and
license-file identities.  URLs are metadata for the host acquisition phase;
the Dockerfile never opens them.  The exact pinned base image is also bound.

The base image's apt/deb closure is intentionally
`NOT_READY_APT_CLOSURE_UNSEALED`.  It has no invented package versions or
hashes: promotion requires a capture inside the exact digest-pinned image of
the exact platform/ROS distro, a non-empty canonically sorted unique dpkg
set, non-empty repositories and apt source files, Release/InRelease URL/SHA
records, rosdep source/cache hashes, every `.deb` filename/URL/size/SHA and
license artifact identity, and a zero-exit install command receipt.  The
manifest, copied `apt-deb-closure.json`, and prefetch receipt must agree on
canonical archive/package/repository-set identities and the closure identity;
empty, reordered, duplicate, mixed, or self-rehashed records fail closed.
Until that reviewed closure exists, the prefetch tool fails closed with
`APT_CLOSURE_NOT_READY`; this candidate cannot produce a build-ready receipt.

The additive `apt_closure_capture.py`,
`apt_closure_outer.py`, and
`scripts/plan_glim_clean_room_r3_apt_capture.py` define the existing capture
boundary.  The capture tool is explicitly `CONTRACT_ONLY`: it validates/saves
synthetic or externally collected bytes, but its `contract-only` command
returns `CONTRACT_ONLY_NOT_RUN` (exit 78) and cannot claim that apt data was
generated.  A separate filesystem-only collector candidate now exists at
`apt_closure_collector.py`; it is explicitly
`IMPLEMENTATION_CANDIDATE_NOT_RUNTIME_VALIDATED`, has not run in a real image,
and cannot promote the production closure.  It imports the capture verifier
from its fixed, read-only mounted tool path in the future image; no sibling
module is assumed to be present by an ambient working directory.

The collector reads a fixed container-root layout, a host-reviewed URI ledger,
and only the six fixed mounts: the writable output root, collector and capture
tools/schemas, package allowlist, and URI ledger.  It receives no plan,
Docker-inspect record, staged-deb directory, base-status file, or
`container-metadata.json`; image/container/network identity is an outer-host
receipt concern.  Before installation, the workflow snapshots the base dpkg
status into `OUTPUT_ROOT/base-dpkg-status.snapshot`.  After `--download-only`,
it copies every expected `.deb` into `OUTPUT_ROOT/staged-debs/` and seals the
file/byte/SHA manifest before installation.  The collector consumes those
workflow-generated paths from the same output mount and does not read
`/var/cache/apt/archives`, so a later apt cleanup hook cannot erase or change
the evidence.  It requires the exact newly acquired package set to differ from
the base dpkg snapshot, validates every staged `.deb` byte and fixed
`dpkg-deb -f Package Version Architecture` result, and rejects base/new
ambiguity.  It also reopens apt source files, Release/InRelease bytes, rosdep
source/cache bytes, and every package copyright file.  URI/repository/Release
spoofing, size/SHA/control mismatch, missing or extra packages/debs,
symlink/hardlink/special files, pre-created workflow outputs, and
partial/reused output fail closed.

`seal_to_capture` reopens the inner filesystem payload, rejects any host
identity fields, and combines it with image/argv/mount/plan identity reopened
from the outer host logs before calling the existing
`apt_closure_capture.seal_capture` and `apt_closure_outer.verify_outer_root`
contracts.  The candidate workflow is recorded in
`apt_closure_collector_candidate.json`:

1. inspect the exact pinned image and prove a fresh output root;
2. run `apt-get update` and copy the host-reviewed URI ledger (copy-only; it
   is not generated by the collector);
3. snapshot the pre-install dpkg status;
4. run exact-version `--download-only`, copy/hash the debs into fresh staging,
   then install the exact allowlist;
5. run the filesystem-only collector with expected exit status zero using the
   ledger/allowlist read-only mounts and the snapshot/staging bytes generated
   earlier under the writable output mount;
6. disconnect, inspect, stop, remove, and verify absence.

Provisioning is the only network phase; collector execution is filesystem-only.
No network, apt, Docker, benchmark, bag, GT, or scorer execution has been
performed for this candidate.

The planner accepts only the exact pinned image, immutable allowlist, and
ledger.  It rejects arbitrary provisioning/capture argv or shell fragments and
emits a fixed, hashed sequence with exactly six mounts.  All read-only mount
sources must exist and be single-link regular files; the output source is the
sole fresh absent placeholder and is created by the fixed workflow phase:

1. host `docker image inspect` and fresh output-root absence check;
2. fixed host `mkdir --mode 0700` followed by `stat` lstat/owner/mode
   evidence, then the exact pre-existing-name absence check;
3. fixed `docker run` with the host-created evidence directory read-write and
   collector, capture verifier/schema, allowlist, and URI ledger inputs
   read-only;
4. fixed `docker exec apt-get update`, URI-ledger copy, base-status snapshot,
   exact-version `--download-only`, staged-deb copy/hash, install, then the
   collector argv;
5. `docker network disconnect`, post-disconnect inspect, stop, remove, and
   exact post-remove absence check.

The planner only describes this transition; it does not create the output or
generated snapshot/staging paths.
The future host executor owns the output-directory `mkdir`/`stat` phases.
`seal_outer_receipt` creates the separate fresh outer log root with mode 0700
and records its own lstat/owner/mode; neither root is implicitly created by
Docker or by an inner payload.

`apt_closure_outer.py` is the host-owned verifier.  It reopens bounded,
single-link log files, parses the image and post-disconnect inspect bytes from
those logs (never from the inner payload), checks command order/argv/exit
status, exact mounts, network absence, cleanup, allowlist bytes, planner/tool
hashes, and sidecars.  If an inner capture root is supplied, it is revalidated
against the host-derived image identity and plan.  The outer and inner sealed
results remain `REVIEW_REQUIRED`, `CONTRACT_ONLY`, and
`benchmark_eligible=false`; a separate proposal writer can emit a
`PROPOSED_REVIEW_REQUIRED` candidate READY manifest, but only with an unsigned
custodian placeholder and never by editing this production manifest.

The machine-readable `apt_closure_collector_plan.schema.json` describes the
outer-only six-mount plan.  `staged_debs.schema.json` describes the sealed
pre-install staging manifest.  The outer verifier reopens every regular-file
mount hash and exact `docker run` mount argv; it rejects any inner plan or
self-declared host authority.  A missing ledger, capture verifier, allowlist,
or any workflow-generated snapshot/staging output is a fail-closed
precondition.

`dependency_prefetch.py` has only `prefetch` (local copy) and `verify`
(reopen-only) operations.  It rejects an existing output root, symlinks,
hardlinks, non-regular files, missing/extra names, mixed manifests, byte/SHA
drift, receipt drift, and any network claim.  A successful receipt is
`glim_clean_room_r3_prefetch_receipt_v1` with `network_used=false` and an
exact file projection.  No package manager or URL fetch is performed by the
tool.

The replacement r3 `Dockerfile` uses `COPY prefetch/` only.  It contains no
URL `ADD`, apt/rosdep command, curl, or wget.  It verifies the same prefetch
receipt in both build and runtime stages, installs only reviewed local `.deb`
bytes, verifies each archive and source-license identity, and builds with
Release, two jobs, and `-march-native` disabled.  The non-executing planner
requires the exact command below and binds candidate-manifest, dependency
manifest, recipe, verifier, planner, prefetch-receipt, and argv hashes:

```text
docker build --network none --pull=false --progress=plain --build-arg JOBS=2 -f <context>/phase3d/r3/Dockerfile -t <reviewed-tag> <context>
```

Any networked command, extra privilege/flag, missing or mixed prefetch set, or
candidate identity mismatch is rejected before Docker is called.  No Docker
build or prefetch was executed for this candidate.

## Execution and evidence contract

`scripts/run_glim_clean_room_r3_candidate.py` is a dry planner unless
`--execute` is explicit.  Its execution command is digest-pinned,
`--pull`-free, network-disabled, read-only-root, capability-dropped, and
uses exactly four bind mounts: input, calibration, config (read-only), and a
fresh output root (read-write).  It never invokes a scorer or opens ground
truth.  The output directory cannot pre-exist.

The container collector follows this sequence:

1. start the installed host-owned Phase 3d node and replay the frozen input;
2. wait for replay completion and call the single finalize service;
3. require a trajectory and map plus the node's completion diagnostic, where
   the diagnostic is emitted after both internal publication calls;
4. atomically write `trajectory.json`, `map.json`, and `resource.json`, or
   atomically write one terminal `failure.json` and no success artifact;
5. let the host reopen every artifact, check frames/order/finite values,
   resource identity, output tree hash, and the canonical per-attempt receipt.

The receipt uses the repository's canonical execution-attempt contract and an
external `attempt.index.json` for the actual pretty-file SHA.  This preserves
the separation between canonical receipt identity and file bytes.  Failed
attempts remain sealed for failure publication but are never success or claim
eligible.  All candidate receipts carry an explicit GT-blind proof and
`benchmark_eligible: false`.

## Static evidence and current blockers

The offline validator and thirty synthetic adversarial tests pass:

```text
PYTHONDONTWRITEBYTECODE=1 python3 scripts/validate_glim_clean_room_r3_candidate.py --json
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  graph_based_slam/test/test_glim_clean_room_phase3d_r3_candidate.py \
  graph_based_slam/test/test_glim_clean_room_r3_offline_contract.py \
  graph_based_slam/test/test_glim_clean_room_r3_apt_capture.py \
  graph_based_slam/test/test_glim_clean_room_r3_apt_collector.py
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  graph_based_slam/test/test_glim_clean_room_r3_apt_collector.py
```

The first command reports `docker_build=NOT_RUN`,
`benchmark_execution=FORBIDDEN`, and `active_r2_modified=false`.  The second
reports the prior candidate tests plus the collector-focused tests; the
collector-focused command reports `7 passed`.  The
existing clean-room source guard also passes for the
expanded Phase 3d tree.  No Docker build, bag replay, dataset, GT, scorer, or
network run was performed for this candidate.

The candidate remains `OPT_IN_NOT_READY` for independent reasons:

- the image apt/deb package closure is still `NOT_READY_APT_CLOSURE_UNSEALED`
  and needs a sealed dependency manifest plus independent revalidation on the
  pinned base;
- the recipe has not had an image build, install audit, or ROS graph smoke;
- frozen input/calibration bytes are unavailable while the evidence volume is
  unavailable, so no real input identity is revalidated;
- the r2 legal/source closure remains authoritative and unchanged.  The
  clean-room core path removes the bridge component as a future Category C
  resolution candidate, but it does not resolve the separate FAST-LIVO2,
  rpg_vikit, or legacy Sophus provenance ambiguities;
- no candidate artifact may enter the active suite until a separately
  reviewed r3 closure/selection and complete required-system-by-sequence-by-
  repetition evidence set exist.

The recipe, manifest, and validator are intentionally additive.  A future
promotion task must issue a new closure/selection revision rather than
rewriting historical r2 receipts.
