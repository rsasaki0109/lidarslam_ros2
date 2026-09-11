# Registration-plugin Humble/Jazzy matrix audit (2026-08)

This is an additive, current-source audit for the external registration
plugin/SDK path.  It does not promote a benchmark, SOTA, replay, map, ground
truth, or scorer result.  The read-only runner and pinned profile are:

- `scripts/audit_registration_plugin_matrix.py`
- `configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json`

The profile binds the current transaction, provenance, loader, scanmatcher,
backend, DSO-gate, and C++14 external-consumer sources by content SHA-256.
Every report recomputes those hashes before inspecting a distro.  A mismatch
is `FAIL_CLOSED`; a historical result is never used to repair it.

## Evidence disposition

The Phase 2 prose claims in
[`registration-plugin-api.md`](registration-plugin-api.md) around the shell
loader and external author template do not name an immutable receipt and
sidecar.  They are therefore reported as
`SUPERSEDED_UNBOUND_DOCUMENT_CLAIM`, not as current Humble/Jazzy evidence.
The historical Small GICP ODR paths under `/tmp` are absent (and have no
verifiable sidecars), so those entries are
`SUPERSEDED_ARTIFACT_OR_SIDECAR_INVALID`.  Even a surviving historical file
would remain superseded unless it binds the current source manifest.

The current profile records the fully qualified, digest-bound official Humble
image reference.  Its OCI index is
`sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988`,
and the verified Linux/amd64 child manifest is
`sha256:32bf718e63618482ffb1fe232cf0f834635c57162e2506fb0bc0b092ef776c1e`.
The one-image acquisition receipt is
`/media/sasaki/aiueo1/benchmarks/plugin_evidence_20260824/registration_plugin_humble_exact_acquisition_20260824T060340Z_agent/humble_exact_acquisition.receipt.json`
(SHA-256
`7d4568867892dfcbfe29b46dc02b976ef7662ac1355ea5d3577d2b6be076bbe6`).
This proves image identity and local pullability only; it is not a release
leg or runtime PASS.  The local install prefix is a symlink overlay rather
than an independent install tree, so it is not counted as an installed-prefix
proof.

## Local matrix result

The audit was run read-only on the current worktree with the Jazzy host
prefix.  The source manifest and static C++14 consumer contract were
`PASS`; the distro result was `PASS_HOST_TOOLCHAIN_ONLY`.  This means the
prefix/package/toolchain identity was inspected and no container or build was
executed.  Optional `fast_gicp` and `small_gicp` were explicitly recorded as
`ABSENT` on this host; they are optional and are not silently treated as
installed.

The local host has no `/opt/ros/humble` setup prefix.  Humble is therefore
`NO_GO:humble` for this environment, with no network or image pull fallback.
The profile's historical Humble digest is not claimed as a local proof.

The external consumer contract is C++14.  The ROS/pluginlib loader and the
scanmatcher production target retain their declared standards and are not
relabelled as C++14.  This audit's consumer check is static; a clean,
current-source external build/load receipt still requires a distro job with
its dependencies available.

## CI gate and scope

The default Humble/Jazzy workflow invokes the runner after checkout and
uploads each immutable JSON report.  The job may pass a distro only when its
current source manifest and required ROS package metadata pass.  It does not
run Docker commands through this audit; any optional image inspection is
explicitly read-only and digest-pinned.

The matrix does not assert that optional adapters are present, does not claim
Humble availability on a host without Humble, and does not claim independent
DSO ODR/load coverage from static markers.  A future distro-specific gate must
produce a fresh receipt with the source/install tree, image/toolchain,
read-only repository mount, C++14 consumer build/load, ELF/ODR properties, and
optional dependency state before promotion.

The source-checkout workflow explicitly exports the checkout root on
`PYTHONPATH` for the release runner and four-leg summary, while the container
launcher binds both `/workspace/src` and its `scripts/` surface before invoking
the runner.  This makes the canonical `lidarslam_benchmark_tools` package
surface deterministic; it is execution plumbing only and is not installed-
prefix or runtime proof.

Safety for this audit is fixed to network/build/pull/run/bag/GT/scorer/map/
formal all false.

## Current-source release matrix (static pin stage; runtime promotion pending)

The release-facing matrix is now described by the additive
`release_matrix` section in
`configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json`
and executed by:

- `scripts/run_registration_plugin_release_matrix.py`
- `scripts/summarize_registration_plugin_release_matrix.py`

It has four required legs: Humble/Jazzy × optional-dependency `absent`/`present`.
The absent leg builds a clean, isolated non-symlink install and must prove that
FAST_GICP/SMALL_GICP selectors and plugin classes are not advertised; selecting
one fails closed.  It also runs the C++14 consumer and author-template checks,
the real external DSO ELF/DT_NEEDED/RPATH/ODR/load-session gate, provenance and
transaction rollback tests, and the real ROS resource-init failure test.  Each
leg binds the current source manifest, dirty-tree identity, installed-tree
manifest, command hashes, toolchain/container digest, and immutable receipt
sidecar.  No benchmark input, GT, scorer, map, or formal replay is part of this
matrix.

Before a `PASS` receipt is eligible for the matrix summary, the summary gate
reopens the fresh per-leg evidence root and validates its
`registration-plugin-artifact-manifest-v1`: every declared relative path is
canonical, non-symlinked, regular, single-link, and has the recorded byte size
and SHA-256.  Required command logs, install/test trees, DSO, provenance, and
resource-failure receipts must be present with exact two-token sidecars; missing,
extra, swapped-root, traversal, or changed-byte artifacts fail closed.  CI
uploads each leg under its unique artifact name without merging same-basename
files.  This is an integrity contract only; runtime promotion remains pending.

### Dependency-environment closure

Each release leg now has one additional sealed, root-relative artifact:
`dependency_environment.receipt.json` and its mode-0444 two-token sidecar.
The host launcher runs
`capture_registration_plugin_dependency_environment.py` immediately after
the apt/rosdep install command and before disconnecting the container network.
The capture records only deterministic identities: the pinned image digest and
platform, ROS distro, sorted `dpkg` name/version/architecture/status rows,
apt source configuration and Release/InRelease index file identities,
rosdep source/cache identities, the exact install-command and capture-script
hashes, and an explicit empty secret-value policy. Credential, proxy, token,
and environment values are never recorded.

The host runner and summary reopen both files without following symlinks,
require regular single-link mode-0444 files, enforce a finite 16 MiB manifest
and 256-byte sidecar bound before hashing or JSON parsing, recheck the sidecar
and canonical projection hash, and bind the receipt and sidecar into the
artifact manifest.
Missing, malformed, changed, or cross-root bytes fail closed. The summary
compares the canonical base-dependency projection between the `absent` and
`present` legs separately for each distro; it deliberately does not require
Humble and Jazzy package closures to be equal, and excludes optional archive
and build outputs from this comparison. A dependency receipt is therefore a
receipt-level identity contract, not a claim that a runtime leg has passed.

The synthetic capture, launcher, and matrix tests cover canonical projection,
exclusive sealing, symlink/non-zero-install rejection, capture ordering and
secret absence, missing/tampered receipts, and absent/present closure drift.
The current profile remains runtime-pending and is not promoted by these
static checks.

Resource policy is intentionally split.  FAST benchmark/formal performance
runs retain the strict quiescence contract (CPU busy `<=5%`, load and forbidden
process checks).  The registration-plugin functional leg may tolerate unrelated
CPU load only when no compiler/colcon/Docker-build process is observed, Docker
is idle (or unavailable because the leg is already isolated), memory and disk
headroom pass, and every build command is constrained to one worker with
`nice -n 19`/`ionice -c 3` when available.  Its snapshots are retained in the
receipt, but all timing/resource fields are
`NON_AUTHORITATIVE_CONTAMINATED` and cannot satisfy a performance or SOTA gate.

The present leg is now `READY_PINNED_STATIC_REVIEW` for both distros.  This is
not a runtime PASS: the profile records immutable official archive, source
tree, and license pins, while the distro build/load receipts remain pending.
The runner verifies the archive SHA before extraction and rejects any
network-capable command after the explicit fetch.  The two current pins are:

| dependency | official revision | immutable archive SHA-256 | source-tree SHA-256 | license |
|---|---|---|---|---|
| `fast_gicp` | `0e7ec1441c99f7be453db2ea216d5de029387417` | `b18c904bbd47c8653a9df4c25aec4bf7cf19522338d63a3d155a20a46a088861` | `ad4515b3f453f9b58ac20a037b19a28c6becb5a1c0d09be5fe0a2536a7b1de28` | BSD-3-Clause (`LICENSE` SHA `62091f5ee5b6cf36ea37faae5db95b8679c14e1b071911af0be7abc72c461c9c`) |
| `small_gicp` | `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e` (v1.0.1) | `74f29b78050d1b9c10a88f56a5760e0c82f4aceebcfd324529d205e9294c6558` | `d29a34b3508b3219b44418506fba0a00f5a2a1e0c71dbdb0181d3432be98307e` | MIT (`LICENSE` SHA `c1a3ab7ff9bf54e320aa78c653acca70f0b4f955d9d129c91a70866059e8d8d0`) |

The official repositories are `https://github.com/SMRT-AIST/fast_gicp` and
`https://github.com/koide3/small_gicp`; archive URLs in the profile are
commit-addressed `codeload.github.com` URLs, never a branch or `latest` URL.
The fast_gicp CMake branch for `ROS_VERSION=2` discovers ament_cmake and
requires PCL/Eigen; small_gicp is ROS-independent, requires Eigen, and uses
C++17 with optional PCL/TBB features disabled.  Those are compatibility
rationales from the pinned upstream CMake files, not Humble/Jazzy runtime
evidence.  A release promotion still requires clean isolated builds, external
C++14 consumer/template checks, DSO/ODR/load smoke, and rollback/resource-init
receipts for all four legs.

The older Phase 2 prose and `/tmp` Small GICP receipts remain historical and
superseded; they do not satisfy this current-source release matrix.  Until all
four receipts are `PASS`, the external-plugin release status remains pending
and the built-in default behavior is unchanged.

### Two-phase dependency and network contract

Dependency installation and official archive acquisition are provisioning
operations.  While the container network is available, apt/rosdep closure is
captured in `dependency_environment.receipt.json`, and the `present` leg
fetches each profile-pinned official archive exactly once through
`registration_plugin_dependency_prefetch.py`.  The host reopens every archive,
sidecar, manifest, redirect/final-URL, size, and SHA-256 identity before it
disconnects the container.  The `absent` leg declares prefetch
`NOT_APPLICABLE` and has no archive allowlist.

The machine-readable receipt/manifest projection is defined in
`configs/slam_benchmark_profiles/registration_plugin_dependency_prefetch_v1.schema.json`;
the runtime validator is stricter than JSON Schema and performs the no-follow
filesystem and profile-pin checks before any archive extraction.

After the host proves that the container has no network attachments, the build
and test runner accepts only the exact dependency receipt plus (for `present`)
the prefetch receipt, manifest, sidecars, and archive allowlist.  It copies
those bytes into its isolated work tree and never invokes curl, wget, git,
rosdep, urllib, or another fetch API.  Missing, extra, duplicate, symlinked,
hard-linked, traversing, oversized, or mutated preseed artifacts fail closed;
the root is never retried or overwritten.

Receipts use truthful phase fields rather than the ambiguous historical
`network_used` flag: `provisioning_network_used` records apt/rosdep,
`archive_fetch_network_used` records only pinned archive prefetch, and
`build_test_network_connected`/`build_test_network_used` describe the build
and test phase.  `HOST_PROMOTION` receipts require disconnected and unused
build/test networking.  The GitHub job cannot independently prove a physical
disconnect inside its job container, so it is explicitly
`FUNCTIONAL_CI_NON_PROMOTING`: connected build/test networking is recorded and
those receipts are not accepted by the four-leg promotion summary.  The
current profile and formal evidence remain NOT_READY.

Promotion summary is outer-receipt bound.  An inner
`registration_plugin_release.receipt.json` cannot promote itself by declaring
`HOST_PROMOTION` in its environment or JSON.  The summary must reopen the
launcher-owned `registration_plugin_host.receipt.json` and its sidecar from
the parent evidence root, then revalidate the launcher/source/profile hashes,
container/image identity, successful network-disconnect command,
`post_disconnect_inspect.log` with an empty Docker network set, immutable host
command-log hashes, cleanup absence, evidence-storage identity, and the exact
child receipt path/bytes/sidecar.  The host receipt's nested child receipt and
artifact revalidation must equal the reopened child bytes before inner
artifact validation is considered.  Functional CI emits only inner
`FUNCTIONAL_CI_NON_PROMOTING` receipts and is not accepted by this gate.

### Immutable campaign-set binding

The four rows are one precommitted campaign, not four independently named
receipts.  `release_matrix.campaign_set` has schema
`registration-plugin-campaign-set-v1`, a stable `campaign_id`, the exact
ordered Humble/Jazzy × absent/present row set, and canonical row-set and
identity SHA-256 values.  The profile content hash binds this declaration;
the launchers never synthesize a campaign from timestamps or accept a caller
supplied campaign name.  Starting a new campaign requires a separately
reviewed profile/set revision and new fresh evidence roots.

The in-container runner, outer host receipt, and matrix summary each carry
and reopen the exact campaign-set object.  Summary validation rejects a
missing, stale, self-rehashed, or mixed-campaign child/host receipt even when
its profile, source manifest, image, and artifact hashes otherwise match.  It
also requires all four expected rows under that same identity before emitting
a summary; partial, duplicate, or cross-campaign rows remain fail-closed.

The prefetch receipt records only the exact response-header allowlist
`content-length`, `content-type`, `etag`, and `last-modified`.  Header names
must already be lowercase; values are bounded to 256 UTF-8 bytes and reject
control characters, unknown names, and secret-bearing fields such as
`Authorization` or `Set-Cookie`.  This same normalization is applied to the
default HTTPS fetch path and to the injected test seam; values are never
truncated before validation.

The outer host command sequence is fail-closed and source-bound:
`docker_run < dependency_install < dependency_environment_capture <
dependency_prefetch (present only) < network_disconnect* <
post_disconnect_inspect < release_runner`.  The prefetch command records the
current prefetch-script SHA and the child's sealed prefetch receipt/manifest
binding; a self-declared environment flag cannot substitute for this order or
binding.  Cleanup records the initial and post-stop inspections, an optional
successful `docker stop` when the container was running, successful `docker rm`,
and a nonzero post-remove `docker inspect` proving absence.  Every referenced
cleanup/inspect/command log is reopened as a mode-0444 single-link file and
matched to the outer log index and SHA/size before promotion.  Host receipts,
sidecars, and logs are bounded before reading, so oversized or empty sealed
inputs fail closed.  A container already observed stopped need not receive a
redundant `docker stop`; it must still prove `stopped`, `remove_requested`,
successful removal, and post-remove absence.

Image and mount identity is also based on reopened Docker output, not only the
outer JSON self-report.  The bounded `docker image inspect` log must show the
profile digest, pinned `RepoDigests`, and `linux/amd64`, and its command must
precede the exact absent preexisting-name inspection and `docker run`.  The run
argv is checked for the exact `--name`, repository read-only bind, evidence
parent read-write bind, and pinned image reference; an allowed `nice`/`ionice`
prefix must preserve that raw suffix.  The post-disconnect inspect must show
the same top-level image digest and exactly the two expected bind mounts,
including source, destination, `RW`, and `Mode`, and those observed values must
match the host receipt projection.  A daemon error, malformed output, wrong
image, swapped mount, or non-exact not-found response is not treated as
absence.

The same boundary validates command vectors by exact reconstruction rather
than token presence.  The `docker run` record must be exactly the profile
image, `tail -f /dev/null`, the seven required environment assignments, and
only the repository `:ro` and evidence-parent `:rw` bind mounts; `--privileged`,
extra capabilities, namespaces, networks, environment keys, or trailing
arguments are rejected.  The dependency install command is exactly
`docker exec <name> bash -lc <DEPENDENCY_INSTALL_COMMAND>`.  The dependency
environment capture command is reconstructed from the output path, distro,
image digest, install-command SHA, exit code, and capture-script SHA.  The
post-disconnect release runner is reconstructed from the same base
environment plus the three host-promotion assignments, the pinned container
name, `bash -lc`, and the canonical runner shell; an alternate shell,
additional environment assignment, or extra exec flag fails closed.  Only the
documented `nice`/`ionice` priority prefix may precede a command, and it must
leave the exact raw argv suffix unchanged.  These checks are exercised by
synthetic rejection tests for an extra `--privileged` flag, a wrong dependency
shell, and an untrusted runner environment key.
