# Distribution and installed CLI

This page describes the supported source and container installation paths and
the remaining boundary before `lidarslam_ros2` can publish ROS buildfarm
packages.

## Supported source install

Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04 are the supported source-build
targets. Install the matching ROS distribution first. The source quickstart
selects its `/opt/ros` setup, prepares pinned submodules and repository-only
dependencies, verifies the exact six-package source inventory, builds only that
explicit package list, then runs the verified demo. It may ask for your sudo
password. Inventory drift fails before rosdep or compilation rather than
silently adding an experimental package to the beginner path.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd lidar_slam_ros2
bash scripts/source_quickstart.sh
```

Use `--dry-run` for a write- and network-free plan, or add `--json` for the
versioned [`source-quickstart-plan-v1` schema](schemas/source-quickstart-plan-v1.schema.json).
The JSON form is valid only with `--dry-run`, writes stdout only, and reports
the exact command arrays without network, APT, submodule checkout, build, demo,
or filesystem writes. Use `--build-only` to skip the 517 MB demo, or
`--viewer none` on a headless host. The installation build skips
test-target generation and does not build unrelated workspace packages. After
it completes, use the absolute installed `lidarslam-map` path printed by the
helper from any directory, including a fresh terminal. That direct launcher
auto-activates the matching aggregate `setup.bash` for its own process and
children; it does not modify the caller's shell. Use
`source ~/ros2_ws/install/setup.bash` when you want the short command name and
the rest of the workspace's ROS tools. Use the full build and checks in
[Operator Workflows](workflows.md) when developing or validating a change.
The [local direct-launcher evidence](evidence/source-launcher-activation-2026-08-12.md)
passes this fresh-terminal behavior on both Humble and Jazzy while keeping the
complete cold-source and package-manager gates separate.

The installed own-bag command is then available from any working directory:

```bash
lidarslam-map doctor                                         # install/readiness check
lidarslam-map demo "$PWD"                                # fixed public first map
lidarslam-map start /path/to/rosbag2 --editable
lidarslam-map doctor /path/to/rosbag2
lidarslam-map setup /path/to/rosbag2
lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map" --editable
lidarslam-map inspect "$PWD/output/my_map"
lidarslam-map view "$PWD/output/my_map"                    # offline 3D preview
lidarslam-map edit "$PWD/output/my_map" --plan plan.json --output-dir "$PWD/output/my_map_edited"
lidarslam-map merge "$PWD/output/day1" "$PWD/output/day2" --output-dir "$PWD/output/site_project"
lidarslam-map view "$PWD/output/my_map" --viewer foxglove # optional live viewer
```

Run `doctor` without a bag immediately after installation to verify the
curated helper set, matching prefix, supported ROS environment, bag reader, and
fixed-demo storage without network access or writes. Add a rosbag2 directory to
the same command when you are ready to inspect sensor compatibility.

The published container exposes the same installed CLI. For a Linux own-bag
run without a ROS workspace, use the repository's host launcher:

```bash
bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2
```

It mounts the whole bag read-only, disables external container networking,
runs as the host UID/GID, keeps every write under one new host output
directory, and invokes `lidarslam-map start` rather than a separate container
workflow. Before creating output, it requires the selected image to expose
`start --help` and binds the run to the resulting local immutable image ID.
`--dry-run` prints the exact expansion
without Docker, network, or writes; `--ros-distro jazzy` selects the Jazzy
image, and `--image <tag-or-digest>` pins an explicit image. Add `--json` to
`--dry-run` for the versioned `docker-map-bag-plan-v1` stdout contract; it also
performs no Docker call, network access, filesystem write, or output-directory
creation. The JSON contains local paths and leaves image identity/contract
preflight deferred until a live run. Sensor setup options follow `--`. The
helper is source/release-bundle delivery tooling, not an additional installed
CLI command.

### Standalone launcher asset for the next release

The release workflow now prepares `lidarslam-map-docker` as a direct,
attested asset beginning with v0.9.1. The builder replaces development markers
with the exact release tag and 40-character source revision, validates shell
syntax, and refuses to overwrite an existing output. A release-built launcher
defaults to `ghcr.io/rsasaki0109/lidar_slam_ros2:v<VERSION>-<distro>` rather
than a moving image tag. The published-release audit downloads but never
executes the remote shell payload; it checks its bounded UTF-8 form, exact
release identity, version-pinned image behavior, read-only input, network
isolation, image-capability preflight, and required-result gate.

The historical v0.9.0 release remains valid with its original six recovery
assets. Releases from v0.9.1 require the seventh launcher asset, and the
release job attests it together with the deterministic release bundle. The
asset is not available until a future finalized release actually publishes
and passes that seven-asset audit; see
[Getting Started](getting-started.md#clone-free-docker-launcher-release-gate)
for the guarded download route.

Enable command and option completion in Bash:

```bash
source "$(ros2 pkg prefix lidarslam)/share/lidarslam/product/completions/lidarslam-map.bash"
```

Use an absolute output path when it matters where artifacts are written. An
installed CLI defaults relative output to the current working directory; it
never writes into the read-only package share.

The launcher enforces that boundary for Python as well: delegated processes
run with bytecode writes disabled, package installation excludes development
`__pycache__`/`.pyc`/`.pyo` artifacts, and the installed-product gate compares
Python cache state across the complete prefix before and after its workflows.
The dual-distro all-six-source check is recorded in
[the local installation evidence](evidence/source-all-packages-install-2026-08-12.md).

## Names and compatibility

The package has historically installed a C++ ROS node named `lidarslam`. That
name remains unchanged:

```bash
ros2 run lidarslam lidarslam
```

The product CLI deliberately uses two non-conflicting installed spellings:

```bash
lidarslam-map --help
ros2 run lidarslam lidarslam-cli --help
```

Both spellings dispatch the same `demo`, `start`, `doctor`, `setup`, `run`
(including `--guided`),
`inspect`, `view`, non-destructive `edit`, and multi-session `merge` contract.
The `ros2 run` form is a compatibility shim, not a separate product workflow.
Inside a source checkout,
`./scripts/lidarslam` exposes the same contract.

Calling the absolute installed `lidarslam-map` path directly finds the curated
CLI resource in that prefix and activates its nearest aggregate `setup.bash`
before delegation. The repo-local wrapper does the same only when the candidate
workspace contains a matching installed CLI resource. It never scans arbitrary
locations or sources an unrelated parent workspace. This setup is private to
the command process; normal shell activation remains explicit.

## What the installation contains

The `lidarslam` package installs a curated runtime set:

- launch files, parameter presets, and RViz configuration;
- the historical C++ node;
- `lidarslam-map` and the `lidarslam-cli` ROS shim;
- the product runner, preflight, diagnosis, verification, conversion, and
  viewer helpers required transitively by maintained profiles;
- the repository product version.

Research trainers, sweep tools, generated benchmark output, and repository
media are not copied into the product-script directory.

Every Humble/Jazzy default CI job creates a fresh, non-symlinked install prefix
and checks all curated resources from an unrelated working directory. The gate
also runs `--version`, a public-demo dry-run, `doctor`, an own-bag dry run,
`inspect`, and the non-launching `view` validation path. It validates session history, comparison,
and a privacy-bounded support ZIP from an installed session, and confirms that
`ros2 run lidarslam lidarslam` was not replaced.

The Docker image is likewise built without `--symlink-install` and verifies
`lidarslam-map --version` before its build tree is removed.

## Source install upgrade contract

The separate install-upgrade gate starts with the immutable `v0.6.0`
`lidarslam` package, installs the candidate into that same non-symlinked merge
prefix, and compares it with a fresh candidate prefix:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/check_install_upgrade.py \
  --baseline-ref v0.6.0 \
  --evidence-dir output/install-upgrade/$ROS_DISTRO \
  --hardware-label my-amd64-$ROS_DISTRO-host
```

The gate rejects stale or missing package-owned paths, executable-bit drift,
and changed text resources after normalizing the install prefix. It then runs
the complete installed-CLI checker against upgraded and fresh prefixes,
including the historical ROS node. Binary content hashes are not compared
because independent build directories can change compiler build IDs.

The named Humble/Jazzy execution and machine-readable reports are in
[clean-prefix upgrade evidence](evidence/install-upgrade-2026-07-28.md).
This covers source-built prefix upgrades; Debian/ROS buildfarm package-manager
upgrades remain a separate release boundary.

## ROS apt install and upgrade gate

The manual `package-manager install and upgrade` workflow exercises that
separate boundary on clean Humble and Jazzy containers. It installs the exact
four product Debian versions, then verifies:

- all four packages have the requested upstream version;
- `ndt_omp_ros2 >= 0.1.0` and `rko_lio >= 0.3.2` resolved from ROS apt;
- every required command, runtime resource, schema, completion file, and
  compatibility shim passes the installed CLI contract under
  `/opt/ros/<distro>`;
- an in-place upgrade starts from a passing older main-channel installation,
  confirms its installed CLI identity, increases the product version, and
  leaves no path whose package ownership was removed;
- the package-manager-installed CLI produces and validates the pinned public
  MID-360 map under the same bounded real-data contract as the other
  distribution gates.

The read-only verifier is
`scripts/check_package_manager_install.py`; its report follows
[`package-manager-install-v1.schema.json`](schemas/package-manager-install-v1.schema.json).
It queries `dpkg`, package ownership, and the installed prefix, but never runs
`apt`. Package installation is isolated to the disposable workflow container.

The workflow is manual while no lidarslam buildfarm package exists, so normal
CI does not create a permanently failing network gate. Run clean-install
against ROS testing as soon as the candidate builds. While the older release
is still in main, run the main-to-testing upgrade before the next sync removes
that two-version window. Finally rerun clean-install against main after sync.
Only the last two named-distro artifacts together prove the normal apt path;
the implemented workflow alone is not evidence that unpublished packages
exist. Each dispatch exposes its exact source ref, product version, apt
channel, and mode in the immutable workflow run name. After the main-channel
run completes, verify its public identity and both matrix jobs without
installing packages locally:

```bash
python3 scripts/check_package_manager_release_readiness.py \
  --version "$(tr -d '\n' < VERSION)" \
  --require-ready
```

The audit uses an explicit `GITHUB_TOKEN` when provided, otherwise it
non-interactively reuses the active `gh auth` credential. If neither is
available, it keeps the anonymous read path and fails closed when that quota
is insufficient. The credential is sent only to the exact
`https://api.github.com` origin, the audit never writes to GitHub, and the
credential is never included in JSON output.

The resulting JSON contract is
[`package-manager-release-readiness-v1.schema.json`](schemas/package-manager-release-readiness-v1.schema.json).
The audit first resolves the exact `v<VERSION>` ref, including annotated tags,
to its public commit and requires the workflow `head_sha` to match that commit.
It inspects successful, failed, and still-running dispatches instead of hiding
non-successful attempts. A missing immutable source ref reports
`SOURCE_REF_MISSING`; no matching attempt reports `NOT_RUN`; an active attempt
reports `RUNNING`; and a completed unsuccessful matrix reports `FAILED`.
GitHub API failures or untrusted run identity report `BLOCKED`. Only `READY`
can satisfy live v1 readiness, and the checker never creates the missing tag or
dispatches the workflow.

Before dispatching the expensive real-data workflow, inspect the two binary
dependencies in disposable Humble and Jazzy containers:

```bash
python3 scripts/check_ros_apt_dependency_readiness.py --require testing
```

The checker reads both public `main` and `testing` channels, requires
`ndt_omp_ros2 >= 0.1.0` and `rko_lio >= 0.3.2` in both named distributions,
and writes a
[`ros-apt-dependency-readiness-v1`](schemas/ros-apt-dependency-readiness-v1.schema.json)
report. `IN_PROGRESS` means at least one testing binary is not ready;
`TESTING_READY` authorizes the testing-channel package-manager E2E;
`MAIN_READY` means the dependency half of the normal-channel gate has synced.
Docker, network, apt, or schema failures are `BLOCKED`, never
`not-published`. This preflight does not replace the installed product E2E:
it prevents a workflow dispatch that is guaranteed to fail before installation.
The package-manager release audit additionally refuses to print a dispatch
command until the exact public source tag resolves. When both prerequisites
pass, its action contains the complete `gh workflow run` command with source
ref, product version, channel, and mode rather than requiring operators to
reconstruct those inputs.
The
[2026-08-12 public-channel snapshot](evidence/ros-apt-dependency-readiness-2026-08-12.json)
records the current asymmetric state: Humble RKO-LIO 0.3.2 is ready in both
main and testing; Jazzy main still exposes 0.2.0 while testing exposes 0.3.2;
and `ndt_omp_ros2` remains unpublished in both channels for both distributions.

## Installed source identity

Every official install includes
`share/lidarslam/product/product-build-info.json`. The map runner uses this
file to populate `software.git_commit` and `software.git_dirty` in
`run_manifest.json`, even when the installed command runs outside its source
checkout.

A normal Git clone records its current 40-character commit and tracked dirty
state at build time. Untracked files are excluded, matching the runtime
manifest policy. The file contains no build timestamp or host path, so two
builds with the same source identity produce identical metadata.

Docker excludes `.git` from its build context. The Humble/Jazzy image
workflows therefore pass the checked-out revision and `dirty=false` explicitly
and reject an image whose installed revision differs from the checkout.
Packagers building from a Git-free source archive must provide the same
identity:

```bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIDARSLAM_SOURCE_REVISION:STRING=<40-character-commit> \
  -DLIDARSLAM_SOURCE_DIRTY:STRING=false
```

Without Git metadata or an explicit revision, the generated file records
`revision: null` and `dirty: null` rather than inventing provenance. Official
clean-prefix checks reject that incomplete identity.

## Container install

GHCR is the supported prebuilt amd64 delivery path for both ROS distributions.
The moving convenience tags track `develop` and are useful for evaluation:

```bash
docker pull ghcr.io/rsasaki0109/lidar_slam_ros2:humble
docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:humble \
  lidarslam-map --version

docker pull ghcr.io/rsasaki0109/lidar_slam_ros2:jazzy
docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:jazzy \
  lidarslam-map --version
```

`latest` remains an alias of the Humble convenience image for compatibility.
It is not a release identifier.

Only a repository `push` event on `develop` can update these moving tags. The
Docker workflow gives pull-request and manual `workflow_dispatch` jobs a
contents-read token, builds into the disposable runner with `push: false`, and
smoke-tests the installed CLI without logging in to GHCR. Those verification
runs cannot publish a package, attestation, candidate identity, or moving tag.
Manually dispatching the convenience workflow is not a candidate-publication
route.

#### Immutable matrix candidate gate (E2 only)

`.github/workflows/candidate-image.yml` is the separate digest-publication
gate. Pull requests execute only its contents-read contract job. Candidate
publication has no `workflow_dispatch` trigger and therefore no selectable
workflow ref: an explicit `e2-publish-candidate-image` `repository_dispatch`
event starts the workflow definition on the repository default branch. The
read-only authorization job then requires all of the following before the
matrix job can receive `packages: write`:

- the workflow ref and repository default branch are both exactly `develop`;
- the requester has the `maintain` or `admin` repository role and supplied the literal
  approval `E2_IMMUTABLE_DIGEST_ONLY`;
- the requested commit is the exact mergeable head of an open, same-repository
  pull request targeting `develop`, and its `VERSION` matches the requested
  product version;
- all nine required exact-head checks succeeded, no check is active or failed,
  and only the named non-publication jobs may be skipped; and
- the `candidate-images` environment already has one to six required
  reviewers, **Prevent self-review**, and exactly one custom deployment branch
  policy named `develop`, with no unknown protection rule.

An absent or unprotected environment is a hard failure, not an implicit
environment setup. Configure that environment in repository settings before
making an E2 decision. The dispatch and deployment approval must remain
separate maintainer actions. Audit the live settings with GET requests only:

```bash
python3 scripts/check_candidate_environment.py --json --require-ready
```

The schema-backed result distinguishes `ABSENT`, `MISCONFIGURED`, and
`BLOCKED`; a 404 by itself is never treated as proof that an environment is
absent. `READY` means only `READY_FOR_SEPARATE_E2_REVIEW`. The command cannot
write repository settings, authorize a dispatch, or publish an artifact.
Without `--json`, the same audit prints a status-specific operator handoff:
`ABSENT` gives the trusted repository-settings URL and the five exact creation
and independent-review steps; `MISCONFIGURED` gives the bounded repair
checklist; `BLOCKED` asks only for read-access recovery and explicitly forbids
settings changes from incomplete evidence; and `READY` points to the separate
exact-head E2 review. Every card ends with the copy-ready read-only verification
command and `Environment writes performed: no`. The handoff describes an
external administrator action; it never performs or authorizes that action.

After a separate E2 approval, the exact event shape is:

```bash
CANDIDATE_PR='<OPEN_SAME_REPOSITORY_PR_NUMBER>'
CANDIDATE_COMMIT='<EXACT_40_CHARACTER_PR_HEAD_SHA>'
CANDIDATE_VERSION='<MATCHING_X.Y.Z_VERSION>'

gh api --method POST \
  repos/rsasaki0109/lidar_slam_ros2/dispatches \
  -f event_type=e2-publish-candidate-image \
  -F "client_payload[pull_request]=${CANDIDATE_PR}" \
  -f "client_payload[source_commit]=${CANDIDATE_COMMIT}" \
  -f "client_payload[product_version]=${CANDIDATE_VERSION}" \
  -f 'client_payload[approval]=E2_IMMUTABLE_DIGEST_ONLY'
```

The POST above is the E2 registry-write boundary. Do not run it merely because
the workflow or PR checks are green. A successful run publishes Humble and
Jazzy manifests by digest only (`push-by-digest=true`), creates no version,
`humble`, `jazzy`, or `latest` tag, smoke-tests the exact digest with container
networking disabled, verifies SBOM, provenance, and GitHub attestation, and
uploads these schema-backed artifacts for 30 days:

- `candidate-image-request.json` — exact PR, actor, environment, CI
  authorization, and immutable default-branch gate commit;
- `candidate-image-humble.json` and `candidate-image-jazzy.json` — exact
  source/digest records with an empty `tags_created` array; and
- `candidate-image-set.json` — a complete, distinct, same-source pair.

The records deliberately say
`registry_retention_status: REQUIRES_REMOTE_AUDIT`. On a prepared disposable
row host, inspect that same request locally before any download, build, host
mutation, or evidence write:

```bash
python3 scripts/start_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/<RUN_ID> \
  --row <docker-humble|docker-jazzy|source-humble|source-jazzy> \
  --output-dir /evidence/<NEW_SESSION_DIRECTORY> \
  --human-measurements prompt \
  --acknowledge-dedicated-trial-host \
  --check-readiness
```

The schema-backed readiness card performs local reads only. It separates a
machine `BLOCKED` result, a missing human `CONFIRMATION_REQUIRED`, and a
runnable but `READY_NONCOMPARABLE` measurement plan from `READY`, then prints
one shell-quoted next command. The acknowledgement must be supplied only after
the documented disposable-host, filesystem, network, and route-mutation
claims are true; no checker can infer them. Automation can consume
[`candidate-trial-readiness-v1`](schemas/candidate-trial-readiness-v1.schema.json)
with `--json`.

After `READY`, the printed command starts from the exact Actions run URL and
finishes one selected row in one command:

```bash
python3 scripts/start_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/<RUN_ID> \
  --row <docker-humble|docker-jazzy|source-humble|source-jazzy> \
  --output-dir /evidence/<NEW_SESSION_DIRECTORY> \
  --human-measurements prompt \
  --acknowledge-dedicated-trial-host
```

Its atomic session contains `handoff/`, `execution/`, and schema-backed
[`session.json`](schemas/candidate-trial-session-v1.schema.json). The handoff
is independently downloaded and audited before the row's live preflight. For a
Docker row, the wrapper derives a local observer tag from the reviewed
Dockerfile SHA-256, builds it only when absent and before timing, and validates
its exact contract, Ubuntu, recipe labels, and immutable local image ID. A
blocked preflight, valid PASS/FAIL, or harness failure is retained as a bounded
terminal session; no remote mutation is performed.

Use the split path when the authenticated handoff must be reviewed or moved to
a different disposable host. First prepare it:

```bash
python3 scripts/prepare_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/<RUN_ID> \
  --output-dir /evidence/candidate-image-run-<RUN_ID>
```

Authenticate `gh` for public Actions artifact and attestation reads, and ensure
Docker Buildx is available. The output parent must already exist; the requested
directory must not. The command downloads all four artifacts, validates their
cross-file derivation, and downloads all four again into a temporary directory.
That independent pass byte-compares all four SHA-256 values, verifies both
registry manifests and attestations, and generates the JSON and Markdown
observer packet. It publishes the directory
atomically only after `REMOTE_AUDIT_PASS`; on any failure the requested output
remains absent. The schema-backed
[`preparation.json`](schemas/candidate-trial-preparation-v1.schema.json) names
the exact bundle/set hashes, expiry, relative outputs, and no-write authority.
The resulting layout is:

```text
candidate-image-run-<RUN_ID>/
├── artifacts/                  # exactly the four canonical JSON records
├── candidate-audit.json        # REMOTE_AUDIT_PASS
├── observer-packet.json
├── observer-packet.md
└── preparation.json            # READY_FOR_OBSERVER
```

Use `artifacts/` as `--candidate-evidence-dir` when independently rerunning
`python3 scripts/audit_candidate_image_set.py --remote --json` or regenerating
a packet. Those primitives remain available for review, but manual artifact
loops are no longer the normal path. Require
`REMOTE_AUDIT_PASS` before using either identity in an onboarding row. The
preparation command grants no GitHub or registry write authority and does not
run a trial. Do not invent release tags: a candidate digest is not a Git tag,
GitHub Release, stable image, or E4 approval.

On each dedicated candidate-trial VM, consume that complete handoff without
copying identities or packet commands by hand:

```bash
python3 scripts/run_candidate_trial.py \
  --handoff-dir /evidence/candidate-image-run-<RUN_ID> \
  --row <docker-humble|docker-jazzy|source-humble|source-jazzy> \
  --output-dir /evidence/<NEW_TRIAL_DIRECTORY> \
  --acknowledge-dedicated-trial-host
```

The row runner locally re-derives the handoff and reruns the selected Docker
remote audit or source public preflight before invoking the maintained probe.
Docker rows also perform the same content-addressed observer-image bootstrap,
so the normal candidate path no longer needs a separate `docker build` command.
Its atomic schema-backed
[`execution.json`](schemas/candidate-trial-execution-v1.schema.json)
distinguishes a blocked preflight, a schema-valid PASS/FAIL trial, and a
harness error. A validated product PASS also retains the exact privacy-bounded
`first-map-validation-receipt.json`; `execution.json` lists it as a bounded
output, and the comparability checker verifies its bytes against the trial
record rather than trusting a copied digest. Interactive `auto` mode prompts for the
two human observations; non-interactive mode leaves them unknown and the trial
non-comparable. It performs network reads and local trial writes, including
the acknowledged source-host or privileged-container work, but no GitHub,
registry, release, issue, or community mutation.

The reported `candidate_bundle_sha256` is reproducible: hash the UTF-8
concatenation of one `<canonical-filename>\t<file-sha256>\n` line for each
file, in request, Humble, Jazzy, set order. The report retains each component
hash as well, so a reviewer never has to infer which bytes a bundle hash names.

Every tagged release publishes exact
`ghcr.io/rsasaki0109/lidar_slam_ros2:v<VERSION>-<distro>` images only after
the repository tag matches `VERSION`. For example:

```bash
IMAGE=ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy
docker pull "$IMAGE"
docker run --rm "$IMAGE" lidarslam-map --version
```

Use an exact digest for deployment or rollback. Each GitHub Release attaches
`release-image-humble.json` and `release-image-jazzy.json`; they record the
tested tag, digest, tag commit, platform, product version, and observed CLI
version. The release workflow also rejects a runtime whose installed source
revision differs from that tag commit. It attaches the schema-validated
rollback plan generated from each record:

```bash
lidarslam-map rollback-plan release-image-jazzy.json
```

Run the printed pull, `gh attestation verify`, and CLI smoke commands, then
substitute the printed `ghcr.io/...@sha256:...` reference in the normal Docker
invocation. Keep the previous release-image JSON with deployment evidence so
the same last-known-good digest remains recoverable. Never implement rollback
by retagging an old image or moving a convenience/versioned tag; map outputs
are likewise immutable, so a post-rollback mapping run uses a new output
directory.

#### Pre-promotion compressed-size gate

Measure a local single-platform OCI export with
`scripts/measure_oci_archive.py` before public promotion. The checker verifies
the complete descriptor and diffID graph, the exact `linux/amd64` platform,
gzip media types, revision/version labels, and archive closure before applying
the reduction threshold. A baseline byte count is accepted only together with
its immutable tag, index digest, and platform-manifest digest. Its output uses
the
[`oci-image-measurement-v1` schema](schemas/oci-image-measurement-v1.schema.json).
The exact command and current Humble/Jazzy evidence are in the
[runtime-image slimming record](evidence/onboarding/runtime-image-slimming-2026-08-11.md#exact-compressed-oci-follow-up-commit-ff92f09).

This measurement export disables provenance only to keep its OCI index to one
runnable platform manifest. It is not a substitute for the SBOM, maximum-mode
provenance, attestation, and digest-bound smoke checks below; the final release
candidate must satisfy both contracts.

The release workflow builds from the tagged recursive checkout and initially
pushes each candidate by digest without a version tag. It requires the
installed CLI version and source-revision label to match, verifies the OCI
SBOM, maximum-mode BuildKit provenance, and signed GitHub attestation, and
then preflights the Humble/Jazzy pair together. Only tested digests are
promoted to version tags. A matching tag is safely reused during recovery; a
tag that already resolves to a different digest aborts the release instead of
moving it. The GitHub Release is created only after both distro images pass
and immutable promotion succeeds.

The deterministic release bundle embeds
`release-bundle-manifest-v1.json`, which records the tag commit and SHA-256 of
every included product document, evidence record, schema, contract, and media
file. `release-promotion.json` records which version tags were created or
reused and always declares `moving_tag_mutated: false`. Verify the GitHub
provenance with:

```bash
gh attestation verify \
  oci://ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:<digest> \
  -R rsasaki0109/lidar_slam_ros2
```

The version examples illustrate the tag contract; use a tag listed on the
GitHub Releases page. Convenience tags are intentionally moving, so recording
their current digest is mandatory when they are used in evaluation evidence.

### Bind-mounted output ownership

The default demo starts as container root so it can use the prebuilt workspace
and its internal dataset cache. On Linux, pass the invoking UID and GID to
return the dedicated output mount to the host user when the demo exits:

```bash
mkdir -p "$PWD/lidarslam_output"
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:humble
```

Both variables are required together and must be numeric. The demo changes the
owner of the dedicated output mount, the requested run directory and any
failed `.partial` or post-processing lock sidecar; unrelated sibling contents
are not changed.

The own-bag command above instead starts the complete process as the host
UID/GID, so no root-owned output is created. `HOME` and `ROS_LOG_DIR` are moved
to writable container/output locations because the image's `/root` is not
writable to that numeric user.

## Profile-specific extras

The flagship PointCloud2 + IMU profile is complete after the recursive source
install above. The `pointcloud_gnss_smoke` and `packet_applanix_smoke`
evaluation profiles additionally use the PyPI `rosbags` package to inspect raw
bag records. It has no Ubuntu 22.04/24.04 rosdep key, so keep it isolated in a
virtual environment:

```bash
python3 -m venv --system-site-packages ~/.venvs/lidarslam
source ~/.venvs/lidarslam/bin/activate
python3 -m pip install rosbags
source ~/ros2_ws/install/setup.bash
```

Activate that environment before invoking either GNSS profile. The workflow
fails before starting ROS processes and points back to this section when the
module is absent. This extra is not required by the default RKO-LIO path.

## Support matrix

| Delivery | Humble amd64 | Jazzy amd64 | arm64 / Jetson | Flagship RKO-LIO path |
| --- | --- | --- | --- | --- |
| Recursive source checkout + `colcon` | Tested in CI | Tested in CI | Evaluation; use the Jetson runbook | Included from the maintained submodule |
| GHCR image | Moving and versioned amd64 images | Moving and versioned amd64 images | Not yet published | Included |
| ROS buildfarm / apt | Not released | Not released | Not released | Official RKO-LIO 0.3.2 passed the Humble/Jazzy product gate |

`amd64` is the tested product target. Jetson/MID-360 workflows have real-device
evidence, but arm64 installation and image publication are still an evaluation
tier rather than a release guarantee.

## Binary-release boundary

There is currently no supported
`sudo apt install ros-<distro>-lidarslam` golden path. Two packaging gates
remain:

1. `ndt_omp_ros2`, which is a declared build dependency, has a source tag and
   generated Bloom PRs, but an unanswered review exposed that it installs the
   same `include/pclomp/*` and `libndt_omp.so` files as Humble's released
   `ndt_omp`. Resolve the review through upstream convergence or a fully
   namespaced replacement before any rosdistro merge. The preferred upstream
   API patch, parent PCL-pointer modernization, and final canonical dependency
   switch have been prepared and verified locally; see the
   [2026-08-12 convergence evidence](evidence/ndt-omp-release-review-2026-08-12.md).
2. Official PRBonn `rko_lio 0.3.2-1` passed the clean installed golden-path
   E2E on Humble and Jazzy. `lidarslam` now declares `rko_lio >= 0.3.2`.
   Humble main satisfies that minimum; Jazzy must still sync 0.3.2 from ROS
   testing to main before normal Jazzy apt dependency resolution can succeed.

See the [official binary evidence](evidence/official-rko-binary-compatibility-2026-07-29.md)
and [rosdistro release runbook](rosdistro-release.md) for the verified
boundary and remaining maintainer prerequisites.

Versioned Humble/Jazzy GHCR tags, release-image SBOM/provenance, digest smoke
tests, and attached installation evidence are automated for the next tagged
release. ROS buildfarm packages remain blocked by the NDT convergence review
and the Jazzy RKO-LIO main-sync gate above. The binary package-manager
clean-install/upgrade gate is implemented but cannot produce release evidence
until those packages exist; arm64 image publication also remains Phase 2 work.
