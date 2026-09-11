# G0 onboarding-trial execution runbook

This runbook executes the four G0 onboarding rows defined by the
[comparable onboarding-trial contract](onboarding-trials.md). It is an
observer protocol, not a new beginner workflow. The operator follows the
canonical page named in the matrix; the observer supplies only identity
pinning, neutral timing, and evidence capture.

Before provisioning anything, run
`python3 scripts/check_onboarding_trial_matrix.py --json`. Its reviewed evidence
index reports which rows already exist and why they are or are not comparable,
so a maintainer does not accidentally replace useful evidence or mistake an
empty argument list for an empty project history.

The record contract is
[onboarding-trial-v1.schema.json](schemas/onboarding-trial-v1.schema.json),
and the validator is `scripts/check_onboarding_trial.py`. A PASS becomes
comparable only when the validator also receives the exact retained
`first-map-validation-receipt.json`; a copied hash string is insufficient.
Do not run this document's trial commands in the product checkout. Use a
disposable environment and a trial root outside the checkout.

## 0. Start one exact candidate session

On a prepared disposable row host, inspect the exact request before any
candidate download, image build, source mutation, or evidence write:

```bash
python3 scripts/start_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/12345 \
  --row docker-jazzy \
  --output-dir /evidence/g0-docker-jazzy-20260815-a \
  --human-measurements prompt \
  --acknowledge-dedicated-trial-host \
  --check-readiness
```

The acknowledgement is a human statement, not a detected property: use it
only after confirming the isolation requirements in
[Isolation and roles](#2-isolation-and-roles). Without it, the card reports
`CONFIRMATION_REQUIRED` instead of guessing. The read-only card checks the
exact URL and destination, Ubuntu/ROS row, x86_64 architecture, one measured
filesystem with at least 8 GiB free, local Docker or source prerequisites,
source RX counter when selected, and neutral-observer measurement mode. It
prints stable findings and exactly one shell-quoted next command.

`READY` means the machine checks, comparison measurements, and explicit
confirmation are in place. `READY_NONCOMPARABLE` means the row can run but
human active time or command count would remain unknown.
`CONFIRMATION_REQUIRED` means only the human isolation statement is missing;
`BLOCKED` lists every detected repair before a recheck. The readiness report
follows
[`candidate-trial-readiness-v1`](schemas/candidate-trial-readiness-v1.schema.json);
`--json` emits that contract. Every status performs zero network reads, zero
writes, and zero trial execution.

After `READY`, run the exact command printed by the card. The shortest
candidate path starts from the same successful Actions run URL and publishes
one complete local session:

```bash
python3 scripts/start_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/12345 \
  --row docker-jazzy \
  --output-dir /evidence/g0-docker-jazzy-20260815-a \
  --human-measurements prompt \
  --acknowledge-dedicated-trial-host
```

Choose `docker-humble`, `docker-jazzy`, `source-humble`, or `source-jazzy`.
This one command downloads and independently authenticates the four candidate
artifacts, creates the observer handoff, reruns the selected row's live
preflight, invokes the maintained probe, and publishes `handoff/`, `execution/`,
and schema-backed `session.json` together. The requested session directory
appears only after both child directories are internally complete. A valid
product `FAIL`, blocked preflight, or harness failure remains a terminal session
instead of being erased.

For a Docker row, the command also removes the manual observer-image build. It
derives a local tag from the exact observer Dockerfile SHA-256, builds that tag
only when absent and before timing, then requires exact contract, Ubuntu, and
recipe-hash labels plus an immutable local image ID. It never replaces an
existing tag. The privileged nested host still requires the explicit dedicated-
host acknowledgement; convenience does not weaken the isolation boundary.

The command performs public GitHub/GHCR/attestation reads and local trial-host
writes only. It does not dispatch a workflow or write to GitHub, GHCR, a
Release, an issue, or a community channel. Use `--human-measurements auto` on a
TTY for observer prompts; non-interactive execution preserves those values as
unknown and therefore non-comparable.

### Split preparation and execution

Before provisioning a host, prepare one local packet for the same product
version across all four rows. This catches a copied digest, a mixed Docker /
source version, or an accidental moving-tag identity before any timed work.
For an already published release, never copy its commit or digests by hand.
Pipe the complete read-only release audit into the packet generator so all
three immutable identities are derived from the same validated report bytes:

```bash
python3 scripts/check_published_release.py \
  --version 0.9.1 --json --require-published | \
python3 scripts/prepare_onboarding_matrix_packet.py \
  --published-release-report - --render \
  --output /tmp/g0-onboarding-observer-packet.md
```

The packet retains the exact release-report SHA-256, tag, commit, URL, and
both image digests. Its Docker preflight uses
`check_published_onboarding_identity.py` to rerun the live release audit and
require that those exact values still match immediately before a row starts.
A merely `PUBLISHED` release with a different commit or moved image digest is
`NOT_READY`, not a substitute packet.

For an authorized digest-only candidate, do not copy those values into release
arguments or manually assemble its evidence. Download, remotely audit, and
prepare the complete observer handoff from the exact Actions run in one command:

```bash
python3 scripts/prepare_candidate_trial.py \
  --workflow-run-url \
    https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/12345 \
  --output-dir /evidence/candidate-image-run-12345
```

The candidate command performs network reads but no trial or remote write. It
publishes nothing unless the independently re-downloaded artifact bytes,
workflow identity, registry manifests, and attestations all pass. Its new
directory contains `artifacts/`, `candidate-audit.json`, both observer packet
formats, and a schema-backed `preparation.json`; use `observer-packet.md` for
the trial. A failure removes staging and leaves the requested output absent.

Use the split path when the authenticated handoff must be reviewed or
transferred to a separately provisioned host. After completing the disposable-
host prerequisites in [Isolation and roles](#2-isolation-and-roles), the
handoff-to-evidence path for one selected row is one command. The
runner does not parse or execute command text from the
packet; it validates the five-file handoff, rebuilds the packet from the four
artifact bytes, selects one structured row, reruns that row's live preflight,
and derives the existing probe arguments itself:

```bash
python3 scripts/run_candidate_trial.py \
  --handoff-dir /evidence/candidate-image-run-12345 \
  --row docker-jazzy \
  --output-dir /evidence/g0-docker-jazzy-20260815-a \
  --acknowledge-dedicated-trial-host
```

Choose `docker-humble`, `docker-jazzy`, `source-humble`, or `source-jazzy`.
The acknowledgement means the selected machine is isolated and disposable,
its `/` filesystem contains no unrelated measured activity, source-row APT
and build changes are acceptable, and a Docker row may use the documented
privileged nested-container host. The command refuses a handoff or output
inside the product checkout and never overwrites an output directory.
For a Docker row it performs the same content-addressed observer-image
bootstrap as the combined session command; no separate Docker build is needed.

With an interactive terminal, the default `--human-measurements auto` asks the
independent observer for active hands-on time and the number of commands they
submitted. In a non-interactive run it records both as `null`; it never derives
human effort from wall time or counts internal subprocesses. Use
`--human-measurements prompt` to require a TTY or
`--human-measurements unknown` to choose the non-comparable mode explicitly.

The new output is atomic and contains a schema-backed `execution.json`, the
row-specific `row-preflight.json`, and, once a probe starts, a bounded
`trial-record.json`, deterministic `trial-audit.json`, the exact shareable
`first-map-validation-receipt.json` for a validated PASS, and `private/`
evidence.
A valid product `FAIL` remains `TRIAL_RECORDED` and is retained. A failed live
preflight becomes `PREFLIGHT_BLOCKED` without starting the probe. Malformed or
missing probe output becomes `HARNESS_ERROR`; any untrusted record is moved
under `private/` instead of being presented as shareable evidence. Review the
private directory before sharing because it may contain paths, logs, commands,
map geometry, and source/build artifacts. None of these statuses authorizes a
remote write.

The lower-level `prepare_onboarding_matrix_packet.py` command remains available
for a published-release report or for regenerating a candidate packet from an
already authenticated `artifacts/` directory. It performs no network read,
trial, cleanup, or GitHub/community write. Release mode requires a complete
`PUBLISHED` report and emits an exact-identity `READY` preflight;
candidate mode emits `REMOTE_AUDIT_PASS`/`READY` preflights. Both packet modes
emit one observer command per row for lower-level review and have status
`READY_FOR_READ_ONLY_PREFLIGHT`, not `COMPARABLE`. Keep the handoff, trial
records, and observer roots outside the product checkout.

Every row must retain all seven measurements named in the packet. In
particular, `active_operator_time_sec` and `command_count` must come from a
neutral human observer; a blank value keeps that row non-comparable. The
packet does not authorize a release, image publication, issue update, cohort
recruitment, or telemetry.

## 1. G0 matrix and current route decisions

Use one fresh environment per row. The suffix is the UTC trial date and an
optional repeat letter; it is part of the private trial record only.

| Trial | Schema documentation_path | Clean environment | Fixed input | Route decision |
| --- | --- | --- | --- | --- |
| g0-docker-humble-YYYYMMDD-a | docker-first-map | Ubuntu 22.04, x86_64, Docker | mid360-public-zenodo-14841855 | Execute [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) with the v0.9.0 digest example below, after verifying the release asset. |
| g0-docker-jazzy-YYYYMMDD-a | docker-first-map | Ubuntu 24.04, x86_64, Docker | mid360-public-zenodo-14841855 | Execute the same [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) with the documented `:jazzy` image and the verified v0.9.0 digest below. |
| g0-source-humble-YYYYMMDD-a | source-quickstart | Ubuntu 22.04, x86_64, ROS 2 Humble | mid360-public-zenodo-14841855 | Execute the [fixed source first map](getting-started.md#2-run-the-fixed-first-map-demo) at the reviewed product commit below. |
| g0-source-jazzy-YYYYMMDD-a | source-quickstart | Ubuntu 24.04, x86_64, ROS 2 Jazzy | mid360-public-zenodo-14841855 | Execute the same fixed source first map at the reviewed product commit below. |

The frozen v0.9.0 Docker examples supplied by the release assets are:

- Humble image digest:
  sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53.
- Jazzy image digest:
  sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef.

These are explicit G0 Docker examples, not permission to trust copied values.
The operator must verify the matching release-image JSON, tag commit, image
manifest digest, and (when available) GitHub attestation before use. The source
rows instead use this reviewed public identity:

~~~bash
SOURCE_COMMIT='549ef03017c776f23fc968881b346aa685356274'
SOURCE_VERSION='0.9.1'
export SOURCE_COMMIT SOURCE_VERSION
~~~

Both source rows must use that exact commit/version pair. The source preflight
below proves that it contains the documented dependency helper and fast
first-build route. A later Draft PR head does not silently replace this frozen
trial identity.

The Docker workflow's Humble/Jazzy matrix and the Dockerfile's `ROS_DISTRO`
argument prove that images are built for both distros. The beginner page now
names both tags and the shared first-map contract. The release workflow
publishes exact `v<VERSION>-<distro>` tags and records their digests. Use that
release evidence when available. The moving convenience tags are not trial
identities. Pull-request and manual Docker workflow runs are verification-only:
they receive no package-write permission and cannot publish or move a tag.
Only a `develop` push can update the convenience tags, and that update still
does not create a trial identity. The separate candidate workflow can publish
only untagged digests after its protected E2 request gate; its existence does
not authorize a run or make its output a release. Audit
`.github/workflows/docker.yml`, `.github/workflows/candidate-image.yml`,
`.github/workflows/release.yml`, `Dockerfile`, and the
[distribution identity rules](distribution.md#installed-source-identity).

If a row has no runnable, documented path after the preflight below, write a
valid FAIL record at the earliest stage. Do not make it pass by adding an
unlisted package, changing the dataset, using --skip-viewer, or inventing a
Jazzy command.

## 2. Isolation and roles

### Environment requirements

Prepare four independent disposable environments. A disposable VM is the
preferred Docker host because it gives the image pull a dedicated filesystem
scope. A disposable Ubuntu container or VM with ROS already installed is
acceptable for the source rows if its trial filesystem is isolated.

- Before a Docker row, the selected daemon must have no
  ghcr.io/rsasaki0109/lidar_slam_ros2 image, project dataset, or project
  output. If the shared development daemon has any of these, stop and use a
  fresh daemon or VM; do not remove them.
- Before a source row, the trial root must contain no project checkout, build,
  install, dataset, or output. ROS itself and platform tools may be part of
  the base environment.
- Put the trial root and its observer output on one dedicated filesystem when
  possible. For a disposable Docker VM, use / as the disk scope. For a
  source trial, use the dedicated trial mount containing the checkout and all
  generated data.
- Do not run docker system prune, docker builder prune, apt clean, or delete a
  shared Git, package, Docker, or dataset cache. Cleanup may remove only the
  named disposable trial root after the private evidence has been archived, or
  may destroy the disposable VM.

### Maintainer container-host probe

The repository includes `docker/onboarding-trial-host.Dockerfile` and
`scripts/run_docker_onboarding_probe.py` for a bounded machine probe when a
dedicated VM is not yet available. Without a dedicated filesystem it is useful
for replacing an unknown Docker product outcome with an honest `PASS` or
`FAIL`, but it remains non-comparable. A disposable VM can opt into the
dedicated-filesystem mode below when a comparable baseline is required.

Invoke this reviewed observer helper from the product checkout, but keep the
bounded record and every reported trial/observer root outside the checkout.
The operator path itself still runs only in the disposable nested host.

The observer image contains Ubuntu, Docker, and network inspection tools only.
Build it before timing, using 22.04 for Humble or 24.04 for Jazzy:

~~~bash
docker build --pull=false \
  -f docker/onboarding-trial-host.Dockerfile \
  --build-arg UBUNTU_VERSION=22.04 \
  -t lidarslam-onboarding-trial-host:22.04 docker
~~~

The probe starts a fresh nested daemon with a unique data root, `rprivate`
binds, and a dedicated network namespace. It never mounts the host Docker
socket. It refuses to start unless the nested daemon reports `overlay2`,
`/var/lib/docker`, zero images, and zero containers. The actual project image
is pulled only after the timer and RX counter start.

This instrumentation uses a privileged container. Prefer to run it inside a
disposable VM, never on an untrusted multi-user host, and never add a broad
host mount. The CLI therefore requires the explicit
`--allow-privileged-container-host` acknowledgement. On a shared host, keep
`peak_disk_bytes` as `null` because the helper's nested Docker paths do not
prove a dedicated filesystem. A dedicated VM may instead measure the host
filesystem that contains the trial and nested Docker data:

~~~bash
python3 scripts/run_docker_onboarding_probe.py \
  --trial-id "g0-docker-${ROS_DISTRO}-$(date -u +%Y%m%d)-a" \
  --ros-distro "$ROS_DISTRO" \
  --image-tag "ghcr.io/rsasaki0109/lidar_slam_ros2:v${VERSION}-${ROS_DISTRO}" \
  --image-digest "$IMAGE_DIGEST" \
  --product-version "$VERSION" \
  --record "$RECORD" \
  --disk-scope / \
  --acknowledge-dedicated-filesystem \
  --prompt-human-measurements \
  --allow-privileged-container-host
~~~

Use that mode only on a disposable VM whose selected filesystem has no
unrelated activity. The explicit acknowledgement is required even for `/`;
the probe checks that its trial root and nested Docker store are on that same
filesystem. If an independent observer has a paused stopwatch and a human
command log, the probe can retain those two aggregate values with
`--prompt-human-measurements` (a shorthand for
`--prompt-active-operator-time --prompt-command-count`). Otherwise pass
`--record-human-measurements-unknown` (a shorthand for
`--record-active-time-unknown --record-command-count-unknown`), or use the
individual unknown flags when only one observation is unavailable. The fields
then remain `null`. The helper's internal Docker
invocation is never an operator-submitted command under the contract. A
successful product route remains `PASS`, while the checker reports
measurement `INCOMPLETE` until every required measurement is present.

The script removes only its named nested-host container. It retains its unique
trial root, private log root, and bounded record for review. Archive and
validate the record first. Cleanup may then remove only the exact reported
trial root; private logs contain paths and exact internal commands and must not
be copied into Git. See the
[first measured Docker probes](evidence/onboarding/docker-machine-probes-2026-08-10.md)
for the resulting v0.9.0 evidence.

### Disposable source-host probe

`scripts/run_source_onboarding_probe.py` automates the observer instrumentation
for one source row without changing the product route. Run it only inside a
disposable x86_64 Ubuntu 22.04/Humble or Ubuntu 24.04/Jazzy VM. The source
quickstart installs apt dependencies and builds directly on that host; a shared
workstation is not an acceptable target.

Prepare an existing empty trial root, a separate observer parent, and a record
path outside both. The disk scope must be one filesystem containing the trial
root, `/usr`, and `/var`; `/` is the normal choice in a dedicated VM. Isolate
the VM network so its selected interface carries only trial traffic.

Before provisioning or timing either source VM, check the reviewed immutable
route from any networked observer checkout:

~~~bash
python3 scripts/run_source_onboarding_probe.py \
  --public-preflight \
  --source-commit "$SOURCE_COMMIT" \
  --product-version "$SOURCE_VERSION"
~~~

This mode requires no ROS installation, trial directory, or acknowledgement.
It performs GitHub reads only and writes nothing. `READY` exits `0`, a public
but unavailable route returns `NOT_READY` and exits `1`, and an API, decoding,
or observer failure exits `2`. The route is ready only when the same commit
contains the exact six-package quickstart inventory, explicit package selection,
repository-only dependency helper, tests-disabled build, canonical Getting
Started instructions, and matching `VERSION`.

After `READY`, review the disposable host and path plan:

~~~bash
python3 scripts/run_source_onboarding_probe.py \
  --trial-id "g0-source-${ROS_DISTRO}-dry-run" \
  --ros-distro "$ROS_DISTRO" \
  --source-commit "$SOURCE_COMMIT" \
  --product-version "$SOURCE_VERSION" \
  --trial-root "$TRIAL_ROOT" \
  --observer-parent "$OBSERVER_PARENT" \
  --disk-scope / \
  --record "$TRIAL_RECORD" \
  --dry-run
~~~

Dry-run validates the host and path contract and prints a plan, but performs no
network request or write. For the measured attempt, start a paused stopwatch
for active operator time, keep a private count of submitted commands, and run:

~~~bash
python3 scripts/run_source_onboarding_probe.py \
  --trial-id "g0-source-${ROS_DISTRO}-$(date -u +%Y%m%d)-a" \
  --ros-distro "$ROS_DISTRO" \
  --source-commit "$SOURCE_COMMIT" \
  --product-version "$SOURCE_VERSION" \
  --trial-root "$TRIAL_ROOT" \
  --observer-parent "$OBSERVER_PARENT" \
  --disk-scope / \
  --record "$TRIAL_RECORD" \
  --prompt-human-measurements \
  --acknowledge-disposable-host \
  --acknowledge-isolated-network
~~~

`--prompt-human-measurements` is the short form for both observed human
measurements; the individual prompt flags remain available when an observer
needs to mix an observed value with an explicitly unknown one. Use
`--record-human-measurements-unknown` when neither observation exists, or use
`--record-active-time-unknown` and/or `--record-command-count-unknown` instead
of the corresponding prompt when only one observation is unavailable. This
keeps the record honest but makes its measurements incomplete. The measured
mode repeats the exact public
preflight before cloning, then runs the pinned source quickstart headlessly. A
public 404, package-inventory drift, missing helper, incomplete fast route, or
version mismatch writes a schema-valid bounded `FAIL` record before timing. An
API or observer failure exits 2 and does not invent an absence record. If the
checked-out quickstart still observes a different package inventory, the
private route log is reduced to stable finding
`source-package-inventory-mismatch` in the bounded record.

The private observer directory retains the generated pinning script, route log,
disk samples, and a copy of the bounded record. Keep it outside Git. The
requested record contains only aggregate measurements and artifact hashes.
Sections 3, 4, 7, and 8 remain the authoritative manual protocol; this helper
applies that protocol consistently rather than defining a second source path.

The operator sees only the canonical documentation and the observer-approved
identity substitution. The observer starts and stops timers and asks for the
operator's aggregate command count after the route; it does not infer that
count from harness commands or suggest fixes. A recovery hint, undocumented
option, or manually supplied path is an intervention and must be counted. If
it is needed for progress, the row cannot be an accepted comparable baseline.

### Private trial workspace

Run this preparation outside the product checkout. These are observer setup
commands and do not count toward measurements.command_count:

~~~bash
TRIAL_ID='g0-docker-humble-20260810-a'
TRIAL_ROOT="$(mktemp -d "/tmp/lidarslam-g0-$TRIAL_ID.XXXXXX")"
OBSERVER_ROOT="$(mktemp -d "/tmp/lidarslam-g0-observer-$TRIAL_ID.XXXXXX")"
mkdir -p "$TRIAL_ROOT" "$OBSERVER_ROOT"
test -z "$(find "$TRIAL_ROOT" -mindepth 1 -print -quit)"
~~~

Keep raw stopwatch notes, disk samples, terminal captures, and any local
paths under OBSERVER_ROOT. They must not be copied into the trial JSON or a
public issue.

## 3. Identity and preflight gates

Complete identity checks before the timed operator path. A failed identity or
route gate is a FAIL and does not justify a local build or a moving tag.

### Docker identity

For a documented Docker row, prefer the exact release-image record attached to
the GitHub Release. For the frozen v0.9.0 Humble example, the identity inputs
are:

~~~bash
IMAGE_TAG='ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble'
EXPECTED_DIGEST='sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53'
EXPECTED_VERSION='0.9.0'
IMAGE_DIGEST="$(docker buildx imagetools inspect "$IMAGE_TAG" \
  --format '{{json .Manifest}}' | jq -er '.digest')"
test "$IMAGE_DIGEST" = "$EXPECTED_DIGEST"
IMAGE_REF="$IMAGE_TAG@$IMAGE_DIGEST"
printf 'image_ref=%s\nversion=%s\n' "$IMAGE_REF" "$EXPECTED_VERSION" \
  > "$OBSERVER_ROOT/image-identity.txt"
~~~

For Jazzy, the frozen release identity is:

~~~bash
IMAGE_TAG='ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy'
EXPECTED_DIGEST='sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef'
EXPECTED_VERSION='0.9.0'
IMAGE_DIGEST="$(docker buildx imagetools inspect "$IMAGE_TAG" \
  --format '{{json .Manifest}}' | jq -er '.digest')"
test "$IMAGE_DIGEST" = "$EXPECTED_DIGEST"
IMAGE_REF="$IMAGE_TAG@$IMAGE_DIGEST"
~~~

The manifest comparison is read-only and does not pull image layers. Also
verify the attached release-image JSON's tag, digest, product version, platform,
and tag commit. If the release process requires attestation verification, use
the documented form:

~~~bash
gh attestation verify \
  oci://ghcr.io/rsasaki0109/lidar_slam_ros2@$EXPECTED_DIGEST \
  -R rsasaki0109/lidar_slam_ros2
~~~

Do not run a CLI smoke container before the timed trial: that would pull and
warm the project image. If the image digest, release asset, or product version
cannot be established without guessing, record FAIL at preflight.

#### Pre-release candidate digest

A candidate row is allowed only after a separately authorized
`e2-publish-candidate-image` run completed its Humble and Jazzy jobs and the
`verify immutable candidate pair` job. Run the section 0 one-command preparation
against that exact run and retain its `artifacts/` directory: the request, both
per-distro image records, and `candidate-image-set.json`. Require the set status
`PASS`, the
same source PR, exact source commit, product version, workflow run URL, and
requester across both images, distinct distro digests, empty `tags_created`
arrays, and false moving-tag/release mutation fields.

The candidate workflow deliberately records
`registry_retention_status: REQUIRES_REMOTE_AUDIT`. The section 0 preparation
already runs the bounded read-only audit and writes its passing report. Before
provisioning a timed host, an independent observer can repeat the packet's audit
against those same retained bytes:

~~~bash
python3 scripts/audit_candidate_image_set.py \
  --candidate-evidence-dir /evidence/candidate-image-run-12345/artifacts \
  --remote \
  --json
~~~

Require `REMOTE_AUDIT_PASS`. The audit checks the exact
`repository_dispatch` run, re-downloads every unexpired artifact into a
temporary directory, byte-compares it with each retained file, and checks each
`ghcr.io/...@sha256:...` manifest and both GitHub attestations without pulling
image layers. It reports the four retained hashes, their canonical bundle
hash, and the earliest artifact-expiry date; the temporary downloads are
removed and no GitHub or registry write authority is granted. If either
digest is absent, an attestation fails, any retained byte differs, an artifact
expired, or the environment/CI request record is missing, the candidate row
is FAIL.

A digest-only candidate has no published `v<VERSION>-<distro>` tag and is not
a GitHub Release. Do not relabel it, create a convenience tag, or pass it to a
release-only public preflight. Observer packet v3 binds both the canonical
four-file bundle SHA-256 and exact set SHA-256, source PR and commit, workflow
run, requester, both immutable references, and retention contract into each
candidate command; it never invents a release tag. The checked-in matrix still
changes only after reviewed trial records are captured. Merely having a
pullable digest is not comparable evidence.

### Source identity

The v0.9.0 tag commit predates the shared source first-map script. The source
rows therefore pin the explicit public commit and matching version declared in
section 1, rather than a moving branch or stale release tag. Verify both its
remote identity and its canonical source-route files before starting a timed
clone.
The `--public-preflight` command above is the maintained machine check. For an
independent manual audit, use:

~~~bash
SOURCE_COMMIT="${SOURCE_COMMIT:?export the reviewed 40-character source commit}"
SOURCE_VERSION="${SOURCE_VERSION:?export the matching product version}"
[[ "$SOURCE_COMMIT" =~ ^[0-9a-f]{40}$ ]]
[[ "$SOURCE_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]
REPO_URL='https://github.com/rsasaki0109/lidar_slam_ros2.git'
REMOTE_COMMIT="$(gh api \
  "repos/rsasaki0109/lidar_slam_ros2/commits/$SOURCE_COMMIT" --jq .sha)"
test "$REMOTE_COMMIT" = "$SOURCE_COMMIT"
test "$(gh api \
  "repos/rsasaki0109/lidar_slam_ros2/contents/scripts/source_quickstart.sh?ref=$SOURCE_COMMIT" \
  --jq .type)" = 'file'
GETTING_STARTED_CONTENT="$(gh api \
  "repos/rsasaki0109/lidar_slam_ros2/contents/docs/getting-started.md?ref=$SOURCE_COMMIT" \
  --jq .content | base64 --decode)"
grep -Fq 'bash scripts/source_quickstart.sh' \
  <<<"$GETTING_STARTED_CONTENT"
grep -Fq '6 ROS packages' <<<"$GETTING_STARTED_CONTENT"
grep -Fq 'BUILD_TESTING=OFF' <<<"$GETTING_STARTED_CONTENT"
test "$(gh api \
  "repos/rsasaki0109/lidar_slam_ros2/contents/VERSION?ref=$SOURCE_COMMIT" \
  --jq .content | base64 --decode)" = "$SOURCE_VERSION"
printf 'source_commit=%s\nsource_version=%s\n' \
  "$SOURCE_COMMIT" "$SOURCE_VERSION" \
  > "$OBSERVER_ROOT/source-identity.txt"
~~~

A `404` is an honest preflight FAIL with finding
`source-candidate-not-published`; a missing helper, package contract, or build
flag is `source-route-contract-missing`. Neither finding authorizes a push, a
local-path clone, or fallback to `develop`. A passing source trial record uses
kind `git-commit` and the exact verified 40-character lowercase value.

### Route audit

Before starting the stopwatch, check the prerequisites already named by the
canonical page. Do not install or repair anything during this audit.

For source rows, audit only the prerequisites named by the canonical build and
fixed-demo sections: the selected ROS distribution, Git, rosdep, colcon, and
the documented build dependencies. The fixed first-map route is headless and
does not require `rosbags-convert`, Docker, RViz, a display, or an Autoware
checkout. If an unlisted prerequisite is required, record FAIL at preflight
with a stable finding rather than installing it privately.

The preflight record for a route that cannot be executed has:

- environment.clean_start: true only if the isolation checks passed;
- the planned exact image digest or source commit, not a made-up identity;
- all unknown measurements as null;
- outcome.status: FAIL;
- failure_stage: preflight and at least one finding code;
- manifest_status: missing, diagnosis_status: missing,
  verifier_status: NOT_RUN, receipt_status: NOT_CREATED;
- both evidence hashes as null.

This is a valid audit of an unavailable route. It is not a skipped row.

## 4. Neutral measurement procedure

The schema's seven measurements are captured as follows. Use the same procedure
for every row and retain the raw observations only under OBSERVER_ROOT.

### Wall time and active operator time

Start wall_time_sec immediately before the operator submits the first command on
the canonical path. Stop at the first of these events:

1. the required first-map receipt is written and the documented path has no
   remaining map-producing work; or
2. the attempt reaches a terminal failure and no documented next action can
   continue it.

The fixed Docker and source routes are headless. Stop at the generated receipt;
an optional viewer command is outside this trial. Record the exact process exit
code observed by the observer.

active_operator_time_sec is a paused stopwatch. Include only time spent
entering commands, answering a documented prompt, reading required output, or
following a documented next action. Pause during downloads, builds, SLAM,
conversion, verification, and viewer startup. Do not include observer note
taking.

### Command count

Count each command submission by the operator. A copied multiline shell block
is one command, as specified by the contract. Commands invoked internally by a
script do not count. A documented recovery command counts; an observer's disk
sampler, identity lookup, evidence hash, or record-writing command does not.

If the route is rejected before the operator submits a product command, set
command_count to null; that correctly makes the record incomplete and
non-comparable while preserving the preflight finding.

### Peak disk bytes

Use the same filesystem scope for all four rows. Capture allocated filesystem
use, not a rounded df display value. The following observer sampler has a fixed
250 ms interval and writes only to the private observer root:

~~~bash
DISK_SCOPE="$TRIAL_ROOT"  # use / only inside a dedicated Docker VM
disk_used_bytes() {
  df --output=used -B1 "$DISK_SCOPE" | awk 'NR == 2 {print $1}'
}
BASE_USED="$(disk_used_bytes)"
(
  while :; do
    printf '%s\t%s\n' "$(date -u +%s.%N)" "$(disk_used_bytes)"
    sleep 0.25
  done
) > "$OBSERVER_ROOT/disk.tsv" &
DISK_PID=$!
~~~

Start the stopwatch after BASE_USED is captured and the sampler is running.
After the route reaches its terminal event, stop the sampler and calculate:

~~~bash
kill "$DISK_PID" 2>/dev/null || true
wait "$DISK_PID" 2>/dev/null || true
PEAK_USED="$(awk 'BEGIN {max = 0} $2 > max {max = $2} END {print max + 0}' \
  "$OBSERVER_ROOT/disk.tsv")"
PEAK_DISK_BYTES=$((PEAK_USED - BASE_USED))
test "$PEAK_DISK_BYTES" -ge 0
~~~

The scope must include the Docker daemon's image layers, or the source
checkout, build/install tree, dataset, temporary output, and final output. If
the scope also contains unrelated activity, set environment.clean_start to
false and retain environment_not_clean as a comparability blocker.

### Workflow and input download bytes

`measurements.workflow_download_bytes` measures the complete cold-start
transfer burden: image layers or source checkout, dependencies, and dataset.
Use a dedicated trial VM or network namespace with no unrelated traffic. On
the system that performs the pull or source build, select the trial interface
and capture its received-byte counter immediately before and after the timed
path:

~~~bash
NET_IFACE="${NET_IFACE:-$(ip -o route show default \
  | awk 'NR == 1 {print $5}')}"
test -r "/sys/class/net/$NET_IFACE/statistics/rx_bytes"
RX_START="$(cat "/sys/class/net/$NET_IFACE/statistics/rx_bytes")"
# Start the timed documented path here.
# Stop after the receipt or terminal failure.
RX_END="$(cat "/sys/class/net/$NET_IFACE/statistics/rx_bytes")"
WORKFLOW_DOWNLOAD_BYTES=$((RX_END - RX_START))
test "$WORKFLOW_DOWNLOAD_BYTES" -ge 0
~~~

If the interface carries background updates, observer SSH traffic, or another
workload, the value is not comparable. Repeat in an isolated environment or
store `null`; do not subtract an estimated background rate. Registry-manifest
layer sizes are useful audit inputs but do not replace the observed total.

`input.download_bytes` is the dataset-only portion of that total. Direct
`DEMO_DATA_DIR` to the private trial root and use the downloader's exact byte
record or retained archive size, for example:

~~~bash
stat --format='%s' \
  "$TRIAL_ROOT/data/driving_slam_mid360/archives/rosbag2_2024_04_16-14_17_01.zip"
~~~

Both Docker and source now use this same MID-360 downloader and fixed input.
When the archive path differs, use its machine-readable download record rather
than a human-readable `517 MB` label. The complete workflow value must be at
least the dataset value. If a route fails before dataset transfer, `0` is valid
only when the observer can prove no dataset bytes were transferred; otherwise
use `null`.

### Output bytes

After the terminal event, identify the one run directory created under the
dedicated trial root. Use allocated bytes, excluding the source dataset:

~~~bash
du -sx --block-size=1 "$RUN_DIR" | awk '{print $1}'
~~~

For a failure with no output directory, 0 is known. If a partial output cannot
be isolated from unrelated data, use null.

## 5. Docker Humble execution

The observer must use the exact v0.9.0 Humble digest in place of the moving
`:humble` convenience tag. The bind mount
and environment overrides below preserve the default command while retaining
the dataset manifest for exact byte capture; they are observer instrumentation,
not operator help.

Prepare an empty trial root and start the disk sampler from section 4. Then
start the stopwatch and submit this one operator command:

~~~bash
mkdir -p "$TRIAL_ROOT/data" "$TRIAL_ROOT/output"
docker run --rm \
  -e DEMO_DATA_DIR=/trial/data \
  -e DEMO_OUTPUT_DIR=/trial/output/mid360_demo \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  --mount type=bind,src="$TRIAL_ROOT",dst=/trial \
  "$IMAGE_REF"
~~~

The mkdir is observer preparation when it is done before timing; the docker
run is the operator command. The default image command remains
bash scripts/run_docker_demo.sh; do not replace it with a lower-level launch or
a local docker build. If the image pull, demo download, map run, verification,
or receipt fails, preserve the partial output and finalize a FAIL with the
earliest applicable stage.

For a successful attempt, the expected run directory is
$TRIAL_ROOT/output/mid360_demo. The required receipt is
first_map_validation_receipt.json; the manifest and verification artifacts are
described in [Getting Started](getting-started.md#4-check-the-result).

## 6. Docker Jazzy execution

The beginner page now explicitly documents the exact v0.9.0 Jazzy tag and the
same fixed first-map contract. Repeat section 5 in a fresh Ubuntu 24.04 trial
environment after setting the Jazzy identity from section 3:

~~~bash
IMAGE_TAG='ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy'
EXPECTED_DIGEST='sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef'
IMAGE_REF="$IMAGE_TAG@$EXPECTED_DIGEST"
~~~

Do not reuse the Humble VM, pulled layers, dataset, or output. The operator's
Docker command and expected run directory remain otherwise identical. Any
Jazzy-only dependency, launch, verification, or receipt failure is retained as
a real row finding.

## 7. Source Humble and Jazzy execution

The source rows use the same documented path on their respective supported
Ubuntu/ROS pair. The observer pins the checkout before the build, and the
operator then follows the source build and fixed-demo sections. Identity
checkout commands are observer instrumentation; they are not product
workarounds.

After the identity and route gates pass, start with an empty TRIAL_ROOT and the
disk and network samplers. Start timing immediately before the first clone
command. Submit the canonical source sequence with the reviewed commit pinned:

~~~bash
mkdir -p "$TRIAL_ROOT/src"
git clone --recursive "$REPO_URL" "$TRIAL_ROOT/src/lidar_slam_ros2"
git -C "$TRIAL_ROOT/src/lidar_slam_ros2" checkout --detach "$SOURCE_COMMIT"
test "$(git -C "$TRIAL_ROOT/src/lidar_slam_ros2" rev-parse HEAD)" = "$SOURCE_COMMIT"
git -C "$TRIAL_ROOT/src/lidar_slam_ros2" submodule update --init --recursive
cd "$TRIAL_ROOT"
bash src/lidar_slam_ros2/scripts/source_quickstart.sh \
  --workspace "$TRIAL_ROOT" --viewer none
~~~

The clone, commit check, and submodule update are the observer's pinning
wrapper around the documented recursive clone. If the operator must perform
any additional checkout repair or dependency installation beyond the page,
count it as an undocumented manual step and set the outcome to FAIL or, at
minimum, non-comparable. Do not substitute the old NTU VIRAL dogfood route, a
different bag, a viewer option, or a hand-written lower-level SLAM command.

For a completed source attempt, locate the single run directory under the
trial root's `output/mid360_demo/` directory, then apply the evidence and byte
rules above. The dataset cache under `datasets/mid360_public/` is input data and
is excluded from `output_bytes`.

## 8. Finalize and validate the trial record

Create the record in a private evidence directory, never in the product
checkout. Do not paste the exact command, private path, hostname, raw log,
operator identity, bag metadata, or map geometry into the JSON.

Capture the two allowed evidence identities when the files exist:

~~~bash
MANIFEST_SHA256="$(sha256sum "$RUN_DIR/run_manifest.json" | awk '{print $1}')"
RECEIPT_SHA256="$(sha256sum "$RUN_DIR/first_map_validation_receipt.json" \
  | awk '{print $1}')"
: "${BOUNDED_EVIDENCE:?set a reviewed bounded-evidence directory}"
VALIDATION_RECEIPT="$BOUNDED_EVIDENCE/first-map-validation-receipt.json"
cp --no-clobber \
  "$RUN_DIR/first_map_validation_receipt.json" "$VALIDATION_RECEIPT"
~~~

The retained receipt is intentionally privacy-bounded by its schema: it has no
map geometry, private paths, or exact command. Preserve its exact bytes; do not
reformat it. The maintained Docker/source probes perform this exclusive copy
when given `--validation-receipt-output`, and the candidate runner publishes it
atomically beside `trial-record.json`. Keep the manifest and all other raw run
material private.

Use the actual artifact state to fill the schema fields:

Also populate the required common fields exactly: schema_version `1`, the
schema URI from the contract, a lower-case slug trial_id, captured_at as a UTC
ISO 8601 timestamp, documentation_path from the matrix, operator_class as
maintainer or external, dataset_class `fixed-public`, and the fixed dataset_id.
Set privacy.contains_private_paths, privacy.contains_exact_command, and
privacy.contains_operator_identity to `false`; set
privacy.review_before_sharing to `true`.

| Record field | Source of truth |
| --- | --- |
| environment.product_version | Exact release-image record or VERSION in the pinned source checkout; never a guessed tag. |
| environment.revision | image-digest (sha256: plus 64 hex digits) for Docker, or git-commit (40 lowercase hex digits) for source. |
| outcome.runner_exit_code | The product route's observed exit code; use null when no route command ran. |
| outcome.manifest_status | run_manifest.json.status, or missing when the file was not created. |
| outcome.diagnosis_status | The generated diagnosis status, or missing when unavailable. |
| outcome.verifier_status | first_map_validation_receipt.json.verification.autoware_status, or NOT_RUN. |
| outcome.receipt_status | first_map_validation_receipt.json.status, or NOT_CREATED. |
| evidence.*_sha256 | SHA-256 of the manifest/receipt file when its corresponding file exists; otherwise null. |
| outcome.failure_stage | Earliest terminal stage: preflight, install, download, mapping, verification, receipt, or viewer. Use none only for a fully passing route. |
| outcome.finding_codes | Stable lower-case slugs. A FAIL always has at least one. |

For PASS, all of the following must be true: route exit 0, manifest
succeeded, diagnosis success, verifier PASS, receipt PASS, zero undocumented
manual steps, failure stage none, and both hashes present. A valid PASS that
is dirty, incomplete, or uses a release tag instead of an immutable identity is
not an accepted comparable baseline; retain its checker-reported blocker.

For FAIL, preserve the observed statuses and hashes even when partial
artifacts exist. A preflight failure has no fabricated receipt. A failed map
run may still have a hashable failed manifest or failed receipt. Never change
the outcome to PASS because a map directory happens to exist.

Validate first as a valid record, then as a comparable baseline when PASS is
expected:

~~~bash
python3 scripts/check_onboarding_trial.py "$TRIAL_RECORD" --json
python3 scripts/check_onboarding_trial.py "$TRIAL_RECORD" \
  --validation-receipt "$VALIDATION_RECEIPT" \
  --json --require-comparable
~~~

The first command must exit 0 for both expected PASS and expected FAIL. The
second must exit 0 only for a complete, clean, immutable, intervention-free
PASS. Exit 1 is useful evidence for a valid but non-comparable attempt; exit 2
means the record itself violates the schema or semantic contract and must be
corrected from observation, not guessed.

When the observer retained a human stopwatch or command tally separately from
the product record, do not edit the original JSON. Create a SHA-bound
measurement supplement instead:

~~~bash
python3 scripts/complete_onboarding_measurements.py \
  "$TRIAL_RECORD" \
  --validation-receipt "$VALIDATION_RECEIPT" \
  --prompt-human-measurements
python3 scripts/check_onboarding_trial.py "$TRIAL_RECORD" \
  --supplement "$TRIAL_RECORD.measurements.json" \
  --validation-receipt "$VALIDATION_RECEIPT" \
  --json --require-comparable
~~~

The supplement is valid only for the exact base-record bytes and can fill null
fields only; it cannot overwrite a value already observed in the trial. For a
dedicated Docker filesystem, provide the separately sampled peak with
`--peak-disk-bytes`. Review the supplement as bounded evidence before adding
its path and the exact retained receipt path to the matrix evidence index. The
index rejects a receipt whose bytes, schema, PASS state, manifest hash, product
version, fixed profile, or source commit disagree with the trial. The helper
defaults to
`$TRIAL_RECORD.measurements.json`, keeps `--json` stdout machine-readable, and
prints the next comparable-check command after writing the supplement. With
`--prompt-human-measurements`, it also prints a stderr measurement card before
the prompts: the card fixes the paused-stopwatch and command-count rules,
shows the wall-time upper bound, and says to leave unobserved values blank.

After all available rows pass their individual validity checks, evaluate the
fixed matrix as a separate gate:

~~~bash
python3 scripts/check_onboarding_trial_matrix.py \
  "$DOCKER_HUMBLE_RECORD" "$DOCKER_JAZZY_RECORD" \
  "$SOURCE_HUMBLE_RECORD" "$SOURCE_JAZZY_RECORD" \
  --validation-receipt "$DOCKER_HUMBLE_RECEIPT" \
  --validation-receipt "$DOCKER_JAZZY_RECEIPT" \
  --validation-receipt "$SOURCE_HUMBLE_RECEIPT" \
  --validation-receipt "$SOURCE_JAZZY_RECEIPT" \
  --json --require-activation-gate
~~~

Missing rows remain `MISSING`; a valid FAIL remains FAIL. The activation gate
requires all four outcomes to be present, all rows to use one product version,
plus at least one comparable Docker PASS and one comparable source PASS. The
checker may report valid rows from different product versions, but it keeps
the activation and all-row gates closed and emits an explicit alignment action.
Use `--require-all-comparable` only for the stricter four-row comparison gate.
This audit does not authorize filling null measurements, publishing a
candidate, or bypassing any image-promotion gate.

Review the generated JSON once for privacy, then summarize only the bounded
fields in the [weekly growth scorecard](growth-scorecard.md). Keep raw observer
material private and outside Git unless a separate evidence review approves it.

## 9. Reviewer sign-off

The G0 execution is ready for review only when the reviewer can answer yes to
each applicable item:

- all four row IDs are present, with a PASS or actionable FAIL for both Docker
  and both source rows;
- every attempted row has a clean-start decision and an immutable image digest
  or 40-character Git commit;
- the operator received no unlisted recovery command or workaround;
- wall time, active time, command count, workflow and input download bytes,
  peak disk, and output bytes are either measured by the rules above or
  deliberately null;
- PASS records satisfy the receipt and evidence gates, and FAIL records name a
  real stage and finding code;
- every comparable PASS is bound to the exact retained, schema-valid first-map
  receipt named by its evidence-index row;
- the JSON passes the validator and contains no private paths, exact commands,
  identities, raw logs, bag metadata, or map geometry;
- no shared cache or unrelated image was pruned.

Only after this sign-off should the row summaries be used to choose the next
activation blocker for the 1,000-star roadmap.
