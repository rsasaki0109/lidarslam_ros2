# rosdistro Binary Release (bloom) — Runbook

Goal: `sudo apt install ros-humble-lidarslam` (and the Jazzy equivalent)
installs the four core packages from the ROS buildfarm.

This page records the dependency analysis and the exact release procedure.
The repository-side prep (versions, SPDX license tags, per-package
`CHANGELOG.rst`) landed with v0.5.0 and is maintained through v0.9.1; what
remains is the bloom/rosdistro procedure itself, which requires the
maintainer's GitHub account.

## Planned package set

| Package | Version | Notes |
|---|---|---|
| `lidarslam_msgs` | 0.9.1 | messages only |
| `scanmatcher` | 0.9.1 | NDT frontend (FastGICP / SmallGICP optional, off on the farm) |
| `graph_based_slam` | 0.9.1 | backend + `/map_save` Autoware bundle |
| `lidarslam` | 0.9.1 | launch + param presets |

These are the only `package.xml` files in the repository outside
`Thirdparty/`, so bloom's package discovery picks up exactly this set.
`Thirdparty/` consists of git submodules, which `git archive` (and therefore
bloom's upstream import) excludes — intended.

## Dependency readiness

| Dependency | rosdep key state | Action |
|---|---|---|
| rclcpp, rclcpp_components, tf2\*, \*_msgs, pcl_conversions, std_srvs | released ROS packages | none |
| `libg2o` | released (`ros-<distro>-libg2o`; verified 2026-06-11: `rosdep resolve libg2o` → `ros-humble-libg2o` on jammy, `ros-jazzy-libg2o` on noble) | none |
| `libpcl-all-dev` | standard rosdep key (system PCL) | none |
| **`ndt_omp_ros2`** | source tag and Humble/Jazzy Bloom PRs exist, but the candidate overlaps Humble's released `ndt_omp` files, has an unanswered convergence review, and both exact PR heads have a failed check run | converge on canonical `ndt_omp`, or fully isolate the fork; answer both reviews and require a current-base green replacement before requesting merge |
| `rko_lio` | PRBonn `0.3.2-1` is registered and built in testing for Humble/Jazzy; main has Humble `0.3.2` and Jazzy `0.2.0`; declared as `rko_lio >= 0.3.2` after the official-binary gate passed | wait for Jazzy `0.3.2` to sync to main before the normal Jazzy apt path |

This table was rechecked directly against the
[Humble distribution](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)
and
[Jazzy distribution](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml)
in `ros/rosdistro` on 2026-07-30. Neither distribution contains
`ndt_omp_ros2`; both register `rko_lio` `0.3.2-1` from
[`PRBonn/rko_lio`](https://github.com/PRBonn/rko_lio) and
`ros2-gbp/rko_lio-release`.

The amd64 apt indexes were rechecked on 2026-08-12. The ROS testing repository
contains `0.3.2-1` builds for both distributions. The main repository, which
normal users install from, now contains Humble `0.3.2` but still contains Jazzy
`0.2.0`. The exact observed versions are preserved in the
[2026-08-12 dependency snapshot](evidence/ros-apt-dependency-readiness-2026-08-12.json).

### NDT package convergence review must close first

`scanmatcher` directly declares `<depend>ndt_omp_ros2</depend>`. The
dependency is consumed from
`https://github.com/rsasaki0109/ndt_omp_ros2` at `8b77fa5`.
It is a downstream ROS 2 fork of `koide3/ndt_omp`, not an independent
algorithm. Its product-required delta is the rotation/translation-prior and
adaptive correspondence API consumed by `scanmatcher`.

The `0.1.0` source tag,
[release repository](https://github.com/rsasaki0109/ndt_omp_ros2-release),
and generated [Humble PR #52949](https://github.com/ros/rosdistro/pull/52949)
and [Jazzy PR #52950](https://github.com/ros/rosdistro/pull/52950) already
exist. Do not recreate the tag or rerun Bloom.

The current candidate is not safely co-installable with Humble's released
`ndt_omp 0.0.0-1`. Although the ROS and CMake package names differ, both
packages install `include/pclomp/*` and `lib/libndt_omp.so` and expose the
same `pclomp` namespace. The rosdistro reviewer correctly asked how the fork
relates to the existing package, and no maintainer response follows that
review. The
[2026-08-12 review audit](evidence/ndt-omp-release-review-2026-08-12.md)
records the exact lineage, consumed API delta, collision, and prepared
response. Both old PR heads also fail the same stale-base OpenEmbedded rosdep
check. The live preflight therefore reports `BLOCKED`, preserves both review
actions, and never treats this as wait-only `IN_PROGRESS`.

Run the read-only preflight immediately before doing any publication work:

```bash
python3 scripts/check_ndt_omp_release_readiness.py
python3 scripts/check_ndt_omp_release_readiness.py \
  --require-ready-to-tag \
  --output-json /tmp/ndt-omp-release-preflight.json
```

The first command describes the current state; the strict command exits 1
unless the exact reviewed candidate is `READY_TO_TAG`. It validates the
parent gitlink, submodule HEAD and cleanliness, package metadata, changelog,
CMake install/export contract, and Bloom CI assets. Its remote inspection
then verifies `origin/humble`, source tag, release-repository existence, both
rosdistro keys, generated PR state and mergeability, every exact-head check
run, and whether the latest actionable human review has a later author
response. Failed, pending, missing, inconsistent, or truncated check evidence
is `BLOCKED`. The checker uses an explicit `GITHUB_TOKEN` when provided,
otherwise it non-interactively reuses the active `gh auth` credential. If
neither is available, it keeps the anonymous read path and fails closed when
that quota is insufficient. The credential is sent only to the exact
`https://api.github.com` origin and is never included in the report. A GitHub
404 means an initial artifact is absent; any other HTTP, malformed response,
or network error is `BLOCKED`, never mistaken for absence, green CI, or
reviewer approval.

CI runs `--offline`, whose successful state is only `LOCAL_READY`. After
publication, use `--require-released`; it passes only when the tag, release
repository, and Humble and Jazzy rosdistro entries all exist. `IN_PROGRESS`
means publication is partial without an unanswered detected review.
`REVIEW_REQUIRED` names each unanswered human-review URL and fails every
strict release gate. A failed/pending/missing check suite, explicitly
unmergeable PR, or unresolved GitHub mergeability calculation never becomes a
wait-only result; it is `BLOCKED`, while any unanswered-review actions remain
visible. The JSON contract is
[`ndt-omp-release-readiness-v2.schema.json`](schemas/ndt-omp-release-readiness-v2.schema.json).
The checker is read-only; it never creates a tag, repository, or PR.

1. Prefer upstream convergence: open the prepared four-API change against
   `koide3/ndt_omp` as a Draft PR after its exact-base and duplicate checks
   pass.
2. Replace the response packet's `<UPSTREAM_PR_URL>` only with that verified
   Draft URL, then post the transparent full response to Jazzy and matching
   concise response to Humble. Acknowledge the downstream lineage and file
   collision; do not request merge of the current package as-is.
3. Address upstream review with focused changes. After acceptance, change
   both `scanmatcher` and `graph_based_slam` to the canonical `ndt_omp`
   package, and coordinate its Humble update and first Jazzy release.
4. Only if upstream declines the project-specific API, fully isolate the fork:
   new package identity, C++ namespace, include root, library/SONAME, CMake
   target, version/tag, Bloom tracks, and replacement rosdistro PRs.
5. Refresh or replace the selected registration from current rosdistro
   `master`, require every exact-head check to pass, and proceed to the four
   lidarslam packages only after both supported distros resolve one
   collision-free NDT dependency from ROS apt.

The implementation needed for step 2 has been prepared and tested locally.
The upstream patch is based on exact `koide3/ndt_omp` commit
`5495fd9214945afcb4b35d5a1da385e405c52bf9`; its SHA-256 is
`7b641c32ec4f30faa302e60aaa89765bb9acf67f3f0feb85f9e4e11e88b4dc9f`.
It builds and passes four focused tests on both Humble and Jazzy. A
canonical-package `scanmatcher` build also passes all 109 tests. The parent has already moved
its registration ownership and casts to the PCL shared-pointer API, so the
post-upstream transition patch is limited to dependency-name replacement and
the upstream spelling of `setOutlierRatio`; that patch has SHA-256
`c090b8f2228b21dcf30650114f9638f38497ca5a0214e3e6063a53aa7bef66b1`.
It covers both direct consumers: two `scanmatcher` build references, seven
`graph_based_slam` build references, and both package manifests. The complete
four-package canonical workspace builds and installs without the fork on the
network-isolated Humble and Jazzy images.
The exact implementation and verification record is in the
[2026-08-12 review audit](evidence/ndt-omp-release-review-2026-08-12.md).
The
[copy-ready upstream PR packet](evidence/growth/canonical-ndt-upstream-pr-packet-2026-08-12.md)
binds the exact local commit, fork branch, Draft PR text, duplicate audit,
current clean Jazzy rerun, and publication sequence. It remains local-only and
authorizes no GitHub write.
Neither patch has been submitted or published.

Before any upstream action, reproduce both exact local identities and current
GitHub state:

```bash
python3 scripts/check_canonical_ndt_convergence.py --json
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout /path/to/clean/koide3-ndt_omp \
  --require-ready-for-upstream-review
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout /path/to/clean/koide3-ndt_omp-at-5495fd9 \
  --candidate-checkout /path/to/clean/ndt_omp-at-618f02f \
  --online \
  --require-ready-for-draft-pr
```

The machine contract is
[`canonical-ndt-convergence-v1.json`](contracts/canonical-ndt-convergence-v1.json).
`READY_FOR_UPSTREAM_REVIEW` is local technical evidence only and never grants
GitHub write authority. `READY_FOR_DRAFT_PR` additionally proves the current
upstream/fork/branch/duplicate state and exact candidate commit, but likewise
does not grant GitHub write authority.

#### Historical NDT 0.1.0 bootstrap commands — do not rerun

The commands below record the already completed first tag and Bloom setup.
They are retained for audit history only. The source tag and release repository
now exist, so the absent-tag assertion intentionally fails. The current action
is reviewer response and dependency convergence, not replaying these commands.

```bash
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
cd ndt_omp_ros2
git fetch --prune origin

RELEASE_COMMIT=8b77fa5a6cdcad45bf35918361c892b6d94a287e
test "$(git rev-parse origin/humble)" = "$RELEASE_COMMIT"
test -z "$(git status --porcelain)"
test "$(python3 -c \
  'import xml.etree.ElementTree as E; print(E.parse("package.xml").findtext("version"))')" \
  = "0.1.0"
test -z "$(git ls-remote --tags origin refs/tags/0.1.0)"

git tag -a 0.1.0 "$RELEASE_COMMIT" -m "ndt_omp_ros2 0.1.0"
git push origin refs/tags/0.1.0
```

The tag is intentionally **not** `v0.1.0`: the Bloom track below uses the
default `:{version}` release-tag template. Do not tag the parent
`lidarslam_ros2` repository.

Create the public release repository once, without generated files, then give
Bloom the `master` branch it expects:

```bash
gh auth status
gh repo create rsasaki0109/ndt_omp_ros2-release \
  --public \
  --description "Bloom release repository for ndt_omp_ros2"
gh auth setup-git

RELEASE_REPO_DIR="$(mktemp -d)/ndt_omp_ros2-release"
git clone https://github.com/rsasaki0109/ndt_omp_ros2-release.git \
  "$RELEASE_REPO_DIR"
git -C "$RELEASE_REPO_DIR" commit --allow-empty -m "Initialize release repository"
git -C "$RELEASE_REPO_DIR" branch -M master
git -C "$RELEASE_REPO_DIR" push -u origin master
```

Install Bloom from the target ROS/Ubuntu package repository, then release
Humble first and Jazzy second:

```bash
sudo apt update
sudo apt install python3-bloom python3-catkin-pkg python3-rosdep

RELEASE_REPO=https://github.com/rsasaki0109/ndt_omp_ros2-release.git
bloom-release ndt_omp_ros2 \
  --rosdistro humble \
  --track humble \
  --new-track \
  --override-release-repository-url "$RELEASE_REPO"

bloom-release ndt_omp_ros2 \
  --rosdistro jazzy \
  --track jazzy \
  --new-track \
  --override-release-repository-url "$RELEASE_REPO"
```

The command options have distinct jobs:

| Option | Why it is present |
|---|---|
| `ndt_omp_ros2` | rosdistro **repository** key; it also matches the single package name here |
| `--rosdistro <distro>` | selects the Humble or Jazzy distribution file and Debian target |
| `--track <distro>` | selects the independently maintained Bloom track |
| `--new-track` | creates that track on its first release; omit it on later releases |
| `--override-release-repository-url` | supplies the release repository before a rosdistro entry exists |

Use these first-run track answers; press Enter for the values shown as
defaults:

| Prompt | Answer |
|---|---|
| repository name | `ndt_omp_ros2` |
| upstream repository | `https://github.com/rsasaki0109/ndt_omp_ros2.git` |
| upstream type | `git` |
| version | `:{auto}` |
| release tag | `:{version}` |
| upstream devel branch | `humble` |
| ROS distro | `humble` or `jazzy`, matching the command |
| patches directory | empty / `None` |
| release repository push URL | empty; the HTTPS origin is already authenticated by `gh auth setup-git` |

Bloom pushes generated branches and tags to the release repository and then
offers the corresponding `ros/rosdistro` PR. Verify that each generated PR
only adds `ndt_omp_ros2`, uses the release URL above, and reports version
`0.1.0-1`. Do not use `--non-interactive` for this first release. If either
command fails, fix the cause and rerun it; do not accept Bloom's offered
force-push without first inspecting the release repository.

### rko_lio binary compatibility is proven for the golden path

The RKO-LIO flagship launch (`rko_lio_slam.launch.py`) uses the `rko_lio`
package at runtime. `lidarslam/package.xml` now declares
`<exec_depend version_gte="0.3.2">rko_lio</exec_depend>`, so an installed
flagship path cannot silently resolve to the older main-repository builds.

An official `rko_lio` release line and binary packages are now present for
both supported distributions, with the versions differing between testing
and main as recorded above. The source checkout pins fork merge commit
`79d71e8`, whose parents are the previous maintained fork (`33402d4`) and
official PRBonn v0.3.2 (`48e12f9`). Its package metadata is therefore aligned
at `0.3.2` while retaining the fork's offline-completion, recovery,
diagnostics, and opt-in research changes. The
`test_bundled_dependency_version_alignment` CI gate rejects a future bundled
package version below the `lidarslam/package.xml` runtime minimum.

The successful official-binary gate still proves only the maintained
MID-360 golden path against the unmodified official package. It does not
claim equivalence for fork-only research features, and the fork must not be
bloomed under the already-owned `rko_lio` package name.

The resolution gate completed in
[public run 30412938777](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30412938777):

1. Run `.github/workflows/official-rko-binary-compatibility.yml`. It builds
   the release-shaped source tree without the RKO-LIO submodule, installs the
   official `ros2-testing-apt-source` configuration and the exact `0.3.2-1`
   testing candidate, builds production packages with source-only fork
   research tests disabled, proves `offline_node` resolves from
   `/opt/ros/<distro>`, and runs the pinned MID-360 E2E independently on
   Humble and Jazzy. The workflow records the full Debian build version and
   executable SHA-256 in its non-geometry evidence artifact.
2. Both passed all 18 E2E checks. The permanent non-geometry record is
   [`official-rko-binary-compatibility-2026-07-29.md`](evidence/official-rko-binary-compatibility-2026-07-29.md).
   `lidarslam` therefore requires `rko_lio >= 0.3.2`; fork-only features
   remain evaluation scope.
3. If a scheduled rerun regresses, upstream the minimum product fixes to
   `PRBonn/rko_lio` or give the maintained fork a non-conflicting package
   identity before releasing it. Do not silently substitute untested source
   code.

Do not advertise `sudo apt install ros-<distro>-lidarslam` as the golden path
until `ndt_omp_ros2` is released, Jazzy `rko_lio 0.3.2` is synced to main, and
the package-manager installation E2E gate passes.

The compatibility workflow runs when its product boundary changes on
`develop`, is scheduled weekly to detect repository drift, and can be
dispatched manually. It intentionally uses the maintained MID-360 preset:
`lidarslam/param/rko_lio_mid360.yaml` only uses parameters implemented by the
official `0.3.2` line. Research presets containing fork-only recovery,
diagnostic, radar, visual, or gravity parameters are outside this gate. The
NTU-VIRAL preset is also outside the first adoption gate because it currently
contains fork-only timestamp-offset and ICP keypoint controls.

## Distro targets

Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04). Both are already exercised by
the CI matrix on every push, including the `ndt_omp_ros2` submodule build, so
no new build risk is expected on the farm. All ament tests run on synthetic
fixtures (no datasets, no network, no GPU), which matches buildfarm
constraints.

## Release procedure (maintainer)

One-time setup:

```bash
pip3 install -U bloom catkin_pkg
# a GitHub token with repo scope, configured for bloom:
#   https://bloom.readthedocs.io/ -> "Automated PR opening"
```

Per release:

```bash
# 0. ensure develop is green and this repo's release prep is merged
# 1. tag the release commit (release.yml publishes the GitHub release)
VERSION="$(tr -d '\n' < VERSION)"
git tag "v${VERSION}" && git push origin "v${VERSION}"

# 2. create an empty release repo once (first release only):
#    https://github.com/rsasaki0109/lidar_slam_ros2-release

# 3. release into each distro (first run is interactive; answers below)
bloom-release --rosdistro humble lidarslam_ros2
bloom-release --rosdistro jazzy lidarslam_ros2
```

First-run interactive answers:

| Prompt | Answer |
|---|---|
| repository name | `lidarslam_ros2` |
| upstream repository | `https://github.com/rsasaki0109/lidar_slam_ros2.git` |
| upstream type | `git` |
| upstream branch | `develop` |
| version | `:{auto}` (reads package.xml) |
| release tag | `v:{version}` (upstream tags are v-prefixed; matches the `v*` trigger in `release.yml`) |
| release repository | `https://github.com/rsasaki0109/lidar_slam_ros2-release.git` |

bloom ends by offering to open the `ros/rosdistro` PR with the configured
GitHub token — submit it, answer review comments (license string, maintainer
email, description quality are the usual ones), and wait for the buildfarm.
Binaries appear in the ROS testing repo first, then sync to main with the
next distro sync (typically 2–6 weeks).

## Package-manager evidence window

When the new lidarslam version appears in ROS testing, run the clean
package-manager path for both supported distributions:

```bash
python3 scripts/check_ros_apt_dependency_readiness.py --require testing
```

Do not dispatch the dataset-backed workflow until this dependency-only
preflight reports `TESTING_READY` or `MAIN_READY`.

```bash
gh workflow run package-manager-install-upgrade.yml \
  -f source_ref=v0.9.1 \
  -f target_version=0.9.1 \
  -f target_channel=testing \
  -f mode=clean-install
```

For every release after the first, also capture the upgrade while the previous
version remains in main and the new version is in testing:

```bash
gh workflow run package-manager-install-upgrade.yml \
  -f source_ref=v0.9.1 \
  -f target_version=0.9.1 \
  -f target_channel=testing \
  -f mode=upgrade \
  -f baseline_version=0.9.0
```

Replace the example versions with the exact immutable source tag and apt
versions under review. The workflow rejects a `source_ref` whose root
`VERSION` differs from `target_version`, missing exact Debian candidates, an
old or failed baseline report, dependency versions below the product
minimums, stale paths, or any installed CLI/real-map failure. Download and
retain both Humble and Jazzy non-geometry artifacts before the testing
candidate syncs.

After the target reaches main, rerun `clean-install` with
`target_channel=main`. Do not add the apt command to the beginner
documentation until both main-channel jobs pass. The schema-backed verifier
and workflow are described in [Distribution and installed CLI](distribution.md#ros-apt-install-and-upgrade-gate).

## After the first sync

- README: add the `sudo apt install ros-humble-lidarslam` install path next
  to the source build.
- Subsequent releases: update each package's `CHANGELOG.rst`, bump all four
  `package.xml` versions together with `VERSION` (see `RELEASING.md`), tag,
  and re-run `bloom-release` (non-interactive after the first time).

## Known caveats

- `lidarslam/images/` demo media (~25 MB of GIF/mp4/png) ships inside the
  source package because the README references those paths. Acceptable for
  now; moving demo media out of the package directory is a possible later
  cleanup to slim the source deb.
- The repository-level `CHANGELOG.md` is the human-facing project changelog;
  the per-package `CHANGELOG.rst` files are what bloom turns into deb
  changelogs (REP-132). Both need updating per release.
- `git archive` drops submodules, so the upstream tarball bloom imports
  contains no `Thirdparty/` sources. Do not move release-relevant code into
  `Thirdparty/`.
