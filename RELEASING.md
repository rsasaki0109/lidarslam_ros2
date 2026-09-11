# Releasing

The repository root `VERSION` file is the release version source of truth.
Versions below `0.9.0` publish as GitHub prereleases. v0.9.x is the stable
release-candidate line and publishes as a normal GitHub Release; it is not a
claim that the separate v1.0 readiness gate is complete.

## Release Scope

The intended public release scope is:

- default permissive-license workflow
- `RKO-LIO + graph_based_slam`
- Autoware-compatible map bundle generation (pointcloud_map / projector info /
  lanelet2)
- benchmark summary / HTML report / release-readiness gate

## Pre-Release Checklist

1. Clean the worktree so generated outputs do not leak into the release commit.
2. Run local checks:

```bash
VERSION="$(tr -d '\n' < VERSION)"
bash scripts/run_default_ci_checks.sh
./scripts/run_product_python_tests.sh
python3 -m mkdocs build --strict
python3 scripts/check_first_map_verification_package.py --json
python3 scripts/check_release_bundle_reproducibility.py \
  "/tmp/lidarslam_ros2_v${VERSION}_release_candidate.tar.gz"
bash scripts/run_release_readiness_checks.sh --skip-default-ci --fail-on-profiles
source install/setup.bash
lidarslam-map doctor
DEMO_WORK_DIR="/tmp/lidarslam_ros2_v${VERSION}_first_map"
test ! -e "${DEMO_WORK_DIR}"
lidarslam-map demo "${DEMO_WORK_DIR}" --viewer none
```

The fixed `lidarslam-map demo` is the release first-map gate. The older NTU
VIRAL Autoware viewer/dogfood quickstart remains an advanced compatibility
check and must not substitute for the canonical product route.

The first-map verification-package audit is a local, fail-closed check of the
Docker/source instructions, receipt handoff, runtime guards, documentation
markers, CI contract, and curated release-bundle inventory. It must report
`READY` before publication; it does not itself prove that the public docs or
container tags have been published. After publication, use the read-only
public documentation and release audits below to establish that handoff.

The bundle rehearsal requires a clean source worktree, derives `v<VERSION>`
and the exact current commit, builds the curated archive twice, runs the same
complete manifest/hash verification used by the post-publication audit, and
only then writes the requested candidate artifact. It refuses to overwrite an
existing file. Main CI runs it before creating synthetic benchmark fixtures.

The benchmark command is fail-closed: `--ape-threshold` and
`--fail-on-profiles` require at least one `metrics.json` below
`--benchmark-root`. An empty benchmark root is a release failure, and
`--skip-benchmark-summary` cannot be used to bypass either hard gate. Generate
or restore the release-candidate benchmark evidence before running this check;
every blocking release profile must have matching data and pass its own
threshold. CI's synthetic threshold fixture tests the gate plumbing but cannot
satisfy the release profiles. Omitting the hard-gate options leaves an empty
root report-only. When a blocking row is `NO_DATA` or `FAIL`, the command
prints the profile's tracked remediation path; follow that path rather than
weakening the threshold or copying evidence from an older commit.

An empty root under `--fail-on-profiles` still writes
`benchmark_summary.md`, `benchmark_summary.csv`, and
`benchmark_summary.log`. The summary evaluates every profile as `NO_DATA`,
separates blocking rows from report-only rows, and prints each blocking row's
acquisition or rerun instruction before exiting 2. Start with that retained
report instead of reconstructing the required dataset list by hand.

3. Push the branch and verify GitHub Actions are green.
   Also inspect the cross-phase product audit; release candidates may remain
   `NOT_READY`, but an invalid contract must stop the release:

```bash
python3 scripts/check_v1_readiness.py --json
```

If the distribution gate is still open on `ndt_omp_ros2`, inspect it without
mutating GitHub or rosdistro:

```bash
python3 scripts/check_ndt_omp_release_readiness.py
```

`LOCAL_READY` is the offline CI result, not remote publication proof.
`READY_TO_TAG` authorizes the initial source-tag and Bloom procedure.
The current 0.1.0 publication is `IN_PROGRESS`; do not recreate its immutable
tag or rerun Bloom while the generated Humble and Jazzy rosdistro PRs remain
current. Only `RELEASED` satisfies this part of the distribution gate.

After Bloom packages enter ROS testing, run
`.github/workflows/package-manager-install-upgrade.yml` for both Humble and
Jazzy. Capture clean-install evidence and, when an older main version exists,
main-to-testing upgrade evidence before the sync. After sync, repeat
clean-install against `main`; see `docs/rosdistro-release.md` for exact
dispatch inputs.

Avoid a guaranteed-failing workflow dispatch by checking the public dependency
channels first:

```bash
python3 scripts/check_ros_apt_dependency_readiness.py --require testing
```

The live v1 gate accepts the normal apt path only after the exact main-channel
clean-install run succeeds for both named distributions:

```bash
python3 scripts/check_package_manager_release_readiness.py \
  --version "$(tr -d '\n' < VERSION)" \
  --require-ready
```

4. Set `VERSION="$(tr -d '\n' < VERSION)"` and confirm `CHANGELOG.md`, the
   per-package `CHANGELOG.rst` files, `docs/comparison.md`,
   `docs/releases/v${VERSION}.md`, `CITATION.cff`, and the core package versions
   match (`test_release_metadata_and_core_package_versions_match` enforces most
   of this). Replace the candidate-only banner and `Release decision: HOLD`
   section with `Release verification` in the final verified notes; the tag
   workflow rejects either candidate marker before any image or release
   publication starts, and the docs contract rejects a marker-free note that
   lacks the final verification section.
5. Review README, `docs/getting-started.md`,
   `docs/autoware-map-authoring.md`, `docs/product-contract.md`,
   `docs/benchmarking.md`, `docs/comparison.md`, and `CONTRIBUTING.md` for
   operator-facing accuracy.
6. Confirm the tagged checkout can build both `ROS_DISTRO=humble` and
   `ROS_DISTRO=jazzy` Docker targets. The release workflow will refuse to
   create the GitHub Release unless both published digests pass the installed
   `lidarslam-map --version` smoke test.

For a v1.0 release, require the complete product gate. This includes the
tracked independent-user ledger rather than checking it as an isolated proxy:

```bash
python3 scripts/check_v1_readiness.py --require-complete
```

Prerelease candidates validate the ledger without pretending that 0/3 is
complete:

```bash
python3 scripts/check_external_first_map_readiness.py --json
```

Build the exact curated bundle before creating a new tag:

```bash
VERSION="$(tr -d '\n' < VERSION)"
python3 scripts/build_release_bundle.py \
  --tag "v${VERSION}" \
  --candidate \
  --output "/tmp/lidarslam_ros2_v${VERSION}_release_bundle.tar.gz"
```

The command requires a clean worktree, refuses an existing tag that names a
different commit, refuses to overwrite its output, and embeds a
schema-validated SHA-256 inventory. Repeating it from the same commit produces
the same bundle bytes.

The pre-release checklist's reproducibility command is the preferred
maintainer path because it performs both builds and the full re-verification
in one fail-closed operation. The lower-level command above remains the exact
single-build primitive used by the tagged release workflow.

Before building the bundle, run the hard benchmark gate from the same clean
commit:

```bash
bash scripts/run_release_readiness_checks.sh --fail-on-profiles
```

The wrapper binds every blocking release profile to the exact current `HEAD`.
Evidence produced by an older clean revision is rejected as `NO_DATA`, so a
release candidate must be benchmarked again after its final code commit.
Report-only profiles remain cross-revision historical comparisons.

## Automated Publication

Two GitHub Actions workflows matter for release:

- `.github/workflows/main.yml` runs the continuing CI matrix, release-readiness
  fixture checks, and weekly scheduled validation.
- `.github/workflows/release.yml` validates the tag, publishes and attests
  untagged Humble and Jazzy candidate digests, smoke-tests both by registry
  digest, verifies their source revision, SBOM, BuildKit provenance, and
  GitHub attestation, then promotes the pair to
  `v<version>-humble` and `v<version>-jazzy`. Promotion preflights both
  digests before creating either tag. A matching existing tag is reused;
  a different digest fails closed and is never overwritten. The workflow then
  publishes the GitHub Release using
  `docs/releases/v<version>.md` as the release body. The release assets include
  the source bundle plus one `release-image-<distro>.json` installation
  evidence file and one digest-pinned `rollback-plan-<distro>.json` per image,
  plus `release-promotion.json`.
  Retain the prior release's JSON assets as the last-known-good recovery
  record; do not move a tag to perform rollback.

After creating the GitHub Release, the workflow runs an independent,
read-only publication audit:

```bash
python3 scripts/check_published_release.py --require-published
```

It requires a non-draft, non-prerelease v0.9 release, resolves the tag commit,
downloads exactly the six release assets, validates every JSON schema and
cross-file identity, and verifies the embedded bundle manifest against every
archived file size and SHA-256. It also resolves the live Humble and Jazzy
version tags from GHCR and requires both to still name the digests recorded in
the attached release-image evidence. HTTP/API or registry failures are
`BLOCKED`; only explicit tag/release 404 responses mean `NOT_PUBLISHED`. Retain the uploaded
`published-release-audit.json` Actions artifact with the release evidence.

The curated bundle contains `release-bundle-manifest-v1.json`, including the
exact tag commit and every bundled file hash. The image build emits an OCI
SBOM and maximum-mode BuildKit provenance. GitHub artifact attestations cover
each image digest and the source release bundle. Verify an image after
publication:

```bash
lidarslam-map rollback-plan release-image-jazzy.json
```

## Tagging

```bash
VERSION="$(tr -d '\n' < VERSION)"
test -f "docs/releases/v${VERSION}.md"
git tag "v${VERSION}"
git push <remote> "v${VERSION}"
```

## Binary release to rosdistro (bloom)

The apt/buildfarm release procedure — dependency analysis, the
`ndt_omp_ros2`-first ordering, and the exact `bloom-release` answers — lives
in [`docs/rosdistro-release.md`](docs/rosdistro-release.md). Repository-side
prep (aligned versions, SPDX license tags, per-package `CHANGELOG.rst`) is
part of the normal pre-release checklist above; keep the `CHANGELOG.rst`
files updated with each version bump.

## Suggested Release Notes

Include at least:

- the default supported workflow
- the Autoware map-bundle scope
- the current comparison page and benchmark snapshot
- the benchmark / release-readiness artifacts
- current limitations, especially around lanelets and full production support
