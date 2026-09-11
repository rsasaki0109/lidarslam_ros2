# Source onboarding public preflight — 2026-08-12

> Status: **PUBLIC_ROUTE_READY / CLEAN_VM_ROWS_PENDING**
>
> Preflight remote mutations performed: **none**

## Finding

The source quickstart said it built six repository packages, but its build used
an unrestricted repository discovery result. It did not compare the discovered
names with the maintained set or pass an explicit selection to the build. An
accidentally added experimental package could therefore expand beginner build
time and dependency failures while the plan still promised six packages.

The disposable-host observer also checked only that a public commit contained
the quickstart invocation, Getting Started invocation, and matching `VERSION`.
It did not prove that the public helper retained the fast tests-disabled build,
repository-only dependency route, or exact package scope. The manual runbook
made a stricter check, but searched Getting Started for
`-DBUILD_TESTING=OFF` while the page intentionally documents
`BUILD_TESTING=OFF`; that copied preflight would reject the current contract.

## Repair

`source_quickstart.sh` now owns one ordered package inventory:

```text
graph_based_slam
lidarslam
lidarslam_msgs
ndt_omp_ros2
rko_lio
scanmatcher
```

After pinned submodules and build tools are available, but before rosdep or
compilation, the helper runs `colcon list --base-paths ... --names-only`, sorts
the result, and requires exact equality. It then passes the same ordered array
to `colcon build --packages-select`. Missing or extra packages fail with
`[source-package-inventory-mismatch]`; dependency resolution, build, and demo
are not started. The source observer recognizes that private log marker and
stores the same stable finding in its privacy-bounded trial record.

The observer also provides:

```bash
python3 scripts/run_source_onboarding_probe.py \
  --public-preflight \
  --source-commit <40-lowercase-hex-commit> \
  --product-version <matching-version>
```

This mode requires no trial VM, ROS installation, output path, privilege
acknowledgement, or active-time choice. It reads the public GitHub API and
returns JSON only:

- `READY`, exit `0`: the exact commit and complete route contract pass;
- `NOT_READY`, exit `1`: the immutable route is absent or incomplete; or
- exit `2`: API, decoding, or observer failure, which must not be translated
  into a false absence claim.

For one immutable commit, it checks commit identity, the exact shell package
array, explicit package selection and mismatch marker, repository-only rosdep
helper, tests-disabled build, demo delegation, canonical Getting Started
instructions, and `VERSION`. It sets `sys.dont_write_bytecode` before importing
adjacent helpers; public preflight therefore does not create cache files in the
observer checkout.

## Public checks

The first check used public commit
`3f4dd70cdc58ad421192559213cdee0bdc41eba8`, before the source route was
published. Running public preflight for version `0.9.0` correctly returned:

```text
status=NOT_READY
exit=1
finding=source-route-contract-missing
detail=public commit lacks scripts/source_quickstart.sh
writes_performed=false
```

That was the expected fail-closed result. The observer did not substitute the
private checkout, a local clone URL, `develop`, or a moving tag. A before/after
snapshot of all `scripts/**/__pycache__`, `.pyc`, and `.pyo` paths, sizes, and
mtimes remained identical.

After the reviewed route became publicly resolvable, the same network-read-only
observer was rerun against exact commit
`549ef03017c776f23fc968881b346aa685356274` and matching product version
`0.9.1`. It returned `READY` with the six expected packages, all four contract
files, no finding codes, exit `0`, and `writes_performed=false`. This exact
commit is the immutable source-route identity used by the current Humble and
Jazzy source records; a branch name or newer moving head must not replace it in
a record.

The two measured source trials subsequently completed on 2026-08-13 using
clean disposable ROS runtime images. Both produced a valid product `PASS`
with a verified map and receipt; the records are
`docs/evidence/onboarding/g0-source-humble-20260813-runtime-a.json` and
`docs/evidence/onboarding/g0-source-jazzy-20260813-runtime-a.json`. They remain
non-comparable because the observer intentionally recorded no human active
time (`active_operator_time_sec=null`) and the runtime images already supplied
ROS and build dependencies. This is progress on route reliability, not a
claim that the cold clean-host activation gate has passed.

## Verification

| Check | Result |
| --- | --- |
| exact six-package plan and explicit selection, Jazzy host | PASS; no quickstart writes |
| same plan, fixed Humble image, source read-only, network disabled | PASS; no quickstart writes |
| extra-package failure before dependency/build | PASS |
| stable mismatch finding across private-log boundary | PASS |
| public route contract, READY/NOT_READY, parser and read-only regressions | PASS |
| focused source quickstart/observer regressions | `26 passed` |
| combined docs/source regressions | `41 passed` |
| changed Python `ament_flake8` | PASS |
| shell syntax | PASS |
| MkDocs strict build | PASS with existing Material/nav notices |
| canonical unsourced Python gate | graph `1,428 passed, 13 skipped, 11 warnings`; lidarslam `631 passed`; `2,059` total |

The final cumulative milestone receipt binds this gate and the patch identity.

## Limits and next gate

This proves selection, public route availability, preflight semantics,
observer safety, and two successful source-route executions. It does not
measure human active time or install dependencies on a cold clean VM, so the
records are valid but not comparable onboarding baselines.

The next step is to provision dedicated disposable Humble and Jazzy source-
trial VMs pinned to
`549ef03017c776f23fc968881b346aa685356274`, add a human observer, and replace
the provisional source rows only when the route satisfies the existing
wall-time, active-time, command-count, download, peak-disk, output, verifier,
and receipt contract. The G0 activation gate remains closed until at least one
Docker PASS and one source PASS are comparable.

The public-preflight command changed no commit, branch, pull request, issue,
label, release, image, package, review reply, or external repository.

On 2026-08-15, the exact-tip refresh encountered the shared unauthenticated
GitHub API quota. The observer was hardened to accept `GITHUB_TOKEN`, scope it
only to `api.github.com`, and exclude it from every report. An authenticated
read-only rerun against exact pushed tip
`2a56f20ccbd6f7ffbd664d3290d06780d8669c26` then returned `READY`; no file or
external state was changed. A focused regression verifies the request scope.

On 2026-08-13, the complementary release/image identity check for the reviewed
`0.9.1` source candidate returned `NOT_PUBLISHED`: the `v0.9.1` tag and Release
were absent, and both exact `v0.9.1-{humble,jazzy}` GHCR manifests were
unknown. The frozen `v0.9.0` tag was also preflighted as a possible alignment
fallback; it returned `source-route-contract-missing` because that public
commit predates `scripts/source_quickstart.sh`. Consequently the existing
`0.9.0` Docker rows and `0.9.1` source rows remain intentionally separate,
and no local or moving image identity is substituted.
