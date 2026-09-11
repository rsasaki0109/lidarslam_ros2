# Contributing

Thanks for contributing to `lidarslam_ros2`.

This repository is trying to be a practical default for ROS 2 LiDAR SLAM and
Autoware pointcloud-map generation. Contributions are most useful when they are
easy to reproduce, easy to compare, and clearly scoped.

Participation is governed by the [Code of Conduct](CODE_OF_CONDUCT.md).
The [Governance](GOVERNANCE.md), [Support](SUPPORT.md), and
[Security](SECURITY.md) policies explain decisions, help routes, and private
vulnerability reporting.

## What To Send

Useful contributions include:

- bug fixes in the default permissive workflow
- Autoware pointcloud-map integration fixes
- benchmark results with reproducible commands and artifacts
- parameter improvements backed by logs and trajectory metrics
- documentation improvements that reduce setup time

## Default Workflow Policy

The default and recommended workflow in this repository is permissive-license
only. Contributions that change the default path should preserve that property.

Current default path:

- `RKO-LIO + graph_based_slam`
- local Scan Context implementation
- Autoware-compatible pointcloud map output

Research frontends with other licenses can still be discussed, but they should
not silently become the default path.

## Repository Layout

Keep the repository root limited to project metadata and primary entry-point
documents. Place commands under `scripts/`, incident-specific utilities under
`scripts/recovery/`, offline libraries under `tools/`, and small research
outputs under `docs/research/artifacts/`. Do not commit colcon manifests or
other generated build products. See the
[repository layout guide](docs/repository-layout.md) for the complete map.

## Autoware Naming And Trademark Guidance

Keep Autoware references descriptive.

Preferred wording:

- `Autoware-compatible pointcloud map`
- `pointcloud-map workflow for Autoware`
- `works with Autoware`
- `built on Autoware`

Avoid branding this repository or a derived product as if it were an official
Autoware product or a Foundation-approved distribution.

Avoid wording such as:

- `Autoware-ready` as a product tag line
- `official Autoware`
- `Autoware <product-name>`
- `certified by Autoware`
- `endorsed by the Autoware Foundation`

If in doubt, prefer compatibility language over product-name language.

## Before Opening An Issue

Please collect the smallest reproducible case you can.
Use the structured bug, feature, sensor-support, benchmark, or Autoware issue
form that best matches the request. Usage questions must include the diagnostic
bundle listed in [SUPPORT.md](SUPPORT.md).

If you completed a first map from the public docs without live maintainer
guidance, use the
[independent first-map validation](docs/external-first-map-validation.md)
form. Passing reports may count toward the v1.0 three-user gate; failed
attempts are recorded as onboarding findings and are just as valuable.

For bug reports, keep ordinary `doctor <bag> --json` output local because it
contains the bag path and local commands. Instead run and review:

```bash
lidarslam-map doctor <rosbag2_dir> --public-json
```

Paste that complete `public-doctor-evidence-v1` path-free JSON, a redacted
command shape, expected/observed behavior, and either one reviewed support ZIP
from an existing session or an explicit no-session/no-ZIP statement with the
first finding code. Never attach a bag, map, trajectory, raw log/data,
parameter file, terminal history, or private-site image.

For Autoware-related reports, include:

- projector type and whether an origin exists, with every coordinate and
  MGRS/grid identifier and precise origin value replaced by `REDACTED`
- the non-geometry verifier findings, with private paths and precise locations
  removed
- a redacted command shape that keeps executable, options and non-private values
- the reviewed privacy-first support ZIP when a session exists
- whether GNSS was enabled

Do not attach the map bundle, pointcloud or lanelet geometry, rosbag, raw private
logs, or screenshots revealing a private place.

For benchmark-related reports, include:

- the canonical identity, source and license for public data, or a redacted
  sensor/environment/duration summary for a private or custom bag
- a redacted command shape with private values replaced by `REDACTED`
- a tracked/public parameter preset and changed non-private arguments
- key metrics entered directly in the issue form
- whether the generated map passed local Autoware verification
- optionally, one reviewed `metrics.json` or public aggregate report

Never attach a rosbag, map, trajectory, raw log, raw sensor data, private-site
image, exact local path, or complete custom parameter YAML to a public benchmark
issue.

## 30-Minute Starter Path

Start with an open
[`good first issue`](https://github.com/rsasaki0109/lidar_slam_ros2/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22).
A bounded starter issue should name the files, non-goals, acceptance criteria,
and one focused check. You are not expected to download a public dataset or run
the full end-to-end suite unless the issue explicitly says so.

See the current published option, local queue, open-PR duplicate status, and one
next action in a single read-only check (requires an authenticated `gh` CLI):

```bash
python3 scripts/contributor_starter_queue.py --next
```

The card never creates or changes an issue, pull request, or label. It directs a
new contributor only to a currently published `good first issue`; unpublished
local tasks remain behind the maintainer coordination gate. If GitHub is not
available, maintainers can still inspect the next five locally prepared tasks
with:

```bash
python3 scripts/contributor_starter_queue.py --list
python3 scripts/contributor_starter_queue.py --task starter-C5
```

The command is read-only and marks the queue `PREPARED_NOT_PUBLISHED`. A new
contributor should start one of these local tasks only after a maintainer has
published its GitHub issue or explicitly confirmed that the task is still
unclaimed. Before publication, maintainers must rerun the open-PR duplicate
check recorded by the queue contract. The default checker validates all five
scopes, focused-command allowlists, known implementation gaps, and the no-write
authority boundary:

```bash
python3 scripts/contributor_starter_queue.py
```

After changing a published task, run its focused profile with `--verify`, for
example `python3 scripts/contributor_starter_queue.py --verify starter-C5`.
This avoids repository-local MkDocs output for documentation tasks and never
executes an arbitrary command loaded from the JSON contract. A passing focused
profile does not replace review of the issue's acceptance criteria.

For a documentation-only change, the normal focused check is:

```bash
python3 -m mkdocs build --strict
```

For a small Python change, run the exact test and lint scope named by the issue,
for example:

```bash
python3 -m pytest -q path/to/test_file.py -k focused_case
python3 -m flake8 --select=E9,F63,F7,F82 \
  path/to/change.py path/to/test_file.py
git diff --check
```

Some legacy files have pre-existing style debt. A starter task must not expand
into an unrelated whole-file cleanup; new files and newly added lines should
still follow the repository's current style.

If the prepared environment still cannot reach the expected result within 30
minutes, report where the time was spent. That is a contributor-path finding,
not a reason to silently expand the task.

## Recommended Local Checks

For Python-only product changes, use the scoped entrypoint instead of running
`pytest` at the repository root:

```bash
bash scripts/run_product_python_tests.sh
```

It checks the ROS Python prerequisite, excludes optional `Thirdparty` tests,
and runs `graph_based_slam/test` and `lidarslam/test` in separate pytest
processes so their legacy duplicate module basename cannot collide. Use
`--suite lidarslam` or pass a focused pytest expression after `--` when the
change has a smaller declared scope. The preflight reports missing declared
dependencies before collection; the full graph suite additionally uses the
CI-pinned `rosbags==0.11.0` development package.

For code changes that touch the default workflow:

```bash
bash scripts/run_default_ci_checks.sh
```

For benchmark/reporting changes:

```bash
bash scripts/run_release_readiness_checks.sh --skip-default-ci --ape-threshold 0.10
```

For Autoware pointcloud-map changes:

```bash
bash scripts/run_autoware_quickstart.sh
```

## Benchmark Result Submissions

If you want to contribute benchmark results, prefer opening the benchmark report
issue template and include:

- ROS 2 distro and Ubuntu version
- sensor topics and frames
- public dataset identity, sequence, source and license; or only a redacted
  sensor/environment/duration summary for a private or custom bag
- tracked/public parameter preset and changed non-private arguments
- redacted command shape, using literal `REDACTED` placeholders for credentials,
  private paths, host or user names and precise locations
- key metrics from `metrics.json`
- whether the generated map passed Autoware verification

The key metrics belong in the form. You may optionally attach or link one
privacy-reviewed `metrics.json` or public aggregate report. Never attach a bag,
map, trajectory, APE/raw log, raw sensor data, private-site image, output path or
complete custom parameter YAML. Review every linked or attached file first.

## Pull Requests

Please keep PRs narrow and explicit.

- explain the operator-visible change first
- include exact commands used for verification
- mention whether the change affects the default workflow
- call out license implications if any dependency choice changes
- link related benchmark or Autoware issues when relevant

For the current integrated product Draft, contributors may review one bounded
capability lane without volunteering for the entire PR. See
[Product Draft review routing](docs/review-routing.md) and render a local lane
card with `python3 scripts/check_product_draft_review_routing.py --lane <ID>`.
The card collects no reviewer identity and never requests or submits a GitHub
review. After checking a lane, use the same page's
`product_draft_review_ledger.py` flow to append an identity-free local PASS or
BLOCKED event. Keep that ledger outside the repository so recording evidence
does not change the exact commit being reviewed.

## Product Entry Points

The four official beginner-facing product workflows are:

- try the fixed public MID-360 demo with the README Docker command or
  `lidarslam-map demo` after installation;
- map an own compatible rosbag2 with `lidarslam-map start <rosbag2_dir>`, or
  `bash scripts/docker_map_bag.sh <rosbag2_dir>` without a ROS installation;
- return to and compare retained work with `lidarslam-map sessions` and
  `lidarslam-map compare <left> <right>`; and
- prepare a privacy-bounded maintainer or first-map handoff with
  `lidarslam-map support <session_bundle>`.

An installed user who is unsure where to begin can run `lidarslam-map` without
arguments on an interactive terminal. Its small home routes to the existing
demo, own-bag, or session workflow; it is not a fifth workflow. Contributor
changes to user-visible behavior should preserve this bounded surface and the
machine contract in `docs/contracts/cli-v1.json`.

Historical beginner wrappers, direct launch files, dataset-specific
quickstarts, benchmark runners, and research scripts remain available only as
advanced or compatibility interfaces unless the
[Product Contract](docs/product-contract.md) explicitly promotes them.

Useful references:

- Autoware quickstart: [docs/autoware-quickstart.md](docs/autoware-quickstart.md)
- comparison page: [docs/comparison.md](docs/comparison.md)
- benchmarking and release gate: [docs/benchmarking.md](docs/benchmarking.md)
- current release candidate notes:
  [docs/releases/v0.9.0.md](docs/releases/v0.9.0.md)
- product roadmap: [docs/roadmap/v0.9.md](docs/roadmap/v0.9.md)
- benchmark fixture generator: `scripts/generate_sample_benchmark_metrics.py`
