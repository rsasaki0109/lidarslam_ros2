# Contributor Python test entrypoint — 2026-08-11

> Status: **LOCAL_DUAL_DISTRO_PASS / PUBLICATION_PENDING**
>
> Implementation revision:
> `96d0763155617331f8bf4d4ac6066aca8a30a055`
>
> Final validation revision:
> `e2a4dfc6eacd06901fcfb54a0214922db88ab82f`
>
> Remote mutations performed: **none**

## Outcome

A contributor can now run the maintained Python product suites from one
documented command:

```bash
bash scripts/run_product_python_tests.sh
```

The entrypoint passed the complete `graph_based_slam` and `lidarslam` Python
suites on both supported ROS distributions at the final validation revision.
It is a local contributor-experience improvement, not evidence of an external
contribution, public CI success, or completion of the G3 ecosystem gate.

## Problem reproduced before implementation

The repository did not have one safe root-level Python test command. Four
distinct failure modes were reproduced:

1. repository-root `pytest` also collected optional `Thirdparty/rko_lio`
   tests even when that package was not installed;
2. collecting `graph_based_slam/test` and `lidarslam/test` in one pytest
   process collided on the legacy duplicate basename
   `test_rko_lio_graph_benchmark_script.py`;
3. pytest's `--import-mode=importlib` avoided that collision but broke an
   existing same-directory helper import; and
4. an unsourced graph suite reached collection without `rosbag2_py`, then
   failed with import errors instead of an actionable setup message.

The supported execution was already two package-scoped pytest processes after
sourcing Humble or Jazzy. The new script makes that implicit contract explicit
without collecting optional third-party tests.

## Entrypoint contract

`scripts/run_product_python_tests.sh` now:

- accepts `--suite all|graph_based_slam|lidarslam`, with `all` as the default;
- accepts a supported environment through `--ros-setup PATH` and otherwise
  discovers Jazzy or Humble when the graph suite needs ROS Python bindings;
- rejects a graph run under a declared ROS distribution outside the Humble and
  Jazzy support contract;
- checks pytest, `rosbag2_py`, and the required product-test Python imports
  before collection;
- gives the exact CI-pinned
  `python3 -m pip install 'rosbags==0.11.0'` recovery command when the graph
  suite lacks that pip-only development extra;
- runs the two maintained package directories in separate pytest processes;
- disables bytecode and pytest cache writes and forwards focused arguments
  supplied after `--`; and
- runs the second selected suite after a test failure, then reports every
  failed suite in one final result.

`CONTRIBUTING.md` documents the command and focused forms. The script contract
has a dedicated 10-test regression file registered in
`graph_based_slam/CMakeLists.txt`, and the main workflow checks the shell
syntax. The existing supported CI matrix already installs
`rosbags==0.11.0` and executes package tests separately through colcon.

## Dual-distribution execution

Both rows used a clean source tree at
`e2a4dfc6eacd06901fcfb54a0214922db88ab82f`; the Humble source and Git metadata
were additionally mounted read-only. The command selected the ROS setup itself
after the inherited ROS variables were removed.

| Environment | Graph suite | lidarslam suite | Entrypoint result |
| --- | ---: | ---: | --- |
| Humble immutable image plus ephemeral development dependencies | 1,382 passed, 37 skipped | 484 passed | `PASS: all` |
| Jazzy host, `/opt/ros/jazzy/setup.bash` | 1,406 passed, 13 skipped, 11 pre-existing ImageIO warnings | 484 passed | `PASS: all` |

The Humble base was the immutable image
`ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447`.
Its curated runtime intentionally contains neither `pip` nor `rosbags`. A
network-isolated first attempt therefore stopped before pytest collection with
exit code 2, named `rosbags`, and printed the pinned install command. For the
full development-suite row, `python3-pip` and `rosbags==0.11.0` were installed
only inside a disposable container. Source and linked-worktree Git metadata
were mounted read-only, and the container was removed after the run.

The first Humble development run also exposed a real test-fixture portability
assumption: the synthetic standard rosbag test unconditionally dropped the
Jazzy-era `message_definitions` table even though Humble's database did not
contain it. Final revision `e2a4dfc` uses `DROP TABLE IF EXISTS`, preserving
the intended no-embedded-definition fixture on both database schemas. A
separate initial Git-provenance failure was traced to incomplete read-only
linked-worktree mounts and was not a product or ROS failure.

## Supporting checks

| Check | Exact result |
| --- | --- |
| Entrypoint contract regression | 10 passed |
| Entrypoint, default-CI, and docs-entrypoint regressions | 30 passed |
| Focused Jazzy standard-rosbag compatibility regression | 1 passed, 1,418 deselected |
| `bash -n scripts/run_product_python_tests.sh` | PASS |
| Python critical flake8 selection `E9,F63,F7,F82` | PASS |
| Workflow YAML parse | PASS |
| `git diff --check` | PASS |
| `mkdocs build --strict` | PASS with pre-existing Material and nav notices |

`shellcheck` was not available on the validation host, so no shellcheck result
is claimed.

## Growth interpretation and next gate

This closes one local setup-ambiguity finding in the contribution funnel: the
maintainer can now give every prepared starter-task contributor the same full
or focused Python check route. It does not yet prove the roadmap's prepared
environment target of at most 30 minutes, because no external contributor
timing was collected and no starter issue was published.

The next evidence transition is:

1. obtain E1 authorization for the exact reviewed source tip;
2. run the public Humble/Jazzy CI matrix on that public revision;
3. if separately authorized under E3, publish a bounded starter task that uses
   this focused-check route; and
4. record prepared-environment active time, outcome, and setup blockers without
   retaining contributor identity or private logs.

E1 source publication and E3 community mutation remain separate decisions.

## 2026-08-12 follow-up

The original revision accurately required `rosbag2_py` only for the graph
suite. Later product work added rosbag2-backed sensor-setup fixtures under
`lidarslam/test`, making that assumption stale. The current local candidate now
auto-sources supported ROS and checks `rosbag2_py` for either suite. An
unsourced `--suite lidarslam` run passed all 622 tests after that repair. The
[follow-up evidence](contributor-product-surface-sync-2026-08-12.md) records the
initial `612 passed / 10 failed` reproduction, updated runner contract, and
current focused checks. Its final unsourced `--suite all` gate passed 1,428
graph tests and 622 lidarslam tests after also catching and repairing one new
interactive-home CTest registration omission. This note does not alter the
historical dual-distro numbers above.
