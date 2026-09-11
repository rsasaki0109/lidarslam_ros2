# Contributor product-surface synchronization — 2026-08-12

> Status: **LOCAL_PASS / PUBLICATION_PENDING**
>
> Remote mutations performed: **none**

## Finding

`CONTRIBUTING.md` had retained a historical list of three “official” beginner
entrypoints. It directed a new contributor toward the older own-bag shell
wrapper and the NTU VIRAL source quickstart, while the current product contract
defines four workflows around `lidarslam-map`: fixed demo, own-bag mapping,
retained sessions, and privacy-bounded support handoff.

All named historical scripts still exist, so link-existence tests could not
detect this semantic drift. A contributor changing user-visible behavior could
therefore follow a working but non-canonical interface and unintentionally
expand the supported surface.

## Repair

The contributor guide now names exactly the four current workflows and their
canonical commands. It also explains that the no-argument interactive home is
a smaller front door to existing workflows, not a fifth workflow. Historical
wrappers, direct launches, dataset-specific quickstarts, benchmarks, and
research scripts are explicitly advanced or compatibility interfaces unless
the product contract promotes them.

The docs-entrypoint regression now checks that:

- all four current workflow command spellings appear in `CONTRIBUTING.md`;
- the focused `run_product_python_tests.sh` path remains discoverable;
- the no-argument home is described without changing workflow count; and
- the old three-entrypoint and old official own-bag wording do not return.

Verification then exposed a second, executable drift. The contributor runner
auto-sourced ROS only for `graph_based_slam`, but current
`lidarslam/test/test_sensor_setup_wizard.py` also builds rosbag2 fixtures with
`rosbag2_py`. Running the documented `--suite lidarslam` route from an
unsourced shell therefore produced `612 passed, 10 failed` instead of an
actionable preflight or a passing suite.

`run_product_python_tests.sh` now treats both maintained suites as ROS-backed.
It auto-discovers Humble or Jazzy for either suite, checks `rosbag2_py` before
collection, and rejects an unsupported declared ROS distribution consistently.
Its fake-environment contract test was changed from asserting that lidarslam
did not need ROS to proving that a missing binding stops before pytest.

The first canonical `--suite all` run then exposed a third drift at the test
registration boundary. The graph suite's invariant test reported that the new
`lidarslam/test/test_cli_interactive_home.py` file was not registered with
CTest. The runner still continued to the second package as designed, producing
`1,427 passed, 13 skipped, 1 failed` for graph and `622 passed` for lidarslam
before returning failure. Adding `test_cli_interactive_home` to
`LIDARSLAM_ADDITIONAL_PYTESTS` closes that gap, so the same public command now
checks the new beginner-facing surface and its CTest discoverability together.

## Verification

| Check | Result |
| --- | --- |
| docs-entrypoint regression | `15 passed` |
| contributor focused lidarslam command | `7 passed, 615 deselected` |
| contributor runner contract regression | `10 passed` |
| contributor focused graph command after repair | `11 passed, 1,430 deselected` |
| unsourced-shell contributor lidarslam command after repair | `622 passed` |
| initial unsourced-shell `--suite all` discovery run | graph: `1,427 passed, 13 skipped, 1 failed`; lidarslam: `622 passed`; overall FAIL |
| focused CTest-registration regression after repair | `1 passed, 6 deselected` |
| final unsourced-shell `--suite all` gate | graph: `1,428 passed, 13 skipped, 11 warnings`; lidarslam: `622 passed`; `PASS: all` |
| `ament_flake8 graph_based_slam/test/test_docs_entrypoints.py` | PASS |
| `ament_flake8 graph_based_slam/test/test_product_python_tests_script.py` | PASS |
| `bash -n scripts/run_product_python_tests.sh` | PASS |
| `mkdocs build --strict` | PASS with existing Material/nav notices |
| `git diff --check` | PASS |

The full and focused commands used the same public contributor entrypoint
documented in `CONTRIBUTING.md`, rather than relying only on a
maintainer-specific pytest invocation. The final full run removed ROS
environment variables first, observed automatic Jazzy selection, and passed
2,050 product tests across the two package-scoped processes, with 13 graph
tests skipped. It also preserved the runner's fail-after-both-suites behavior
during the initial CTest-registration failure, giving contributors one
complete result instead of stopping after the first package.

## Limits and next gate

This closes the local documentation, dependency-preflight, and CTest
registration drift; it does not prove that an external contributor finishes in
30 minutes. That metric still requires an authorized, published starter issue,
a publicly resolvable candidate revision, and a privacy-bounded
prepared-environment timing record. No issue, label, pull request, repository
setting, or public documentation was changed here.
