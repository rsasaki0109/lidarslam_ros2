# Source-install direct launcher activation — 2026-08-12

> Status: **LOCAL_DUAL_DISTRO_PASS / PUBLICATION_PENDING**
>
> Remote mutations performed: **none**

## User-facing finding

The source quickstart already detected Humble or Jazzy, installed repository
dependencies, built the six maintained packages, and ran the verified demo.
Its next-terminal instruction still began with
`source <workspace>/install/setup.bash`, however. Calling the absolute installed
`lidarslam-map` path without that remembered activation loaded the curated
Python CLI but did not activate ROS Python or package paths.

This was not always a loud startup failure. A pre-repair installed artifact
returned exit `0` for `doctor` on a two-message PointCloud2+Imu bag, but fell
back from unavailable `rosbag2_py` to a less capable reader. It reported
`pointcloud-inspection-unavailable` and
`timestamp-inspection-unavailable`, selected no maintained profile, and hid a
valid RKO-LIO route. A command that appears to work while losing its safe
recommendation is a worse beginner failure than a clear setup error.

The executable regression captured the same boundary more directly. Before
the repair, merged-install, isolated-install, and source-layout launcher probes
all delegated without the setup marker (`3 failed`).

## Repair

The shared `scripts/lidarslam` launcher now activates the product environment
inside its own process before delegating:

- an installed launcher derives its prefix from the selected curated
  `share/lidarslam/product/scripts/lidarslam_cli.py` resource and accepts only
  that prefix's aggregate `setup.bash` or the immediate isolated-workspace
  aggregate;
- a repo-local launcher considers the documented parent workspace and existing
  local layouts, but accepts one only when its install tree contains the
  matching curated CLI resource and then delegates to that installed resource,
  keeping the activated runtime and command implementation together;
- no arbitrary filesystem search, shell startup-file edit, or unrelated parent
  workspace sourcing is performed;
- relative or absolute symlinks are resolved with a bounded depth before
  locating resources;
- setup failure returns the product's internal/tooling exit code `70` with the
  exact failed setup path; and
- activation changes only the launcher and child environment, never the
  caller's shell; and
- delegated Python processes disable bytecode writes, preserving the installed
  prefix as a read-only product surface.

`source_quickstart.sh` now also rejects a post-build `lidarslam-map` resolved
outside the selected workspace, requires the command to be executable, and
prints its absolute direct path as the primary next-terminal action. Explicit
workspace sourcing remains available for the short command name and other ROS
tools.

The clean installed-product checker now removes inherited ROS, colcon, Python,
and library-path variables for its real-bag `doctor` invocation. Future clean
prefix evidence therefore fails if the absolute launcher cannot reconstruct
its own environment or silently loses the maintained recommendation.
It also snapshots Python cache state across the whole install prefix. CMake
excludes development bytecode from `launch/`, the checker disables bytecode for
its own dynamic imports, and successful validation now fails if any cache path,
size, mtime, or content changes.

## Actual installed execution

Two fresh Jazzy installs were built from the local candidate: an isolated
colcon install and a merged clean prefix. Both builds completed. The isolated
absolute command was then invoked with `AMENT_PREFIX_PATH`,
`CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`, `ROS_DISTRO`, ROS version variables,
`PYTHONPATH`, and `LD_LIBRARY_PATH` removed.

On the synthetic PointCloud2+Imu rosbag2 fixture, the repaired direct command:

- returned exit `0` from `doctor --json`;
- inspected the `timestamp` point field through `rosbag2_py`;
- proved monotonic PointCloud2 and Imu header timestamps;
- selected `rko_lio_graph_public_path`; and
- ran `start --dry-run --viewer none` to the expected calibration-review card,
  returned exit `0`, and wrote no files.

The merged install passed the complete `check_installed_product_cli.py`
contract after its `doctor` step was changed to start from that clean
environment. This includes installed resources, command help, compatibility
shim, demo plan, sessions, recovery, support, migration, and dry-run behavior;
the activation assertion is not a standalone mock-only claim.

### Humble follow-up

The same candidate then passed in the locally available immutable Humble image
`ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447`.
The container used `--network none`, mounted the candidate source read-only,
ran as the host UID/GID, and rebuilt `lidarslam` into a fresh non-symlinked
merged prefix against the image's six-package underlay.

The first build completed but its checker correctly rejected unknown source
identity. This clone uses a Git object alternate outside the mounted source,
so the container could read `HEAD` but could not resolve its objects. The final
build used the documented Git-free packager interface to bind exact revision
`3f4dd70cdc58ad421192559213cdee0bdc41eba8` and `dirty=true`; the installed
`product-build-info.json` reported `source: override`, and the checker required
that exact revision. No clean-revision claim is made for this cumulative local
candidate.

Humble's base `setup.bash` also reproduced a distribution-specific reason for
the launcher's bounded `set +u` section: sourcing it directly under `set -u`
failed on an unset `AMENT_TRACE_SETUP_FILES`. The repaired launcher handled the
same setup from a fresh process without weakening nounset for the delegated
command.

With all inherited ROS, colcon, Python, and library-path variables removed,
the installed Humble command matched the source launcher SHA-256
`48277513dab87837c808d41a63a9ccdb4ed172cad5376add05f5f2f75d374550`,
selected `rko_lio_graph_public_path`, fully inspected the `time` point field and
header order, and returned the calibration-review `start --dry-run` card without
creating the requested output. The complete installed-product checker passed
from the same prefix.

### All-six source follow-up

The next controlled run built all six maintained source packages into new
merged prefixes in both fixed images, still with the candidate mounted
read-only, network disabled, and exact dirty source identity bound. Humble
finished in 302 seconds and Jazzy in 273 seconds. Their source and installed
package lists are identical, and both complete installed checkers plus direct
fresh-terminal real-bag paths pass.

That follow-up also found and repaired Python cache leakage at both build and
runtime boundaries. After reinstall, each complete prefix contains zero
`__pycache__`, `.pyc`, or `.pyo` artifacts before and after validation. See the
[dual-distro all-source evidence](source-all-packages-install-2026-08-12.md).

## Verification

| Check | Result |
| --- | --- |
| pre-repair launcher activation regression | `3 failed` as expected |
| merged, isolated, source-layout, symlink, mismatch, and setup-failure launcher regressions | PASS |
| source quickstart regression | `8 passed` |
| launcher/source/CLI-contract regression group | `36 passed` |
| docs-entrypoint regression | `15 passed` |
| fresh isolated Jazzy package build | PASS |
| direct isolated `doctor --json` from removed ROS environment | PASS; `rko_lio_graph_public_path` selected |
| direct isolated `start --dry-run --viewer none` | PASS; calibration review, no files written |
| fresh merged Jazzy package build | PASS |
| clean merged Jazzy installed-product CLI checker | PASS |
| network-disabled Humble `lidarslam` build against immutable image underlay | PASS; exact dirty source identity bound |
| direct merged Humble `doctor --json` from removed ROS environment | PASS; `rko_lio_graph_public_path` selected |
| direct merged Humble `start --dry-run --viewer none` | PASS; calibration review, requested output absent |
| clean merged Humble installed-product CLI checker | PASS |
| network-disabled Humble all-six source build/install | PASS; 302 s; exact six-package source/install lists |
| network-disabled Jazzy all-six source build/install | PASS; 273 s; exact six-package source/install lists |
| CMake package-content and runtime bytecode write exclusion | PASS; zero cache artifacts before and after complete checker on both distros |
| canonical unsourced contributor gate | graph: `1,428 passed, 13 skipped, 11 warnings`; lidarslam: `626 passed`; `PASS: all` |
| `ament_flake8` on changed Python tests/checker | PASS |
| `bash -n` on both launchers | PASS |
| CLI JSON parse and `git diff --check` | PASS |

The two package-scoped product processes passed 2,054 tests in total. The
existing graph warnings are the previously recorded ImageIO v3 notices.

## Limits and next gate

This closes the remembered-activation gap and the local all-six source-overlay
build question; it is not a substitute for a package-manager install or a
complete cold-machine source quickstart. The immutable images still supply
ROS, system dependencies, and build tooling. The normal short
`lidarslam-map` spelling still requires explicit PATH activation until ROS
buildfarm packages exist. NDT upstream convergence, Jazzy RKO-LIO main sync,
binary package-manager E2E, public candidate publication, dependency bootstrap,
the full public demo, and comparable external onboarding timing remain separate
gates.

No shell profile, commit, branch, pull request, issue, label, release, image,
package, review reply, or external repository was changed.
