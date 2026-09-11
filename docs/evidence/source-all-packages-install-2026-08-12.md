# Dual-distro all-source install and read-only CLI — 2026-08-12

> Status: **LOCAL_DUAL_DISTRO_PASS / PUBLICATION_PENDING**
>
> Remote mutations performed: **none**

## User-facing result

The current local candidate built and installed every maintained ROS package
from the mounted source on both Humble and Jazzy:

```text
graph_based_slam
lidarslam
lidarslam_msgs
ndt_omp_ros2
rko_lio
scanmatcher
```

The exact source and installed package lists match. From a terminal with all
inherited ROS, colcon, Python, and library-path variables removed, the absolute
installed `lidarslam-map` command then inspected a real rosbag2 fixture,
selected `rko_lio_graph_public_path`, and produced the calibration-review
`start --dry-run` plan without creating its requested output. The complete
installed-product contract passed on both distributions.

This closes the local all-six-overlay build question left by the earlier
package-only launcher proof. It does not yet create a public comparable source
onboarding row: the cumulative candidate is private and dirty, and these fixed
images already contain ROS and system dependencies.

## Controlled build

Both executions used the same controls:

- candidate source mounted read-only at `/repo`;
- a separate writable build/install root;
- `--network none`;
- host UID/GID rather than root-owned output;
- a non-symlinked merged install;
- `Release` and `BUILD_TESTING=OFF`; and
- packager-supplied source identity
  `3f4dd70cdc58ad421192559213cdee0bdc41eba8`, `dirty=true`,
  `source=override`.

| Field | Humble | Jazzy |
| --- | --- | --- |
| immutable image | `sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447` | `sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` |
| all-six build wall time | 302 s | 273 s |
| all-six build result | PASS | PASS |
| source/install list SHA-256 | `fc4bb8008dbfc5fdff9489cc93991a2b5d44f8ab1ef6c15aeffed9786772b36c` | same |
| final install bytes | 436,764,668 | 402,860,371 |
| initial build stdout SHA-256 | `534c9c3d4fb6aae722e494262bc7db54efbe57c3e276a40917a842c6e48e6d2f` | `d7ff466ce575a4100d223da4352d48a0fb1c940f2a36a07b46155c6b7f9d3361` |
| final direct-launcher SHA-256 | `48277513dab87837c808d41a63a9ccdb4ed172cad5376add05f5f2f75d374550` | same |
| build-info SHA-256 | `7acad95525b3569e38594f835ba22071236f0ec4d160660c467d434df65f804e` | same |

The source and installed launcher hashes also match byte for byte on both
distributions. Reinstalling `lidarslam` after the read-only repair retained the
same six package index entries and exact source identity.

## Fail-closed storage behavior

The first Humble checker run started immediately after the all-six build had
left its build tree in place. The normal map-output safety floor required
5.00 GiB but measured only 4.30 GiB, so `run --dry-run` refused with exit `2`
and an actionable cleanup message. After the hashed build artifacts were
removed, free space returned above the floor and the same installed prefix
passed. The gate was not lowered to make the test green.

## Package-share mutation found and repaired

The successful all-source exercise exposed two ways an apparently read-only
installed product could still acquire Python cache artifacts:

1. `install(DIRECTORY launch ...)` copied an ignored development
   `launch/__pycache__` into the product; and
2. the installed launcher and the checker’s direct module imports allowed
   Python to generate more caches during normal inspection.

Before repair, each validated prefix contained 17 `.pyc` files in four
`__pycache__` directories. The repair now:

- excludes `__pycache__`, `*.pyc`, and `*.pyo` from the CMake directory
  install;
- exports `PYTHONDONTWRITEBYTECODE=1` from the shared source/installed
  launcher, so delegated Python processes inherit the rule;
- sets `sys.dont_write_bytecode` before the installed checker dynamically
  imports product modules; and
- snapshots every Python cache artifact in the complete install prefix before
  and after the checker, comparing path, size, nanosecond mtime, and content
  digest.

After cleaning the already generated caches and reinstalling the candidate,
both prefixes began with zero cache artifacts, passed the complete checker,
passed direct `doctor` and `start --dry-run`, and still contained zero cache
artifacts. The dry-run output directory remained absent.

## Verification summary

| Check | Humble | Jazzy |
| --- | --- | --- |
| exact six source packages discovered | PASS | PASS |
| exact six packages built and indexed in fresh prefix | PASS | PASS |
| source mounted read-only and network disabled | PASS | PASS |
| complete installed-product CLI checker | PASS, 11.8 s post-repair | PASS, 13.8 s post-repair |
| absolute `doctor --json` with inherited ROS/Python paths removed | PASS; point field inspected, timestamp order passed, maintained profile selected | same |
| absolute `start --dry-run --json` | PASS; calibration retained, no output written | same |
| install-prefix Python cache snapshot unchanged | PASS; zero before and after | PASS; zero before and after |
| launcher environment/CMake exclusion regressions | `8 passed` on the host contract group | shared candidate |

The full cumulative Python gate had already passed immediately before this
follow-up: graph `1,428 passed, 13 skipped`, lidarslam `626 passed`, 2,054 total.
Focused tests are rerun after this repair; the final cumulative artifact receipt
records the final gate state.

## Limits and next gate

This is a dual-distro all-source **overlay** proof, not a cold-machine source
quickstart measurement. The immutable images provide Ubuntu, ROS, apt-managed
libraries, build tools, and other system prerequisites. The execution did not
install dependencies from an empty host, download or run the 517 MB public
demo, measure network RX, use a publicly resolvable candidate revision, or
exercise Debian package ownership and upgrades.

The next G0 action is therefore a clean-machine onboarding trial from a
published immutable candidate, first on Humble and Jazzy source paths and then
as the comparable four-row Docker/source matrix. Package-manager proof remains
blocked on collision-free `ndt_omp_ros2` publication and the required public
repository state.

The immediate follow-up now makes that transition fail closed: the source
quickstart validates and explicitly selects this exact package list, while a
separate public-route preflight rejects an unpublished or incomplete commit
before a trial VM is consumed. See the
[source onboarding preflight evidence](source-onboarding-public-preflight-2026-08-12.md).

No commit, branch, pull request, issue, label, release, package, image, review
reply, or external repository was changed.
