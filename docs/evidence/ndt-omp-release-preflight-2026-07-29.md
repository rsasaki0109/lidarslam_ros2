# ndt_omp_ros2 Initial-Release Preflight — 2026-07-29

This is the permanent review record for the `ndt_omp_ros2` `0.1.0` candidate.
It is historical evidence, not a replacement for rerunning the live
preflight immediately before publication.

## Result

From the parent `lidarslam_ros2` checkout:

```bash
python3 scripts/check_ndt_omp_release_readiness.py \
  --require-ready-to-tag \
  --json
```

The command exited 0 with `status: READY_TO_TAG`.

- Parent gitlink, submodule HEAD, and public `origin/humble` all resolved to
  `8b77fa5a6cdcad45bf35918361c892b6d94a287e`.
- The submodule was clean and declared package `ndt_omp_ros2` version `0.1.0`
  under `BSD-2-Clause`.
- The changelog, installed/exported CMake target, Bloom preflight, JSON schema,
  and Humble/Jazzy CI contract were present.
- GitHub reported no `0.1.0` source tag and returned 404 for
  `rsasaki0109/ndt_omp_ros2-release`.
- The current Humble and Jazzy `ros/rosdistro` distribution files contained
  no `ndt_omp_ros2` repository key.

All remote queries succeeded. The checker treats only an explicit 404 as an
absent GitHub artifact; rate limits, authorization failures, server errors,
timeouts, malformed responses, and candidate branch drift produce `BLOCKED`.
No remote state was changed by this check.

## Reproduction and promotion rule

CI reproduces the repository-owned portion with:

```bash
python3 scripts/check_ndt_omp_release_readiness.py --offline
```

That result is `LOCAL_READY`, not permission to publish. The maintainer must
rerun the online `--require-ready-to-tag` command immediately before creating
the source tag. After both Bloom PRs merge, `--require-released` must pass
before this dependency is considered complete in the v1 distribution gate.
