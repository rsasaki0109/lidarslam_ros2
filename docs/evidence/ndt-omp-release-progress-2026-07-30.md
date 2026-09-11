# ndt_omp_ros2 0.1.0 release progress — 2026-07-30

The read-only release audit was rerun from the pinned lidarslam_ros2 checkout:

```bash
python3 scripts/check_ndt_omp_release_readiness.py --json
```

It reported `IN_PROGRESS` with no remote-inspection errors:

- the local candidate is the reviewed commit
  `8b77fa5a6cdcad45bf35918361c892b6d94a287e`;
- source tag `0.1.0` exists;
- `rsasaki0109/ndt_omp_ros2-release` exists with generated Humble and Jazzy
  release tracks;
- neither public rosdistro distribution file contains `ndt_omp_ros2` yet.

The generated registration pull requests are:

- Humble: [ros/rosdistro#52949](https://github.com/ros/rosdistro/pull/52949)
- Jazzy: [ros/rosdistro#52950](https://github.com/ros/rosdistro/pull/52950)

Both PRs were still open and mergeable, with every automated check passing,
when reviewed again on 2026-07-31. The live audit now names those existing PRs
and explicitly forbids recreating the immutable source tag or rerunning Bloom
while they remain current. They remain external maintainer actions. This
evidence records progress only; the distribution gate stays incomplete until
the live audit reports `RELEASED`, RKO-LIO 0.3.2 is present in the normal apt
repository, and the package-manager install/upgrade E2E passes.
