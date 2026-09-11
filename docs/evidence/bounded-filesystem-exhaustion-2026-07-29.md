# Bounded-filesystem exhaustion evidence — 2026-07-29

## Scope

Candidate commit `c1da11f3c2d44985c5483748d5e12648adbbbee7`
was built as the installed Jazzy product image and run against the pinned
MID-360 public bag with the complete map output confined to a 32 MiB Docker
tmpfs. The harness worktree was clean and the image
`org.opencontainers.image.revision` exactly matched the candidate commit.

This run tests a real capacity limit, not a mocked write failure. The full
machine-readable report remains outside the repository because it includes
runtime logs; the public workflow uploads the same non-geometry evidence
contract as a retained artifact.

## Reproduction

```bash
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --label "org.opencontainers.image.revision=$(git rev-parse HEAD)" \
  --tag "lidarslam-enospc:$(git rev-parse --short HEAD)" \
  .

python3 scripts/run_bounded_filesystem_exhaustion.py \
  datasets/real-data-e2e/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01 \
  --image lidarslam-enospc:c1da11f \
  --tmpfs-mib 32 \
  --timeout-secs 600 \
  --hardware-label sasaki-laptop-i5-1145G7-32GiB-jazzy-docker \
  --evidence-dir output/bounded-filesystem/evidence
```

Input identity:

- metadata SHA-256:
  `65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a`;
- sqlite3 storage file SHA-256:
  `3bbd390a97e57af47ad6699baa36eb4c5f39f61b35275505ecaf221c126354f5`;
- storage size: 1,468,932,096 bytes; message count: 58,217.

Runtime identity:

- Jazzy image ID:
  `sha256:d69decda77080b57bca470b36ec33ec1d5d23b1ba63a37f1e3fbe9a7708d2c36`;
- harness script SHA-256:
  `831d1e75d9e8ac082705fe5e9c16dd9b5e2aa960724520f2f58f9a51f1d41640`;
- embedded runner/diagnosis payload SHA-256:
  `9df208f8d0668927ad6437bf28bd226c2b42b7634e280066954eceb846f62d75`.

## Result

The schema-validated report passed all ten required checks in 146.410
seconds:

- the 33,554,432-byte tmpfs ended with 2,240,512 bytes available, below the
  10% exhaustion threshold;
- PCL emitted two real storage signatures, including
  `raw_fallocate(length=545975) returned 28`;
- the product and container both exited `1` before the 600-second deadline;
- the final manifest recorded `status: failed`, lifecycle stage `complete`,
  and runner exit code `1`;
- diagnosis recorded `runtime_failed` and explicitly identified exhausted
  output storage;
- no success state was claimed;
- only the manifest, diagnosis, and text logs were exported; pointcloud
  geometry was not copied.

The report SHA-256 is
`ba208e611e15b558174eb8ba154d7fd9be077a5099b304af7533b2fd9b3cc1ee`;
the container log SHA-256 is
`0ac93ef25d33a28ebea53b5a841d87cbfd166bd85a07c6a101b828a07764e73b`.

## Public integration

Merge commit `4151bf588aad3a2bed352f626a3146e3801df4e1` triggered the
[first public bounded-filesystem run](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30401935038).
The exact-revision image build, real-bag exhaustion run, outer report schema,
embedded manifest-v2 schema, embedded diagnosis-v1 schema, evidence summary,
and `bounded-filesystem-exhaustion-evidence` artifact upload all passed.

The same workflow remains scheduled weekly and is also change-gated for the
map runtime, runner, diagnosis, image, dataset intake, schema, and workflow
paths.

## Remaining boundary

The local and public executions close the bounded-capacity artifact gate.
They do not close the separate tagged-release rollback-assets gate.
