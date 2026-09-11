# Published v0.9.0 release evidence

The first stable release-candidate publication completed on 2026-07-31.

- Release: <https://github.com/rsasaki0109/lidar_slam_ros2/releases/tag/v0.9.0>
- Workflow: <https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30629881856>
- Immutable tag: `v0.9.0`
- Tag commit: `0df0c4a86df9f68a894c83f8342e4107c3d23b0f`
- Workflow conclusion: `success`
- Release state: final, stable (`draft=false`, `prerelease=false`)

The tagged workflow built, attested, and smoke-tested the Humble and Jazzy
amd64 images by digest before promoting their immutable version tags. It then
published and independently audited these six recovery assets:

- `lidarslam_ros2_v0.9.0_release_bundle.tar.gz`
- `release-image-humble.json`
- `release-image-jazzy.json`
- `release-promotion.json`
- `rollback-plan-humble.json`
- `rollback-plan-jazzy.json`

After publication, the following read-only command completed with
`Published release audit: PUBLISHED`:

```bash
python3 scripts/check_published_release.py --require-published
```

Every contract check passed: immutable tag identity, stable release state,
required assets, cross-asset source and image identity, embedded release
bundle hashes, rollback records, promotion record, and live Humble/Jazzy GHCR
version-tag digests. This evidence closes the reliability and stable v0.9
release-publication readiness gates; it does not close ROS apt distribution
or independent-user adoption.
