# Social demo copy — v0.9.1 candidate

> Status: **PUBLICATION_CANDIDATE / NOT_PUBLISHED**
>
> External publication authorized: **false**
>
> Source revision: `d0c84bb9bb7bef37d7e318000e3071a7f536d631`

Suggested attachment: `social_autoware_map_authoring_demo.mp4` with `social_autoware_map_authoring_demo.en.vtt`.

## Japanese

rosbag2 から、検証可能な Autoware-compatible map bundle までを1本の流れにしました。

```bash
lidarslam-map demo
lidarslam-map start /path/to/rosbag2
```

成功時は `pointcloud_map/`、`map_projector_info.yaml`、Lanelet2、
`map_verify: PASS`、検証 receipt を同じセッションで確認できます。
公開前の exact-version guide: <https://github.com/rsasaki0109/lidar_slam_ros2/blob/d0c84bb9bb7bef37d7e318000e3071a7f536d631/docs/getting-started.md>

この下書きはリリース、対応パッケージ、性能優位、センサー互換性を主張しません。
投稿する場合は、同じ版の公開済み artifact と public docs の監査完了後に文言を再確認してください。

## English

One guided path now takes a rosbag2 recording to a verifiable
Autoware-compatible map bundle.

```bash
lidarslam-map demo
lidarslam-map start /path/to/rosbag2
```

A successful session keeps the map bundle, `map_verify: PASS`, and its
validation receipt together. Exact-version guide before publication:
<https://github.com/rsasaki0109/lidar_slam_ros2/blob/d0c84bb9bb7bef37d7e318000e3071a7f536d631/docs/getting-started.md>

This draft makes no release, package-availability, performance-superiority, or
sensor-compatibility claim. Recheck the wording only after the same-version
artifacts and public documentation pass their publication audits.

## Alt text

Four-slide silent demo showing a rosbag2-to-Autoware map workflow, the fixed
demo and own-bag commands, an Autoware map-loader view, and the verification
artifacts retained after a successful mapping session.
