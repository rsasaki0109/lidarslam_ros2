# Docker first-map trial — 2026-07-28

## Result

A maintainer-operated fresh-environment trial of the README Docker command
completed the tracked MID-360 workflow and produced the versioned product
artifacts after two onboarding defects were corrected:

1. the demo selected an outer extraction directory instead of the nested
   rosbag2 directory that contained `metadata.yaml`;
2. the Docker default bypassed `lidarslam-map`, so a usable map lacked the
   product manifest, verification log and diagnosis artifacts.

The fixes landed in
[PR #406](https://github.com/rsasaki0109/lidar_slam_ros2/pull/406) and
[PR #407](https://github.com/rsasaki0109/lidar_slam_ros2/pull/407).
The final published-image rerun passed.

This is one maintainer-operated usability trial. It is direct evidence for the
Docker first-success path, but it does not count as any of the three
independent-user validations required before v1.0.

## Trial environment

| Item | Recorded value |
| --- | --- |
| Host | `sasaki-pc`, x86_64 |
| Host OS | Ubuntu 24.04.4 LTS |
| Kernel | `6.17.0-35-generic` |
| Docker client/server | `29.4.3` / `29.4.3` |
| ROS distribution in image | Humble |
| Publish workflow | [develop docker image run 30340468419](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30340468419) |
| Final image revision | `1fe7e20ce4f1cc60b07eb5066d1347e96abe2bf6` |
| Final image digest | `sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061` |
| Input archive | `rosbag2_2024_04_16-14_17_01.zip`, 517,088,133 bytes |
| Input MD5 | `0836c50859bb1af591966b69da166186` |
| Source | Koide, *Driving SLAM Test with Livox MID360*, Zenodo DOI `10.5281/zenodo.14841855`, CC-BY 4.0 |

The output directory was empty before each attempt. The first attempt also
started without a cached dataset. The final acceptance attempt used the
published image by digest and the README command shape:

```bash
docker run --rm \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061
```

The moving `:humble` tag was resolved to the recorded digest before acceptance.

## Failure and correction sequence

### Attempt 1 — README command failed after download

The initial image was revision
`0ec55575ffc16eb008e9f24bd6c6f24700bf2f8a`, digest
`sha256:782b5d92794f03fedc272c0ee632bca23a429769d05d3d481fc652a9f35df809`.
It downloaded the 517.1 MB archive, verified MD5
`0836c50859bb1af591966b69da166186`, and extracted the bag. It then exited:

```text
error: demo bag not found under /lidarslam_ws/datasets/mid360_public
```

The archive contains an outer directory and an inner rosbag2 directory with
the same name. `run_docker_demo.sh` selected the first matching directory
instead of selecting the directory that actually contained `metadata.yaml`.
PR #406 changed discovery to use the metadata file and added a regression
fixture with the same nested layout.

### Attempt 2 — first map exposed a contract split

With the discovery fix mounted read-only, the same image completed mapping:

- 577 corrected poses and 2,687 raw poses;
- 365 point-cloud tiles;
- 42 Lanelet2 lanelets;
- 127 MB output;
- independent Autoware verification: 8 pass, 0 warn, 0 fail.

The output did not contain `run_manifest.json`,
`autoware_map_diagnosis.json`, `autoware_map_diagnosis.md`, or
`verify_autoware_map.log`. The Docker entrypoint was calling the lower-level
dogfood runner instead of the official product CLI, contrary to the common
successful-output contract.

PR #407 routes the default through:

```bash
lidarslam-map run "$bag" \
  --profile rko_lio_graph_mid360_preset \
  --output-dir /lidarslam_ws/output/mid360_demo
```

### Attempt 3 — unified candidate passed

The candidate wrapper completed the installed golden path in 83.530 seconds.
It produced a schema-v2 `run_manifest.json` with status `succeeded`, lifecycle
stage `complete`, runner exit code `0`, and diagnosis status `success`.
Verification reported 8 pass, 0 warn and 0 fail. Atomic finalization removed
the `.partial` sibling.

## Final published-image acceptance

| Check | Result |
| --- | --- |
| OCI revision contains merged PR #407 | PASS — `git merge-base --is-ancestor` exit 0 |
| Registry attestation verification | PASS — GitHub CLI 2.87.1 `gh attestation verify` exit 0 |
| Installed `run --help` option groups | PASS — product usage plus five option groups, with no internal Python entrypoint |
| Demo process exit code | `0` |
| Manifest schema/status | schema `2`, status `succeeded`, profile `rko_lio_graph_mid360_preset` |
| Lifecycle stage/runner exit | `complete` / `0`, verification enabled |
| Diagnosis | `success` |
| Autoware verification | PASS — 8 pass, 0 warn, 0 fail |
| Corrected/raw poses | 576 / 2,689 |
| Point-cloud tiles | 363 |
| Lanelets | 42 relations |
| Output size | 127 MB |
| `.partial` sibling absent | PASS — 0 matching directories |

Selected final artifact identities:

| Artifact | SHA-256 |
| --- | --- |
| `run_manifest.json` | `0b5f7f01b830d77aabd97a09bb2d26054b3b6b4a2c2ad8a14cd9b1421fc2941b` |
| `autoware_map_diagnosis.json` | `696236c2f237458c62aec959b63673c4583c1458ed0c6466595de053c0ba8270` |
| `verify_autoware_map.log` | `f8d9a43f88615dbf2421bf1c42d303457f65e338f4836a267f6958149fbbdee5` |
| `pointcloud_map/pointcloud_map_metadata.yaml` | `d92eeefc4b068c2cf121bb7664851a2fb03b5a100dd43c2e56347dc541355710` |
| `traj_corrected.tum` | `001afc405f18b36c9aefb17983ca985da734fed7b2358e9d2d20c7a678eb0338` |

## Findings and follow-up

- The trial image had no periodic progress display during the first 517.1 MB
  download. The current downloader reports bytes, percentage and transfer rate
  at time or byte intervals.
- The trial image left bind-mounted files owned by container UID/GID `0:0`.
  The current Docker demo accepts an explicit host UID/GID pair and restores
  the dedicated output mount, run directory, `.partial` output and lock sidecar
  on both successful and failed exits.
- A moving convenience tag is appropriate for evaluation, but evidence and
  deployment must continue to record an immutable digest.
- This trial covers one amd64 host and Humble image. It does not replace the
  Jazzy gate, source-workspace usability trial, arm64 evaluation, or three
  independent-user first-map reports.

The original findings do not invalidate the generated map. Progress visibility
and host-file ownership were closed as onboarding issues; the platform and
independent-user coverage gaps remain before v1.0.
