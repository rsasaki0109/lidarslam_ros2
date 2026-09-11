# Pinned real-data E2E gate

The scheduled `real-data-e2e` workflow proves that the installed golden path
can turn one unchanged public rosbag into a verified Autoware-compatible map.
It runs nightly on ROS 2 Jazzy and can also be started with
`workflow_dispatch`. It is intentionally separate from pull-request CI because
the source archive is 517 MB.

## Pinned input

The first contract is
[`driving_slam_mid360_v1`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/configs/real_data_e2e/driving_slam_mid360_v1.json):

- source: [Driving SLAM Test with Livox MID360](https://zenodo.org/records/14841855),
  DOI `10.5281/zenodo.14841855`;
- archive: `rosbag2_2024_04_16-14_17_01.zip`;
- exact size: `517088133` bytes;
- MD5: `0836c50859bb1af591966b69da166186`;
- SHA-256: `f8f89eebf2aaf9cc1d465bfa5451bbb599cd92d079b59949104bb4e5cb619bdd`;
- bag identity: exact `metadata.yaml` and sqlite3 SHA-256 values in the
  contract;
- public sensor payload: 2772 PointCloud2 records and 55435 IMU records over
  277.16683667 seconds.

The registry and intake pin the archive size, SHA-256, and legacy MD5. The
cache key still includes size and MD5 for compatibility, but a restored cache
is re-hashed with SHA-256 by the intake and again checked by the E2E validator,
so a corrupt or replaced archive fails closed. Interrupted network transfers
remain under `.part` and resume only after an exact HTTP `Content-Range` check.

The Zenodo record declares the dataset under Creative Commons Attribution 4.0
International. The workflow downloads from the publisher for validation and
does not upload the source bag, trajectories, or map geometry. Its retained
artifact contains only the contract, intake identity, run manifest, diagnosis,
verification result, and logs.

## Blocking assertions

The run uses the installed `lidarslam-map doctor` and `lidarslam-map run`
commands. `validate_real_data_e2e.py` then requires:

- valid schema-v2 run manifest and diagnosis-v1 report;
- exact archive, metadata, and sqlite3 identities;
- the maintained `rko_lio_graph_mid360_preset` on ROS 2 Jazzy;
- successful, finalized terminal state within 600 seconds;
- exact preflight duration, message counts, types, and topics;
- Autoware verification with at least eight passes and no failures;
- at least 2500 raw poses, 500 corrected poses, 300 pointcloud tiles, and
  50 MB of tiled pointcloud evidence.

Any failed assertion exits non-zero. The workflow also has a 45-minute job
limit and a 20-minute product-run limit. If the product run times out, its
SIGTERM recovery path preserves terminal evidence before the job fails.

## Reproduce locally

From a built Jazzy workspace:

```bash
python3 scripts/download_mid360_robot_public_dataset.py \
  --dataset driving_slam_mid360 \
  --dataset-root datasets/real-data-e2e

source /opt/ros/jazzy/setup.bash
source install/setup.bash

bag="datasets/real-data-e2e/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01"
lidarslam-map doctor "$bag" --json
lidarslam-map run "$bag" --output-dir output/real-data-e2e/run

python3 scripts/validate_real_data_e2e.py \
  --contract configs/real_data_e2e/driving_slam_mid360_v1.json \
  --intake-manifest datasets/real-data-e2e/driving_slam_mid360/mid360_robot_public_dataset_intake.json \
  --run-dir output/real-data-e2e/run
```

The nightly gate proves one flagship real-data path. It does not replace
Humble/Jazzy build CI, multi-dataset accuracy benchmarks, long-duration soak
tests, disk-pressure injection, or independent third-party first-map
validation.

The separate weekly
[`bounded filesystem exhaustion`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/.github/workflows/bounded-filesystem-exhaustion.yml)
workflow reuses this exact public input for a destructive-output reliability
gate. It mounts the bag read-only, constrains map output to a 32 MiB tmpfs and
retains only non-geometry failure evidence.
