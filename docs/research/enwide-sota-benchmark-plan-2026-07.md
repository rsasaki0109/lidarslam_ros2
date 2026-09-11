# ENWIDE Degenerate-LIO SOTA Benchmark Plan

## Claim status

No SOTA claim is currently permitted. The radar-less tunnel result is an
internal single-sequence result and is not directly comparable with a public
benchmark.

`degenerate_lio_sota_v1.yaml` preregisters the initial public track:

- LiDAR geometry, LiDAR intensity, and IMU are allowed;
- radar, wheel odometry, GNSS, and cameras are forbidden;
- one frozen parameter set must be used for every sequence;
- failed and incomplete runs remain in the result set;
- each system is run three times on identical input bytes and hardware;
- SE(3) alignment never estimates scale;
- 10 m RTE is the mean translational drift over all complete 10 m
  ground-truth-path segments.

## First public sequences

The bring-up uses ENWIDE TunnelS and TunnelD. ENWIDE supplies an Ouster OS0-128
point cloud at 10 Hz, IMU at 100 Hz, and position-only Leica MS60 ground truth.
The official topics are `/ouster/points` and `/ouster/imu`.

The COIN-LIO paper reports the following reference results using ATE RMSE and
10 m relative translational error:

| Sequence | Length | ATE RMSE | RTE |
|---|---:|---:|---:|
| TunnelS | 251.58 m | 0.743 m | 1.60% |
| TunnelD | 179.71 m | 0.487 m | 1.59% |

Published values are context, not the final gate. The gate uses reproduced
results from pinned rival revisions on the same machine and exact bag bytes.

## Dataset acquisition

```bash
bash scripts/download_enwide.sh \
  --sequence tunnel_d \
  --dest /media/sasaki/aiueo/datasets/enwide \
  --convert
```

The downloader verifies official byte counts, supports resumed transfers, and
writes SHA-256 identities for the source bag and converted rosbag2 tree. Dataset
content remains outside the repository and is licensed CC BY 4.0.

The preregistered RKO-LIO candidate is
`configs/enwide/rko_lio_os0_degenerate_sota_v1.yaml`. Its LiDAR-to-IMU transform
is derived from the official Ouster metadata, and both external-velocity modes
are explicitly disabled.

After conversion, one command runs all three repetitions with the fixed topics,
configuration, prism lever arm, scoring policy, and completion gate:

```bash
bash scripts/run_enwide_sota_benchmark.sh \
  --sequence-dir /media/sasaki/aiueo/datasets/enwide/tunnel_d \
  --output-dir output/enwide_tunnel_d_rko_sota_v1
```

## Path to a defensible SOTA claim

1. Bring up TunnelD and TunnelS without per-sequence tuning.
2. Reproduce COIN-LIO, BIEVR-LIO, FAST-LIO2, Point-LIO, and the merged RKO-LIO
   baseline from pinned commits.
3. Extend the same frozen configuration to all ten ENWIDE sequences.
4. Add GEODE degenerate sequences and a preregistered hidden tunnel holdout.
5. Publish input hashes, machine fingerprint, raw trajectories, failures,
   runtime, memory, ATE, RTE, and distance-dependent point-projection metrics.
6. Unlock the SOTA claim only after zero catastrophic failures and at least a
   3% primary-metric improvement over the strongest reproduced rival.
