# RKO-LIO voxel compatibility and release-gate evidence — 2026-07-31

## Scope

Clean candidate commit
`42c87e9df499af7fb3a9a138b2a097b13cfe8402` pins RKO-LIO commit
`597958e` and completes the existing `legacy_voxel_downsample` compatibility
contract. In double-downsample mode, compatibility now covers both the
half-voxel map pass and the ICP-keypoint pass. The default remains `false`;
no new public option was added.

This change fixes a Newer College Maths-Hard accuracy regression without
changing the modern default path used by the other release profiles.

## Root cause

A history bisect isolated upstream RKO-LIO commit `d167eb6`:

| RKO-LIO revision | first-pass sampler | Maths-Hard APE RMSE |
| --- | --- | ---: |
| `e27d9c8` | pre-v0.3 unordered-map sampler | 0.0812466 m |
| `d167eb6` | hash-sorted sampler | 0.1407549 m |

The latter result is identical to the v0.3.2 failure. The earlier
`legacy_voxel_downsample` implementation changed only the second pass, so it
could not restore the map input or the point representatives presented to that
pass. The fix applies the compatibility sampler to both passes and adds
separate single-pass and two-pass tests.

## Verification

The RKO-LIO build passed all 66 discovered C++ tests. The two explicit
legacy preprocessing tests also passed independently:

```text
preprocess_scan: legacy voxel mode preserves compatibility order
preprocess_scan: legacy voxel mode covers both downsample passes
```

Five real-data runs were then generated from the clean parent commit. Every
`metrics.json` records the same full candidate SHA and `git_dirty: false`.

| Blocking profile | mode | APE RMSE | pass | pairs | poses | RTF | status |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| Newer College Maths-Hard | legacy two-pass | 0.0812466 m | 0.100 m | 2,438 | 2,440 / 2,440 | 0.743 | PASS |
| NTU VIRAL tnp_01 | modern default | 0.9832244 m | 1.000 m | 9,200 | 5,794 / 5,795 | 0.334 | PASS |
| RTK-SLAM Construction 1 | modern default | 0.3211747 m | 0.550 m | 16 | 7,284 / 7,286 | 2.011 | PASS |
| RTK-SLAM Construction 2 | modern default | 0.0864482 m | 0.300 m | 16 | 5,875 / 5,878 | 1.667 | TARGET_MET |
| Leo Drive all-sensors-bag1 | classic path | 0.1388692 m | 1.500 m | 580 | 362 raw poses | 1.470 | TARGET_MET |

Maths-Hard improved from 0.1407549 m to 0.0812466 m. NTU VIRAL and both
RTK-SLAM values are bit-for-bit equal to their preceding clean-candidate
accuracy values, proving that the modern default path did not change. Leo
Drive also reproduced its preceding corrected APE value and its generated map
passed all eight Autoware compatibility checks with zero failures.

The exact candidate gate completed with exit code zero:

```bash
python3 scripts/benchmark_summary.py \
  --root /path/to/v090_release_20260731 \
  --release-profile scripts/release_profiles.yaml \
  --required-git-commit 42c87e9df499af7fb3a9a138b2a097b13cfe8402 \
  --fail-on-profiles \
  --write-md /path/to/v090_release_20260731/release_gate.md
```

All blocking rows were `PASS` or `TARGET_MET`. The MID-360 cross-validation,
Stadtgarten, and HILTI rows remained `NO_DATA` under their explicit
`report_only_until` policies and did not authorize or block the candidate.

## Reproduction

Run the form-gated Maths-Hard command in
[Benchmarking And Release Gate](../benchmarking.md#newer-college-maths-hard)
with `legacy_voxel_downsample: true` in that dataset's RKO-LIO parameter file.
Run the canonical NTU command from the same guide, then execute:

```bash
python3 scripts/run_rtk_slam_accuracy_suite.py \
  --dataset-root /path/to/rtk_slam \
  --sequence construction_seq1 \
  --sequence construction_seq2 \
  --output-dir /path/to/release-evidence/rtk_accuracy

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag /path/to/all-sensors-bag1 \
  --output-dir /path/to/release-evidence/leo_drive \
  --verify-map
```

The raw datasets and generated benchmark outputs are not committed. Artifact
identity is retained here so a copied evidence directory can be audited:

| artifact | SHA-256 |
| --- | --- |
| Maths-Hard `metrics.json` | `db1f951938d09ceb406e78bfe0ed3d823b7994ab6f849cb840cac82f74f5b834` |
| NTU VIRAL `metrics.json` | `6dfce883b4cbe28aca43f12987add0ae8330e68a2a894153ec3e9fecb0460b2a` |
| Construction 1 `metrics.json` | `602ec9dd0b10f834a52d2084d800546981cacfe92dde1072f1ddc576c470ce6a` |
| Construction 2 `metrics.json` | `5a8155dbcc1212e5f35f7ea8f13ca8a2a3820ba3785b0b2f3f19b88a23fb04b4` |
| Leo Drive `metrics.json` | `7c7888e4079d482a2d112451c69b4b6892109298d922ed4923ca44812de907a1` |
| rendered release gate | `b02c949ba4f08026e2e034fce7661ee0acf09c661b56c0cd5544535b7e1a034c` |

Each metrics artifact additionally embeds the input bag, reference,
parameter-file, harness, runtime-binary, and software hashes required by the
benchmark schema.
