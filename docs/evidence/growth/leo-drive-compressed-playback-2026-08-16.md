# Leo Drive compressed-playback evidence — 2026-08-16

## Outcome

Implementation tip: `92bb5246fd46255c0557444eb1489b55bb29dc96`.

The exact clean implementation tip satisfies the blocking
`leo_drive_applanix_velodyne_cross` release profile at `TARGET_MET`. It also
keeps rosbag2 FILE decompression out of the source dataset directory and
removes the private playback view automatically after the run. This is
Applanix cross-validation, not independent ground truth, a GLIM execution, or
release authority.

## Current product-Draft rerun carrier — 2026-08-17

The clean public Draft carrier
`4163b8c6f0ebb63504a0de398d9654333bf3919e` was rerun against the same
byte-identified compressed input before this evidence synchronization. The
36.513-second bag completed in 79.37 seconds of wall time with 89,012 KiB
maximum resident memory. The schema-valid output records:

| Measurement | Result |
| --- | ---: |
| aligned Applanix cross-validation APE RMSE | 0.1391520544 m |
| release-profile pass threshold | 1.500 m |
| release-profile target | 0.500 m |
| profile status | `TARGET_MET` |
| matched poses | 571 |
| raw trajectory APE RMSE | 0.2697120408 m |
| lidarslam real-time factor | 1.568992 |

The exact carrier output identities are:

- `metrics.json` SHA-256:
  `76d6465729a7c48a46919d2b1029996393be6d0e615a893fee28440637963627`;
- `map.pcd` SHA-256:
  `cf54234f0a75b341ea03c0e079559db281510a03e4251ebc9c25780063d2b3ab`;
- generated reference TUM SHA-256:
  `98ea2825378036b2bad838cdff921540bcd4abd050f33b80ba35c5b9db4dba26`;
- map verification: 8 PASS / 1 informational WARN / 0 FAIL.

The output provenance names that exact Git commit, records a clean worktree,
and retains all input, parameter, harness, writer, and runtime-artifact
identities. The source-side uncompressed DB3 remained absent, the isolated
2,506,907,648-byte playback view was removed on exit, and the hard profile
gate reduced from five to four blocking `NO_DATA` rows. Because this Markdown
update creates a later docs-only commit, the later exact candidate must rerun
the benchmark; this carrier must not be relabeled as that later head.

## Exact input identity

The local Leo Drive `all-sensors-bag1` input retained these identities before
and after playback:

| Artifact | Size | SHA-256 |
| --- | ---: | --- |
| `metadata.yaml` | 9,685 bytes | `b3920a457de09655a1a473740e21609a88b910799787f8b7c0f8bd66c475bc83` |
| `all-sensors-bag1_compressed_0.db3.zstd` | 1,853,512,774 bytes | `b22ae495641c797ac540383c0059038a0c86548359b5ec7ed42f515e4f2cf667` |
| streamed uncompressed SQLite content | 2,506,907,648 bytes | `74e5915719a7b7b4820b5339207eeade0c656deaa38b8e5b5e8d18787a58ac22` |

The wrapper selected the front packet topic
`/sensing/lidar/front/velodyne_packets`, native `/gnss/fix`, and frame
`velodyne_front`. Its generated Applanix reference TUM SHA-256 was
`98ea2825378036b2bad838cdff921540bcd4abd050f33b80ba35c5b9db4dba26`.

## User-facing storage behavior

Before playback, the source-side uncompressed
`all-sensors-bag1_compressed_0.db3` did not exist. During playback, ROS 2
created the expected 2,506,907,648-byte database only under the run output's
marker-backed `.lidarslam-rosbag2-playback-*` directory. The source-side file
remained absent. On normal exit, the wrapper reported
`removed_playback_staging`, the private directory was absent, and the source
compressed file retained the same SHA-256.

The same helper is used by the full benchmark and smoke wrappers. It passes
ordinary uncompressed bags through unchanged, validates compressed storage
paths before linking them into the private view, and refuses to delete a
directory without the expected name, direct-parent relationship, and marker.

## Exact-head result

The 36.513-second bag completed in 79.52 seconds of wall time with 89,164 KiB
maximum resident memory. The schema-valid `metrics.json` recorded:

| Measurement | Result |
| --- | ---: |
| aligned Applanix cross-validation APE RMSE | 0.1391520544 m |
| release-profile pass threshold | 1.500 m |
| release-profile target | 0.500 m |
| profile status | `TARGET_MET` |
| matched poses | 571 |
| raw trajectory APE RMSE | 0.2697120408 m |
| estimated path length | 95.439811 m |
| lidarslam real-time factor | 1.570325 |

The output identities were:

- `metrics.json` SHA-256:
  `85dea38125a96d2b95f43c6267c5c4b384b1569750f53e9ef5ef787a14d12ba0`;
- `map.pcd` SHA-256:
  `cf54234f0a75b341ea03c0e079559db281510a03e4251ebc9c25780063d2b3ab`;
- map verification: 8 PASS / 1 informational WARN / 0 FAIL.

The hard release-profile command still exited `2`, correctly leaving four
blocking profiles at `NO_DATA`:

- `newer_college_math_hard`;
- `ntu_viral_tnp_01`;
- `mid360_gt_rtkslam_construction_seq2`; and
- `mid360_gt_rtkslam_construction_seq1`.

Six historical/report-only rows remain separate and do not block the current
gate. No tag, release, image, upload, issue, or community mutation was
performed.
