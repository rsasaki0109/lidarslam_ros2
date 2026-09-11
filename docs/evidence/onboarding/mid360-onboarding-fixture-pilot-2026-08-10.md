# MID-360 onboarding fixture pilot — 2026-08-10

## Result

A provenance-pinned 50-second derivative of the public MID-360 bag is a viable
onboarding candidate. Two clean builds from commit
`0f91452c505e03fc810c79a0b6e602101ffefce7` produced byte-identical ZIPs and
byte-identical manifests. An extracted candidate then completed the installed
golden-path map route with a schema-valid seven-of-seven `PASS` receipt.

This is a local candidate, not a published asset, a cold onboarding trial, or
a replacement for the full 277-second real-data gate. The default demo and
release workflow are unchanged.

## Candidate identity

The source is [Driving SLAM Test with Livox MID360](https://zenodo.org/records/14841855)
by Kenji Koide, DOI `10.5281/zenodo.14841855`, licensed under CC BY 4.0. The
candidate ZIP contains `ATTRIBUTION.md` with the creator, source, DOI, license
link, change notice, and no-endorsement notice.

| Identity | Full source | 50-second candidate |
| --- | ---: | ---: |
| Requested slice window | not applicable | 50.000000000 s |
| Recorded / selected-message duration | 277.166836670 s | 49.999532958 s |
| LiDAR messages | 2,772 | 500 |
| IMU messages | 55,435 | 10,001 |
| Archive / ZIP bytes | 517,088,133 | 98,873,952 |
| rosbag2 sqlite bytes | 1,468,932,096 | 264,880,128 |

The candidate is 418,214,181 bytes, or 80.879%, smaller than the current
source download. It passes the explicit `100,000,000`-byte gate with a
1,126,048-byte margin.

Exact candidate identities are retained in the
[schema-valid build manifest](mid360-onboarding-50s-v1-build-20260810.json):

- ZIP SHA-256:
  `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3`;
- build-manifest SHA-256:
  `60c37f5c7efa7d61ca20f21803fa11b02add4bad047ae99d277e9e6811fbbb6e`;
- sqlite SHA-256:
  `0a38fbccdb9c135b9a8e72141515b87ff64172a3b80a2d27a260ecd30683b5dd`;
- metadata SHA-256:
  `d866804b49e8208dfcd70c2f91584976df0ca3d918c910d2ae20786cd1fa5b18`;
- attribution SHA-256:
  `d363a9e9cf529876e38fa4b705e3b323cd40b0d642c02cf9ef5a003c3061aee8`.

## Reproducibility and provenance gates

The builder verified the exact 517,088,133-byte source archive by MD5 and
SHA-256, then verified the original `metadata.yaml` and 1,468,932,096-byte
sqlite file by SHA-256. It re-hashed the source bag after slicing to detect a
change during generation.

The original Humble rosbag does not embed message definitions. The slicer now
permits that case only when the generated definition and RIHS01 hash are
identical in the maintained Humble and Jazzy typestores. An unverifiable custom
type, a source-hash mismatch, a source or storage symlink, a root-escaping
storage path, an existing output, or an oversized ZIP fails closed.

Both clean builds used:

| Component | Value |
| --- | --- |
| Git revision | `0f91452c505e03fc810c79a0b6e602101ffefce7` |
| Git dirty | `false` |
| Python | `3.12.3` |
| rosbags | `0.11.0` |
| sqlite | `3.45.1` |
| zlib compile / runtime | `1.3` / `1.3` |
| ZIP recipe | DEFLATE level 9, sorted members, mode `0644`, fixed 1980 timestamp |

The two ZIP byte streams and two JSON manifest byte streams matched exactly.
The manifest intentionally reports `map_validation_status: NOT_RUN`: it is the
immutable output of the build step. Map proof is retained separately so
validation cannot mutate the artifact identity after generation.

## Installed product map proof

The ZIP was integrity-checked, safely extracted, and passed to the installed
`lidarslam-map run` route with profile `rko_lio_graph_mid360_preset`. The
installed product build information was clean and identified the same commit
as the generator.

| Check | Result |
| --- | ---: |
| Product exit / manifest / lifecycle | `0` / `succeeded` / `complete` |
| Diagnosis / Autoware verifier | `success` / `PASS` |
| Receipt | `PASS`, exactly 7 / 7 checks |
| Product execution time | 15.043069 s |
| Raw / corrected poses | 48 / 87 |
| Pointcloud tiles | 86 |
| Pointcloud tile bytes | 3,990,757 |
| Combined `map.pcd` bytes | 3,970,854 |
| Output file / allocated bytes | 8,249,858 / 8,585,216 |
| Final `.partial` output | absent |

The reviewed, geometry-free
[first-map receipt](mid360-onboarding-50s-v1-map-receipt-20260810.json) contains
no local path or exact command. Its SHA-256 is
`86d2b5d2aa493cbb6ecc6efd88095a591f247bcea4bc171c68093cf165cc0754`.
The receipt binds these private run artifacts without publishing them:

- run manifest:
  `3875f246c2379cdec36bffefea365417cd92564678a7526e6f46359f8153aa17`;
- diagnosis:
  `890b3be607d0b2f88b03a10e8bba410cc8963ea9e431b5dd96896cdf154efdd5`;
- verifier log:
  `63053526d4a3686219461a2c2c7277890940bf8522d44dfd6f2d82748de1dfea`.

The build manifest and receipt were validated against their Draft 7 schemas.
`review_before_publishing: true` and `review_before_sharing: true` are mandatory
review requirements, not automatic clearance. The two bounded JSON files were
reviewed before being added to Git. The ZIP, sqlite bag, maps, trajectories,
and private logs were not added.

## Activation impact and limits

Replacing only the 517,088,133-byte dataset transfer in the existing Docker
machine probes gives an arithmetic estimate of 1,352,422,163 bytes for Humble
and 1,488,595,341 bytes for Jazzy. Those would be reductions of 23.619% and
21.933%, respectively. They are not measurements: image layers, registry
compression, cache state, and network conditions were not rerun or controlled.

The estimate also shows that the image side remains larger than the candidate
dataset. A smaller bag alone cannot reach the stretch target of less than 1 GB
cold workflow RX. The next independent intervention is a multi-stage/runtime
image experiment with at least 25% image-size reduction as its promotion gate.

This local run did not measure a clean download, cold wall time, human active
time, command count, peak disk, Humble runtime, or an external user's first
success. It therefore cannot be promoted into the comparable onboarding
matrix. It also does not make an accuracy claim.

The shortened run produces 48 raw poses, 87 corrected poses, 86 tiles, and
3,990,757 tiled pointcloud bytes. The full gate requires 2,500 raw poses, 500
corrected poses, 300 tiles, and 50,000,000 pointcloud bytes. Keeping the full
277-second scheduled proof is therefore a hard requirement, not a preference.

## Reproduce the candidate locally

Use an exact clean checkout of the generator commit and the source identities
in `configs/real_data_e2e/driving_slam_mid360_v1.json`. Keep all geometry
outside the checkout:

```bash
python3 scripts/build_mid360_onboarding_fixture.py \
  --source-archive "$SOURCE_ARCHIVE" \
  --source-bag "$SOURCE_BAG" \
  --output-dir "$BUILD_A"

python3 scripts/build_mid360_onboarding_fixture.py \
  --source-archive "$SOURCE_ARCHIVE" \
  --source-bag "$SOURCE_BAG" \
  --output-dir "$BUILD_B"

cmp "$BUILD_A/mid360_onboarding_50s_v1.zip" \
  "$BUILD_B/mid360_onboarding_50s_v1.zip"
cmp "$BUILD_A/mid360_onboarding_50s_v1.manifest.json" \
  "$BUILD_B/mid360_onboarding_50s_v1.manifest.json"
```

Do not use `--allow-dirty` for a publication candidate. That option exists only
for explicitly non-publishable local experiments.

## Publication review follow-up — 2026-08-11

The separate
[publication review](mid360-fixture-publication-review-2026-08-11.md) now
revalidates the exact ZIP, all three member byte streams, attribution, map
receipt, and two clean rebuilds through a fail-closed machine-readable gate.
The local packet reports `LOCAL_ARTIFACT_PASS`, but the overall decision is
`AWAITING_PUBLICATION_DECISION`: the generator and reviewed gate revisions are
not yet publicly resolvable, no host has been selected, and no explicit upload
authorization exists. No fixture or source branch was published by the
review.

## Promotion decision

The candidate passes the local build, size, provenance, attribution,
reproducibility, map, verifier, and receipt gates. It is not yet approved for
upload or for changing the default demo. Promotion remains gated on:

1. an explicit source-push, host, and release/upload decision after the local
   publication review;
2. hosting the geometry-bearing ZIP outside Git with the exact manifest and
   checksum;
3. a `REMOTE_ARTIFACT_PASS` from the
   [host-aware publication audit](published-fixture-audit-gate-2026-08-11.md);
4. activation of the reviewed public URL through the tested
   [resumable, checksum-pinned acquisition path](public-dataset-acquisition-hardening-2026-08-11.md),
   retaining the full proof route as fallback;
5. clean dedicated-VM Humble/Jazzy Docker and source trials with cold RX, wall
   time, active time, command count, and isolated peak disk;
6. the external first-map cohort; and
7. a separate runtime-image slimming experiment.

Until those gates pass, the full public bag remains the default proof route and
broad promotion remains paused.
