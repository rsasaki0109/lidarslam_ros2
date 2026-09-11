# Runtime image slimming pilot — 2026-08-11

## Result

Local multi-stage Humble and Jazzy images retained the installed first-map
contract while reducing Docker's local image-size measurement by more than
half. Both candidates were built from clean commit
`08d0040c2849412a88147f825841af2b3a516262`; neither image was pushed or used
to change a public tag.

| ROS distribution | Published local baseline | Local candidate | Reduction | 25% gate ceiling | Local result |
| --- | ---: | ---: | ---: | ---: | --- |
| Humble | 1,227,397,566 B | 569,009,937 B | 658,387,629 B (53.6409%) | 920,548,174 B | `PASS` |
| Jazzy | 1,357,460,149 B | 547,466,572 B | 809,993,577 B (59.6698%) | 1,018,095,111 B | `PASS` |

These values are the `.Size` returned by `docker image inspect` for the
locally present images. The public Jazzy registry layers summed to
1,357,449,522 bytes when the baseline was captured, but the candidates have no
registry artifact. The table therefore passes the local size proxy, not the
precommitted compressed-OCI promotion measurement. A reviewed registry export
or publication candidate must measure that value separately.

The local image identities were:

| ROS distribution | Published baseline image ID | Candidate image ID |
| --- | --- | --- |
| Humble | `sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447` | `sha256:6ad0e172952995ce1593e0269f7dce6b267c773bd45f3fd15082a0026ceec99f` |
| Jazzy | `sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` | `sha256:9a89b816f4bfcc3c0811a1f961f2c0f4d4d401354e5a978dc856bbdf06d9731d` |

Image IDs are local BuildKit/Docker identities, not public OCI digests.

## Source-free runtime closure

The builder installs the supported dependencies and compiles the six-package
product closure. A deterministic collector then scans installed ELF files,
resolves their shared libraries, maps external files and maintained runtime
commands to Debian package owners, and rejects unresolved or unsafe paths.
The runtime stage starts again from `ros:<distro>-ros-core`, installs that
closure, and copies only `install/` plus the fail-closed entrypoint.

| Check | Humble | Jazzy |
| --- | ---: | ---: |
| Installed ELF files scanned | 28 | 27 |
| Linked libraries resolved | 200 | 225 |
| Direct runtime packages collected | 138 | 150 |
| Direct development packages | none | none |
| Installed ELF files with missing `ldd` links | 0 | 0 |

The final images contain no checkout, `.git`, `build/`, `log/`, or
`Thirdparty/` tree. `git`, `colcon`, and `rosdep` are absent in both images.
The installed provenance record in each image reports the exact 40-character
commit, `dirty: false`, and `source: override`.

Humble retains `gcc`, `g++`, `cc`, `c++`, and `cmake`; `make`, `ninja`, and
`build-essential` are absent. These tools are automatically installed Jammy
package dependencies rather than direct collector output. For example,
`python3-scipy` depends on `python3-pythran`, which in turn depends on `g++`,
development headers, and Boost. Deleting package-owned files after installation
was rejected because it would make the package database and supported security
updates inconsistent. Jazzy has none of `gcc`, `g++`, `cc`, `c++`, `make`, or
`ninja`; its `cmake` comes from the official `ros:jazzy-ros-core` base.

## Installed-product smoke matrix

Every row was run with `--network none` against the final local candidate.

| Contract | Humble | Jazzy |
| --- | --- | --- |
| `lidarslam-map --version` and `--help` | `PASS` | `PASS` |
| `ros2 pkg prefix lidarslam` | `PASS` | `PASS` |
| `ros2 launch --help` / `ros2 bag --help` | `PASS` / `PASS` | `PASS` / `PASS` |
| `rclpy`, `rosbag2_py`, `scipy`, `jsonschema`, `rosidl_runtime_py.utilities` | 5 / 5 | 5 / 5 |
| Installed first-map/demo helper files | 4 / 4 | 4 / 4 |
| Provenance revision and clean state | `PASS` | `PASS` |
| Source and build trees absent | `PASS` | `PASS` |
| Installed ELF `ldd` closure | 28 / 28 | 27 / 27 |

The four newly required installed helpers are `run_first_map_demo.sh`,
`run_docker_demo.sh`, `download_mid360_robot_public_dataset.py`, and
`mid360_robot_public_datasets.py`. The downloader no longer imports a
source-only helper. Its optional source recording checker still fails closed
with exit code 127 and a direct recovery message when requested from the
curated runtime.

## Network-isolated 50-second map proof

The unchanged default image command was exercised with the local 50-second
[MID-360 fixture candidate](mid360-onboarding-fixture-pilot-2026-08-10.md)
mounted read-only under the public bag name. The container had no network,
the output root was initially empty, and the product ownership handoff was set
to UID/GID `1000:1000`.

Fixture identity:

- ZIP size: 98,873,952 bytes;
- ZIP SHA-256:
  `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3`;
- source archive size: 517,088,133 bytes;
- source archive SHA-256:
  `f8f89eebf2aaf9cc1d465bfa5451bbb599cd92d079b59949104bb4e5cb619bdd`.

| Observation | Humble | Jazzy |
| --- | ---: | ---: |
| Container exit | 0 | 0 |
| Observed wall time | 17.83 s | 16.98 s |
| Manifest / diagnosis / receipt schemas | `PASS` | `PASS` |
| Manifest status / lifecycle | `succeeded` / `complete` | `succeeded` / `complete` |
| Diagnosis / Autoware verification | `success` / 8 PASS, 0 WARN, 0 FAIL | `success` / 8 PASS, 0 WARN, 0 FAIL |
| First-map receipt | 7 / 7 `PASS` | 7 / 7 `PASS` |
| Manifest-recorded artifacts re-hashed | 116 / 116 | 114 / 114 |
| Raw / corrected poses | 372 / 88 | 351 / 87 |
| Pointcloud tiles | 89 | 87 |
| Pointcloud tile bytes | 4,038,829 | 3,991,647 |
| Combined `map.pcd` bytes | 4,017,969 | 3,971,519 |
| Regular-file / allocated bytes | 8,397,019 / 8,704,000 | 8,297,206 / 8,617,984 |
| Final transaction directory | absent | absent |
| Output ownership | `1000:1000` | `1000:1000` |

The receipts bind these non-published local artifacts:

| Artifact SHA-256 | Humble | Jazzy |
| --- | --- | --- |
| Run manifest | `6f6beea91d3b720188cc8e2458a490a8884c64919a2c0db1bb277a17057fd359` | `f2a6294e6741564bd3ff76b7dae7c732f046b01404e6aedc6ea8d0178ae72a6c` |
| Diagnosis | `18fe1bea5ea985a0ac09bd039b45664d4aaed841251e1bfededc7a672d06c621` | `18fe1bea5ea985a0ac09bd039b45664d4aaed841251e1bfededc7a672d06c621` |
| Verifier log | `f8d9a43f88615dbf2421bf1c42d303457f65e338f4836a267f6958149fbbdee5` | `f8d9a43f88615dbf2421bf1c42d303457f65e338f4836a267f6958149fbbdee5` |
| First-map receipt | `e018669d1e123481d2c19dc7e34179c0b77570189cf7fe51400df7206e406375` | `4da927cd9e87aac5a1dc3fa619063e9b1c3a993471f808d72f095f035ebdfddf` |

Do not interpret the two wall times or output-count differences as a ROS
distribution performance comparison. There is one warm local run per row,
launch scheduling was not controlled, and byte-identical map output was not a
gate. The shared contract is the schema-valid, hash-bound successful map and
verifier outcome.

## Findings and promotion decision

The local size proxy, source-free runtime, installed CLI, default command,
Autoware verifier, and receipt gates pass on both supported distributions.
The pilot proves that the previous image size was not required by the product
runtime path.

At the pilot revision, it was not ready for public image replacement.
Promotion remained blocked on:

1. an explicitly reviewed compressed-OCI measurement;
2. clean dedicated-VM Humble and Jazzy Docker/source rows with cold RX, wall
   time, active time, command count, and isolated peak disk;
3. a full 277-second public demo rerun from the exact candidate revision;
4. publication review for the still-local 50-second fixture;
5. repair or deliberate removal of Jazzy's non-contract `.ros_log/latest`
   symlink, which retains the absolute `.partial` path after atomic output
   finalization; and
6. a Docker cache-boundary improvement: copying the full source before
   dependency installation currently makes small source changes repeat the
   expensive builder dependency layer.

## Follow-up validation — commit `2f98a6c`

The two local implementation findings above were repaired in separate commits:

- `946caec` normalizes a transaction-local `.ros_log/latest` into a portable
  relative link immediately before atomic finalization; and
- `2f98a6c` keys the expensive builder dependency layer on the six maintained
  `package.xml` manifests, then copies the full source before compilation.

A clean Jazzy build at exact commit
`2f98a6c613f362ea78d4793368c2dd09d1832ee4` proved that the manifest-only tree
is sufficient for `rosdep`. The dependency step completed successfully in
222.5 seconds; the subsequent compile and runtime-closure collector reported
27 ELF files, 225 linked libraries, 150 direct runtime packages, and no direct
development packages. The loaded local image measured 547,466,584 bytes by
Docker's `.Size` field, a 59.669786% reduction from the same local Jazzy
baseline. Its current local image ID is
`sha256:57f9895e807c20c3a51063545cbf31e593346a99880eebe85955a82388448eee`.
This remains a local uncompressed-size proxy and identity, not a published OCI
measurement or digest.

An immediately repeated identical build completed in 0.31 seconds. BuildKit
reported every builder and runtime step as `CACHED`, including all six
manifest copies, the `rosdep` dependency layer, the full source copy, the
compile/collector layer, and the final runtime checks. A network-isolated
installed-product smoke check also passed the five CLI routes, five Python
imports, four installed helpers, source/build-tree absence, builder-tool
absence, exact clean provenance, and `ldd` closure for all 27 installed ELF
files.

The unchanged default image command was then run with no network against a new
output root and the same read-only 50-second fixture:

| Observation | Jazzy follow-up |
| --- | ---: |
| Container exit / observed wall time | 0 / 26.86 s |
| Manifest / diagnosis / receipt schemas | 3 / 3 `PASS` |
| Manifest status / lifecycle | `succeeded` / `complete` |
| Diagnosis / Autoware verification | `success` / 8 PASS, 0 WARN, 0 FAIL |
| First-map receipt | 7 / 7 `PASS` |
| Manifest-recorded artifacts re-hashed | 113 / 113 |
| Raw / corrected poses | 381 / 87 |
| Pointcloud tiles / tile bytes | 86 / 3,994,553 |
| Combined `map.pcd` bytes | 3,974,587 |
| Regular-file / allocated bytes | 8,308,096 / 8,613,888 |
| Final transaction directory / broken symlinks | absent / 0 |
| Output ownership | `1000:1000` |

`.ros_log/latest` now targets the existing sibling directory
`2026-08-10-19-13-19-649508-95db31a176dd-61` through a relative link. It is
not dangling and contains no `.partial` path. This closes local findings 5 and
6 for Jazzy. Compressed-OCI measurement, the dedicated-VM four-row matrix, the
full 277-second public demo, and the fixture publication decision remain
promotion gates; the dedicated Humble row must also confirm the shared fixes.

## Exact compressed OCI follow-up — commit `ff92f09`

The compressed-image experiment gate is now measured directly rather than
inferred from Docker's local `.Size`. Clean commit
`ff92f0950fe28e24de63db455c76fefa0039b03f` was exported for `linux/amd64`
with gzip OCI layers and the same title, description, license, revision, and
version labels used by the release workflow. Neither candidate was loaded
under a public tag or pushed.

The immutable v0.9.0 platform manifests were re-read as the baselines:

| ROS distribution | Immutable tag | Index digest | `linux/amd64` manifest digest | Layers | Compressed layer bytes |
| --- | --- | --- | --- | ---: | ---: |
| Humble | `ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble` | `sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53` | `sha256:3cbe706a339f01b1a3e022a9b38ca85e15249d67fbb85bc30252281c075bf16c` | 11 | 1,227,220,038 |
| Jazzy | `ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy` | `sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef` | `sha256:365b507b2ae8ccace86e6f73fe2d902a4b1f2cde217b973be9be9e011f8529f0` | 11 | 1,357,599,697 |

The candidate result is:

| ROS distribution | Candidate compressed layers | Uncompressed diffID bytes | Reduction | 25% ceiling | Margin | Gate |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Humble | 568,999,756 B | 2,015,947,776 B | 658,220,282 B (53.635066%) | 920,415,029 B | 351,415,273 B | `PASS` |
| Jazzy | 547,456,033 B | 1,848,688,128 B | 810,143,664 B (59.674709%) | 1,018,199,773 B | 470,743,740 B | `PASS` |

`scripts/measure_oci_archive.py` validates the archive before reporting a
byte. It rejects unsafe or duplicate members,
non-gzip layers, platform ambiguity, missing or unreferenced blobs, descriptor
size or SHA mismatch, decompressed diffID mismatch, and revision/version label
mismatch. It also binds the baseline reference, index digest, platform
manifest digest, and byte sum into the
[`oci-image-measurement-v1`](../../schemas/oci-image-measurement-v1.schema.json)
record. Both candidates passed 18 safe members, 14 verified descriptors, 12
verified gzip layers and diffIDs, and zero unreferenced blobs.

| ROS distribution | Candidate manifest | Config digest | OCI tar bytes / local SHA-256 | Measurement record / SHA-256 |
| --- | --- | --- | --- | --- |
| Humble | `sha256:8118d035c4b4ced92057a661db5983f9f43afc4837766b672c54539b61d5e990` | `sha256:bac3c758ad5d093360021bb63bdfb4c3c00eba847a1778adc234255af0b492c7` | 569,025,024 / `0f65c0fec70c6cbf44487afb4d6caea73e9269ad8d57adf7bffe1bf03f87920b` | [JSON](runtime-image-oci-humble-2026-08-11.json) / `6fe03086ac61c00e64426ed845dcbd18ac10848c1ad15cffcbfc02410e9b262c` |
| Jazzy | `sha256:3bdf3cbc808517784740238fb4eb9351a59b9a60320f20cb8e9baefa2ce1bbfc` | `sha256:8f4a0241fc9a844054457837d01c975e7b943274b1318ffdf7fc0bab5dd38958` | 547,481,600 / `328c7a5aa302d7de8e5045fc442f2456aac34d51ac742425989786cc30a73e19` | [JSON](runtime-image-oci-jazzy-2026-08-11.json) / `fba86dc84512dc5b751302dd5c52ec01d1b33e60d4c8d41e8d192b8e4f97f52c` |

The tar SHA-256 is a local audit identity, not the canonical image identity.
An identical cached Humble export completed in 8.82 seconds after the
871.05-second clean build. Every builder and runtime step was `CACHED`, and
the repeated manifest, config, layer descriptors, compressed byte sum, and
diffIDs were identical. Its tar SHA-256 was instead
`646369dee95f7debc2a7310e7dc5ecab11856616534cb36350ad6e7e38b295d7`
because BuildKit changed only the `index.json` created annotation from
`2026-08-10T22:36:38Z` to `2026-08-10T22:40:25Z`. The earlier Jazzy cached
export likewise completed in 6.40 seconds with the canonical image graph
unchanged. Reproducibility claims therefore use the manifest graph, not the
outer tar wrapper.

A separate linked-worktree repeat exposed a narrower cache boundary. The exact
main commit and both exact submodule commits reused the dependency layer, but
`COPY .` missed. That attempt was canceled after 47.77 seconds and produced no
OCI archive. Inspection found worktree-specific nested `.git` pointers, so
`.dockerignore` now excludes `**/.git` explicitly with a static contract test.

A scratch BuildKit probe after that repair compared the two exact-commit
contexts. All 1,427 included members had identical content SHA, mode, owner,
group, size, type, and link target; only checkout mtimes differed, and BuildKit
therefore emitted distinct source-layer digests. Cross-worktree compile-cache
reuse is not claimed. The 8.82-second result above is deliberately scoped to
an identical rebuild in the same unchanged worktree with all WIP safely
removed during measurement. Timestamp-normalized build contexts remain an
optional future optimization, not a release-correctness gate.

The measurement exports deliberately used `--provenance=false` so the index
contained exactly one runnable platform manifest. This does not weaken the
public release contract: the release workflow must still produce its SBOM,
maximum-mode provenance, GitHub attestation, installed-product smoke result,
and immutable digest records. The exact release candidate must reproduce this
runnable-manifest size gate before promotion.

This closes the local compressed-OCI blocker for both supported
distributions. Promotion still requires the dedicated-VM Docker/source matrix,
the full 277-second public demo, an authorized publication decision after the
[50-second fixture review](mid360-fixture-publication-review-2026-08-11.md),
a publicly resolvable candidate source revision, and the attested
release-candidate rerun. The fixture packet now passes its local review, but no
public artifact was changed.

No image, fixture, map, bag, private log, or geometry-bearing result was added
to Git or pushed by this pilot. Only privacy-bounded OCI descriptor evidence
was added. The next release decision must use the dedicated-VM evidence rather
than promote these local archives directly.

## Reproduce locally

Build from an exact clean checkout and inject the source identity explicitly:

```bash
docker buildx build --load --progress=plain \
  --build-arg ROS_DISTRO=humble \
  --build-arg LIDARSLAM_SOURCE_REVISION=<40-character-commit> \
  --build-arg LIDARSLAM_SOURCE_DIRTY=false \
  -t lidarslam-runtime-pilot:humble .

docker buildx build --load --progress=plain \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg LIDARSLAM_SOURCE_REVISION=<40-character-commit> \
  --build-arg LIDARSLAM_SOURCE_DIRTY=false \
  -t lidarslam-runtime-pilot:jazzy .
```

For the compressed gate, export instead of loading, then bind the candidate to
an immutable baseline identity:

```bash
docker buildx build --platform=linux/amd64 --provenance=false \
  --build-arg ROS_DISTRO=<humble-or-jazzy> \
  --build-arg LIDARSLAM_SOURCE_REVISION=<40-character-commit> \
  --build-arg LIDARSLAM_SOURCE_DIRTY=false \
  --output type=oci,dest=candidate.oci.tar,compression=gzip .

python3 scripts/measure_oci_archive.py candidate.oci.tar \
  --expected-revision <40-character-commit> \
  --expected-version <version> \
  --baseline-compressed-bytes <verified-byte-sum> \
  --baseline-reference <immutable-version-tag> \
  --baseline-index-digest sha256:<index-digest> \
  --baseline-manifest-digest sha256:<platform-manifest-digest> \
  --minimum-reduction-percent 25 \
  --output candidate-measurement.json
```

Use the published full public bag for release proof. If the unpublished
fixture is available for a bounded local check, mount its extracted bag
read-only, disable networking, use a new output directory, and invoke the image
without replacing its default command. Keep the geometry outside the checkout
and review privacy-bounded receipts before sharing them.
