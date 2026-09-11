# Formal required-rival candidates for the competitive SLAM tracks

Audit date: 2026-08-25.  This is an additive design and upstream-source audit.
It does not edit `competitive_slam_v1.yaml`, the frozen selection, any receipt,
or the current r2 source-closure record.  It does not create a performance
claim.  The current evidence and legal state therefore remain **NOT_READY**.

## Decision boundary

The current objective has two different comparison contracts:

* LiDAR--IMU: beat GLIM under the CPU/GPU track conditions.
* LiDAR--IMU--visual: beat FAST-LIVO2 without dropping the camera contract.

The required systems in the current profile remain `ours`, `glim`, and
`fast_livo2`.  A system below is a *future candidate* only.  `CONDITIONAL_GO`
means that the candidate is worth implementing and can become required after
the listed gates; it is not a claim-eligible or legally reviewed result.
`NO_GO_CURRENT` means that the exact upstream closure cannot presently be used
for a formal comparison.  A candidate with only LiDAR input cannot replace a
visual rival merely because its license is easier to document.

## Audit method and immutable upstream observations

The audit used commit-addressed archives from the official upstream GitHub
repositories only.  The downloaded archives were checked as valid tarballs and
their source trees were hashed as sorted relative POSIX path plus NUL plus file
bytes (`relative_path_content_sha256_v1`).  The temporary audit root was
`/tmp/formal-rival-candidate-audit.42aePw`; it is not benchmark evidence and is
not a profile input.  No branch, tag, package manifest, or newer license was
used to repair an older commit.

| candidate | official immutable source | archive SHA-256 | source-tree SHA-256 | license artifact in that exact tree | current disposition |
| --- | --- | --- | --- | --- | --- |
| GLIM core | [`koide3/glim@faa264a1`](https://github.com/koide3/glim/commit/faa264a1bce1bda406f73457e35511f56cdc2eaa) | `d8176e85199a2297269d34fcfb57e4cb2c2d53e593439f665f614cae57972c25` | `2394e58c0c7fe218770b6db20cdab71395c885608783966ae94491e85899260e` | `LICENSE`, MIT, `e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4` | `CONDITIONAL_GO` for a new clean-room adapter |
| FAST-LIO2 / FAST_LIO | [`hku-mars/FAST_LIO@7cc4175d`](https://github.com/hku-mars/FAST_LIO/commit/7cc4175de6f8ba2edf34bab02a42195b141027e9) | `10fea840f01cb36c5c8922eb0d491864d940976e053ca31430ec2549c76ebd0c` | `284216657641378816d3039fa4ea376ec3aa906427f60895dea2ff5085757430` | root `LICENSE` is GPL-2.0-only, `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`; `package.xml` says BSD (`193b7e780cc9d39d178bfb1c5f16d8153e85e92eaaca564dc3cf98cad7461da1`) | `NO_GO_CURRENT` |
| Point-LIO default branch | [`hku-mars/Point-LIO@4b86a469`](https://github.com/hku-mars/Point-LIO/commit/4b86a469eb5572e70ed575af25b5f15dd06e8e3c) (`point-lio-with-grid-map`) | `5fef9d7c5b50ff8dad692b21bd998dffd881438e72c0b518644315319534d7ff` | `392efa0f9456057e8f9fc527b52f9016d6962c0b3cef45ab7ee3acd178b6e9e4` | root BSD-3-Clause-style `LICENSE`, `baf732b4dd36cdb1fcb6a2fe0c343cebe78a0f95adc3068f75844053a686447e`; vendored `include/IKFoM/LICENSE` is GPL-2.0-only, `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643` | `CONDITIONAL_GO`, legal review required |
| Point-LIO master | [`hku-mars/Point-LIO@1510c3bb`](https://github.com/hku-mars/Point-LIO/commit/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b) | `a8b3a427a4942e20db719220e1755eb24ba5fdbc3cee17d98846bee7479d28f2` | `c27e2e763756a7900443e3f4919274ae87fcbc19f44e804caad17ce23dc5197b` | root and vendored IKFoM licenses are GPL-2.0-only, `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`; vendored ikd-Tree has no license file | `NO_GO_CURRENT` |
| LIO-SAM ROS 2 | [`TixiaoShan/LIO-SAM@08af3f32`](https://github.com/TixiaoShan/LIO-SAM/commit/08af3f32f01725372d4269838dc44c19c6d9e76b) | `ae9ce10afcbda106f2a91fef480e7cf3151f94bed1597580aeba1432a6351833` | `20bd83c9d734f70b5c1ab736cbb0e932b42854673c129bf6dcf2ed549ffdb603` | root BSD-3-Clause `LICENSE`, `03434f61258c94068eb798bf4154a3ca95c61eab634886ba3d66d8f03c7c941e`; `package.xml` says `TODO` (`c47be593f8bff4f0600d6b17e79d44ccb45ae01738a6c585feb0f538ef196923`) | `CONDITIONAL_GO`, exact GTSAM closure required |
| KISS-ICP | [`PRBonn/kiss-icp@1ffa7d75`](https://github.com/PRBonn/kiss-icp/commit/1ffa7d7512f10bfc8b1185095011fa31184019e3) | `24148cc4370a8dfe378da8a3875fcd6d291005910dd7b3f364d7a79059559582` | `c0b5bc8ac6ee9873c379c2fe1a56274b00d92ea4566997e74bc059c3571c0d37` | MIT root `LICENSE`, `3e4bd5a7d241a40aaa6a8bdd8575c940ad86dc40baadb206b70143c658215ea1`; bundled Eigen/Sophus/TBB/tsl-robin notices are present | `CONDITIONAL_GO` for a separate LiDAR-only track; not current required set |

The exact Point-LIO master archive is reported separately because changing from
the default branch to master would be a new recipe decision, not a repair of
the default-branch candidate.

The two direct FAST-LIO dependency observations are also material:

* The exact `include/ikd-Tree` gitlink is
  [`hku-mars/ikd-Tree@e2e3f4e9`](https://github.com/hku-mars/ikd-Tree/commit/e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4),
  archive SHA `74f6e20d5b8e369bd20fede7bef2735d0e263658ee0b912a1f126dc5f63f7d59`,
  source-tree SHA `e79996fbe17ee357cd8f0014805e0fa63af30606e6cbb490115d8107b80d4402`.
  The exact tree has no `LICENSE`, `COPYING`, or `NOTICE` artifact.
* The Point-LIO vendored IKFoM bytes are GPL-2.0-only.  The official
  `IKFoM` toolkit reference is [`hku-mars/IKFoM@6cb2df40`](https://github.com/hku-mars/IKFoM/commit/6cb2df40d2cc7897d19f5e31402daff100b6d2a1),
  archive SHA `e9450fad1d3fccc16777abc962ef73ef720d65070761b6b99491063f549ac3f7`,
  source-tree SHA `a9e8209381e689b77b18f6c15659147878351f8ab4f1990444deed9d1c8fb328`,
  and `LICENSE` SHA `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`.  This confirms the component license; it does
  not convert Point-LIO's vendored bytes into a reviewed distribution.

## Dependency and recipe closure findings

These are source facts, not a claim that a dependency is unlicensed merely
because this audit did not pin it.

| candidate | exact closure work still required |
| --- | --- |
| GLIM core/adapter | The pinned core builds as C++17 and declares GTSAM 4.2, `gtsam_points` 1.2.2, Boost serialization, Eigen3, OpenMP, spdlog, and optional OpenCV/viewer dependencies. Each build-linked dependency needs its own commit/archive/license closure and base/toolchain identity. |
| FAST-LIO2 | The root GPL/BSD conflict cannot be settled by preferring `package.xml`. The recipe also names a moving `livox_ros_driver` installation and a branch-named ikd-Tree dependency. Exact driver commit/archive/license, the ikd-Tree legal artifact, and a ROS 2/ROS 1 adapter are missing. |
| Point-LIO | Both observed revisions use ROS 1 catkin, PCL/Eigen/OpenMP/Python and `livox_ros_driver`. The default branch embeds GPL IKFoM; master embeds GPL IKFoM and ikd-Tree bytes while retaining `.gitmodules` branch hints. A future recipe must pin the actual bytes and model GPL obligations component-by-component. |
| LIO-SAM ROS 2 | It uses GTSAM, PCL, OpenCV, Eigen, OpenMP, and ROS 2 packages. The README instructs an apt/PPA GTSAM installation rather than an immutable commit/archive. No exact GTSAM revision or license artifact is bound by the upstream commit. |
| KISS-ICP | The in-tree build contains notices for Eigen, Sophus, TBB, and tsl-robin, but the out-of-tree ROS CMake path fetches a version tag through `FetchContent`. A formal recipe must use the inspected commit archive, pin system package/toolchain bytes, and verify the same component closure rather than the moving tag. The ROS wrapper requires C++20. |

KISS-ICP's exact bundled notice hashes are: `cpp/kiss_icp/3rdparty/eigen/LICENSE`
`c83230b770f17ef1386ea1fd3681271dd98aa93646bdbfb5bff3a1b7050fff9d`,
`cpp/kiss_icp/3rdparty/sophus/LICENSE`
`018a55079028ffe2458f99da21c5a1f26551b1e20ca5d0ab06dd65ba74b21176`,
`cpp/kiss_icp/3rdparty/tbb/LICENSE`
`c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4`, and
`cpp/kiss_icp/3rdparty/tsl_robin/LICENSE`
`11f2c685ba565a31aef1219f63a7a8845602c97bff70016b9e3278dfe51a2ec5`.

No missing artifact is filled with a guessed revision or a package metadata
value.  A future candidate closure is invalid until every build-linked and
redistributed component is represented separately.

## GLIM clean-room/offline adapter

### Feasibility

Yes, the license-missing `glim_ros2` bridge can be removed technically.  The
adapter must be a new host-owned implementation written from the public API of
the pinned MIT GLIM core.  It must not copy, include, link, patch, or derive
source from the exact `glim_ros2` tree.  The result is still only a candidate:
the GLIM core's own C++17/dependency closure and a complete adapter evidence
gate are required before promotion.

The public core API observed at `faa264a1` is sufficient for a deterministic
LiDAR--IMU path:

* `glim/util/raw_points.hpp` defines `RawPoints` with scan stamp, per-point
  relative times, intensity, homogeneous point coordinates, colors, and ring
  numbers.
* `glim/preprocess/cloud_preprocessor.hpp` provides
  `CloudPreprocessor::preprocess(RawPoints::ConstPtr)`.
* `glim/odometry/odometry_estimation_cpu.hpp` and
  `glim/odometry/odometry_estimation_imu.hpp` provide the CPU/IMU estimator;
  `insert_imu(stamp, linear_acc, angular_vel)` and
  `insert_frame(PreprocessedFrame::Ptr, marginalized_states)` are the input
  boundary.
* `glim/odometry/async_odometry_estimation.hpp` provides the thread-safe
  `insert_imu`, `insert_frame`, `join`, and `get_results` executor.  Each
  `EstimationFrame` exposes `stamp`, `T_world_lidar`, `T_world_imu`, and the
  associated state/trajectory fields.
* `glim/mapping/sub_mapping.hpp` or
  `glim/mapping/async_sub_mapping.hpp` consumes estimation frames and exposes
  `submit_end_of_sequence`/`get_results` for map/submap materialization.

The adapter can therefore have two equivalent front ends: a ROS 2 message
converter for the live/replay shell and an offline feeder that reads the
already canonicalized LiDAR/IMU records.  Both must call the same conversion
and session code.  A visual mode must not be implied: optional OpenCV support
in the core is not evidence that a camera contract equivalent to FAST-LIVO2
exists.

### Input/output equivalence contract

Before any run, the adapter records and verifies:

1. exactly the preregistered LiDAR and IMU topic roles, message counts, header
   timestamps, frame IDs, and storage ordering;
2. every point's `x/y/z`, intensity, ring, and relative-in-scan time, with the
   time unit made explicit and no silent global-shutter substitution;
3. IMU acceleration/gyro units, timestamp basis, calibration/extrinsic
   convention, and sequence start/end policy;
4. the exact GLIM config, core/dependency closure, compiler/ABI tag, and
   adapter source/config hashes; and
5. output trajectory timestamps/frame convention and map/submap serialization
   semantics, including an EOF/drain marker and output tree hash.

The adapter must never read GT, invoke a scorer, load a saved map, or switch to
localization.  A semantic comparator may compare a retained historical bridge
fixture during development, but the bridge source and its artifacts cannot be
redistributed or used as the production implementation.

### Fairness instrumentation and gates

The adapter emits the same phase/consumer/EOF evidence as every other runner:
input counts and callback timing, process-tree RSS/cgroup/OOM identity, thread
and hardware identity, config/calibration/source hashes, output trajectory/map
hashes, and failure receipts.  It cannot claim fairness from log lines or
publisher counts alone.

The implementation order is:

1. static clean-room audit proving no `glim_ros2` include, library, path, or
   copied source;
2. exact GLIM core and all build-linked dependency closure, including the
   C++17/toolchain scope;
3. synthetic message conversion tests for fields, units, ordering, and EOF;
4. canonical HILTI/NTU role and calibration verification without GT access;
5. trajectory/map semantic and determinism tests; and
6. three complete matched runs per sequence for every assigned system, then
   the existing APE, map, RTF, RSS, and failure gates.

Until all six gates pass, the adapter is `CONDITIONAL_GO`, not a replacement
receipt or a profile mutation.

## Dataset and fairness compatibility

The current HILTI selection binds one Hesai PandarXT-32, one Bosch BMI085 IMU,
and five camera topics (`/hesai/pandar`, `/alphasense/imu`, and
`/alphasense/cam0..cam4/image_raw`) with official calibration.  FAST-LIO,
Point-LIO, LIO-SAM, and KISS-ICP are LiDAR--IMU or LiDAR-only systems; none of
them is a camera-equivalent FAST-LIVO2 competitor.  They may be evaluated on a
separate LiDAR--IMU projection of the same frozen input only if the projection,
calibration, message fields, and resource policy are precommitted for every
system.  The visual track must retain all required camera roles.

The NTU VIRAL proposal is likewise a fresh, unopened family with its selected
sequences and calibration roles precommitted separately.  Systems that accept
only one LiDAR/IMU stream need an explicit sensor-role selection when a dataset
contains multiple streams.  No candidate may silently select a convenient
sensor, use an additional camera/IMU, or reuse the development sequence.

For every candidate and sequence, the future runner must bind the same:

* selected LiDAR/IMU/camera roles, message fields, point-time basis, and
  calibration frame convention;
* CPU model, thread count, compiler/release, accelerator policy, and resource
  sampler/tool revision;
* initialization, online extrinsic estimation, downsampling, loop-closure,
  GPS, and map-save settings; and
* trajectory/map output contract and failure/timeout policy.

LIO-SAM's loop closure and GPS options are especially important: they must be
explicitly disabled or enabled by the same preregistered track policy, never
left to the upstream default.  FAST-LIO and Point-LIO expose PCD/map output,
but no visual input.  KISS-ICP publishes LiDAR odometry and local-map data but
does not consume IMU; it cannot satisfy the current LiDAR--IMU or visual
required-rival role.

## Go/No-Go matrix

| candidate | legal/source closure | input compatibility | map/output contract | maintenance/build risk | fair competitive role | decision |
| --- | --- | --- | --- | --- | --- | --- |
| GLIM clean-room adapter | MIT core artifact present; dependencies and adapter are not yet closed | Public API preserves LiDAR per-point time/ring and IMU; visual equivalence is unproven | Core exposes trajectory and submapping; semantic serialization gate required | New C++17 adapter and dependency closure | Strong LiDAR--IMU anchor if all gates pass | `CONDITIONAL_GO` |
| FAST-LIO2 | Root GPL text conflicts with BSD package metadata; ikd-Tree exact tree has no license artifact; driver is unpinned | LiDAR/IMU and point-time/ring paths; ROS 1 and Livox driver assumptions need adapter | Incremental map and PCD output | ROS 1, moving driver, legal ambiguity | Strong algorithmic relevance but cannot be claim comparator now | `NO_GO_CURRENT` |
| Point-LIO default | BSD root plus vendored GPL IKFoM; obligations and driver closure unresolved | LiDAR/IMU, point-time/ring, ROS 1; explicit extrinsic/units needed | PCD/map output; no camera | ROS 1 and branch choice; vendored dependency | Relevant LiDAR--IMU candidate after legal review | `CONDITIONAL_GO` |
| Point-LIO master | GPL root/IKFoM and no ikd-Tree license artifact | Same sensor limits; exact tree contains vendored components | PCD/map output | Larger legal and recipe burden | Do not select merely because revision is older | `NO_GO_CURRENT` |
| LIO-SAM ROS 2 | BSD root text present; package says TODO; GTSAM revision/license closure absent | ROS 2 LiDAR/IMU with point time/ring and mechanical-lidar assumptions | Save-map service and loop closure; policy must be frozen | GTSAM apt/PPA and heavy ROS/PCL/OpenCV closure | Relevant LiDAR--IMU candidate after exact dependency pin | `CONDITIONAL_GO` |
| KISS-ICP | MIT root and bundled notices are promising; out-of-tree tag fetch/system package closure remains | PointCloud2 LiDAR only; no IMU or camera | Odometry/local map, not the current IMU/map track | C++20 and moving `FetchContent` fallback | Separate LiDAR-only reference only | `CONDITIONAL_GO`, out of current claim |
| FAST-LIVO2 | Existing r2 legal conflict remains authoritative | Only audited visual target in current objective | Existing visual/map contract retained | Existing r2 recipe drift and legal blocker | Must not be replaced by LiDAR-only systems | `NO_GO_CURRENT`, retained |

No candidate receives a performance ranking here.  “Competitive relevance” is
an eligibility question; it is not a result.

## Recommended future required set and implementation order

The smallest fair formal set is track-specific:

1. **LiDAR--IMU:** retain GLIM as the target, implement the clean-room GLIM
   adapter, and add Point-LIO and LIO-SAM only after their independent closure
   and matched-run gates pass.  FAST-LIO2 remains a required target to resolve,
   not an eligible result while its exact legal conflict is open.
2. **LiDAR--IMU--visual:** retain FAST-LIVO2 and its camera contract.  None of
   FAST-LIO2, Point-LIO, LIO-SAM, or KISS-ICP is a lawful modality-equivalent
   substitute.  A future visual rival must be audited as a visual system, not
   inferred from LiDAR--IMU performance.
3. **Optional LiDAR-only:** KISS-ICP can be added as a separately labeled
   reference track after its exact C++20/toolchain and dependency recipe is
   closed.  It must not enter the current required-rival gate.

Recommended implementation order is GLIM adapter, Point-LIO closure and ROS 2
runner, LIO-SAM GTSAM closure and runner, FAST-LIO2 legal/dependency resolution,
then a genuinely visual rival.  This order does not authorize changing the
current profile or historical receipts.

## Future additive schema (proposal only)

If a new profile revision is authorized later, its candidate record should
require, at minimum:

```yaml
candidate_rival_set_v1:
  revision: <immutable revision>
  track_id: <lidar_imu | lidar_imu_visual | lidar_only>
  system_id: <stable id>
  official_source:
    repository_url: <immutable repository URL>
    commit: <40 hex>
    archive_url: <commit-addressed URL>
    archive_sha256: <64 hex>
    source_tree_sha256: <64 hex>
  dependencies:
    - role: <build-linked or redistributed component>
      repository_url: <official immutable URL>
      commit: <40 hex>
      archive_sha256: <64 hex>
      source_tree_sha256: <64 hex>
      license_artifacts: [{path: <exact path>, sha256: <64 hex>}]
  recipe: {base_image_digest: <sha256>, dockerfile_sha256: <64 hex>, build_script_sha256: <64 hex>, patches: []}
  input_contract: {modalities: [], roles: [], point_time_basis: <frozen>, calibration_sha256: <64 hex>}
  output_contract: {trajectory: <frozen>, map: <frozen>, loop_closure: <frozen>}
  fairness_identity: {hardware: <frozen>, threads: <integer>, resource_tool_sha256: <64 hex>}
  status: <CONDITIONAL_GO | NO_GO_CURRENT | READY_AFTER_REVIEW>
  required_for_claim: <boolean>
  not_ready_reasons: []
```

The schema must reject branch/tag/latest-only sources, missing component
licenses, moving dependency URLs, profile/runner drift, modality mismatch, and
recorded-but-not-revalidated input bytes.  A future schema or profile revision
must be additive and must mark old receipts `SUPERSEDED_RECIPE_REVISION`; this
audit does not perform that transition.

## Explicit remaining blockers

1. The current r2 closure remains legally and recipe-invalid as documented in
   `rival-source-legal-provenance-audit-r2-2026-08.md`; no r3 is issued here.
2. GLIM's clean-room adapter has not been implemented or independently built;
   its C++17 dependency closure and semantic equivalence evidence are absent.
3. FAST-LIO2 cannot pass until the GPL/BSD conflict, missing ikd-Tree license,
   moving Livox driver, and ROS 1 recipe are resolved by authoritative sources.
4. Point-LIO requires a chosen exact branch revision, component-scoped legal
   review, and ROS 2/input runner work.
5. LIO-SAM requires an exact GTSAM source/license/build pin and an explicit
   loop/GPS/map policy.
6. No audited candidate except FAST-LIVO2 has a visual contract, and FAST-LIVO2
   itself remains `NOT_READY`; therefore the visual claim gate cannot be closed
   by this candidate list.
