# v44e raw shadow replay execution contract (2026-08-10)

## Result

v44e defines the first executable boundary around the isolated v44 fixed-lag
shadow estimator. It does not run that estimator on raw data. The gate binds
one read-only adapter to the exact three v44a canonical bags, their topics,
frames, serialized stream digests, and fixed sensor calibrations. It also binds
one retained v17 state trajectory and one retained v17 map per sequence as
hash-only protected artifacts.

The machine-readable contract is
`configs/sota_v6/development/v44e_raw_shadow_replay_execution_contract.json`
(SHA-256 `08d7f8850f5081fd18d58a860cae3d9566846605460dadb37660b3c3111d1692`).
The adapter is `scripts/v44_raw_shadow_replay_adapter.py` (SHA-256
`6fbf16d811a5bcf26c2cac9190f0557844cd3596cf7704cfdc016c7080ddf0ee`),
and the static auditor is
`scripts/audit_v44e_raw_shadow_replay_contract.py` (SHA-256
`8fa736d6c59b17eb3364c50cad0a9ec88cf01ffe867864e9558387db37909389`).

On two repeatable static passes, the sealed decision is
`AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION`. This authorization is
limited to six report-only runs: two repetitions of each exact source. It does
not authorize accuracy scoring, ground truth, reference maps, ROS publication,
primary trajectory/map/bias writeback, loop closure, global correction, or a
fresh holdout.

## Exact source and protection boundary

| Sequence | LiDAR / IMU | Receive watermark | LiDAR-to-body translation | Protected v17 state / map SHA-256 |
|---|---|---:|---|---|
| `navinst_indoor02` | `/velodyne/lidar/points` / `/novatel/imu` | 3,573,549 ns | `[0, 0, 0.21941]` m | `81054c8c…` / `be95206f…` |
| `oxford_spires_keble_05` | `/hesai/pandar` / `/alphasense_driver_ros/imu` | 135,957,579 ns | `[-0.0006517, -0.0155103, 0.0846327]` m | `adc37961…` / `2d088933…` |
| `urbannav_hk_tunnel_1` | `/velodyne_points` / `/imu/data` | 136,013,359 ns | `[0, 0, 0.28]` m | `15c15de3…` / `87caad98…` |

The full bag SHA-256 values remain the v44a bindings:

- NavINST: `b8afd9649a310669dcc39737a8f3e1d40f00adfa619ff61afda712c2924fc8ff`
- Oxford: `8e7660e600f1f52758e0301f861a2174500ef38f60df5ac9f9d7198df9c54217`
- UrbanNav: `95524232d9a4c2781d919a9dc1c16f6037cd88dd4d4d40bf9958ab0a219297a1`

The adapter hashes a bag before opening it, verifies its exact LiDAR and IMU
connection counts, and recomputes the separate serialized LiDAR and IMU stream
digests while reading. Bag metadata must be unchanged at close. It hashes both
protected v17 artifacts before and after the shadow run and gives only those
digest dictionaries—not paths or contents—to the estimator's
`ProtectedOutputGuard`.

## Decoder and event-order contract

The canonical PointCloud2 decoder accepts only the v44a 48-byte little-endian
layout. It passes `x`, `y`, `z`, uint32 nanosecond `t`, and `ring` into immutable
core records. The IMU decoder passes only header time, angular velocity, and
linear acceleration. Message orientation and every covariance field are
intentionally unread.

ROS bag receive order is not estimator order. Oxford and UrbanNav LiDAR
messages arrive roughly one scan later than their header time. The adapter
therefore holds a bounded 128-message / 32 MiB heap and uses each source's
exact v44a maximum `receive_stamp - header_stamp` as a watermark. Events are
emitted by `(header_stamp_ns, IMU-before-LiDAR, source_index)`. A larger delay,
non-monotonic receive time, duplicate/out-of-order event, or capacity overflow
terminates the shadow run.

Every emitted record is decoded exactly once. The estimator never receives a
sequence ID, dataset name, bag path, accuracy input, protected artifact path,
or output callback. Source-specific logic ends at topic/frame decoding and the
sealed `T_BL`; all algorithm parameters remain the one global v44b contract.

## Runtime and output boundary

After every scan record produced by the core, the adapter measures process RSS
and cumulative processing wall time relative to elapsed sensor time. The core's
existing resource method enforces `RSS <= 330 MiB` and processing
`RTF <= 0.85`; either violation is terminal. The receive reorder buffer,
serialized message size (`<= 8 MiB`), active estimator allocations, and output
bytes are independently bounded.

The CLI accepts only:

```text
replay --contract CONTRACT --sequence-id ID --repetition {1,2}
```

There is no bag, output, trajectory, map, or calibration path option. The
contract derives the exact input and writes exclusively to:

```text
sota_v6_dev_v44e_raw_shadow_replay_contract_20260810/
  raw_replay/<sequence_id>/run_NN/
    diagnostics.jsonl
    run.json
```

The run directory must be new, real (not symlinked), and under the exact
evidence root. The two filenames are opened exclusively and share the v44b
256 MiB diagnostic ceiling. No overwrite route exists.

## Authorization chain and static audit

Opening a bag is impossible until the adapter verifies all of the following:

1. Its own source, the v44 core, v44b architecture, v44a inputs/readiness, and
   v44d source-gate hashes match the execution contract.
2. The v44e aggregate has the exact contract, adapter, and auditor hashes and
   the required PASS decision.
3. Both hash-listed source reports behind that aggregate are re-read, have
   repetitions 1 and 2, have identical deterministic payloads, and state that
   no raw bag was opened by the static gate.

The static auditor verifies 29 source/binding checks and runs 10 synthetic
adapter probes. The probes cover exact IMU and PointCloud2 decoding, malformed
schema and message-size rejection, watermark order and capacity, all three
extrinsics, closed authorization, protected-output identity, and exclusive
bounded writing. The focused adapter/auditor suite passes 49 tests in 4.89 s,
and the v40--v44e regression passes 261 tests in 24.69 s. Static validation
hashes protected v17 artifacts but does not import `rosbags`, open a raw bag,
or execute `replay`.

Two final standalone validations produced the same deterministic report
payload SHA-256
`b98739d779a6e79b2027f100172230f48700b4045dc4b8610b3b54e5fe503d3f`.
Their report SHA-256 values are
`27d98bb4fe182efa5c38ce97f5ee8f876e588ca353e0a393be4339d8cf26bae9`
and
`30a9ba4a02399278e2682e9acdd45daeab82f038eb8b1cebb30763d5491082a4`.
They started at 34.63--34.73 MiB RSS, peaked at 43.16--43.23 MiB, and added
8.50--8.53 MiB. The aggregate JSON SHA-256 is
`5d3516edcadea924c5481688666edddec2db7b2be61eaa5cf42f7c52cc278181`,
with deterministic aggregate payload SHA-256
`7550b7303021d8440b749dc2b19279f3cdb66c8c6dfade6f3d2a27a64c91c10e`.
Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44e_raw_shadow_replay_contract_20260810/`.

## Next gate

The next action is the six authorized raw shadow runs only. Each sequence must
complete twice, retain unchanged protected v17 hashes, stay below both runtime
bounds after every scan, consume the exact bound message and point counts, and
produce the same non-null core state payload SHA-256 across repetitions. A
single failure rejects the raw shadow route. Even a six-run PASS would authorize
only a separately defined accuracy-screen contract; it would not itself open
ground truth or promote the estimator.
