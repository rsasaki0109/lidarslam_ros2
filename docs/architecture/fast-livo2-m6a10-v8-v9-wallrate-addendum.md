# FAST-LIVO2 M6a10 v8/v9 wall-time root-cause addendum

This record leaves the v8 closure immutable and defines the v9 source delta.
It is an implementation/preflight note, not an online-compute or estimator
result.

## Exact ordering and failure boundary

At the pinned FAST-LIVO2 revision (`0d2c0346107b75b59934975adec9a6eeeb913c64`),
`src/main.cpp` executes `ros::init`, constructs `LIVMapper`, calls
`initializeSubscribersAndPublishers`, and then calls `mapper.run()`. The v7/v8
base patch advertises the four M6a10 services through the ordinary node handle;
the v8 delta moves those service callbacks to a dedicated callback queue and
spinner. In `LIVMapper::run()` the order is:

1. `ros::spinOnce()` on the global queue;
2. `sync_packages(LidarMeasures)`;
3. when synchronization is false, `ros::Rate rate(5000)::sleep()`;
4. the next global `spinOnce()` only after that sleep returns.

The container wrapper sets `use_sim_time=true` before launching the mapper.
With no `/clock` message, the `ros::Rate` sleep is simulated-time dependent and
can prevent the global queue from reaching the sensor subscriber callback.
The v8 service spinner therefore makes service RPCs live, but it does not
guarantee sensor callback dispatch: the v8 closure records that the first
publisher call returned while callback polling still reached its deadline.

This is a scheduling explanation supported by source ordering and the retained
v8 receipt; it does not claim that no transport queue/drop occurred, nor that
the estimator crashed.

## v8 immutable accounting correction

The authoritative v8 closure is
`b94b96f153423964a01515ab0e0208a15eaf3f68fe263216726d89601165d7fd`.
Its correction records that the rosbag summary has a top-level total of
`236687` messages and per-topic entries of `5793`, `225102`, and `5792`; a
convenience parser had retained the last per-topic value (`5793`). The observed
completed publish and ACK counts remain zero. The separate v8 addendum records
the first-publish marker as diagnostic (`publisher_returned=true`), not as a
completed consumer count.

## v9 scope and limitation

`docker/patches/fast_livo2.m6a10-v2c-v9-wallrate.patch` changes only the idle
wait from `ros::Rate(5000)` to `ros::WallRate(5000)`. The nominal loop rate,
`spinOnce` ordering, sensor callbacks, queue contract, estimator, and mapping
algorithm are unchanged. The v9 image verifies and applies base, v8, and v9
patches in that order.

The v9 live preflight publishes one complete Ouster `PointCloud2` on
`/os1_cloud_node1/points`, then proves exactly one consumer callback, one ACK,
duplicate-ACK rejection, zero cross-topic counts, zero observed drops/overflow,
and callback latency no greater than 250 ms without any host mount. It does not
publish IMU/image prerequisites, does not call the final estimator barrier,
and explicitly makes no odometry/mapping claim.
