# FAST-LIVO2 M6a10 v7 root-cause addendum

This addendum is the immutable root-cause record for the single authorized
v7 replay.  It is evidence-bound, input-only, and does not promote a replay
result.  A later attempt must reference this file and add a new receipt; it
must not rewrite the v7 closure.

## Finding

The v7 attempt failed before its first publish because the feeder's bounded
`/m6a10/consumer_status` RPC could not be serviced.  The exact closure is
`FAIL_CLOSED`, with `failure_kind:
consumer_status_rpc_timeout_before_first_ack` and
`failure_stage: feeder_status_rpc_before_first_publish`.

The cause is a callback-queue starvation interaction, not a bag, scorer, GT,
or memory failure:

1. The v7 source patch advertised `/m6a10/consumer_eof`,
   `/m6a10/consumer_finalize`, `/m6a10/consumer_status`, and
   `/m6a10/consumer_ack` through the ordinary `nh.advertiseService(...)`
   path (`docker/patches/fast_livo2.m6a10-v2c.patch`, `LIVMapper.cpp` hunk
   around lines 515--524).
2. The same patch's `run()` loop called `ros::spinOnce()` and then entered
   `rate.sleep()` whenever `sync_packages(...)` returned false (the hunk
   around lines 529--541).  The v7 wrapper had set `use_sim_time true`; before
   a `/clock` message exists, that sleep can block while the global callback
   queue is not serviced again.
3. The feeder log begins with `FAST feeder first publish
   topic=/os1_cloud_node1/points`, then records `DeadlineExceeded: consumer
   status RPC deadline expired`.  Its progress marker remains
   `phase: publish_waiting_for_callback`, with zero published and zero
   acknowledged records and `publish_call_started_at: null`.

Therefore the status request was made before the first `publisher.publish`
call and could not reach the service callback.  No callback ACK, trajectory,
or online-compute result was observed.  The bounded service-queue/spinner
delta in v8 moves only the four benchmark service callbacks to a dedicated
`ros::CallbackQueue` serviced by one `ros::AsyncSpinner`; sensor callbacks and
the FAST-LIVO2 algorithm remain on their original queue and retain their
algorithm path.

## Immutable evidence bindings

| Evidence | SHA-256 | Relevant fact |
| --- | --- | --- |
| v7 closure receipt | `a4e36e2fd56085f14f3d5c7fcea56a2f6e6c3586d3b2d957557eeac3835e171b` | failure kind/stage and zero observed records |
| feeder log | `01c629bd7bd11b4ec3bb576c999f40af9407a0c2d3d629802bcf3e08a29bce9e` | status RPC timeout before publish |
| feeder progress | `cd35fedf656c23ed15bf3fa2eeb6889a39f34a12f3504f0a84ece6e9db4497a0` | publish-waiting, zero publish/ACK, publish not started |
| mapper log | `f618dc142e445c8bf9b3c5c58cea4a2c05b28ad9d95dc987af7b3b338840be6e` | parameter startup only; no sensor callback evidence |
| phase evidence | `08e32f1340bf13394fd62d7b53fe2f296e33b487afbea082ef647df71cede6a5` | invalid: missing `input_end` |
| phase events | `53353f0c4192511cb235f97575e80b55940caeaf68d01bb2ed26ac3ec62a11e8` | input start observed; input end/drain absent |
| v7 base patch | `33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297` | byte-identical to the v7 build `patch.snapshot` |

The v7 image identity was
`sha256:69dc479c131b25bda63ede9e0ab7f12d7654f2c4aaa69f8440db0e57ccd96b45`
and its preregistered patch binding was the base-patch hash above.  The v8
delta is a separate file and must never be folded into or overwrite that base
patch.

## Scope of the v8 correction

The v8 delta is restricted to service callback scheduling, a mutex-protected
queue-counter snapshot, and shutdown ordering.  It does not change FAST-LIVO2
state estimation, mapping, sensor message contents, or algorithm parameters.
The v8 image build verifies the base hash, verifies the delta hash, applies
the base first and the delta second, and records both hashes in image labels.
