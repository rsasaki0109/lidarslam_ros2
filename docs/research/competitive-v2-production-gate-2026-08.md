# Competitive-v2 / Candidate2 production gate (2026-08-01)

## Decision

- **competitive-v2 frontend/backend transport profile:** **promote**. The clean
  HILTI gate passed on exp01 and on three deterministic exp04 repetitions:
  every RTF was below 1.0, accuracy improved against the recorded baseline,
  and all three exp04 trajectories were byte-identical.
- **Candidate2 backend:** **do not promote** under the frozen
  `competitive_slam_v1` contract. Only `spms_01` remained without an exposed
  candidate result, and even that slot is retry-only rather than pristine;
  the contract requires three assigned, frozen holdouts and three wins. In
  addition, the frozen manifest identifies a dirty source-tree hash rather
  than a reviewable commit, and its enabled planar-map refinement is not part
  of the isolated Candidate2 delta.

This decision separates implementation readiness from benchmark eligibility.
The code below is reviewable and builds in isolation; a successful regression
run cannot retroactively make a previously consumed sequence an unseen
holdout.

## Reviewable source boundary

| Area | Revision | Scope |
| --- | --- | --- |
| current backend scheduling | `de2f394` base | event-driven loop scheduling and tests already present in current `develop` |
| current backend transport | `0915034` | exact/approximate stamp policy, selectable reliable cloud QoS, and queue headroom in the Pimpl backend |
| current frontend release pin | `99b55dc` | RKO-LIO PR #12; competitive-v2 pruning/queue changes on the v0.3.2 configurable-voxel line |
| original measured frontend | `60b861a`, `4619565` | sparse-grid pruning and signed-boundary brute-force contract tests used by the first isolated gate |
| current production profiles | `67231b6`, `acf7249`, `c590e29`, `9e19b72` | HILTI/NTU profiles, deterministic transport, and explicit RKO output topic |
| benchmark overlay support | `7fc63a8` | preserve an explicitly sourced isolated overlay |

The merge branch is `agent/kaizen-production-gate-current`, based on current
`develop` revision `de2f394`. The original measurement branch was
`agent/kaizen-production-gate-20260801`, based on `3d44bc5`. Benchmark/build
directories are not source changes.

## Build and unit verification

- RKO-LIO release-pin core, `offline_node`, and `online_node_component`: built
  and linked from the isolated submodule worktree.
- Current `test_voxel_hash_map`: 4/4 cases passed, including exact signed
  boundary comparisons against brute force.
- `test_loop_search_schedule`: 3/3 passed.
- `test_graph_slam_config`: 5/5 passed, including exact/reliable overrides.
- Benchmark script safety tests: 26/26 passed.
- `graph_based_slam` production targets: colcon build passed in the isolated
  overlay.
- Overlay provenance was checked with `ros2 pkg prefix`; both `rko_lio` and
  `graph_based_slam` resolve inside the isolated worktree.
- Runtime startup confirmed `exact-stamp-synced, queue 256, cloud reliable`.

## Clean HILTI regression gate

Measurements must begin only with load1 <= 2.0 and sampled CPU busy <= 10%.
Runs performed while another benchmark owns the CPU are invalid.

| Sequence | Candidate RTF | Raw APE RMSE | Baseline RTF | Baseline APE RMSE | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| exp01, run 1 | 0.832433 | 0.06080387 m | 0.9783 | 0.06221018 m | pass (-2.26% APE) |
| exp04, run 1 | 0.439305 | 0.04836338 m | 0.5037 | 0.06739868 m | pass (-28.24% APE) |
| exp04, run 2 | 0.423233 | 0.04836338 m | 0.5037 | 0.06739868 m | pass (-28.24% APE) |
| exp04, run 3 | 0.423418 | 0.04836338 m | 0.5037 | 0.06739868 m | pass (-28.24% APE) |

The gate is RTF <= 1.0 and no accuracy regression above 2%.

### Measurement provenance and reproducibility

Accepted current-develop artifacts are under
`competitive_ours/kaizen_current_9e19b72_20260801`, measured at top revision
`9e19b72` and RKO-LIO release revision `99b55dc`. Each run began only after three
consecutive preflight samples satisfied load1 <= 2.0 and CPU busy <= 10%.
All runs exited 0, used exact-stamp pairing with queue 256, and produced
complete trajectories. exp01 produced 2277 poses. Each exp04 run produced
1258 poses; median RTF was 0.423418 and maximum RTF was 0.439305. The last
poses were 0.043 s (exp01) and 0.017 s (exp04) before the respective bag ends,
inside the strict 0.25 s completion margin. The explicit zero base-to-reference
offset preserves the legacy HILTI scoring frame while satisfying the current
reference metadata contract.

The exp01 raw trajectory SHA-256 is
`4bfbb5dd7b5928d6820095b757d2ad5ca76f942e738bee1349004965cda5f8dc`.
All three exp04 raw trajectories have SHA-256
`760cd0e3234988ff8eb7f072f182f0897abe2381debe11126fb618908dcd1ce4`;
all three densified passthrough copies have SHA-256
`fb304781eab2a24a3b580ab4c9159fe2c9b3f618a226b36867d6c16d92dd91f2`.
This satisfies the RTF, accuracy, completeness, and deterministic-repeat gate.

The earlier isolated `4619565` gate also passed (exp01 RTF 0.695946; exp04
maximum RTF 0.355620). It remains useful historical evidence, but the promotion
decision above is now supported directly by the current release pin.

## Holdout eligibility audit

| Dataset | Eligibility | Reason |
| --- | --- | --- |
| HILTI exp02/exp03/exp21 | consumed | explicitly used for candidate-1 loop/overlap development |
| HILTI exp01/exp04/exp07 | regression only | used during frontend/backend development |
| NTU-VIRAL rtp_01 | consumed | candidate runs, including a first-130-second run, already inspected |
| NTU-VIRAL tnp_02 | consumed | candidate runs, including a first-130-second run, already inspected |
| NTU-VIRAL spms_01 | retry-only | three infrastructure-failed attempts produced zero poses; no candidate metric was exposed, but the slot is not pristine/unopened |

The frozen Candidate2 manifest records correction caps of 0.5 m and 2 deg,
minimum overlap 0.76, and loop-search stride 1. Its formal profile requires
three frozen holdout slots and three wins. The local dataset history leaves
only one eligible sequence, so the formal promotion contract cannot be
satisfied without registering and freezing two genuinely new sequences before
examining their results.

The manifest pins repository revision `3d44bc5` and RKO-LIO revision
`689106a`, but separately fingerprints an 845-file dirty source tree. It also
enables `use_planar_map_filter`; that implementation belongs to the broader
backend research WIP and is absent from the isolated loop-scheduling commit.
Consequently, the frozen artifact cannot be reconstructed from its recorded
Git revisions alone. Candidate2 needs a new clean freeze after its map-export
delta is reviewed, even before the missing-holdout requirement is considered.

## Diagnostic evidence (not an unseen holdout)

The recoverable `rtp_01/ours_lio/run_01` trajectory completed 1739 frontend
poses and 99 graph anchors. Two loop edges were accepted: 28->94 and 37->98.
Their corrections were 0.0965848 m / 0.197824 deg and 0.133148 m / 0.531555
deg, with overlaps 0.809863 and 0.780715 respectively.

Post-hoc dense scoring using the frozen body-to-prism translation
(-0.293656, -0.012288, -0.273095) produced:

- raw APE RMSE: 1.8969631489 m
- dense corrected APE RMSE: 1.8929973655 m
- relative change: -0.209% (non-harmful)

This is useful safety evidence but is neither a complete formal run nor an
unused holdout.

## Retry-only holdout diagnostic

`spms_01` was run once as a clean isolated retry-only diagnostic at top
revision `8908e60` / RKO-LIO revision `4619565`. The frozen two-file input-tree
hash was recomputed as
`107980ff0a1992570cc87e46bca824324478c92358838a2fe0c3130d602a5195`,
matching the manifest before result inspection. The run began after three
quiet preflight samples and exited 0.

The 418.172 s bag produced 4181 poses in 340.115 s (RTF 0.813338). Its last
pose was 0.081 s before the bag end, inside the 0.25 s completeness margin.
Raw APE RMSE was 3.308048 m with 6242 timestamp pairs. The backend evaluated
74 best candidates and logged 209 candidate rejections; all observed overlap
ratios were zero, the minimum best-candidate fitness was 0.784896 against the
0.7 threshold, and **zero loop edges were accepted**. Raw and corrected
trajectory files are therefore byte-identical, with SHA-256
`50ef8678b38608c85660475c73d76c1d2870337f40a08fd3e7826b5a77e3b76c`.

The generic metrics file also reports a 3.587707 m "corrected" score, but that
invocation used a 10 s association gap while the raw score used 0.05 s. Since
the underlying trajectories are identical, that number is not a correction
effect and is excluded from the gate decision.

This result demonstrates safe rejection and real-time completion only; it
does not establish a Candidate2 win and is not a pristine unseen holdout.
Candidate2 remains a **no-go** until its dirty map-export delta is isolated and
reviewed, a new clean freeze is created, and two additional genuinely new
holdouts are registered so the required three-holdout/three-win contract can
actually be evaluated.
