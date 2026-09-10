# Competitive rival source/legal-provenance audit: r2

Audit date: 2026-08-25 (read-only; no Docker, build, bag, GT, or evidence
volume access).

This is an additive audit of the current
`evidence_gate_v2.rival_source_closure` record.  It does not rewrite the
closure, the selection sidecar, a receipt, or any historical result.

## Disposition

The current record remains:

- closure: `competitive-rival-source-closure-2026-08-r2`, revision `2`;
- current selection: `competitive-execution-selection-2026-08-r2`;
- selection SHA-256:
  `941d7739aa1ed72bc10b0b7b81e8569f29364e6a8149fe776a72aa93dcfc8d6b`;
- closure-identity SHA-256:
  `e876339f843b28c1be5d835e8f6a9a062c52715498c85d93afbdb83c6dbae4e8`;
- legal status: `NOT_READY`, with image publication and claim eligibility
  blocked.

No additive r3 can be issued from this audit.  Four pinned components still
lack an authoritative, reopenable license artifact or have conflicting legal
metadata.  A package manifest's `<license>` value is metadata only and is not
treated as the license text.

## Official commit/archive verification

Each commit-addressed archive below was downloaded once from the official
GitHub archive URL into a fresh temporary audit directory and checked for a
valid archive.  The archive and relative-path/content source-tree hashes
match the values already recorded by r2.  The temporary download is evidence
for this audit only; it is not a new benchmark input or a profile pin.

| component | official immutable commit | archive SHA-256 | source-tree SHA-256 | authoritative license artifact | result |
| --- | --- | --- | --- | --- | --- |
| GLIM core | [`koide3/glim@faa264a1`](https://github.com/koide3/glim/commit/faa264a1bce1bda406f73457e35511f56cdc2eaa) | `d8176e85199a2297269d34fcfb57e4cb2c2d53e593439f665f614cae57972c25` | `2394e58c0c7fe218770b6db20cdab71395c885608783966ae94491e85899260e` | `LICENSE`, MIT, `e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4` | READY |
| GLIM ROS 2 bridge | [`koide3/glim_ros2@4a9e7a4c`](https://github.com/koide3/glim_ros2/commit/4a9e7a4cb084967c8525a1be529ad3ba2a118ae7) | `cdb99188e20c93b20ed86c4b95a42643c1a659cd4e9e06d7cc3ffb2639aeae17` | `b5bc8504577d1fda52289e8e627da857f0834fa81b228ec9c8feaeb3118d9f52` | none in the exact tree; `package.xml` says MIT (metadata SHA `eb9505cc2aa44e11900f708c02d8364442cbe57666531e9d91e570e593abcbb1`) | `NOT_READY_LEGAL_PROVENANCE` |
| FAST-LIVO2 root | [`hku-mars/FAST-LIVO2@0d2c0346`](https://github.com/hku-mars/FAST-LIVO2/commit/0d2c0346107b75b59934975adec9a6eeeb913c64) | `1da2652f835f2efb02c0992bc4d64e8dcde48571e538f35dddaf6ba6a61c17d1` | `4b7df0a03cbfb5c4d46eee86b408d61045e1f34db7187e65af751b97cd7dc14b` | root `LICENSE` is GPL-2.0-only, `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`; `package.xml` says BSD (metadata SHA `0839b4814f283626faafc65a9bed37da754fe04092fcfc1859e52330fe206bde`) | `NOT_READY_LEGAL_PROVENANCE` |
| rpg_vikit / `vikit_common` | [`xuankuzcr/rpg_vikit@6c886c8e`](https://github.com/xuankuzcr/rpg_vikit/commit/6c886c8e5d83997806e00294826d528cea3581dd) | `8162d0b616c443118a52e654394d3eb0807d6dac3b782f4dc3d6823c8815fede` | `3ae228148396236869cc8f64837431858f8d2ca333d39afdae649fd6f3114b5f` | no license/copyright/notice text; `vikit_common/package.xml` says GPLv3 (metadata SHA `557a69ea7c36a2b5a3142466fa8c392e0db6707d310d57447bb091f3085f7328`) | `NOT_READY_LEGAL_PROVENANCE` |
| rpg_vikit / `vikit_py` | same pinned commit | same archive | same tree | no license text; `vikit_py/package.xml` says BSD (metadata SHA `d91466ba0fc4814114f843b2020aae6853a46acca24cbb1c3844eae82e7a93ff`) | `NOT_READY_LEGAL_PROVENANCE` |
| rpg_vikit / `vikit_ros` | same pinned commit | same archive | same tree | no license text; `vikit_ros/package.xml` says GPLv3 (metadata SHA `52424cae1abfa5e5c7ad0109e636c1c287473a132a6fdd31bbd2629d2e78054f`) | `NOT_READY_LEGAL_PROVENANCE` |
| Sophus legacy dependency | [`strasdat/Sophus@a621ff2e`](https://github.com/strasdat/Sophus/commit/a621ff2e56c56c839a6c40418d42c3c254424b5c) | `67deb7b0ea38ac6a44967ceee2875507ea9d64a352195080223d5ca3e1584fa4` | `383fcd09914533085a82e7767790fa61c05c43f0d7973ba6b98b4111c40c2122` | no license, copying, copyright, notice, or package-license artifact in the exact tree | `NOT_READY_LEGAL_PROVENANCE` |

The exact GLIM ROS 2 and rpg_vikit trees have no license-like file in the
official commit archives.  The fetched upstream histories also contained no
such artifact for those projects.  FAST-LIVO2 has a real root license file,
but its BSD package declaration conflicts with the GPL-2.0-only text.  The
conflict cannot be resolved by choosing the more convenient declaration.

Sophus illustrates why a newer upstream license cannot be copied backward:
the fetched current main commit is `d0b7315a0d90fc6143defa54596a3a95d9fa10ec`
and has `LICENSE.txt` (MIT, SHA-256
`22ba6b6434263aea3a4c80328257e8b23ff3148930d3ce8320cc1cb33e085c5d`), but
that is not the pinned `a621ff2e...` source.  It is only a rebuild candidate,
not evidence for the legacy dependency.

## Docker recipe audit

The r2 Dockerfile, build-script, wrapper, patch, and configuration bytes
remain as recorded for the common recipe fields.  The current worktree has
two runner-byte drifts that the offline checker correctly rejects; r2 is not
silently repaired:

| runner | r2 declared SHA-256 | current worktree SHA-256 | disposition |
| --- | --- | --- | --- |
| `scripts/run_glim_benchmark.py` | `2345b930ce8b679d92d315bae8ed372d99fde1c43fb744f5d8b7c7fc77588794` | `1cb5a1e7354c168ae2317ff96e9450fb9e7e93b1314db8690bc63177e04913a1` | recipe drift; r2 checker `INVALID` |
| `scripts/run_fast_livo2_benchmark.py` | `b8a57115aba296e31ee53e380052d5a35eb8df4fbb72e8797ae65bab694dbd30` | `eda2373a1f04098851ffc48ccb0b1d8ba282b8b84c2a3f8102e18e3b45284352` | recipe drift; r2 checker `INVALID` |

The exact Dockerfile hashes are `dcd01d013eeb5cb50e2582646548c06f2c3b1a93820412b110b7fb25b769d70d`
for GLIM and `1b916086ce40f386ea1a4bb2dc591f784c249c81723f6d7e486bca217a9b3da5`
for FAST-LIVO2.  The shared build script is
`136f52c41da1b090f47c310bb3ce85fd8c317e4824410cecba960815c7ad42d1`; wrapper
hashes are `6b9948a1319ba5eaeba1bd53fca9d35ae6b3f93cda41ebea3e8d79a2b467b19e`
and `f4236ec79659becacb12dc5f76c2d7ef41bc83be7616df88c1ee6ebc60e65d20`.
Patch hashes are `f3e7549ee1730df37125f17c4be00b5643a6b9db9eacbc96c9ecb2f682b08867`,
`ede64c7a00f409a19138f41c19685f08d58b5c8c8d9d9ffa47b6c7dcb774c0f2`, and
`33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297`.

The current checker result is therefore `INVALID`, with the two runner drift
errors above and the declared legal `NOT_READY` reasons.  This audit does
not update old recipe hashes or reinterpret any receipt.  A future additive
closure must bind a fresh selection and all current recipe bytes after legal
provenance is separately resolved.

## Resolution matrix and clean-room GLIM path

The following classification is deliberately narrower than a legal opinion:
`A` means an exact, reopenable upstream artifact is already present; `B` means
the exact upstream record is ambiguous or missing and needs rights-holder or
custodian clarification; `C` means a component can be removed only in a new,
behaviorally equivalent recipe; and `D` means replacement or repinning would
change the selected rival and therefore needs an explicit new selection.

| component | exact primary-source observation | category | safe resolution now |
| --- | --- | --- | --- |
| GLIM core | `koide3/glim@faa264a1...` contains the MIT `LICENSE` artifact recorded above | A | No legal blocker for the core itself; keep the exact commit and hash |
| `glim_ros2` in r2 | `koide3/glim_ros2@4a9e7a4c...` has an MIT `<license>` in `package.xml`, but no license text in the exact archive | B / future C | Do not infer redistribution permission. A future host-owned clean-room ROS 2 adapter may remove this bridge, but only through a new recipe/selection revision |
| FAST-LIVO2 root | The exact `LICENSE` and README state GPLv2 while exact `package.xml` states BSD | B | Require upstream rights-holder clarification bound to this commit/component; editing or preferring `package.xml` is not a repair |
| `rpg_vikit` components | Exact `vikit_common`/`vikit_ros` metadata states GPLv3 and `vikit_py` states BSD; the exact archive has no license text | B | Require component-scoped upstream clarification, or prove a new FAST recipe does not build/link these components; no current removal |
| Sophus legacy | Exact `strasdat/Sophus@a621ff2e...` tree has no license/copying artifact; the newer main `LICENSE.txt` is a different revision | B / future D | Do not back-port the newer notice. A newer Sophus pin is a new compatibility/recipe decision and cannot repair r2 |

The downloaded official archives in `/tmp/rival-license-audit.NFAgzM` matched
the r2 archive and tree hashes; the archive inspection found no unsafe member,
symlink, or hardlink.  This temporary audit root is not benchmark evidence and
is not a profile input.  The official primary records used for the ambiguous
entries are the exact [FAST-LIVO2 commit](https://github.com/hku-mars/FAST-LIVO2/commit/0d2c0346107b75b59934975adec9a6eeeb913c64),
[rpg_vikit commit](https://github.com/xuankuzcr/rpg_vikit/commit/6c886c8e5d83997806e00294826d528cea3581dd),
[Sophus commit](https://github.com/strasdat/Sophus/commit/a621ff2e56c56c839a6c40418d42c3c254424b5c),
and [GLIM ROS 2 commit](https://github.com/koide3/glim_ros2/commit/4a9e7a4cb084967c8525a1be529ad3ba2a118ae7).

### Phase3d clean-room promotion audit

The additive Phase3d candidate is a technically credible `C` path for the
`glim_ros2` component, but it is not an r2 repair.  Its preserved Jazzy
evidence proves a host-owned ROS 2 node can link only the pinned GLIM core,
perform bounded PointCloud2/IMU ingress, explicit finalize, and validated
trajectory/map publication; the source guard rejects bridge provenance.  It
does not yet prove promotion of the active benchmark because:

1. `docker/glim_cpu_benchmark.Dockerfile` and
   `scripts/run_glim_benchmark.py` still clone, patch, label, and require the
   `glim_ros2` bridge;
2. Phase3d is an opt-in package under the additive adapter tree, not the
   active image/runner, and does not yet emit the active runner's bag-bound
   per-attempt/resource/result receipt contract;
3. its synthetic evidence covers the ROS boundary only (two LiDAR frames and
   three IMU messages), not HILTI/NTU input/calibration equivalence or the
   required matched repetitions; and
4. the remaining GLIM dependency/source closure and current runner hash drift
   still block a claim-eligible recipe.

The concrete next engineering track is therefore a new, opt-in r3 candidate:
remove the `glim_ros2` clone/patch/label from a copied recipe, install the
Phase3d node and its exact-core/dependency closure, bind the existing frozen
input/calibration/config and resource/attempt receipt fields, and prove the
same trajectory/map/failure semantics before any formal comparison.  Only
after that candidate passes its own source/recipe and legal gates may root
decide whether to add it to a new selection.  The current r2 profile,
selection, receipts, and required rival set remain unchanged and
`NOT_READY`.

## Fair, legal recovery options

The smallest option that preserves the current comparison is an upstream
provenance resolution tied to the exact commits: obtain an upstream-authored
license/notice artifact or written clarification from the relevant rights
holder for GLIM ROS 2, each used rpg_vikit component, and Sophus, and resolve
the FAST-LIVO2 BSD/GPL conflict.  The clarification must be bound to the
exact commit and component scope; a package.xml edit in this worktree is not
evidence.  After review, issue a new closure/selection revision, regenerate
all recipe identity fields, and rerun the full comparison.

Other options are not silent repairs:

1. Rebuild FAST-LIVO2 against the official newer Sophus commit that carries
   `LICENSE.txt`.  This changes a pinned dependency and requires an explicit
   compatibility review, new archive/tree/recipe hashes, a new image, and a
   complete rerun.
2. Remove an unresolved bridge/dependency from a rival.  This changes the
   rival's executable and is not a fair continuation of the current claim;
   it requires a new preregistered system and complete rerun.
3. Replace the rival with another implementation.  The replacement must have
   the same hardware/thread/release, input/calibration/sequence conditions,
   and a complete official source/license closure before comparison.  It is
   not a way to upgrade the current receipts.

Until one of these paths is completed, the only permitted interim mode is
`SOURCE_FETCH_ONLY_NO_REDISTRIBUTION`: source may be fetched locally for
reproducibility, but no source, binary, image publication, or competitive
claim is authorized.  The gate remains `NOT_READY`.
