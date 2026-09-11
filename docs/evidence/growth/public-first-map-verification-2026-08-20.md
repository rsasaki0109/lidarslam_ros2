# Public first-map verification checkpoint — 2026-08-20

> Decision: **MAINTAINER_DOCKER_SOURCE_PASS / EXTERNAL_COHORT_NOT_READY**
>
> Public writes performed by this checkpoint: **none**
>
> Independent validations accepted: **0 / 3**

This checkpoint records a fresh read-only audit of the public `develop` source,
one maintainer-controlled Docker run, and one source-builder run. It is evidence
that the practical first-map workflow works end to end through both maintained
entry paths; it is not an independent cohort report, a release publication, or
authorization to recruit validators.

## Exact public identity

- Repository: `rsasaki0109/lidar_slam_ros2`
- Public branch: `develop`
- Source revision: `dec8ec286953aea42cbcb2f7de70a41042f24e62`
- Product version: `0.9.1`
- ROS distribution exercised: Jazzy
- Public source route: [Getting Started — source quickstart](https://rsasaki0109.github.io/lidar_slam_ros2/getting-started.html#1-install-and-build-from-source)
- Pages manifest: [docs-deployment-v1.json](https://rsasaki0109.github.io/lidar_slam_ros2/docs-deployment-v1.json)
- Deployed page SHA-256: `e675e4a97353b110934ec014e5cfe085a2b364adf5de4447fda994a4bdd2413a`

Before the content-contract hardening below, the bounded
`check_public_docs_deployment.py` audit returned `VERIFIED` because the old
manifest, route fragment, page size, page hash, product version, and source
revision all matched. The audit now also requires the rendered first-map
handoff markers: fixed-demo output-directory handoff, stable-image/report
compatibility, receipt-only attachment, source receipt-helper fallback, and
stable-image receipt-helper invocation, plus the immutable image identity
check. Against the still-deployed old manifest it correctly returns
`NOT_READY` with `manifest-content-contract-mismatch` and
`content-marker-missing`; the audit detail now enumerates all six absent
markers. This keeps a stale page from reopening the cohort gate merely by
retaining an old revision and matching SHA.

## Docker build and first map

The product Dockerfile was built locally from the exact public revision with
`LIDARSLAM_SOURCE_DIRTY=false`. The resulting image passed these build/runtime
checks:

- `lidarslam-map --version` reported `lidarslam_ros2 0.9.1`;
- `lidarslam-map start --help` exposed the maintained map workflow;
- the installed `product-build-info.json` bound `dirty=false` and the exact
  source revision above; and
- the image completed the fixed MID-360 first-map demo from the public Zenodo
  dataset (DOI `10.5281/zenodo.14841855`, CC-BY 4.0).

The demo verified the archive and extracted bag member hashes before playback.
The mapping workflow then completed atomically and produced an Autoware map:

- Autoware verifier: **8 PASS / 0 WARN / 0 FAIL**;
- corrected trajectory: **576 poses**;
- native RKO-LIO trajectory: **2772 poses**;
- generated Lanelet2 structure: **42 lanelets**;
- output contract: `map.pcd`, `pointcloud_map/`, `lanelet2_map.osm`,
  `map_projector_info.yaml`, diagnosis, run manifest, and both Markdown/JSON
  first-map receipts;
- first-map receipt status: **PASS**; all seven receipt checks passed;
- receipt identity: product `0.9.1`, commit
  `dec8ec286953aea42cbcb2f7de70a41042f24e62`, profile
  `rko_lio_graph_mid360_preset`.

This proves the value proposition in a single reproducible path:

`public rosbag2 -> sensor/profile preflight -> SLAM -> Autoware-compatible map -> privacy-bounded receipt`.

The image and map run were maintainer-controlled local evidence. The image was
not pushed to GHCR and no GitHub issue or community message was created.

As a separate read-only compatibility probe, the published
`ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy` image was pulled at digest
`sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef` and
ran the documented fixed demo against an already retained dataset cache. The
map completed with a PASS receipt (7/7 checks), product `0.9.0`, source commit
`0df0c4a86df9f68a894c83f8342e4107c3d23b0f`, receipt SHA-256
`549cc59ca849f757b48d33fb967de3fc8adac7acab401748debccdefcda110d8`, and
manifest SHA-256
`3237532c01545cdc4a6f32c425c08b06be1819cc3cbfc940f427ff154051fe29`.
This was not a clean-host comparable row because the dataset cache was warm.
The same published image exposes no `lidarslam-map report` command, so its
receipt can be reviewed and attached manually but cannot produce the new
copy-ready handoff until a reviewed image containing the v0.9.1 CLI is
published. This confirms the remaining release/image gate is a real public
compatibility boundary rather than an untested assumption.

The documented read-only
`docker image inspect --format='{{index .RepoDigests 0}}'` identity check
returned the same immutable digest
`ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef`.

The published image's installed `create_first_map_validation_receipt.py`
helper was then exercised against that retained output with the output mounted
read-only and container networking disabled. It printed the same seven-check
**PASS** summary, including product `0.9.0`, commit
`0df0c4a86df9f68a894c83f8342e4107c3d23b0f`, and manifest SHA-256
`3237532c01545cdc4a6f32c425c08b06be1819cc3cbfc940f427ff154051fe29`.
This is a compatibility fallback for receipt review; it performs no output
mutation: the pre/post receipt SHA-256 remained
`549cc59ca849f757b48d33fb967de3fc8adac7acab401748debccdefcda110d8`.
It is now documented for third parties until the reviewed `report` image is
published.

The same compatibility boundary was checked from the public `dec8ec2` source
tree by extracting only its tracked receipt helper and running it with
`--json` and no `--write` against the retained source output. It returned
product `0.9.1`, the exact public commit, **PASS**, and all seven checks. This
confirms that the source fallback does not depend on the locally modified
`report <output>` command; the retained receipt SHA-256 remained
`50bd25f7eb1b0199d43ed7819013ffe4748c714ec2f5e936ebd2cb711b5fa636`.

## Tracking-surface audit

The public tracking issue [#422](https://github.com/rsasaki0109/lidar_slam_ros2/issues/422)
is open and still reports **0 / 3**. Its Docker and own-bag links resolve, but
its source link uses the historical fragment `#1-build-the-workspace`; the
deployed page currently exposes `#1-install-and-build-from-source` instead.
The local docs patch keeps the historical fragment as a compatibility anchor
for the next public deployment. Updating the issue body itself remains a
GitHub write and was not performed.

## Source route boundary

From a correctly laid out public checkout (`<workspace>/src/lidar_slam_ros2`),
`bash scripts/source_quickstart.sh --dry-run --json` returned a ready plan for
the six maintained packages and the fixed first-map demo. A live host attempt
initialized the pinned submodules, then stopped at the documented base-tool
installation stage because this host has no `rosdep` command and cannot answer
the `sudo` password prompt. That is a host permission boundary, not a source
build result; no source first-map PASS is claimed from that host attempt.

As a fresh public-state check, a depth-one clone of `develop` at
`dec8ec286953aea42cbcb2f7de70a41042f24e62` was fetched with both pinned
submodules. The public checkout has no package-audit script yet (the expected
pre-publication state), while its own source quickstart dry-run reports
`ready`, the exact six-package inventory, and all side-effect flags false. The
same unmodified checkout completed `source_quickstart.sh --build-only` in the
Jazzy builder: rosdep install/check passed and all six packages finished. Its
installed product build-info records `source=git`, `dirty=false`, and the exact
`dec8ec286953aea42cbcb2f7de70a41042f24e62` revision. Its installed product help
still exposes the older `report <session>` boundary,
not the candidate `report <output>` handoff, so the public source fallback
receipt helper remains a required part of the documentation transition. No
first-map demo was run in this clean-state build-only check.
The exact public-clone `create_first_map_validation_receipt.py` was also run
read-only against the retained stable demo output: it returned the raw
schema-valid receipt `PASS` with all seven checks true, product `0.9.0`, commit
`0df0c4a86df9f68a894c83f8342e4107c3d23b0f`, and manifest SHA-256
`3237532c01545cdc4a6f32c425c08b06be1819cc3cbfc940f427ff154051fe29`; the
receipt file remained unchanged.

The offline plan was freshly rerun with an explicit ROS Jazzy workspace and
`--viewer none`: `status=ready`, six packages, no missing submodules, and the
only missing host tool was `python3-rosdep`. Its side-effect contract remained
all false (`network_accessed`, `apt_executed`, `submodule_checkout`,
`workspace_build_executed`, `demo_executed`, and `filesystem_writes`), while
the planned stages retained dependency preparation, a Release source build,
and the fixed demo handoff. This validates the clean-checkout plan without
silently claiming that the dry-run itself built or mapped anything.

To separate that host boundary from the product path, the same public checkout
was also run in the ROS Jazzy builder environment with the exact documented
workspace layout and `source_quickstart.sh --build-only`. The helper completed
the six-package inventory, `rosdep install`/`rosdep check`, Release symlink
build with tests disabled, and the installed CLI smoke. Its build metadata
reported `source=git`, `dirty=false`, and the exact public revision above. The
same source-built install then ran the documented fixed MID-360 demo through
`source_quickstart.sh --viewer none` with the prepared public dataset cache.
The wrapper completed with:

- first-map receipt status: **PASS**; all seven receipt checks passed;
- receipt identity: product `0.9.1`, commit
  `dec8ec286953aea42cbcb2f7de70a41042f24e62`, profile
  `rko_lio_graph_mid360_preset`;
- Autoware verifier: **8 PASS / 0 WARN / 0 FAIL**;
- corrected trajectory: **575 poses**; native RKO-LIO trajectory: **2772
  poses**; Lanelet2 structure: **42 lanelets**; and
- receipt SHA-256:
  `12d02409db55d47380d7a9a6c8bfcd25ef323fbf9adb6c69fd920901dbb0f03e`.

The source output was also reproduced with the installed product runner after
binding Git's safe-directory metadata; its receipt remained **PASS** with the
same exact commit and profile (receipt SHA-256
`7fc9ce472047be948b3f278cd98996af40bf361ba1d986f96bbd190586db1028`). These
container-level runs establish source-path reproducibility, but remain
maintainer-controlled evidence rather than independent cohort validations.

## First-map handoff boundary

The fixed Docker/source demo output contains the privacy-bounded receipt,
manifest, diagnosis, and verification log but intentionally has no
`session.json` page. The previous report command accepted only a session
bundle, so a third party following the demo path could finish mapping but be
blocked at report preparation. The source implementation now accepts either a
verified session bundle or a fixed-demo output directory, revalidates the same
receipt-bound hashes, and emits the schema-valid `first-map-handoff-v1` JSON
without writing or contacting GitHub.

Against the retained source-builder output, the direct handoff command returned
`READY_FOR_REVIEW` with receipt `PASS`, exact commit
`dec8ec286953aea42cbcb2f7de70a41042f24e62`, profile
`rko_lio_graph_mid360_preset`, and manifest SHA-256
`69ac21859e623b361822dd08a160c0a5da3248993d8a68eb582ad5c2aa1ada47`.
The same installed CLI accepted the maintainer Docker output and returned
`READY_FOR_REVIEW` with receipt `PASS` and manifest SHA-256
`d7d97c5f8292db95b6fe96a229cedee01eb41ed489c63195d66e0beb7c5d9b8b`.
This closes the report-preparation gap for the public fixed-demo route; it
does not create an issue, upload a receipt, or count as an independent report.
The handoff fix and compatibility-anchor edits are currently local changes;
the deployed Pages bytes and existing public Docker images will include them
only after an authorized reviewed publication.

The final runtime stage was then built locally from the current worktree with
the same exact public source revision (local image ID
`sha256:0205654aa9c005f90d4df0ec44473487654706f181c979f921272013c479603a`).
Its installed CLI exposes `report <output>` and returned
`READY_FOR_REVIEW / PASS` for both the current source-builder output and the
retained public `v0.9.0-jazzy` output. The latter handoff preserved product
`0.9.0`, commit `0df0c4a86df9f68a894c83f8342e4107c3d23b0f`, and manifest SHA
`3237532c01545cdc4a6f32c425c08b06be1819cc3cbfc940f427ff154051fe29`.
This runtime check is local release-candidate evidence only; it does not
change the public image or release state.

The same Dockerfile was also built locally for ROS 2 Humble as
`lidarslam-local-humble-runtime:handoff`. The image completed the six-package
build, installed the 138-package runtime closure, and passed the image-stage
`lidarslam-map --version`/help guard. Its local immutable image identity is
`sha256:86ab168ea73c7736a7d75663dc4dfc740ecac0df1a225a019b1a6ea77ffdca5a`
and its embedded build-info records revision
`b48ef062c09421436bb4a4f721785a9bc216cbf4`, `dirty: true`, and
`source: override`; this is intentionally a local candidate, not a public
release identity. With the retained public v0.9.0 output mounted read-only,
the Humble image returned `READY_FOR_REVIEW`, receipt `PASS`, product `0.9.0`,
commit `0df0c4a86df9f68a894c83f8342e4107c3d23b0f`, and manifest SHA-256
`3237532c01545cdc4a6f32c425c08b06be1819cc3cbfc940f427ff154051fe29`.
The receipt SHA-256 stayed
`549cc59ca849f757b48d33fb967de3fc8adac7acab401748debccdefcda110d8` before
and after the run. This adds local Humble/Jazzy runtime coverage while
preserving the read-only, no-publication boundary.

To keep this boundary from regressing, both Dockerfile stages and the Docker,
candidate-image, and release smoke checks now fail closed unless the installed
image help contains `report <output>`. This is a publication guard, not a
claim that the already-published v0.9.0 image has been modified.

The Pages deployment contract now has the same fail-closed property: the
rendered `getting-started.html` must advertise the fixed-demo output-directory
handoff, the published-v0.9.0/report compatibility boundary, and the
receipt-only attachment rule. The local MkDocs build plus manifest generator
passed these markers; the public site must be redeployed before the audit can
return `VERIFIED` again.

The repository README now exposes the same receipt-only `report` next step and
links the independent-validation guide, so a source checkout's front door does
not stop at the older `support --first-map` spelling.

The installed-product validation script also invokes `report --json` directly
against the receipt-bearing map directory after its normal session-bundle
check, validates `first-map-handoff-v1`, and asserts that no output tree is
mutated. The current combined handoff/docs/runtime/installed-CLI group passed
(`122 passed`), including the stable-image fallback, README entrypoint, and
verification-package assertions. The release checklist now runs the same
package audit before bundle rehearsal, so a release handoff cannot omit this
third-party verification surface.

The rendered local Pages artifact was also audited with the same read-only
deployment verifier used for the public URL. A fresh temporary artifact bound
the current source revision
`b48ef062c09421436bb4a4f721785a9bc216cbf4`, product `0.9.1`, and page
SHA-256 `24955689431ea23d2baacecc65082f7cca3320753e4c83794bdff2200f95612a`;
all nine deployment checks returned **VERIFIED**, including all six content
markers. The temporary artifact and manifest were discarded; this proves the
candidate Pages output, not the still-old public deployment.

## First-map verification package audit

The checkout now exposes one package-level, read-only audit for the complete
third-party handoff surface:

```bash
python3 scripts/check_first_map_verification_package.py --json
```

The local result was `READY`, with all eight checks true:

- required regular files and schemas;
- Docker/source documentation, including the stable-image helper and digest
  fallback;
- receipt-only `report` evidence contract;
- Dockerfile and image-workflow `report <output>` guards;
- all six rendered first-map content markers;
- README, English/Japanese Getting Started, and external-validation audit
  instructions;
- the main, Pages, and release CI gates; and
- release-bundle inclusion of the audit and schema.

The report is schema-valid under
`first-map-verification-package-v1.schema.json`, declares
`network_requested: false` and `writes_performed: false`, and returns
`NOT_READY` for an incomplete checkout. MkDocs strict plus the deployment
manifest generator passed locally; the generated manifest advertised the same
six marker IDs. The package audit is therefore a local completion gate, not a
claim that the currently deployed Pages bytes or GHCR tags have changed.

An additional release-boundary audit exposed and closed one packaging gap: the
first candidate bundle carried the audit script but not every CLI/source helper
and Docker/docs workflow that the package itself requires. The release
inventory now includes that complete surface, including the source dependency
helper required by the quickstart dry-run. Two byte-identical local builds of
the v0.9.1 candidate bundle produced the same 300-file archive. The
archive digest is intentionally emitted as detached build output rather than
written into this bundled evidence file, which prevents a self-referential
hash claim.
After extraction, running the bundled audit against the extracted root returned
`READY` with all eight checks true and both `network_requested` and
`writes_performed` false. The audit was then invoked from the extracted bundle's
own `scripts/check_first_map_verification_package.py`, and the bundled
`source_quickstart.sh --help` exposed the documented workspace, distro,
viewer, build-only, dry-run, and JSON options. This proves the verification
package is self-contained inside the candidate bundle; it remains local
pre-publication evidence.

## Remaining public gates

The v0.9.1 release/image identity is still absent from public distribution:

```text
check_published_release.py --version 0.9.1 --json
  status: NOT_PUBLISHED
  v0.9.1-humble: ABSENT
  v0.9.1-jazzy:  ABSENT
```

The latest public GitHub release remains v0.9.0, so a v0.9.0 Docker image and
the v0.9.1 source/docs route cannot form a same-version comparable matrix row.
Human active-time/command-count observations and clean-host source/Docker rows
are also still missing. Because the deployed v0.9.0 image has no `report`
command and the deployed Getting Started page lacks the receipt-only handoff
markers, `copy_ready_handoff_public` is now explicitly `false` in the tracked
cohort contract. The cohort therefore remains `WAITING_FOR_PUBLIC_GATES`; no
recruitment or external write is authorized.

## Next transition

1. Publish a reviewed v0.9.1 release and immutable Humble/Jazzy GHCR images
   (requires explicit maintainer authority).
2. Run fresh paired clean-host Docker/source rows at that one identity with
   human active-time and command-count observations.
3. Re-run the public docs provenance and cohort checks, then request the
   separate community/GitHub write decision.
4. Start at most two independent attempts; retain both PASS and FAIL receipts.

## Publication handoff (not executed)

The following transition is prepared but intentionally not performed by this
checkpoint because it requires separate maintainer authority for a public
merge, Pages deployment, release tag, and GHCR publication:

1. Review and merge the local docs/CLI/handoff changes into the trusted
   `develop` path; wait for the Pages artifact to finish.
2. Run the exact public-docs audit for the newly merged `develop` revision
   (the current pre-publication baseline is
   `dec8ec286953aea42cbcb2f7de70a41042f24e62`) and product `0.9.1`:

   ```bash
   EXPECTED_PUBLIC_REVISION="$(git rev-parse HEAD)"
   python3 scripts/check_public_docs_deployment.py \
     --expected-revision "$EXPECTED_PUBLIC_REVISION" \
     --expected-product-version 0.9.1 \
     --route source-quickstart --json
   ```

   Continue only when it returns `VERIFIED` and all six content markers are
   present in the deployed page.
3. Publish the reviewed `v0.9.1` release and immutable Humble/Jazzy images,
   then run:

   ```bash
   python3 scripts/check_published_release.py --version 0.9.1 --json
   ```

   Continue only when the tag, release, and both GHCR image identities are
   present and mutually consistent.
4. Run fresh clean-host Docker/source rows at that one identity, record human
   active time and command count, and rerun
   `python3 scripts/first_map_validator_cohort.py --json`. Recruitment remains
   unauthorized until that report returns a launch-ready state.
