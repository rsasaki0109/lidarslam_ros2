# Onboarding matrix evidence index — 2026-08-12

> Decision: **TRUTHFUL_4_OF_4 / ACTIVATION_GATE_CLOSED**
>
> Public source route inspected:
> `549ef03017c776f23fc968881b346aa685356274` (`0.9.1`)
>
> Preflight remote mutations performed: **none**

## Outcome

The shortest maintainer audit now reports the evidence already checked into
the repository instead of treating an omitted positional argument list as an
empty project history:

```bash
python3 scripts/check_onboarding_trial_matrix.py --json
```

The command loads
`docs/contracts/g0-onboarding-matrix-evidence-v1.json`, whose four explicit
rows point to the reviewed Humble and Jazzy Docker records plus the two
reviewed source records. It does not glob a directory, choose a latest file,
or infer success from a missing record. Explicit paths remain available for
auditing provisional records before maintainers update the index.

## Current truthful matrix

| Row | Present | Outcome | Measurement | Comparable |
| --- | --- | --- | --- | --- |
| Docker Humble | yes | `PASS` | `INCOMPLETE` | no |
| Docker Jazzy | yes | `PASS` | `INCOMPLETE` | no |
| Source Humble | yes | `PASS` | `INCOMPLETE` | no |
| Source Jazzy | yes | `PASS` | `INCOMPLETE` | no |

The aggregate is four present rows, four product PASS outcomes, zero comparable
rows, and a closed activation gate. The Docker records still lack human active
time, human command count, and isolated peak disk. The source records have
complete route, build, map, verifier, and receipt outcomes, but deliberately
record both `active_operator_time_sec=null` and `command_count=null` because no
human observer measurement was retained. The Docker rows are the immutable
`0.9.0` release records while the source rows are the reviewed
`0.9.1` candidate records, so the checker also reports
`product_version_aligned=false`. It exposes all four rows for review but keeps
the matrix `BLOCKED`; no cross-version evidence is allowed to pass a route
comparison gate.

## Version-alignment preflight — 2026-08-13

The read-only release audit for the reviewed source candidate is still
`NOT_PUBLISHED`:

```json
{
  "expected_tag": "v0.9.1",
  "expected_version": "0.9.1",
  "release_present": false,
  "tag_present": false,
  "assets": [],
  "status": "NOT_PUBLISHED"
}
```

The exact image probes also return `manifest unknown` for both
`ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.1-humble` and
`ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.1-jazzy`. The apparent fallback—
rerunning source against the frozen `v0.9.0` Docker identity—is not a valid
route: its public tag commit
`0df0c4a86df9f68a894c83f8342e4107c3d23b0f` returns
`source-route-contract-missing` because it predates
`scripts/source_quickstart.sh`. No local build or moving image tag is promoted
to matrix evidence.

The release audit now exposes the same image gate without a second manual
manifest command:

```bash
python3 scripts/check_published_release.py --version 0.9.1 --json
```

Its `images` array reports both exact tags as `ABSENT` while the overall result
remains `NOT_PUBLISHED`; registry/API failures remain distinguishable as
`ERROR`/`BLOCKED`. This is a read-only maintainer aid and does not publish or
retag an image.

## Public source diagnosis

The network-read-only source preflight against exact public Draft PR commit
`549ef030` returns `READY` for product version `0.9.1`: it resolves all six
maintained source packages and all four route-contract files with no finding
codes or writes. Separate clean disposable Humble and Jazzy runtime-image
trials then completed the documented source route and produced map, verifier,
and receipt PASS results. The checked-in records are valid product outcomes,
but remain non-comparable because active human time was not observed. The
runtime images already contained ROS and the build dependencies, so these rows
also do not claim a cold package-install baseline. The matrix checker reports
both this measurement blocker and the Docker/source version-alignment blocker
as explicit next actions.

The first exact-tip check also exposed a GitHub API boundary: the commits API
returns HTTP 422, rather than 404, for a syntactically valid but unpublished
40-character commit. The observer now treats 404 and 422 from that commit
lookup as `source-candidate-not-published`, while a 422 from a content lookup
remains an observer error. This keeps an unpublished candidate machine-
decidable without hiding authentication, validation, or malformed-content
failures.

## Fail-closed checks

The evidence-index schema fixes the matrix ID, four row IDs, repository, safe
repository-relative JSON paths, and no-remote-mutation authority. The checker
also requires the fixed row order, regular non-symlink files, and agreement
between an index row and the record's route, distribution, and environment.
Missing, escaped, shuffled, or mislabeled records fail validation.

Validation results:

- onboarding matrix regressions: **12 passed**;
- source public-preflight regressions: **20 passed**, including real 422
  unpublished-tip classification;
- no-argument report: **4/4 present, 4 PASS, 0 comparable, `BLOCKED`**;
- runner and regression-test full flake8: **PASS**;
- complete maintained Python gate: graph **1,433 passed / 13 skipped**,
  lidar_slam **751 passed**, **2,184 total**;
- strict MkDocs build: **PASS** with pre-existing notices;
- source public preflight: **READY** at exact `549ef030`, no writes;
- issue, PR, branch, release, image, and other mutation by the preflight:
  **none**.

## Next gate

Align all four rows to one product version first. At present that requires a
reviewed `v0.9.1` Release and its immutable Humble/Jazzy images; the `v0.9.0`
source fallback is closed by the route-contract preflight above. After that
identity exists, run dedicated disposable Humble and Jazzy hosts with a human
observer and replace rows only after active time and command count are
measured. The Docker rows likewise require a dedicated filesystem and human
active-time observation; another shared-host automation run would not close
that gap.
Until at least one Docker PASS and one source PASS are comparable, the G0
activation gate stays closed and no independent-user cohort is recruited.
