# Published onboarding identity gate — 2026-08-16

> Decision: **LOCAL_CONTRACT_PASS / PUBLIC_0.9.1_IDENTITY_PENDING**
>
> Public writes performed by this audit: **none**

Implementation tip: `289f7675a242b00f342528483cde3e5f602a11fc`.

## Why this gate exists

The release-mode onboarding packet previously accepted a manually entered
product version, source commit, Humble digest, and Jazzy digest. Although each
field was validated syntactically, the four values did not have to originate
from one authenticated live observation. A typo or stale copy could therefore
create a plausible clean-host plan whose Docker and source rows were not the
same public product.

Release-mode packets now accept one bounded
`published-release-v1` report only. The report must validate against its Draft
7 schema, have status `PUBLISHED`, contain the complete release check set, and
bind the finalized GitHub release tag and commit to exactly one Humble and one
Jazzy image digest. The packet records the report SHA-256 and derives all four
identities from those bytes. Manual release identity options are rejected.

Before an observer runs a Docker row, the generated packet invokes
`check_published_onboarding_identity.py`. That read-only check repeats the live
release audit and requires the current tag commit and both image digests to
equal the packet's expected identity. Its result is `READY`, `NOT_READY`, or
`BLOCKED`; it performs no release, registry, trial, or GitHub write.

## Live read-only audit

The 2026-08-16 v0.9.0 audit produced a schema-valid packet directly through
the report-to-packet pipeline. The exact report bytes had SHA-256
`45ce5f5e7beda3637e878a894864ef5cacfff236e50172572554adb340ff8a35` and
bound:

- release tag `v0.9.0` and commit
  `0df0c4a86df9f68a894c83f8342e4107c3d23b0f`;
- Humble digest
  `sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53`;
- Jazzy digest
  `sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef`;
  and
- release URL
  `https://github.com/rsasaki0109/lidar_slam_ros2/releases/tag/v0.9.0`.

The exact live identity check returned `READY`. The independent source-route
preflight correctly returned `NOT_READY` with
`source-route-contract-missing`, because v0.9.0 predates
`scripts/source_quickstart.sh`. Therefore v0.9.0 still cannot supply a
same-version Docker/source comparison row.

The v0.9.1 audit returned `NOT_PUBLISHED`; feeding that report to the packet
generator failed closed before a packet was emitted. No candidate-image
evidence directory was present or synthesized. G0 remains 4/4 product rows
present, 0/4 comparable, with human measurements still missing.

## Operator route

```bash
python3 scripts/check_published_release.py \
  --version 0.9.1 --json --require-published \
  | python3 scripts/prepare_onboarding_matrix_packet.py \
      --published-release-report - --render
```

The pipeline is deliberately one-way: a non-published or incomplete audit
cannot be repaired by typing substitute identities. Candidate mode remains a
separate authenticated four-file evidence route.

## Verification boundary

Focused regressions cover exact live identity matches, commit and digest drift,
missing and blocked releases, duplicate or mislabeled images, report schema and
check-set tampering, non-published reports, symlinked inputs, manual identity
rejection, and release/candidate mode separation. The release bundle and docs
entrypoint tests also require the new checker. The exact local results are 13
packet tests, 5 published-identity tests, 5 G0 dashboard tests, 14 publication
plan tests, and 25 docs-entrypoint tests: 62 / 62 passed. All four affected
ament CTest registrations pass after a Jazzy reconfigure. The complete
maintained product gate is 1,442 graph tests passed / 13 skipped and 975
lidarslam tests passed, for 2,417 passes total. A clean v0.9.1 candidate release
bundle at the implementation tip contains 249 manifest-bound files, including
the new checker, schema, and this evidence record.

This work establishes an exact preflight contract only. It does not publish
v0.9.1, create candidate images, run a clean-host trial, collect human
measurements, merge PR #427, deploy Pages, or authorize E2, E3, or E4.
