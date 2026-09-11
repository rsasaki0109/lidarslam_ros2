# Public documentation deployment provenance — 2026-08-16

> Decision: **LOCAL_CONTRACT_PASS / PUBLIC_DEPLOYMENT_PENDING**
>
> Public writes performed by this audit: **none**
>
> Fail-closed artifact implementation tip:
> `5b8c8c477cceb4955184a64afa874712b9dea5aa`

## Why this gate exists

The independent first-map cohort previously accepted a canonical GitHub Pages
URL by shape alone. That was insufficient: a public source commit can resolve
while the deployed Getting Started page still comes from an older `develop`
revision. A first-time operator could therefore receive instructions that do
not match the runtime being measured.

The live read-only audit on 2026-08-16 confirmed this is a real boundary, not a
hypothetical one. The captured public Draft PR #427 head was
`ac22a3ff1e49c1dae3fcde47f52ae8bf8ccdb1eb`, while `develop` remained at
`86fa9b610c07ccf4d2b0f10939e17c129d34b40a`; the deployed site does not yet
publish `docs-deployment-v1.json`. The exact audit returned `BLOCKED` with
`manifest-unavailable` and HTTP 404. No cohort route is promoted from that
evidence.

## Product and workflow contract

The `docs-site` workflow now builds the site first and then generates a
deterministic `docs-deployment-v1.json` inside the Pages artifact. The manifest
binds:

- the exact 40-character source revision checked out by Actions;
- the product version;
- the exact byte count and SHA-256 of `getting-started.html`;
- the Docker first-map and source-quickstart fragment IDs; and
- the rendered first-map handoff markers: fixed-demo output-directory
  handoff, stable-image/report compatibility boundary, and receipt-only
  attachment rule, source and stable-image receipt-helper fallbacks, and the
  immutable image identity check; and
- the trusted `develop`-only Pages deployment workflow identity.

Both workflow jobs also require the actual Actions ref to equal
`refs/heads/develop`. A manually dispatched feature branch cannot build or
deploy an artifact that falsely claims the trusted deployment identity.

Generation fails if the rendered page is empty, oversized, non-UTF-8,
symlinked, lacks either canonical fragment, or lacks any first-map handoff
marker. An existing manifest is never
overwritten inside the artifact. Before exclusive creation, the generator now
validates the complete payload against the checked-in Draft 7 schema. The
workflow installs `jsonschema` explicitly, reruns when the shared schema
validator changes, and retains a final JSON parse check. Schema/output drift
therefore stops before the Pages artifact is uploaded rather than being found
only after deployment.

`check_public_docs_deployment.py` performs only bounded HTTPS GETs. It validates
the public manifest against its Draft 7 schema, requires the fixed route
pairing, fetches the exact page, verifies byte count and SHA-256, checks the
selected fragment and rendered handoff markers, and compares source revision
and product version with operator-supplied expectations. It returns
`VERIFIED`, `NOT_READY`, or `BLOCKED`; no result authorizes a merge, Pages
deployment, trial, recruitment, image publication, or release. The currently
deployed old manifest still passes the identity checks but is now correctly
`NOT_READY` for the missing handoff contract.

## Cohort binding

The first-map cohort contract now requires one
`canonical_documentation_provenance` object in addition to the canonical path,
URL, and runtime identity. Its deployment revision must equal the cohort's
public source revision. Every active attempt also records the page SHA-256 and
must match the selected provenance. URL presence alone can no longer open the
cohort.

## Verification

```bash
python3 -m pytest -q \
  lidarslam/test/test_docs_deployment_provenance.py \
  lidarslam/test/test_first_map_validator_cohort.py \
  lidarslam/test/test_check_g0_readiness.py

python3 scripts/check_public_docs_deployment.py \
  --expected-revision ac22a3ff1e49c1dae3fcde47f52ae8bf8ccdb1eb \
  --expected-product-version 0.9.1 \
  --route source-quickstart \
  --json
```

The twelve focused deployment regressions cover exact identity, pre-write
schema validation, missing route fragments and handoff markers, symlink
rejection, exclusive manifest creation, revision/version drift, page
tampering, route-pair drift, legacy-manifest rejection, and unavailable public
evidence. The live command currently exits 1 with `NOT_READY`, as required
until a reviewed `develop` deployment exposes the matching provenance and
handoff content.

## Honest boundary

This increment makes a future public route auditable; it does not make the
current route comparable. Docker/source rows remain mixed-version and lack
complete human measurements. The `candidate-images` environment, v0.9.1
images, accepted independent maps, and paired GLIM usability records remain
absent. Pages deployment occurs only after a separately reviewed source merge.
