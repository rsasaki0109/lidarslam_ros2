# Independent First-map Validation

`lidarslam_ros2` does not call the v1.0 onboarding path ready until at least
three independent users generate and verify a first map from the public
documentation. Maintainer-operated demos, CI jobs, and users guided live
through each step do not count.

The current accepted count is recorded in the
[machine-readable validation ledger](evidence/external-first-map-validations.json).
An empty ledger is an honest `0 / 3`, not missing evidence.

Before running either public path from a source checkout, verify that the
checkout contains one internally consistent handoff package:

```bash
python3 scripts/check_first_map_verification_package.py --json
```

This package audit is local-only and read-only. Continue only on `READY`; a
`NOT_READY` result means the documented Docker/source fallback, receipt
contract, runtime guard, public-page contract, or release/CI wiring is not
complete enough for an external validation attempt.

## Participate

1. Choose one official path without private maintainer instructions:

   - [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace)
   - [source quickstart](getting-started.md#1-install-and-build-from-source)
   - [own-bag golden path](golden-path-cli.md)

2. A current `lidarslam-map run` writes
   `first_map_validation_receipt.json` and
   `first_map_validation_receipt.md` after finalizing the manifest,
   diagnosis, and Autoware verification log. From the successful session page,
   copy **Prepare a first-map report**, or run:

   ```bash
   lidarslam-map report /path/to/session_bundle
   ```

   The fixed Docker and source first-map demos also retain the same receipt
   contract but do not create a `session.json` page. For those paths, pass the
   demo output directory itself (for example,
   `lidarslam-map report /lidarslam_ws/output/mid360_demo`); the command
   validates the receipt-bound manifest, diagnosis, and verification log in
   place and produces the same read-only handoff.

   The published `v0.9.0-{humble,jazzy}` images predate the `report` command.
   If you use one of those stable images, review
   `output/mid360_demo/first_map_validation_receipt.json` and complete the
   issue form manually with its verification summary and the exact image
   digest; attach only that reviewed receipt. To reprint the summary without
   writing or networking, the stable image also provides this fallback (use
   the `jazzy` tag on Ubuntu 24.04):

   ```bash
   docker run --rm --network none \
     -v "$PWD/lidarslam_output:/output:ro" \
     ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble \
     bash -lc 'helper="$(ros2 pkg prefix lidarslam)/share/lidarslam/product/scripts/create_first_map_validation_receipt.py"; python3 "$helper" /output/mid360_demo'
   ```

   Review the printed PASS summary and attach only the already-generated JSON receipt.
   A reviewed `v0.9.1` image (or a later image carrying this CLI contract)
   provides the copy-ready `lidarslam-map report` command.

   Record the immutable digest of the pulled stable image:

   ```bash
   docker image inspect \
     --format='{{index .RepoDigests 0}}' \
     ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble
   ```

   If no `ghcr.io/...@sha256:...` value is returned, stop rather than
   reporting a mutable tag as the runtime identity.

   For a public source checkout that predates the output-directory handoff,
   print the same receipt without writing it:

   ```bash
   python3 scripts/create_first_map_validation_receipt.py \
     /path/to/ros2_ws/output/mid360_demo --json
   ```

   Review the `PASS` JSON and attach only that receipt; omit `--write` and do
   not attach maps, manifests, diagnoses, or logs.

   The command performs no write or network request. It revalidates the PASS
   receipt against the retained manifest, diagnosis, and verification log,
   then prints the copy-ready PASS result, source commit or product-version
   fallback, exact verification summary, JSON attachment path, issue form,
   and a field-by-field template for the four values that still need your own
   input. If you ran an immutable
   image digest, replace the suggested release value with that digest.
   For automation or a wrapper UI, add `--json`; the same read-only handoff is
   returned as the schema-valid
   [`first-map-handoff-v1`](schemas/first-map-handoff-v1.schema.json) object,
   including safe OS/architecture/ROS hints and the four operator-supplied
   fields. This handoff JSON is local-only because it includes a local receipt
   path; attach only the reviewed receipt named inside it.

   For an existing output produced before automatic receipts were added, you
   may regenerate the receipt manually:

   ```bash
   python3 scripts/create_first_map_validation_receipt.py \
     /path/to/output \
     --write
   ```

   Older sessions may not reference that new receipt from `session.json`, so
   the fail-closed `report` handoff can still reject them. Review
   the generated JSON and submit it manually through the same issue form; do
   not edit old session evidence merely to make the handoff pass.

3. Record the release tag, commit, or immutable image digest and the redacted
   command shape you ran. Keep the executable, options, and non-private values,
   but replace credentials, private paths, host or user names, and precise
   locations with the literal `REDACTED` placeholder.
4. Follow the printed
   [Independent First-map Validation issue form](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=first-map-validation.yml),
   paste its copy-ready `Verification summary`. For a PASS report, review the
   named `first_map_validation_receipt.json`, then drag only that file into
   the **Privacy-bounded JSON receipt** field. For a FAIL report with no
   generated receipt, leave that field empty and paste the first actionable
   terminal finding instead. GitHub stores issue attachments publicly. The
   generated JSON contains evidence hashes but no map geometry, private paths,
   or exact command; do not attach any other run artifact.

   The required privacy attestation has two honest branches: review the one
   attached JSON receipt, or, for a FAIL where no receipt was produced, attach
   no file. A missing receipt never permits a PASS report, and a FAIL report
   never requires inventing or attaching another run artifact.

Both passing and failing reports are useful. A failed attempt is an onboarding
finding, not an accepted validation, and must be resolved or explicitly
documented before v1.0.

!!! warning "Do not publish map geometry"

    Replace credentials, private paths, host or user names, and precise
    locations with `REDACTED`; remove rosbag payloads, point-cloud tiles,
    trajectories, and screenshots that reveal private places. The issue form
    asks for a redacted command shape, environment, status, verifier
    summary, and the privacy-bounded JSON receipt. You must still review the
    receipt before attaching it and redact private paths from the separately
    pasted command.

## What the receipt proves

The
[`first-map-validation-receipt-v1` schema](schemas/first-map-validation-receipt-v1.schema.json)
contains only the product version/revision, profile, run ID, status values,
file hashes, and seven boolean checks. A passing receipt proves:

- the final manifest says `succeeded`, lifecycle `complete`, and runner exit
  code `0`;
- the diagnosis says `success`;
- the Autoware verification result is `PASS`;
- the diagnosis and verification-log hashes match the identities frozen into
  the manifest.

The receipt is derived after the final manifest write, so receipt files are
intentionally excluded from the manifest's artifact-checksum list. This avoids
a circular manifest → receipt → manifest hash while keeping the three
authoritative inputs cryptographically bound.

## Acceptance contract

A report counts only when all of these are true:

- the reporter self-attests that they are not a maintainer and did not receive
  live step-by-step help;
- the run started from one of the three public documentation paths;
- the report identifies the exact release, revision, or image digest;
- `run_manifest.json` says `succeeded`;
- the diagnosis status is `success`;
- Autoware map verification is `PASS`;
- the generated receipt says `PASS` and its manifest SHA-256 matches the
  submitted verification summary;
- a maintainer reviews the public issue and records its review link;
- every finding is either resolved or explicitly documented with a public
  resolution link;
- the reporter, issue URL, validation ID, and manifest SHA-256 are unique
  across the ledger.

The tracked
[`external-first-map-validations-v1` schema](schemas/external-first-map-validations-v1.schema.json)
requires successful, accepted evidence. Failed reports remain in GitHub
issues and are linked from their eventual resolution; they are never rewritten
as passing evidence.

## Reviewer workflow

Do not hand-edit the tracked ledger first. After resolving any onboarding
findings:

1. Download `first_map_validation_receipt.json` from the public issue. Confirm
   it is the generated privacy-bounded receipt—not a manifest, map, log, or
   other run artifact—and that the issue contains its matching verification
   summary.
2. Create `/tmp/validation-entry.json` with one `validation` object matching
   the
   [`external-first-map-validations-v1` schema](schemas/external-first-map-validations-v1.schema.json).
   Record the public issue-comment or pull-request review URL in
   `acceptance.review_url`.
3. Prepare a new ledger file:

   ```bash
   python3 scripts/prepare_external_first_map_acceptance.py \
     --entry /tmp/validation-entry.json \
     --receipt /path/to/first_map_validation_receipt.json \
     --output-ledger /tmp/external-first-map-validations.proposed.json
   ```

4. Review the proposed file and its diff, then copy that reviewed proposal to
   `docs/evidence/external-first-map-validations.json` in a normal pull
   request. Do not commit a second copy of the reporter's receipt; the public
   issue attachment remains its review source.

The intake command is fail-closed. It requires a schema-valid PASS receipt,
all seven receipt checks, exact verification-field and manifest-hash binding,
a release reference matching the receipt version/revision or an immutable
image digest, chronological submission and review timestamps, a public
repository review URL on the submitted issue (or its review PR), a known
maintainer reviewer, and an independent reporter. The current maintainer is
explicitly excluded as an independent reporter.
It appends the entry in memory and revalidates the complete ledger, so
duplicate reporters, issues, validation IDs, and manifest hashes are rejected.

The command never modifies the input ledger, refuses to write to its path, and
refuses to overwrite any existing proposal. Without `--output-ledger`, it
performs a dry run and prints a schema-valid
[`external-first-map-acceptance-v1` report](schemas/external-first-map-acceptance-v1.schema.json).

After reviewing and placing the proposal in the tracked ledger, validate it:

```bash
python3 scripts/check_external_first_map_readiness.py --json
```

The command exits `0` when the ledger is structurally valid, even before three
reports exist, and reports `NOT_READY` with the exact remaining count. The
v1.0 release gate is stricter:

```bash
python3 scripts/check_external_first_map_readiness.py --require-complete
```

Exit codes are:

| Code | Meaning |
| ---: | --- |
| `0` | Ledger is valid; with `--require-complete`, the 3-user gate is ready |
| `1` | Ledger is valid but fewer than three accepted reports exist |
| `2` | Ledger or schema is invalid, including duplicate evidence |

The intake command uses `0` for `READY_TO_PROPOSE` and `2` for invalid,
unbound, duplicate, unreviewed, or unsafe-to-write evidence.

The validator reports counts by documentation path for coverage visibility,
but the v1.0 contract does not invent a per-path quota: the published
requirement is three distinct independent users completing a first map from
public documentation.
