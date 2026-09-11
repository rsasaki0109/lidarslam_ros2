# Competitive execution-selection r3 external capture (2026-08)

This is an additive observation utility. It does not edit the active profile,
the historical receipt, the r2 selection, or the r3 candidate. Its output is
always `NOT_REVIEWED_EXTERNAL`, `benchmark_eligible: false`, and
`claim_eligible: false`, including when every synthetic probe passes.

## Boundaries

`capture_competitive_execution_selection_r3_external.py` reopens and validates
the checked-in r3 candidate before any output directory is created. It binds
the candidate path, file SHA-256, canonical candidate identity, candidate id,
and all three current profile image digests (`ours`, `glim`, and `fast_livo2`).
The candidate remains `NOT_READY`; this utility cannot reseal or promote it.

Image observation uses only the fixed command shape:

```text
docker image inspect --format '{{json .}}' sha256:<candidate-digest>
```

No pull, build, network, or shell command is available to this path. The
sanitized observation records only image identity and non-secret platform
metadata; environment, labels, mounts, and arbitrary inspect fields are not
copied into evidence.

Toolchain observation is disabled unless `--probe-toolchain` is explicitly
provided. For an observed local image, each fixed compiler/linker/ROS/dependency
probe runs with:

```text
docker run --rm --pull=never --network none --read-only \
  --entrypoint <fixed-probe> sha256:<candidate-digest> <fixed-args>
```

The probe runner uses `shell=False`, bounded timeout/output, and no caller
argv. A missing image, malformed inspect, digest mismatch, timeout, or failed
probe remains `PENDING`; toolchain probes are not attempted when image
identity was not observed.

The dependency projection is inherited explicitly from the existing capture
contract and is part of the sealed toolchain fingerprint. `ours` and
`fast_livo2` require every listed probe; `glim` marks only `pcl` as
`not_applicable` because that field is not part of its existing capture
contract. The artifact contains `not_applicable_fields` and a typed
`not_applicable` probe (`command: null`, empty-output SHA-256, and a fixed
reason). No other system may use that projection, and removing a required
probe is invalid. The schema permits only the named N/A token; the runtime
validator binds it to the system-specific mapping above.

## Immutable output

Each fresh output directory contains six sealed files:

* `image_inspect.json` and its `.sha256` sidecar;
* `toolchain_capture.json` and its `.sha256` sidecar;
* `capture_manifest.json` and its `.sha256` sidecar.

Files are created exclusively, fsynced, mode `0444`, regular, and single-link.
Parent symlinks, output collisions, hard links, extra files, stale sidecars,
partial pair writes, path traversal, and mutation between reads fail closed.
Rollback removes only inodes owned by the failed write. Validation reopens
both the payload and sidecar, revalidates the candidate at the end of capture,
and requires exact candidate/digest/argv/safety bindings. The manifest is not
an external review receipt: a custodian must
independently review and materialize any future `REVIEWED_EXTERNAL` artifact.

## Status and verification

The capture set has strict schema identity
`competitive_execution_selection_r3_external_capture_manifest_v1`; the same
schema validates image and toolchain documents. A complete synthetic fixture is
reported as `UNREVIEWED_COMPLETE_OBSERVATION`, never `READY`. Without explicit
toolchain opt-in or with any failed image/probe, it is `PARTIAL_OBSERVATION`.

The implementation and adversarial tests cover digest mismatch, missing local
images, no-toolchain opt-in, exact network/read-only argv, command failure,
collision, symlink/hardlink/extra files, self-rehashed candidate binding,
partial sealing, and mutation during reopen. Only injected fake command
runners are used in tests; no real toolchain container probe, benchmark, bag,
GT, scorer, pull, build, or network operation is part of this evidence.

## External custodian review response

`validate_competitive_execution_selection_r3_external_review.py` is the
separate, non-promoting review gate for a custodian response. The checked-in
trust policy is intentionally `NOT_READY` with `authorized_keys: []`; it can
never authorize a response in place. A test or future handoff may provide an
external `READY` policy, but the validator reopens the current candidate,
profile/r2 selection bindings, capture producer and schema, and all six
capture files (the three JSON files and their three `.sha256` sidecars).

The response is domain-separated Ed25519 evidence and binds all three systems,
their exact image digest/platform/RepoDigests, complete toolchain fingerprints,
the fixed network-none/read-only probe scope, a bounded validity window, and a
one-shot nonce ledger. `ACCEPTED` remains offline review evidence only:
`benchmark_eligible`, `claim_eligible`, and `active_profile_switch` are always
false, and a separately reviewed candidate/profile reseal is required before
any promotion. Partial/pending capture, expired or malformed responses,
wrong-key signatures, changed six-file evidence, replayed nonces, and checked-in
policy use fail closed. Current repository evidence is synthetic-test-only;
real custodian keys, signatures, Docker, network, benchmark, bag, GT, and
scorer execution are `NOT_RUN`.
