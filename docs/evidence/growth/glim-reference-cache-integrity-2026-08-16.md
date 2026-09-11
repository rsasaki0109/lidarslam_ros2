# GLIM reference-cache integrity evidence — 2026-08-16

## Outcome

Initial content-bound implementation tip:
`3de7f84bb51acd2bd1c2b40724529be9c281d2fe`. The current publication-plan tip
also includes the concurrent-publication hardening described below.

The existing `compare_with_glim.sh` workflow no longer accepts a cached GLIM
trajectory from a path/topic-only key. Cache fallback is content-bound,
schema-backed, and fail-closed. This is local cross-validation integrity, not
ground truth, a fresh GLIM execution, or a comparative usability claim.

## Risk removed

The former SHA-1 key covered the resolved bag path, topics, mode, and no-IMU
flag. It did not bind the bag bytes, effective GLIM configuration, runtime,
preset, viewer/OMP options, comparison implementation, or cached trajectory.
A changed input at the same path could therefore reuse an unrelated reference.
Legacy entries have no manifest that can prove those relationships and are now
ignored rather than migrated.

## Enforced contract

`scripts/glim_reference_cache.py` computes a path-free SHA-256 identity over:

- every regular file in the rosbag2 and effective-config trees, including
  relative file layout, size, and bytes;
- the Docker image ID or selected local GLIM executable/library/package bytes;
- the points/IMU topics, GLIM mode and preset, no-IMU/viewer/OMP settings; and
- the comparison harness and cache-helper bytes.

Storage validates at least two finite, strictly time-increasing, eight-field
TUM poses. An immutable manifest conforming to
`glim-reference-cache-v1.schema.json` binds the identity to trajectory SHA-256,
byte count, pose count, and creation time. Lookup recomputes and checks the
identity key, manifest, and trajectory. Symlinks, malformed TUM, missing
artifacts, byte drift, contradictory manifests, and same-key/different-output
collisions are rejected without replacing the existing entry.

Trajectory and manifest bytes are now staged in the cache directory and made
visible with an exclusive hard link only after the complete file is flushed.
If two identical stores race, the loser validates and returns the first
complete manifest. If same-key outputs differ, the loser preserves the entry
that won publication; it cannot delete another writer's file during rollback.

The comparison metrics expose cache enabled/status/key/identity/manifest
fields. `--no-glim-cache` provides an explicit fresh-run-only mode. Cache reuse
also requires the current GLIM runtime identity, so a missing runtime cannot be
hidden by an old trajectory.

## Verification

The focused test suite covers byte-sensitive identities, path omission,
store/lookup idempotence, trajectory and manifest tampering, changed bag bytes,
collision refusal, competing trajectory and manifest publication, malformed
TUM variants, symlink rejection, the public CLI, shell integration, and
release-bundle inclusion:

```text
python3 -m pytest -q -p no:cacheprovider \
  lidarslam/test/test_glim_reference_cache.py
14 passed

ament_flake8 scripts/glim_reference_cache.py \
  lidarslam/test/test_glim_reference_cache.py
No problems found

pydocstyle scripts/glim_reference_cache.py \
  lidarslam/test/test_glim_reference_cache.py
No problems found

bash -n scripts/compare_with_glim.sh
PASS
```

No GLIM run, benchmark claim, network request, upload, or other remote mutation
was performed for this local integrity increment.
