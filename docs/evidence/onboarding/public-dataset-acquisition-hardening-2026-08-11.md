# Public dataset acquisition hardening — 2026-08-11

## Result

The installed public MID-360 intake now supports checksum-pinned HTTP Range
resume and transaction-safe ZIP extraction. The implementation is fixed at
commit `eae85479180d5cafb797cf53db03af57a4363067`. It prepares the acquisition
path required by the 50-second onboarding fixture without publishing that
fixture, assigning it a URL, or changing the default demo.

The existing full `Driving SLAM Test with Livox MID360` registry row is now
pinned to all three known identities:

| Identity | Expected | Read-only local recheck |
| --- | --- | --- |
| Size | 517,088,133 B | `PASS` |
| SHA-256 | `f8f89eebf2aaf9cc1d465bfa5451bbb599cd92d079b59949104bb4e5cb619bdd` | `PASS` |
| Legacy MD5 | `0836c50859bb1af591966b69da166186` | `PASS` |

The recheck streamed the existing public source archive without modifying it.
Its ZIP directory also passed the new extraction preflight with exactly two
regular members and 1,468,937,686 uncompressed bytes.

## Download contract

Downloads continue to use `<archive>.part` until all registered identities
pass. The final archive name appears only after an atomic rename. The intake
manifest now records expected and actual size, SHA-256, MD5, source
(`network`, `resume`, `complete-part`, or `cache`), resumed bytes, transferred
bytes, and each verification result.

The resume path requires all of the following:

- an existing regular `.part` file, never a symlink or directory;
- `Range: bytes=<current-size>-` with `Accept-Encoding: identity`;
- HTTP `206` with an exact `Content-Range` start, total, and consistent
  `Content-Length` before appending;
- safe restart with write mode `wb` when a server ignores Range and returns
  the complete object with HTTP `200`;
- no write beyond a registered expected size;
- exact response-byte completion when `Content-Length` or `Content-Range`
  provides a bound; and
- final size and SHA-256 equality before promotion from `.part`.

The legacy `--skip-md5` option cannot disable a registered SHA-256 or size
check. Existing final archives are re-hashed instead of being trusted by
filename. A failed transfer or digest check retains diagnostic partial bytes
but never replaces the final archive; `--force` is the explicit restart path.

## Extraction contract

The intake no longer calls `ZipFile.extractall()` directly into its final
directory. It validates the entire ZIP directory first and rejects:

- absolute, traversal, backslash, NUL, or non-normalized paths;
- exact or case-folded duplicate paths;
- encrypted members; and
- symlinks and every other non-regular special-file type.

Validated members are streamed into `.<name>.partial`. Only a complete
extraction is renamed into the final directory. During explicit forced
replacement, the old final directory remains in place until the new
transaction completes; an injected copy failure proved the old directory is
not destroyed. A stale partial or previous-transaction directory fails with a
specific recovery instruction instead of being silently reused.

## Automated evidence

The focused public-dataset and installed-runtime suites completed with
`45 passed`. The 21 direct intake cases include:

- exact `206` resume;
- a Range-ignoring `200` restart;
- wrong `Content-Range` rejection;
- SHA-256/MD5 mismatch rejection;
- SHA-256 enforcement with `--skip-md5`;
- oversized response rejection;
- connection-loss partial retention;
- complete-part finalization without network access;
- existing-cache tamper detection;
- traversal and ZIP-symlink rejection;
- failed forced-extraction preservation; and
- successful transaction replacement.

The related dataset report, map-runner, historical verified-file,
installed-product CLI, and curated-runtime contracts also passed. Python
compilation, critical flake8 checks, and `git diff --check` passed.

Reproduce the focused suite with:

```bash
python3 -m pytest -q \
  graph_based_slam/test/test_mid360_robot_public_datasets.py \
  graph_based_slam/test/test_mid360_robot_public_dataset_report.py \
  graph_based_slam/test/test_mid360_robot_public_dataset_map_runner.py \
  graph_based_slam/test/test_download_verified_file.py \
  lidarslam/test/test_runtime_docker_image_contract.py \
  lidarslam/test/test_installed_product_cli_contract.py
```

## Fixture activation boundary

No unpublished URL is present in the registry. After a host is explicitly
selected and the remote fixture passes the
[host-aware publication audit](published-fixture-audit-gate-2026-08-11.md),
activation requires one registry row containing the exact 98,873,952-byte
size and
`20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3`
SHA-256. The same installed downloader can then acquire either the bounded
onboarding fixture or the full proof bag without a second acquisition
implementation.

This work does not prove a cold network transfer, hosting availability,
dedicated-VM disk peak, Humble/Jazzy parity, or an external user's first map.
Those remain publication and four-row onboarding gates. The full 277-second
dataset remains the default until the shortened fixture is authorized,
published, downloaded back, hash-audited, and measured on clean VMs.
