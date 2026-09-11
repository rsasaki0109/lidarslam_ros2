# Humble and Jazzy clean-prefix upgrade evidence — 2026-07-28

Status: **PASS on both supported ROS distributions**.

This is the named local execution record for the source-install upgrade
contract. The automated workflow repeats the same comparison weekly and on
manual dispatch.

## Contract

For each ROS distribution, the gate:

1. extracts the immutable `v0.6.0` `lidarslam` package;
2. builds it into a non-symlinked merge-install prefix;
3. builds candidate commit
   `debbf78beda219c61b06a66d082d568eb979c246` into that same prefix;
4. builds the candidate again into a fresh prefix;
5. compares every package-owned path, executable bit and normalized text
   resource;
6. runs the installed CLI checker against both candidate prefixes.

Binary hashes are intentionally not compared because independent build
directories can change compiler build IDs. Both binaries must exist and retain
their executable mode. Text resources are compared after replacing the
absolute install prefix with `<PREFIX>`.

## Environment and identity

| Field | Humble | Jazzy |
| --- | --- | --- |
| Runtime | ROS 2 Humble container | ROS 2 Jazzy container |
| Architecture | amd64 | amd64 |
| Baseline | `v0.6.0` / `ea3e6fe8464fc7a5a48bfc628b0894176ef2a117` | same |
| Candidate | `debbf78beda219c61b06a66d082d568eb979c246`, clean | same |
| Container digest | `sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061` | `sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` |

## Result

| Assertion | Humble | Jazzy |
| --- | ---: | ---: |
| Gate checks | 11 / 11 PASS | 11 / 11 PASS |
| Baseline package-owned files | 45 | 45 |
| Upgraded candidate files | 104 | 104 |
| Fresh candidate files | 104 | 104 |
| Stale paths | 0 | 0 |
| Missing paths | 0 | 0 |
| Metadata/text mismatches | 0 | 0 |
| Historical `ros2 run lidarslam lidarslam` node | preserved | preserved |
| New `lidarslam-map` product CLI | executable and validated | executable and validated |

The installed checker also exercised `--version`, `doctor`, own-bag dry-run,
`inspect`, the ROS CLI shim and historical ROS executable from an unrelated
working directory.

Machine-readable reports:

- [Humble JSON](install-upgrade-2026-07-28-humble.json) —
  SHA-256 `e1b62b0b47e0f2c6ffe334c6451179be6594248fefdb0149acfd73684f0832b1`
- [Jazzy JSON](install-upgrade-2026-07-28-jazzy.json) —
  SHA-256 `075a997d36ffa49df68e3f088b81941e6fb2b54a7ce44ec40e9bebcd50fd9315`

## Limits

- This proves source-built, non-symlinked merge-prefix upgrades. It does not
  substitute for Debian/ROS buildfarm package-manager upgrade evidence.
- Both source trees currently declare package version `0.6.0`; the gate proves
  install shape and behavior, not semantic-version monotonicity. Release
  version alignment remains a separate gate.
- The execution is amd64-only. arm64/Jetson retains its documented evaluation
  support tier.
