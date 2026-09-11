# Named-hardware real-data soak evidence — 2026-07-28

This ledger records the first completed one-hour and eight-hour executions of
the v4 soak contract on named hardware. It is evidence for the Phase 3
resource-measurement gate, not a universal performance claim. The compact
machine-readable record is
[`real-data-soak-2026-07-28.json`](real-data-soak-2026-07-28.json).

## Fixed identities

| Item | Recorded identity |
| --- | --- |
| Hardware | `sasaki-laptop-i5-1145G7-32GiB-jazzy-native`; Intel i5-1145G7, 8 logical CPUs, 32 GiB, x86_64 |
| OS / ROS | Ubuntu 24.04.4 LTS, kernel 6.17.0-35-generic, ROS 2 Jazzy |
| Product and harness | clean commit `0ec55575ffc16eb008e9f24bd6c6f24700bf2f8a`, product 0.6.0 |
| Map profile | `pointcloud_gnss_smoke` |
| Bag metadata | SHA-256 `1b9be8f18b0cbbb369c3f38549ac4c3bb7f8bb895ef417c0fbe57a0a926d2f71` |
| Bag storage | `istanbul_bag4_0.db3`, 1,061,437,440 bytes, SHA-256 `e833a6ae89d5f0ccf0ee8d4bdde3232e7c49d04a8342db452b4ad743ef43b185` |

Both reports recorded the same clean software, harness and input identities.
The full reports were validated against
[`soak-report-v4.schema.json`](../schemas/soak-report-v4.schema.json).

## Results

| Profile | Result | Iterations | Wall time | Longest iteration | Peak RSS | Output | Minimum free | Drops |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| one-hour | PASS | 86 / 86 | 3,638.645 s | 43.164 s / 300 s | 209.320 MiB / 1,024 MiB | 1,889,888,063 B / 5 GiB | 85,689,724,928 B / 60 GiB | 0 / 0 |
| eight-hour | PASS | 671 / 671 | 28,834.115 s | 45.732 s / 300 s | 210.871 MiB / 1,024 MiB | 14,553,359,036 B / 30 GiB | 70,752,927,744 B / 40 GiB | 0 / 0 |

All eight terminal checks were `true` for both runs:
`all_iterations_succeeded`, `dropped_inputs_within_budget`,
`free_space_within_budget`, `iteration_duration_within_budget`,
`output_size_within_budget`, `peak_rss_within_budget`,
`provenance_recorded`, and `target_duration_reached`.

The one-hour report SHA-256 is
`25a677d0944fec42bc63421c67a3a055993afd6d44a1991dfe5b49b973f35a3e`.
The eight-hour report SHA-256 is
`67aace0d8521178357bdc3264acdeca57e35c414ecf9dd516b29cf9612eb1361`.
After the eight-hour run, no process scoped to the evidence worktree remained
and the detached evidence worktree was clean.

## Reproduction template

Use the exact input represented by the hashes above, a clean checkout of the
recorded commit, and budgets appropriate for the target machine:

```bash
python3 scripts/run_map_soak.py /data/bags/istanbul_bag4_uncompressed \
  --output-root /data/soak/one-hour \
  --soak-profile one-hour \
  --hardware-label '<non-secret-machine-label>' \
  --map-profile pointcloud_gnss_smoke \
  --max-peak-rss-mib 1024 \
  --max-output-gib 5 \
  --max-dropped-inputs 0 \
  --max-iteration-secs 300 \
  --min-free-space-gib 60 \
  --telemetry-interval-secs 30

python3 scripts/run_map_soak.py /data/bags/istanbul_bag4_uncompressed \
  --output-root /data/soak/eight-hour \
  --soak-profile eight-hour \
  --hardware-label '<non-secret-machine-label>' \
  --map-profile pointcloud_gnss_smoke \
  --max-peak-rss-mib 1024 \
  --max-output-gib 30 \
  --max-dropped-inputs 0 \
  --max-iteration-secs 300 \
  --min-free-space-gib 40 \
  --telemetry-interval-secs 30
```

Do not compare a rerun to these numbers unless its input hashes, product
commit, map profile and resource budgets are recorded. A passing soak proves
bounded operational behavior for this configuration; map geometry quality
remains covered by the separate real-data and release gates.
