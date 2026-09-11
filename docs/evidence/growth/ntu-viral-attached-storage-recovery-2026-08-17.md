# NTU VIRAL attached-storage recovery — 2026-08-17

## Decision

The canonical NTU VIRAL `tnp_01` acquisition is now actionable on a host whose
root filesystem cannot hold the complete working set. Exact implementation
`8a856f521de825976c80c6a3c410224c4fb4e433` adds the read-only storage
resolver and bundle closure; `657746f…` registers the new regression surface,
and `d6e8bad…` adds exact byte reporting.

This is acquisition readiness only. The blocking `ntu_viral_tnp_01` release
profile remains `NO_DATA`; no archive, bag, conversion, benchmark result, tag,
image, or comparative claim was created.

## Reproduced user problem

A fresh default preparation retains the official archive, extracted ROS 1 bag,
converted rosbag2, and RKO-LIO-restamped rosbag2 until completion. The exact
planner therefore requires **49,209,878,965 bytes**, including its conservative
10% reserve. The final implementation-carrier dry-run observed only
**6,326,681,600 bytes** free on the root filesystem, an exact shortfall of
**42,883,197,365 bytes**.

The host already exposes one suitable attached filesystem:

- device: `/dev/sda1`;
- model: SanDisk Extreme Portable SSD;
- filesystem: ext4;
- label: `aiueo`;
- partition size: 2,000,397,795,328 bytes; and
- state: unmounted, with free capacity deliberately unverified.

The old recovery only said to choose a large path and replace `--dest DIR`.
That left device discovery, mounting, mount-path lookup, and command
reconstruction to the user.

## Enforced recovery contract

The shared helper performs one bounded Linux `lsblk` read with a three-second
timeout. It accepts only writable hotplug/USB filesystems large enough to be a
candidate, excludes mounted and internal NVMe partitions, strips control
characters from optional display text, and never probes filesystem contents.
Partition size is never counted as available bytes.

The NTU command now:

- reports the candidate and `UNVERIFIED_UNTIL_MOUNTED` capacity boundary;
- chooses `udisksctl mount -b /dev/sda1` as the one next action when available;
- accepts mutually exclusive `--dest-device /dev/sda1`;
- resolves exactly one printable absolute mountpoint after the user mounts it;
- appends `ntu_viral` without mount-path substitution;
- preserves `--keep-zip`, `--no-convert`, and `--no-restamp` in generated retry
  commands; and
- reruns the real free-space plan before any download or destination creation.

An absent, malformed, unmounted, ambiguously mounted, or control-character
mountpoint fails closed. The helper does not mount, request a password, bypass
Polkit, inspect data, write a directory, or start network work.

The exact current handoff is:

```bash
udisksctl mount -b /dev/sda1

bash scripts/download_ntu_viral_tnp01.sh \
  --dest-device /dev/sda1 \
  --dry-run
```

Only after the second command reports `READY` may the operator remove
`--dry-run`.

## Release-bundle closure

`docs/benchmarking.md` already starts the recommended benchmark with
`scripts/download_ntu_viral_tnp01.sh`, but the curated candidate bundle
previously omitted that script. The release inventory now carries both the NTU
helper and `scripts/attached_storage.py`. The bundle regression extracts and
executes both help surfaces, so the documented first command is no longer a
source-checkout-only accident.

## Validation before evidence synchronization

- shared storage, NTU, and RTK acquisition regressions: **34 passed**;
- deterministic release-bundle regressions: **9 passed**;
- focused total: **43 passed**;
- every pytest file registered with CTest: **PASS**;
- changed-Python Jazzy `ament_flake8`: **PASS**;
- NTU shell parse and repository `git diff --check`: **PASS**;
- real `--dry-run`: **BLOCKED_INSUFFICIENT_SPACE**, exact candidate and
  commands reported, exit 0, destination absent; and
- real `--dest-device /dev/sda1 --dry-run` while unmounted: exit 2 with the
  single mount action, before planning or side effects.

Clean validation carrier `b01b251cb8e4b7430d730476977e4c012e091afe`
passes the complete source-explicit product gate:

- `graph_based_slam`: **1,477 passed / 13 skipped / 11 known ImageIO
  warnings**;
- `lidarslam`: **1,040 passed**; and
- combined maintained Python gate: **2,517 passed / 13 skipped**.

Strict MkDocs, the 317-path publication plan, shell parsing, repository diff,
and changed-Python Jazzy style also pass. The same clean carrier passes the
canonical two-build and two-reverification candidate-bundle rehearsal. Both
builds are byte-identical and contain 267 manifest files; the retained
11,959,011-byte archive has SHA-256
`f963391bf76f67e27828bf0c8eadada484ac8b8b9481e5f904fcec72e1c64bad`.
Its archived NTU, attached-storage, and RTK helpers execute their bounded help
or identity-list surfaces. This is local candidate evidence, not a published
release asset, and a later commit must rerun rather than reuse its checksum.
