# Docker onboarding machine probes — 2026-08-10

## Result

The immutable v0.9.0 Humble and Jazzy Docker routes both produced a verified
first map with no observed undocumented manual step. These are privacy-bounded
maintainer machine probes, not comparable G0 baselines. The container-host
method could not honestly observe a human's active time or submitted commands,
or isolate a whole filesystem for peak-disk measurement, so all three values
remain `null`.

| Row | Product outcome | Wall time | Workflow RX | Dataset | Output | Measurement status |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| [Humble](g0-docker-humble-20260810-machine-a.json) | `PASS` | 1,440.865 s | 1,770,636,344 B | 517,088,133 B | 132,587,520 B | `INCOMPLETE` |
| [Jazzy](g0-docker-jazzy-20260810-machine-a.json) | `PASS` | 1,140.525 s | 1,906,809,522 B | 517,088,133 B | 132,587,520 B | `INCOMPLETE` |

For both rows, the runner exited zero, the run manifest reported `succeeded`,
the diagnosis reported `success`, the Autoware verifier and first-map receipt
reported `PASS`, and the manifest and receipt hashes were retained. The
onboarding-trial checker reports three missing measurements:
`measurements.active_operator_time_sec`, `measurements.command_count`, and
`measurements.peak_disk_bytes`. The harness invoked the product route itself;
script-internal Docker commands are not operator-submitted commands under the
trial contract and cannot be reconstructed as a human command count.

Do not use the 300.340-second wall-time difference as a ROS distribution or
SLAM performance claim. This is one cold network observation per row; Jazzy
received 136,173,178 more bytes but completed 20.8% sooner. Network throughput
and registry/Zenodo service conditions were not controlled.

## Identity and isolation

Before execution, the GitHub Release image records and remote OCI manifests
agreed on all of the following:

| ROS distribution | v0.9.0 image-index digest |
| --- | --- |
| Humble | `sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53` |
| Jazzy | `sha256:6eabb19ac77ad24fd123772333357a0c5bfdb38055945213722f6484e0f134ef` |

Both release records also reported product version `0.9.0`, platform
`linux/amd64`, status `PASS`, and tag commit
`0df0c4a86df9f68a894c83f8342e4107c3d23b0f`. The installed GitHub CLI did not
provide the `gh attestation` command, so attestation verification was not
claimed.

Each row used a new Ubuntu Docker host container, a new nested daemon data
root, `rprivate` bind propagation, a dedicated network namespace, and empty
project data/output directories. The host Docker socket was never mounted,
and the public project image was pulled only into the fresh nested daemon.
The nested daemon had to report `overlay2`, `/var/lib/docker`, zero images, and
zero containers before timing began.

The container host uses `--privileged`; it is maintainer-only instrumentation
and should preferably run inside a disposable VM. The host's shared root
filesystem also means `df` cannot isolate project disk changes. Publishing a
directory-size estimate as peak disk would violate the trial contract, so the
probe deliberately records `null` instead.

After each bounded record and artifact hash were checked, the exact disposable
trial root was removed. The removed roots occupied 5.9 GB for Humble and
6.3 GB for Jazzy. No shared image, builder, package, or dataset cache was
pruned. Private route logs remain outside Git for audit; they contain paths and
exact internal commands that do not belong in the public records.
`privacy.review_before_sharing: true` records a mandatory review requirement,
not an automatic claim that arbitrary private logs are safe to publish. The
two bounded JSON records were reviewed before being added to Git.

## Product decision

There is no observed Docker route failure to repair: both supported ROS
distributions reached the same receipt contract without help. The strongest
visible activation cost is cold first success itself: one run received
1.77–1.91 GB and took 19–24 minutes. Observer logs show the fixed 517 MB
dataset transfer as the longest visible phase. This activates the roadmap item
to evaluate a substantially smaller onboarding fixture while retaining the
full MID-360 run as the trust/proof route.

The four-row matrix is still incomplete. The immediate matrix blocker is that
the reviewed source-route commit is not published, so a clean remote clone
cannot execute it honestly. Comparable promotion also still requires a
dedicated VM/filesystem and a human active-time observation. Current coverage
is therefore:

- measured product outcomes: `2 / 4` rows, both Docker `PASS`;
- comparable baselines: `0 / 4`;
- source rows: not yet executed;
- independent-user first maps: `0 / 3`.

## Reproduce the machine probe

Follow the isolation and identity gates in the
[execution runbook](../../onboarding-trial-execution.md). Build the observer
host before timing. Invoke the reviewed helper from the product checkout, but
keep its bounded record and all reported trial/observer roots outside that
checkout. A Humble example is:

```bash
BOUNDED_ROOT="$(mktemp -d /tmp/lidarslam-g0-bounded-humble.XXXXXX)"

docker build --pull=false \
  -f docker/onboarding-trial-host.Dockerfile \
  --build-arg UBUNTU_VERSION=22.04 \
  -t lidarslam-onboarding-trial-host:22.04 docker

python3 scripts/run_docker_onboarding_probe.py \
  --trial-id g0-docker-humble-YYYYMMDD-machine-a \
  --ros-distro humble \
  --image-tag ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble \
  --image-digest sha256:27934744bc21ee7081619f35e322177345479ed69079cda8e37ee61fbfbdbe53 \
  --record "$BOUNDED_ROOT/bounded-record.json" \
  --allow-privileged-container-host
```

The acknowledgement flag is intentionally required. The script removes its
named nested-host container but retains the unique trial and observer roots
for review. Validate and archive the bounded record before removing only that
reported trial root; never use a prune command or broad glob.
