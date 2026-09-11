# SOTA-v6 fresh holdout acquisition plan (2026-08-10)

Status: candidate shortlist only. No fresh holdout has been selected, no
ground-truth trajectory has been opened, and no accuracy result may be
reported from this document.

This document implements the next action from the recovery plan: acquire at
least three dataset families that are absent from the prior exposure ledger.
The existing mounted inventory is consumed and must not be relabeled as fresh.

## Entry rules

Before any candidate is promoted to a holdout, record:

1. The official source, license, dataset family, exact sequence, sensor archive
   URL, byte count, and SHA-256.
2. A sensor-only receipt made before opening any trajectory ground truth.
3. The candidate revision, fixed rival revisions, machine identity, metrics,
   thresholds, and run order.
4. A one-global-configuration decision. No sensor- or sequence-specific
   algorithm tuning is allowed after the candidate is admitted.

Ground truth must remain in a separate sealed location until all three
   repetitions are terminal and raw outputs are hash-sealed. A dataset that
   cannot provide this separation is a candidate for compatibility testing, not
   a SOTA holdout.

## Candidate families

These are not yet selected. They are the first acquisition targets because
their official descriptions expose the required sensor and reference
information and they are absent from the mounted inventory audit.

| Family | First sequence to acquire | Sensor/reference indication | Main risk | Official source |
|---|---|---|---|---|
| UrbanLoco | `HK-Data20190117` (6.11 GB) | ROS LiDAR + IMU; SPAN-CPT RTK/INS reference | The ROS bag may contain reference topics and the published PointCloud2 description does not promise a per-point time field; isolate/reference-strip before compatibility testing. | [IPNL dataset page](https://ipnl.io/en/resources/urbanloco-dataset/), [official repository](https://github.com/weisongwen/UrbanLoco) |
| M2DGR | `gate_01` (16.4 GB) | Velodyne VLP-32C + IMU; RTK/INS ground truth | OneDrive availability and large working footprint; keep the separate GT file sealed. | [official repository and sequence table](https://github.com/SJTU-ViSYS/M2DGR) |
| MulRan | `DCC/01` | Ouster OS1-64 + released IMU/GPS; 6D baseline trajectory | Download requires an official request; raw `.bin`/CSV conversion and baseline separation must be audited before admission. | [official download page](https://sites.google.com/view/mulran-pr/download), [official project page](https://sites.google.com/view/mulran-pr/home) |

The smallest listed sensor payloads are already roughly 28 GiB before
archives, extracted files, normalized bags, and run outputs. The storage gate
was later provisioned on the root filesystem; the dataset mount is still too
full to be used as the acquisition target.

## Acquisition order

1. Provision at least 100 GiB of dedicated free space and record the mount
   identity.
2. Acquire sensor archives only, with resumable transfer logs and SHA-256
   receipts. Do not download or open the reference files in the same step when
   the official source separates them.
3. For archives that contain reference topics/files, make a byte-preserving
   sealed copy first, then expose only the LiDAR/IMU allowlist to the
   compatibility converter.
4. Normalize each sensor-only input to the repository's canonical format and
   run structural checks: message types, timestamp monotonicity, LiDAR frame,
   IMU availability, per-point timing, and calibration provenance.
5. Only when all three families pass the sensor-only audit, create the formal
   preregistration and freeze candidate/rival revisions and thresholds.
6. Execute the three repetitions, seal outputs, and only then open the matching
   ground truth for scoring.

## Acquisition status (2026-08-10)

The first target remains M2DGR `gate_01` (the plan's 16.4 GB sequence). The
official sequence table exposes a separate sensor-bag URL and GT URL. A
sensor-only transfer probe was made against the official bag URL, but the
current SharePoint endpoint redirected to Microsoft authentication and
returned HTTP 403 from the command-line client. No bag bytes were retained,
and the GT URL was not opened. This sequence is not acquired or admitted as a
holdout.

The next acquisition action is to obtain a working public transfer path for
M2DGR `gate_01`, or to move to the next official candidate if the source access
condition cannot be resolved. Existing `ntu_viral`, `mid360`, `koide_hard`,
`Boreas`, `MCD`, and other mounted data must not be used to bypass this gate;
the recovery plan marks those families as consumed.

The other two official paths also require an external access step: UrbanLoco's
official form requires agreement to its non-commercial terms and sends the
download link by email, while MulRan exposes an official download request form.
No form was submitted on the user's behalf. Until one of these access steps is
completed or a valid public transfer URL is supplied, no fresh family can be
downloaded or promoted.
