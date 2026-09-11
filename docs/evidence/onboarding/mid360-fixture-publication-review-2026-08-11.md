# MID-360 fixture publication review — 2026-08-11

## Decision

The 50-second MID-360 fixture is locally verified but not authorized for
publication. The machine-readable review reports
`LOCAL_ARTIFACT_PASS` and `AWAITING_PUBLICATION_DECISION`. No branch, tag,
release, Zenodo record, image, fixture, map, or bag was pushed or uploaded.
The full 277-second public route remains the default.

The four current publication blockers are:

1. generator commit `0f91452c505e03fc810c79a0b6e602101ffefce7` is not
   resolvable from the public GitHub repository;
2. reviewed product commit `eae85479180d5cafb797cf53db03af57a4363067` is not
   publicly resolvable;
3. the publication host has not been selected; and
4. no explicit upload authorization has been granted.

The generator is an ancestor of the reviewed gate commit. Publishing the
reviewed branch would therefore make both commits inspectable, but that push
is a separate maintainer decision.

## Verified packet

Two clean rebuilds from the exact generator revision completed in 64.44 and
61.84 seconds. Their ZIP and manifest byte streams matched exactly.

| Item | Size | SHA-256 / result |
| --- | ---: | --- |
| Fixture ZIP | 98,873,952 B | `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3` |
| Build manifest | 6,078 B | `60c37f5c7efa7d61ca20f21803fa11b02add4bad047ae99d277e9e6811fbbb6e` |
| Geometry-free map receipt | 2,006 B | `86d2b5d2aa493cbb6ecc6efd88095a591f247bcea4bc171c68093cf165cc0754` |
| Publication review | 4,628 B | `efd7e7b08f1050c89f45a71580a58bd0189b4e1264155826c39caea0c25193a8` |
| Local gate | 13 / 13 | `LOCAL_ARTIFACT_PASS` |
| Map route | 7 / 7 | `PASS` |

The [schema-valid publication review](mid360-onboarding-50s-v1-publication-review-20260811.json)
contains only basenames, hashes, counts, revisions, and decision state. It has
no local path, command transcript, map geometry, or upload credential.

The gate revalidated both JSON schemas and exact JSON hashes, then opened the
ZIP without extracting it. It streamed and hashed all 264,882,669
uncompressed member bytes and required:

- exactly three sorted, unique, fixed-root members;
- no traversal path, absolute path, case-fold collision, directory, symlink,
  encryption, archive comment, member comment, or extra field;
- Unix regular-file mode `0644`, fixed 1980 timestamp, and DEFLATE for every
  member;
- exact outer ZIP size/hash and exact per-member size/hash closure;
- creator, title, source, DOI, citation, CC BY 4.0 link, change notice,
  onboarding-only limit, full-gate limit, and no-endorsement notice; and
- the exact seven-check map receipt, successful verifier result, same
  generator revision, and geometry-free shareability boundary.

The geometry-free receipt binds the reviewed private run outputs by hash but
does not embed the input ZIP hash. The gate therefore treats it as the exact
reviewed companion receipt, not as independent proof of a future remote
download. Every post-publication VM row must first hash the downloaded ZIP and
record that identity in its trial evidence.

## Non-publishing gate

The review tool has no upload, release, tag, or push operation. Exit code `0`
means all local and external readiness inputs support `PUBLICATION_READY`;
exit code `1` means the local packet passed but a publication decision or
external prerequisite remains; exit code `2` means the evidence is invalid.

The current review can be reproduced without exposing a local path in its
output:

```bash
python3 scripts/check_fixture_publication.py "$FIXTURE_ZIP" \
  --manifest docs/evidence/onboarding/mid360-onboarding-50s-v1-build-20260810.json \
  --map-receipt docs/evidence/onboarding/mid360-onboarding-50s-v1-map-receipt-20260810.json \
  --expected-manifest-sha256 60c37f5c7efa7d61ca20f21803fa11b02add4bad047ae99d277e9e6811fbbb6e \
  --expected-map-receipt-sha256 86d2b5d2aa493cbb6ecc6efd88095a591f247bcea4bc171c68093cf165cc0754 \
  --review-id mid360-onboarding-50s-v1-2026-08-11 \
  --reviewed-on 2026-08-11 \
  --review-revision eae85479180d5cafb797cf53db03af57a4363067 \
  --generator-revision-remote-status UNRESOLVABLE \
  --review-revision-remote-status UNRESOLVABLE \
  --clean-rebuilds 2 \
  --rebuild-artifacts-byte-identical \
  --rebuild-manifests-byte-identical \
  --output fixture-publication-review.json
```

The two remote statuses came from read-only GitHub commit lookups. They are
explicit review inputs so the local archive audit stays deterministic and
network failures cannot silently become a pass.

## Hosting decision packet

The 98,873,952-byte artifact fits both candidate services. Current official
GitHub documentation permits release assets below 2 GiB and exposes asset
SHA-256 and download count through the release API. GitHub also supports
immutable future releases, which lock the tag and attached assets and provide
a verifiable attestation. The read-only release API reported
`immutable: false` for the existing `v0.9.0` release. Before choosing that
route, an authenticated admin must enable release immutability and the future
release itself must report `immutable: true`.

Zenodo currently permits up to 100 files and 50 GB per upload and registers a
DOI when a record is published; a DOI may also be reserved while the record is
a draft. That route provides a dataset-oriented citation and metadata record,
but requires a deliberate derivative-record description and version policy.

| Option | Strong fit | Required pre-publication proof | Trade-off |
| --- | --- | --- | --- |
| GitHub immutable Release asset | source-version proximity, familiar download path, API digest and download count | enable immutability for a future release; draft the release; attach ZIP, manifest, ready review, and checksum; publish only after asset verification | no DOI; release/tag lifecycle is coupled to the source repository |
| Zenodo derivative record | DOI, citation metadata, license and source relationship | reserve/record the DOI; describe the derivative and CC BY 4.0 attribution; upload the exact packet; publish only after checksum review | additional metadata and version-management work |

References: [GitHub release quotas](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases),
[GitHub immutable releases](https://docs.github.com/en/code-security/how-tos/secure-your-supply-chain/establish-provenance-and-integrity/prevent-release-changes),
[GitHub release integrity](https://docs.github.com/en/code-security/how-tos/secure-your-supply-chain/secure-your-dependencies/verify-release-integrity),
[Zenodo upload limits](https://help.zenodo.org/docs/deposit/create-new-upload/), and
[Zenodo DOI reservation](https://help.zenodo.org/docs/deposit/describe-records/reserve-doi/).

No host is selected by this review. Whichever route is approved must preserve
the exact ZIP identity, publish the manifest and ready-state review alongside
it, and pass a fresh remote download/hash audit before the product downloader
or onboarding documentation changes.

## Remaining promotion sequence

1. Obtain explicit permission to publish the reviewed source branch; verify
   both exact commits through the public GitHub API.
2. Select one host and explicitly authorize or reject the geometry-bearing
   upload.
3. Rerun the non-publishing gate with the remote statuses, selected host, and
   authorization; require `PUBLICATION_READY`.
4. Create a draft, upload only the exact verified packet, publish it, and
   record the immutable record/release ID, URL, remote digest, and downloaded
   SHA-256 with the implemented
   [host-aware remote audit](published-fixture-audit-gate-2026-08-11.md).
5. Register the audited remote URL in the now
   [checksum-pinned resumable acquisition path](public-dataset-acquisition-hardening-2026-08-11.md),
   with the full public route as a fallback; do not silently replace the full
   proof gate.
6. Run the four cold Humble/Jazzy Docker/source VM rows from the publicly
   resolvable reviewed revision before changing an image tag or broadening
   promotion.
