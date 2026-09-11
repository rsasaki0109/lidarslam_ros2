# Published fixture remote-audit gate — 2026-08-11

## Result

The project now has a host-aware, read-only audit for the onboarding fixture
after publication. The implementation is fixed at commit
`f97709b8e65433e22ab2a48f3382540b703bc9f4` and emits the
[`fixture-publication-audit-v1`](../../schemas/fixture-publication-audit-v1.schema.json)
contract only after host metadata and downloaded bytes agree with the exact
publication-readiness review.

Current fixture status remains **unpublished**. This work did not push a
branch, enable a repository setting, create or edit a release, create a Zenodo
record, upload the geometry-bearing ZIP, or change the default demo. The gate
is therefore `IMPLEMENTED_NOT_RUN_ON_FIXTURE`, not remote publication proof.

## Readiness boundary

The command first requires the exact SHA-256 of a schema-valid
`PUBLICATION_READY` review. It fails before any host request unless that
review records:

- a 13/13 `LOCAL_ARTIFACT_PASS`;
- both generator and reviewed product revisions as publicly resolvable;
- one selected host and explicit upload authorization;
- no publication blockers; and
- the exact fixture filename, size, and SHA-256.

The current tracked review is deliberately
`AWAITING_PUBLICATION_DECISION`, so it cannot be used to manufacture a remote
PASS.

## Host metadata checks

| Host | Required metadata before download | Recorded integrity level |
| --- | --- | --- |
| GitHub Release | exact repository and simple release tag; final, non-prerelease release; `immutable: true`; one uploaded asset matching filename, size, and SHA-256 digest; immutable release/asset IDs and canonical browser URL | `ENFORCED` |
| Zenodo | exact numeric record ID; published, submitted, open-access version; valid version DOI and canonical record URL; unique filename with exact size and Zenodo MD5; version-scoped file endpoint and file ID | `CHECKSUM_PINNED_VERSION` |

The GitHub checks follow the official
[release response](https://docs.github.com/en/rest/releases/releases),
[asset digest](https://docs.github.com/en/rest/releases/assets), and
[release immutability](https://docs.github.com/en/code-security/how-tos/secure-your-supply-chain/establish-provenance-and-integrity/prevent-release-changes)
contracts. A read-only inspection of the existing `v0.9.0` release on
2026-08-11 returned `immutable: false`; the new gate rejected it before asset
download. A future GitHub fixture release therefore cannot pass until release
immutability is enabled and the future release reports the exact reviewed
asset digest.

The Zenodo checks follow the official
[records and file API](https://developers.zenodo.org/) and
[published-record workflow](https://help.zenodo.org/docs/get-started/quickstart/).
The adapter was exercised read-only against the existing public source record
`14841855`: it resolved the 517,088,133-byte file and MD5 metadata correctly.
That check did not download the full source bag and does not imply that the
new fixture exists on Zenodo.

## Download and independent verification

After metadata passes, both hosts use the same hardened acquisition path as
the installed public-data intake:

- exact-size and SHA-256 pins from the readiness review;
- Zenodo MD5 as an additional host-reported identity;
- retained `.part` state and validated HTTP Range resume;
- safe restart when a server ignores Range;
- cache re-hashing and atomic finalization; and
- final-path and output-directory symlink rejection.

The audit then opens the finalized file independently and recomputes its size,
SHA-256, and MD5. It rejects disagreement between the file, downloader report,
host digest, source classification, or transfer-byte totals. The shareable
receipt contains no local path, raw response, credential, map geometry, or
raw bag metadata.

## Reproduction commands after publication

First regenerate and checksum a `PUBLICATION_READY` review. For an immutable
GitHub Release:

```bash
python3 scripts/audit_published_fixture.py \
  --readiness-review /path/to/publication-ready-review.json \
  --expected-review-sha256 <64-hex-review-sha256> \
  --output-dir /path/to/cold-audit-download \
  --github-release-tag <future-immutable-release-tag> \
  --output /path/to/published-fixture-audit.json
```

For a published Zenodo version record:

```bash
python3 scripts/audit_published_fixture.py \
  --readiness-review /path/to/publication-ready-review.json \
  --expected-review-sha256 <64-hex-review-sha256> \
  --output-dir /path/to/cold-audit-download \
  --zenodo-record-id <numeric-version-record-id> \
  --output /path/to/published-fixture-audit.json
```

Do not use `--force` for the first cold audit. It exists only to restart an
inspected partial transfer or deliberately replace a cached artifact; the
same size and SHA-256 gates remain mandatory.

## Verification evidence

The new direct suite has 14 tests covering both host PASS routes and rejection
of mutable GitHub releases, wrong host digests, draft/restricted Zenodo
records, a waiting or changed readiness review, changed downloaded bytes,
inconsistent transfer accounting, symlinked output, and host-selector drift.
The hardened standalone downloader adds cache, missing-pin, and symlink tests.

Validation completed on 2026-08-11:

- 39 focused audit/downloader/CTest registration checks passed;
- all 466 `lidarslam/test` cases passed;
- 44 related public-data, default-CI, and documentation-entrypoint checks
  passed;
- Python compilation, Draft 7 schema validation, full flake8 on the new audit
  files, critical flake8 on reused intake files, and `git diff --check`
  passed; and
- the clean source at `f97709b8e65433e22ab2a48f3382540b703bc9f4`
  produced two byte-identical, internally verified 139-file release bundles:
  11,267,613 bytes, SHA-256
  `406af79cfad535b506164802eea144e2f0f5c55b880a47731906b31a82c39969`.

Neither synthetic tests nor existing-host metadata substitute for the future
fixture's cold download. A real `REMOTE_ARTIFACT_PASS` receipt remains a hard
gate before registry activation, default-demo changes, dedicated-VM trials,
or broad promotion.
