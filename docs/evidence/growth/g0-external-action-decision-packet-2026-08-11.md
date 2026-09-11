# G0 external-action decision packet — 2026-08-11

> Status: **DECISION_REQUIRED / NO_REMOTE_ACTION_TAKEN**
>
> Audited product revision:
> `6a8727a9014aea1ecfe8ea9c65d6f10cffb87cd3`
>
> Public base: `develop` at
> `86fa9b610c07ccf4d2b0f10939e17c129d34b40a`

This packet separates source publication, geometry-bearing fixture
publication, community mutations, and a stable release. Approval of one row
does not approve another.

The audit and this packet must be carried by one documentation-only descendant
of the product revision. E1 approval must name that final carrier hash after
its exact-tip checks pass; approval of `6a8727a` alone does not silently
authorize an unknown later commit.

## Recommended sequence

1. review the fixed 42-commit product candidate and this decision packet;
2. audit the one-commit evidence carrier and name its exact hash;
3. approve E1 source publication as a non-force branch push plus Draft PR only;
4. review public Humble/Jazzy CI and the complete product/carrier diff;
5. choose E2a Zenodo or E2b GitHub immutable Release for the fixture, or defer;
6. after a successful remote fixture audit, run four fresh onboarding rows;
7. authorize any E3 issue operation only after its read-only drift check; and
8. keep E4 stable release denied until G0 evidence and version planning pass.

This order makes source, fixture, and community identities independently
reviewable before external validators are asked to use them.

## E1 — source branch and Draft PR

### Proposed operation

- rerun lineage, worktree, issue-drift, docs, and scoped test checks on the
  exact final evidence-carrier tip;
- require the carrier delta from `6a8727a` to contain only
  `g0-clean-candidate-audit-2026-08-11.md` and
  `g0-external-action-decision-packet-2026-08-11.md`;
- push local branch `agent/product-g0-guided-ux` to `origin` without force;
- open one **Draft** pull request into `develop`;
- describe the five review themes from the
  [clean-candidate audit](g0-clean-candidate-audit-2026-08-11.md); and
- do not merge, tag, publish a release, move an image tag, or change an issue.

### Evidence

- public `develop` equals local `origin/develop` at
  `86fa9b610c07ccf4d2b0f10939e17c129d34b40a`;
- 42 linear product commits, 0 merges, clean worktree, and 116 changed paths;
- 66 added and 50 modified paths, with no deletion, binary, symlink,
  research/generated, bag, map, archive, model, or private-data path;
- largest changed blob is 75,559 bytes;
- exact-tip Jazzy product Python suites pass 1,406 graph tests and 484
  lidarslam tests;
- Humble product Python suites pass 1,382 graph tests and 484 lidarslam tests
  at code revision `e2a4dfc`; its product/code delta to `6a8727a` is empty;
- Humble/Jazzy scanmatcher component recovery passes ten consecutive runs and
  complete 10 / 10 CTest; no later scanmatcher path changed;
- issue proposal coverage/live drift is 29 / 29 PASS with 18 checker tests and
  no mutation;
- v1 remains honestly `8 / 10`; matrix remains 2 / 4 present and 0 / 4
  comparable; and
- the remote branch is absent and GitHub reports no commit for `6a8727a`.

### Recommendation

**APPROVE E1** if the intended outcome is public review, but only after the
final carrier hash is shown and its two-file delta passes the prescribed
audit. A Draft PR is the smallest reversible source-publication action that
removes the source-identity blocker while retaining merge and release control.

### Reversibility

The Draft PR can be closed and the remote branch deleted. Public review events
and commit objects may remain visible in GitHub history or caches, so source
publication is not equivalent to a private preview. No force push is proposed.

## E2 — onboarding fixture host and upload

The exact derivative packet remains:

| Item | Identity |
| --- | --- |
| ZIP | `mid360_onboarding_50s_v1.zip`; 98,873,952 bytes; SHA-256 `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3` |
| Build manifest | SHA-256 `60c37f5c7efa7d61ca20f21803fa11b02add4bad047ae99d277e9e6811fbbb6e` |
| Geometry-free map receipt | SHA-256 `86d2b5d2aa493cbb6ecc6efd88095a591f247bcea4bc171c68093cf165cc0754` |
| Local gate | 13 / 13 PASS; two byte-identical clean rebuilds |
| License/provenance | CC BY 4.0 derivative attribution to the pinned public Zenodo MID-360 source |

### E2a — Zenodo derivative record (recommended)

Use a versioned Zenodo record because this is a dataset derivative with its
own attribution, checksum, and lifecycle. Reserve the DOI in a draft, review
metadata and exact files, then publish only after a fresh readiness PASS.

A published Zenodo version is intentionally durable. Corrections require a
new version and explicit linkage rather than replacement.

### E2b — future GitHub immutable Release asset

Use only a future release that reports `immutable: true`; existing v0.9.0 is
not immutable and must not be altered for this purpose. This keeps the file
near source but couples dataset lifecycle to a source release and provides no
dataset DOI.

### E2c — defer

Keep the packet local. This is safe but leaves the smaller onboarding route and
public comparable fixture rows blocked. The full 517,088,133-byte public
source route remains available and unchanged.

### Recommendation

**APPROVE E2a only after E1 is publicly resolvable**, if publishing the
geometry-bearing derivative is intended. Approval must name Zenodo and the
exact ZIP SHA-256 above. Draft creation/upload and final publication remain
two review points; no credential or local path enters evidence.

## E3 — GitHub issue/community mutations

The read-only proposal still covers all 29 open issues and passes current live
drift. It proposes 23 reasoned closures, four supported-version reproduction
requests, two keep-open issues, and five separate starter drafts bounded to
approximately 30 prepared-environment minutes.

Choose each batch independently:

| Batch | Proposed mutation | Recommendation |
| --- | --- | --- |
| E3a — labels and reproduction requests | apply reviewed labels without removing observed labels; post four bounded current-reproduction requests | approve only with an immediate live drift check and exact mutation log |
| E3b — 23 reasoned closures | post issue-specific answered/resolved/superseded/not-planned text and close | review wording separately; do not infer from E3a or generic cleanup approval |
| E3c — five starter issues | duplicate-check and create C1–C5 with files, non-goals, acceptance, and focused checks | useful after E1 so contributors can reference public candidate files |
| E3d — Discussions | enable/configure categories and moderation | defer until weekly moderation capacity and support boundary are staffed |

Issue comments and closures remain in public event history even if later
reopened. The checker has no write mode; every approved mutation needs a
separate execution log and post-write read-back.

## E4 — stable release

### Recommendation

**DO NOT APPROVE E4 now.**

`VERSION` remains `0.9.0`, and immutable tag `v0.9.0` names historical
commit `0df0c4a`. Exact-candidate bundle rehearsal correctly refuses to reuse
that version. The four-row onboarding matrix is incomplete, source and fixture
identities are unpublished, and v1 readiness remains `8 / 10`.

A later release decision must select a new semantic version, update all
versioned package/release records, reproduce the two-build bundle and image
gates, and explicitly authorize tag, release, package, and image publication.

## Exact decision form

A maintainer can respond with one line per gate, for example:

```text
E1: APPROVE exact tip <FULL_40_CHARACTER_HASH> for non-force branch push + Draft PR only
E2: APPROVE Zenodo draft/upload of SHA-256 20e515...bd3; final publish requires another review
E3a: APPROVE labels + four reproduction requests
E3b: DEFER 23 closures
E3c: APPROVE five starter issues after E1
E3d: DEFER Discussions
E4: DENY stable release for now
```

Omitted rows remain **not authorized**. “Continue,” “go ahead,” approval of the
long-term goal, or approval of one gate does not silently authorize a push,
upload, issue change, release, or another gate.

## Decision consequences

| Choice | Immediate next evidence |
| --- | --- |
| E1 approved for exact carrier | public branch/PR identity, supported CI, review result, and public commit resolution |
| E2a/E2b approved after E1 | `PUBLICATION_READY`, host draft review, immutable published identity, remote re-download SHA-256 |
| E3 batch approved | pre-write live drift PASS, bounded mutation log, post-write read-back |
| E4 later approved | version plan, clean release-candidate revision, bundle/image/package publication audits |
| all deferred | continue local reliability and documentation; G0 remains HOLD |
