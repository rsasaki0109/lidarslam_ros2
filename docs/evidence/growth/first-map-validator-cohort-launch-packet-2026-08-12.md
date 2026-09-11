# Independent first-map validator cohort packet — 2026-08-12

> Status: **WAITING_FOR_PUBLIC_GATES / NOT_AUTHORIZED**
>
> Accepted first maps: **0 / 3**
>
> Community/GitHub writes authorized: **no / no**
>
> Remote mutations performed: **none**

This is the bounded launch plan for the first independent-user cohort. It
prepares the operating loop; it does not recruit anyone, send a direct
message, post to a community, modify issue #422, or claim a validation.

## Why launch is waiting

The tracked machine contract reports `WAITING_FOR_PUBLIC_GATES` because all of
the following must become true together:

- the reviewed product candidate is a publicly resolvable exact commit;
- at least one clean Docker row is comparable;
- at least one clean source row is comparable;
- the lower-burden passing row is selected as the canonical cohort path; and
- the deployed page manifest binds that path's bytes to the exact public
  source revision; and
- that path has an exact public commit or immutable GHCR digest; and
- the copy-ready `support --first-map` handoff is present in that public
  revision.

The current public source/docs identity is product `0.9.1` at exact commit
`dec8ec286953aea42cbcb2f7de70a41042f24e62`. The earlier identity-only Pages
audit was `VERIFIED` (page SHA-256
`e675e4a97353b110934ec014e5cfe085a2b364adf5de4447fda994a4bdd2413a`), but the
new rendered-content audit is `NOT_READY` until the receipt-only handoff is
deployed. The source preflight returns `READY`. A maintainer-controlled Docker
run and a source-builder run both completed the fixed first map with PASS
receipts; the full details are retained in the [2026-08-20 public first-map
checkpoint](public-first-map-verification-2026-08-20.md).

The cohort still has zero comparable clean-host rows and
`copy_ready_handoff_public=false` because the deployed page/image pair has not
yet carried the receipt-only handoff contract. The Docker path is the
published stable `v0.9.0` image while the source path is the `v0.9.1`
candidate, and the v0.9.1 release plus immutable Humble/Jazzy GHCR images are
absent. Human active-time/command-count observations and independent reports
are also missing. Broad promotion or recruitment now would turn these missing
gates into volunteer support work, so the packet remains
`WAITING_FOR_PUBLIC_GATES`. The receipt-only report handoff and historical
source-anchor compatibility fix described in the checkpoint are local changes
until an authorized reviewed publication updates the public Pages bytes and
runtime image. The source-checkout receipt-helper fallback is likewise local
documentation until that publication; it does not reopen the cohort gate by
itself.

## Cohort shape

| Control | Fixed value | Reason |
| --- | ---: | --- |
| accepted target | 3 | v1 external-adoption contract |
| first batch | at most 5 attempts | enough room for honest failures without opening an unbounded campaign |
| hard assessment point | 10 attempts | roadmap completion-rate and active-time decision point |
| concurrent attempts | at most 2 | keeps evidence review and product repair inside maintainer WIP |
| completion threshold at attempt 10 | at least 80% | pause acquisition when onboarding is the constraint |
| median active operator time at attempt 10 | at most 10 minutes | GLIM-level convenience target for the fixed demo |

An attempt begins when an eligible volunteer starts the named public route at
the exact public revision. Abandoned, failed, and successful attempts all
count in the completion denominator when their outcome is reported. GitHub
traffic, clones, CI, maintainer demos, and silent visitors are not attempts.

The machine-readable
[`first-map-validator-cohort-state.json`](first-map-validator-cohort-state.json)
records only anonymous attempt IDs, lifecycle state, public product identity,
the exact public documentation page SHA-256, timing, public report/blocker
links, and an accepted-ledger ID when applicable.
It never stores a participant handle or private contact detail. The evaluator
cross-checks every `accepted-pass` against the authoritative accepted-first-map
ledger, so a planning record cannot create an adoption claim.
`closed-pass` contributes only to the product completion-rate denominator and
numerator; it does not increase the accepted count without a matching ledger
entry.

## Independence and support boundary

- Maintainers cannot count.
- The operator follows only public documentation and receives no live
  step-by-step help during the attempt.
- Both PASS and FAIL reports are retained. A failure is not rewritten into a
  success.
- After an attempt ends, maintainers may acknowledge it and discuss the
  product finding publicly. The original attempt remains failed.
- A later run may count only after the underlying product/docs fix is public
  and the operator again works independently; it is recorded as a new
  attempt, never as an edited outcome.
- No private bag, map, location, parameter set, raw log, or screenshot is
  requested. Only a human-reviewed privacy-bounded receipt may be attached.

## Bounded service level

- acknowledge a cohort report within 2 business days;
- complete receipt/evidence review within 5 business days;
- publicly disposition a supported P0 within 7 days and P1 within 14 days;
- refresh the public source/docs, supported-P0, release-gate, and
  privacy/safety signal audit at least every 48 hours while attempts are open;
- keep no more than two attempts or receipts awaiting substantive review.

These are capacity promises, not acceptance guarantees. If they cannot be
met, recruitment pauses and the supported scope is reduced before another
channel is opened.

## Stop and repair

Pause new attempts immediately when any condition below is true:

1. public source or documentation drifts from the named revision;
2. a supported P0 or failed release gate is open;
3. two receipts are waiting for review;
4. two attempts hit the same blocker;
5. any privacy or safety incident occurs;
6. completion is below 80% at attempt 10; or
7. median active operator time exceeds 10 minutes at attempt 10.

For a repeated blocker, fix the public product or docs, rerun the affected
clean matrix row, and only then resume. Do not answer with private workaround
instructions that preserve a broken public path.

`python3 scripts/first_map_validator_cohort.py --json` enforces this loop. It
refuses partial operational-signal snapshots, more than two combined
active/unreviewed items, more than five attempts before
an exact public extension-decision comment, duplicate public reports, unbound
accepted IDs, active routes with a different page hash, and more than ten
attempts. A signal snapshot older than 48 hours returns to
`WAITING_FOR_OPERATIONAL_SIGNALS`, while future-dated evidence is rejected.
Accepted attempts must match the accepted ledger's public report URL,
documentation path, and immutable runtime identity. Cohort accepted IDs are a
validated subset of the cumulative adoption ledger, so later cohorts can grow
that ledger without rewriting this bounded state. The evaluator derives
`CAPACITY_FULL`, `INITIAL_BATCH_REVIEW`,
`HARD_CAP_REVIEW`, `PAUSED_REPAIR`, or `TARGET_MET`; only
`READY_FOR_NEXT_ATTEMPT` permits the next attempt at the state layer. None of
those states authorizes recruitment or a GitHub write.

## Recruitment policy

After the launch gates pass and a separate E3 decision approves the exact
wording/channel:

- update or reference the existing public tracking issue #422 first;
- make at most one bounded post in one relevant ROS 2/Autoware community;
- accept volunteers in arrival order subject only to independence and the
  two-attempt WIP limit;
- do not cold-message stargazers, scrape identities, buy traffic, exchange
  rewards for Stars, or require a positive result; and
- stop recruiting when three reports are accepted or any stop condition
  fires.

The GET-only contributor queue now enforces the same precondition for the
existing issue. It retains #422 in the published inventory but does not offer
it as a contributor next step unless this evaluator derives exactly
`READY_FOR_NEXT_ATTEMPT`. `WAITING_FOR_PUBLIC_GATES`, stale operational
signals, any pause/review state, or full WIP all remain blocked. This local
decision card neither edits #422 nor supplies E3 authority.

The script renders final recruitment wording only after every public launch
gate is true:

```bash
python3 scripts/first_map_validator_cohort.py --json
python3 scripts/first_map_validator_cohort.py --render
```

Today the first command validates launch contract, anonymous operating state,
and accepted ledger together, reports `WAITING_FOR_PUBLIC_GATES`, and the
second fails closed. Rendering copy-ready text still does not authorize
posting it.

Without `--json`, the first command prints a human-facing status card with
accepted/attempt/WIP counts, completion and median-time values, signal
freshness, stop conditions, the write-authority boundary, and exactly one
next action for the derived state. It contains no participant identity or
private evidence.

## Privacy-bounded reporting

The repository stores anonymous attempt IDs, aggregate accepted/attempted
counts, durations, public issue links required by the operating/acceptance
ledgers, and explicit product findings.
It does not store a recruited-person list, stargazer identities, private
contact details, raw receipts, maps, bags, or telemetry. Public issue authors
remain visible on GitHub by their own voluntary submission; their identities
are not copied into growth snapshots or this packet.

Exact local follow-up `da99c7ef82136449727ef97a58c1a2db4ffd6955` removes a
form contradiction before launch: PASS still requires one reviewed receipt,
while a FAIL whose run produced no receipt can attest no-receipt/no-file
without falsely claiming an attachment review. Every privacy checkbox remains
required. This makes future failed attempts reportable; it is not recruitment,
a cohort attempt, an accepted validation, an upload, or a GitHub write.

## Next transition

1. Align the Docker and source rows to one reviewed product version, then run
   fresh dedicated-VM Docker/source rows against the exact identities with
   human active-time observation.
2. Repair any repeated blocker and select the lower-burden comparable PASS.
3. Run `check_public_docs_deployment.py` against the selected exact revision;
   update the machine contract with its verified page SHA-256, public path,
   and immutable runtime identity; require `COPY_READY_NOT_AUTHORIZED` and
   review the rendered text.
4. Record a fresh public operational-signal audit in the anonymous state and
   require `READY_FOR_NEXT_ATTEMPT`.
5. Request E3 authorization for the exact issue/community write scope.
6. Start at most two attempts and operate the stop/repair loop above.

No step may mark the v1 gate complete until three distinct public reports pass
the existing receipt and maintainer acceptance contract.
