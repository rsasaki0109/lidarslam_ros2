# 1,000-Star quarterly health-review template

Copy this template into a dated evidence document. Replace every placeholder;
do not delete an unanswered field. The review is a decision record, not a
marketing update.

## Review identity

| Field | Value |
| --- | --- |
| Review window | `YYYY-MM-DD` through `YYYY-MM-DD` |
| Captured at | `YYYY-MM-DDTHH:MM:SSZ` |
| Current phase | `G0`–`G9` |
| Source revision | full 40-character public or explicitly local-only Git SHA |
| Reviewer | role or approved public identity |
| Decision | `ADVANCE`, `HOLD`, `REPLAN`, or `SUSTAIN` |
| Next review | `YYYY-MM-DD` |

## Outcome summary

In five sentences or fewer, state what became easier, what remains unsafe or
unproven, and why the selected decision follows from the evidence.

## Phase exits

| Required exit | Status | Evidence | Blocker / next action |
| --- | --- | --- | --- |
| replace with every exit for the current phase | `PASS`, `FAIL`, or `INCOMPLETE` | repository-relative path or public URL | exact condition and owner |

No phase advances with a missing row or an `INCOMPLETE` quality exit. A Star
checkpoint is reported separately and cannot turn an exit into `PASS`.

## Funnel scorecard

| Stage | Current value | Prior comparable value | Interpretation |
| --- | ---: | ---: | --- |
| Stars | — | — | lagging discovery signal |
| 14-day qualified referrals | — | — | top-referrer limitation stated |
| Primary bundle downloads | — | — | release and age stated |
| Independent first maps | — | — | accepted cumulative / attempted window |
| First-run completion and active time | — | — | sample size and workflow stated |
| External PRs / merged contributors | — | — | 90-day / 180-day windows |
| Untriaged issues and response time | — | — | support scope stated |
| Stable release age | — | — | exception, if any, stated |
| Review-owner coverage | — | — | named domain, not only a count |

Use privacy-bounded aggregates. Do not include stargazer, issue-author,
validator, or contributor identities without a separate reason and consent.

## Product and evidence health

| Check | Status | Evidence / finding |
| --- | --- | --- |
| Supported P0/P1 queue | — | — |
| Clean install and upgrade | — | — |
| Canonical first-map path | — | — |
| Release rollback/recovery | — | — |
| Public claim reproduction | — | — |
| Dependency and license drift | — | — |
| Private/local-only critical input | — | — |

## Capacity and ownership

| Signal | State |
| --- | --- |
| Product/community/research allocation | — |
| Product WIP | — |
| Community WIP | — |
| Research WIP | — |
| Contributions awaiting substantive review | — |
| Single-owner procedure or subsystem | — |
| Support load exception | — |

State which scope will be reduced if capacity is insufficient. Do not reduce a
quality gate to fit the quarter.

## Experiment decisions

| Previous experiment | Baseline | Result | Decision and reason |
| --- | --- | --- | --- |
| one row per active experiment | — | — | `CONTINUE`, `STOP`, or `REDIRECT` |

## Largest current constraint

Name exactly one constraint in the discovery → activation → trust →
contribution → sustainability chain. Include the evidence that makes it larger
than the alternatives.

## Next bounded portfolio

| Slice | Outcome | Owner | Stop condition | Due/review date |
| --- | --- | --- | --- | --- |
| Product/release | one outcome | — | — | — |
| Community | one outcome | — | — | — |
| Research | zero or one outcome | — | — | — |

## External-action state

| Gate | State | Exact authorized scope or blocker |
| --- | --- | --- |
| E1 source publication | `AUTHORIZED` or `NOT_AUTHORIZED` | — |
| E2 artifact publication | `AUTHORIZED` or `NOT_AUTHORIZED` | — |
| E3 community mutation | `AUTHORIZED` or `NOT_AUTHORIZED` | — |
| E4 stable release | `AUTHORIZED` or `NOT_AUTHORIZED` | — |

Authorization for one row must not be copied into another.

## Final decision

Record `ADVANCE`, `HOLD`, `REPLAN`, or `SUSTAIN`, the evidence-based reason,
the next review date, and the person or role responsible for the next bounded
outcome.
