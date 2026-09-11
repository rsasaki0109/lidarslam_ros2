# Japanese beginner quickstart — 2026-08-12

> Decision: **LOCAL_JAPANESE_ENTRY_PASS / PUBLICATION_PENDING**
>
> Candidate base: `36f8cad98397d570b0aef1968d42e1b434e74f41`
>
> Remote mutations performed: **none**

## Why this increment

The product candidate already had one no-argument terminal home, a read-only
system doctor, one-command Docker and source demos, and a guided own-bag path.
The README and maintained documentation exposed those routes only in English.
A Japanese user still had to translate the choice, failure, and distribution
boundaries before entering the same product workflow.

A read-only live review on 2026-08-12 found 837 Stars and an unchanged Draft PR
`#427` at public head `3f4dd70cdc58ad421192559213cdee0bdc41eba8`.
All nine reported PR checks pass. The two `ndt_omp_ros2` rosdistro PRs remain
open with the package-lineage question unanswered. The local follow-up was not
substituted for that public state, and no GitHub comment, label, push, release,
or review reply was made.

## Product change

`docs/getting-started-ja.md` is a deliberately short decision page rather than
a translation fork of the full manual. It exposes only:

1. read-only installation and bag diagnosis;
2. the fixed Humble/Jazzy Docker demo;
3. the six-package source quickstart and dry-run;
4. the canonical `lidarslam-map start` own-bag path;
5. Docker own-bag execution from a checkout; and
6. verified success, stable failure actions, and safe demo post-processing
   resume.

The README caption and Docs home link directly to the page. MkDocs navigation
and the deterministic release bundle include it. The page states the 517 MB,
8 GiB, and approximately 30-minute first-demo expectations and explicitly says
that GLIM-style PPA/package-manager installation is not yet available. Detailed
English contracts remain authoritative, so maintenance does not require two
complete manuals.

## Synchronization contract

The docs entrypoint regression requires the Japanese page to retain the exact
canonical commands, read-only doctor boundary, read-only Docker bag mount,
mapping-free `demo --resume`, `map_verify: PASS`, resource expectations,
package-manager limitation, English canonical links, README/Docs discovery,
and release-bundle inclusion. This turns translation drift into a test failure.

## Verification

| Check | Result |
| --- | --- |
| docs, release bundle, published-release, and install/upgrade regressions | `48 passed` |
| complete maintained Python gate | graph: `1,429 passed / 13 skipped / 11 existing warnings`; lidar_slam: `694 passed`; `2,123 total` |
| Python syntax/undefined-name safety lint | PASS |
| `git diff --check` | PASS |
| `mkdocs build --strict` | PASS with existing Material and unlisted-page notices |
| rendered Japanese page | title, doctor commands, and `map_verify: PASS` present |
| remote writes | none |

## Honest boundary and next measurement

This change reduces Japanese command-discovery cost; it does not prove a lower
completion time, create a binary package, answer the rosdistro review, publish
the larger local candidate, or close any onboarding matrix row. After an exact
candidate is public, one first-time Japanese operator should be timed from
README arrival to a selected safe command, then through the same neutral
onboarding contract. Findings must repair the canonical workflow, not create a
Japanese-only execution path.
