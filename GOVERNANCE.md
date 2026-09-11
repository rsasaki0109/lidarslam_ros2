# Governance

## Model

`lidarslam_ros2` uses a maintainer-led governance model. The repository owner,
Ryohei Sasaki (`@rsasaki0109`), is the current lead maintainer and release
owner.

The lead maintainer is responsible for:

- defining the supported product surface;
- reviewing and merging changes;
- assigning release-blocking gates;
- security and conduct enforcement;
- release publication and rollback decisions;
- appointing additional maintainers when sustained contribution and project
  need justify it.

This model may evolve as the maintainer group grows. Changes to governance are
made through a reviewed pull request to this file.

## Decision process

Small, reversible changes use normal pull-request review. Decisions that alter
the product contract, defaults, compatibility, licensing, release gates or
governance require:

1. a written problem statement and alternatives;
2. direct evidence where behavior or performance is involved;
3. an explicit maintainer decision in the pull request, roadmap or research
   record;
4. documentation and migration impact in the same release cycle.

When consensus is not available, the lead maintainer makes the final decision
and records the rationale.

## Contribution and review

Contributors do not need prior permission to open an issue or pull request.
Changes should be narrow, reproducible and consistent with
[CONTRIBUTING.md](CONTRIBUTING.md).

Merge rights imply responsibility for review quality, compatibility, license
compliance and keeping CI green. Maintainer status is based on sustained
trusted work, not a fixed number of commits.

## Releases

The release owner verifies the checklist in [RELEASING.md](RELEASING.md),
confirms required gates, creates the tag and publishes or rolls back release
artifacts. No benchmark improvement may bypass a failed product, compatibility
or security gate.

## Community conduct

All project spaces follow [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
Enforcement decisions are made privately by the lead maintainer or a delegated
maintainer without a conflict of interest.
