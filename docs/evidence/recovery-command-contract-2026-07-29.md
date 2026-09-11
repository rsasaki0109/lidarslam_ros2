# Recovery command contract evidence — 2026-07-29

## Scope

Candidate commit `6b8c91b` separates routine map authoring from advanced
recovery while keeping both surfaces machine-readable:

- normal top-level help contains `doctor`, `run`, `inspect`, and `view`;
- `--help-all` additionally lists `migrate-manifest` and `rollback-plan`;
- CLI contract JSON, explicit command help, Bash completion, source execution,
  and installed execution publish the same options.

## Safety results

The source contract suite passed 40/40 selected tests. It proves that:

- migration requires an explicit historical verification mode;
- only terminal schema-v1 records with durable execution results are accepted;
- the source and existing destinations are never overwritten, including a
  destination-creation race during atomic publication;
- migrated schema-v2 records use lifecycle stage `complete` and report
  `resume_allowed: false`;
- rollback evidence must match its tag distro, version, CLI version, commit,
  platform, and digest schemas;
- generated pull, attestation, and CLI smoke commands use only an immutable
  `ghcr.io/...@sha256:...` reference and never mutate a tag.

Both new Draft 7 schemas, the CLI JSON contract, Bash syntax, release workflow
YAML, and Python bytecode compilation passed local validation.

## Clean-install execution

A non-symlinked Jazzy merge prefix was built from the candidate with
`BUILD_TESTING=OFF` and `CMAKE_BUILD_TYPE=Release`. The installed-product
checker passed from a temporary unrelated directory. In addition to the
existing version, doctor, dry-run, inspect, viewer, PATH, and ROS shim checks,
it:

- found all four installed recovery schemas;
- migrated a terminal v1 fixture with the installed CLI and installed schemas;
- confirmed the result was inspect-only and non-resumable;
- generated and schema-validated a digest-pinned rollback plan.

The complete package test invocation produced 170 passes and two failures in
the pre-existing benchmark-summary expectation for `mid360_vs_glim`: the test
expects a blocking profile, while the current release profile marks it
`superseded-by-mid360-gt (D-GT-2)` and therefore returns `WARN`. The focused
recovery, CLI, installation, upgrade-contract, and docs suites all passed.
The stale v0.4 expectations were subsequently aligned with the documented
v0.5 D-GT-2 policy; the broader Python suite then passed.

## Remaining boundary

Humble and Jazzy public CI must pass after integration. A real tagged release
must still publish the first `release-image-<distro>.json` and
`rollback-plan-<distro>.json` assets; this local evidence does not claim that
publication occurred.
