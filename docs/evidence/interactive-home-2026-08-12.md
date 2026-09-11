# Interactive first-decision home evidence — 2026-08-12

## Decision

The installed product now treats a no-argument invocation on an interactive
terminal as a small intent selector. It offers only the existing fixed demo,
own-rosbag2 `start`, previous-session catalog, read-only installation `doctor`,
and complete help. This repairs the “discover the supported path” task in the
GLIM usability scorecard without adding an estimator or mapping workflow.

No-argument behavior remains mode-sensitive and fail-closed:

- both stdin and stdout must be interactive terminals before a prompt appears;
- non-interactive invocation preserves stderr help and usage exit `2`;
- every selected route shows one shell-quoted, copy-ready command before it can
  delegate;
- demo download and writes require an explicit `yes`, with Enter meaning no;
- own-bag selection retains the normal sensor and calibration review;
- installation doctor uses no network, writes no files, and needs no
  confirmation;
- EOF, quit, empty bag input, rejected confirmation, and three invalid choices
  start no delegated command.

The machine contract is
[`interactive_home_contract`](../contracts/cli-v1.json). The product contract
continues to expose bounded beginner workflows; the home is only a smaller
front door to four common user intents plus complete help.

## Verification

The candidate was verified from the private product worktree based on
`3f4dd70cdc58ad421192559213cdee0bdc41eba8`:

- focused interactive-home, option-contract, installed-contract, and docs
  tests: `46 passed`;
- complete ROS-sourced `lidarslam/test` suite: `622 passed`;
- `ament_flake8` on the five changed Python files: PASS;
- `ament_pep257` on the implementation, installed checker, and new test: PASS;
- `ament_copyright` on the new test: PASS;
- strict MkDocs build: PASS;
- README line budget: `220 / 220`;
- source command in a real PTY: selecting demo and accepting the default-no
  confirmation exited zero with `No changes made` and started no workflow;
- fresh Jazzy `--merge-install` copy build of `lidarslam`: PASS;
- complete installed-product CLI validator, including captured no-argument
  exit `2`: PASS;
- installed `lidarslam-map` in a real PTY: displayed the home and quit with
  exit zero and `No changes made`.

An initial unsourced Python run passed 612 tests and failed ten rosbag fixture
tests because `rosbag2_py` was not on `PYTHONPATH`. The same complete suite was
rerun after sourcing `/opt/ros/jazzy/setup.bash` and passed all 622 tests; the
environmental failure is not counted as product evidence.

The later bag-optional doctor follow-up is verified separately in the
[system-doctor evidence](growth/glim-parity-system-doctor-2026-08-12.md). It
adds the fourth intent without weakening the original non-interactive, demo
confirmation, or own-bag calibration contracts.

## Limits and next evidence

This change reduces command-choice friction, but no claim of GLIM usability
parity is made. The clean Humble/Jazzy onboarding matrix still needs a published
candidate, fresh disposable hosts, human active-time measurement, and the full
fixed dataset. Package-manager convenience also remains behind upstream
`ndt_omp` convergence and rosdistro evidence.

No commit, push, pull request, review reply, package publication, image
publication, or public UX claim was made in this milestone.
