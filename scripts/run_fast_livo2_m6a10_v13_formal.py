#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""v13 mount-corrected formal candidate.

This additive candidate keeps the v12 image, wrapper, profile, and evidence
lineage immutable.  It changes only the Docker argv construction: v12 placed
both a tmpfs and a bind mount at ``/out``.  Docker selected the tmpfs, so the
container's evidence disappeared when it exited.  v13 retains the output bind
mount and removes only the duplicate ``/out`` tmpfs.  It is still unauthorized
by default and must not be used for formal replay until a new authorization is
sealed.
"""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v12_formal as v12  # noqa: E402


CANDIDATE_VERSION = "v13-mount-corrected"
PARENT_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v12_formal.py"
PARENT_LAUNCHER_SHA256 = "a33d30b65c5cd9eceb613985dbb18e7b7776ce33e5a971430115649a9e0683f8"
PARENT_PROFILE_PATH = v12.PROFILE_PATH
PARENT_PROFILE_SHA256 = v12.PROFILE_SHA256
IMAGE_ID = v12.IMAGE_ID
IMAGE_TAG = v12.IMAGE_TAG
PARENT_WRAPPER_PATH = ROOT / "scripts/fast_livo2_m6a10_v12_formal_container_run.sh"
PARENT_WRAPPER_SHA256 = "af6fa54f834ae209758ee5a9903695674f4095c1063ac57b46c2774bfbc5c7a9"

FAILED_ATTEMPT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_formal_replay_20260823T190641Z_agentv12"
)
FAILED_CLOSURE_PATH = FAILED_ATTEMPT_ROOT / "closure_receipt.json"
FAILED_CLOSURE_SHA256 = "499e739ecba7f79eefcc1a50f9d7c88c6860ae441b4c037acdf58e7ada82111c"
FAILED_CONTAINER_INSPECT_SHA256 = "f82008ca5a12f88279d13168a354066fec052548b9afdf038c9b31acaf786316"
FAILED_CONTAINER_LOG_SHA256 = "9ebcdbb9936de7138db2039a90ae8a4e94cd17a269937bfe4b223a5afb4ae048"
FAILED_CONTAINER_DIFF_SHA256 = "5746c35982671142f4d49f085f3024f37e675b4e438e9e145329205a66676559"

OUTPUT_DESTINATION = "/out"
OUTPUT_BIND_CONTRACT = "v13-single-output-bind-no-duplicate-mount-v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _assert_pin(path: Path, expected: str, label: str) -> str:
    v12._regular(path, label)
    observed = _sha256(path)
    if observed != expected:
        raise v12.CandidateError("SOURCE_DRIFT", "%s SHA differs from pin" % label)
    return observed


def verify_v13_lineage(repo_root: Path = ROOT) -> Mapping[str, Any]:
    """Verify v12 lineage and sealed failed-attempt evidence read-only."""
    parent = repo_root / PARENT_LAUNCHER_PATH.relative_to(ROOT)
    profile = repo_root / PARENT_PROFILE_PATH.relative_to(ROOT)
    wrapper = repo_root / PARENT_WRAPPER_PATH.relative_to(ROOT)
    _assert_pin(parent, PARENT_LAUNCHER_SHA256, "v12 parent launcher")
    _assert_pin(profile, PARENT_PROFILE_SHA256, "v12 parent profile")
    _assert_pin(wrapper, PARENT_WRAPPER_SHA256, "v12 parent wrapper")
    v12.verify_candidate_profile(profile, repo_root)

    _assert_pin(FAILED_CLOSURE_PATH, FAILED_CLOSURE_SHA256, "failed v12 closure")
    diagnostics = {
        "container_inspect": FAILED_ATTEMPT_ROOT / "container_diagnostics/container.inspect.json",
        "container_logs": FAILED_ATTEMPT_ROOT / "container_diagnostics/container.logs.txt",
        "container_diff": FAILED_ATTEMPT_ROOT / "container_diagnostics/container.diff.txt",
    }
    expected_diagnostics = {
        "container_inspect": FAILED_CONTAINER_INSPECT_SHA256,
        "container_logs": FAILED_CONTAINER_LOG_SHA256,
        "container_diff": FAILED_CONTAINER_DIFF_SHA256,
    }
    observed_diagnostics = {}
    for label, path in diagnostics.items():
        observed_diagnostics[label] = _assert_pin(path, expected_diagnostics[label], label)
    return {
        "candidate_version": CANDIDATE_VERSION,
        "parent_launcher": {"path": str(parent), "sha256": PARENT_LAUNCHER_SHA256},
        "profile": {"path": str(profile), "sha256": PARENT_PROFILE_SHA256},
        "wrapper": {"path": str(wrapper), "sha256": PARENT_WRAPPER_SHA256},
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "failed_attempt": {
            "root": str(FAILED_ATTEMPT_ROOT),
            "closure_path": str(FAILED_CLOSURE_PATH),
            "closure_sha256": FAILED_CLOSURE_SHA256,
            "diagnostics": observed_diagnostics,
        },
    }


def _mount_destination(spec: str) -> str:
    for field in spec.split(","):
        if field.startswith("dst="):
            return field[4:]
    raise v12.CandidateError("DOCKER_ARGV", "mount has no destination")


def build_safe_docker_argv(config: Any, output_dir: Path) -> List[str]:
    """Build v13 argv by removing only v12's duplicate ``/out`` tmpfs."""
    parent_argv = v12.build_safe_docker_argv(config, output_dir)
    argv: List[str] = []
    index = 0
    while index < len(parent_argv):
        if (parent_argv[index] == "--tmpfs" and index + 1 < len(parent_argv) and
                parent_argv[index + 1].startswith("/out:")):
            index += 2
            continue
        argv.append(parent_argv[index])
        index += 1

    if "--rm" in argv or any(item == "rw" for item in argv):
        raise v12.CandidateError("DOCKER_ARGV", "unsafe cleanup or bare rw token")
    mount_specs = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]
    destinations = [_mount_destination(spec) for spec in mount_specs]
    if len(destinations) != len(set(destinations)):
        raise v12.CandidateError("DOCKER_ARGV", "duplicate Docker mount destination")
    output_mounts = [spec for spec in mount_specs if _mount_destination(spec) == OUTPUT_DESTINATION]
    if output_mounts != ["type=bind,src=%s,dst=/out,readonly=false" % output_dir.resolve()]:
        raise v12.CandidateError("DOCKER_ARGV", "output mount is not the single explicit RW bind")
    if any(item == "--tmpfs" and argv[index + 1].startswith("/out:")
           for index, item in enumerate(argv[:-1])):
        raise v12.CandidateError("DOCKER_ARGV", "output tmpfs was not removed")
    return argv


def run_formal(config: Any, **kwargs: Any) -> Mapping[str, Any]:
    """Run the v12 injected lifecycle with only the v13 argv correction."""
    original = v12.build_safe_docker_argv
    v12.build_safe_docker_argv = build_safe_docker_argv
    try:
        return v12.run_formal(config, **kwargs)
    finally:
        v12.build_safe_docker_argv = original


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Delegate to the fail-closed v12 CLI with the v13 argv patch."""
    original = v12.build_safe_docker_argv
    v12.build_safe_docker_argv = build_safe_docker_argv
    try:
        return v12.main(argv)
    finally:
        v12.build_safe_docker_argv = original


if __name__ == "__main__":
    raise SystemExit(main())
