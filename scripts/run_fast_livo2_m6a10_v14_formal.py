#!/usr/bin/env python3
"""Unauthorized v14 formal candidate with scoped mount-builder state.

v14 is an additive successor to the v13 persistence candidate.  It fixes the
candidate-only monkeypatch recursion observed before the v13 formal Popen:
the original v12 Docker argv callable is captured before any patch, and the
v14 transformation calls that captured callable directly.  Shared v12 state
is patched only inside ``scoped_runtime`` and restored on every exit path.

The production entry point is intentionally unauthorized.  The injected
one-start seam is host-only and does not run Docker, ROS, bag input, GT,
scoring, or map generation.
"""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Iterator, List, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v12_formal as v12  # noqa: E402


CANDIDATE_VERSION = "v14-scoped-original-builder"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v14_formal_candidate.yaml"
PROFILE_SHA256 = "849585ae1d2bb118db9a261f42305886b0305628473cea964781695f01b63996"
IMAGE_TAG = "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
OUTPUT_DESTINATION = "/out"
WRAPPER_DESTINATION = "/runner/v12_runtime.sh"

# This binding is intentionally captured before scoped_runtime can mutate the
# shared v12 module.  Tests replace this variable with a counting fake.
ORIGINAL_V12_BUILD_DOCKER_ARGV = v12.build_safe_docker_argv


class CandidateError(ValueError):
    """Fail-closed candidate error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _mount_destination(spec: str) -> str:
    for field in spec.split(","):
        if field.startswith("dst="):
            return field[4:]
    raise CandidateError("DOCKER_ARGV", "mount has no destination")


def _mounts(argv: Sequence[str]) -> List[str]:
    return [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]


def _tmpfs_destinations(argv: Sequence[str]) -> List[str]:
    return [argv[index + 1].split(":", 1)[0]
            for index, item in enumerate(argv[:-1]) if item == "--tmpfs"]


def build_safe_docker_argv(config: Any, output_dir: Path) -> List[str]:
    """Transform the captured original argv exactly once and validate it."""
    parent_argv = ORIGINAL_V12_BUILD_DOCKER_ARGV(config, output_dir)
    argv: List[str] = []
    index = 0
    while index < len(parent_argv):
        if (parent_argv[index] == "--tmpfs" and index + 1 < len(parent_argv) and
                parent_argv[index + 1].startswith("/out:")):
            index += 2
            continue
        argv.append(parent_argv[index])
        index += 1
    mount_specs = _mounts(argv)
    destinations = [_mount_destination(spec) for spec in mount_specs]
    if destinations.count(OUTPUT_DESTINATION) != 1 or len(destinations) != len(set(destinations)):
        raise CandidateError("DOCKER_ARGV", "duplicate or missing mount destination")
    expected_destinations = {
        "/input/ntu_viral.bag",
        OUTPUT_DESTINATION,
        WRAPPER_DESTINATION,
        "/runner/scripts/fast_livo2_m6a10_feeder.py",
    }
    if set(destinations) != expected_destinations or len(mount_specs) != len(expected_destinations):
        raise CandidateError("DOCKER_ARGV", "mount destination allowlist drift")
    if OUTPUT_DESTINATION in _tmpfs_destinations(argv):
        raise CandidateError("DOCKER_ARGV", "output tmpfs remains")
    if _tmpfs_destinations(argv) != ["/tmp", "/root/.ros"]:
        raise CandidateError("DOCKER_ARGV", "tmpfs allowlist drift")
    output = output_dir.resolve()
    expected_output = "type=bind,src=%s,dst=/out,readonly=false" % output
    output_mounts = [spec for spec in mount_specs if _mount_destination(spec) == OUTPUT_DESTINATION]
    if output_mounts != [expected_output]:
        raise CandidateError("DOCKER_ARGV", "output bind contract drift")
    if "--rm" in argv or any(item == "rw" for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe cleanup token")
    if argv[:3] != ["docker", "run", "--name"]:
        raise CandidateError("DOCKER_ARGV", "argv is not a docker run invocation")
    if "--network" not in argv or argv[argv.index("--network") + 1] != "none":
        raise CandidateError("DOCKER_ARGV", "network is not none")
    if "--read-only" not in argv or "--init" not in argv or "--pull=never" not in argv:
        raise CandidateError("DOCKER_ARGV", "container safety flags drift")
    forbidden_flags = {
        "--privileged", "--cap-add", "--cap-drop", "--device", "--pid=host",
        "--ipc=host", "--uts=host", "--security-opt", "--userns=host",
    }
    if any(item in forbidden_flags for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe capability or namespace flag")
    if "--entrypoint" not in argv or argv[argv.index("--entrypoint") + 1] != WRAPPER_DESTINATION:
        raise CandidateError("DOCKER_ARGV", "entrypoint drift")
    if any(item in {"sh", "bash", "-c", "-lc", "--shell"} for item in argv):
        raise CandidateError("DOCKER_ARGV", "shell execution token is forbidden")
    if argv[-1] != IMAGE_ID:
        raise CandidateError("IMAGE_IDENTITY", "image ID drift")
    return argv


@contextmanager
def scoped_runtime() -> Iterator[None]:
    """Patch the shared builder only for the dynamic scope and always restore."""
    original = v12.build_safe_docker_argv
    v12.build_safe_docker_argv = build_safe_docker_argv
    try:
        yield
    finally:
        v12.build_safe_docker_argv = original


ProcessFactory = Callable[[Sequence[str], Path], Any]


def run_injected_once(config: Any, process_factory: ProcessFactory) -> Mapping[str, Any]:
    """Exercise one shell-free Popen seam without Docker or benchmark input."""
    output_dir = Path(config.root) / "out"
    output_dir.mkdir(parents=True, exist_ok=False)
    with scoped_runtime():
        argv = v12.build_safe_docker_argv(config, output_dir)
        process = process_factory(argv, Path(config.repo_root))
        pid = getattr(process, "pid", None)
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
            raise CandidateError("RUNNER_PID", "injected process has no positive PID")
        returncode = int(process.wait())
    return {"argv": list(argv), "pid": pid, "returncode": returncode, "start_count": 1, "popen_count": 1}


def verify_candidate_profile(profile_path: Path = PROFILE_PATH) -> Mapping[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v14 profile path differs")
    if profile_path.is_symlink() or not profile_path.is_file():
        raise CandidateError("PROFILE", "v14 profile is not regular")
    observed_sha = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    if observed_sha != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v14 profile SHA-256 differs from candidate pin")
    text = profile_path.read_text(encoding="utf-8")
    required = (
        "V14_FORMAL_CANDIDATE_UNAUTHORIZED",
        "formal_replay_forbidden: true",
        "feeder_root_cause_fixed: false",
        "DOCKER_ARGV_BUILDER_RECURSION",
        "duplicate_output_destinations: false",
        "output_tmpfs: false",
        "persistence_gate.receipt.json",
        "input_opened: false",
    )
    if any(item not in text for item in required):
        raise CandidateError("PROFILE", "v14 profile contract drift")
    return {"path": str(profile_path.resolve()), "sha256": observed_sha}


def main(argv: Optional[Sequence[str]] = None) -> int:
    del argv
    raise SystemExit("FORMAL_REPLAY_UNAUTHORIZED: v14 candidate has no authorization")


if __name__ == "__main__":
    main()
