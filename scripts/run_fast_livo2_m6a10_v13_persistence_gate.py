#!/usr/bin/env python3
"""Run one isolated v13 output-persistence gate.

This is a report-only candidate gate.  It does not open the training input,
start ROS, run a feeder, invoke a scorer, or save a map.  A tiny shell probe
inside the pinned image writes deterministic marker/stderr/receipt bytes to a
single RW ``/out`` bind and exits naturally.  The host then verifies those
bytes after exit, captures diagnostics, removes only the verified exited
container, and seals an immutable receipt.

The v12 feeder's underlying nonzero cause remains unknown.  This gate proves
only the previously established output-loss mechanism and the v13 corrected
mount contract.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v13_formal as v13  # noqa: E402


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v13_persistence_candidate.yaml"
PROFILE_SHA256 = "4559f7c359156ea878ae962fab334263481a649c32c12610581ba81536eb9760"
V13_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v13_formal.py"
V13_LAUNCHER_SHA256 = "1dbfa6aeee1c3d21239b71a264e6e6d73226c9a77e65cb59872c9537a465b76a"
V12_WRAPPER_PATH = ROOT / "scripts/fast_livo2_m6a10_v12_formal_container_run.sh"
V12_WRAPPER_SHA256 = "af6fa54f834ae209758ee5a9903695674f4095c1063ac57b46c2774bfbc5c7a9"
IMAGE_TAG = "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"

FAILED_ATTEMPT_ROOT = v13.FAILED_ATTEMPT_ROOT
FAILED_CLOSURE_PATH = v13.FAILED_CLOSURE_PATH
FAILED_CLOSURE_SHA256 = v13.FAILED_CLOSURE_SHA256
FAILED_DIAGNOSTIC_SHA256 = {
    "inspect": v13.FAILED_CONTAINER_INSPECT_SHA256,
    "logs": v13.FAILED_CONTAINER_LOG_SHA256,
    "diff": v13.FAILED_CONTAINER_DIFF_SHA256,
}

V12_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_candidate.yaml"
V12_PROFILE_SHA256 = "46e2f91d4cd1c2d84de499e0a45b7a1c88aa684ed8439923739946bb3b0ce207"
V12_NO_INPUT_RECEIPT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_dual_service_attempt6_20260823T175000Z_agentv12/"
    "no_input_dual_evidence.receipt.json"
)
V12_NO_INPUT_RECEIPT_SHA256 = "c5afa2452f96588bf46dc7f0d0b4504e816d0069bbd3a107009eb2c3beac0b45"
V12_NO_INPUT_SIDECAR_SHA256 = "076cd96e10c78d0bf73a6a7ed66d3549aaa2dd0f75c7503d5d88e19237d608e3"
V12_HOST_RECEIPT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_host_evidence_gate_20260823T180718Z_agentv12/"
    "host_evidence_gate.receipt.json"
)
V12_HOST_RECEIPT_SHA256 = "3a603bc4ece0e9167b0b3943df6be569f2e64bce196beb18da29ff09ad69f643"
V12_HOST_SIDECAR_SHA256 = "ff50620daf076ed357032111d33197da1903b840605fa34259f3880ae0e49f66"

CONTRACT = "m6a10-v13-output-persistence-no-input-v1"
OUTPUT_DIR_NAME = "out"
OUTPUT_DESTINATION = "/out"
WRAPPER_DESTINATION = "/runner/v12_runtime.sh"
MARKER_PATH = "/out/persistence.marker"
STDERR_PATH = "/out/feeder.stderr"
RECEIPT_PATH = "/out/feeder_receipt.json"
MARKER_BYTES = b"m6a10-v13-persistence-marker-v1\n"
STDERR_BYTES = b"m6a10-v13 synthetic feeder stderr persistence probe\n"
RECEIPT_BYTES = (
    b'{"contract":"m6a10-v13-output-persistence-no-input-v1",'
    b'"formal_replay_started":false,"status":"PASS"}\n'
)
PAYLOAD = "\n".join(
    (
        "set -eu",
        "printf '%s\\n' " + shlex.quote(MARKER_BYTES.decode().rstrip("\n")) + " > /out/persistence.marker",
        "printf '%s\\n' " + shlex.quote(STDERR_BYTES.decode().rstrip("\n")) + " > /out/feeder.stderr",
        "printf '%s\\n' " + shlex.quote(RECEIPT_BYTES.decode().rstrip("\n")) + " > /out/feeder_receipt.json",
        "printf '%s\\n' " + shlex.quote(STDERR_BYTES.decode().rstrip("\n")) + " >&2",
        "sync",
    )
) + "\n"


class GateError(RuntimeError):
    """Fail-closed persistence gate error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess]


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise GateError("NOT_REGULAR", "not a regular file: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_pin(path: Path, expected: str, label: str) -> str:
    observed = sha256_file(path)
    if observed != expected:
        raise GateError("SOURCE_DRIFT", "%s SHA mismatch" % label)
    return observed


def _atomic_create(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise GateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name + ".part")
    if os.path.lexists(staging):
        raise GateError("OUTPUT_STAGING", "staging path exists: %s" % staging)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(staging, flags, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(staging, path, follow_symlinks=False)
        os.unlink(staging)
        path.chmod(mode)
    except Exception:
        try:
            staging.unlink()
        except FileNotFoundError:
            pass
        raise
    return sha256_file(path)


def reserve_root(root: Path) -> Path:
    root = Path(root)
    if os.path.lexists(root):
        raise GateError("ROOT_REUSE", "fresh root required: %s" % root)
    if root.parent.is_symlink() or not root.parent.is_dir():
        raise GateError("ROOT_PARENT_INVALID", str(root.parent))
    root.mkdir(mode=0o755)
    marker = root / ".reserved-v13-persistence-gate"
    _atomic_create(marker, (CONTRACT + "\n").encode("ascii"))
    output = root / OUTPUT_DIR_NAME
    output.mkdir(mode=0o755)
    return root


def verify_candidate_lineage(repo_root: Path = ROOT) -> Mapping[str, Any]:
    """Verify all old sealed pins and the additive v13 source pins read-only."""
    observed = v13.verify_v13_lineage(repo_root)
    _regular_pin(repo_root / PROFILE_PATH.relative_to(ROOT), PROFILE_SHA256, "v13 persistence profile")
    _regular_pin(repo_root / V13_LAUNCHER_PATH.relative_to(ROOT), V13_LAUNCHER_SHA256, "v13 launcher")
    _regular_pin(repo_root / V12_WRAPPER_PATH.relative_to(ROOT), V12_WRAPPER_SHA256, "v12 wrapper")
    _regular_pin(V12_PROFILE_PATH, V12_PROFILE_SHA256, "v12 candidate profile")
    _regular_pin(V12_NO_INPUT_RECEIPT, V12_NO_INPUT_RECEIPT_SHA256, "v12 no-input receipt")
    _regular_pin(V12_NO_INPUT_RECEIPT.with_suffix(V12_NO_INPUT_RECEIPT.suffix + ".sha256"), V12_NO_INPUT_SIDECAR_SHA256, "v12 no-input sidecar")
    _regular_pin(V12_HOST_RECEIPT, V12_HOST_RECEIPT_SHA256, "v12 host receipt")
    _regular_pin(V12_HOST_RECEIPT.with_suffix(V12_HOST_RECEIPT.suffix + ".sha256"), V12_HOST_SIDECAR_SHA256, "v12 host sidecar")
    return {
        "v12_lineage": observed,
        "v13_profile": {"path": str(PROFILE_PATH), "sha256": PROFILE_SHA256},
        "v13_launcher": {"path": str(V13_LAUNCHER_PATH), "sha256": V13_LAUNCHER_SHA256},
        "v12_wrapper": {"path": str(V12_WRAPPER_PATH), "sha256": V12_WRAPPER_SHA256},
        "v12_candidate_profile": {"path": str(V12_PROFILE_PATH), "sha256": V12_PROFILE_SHA256},
        "v12_host_receipts": {
            "no_input": {"path": str(V12_NO_INPUT_RECEIPT), "sha256": V12_NO_INPUT_RECEIPT_SHA256},
            "host_evidence": {"path": str(V12_HOST_RECEIPT), "sha256": V12_HOST_RECEIPT_SHA256},
        },
    }


def container_name(root: Path) -> str:
    token = hashlib.sha256(str(root.resolve(strict=False)).encode("utf-8")).hexdigest()[:20]
    return "m6a10-v13-persist-" + token


def _mount_destination(spec: str) -> str:
    fields = dict(field.split("=", 1) for field in spec.split(",") if "=" in field)
    if "dst" not in fields:
        raise GateError("DOCKER_ARGV", "mount has no destination")
    return fields["dst"]


def build_persistence_argv(root: Path, repo_root: Path = ROOT) -> List[str]:
    """Build one shell-free Docker probe with exactly one output destination."""
    output = (root / OUTPUT_DIR_NAME).resolve()
    wrapper = (repo_root / V12_WRAPPER_PATH.relative_to(ROOT)).resolve()
    if output.is_symlink() or wrapper.is_symlink() or not wrapper.is_file():
        raise GateError("PATH_INVALID", "output or wrapper is not a safe regular path")
    mounts = [
        "type=bind,src=%s,dst=/out,readonly=false" % output,
        "type=bind,src=%s,dst=%s,readonly" % (wrapper, WRAPPER_DESTINATION),
    ]
    argv = [
        "docker", "run", "-d", "-i", "--name", container_name(root),
        "--network", "none", "--read-only", "--init", "--pull=never",
        "--tmpfs", "/tmp:rw,noexec,nosuid,size=32m",
        "--tmpfs", "/root/.ros:rw,noexec,nosuid,size=16m",
    ]
    for mount in mounts:
        argv.extend(("--mount", mount))
    argv.extend(("--entrypoint", "/bin/sh", IMAGE_ID, "-ceu", PAYLOAD))
    if "--rm" in argv or any(item == "rw" for item in argv):
        raise GateError("DOCKER_ARGV", "unsafe cleanup or bare rw token")
    parsed_mounts = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]
    destinations = [_mount_destination(item) for item in parsed_mounts]
    if len(destinations) != len(set(destinations)) or destinations.count(OUTPUT_DESTINATION) != 1:
        raise GateError("DOCKER_ARGV", "duplicate or missing output destination")
    if any(item == "--tmpfs" and argv[index + 1].split(":", 1)[0] == OUTPUT_DESTINATION
           for index, item in enumerate(argv[:-1])):
        raise GateError("DOCKER_ARGV", "output tmpfs is forbidden")
    if any("/input" in item or "bag" in item.lower() for item in parsed_mounts):
        raise GateError("DOCKER_ARGV", "input mount is forbidden")
    output_mount = [item for item in parsed_mounts if _mount_destination(item) == OUTPUT_DESTINATION]
    if output_mount != ["type=bind,src=%s,dst=/out,readonly=false" % output]:
        raise GateError("DOCKER_ARGV", "output bind contract drift")
    return argv


def canonical_argv_sha256(argv: Sequence[str]) -> str:
    payload = json.dumps(list(argv), separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return sha256_bytes(payload)


def _default_command_runner(argv: Sequence[str]) -> subprocess.CompletedProcess:
    return subprocess.run(list(argv), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False, check=False)


def _decode(result: subprocess.CompletedProcess) -> bytes:
    value = result.stdout
    if isinstance(value, str):
        return value.encode("utf-8")
    return bytes(value or b"")


def _inspect_state(value: Mapping[str, Any]) -> Mapping[str, Any]:
    state = value.get("State")
    if not isinstance(state, Mapping):
        raise GateError("CONTAINER_INSPECT", "State missing")
    if state.get("Status") != "exited" or state.get("ExitCode") != 0:
        raise GateError("CONTAINER_EXIT", "probe did not naturally exit successfully")
    if state.get("OOMKilled") is not False:
        raise GateError("CONTAINER_OOM", "probe OOM state is not false")
    return state


def validate_persisted_output(output: Path) -> Mapping[str, Any]:
    """Validate exact bytes written before container exit."""
    expected = {
        "persistence.marker": MARKER_BYTES,
        "feeder.stderr": STDERR_BYTES,
        "feeder_receipt.json": RECEIPT_BYTES,
    }
    result: Dict[str, Any] = {}
    for name, payload in expected.items():
        path = output / name
        observed = path.read_bytes() if not path.is_symlink() and path.is_file() else None
        if observed != payload:
            raise GateError("PERSISTENCE_MISMATCH", "exact bytes mismatch: %s" % path)
        result[name] = {"path": str(path), "bytes": len(payload), "sha256": sha256_bytes(payload)}
    document = json.loads(RECEIPT_BYTES.decode("utf-8"))
    if document != {"contract": CONTRACT, "formal_replay_started": False, "status": "PASS"}:
        raise GateError("PERSISTENCE_RECEIPT", "probe receipt contract drift")
    return result


def run_gate(root: Path, repo_root: Path = ROOT,
             command_runner: Optional[CommandRunner] = None) -> Mapping[str, Any]:
    """Execute exactly one detached Docker probe and seal a PASS receipt."""
    runner = command_runner or _default_command_runner
    root = reserve_root(Path(root))
    lineage = verify_candidate_lineage(repo_root)
    profile_path = repo_root / PROFILE_PATH.relative_to(ROOT)
    output = root / OUTPUT_DIR_NAME
    argv = build_persistence_argv(root, repo_root)
    argv_bytes = json.dumps(argv, indent=2, ensure_ascii=True).encode("utf-8") + b"\n"
    argv_sha = _atomic_create(root / "docker_argv.json", argv_bytes)
    image_check = runner(("docker", "image", "inspect", IMAGE_ID, "--format", "{{json .}}"))
    if image_check.returncode != 0:
        raise GateError("IMAGE_INSPECT", _decode(image_check).decode("utf-8", "replace"))
    try:
        image_document = json.loads(_decode(image_check).decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise GateError("IMAGE_INSPECT", "invalid image inspect JSON") from error
    if isinstance(image_document, list):
        image_document = image_document[0] if image_document else {}
    if not isinstance(image_document, Mapping) or image_document.get("Id") != IMAGE_ID:
        raise GateError("IMAGE_IDENTITY", "image ID mismatch")
    tags = image_document.get("RepoTags") or []
    if IMAGE_TAG not in tags:
        raise GateError("IMAGE_IDENTITY", "image tag missing")

    start = runner(argv)
    if start.returncode != 0:
        raise GateError("CONTAINER_START", _decode(start).decode("utf-8", "replace"))
    container = _decode(start).decode("ascii", "strict").strip().splitlines()[-1]
    if not container:
        raise GateError("CONTAINER_START", "Docker returned no container ID")
    wait = runner(("docker", "wait", container_name(root)))
    if wait.returncode != 0 or _decode(wait).decode("ascii", "replace").strip() != "0":
        raise GateError("CONTAINER_WAIT", _decode(wait).decode("utf-8", "replace"))
    inspect = runner(("docker", "inspect", container_name(root), "--format", "{{json .}}"))
    if inspect.returncode != 0:
        raise GateError("CONTAINER_INSPECT", _decode(inspect).decode("utf-8", "replace"))
    inspect_bytes = _decode(inspect)
    _atomic_create(root / "container.inspect.json", inspect_bytes)
    try:
        inspect_value = json.loads(inspect_bytes.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise GateError("CONTAINER_INSPECT", "invalid inspect JSON") from error
    if isinstance(inspect_value, list):
        inspect_value = inspect_value[0] if inspect_value else {}
    if not isinstance(inspect_value, Mapping):
        raise GateError("CONTAINER_INSPECT", "inspect document is not an object")
    state = _inspect_state(inspect_value)
    mounts = inspect_value.get("Mounts") or []
    output_mounts = [item for item in mounts if isinstance(item, Mapping) and item.get("Destination") == OUTPUT_DESTINATION]
    wrapper_mounts = [item for item in mounts if isinstance(item, Mapping) and item.get("Destination") == WRAPPER_DESTINATION]
    if len(output_mounts) != 1 or output_mounts[0].get("RW") is not True or len(wrapper_mounts) != 1 or wrapper_mounts[0].get("RW") is not False:
        raise GateError("MOUNT_INSPECT", "runtime mount contract drift")
    if inspect_value.get("HostConfig", {}).get("NetworkMode") != "none":
        raise GateError("NETWORK_INSPECT", "network mode drift")
    logs = runner(("docker", "logs", container_name(root)))
    _atomic_create(root / "container.logs.txt", _decode(logs))
    persisted = validate_persisted_output(output)
    remove = runner(("docker", "rm", container_name(root)))
    if remove.returncode != 0:
        raise GateError("CLEANUP", "stopped-only docker rm failed")

    receipt = {
        "schema_version": 1,
        "kind": CONTRACT,
        "status": "PASS",
        "profile": {"path": str(profile_path), "sha256": PROFILE_SHA256},
        "lineage": lineage,
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "container": {
            "name": container_name(root), "id": container, "start_count": 1,
            "exit_code": state.get("ExitCode"), "oom_killed": state.get("OOMKilled"),
            "natural_exit": True, "manual_stop": False,
        },
        "command": {"argv": argv, "canonical_sha256": canonical_argv_sha256(argv), "argv_file_sha256": argv_sha},
        "mount_contract": {
            "network": "none", "rootfs": "read_only", "input_mounts": 0,
            "ground_truth_mount": False, "scorer_mount": False, "map_mount": False,
            "output_destination": OUTPUT_DESTINATION, "output_readonly": False,
            "output_tmpfs": False, "duplicate_output_destinations": False,
            "wrapper_destination": WRAPPER_DESTINATION, "wrapper_readonly": True,
        },
        "persisted_output": persisted,
        "cleanup": {"diagnostics_captured_before_remove": True, "removed": True, "stopped_only": True},
        "safety": {
            "formal_replay_authorized": False, "formal_replay_started": False,
            "input_opened": False, "ground_truth_content_opened": False,
            "scorer_invoked": False, "map_saved": False,
        },
        "feeder_root_cause": "unknown",
    }
    receipt_bytes = json.dumps(receipt, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    receipt_path = root / "persistence_gate.receipt.json"
    receipt_sha = _atomic_create(receipt_path, receipt_bytes)
    sidecar = receipt_path.with_suffix(receipt_path.suffix + ".sha256")
    sidecar_sha = _atomic_create(sidecar, (receipt_sha + "  " + receipt_path.name + "\n").encode("ascii"))
    return {"root": str(root), "receipt_path": str(receipt_path), "receipt_sha256": receipt_sha, "sidecar_sha256": sidecar_sha}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    try:
        result = run_gate(args.root, args.repo_root)
    except GateError as error:
        print("FAIL_CLOSED %s: %s" % (error.kind, error), file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
