#!/usr/bin/env python3
"""Run the additive v15 schema-3 no-input identity gate exactly once.

This is deliberately a small host-side identity gate.  It does not start the
mapper, open a dataset, or bind a host directory into the container.  The
container only imports the installed v15 feeder and validates one pinned
schema-3 pass document plus one deliberately-invalid document.  All ROS
environment values needed by the image entrypoint are explicit Docker
``--env`` arguments; no host ROS environment is inherited.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]

IMAGE_TAG = (
    "m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-"
    "fast-livo2-benchmark:ros1-pinned"
)
IMAGE_ID = (
    "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
)
BASE_IMAGE_TAG = (
    "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-"
    "fast-livo2-benchmark:ros1-pinned"
)
BASE_IMAGE_ID = (
    "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
)
FEEDER_PATH = Path("scripts/fast_livo2_m6a10_v15_feeder.py")
FEEDER_SHA256 = (
    "6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
)
PROFILE_PATH = Path(
    "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml"
)
PROFILE_SHA256 = (
    "070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f"
)
WRAPPER_PATH = Path("scripts/fast_livo2_m6a10_v15_formal_container_run.sh")
WRAPPER_SHA256 = (
    "d3ab90871422100aa2d89e2469ac413c69346300e2766cc2cc88096e9b45e8f2"
)
FIXTURE_PATH = Path(
    "graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json"
)
FIXTURE_SHA256 = (
    "a30a5d50ea9a02fe9d7e0edc230801b33ee57676a06ac5e3a9fe30d0eede4a0a"
)
V12_PATCH_PATH = Path(
    "docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch"
)
V12_PATCH_SHA256 = (
    "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
)

PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
ROS_ENV = {
    "ROS_MASTER_URI": "http://127.0.0.1:11311",
    "ROS_IP": "127.0.0.1",
    "ROS_HOSTNAME": "127.0.0.1",
    "ROS_HOME": "/tmp/ros_home",
    "ROS_LOG_DIR": "/tmp/ros_logs",
}
TMPFS = (
    "/tmp:rw,nosuid,nodev",
    "/root/.ros:rw,nosuid,nodev",
    "/out:rw,nosuid,nodev",
)
CONTAINER_CONTRACT = "m6a10-v15-schema3-feeder-no-input-identity-v1"
SELFTEST_MARKER = "V15_SCHEMA3_FEEDER_CORRECTION_PASS"
FORBIDDEN_ARGV_TOKENS = ("rosbag", "ground_truth", "scorer", "map", "input")
EXPECTED_LABELS = {
    "benchmark.fast_livo2.m6a10_v12_base_image": BASE_IMAGE_TAG,
    "benchmark.fast_livo2.m6a10_v12_base_id": BASE_IMAGE_ID,
    "benchmark.fast_livo2.m6a10_v12_patch_sha256": V12_PATCH_SHA256,
    "benchmark.fast_livo2.m6a10_v15_feeder_sha256": FEEDER_SHA256,
    "benchmark.fast_livo2.m6a10_v15_profile_sha256": PROFILE_SHA256,
    "benchmark.fast_livo2.m6a10_v15_wrapper_sha256": WRAPPER_SHA256,
    "benchmark.fast_livo2.m6a10_v15_fixture_sha256": FIXTURE_SHA256,
    "benchmark.fast_livo2.m6a10_phase_contract": PHASE_CONTRACT,
    "benchmark.fast_livo2.m6a10_transport_contract": TRANSPORT_CONTRACT,
    "benchmark.fast_livo2.m6a10_network_expectation": "none",
    "benchmark.fast_livo2.m6a10_rootfs_expectation": "read_only_runtime",
    "benchmark.fast_livo2.m6a10_input_present": "false",
    "benchmark.fast_livo2.m6a10_ground_truth_present": "false",
    "benchmark.fast_livo2.m6a10_scorer_present": "false",
    "benchmark.fast_livo2.m6a10_map_present": "false",
    "benchmark.fast_livo2.m6a10_formal_replay_forbidden": "true",
}


class GateError(RuntimeError):
    """A fail-closed identity-gate error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise GateError("SOURCE_MISSING", "not a regular source file: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_ros_env(value: Mapping[str, str]) -> Dict[str, str]:
    """Return the exact explicit ROS env contract; reject drift or omissions."""
    if set(value) != set(ROS_ENV):
        raise GateError("ROS_ENV_CONTRACT", "ROS env keys must be exact")
    normalized = {str(key): str(val) for key, val in value.items()}
    if normalized != ROS_ENV:
        raise GateError("ROS_ENV_CONTRACT", "ROS env values must be pinned")
    return normalized


def _selftest_python() -> str:
    # This code is passed as one argv element to bash -lc after the image's
    # /ros_entrypoint.sh has sourced ROS.  It has no host paths or inputs.
    return "\n".join((
        "import copy, hashlib, json",
        "from pathlib import Path",
        "import fast_livo2_m6a10_v15_feeder as feeder",
        "fixture = Path('/opt/fast_livo_v15/m6a10_v15_consumer_status_pass.json')",
        "value = json.loads(fixture.read_text(encoding='utf-8'))",
        "assert feeder.validate_consumer_status(value)['schema_version'] == 3",
        "bad = copy.deepcopy(value)",
        "bad['transport_contract_version'] = 'wrong-transport-contract'",
        "rejected = False",
        "try:",
        "    feeder.validate_consumer_status(bad)",
        "except feeder.ConsumerStatusError:",
        "    rejected = True",
        "assert rejected",
        "installed = Path('/runner/scripts/fast_livo2_m6a10_feeder.py')",
        "assert not installed.is_symlink() and installed.is_file()",
        "assert hashlib.sha256(installed.read_bytes()).hexdigest() == "
        + repr(FEEDER_SHA256),
        "print(" + repr(SELFTEST_MARKER) + ")",
    ))


def selftest_command() -> str:
    return "PYTHONPATH=/runner/scripts python3 -c %s" % shlex.quote(
        _selftest_python()
    )


def build_docker_argv(container_name: str) -> List[str]:
    """Build the fixed no-input argv; callers cannot inject env or mounts."""
    if not container_name or "/" in container_name or " " in container_name:
        raise GateError("CONTAINER_NAME_INVALID", "invalid Docker container name")
    env = require_ros_env(ROS_ENV)
    argv = ["docker", "run", "--name", container_name, "--network", "none",
            "--read-only"]
    for tmpfs in TMPFS:
        argv.extend(("--tmpfs", tmpfs))
    for key in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME", "ROS_HOME", "ROS_LOG_DIR"):
        argv.extend(("--env", "%s=%s" % (key, env[key])))
    argv.extend((IMAGE_ID, "/bin/bash", "-lc", selftest_command()))
    flattened = " ".join(argv).lower()
    if any(token in flattened for token in FORBIDDEN_ARGV_TOKENS):
        raise GateError("ARGV_SCOPE", "forbidden input/evaluation token in argv")
    if "--mount" in argv or "-v" in argv or "--volume" in argv:
        raise GateError("ARGV_MOUNT", "host mount syntax is forbidden")
    if "--rm" in argv or IMAGE_TAG in argv:
        raise GateError("ARGV_IDENTITY", "tag/automatic removal is forbidden")
    return argv


def _fixed_subprocess_env() -> dict[str, str]:
    # Docker gets only explicit --env values above.  This minimal environment
    # is for locating the host docker executable and is never inherited into
    # the container.
    return {
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "LANG": "C.UTF-8",
    }


def _run(argv: Sequence[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(list(argv), env=_fixed_subprocess_env(), **kwargs)


def _docker_json(argv: Sequence[str]) -> Any:
    result = _run(argv, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise GateError("DOCKER_INSPECT", result.stderr.strip() or result.stdout.strip())
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise GateError("DOCKER_INSPECT", "Docker inspect was not JSON") from error


def reserve_root(root: Path) -> Path:
    root = Path(root)
    if root.exists() or root.is_symlink():
        raise GateError("ROOT_REUSE", "fresh root required: %s" % root)
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise GateError("ROOT_PARENT", "root parent must be an existing directory")
    try:
        root.mkdir(mode=0o755)
    except FileExistsError as error:
        raise GateError("ROOT_REUSE", "root appeared during reservation") from error
    marker = root / ".reserved-v15-no-input-identity"
    fd = os.open(str(marker), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="ascii") as stream:
            stream.write(CONTAINER_CONTRACT + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            marker.unlink()
        except FileNotFoundError:
            pass
        raise
    marker.chmod(0o444)
    return root


def _container_name(root: Path) -> str:
    suffix = hashlib.sha256(str(root.resolve(strict=False)).encode("utf-8")).hexdigest()[:20]
    # Keep the Docker argv free of dataset/evaluation vocabulary.  The
    # no-input scope is represented in the receipt, not in the container name.
    return "m6a10-v15-id-%s" % suffix


def _source_bindings() -> Dict[str, str]:
    expected = {
        "feeder": (FEEDER_PATH, FEEDER_SHA256),
        "profile": (PROFILE_PATH, PROFILE_SHA256),
        "wrapper": (WRAPPER_PATH, WRAPPER_SHA256),
        "fixture": (FIXTURE_PATH, FIXTURE_SHA256),
        "v12_patch": (V12_PATCH_PATH, V12_PATCH_SHA256),
    }
    bound: Dict[str, str] = {}
    for name, (relative, pinned) in expected.items():
        actual = sha256_file(ROOT / relative)
        if actual != pinned:
            raise GateError("SOURCE_DRIFT", "%s hash drift" % relative)
        bound[name] = actual
    return bound


def _labels_from_image(document: Mapping[str, Any]) -> Dict[str, str]:
    image_id = document.get("Id")
    if image_id != IMAGE_ID:
        raise GateError("IMAGE_ID_DRIFT", "image ID does not match pinned image")
    config = document.get("Config")
    if not isinstance(config, Mapping):
        raise GateError("IMAGE_LABELS", "image Config missing")
    labels = config.get("Labels")
    if not isinstance(labels, Mapping):
        raise GateError("IMAGE_LABELS", "image labels missing")
    normalized = {str(key): str(value) for key, value in labels.items()}
    for key, expected in EXPECTED_LABELS.items():
        if normalized.get(key) != expected:
            raise GateError("IMAGE_LABELS", "label mismatch: %s" % key)
    return normalized


def inspect_image() -> Dict[str, Any]:
    document = _docker_json(("docker", "image", "inspect", IMAGE_TAG))
    if not isinstance(document, list) or len(document) != 1:
        raise GateError("IMAGE_INSPECT", "expected exactly one image document")
    image = document[0]
    if not isinstance(image, Mapping):
        raise GateError("IMAGE_INSPECT", "image document must be an object")
    labels = _labels_from_image(image)
    return {"id": image["Id"], "labels": labels}


def _safe_json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True) + "\n").encode("utf-8")


def _seal_bytes(target: Path, raw: bytes) -> str:
    if target.exists() or target.is_symlink():
        raise GateError("RECEIPT_OVERWRITE", "receipt already exists: %s" % target)
    part = target.with_name(".%s.%s.part" % (target.name, os.getpid()))
    if part.exists() or part.is_symlink():
        raise GateError("RECEIPT_OVERWRITE", "receipt staging path exists")
    fd = os.open(str(part), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(str(part), str(target))
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    target.chmod(0o444)
    return hashlib.sha256(raw).hexdigest()


def _seal_receipt(root: Path, value: Mapping[str, Any]) -> Tuple[Path, Path, str]:
    receipt = root / "no_input_correction.receipt.json"
    sidecar = root / "no_input_correction.receipt.json.sha256"
    raw = _safe_json_bytes(value)
    digest = _seal_bytes(receipt, raw)
    side_raw = ("%s  %s\n" % (digest, receipt.name)).encode("ascii")
    _seal_bytes(sidecar, side_raw)
    return receipt, sidecar, digest


def _inspect_container(name: str) -> Dict[str, Any]:
    result = _run(("docker", "inspect", name), capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise GateError("CONTAINER_INSPECT", result.stderr.strip() or result.stdout.strip())
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise GateError("CONTAINER_INSPECT", "container inspect was not JSON") from error
    if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], Mapping):
        raise GateError("CONTAINER_INSPECT", "expected one container document")
    return dict(value[0])


def _validate_container(document: Mapping[str, Any], returncode: int,
                        log_text: str) -> Dict[str, Any]:
    state = document.get("State")
    host = document.get("HostConfig")
    if not isinstance(state, Mapping) or not isinstance(host, Mapping):
        raise GateError("CONTAINER_SHAPE", "container state/host config missing")
    if returncode != 0 or state.get("Status") != "exited" or state.get("ExitCode") != 0:
        raise GateError("SELFTEST_FAILED", "identity selftest did not exit successfully")
    if state.get("OOMKilled") is not False:
        raise GateError("OOM", "OOMKilled was not false")
    if host.get("NetworkMode") != "none" or host.get("ReadonlyRootfs") is not True:
        raise GateError("ISOLATION", "network/rootfs contract mismatch")
    binds = host.get("Binds")
    if binds not in (None, []):
        raise GateError("HOST_MOUNT", "host bind mount detected")
    tmpfs = host.get("Tmpfs")
    if not isinstance(tmpfs, Mapping) or set(tmpfs) != {"/tmp", "/root/.ros", "/out"}:
        raise GateError("TMPFS", "tmpfs destinations mismatch")
    mounts = document.get("Mounts")
    if not isinstance(mounts, list):
        raise GateError("MOUNTS", "container mounts missing")
    # Docker reports tmpfs in HostConfig.Tmpfs and commonly leaves the
    # top-level Mounts list empty.  If Mounts is populated, it must still be
    # an exact tmpfs-only view; either representation must prove the same
    # three destinations and zero host binds.
    if mounts:
        if any(not isinstance(mount, Mapping) or mount.get("Type") != "tmpfs"
               for mount in mounts):
            raise GateError("HOST_MOUNT", "non-tmpfs mount detected")
        mounted_destinations = {mount.get("Destination") for mount in mounts}
        if mounted_destinations != {"/tmp", "/root/.ros", "/out"}:
            raise GateError("TMPFS", "mounted tmpfs destinations mismatch")
    else:
        mounted_destinations = set(tmpfs)
    if SELFTEST_MARKER not in log_text:
        raise GateError("SELFTEST_MARKER", "schema-3 selftest marker missing")
    return {
        "status": state.get("Status"),
        "exit_code": int(state.get("ExitCode")),
        "oom_killed": bool(state.get("OOMKilled")),
        "network_mode": host.get("NetworkMode"),
        "readonly_rootfs": bool(host.get("ReadonlyRootfs")),
        "host_bind_mounts": 0,
        "tmpfs_destinations": sorted(mounted_destinations),
    }


def _remove_stopped(name: str) -> bool:
    result = _run(("docker", "rm", name), capture_output=True, text=True, check=False)
    return result.returncode == 0


def _utc_now() -> str:
    return _datetime.datetime.now(_datetime.timezone.utc).isoformat()


def run_gate(root: Path) -> Tuple[Path, Path, str, str]:
    root = reserve_root(root)
    source_hashes = _source_bindings()
    image = inspect_image()
    name = _container_name(root)
    existing = _run(("docker", "inspect", name), capture_output=True, text=True, check=False)
    if existing.returncode == 0:
        raise GateError("CONTAINER_REUSE", "container name already exists")
    argv = build_docker_argv(name)
    log_path = root / "container_identity.log"
    inspect_path = root / "container.inspect.json"
    start_time = _utc_now()
    with log_path.open("x", encoding="utf-8") as log:
        result = _run(argv, stdout=log, stderr=subprocess.STDOUT, text=True, check=False)
    log_hash = sha256_file(log_path)
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    inspect_doc = _inspect_container(name)
    inspect_raw = _safe_json_bytes(inspect_doc)
    inspect_path.write_bytes(inspect_raw)
    inspect_path.chmod(0o444)
    validation_error = None
    try:
        validation = _validate_container(inspect_doc, result.returncode, log_text)
    except GateError as error:
        validation_error = error
        validation = {}
    removed = _remove_stopped(name) if inspect_doc.get("State", {}).get("Status") == "exited" else False
    if validation_error is not None:
        raise validation_error
    if not removed:
        raise GateError("CLEANUP", "stopped container removal failed")
    runner_path = Path(__file__).resolve()
    runner_hash = sha256_file(runner_path)
    command_hash = hashlib.sha256("\0".join(argv).encode("utf-8")).hexdigest()
    receipt_value = {
        "schema_version": 1,
        "receipt_kind": "v15_no_input_identity_correction",
        "status": "PASS",
        "created_at_utc": start_time,
        "image": {"tag": IMAGE_TAG, "id": image["id"], "labels": image["labels"]},
        "source": {
            "feeder_path": str(FEEDER_PATH), "feeder_sha256": source_hashes["feeder"],
            "profile_path": str(PROFILE_PATH), "profile_sha256": source_hashes["profile"],
            "wrapper_path": str(WRAPPER_PATH), "wrapper_sha256": source_hashes["wrapper"],
            "fixture_path": str(FIXTURE_PATH), "fixture_sha256": source_hashes["fixture"],
            "v12_patch_path": str(V12_PATCH_PATH), "v12_patch_sha256": source_hashes["v12_patch"],
            "runner_path": str(runner_path), "runner_sha256_observed": runner_hash,
        },
        "contracts": {"phase": PHASE_CONTRACT, "transport": TRANSPORT_CONTRACT},
        "ros_environment": dict(ROS_ENV),
        "docker": {
            "argv": argv, "argv_sha256": command_hash,
            "host_environment_inherited": False, "container_name": name,
            "start_count": 1, "container_count": 1,
        },
        "validation": {
            "schema3_valid_document": True,
            "schema3_invalid_document_rejected": True,
            "installed_feeder_sha256": FEEDER_SHA256,
            "selftest_marker": SELFTEST_MARKER,
        },
        "execution": {
            **validation, "natural_exit": True,
            "diagnostics_captured_before_cleanup": True,
            "cleanup_stopped_only": True, "remove_success": True,
        },
        "artifacts": {
            "container_log_path": str(log_path), "container_log_sha256": log_hash,
            "container_inspect_path": str(inspect_path),
            "container_inspect_sha256": sha256_file(inspect_path),
        },
        "safety": {
            "host_mounts": 0, "input_mounts": 0, "input_opened": False,
            "ground_truth_content_opened": False, "scorer_invoked": False,
            "map_saved": False, "formal_replay_started": False,
        },
    }
    return (*_seal_receipt(root, receipt_value), receipt_value["status"])


def _failure_receipt(root: Path, error: GateError) -> Optional[Tuple[Path, Path, str]]:
    if not root.is_dir() or root.is_symlink():
        return None
    receipt = root / "no_input_correction.receipt.json"
    if receipt.exists() or receipt.is_symlink():
        return None
    value = {
        "schema_version": 1, "receipt_kind": "v15_no_input_identity_correction",
        "status": "FAIL_CLOSED", "failure_kind": error.kind,
        "failure_message": str(error), "formal_replay_started": False,
        "safety": {"host_mounts": 0, "input_mounts": 0,
                    "input_opened": False, "ground_truth_content_opened": False,
                    "scorer_invoked": False, "map_saved": False},
    }
    return _seal_receipt(root, value)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args(argv)
    root = args.root
    try:
        receipt, sidecar, digest, _status = run_gate(root)
    except GateError as error:
        try:
            sealed = _failure_receipt(root, error)
            if sealed is not None:
                print("FAIL_CLOSED %s %s" % (sealed[0], sealed[2]), file=sys.stderr)
        except Exception as seal_error:  # pragma: no cover - last-resort CLI path
            print("FAIL_CLOSED receipt sealing failed: %s" % seal_error, file=sys.stderr)
        print("FAIL_CLOSED[%s] %s" % (error.kind, error), file=sys.stderr)
        return 1
    print("PASS %s %s %s" % (receipt, sidecar, digest))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
