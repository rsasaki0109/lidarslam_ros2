#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Run the fixed, host-only v12 evidence test gate once.

This runner validates the sealed no-input attempt-6 evidence, hashes the
current gate sources, and invokes one explicit pytest argv without a shell.
It does not start Docker/ROS or open any benchmark input.  A fresh output root
and immutable receipt are required even when the test command fails.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]

ATTEMPT6_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_dual_service_attempt6_20260823T175000Z_agentv12"
)
ATTEMPT6_RECEIPT = ATTEMPT6_ROOT / "no_input_dual_evidence.receipt.json"
ATTEMPT6_RECEIPT_SHA256 = (
    "c5afa2452f96588bf46dc7f0d0b4504e816d0069bbd3a107009eb2c3beac0b45"
)
ATTEMPT6_RECEIPT_SIDECAR_SHA256 = (
    "076cd96e10c78d0bf73a6a7ed66d3549aaa2dd0f75c7503d5d88e19237d608e3"
)
ATTEMPT6_CALLBACK_SHA256 = (
    "7b61b2299c3f9c79ddae33e9ae1bbf14855805b89e46a7e56d99ffba8cdfa84c"
)
ATTEMPT6_TERMINAL_SHA256 = (
    "f843ce9bed5b16e4d44d6b7b7f9da945282ef0d3801ae94aa525717669b128b4"
)
ATTEMPT6_JOURNAL_SHA256 = (
    "f6608931454694d39cbf0fa5fc6f9af6db791d5ca2d8ffa9fed02608e026c26a"
)
ATTEMPT6_OBSERVED_RUNNER_SHA256 = (
    "f3c41d6533475229a765fe92682d483758347053c6f146dd148fc85a6274648a"
)

IMAGE_TAG = (
    "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-"
    "fast-livo2-benchmark:ros1-pinned"
)
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
PHASE_MODE = "unpaced_ack"
EXPECTED_COUNTS = {"lidar": 1, "imu": 0, "image": 0}
EXPECTED_SERVICES = [
    "/m6a10/consumer_status", "/m6a10/consumer_ack",
    "/m6a10/consumer_eof", "/m6a10/consumer_finalize",
    "/m6a10/terminal_status", "/m6a10/terminal_eof",
    "/m6a10/terminal_finalize",
]
CONTRACT_VERSION = "m6a10-v12-host-evidence-gate-v1"
RECEIPT_NAME = "host_evidence_gate.receipt.json"

# These are deliberately explicit.  No directory-wide pytest discovery is
# allowed to change the evidence surface after this gate is reviewed.
FIXED_TESTS = (
    "graph_based_slam/test/test_bind_fast_livo2_v12_consumer_evidence.py",
    "graph_based_slam/test/test_compose_fast_livo2_v12_terminal_evidence.py",
    "graph_based_slam/test/test_fast_livo2_m6a10_v12_no_input_container_payload.py",
    "graph_based_slam/test/test_fast_livo2_m6a10_v12_no_input_gate.py",
    "graph_based_slam/test/test_monitor_m6a10_host_interference.py",
    "graph_based_slam/test/test_fast_livo2_m6a10_v12_formal_container_run.py",
    "graph_based_slam/test/test_benchmark_phase_contract.py",
    "graph_based_slam/test/test_fast_livo2_m6a10_v12_unit_gate.py",
    "graph_based_slam/test/test_run_fast_livo2_m6a10_v12_host_evidence_gate.py",
)

SOURCE_PATHS = {
    "binder": "scripts/bind_fast_livo2_v12_consumer_evidence.py",
    "compositor": "scripts/compose_fast_livo2_v12_terminal_evidence.py",
    "monitor": "scripts/monitor_m6a10_host_interference.py",
    "no_input_runner": "scripts/run_fast_livo2_m6a10_v12_no_input_gate.py",
    "no_input_runner_test": "graph_based_slam/test/test_fast_livo2_m6a10_v12_no_input_gate.py",
    "formal_wrapper": "scripts/fast_livo2_m6a10_v12_formal_container_run.sh",
    "formal_wrapper_test": "graph_based_slam/test/test_fast_livo2_m6a10_v12_formal_container_run.py",
    "host_gate_runner": "scripts/run_fast_livo2_m6a10_v12_host_evidence_gate.py",
}

# Test paths necessarily contain ``no_input``; these are command-injection or
# forbidden evaluation surfaces, not ordinary words in a test filename.
FORBIDDEN_ARGV_SUBSTRINGS = (
    "docker", "rosbag", "ground_truth", "scorer", "map_save",
    "shell=", "shell_true", "&&", "||", ";", "`", "$(", ">", "<",
)


class HostEvidenceGateError(ValueError):
    """Fail-closed host evidence gate error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _error(kind: str, message: str) -> HostEvidenceGateError:
    return HostEvidenceGateError(kind, message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _reject_symlink_components(path: Path, label: str) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise _error("SYMLINK_REJECTED", f"{label} contains symlink {current}")


def _regular(path: Path, label: str) -> None:
    _reject_symlink_components(path, label)
    if os.path.lexists(path) is False or path.is_symlink() or not path.is_file():
        raise _error("NOT_REGULAR", f"{label} is not a regular file")
    if path.name.endswith(".part"):
        raise _error("STAGING_FILE", f"{label} is a staging file")


def _json(path: Path, label: str) -> tuple[dict[str, Any], str]:
    _regular(path, label)
    try:
        raw = path.read_bytes()
        value = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error("JSON_INVALID", f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise _error("JSON_OBJECT_REQUIRED", f"{label} must be an object")
    return value, hashlib.sha256(raw).hexdigest()


def _verify_sidecar(path: Path, expected_file_sha256: str, expected_sidecar_sha256: str) -> None:
    _regular(path, "attempt-6 receipt sidecar")
    if sha256_file(path) != expected_sidecar_sha256:
        raise _error("ATTEMPT6_SIDECAR_SHA", "attempt-6 receipt sidecar SHA drift")
    expected_text = f"{expected_file_sha256}  {ATTEMPT6_RECEIPT.name}\n"
    if path.read_text(encoding="ascii") != expected_text:
        raise _error("ATTEMPT6_SIDECAR_CONTENT", "attempt-6 receipt sidecar content drift")


def _exact_counts(value: Any, label: str) -> None:
    if value != EXPECTED_COUNTS:
        raise _error("COUNTS_DRIFT", f"{label} differs from exact synthetic counts")


def _zero_counts(value: Any, label: str) -> None:
    if value != {"lidar": 0, "imu": 0, "image": 0}:
        raise _error("COUNTS_DRIFT", f"{label} is not zero")


def _false_safety(value: Mapping[str, Any], label: str) -> None:
    required = {
        "formal_replay_started", "ground_truth_content_opened", "host_mounts_exposed",
        "input_opened", "map_saved", "scorer_invoked",
    }
    if not isinstance(value, Mapping) or any(value.get(key) is not False for key in required):
        raise _error("SAFETY_FLAGS", f"{label} safety flags are not all false")


def _verify_callback(path: Path) -> dict[str, Any]:
    value, observed_sha = _json(path, "attempt-6 callback")
    if observed_sha != ATTEMPT6_CALLBACK_SHA256:
        raise _error("CALLBACK_SHA", "attempt-6 callback SHA drift")
    if value.get("schema_version") != 3 or value.get("contract_version") != PHASE_CONTRACT or \
            value.get("transport_contract_version") != TRANSPORT_CONTRACT or \
            value.get("phase_mode") != PHASE_MODE or value.get("system") != "fast_livo2" or \
            value.get("status") != "pass":
        raise _error("CALLBACK_CONTRACT", "attempt-6 callback schema/contract/status drift")
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise _error("CALLBACK_SAFETY", "attempt-6 callback safety drift")
    consumer = value.get("consumer")
    if not isinstance(consumer, dict):
        raise _error("CALLBACK_LEDGER", "attempt-6 callback consumer ledger missing")
    _exact_counts(consumer.get("received_topic_counts"), "callback.received_topic_counts")
    for key in ("received_messages", "acked_messages"):
        if consumer.get(key) != 1:
            raise _error("CALLBACK_COUNTS", f"callback.{key} drift")
    if consumer.get("ack_exact") is not True or consumer.get("transport_outstanding_at_drain") != 0 or \
            consumer.get("maximum_transport_outstanding_messages") != 1 or \
            consumer.get("maximum_allowed_transport_outstanding_messages") != 1:
        raise _error("CALLBACK_TRANSPORT", "attempt-6 callback transport proof drift")
    return {
        "path": str(path),
        "sha256": observed_sha,
        "schema_version": value["schema_version"],
        "contract_version": value["contract_version"],
        "transport_contract_version": value["transport_contract_version"],
        "status": value["status"],
    }


def _verify_terminal(path: Path) -> dict[str, Any]:
    value, observed_sha = _json(path, "attempt-6 terminal")
    if observed_sha != ATTEMPT6_TERMINAL_SHA256:
        raise _error("TERMINAL_SHA", "attempt-6 terminal SHA drift")
    if value.get("schema_version") != 1 or value.get("contract_id") != TERMINAL_CONTRACT or \
            value.get("phase_mode") != PHASE_MODE or value.get("system") != "fast_livo2" or \
            value.get("status") != "invalid":
        raise _error("TERMINAL_CONTRACT", "attempt-6 terminal schema/contract/status drift")
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise _error("TERMINAL_SAFETY", "attempt-6 terminal safety drift")
    _exact_counts(value.get("received_topic_counts"), "terminal.received_topic_counts")
    _zero_counts(value.get("completed_counts"), "terminal.completed_counts")
    _zero_counts(value.get("dropped_counts"), "terminal.dropped_counts")
    _zero_counts(value.get("overflow_counts"), "terminal.overflow_counts")
    if value.get("processing_failures") != 0:
        raise _error("TERMINAL_FAILURES", "attempt-6 terminal processing failure drift")
    backend = value.get("backend")
    if not isinstance(backend, dict) or backend.get("completed_counts") != {"lidar": None, "imu": None, "image": None} or \
            backend.get("completed_synchronization_units") is not None or \
            backend.get("quiescent") is not False or backend.get("in_flight") != {"active": True}:
        raise _error("TERMINAL_BOUNDARY", "attempt-6 no-boundary backend shape drift")
    buffers = value.get("buffers")
    if not isinstance(buffers, dict) or buffers.get("lidar", {}).get("count") != 1 or \
            buffers.get("imu", {}).get("count") != 0 or buffers.get("image", {}).get("count") != 0:
        raise _error("TERMINAL_BUFFERS", "attempt-6 terminal buffer counts drift")
    lidar_records = buffers.get("lidar", {}).get("records")
    if not isinstance(lidar_records, list) or len(lidar_records) != 1:
        raise _error("TERMINAL_RECORDS", "attempt-6 lidar residual record missing")
    record = lidar_records[0]
    if record.get("record_id") != "lidar-0" or record.get("timestamp_seconds") != 10 or \
            record.get("reason_code") != "residual_lidar_rejected" or \
            record.get("support_context_proven") is not False or \
            record.get("can_form_synchronization_unit") is not False:
        raise _error("TERMINAL_RECORD", "attempt-6 residual lidar record drift")
    support = value.get("terminal_support_context")
    if not isinstance(support, dict) or support.get("total_count") != 0 or \
            support.get("by_topic") != {"lidar": 0, "imu": 0, "image": 0}:
        raise _error("TERMINAL_SUPPORT", "attempt-6 terminal support counts drift")
    observation = value.get("terminal_observation")
    if not isinstance(observation, dict) or observation.get("eof_observed") is not True or \
            observation.get("stable") is not True or observation.get("poll_count") != 2 or \
            observation.get("stable_poll_count") != 2 or observation.get("identical_snapshot") is not True or \
            observation.get("minimum_poll_wall_seconds") != 0.05:
        raise _error("TERMINAL_POLLS", "attempt-6 terminal poll proof drift")
    trajectory = value.get("trajectory")
    if not isinstance(trajectory, dict) or trajectory.get("coverage_verified") is not True or \
            trajectory.get("end_gap_seconds") is not None:
        raise _error("TERMINAL_TRAJECTORY", "attempt-6 terminal trajectory shape drift")
    return {
        "path": str(path),
        "sha256": observed_sha,
        "schema_version": value["schema_version"],
        "contract_id": value["contract_id"],
        "status": value["status"],
    }


def _verify_journal(path: Path) -> dict[str, Any]:
    _regular(path, "attempt-6 service journal")
    observed_sha = sha256_file(path)
    if observed_sha != ATTEMPT6_JOURNAL_SHA256:
        raise _error("JOURNAL_SHA", "attempt-6 service journal SHA drift")
    try:
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error("JOURNAL_INVALID", "attempt-6 service journal is invalid") from exc
    expected_order = [
        "/m6a10/consumer_status", "/m6a10/consumer_ack", "/m6a10/consumer_ack",
        "/m6a10/consumer_eof", "/m6a10/consumer_status", "/m6a10/consumer_finalize",
        "/m6a10/terminal_eof", "/m6a10/terminal_status", "/m6a10/terminal_status",
        "/m6a10/terminal_finalize",
    ]
    if [row.get("service") for row in rows] != expected_order:
        raise _error("JOURNAL_ORDER", "attempt-6 service journal order drift")
    return {"path": str(path), "sha256": observed_sha, "records": len(rows)}


def verify_attempt6(
        root: Path = ATTEMPT6_ROOT,
        receipt_path: Path = ATTEMPT6_RECEIPT,
) -> dict[str, Any]:
    """Strictly validate the immutable attempt-6 fixture and its raw artifacts."""
    if root != ATTEMPT6_ROOT or receipt_path != ATTEMPT6_RECEIPT:
        raise _error("ATTEMPT6_BINDING", "attempt-6 path is not the pinned root")
    receipt, receipt_sha = _json(receipt_path, "attempt-6 receipt")
    if receipt_sha != ATTEMPT6_RECEIPT_SHA256:
        raise _error("ATTEMPT6_RECEIPT_SHA", "attempt-6 receipt SHA drift")
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    _verify_sidecar(sidecar, receipt_sha, ATTEMPT6_RECEIPT_SIDECAR_SHA256)
    if receipt.get("status") != "PASS" or receipt.get("attempt_index") != 6 or \
            receipt.get("contract_version") != "m6a10-v12-no-input-dual-service-gate-v1":
        raise _error("ATTEMPT6_STATUS", "attempt-6 receipt status/index/contract drift")
    if receipt.get("services") != EXPECTED_SERVICES:
        raise _error("ATTEMPT6_SERVICES", "attempt-6 service set drift")
    execution = receipt.get("execution")
    expected_execution = {
        "host_mounts": [], "input_mounts": 0, "manual_stop": False,
        "network": "none", "one_start": True, "retry_count": 0,
        "rootfs": "read_only", "start_count": 1, "started": True,
        "stop_requested": True, "tmpfs": ["/tmp", "/root/.ros", "/out"],
    }
    if execution != expected_execution:
        raise _error("ATTEMPT6_EXECUTION", "attempt-6 execution isolation/cardinality drift")
    _false_safety(receipt.get("safety", {}), "attempt-6 receipt")
    cleanup = receipt.get("cleanup")
    if not isinstance(cleanup, dict) or cleanup.get("diagnostics_before_stop") is not True or \
            cleanup.get("oom_killed") is not False or cleanup.get("remove_success") is not True or \
            cleanup.get("stopped_only") is not True or cleanup.get("stopped_state") != "exited" or \
            cleanup.get("remove", {}).get("returncode") != 0:
        raise _error("ATTEMPT6_CLEANUP", "attempt-6 cleanup/OOM safety drift")
    protocol = receipt.get("protocol")
    if not isinstance(protocol, dict) or protocol.get("services") != EXPECTED_SERVICES or \
            protocol.get("first_ack_success") is not True or protocol.get("duplicate_ack_rejected") is not True or \
            protocol.get("published_lidar_callbacks") != 1 or \
            protocol.get("status_received_counts") != EXPECTED_COUNTS or \
            protocol.get("consumer_sequence") != ["consumer_eof", "consumer_status", "consumer_finalize"] or \
            protocol.get("terminal_sequence") != ["terminal_eof", "terminal_status", "terminal_status", "terminal_finalize"] or \
            protocol.get("terminal_expected_invalid_no_completed_boundary") is not True:
        raise _error("ATTEMPT6_PROTOCOL", "attempt-6 protocol evidence drift")
    image = receipt.get("image")
    if not isinstance(image, dict) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise _error("ATTEMPT6_IMAGE", "attempt-6 image identity drift")
    source_receipt = receipt.get("sources", {})
    observed_runner = source_receipt.get("runner_runtime_observed", {})
    if observed_runner.get("sha256") != ATTEMPT6_OBSERVED_RUNNER_SHA256:
        raise _error("ATTEMPT6_RUNNER_SHA", "attempt-6 observed runner SHA drift")
    callback_path = root / "out/callback.json"
    terminal_path = root / "out/terminal.json"
    journal_path = root / "service_responses.jsonl"
    if receipt.get("callback", {}).get("path") != str(callback_path) or \
            receipt.get("terminal", {}).get("path") != str(terminal_path) or \
            receipt.get("service_journal_path") != str(journal_path):
        raise _error("ATTEMPT6_ARTIFACT_PATH", "attempt-6 artifact path drift")
    callback = _verify_callback(callback_path)
    terminal = _verify_terminal(terminal_path)
    journal = _verify_journal(journal_path)
    return {
        "receipt_path": str(receipt_path),
        "receipt_sha256": receipt_sha,
        "receipt_sidecar_path": str(sidecar),
        "receipt_sidecar_sha256": ATTEMPT6_RECEIPT_SIDECAR_SHA256,
        "attempt_index": 6,
        "image_tag": IMAGE_TAG,
        "image_id": IMAGE_ID,
        "callback": callback,
        "terminal": terminal,
        "service_journal": journal,
        "status": "PASS",
    }


def _source_bindings(repo_root: Path) -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}
    for label, relative in SOURCE_PATHS.items():
        path = repo_root / relative
        lowered = str(path).lower()
        if any(token in lowered for token in FORBIDDEN_ARGV_SUBSTRINGS):
            raise _error("SOURCE_FORBIDDEN_PATH", f"{label} path contains a forbidden surface")
        _regular(path, label)
        result[label] = {"path": str(path.resolve()), "sha256": sha256_file(path)}
    if result["no_input_runner"]["sha256"] != ATTEMPT6_OBSERVED_RUNNER_SHA256:
        raise _error("SOURCE_DRIFT", "current no-input runner differs from attempt-6 observed source")
    return result


def build_pytest_argv(repo_root: Path = ROOT) -> list[str]:
    paths = [str(repo_root / relative) for relative in FIXED_TESTS]
    argv = [str(Path(sys.executable).resolve()), "-m", "pytest", "-q", *paths]
    validate_pytest_argv(argv, repo_root)
    return argv


def validate_pytest_argv(argv: Sequence[str], repo_root: Path = ROOT) -> None:
    if not isinstance(argv, (list, tuple)) or not argv:
        raise _error("PYTEST_ARGV", "pytest argv is empty")
    if any(not isinstance(item, str) or not item for item in argv):
        raise _error("PYTEST_ARGV", "pytest argv contains a non-string/empty item")
    if argv[1:4] != ["-m", "pytest", "-q"]:
        raise _error("PYTEST_ARGV", "pytest argv is not the fixed module invocation")
    expected_paths = [str(repo_root / relative) for relative in FIXED_TESTS]
    if list(argv[4:]) != expected_paths:
        raise _error("PYTEST_ARGV", "pytest test collection drift")
    lowered = "\n".join(argv).lower()
    if any(token in lowered for token in FORBIDDEN_ARGV_SUBSTRINGS):
        raise _error("PYTEST_FORBIDDEN_ARG", "pytest argv/path contains a forbidden surface")
    for path in expected_paths:
        _regular(Path(path), "pytest test source")


def canonical_command_sha256(argv: Sequence[str]) -> str:
    payload = json.dumps(list(argv), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_create_bytes(path: Path, payload: bytes, *, mode: int = 0o444) -> None:
    if os.path.lexists(path):
        raise _error("OUTPUT_OVERWRITE", f"refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise _error("OUTPUT_STAGING", f"stale staging file exists: {part}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(part, flags, 0o600)
    try:
        with os.fdopen(fd, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(part, path, follow_symlinks=False)
        except FileExistsError as exc:
            raise _error("OUTPUT_OVERWRITE", f"refusing to overwrite {path}") from exc
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)


def reserve_root(root: Path) -> Path:
    if os.path.lexists(root):
        raise _error("ROOT_NOT_FRESH", f"fresh host evidence root already exists: {root}")
    root.parent.mkdir(parents=True, exist_ok=True)
    try:
        root.mkdir()
    except FileExistsError as exc:
        raise _error("ROOT_NOT_FRESH", f"fresh host evidence root already exists: {root}") from exc
    return root


def _pytest_version() -> str:
    try:
        return importlib.metadata.version("pytest")
    except importlib.metadata.PackageNotFoundError as exc:
        raise _error("PYTEST_MISSING", "pytest distribution is unavailable") from exc


Executor = Callable[[Sequence[str], Path], Any]


def _default_executor(argv: Sequence[str], repo_root: Path) -> Any:
    return subprocess.run(
        list(argv), cwd=repo_root, shell=False, capture_output=True, check=False,
    )


def _as_bytes(value: Any) -> bytes:
    if value is None:
        return b""
    if isinstance(value, bytes):
        return value
    return str(value).encode("utf-8", errors="replace")


def run_gate(
        root: Path, *, repo_root: Path = ROOT, executor: Optional[Executor] = None,
        now: Optional[str] = None) -> dict[str, Any]:
    """Reserve a fresh root and run exactly one fixed pytest command."""
    reserve_root(root)
    receipt_path = root / RECEIPT_NAME
    started = now or dt.datetime.now(dt.timezone.utc).isoformat()
    argv: list[str] | None = None
    stdout = b""
    stderr = b""
    returncode: int | None = None
    failure_kind: str | None = None
    failure_message: str | None = None
    attempt: dict[str, Any] | None = None
    sources: dict[str, dict[str, str]] | None = None
    pytest_metadata: dict[str, Any] = {}
    try:
        attempt = verify_attempt6()
        sources = _source_bindings(repo_root)
        argv = build_pytest_argv(repo_root)
        command_sha = canonical_command_sha256(argv)
        pytest_metadata = {
            "version": _pytest_version(),
            "python_executable": str(Path(sys.executable).resolve()),
            "argv": list(argv),
            "command_canonical_sha256": command_sha,
            "fixed_tests": list(FIXED_TESTS),
        }
        runner = _default_executor if executor is None else executor
        try:
            completed = runner(argv, repo_root)
            returncode = int(completed.returncode)
            stdout = _as_bytes(getattr(completed, "stdout", b""))
            stderr = _as_bytes(getattr(completed, "stderr", b""))
        except Exception as exc:  # no retry; preserve failure in the receipt
            returncode = 1
            failure_kind = "PYTEST_EXECUTOR_EXCEPTION"
            failure_message = f"{type(exc).__name__}: {exc}"
            stderr = failure_message.encode("utf-8", errors="replace")
        _atomic_create_bytes(root / "pytest.stdout", stdout, mode=0o444)
        _atomic_create_bytes(root / "pytest.stderr", stderr, mode=0o444)
        pytest_metadata.update({
            "returncode": returncode,
            "stdout_path": str(root / "pytest.stdout"),
            "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
            "stderr_path": str(root / "pytest.stderr"),
            "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        })
        if returncode != 0:
            failure_kind = failure_kind or "PYTEST_FAILED"
            failure_message = failure_message or f"pytest returned {returncode}"
    except HostEvidenceGateError as exc:
        failure_kind = exc.kind
        failure_message = str(exc)
    except Exception as exc:  # fail closed while preserving a sealed receipt
        failure_kind = "HOST_GATE_EXCEPTION"
        failure_message = f"{type(exc).__name__}: {exc}"

    status = "PASS" if failure_kind is None and returncode == 0 else "FAIL_CLOSED"
    receipt: dict[str, Any] = {
        "schema_version": 1,
        "contract_version": CONTRACT_VERSION,
        "status": status,
        "created_at_utc": started,
        "root": {"path": str(root), "fresh_reserved": True, "overwrite_refused": True},
        "attempt6": attempt,
        "sources": sources,
        "pytest": pytest_metadata,
        "failure_kind": failure_kind,
        "failure_message": failure_message,
        "safety": {
            "formal_replay_started": False,
            "ground_truth_content_opened": False,
            "input_opened": False,
            "map_saved": False,
            "scorer_invoked": False,
        },
        "formal_replay_started": False,
    }
    payload = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8")
    _atomic_create_bytes(receipt_path, payload, mode=0o444)
    receipt_sha = sha256_file(receipt_path)
    sidecar_path = receipt_path.with_name(receipt_path.name + ".sha256")
    _atomic_create_bytes(
        sidecar_path, f"{receipt_sha}  {receipt_path.name}\n".encode("ascii"), mode=0o444)
    receipt["receipt_path"] = str(receipt_path)
    receipt["receipt_sha256"] = receipt_sha
    receipt["sidecar_path"] = str(sidecar_path)
    return receipt


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = __import__("argparse").ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    result = run_gate(args.root, repo_root=args.repo_root)
    print(json.dumps({key: result.get(key) for key in (
        "status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
