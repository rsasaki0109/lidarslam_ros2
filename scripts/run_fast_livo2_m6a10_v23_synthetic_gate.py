#!/usr/bin/env python3
"""One-shot v23 input-free Docker runtime gate.

The actual container argv and persistence checks are inherited from the real
v22 synthetic gate module.  v23 adds a separate synthetic authorization
domain, read-only host-interference observation, and an immutable runtime
receipt.  No bag, input mount, GT, scorer, map, or formal launcher is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v22_synthetic_gate as v22_gate
import lidarslam_benchmark_tools.fast_livo2_m6a10_v23_shared_auth as shared_auth
from lidarslam_benchmark_tools.monitor_m6a10_host_interference import HostInterferenceMonitor


ROOT = Path(__file__).resolve().parents[1]
RUNTIME_RECEIPT_NAME = "v23_synthetic_runtime.receipt.json"
RUNTIME_CONTRACT = "m6a10-v23-synthetic-runtime-gate-v1"


class SyntheticGateError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _seal_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path) or os.path.lexists(path.with_name(path.name + ".part")):
        raise SyntheticGateError("OUTPUT_OVERWRITE", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise SyntheticGateError("OUTPUT_PARENT", str(path.parent))
    part = path.with_name(path.name + ".part")
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                 getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
        os.chmod(path, mode, follow_symlinks=False)
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    return hashlib.sha256(payload).hexdigest()


def _seal_json(path: Path, value: Mapping[str, Any]) -> str:
    return _seal_bytes(path, (json.dumps(value, sort_keys=True, indent=2) + "\n").encode())


def run_gate(*, root: Path, authorization_path: Path = shared_auth.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH,
             authorization_sha256: str = shared_auth.V23_SYNTHETIC_AUTHORIZATION_SHA256,
             now: Optional[str] = None) -> Dict[str, Any]:
    """Run exactly one actual input-free container gate and seal its result."""
    if os.path.lexists(root) or root.is_symlink():
        raise SyntheticGateError("ROOT_NOT_FRESH", str(root))
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise SyntheticGateError("ROOT_PARENT", str(root.parent))
    authorization = shared_auth.verify_synthetic_authorization(
        path=authorization_path, expected_sha256=authorization_sha256,
        expected_runtime_root=shared_auth.V23_SYNTHETIC_RUNTIME_ROOT)
    root.mkdir()
    monitor: Optional[HostInterferenceMonitor] = None
    monitor_summary: Mapping[str, Any] = {}
    gate_result: Optional[Mapping[str, Any]] = None
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    try:
        monitor = HostInterferenceMonitor(root / "host_interference.samples.jsonl",
                                          interval_seconds=4.0)
        monitor.start()
        gate_result = v22_gate.run_gate(root / "container_gate", "success")
        if gate_result.get("status") != "PASS":
            raise SyntheticGateError("CONTAINER_GATE", "v22 actual synthetic gate did not PASS")
    except Exception as exc:
        failure_kind = getattr(exc, "kind", "V23_SYNTHETIC_GATE_FAIL_CLOSED")
        failure_message = str(exc)
    finally:
        if monitor is not None:
            try:
                monitor.stop()
                monitor_summary = monitor.finalize(root / "host_interference.summary.json")
            except Exception as exc:
                failure_kind = failure_kind or "HOST_MONITOR_FAIL_CLOSED"
                failure_message = failure_message or str(exc)

    status = "PASS" if failure_kind is None and gate_result is not None and \
        monitor_summary.get("status") == "PASS" and monitor_summary.get("contaminated") is False and \
        monitor_summary.get("invalid") is False else "FAIL_CLOSED"
    if status != "PASS":
        failure_kind = failure_kind or "V23_SYNTHETIC_GATE_FAIL_CLOSED"
        failure_message = failure_message or "synthetic runtime evidence was not admissible"

    gate_receipt = None
    gate_sidecar = None
    if gate_result is not None:
        gate_receipt_path = Path(str(gate_result["receipt_path"]))
        gate_receipt = {"path": str(gate_receipt_path), "sha256": _sha256(gate_receipt_path)}
        sidecar = gate_receipt_path.with_name(gate_receipt_path.name + ".sha256")
        gate_sidecar = {"path": str(sidecar), "sha256": _sha256(sidecar)}
    value: Dict[str, Any] = {
        "schema_version": 1,
        "contract_version": RUNTIME_CONTRACT,
        "status": status,
        "gate_kind": "synthetic_runtime_only",
        "authorization": authorization,
        "authorization_path": str(authorization_path),
        "authorization_sha256": authorization_sha256,
        "runtime_root": str(root),
        "image": {"tag": shared_auth.V17_IMAGE_TAG, "id": shared_auth.V17_IMAGE_ID},
        "execution": {
            "container_start_count": 1 if gate_result is not None else 0,
            "retry": False, "manual_stop": False,
            "natural_exit": gate_result is not None,
            "stopped_only_remove_success": gate_result is not None,
            "remove_force": False, "oom_killed": False,
            "network": "none", "rootfs": "read_only",
            "input_mount_count": 0, "rw_mount_destination": "/out",
            "output_tmpfs": False,
        },
        "monitor": dict(monitor_summary),
        "container_gate": gate_result,
        "container_gate_receipt": gate_receipt,
        "container_gate_sidecar": gate_sidecar,
        "source": {
            "v22_gate_path": str(v22_gate.__file__),
            "v22_gate_sha256": _sha256(Path(v22_gate.__file__)),
            "v22_wrapper_path": str(v22_gate.V22_WRAPPER),
            "v22_wrapper_sha256": v22_gate.V22_WRAPPER_SHA256,
            "v23_shared_auth_path": str(shared_auth.__file__),
            "v23_shared_auth_sha256": _sha256(Path(shared_auth.__file__)),
        },
        "artifacts": {
            "exit_reason_persisted": bool(gate_result),
            "manifest_persisted": bool(gate_result),
            "timing_persisted": bool(gate_result),
        },
        "safety": {
            "formal_replay_started": False, "input_opened": False,
            "ground_truth_content_opened": False, "scorer_invoked": False,
            "map_saved": False,
        },
        "sealed_at_utc": now or "synthetic-runtime",
    }
    if status != "PASS":
        value["failure_kind"] = failure_kind
        value["failure_message"] = failure_message
    receipt = root / RUNTIME_RECEIPT_NAME
    digest = _seal_json(receipt, value)
    sidecar = receipt.with_name(receipt.name + ".sha256")
    side_digest = _seal_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--now", default="synthetic-runtime")
    args = parser.parse_args(argv)
    try:
        result = run_gate(root=args.root, now=args.now)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V23_SYNTHETIC_GATE_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({key: result.get(key) for key in
                      ("status", "gate_kind", "receipt_path", "receipt_sha256", "sidecar_sha256")}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
