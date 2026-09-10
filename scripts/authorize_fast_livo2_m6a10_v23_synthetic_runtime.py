#!/usr/bin/env python3
"""Create the separate v23 synthetic-runtime authorization receipt.

This authorization is deliberately not formal authorization.  It binds only
one input-free Docker smoke gate, reuses the immutable v22 PASS window files as
fixtures, and sets ``formal_replay_authorized`` to false.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import lidarslam_benchmark_tools.fast_livo2_m6a10_v23_shared_auth as shared_auth


ROOT = Path(__file__).resolve().parents[1]
AUTH_ROOT = shared_auth.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH.parent
RUNTIME_ROOT = Path(shared_auth.V23_SYNTHETIC_RUNTIME_ROOT)
RECEIPT_NAME = shared_auth.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH.name
CONTRACT = "m6a10-v23-synthetic-runtime-authorization-v1"


class SyntheticAuthorizationError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _seal_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path) or os.path.lexists(path.with_name(path.name + ".part")):
        raise SyntheticAuthorizationError("OUTPUT_OVERWRITE", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise SyntheticAuthorizationError("OUTPUT_PARENT", str(path.parent))
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


def authorize(*, now: Optional[str] = None) -> Dict[str, Any]:
    if os.path.lexists(AUTH_ROOT) or AUTH_ROOT.is_symlink():
        raise SyntheticAuthorizationError("ROOT_NOT_FRESH", str(AUTH_ROOT))
    if os.path.lexists(RUNTIME_ROOT) or RUNTIME_ROOT.is_symlink():
        raise SyntheticAuthorizationError("RUNTIME_ROOT_NOT_FRESH", str(RUNTIME_ROOT))
    if not AUTH_ROOT.parent.is_dir() or AUTH_ROOT.parent.is_symlink():
        raise SyntheticAuthorizationError("AUTH_PARENT", str(AUTH_ROOT.parent))
    fixture = shared_auth.verify_v22_authorization()
    AUTH_ROOT.mkdir()
    try:
        formal_document = json.loads(shared_auth.V22_AUTHORIZATION_RECEIPT_PATH.read_text(encoding="utf-8"))
        windows = formal_document.get("windows")
        if not isinstance(windows, list) or len(windows) != 3:
            raise SyntheticAuthorizationError("WINDOW_FIXTURE", "v22 fixture windows are incomplete")
        authorizer_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        shared_sha = hashlib.sha256(shared_auth.__file__ and Path(shared_auth.__file__).read_bytes()).hexdigest()
        value: Dict[str, Any] = {
            "schema_version": 1,
            "contract_version": CONTRACT,
            "status": "AUTHORIZED",
            "authorized": True,
            "gate_kind": "synthetic_runtime_only",
            "runtime_execution_authorized": True,
            "formal_replay_authorized": False,
            "formal_replay_forbidden": True,
            "formal_replay_started": False,
            "authorization_root": str(AUTH_ROOT),
            "runtime_root": str(RUNTIME_ROOT),
            "formal_attempt_root": shared_auth.V22_ATTEMPT_ROOT,
            "created_at_utc": now or "synthetic-runtime",
            "image": {"tag": shared_auth.V17_IMAGE_TAG, "id": shared_auth.V17_IMAGE_ID},
            "window_contract": {
                "fixture_domain": "v22_authorized_windows",
                "continuous": True, "count": 3, "sample_seconds": 4.0,
                "runner_start_allowed": True,
            },
            "windows": windows,
            "execution": {
                "container_start_count": 1, "input_mount_count": 0,
                "network": "none", "rootfs": "read_only", "output": "/out",
                "output_readonly": False, "output_tmpfs": False,
                "retry": False, "manual_stop": False,
            },
            "source": {
                "v22_authorized_receipt_path": str(shared_auth.V22_AUTHORIZATION_RECEIPT_PATH),
                "v22_authorized_receipt_sha256": shared_auth.V22_AUTHORIZATION_SHA256,
                "v22_authorized_sidecar_sha256": shared_auth.V22_AUTHORIZATION_SIDECAR_SHA256,
                "v22_wrapper_sha256": shared_auth.V22_WRAPPER_SHA256,
                "v19_adapter_sha256": shared_auth.V19_ADAPTER_SHA256,
                "v23_shared_auth_sha256": shared_sha,
                "v23_synthetic_authorizer_sha256": authorizer_sha,
            },
            "formal_lineage": {
                "status": fixture["status"],
                "receipt_path": fixture["receipt_path"],
                "receipt_sha256": fixture["receipt_sha256"],
                "formal_replay_authorized": False,
            },
            "safety": {
                "formal_replay_started": False, "input_opened": False,
                "ground_truth_content_opened": False, "scorer_invoked": False,
                "map_saved": False,
            },
        }
        receipt = AUTH_ROOT / RECEIPT_NAME
        digest = _seal_json(receipt, value)
        sidecar = receipt.with_name(receipt.name + ".sha256")
        side_digest = _seal_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
        return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                    sidecar_path=str(sidecar), sidecar_sha256=side_digest)
    except Exception as exc:
        receipt = AUTH_ROOT / RECEIPT_NAME
        if not receipt.exists():
            failure = {
                "schema_version": 1, "contract_version": CONTRACT,
                "status": "FAIL_CLOSED", "authorized": False,
                "gate_kind": "synthetic_runtime_only",
                "formal_replay_authorized": False, "formal_replay_forbidden": True,
                "formal_replay_started": False, "runtime_root": str(RUNTIME_ROOT),
                "failure_kind": getattr(exc, "kind", "SYNTHETIC_AUTHORIZATION_FAIL_CLOSED"),
                "failure_message": str(exc),
                "safety": {"formal_replay_started": False, "input_opened": False,
                           "ground_truth_content_opened": False, "scorer_invoked": False,
                           "map_saved": False},
            }
            _seal_json(receipt, failure)
        raise


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--now", default="synthetic-runtime")
    args = parser.parse_args(argv)
    try:
        value = authorize(now=args.now)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "SYNTHETIC_AUTHORIZATION_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({key: value.get(key) for key in
                      ("status", "gate_kind", "receipt_path", "receipt_sha256", "sidecar_sha256")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
