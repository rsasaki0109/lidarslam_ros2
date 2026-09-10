#!/usr/bin/env python3
"""Host-only v18 exact-root authorizer using the shared verifier.

This creates one additive v18 authorization and exactly three read-only
quiescence receipts.  It never opens input, starts Docker, runs a replay,
invokes scoring, or writes a map.  The sealed result is immediately checked by
the same shared verifier used by the launcher.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402
import lidarslam_benchmark_tools.verify_fast_livo2_m6a10_v18_authorization as verifier  # noqa: E402
from lidarslam_benchmark_tools.verify_fast_livo2_m6a10_v18_authorization import (  # noqa: F401,E402
    AUTHORIZATION_RECEIPT_PATH,
    AuthorizationError,
    verify_authorization,
)


ROOT = Path(__file__).resolve().parents[1]
AUTHORIZATION_ROOT = verifier.V18_AUTHORIZATION_RECEIPT_PATH.parent
ATTEMPT_ROOT = verifier.V18_ATTEMPT_ROOT
WINDOW_COUNT = 3
WINDOW_SECONDS = 4.0
MAX_BUSY_PERCENT = 5.0
MAX_LOAD_PER_CPU = 0.50


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise AuthorizationError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise AuthorizationError("OUTPUT_STAGING", "staging file exists")
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)
    return hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> str:
    return _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _run_window(path: Path, *, proc_root: Path = Path("/proc"), now: Optional[str] = None) -> Mapping[str, Any]:
    excluded = quiescence.ancestor_pids(proc_root)
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    receipt = quiescence.build_receipt(observation, now=now)
    receipt.update({"authorization_window": True, "launcher_pid": os.getpid(),
                    "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, receipt)
    return {"path": str(path), "sha256": digest, "status": receipt.get("status"),
            "runner_start_allowed": receipt.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", [])}


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / "formal_authorization.receipt.json"
    digest = _write_json(receipt, value)
    sidecar = receipt.with_name(receipt.name + ".sha256")
    sidecar_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=sidecar_digest)


WindowRunner = Callable[..., Mapping[str, Any]]


def authorize(*, authorization_root: Path = AUTHORIZATION_ROOT,
              attempt_root: Path = ATTEMPT_ROOT, repo_root: Path = ROOT,
              proc_root: Path = Path("/proc"), now: Optional[str] = None,
              window_runner: Optional[WindowRunner] = None) -> Dict[str, Any]:
    if authorization_root.resolve() != AUTHORIZATION_ROOT.resolve() or attempt_root.resolve() != ATTEMPT_ROOT.resolve():
        raise AuthorizationError("AUTHORIZATION_ROOT", "v18 roots differ from fixed candidate roots")
    if os.path.lexists(authorization_root) or os.path.lexists(attempt_root):
        raise AuthorizationError("ROOT_NOT_FRESH", "v18 authorization/formal root must be absent")
    authorization_root.mkdir(parents=True)
    try:
        lineage = verifier.build_v18_lineage(repo_root)
        runner = window_runner or _run_window
        windows: List[Dict[str, Any]] = []
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now))
            result["window_index"] = index
            windows.append(result)
        passed = len(windows) == WINDOW_COUNT and all(
            item.get("status") == "PASS" and item.get("runner_start_allowed") is True and
            not item.get("forbidden_processes") for item in windows)
        value: Dict[str, Any] = {
            "schema_version": 1, "contract_version": verifier.V18_AUTHORIZATION_CONTRACT,
            "status": "AUTHORIZED" if passed else "FAIL_CLOSED", "authorized": passed,
            "formal_execution": passed, "formal_replay_forbidden": not passed,
            "formal_replay_started": False, "attempt_root": str(attempt_root),
            "attempt_count": 1, "one_start": True, "retry": False, "manual_stop": False,
            "watchdog_seconds": verifier.WATCHDOG_SECONDS,
            "image": {"tag": verifier.IMAGE_TAG, "id": verifier.IMAGE_ID},
            "input": {"path": verifier.INPUT_PATH, "bytes": verifier.INPUT_BYTES,
                      "sha256": verifier.INPUT_SHA256, "expected_messages": verifier.EXPECTED_MESSAGES,
                      "expected_topic_counts": verifier.EXPECTED_COUNTS,
                      "required_end_timestamp_seconds": verifier.REQUIRED_END_TIMESTAMP_SECONDS,
                      "sensor_duration_seconds": verifier.SENSOR_DURATION_SECONDS},
            "mount_contract": {"network": "none", "rootfs": "read_only", "output": "/out",
                               "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                               "ground_truth_mount": False, "scorer_mount": False, "map_mount": False},
            "monitor": {"continuous": True, "interval_seconds": 4.0, "natural_completion": True},
            "lineage": lineage, "windows": windows,
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        }
        if not passed:
            value["failure_kind"] = "QUIESCENCE_FAIL_CLOSED"
        sealed = _seal(authorization_root, value)
        # Re-read the just-sealed bytes through the production verifier.  This
        # is still host-only and does not reserve or open the formal root.
        if passed:
            verifier.verify_authorization(Path(sealed["receipt_path"]), attempt_root,
                                          sealed["receipt_sha256"], repo_root=repo_root)
        return sealed
    except Exception as exc:
        if (authorization_root / "formal_authorization.receipt.json").exists():
            raise
        return _seal(authorization_root, {
            "schema_version": 1, "contract_version": verifier.V18_AUTHORIZATION_CONTRACT,
            "status": "FAIL_CLOSED", "authorized": False, "formal_execution": False,
            "formal_replay_forbidden": True, "formal_replay_started": False,
            "attempt_root": str(attempt_root), "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(exc),
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        })


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION_ROOT)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--create", action="store_true")
    args = parser.parse_args(argv)
    if args.create:
        try:
            value = authorize(authorization_root=args.authorization, attempt_root=args.attempt_root)
        except Exception as exc:
            print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                              "failure_message": str(exc)}, sort_keys=True))
            return 11
        print(json.dumps({key: value.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")},
                         sort_keys=True))
        return 0 if value.get("status") == "AUTHORIZED" else 1
    if not args.authorization_sha256:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": "AUTHORIZATION_SHA_REQUIRED"}, sort_keys=True))
        return 11
    try:
        value = verify_authorization(args.authorization, args.attempt_root,
                                     args.authorization_sha256)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({"status": value.get("status"), "attempt_root": value.get("attempt_root")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
