#!/usr/bin/env python3
"""Authorized additive v16 runner.

The immutable v16 candidate remains unauthorized by itself.  This wrapper
binds one independently sealed v16 authorization receipt, supplies its
three-window result to the candidate without taking a second window, and
preserves the v16 shell-free one-start lifecycle.  It does not retry or stop
a process manually; Docker/ROS/bag access occurs only after authorization.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v16_formal as authorization  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v16_formal as candidate  # noqa: E402


CANDIDATE_LAUNCHER_SHA256 = "77cf40effb3e578d0c2c95e48ea34b6dc5aa77d7d2fbc7bdd954936b63949bcd"
AUTHORIZER_SHA256 = "3f26dd7c1b530d7eb6828a39c49a9829f692c8528e3da12f7515ea76bd397b26"
CONTRACT_VERSION = "m6a10-v16-authorized-closure-v1"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise RuntimeError("refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise RuntimeError("staging file exists: %s" % part)
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


def _seal_pre_popen_failure(config: Any, error: BaseException) -> Dict[str, Any]:
    """Seal a failure even when authorization fails before candidate root reservation."""
    root = Path(config.root)
    if os.path.lexists(root) is False:
        root.mkdir(parents=True)
    receipt_path = root / "closure_receipt.json"
    receipt = {
        "schema_version": 1,
        "contract_version": CONTRACT_VERSION,
        "status": "FAIL_CLOSED",
        "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "failure_kind": getattr(error, "kind", "PRE_POPEN_FAIL_CLOSED"),
        "failure_message": str(error),
        "pre_popen_failure": True,
        "candidate_launcher_sha256": CANDIDATE_LAUNCHER_SHA256,
        "authorization_path": str(config.authorization_path) if config.authorization_path else None,
        "authorization_sha256": config.authorization_sha256,
        "execution": {"formal_replay_started": False, "candidate_process_started": False,
                       "one_start": False, "popen_count": 0, "retry": False,
                       "manual_stop": False, "returncode": None, "shell": False},
        "safety": {"input_opened": False, "ground_truth_content_opened": False,
                   "scorer_invoked": False, "map_saved": False},
    }
    raw = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8")
    receipt_sha = _atomic_bytes(receipt_path, raw)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, ("%s  %s\n" % (receipt_sha, receipt_path.name)).encode("ascii"))
    receipt.update({"receipt_path": str(receipt_path), "receipt_sha256": receipt_sha,
                    "sidecar_path": str(sidecar), "sidecar_sha256": sidecar_sha})
    return receipt


def _authorized_seams(config: Any):
    cache: Dict[str, Mapping[str, Any]] = {}

    def validator(value: Any) -> Mapping[str, Any]:
        if value.root != config.root:
            raise authorization.AuthorizationError("AUTHORIZATION_ROOT", "config root changed")
        if _sha256_file(authorization.SCRIPT) != AUTHORIZER_SHA256:
            raise authorization.AuthorizationError("AUTHORIZER_DRIFT", "v16 authorizer source drift")
        result = authorization.verify_authorization(
            value.authorization_path, value.root, value.authorization_sha256,
            repo_root=value.repo_root,
        )
        cache[str(value.root)] = result
        return {"authorized": True, "formal_execution": True, "receipt": result}

    def quiescence(value: Any) -> Mapping[str, Any]:
        result = cache.get(str(value.root))
        if result is None:
            raise authorization.AuthorizationError("AUTHORIZATION_QUIESCENCE", "authorization result was not cached")
        windows = result.get("windows")
        if not isinstance(windows, list) or len(windows) != 3:
            raise authorization.AuthorizationError("AUTHORIZATION_QUIESCENCE", "sealed windows are incomplete")
        if any(item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or
               item.get("forbidden_processes") for item in windows):
            raise authorization.AuthorizationError("AUTHORIZATION_QUIESCENCE", "sealed windows are not PASS")
        return {"status": "PASS", "window_count": 3, "consecutive_passes": 3,
                "windows": windows, "sealed": True}

    return validator, quiescence


def run_authorized(config: Any, **kwargs: Any) -> Mapping[str, Any]:
    """Run candidate lifecycle after one exact authorization verification."""
    validator, quiescence = _authorized_seams(config)
    kwargs.pop("authorization_validator", None)
    kwargs.pop("quiescence_probe", None)
    return candidate.run_formal(
        config,
        authorization_validator=validator,
        quiescence_probe=quiescence,
        **kwargs,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=candidate.INPUT_PATH)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--authorization-sha256", required=True)
    parser.add_argument("--container-name", default="m6a10-v16-formal-authorized")
    args = parser.parse_args(argv)
    config = candidate.v12.CandidateConfig(
        root=args.root, repo_root=args.repo_root, bag_path=args.bag,
        profile_path=candidate.PROFILE_PATH, container_name=args.container_name,
        authorization_path=args.authorization,
        authorization_sha256=args.authorization_sha256,
    )
    try:
        result = run_authorized(config)
    except Exception as error:
        try:
            result = _seal_pre_popen_failure(config, error)
        except Exception as seal_error:
            print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": "CLOSURE_SEAL_FAILURE",
                              "failure_message": "%s; seal: %s" % (error, seal_error)}, sort_keys=True))
            return 11
        print(json.dumps({"status": result.get("status"), "failure_kind": result.get("failure_kind"),
                          "receipt_path": result.get("receipt_path"),
                          "receipt_sha256": result.get("receipt_sha256")}, sort_keys=True))
        return 11
    print(json.dumps({"status": result.get("status"), "failure_kind": result.get("failure_kind"),
                      "receipt_path": result.get("receipt_path"),
                      "receipt_sha256": result.get("receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
