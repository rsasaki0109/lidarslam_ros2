#!/usr/bin/env python3
"""One authorized v14 formal lifecycle with a scoped original-builder fix.

This file is additive to the unauthorized v14 candidate.  It never installs
authorization on import and never retries a root.  The v12 lifecycle is used
for the already-reviewed identity, monitor, capture, composition, and
stopped-only cleanup seams; only the profile, authorization validator,
original-builder mount transform, and persistence capture are scoped and
restored around one call.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v14_formal as authorizer  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v14_formal as candidate  # noqa: E402


BASE = candidate.v12
BASE_ATOMIC_CREATE_JSON = BASE._atomic_create_json
PROFILE_PATH = authorizer.PROFILE_PATH
PROFILE_SHA256 = authorizer.PROFILE_SHA256
AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v14_formal.py"
AUTHORIZER_SHA256 = "9ed8403c4e399570aef180962a19119811a70df6295ac9024df07c6d713e8e66"
AUTHORIZED_CONTRACT = authorizer.CONTRACT_VERSION
V14_CANDIDATE_VERSION = "v14-scoped-original-builder"


class CandidateError(ValueError):
    """Fail-closed wrapper error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _capture_with_persistence_diagnostics(config: Any, root: Path) -> Mapping[str, Any]:
    """Capture raw evidence and feeder stderr/receipt bytes after natural exit."""
    paths = {
        "feeder": root / "out/feeder_receipt.json",
        "callback": root / "out/callback_consumer_evidence.json",
        "terminal": root / "out/consumer_evidence.json",
        "timing": root / "out/online_compute_timing.json",
        "feeder_log": root / "out/feeder.log",
        "feeder_exit_status": root / "out/feeder_exit_status.txt",
    }
    values: Dict[str, Any] = {}
    bindings: Dict[str, Dict[str, Any]] = {}
    missing = []
    for label, path in paths.items():
        if path.is_symlink() or not path.is_file():
            missing.append(label)
            continue
        raw = path.read_bytes()
        bindings[label] = {
            "path": str(path),
            "bytes": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest(),
        }
        if label in {"feeder", "callback", "terminal", "timing"}:
            try:
                values[label] = json.loads(raw.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError):
                values[label] = {"_invalid_json": True}
        elif label == "feeder_exit_status":
            values[label] = raw.decode("utf-8", "replace").strip()
        else:
            values[label] = {"bytes": len(raw), "sha256": bindings[label]["sha256"]}
    return {
        "terminal": values.get("terminal", {}),
        "documents": values,
        "bindings": bindings,
        "missing": missing,
        "feeder_persistence": {
            "receipt_present": "feeder" not in missing,
            "stderr_present": "feeder_log" not in missing,
            "exit_status_present": "feeder_exit_status" not in missing,
        },
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
    }


def _enriched_atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    if path.name == "closure_receipt.json":
        enriched = dict(value)
        enriched.update({
            "contract_version": "m6a10-v14-formal-candidate-closure-v1",
            "candidate_version": V14_CANDIDATE_VERSION,
            "v14_builder_fix": "capture_original_before_patch_and_call_original_only",
            "authorization_contract": AUTHORIZED_CONTRACT,
            "feeder_root_cause_fixed": False,
            "feeder_underlying_nonzero_cause": "unknown",
            "v13_recursion_failure_sha256": authorizer.V13_CLOSURE_SHA256,
            "v13_recursion_correction_sha256": authorizer.V13_CORRECTION_SHA256,
            "v13_persistence_receipt_sha256": authorizer.V13_PERSISTENCE_SHA256,
        })
        value = enriched
    return BASE_ATOMIC_CREATE_JSON(path, value, mode)


def _verify_profile(profile_path: Path = PROFILE_PATH, repo_root: Path = ROOT) -> Mapping[str, Any]:
    return authorizer._verify_v14_profile(repo_root)


def _verify_authorization(config: Any) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        raise authorizer.AuthorizationError(
            "FORMAL_REPLAY_UNAUTHORIZED",
            "v14 formal authorization path and SHA are required",
        )
    if not AUTHORIZER_PATH.is_file() or AUTHORIZER_PATH.is_symlink() or \
            hashlib.sha256(AUTHORIZER_PATH.read_bytes()).hexdigest() != AUTHORIZER_SHA256:
        raise authorizer.AuthorizationError("AUTHORIZER_DRIFT", "v14 authorization source drift")
    value = authorizer.verify_authorization(
        config.authorization_path,
        config.root,
        config.authorization_sha256,
        repo_root=config.repo_root,
    )
    if value.get("authorized") is not True or value.get("formal_execution") is not True:
        raise authorizer.AuthorizationError("AUTHORIZATION_STATUS", "v14 authorization is not executable")
    return value


def _base_snapshot() -> Dict[str, Any]:
    return {
        name: getattr(BASE, name) for name in (
            "PROFILE_PATH", "PROFILE_SHA256", "verify_candidate_profile",
            "build_safe_docker_argv", "_production_capture", "_atomic_create_json",
        )
    }


def _restore_base(snapshot: Mapping[str, Any]) -> None:
    for name, value in snapshot.items():
        setattr(BASE, name, value)


def run_formal(config: Any, **kwargs: Any) -> Mapping[str, Any]:
    """Run exactly one v12 lifecycle under scoped v14 state."""
    snapshot = _base_snapshot()
    BASE.PROFILE_PATH = PROFILE_PATH
    BASE.PROFILE_SHA256 = PROFILE_SHA256
    BASE.verify_candidate_profile = _verify_profile
    # Crucially, v14's transform calls candidate.ORIGINAL_V12_BUILD_DOCKER_ARGV,
    # not this patched shared symbol, so v13's recursion cannot recur.
    BASE.build_safe_docker_argv = candidate.build_safe_docker_argv
    BASE._production_capture = _capture_with_persistence_diagnostics
    BASE._atomic_create_json = _enriched_atomic_json
    try:
        return BASE.run_formal(
            config,
            authorization_validator=_verify_authorization,
            **kwargs,
        )
    finally:
        _restore_base(snapshot)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=BASE.INPUT_PATH)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--authorization-sha256", required=True)
    parser.add_argument("--container-name", default="m6a10-v14-formal-candidate")
    args = parser.parse_args(argv)
    try:
        result = run_formal(BASE.CandidateConfig(
            root=args.root,
            repo_root=args.repo_root,
            bag_path=args.bag,
            profile_path=PROFILE_PATH,
            container_name=args.container_name,
            authorization_path=args.authorization,
            authorization_sha256=args.authorization_sha256,
        ))
    except (BASE.CandidateError, authorizer.AuthorizationError) as error:
        print(json.dumps({
            "status": "FAIL_CLOSED",
            "failure_kind": getattr(error, "kind", "FORMAL_FAIL_CLOSED"),
            "failure_message": str(error),
        }, sort_keys=True))
        return 11
    print(json.dumps({
        key: result.get(key)
        for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")
    }, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
