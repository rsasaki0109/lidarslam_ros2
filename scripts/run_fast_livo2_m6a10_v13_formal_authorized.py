#!/usr/bin/env python3
"""Authorized v13 formal lifecycle built on the tested v12 seams.

This additive launcher keeps the existing v13 mount correction and v12
runtime/image immutable.  It changes candidate identity/authorization and
captures feeder log/exit artifacts before host composition, so a nonzero
feeder result is diagnosable without claiming a root-cause fix.  The default
path remains fail-closed unless the exact v13 authorization is supplied.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v13_formal as authorizer  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v13_formal as mount_candidate  # noqa: E402


BASE = mount_candidate.v12
BASE_ATOMIC_JSON = BASE._atomic_create_json
ORIGINAL_VERIFY_PROFILE = BASE.verify_candidate_profile
ORIGINAL_V12_PROFILE_PATH = mount_candidate.PARENT_PROFILE_PATH
ORIGINAL_V12_PROFILE_SHA256 = mount_candidate.PARENT_PROFILE_SHA256
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v13_formal_candidate.yaml"
PROFILE_SHA256 = "53d49adf595ae1783f741e264351dca45e2b17b6c91e9259f3e5fa5d036060ab"
AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v13_formal.py"
AUTHORIZER_SHA256 = "180a0b7e6d310560d02507ce46cf0d660254dd1e721cb8c3094a4e131864a033"
V13_CONTRACT_VERSION = "m6a10-v13-formal-candidate-closure-v1"
V13_CANDIDATE_VERSION = "v13-mount-corrected-evidence-persistence"


def _base_snapshot() -> Dict[str, Any]:
    return {
        name: getattr(BASE, name) for name in (
            "PROFILE_PATH", "PROFILE_SHA256", "AUTHORIZER_PATH", "AUTHORIZER_SHA256",
            "verify_candidate_profile", "build_safe_docker_argv", "_production_capture",
            "_atomic_create_json",
        )
    }


def _restore_base(snapshot: Mapping[str, Any]) -> None:
    for name, value in snapshot.items():
        setattr(BASE, name, value)


def _verify_profile(profile_path: Path = PROFILE_PATH, repo_root: Path = ROOT) -> Mapping[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        if profile_path.resolve() == mount_candidate.PARENT_PROFILE_PATH.resolve():
            saved_path = BASE.PROFILE_PATH
            saved_sha = BASE.PROFILE_SHA256
            BASE.PROFILE_PATH = ORIGINAL_V12_PROFILE_PATH
            BASE.PROFILE_SHA256 = ORIGINAL_V12_PROFILE_SHA256
            try:
                return ORIGINAL_VERIFY_PROFILE(profile_path, repo_root)
            finally:
                BASE.PROFILE_PATH = saved_path
                BASE.PROFILE_SHA256 = saved_sha
        raise BASE.CandidateError("PROFILE_PATH", "v13 profile path differs")
    document = authorizer._verify_profile()
    lineage = authorizer._verify_immutable_lineage(repo_root)
    return {
        "path": str(profile_path.resolve()),
        "sha256": PROFILE_SHA256,
        "profile_key": "m6a10_fast_livo2_v2c_v13_formal_candidate",
        "document": document,
        "lineage": lineage,
    }


def _capture_with_feeder_diagnostics(config: Any, root: Path) -> Mapping[str, Any]:
    """Capture raw JSON and feeder diagnostics without dropping available logs."""
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
        bindings[label] = {"path": str(path), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
        if label in {"feeder", "callback", "terminal", "timing"}:
            try:
                values[label] = json.loads(raw.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError):
                values[label] = {"_invalid_json": True}
        elif label == "feeder_exit_status":
            values[label] = raw.decode("utf-8", "replace").strip()
    return {
        "terminal": values.get("terminal", {}),
        "documents": values,
        "bindings": bindings,
        "missing": missing,
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
    }


def _enriched_atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    if path.name == "closure_receipt.json":
        enriched = dict(value)
        enriched["contract_version"] = V13_CONTRACT_VERSION
        enriched["candidate_version"] = V13_CANDIDATE_VERSION
        enriched["evidence_persistence_candidate"] = True
        enriched["feeder_root_cause_fixed"] = False
        enriched["feeder_underlying_nonzero_cause"] = "unknown"
        enriched["persistence_receipt_sha256"] = authorizer.PERSISTENCE_RECEIPT_SHA256
        value = enriched
    return BASE_ATOMIC_JSON(path, value, mode)


def _patch_runtime() -> None:
    authorizer._patch_base_globals()
    BASE.PROFILE_PATH = PROFILE_PATH
    BASE.PROFILE_SHA256 = PROFILE_SHA256
    BASE.AUTHORIZER_PATH = AUTHORIZER_PATH
    BASE.AUTHORIZER_SHA256 = AUTHORIZER_SHA256
    BASE.verify_candidate_profile = _verify_profile
    BASE.build_safe_docker_argv = mount_candidate.build_safe_docker_argv
    BASE._production_capture = _capture_with_feeder_diagnostics
    BASE._atomic_create_json = _enriched_atomic_json


def build_safe_docker_argv(config: Any, output_dir: Path):
    return mount_candidate.build_safe_docker_argv(config, output_dir)


def run_formal(config: Any, **kwargs: Any) -> Mapping[str, Any]:
    snapshot = _base_snapshot()
    _patch_runtime()
    try:
        return BASE.run_formal(config, **kwargs)
    finally:
        _restore_base(snapshot)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=BASE.INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v13-formal-candidate")
    args = parser.parse_args(argv)
    try:
        result = run_formal(BASE.CandidateConfig(
            root=args.root, repo_root=args.repo_root, bag_path=args.bag,
            profile_path=PROFILE_PATH, container_name=args.container_name,
            authorization_path=args.authorization,
            authorization_sha256=args.authorization_sha256,
        ))
    except BASE.CandidateError as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": error.kind, "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
