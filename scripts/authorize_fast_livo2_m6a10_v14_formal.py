#!/usr/bin/env python3
"""Create or verify one additive v14 exact-root formal authorization.

The v14 authorization is intentionally separate from the immutable v14
candidate profile.  It binds the v12 no-input/host evidence, the v13 output
persistence receipt, and the v13 shared-builder recursion failure/correction.
Authorization never opens the bag, starts Docker/ROS, invokes scoring, or
creates a map.  The candidate remains unauthorized for every other root.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v12_formal as base  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v13_persistence_gate as persistence  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v14_formal as candidate  # noqa: E402


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v14_formal_candidate.yaml"
PROFILE_SHA256 = "849585ae1d2bb118db9a261f42305886b0305628473cea964781695f01b63996"
CANDIDATE_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v14_formal.py"
CANDIDATE_LAUNCHER_SHA256 = "26610f1de4f0cec2d231e54641c88300dbe3fefb8fb1dfbbec686a2acb266459"
CANDIDATE_TEST_PATH = ROOT / "graph_based_slam/test/test_fast_livo2_m6a10_v14_formal.py"
CANDIDATE_TEST_SHA256 = "633e2db908c442b2c4fdcfe23c0db14499ed69a314d18c214e181298fffa5879"
AUTHORIZED_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v14_formal_authorized.py"
IMAGE_TAG = candidate.IMAGE_TAG
IMAGE_ID = candidate.IMAGE_ID
PHASE_CONTRACT = base.PHASE_CONTRACT
TRANSPORT_CONTRACT = base.TRANSPORT_CONTRACT
INPUT_PATH = base.INPUT_PATH
INPUT_BYTES = base.INPUT_BYTES
INPUT_SHA256 = base.INPUT_SHA256
EXPECTED_COUNTS = base.EXPECTED_COUNTS
EXPECTED_MESSAGES = base.EXPECTED_MESSAGES
CONTRACT_VERSION = "m6a10-v14-formal-exact-root-authorization-v1"
RECEIPT_NAME = "formal_authorization.receipt.json"

V13_AUTHORIZATION_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_formal_authorization_20260823T194325Z/"
    "formal_authorization.receipt.json"
)
V13_AUTHORIZATION_SHA256 = "e7e2aa9cd9f8d1ef884595cc4c71158c077d7e8ef8dcfd2b9d1ff23152c2ae83"
V13_AUTHORIZATION_SIDECAR_SHA256 = "356a21489f3de543163da15618e3af70f658e3a5a1fdbf6cb180505288492c44"
V13_CLOSURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_formal_replay_20260823T194325Z_agentv13formal/"
    "closure_receipt.json"
)
V13_CLOSURE_SHA256 = "73252dd828388a5ff026413ec957cb02504dea4ba14b936678257333954b8212"
V13_CORRECTION_PATH = V13_CLOSURE_PATH.with_name("closure_correction.receipt.json")
V13_CORRECTION_SHA256 = "24c9ceb1d2b2da8e8ec95c54aa00ad723f1713146ff21a4b235533ebf776fd0e"
V13_PERSISTENCE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_persistence_20260823T192720Z_agentv13/"
    "persistence_gate.receipt.json"
)
V13_PERSISTENCE_SHA256 = "18998f6177418eaf7e98fe517691d0642d59ac48f2faeb9b45ef0bc8fef62c72"
V13_PERSISTENCE_SIDECAR_SHA256 = "1988676efabdf1276ca3bb635f784941d2539aa15f58f94a8f249bcb730eabb7"


class AuthorizationError(ValueError):
    """Fail-closed authorization error with stable machine-readable kind."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _pin(path: Path, expected: str, label: str) -> str:
    try:
        base._regular(path, label)
    except Exception as exc:
        raise AuthorizationError(getattr(exc, "kind", "NOT_REGULAR"), str(exc)) from exc
    observed = sha256_file(path)
    if observed != expected:
        raise AuthorizationError("SOURCE_DRIFT", "%s SHA drift" % label)
    return observed


def _json(path: Path, expected: str, label: str, sidecar_sha: Optional[str] = None) -> Dict[str, Any]:
    _pin(path, expected, label)
    if sidecar_sha is not None:
        sidecar = path.with_name(path.name + ".sha256")
        _pin(sidecar, sidecar_sha, "%s sidecar" % label)
        expected_line = "%s  %s\n" % (expected, path.name)
        if sidecar.read_text(encoding="ascii") != expected_line:
            raise AuthorizationError("RECEIPT_SIDECAR", "%s sidecar content drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_INVALID", "%s is not valid JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s is not an object" % label)
    return value


def _verify_v14_profile(repo_root: Path = ROOT) -> Dict[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    if path.resolve() != PROFILE_PATH.resolve():
        raise AuthorizationError("PROFILE_PATH", "v14 profile path differs")
    _pin(path, PROFILE_SHA256, "v14 candidate profile")
    value = candidate.verify_candidate_profile(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("status") != "V14_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0:
        raise AuthorizationError("PROFILE_AUTHORITY", "v14 profile is not fail-closed")
    return {"path": str(path.resolve()), "sha256": value["sha256"], "document": document}


def _verify_immutable_lineage(repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify all v12/v13 receipts and v14 source identity read-only."""
    profile = _verify_v14_profile(repo_root)
    _pin(repo_root / CANDIDATE_LAUNCHER_PATH.relative_to(ROOT), CANDIDATE_LAUNCHER_SHA256, "v14 candidate launcher")
    _pin(repo_root / CANDIDATE_TEST_PATH.relative_to(ROOT), CANDIDATE_TEST_SHA256, "v14 candidate test")
    authorized_launcher_sha = _pin(
        repo_root / AUTHORIZED_LAUNCHER_PATH.relative_to(ROOT),
        sha256_file(repo_root / AUTHORIZED_LAUNCHER_PATH.relative_to(ROOT)),
        "v14 authorized launcher",
    )

    # This existing verifier binds the v12 build/no-input/host evidence and
    # the v13 source/persistence lineage without opening the bag.
    v12_v13 = persistence.verify_candidate_lineage(repo_root)
    v13_auth = _json(
        V13_AUTHORIZATION_PATH, V13_AUTHORIZATION_SHA256,
        "v13 authorization", V13_AUTHORIZATION_SIDECAR_SHA256,
    )
    if v13_auth.get("status") != "AUTHORIZED" or v13_auth.get("formal_replay_authorized") is not True or \
            v13_auth.get("formal_execution") is not True:
        raise AuthorizationError("V13_AUTHORIZATION", "v13 authorization is not immutable AUTHORIZED")

    v13_closure = _json(V13_CLOSURE_PATH, V13_CLOSURE_SHA256, "v13 recursion closure")
    if v13_closure.get("status") != "FAIL_CLOSED" or \
            v13_closure.get("failure_kind") != "DOCKER_ARGV_BUILDER_RECURSION" or \
            v13_closure.get("execution", {}).get("popen_count") != 0:
        raise AuthorizationError("V13_FAILURE_LINEAGE", "v13 recursion closure is not the expected failure")
    v13_correction = _json(V13_CORRECTION_PATH, V13_CORRECTION_SHA256, "v13 recursion correction")
    if v13_correction.get("status") != "CORRECTION_BOUND_TO_IMMUTABLE_CLOSURE" or \
            v13_correction.get("actual_process_exit_code") != 1 or \
            v13_correction.get("failure_kind") != "DOCKER_ARGV_BUILDER_RECURSION":
        raise AuthorizationError("V13_CORRECTION", "v13 correction does not bind process exit 1")
    persistence_receipt = _json(
        V13_PERSISTENCE_PATH, V13_PERSISTENCE_SHA256,
        "v13 Docker persistence receipt", V13_PERSISTENCE_SIDECAR_SHA256,
    )
    if persistence_receipt.get("status") != "PASS" or \
            persistence_receipt.get("feeder_root_cause") != "unknown":
        raise AuthorizationError("V13_PERSISTENCE", "v13 persistence receipt is not admissible")
    return {
        "v14_profile": profile,
        "v14_candidate_launcher": {
            "path": str(CANDIDATE_LAUNCHER_PATH.resolve()),
            "sha256": CANDIDATE_LAUNCHER_SHA256,
        },
        "v14_candidate_test": {
            "path": str(CANDIDATE_TEST_PATH.resolve()),
            "sha256": CANDIDATE_TEST_SHA256,
        },
        "v14_authorized_launcher": {
            "path": str(AUTHORIZED_LAUNCHER_PATH.resolve()),
            "sha256": authorized_launcher_sha,
        },
        "v12_v13_lineage": v12_v13,
        "v13_authorization": {"path": str(V13_AUTHORIZATION_PATH), "sha256": V13_AUTHORIZATION_SHA256},
        "v13_recursion_closure": {"path": str(V13_CLOSURE_PATH), "sha256": V13_CLOSURE_SHA256},
        "v13_recursion_correction": {"path": str(V13_CORRECTION_PATH), "sha256": V13_CORRECTION_SHA256},
        "v13_persistence": {"path": str(V13_PERSISTENCE_PATH), "sha256": V13_PERSISTENCE_SHA256},
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
    }


def _enriched_write_json(path: Path, value: Mapping[str, Any]) -> str:
    if path.name == RECEIPT_NAME:
        enriched = dict(value)
        enriched.update({
            "candidate_version": "v14-scoped-original-builder",
            "v14_builder_fix": "capture_original_before_patch_and_call_original_only",
            "feeder_root_cause_fixed": False,
            "feeder_underlying_nonzero_cause": "unknown",
            "v13_recursion_failure_sha256": V13_CLOSURE_SHA256,
            "v13_recursion_correction_sha256": V13_CORRECTION_SHA256,
            "v13_persistence_receipt_sha256": V13_PERSISTENCE_SHA256,
        })
        value = enriched
    return BASE_WRITE_JSON(path, value)


_BASE_GLOBALS = (
    "PROFILE_PATH", "PROFILE_SHA256", "IMAGE_TAG", "IMAGE_ID", "PHASE_CONTRACT",
    "TRANSPORT_CONTRACT", "CONTRACT_VERSION", "RECEIPT_NAME", "_verify_immutable_lineage",
    "_write_json",
)
BASE_WRITE_JSON = base._write_json


def _patch_base() -> Dict[str, Any]:
    saved = {name: getattr(base, name) for name in _BASE_GLOBALS}
    base.PROFILE_PATH = PROFILE_PATH
    base.PROFILE_SHA256 = PROFILE_SHA256
    base.IMAGE_TAG = IMAGE_TAG
    base.IMAGE_ID = IMAGE_ID
    base.PHASE_CONTRACT = PHASE_CONTRACT
    base.TRANSPORT_CONTRACT = TRANSPORT_CONTRACT
    base.CONTRACT_VERSION = CONTRACT_VERSION
    base.RECEIPT_NAME = RECEIPT_NAME
    base._verify_immutable_lineage = _verify_immutable_lineage
    base._write_json = _enriched_write_json
    return saved


def _restore_base(saved: Mapping[str, Any]) -> None:
    for name, value in saved.items():
        setattr(base, name, value)


def authorize(*args: Any, **kwargs: Any) -> Dict[str, Any]:
    saved = _patch_base()
    try:
        return base.authorize(*args, **kwargs)
    finally:
        _restore_base(saved)


def verify_authorization(*args: Any, **kwargs: Any) -> Dict[str, Any]:
    saved = _patch_base()
    try:
        value = base.verify_authorization(*args, **kwargs)
    finally:
        _restore_base(saved)
    if value.get("candidate_version") != "v14-scoped-original-builder" or \
            value.get("v14_builder_fix") != "capture_original_before_patch_and_call_original_only" or \
            value.get("feeder_root_cause_fixed") is not False or \
            value.get("feeder_underlying_nonzero_cause") != "unknown":
        raise AuthorizationError("AUTHORIZATION_CANDIDATE", "v14 authorization candidate binding drift")
    launcher = value.get("lineage", {}).get("v14_authorized_launcher", {})
    if not isinstance(launcher, Mapping) or launcher.get("path") != str(AUTHORIZED_LAUNCHER_PATH.resolve()) or \
            sha256_file(AUTHORIZED_LAUNCHER_PATH) != launcher.get("sha256"):
        raise AuthorizationError("AUTHORIZED_LAUNCHER_DRIFT", "v14 authorized launcher source drift")
    value = dict(value)
    value["authorized"] = True
    return value


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    args = parser.parse_args(argv)
    try:
        result = authorize(
            args.authorization_root, args.attempt_root,
            repo_root=args.repo_root, proc_root=args.proc_root,
        )
    except (AuthorizationError, base.AuthorizationError) as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"), "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
