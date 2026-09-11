#!/usr/bin/env python3
"""v22 formal-candidate lineage and authorization verifier.

The v22 candidate is intentionally unauthorized.  This module only verifies
the immutable v17/v21 lineage and the two fresh, input-free v22 synthetic gate
receipts.  It never opens a bag, probes Docker, starts ROS, or creates an
attempt root.  A future additive authorization must be installed explicitly;
absence of that receipt is a hard preflight failure.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v22_formal_candidate.yaml"
AUTHORIZATION_CONTRACT = "m6a10-v22-formal-authorization-v1"
V22_WRAPPER_PATH = ROOT / "scripts/fast_livo2_m6a10_v22_formal_container_run.sh"
V22_WRAPPER_SHA256 = "6e74246d2b7ee60086f3b316b22c6ef69ca3f16307ffdecea3eae9ad2d794f67"
V22_GATE_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v22_synthetic_gate.py"
V22_GATE_SHA256 = "c306e9f75011c036e53738d2ab705e1d342288925432dbcac20f05fa82fb87c6"
V22_GATE_TEST_PATH = ROOT / "graph_based_slam/test/test_fast_livo2_m6a10_v22_synthetic_gate.py"
V22_GATE_TEST_SHA256 = "b6a34f8cb3a73fe95a93a1bba4927d7c29544ddc930151f79d0ce683eb2f27b7"

V17_IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
V17_IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
V17_BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json"
)
V17_BUILD_RECEIPT_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
V17_BUILD_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
V17_NO_INPUT_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/no_input.receipt.json"
)
V17_NO_INPUT_RECEIPT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
V17_NO_INPUT_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"

V21_CLEANUP_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v21_formal_replay_20260824T012000Z_agentv21formal/cleanup_correction.receipt.json"
)
V21_CLEANUP_RECEIPT_SHA256 = "3e68466eda1b22954bd7e81b132ea2b6decb7b72e0e745ed88d82ce4e1bb8988"
V21_CLEANUP_SIDECAR_SHA256 = "43bff7f7cafae1f2dc2b8cde21da2c4080eb0738f27d919f0f3d1295def66279"
V21_CLOSURE_PATH = V21_CLEANUP_RECEIPT_PATH.parent / "closure_receipt.json"
V21_CLOSURE_SHA256 = "8b2c5ed3ea1b7eeab130838f7c92a620549e5ffe50a52752a710360d8bd9875c"
V21_CLOSURE_SIDECAR_SHA256 = "acfc839d02f1c96db7e642253700d9e157f29977135a7f2489a649e1a5e528be"
V21_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v21_formal_candidate.yaml"
V21_PROFILE_SHA256 = "9f6e574ba0fe7c927eecdd612f948352dc2bf986b576c4e4c52708e0bfcd013a"
V21_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v21_formal.py"
V21_AUTHORIZER_SHA256 = "ad7afef771a985b57d165b508f7433b7ebe41e74e8290dfb275a7e122ec5ed81"
V21_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v21_formal.py"
V21_LAUNCHER_SHA256 = "fe30164f05cc902fdc0c0bc34bcbc436af37549329461e7ebdc2eeb5be5be0e4"

V22_SUCCESS_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v22_synthetic_success_20260824T023200Z_agentv22/synthetic_gate.receipt.json"
)
V22_SUCCESS_RECEIPT_SHA256 = "87a054210b36d6015272a177d6c1d86f4535bee3334a6b1bf6ee06bf595899d0"
V22_SUCCESS_SIDECAR_SHA256 = "0537e9b3db974af501fc1f3f11a8fef60ea4647c90db66170e1f798980dcdce2"
V22_FAILURE_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v22_synthetic_failure_20260824T023300Z_agentv22/synthetic_gate.receipt.json"
)
V22_FAILURE_RECEIPT_SHA256 = "73b37d869af9460c10d97e74c5e424bd831959eae1aea32eafcfbaf35088998f"
V22_FAILURE_SIDECAR_SHA256 = "178d2e0f83d42d91099615c12aab9e8618f95da624db93fe3ca9ee16c51721e9"

PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
EXPECTED_MESSAGES = 236687
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
SENSOR_DURATION_SECONDS = 579.278127298


class AuthorizationError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise AuthorizationError("SOURCE_NOT_REGULAR", "not a regular file: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_sealed(path: Path, expected_sha: str, expected_sidecar_sha: str,
                 label: str, *, sidecar_target_name: Optional[str] = None) -> Dict[str, Any]:
    if path.is_symlink() or not path.is_file() or path.stat().st_mode & 0o777 != 0o444:
        raise AuthorizationError("IMMUTABLE_EVIDENCE", "%s is not an immutable regular file" % label)
    observed = sha256_file(path)
    if observed != expected_sha:
        raise AuthorizationError("EVIDENCE_SHA", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    if sidecar.is_symlink() or not sidecar.is_file() or sidecar.stat().st_mode & 0o777 != 0o444:
        raise AuthorizationError("EVIDENCE_SIDECAR", "%s sidecar is not immutable" % label)
    target_name = sidecar_target_name or path.name
    if sha256_file(sidecar) != expected_sidecar_sha or \
            sidecar.read_bytes() != (expected_sha + "  " + target_name + "\n").encode("ascii"):
        raise AuthorizationError("EVIDENCE_SIDECAR", "%s sidecar drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("EVIDENCE_JSON", "%s JSON invalid" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("EVIDENCE_JSON", "%s must be an object" % label)
    return value


def _verify_gate(path: Path, sha: str, sidecar_sha: str, mode: str,
                 *, repo_root: Path = ROOT) -> Mapping[str, Any]:
    value = _read_sealed(path, sha, sidecar_sha, "v22 %s gate" % mode)
    if value.get("contract_version") != "m6a10-v22-synthetic-wrapper-persistence-v1" or \
            value.get("mode") != mode or value.get("image", {}).get("id") != V17_IMAGE_ID:
        raise AuthorizationError("V22_GATE_CONTRACT", "%s gate identity drift" % mode)
    expected_status = "PASS" if mode == "success" else "FAIL_CLOSED"
    if value.get("status") != expected_status or value.get("evidence", {}).get("status") != expected_status:
        raise AuthorizationError("V22_GATE_STATUS", "%s gate status drift" % mode)
    execution = value.get("execution", {})
    if execution.get("start_count") != 1 or execution.get("retry") is not False or \
            execution.get("manual_stop") is not False or execution.get("network") != "none" or \
            execution.get("rootfs") != "read_only" or execution.get("input_mount_count") != 0 or \
            execution.get("rw_mount_destination") != "/out":
        raise AuthorizationError("V22_GATE_EXECUTION", "%s gate execution drift" % mode)
    safety = value.get("safety", {})
    if any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved",
            "formal_replay_started")):
        raise AuthorizationError("V22_GATE_SAFETY", "%s gate safety drift" % mode)
    source = value.get("source", {})
    if source.get("v22_wrapper_path") != str(repo_root / "scripts/fast_livo2_m6a10_v22_formal_container_run.sh") or \
            source.get("v22_wrapper_sha256") != V22_WRAPPER_SHA256:
        raise AuthorizationError("V22_GATE_SOURCE", "%s gate wrapper binding drift" % mode)
    if mode == "success":
        evidence = value.get("evidence", {})
        if evidence.get("feeder_exit_status") != "0" or evidence.get("timing", {}).get("status") != "PASS" or \
                evidence.get("missing"):
            raise AuthorizationError("V22_GATE_SUCCESS", "success persistence evidence is incomplete")
    else:
        if value.get("evidence", {}).get("feeder_exit_status") != "17" or \
                not value.get("evidence", {}).get("missing"):
            raise AuthorizationError("V22_GATE_FAILURE", "failure persistence evidence is incomplete")
    return value


def verify_lineage(*, repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify all additive evidence without probing runtime state."""
    for path, sha, label in (
            (V22_WRAPPER_PATH, V22_WRAPPER_SHA256, "v22 wrapper"),
            (V22_GATE_PATH, V22_GATE_SHA256, "v22 gate"),
            (V22_GATE_TEST_PATH, V22_GATE_TEST_SHA256, "v22 gate test"),
            (V17_BUILD_RECEIPT_PATH, V17_BUILD_RECEIPT_SHA256, "v17 build receipt"),
            (V17_NO_INPUT_RECEIPT_PATH, V17_NO_INPUT_RECEIPT_SHA256, "v17 no-input receipt"),
            (V21_CLEANUP_RECEIPT_PATH, V21_CLEANUP_RECEIPT_SHA256, "v21 cleanup correction"),
            (V21_CLOSURE_PATH, V21_CLOSURE_SHA256, "v21 closure"),
            (V21_PROFILE_PATH, V21_PROFILE_SHA256, "v21 profile"),
            (V21_AUTHORIZER_PATH, V21_AUTHORIZER_SHA256, "v21 authorizer"),
            (V21_LAUNCHER_PATH, V21_LAUNCHER_SHA256, "v21 launcher")):
        observed = sha256_file(path)
        if observed != sha:
            raise AuthorizationError("LINEAGE_SHA", "%s drift" % label)
    build = _read_sealed(V17_BUILD_RECEIPT_PATH, V17_BUILD_RECEIPT_SHA256,
                         V17_BUILD_SIDECAR_SHA256, "v17 build receipt")
    no_input = _read_sealed(V17_NO_INPUT_RECEIPT_PATH, V17_NO_INPUT_RECEIPT_SHA256,
                            V17_NO_INPUT_SIDECAR_SHA256, "v17 no-input receipt")
    cleanup = _read_sealed(V21_CLEANUP_RECEIPT_PATH, V21_CLEANUP_RECEIPT_SHA256,
                           V21_CLEANUP_SIDECAR_SHA256, "v21 cleanup correction",
                           sidecar_target_name=str(V21_CLEANUP_RECEIPT_PATH))
    closure = _read_sealed(V21_CLOSURE_PATH, V21_CLOSURE_SHA256,
                           V21_CLOSURE_SIDECAR_SHA256, "v21 closure")
    if build.get("status") != "PASS" or no_input.get("status") != "PASS" or \
            cleanup.get("status") != "PASS" or cleanup.get("action", {}).get("force") is not False or \
            closure.get("status") != "FAIL_CLOSED":
        raise AuthorizationError("LINEAGE_CONTRACT", "immutable v17/v21 lineage contract drift")
    success = _verify_gate(V22_SUCCESS_RECEIPT_PATH, V22_SUCCESS_RECEIPT_SHA256,
                           V22_SUCCESS_SIDECAR_SHA256, "success", repo_root=repo_root)
    failure = _verify_gate(V22_FAILURE_RECEIPT_PATH, V22_FAILURE_RECEIPT_SHA256,
                           V22_FAILURE_SIDECAR_SHA256, "failure", repo_root=repo_root)
    return {"v17_build": build, "v17_no_input": no_input, "v21_cleanup": cleanup,
            "v21_closure": closure, "v22_success": success, "v22_failure": failure}


def verify_authorization(path: Optional[Path] = None, *, expected_sha256: Optional[str] = None,
                         attempt_root: Optional[Path] = None) -> Mapping[str, Any]:
    """Reject every authorization until an additive v22 receipt is installed."""
    del path, expected_sha256, attempt_root
    raise AuthorizationError("AUTHORIZATION_NOT_INSTALLED",
                             "v22 formal authorization is intentionally not installed")
