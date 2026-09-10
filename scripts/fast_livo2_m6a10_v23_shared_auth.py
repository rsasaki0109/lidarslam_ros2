#!/usr/bin/env python3
"""Shared, production-shaped authorization verification for the v23 candidate.

The v22 candidate deliberately kept its production authorizer inert.  v23 does
not patch that immutable module and does not duplicate its sentinel.  Instead,
this module is the one receipt-policy boundary used by the v23 launcher.  The
sealed v22 authorization is read through the existing v19 adapter/verifier
primitives, while the v22/v19 source and lineage identities are imported from
their real repository modules.

This module is verification-only: it never opens the input bag, starts Docker
or ROS, invokes a scorer, or saves a map.
"""

from __future__ import annotations

import hashlib
import importlib
import os
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v19_formal as v19_verifier  # noqa: E402
import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v22_formal as v22_authorizer  # noqa: E402
import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as shared_adapter  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v19_formal as v19_launcher  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v22_formal as v22_launcher  # noqa: E402


SHARED_AUTH_CONTRACT = "m6a10-v23-shared-formal-authorization-v1"
V22_AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v22_formal_authorization2_20260824T041708Z_agentv22/"
    "formal_authorization.receipt.json"
)
V22_AUTHORIZATION_SHA256 = "73076240a13335040945c70e1c5b69287f7277e92e6d96948a444e2ec4ae66c4"
V22_AUTHORIZATION_SIDECAR_SHA256 = "ca069e5c95e99fe3249942e35a4dc21325b424ce85c1be85dffafda2dc333cc1"
V22_ATTEMPT_ROOT = (
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v22_formal_replay2_20260824T041708Z_agentv22formal"
)

V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v23_synthetic_runtime_authorization_20260824T044145Z_agentv23/"
    "synthetic_runtime_authorization.receipt.json"
)
V23_SYNTHETIC_AUTHORIZATION_SHA256 = "909b736466c28d1bad32412856984f4c5c7b75b82cae51a22d62911e717fec3b"
V23_SYNTHETIC_AUTHORIZATION_SIDECAR_SHA256 = "960aa2a813085b01c50b1fc2bec5f2e552c650bc8e3b203caddfdb2d4fa9933b"
V23_SYNTHETIC_RUNTIME_ROOT = (
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v23_synthetic_runtime_20260824T044145Z_agentv23"
)
V23_SYNTHETIC_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v23_synthetic_runtime.py"
V23_SYNTHETIC_AUTHORIZER_SHA256 = "3db08ba29a75173c7aad9cfd2b3751b22276d466707872229ade74e5717268ce"

V22_PRIOR_AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v22_formal_authorization_20260824T024547Z_agentv22/"
    "formal_authorization.receipt.json"
)
V22_PRIOR_AUTHORIZATION_SHA256 = "1eaded90ebf66b208a787cd34201b7433fde026f65b2fecd85e357a1a6ad819c"
V22_PRIOR_AUTHORIZATION_SIDECAR_SHA256 = "3de37754251f3aa9b40f199d6dd71141fa6de3de5dd159e2faeb7740045f2464"

V19_ADAPTER_PATH = ROOT / "scripts/fast_livo2_m6a10_v19_base_adapter.py"
V19_ADAPTER_SHA256 = "61b3396aa54079773aa7066e5652527e27ddaa46121650b711f61807d0c82846"
V19_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v19_formal.py"
V19_AUTHORIZER_SHA256 = "1b2a82e3de212911ab558c842a346bbeb73e85816143119b2e862d4daeaf25df"
V19_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v19_formal.py"
V19_LAUNCHER_SHA256 = "49cfb1cb323e06b17514987aa17dc14d52a509b700e625a6d65beddb839f5771"

V22_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v22_formal.py"
V22_AUTHORIZER_SHA256 = "ee13d3e4e685cfab5ef3986d26d96692a96ff7c7f864d9f4e9d0fa6e217957c0"
V22_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v22_formal.py"
V22_LAUNCHER_SHA256 = "35e5ed40f312e017e45197061e8be604613804152675c179ed288b5f1a79578d"
V22_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v22_formal_candidate.yaml"
V22_PROFILE_SHA256 = "0fdadfefb7c30b9eb910e68529e47eaf1cacdee559d64ee0f262483452aeda6d"
V22_WRAPPER_SHA256 = "6e74246d2b7ee60086f3b316b22c6ef69ca3f16307ffdecea3eae9ad2d794f67"
V22_SYNTHETIC_GATE_SHA256 = "950d115937ced346116c116cdcf25b0f70d05c8dea3052efbae7394b9e74930a"
V22_GATE_TEST_SHA256 = "0b03253e498acf75f8fa0050c4dba6188477f649fe1ae0d87cc7895671927dd7"
V22_FORMAL_TEST_SHA256 = "2d43b7847536c0321cf81d10f3fbebda96cdf512d41201a3aa71fc0121d7555c"

V17_IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
V17_IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"

PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
EXPECTED_MESSAGES = 236687
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
SENSOR_DURATION_SECONDS = 579.278127298


class SharedAuthorizationError(ValueError):
    """Stable fail-closed error emitted by the shared authorization boundary."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    return shared_adapter.sha256_file(path)


def _regular(path: Path, label: str, *, immutable: bool = False) -> None:
    try:
        shared_adapter._regular(path, label, immutable=immutable)
    except Exception as exc:
        raise SharedAuthorizationError(getattr(exc, "kind", "NOT_REGULAR"), str(exc)) from exc


def _read_sealed(path: Path, expected_sha: str, expected_sidecar_sha: str,
                 label: str) -> Dict[str, Any]:
    """Use the v19 verifier's immutable receipt reader, never a local copy."""
    try:
        value = v19_verifier._read_receipt(path, expected_sha, expected_sidecar_sha, label)
    except Exception as exc:
        raise SharedAuthorizationError(getattr(exc, "kind", "RECEIPT_INVALID"), str(exc)) from exc
    if not isinstance(value, dict):
        raise SharedAuthorizationError("RECEIPT_JSON", "%s is not an object" % label)
    return value


def _expected_source_sha() -> Dict[str, str]:
    return {
        "v19_adapter": V19_ADAPTER_SHA256,
        "v19_authorizer": V19_AUTHORIZER_SHA256,
        "v19_launcher": V19_LAUNCHER_SHA256,
        "v22_authorizer": V22_AUTHORIZER_SHA256,
        "v22_launcher": V22_LAUNCHER_SHA256,
        "v22_profile": V22_PROFILE_SHA256,
        "v22_wrapper": V22_WRAPPER_SHA256,
        "v22_synthetic_gate": V22_SYNTHETIC_GATE_SHA256,
        "v22_gate_test": V22_GATE_TEST_SHA256,
        "v22_formal_test": V22_FORMAL_TEST_SHA256,
    }


def _expected_v22_receipt_source_sha() -> Dict[str, str]:
    """The source map emitted by the immutable v22 authorization receipt."""
    return {
        "v19_adapter": V19_ADAPTER_SHA256,
        "v19_launcher": V19_LAUNCHER_SHA256,
        "v22_authorizer": V22_AUTHORIZER_SHA256,
        "v22_formal_test": V22_FORMAL_TEST_SHA256,
        "v22_gate_test": V22_GATE_TEST_SHA256,
        "v22_launcher": V22_LAUNCHER_SHA256,
        "v22_profile": V22_PROFILE_SHA256,
        "v22_synthetic_gate": V22_SYNTHETIC_GATE_SHA256,
        "v22_wrapper": V22_WRAPPER_SHA256,
    }


def _validate_window_document(document: Mapping[str, Any], index: int) -> None:
    if document.get("schema_version") != 1 or document.get("contract_version") != "m6a10-quiescence-v1" or \
            document.get("status") != "PASS" or document.get("runner_start_allowed") is not True or \
            document.get("ground_truth_content_opened") is not False or document.get("scorer_invoked") is not False:
        raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "window %d contract drift" % index)
    observation = document.get("observation")
    if not isinstance(observation, Mapping) or observation.get("forbidden_processes") or \
            observation.get("checks", {}).get("cpu_busy_within_limit") is not True or \
            observation.get("checks", {}).get("load1_per_cpu_within_limit") is not True or \
            observation.get("checks", {}).get("no_forbidden_processes") is not True:
        raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "window %d is not clean PASS" % index)


def _validate_v22_document(value: Mapping[str, Any], *, expected_attempt_root: str) -> None:
    """Validate the exact sealed v22 authorization document shape."""
    if value.get("schema_version") != 1 or value.get("contract_version") != v22_authorizer.AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_replay_authorized") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False or value.get("formal_execution") is not False:
        raise SharedAuthorizationError("AUTHORIZATION_STATUS", "v22 receipt status drift")
    if value.get("attempt_root") != expected_attempt_root or \
            value.get("authorization_root") != str(V22_AUTHORIZATION_RECEIPT_PATH.parent):
        raise SharedAuthorizationError("AUTHORIZATION_ROOT", "v22 exact root drift")
    if value.get("attempt_root_precondition") != {"absent_before_gate": True, "created": False, "symlink": False} or \
            value.get("authorization_root_precondition") != {"absent_before_gate": True, "created": True, "symlink": False}:
        raise SharedAuthorizationError("AUTHORIZATION_ROOT", "v22 root precondition drift")
    execution = value.get("execution")
    if execution != {"bag_opened": False, "container_started": False, "gt_scoring_map": False,
                     "manual_stop": False, "one_start": False, "popen_count": 0, "retry": False}:
        raise SharedAuthorizationError("AUTHORIZATION_EXECUTION", "v22 execution state drift")
    if value.get("image") != {"formal_replay_forbidden": False, "id": V17_IMAGE_ID,
                               "tag": V17_IMAGE_TAG}:
        raise SharedAuthorizationError("AUTHORIZATION_IMAGE", "v22 image identity drift")
    if value.get("phase") != {
            "contract": PHASE_CONTRACT, "expected_messages": EXPECTED_MESSAGES,
            "expected_topic_counts": EXPECTED_COUNTS,
            "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
            "sensor_duration_seconds": SENSOR_DURATION_SECONDS,
            "transport_contract": TRANSPORT_CONTRACT}:
        raise SharedAuthorizationError("AUTHORIZATION_PHASE", "v22 phase contract drift")
    if value.get("window_contract") != {
            "continuous": True, "count": 3, "sample_seconds": 4.0,
            "max_cpu_busy_percent": 5.0, "max_load1_per_cpu": 0.5,
            "runner_start_allowed": True, "no_retry": True}:
        raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "v22 window contract drift")
    if value.get("monitor") != {"continuous": True, "interval_seconds": 4.0,
                                  "natural_completion_required": True,
                                  "host_interference_stop": False}:
        raise SharedAuthorizationError("AUTHORIZATION_MONITOR", "v22 monitor contract drift")
    if value.get("mount_contract") != {
            "feeder_readonly": True, "ground_truth_mount": False, "input_mounts": 1,
            "map_mount": False, "network": "none", "output": "/out",
            "output_readonly": False, "rootfs": "read_only", "scorer_mount": False,
            "wrapper_readonly": True}:
        raise SharedAuthorizationError("AUTHORIZATION_MOUNTS", "v22 mount contract drift")
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "formal_replay_started", "ground_truth_content_opened", "input_opened",
            "map_saved", "scorer_invoked")):
        raise SharedAuthorizationError("AUTHORIZATION_SAFETY", "v22 safety flags drift")
    if value.get("source_sha256") != _expected_v22_receipt_source_sha():
        raise SharedAuthorizationError("AUTHORIZATION_SOURCE", "v22 source binding drift")
    if value.get("lineage_verification") != {
            "v17_build": "PASS", "v17_no_input": "PASS", "v21_cleanup": "PASS",
            "v21_closure": "FAIL_CLOSED", "v22_failure": "FAIL_CLOSED", "v22_success": "PASS"}:
        raise SharedAuthorizationError("AUTHORIZATION_LINEAGE", "v22 lineage status drift")
    previous = value.get("previous_authorization")
    if not isinstance(previous, Mapping) or previous.get("path") != str(V22_PRIOR_AUTHORIZATION_RECEIPT_PATH) or \
            previous.get("receipt_sha256") != V22_PRIOR_AUTHORIZATION_SHA256 or \
            previous.get("sidecar_path") != str(V22_PRIOR_AUTHORIZATION_RECEIPT_PATH.with_name(
                V22_PRIOR_AUTHORIZATION_RECEIPT_PATH.name + ".sha256")) or \
            previous.get("sidecar_sha256") != V22_PRIOR_AUTHORIZATION_SIDECAR_SHA256 or \
            previous.get("status") != "FAIL_CLOSED" or previous.get("failure_kind") != "HOST_QUIESCENCE_CPU_LIMIT" or \
            previous.get("formal_replay_started") is not False:
        raise SharedAuthorizationError("AUTHORIZATION_LINEAGE", "v22 prior authorization drift")
    windows = value.get("windows")
    if not isinstance(windows, list) or len(windows) != 3:
        raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "v22 must contain exactly three windows")
    seen_paths = set()
    seen_hashes = set()
    for index, item in enumerate(windows, 1):
        expected_path = V22_AUTHORIZATION_RECEIPT_PATH.parent / (
            "quiescence_window_%02d.receipt.json" % index)
        expected_sidecar = expected_path.with_name(expected_path.name + ".sha256")
        if not isinstance(item, Mapping) or item.get("index") != index or item.get("path") != str(expected_path) or \
                item.get("sidecar_path") != str(expected_sidecar) or item.get("status") != "PASS" or \
                item.get("runner_start_allowed") is not True or item.get("forbidden_processes"):
            raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "window %d summary drift" % index)
        _regular(expected_path, "window %d" % index, immutable=True)
        _regular(expected_sidecar, "window %d sidecar" % index, immutable=True)
        if sha256_file(expected_path) != item.get("sha256") or sha256_file(expected_sidecar) != item.get("sidecar_sha256") or \
                expected_sidecar.read_bytes() != (item.get("sha256") + "  " + expected_path.name + "\n").encode("ascii"):
            raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "window %d hash drift" % index)
        document = _read_sealed(expected_path, item["sha256"], item["sidecar_sha256"],
                                "window %d" % index)
        _validate_window_document(document, index)
        seen_paths.add(str(expected_path))
        seen_hashes.add(item["sha256"])
    if len(seen_paths) != 3 or len(seen_hashes) != 3:
        raise SharedAuthorizationError("AUTHORIZATION_WINDOW", "windows are not distinct")


def verify_v22_authorization(*, path: Path = V22_AUTHORIZATION_RECEIPT_PATH,
                             expected_sha256: str = V22_AUTHORIZATION_SHA256,
                             expected_attempt_root: str = V22_ATTEMPT_ROOT,
                             repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify the immutable v22 receipt through the v19 reader primitives."""
    if path.resolve() != V22_AUTHORIZATION_RECEIPT_PATH.resolve():
        raise SharedAuthorizationError("AUTHORIZATION_PATH", "v22 authorization path drift")
    if expected_attempt_root != V22_ATTEMPT_ROOT:
        raise SharedAuthorizationError("AUTHORIZATION_ROOT", "v22 attempt root is not the pinned root")
    value = _read_sealed(path, expected_sha256, V22_AUTHORIZATION_SIDECAR_SHA256,
                         "v22 authorization")
    _validate_v22_document(value, expected_attempt_root=expected_attempt_root)
    prior = _read_sealed(V22_PRIOR_AUTHORIZATION_RECEIPT_PATH, V22_PRIOR_AUTHORIZATION_SHA256,
                         V22_PRIOR_AUTHORIZATION_SIDECAR_SHA256, "v22 prior authorization")
    if prior.get("status") != "FAIL_CLOSED" or prior.get("failure_kind") != "HOST_QUIESCENCE_CPU_LIMIT" or \
            prior.get("formal_replay_started") is not False:
        raise SharedAuthorizationError("AUTHORIZATION_LINEAGE", "v22 prior failure is not admissible")
    return {
        "status": value["status"],
        "authorized": value["authorized"],
        "formal_replay_started": value["formal_replay_started"],
        "receipt_path": str(path),
        "receipt_sha256": expected_sha256,
        "sidecar_path": str(path.with_name(path.name + ".sha256")),
        "sidecar_sha256": V22_AUTHORIZATION_SIDECAR_SHA256,
        "attempt_root": expected_attempt_root,
        "previous_failure": {"path": str(V22_PRIOR_AUTHORIZATION_RECEIPT_PATH),
                              "sha256": V22_PRIOR_AUTHORIZATION_SHA256,
                              "failure_kind": prior["failure_kind"]},
    }


def verify_synthetic_authorization(*, path: Path = V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH,
                                   expected_sha256: str = V23_SYNTHETIC_AUTHORIZATION_SHA256,
                                   expected_runtime_root: str = V23_SYNTHETIC_RUNTIME_ROOT,
                                   repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify the separate synthetic-runtime authorization domain.

    This receipt intentionally cannot satisfy ``verify_v22_authorization``:
    its contract and path are different, and ``formal_replay_authorized`` is
    false.  It reuses the already sealed v22 windows as read-only fixtures.
    """
    if path.resolve() != V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH.resolve():
        raise SharedAuthorizationError("SYNTHETIC_AUTH_PATH", "synthetic authorization path drift")
    if expected_runtime_root != V23_SYNTHETIC_RUNTIME_ROOT:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_ROOT", "synthetic runtime root drift")
    if expected_sha256.startswith("__"):
        raise SharedAuthorizationError("SYNTHETIC_AUTH_SHA", "synthetic authorization SHA is not bound")
    # The three PASS windows and all current v19/v22 source identities are
    # checked again through the formal-domain fixture, but never authorize it.
    formal_fixture = verify_v22_authorization(repo_root=repo_root)
    value = _read_sealed(path, expected_sha256, V23_SYNTHETIC_AUTHORIZATION_SIDECAR_SHA256,
                         "v23 synthetic authorization")
    _validate_synthetic_domain_fields(value, expected_runtime_root=expected_runtime_root)
    if value.get("authorization_root") != str(path.parent) or \
            value.get("runtime_root") != expected_runtime_root or \
            value.get("formal_attempt_root") != V22_ATTEMPT_ROOT:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_ROOT", "synthetic exact roots drift")
    if value.get("image") != {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID}:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_IMAGE", "synthetic image identity drift")
    if value.get("window_contract") != {
            "fixture_domain": "v22_authorized_windows",
            "continuous": True, "count": 3, "sample_seconds": 4.0,
            "runner_start_allowed": True}:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_WINDOW", "synthetic window contract drift")
    if value.get("execution") != {
            "container_start_count": 1, "input_mount_count": 0,
            "network": "none", "rootfs": "read_only", "output": "/out",
            "output_readonly": False, "output_tmpfs": False,
            "retry": False, "manual_stop": False}:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_EXECUTION", "synthetic execution contract drift")
    if value.get("safety") != {
            "formal_replay_started": False, "input_opened": False,
            "ground_truth_content_opened": False, "scorer_invoked": False,
            "map_saved": False}:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_SAFETY", "synthetic safety flags drift")
    source = value.get("source")
    if not isinstance(source, Mapping) or source.get("v22_authorized_receipt_sha256") != V22_AUTHORIZATION_SHA256 or \
            source.get("v22_authorized_sidecar_sha256") != V22_AUTHORIZATION_SIDECAR_SHA256 or \
            source.get("v22_wrapper_sha256") != V22_WRAPPER_SHA256 or \
            source.get("v19_adapter_sha256") != V19_ADAPTER_SHA256:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_SOURCE", "synthetic source binding drift")
    windows = value.get("windows")
    formal_windows = _read_sealed(V22_AUTHORIZATION_RECEIPT_PATH, V22_AUTHORIZATION_SHA256,
                                  V22_AUTHORIZATION_SIDECAR_SHA256, "v22 formal fixture").get("windows")
    if windows != formal_windows:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_WINDOW", "synthetic window fixture drift")
    return {
        "status": value["status"], "authorized": True,
        "formal_replay_started": value["formal_replay_started"],
        "gate_kind": value["gate_kind"], "runtime_root": expected_runtime_root,
        "receipt_path": str(path), "receipt_sha256": expected_sha256,
        "sidecar_path": str(path.with_name(path.name + ".sha256")),
        "sidecar_sha256": V23_SYNTHETIC_AUTHORIZATION_SIDECAR_SHA256,
        "formal_fixture": formal_fixture,
    }


def _validate_synthetic_domain_fields(value: Mapping[str, Any], *, expected_runtime_root: str) -> None:
    """Validate domain/status/root fields independently of file I/O."""
    if value.get("schema_version") != 1 or \
            value.get("contract_version") != "m6a10-v23-synthetic-runtime-authorization-v1" or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("gate_kind") != "synthetic_runtime_only" or \
            value.get("formal_replay_authorized") is not False or \
            value.get("formal_replay_forbidden") is not True or \
            value.get("formal_replay_started") is not False or \
            value.get("runtime_execution_authorized") is not True:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_STATUS", "synthetic authorization domain drift")
    if value.get("runtime_root") != expected_runtime_root:
        raise SharedAuthorizationError("SYNTHETIC_AUTH_ROOT", "synthetic runtime root drift")


def verify_candidate_identity(*, repo_root: Path = ROOT, launcher_path: Optional[Path] = None) -> Dict[str, Any]:
    """Import actual v19/v22 modules and verify their pinned production surface."""
    if repo_root.resolve() != ROOT.resolve():
        raise SharedAuthorizationError("REPO_ROOT", "v23 repo root drift")
    paths = {
        "v19_adapter": V19_ADAPTER_PATH,
        "v19_authorizer": V19_AUTHORIZER_PATH,
        "v19_launcher": V19_LAUNCHER_PATH,
        "v22_authorizer": V22_AUTHORIZER_PATH,
        "v22_launcher": V22_LAUNCHER_PATH,
        "v22_profile": V22_PROFILE_PATH,
    }
    expected = _expected_source_sha()
    for key, path in paths.items():
        _regular(path, key)
        if sha256_file(path) != expected[key]:
            raise SharedAuthorizationError("SOURCE_DRIFT", "%s SHA drift" % key)
    if Path(str(vars(v19_verifier)["__file__"])).resolve() != V19_AUTHORIZER_PATH.resolve() or \
            Path(str(vars(v19_launcher)["__file__"])).resolve() != V19_LAUNCHER_PATH.resolve() or \
            Path(str(vars(v22_authorizer)["__file__"])).resolve() != V22_AUTHORIZER_PATH.resolve() or \
            Path(str(vars(v22_launcher)["__file__"])).resolve() != V22_LAUNCHER_PATH.resolve():
        raise SharedAuthorizationError("MODULE_PATH", "v19/v22 imported module path drift")
    surface = shared_adapter.load_base_surface()
    runtime = shared_adapter.load_v17_runtime_surface(surface)
    if runtime.image_id != V17_IMAGE_ID:
        raise SharedAuthorizationError("IMAGE_ID", "actual v17 runtime image drift")
    if launcher_path is not None:
        shared_adapter.assert_launcher_uses_adapter(launcher_path)
    lineage = v22_authorizer.verify_lineage(repo_root=repo_root)
    return {
        "v19_adapter": {"path": str(V19_ADAPTER_PATH), "sha256": V19_ADAPTER_SHA256},
        "v19_authorizer": {"path": str(V19_AUTHORIZER_PATH), "sha256": V19_AUTHORIZER_SHA256},
        "v19_launcher": {"path": str(V19_LAUNCHER_PATH), "sha256": V19_LAUNCHER_SHA256},
        "v22_authorizer": {"path": str(V22_AUTHORIZER_PATH), "sha256": V22_AUTHORIZER_SHA256},
        "v22_launcher": {"path": str(V22_LAUNCHER_PATH), "sha256": V22_LAUNCHER_SHA256},
        "v22_profile": {"path": str(V22_PROFILE_PATH), "sha256": V22_PROFILE_SHA256},
        "base_surface": surface.as_identity(),
        "v17_runtime": runtime.as_identity(),
        "v22_lineage_status": {key: value.get("status") for key, value in lineage.items()},
    }


def validate_runtime_argv(argv: Sequence[str]) -> None:
    """Reuse the v22 injected argv gate and require the v23 isolation subset."""
    try:
        v22_launcher._validate_injected_argv(argv)
    except Exception as exc:
        raise SharedAuthorizationError(getattr(exc, "kind", "ARGV_INVALID"), str(exc)) from exc
    if not isinstance(argv, (list, tuple)) or argv.count("--mount") != 2:
        raise SharedAuthorizationError("ARGV_MOUNTS", "v23 requires exactly two test mounts")
    mounts = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]
    destinations = [field[4:] for mount in mounts for field in mount.split(",") if field.startswith("dst=")]
    if destinations != ["/runner/v22_runtime.sh", "/out"] or len(set(destinations)) != 2:
        raise SharedAuthorizationError("ARGV_MOUNTS", "v23 mount destinations drift")
    if "--rm" in argv or any("/input" in mount for mount in mounts):
        raise SharedAuthorizationError("ARGV_FORBIDDEN", "v23 argv contains an unsafe surface")
