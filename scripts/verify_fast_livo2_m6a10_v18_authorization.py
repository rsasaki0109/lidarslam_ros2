#!/usr/bin/env python3
"""Shared, read-only verifier for the additive v18 authorization candidate.

This module is the only production authorization verifier used by the v18
launcher and v18 authorizer.  It accepts the sealed v17 authorization as a
lineage fixture, but it does not authorize a reused attempt root.  No Docker,
ROS, bag, ground-truth, scoring, or map operation is performed here.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


ROOT = Path(__file__).resolve().parents[1]
AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_formal_authorization_20260823T230500Z_agentv17/"
    "formal_authorization.receipt.json"
)
AUTHORIZATION_RECEIPT_SHA256 = "7304f81e70c0ece68d49fd24de05862d1c5cdac36fd041acb5e709df78ef3c83"
AUTHORIZATION_SIDECAR_SHA256 = "8bcb71b86cb0d27af5e34970d30b190b1afd7269e9cd73f944aebfde7057b429"
AUTHORIZATION_CONTRACT = "m6a10-v17-formal-exact-root-authorization-v1"
V18_AUTHORIZATION_CONTRACT = "m6a10-v18-formal-exact-root-authorization-v1"
QUIESCENCE_CONTRACT = "m6a10-quiescence-v1"
# The first v18 authorization is an immutable failed-preflight lineage.  A
# second authorization must never overwrite or reuse it; it is checked as a
# sealed input to the new candidate lineage below.
PREVIOUS_V18_AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_authorization_20260823T234000Z_agentv18/"
    "formal_authorization.receipt.json"
)
PREVIOUS_V18_AUTHORIZATION_SHA256 = "523fcb57d42099fd91e4551ab08952f7daf0f0a33fabb43dfaab4dc9b7542a95"
PREVIOUS_V18_AUTHORIZATION_SIDECAR_SHA256 = "870737cfe0e7cbc76135dfa279884f514614d320dc40985bc88498f374b52d66"
PREVIOUS_V18_CLOSURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_replay_20260823T234000Z_agentv18formal/"
    "closure_receipt.json"
)
PREVIOUS_V18_CLOSURE_SHA256 = "c4b068ebbfedea4727776c2785ee6d40df4baa17d3cadc60283fa2cf15bc5145"
PREVIOUS_V18_CLOSURE_SIDECAR_SHA256 = "0095812aa7bc34bb274c5711d38f95c0e8914a0b5d75382a1e8d829cd6445be5"
V18_AUTHORIZATION_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_authorization_20260824T003000Z_agentv18b/"
    "formal_authorization.receipt.json"
)
V18_ATTEMPT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_replay_20260824T003000Z_agentv18bformal"
)

V17_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v17_formal.py"
V17_LAUNCHER_SHA256 = "a94a5f2e5da69cde0195572a8b1ab9b5357634604ace15fe098f67307d0244d7"
V17_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v17_formal.py"
V17_AUTHORIZER_SHA256 = "55abdb0551a5e67215597e95a888c5a1a3baa370cbba1ad84f6ab3efeb0e55c4"
V17_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_ready.yaml"
V17_PROFILE_SHA256 = "10ddffb6eed647eb3b7a2ee7ede49505923755eb917d8b87737675853e35d9c5"
IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"

V17_BUILD_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/"
    "build_identity.receipt.json"
)
V17_BUILD_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
V17_BUILD_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
V17_NO_INPUT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/"
    "no_input.receipt.json"
)
V17_NO_INPUT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
V17_NO_INPUT_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"
V17_NO_INPUT_FAILURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction_20260823T224017Z_agentv17/"
    "no_input.receipt.json"
)
V17_NO_INPUT_FAILURE_SHA256 = "93296e2d47e37aeea739daf3c44d4c7bf988c7a9bcfc61f11a4cbab31ec1203a"
V17_NO_INPUT_FAILURE_SIDECAR_SHA256 = "a0db9e7c7303974edc575033088bc92e0bc14ab8db7eda054ae16c1ec4d9c2f7"
V17_PREFLIGHT_CLOSURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_formal_replay_20260823T230500Z_agentv17formal/"
    "closure_receipt.json"
)
V17_PREFLIGHT_CLOSURE_SHA256 = "d52df67a0f28bf778ae956f5fdf2ddb875d50fcee3af1e1d6c3c1911201bf662"
V18_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v18_formal.py"
V18_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v18_formal.py"
V18_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v18_formal_candidate.yaml"

EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
EXPECTED_MESSAGES = 236687
INPUT_PATH = "/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
INPUT_BYTES = 11290464091
INPUT_SHA256 = "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310"
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
SENSOR_DURATION_SECONDS = 579.278127298
WATCHDOG_SECONDS = 1200


class AuthorizationError(ValueError):
    """Stable fail-closed verification error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, immutable: bool = False) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if not os.path.lexists(path) or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not a regular file" % label)
    if immutable and (path.stat().st_mode & 0o777) != 0o444:
        raise AuthorizationError("IMMUTABILITY", "%s is not mode 0444" % label)


def _sidecar(path: Path, file_sha: str, expected_sha: Optional[str], label: str) -> str:
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label, immutable=True)
    observed = sha256_file(sidecar)
    if expected_sha is not None and observed != expected_sha:
        raise AuthorizationError("RECEIPT_SIDECAR_DRIFT", "%s sidecar SHA drift" % label)
    expected_line = (file_sha + "  " + path.name + "\n").encode("ascii")
    if sidecar.read_bytes() != expected_line:
        raise AuthorizationError("RECEIPT_SIDECAR_CONTENT", "%s sidecar content drift" % label)
    return observed


def _json_file(path: Path, expected_sha: str, sidecar_sha: Optional[str], label: str) -> Dict[str, Any]:
    _regular(path, label, immutable=True)
    observed = sha256_file(path)
    if observed != expected_sha:
        raise AuthorizationError("RECEIPT_DRIFT", "%s SHA drift" % label)
    _sidecar(path, observed, sidecar_sha, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_INVALID", "%s is not JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s must be an object" % label)
    return value


def _source(path: Path, expected_sha: str, label: str) -> None:
    _regular(path, label)
    if sha256_file(path) != expected_sha:
        raise AuthorizationError("SOURCE_DRIFT", "%s SHA drift" % label)


def _false_safety(value: Mapping[str, Any], label: str) -> None:
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(
            safety.get(key) is not False
            for key in ("input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "%s safety fields are not false" % label)


def _verify_lineage(repo_root: Path) -> Dict[str, Any]:
    _source(repo_root / V17_LAUNCHER_PATH.relative_to(ROOT), V17_LAUNCHER_SHA256, "v17 launcher")
    _source(repo_root / V17_AUTHORIZER_PATH.relative_to(ROOT), V17_AUTHORIZER_SHA256, "v17 authorizer")
    _source(repo_root / V17_PROFILE_PATH.relative_to(ROOT), V17_PROFILE_SHA256, "v17 profile")

    build = _json_file(V17_BUILD_PATH, V17_BUILD_SHA256, V17_BUILD_SIDECAR_SHA256, "v17 build")
    if build.get("status") != "PASS" or build.get("image", {}).get("id") != IMAGE_ID or \
            build.get("image", {}).get("tag") != IMAGE_TAG or \
            build.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("BUILD_LINEAGE", "v17 build identity or safety drift")
    no_input = _json_file(V17_NO_INPUT_PATH, V17_NO_INPUT_SHA256,
                          V17_NO_INPUT_SIDECAR_SHA256, "v17 no-input PASS")
    runtime = no_input.get("runtime")
    if no_input.get("status") != "PASS" or no_input.get("image", {}).get("id") != IMAGE_ID or \
            not isinstance(runtime, Mapping) or runtime.get("start_count") != 1 or \
            runtime.get("network") != "none" or runtime.get("rootfs") != "read_only" or \
            runtime.get("host_mount_count") != 0 or runtime.get("oom_killed") is not False or \
            runtime.get("cleanup") != "stopped_only_remove_success":
        raise AuthorizationError("NO_INPUT_LINEAGE", "v17 no-input identity or safety drift")
    _false_safety(no_input, "v17 no-input")
    prior = _json_file(V17_NO_INPUT_FAILURE_PATH, V17_NO_INPUT_FAILURE_SHA256,
                       V17_NO_INPUT_FAILURE_SIDECAR_SHA256, "v17 prior no-input failure")
    if prior.get("status") != "FAIL_CLOSED":
        raise AuthorizationError("FAILURE_LINEAGE", "v17 prior failure drift")
    closure = _json_file(V17_PREFLIGHT_CLOSURE_PATH, V17_PREFLIGHT_CLOSURE_SHA256,
                         None, "v17 preflight closure")
    if closure.get("status") != "FAIL_CLOSED" or \
            closure.get("failure_kind") != "V16_FORMAL_REPLAY_UNAUTHORIZED" or \
            closure.get("execution", {}).get("one_start") is not False:
        raise AuthorizationError("PREFLIGHT_LINEAGE", "v17 pre-Popen failure closure drift")
    _false_safety(closure, "v17 preflight closure")
    return {
        "candidate": {"launcher_path": str(V17_LAUNCHER_PATH), "launcher_sha256": V17_LAUNCHER_SHA256,
                      "profile_path": str(V17_PROFILE_PATH), "profile_sha256": V17_PROFILE_SHA256},
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "build": {"path": str(V17_BUILD_PATH), "sha256": V17_BUILD_SHA256},
        "no_input": {"path": str(V17_NO_INPUT_PATH), "sha256": V17_NO_INPUT_SHA256},
        "preflight_failure": {"path": str(V17_PREFLIGHT_CLOSURE_PATH),
                               "sha256": V17_PREFLIGHT_CLOSURE_SHA256},
    }


def _verify_previous_v18_failure() -> Dict[str, Any]:
    """Verify the immutable first v18 preflight failure before reauthorization."""
    authorization = _json_file(
        PREVIOUS_V18_AUTHORIZATION_RECEIPT_PATH,
        PREVIOUS_V18_AUTHORIZATION_SHA256,
        PREVIOUS_V18_AUTHORIZATION_SIDECAR_SHA256,
        "previous v18 authorization failure",
    )
    if authorization.get("status") != "FAIL_CLOSED" or \
            authorization.get("authorized") is not False or \
            authorization.get("formal_execution") is not False or \
            authorization.get("formal_replay_started") is not False or \
            authorization.get("failure_kind") != "QUIESCENCE_FAIL_CLOSED":
        raise AuthorizationError("PREVIOUS_V18_FAILURE", "previous v18 authorization failure drift")
    closure = _json_file(
        PREVIOUS_V18_CLOSURE_PATH,
        PREVIOUS_V18_CLOSURE_SHA256,
        PREVIOUS_V18_CLOSURE_SIDECAR_SHA256,
        "previous v18 pre-Popen closure",
    )
    execution = closure.get("execution")
    if closure.get("status") != "FAIL_CLOSED" or \
            closure.get("failure_kind") != "AUTHORIZATION_QUIESCENCE" or \
            not isinstance(execution, Mapping) or \
            execution.get("formal_replay_started") is not False or \
            execution.get("popen_count") != 0 or \
            execution.get("one_start") is not False or \
            closure.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("PREVIOUS_V18_FAILURE", "previous v18 closure drift")
    return {
        "authorization": {
            "path": str(PREVIOUS_V18_AUTHORIZATION_RECEIPT_PATH),
            "sha256": PREVIOUS_V18_AUTHORIZATION_SHA256,
            "sidecar_sha256": PREVIOUS_V18_AUTHORIZATION_SIDECAR_SHA256,
            "failure_kind": authorization.get("failure_kind"),
            "formal_replay_started": False,
        },
        "closure": {
            "path": str(PREVIOUS_V18_CLOSURE_PATH),
            "sha256": PREVIOUS_V18_CLOSURE_SHA256,
            "sidecar_sha256": PREVIOUS_V18_CLOSURE_SIDECAR_SHA256,
            "failure_kind": closure.get("failure_kind"),
            "formal_replay_started": False,
            "popen_count": 0,
        },
    }


def build_v18_lineage(repo_root: Path = ROOT) -> Dict[str, Any]:
    """Collect the source/evidence bindings for one v18 authorization.

    This is read-only and does not create an authorization or inspect input.
    The authorizer calls it before sealing its receipt; the verifier recomputes
    the same values while validating that receipt.
    """
    predecessor = _verify_lineage(repo_root)
    previous_failure = _verify_previous_v18_failure()
    predecessor_root = Path(
        "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
        "fast_livo2_v2c_v17_formal_replay_20260823T230500Z_agentv10formal"
    )
    # The immutable v17 authorization is a fixture; its original attempt root
    # is intentionally not reused by v18.
    v17_authorization = verify_receipt_document(
        AUTHORIZATION_RECEIPT_PATH,
        AUTHORIZATION_RECEIPT_SHA256,
        Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v17_formal_replay_20260823T230500Z_agentv17formal"
        ),
        repo_root=repo_root,
        expected_receipt_path=AUTHORIZATION_RECEIPT_PATH,
        expected_sidecar_sha256=AUTHORIZATION_SIDECAR_SHA256,
        require_fresh_root=False,
    )
    del predecessor_root  # retained above only to make the non-reuse intent explicit
    _source(repo_root / V18_LAUNCHER_PATH.relative_to(ROOT),
            sha256_file(repo_root / V18_LAUNCHER_PATH.relative_to(ROOT)), "v18 launcher")
    _source(repo_root / V18_AUTHORIZER_PATH.relative_to(ROOT),
            sha256_file(repo_root / V18_AUTHORIZER_PATH.relative_to(ROOT)), "v18 authorizer")
    _source(repo_root / V18_PROFILE_PATH.relative_to(ROOT),
            sha256_file(repo_root / V18_PROFILE_PATH.relative_to(ROOT)), "v18 profile")
    return {
        "formal_replay_forbidden_lineage": True,
        "v17_authorization": {
            "path": str(AUTHORIZATION_RECEIPT_PATH),
            "sha256": AUTHORIZATION_RECEIPT_SHA256,
            "sidecar_sha256": AUTHORIZATION_SIDECAR_SHA256,
            "attempt_root": v17_authorization["attempt_root"],
        },
        "v17_lineage": predecessor,
        "v18_candidate": {
            "launcher_path": str(V18_LAUNCHER_PATH),
            "launcher_sha256": sha256_file(repo_root / V18_LAUNCHER_PATH.relative_to(ROOT)),
            "authorizer_path": str(V18_AUTHORIZER_PATH),
            "authorizer_sha256": sha256_file(repo_root / V18_AUTHORIZER_PATH.relative_to(ROOT)),
            "verifier_path": str(Path(__file__).resolve()),
            "verifier_sha256": sha256_file(Path(__file__).resolve()),
            "profile_path": str(V18_PROFILE_PATH),
            "profile_sha256": sha256_file(repo_root / V18_PROFILE_PATH.relative_to(ROOT)),
            "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        },
        "predecessor_failure": {
            "path": str(V17_PREFLIGHT_CLOSURE_PATH),
            "sha256": V17_PREFLIGHT_CLOSURE_SHA256,
            "failure_kind": "V16_FORMAL_REPLAY_UNAUTHORIZED",
            "formal_replay_started": False,
        },
        "previous_v18_failure": previous_failure,
    }


def _verify_windows(value: Mapping[str, Any], receipt_path: Path) -> None:
    windows = value.get("windows")
    if not isinstance(windows, list) or len(windows) != 3:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "exactly three quiescence windows are required")
    paths = []
    hashes = []
    for index, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != index or \
                item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or \
                item.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence window %d is not PASS" % index)
        path = Path(str(item.get("path", "")))
        expected_name = "quiescence_window_%02d.receipt.json" % index
        if path.parent.resolve() != receipt_path.parent.resolve() or path.name != expected_name:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence window path drift")
        _regular(path, "quiescence window %d" % index, immutable=True)
        try:
            window = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence window JSON invalid") from exc
        observation = window.get("observation")
        forbidden = observation.get("forbidden_processes", []) if isinstance(observation, Mapping) else None
        if window.get("schema_version") != 1 or window.get("contract_version") != QUIESCENCE_CONTRACT or \
                window.get("status") != "PASS" or window.get("runner_start_allowed") is not True or forbidden:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence window contract drift")
        observed_sha = sha256_file(path)
        if observed_sha != item.get("sha256"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence window SHA drift")
        paths.append(str(path))
        hashes.append(observed_sha)
    if len(set(paths)) != 3 or len(set(hashes)) != 3:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "quiescence windows are not distinct")


def _verify_document(value: Mapping[str, Any], *, receipt_path: Path, attempt_root: Path,
                     repo_root: Path) -> Dict[str, Any]:
    if value.get("schema_version") != 1 or value.get("contract_version") != AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "authorization is not executable")
    if value.get("attempt_root") != str(attempt_root) or not attempt_root.is_absolute():
        raise AuthorizationError("AUTHORIZATION_ROOT", "exact formal attempt root mismatch")
    if value.get("attempt_count") != 1 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False or \
            value.get("watchdog_seconds") != WATCHDOG_SECONDS:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "one-start/retry contract drift")
    image = value.get("image")
    if not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "image identity drift")
    input_value = value.get("input")
    if not isinstance(input_value, Mapping) or input_value.get("path") != INPUT_PATH or \
            input_value.get("bytes") != INPUT_BYTES or input_value.get("sha256") != INPUT_SHA256 or \
            input_value.get("expected_messages") != EXPECTED_MESSAGES or \
            input_value.get("expected_topic_counts") != EXPECTED_COUNTS or \
            input_value.get("required_end_timestamp_seconds") != REQUIRED_END_TIMESTAMP_SECONDS or \
            input_value.get("sensor_duration_seconds") != SENSOR_DURATION_SECONDS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "input identity/count/end contract drift")
    expected_mount = {"network": "none", "rootfs": "read_only", "output": "/out",
                      "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                      "ground_truth_mount": False, "scorer_mount": False, "map_mount": False}
    if value.get("mount_contract") != expected_mount:
        raise AuthorizationError("AUTHORIZATION_MOUNTS", "mount contract drift")
    monitor = value.get("monitor")
    if not isinstance(monitor, Mapping) or monitor.get("continuous") is not True or \
            monitor.get("interval_seconds") != 4.0 or monitor.get("natural_completion") is not True:
        raise AuthorizationError("AUTHORIZATION_MONITOR", "monitor contract drift")
    lineage = value.get("lineage")
    expected_lineage = _verify_lineage(repo_root)
    if not isinstance(lineage, Mapping) or lineage.get("formal_replay_forbidden_lineage") is not True:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "forbidden lineage missing")
    candidate = lineage.get("candidate")
    if not isinstance(candidate, Mapping) or candidate.get("launcher_path") != expected_lineage["candidate"]["launcher_path"] or \
            candidate.get("launcher_sha256") != expected_lineage["candidate"]["launcher_sha256"] or \
            candidate.get("path") != expected_lineage["candidate"]["profile_path"] or \
            candidate.get("sha256") != expected_lineage["candidate"]["profile_sha256"]:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "candidate launcher/profile binding drift")
    if lineage.get("image") != expected_lineage["image"]:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "lineage image drift")
    receipts = lineage.get("receipts")
    if not isinstance(receipts, Mapping):
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "lineage receipts missing")
    for key, entry in (("v17_build", expected_lineage["build"]), ("v17_no_input_pass", expected_lineage["no_input"]),
                       ("v17_no_input_failure", {"path": str(V17_NO_INPUT_FAILURE_PATH), "sha256": V17_NO_INPUT_FAILURE_SHA256})):
        actual = receipts.get(key)
        if not isinstance(actual, Mapping) or actual.get("path") != entry["path"] or actual.get("sha256") != entry["sha256"]:
            raise AuthorizationError("AUTHORIZATION_LINEAGE", "%s receipt binding drift" % key)
    authorizer = lineage.get("authorizer")
    if not isinstance(authorizer, Mapping) or authorizer.get("path") != str(V17_AUTHORIZER_PATH) or \
            authorizer.get("sha256") != V17_AUTHORIZER_SHA256:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v17 authorizer binding drift")
    _verify_windows(value, receipt_path)
    _false_safety(value, "authorization")
    return {"authorized": True, "formal_execution": True, "status": "AUTHORIZED",
            "attempt_root": str(attempt_root), "image_id": IMAGE_ID,
            "lineage": expected_lineage, "receipt_path": str(receipt_path)}


def _verify_v18_profile(repo_root: Path) -> None:
    profile = repo_root / V18_PROFILE_PATH.relative_to(ROOT)
    _regular(profile, "v18 profile")
    try:
        import yaml
        document = yaml.safe_load(profile.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AuthorizationError("PROFILE_INVALID", "v18 profile cannot be parsed") from exc
    if not isinstance(document, Mapping) or document.get("schema_version") != 1 or \
            document.get("status") != "V18_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or document.get("replay_count") != 0:
        raise AuthorizationError("PROFILE_AUTHORITY", "v18 profile is not unauthorized")
    candidate = document.get("candidate")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED":
        raise AuthorizationError("PROFILE_STATUS", "v18 profile status drift")
    if candidate.get("shared_verifier") != "scripts/verify_fast_livo2_m6a10_v18_authorization.py" or \
            candidate.get("shared_verifier_sha256") != sha256_file(Path(__file__).resolve()) or \
            candidate.get("authorization_receipt_path") != str(V18_AUTHORIZATION_RECEIPT_PATH) or \
            candidate.get("formal_attempt_root") != str(V18_ATTEMPT_ROOT):
        raise AuthorizationError("PROFILE_AUTHORITY", "v18 verifier/root authorization pin drift")
    previous = candidate.get("previous_preflight_failure")
    if not isinstance(previous, Mapping) or previous.get("path") != str(V17_PREFLIGHT_CLOSURE_PATH) or \
            previous.get("sha256") != V17_PREFLIGHT_CLOSURE_SHA256 or \
            previous.get("failure_kind") != "V16_FORMAL_REPLAY_UNAUTHORIZED" or \
            previous.get("formal_replay_started") is not False:
        raise AuthorizationError("PROFILE_LINEAGE", "v17 pre-Popen failure pin drift")
    previous_v18 = candidate.get("previous_v18_failure")
    if not isinstance(previous_v18, Mapping) or \
            previous_v18.get("authorization_path") != str(PREVIOUS_V18_AUTHORIZATION_RECEIPT_PATH) or \
            previous_v18.get("authorization_sha256") != PREVIOUS_V18_AUTHORIZATION_SHA256 or \
            previous_v18.get("closure_path") != str(PREVIOUS_V18_CLOSURE_PATH) or \
            previous_v18.get("closure_sha256") != PREVIOUS_V18_CLOSURE_SHA256 or \
            previous_v18.get("failure_kind") != "QUIESCENCE_FAIL_CLOSED" or \
            previous_v18.get("formal_replay_started") is not False:
        raise AuthorizationError("PROFILE_LINEAGE", "previous v18 failure pin drift")
    image = document.get("image")
    if not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise AuthorizationError("PROFILE_IMAGE", "v18 profile image drift")


def _verify_v18_document(value: Mapping[str, Any], *, receipt_path: Path,
                         attempt_root: Path, repo_root: Path) -> Dict[str, Any]:
    if value.get("schema_version") != 1 or value.get("contract_version") != V18_AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "v18 authorization is not executable")
    if receipt_path.resolve() != V18_AUTHORIZATION_RECEIPT_PATH.resolve() or \
            attempt_root.resolve() != V18_ATTEMPT_ROOT.resolve() or \
            value.get("attempt_root") != str(V18_ATTEMPT_ROOT):
        raise AuthorizationError("AUTHORIZATION_ROOT", "v18 exact authorization/attempt root mismatch")
    if value.get("attempt_count") != 1 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False or \
            value.get("watchdog_seconds") != WATCHDOG_SECONDS:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "v18 one-start/retry contract drift")
    image = value.get("image")
    if not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "v18 image identity drift")
    input_value = value.get("input")
    if not isinstance(input_value, Mapping) or input_value.get("path") != INPUT_PATH or \
            input_value.get("bytes") != INPUT_BYTES or input_value.get("sha256") != INPUT_SHA256 or \
            input_value.get("expected_messages") != EXPECTED_MESSAGES or \
            input_value.get("expected_topic_counts") != EXPECTED_COUNTS or \
            input_value.get("required_end_timestamp_seconds") != REQUIRED_END_TIMESTAMP_SECONDS or \
            input_value.get("sensor_duration_seconds") != SENSOR_DURATION_SECONDS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "v18 input identity/count/end drift")
    expected_mount = {"network": "none", "rootfs": "read_only", "output": "/out",
                      "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                      "ground_truth_mount": False, "scorer_mount": False, "map_mount": False}
    if value.get("mount_contract") != expected_mount:
        raise AuthorizationError("AUTHORIZATION_MOUNTS", "v18 mount contract drift")
    monitor = value.get("monitor")
    if not isinstance(monitor, Mapping) or monitor.get("continuous") is not True or \
            monitor.get("interval_seconds") != 4.0 or monitor.get("natural_completion") is not True:
        raise AuthorizationError("AUTHORIZATION_MONITOR", "v18 monitor contract drift")
    _verify_v18_profile(repo_root)
    expected_lineage = build_v18_lineage(repo_root)
    if value.get("lineage") != expected_lineage:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v18 source/evidence lineage drift")
    _verify_windows(value, receipt_path)
    _false_safety(value, "v18 authorization")
    return {"authorized": True, "formal_execution": True, "status": "AUTHORIZED",
            "attempt_root": str(attempt_root), "image_id": IMAGE_ID,
            "lineage": expected_lineage, "receipt_path": str(receipt_path),
            "windows": value.get("windows")}


def verify_receipt_document(path: Path, expected_sha256: str, attempt_root: Path, *,
                            repo_root: Path = ROOT, expected_receipt_path: Optional[Path] = None,
                            expected_sidecar_sha256: Optional[str] = None,
                            require_fresh_root: bool = True) -> Dict[str, Any]:
    """Verify a sealed authorization document without executing anything.

    ``expected_receipt_path`` and ``require_fresh_root`` are explicit testable
    policy inputs.  Production ``verify_authorization`` supplies both pinned
    values; tests may validate a byte-preserving fixture copy separately.
    """
    if expected_receipt_path is not None and path.resolve() != expected_receipt_path.resolve():
        raise AuthorizationError("AUTHORIZATION_PATH", "authorization receipt path drift")
    if not path.is_absolute():
        raise AuthorizationError("AUTHORIZATION_PATH", "authorization receipt path must be absolute")
    _regular(path, "authorization receipt", immutable=True)
    observed = sha256_file(path)
    if observed != expected_sha256:
        raise AuthorizationError("AUTHORIZATION_SHA", "authorization receipt SHA drift")
    _sidecar(path, observed, expected_sidecar_sha256, "authorization receipt")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("AUTHORIZATION_JSON", "authorization receipt is not JSON") from exc
    if not isinstance(value, Mapping):
        raise AuthorizationError("AUTHORIZATION_JSON", "authorization receipt must be an object")
    if require_fresh_root and os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSE", "formal attempt root already exists")
    return _verify_document(value, receipt_path=path, attempt_root=attempt_root, repo_root=repo_root)


def verify_v18_receipt_document(path: Path, expected_sha256: str, attempt_root: Path, *,
                                repo_root: Path = ROOT, expected_receipt_path: Optional[Path] = None,
                                expected_sidecar_sha256: Optional[str] = None,
                                require_fresh_root: bool = True) -> Dict[str, Any]:
    """Verify a newly sealed v18 receipt and its exact fresh attempt root."""
    expected_path = expected_receipt_path or V18_AUTHORIZATION_RECEIPT_PATH
    if path.resolve() != expected_path.resolve() or not path.is_absolute():
        raise AuthorizationError("AUTHORIZATION_PATH", "v18 authorization receipt path drift")
    _regular(path, "v18 authorization receipt", immutable=True)
    observed = sha256_file(path)
    if observed != expected_sha256:
        raise AuthorizationError("AUTHORIZATION_SHA", "v18 authorization receipt SHA drift")
    _sidecar(path, observed, expected_sidecar_sha256, "v18 authorization receipt")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("AUTHORIZATION_JSON", "v18 authorization receipt is not JSON") from exc
    if not isinstance(value, Mapping):
        raise AuthorizationError("AUTHORIZATION_JSON", "v18 authorization receipt must be an object")
    if require_fresh_root and os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSE", "v18 formal attempt root already exists")
    return _verify_v18_document(value, receipt_path=path, attempt_root=attempt_root, repo_root=repo_root)


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str, *,
                         repo_root: Path = ROOT) -> Dict[str, Any]:
    """Production entry point for the pinned v17 fixture or v18 receipt."""
    resolved = path.resolve()
    if resolved == AUTHORIZATION_RECEIPT_PATH.resolve():
        if expected_sha256 != AUTHORIZATION_RECEIPT_SHA256:
            raise AuthorizationError("AUTHORIZATION_PATH", "v17 authorization fixture SHA drift")
        return verify_receipt_document(
            path, expected_sha256, attempt_root, repo_root=repo_root,
            expected_receipt_path=AUTHORIZATION_RECEIPT_PATH,
            expected_sidecar_sha256=AUTHORIZATION_SIDECAR_SHA256,
            require_fresh_root=True,
        )
    if resolved == V18_AUTHORIZATION_RECEIPT_PATH.resolve():
        return verify_v18_receipt_document(
            path, expected_sha256, attempt_root, repo_root=repo_root,
            expected_receipt_path=V18_AUTHORIZATION_RECEIPT_PATH,
            require_fresh_root=True,
        )
    raise AuthorizationError("AUTHORIZATION_PATH", "authorization receipt path is not an approved v18 root")
