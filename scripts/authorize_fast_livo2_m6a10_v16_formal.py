#!/usr/bin/env python3
"""Create/verify one additive v16 exact-root formal authorization.

This module is host-only.  It verifies the immutable v12-v15 evidence
lineage and performs three read-only quiescence windows.  It never opens the
bag, invokes Docker/ROS, runs GT/scoring, or creates a map.  The v15 and v16
candidate files remain immutable; this authorization is valid only for the
fresh attempt root named in its sealed receipt.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402


V16_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v16_formal.py"
V16_LAUNCHER_SHA256 = "77e46d0aa326f9940f666dbbc885ddf87cace1ba643c8e4f67daba9dba1c5757"
V16_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v16_formal_candidate.yaml"
V16_PROFILE_SHA256 = "a9ca34982cfb22fe7e3bb0e2a5437a10875990926fddb820fb6a29f6abc6461f"
V16_CONTRACT = "m6a10-v16-formal-candidate-closure-v1"
AUTHORIZATION_CONTRACT = "m6a10-v16-formal-exact-root-authorization-v1"
IMAGE_TAG = "m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
INPUT_PATH = (
    "/media/sasaki/aiueo1/datasets/ntu_viral_release/"
    "tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
)
INPUT_BYTES = 11290464091
INPUT_SHA256 = "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310"
EXPECTED_MESSAGES = 236687
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
SENSOR_DURATION_SECONDS = 579.278127298
WATCHDOG_SECONDS = 1200
WINDOW_COUNT = 3
WINDOW_SECONDS = 5.0
MAX_BUSY_PERCENT = 5.0
MAX_LOAD_PER_CPU = 0.50
RECEIPT_NAME = "formal_authorization.receipt.json"

V15_BUILD_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/build_identity.receipt.json"
)
V15_BUILD_SHA256 = "5005c8d392a8591bdfcda8e1de31be3a550a39c61ed22a3ae48455e1a05bce40"
V15_BUILD_SIDECAR_SHA256 = "6940f5762e87bf9f08752b950f12de2ae8423c9b0217841e212af56f61a04c65"
V15_NO_INPUT_PASS_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T204349Z_agentv15/no_input_correction.receipt.json"
)
V15_NO_INPUT_PASS_SHA256 = "e5e1703b6cfcd6b10c9703af06f7d89ede2c638519b285d3ae12152b3e63b5ff"
V15_NO_INPUT_PASS_SIDECAR_SHA256 = "d2cf11c36fb203d3972daeef9f9c2f0fe462a82dfac19070a822c6672fb086a3"
V15_NO_INPUT_FAIL_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/no_input_identity.receipt.json"
)
V15_NO_INPUT_FAIL_SHA256 = "36782a546e558a537619f183599112354d547f0fd0a70f06c4618ec76aa5a4de"
V15_NO_INPUT_FAIL_SIDECAR_SHA256 = "197ee62c79b397d9f7ef6c6d6be40f1063584dea7cc69ef5c47558991b52320f"
V15_VALIDATOR_FAIL_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T203806Z_agentv15/no_input_correction.receipt.json"
)
V15_VALIDATOR_FAIL_SHA256 = "804620192884f097de5fbc4e7946c789af2237c679f38d1ffbdecf616a28aca8"
V15_VALIDATOR_FAIL_SIDECAR_SHA256 = "581f9835c23a200f2a651ded025e3fe783371c4e12ebb334b13b0529d4e0ed7a"

V12_NO_INPUT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_dual_service_attempt6_20260823T175000Z_agentv12/no_input_dual_evidence.receipt.json"
)
V12_NO_INPUT_SHA256 = "c5afa2452f96588bf46dc7f0d0b4504e816d0069bbd3a107009eb2c3beac0b45"
V12_NO_INPUT_SIDECAR_SHA256 = "076cd96e10c78d0bf73a6a7ed66d3549aaa2dd0f75c7503d5d88e19237d608e3"
V12_HOST_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_host_evidence_gate_20260823T180718Z_agentv12/host_evidence_gate.receipt.json"
)
V12_HOST_SHA256 = "3a603bc4ece0e9167b0b3943df6be569f2e64bce196beb18da29ff09ad69f643"
V12_HOST_SIDECAR_SHA256 = "ff50620daf076ed357032111d33197da1903b840605fa34259f3880ae0e49f66"
V13_PERSISTENCE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_persistence_20260823T192720Z_agentv13/persistence_gate.receipt.json"
)
V13_PERSISTENCE_SHA256 = "18998f6177418eaf7e98fe517691d0642d59ac48f2faeb9b45ef0bc8fef62c72"
V13_PERSISTENCE_SIDECAR_SHA256 = "1988676efabdf1276ca3bb635f784941d2539aa15f58f94a8f249bcb730eabb7"
V13_FAILURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_formal_replay_20260823T194325Z_agentv13formal/closure_receipt.json"
)
V13_FAILURE_SHA256 = "73252dd828388a5ff026413ec957cb02504dea4ba14b936678257333954b8212"
V13_FAILURE_SIDECAR_SHA256 = "532a63278fa6fa1e4cbaaa52ef6bd6c9cfdc8a9bfe8b19961aa858c27d8734cb"
V14_FAILURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v14_formal_replay_20260823T200339Z_agentv14formal/closure_receipt.json"
)
V14_FAILURE_SHA256 = "1e086c34d8f3af6c8f06543bfa91ef28c385db707a6be82b91541863f2df0096"
V14_FAILURE_SIDECAR_SHA256 = "d7076bc257187766d02c938136d67f1ae1640e4a7c193861533e407c0478b725"


class AuthorizationError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if os.path.lexists(path) is False or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not a regular file" % label)


def _json_receipt(path: Path, expected_sha: str, sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label)
    observed = sha256_file(path)
    if observed != expected_sha:
        raise AuthorizationError("RECEIPT_DRIFT", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label)
    if sha256_file(sidecar) != sidecar_sha:
        raise AuthorizationError("RECEIPT_SIDECAR_DRIFT", "%s sidecar SHA drift" % label)
    expected_line = ("%s  %s\n" % (expected_sha, path.name)).encode("ascii")
    if sidecar.name != path.name + ".sha256" or sidecar.read_bytes() != expected_line:
        raise AuthorizationError("RECEIPT_SIDECAR_CONTENT", "%s sidecar filename/content drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_INVALID", "%s is not valid JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s must be an object" % label)
    return value


def _load_v16() -> Any:
    spec = importlib.util.spec_from_file_location("m6a10_v16_authorizer_candidate", V16_LAUNCHER_PATH)
    if spec is None or spec.loader is None:
        raise AuthorizationError("CANDIDATE_LOAD", "v16 launcher cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _verify_candidate() -> Dict[str, Any]:
    _regular(V16_LAUNCHER_PATH, "v16 launcher")
    _regular(V16_PROFILE_PATH, "v16 candidate profile")
    if sha256_file(V16_LAUNCHER_PATH) != V16_LAUNCHER_SHA256:
        raise AuthorizationError("SOURCE_DRIFT", "v16 launcher SHA drift")
    if sha256_file(V16_PROFILE_PATH) != V16_PROFILE_SHA256:
        raise AuthorizationError("SOURCE_DRIFT", "v16 profile SHA drift")
    candidate = _load_v16()
    profile = candidate.verify_candidate_profile()
    if profile.get("sha256") != V16_PROFILE_SHA256 or \
            profile.get("phase_contract") != candidate.PHASE_CONTRACT or \
            profile.get("transport_contract") != candidate.TRANSPORT_CONTRACT:
        raise AuthorizationError("PROFILE_DRIFT", "v16 candidate profile contract drift")
    return {"path": str(V16_PROFILE_PATH), "sha256": V16_PROFILE_SHA256,
            "launcher_path": str(V16_LAUNCHER_PATH), "launcher_sha256": V16_LAUNCHER_SHA256}


def _verify_lineage() -> Dict[str, Any]:
    candidate = _verify_candidate()
    build = _json_receipt(V15_BUILD_PATH, V15_BUILD_SHA256, V15_BUILD_SIDECAR_SHA256, "v15 build")
    if build.get("status") != "PASS" or build.get("image", {}).get("id") != IMAGE_ID or \
            build.get("image", {}).get("tag") != IMAGE_TAG or \
            build.get("execution", {}).get("formal_replay_started") is not False or \
            build.get("safety", {}).get("ground_truth_content_opened") is not False:
        raise AuthorizationError("BUILD_LINEAGE", "v15 build identity/safety drift")
    no_input = _json_receipt(V15_NO_INPUT_PASS_PATH, V15_NO_INPUT_PASS_SHA256,
                             V15_NO_INPUT_PASS_SIDECAR_SHA256, "v15 no-input PASS")
    if no_input.get("status") != "PASS" or no_input.get("image", {}).get("id") != IMAGE_ID or \
            no_input.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("NO_INPUT_LINEAGE", "v15 no-input PASS drift")
    old_no_input = _json_receipt(V15_NO_INPUT_FAIL_PATH, V15_NO_INPUT_FAIL_SHA256,
                                 V15_NO_INPUT_FAIL_SIDECAR_SHA256, "v15 no-input failure")
    old_validator = _json_receipt(V15_VALIDATOR_FAIL_PATH, V15_VALIDATOR_FAIL_SHA256,
                                  V15_VALIDATOR_FAIL_SIDECAR_SHA256, "v15 validator failure")
    if old_no_input.get("status") != "FAIL_CLOSED" or old_validator.get("status") != "FAIL_CLOSED" or \
            old_validator.get("failure_kind") != "TMPFS":
        raise AuthorizationError("V15_FAILURE_LINEAGE", "v15 failure lineage drift")

    v12_no_input = _json_receipt(V12_NO_INPUT_PATH, V12_NO_INPUT_SHA256,
                                 V12_NO_INPUT_SIDECAR_SHA256, "v12 no-input")
    protocol = v12_no_input.get("protocol", {})
    execution = v12_no_input.get("execution", {})
    safety = v12_no_input.get("safety", {})
    if v12_no_input.get("status") != "PASS" or v12_no_input.get("attempt_index") != 6 or \
            execution.get("one_start") is not True or execution.get("start_count") != 1 or \
            execution.get("network") != "none" or execution.get("rootfs") != "read_only" or \
            execution.get("host_mounts") != [] or execution.get("input_mounts") != 0 or \
            execution.get("retry_count") != 0 or execution.get("manual_stop") is not False or \
            safety.get("formal_replay_started") is not False or safety.get("input_opened") is not False or \
            safety.get("ground_truth_content_opened") is not False or safety.get("scorer_invoked") is not False or \
            safety.get("map_saved") is not False or protocol.get("published_lidar_callbacks") != 1 or \
            protocol.get("first_ack_success") is not True or protocol.get("duplicate_ack_rejected") is not True or \
            len(protocol.get("services", [])) != 7:
        raise AuthorizationError("V12_NO_INPUT_LINEAGE", "v12 no-input protocol/safety drift")
    v12_host = _json_receipt(V12_HOST_PATH, V12_HOST_SHA256,
                             V12_HOST_SIDECAR_SHA256, "v12 host evidence")
    if v12_host.get("status") != "PASS" or v12_host.get("formal_replay_started") is not False or \
            any(v12_host.get("safety", {}).get(key) is not False for key in (
                "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("V12_HOST_LINEAGE", "v12 host evidence/safety drift")

    v13_persistence = _json_receipt(V13_PERSISTENCE_PATH, V13_PERSISTENCE_SHA256,
                                     V13_PERSISTENCE_SIDECAR_SHA256, "v13 persistence")
    if v13_persistence.get("status") != "PASS" or \
            v13_persistence.get("safety", {}).get("formal_replay_started") is not False or \
            v13_persistence.get("cleanup", {}).get("stopped_only") is not True or \
            v13_persistence.get("cleanup", {}).get("removed") is not True:
        raise AuthorizationError("V13_PERSISTENCE_LINEAGE", "v13 persistence evidence drift")
    v13_failure = _json_receipt(V13_FAILURE_PATH, V13_FAILURE_SHA256,
                                V13_FAILURE_SIDECAR_SHA256, "v13 failure")
    if v13_failure.get("status") != "FAIL_CLOSED" or \
            v13_failure.get("failure_kind") != "DOCKER_ARGV_BUILDER_RECURSION" or \
            v13_failure.get("execution", {}).get("popen_count") != 0:
        raise AuthorizationError("V13_FAILURE_LINEAGE", "v13 recursion failure drift")
    v14_failure = _json_receipt(V14_FAILURE_PATH, V14_FAILURE_SHA256,
                                V14_FAILURE_SIDECAR_SHA256, "v14 failure")
    if v14_failure.get("status") != "FAIL_CLOSED" or \
            v14_failure.get("failure_kind") != "COUNTS_INVALID" or \
            v14_failure.get("execution", {}).get("one_start") is not True or \
            v14_failure.get("safety", {}).get("ground_truth_content_opened") is not False or \
            v14_failure.get("safety", {}).get("scorer_invoked") is not False or \
            v14_failure.get("safety", {}).get("map_saved") is not False:
        raise AuthorizationError("V14_FAILURE_LINEAGE", "v14 failure/safety drift")
    return {
        "authorizer": {"path": str(SCRIPT), "sha256": sha256_file(SCRIPT)},
        "candidate": candidate,
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "receipts": {
            "v15_build": {"path": str(V15_BUILD_PATH), "sha256": V15_BUILD_SHA256,
                           "sidecar_sha256": V15_BUILD_SIDECAR_SHA256},
            "v15_no_input_pass": {"path": str(V15_NO_INPUT_PASS_PATH), "sha256": V15_NO_INPUT_PASS_SHA256,
                                   "sidecar_sha256": V15_NO_INPUT_PASS_SIDECAR_SHA256},
            "v12_no_input": {"path": str(V12_NO_INPUT_PATH), "sha256": V12_NO_INPUT_SHA256,
                              "sidecar_sha256": V12_NO_INPUT_SIDECAR_SHA256},
            "v12_host": {"path": str(V12_HOST_PATH), "sha256": V12_HOST_SHA256,
                          "sidecar_sha256": V12_HOST_SIDECAR_SHA256},
            "v13_persistence": {"path": str(V13_PERSISTENCE_PATH), "sha256": V13_PERSISTENCE_SHA256,
                                 "sidecar_sha256": V13_PERSISTENCE_SIDECAR_SHA256},
            "v13_failure": {"path": str(V13_FAILURE_PATH), "sha256": V13_FAILURE_SHA256,
                             "sidecar_sha256": V13_FAILURE_SIDECAR_SHA256},
            "v14_failure": {"path": str(V14_FAILURE_PATH), "sha256": V14_FAILURE_SHA256,
                             "sidecar_sha256": V14_FAILURE_SIDECAR_SHA256},
            "v15_no_input_failure": {"path": str(V15_NO_INPUT_FAIL_PATH), "sha256": V15_NO_INPUT_FAIL_SHA256,
                                      "sidecar_sha256": V15_NO_INPUT_FAIL_SIDECAR_SHA256},
            "v15_validator_failure": {"path": str(V15_VALIDATOR_FAIL_PATH), "sha256": V15_VALIDATOR_FAIL_SHA256,
                                       "sidecar_sha256": V15_VALIDATOR_FAIL_SIDECAR_SHA256},
        },
        "formal_replay_forbidden_lineage": True,
    }


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
        proc_root=proc_root,
        sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT,
        max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    receipt = quiescence.build_receipt(observation, now=now)
    receipt.update({"authorization_window": True, "launcher_pid": os.getpid(),
                    "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, receipt)
    return {"path": str(path), "sha256": digest, "status": receipt.get("status"),
            "runner_start_allowed": receipt.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", [])}


def _sealed_windows_pass(windows: Any, authorization_root: Path) -> bool:
    if not isinstance(windows, list) or len(windows) != WINDOW_COUNT:
        return False
    for expected_index, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != expected_index:
            return False
        try:
            path = Path(str(item["path"]))
            if path.parent.resolve() != authorization_root.resolve() or \
                    path.name != ("quiescence_window_%02d.receipt.json" % expected_index):
                return False
            _regular(path, "authorization quiescence window")
            if sha256_file(path) != item.get("sha256"):
                return False
            value = json.loads(path.read_text(encoding="utf-8"))
        except (KeyError, OSError, UnicodeError, json.JSONDecodeError, AuthorizationError):
            return False
        observation = value.get("observation")
        forbidden = (observation.get("forbidden_processes")
                     if isinstance(observation, Mapping)
                     else value.get("forbidden_processes"))
        runner_allowed = value.get("runner_start_allowed")
        if not isinstance(value, Mapping) or value.get("schema_version") != 1 or \
                value.get("contract_version") != "m6a10-quiescence-v1" or \
                value.get("status") != "PASS" or runner_allowed is not True or forbidden:
            return False
        if item.get("status") != value.get("status") or \
                item.get("runner_start_allowed") is not runner_allowed or \
                item.get("forbidden_processes") != forbidden:
            return False
    return True


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt_path = root / RECEIPT_NAME
    receipt_sha = _write_json(receipt_path, value)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, ("%s  %s\n" % (receipt_sha, receipt_path.name)).encode("ascii"))
    result = dict(value)
    result.update({"receipt_path": str(receipt_path), "receipt_sha256": receipt_sha,
                   "sidecar_path": str(sidecar), "sidecar_sha256": sidecar_sha})
    return result


WindowRunner = Callable[..., Mapping[str, Any]]


def authorize(authorization_root: Path, attempt_root: Path, *, repo_root: Path = ROOT,
              proc_root: Path = Path("/proc"), now: Optional[str] = None,
              window_runner: Optional[WindowRunner] = None) -> Dict[str, Any]:
    if os.path.lexists(authorization_root):
        raise AuthorizationError("AUTH_ROOT_NOT_FRESH", "authorization root already exists")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_NOT_FRESH", "formal attempt root must be absent")
    authorization_root.mkdir(parents=True)
    try:
        lineage = _verify_lineage()
        runner = _run_window if window_runner is None else window_runner
        windows: List[Dict[str, Any]] = []
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now))
            result["window_index"] = index
            windows.append(result)
        passed = _sealed_windows_pass(windows, authorization_root)
        value: Dict[str, Any] = {
            "schema_version": 1,
            "contract_version": AUTHORIZATION_CONTRACT,
            "status": "AUTHORIZED" if passed else "FAIL_CLOSED",
            "authorized": passed,
            "formal_execution": passed,
            "formal_replay_forbidden": not passed,
            "formal_replay_started": False,
            "attempt_root": str(attempt_root),
            "attempt_count": 1,
            "retry": False,
            "manual_stop": False,
            "watchdog_seconds": WATCHDOG_SECONDS,
            "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
            "input": {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256,
                      "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS,
                      "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
                      "sensor_duration_seconds": SENSOR_DURATION_SECONDS},
            "mount_contract": {"network": "none", "rootfs": "read_only", "output": "/out",
                               "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                               "ground_truth_mount": False, "scorer_mount": False, "map_mount": False},
            "one_start": True,
            "monitor": {"continuous": True, "interval_seconds": 4.0, "natural_completion": True},
            "lineage": lineage,
            "windows": windows,
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        }
        if not passed:
            value["failure_kind"] = "QUIESCENCE_FAIL_CLOSED"
        return _seal(authorization_root, value)
    except Exception as error:
        if (authorization_root / RECEIPT_NAME).exists():
            raise
        value = {
            "schema_version": 1,
            "contract_version": AUTHORIZATION_CONTRACT,
            "status": "FAIL_CLOSED",
            "authorized": False,
            "formal_execution": False,
            "formal_replay_forbidden": True,
            "formal_replay_started": False,
            "attempt_root": str(attempt_root),
            "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(error),
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        }
        return _seal(authorization_root, value)


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Dict[str, Any]:
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "v16 authorization sidecar")
    value = _json_receipt(path, expected_sha256, sha256_file(sidecar), "v16 authorization")
    lineage = _verify_lineage()
    if value.get("contract_version") != AUTHORIZATION_CONTRACT or value.get("status") != "AUTHORIZED" or \
            value.get("authorized") is not True or value.get("formal_execution") is not True or \
            value.get("formal_replay_forbidden") is not False or value.get("formal_replay_started") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "v16 authorization is not executable")
    if value.get("attempt_root") != str(attempt_root):
        raise AuthorizationError("AUTHORIZATION_ROOT", "v16 exact attempt root mismatch")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSE", "v16 attempt root already exists")
    if value.get("attempt_count") != 1 or value.get("retry") is not False or value.get("manual_stop") is not False or \
            value.get("watchdog_seconds") != WATCHDOG_SECONDS:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "v16 one-start/retry contract drift")
    if value.get("image", {}).get("id") != IMAGE_ID or value.get("image", {}).get("tag") != IMAGE_TAG:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "v16 image identity drift")
    if not _sealed_windows_pass(value.get("windows"), path.parent):
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "v16 authorization windows are not PASS")
    input_contract = value.get("input")
    if not isinstance(input_contract, Mapping) or \
            input_contract.get("path") != INPUT_PATH or input_contract.get("bytes") != INPUT_BYTES or \
            input_contract.get("sha256") != INPUT_SHA256 or input_contract.get("expected_messages") != EXPECTED_MESSAGES or \
            input_contract.get("expected_topic_counts") != EXPECTED_COUNTS or \
            input_contract.get("required_end_timestamp_seconds") != REQUIRED_END_TIMESTAMP_SECONDS or \
            input_contract.get("sensor_duration_seconds") != SENSOR_DURATION_SECONDS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "v16 input contract drift")
    mount_contract = value.get("mount_contract")
    expected_mount = {"network": "none", "rootfs": "read_only", "output": "/out",
                      "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                      "ground_truth_mount": False, "scorer_mount": False, "map_mount": False}
    if mount_contract != expected_mount:
        raise AuthorizationError("AUTHORIZATION_MOUNTS", "v16 mount contract drift")
    monitor = value.get("monitor")
    if not isinstance(monitor, Mapping) or monitor.get("continuous") is not True or \
            monitor.get("interval_seconds") != 4.0 or monitor.get("natural_completion") is not True:
        raise AuthorizationError("AUTHORIZATION_MONITOR", "v16 monitor contract drift")
    authorizer_source = value.get("lineage", {}).get("authorizer")
    if not isinstance(authorizer_source, Mapping) or authorizer_source.get("path") != str(SCRIPT) or \
            authorizer_source.get("sha256") != sha256_file(SCRIPT) or \
            lineage.get("authorizer") != authorizer_source:
        raise AuthorizationError("AUTHORIZER_DRIFT", "v16 authorizer source binding drift")
    if value.get("lineage", {}).get("formal_replay_forbidden_lineage") is not True:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v16 formal-forbidden lineage drift")
    if any(value.get("safety", {}).get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "v16 authorization safety drift")
    return dict(value, authorized=True)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    try:
        result = authorize(args.authorization_root, args.attempt_root, repo_root=args.repo_root)
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                          "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
