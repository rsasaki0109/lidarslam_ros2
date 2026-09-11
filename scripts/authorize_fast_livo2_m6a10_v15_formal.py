#!/usr/bin/env python3
"""Create or verify one additive v15 exact-root formal authorization.

Authorization is separate from the v15 candidate profile.  This module is
host-only: it verifies immutable image/source/identity receipts and (when
explicitly requested by a future operator) read-only quiescence windows.  It
never opens the bag, starts Docker/ROS, invokes GT/scoring, or creates a map.
The launcher source is intentionally not pinned here to avoid a
self-referential authorization hash; the launcher pins this authorizer.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_candidate.yaml"
PROFILE_SHA256 = "017e8583e1085b1f11da4117a1c53497cdcb1055c6c6224568df2f73ba751bc7"
READY_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml"
READY_PROFILE_SHA256 = "1711e33baef08e82d24c5b0bdb0d960ad2e5e1c2c41cfb9a7f8b31a0cc1bcce0"
IMAGE_TAG = "m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
EXPECTED_MESSAGES = 236687
INPUT_PATH = (
    "/media/sasaki/aiueo1/datasets/ntu_viral_release/"
    "tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
)
INPUT_BYTES = 11290464091
INPUT_SHA256 = "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310"
WINDOW_COUNT = 3
WINDOW_SECONDS = 5.0
MAX_BUSY_PERCENT = 5.0
MAX_LOAD_PER_CPU = 0.50
CONTRACT_VERSION = "m6a10-v15-formal-exact-root-authorization-v1"
RECEIPT_NAME = "formal_authorization.receipt.json"

FEEDER_PATH = ROOT / "scripts/fast_livo2_m6a10_v15_feeder.py"
FEEDER_SHA256 = "6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
WRAPPER_PATH = ROOT / "scripts/fast_livo2_m6a10_v15_formal_container_run.sh"
WRAPPER_SHA256 = "d3ab90871422100aa2d89e2469ac413c69346300e2766cc2cc88096e9b45e8f2"
V12_PATCH_PATH = ROOT / "docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch"
V12_PATCH_SHA256 = "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/"
    "build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "5005c8d392a8591bdfcda8e1de31be3a550a39c61ed22a3ae48455e1a05bce40"
BUILD_RECEIPT_SIDECAR_SHA256 = "6940f5762e87bf9f08752b950f12de2ae8423c9b0217841e212af56f61a04c65"
NO_INPUT_PASS_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T204349Z_agentv15/"
    "no_input_correction.receipt.json"
)
NO_INPUT_PASS_RECEIPT_SHA256 = "e5e1703b6cfcd6b10c9703af06f7d89ede2c638519b285d3ae12152b3e63b5ff"
NO_INPUT_PASS_SIDECAR_SHA256 = "d2cf11c36fb203d3972daeef9f9c2f0fe462a82dfac19070a822c6672fb086a3"
PRIOR_NO_INPUT_FAIL_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/"
    "no_input_identity.receipt.json"
)
PRIOR_NO_INPUT_FAIL_SHA256 = "36782a546e558a537619f183599112354d547f0fd0a70f06c4618ec76aa5a4de"
PRIOR_NO_INPUT_FAIL_SIDECAR_SHA256 = "197ee62c79b397d9f7ef6c6d6be40f1063584dea7cc69ef5c47558991b52320f"
PRIOR_NO_INPUT_FAIL_LOG_PATH = PRIOR_NO_INPUT_FAIL_RECEIPT_PATH.with_name("no_input_identity.log")
PRIOR_NO_INPUT_FAIL_LOG_SHA256 = "004af952222b8a1ba4378aafb904f0a192b1ba306d421c1fb76d769fa4c2612c"
PRIOR_VALIDATOR_FAIL_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T203806Z_agentv15/"
    "no_input_correction.receipt.json"
)
PRIOR_VALIDATOR_FAIL_SHA256 = "804620192884f097de5fbc4e7946c789af2237c679f38d1ffbdecf616a28aca8"
PRIOR_VALIDATOR_FAIL_SIDECAR_SHA256 = "581f9835c23a200f2a651ded025e3fe783371c4e12ebb334b13b0529d4e0ed7a"

# The authorization deliberately does not pin its launcher: the launcher
# pins this file after its source is complete, avoiding a hash cycle.
CANDIDATE_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v15_formal.py"
CANDIDATE_LAUNCHER_SHA256 = None


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
    current = Path(path.absolute().anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if os.path.lexists(path) is False or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not regular" % label)


def _pin(path: Path, expected: str, label: str) -> str:
    _regular(path, label)
    observed = sha256_file(path)
    if observed != expected:
        raise AuthorizationError("SOURCE_DRIFT", "%s SHA drift" % label)
    return observed


def _json(path: Path, expected: str, label: str,
          sidecar_sha: Optional[str] = None) -> Dict[str, Any]:
    computed = _pin(path, expected, label)
    if computed != hashlib.sha256(path.read_bytes()).hexdigest():
        raise AuthorizationError("RECEIPT_SHA256", "%s computed SHA drift" % label)
    if sidecar_sha is not None:
        sidecar = path.with_name(path.name + ".sha256")
        if sidecar.name != "%s.sha256" % path.name:
            raise AuthorizationError("RECEIPT_SIDECAR", "%s sidecar filename drift" % label)
        sidecar_computed = _pin(sidecar, sidecar_sha, "%s sidecar" % label)
        expected_line = "%s  %s\n" % (expected, path.name)
        if sidecar_computed != hashlib.sha256(sidecar.read_bytes()).hexdigest() or \
                sidecar.read_bytes() != expected_line.encode("ascii"):
            raise AuthorizationError("RECEIPT_SIDECAR", "%s sidecar content drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AuthorizationError("RECEIPT_INVALID", "%s is not JSON" % label) from error
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s is not an object" % label)
    return value


def _verify_profile(repo_root: Path = ROOT) -> Dict[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    _pin(path, PROFILE_SHA256, "v15 candidate profile")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise AuthorizationError("PROFILE_INVALID", "v15 candidate profile YAML invalid") from error
    candidate = document.get("formal_candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    if document.get("status") != "V15_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or \
            not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise AuthorizationError("PROFILE_AUTHORITY", "v15 profile is not fail-closed")
    _pin(repo_root / READY_PROFILE_PATH.relative_to(ROOT), READY_PROFILE_SHA256, "v15 ready profile")
    return {"path": str(path.resolve()), "sha256": PROFILE_SHA256, "document": document}


def _verify_immutable_lineage(repo_root: Path = ROOT) -> Dict[str, Any]:
    profile = _verify_profile(repo_root)
    source_pins = {
        "v15_feeder": (FEEDER_PATH.relative_to(ROOT), FEEDER_SHA256),
        "v15_wrapper": (WRAPPER_PATH.relative_to(ROOT), WRAPPER_SHA256),
        "v12_patch": (V12_PATCH_PATH.relative_to(ROOT), V12_PATCH_SHA256),
    }
    sources = {}
    for name, (relative, expected) in source_pins.items():
        path = repo_root / relative
        _pin(path, expected, name)
        sources[name] = {"path": str(path.resolve()), "sha256": expected}

    build = _json(BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, "v15 build",
                  BUILD_RECEIPT_SIDECAR_SHA256)
    if build.get("status") != "PASS" or build.get("image", {}).get("id") != IMAGE_ID or \
            build.get("image", {}).get("tag") != IMAGE_TAG or \
            build.get("execution", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("BUILD_RECEIPT", "v15 build identity is not admissible")

    no_input = _json(NO_INPUT_PASS_RECEIPT_PATH, NO_INPUT_PASS_RECEIPT_SHA256,
                     "v15 no-input PASS", NO_INPUT_PASS_SIDECAR_SHA256)
    execution = no_input.get("execution", {})
    validation = no_input.get("validation", {})
    if no_input.get("status") != "PASS" or no_input.get("image", {}).get("id") != IMAGE_ID or \
            validation.get("schema3_valid_document") is not True or \
            validation.get("schema3_invalid_document_rejected") is not True or \
            execution.get("network_mode") != "none" or execution.get("readonly_rootfs") is not True or \
            execution.get("host_bind_mounts") != 0 or execution.get("oom_killed") is not False or \
            no_input.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("NO_INPUT_RECEIPT", "v15 no-input PASS receipt is not admissible")

    prior = _json(PRIOR_NO_INPUT_FAIL_RECEIPT_PATH, PRIOR_NO_INPUT_FAIL_SHA256,
                  "prior v15 no-input failure", PRIOR_NO_INPUT_FAIL_SIDECAR_SHA256)
    if prior.get("status") != "FAIL_CLOSED" or prior.get("execution", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("PRIOR_FAILURE_LINEAGE", "prior v15 no-input failure drift")
    _pin(PRIOR_NO_INPUT_FAIL_LOG_PATH, PRIOR_NO_INPUT_FAIL_LOG_SHA256, "prior no-input log")
    if "ROS_MASTER_URI" not in PRIOR_NO_INPUT_FAIL_LOG_PATH.read_text(encoding="utf-8", errors="replace"):
        raise AuthorizationError("PRIOR_FAILURE_EVIDENCE", "prior ROS env failure evidence missing")

    prior_validator = _json(PRIOR_VALIDATOR_FAIL_RECEIPT_PATH, PRIOR_VALIDATOR_FAIL_SHA256,
                            "prior tmpfs validator failure", PRIOR_VALIDATOR_FAIL_SIDECAR_SHA256)
    if prior_validator.get("status") != "FAIL_CLOSED" or \
            prior_validator.get("failure_kind") != "TMPFS":
        raise AuthorizationError("PRIOR_FAILURE_LINEAGE", "prior tmpfs validation failure drift")
    return {
        "profile": profile, "sources": sources,
        "build": {"path": str(BUILD_RECEIPT_PATH), "sha256": BUILD_RECEIPT_SHA256},
        "no_input_pass": {"path": str(NO_INPUT_PASS_RECEIPT_PATH), "sha256": NO_INPUT_PASS_RECEIPT_SHA256},
        "prior_no_input_fail": {"path": str(PRIOR_NO_INPUT_FAIL_RECEIPT_PATH), "sha256": PRIOR_NO_INPUT_FAIL_SHA256},
        "prior_validator_fail": {"path": str(PRIOR_VALIDATOR_FAIL_RECEIPT_PATH), "sha256": PRIOR_VALIDATOR_FAIL_SHA256},
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
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


def _run_window(path: Path, *, proc_root: Path, now: Optional[str] = None) -> Dict[str, Any]:
    excluded = quiescence.ancestor_pids(proc_root)
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    receipt = quiescence.build_receipt(observation, now=now)
    receipt.update({"authorization_window": True, "launcher_pid": os.getpid(),
                    "excluded_ancestor_pids": sorted(excluded)})
    _write_json(path, receipt)
    return {"path": str(path.resolve()), "sha256": sha256_file(path),
            "status": receipt.get("status"),
            "runner_start_allowed": receipt.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", [])}


def _seal_authorization(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
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
        raise AuthorizationError("ATTEMPT_ROOT_NOT_FRESH", "attempt root must be absent")
    authorization_root.mkdir(parents=True)
    try:
        lineage = _verify_immutable_lineage(repo_root)
        runner = _run_window if window_runner is None else window_runner
        windows = []
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now))
            result["window_index"] = index
            windows.append(result)
        if len(windows) != WINDOW_COUNT or any(
                item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or
                item.get("forbidden_processes") for item in windows):
            value = {"schema_version": 1, "contract_version": CONTRACT_VERSION,
                     "status": "FAIL_CLOSED", "failure_kind": "QUIESCENCE_FAIL_CLOSED",
                     "formal_execution": False, "formal_replay_started": False,
                     "attempt_root": str(attempt_root), "windows": windows,
                     "lineage": lineage, "safety": {"input_opened": False,
                     "ground_truth_content_opened": False, "scorer_invoked": False,
                     "map_saved": False}}
            return _seal_authorization(authorization_root, value)
        value = {
            "schema_version": 1, "contract_version": CONTRACT_VERSION,
            "status": "AUTHORIZED", "authorized": True, "formal_execution": True,
            "formal_replay_forbidden": False, "formal_replay_started": False,
            "attempt_root": str(attempt_root), "attempt_count": 1,
            "retry": False, "manual_stop": False, "watchdog_seconds": 1200,
            "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
            "input": {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256,
                      "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS},
            "windows": windows, "lineage": lineage,
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        }
        return _seal_authorization(authorization_root, value)
    except Exception as error:
        if (authorization_root / RECEIPT_NAME).exists():
            raise
        value = {"schema_version": 1, "contract_version": CONTRACT_VERSION,
                 "status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                 "failure_message": str(error), "formal_execution": False,
                 "formal_replay_started": False, "attempt_root": str(attempt_root),
                 "safety": {"input_opened": False, "ground_truth_content_opened": False,
                            "scorer_invoked": False, "map_saved": False}}
        return _seal_authorization(authorization_root, value)


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Dict[str, Any]:
    if not expected_sha256:
        raise AuthorizationError("AUTHORIZATION_SHA", "authorization SHA is required")
    value = _json(path, expected_sha256, "v15 authorization")
    _verify_immutable_lineage(repo_root)
    if value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_started") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "authorization is not executable")
    if value.get("attempt_root") != str(attempt_root):
        raise AuthorizationError("AUTHORIZATION_ROOT", "authorization exact root mismatch")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSE", "attempt root already exists")
    if value.get("attempt_count") != 1 or value.get("retry") is not False or value.get("manual_stop") is not False:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "authorization replay contract drift")
    image = value.get("image", {})
    if image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "authorization image identity drift")
    return dict(value, authorized=True)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    result = authorize(args.authorization_root, args.attempt_root, repo_root=args.repo_root)
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
