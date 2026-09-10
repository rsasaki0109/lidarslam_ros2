#!/usr/bin/env python3
"""Host-only exact-root authorization for the additive v17 candidate.

The authorizer verifies the immutable v17 build/no-input receipts, the prior
v17 fail-closed receipt, and the v16 terminal-invalid closure lineage.  It
performs three read-only quiescence windows and seals an authorization only
for a fresh attempt root.  It never opens input, starts a container, runs a
replay, invokes scoring, or writes a map.
"""

from __future__ import annotations

import argparse
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


V17_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v17_formal.py"
V17_LAUNCHER_SHA256 = "ab10bd00e06919074c794032314612bfa8781b2097babe88b9f1a6b40ecdf514"
V17_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_ready.yaml"
V17_PROFILE_SHA256 = "10ddffb6eed647eb3b7a2ee7ede49505923755eb917d8b87737675853e35d9c5"
V17_CONTRACT = "m6a10-v17-formal-candidate-closure-v1"
AUTHORIZATION_CONTRACT = "m6a10-v17-formal-exact-root-authorization-v1"
IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
INPUT_PATH = "/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
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

V17_BUILD_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json")
V17_BUILD_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
V17_BUILD_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
V17_NO_INPUT_PASS_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/no_input.receipt.json")
V17_NO_INPUT_PASS_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
V17_NO_INPUT_PASS_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"
V17_NO_INPUT_FAIL_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction_20260823T224017Z_agentv17/no_input.receipt.json")
V17_NO_INPUT_FAIL_SHA256 = "93296e2d47e37aeea739daf3c44d4c7bf988c7a9bcfc61f11a4cbab31ec1203a"
V17_NO_INPUT_FAIL_SIDECAR_SHA256 = "a0db9e7c7303974edc575033088bc92e0bc14ab8db7eda054ae16c1ec4d9c2f7"
V16_CLOSURE_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v16_formal_replay_20260823T213533Z_agentv16formal/closure_receipt.json")
V16_CLOSURE_SHA256 = "a7a5bfaaaf99a0851cb1b5b19759cdc568c3d4f681f8f2ac81c438944d1c1ccc"
V16_TERMINAL_RAW_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v16_formal_replay_20260823T213533Z_agentv16formal/out/consumer_evidence.json")
V16_TERMINAL_RAW_SHA256 = "56523b593676961b83ca9f9340706e964086b6aa3916e47902219b81dc41d21a"


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
    if not os.path.lexists(path) or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not a regular file" % label)


def _json_receipt(path: Path, expected_sha: str, sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label)
    if sha256_file(path) != expected_sha:
        raise AuthorizationError("RECEIPT_DRIFT", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label)
    if sha256_file(sidecar) != sidecar_sha:
        raise AuthorizationError("RECEIPT_SIDECAR_DRIFT", "%s sidecar SHA drift" % label)
    if sidecar.read_bytes() != (expected_sha + "  " + path.name + "\n").encode("ascii"):
        raise AuthorizationError("RECEIPT_SIDECAR_CONTENT", "%s sidecar content drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_INVALID", "%s is not JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s must be an object" % label)
    return value


def _load_v17() -> Any:
    spec = importlib.util.spec_from_file_location("m6a10_v17_authorizer_candidate", V17_LAUNCHER_PATH)
    if spec is None or spec.loader is None:
        raise AuthorizationError("CANDIDATE_LOAD", "v17 launcher cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _verify_candidate() -> Dict[str, Any]:
    _regular(V17_LAUNCHER_PATH, "v17 launcher")
    _regular(V17_PROFILE_PATH, "v17 profile")
    if V17_LAUNCHER_SHA256.startswith("TO_BE_") or sha256_file(V17_LAUNCHER_PATH) != V17_LAUNCHER_SHA256:
        raise AuthorizationError("SOURCE_DRIFT", "v17 launcher SHA drift or unbound pin")
    if sha256_file(V17_PROFILE_PATH) != V17_PROFILE_SHA256:
        raise AuthorizationError("SOURCE_DRIFT", "v17 profile SHA drift")
    candidate = _load_v17()
    profile = candidate.verify_candidate_profile()
    if profile.get("sha256") != V17_PROFILE_SHA256 or profile.get("image_id") != IMAGE_ID:
        raise AuthorizationError("PROFILE_DRIFT", "v17 profile contract drift")
    return {"path": str(V17_PROFILE_PATH), "sha256": V17_PROFILE_SHA256,
            "launcher_path": str(V17_LAUNCHER_PATH), "launcher_sha256": V17_LAUNCHER_SHA256}


def _verify_lineage() -> Dict[str, Any]:
    candidate = _verify_candidate()
    build = _json_receipt(V17_BUILD_PATH, V17_BUILD_SHA256, V17_BUILD_SIDECAR_SHA256, "v17 build")
    if build.get("status") != "PASS" or build.get("image", {}).get("id") != IMAGE_ID or \
            build.get("image", {}).get("tag") != IMAGE_TAG or build.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("BUILD_LINEAGE", "v17 build identity/safety drift")
    no_input = _json_receipt(V17_NO_INPUT_PASS_PATH, V17_NO_INPUT_PASS_SHA256,
                             V17_NO_INPUT_PASS_SIDECAR_SHA256, "v17 no-input PASS")
    runtime = no_input.get("runtime", {})
    if no_input.get("status") != "PASS" or no_input.get("image", {}).get("id") != IMAGE_ID or \
            runtime.get("start_count") != 1 or runtime.get("network") != "none" or \
            runtime.get("rootfs") != "read_only" or runtime.get("host_mount_count") != 0 or \
            runtime.get("oom_killed") is not False or runtime.get("cleanup") != "stopped_only_remove_success":
        raise AuthorizationError("NO_INPUT_LINEAGE", "v17 no-input lineage drift")
    prior = _json_receipt(V17_NO_INPUT_FAIL_PATH, V17_NO_INPUT_FAIL_SHA256,
                          V17_NO_INPUT_FAIL_SIDECAR_SHA256, "v17 prior failure")
    if prior.get("status") != "FAIL_CLOSED" or prior.get("runtime", {}).get("start_count") != 1:
        raise AuthorizationError("FAILURE_LINEAGE", "v17 prior failure drift")
    closure_sidecar = V16_CLOSURE_PATH.with_name(V16_CLOSURE_PATH.name + ".sha256")
    closure = _json_receipt(V16_CLOSURE_PATH, V16_CLOSURE_SHA256, sha256_file(closure_sidecar), "v16 closure")
    if closure.get("status") != "FAIL_CLOSED" or closure.get("execution", {}).get("one_start") is not True or \
            any(closure.get("safety", {}).get(key) is not False for key in (
                "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("V16_LINEAGE", "v16 closure/cleanup drift")
    _regular(V16_TERMINAL_RAW_PATH, "v16 terminal raw")
    if sha256_file(V16_TERMINAL_RAW_PATH) != V16_TERMINAL_RAW_SHA256:
        raise AuthorizationError("V16_LINEAGE", "v16 terminal raw drift")
    return {
        "authorizer": {"path": str(SCRIPT), "sha256": sha256_file(SCRIPT)},
        "candidate": candidate, "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "receipts": {
            "v17_build": {"path": str(V17_BUILD_PATH), "sha256": V17_BUILD_SHA256, "sidecar_sha256": V17_BUILD_SIDECAR_SHA256},
            "v17_no_input_pass": {"path": str(V17_NO_INPUT_PASS_PATH), "sha256": V17_NO_INPUT_PASS_SHA256, "sidecar_sha256": V17_NO_INPUT_PASS_SIDECAR_SHA256},
            "v17_no_input_failure": {"path": str(V17_NO_INPUT_FAIL_PATH), "sha256": V17_NO_INPUT_FAIL_SHA256, "sidecar_sha256": V17_NO_INPUT_FAIL_SIDECAR_SHA256},
            "v16_closure": {"path": str(V16_CLOSURE_PATH), "sha256": V16_CLOSURE_SHA256},
            "v16_terminal_raw": {"path": str(V16_TERMINAL_RAW_PATH), "sha256": V16_TERMINAL_RAW_SHA256},
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
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(part, flags, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
    finally:
        try: part.unlink()
        except FileNotFoundError: pass
    os.chmod(path, mode, follow_symlinks=False)
    return hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> str:
    return _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _run_window(path: Path, *, proc_root: Path = Path("/proc"), now: Optional[str] = None) -> Mapping[str, Any]:
    excluded = quiescence.ancestor_pids(proc_root)
    observation = quiescence.collect_observation(proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
                                                  max_busy_percent=MAX_BUSY_PERCENT,
                                                  max_load_per_cpu=MAX_LOAD_PER_CPU,
                                                  excluded_pids=excluded)
    receipt = quiescence.build_receipt(observation, now=now)
    receipt.update({"authorization_window": True, "launcher_pid": os.getpid(),
                    "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, receipt)
    return {"path": str(path), "sha256": digest, "status": receipt.get("status"),
            "runner_start_allowed": receipt.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", [])}


def _sealed_windows_pass(windows: Any, root: Path) -> bool:
    if not isinstance(windows, list) or len(windows) != WINDOW_COUNT:
        return False
    for index, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != index:
            return False
        path = Path(str(item.get("path", "")))
        if path.parent.resolve() != root.resolve() or path.name != "quiescence_window_%02d.receipt.json" % index:
            return False
        try:
            _regular(path, "quiescence window")
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError, AuthorizationError):
            return False
        observation = value.get("observation", {})
        forbidden = observation.get("forbidden_processes", []) if isinstance(observation, Mapping) else []
        if value.get("schema_version") != 1 or value.get("contract_version") != "m6a10-quiescence-v1" or \
                value.get("status") != "PASS" or value.get("runner_start_allowed") is not True or forbidden:
            return False
        if sha256_file(path) != item.get("sha256") or item.get("status") != "PASS" or item.get("forbidden_processes"):
            return False
    return True


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / RECEIPT_NAME
    receipt_sha = _write_json(receipt, value)
    sidecar = receipt.with_name(receipt.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, (receipt_sha + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=receipt_sha,
                sidecar_path=str(sidecar), sidecar_sha256=sidecar_sha)


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
        runner = window_runner or _run_window
        windows: List[Dict[str, Any]] = []
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now)); result["window_index"] = index
            windows.append(result)
        passed = _sealed_windows_pass(windows, authorization_root)
        value: Dict[str, Any] = {
            "schema_version": 1, "contract_version": AUTHORIZATION_CONTRACT,
            "status": "AUTHORIZED" if passed else "FAIL_CLOSED", "authorized": passed,
            "formal_execution": passed, "formal_replay_forbidden": not passed,
            "formal_replay_started": False, "attempt_root": str(attempt_root),
            "attempt_count": 1, "retry": False, "manual_stop": False,
            "watchdog_seconds": WATCHDOG_SECONDS,
            "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
            "input": {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256,
                      "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS,
                      "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
                      "sensor_duration_seconds": SENSOR_DURATION_SECONDS},
            "mount_contract": {"network": "none", "rootfs": "read_only", "output": "/out",
                               "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                               "ground_truth_mount": False, "scorer_mount": False, "map_mount": False},
            "one_start": True, "monitor": {"continuous": True, "interval_seconds": 4.0,
                                              "natural_completion": True}, "lineage": lineage,
            "windows": windows,
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        }
        if not passed: value["failure_kind"] = "QUIESCENCE_FAIL_CLOSED"
        return _seal(authorization_root, value)
    except Exception as error:
        if (authorization_root / RECEIPT_NAME).exists(): raise
        return _seal(authorization_root, {
            "schema_version": 1, "contract_version": AUTHORIZATION_CONTRACT,
            "status": "FAIL_CLOSED", "authorized": False, "formal_execution": False,
            "formal_replay_forbidden": True, "formal_replay_started": False,
            "attempt_root": str(attempt_root), "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(error),
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        })


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Dict[str, Any]:
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "v17 authorization sidecar")
    value = _json_receipt(path, expected_sha256, sha256_file(sidecar), "v17 authorization")
    lineage = _verify_lineage()
    if value.get("contract_version") != AUTHORIZATION_CONTRACT or value.get("status") != "AUTHORIZED" or \
            value.get("authorized") is not True or value.get("formal_execution") is not True or \
            value.get("formal_replay_forbidden") is not False or value.get("formal_replay_started") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "v17 authorization is not executable")
    if value.get("attempt_root") != str(attempt_root):
        raise AuthorizationError("AUTHORIZATION_ROOT", "v17 exact attempt root mismatch")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSE", "v17 attempt root already exists")
    if value.get("attempt_count") != 1 or value.get("retry") is not False or value.get("manual_stop") is not False or \
            value.get("watchdog_seconds") != WATCHDOG_SECONDS:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "v17 one-start/retry contract drift")
    if value.get("image", {}).get("id") != IMAGE_ID or value.get("image", {}).get("tag") != IMAGE_TAG:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "v17 image identity drift")
    if not _sealed_windows_pass(value.get("windows"), path.parent):
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "v17 authorization windows are not PASS")
    if value.get("input", {}).get("path") != INPUT_PATH or value.get("input", {}).get("bytes") != INPUT_BYTES or \
            value.get("input", {}).get("sha256") != INPUT_SHA256 or value.get("input", {}).get("expected_messages") != EXPECTED_MESSAGES or \
            value.get("input", {}).get("expected_topic_counts") != EXPECTED_COUNTS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "v17 input contract drift")
    expected_mount = {"network": "none", "rootfs": "read_only", "output": "/out",
                      "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                      "ground_truth_mount": False, "scorer_mount": False, "map_mount": False}
    if value.get("mount_contract") != expected_mount:
        raise AuthorizationError("AUTHORIZATION_MOUNTS", "v17 mount contract drift")
    monitor = value.get("monitor")
    if not isinstance(monitor, Mapping) or monitor.get("continuous") is not True or monitor.get("interval_seconds") != 4.0 or monitor.get("natural_completion") is not True:
        raise AuthorizationError("AUTHORIZATION_MONITOR", "v17 monitor contract drift")
    if value.get("lineage", {}).get("authorizer", {}).get("path") != str(SCRIPT) or \
            value.get("lineage", {}).get("authorizer", {}).get("sha256") != sha256_file(SCRIPT) or \
            value.get("lineage", {}).get("authorizer") != lineage.get("authorizer"):
        raise AuthorizationError("AUTHORIZER_DRIFT", "v17 authorizer source binding drift")
    if value.get("lineage", {}).get("formal_replay_forbidden_lineage") is not True:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v17 forbidden lineage drift")
    if any(value.get("safety", {}).get(key) is not False for key in ("input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "v17 authorization safety drift")
    return dict(value, authorized=True)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = authorize(args.authorization_root, args.attempt_root)
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "AUTHORIZATION_FAIL_CLOSED"), "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
