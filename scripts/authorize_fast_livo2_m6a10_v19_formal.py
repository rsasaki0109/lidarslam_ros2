#!/usr/bin/env python3
"""Create and verify the additive v19 exact-root formal authorization.

This host-only authorizer verifies the real v12 adapter and immutable v17
production lineage, then takes exactly three contiguous read-only quiescence
windows.  It never opens the bag, starts Docker/ROS, invokes a scorer, or
writes a map.  A failed window seals a fail-closed authorization and cannot be
retried or reused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402
import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as adapter  # noqa: E402


AUTHORIZATION_CONTRACT = "m6a10-v19-formal-exact-root-authorization-v1"
QUIESCENCE_CONTRACT = "m6a10-quiescence-v1"
AUTHORIZATION_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v19_formal_authorization_20260824T005300Z_agentv19"
)
AUTHORIZATION_RECEIPT_PATH = AUTHORIZATION_ROOT / "formal_authorization.receipt.json"
ATTEMPT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v19_formal_replay_20260824T005300Z_agentv19formal"
)
V19_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v19_formal_candidate.yaml"
V19_ADAPTER_PATH = ROOT / "scripts/fast_livo2_m6a10_v19_base_adapter.py"
V19_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v19_formal.py"
V19_AUTHORIZER_PATH = Path(__file__).resolve()
V19_BASE_MODULE_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v12_formal.py"
V17_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v17_formal.py"
V17_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_ready.yaml"
V17_LAUNCHER_SHA256 = "107ead9612a32f41cef0c89f18affe00e97553cfb370a06abdcaf64f2e29446d"
V17_PROFILE_SHA256 = "10ddffb6eed647eb3b7a2ee7ede49505923755eb917d8b87737675853e35d9c5"
V17_IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
V17_IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
V17_WRAPPER_SHA256 = "506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e"
V17_FEEDER_SHA256 = "6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
V17_PATCH_SHA256 = "c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001"
V17_BUILD_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json"
)
V17_BUILD_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
V17_BUILD_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
V17_NO_INPUT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/no_input.receipt.json"
)
V17_NO_INPUT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
V17_NO_INPUT_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"
V17_NO_INPUT_FAILURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction_20260823T224017Z_agentv17/no_input.receipt.json"
)
V17_NO_INPUT_FAILURE_SHA256 = "93296e2d47e37aeea739daf3c44d4c7bf988c7a9bcfc61f11a4cbab31ec1203a"
V17_NO_INPUT_FAILURE_SIDECAR_SHA256 = "a0db9e7c7303974edc575033088bc92e0bc14ab8db7eda054ae16c1ec4d9c2f7"
V18B_AUTHORIZATION_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/"
    "m6a10_training_20260824/fast_livo2_v2c_v18_formal_authorization_20260824T003000Z_agentv18b/"
    "formal_authorization.receipt.json"
)
V18B_AUTHORIZATION_SHA256 = "78fc1ea039f4c8725a773a81dafca64486341a14eb4c052ddbba3f8d1ee3a291"
V18B_AUTHORIZATION_SIDECAR_SHA256 = "14a84f3fd29b829d151214e3dd611215c07eb4e92511a8bb2ebd727d0d4cef80"
V18B_CLOSURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/"
    "m6a10_training_20260824/fast_livo2_v2c_v18_formal_replay_20260824T003000Z_agentv18bformal/"
    "closure_receipt.json"
)
V18B_CLOSURE_SHA256 = "1e532bbf58c5bfffc7315a2b228342228c96fc45fe799bd2a0893605841550fd"
V18B_CLOSURE_SIDECAR_SHA256 = "267dbf0ac1426645f2cac2e1a5c639c942c51e72c063da64ce57201eabfc1901"
V18B_ATTEMPT_ROOT = "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v18_formal_replay_20260824T003000Z_agentv18bformal"
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
WINDOW_SECONDS = 4.0
MAX_BUSY_PERCENT = 5.0
MAX_LOAD_PER_CPU = 0.50


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


def _regular(path: Path, label: str, *, immutable: bool = False) -> None:
    current = Path(path.absolute().anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if not os.path.lexists(path) or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not a regular file" % label)
    if immutable and (path.stat().st_mode & 0o777) != 0o444:
        raise AuthorizationError("IMMUTABILITY", "%s is not mode 0444" % label)


def _read_receipt(path: Path, expected_sha: str, expected_sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label, immutable=True)
    observed = sha256_file(path)
    if observed != expected_sha:
        raise AuthorizationError("RECEIPT_SHA", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label, immutable=True)
    if sha256_file(sidecar) != expected_sidecar_sha or \
            sidecar.read_bytes() != (expected_sha + "  " + path.name + "\n").encode("ascii"):
        raise AuthorizationError("RECEIPT_SIDECAR", "%s sidecar drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_JSON", "%s is not JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_JSON", "%s must be an object" % label)
    return value


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise AuthorizationError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise AuthorizationError("OUTPUT_STAGING", "staging file exists: %s" % part)
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


def _verify_v17_lineage(repo_root: Path) -> Dict[str, Any]:
    if sha256_file(repo_root / V17_LAUNCHER_PATH.relative_to(ROOT)) != V17_LAUNCHER_SHA256:
        raise AuthorizationError("V17_SOURCE_DRIFT", "v17 launcher SHA drift")
    if sha256_file(V17_PROFILE_PATH) != V17_PROFILE_SHA256:
        raise AuthorizationError("V17_PROFILE_DRIFT", "v17 profile SHA drift")
    surface = adapter.load_base_surface()
    runtime = adapter.load_v17_runtime_surface(surface)
    try:
        v17_profile = runtime.verify_candidate_profile(runtime.profile_path, repo_root)
    except Exception as exc:
        raise AuthorizationError("V17_PROFILE_INVALID", str(exc)) from exc
    build = _read_receipt(V17_BUILD_PATH, V17_BUILD_SHA256, V17_BUILD_SIDECAR_SHA256, "v17 build")
    if build.get("status") != "PASS" or build.get("image", {}).get("id") != V17_IMAGE_ID or \
            build.get("safety", {}).get("formal_replay_started") is not False:
        raise AuthorizationError("V17_BUILD_INVALID", "v17 build receipt is not admissible")
    no_input = _read_receipt(V17_NO_INPUT_PATH, V17_NO_INPUT_SHA256, V17_NO_INPUT_SIDECAR_SHA256, "v17 no-input")
    no_runtime = no_input.get("runtime", {})
    if no_input.get("status") != "PASS" or no_input.get("image", {}).get("id") != V17_IMAGE_ID or \
            no_runtime.get("start_count") != 1 or no_runtime.get("network") != "none" or \
            no_runtime.get("rootfs") != "read_only" or no_runtime.get("host_mount_count") != 0 or \
            no_runtime.get("oom_killed") is not False or no_runtime.get("cleanup") != "stopped_only_remove_success":
        raise AuthorizationError("V17_NO_INPUT_INVALID", "v17 no-input receipt is not admissible")
    failed = _read_receipt(V17_NO_INPUT_FAILURE_PATH, V17_NO_INPUT_FAILURE_SHA256,
                            V17_NO_INPUT_FAILURE_SIDECAR_SHA256, "v17 no-input failure")
    if failed.get("status") != "FAIL_CLOSED":
        raise AuthorizationError("V17_FAILURE_LINEAGE", "v17 failure lineage drift")
    v18b = adapter.verify_v18b_lineage(
        authorization_path=V18B_AUTHORIZATION_PATH,
        authorization_sha256=V18B_AUTHORIZATION_SHA256,
        authorization_sidecar_sha256=V18B_AUTHORIZATION_SIDECAR_SHA256,
        closure_path=V18B_CLOSURE_PATH,
        closure_sha256=V18B_CLOSURE_SHA256,
        closure_sidecar_sha256=V18B_CLOSURE_SIDECAR_SHA256,
        expected_attempt_root=V18B_ATTEMPT_ROOT,
    )
    return {
        "v12_adapter": {"path": str(V19_ADAPTER_PATH), "sha256": sha256_file(V19_ADAPTER_PATH),
                        "base_module_path": str(V19_BASE_MODULE_PATH),
                        "base_module_sha256": surface.module_sha256},
        "v17": {"launcher_path": str(V17_LAUNCHER_PATH), "launcher_sha256": V17_LAUNCHER_SHA256,
                 "profile_path": str(V17_PROFILE_PATH), "profile_sha256": V17_PROFILE_SHA256,
                 "image": {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID},
                 "runtime_surface": runtime.as_identity(), "profile": v17_profile,
                 "build": {"path": str(V17_BUILD_PATH), "sha256": V17_BUILD_SHA256,
                            "sidecar_sha256": V17_BUILD_SIDECAR_SHA256},
                 "no_input": {"path": str(V17_NO_INPUT_PATH), "sha256": V17_NO_INPUT_SHA256,
                              "sidecar_sha256": V17_NO_INPUT_SIDECAR_SHA256},
                 "prior_no_input_failure": {"path": str(V17_NO_INPUT_FAILURE_PATH),
                                             "sha256": V17_NO_INPUT_FAILURE_SHA256}},
        "v18b": v18b,
        "feeder_sha256": V17_FEEDER_SHA256,
        "wrapper_sha256": V17_WRAPPER_SHA256,
        "patch_sha256": V17_PATCH_SHA256,
    }


def _verify_v19_profile(repo_root: Path, adapter_sha: str) -> str:
    path = repo_root / V19_PROFILE_PATH.relative_to(ROOT)
    if path.resolve() != V19_PROFILE_PATH.resolve() or sha256_file(path) == "":
        raise AuthorizationError("V19_PROFILE_PATH", "v19 profile path drift")
    try:
        import yaml
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AuthorizationError("V19_PROFILE_INVALID", "v19 profile is not valid YAML") from exc
    candidate = document.get("candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    if document.get("schema_version") != 1 or document.get("status") != "V19_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or not isinstance(candidate, Mapping) or \
            candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("base_adapter_sha256") != adapter_sha or \
            not isinstance(image, Mapping) or image.get("id") != V17_IMAGE_ID or image.get("tag") != V17_IMAGE_TAG:
        raise AuthorizationError("V19_PROFILE_INVALID", "v19 profile identity/status drift")
    return sha256_file(path)


def _run_window(path: Path, *, proc_root: Path = Path("/proc"), now: Optional[str] = None) -> Mapping[str, Any]:
    excluded = quiescence.ancestor_pids(proc_root, pid=os.getpid())
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    value = quiescence.build_receipt(observation, now=now)
    window_index = int(path.name.split("_")[-1].split(".")[0])
    value.update({"authorization_window": True, "window_index": window_index,
                  "launcher_pid": os.getpid(), "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, value)
    return {"path": str(path), "sha256": digest, "status": value.get("status"),
            "runner_start_allowed": value.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", [])}


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / AUTHORIZATION_RECEIPT_PATH.name
    digest = _write_json(receipt, value)
    sidecar = receipt.with_name(receipt.name + ".sha256")
    sidecar_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=sidecar_digest)


WindowRunner = Callable[..., Mapping[str, Any]]


def authorize(*, authorization_root: Path = AUTHORIZATION_ROOT, attempt_root: Path = ATTEMPT_ROOT,
              repo_root: Path = ROOT, proc_root: Path = Path("/proc"), now: Optional[str] = None,
              window_runner: Optional[WindowRunner] = None) -> Dict[str, Any]:
    if authorization_root.resolve() != AUTHORIZATION_ROOT.resolve() or attempt_root.resolve() != ATTEMPT_ROOT.resolve():
        raise AuthorizationError("AUTHORIZATION_ROOT", "v19 roots differ from fixed exact roots")
    if os.path.lexists(authorization_root) or os.path.lexists(attempt_root):
        raise AuthorizationError("ROOT_NOT_FRESH", "v19 authorization/formal roots must be absent")
    authorization_root.mkdir(parents=True)
    try:
        lineage = _verify_v17_lineage(repo_root)
        profile_sha = sha256_file(V19_PROFILE_PATH)
        adapter_sha = sha256_file(V19_ADAPTER_PATH)
        profile_sha = _verify_v19_profile(repo_root, adapter_sha)
        launcher_sha = sha256_file(V19_LAUNCHER_PATH)
        authorizer_sha = sha256_file(V19_AUTHORIZER_PATH)
        windows: List[Dict[str, Any]] = []
        runner = window_runner or _run_window
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now))
            result["window_index"] = index
            windows.append(result)
        passed = len(windows) == WINDOW_COUNT and all(
            item.get("window_index") == index and item.get("status") == "PASS" and
            item.get("runner_start_allowed") is True and not item.get("forbidden_processes")
            for index, item in enumerate(windows, 1)
        )
        value: Dict[str, Any] = {
            "schema_version": 1, "contract_version": AUTHORIZATION_CONTRACT,
            "status": "AUTHORIZED" if passed else "FAIL_CLOSED", "authorized": passed,
            "formal_execution": passed, "formal_replay_forbidden": not passed,
            "formal_replay_started": False, "attempt_root": str(attempt_root),
            "attempt_count": 1, "one_start": True, "retry": False, "manual_stop": False,
            "watchdog_seconds": WATCHDOG_SECONDS,
            "image": {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID},
            "input": {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256,
                      "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS,
                      "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
                      "sensor_duration_seconds": SENSOR_DURATION_SECONDS},
            "mount_contract": {"network": "none", "rootfs": "read_only", "output": "/out",
                               "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                               "ground_truth_mount": False, "scorer_mount": False, "map_mount": False,
                               "wrapper_readonly": True, "feeder_readonly": True,
                               "duplicate_output_destinations": False},
            "monitor": {"continuous": True, "interval_seconds": 4.0, "natural_completion": True,
                        "host_interference_stop": False},
            "lineage": dict(lineage, v19={
                "adapter_path": str(V19_ADAPTER_PATH), "adapter_sha256": adapter_sha,
                "launcher_path": str(V19_LAUNCHER_PATH), "launcher_sha256": launcher_sha,
                "profile_path": str(V19_PROFILE_PATH), "profile_sha256": profile_sha,
                "authorizer_path": str(V19_AUTHORIZER_PATH), "authorizer_sha256": authorizer_sha,
            }),
            "windows": windows,
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                        "scorer_invoked": False, "map_saved": False},
        }
        if not passed:
            value["failure_kind"] = "QUIESCENCE_FAIL_CLOSED"
        sealed = _seal(authorization_root, value)
        if passed:
            verify_authorization(Path(sealed["receipt_path"]), attempt_root,
                                 sealed["receipt_sha256"], repo_root=repo_root)
        return sealed
    except Exception as exc:
        receipt = authorization_root / AUTHORIZATION_RECEIPT_PATH.name
        if receipt.exists():
            raise
        return _seal(authorization_root, {
            "schema_version": 1, "contract_version": AUTHORIZATION_CONTRACT,
            "status": "FAIL_CLOSED", "authorized": False, "formal_execution": False,
            "formal_replay_forbidden": True, "formal_replay_started": False,
            "attempt_root": str(attempt_root), "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(exc),
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                       "scorer_invoked": False, "map_saved": False},
        })


def _verify_windows(value: Mapping[str, Any], receipt_path: Path) -> None:
    windows = value.get("windows")
    if not isinstance(windows, list) or len(windows) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "exactly three windows are required")
    seen_paths = set()
    seen_hashes = set()
    for index, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != index or \
                item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or \
                item.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window %d is not PASS" % index)
        path = Path(str(item.get("path", "")))
        if path.parent.resolve() != receipt_path.parent.resolve() or \
                path.name != "quiescence_window_%02d.receipt.json" % index:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window path drift")
        _regular(path, "window %d" % index, immutable=True)
        observed = sha256_file(path)
        if observed != item.get("sha256"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window %d SHA drift" % index)
        document = json.loads(path.read_text(encoding="utf-8"))
        observation = document.get("observation", {})
        if document.get("schema_version") != 1 or document.get("contract_version") != QUIESCENCE_CONTRACT or \
                document.get("status") != "PASS" or document.get("runner_start_allowed") is not True or \
                observation.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window %d contract drift" % index)
        seen_paths.add(str(path)); seen_hashes.add(observed)
    if len(seen_paths) != WINDOW_COUNT or len(seen_hashes) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "windows are not distinct")


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Dict[str, Any]:
    if path.resolve() != AUTHORIZATION_RECEIPT_PATH.resolve() or attempt_root.resolve() != ATTEMPT_ROOT.resolve():
        raise AuthorizationError("AUTHORIZATION_ROOT", "v19 exact path/root mismatch")
    value = _read_receipt(path, expected_sha256, sha256_file(path.with_name(path.name + ".sha256")),
                          "v19 authorization")
    if value.get("schema_version") != 1 or value.get("contract_version") != AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False or value.get("attempt_root") != str(attempt_root):
        raise AuthorizationError("AUTHORIZATION_STATUS", "v19 authorization is not executable")
    if value.get("attempt_count") != 1 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False or \
            value.get("watchdog_seconds") != WATCHDOG_SECONDS:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "one-start/retry contract drift")
    if value.get("image") != {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID}:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "v17 image identity drift")
    if value.get("input") != {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256,
                                "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS,
                                "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
                                "sensor_duration_seconds": SENSOR_DURATION_SECONDS}:
        raise AuthorizationError("AUTHORIZATION_INPUT", "input contract drift")
    expected_mount = {"network": "none", "rootfs": "read_only", "output": "/out",
                      "output_readonly": False, "output_tmpfs": False, "input_mounts": 1,
                      "ground_truth_mount": False, "scorer_mount": False, "map_mount": False,
                      "wrapper_readonly": True, "feeder_readonly": True,
                      "duplicate_output_destinations": False}
    if value.get("mount_contract") != expected_mount:
        raise AuthorizationError("AUTHORIZATION_MOUNTS", "mount contract drift")
    monitor = value.get("monitor")
    if monitor != {"continuous": True, "interval_seconds": 4.0, "natural_completion": True,
                   "host_interference_stop": False}:
        raise AuthorizationError("AUTHORIZATION_MONITOR", "monitor contract drift")
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "safety flags drift")
    lineage = value.get("lineage")
    if not isinstance(lineage, Mapping) or lineage.get("v19", {}).get("adapter_path") != str(V19_ADAPTER_PATH) or \
            lineage.get("v19", {}).get("launcher_path") != str(V19_LAUNCHER_PATH):
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v19 source lineage missing")
    for key, path_value in (("adapter_path", V19_ADAPTER_PATH), ("launcher_path", V19_LAUNCHER_PATH),
                            ("profile_path", V19_PROFILE_PATH), ("authorizer_path", V19_AUTHORIZER_PATH)):
        entry = lineage["v19"]
        expected = entry.get(key.replace("_path", "_sha256"))
        if expected != sha256_file(path_value):
            raise AuthorizationError("AUTHORIZATION_LINEAGE", "%s SHA drift" % key)
    current = _verify_v17_lineage(repo_root)
    _verify_v19_profile(repo_root, lineage["v19"].get("adapter_sha256", ""))
    if lineage.get("v12_adapter") != current.get("v12_adapter") or \
            lineage.get("v18b") != current.get("v18b") or \
            lineage.get("feeder_sha256") != V17_FEEDER_SHA256 or \
            lineage.get("wrapper_sha256") != V17_WRAPPER_SHA256 or \
            lineage.get("patch_sha256") != V17_PATCH_SHA256:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "adapter/v18b/source lineage drift")
    if lineage.get("v17", {}).get("image") != current["v17"]["image"] or \
            lineage.get("v17", {}).get("build") != current["v17"]["build"] or \
            lineage.get("v17", {}).get("no_input") != current["v17"]["no_input"]:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v17 lineage drift")
    _verify_windows(value, path)
    return {"authorized": True, "formal_execution": True, "status": "AUTHORIZED",
            "attempt_root": str(attempt_root), "image_id": V17_IMAGE_ID,
            "receipt_path": str(path), "receipt_sha256": expected_sha256,
            "lineage": lineage, "windows": value.get("windows")}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION_ROOT)
    parser.add_argument("--attempt-root", type=Path, default=ATTEMPT_ROOT)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--create", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.create:
            value = authorize(authorization_root=args.authorization, attempt_root=args.attempt_root)
        else:
            if not args.authorization_sha256:
                raise AuthorizationError("AUTHORIZATION_SHA_REQUIRED", "authorization SHA is required")
            value = verify_authorization(args.authorization, args.attempt_root, args.authorization_sha256)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: value.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")},
                     sort_keys=True))
    return 0 if value.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
