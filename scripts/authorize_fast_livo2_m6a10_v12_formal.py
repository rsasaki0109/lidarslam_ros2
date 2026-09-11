#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Create one exact-root v12 formal authorization, or seal a failure.

This authorizer is additive to the v12 candidate profile.  It never opens the
bag, starts Docker/ROS, invokes GT/scoring, or retries a root.  Three fixed
five-second read-only procfs windows are required before an authorization can
be sealed.  The attempt root is bound while still absent; the formal launcher
is the only process allowed to reserve it after verifying this receipt.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_candidate.yaml"
PROFILE_SHA256 = "46e2f91d4cd1c2d84de499e0a45b7a1c88aa684ed8439923739946bb3b0ce207"
READY_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml"
READY_PROFILE_SHA256 = "675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d"
IMAGE_TAG = (
    "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-"
    "fast-livo2-benchmark:ros1-pinned"
)
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_build_20260823T161850Z_agentv12/build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "e0e5c924025a24083661a6838bc5af28f07c34c419ce6e103c37890ed1382dda"
ATTEMPT6_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_dual_service_attempt6_20260823T175000Z_agentv12/"
    "no_input_dual_evidence.receipt.json"
)
ATTEMPT6_RECEIPT_SHA256 = "c5afa2452f96588bf46dc7f0d0b4504e816d0069bbd3a107009eb2c3beac0b45"
HOST_GATE_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_host_evidence_gate_20260823T180718Z_agentv12/"
    "host_evidence_gate.receipt.json"
)
HOST_GATE_RECEIPT_SHA256 = "3a603bc4ece0e9167b0b3943df6be569f2e64bce196beb18da29ff09ad69f643"
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
CONTRACT_VERSION = "m6a10-v12-formal-exact-root-authorization-v1"
RECEIPT_NAME = "formal_authorization.receipt.json"


def _extra_forbidden_processes(proc_root: Path, excluded: Iterable[int]) -> List[Dict[str, Any]]:
    """Catch external GNSS builders not covered by the legacy regex helper."""
    excluded_set = set(excluded)
    matches: List[Dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded_set:
            continue
        pid = int(entry.name)
        try:
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
            command_bytes = (entry / "cmdline").read_bytes()
            command = command_bytes.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
            lowered = (comm + " " + command).lower()
        except (OSError, UnicodeError):
            continue
        classification = None
        if (comm.lower() == "gnss_ppp" or re.search(r"(?:^|[\s/])gnss_ppp(?:$|[\s])", lowered) or
                comm.lower() == "rtk" or re.search(r"(?:^|[\s/])rtk(?:$|[\s])", lowered)):
            classification = "external_gnss_build"
        elif "cc1plus" in lowered or "gcc" in lowered or "g++" in lowered or "clang" in lowered:
            classification = "compiler"
        if classification is not None:
            matches.append({
                "pid": pid, "comm": comm[:128], "class": classification,
                "argv_sha256": hashlib.sha256(command.encode("utf-8")).hexdigest(),
            })
    return sorted(matches, key=lambda item: (item["class"], item["pid"]))


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


def _pin(path: Path, expected: str, label: str) -> None:
    _regular(path, label)
    if sha256_file(path) != expected:
        raise AuthorizationError("SOURCE_DRIFT", "%s SHA drift" % label)


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise AuthorizationError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise AuthorizationError("OUTPUT_STAGING", "staging file exists: %s" % part)
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb", closefd=True) as stream:
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


def _verify_immutable_lineage(repo_root: Path) -> Dict[str, Any]:
    _pin(PROFILE_PATH, PROFILE_SHA256, "candidate profile")
    _pin(READY_PROFILE_PATH, READY_PROFILE_SHA256, "ready profile")
    source_pins = {
        "v12_delta_patch": ("docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch", "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"),
        "v12_wrapper": ("scripts/fast_livo2_m6a10_v12_formal_container_run.sh", "af6fa54f834ae209758ee5a9903695674f4095c1063ac57b46c2774bfbc5c7a9"),
        "binder": ("scripts/bind_fast_livo2_v12_consumer_evidence.py", "3a708c87137f81330b479579dbcb184ac25853596546263c406473dbef106aab"),
        "compositor": ("scripts/compose_fast_livo2_v12_terminal_evidence.py", "e37c2f22ac6220f7cfc5739d27f7c24360c395bac7b2dd0f90ff80ca9e8db99f"),
        "monitor": ("scripts/monitor_m6a10_host_interference.py", "d19cac9b7755b30e7cf45adcdb3fe7869855c4eb9814e337a50b102048dd5e7d"),
    }
    observed = {}
    for name, (relative, expected) in source_pins.items():
        path = repo_root / relative
        _pin(path, expected, name)
        observed[name] = {"path": str(path.resolve()), "sha256": expected}
    build = _pin_external_receipt(BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, "build")
    attempt = _pin_external_receipt(ATTEMPT6_RECEIPT_PATH, ATTEMPT6_RECEIPT_SHA256, "attempt6")
    host = _pin_external_receipt(HOST_GATE_RECEIPT_PATH, HOST_GATE_RECEIPT_SHA256, "host_evidence")
    for value, name in ((build, "build"), (attempt, "attempt6"), (host, "host_evidence")):
        if value.get("status") != "PASS":
            raise AuthorizationError("RECEIPT_STATUS", "%s receipt is not PASS" % name)
    if build.get("image_id") != IMAGE_ID or build.get("image_tag") != IMAGE_TAG:
        raise AuthorizationError("IMAGE_IDENTITY", "build receipt image identity drift")
    if attempt.get("attempt_index") != 6 or attempt.get("status") != "PASS":
        raise AuthorizationError("ATTEMPT6_IDENTITY", "attempt6 receipt identity drift")
    if host.get("status") != "PASS" or host.get("formal_replay_started") is not False:
        raise AuthorizationError("HOST_GATE_IDENTITY", "host gate receipt identity drift")
    return {"profile_sha256": PROFILE_SHA256, "sources": observed}


def _pin_external_receipt(path: Path, expected_sha: str, label: str) -> Dict[str, Any]:
    _pin(path, expected_sha, "%s receipt" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_INVALID", "%s receipt invalid" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_INVALID", "%s receipt is not object" % label)
    return value


def _run_window(path: Path, *, proc_root: Path, now: Optional[str] = None) -> Dict[str, Any]:
    excluded = quiescence.ancestor_pids(proc_root)
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    extra_forbidden = _extra_forbidden_processes(proc_root, excluded)
    if extra_forbidden:
        observation["forbidden_processes"] = list(observation.get("forbidden_processes", [])) + extra_forbidden
        observation["checks"]["no_forbidden_processes"] = False
    receipt = quiescence.build_receipt(observation, now=now)
    receipt["authorization_window"] = True
    receipt["launcher_pid"] = os.getpid()
    receipt["excluded_ancestor_pids"] = sorted(excluded)
    _write_json(path, receipt)
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "status": receipt["status"],
        "runner_start_allowed": receipt["runner_start_allowed"],
        "observation": observation,
    }


def authorize(
        authorization_root: Path, attempt_root: Path, *, repo_root: Path = ROOT,
        proc_root: Path = Path("/proc"), now: Optional[str] = None,
        window_runner: Any = None,
) -> Dict[str, Any]:
    """Seal one authorization; a failed window seals FAIL_CLOSED and stops."""
    if os.path.lexists(authorization_root):
        raise AuthorizationError("AUTH_ROOT_NOT_FRESH", "authorization root already exists")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_NOT_FRESH", "attempt root must be absent before authorization")
    lineage: Optional[Dict[str, Any]] = None
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    windows: List[Dict[str, Any]] = []
    authorization_root.mkdir(parents=True)
    try:
        lineage = _verify_immutable_lineage(repo_root)
        window_runner = _run_window if window_runner is None else window_runner
        for index in range(1, WINDOW_COUNT + 1):
            window_path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            try:
                result = window_runner(window_path, proc_root=proc_root, now=now)
            except Exception as exc:
                result = {
                    "path": str(window_path), "sha256": None,
                    "status": "FAIL_CLOSED", "runner_start_allowed": False,
                    "error": "%s: %s" % (type(exc).__name__, exc),
                }
            if result.get("path") and result.get("sha256"):
                try:
                    window_file = Path(str(result["path"]))
                    _regular(window_file, "quiescence window")
                    if sha256_file(window_file) != result["sha256"]:
                        raise AuthorizationError("QUIESCENCE_SHA", "quiescence window SHA drift")
                except Exception as exc:
                    result = {
                        "path": str(result.get("path", window_path)),
                        "sha256": None,
                        "status": "FAIL_CLOSED",
                        "runner_start_allowed": False,
                        "error": "%s: %s" % (type(exc).__name__, exc),
                    }
            windows.append(result)
            if result.get("status") != "PASS" or result.get("runner_start_allowed") is not True:
                failure_kind = failure_kind or "QUIESCENCE_FAIL_CLOSED"
                failure_message = failure_message or "quiescence window %d did not PASS" % index
    except AuthorizationError as exc:
        failure_kind = exc.kind
        failure_message = str(exc)
    except Exception as exc:
        failure_kind = "AUTHORIZATION_PREFLIGHT_FAIL_CLOSED"
        failure_message = "%s: %s" % (type(exc).__name__, exc)
    status = "AUTHORIZED" if failure_kind is None and len(windows) == WINDOW_COUNT else "FAIL_CLOSED"
    receipt: Dict[str, Any] = {
        "schema_version": 1,
        "contract_version": CONTRACT_VERSION,
        "status": status,
        "authorization_root": str(authorization_root.resolve()),
        "attempt_root": str(attempt_root),
        "attempt_root_must_be_absent": True,
        "formal_replay_authorized": status == "AUTHORIZED",
        "formal_execution": status == "AUTHORIZED",
        "replay_count": 0,
        "one_start": True,
        "retry": False,
        "manual_stop": False,
        "phase_contract": PHASE_CONTRACT,
        "transport_contract": TRANSPORT_CONTRACT,
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "input": {"path": INPUT_PATH, "bytes": INPUT_BYTES, "sha256": INPUT_SHA256, "expected_messages": EXPECTED_MESSAGES, "expected_topic_counts": EXPECTED_COUNTS},
        "profile": {"path": str(PROFILE_PATH.resolve()), "sha256": PROFILE_SHA256},
        "lineage": lineage,
        "quiescence": {
            "window_count": WINDOW_COUNT,
            "window_seconds": WINDOW_SECONDS,
            "max_busy_percent": MAX_BUSY_PERCENT,
            "max_load_per_cpu": MAX_LOAD_PER_CPU,
            "consecutive_passes": len(windows) if status == "AUTHORIZED" else 0,
            "windows": windows,
            "forbidden_processes": [],
        },
        "safety": {
            "input_opened": False,
            "ground_truth_content_opened": False,
            "scorer_invoked": False,
            "map_saved": False,
            "formal_replay_started": False,
        },
        "authorized_delta": {
            "candidate_profile_formal_replay_forbidden": True,
            "supersedes_for_exact_attempt_root_only": status == "AUTHORIZED",
            "no_other_root_authorized": True,
        },
        "failure_kind": failure_kind,
        "failure_message": failure_message,
        "created_at_utc": now or dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    receipt_path = authorization_root / RECEIPT_NAME
    receipt_sha = _write_json(receipt_path, receipt)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, ("%s  %s\n" % (receipt_sha, receipt_path.name)).encode("ascii"))
    receipt["receipt_path"] = str(receipt_path)
    receipt["receipt_sha256"] = receipt_sha
    receipt["sidecar_path"] = str(sidecar)
    receipt["sidecar_sha256"] = sidecar_sha
    return receipt


def verify_authorization(
        receipt_path: Path, attempt_root: Path, expected_sha256: str,
        *, repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify one sealed authorization before any input/image probe."""
    _regular(receipt_path, "authorization receipt")
    observed = sha256_file(receipt_path)
    if observed != expected_sha256:
        raise AuthorizationError("AUTHORIZATION_SHA", "authorization receipt SHA drift")
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    _regular(sidecar, "authorization sidecar")
    if sidecar.read_text(encoding="ascii") != "%s  %s\n" % (observed, receipt_path.name):
        raise AuthorizationError("AUTHORIZATION_SIDECAR", "authorization sidecar content drift")
    try:
        value = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("AUTHORIZATION_JSON", "authorization receipt is invalid") from exc
    if not isinstance(value, dict) or value.get("schema_version") != 1 or \
            value.get("contract_version") != CONTRACT_VERSION or value.get("status") != "AUTHORIZED" or \
            value.get("formal_replay_authorized") is not True or value.get("formal_execution") is not True or \
            value.get("replay_count") != 0 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False:
        raise AuthorizationError("AUTHORIZATION_STATUS", "authorization is not an exact one-start grant")
    if value.get("attempt_root") != str(attempt_root) or value.get("attempt_root_must_be_absent") is not True:
        raise AuthorizationError("AUTHORIZATION_ROOT", "authorization attempt root differs")
    if os.path.lexists(attempt_root):
        raise AuthorizationError("ATTEMPT_ROOT_REUSED", "authorized attempt root already exists")
    if value.get("phase_contract") != PHASE_CONTRACT or value.get("transport_contract") != TRANSPORT_CONTRACT:
        raise AuthorizationError("AUTHORIZATION_CONTRACT", "authorization phase/transport drift")
    image = value.get("image")
    if not isinstance(image, Mapping) or image.get("tag") != IMAGE_TAG or image.get("id") != IMAGE_ID:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "authorization image identity drift")
    input_value = value.get("input")
    if not isinstance(input_value, Mapping) or input_value.get("path") != INPUT_PATH or \
            input_value.get("bytes") != INPUT_BYTES or input_value.get("sha256") != INPUT_SHA256 or \
            input_value.get("expected_messages") != EXPECTED_MESSAGES or \
            input_value.get("expected_topic_counts") != EXPECTED_COUNTS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "authorization input identity drift")
    profile = value.get("profile")
    if not isinstance(profile, Mapping) or profile.get("path") != str(PROFILE_PATH.resolve()) or \
            profile.get("sha256") != PROFILE_SHA256:
        raise AuthorizationError("AUTHORIZATION_PROFILE", "authorization profile identity drift")
    quiescence_value = value.get("quiescence")
    windows = quiescence_value.get("windows") if isinstance(quiescence_value, Mapping) else None
    if not isinstance(quiescence_value, Mapping) or quiescence_value.get("window_count") != WINDOW_COUNT or \
            quiescence_value.get("window_seconds") != WINDOW_SECONDS or \
            quiescence_value.get("consecutive_passes") != WINDOW_COUNT or \
            not isinstance(windows, list) or len(windows) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "authorization quiescence contract drift")
    for window in windows:
        if not isinstance(window, Mapping) or window.get("status") != "PASS" or window.get("runner_start_allowed") is not True:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "authorization has a non-PASS window")
        window_path = Path(str(window.get("path", "")))
        if window_path.parent != receipt_path.parent:
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window path escapes authorization root")
        _pin(window_path, str(window.get("sha256")), "quiescence window")
    _verify_immutable_lineage(repo_root)
    return value


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    args = parser.parse_args(argv)
    try:
        result = authorize(args.authorization_root, args.attempt_root, repo_root=args.repo_root, proc_root=args.proc_root)
    except AuthorizationError as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": exc.kind, "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result["status"] == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
