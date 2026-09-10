#!/usr/bin/env python3
"""Additive v20 exact-root authorizer with strict window filename binding.

The v19 failed authorization and all v19 source/evidence are immutable
lineage.  v20 only corrects the authorization-window filename parser and
keeps the real v17 image/runtime contract.  This module is host-only: it
does not open the bag, start Docker/ROS, score, or save a map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402
import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v19_formal as previous  # noqa: E402
import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as adapter  # noqa: E402


AUTHORIZATION_CONTRACT = "m6a10-v20-formal-exact-root-authorization-v1"
QUIESCENCE_CONTRACT = "m6a10-quiescence-v1"
WINDOW_RE = re.compile(r"^quiescence_window_(\d{2})\.receipt\.json$")
AUTHORIZATION_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v20_formal_authorization_20260824T011000Z_agentv20"
)
AUTHORIZATION_RECEIPT_PATH = AUTHORIZATION_ROOT / "formal_authorization.receipt.json"
ATTEMPT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v20_formal_replay_20260824T011000Z_agentv20formal"
)
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v20_formal_candidate.yaml"
V20_AUTHORIZER_PATH = Path(__file__).resolve()
V20_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v20_formal.py"
V20_LAUNCHER_SHA256 = "2fe3ba3cb577ee82433290d3a2a9867af6a1fd186a274af02b629061bef7e351"
V19_FAILURE_AUTHORIZATION_PATH = previous.AUTHORIZATION_ROOT / previous.AUTHORIZATION_RECEIPT_PATH.name
V19_FAILURE_AUTHORIZATION_SHA256 = "ddd42d38f60710bd39d984292194f9869c45d81993fd98ae4962bc8d7d4cb747"
V19_FAILURE_AUTHORIZATION_SIDECAR_SHA256 = "3c6b1851455d6139b517aa00893d713b2ab0d86983fd600d7b440fd3abf6ff98"
V19_INDEX_BUG_SHA256 = "33aa3a0230d98d8d8da4c64f568fbe221ce8dfcccf261f3eb4c4d672f4e88f34"
V19_CORRECTED_AUTHORIZER_SHA256 = "0de44fa9a44423c59ab7587acc567a9a7a1dac3b03a64c0672d49e9f7dfb15c9"
V19_ADAPTER_SHA256 = "61b3396aa54079773aa7066e5652527e27ddaa46121650b711f61807d0c82846"
V19_LAUNCHER_SHA256 = "854c138fe0673fd82766e0c26a262616c3c883577d1ab781ebabc3822fb8528a"
V19_PROFILE_SHA256 = "4decdb6d5091a8d0ba04e116e61a238383a2283ea0ba206f54a788108f266f6c"
V17_IMAGE_TAG = previous.V17_IMAGE_TAG
V17_IMAGE_ID = previous.V17_IMAGE_ID
INPUT_PATH = previous.INPUT_PATH
INPUT_BYTES = previous.INPUT_BYTES
INPUT_SHA256 = previous.INPUT_SHA256
EXPECTED_MESSAGES = previous.EXPECTED_MESSAGES
EXPECTED_COUNTS = previous.EXPECTED_COUNTS
REQUIRED_END_TIMESTAMP_SECONDS = previous.REQUIRED_END_TIMESTAMP_SECONDS
SENSOR_DURATION_SECONDS = previous.SENSOR_DURATION_SECONDS
WATCHDOG_SECONDS = previous.WATCHDOG_SECONDS
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
        raise AuthorizationError("NOT_REGULAR", "%s is not regular" % label)
    if immutable and (path.stat().st_mode & 0o777) != 0o444:
        raise AuthorizationError("IMMUTABILITY", "%s is not mode 0444" % label)


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise AuthorizationError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise AuthorizationError("OUTPUT_STAGING", "staging exists: %s" % part)
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


def _read_receipt(path: Path, expected_sha: str, expected_sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label, immutable=True)
    if sha256_file(path) != expected_sha:
        raise AuthorizationError("RECEIPT_SHA", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label, immutable=True)
    if sha256_file(sidecar) != expected_sidecar_sha or \
            sidecar.read_bytes() != (expected_sha + "  " + path.name + "\n").encode("ascii"):
        raise AuthorizationError("RECEIPT_SIDECAR", "%s sidecar drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AuthorizationError("RECEIPT_JSON", "%s is invalid JSON" % label) from exc
    if not isinstance(value, dict):
        raise AuthorizationError("RECEIPT_JSON", "%s is not an object" % label)
    return value


def _verify_profile(repo_root: Path) -> Dict[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    observed = sha256_file(path)
    import yaml
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AuthorizationError("V20_PROFILE_INVALID", "v20 profile YAML invalid") from exc
    candidate = document.get("candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    if document.get("schema_version") != 1 or document.get("status") != "V20_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or not isinstance(candidate, Mapping) or \
            candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("v19_failed_authorization", {}).get("sha256") != V19_FAILURE_AUTHORIZATION_SHA256 or \
            candidate.get("v19_failed_authorization", {}).get("failure_kind") != "AUTHORIZATION_FAIL_CLOSED" or \
            candidate.get("v19_corrected_authorizer_sha256") != V19_CORRECTED_AUTHORIZER_SHA256 or \
            candidate.get("v19_index_bug_sha256") != V19_INDEX_BUG_SHA256 or \
            candidate.get("v19_adapter_sha256") != V19_ADAPTER_SHA256 or \
            candidate.get("v19_launcher_sha256") != V19_LAUNCHER_SHA256 or \
            candidate.get("v19_profile_sha256") != V19_PROFILE_SHA256 or \
            candidate.get("v20_launcher_sha256") != V20_LAUNCHER_SHA256 or \
            candidate.get("v20_authorizer_sha256") != sha256_file(V20_AUTHORIZER_PATH) or \
            not isinstance(image, Mapping) or image.get("id") != V17_IMAGE_ID or image.get("tag") != V17_IMAGE_TAG:
        raise AuthorizationError("V20_PROFILE_INVALID", "v20 authority/lineage drift")
    return {"path": str(path), "sha256": observed, "status": document.get("status"),
            "candidate": dict(candidate), "image": dict(image)}


def _run_window(path: Path, *, proc_root: Path = Path("/proc"), now: Optional[str] = None) -> Mapping[str, Any]:
    match = WINDOW_RE.fullmatch(path.name)
    if match is None:
        raise AuthorizationError("WINDOW_FILENAME", "window filename does not match exact contract")
    index = int(match.group(1))
    if index not in range(1, WINDOW_COUNT + 1):
        raise AuthorizationError("WINDOW_FILENAME", "window index is outside 01..03")
    excluded = quiescence.ancestor_pids(proc_root, pid=os.getpid())
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded,
    )
    value = quiescence.build_receipt(observation, now=now)
    value.update({"authorization_window": True, "window_index": index,
                  "launcher_pid": os.getpid(), "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, value)
    return {"path": str(path), "sha256": digest, "status": value.get("status"),
            "runner_start_allowed": value.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", []), "window_index": index}


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
        raise AuthorizationError("AUTHORIZATION_ROOT", "v20 exact root drift")
    if os.path.lexists(authorization_root) or os.path.lexists(attempt_root):
        raise AuthorizationError("ROOT_NOT_FRESH", "v20 roots must be absent")
    authorization_root.mkdir(parents=True)
    try:
        profile = _verify_profile(repo_root)
        lineage = previous._verify_v17_lineage(repo_root)
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
            "lineage": {"v19": {"failed_authorization_path": str(V19_FAILURE_AUTHORIZATION_PATH),
                                  "failed_authorization_sha256": V19_FAILURE_AUTHORIZATION_SHA256,
                                  "failed_authorization_sidecar_sha256": V19_FAILURE_AUTHORIZATION_SIDECAR_SHA256,
                                  "failure_kind": "AUTHORIZATION_FAIL_CLOSED",
                                  "index_bug_sha256": V19_INDEX_BUG_SHA256,
                                  "corrected_authorizer_sha256": V19_CORRECTED_AUTHORIZER_SHA256,
                                  "adapter_sha256": V19_ADAPTER_SHA256,
                                  "launcher_sha256": V19_LAUNCHER_SHA256,
                                  "profile_sha256": V19_PROFILE_SHA256},
                        "v20": {"authorizer_path": str(V20_AUTHORIZER_PATH),
                                "authorizer_sha256": sha256_file(V20_AUTHORIZER_PATH),
                                "launcher_path": str(V20_LAUNCHER_PATH),
                                "launcher_sha256": V20_LAUNCHER_SHA256,
                                "profile_path": str(PROFILE_PATH),
                                "profile_sha256": profile["sha256"]},
                        "v17": lineage["v17"], "v18b": lineage["v18b"],
                        "v12_adapter": lineage["v12_adapter"],
                        "feeder_sha256": lineage["feeder_sha256"],
                        "wrapper_sha256": lineage["wrapper_sha256"],
                        "patch_sha256": lineage["patch_sha256"]},
            "profile": profile, "windows": windows,
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
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "exactly three windows required")
    paths = set(); hashes = set()
    for expected_index, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != expected_index or \
                item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or \
                item.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window order/status drift")
        path = Path(str(item.get("path", "")))
        match = WINDOW_RE.fullmatch(path.name)
        if match is None or int(match.group(1)) != expected_index or \
                path.parent.resolve() != receipt_path.parent.resolve():
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window filename/path drift")
        _regular(path, "window %02d" % expected_index, immutable=True)
        observed = sha256_file(path)
        if observed != item.get("sha256"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window SHA drift")
        document = json.loads(path.read_text(encoding="utf-8"))
        observation = document.get("observation", {})
        if document.get("schema_version") != 1 or document.get("contract_version") != QUIESCENCE_CONTRACT or \
                document.get("status") != "PASS" or document.get("runner_start_allowed") is not True or \
                document.get("window_index") != expected_index or observation.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window receipt contract drift")
        paths.add(str(path)); hashes.add(observed)
    if len(paths) != WINDOW_COUNT or len(hashes) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "windows are not distinct")


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Dict[str, Any]:
    if path.resolve() != AUTHORIZATION_RECEIPT_PATH.resolve() or attempt_root.resolve() != ATTEMPT_ROOT.resolve():
        raise AuthorizationError("AUTHORIZATION_ROOT", "v20 exact authorization/root mismatch")
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "v20 sidecar", immutable=True)
    value = _read_receipt(path, expected_sha256, sha256_file(sidecar), "v20 authorization")
    if value.get("schema_version") != 1 or value.get("contract_version") != AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False or value.get("attempt_root") != str(attempt_root):
        raise AuthorizationError("AUTHORIZATION_STATUS", "v20 authorization is not executable")
    if value.get("attempt_count") != 1 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "one-start/retry drift")
    if value.get("image") != {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID}:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "image identity drift")
    if value.get("input", {}).get("path") != INPUT_PATH or value.get("input", {}).get("bytes") != INPUT_BYTES or \
            value.get("input", {}).get("sha256") != INPUT_SHA256 or \
            value.get("input", {}).get("expected_messages") != EXPECTED_MESSAGES or \
            value.get("input", {}).get("expected_topic_counts") != EXPECTED_COUNTS:
        raise AuthorizationError("AUTHORIZATION_INPUT", "input contract drift")
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "safety drift")
    _verify_profile(repo_root)
    lineage = value.get("lineage")
    if not isinstance(lineage, Mapping) or lineage.get("v19", {}).get("failed_authorization_sha256") != V19_FAILURE_AUTHORIZATION_SHA256 or \
            lineage.get("v19", {}).get("corrected_authorizer_sha256") != V19_CORRECTED_AUTHORIZER_SHA256 or \
            lineage.get("v19", {}).get("adapter_sha256") != V19_ADAPTER_SHA256 or \
            lineage.get("v19", {}).get("launcher_sha256") != V19_LAUNCHER_SHA256:
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v19 lineage drift")
    v20 = lineage.get("v20")
    if not isinstance(v20, Mapping) or v20.get("authorizer_path") != str(V20_AUTHORIZER_PATH) or \
            v20.get("authorizer_sha256") != sha256_file(V20_AUTHORIZER_PATH) or \
            v20.get("launcher_path") != str(V20_LAUNCHER_PATH) or \
            v20.get("launcher_sha256") != V20_LAUNCHER_SHA256 or \
            v20.get("profile_path") != str(PROFILE_PATH) or \
            v20.get("profile_sha256") != sha256_file(PROFILE_PATH):
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v20 source lineage drift")
    current = previous._verify_v17_lineage(repo_root)
    if lineage.get("v17") != current.get("v17") or lineage.get("v18b") != current.get("v18b") or \
            lineage.get("v12_adapter") != current.get("v12_adapter"):
        raise AuthorizationError("AUTHORIZATION_LINEAGE", "v17/v18b adapter lineage drift")
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
                raise AuthorizationError("AUTHORIZATION_SHA_REQUIRED", "authorization SHA required")
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
