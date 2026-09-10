#!/usr/bin/env python3
"""Additive v21 exact-root authorization boundary.

v21 is a host-only candidate.  It binds the immutable v20 failure lineage,
requires both authorization and formal roots to be genuinely absent, and
checks that their existing parents are non-symlink directories writable by
the current user.  It never creates or opens the formal attempt root, starts
Docker/ROS, opens the bag, invokes a scorer, or writes a map.
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

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v20_formal as previous  # noqa: E402
import lidarslam_benchmark_tools.check_m6a10_quiescence as quiescence  # noqa: E402


AUTHORIZATION_CONTRACT = "m6a10-v21-formal-exact-root-authorization-v1"
QUIESCENCE_CONTRACT = previous.QUIESCENCE_CONTRACT
WINDOW_RE = re.compile(r"^quiescence_window_(\d{2})\.receipt\.json$")
WINDOW_COUNT = 3
WINDOW_SECONDS = previous.WINDOW_SECONDS
MAX_BUSY_PERCENT = previous.MAX_BUSY_PERCENT
MAX_LOAD_PER_CPU = previous.MAX_LOAD_PER_CPU
AUTHORIZATION_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v21_formal_authorization_20260824T012000Z_agentv21"
)
AUTHORIZATION_RECEIPT_PATH = AUTHORIZATION_ROOT / "formal_authorization.receipt.json"
ATTEMPT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v21_formal_replay_20260824T012000Z_agentv21formal"
)
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v21_formal_candidate.yaml"
V21_AUTHORIZER_PATH = Path(__file__).resolve()
V21_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v21_formal.py"

V20_AUTHORIZATION_PATH = previous.AUTHORIZATION_RECEIPT_PATH
V20_AUTHORIZATION_SHA256 = "10e38c08baf04899fb478b35f9e9342d6478a218d5d50e84a5502c6ac8bd16ad"
V20_AUTHORIZATION_SIDECAR_SHA256 = "2c4210c1f8c05335cb45a663df72c4d62add421fef5bf79b6f36234dd0140e46"
V20_ATTEMPT_ROOT = previous.ATTEMPT_ROOT
V20_CLOSURE_PATH = V20_ATTEMPT_ROOT / "closure_receipt.json"
V20_CLOSURE_SHA256 = "ff786cc763e9f73bbeef7d94b2df5a12e7902f6d2d3c1446811fc5ade3f9f1c8"
V20_CLOSURE_SIDECAR_SHA256 = "1cd28060d6db840e731ba10b417ea9acaaae41d7194e48525e71b3305b4ff313"
V20_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v20_formal.py"
V20_AUTHORIZER_SHA256 = "55cb9fcf9771245f6b35c835a6c9ecfdf101b1a969ab7f65fcaeee32c1f4f982"
V20_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v20_formal.py"
V20_LAUNCHER_SHA256 = "5d746af5aa82572666788a444b527d12f40520062fbab9c9bf96f778696f7003"
V20_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v20_formal_candidate.yaml"
V20_PROFILE_SHA256 = "49ebb47a33f69a41545be9b2333d01d07ddd6f10cd23f7fb10cec0aa0cf91392"
V20_FAILURE_KIND = "OUTPUT_ROOT_NOT_RESERVED"
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


class AuthorizationError(ValueError):
    """A fail-closed authorization or freshness error."""

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
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if not os.path.lexists(path) or path.is_symlink() or not path.is_file():
        raise AuthorizationError("NOT_REGULAR", "%s is not regular" % label)
    if immutable and (path.stat().st_mode & 0o777) != 0o444:
        raise AuthorizationError("IMMUTABILITY", "%s is not mode 0444" % label)


def _parent_ready(root: Path, label: str) -> None:
    """Check a fresh root without creating any component of it."""
    if not root.is_absolute():
        raise AuthorizationError("ROOT_NOT_ABSOLUTE", "%s must be absolute" % label)
    if os.path.lexists(root):
        raise AuthorizationError("ROOT_NOT_FRESH", "%s already exists" % label)
    parent = root.parent
    if parent == root or not os.path.lexists(parent) or parent.is_symlink() or not parent.is_dir():
        raise AuthorizationError("ROOT_PARENT_INVALID", "%s parent is not a regular directory" % label)
    absolute = Path(os.path.abspath(parent))
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise AuthorizationError("SYMLINK_REJECTED", "%s parent contains a symlink" % label)
    if not os.access(parent, os.W_OK | os.X_OK) or (parent.stat().st_mode & 0o222) == 0:
        raise AuthorizationError("ROOT_PARENT_NOT_WRITABLE", "%s parent is not writable" % label)


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise AuthorizationError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    if not path.parent.exists() or path.parent.is_symlink():
        raise AuthorizationError("OUTPUT_PARENT_INVALID", "output parent is invalid")
    staging = path.with_name(path.name + ".part")
    if os.path.lexists(staging):
        raise AuthorizationError("OUTPUT_STAGING", "staging exists: %s" % staging)
    fd = os.open(staging, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                 getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(staging, path, follow_symlinks=False)
    finally:
        try:
            staging.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)
    return hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> str:
    return _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _read_immutable(path: Path, expected_sha: str, expected_sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label, immutable=True)
    if sha256_file(path) != expected_sha:
        raise AuthorizationError("RECEIPT_SHA", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, label + " sidecar", immutable=True)
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


def _verify_v20_lineage(repo_root: Path = ROOT) -> Mapping[str, Any]:
    if sha256_file(V20_AUTHORIZER_PATH) != V20_AUTHORIZER_SHA256 or \
            sha256_file(V20_LAUNCHER_PATH) != V20_LAUNCHER_SHA256 or \
            sha256_file(V20_PROFILE_PATH) != V20_PROFILE_SHA256:
        raise AuthorizationError("V20_SOURCE_DRIFT", "v20 source lineage drift")
    previous._verify_profile(repo_root)
    authority = previous.verify_authorization(V20_AUTHORIZATION_PATH, V20_ATTEMPT_ROOT,
                                               V20_AUTHORIZATION_SHA256, repo_root=repo_root)
    closure = _read_immutable(V20_CLOSURE_PATH, V20_CLOSURE_SHA256,
                              V20_CLOSURE_SIDECAR_SHA256, "v20 closure")
    if closure.get("status") != "FAIL_CLOSED" or closure.get("failure_kind") != V20_FAILURE_KIND or \
            closure.get("execution", {}).get("formal_replay_started") is not False or \
            closure.get("execution", {}).get("popen_count") != 0 or \
            closure.get("safety", {}).get("input_opened") is not False:
        raise AuthorizationError("V20_CLOSURE_INVALID", "v20 pre-Popen closure is not immutable failure lineage")
    return {"authorization": authority, "closure": {"path": str(V20_CLOSURE_PATH),
            "sha256": V20_CLOSURE_SHA256, "sidecar_sha256": V20_CLOSURE_SIDECAR_SHA256,
            "failure_kind": V20_FAILURE_KIND}}


def _verify_profile(repo_root: Path = ROOT) -> Dict[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    _regular(path, "v21 profile")
    observed = sha256_file(path)
    import yaml
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AuthorizationError("V21_PROFILE_INVALID", "v21 profile YAML invalid") from exc
    candidate = document.get("candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    lineage = document.get("lineage") if isinstance(document, Mapping) else None
    if document.get("schema_version") != 1 or document.get("status") != "V21_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or document.get("formal_replay_authorized") is not False or \
            document.get("formal_replay_started") is not False or document.get("replay_count") != 0 or \
            not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("v20_failure_closure_sha256") != V20_CLOSURE_SHA256 or \
            candidate.get("v20_failure_kind") != V20_FAILURE_KIND or \
            candidate.get("v21_authorizer_sha256") != sha256_file(V21_AUTHORIZER_PATH) or \
            not isinstance(image, Mapping) or image.get("id") != V17_IMAGE_ID or image.get("tag") != V17_IMAGE_TAG or \
            not isinstance(lineage, Mapping) or lineage.get("v20_authorization_sha256") != V20_AUTHORIZATION_SHA256 or \
            lineage.get("v20_closure_sha256") != V20_CLOSURE_SHA256:
        raise AuthorizationError("V21_PROFILE_INVALID", "v21 authority/lineage drift")
    safety = document.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("V21_PROFILE_SAFETY", "v21 profile safety drift")
    return {"path": str(path), "sha256": observed, "status": document.get("status"),
            "candidate": dict(candidate), "image": dict(image), "lineage": dict(lineage)}


def _run_window(path: Path, *, proc_root: Path = Path("/proc"), now: Optional[str] = None) -> Mapping[str, Any]:
    match = WINDOW_RE.fullmatch(path.name)
    if match is None or int(match.group(1)) not in range(1, WINDOW_COUNT + 1):
        raise AuthorizationError("WINDOW_FILENAME", "window filename must be quiescence_window_01..03")
    excluded = quiescence.ancestor_pids(proc_root, pid=os.getpid())
    observation = quiescence.collect_observation(
        proc_root=proc_root, sample_seconds=WINDOW_SECONDS,
        max_busy_percent=MAX_BUSY_PERCENT, max_load_per_cpu=MAX_LOAD_PER_CPU,
        excluded_pids=excluded)
    value = quiescence.build_receipt(observation, now=now)
    value.update({"authorization_window": True, "window_index": int(match.group(1)),
                  "launcher_pid": os.getpid(), "excluded_ancestor_pids": sorted(excluded)})
    digest = _write_json(path, value)
    return {"path": str(path), "sha256": digest, "status": value.get("status"),
            "runner_start_allowed": value.get("runner_start_allowed"),
            "forbidden_processes": observation.get("forbidden_processes", []),
            "window_index": int(match.group(1))}


def _verify_windows(value: Mapping[str, Any], receipt_path: Path) -> None:
    windows = value.get("windows")
    if not isinstance(windows, list) or len(windows) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "exactly three windows required")
    paths = set(); hashes = set()
    for expected, item in enumerate(windows, 1):
        if not isinstance(item, Mapping) or item.get("window_index") != expected or \
                item.get("status") != "PASS" or item.get("runner_start_allowed") is not True or \
                item.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window status/order drift")
        path = Path(str(item.get("path", "")))
        match = WINDOW_RE.fullmatch(path.name)
        if match is None or int(match.group(1)) != expected or \
                path.parent.absolute() != receipt_path.parent.absolute():
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window path drift")
        _regular(path, "window %02d" % expected, immutable=True)
        observed = sha256_file(path)
        if observed != item.get("sha256"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window SHA drift")
        document = json.loads(path.read_text(encoding="utf-8"))
        observation = document.get("observation", {})
        if document.get("schema_version") != 1 or document.get("contract_version") != QUIESCENCE_CONTRACT or \
                document.get("status") != "PASS" or document.get("runner_start_allowed") is not True or \
                document.get("window_index") != expected or observation.get("forbidden_processes"):
            raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "window receipt contract drift")
        paths.add(str(path)); hashes.add(observed)
    if len(paths) != WINDOW_COUNT or len(hashes) != WINDOW_COUNT:
        raise AuthorizationError("AUTHORIZATION_QUIESCENCE", "windows must be distinct")


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / AUTHORIZATION_RECEIPT_PATH.name
    digest = _write_json(receipt, value)
    sidecar = receipt.with_name(receipt.name + ".sha256")
    side_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


WindowRunner = Callable[..., Mapping[str, Any]]


def authorize(*, authorization_root: Path = AUTHORIZATION_ROOT, attempt_root: Path = ATTEMPT_ROOT,
              repo_root: Path = ROOT, proc_root: Path = Path("/proc"), now: Optional[str] = None,
              window_runner: Optional[WindowRunner] = None) -> Dict[str, Any]:
    if Path(os.path.abspath(authorization_root)) != Path(os.path.abspath(AUTHORIZATION_ROOT)) or \
            Path(os.path.abspath(attempt_root)) != Path(os.path.abspath(ATTEMPT_ROOT)):
        raise AuthorizationError("AUTHORIZATION_ROOT", "v21 exact root drift")
    _parent_ready(authorization_root, "authorization root")
    _parent_ready(attempt_root, "attempt root")
    try:
        os.mkdir(authorization_root, 0o755)
    except FileExistsError as exc:
        raise AuthorizationError("ROOT_NOT_FRESH", "v21 authorization root raced into existence") from exc
    try:
        profile = _verify_profile(repo_root)
        lineage = _verify_v20_lineage(repo_root)
        runner = window_runner or _run_window
        windows: List[Dict[str, Any]] = []
        for index in range(1, WINDOW_COUNT + 1):
            path = authorization_root / ("quiescence_window_%02d.receipt.json" % index)
            result = dict(runner(path, proc_root=proc_root, now=now))
            result["window_index"] = index
            windows.append(result)
        passed = len(windows) == WINDOW_COUNT and all(
            item.get("window_index") == index and item.get("status") == "PASS" and
            item.get("runner_start_allowed") is True and not item.get("forbidden_processes")
            for index, item in enumerate(windows, 1))
        value: Dict[str, Any] = {
            "schema_version": 1, "contract_version": AUTHORIZATION_CONTRACT,
            "status": "AUTHORIZED" if passed else "FAIL_CLOSED", "authorized": passed,
            "formal_execution": passed, "formal_replay_forbidden": not passed,
            "formal_replay_started": False, "attempt_root": str(attempt_root),
            "authorization_root": str(authorization_root), "attempt_root_reserved": False,
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
            "monitor": {"continuous": True, "interval_seconds": WINDOW_SECONDS,
                        "natural_completion": True, "host_interference_stop": False},
            "lineage": {"v20": {"authorization_path": str(V20_AUTHORIZATION_PATH),
                                  "authorization_sha256": V20_AUTHORIZATION_SHA256,
                                  "authorization_sidecar_sha256": V20_AUTHORIZATION_SIDECAR_SHA256,
                                  "closure_path": str(V20_CLOSURE_PATH),
                                  "closure_sha256": V20_CLOSURE_SHA256,
                                  "closure_sidecar_sha256": V20_CLOSURE_SIDECAR_SHA256,
                                  "failure_kind": V20_FAILURE_KIND,
                                  "authorizer_sha256": V20_AUTHORIZER_SHA256,
                                  "launcher_sha256": V20_LAUNCHER_SHA256,
                                  "profile_sha256": V20_PROFILE_SHA256}},
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
            "attempt_root": str(attempt_root), "authorization_root": str(authorization_root),
            "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(exc),
            "safety": {"input_opened": False, "ground_truth_content_opened": False,
                        "scorer_invoked": False, "map_saved": False},
        })


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT, allow_reserved: bool = False) -> Dict[str, Any]:
    if path.absolute() != AUTHORIZATION_RECEIPT_PATH.absolute() or \
            attempt_root.absolute() != ATTEMPT_ROOT.absolute():
        raise AuthorizationError("AUTHORIZATION_ROOT", "v21 exact authorization/root mismatch")
    if allow_reserved:
        if not os.path.lexists(attempt_root) or attempt_root.is_symlink() or not attempt_root.is_dir():
            raise AuthorizationError("ROOT_NOT_RESERVED", "reserved attempt root is not a directory")
    else:
        _parent_ready(attempt_root, "attempt root")
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "v21 sidecar", immutable=True)
    value = _read_immutable(path, expected_sha256, sha256_file(sidecar), "v21 authorization")
    if value.get("schema_version") != 1 or value.get("contract_version") != AUTHORIZATION_CONTRACT or \
            value.get("status") != "AUTHORIZED" or value.get("authorized") is not True or \
            value.get("formal_execution") is not True or value.get("formal_replay_forbidden") is not False or \
            value.get("formal_replay_started") is not False or value.get("attempt_root") != str(attempt_root) or \
            value.get("authorization_root") != str(path.parent):
        raise AuthorizationError("AUTHORIZATION_STATUS", "v21 authorization is not executable")
    if value.get("attempt_count") != 1 or value.get("one_start") is not True or \
            value.get("retry") is not False or value.get("manual_stop") is not False:
        raise AuthorizationError("AUTHORIZATION_REPLAY", "v21 one-start/retry drift")
    if value.get("image") != {"tag": V17_IMAGE_TAG, "id": V17_IMAGE_ID}:
        raise AuthorizationError("AUTHORIZATION_IMAGE", "v21 image identity drift")
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise AuthorizationError("AUTHORIZATION_SAFETY", "v21 safety drift")
    _verify_profile(repo_root)
    _verify_v20_lineage(repo_root)
    _verify_windows(value, path)
    return {"authorized": True, "formal_execution": True, "status": "AUTHORIZED",
            "attempt_root": str(attempt_root), "image_id": V17_IMAGE_ID,
            "receipt_path": str(path), "receipt_sha256": expected_sha256,
            "windows": value.get("windows")}


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
        elif args.authorization_sha256:
            value = verify_authorization(args.authorization, args.attempt_root, args.authorization_sha256)
        else:
            raise AuthorizationError("AUTHORIZATION_SHA_REQUIRED", "authorization SHA required")
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: value.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if value.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
