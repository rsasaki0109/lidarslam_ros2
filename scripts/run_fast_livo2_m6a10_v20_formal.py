#!/usr/bin/env python3
"""v20 additive formal launcher using the real v19 adapter/v17 runtime.

The v19 candidate and its failed authorization are immutable.  v20 changes
only the authorization/profile boundary; production execution delegates to
the already-tested real v17 argv/Popen/monitor/capture/compose helper through
a scoped closure-contract override.  No shell or retry path exists.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v20_formal as authorizer  # noqa: E402
import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as adapter  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v19_formal as v19  # noqa: E402


CANDIDATE_VERSION = "v20-authorization-window-correction"
CLOSURE_CONTRACT = "m6a10-v20-formal-closure-v1"
PROFILE_PATH = authorizer.PROFILE_PATH
AUTHORIZATION_PATH = authorizer.AUTHORIZATION_RECEIPT_PATH
ATTEMPT_ROOT = authorizer.ATTEMPT_ROOT
IMAGE_TAG = authorizer.V17_IMAGE_TAG
IMAGE_ID = authorizer.V17_IMAGE_ID
INPUT_PATH = authorizer.INPUT_PATH
WATCHDOG_SECONDS = authorizer.WATCHDOG_SECONDS
V19_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v19_formal.py"
V19_LAUNCHER_SHA256 = authorizer.V19_LAUNCHER_SHA256
V20_LAUNCHER_PATH = Path(__file__).resolve()


class CandidateError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise CandidateError("SOURCE_NOT_REGULAR", "not a regular file: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise CandidateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
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


def _seal_preflight_failure(root: Path, *, kind: str, message: str,
                            authorization: Optional[Mapping[str, Any]] = None,
                            profile: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    if os.path.lexists(root):
        raise CandidateError("ROOT_NOT_FRESH", "v20 formal root already exists")
    root.parent.mkdir(parents=True, exist_ok=True)
    root.mkdir()
    value = {
        "schema_version": 1, "contract_version": CLOSURE_CONTRACT,
        "candidate_version": CANDIDATE_VERSION, "status": "FAIL_CLOSED",
        "failure_kind": kind, "failure_message": message,
        "profile": dict(profile or {}), "authorization": dict(authorization or {}),
        "execution": {"formal_replay_started": False, "candidate_process_started": False,
                       "one_start": False, "popen_count": 0, "retry": False,
                       "manual_stop": False, "returncode": None, "shell": False},
        "safety": {"formal_replay_started": False, "input_opened": False,
                   "ground_truth_content_opened": False, "scorer_invoked": False,
                   "map_saved": False},
        "lineage": {"v19_launcher_sha256": V19_LAUNCHER_SHA256,
                     "v20_launcher_sha256_observed": sha256_file(V20_LAUNCHER_PATH)},
    }
    receipt = root / "closure_receipt.json"
    digest = _atomic_bytes(receipt, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    sidecar = receipt.with_name(receipt.name + ".sha256")
    side_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(value, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


def _verify_profile(repo_root: Path = ROOT) -> Mapping[str, Any]:
    return authorizer._verify_profile(repo_root)


def verify_authorization(path: Path, attempt_root: Path, expected_sha256: str,
                         *, repo_root: Path = ROOT) -> Mapping[str, Any]:
    value = authorizer.verify_authorization(path, attempt_root, expected_sha256, repo_root=repo_root)
    if value.get("authorized") is not True or value.get("formal_execution") is not True:
        raise CandidateError("AUTHORIZATION_STATUS", "v20 authorization is not executable")
    return value


def run_formal(*, root: Path, repo_root: Path = ROOT, bag_path: str = INPUT_PATH,
               authorization_path: Optional[Path] = None,
               authorization_sha256: Optional[str] = None) -> Dict[str, Any]:
    """Verify v20 authority then invoke the real v17 lifecycle exactly once."""
    surface: Optional[adapter.BaseV12Surface] = None
    profile: Optional[Mapping[str, Any]] = None
    authority: Optional[Mapping[str, Any]] = None
    try:
        if root.resolve() != ATTEMPT_ROOT.resolve():
            raise CandidateError("AUTHORIZATION_ROOT", "v20 exact attempt root mismatch")
        if authorization_path is None or not authorization_sha256:
            raise CandidateError("AUTHORIZATION_REQUIRED", "v20 authorization is required")
        surface = adapter.load_base_surface()
        profile = _verify_profile(repo_root)
        authority = verify_authorization(authorization_path, root, authorization_sha256, repo_root=repo_root)
        if sha256_file(V19_LAUNCHER_PATH) != V19_LAUNCHER_SHA256:
            raise CandidateError("V19_LAUNCHER_SHA256", "v19 launcher lineage drift")
        if os.path.lexists(root):
            raise CandidateError("ROOT_NOT_FRESH", "v20 attempt root already exists")
    except Exception as exc:
        if not os.path.lexists(root):
            return _seal_preflight_failure(root, kind=getattr(exc, "kind", "V20_PREFLIGHT_FAIL_CLOSED"),
                                          message=str(exc), authorization=authority, profile=profile)
        raise
    config = v19.CandidateConfig(root=root, repo_root=repo_root, bag_path=bag_path,
                                 profile_path=PROFILE_PATH, authorization_path=authorization_path,
                                 authorization_sha256=authorization_sha256,
                                 container_name="m6a10-v20-formal-candidate")
    original_contract = v19.CLOSURE_CONTRACT
    try:
        # Scoped only: v19 source/behavior remains immutable and is restored
        # even when the real runtime fails before Popen.
        v19.CLOSURE_CONTRACT = CLOSURE_CONTRACT
        result = v19._run_actual_v17(config, surface, profile or {}, authority or {}, None)
        result["candidate_version"] = CANDIDATE_VERSION
        result["v20_launcher_sha256_observed"] = sha256_file(V20_LAUNCHER_PATH)
        return result
    finally:
        v19.CLOSURE_CONTRACT = original_contract


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION_PATH)
    parser.add_argument("--authorization-sha256", required=True)
    args = parser.parse_args(argv)
    try:
        result = run_formal(root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                            authorization_path=args.authorization,
                            authorization_sha256=args.authorization_sha256)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V20_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")},
                     sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
