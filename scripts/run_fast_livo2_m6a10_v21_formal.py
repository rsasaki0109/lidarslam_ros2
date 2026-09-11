#!/usr/bin/env python3
"""v21 additive formal candidate with an atomic attempt-root boundary.

The exact attempt root is reserved with one kernel ``mkdir`` before any
``out``, monitor, or closure path is created.  All later failures are sealed
inside that reserved root; a pre-existing, symlink, or invalid-parent root is
never touched.  The real v19/v17 lifecycle remains the only production
runtime path.  This candidate is unauthorized by default and this module is
never invoked with a bag during its unit tests.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v21_formal as authorizer  # noqa: E402
import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as adapter  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v19_formal as v19  # noqa: E402


CANDIDATE_VERSION = "v21-atomic-attempt-root-reservation"
CLOSURE_CONTRACT = "m6a10-v21-formal-closure-v1"
PROFILE_PATH = authorizer.PROFILE_PATH
PROFILE_SHA256 = "9f6e574ba0fe7c927eecdd612f948352dc2bf986b576c4e4c52708e0bfcd013a"
ATTEMPT_ROOT = authorizer.ATTEMPT_ROOT
AUTHORIZATION_PATH = authorizer.AUTHORIZATION_RECEIPT_PATH
IMAGE_TAG = authorizer.V17_IMAGE_TAG
IMAGE_ID = authorizer.V17_IMAGE_ID
INPUT_PATH = authorizer.INPUT_PATH
V19_ADAPTER_PATH = ROOT / "scripts/fast_livo2_m6a10_v19_base_adapter.py"
V19_ADAPTER_SHA256 = "61b3396aa54079773aa7066e5652527e27ddaa46121650b711f61807d0c82846"
V19_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v19_formal.py"
V19_LAUNCHER_SHA256 = "49cfb1cb323e06b17514987aa17dc14d52a509b700e625a6d65beddb839f5771"
V21_LAUNCHER_PATH = Path(__file__).resolve()
RECEIPT_NAME = "closure_receipt.json"

CandidateError = adapter.BaseAdapterError
AuthorizationValidator = Callable[["v21_config"], Mapping[str, Any]]


def sha256_file(path: Path) -> str:
    return adapter.sha256_file(path)


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise CandidateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise CandidateError("OUTPUT_PARENT_INVALID", "closure parent is invalid")
    staging = path.with_name(path.name + ".part")
    if os.path.lexists(staging):
        raise CandidateError("OUTPUT_STAGING", "staging file exists: %s" % staging)
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


def _seal(surface: Optional[adapter.BaseV12Surface], root: Path,
          value: Mapping[str, Any]) -> Dict[str, Any]:
    enriched = dict(value)
    enriched.update({
        "candidate_version": CANDIDATE_VERSION,
        "v21_closure_contract": CLOSURE_CONTRACT,
        "attempt_root_reserved": True,
        "v21_launcher_sha256_observed": sha256_file(V21_LAUNCHER_PATH),
        "v19_adapter_sha256": V19_ADAPTER_SHA256,
    })
    path = root / RECEIPT_NAME
    payload = (json.dumps(enriched, indent=2, sort_keys=True) + "\n").encode("utf-8")
    digest = _atomic_bytes(path, payload)
    sidecar = path.with_name(path.name + ".sha256")
    side_digest = _atomic_bytes(sidecar, (digest + "  " + path.name + "\n").encode("ascii"))
    return dict(enriched, receipt_path=str(path), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


def _seal_reserved_failure(root: Path, *, kind: str, message: str,
                           profile: Optional[Mapping[str, Any]] = None,
                           authorization: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Seal a pre-Popen failure only after ``root`` was reserved by us."""
    if not os.path.lexists(root) or root.is_symlink() or not root.is_dir():
        raise CandidateError("ROOT_NOT_RESERVED", "cannot seal outside a reserved directory")
    return _seal(None, root, {
        "schema_version": 1, "contract_version": CLOSURE_CONTRACT,
        "status": "FAIL_CLOSED", "failure_kind": kind, "failure_message": message,
        "profile": dict(profile or {}), "authorization": dict(authorization or {}),
        "execution": {"formal_replay_started": False, "candidate_process_started": False,
                       "one_start": False, "popen_count": 0, "retry": False,
                       "manual_stop": False, "returncode": None, "shell": False,
                       "root_reserved_before_output": True},
        "artifacts": {"output_created": (root / "out").exists(),
                      "monitor_created": (root / "host_interference.summary.json").exists()},
        "safety": {"formal_replay_started": False, "input_opened": False,
                   "ground_truth_content_opened": False, "scorer_invoked": False,
                   "map_saved": False},
    })


def _reserve_exact_root(root: Path) -> Path:
    """Atomically reserve exactly one absent root; never create its parent."""
    if not root.is_absolute():
        raise CandidateError("ROOT_NOT_ABSOLUTE", "v21 attempt root must be absolute")
    if Path(os.path.abspath(root)) != Path(os.path.abspath(ATTEMPT_ROOT)):
        raise CandidateError("AUTHORIZATION_ROOT", "v21 exact attempt root mismatch")
    if os.path.lexists(root):
        raise CandidateError("ROOT_NOT_FRESH", "v21 attempt root already exists")
    parent = root.parent
    if not os.path.lexists(parent) or parent.is_symlink() or not parent.is_dir():
        raise CandidateError("ROOT_PARENT_INVALID", "v21 attempt root parent is invalid")
    absolute_parent = Path(os.path.abspath(parent))
    current = Path(absolute_parent.anchor)
    for component in absolute_parent.parts[1:]:
        current /= component
        if current.is_symlink():
            raise CandidateError("SYMLINK_REJECTED", "v21 attempt root parent contains a symlink")
    if not os.access(parent, os.W_OK | os.X_OK) or (parent.stat().st_mode & 0o222) == 0:
        raise CandidateError("ROOT_PARENT_NOT_WRITABLE", "v21 attempt root parent is not writable")
    try:
        os.mkdir(root, 0o755)
    except FileExistsError as exc:
        raise CandidateError("ROOT_NOT_FRESH", "v21 attempt root raced into existence") from exc
    if root.is_symlink() or not root.is_dir():
        raise CandidateError("ROOT_RESERVATION_INVALID", "v21 reservation is not a directory")
    return root


def verify_profile(repo_root: Path = ROOT) -> Mapping[str, Any]:
    if PROFILE_PATH.resolve() != (repo_root / PROFILE_PATH.relative_to(ROOT)).resolve():
        raise CandidateError("PROFILE_PATH", "v21 profile path drift")
    if sha256_file(PROFILE_PATH) != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v21 profile SHA is not bound")
    return authorizer._verify_profile(repo_root)


def _default_authorization(config: Any) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        raise CandidateError("AUTHORIZATION_REQUIRED", "v21 authorization is required")
    return authorizer.verify_authorization(config.authorization_path, config.root,
                                           config.authorization_sha256,
                                           repo_root=config.repo_root, allow_reserved=True)


def run_formal(config: Any = None, *, root: Optional[Path] = None,
               repo_root: Path = ROOT, bag_path: str = INPUT_PATH,
               authorization_path: Optional[Path] = None,
               authorization_sha256: Optional[str] = None,
               authorization_validator: Optional[AuthorizationValidator] = None,
               now: Optional[str] = None) -> Dict[str, Any]:
    """Reserve first, then verify and invoke the immutable v19 runtime once."""
    if config is None:
        if root is None:
            raise CandidateError("ROOT_REQUIRED", "v21 attempt root is required")
        config = v19.CandidateConfig(root=root, repo_root=repo_root, bag_path=bag_path,
                                     profile_path=PROFILE_PATH,
                                     authorization_path=authorization_path,
                                     authorization_sha256=authorization_sha256,
                                     container_name="m6a10-v21-formal-candidate")
    elif root is not None:
        raise CandidateError("CONFIG_ROOT_AMBIGUOUS", "provide config or root, not both")
    # This call is intentionally the first filesystem state transition.  In
    # particular, no output, monitor, closure, bag, or runtime path exists
    # before the exact root has been atomically created.
    try:
        _reserve_exact_root(config.root)
    except Exception:
        # Existing/invalid roots are not ours and must not receive a closure.
        raise
    surface: Optional[adapter.BaseV12Surface] = None
    profile: Optional[Mapping[str, Any]] = None
    authority: Optional[Mapping[str, Any]] = None
    original_contract = v19.CLOSURE_CONTRACT
    original_candidate_version = v19.CANDIDATE_VERSION
    try:
        if sha256_file(V19_ADAPTER_PATH) != V19_ADAPTER_SHA256:
            raise CandidateError("V19_ADAPTER_SHA256", "v19 adapter lineage drift")
        surface = adapter.load_base_surface()
        profile = verify_profile(config.repo_root)
        authority = (authorization_validator or _default_authorization)(config)
        if not isinstance(authority, Mapping) or authority.get("authorized") is not True or \
                authority.get("formal_execution") is not True:
            raise CandidateError("AUTHORIZATION_STATUS", "v21 authorization is not executable")
        if sha256_file(V19_LAUNCHER_PATH) != V19_LAUNCHER_SHA256:
            raise CandidateError("V19_LAUNCHER_SHA256", "v19 launcher lineage drift")
        # v19's helper is the actual production argv/Popen/monitor/capture /
        # compose path.  Only its closure contract is scoped and restored.
        v19.CLOSURE_CONTRACT = CLOSURE_CONTRACT
        v19.CANDIDATE_VERSION = CANDIDATE_VERSION
        result = v19._run_actual_v17(config, surface, profile, authority, now)
        return result
    except Exception as exc:
        return _seal_reserved_failure(config.root, kind=getattr(exc, "kind", "V21_FAIL_CLOSED"),
                                      message=str(exc), profile=profile, authorization=authority)
    finally:
        v19.CLOSURE_CONTRACT = original_contract
        v19.CANDIDATE_VERSION = original_candidate_version


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION_PATH)
    parser.add_argument("--authorization-sha256", required=True)
    parser.add_argument("--container-name", default="m6a10-v21-formal-candidate")
    args = parser.parse_args(argv)
    config = v19.CandidateConfig(root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                                 profile_path=PROFILE_PATH,
                                 authorization_path=args.authorization,
                                 authorization_sha256=args.authorization_sha256,
                                 container_name=args.container_name)
    try:
        result = run_formal(config, **dict(runtime or {}))
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V21_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
