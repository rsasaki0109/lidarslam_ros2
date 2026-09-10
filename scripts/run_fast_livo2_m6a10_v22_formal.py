#!/usr/bin/env python3
"""v22 additive formal launcher candidate.

The production entrypoint is deliberately unauthorized.  It verifies the
immutable image/source/evidence lineage and then fails before any bag, image,
Docker, or ROS probe unless a future explicit authorization is installed.
``run_formal(..., runtime={"injected": True, ...})`` is a test-only seam for
the lifecycle/closure contract; it uses a supplied fake Popen and never starts
an external process.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v22_formal as authorizer  # noqa: E402


CANDIDATE_VERSION = "v22-synthetic-evidence-persistence-candidate"
CLOSURE_CONTRACT = "m6a10-v22-formal-closure-v1"
PROFILE_PATH = authorizer.PROFILE_PATH
PROFILE_SHA256 = "0fdadfefb7c30b9eb910e68529e47eaf1cacdee559d64ee0f262483452aeda6d"
V22_LAUNCHER_PATH = Path(__file__).resolve()
V22_LAUNCHER_SHA256_OBSERVED = "__V22_LAUNCHER_SHA256_OBSERVED__"
V22_AUTHORIZER_PATH = Path(authorizer.__file__).resolve()
V22_WRAPPER_PATH = authorizer.V22_WRAPPER_PATH
V22_WRAPPER_SHA256 = authorizer.V22_WRAPPER_SHA256
IMAGE_TAG = authorizer.V17_IMAGE_TAG
IMAGE_ID = authorizer.V17_IMAGE_ID
INPUT_PATH = "/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
WATCHDOG_SECONDS = 1200


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
    if os.path.lexists(path) or os.path.lexists(path.with_name(path.name + ".part")):
        raise CandidateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise CandidateError("OUTPUT_PARENT_INVALID", "invalid output parent")
    part = path.with_name(path.name + ".part")
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                 getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
        os.chmod(path, mode, follow_symlinks=False)
        dfd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    return hashlib.sha256(payload).hexdigest()


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / "closure_receipt.json"
    enriched = dict(value)
    enriched.update({
        "schema_version": 1,
        "contract_version": CLOSURE_CONTRACT,
        "candidate_version": CANDIDATE_VERSION,
        "v22_launcher_sha256_observed": sha256_file(V22_LAUNCHER_PATH),
        "v22_wrapper_sha256": V22_WRAPPER_SHA256,
    })
    digest = _atomic_bytes(receipt, (json.dumps(enriched, sort_keys=True, indent=2) + "\n").encode())
    sidecar = receipt.with_name(receipt.name + ".sha256")
    side_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(enriched, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


def _load_profile(repo_root: Path = ROOT) -> Mapping[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    if path.is_symlink() or not path.is_file() or sha256_file(path) != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v22 profile path/bytes drift")
    try:
        import yaml
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise CandidateError("PROFILE_INVALID", "v22 profile is not valid YAML") from exc
    if not isinstance(document, Mapping) or document.get("schema_version") != 1 or \
            document.get("status") != "V22_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("formal_replay_started") is not False or \
            document.get("replay_count") != 0:
        raise CandidateError("PROFILE_CONTRACT", "v22 profile is not unauthorized candidate form")
    candidate = document.get("candidate")
    image = document.get("image")
    safety = document.get("safety")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("v22_wrapper_sha256") != V22_WRAPPER_SHA256 or \
            candidate.get("v22_authorizer_sha256") != sha256_file(V22_AUTHORIZER_PATH) or \
            candidate.get("v22_gate_sha256") != authorizer.V22_GATE_SHA256 or \
            candidate.get("v22_gate_test_sha256") != sha256_file(ROOT / "graph_based_slam/test/test_fast_livo2_m6a10_v22_synthetic_gate.py") or \
            candidate.get("v22_formal_test_sha256") != sha256_file(ROOT / "graph_based_slam/test/test_fast_livo2_m6a10_v22_formal.py") or \
            not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or \
            image.get("tag") != IMAGE_TAG or not isinstance(safety, Mapping) or \
            any(safety.get(key) is not False for key in (
                "formal_replay_started", "input_opened", "ground_truth_content_opened",
                "scorer_invoked", "map_saved")):
        raise CandidateError("PROFILE_CONTRACT", "v22 profile identity/safety drift")
    return {"path": str(path), "sha256": PROFILE_SHA256,
            "status": document.get("status"), "candidate": dict(candidate),
            "image": dict(image), "safety": dict(safety)}


def verify_profile(repo_root: Path = ROOT) -> Mapping[str, Any]:
    return _load_profile(repo_root)


def _validate_injected_argv(argv: Sequence[str]) -> None:
    if not isinstance(argv, (list, tuple)) or not argv or any(not isinstance(x, str) for x in argv):
        raise CandidateError("ARGV_INVALID", "injected argv must be a string sequence")
    rendered = " ".join(argv).lower()
    if "shell=true" in rendered or "rosbag play" in rendered or "ground_truth" in rendered or \
            "scorer" in rendered or "map_save" in rendered or "--rm" in argv:
        raise CandidateError("ARGV_FORBIDDEN", "injected argv contains forbidden surface")
    if IMAGE_ID not in argv or "--network" not in argv or \
            argv[argv.index("--network") + 1] != "none" or "--read-only" not in argv:
        raise CandidateError("ARGV_ISOLATION", "injected argv isolation drift")
    mounts = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]
    if sum(spec.endswith("dst=/out,readonly=false") for spec in mounts) != 1 or \
            any("dst=/input" in spec for spec in mounts):
        raise CandidateError("ARGV_MOUNTS", "injected argv requires one RW /out and no input")


def _run_injected(root: Path, runtime: Mapping[str, Any], profile: Mapping[str, Any],
                  lineage: Mapping[str, Any]) -> Dict[str, Any]:
    if root.is_symlink() or os.path.lexists(root):
        raise CandidateError("ROOT_NOT_FRESH", "injected root must be absent")
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise CandidateError("ROOT_PARENT_INVALID", "injected root parent invalid")
    root.mkdir()
    (root / "out").mkdir()
    try:
        argv = list(runtime.get("argv", ()))
        _validate_injected_argv(argv)
        popen = runtime.get("popen")
        if not callable(popen):
            raise CandidateError("INJECTED_POPEN_MISSING", "injected Popen hook is required")
        process = popen(argv, cwd=str(root), shell=False)
        returncode = int(process.wait())
        artifact_validator = runtime.get("artifact_validator")
        artifacts = artifact_validator(root) if callable(artifact_validator) else {
            "status": "PASS" if returncode == 0 else "FAIL_CLOSED"}
        if not isinstance(artifacts, Mapping):
            raise CandidateError("ARTIFACT_VALIDATOR", "injected artifact result is not a mapping")
        status = "PASS" if returncode == 0 and artifacts.get("status") == "PASS" else "FAIL_CLOSED"
        value: Dict[str, Any] = {
            "status": status,
            "failure_kind": None if status == "PASS" else "INJECTED_RUNTIME_FAILURE",
            "failure_message": None if status == "PASS" else str(artifacts.get("reason", "runtime failed")),
            "profile": dict(profile),
            "lineage_receipts_verified": True,
            "injected_runtime": True,
            "execution": {"formal_replay_started": False, "candidate_process_started": True,
                           "one_start": True, "popen_count": 1, "retry": False,
                           "manual_stop": False, "returncode": returncode, "shell": False,
                           "image_id": IMAGE_ID, "network": "none", "rootfs": "read_only",
                           "rw_mount_destination": "/out", "input_mount_count": 0},
            "artifacts": dict(artifacts),
            "safety": {"formal_replay_started": False, "input_opened": False,
                       "ground_truth_content_opened": False, "scorer_invoked": False,
                       "map_saved": False},
            "synthetic_gate_lineage": {
                "success_receipt_sha256": authorizer.V22_SUCCESS_RECEIPT_SHA256,
                "failure_receipt_sha256": authorizer.V22_FAILURE_RECEIPT_SHA256,
            },
        }
        return _seal(root, value)
    except Exception as exc:
        if root.exists() and not (root / "closure_receipt.json").exists():
            return _seal(root, {
                "status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V22_INJECTED_FAILURE"),
                "failure_message": str(exc), "profile": dict(profile),
                "lineage_receipts_verified": True, "injected_runtime": True,
                "execution": {"formal_replay_started": False, "candidate_process_started": False,
                               "one_start": False, "popen_count": 0, "retry": False,
                               "manual_stop": False, "returncode": None, "shell": False},
                "safety": {"formal_replay_started": False, "input_opened": False,
                           "ground_truth_content_opened": False, "scorer_invoked": False,
                           "map_saved": False},
            })
        raise


def run_formal(*, root: Path, repo_root: Path = ROOT, bag_path: str = INPUT_PATH,
               authorization_path: Optional[Path] = None,
               authorization_sha256: Optional[str] = None,
               runtime: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Verify lineage then reject unauthorized production execution.

    ``bag_path`` is intentionally unused before authorization.  The only
    executable path is the explicit injected test seam.
    """
    del bag_path
    profile = verify_profile(repo_root)
    lineage = authorizer.verify_lineage(repo_root=repo_root)
    if runtime and runtime.get("injected") is True:
        validator = runtime.get("authorization_validator")
        if not callable(validator) or validator() is not True:
            raise CandidateError("AUTHORIZATION_STATUS", "injected authorization must be explicit true")
        return _run_injected(root, runtime, profile, lineage)
    # This is deliberately before any bag/image/runtime probe or root create.
    authorizer.verify_authorization(authorization_path,
                                    expected_sha256=authorization_sha256,
                                    attempt_root=root)
    raise CandidateError("AUTHORIZATION_NOT_INSTALLED", "unreachable authorization path")


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    args = parser.parse_args(argv)
    try:
        result = run_formal(root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                            authorization_path=args.authorization,
                            authorization_sha256=args.authorization_sha256,
                            runtime=runtime)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V22_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
