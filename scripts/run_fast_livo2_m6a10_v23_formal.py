#!/usr/bin/env python3
"""v23 additive formal launcher with the shared v19 authorization boundary.

The v22 files remain immutable.  This candidate imports the actual v19
adapter/verifier and the actual v22 modules, validates the sealed v22 receipt
through :mod:`fast_livo2_m6a10_v23_shared_auth`, and exposes only an injected
fake-Popen lifecycle for unit testing.  The default entrypoint stays
execution-inert until a future production runtime is explicitly installed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml

import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as shared_adapter
import lidarslam_benchmark_tools.fast_livo2_m6a10_v23_shared_auth as shared_auth


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v23_formal_candidate.yaml"
PROFILE_SHA256 = "ad10d579c18e697bd3016aa352b717eb798a5d09e997ba7330172edf0e14e908"
SHARED_AUTH_PATH = ROOT / "scripts/fast_livo2_m6a10_v23_shared_auth.py"
SHARED_AUTH_SHA256 = "db13718b9edbd3bb7cb97322190a86ae4783cd4d67cd2110c99b52b649683e32"
V23_LAUNCHER_PATH = Path(__file__).resolve()
CANDIDATE_VERSION = "v23-shared-auth-integration-candidate"
CLOSURE_CONTRACT = "m6a10-v23-formal-closure-v1"
RECEIPT_NAME = "closure_receipt.json"


class CandidateError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, immutable: bool = False) -> None:
    try:
        shared_adapter._regular(path, label, immutable=immutable)
    except Exception as exc:
        raise CandidateError(getattr(exc, "kind", "NOT_REGULAR"), str(exc)) from exc


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
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    return hashlib.sha256(payload).hexdigest()


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    receipt = root / RECEIPT_NAME
    enriched = dict(value)
    enriched.update({
        "schema_version": 1,
        "contract_version": CLOSURE_CONTRACT,
        "candidate_version": CANDIDATE_VERSION,
        "v23_launcher_sha256_observed": sha256_file(V23_LAUNCHER_PATH),
        "v23_shared_auth_sha256": SHARED_AUTH_SHA256,
    })
    digest = _atomic_bytes(receipt, (json.dumps(enriched, sort_keys=True, indent=2) + "\n").encode())
    sidecar = receipt.with_name(receipt.name + ".sha256")
    side_digest = _atomic_bytes(sidecar, (digest + "  " + receipt.name + "\n").encode("ascii"))
    return dict(enriched, receipt_path=str(receipt), receipt_sha256=digest,
                sidecar_path=str(sidecar), sidecar_sha256=side_digest)


def _validate_profile_document(document: Mapping[str, Any], *, repo_root: Path = ROOT) -> None:
    if document.get("schema_version") != 1 or document.get("status") != "V23_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("formal_replay_started") is not False or document.get("replay_count") != 0:
        raise CandidateError("PROFILE_AUTHORITY", "v23 profile is not unauthorized")
    candidate = document.get("candidate")
    safety = document.get("safety")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("shared_auth_path") != str(SHARED_AUTH_PATH.relative_to(repo_root)) or \
            candidate.get("shared_auth_sha256") != SHARED_AUTH_SHA256 or \
            candidate.get("synthetic_authorizer_path") != str(shared_auth.V23_SYNTHETIC_AUTHORIZER_PATH.relative_to(repo_root)) or \
            candidate.get("synthetic_authorizer_sha256") != shared_auth.V23_SYNTHETIC_AUTHORIZER_SHA256 or \
            candidate.get("synthetic_auth_receipt_path") != str(shared_auth.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH) or \
            candidate.get("synthetic_auth_receipt_sha256") != shared_auth.V23_SYNTHETIC_AUTHORIZATION_SHA256 or \
            candidate.get("synthetic_auth_sidecar_sha256") != shared_auth.V23_SYNTHETIC_AUTHORIZATION_SIDECAR_SHA256 or \
            candidate.get("synthetic_runtime_root") != shared_auth.V23_SYNTHETIC_RUNTIME_ROOT or \
            candidate.get("v22_authorized_receipt_path") != str(shared_auth.V22_AUTHORIZATION_RECEIPT_PATH) or \
            candidate.get("v22_authorized_receipt_sha256") != shared_auth.V22_AUTHORIZATION_SHA256 or \
            candidate.get("v22_authorized_sidecar_sha256") != shared_auth.V22_AUTHORIZATION_SIDECAR_SHA256 or \
            candidate.get("v22_authorized_attempt_root") != shared_auth.V22_ATTEMPT_ROOT or \
            candidate.get("v22_pre_popen_failure_path") != str(shared_auth.V22_PRIOR_AUTHORIZATION_RECEIPT_PATH) or \
            candidate.get("v22_pre_popen_failure_sha256") != shared_auth.V22_PRIOR_AUTHORIZATION_SHA256 or \
            candidate.get("v22_pre_popen_failure_kind") != "HOST_QUIESCENCE_CPU_LIMIT" or \
            candidate.get("v19_adapter_sha256") != shared_auth.V19_ADAPTER_SHA256 or \
            candidate.get("v19_authorizer_sha256") != shared_auth.V19_AUTHORIZER_SHA256 or \
            candidate.get("v19_launcher_sha256") != shared_auth.V19_LAUNCHER_SHA256 or \
            candidate.get("v22_launcher_sha256") != shared_auth.V22_LAUNCHER_SHA256 or \
            candidate.get("v22_profile_sha256") != shared_auth.V22_PROFILE_SHA256 or \
            candidate.get("v22_phase_contract") != shared_auth.PHASE_CONTRACT or \
            candidate.get("v22_transport_contract") != shared_auth.TRANSPORT_CONTRACT or \
            candidate.get("image_tag") != shared_auth.V17_IMAGE_TAG or \
            candidate.get("image_id") != shared_auth.V17_IMAGE_ID or \
            not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
                "formal_replay_started", "input_opened", "ground_truth_content_opened",
                "scorer_invoked", "map_saved")):
        raise CandidateError("PROFILE_CONTRACT", "v23 profile identity/safety drift")


def verify_profile(repo_root: Path = ROOT) -> Mapping[str, Any]:
    path = repo_root / PROFILE_PATH.relative_to(ROOT)
    if path.resolve() != PROFILE_PATH.resolve() or path.is_symlink() or not path.is_file() or \
            PROFILE_SHA256.startswith("__") or sha256_file(path) != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v23 profile path/bytes drift")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise CandidateError("PROFILE_INVALID", "v23 profile is not valid YAML") from exc
    if not isinstance(document, Mapping):
        raise CandidateError("PROFILE_INVALID", "v23 profile must be a mapping")
    _validate_profile_document(document, repo_root=repo_root)
    return {"path": str(path), "sha256": PROFILE_SHA256, "status": document["status"],
            "candidate": dict(document["candidate"]), "safety": dict(document["safety"])}


def _reserve_root(root: Path) -> None:
    if os.path.lexists(root) or root.is_symlink():
        raise CandidateError("ROOT_NOT_FRESH", "v23 root already exists")
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise CandidateError("ROOT_PARENT_INVALID", "v23 root parent is invalid")
    root.mkdir()


def _run_injected(root: Path, profile: Mapping[str, Any], identity: Mapping[str, Any],
                  authorization: Mapping[str, Any], runtime: Mapping[str, Any]) -> Dict[str, Any]:
    _reserve_root(root)
    (root / "out").mkdir()
    try:
        argv = runtime.get("argv")
        if not isinstance(argv, (list, tuple)):
            raise CandidateError("ARGV_INVALID", "injected argv is required")
        shared_auth.validate_runtime_argv(argv)
        process_factory = runtime.get("popen")
        if not callable(process_factory):
            raise CandidateError("INJECTED_POPEN_MISSING", "injected Popen hook is required")
        artifact_validator = runtime.get("artifact_validator")
        if not callable(artifact_validator):
            raise CandidateError("ARTIFACT_VALIDATOR_MISSING", "injected artifact validator is required")
        process = process_factory(list(argv), cwd=str(root), shell=False)
        returncode = int(process.wait())
        artifacts = artifact_validator(root)
        if not isinstance(artifacts, Mapping):
            raise CandidateError("ARTIFACT_VALIDATOR", "artifact result is not a mapping")
        status = "PASS" if returncode == 0 and artifacts.get("status") == "PASS" else "FAIL_CLOSED"
        failure_kind = None if status == "PASS" else "INJECTED_RUNTIME_FAILURE"
        failure_message = None if status == "PASS" else str(artifacts.get("reason", "runtime failed"))
        return _seal(root, {
            "status": status,
            "failure_kind": failure_kind,
            "failure_message": failure_message,
            "profile": dict(profile),
            "identity": dict(identity),
            "authorization": dict(authorization),
            "injected_runtime": True,
            "test_only_root_alias": root.resolve() != Path(shared_auth.V22_ATTEMPT_ROOT),
            "execution": {
                "formal_replay_started": False,
                "candidate_process_started": True,
                "one_start": True,
                "popen_count": 1,
                "retry": False,
                "manual_stop": False,
                "returncode": returncode,
                "shell": False,
                "image_id": shared_auth.V17_IMAGE_ID,
                "network": "none",
                "rootfs": "read_only",
                "rw_mount_destination": "/out",
                "input_mount_count": 0,
            },
            "artifacts": dict(artifacts),
            "safety": {
                "formal_replay_started": False,
                "input_opened": False,
                "ground_truth_content_opened": False,
                "scorer_invoked": False,
                "map_saved": False,
            },
        })
    except Exception as exc:
        if root.exists() and not (root / RECEIPT_NAME).exists():
            return _seal(root, {
                "status": "FAIL_CLOSED",
                "failure_kind": getattr(exc, "kind", "V23_INJECTED_FAILURE"),
                "failure_message": str(exc),
                "profile": dict(profile),
                "identity": dict(identity),
                "authorization": dict(authorization),
                "injected_runtime": True,
                "test_only_root_alias": root.resolve() != Path(shared_auth.V22_ATTEMPT_ROOT),
                "execution": {"formal_replay_started": False, "candidate_process_started": False,
                               "one_start": False, "popen_count": 0, "retry": False,
                               "manual_stop": False, "returncode": None, "shell": False},
                "safety": {"formal_replay_started": False, "input_opened": False,
                           "ground_truth_content_opened": False, "scorer_invoked": False,
                           "map_saved": False},
            })
        raise


def run_formal(*, root: Path, repo_root: Path = ROOT,
               authorization_path: Path = shared_auth.V22_AUTHORIZATION_RECEIPT_PATH,
               authorization_sha256: str = shared_auth.V22_AUTHORIZATION_SHA256,
               runtime: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Validate shared authorization and run only the explicit test seam."""
    profile = verify_profile(repo_root)
    identity = shared_auth.verify_candidate_identity(repo_root=repo_root, launcher_path=V23_LAUNCHER_PATH)
    authorization = shared_auth.verify_v22_authorization(
        path=authorization_path, expected_sha256=authorization_sha256,
        expected_attempt_root=shared_auth.V22_ATTEMPT_ROOT, repo_root=repo_root)
    if runtime is None or runtime.get("injected") is not True:
        raise CandidateError("V23_FORMAL_UNAUTHORIZED", "production v23 runtime is not installed")
    if "authorization_validator" in runtime:
        raise CandidateError("AUTHORIZATION_BYPASS_FORBIDDEN", "fake authorization validators are forbidden")
    if root.resolve() != Path(shared_auth.V22_ATTEMPT_ROOT).resolve() and \
            runtime.get("test_only_root_alias") is not True:
        raise CandidateError("ROOT_IDENTITY", "production root differs from authorized exact root")
    return _run_injected(root, profile, identity, authorization, runtime)


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--authorization", type=Path, default=shared_auth.V22_AUTHORIZATION_RECEIPT_PATH)
    parser.add_argument("--authorization-sha256", default=shared_auth.V22_AUTHORIZATION_SHA256)
    args = parser.parse_args(argv)
    try:
        result = run_formal(root=args.root, repo_root=args.repo_root,
                            authorization_path=args.authorization,
                            authorization_sha256=args.authorization_sha256,
                            runtime=runtime)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V23_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
