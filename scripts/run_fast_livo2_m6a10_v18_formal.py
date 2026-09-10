#!/usr/bin/env python3
"""Additive v18 formal candidate with a shared authorization verifier.

The default path is deliberately execution-inert: v18 is an unauthorized
candidate and its default process factory refuses to start Docker.  Tests may
inject a shell-free process/monitor/composition seam to prove one Popen and an
immutable closure without opening a bag or invoking scoring.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.verify_fast_livo2_m6a10_v18_authorization as authorization  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v17_formal as v17  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v12_formal as v12  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v15_formal as v15  # noqa: E402


CANDIDATE_VERSION = "v18-shared-authorization-verifier"
CONTRACT_VERSION = "m6a10-v18-formal-candidate-closure-v1"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v18_formal_candidate.yaml"
PROFILE_SHA256 = "ed02e9ef631d3932ef8f6152742a4283b351f53f765702e3cb22ce51ce92f587"
VERIFIER_PATH = ROOT / "scripts/verify_fast_livo2_m6a10_v18_authorization.py"
VERIFIER_SHA256 = "b40caf950cd63b5e2893f1cd16df862bf2c4390b8d90fc64996b0f443ecc6e4c"
IMAGE_TAG = authorization.IMAGE_TAG
IMAGE_ID = authorization.IMAGE_ID
INPUT_PATH = authorization.INPUT_PATH
EXPECTED_COUNTS = dict(authorization.EXPECTED_COUNTS)
EXPECTED_MESSAGES = authorization.EXPECTED_MESSAGES
REQUIRED_END_TIMESTAMP_SECONDS = authorization.REQUIRED_END_TIMESTAMP_SECONDS
SENSOR_DURATION_SECONDS = authorization.SENSOR_DURATION_SECONDS
WATCHDOG_SECONDS = authorization.WATCHDOG_SECONDS
RECEIPT_NAME = "closure_receipt.json"
WRAPPER_PATH_IN_CONTAINER = v17.WRAPPER_PATH_IN_CONTAINER
AUTHORIZATION_PATH = authorization.V18_AUTHORIZATION_RECEIPT_PATH
ATTEMPT_ROOT = authorization.V18_ATTEMPT_ROOT
BASE_V12_ATOMIC_CREATE_JSON = v12._atomic_create_json

CandidateError = authorization.AuthorizationError
ProcessFactory = Callable[[Sequence[str], Path], Any]
Verifier = Callable[..., Mapping[str, Any]]


@dataclass(frozen=True)
class CandidateConfig:
    root: Path
    repo_root: Path = ROOT
    bag_path: str = INPUT_PATH
    profile_path: Path = PROFILE_PATH
    container_name: str = "m6a10-v18-formal-candidate"
    authorization_path: Optional[Path] = None
    authorization_sha256: Optional[str] = None


def sha256_file(path: Path) -> str:
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
    if os.path.lexists(part):
        raise CandidateError("OUTPUT_STAGING", "staging file already exists")
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
    except FileExistsError as exc:
        raise CandidateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path) from exc
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)
    return hashlib.sha256(payload).hexdigest()


def _atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    return _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"), mode)


def verify_profile(profile_path: Path = PROFILE_PATH, repo_root: Path = ROOT) -> Dict[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v18 profile path drift")
    if PROFILE_SHA256.startswith("TO_BE_"):
        raise CandidateError("PROFILE_SHA256", "v18 profile pin is not bound")
    if sha256_file(profile_path) != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v18 profile SHA drift")
    try:
        import yaml
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise CandidateError("PROFILE_INVALID", "v18 profile cannot be parsed") from exc
    if not isinstance(document, Mapping) or document.get("schema_version") != 1 or \
            document.get("status") != "V18_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or document.get("replay_count") != 0:
        raise CandidateError("PROFILE_AUTHORITY", "v18 profile is not unauthorized")
    candidate = document.get("candidate")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED":
        raise CandidateError("PROFILE_STATUS", "v18 candidate status drift")
    if candidate.get("shared_verifier") != str(VERIFIER_PATH.relative_to(repo_root)) or \
            candidate.get("shared_verifier_sha256") != VERIFIER_SHA256 or \
            candidate.get("authorization_receipt_path") != str(AUTHORIZATION_PATH) or \
            candidate.get("formal_attempt_root") != str(ATTEMPT_ROOT):
        raise CandidateError("PROFILE_AUTHORITY", "v18 verifier/root authorization pin drift")
    previous = candidate.get("previous_preflight_failure")
    if not isinstance(previous, Mapping) or previous.get("path") != str(authorization.V17_PREFLIGHT_CLOSURE_PATH) or \
            previous.get("sha256") != authorization.V17_PREFLIGHT_CLOSURE_SHA256 or \
            previous.get("failure_kind") != "V16_FORMAL_REPLAY_UNAUTHORIZED" or \
            previous.get("formal_replay_started") is not False:
        raise CandidateError("PROFILE_LINEAGE", "v17 pre-Popen failure lineage drift")
    image = document.get("image")
    if not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG:
        raise CandidateError("PROFILE_IMAGE", "v18 image identity drift")
    return {"path": str(profile_path), "sha256": PROFILE_SHA256, "image_id": IMAGE_ID,
            "status": document.get("status")}


def _safe_name(value: str) -> str:
    if not value or re.fullmatch(r"[A-Za-z0-9_.-]+", value) is None:
        raise CandidateError("CONTAINER_NAME", "container name is not a fixed safe token")
    return value


def build_safe_docker_argv(config: CandidateConfig, output_dir: Path) -> Sequence[str]:
    """Reuse the proven v17 production argv with v18 authorization state."""
    if config.authorization_path is None or not config.authorization_sha256:
        raise CandidateError("AUTHORIZATION_REQUIRED", "v18 argv requires a verified authorization")
    argv = list(v17.build_safe_docker_argv(config, output_dir))
    if argv[-1] != IMAGE_ID or "--network" not in argv or argv[argv.index("--network") + 1] != "none":
        raise CandidateError("DOCKER_ARGV", "v18 image/network argv drift")
    mounts = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]
    destinations = [field[4:] for spec in mounts for field in spec.split(",") if field.startswith("dst=")]
    if len(destinations) != 4 or len(set(destinations)) != 4 or "/out" not in destinations:
        raise CandidateError("DOCKER_ARGV", "v18 mount contract drift")
    if "--rm" in argv or any(item in {"rw", "sh", "bash", "-c", "-lc"} for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe v18 argv token")
    if "M6A10_V17_AUTHORIZATION_STATUS=AUTHORIZED_FOR_EXACT_ROOT" not in [
            argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--env"]:
        raise CandidateError("DOCKER_ENV", "authorization state was not exported")
    return argv


def _reserve_root(root: Path) -> None:
    if os.path.lexists(root):
        raise CandidateError("ROOT_NOT_FRESH", "formal candidate root already exists")
    root.parent.mkdir(parents=True, exist_ok=True)
    root.mkdir()


def _seal(root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    enriched = dict(value)
    enriched.update({"candidate_version": CANDIDATE_VERSION, "v18_contract_version": CONTRACT_VERSION,
                     "shared_verifier_path": str(VERIFIER_PATH), "shared_verifier_sha256": VERIFIER_SHA256})
    path = root / RECEIPT_NAME
    receipt_sha = _atomic_json(path, enriched, 0o444)
    sidecar = path.with_name(path.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, (receipt_sha + "  " + path.name + "\n").encode("ascii"), 0o444)
    return dict(enriched, receipt_path=str(path), receipt_sha256=receipt_sha,
                sidecar_path=str(sidecar), sidecar_sha256=sidecar_sha)


def _default_identity(_config: CandidateConfig) -> Mapping[str, Any]:
    return {"image_id": IMAGE_ID, "tag": IMAGE_TAG, "opened": False}


def _default_bag(_config: CandidateConfig) -> Mapping[str, Any]:
    return {"path": INPUT_PATH, "bytes": authorization.INPUT_BYTES, "sha256": authorization.INPUT_SHA256,
            "opened": False}


def _default_process(_argv: Sequence[str], _cwd: Path) -> Any:
    raise CandidateError("V18_RUNTIME_NOT_INSTALLED", "v18 default process factory is execution-inert")


class _NoopMonitor:
    def start(self) -> None:
        return None

    def allow_owned_pid(self, _pid: int) -> None:
        return None

    def stop(self) -> None:
        return None

    def finalize(self, _path: Path) -> Mapping[str, Any]:
        return {"status": "PASS", "contaminated": False, "invalid": False,
                "coverage": {"coverage_gap": False}}


def run_formal(config: CandidateConfig, *, receipt_verifier: Verifier = authorization.verify_authorization,
               identity_probe: Optional[Callable[[CandidateConfig], Mapping[str, Any]]] = None,
               bag_probe: Optional[Callable[[CandidateConfig], Mapping[str, Any]]] = None,
               process_factory: ProcessFactory = _default_process,
               monitor_factory: Optional[Callable[[Path], Any]] = None,
               argv_builder: Callable[[CandidateConfig, Path], Sequence[str]] = build_safe_docker_argv,
               composer: Optional[Callable[[Mapping[str, Any], CandidateConfig], Mapping[str, Any]]] = None,
               now: Optional[str] = None) -> Dict[str, Any]:
    authorization_value: Optional[Mapping[str, Any]] = None
    try:
        if VERIFIER_SHA256.startswith("TO_BE_") or sha256_file(VERIFIER_PATH) != VERIFIER_SHA256:
            raise CandidateError("VERIFIER_DRIFT", "v18 shared verifier SHA drift")
        if config.authorization_path is None or not config.authorization_sha256:
            raise CandidateError("V18_AUTHORIZATION_REQUIRED", "v18 authorization is required")
        authorization_value = receipt_verifier(
            config.authorization_path, config.root, config.authorization_sha256, repo_root=config.repo_root)
        if authorization_value.get("authorized") is not True or authorization_value.get("formal_execution") is not True:
            raise CandidateError("AUTHORIZATION_STATUS", "v18 authorization is not executable")
    except Exception as exc:
        if os.path.lexists(config.root):
            raise
        _reserve_root(config.root)
        return _seal(config.root, {
            "schema_version": 1, "contract_version": CONTRACT_VERSION, "status": "FAIL_CLOSED",
            "created_at_utc": now or "injected", "authorization": {"formal_replay_authorized": False,
                "exact_root": str(config.root)}, "execution": {"formal_replay_started": False,
                "candidate_process_started": False, "one_start": False, "popen_count": 0,
                "retry": False, "manual_stop": False, "returncode": None, "shell": False},
            "failure_kind": getattr(exc, "kind", "AUTHORIZATION_FAIL_CLOSED"),
            "failure_message": str(exc), "safety": {"formal_replay_started": False,
                "input_opened": False, "ground_truth_content_opened": False,
                "scorer_invoked": False, "map_saved": False},
        })

    profile: Optional[Mapping[str, Any]] = None
    try:
        profile = verify_profile(config.profile_path, config.repo_root)
        _reserve_root(config.root)
        output_dir = config.root / "out"
        output_dir.mkdir()
        identity = (identity_probe or _default_identity)(config)
        bag = (bag_probe or _default_bag)(config)
        argv = list(argv_builder(config, output_dir))
    except Exception as exc:
        if not os.path.lexists(config.root):
            _reserve_root(config.root)
        return _seal(config.root, {
            "schema_version": 1, "contract_version": CONTRACT_VERSION, "status": "FAIL_CLOSED",
            "created_at_utc": now or "injected", "profile": profile,
            "authorization": dict(authorization_value or {}), "execution": {
                "formal_replay_started": False, "candidate_process_started": False,
                "one_start": False, "popen_count": 0, "retry": False, "manual_stop": False,
                "returncode": None, "shell": False}, "failure_kind": getattr(exc, "kind", "PREFLIGHT_FAIL_CLOSED"),
            "failure_message": str(exc), "safety": {"formal_replay_started": False,
                "input_opened": False, "ground_truth_content_opened": False,
                "scorer_invoked": False, "map_saved": False},
        })

    monitor = (monitor_factory or (lambda _root: _NoopMonitor()))(config.root)
    started = False
    returncode: Optional[int] = None
    monitor_summary: Mapping[str, Any] = {}
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    process: Any = None
    try:
        monitor.start()
        process = process_factory(argv, config.repo_root)
        started = True
        pid = getattr(process, "pid", None)
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
            raise CandidateError("RUNNER_PID_INVALID", "runner did not expose a positive PID")
        monitor.allow_owned_pid(pid)
        returncode = int(process.wait())
    except Exception as exc:
        failure_kind = getattr(exc, "kind", "PROCESS_LIFECYCLE_FAILURE")
        failure_message = str(exc)
    finally:
        try:
            monitor.stop()
            monitor_summary = dict(monitor.finalize(config.root / "host_interference.summary.json"))
        except Exception as exc:
            failure_kind = failure_kind or "MONITOR_FAIL_CLOSED"
            failure_message = failure_message or str(exc)

    if returncode not in (None, 0):
        failure_kind = failure_kind or "PROCESS_FAILURE"
        failure_message = failure_message or "runner exited with code %d" % returncode
    if monitor_summary.get("status") != "PASS" or monitor_summary.get("contaminated") is True or \
            monitor_summary.get("invalid") is True or monitor_summary.get("coverage", {}).get("coverage_gap") is True:
        failure_kind = failure_kind or "HOST_INTERFERENCE_FAIL_CLOSED"
        failure_message = failure_message or "monitor evidence is not admissible"
    composed: Optional[Mapping[str, Any]] = None
    if failure_kind is None:
        if composer is None:
            failure_kind = "COMPOSER_NOT_INSTALLED"
            failure_message = "v18 composition seam is required"
        else:
            try:
                composed = composer({"argv": argv, "returncode": returncode}, config)
                if composed.get("status") != "PASS":
                    failure_kind = "COMPOSITION_FAIL_CLOSED"
                    failure_message = "composition did not PASS"
            except Exception as exc:
                failure_kind = getattr(exc, "kind", "COMPOSITION_FAIL_CLOSED")
                failure_message = str(exc)
    status = "PASS" if failure_kind is None else "FAIL_CLOSED"
    bag_opened = bool(isinstance(bag, Mapping) and bag.get("opened") is True)
    return _seal(config.root, {
        "schema_version": 1, "contract_version": CONTRACT_VERSION, "status": status,
        "created_at_utc": now or "injected", "profile": profile,
        "authorization": dict(authorization_value or {}), "identity": dict(identity),
        "bag_identity": dict(bag), "execution": {"formal_replay_started": started,
            "candidate_process_started": started, "one_start": started, "popen_count": int(started),
            "retry": False, "manual_stop": False, "returncode": returncode,
            "argv": argv, "shell": False}, "monitor": dict(monitor_summary),
        "composition": dict(composed) if isinstance(composed, Mapping) else None,
        "failure_kind": failure_kind, "failure_message": failure_message,
        "safety": {"formal_replay_started": started, "input_opened": bag_opened,
            "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False},
    })


def _enriched_atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    if path.name == RECEIPT_NAME:
        enriched = dict(value)
        enriched["contract_version"] = CONTRACT_VERSION
        enriched.update({"candidate_version": CANDIDATE_VERSION,
                         "v18_contract_version": CONTRACT_VERSION,
                         "shared_verifier_path": str(VERIFIER_PATH),
                         "shared_verifier_sha256": VERIFIER_SHA256,
                         "v18_profile_path": str(PROFILE_PATH),
                         "v18_profile_sha256": PROFILE_SHA256})
        value = enriched
    return BASE_V12_ATOMIC_CREATE_JSON(path, value, mode)


def _run_production_formal(config: CandidateConfig) -> Mapping[str, Any]:
    """Run the existing production lifecycle under the shared v18 verifier.

    v12/v17 provide the tested Docker, monitor, capture, and composition
    machinery.  The v18 layer only changes identity/profile/authorization and
    reuses the already sealed authorization windows instead of sampling a
    second quiescence sequence.
    """
    verify_profile(config.profile_path, config.repo_root)
    authorization_holder: Dict[str, Mapping[str, Any]] = {}

    def validator(candidate_config: Any) -> Mapping[str, Any]:
        if candidate_config.authorization_path is None or not candidate_config.authorization_sha256:
            raise CandidateError("V18_AUTHORIZATION_REQUIRED", "v18 authorization is required")
        value = authorization.verify_authorization(
            candidate_config.authorization_path, candidate_config.root,
            candidate_config.authorization_sha256, repo_root=candidate_config.repo_root)
        authorization_holder["value"] = value
        return value

    def sealed_quiescence(_candidate_config: Any) -> Mapping[str, Any]:
        value = authorization_holder.get("value")
        if not isinstance(value, Mapping) or value.get("status") != "AUTHORIZED":
            raise CandidateError("AUTHORIZATION_QUIESCENCE", "sealed v18 quiescence is unavailable")
        return {"status": "PASS", "window_count": 3, "consecutive_passes": 3,
                "windows": value.get("windows", []), "source": "sealed_v18_authorization"}

    snapshot = {name: getattr(v12, name) for name in (
        "PROFILE_PATH", "PROFILE_SHA256", "READY_PROFILE_PATH", "READY_PROFILE_SHA256",
        "IMAGE_TAG", "IMAGE_ID", "PHASE_CONTRACT", "TRANSPORT_CONTRACT", "CONTRACT_VERSION",
        "RECEIPT_NAME", "verify_candidate_profile", "build_safe_docker_argv",
        "_production_identity_probe", "_production_capture", "_production_compose",
        "_atomic_create_json", "_continuous_quiescence")}
    v12.PROFILE_PATH = v17.PROFILE_PATH
    v12.PROFILE_SHA256 = v17.PROFILE_SHA256
    v12.READY_PROFILE_PATH = v17.READY_PROFILE_PATH
    v12.READY_PROFILE_SHA256 = v17.READY_PROFILE_SHA256
    v12.IMAGE_TAG = v17.IMAGE_TAG
    v12.IMAGE_ID = v17.IMAGE_ID
    v12.PHASE_CONTRACT = v17.PHASE_CONTRACT
    v12.TRANSPORT_CONTRACT = v17.TRANSPORT_CONTRACT
    v12.CONTRACT_VERSION = CONTRACT_VERSION
    v12.RECEIPT_NAME = RECEIPT_NAME
    v12.verify_candidate_profile = v17.verify_candidate_profile
    v12.build_safe_docker_argv = build_safe_docker_argv
    v12._production_identity_probe = v15._production_identity_probe
    v12._production_capture = v17._capture_with_persistence_diagnostics
    v12._production_compose = v12._production_compose
    v12._atomic_create_json = _enriched_atomic_json
    v12._continuous_quiescence = sealed_quiescence
    production_config = v12.CandidateConfig(
        root=config.root, repo_root=config.repo_root, bag_path=config.bag_path,
        profile_path=v17.PROFILE_PATH, container_name=config.container_name,
        authorization_path=config.authorization_path,
        authorization_sha256=config.authorization_sha256,
    )
    try:
        return v12.run_formal(production_config, authorization_validator=validator)
    finally:
        for name, value in snapshot.items():
            setattr(v12, name, value)


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v18-formal-candidate")
    args = parser.parse_args(argv)
    config = CandidateConfig(root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                             authorization_path=args.authorization,
                             authorization_sha256=args.authorization_sha256,
                             container_name=args.container_name)
    try:
        result = (_run_production_formal(config) if runtime is None else
                  run_formal(config, **dict(runtime)))
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V18_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")},
                     sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
