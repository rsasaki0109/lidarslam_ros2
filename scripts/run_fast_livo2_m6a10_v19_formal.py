#!/usr/bin/env python3
"""Additive v19 candidate using the typed v12 base adapter.

The candidate is unauthorized by default.  Its injected lifecycle is useful
for proving the one-start/immutable-closure boundary without starting Docker,
ROS, opening a bag, or invoking scoring.  Production callers can only reach
the v12 surface through ``fast_livo2_m6a10_v19_base_adapter``.
"""

from __future__ import annotations

from dataclasses import dataclass
from contextlib import nullcontext
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.fast_livo2_m6a10_v19_base_adapter as base_adapter  # noqa: E402


CANDIDATE_VERSION = "v19-shared-v12-base-adapter"
CLOSURE_CONTRACT = "m6a10-v19-shared-base-adapter-closure-v1"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v19_formal_candidate.yaml"
PROFILE_SHA256 = "4decdb6d5091a8d0ba04e116e61a238383a2283ea0ba206f54a788108f266f6c"
BASE_ADAPTER_PATH = ROOT / "scripts/fast_livo2_m6a10_v19_base_adapter.py"
BASE_ADAPTER_SHA256 = "61b3396aa54079773aa7066e5652527e27ddaa46121650b711f61807d0c82846"
BASE_MODULE_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v12_formal.py"
V17_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v17_formal.py"
V17_LAUNCHER_SHA256 = "a94a5f2e5da69cde0195572a8b1ab9b5357634604ace15fe098f67307d0244d7"
V17_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_ready.yaml"
V17_PROFILE_SHA256 = "10ddffb6eed647eb3b7a2ee7ede49505923755eb917d8b87737675853e35d9c5"
V17_IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
V17_IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
V17_BUILD_RECEIPT_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json")
V17_BUILD_RECEIPT_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
V17_NO_INPUT_RECEIPT_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/no_input.receipt.json")
V17_NO_INPUT_RECEIPT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v19_formal.py"
AUTHORIZATION_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v19_formal_authorization_20260824T005300Z_agentv19/formal_authorization.receipt.json")
AUTHORIZED_ATTEMPT_ROOT = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v19_formal_replay_20260824T005300Z_agentv19formal")
AUTHORIZATION_CONTRACT = "m6a10-v19-formal-exact-root-authorization-v1"
V18B_AUTHORIZATION_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_authorization_20260824T003000Z_agentv18b/"
    "formal_authorization.receipt.json"
)
V18B_AUTHORIZATION_SHA256 = "78fc1ea039f4c8725a773a81dafca64486341a14eb4c052ddbba3f8d1ee3a291"
V18B_AUTHORIZATION_SIDECAR_SHA256 = "14a84f3fd29b829d151214e3dd611215c07eb4e92511a8bb2ebd727d0d4cef80"
V18B_CLOSURE_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v18_formal_replay_20260824T003000Z_agentv18bformal/"
    "closure_receipt.json"
)
V18B_CLOSURE_SHA256 = "1e532bbf58c5bfffc7315a2b228342228c96fc45fe799bd2a0893605841550fd"
V18B_CLOSURE_SIDECAR_SHA256 = "267dbf0ac1426645f2cac2e1a5c639c942c51e72c063da64ce57201eabfc1901"
V18B_ATTEMPT_ROOT = "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v18_formal_replay_20260824T003000Z_agentv18bformal"
INPUT_PATH = (
    "/media/sasaki/aiueo1/datasets/ntu_viral_release/"
    "tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
)
WATCHDOG_SECONDS = 1200
RECEIPT_NAME = "closure_receipt.json"

CandidateError = base_adapter.BaseAdapterError
ProcessFactory = Callable[[Sequence[str], Path], Any]
AuthorizationValidator = Callable[["CandidateConfig"], Mapping[str, Any]]
IdentityProbe = Callable[["CandidateConfig"], Mapping[str, Any]]
BagProbe = Callable[["CandidateConfig"], Mapping[str, Any]]
Capture = Callable[["CandidateConfig", Path], Mapping[str, Any]]
Composer = Callable[[Mapping[str, Any], "CandidateConfig"], Mapping[str, Any]]


@dataclass(frozen=True)
class CandidateConfig:
    root: Path
    repo_root: Path = ROOT
    bag_path: str = INPUT_PATH
    profile_path: Path = PROFILE_PATH
    container_name: str = "m6a10-v19-formal-candidate"
    authorization_path: Optional[Path] = None
    authorization_sha256: Optional[str] = None


def sha256_file(path: Path) -> str:
    return base_adapter.sha256_file(path)


def _atomic_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise CandidateError("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name + ".part")
    if os.path.lexists(staging):
        raise CandidateError("OUTPUT_STAGING", "staging file exists: %s" % staging)
    fd = os.open(staging, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
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


def _atomic_json(surface: base_adapter.BaseV12Surface, path: Path,
                 value: Mapping[str, Any], mode: int = 0o444) -> str:
    """Use the real base atomic writer through the typed surface."""
    return surface.atomic_create_json(path, value, mode)


def _reserve_root(root: Path) -> None:
    if os.path.lexists(root):
        raise CandidateError("ROOT_NOT_FRESH", "v19 formal root already exists")
    root.parent.mkdir(parents=True, exist_ok=True)
    root.mkdir()


def _seal(surface: base_adapter.BaseV12Surface, root: Path, value: Mapping[str, Any]) -> Dict[str, Any]:
    enriched = dict(value)
    enriched.update({
        "candidate_version": CANDIDATE_VERSION,
        "v19_closure_contract": CLOSURE_CONTRACT,
        "base_adapter_path": str(BASE_ADAPTER_PATH),
        "base_adapter_sha256": BASE_ADAPTER_SHA256,
        "base_surface": surface.as_identity(),
    })
    path = root / RECEIPT_NAME
    receipt_sha = _atomic_json(surface, path, enriched, 0o444)
    sidecar = path.with_name(path.name + ".sha256")
    sidecar_sha = _atomic_bytes(sidecar, (receipt_sha + "  " + path.name + "\n").encode("ascii"), 0o444)
    return dict(enriched, receipt_path=str(path), receipt_sha256=receipt_sha,
                sidecar_path=str(sidecar), sidecar_sha256=sidecar_sha)


def _profile_value(profile_path: Path) -> Mapping[str, Any]:
    try:
        import yaml
        value = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise CandidateError("PROFILE_INVALID", "v19 profile is not valid YAML") from exc
    if not isinstance(value, Mapping):
        raise CandidateError("PROFILE_INVALID", "v19 profile must be a mapping")
    return value


def verify_profile(profile_path: Path = PROFILE_PATH, repo_root: Path = ROOT,
                   surface: Optional[base_adapter.BaseV12Surface] = None) -> Dict[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v19 profile path drift")
    base_adapter._regular(profile_path, "v19 profile")
    if PROFILE_SHA256.startswith("TO_BE_") or sha256_file(profile_path) != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v19 profile SHA is not bound")
    value = _profile_value(profile_path)
    if value.get("schema_version") != 1 or value.get("status") != "V19_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            value.get("formal_replay_forbidden") is not True or \
            value.get("formal_replay_authorized") is not False or value.get("replay_count") != 0:
        raise CandidateError("PROFILE_AUTHORITY", "v19 profile is not unauthorized")
    candidate = value.get("candidate")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED":
        raise CandidateError("PROFILE_STATUS", "v19 candidate status drift")
    if candidate.get("base_adapter") != str(BASE_ADAPTER_PATH.relative_to(repo_root)) or \
            candidate.get("base_adapter_sha256") != BASE_ADAPTER_SHA256 or \
            candidate.get("base_module") != str(BASE_MODULE_PATH.relative_to(repo_root)):
        raise CandidateError("PROFILE_BASE_ADAPTER", "v19 base adapter binding drift")
    expected = surface or base_adapter.load_base_surface()
    if candidate.get("required_symbols") != list(base_adapter.REQUIRED_SYMBOLS) or \
            candidate.get("base_module_sha256") != expected.module_sha256:
        raise CandidateError("PROFILE_BASE_ADAPTER", "v12 symbol/hash binding drift")
    v17 = candidate.get("v17_lineage")
    if not isinstance(v17, Mapping) or v17.get("launcher_path") != str(V17_LAUNCHER_PATH.relative_to(repo_root)) or \
            v17.get("launcher_sha256") != V17_LAUNCHER_SHA256 or \
            v17.get("profile_path") != str(V17_PROFILE_PATH.relative_to(repo_root)) or \
            v17.get("profile_sha256") != V17_PROFILE_SHA256 or \
            v17.get("image_id") != V17_IMAGE_ID or v17.get("image_tag") != V17_IMAGE_TAG or \
            v17.get("build_receipt_path") != str(V17_BUILD_RECEIPT_PATH) or \
            v17.get("build_receipt_sha256") != V17_BUILD_RECEIPT_SHA256 or \
            v17.get("no_input_receipt_path") != str(V17_NO_INPUT_RECEIPT_PATH) or \
            v17.get("no_input_receipt_sha256") != V17_NO_INPUT_RECEIPT_SHA256:
        raise CandidateError("PROFILE_V17_LINEAGE", "v17 image/evidence binding drift")
    previous = candidate.get("v18b_lineage")
    if not isinstance(previous, Mapping) or previous.get("authorization_path") != str(V18B_AUTHORIZATION_PATH) or \
            previous.get("authorization_sha256") != V18B_AUTHORIZATION_SHA256 or \
            previous.get("closure_path") != str(V18B_CLOSURE_PATH) or \
            previous.get("closure_sha256") != V18B_CLOSURE_SHA256:
        raise CandidateError("PROFILE_LINEAGE", "v18b lineage binding drift")
    base_adapter.verify_v18b_lineage(
        authorization_path=V18B_AUTHORIZATION_PATH,
        authorization_sha256=V18B_AUTHORIZATION_SHA256,
        authorization_sidecar_sha256=V18B_AUTHORIZATION_SIDECAR_SHA256,
        closure_path=V18B_CLOSURE_PATH,
        closure_sha256=V18B_CLOSURE_SHA256,
        closure_sidecar_sha256=V18B_CLOSURE_SIDECAR_SHA256,
        expected_attempt_root=V18B_ATTEMPT_ROOT,
    )
    safety = value.get("safety")
    if not isinstance(safety, Mapping) or any(safety.get(key) is not False for key in (
            "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise CandidateError("PROFILE_SAFETY", "v19 profile safety drift")
    return {"path": str(profile_path), "sha256": PROFILE_SHA256,
            "status": value.get("status"), "base_surface": expected.as_identity(),
            "v17_lineage": dict(v17)}


def _default_authorization(config: CandidateConfig) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        raise CandidateError("V19_AUTHORIZATION_REQUIRED", "v19 formal authorization is required")
    base_adapter._regular(AUTHORIZER_PATH, "v19 authorizer")
    spec = importlib.util.spec_from_file_location("m6a10_v19_authorizer_runtime", AUTHORIZER_PATH)
    if spec is None or spec.loader is None:
        raise CandidateError("AUTHORIZER_LOAD", "v19 authorizer cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    value = module.verify_authorization(config.authorization_path, config.root,
                                        config.authorization_sha256, repo_root=config.repo_root)
    if value.get("authorized") is not True or value.get("formal_execution") is not True:
        raise CandidateError("AUTHORIZATION_STATUS", "v19 authorization is not executable")
    return value


def _default_identity(_config: CandidateConfig) -> Mapping[str, Any]:
    raise CandidateError("IDENTITY_PROBE_NOT_INSTALLED", "v19 identity probe is not installed")


def _default_bag(_config: CandidateConfig) -> Mapping[str, Any]:
    raise CandidateError("INPUT_PROBE_NOT_INSTALLED", "v19 input probe is not installed")


def _default_process(_argv: Sequence[str], _cwd: Path) -> Any:
    raise CandidateError("V19_RUNTIME_NOT_INSTALLED", "v19 default process factory is execution-inert")


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


def _run_actual_v17(config: CandidateConfig, surface: base_adapter.BaseV12Surface,
                    profile: Mapping[str, Any], authorization: Mapping[str, Any],
                    now: Optional[str]) -> Dict[str, Any]:
    """Run the real v17 lifecycle seams after v19 authorization.

    This is intentionally separate from the injected test lifecycle.  Every
    production callable comes from the real v17 module through the typed
    adapter: image identity, fixed-input probe, Docker argv, monitor, shell-
    free Popen, persistence capture, and host composition.  The authorizer's
    three windows are consumed as evidence; no second preflight window is
    silently substituted here.
    """
    runtime = base_adapter.load_v17_runtime_surface(surface)
    if runtime.module_sha256 != V17_LAUNCHER_SHA256:
        raise CandidateError("V17_LAUNCHER_SHA256", "v17 production launcher SHA drift")
    runtime.verify_candidate_profile(runtime.profile_path, config.repo_root)
    output_dir = config.root / "out"
    output_dir.mkdir()
    identity: Mapping[str, Any] = {}
    bag_identity: Mapping[str, Any] = {}
    argv: Sequence[str] = ()
    monitor: Any = None
    monitor_summary: Mapping[str, Any] = {}
    process: Any = None
    started = False
    returncode: Optional[int] = None
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    raw: Optional[Mapping[str, Any]] = None
    composed: Optional[Mapping[str, Any]] = None
    monitor_started = False
    context = runtime.scoped_runtime()
    try:
        with context:
            identity = runtime.production_identity_probe(config)
            bag_identity = runtime.production_bag_probe(config)
            argv = list(runtime.build_safe_docker_argv(config, output_dir))
            monitor = runtime.default_monitor(config.root)
            monitor.start()
            monitor_started = True
            process = runtime.default_popen(argv, config.repo_root)
            started = True
            pid = getattr(process, "pid", None)
            if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
                raise CandidateError("RUNNER_PID_INVALID", "v17 runner PID is invalid")
            monitor.allow_owned_pid(pid)
            returncode = int(process.wait())
            # Natural completion is the only normal stop boundary.  No
            # process/container stop is issued from this path.
            monitor.stop()
            monitor_started = False
            monitor_summary = dict(monitor.finalize(config.root / "host_interference.summary.json"))
            raw = runtime.production_capture(config, config.root)
            composed = runtime.production_compose(raw, config)
    except Exception as exc:
        failure_kind = getattr(exc, "kind", "V19_RUNTIME_FAIL_CLOSED")
        failure_message = str(exc)
        # A started process is never forcibly stopped here.  Diagnostics are
        # represented in the closure before any external cleanup authority.
    finally:
        if monitor_started and monitor is not None:
            try:
                monitor.stop()
                monitor_summary = dict(monitor.finalize(config.root / "host_interference.summary.json"))
            except Exception as exc:
                failure_kind = failure_kind or "MONITOR_FAIL_CLOSED"
                failure_message = failure_message or str(exc)
    if returncode not in (None, 0):
        failure_kind = failure_kind or "PROCESS_FAILURE"
        failure_message = failure_message or "v17 runner exited with code %d" % returncode
    oom_killed = returncode == 137
    if oom_killed:
        failure_kind = failure_kind or "OOM_KILLED"
        failure_message = failure_message or "v17 runner exited with OOM code 137"
    if monitor_summary.get("status") != "PASS" or monitor_summary.get("contaminated") is True or \
            monitor_summary.get("invalid") is True or \
            monitor_summary.get("coverage", {}).get("coverage_gap") is True:
        failure_kind = failure_kind or "HOST_INTERFERENCE_FAIL_CLOSED"
        failure_message = failure_message or "host monitor evidence is not admissible"
    if started and raw is None:
        failure_kind = failure_kind or "RAW_CAPTURE_FAIL_CLOSED"
        failure_message = failure_message or "v17 raw evidence was not captured after natural exit"
    if started and composed is None:
        failure_kind = failure_kind or "COMPOSITION_FAIL_CLOSED"
        failure_message = failure_message or "v17 host composition was not produced"
    if composed is not None and composed.get("status") != "PASS":
        failure_kind = failure_kind or "COMPOSITION_FAIL_CLOSED"
        failure_message = failure_message or "v17 host composition did not PASS"
    status = "PASS" if failure_kind is None else "FAIL_CLOSED"
    return _seal(surface, config.root, {
        "schema_version": 1, "contract_version": CLOSURE_CONTRACT, "status": status,
        "created_at_utc": now or "production", "profile": profile,
        "authorization": dict(authorization), "v17_runtime": runtime.as_identity(),
        "identity": dict(identity), "bag_identity": dict(bag_identity),
        "execution": {"formal_replay_started": started, "candidate_process_started": started,
                       "one_start": started, "popen_count": int(started), "retry": False,
                       "manual_stop": False, "returncode": returncode, "argv": list(argv),
                       "shell": False, "natural_completion": bool(started and returncode is not None)},
        "monitor": dict(monitor_summary), "raw_evidence": dict(raw or {}),
        "composition": dict(composed) if isinstance(composed, Mapping) else None,
        "runtime_diagnostics": {"captured_before_cleanup": True, "cleanup": "stopped_only_remove_after_capture",
                                "oom_killed": oom_killed},
        "failure_kind": failure_kind, "failure_message": failure_message,
        "safety": {"formal_replay_started": started,
                    "input_opened": bool(bag_identity.get("opened") is True),
                    "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False},
    })


def run_formal(
        config: CandidateConfig,
        *,
        authorization_validator: Optional[AuthorizationValidator] = None,
        identity_probe: Optional[IdentityProbe] = None,
        bag_probe: Optional[BagProbe] = None,
        process_factory: ProcessFactory = _default_process,
        monitor_factory: Optional[Callable[[Path], Any]] = None,
        argv_builder: Optional[Callable[[CandidateConfig, Path, base_adapter.BaseV12Surface], Sequence[str]]] = None,
        raw_capture: Optional[Capture] = None,
        composer: Optional[Composer] = None,
        now: Optional[str] = None,
) -> Dict[str, Any]:
    surface: Optional[base_adapter.BaseV12Surface] = None
    authorization_value: Optional[Mapping[str, Any]] = None
    profile: Optional[Mapping[str, Any]] = None
    try:
        base_adapter.assert_launcher_uses_adapter(Path(__file__).resolve())
        base_adapter._regular(BASE_ADAPTER_PATH, "v19 base adapter")
        if BASE_ADAPTER_SHA256.startswith("TO_BE_") or sha256_file(BASE_ADAPTER_PATH) != BASE_ADAPTER_SHA256:
            raise CandidateError("BASE_ADAPTER_SHA256", "v19 base adapter SHA is not bound")
        surface = base_adapter.load_base_surface()
        profile = verify_profile(config.profile_path, config.repo_root, surface)
        authorization_value = (authorization_validator or _default_authorization)(config)
        if not isinstance(authorization_value, Mapping) or authorization_value.get("authorized") is not True or \
                authorization_value.get("formal_execution") is not True:
            raise CandidateError("AUTHORIZATION_STATUS", "v19 authorization is not executable")
    except Exception as exc:
        if surface is None:
            # The base module could not be loaded, so no base atomic helper is
            # available.  This path uses a local writer only for the closure.
            if not os.path.lexists(config.root):
                _reserve_root(config.root)
            value = dict({
                "schema_version": 1, "contract_version": CLOSURE_CONTRACT,
                "status": "FAIL_CLOSED", "created_at_utc": now or "injected",
                "failure_kind": getattr(exc, "kind", "BASE_ADAPTER_FAIL_CLOSED"),
                "failure_message": str(exc), "authorization": None,
                "execution": {"formal_replay_started": False, "candidate_process_started": False,
                               "one_start": False, "popen_count": 0, "retry": False,
                               "manual_stop": False, "returncode": None, "shell": False},
                "safety": {"formal_replay_started": False, "input_opened": False,
                           "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False},
            })
            path = config.root / RECEIPT_NAME
            digest = _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode(), 0o444)
            sidecar = path.with_name(path.name + ".sha256")
            side_digest = _atomic_bytes(sidecar, (digest + "  " + path.name + "\n").encode("ascii"), 0o444)
            return dict(value, receipt_path=str(path), receipt_sha256=digest,
                        sidecar_path=str(sidecar), sidecar_sha256=side_digest)
        if not os.path.lexists(config.root):
            _reserve_root(config.root)
        return _seal(surface, config.root, {
            "schema_version": 1, "contract_version": CLOSURE_CONTRACT,
            "status": "FAIL_CLOSED", "created_at_utc": now or "injected",
            "failure_kind": getattr(exc, "kind", "V19_PREFLIGHT_FAIL_CLOSED"),
            "failure_message": str(exc), "profile": profile,
            "authorization": dict(authorization_value or {}),
            "execution": {"formal_replay_started": False, "candidate_process_started": False,
                           "one_start": False, "popen_count": 0, "retry": False,
                           "manual_stop": False, "returncode": None, "shell": False},
            "safety": {"formal_replay_started": False, "input_opened": False,
                       "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False},
        })

    assert surface is not None
    if os.path.lexists(config.root):
        raise CandidateError("ROOT_NOT_FRESH", "v19 root exists after preflight")
    _reserve_root(config.root)
    production_requested = authorization_validator is None and identity_probe is None and \
        bag_probe is None and process_factory is _default_process and monitor_factory is None and \
        argv_builder is None and raw_capture is None and composer is None
    if production_requested:
        return _run_actual_v17(config, surface, profile or {}, authorization_value or {}, now)
    output_dir = config.root / "out"
    output_dir.mkdir()
    identity: Mapping[str, Any] = {}
    bag_identity: Mapping[str, Any] = {}
    argv: Sequence[str] = ()
    monitor = (monitor_factory or (lambda _root: _NoopMonitor()))(config.root)
    process: Any = None
    returncode: Optional[int] = None
    started = False
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    monitor_summary: Mapping[str, Any] = {}
    raw: Optional[Mapping[str, Any]] = None
    composed: Optional[Mapping[str, Any]] = None
    try:
        identity = (identity_probe or _default_identity)(config)
        bag_identity = (bag_probe or _default_bag)(config)
        argv = list(argv_builder(config, output_dir, surface) if argv_builder else
                    surface.build_safe_docker_argv(config, output_dir))
        monitor.start()
        process = process_factory(argv, config.repo_root)
        started = True
        pid = process.pid
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise CandidateError("RUNNER_PID_INVALID", "runner PID is invalid")
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
    if started and raw_capture is not None:
        try:
            raw = raw_capture(config, config.root)
        except Exception as exc:
            failure_kind = failure_kind or "RAW_CAPTURE_FAIL_CLOSED"
            failure_message = failure_message or str(exc)
    if returncode not in (None, 0):
        failure_kind = failure_kind or "PROCESS_FAILURE"
        failure_message = failure_message or "runner exited with code %d" % returncode
    if monitor_summary.get("status") != "PASS" or monitor_summary.get("contaminated") is True or \
            monitor_summary.get("invalid") is True or monitor_summary.get("coverage", {}).get("coverage_gap") is True:
        failure_kind = failure_kind or "HOST_INTERFERENCE_FAIL_CLOSED"
        failure_message = failure_message or "monitor evidence is not admissible"
    if failure_kind is None:
        if composer is None or raw is None:
            failure_kind = "COMPOSITOR_NOT_INSTALLED"
            failure_message = "v19 injected composition is required"
        else:
            try:
                composed = composer(raw, config)
                if composed.get("status") != "PASS":
                    failure_kind = "COMPOSITION_FAIL_CLOSED"
                    failure_message = "composition did not PASS"
            except Exception as exc:
                failure_kind = getattr(exc, "kind", "COMPOSITION_FAIL_CLOSED")
                failure_message = str(exc)
    status = "PASS" if failure_kind is None else "FAIL_CLOSED"
    return _seal(surface, config.root, {
        "schema_version": 1, "contract_version": CLOSURE_CONTRACT, "status": status,
        "created_at_utc": now or "injected", "profile": profile,
        "authorization": dict(authorization_value or {}), "identity": dict(identity),
        "bag_identity": dict(bag_identity),
        "execution": {"formal_replay_started": started, "candidate_process_started": started,
                       "one_start": started, "popen_count": int(started), "retry": False,
                       "manual_stop": False, "returncode": returncode,
                       "argv": list(argv), "shell": False},
        "monitor": dict(monitor_summary), "raw_evidence": dict(raw or {}),
        "composition": dict(composed) if isinstance(composed, Mapping) else None,
        "failure_kind": failure_kind, "failure_message": failure_message,
        "safety": {"formal_replay_started": started,
                    "input_opened": bool(bag_identity.get("opened") is True),
                    "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False},
    })


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v19-formal-candidate")
    args = parser.parse_args(argv)
    config = CandidateConfig(root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                             authorization_path=args.authorization,
                             authorization_sha256=args.authorization_sha256,
                             container_name=args.container_name)
    try:
        result = run_formal(config, **dict(runtime or {}))
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "V19_FAIL_CLOSED"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
