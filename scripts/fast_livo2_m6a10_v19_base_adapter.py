#!/usr/bin/env python3
"""Typed, fail-closed access to the real v12 lifecycle module.

Later formal candidates must not reach into the v12 module while assembling a
snapshot.  This adapter imports the installed source module, checks the exact
allowlisted surface, and exposes only typed fields to the caller.  It does not
start Docker, ROS, or any benchmark process.
"""

from __future__ import annotations

from dataclasses import dataclass
import ast
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import types
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
BASE_MODULE_NAME = "run_fast_livo2_m6a10_v12_formal"
BASE_MODULE_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v12_formal.py"
V17_MODULE_NAME = "run_fast_livo2_m6a10_v17_formal"
V17_MODULE_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v17_formal.py"
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# This is intentionally an explicit allowlist of symbols that the adapter
# surface may expose.  In particular, CONTRACT_VERSION is not a v12 symbol;
# treating it as optional would recreate the v18 pre-Popen failure.
REQUIRED_SYMBOLS: Tuple[str, ...] = (
    "PROFILE_PATH",
    "PROFILE_SHA256",
    "READY_PROFILE_PATH",
    "READY_PROFILE_SHA256",
    "PHASE_CONTRACT",
    "TRANSPORT_CONTRACT",
    "IMAGE_TAG",
    "IMAGE_ID",
    "CandidateConfig",
    "verify_candidate_profile",
    "build_safe_docker_argv",
    "_production_identity_probe",
    "_production_capture",
    "_production_compose",
    "_atomic_create_json",
    "_continuous_quiescence",
    "run_formal",
)
CALLABLE_SYMBOLS = frozenset({
    "CandidateConfig", "verify_candidate_profile", "build_safe_docker_argv",
    "_production_identity_probe", "_production_capture", "_production_compose",
    "_atomic_create_json", "_continuous_quiescence", "run_formal",
})
FORBIDDEN_SYMBOLS = frozenset({"CONTRACT_VERSION"})


class BaseAdapterError(ValueError):
    """Stable fail-closed error for an invalid base-module surface."""

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
    current = Path(path.anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise BaseAdapterError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if not os.path.lexists(path) or path.is_symlink() or not path.is_file():
        raise BaseAdapterError("NOT_REGULAR", "%s is not a regular file" % label)
    if immutable and (path.stat().st_mode & 0o777) != 0o444:
        raise BaseAdapterError("IMMUTABILITY", "%s is not mode 0444" % label)


def _sidecar(path: Path, file_sha: str, expected_sidecar_sha: str, label: str) -> None:
    sidecar = path.with_name(path.name + ".sha256")
    _regular(sidecar, "%s sidecar" % label, immutable=True)
    observed = sha256_file(sidecar)
    if observed != expected_sidecar_sha:
        raise BaseAdapterError("RECEIPT_SIDECAR_SHA", "%s sidecar SHA drift" % label)
    expected = (file_sha + "  " + path.name + "\n").encode("ascii")
    if sidecar.read_bytes() != expected:
        raise BaseAdapterError("RECEIPT_SIDECAR_CONTENT", "%s sidecar content drift" % label)


def _json_receipt(path: Path, expected_sha: str, expected_sidecar_sha: str, label: str) -> Dict[str, Any]:
    _regular(path, label, immutable=True)
    observed = sha256_file(path)
    if observed != expected_sha:
        raise BaseAdapterError("RECEIPT_SHA", "%s SHA drift" % label)
    _sidecar(path, observed, expected_sidecar_sha, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BaseAdapterError("RECEIPT_JSON", "%s is not valid JSON" % label) from exc
    if not isinstance(value, dict):
        raise BaseAdapterError("RECEIPT_JSON", "%s must be an object" % label)
    return value


@dataclass(frozen=True)
class BaseV12Surface:
    """The only v12 values/functions available to a later launcher."""

    module: types.ModuleType
    module_path: Path
    module_sha256: str
    profile_path: Path
    profile_sha256: str
    ready_profile_path: Path
    ready_profile_sha256: str
    phase_contract: str
    transport_contract: str
    image_tag: str
    image_id: str
    candidate_config: Any
    verify_candidate_profile: Callable[..., Mapping[str, Any]]
    build_safe_docker_argv: Callable[..., Sequence[str]]
    production_identity_probe: Callable[..., Mapping[str, Any]]
    production_capture: Callable[..., Mapping[str, Any]]
    production_compose: Callable[..., Mapping[str, Any]]
    atomic_create_json: Callable[..., str]
    continuous_quiescence: Callable[..., Mapping[str, Any]]
    run_formal: Callable[..., Mapping[str, Any]]

    def as_identity(self) -> Dict[str, Any]:
        return {
            "module": BASE_MODULE_NAME,
            "path": str(self.module_path),
            "sha256": self.module_sha256,
            "required_symbols": list(REQUIRED_SYMBOLS),
            "phase_contract": self.phase_contract,
            "transport_contract": self.transport_contract,
            "image_tag": self.image_tag,
            "image_id": self.image_id,
        }


@dataclass(frozen=True)
class V17RuntimeSurface:
    """The immutable v17 production seams used by the v19 lifecycle.

    The v17 module remains an immutable predecessor.  This typed view is
    deliberately assembled from its real namespace and the real v12 module
    it scopes; no test-only clone can satisfy the path and hash checks.
    """

    module: types.ModuleType
    module_path: Path
    module_sha256: str
    profile_path: Path
    profile_sha256: str
    image_tag: str
    image_id: str
    verify_candidate_profile: Callable[..., Mapping[str, Any]]
    build_safe_docker_argv: Callable[..., Sequence[str]]
    production_identity_probe: Callable[..., Mapping[str, Any]]
    production_bag_probe: Callable[..., Mapping[str, Any]]
    production_capture: Callable[..., Mapping[str, Any]]
    production_compose: Callable[..., Mapping[str, Any]]
    scoped_runtime: Callable[..., Any]
    default_monitor: Callable[..., Any]
    default_popen: Callable[..., Any]

    def as_identity(self) -> Dict[str, Any]:
        return {
            "module": V17_MODULE_NAME,
            "path": str(self.module_path),
            "sha256": self.module_sha256,
            "profile_path": str(self.profile_path),
            "profile_sha256": self.profile_sha256,
            "image_tag": self.image_tag,
            "image_id": self.image_id,
            "production_surface": [
                "verify_candidate_profile", "build_safe_docker_argv",
                "_production_identity_probe", "_production_bag_probe",
                "_capture_with_persistence_diagnostics", "BASE_PRODUCTION_COMPOSE",
                "scoped_runtime", "_default_monitor", "_default_popen",
            ],
        }


def _load_actual_module() -> types.ModuleType:
    """Import the repository's actual v12 module, never a test-only clone."""
    _regular(BASE_MODULE_PATH, "v12 base module")
    if BASE_MODULE_PATH.resolve() != (ROOT / "scripts/run_fast_livo2_m6a10_v12_formal.py").resolve():
        raise BaseAdapterError("BASE_PATH", "v12 base module path drift")
    module = importlib.import_module(BASE_MODULE_NAME)
    if not isinstance(module, types.ModuleType):
        raise BaseAdapterError("BASE_IMPORT", "v12 import did not produce a module")
    module_file = vars(module).get("__file__")
    if not isinstance(module_file, str) or Path(module_file).resolve() != BASE_MODULE_PATH.resolve():
        raise BaseAdapterError("BASE_IMPORT", "v12 imported module path drift")
    return module


def load_base_surface(module: Optional[types.ModuleType] = None) -> BaseV12Surface:
    """Load and validate the actual v12 symbol surface.

    ``module`` is an explicit unit-test seam only; production callers omit it,
    which forces an import of the real source file above.
    """
    actual = module if module is not None else _load_actual_module()
    if not isinstance(actual, types.ModuleType):
        raise BaseAdapterError("BASE_IMPORT", "base module is not a module")
    namespace = vars(actual)
    forbidden = sorted(name for name in FORBIDDEN_SYMBOLS if name in namespace)
    if forbidden:
        raise BaseAdapterError("BASE_SYMBOL_FORBIDDEN", "forbidden v12 symbols: %s" % ",".join(forbidden))
    missing = [name for name in REQUIRED_SYMBOLS if name not in namespace]
    if missing:
        raise BaseAdapterError("BASE_SYMBOL_MISSING", "missing v12 symbols: %s" % ",".join(missing))
    for name in CALLABLE_SYMBOLS:
        if not callable(namespace[name]):
            raise BaseAdapterError("BASE_SYMBOL_TYPE", "%s is not callable" % name)
    values = namespace
    phase = values["PHASE_CONTRACT"]
    transport = values["TRANSPORT_CONTRACT"]
    if not isinstance(phase, str) or not phase or not isinstance(transport, str) or not transport:
        raise BaseAdapterError("BASE_SYMBOL_TYPE", "contract symbols must be non-empty strings")
    module_path = BASE_MODULE_PATH
    return BaseV12Surface(
        module=actual,
        module_path=module_path,
        module_sha256=sha256_file(module_path),
        profile_path=values["PROFILE_PATH"],
        profile_sha256=values["PROFILE_SHA256"],
        ready_profile_path=values["READY_PROFILE_PATH"],
        ready_profile_sha256=values["READY_PROFILE_SHA256"],
        phase_contract=phase,
        transport_contract=transport,
        image_tag=values["IMAGE_TAG"],
        image_id=values["IMAGE_ID"],
        candidate_config=values["CandidateConfig"],
        verify_candidate_profile=values["verify_candidate_profile"],
        build_safe_docker_argv=values["build_safe_docker_argv"],
        production_identity_probe=values["_production_identity_probe"],
        production_capture=values["_production_capture"],
        production_compose=values["_production_compose"],
        atomic_create_json=values["_atomic_create_json"],
        continuous_quiescence=values["_continuous_quiescence"],
        run_formal=values["run_formal"],
    )


def load_v17_runtime_surface(base: Optional[BaseV12Surface] = None) -> V17RuntimeSurface:
    """Load the actual v17 argv/Popen/monitor/artifact/composition surface.

    The v17 module is imported from its pinned repository path.  Its scoped
    runtime must reference the same actual v12 module exposed by this
    adapter; otherwise the production boundary is rejected before any bag or
    Docker operation.
    """
    _regular(V17_MODULE_PATH, "v17 launcher")
    actual = importlib.import_module(V17_MODULE_NAME)
    if not isinstance(actual, types.ModuleType):
        raise BaseAdapterError("V17_IMPORT", "v17 import did not produce a module")
    module_file = vars(actual).get("__file__")
    if not isinstance(module_file, str) or Path(module_file).resolve() != V17_MODULE_PATH.resolve():
        raise BaseAdapterError("V17_IMPORT", "v17 imported module path drift")
    namespace = vars(actual)
    required = (
        "PROFILE_PATH", "PROFILE_SHA256", "IMAGE_TAG", "IMAGE_ID",
        "verify_candidate_profile", "build_safe_docker_argv",
        "_capture_with_persistence_diagnostics", "BASE_PRODUCTION_COMPOSE",
        "scoped_runtime", "v12", "v15",
    )
    missing = [name for name in required if name not in namespace]
    if missing:
        raise BaseAdapterError("V17_SYMBOL_MISSING", "missing v17 symbols: %s" % ",".join(missing))
    scoped_base = namespace["v12"]
    if not isinstance(scoped_base, types.ModuleType):
        raise BaseAdapterError("V17_BASE_IMPORT", "v17 scope does not expose a module")
    base_surface = base or load_base_surface()
    if vars(scoped_base).get("__file__") is None or \
            Path(str(vars(scoped_base)["__file__"])).resolve() != base_surface.module_path.resolve():
        raise BaseAdapterError("V17_BASE_IMPORT", "v17 scope points at a different v12 module")
    scoped_v15 = namespace["v15"]
    if not isinstance(scoped_v15, types.ModuleType) or not callable(vars(scoped_v15).get("_production_identity_probe")):
        raise BaseAdapterError("V17_V15_IMPORT", "v17 scope lacks actual v15 identity probe")
    values = vars(scoped_base)
    required_base = ("_default_monitor", "_default_popen", "_production_bag_probe")
    missing_base = [name for name in required_base if name not in values]
    if missing_base:
        raise BaseAdapterError("V17_BASE_SYMBOL_MISSING", "missing scoped v12 symbols: %s" % ",".join(missing_base))
    callables = (
        "verify_candidate_profile", "build_safe_docker_argv",
        "_capture_with_persistence_diagnostics",
        "BASE_PRODUCTION_COMPOSE", "scoped_runtime", "_default_monitor",
        "_default_popen", "_production_bag_probe",
    )
    for name in callables:
        value = namespace.get(name, values.get(name))
        if not callable(value):
            raise BaseAdapterError("V17_SYMBOL_TYPE", "%s is not callable" % name)
    if not callable(vars(scoped_v15)["_production_identity_probe"]):
        raise BaseAdapterError("V17_SYMBOL_TYPE", "_production_identity_probe is not callable")
    return V17RuntimeSurface(
        module=actual,
        module_path=V17_MODULE_PATH,
        module_sha256=sha256_file(V17_MODULE_PATH),
        profile_path=namespace["PROFILE_PATH"],
        profile_sha256=namespace["PROFILE_SHA256"],
        image_tag=namespace["IMAGE_TAG"],
        image_id=namespace["IMAGE_ID"],
        verify_candidate_profile=namespace["verify_candidate_profile"],
        build_safe_docker_argv=namespace["build_safe_docker_argv"],
        production_identity_probe=vars(scoped_v15)["_production_identity_probe"],
        production_bag_probe=values["_production_bag_probe"],
        production_capture=namespace["_capture_with_persistence_diagnostics"],
        production_compose=namespace["BASE_PRODUCTION_COMPOSE"],
        scoped_runtime=namespace["scoped_runtime"],
        default_monitor=values["_default_monitor"],
        default_popen=values["_default_popen"],
    )


def assert_launcher_uses_adapter(path: Path) -> None:
    """AST gate: later launchers may not directly reach into the v12 module."""
    _regular(path, "v19 launcher")
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id == "CONTRACT_VERSION":
            raise BaseAdapterError("AST_DIRECT_BASE_ACCESS", "CONTRACT_VERSION is forbidden in v19 launcher")
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in {
                "v12", "base_module", "base"}:
            raise BaseAdapterError("AST_DIRECT_BASE_ACCESS", "direct base-module attribute access is forbidden")
    source = path.read_text(encoding="utf-8")
    if "run_fast_livo2_m6a10_v12_formal as v12" in source or "import run_fast_livo2_m6a10_v12_formal" in source:
        raise BaseAdapterError("AST_DIRECT_BASE_ACCESS", "v12 must be imported by the adapter")


def verify_v18b_lineage(*, authorization_path: Path, authorization_sha256: str,
                        authorization_sidecar_sha256: str, closure_path: Path,
                        closure_sha256: str, closure_sidecar_sha256: str,
                        expected_attempt_root: Optional[str] = None) -> Dict[str, Any]:
    """Read-only verification of the sealed v18b auth and pre-Popen closure."""
    authorization = _json_receipt(authorization_path, authorization_sha256,
                                  authorization_sidecar_sha256, "v18b authorization")
    if authorization.get("status") != "AUTHORIZED" or authorization.get("authorized") is not True or \
            authorization.get("formal_execution") is not True or authorization.get("formal_replay_started") is not False:
        raise BaseAdapterError("V18B_AUTHORIZATION", "v18b authorization status drift")
    attempt_root = authorization.get("attempt_root")
    if expected_attempt_root is not None and attempt_root != expected_attempt_root:
        raise BaseAdapterError("V18B_ROOT", "v18b attempt root drift")
    closure = _json_receipt(closure_path, closure_sha256, closure_sidecar_sha256, "v18b pre-Popen closure")
    execution = closure.get("execution")
    safety = closure.get("safety")
    if closure.get("status") != "FAIL_CLOSED" or closure.get("failure_kind") != "V18_FAIL_CLOSED" or \
            not isinstance(execution, Mapping) or execution.get("popen_count") != 0 or \
            execution.get("formal_replay_started") is not False or \
            not isinstance(safety, Mapping) or any(safety.get(k) is not False for k in (
                "input_opened", "ground_truth_content_opened", "scorer_invoked", "map_saved")):
        raise BaseAdapterError("V18B_CLOSURE", "v18b pre-Popen closure status drift")
    return {
        "authorization": {"path": str(authorization_path), "sha256": authorization_sha256,
                           "sidecar_sha256": authorization_sidecar_sha256,
                           "status": authorization.get("status"), "attempt_root": attempt_root},
        "closure": {"path": str(closure_path), "sha256": closure_sha256,
                     "sidecar_sha256": closure_sidecar_sha256,
                     "status": closure.get("status"), "failure_kind": closure.get("failure_kind"),
                     "popen_count": execution.get("popen_count")},
    }
