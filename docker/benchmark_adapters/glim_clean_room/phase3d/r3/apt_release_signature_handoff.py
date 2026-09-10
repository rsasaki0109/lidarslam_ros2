#!/usr/bin/env python3
"""Create and validate an unsigned r3 custodian handoff request.

This is a review artifact producer only.  It never generates keys, signatures,
or authorization policy, and it cannot select a caller-provided trust store.
Machine identity is deliberately an offline artifact reference; this module
does not claim that it matches the current host.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[5]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
MODULE_PATH = Path(__file__)
MODULE_RELATIVE = MODULE_PATH.relative_to(ROOT).as_posix()
SCHEMA = "glim_clean_room_r3_custodian_handoff_v1"
SIDECAR_SCHEMA = "glim_clean_room_r3_custodian_handoff_sidecar_v1"
DOMAIN = "lidarslam/glim-clean-room-r3/custodian-handoff/sha256/v1"
MAX_JSON_BYTES = 16 * 1024 * 1024
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
CAMPAIGN_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
NONCE_RE = re.compile(r"^[A-Za-z0-9_-]{32,128}$")


class HandoffError(ValueError):
    """Raised when a custodian request is incomplete or unsafe."""


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise HandoffError(f"cannot load contract: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MACHINE = _load("r3_handoff_machine", R3 / "apt_machine_identity.py")
SIG = _load("r3_handoff_signature", R3 / "apt_release_signature.py")
AUTH = _load("r3_handoff_authorization", R3 / "apt_release_signature_authorization.py")
EXECUTOR = R3 / "apt_release_signature_executor.py"
PLAN = _load("r3_handoff_plan", ROOT / "scripts/plan_glim_clean_room_r3_apt_signature.py")


def _backend_descriptor() -> dict[str, Any]:
    try:
        import cryptography
    except ImportError as error:
        raise HandoffError("cryptography backend is unavailable") from error
    return {**dict(AUTH.BACKEND), "version": getattr(cryptography, "__version__", ""),
            "implementation_sha256": _sha(_regular(AUTH.IMPLEMENTATION_PATH,
                                                    "authorization verifier"))}


def _authorization_template(*, candidate: Mapping[str, Any], policy: Mapping[str, Any],
                            plan: Mapping[str, Any], plan_source: Mapping[str, Any],
                            executor: Mapping[str, Any], machine: Mapping[str, Any],
                            trust_root: Mapping[str, Any], repository: Mapping[str, Any],
                            binding: Mapping[str, Any], scope: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema": AUTH.AUTH_SCHEMA, "domain": AUTH.DOMAIN,
        "unsigned_field_names": [
            "schema", "schema_version", "status", "candidate", "policy", "plan",
            "executor", "trust_root", "repository", "binding", "deb_bindings",
            "deb_bindings_sha256", "tool", "scope", "machine_fingerprint",
            "machine_identity", "issued_at", "expires_at", "nonce", "network_used",
            "shell", "timeout_seconds", "resource_limits", "workflow", "phases"],
        "signature_field_names": ["algorithm", "key_id", "public_key_sha256",
                                  "payload_sha256", "signature_base64", "backend"],
        "bindings": {
            "candidate": dict(candidate), "policy": dict(policy),
            "plan": {"path": plan_source["path"], "file_sha256": plan_source["file_sha256"],
                     "canonical_sha256": plan["canonical_sha256"]},
            "executor": dict(executor), "trust_root": dict(trust_root),
            "repository": dict(repository),
            "binding_identity_sha256": binding["binding_identity_sha256"],
            "deb_bindings_sha256": canonical_hash(binding["debs"]),
            "tool": dict(plan["tool"]), "scope": dict(scope),
            "machine_identity": dict(machine),
            "machine_fingerprint": machine["file_sha256"],
            "workflow_sha256": canonical_hash(plan["workflow"]),
            "phases_sha256": canonical_hash(plan["phases"]),
        },
    }


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if isinstance(value, Mapping):
        value = {str(key): item for key, item in value.items()
                 if key != excluded}
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_relative(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or Path(value).as_posix() != value or \
            any(part in {"", ".", ".."} for part in value.split("/")):
        raise HandoffError(f"{label} is not a safe relative path")
    return Path(value)


def _checked_root(path: Path, label: str = "input root") -> Path:
    path = Path(path)
    if not path.is_absolute() or path.as_posix() != str(path) or \
            any(part in {"", ".", ".."} for part in path.parts[1:]):
        raise HandoffError(f"{label} must be absolute")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise HandoffError(f"{label} is unavailable: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise HandoffError(f"{label} contains a symlink")
    if not stat.S_ISDIR(path.lstat().st_mode):
        raise HandoffError(f"{label} is not a directory")
    return path


def _safe_absolute(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or \
            Path(value).as_posix() != value or any(part in {"", ".", ".."}
                                                    for part in value[1:].split("/")):
        raise HandoffError(f"{label} is not an absolute normalized path")
    return value


def _parent_no_symlink(path: Path, label: str) -> None:
    current = Path(path.anchor)
    for component in path.parent.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise HandoffError(f"{label} parent is unavailable") from error
        if stat.S_ISLNK(info.st_mode):
            raise HandoffError(f"{label} parent contains a symlink")


def _regular(path: Path, label: str, *, limit: int = MAX_JSON_BYTES) -> bytes:
    _parent_no_symlink(path, label)
    try:
        before = path.lstat()
    except OSError as error:
        raise HandoffError(f"{label} cannot be inspected") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size <= 0 or before.st_size > limit:
        raise HandoffError(f"{label} is not a bounded single-link regular file")
    try:
        with path.open("rb") as stream:
            first = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise HandoffError(f"{label} cannot be read") from error
    if first.st_ino != before.st_ino or first.st_dev != before.st_dev or \
            first.st_nlink != before.st_nlink or fd_after.st_ino != before.st_ino or \
            fd_after.st_dev != before.st_dev or fd_after.st_nlink != before.st_nlink or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or len(data) > limit:
        raise HandoffError(f"{label} changed while being read")
    return data


def _json(path: Path, label: str) -> tuple[dict[str, Any], bytes]:
    data = _regular(path, label)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise HandoffError(f"{label} is invalid JSON") from error
    if not isinstance(value, dict):
        raise HandoffError(f"{label} is not an object")
    return value, data


def _source(path: Path, label: str, *, expected_relative: str | None = None) -> dict[str, Any]:
    path = Path(path)
    data = _regular(path, label, limit=MAX_JSON_BYTES)
    try:
        relative = path.relative_to(ROOT).as_posix()
    except ValueError as error:
        raise HandoffError(f"{label} is outside the repository") from error
    if expected_relative is not None and relative != expected_relative:
        raise HandoffError(f"{label} is not the fixed repository input")
    result: dict[str, Any] = {"path": relative, "file_sha256": _sha(data),
                              "file_bytes": len(data)}
    if path.suffix == ".json":
        value, _ = _json(path, label)
        if "canonical_sha256" not in value or \
                value["canonical_sha256"] != canonical_hash(value, "canonical_sha256"):
            raise HandoffError(f"{label} canonical identity drift")
        result["canonical_sha256"] = value["canonical_sha256"]
        result["schema"] = value.get("schema", "")
    else:
        result["canonical_sha256"] = ""
    return result


def _input_source(path: Path, root: Path, label: str) -> dict[str, Any]:
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError as error:
        raise HandoffError(f"{label} escapes root_inputs") from error
    _safe_relative(relative, f"{label} path")
    value, data = _json(path, label)
    if value.get("canonical_sha256") != canonical_hash(value, "canonical_sha256"):
        raise HandoffError(f"{label} canonical identity drift")
    return {"path": relative, "file_sha256": _sha(data), "file_bytes": len(data),
            "canonical_sha256": value["canonical_sha256"], "schema": value.get("schema", "")}


def _machine_descriptor(path: Path, root: Path, campaign_id: str) -> dict[str, Any]:
    if not path.is_absolute():
        path = root / path
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError as error:
        raise HandoffError("machine identity artifact escapes root_inputs") from error
    _safe_relative(relative, "machine identity path")
    document, data = _json(path, "machine identity artifact")
    try:
        if document.get("schema") == MACHINE.KEYED_SCHEMA:
            validation = MACHINE.validate_keyed_file(path, live=False)
        elif document.get("schema") == MACHINE.SCHEMA:
            validation = MACHINE.validate_file(path, live=False)
        else:
            raise HandoffError("machine artifact schema is not a supported tagged method")
    except HandoffError:
        raise
    except Exception as error:
        raise HandoffError(f"machine artifact failed offline validation: {error}") from error
    if document.get("campaign_id") != campaign_id or document.get("status") != "LIVE_CAPTURED":
        raise HandoffError("machine artifact must be a live-capture status artifact")
    if validation.get("live_host_match") is not False:
        raise HandoffError("offline machine validation cannot claim live host match")
    file_sha256 = _sha(data)
    if validation.get("artifact_sha256") != file_sha256 or \
            validation.get("canonical_sha256") != document.get("canonical_sha256") or \
            validation.get("identity_sha256") != document.get("identity_sha256") or \
            validation.get("campaign_id") != document.get("campaign_id"):
        raise HandoffError("machine artifact changed between validation and descriptor read")
    if document.get("schema") == MACHINE.KEYED_SCHEMA:
        descriptor = AUTH._keyed_machine_descriptor(
            document, path, data, root, policy=False)
        descriptor["validation"] = "OFFLINE_ARTIFACT_ONLY"
        return descriptor
    return {"schema": document["schema"], "path": relative,
            "file_sha256": file_sha256,
            "canonical_sha256": document["canonical_sha256"],
            "campaign_domain": document["campaign_domain"],
            "campaign_id": document["campaign_id"],
            "identity_sha256": document["identity_sha256"],
            "validation": "OFFLINE_ARTIFACT_ONLY"}


def _fixed_sources() -> dict[str, dict[str, Any]]:
    candidate = _source(R3 / "apt_allowlist_candidate.json", "candidate manifest",
                        expected_relative="docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_allowlist_candidate.json")
    policy = _source(R3 / "apt_release_signature_authorization_policy.json", "authorization policy",
                     expected_relative="docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_release_signature_authorization_policy.json")
    policy_value, _ = _json(R3 / "apt_release_signature_authorization_policy.json", "authorization policy")
    if policy_value.get("schema") != AUTH.POLICY_SCHEMA or policy_value.get("status") != "NOT_READY" or \
            policy_value.get("authorized_keys") != [] or policy_value.get("machine_identity", {}).get("status") != "NOT_READY":
        raise HandoffError("fixed authorization policy schema drift")
    if policy_value.get("backend") != _backend_descriptor():
        raise HandoffError("fixed authorization backend source drift")
    candidate_value, _ = _json(R3 / "apt_allowlist_candidate.json", "candidate manifest")
    if candidate_value.get("status") != "OPT_IN_NOT_READY" or \
            candidate_value.get("benchmark_eligible") is not False:
        raise HandoffError("fixed candidate is not explicitly non-promoting")
    return {"candidate": candidate, "policy": policy,
            "executor": _source(EXECUTOR, "signature executor"),
            "authorization_verifier": _source(AUTH.IMPLEMENTATION_PATH,
                                                "authorization verifier")}


def _write_exclusive(path: Path, data: bytes) -> None:
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink():
        raise HandoffError("handoff output must be fresh and non-symlink")
    created: tuple[int, int] | None = None
    try:
        with path.open("xb") as stream:
            info = os.fstat(stream.fileno())
            created = (info.st_dev, info.st_ino)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        path.chmod(0o444)
    except OSError as error:
        if created is not None:
            try:
                current = path.lstat()
                if (current.st_dev, current.st_ino) == created:
                    path.unlink()
            except (FileNotFoundError, OSError):
                pass
        raise HandoffError("handoff output could not be sealed") from error


def _fresh(path: Path, label: str) -> None:
    _parent_no_symlink(path, label)
    try:
        path.lstat()
    except FileNotFoundError:
        return
    except OSError as error:
        raise HandoffError(f"{label} cannot be inspected") from error
    raise HandoffError(f"{label} must be fresh")


def _seal_pair(path: Path, data: bytes, sidecar: Path, sidecar_data: bytes) -> None:
    _fresh(path, "handoff request")
    _fresh(sidecar, "handoff sidecar")
    try:
        parent_before = path.parent.lstat()
    except OSError as error:
        raise HandoffError("handoff output parent cannot be inspected") from error
    created: list[tuple[Path, tuple[int, int]]] = []
    try:
        _write_exclusive(path, data)
        info = path.lstat()
        created.append((path, (info.st_dev, info.st_ino)))
        parent_now = path.parent.lstat()
        if (parent_now.st_dev, parent_now.st_ino) != (parent_before.st_dev, parent_before.st_ino):
            raise HandoffError("handoff output parent changed during seal")
        _write_exclusive(sidecar, sidecar_data)
        info = sidecar.lstat()
        created.append((sidecar, (info.st_dev, info.st_ino)))
        parent_now = path.parent.lstat()
        if (parent_now.st_dev, parent_now.st_ino) != (parent_before.st_dev, parent_before.st_ino):
            raise HandoffError("handoff output parent changed during seal")
    except Exception:
        for owned, identity in reversed(created):
            try:
                current = owned.lstat()
                if (current.st_dev, current.st_ino) == identity:
                    owned.unlink()
            except (FileNotFoundError, OSError):
                pass
        raise


def build_request(*, output: Path, plan_path: Path, machine_artifact: Path,
                  root_inputs: Path, repository: Mapping[str, Any],
                  deb_bindings: list[Mapping[str, Any]], campaign_id: str,
                  issued_at: int, expires_at: int, nonce: str) -> dict[str, Any]:
    root = _checked_root(Path(root_inputs))
    if not isinstance(campaign_id, str) or CAMPAIGN_RE.fullmatch(campaign_id) is None:
        raise HandoffError("campaign_id is invalid")
    if type(issued_at) is not int or type(expires_at) is not int or expires_at <= issued_at:
        raise HandoffError("requested validity window is invalid")
    if not isinstance(nonce, str) or NONCE_RE.fullmatch(nonce) is None:
        raise HandoffError("requested nonce is invalid")
    plan_path = Path(plan_path)
    if not plan_path.is_absolute():
        plan_path = root / plan_path
    plan, plan_data = _json(plan_path, "signature plan")
    plan_source = _input_source(plan_path, root, "signature plan")
    if plan.get("schema") != SIG.PLAN_SCHEMA or \
            plan.get("canonical_sha256") != canonical_hash(plan, "canonical_sha256"):
        raise HandoffError("signature plan schema/canonical drift")
    try:
        PLAN.validate_plan(plan, root=root, repository=repository, deb_bindings=deb_bindings)
    except Exception as error:
        raise HandoffError(f"signature plan full validation failed: {error}") from error
    trust_path = root / _safe_relative(plan["trust_root"]["path"], "trust-root path")
    trust = SIG.validate_trust_root(root, trust_path)
    signature_path = (root / _safe_relative(plan["scope"]["signature_path"], "signature path")
                      if plan["scope"].get("signature_path") else None)
    binding = SIG._binding(
        root, root / _safe_relative(plan["scope"]["release_path"], "Release path"),
        root / _safe_relative(plan["scope"]["packages_path"], "Packages path"),
        repository, deb_bindings, release_kind=plan["scope"]["release_kind"],
        signature_path=signature_path)
    machine = _machine_descriptor(Path(machine_artifact), root, campaign_id)
    fixed = _fixed_sources()
    trust_source = _input_source(trust_path, root, "trust root")
    trust_descriptor = {**trust_source, "canonical_sha256": trust["canonical_sha256"],
                        "key_id": trust["key_id"], "fingerprint": trust["fingerprint"]}
    scope = {"root_inputs": str(root), "homedir": plan["scope"]["homedir"],
             "output_root": plan["scope"]["output_root"]}
    request: dict[str, Any] = {
        "schema": SCHEMA, "schema_version": 1,
        "status": "UNSIGNED_REVIEW_REQUIRED", "benchmark_eligible": False,
        "domain": DOMAIN, "campaign_id": campaign_id,
        "candidate": fixed["candidate"], "policy": fixed["policy"],
        "executor": fixed["executor"],
        "authorization_verifier": fixed["authorization_verifier"],
        "authorization_backend": _backend_descriptor(),
        "machine_identity": machine,
        "plan": {**plan_source, "canonical_sha256": plan["canonical_sha256"]},
        "trust_root": trust_descriptor,
        "repository": dict(repository), "binding": binding,
        "deb_bindings": binding["debs"],
        "deb_bindings_sha256": canonical_hash(binding["debs"]),
        "scope": scope,
        "safety": {"network_used": False, "shell": False, "timeout_seconds": 30,
                   "resource_limits": {"RLIMIT_CORE": 0, "RLIMIT_FSIZE": 262144},
                   "machine_validation": "OFFLINE_ARTIFACT_ONLY"},
        "requested_validity": {"issued_at": issued_at, "expires_at": expires_at},
        "nonce": nonce,
        "authorization_template": _authorization_template(
            candidate=fixed["candidate"], policy=fixed["policy"], plan=plan,
            plan_source=plan_source, executor=fixed["executor"], machine=machine,
            trust_root=trust_descriptor, repository=repository, binding=binding,
            scope=scope),
        "custodian": {"status": "UNSIGNED_REVIEW_REQUIRED", "algorithm": "Ed25519",
                       "key_id": "", "public_key_base64": "", "signature_base64": ""},
        "canonical_sha256": "",
    }
    request["canonical_sha256"] = canonical_hash(request, "canonical_sha256")
    output = Path(output)
    if not output.is_absolute() or output.as_posix() != str(output):
        raise HandoffError("handoff output must be an absolute normalized path")
    _parent_no_symlink(output, "handoff output")
    data = canonical_bytes(request) + b"\n"
    sidecar = {"schema": SIDECAR_SCHEMA, "schema_version": 1,
               "artifact_path": output.name, "artifact_bytes": len(data),
               "artifact_sha256": _sha(data),
               "canonical_sha256": request["canonical_sha256"]}
    sidecar_path = output.with_name(output.name + ".sha256.json")
    _seal_pair(output, data, sidecar_path, canonical_bytes(sidecar) + b"\n")
    validate_request(output)
    del plan_data
    return {"status": "PASS", "request_path": str(output),
            "request_sha256": _sha(data), "canonical_sha256": request["canonical_sha256"],
            "benchmark_eligible": False}


def validate_request(path: Path) -> dict[str, Any]:
    path = Path(path)
    sidecar_path = path.with_name(path.name + ".sha256.json")
    for sealed, label in ((path, "handoff request"), (sidecar_path, "handoff sidecar")):
        try:
            info = sealed.lstat()
        except OSError as error:
            raise HandoffError(f"{label} cannot be inspected") from error
        if info.st_mode & 0o222 or stat.S_ISLNK(info.st_mode) or info.st_nlink != 1:
            raise HandoffError(f"{label} is not immutable single-link evidence")
    request, data = _json(Path(path), "custodian handoff request")
    sidecar, sidecar_data = _json(sidecar_path, "handoff sidecar")
    if set(sidecar) != {"schema", "schema_version", "artifact_path", "artifact_bytes",
                         "artifact_sha256", "canonical_sha256"} or \
            sidecar["schema"] != SIDECAR_SCHEMA or sidecar["schema_version"] != 1 or \
            sidecar["artifact_path"] != path.name or sidecar["artifact_bytes"] != len(data) or \
            sidecar["artifact_sha256"] != _sha(data) or sidecar["canonical_sha256"] != request.get("canonical_sha256"):
        raise HandoffError("handoff sidecar binding drift")
    del sidecar_data
    required = {"schema", "schema_version", "status", "benchmark_eligible", "domain",
                "campaign_id", "candidate", "policy", "executor", "authorization_verifier",
                "authorization_backend", "machine_identity", "plan", "trust_root",
                "repository", "binding", "deb_bindings", "deb_bindings_sha256", "scope",
                "safety", "requested_validity", "nonce", "authorization_template",
                "custodian", "canonical_sha256"}
    if set(request) != required or request["schema"] != SCHEMA or request["schema_version"] != 1 or \
            request["status"] != "UNSIGNED_REVIEW_REQUIRED" or request["benchmark_eligible"] is not False or \
            request["domain"] != DOMAIN or request["canonical_sha256"] != canonical_hash(request, "canonical_sha256"):
        raise HandoffError("handoff schema/status/canonical identity is invalid")
    if not isinstance(request["custodian"], Mapping) or \
            request["custodian"] != {"status": "UNSIGNED_REVIEW_REQUIRED", "algorithm": "Ed25519",
                                     "key_id": "", "public_key_base64": "", "signature_base64": ""}:
        raise HandoffError("handoff must not contain a caller or custodian key/signature")
    machine = request["machine_identity"]
    if not isinstance(machine, Mapping) or machine.get("validation") != "OFFLINE_ARTIFACT_ONLY" or \
            machine.get("campaign_id") != request["campaign_id"]:
        raise HandoffError("handoff machine identity is not an offline campaign-bound artifact")
    if request["safety"].get("network_used") is not False or request["safety"].get("shell") is not False:
        raise HandoffError("handoff safety contract is not offline")
    fixed = _fixed_sources()
    for key in ("candidate", "policy", "executor", "authorization_verifier"):
        if request[key] != fixed[key]:
            raise HandoffError(f"handoff {key} source identity drift")
    if request["authorization_backend"] != _backend_descriptor():
        raise HandoffError("handoff authorization backend drift")
    scope = request["scope"]
    for key in ("root_inputs", "homedir", "output_root"):
        _safe_absolute(scope[key], f"handoff scope {key}")
    root = _checked_root(Path(scope["root_inputs"]), "handoff root_inputs")
    machine_path = root / _safe_relative(machine["path"], "machine identity path")
    if _machine_descriptor(machine_path, root, request["campaign_id"]) != dict(machine):
        raise HandoffError("handoff machine artifact was replaced or drifted")
    plan_path = root / _safe_relative(request["plan"]["path"], "signature plan path")
    if _input_source(plan_path, root, "signature plan") != dict(request["plan"]):
        raise HandoffError("handoff signature plan was replaced or drifted")
    plan, _ = _json(plan_path, "signature plan")
    raw_debs = [{key: item[key] for key in ("name", "version", "architecture", "path")}
                for item in request["deb_bindings"]]
    try:
        PLAN.validate_plan(plan, root=root, repository=request["repository"],
                           deb_bindings=raw_debs)
    except Exception as error:
        raise HandoffError(f"handoff signature plan full validation failed: {error}") from error
    if request["scope"] != {"root_inputs": str(root),
                             "homedir": plan["scope"]["homedir"],
                             "output_root": plan["scope"]["output_root"]}:
        raise HandoffError("handoff scope is not exactly plan-bound")
    if set(request["safety"]) != {"network_used", "shell", "timeout_seconds",
                                   "resource_limits", "machine_validation"} or \
            request["safety"] != {"network_used": False, "shell": False,
                                   "timeout_seconds": 30,
                                   "resource_limits": {"RLIMIT_CORE": 0, "RLIMIT_FSIZE": 262144},
                                   "machine_validation": "OFFLINE_ARTIFACT_ONLY"}:
        raise HandoffError("handoff safety contract drift")
    validity = request["requested_validity"]
    if set(validity) != {"issued_at", "expires_at"} or type(validity["issued_at"]) is not int or \
            type(validity["expires_at"]) is not int or validity["expires_at"] <= validity["issued_at"] or \
            validity["expires_at"] - validity["issued_at"] > 7 * 24 * 60 * 60:
        raise HandoffError("handoff validity window is invalid")
    if not isinstance(request["nonce"], str) or NONCE_RE.fullmatch(request["nonce"]) is None:
        raise HandoffError("handoff nonce is invalid")
    trust_path = root / _safe_relative(request["trust_root"]["path"], "trust root path")
    trust = SIG.validate_trust_root(root, trust_path)
    trust_source = _input_source(trust_path, root, "trust root")
    expected_trust = {**trust_source, "canonical_sha256": trust["canonical_sha256"],
                      "key_id": trust["key_id"], "fingerprint": trust["fingerprint"]}
    if expected_trust != dict(request["trust_root"]):
        raise HandoffError("handoff trust root was replaced or drifted")
    signature_path = (root / _safe_relative(plan["scope"]["signature_path"], "signature path")
                      if plan["scope"].get("signature_path") else None)
    actual_binding = SIG._binding(
        root, root / _safe_relative(plan["scope"]["release_path"], "Release path"),
        root / _safe_relative(plan["scope"]["packages_path"], "Packages path"),
        request["repository"], raw_debs, release_kind=plan["scope"]["release_kind"],
        signature_path=signature_path)
    if actual_binding != request["binding"] or \
            actual_binding["debs"] != request["deb_bindings"] or \
            request["deb_bindings_sha256"] != canonical_hash(actual_binding["debs"]):
        raise HandoffError("handoff Release/Packages/deb binding drift")
    expected_template = _authorization_template(
        candidate=request["candidate"], policy=request["policy"], plan=plan,
        plan_source=request["plan"], executor=request["executor"],
        machine=request["machine_identity"], trust_root=request["trust_root"],
        repository=request["repository"], binding=actual_binding, scope=request["scope"])
    if request["authorization_template"] != expected_template:
        raise HandoffError("handoff authorization body template drift")
    return {"status": "PASS", "request_sha256": _sha(_regular(Path(path), "handoff request")),
            "canonical_sha256": request["canonical_sha256"],
            "benchmark_eligible": False, "offline_host_match_claimed": False,
            "custodian_status": request["custodian"]["status"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prepare = sub.add_parser("prepare-request")
    prepare.add_argument("--output", type=Path, required=True)
    prepare.add_argument("--plan", type=Path, required=True)
    prepare.add_argument("--machine-artifact", type=Path, required=True)
    prepare.add_argument("--root-inputs", type=Path, required=True)
    prepare.add_argument("--repository-file", type=Path, required=True)
    prepare.add_argument("--deb-bindings-file", type=Path, required=True)
    prepare.add_argument("--campaign-id", required=True)
    prepare.add_argument("--issued-at", type=int, required=True)
    prepare.add_argument("--expires-at", type=int, required=True)
    prepare.add_argument("--nonce", required=True)
    validate = sub.add_parser("validate-request")
    validate.add_argument("--request", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "prepare-request":
            repository, _ = _json(args.repository_file, "repository input")
            deb_bytes = _regular(args.deb_bindings_file, "deb binding input")
            try:
                deb_bindings = json.loads(deb_bytes.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError) as error:
                raise HandoffError("deb binding input is invalid JSON") from error
            if not isinstance(deb_bindings, list):
                raise HandoffError("deb binding input must be an array")
            result = build_request(
                output=args.output, plan_path=args.plan,
                machine_artifact=args.machine_artifact, root_inputs=args.root_inputs,
                repository=repository, deb_bindings=deb_bindings,
                campaign_id=args.campaign_id, issued_at=args.issued_at,
                expires_at=args.expires_at, nonce=args.nonce)
        else:
            result = validate_request(args.request)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (HandoffError, OSError, SIG.SignatureError) as error:
        print(f"error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
