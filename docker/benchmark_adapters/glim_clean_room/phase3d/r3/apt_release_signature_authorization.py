#!/usr/bin/env python3
"""Verify the independent Ed25519 authorization for an r3 host run.

The trust policy and candidate manifest are fixed repository inputs.  Callers
cannot select an arbitrary trust store.  The checked-in policy is deliberately
``NOT_READY`` with no authorized key, so this module cannot authorize a real
GnuPG process in the current candidate.  Tests may replace the module-level
paths with a temporary synthetic policy; that seam is not exposed by the
executor CLI and never makes a fixture receipt promotion-eligible.
"""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import time
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[5]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
CANDIDATE_MANIFEST_PATH = R3 / "apt_allowlist_candidate.json"
POLICY_PATH = R3 / "apt_release_signature_authorization_policy.json"
POLICY_SCHEMA_PATH = R3 / "apt_release_signature_authorization_policy.schema.json"
IMPLEMENTATION_PATH = Path(__file__)
IMPLEMENTATION_RELATIVE = IMPLEMENTATION_PATH.relative_to(ROOT).as_posix()
AUTH_SCHEMA = "glim_clean_room_r3_apt_signature_authorization_v1"
POLICY_SCHEMA = "glim_clean_room_r3_apt_signature_authorization_policy_v1"
DOMAIN = "lidarslam/glim-clean-room-r3/apt-signature-host-authorization/ed25519/v1"
ALGORITHM = "Ed25519"
BACKEND = {
    "name": "python-cryptography",
    "implementation": (
        "cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify"),
    "implementation_file": IMPLEMENTATION_RELATIVE,
}
TIMEOUT_SECONDS = 30
RESOURCE_LIMITS = {"RLIMIT_CORE": 0, "RLIMIT_FSIZE": 262144}
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
KEY_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
NONCE_RE = re.compile(r"^[A-Za-z0-9_-]{32,128}$")
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_KEYRING_BYTES = 32 * 1024 * 1024
MAX_TOOL_BYTES = 32 * 1024 * 1024


class AuthorizationError(ValueError):
    """Raised when host authorization cannot be independently verified."""


def _load_signature_contract():
    path = Path(__file__).with_name("apt_release_signature.py")
    spec = importlib.util.spec_from_file_location("r3_authorization_signature_contract", path)
    if spec is None or spec.loader is None:
        raise AuthorizationError("signature contract cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIG = _load_signature_contract()


def _load_machine_contract():
    path = Path(__file__).with_name("apt_machine_identity.py")
    spec = importlib.util.spec_from_file_location("r3_authorization_machine_contract", path)
    if spec is None or spec.loader is None:
        raise AuthorizationError("machine identity contract cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MACHINE = _load_machine_contract()
MACHINE_SCHEMA = MACHINE.SCHEMA
LEGACY_MACHINE_METHOD = "LEGACY_V1"
KEYED_MACHINE_METHOD = MACHINE.KEY_METHOD
KEYED_MACHINE_SCHEMA = MACHINE.KEYED_SCHEMA
KEYED_MACHINE_DOMAIN = MACHINE.KEY_DOMAIN


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if excluded is not None and isinstance(value, Mapping):
        value = {str(key): item for key, item in value.items() if key != excluded}
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def authorization_payload(auth: Mapping[str, Any]) -> bytes:
    if not isinstance(auth, Mapping):
        raise AuthorizationError("authorization is not an object")
    unsigned = {str(key): value for key, value in auth.items() if key != "signature"}
    signature = auth.get("signature")
    if not isinstance(signature, Mapping):
        raise AuthorizationError("authorization signature metadata is missing")
    # The detached signature bytes and their hash are the only circular fields.
    # Algorithm/key identity/backend metadata remain inside the signed payload.
    unsigned["signature"] = {
        str(key): value for key, value in signature.items()
        if key not in {"signature_base64", "payload_sha256"}}
    return canonical_bytes({"domain": DOMAIN, "schema_version": 1,
                            "authorization": unsigned})


def authorization_payload_sha256(auth: Mapping[str, Any]) -> str:
    return hashlib.sha256(authorization_payload(auth)).hexdigest()


def _regular(path: Path, label: str, *, limit: int = MAX_JSON_BYTES,
             allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise AuthorizationError(f"{label} cannot be inspected: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > limit or \
            (before.st_size == 0 and not allow_empty):
        raise AuthorizationError(f"{label} is not a bounded regular single-link file")
    try:
        with path.open("rb") as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise AuthorizationError(f"{label} cannot be read: {error}") from error
    if (fd_before.st_dev, fd_before.st_ino) != (before.st_dev, before.st_ino) or \
            (fd_after.st_dev, fd_after.st_ino) != (before.st_dev, before.st_ino) or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > limit or (not data and not allow_empty):
        raise AuthorizationError(f"{label} changed while being read")
    return data


def _safe_relative(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            Path(value).as_posix() != value or any(
                part in {"", ".", ".."} for part in value.split("/")):
        raise AuthorizationError(f"{label} is not a safe relative path")
    return Path(value)


def _path_no_symlink(path: Path, label: str) -> None:
    current = path if path.is_absolute() else Path.cwd() / path
    components = current.parts
    probe = Path(components[0])
    for component in components[1:]:
        probe /= component
        try:
            info = probe.lstat()
        except OSError as error:
            raise AuthorizationError(f"{label} path component is unavailable: {error}") from error
        if stat.S_ISLNK(info.st_mode):
            raise AuthorizationError(f"{label} contains a symlink component")


def _sha_file(path: Path, label: str, *, limit: int = MAX_JSON_BYTES) -> tuple[str, bytes]:
    _path_no_symlink(path, label)
    data = _regular(path, label, limit=limit)
    return hashlib.sha256(data).hexdigest(), data


def _read_json(path: Path, label: str) -> tuple[dict[str, Any], bytes]:
    digest, data = _sha_file(path, label)
    del digest
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise AuthorizationError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise AuthorizationError(f"{label} is not an object")
    return value, data


def _sha(value: Any, label: str) -> None:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise AuthorizationError(f"{label} must be a lowercase SHA-256")


def _validate_keyed_machine_identity(value: Any, label: str, *, policy: bool) -> None:
    """Validate the keyed branch of the machine-identity tagged union.

    The policy branch carries ``status=PRECOMMITTED``.  The signed
    authorization/receipt branch intentionally omits that policy-only field,
    but keeps every artifact/source projection that can identify the keyed
    proof chain.  Keeping the branches exact prevents a legacy artifact from
    being relabeled as keyed (or vice versa) by merely rehashing JSON.
    """
    common = {
        "method", "schema", "path", "file_sha256", "canonical_sha256",
        "campaign_domain", "campaign_id", "identity_sha256",
        "host_public_key", "challenge", "proof",
    }
    required = common | ({"status"} if policy else set())
    if not isinstance(value, Mapping) or set(value) != required:
        raise AuthorizationError(f"{label} keyed fields are incomplete or extra")
    if policy and value.get("status") != "PRECOMMITTED":
        raise AuthorizationError(f"{label} keyed status is not PRECOMMITTED")
    if value.get("method") != KEYED_MACHINE_METHOD or \
            value.get("schema") != KEYED_MACHINE_SCHEMA or \
            value.get("campaign_domain") != KEYED_MACHINE_DOMAIN or \
            MACHINE.CAMPAIGN_RE.fullmatch(value.get("campaign_id", "")) is None:
        raise AuthorizationError(f"{label} keyed method/schema/domain drift")
    _safe_relative(value.get("path"), f"{label} keyed path")
    for field in ("file_sha256", "canonical_sha256", "identity_sha256"):
        _sha(value.get(field), f"{label}.{field}")
    key = value["host_public_key"]
    key_fields = {"schema", "path", "file_sha256", "file_bytes", "canonical_sha256",
                  "campaign_id", "key_id", "algorithm", "public_key_sha256",
                  "provisioning_receipt_sha256"}
    if not isinstance(key, Mapping) or set(key) != key_fields or \
            key.get("schema") != MACHINE.KEY_DESCRIPTOR_SCHEMA or \
            key.get("campaign_id") != value["campaign_id"] or \
            key.get("algorithm") != MACHINE.KEY_ALGORITHM:
        raise AuthorizationError(f"{label} keyed public-key projection drift")
    _safe_relative(key.get("path"), f"{label} keyed public-key path")
    for field in ("file_sha256", "canonical_sha256", "public_key_sha256",
                  "provisioning_receipt_sha256"):
        _sha(key.get(field), f"{label}.host_public_key.{field}")
    if type(key.get("file_bytes")) is not int or key["file_bytes"] <= 0 or \
            key["file_bytes"] > MACHINE.MAX_KEY_INPUT_BYTES:
        raise AuthorizationError(f"{label} keyed public-key size is invalid")
    challenge = value["challenge"]
    challenge_fields = {"schema", "path", "file_sha256", "file_bytes", "canonical_sha256",
                        "challenge_payload_sha256"}
    if not isinstance(challenge, Mapping) or set(challenge) != challenge_fields or \
            challenge.get("schema") != MACHINE.KEY_CHALLENGE_SCHEMA:
        raise AuthorizationError(f"{label} keyed challenge projection drift")
    _safe_relative(challenge.get("path"), f"{label} keyed challenge path")
    for field in ("file_sha256", "canonical_sha256", "challenge_payload_sha256"):
        _sha(challenge.get(field), f"{label}.challenge.{field}")
    if type(challenge.get("file_bytes")) is not int or challenge["file_bytes"] <= 0 or \
            challenge["file_bytes"] > MACHINE.MAX_KEY_INPUT_BYTES:
        raise AuthorizationError(f"{label} keyed challenge size is invalid")
    proof = value["proof"]
    proof_fields = {"schema", "path", "file_sha256", "file_bytes", "canonical_sha256",
                    "challenge_payload_sha256", "signature_sha256"}
    if not isinstance(proof, Mapping) or set(proof) != proof_fields or \
            proof.get("schema") != MACHINE.KEY_PROOF_SCHEMA:
        raise AuthorizationError(f"{label} keyed proof projection drift")
    _safe_relative(proof.get("path"), f"{label} keyed proof path")
    for field in ("file_sha256", "canonical_sha256", "challenge_payload_sha256",
                  "signature_sha256"):
        _sha(proof.get(field), f"{label}.proof.{field}")
    if type(proof.get("file_bytes")) is not int or proof["file_bytes"] <= 0 or \
            proof["file_bytes"] > MACHINE.MAX_KEY_INPUT_BYTES:
        raise AuthorizationError(f"{label} keyed proof size is invalid")


def _keyed_machine_descriptor(document: Mapping[str, Any], path: Path, data: bytes,
                               root: Path, *, policy: bool) -> dict[str, Any]:
    """Project a validated keyed artifact into policy/auth binding fields."""
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError as error:
        raise AuthorizationError("keyed machine artifact escapes input root") from error
    descriptor: dict[str, Any] = {
        "method": document["method"], "schema": document["schema"],
        "path": relative, "file_sha256": hashlib.sha256(data).hexdigest(),
        "canonical_sha256": document["canonical_sha256"],
        "campaign_domain": document["campaign_domain"],
        "campaign_id": document["campaign_id"],
        "identity_sha256": document["identity_sha256"],
        "host_public_key": dict(document["host_public_key"]),
        "challenge": dict(document["challenge"]),
        "proof": dict(document["proof"]),
    }
    if policy:
        descriptor = {"status": "PRECOMMITTED", **descriptor}
    _validate_keyed_machine_identity(descriptor, "keyed machine artifact", policy=policy)
    return descriptor


def _decode(value: Any, label: str, expected: int) -> bytes:
    if not isinstance(value, str) or not value:
        raise AuthorizationError(f"{label} is missing")
    try:
        decoded = base64.b64decode(value.encode("ascii"), validate=True)
    except (UnicodeError, ValueError) as error:
        raise AuthorizationError(f"{label} is not strict base64") from error
    if len(decoded) != expected:
        raise AuthorizationError(f"{label} has the wrong byte length")
    return decoded


def _candidate_identity() -> tuple[dict[str, Any], str, bytes]:
    manifest, data = _read_json(CANDIDATE_MANIFEST_PATH, "candidate manifest")
    canonical = manifest.get("canonical_sha256")
    if not isinstance(canonical, str) or canonical != canonical_hash(manifest, "canonical_sha256"):
        raise AuthorizationError("candidate manifest canonical identity drift")
    file_sha = hashlib.sha256(data).hexdigest()
    policy = manifest.get("authorization_policy")
    if not isinstance(policy, Mapping):
        raise AuthorizationError("candidate manifest has no authorization policy binding")
    return manifest, file_sha, data


def _load_fixed_policy() -> tuple[dict[str, Any], dict[str, Any], str, str]:
    """Load only the policy path pinned by the repository candidate manifest."""
    manifest, candidate_file_sha, _ = _candidate_identity()
    policy_binding = manifest["authorization_policy"]
    required_binding = {"path", "file_sha256", "canonical_sha256", "schema",
                        "status", "authorized_runtime"}
    if set(policy_binding) != required_binding:
        raise AuthorizationError("candidate authorization policy binding is incomplete")
    if policy_binding["schema"] != POLICY_SCHEMA or \
            policy_binding["status"] not in {"NOT_READY", "READY"} or \
            policy_binding["authorized_runtime"] is not (policy_binding["status"] == "READY"):
        raise AuthorizationError("candidate authorization policy status binding is invalid")
    relative = _safe_relative(policy_binding["path"], "candidate authorization policy path")
    expected_path = ROOT / relative
    if expected_path != POLICY_PATH:
        raise AuthorizationError("candidate authorization policy path is not fixed")
    policy, policy_data = _read_json(expected_path, "authorization policy")
    actual_file_sha = hashlib.sha256(policy_data).hexdigest()
    if actual_file_sha != policy_binding["file_sha256"]:
        raise AuthorizationError("authorization policy file identity drift")
    if policy_binding["schema"] != POLICY_SCHEMA or \
            policy.get("schema") != POLICY_SCHEMA or \
            policy.get("status") != policy_binding["status"] or \
            policy.get("canonical_sha256") != canonical_hash(policy, "canonical_sha256") or \
            policy.get("canonical_sha256") != policy_binding["canonical_sha256"]:
        raise AuthorizationError("authorization policy canonical/schema drift")
    candidate = policy.get("candidate_manifest")
    if not isinstance(candidate, Mapping) or set(candidate) != {
            "path", "candidate_id", "revision"}:
        raise AuthorizationError("authorization policy candidate binding is incomplete")
    expected_candidate_path = Path(CANDIDATE_MANIFEST_PATH).relative_to(ROOT).as_posix()
    if candidate["path"] != expected_candidate_path:
        raise AuthorizationError("authorization policy candidate path drift")
    if candidate.get("candidate_id") != manifest.get("candidate_id") or \
            candidate.get("revision") != manifest.get("revision"):
        raise AuthorizationError("authorization policy candidate identity drift")
    return policy, manifest, candidate_file_sha, actual_file_sha


def _validate_policy(policy: Mapping[str, Any], now: int) -> dict[str, Any]:
    required = {"schema", "schema_version", "status", "candidate_manifest", "backend",
                "machine_identity", "authorized_keys", "canonical_sha256"}
    if set(policy) != required or policy.get("schema") != POLICY_SCHEMA or \
            policy.get("schema_version") != 1 or \
            policy.get("canonical_sha256") != canonical_hash(policy, "canonical_sha256"):
        raise AuthorizationError("authorization policy shape or canonical identity is invalid")
    backend = policy.get("backend")
    if not isinstance(backend, Mapping) or set(backend) != {
            "name", "version", "implementation", "implementation_file",
            "implementation_sha256"} or \
            backend.get("name") != BACKEND["name"] or \
            backend.get("implementation") != BACKEND["implementation"] or \
            backend.get("implementation_file") != IMPLEMENTATION_RELATIVE or \
            not isinstance(backend.get("version"), str) or not backend["version"]:
        raise AuthorizationError("authorization backend contract is invalid")
    implementation_sha = hashlib.sha256(_regular(IMPLEMENTATION_PATH,
                                                   "authorization implementation",
                                                   limit=MAX_TOOL_BYTES)).hexdigest()
    if backend.get("implementation_sha256") != implementation_sha:
        raise AuthorizationError("authorization implementation hash drift")
    machine_identity = policy.get("machine_identity")
    if policy["status"] == "NOT_READY":
        if machine_identity != {"status": "NOT_READY", "schema": "", "path": "",
                                "file_sha256": "", "canonical_sha256": "",
                                "campaign_domain": "", "campaign_id": "",
                                "identity_sha256": ""}:
            raise AuthorizationError("NOT_READY machine identity policy is not empty")
        machine_method = LEGACY_MACHINE_METHOD
    else:
        if not isinstance(machine_identity, Mapping) or \
                machine_identity.get("status") != "PRECOMMITTED":
            raise AuthorizationError("precommitted machine identity policy is malformed")
        if machine_identity.get("schema") == MACHINE_SCHEMA:
            legacy_fields = {"status", "schema", "path", "file_sha256", "canonical_sha256",
                             "campaign_domain", "campaign_id", "identity_sha256"}
            if set(machine_identity) != legacy_fields or \
                    machine_identity["campaign_domain"] != MACHINE.DOMAIN or \
                    MACHINE.CAMPAIGN_RE.fullmatch(machine_identity["campaign_id"]) is None:
                raise AuthorizationError("precommitted legacy machine identity is malformed")
            _safe_relative(machine_identity["path"], "machine identity path")
            for field in ("file_sha256", "canonical_sha256", "identity_sha256"):
                _sha(machine_identity[field], f"machine identity {field}")
            machine_method = LEGACY_MACHINE_METHOD
        elif machine_identity.get("schema") == KEYED_MACHINE_SCHEMA:
            _validate_keyed_machine_identity(machine_identity,
                                              "precommitted keyed machine identity",
                                              policy=True)
            machine_method = KEYED_MACHINE_METHOD
        else:
            raise AuthorizationError("precommitted machine identity schema is unknown")
    if policy.get("status") != "READY":
        raise AuthorizationError("authorization policy is NOT_READY")
    keys = policy.get("authorized_keys")
    if not isinstance(keys, list) or not keys:
        raise AuthorizationError("authorization policy has no authorized key")
    seen: set[str] = set()
    valid_keys: dict[str, dict[str, Any]] = {}
    for index, raw in enumerate(keys):
        label = f"authorized_keys[{index}]"
        if not isinstance(raw, Mapping) or set(raw) != {
                "key_id", "algorithm", "status", "public_key_base64",
                "public_key_sha256", "not_before", "not_after"}:
            raise AuthorizationError(f"{label} is malformed")
        key_id = raw.get("key_id")
        if not isinstance(key_id, str) or KEY_ID_RE.fullmatch(key_id) is None or key_id in seen:
            raise AuthorizationError(f"{label}.key_id is missing or duplicated")
        seen.add(key_id)
        if raw.get("algorithm") != ALGORITHM or raw.get("status") != "ACTIVE":
            raise AuthorizationError(f"{label} is not active Ed25519")
        if type(raw.get("not_before")) is not int or type(raw.get("not_after")) is not int or \
                raw["not_after"] <= raw["not_before"] or \
                now < raw["not_before"] or now >= raw["not_after"]:
            raise AuthorizationError(f"{label} is expired, revoked, or not yet valid")
        public_key = _decode(raw.get("public_key_base64"), f"{label}.public_key_base64", 32)
        _sha(raw.get("public_key_sha256"), f"{label}.public_key_sha256")
        if hashlib.sha256(public_key).hexdigest() != raw["public_key_sha256"]:
            raise AuthorizationError(f"{label} public-key hash drift")
        valid_keys[key_id] = {**dict(raw), "public_key": public_key}
    if list(seen) != sorted(seen):
        raise AuthorizationError("authorization keys are not sorted")
    return {"backend": dict(backend), "keys": valid_keys,
            "machine_identity": dict(machine_identity), "machine_method": machine_method}


def _expected_context(*, plan: Mapping[str, Any], root_inputs: Path,
                      repository: Mapping[str, Any], deb_bindings: list[Mapping[str, Any]],
                      executor_source_sha256: str) -> dict[str, Any]:
    root = SIG._checked_root(Path(root_inputs))
    scope = plan.get("scope")
    trust_projection = plan.get("trust_root")
    tool = plan.get("tool")
    if not isinstance(scope, Mapping) or not isinstance(trust_projection, Mapping) or \
            not isinstance(tool, Mapping):
        raise AuthorizationError("plan scope/trust/tool projection is incomplete")
    trust_path = root / SIG._safe_relative(plan["trust_root"]["path"], "trust-root path")
    trust = SIG.validate_trust_root(root, trust_path)
    trust_data = SIG._regular(trust_path, "trust-root manifest", limit=SIG.MAX_JSON_BYTES)
    release = root / SIG._safe_relative(scope["release_path"], "Release path")
    packages = root / SIG._safe_relative(scope["packages_path"], "Packages path")
    signature = root / SIG._safe_relative(scope["signature_path"], "signature path") \
        if scope["signature_path"] else None
    binding = SIG._binding(root, release, packages, repository, deb_bindings,
                           release_kind=scope["release_kind"], signature_path=signature)
    executable = trust["verifier"]["binary_path"]
    executable_path = Path(executable)
    _path_no_symlink(executable_path, "GnuPG executable")
    executable_data = _regular(executable_path, "GnuPG executable", limit=MAX_KEYRING_BYTES)
    executable_info = executable_path.lstat()
    if not executable_info.st_mode & stat.S_IXUSR:
        raise AuthorizationError("GnuPG executable is not executable")
    expected_workflow = plan.get("workflow")
    expected_phases = plan.get("phases")
    if not isinstance(expected_workflow, list) or not isinstance(expected_phases, list):
        raise AuthorizationError("plan workflow/phases are missing")
    return {
        "root": root,
        "trust_path": trust_path,
        "trust": trust,
        "trust_data": trust_data,
        "release": release,
        "packages": packages,
        "signature": signature,
        "binding": binding,
        "executable": executable,
        "executable_sha256": hashlib.sha256(executable_data).hexdigest(),
        "tool": tool,
        "workflow": expected_workflow,
        "phases": expected_phases,
        "plan_sha256": hashlib.sha256(canonical_bytes(plan)).hexdigest(),
        "plan_canonical_sha256": plan.get("canonical_sha256"),
        "executor_source_sha256": executor_source_sha256,
        "deb_bindings": deb_bindings,
        "repository": repository,
        "scope": scope,
    }


def verify_authorization(auth: Mapping[str, Any], *, plan: Mapping[str, Any],
                         root_inputs: Path, repository: Mapping[str, Any],
                         deb_bindings: list[Mapping[str, Any]],
                         executor_source_sha256: str,
                         now: int | None = None,
                         machine_provider: Any = None,
                         require_live_machine: bool = True,
                         _fixed_inputs: tuple[Mapping[str, Any], Mapping[str, Any], str, str] | None = None,
                         _candidate_path: Path | None = None,
                         _policy_path: Path | None = None) -> dict[str, Any]:
    """Verify a signed, exact host authorization before any output is created."""
    if now is None:
        now = int(time.time())
    if type(now) is not int or now < 0:
        raise AuthorizationError("authorization verification time is invalid")
    if _fixed_inputs is None:
        policy, manifest, candidate_file_sha, policy_file_sha = _load_fixed_policy()
        candidate_path = CANDIDATE_MANIFEST_PATH
        policy_path = POLICY_PATH
    else:
        if len(_fixed_inputs) != 4:
            raise AuthorizationError("response policy input tuple is incomplete")
        policy, manifest, candidate_file_sha, policy_file_sha = _fixed_inputs
        candidate_path = _candidate_path
        policy_path = _policy_path
        if not isinstance(candidate_path, Path) or not isinstance(policy_path, Path):
            raise AuthorizationError("response policy source paths are missing")
        if not candidate_path.is_absolute() or not policy_path.is_absolute():
            raise AuthorizationError("response policy source paths are not absolute")
    if not isinstance(policy, Mapping) or not isinstance(manifest, Mapping):
        raise AuthorizationError("response policy inputs are not objects")
    try:
        expected_candidate_path = Path(candidate_path).relative_to(ROOT).as_posix()
    except ValueError:
        expected_candidate_path = Path(candidate_path).name
    try:
        expected_policy_path = Path(policy_path).relative_to(ROOT).as_posix()
    except ValueError:
        expected_policy_path = Path(policy_path).name
    policy_state = _validate_policy(policy, now)
    try:
        context = _expected_context(plan=plan, root_inputs=root_inputs, repository=repository,
                                    deb_bindings=deb_bindings,
                                    executor_source_sha256=executor_source_sha256)
    except AuthorizationError:
        raise
    except Exception as error:
        raise AuthorizationError(f"authorization source context is invalid: {error}") from error
    required = {"schema", "schema_version", "status", "candidate", "policy", "plan",
                "executor", "trust_root", "repository", "binding", "deb_bindings",
                "deb_bindings_sha256", "tool", "scope", "machine_fingerprint",
                "machine_identity", "issued_at", "expires_at", "nonce", "network_used", "shell",
                "timeout_seconds", "resource_limits", "workflow", "phases", "signature"}
    if not isinstance(auth, Mapping) or set(auth) != required or \
            auth.get("schema") != AUTH_SCHEMA or auth.get("schema_version") != 1 or \
            auth.get("status") != "HOST_RUNTIME_AUTHORIZATION":
        raise AuthorizationError("host authorization shape/status is invalid")
    candidate = auth["candidate"]
    if not isinstance(candidate, Mapping) or set(candidate) != {
            "path", "file_sha256", "canonical_sha256", "candidate_id", "revision"} or \
            candidate.get("path") != expected_candidate_path or \
            candidate.get("file_sha256") != candidate_file_sha or \
            candidate.get("canonical_sha256") != manifest.get("canonical_sha256") or \
            candidate.get("candidate_id") != manifest.get("candidate_id") or \
            candidate.get("revision") != manifest.get("revision"):
        raise AuthorizationError("authorization candidate binding drift")
    if auth["policy"] != {"path": expected_policy_path,
                           "file_sha256": policy_file_sha,
                           "canonical_sha256": policy["canonical_sha256"]}:
        raise AuthorizationError("authorization policy binding drift")
    if auth["plan"] != {"sha256": context["plan_sha256"],
                         "canonical_sha256": context["plan_canonical_sha256"]}:
        raise AuthorizationError("authorization plan identity drift")
    if auth["executor"] != {"source_sha256": context["executor_source_sha256"]}:
        raise AuthorizationError("authorization executor source drift")
    expected_trust = {
        "path": plan["trust_root"]["path"],
        "file_sha256": plan["trust_root"]["file_sha256"],
        "canonical_sha256": plan["trust_root"]["canonical_sha256"],
        "keyring_path": plan["trust_root"]["keyring_path"],
        "keyring_sha256": plan["trust_root"]["keyring_sha256"],
        "key_id": context["trust"]["key_id"],
        "fingerprint": context["trust"]["fingerprint"],
    }
    if auth["trust_root"] != expected_trust:
        raise AuthorizationError("authorization trust-root identity drift")
    if auth["repository"] != dict(context["repository"]):
        raise AuthorizationError("authorization repository identity drift")
    if auth["binding"] != context["binding"]:
        raise AuthorizationError("authorization Release/Packages/deb binding drift")
    if auth["deb_bindings"] != context["deb_bindings"] or \
            auth["deb_bindings_sha256"] != canonical_hash(context["deb_bindings"]):
        raise AuthorizationError("authorization deb binding set drift")
    expected_tool = {"path": context["executable"],
                     "sha256": context["executable_sha256"],
                     "version": context["trust"]["verifier"]["version"]}
    if auth["tool"] != expected_tool:
        raise AuthorizationError("authorization GnuPG tool identity drift")
    expected_scope = {"root_inputs": str(context["root"]),
                      "homedir": context["scope"]["homedir"],
                      "output_root": context["scope"]["output_root"]}
    if auth["scope"] != expected_scope:
        raise AuthorizationError("authorization root/homedir/output binding drift")
    machine_policy = policy_state["machine_identity"]
    if machine_policy["status"] != "PRECOMMITTED":
        raise AuthorizationError("authorization machine identity is NOT_READY")
    machine_relative = _safe_relative(machine_policy["path"], "machine identity path")
    machine_path = context["root"] / machine_relative
    try:
        if policy_state["machine_method"] == KEYED_MACHINE_METHOD:
            machine_result = MACHINE.validate_keyed_file(
                machine_path, live=require_live_machine, provider=machine_provider)
        else:
            machine_result = MACHINE.validate_file(machine_path, live=require_live_machine,
                                                   provider=machine_provider)
    except Exception as error:
        raise AuthorizationError(f"machine identity artifact is invalid: {error}") from error
    machine_document, machine_data = _read_json(machine_path, "machine identity artifact")
    if machine_document.get("status") != "LIVE_CAPTURED" or \
            machine_result.get("live_host_match") is not bool(require_live_machine):
        raise AuthorizationError("machine identity artifact/policy binding drift")
    actual_machine = hashlib.sha256(machine_data).hexdigest()
    if policy_state["machine_method"] == KEYED_MACHINE_METHOD:
        expected_policy_identity = _keyed_machine_descriptor(
            machine_document, machine_path, machine_data, context["root"], policy=True)
        if machine_policy != expected_policy_identity:
            raise AuthorizationError("keyed machine identity policy binding drift")
        expected_machine_identity = {
            key: value for key, value in expected_policy_identity.items() if key != "status"}
    else:
        expected_machine_identity = {
            "schema": machine_policy["schema"], "path": machine_policy["path"],
            "file_sha256": machine_policy["file_sha256"],
            "canonical_sha256": machine_policy["canonical_sha256"],
            "campaign_domain": machine_policy["campaign_domain"],
            "campaign_id": machine_policy["campaign_id"],
            "identity_sha256": machine_policy["identity_sha256"]}
        if machine_document.get("schema") != machine_policy["schema"] or \
                machine_document.get("campaign_domain") != machine_policy["campaign_domain"] or \
                machine_document.get("campaign_id") != machine_policy["campaign_id"] or \
                machine_document.get("identity_sha256") != machine_policy["identity_sha256"] or \
                actual_machine != machine_policy["file_sha256"] or \
                machine_document.get("canonical_sha256") != machine_policy["canonical_sha256"]:
            raise AuthorizationError("legacy machine identity policy binding drift")
    if auth["machine_identity"] != expected_machine_identity or \
            auth["machine_fingerprint"] != actual_machine:
        raise AuthorizationError("authorization machine identity/fingerprint drift")
    if type(auth["issued_at"]) is not int or type(auth["expires_at"]) is not int or \
            auth["expires_at"] <= auth["issued_at"] or now < auth["issued_at"] or \
            now >= auth["expires_at"]:
        raise AuthorizationError("authorization is expired, not-yet-valid, or empty")
    if not isinstance(auth["nonce"], str) or NONCE_RE.fullmatch(auth["nonce"]) is None:
        raise AuthorizationError("authorization nonce is malformed")
    if auth["network_used"] is not False or auth["shell"] is not False or \
            auth["timeout_seconds"] != TIMEOUT_SECONDS or \
            auth["resource_limits"] != RESOURCE_LIMITS:
        raise AuthorizationError("authorization execution safety policy drift")
    if auth["workflow"] != context["workflow"] or auth["phases"] != context["phases"]:
        raise AuthorizationError("authorization expected workflow/phase drift")
    signature = auth["signature"]
    if not isinstance(signature, Mapping) or set(signature) != {
            "algorithm", "key_id", "public_key_sha256", "payload_sha256",
            "signature_base64", "backend"}:
        raise AuthorizationError("authorization signature metadata is incomplete")
    if signature["algorithm"] != ALGORITHM or signature["backend"] != policy_state["backend"]:
        raise AuthorizationError("authorization signature algorithm/backend drift")
    key_id = signature["key_id"]
    key = policy_state["keys"].get(key_id)
    if key is None:
        raise AuthorizationError("authorization key is not in the fixed active policy")
    public_key = key["public_key"]
    expected_public_sha = key["public_key_sha256"]
    if signature["public_key_sha256"] != expected_public_sha:
        raise AuthorizationError("authorization public-key identity drift")
    payload_sha = authorization_payload_sha256(auth)
    if signature["payload_sha256"] != payload_sha:
        raise AuthorizationError("authorization payload hash drift")
    signed = _decode(signature["signature_base64"], "authorization signature", 64)
    try:
        import cryptography
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    except ImportError as error:
        raise AuthorizationError(f"Ed25519 backend unavailable: {error}") from error
    installed_version = getattr(cryptography, "__version__", None)
    if installed_version != policy_state["backend"]["version"]:
        raise AuthorizationError("Ed25519 backend version drift")
    try:
        Ed25519PublicKey.from_public_bytes(public_key).verify(signed, authorization_payload(auth))
    except InvalidSignature as error:
        raise AuthorizationError("authorization Ed25519 signature is invalid") from error
    except (TypeError, ValueError) as error:
        raise AuthorizationError(f"authorization Ed25519 verification failed: {error}") from error
    return {
        "status": "PASS", "verified": True, "authorization_schema": AUTH_SCHEMA,
        "key_id": key_id, "public_key_sha256": expected_public_sha,
        "payload_sha256": payload_sha, "candidate_canonical_sha256": manifest["canonical_sha256"],
        "policy_canonical_sha256": policy["canonical_sha256"],
        "backend": policy_state["backend"], "machine_fingerprint": actual_machine,
        "issued_at": auth["issued_at"], "expires_at": auth["expires_at"],
        "verification_time": now,
        "live_host_match": bool(require_live_machine),
        "machine_method": policy_state["machine_method"],
        "machine_identity": expected_machine_identity,
    }


def verify_authorization_against_policy(auth: Mapping[str, Any], *,
                                        policy: Mapping[str, Any],
                                        manifest: Mapping[str, Any],
                                        candidate_file_sha: str,
                                        policy_file_sha: str,
                                        candidate_path: Path,
                                        policy_path: Path,
                                        plan: Mapping[str, Any],
                                        root_inputs: Path,
                                        repository: Mapping[str, Any],
                                        deb_bindings: list[Mapping[str, Any]],
                                        executor_source_sha256: str,
                                        now: int,
                                        machine_provider: Any = None,
                                        require_live_machine: bool = False) -> dict[str, Any]:
    """Review an externally supplied READY policy without changing fixed policy.

    This is intentionally separate from the executor's fixed-policy entry
    point.  The caller must reopen and bind the candidate/policy bytes before
    calling this review-only seam.  It never changes the checked-in policy or
    makes an externally supplied response eligible by itself.
    """
    return verify_authorization(
        auth, plan=plan, root_inputs=root_inputs, repository=repository,
        deb_bindings=deb_bindings, executor_source_sha256=executor_source_sha256,
        now=now, machine_provider=machine_provider,
        require_live_machine=require_live_machine,
        _fixed_inputs=(policy, manifest, candidate_file_sha, policy_file_sha),
        _candidate_path=Path(candidate_path), _policy_path=Path(policy_path))


def verify_fixed_policy_only() -> dict[str, Any]:
    """Validate the pinned policy without claiming that it authorizes execution."""
    policy, manifest, candidate_file_sha, policy_file_sha = _load_fixed_policy()
    del manifest
    if policy.get("status") != "NOT_READY":
        raise AuthorizationError("checked-in policy unexpectedly enables runtime")
    return {"status": "NOT_READY", "benchmark_eligible": False,
            "candidate_file_sha256": candidate_file_sha,
            "policy_file_sha256": policy_file_sha,
            "policy_canonical_sha256": policy["canonical_sha256"]}
