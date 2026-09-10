#!/usr/bin/env python3
"""Fail-closed binding of host signature receipts to an apt repository set.

This module is deliberately a promotion boundary, not a signature executor.
It reopens an explicit receipt index, revalidates every executor receipt with
the strong source-aware validator, and rejects the current fixture-only
``INJECTED_FIXTURE_ONLY``/``NOT_RUNTIME`` results.  A static or self-rehashed
JSON document is never sufficient.  No gpg, apt, Docker, network, or scorer is
invoked here.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
from typing import Any, Mapping


INDEX_SCHEMA = "glim_clean_room_r3_apt_signature_receipt_index_v1"
INDEX_SCHEMA_VERSION = 1
INDEX_NAME = "apt-release-signature.receipt-index.json"
RECEIPT_NAME = "apt-release-signature.executor.receipt.json"
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
SHA_HEX = frozenset("0123456789abcdef")
ROOT = Path(__file__).resolve().parent
EXECUTOR_PATH = ROOT / "apt_release_signature_executor.py"


class SignaturePromotionError(ValueError):
    """Raised when repository signature evidence cannot authorize promotion."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    projection = dict(value) if isinstance(value, Mapping) else value
    if excluded is not None and isinstance(projection, dict):
        projection.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(ch not in SHA_HEX for ch in value):
        raise SignaturePromotionError(f"{label} is not a lowercase SHA-256")
    return value


def _safe_abs(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith("/") or "\x00" in value:
        raise SignaturePromotionError(f"{label} is not an absolute path")
    path = Path(value)
    if path != Path(os.path.normpath(value)) or path == Path("/"):
        raise SignaturePromotionError(f"{label} is not normalized")
    return path


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or Path(value).as_posix() != value or \
            any(part in {"", ".", ".."} for part in value.split("/")):
        raise SignaturePromotionError(f"{label} is not a safe relative path")
    return value


def _regular_bytes(path: Path, label: str, *, max_bytes: int = MAX_JSON_BYTES,
                   allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise SignaturePromotionError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > max_bytes or \
            (before.st_size == 0 and not allow_empty):
        raise SignaturePromotionError(f"{label} must be a bounded single-link regular file")
    try:
        with path.open("rb") as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(max_bytes + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise SignaturePromotionError(f"{label} cannot be read: {path}: {error}") from error
    if (fd_before.st_dev, fd_before.st_ino) != (before.st_dev, before.st_ino) or \
            (fd_after.st_dev, fd_after.st_ino) != (before.st_dev, before.st_ino) or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > max_bytes or (not data and not allow_empty):
        raise SignaturePromotionError(f"{label} changed while being read: {path}")
    return data


def _read_json(path: Path, label: str, *, max_bytes: int = MAX_JSON_BYTES) -> tuple[dict[str, Any], bytes]:
    data = _regular_bytes(path, label, max_bytes=max_bytes)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise SignaturePromotionError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise SignaturePromotionError(f"{label} must be a JSON object")
    return value, data


def _directory(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise SignaturePromotionError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise SignaturePromotionError(f"{label} must be a non-symlink directory: {path}")


def _path_components_safe(path: Path, label: str) -> None:
    """Reject symlink parents as well as a symlink leaf without resolving them."""
    if not path.is_absolute():
        raise SignaturePromotionError(f"{label} is not absolute")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise SignaturePromotionError(f"{label} component is missing: {current}") from error
        if stat.S_ISLNK(info.st_mode):
            raise SignaturePromotionError(f"{label} contains a symlink component: {current}")


def _load_executor():
    spec = importlib.util.spec_from_file_location("r3_signature_executor_for_promotion", EXECUTOR_PATH)
    if spec is None or spec.loader is None:
        raise SignaturePromotionError("signature executor cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EXECUTOR = _load_executor()


def repository_key(repository: Mapping[str, Any]) -> tuple[str, str]:
    if not isinstance(repository, Mapping):
        raise SignaturePromotionError("repository is not an object")
    required = {"url", "release_url", "release_path", "release_sha256",
                "packages_url", "packages_path", "packages_sha256"}
    if set(repository) != required:
        raise SignaturePromotionError("repository signature fields are incomplete or extra")
    for key in ("url", "release_url", "packages_url"):
        if not isinstance(repository[key], str) or not repository[key].startswith("https://"):
            raise SignaturePromotionError("repository signature URL is not HTTPS")
    _safe_relative(repository["release_path"], "repository Release path")
    _safe_relative(repository["packages_path"], "repository Packages path")
    _sha(repository["release_sha256"], "repository Release SHA")
    _sha(repository["packages_sha256"], "repository Packages SHA")
    return repository["url"], repository["release_url"]


def _expected_repository_map(repositories: Any) -> dict[tuple[str, str], dict[str, Any]]:
    if not isinstance(repositories, list) or not repositories:
        raise SignaturePromotionError("expected repository set is empty")
    result: dict[tuple[str, str], dict[str, Any]] = {}
    for item in repositories:
        repository = dict(item) if isinstance(item, Mapping) else item
        key = repository_key(repository)
        if key in result:
            raise SignaturePromotionError("expected repository set has duplicate identity")
        result[key] = repository
    if list(result) != sorted(result):
        raise SignaturePromotionError("expected repository set is not sorted")
    return result


def _expected_deb_identities(ledger: Mapping[str, Any], key: tuple[str, str]) -> set[tuple[str, str, str]]:
    entries = ledger.get("entries")
    if not isinstance(entries, list):
        raise SignaturePromotionError("acquisition ledger entries are missing")
    result = set()
    for item in entries:
        if not isinstance(item, Mapping):
            raise SignaturePromotionError("acquisition ledger entry is not an object")
        if (item.get("repository_url"), item.get("release_url")) == key:
            identity = tuple(item.get(field) for field in ("name", "version", "architecture"))
            if any(not isinstance(value, str) or not value for value in identity):
                raise SignaturePromotionError("acquisition ledger package identity is malformed")
            if identity in result:
                raise SignaturePromotionError("acquisition ledger has duplicate repository package")
            result.add(identity)
    if not result:
        raise SignaturePromotionError("repository has no package binding")
    return result


def _validate_deb_bindings(value: Any, expected: set[tuple[str, str, str]]) -> list[dict[str, str]]:
    if not isinstance(value, list) or not value:
        raise SignaturePromotionError("signature deb binding set is empty")
    result: list[dict[str, str]] = []
    seen: set[tuple[str, str, str]] = set()
    for item in value:
        if not isinstance(item, Mapping) or set(item) != {"name", "version", "architecture", "path"}:
            raise SignaturePromotionError("signature deb binding fields are incomplete or extra")
        identity = tuple(item[key] for key in ("name", "version", "architecture"))
        if any(not isinstance(part, str) or not part for part in identity) or identity in seen:
            raise SignaturePromotionError("signature deb bindings are duplicate or malformed")
        _safe_relative(item["path"], "signature deb binding path")
        seen.add(identity)
        result.append({key: item[key] for key in ("name", "version", "architecture", "path")})
    if seen != expected or [tuple(item[key] for key in ("name", "version", "architecture"))
                           for item in result] != sorted(seen):
        raise SignaturePromotionError("signature deb binding set does not exactly match repository ledger")
    return result


def _validate_promotable_receipt(receipt_root: Path) -> dict[str, Any]:
    """Accept only a source-revalidated, authorized runtime receipt."""
    path = receipt_root / RECEIPT_NAME
    receipt, data = _read_json(path, "executor receipt", max_bytes=MAX_RECEIPT_BYTES)
    _sha(hashlib.sha256(data).hexdigest(), "executor receipt SHA")
    if receipt.get("candidate_status") != "HOST_AUTHORIZED_SUBPROCESS" or \
            receipt.get("signature_runtime") != "RUNTIME_VERIFIED" or \
            receipt.get("execution_mode") != "HOST_AUTHORIZED_SUBPROCESS" or \
            receipt.get("promotion") != "HOST_AUTHORIZED_RUNTIME" or \
            receipt.get("benchmark_eligible") is not True or \
            not isinstance(receipt.get("authorization"), Mapping):
        raise SignaturePromotionError(
            "executor receipt is static, fixture-only, NOT_RUNTIME, or not host-authorized")
    if receipt.get("status") != "SEALED_EXECUTOR_RESULT" or receipt.get("outcome") != "PASS":
        raise SignaturePromotionError("executor receipt is not a sealed successful host result")
    return receipt


def _validate_entry(entry: Mapping[str, Any], expected: Mapping[str, Any], ledger: Mapping[str, Any],
                    *, require_promotable: bool) -> dict[str, Any]:
    required = {"repository", "receipt_root", "receipt_file_sha256", "plan_path",
                "plan_file_sha256", "plan_identity_sha256", "root_inputs", "deb_bindings"}
    if set(entry) != required:
        raise SignaturePromotionError("signature receipt entry fields are incomplete or extra")
    repository = dict(entry["repository"]) if isinstance(entry["repository"], Mapping) else entry["repository"]
    if repository_key(repository) != repository_key(expected) or repository != dict(expected):
        raise SignaturePromotionError("signature receipt repository binding does not match expected ledger")
    receipt_root = _safe_abs(entry["receipt_root"], "signature receipt root")
    plan_path = _safe_abs(entry["plan_path"], "signature plan path")
    root_inputs = _safe_abs(entry["root_inputs"], "signature input root")
    _path_components_safe(receipt_root, "signature receipt root")
    _path_components_safe(plan_path, "signature plan path")
    _path_components_safe(root_inputs, "signature input root")
    _directory(receipt_root, "signature receipt root")
    _directory(root_inputs, "signature input root")
    plan, plan_data = _read_json(plan_path, "signature plan")
    _sha(entry["plan_file_sha256"], "signature plan file SHA")
    if hashlib.sha256(plan_data).hexdigest() != entry["plan_file_sha256"]:
        raise SignaturePromotionError("signature plan file SHA drift")
    _sha(entry["plan_identity_sha256"], "signature plan identity")
    if plan.get("plan_identity_sha256") != entry["plan_identity_sha256"]:
        raise SignaturePromotionError("signature plan identity drift")
    deb_bindings = _validate_deb_bindings(entry["deb_bindings"],
                                          _expected_deb_identities(ledger, repository_key(repository)))
    receipt_path = receipt_root / RECEIPT_NAME
    receipt_data = _regular_bytes(receipt_path, "signature executor receipt", max_bytes=MAX_RECEIPT_BYTES)
    _sha(entry["receipt_file_sha256"], "signature receipt file SHA")
    if hashlib.sha256(receipt_data).hexdigest() != entry["receipt_file_sha256"]:
        raise SignaturePromotionError("signature receipt file SHA drift")
    # This is the source-aware, plan-aware re-open.  Static validation is never
    # accepted at this boundary.
    try:
        validated = EXECUTOR.validate_executor_receipt(
            receipt_root, plan=plan, root_inputs=root_inputs,
            repository=repository, deb_bindings=deb_bindings)
    except Exception as error:
        raise SignaturePromotionError(f"strong executor receipt validation failed: {error}") from error
    if require_promotable:
        _validate_promotable_receipt(receipt_root)
    return {"repository": repository, "receipt_root": str(receipt_root),
            "receipt_file_sha256": entry["receipt_file_sha256"],
            "plan_identity_sha256": entry["plan_identity_sha256"],
            "executor_validation": validated}


def validate_receipt_index(index_path: Path, *, expected_repositories: Any,
                           ledger: Mapping[str, Any], proposal_sha256: str,
                           require_promotable: bool = True) -> dict[str, Any]:
    """Reopen and validate exactly one strong receipt for every repository.

    ``require_promotable=True`` is the production materialization mode.  The
    current r3 executor intentionally cannot satisfy it because no authorized
    host GnuPG run exists; this is the desired fail-closed result.  Tests and
    review tooling may use ``False`` only to exercise structural/index checks,
    but even then the strong source-aware validator is always called.
    """
    index_path = Path(index_path)
    _path_components_safe(index_path, "signature receipt index")
    index, data = _read_json(index_path, "signature receipt index")
    required = {"schema", "schema_version", "status", "benchmark_eligible",
                "proposal_sha256", "entries", "canonical_sha256"}
    if set(index) != required or index.get("schema") != INDEX_SCHEMA or \
            index.get("schema_version") != INDEX_SCHEMA_VERSION or \
            index.get("status") != "REVIEW_REQUIRED" or index.get("benchmark_eligible") is not False:
        raise SignaturePromotionError("signature receipt index is not review-only")
    _sha(proposal_sha256, "proposal SHA")
    if index["proposal_sha256"] != proposal_sha256:
        raise SignaturePromotionError("signature receipt index proposal binding drift")
    if index["canonical_sha256"] != canonical_hash(index, "canonical_sha256"):
        raise SignaturePromotionError("signature receipt index canonical identity drift")
    repositories = _expected_repository_map(expected_repositories)
    entries = index["entries"]
    if not isinstance(entries, list) or len(entries) != len(repositories):
        raise SignaturePromotionError("signature receipt index is missing or has extra repositories")
    keys: list[tuple[str, str]] = []
    roots: set[str] = set()
    receipts: set[str] = set()
    result = []
    structural: list[tuple[tuple[str, str], Mapping[str, Any]]] = []
    for item in entries:
        if not isinstance(item, Mapping) or not isinstance(item.get("repository"), Mapping):
            raise SignaturePromotionError("signature receipt index entry is malformed")
        key = repository_key(item["repository"])
        if key in keys:
            raise SignaturePromotionError("signature receipt index has duplicate repository receipt")
        if key not in repositories:
            raise SignaturePromotionError("signature receipt index has an extra/cross-campaign repository")
        root_text = item.get("receipt_root")
        if root_text in roots:
            raise SignaturePromotionError("one executor receipt root is replayed across repositories")
        roots.add(root_text)
        receipt_sha = item.get("receipt_file_sha256")
        if receipt_sha in receipts:
            raise SignaturePromotionError("one executor receipt bytes are replayed across repositories")
        receipts.add(receipt_sha)
        keys.append(key)
        structural.append((key, item))
    if keys != sorted(keys) or set(keys) != set(repositories):
        raise SignaturePromotionError("signature receipt index repository coverage/order is invalid")
    for key, item in structural:
        result.append(_validate_entry(item, repositories[key], ledger,
                                      require_promotable=require_promotable))
    return {"status": "PASS", "benchmark_eligible": False,
            "promotion": "AUTHORIZED_HOST_RECEIPT_REQUIRED",
            "proposal_sha256": proposal_sha256,
            "index_file_sha256": hashlib.sha256(data).hexdigest(),
            "entries": result}
