#!/usr/bin/env python3
"""Fixture-only host contract for verifying an apt Release/InRelease signature.

This module never invokes gpg, Docker, apt, or a network client.  It reopens a
precommitted trust root and synthetic verifier status output, and seals a
non-promoting receipt.  A future host executor must run the exact argv emitted
by the companion planner and bind its logs, binary identity, key metadata, and
cleanup records before any receipt can be promoted.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
from typing import Any, Mapping


TRUST_SCHEMA = "glim_clean_room_r3_apt_signature_trust_root_v1"
RECEIPT_SCHEMA = "glim_clean_room_r3_apt_signature_receipt_v1"
PLAN_SCHEMA = "glim_clean_room_r3_apt_signature_plan_v1"
MAX_JSON_BYTES = 2 * 1024 * 1024
MAX_TEXT_BYTES = 32 * 1024 * 1024
MAX_KEYRING_BYTES = 16 * 1024 * 1024
MAX_STATUS_BYTES = 256 * 1024
PHASES = ("trust_root_reopen", "release_reopen", "signature_verify",
          "binding_reopen", "seal_receipt", "cleanup")
FORBIDDEN = {"sh", "bash", "-c", "-lc", "curl", "wget", "nc", "socat"}
BAD_STATUS = {"BADSIG", "ERRSIG", "NO_PUBKEY", "EXPSIG", "EXPKEYSIG",
              "REVKEYSIG", "KEYEXPIRED", "SIGEXPIRED", "NODATA", "FAILURE"}
ALLOWED_STATUS = {"NEWSIG", "GOODSIG", "VALIDSIG", "TRUST_FULLY",
                 "TRUST_ULTIMATE", "KEY_CONSIDERED"}


class SignatureError(ValueError):
    """Raised when an input, verifier result, or receipt is not fail-closed."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if isinstance(value, Mapping):
        value = dict(value)
        if excluded is not None:
            value.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise SignatureError(f"{label} is not a lowercase SHA-256")
    return value


def _safe_text(value: Any, label: str, limit: int = 256) -> str:
    if not isinstance(value, str) or not value or len(value) > limit or \
            any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise SignatureError(f"{label} is not safe text")
    return value


def _safe_absolute(value: Any, label: str, limit: int = 4096) -> str:
    parts = value[1:].split("/") if isinstance(value, str) and value.startswith("/") else []
    if not isinstance(value, str) or not value.startswith("/") or len(value) > limit or \
            Path(value).as_posix() != value or any(part in {"", ".", ".."} for part in parts) or \
            "\x00" in value or "\n" in value or "\r" in value:
        raise SignatureError(f"{label} is not a safe absolute path")
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or Path(value).as_posix() != value or \
            any(part in {"", ".", ".."} for part in value.split("/")):
        raise SignatureError(f"{label} is not a normalized relative path")
    return value


def _ensure_parent_no_symlink(path: Path, label: str) -> None:
    parent = Path(path).parent
    current = Path(parent.anchor)
    for component in parent.parts[1:]:
        current = current / component
        try:
            info = current.lstat()
        except OSError as error:
            raise SignatureError(f"{label} parent cannot be inspected: {current}") from error
        if stat.S_ISLNK(info.st_mode):
            raise SignatureError(f"{label} parent contains a symlink: {current}")


def _checked_root(path: Path, label: str = "signature input root") -> Path:
    path = Path(path)
    if not path.is_absolute():
        raise SignatureError(f"{label} must be absolute")
    _ensure_parent_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError as error:
        raise SignatureError(f"{label} cannot be inspected: {path}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise SignatureError(f"{label} must be a non-symlink directory")
    return path


def _regular(path: Path, label: str, *, limit: int, allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise SignatureError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > limit or \
            (before.st_size == 0 and not allow_empty):
        raise SignatureError(f"{label} must be a bounded single-link regular file")
    try:
        with path.open("rb") as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise SignatureError(f"{label} cannot be read: {path}: {error}") from error
    if fd_before.st_dev != before.st_dev or fd_before.st_ino != before.st_ino or \
            fd_after.st_dev != before.st_dev or fd_after.st_ino != before.st_ino or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > limit or (not data and not allow_empty):
        raise SignatureError(f"{label} changed while being read")
    return data


def _read_json(path: Path, label: str) -> dict[str, Any]:
    data = _regular(path, label, limit=MAX_JSON_BYTES)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise SignatureError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise SignatureError(f"{label} must be a JSON object")
    return value


def _fingerprint(value: Any, label: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9A-F]{40}", value) is None:
        raise SignatureError(f"{label} is not an uppercase OpenPGP fingerprint")
    return value


def _key_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9A-F]{16}", value) is None:
        raise SignatureError(f"{label} is not an uppercase key id")
    return value


def _validate_validity(value: Any, fingerprint: str) -> dict[str, Any]:
    required = {"primary_fingerprint", "created_at", "expires_at", "revoked", "subkeys"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise SignatureError("trust-root key validity fields are incomplete or extra")
    if value["primary_fingerprint"] != fingerprint or type(value["revoked"]) is not bool:
        raise SignatureError("trust-root primary key validity is inconsistent")
    if type(value["created_at"]) is not int or type(value["expires_at"]) is not int or \
            value["created_at"] < 0 or value["expires_at"] < 0:
        raise SignatureError("trust-root key validity times are invalid")
    subkeys = value["subkeys"]
    if not isinstance(subkeys, list):
        raise SignatureError("trust-root subkey validity is not a list")
    seen: set[str] = set()
    for item in subkeys:
        if not isinstance(item, Mapping) or set(item) != {
                "fingerprint", "created_at", "expires_at", "revoked"}:
            raise SignatureError("trust-root subkey validity fields are incomplete or extra")
        subkey = _fingerprint(item["fingerprint"], "trust-root subkey fingerprint")
        if subkey in seen or subkey == fingerprint:
            raise SignatureError("trust-root subkey fingerprints are not unique")
        seen.add(subkey)
        if type(item["revoked"]) is not bool or type(item["created_at"]) is not int or \
                type(item["expires_at"]) is not int or item["created_at"] < 0 or \
                item["expires_at"] < 0:
            raise SignatureError("trust-root subkey validity values are invalid")
    return dict(value)


def validate_trust_root(root: Path, path: Path) -> dict[str, Any]:
    root = _checked_root(Path(root))
    path = Path(path)
    data = _read_json(path, "trust-root manifest")
    required = {"schema", "schema_version", "status", "key_id", "fingerprint",
                "keyring_path", "keyring_bytes", "keyring_sha256", "verification_time",
                "validity", "verifier", "canonical_sha256"}
    if set(data) != required or data["schema"] != TRUST_SCHEMA or \
            data["schema_version"] != 1 or data["status"] != "PRECOMMITTED":
        raise SignatureError("trust-root schema/status is not exact")
    _key_id(data["key_id"], "trust-root key id")
    fingerprint = _fingerprint(data["fingerprint"], "trust-root fingerprint")
    if type(data["keyring_bytes"]) is not int or data["keyring_bytes"] <= 0 or \
            data["keyring_bytes"] > MAX_KEYRING_BYTES:
        raise SignatureError("trust-root keyring byte count is invalid")
    _sha(data["keyring_sha256"], "trust-root keyring SHA")
    if type(data["verification_time"]) is not int or data["verification_time"] <= 0:
        raise SignatureError("trust-root verification time is invalid")
    validity = _validate_validity(data["validity"], fingerprint)
    if validity["created_at"] > data["verification_time"] or validity["revoked"] or \
            validity["expires_at"] not in (0,) and validity["expires_at"] <= data["verification_time"]:
        raise SignatureError("trust-root primary key is expired, revoked, or not yet valid")
    for subkey in validity["subkeys"]:
        if subkey["created_at"] > data["verification_time"] or subkey["revoked"] or \
                subkey["expires_at"] not in (0,) and subkey["expires_at"] <= data["verification_time"]:
            raise SignatureError("trust-root subkey is expired, revoked, or not yet valid")
    verifier = data["verifier"]
    if not isinstance(verifier, Mapping) or set(verifier) != {
            "name", "version", "binary_sha256", "status_protocol", "binary_path"} or \
            verifier["name"] != "gpg" or verifier["status_protocol"] != "gpg-status-fd-v1":
        raise SignatureError("trust-root verifier identity is not the fixed gpg contract")
    _safe_text(verifier["version"], "trust-root verifier version", 64)
    _sha(verifier["binary_sha256"], "trust-root verifier binary SHA")
    _safe_absolute(verifier["binary_path"], "trust-root verifier binary path")
    keyring_path = _safe_relative(data["keyring_path"], "trust-root keyring path")
    _, keyring_data = _input_relative(root, root / keyring_path,
                                      "trust-root keyring", limit=MAX_KEYRING_BYTES)
    if len(keyring_data) != data["keyring_bytes"] or sha256_bytes(keyring_data) != data["keyring_sha256"]:
        raise SignatureError("trust-root keyring bytes/SHA drift")
    if data["canonical_sha256"] != canonical_hash(data, "canonical_sha256"):
        raise SignatureError("trust-root canonical identity drift")
    return data


def build_verify_argv(*, homedir: Path, keyring: Path, release: Path,
                      release_kind: str, signature: Path | None = None,
                      executable: str) -> list[str]:
    if release_kind not in {"INRELEASE", "RELEASE_DETACHED"}:
        raise SignatureError("release kind is unsupported")
    homedir = Path(homedir).resolve()
    keyring = Path(keyring).resolve()
    release = Path(release).resolve()
    executable = _safe_absolute(executable, "signature verifier executable")
    argv = [executable, "--batch", "--no-options", "--no-default-keyring",
            "--no-auto-check-trustdb", "--no-auto-key-retrieve",
            "--no-auto-key-import", "--no-auto-key-locate", "--no-autostart",
            "--homedir", str(homedir), "--status-fd", "1", "--keyring", str(keyring),
            "--verify"]
    if release_kind == "RELEASE_DETACHED":
        if signature is None:
            raise SignatureError("detached Release requires a signature path")
        argv.extend([str(signature.resolve()), str(release)])
    else:
        if signature is not None:
            raise SignatureError("InRelease must not have a detached signature")
        argv.append(str(release))
    return argv


def build_key_inventory_argv(*, homedir: Path, keyring: Path,
                             executable: str) -> list[str]:
    """Return the only permitted future keyring inventory command.

    The command is recorded for a future host executor, but this module never
    starts it.  ``--no-default-keyring`` prevents ambient user keyrings and
    ``--show-keys`` makes the inspected keyring an explicit input artifact.
    """
    executable = _safe_absolute(executable, "key inventory executable")
    return [executable, "--batch", "--no-options", "--no-default-keyring",
            "--no-auto-key-retrieve", "--no-auto-key-import",
            "--no-auto-key-locate", "--no-autostart", "--homedir",
            str(Path(homedir).resolve()), "--with-colons", "--with-fingerprint",
            "--with-subkey-fingerprint", "--show-keys", str(Path(keyring).resolve())]


def validate_key_inventory_argv(argv: Any, expected: list[str]) -> None:
    validate_verify_argv(argv, expected)


def validate_key_inventory_output(data: bytes, trust_root: Mapping[str, Any]) -> dict[str, Any]:
    """Parse a bounded ``gpg --with-colons --show-keys`` fixture.

    Only key and fingerprint records are used for identity; UID records are
    accepted but never trusted as a key identity.  Any other record is
    rejected so a future executor cannot silently hide an unknown key/status.
    """
    if len(data) == 0 or len(data) > MAX_STATUS_BYTES:
        raise SignatureError("key inventory is empty or oversized")
    try:
        lines = data.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise SignatureError("key inventory is not UTF-8") from error
    expected_fp = trust_root["fingerprint"]
    expected_key_id = trust_root["key_id"]
    primary: dict[str, Any] | None = None
    subkeys: list[dict[str, Any]] = []
    pending_kind: str | None = None
    pending_key_id: str | None = None
    for ordinal, line in enumerate(lines):
        fields = line.split(":")
        if not fields or fields[0] not in {"pub", "sub", "fpr", "uid"}:
            raise SignatureError(f"key inventory line {ordinal} has unknown record type")
        kind = fields[0]
        if kind in {"pub", "sub"}:
            if len(fields) < 7:
                raise SignatureError("key inventory key record is truncated")
            if fields[1] not in {"-", "q", "n", "u", "f", "m"}:
                raise SignatureError("key inventory key validity is unknown/revoked/expired")
            key_id = _key_id(fields[4].upper(), "key inventory key id")
            created = _check_time(fields[5], "key inventory creation time")
            expires = _check_time(fields[6] or "0", "key inventory expiry time")
            pending_kind, pending_key_id = kind, key_id
            record = {"key_id": key_id, "created_at": created,
                      "expires_at": expires, "revoked": False}
            if kind == "pub":
                if primary is not None:
                    raise SignatureError("key inventory has duplicate primary key")
                primary = record
            else:
                subkeys.append(record)
        elif kind == "fpr":
            if len(fields) <= 9 or pending_kind is None or pending_key_id is None:
                raise SignatureError("key inventory fingerprint is not bound to a key")
            fp = _fingerprint(fields[9].upper(), "key inventory fingerprint")
            if pending_kind == "pub":
                if fp != expected_fp or pending_key_id != expected_key_id:
                    raise SignatureError("key inventory primary identity mismatch")
                primary["fingerprint"] = fp
            else:
                if fp == expected_fp or any(item.get("fingerprint") == fp for item in subkeys):
                    raise SignatureError("key inventory subkey identity is duplicated")
                subkeys[-1]["fingerprint"] = fp
            pending_kind = pending_key_id = None
        else:  # uid
            if len(fields) < 10 or not fields[9]:
                raise SignatureError("key inventory UID record is malformed")
    if primary is None or primary.get("fingerprint") != expected_fp or \
            any("fingerprint" not in item for item in subkeys):
        raise SignatureError("key inventory does not prove the precommitted key")
    expected_primary = trust_root["validity"]
    if (primary["created_at"], primary["expires_at"], primary["revoked"]) != \
            (expected_primary["created_at"], expected_primary["expires_at"],
             expected_primary["revoked"]):
        raise SignatureError("key inventory primary validity drift")
    expected_subkeys = trust_root["validity"]["subkeys"]
    projected = {"primary": primary, "subkeys": subkeys}
    if [(item["fingerprint"], item["created_at"], item["expires_at"])
            for item in subkeys] != [(item["fingerprint"], item["created_at"], item["expires_at"])
                                      for item in expected_subkeys]:
        raise SignatureError("key inventory subkey validity drift")
    projected["sha256"] = sha256_bytes(data)
    projected["bytes"] = len(data)
    return projected


def validate_verify_argv(argv: Any, expected: list[str]) -> None:
    if not isinstance(argv, list) or argv != expected or \
            any(not isinstance(item, str) or not item for item in argv) or \
            any(item in FORBIDDEN for item in argv):
        raise SignatureError("signature verifier argv is not the exact bounded command")


def _check_time(value: str, label: str) -> int:
    if re.fullmatch(r"[0-9]+", value) is None:
        raise SignatureError(f"{label} is not an integer timestamp")
    return int(value, 10)


def validate_status_output(data: bytes, trust_root: Mapping[str, Any]) -> dict[str, Any]:
    if len(data) == 0 or len(data) > MAX_STATUS_BYTES:
        raise SignatureError("gpg status output is empty or oversized")
    try:
        lines = data.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise SignatureError("gpg status output is not UTF-8") from error
    fingerprint = trust_root["fingerprint"]
    valid_fingerprints = {fingerprint} | {
        item["fingerprint"] for item in trust_root["validity"]["subkeys"]}
    valid_key_ids = {value[-16:] for value in valid_fingerprints}
    counts: dict[str, int] = {}
    valid = None
    signer_key_id = None
    for ordinal, line in enumerate(lines):
        if not line.startswith("[GNUPG:] "):
            raise SignatureError(f"gpg status line {ordinal} is not machine status")
        fields = line[len("[GNUPG:] "):].split()
        if not fields:
            raise SignatureError("gpg status line is empty")
        tag = fields[0]
        counts[tag] = counts.get(tag, 0) + 1
        if tag in BAD_STATUS or tag not in ALLOWED_STATUS:
            raise SignatureError(f"gpg status contains rejected/unknown token: {tag}")
        if tag == "GOODSIG":
            if len(fields) < 3 or fields[1] not in valid_key_ids:
                raise SignatureError("gpg GOODSIG key id does not match trust root")
            signer_key_id = fields[1]
        elif tag == "KEY_CONSIDERED":
            if len(fields) != 3 or fields[1] not in valid_fingerprints or \
                    fields[2] != "0":
                raise SignatureError("gpg KEY_CONSIDERED signer mismatch")
        elif tag == "VALIDSIG":
            # After the tag, GnuPG emits fingerprint, ISO date, creation and
            # expiration timestamps, version/reserved bytes, public-key/hash
            # algorithms, signature class, then (optionally) the primary-key
            # fingerprint.  In particular the primary fingerprint is field
            # 10 of the split status record, not field 9 (the signature class).
            if len(fields) not in {10, 11} or fields[1] not in valid_fingerprints:
                raise SignatureError("gpg VALIDSIG fingerprint does not match trust root")
            if re.fullmatch(r"[0-9]{4}-[0-9]{2}-[0-9]{2}", fields[2]) is None or \
                    any(re.fullmatch(r"[0-9]+", fields[index]) is None
                        for index in (5, 6, 7, 8)) or \
                    re.fullmatch(r"[0-9A-Fa-f]{2,}", fields[9]) is None:
                raise SignatureError("gpg VALIDSIG fields are malformed")
            signed_at = _check_time(fields[3], "gpg signature timestamp")
            expires_at = _check_time(fields[4], "gpg signature expiration")
            if signed_at > trust_root["verification_time"] or \
                    expires_at not in (0,) and expires_at <= trust_root["verification_time"]:
                raise SignatureError("gpg signature is outside the fixed verification time")
            if len(fields) == 11 and fields[10] != trust_root["fingerprint"]:
                raise SignatureError("gpg primary fingerprint does not match trust root")
            valid = {"fingerprint": fields[1], "signed_at": signed_at,
                     "expires_at": expires_at}
        elif tag in {"TRUST_FULLY", "TRUST_ULTIMATE"}:
            if len(fields) < 2 or fields[1] not in {"0", "1", "2", "3", "4", "5"}:
                raise SignatureError("gpg trust status is malformed")
    if counts.get("GOODSIG") != 1 or counts.get("VALIDSIG") != 1 or valid is None or \
            signer_key_id is None or \
            counts.get("TRUST_ULTIMATE", 0) + counts.get("TRUST_FULLY", 0) != 1 or \
            counts.get("KEY_CONSIDERED", 0) != 1:
        raise SignatureError("gpg status lacks exactly one trusted valid signature")
    return {"status": "PASS", "key_id": signer_key_id, **valid,
            "status_output_sha256": sha256_bytes(data), "status_output_bytes": len(data)}


def _validate_repository(repository: Mapping[str, Any]) -> dict[str, Any]:
    required = {"url", "release_url", "release_path", "release_sha256",
                "packages_url", "packages_path", "packages_sha256"}
    if not isinstance(repository, Mapping) or set(repository) != required:
        raise SignatureError("signature repository binding fields are incomplete")
    for key in ("url", "release_url", "packages_url"):
        if not isinstance(repository[key], str) or not repository[key].startswith("https://"):
            raise SignatureError("signature repository URL is not HTTPS")
    _safe_relative(repository["release_path"], "signature Release path")
    _safe_relative(repository["packages_path"], "signature Packages path")
    _sha(repository["release_sha256"], "signature Release SHA")
    _sha(repository["packages_sha256"], "signature Packages SHA")
    return dict(repository)


def _input_relative(root: Path, path: Path, label: str, *, limit: int) -> tuple[str, bytes]:
    root = _checked_root(Path(root))
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = root / candidate
    try:
        relative = candidate.relative_to(root)
    except ValueError as error:
        raise SignatureError(f"{label} escapes the input root") from error
    relative_text = _safe_relative(relative.as_posix(), f"{label} path")
    current = root
    for component in relative.parts:
        current = current / component
        try:
            info = current.lstat()
        except OSError as error:
            raise SignatureError(f"{label} component cannot be inspected: {current}") from error
        if stat.S_ISLNK(info.st_mode):
            raise SignatureError(f"{label} contains a symlink component: {current}")
        if component != relative.parts[-1] and not stat.S_ISDIR(info.st_mode):
            raise SignatureError(f"{label} parent component is not a directory: {current}")
    return relative_text, _regular(current, label, limit=limit)


def _signature_binding(root: Path, release_kind: str, signature_path: Path | None) -> dict[str, Any]:
    if release_kind == "INRELEASE":
        if signature_path is not None:
            raise SignatureError("InRelease must not carry detached signature bytes")
        return {"kind": "INRELEASE", "path": "", "bytes": 0, "sha256": ""}
    if release_kind != "RELEASE_DETACHED" or signature_path is None:
        raise SignatureError("detached Release signature artifact is missing")
    relative, data = _input_relative(root, signature_path, "detached Release signature",
                                     limit=MAX_TEXT_BYTES)
    return {"kind": "RELEASE_DETACHED", "path": relative,
            "bytes": len(data), "sha256": sha256_bytes(data)}


def _load_composer():
    import importlib.util
    path = Path(__file__).with_name("apt_allowlist_composer.py")
    spec = importlib.util.spec_from_file_location("r3_signature_composer", path)
    if spec is None or spec.loader is None:
        raise SignatureError("existing apt binding composer cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _binding(root: Path, release_path: Path, packages_path: Path,
             repository: Mapping[str, Any], deb_bindings: list[Mapping[str, Any]],
             *, release_kind: str = "INRELEASE",
             signature_path: Path | None = None) -> dict[str, Any]:
    repository = _validate_repository(repository)
    _, release_data = _input_relative(root, release_path, "Release/InRelease",
                                      limit=MAX_TEXT_BYTES)
    _, packages_data = _input_relative(root, packages_path, "Packages index",
                                       limit=MAX_TEXT_BYTES)
    if sha256_bytes(release_data) != repository["release_sha256"] or \
            sha256_bytes(packages_data) != repository["packages_sha256"]:
        raise SignatureError("signature binding Release/Packages SHA drift")
    try:
        _load_composer()._validate_release_packages_binding(
            release_data, packages_data, repository)
    except Exception as error:
        if isinstance(error, SignatureError):
            raise
        raise SignatureError(f"existing Release/Packages binding rejected: {error}") from error
    if not isinstance(deb_bindings, list) or not deb_bindings:
        raise SignatureError("signature binding deb set is empty")
    debs: list[dict[str, Any]] = []
    identities: set[tuple[str, str, str]] = set()
    for item in deb_bindings:
        if not isinstance(item, Mapping) or set(item) != {
                "name", "version", "architecture", "path"}:
            raise SignatureError("signature deb binding fields are incomplete or extra")
        identity = tuple(item[key] for key in ("name", "version", "architecture"))
        if any(not isinstance(value, str) or not value for value in identity) or identity in identities:
            raise SignatureError("signature deb identities are not unique")
        identities.add(identity)
        relative = _safe_relative(item["path"], "signature deb path")
        _, data = _input_relative(root, root / relative, "signature deb",
                                  limit=512 * 1024 * 1024)
        debs.append({"name": identity[0], "version": identity[1],
                     "architecture": identity[2], "path": relative,
                     "bytes": len(data), "sha256": sha256_bytes(data)})
    debs.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
    result = {"repository": {"release_url": repository["release_url"],
                              "release_path": repository["release_path"],
                              "release_sha256": repository["release_sha256"],
                              "packages_url": repository["packages_url"],
                              "packages_path": repository["packages_path"],
                              "packages_sha256": repository["packages_sha256"]},
              "release_kind": release_kind,
              "signature": _signature_binding(root, release_kind, signature_path),
              "release_sha256": sha256_bytes(release_data),
              "packages_sha256": sha256_bytes(packages_data),
              "debs": debs}
    result["binding_identity_sha256"] = canonical_hash(result)
    return result


def _validate_phases(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or [item.get("phase") for item in value] != list(PHASES):
        raise SignatureError("signature phase records are missing or reordered")
    result = []
    for ordinal, item in enumerate(value):
        if not isinstance(item, Mapping) or set(item) != {"ordinal", "phase", "status"} or \
                item["ordinal"] != ordinal or not isinstance(item["status"], str):
            raise SignatureError("signature phase record is incomplete")
        if item["status"] not in {"INJECTED_REOPEN", "INJECTED_VERIFIER",
                                   "SEALED_FIXTURE", "NOT_RUN"}:
            raise SignatureError("signature phase status is unknown")
        result.append(dict(item))
    return result


def seal_signature_receipt(*, root: Path, trust_root_path: Path, release_path: Path,
                           packages_path: Path, repository: Mapping[str, Any],
                           deb_bindings: list[Mapping[str, Any]], homedir: Path,
                           release_kind: str, signature_path: Path | None,
                           verifier_log_path: Path, key_inventory_path: Path,
                           command: list[str],
                           verifier_exit_status: int, phases: list[dict[str, Any]],
                           output_root: Path) -> dict[str, Any]:
    """Seal a synthetic verifier result; no verification process is started."""
    root = _checked_root(Path(root))
    trust = validate_trust_root(root, Path(trust_root_path))
    keyring = root / trust["keyring_path"]
    expected_argv = build_verify_argv(homedir=homedir, keyring=keyring,
                                      release=release_path, release_kind=release_kind,
                                      signature=signature_path,
                                      executable=trust["verifier"]["binary_path"])
    validate_verify_argv(command, expected_argv)
    inventory_command = build_key_inventory_argv(
        homedir=homedir, keyring=keyring, executable=trust["verifier"]["binary_path"])
    validate_key_inventory_argv(inventory_command, inventory_command)
    if type(verifier_exit_status) is not int or verifier_exit_status != 0:
        raise SignatureError("signature verifier did not exit successfully")
    status_data = _regular(Path(verifier_log_path), "gpg status log", limit=MAX_STATUS_BYTES)
    status = validate_status_output(status_data, trust)
    status_relative, _ = _input_relative(root, Path(verifier_log_path),
                                          "gpg status log", limit=MAX_STATUS_BYTES)
    inventory_relative, inventory_data = _input_relative(
        root, Path(key_inventory_path), "gpg key inventory", limit=MAX_STATUS_BYTES)
    inventory = validate_key_inventory_output(inventory_data, trust)
    binding = _binding(root, Path(release_path), Path(packages_path), repository, deb_bindings,
                       release_kind=release_kind, signature_path=signature_path)
    phases = _validate_phases(phases)
    if phases[2]["status"] != "INJECTED_VERIFIER" or phases[5]["status"] != "NOT_RUN":
        raise SignatureError("synthetic signature phase is not explicitly non-runtime")
    output_root = Path(output_root)
    _ensure_parent_no_symlink(output_root, "signature receipt output")
    if not output_root.is_absolute() or output_root.exists() or output_root.is_symlink() or \
            not output_root.parent.is_dir() or output_root.parent.is_symlink():
        raise SignatureError("signature receipt output root must be fresh")
    output_root.mkdir(mode=0o700)
    receipt = {
        "schema": RECEIPT_SCHEMA, "schema_version": 1,
        "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
        "execution": "NOT_RUN", "signature_status": "SYNTHETIC_PASS_NOT_RUNTIME_VALIDATED",
        "trust_root": {"key_id": trust["key_id"], "fingerprint": trust["fingerprint"],
                        "keyring_path": trust["keyring_path"],
                        "keyring_sha256": trust["keyring_sha256"],
                        "verification_time": trust["verification_time"],
                        "validity_sha256": canonical_hash(trust["validity"])},
        "verifier": {"name": trust["verifier"]["name"], "version": trust["verifier"]["version"],
                     "binary_sha256": trust["verifier"]["binary_sha256"],
                     "binary_path": trust["verifier"]["binary_path"],
                     "status_protocol": trust["verifier"]["status_protocol"],
                     "mode": "INJECTED_FIXTURE_ONLY", "exit_status": verifier_exit_status,
                     "homedir": str(Path(homedir).resolve()),
                     "keyring": str(keyring.resolve()),
                     "release": str(Path(release_path).resolve()),
                     "release_kind": release_kind,
                     "signature": "" if signature_path is None else str(Path(signature_path).resolve()),
                     "command": list(command), "command_sha256": canonical_hash(command),
                     "inventory_command": inventory_command,
                     "inventory_command_sha256": canonical_hash(inventory_command),
                     "status_log_path": status_relative,
                     "status_log_sha256": status["status_output_sha256"],
                     "status_log_bytes": status["status_output_bytes"],
                     "inventory_log_path": inventory_relative,
                     "inventory_log_sha256": inventory["sha256"],
                     "inventory_log_bytes": inventory["bytes"],
                     "key_inventory": inventory},
        "signature": status,
        "binding": binding,
        "phases": phases,
        "outer_executor": {"status": "ABSENT_FAIL_CLOSED", "entrypoint": "",
                            "sha256": "0" * 64},
        "canonical_sha256": "",
    }
    receipt["canonical_sha256"] = canonical_hash(receipt, "canonical_sha256")
    data = canonical_bytes(receipt) + b"\n"
    receipt_path = output_root / "apt-release-signature.receipt.json"
    sidecar = output_root / "apt-release-signature.receipt.json.sha256"
    with receipt_path.open("xb") as stream:
        stream.write(data)
        stream.flush()
    receipt_path.chmod(0o444)
    sidecar_data = (sha256_bytes(data) + "  apt-release-signature.receipt.json\n").encode("ascii")
    with sidecar.open("xb") as stream:
        stream.write(sidecar_data)
        stream.flush()
    sidecar.chmod(0o444)
    return validate_receipt(output_root, root_inputs=root, trust_root_path=trust_root_path,
                            release_path=release_path, packages_path=packages_path,
                            repository=repository, deb_bindings=deb_bindings,
                            homedir=homedir, release_kind=release_kind,
                            signature_path=signature_path)


def validate_receipt(root: Path, *, root_inputs: Path, trust_root_path: Path,
                     release_path: Path, packages_path: Path,
                     repository: Mapping[str, Any],
                     deb_bindings: list[Mapping[str, Any]], homedir: Path,
                     release_kind: str, signature_path: Path | None) -> dict[str, Any]:
    root = Path(root)
    receipt_path = root / "apt-release-signature.receipt.json"
    sidecar_path = root / "apt-release-signature.receipt.json.sha256"
    data = _regular(receipt_path, "signature receipt", limit=MAX_JSON_BYTES)
    sidecar = _regular(sidecar_path, "signature receipt sidecar", limit=512)
    if sidecar.decode("ascii").strip().split() != [sha256_bytes(data), receipt_path.name]:
        raise SignatureError("signature receipt sidecar mismatch")
    receipt = json.loads(data.decode("utf-8"))
    required = {"schema", "schema_version", "status", "benchmark_eligible", "execution",
                "signature_status", "trust_root", "verifier", "signature", "binding",
                "phases", "outer_executor", "canonical_sha256"}
    if not isinstance(receipt, Mapping) or set(receipt) != required or \
            receipt["schema"] != RECEIPT_SCHEMA or receipt["schema_version"] != 1 or \
            receipt["status"] != "REVIEW_REQUIRED" or receipt["benchmark_eligible"] is not False or \
            receipt["execution"] != "NOT_RUN" or \
            receipt["signature_status"] != "SYNTHETIC_PASS_NOT_RUNTIME_VALIDATED":
        raise SignatureError("signature receipt status/fields are not non-promoting")
    if receipt["canonical_sha256"] != canonical_hash(receipt, "canonical_sha256"):
        raise SignatureError("signature receipt canonical identity drift")
    if receipt["outer_executor"] != {"status": "ABSENT_FAIL_CLOSED", "entrypoint": "",
                                     "sha256": "0" * 64}:
        raise SignatureError("signature receipt executor is not absent/fail-closed")
    phases = _validate_phases(receipt["phases"])
    if [item["status"] for item in phases] != ["INJECTED_REOPEN", "INJECTED_REOPEN",
                                                 "INJECTED_VERIFIER", "INJECTED_REOPEN",
                                                 "SEALED_FIXTURE", "NOT_RUN"]:
        raise SignatureError("signature receipt phase status/order is not exact")
    trust = validate_trust_root(Path(root_inputs), Path(trust_root_path))
    trust_projection = receipt["trust_root"]
    if trust_projection != {
            "key_id": trust["key_id"], "fingerprint": trust["fingerprint"],
            "keyring_path": trust["keyring_path"], "keyring_sha256": trust["keyring_sha256"],
            "verification_time": trust["verification_time"],
            "validity_sha256": canonical_hash(trust["validity"])}:
        raise SignatureError("signature receipt trust-root binding drift")
    expected_binding = _binding(Path(root_inputs), Path(release_path), Path(packages_path),
                                repository, deb_bindings, release_kind=release_kind,
                                signature_path=signature_path)
    if receipt["binding"] != expected_binding:
        raise SignatureError("signature receipt Release/Packages/deb binding drift")
    verifier = receipt["verifier"]
    if not isinstance(verifier, Mapping) or set(verifier) != {
            "name", "version", "binary_sha256", "binary_path", "status_protocol", "mode",
            "exit_status", "homedir", "keyring", "release", "release_kind",
            "signature", "command", "command_sha256", "inventory_command",
            "inventory_command_sha256", "status_log_path", "status_log_sha256",
            "status_log_bytes", "inventory_log_path", "inventory_log_sha256",
            "inventory_log_bytes", "key_inventory"} or \
            verifier.get("mode") != "INJECTED_FIXTURE_ONLY" or \
            verifier.get("exit_status") != 0:
        raise SignatureError("signature receipt verifier is not an injected bounded result")
    trust_keyring = Path(root_inputs) / trust["keyring_path"]
    expected_command = build_verify_argv(
        homedir=homedir, keyring=trust_keyring, release=release_path,
        release_kind=release_kind, signature=signature_path,
        executable=trust["verifier"]["binary_path"])
    validate_verify_argv(verifier.get("command"), expected_command)
    if verifier.get("command_sha256") != canonical_hash(expected_command) or \
            verifier.get("homedir") != str(Path(homedir).resolve()) or \
            verifier.get("keyring") != str(trust_keyring.resolve()) or \
            verifier.get("release") != str(Path(release_path).resolve()) or \
            verifier.get("release_kind") != release_kind or \
            verifier.get("signature") != ("" if signature_path is None else
                                           str(Path(signature_path).resolve())):
        raise SignatureError("signature receipt verifier command identity drift")
    expected_inventory_command = build_key_inventory_argv(
        homedir=homedir, keyring=trust_keyring,
        executable=trust["verifier"]["binary_path"])
    validate_key_inventory_argv(verifier.get("inventory_command"), expected_inventory_command)
    if verifier.get("inventory_command_sha256") != canonical_hash(expected_inventory_command):
        raise SignatureError("signature receipt key inventory command identity drift")
    status_path = Path(root_inputs) / _safe_relative(verifier["status_log_path"],
                                                     "signature status log path")
    status_relative, status_data = _input_relative(
        Path(root_inputs), status_path, "synthetic gpg status log", limit=MAX_STATUS_BYTES)
    if status_relative != verifier["status_log_path"]:
        raise SignatureError("signature status log path normalization drift")
    status = validate_status_output(status_data, trust)
    inventory_path = Path(root_inputs) / _safe_relative(verifier["inventory_log_path"],
                                                        "signature inventory log path")
    inventory_relative, inventory_data = _input_relative(
        Path(root_inputs), inventory_path, "synthetic gpg key inventory", limit=MAX_STATUS_BYTES)
    if inventory_relative != verifier["inventory_log_path"]:
        raise SignatureError("signature inventory log path normalization drift")
    inventory = validate_key_inventory_output(inventory_data, trust)
    if verifier.get("status_log_sha256") != status["status_output_sha256"] or \
            verifier.get("status_log_bytes") != status["status_output_bytes"] or \
            verifier.get("inventory_log_sha256") != inventory["sha256"] or \
            verifier.get("inventory_log_bytes") != inventory["bytes"] or \
            verifier.get("key_inventory") != inventory or \
            receipt["signature"] != status:
        raise SignatureError("signature receipt verifier output binding drift")
    return {"status": "PASS", "benchmark_eligible": False,
            "signature_status": receipt["signature_status"],
            "receipt_sha256": sha256_bytes(data),
            "binding_identity_sha256": expected_binding["binding_identity_sha256"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--help-contract", action="store_true")
    args = parser.parse_args()
    if args.help_contract:
        print(__doc__)
        return 0
    parser.error("runtime signature execution is not implemented; use fixture APIs")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
