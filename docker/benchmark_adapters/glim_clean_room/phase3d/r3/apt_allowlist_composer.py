#!/usr/bin/env python3
"""Fixture-only candidate for a reviewed Jazzy apt/deb closure.

This module deliberately does not invoke Docker, apt, dpkg, a resolver, or a
network client.  A future host executor may produce the input documents, but
this boundary only reopens bounded bytes and composes a deterministic,
``PROPOSED_REVIEW_REQUIRED`` closure.  The two resolver documents model the
machine-readable equivalent of ``apt-get --print-uris``; their raw URI lines
are still parsed with a strict grammar so shell quoting, percent encoding,
SHA-256 and filename identity cannot be silently changed.

The proposal is not an offline manifest and cannot be supplied to the r3
collector until an independent custodian review document binds its exact
proposal hash.  No production profile or r2 receipt is edited by this module.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import lzma
import os
from pathlib import Path
import re
import shlex
import stat
from typing import Any, Callable, Mapping
from urllib.parse import unquote, urlsplit


SCHEMA = "glim_clean_room_r3_apt_allowlist_generation_proposal_v1"
SCHEMA_VERSION = 1
REQUIREMENTS_SCHEMA = "glim_clean_room_r3_package_requirements_v1"
RESOLVER_SCHEMA = "glim_clean_room_r3_apt_resolver_output_v1"
SOURCE_SCHEMA = "glim_clean_room_r3_apt_source_snapshot_v1"
SOURCE_SCHEMA_V2 = "glim_clean_room_r3_apt_source_snapshot_v2"
REVIEW_SCHEMA = "glim_clean_room_r3_apt_allowlist_review_v1"
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_TEXT_BYTES = 64 * 1024 * 1024
MAX_DECODED_PACKAGES_BYTES = 256 * 1024 * 1024
MAX_DEB_BYTES = 512 * 1024 * 1024
MAX_LOG_BYTES = 8 * 1024 * 1024
# This limit applies only to a size declared by a signed Release file.  It is
# deliberately separate from the materially smaller limits for downloaded
# Packages/deb/archive bytes: parsing a declaration never allocates or fetches
# that many bytes.  Ubuntu Contents indexes observed in the field are below
# this finite 16 GiB ceiling.  It applies only to signed metadata
# declarations; materialized/downloaded artifacts retain their smaller
# independent limits below.
MAX_RELEASE_DECLARED_BYTES = 16 * 1024 * 1024 * 1024
RELEASE_DECLARATION_POLICY = {
    "schema": "registration-plugin-release-declaration-size-policy-v1",
    "schema_version": 1,
    "max_declared_bytes": MAX_RELEASE_DECLARED_BYTES,
    "decimal_grammar": "0|[1-9][0-9]*",
    "zero_allowed": True,
}
EMPTY_PACKAGES_SHA256 = (
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
)
SIGNED_EMPTY_PACKAGES_POLICY = {
    "schema": "registration-plugin-signed-empty-packages-policy-v1",
    "schema_version": 1,
    "enabled": True,
    "signed_empty": True,
    "decoded_sha256": EMPTY_PACKAGES_SHA256,
    "decoded_bytes": 0,
    "allowed_formats": ["xz"],
    "xz_stream_count": 1,
    "xz_compressed_bytes": 32,
    "padding": "forbidden",
    "trailing_data": "reject",
    "binding": "signed-release-pair-compressed-bytes-and-exact-empty-decode",
}
PACKAGES_FORMAT_POLICY = {
    "schema": "registration-plugin-packages-format-policy-v1",
    "schema_version": 1,
    "selection_order": ["xz", "gz", "plain"],
    "allowed_formats": ["xz", "gz", "plain"],
    "compressed_max_bytes": MAX_TEXT_BYTES,
    "decoded_max_bytes": MAX_DECODED_PACKAGES_BYTES,
    "max_expansion_ratio": 256,
    "trailing_data": "reject",
    "concatenated_streams": "reject",
    "terminal_empty_stream_policy": {
        "schema": "registration-plugin-xz-terminal-empty-stream-policy-v1",
        "schema_version": 1,
        "enabled": True,
        "max_streams": 2,
        "compressed_bytes": 32,
        "decoded_bytes": 0,
        "padding": "forbidden",
        "binding": "index-offset-bytes-sha256-decoded-bytes",
    },
    "signed_empty_packages_policy": SIGNED_EMPTY_PACKAGES_POLICY,
}
PACKAGE_ARTIFACT_IDENTITY_DOMAIN = (
    "registration-plugin-packages-artifact-identity-v1"
)
PACKAGE_ARTIFACT_IDENTITY_FIELDS = frozenset({
    "domain", "repository_url", "suite", "component", "architecture",
    "release_kind", "release_identity", "release_url", "release_record_path",
    "release_file_sha256", "release_file_bytes", "signed_compressed_path",
    "signed_plain_path", "format", "packages_url", "compressed_sha256",
    "compressed_bytes", "decoded_sha256", "decoded_bytes", "signed_empty", "identity",
})
GPGV_STATUS_POLICY = {
    "schema": "registration-plugin-gpgv-status-policy-v1",
    "schema_version": 1,
    "gnupg_version": "2.2.27",
    "required_proof_tags": ["NEWSIG", "GOODSIG", "VALIDSIG"],
    "optional_informational_tags": [
        "KEY_CONSIDERED", "VERIFICATION_COMPLIANCE_MODE"
    ],
    "informational_pair": "independently-optional",
    "status_order": [
        "NEWSIG", "KEY_CONSIDERED", "SIG_ID", "GOODSIG", "VALIDSIG",
        "VERIFICATION_COMPLIANCE_MODE"
    ],
    "key_considered_fields": 4,
    "key_considered_fingerprint": "validsig-primary-or-signer",
    "key_considered_flags": "canonical-decimal-0..2147483647",
    "verification_compliance_mode_fields": 3,
    "verification_compliance_mode": "canonical-decimal-0..2147483647",
    "informational_only": True,
    "proof": "GOODSIG_VALIDSIG_only",
    "keyring_input": "dearmored_path_only",
}
GPG_INVENTORY_POLICY = {
    "schema": "registration-plugin-gpg-key-inventory-policy-v1",
    "schema_version": 1,
    "executable": "/usr/bin/gpg",
    "versions": ["gpg (GnuPG) 2.2.27", "gpg (GnuPG) 2.4.4"],
    "version_argv": ["/usr/bin/gpg", "--version"],
    "argv_template": [
        "/usr/bin/gpg", "--batch", "--no-options", "--no-default-keyring",
        "--no-auto-key-retrieve", "--no-auto-key-import", "--no-auto-key-locate",
        "--no-autostart", "--homedir", "<fresh-homedir>", "--with-colons",
        "--with-fingerprint", "--with-subkey-fingerprint", "--import-options",
        "show-only", "--import", "<dearmored-keyring>",
    ],
    "homedir": {
        "parent": "/workspace/evidence-parent/gpg-inventory-work",
        "parent_relative": "gpg-inventory-work",
        "prefix": "glim-clean-room-r3-gpg-inventory-", "token_hex_length": 24,
        "mode": 448, "post_absent": True, "parent_empty_before_after": True,
        "cleanup": "owned-tree-only-and-absence-required",
    },
    "environment": "fixed-locale-no-home-or-agent",
    "shell": False, "network": "none",
    "version_timeout_seconds": 30, "inventory_timeout_seconds": 120,
    "metadata_residue_schema": "registration-plugin-gpg-inventory-metadata-v1",
    "metadata_residue_policy": "exact-pubring.kbx-trustdb.gpg-supported-show-only",
    "metadata_residue_names": ["pubring.kbx", "trustdb.gpg"],
    "metadata_residue_mode": 384,
    "metadata_residue_max_bytes": 16777216,
    "metadata_residue_retention": "descriptor-only-cleaned",
    "failure_status": "REVIEW_REQUIRED",
}
PINNED_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
ROLES = ("build", "runtime")
REQUIRED_SOURCE_ROLES = frozenset({"apt_source", "apt_release", "apt_packages"})
LEGACY_SOURCE_ROLES = frozenset((*REQUIRED_SOURCE_ROLES, "rosdep_source"))
PACKAGE_KEYS = ("name", "version", "architecture")
FINGERPRINT_RE = re.compile(r"^[0-9A-Fa-f]{40}$")
APT_SOURCE_URL_POLICY = {
    "schema": "registration-plugin-apt-source-url-policy-v1",
    "schema_version": 1,
    "https": "allowed",
    "http_prefixes": [
        "http://archive.ubuntu.com/ubuntu",
        "http://packages.ros.org/ros2/ubuntu",
        "http://security.ubuntu.com/ubuntu",
    ],
    "http_default_port": 80,
    "redirect_policy": "same-host-only-and-requested-url-recorded",
}
UBUNTU_ARCHIVE_KEYRING = "/usr/share/keyrings/ubuntu-archive-keyring.gpg"
SOURCE_OPTION_FIELDS = frozenset({
    "architectures", "by-hash", "check-valid-until", "date-max-future",
    "description", "enabled", "identifier", "inrelease-path", "languages",
    "pdiffs", "snapshot", "targets", "trusted", "allow-insecure",
    "allow-weak", "allow-downgrade-to-insecure", "valid-until-min",
    "valid-until-max",
})


class ComposerError(ValueError):
    """Raised when a proposed closure is not independently reproducible."""


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
    if not isinstance(value, str) or len(value) != 64 or \
            any(char not in "0123456789abcdef" for char in value):
        raise ComposerError(f"{label} is not a lowercase SHA-256")
    return value


def _validate_package_artifact_identity(value: Any, label: str) -> dict[str, Any]:
    """Validate the complete identity of one signed Packages artifact."""
    if not isinstance(value, Mapping) or set(value) != PACKAGE_ARTIFACT_IDENTITY_FIELDS:
        raise ComposerError(f"{label} identity fields are invalid")
    if value["domain"] != PACKAGE_ARTIFACT_IDENTITY_DOMAIN or \
            value["architecture"] != "amd64" or \
            value["release_kind"] not in {"inrelease", "detached"} or \
            value["format"] not in {"xz", "gz", "plain"}:
        raise ComposerError(f"{label} identity policy is invalid")
    for field in ("release_identity", "release_file_sha256", "compressed_sha256",
                  "decoded_sha256", "identity"):
        _sha(value[field], f"{label} {field}")
    for field in ("repository_url", "release_url", "packages_url"):
        _apt_or_https(value[field], f"{label} {field}")
    for field in ("release_record_path", "signed_compressed_path",
                  "signed_plain_path"):
        _safe_relative(value[field], f"{label} {field}")
    for field in ("suite", "component"):
        if not isinstance(value[field], str) or not re.fullmatch(
                r"[A-Za-z0-9][A-Za-z0-9+_.-]{0,127}", value[field]):
            raise ComposerError(f"{label} {field} is invalid")
    if type(value["signed_empty"]) is not bool:
        raise ComposerError(f"{label} signed_empty is invalid")
    for field in ("release_file_bytes", "compressed_bytes"):
        if type(value[field]) is not int or not 0 < value[field] <= MAX_TEXT_BYTES:
            raise ComposerError(f"{label} {field} is invalid")
    if type(value["decoded_bytes"]) is not int or not 0 <= value["decoded_bytes"] <= MAX_DECODED_PACKAGES_BYTES:
        raise ComposerError(f"{label} decoded_bytes is invalid")
    if value["decoded_bytes"] == 0:
        policy = PACKAGES_FORMAT_POLICY["signed_empty_packages_policy"]
        if (value["signed_empty"] is not True or value["format"] != "xz" or
                value["decoded_sha256"] != EMPTY_PACKAGES_SHA256 or
                value["compressed_bytes"] != policy["xz_compressed_bytes"]):
            raise ComposerError(f"{label} signed empty policy is invalid")
    elif value["signed_empty"] is not False:
        raise ComposerError(f"{label} signed_empty is inconsistent")
    base = value["repository_url"].rstrip("/")
    release_name = "InRelease" if value["release_kind"] == "inrelease" else "Release"
    expected_release = f"{base}/dists/{value['suite']}/{release_name}"
    plain = f"{value['component']}/binary-amd64/Packages"
    compressed = plain if value["format"] == "plain" else \
        plain + "." + value["format"]
    expected_packages = f"{base}/dists/{value['suite']}/{compressed}"
    expected_by_hash = (
        f"{base}/dists/{value['suite']}/"
        f"{value['component']}/binary-amd64/by-hash/SHA256/"
        f"{value['compressed_sha256']}")
    if (value["release_url"] != expected_release or
            value["packages_url"] not in {expected_packages, expected_by_hash} or
            value["signed_plain_path"] != plain or
            value["signed_compressed_path"] != compressed):
        raise ComposerError(f"{label} path/URL binding is invalid")
    if value["identity"] != canonical_hash(dict(value), "identity"):
        raise ComposerError(f"{label} identity hash is invalid")
    return dict(value)


def _safe_text(value: Any, label: str, *, max_len: int = 256) -> str:
    if not isinstance(value, str) or not value or len(value) > max_len or \
            any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ComposerError(f"{label} is not safe text")
    return value


def _safe_name(value: Any, label: str) -> str:
    value = _safe_text(value, label, max_len=128)
    if value in {".", ".."} or any(char in value for char in "/\\\x00"):
        raise ComposerError(f"{label} is not a package/file name")
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or Path(value).as_posix() != value or \
            any(part in {"", ".", ".."} for part in value.split("/")):
        raise ComposerError(f"{label} is not a normalized relative path")
    return value


def _https(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("https://") or \
            any(char in value for char in "\x00\r\n"):
        raise ComposerError(f"{label} is not an HTTPS URL")
    parts = urlsplit(value)
    if not parts.netloc or parts.query or parts.fragment:
        raise ComposerError(f"{label} has an ambiguous URL form")
    return value


def _validate_apt_source_url_policy(value: Any, label: str = "APT URL policy") -> dict[str, Any]:
    if value != APT_SOURCE_URL_POLICY:
        raise ComposerError(f"{label} is not the fixed profile policy")
    return dict(APT_SOURCE_URL_POLICY)


def _source_atom(value: Any, label: str, *, maximum: int = 256) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum or \
            any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ComposerError(f"{label} is not safe source text")
    return value


def _source_signed_by(value: str, label: str) -> dict[str, Any]:
    """Return the exact key form used by the capture source parser."""
    value = value.strip()
    if value.startswith("-----BEGIN PGP PUBLIC KEY BLOCK-----"):
        lines = value.splitlines()
        if (len(lines) < 3 or lines[0] != "-----BEGIN PGP PUBLIC KEY BLOCK-----" or
                lines[-1] != "-----END PGP PUBLIC KEY BLOCK-----" or
                any("-----BEGIN PGP PUBLIC KEY BLOCK-----" in line for line in lines[1:])):
            raise ComposerError(f"{label} has malformed inline key material")
        data = ("\n".join(lines) + "\n").encode("utf-8")
        if len(data) > MAX_TEXT_BYTES:
            raise ComposerError(f"{label} inline key material is oversized")
        return {"kind": "inline", "source": "inline", "bytes": len(data),
                "sha256": sha256_bytes(data), "data": data}
    if "-----BEGIN PGP" in value or "-----END PGP" in value:
        raise ComposerError(f"{label} has malformed inline key material")
    if (not value.startswith("/") or Path(value).as_posix() != value or
            any(part in {"", ".", ".."} for part in value.split("/")[1:])):
        raise ComposerError(f"{label} keyring path is unsafe")
    return {"kind": "path", "source": value}


def _repository_family(url: str) -> str:
    parts = urlsplit(url)
    path = parts.path.rstrip("/")
    if parts.hostname in {"archive.ubuntu.com", "security.ubuntu.com"} and \
            path == "/ubuntu":
        return "ubuntu"
    if parts.hostname == "packages.ros.org" and path == "/ros2/ubuntu":
        return "ros"
    return "other"


def _resolve_source_keyring(record: Mapping[str, Any], label: str) -> dict[str, Any]:
    """Apply the same repository-scoped keyring policy as the capture tool."""
    uris = record.get("uris")
    signed_by = record.get("signed_by")
    if not isinstance(uris, list) or not uris or not isinstance(signed_by, Mapping):
        raise ComposerError(f"{label} has an invalid keyring projection")
    families = {_repository_family(url) for url in uris}
    if len(families) != 1:
        raise ComposerError(f"{label} mixes repository keyring families")
    family = next(iter(families))
    if family == "ubuntu":
        if signed_by.get("kind") == "none":
            return {"kind": "path", "source": UBUNTU_ARCHIVE_KEYRING}
        if signed_by.get("kind") != "path" or \
                signed_by.get("source") != UBUNTU_ARCHIVE_KEYRING:
            raise ComposerError(f"{label} does not use the Ubuntu archive keyring")
    elif family == "ros":
        if signed_by.get("kind") != "inline":
            raise ComposerError(f"{label} lacks the ROS inline key")
    elif signed_by.get("kind") == "none":
        raise ComposerError(f"{label} has no repository-scoped keyring")
    return dict(signed_by)


def _parse_legacy_source(value: str, label: str, policy: Mapping[str, Any]
                          ) -> dict[str, Any] | None:
    try:
        tokens = shlex.split(value, comments=True, posix=True)
    except ValueError as error:
        raise ComposerError(f"{label} has invalid shell quoting") from error
    if not tokens:
        return None
    if tokens[0] not in {"deb", "deb-src"}:
        raise ComposerError(f"{label} is not an active legacy entry")
    index = 1
    option_tokens: list[str] = []
    if index < len(tokens) and tokens[index].startswith("["):
        while index < len(tokens):
            option_tokens.append(tokens[index])
            if tokens[index].endswith("]"):
                break
            index += 1
        if not option_tokens or not option_tokens[-1].endswith("]"):
            raise ComposerError(f"{label} has unterminated options")
        index += 1
        option_text = " ".join(option_tokens)[1:-1].strip()
        option_values = option_text.split() if option_text else []
    elif index < len(tokens) and "]" in tokens[index]:
        raise ComposerError(f"{label} has malformed options")
    else:
        option_values = []
    if len(tokens) < index + 3:
        raise ComposerError(f"{label} lacks suite or components")
    options: list[dict[str, str]] = []
    option_names: set[str] = set()
    signed_by: dict[str, Any] = {"kind": "none"}
    for option in option_values:
        if "=" not in option:
            raise ComposerError(f"{label} has malformed option")
        name, option_value = option.split("=", 1)
        name = _source_atom(name.lower(), f"{label} option name", maximum=64)
        option_value = _source_atom(option_value, f"{label} option value")
        if name in option_names:
            raise ComposerError(f"{label} has duplicate option")
        if name not in SOURCE_OPTION_FIELDS and name != "signed-by":
            raise ComposerError(f"{label} has an unknown option")
        option_names.add(name)
        options.append({"name": name, "value": option_value})
        if name == "signed-by":
            signed_by = _source_signed_by(option_value, f"{label} Signed-By")
    uri = _apt_or_https(tokens[index], f"{label} URI")
    suite = _source_atom(tokens[index + 1], f"{label} suite", maximum=128)
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", suite) is None:
        raise ComposerError(f"{label} suite is invalid")
    components = tokens[index + 2:]
    if any(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+:-]{0,127}", item) is None
           for item in components):
        raise ComposerError(f"{label} component is invalid")
    return {"format": "legacy", "types": [tokens[0]], "uris": [uri],
            "suites": [suite], "components": components, "options": options,
            "signed_by": signed_by}


def _parse_deb822_source(fields: Mapping[str, str], label: str,
                         policy: Mapping[str, Any]) -> dict[str, Any]:
    required = {"types", "uris", "suites", "components"}
    if not required.issubset(fields):
        raise ComposerError(f"{label} lacks required deb822 fields")
    types = fields["types"].split()
    if not types or any(item not in {"deb", "deb-src"} for item in types) or \
            len(types) != len(set(types)):
        raise ComposerError(f"{label} has invalid Types")
    uris = [_apt_or_https(item, f"{label} URI") for item in fields["uris"].split()]
    suites = fields["suites"].split()
    components = fields["components"].split()
    if not uris or not suites or not components or len(uris) != len(set(uris)) or \
            len(suites) != len(set(suites)):
        raise ComposerError(f"{label} has duplicate or empty values")
    if any(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", item) is None
           for item in suites):
        raise ComposerError(f"{label} suite is invalid")
    if any(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+:-]{0,127}", item) is None
           for item in components):
        raise ComposerError(f"{label} component is invalid")
    options: list[dict[str, str]] = []
    for key, value in fields.items():
        if key in required or key == "signed-by":
            continue
        if key not in SOURCE_OPTION_FIELDS:
            raise ComposerError(f"{label} has unknown field {key}")
        options.append({"name": key, "value": _source_atom(
            value, f"{label} {key}", maximum=4096)})
    signed_by = _source_signed_by(fields["signed-by"], f"{label} Signed-By") \
        if "signed-by" in fields else {"kind": "none"}
    return {"format": "deb822", "types": types, "uris": uris,
            "suites": suites, "components": components, "options": options,
            "signed_by": signed_by}


def _parse_source_entries(data: bytes, label: str,
                          policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    try:
        lines = data.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise ComposerError(f"{label} is not UTF-8") from error
    deb822 = any(line and not line.lstrip().startswith("#") and
                 not line.startswith((" ", "\t", "deb ", "deb-src ")) and
                 ":" in line for line in lines)
    records: list[dict[str, Any]] = []
    if not deb822:
        for line_number, line in enumerate(lines, 1):
            record = _parse_legacy_source(line, f"{label}:{line_number}", policy)
            if record is not None:
                record["ordinal"] = len(records)
                record["line_start"] = line_number
                record["line_end"] = line_number
                records.append(record)
    else:
        fields: dict[str, str] = {}
        current_key: str | None = None
        stanza_start: int | None = None
        stanza_end: int | None = None
        for line_number, line in enumerate(lines, 1):
            if line.lstrip().startswith("#"):
                continue
            if not line.strip():
                if fields:
                    record = _parse_deb822_source(
                        fields, f"{label} stanza {len(records)}", policy)
                    record["ordinal"] = len(records)
                    record["line_start"] = stanza_start
                    record["line_end"] = stanza_end
                    records.append(record)
                    fields, current_key = {}, None
                    stanza_start = None
                    stanza_end = None
                continue
            if line[0].isspace():
                if current_key != "signed-by" or not fields:
                    raise ComposerError(f"{label}:{line_number} invalid continuation")
                continuation = line.lstrip()
                if not continuation:
                    raise ComposerError(f"{label}:{line_number} empty continuation")
                # deb822 uses a single dot to encode an empty continuation
                # line.  Normalize it before parsing inline OpenPGP armor so
                # the key identity is based on the unfolded key bytes.
                fields[current_key] += "\n" + ("" if continuation == "." else continuation)
                stanza_end = line_number
                continue
            if ":" not in line:
                raise ComposerError(f"{label}:{line_number} invalid deb822 field")
            key, field_value = line.split(":", 1)
            key = key.strip().lower()
            if re.fullmatch(r"[a-z][a-z0-9-]{0,63}", key) is None or key in fields:
                raise ComposerError(f"{label}:{line_number} duplicate/invalid field")
            if stanza_start is None:
                stanza_start = line_number
            fields[key] = field_value.strip()
            current_key = key
            stanza_end = line_number
        if fields:
            record = _parse_deb822_source(
                fields, f"{label} stanza {len(records)}", policy)
            record["ordinal"] = len(records)
            record["line_start"] = stanza_start
            record["line_end"] = stanza_end
            records.append(record)
    return records


def _http_url_matches_prefix(value: str, prefix: str) -> bool:
    try:
        actual = urlsplit(value)
        expected = urlsplit(prefix)
        port = actual.port
    except ValueError:
        return False
    if actual.scheme != "http" or expected.scheme != "http" or port not in (None, 80):
        return False
    if actual.hostname != expected.hostname or actual.query or actual.fragment:
        return False
    base_path = expected.path.rstrip("/")
    return actual.path == base_path or actual.path.startswith(base_path + "/")


def _apt_or_https(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) > 2048 or \
            any(char in value for char in "\x00\r\n"):
        raise ComposerError(f"{label} is not a valid URL")
    try:
        parts = urlsplit(value)
        netloc, query, fragment = parts.netloc, parts.query, parts.fragment
        username, password = parts.username, parts.password
    except ValueError as error:
        raise ComposerError(f"{label} has an invalid URL form") from error
    if not netloc or query or fragment or username or password:
        raise ComposerError(f"{label} has an ambiguous URL form")
    if parts.scheme == "https":
        return value
    if parts.scheme != "http" or not any(
            _http_url_matches_prefix(value, prefix)
            for prefix in APT_SOURCE_URL_POLICY["http_prefixes"]):
        raise ComposerError(f"{label} is outside the fixed APT HTTP allowlist")
    return value


def _regular_bytes(path: Path, label: str, *, max_bytes: int = MAX_TEXT_BYTES,
                   allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise ComposerError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > max_bytes or \
            (before.st_size == 0 and not allow_empty):
        raise ComposerError(f"{label} must be a bounded single-link regular file: {path}")
    try:
        with path.open("rb") as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(max_bytes + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise ComposerError(f"{label} cannot be read: {path}: {error}") from error
    if (fd_before.st_dev != before.st_dev or fd_before.st_ino != before.st_ino or
            fd_after.st_dev != before.st_dev or fd_after.st_ino != before.st_ino or
            fd_after.st_size != before.st_size or len(data) != before.st_size or
            len(data) > max_bytes or (not data and not allow_empty)):
        raise ComposerError(f"{label} changed while being read: {path}")
    return data


def _read_json(path: Path, label: str, *, max_bytes: int = MAX_JSON_BYTES) -> dict[str, Any]:
    data = _regular_bytes(path, label, max_bytes=max_bytes)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ComposerError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ComposerError(f"{label} must be a JSON object")
    return value


def _fresh_dir(path: Path, label: str) -> None:
    if not path.is_absolute() or path.exists() or path.is_symlink() or \
            not path.parent.is_dir() or path.parent.is_symlink():
        raise ComposerError(f"{label} must be a fresh absolute directory: {path}")


def _write_exclusive(path: Path, data: bytes) -> None:
    if path.exists() or path.is_symlink() or not path.parent.is_dir():
        raise ComposerError(f"output is not fresh: {path}")
    try:
        with path.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        path.chmod(0o444)
    except OSError as error:
        raise ComposerError(f"cannot seal output {path}: {error}") from error


def _identity(item: Mapping[str, Any]) -> tuple[str, str, str]:
    return tuple(str(item.get(key, "")) for key in PACKAGE_KEYS)


def _sorted_packages(value: Any, label: str, *, allow_empty: bool = False) -> list[dict[str, Any]]:
    if not isinstance(value, list) or (not value and not allow_empty):
        raise ComposerError(f"{label} must be {'possibly ' if allow_empty else ''}non-empty list")
    if any(not isinstance(item, Mapping) for item in value):
        raise ComposerError(f"{label} contains a non-object")
    result = [dict(item) for item in value]
    identities = [_identity(item) for item in result]
    if any(not all(identity) for identity in identities) or \
            len(set(identities)) != len(identities) or identities != sorted(identities):
        raise ComposerError(f"{label} is not sorted and unique")
    for item in result:
        _safe_name(item["name"], f"{label} package name")
        _safe_name(item["version"], f"{label} package version")
        _safe_name(item["architecture"], f"{label} package architecture")
    return result


def _load_collector():
    path = Path(__file__).with_name("apt_closure_collector.py")
    spec = importlib.util.spec_from_file_location("r3_collector_for_allowlist", path)
    if spec is None or spec.loader is None:
        raise ComposerError("r3 collector contract cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


COLLECTOR = _load_collector()


def _load_signature_promotion():
    path = Path(__file__).with_name("apt_signature_promotion.py")
    spec = importlib.util.spec_from_file_location("r3_signature_promotion_for_allowlist", path)
    if spec is None or spec.loader is None:
        raise ComposerError("signature promotion contract cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIGNATURE_PROMOTION = _load_signature_promotion()


def validate_requirements(value: Any) -> dict[str, Any]:
    required = {"schema", "schema_version", "status", "build", "runtime", "canonical_sha256"}
    if not isinstance(value, Mapping) or set(value) != required or \
            value.get("schema") != REQUIREMENTS_SCHEMA or value.get("schema_version") != 1 or \
            value.get("status") != "PRECOMMITTED":
        raise ComposerError("versionless package requirements schema/status is not exact")
    result = dict(value)
    for role in ROLES:
        items = result[role]
        if not isinstance(items, list) or not items:
            raise ComposerError(f"{role} requirements are empty")
        if any(not isinstance(item, Mapping) or set(item) != {"name", "architecture"}
               for item in items):
            raise ComposerError(f"{role} requirements have extra/missing fields")
        names = [(str(item["name"]), str(item["architecture"])) for item in items]
        if names != sorted(names) or len(set(names)) != len(names):
            raise ComposerError(f"{role} requirements are not sorted and unique")
        for item in items:
            _safe_name(item["name"], f"{role} requirement name")
            if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+.-]{0,127}", item["name"]) is None:
                raise ComposerError(f"{role} requirement name is not an apt package name")
            architecture = _safe_name(item["architecture"], f"{role} requirement architecture")
            if architecture not in {"amd64", "arm64", "armhf", "all"}:
                raise ComposerError(f"unsupported explicit {role} architecture: {architecture}")
    if result["canonical_sha256"] != canonical_hash(result, "canonical_sha256"):
        raise ComposerError("versionless requirements canonical identity drift")
    return result


def _parse_uri_line(line: str, label: str) -> dict[str, Any]:
    if not isinstance(line, str) or not line or "\x00" in line:
        raise ComposerError(f"{label} is empty or contains NUL")
    try:
        fields = shlex.split(line, posix=True)
    except ValueError as error:
        raise ComposerError(f"{label} has invalid shell quoting: {error}") from error
    if len(fields) != 4:
        raise ComposerError(f"{label} must have URL filename bytes digest fields")
    url, filename, byte_text, digest_text = fields
    _apt_or_https(url, f"{label} URL")
    if not filename.endswith(".deb") or "/" in filename or "\\" in filename or \
            filename in {".", ".."}:
        raise ComposerError(f"{label} filename is unsafe")
    try:
        size = int(byte_text, 10)
    except ValueError as error:
        raise ComposerError(f"{label} size is not decimal") from error
    if size <= 0 or size > MAX_DEB_BYTES:
        raise ComposerError(f"{label} size is outside bounds")
    if digest_text.startswith("SHA256:"):
        digest_algorithm = "sha256"
        digest = digest_text[7:]
        _sha(digest, f"{label} URI SHA256")
    elif digest_text.startswith("MD5Sum:"):
        digest_algorithm = "md5"
        digest = digest_text[7:]
        if not isinstance(digest, str) or len(digest) != 32 or \
                any(char not in "0123456789abcdef" for char in digest):
            raise ComposerError(f"{label} URI MD5Sum is malformed")
    else:
        raise ComposerError(f"{label} must provide SHA256 or an MD5 locator")
    path = urlsplit(url).path
    decoded_name = unquote(path.rsplit("/", 1)[-1])
    cache_parts = filename.rsplit("_", 2)
    pool_name_from_cache = filename
    if len(cache_parts) == 3:
        epoch = re.fullmatch(r"([0-9]+)%3a(.+)", cache_parts[1])
        if epoch:
            pool_name_from_cache = "{}_{}_{}".format(
                cache_parts[0], epoch.group(2), cache_parts[2])
    if decoded_name not in {filename, pool_name_from_cache} or \
            any(part in {"", ".", ".."} for part in filename.split("/")):
        raise ComposerError(f"{label} URL basename/percent encoding does not match filename")
    return {"url": url, "filename": filename, "bytes": size,
            "digest_algorithm": digest_algorithm, "digest": digest,
            "sha256": digest if digest_algorithm == "sha256" else None}


def parse_print_uris(value: Any, label: str, *, require_canonical: bool = True
                     ) -> list[dict[str, Any]]:
    if not isinstance(value, list) or not value:
        raise ComposerError(f"{label} URI lines are empty")
    result = [_parse_uri_line(line, f"{label} URI line {index}")
              for index, line in enumerate(value)]
    identities = [(item["url"], item["filename"]) for item in result]
    if len(set(identities)) != len(identities) or \
            len({item["filename"] for item in result}) != len(result):
        raise ComposerError(f"{label} URI set contains duplicates")
    if require_canonical and identities != sorted(identities):
        raise ComposerError(f"{label} URI set is not canonically sorted")
    return result


def _validate_repository(value: Any, label: str) -> dict[str, Any]:
    required = {"url", "release_url", "release_path", "release_sha256",
                "packages_url", "packages_path", "packages_sha256"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise ComposerError(f"{label} repository fields are incomplete or extra")
    result = dict(value)
    _apt_or_https(result["url"], f"{label} URL")
    _apt_or_https(result["release_url"], f"{label} Release URL")
    _safe_relative(result["release_path"], f"{label} Release path")
    _sha(result["release_sha256"], f"{label} Release SHA")
    _apt_or_https(result["packages_url"], f"{label} Packages URL")
    _safe_relative(result["packages_path"], f"{label} Packages path")
    _sha(result["packages_sha256"], f"{label} Packages SHA")
    return result


def _validate_base_status(value: Any) -> dict[tuple[str, str, str], dict[str, str]]:
    if not isinstance(value, list) or not value:
        raise ComposerError("base dpkg status records are empty")
    result: dict[tuple[str, str, str], dict[str, str]] = {}
    for item in value:
        if not isinstance(item, Mapping) or set(item) != {
                "name", "version", "architecture", "status"} or \
                item["status"] != "install ok installed":
            raise ComposerError("base dpkg status record is incomplete or not installed")
        identity = _identity(item)
        if identity in result:
            raise ComposerError("base dpkg status contains duplicate identity")
        for key in PACKAGE_KEYS:
            _safe_name(item[key], f"base dpkg {key}")
        result[identity] = dict(item)
    if list(result) != sorted(result):
        raise ComposerError("base dpkg status records are not sorted")
    return result


def _validate_base_status_file(path: Path) -> tuple[dict[tuple[str, str, str], dict[str, str]], str]:
    data = _regular_bytes(path, "base dpkg status snapshot")
    # The snapshot is a JSON projection, not a parser's mutable in-memory claim.
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ComposerError(f"base dpkg status snapshot is invalid JSON: {error}") from error
    if not isinstance(value, Mapping) or set(value) != {"schema", "schema_version",
            "status", "packages", "canonical_sha256"} or \
            value["schema"] != "glim_clean_room_r3_base_dpkg_status_v1" or \
            value["schema_version"] != 1 or value["status"] != "SEALED_PREINSTALL":
        raise ComposerError("base dpkg status snapshot schema/status is not exact")
    if value["canonical_sha256"] != canonical_hash(value, "canonical_sha256"):
        raise ComposerError("base dpkg status snapshot self-hash drift")
    return _validate_base_status(value["packages"]), sha256_bytes(data)


def _release_packages_relative_path(repository: Mapping[str, Any]) -> str:
    release = urlsplit(repository["release_url"])
    packages = urlsplit(repository["packages_url"])
    release_base = release.path.rsplit("/", 1)[0].rstrip("/") + "/"
    if release.netloc != packages.netloc or not packages.path.startswith(release_base):
        raise ComposerError("Packages URL is not under the repository Release URL")
    relative = packages.path[len(release_base):]
    return _safe_relative(relative, "Release Packages path")


def _release_payload(data: bytes) -> bytes:
    """Return the cleartext payload for an InRelease or raw Release file."""
    try:
        text = data.decode("utf-8")
    except UnicodeError as error:
        raise ComposerError("Release/InRelease is not UTF-8") from error
    if not text.startswith("-----BEGIN PGP SIGNED MESSAGE-----"):
        return data
    lines = text.splitlines()
    try:
        blank = lines.index("")
        signature = next(index for index, line in enumerate(lines[blank + 1:], blank + 1)
                         if line == "-----BEGIN PGP SIGNATURE-----")
    except (ValueError, StopIteration) as error:
        raise ComposerError("InRelease clear-signed framing is invalid") from error
    cleartext = [line[1:] if line.startswith("-") else line
                 for line in lines[blank + 1:signature]]
    return ("\n".join(cleartext) + "\n").encode("utf-8")


def _release_entries(data: bytes, label: str) -> dict[str, tuple[str, int]]:
    """Parse the signed Release SHA256 stanza without trusting local indexes."""
    try:
        lines = _release_payload(data).decode("utf-8").splitlines()
    except UnicodeError as error:
        raise ComposerError(f"{label} is not UTF-8") from error
    starts = [index for index, line in enumerate(lines) if line == "SHA256:"]
    if len(starts) != 1:
        raise ComposerError(f"{label} must contain exactly one SHA256 section")
    records: dict[str, tuple[str, int]] = {}
    for ordinal, line in enumerate(lines[starts[0] + 1:], 1):
        if not line:
            continue
        if not line[0].isspace():
            break
        fields = line.split()
        if len(fields) != 3:
            raise ComposerError(f"{label} SHA256 entry {ordinal} is malformed")
        digest, size_text, path = fields
        _sha(digest, f"{label} SHA256 digest")
        if re.fullmatch(r"(?:0|[1-9][0-9]*)", size_text) is None:
            raise ComposerError(f"{label} size is not canonical decimal")
        limit_text = str(MAX_RELEASE_DECLARED_BYTES)
        if (len(size_text) > len(limit_text) or
                (len(size_text) == len(limit_text) and size_text > limit_text)):
            raise ComposerError(f"{label} size is outside bounds")
        size = int(size_text, 10)
        if size > MAX_RELEASE_DECLARED_BYTES:
            raise ComposerError(f"{label} size is outside bounds")
        path = _safe_relative(path, f"{label} path")
        if path in records:
            raise ComposerError(f"{label} SHA256 section contains duplicate paths")
        records[path] = (digest, size)
    if not records:
        raise ComposerError(f"{label} SHA256 section is empty")
    return records


def _validate_release_packages_binding(release_data: bytes, packages_data: bytes,
                                       repository: Mapping[str, Any]) -> None:
    """Bind the Packages bytes to the Release SHA256 index.

    This proves the local bytes match the captured Release metadata.  It does
    not verify an OpenPGP signature; that outer-host/custodian operation is a
    separate, unrun receipt requirement for this candidate.
    """
    records = _release_entries(release_data, "Release/InRelease")
    expected_path = _release_packages_relative_path(repository)
    indexed = records.get(expected_path)
    if indexed is None:
        raise ComposerError("Release SHA256 section lacks the repository Packages path")
    actual = (sha256_bytes(packages_data), len(packages_data))
    if indexed != actual or repository["packages_sha256"] != actual[0]:
        raise ComposerError("Release SHA256 Packages binding drift")


def _check_no_symlink_parents(path: Path, label: str) -> None:
    current = path.parent
    while True:
        try:
            info = current.lstat()
        except OSError as error:
            raise ComposerError(f"{label} parent cannot be inspected: {current}") from error
        if stat.S_ISLNK(info.st_mode):
            raise ComposerError(f"{label} parent contains a symlink: {current}")
        if current == Path(current.anchor):
            break
        current = current.parent


def _validate_descriptor_stat(value: Any, label: str, *, absolute_path: bool,
                              maximum: int = MAX_TEXT_BYTES,
                              allow_empty: bool = False,
                              allowed_modes: frozenset[int] | None = None) -> None:
    """Validate the complete stat projection carried by a v2 source record."""
    required = {"path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                "device", "inode"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise ComposerError(f"{label} stat descriptor fields are invalid")
    path = value["path"]
    if absolute_path:
        if not isinstance(path, str) or not path.startswith("/") or \
                Path(path).as_posix() != path:
            raise ComposerError(f"{label} stat descriptor path is invalid")
    else:
        _safe_relative(path, f"{label} stat descriptor path")
    if (type(value["bytes"]) is not int or value["bytes"] < 0 or
            value["bytes"] > maximum or
            (value["bytes"] == 0 and not allow_empty)):
        raise ComposerError(f"{label} stat descriptor size is invalid")
    _sha(value["sha256"], f"{label} stat descriptor SHA")
    for field in ("mode", "uid", "gid", "nlink", "device", "inode"):
        if type(value[field]) is not int:
            raise ComposerError(f"{label} stat descriptor metadata is invalid")
    mode_is_unsafe = (value["mode"] not in allowed_modes
                      if allowed_modes is not None
                      else not 0 <= value["mode"] <= 0o7777 or
                      bool(value["mode"] & 0o022))
    if mode_is_unsafe or \
            value["uid"] < 0 or value["gid"] < 0 or value["nlink"] != 1 or \
            value["device"] <= 0 or value["inode"] <= 0:
        raise ComposerError(f"{label} stat descriptor metadata is unsafe")


def _reopen_regular_descriptor(path: Path, label: str, *, maximum: int,
                               expected_mode: int | None = None,
                               allow_empty: bool = False) -> dict[str, Any]:
    _check_no_symlink_parents(path, label)
    try:
        parent_before = path.parent.lstat()
        before = path.lstat()
    except OSError as error:
        raise ComposerError(f"{label} cannot be inspected: {path}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or stat.S_IMODE(before.st_mode) & 0o022:
        raise ComposerError(f"{label} is not a safe regular file")
    data = _regular_bytes(path, label, max_bytes=maximum,
                          allow_empty=allow_empty)
    try:
        after = path.lstat()
        parent_after = path.parent.lstat()
    except OSError as error:
        raise ComposerError(f"{label} changed while being read") from error
    before_identity = (before.st_dev, before.st_ino, before.st_size, before.st_nlink)
    after_identity = (after.st_dev, after.st_ino, after.st_size, after.st_nlink)
    if (after_identity != before_identity or len(data) != before.st_size or
            (parent_after.st_dev, parent_after.st_ino) !=
            (parent_before.st_dev, parent_before.st_ino) or
            stat.S_ISLNK(parent_after.st_mode)):
        raise ComposerError(f"{label} changed while being read")
    mode = stat.S_IMODE(before.st_mode)
    if expected_mode is not None and mode != expected_mode:
        raise ComposerError(f"{label} has an unexpected mode")
    return {"path": str(path), "bytes": len(data), "sha256": sha256_bytes(data),
            "mode": mode, "uid": before.st_uid, "gid": before.st_gid,
            "nlink": before.st_nlink, "device": before.st_dev, "inode": before.st_ino}


def _reopen_v2_source_descriptor(item: Mapping[str, Any]) -> None:
    """Reopen source bytes and stat metadata, including the fixed Humble alias."""
    descriptor = item["descriptor"]
    logical = descriptor["logical_path"]
    source = Path(descriptor["path"])
    if not str(source).endswith("/" + logical):
        raise ComposerError("apt source v2 source descriptor path drift")
    if descriptor["kind"] == "regular":
        observed = _reopen_regular_descriptor(source, "apt source v2 source",
                                               maximum=MAX_TEXT_BYTES)
    else:
        _check_no_symlink_parents(source, "apt source v2 source symlink")
        try:
            parent_before = source.parent.lstat()
            before = source.lstat()
            target_name = source.readlink()
        except OSError as error:
            raise ComposerError("apt source v2 source symlink cannot be read") from error
        if not stat.S_ISLNK(before.st_mode) or target_name != descriptor["target"]:
            raise ComposerError("apt source v2 source symlink target drift")
        target = Path(descriptor["target_descriptor"]["path"])
        if not str(target).endswith("/usr/share/ros-apt-source/ros2.sources"):
            raise ComposerError("apt source v2 source symlink target path drift")
        observed_target = _reopen_regular_descriptor(
            target, "apt source v2 source symlink target", maximum=MAX_TEXT_BYTES)
        try:
            after = source.lstat()
            parent_after = source.parent.lstat()
        except OSError as error:
            raise ComposerError("apt source v2 source symlink changed") from error
        if ((after.st_dev, after.st_ino, after.st_size, after.st_nlink) !=
                (before.st_dev, before.st_ino, before.st_size, before.st_nlink) or
                source.readlink() != target_name or
                (parent_after.st_dev, parent_after.st_ino) !=
                (parent_before.st_dev, parent_before.st_ino) or
                stat.S_ISLNK(parent_after.st_mode)):
            raise ComposerError("apt source v2 source symlink changed")
        observed = {"path": str(source), "kind": "exact_symlink",
                    "logical_path": logical, "target": target_name,
                    "target_descriptor": observed_target,
                    "bytes": observed_target["bytes"],
                    "sha256": observed_target["sha256"],
                    "mode": stat.S_IMODE(before.st_mode), "uid": before.st_uid,
                    "gid": before.st_gid, "nlink": before.st_nlink,
                    "device": before.st_dev, "inode": before.st_ino}
    if descriptor["kind"] == "regular":
        observed["kind"] = "regular"
        observed["logical_path"] = logical
    if observed != dict(descriptor):
        raise ComposerError("apt source v2 source descriptor metadata drift")


def _is_container_transport_source(descriptor: Mapping[str, Any]) -> bool:
    """Identify a source path whose absolute name belongs to the container.

    The connected helper records and stably reopens these paths before sealing
    the snapshot.  The host composer must not confuse an absolute container
    path such as ``/etc/apt/sources.list`` with the host file of the same name.
    Non-transport fixture paths remain reopenable for adversarial tests.
    """
    logical = descriptor.get("logical_path")
    source = descriptor.get("path")
    if not isinstance(logical, str) or not isinstance(source, str):
        return False
    return source == "/" + logical


def _validate_v2_package_evidence(value: Any, manifest_path: Path,
                                  label: str) -> dict[str, Any]:
    """Reopen one compressed Packages artifact and its decoded projection."""
    required = {
        "source", "url", "final_url", "format", "release_path", "release_sha256",
        "release_bytes", "bytes", "sha256", "output", "release_record_path",
        "release_file_sha256", "release_file_bytes", "source_record_path",
        "decoded_release_path", "decoded_release_sha256", "decoded_release_bytes",
        "decoded_output", "stream_descriptors", "signed_empty", "artifact_identity",
    }
    if not isinstance(value, Mapping) or set(value) != required:
        raise ComposerError(f"{label} fields are incomplete or extra")
    if value["source"] not in {"container-cache", "network"}:
        raise ComposerError(f"{label} source is invalid")
    _apt_or_https(value["url"], f"{label} URL")
    _apt_or_https(value["final_url"], f"{label} final URL")
    requested = urlsplit(value["url"])
    final = urlsplit(value["final_url"])
    if (requested.scheme, requested.hostname, requested.port) != \
            (final.scheme, final.hostname, final.port):
        raise ComposerError(f"{label} redirect changed scheme/host/port")
    fmt = value["format"]
    if fmt not in {"xz", "gz", "plain"}:
        raise ComposerError(f"{label} format is not profile-approved")
    artifact = _validate_package_artifact_identity(
        value["artifact_identity"], f"{label} artifact")
    if (type(value["signed_empty"]) is not bool or
            value["signed_empty"] != artifact["signed_empty"]):
        raise ComposerError(f"{label} signed_empty binding is invalid")
    streams = value["stream_descriptors"]
    if not isinstance(streams, list) or len(streams) > 2:
        raise ComposerError(f"{label} stream descriptors are invalid")
    for index, stream in enumerate(streams):
        if not isinstance(stream, Mapping) or set(stream) != {
                "index", "compressed_offset", "compressed_bytes",
                "compressed_sha256", "decoded_bytes"}:
            raise ComposerError(f"{label} stream descriptor fields are invalid")
        if (stream["index"] != index or type(stream["compressed_offset"]) is not int or
                type(stream["compressed_bytes"]) is not int or
                type(stream["decoded_bytes"]) is not int or
                stream["compressed_offset"] < 0 or
                stream["compressed_bytes"] <= 0 or stream["decoded_bytes"] < 0):
            raise ComposerError(f"{label} stream descriptor values are invalid")
        _sha(stream["compressed_sha256"], f"{label} stream SHA")
    if fmt != "xz" and streams:
        raise ComposerError(f"{label} non-XZ stream descriptors are invalid")
    release_path = _safe_relative(value["release_path"], f"{label} Release path")
    suffix = "" if fmt == "plain" else "." + fmt
    if not release_path.endswith("/binary-amd64/Packages" + suffix):
        raise ComposerError(f"{label} signed path has the wrong format")
    if (artifact["format"] != fmt or
            artifact["signed_compressed_path"] != release_path or
            artifact["packages_url"] != value["url"] or
            artifact["compressed_sha256"] != value["release_sha256"] or
            artifact["compressed_bytes"] != value["release_bytes"] or
            artifact["release_record_path"] != value["release_record_path"] or
            artifact["release_file_sha256"] != value["release_file_sha256"] or
            artifact["release_file_bytes"] != value["release_file_bytes"] or
            artifact["signed_plain_path"] != value["decoded_release_path"] or
            artifact["decoded_sha256"] != value["decoded_release_sha256"] or
            artifact["decoded_bytes"] != value["decoded_release_bytes"]):
        raise ComposerError(f"{label} artifact binding drift")
    for key in ("release_sha256", "sha256", "release_file_sha256",
                "decoded_release_sha256"):
        _sha(value[key], f"{label} {key}")
    for key in ("release_bytes", "bytes", "release_file_bytes"):
        if type(value[key]) is not int or not 0 < value[key] <= MAX_TEXT_BYTES:
            raise ComposerError(f"{label} byte count is invalid")
    decoded_bytes = value["decoded_release_bytes"]
    if (type(decoded_bytes) is not int or
            not 0 <= decoded_bytes <= MAX_DECODED_PACKAGES_BYTES or
            (decoded_bytes == 0) != value["signed_empty"]):
        raise ComposerError(f"{label} decoded byte count is invalid")
    if value["signed_empty"] and (
            decoded_bytes != 0 or value["decoded_release_sha256"] != EMPTY_PACKAGES_SHA256 or
            value["format"] != "xz"):
        raise ComposerError(f"{label} signed empty binding is invalid")
    for key in ("release_record_path", "source_record_path", "decoded_release_path"):
        _safe_relative(value[key], f"{label} {key}")

    output = value["output"]
    decoded_output = value["decoded_output"]
    for descriptor, prefix, descriptor_label in (
            (output, "raw-indexes/", "compressed"),
            (decoded_output, "apt-source/", "decoded")):
        if not isinstance(descriptor, Mapping):
            raise ComposerError(f"{label} {descriptor_label} descriptor is invalid")
        descriptor_maximum = (MAX_DECODED_PACKAGES_BYTES
                              if descriptor_label == "decoded"
                              else MAX_TEXT_BYTES)
        _validate_descriptor_stat(
            descriptor, f"{label} {descriptor_label}", absolute_path=False,
            maximum=descriptor_maximum, allow_empty=(
                value["signed_empty"] and descriptor_label == "decoded"))
        if not descriptor["path"].startswith(prefix):
            raise ComposerError(f"{label} {descriptor_label} path is invalid")
        observed = _reopen_regular_descriptor(
            manifest_path.parent / descriptor["path"],
            f"{label} {descriptor_label} bytes", maximum=descriptor_maximum,
            expected_mode=0o444,
            allow_empty=(value["signed_empty"] and descriptor_label == "decoded"))
        expected = dict(descriptor)
        expected["path"] = str(manifest_path.parent / descriptor["path"])
        if observed != expected:
            raise ComposerError(f"{label} {descriptor_label} descriptor drift")
    expected_raw_name = Path(artifact["signed_compressed_path"]).name + "-" + \
        artifact["identity"][:24]
    expected_decoded_name = Path(artifact["signed_plain_path"]).name + "-" + \
        artifact["identity"][:24]
    if (Path(output["path"]).name != expected_raw_name or
            Path(decoded_output["path"]).name != expected_decoded_name):
        raise ComposerError(f"{label} identity-derived output name drift")
    compressed_bytes = _regular_bytes(
        manifest_path.parent / output["path"], f"{label} compressed bytes",
        max_bytes=MAX_TEXT_BYTES)
    if fmt == "xz":
        if not streams:
            raise ComposerError(f"{label} XZ stream descriptors are missing")
        offset = 0
        observed_streams: list[dict[str, Any]] = []
        try:
            while offset < len(compressed_bytes):
                if len(observed_streams) >= 2:
                    raise ComposerError(f"{label} has too many XZ streams")
                stream_offset = offset
                decoder = lzma.LZMADecompressor(format=lzma.FORMAT_XZ)
                decoded = bytearray()
                while offset < len(compressed_bytes) and not decoder.eof:
                    block = compressed_bytes[offset:offset + 65536]
                    offset += len(block)
                    chunk = decoder.decompress(
                        block, max_length=MAX_DECODED_PACKAGES_BYTES - len(decoded) + 1)
                    decoded.extend(chunk)
                    if len(decoded) > MAX_DECODED_PACKAGES_BYTES:
                        raise ComposerError(f"{label} decoded XZ data is oversized")
                if not decoder.eof:
                    raise ComposerError(f"{label} XZ stream is truncated")
                unused = decoder.unused_data
                consumed_end = offset - len(unused)
                if consumed_end <= stream_offset:
                    raise ComposerError(f"{label} XZ stream has no progress")
                offset = consumed_end
                stream_data = compressed_bytes[stream_offset:consumed_end]
                observed_streams.append({
                    "index": len(observed_streams),
                    "compressed_offset": stream_offset,
                    "compressed_bytes": len(stream_data),
                    "compressed_sha256": sha256_bytes(stream_data),
                    "decoded_bytes": len(decoded),
                })
                if len(decoded) > max(1, len(stream_data)) * 256:
                    raise ComposerError(f"{label} XZ expansion ratio is invalid")
            if (not value["signed_empty"] and
                    observed_streams[0]["decoded_bytes"] <= 0):
                raise ComposerError(f"{label} first XZ stream is empty")
            if value["signed_empty"] and (
                    len(observed_streams) != 1 or
                    observed_streams[0]["compressed_bytes"] != 32 or
                    observed_streams[0]["decoded_bytes"] != 0 or
                    len(compressed_bytes) != 32):
                raise ComposerError(f"{label} signed empty XZ stream is invalid")
            if len(observed_streams) == 2 and (
                    observed_streams[1]["compressed_bytes"] != 32 or
                    observed_streams[1]["decoded_bytes"] != 0):
                raise ComposerError(f"{label} terminal XZ stream is not approved")
        except (EOFError, lzma.LZMAError, ValueError) as error:
            raise ComposerError(f"{label} XZ stream is invalid") from error
        if streams != observed_streams:
            raise ComposerError(f"{label} stream descriptor drift")
    elif streams:
        raise ComposerError(f"{label} stream descriptors are invalid")
    if output["bytes"] != value["bytes"] or output["sha256"] != value["sha256"]:
        raise ComposerError(f"{label} compressed byte identity drift")
    if decoded_output["bytes"] != value["decoded_release_bytes"] or \
            decoded_output["sha256"] != value["decoded_release_sha256"]:
        raise ComposerError(f"{label} decoded byte identity drift")
    return dict(value)


GPG_INVENTORY_FAILURE_KINDS = frozenset({
    "GPG_KEY_INVENTORY_FAILED", "GPG_KEY_INVENTORY_OVERSIZE",
    "GPG_KEY_INVENTORY_NOT_UTF8", "GPG_KEY_INVENTORY_UNEXPECTED_RECORD",
    "GPG_KEY_INVENTORY_KEY_INVALID", "GPG_KEY_INVENTORY_FINGERPRINT_INVALID",
    "GPG_KEY_INVENTORY_UID_INVALID", "GPG_KEY_INVENTORY_FINGERPRINT_MISSING",
    "GPG_KEY_INVENTORY_DUPLICATE", "GPG_KEY_INVENTORY_RESIDUE",
    "GPG_KEY_INVENTORY_HOMEDIR_COLLISION", "GPG_KEY_INVENTORY_HOMEDIR_INVALID",
    "GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID",
    "GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY",
    "GPG_KEY_INVENTORY_HOMEDIR_CREATE_FAILED",
    "GPG_KEY_INVENTORY_HOMEDIR_CHANGED", "GPG_KEY_INVENTORY_HOMEDIR_READ_FAILED",
    "GPG_KEY_INVENTORY_HOMEDIR_NOT_ABSENT", "GPG_KEY_INVENTORY_CLEANUP_FAILED",
    "GPG_KEY_INVENTORY_VERSION_INVALID", "GPG_KEY_INVENTORY_TOOL_INVALID",
})


def _validate_v2_inventory_command(command: Any, manifest_path: Path,
                                   dearmored_path: str, label: str) -> list[str]:
    if not isinstance(command, list) or len(command) != 17 or \
            any(not isinstance(item, str) or not item for item in command):
        raise ComposerError(f"{label} command is invalid")
    expected = [
        "/usr/bin/gpg", "--batch", "--no-options", "--no-default-keyring",
        "--no-auto-key-retrieve", "--no-auto-key-import", "--no-auto-key-locate",
        "--no-autostart", "--homedir", None, "--with-colons", "--with-fingerprint",
        "--with-subkey-fingerprint", "--import-options", "show-only", "--import", None,
    ]
    homedir = command[9]
    if re.fullmatch(
            r"/workspace/evidence-parent/gpg-inventory-work/"
            r"glim-clean-room-r3-gpg-inventory-[0-9a-f]{24}",
            homedir) is None:
        raise ComposerError(f"{label} homedir is invalid")
    expected[9] = homedir
    expected[16] = "/workspace/evidence-parent/" + dearmored_path
    if command != expected:
        raise ComposerError(f"{label} command is not the fixed isolated command")
    return list(command)


def _validate_v2_inventory_homedir(value: Any, label: str) -> dict[str, Any]:
    required = {"path", "status", "mode", "uid", "gid", "nlink", "device", "inode",
                "parent_path", "parent_relative", "post_absent", "cleanup_status"}
    if not isinstance(value, Mapping) or set(value) != required or \
            not isinstance(value["path"], str) or \
            not isinstance(value["parent_path"], str) or \
            value["parent_relative"] != "gpg-inventory-work" or \
            re.fullmatch(
                r"/.+/gpg-inventory-work/glim-clean-room-r3-gpg-inventory-[0-9a-f]{24}",
                value["path"]) is None:
        raise ComposerError(f"{label} is invalid")
    path = Path(value["path"])
    parent = Path(value["parent_path"])
    if (not parent.is_absolute() or parent.as_posix() != value["parent_path"] or
            parent.name != "gpg-inventory-work" or path.parent != parent):
        raise ComposerError(f"{label} parent is invalid")
    if value["status"] != "FRESH" or value["mode"] != 0o700 or \
            value["post_absent"] is not True or value["cleanup_status"] != "PASS":
        raise ComposerError(f"{label} status is invalid")
    for field in ("uid", "gid", "nlink", "device", "inode"):
        if type(value[field]) is not int or value[field] < 0:
            raise ComposerError(f"{label} metadata is invalid")
    if value["nlink"] < 2 or value["device"] <= 0 or value["inode"] <= 0:
        raise ComposerError(f"{label} metadata is invalid")
    return dict(value)


def _validate_v2_inventory_metadata(value: Any, identity: Mapping[str, Any],
                                    stderr: Any, label: str, *,
                                    require_exact: bool = False) -> dict[str, Any] | None:
    """Validate the pinned GnuPG metadata residue evidence.

    GnuPG's show-only import creates a small amount of keybox/trustdb metadata
    in its otherwise fresh home.  The child is removed before the outer
    receipt is reopened, so this descriptor is the retained proof of what was
    observed and removed.  It is deliberately checked here as well as by the
    capture executor: the composer is an independent source-snapshot
    readback boundary and must not accept a receipt that merely self-rehashed
    around an omitted residue projection.
    """
    if value is None:
        if require_exact:
            raise ComposerError(f"{label} metadata is missing")
        return None
    required = {"schema", "schema_version", "policy", "expected_names", "entries",
                "status", "failure_reason", "creation_evidence", "retention"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise ComposerError(f"{label} metadata fields are invalid")
    expected_names = list(GPG_INVENTORY_POLICY["metadata_residue_names"])
    if value["schema"] != GPG_INVENTORY_POLICY["metadata_residue_schema"] or \
            value["schema_version"] != 1 or \
            value["policy"] != GPG_INVENTORY_POLICY["metadata_residue_policy"] or \
            value["expected_names"] != expected_names or \
            value["retention"] != GPG_INVENTORY_POLICY["metadata_residue_retention"]:
        raise ComposerError(f"{label} metadata policy is invalid")
    evidence = value["creation_evidence"]
    if not isinstance(evidence, Mapping):
        raise ComposerError(f"{label} creation evidence is missing")
    if not isinstance(stderr, Mapping) or dict(evidence) != dict(stderr):
        raise ComposerError(f"{label} creation evidence is not bound to stderr")
    _validate_descriptor_stat(evidence, f"{label} creation evidence",
                              absolute_path=False, maximum=MAX_LOG_BYTES)
    entry_fields = {"name", "type", "mode", "uid", "gid", "nlink", "size",
                    "device", "inode", "sha256", "xattrs", "status", "reason"}
    entry_types = {"regular", "symlink", "sock" + "et", "fifo", "directory", "block",
                   "char", "other"}
    reasons = {"accepted", "unexpected_name", "type_invalid", "mode_invalid",
               "owner_invalid", "nlink_invalid", "size_invalid", "xattrs_invalid",
               "xattrs_unreadable", "hash_missing", "metadata_race", "read_failed",
               "stat_failed"}
    entries = value["entries"]
    if not isinstance(entries, list) or len(entries) > 32:
        raise ComposerError(f"{label} entries are invalid")
    names: list[str] = []
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != entry_fields:
            raise ComposerError(f"{label} entry fields are invalid")
        name = entry["name"]
        if not isinstance(name, str) or re.fullmatch(r"[^/\\\x00\r\n]+", name) is None or \
                name in names:
            raise ComposerError(f"{label} entry name is invalid")
        names.append(name)
        if entry["type"] not in entry_types or \
                entry["status"] not in {"ACCEPTED", "REJECTED"} or \
                entry["reason"] not in reasons:
            raise ComposerError(f"{label} entry classification is invalid")
        for field in ("mode", "uid", "gid", "nlink", "size", "device", "inode"):
            if type(entry[field]) is not int or entry[field] < 0:
                raise ComposerError(f"{label} entry metadata is invalid")
        if entry["mode"] > 0o7777 or \
                entry["size"] > GPG_INVENTORY_POLICY["metadata_residue_max_bytes"]:
            raise ComposerError(f"{label} entry bounds are invalid")
        if entry["sha256"] is not None:
            _sha(entry["sha256"], f"{label} entry SHA")
        xattrs = entry["xattrs"]
        if xattrs is not None and (not isinstance(xattrs, list) or
                                   xattrs != sorted(set(xattrs)) or
                                   any(not isinstance(item, str) for item in xattrs)):
            raise ComposerError(f"{label} entry xattrs are invalid")
        if entry["status"] == "ACCEPTED":
            if (name not in expected_names or entry["type"] != "regular" or
                    entry["mode"] != GPG_INVENTORY_POLICY["metadata_residue_mode"] or
                    entry["uid"] != identity["uid"] or entry["gid"] != identity["gid"] or
                    entry["nlink"] != 1 or not 0 < entry["size"] <=
                    GPG_INVENTORY_POLICY["metadata_residue_max_bytes"] or
                    entry["sha256"] is None or xattrs != [] or
                    entry["reason"] != "accepted"):
                raise ComposerError(f"{label} accepted entry is unsafe")
        elif entry["reason"] == "accepted":
            raise ComposerError(f"{label} rejected entry is misclassified")
    if names != sorted(names):
        raise ComposerError(f"{label} entries are not sorted")
    status = value["status"]
    failure_reason = value["failure_reason"]
    valid_statuses = {"EXACT", "EMPTY", "REJECTED"}
    failure_reasons = reasons | {"child_missing", "list_failed", "too_many_entries"}
    if status not in valid_statuses or \
            (failure_reason is not None and failure_reason not in failure_reasons):
        raise ComposerError(f"{label} status is invalid")
    if status in {"EXACT", "EMPTY"} and failure_reason is not None:
        raise ComposerError(f"{label} successful status has a failure reason")
    if status == "EMPTY" and entries:
        raise ComposerError(f"{label} empty status has entries")
    if status == "EXACT" and (names != expected_names or
                               any(entry["status"] != "ACCEPTED" for entry in entries)):
        raise ComposerError(f"{label} exact status is incomplete")
    if status == "REJECTED" and (failure_reason is None or
                                  failure_reason not in failure_reasons):
        raise ComposerError(f"{label} rejected status lacks a reason")
    if require_exact and status != "EXACT":
        raise ComposerError(f"{label} is not exact")
    return dict(value)


def _validate_v2_inventory_tool(value: Any, manifest_path: Path, homedir: Path,
                                label: str) -> dict[str, Any]:
    required = {"name", "path", "sha256", "bytes", "version", "version_command",
                "version_command_sha256", "version_exit_status", "version_timed_out",
                "version_signal", "version_stdout", "version_stderr", "binary_output",
                "homedir"}
    if not isinstance(value, Mapping) or set(value) != required or \
            value["name"] != "gpg" or value["path"] != "/usr/bin/gpg" or \
            value["homedir"] != str(homedir):
        raise ComposerError(f"{label} fields are invalid")
    _sha(value["sha256"], f"{label} binary SHA")
    if type(value["bytes"]) is not int or not 0 < value["bytes"] <= MAX_TEXT_BYTES or \
            value["version"] not in GPG_INVENTORY_POLICY["versions"] or \
            value["version_command"] != GPG_INVENTORY_POLICY["version_argv"] or \
            value["version_command_sha256"] != canonical_hash(value["version_command"]) or \
            value["version_exit_status"] != 0 or value["version_timed_out"] is not False or \
            value["version_signal"] is not None:
        raise ComposerError(f"{label} tool identity is invalid")
    for stream in ("version_stdout", "version_stderr"):
        descriptor = value[stream]
        if not isinstance(descriptor, Mapping) or set(descriptor) != {
                "path", "bytes", "sha256", "mode", "uid", "gid", "nlink", "device", "inode"}:
            raise ComposerError(f"{label} version log is invalid")
        _validate_descriptor_stat(descriptor, f"{label} version log", absolute_path=False,
                                  maximum=MAX_LOG_BYTES)
        observed = _reopen_regular_descriptor(
            manifest_path.parent / descriptor["path"], f"{label} version log",
            maximum=MAX_LOG_BYTES, expected_mode=0o444)
        expected = dict(descriptor)
        expected["path"] = str(manifest_path.parent / descriptor["path"])
        if observed != expected:
            raise ComposerError(f"{label} version log drift")
    binary = value["binary_output"]
    if not isinstance(binary, Mapping) or not str(binary.get("path", "")).startswith("apt-keyrings/"):
        raise ComposerError(f"{label} binary output is invalid")
    _validate_descriptor_stat(binary, f"{label} binary output", absolute_path=False,
                              maximum=MAX_TEXT_BYTES)
    observed = _reopen_regular_descriptor(
        manifest_path.parent / binary["path"], f"{label} binary output",
        maximum=MAX_TEXT_BYTES, expected_mode=0o444)
    expected = dict(binary)
    expected["path"] = str(manifest_path.parent / binary["path"])
    if observed != expected or binary["bytes"] != value["bytes"] or \
            binary["sha256"] != value["sha256"]:
        raise ComposerError(f"{label} binary output drift")
    return dict(value)


def _validate_v2_keyring_descriptor(value: Any, manifest_path: Path,
                                    label: str) -> dict[str, Any]:
    """Reopen one repository-scoped keyring and its inventory evidence."""
    common = {
        "path", "kind", "source", "bytes", "sha256",
        "normalized_armor_bytes", "normalized_armor_sha256", "dearmored_path",
        "dearmored_bytes", "dearmored_sha256", "primary_fingerprints",
        "subkey_fingerprints", "expected_signer_fingerprints",
        "signer_fingerprint", "inventory",
    }
    if not isinstance(value, Mapping) or value.get("kind") not in {"path", "inline"}:
        raise ComposerError(f"{label} descriptor is invalid")
    inventory_value = value.get("inventory")
    inventory_status = (inventory_value.get("status")
                        if isinstance(inventory_value, Mapping) else None)
    if inventory_status == "FAILED":
        required = (common - {"signer_fingerprint"}) | \
            ({"fingerprint"} if value["kind"] == "inline" else set())
    else:
        required = common | ({"fingerprint"} if value["kind"] == "inline" else set())
    if set(value) != required:
        raise ComposerError(f"{label} descriptor fields are incomplete or extra")
    path = _safe_relative(value["path"], f"{label} path")
    if not path.startswith("apt-keyrings/"):
        raise ComposerError(f"{label} path is outside apt-keyrings")
    if value["kind"] == "path":
        source = value["source"]
        if not isinstance(source, str) or not source.startswith("/") or \
                Path(source).as_posix() != source or \
                any(part in {"", ".", ".."} for part in source.split("/")[1:]):
            raise ComposerError(f"{label} source path is invalid")
        if value["normalized_armor_bytes"] is not None or \
                value["normalized_armor_sha256"] is not None:
            raise ComposerError(f"{label} path key has armor projection")
    else:
        if value["source"] != "inline" or \
                type(value["normalized_armor_bytes"]) is not int or \
                not isinstance(value["normalized_armor_sha256"], str) or \
                not isinstance(value.get("fingerprint"), str) or \
                FINGERPRINT_RE.fullmatch(value["fingerprint"]) is None:
            raise ComposerError(f"{label} inline projection is invalid")
        if value["normalized_armor_bytes"] != value["bytes"] or \
                value["normalized_armor_sha256"] != value["sha256"]:
            raise ComposerError(f"{label} inline armor identity drifts")
    for field in ("sha256", "dearmored_sha256"):
        _sha(value[field], f"{label} {field}")
    for field in ("bytes", "dearmored_bytes"):
        if type(value[field]) is not int or not 0 < value[field] <= MAX_TEXT_BYTES:
            raise ComposerError(f"{label} {field} is invalid")
    dearmored_path = _safe_relative(value["dearmored_path"],
                                    f"{label} dearmored path")
    if not dearmored_path.startswith("apt-keyrings/"):
        raise ComposerError(f"{label} dearmored path is outside apt-keyrings")
    if value["kind"] == "inline" and (
            not path.endswith(".asc") or not dearmored_path.endswith(".gpg") or
            path == dearmored_path):
        raise ComposerError(f"{label} inline keyring paths are invalid")
    if value["kind"] == "path" and (
            dearmored_path != path or value["dearmored_bytes"] != value["bytes"] or
            value["dearmored_sha256"] != value["sha256"]):
        raise ComposerError(f"{label} path dearmored identity drifts")
    for field in ("primary_fingerprints", "subkey_fingerprints",
                  "expected_signer_fingerprints"):
        values = value[field]
        if not isinstance(values, list) or values != sorted(set(values)) or \
                any(not isinstance(item, str) or FINGERPRINT_RE.fullmatch(item) is None
                    for item in values):
            raise ComposerError(f"{label} {field} is invalid")
    if not set(value["expected_signer_fingerprints"]).issubset(
            set(value["primary_fingerprints"] + value["subkey_fingerprints"])):
        raise ComposerError(f"{label} signer inventory is inconsistent")
    if inventory_status != "FAILED" and (
            value["signer_fingerprint"] not in value["expected_signer_fingerprints"] or
            FINGERPRINT_RE.fullmatch(value["signer_fingerprint"]) is None):
        raise ComposerError(f"{label} signer fingerprint is not inventoried")
    if value["kind"] == "inline" and "signer_fingerprint" in value and \
            value["fingerprint"] != value["signer_fingerprint"]:
        raise ComposerError(f"{label} inline signer fingerprint drifts")
    for relative, expected_bytes, expected_sha, suffix in (
            (path, value["bytes"], value["sha256"], "armor"),
            (dearmored_path, value["dearmored_bytes"], value["dearmored_sha256"],
             "dearmored")):
        absolute = manifest_path.parent / relative
        observed = _reopen_regular_descriptor(
            absolute, f"{label} {suffix}", maximum=MAX_TEXT_BYTES,
            expected_mode=0o444)
        if observed["bytes"] != expected_bytes or observed["sha256"] != expected_sha:
            raise ComposerError(f"{label} {suffix} byte identity drifts")
    inventory = value["inventory"]
    if not isinstance(inventory, Mapping):
        raise ComposerError(f"{label} inventory is invalid")
    if inventory.get("status") == "SYNTHETIC_TEST_ONLY":
        if set(inventory) != {"status"}:
            raise ComposerError(f"{label} synthetic inventory has extra fields")
    elif inventory.get("status") == "VERIFIED":
        required_inventory = {"status", "identity", "command", "command_sha256", "exit_status",
                              "timed_out", "signal", "stdout", "stderr", "tool",
                              "homedir", "metadata_residue"}
        if set(inventory) != required_inventory:
            raise ComposerError(f"{label} inventory fields are incomplete or extra")
        command = _validate_v2_inventory_command(
            inventory["command"], manifest_path, dearmored_path,
            f"{label} inventory")
        if inventory["command_sha256"] != canonical_hash(command) or \
                inventory["exit_status"] != 0 or inventory["timed_out"] is not False or \
                inventory["signal"] is not None:
            raise ComposerError(f"{label} inventory command is invalid")
        for stream in ("stdout", "stderr"):
            descriptor = inventory[stream]
            if not isinstance(descriptor, Mapping) or set(descriptor) != {
                    "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                    "device", "inode"}:
                raise ComposerError(f"{label} inventory log is invalid")
            stream_path = _safe_relative(descriptor["path"],
                                         f"{label} inventory log path")
            _validate_descriptor_stat(
                descriptor, f"{label} inventory log", absolute_path=False,
                maximum=MAX_LOG_BYTES)
            observed = _reopen_regular_descriptor(
                manifest_path.parent / stream_path, f"{label} inventory log",
                maximum=MAX_LOG_BYTES, expected_mode=0o444)
            expected = dict(descriptor)
            expected["path"] = str(manifest_path.parent / stream_path)
            if observed != expected:
                raise ComposerError(f"{label} inventory log descriptor drift")
        homedir = _validate_v2_inventory_homedir(
            inventory["homedir"], f"{label} inventory homedir")
        _sha(inventory["identity"], f"{label} inventory identity")
        if Path(homedir["path"]).name != \
                "glim-clean-room-r3-gpg-inventory-" + inventory["identity"][:24]:
            raise ComposerError(f"{label} inventory identity/homedir drifts")
        _validate_v2_inventory_metadata(
            inventory["metadata_residue"], homedir, inventory["stderr"],
            f"{label} inventory", require_exact=True)
        _validate_v2_inventory_tool(
            inventory["tool"], manifest_path, Path(homedir["path"]),
            f"{label} inventory tool")
    elif inventory.get("status") == "FAILED":
        required_inventory = {"status", "identity", "failure_kind", "command", "command_sha256",
                              "exit_status", "timed_out", "signal", "stdout", "stderr",
                              "tool", "homedir", "version_stdout", "version_stderr",
                              "metadata_residue"}
        if set(inventory) != required_inventory or \
                inventory["failure_kind"] not in GPG_INVENTORY_FAILURE_KINDS:
            raise ComposerError(f"{label} failed inventory fields are invalid")
        command = _validate_v2_inventory_command(
            inventory["command"], manifest_path, dearmored_path,
            f"{label} failed inventory")
        if inventory["command_sha256"] != canonical_hash(command):
            raise ComposerError(f"{label} failed inventory command hash drifts")
        if not isinstance(inventory["homedir"], Mapping):
            raise ComposerError(f"{label} failed inventory homedir is missing")
        homedir = _validate_v2_inventory_homedir(
            inventory["homedir"], f"{label} failed inventory homedir")
        _sha(inventory["identity"], f"{label} failed inventory identity")
        if Path(homedir["path"]).name != \
                "glim-clean-room-r3-gpg-inventory-" + inventory["identity"][:24]:
            raise ComposerError(f"{label} failed inventory identity/homedir drifts")
        _validate_v2_inventory_metadata(
            inventory["metadata_residue"], homedir, inventory["stderr"],
            f"{label} failed inventory")
        if inventory["tool"] is not None:
            _validate_v2_inventory_tool(
                inventory["tool"], manifest_path, Path(homedir["path"]),
                f"{label} failed inventory tool")
        for stream in ("stdout", "stderr", "version_stdout", "version_stderr"):
            descriptor = inventory[stream]
            if descriptor is None:
                continue
            if not isinstance(descriptor, Mapping) or set(descriptor) != {
                    "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                    "device", "inode"}:
                raise ComposerError(f"{label} failed inventory log is invalid")
            stream_path = _safe_relative(
                descriptor["path"], f"{label} failed inventory log path")
            _validate_descriptor_stat(
                descriptor, f"{label} failed inventory log", absolute_path=False,
                maximum=MAX_LOG_BYTES)
            observed = _reopen_regular_descriptor(
                manifest_path.parent / stream_path, f"{label} failed inventory log",
                maximum=MAX_LOG_BYTES, expected_mode=0o444)
            expected = dict(descriptor)
            expected["path"] = str(manifest_path.parent / stream_path)
            if observed != expected:
                raise ComposerError(f"{label} failed inventory log drift")
    else:
        raise ComposerError(f"{label} inventory status is not reviewable")
    return dict(value)


def _source_snapshot_urls(value: Any, *, is_v2: bool, role: str) -> list[str]:
    """Validate one canonical URL projection, including inactive v2 files."""
    if (not isinstance(value, list) or
            any(not isinstance(item, str) for item in value) or
            value != sorted(set(value))):
        raise ComposerError("source snapshot URL set is not sorted and unique")
    if not value and not (is_v2 and role == "apt_source"):
        raise ComposerError("source snapshot URL set is not sorted and unique")
    return list(value)


def _validate_v2_source_snapshot_structure(manifest: Mapping[str, Any], root: Path,
                                           manifest_path: Path, files: list[Any],
                                           policy: Mapping[str, Any] | None) -> None:
    """Reopen every v2 key and source-entry projection without collapsing it."""
    source_files: dict[str, Mapping[str, Any]] = {}
    for item in files:
        if not isinstance(item, Mapping) or item.get("role") != "apt_source":
            continue
        path = item.get("path")
        if not isinstance(path, str) or path in source_files:
            raise ComposerError("apt source v2 source file is duplicated")
        _safe_relative(path, "apt source v2 source file path")
        if set(item) != {"path", "role", "urls", "bytes", "sha256"}:
            raise ComposerError("apt source v2 source file fields are invalid")
        _source_snapshot_urls(item["urls"], is_v2=True, role="apt_source")
        if type(item["bytes"]) is not int or not 0 < item["bytes"] <= MAX_TEXT_BYTES:
            raise ComposerError("apt source v2 source file size is invalid")
        _sha(item["sha256"], "apt source v2 source file SHA")
        for url in item["urls"]:
            _apt_or_https(url, "apt source v2 source URL")
        source_files[path] = item
    if not source_files:
        raise ComposerError("apt source v2 source file set is empty")
    package_files: dict[str, Mapping[str, Any]] = {}
    for item in files:
        if not isinstance(item, Mapping) or item.get("role") != "apt_packages":
            continue
        path = item.get("path")
        if not isinstance(path, str) or path in package_files:
            raise ComposerError("apt source v2 Packages file is duplicated")
        if set(item) != {"path", "role", "url", "bytes", "sha256", "compressed"}:
            raise ComposerError("apt source v2 Packages file fields are invalid")
        _safe_relative(path, "apt source v2 Packages file path")
        _apt_or_https(item["url"], "apt source v2 Packages URL")
        signed_empty = (
            isinstance(item.get("compressed"), Mapping) and
            item["compressed"].get("signed_empty") is True)
        if type(item["bytes"]) is not int or item["bytes"] < 0 or \
                (item["bytes"] == 0 and not signed_empty) or \
                item["bytes"] > MAX_DECODED_PACKAGES_BYTES:
            raise ComposerError("apt source v2 Packages file size is invalid")
        _sha(item["sha256"], "apt source v2 Packages file SHA")
        compressed = _validate_v2_package_evidence(
            item["compressed"], manifest_path,
            "apt source v2 compressed Packages " + path)
        if compressed["source_record_path"] != path or \
                compressed["decoded_output"]["path"] != "apt-source/" + path or \
                compressed["decoded_release_sha256"] != item["sha256"] or \
                compressed["decoded_release_bytes"] != item["bytes"]:
            raise ComposerError("apt source v2 Packages source-record binding drift")
        package_files[path] = item
    if not package_files:
        raise ComposerError("apt source v2 Packages file set is empty")
    keyrings = manifest.get("keyrings")
    if not isinstance(keyrings, list) or not keyrings:
        raise ComposerError("apt source v2 keyring set is empty")
    key_map: dict[str, Mapping[str, Any]] = {}
    key_paths: list[str] = []
    keyring_refs: set[str] = set()
    for keyring in keyrings:
        keyring = _validate_v2_keyring_descriptor(
            keyring, manifest_path, "apt source v2 keyring")
        path = _safe_relative(keyring["path"], "apt source v2 keyring path")
        if not path.startswith("apt-keyrings/") or path in key_paths:
            raise ComposerError("apt source v2 keyring path is invalid or duplicated")
        kind = keyring["kind"]
        source = keyring["source"]
        if kind == "path":
            ref = "path:" + source
            if ref in key_map:
                raise ComposerError("apt source v2 keyring source is duplicated")
            key_map[ref] = keyring
            keyring_refs.add(ref)
        elif kind == "inline":
            ref = "inline:" + str(keyring["sha256"])
            if ref in key_map:
                raise ComposerError("apt source v2 inline key is duplicated")
            key_map[ref] = keyring
            keyring_refs.add(ref)
        else:
            raise ComposerError("apt source v2 keyring kind is invalid")
        key_paths.append(path)
    if key_paths != sorted(key_paths):
        raise ComposerError("apt source v2 keyrings are not sorted")
    descriptors = manifest.get("source_descriptors")
    if not isinstance(descriptors, list) or not descriptors:
        raise ComposerError("apt source v2 source descriptors are empty")
    descriptor_by_output: dict[str, Mapping[str, Any]] = {}
    descriptor_sources: set[str] = set()
    for item in descriptors:
        if not isinstance(item, Mapping) or set(item) != {"source", "descriptor", "output"}:
            raise ComposerError("apt source v2 source descriptor fields are invalid")
        source = item["source"]
        if not isinstance(source, str) or not source.startswith("/") or \
                Path(source).as_posix() != source or \
                any(part in {"", ".", ".."} for part in source.split("/")[1:]):
            raise ComposerError("apt source v2 source descriptor path is invalid")
        if source in descriptor_sources:
            raise ComposerError("apt source v2 source descriptors are duplicated")
        descriptor_sources.add(source)
        descriptor = item["descriptor"]
        output = item["output"]
        if not isinstance(descriptor, Mapping) or not isinstance(output, Mapping):
            raise ComposerError("apt source v2 source descriptor is not an object")
        if descriptor.get("path") != source or descriptor.get("kind") not in {
                "regular", "exact_symlink"}:
            raise ComposerError("apt source v2 source descriptor identity is invalid")
        if descriptor["kind"] == "regular":
            if set(descriptor) != {"path", "kind", "logical_path", "bytes", "sha256",
                                   "mode", "uid", "gid", "nlink", "device", "inode"}:
                raise ComposerError("apt source v2 regular descriptor fields are invalid")
            _validate_descriptor_stat(
                {key: descriptor[key] for key in ("path", "bytes", "sha256", "mode",
                                                   "uid", "gid", "nlink", "device", "inode")},
                source, absolute_path=True,
                allowed_modes=frozenset({0o444, 0o644}))
            _safe_relative(descriptor["logical_path"],
                           "apt source v2 regular logical path")
        else:
            if set(descriptor) != {"path", "kind", "logical_path", "target",
                                   "target_descriptor", "bytes", "sha256", "mode", "uid", "gid",
                                   "nlink", "device", "inode"} or \
                    descriptor["target"] != "/usr/share/ros-apt-source/ros2.sources":
                raise ComposerError("apt source v2 symlink descriptor is invalid")
            _validate_descriptor_stat(
                {key: descriptor[key] for key in ("path", "bytes", "sha256", "mode",
                                                   "uid", "gid", "nlink", "device", "inode")},
                source, absolute_path=True,
                allowed_modes=frozenset({0o777}))
            _safe_relative(descriptor["logical_path"],
                           "apt source v2 symlink logical path")
            target = descriptor["target_descriptor"]
            if not isinstance(target, Mapping) or target.get("kind") != "regular" or \
                    set(target) != {"path", "kind", "bytes", "sha256", "mode", "uid",
                                    "gid", "nlink", "device", "inode"} or \
                    target["nlink"] != 1:
                raise ComposerError("apt source v2 symlink target descriptor is invalid")
            _validate_descriptor_stat(
                {key: target[key] for key in ("path", "bytes", "sha256", "mode", "uid",
                                              "gid", "nlink", "device", "inode")},
                source + " target", absolute_path=True,
                allowed_modes=frozenset({0o444, 0o644}))
            if not str(target["path"]).endswith(
                    "/usr/share/ros-apt-source/ros2.sources"):
                raise ComposerError("apt source v2 symlink target path is invalid")
        if set(output) != {"path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                           "device", "inode"}:
            raise ComposerError("apt source v2 output descriptor fields are invalid")
        output_path = _safe_relative(output["path"], "apt source v2 output descriptor path")
        if not output_path.startswith("apt-source/") or descriptor.get("logical_path") != \
                output_path[len("apt-source/"):]:
            raise ComposerError("apt source v2 source descriptor logical path is invalid")
        if output_path in descriptor_by_output:
            raise ComposerError("apt source v2 source descriptors are duplicated")
        _validate_descriptor_stat(output, source + " output", absolute_path=False)
        if output["mode"] != 0o444:
            raise ComposerError("apt source v2 source output mode is invalid")
        descriptor_by_output[output_path] = item
    output_paths = list(descriptor_by_output)
    if output_paths != sorted(output_paths):
        raise ComposerError("apt source v2 source descriptors are not sorted")
    expected_outputs = {"apt-source/" + path for path in source_files}
    if set(descriptor_by_output) != expected_outputs:
        raise ComposerError("apt source v2 source descriptor set drift")
    for source_item in source_files.values():
        expected_output = "apt-source/" + source_item["path"]
        item = descriptor_by_output.get(expected_output)
        if item is None:
            raise ComposerError("apt source v2 source descriptor is missing")
        descriptor = item["descriptor"]
        output = item["output"]
        if descriptor["bytes"] != source_item["bytes"] or \
                descriptor["sha256"] != source_item["sha256"] or \
                output["bytes"] != source_item["bytes"] or \
                output["sha256"] != source_item["sha256"]:
            raise ComposerError("apt source v2 source descriptor byte identity drift")
        if descriptor["kind"] == "exact_symlink":
            target = descriptor["target_descriptor"]
            if target["bytes"] != source_item["bytes"] or \
                    target["sha256"] != source_item["sha256"]:
                raise ComposerError("apt source v2 symlink target byte identity drift")
        if not _is_container_transport_source(descriptor):
            _reopen_v2_source_descriptor(item)
        output_path = manifest_path.parent / output["path"]
        observed_output = _reopen_regular_descriptor(
            output_path, "apt source v2 output", maximum=MAX_TEXT_BYTES,
            expected_mode=0o444)
        expected_output = dict(output)
        expected_output["path"] = str(output_path)
        if observed_output != expected_output:
            raise ComposerError("apt source v2 output descriptor metadata drift")
    entries = manifest.get("source_entries")
    if not isinstance(entries, list) or not entries:
        raise ComposerError("apt source v2 entry set is empty")
    expected_entries: list[dict[str, Any]] = []
    for path in sorted(source_files):
        source_data = _regular_bytes(root / path, "apt source v2 source bytes",
                                     max_bytes=MAX_TEXT_BYTES)
        line_count = len(source_data.splitlines())
        parsed = _parse_source_entries(source_data, "apt source v2 " + path,
                                       policy or APT_SOURCE_URL_POLICY)
        for record in parsed:
            if (type(record["line_start"]) is not int or
                    type(record["line_end"]) is not int or
                    not 1 <= record["line_start"] <= record["line_end"] <= line_count):
                raise ComposerError("apt source v2 entry line provenance is invalid")
            signed_by = dict(record["signed_by"])
            for field in ("data", "dearmored_data", "normalized_armor_bytes",
                          "normalized_armor_sha256", "dearmored_bytes",
                          "dearmored_sha256"):
                signed_by.pop(field, None)
            signed_by = _resolve_source_keyring(
                {**record, "signed_by": record["signed_by"]},
                "apt source v2 " + path)
            for field in ("data", "dearmored_data", "normalized_armor_bytes",
                          "normalized_armor_sha256", "dearmored_bytes",
                          "dearmored_sha256"):
                signed_by.pop(field, None)
            if signed_by["kind"] == "path":
                reference = key_map.get("path:" + signed_by["source"])
                referenced = "path:" + signed_by["source"]
            elif signed_by["kind"] == "inline":
                reference = key_map.get("inline:" + signed_by["sha256"])
                referenced = "inline:" + signed_by["sha256"]
            else:
                reference = None
                referenced = None
            if signed_by["kind"] != "none" and reference is None:
                raise ComposerError("apt source v2 entry key binding is missing")
            if referenced is not None and referenced not in keyring_refs:
                raise ComposerError("apt source v2 entry key binding is unknown")
            if reference is not None:
                signed_by.update({"artifact_path": reference["path"],
                                  "bytes": reference["bytes"],
                                  "sha256": reference["sha256"]})
                if signed_by["kind"] == "inline":
                    signed_by["fingerprint"] = reference["fingerprint"]
            expected_entries.append({
                "path": path, "ordinal": record["ordinal"],
                "line_start": record["line_start"], "line_end": record["line_end"],
                "format": record["format"], "types": list(record["types"]),
                "uris": list(record["uris"]), "suites": list(record["suites"]),
                "components": list(record["components"]),
                "options": [dict(option) for option in record["options"]],
                "signed_by": signed_by,
            })
    observed_entries = list(entries)
    if observed_entries != sorted(observed_entries,
                                  key=lambda item: (item.get("path", ""),
                                                     item.get("ordinal", -1))):
        raise ComposerError("apt source v2 entries are not sorted")
    for entry in observed_entries:
        if not isinstance(entry, Mapping) or type(entry.get("line_start")) is not int or \
                type(entry.get("line_end")) is not int or entry["line_start"] < 1 or \
                entry["line_end"] < entry["line_start"]:
            raise ComposerError("apt source v2 entry line provenance is invalid")
    if observed_entries != expected_entries:
        raise ComposerError("apt source v2 entries omit, duplicate, or alter a stanza")
    referenced_keys = {
        ("path:" + item["signed_by"]["source"])
        if item["signed_by"]["kind"] == "path" else
        ("inline:" + item["signed_by"]["sha256"])
        for item in expected_entries if item["signed_by"]["kind"] != "none"
    }
    if referenced_keys != keyring_refs:
        raise ComposerError("apt source v2 keyring set is not exactly referenced")


def _validate_source_snapshot(root: Path, manifest_path: Path,
                              repositories: list[dict[str, Any]]) -> tuple[dict[str, Any], str]:
    try:
        root_info = root.lstat()
    except OSError as error:
        raise ComposerError(f"apt source snapshot root cannot be inspected: {root}: {error}") from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise ComposerError("apt source snapshot root must be a non-symlink directory")
    manifest = _read_json(manifest_path, "apt source snapshot manifest")
    is_v2 = manifest.get("schema") == SOURCE_SCHEMA_V2
    required = {"schema", "schema_version", "status", "files", "canonical_sha256"}
    if is_v2:
        required |= {"source_entries", "keyrings", "source_descriptors",
                     "apt_source_url_policy", "release_declaration_policy",
                     "packages_format_policy", "gpgv_status_policy"}
    allowed = required | {"repositories", "apt_source_url_policy"}
    if not required.issubset(set(manifest)) or \
            set(manifest) - allowed or \
            manifest["schema"] not in {SOURCE_SCHEMA, SOURCE_SCHEMA_V2} or \
            manifest["schema_version"] != (2 if is_v2 else 1) or \
            manifest["status"] != "SEALED_REVIEW_REQUIRED":
        raise ComposerError("apt source snapshot schema/status is not exact")
    if manifest["canonical_sha256"] != canonical_hash(manifest, "canonical_sha256"):
        raise ComposerError("apt source snapshot canonical identity drift")
    policy = manifest.get("apt_source_url_policy")
    if policy is not None:
        _validate_apt_source_url_policy(policy, "apt source snapshot URL policy")
    if is_v2 and manifest.get("release_declaration_policy") != RELEASE_DECLARATION_POLICY:
        raise ComposerError("apt source snapshot Release declaration policy is invalid")
    if is_v2 and manifest.get("packages_format_policy") != PACKAGES_FORMAT_POLICY:
        raise ComposerError("apt source snapshot Packages format policy is invalid")
    if is_v2 and manifest.get("gpgv_status_policy") != GPGV_STATUS_POLICY:
        raise ComposerError("apt source snapshot gpgv status policy is invalid")
    if "repositories" in manifest:
        repositories_value = manifest["repositories"]
        if not isinstance(repositories_value, list) or not repositories_value:
            raise ComposerError("apt source snapshot repository set is empty")
        manifest_repositories = [_validate_repository(item, "source snapshot repository")
                                 for item in repositories_value]
        repository_keys = [
            (item["url"], item["release_url"], item["packages_url"])
            for item in manifest_repositories]
        if repository_keys != sorted(repository_keys) or len(set(repository_keys)) != len(repository_keys):
            raise ComposerError("apt source snapshot repositories are not sorted and unique")
    else:
        manifest_repositories = repositories
    if policy is None and any(
            str(item[key]).startswith("http://")
            for item in manifest_repositories
            for key in ("url", "release_url", "packages_url")):
        raise ComposerError("HTTP repository URL lacks the profile policy")
    files = manifest["files"]
    if not isinstance(files, list) or not files:
        raise ComposerError("apt source snapshot files are empty")
    seen: set[str] = set()
    roles: set[str] = set()
    required_roles = REQUIRED_SOURCE_ROLES if is_v2 else LEGACY_SOURCE_ROLES
    for item in files:
        if is_v2 and isinstance(item, Mapping) and item.get("role") == "apt_source":
            expected_fields = {"path", "role", "urls", "bytes", "sha256"}
        elif is_v2 and isinstance(item, Mapping) and item.get("role") == "apt_packages":
            expected_fields = {"path", "role", "url", "bytes", "sha256", "compressed"}
        else:
            expected_fields = {"path", "role", "url", "bytes", "sha256"}
        if not isinstance(item, Mapping) or set(item) != expected_fields:
            raise ComposerError("apt source snapshot file fields are incomplete or extra")
        path = _safe_relative(item["path"], "source snapshot path")
        if path in seen:
            raise ComposerError("apt source snapshot contains duplicate paths")
        seen.add(path)
        role = _safe_name(item["role"], "source snapshot role")
        if role not in required_roles:
            raise ComposerError(f"unsupported source snapshot role: {role}")
        roles.add(role)
        urls = item["urls"] if is_v2 and role == "apt_source" else [item["url"]]
        urls = _source_snapshot_urls(urls, is_v2=is_v2, role=role)
        for url in urls:
            if url.startswith("http://") and policy is None:
                raise ComposerError("HTTP source snapshot URL lacks the profile policy")
            if role == "rosdep_source":
                _https(url, "source snapshot URL")
            else:
                _apt_or_https(url, "source snapshot URL")
        signed_empty = bool(
            is_v2 and role == "apt_packages" and
            isinstance(item.get("compressed"), Mapping) and
            item["compressed"].get("signed_empty") is True)
        maximum = (MAX_DECODED_PACKAGES_BYTES
                   if is_v2 and role == "apt_packages"
                   else MAX_TEXT_BYTES)
        if type(item["bytes"]) is not int or item["bytes"] < 0 or \
                (item["bytes"] == 0 and not signed_empty) or \
                item["bytes"] > maximum:
            raise ComposerError("source snapshot byte count is invalid")
        _sha(item["sha256"], "source snapshot SHA")
        current = root
        for component in path.split("/"):
            current = current / component
            try:
                component_info = current.lstat()
            except OSError as error:
                raise ComposerError(f"source snapshot path is missing: {current}") from error
            if current != root and stat.S_ISLNK(component_info.st_mode):
                raise ComposerError("source snapshot path contains a symlink")
        data = _regular_bytes(
            current, "source snapshot bytes", max_bytes=maximum,
            allow_empty=signed_empty)
        if len(data) != item["bytes"] or sha256_bytes(data) != item["sha256"]:
            raise ComposerError("source snapshot byte/SHA drift")
    if roles != required_roles:
        raise ComposerError("source snapshot lacks its required source roles")
    paths = [str(item["path"]) for item in files]
    if paths != sorted(paths):
        raise ComposerError("source snapshot files are not sorted")
    if is_v2:
        _validate_v2_source_snapshot_structure(
            manifest, root, Path(manifest_path), files, policy)
    release_by_path = {item["release_path"]: item for item in manifest_repositories}
    packages_by_path = {item["packages_path"]: item for item in manifest_repositories}
    for item in files:
        if item["role"] == "apt_release":
            repo = release_by_path.get(item["path"])
            if repo is None or item["url"] != repo["release_url"] or \
                    item["sha256"] != repo["release_sha256"]:
                raise ComposerError("Release byte is not bound to repository identity")
        if item["role"] == "apt_packages":
            repo = packages_by_path.get(item["path"])
            if repo is None or item["url"] != repo["packages_url"] or \
                    item["sha256"] != repo["packages_sha256"]:
                raise ComposerError("Packages index is not bound to repository identity")
            if is_v2:
                compressed = item["compressed"]
                if compressed["url"] != repo["packages_url"] or \
                        compressed["decoded_output"]["path"] != "apt-source/" + item["path"]:
                    raise ComposerError("compressed Packages is not bound to repository identity")
    for repository in manifest_repositories:
        release_path = root / repository["release_path"]
        packages_path = root / repository["packages_path"]
        release_data = _regular_bytes(release_path, "Release/InRelease bytes",
                                      max_bytes=MAX_TEXT_BYTES)
        package_item = None
        compressed = None
        if is_v2:
            package_item = next(
                (item for item in files
                 if item.get("role") == "apt_packages" and
                 item.get("path") == repository["packages_path"]), None)
            if package_item is None:
                raise ComposerError("v2 Packages source record is missing")
            compressed = package_item["compressed"]
        packages_data = _regular_bytes(packages_path, "Packages index bytes",
                                       max_bytes=MAX_DECODED_PACKAGES_BYTES,
                                       allow_empty=bool(
                                           is_v2 and compressed is not None and
                                           compressed["signed_empty"]))
        if not is_v2:
            _validate_release_packages_binding(release_data, packages_data, repository)
            continue
        entries = _release_entries(release_data, "Release/InRelease")
        # Acquire-By-Hash changes the retrieval URL, but the signed Release
        # declaration remains the canonical Packages.{xz,gz} path captured in
        # the v2 artifact.  Never derive the signed path from a by-hash URL.
        compressed_relative = _safe_relative(
            compressed["release_path"], "compressed Packages Release path")
        if entries.get(compressed_relative) != (compressed["sha256"], compressed["bytes"]):
            raise ComposerError("compressed Packages Release SHA/size binding drift")
        plain_relative = compressed["decoded_release_path"]
        suffix = "" if compressed["format"] == "plain" else "." + compressed["format"]
        if compressed_relative != plain_relative + suffix or \
                entries.get(plain_relative) != (sha256_bytes(packages_data), len(packages_data)):
            raise ComposerError("decoded Packages Release SHA/size binding drift")
    return manifest, sha256_bytes(_regular_bytes(manifest_path, "source snapshot manifest"))


def _read_packages_index(root: Path, repository: Mapping[str, Any]) -> dict[tuple[str, str, str], dict[str, Any]]:
    """Read an uncompressed Debian Packages index bound to one repository.

    The apt ``--print-uris`` output is only a locator.  Its MD5Sum (and even
    an emitted SHA256) is not accepted as the package authenticity root.  The
    downloaded bytes are checked against this Release-bound Packages index.
    """
    path = root / repository["packages_path"]
    signed_empty = repository.get("packages_sha256") == EMPTY_PACKAGES_SHA256
    data = _regular_bytes(
        path, "Packages index", max_bytes=MAX_DECODED_PACKAGES_BYTES,
        allow_empty=signed_empty)
    try:
        text = data.decode("utf-8")
    except UnicodeError as error:
        raise ComposerError("Packages index is not UTF-8") from error
    records: dict[tuple[str, str, str], dict[str, Any]] = {}
    required = {"Package", "Version", "Architecture", "Filename", "Size", "SHA256"}
    for ordinal, stanza in enumerate(re.split(r"\n\s*\n", text)):
        fields: dict[str, str] = {}
        lines = stanza.splitlines()
        for line_index, line in enumerate(lines):
            if not line:
                continue
            if line[0].isspace():
                continue
            empty_multiline = False
            if ": " in line:
                key, value = line.split(": ", 1)
            elif (line.endswith(":") and line_index + 1 < len(lines) and
                  lines[line_index + 1][:1].isspace()):
                # Debian control fields may have an empty first value when an
                # immediate continuation supplies the value (for example
                # Noble's X-Cargo-Built-Using).  Required package identity and
                # artifact fields remain non-empty and use the normal form.
                key, value = line[:-1], ""
                empty_multiline = True
            else:
                raise ComposerError(f"Packages index stanza {ordinal} has malformed field")
            if (key in fields or (not value and not empty_multiline) or
                    (empty_multiline and (
                        key in required or
                        re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9-]{0,63}", key) is None))):
                raise ComposerError(f"Packages index stanza {ordinal} has duplicate/empty field")
            fields[key] = value
        if not fields:
            continue
        if not required.issubset(fields):
            raise ComposerError(f"Packages index stanza {ordinal} lacks Filename/Size/SHA256")
        name = _safe_name(fields["Package"], "Packages package name")
        version = _safe_name(fields["Version"], "Packages package version")
        architecture = _safe_name(fields["Architecture"], "Packages package architecture")
        filename = _safe_relative(fields["Filename"], "Packages Filename")
        if not filename.endswith(".deb"):
            raise ComposerError("Packages Filename is not a deb")
        try:
            size = int(fields["Size"], 10)
        except ValueError as error:
            raise ComposerError("Packages Size is not decimal") from error
        # An index can describe packages that this capture never selects.
        # Parse those signed declarations up to the Release declaration
        # bound; selected URI/download bytes retain the stricter DEB bound.
        if size <= 0 or size > MAX_RELEASE_DECLARED_BYTES:
            raise ComposerError("Packages Size is outside bounds")
        digest = fields["SHA256"]
        _sha(digest, "Packages SHA256")
        identity = (name, version, architecture)
        if identity in records:
            raise ComposerError("Packages index contains duplicate package identity")
        records[identity] = {"name": name, "version": version,
                             "architecture": architecture, "filename": filename,
                             "bytes": size, "sha256": digest}
    if not records and not signed_empty:
        raise ComposerError("Packages index is empty")
    return records


def _exclude_download_housekeeping(
        children: list[Path], root_info: os.stat_result, role: str) -> list[Path]:
    """Validate and exclude fixed APT/restore-owner bookkeeping entries."""
    partials = [child for child in children if child.name == "partial"]
    if len(partials) > 1:
        raise ComposerError(f"{role} downloaded partial directory is duplicated")
    excluded: set[Path] = set()
    if partials:
        partial = partials[0]
        try:
            before = partial.lstat()
            entries = list(os.scandir(str(partial)))
            after = partial.lstat()
        except OSError as error:
            raise ComposerError(
                f"{role} downloaded partial directory cannot be inspected") from error
        identity_before = (before.st_dev, before.st_ino, before.st_mode,
                           before.st_uid, before.st_gid, before.st_nlink)
        identity_after = (after.st_dev, after.st_ino, after.st_mode,
                          after.st_uid, after.st_gid, after.st_nlink)
        if (not stat.S_ISDIR(before.st_mode) or stat.S_ISLNK(before.st_mode) or
                stat.S_IMODE(before.st_mode) != 0o700 or before.st_nlink != 2 or
                before.st_uid != root_info.st_uid or
                before.st_gid != root_info.st_gid or entries or
                identity_after != identity_before):
            raise ComposerError(f"{role} downloaded partial directory is unsafe")
        excluded.add(partial)

    locks = [child for child in children if child.name == "lock"]
    if len(locks) > 1:
        raise ComposerError(f"{role} downloaded lock is duplicated")
    if locks:
        lock = locks[0]
        try:
            before = lock.lstat()
            xattrs = os.listxattr(str(lock), follow_symlinks=False)
            after = lock.lstat()
        except OSError as error:
            raise ComposerError(f"{role} downloaded lock cannot be inspected") from error
        identity_before = (before.st_dev, before.st_ino, before.st_mode,
                           before.st_uid, before.st_gid, before.st_nlink,
                           before.st_size)
        identity_after = (after.st_dev, after.st_ino, after.st_mode,
                          after.st_uid, after.st_gid, after.st_nlink,
                          after.st_size)
        allowed_owners = {(0, 0), (root_info.st_uid, root_info.st_gid)}
        if (not stat.S_ISREG(before.st_mode) or stat.S_ISLNK(before.st_mode) or
                stat.S_IMODE(before.st_mode) != 0o640 or before.st_nlink != 1 or
                before.st_size != 0 or (before.st_uid, before.st_gid) not in
                allowed_owners or xattrs or identity_after != identity_before):
            raise ComposerError(f"{role} downloaded lock is unsafe")
        excluded.add(lock)
    return [child for child in children if child not in excluded]


def _validate_package_records(value: Any, label: str, *, allow_empty: bool = False) -> list[dict[str, Any]]:
    if not isinstance(value, list) or (not value and not allow_empty):
        raise ComposerError(f"{label} is empty")
    result: list[dict[str, Any]] = []
    identities: list[tuple[str, str, str]] = []
    for item in value:
        if not isinstance(item, Mapping) or set(item) != {
                "name", "version", "architecture", "filename", "url", "bytes",
                "sha256", "repository_url", "release_url", "release_path",
                "release_sha256", "packages_url", "packages_path", "packages_sha256",
                "uri_digest_algorithm", "uri_digest", "essential", "pre_depends",
                "multiarch", "base_installed"}:
            raise ComposerError(f"{label} package fields are incomplete or extra")
        item = dict(item)
        identities.append(_identity(item))
        for key in PACKAGE_KEYS:
            _safe_name(item[key], f"{label} package {key}")
        if type(item["base_installed"]) is not bool or type(item["essential"]) is not bool:
            raise ComposerError(f"{label} package flags are not boolean")
        if not isinstance(item["pre_depends"], list) or \
                any(not isinstance(dep, str) or not dep for dep in item["pre_depends"]):
            raise ComposerError(f"{label} Pre-Depends is invalid")
        if item["pre_depends"] != sorted(set(item["pre_depends"])):
            raise ComposerError(f"{label} Pre-Depends is not sorted and unique")
        _safe_name(item["multiarch"], f"{label} Multi-Arch")
        _validate_repository({"url": item["repository_url"], "release_url": item["release_url"],
                              "release_path": item["release_path"],
                              "release_sha256": item["release_sha256"],
                              "packages_url": item["packages_url"],
                              "packages_path": item["packages_path"],
                              "packages_sha256": item["packages_sha256"]}, label)
        if not isinstance(item["uri_digest"], str):
            raise ComposerError(f"{label} URI digest metadata is invalid")
        if item["base_installed"]:
            if item["uri_digest_algorithm"] != "none" or item["uri_digest"] != "":
                raise ComposerError(f"{label} base package carries a URI digest")
        elif item["uri_digest_algorithm"] == "sha256":
            _sha(item["uri_digest"], f"{label} URI SHA256")
        elif item["uri_digest_algorithm"] == "md5":
            if len(item["uri_digest"]) != 32 or \
                    any(char not in "0123456789abcdef" for char in item["uri_digest"]):
                raise ComposerError(f"{label} URI MD5Sum is invalid")
        else:
            raise ComposerError(f"{label} URI digest metadata is invalid")
        if item["base_installed"]:
            if item["filename"] != "" or item["url"] != "" or item["bytes"] != 0 or item["sha256"] != "":
                raise ComposerError(f"{label} base-installed package carries an acquired deb")
        else:
            if not item["filename"].endswith(".deb") or "/" in item["filename"] or \
                    "\\" in item["filename"]:
                raise ComposerError(f"{label} acquired filename is unsafe")
            _apt_or_https(item["url"], f"{label} package URL")
            if type(item["bytes"]) is not int or item["bytes"] <= 0 or item["bytes"] > MAX_DEB_BYTES:
                raise ComposerError(f"{label} acquired byte count is invalid")
            _sha(item["sha256"], f"{label} acquired SHA")
        result.append(item)
    if identities != sorted(identities) or len(set(identities)) != len(identities):
        raise ComposerError(f"{label} package identities are not sorted and unique")
    return result


def _validate_resolver(value: Any, role: str, requirements_sha: str,
                       base_sha: str) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    required = {"schema", "schema_version", "status", "role", "requirements_sha256",
                "base_status_sha256", "capture_mode", "raw_log_sha256", "command",
                "command_sha256", "exit_status", "uri_lines", "packages", "repositories",
                "canonical_sha256"}
    if not isinstance(value, Mapping) or set(value) != required or \
            value["schema"] != RESOLVER_SCHEMA or value["schema_version"] != 1 or \
            value["status"] != "SEALED_REVIEW_REQUIRED" or value["role"] != role or \
            value["capture_mode"] != "HOST_RAW_PRINT_URIS":
        raise ComposerError(f"{role} resolver schema/status/role is not exact")
    _sha(value["raw_log_sha256"], f"{role} raw resolver log SHA")
    if value["requirements_sha256"] != requirements_sha or value["base_status_sha256"] != base_sha:
        raise ComposerError(f"{role} resolver precommit/base identity drift")
    if type(value["exit_status"]) is not int or value["exit_status"] != 0:
        raise ComposerError(f"{role} resolver did not exit successfully")
    command = value["command"]
    if not isinstance(command, list) or not command or any(not isinstance(x, str) or not x for x in command):
        raise ComposerError(f"{role} resolver command is invalid")
    command_prefixes = (
        ["apt-get", "--print-uris", "--download-only", "--yes"],
        ["apt-get", "--quiet=2", "--print-uris", "--download-only", "--yes"],
    )
    if not any(command[:len(prefix)] == prefix for prefix in command_prefixes) or \
            "install" not in command or command.count("install") != 1 or \
            any(token in {"bash", "sh", "-c", "-lc", "curl", "wget"} for token in command):
        raise ComposerError(f"{role} resolver command is not the fixed print-uris argv")
    if value["command_sha256"] != canonical_hash(command):
        raise ComposerError(f"{role} resolver command identity drift")
    uris = parse_print_uris(value["uri_lines"], role)
    packages = _validate_package_records(value["packages"], f"{role} resolver packages")
    repos = [_validate_repository(item, f"{role} resolver repository") for item in value["repositories"]]
    repo_identities = [
        (x["url"], x["release_url"], x["packages_url"]) for x in repos]
    if not repos or repo_identities != sorted(repo_identities) or \
            len(repo_identities) != len(set(repo_identities)):
        raise ComposerError(f"{role} resolver repositories are not sorted/non-empty")
    acquired = [item for item in packages if not item["base_installed"]]
    uri_by_name = {item["filename"]: item for item in uris}
    if set(uri_by_name) != {item["filename"] for item in acquired}:
        raise ComposerError(f"{role} resolver URI set does not exactly cover acquired packages")
    repo_keys = {
        (repo["url"], repo["release_url"], repo["packages_url"]): repo
        for repo in repos}
    for package in packages:
        if package["base_installed"]:
            continue
        uri = uri_by_name[package["filename"]]
        if any(package[key] != uri[key] for key in ("filename", "url", "bytes")) or \
                package["uri_digest_algorithm"] != uri["digest_algorithm"] or \
                package["uri_digest"] != uri["digest"]:
            raise ComposerError(f"{role} URI/package identity drift")
        repo = repo_keys.get((package["repository_url"], package["release_url"],
                              package["packages_url"]))
        if repo is None or package["release_sha256"] != repo["release_sha256"] or \
                package["packages_url"] != repo["packages_url"] or \
                package["packages_path"] != repo["packages_path"] or \
                package["packages_sha256"] != repo["packages_sha256"]:
            raise ComposerError(f"{role} package Release binding drift")
    if value["canonical_sha256"] != canonical_hash(
            {key: value[key] for key in required if key != "canonical_sha256"}):
        raise ComposerError(f"{role} resolver canonical identity drift")
    return dict(value), packages, repos


def _default_dpkg_reader(path: Path) -> dict[str, str]:
    raise ComposerError(f"dpkg-deb metadata reader must be injected for fixture-only compose: {path}")


def _validate_deb_root(root: Path, packages: list[dict[str, Any]], role: str,
                       dpkg_reader: Callable[[Path], Mapping[str, str]]) -> None:
    try:
        root_info = root.lstat()
    except OSError as error:
        raise ComposerError(f"{role} downloaded deb root cannot be inspected: {error}") from error
    if not stat.S_ISDIR(root_info.st_mode) or stat.S_ISLNK(root_info.st_mode):
        raise ComposerError(f"{role} downloaded deb root is not a directory")
    acquired = [item for item in packages if not item["base_installed"]]
    expected = {item["filename"]: item for item in acquired}
    found: dict[str, Path] = {}
    children = _exclude_download_housekeeping(
        list(root.iterdir()), root_info, role)
    for child in children:
        info = child.lstat()
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ComposerError(f"{role} deb root has a symlink/hardlink/special file")
        if child.name in found or child.name not in expected:
            raise ComposerError(f"{role} deb root has an extra/duplicate file: {child.name}")
        found[child.name] = child
    if set(found) != set(expected):
        raise ComposerError(f"{role} deb root is missing an expected deb")
    for filename, package in expected.items():
        data = _regular_bytes(found[filename], f"{role} deb {filename}", max_bytes=MAX_DEB_BYTES)
        if len(data) != package["bytes"] or sha256_bytes(data) != package["sha256"]:
            raise ComposerError(f"{role} deb bytes/SHA drift: {filename}")
        try:
            metadata = dict(dpkg_reader(found[filename]))
        except Exception as error:  # injected tool boundary is fail-closed
            raise ComposerError(f"{role} dpkg-deb metadata failed: {filename}: {error}") from error
        if not set(PACKAGE_KEYS).issubset(metadata) or \
                _identity(metadata) != _identity(package):
            raise ComposerError(f"{role} dpkg-deb control identity drift: {filename}")


def seal_base_status_snapshot(*, raw_path: Path, output_path: Path) -> dict[str, Any]:
    """Convert host-captured ``dpkg-query`` TSV bytes into a sealed snapshot.

    The command that produced the raw bytes is owned by the outer host receipt;
    this helper only parses/reopens the bytes and never invokes ``dpkg-query``.
    """
    data = _regular_bytes(Path(raw_path), "raw base dpkg snapshot")
    records: list[dict[str, str]] = []
    for index, line in enumerate(data.decode("utf-8").splitlines()):
        fields = line.split("\t")
        if len(fields) != 4 or any(not field for field in fields):
            raise ComposerError(f"raw base dpkg snapshot line {index} is not four-field TSV")
        name, version, architecture, status = fields
        records.append({"name": name, "version": version, "architecture": architecture,
                        "status": status})
    records.sort(key=_identity)
    _validate_base_status(records)
    snapshot = {"schema": "glim_clean_room_r3_base_dpkg_status_v1", "schema_version": 1,
                "status": "SEALED_PREINSTALL", "packages": records}
    snapshot["canonical_sha256"] = canonical_hash(snapshot)
    output_path = Path(output_path)
    if not output_path.parent.is_dir():
        raise ComposerError("base snapshot output parent is missing")
    _write_exclusive(output_path, canonical_bytes(snapshot) + b"\n")
    return {"status": "SEALED_PREINSTALL", "file_sha256": sha256_bytes(
        _regular_bytes(output_path, "sealed base dpkg snapshot"))}


def build_resolver_output(*, role: str, raw_log_path: Path, downloaded_root: Path,
                          requirements_path: Path, base_status_path: Path,
                          source_root: Path, source_manifest_path: Path,
                          command: list[str], output_path: Path,
                          dpkg_reader: Callable[[Path], Mapping[str, Any]] = _default_dpkg_reader,
                          exit_status: int = 0) -> dict[str, Any]:
    """Build a resolver document from raw host log and downloaded bytes.

    ``resolver_output.json`` is never accepted as the source of truth here:
    URI lines, downloaded files, dpkg metadata, base status, and Release bytes
    are all reopened before the document is written.  This is fixture-only;
    no resolver/download command is executed.
    """
    if role not in ROLES:
        raise ComposerError("resolver role is not build/runtime")
    raw_data = _regular_bytes(Path(raw_log_path), f"{role} raw resolver log")
    lines = raw_data.decode("utf-8").splitlines()
    uris = parse_print_uris(lines, role, require_canonical=False)
    ordered = sorted(zip(uris, lines), key=lambda pair: (
        pair[0]["url"], pair[0]["filename"]))
    uris = [pair[0] for pair in ordered]
    lines = [pair[1] for pair in ordered]
    if type(exit_status) is not int or exit_status != 0:
        raise ComposerError(f"{role} raw resolver command did not exit successfully")
    command_prefixes = (
        ["apt-get", "--print-uris", "--download-only", "--yes"],
        ["apt-get", "--quiet=2", "--print-uris", "--download-only", "--yes"],
    )
    if not isinstance(command, list) or not any(
            command[:len(prefix)] == prefix for prefix in command_prefixes) or \
            command.count("install") != 1 or any(token in {
                "bash", "sh", "-c", "-lc", "curl", "wget"} for token in command):
        raise ComposerError(f"{role} resolver command lacks exact install argv")
    requirements = validate_requirements(_read_json(Path(requirements_path), "package requirements"))
    base, base_file_sha = _validate_base_status_file(Path(base_status_path))
    source_manifest = _read_json(Path(source_manifest_path), "apt source snapshot manifest")
    source, _ = _validate_source_snapshot(Path(source_root), Path(source_manifest_path),
                                           source_manifest.get("repositories", []))
    repositories = source.get("repositories")
    if not isinstance(repositories, list) or not repositories:
        raise ComposerError("runtime resolver generation requires explicit source repositories")
    package_indexes = {
        (repository["url"], repository["release_url"], repository["packages_url"]):
            _read_packages_index(source_root, repository)
        for repository in repositories}
    root = Path(downloaded_root)
    try:
        root_info = root.lstat()
    except OSError as error:
        raise ComposerError(f"{role} downloaded root cannot be inspected: {error}") from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise ComposerError(f"{role} downloaded root is not a non-symlink directory")
    expected = {item["filename"]: item for item in uris}
    children = _exclude_download_housekeeping(list(root.iterdir()), root_info, role)
    if len(children) != len(expected):
        raise ComposerError(f"{role} downloaded root has missing or extra debs")
    packages: list[dict[str, Any]] = []
    for child in sorted(children, key=lambda item: item.name):
        info = child.lstat()
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ComposerError(f"{role} downloaded root contains a symlink/hardlink/special file")
        uri = expected.get(child.name)
        if uri is None:
            raise ComposerError(f"{role} downloaded root has an unexpected filename")
        data = _regular_bytes(child, f"{role} downloaded deb", max_bytes=MAX_DEB_BYTES)
        if len(data) != uri["bytes"]:
            raise ComposerError(f"{role} downloaded deb byte count drift")
        try:
            metadata = dict(dpkg_reader(child))
        except Exception as error:  # injected tool boundary is fail-closed
            raise ComposerError(f"{role} dpkg-deb metadata failed: {child.name}: {error}") from error
        if not set(PACKAGE_KEYS).issubset(metadata) or any(
                not isinstance(metadata[key], str) or not metadata[key] for key in PACKAGE_KEYS):
            raise ComposerError(f"{role} dpkg-deb metadata is incomplete")
        identity = (metadata["name"], metadata["version"], metadata["architecture"])
        if identity in base:
            raise ComposerError(f"{role} downloaded package is already base-installed")
        expected_cache_name = "{}_{}_{}.deb".format(
            metadata["name"], metadata["version"].replace(":", "%3a"),
            metadata["architecture"])
        if child.name != expected_cache_name:
            raise ComposerError(f"{role} downloaded deb cache filename disagrees with dpkg identity")
        pool_basename = unquote(urlsplit(uri["url"]).path.rsplit("/", 1)[-1])
        digest = sha256_bytes(data)
        matches = []
        for candidate in repositories:
            if not uri["url"].startswith(candidate["url"].rstrip("/") + "/"):
                continue
            index = package_indexes[(candidate["url"], candidate["release_url"],
                                     candidate["packages_url"])]
            indexed_candidate = index.get(identity)
            if indexed_candidate is not None and \
                    Path(indexed_candidate["filename"]).name == pool_basename and \
                    indexed_candidate["bytes"] == len(data) and \
                    indexed_candidate["sha256"] == digest:
                matches.append((candidate, indexed_candidate))
        if not matches:
            raise ComposerError(f"{role} downloaded deb is not bound to Packages SHA256")
        matches.sort(key=lambda pair: (
            pair[0]["release_url"], pair[0]["packages_url"], pair[0]["packages_path"]))
        repository, indexed = matches[0]
        release_url = repository["release_url"]
        if uri["sha256"] is not None and uri["sha256"] != indexed["sha256"]:
            raise ComposerError(f"{role} apt URI SHA256 disagrees with Packages index")
        package = {
            "name": metadata["name"], "version": metadata["version"],
            "architecture": metadata["architecture"], "filename": child.name,
            "url": uri["url"], "bytes": uri["bytes"], "sha256": indexed["sha256"],
            "repository_url": repository["url"], "release_url": release_url,
            "release_path": repository["release_path"],
            "release_sha256": repository["release_sha256"],
            "packages_url": repository["packages_url"],
            "packages_path": repository["packages_path"],
            "packages_sha256": repository["packages_sha256"],
            "uri_digest_algorithm": uri["digest_algorithm"],
            "uri_digest": uri["digest"],
            "essential": bool(metadata.get("essential", False)),
            "pre_depends": sorted(set(metadata.get("pre_depends", []))),
            "multiarch": str(metadata.get("multiarch", "same")),
            "base_installed": False,
        }
        packages.append(package)
    packages.sort(key=_identity)
    resolver = {
        "schema": RESOLVER_SCHEMA, "schema_version": 1,
        "status": "SEALED_REVIEW_REQUIRED", "role": role,
        "requirements_sha256": requirements["canonical_sha256"],
        "base_status_sha256": base_file_sha,
        "capture_mode": "HOST_RAW_PRINT_URIS", "raw_log_sha256": sha256_bytes(raw_data),
        "command": list(command), "command_sha256": canonical_hash(command),
        "exit_status": 0, "uri_lines": lines, "packages": packages,
        "repositories": repositories,
    }
    resolver["canonical_sha256"] = canonical_hash(resolver)
    _validate_resolver(resolver, role, requirements["canonical_sha256"], base_file_sha)
    output_path = Path(output_path)
    if not output_path.parent.is_dir():
        raise ComposerError("resolver output parent is missing")
    _write_exclusive(output_path, canonical_bytes(resolver) + b"\n")
    return {"status": "SEALED_REVIEW_REQUIRED", "role": role,
            "resolver_sha256": sha256_bytes(_regular_bytes(output_path, "resolver output")),
            "raw_log_sha256": resolver["raw_log_sha256"]}


def _source_entries(manifest: Mapping[str, Any], role: str) -> list[dict[str, Any]]:
    return [dict(item) for item in manifest["files"] if item["role"] == role]


def _build_allowlist(packages_by_role: Mapping[str, list[dict[str, Any]]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for role in ROLES:
        acquired = [dict(item) for item in packages_by_role[role] if not item["base_installed"]]
        result[role] = [{key: item[key] for key in PACKAGE_KEYS} for item in acquired]
        if not result[role]:
            raise ComposerError(f"{role} has no newly acquired package; collector materialization is ambiguous")
        result[role].sort(key=lambda item: _identity(item))
    identities = {tuple(item[key] for key in PACKAGE_KEYS) for role in ROLES for item in result[role]}
    result["union"] = [dict(zip(PACKAGE_KEYS, identity)) for identity in sorted(identities)]
    result["canonical_sha256"] = canonical_hash(result)
    COLLECTOR._validate_allowlist(result)
    return result


def _build_ledger(packages_by_role: Mapping[str, list[dict[str, Any]]],
                  repositories: list[dict[str, Any]], source: Mapping[str, Any]) -> dict[str, Any]:
    entries_by_identity: dict[tuple[str, str, str], dict[str, Any]] = {}
    for role in ROLES:
        for item in packages_by_role[role]:
            if item["base_installed"]:
                continue
            entry = {key: item[key] for key in (
                "name", "version", "architecture", "filename", "url", "bytes", "sha256",
                "repository_url", "release_url", "release_sha256", "packages_url",
                "packages_path", "packages_sha256")}
            identity = _identity(entry)
            prior = entries_by_identity.get(identity)
            if prior is not None and prior != entry:
                raise ComposerError("build/runtime package identity has conflicting ledger bytes")
            entries_by_identity[identity] = entry
    entries = [entries_by_identity[key] for key in sorted(entries_by_identity)]
    repo_projection = [dict(item) for item in repositories]
    repo_projection.sort(key=lambda item: (
        item["url"], item["release_url"], item["packages_url"]))
    ledger = {
        "schema": "glim_clean_room_r3_apt_acquisition_ledger_v1",
        "schema_version": 1,
        "status": "SEALED_REVIEW_REQUIRED",
        "entries": entries,
        "repositories": repo_projection,
        "apt_source_urls": [],
        # rosdep source/cache evidence is owned by the post-disconnect
        # rosdep_prepare receipt, never by the APT source snapshot ledger.
        "rosdep_source_urls": [
            {"path": item["path"], "url": item["url"]}
            for item in _source_entries(source, "rosdep_source")
        ] if source.get("schema") != SOURCE_SCHEMA_V2 else [],
    }
    if source.get("schema") == SOURCE_SCHEMA_V2:
        apt_sources = _source_entries(source, "apt_source")
        ledger["source_manifest_schema"] = SOURCE_SCHEMA_V2
        ledger["source_manifest_schema_version"] = 2
        ledger["packages_format_policy"] = dict(PACKAGES_FORMAT_POLICY)
        ledger["source_entries"] = [dict(item) for item in source["source_entries"]]
        ledger["apt_source_urls"] = [
            {"path": item["path"], "urls": list(item["urls"])}
            for item in apt_sources]
    else:
        ledger["apt_source_urls"] = [
            {"path": item["path"], "url": item["url"]}
            for item in _source_entries(source, "apt_source")]
    ledger["canonical_sha256"] = canonical_hash(ledger)
    COLLECTOR._validate_ledger(ledger)
    return ledger


def _signature_requirement(repositories: list[dict[str, Any]]) -> dict[str, Any]:
    """Describe the receipt gate without claiming that it has been met."""
    keys = []
    for repository in repositories:
        keys.append({key: repository[key] for key in (
            "url", "release_url", "release_path", "release_sha256",
            "packages_url", "packages_path", "packages_sha256")})
    keys.sort(key=lambda item: (
        item["url"], item["release_url"], item["packages_url"]))
    return {
        "status": "REQUIRED_NOT_PROVEN",
        "receipt_index_schema": SIGNATURE_PROMOTION.INDEX_SCHEMA,
        "receipt_index_required": True,
        "repository_keys": keys,
        "runtime": "NOT_RUN",
        "promotion": "FORBIDDEN_UNTIL_HOST_AUTHORIZED_RECEIPTS",
    }


def _check_resolved_requirements(requirements: Mapping[str, Any], packages: list[dict[str, Any]],
                                 base: Mapping[tuple[str, str, str], Mapping[str, str]], role: str) -> None:
    resolved = {_identity(item) for item in packages}
    for item in requirements[role]:
        matches = [identity for identity in (*resolved, *base)
                   if identity[0] == item["name"] and
                   identity[2] in {item["architecture"], "all"}]
        if not matches:
            raise ComposerError(
                f"{role} top-level requirement is absent from resolved/base set: "
                f"{item['name']}")
    for item in packages:
        identity = _identity(item)
        if item["base_installed"] and identity not in base:
            raise ComposerError(f"{role} marks a package base-installed but it is absent from base status")
        known_names = {candidate[0] for candidate in resolved} | {candidate[0] for candidate in base}
        for dependency in item["pre_depends"]:
            dependency_name = dependency.split("(", 1)[0].split(":", 1)[0].strip()
            if dependency_name not in known_names:
                raise ComposerError(f"{role} Pre-Depends package is absent from resolved/base set")


def _check_cross_role(packages_by_role: Mapping[str, list[dict[str, Any]]]) -> None:
    versions: dict[tuple[str, str], str] = {}
    for role in ROLES:
        for item in packages_by_role[role]:
            key = (item["name"], item["architecture"])
            prior = versions.setdefault(key, item["version"])
            if prior != item["version"]:
                raise ComposerError("build/runtime resolve different versions for one name/architecture")


def compose_proposal(*, requirements_path: Path, base_status_path: Path,
                     build_resolver_path: Path, runtime_resolver_path: Path,
                     build_resolver_log_path: Path, runtime_resolver_log_path: Path,
                     source_root: Path, source_manifest_path: Path,
                     build_deb_root: Path, runtime_deb_root: Path,
                     output_root: Path,
                     dpkg_reader: Callable[[Path], Mapping[str, str]] = _default_dpkg_reader) -> dict[str, Any]:
    """Compose a fresh proposal from already materialized fixture bytes.

    No input is copied blindly: every source, resolver, Release and deb is
    reopened and hashed before any output is sealed.
    """
    _fresh_dir(Path(output_root), "proposal output root")
    requirements = validate_requirements(_read_json(Path(requirements_path), "package requirements"))
    requirements_file_sha = sha256_bytes(_regular_bytes(Path(requirements_path), "package requirements"))
    base, base_file_sha = _validate_base_status_file(Path(base_status_path))
    build_doc = _read_json(Path(build_resolver_path), "build resolver output")
    runtime_doc = _read_json(Path(runtime_resolver_path), "runtime resolver output")
    build, build_packages, build_repos = _validate_resolver(
        build_doc, "build", requirements["canonical_sha256"], base_file_sha)
    runtime, runtime_packages, runtime_repos = _validate_resolver(
        runtime_doc, "runtime", requirements["canonical_sha256"], base_file_sha)
    build_log_sha = sha256_bytes(_regular_bytes(Path(build_resolver_log_path),
                                                "build raw resolver log"))
    runtime_log_sha = sha256_bytes(_regular_bytes(Path(runtime_resolver_log_path),
                                                  "runtime raw resolver log"))
    if build["raw_log_sha256"] != build_log_sha or runtime["raw_log_sha256"] != runtime_log_sha:
        raise ComposerError("resolver document is not bound to the exact raw host log")
    packages_by_role = {"build": build_packages, "runtime": runtime_packages}
    _check_resolved_requirements(requirements, build_packages, base, "build")
    _check_resolved_requirements(requirements, runtime_packages, base, "runtime")
    _check_cross_role(packages_by_role)
    if {(repo["url"], repo["release_url"], repo["release_sha256"]) for repo in build_repos} != \
            {(repo["url"], repo["release_url"], repo["release_sha256"]) for repo in runtime_repos}:
        raise ComposerError("build/runtime repository closure differs")
    source, source_file_sha = _validate_source_snapshot(Path(source_root), Path(source_manifest_path), build_repos)
    _validate_deb_root(Path(build_deb_root), build_packages, "build", dpkg_reader)
    _validate_deb_root(Path(runtime_deb_root), runtime_packages, "runtime", dpkg_reader)
    allowlist = _build_allowlist(packages_by_role)
    ledger = _build_ledger(packages_by_role, build_repos, source)
    signature_requirement = _signature_requirement(build_repos)
    role_bindings = []
    for role in ROLES:
        for item in packages_by_role[role]:
            role_bindings.append({"role": role, "identity": list(_identity(item)),
                                  "base_installed": item["base_installed"]})
    role_bindings.sort(key=lambda item: (tuple(item["identity"]), item["role"]))
    resolver_hashes = {"build": sha256_bytes(_regular_bytes(Path(build_resolver_path), "build resolver")),
                      "runtime": sha256_bytes(_regular_bytes(Path(runtime_resolver_path), "runtime resolver"))}
    proposal = {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "status": "PROPOSED_REVIEW_REQUIRED", "benchmark_eligible": False,
        "production_manifest_modified": False, "active_r2_modified": False,
        "custodian_review": {"status": "UNSIGNED_PLACEHOLDER", "required": True,
                              "signature": ""},
        "pinned_base_image": PINNED_IMAGE,
        "requirements": {"path": Path(requirements_path).name,
                          "file_sha256": requirements_file_sha,
                          "canonical_sha256": requirements["canonical_sha256"]},
        "base_status": {"path": Path(base_status_path).name, "file_sha256": base_file_sha,
                         "package_set_sha256": canonical_hash([base[key] for key in sorted(base)])},
        "resolver_outputs": resolver_hashes,
        "resolver_logs": {"build": build_log_sha, "runtime": runtime_log_sha},
        "source_snapshot": {"manifest_path": Path(source_manifest_path).name,
                             "manifest_file_sha256": source_file_sha,
                             "canonical_sha256": source["canonical_sha256"]},
        "package_allowlist": allowlist,
        "apt_acquisition_ledger": ledger,
        "role_bindings": role_bindings,
        "signature_verification": signature_requirement,
        "closure_identity_sha256": canonical_hash({
            "requirements": requirements["canonical_sha256"],
            "base_status": base_file_sha,
            "resolver_outputs": resolver_hashes,
            "resolver_logs": {"build": build_log_sha, "runtime": runtime_log_sha},
            "source_snapshot": source["canonical_sha256"],
            "package_allowlist": allowlist["canonical_sha256"],
            "apt_acquisition_ledger": ledger["canonical_sha256"],
            "signature_verification": signature_requirement,
            "role_bindings": role_bindings}),
        "materialization": {"collector_input": False,
                            "requires_review_status": "REVIEWED_FOR_COLLECTOR",
                            "network_used": "NOT_RUN",
                            "capture_mode": "FIXTURE_ONLY_NOT_RUNTIME_VALIDATED"},
    }
    proposal["canonical_sha256"] = canonical_hash(proposal)
    output_root = Path(output_root)
    output_root.mkdir(mode=0o700)
    data = canonical_bytes(proposal) + b"\n"
    _write_exclusive(output_root / "apt-allowlist-generation-proposal.json", data)
    _write_exclusive(output_root / "apt-allowlist-generation-proposal.json.sha256",
                     (sha256_bytes(data) + "  apt-allowlist-generation-proposal.json\n").encode())
    allow_data = canonical_bytes(allowlist) + b"\n"
    _write_exclusive(output_root / "package-allowlist.json", allow_data)
    _write_exclusive(output_root / "package-allowlist.json.sha256",
                     (sha256_bytes(allow_data) + "  package-allowlist.json\n").encode())
    ledger_data = canonical_bytes(ledger) + b"\n"
    _write_exclusive(output_root / "apt-acquisition-ledger.json", ledger_data)
    _write_exclusive(output_root / "apt-acquisition-ledger.json.sha256",
                     (sha256_bytes(ledger_data) + "  apt-acquisition-ledger.json\n").encode())
    return {"status": proposal["status"], "benchmark_eligible": False,
            "proposal_sha256": sha256_bytes(data),
            "closure_identity_sha256": proposal["closure_identity_sha256"],
            "output_root": str(output_root)}


def validate_proposal(root: Path) -> dict[str, Any]:
    root = Path(root)
    proposal_path = root / "apt-allowlist-generation-proposal.json"
    data = _regular_bytes(proposal_path, "sealed proposal", max_bytes=MAX_JSON_BYTES)
    proposal = json.loads(data.decode("utf-8"))
    if not isinstance(proposal, dict) or proposal.get("schema") != SCHEMA or \
            proposal.get("status") != "PROPOSED_REVIEW_REQUIRED" or \
            proposal.get("benchmark_eligible") is not False or \
            proposal.get("canonical_sha256") != canonical_hash(proposal, "canonical_sha256"):
        raise ComposerError("proposal status/eligibility/canonical identity is invalid")
    allowlist = proposal.get("package_allowlist")
    ledger = proposal.get("apt_acquisition_ledger")
    if not isinstance(allowlist, Mapping) or not isinstance(ledger, Mapping):
        raise ComposerError("proposal closure projections are missing")
    COLLECTOR._validate_allowlist(allowlist)
    COLLECTOR._validate_ledger(ledger)
    repositories = ledger.get("repositories")
    expected_signature = _signature_requirement(repositories)
    if proposal.get("signature_verification") != expected_signature:
        raise ComposerError("proposal signature receipt requirement is missing or drifted")
    role_bindings = proposal.get("role_bindings")
    if not isinstance(role_bindings, list) or not role_bindings or \
            role_bindings != sorted(role_bindings, key=lambda item: (tuple(item.get("identity", [])),
                                                                      item.get("role", ""))):
        raise ComposerError("proposal role bindings are not sorted")
    expected_closure = canonical_hash({
        "requirements": proposal["requirements"]["canonical_sha256"],
        "base_status": proposal["base_status"]["file_sha256"],
        "resolver_outputs": proposal["resolver_outputs"],
        "resolver_logs": proposal["resolver_logs"],
        "source_snapshot": proposal["source_snapshot"]["canonical_sha256"],
        "package_allowlist": allowlist["canonical_sha256"],
        "apt_acquisition_ledger": ledger["canonical_sha256"],
        "signature_verification": expected_signature,
        "role_bindings": role_bindings})
    if proposal.get("closure_identity_sha256") != expected_closure:
        raise ComposerError("proposal closure identity drift")
    sidecar = _regular_bytes(root / "apt-allowlist-generation-proposal.json.sha256", "proposal sidecar",
                             max_bytes=512).decode("ascii").strip()
    if sidecar != f"{sha256_bytes(data)}  apt-allowlist-generation-proposal.json":
        raise ComposerError("proposal sidecar identity drift")
    for filename, expected in (("package-allowlist.json", proposal["package_allowlist"]),
                               ("apt-acquisition-ledger.json", proposal["apt_acquisition_ledger"])):
        child = _regular_bytes(root / filename, f"proposal {filename}")
        if json.loads(child.decode("utf-8")) != expected:
            raise ComposerError(f"proposal {filename} content drift")
        side = _regular_bytes(root / (filename + ".sha256"), f"proposal {filename} sidecar",
                              max_bytes=512).decode("ascii").strip()
        if side != f"{sha256_bytes(child)}  {filename}":
            raise ComposerError(f"proposal {filename} sidecar drift")
    return {"status": "PASS", "proposal_sha256": sha256_bytes(data),
            "benchmark_eligible": False}


def materialize_reviewed(*, proposal_root: Path, review_path: Path,
                         output_root: Path) -> dict[str, Any]:
    """Materialize collector inputs only after a strong per-repository receipt gate.

    A non-empty custodian signature is not a substitute for host evidence.  The
    review must reference an immutable receipt index; every repository entry is
    source/plan/Release/Packages/deb revalidated by the executor's strong
    validator and then required to be an authorized host result.  The current
    candidate has no such result, so this function intentionally remains
    fail-closed.
    """
    proposal_result = validate_proposal(Path(proposal_root))
    review = _read_json(Path(review_path), "custodian review")
    required = {"schema", "schema_version", "status", "proposal_sha256", "custodian_key_id",
                "signature", "signature_receipt_index_path", "signature_receipt_index_sha256",
                "canonical_sha256"}
    if set(review) != required or review["schema"] != REVIEW_SCHEMA or \
            review["schema_version"] != 1 or review["status"] != "REVIEWED_FOR_COLLECTOR" or \
            not isinstance(review["signature"], str) or not review["signature"]:
        raise ComposerError("collector materialization requires a non-empty independent review")
    if review["proposal_sha256"] != proposal_result["proposal_sha256"] or \
            review["canonical_sha256"] != canonical_hash(review, "canonical_sha256"):
        raise ComposerError("custodian review is not bound to the exact proposal")
    _safe_name(review["custodian_key_id"], "custodian key id")
    index_path_text = review["signature_receipt_index_path"]
    if not isinstance(index_path_text, str) or not index_path_text.startswith("/") or \
            Path(index_path_text) != Path(os.path.normpath(index_path_text)):
        raise ComposerError("signature receipt index path is not normalized absolute")
    index_path = Path(index_path_text)
    index_data = _regular_bytes(index_path, "signature receipt index", max_bytes=MAX_JSON_BYTES)
    _sha(review["signature_receipt_index_sha256"], "signature receipt index SHA")
    if sha256_bytes(index_data) != review["signature_receipt_index_sha256"]:
        raise ComposerError("signature receipt index byte identity drift")
    ledger = _read_json(Path(proposal_root) / "apt-acquisition-ledger.json",
                        "proposal acquisition ledger")
    try:
        SIGNATURE_PROMOTION.validate_receipt_index(
            index_path, expected_repositories=ledger["repositories"], ledger=ledger,
            proposal_sha256=proposal_result["proposal_sha256"], require_promotable=True)
    except Exception as error:
        raise ComposerError(f"strong per-repository signature receipt gate rejected: {error}") from error
    _fresh_dir(Path(output_root), "materialized collector output")
    output_root = Path(output_root)
    output_root.mkdir(mode=0o700)
    for filename in ("package-allowlist.json", "apt-acquisition-ledger.json"):
        data = _regular_bytes(Path(proposal_root) / filename, f"reviewed {filename}")
        _write_exclusive(output_root / filename, data)
        _write_exclusive(output_root / (filename + ".sha256"),
                         (sha256_bytes(data) + f"  {filename}\n").encode())
    _write_exclusive(output_root / SIGNATURE_PROMOTION.INDEX_NAME, index_data)
    _write_exclusive(output_root / (SIGNATURE_PROMOTION.INDEX_NAME + ".sha256"),
                     (sha256_bytes(index_data) + f"  {SIGNATURE_PROMOTION.INDEX_NAME}\n").encode())
    return {"status": "REVIEWED_FOR_COLLECTOR", "benchmark_eligible": False,
            "proposal_sha256": proposal_result["proposal_sha256"],
            "signature_receipt_index_sha256": sha256_bytes(index_data),
            "output_root": str(output_root)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("parse-uri-lines")
    check.add_argument("--input", type=Path, required=True)
    validate = sub.add_parser("validate-proposal")
    validate.add_argument("--root", type=Path, required=True)
    build = sub.add_parser("build-resolver")
    build.add_argument("--role", choices=ROLES, required=True)
    build.add_argument("--raw-log", type=Path, required=True)
    build.add_argument("--downloaded-root", type=Path, required=True)
    build.add_argument("--requirements", type=Path, required=True)
    build.add_argument("--base", type=Path, required=True)
    build.add_argument("--source-root", type=Path, required=True)
    build.add_argument("--source-manifest", type=Path, required=True)
    build.add_argument("--command-json", type=Path, required=True)
    build.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "parse-uri-lines":
            lines = _regular_bytes(args.input, "URI lines").decode("utf-8").splitlines()
            print(json.dumps(parse_print_uris(lines, "resolver"), sort_keys=True))
        elif args.command == "validate-proposal":
            print(json.dumps(validate_proposal(args.root), sort_keys=True))
        else:
            command_value = json.loads(_regular_bytes(args.command_json, "resolver command JSON")
                                       .decode("utf-8"))
            if not isinstance(command_value, list) or \
                    any(not isinstance(token, str) or not token for token in command_value):
                raise ComposerError("resolver command JSON must be a non-empty argv list")
            print(json.dumps(build_resolver_output(
                role=args.role, raw_log_path=args.raw_log,
                downloaded_root=args.downloaded_root, requirements_path=args.requirements,
                base_status_path=args.base, source_root=args.source_root,
                source_manifest_path=args.source_manifest, command=command_value,
                output_path=args.output, dpkg_reader=COLLECTOR.read_deb_metadata), sort_keys=True))
        return 0
    except (ComposerError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
