#!/usr/bin/env python3
"""Filesystem-only candidate for the GLIM r3 apt/deb collector.

The candidate is intentionally independent of Docker, apt and the network.
It reads a fixed container filesystem layout after provisioning has completed,
revalidates the immutable acquisition ledger, parses the installed dpkg
status set, invokes only the fixed ``dpkg-deb -f`` metadata command, and
seals a review-only payload.  It is not runtime-validated in a real image;
the production closure therefore remains NOT_READY.

The output is consumed by :mod:`apt_closure_capture`: ``seal_to_capture``
reopens this output and then uses the existing capture/outer binding.  Tests
inject only a bounded ``dpkg-deb`` result function; no package manager,
network, Docker or benchmark is run by the test suite.
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
import subprocess
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit


SCHEMA = "glim_clean_room_r3_apt_closure_collector_inner_v1"
SCHEMA_VERSION = 1
RECEIPT_NAME = "apt-closure.collector.receipt.json"
PAYLOAD_NAME = "apt-closure.collector.payload.json"
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_ARTIFACT_BYTES = 512 * 1024 * 1024
MAX_DPKG_DEB_SECONDS = 10
CAPTURE_MODE = "IMPLEMENTATION_CANDIDATE_NOT_RUNTIME_VALIDATED"
PINNED_BASE_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
DPKG_DEB_COMMAND = ("dpkg-deb", "-f")
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
SOURCE_SCHEMA_V2 = "glim_clean_room_r3_apt_source_snapshot_v2"
FINGERPRINT_RE = re.compile(r"^[0-9A-Fa-f]{40}$")
PACKAGES_FORMAT_POLICY = {
    "schema": "registration-plugin-packages-format-policy-v1",
    "schema_version": 1,
    "selection_order": ["xz", "gz", "plain"],
    "allowed_formats": ["xz", "gz", "plain"],
    "compressed_max_bytes": 64 * 1024 * 1024,
    "decoded_max_bytes": 256 * 1024 * 1024,
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
    "signed_empty_packages_policy": {
        "schema": "registration-plugin-signed-empty-packages-policy-v1",
        "schema_version": 1,
        "enabled": True,
        "signed_empty": True,
        "decoded_sha256": (
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        ),
        "decoded_bytes": 0,
        "allowed_formats": ["xz"],
        "xz_stream_count": 1,
        "xz_compressed_bytes": 32,
        "padding": "forbidden",
        "trailing_data": "reject",
        "binding": "signed-release-pair-compressed-bytes-and-exact-empty-decode",
    },
}
LEDGER_SOURCE_FIELDS = {
    "path", "ordinal", "format", "types", "uris", "suites", "components",
    "options", "signed_by", "line_start", "line_end",
}


class CollectorError(ValueError):
    """Raised when a candidate collector input is not independently exact."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if isinstance(value, Mapping):
        value = dict(value)
        if excluded is not None:
            value.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _load_capture():
    path = Path(__file__).with_name("apt_closure_capture.py")
    spec = importlib.util.spec_from_file_location("r3_capture_for_collector", path)
    if spec is None or spec.loader is None:
        raise CollectorError("capture verifier cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAPTURE = _load_capture()


def _load_signature_promotion():
    path = Path(__file__).with_name("apt_signature_promotion.py")
    spec = importlib.util.spec_from_file_location("r3_signature_promotion_for_collector", path)
    if spec is None or spec.loader is None:
        raise CollectorError("signature promotion contract cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIGNATURE_PROMOTION = _load_signature_promotion()


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or any(part in ("", ".", "..") for part in value.split("/")) or \
            Path(value).as_posix() != value:
        raise CollectorError(f"{label} is not a safe relative path")
    return value


def _safe_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 128 or \
            any(char not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.:-"
                for char in value):
        raise CollectorError(f"{label} is not a safe identifier")
    return value


def _sha_value(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or \
            any(char not in "0123456789abcdef" for char in value):
        raise CollectorError(f"{label} is not a lowercase SHA-256")
    return value


def _url(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("https://") or \
            any(char in value for char in ("\x00", "\r", "\n")):
        raise CollectorError(f"{label} is not an HTTPS URL")
    return value


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
        raise CollectorError(f"{label} is not a valid URL")
    try:
        parts = urlsplit(value)
        netloc, query, fragment = parts.netloc, parts.query, parts.fragment
        username, password = parts.username, parts.password
    except ValueError as error:
        raise CollectorError(f"{label} has an invalid URL form") from error
    if not netloc or query or fragment or username or password:
        raise CollectorError(f"{label} has an ambiguous URL form")
    if parts.scheme == "https":
        return value
    if parts.scheme != "http" or not any(
            _http_url_matches_prefix(value, prefix)
            for prefix in APT_SOURCE_URL_POLICY["http_prefixes"]):
        raise CollectorError(f"{label} is outside the fixed APT HTTP allowlist")
    return value


def _regular_bytes(path: Path, label: str, *, max_bytes: int = MAX_ARTIFACT_BYTES,
                   allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise CollectorError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > max_bytes or \
            (not allow_empty and before.st_size == 0):
        raise CollectorError(f"{label} must be a bounded single-link regular file: {path}")
    try:
        with path.open("rb") as stream:
            fd_info = os.fstat(stream.fileno())
            data = stream.read(max_bytes + 1)
            after = os.fstat(stream.fileno())
    except OSError as error:
        raise CollectorError(f"{label} cannot be read: {path}: {error}") from error
    if fd_info.st_ino != before.st_ino or fd_info.st_dev != before.st_dev or \
            after.st_ino != before.st_ino or after.st_dev != before.st_dev or \
            len(data) != before.st_size or after.st_size != before.st_size or \
            len(data) > max_bytes or (not allow_empty and not data):
        raise CollectorError(f"{label} changed while being read: {path}")
    return data


def _directory(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise CollectorError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise CollectorError(f"{label} must be a non-symlink directory: {path}")


def _join(root: Path, relative: str, label: str) -> Path:
    relative = _safe_relative(relative, label)
    _directory(root, "container root")
    current = root
    for component in relative.split("/"):
        current = current / component
        try:
            info = current.lstat()
        except OSError as error:
            raise CollectorError(f"{label} is missing: {current}: {error}") from error
        if current != root and stat.S_ISLNK(info.st_mode):
            raise CollectorError(f"{label} contains a symlink: {relative}")
        if component != relative.split("/")[-1] and not stat.S_ISDIR(info.st_mode):
            raise CollectorError(f"{label} contains a non-directory parent: {relative}")
    return current


def _walk_files(root: Path, relative: str, label: str) -> list[str]:
    directory = _join(root, relative, label)
    _directory(directory, label)
    found: list[str] = []
    pending = [directory]
    while pending:
        current = pending.pop()
        for child in sorted(current.iterdir(), key=lambda item: item.name):
            info = child.lstat()
            child_rel = child.relative_to(root).as_posix()
            if stat.S_ISLNK(info.st_mode):
                raise CollectorError(f"{label} contains a symlink: {child_rel}")
            if stat.S_ISDIR(info.st_mode):
                pending.append(child)
            elif stat.S_ISREG(info.st_mode) and info.st_nlink == 1:
                found.append(child_rel)
            else:
                raise CollectorError(f"{label} contains a special or hard-linked file: {child_rel}")
    if not found:
        raise CollectorError(f"{label} is empty")
    return sorted(found)


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        data = _regular_bytes(path, label, max_bytes=MAX_JSON_BYTES)
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CollectorError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise CollectorError(f"{label} must be a JSON object")
    return value


def _write_exclusive(path: Path, data: bytes) -> None:
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink():
        raise CollectorError(f"collector output is not fresh: {path}")
    try:
        with path.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        path.chmod(0o444)
    except OSError as error:
        raise CollectorError(f"cannot seal collector output {path}: {error}") from error


def _validate_allowlist(value: Any) -> dict[str, Any]:
    try:
        return CAPTURE._validate_allowlist(value)
    except (AttributeError, ValueError) as error:
        raise CollectorError(f"package allowlist is invalid: {error}") from error


def _parse_status(path: Path, label: str) -> dict[tuple[str, str, str], dict[str, str]]:
    data = _regular_bytes(path, label, max_bytes=MAX_JSON_BYTES).decode("utf-8")
    records: dict[tuple[str, str, str], dict[str, str]] = {}
    for paragraph in data.split("\n\n"):
        fields: dict[str, str] = {}
        for line in paragraph.splitlines():
            if not line or line[0].isspace() or ":" not in line:
                continue
            key, value = line.split(":", 1)
            fields[key] = value.lstrip()
        if not fields:
            continue
        required = ("Package", "Version", "Architecture", "Status")
        if any(not fields.get(key) for key in required):
            raise CollectorError(f"{label} contains an incomplete dpkg record")
        if fields["Status"] != "install ok installed":
            continue
        identity = (fields["Package"], fields["Version"], fields["Architecture"])
        if identity in records:
            raise CollectorError(f"{label} contains a duplicate package: {identity[0]}")
        records[identity] = {
            "name": identity[0], "version": identity[1], "architecture": identity[2],
            "status": fields["Status"],
        }
    if not records:
        raise CollectorError(f"{label} has no installed package records")
    return records


def _validate_ledger(value: Any) -> dict[str, Any]:
    required = {"schema", "schema_version", "status", "entries", "repositories",
                "apt_source_urls", "rosdep_source_urls", "canonical_sha256"}
    is_v2 = isinstance(value, Mapping) and value.get("source_manifest_schema") == SOURCE_SCHEMA_V2
    expected_fields = required | {"source_manifest_schema", "source_manifest_schema_version",
                                  "source_entries", "packages_format_policy"} if is_v2 else required
    if not isinstance(value, Mapping) or set(value) != expected_fields or \
            value.get("schema") != "glim_clean_room_r3_apt_acquisition_ledger_v1" or \
            value.get("schema_version") != 1 or \
            value.get("status") != "SEALED_REVIEW_REQUIRED":
        raise CollectorError("acquisition ledger schema/status is not exact")
    if is_v2 and value.get("source_manifest_schema_version") != 2:
        raise CollectorError("acquisition ledger source manifest version is not exact")
    if is_v2 and value.get("packages_format_policy") != PACKAGES_FORMAT_POLICY:
        raise CollectorError("acquisition ledger Packages format policy is not exact")
    entries = value["entries"]
    if not isinstance(entries, list):
        raise CollectorError("acquisition ledger entries are missing")
    entry_keys = ("name", "version", "architecture")
    identities = [tuple(str(item.get(key, "")) for key in entry_keys)
                  for item in entries if isinstance(item, Mapping)]
    if len(identities) != len(entries) or any(not all(identity) for identity in identities) or \
            len(set(identities)) != len(identities) or identities != sorted(identities):
        raise CollectorError("acquisition ledger entries are not sorted and unique")
    for entry in entries:
        if set(entry) != {"name", "version", "architecture", "filename", "url", "bytes",
                          "sha256", "repository_url", "release_url", "release_sha256",
                          "packages_url", "packages_path", "packages_sha256"}:
            raise CollectorError("acquisition ledger entry fields are incomplete or extra")
        if not isinstance(entry["filename"], str) or not entry["filename"].endswith(".deb") or \
                "/" in entry["filename"] or "\\" in entry["filename"]:
            raise CollectorError("acquisition ledger filename is unsafe")
        _apt_or_https(entry["url"], "acquisition ledger package URL")
        _apt_or_https(entry["repository_url"], "acquisition ledger repository URL")
        _apt_or_https(entry["release_url"], "acquisition ledger Release URL")
        _sha_value(entry["sha256"], "acquisition ledger package SHA")
        _sha_value(entry["release_sha256"], "acquisition ledger Release SHA")
        _apt_or_https(entry["packages_url"], "acquisition ledger Packages URL")
        _safe_relative(entry["packages_path"], "acquisition ledger Packages path")
        _sha_value(entry["packages_sha256"], "acquisition ledger Packages SHA")
        if type(entry["bytes"]) is not int or entry["bytes"] <= 0:
            raise CollectorError("acquisition ledger package size is invalid")
    repositories = value["repositories"]
    if not isinstance(repositories, list) or not repositories:
        raise CollectorError("acquisition ledger repository set is empty")
    repo_keys = [(str(item.get("url", "")), str(item.get("release_url", "")),
                  str(item.get("packages_url", "")))
                 for item in repositories if isinstance(item, Mapping)]
    if len(repo_keys) != len(repositories) or len(set(repo_keys)) != len(repo_keys) or \
            repo_keys != sorted(repo_keys):
        raise CollectorError("acquisition ledger repositories are not sorted and unique")
    for repo in repositories:
        if set(repo) != {"url", "release_url", "release_path", "release_sha256",
                         "packages_url", "packages_path", "packages_sha256"}:
            raise CollectorError("acquisition ledger repository fields are incomplete or extra")
        _apt_or_https(repo["url"], "repository URL")
        _apt_or_https(repo["release_url"], "repository Release URL")
        _safe_relative(repo["release_path"], "repository Release path")
        _sha_value(repo["release_sha256"], "repository Release SHA")
        _apt_or_https(repo["packages_url"], "repository Packages URL")
        _safe_relative(repo["packages_path"], "repository Packages path")
        _sha_value(repo["packages_sha256"], "repository Packages SHA")
    repository_keys = {
        (item["url"], item["release_url"], item["packages_url"]): item
        for item in repositories}
    for entry in entries:
        repository = repository_keys.get((
            entry["repository_url"], entry["release_url"], entry["packages_url"]))
        if repository is None or entry["release_sha256"] != repository["release_sha256"] or \
                entry["packages_url"] != repository["packages_url"] or \
                entry["packages_path"] != repository["packages_path"] or \
                entry["packages_sha256"] != repository["packages_sha256"]:
            raise CollectorError("acquisition ledger package repository/Release binding drift")
    for field, label in (("apt_source_urls", "apt source URL set"),
                         ("rosdep_source_urls", "rosdep source URL set")):
        values = value[field]
        allow_empty = is_v2 and field == "rosdep_source_urls"
        if not isinstance(values, list) or (not values and not allow_empty):
            raise CollectorError(f"{label} is empty")
        paths = [item.get("path") for item in values if isinstance(item, Mapping)]
        if len(paths) != len(values) or any(not isinstance(path, str) for path in paths) or \
                paths != sorted(paths) or len(set(paths)) != len(paths):
            raise CollectorError(f"{label} is not sorted and unique")
        for item in values:
            if is_v2 and field == "apt_source_urls":
                if set(item) != {"path", "urls"} or not isinstance(item["urls"], list) or \
                        item["urls"] != sorted(set(item["urls"])):
                    raise CollectorError(f"{label} has incomplete or extra fields")
                _safe_relative(item["path"], f"{label} path")
                for url in item["urls"]:
                    _apt_or_https(url, f"{label} URL")
            else:
                if set(item) != {"path", "url"}:
                    raise CollectorError(f"{label} has incomplete or extra fields")
                _safe_relative(item["path"], f"{label} path")
                if field == "rosdep_source_urls":
                    _url(item["url"], f"{label} URL")
                else:
                    _apt_or_https(item["url"], f"{label} URL")
    if is_v2:
        source_entries = value["source_entries"]
        if not isinstance(source_entries, list) or not source_entries:
            raise CollectorError("acquisition ledger source entries are empty")
        if source_entries != sorted(
                source_entries, key=lambda item: (item.get("path", ""),
                                                   item.get("ordinal", -1))):
            raise CollectorError("acquisition ledger source entries are not sorted")
        source_paths = {item["path"] for item in value["apt_source_urls"]}
        source_urls_by_path = {
            item["path"]: list(item["urls"])
            for item in value["apt_source_urls"]
        }
        observed_paths: set[str] = set()
        observed_ordinals: set[tuple[str, int]] = set()
        observed_uris_by_path: dict[str, set[str]] = {}
        for item in source_entries:
            if not isinstance(item, Mapping) or set(item) != LEDGER_SOURCE_FIELDS:
                raise CollectorError("acquisition ledger source entry fields are invalid")
            path = _safe_relative(item["path"], "acquisition ledger source entry path")
            if path not in source_paths:
                raise CollectorError("acquisition ledger source entry has unknown path")
            if type(item["ordinal"]) is not int or item["ordinal"] < 0 or \
                    (path, item["ordinal"]) in observed_ordinals:
                raise CollectorError("acquisition ledger source entry ordinal is invalid")
            if type(item["line_start"]) is not int or type(item["line_end"]) is not int or \
                    item["line_start"] < 1 or item["line_end"] < item["line_start"]:
                raise CollectorError("acquisition ledger source entry provenance is invalid")
            observed_paths.add(path)
            observed_ordinals.add((path, item["ordinal"]))
            if item["format"] not in {"legacy", "deb822"}:
                raise CollectorError("acquisition ledger source entry format is invalid")
            for key in ("types", "uris", "suites", "components"):
                values = item[key]
                if (not isinstance(values, list) or not values or
                        len(values) != len(set(values))):
                    raise CollectorError("acquisition ledger source entry values are invalid")
            if any(value not in {"deb", "deb-src"} for value in item["types"]):
                raise CollectorError("acquisition ledger source entry type is invalid")
            for url in item["uris"]:
                _apt_or_https(url, "acquisition ledger source entry URI")
            observed_uris_by_path.setdefault(path, set()).update(item["uris"])
            for atom in item["suites"] + item["components"]:
                if not isinstance(atom, str) or not atom or \
                        any(char in atom for char in "/\\\x00\r\n"):
                    raise CollectorError("acquisition ledger source entry atom is invalid")
            if not isinstance(item["options"], list):
                raise CollectorError("acquisition ledger source entry options are invalid")
            option_names: list[str] = []
            for option in item["options"]:
                if not isinstance(option, Mapping) or set(option) != {"name", "value"} or \
                        not isinstance(option["name"], str) or not isinstance(option["value"], str):
                    raise CollectorError("acquisition ledger source entry option is invalid")
                option_names.append(option["name"])
            if option_names != sorted(set(option_names)):
                raise CollectorError("acquisition ledger source entry options are duplicated")
            signed_by = item["signed_by"]
            if not isinstance(signed_by, Mapping):
                raise CollectorError("acquisition ledger source entry key binding is invalid")
            kind = signed_by.get("kind")
            if kind == "none":
                if set(signed_by) != {"kind"}:
                    raise CollectorError("acquisition ledger source entry key binding is extra")
            elif kind in {"path", "inline"}:
                expected_key_fields = {"kind", "source", "artifact_path", "bytes", "sha256"}
                if kind == "inline":
                    expected_key_fields.add("fingerprint")
                if set(signed_by) != expected_key_fields:
                    raise CollectorError("acquisition ledger source entry key binding is incomplete")
                if kind == "path":
                    source = signed_by["source"]
                    if not isinstance(source, str) or not source.startswith("/") or \
                            Path(source).as_posix() != source:
                        raise CollectorError("acquisition ledger keyring source is invalid")
                elif signed_by["source"] != "inline":
                    raise CollectorError("acquisition ledger inline key source is invalid")
                if kind == "inline" and (
                        not isinstance(signed_by.get("fingerprint"), str) or
                        FINGERPRINT_RE.fullmatch(signed_by["fingerprint"]) is None):
                    raise CollectorError("acquisition ledger inline key fingerprint is invalid")
                _safe_relative(signed_by["artifact_path"],
                               "acquisition ledger key artifact path")
                _sha_value(signed_by["sha256"], "acquisition ledger key artifact SHA")
                if type(signed_by["bytes"]) is not int or signed_by["bytes"] <= 0:
                    raise CollectorError("acquisition ledger key artifact size is invalid")
            else:
                raise CollectorError("acquisition ledger source entry key kind is invalid")
        active_source_paths = {
            path for path, urls in source_urls_by_path.items() if urls}
        if observed_paths != active_source_paths:
            raise CollectorError("acquisition ledger source entries do not cover source files")
        for path in sorted(source_paths):
            expected_urls = source_urls_by_path[path]
            observed_urls = sorted(observed_uris_by_path.get(path, set()))
            if observed_urls != expected_urls:
                raise CollectorError(
                    "acquisition ledger source URI set does not match source entries")
    identity = canonical_hash({key: value[key] for key in required if key != "canonical_sha256"})
    if is_v2:
        identity = canonical_hash({key: value[key] for key in expected_fields
                                   if key != "canonical_sha256"})
    if value["canonical_sha256"] != identity:
        raise CollectorError("acquisition ledger canonical identity drift")
    return {key: value[key] for key in value}


INNER_SCHEMA = "glim_clean_room_r3_apt_closure_collector_inner_v1"
INNER_REQUIRED = {
    "schema", "schema_version", "status", "benchmark_eligible", "capture_id",
    "package_allowlist", "dpkg_packages_sorted", "repositories",
    "apt_source_files_sha256", "apt_release_or_inrelease", "rosdep_sources_sha256",
    "rosdep_cache_files", "artifacts", "archive_set_identity_sha256",
    "custodian_review",
}


def _validate_inner_payload(value: Any) -> dict[str, Any]:
    """Validate only filesystem/package closure fields from the inner phase.

    Host identity (image, Docker argv/mounts, network phase, and plan identity)
    is intentionally absent.  The outer host receipt supplies and binds those
    fields when a future capture is composed.
    """
    if not isinstance(value, Mapping) or set(value) != INNER_REQUIRED:
        raise CollectorError("inner payload fields contain host identity or are incomplete")
    if value["schema"] != INNER_SCHEMA or value["schema_version"] != 1 or \
            value["status"] != "REVIEW_REQUIRED" or value["benchmark_eligible"] is not False:
        raise CollectorError("inner payload is not review-only")
    _safe_id(value["capture_id"], "capture id")
    try:
        allowlist = CAPTURE._validate_allowlist(value["package_allowlist"])
        artifacts = CAPTURE._validate_file_specs(value["artifacts"])
        artifact_map = CAPTURE._artifact_map(artifacts)
        packages = CAPTURE._validate_package_records(
            value["dpkg_packages_sorted"], allowlist, artifact_map)
        repositories = CAPTURE._validate_repositories(value["repositories"])
        source_records = value["apt_source_files_sha256"]
        source_urls_are_plural = isinstance(source_records, list) and any(
            isinstance(item, Mapping) and "urls" in item for item in source_records)
        apt_sources = CAPTURE._validate_ref_records(
            source_records, "apt source file set", artifact_map, "apt_source",
            allow_url_list=source_urls_are_plural)
        releases = CAPTURE._validate_ref_records(
            value["apt_release_or_inrelease"], "Release/InRelease set", artifact_map, "apt_release")
        rosdep_sources = CAPTURE._validate_ref_records(
            value["rosdep_sources_sha256"], "rosdep source set", artifact_map, "rosdep_source")
        rosdep_cache = CAPTURE._validate_cache_records(value["rosdep_cache_files"], artifact_map)
        CAPTURE._assert_role_artifact_set(
            apt_sources, artifact_map, "apt_source", "apt source")
        CAPTURE._assert_role_artifact_set(
            releases, artifact_map, "apt_release", "Release/InRelease")
        CAPTURE._assert_single_url_bindings(
            releases, artifact_map, "Release/InRelease")
        CAPTURE._assert_role_artifact_set(
            rosdep_sources, artifact_map, "rosdep_source", "rosdep source")
        CAPTURE._assert_single_url_bindings(
            rosdep_sources, artifact_map, "rosdep source")
        CAPTURE._assert_role_artifact_set(
            rosdep_cache, artifact_map, "rosdep_cache", "rosdep cache")
        CAPTURE._assert_role_artifact_set(
            [{"artifact_path": package["artifact_path"]}
             for package in packages], artifact_map, "deb", "package deb")
        CAPTURE._assert_role_artifact_set(
            [{"artifact_path": package["license"]["artifact_path"]}
             for package in packages], artifact_map, "license", "package license")
        for package in packages:
            deb_artifact = artifact_map[package["artifact_path"]]
            if ("source_url" not in deb_artifact or "source_urls" in deb_artifact or
                    deb_artifact["source_url"] != package["url"]):
                raise CollectorError(
                    f"dpkg package {package['name']} artifact URL binding drift")
            license_artifact = artifact_map[package["license"]["artifact_path"]]
            if "source_url" in license_artifact or "source_urls" in license_artifact:
                raise CollectorError(
                    f"dpkg package {package['name']} license URL is unexpected")
    except (AttributeError, ValueError) as error:
        raise CollectorError(f"inner payload closure validation failed: {error}") from error
    _sha_value(value["archive_set_identity_sha256"], "archive set identity")
    if value["custodian_review"] != {
            "status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True}:
        raise CollectorError("inner custodian review placeholder is not exact")
    return {**dict(value), "package_allowlist": allowlist, "artifacts": artifacts,
            "dpkg_packages_sorted": packages, "repositories": repositories,
            "apt_source_files_sha256": apt_sources, "apt_release_or_inrelease": releases,
            "rosdep_sources_sha256": rosdep_sources, "rosdep_cache_files": rosdep_cache}


def read_deb_metadata(path: Path) -> dict[str, str]:
    """Read exactly Package/Version/Architecture using the fixed dpkg-deb argv."""
    argv = [*DPKG_DEB_COMMAND, str(path), "Package", "Version", "Architecture"]
    try:
        result = subprocess.run(argv, shell=False, check=False, capture_output=True,
                                timeout=MAX_DPKG_DEB_SECONDS)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise CollectorError(f"fixed dpkg-deb metadata command unavailable: {error}") from error
    if result.returncode != 0 or result.stderr:
        raise CollectorError(f"dpkg-deb metadata failed for {path.name}")
    try:
        lines = result.stdout.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise CollectorError(f"dpkg-deb metadata is not UTF-8: {path.name}") from error
    if len(lines) != 3 or any(not line or "\x00" in line for line in lines):
        raise CollectorError(f"dpkg-deb metadata shape is invalid: {path.name}")
    return {"name": lines[0], "version": lines[1], "architecture": lines[2]}


def _artifact_path(role: str, source_relative: str) -> str:
    return f"artifacts/{role}/{source_relative}"


def _copy_artifact(container_root: Path, source_relative: str, source_root: Path,
                   role: str, url: str | None = None,
                   urls: list[str] | None = None) -> dict[str, Any]:
    source = _join(container_root, source_relative, "collector source artifact")
    data = _regular_bytes(source, "collector source artifact")
    destination_relative = "raw/" + source_relative
    destination = source_root / destination_relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    _write_exclusive(destination, data)
    record: dict[str, Any] = {
        "path": _artifact_path(role, source_relative),
        "role": role,
        "source_path": destination_relative,
        "bytes": len(data),
        "sha256": _sha(data),
    }
    if url is not None:
        record["source_url"] = url
    if urls is not None:
        record["source_urls"] = list(urls)
    return record


def _artifact_lookup(artifacts: list[Mapping[str, Any]], source_relative: str) -> Mapping[str, Any]:
    matches = [item for item in artifacts if item.get("source_path") == "raw/" + source_relative]
    if len(matches) != 1:
        raise CollectorError(f"collector artifact binding is missing: {source_relative}")
    return matches[0]


def _repository_map(ledger: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    return {str(item["release_path"]): item for item in ledger["repositories"]}


def _apt_source_url_map(ledger: Mapping[str, Any]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    v2 = ledger.get("source_manifest_schema") == SOURCE_SCHEMA_V2
    for item in ledger["apt_source_urls"]:
        if v2:
            urls = list(item["urls"])
        else:
            urls = [item["url"]]
        if item["path"] in result:
            raise CollectorError("apt source URL ledger contains duplicate paths")
        result[item["path"]] = urls
    return result


def _staged_debs(staged_root: Path, ledger: Mapping[str, Any],
                 dpkg_reader: Callable[[Path], dict[str, str]]) -> dict[str, Path]:
    """Reopen a fresh, immutable deb staging directory.

    The staging directory is produced before ``apt-get install`` and mounted
    read-only for collection.  It is deliberately independent of apt's cache,
    which may be removed by a package-manager cleanup hook.
    """
    staged_root = Path(staged_root)
    _directory(staged_root, "staged deb root")
    expected = {str(entry["filename"]): entry for entry in ledger["entries"]}
    manifest_path = staged_root / "staged-debs.manifest.json"
    manifest = _read_json(manifest_path, "staged deb manifest")
    if set(manifest) != {"schema", "schema_version", "status", "entries", "canonical_sha256"} or \
            manifest.get("schema") != "glim_clean_room_r3_staged_debs_v1" or \
            manifest.get("schema_version") != 1 or \
            manifest.get("status") != "SEALED_REVIEW_REQUIRED":
        raise CollectorError("staged deb manifest schema/status is not exact")
    entries = manifest["entries"]
    if not isinstance(entries, list) or [item.get("filename") for item in entries] != sorted(expected) or \
            len(entries) != len(expected) or len({item.get("filename") for item in entries}) != len(entries):
        raise CollectorError("staged deb manifest file set is not exact")
    if manifest["canonical_sha256"] != canonical_hash(
            {key: manifest[key] for key in manifest if key != "canonical_sha256"}):
        raise CollectorError("staged deb manifest identity drift")
    for entry in entries:
        if set(entry) != {"filename", "bytes", "sha256"} or entry["filename"] not in expected:
            raise CollectorError("staged deb manifest contains an unexpected entry")
        _sha_value(entry["sha256"], "staged deb SHA")
        if type(entry["bytes"]) is not int or entry["bytes"] <= 0:
            raise CollectorError("staged deb manifest size is invalid")
    actual = []
    manifest_bytes = _regular_bytes(manifest_path, "staged deb manifest", max_bytes=MAX_JSON_BYTES)
    manifest_sidecar = staged_root / "staged-debs.manifest.json.sha256"
    sidecar_bytes = _regular_bytes(manifest_sidecar, "staged deb manifest sidecar", max_bytes=512)
    if sidecar_bytes.decode("ascii").strip() != \
            f"{_sha(manifest_bytes)}  staged-debs.manifest.json":
        raise CollectorError("staged deb manifest sidecar mismatch")
    for child in sorted(staged_root.iterdir(), key=lambda item: item.name):
        info = child.lstat()
        if child.name in {manifest_path.name, manifest_sidecar.name}:
            continue
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or \
                not child.name.endswith(".deb"):
            raise CollectorError("staged deb root contains an extra or unsafe entry")
        actual.append(child.name)
    if actual != sorted(expected):
        raise CollectorError("staged deb file set contains missing or extra packages")
    paths: dict[str, Path] = {}
    manifest_by_name = {entry["filename"]: entry for entry in entries}
    for filename in actual:
        path = staged_root / filename
        data = _regular_bytes(path, "staged deb")
        entry = manifest_by_name[filename]
        ledger_entry = expected[filename]
        if len(data) != entry["bytes"] or _sha(data) != entry["sha256"] or \
                len(data) != ledger_entry["bytes"] or _sha(data) != ledger_entry["sha256"]:
            raise CollectorError(f"staged deb byte identity drift: {filename}")
        package = dpkg_reader(path)
        if (package.get("name"), package.get("version"), package.get("architecture")) != \
                (ledger_entry["name"], ledger_entry["version"], ledger_entry["architecture"]):
            raise CollectorError(f"dpkg-deb metadata mismatch: {filename}")
        paths[filename] = path
    return paths


def snapshot_base(*, container_root: Path, output_path: Path) -> dict[str, Any]:
    """Copy the pre-install dpkg status into a fresh immutable snapshot."""
    source = _join(Path(container_root), "var/lib/dpkg/status", "base dpkg status source")
    data = _regular_bytes(source, "base dpkg status source", max_bytes=MAX_JSON_BYTES)
    output_path = Path(output_path)
    _write_exclusive(output_path, data)
    sidecar = Path(str(output_path) + ".sha256")
    _write_exclusive(sidecar, (_sha(data) + "  " + output_path.name + "\n").encode("ascii"))
    return {"status": "SEALED_REVIEW_REQUIRED", "sha256": _sha(data),
            "bytes": len(data), "path": str(output_path)}


def stage_debs(*, container_root: Path, ledger_path: Path, output_root: Path) -> dict[str, Any]:
    """Copy downloaded debs before install and seal their exact byte ledger."""
    root = Path(container_root)
    ledger = _validate_ledger(_read_json(Path(ledger_path), "acquisition ledger"))
    cache = _join(root, "var/cache/apt/archives", "apt archive cache")
    output_root = Path(output_root)
    if output_root.exists() or output_root.is_symlink() or not output_root.parent.is_dir():
        raise CollectorError("staged deb output must be fresh")
    output_root.mkdir(mode=0o700)
    expected = {entry["filename"]: entry for entry in ledger["entries"]}
    children = sorted(cache.iterdir(), key=lambda item: item.name)
    if any(child.is_symlink() or not child.is_file() or child.name not in expected
           for child in children) or sorted(item.name for item in children) != sorted(expected):
        raise CollectorError("apt archive cache does not exactly match acquisition ledger")
    entries = []
    for filename in sorted(expected):
        data = _regular_bytes(cache / filename, "downloaded deb")
        if len(data) != expected[filename]["bytes"] or _sha(data) != expected[filename]["sha256"]:
            raise CollectorError(f"downloaded deb byte identity drift: {filename}")
        _write_exclusive(output_root / filename, data)
        entries.append({"filename": filename, "bytes": len(data), "sha256": _sha(data)})
    manifest = {"schema": "glim_clean_room_r3_staged_debs_v1", "schema_version": 1,
                "status": "SEALED_REVIEW_REQUIRED", "entries": entries}
    manifest["canonical_sha256"] = canonical_hash(manifest)
    manifest_data = (json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n").encode()
    _write_exclusive(output_root / "staged-debs.manifest.json", manifest_data)
    _write_exclusive(output_root / "staged-debs.manifest.json.sha256",
                     (_sha(manifest_data) + "  staged-debs.manifest.json\n").encode("ascii"))
    return {"status": "SEALED_REVIEW_REQUIRED", "entries": entries,
            "canonical_sha256": manifest["canonical_sha256"]}


def collect_payload(*, container_root: Path, allowlist_path: Path, ledger_path: Path,
                    base_status_path: Path, staged_deb_root: Path, output_root: Path,
                    dpkg_reader: Callable[[Path], dict[str, str]] = read_deb_metadata) -> dict[str, Any]:
    """Collect filesystem bytes into an inner, host-identity-free payload.

    ``base_status_path`` and ``staged_deb_root`` must be produced by the fixed
    snapshot/stage phases below ``output_root.parent``.  They are deliberately
    not input mounts and no Docker/image/plan identity is accepted here.
    """
    container_root = Path(container_root)
    allowlist_path = Path(allowlist_path)
    ledger_path = Path(ledger_path)
    base_status_path = Path(base_status_path)
    staged_deb_root = Path(staged_deb_root)
    output_root = Path(output_root)
    _directory(container_root, "container root")
    allowlist = _validate_allowlist(_read_json(allowlist_path, "package allowlist"))
    ledger = _validate_ledger(_read_json(ledger_path, "acquisition ledger"))
    if output_root.exists() or output_root.is_symlink() or not output_root.parent.is_dir() or \
            output_root.parent.is_symlink():
        raise CollectorError("collector output root must be fresh and have a safe parent")
    capture_root = output_root.parent
    if base_status_path != capture_root / "base-dpkg-status.snapshot" or \
            staged_deb_root != capture_root / "staged-debs":
        raise CollectorError("snapshot/staging paths must be fixed workflow outputs")
    current = _parse_status(_join(container_root, "var/lib/dpkg/status", "dpkg status"),
                            "current dpkg status")
    base_status_bytes = _regular_bytes(base_status_path, "base dpkg status snapshot",
                                       max_bytes=MAX_JSON_BYTES)
    base = _parse_status(base_status_path, "base dpkg status snapshot")
    allowed = {(item["name"], item["version"], item["architecture"])
               for item in allowlist["union"]}
    if not allowed.issubset(current):
        raise CollectorError("allowlisted package is missing from installed dpkg status")
    current_extra = set(current) - set(base)
    if not current_extra.issubset(allowed):
        raise CollectorError("new installed package is outside the exact allowlist")
    new_packages = sorted(current_extra)
    if allowed != set(new_packages):
        raise CollectorError(
            "allowlist must equal the newly acquired package set; base-image packages "
            "must remain outside this acquisition closure")
    ledger_entries = {tuple(entry[key] for key in ("name", "version", "architecture")): entry
                      for entry in ledger["entries"]}
    if set(ledger_entries) != set(new_packages):
        raise CollectorError("acquisition ledger does not exactly cover newly installed packages")
    staged_paths = _staged_debs(staged_deb_root, ledger, dpkg_reader)
    staged_manifest_bytes = _regular_bytes(
        staged_deb_root / "staged-debs.manifest.json", "staged deb manifest",
        max_bytes=MAX_JSON_BYTES)
    apt_sources = []
    main_sources = "etc/apt/sources.list"
    try:
        main_source = _join(container_root, main_sources, "apt source file")
        if stat.S_ISREG(main_source.lstat().st_mode):
            apt_sources.append(main_sources)
    except CollectorError as error:
        if "is missing" not in str(error):
            raise
    source_dir = "etc/apt/sources.list.d"
    try:
        apt_sources.extend(_walk_files(container_root, source_dir, "apt source files"))
    except CollectorError as error:
        if "cannot be inspected" not in str(error) and "is missing" not in str(error):
            raise
    if not apt_sources:
        raise CollectorError("apt source files are missing")
    apt_source_urls = _apt_source_url_map(ledger)
    if set(apt_sources) != set(apt_source_urls):
        raise CollectorError("apt source URL ledger does not cover exact source files")
    if ledger.get("source_manifest_schema") == SOURCE_SCHEMA_V2:
        source_entry_paths = {item["path"] for item in ledger["source_entries"]}
        active_source_paths = {
            path for path, urls in apt_source_urls.items() if urls}
        if source_entry_paths != active_source_paths:
            raise CollectorError("source entry ledger does not cover exact active source files")
    release_map = _repository_map(ledger)
    release_paths = sorted(release_map)
    release_files = sorted(path for path in _walk_files(container_root, "var/lib/apt/lists",
                                                         "apt Release files")
                           if path.endswith("Release") or path.endswith("InRelease"))
    if release_files != release_paths:
        raise CollectorError("Release/InRelease file set does not match repository ledger")
    rosdep_source_urls = {item["path"]: item["url"] for item in ledger["rosdep_source_urls"]}
    rosdep_source_files = _walk_files(container_root, "etc/ros/rosdep/sources.list.d",
                                      "rosdep source files")
    if rosdep_source_files != sorted(rosdep_source_urls):
        raise CollectorError("rosdep source file set does not match ledger")
    rosdep_cache_files = _walk_files(container_root, "var/lib/ros/rosdep", "rosdep cache")
    license_sources = []
    for identity in sorted(allowed):
        license_sources.append(f"usr/share/doc/{identity[0]}/copyright")
    source_root = output_root / "collector-source"
    output_root.mkdir(parents=True, mode=0o700)
    source_root.mkdir(mode=0o700)
    artifacts: list[dict[str, Any]] = []
    source_manifest_is_v2 = ledger.get("source_manifest_schema") == SOURCE_SCHEMA_V2
    for path in apt_sources:
        if source_manifest_is_v2:
            artifacts.append(_copy_artifact(
                container_root, path, source_root, "apt_source",
                urls=apt_source_urls[path]))
        else:
            artifacts.append(_copy_artifact(
                container_root, path, source_root, "apt_source",
                url=apt_source_urls[path][0]))
    for path in release_paths:
        artifacts.append(_copy_artifact(
            container_root, path, source_root, "apt_release",
            url=release_map[path]["release_url"]))
    for path in rosdep_source_files:
        artifacts.append(_copy_artifact(container_root, path, source_root, "rosdep_source",
                                         rosdep_source_urls[path]))
    for path in rosdep_cache_files:
        artifacts.append(_copy_artifact(container_root, path, source_root, "rosdep_cache"))
    for entry in ledger["entries"]:
        path = staged_paths[entry["filename"]]
        data = _regular_bytes(path, "staged deb")
        relative = "staged-debs/" + entry["filename"]
        destination = source_root / "raw" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        _write_exclusive(destination, data)
        artifacts.append({"path": _artifact_path("deb", relative), "role": "deb",
                          "source_path": "raw/" + relative, "bytes": len(data),
                          "sha256": _sha(data), "source_url": entry["url"]})
    for path in license_sources:
        artifacts.append(_copy_artifact(container_root, path, source_root, "license"))
    artifacts.sort(key=lambda item: item["path"])
    artifact_by_source = {item["source_path"]: item for item in artifacts}
    packages: list[dict[str, Any]] = []
    for identity in sorted(allowed):
        record = current[identity]
        ledger_entry = ledger_entries.get(identity)
        license_source = "raw/" + f"usr/share/doc/{identity[0]}/copyright"
        license_artifact = artifact_by_source[license_source]
        if ledger_entry is None:
            filename = f"base-{identity[0]}.deb"
            package_url = "https://invalid.example/base-package-not-downloaded"
            deb_artifact = None
        else:
            filename = ledger_entry["filename"]
            package_url = ledger_entry["url"]
            deb_artifact = artifact_by_source[
                "raw/staged-debs/" + ledger_entry["filename"]]
        package = {
            **record,
            "filename": filename,
            "url": package_url,
            "sha256": deb_artifact["sha256"] if deb_artifact else _sha(b"base-package"),
            "bytes": deb_artifact["bytes"] if deb_artifact else len(b"base-package"),
            "artifact_path": deb_artifact["path"] if deb_artifact else license_artifact["path"],
            "license": {
                "path": f"usr/share/doc/{identity[0]}/copyright",
                "identity": "copyright-artifact",
                "sha256": license_artifact["sha256"],
                "bytes": license_artifact["bytes"],
                "artifact_path": license_artifact["path"],
            },
        }
        if deb_artifact is None:
            # The existing capture schema models every allowlisted package as a
            # deb artifact.  A base package is therefore represented only when
            # it has an exact ledger entry; otherwise fail closed rather than
            # inventing a package byte identity.
            raise CollectorError("base allowlisted package lacks an acquired deb identity")
        packages.append(package)
    packages.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
    repositories = []
    for repo in ledger["repositories"]:
        release_path = repo["release_path"]
        release_artifact = artifact_by_source["raw/" + release_path]
        if release_artifact["sha256"] != repo["release_sha256"]:
            raise CollectorError("Release/InRelease byte identity does not match ledger")
        repositories.append({
            "url": repo["url"],
            "release_or_inrelease_url": repo["release_url"],
            "release_or_inrelease_sha256": release_artifact["sha256"],
            "release_or_inrelease_bytes": release_artifact["bytes"],
        })
    repositories.sort(key=lambda item: (item["url"], item["release_or_inrelease_url"]))
    if ledger.get("source_manifest_schema") == SOURCE_SCHEMA_V2:
        apt_source_records = [{"urls": apt_source_urls[path],
                               "artifact_path": artifact_by_source["raw/" + path]["path"],
                               "sha256": artifact_by_source["raw/" + path]["sha256"],
                               "bytes": artifact_by_source["raw/" + path]["bytes"]}
                              for path in sorted(apt_sources)]
        apt_source_records.sort(key=lambda item: (tuple(item["urls"]),
                                                  item["artifact_path"]))
    else:
        apt_source_records = [{"url": apt_source_urls[path][0],
                               "artifact_path": artifact_by_source["raw/" + path]["path"],
                               "sha256": artifact_by_source["raw/" + path]["sha256"],
                               "bytes": artifact_by_source["raw/" + path]["bytes"]}
                              for path in sorted(apt_sources)]
        apt_source_records.sort(key=lambda item: (item["url"], item["artifact_path"]))
    release_records = []
    for release_path, repository in sorted(release_map.items()):
        release_artifact = artifact_by_source["raw/" + release_path]
        release_records.append({
            "url": repository["release_url"],
            "artifact_path": release_artifact["path"],
            "sha256": release_artifact["sha256"],
            "bytes": release_artifact["bytes"],
        })
    release_records.sort(key=lambda item: (item["url"], item["artifact_path"]))
    rosdep_records = [{"url": rosdep_source_urls[path],
                       "artifact_path": artifact_by_source["raw/" + path]["path"],
                       "sha256": artifact_by_source["raw/" + path]["sha256"],
                       "bytes": artifact_by_source["raw/" + path]["bytes"]}
                      for path in sorted(rosdep_source_files)]
    rosdep_records.sort(key=lambda item: (item["url"], item["artifact_path"]))
    rosdep_cache_records = [{"artifact_path": item["path"], "sha256": item["sha256"],
                             "bytes": item["bytes"]}
                            for item in artifacts if item["role"] == "rosdep_cache"]
    archive_set_identity = canonical_hash({"entries": ledger["entries"]})
    capture_id = "r3-apt-inner-" + canonical_hash({
        "allowlist": allowlist["canonical_sha256"],
        "archive_set": archive_set_identity,
    })[:24]
    payload = {
        "schema": INNER_SCHEMA,
        "schema_version": 1,
        "status": "REVIEW_REQUIRED",
        "benchmark_eligible": False,
        "capture_id": capture_id,
        "package_allowlist": allowlist,
        "dpkg_packages_sorted": packages,
        "repositories": repositories,
        "apt_source_files_sha256": apt_source_records,
        "apt_release_or_inrelease": release_records,
        "rosdep_sources_sha256": rosdep_records,
        "rosdep_cache_files": rosdep_cache_records,
        "artifacts": artifacts,
        "archive_set_identity_sha256": archive_set_identity,
        "custodian_review": {"status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True},
    }
    payload = _validate_inner_payload(payload)
    artifact_projection = [{key: item[key] for key in ("path", "role", "source_path", "bytes", "sha256")}
                           for item in artifacts]
    payload_bytes = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode()
    source_identity = canonical_hash(artifact_projection)
    ledger_bytes = _regular_bytes(ledger_path, "acquisition ledger", max_bytes=MAX_JSON_BYTES)
    receipt = {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "status": "REVIEW_REQUIRED", "capture_mode": CAPTURE_MODE,
        "benchmark_eligible": False, "payload_sha256": _sha(payload_bytes),
        "capture_id": payload["capture_id"],
        "ledger_path": "apt-acquisition-ledger.json",
        "ledger_sha256": _sha(ledger_bytes), "allowlist_sha256": _sha(
            _regular_bytes(allowlist_path, "package allowlist", max_bytes=MAX_JSON_BYTES)),
        "base_status_sha256": _sha(base_status_bytes),
        "staged_deb_manifest_sha256": _sha(staged_manifest_bytes),
        "archive_set_identity_sha256": archive_set_identity,
        "source_identity_sha256": source_identity,
        "files": artifact_projection,
        "receipt_sha256": "",
    }
    receipt["receipt_sha256"] = canonical_hash(receipt, excluded="receipt_sha256")
    output_payload_path = output_root / PAYLOAD_NAME
    output_receipt_path = output_root / RECEIPT_NAME
    _write_exclusive(output_root / (PAYLOAD_NAME + ".sha256"),
                     (_sha(payload_bytes) + "  " + PAYLOAD_NAME + "\n").encode("ascii"))
    _write_exclusive(output_payload_path, payload_bytes)
    _write_exclusive(output_root / "apt-acquisition-ledger.json", ledger_bytes)
    _write_exclusive(output_root / "apt-acquisition-ledger.json.sha256",
                     (_sha(ledger_bytes) + "  apt-acquisition-ledger.json\n").encode("ascii"))
    _write_exclusive(output_root / "base-dpkg-status.snapshot", base_status_bytes)
    _write_exclusive(output_root / "base-dpkg-status.snapshot.sha256",
                     (_sha(base_status_bytes) + "  base-dpkg-status.snapshot\n").encode("ascii"))
    _write_exclusive(output_root / "staged-debs.manifest.json", staged_manifest_bytes)
    _write_exclusive(output_root / "staged-debs.manifest.json.sha256",
                     (_sha(staged_manifest_bytes) + "  staged-debs.manifest.json\n").encode("ascii"))
    receipt_bytes = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode()
    _write_exclusive(output_receipt_path, receipt_bytes)
    _write_exclusive(output_root / (RECEIPT_NAME + ".sha256"),
                     (_sha(receipt_bytes) + "  " + RECEIPT_NAME + "\n").encode("ascii"))
    return verify_collector_output(output_root)


def verify_collector_output(output_root: Path) -> dict[str, Any]:
    """Reopen every candidate byte and reject extra/missing/special entries."""
    output_root = Path(output_root)
    _directory(output_root, "collector output root")
    payload_path = output_root / PAYLOAD_NAME
    receipt_path = output_root / RECEIPT_NAME
    payload_bytes = _regular_bytes(payload_path, "collector payload", max_bytes=MAX_JSON_BYTES)
    receipt_bytes = _regular_bytes(receipt_path, "collector receipt", max_bytes=MAX_JSON_BYTES)
    payload_sidecar = _regular_bytes(output_root / (PAYLOAD_NAME + ".sha256"),
                                      "collector payload sidecar", max_bytes=512)
    receipt_sidecar = _regular_bytes(output_root / (RECEIPT_NAME + ".sha256"),
                                      "collector receipt sidecar", max_bytes=512)
    if payload_sidecar.decode("ascii").strip() != f"{_sha(payload_bytes)}  {PAYLOAD_NAME}":
        raise CollectorError("collector payload sidecar mismatch")
    if receipt_sidecar.decode("ascii").strip() != f"{_sha(receipt_bytes)}  {RECEIPT_NAME}":
        raise CollectorError("collector receipt sidecar mismatch")
    payload = json.loads(payload_bytes.decode("utf-8"))
    receipt = json.loads(receipt_bytes.decode("utf-8"))
    if not isinstance(payload, Mapping) or not isinstance(receipt, Mapping):
        raise CollectorError("collector payload/receipt root is not an object")
    _validate_inner_payload(payload)
    if set(receipt) != {"schema", "schema_version", "status", "capture_mode",
                        "benchmark_eligible", "payload_sha256", "capture_id", "ledger_sha256",
                        "ledger_path",
                        "allowlist_sha256", "base_status_sha256", "staged_deb_manifest_sha256",
                        "archive_set_identity_sha256", "source_identity_sha256",
                        "files", "receipt_sha256"}:
        raise CollectorError("collector receipt fields are incomplete or extra")
    if receipt["schema"] != SCHEMA or receipt["schema_version"] != SCHEMA_VERSION or \
            receipt["status"] != "REVIEW_REQUIRED" or receipt["capture_mode"] != CAPTURE_MODE or \
            receipt["benchmark_eligible"] is not False:
        raise CollectorError("collector receipt is not review-only candidate status")
    ledger_path = output_root / str(receipt["ledger_path"])
    ledger_bytes = _regular_bytes(ledger_path, "sealed acquisition ledger",
                                  max_bytes=MAX_JSON_BYTES)
    ledger_sidecar = _regular_bytes(output_root / "apt-acquisition-ledger.json.sha256",
                                    "sealed acquisition ledger sidecar", max_bytes=512)
    if receipt["ledger_path"] != "apt-acquisition-ledger.json" or \
            ledger_sidecar.decode("ascii").strip() != \
            f"{_sha(ledger_bytes)}  apt-acquisition-ledger.json" or \
            receipt["ledger_sha256"] != _sha(ledger_bytes) or \
            receipt["payload_sha256"] != _sha(payload_bytes) or \
            receipt["capture_id"] != payload["capture_id"] or \
            receipt["archive_set_identity_sha256"] != payload["archive_set_identity_sha256"] or \
            receipt["receipt_sha256"] != canonical_hash(receipt, excluded="receipt_sha256"):
        raise CollectorError("collector receipt or ledger identity drift")
    _validate_ledger(json.loads(ledger_bytes.decode("utf-8")))
    base_snapshot = _regular_bytes(output_root / "base-dpkg-status.snapshot",
                                   "sealed base status snapshot", max_bytes=MAX_JSON_BYTES)
    base_snapshot_sidecar = _regular_bytes(output_root / "base-dpkg-status.snapshot.sha256",
                                           "sealed base status sidecar", max_bytes=512)
    staged_manifest = _regular_bytes(output_root / "staged-debs.manifest.json",
                                     "sealed staged deb manifest", max_bytes=MAX_JSON_BYTES)
    staged_manifest_sidecar = _regular_bytes(output_root / "staged-debs.manifest.json.sha256",
                                             "sealed staged deb sidecar", max_bytes=512)
    if base_snapshot_sidecar.decode("ascii").strip() != \
            f"{_sha(base_snapshot)}  base-dpkg-status.snapshot" or \
            staged_manifest_sidecar.decode("ascii").strip() != \
            f"{_sha(staged_manifest)}  staged-debs.manifest.json" or \
            receipt["base_status_sha256"] != _sha(base_snapshot) or \
            receipt["staged_deb_manifest_sha256"] != _sha(staged_manifest):
        raise CollectorError("collector status/staging identity is invalid")
    raw_root = output_root / "collector-source" / "raw"
    _directory(raw_root, "collector raw source root")
    actual_files: list[dict[str, Any]] = []
    pending = [raw_root]
    while pending:
        current = pending.pop()
        for child in sorted(current.iterdir(), key=lambda item: item.name):
            info = child.lstat()
            if stat.S_ISLNK(info.st_mode):
                raise CollectorError("collector source contains a symlink")
            if stat.S_ISDIR(info.st_mode):
                pending.append(child)
            elif stat.S_ISREG(info.st_mode) and info.st_nlink == 1:
                data = _regular_bytes(child, "collector raw artifact")
                actual_files.append({"path": child.relative_to(raw_root).as_posix(),
                                     "bytes": len(data), "sha256": _sha(data)})
            else:
                raise CollectorError("collector source contains a special or hard-linked file")
    expected_files = sorted({item["source_path"].removeprefix("raw/"):
                             {"bytes": item["bytes"], "sha256": item["sha256"]}
                             for item in payload["artifacts"]}.items())
    observed_files = sorted((item["path"], {"bytes": item["bytes"], "sha256": item["sha256"]})
                            for item in actual_files)
    if observed_files != expected_files:
        raise CollectorError("collector raw artifact set or byte identity drift")
    projection = [{key: item[key] for key in ("path", "role", "source_path", "bytes", "sha256")}
                  for item in payload["artifacts"]]
    if receipt["files"] != projection or receipt["source_identity_sha256"] != canonical_hash(projection):
        raise CollectorError("collector source identity binding drift")
    collector_children = {item.name for item in (output_root / "collector-source").iterdir()}
    if collector_children != {"raw"}:
        raise CollectorError("collector source root contains extra or missing entries")
    expected_names = {PAYLOAD_NAME, PAYLOAD_NAME + ".sha256", RECEIPT_NAME,
                      RECEIPT_NAME + ".sha256", "apt-acquisition-ledger.json",
                      "apt-acquisition-ledger.json.sha256", "base-dpkg-status.snapshot",
                      "base-dpkg-status.snapshot.sha256", "staged-debs.manifest.json",
                      "staged-debs.manifest.json.sha256", "collector-source"}
    actual_names = {item.name for item in output_root.iterdir()}
    if actual_names != expected_names:
        raise CollectorError("collector output contains missing or extra entries")
    return {
        "status": "REVIEW_REQUIRED", "capture_mode": CAPTURE_MODE,
        "benchmark_eligible": False, "payload_sha256": receipt["payload_sha256"],
        "receipt_sha256": _sha(receipt_bytes), "source_identity_sha256": receipt[
            "source_identity_sha256"], "root": str(output_root),
    }


def seal_to_capture(collector_root: Path, capture_output: Path,
                    expected_binding: Mapping[str, Any]) -> dict[str, Any]:
    """Connect inner bytes to the capture verifier using outer host evidence.

    The inner payload is never allowed to provide image, network, argv, mount,
    or plan identity.  ``expected_binding`` is the only source for those
    fields and is expected to have been re-opened from outer host logs.
    """
    verify_collector_output(collector_root)
    payload = _read_json(Path(collector_root) / PAYLOAD_NAME, "collector payload")
    required_outer = {"image_reference", "argv", "mounts", "image_inspect",
                      "plan_identity_sha256"}
    if not isinstance(expected_binding, Mapping) or not required_outer.issubset(expected_binding):
        raise CollectorError("outer host binding is incomplete")
    image_reference = expected_binding["image_reference"]
    if image_reference != PINNED_BASE_IMAGE:
        raise CollectorError("outer image binding is not the pinned base")
    signature_gate = None
    if expected_binding.get("collector_input_allowed") is True:
        index_path = expected_binding.get("signature_receipt_index_path")
        proposal_sha = expected_binding.get("proposal_sha256")
        if not isinstance(index_path, str) or not isinstance(proposal_sha, str):
            raise CollectorError("collector promotion requires a signature receipt index and proposal SHA")
        ledger = _read_json(Path(collector_root) / "apt-acquisition-ledger.json",
                            "collector acquisition ledger")
        try:
            signature_gate = SIGNATURE_PROMOTION.validate_receipt_index(
                Path(index_path), expected_repositories=ledger["repositories"], ledger=ledger,
                proposal_sha256=proposal_sha, require_promotable=True)
        except Exception as error:
            raise CollectorError(f"collector promotion signature gate rejected: {error}") from error
    base_image = {"reference": image_reference, "digest": image_reference.replace("ros@", ""),
                  "distribution": "jazzy", "platform": "linux/amd64"}
    provisioning = ["apt-get", "install", "--yes", "--no-install-recommends"] + [
        f"{item['name']}={item['version']}" for item in payload["package_allowlist"]["union"]]
    capture_payload = {
        "schema": CAPTURE.SCHEMA, "schema_version": CAPTURE.SCHEMA_VERSION,
        "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
        "capture_id": payload["capture_id"], "base_image": base_image,
        "network": {"network_used": True, "phase": "PROVISIONING_CONNECTED",
                    "build_test_network": "NOT_STARTED", "disconnect_required": True},
        "provisioning": {"command": provisioning,
                          "command_sha256": canonical_hash({"argv": provisioning}),
                          "exit_status": 0},
        "container": {
            "image_reference": image_reference,
            "image_inspect": expected_binding["image_inspect"],
            "argv": list(expected_binding["argv"]), "mounts": list(expected_binding["mounts"]),
            "network_mode": "bridge", "network_used": True,
            "phase_trace": [
                {"phase": "image_inspect", "network_used": False},
                {"phase": "container_run", "network_used": True},
                {"phase": "payload_capture", "network_used": True},
                {"phase": "disconnect_pending", "network_used": True},
            ],
        },
        "package_allowlist": payload["package_allowlist"],
        "dpkg_packages_sorted": payload["dpkg_packages_sorted"],
        "repositories": payload["repositories"],
        "apt_source_files_sha256": payload["apt_source_files_sha256"],
        "apt_release_or_inrelease": payload["apt_release_or_inrelease"],
        "rosdep_sources_sha256": payload["rosdep_sources_sha256"],
        "rosdep_cache_files": payload["rosdep_cache_files"],
        "artifacts": payload["artifacts"],
        "archive_set_identity_sha256": payload["archive_set_identity_sha256"],
        "plan_identity_sha256": expected_binding["plan_identity_sha256"],
        "custodian_review": payload["custodian_review"],
    }
    CAPTURE.seal_capture(Path(collector_root) / "collector-source",
                         Path(capture_output), capture_payload)
    binding = dict(expected_binding)
    binding.setdefault("receipt_sha256", CAPTURE.sha256_file(
        Path(capture_output) / CAPTURE.CAPTURE_RECEIPT))
    verified = CAPTURE.verify_capture_root(Path(capture_output), binding)
    if signature_gate is not None:
        verified["signature_gate"] = signature_gate
    return verified


def uri_ledger(*, input_path: Path, output_path: Path) -> dict[str, Any]:
    """Copy and independently validate the fixed provisioning ledger artifact."""
    ledger = _validate_ledger(_read_json(input_path, "provisioning acquisition ledger"))
    data = (json.dumps(ledger, sort_keys=True, separators=(",", ":")) + "\n").encode()
    output_path = Path(output_path)
    _write_exclusive(output_path, data)
    _write_exclusive(Path(str(output_path) + ".sha256"),
                     (_sha(data) + "  " + output_path.name + "\n").encode("ascii"))
    return {"status": "REVIEW_REQUIRED", "ledger_sha256": _sha(data),
            "output": str(output_path)}


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    ledger_parser = subparsers.add_parser("uri-ledger")
    ledger_parser.add_argument("--input", required=True)
    ledger_parser.add_argument("--output", required=True)
    snapshot_parser = subparsers.add_parser("snapshot-base")
    snapshot_parser.add_argument("--root", required=True)
    snapshot_parser.add_argument("--output", required=True)
    stage_parser = subparsers.add_parser("stage-debs")
    stage_parser.add_argument("--root", required=True)
    stage_parser.add_argument("--ledger", required=True)
    stage_parser.add_argument("--output", required=True)
    collect_parser = subparsers.add_parser("collect")
    collect_parser.add_argument("--root", required=True)
    collect_parser.add_argument("--allowlist", required=True)
    collect_parser.add_argument("--ledger", required=True)
    collect_parser.add_argument("--base-status", required=True)
    collect_parser.add_argument("--staged-debs", required=True)
    collect_parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        if args.command == "uri-ledger":
            result = uri_ledger(input_path=Path(args.input), output_path=Path(args.output))
        elif args.command == "snapshot-base":
            result = snapshot_base(container_root=Path(args.root), output_path=Path(args.output))
        elif args.command == "stage-debs":
            result = stage_debs(container_root=Path(args.root), ledger_path=Path(args.ledger),
                                output_root=Path(args.output))
        else:
            result = collect_payload(container_root=Path(args.root),
                                     allowlist_path=Path(args.allowlist),
                                     ledger_path=Path(args.ledger),
                                     base_status_path=Path(args.base_status),
                                     staged_deb_root=Path(args.staged_debs),
                                     output_root=Path(args.output))
    except (CollectorError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"collector FAIL: {error}")
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
