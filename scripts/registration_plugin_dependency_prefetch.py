#!/usr/bin/env python3
"""Immutable host-side archive prefetch contract for release legs.

Dependency installation and official archive acquisition are provisioning
operations.  The release runner consumes the sealed files produced here after
the host has disconnected the container.  This module deliberately keeps the
network implementation separate from the in-container build runner: callers
must inject a fetcher in tests and must validate the sealed root again before
starting a build.
"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import re
import stat
import sys
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
if (
        (ROOT / "lidarslam_benchmark_tools" / "__init__.py").is_file()
        and str(ROOT) not in sys.path):
    sys.path.insert(0, str(ROOT))


SCHEMA = "registration-plugin-dependency-prefetch-v1"
MANIFEST_SCHEMA = "registration-plugin-dependency-prefetch-manifest-v1"
SCHEMA_VERSION = 1
DEPENDENCY_ENVIRONMENT_NAME = "dependency_environment.receipt.json"
PREFETCH_RECEIPT_NAME = "dependency_prefetch.receipt.json"
PREFETCH_MANIFEST_NAME = "dependency_prefetch_manifest.json"
PREFETCH_DIR = "prefetch"
ARCHIVE_DIR = "prefetch/archives"
MAX_ARCHIVE_BYTES = 512 * 1024 * 1024
MAX_RECEIPT_BYTES = 4 * 1024 * 1024
MAX_MANIFEST_BYTES = 4 * 1024 * 1024
MAX_SIDECAR_BYTES = 256
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SAFE_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
ALLOWED_AUTHORITIES = frozenset(("HOST_PROMOTION", "FUNCTIONAL_CI_NON_PROMOTING"))
SAFE_RESPONSE_HEADERS = frozenset((
    "content-length", "content-type", "etag", "last-modified",
))
MAX_RESPONSE_HEADER_VALUE_BYTES = 256


class PrefetchError(RuntimeError):
    """A fail-closed prefetch or immutable-root error."""

    def __init__(self, kind, message, details=None):
        super(PrefetchError, self).__init__(message)
        self.kind = kind
        self.details = details


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_relative(value, label):
    if not isinstance(value, str) or not value or value.startswith("/") or "\\" in value:
        raise PrefetchError("PATH_INVALID", "{} is not a safe relative path".format(label))
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise PrefetchError("PATH_INVALID", "{} contains traversal".format(label))
    return value


def _archive_filename(name):
    if not isinstance(name, str) or not SAFE_NAME_RE.fullmatch(name):
        raise PrefetchError("DEPENDENCY_NAME_INVALID", str(name))
    return "{}.tar.gz".format(name)


def expected_file_paths(dependencies, include_prefetch=True):
    """Return the exact relative file allowlist for one evidence root."""
    result = {
        DEPENDENCY_ENVIRONMENT_NAME,
        DEPENDENCY_ENVIRONMENT_NAME + ".sha256",
    }
    if include_prefetch:
        result.update({
            PREFETCH_RECEIPT_NAME, PREFETCH_RECEIPT_NAME + ".sha256",
            PREFETCH_MANIFEST_NAME, PREFETCH_MANIFEST_NAME + ".sha256",
        })
        for dependency in dependencies:
            filename = _archive_filename(dependency.get("name"))
            relative = ARCHIVE_DIR + "/" + filename
            result.update({relative, relative + ".sha256"})
    return frozenset(sorted(result))


def _expected_dirs(dependencies):
    return frozenset((PREFETCH_DIR, ARCHIVE_DIR)) if dependencies else frozenset()


def _lstat_regular(path, maximum, label, require_nonempty=True):
    path = Path(path)
    try:
        info = path.lstat()
    except OSError as exc:
        raise PrefetchError("PREFETCH_ARTIFACT_MISSING", "{}: {}".format(label, exc))
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise PrefetchError("PREFETCH_ARTIFACT_NOT_REGULAR", str(path))
    if stat.S_IMODE(info.st_mode) != 0o444:
        raise PrefetchError("PREFETCH_ARTIFACT_MODE_INVALID", str(path))
    if require_nonempty and info.st_size <= 0:
        raise PrefetchError("PREFETCH_ARTIFACT_EMPTY", str(path))
    if info.st_size > maximum:
        raise PrefetchError("PREFETCH_ARTIFACT_OVERSIZE", "{}: {}".format(label, info.st_size))
    return info


def _read_sidecar(path, digest):
    sidecar = Path(str(path) + ".sha256")
    _lstat_regular(sidecar, MAX_SIDECAR_BYTES, "prefetch sidecar")
    try:
        tokens = sidecar.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        raise PrefetchError("PREFETCH_SIDECAR_INVALID", str(exc))
    if tokens != [digest, Path(path).name]:
        raise PrefetchError("PREFETCH_SIDECAR_MISMATCH", str(path))
    return sidecar


def _sealed_bytes(path, maximum, label):
    _lstat_regular(path, maximum, label)
    digest = _sha256_file(path)
    sidecar = _read_sidecar(path, digest)
    try:
        payload = Path(path).read_bytes()
    except OSError as exc:
        raise PrefetchError("PREFETCH_READ_FAILED", str(exc))
    return payload, digest, _sha256_file(sidecar)


def _walk_root(root):
    """Walk without following links and return relative files/directories."""
    root = Path(root)
    try:
        root_info = root.lstat()
    except OSError as exc:
        raise PrefetchError("PREFETCH_ROOT_INVALID", str(exc))
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise PrefetchError("PREFETCH_ROOT_INVALID", str(root))
    files = set()
    directories = set()
    pending = [root]
    while pending:
        current = pending.pop()
        for entry in sorted(os.scandir(str(current)), key=lambda item: item.name):
            relative = Path(entry.path).relative_to(root).as_posix()
            info = entry.stat(follow_symlinks=False)
            if stat.S_ISLNK(info.st_mode):
                raise PrefetchError("PREFETCH_LINK_FORBIDDEN", relative)
            if stat.S_ISDIR(info.st_mode):
                directories.add(relative)
                pending.append(Path(entry.path))
            elif stat.S_ISREG(info.st_mode):
                if info.st_nlink != 1:
                    raise PrefetchError("PREFETCH_HARDLINK_FORBIDDEN", relative)
                files.add(relative)
            else:
                raise PrefetchError("PREFETCH_SPECIAL_FILE_FORBIDDEN", relative)
    return files, directories


def _check_root_shape(root, dependencies, allowed_host_directories=(),
                      include_prefetch_dirs=False):
    files, directories = _walk_root(root)
    expected_files = set(expected_file_paths(dependencies, include_prefetch=bool(dependencies)))
    expected_dirs = set(_expected_dirs(dependencies))
    if include_prefetch_dirs:
        expected_dirs.update({PREFETCH_DIR, ARCHIVE_DIR})
    allowed = set(allowed_host_directories or ())
    if any(not isinstance(item, str) or item in ("", ".") or
           item.startswith("/") or "\\" in item or
           any(part in ("", ".", "..") for part in item.split("/"))
           for item in allowed):
        raise PrefetchError("PREFETCH_HOST_DIRECTORY_CONTRACT_INVALID", "unsafe host directory")
    expected_dirs.update(allowed)
    if files != expected_files or directories != expected_dirs:
        raise PrefetchError("PREFETCH_ROOT_ALLOWLIST_MISMATCH", _canonical({
            "expected_files": sorted(expected_files), "observed_files": sorted(files),
            "expected_dirs": sorted(expected_dirs), "observed_dirs": sorted(directories),
        }))


def _validate_url(value, label):
    if not isinstance(value, str):
        raise PrefetchError("PREFETCH_URL_INVALID", label)
    parsed = urlparse(value)
    if parsed.scheme != "https" or not parsed.netloc or parsed.username or parsed.password or \
            parsed.fragment or parsed.query:
        raise PrefetchError("PREFETCH_URL_INVALID", "{}: {}".format(label, value))
    return value


def _validate_archive_identity(dependency, payload, label):
    if not isinstance(payload, bytes) or not payload:
        raise PrefetchError("PREFETCH_ARCHIVE_EMPTY", label)
    if len(payload) > MAX_ARCHIVE_BYTES:
        raise PrefetchError("PREFETCH_ARCHIVE_OVERSIZE", label)
    expected = dependency.get("archive_sha256")
    if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
        raise PrefetchError("PREFETCH_PROFILE_SHA_INVALID", dependency.get("name"))
    observed = _sha256_bytes(payload)
    if observed != expected:
        raise PrefetchError("PREFETCH_ARCHIVE_SHA_MISMATCH", "{} expected {} observed {}".format(
            label, expected, observed))
    return observed


def _write_exclusive(path, payload):
    path = Path(path)
    if path.exists() or path.is_symlink():
        raise PrefetchError("PREFETCH_OVERWRITE_FORBIDDEN", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise PrefetchError("PREFETCH_PARENT_INVALID", str(path.parent))
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(str(path), flags, 0o600)
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
        os.chmod(str(path), 0o444)
    except OSError as exc:
        raise PrefetchError("PREFETCH_WRITE_FAILED", str(exc))
    return path


def _write_sealed(path, payload):
    path = _write_exclusive(path, payload)
    digest = _sha256_bytes(payload)
    sidecar_payload = ("{}  {}\n".format(digest, path.name)).encode("ascii")
    sidecar = _write_exclusive(Path(str(path) + ".sha256"), sidecar_payload)
    return path, sidecar, digest, _sha256_bytes(sidecar_payload)


def _canonical_hash(value, key="canonical_sha256"):
    projection = dict(value)
    projection.pop(key, None)
    return _sha256_bytes(_canonical(projection).encode("utf-8"))


def _default_fetch(url, maximum):
    request = Request(url, headers={"User-Agent": "lidarslam-release-prefetch/1"})
    redirects = []
    try:
        with urlopen(request, timeout=60) as response:
            final_url = response.geturl()
            status = getattr(response, "status", None) or response.getcode()
            payload = response.read(maximum + 1)
            headers = {}
            for key in ("content-length", "content-type", "etag", "last-modified"):
                value = response.headers.get(key)
                if value is not None:
                    # Do not truncate before the common validator sees the
                    # value: truncation could turn an oversized/secret-bearing
                    # response header into an apparently safe receipt field.
                    headers[key] = str(value)
    except Exception as exc:
        raise PrefetchError("PREFETCH_NETWORK_FAILED", str(exc))
    return {"payload": payload, "final_url": final_url, "redirects": redirects,
            "status": int(status), "headers": headers}


def _normalize_response_headers(headers):
    """Keep only deterministic, non-secret response metadata in receipts."""
    if not isinstance(headers, dict):
        raise PrefetchError("PREFETCH_RESPONSE_HEADERS_INVALID", "headers must be an object")
    normalized = {}
    for key, value in headers.items():
        if (not isinstance(key, str) or key != key.lower() or
                key not in SAFE_RESPONSE_HEADERS or key in normalized):
            raise PrefetchError("PREFETCH_RESPONSE_HEADER_FORBIDDEN", str(key))
        if not isinstance(value, str) or len(value.encode("utf-8")) > MAX_RESPONSE_HEADER_VALUE_BYTES or \
                any(ord(character) < 0x20 or ord(character) == 0x7f for character in value):
            raise PrefetchError("PREFETCH_RESPONSE_HEADER_INVALID", str(key))
        normalized[key] = value
    return dict(sorted(normalized.items()))


def _normalize_fetch_result(value, requested):
    if isinstance(value, bytes):
        value = {"payload": value, "final_url": requested, "redirects": [], "status": 200,
                 "headers": {}}
    if not isinstance(value, dict) or set(value) != {"payload", "final_url", "redirects", "status", "headers"}:
        raise PrefetchError("PREFETCH_FETCH_RESULT_INVALID", requested)
    if value["status"] != 200 or not isinstance(value["redirects"], list) or \
            not isinstance(value["headers"], dict):
        raise PrefetchError("PREFETCH_FETCH_RESULT_INVALID", requested)
    value["headers"] = _normalize_response_headers(value["headers"])
    _validate_url(value["final_url"], "final URL")
    for redirect in value["redirects"]:
        _validate_url(redirect, "redirect URL")
    requested_host = urlparse(requested).netloc
    if urlparse(value["final_url"]).netloc != requested_host or any(
            urlparse(item).netloc != requested_host for item in value["redirects"]):
        raise PrefetchError("PREFETCH_REDIRECT_HOST_MISMATCH", requested)
    return value


def _dependency_record(dependency, payload, fetch_result, archive_relative, sidecar_relative):
    archive_sha = _validate_archive_identity(dependency, payload, dependency["name"])
    return {
        "name": dependency["name"],
        "official_url": dependency["official_url"],
        "archive_url": dependency["archive_url"],
        "requested_url": dependency["archive_url"],
        "final_url": fetch_result["final_url"],
        "redirects": list(fetch_result["redirects"]),
        "http_status": fetch_result["status"],
        "response_headers": dict(sorted(fetch_result["headers"].items())),
        "commit": dependency["commit"],
        "archive_top_level": dependency["archive_top_level"],
        "archive_path_relative": archive_relative,
        "sidecar_path_relative": sidecar_relative,
        "size_bytes": len(payload),
        "archive_sha256": archive_sha,
        "source_tree_sha256": dependency["source_tree_sha256"],
        "license_path": dependency["license_path"],
        "license_sha256": dependency["license_sha256"],
    }


def prefetch_pinned_archives(dependencies, evidence_root, distro, image_digest,
                             authority="HOST_PROMOTION", fetcher=None,
                             allowed_host_directories=()):
    """Fetch and seal all profile-pinned archives exactly once.

    ``fetcher`` is a test seam with the same return mapping as ``_default_fetch``.
    No profile data is changed and an existing prefetch root is never reused.
    """
    if authority not in ALLOWED_AUTHORITIES:
        raise PrefetchError("PREFETCH_AUTHORITY_INVALID", authority)
    dependencies = list(dependencies or [])
    if not dependencies:
        raise PrefetchError("PREFETCH_DEPENDENCIES_EMPTY", "present prefetch requires dependencies")
    root = Path(evidence_root).absolute()
    if allowed_host_directories:
        _check_root_shape(
            root, [], allowed_host_directories=allowed_host_directories,
            include_prefetch_dirs=True,
        )
    else:
        # Preserve the historical standalone test seam.  The production
        # launcher always supplies the non-empty host layout contract, which
        # requires these directories to pre-exist.
        _check_root_shape(root, [])
    expected_names = [item for item in expected_file_paths(dependencies, include_prefetch=True)
                      if item not in expected_file_paths([], include_prefetch=False)]
    del expected_names
    prefetch_dir = root / PREFETCH_DIR
    archive_dir = root / ARCHIVE_DIR
    if not allowed_host_directories:
        prefetch_dir.mkdir(mode=0o755)
        archive_dir.mkdir(mode=0o755)
    else:
        for directory in (prefetch_dir, archive_dir):
            try:
                info = directory.lstat()
            except OSError as exc:
                raise PrefetchError("PREFETCH_DIRECTORY_INVALID", str(directory)) from exc
            if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
                    info.st_nlink < 2 or stat.S_IMODE(info.st_mode) != 0o700):
                raise PrefetchError("PREFETCH_DIRECTORY_INVALID", str(directory))
    fetch = fetcher or _default_fetch
    records = []
    seen = set()
    try:
        for dependency in dependencies:
            name = dependency.get("name")
            if name in seen:
                raise PrefetchError("PREFETCH_DEPENDENCY_DUPLICATE", str(name))
            seen.add(name)
            requested = _validate_url(dependency.get("archive_url"), "archive URL")
            result = _normalize_fetch_result(fetch(requested, MAX_ARCHIVE_BYTES), requested)
            payload = result["payload"]
            if not isinstance(payload, bytes) or len(payload) > MAX_ARCHIVE_BYTES:
                raise PrefetchError("PREFETCH_ARCHIVE_OVERSIZE", str(name))
            relative = ARCHIVE_DIR + "/" + _archive_filename(name)
            archive = root / relative
            _validate_archive_identity(dependency, payload, str(name))
            _write_sealed(archive, payload)
            records.append(_dependency_record(
                dependency, payload, result, relative,
                ARCHIVE_DIR + "/" + _archive_filename(name) + ".sha256"))
        entries = [{
            "name": record["name"], "path_relative": record["archive_path_relative"],
            "sidecar_path_relative": record["sidecar_path_relative"],
            "size_bytes": record["size_bytes"], "sha256": record["archive_sha256"],
        } for record in sorted(records, key=lambda item: item["name"])]
        manifest = {
            "schema": MANIFEST_SCHEMA, "schema_version": SCHEMA_VERSION,
            "status": "PASS", "entries": entries, "entry_count": len(entries),
        }
        manifest["canonical_sha256"] = _canonical_hash(manifest)
        manifest_payload = (_canonical(manifest) + "\n").encode("utf-8")
        manifest_path, manifest_sidecar, manifest_sha, manifest_sidecar_sha = _write_sealed(
            root / PREFETCH_MANIFEST_NAME, manifest_payload)
        receipt = {
            "schema": SCHEMA, "schema_version": SCHEMA_VERSION, "status": "PASS",
            "authority": authority, "distro": distro, "base_image_digest": image_digest,
            "provisioning_network_used": True, "archive_fetch_network_used": True,
            "manifest": {"path_relative": PREFETCH_MANIFEST_NAME, "sha256": manifest_sha,
                         "sidecar_path_relative": PREFETCH_MANIFEST_NAME + ".sha256",
                         "sidecar_sha256": manifest_sidecar_sha},
            "records": sorted(records, key=lambda item: item["name"]),
        }
        receipt["canonical_sha256"] = _canonical_hash(receipt)
        receipt_payload = (_canonical(receipt) + "\n").encode("utf-8")
        receipt_path, receipt_sidecar, receipt_sha, receipt_sidecar_sha = _write_sealed(
            root / PREFETCH_RECEIPT_NAME, receipt_payload)
        del manifest_path, manifest_sidecar, receipt_path, receipt_sidecar
        return validate_prefetch_root(
            root, dependencies, distro, image_digest, authority,
            allowed_host_directories=allowed_host_directories,
        )
    except Exception:
        # The root is intentionally left sealed/partial for diagnostics.  No
        # later invocation may treat a partial root as reusable.
        raise


def _validate_manifest(root, dependencies):
    path = root / PREFETCH_MANIFEST_NAME
    payload, digest, sidecar_digest = _sealed_bytes(path, MAX_MANIFEST_BYTES,
                                                     "prefetch manifest")
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeError, ValueError) as exc:
        raise PrefetchError("PREFETCH_MANIFEST_INVALID", str(exc))
    if not isinstance(value, dict) or set(value) != {
            "schema", "schema_version", "status", "entries", "entry_count", "canonical_sha256"}:
        raise PrefetchError("PREFETCH_MANIFEST_SCHEMA_INVALID", "manifest fields")
    if value["schema"] != MANIFEST_SCHEMA or value["schema_version"] != SCHEMA_VERSION or \
            value["status"] != "PASS" or value["canonical_sha256"] != _canonical_hash(value):
        raise PrefetchError("PREFETCH_MANIFEST_IDENTITY_INVALID", "manifest identity")
    expected = {}
    for dependency in dependencies:
        name = dependency.get("name")
        if name in expected:
            raise PrefetchError("PREFETCH_DEPENDENCY_DUPLICATE", name)
        expected[name] = dependency
    entries = value["entries"]
    if not isinstance(entries, list) or len(entries) != len(expected) or \
            entries != sorted(entries, key=lambda item: item.get("name", "")):
        raise PrefetchError("PREFETCH_MANIFEST_ENTRIES_INVALID", "entries")
    observed = []
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {
                "name", "path_relative", "sidecar_path_relative", "size_bytes", "sha256"}:
            raise PrefetchError("PREFETCH_MANIFEST_ENTRY_INVALID", "entry")
        dependency = expected.get(entry.get("name"))
        if dependency is None or entry["path_relative"] != ARCHIVE_DIR + "/" + _archive_filename(entry["name"]):
            raise PrefetchError("PREFETCH_MANIFEST_ENTRY_INVALID", str(entry))
        expected_path = root / entry["path_relative"]
        if entry["sidecar_path_relative"] != entry["path_relative"] + ".sha256":
            raise PrefetchError("PREFETCH_MANIFEST_ENTRY_INVALID", str(entry))
        info = _lstat_regular(expected_path, MAX_ARCHIVE_BYTES, "archive")
        observed_sha = _sha256_file(expected_path)
        if (type(entry["size_bytes"]) is not int or entry["size_bytes"] != info.st_size or
                entry["sha256"] != observed_sha or entry["sha256"] != dependency["archive_sha256"]):
            raise PrefetchError("PREFETCH_ARCHIVE_IDENTITY_MISMATCH", entry["name"])
        _read_sidecar(expected_path, observed_sha)
        observed.append(entry)
    if value["entry_count"] != len(observed):
        raise PrefetchError("PREFETCH_MANIFEST_COUNT_INVALID", "entry count")
    return value, digest, sidecar_digest


def validate_prefetch_root(evidence_root, dependencies, distro, image_digest,
                           authority=None, allowed_host_directories=()):
    """Reopen every prefetch byte and return a canonical runner binding."""
    dependencies = list(dependencies or [])
    if not dependencies:
        raise PrefetchError("PREFETCH_DEPENDENCIES_EMPTY", "present prefetch requires dependencies")
    root = Path(evidence_root).absolute()
    _check_root_shape(
        root, dependencies, allowed_host_directories=allowed_host_directories,
    )
    manifest, manifest_sha, manifest_sidecar_sha = _validate_manifest(root, dependencies)
    receipt_payload, receipt_sha, receipt_sidecar_sha = _sealed_bytes(
        root / PREFETCH_RECEIPT_NAME, MAX_RECEIPT_BYTES, "prefetch receipt")
    try:
        receipt = json.loads(receipt_payload.decode("utf-8"))
    except (UnicodeError, ValueError) as exc:
        raise PrefetchError("PREFETCH_RECEIPT_INVALID", str(exc))
    required = {
        "schema", "schema_version", "status", "authority", "distro", "base_image_digest",
        "provisioning_network_used", "archive_fetch_network_used", "manifest", "records",
        "canonical_sha256",
    }
    if not isinstance(receipt, dict) or set(receipt) != required or \
            receipt.get("schema") != SCHEMA or receipt.get("schema_version") != SCHEMA_VERSION or \
            receipt.get("status") != "PASS" or receipt.get("canonical_sha256") != _canonical_hash(receipt):
        raise PrefetchError("PREFETCH_RECEIPT_SCHEMA_INVALID", "receipt fields")
    if authority is not None and receipt["authority"] != authority:
        raise PrefetchError("PREFETCH_AUTHORITY_MISMATCH", receipt["authority"])
    if receipt["authority"] not in ALLOWED_AUTHORITIES or receipt["distro"] != distro or \
            receipt["base_image_digest"] != image_digest or \
            receipt["provisioning_network_used"] is not True or \
            receipt["archive_fetch_network_used"] is not True:
        raise PrefetchError("PREFETCH_PHASE_IDENTITY_INVALID", "receipt phase")
    manifest_binding = receipt["manifest"]
    if manifest_binding != {
            "path_relative": PREFETCH_MANIFEST_NAME, "sha256": manifest_sha,
            "sidecar_path_relative": PREFETCH_MANIFEST_NAME + ".sha256",
            "sidecar_sha256": manifest_sidecar_sha}:
        raise PrefetchError("PREFETCH_MANIFEST_BINDING_INVALID", "receipt manifest")
    expected = {item.get("name"): item for item in dependencies}
    records = receipt["records"]
    if not isinstance(records, list) or len(records) != len(expected) or \
            [item.get("name") for item in records] != sorted(expected):
        raise PrefetchError("PREFETCH_RECEIPT_RECORDS_INVALID", "record names")
    manifest_by_name = {item["name"]: item for item in manifest["entries"]}
    normalized = []
    for record in records:
        dependency = expected.get(record.get("name"))
        if dependency is None:
            raise PrefetchError("PREFETCH_RECEIPT_RECORD_INVALID", str(record))
        required_record = {
            "name", "official_url", "archive_url", "requested_url", "final_url", "redirects",
            "http_status", "response_headers", "commit", "archive_top_level", "archive_path_relative",
            "sidecar_path_relative", "size_bytes", "archive_sha256", "source_tree_sha256",
            "license_path", "license_sha256",
        }
        if set(record) != required_record or record["archive_url"] != dependency["archive_url"] or \
                record["requested_url"] != dependency["archive_url"] or \
                record["archive_sha256"] != dependency["archive_sha256"] or \
                record["commit"] != dependency["commit"] or \
                record["archive_top_level"] != dependency["archive_top_level"] or \
                record["source_tree_sha256"] != dependency["source_tree_sha256"] or \
                record["license_path"] != dependency["license_path"] or \
                record["license_sha256"] != dependency["license_sha256"]:
            raise PrefetchError("PREFETCH_RECEIPT_RECORD_DRIFT", record["name"])
        if record["archive_path_relative"] != ARCHIVE_DIR + "/" + _archive_filename(record["name"]):
            raise PrefetchError("PREFETCH_RECEIPT_PATH_INVALID", record["name"])
        if manifest_by_name.get(record["name"]) != {
                "name": record["name"], "path_relative": record["archive_path_relative"],
                "sidecar_path_relative": record["sidecar_path_relative"],
                "size_bytes": record["size_bytes"], "sha256": record["archive_sha256"]}:
            raise PrefetchError("PREFETCH_RECEIPT_MANIFEST_DRIFT", record["name"])
        normalized.append({"name": record["name"], "archive_path_relative": record["archive_path_relative"],
                           "size_bytes": record["size_bytes"], "archive_sha256": record["archive_sha256"]})
    return {
        "status": "PASS", "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "authority": receipt["authority"], "distro": distro,
        "base_image_digest": image_digest, "receipt_path_relative": PREFETCH_RECEIPT_NAME,
        "receipt_sha256": receipt_sha, "receipt_sidecar_sha256": receipt_sidecar_sha,
        "manifest_path_relative": PREFETCH_MANIFEST_NAME, "manifest_sha256": manifest_sha,
        "manifest_sidecar_sha256": manifest_sidecar_sha, "records": normalized,
    }


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", required=True)
    parser.add_argument("--distro", choices=("humble", "jazzy"), required=True)
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--authority", choices=sorted(ALLOWED_AUTHORITIES), required=True)
    parser.add_argument("--dependencies-json", required=True,
                        help="JSON array containing the profile present-leg dependency pins")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        dependencies = json.loads(args.dependencies_json)
        result = prefetch_pinned_archives(
            dependencies, args.evidence_root, args.distro, args.image_digest,
            authority=args.authority)
        print(json.dumps(result, sort_keys=True))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "kind": getattr(exc, "kind", "PREFETCH_FAILURE"),
                          "message": str(exc)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
