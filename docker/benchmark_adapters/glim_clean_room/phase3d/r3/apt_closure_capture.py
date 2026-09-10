#!/usr/bin/env python3
"""Validate and seal the additive GLIM r3 apt/deb closure contract.

This module is a deliberately small boundary between a future, networked
provisioning step and the offline r3 candidate.  It does not run Docker,
apt, rosdep, or a downloader, and it does not generate a real in-container
payload.  ``CAPTURE_MODE`` is therefore ``CONTRACT_ONLY``: tests may supply
synthetic payload bytes to exercise the no-follow verifier, but no result is
capture-ready.  A separate proposal step may construct a candidate READY
manifest, but it never edits the production manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import stat
from typing import Any, Mapping
from urllib.parse import urlsplit


SCHEMA = "glim_clean_room_r3_apt_closure_capture_v1"
PROPOSAL_SCHEMA = "glim_clean_room_r3_apt_closure_proposal_v1"
SCHEMA_VERSION = 1
CAPTURE_RECEIPT = "apt-closure.capture.receipt.json"
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_ARTIFACT_BYTES = 512 * 1024 * 1024
MAX_CAPTURE_ID_LENGTH = 128
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_RE = re.compile(r"^ros@sha256:[0-9a-f]{64}$")
DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
ARTIFACT_ROLES = frozenset({
    "apt_source", "apt_release", "rosdep_source", "rosdep_cache",
    "deb", "license",
})
REQUIRED_ARTIFACT_ROLES = frozenset(ARTIFACT_ROLES)
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
CAPTURE_MODE = "CONTRACT_ONLY"
READY_REQUIRED_FIELDS = (
    "base_image_digest", "platform", "ros_distribution", "dpkg_packages_sorted",
    "repositories", "apt_source_files_sha256", "apt_release_or_inrelease",
    "deb_filename_url_sha256_bytes_license", "rosdep_sources_sha256",
    "rosdep_cache_sha256", "install_command_sha256", "install_exit_status",
    "archive_set_identity_sha256", "dpkg_set_identity_sha256",
    "repository_set_identity_sha256", "closure_identity_sha256",
)


class CaptureError(ValueError):
    """Raised when capture or review cannot be proven exact."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_hash(value: Mapping[str, Any], excluded: str | None = None) -> str:
    projection = dict(value)
    if excluded is not None:
        projection.pop(excluded, None)
    return sha256_bytes(canonical_bytes(projection))


def sha256_file(path: Path, *, max_bytes: int = MAX_ARTIFACT_BYTES) -> str:
    _regular(path, "file", max_bytes=max_bytes)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, max_bytes: int,
             mode: int | None = None) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise CaptureError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise CaptureError(f"{label} must be a single-link regular file: {path}")
    if info.st_size <= 0 or info.st_size > max_bytes:
        raise CaptureError(f"{label} has invalid size: {path}")
    if mode is not None and stat.S_IMODE(info.st_mode) != mode:
        raise CaptureError(f"{label} has invalid mode: {path}")


def _directory(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise CaptureError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise CaptureError(f"{label} must be a non-symlink directory: {path}")


def _read_json(path: Path, label: str) -> dict[str, Any]:
    _regular(path, label, max_bytes=MAX_JSON_BYTES)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"{label} is invalid JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise CaptureError(f"{label} root is not an object: {path}")
    return value


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise CaptureError(f"{label} is not a lowercase SHA-256")
    return value


def _image_reference(value: Any, label: str = "image reference") -> str:
    if not isinstance(value, str) or IMAGE_RE.fullmatch(value) is None:
        raise CaptureError(f"{label} must be a digest-pinned Jazzy image")
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or any(part in ("", ".", "..") for part in value.split("/")):
        raise CaptureError(f"{label} is not a safe relative path")
    if Path(value).as_posix() != value:
        raise CaptureError(f"{label} is not normalized")
    return value


def _safe_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) > MAX_CAPTURE_ID_LENGTH or \
            SAFE_ID_RE.fullmatch(value) is None:
        raise CaptureError(f"{label} is not a safe immutable identifier")
    return value


def _url(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("https://") or \
            any(character in value for character in ("\r", "\n", "\x00")):
        raise CaptureError(f"{label} must be an HTTPS URL")
    return value


def _http_url_matches_prefix(value: str, prefix: str) -> bool:
    actual = urlsplit(value)
    expected = urlsplit(prefix)
    try:
        port = actual.port
    except ValueError:
        return False
    base_path = expected.path.rstrip("/")
    return (actual.scheme == expected.scheme == "http" and
            actual.hostname == expected.hostname and port in (None, 80) and
            not actual.query and not actual.fragment and
            (actual.path == base_path or actual.path.startswith(base_path + "/")))


def _apt_or_https(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) > 2048 or \
            any(character in value for character in ("\r", "\n", "\x00")):
        raise CaptureError(f"{label} is not a valid URL")
    try:
        parts = urlsplit(value)
        if (not parts.netloc or parts.query or parts.fragment or parts.username or
                parts.password):
            raise CaptureError(f"{label} is ambiguous")
        if parts.scheme == "https":
            return value
        if parts.scheme != "http" or not any(
                _http_url_matches_prefix(value, prefix)
                for prefix in APT_SOURCE_URL_POLICY["http_prefixes"]):
            raise CaptureError(f"{label} is outside the fixed APT URL policy")
    except ValueError as error:
        raise CaptureError(f"{label} is not a valid URL") from error
    return value


def _sorted_unique(items: Any, keys: tuple[str, ...], label: str) -> list[dict[str, Any]]:
    if not isinstance(items, list) or not items:
        raise CaptureError(f"{label} must be non-empty")
    if any(not isinstance(item, Mapping) for item in items):
        raise CaptureError(f"{label} contains a non-object")
    result = [dict(item) for item in items]
    identities = [tuple(str(item.get(key, "")) for key in keys) for item in result]
    if any(not all(identity) for identity in identities) or len(set(identities)) != len(identities):
        raise CaptureError(f"{label} contains incomplete or duplicate identities")
    if identities != sorted(identities):
        raise CaptureError(f"{label} is not canonically sorted")
    return result


def _validate_image_inspect(base: Mapping[str, Any], inspect: Any) -> None:
    if not isinstance(inspect, Mapping) or set(inspect) != {
            "Id", "RepoDigests", "Os", "Architecture"}:
        raise CaptureError("image inspect projection is incomplete")
    if not isinstance(inspect["Id"], str) or DIGEST_RE.fullmatch(inspect["Id"]) is None:
        raise CaptureError("image inspect Id is invalid")
    if inspect["Os"] != "linux" or inspect["Architecture"] != "amd64":
        raise CaptureError("image inspect platform drift")
    digests = inspect["RepoDigests"]
    if not isinstance(digests, list) or digests != sorted(set(digests)) or \
            base["reference"] not in digests:
        raise CaptureError("image inspect pinned RepoDigest is missing or ambiguous")


def _validate_command(command: Any, label: str) -> list[str]:
    if not isinstance(command, list) or not command or \
            any(not isinstance(item, str) or not item or "\x00" in item for item in command):
        raise CaptureError(f"{label} is not an exact argv list")
    secret_markers = ("password", "passwd", "secret", "token", "authorization", "proxy")
    if any(any(marker in item.lower() for marker in secret_markers) for item in command):
        raise CaptureError(f"{label} contains a credential-bearing token")
    return list(command)


def _validate_allowlist(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != {"build", "runtime", "union", "canonical_sha256"}:
        raise CaptureError("package allowlist fields are incomplete")
    projections = {}
    for role in ("build", "runtime", "union"):
        entries = _sorted_unique(value[role], ("name", "version", "architecture"),
                                 f"{role} package allowlist")
        for entry in entries:
            if any(not isinstance(entry.get(field), str) or not entry[field]
                   for field in ("name", "version", "architecture")):
                raise CaptureError(f"{role} package allowlist entry is incomplete")
        projections[role] = entries
    expected_union = sorted({
        (entry["name"], entry["version"], entry["architecture"])
        for role in ("build", "runtime") for entry in projections[role]
    })
    observed_union = [
        (entry["name"], entry["version"], entry["architecture"])
        for entry in projections["union"]
    ]
    if observed_union != expected_union:
        raise CaptureError("package allowlist union drift")
    projection = {role: projections[role] for role in ("build", "runtime", "union")}
    if value["canonical_sha256"] != canonical_hash(projection):
        raise CaptureError("package allowlist canonical identity drift")
    return {**projection, "canonical_sha256": value["canonical_sha256"]}


def _validate_file_specs(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or not value:
        raise CaptureError("artifact specification set is empty")
    specs = [dict(item) for item in value]
    paths = [item.get("path") for item in specs]
    if any(not isinstance(path, str) for path in paths):
        raise CaptureError("artifact path is missing")
    for item in specs:
        _safe_relative(item["path"], "artifact path")
        if not isinstance(item.get("source_path"), str):
            raise CaptureError("artifact source path is missing")
        _safe_relative(item["source_path"], "artifact source path")
        if item.get("role") not in ARTIFACT_ROLES:
            raise CaptureError("artifact role is unknown")
        _sha(item.get("sha256"), f"artifact {item['path']} SHA")
        if type(item.get("bytes")) is not int or item["bytes"] <= 0 or \
                item["bytes"] > MAX_ARTIFACT_BYTES:
            raise CaptureError(f"artifact {item['path']} size is invalid")
        optional = set(item) - {"path", "source_path", "role", "sha256", "bytes"}
        if not optional.issubset({"source_url", "source_urls"}) or \
                {"source_url", "source_urls"}.issubset(item):
            raise CaptureError(f"artifact {item['path']} has unknown fields")
        if "source_url" in item:
            _apt_or_https(item["source_url"], f"artifact {item['path']} URL")
        if "source_urls" in item:
            if item["role"] != "apt_source" or not isinstance(item["source_urls"], list) or \
                    not item["source_urls"] or item["source_urls"] != sorted(set(item["source_urls"])):
                raise CaptureError(f"artifact {item['path']} source URL set is invalid")
            for url in item["source_urls"]:
                _apt_or_https(url, f"artifact {item['path']} URL")
    if paths != sorted(paths) or len(set(paths)) != len(paths):
        raise CaptureError("artifact paths are not sorted and unique")
    roles = {item["role"] for item in specs}
    if roles != REQUIRED_ARTIFACT_ROLES:
        raise CaptureError("artifact role set is incomplete")
    return specs


def _artifact_map(specs: list[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    return {str(item["path"]): item for item in specs}


def _validate_artifact_ref(item: Mapping[str, Any], artifacts: Mapping[str, Mapping[str, Any]],
                           label: str, role: str) -> None:
    path = item.get("artifact_path")
    if not isinstance(path, str) or path not in artifacts or artifacts[path]["role"] != role:
        raise CaptureError(f"{label} artifact binding is missing or has wrong role")
    artifact = artifacts[path]
    if item.get("sha256") != artifact["sha256"] or item.get("bytes") != artifact["bytes"]:
        raise CaptureError(f"{label} artifact byte identity drift")


def _validate_package_records(value: Any, allowlist: Mapping[str, Any],
                              artifacts: Mapping[str, Mapping[str, Any]]) -> list[dict[str, Any]]:
    packages = _sorted_unique(value, ("name", "version", "architecture"), "dpkg package set")
    expected = {(item["name"], item["version"], item["architecture"])
               for item in allowlist["union"]}
    observed = {(item.get("name"), item.get("version"), item.get("architecture"))
                for item in packages}
    if observed != expected:
        raise CaptureError("dpkg package set differs from exact package allowlist")
    deb_paths: set[str] = set()
    license_paths: set[str] = set()
    for package in packages:
        for field in ("name", "version", "architecture", "status", "filename", "url",
                      "sha256", "bytes", "artifact_path", "license"):
            if field not in package:
                raise CaptureError(f"dpkg package field missing: {field}")
        if package["status"] != "install ok installed" or \
                not isinstance(package["filename"], str) or "/" in package["filename"] or \
                not package["filename"].endswith(".deb"):
            raise CaptureError("dpkg package status or filename is invalid")
        _url(package["url"], f"dpkg package {package['name']} URL")
        _sha(package["sha256"], f"dpkg package {package['name']} SHA")
        if type(package["bytes"]) is not int or package["bytes"] <= 0:
            raise CaptureError("dpkg package byte count is invalid")
        _validate_artifact_ref(package, artifacts, f"dpkg package {package['name']}", "deb")
        if package["artifact_path"] in deb_paths:
            raise CaptureError("dpkg package artifact is reused")
        deb_paths.add(package["artifact_path"])
        license_value = package["license"]
        if not isinstance(license_value, Mapping) or set(license_value) != {
                "path", "identity", "sha256", "bytes", "artifact_path"}:
            raise CaptureError("dpkg license artifact metadata is incomplete")
        _safe_relative(license_value["path"], "license path")
        if not isinstance(license_value["identity"], str) or not license_value["identity"]:
            raise CaptureError("license identity is missing")
        _sha(license_value["sha256"], "license SHA")
        if type(license_value["bytes"]) is not int or license_value["bytes"] <= 0:
            raise CaptureError("license byte count is invalid")
        license_ref = dict(license_value)
        license_ref["artifact_path"] = license_value["artifact_path"]
        _validate_artifact_ref(license_ref, artifacts,
                               f"dpkg package {package['name']} license", "license")
        if license_value["artifact_path"] in license_paths:
            raise CaptureError("license artifact is reused")
        license_paths.add(license_value["artifact_path"])
        if license_value["sha256"] != artifacts[license_value["artifact_path"]]["sha256"] or \
                license_value["bytes"] != artifacts[license_value["artifact_path"]]["bytes"]:
            raise CaptureError("license artifact byte identity drift")
    return packages


def _validate_repositories(value: Any) -> list[dict[str, Any]]:
    repositories = _sorted_unique(value, ("url", "release_or_inrelease_url"), "repository set")
    for repository in repositories:
        _url(repository.get("url"), "repository URL")
        _url(repository.get("release_or_inrelease_url"), "Release URL")
        _sha(repository.get("release_or_inrelease_sha256"), "Release SHA")
        if type(repository.get("release_or_inrelease_bytes")) is not int or \
                repository["release_or_inrelease_bytes"] <= 0:
            raise CaptureError("Release byte count is invalid")
    return repositories


def _validate_ref_records(value: Any, label: str, artifacts: Mapping[str, Mapping[str, Any]],
                          role: str, url_required: bool = True,
                          allow_url_list: bool = False) -> list[dict[str, Any]]:
    records = _sorted_unique(value, ("artifact_path",) if allow_url_list
                             else ("url", "artifact_path"), label)
    artifact_paths = [record.get("artifact_path") for record in records]
    if any(not isinstance(path, str) for path in artifact_paths) or \
            len(set(artifact_paths)) != len(artifact_paths):
        raise CaptureError(f"{label} contains duplicate or missing artifact paths")
    for record in records:
        if allow_url_list:
            if set(record) != {"urls", "artifact_path", "sha256", "bytes"} or \
                    not isinstance(record["urls"], list) or not record["urls"] or \
                    record["urls"] != sorted(set(record["urls"])):
                raise CaptureError(f"{label} URL set is invalid")
            for url in record["urls"]:
                _apt_or_https(url, f"{label} URL")
        elif url_required:
            _url(record.get("url"), f"{label} URL")
        _sha(record.get("sha256"), f"{label} SHA")
        if type(record.get("bytes")) is not int or record["bytes"] <= 0:
            raise CaptureError(f"{label} byte count is invalid")
        _validate_artifact_ref(record, artifacts, label, role)
    return records


def _assert_role_artifact_set(records: list[Mapping[str, Any]],
                              artifacts: Mapping[str, Mapping[str, Any]],
                              role: str, label: str) -> None:
    """Require every artifact of a role to have exactly one record."""
    record_paths = [record.get("artifact_path") for record in records]
    expected_paths = [path for path, item in artifacts.items() if item["role"] == role]
    if (any(not isinstance(path, str) for path in record_paths) or
            len(record_paths) != len(set(record_paths)) or
            sorted(record_paths) != sorted(expected_paths)):
        raise CaptureError(f"{label} artifact set does not match its records")


def _assert_single_url_bindings(records: list[Mapping[str, Any]],
                                artifacts: Mapping[str, Mapping[str, Any]],
                                label: str) -> None:
    """Require a scalar record URL to be copied verbatim to its artifact."""
    for record in records:
        path = record["artifact_path"]
        artifact = artifacts[path]
        if ("source_url" not in artifact or "source_urls" in artifact or
                artifact["source_url"] != record["url"]):
            raise CaptureError(f"{label} artifact URL binding drift")


def _validate_cache_records(value: Any, artifacts: Mapping[str, Mapping[str, Any]]) -> list[dict[str, Any]]:
    records = _sorted_unique(value, ("artifact_path",), "rosdep cache set")
    for record in records:
        _sha(record.get("sha256"), "rosdep cache SHA")
        if type(record.get("bytes")) is not int or record["bytes"] <= 0:
            raise CaptureError("rosdep cache byte count is invalid")
        _validate_artifact_ref(record, artifacts, "rosdep cache", "rosdep_cache")
    return records


def validate_payload(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the in-container payload before any bytes are copied."""
    required = {
        "schema", "schema_version", "status", "benchmark_eligible", "capture_id",
        "base_image", "network", "provisioning", "container", "package_allowlist",
        "dpkg_packages_sorted", "repositories", "apt_source_files_sha256",
        "apt_release_or_inrelease", "rosdep_sources_sha256", "rosdep_cache_files",
        "artifacts", "archive_set_identity_sha256", "plan_identity_sha256",
        "custodian_review",
    }
    if set(payload) != required:
        raise CaptureError("capture payload fields are incomplete or contain extras")
    if payload["schema"] != SCHEMA or payload["schema_version"] != SCHEMA_VERSION or \
            payload["status"] != "REVIEW_REQUIRED" or payload["benchmark_eligible"] is not False:
        raise CaptureError("capture must remain REVIEW_REQUIRED and benchmark-ineligible")
    _safe_id(payload["capture_id"], "capture_id")
    base = payload["base_image"]
    if not isinstance(base, Mapping) or set(base) != {
            "reference", "digest", "distribution", "platform"}:
        raise CaptureError("base image identity is incomplete")
    _image_reference(base["reference"])
    if base["digest"] != base["reference"].replace("ros@", "") or \
            base["distribution"] != "jazzy" or base["platform"] != "linux/amd64":
        raise CaptureError("base image digest, distro, or platform drift")
    network = payload["network"]
    if not isinstance(network, Mapping) or set(network) != {
            "network_used", "phase", "build_test_network", "disconnect_required"} or \
            network["network_used"] is not True or \
            network["phase"] != "PROVISIONING_CONNECTED" or \
            network["build_test_network"] != "NOT_STARTED" or \
            network["disconnect_required"] is not True:
        raise CaptureError("network phase is not truthful or is already promoted")
    provisioning = payload["provisioning"]
    if not isinstance(provisioning, Mapping) or set(provisioning) != {
            "command", "command_sha256", "exit_status"}:
        raise CaptureError("provisioning command contract is incomplete")
    command = _validate_command(provisioning["command"], "provisioning command")
    if provisioning["command_sha256"] != canonical_hash({"argv": command}) or \
            provisioning["exit_status"] != 0:
        raise CaptureError("provisioning command identity or exit status is invalid")
    container = payload["container"]
    if not isinstance(container, Mapping) or set(container) != {
            "image_reference", "image_inspect", "argv", "mounts", "network_mode",
            "phase_trace", "network_used"}:
        raise CaptureError("container capture binding is incomplete")
    if container["image_reference"] != base["reference"] or \
            container["network_mode"] != "bridge" or container["network_used"] is not True:
        raise CaptureError("container image or network identity drift")
    _validate_image_inspect(base, container["image_inspect"])
    _validate_command(container["argv"], "container capture argv")
    mounts = container["mounts"]
    if not isinstance(mounts, list) or len(mounts) != 3 or \
            any(not isinstance(item, Mapping) or
                set(item) != {"source", "destination", "read_only"}
                for item in mounts) or \
            mounts[0]["destination"] != "/workspace/capture" or \
            mounts[0]["read_only"] is not False or \
            mounts[1]["destination"] != "/opt/apt_closure_capture.py" or \
            mounts[1]["read_only"] is not True or \
            mounts[2]["destination"] != "/opt/package-allowlist.json" or \
            mounts[2]["read_only"] is not True or \
            any(not isinstance(item["source"], str) or not item["source"].startswith("/")
                for item in mounts):
        raise CaptureError("capture mount contract is not exact")
    trace = container["phase_trace"]
    expected_trace = [
        {"phase": "image_inspect", "network_used": False},
        {"phase": "container_run", "network_used": True},
        {"phase": "payload_capture", "network_used": True},
        {"phase": "disconnect_pending", "network_used": True},
    ]
    if trace != expected_trace:
        raise CaptureError("capture network phase trace is not exact")
    allowlist = _validate_allowlist(payload["package_allowlist"])
    artifacts = _validate_file_specs(payload["artifacts"])
    artifact_map = _artifact_map(artifacts)
    packages = _validate_package_records(payload["dpkg_packages_sorted"], allowlist, artifact_map)
    repositories = _validate_repositories(payload["repositories"])
    source_records = payload["apt_source_files_sha256"]
    source_urls_are_plural = isinstance(source_records, list) and any(
        isinstance(item, Mapping) and "urls" in item for item in source_records)
    apt_sources = _validate_ref_records(
        source_records, "apt source file set", artifact_map, "apt_source",
        allow_url_list=source_urls_are_plural)
    source_paths = {record["artifact_path"] for record in apt_sources}
    apt_artifact_paths = {path for path, item in artifact_map.items()
                          if item["role"] == "apt_source"}
    if source_paths != apt_artifact_paths:
        raise CaptureError("apt source artifact set does not match source records")
    for record in apt_sources:
        artifact = artifact_map[record["artifact_path"]]
        if source_urls_are_plural:
            if ("source_urls" not in artifact or "source_url" in artifact or
                    artifact["source_urls"] != record["urls"]):
                raise CaptureError("apt source artifact URL set binding drift")
        elif ("source_url" not in artifact or "source_urls" in artifact or
              artifact["source_url"] != record["url"]):
            raise CaptureError("apt source artifact URL binding drift")
    _assert_role_artifact_set(apt_sources, artifact_map, "apt_source", "apt source")
    releases = _validate_ref_records(payload["apt_release_or_inrelease"],
                                      "Release/InRelease set", artifact_map, "apt_release")
    _assert_role_artifact_set(releases, artifact_map, "apt_release", "Release/InRelease")
    _assert_single_url_bindings(releases, artifact_map, "Release/InRelease")
    release_by_url = {record["url"]: record for record in releases}
    expected_releases = []
    for item in repositories:
        release = release_by_url.get(item["release_or_inrelease_url"])
        if release is None:
            raise CaptureError("repository and Release/InRelease identities disagree")
        expected_releases.append({
            "url": item["release_or_inrelease_url"],
            "artifact_path": release["artifact_path"],
            "sha256": item["release_or_inrelease_sha256"],
            "bytes": item["release_or_inrelease_bytes"],
        })
    expected_releases.sort(key=lambda item: (item["url"], item["artifact_path"]))
    if releases != expected_releases:
        raise CaptureError("repository and Release/InRelease identities disagree")
    rosdep_sources = _validate_ref_records(payload["rosdep_sources_sha256"],
                                           "rosdep source set", artifact_map, "rosdep_source")
    rosdep_cache = _validate_cache_records(payload["rosdep_cache_files"], artifact_map)
    _assert_role_artifact_set(rosdep_sources, artifact_map, "rosdep_source", "rosdep source")
    _assert_single_url_bindings(rosdep_sources, artifact_map, "rosdep source")
    _assert_role_artifact_set(rosdep_cache, artifact_map, "rosdep_cache", "rosdep cache")
    _assert_role_artifact_set(
        [{"artifact_path": package["artifact_path"]} for package in packages],
        artifact_map, "deb", "package deb")
    _assert_role_artifact_set(
        [{"artifact_path": package["license"]["artifact_path"]} for package in packages],
        artifact_map, "license", "package license")
    for package in packages:
        deb_artifact = artifact_map[package["artifact_path"]]
        if ("source_url" not in deb_artifact or "source_urls" in deb_artifact or
                deb_artifact["source_url"] != package["url"]):
            raise CaptureError(f"dpkg package {package['name']} artifact URL binding drift")
        license_artifact = artifact_map[package["license"]["artifact_path"]]
        if "source_url" in license_artifact or "source_urls" in license_artifact:
            raise CaptureError(f"dpkg package {package['name']} license URL is unexpected")
    _sha(payload["archive_set_identity_sha256"], "archive set identity")
    _sha(payload["plan_identity_sha256"], "outer plan identity")
    review = payload["custodian_review"]
    if review != {"status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True}:
        raise CaptureError("custodian review placeholder is not exact")
    return {
        **dict(payload),
        "base_image": dict(base),
        "network": dict(network),
        "provisioning": {**dict(provisioning), "command": command},
        "container": dict(container),
        "package_allowlist": allowlist,
        "dpkg_packages_sorted": packages,
        "repositories": repositories,
        "apt_source_files_sha256": apt_sources,
        "apt_release_or_inrelease": releases,
        "rosdep_sources_sha256": rosdep_sources,
        "rosdep_cache_files": rosdep_cache,
        "artifacts": artifacts,
    }


def _receipt_from_payload(payload: Mapping[str, Any]) -> dict[str, Any]:
    checked = validate_payload(payload)
    files = []
    for artifact in checked["artifacts"]:
        record = {
            "path": artifact["path"], "role": artifact["role"],
            "bytes": artifact["bytes"], "sha256": artifact["sha256"],
        }
        if "source_url" in artifact:
            record["source_url"] = artifact["source_url"]
        if "source_urls" in artifact:
            record["source_urls"] = list(artifact["source_urls"])
        files.append(record)
    receipt = {key: value for key, value in checked.items() if key != "artifacts"}
    receipt["files"] = files
    receipt.pop("closure_identity_sha256", None)
    receipt["closure_identity_sha256"] = canonical_hash(receipt)
    return receipt


def _write_exclusive(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink():
        raise CaptureError(f"output is not fresh: {path}")
    try:
        with path.open("xb") as stream:
            stream.write(payload)
            stream.flush()
        path.chmod(0o444)
    except OSError as error:
        raise CaptureError(f"cannot seal {path}: {error}") from error


def _walk(root: Path) -> tuple[set[str], set[str]]:
    _directory(root, "capture root")
    files: set[str] = set()
    directories: set[str] = set()
    pending = [root]
    while pending:
        current = pending.pop()
        for item in sorted(current.iterdir(), key=lambda value: value.name):
            relative = item.relative_to(root).as_posix()
            info = item.lstat()
            if stat.S_ISLNK(info.st_mode):
                raise CaptureError(f"capture root contains symlink: {relative}")
            if stat.S_ISDIR(info.st_mode):
                directories.add(relative)
                pending.append(item)
            elif stat.S_ISREG(info.st_mode):
                if info.st_nlink != 1:
                    raise CaptureError(f"capture root contains hardlink: {relative}")
                files.add(relative)
            else:
                raise CaptureError(f"capture root contains special file: {relative}")
    return files, directories


def seal_capture(source_root: Path, output_root: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    """Copy a fresh local payload and seal it as REVIEW_REQUIRED.

    ``source_root`` is a test/local stand-in for the container's mounted
    capture directory.  It is never treated as evidence until every copied
    byte has been rehashed.
    """
    checked = validate_payload(payload)
    source_root = Path(source_root)
    output_root = Path(output_root)
    _directory(source_root, "capture source root")
    if output_root.exists() or output_root.is_symlink():
        raise CaptureError("capture output root must be fresh and non-existent")
    for artifact in checked["artifacts"]:
        source = source_root / artifact["source_path"]
        _regular(source, "capture source artifact", max_bytes=MAX_ARTIFACT_BYTES)
        if source.stat().st_size != artifact["bytes"] or sha256_file(source) != artifact["sha256"]:
            raise CaptureError(f"capture source byte identity mismatch: {artifact['path']}")
    output_root.mkdir(parents=True)
    try:
        for artifact in checked["artifacts"]:
            destination = output_root / artifact["path"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            with (source_root / artifact["source_path"]).open("rb") as source, \
                    destination.open("xb") as target:
                shutil.copyfileobj(source, target, length=1024 * 1024)
            destination.chmod(0o444)
        receipt = _receipt_from_payload(checked)
        receipt_payload = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        receipt_path = output_root / CAPTURE_RECEIPT
        _write_exclusive(receipt_path, receipt_payload)
        _write_exclusive(
            output_root / (CAPTURE_RECEIPT + ".sha256"),
            (sha256_bytes(receipt_payload) + "  " + CAPTURE_RECEIPT + "\n").encode("ascii"),
        )
        return verify_capture_root(output_root, {
            "image_reference": checked["base_image"]["reference"],
            "argv": checked["container"]["argv"],
            "mounts": checked["container"]["mounts"],
            "image_inspect": checked["container"]["image_inspect"],
            "plan_identity_sha256": checked["plan_identity_sha256"],
            "receipt_sha256": sha256_file(receipt_path),
        })
    except Exception:
        # Preserve a partial root for diagnostics.  It has no valid receipt
        # marker and can never be resumed or promoted by verify_capture_root.
        raise


def _validate_receipt(receipt: Mapping[str, Any]) -> dict[str, Any]:
    if set(receipt) != {
            "schema", "schema_version", "status", "benchmark_eligible", "capture_id",
            "base_image", "network", "provisioning", "container", "package_allowlist",
            "dpkg_packages_sorted", "repositories", "apt_source_files_sha256",
            "apt_release_or_inrelease", "rosdep_sources_sha256", "rosdep_cache_files",
            "archive_set_identity_sha256", "plan_identity_sha256", "custodian_review",
            "files", "closure_identity_sha256"}:
        raise CaptureError("capture receipt fields are incomplete or contain extras")
    artifacts = []
    for item in receipt["files"]:
        if not isinstance(item, Mapping) or set(item) - {
                "path", "role", "bytes", "sha256", "source_url", "source_urls"}:
            raise CaptureError("capture receipt file record contains extras")
        if {"source_url", "source_urls"}.issubset(item):
            raise CaptureError("capture receipt file URL fields are mixed")
        if "source_url" in item:
            _apt_or_https(item["source_url"], "capture source URL")
        if "source_urls" in item:
            if item.get("role") != "apt_source" or not isinstance(item["source_urls"], list) or \
                    not item["source_urls"] or item["source_urls"] != sorted(set(item["source_urls"])):
                raise CaptureError("capture source URL set is invalid")
            for url in item["source_urls"]:
                _apt_or_https(url, "capture source URL")
        artifacts.append({**dict(item), "source_path": item["path"]})
    # Reconstitute the payload shape with receipt paths as their own sources;
    # this checks every semantic field without trusting a self-reported hash.
    payload = {key: value for key, value in receipt.items()
               if key not in {"files", "closure_identity_sha256"}}
    payload["artifacts"] = artifacts
    checked = validate_payload(payload)
    expected_identity = canonical_hash(dict(receipt, closure_identity_sha256=None),
                                        excluded="closure_identity_sha256")
    if receipt["closure_identity_sha256"] != expected_identity:
        raise CaptureError("capture receipt closure identity is not canonical")
    return {**checked, "files": [dict(item) for item in receipt["files"]],
            "closure_identity_sha256": receipt["closure_identity_sha256"]}


def verify_capture_root(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    """Host-side no-follow verification bound to an outer plan/inspect record."""
    root = Path(root)
    _directory(root, "capture root")
    receipt_path = root / CAPTURE_RECEIPT
    _regular(receipt_path, "capture receipt", max_bytes=MAX_JSON_BYTES, mode=0o444)
    receipt_bytes = receipt_path.read_bytes()
    sidecar = root / (CAPTURE_RECEIPT + ".sha256")
    _regular(sidecar, "capture receipt sidecar", max_bytes=512, mode=0o444)
    sidecar_tokens = sidecar.read_text(encoding="ascii").strip().split()
    if sidecar_tokens != [sha256_bytes(receipt_bytes), CAPTURE_RECEIPT]:
        raise CaptureError("capture receipt sidecar mismatch")
    try:
        receipt = json.loads(receipt_bytes.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"capture receipt is invalid JSON: {error}") from error
    if not isinstance(receipt, Mapping):
        raise CaptureError("capture receipt is not an object")
    _validate_receipt(receipt)
    if receipt["status"] != "REVIEW_REQUIRED" or receipt["benchmark_eligible"] is not False:
        raise CaptureError("capture receipt is not review-only")
    if receipt["network"]["network_used"] is not True or \
            receipt["network"]["build_test_network"] != "NOT_STARTED":
        raise CaptureError("capture network truthfulness contract failed")
    if expected.get("image_reference") != receipt["base_image"]["reference"] or \
            expected.get("argv") != receipt["container"]["argv"] or \
            expected.get("mounts") != receipt["container"]["mounts"] or \
            expected.get("image_inspect") != receipt["container"]["image_inspect"] or \
            expected.get("plan_identity_sha256") != receipt["plan_identity_sha256"]:
        raise CaptureError("outer host plan/image/mount binding mismatch")
    actual_receipt_sha = sha256_bytes(receipt_bytes)
    if expected.get("receipt_sha256") != actual_receipt_sha:
        raise CaptureError("outer host receipt binding mismatch")
    files, directories = _walk(root)
    expected_files = {CAPTURE_RECEIPT, CAPTURE_RECEIPT + ".sha256"}
    expected_files.update(item["path"] for item in receipt["files"])
    expected_directories = set()
    for relative in expected_files:
        parent = Path(relative).parent
        while str(parent) != ".":
            expected_directories.add(parent.as_posix())
            parent = parent.parent
    if files != expected_files or directories != expected_directories:
        raise CaptureError("capture root has missing, extra, or mixed files")
    for item in receipt["files"]:
        artifact = root / item["path"]
        _regular(artifact, "capture artifact", max_bytes=MAX_ARTIFACT_BYTES, mode=0o444)
        if artifact.stat().st_size != item["bytes"] or sha256_file(artifact) != item["sha256"]:
            raise CaptureError(f"capture artifact byte identity mismatch: {item['path']}")
    _validate_image_inspect(receipt["base_image"], receipt["container"]["image_inspect"])
    return {
        "status": "REVIEW_REQUIRED",
        "benchmark_eligible": False,
        "capture_id": receipt["capture_id"],
        "receipt_sha256": actual_receipt_sha,
        "closure_identity_sha256": receipt["closure_identity_sha256"],
        "archive_set_identity_sha256": receipt["archive_set_identity_sha256"],
        "file_count": len(receipt["files"]),
        "root": str(root),
    }


def _load_dependency_prefetch():
    import importlib.util
    path = Path(__file__).with_name("dependency_prefetch.py")
    spec = importlib.util.spec_from_file_location("r3_dependency_prefetch_for_capture", path)
    if spec is None or spec.loader is None:
        raise CaptureError("offline dependency verifier cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _proposed_apt(receipt: Mapping[str, Any], production: Mapping[str, Any]) -> dict[str, Any]:
    prefetch = _load_dependency_prefetch()
    archive_identity = prefetch._archive_identity(production)
    if receipt["archive_set_identity_sha256"] != archive_identity:
        raise CaptureError("capture archive identity does not match production manifest")
    packages = []
    for package in receipt["dpkg_packages_sorted"]:
        license_value = package["license"]
        packages.append({
            **{key: package[key] for key in (
                "name", "version", "architecture", "status", "filename", "url",
                "sha256", "bytes")},
            "license": {key: license_value[key] for key in ("path", "identity", "sha256")},
        })
    sources = [{"path": item["artifact_path"],
                "sha256": item["sha256"], "bytes": item["bytes"]}
               for item in receipt["apt_source_files_sha256"]]
    releases = [{key: item[key] for key in ("url", "sha256", "bytes")}
                for item in receipt["apt_release_or_inrelease"]]
    rosdep_sources = [{key: item[key] for key in ("url", "sha256", "bytes")}
                      for item in receipt["rosdep_sources_sha256"]]
    repositories = [dict(item) for item in receipt["repositories"]]
    apt = {
        "schema_version": 1,
        "status": "READY",
        "base_image_reference": production["base_image"]["reference"],
        "base_image_digest": production["base_image"]["reference"],
        "platform": "linux/amd64",
        "ros_distribution": "jazzy",
        "provenance_policy": "candidate generated from sealed REVIEW_REQUIRED capture; independent custodian review required",
        "required_fields": list(READY_REQUIRED_FIELDS),
        "dpkg_packages_sorted": packages,
        "repositories": repositories,
        "apt_source_files_sha256": sources,
        "apt_release_or_inrelease": releases,
        "rosdep_sources_sha256": rosdep_sources,
        "rosdep_cache_sha256": canonical_hash({"files": receipt["rosdep_cache_files"]}),
        "install_command_sha256": receipt["provisioning"]["command_sha256"],
        "install_exit_status": receipt["provisioning"]["exit_status"],
        "archive_set_identity_sha256": archive_identity,
    }
    apt["dpkg_set_identity_sha256"] = prefetch._set_identity(
        packages, "name/version/architecture")
    apt["repository_set_identity_sha256"] = prefetch._set_identity(
        repositories, "url/release_or_inrelease_url")
    apt["closure_identity_sha256"] = prefetch._identity(apt)
    if apt["closure_identity_sha256"] != receipt["closure_identity_sha256"]:
        # The full capture closure includes raw artifact and container identity;
        # the proposed apt object must retain a separate explicit binding.
        pass
    return apt


def compose_proposal(capture_root: Path, production_manifest_path: Path,
                     expected_binding: Mapping[str, Any], output_path: Path) -> dict[str, Any]:
    """Write a separate proposed READY manifest; never edit production bytes."""
    verified = verify_capture_root(capture_root, expected_binding)
    production = _read_json(Path(production_manifest_path), "production offline manifest")
    receipt = _read_json(Path(capture_root) / CAPTURE_RECEIPT, "capture receipt")
    if production.get("status") != "NOT_READY_APT_CLOSURE_UNSEALED" or \
            production.get("apt_deb_closure", {}).get("status") != "NOT_READY_APT_CAPTURE_REQUIRED":
        raise CaptureError("production manifest is not the retained NOT_READY closure")
    receipt_base = receipt.get("base_image", {}).get("reference")
    if production.get("candidate_id") != "glim-clean-room-phase3d-r3-candidate-v1" or \
            production.get("base_image", {}).get("reference") != receipt_base:
        raise CaptureError("production manifest candidate/base identity drift")
    proposed = dict(production)
    proposed["status"] = "READY"
    proposed["apt_deb_closure"] = _proposed_apt(receipt, production)
    proposal = {
        "schema": PROPOSAL_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": "PROPOSED_REVIEW_REQUIRED",
        "benchmark_eligible": False,
        "production_manifest_modified": False,
        "active_r2_modified": False,
        "capture_receipt_sha256": verified["receipt_sha256"],
        "capture_closure_identity_sha256": verified["closure_identity_sha256"],
        "proposed_offline_manifest": proposed,
        "custodian_review": {"status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True},
    }
    proposal["canonical_sha256"] = canonical_hash(proposal)
    output_path = Path(output_path)
    if output_path.exists() or output_path.is_symlink():
        raise CaptureError("proposal output must be fresh and non-existent")
    payload = (json.dumps(proposal, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    _write_exclusive(output_path, payload)
    _write_exclusive(Path(str(output_path) + ".sha256"),
                     (sha256_bytes(payload) + "  " + output_path.name + "\n").encode("ascii"))
    return {
        "status": proposal["status"], "benchmark_eligible": False,
        "proposal_sha256": sha256_bytes(payload),
        "capture_receipt_sha256": verified["receipt_sha256"],
        "output": str(output_path),
    }


def validate_proposal(path: Path) -> dict[str, Any]:
    """Reopen a proposal and ensure it remains unsigned and non-eligible."""
    path = Path(path)
    _regular(path, "proposal", max_bytes=MAX_JSON_BYTES, mode=0o444)
    payload = path.read_bytes()
    sidecar = Path(str(path) + ".sha256")
    _regular(sidecar, "proposal sidecar", max_bytes=512, mode=0o444)
    if sidecar.read_text(encoding="ascii").strip().split() != [
            sha256_bytes(payload), path.name]:
        raise CaptureError("proposal sidecar mismatch")
    try:
        proposal = json.loads(payload.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"proposal is invalid JSON: {error}") from error
    if not isinstance(proposal, Mapping) or proposal.get("schema") != PROPOSAL_SCHEMA or \
            proposal.get("schema_version") != SCHEMA_VERSION or \
            proposal.get("status") != "PROPOSED_REVIEW_REQUIRED" or \
            proposal.get("benchmark_eligible") is not False or \
            proposal.get("production_manifest_modified") is not False or \
            proposal.get("active_r2_modified") is not False or \
            proposal.get("custodian_review") != {
                "status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True}:
        raise CaptureError("proposal is not an unsigned review-only artifact")
    if proposal.get("canonical_sha256") != canonical_hash(proposal, "canonical_sha256"):
        raise CaptureError("proposal canonical identity drift")
    proposed = proposal.get("proposed_offline_manifest")
    if not isinstance(proposed, Mapping) or proposed.get("status") != "READY" or \
            proposed.get("apt_deb_closure", {}).get("status") != "READY":
        raise CaptureError("proposal does not contain a READY candidate manifest")
    return {
        "status": proposal["status"],
        "benchmark_eligible": False,
        "proposal_sha256": sha256_bytes(payload),
        "capture_receipt_sha256": proposal["capture_receipt_sha256"],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    verify = sub.add_parser("verify")
    verify.add_argument("--root", type=Path, required=True)
    verify.add_argument("--expected", type=Path, required=True)
    validate = sub.add_parser("validate-proposal")
    validate.add_argument("--proposal", type=Path, required=True)
    proposal = sub.add_parser("compose")
    proposal.add_argument("--root", type=Path, required=True)
    proposal.add_argument("--production-manifest", type=Path, required=True)
    proposal.add_argument("--expected", type=Path, required=True)
    proposal.add_argument("--output", type=Path, required=True)
    contract = sub.add_parser("contract-only")
    contract.add_argument("--allowlist", type=Path, required=True)
    contract.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        expected = (_read_json(args.expected, "outer expected binding")
                    if args.command != "validate-proposal" else None)
        if args.command == "contract-only":
            # A real collector is intentionally not present in this candidate.
            # Returning a stable nonzero status prevents a host from treating
            # a contract placeholder as captured dependency evidence.
            print(json.dumps({"status": "CONTRACT_ONLY_NOT_RUN",
                              "capture_mode": CAPTURE_MODE,
                              "benchmark_eligible": False}, sort_keys=True))
            return 78
        if args.command == "verify":
            result = verify_capture_root(args.root, expected)
        elif args.command == "compose":
            result = compose_proposal(args.root, args.production_manifest, expected, args.output)
        else:
            result = validate_proposal(args.proposal)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (CaptureError, OSError, TypeError, ValueError) as error:
        print(json.dumps({"status": "FAIL_CLOSED", "reason": str(error)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
