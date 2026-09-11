#!/usr/bin/env python3
"""Verify and copy the Phase 3d r3 dependency set without network access.

This module is intentionally usable from both the host and the offline image.
It never opens a URL and never invokes a package manager.  A prefetch is a
copy from a caller-owned local directory after exact allowlist, link, size,
and SHA checks.  The candidate's apt/deb closure is deliberately NOT_READY;
therefore the production manifest cannot be promoted by this tool until an
independently captured closure is reviewed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any, Mapping


MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_ARCHIVE_BYTES = 512 * 1024 * 1024
MAX_RECEIPT_BYTES = 2 * 1024 * 1024
RECEIPT_KIND = "glim_clean_room_r3_prefetch_receipt_v1"
MANIFEST_KIND = "glim_clean_room_phase3d_r3_offline_dependency_manifest_v1"


class PrefetchError(ValueError):
    """Raised when an offline dependency closure cannot be proven."""


def sha256_file(path: Path, *, max_bytes: int = MAX_ARCHIVE_BYTES) -> str:
    _regular(path, "file", max_bytes=max_bytes)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, max_bytes: int) -> None:
    if path.is_symlink() or not path.is_file():
        raise PrefetchError(f"{label} is not a regular file: {path}")
    stat_result = path.stat()
    if stat_result.st_nlink != 1:
        raise PrefetchError(f"{label} is a hardlink or shared inode: {path}")
    if stat_result.st_size <= 0 or stat_result.st_size > max_bytes:
        raise PrefetchError(f"{label} has invalid size: {path}")


def _read_json(path: Path, *, max_bytes: int = MAX_JSON_BYTES) -> dict[str, Any]:
    _regular(path, "JSON", max_bytes=max_bytes)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PrefetchError(f"invalid JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise PrefetchError(f"JSON root is not an object: {path}")
    return value


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8")


SHA_RE = re.compile(r"^[0-9a-f]{64}$")
READY_REQUIRED_FIELDS = frozenset({
    "base_image_digest", "platform", "ros_distribution", "dpkg_packages_sorted",
    "repositories", "apt_source_files_sha256", "apt_release_or_inrelease",
    "deb_filename_url_sha256_bytes_license", "rosdep_sources_sha256",
    "rosdep_cache_sha256", "install_command_sha256", "install_exit_status",
    "archive_set_identity_sha256", "dpkg_set_identity_sha256",
    "repository_set_identity_sha256", "closure_identity_sha256",
})


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise PrefetchError(f"{label} must be lowercase SHA-256")
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            ".." in Path(value).parts or "\\" in value:
        raise PrefetchError(f"{label} is not a safe relative path")
    return value


def _identity(value: Any) -> str:
    if isinstance(value, Mapping):
        projection = dict(value)
        projection.pop("closure_identity_sha256", None)
        value = projection
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _archive_identity(manifest: Mapping[str, Any]) -> str:
    archives = sorted(manifest["archives"], key=lambda item: item["name"])
    return _identity(archives)


def _set_identity(values: list[Mapping[str, Any]], key: str) -> str:
    return _identity(sorted(values, key=lambda item: tuple(str(item[field]) for field in key.split("/"))))


def _validate_unique_sorted(values: list[Mapping[str, Any]], keys: tuple[str, ...], label: str) -> None:
    if not values:
        raise PrefetchError(f"READY {label} set must be non-empty")
    if any(not isinstance(value, Mapping) for value in values):
        raise PrefetchError(f"READY {label} set contains a non-object")
    identities = [tuple(str(value.get(key, "")) for key in keys) for value in values]
    if any(not all(identity) for identity in identities):
        raise PrefetchError(f"READY {label} identity fields are incomplete")
    if len(set(identities)) != len(identities):
        raise PrefetchError(f"READY {label} set contains duplicates")
    if identities != sorted(identities):
        raise PrefetchError(f"READY {label} set is not canonically sorted")


def _validate_license(value: Any, label: str) -> None:
    if not isinstance(value, Mapping) or \
            not isinstance(value.get("identity"), str) or not value["identity"] or \
            not isinstance(value.get("sha256"), str) or SHA_RE.fullmatch(value["sha256"]) is None:
        raise PrefetchError(f"{label} license provenance is incomplete")
    _safe_relative(value.get("path"), f"{label} license path")


def _validate_ready_apt(manifest: Mapping[str, Any]) -> Mapping[str, Any]:
    apt = manifest["apt_deb_closure"]
    if apt.get("status") != "READY":
        return apt
    if apt.get("schema_version") != 1:
        raise PrefetchError("READY apt/deb schema version is unsupported")
    if not isinstance(apt.get("provenance_policy"), str) or \
            not apt["provenance_policy"].strip():
        raise PrefetchError("READY apt/deb provenance policy is missing")
    if set(apt.get("required_fields", [])) != READY_REQUIRED_FIELDS:
        raise PrefetchError("READY apt/deb required_fields are incomplete or drifted")
    base = manifest["base_image"]
    if apt.get("base_image_reference") != base.get("reference") or \
            apt.get("base_image_digest") != base.get("reference") or \
            apt.get("platform") != base.get("architecture") or \
            apt.get("ros_distribution") != base.get("distribution"):
        raise PrefetchError("READY apt/deb base image or platform identity drift")
    packages = apt.get("dpkg_packages_sorted")
    if not isinstance(packages, list):
        raise PrefetchError("READY dpkg package set is missing")
    _validate_unique_sorted(packages, ("name", "version", "architecture"), "dpkg package")
    for package in packages:
        required = ("name", "version", "architecture", "status", "filename", "url",
                    "sha256", "bytes", "license")
        if any(not isinstance(package.get(field), str) for field in required[:4]) or \
                package.get("status") != "installed" or \
                not isinstance(package.get("filename"), str) or \
                not package["filename"].endswith(".deb") or \
                "/" in package["filename"] or "\\" in package["filename"] or \
                not isinstance(package.get("url"), str) or \
                not package["url"].startswith("https://") or \
                SHA_RE.fullmatch(str(package.get("sha256", ""))) is None or \
                not isinstance(package.get("bytes"), int) or package["bytes"] <= 0:
            raise PrefetchError("READY dpkg package provenance is incomplete")
        _validate_license(package.get("license"), f"dpkg {package['name']}")
    repositories = apt.get("repositories")
    if not isinstance(repositories, list):
        raise PrefetchError("READY repository set is missing")
    _validate_unique_sorted(repositories, ("url", "release_or_inrelease_url"), "repository")
    for repository in repositories:
        if not isinstance(repository.get("url"), str) or \
                not repository["url"].startswith("https://") or \
                not isinstance(repository.get("release_or_inrelease_url"), str) or \
                not repository["release_or_inrelease_url"].startswith("https://") or \
                SHA_RE.fullmatch(str(repository.get("release_or_inrelease_sha256", ""))) is None or \
                not isinstance(repository.get("release_or_inrelease_bytes"), int) or \
                repository["release_or_inrelease_bytes"] <= 0:
            raise PrefetchError("READY repository provenance is incomplete")
    source_files = apt.get("apt_source_files_sha256")
    if not isinstance(source_files, list):
        raise PrefetchError("READY apt source file set is missing")
    _validate_unique_sorted(source_files, ("path",), "apt source file")
    for source_file in source_files:
        _safe_relative(source_file.get("path"), "apt source file path")
        _sha(source_file.get("sha256"), "apt source file SHA")
        if not isinstance(source_file.get("bytes"), int) or source_file["bytes"] <= 0:
            raise PrefetchError("READY apt source file size is invalid")
    releases = apt.get("apt_release_or_inrelease")
    if not isinstance(releases, list):
        raise PrefetchError("READY apt Release/InRelease set is missing")
    _validate_unique_sorted(releases, ("url",), "apt Release/InRelease")
    for release in releases:
        if not isinstance(release.get("url"), str) or not release["url"].startswith("https://") or \
                SHA_RE.fullmatch(str(release.get("sha256", ""))) is None or \
                not isinstance(release.get("bytes"), int) or release["bytes"] <= 0:
            raise PrefetchError("READY apt Release/InRelease provenance is incomplete")
    expected_releases = sorted(
        [{"url": item["release_or_inrelease_url"],
          "sha256": item["release_or_inrelease_sha256"],
          "bytes": item["release_or_inrelease_bytes"]} for item in repositories],
        key=lambda item: item["url"])
    if releases != expected_releases:
        raise PrefetchError("READY repository and Release/InRelease sets disagree")
    rosdep_sources = apt.get("rosdep_sources_sha256")
    if not isinstance(rosdep_sources, list):
        raise PrefetchError("READY rosdep source set is missing")
    _validate_unique_sorted(rosdep_sources, ("url",), "rosdep source")
    for source in rosdep_sources:
        if not isinstance(source.get("url"), str) or not source["url"].startswith("https://") or \
                SHA_RE.fullmatch(str(source.get("sha256", ""))) is None or \
                not isinstance(source.get("bytes"), int) or source["bytes"] <= 0:
            raise PrefetchError("READY rosdep source provenance is incomplete")
    _sha(apt.get("rosdep_cache_sha256"), "rosdep cache SHA")
    _sha(apt.get("install_command_sha256"), "install command SHA")
    if apt.get("install_exit_status") != 0:
        raise PrefetchError("READY dependency install did not exit zero")
    if apt.get("archive_set_identity_sha256") != _archive_identity(manifest):
        raise PrefetchError("READY archive set identity drift")
    package_identity = _set_identity(packages, "name/version/architecture")
    repository_identity = _set_identity(repositories, "url/release_or_inrelease_url")
    if apt.get("dpkg_set_identity_sha256") != package_identity or \
            apt.get("repository_set_identity_sha256") != repository_identity:
        raise PrefetchError("READY package/repository set identity drift")
    if apt.get("closure_identity_sha256") != _identity(apt):
        raise PrefetchError("READY apt/deb closure identity is not canonical")
    return apt


def _manifest(path: Path) -> dict[str, Any]:
    manifest = _read_json(path)
    if manifest.get("manifest_kind") != MANIFEST_KIND or \
            manifest.get("schema_version") != 1 or \
            manifest.get("candidate_id") != "glim-clean-room-phase3d-r3-candidate-v1":
        raise PrefetchError("offline dependency manifest identity drift")
    base = manifest.get("base_image")
    if not isinstance(base, Mapping) or base.get("pull_policy") != "never" or \
            base.get("build_network") != "none":
        raise PrefetchError("offline base image/network contract drift")
    archives = manifest.get("archives")
    if not isinstance(archives, list) or len(archives) != 4:
        raise PrefetchError("offline archive closure must contain four archives")
    names: set[str] = set()
    for archive in archives:
        if not isinstance(archive, Mapping):
            raise PrefetchError("offline archive entry is not an object")
        name = archive.get("filename")
        if not isinstance(name, str) or not name or name in names or "/" in name or \
                "\\" in name or name in {".", ".."}:
            raise PrefetchError("offline archive filename is unsafe or duplicated")
        names.add(name)
        if not isinstance(archive.get("bytes"), int) or archive["bytes"] <= 0 or \
                archive["bytes"] > MAX_ARCHIVE_BYTES:
            raise PrefetchError(f"offline archive size is invalid: {name}")
        if not isinstance(archive.get("sha256"), str) or \
                len(archive["sha256"]) != 64 or \
                any(char not in "0123456789abcdef" for char in archive["sha256"]):
            raise PrefetchError(f"offline archive SHA is invalid: {name}")
        url = archive.get("url")
        if not isinstance(url, str) or not url.startswith("https://"):
            raise PrefetchError(f"offline archive URL is not official HTTPS: {name}")
        if name == "boost_1_83_0.tar.bz2":
            expected_url = (
                "https://archives.boost.io/release/1.83.0/source/"
                "boost_1_83_0.tar.bz2")
            if url != expected_url:
                raise PrefetchError("Boost URL is not the exact official release URL")
        elif re.fullmatch(
                r"https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/archive/"
                r"[0-9a-f]{40}\.tar\.gz", url) is None:
            raise PrefetchError(f"archive URL is not commit-addressed: {name}")
        licenses = archive.get("license_files")
        if not isinstance(licenses, list) or not licenses:
            raise PrefetchError(f"offline license provenance is missing: {name}")
        for license_file in licenses:
            license_path = license_file.get("path") if isinstance(
                license_file, Mapping) else None
            if not isinstance(license_file, Mapping) or not isinstance(
                    license_path, str) or not license_path or \
                    license_path.startswith("/") or ".." in Path(license_path).parts or \
                    not isinstance(license_file.get("identity"), str) or \
                    not isinstance(license_file.get("sha256"), str) or \
                    len(license_file["sha256"]) != 64:
                raise PrefetchError(f"offline license identity is invalid: {name}")
    apt = manifest.get("apt_deb_closure")
    if not isinstance(apt, Mapping) or apt.get("status") not in {
            "READY", "NOT_READY_APT_CAPTURE_REQUIRED"}:
        raise PrefetchError("apt/deb closure status is invalid")
    if apt.get("base_image_reference") != base.get("reference"):
        raise PrefetchError("apt/deb closure is bound to a different base image")
    _validate_ready_apt(manifest)
    return manifest


def _apt_names(manifest: Mapping[str, Any]) -> list[str]:
    apt = manifest["apt_deb_closure"]
    packages = apt.get("dpkg_packages_sorted", [])
    if not isinstance(packages, list):
        raise PrefetchError("apt/deb package list is invalid")
    names = ["apt-deb-closure.json"]
    for package in packages:
        if not isinstance(package, Mapping):
            raise PrefetchError("apt/deb package entry is invalid")
        filename = package.get("filename")
        if not isinstance(filename, str) or not filename.endswith(".deb") or \
                "/" in filename or "\\" in filename or filename in names:
            raise PrefetchError("apt/deb filename is unsafe or duplicated")
        names.append(filename)
    return names


def expected_data_names(manifest: Mapping[str, Any]) -> set[str]:
    names = {str(item["filename"]) for item in manifest["archives"]}
    names.update(_apt_names(manifest))
    return names


def _validate_root_shape(root: Path, *, expected: set[str], label: str) -> None:
    if root.is_symlink() or not root.is_dir():
        raise PrefetchError(f"{label} is not a regular directory: {root}")
    actual: set[str] = set()
    for item in root.iterdir():
        if item.is_symlink() or item.is_dir() or not item.is_file():
            raise PrefetchError(f"{label} contains non-regular entry: {item.name}")
        _regular(item, f"{label} entry", max_bytes=MAX_ARCHIVE_BYTES)
        actual.add(item.name)
    if actual != expected:
        raise PrefetchError(
            f"{label} file set mismatch; missing={sorted(expected - actual)}, "
            f"extra={sorted(actual - expected)}")


def _require_ready(manifest: Mapping[str, Any]) -> None:
    if manifest.get("status") != "READY" or \
            manifest["apt_deb_closure"].get("status") != "READY":
        raise PrefetchError(
            "APT_CLOSURE_NOT_READY: reviewed apt/deb manifest and package bytes are required")


def _archive_records(manifest: Mapping[str, Any], root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for archive in sorted(manifest["archives"], key=lambda value: value["filename"]):
        path = root / str(archive["filename"])
        _regular(path, "prefetched archive", max_bytes=MAX_ARCHIVE_BYTES)
        actual_size = path.stat().st_size
        actual_sha = sha256_file(path)
        if actual_size != archive["bytes"] or actual_sha != archive["sha256"]:
            raise PrefetchError(f"archive byte identity mismatch: {path.name}")
        records.append({
            "filename": path.name,
            "bytes": actual_size,
            "sha256": actual_sha,
            "url": archive["url"],
            "commit": archive.get("commit"),
            "license_files": archive["license_files"],
        })
    return records


def _apt_records(manifest: Mapping[str, Any], root: Path) -> list[dict[str, Any]]:
    closure_path = root / "apt-deb-closure.json"
    closure = _read_json(closure_path)
    expected = manifest["apt_deb_closure"]
    if closure.get("schema_version") != 1 or closure.get("status") != "READY":
        raise PrefetchError("prefetched apt/deb closure is not READY")
    if closure != expected:
        raise PrefetchError("prefetched apt/deb closure is mixed or mutated")
    records: list[dict[str, Any]] = [{
        "filename": closure_path.name,
        "bytes": closure_path.stat().st_size,
        "sha256": sha256_file(closure_path, max_bytes=MAX_JSON_BYTES),
    }]
    for package in expected.get("dpkg_packages_sorted", []):
        path = root / str(package["filename"])
        _regular(path, "prefetched deb", max_bytes=MAX_ARCHIVE_BYTES)
        if path.stat().st_size != package["bytes"] or sha256_file(path) != package["sha256"]:
            raise PrefetchError(f"deb byte identity mismatch: {path.name}")
        records.append({
            "filename": path.name,
            "bytes": path.stat().st_size,
            "sha256": package["sha256"],
            "url": package["url"],
            "license": package["license"],
        })
    return records


def _receipt_path(root: Path) -> Path:
    return root / "prefetch.receipt.json"


def verify_prefetch_root(root: Path, manifest_path: Path) -> dict[str, Any]:
    """Reopen and verify a complete prefetch root, including its receipt."""
    manifest = _manifest(manifest_path)
    _require_ready(manifest)
    expected = expected_data_names(manifest) | {_receipt_path(root).name}
    _validate_root_shape(root, expected=expected, label="prefetch root")
    archive_records = _archive_records(manifest, root)
    apt_records = _apt_records(manifest, root)
    receipt = _read_json(_receipt_path(root), max_bytes=MAX_RECEIPT_BYTES)
    manifest_sha = sha256_file(manifest_path, max_bytes=MAX_JSON_BYTES)
    if receipt.get("receipt_kind") != RECEIPT_KIND or receipt.get("status") != "PASS" or \
            receipt.get("manifest_sha256") != manifest_sha or \
            receipt.get("network_used") is not False:
        raise PrefetchError("prefetch receipt identity or network contract is invalid")
    actual_records = archive_records + apt_records
    apt = manifest["apt_deb_closure"]
    identity_projection = {
        "archive_set_identity_sha256": _archive_identity(manifest),
        "dpkg_set_identity_sha256": apt["dpkg_set_identity_sha256"],
        "repository_set_identity_sha256": apt["repository_set_identity_sha256"],
        "apt_deb_closure_identity_sha256": apt["closure_identity_sha256"],
        "apt_deb_closure_file_sha256": apt_records[0]["sha256"],
    }
    for key, value in identity_projection.items():
        if receipt.get(key) != value:
            raise PrefetchError(f"prefetch receipt canonical identity drift: {key}")
    if receipt.get("files") != actual_records:
        raise PrefetchError("prefetch receipt file projection drift")
    return {
        "status": "PASS",
        "receipt": receipt,
        "manifest_sha256": manifest_sha,
        "files": actual_records,
        "root": str(root),
    }


def prefetch_local(source_root: Path, output_root: Path, manifest_path: Path) -> dict[str, Any]:
    """Copy exact local bytes into a new root and seal a non-network receipt."""
    manifest = _manifest(manifest_path)
    _require_ready(manifest)
    if output_root.exists() or output_root.is_symlink():
        raise PrefetchError(f"prefetch output must be new and non-existent: {output_root}")
    source_expected = expected_data_names(manifest)
    _validate_root_shape(source_root, expected=source_expected, label="prefetch source")
    output_root.mkdir(parents=True)
    try:
        for name in sorted(source_expected):
            source = source_root / name
            destination = output_root / name
            _regular(source, "prefetch source", max_bytes=MAX_ARCHIVE_BYTES)
            with source.open("rb") as source_stream, destination.open("xb") as destination_stream:
                shutil.copyfileobj(source_stream, destination_stream, length=1024 * 1024)
        archive_records = _archive_records(manifest, output_root)
        apt_records = _apt_records(manifest, output_root)
        receipt = {
            "schema_version": 1,
            "receipt_kind": RECEIPT_KIND,
            "status": "PASS",
            "manifest_sha256": sha256_file(manifest_path, max_bytes=MAX_JSON_BYTES),
            "base_image_reference": manifest["base_image"]["reference"],
            "network_used": False,
            "archive_set_identity_sha256": _archive_identity(manifest),
            "dpkg_set_identity_sha256": manifest["apt_deb_closure"][
                "dpkg_set_identity_sha256"],
            "repository_set_identity_sha256": manifest["apt_deb_closure"][
                "repository_set_identity_sha256"],
            "apt_deb_closure_identity_sha256": manifest["apt_deb_closure"][
                "closure_identity_sha256"],
            "apt_deb_closure_file_sha256": apt_records[0]["sha256"],
            "files": archive_records + apt_records,
        }
        with _receipt_path(output_root).open("xb") as receipt_stream:
            receipt_stream.write(_canonical_json(receipt))
        return verify_prefetch_root(output_root, manifest_path)
    except Exception as error:
        failure = {
            "schema_version": 1,
            "receipt_kind": RECEIPT_KIND,
            "status": "FAIL_CLOSED",
            "reason": str(error),
            "network_used": False,
        }
        failure_path = output_root / "prefetch.failure.json"
        if not failure_path.exists():
            failure_path.write_bytes(_canonical_json(failure))
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    verify = subparsers.add_parser("verify")
    verify.add_argument("--root", type=Path, required=True)
    verify.add_argument("--manifest", type=Path, required=True)
    prefetch = subparsers.add_parser("prefetch")
    prefetch.add_argument("--source-root", type=Path, required=True)
    prefetch.add_argument("--output-root", type=Path, required=True)
    prefetch.add_argument("--manifest", type=Path, required=True)
    plan = subparsers.add_parser("plan")
    plan.add_argument("--manifest", type=Path, required=True)
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.command == "verify":
            result = verify_prefetch_root(args.root, args.manifest)
        elif args.command == "prefetch":
            result = prefetch_local(args.source_root, args.output_root, args.manifest)
        else:
            manifest = _manifest(args.manifest)
            result = {
                "status": "PASS" if manifest.get("status") == "READY" else "NOT_READY",
                "network_used": False,
                "expected_data_names": sorted(expected_data_names(manifest)),
            }
        print(json.dumps(result, sort_keys=True))
        return 0 if result.get("status") == "PASS" else 2
    except (OSError, PrefetchError, TypeError, ValueError) as error:
        print(json.dumps({"status": "FAIL_CLOSED", "reason": str(error)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
