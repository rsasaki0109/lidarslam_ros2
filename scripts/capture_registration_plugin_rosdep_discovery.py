#!/usr/bin/env python3
"""Capture a bounded, review-only rosdep/rosdistro discovery packet.

This command is intentionally separate from the four-leg release capture.  It
uses the already-present Humble image to discover the exact APT resolution and
the current commit of the upstream ``ros/rosdistro`` repository.  It does not
run ``rosdep update`` and it never authorizes a build, benchmark, promotion,
or active-profile change.  Every output is a fresh immutable file with a
sidecar and is reopened before the receipt is returned.

The discovery packet is exploratory evidence, not a dependency closure.  A
future candidate may consume it only after an independent review has pinned
the selected source graph and verified the APT/GPG chain.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import selectors
import stat
import subprocess
import sys
import tarfile
import time
from typing import Any, Mapping
from urllib.parse import urljoin, urlsplit
import urllib.error
import urllib.request
import uuid


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / (
    "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
)
DEFAULT_OUTPUT_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/registration-plugin-rosdep-discovery-20260827-01"
)
IMAGE_DIGEST = (
    "sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988"
)
IMAGE_REFERENCE = "docker.io/library/ros:humble-ros-core@" + IMAGE_DIGEST
SCHEMA = "registration-plugin-rosdep-discovery-v1"
SCHEMA_VERSION = 1
RECEIPT_NAME = "registration_plugin_rosdep_discovery.receipt.json"
MAX_COMMAND_OUTPUT = 16 * 1024 * 1024
MAX_ARCHIVE_BYTES = 128 * 1024 * 1024
MAX_ARCHIVE_MEMBERS = 100_000
MAX_SELECTED_FILES = 512
MAX_SELECTED_FILE_BYTES = 16 * 1024 * 1024
MAX_SELECTED_TOTAL_BYTES = 64 * 1024 * 1024
MAX_PACKAGE_BYTES = 512 * 1024 * 1024
COMMAND_TIMEOUT = 600
GIT_TIMEOUT = 45
HTTP_TIMEOUT = 45
MAX_RETRIES = 2
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
DISCOVERY_ID_RE = re.compile(
    r"^registration-plugin-rosdep-discovery-(?P<date>[0-9]{8})-(?P<seq>[0-9]{2})$"
)
PACKAGE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9+.-]*$")
VERSION_RE = re.compile(r"^[^\x00\r\n\t ]+$")
ALLOWED_PACKAGE_NAMES = (
    "python3-rosdep",
    "python3-rosdep-modules",
    "python3-rosdistro",
    "python3-rosdistro-modules",
)
APT_PREFIXES = (
    "http://archive.ubuntu.com/ubuntu",
    "http://security.ubuntu.com/ubuntu",
    "http://packages.ros.org/ros2/ubuntu",
)
GIT_ENDPOINT = "https://github.com/ros/rosdistro.git"
ARCHIVE_HOSTS = frozenset({
    "codeload.github.com", "codeload.githubusercontent.com", "github.com",
})
RAW_HOST = "raw.githubusercontent.com"
ALLOWED_CACHE_METADATA = frozenset({"lock", "partial"})
ROSDISTRO_OUTPUT_DIRECTORIES = (
    "artifacts",
    "artifacts/apt",
    "artifacts/apt/debs",
    "artifacts/rosdistro",
    "artifacts/rosdistro/files",
    "artifacts/rosdistro/files/releases",
    "artifacts/rosdistro/files/rosdep",
    "artifacts/rosdistro/files/rosdep/sources.list.d",
)
CACHE_SOURCE_REMOVE_SCRIPT = r'''
import fcntl
import hashlib
import json
import os
import re
import stat
import sys

parent, name, expected_size_text, expected_sha = sys.argv[1:]
if re.fullmatch(r"/tmp/registration-plugin-apt-[a-f0-9]{32}", parent) is None:
    raise RuntimeError("cache parent is not a generated path")
if re.fullmatch(r"[A-Za-z0-9+_.-]+", name) is None or "/" in name:
    raise RuntimeError("cache entry name is not a basename")
if re.fullmatch(r"0|[1-9][0-9]*", expected_size_text) is None:
    raise RuntimeError("expected size is not canonical")
if re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None:
    raise RuntimeError("expected SHA is not canonical")
expected_size = int(expected_size_text, 10)
flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
tmp_fd = os.open("/tmp", flags)
try:
    parent_fd = os.open(parent.rsplit("/", 1)[1], flags, dir_fd=tmp_fd)
finally:
    os.close(tmp_fd)
try:
    fcntl.flock(parent_fd, fcntl.LOCK_EX)
    first = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    if (
        not stat.S_ISREG(first.st_mode)
        or first.st_nlink != 1
        or first.st_uid != 0
        or first.st_gid != 0
        or stat.S_IMODE(first.st_mode) not in (0o600, 0o640, 0o644)
        or first.st_size != expected_size
    ):
        raise RuntimeError("cache source metadata changed")
    file_fd = os.open(
        name, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=parent_fd
    )
    try:
        opened = os.fstat(file_fd)
        if (
            (opened.st_dev, opened.st_ino, opened.st_mode, opened.st_uid,
             opened.st_gid, opened.st_nlink, opened.st_size)
            != (first.st_dev, first.st_ino, first.st_mode, first.st_uid,
                first.st_gid, first.st_nlink, first.st_size)
        ):
            raise RuntimeError("cache source was replaced")

        def digest_once():
            os.lseek(file_fd, 0, os.SEEK_SET)
            digest = hashlib.sha256()
            total = 0
            while True:
                block = os.read(file_fd, 1024 * 1024)
                if not block:
                    break
                total += len(block)
                if total > expected_size:
                    raise RuntimeError("cache source is oversized")
                digest.update(block)
            return total, digest.hexdigest()

        total_one, sha_one = digest_once()
        middle = os.fstat(file_fd)
        total_two, sha_two = digest_once()
        final = os.fstat(file_fd)
        fields = (
            "st_dev", "st_ino", "st_mode", "st_uid", "st_gid", "st_nlink",
            "st_size", "st_mtime_ns", "st_ctime_ns",
        )
        if any(getattr(first, field) != getattr(final, field) for field in fields):
            raise RuntimeError("cache source changed while hashing")
        if any(getattr(opened, field) != getattr(middle, field) for field in fields):
            raise RuntimeError("cache source changed while hashing")
        if total_one != expected_size or total_two != expected_size:
            raise RuntimeError("cache source size changed while hashing")
        if sha_one != expected_sha or sha_two != expected_sha or sha_one != sha_two:
            raise RuntimeError("cache source hash mismatch")
        proof = {
            "name": name,
            "path": parent + "/" + name,
            "type": "regular file",
            "mode": stat.S_IMODE(first.st_mode),
            "uid": first.st_uid,
            "gid": first.st_gid,
            "nlink": first.st_nlink,
            "bytes": first.st_size,
            "device": first.st_dev,
            "inode": first.st_ino,
            "sha256": sha_one,
            "fsynced": False,
            "post_absent": False,
            "removed": False,
        }
        os.unlink(name, dir_fd=parent_fd)
        os.fsync(parent_fd)
        proof["fsynced"] = True
        try:
            os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            proof["post_absent"] = True
        if not proof["post_absent"]:
            raise RuntimeError("cache source remained after unlink")
        proof["removed"] = True
        print(json.dumps(proof, sort_keys=True, separators=(",", ":")))
    finally:
        os.close(file_fd)
finally:
    os.close(parent_fd)
'''


class DiscoveryError(RuntimeError):
    """A fail-closed discovery error."""

    def __init__(self, kind: str, message: str):
        super().__init__(message)
        self.kind = kind


def canonical_bytes(value: Any) -> bytes:
    """Return the deterministic JSON representation used by this contract."""
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def canonical_hash(value: Mapping[str, Any]) -> str:
    """Hash a receipt after removing its self-referential canonical field."""
    projection = dict(value)
    projection.pop("canonical_sha256", None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def sha256_bytes(value: bytes) -> str:
    """Hash bytes without exposing their contents in the receipt."""
    return hashlib.sha256(value).hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise DiscoveryError("SHA_INVALID", label)
    return value


def _absolute(path: Any, label: str) -> Path:
    if not isinstance(path, (str, os.PathLike)):
        raise DiscoveryError("PATH_INVALID", label)
    value = os.fspath(path)
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or "\x00" in value
        or "\r" in value
        or "\n" in value
    ):
        raise DiscoveryError("PATH_INVALID", label)
    path_value = Path(value)
    if path_value.as_posix() != value or any(
        part in ("", ".", "..") for part in value.split("/")[1:]
    ):
        raise DiscoveryError("PATH_NOT_NORMALIZED", label)
    return path_value


def _relative(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value.startswith("/")
        or "\\" in value
        or "\x00" in value
        or "\r" in value
        or "\n" in value
    ):
        raise DiscoveryError("PATH_INVALID", label)
    path = PurePosixPath(value)
    if value != path.as_posix() or any(
        part in ("", ".", "..") for part in path.parts
    ):
        raise DiscoveryError("PATH_NOT_NORMALIZED", label)
    return value


def _check_parent(path: Path, label: str) -> None:
    """Reject symlink/non-directory parents without canonicalizing the path."""
    current = path.parent
    while True:
        try:
            info = os.lstat(str(current))
        except OSError as exc:
            raise DiscoveryError("PARENT_INVALID", "{}: {}".format(label, exc)) from exc
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise DiscoveryError("PARENT_INVALID", label)
        if current == Path("/"):
            return
        current = current.parent


def _fsync_parent(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        count = os.write(descriptor, payload[offset:])
        if count <= 0:
            raise DiscoveryError("WRITE_NO_PROGRESS", "output write made no progress")
        offset += count


def _open_regular(path: Path, label: str, *, writable: bool = False) -> tuple[int, os.stat_result]:
    flags = (os.O_WRONLY if writable else os.O_RDONLY) | getattr(os, "O_NOFOLLOW", 0)
    flags |= getattr(os, "O_CLOEXEC", 0)
    descriptor = os.open(str(path), flags)
    info = os.fstat(descriptor)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        os.close(descriptor)
        raise DiscoveryError("ARTIFACT_NOT_SINGLE_REGULAR", label)
    return descriptor, info


def seal_bytes(path: Path, payload: bytes) -> dict[str, Any]:
    """Create one fresh 0444 file and exact two-token SHA sidecar."""
    path = _absolute(path, "artifact path")
    if len(payload) > MAX_PACKAGE_BYTES:
        raise DiscoveryError("ARTIFACT_OVERSIZE", str(path))
    _check_parent(path, "artifact parent")
    sidecar = path.with_name(path.name + ".sha256")
    created: list[tuple[Path, int, int]] = []
    digest = sha256_bytes(payload)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    flags |= getattr(os, "O_CLOEXEC", 0)
    try:
        try:
            descriptor = os.open(str(path), flags, 0o600)
        except FileExistsError as exc:
            raise DiscoveryError("ARTIFACT_COLLISION", "fresh artifact required") from exc
        info = os.fstat(descriptor)
        created.append((path, info.st_dev, info.st_ino))
        try:
            _write_all(descriptor, payload)
            os.fchmod(descriptor, 0o444)
            os.fsync(descriptor)
            after = os.fstat(descriptor)
            if (
                not stat.S_ISREG(after.st_mode)
                or after.st_nlink != 1
                or after.st_size != len(payload)
                or stat.S_IMODE(after.st_mode) != 0o444
                or (after.st_dev, after.st_ino) != (info.st_dev, info.st_ino)
            ):
                raise DiscoveryError("ARTIFACT_IDENTITY_CHANGED", str(path))
        finally:
            os.close(descriptor)
        _fsync_parent(path.parent)
        sidecar_payload = (digest + " " + path.name + "\n").encode("ascii")
        try:
            side_descriptor = os.open(str(sidecar), flags, 0o600)
        except FileExistsError as exc:
            raise DiscoveryError("SIDECAR_COLLISION", "fresh sidecar required") from exc
        side_info = os.fstat(side_descriptor)
        created.append((sidecar, side_info.st_dev, side_info.st_ino))
        try:
            _write_all(side_descriptor, sidecar_payload)
            os.fchmod(side_descriptor, 0o444)
            os.fsync(side_descriptor)
            side_after = os.fstat(side_descriptor)
            if (
                not stat.S_ISREG(side_after.st_mode)
                or side_after.st_nlink != 1
                or side_after.st_size != len(sidecar_payload)
                or stat.S_IMODE(side_after.st_mode) != 0o444
                or (side_after.st_dev, side_after.st_ino)
                != (side_info.st_dev, side_info.st_ino)
            ):
                raise DiscoveryError("SIDECAR_IDENTITY_CHANGED", str(sidecar))
        finally:
            os.close(side_descriptor)
        _fsync_parent(path.parent)
        descriptor_value = {
            "path": str(path),
            "sidecar": str(sidecar),
            "bytes": len(payload),
            "sha256": digest,
            "mode": 0o444,
            "nlink": 1,
        }
        reopen_artifact(path, descriptor_value)
        return descriptor_value
    except Exception:
        for candidate, device, inode in reversed(created):
            try:
                info = os.lstat(str(candidate))
                if (info.st_dev, info.st_ino) == (device, inode):
                    os.unlink(str(candidate))
            except OSError:
                pass
        raise


def _read_regular(path: Path, maximum: int, label: str) -> bytes:
    descriptor, before = _open_regular(path, label)
    try:
        if before.st_size < 0 or before.st_size > maximum:
            raise DiscoveryError("ARTIFACT_OVERSIZE", label)
        chunks: list[bytes] = []
        total = 0
        while True:
            block = os.read(descriptor, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            total += len(block)
            if total > maximum:
                raise DiscoveryError("ARTIFACT_OVERSIZE", label)
            chunks.append(block)
        after = os.fstat(descriptor)
        if (
            (before.st_dev, before.st_ino, before.st_size, before.st_nlink)
            != (after.st_dev, after.st_ino, after.st_size, after.st_nlink)
        ):
            raise DiscoveryError("ARTIFACT_CHANGED", label)
        if total != before.st_size:
            raise DiscoveryError("ARTIFACT_SIZE_CHANGED", label)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def reopen_artifact(path: Path, descriptor: Mapping[str, Any]) -> dict[str, Any]:
    """Reopen an artifact and its sidecar with exact identity/content checks."""
    path = _absolute(path, "reopen artifact")
    if not isinstance(descriptor, Mapping):
        raise DiscoveryError("ARTIFACT_DESCRIPTOR_INVALID", str(path))
    if descriptor.get("path") != str(path) or descriptor.get("mode") != 0o444:
        raise DiscoveryError("ARTIFACT_DESCRIPTOR_INVALID", str(path))
    if descriptor.get("nlink") != 1 or type(descriptor.get("bytes")) is not int:
        raise DiscoveryError("ARTIFACT_DESCRIPTOR_INVALID", str(path))
    expected = _sha(descriptor.get("sha256"), "artifact SHA")
    sidecar = _absolute(descriptor.get("sidecar"), "artifact sidecar")
    if sidecar != path.with_name(path.name + ".sha256"):
        raise DiscoveryError("SIDECAR_PATH_INVALID", str(path))
    _check_parent(path, "reopen artifact parent")
    data = _read_regular(path, MAX_PACKAGE_BYTES, "artifact")
    info = os.lstat(str(path))
    if (
        stat.S_IMODE(info.st_mode) != 0o444
        or info.st_nlink != 1
        or info.st_size != descriptor["bytes"]
        or sha256_bytes(data) != expected
    ):
        raise DiscoveryError("ARTIFACT_BINDING_INVALID", str(path))
    side_data = _read_regular(sidecar, 4096, "artifact sidecar")
    if side_data.decode("ascii", "strict") != expected + " " + path.name + "\n":
        raise DiscoveryError("SIDECAR_BINDING_INVALID", str(sidecar))
    side_info = os.lstat(str(sidecar))
    if stat.S_IMODE(side_info.st_mode) != 0o444 or side_info.st_nlink != 1:
        raise DiscoveryError("SIDECAR_METADATA_INVALID", str(sidecar))
    return dict(descriptor)


def create_fresh_root(path: Path) -> dict[str, Any]:
    """Create the one fresh root and bind its host identity."""
    path = _absolute(path, "discovery root")
    _check_parent(path, "discovery root parent")
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    parent_descriptor = os.open(str(path.parent), flags)
    try:
        try:
            os.stat(path.name, dir_fd=parent_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise DiscoveryError("ROOT_NOT_FRESH", str(path))
        os.mkdir(path.name, 0o700, dir_fd=parent_descriptor)
        child = os.open(path.name, flags, dir_fd=parent_descriptor)
        try:
            os.fchmod(child, 0o700)
            os.fsync(child)
            info = os.fstat(child)
        finally:
            os.close(child)
        os.fsync(parent_descriptor)
    finally:
        os.close(parent_descriptor)
    if (
        not stat.S_ISDIR(info.st_mode)
        or stat.S_IMODE(info.st_mode) != 0o700
        or info.st_nlink < 2
        or (info.st_uid, info.st_gid) != (os.getuid(), os.getgid())
    ):
        raise DiscoveryError("ROOT_IDENTITY_INVALID", str(path))
    return {
        "path": str(path),
        "device": int(info.st_dev),
        "inode": int(info.st_ino),
        "uid": int(info.st_uid),
        "gid": int(info.st_gid),
        "mode": stat.S_IMODE(info.st_mode),
        "nlink": int(info.st_nlink),
    }


def _discovery_id_for_root(path: Path) -> str:
    """Derive and validate the discovery identifier from a fresh root name."""
    path = _absolute(path, "discovery root")
    match = DISCOVERY_ID_RE.fullmatch(path.name)
    if match is None:
        raise DiscoveryError("DISCOVERY_ID_INVALID", path.name)
    try:
        datetime.datetime.strptime(match.group("date"), "%Y%m%d")
    except ValueError as exc:
        raise DiscoveryError("DISCOVERY_ID_INVALID", path.name) from exc
    if match.group("seq") == "00":
        raise DiscoveryError("DISCOVERY_ID_INVALID", path.name)
    return path.name


def ensure_directory(root: Path, relative: str) -> Path:
    """Create a fixed descendant directory without following links."""
    relative = _relative(relative, "directory relative path")
    target = root.joinpath(*relative.split("/"))
    target.relative_to(root)
    current = root
    for component in relative.split("/"):
        current = current / component
        try:
            info = os.lstat(str(current))
        except FileNotFoundError:
            os.mkdir(str(current), 0o700)
            info = os.lstat(str(current))
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise DiscoveryError("DIRECTORY_INVALID", str(current))
        if (info.st_uid, info.st_gid) != (os.getuid(), os.getgid()):
            raise DiscoveryError("DIRECTORY_OWNER_INVALID", str(current))
        os.chmod(str(current), 0o700)
    return target


def _validated_owner(
    expected_owner: tuple[int, int] | None,
    label: str,
) -> tuple[int, int]:
    """Return the sealed owner used for directory readback.

    Directory creation is performed by the invoking host and may use the
    current uid/gid.  Readback can instead run in a root container, so it
    must use the owner recorded in the sealed root receipt.  Keeping this
    conversion in one helper also rejects boolean/non-canonical owner claims.
    """
    if expected_owner is None:
        return (os.getuid(), os.getgid())
    if (
        not isinstance(expected_owner, (tuple, list))
        or len(expected_owner) != 2
        or any(type(value) is not int or value < 0 for value in expected_owner)
    ):
        raise DiscoveryError("OWNER_INVALID", label)
    return (int(expected_owner[0]), int(expected_owner[1]))


def _layout_directory_identity(
    root: Path,
    relative: str,
    *,
    expected_owner: tuple[int, int] | None = None,
) -> dict[str, Any]:
    """Read one precreated output directory without following links.

    With no explicit owner this is the creation-time helper and derives the
    invoking process owner.  Receipt validation always supplies the sealed
    root owner explicitly; this is important when the validator runs as
    container root against a host-owned bind mount.
    """
    relative = _relative(relative, "output directory relative path")
    owner = _validated_owner(expected_owner, "output directory owner")
    path = root.joinpath(*relative.split("/"))
    _check_parent(path, "output directory parent")
    try:
        info = os.lstat(str(path))
    except OSError as exc:
        raise DiscoveryError("OUTPUT_DIRECTORY_MISSING", relative) from exc
    if (
        stat.S_ISLNK(info.st_mode)
        or not stat.S_ISDIR(info.st_mode)
        or stat.S_IMODE(info.st_mode) != 0o700
        or (info.st_uid, info.st_gid) != owner
        or info.st_nlink < 2
    ):
        raise DiscoveryError("OUTPUT_DIRECTORY_INVALID", relative)
    return {
        "path": relative,
        "type": "directory",
        "mode": stat.S_IMODE(info.st_mode),
        "uid": int(info.st_uid),
        "gid": int(info.st_gid),
        "nlink": int(info.st_nlink),
        "device": int(info.st_dev),
        "inode": int(info.st_ino),
    }


def _create_rosdistro_output_layout(root: Path) -> dict[str, Any]:
    """Create and bind every directory used by rosdistro evidence output."""
    root = _absolute(root, "output layout root")
    directories: dict[str, dict[str, Any]] = {}
    for relative in sorted(
        ROSDISTRO_OUTPUT_DIRECTORIES, key=lambda item: (item.count("/"), item)
    ):
        path = root.joinpath(*relative.split("/"))
        _check_parent(path, "output directory parent")
        if os.path.lexists(str(path)):
            raise DiscoveryError("OUTPUT_DIRECTORY_COLLISION", relative)
        try:
            os.mkdir(str(path), 0o700)
        except OSError as exc:
            raise DiscoveryError("OUTPUT_DIRECTORY_CREATE_FAILED", relative) from exc
        _fsync_parent(path.parent)
    # Capture all pre-container identities only after the complete hierarchy
    # exists; otherwise creating a descendant changes its parent's nlink.
    for relative in ROSDISTRO_OUTPUT_DIRECTORIES:
        descriptor = _layout_directory_identity(root, relative)
        directories[relative] = {"before": descriptor}
    return {
        "status": "PRECREATED",
        "allowed_directories": list(ROSDISTRO_OUTPUT_DIRECTORIES),
        "directories": directories,
    }


def _finalize_rosdistro_output_layout(
    root: Path, layout: dict[str, Any]
) -> dict[str, Any]:
    """Reopen the output layout and reject replacement or unknown directories."""
    if (
        not isinstance(layout, dict)
        or layout.get("allowed_directories") != list(ROSDISTRO_OUTPUT_DIRECTORIES)
        or set(layout.get("directories", {})) != set(ROSDISTRO_OUTPUT_DIRECTORIES)
    ):
        raise DiscoveryError("OUTPUT_LAYOUT_INVALID", "allowed directories")
    base = root / "artifacts"
    for path in base.rglob("*"):
        info = os.lstat(str(path))
        relative = path.relative_to(root).as_posix()
        if stat.S_ISLNK(info.st_mode):
            raise DiscoveryError("OUTPUT_DIRECTORY_INVALID", relative)
        if stat.S_ISDIR(info.st_mode) and relative not in ROSDISTRO_OUTPUT_DIRECTORIES:
            raise DiscoveryError("OUTPUT_DIRECTORY_UNEXPECTED", relative)
    result = {
        "status": "VERIFIED",
        "allowed_directories": list(ROSDISTRO_OUTPUT_DIRECTORIES),
        "directories": {},
    }
    for relative in ROSDISTRO_OUTPUT_DIRECTORIES:
        before = layout["directories"][relative].get("before")
        after = _layout_directory_identity(root, relative)
        if not isinstance(before, Mapping):
            raise DiscoveryError("OUTPUT_LAYOUT_INVALID", relative)
        immutable_fields = (
            "path", "type", "mode", "uid", "gid", "device", "inode", "nlink"
        )
        if any(before.get(key) != after.get(key) for key in immutable_fields):
            raise DiscoveryError("OUTPUT_DIRECTORY_CHANGED", relative)
        result["directories"][relative] = {
            "before": dict(before),
            "after": after,
        }
    return result


def _require_rosdistro_output_directory(
    root: Path, relative: str, layout: Mapping[str, Any] | None
) -> None:
    """Require a known precreated parent for a rosdistro output path."""
    relative = _relative(relative, "rosdistro output relative path")
    if layout is None:
        raise DiscoveryError("OUTPUT_LAYOUT_MISSING", relative)
    directories = layout.get("directories")
    if not isinstance(directories, Mapping) or relative not in directories:
        raise DiscoveryError("OUTPUT_DIRECTORY_UNEXPECTED", relative)
    _layout_directory_identity(root, relative)


def _json_artifact(root: Path, relative: str, value: Any) -> dict[str, Any]:
    data = canonical_bytes(value) + b"\n"
    return seal_bytes(root / _relative(relative, "JSON artifact"), data)


def _relative_descriptor(root: Path, descriptor: Mapping[str, Any]) -> dict[str, Any]:
    path = _absolute(descriptor["path"], "descriptor path")
    relative = path.relative_to(root).as_posix()
    sidecar = _absolute(descriptor["sidecar"], "descriptor sidecar")
    if sidecar != path.with_name(path.name + ".sha256"):
        raise DiscoveryError("SIDECAR_PATH_INVALID", relative)
    result = dict(descriptor)
    result["path"] = relative
    result["sidecar"] = sidecar.relative_to(root).as_posix()
    return result


class CommandCapture:
    """Run fixed argv through a bounded stdout/stderr selector."""

    def __init__(self, root: Path):
        self.root = root
        self.ordinal = 0
        self.records: list[dict[str, Any]] = []

    def run(
        self,
        argv: list[str],
        label: str,
        *,
        timeout: int = COMMAND_TIMEOUT,
        env: Mapping[str, str] | None = None,
    ) -> dict[str, Any]:
        if not argv or any(not isinstance(item, str) or "\x00" in item for item in argv):
            raise DiscoveryError("ARGV_INVALID", label)
        started = int(time.time())
        child_env = os.environ.copy()
        if env is not None:
            child_env.update(env)
        child_env.update({"LC_ALL": "C", "LANG": "C", "TZ": "UTC"})
        try:
            process = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=child_env,
                close_fds=True,
            )
        except OSError as exc:
            raise DiscoveryError("COMMAND_START_FAILED", label) from exc
        assert process.stdout is not None and process.stderr is not None
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        streams = {"stdout": bytearray(), "stderr": bytearray()}
        timed_out = False
        oversized = False
        deadline = time.monotonic() + timeout
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                timed_out = True
                process.kill()
                break
            for key, _ in selector.select(min(1.0, remaining)):
                block = key.fileobj.read1(64 * 1024)
                if not block:
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    continue
                stream = streams[key.data]
                stream.extend(block)
                if len(stream) > MAX_COMMAND_OUTPUT:
                    oversized = True
                    process.kill()
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    break
            if oversized:
                break
        selector.close()
        if timed_out or oversized:
            try:
                process.kill()
            except OSError:
                pass
        try:
            returncode = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            returncode = -9
        ended = int(time.time())
        stdout = bytes(streams["stdout"])
        stderr = bytes(streams["stderr"])
        self.ordinal += 1
        stem = "commands/{:03d}-{}".format(self.ordinal, _safe_stem(label))
        ensure_directory(self.root, "commands")
        stdout_descriptor = _relative_descriptor(
            self.root, seal_bytes(self.root / (stem + ".stdout"), stdout))
        stderr_descriptor = _relative_descriptor(
            self.root, seal_bytes(self.root / (stem + ".stderr"), stderr))
        record = {
            "label": label,
            "argv": list(argv),
            "argv_sha256": sha256_bytes(canonical_bytes(argv)),
            "returncode": returncode,
            "timed_out": timed_out,
            "oversized": oversized,
            "started_at_unix": started,
            "ended_at_unix": ended,
            "stdout": stdout_descriptor,
            "stderr": stderr_descriptor,
        }
        record_descriptor = _relative_descriptor(
            self.root, _json_artifact(self.root, stem + ".json", record))
        record["record"] = record_descriptor
        self.records.append(record)
        return record


def _safe_stem(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,63}", value):
        raise DiscoveryError("LABEL_INVALID", value)
    return value


def validate_archive_url(url: str, commit: str | None = None) -> str:
    """Validate the exact codeload endpoint and optional commit binding."""
    if not isinstance(url, str) or "\x00" in url:
        raise DiscoveryError("ARCHIVE_URL_INVALID", "not a URL")
    parts = urlsplit(url)
    if (
        parts.scheme != "https"
        or parts.hostname not in ARCHIVE_HOSTS
        or parts.username is not None
        or parts.password is not None
        or parts.port is not None
        or parts.query
        or parts.fragment
    ):
        raise DiscoveryError("ARCHIVE_URL_INVALID", url)
    path = parts.path
    if not (
        path.startswith("/ros/rosdistro/")
        and ("/tar.gz/" in path or "/legacy.tar.gz/" in path or "/archive/" in path)
    ):
        raise DiscoveryError("ARCHIVE_URL_INVALID", url)
    if commit is not None and commit not in path:
        raise DiscoveryError("ARCHIVE_URL_COMMIT_MISMATCH", url)
    return url


def _validate_git_endpoint(url: str) -> str:
    parts = urlsplit(url)
    if (
        parts.scheme != "https"
        or parts.hostname != "github.com"
        or parts.path != "/ros/rosdistro.git"
        or parts.username is not None
        or parts.password is not None
        or parts.port is not None
        or parts.query
        or parts.fragment
    ):
        raise DiscoveryError("GIT_URL_INVALID", url)
    return url


def _validate_raw_url(url: str, relative: str, commit: str) -> str:
    parts = urlsplit(url)
    prefix = "/ros/rosdistro/"
    if (
        parts.scheme != "https"
        or parts.hostname != RAW_HOST
        or parts.username is not None
        or parts.password is not None
        or parts.port is not None
        or parts.query
        or parts.fragment
        or not parts.path.startswith(prefix)
    ):
        raise DiscoveryError("ROS_DISTRO_URL_INVALID", url)
    tail = parts.path[len(prefix):]
    if not tail.startswith("master/") or tail[7:] != relative:
        raise DiscoveryError("ROS_DISTRO_URL_PATH_INVALID", url)
    return "https://raw.githubusercontent.com/ros/rosdistro/{}/{}".format(
        commit, relative)


def parse_apt_package_records(data: bytes, repository_base: str) -> list[dict[str, Any]]:
    """Parse only complete apt-cache stanzas with signed-index fields."""
    try:
        text = data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("APT_METADATA_NOT_UTF8", "package metadata") from exc
    if not repository_base.endswith("/"):
        repository_base += "/"
    records = []
    for stanza in re.split(r"\n\s*\n", text.strip()):
        fields: dict[str, str] = {}
        current: str | None = None
        for line in stanza.splitlines():
            if line.startswith((" ", "\t")):
                if current is None:
                    raise DiscoveryError("APT_STANZA_INVALID", "continuation without field")
                fields[current] += "\n" + line[1:]
                continue
            if ":" not in line:
                raise DiscoveryError("APT_STANZA_INVALID", line)
            key, value = line.split(":", 1)
            if not key or key in fields:
                raise DiscoveryError("APT_STANZA_DUPLICATE", key)
            current = key
            fields[key] = value.lstrip(" ")
        required = {"Package", "Version", "Architecture", "Filename", "Size", "SHA256"}
        if not required.issubset(fields):
            continue
        name = fields["Package"]
        version = fields["Version"]
        filename = fields["Filename"]
        size = fields["Size"]
        digest = fields["SHA256"]
        if (
            PACKAGE_NAME_RE.fullmatch(name) is None
            or VERSION_RE.fullmatch(version) is None
            or fields["Architecture"] not in {"all", "amd64"}
            or not filename.startswith("pool/")
            or "/" in filename and any(part in {"", ".", ".."} for part in filename.split("/"))
            or re.fullmatch(r"0|[1-9][0-9]*", size) is None
            or digest != digest.lower()
            or SHA_RE.fullmatch(digest) is None
        ):
            raise DiscoveryError("APT_PACKAGE_RECORD_INVALID", name)
        size_value = int(size, 10)
        if size_value <= 0 or size_value > MAX_PACKAGE_BYTES:
            raise DiscoveryError("APT_PACKAGE_SIZE_INVALID", name)
        records.append({
            "name": name,
            "version": version,
            "architecture": fields["Architecture"],
            "filename": filename,
            "size_bytes": size_value,
            "sha256": digest,
            "uri": urljoin(repository_base, filename),
        })
    if not records:
        raise DiscoveryError("APT_PACKAGE_RECORDS_EMPTY", "no complete package records")
    return records


def _parse_print_uris(data: bytes) -> dict[str, str]:
    try:
        text = data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("APT_URI_OUTPUT_NOT_UTF8", "print uris") from exc
    result: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"^'([^']+)'\s+(\S+)(?:\s+.*)?$", line.strip())
        if match is None:
            continue
        url, filename = match.groups()
        parts = urlsplit(url)
        if parts.scheme not in {"http", "https"} or parts.username or parts.password:
            raise DiscoveryError("APT_URI_INVALID", url)
        if not any(
            url == prefix or url.startswith(prefix + "/") for prefix in APT_PREFIXES
        ):
            raise DiscoveryError("APT_URI_NOT_ALLOWLISTED", url)
        if filename in result and result[filename] != url:
            raise DiscoveryError("APT_URI_DUPLICATE", filename)
        result[filename] = url
    return result


def _validate_uri_set(
    expected: list[dict[str, Any]], uris: Mapping[str, str]
) -> dict[str, str]:
    """Require one exact URI for each selected Package filename."""
    expected_names = [PurePosixPath(item["filename"]).name for item in expected]
    if len(expected_names) != len(set(expected_names)):
        raise DiscoveryError("APT_PACKAGE_FILENAME_DUPLICATE", "signed Package records")
    if not isinstance(uris, Mapping) or set(uris) != set(expected_names):
        missing = sorted(set(expected_names) - set(uris))
        extra = sorted(set(uris) - set(expected_names)) if isinstance(uris, Mapping) else []
        raise DiscoveryError(
            "APT_URI_SET_INVALID",
            "missing=" + ",".join(missing) + ";extra=" + ",".join(extra),
        )
    return {name: uris[name] for name in expected_names}


def _parse_package_output(data: bytes) -> list[dict[str, str]]:
    try:
        text = data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("DPKG_OUTPUT_NOT_UTF8", "dpkg query") from exc
    rows = []
    for line in text.splitlines():
        fields = line.split("\t")
        if len(fields) != 4 or fields[3] != "install ok installed":
            raise DiscoveryError("DPKG_ROW_INVALID", line)
        rows.append({"name": fields[0], "version": fields[1], "architecture": fields[2]})
    if len(rows) != len(ALLOWED_PACKAGE_NAMES) or {
        row["name"] for row in rows
    } != set(ALLOWED_PACKAGE_NAMES):
        raise DiscoveryError("DPKG_PACKAGE_SET_INVALID", "installed package set")
    return rows


def _exact_package_specs(rows: list[dict[str, str]]) -> list[str]:
    """Return deterministic ``name=version`` specs for the target packages."""
    if not isinstance(rows, list) or len(rows) != len(ALLOWED_PACKAGE_NAMES):
        raise DiscoveryError("APT_PACKAGE_SET_INVALID", "exact package specs")
    seen: set[str] = set()
    specs = []
    for row in rows:
        if not isinstance(row, Mapping):
            raise DiscoveryError("APT_PACKAGE_SET_INVALID", "package row")
        name = row.get("name")
        version = row.get("version")
        if (
            not isinstance(name, str)
            or name not in ALLOWED_PACKAGE_NAMES
            or name in seen
            or not isinstance(version, str)
            or VERSION_RE.fullmatch(version) is None
        ):
            raise DiscoveryError("APT_PACKAGE_SET_INVALID", "exact package specs")
        seen.add(name)
        specs.append(name + "=" + version)
    if seen != set(ALLOWED_PACKAGE_NAMES):
        raise DiscoveryError("APT_PACKAGE_SET_INVALID", "exact package specs")
    return sorted(specs)


def _parse_cache_inventory(data: bytes) -> dict[str, dict[str, Any]]:
    """Parse exact stat descriptors for a fresh apt archive directory."""
    try:
        text = data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("APT_CACHE_INVENTORY_NOT_UTF8", "cache inventory") from exc
    result: dict[str, dict[str, Any]] = {}
    for line in text.splitlines():
        fields = line.split("\t")
        if len(fields) != 9:
            raise DiscoveryError("APT_CACHE_INVENTORY_INVALID", line)
        path, kind, mode, uid, gid, nlink, size, device, inode = fields
        try:
            normalized_path = _absolute(path, "cache inventory path")
        except DiscoveryError:
            raise DiscoveryError("APT_CACHE_INVENTORY_INVALID", path)
        name = normalized_path.name
        if (
            not name
            or name in {".", ".."}
            or "/" in name
            or "\\" in name
            or "\x00" in name
            or name in result
        ):
            raise DiscoveryError("APT_CACHE_INVENTORY_INVALID", name)
        if kind not in {"directory", "regular file", "regular empty file"}:
            raise DiscoveryError("APT_CACHE_INVENTORY_INVALID", kind)
        if (
            re.fullmatch(r"[0-7]+", mode) is None
                or mode != format(int(mode, 8), "o")
            or re.fullmatch(r"[0-9]+", uid) is None
            or re.fullmatch(r"[0-9]+", gid) is None
            or re.fullmatch(r"[0-9]+", nlink) is None
            or re.fullmatch(r"[0-9]+", size) is None
            or re.fullmatch(r"[0-9]+", device) is None
            or re.fullmatch(r"[0-9]+", inode) is None
        ):
            raise DiscoveryError("APT_CACHE_INVENTORY_INVALID", name)
        result[name] = {
            "name": name,
            "path": path,
            "type": kind,
            "mode": int(mode, 8),
            "uid": int(uid, 10),
            "gid": int(gid, 10),
            "nlink": int(nlink, 10),
            "bytes": int(size, 10),
            "device": int(device, 10),
            "inode": int(inode, 10),
        }
    return result


def _parse_cache_identity(data: bytes, cache_path: str) -> dict[str, Any]:
    """Parse the identity descriptor for the cache directory itself."""
    try:
        text = data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", cache_path) from exc
    lines = text.splitlines()
    if len(lines) != 1:
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", cache_path)
    fields = lines[0].split("\t")
    if len(fields) != 9 or fields[0] != cache_path:
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", cache_path)
    path, kind, mode, uid, gid, nlink, size, device, inode = fields
    if (
        re.fullmatch(r"[0-7]+", mode) is None
        or mode != format(int(mode, 8), "o")
        or any(re.fullmatch(r"[0-9]+", value) is None for value in fields[3:])
    ):
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", cache_path)
    identity = {
        "path": path,
        "type": kind,
        "mode": int(mode, 8),
        "uid": int(uid, 10),
        "gid": int(gid, 10),
        "nlink": int(nlink, 10),
        "bytes": int(size, 10),
        "device": int(device, 10),
        "inode": int(inode, 10),
    }
    if identity["type"] != "directory" or identity["mode"] != 0o700:
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", cache_path)
    return identity


def _cache_immutable_identity(identity: Mapping[str, Any]) -> dict[str, Any]:
    """Project cache identity fields that must not change during cleanup."""
    return {
        key: identity[key]
        for key in ("path", "type", "mode", "uid", "gid", "device", "inode")
    }


def _validate_cache_lifecycle(
    identity_before: Mapping[str, Any],
    identity_after: Mapping[str, Any],
    metadata_names: set[str],
    inventory_after: Mapping[str, Any],
) -> None:
    """Validate cache identity while permitting only expected cleanup changes."""
    if metadata_names not in ({"lock"}, {"lock", "partial"}):
        raise DiscoveryError("APT_CACHE_METADATA_INVALID", identity_before["path"])
    for identity in (identity_before, identity_after):
        if (
            identity.get("type") != "directory"
            or identity.get("mode") != 0o700
            or identity.get("uid") != 0
            or identity.get("gid") != 0
        ):
            raise DiscoveryError("APT_CACHE_IDENTITY_CHANGED", identity_before["path"])
    if _cache_immutable_identity(identity_after) != _cache_immutable_identity(
        identity_before
    ):
        raise DiscoveryError("APT_CACHE_IDENTITY_CHANGED", identity_before["path"])
    # A freshly-created cache has two links with only ``lock``; the
    # ``partial`` child contributes exactly one additional directory link.
    # Cleanup removes those entries and must return the directory to nlink=2.
    expected_before_nlink = 3 if metadata_names == {"lock", "partial"} else 2
    if identity_before["nlink"] != expected_before_nlink:
        raise DiscoveryError("APT_CACHE_NLINK_INVALID", identity_before["path"])
    if identity_after["nlink"] != 2:
        raise DiscoveryError("APT_CACHE_NLINK_INVALID", identity_before["path"])
    if not isinstance(inventory_after, Mapping) or identity_after["bytes"] < 0:
        raise DiscoveryError("APT_CACHE_IDENTITY_INVALID", identity_before["path"])
    if inventory_after:
        raise DiscoveryError("APT_CACHE_NOT_EMPTY", identity_before["path"])


SOURCE_REMOVAL_FIELDS = frozenset({
    "name", "path", "type", "mode", "uid", "gid", "nlink", "bytes",
    "device", "inode", "sha256", "fsynced", "post_absent", "removed",
})


def _validate_source_removal_evidence(
    evidence: Mapping[str, Any],
    cache_path: str,
    item: Mapping[str, Any],
    baseline: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate one dirfd-based source inode removal proof."""
    if not isinstance(evidence, Mapping):
        raise DiscoveryError("APT_DEB_SOURCE_EVIDENCE_INVALID", str(item))
    if set(evidence) != SOURCE_REMOVAL_FIELDS:
        raise DiscoveryError(
            "APT_DEB_SOURCE_EVIDENCE_INVALID", str(evidence.get("name"))
        )
    filename = PurePosixPath(item.get("filename", "")).name
    expected_path = cache_path + "/" + filename
    if (
        evidence.get("name") != filename
        or evidence.get("path") != expected_path
        or evidence.get("type") != "regular file"
        or evidence.get("mode") not in {0o600, 0o640, 0o644}
        or evidence.get("uid") != 0
        or evidence.get("gid") != 0
        or evidence.get("nlink") != 1
        or evidence.get("bytes") != item.get("size_bytes")
        or evidence.get("sha256") != item.get("sha256")
        or evidence.get("fsynced") is not True
        or evidence.get("post_absent") is not True
        or evidence.get("removed") is not True
    ):
        raise DiscoveryError("APT_DEB_SOURCE_EVIDENCE_INVALID", filename)
    for key in ("mode", "uid", "gid", "nlink", "bytes", "device", "inode"):
        if type(evidence.get(key)) is not int or evidence[key] < 0:
            raise DiscoveryError("APT_DEB_SOURCE_EVIDENCE_INVALID", filename)
    _sha(evidence.get("sha256"), "source removal SHA")
    if not isinstance(baseline, Mapping):
        raise DiscoveryError("APT_DEB_SOURCE_BASELINE_INVALID", filename)
    for key in (
        "path", "type", "mode", "uid", "gid", "nlink", "bytes", "device",
        "inode",
    ):
        if evidence.get(key) != baseline.get(key):
            raise DiscoveryError("APT_DEB_SOURCE_IDENTITY_CHANGED", filename)
    return dict(evidence)


def _partial_source_removal_state(
    partial_state: dict[str, Any] | None,
    filename: str,
    record: Mapping[str, Any],
) -> dict[str, Any] | None:
    """Create a mutable partial projection before source deletion begins."""
    if partial_state is None:
        return None
    details = partial_state.setdefault("cache_details", {})
    removals = details.setdefault("source_removals", [])
    entry = {
        "filename": filename,
        "status": "ATTEMPTED",
        "command_record": dict(record["record"]),
    }
    removals.append(entry)
    return entry


def _remove_verified_cache_deb(
    root: Path,
    command: CommandCapture,
    container_name: str,
    cache_path: str,
    item: Mapping[str, Any],
    baseline: Mapping[str, Any],
    *,
    index: int,
    partial_state: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Remove exactly one verified cache inode using a container dirfd."""
    original_filename = item.get("filename", "")
    filename = PurePosixPath(original_filename).name
    if (
        not filename.endswith(".deb")
        or not isinstance(original_filename, str)
        or original_filename.split("/")[-1] != filename
        or not re.fullmatch(r"[A-Za-z0-9+_.-]+", filename)
    ):
        raise DiscoveryError("APT_DEB_SOURCE_NAME_INVALID", filename)
    record = command.run(
        [
            "docker", "exec", container_name, "python3", "-c",
            CACHE_SOURCE_REMOVE_SCRIPT, cache_path, filename,
            str(item.get("size_bytes")), str(item.get("sha256")),
        ],
        "apt_deb_source_remove_{:03d}".format(index),
        timeout=120,
    )
    partial_entry = _partial_source_removal_state(
        partial_state, filename, record
    )
    try:
        _require_command_ok(record, "apt_deb_source_remove_" + filename)
        try:
            raw = _read_record_stream(root, record, "stdout")
            evidence = json.loads(raw.decode("utf-8", "strict"))
        except (UnicodeDecodeError, json.JSONDecodeError, OSError) as exc:
            raise DiscoveryError(
                "APT_DEB_SOURCE_EVIDENCE_INVALID", filename
            ) from exc
        validated = _validate_source_removal_evidence(
            evidence, cache_path, item, baseline
        )
    except Exception as exc:
        if partial_entry is not None:
            partial_entry["status"] = "FAILED"
            partial_entry["failure"] = {
                "kind": getattr(exc, "kind", "APT_DEB_SOURCE_REMOVE_FAILED"),
                "message": str(exc),
            }
        raise
    if partial_entry is not None:
        partial_entry["status"] = "REMOVED"
        partial_entry["source"] = dict(validated)
    return {
        "filename": filename,
        "source": dict(validated),
        "command_record": dict(record["record"]),
    }


def _parse_sha256sum(data: bytes, expected_path: str) -> str:
    """Parse one exact sha256sum record for a fixed cache metadata file."""
    try:
        text = data.decode("ascii", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("APT_CACHE_HASH_INVALID", expected_path) from exc
    match = re.fullmatch(r"([0-9a-f]{64})  (.+)\n", text)
    if match is None or match.group(2) != expected_path:
        raise DiscoveryError("APT_CACHE_HASH_INVALID", expected_path)
    return match.group(1)


def _classify_cache_entries(
    inventory: Mapping[str, Mapping[str, Any]],
    expected: list[dict[str, Any]],
) -> tuple[set[str], set[str]]:
    """Split one cache inventory into target debs and allowed metadata."""
    expected_names = {
        PurePosixPath(item["filename"]).name for item in expected
    }
    if len(expected_names) != len(expected):
        raise DiscoveryError("APT_DEB_DUPLICATE", "expected package set")
    names = set(inventory)
    package_names = names & expected_names
    metadata_names = names - package_names
    missing = expected_names - package_names
    unknown = metadata_names - ALLOWED_CACHE_METADATA
    if missing or unknown:
        raise DiscoveryError(
            "APT_DEB_CACHE_SET_INVALID",
            "missing=" + ",".join(sorted(missing))
            + ";extra=" + ",".join(sorted(unknown)),
        )
    return package_names, metadata_names


def _validate_cache_package_set(
    inventory: Mapping[str, Mapping[str, Any]],
    expected: list[dict[str, Any]],
    classified: tuple[set[str], set[str]] | None = None,
) -> tuple[set[str], set[str]]:
    """Require exact deb cardinality without counting cache metadata."""
    expected_names = {
        PurePosixPath(item["filename"]).name for item in expected
    }
    package_names, metadata_names = (
        _classify_cache_entries(inventory, expected)
        if classified is None else classified
    )
    if package_names != expected_names or not metadata_names <= ALLOWED_CACHE_METADATA:
        missing = sorted(expected_names - package_names)
        extra = sorted(package_names - expected_names)
        raise DiscoveryError(
            "APT_DEB_CACHE_SET_INVALID",
            "missing=" + ",".join(missing) + ";extra=" + ",".join(extra),
        )
    return package_names, metadata_names


def _validate_cache_metadata(
    inventory: Mapping[str, Mapping[str, Any]],
    expected: list[dict[str, Any]],
    lock_sha256: str,
    classified: tuple[set[str], set[str]] | None = None,
) -> dict[str, dict[str, Any]]:
    """Validate the only allowed fresh-cache metadata and package set."""
    package_names, metadata_names = _validate_cache_package_set(
        inventory, expected, classified
    )
    lock = inventory.get("lock")
    if lock is None or (
        lock.get("type") != "regular empty file"
        or lock.get("mode") != 0o640
        or lock.get("uid") != 0
        or lock.get("gid") != 0
        or lock.get("nlink") != 1
        or lock.get("bytes") != 0
        or not str(lock.get("path", "")).endswith("/lock")
    ):
        raise DiscoveryError("APT_CACHE_LOCK_METADATA_INVALID", "lock")
    if lock_sha256 != hashlib.sha256(b"").hexdigest():
        raise DiscoveryError("APT_CACHE_LOCK_HASH_INVALID", "lock")
    partial = inventory.get("partial")
    if partial is not None and (
        partial.get("type") != "directory"
        or partial.get("mode") != 0o700
        or partial.get("uid") != 100
        or partial.get("gid") != 0
        or partial.get("nlink") != 2
        or partial.get("bytes") != 4096
    ):
        raise DiscoveryError("APT_CACHE_PARTIAL_METADATA_INVALID", "partial")
    expected_by_name = {
        PurePosixPath(item["filename"]).name: item for item in expected
    }
    for name in package_names:
        item = inventory[name]
        if (
            item.get("type") not in {"regular file", "regular empty file"}
            or item.get("mode") not in {0o600, 0o640, 0o644}
            or item.get("uid") != 0
            or item.get("gid") != 0
            or item.get("nlink") != 1
            or item.get("bytes") != expected_by_name[name]["size_bytes"]
        ):
            raise DiscoveryError("APT_DEB_CACHE_ARTIFACT_METADATA_INVALID", name)
    return {
        name: dict(inventory[name])
        for name in sorted(package_names | metadata_names)
    }


def _validate_deb_name_set(
    expected: list[dict[str, Any]], actual: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    """Require exactly one downloaded artifact for every expected package."""
    expected_names: list[str] = []
    for item in expected:
        filename = PurePosixPath(item["filename"]).name
        if filename != item["filename"].split("/")[-1] or not filename.endswith(".deb"):
            raise DiscoveryError("APT_DEB_FILENAME_INVALID", str(item.get("filename")))
        if filename in expected_names:
            raise DiscoveryError("APT_DEB_DUPLICATE", filename)
        expected_names.append(filename)
    if not isinstance(actual, Mapping):
        raise DiscoveryError("APT_DEB_ARTIFACTS_INVALID", "mapping required")
    actual_names = list(actual)
    if len(actual_names) != len(set(actual_names)):
        raise DiscoveryError("APT_DEB_DUPLICATE", "downloaded artifact")
    if set(actual_names) != set(expected_names):
        missing = sorted(set(expected_names) - set(actual_names))
        extra = sorted(set(actual_names) - set(expected_names))
        detail = "missing=" + ",".join(missing) + ";extra=" + ",".join(extra)
        raise DiscoveryError("APT_DEB_SET_INVALID", detail)
    return {name: actual[name] for name in expected_names}


def _bind_downloaded_debs(
    root: Path,
    expected: list[dict[str, Any]],
    staged: Mapping[str, Path | bytes],
    output_layout: Mapping[str, Any] | None = None,
) -> dict[str, dict[str, Any]]:
    """Verify staged bytes against signed Package records and seal each once."""
    selected = _validate_deb_name_set(expected, staged)
    if output_layout is None:
        ensure_directory(root, "artifacts/apt/debs")
    else:
        _require_rosdistro_output_directory(root, "artifacts/apt/debs", output_layout)
    result: dict[str, dict[str, Any]] = {}
    for item in expected:
        filename = PurePosixPath(item["filename"]).name
        source = selected[filename]
        if isinstance(source, bytes):
            data = source
        elif isinstance(source, Path):
            data = _read_regular(source, MAX_PACKAGE_BYTES, "staged deb " + filename)
        else:
            raise DiscoveryError("APT_DEB_ARTIFACT_INVALID", filename)
        if len(data) != item["size_bytes"] or sha256_bytes(data) != item["sha256"]:
            raise DiscoveryError("APT_DEB_ARTIFACT_BINDING_INVALID", filename)
        destination = root / "artifacts/apt/debs" / filename
        descriptor = seal_bytes(destination, data)
        result[filename] = descriptor
    return result


def _validate_deb_descriptor_map(
    root: Path,
    expected: list[dict[str, Any]],
    descriptors: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    """Reopen an exact sealed deb set and compare signed size/hash bindings."""
    selected = _validate_deb_name_set(expected, descriptors)
    result: dict[str, dict[str, Any]] = {}
    for item in expected:
        filename = PurePosixPath(item["filename"]).name
        descriptor = selected[filename]
        if not isinstance(descriptor, Mapping):
            raise DiscoveryError("APT_DEB_ARTIFACT_INVALID", filename)
        path = _absolute(descriptor.get("path"), "deb artifact path")
        expected_descriptor = dict(descriptor)
        if path.is_relative_to(root):
            expected_descriptor["path"] = str(path)
        else:
            raise DiscoveryError("APT_DEB_ARTIFACT_PATH_INVALID", filename)
        sidecar = _absolute(descriptor.get("sidecar"), "deb artifact sidecar")
        expected_descriptor["sidecar"] = str(sidecar)
        reopened = reopen_artifact(path, expected_descriptor)
        if (
            reopened.get("bytes") != item["size_bytes"]
            or reopened.get("sha256") != item["sha256"]
        ):
            raise DiscoveryError("APT_DEB_ARTIFACT_BINDING_INVALID", filename)
        result[filename] = reopened
    return result


def _archive_member_safe(member: tarfile.TarInfo) -> str:
    name = member.name
    if (
        not isinstance(name, str)
        or "\x00" in name
        or "\\" in name
        or name.startswith("/")
    ):
        raise DiscoveryError("ARCHIVE_MEMBER_UNSAFE", name)
    path = PurePosixPath(name)
    if any(part in {"", ".", ".."} for part in path.parts):
        raise DiscoveryError("ARCHIVE_MEMBER_UNSAFE", name)
    if member.isdir():
        return path.as_posix()
    if not member.isfile():
        raise DiscoveryError("ARCHIVE_MEMBER_NOT_REGULAR", name)
    if member.size < 0 or member.size > MAX_SELECTED_FILE_BYTES:
        raise DiscoveryError("ARCHIVE_MEMBER_OVERSIZE", name)
    return path.as_posix()


def _source_records(source_data: bytes, commit: str) -> list[dict[str, Any]]:
    try:
        text = source_data.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("ROSDEP_SOURCE_NOT_UTF8", "20-default.list") from exc
    records = []
    for line_number, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) not in {2, 3} or fields[0] not in {"yaml", "gbpdistro"}:
            raise DiscoveryError("ROSDEP_SOURCE_LINE_INVALID", str(line_number))
        source_url = fields[1]
        parts = urlsplit(source_url)
        prefix = "/ros/rosdistro/master/"
        if parts.hostname != RAW_HOST or not parts.path.startswith(prefix):
            raise DiscoveryError("ROSDEP_SOURCE_HOST_INVALID", source_url)
        relative = parts.path[len(prefix):]
        _relative(relative, "rosdep source relative path")
        canonical_url = _validate_raw_url(source_url, relative, commit)
        record = {
            "line": line_number,
            "type": fields[0],
            "requested_url": source_url,
            "commit_url": canonical_url,
            "relative_path": relative,
        }
        if len(fields) == 3:
            record["distribution"] = fields[2]
        records.append(record)
    if not records:
        raise DiscoveryError("ROSDEP_SOURCE_EMPTY", "20-default.list")
    paths = [item["relative_path"] for item in records]
    if len(paths) != len(set(paths)):
        raise DiscoveryError("ROSDEP_SOURCE_DUPLICATE", "duplicate source path")
    return records


def inspect_rosdistro_archive(
    archive_path: Path,
    root: Path,
    commit: str,
    command: CommandCapture,
    output_layout: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Inspect and selectively seal the index/source files from a tarball."""
    archive_size = archive_path.stat().st_size
    if archive_size <= 0 or archive_size > MAX_ARCHIVE_BYTES:
        raise DiscoveryError("ROSDISTRO_ARCHIVE_SIZE_INVALID", str(archive_path))
    selected: dict[str, tarfile.TarInfo] = {}
    top_levels: set[str] = set()
    member_count = 0
    with tarfile.open(str(archive_path), mode="r:gz") as stream:
        for member in stream:
            member_count += 1
            if member_count > MAX_ARCHIVE_MEMBERS:
                raise DiscoveryError("ROSDISTRO_ARCHIVE_MEMBER_LIMIT", "archive")
            name = _archive_member_safe(member)
            parts = name.split("/")
            top_levels.add(parts[0])
            relative = "/".join(parts[1:])
            if relative in {"index.yaml", "index-v4.yaml"} or relative == (
                "rosdep/sources.list.d/20-default.list"
            ):
                selected[relative] = member
        if len(top_levels) != 1:
            raise DiscoveryError("ROSDISTRO_ARCHIVE_ROOT_INVALID", "multiple roots")
        source_member = selected.get("rosdep/sources.list.d/20-default.list")
        if source_member is None:
            raise DiscoveryError("ROSDEP_SOURCE_MISSING", "archive source list")
        source_file = stream.extractfile(source_member)
        if source_file is None:
            raise DiscoveryError("ROSDEP_SOURCE_READ_FAILED", "archive source list")
        source_data = source_file.read(MAX_SELECTED_FILE_BYTES + 1)
        if len(source_data) > MAX_SELECTED_FILE_BYTES:
            raise DiscoveryError("ROSDEP_SOURCE_OVERSIZE", "archive source list")
        source_records = _source_records(source_data, commit)
        for record in source_records:
            relative = record["relative_path"]
            member = next(
                (
                    item
                    for item in stream.getmembers()
                    if "/".join(item.name.split("/")[1:]) == relative
                ),
                None,
            )
            if member is None:
                raise DiscoveryError("ROSDEP_SOURCE_FILE_MISSING", relative)
            _archive_member_safe(member)
            if relative in selected:
                raise DiscoveryError("ROSDISTRO_FILE_DUPLICATE", relative)
            selected[relative] = member
        if len(selected) > MAX_SELECTED_FILES:
            raise DiscoveryError("ROSDISTRO_SELECTED_FILE_LIMIT", "archive")
        index_relative = "index-v4.yaml" if "index-v4.yaml" in selected else "index.yaml"
        if index_relative not in selected:
            raise DiscoveryError("ROSDISTRO_INDEX_MISSING", "index-v4.yaml/index.yaml")
        selected_total = 0
        file_records = []
        _require_rosdistro_output_directory(
            root, "artifacts/rosdistro/files", output_layout
        )
        for relative in sorted(selected):
            member = selected[relative]
            extracted = stream.extractfile(member)
            if extracted is None:
                raise DiscoveryError("ROSDISTRO_FILE_READ_FAILED", relative)
            data = extracted.read(MAX_SELECTED_FILE_BYTES + 1)
            if len(data) > MAX_SELECTED_FILE_BYTES or len(data) != member.size:
                raise DiscoveryError("ROSDISTRO_FILE_SIZE_INVALID", relative)
            selected_total += len(data)
            if selected_total > MAX_SELECTED_TOTAL_BYTES:
                raise DiscoveryError("ROSDISTRO_SELECTED_BYTES_LIMIT", "archive")
            output_parent = PurePosixPath(relative).parent
            output_directory = "artifacts/rosdistro/files"
            if str(output_parent) != ".":
                output_directory += "/" + output_parent.as_posix()
            _require_rosdistro_output_directory(
                root, output_directory, output_layout
            )
            descriptor = seal_bytes(
                root / "artifacts/rosdistro/files" / relative, data
            )
            file_records.append({
                "relative_path": relative,
                "archive_member": member.name,
                "bytes": len(data),
                "sha256": sha256_bytes(data),
                "artifact": _relative_descriptor(root, descriptor),
            })
    return {
        "archive_top_level": sorted(top_levels)[0],
        "member_count": member_count,
        "index_path": index_relative,
        "source_list_path": "rosdep/sources.list.d/20-default.list",
        "source_records": source_records,
        "selected_files": file_records,
        "selected_total_bytes": sum(item["bytes"] for item in file_records),
    }


class RedirectPolicy(urllib.request.HTTPRedirectHandler):
    """Record and constrain redirects for the commit-addressed archive."""

    def __init__(self, commit: str):
        super().__init__()
        self.commit = commit
        self.redirects: list[str] = []

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        validate_archive_url(newurl, self.commit)
        self.redirects.append(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def fetch_archive(
    url: str,
    commit: str,
    root: Path,
    command: CommandCapture,
    output_layout: Mapping[str, Any] | None = None,
) -> tuple[dict[str, Any], Path]:
    """Fetch one bounded archive with explicit redirect/retry evidence."""
    validate_archive_url(url, commit)
    if output_layout is None:
        ensure_directory(root, "artifacts/rosdistro")
    else:
        _require_rosdistro_output_directory(
            root, "artifacts/rosdistro", output_layout
        )
    partial = bytearray()
    last_error: str | None = None
    requested = url
    final_url = None
    redirect_chain: list[str] = []
    for attempt in range(MAX_RETRIES + 1):
        handler = RedirectPolicy(commit)
        opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), handler
        )
        try:
            request = urllib.request.Request(
                requested,
                headers={"Accept": "application/gzip", "User-Agent": "lidarslam-r3-discovery/1"},
                method="GET",
            )
            with opener.open(request, timeout=HTTP_TIMEOUT) as response:
                final_url = response.geturl()
                validate_archive_url(final_url, commit)
                redirect_chain = list(handler.redirects)
                content_length = response.headers.get("Content-Length")
                if content_length is not None:
                    if not re.fullmatch(r"0|[1-9][0-9]*", content_length):
                        raise DiscoveryError("ARCHIVE_LENGTH_INVALID", "Content-Length")
                    if int(content_length) > MAX_ARCHIVE_BYTES:
                        raise DiscoveryError("ARCHIVE_OVERSIZE", "Content-Length")
                partial = bytearray()
                while True:
                    block = response.read(1024 * 1024)
                    if not block:
                        break
                    partial.extend(block)
                    if len(partial) > MAX_ARCHIVE_BYTES:
                        raise DiscoveryError("ARCHIVE_OVERSIZE", "download")
                break
        except (DiscoveryError, OSError, urllib.error.URLError, urllib.error.HTTPError) as exc:
            last_error = str(exc)
            if attempt >= MAX_RETRIES:
                raise DiscoveryError("ROSDISTRO_ARCHIVE_FETCH_FAILED", last_error) from exc
    if not partial or final_url is None:
        raise DiscoveryError("ROSDISTRO_ARCHIVE_EMPTY", "archive response")
    archive_path = root / "artifacts/rosdistro" / ("rosdistro-{}-archive.tar.gz".format(commit))
    archive_descriptor = seal_bytes(archive_path, bytes(partial))
    fetch_record = {
        "requested_url": requested,
        "final_url": final_url,
        "redirects": redirect_chain,
        "attempts": attempt + 1,
        "archive": _relative_descriptor(root, archive_descriptor),
    }
    fetch_descriptor = _json_artifact(
        root, "artifacts/rosdistro/fetch.json", fetch_record
    )
    fetch_record["record"] = _relative_descriptor(root, fetch_descriptor)
    return fetch_record, archive_path


def _profile_binding(profile_path: Path) -> dict[str, Any]:
    profile_data = json.loads(_read_regular(profile_path, 32 * 1024 * 1024, "profile"))
    if not isinstance(profile_data, Mapping):
        raise DiscoveryError("PROFILE_INVALID", "profile object")
    for distro in profile_data.get("distros", []):
        if isinstance(distro, Mapping) and distro.get("name") == "humble":
            image = distro.get("image")
            if not isinstance(image, Mapping) or image.get("digest") != IMAGE_DIGEST:
                raise DiscoveryError("PROFILE_IMAGE_MISMATCH", "Humble digest")
            return {
                "path": str(profile_path),
                "sha256": sha256_bytes(profile_path.read_bytes()),
                "distro": "humble",
                "image_digest": IMAGE_DIGEST,
            }
    raise DiscoveryError("PROFILE_HUMBLE_MISSING", str(profile_path))


def _image_observation(root: Path, result: Mapping[str, Any]) -> dict[str, Any]:
    if result["returncode"] != 0 or result["timed_out"] or result["oversized"]:
        raise DiscoveryError("IMAGE_INSPECT_FAILED", "image inspect")
    try:
        values = json.loads(_read_regular(
            root / result["stdout"]["path"], MAX_COMMAND_OUTPUT,
            "image inspect output"
        ))
    except (json.JSONDecodeError, OSError) as exc:
        raise DiscoveryError("IMAGE_INSPECT_OUTPUT_INVALID", "image inspect") from exc
    if not isinstance(values, Mapping):
        raise DiscoveryError("IMAGE_INSPECT_OUTPUT_INVALID", "image object")
    if (
        values.get("Id") != IMAGE_DIGEST
        or values.get("Architecture") != "amd64"
        or values.get("Os") != "linux"
        or not any(
            isinstance(item, str) and item.endswith("@" + IMAGE_DIGEST)
            for item in values.get("RepoDigests", [])
        )
    ):
        raise DiscoveryError("IMAGE_BINDING_INVALID", "local Humble image")
    return {
        "id": values["Id"],
        "repo_digests": list(values.get("RepoDigests", [])),
        "architecture": values["Architecture"],
        "os": values["Os"],
        "size": values.get("Size"),
    }


def _container_name() -> str:
    return "registration-plugin-rosdep-discovery-{}-{}".format(
        time.strftime("%Y%m%d%H%M%S", time.gmtime()), uuid.uuid4().hex[:12]
    )


def _container_absent(command: CommandCapture, name: str) -> None:
    record = command.run(
        ["docker", "ps", "-a", "--filter", "name=^{}$".format(name), "--format", "{{.Names}}"],
        "container_collision_check",
        timeout=30,
    )
    if record["returncode"] != 0 or record["timed_out"] or record["oversized"]:
        raise DiscoveryError("CONTAINER_PROBE_FAILED", name)
    data = _read_regular(
        command.root / record["stdout"]["path"], MAX_COMMAND_OUTPUT, "container probe"
    ).decode("utf-8", "strict")
    if data.strip():
        raise DiscoveryError("CONTAINER_COLLISION", name)


def _cleanup_container(command: CommandCapture, name: str) -> dict[str, Any]:
    """Remove one owned container and prove the exact name is absent."""
    stop = command.run(
        ["docker", "rm", "-f", name], "container_remove", timeout=120
    )
    try:
        stop_stderr = _read_record_stream(command.root, stop, "stderr")
    except DiscoveryError:
        stop_stderr = b"invalid"
    removed = (
        stop["returncode"] == 0
        and not stop["timed_out"]
        and not stop["oversized"]
        and stop_stderr == b""
    )
    probe = command.run(
        [
            "docker", "ps", "-a", "--filter", "name=^{}$".format(name),
            "--format", "{{.Names}}",
        ],
        "container_post_remove_check",
        timeout=30,
    )
    try:
        probe_stdout = _read_record_stream(command.root, probe, "stdout")
        probe_stderr = _read_record_stream(command.root, probe, "stderr")
    except DiscoveryError:
        probe_stdout = b"invalid"
        probe_stderr = b"invalid"
    post_absent = (
        probe["returncode"] == 0
        and not probe["timed_out"]
        and not probe["oversized"]
        and probe_stdout == b""
        and probe_stderr == b""
    )
    return {
        "remove_requested": True,
        "remove_returncode": stop["returncode"],
        "remove_record": dict(stop["record"]),
        "post_remove_absent": post_absent,
        "post_remove_record": dict(probe["record"]),
        "status": "PASS" if removed and post_absent else "FAIL_CLOSED",
    }


def _require_command_ok(record: Mapping[str, Any], label: str) -> None:
    if record["returncode"] != 0 or record["timed_out"] or record["oversized"]:
        raise DiscoveryError("COMMAND_FAILED", label)


def _read_record_stream(root: Path, record: Mapping[str, Any], stream: str) -> bytes:
    descriptor = record[stream]
    return _read_regular(root / descriptor["path"], MAX_COMMAND_OUTPUT, stream)


def _command_output_text(root: Path, record: Mapping[str, Any], stream: str = "stdout") -> str:
    try:
        return _read_record_stream(root, record, stream).decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise DiscoveryError("COMMAND_OUTPUT_NOT_UTF8", record["label"]) from exc


def _package_records(
    root: Path,
    print_record: Mapping[str, Any],
    show_record: Mapping[str, Any],
    dpkg_record: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Build exact target package records before any deb bytes are accepted."""
    uris = _parse_print_uris(_read_record_stream(root, print_record, "stdout"))
    rows = _parse_package_output(_read_record_stream(root, dpkg_record, "stdout"))
    show_data = _read_record_stream(root, show_record, "stdout")
    records = parse_apt_package_records(show_data, "http://packages.ros.org/ros2/ubuntu")
    by_name_version: dict[tuple[str, str], dict[str, Any]] = {}
    for item in records:
        key = (item["name"], item["version"])
        if key in by_name_version and by_name_version[key] != item:
            raise DiscoveryError("APT_PACKAGE_DUPLICATE", item["name"])
        by_name_version[key] = item
    expected = []
    for row in rows:
        item = by_name_version.get((row["name"], row["version"]))
        if item is None:
            raise DiscoveryError("APT_SELECTED_VERSION_NOT_IN_INDEX", row["name"])
        expected.append(item)
    uris = _validate_uri_set(expected, uris)
    records = []
    for item in expected:
        filename = PurePosixPath(item["filename"]).name
        uri = uris[filename]
        if uri != item["uri"]:
            raise DiscoveryError("APT_DEB_URI_BINDING_INVALID", filename)
        records.append({
            "name": item["name"],
            "version": item["version"],
            "architecture": item["architecture"],
            "filename": item["filename"],
            "uri": uri,
            "size_bytes": item["size_bytes"],
            "sha256": item["sha256"],
            "source": "apt-cache-show-signed-Packages-field",
        })
    return records


def _package_evidence(
    root: Path, command: CommandCapture, print_record: Mapping[str, Any],
    show_record: Mapping[str, Any], dpkg_record: Mapping[str, Any],
    deb_artifacts: Mapping[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """Return package records only after exact sealed debs are reopened."""
    del command
    records = _package_records(root, print_record, show_record, dpkg_record)
    if deb_artifacts is None:
        raise DiscoveryError("APT_DEB_ARTIFACTS_MISSING", "signed Package set")
    validated = _validate_deb_descriptor_map(root, records, deb_artifacts)
    evidence = []
    for item in records:
        result = dict(item)
        filename = PurePosixPath(item["filename"]).name
        result["deb_artifact"] = _relative_descriptor(root, validated[filename])
        evidence.append(result)
    return evidence


def _download_package_artifacts(
    root: Path,
    command: CommandCapture,
    container_name: str,
    expected: list[dict[str, Any]],
    cache_path: str,
    partial_state: dict[str, Any] | None = None,
    output_layout: Mapping[str, Any] | None = None,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    """Copy an exact fresh apt cache out and seal the signed deb bytes."""
    if (
        not isinstance(cache_path, str)
        or not re.fullmatch(r"/tmp/registration-plugin-apt-[a-f0-9]{32}", cache_path)
    ):
        raise DiscoveryError("APT_CACHE_PATH_INVALID", cache_path)
    inventory_record = command.run(
        [
            "docker", "exec", container_name, "find", cache_path,
            "-mindepth", "1", "-maxdepth", "1", "-exec", "stat", "-c",
            "%n\t%F\t%a\t%u\t%g\t%h\t%s\t%d\t%i", "{}", "+",
        ],
        "apt_deb_cache_inventory",
        timeout=30,
    )
    _require_command_ok(inventory_record, "apt_deb_cache_inventory")
    inventory = _parse_cache_inventory(
        _read_record_stream(root, inventory_record, "stdout")
    )
    classified = _classify_cache_entries(inventory, expected)
    cache_identity_record = command.run(
        [
            "docker", "exec", container_name, "stat", "-c",
            "%n\t%F\t%a\t%u\t%g\t%h\t%s\t%d\t%i", cache_path,
        ],
        "apt_deb_cache_identity_before",
        timeout=30,
    )
    _require_command_ok(cache_identity_record, "apt_deb_cache_identity_before")
    cache_identity = _parse_cache_identity(
        _read_record_stream(root, cache_identity_record, "stdout"), cache_path
    )
    lock_hash_record = command.run(
        ["docker", "exec", container_name, "sha256sum", cache_path + "/lock"],
        "apt_deb_cache_lock_hash",
        timeout=30,
    )
    _require_command_ok(lock_hash_record, "apt_deb_cache_lock_hash")
    lock_sha256 = _parse_sha256sum(
        _read_record_stream(root, lock_hash_record, "stdout"), cache_path + "/lock"
    )
    metadata = _validate_cache_metadata(
        inventory, expected, lock_sha256, classified
    )
    _validate_cache_package_set(inventory, expected, classified)
    if partial_state is not None:
        partial_state["cache_details"] = {
            "path": cache_path,
            "identity_before": dict(cache_identity),
            "entries_before": {
                name: dict(entry) for name, entry in metadata.items()
            },
            "lock_sha256": lock_sha256,
            "source_removals": [],
        }
    staging = ensure_directory(root, "staging/apt-debs")
    staged: dict[str, Path] = {}
    identities: dict[str, tuple[int, int]] = {}
    try:
        for index, item in enumerate(expected):
            filename = PurePosixPath(item["filename"]).name
            destination = staging / filename
            if os.path.lexists(str(destination)):
                raise DiscoveryError("APT_DEB_OUTPUT_COLLISION", filename)
            source = cache_path + "/" + filename
            copied = command.run(
                [
                    "docker", "cp", container_name + ":" + source,
                    str(destination),
                ],
                "apt_deb_copy_{:03d}".format(index),
                timeout=120,
            )
            _require_command_ok(copied, "apt_deb_copy_{}".format(filename))
            info = os.lstat(str(destination))
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise DiscoveryError("APT_DEB_OUTPUT_INVALID", filename)
            staged[filename] = destination
            identities[filename] = (info.st_dev, info.st_ino)
        deb_artifacts = _bind_downloaded_debs(
            root, expected, staged, output_layout
        )
        if partial_state is not None:
            partial_state["deb_artifacts"] = {
                name: dict(descriptor)
                for name, descriptor in deb_artifacts.items()
            }
        _validate_deb_descriptor_map(root, expected, deb_artifacts)
        source_removals = []
        for index, item in enumerate(expected):
            filename = PurePosixPath(item["filename"]).name
            source_removals.append(
                _remove_verified_cache_deb(
                    root,
                    command,
                    container_name,
                    cache_path,
                    item,
                    inventory[filename],
                    index=index,
                    partial_state=partial_state,
                )
            )
    finally:
        for filename, path in staged.items():
            try:
                info = os.lstat(str(path))
                if identities.get(filename) == (info.st_dev, info.st_ino):
                    os.unlink(str(path))
            except OSError:
                pass
    remove_lock = command.run(
        ["docker", "exec", container_name, "rm", "--", cache_path + "/lock"],
        "apt_deb_cache_lock_remove",
        timeout=30,
    )
    _require_command_ok(remove_lock, "apt_deb_cache_lock_remove")
    remove_partial = command.run(
        ["docker", "exec", container_name, "rmdir", "--", cache_path + "/partial"],
        "apt_deb_cache_partial_remove",
        timeout=30,
    )
    _require_command_ok(remove_partial, "apt_deb_cache_partial_remove")
    sync_record = command.run(
        ["docker", "exec", container_name, "sync"],
        "apt_deb_cache_sync",
        timeout=30,
    )
    _require_command_ok(sync_record, "apt_deb_cache_sync")
    post_identity_record = command.run(
        [
            "docker", "exec", container_name, "stat", "-c",
            "%n\t%F\t%a\t%u\t%g\t%h\t%s\t%d\t%i", cache_path,
        ],
        "apt_deb_cache_identity_after",
        timeout=30,
    )
    _require_command_ok(post_identity_record, "apt_deb_cache_identity_after")
    post_identity = _parse_cache_identity(
        _read_record_stream(root, post_identity_record, "stdout"), cache_path
    )
    post_inventory_record = command.run(
        [
            "docker", "exec", container_name, "find", cache_path,
            "-mindepth", "1", "-maxdepth", "1", "-exec", "stat", "-c",
            "%n\t%F\t%a\t%u\t%g\t%h\t%s\t%d\t%i", "{}", "+",
        ],
        "apt_deb_cache_inventory_after",
        timeout=30,
    )
    _require_command_ok(post_inventory_record, "apt_deb_cache_inventory_after")
    post_inventory = _parse_cache_inventory(
        _read_record_stream(root, post_inventory_record, "stdout")
    )
    if partial_state is not None:
        partial_state["cache_details"] = {
            **dict(partial_state.get("cache_details", {})),
            "identity_after": dict(post_identity),
            "inventory_after": {
                name: dict(entry) for name, entry in post_inventory.items()
            },
        }
    _validate_cache_lifecycle(
        cache_identity, post_identity, classified[1], post_inventory
    )
    details = {
        "path": cache_path,
        "identity_before": cache_identity,
        "entries_before": metadata,
        "lock_sha256": lock_sha256,
            "lock_hash_record": dict(lock_hash_record["record"]),
            "source_removals": source_removals,
            "identity_after": post_identity,
        "inventory_after": {},
        "cleanup": {
            "remove_lock": dict(remove_lock["record"]),
            "remove_partial": dict(remove_partial["record"]),
            "sync": dict(sync_record["record"]),
            "identity_after": dict(post_identity_record["record"]),
            "inventory_after": dict(post_inventory_record["record"]),
        },
    }
    if partial_state is not None:
        partial_state["cache_details"] = dict(details)
    return deb_artifacts, details


def _attach_partial_deb_evidence(
    receipt: dict[str, Any],
    root: Path,
    deb_artifacts: Mapping[str, Mapping[str, Any]],
    cache_details: Mapping[str, Any],
) -> None:
    """Expose verified debs when a later cache lifecycle check fails."""
    if not deb_artifacts:
        return
    receipt["apt"].update({
        "status": "PARTIAL_REVIEW_REQUIRED",
        "partial_cache": dict(cache_details),
        "partial_deb_artifacts": [
            _relative_descriptor(root, descriptor)
            for descriptor in deb_artifacts.values()
        ],
    })


def _base_receipt(
    root_identity: Mapping[str, Any],
    profile: Mapping[str, Any],
    name: str,
    discovery_id: str,
) -> dict[str, Any]:
    expected_discovery_id = _discovery_id_for_root(Path(root_identity["path"]))
    if discovery_id != expected_discovery_id:
        raise DiscoveryError("DISCOVERY_ID_INVALID", discovery_id)
    return {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": "REVIEW_REQUIRED",
        "outcome": "PARTIAL_FAILURE",
        "benchmark_eligible": False,
        "promotion": "FORBIDDEN_UNTIL_SIGNED_REVIEW",
        "rosdep_update_executed": False,
        "discovery_id": discovery_id,
        "profile": profile,
        "image": {
            "reference": IMAGE_REFERENCE,
            "digest": IMAGE_DIGEST,
            "platform": "linux/amd64",
        },
        "container": {
            "name": name,
            "network": "bridge-during-provisioning-only",
            "pull": False,
            "build": False,
            "import": False,
            "tag": False,
        },
        "root": dict(root_identity),
        "network_policy": {
            "apt": "profile-allowlisted-only",
            "git": GIT_ENDPOINT,
            "archive_hosts": sorted(ARCHIVE_HOSTS),
            "redirects": "allowlisted-codeload-hosts-only",
            "timeout_seconds": HTTP_TIMEOUT,
            "max_retries": MAX_RETRIES,
        },
        "output_layout": None,
        "apt": {"status": "NOT_CAPTURED", "packages": []},
        "rosdep": {"status": "NOT_CAPTURED", "update_executed": False},
        "rosdistro": {"status": "NOT_CAPTURED"},
        "commands": [],
        "artifacts": [],
        "cleanup": {
            "stop_requested": False,
            "remove_requested": False,
            "post_remove_absent": False,
        },
        "failure": None,
        "started_at_unix": int(time.time()),
        "ended_at_unix": None,
    }


def _validate_output_layout_receipt(
    root: Path,
    layout: Mapping[str, Any],
    *,
    expected_owner: tuple[int, int] | None = None,
) -> None:
    """Validate the sealed pre/post descriptors for known output parents."""
    if expected_owner is None:
        # Backwards-compatible direct helper use remains safe: derive the
        # expectation from the no-follow root object, never from the current
        # process (which may be container root).
        try:
            root_info = os.lstat(str(root))
        except OSError as exc:
            raise DiscoveryError("ROOT_IDENTITY_INVALID", str(root)) from exc
        if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
            raise DiscoveryError("ROOT_IDENTITY_INVALID", str(root))
        expected_owner = (int(root_info.st_uid), int(root_info.st_gid))
    owner = _validated_owner(expected_owner, "output layout owner")
    if not isinstance(layout, Mapping):
        raise DiscoveryError("OUTPUT_LAYOUT_INVALID", "receipt layout")
    status = layout.get("status")
    allowed = layout.get("allowed_directories")
    directories = layout.get("directories")
    if (
        status not in {"PRECREATED", "VERIFIED", "FAILED"}
        or allowed != list(ROSDISTRO_OUTPUT_DIRECTORIES)
        or not isinstance(directories, Mapping)
        or set(directories) != set(ROSDISTRO_OUTPUT_DIRECTORIES)
    ):
        raise DiscoveryError("OUTPUT_LAYOUT_INVALID", "receipt layout")
    expected_descriptor_fields = {
        "path", "type", "mode", "uid", "gid", "nlink", "device", "inode"
    }
    for relative in ROSDISTRO_OUTPUT_DIRECTORIES:
        entry = directories.get(relative)
        if not isinstance(entry, Mapping):
            raise DiscoveryError("OUTPUT_LAYOUT_INVALID", relative)
        expected_entry_fields = {
            "before", "after"
        } if status == "VERIFIED" else {"before"}
        if set(entry) != expected_entry_fields:
            raise DiscoveryError("OUTPUT_LAYOUT_INVALID", relative)
        before = entry.get("before")
        after = entry.get("after")
        if not isinstance(before, Mapping):
            raise DiscoveryError("OUTPUT_LAYOUT_INVALID", relative)
        for descriptor in (before, after):
            if descriptor is None and status in {"PRECREATED", "FAILED"}:
                continue
            if (
                not isinstance(descriptor, Mapping)
                or set(descriptor) != expected_descriptor_fields
                or descriptor.get("path") != relative
                or descriptor.get("type") != "directory"
                or descriptor.get("mode") != 0o700
                or descriptor.get("uid") != owner[0]
                or descriptor.get("gid") != owner[1]
                or type(descriptor.get("nlink")) is not int
                or descriptor.get("nlink") < 2
                or type(descriptor.get("device")) is not int
                or descriptor.get("device") < 0
                or type(descriptor.get("inode")) is not int
                or descriptor.get("inode") < 0
            ):
                raise DiscoveryError("OUTPUT_LAYOUT_INVALID", relative)
        if status == "VERIFIED":
            if not isinstance(after, Mapping) or dict(before) != dict(after):
                raise DiscoveryError("OUTPUT_DIRECTORY_CHANGED", relative)
            current = _layout_directory_identity(
                root, relative, expected_owner=owner
            )
            if current != dict(after):
                raise DiscoveryError("OUTPUT_DIRECTORY_CHANGED", relative)
    if status == "FAILED":
        failure = layout.get("failure")
        if (
            set(layout) != {"status", "allowed_directories", "directories", "failure"}
            or not isinstance(failure, Mapping)
            or set(failure) != {"kind", "message"}
            or not isinstance(failure.get("kind"), str)
            or not isinstance(failure.get("message"), str)
        ):
            raise DiscoveryError("OUTPUT_LAYOUT_INVALID", "failed receipt layout")
    elif set(layout) != {"status", "allowed_directories", "directories"}:
        raise DiscoveryError("OUTPUT_LAYOUT_INVALID", "receipt layout fields")


def validate_receipt(root: Path, *, logical_root: Path | None = None) -> dict[str, Any]:
    """Reopen a receipt through a host or fixed transport root.

    ``root`` is the namespace in which the sealed bytes are currently
    readable.  ``logical_root`` is the producer's host path recorded in the
    receipt.  They are intentionally separate for the prepare container's
    read-only discovery bind; all inode/content checks still use ``root`` and
    only the logical path/id checks use ``logical_root``.
    """
    root = _absolute(root, "receipt root")
    if logical_root is None:
        logical_root = root
    else:
        logical_root = _absolute(logical_root, "logical receipt root")
    receipt_path = root / RECEIPT_NAME
    data = _read_regular(receipt_path, 8 * 1024 * 1024, "receipt")
    try:
        value = json.loads(data.decode("utf-8", "strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DiscoveryError("RECEIPT_INVALID", "receipt JSON") from exc
    if not isinstance(value, dict):
        raise DiscoveryError("RECEIPT_INVALID", "receipt object")
    expected_discovery_id = _discovery_id_for_root(logical_root)
    if (
        value.get("schema") != SCHEMA
        or value.get("schema_version") != SCHEMA_VERSION
        or value.get("status") != "REVIEW_REQUIRED"
        or value.get("benchmark_eligible") is not False
        or value.get("promotion") != "FORBIDDEN_UNTIL_SIGNED_REVIEW"
        or value.get("rosdep_update_executed") is not False
        or value.get("discovery_id") != expected_discovery_id
        or not isinstance(value.get("root"), Mapping)
        or value.get("canonical_sha256") != canonical_hash(value)
    ):
        raise DiscoveryError("RECEIPT_BINDING_INVALID", "receipt contract")
    root_value = value["root"]
    if (
        set(root_value) != {
            "path", "device", "inode", "uid", "gid", "mode", "nlink"
        }
        or root_value.get("path") != str(logical_root)
        or type(root_value.get("device")) is not int
        or root_value.get("device") < 0
        or type(root_value.get("inode")) is not int
        or root_value.get("inode") < 1
        or type(root_value.get("uid")) is not int
        or root_value.get("uid") < 0
        or type(root_value.get("gid")) is not int
        or root_value.get("gid") < 0
        or root_value.get("mode") != 0o700
        or type(root_value.get("nlink")) is not int
        or root_value.get("nlink") < 2
    ):
        raise DiscoveryError("ROOT_IDENTITY_INVALID", "receipt root descriptor")
    _check_parent(root, "receipt root parent")
    try:
        root_info = os.lstat(str(root))
    except OSError as exc:
        raise DiscoveryError("ROOT_IDENTITY_INVALID", str(root)) from exc
    if (
        stat.S_ISLNK(root_info.st_mode)
        or not stat.S_ISDIR(root_info.st_mode)
        or stat.S_IMODE(root_info.st_mode) != root_value["mode"]
        or root_info.st_uid != root_value["uid"]
        or root_info.st_gid != root_value["gid"]
        or root_info.st_dev != root_value["device"]
        or root_info.st_ino != root_value["inode"]
        or root_info.st_nlink < root_value["nlink"]
    ):
        raise DiscoveryError("ROOT_IDENTITY_INVALID", "receipt root changed")
    expected_owner = (root_value["uid"], root_value["gid"])
    output_layout = value.get("output_layout")
    if output_layout is not None:
        _validate_output_layout_receipt(
            root, output_layout, expected_owner=expected_owner
        )
    for descriptor in value.get("artifacts", []):
        if not isinstance(descriptor, Mapping):
            raise DiscoveryError("RECEIPT_ARTIFACT_INVALID", "artifact list")
        path = root / _relative(descriptor.get("path"), "receipt artifact")
        sidecar = root / _relative(descriptor.get("sidecar"), "receipt sidecar")
        expected_descriptor = dict(descriptor)
        expected_descriptor["path"] = str(path)
        expected_descriptor["sidecar"] = str(sidecar)
        reopen_artifact(path, expected_descriptor)
    return value


def discover(
    output_root: Path,
    *,
    profile_path: Path = PROFILE_PATH,
) -> dict[str, Any]:
    """Run one discovery attempt and seal either complete or partial evidence."""
    root = _absolute(output_root, "discovery root")
    discovery_id = _discovery_id_for_root(root)
    root_identity = create_fresh_root(root)
    profile = _profile_binding(profile_path)
    container_name = _container_name()
    receipt = _base_receipt(
        root_identity, profile, container_name, discovery_id
    )
    command = CommandCapture(root)
    container_started = False
    deb_artifacts: dict[str, dict[str, Any]] = {}
    cache_details: dict[str, Any] = {}
    partial_state: dict[str, Any] = {}
    output_layout: dict[str, Any] | None = None
    try:
        output_layout = _create_rosdistro_output_layout(root)
        receipt["output_layout"] = output_layout
        inspect = command.run(
            ["docker", "image", "inspect", IMAGE_DIGEST, "--format", "{{json .}}"],
            "image_inspect",
            timeout=30,
        )
        _require_command_ok(inspect, "image_inspect")
        receipt["image"]["observed"] = _image_observation(root, inspect)
        _container_absent(command, container_name)
        run_record = command.run(
            [
                "docker", "run", "-d", "--name", container_name,
                "--platform", "linux/amd64",
                "--pull=never", "--network", "bridge", IMAGE_REFERENCE,
                "/bin/sleep", "3600",
            ],
            "container_start",
            timeout=60,
        )
        _require_command_ok(run_record, "container_start")
        container_started = True
        apt_update = command.run(
            [
                "docker", "exec", container_name, "apt-get", "update",
                "-o", "APT::Update::Error-Mode=any",
            ],
            "apt_update",
        )
        _require_command_ok(apt_update, "apt_update")
        packages = list(ALLOWED_PACKAGE_NAMES)
        show = command.run(
            ["docker", "exec", container_name, "apt-cache", "show", *packages],
            "apt_cache_show",
        )
        _require_command_ok(show, "apt_cache_show")
        policy = command.run(
            [
                "docker", "exec", container_name, "apt-cache", "policy",
                *packages,
            ],
            "apt_cache_policy",
        )
        _require_command_ok(policy, "apt_cache_policy")
        install = command.run(
            [
                "docker", "exec", container_name,
                "env", "DEBIAN_FRONTEND=noninteractive",
                "apt-get", "install", "--yes", "--no-install-recommends", *packages,
            ],
            "apt_install_rosdep_packages",
        )
        _require_command_ok(install, "apt_install_rosdep_packages")
        dpkg = command.run(
            [
                "docker", "exec", container_name, "dpkg-query", "-W", "-f",
                "${Package}\t${Version}\t${Architecture}\t${Status}\n", *packages,
            ],
            "dpkg_selected_versions",
        )
        _require_command_ok(dpkg, "dpkg_selected_versions")
        dpkg_rows = _parse_package_output(
            _read_record_stream(root, dpkg, "stdout")
        )
        exact_specs = _exact_package_specs(dpkg_rows)
        cache_path = "/tmp/registration-plugin-apt-" + uuid.uuid4().hex
        cache_create = command.run(
            [
                "docker", "exec", container_name, "mkdir", "--mode", "0700",
                "--", cache_path,
            ],
            "apt_deb_cache_create",
            timeout=30,
        )
        _require_command_ok(cache_create, "apt_deb_cache_create")
        partial_create = command.run(
            [
                "docker", "exec", container_name, "mkdir", "--mode", "0700", "--",
                cache_path + "/partial",
            ],
            "apt_deb_cache_partial_create",
            timeout=30,
        )
        _require_command_ok(partial_create, "apt_deb_cache_partial_create")
        print_uris = command.run(
            [
                "docker", "exec", container_name, "apt-get", "--print-uris", "--yes",
                "--reinstall", "--download-only", "--no-install-recommends",
                "-o", "Dir::Cache::archives=" + cache_path,
                "install", *exact_specs,
            ],
            "apt_print_uris",
        )
        _require_command_ok(print_uris, "apt_print_uris")
        expected_packages = _package_records(
            root, print_uris, show, dpkg
        )
        download = command.run(
            [
                "docker", "exec", container_name, "apt-get", "--reinstall",
                "--download-only", "--yes", "--no-install-recommends",
                "-o", "Dir::Cache::archives=" + cache_path,
                "install", *exact_specs,
            ],
            "apt_download_packages",
        )
        _require_command_ok(download, "apt_download_packages")
        deb_artifacts, cache_details = _download_package_artifacts(
            root, command, container_name, expected_packages, cache_path,
            partial_state, output_layout,
        )
        apt_sources = []
        for index, path in enumerate(("/etc/apt/sources.list", "/etc/apt/sources.list.d/ros2.sources")):
            record = command.run(
                ["docker", "exec", container_name, "cat", path],
                "apt_source_{}".format(index),
            )
            _require_command_ok(record, "apt_source_{}".format(index))
            source_data = _read_record_stream(root, record, "stdout")
            apt_sources.append({"path": path, "sha256": sha256_bytes(source_data), "bytes": len(source_data)})
        apt_packages = _package_evidence(
            root, command, print_uris, show, dpkg, deb_artifacts
        )
        receipt["apt"] = {
            "status": "CAPTURED_REVIEW_REQUIRED",
            "update": {"command_record": dict(apt_update["record"])},
            "cache_show": dict(show["record"]),
            "cache_policy": dict(policy["record"]),
            "print_uris": dict(print_uris["record"]),
            "download": dict(download["record"]),
            "cache_create": dict(cache_create["record"]),
            "cache_partial_create": dict(partial_create["record"]),
            "cache_metadata": cache_details,
            "cache_inventory": dict(
                next(
                    item["record"]
                    for item in command.records
                    if item["label"] == "apt_deb_cache_inventory"
                )
            ),
            "deb_copies": [
                dict(item["record"])
                for item in command.records
                if item["label"].startswith("apt_deb_copy_")
            ],
            "deb_artifacts": [item["deb_artifact"] for item in apt_packages],
            "install": dict(install["record"]),
            "sources": apt_sources,
            "packages": apt_packages,
            "signed_metadata_claim": "apt-update-verification-observed-not-independent-gpgv-review",
        }
        rosdep_commands = []
        for label, argv in (
            ("rosdep_version", ["rosdep", "--version"]),
            ("rosdep_help", ["rosdep", "--help"]),
            ("rosdep_update_help", ["rosdep", "update", "--help"]),
            ("rosdep_install_help", ["rosdep", "install", "--help"]),
            ("rosdep_resolve_help", ["rosdep", "resolve", "--help"]),
        ):
            record = command.run(
                ["docker", "exec", container_name, *argv], label, timeout=120
            )
            _require_command_ok(record, label)
            rosdep_commands.append(dict(record["record"]))
        receipt["rosdep"] = {
            "status": "INSTALLED_CLI_OBSERVED",
            "version": _command_output_text(root, command.records[-5], "stdout").strip(),
            "help_records": rosdep_commands,
            "update_executed": False,
        }
        git_env = {
            "HOME": "/nonexistent",
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_CONFIG_GLOBAL": "/dev/null",
            "GIT_TERMINAL_PROMPT": "0",
        }
        git_record = command.run(
            ["git", "-c", "credential.helper=", "-c", "protocol.version=2", "ls-remote", "--heads", "--refs", GIT_ENDPOINT, "master"],
            "rosdistro_ls_remote",
            timeout=GIT_TIMEOUT,
            env=git_env,
        )
        _require_command_ok(git_record, "rosdistro_ls_remote")
        git_text = _command_output_text(root, git_record)
        refs = []
        for line in git_text.splitlines():
            fields = line.split("\t")
            if len(fields) == 2 and fields[1] == "refs/heads/master" and COMMIT_RE.fullmatch(fields[0]):
                refs.append(fields[0])
        if len(refs) != 1:
            raise DiscoveryError("ROSDISTRO_COMMIT_INVALID", "ls-remote master")
        commit = refs[0]
        archive_url = "https://codeload.github.com/ros/rosdistro/tar.gz/{}".format(commit)
        fetch_record, archive_path = fetch_archive(
            archive_url, commit, root, command, output_layout
        )
        archive_details = inspect_rosdistro_archive(
            archive_path, root, commit, command, output_layout
        )
        receipt["rosdistro"] = {
            "status": "COMMIT_ARCHIVE_CAPTURED_REVIEW_REQUIRED",
            "repository": GIT_ENDPOINT,
            "ref": "refs/heads/master",
            "commit": commit,
            "ls_remote": dict(git_record["record"]),
            "archive": fetch_record,
            "archive_details": archive_details,
            "source_graph_policy": "commit-addressed-files-only; external-review-required",
        }
        receipt["commands"] = [dict(item["record"]) for item in command.records]
        receipt["artifacts"] = []
        for item in command.records:
            receipt["artifacts"].extend([item["stdout"], item["stderr"], item["record"]])
        receipt["artifacts"].append(fetch_record["archive"])
        receipt["artifacts"].append(fetch_record["record"])
        receipt["artifacts"].extend(item["artifact"] for item in archive_details["selected_files"])
        receipt["cleanup"]["status"] = "PENDING"
    except Exception as exc:
        receipt["failure"] = {
            "kind": getattr(exc, "kind", "DISCOVERY_FAILED"),
            "message": str(exc),
        }
    finally:
        if output_layout is not None:
            try:
                receipt["output_layout"] = _finalize_rosdistro_output_layout(
                    root, output_layout
                )
            except Exception as exc:
                if receipt["failure"] is None:
                    receipt["failure"] = {
                        "kind": getattr(exc, "kind", "OUTPUT_LAYOUT_INVALID"),
                        "message": str(exc),
                    }
                failed_layout = dict(output_layout)
                failed_layout["status"] = "FAILED"
                failed_layout["failure"] = {
                    "kind": getattr(exc, "kind", "OUTPUT_LAYOUT_INVALID"),
                    "message": str(exc),
                }
                receipt["output_layout"] = failed_layout
        for artifact_name, descriptor in partial_state.get("deb_artifacts", {}).items():
            if isinstance(descriptor, Mapping):
                deb_artifacts[artifact_name] = dict(descriptor)
        if partial_state.get("cache_details"):
            cache_details = dict(partial_state["cache_details"])
        if receipt["failure"] is not None and deb_artifacts:
            _attach_partial_deb_evidence(
                receipt, root, deb_artifacts, cache_details
            )
        if container_started:
            receipt["cleanup"].update(
                _cleanup_container(command, container_name)
            )
        else:
            receipt["cleanup"].update({
                "status": "NOT_STARTED",
                "post_remove_absent": True,
            })
        receipt["commands"] = [dict(item["record"]) for item in command.records]
        receipt["artifacts"] = []
        for item in command.records:
            receipt["artifacts"].extend([item["stdout"], item["stderr"], item["record"]])
        receipt["artifacts"].extend(
            item["deb_artifact"]
            for item in receipt.get("apt", {}).get("packages", [])
            if isinstance(item, Mapping) and "deb_artifact" in item
        )
        known_artifacts = {
            item.get("path")
            for item in receipt["artifacts"]
            if isinstance(item, Mapping)
        }
        for item in deb_artifacts.values():
            if not isinstance(item, Mapping):
                continue
            relative = _relative_descriptor(root, item)
            if relative["path"] not in known_artifacts:
                receipt["artifacts"].append(relative)
                known_artifacts.add(relative["path"])
        if receipt.get("rosdistro", {}).get("archive"):
            receipt["artifacts"].append(receipt["rosdistro"]["archive"]["archive"])
            receipt["artifacts"].append(receipt["rosdistro"]["archive"]["record"])
            receipt["artifacts"].extend(
                item["artifact"] for item in receipt["rosdistro"].get("archive_details", {}).get("selected_files", [])
            )
        receipt["ended_at_unix"] = int(time.time())
        if receipt["failure"] is None:
            receipt["outcome"] = "PASS_REVIEW_REQUIRED"
        receipt["canonical_sha256"] = canonical_hash(receipt)
        receipt_descriptor = seal_bytes(root / RECEIPT_NAME, canonical_bytes(receipt) + b"\n")
        receipt["receipt_sha256"] = receipt_descriptor["sha256"]
        # receipt_sha256 is informational and would make the canonical hash
        # circular; it is deliberately not included in the self-hash projection.
        # Rewrite is forbidden, so retain the canonical value as the authority.
    # The receipt descriptor is already sealed; validate the on-disk value.
    return validate_receipt(root)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        value = discover(args.output_root, profile_path=args.profile)
        print(json.dumps({
            "status": value["status"],
            "outcome": value["outcome"],
            "root": str(args.output_root),
            "canonical_sha256": value["canonical_sha256"],
        }, sort_keys=True))
        return 0
    except Exception as exc:
        print(json.dumps({
            "status": "FAIL_CLOSED",
            "kind": getattr(exc, "kind", "DISCOVERY_FAILED"),
            "message": str(exc),
        }, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
