#!/usr/bin/env python3
"""Consume the sealed rosdep discovery packet in an offline prepare phase.

The discovery packet is deliberately a candidate input, not a promotion
receipt.  This module reopens the packet and its four debs/eight selected
rosdistro files before constructing a local-only rosdep source list.  The
phase executor is injected by callers (and by tests); there is no default
Docker, apt, or network runner.  Consequently importing or invoking this
module cannot silently provision anything.

The fixed discovery-11 binding below is intentionally exact.  A future
discovery must be reviewed and pinned as a new contract instead of being
accepted through a mutable path or a ``master`` URL.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tarfile
import time
from typing import Any, Callable, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[0]
REPOSITORY_PROFILE_PATH = ROOT / (
    "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
)
BUNDLE_PROFILE_PATH = SCRIPT_DIR / "consumer-profile.json"
PROFILE_PATH = BUNDLE_PROFILE_PATH if BUNDLE_PROFILE_PATH.is_file() else REPOSITORY_PROFILE_PATH
DISCOVERY_ROOT = Path(
    "/media/sasaki/aiueo1/benchmarks/"
    "registration-plugin-rosdep-discovery-20260827-11"
)
# The discovery packet is a host-owned input.  A production container never
# receives that host path as an argv operand: it is mounted at this fixed,
# read-only transport location instead.  The logical path remains in the
# receipt so the producer identity and the container transport identity can be
# checked independently.
DISCOVERY_TRANSPORT_ROOT = Path("/opt/registration-plugin/discovery")
# This value is immutable even when tests redirect the validator's input root
# to a copied fixture.  The active profile must continue to allowlist the
# sealed production discovery root, not an arbitrary caller-selected path.
PINNED_DISCOVERY_ROOT = DISCOVERY_ROOT
DISCOVERY_RECEIPT_NAME = "registration_plugin_rosdep_discovery.receipt.json"
DISCOVERY_SCHEMA = "registration-plugin-rosdep-discovery-v1"
PREPARE_SCHEMA = "registration-plugin-rosdep-prepare-v1"
SCHEMA_VERSION = 1
DISCOVERY_ID = "registration-plugin-rosdep-discovery-20260827-11"
EXPECTED_RECEIPT_SHA256 = (
    "c9bfbd31a61a95a9263e8b46f7ffdb99f4353daa672ed8bd85d647065b7ac780"
)
EXPECTED_DISCOVERY_CANONICAL_SHA256 = (
    "80724060f8b94b724416c84f0cd25bdc8dbad95b2c79a0faa5005dea26a700a7"
)
DISCOVERY_PROFILE_SHA256 = (
    "6f4faf87ff7a81dabe75141d0335367a7f0cd3ae39ee74d878aaf68da4888c86"
)
# Kept as a narrow compatibility alias for the discovery producer contract.
# It is never compared with the active consumer profile bytes.
EXPECTED_PROFILE_SHA256 = DISCOVERY_PROFILE_SHA256
EXPECTED_IMAGE_DIGEST = (
    "sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988"
)
EXPECTED_IMAGE_PLATFORM = "linux/amd64"
EXPECTED_COMMIT = "2667056d0320bb77edf1e28fec13d56aa8d325e2"
EXPECTED_ARCHIVE_SHA256 = (
    "0524ace3a4bd5db78320ac92a69ac6dde09c60fa07f537c0622f866136e47f17"
)
EXPECTED_ARCHIVE_BYTES = 690616
MAX_INPUT_BYTES = 128 * 1024 * 1024
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
MAX_OUTPUT_BYTES = 16 * 1024 * 1024
MAX_CACHE_ENTRIES = 4096
MAX_CACHE_LINE_BYTES = 4096
COMMAND_TIMEOUT_SECONDS = 300
SAFE_SHA = re.compile(r"^[0-9a-f]{64}$")
SAFE_PACKAGE = re.compile(r"^[a-z0-9][a-z0-9+.-]*$")
SAFE_VERSION = re.compile(r"^[^\x00\r\n\t ]+$")
SAFE_RELATIVE = re.compile(r"^[^/\.][^\\\x00\r\n]*$")
EXPECTED_PACKAGES = (
    {
        "name": "python3-rosdep",
        "version": "0.26.0-1",
        "architecture": "all",
        "filename": (
            "pool/main/p/python3-rosdep/"
            "python3-rosdep_0.26.0-1_all.deb"
        ),
        "size_bytes": 3320,
        "sha256": (
            "456a9980b625b46f842c370b76ff63da2647b0df19890eb9132c6d06cc7949e4"
        ),
        "uri": (
            "http://packages.ros.org/ros2/ubuntu/pool/main/p/python3-rosdep/"
            "python3-rosdep_0.26.0-1_all.deb"
        ),
        "artifact_path": (
            "artifacts/apt/debs/python3-rosdep_0.26.0-1_all.deb"
        ),
    },
    {
        "name": "python3-rosdep-modules",
        "version": "0.26.0-1",
        "architecture": "all",
        "filename": (
            "pool/main/p/python3-rosdep-modules/"
            "python3-rosdep-modules_0.26.0-1_all.deb"
        ),
        "size_bytes": 54676,
        "sha256": (
            "a152df5e76adbf08b69df6d414d38847221b4db7929a4329b940e74bdd1a0ad4"
        ),
        "uri": (
            "http://packages.ros.org/ros2/ubuntu/pool/main/p/python3-rosdep-"
            "modules/python3-rosdep-modules_0.26.0-1_all.deb"
        ),
        "artifact_path": (
            "artifacts/apt/debs/python3-rosdep-modules_0.26.0-1_all.deb"
        ),
    },
    {
        "name": "python3-rosdistro",
        "version": "1.0.1-100",
        "architecture": "all",
        "filename": (
            "pool/main/p/python3-rosdistro/"
            "python3-rosdistro_1.0.1-100_all.deb"
        ),
        "size_bytes": 3724,
        "sha256": (
            "fbf524890faed0fff7fa4fcf08c80f656d65dd6db627f4887ff130fdb23541b2"
        ),
        "uri": (
            "http://packages.ros.org/ros2/ubuntu/pool/main/p/python3-rosdistro/"
            "python3-rosdistro_1.0.1-100_all.deb"
        ),
        "artifact_path": (
            "artifacts/apt/debs/python3-rosdistro_1.0.1-100_all.deb"
        ),
    },
    {
        "name": "python3-rosdistro-modules",
        "version": "1.0.1-1",
        "architecture": "all",
        "filename": (
            "pool/main/p/python3-rosdistro-modules/"
            "python3-rosdistro-modules_1.0.1-1_all.deb"
        ),
        "size_bytes": 36220,
        "sha256": (
            "869aea37baac9bd129b6deda48284ab8eec7401f28558c596358e01df659c365"
        ),
        "uri": (
            "http://packages.ros.org/ros2/ubuntu/pool/main/p/python3-rosdistro-"
            "modules/python3-rosdistro-modules_1.0.1-1_all.deb"
        ),
        "artifact_path": (
            "artifacts/apt/debs/python3-rosdistro-modules_1.0.1-1_all.deb"
        ),
    },
)
EXPECTED_SELECTED_FILES = (
    {
        "relative_path": "index-v4.yaml",
        "bytes": 4857,
        "sha256": (
            "d515c14da215282a0b6ed456e6f6267633bbc8f6cf6abe379c0f2a25aeb6453f"
        ),
        "artifact_path": "artifacts/rosdistro/files/index-v4.yaml",
    },
    {
        "relative_path": "index.yaml",
        "bytes": 3054,
        "sha256": (
            "db265add7fe4784d4f4005ef67515f9d39d828092c84d70ab4e95c6e23c1a779"
        ),
        "artifact_path": "artifacts/rosdistro/files/index.yaml",
    },
    {
        "relative_path": "releases/fuerte.yaml",
        "bytes": 5604,
        "sha256": (
            "f7df4e02a4d20d712757b14c6faa299e4cf17d958a740a7c3e4aa76c72d4f80f"
        ),
        "artifact_path": "artifacts/rosdistro/files/releases/fuerte.yaml",
    },
    {
        "relative_path": "rosdep/base.yaml",
        "bytes": 274441,
        "sha256": (
            "9e93b1b9a5c3ee4b0f39e947c72717979a5dc3b7b316de0bb82987015f333882"
        ),
        "artifact_path": "artifacts/rosdistro/files/rosdep/base.yaml",
    },
    {
        "relative_path": "rosdep/osx-homebrew.yaml",
        "bytes": 12185,
        "sha256": (
            "21a4ef81e25d3ae0956b1cfa609d53d59e25a1c6a1cbbb8d3149a1fdc36f47ac"
        ),
        "artifact_path": (
            "artifacts/rosdistro/files/rosdep/osx-homebrew.yaml"
        ),
    },
    {
        "relative_path": "rosdep/python.yaml",
        "bytes": 216331,
        "sha256": (
            "dbca7ac0ebc8ae827f530f030f5e87e1f3b3ab9144bc8739b44a1a4454adfc5e"
        ),
        "artifact_path": "artifacts/rosdistro/files/rosdep/python.yaml",
    },
    {
        "relative_path": "rosdep/ruby.yaml",
        "bytes": 2233,
        "sha256": (
            "0beda9b7528c4d88d2a4acead61dd019075f76f052dd7b4d978f94561341c65a"
        ),
        "artifact_path": "artifacts/rosdistro/files/rosdep/ruby.yaml",
    },
    {
        "relative_path": "rosdep/sources.list.d/20-default.list",
        "bytes": 588,
        "sha256": (
            "47fee93015482cb2f61193315f23ee1e3fb83c410ffeee506437002f08b3b34f"
        ),
        "artifact_path": (
            "artifacts/rosdistro/files/rosdep/sources.list.d/20-default.list"
        ),
    },
)
EXPECTED_SOURCE_RECORDS = (
    {
        "commit_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/"
            "2667056d0320bb77edf1e28fec13d56aa8d325e2/"
            "rosdep/osx-homebrew.yaml"
        ),
        "distribution": "osx",
        "line": 2,
        "relative_path": "rosdep/osx-homebrew.yaml",
        "requested_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/master/"
            "rosdep/osx-homebrew.yaml"
        ),
        "type": "yaml",
    },
    {
        "commit_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/"
            "2667056d0320bb77edf1e28fec13d56aa8d325e2/rosdep/base.yaml"
        ),
        "line": 5,
        "relative_path": "rosdep/base.yaml",
        "requested_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/master/"
            "rosdep/base.yaml"
        ),
        "type": "yaml",
    },
    {
        "commit_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/"
            "2667056d0320bb77edf1e28fec13d56aa8d325e2/rosdep/python.yaml"
        ),
        "line": 6,
        "relative_path": "rosdep/python.yaml",
        "requested_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/master/"
            "rosdep/python.yaml"
        ),
        "type": "yaml",
    },
    {
        "commit_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/"
            "2667056d0320bb77edf1e28fec13d56aa8d325e2/rosdep/ruby.yaml"
        ),
        "line": 7,
        "relative_path": "rosdep/ruby.yaml",
        "requested_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/master/"
            "rosdep/ruby.yaml"
        ),
        "type": "yaml",
    },
    {
        "commit_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/"
            "2667056d0320bb77edf1e28fec13d56aa8d325e2/releases/fuerte.yaml"
        ),
        "distribution": "fuerte",
        "line": 8,
        "relative_path": "releases/fuerte.yaml",
        "requested_url": (
            "https://raw.githubusercontent.com/ros/rosdistro/master/"
            "releases/fuerte.yaml"
        ),
        "type": "gbpdistro",
    },
)
SOURCE_RECORD_PATHS = frozenset(
    item["relative_path"] for item in EXPECTED_SOURCE_RECORDS
)
EXPECTED_LOCAL_SOURCE_ROOT = "/opt/registration-plugin/rosdistro/files"
EXPECTED_LOCAL_SOURCE_LIST = (
    "/opt/registration-plugin/rosdistro/sources.list.d/20-local.list"
)
EXPECTED_ROS_HOME = "/opt/registration-plugin/ros-home"
EXPECTED_ROSDEP_CACHE = "/opt/registration-plugin/rosdep-cache"
EXPECTED_GENERATED_ROSDEP_CACHE = EXPECTED_ROS_HOME + "/rosdep/sources.cache"
EXPECTED_SOURCE_LIST_DIR = "/opt/registration-plugin/rosdistro/sources.list.d"
EXPECTED_OFFLINE_INDEX = (
    "/workspace/evidence-parent/rosdep-prepare/"
    "artifacts/rosdep/index-v4-offline.yaml"
)
SUPPORTED_DISTROS = ("humble", "jazzy")
OFFLINE_DISTRIBUTIONS = {
    "humble": {
        "member": "rosdistro-{}/humble/distribution.yaml".format(EXPECTED_COMMIT),
        "bytes": 413293,
        "sha256": "e6e6e374f89ea9d961dc62e0f000adaf5d67ae82b4fd319078aac0c3229d50b2",
    },
    "jazzy": {
        "member": "rosdistro-{}/jazzy/distribution.yaml".format(EXPECTED_COMMIT),
        "bytes": 393061,
        "sha256": "0c67bbab0e1c6d4b223e424f76b016493d272ef60b012d1c3741c271e124f9a8",
    },
}
# Compatibility names retained for existing Humble-focused consumers/tests.
EXPECTED_HUMBLE_DISTRIBUTION_MEMBER = OFFLINE_DISTRIBUTIONS["humble"]["member"]
EXPECTED_HUMBLE_DISTRIBUTION_BYTES = OFFLINE_DISTRIBUTIONS["humble"]["bytes"]
EXPECTED_HUMBLE_DISTRIBUTION_SHA256 = OFFLINE_DISTRIBUTIONS["humble"]["sha256"]


def _offline_distribution_relative(distro: str) -> str:
    if distro not in SUPPORTED_DISTROS:
        raise PrepareError("DISTRO_INVALID", distro)
    return "artifacts/rosdep/{}-distribution.yaml".format(distro)


def _offline_index_bytes(distro: str) -> bytes:
    distribution = _offline_distribution_relative(distro)
    return (
        "%YAML 1.1\n---\ntype: index\nversion: 4\ndistributions:\n"
        "  {0}:\n"
        "    distribution:\n"
        "      - file:///workspace/evidence-parent/rosdep-prepare/{1}\n"
        "    distribution_status: active\n"
        "    distribution_type: ros2\n"
        "    python_version: 3\n"
    ).format(distro, distribution).encode("ascii")


OFFLINE_INDEX_BYTES = _offline_index_bytes("humble")
EXPECTED_ROSDEP_KEYS = ("libg2o", "libpcl-all-dev", "python3-yaml")
ENABLED_SOURCE_TYPES = frozenset({"yaml"})
ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING = (
    b"/usr/bin/rosdep:6: DeprecationWarning: pkg_resources is deprecated as an "
    b"API. See https://setuptools.pypa.io/en/latest/pkg_resources.html\n"
    b"  from pkg_resources import load_entry_point\n"
)
EXPECTED_NETWORK_ENV = {
    "HOME": "/nonexistent",
    "PATH": "/usr/sbin:/usr/bin:/sbin:/bin",
    "ROS_HOME": EXPECTED_ROS_HOME,
    "ROSDEP_SOURCE_PATH": EXPECTED_SOURCE_LIST_DIR,
    "ROSDEP_CACHE_DIR": EXPECTED_GENERATED_ROSDEP_CACHE,
    "ROSDISTRO_INDEX_URL": "file://" + EXPECTED_OFFLINE_INDEX,
    "http_proxy": "",
    "https_proxy": "",
    "all_proxy": "",
    "NO_PROXY": "*",
}
EXPECTED_INSTALL_ENV = {
    "DEBIAN_FRONTEND": "noninteractive",
    "HOME": "/nonexistent",
    "PATH": "/usr/sbin:/usr/bin:/sbin:/bin",
}
# These directories are created and identity-bound by the outer capture
# process.  The prepare helper may use them but must not recreate, replace, or
# recursively scan them as if they were its own output tree.
HOST_PRECREATED_RELATIVES = frozenset({
    "input",
    "work",
    "work/rosdistro",
    # This is the host-side Docker mount target for the read-only rosdistro
    # files bind.  It must exist before container creation so Docker cannot
    # create it and change the parent directory link count.
    "work/rosdistro/files",
    "work/rosdistro/sources.list.d",
    "work/rosdep-cache",
    "work/ros-home",
    "work/ros-home/rosdep",
    "work/ros-home/rosdep/meta.cache",
    "work/ros-home/rosdep/sources.cache",
})
PREPARE_OUTPUT_DIRECTORY_RELATIVES = frozenset({
    "commands",
    "artifacts",
    "artifacts/rosdep",
    "artifacts/rosdep/sources.list.d",
    "artifacts/rosdep/cache",
})
PREPARE_PRECREATED_DIRECTORY_RELATIVES = frozenset(
    PREPARE_OUTPUT_DIRECTORY_RELATIVES | HOST_PRECREATED_RELATIVES
)
PREPARE_INPUT_FILE_NAMES = frozenset({
    "prepare_registration_plugin_rosdep.py",
    "registration_plugin_rosdep_prepare_v1.schema.json",
    "capture_registration_plugin_rosdep_discovery.py",
    "consumer-profile.json",
})
PREPARE_INPUT_BINDING_SCHEMA = "registration-plugin-rosdep-prepare-input-v1"
PREPARE_INPUT_CONTAINER_ROOT = "/opt/registration-plugin/prepare"
PREPARE_INPUT_SPECS = (
    {
        "role": "tool",
        "source_path": "scripts/prepare_registration_plugin_rosdep.py",
        "container_path": (
            PREPARE_INPUT_CONTAINER_ROOT
            + "/prepare_registration_plugin_rosdep.py"
        ),
        "relative_path": "prepare_registration_plugin_rosdep.py",
        "repository_path": SCRIPT_DIR / "prepare_registration_plugin_rosdep.py",
    },
    {
        "role": "schema",
        "source_path": (
            "configs/slam_benchmark_profiles/"
            "registration_plugin_rosdep_prepare_v1.schema.json"
        ),
        "container_path": (
            PREPARE_INPUT_CONTAINER_ROOT
            + "/registration_plugin_rosdep_prepare_v1.schema.json"
        ),
        "relative_path": "registration_plugin_rosdep_prepare_v1.schema.json",
        "repository_path": ROOT / (
            "configs/slam_benchmark_profiles/"
            "registration_plugin_rosdep_prepare_v1.schema.json"
        ),
    },
    {
        "role": "discovery_validator",
        "source_path": "scripts/capture_registration_plugin_rosdep_discovery.py",
        "container_path": (
            PREPARE_INPUT_CONTAINER_ROOT
            + "/capture_registration_plugin_rosdep_discovery.py"
        ),
        "relative_path": "capture_registration_plugin_rosdep_discovery.py",
        "repository_path": SCRIPT_DIR / "capture_registration_plugin_rosdep_discovery.py",
    },
    {
        "role": "consumer_profile",
        "source_path": "<active-profile>",
        "container_path": PREPARE_INPUT_CONTAINER_ROOT + "/consumer-profile.json",
        "relative_path": "consumer-profile.json",
        "repository_path": PROFILE_PATH,
    },
)
PREPARE_DIRECTORY_POLICY = (
    "host-precreated-exact-tree-0700-owner-bound-nofollow-v1"
)


class PrepareError(RuntimeError):
    """A fail-closed discovery-consumer or prepare error."""

    def __init__(self, kind: str, message: str):
        super().__init__("{}: {}".format(kind, message))
        self.kind = kind


def canonical_bytes(value: Any) -> bytes:
    """Return the deterministic JSON bytes used by this contract."""
    return json.dumps(
        value, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_hash(value: Mapping[str, Any]) -> str:
    """Hash a value after removing its self-referential canonical field."""
    projection = dict(value)
    projection.pop("canonical_sha256", None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_relative(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value.startswith("/")
        or "\\" in value
        or any(part in ("", ".", "..") for part in value.split("/"))
        or PurePosixPath(value).as_posix() != value
    ):
        raise PrepareError("PATH_INVALID", label)
    return value


def _check_absolute(value: Any, label: str) -> Path:
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or "\x00" in value
        or "\r" in value
        or "\n" in value
        or Path(value).as_posix() != value
        or any(part in ("", ".", "..") for part in value.split("/")[1:])
    ):
        raise PrepareError("PATH_INVALID", label)
    return Path(value)


def _check_parents(path: Path, label: str) -> None:
    """Reject symlink or non-directory components without resolving them."""
    current = path.parent
    while True:
        try:
            info = os.lstat(str(current))
        except OSError as exc:
            raise PrepareError("PARENT_INVALID", label) from exc
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise PrepareError("PARENT_INVALID", label)
        if current == Path("/"):
            return
        current = current.parent


def _open_regular(path: Path, maximum: int, label: str) -> tuple[int, os.stat_result]:
    """Open a regular single-link file without following the final link."""
    _check_parents(path, label)
    try:
        descriptor = os.open(
            str(path), os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
        )
        before = os.fstat(descriptor)
    except OSError as exc:
        raise PrepareError("ARTIFACT_OPEN_FAILED", label) from exc
    if (
        not stat.S_ISREG(before.st_mode)
        or before.st_nlink != 1
        or before.st_size < 0
        or before.st_size > maximum
        or stat.S_IMODE(before.st_mode) != 0o444
    ):
        os.close(descriptor)
        raise PrepareError("ARTIFACT_METADATA_INVALID", label)
    return descriptor, before


def _read_descriptor(path: Path, maximum: int, label: str) -> tuple[bytes, dict[str, Any]]:
    descriptor, before = _open_regular(path, maximum, label)
    try:
        chunks: list[bytes] = []
        total = 0
        while True:
            block = os.read(descriptor, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            total += len(block)
            if total > maximum:
                raise PrepareError("ARTIFACT_OVERSIZE", label)
            chunks.append(block)
        after = os.fstat(descriptor)
        if (
            (before.st_dev, before.st_ino, before.st_size, before.st_mode,
             before.st_uid, before.st_gid, before.st_nlink)
            != (after.st_dev, after.st_ino, after.st_size, after.st_mode,
                after.st_uid, after.st_gid, after.st_nlink)
            or total != before.st_size
        ):
            raise PrepareError("ARTIFACT_CHANGED", label)
        payload = b"".join(chunks)
        digest = _sha256(payload)
        os.lseek(descriptor, 0, os.SEEK_SET)
        second = b"".join(iter(lambda: os.read(descriptor, 1024 * 1024), b""))
        final = os.fstat(descriptor)
        if (
            digest != _sha256(second)
            or (after.st_dev, after.st_ino, after.st_size, after.st_mode,
                after.st_uid, after.st_gid, after.st_nlink)
            != (final.st_dev, final.st_ino, final.st_size, final.st_mode,
                final.st_uid, final.st_gid, final.st_nlink)
        ):
            raise PrepareError("ARTIFACT_CHANGED", label)
        return payload, {
            "path": str(path),
            "bytes": len(payload),
            "sha256": digest,
            "mode": stat.S_IMODE(before.st_mode),
            "uid": before.st_uid,
            "gid": before.st_gid,
            "nlink": before.st_nlink,
            "device": before.st_dev,
            "inode": before.st_ino,
        }
    finally:
        os.close(descriptor)


def _read_sidecar(path: Path, digest: str) -> dict[str, Any]:
    sidecar = Path(str(path) + ".sha256")
    payload, descriptor = _read_descriptor(sidecar, 512, "sidecar")
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise PrepareError("SIDECAR_INVALID", str(sidecar)) from exc
    if text != "{} {}\n".format(digest, path.name):
        raise PrepareError("SIDECAR_MISMATCH", str(path))
    return descriptor


def _read_sealed(path: Path, maximum: int, label: str) -> tuple[bytes, dict[str, Any]]:
    payload, descriptor = _read_descriptor(path, maximum, label)
    digest = descriptor["sha256"]
    sidecar = _read_sidecar(path, digest)
    descriptor["sidecar"] = sidecar
    return payload, descriptor


def _read_input_source_descriptor(
    path: Path, maximum: int, label: str
) -> dict[str, Any]:
    """Read an input file without requiring a producer-side sidecar.

    The four files in the prepare bundle are immutable inputs to this phase,
    but the source copy may be the repository checkout (where a sidecar is
    not present) or the 0444 staged bundle.  In both cases the descriptor is
    obtained from one ``O_NOFOLLOW`` file descriptor and checked again after
    reading.  The copied/bind-visible descriptor is therefore never inferred
    from a pathname alone.
    """
    _check_parents(path, label)
    try:
        descriptor = os.open(
            str(path), os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
        )
        before = os.fstat(descriptor)
    except OSError as exc:
        raise PrepareError("INPUT_BINDING_OPEN_FAILED", label) from exc
    try:
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or before.st_size < 1
            or before.st_size > maximum
        ):
            raise PrepareError("INPUT_BINDING_METADATA_INVALID", label)
        chunks: list[bytes] = []
        total = 0
        while True:
            block = os.read(descriptor, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            total += len(block)
            if total > maximum:
                raise PrepareError("INPUT_BINDING_OVERSIZE", label)
            chunks.append(block)
        after = os.fstat(descriptor)
        if (
            total != before.st_size
            or (before.st_dev, before.st_ino, before.st_size, before.st_mode,
                before.st_uid, before.st_gid, before.st_nlink)
            != (after.st_dev, after.st_ino, after.st_size, after.st_mode,
                after.st_uid, after.st_gid, after.st_nlink)
        ):
            raise PrepareError("INPUT_BINDING_CHANGED", label)
        payload = b"".join(chunks)
        digest = _sha256(payload)
        return {
            "path": str(path),
            "bytes": len(payload),
            "sha256": digest,
            "mode": stat.S_IMODE(before.st_mode),
            "uid": int(before.st_uid),
            "gid": int(before.st_gid),
            "nlink": int(before.st_nlink),
            "device": int(before.st_dev),
            "inode": int(before.st_ino),
        }
    finally:
        os.close(descriptor)


def _input_repository_path(spec: Mapping[str, Any]) -> Path:
    """Resolve one input from the staged bundle or checkout.

    A production invocation runs from the four-file read-only bundle, so the
    basename under ``SCRIPT_DIR`` wins.  Unit tests and direct offline use
    run from the checkout, where the schema and active profile retain their
    repository locations.  No caller-supplied path is accepted.
    """
    bundled = SCRIPT_DIR / str(spec["relative_path"])
    if bundled.exists():
        return bundled
    # Resolve the checkout location from current module globals rather than
    # from the import-time constants in ``PREPARE_INPUT_SPECS``.  This keeps
    # readback strict when a test stages an alternate active profile, while a
    # container invocation still takes the four bundled files above.
    role = spec["role"]
    if role == "consumer_profile":
        return PROFILE_PATH
    if role == "schema":
        return ROOT / (
            "configs/slam_benchmark_profiles/"
            "registration_plugin_rosdep_prepare_v1.schema.json"
        )
    return ROOT / "scripts" / str(spec["relative_path"])


def _prepare_input_binding() -> dict[str, Any]:
    """Build the immutable four-file binding used by every receipt outcome."""
    paths = [_input_repository_path(spec) for spec in PREPARE_INPUT_SPECS]
    try:
        common_root = Path(os.path.commonpath([str(path) for path in paths]))
    except ValueError as exc:
        raise PrepareError("INPUT_BINDING_PATH_INVALID", "common root") from exc
    files: list[dict[str, Any]] = []
    for spec, path in zip(PREPARE_INPUT_SPECS, paths):
        source = _read_input_source_descriptor(
            path, MAX_INPUT_BYTES, str(spec["role"]) + " input"
        )
        # ``source``, ``before`` and ``after`` are deliberately distinct
        # fields even when direct/offline execution sees the same inode.  A
        # container transport has no permission to rewrite these identities;
        # reopening the receipt recomputes all three from the visible input.
        files.append({
            "role": spec["role"],
            "source_path": spec["source_path"],
            "container_path": spec["container_path"],
            "relative_path": spec["relative_path"],
            "source": dict(source),
            "before": dict(source),
            "after": dict(source),
        })
    binding: dict[str, Any] = {
        "schema": PREPARE_INPUT_BINDING_SCHEMA,
        "schema_version": 1,
        "relative_root": "rosdep-prepare/input",
        "container_root": PREPARE_INPUT_CONTAINER_ROOT,
        "mount": {
            "source": str(common_root),
            "target": PREPARE_INPUT_CONTAINER_ROOT,
            "read_only": True,
            "noexec": False,
            "repository_bind": "forbidden",
        },
        "files": files,
    }
    binding["identity_sha256"] = hashlib.sha256(
        canonical_bytes(binding)
    ).hexdigest()
    return binding


def _validate_prepare_input_binding(
    value: Any, *, reopen: bool = True
) -> None:
    """Reopen and strictly validate the four-file binding in a receipt."""
    if not isinstance(value, Mapping) or set(value) != {
        "schema", "schema_version", "relative_root", "container_root",
        "mount", "files", "identity_sha256",
    }:
        raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "fields")
    if (
        value.get("schema") != PREPARE_INPUT_BINDING_SCHEMA
        or value.get("schema_version") != 1
        or value.get("relative_root") != "rosdep-prepare/input"
        or value.get("container_root") != PREPARE_INPUT_CONTAINER_ROOT
        or value.get("identity_sha256") != hashlib.sha256(
            canonical_bytes({
                key: value[key] for key in value
                if key != "identity_sha256"
            })
        ).hexdigest()
    ):
        raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "identity")
    mount = value.get("mount")
    if (
        not isinstance(mount, Mapping)
        or set(mount) != {
            "source", "target", "read_only", "noexec", "repository_bind"
        }
        or not isinstance(mount.get("source"), str)
        or not mount["source"].startswith("/")
        or mount.get("target") != PREPARE_INPUT_CONTAINER_ROOT
        or mount.get("read_only") is not True
        or mount.get("noexec") is not False
        or mount.get("repository_bind") != "forbidden"
    ):
        raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "mount")
    files = value.get("files")
    if not isinstance(files, list) or len(files) != len(PREPARE_INPUT_SPECS):
        raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "file count")
    observed_paths: set[str] = set()
    descriptor_fields = {
        "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
        "device", "inode",
    }
    for item, spec in zip(files, PREPARE_INPUT_SPECS):
        if (
            not isinstance(item, Mapping)
            or set(item) != {
                "role", "source_path", "container_path", "relative_path",
                "source", "before", "after",
            }
            or item.get("role") != spec["role"]
            or item.get("source_path") != spec["source_path"]
            or item.get("container_path") != spec["container_path"]
            or item.get("relative_path") != spec["relative_path"]
        ):
            raise PrepareError("PREPARE_INPUT_BINDING_INVALID", str(spec["role"]))
        relative = item["relative_path"]
        if relative in observed_paths:
            raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "duplicate path")
        observed_paths.add(relative)
        source = item.get("source")
        before = item.get("before")
        after = item.get("after")
        for label, descriptor in (
            ("source", source), ("before", before), ("after", after)
        ):
            if (
                not isinstance(descriptor, Mapping)
                or set(descriptor) != descriptor_fields
                or not isinstance(descriptor.get("path"), str)
                or not descriptor["path"].startswith("/")
                or type(descriptor.get("bytes")) is not int
                or descriptor["bytes"] < 1
                or descriptor["bytes"] > MAX_INPUT_BYTES
                or SAFE_SHA.fullmatch(str(descriptor.get("sha256"))) is None
                or type(descriptor.get("mode")) is not int
                or descriptor["mode"] < 0
                or descriptor["mode"] > 0o777
                or type(descriptor.get("uid")) is not int
                or descriptor["uid"] < 0
                or type(descriptor.get("gid")) is not int
                or descriptor["gid"] < 0
                or descriptor.get("nlink") != 1
                or type(descriptor.get("device")) is not int
                or descriptor["device"] < 0
                or type(descriptor.get("inode")) is not int
                or descriptor["inode"] < 1
            ):
                raise PrepareError(
                    "PREPARE_INPUT_BINDING_INVALID", str(spec["role"]) + ":" + label
                )
        if before != after or source != before:
            raise PrepareError("PREPARE_INPUT_BINDING_INVALID", str(spec["role"]))
        if reopen:
            actual = _read_input_source_descriptor(
                Path(source["path"]), MAX_INPUT_BYTES,
                str(spec["role"]) + " reopen"
            )
            if actual != dict(source):
                raise PrepareError(
                    "PREPARE_INPUT_BINDING_CHANGED", str(spec["role"])
                )
    if observed_paths != {
        str(spec["relative_path"]) for spec in PREPARE_INPUT_SPECS
    }:
        raise PrepareError("PREPARE_INPUT_BINDING_INVALID", "path set")


def _discovery_module(logical_profile_path: str | None = None) -> Any:
    path = ROOT / "scripts/capture_registration_plugin_rosdep_discovery.py"
    bundled = SCRIPT_DIR / "capture_registration_plugin_rosdep_discovery.py"
    if bundled.is_file():
        path = bundled
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_rosdep_discovery_consumer", path
    )
    if spec is None or spec.loader is None:
        raise PrepareError("DISCOVERY_IMPORT_FAILED", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if logical_profile_path is not None:
        module.PROFILE_PATH = Path(logical_profile_path)
    return module


def _profile_sha(path: Path) -> str:
    _check_parents(path, "profile")
    try:
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise PrepareError("PROFILE_INVALID", str(path))
        data = path.read_bytes()
    except OSError as exc:
        raise PrepareError("PROFILE_READ_FAILED", str(path)) from exc
    if len(data) > MAX_INPUT_BYTES:
        raise PrepareError("PROFILE_OVERSIZE", str(path))
    return _sha256(data)


def _discovery_allowlist(logical_profile_path: str | None = None) -> dict[str, Any]:
    """Return the immutable discovery identity allowlisted by the consumer."""
    if logical_profile_path is None:
        logical_profile_path = str(REPOSITORY_PROFILE_PATH)
    archive_path = (
        "artifacts/rosdistro/rosdistro-{}-archive.tar.gz".format(EXPECTED_COMMIT)
    )
    package_paths = [item["artifact_path"] for item in EXPECTED_PACKAGES]
    selected_paths = [item["artifact_path"] for item in EXPECTED_SELECTED_FILES]
    return {
        "discovery_id": DISCOVERY_ID,
        "root": str(PINNED_DISCOVERY_ROOT),
        "receipt": {
            "path": str(PINNED_DISCOVERY_ROOT / DISCOVERY_RECEIPT_NAME),
            "sha256": EXPECTED_RECEIPT_SHA256,
            "canonical_sha256": EXPECTED_DISCOVERY_CANONICAL_SHA256,
        },
        "profile": {
            "path": logical_profile_path, "sha256": DISCOVERY_PROFILE_SHA256,
        },
        "image": {
            "digest": EXPECTED_IMAGE_DIGEST,
            "platform": EXPECTED_IMAGE_PLATFORM,
            "repo_digests": ["ros@" + EXPECTED_IMAGE_DIGEST],
        },
        "rosdistro": {
            "repository": "https://github.com/ros/rosdistro.git",
            "commit": EXPECTED_COMMIT,
            "archive_url": (
                "https://codeload.github.com/ros/rosdistro/tar.gz/"
                + EXPECTED_COMMIT
            ),
            "archive_bytes": EXPECTED_ARCHIVE_BYTES,
            "archive_sha256": EXPECTED_ARCHIVE_SHA256,
        },
        "packages": [
            {key: item[key] for key in (
                "name", "version", "architecture", "filename", "uri",
                "size_bytes", "sha256", "artifact_path"
            )}
            for item in EXPECTED_PACKAGES
        ],
        "selected_files": [
            {key: item[key] for key in (
                "relative_path", "bytes", "sha256", "artifact_path"
            )}
            for item in EXPECTED_SELECTED_FILES
        ],
        "source_records": [dict(item) for item in EXPECTED_SOURCE_RECORDS],
        "receipt_bytes_sha256": EXPECTED_RECEIPT_SHA256,
        "artifact_paths": sorted([archive_path, *package_paths, *selected_paths]),
    }


def _load_consumer_profile_policy(profile_path: Path) -> Mapping[str, Any]:
    """Load the explicit consumer policy from the active profile."""
    try:
        payload = profile_path.read_bytes()
        profile = json.loads(payload.decode("utf-8", "strict"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PrepareError("CONSUMER_PROFILE_INVALID", str(profile_path)) from exc
    try:
        policy = profile["release_matrix"]["dependency_closure"][
            "consumer_profile_binding"
        ]
    except (KeyError, TypeError) as exc:
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_MISSING", str(profile_path)) from exc
    return policy


def _logical_consumer_profile_path(profile_path: Path) -> str:
    """Return the producer's logical profile path from the consumer policy.

    The profile used to execute this module may be a staged container file.
    That transport path is deliberately not the identity recorded by the
    discovery producer.  Only the policy's absolute, current-profile path is
    allowed to bind the producer receipt.
    """
    policy = _load_consumer_profile_policy(profile_path)
    consumer = policy.get("consumer_profile") if isinstance(policy, Mapping) else None
    logical_profile_path = (
        consumer.get("path") if isinstance(consumer, Mapping) else None
    )
    if (
        not isinstance(consumer, Mapping)
        or set(consumer) != {"path", "sha256_source"}
        or not isinstance(logical_profile_path, str)
        or consumer.get("sha256_source") != "active_profile_bytes_sha256"
    ):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "consumer profile")
    try:
        _check_absolute(logical_profile_path, "consumer profile path")
    except PrepareError as exc:
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "profile path") from exc
    if not logical_profile_path.endswith(
        "/configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
    ):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "profile path")
    return logical_profile_path


def _consumer_profile_binding(profile_path: Path,
                              discovery: Mapping[str, Any]) -> dict[str, str]:
    """Validate the active profile's explicit immutable discovery allowlist."""
    if profile_path != PROFILE_PATH or not profile_path.is_file():
        raise PrepareError("PROFILE_NOT_PINNED", str(profile_path))
    policy = _load_consumer_profile_policy(profile_path)
    if not isinstance(policy, Mapping) or set(policy) != {
            "schema", "schema_version", "consumer_profile",
            "discovery_profile_sha256", "discovery_allowlist"}:
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "fields")
    if (policy["schema"] != "registration-plugin-rosdep-consumer-profile-binding-v1"
            or policy["schema_version"] != 1
            or policy["discovery_profile_sha256"] != DISCOVERY_PROFILE_SHA256):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "schema/identity")
    consumer = policy["consumer_profile"]
    logical_profile_path = (
        consumer.get("path") if isinstance(consumer, Mapping) else None
    )
    if not isinstance(consumer, Mapping) or set(consumer) != {
            "path", "sha256_source"} or \
            not isinstance(logical_profile_path, str) or \
            not logical_profile_path.startswith("/") or \
            consumer["sha256_source"] != "active_profile_bytes_sha256":
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "consumer profile")
    if (profile_path == REPOSITORY_PROFILE_PATH and
            logical_profile_path != str(REPOSITORY_PROFILE_PATH)):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "profile path")
    # Keep this check aligned with the helper used by discovery projection.
    if logical_profile_path != _logical_consumer_profile_path(profile_path):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "profile path")
    if policy["discovery_allowlist"] != _discovery_allowlist(logical_profile_path):
        raise PrepareError("CONSUMER_PROFILE_ALLOWLIST_INVALID", "discovery allowlist")
    observed = _discovery_projection(discovery)
    observed["root"] = str(PINNED_DISCOVERY_ROOT)
    observed["receipt"]["path"] = str(
        PINNED_DISCOVERY_ROOT / DISCOVERY_RECEIPT_NAME
    )
    if observed != policy["discovery_allowlist"]:
        # The discovery projection is independently checked before reaching
        # here; this comparison prevents a profile policy from being merely
        # decorative or rebound to a different receipt.
        raise PrepareError("DISCOVERY_ALLOWLIST_MISMATCH", "sealed discovery")
    return {"path": logical_profile_path, "sha256": _profile_sha(profile_path)}


def _discovery_projection(binding: Mapping[str, Any]) -> dict[str, Any]:
    """Remove runtime inode descriptors while retaining every pinned identity."""
    if not isinstance(binding, Mapping):
        raise PrepareError("DISCOVERY_ALLOWLIST_MISMATCH", "binding")
    try:
        packages = [
            {key: item[key] for key in (
                "name", "version", "architecture", "filename", "uri",
                "size_bytes", "sha256"
            )} | {"artifact_path": item["artifact"]["path"]}
            for item in binding["packages"]
        ]
        selected_files = [
            {key: item[key] for key in (
                "relative_path", "bytes", "sha256"
            )} | {"artifact_path": item["artifact"]["path"]}
            for item in binding["selected_files"]
        ]
        archive = binding["rosdistro"]["archive_artifact"]
        archive_path = archive["path"]
        archive_relative = _safe_relative(archive_path, "archive path")
        return {
            "discovery_id": binding["discovery_id"],
            "root": binding["root"],
            "receipt": {
                key: binding["receipt"][key]
                for key in ("path", "sha256", "canonical_sha256")
            },
            "profile": dict(binding["profile"]),
            "image": dict(binding["image"]),
            "rosdistro": {
                key: binding["rosdistro"][key]
                for key in (
                    "repository", "commit", "archive_url", "archive_bytes",
                    "archive_sha256"
                )
            },
            "packages": packages,
            "selected_files": selected_files,
            "source_records": [dict(item) for item in binding["source_records"]],
            "receipt_bytes_sha256": binding["receipt_bytes_sha256"],
            "artifact_paths": sorted(
                [archive_relative]
                + [item["artifact_path"] for item in packages]
                + [item["artifact_path"] for item in selected_files]
            ),
        }
    except (KeyError, TypeError) as exc:
        raise PrepareError("DISCOVERY_ALLOWLIST_MISMATCH", "binding fields") from exc


def _relative_artifact(root: Path, descriptor: Mapping[str, Any], label: str) -> tuple[Path, dict[str, Any]]:
    value = _safe_relative(descriptor.get("path"), label)
    sidecar = _safe_relative(descriptor.get("sidecar"), label + " sidecar")
    if sidecar != value + ".sha256":
        raise PrepareError("ARTIFACT_BINDING_INVALID", label)
    path = root / value
    expected = dict(descriptor)
    expected["path"] = str(path)
    expected["sidecar"] = str(path) + ".sha256"
    return path, expected


def _validate_artifact(root: Path, descriptor: Mapping[str, Any], expected_sha: str,
                       expected_bytes: int, label: str) -> dict[str, Any]:
    if (
        not isinstance(expected_sha, str)
        or SAFE_SHA.fullmatch(expected_sha) is None
        or type(expected_bytes) is not int
        or expected_bytes < 0
    ):
        raise PrepareError("ARTIFACT_BINDING_INVALID", label)
    path, expected = _relative_artifact(root, descriptor, label)
    if (
        set(descriptor) != {
            "bytes", "mode", "nlink", "path", "sha256", "sidecar"
        }
        or descriptor.get("sha256") != expected_sha
        or descriptor.get("bytes") != expected_bytes
        or descriptor.get("mode") != 0o444
        or descriptor.get("nlink") != 1
    ):
        raise PrepareError("ARTIFACT_BINDING_INVALID", label)
    payload, actual = _read_sealed(path, expected_bytes, label)
    if _sha256(payload) != expected_sha or len(payload) != expected_bytes:
        raise PrepareError("ARTIFACT_BINDING_INVALID", label)
    if any(actual.get(key) != descriptor.get(key) for key in (
        "bytes", "sha256", "mode", "nlink"
    )):
        raise PrepareError("ARTIFACT_METADATA_INVALID", label)
    if expected["sidecar"] != actual["sidecar"]["path"]:
        raise PrepareError("SIDECAR_BINDING_INVALID", label)
    return dict(descriptor)


def _validate_receipt_artifact_index(
    root: Path,
    receipt: Mapping[str, Any],
    required: Mapping[str, Mapping[str, Any]],
) -> None:
    """Reopen the complete discovery artifact index, rejecting aliases."""
    artifacts = receipt.get("artifacts")
    if not isinstance(artifacts, list):
        raise PrepareError("DISCOVERY_ARTIFACT_INDEX_INVALID", "artifact list")
    by_path: dict[str, Mapping[str, Any]] = {}
    for descriptor in artifacts:
        if not isinstance(descriptor, Mapping):
            raise PrepareError("DISCOVERY_ARTIFACT_INDEX_INVALID", "descriptor")
        if set(descriptor) != {
            "bytes", "mode", "nlink", "path", "sha256", "sidecar"
        }:
            raise PrepareError("DISCOVERY_ARTIFACT_INDEX_INVALID", "fields")
        path = _safe_relative(descriptor.get("path"), "discovery artifact")
        if path in by_path:
            raise PrepareError("DISCOVERY_ARTIFACT_INDEX_INVALID", "duplicate")
        by_path[path] = descriptor
        _validate_artifact(
            root, descriptor, descriptor.get("sha256"), descriptor.get("bytes"),
            path
        )
    for path, expected in required.items():
        if by_path.get(path) != expected:
            raise PrepareError("DISCOVERY_ARTIFACT_INDEX_INVALID", path)


def _validate_offline_command(
    argv: list[str], env: Mapping[str, str], label: str
) -> None:
    """Apply the fixed no-network command and environment allowlist."""
    if not argv or argv[0] not in {"dpkg", "dpkg-query", "rosdep", "find"}:
        raise PrepareError("COMMAND_POLICY_INVALID", label)
    if any(
        token.lower().startswith(("http:", "https:"))
        or token.lower() in {"apt", "apt-get", "curl", "wget", "git", "docker"}
        for token in argv
    ):
        raise PrepareError("COMMAND_POLICY_INVALID", label)
    if any(
        not isinstance(key, str)
        or not isinstance(value, str)
        or "\x00" in value
        or "\r" in value
        or "\n" in value
        for key, value in env.items()
    ):
        raise PrepareError("COMMAND_ENV_INVALID", label)
    if argv[0] == "rosdep" or argv[0] == "find":
        if dict(env) != EXPECTED_NETWORK_ENV:
            raise PrepareError("COMMAND_ENV_INVALID", label)
    elif dict(env) != EXPECTED_INSTALL_ENV:
        raise PrepareError("COMMAND_ENV_INVALID", label)


def _expected_source_record(relative_path: str) -> Mapping[str, Any]:
    for item in EXPECTED_SOURCE_RECORDS:
        if item["relative_path"] == relative_path:
            return item
    raise PrepareError("SOURCE_RECORD_UNEXPECTED", relative_path)


def _binding_projection(
    receipt: Mapping[str, Any],
    root: Path,
    *,
    logical_root: Path | None = None,
    logical_profile_path: str | None = None,
) -> dict[str, Any]:
    """Project a discovery receipt while separating path namespaces.

    ``root`` names the readable (host or container) tree.  ``logical_root``
    names the immutable producer path recorded by discovery.  This distinction
    is required when the same host inode is exposed at
    ``/opt/registration-plugin/discovery`` inside the prepare container.
    """
    root = _check_absolute(str(root), "discovery transport root")
    if logical_root is None:
        logical_root = root
    else:
        logical_root = _check_absolute(str(logical_root), "discovery logical root")
    if logical_root != PINNED_DISCOVERY_ROOT:
        raise PrepareError("DISCOVERY_ROOT_NOT_PINNED", str(logical_root))
    if (
        receipt.get("schema") != DISCOVERY_SCHEMA
        or receipt.get("schema_version") != 1
        or receipt.get("status") != "REVIEW_REQUIRED"
        or receipt.get("outcome") != "PASS_REVIEW_REQUIRED"
        or receipt.get("benchmark_eligible") is not False
        or receipt.get("promotion") != "FORBIDDEN_UNTIL_SIGNED_REVIEW"
        or receipt.get("discovery_id") != DISCOVERY_ID
    ):
        raise PrepareError("DISCOVERY_STATUS_INVALID", "candidate status")
    if (
        receipt.get("canonical_sha256") != EXPECTED_DISCOVERY_CANONICAL_SHA256
        or canonical_hash(receipt) != receipt.get("canonical_sha256")
    ):
        raise PrepareError("DISCOVERY_CANONICAL_INVALID", "canonical hash")
    root_value = receipt.get("root")
    if (
        not isinstance(root_value, Mapping)
        or set(root_value) != {
            "device", "gid", "inode", "mode", "nlink", "path", "uid"
        }
        or root_value.get("path") != str(logical_root)
    ):
        raise PrepareError("DISCOVERY_ROOT_INVALID", "root binding")
    try:
        root_info = os.lstat(str(root))
    except OSError as exc:
        raise PrepareError("DISCOVERY_ROOT_INVALID", str(root)) from exc
    if (
        stat.S_ISLNK(root_info.st_mode)
        or not stat.S_ISDIR(root_info.st_mode)
        or {
            "device": root_info.st_dev,
            "inode": root_info.st_ino,
            "mode": stat.S_IMODE(root_info.st_mode),
            "uid": root_info.st_uid,
            "gid": root_info.st_gid,
        }
        != {key: root_value[key] for key in (
            "device", "inode", "mode", "uid", "gid"
        )}
        or type(root_value.get("nlink")) is not int
        or root_value["nlink"] < 2
        or root_info.st_nlink < root_value["nlink"]
    ):
        raise PrepareError("DISCOVERY_ROOT_INVALID", "root identity")
    image = receipt.get("image")
    observed = image.get("observed") if isinstance(image, Mapping) else None
    if (
        not isinstance(image, Mapping)
        or image.get("digest") != EXPECTED_IMAGE_DIGEST
        or image.get("platform") != EXPECTED_IMAGE_PLATFORM
        or not isinstance(observed, Mapping)
        or observed.get("id") != EXPECTED_IMAGE_DIGEST
        or observed.get("os") != "linux"
        or observed.get("architecture") != "amd64"
        or observed.get("repo_digests") != [
            "ros@" + EXPECTED_IMAGE_DIGEST
        ]
    ):
        raise PrepareError("DISCOVERY_IMAGE_INVALID", "fixed image binding")
    expected_logical_profile_path = _logical_consumer_profile_path(PROFILE_PATH)
    if logical_profile_path is None:
        logical_profile_path = expected_logical_profile_path
    elif logical_profile_path != expected_logical_profile_path:
        raise PrepareError("DISCOVERY_PROFILE_INVALID", "consumer path binding")
    profile = receipt.get("profile")
    if (
        not isinstance(profile, Mapping)
        or profile.get("path") != logical_profile_path
        or profile.get("sha256") != DISCOVERY_PROFILE_SHA256
    ):
        raise PrepareError("DISCOVERY_PROFILE_INVALID", "profile binding")
    rosdistro = receipt.get("rosdistro")
    if (
        not isinstance(rosdistro, Mapping)
        or rosdistro.get("repository") != "https://github.com/ros/rosdistro.git"
        or rosdistro.get("commit") != EXPECTED_COMMIT
        or rosdistro.get("ref") != "refs/heads/master"
        or rosdistro.get("status") != "COMMIT_ARCHIVE_CAPTURED_REVIEW_REQUIRED"
    ):
        raise PrepareError("DISCOVERY_SOURCE_INVALID", "rosdistro identity")
    archive = rosdistro.get("archive")
    if not isinstance(archive, Mapping):
        raise PrepareError("DISCOVERY_ARCHIVE_INVALID", "archive missing")
    expected_archive_url = (
        "https://codeload.github.com/ros/rosdistro/tar.gz/" + EXPECTED_COMMIT
    )
    archive_descriptor = archive.get("archive")
    if (
        archive.get("requested_url") != expected_archive_url
        or archive.get("final_url") != expected_archive_url
        or archive.get("redirects") != []
        or archive.get("attempts") != 1
        or not isinstance(archive_descriptor, Mapping)
        or archive_descriptor.get("bytes") != EXPECTED_ARCHIVE_BYTES
        or archive_descriptor.get("sha256") != EXPECTED_ARCHIVE_SHA256
        or archive_descriptor.get("path")
        != "artifacts/rosdistro/rosdistro-{}-archive.tar.gz".format(
            EXPECTED_COMMIT
        )
        or set(archive_descriptor) != {
            "bytes", "mode", "nlink", "path", "sha256", "sidecar"
        }
    ):
        raise PrepareError("DISCOVERY_ARCHIVE_INVALID", "commit archive")
    archive_path, _ = _relative_artifact(root, archive_descriptor, "archive")
    _validate_artifact(
        root, archive_descriptor, EXPECTED_ARCHIVE_SHA256,
        EXPECTED_ARCHIVE_BYTES, "archive"
    )
    if archive_path.name != "rosdistro-{}-archive.tar.gz".format(EXPECTED_COMMIT):
        raise PrepareError("DISCOVERY_ARCHIVE_INVALID", "archive filename")
    apt = receipt.get("apt")
    packages = apt.get("packages") if isinstance(apt, Mapping) else None
    if (
        not isinstance(apt, Mapping)
        or apt.get("status") != "CAPTURED_REVIEW_REQUIRED"
        or not isinstance(packages, list)
        or len(packages) != len(EXPECTED_PACKAGES)
    ):
        raise PrepareError("DISCOVERY_PACKAGES_INVALID", "package count")
    package_bindings: list[dict[str, Any]] = []
    for observed_package, expected_package in zip(packages, EXPECTED_PACKAGES):
        if not isinstance(observed_package, Mapping):
            raise PrepareError("DISCOVERY_PACKAGES_INVALID", "package record")
        for key in (
            "name", "version", "architecture", "filename", "size_bytes",
            "sha256", "uri"
        ):
            if observed_package.get(key) != expected_package[key]:
                raise PrepareError("DISCOVERY_PACKAGE_BINDING_INVALID", key)
        if set(observed_package) != {
            "architecture", "deb_artifact", "filename", "name", "sha256",
            "size_bytes", "source", "uri", "version"
        } or observed_package.get("source") != "apt-cache-show-signed-Packages-field":
            raise PrepareError("DISCOVERY_PACKAGE_BINDING_INVALID", "fields")
        artifact = observed_package.get("deb_artifact")
        if not isinstance(artifact, Mapping):
            raise PrepareError("DISCOVERY_PACKAGE_ARTIFACT_INVALID", "missing")
        if artifact.get("path") != expected_package["artifact_path"]:
            raise PrepareError("DISCOVERY_PACKAGE_ARTIFACT_INVALID", "path")
        _validate_artifact(
            root, artifact, expected_package["sha256"],
            expected_package["size_bytes"], expected_package["name"]
        )
        package_bindings.append({
            "name": expected_package["name"],
            "version": expected_package["version"],
            "architecture": expected_package["architecture"],
            "filename": expected_package["filename"],
            "uri": expected_package["uri"],
            "size_bytes": expected_package["size_bytes"],
            "sha256": expected_package["sha256"],
            "artifact": dict(artifact),
        })
    if (
        apt.get("deb_artifacts")
        != [item["deb_artifact"] for item in packages]
    ):
        raise PrepareError("DISCOVERY_PACKAGE_ARTIFACT_INVALID", "deb set")
    details = rosdistro.get("archive_details")
    selected = details.get("selected_files") if isinstance(details, Mapping) else None
    source_records = details.get("source_records") if isinstance(details, Mapping) else None
    if (
        not isinstance(details, Mapping)
        or details.get("archive_top_level") != "rosdistro-" + EXPECTED_COMMIT
        or not isinstance(selected, list)
        or len(selected) != len(EXPECTED_SELECTED_FILES)
        or source_records != list(EXPECTED_SOURCE_RECORDS)
    ):
        raise PrepareError("DISCOVERY_ROSDISTRO_INVALID", "selected graph")
    selected_bindings: list[dict[str, Any]] = []
    for observed_file, expected_file in zip(selected, EXPECTED_SELECTED_FILES):
        if not isinstance(observed_file, Mapping):
            raise PrepareError("DISCOVERY_FILE_INVALID", "file record")
        for key in ("relative_path", "bytes", "sha256"):
            if observed_file.get(key) != expected_file[key]:
                raise PrepareError("DISCOVERY_FILE_BINDING_INVALID", key)
        expected_member = "rosdistro-{}/{}".format(
            EXPECTED_COMMIT, expected_file["relative_path"]
        )
        if (
            set(observed_file) != {
                "archive_member", "artifact", "bytes", "relative_path",
                "sha256"
            }
            or observed_file.get("archive_member") != expected_member
        ):
            raise PrepareError("DISCOVERY_FILE_BINDING_INVALID", "member")
        artifact = observed_file.get("artifact")
        if (
            not isinstance(artifact, Mapping)
            or artifact.get("path") != expected_file["artifact_path"]
        ):
            raise PrepareError("DISCOVERY_FILE_ARTIFACT_INVALID", "path")
        _validate_artifact(
            root, artifact, expected_file["sha256"], expected_file["bytes"],
            expected_file["relative_path"]
        )
        selected_bindings.append({
            "relative_path": expected_file["relative_path"],
            "bytes": expected_file["bytes"],
            "sha256": expected_file["sha256"],
            "artifact": dict(artifact),
        })
    selected_paths = {
        item["relative_path"] for item in selected_bindings
    }
    if not SOURCE_RECORD_PATHS.issubset(selected_paths):
        raise PrepareError("DISCOVERY_SOURCE_FILES_MISSING", "source graph")
    required_artifacts: dict[str, Mapping[str, Any]] = {
        archive_descriptor["path"]: archive_descriptor
    }
    for package in packages:
        artifact = package["deb_artifact"]
        required_artifacts[artifact["path"]] = artifact
    for selected_file in selected:
        artifact = selected_file["artifact"]
        required_artifacts[artifact["path"]] = artifact
    _validate_receipt_artifact_index(root, receipt, required_artifacts)
    receipt_payload, receipt_descriptor = _read_sealed(
        root / DISCOVERY_RECEIPT_NAME, MAX_RECEIPT_BYTES, "discovery receipt"
    )
    if receipt_descriptor["sha256"] != EXPECTED_RECEIPT_SHA256:
        raise PrepareError("DISCOVERY_RECEIPT_SHA_INVALID", "receipt bytes")
    try:
        if json.loads(receipt_payload.decode("utf-8", "strict")) != dict(receipt):
            raise PrepareError("DISCOVERY_RECEIPT_CHANGED", "receipt reread")
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PrepareError("DISCOVERY_RECEIPT_INVALID", "receipt reread") from exc
    # ``receipt_descriptor`` is obtained from ``root`` (the readable
    # transport tree).  Keep that descriptor intact as the transport proof,
    # while rebasing only the path-bearing fields of the producer/logical
    # descriptor.  Comparing a container path with the host logical path
    # would incorrectly reject an otherwise identical bind mount.
    logical_receipt_descriptor = dict(receipt_descriptor)
    logical_receipt_descriptor["path"] = str(
        logical_root / DISCOVERY_RECEIPT_NAME
    )
    if isinstance(logical_receipt_descriptor.get("sidecar"), Mapping):
        logical_receipt_descriptor["sidecar"] = dict(
            logical_receipt_descriptor["sidecar"]
        )
        logical_receipt_descriptor["sidecar"]["path"] = (
            str(logical_root / DISCOVERY_RECEIPT_NAME) + ".sha256"
        )
    transport_receipt_descriptor = dict(receipt_descriptor)
    transport_receipt_descriptor["path"] = str(
        DISCOVERY_TRANSPORT_ROOT / DISCOVERY_RECEIPT_NAME
    )
    if isinstance(transport_receipt_descriptor.get("sidecar"), Mapping):
        transport_receipt_descriptor["sidecar"] = dict(
            transport_receipt_descriptor["sidecar"]
        )
        transport_receipt_descriptor["sidecar"]["path"] = (
            str(DISCOVERY_TRANSPORT_ROOT / DISCOVERY_RECEIPT_NAME) + ".sha256"
        )
    return {
        "discovery_id": DISCOVERY_ID,
        "root": str(logical_root),
        "receipt": {
            "path": str(logical_root / DISCOVERY_RECEIPT_NAME),
            "sha256": EXPECTED_RECEIPT_SHA256,
            "canonical_sha256": EXPECTED_DISCOVERY_CANONICAL_SHA256,
            "descriptor": logical_receipt_descriptor,
            "transport_descriptor": transport_receipt_descriptor,
        },
        "profile": {
            "path": logical_profile_path,
            "sha256": DISCOVERY_PROFILE_SHA256,
        },
        "image": {
            "digest": EXPECTED_IMAGE_DIGEST,
            "platform": EXPECTED_IMAGE_PLATFORM,
            "repo_digests": ["ros@" + EXPECTED_IMAGE_DIGEST],
        },
        "rosdistro": {
            "repository": "https://github.com/ros/rosdistro.git",
            "commit": EXPECTED_COMMIT,
            "archive_url": expected_archive_url,
            "archive_bytes": EXPECTED_ARCHIVE_BYTES,
            "archive_sha256": EXPECTED_ARCHIVE_SHA256,
            "archive_artifact": dict(archive_descriptor),
        },
        "packages": package_bindings,
        "selected_files": selected_bindings,
        "source_records": [dict(item) for item in EXPECTED_SOURCE_RECORDS],
        "receipt_bytes_sha256": _sha256(receipt_payload),
    }


def _discovery_transport_projection(
    binding: Mapping[str, Any],
    transport_root: Path,
    *,
    logical_root: Path,
    actual_root: Path | None = None,
) -> dict[str, Any]:
    """Bind every discovery artifact to the fixed container transport path.

    Discovery descriptors retain relative host paths.  This projection records
    the safe, container-visible path for each one and reopens the corresponding
    bytes through ``transport_root`` before the prepare receipt is sealed.
    """
    transport_root = _check_absolute(str(transport_root), "discovery transport root")
    logical_root = _check_absolute(str(logical_root), "discovery logical root")
    if actual_root is None:
        actual_root = transport_root
    else:
        actual_root = _check_absolute(str(actual_root), "discovery actual root")
    if logical_root != PINNED_DISCOVERY_ROOT:
        raise PrepareError("DISCOVERY_ROOT_NOT_PINNED", str(logical_root))
    if transport_root != DISCOVERY_TRANSPORT_ROOT and transport_root != logical_root:
        raise PrepareError("DISCOVERY_TRANSPORT_ROOT_INVALID", str(transport_root))
    mount = {
        "source": str(logical_root),
        "target": str(transport_root),
        "read_only": True,
        "type": "bind",
        "noexec": False,
    }
    artifacts: list[dict[str, Any]] = []

    def add(role: str, index: int, descriptor: Mapping[str, Any]) -> None:
        relative = _safe_relative(descriptor.get("path"), role)
        # ``_validate_artifact`` checks sidecar, mode, nlink, bytes and content
        # using the transport namespace.  No path from the receipt is allowed
        # to escape the transport root.
        _validate_artifact(
            actual_root, descriptor, descriptor.get("sha256"),
            descriptor.get("bytes"), role,
        )
        actual_target = actual_root / relative
        target = transport_root / relative
        if (target.as_posix() != str(transport_root) + "/" + relative or
                actual_target.as_posix() != str(actual_root) + "/" + relative):
            raise PrepareError("DISCOVERY_TRANSPORT_PATH_INVALID", role)
        # Reopen the bind-visible object a second time and retain the actual
        # descriptor (including the sidecar descriptor).  The source receipt
        # descriptor is the producer claim; this descriptor proves that the
        # exact bytes/metadata visible at the container transport path are
        # the same object before any package command consumes them.
        _, actual_descriptor = _read_sealed(
            actual_target, int(descriptor["bytes"]),
            role + " transport artifact"
        )
        transport_descriptor = dict(actual_descriptor)
        transport_descriptor["path"] = str(target)
        if isinstance(transport_descriptor.get("sidecar"), Mapping):
            transport_descriptor["sidecar"] = dict(
                transport_descriptor["sidecar"]
            )
            transport_descriptor["sidecar"]["path"] = str(target) + ".sha256"
        artifacts.append({
            "role": role,
            "index": index,
            "relative_path": relative,
            "transport_path": str(target),
            "descriptor": dict(descriptor),
            "transport_descriptor": transport_descriptor,
        })

    rosdistro = binding.get("rosdistro")
    archive = rosdistro.get("archive_artifact") if isinstance(rosdistro, Mapping) else None
    if not isinstance(archive, Mapping):
        raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_INVALID", "archive")
    add("archive", 0, archive)
    packages = binding.get("packages")
    if not isinstance(packages, list):
        raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_INVALID", "packages")
    for index, package in enumerate(packages):
        artifact = package.get("artifact") if isinstance(package, Mapping) else None
        if not isinstance(artifact, Mapping):
            raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_INVALID", "package")
        add("package", index, artifact)
    selected = binding.get("selected_files")
    if not isinstance(selected, list):
        raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_INVALID", "selected files")
    for index, selected_file in enumerate(selected):
        artifact = selected_file.get("artifact") if isinstance(selected_file, Mapping) else None
        if not isinstance(artifact, Mapping):
            raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_INVALID", "selected file")
        add("selected_file", index, artifact)
    if len({item["transport_path"] for item in artifacts}) != len(artifacts):
        raise PrepareError("DISCOVERY_TRANSPORT_ARTIFACT_DUPLICATE", "transport path")
    projection: dict[str, Any] = {
        "schema": "registration-plugin-discovery-transport-binding-v1",
        "schema_version": 1,
        "logical_root": str(logical_root),
        "transport_root": str(transport_root),
        "mount": mount,
        "artifact_count": len(artifacts),
        "artifacts": artifacts,
    }
    projection["identity_sha256"] = hashlib.sha256(
        canonical_bytes(projection)
    ).hexdigest()
    return projection


def validate_discovery_receipt(
    root: Path = DISCOVERY_ROOT,
    *,
    profile_path: Path = PROFILE_PATH,
    logical_root: Path | None = None,
) -> dict[str, Any]:
    """Validate discovery-11 through a host or fixed transport root.

    The producer's path is always ``PINNED_DISCOVERY_ROOT``.  When this helper
    runs inside the capture container, ``root`` is the read-only
    ``DISCOVERY_TRANSPORT_ROOT`` bind and ``logical_root`` remains the host
    path.  The discovery validator receives both namespaces so artifact bytes
    are reopened from the bind while path/id claims remain host-bound.
    """
    root = _check_absolute(str(root), "discovery root")
    if logical_root is None:
        logical_root = PINNED_DISCOVERY_ROOT
    else:
        logical_root = _check_absolute(str(logical_root), "discovery logical root")
    if logical_root != PINNED_DISCOVERY_ROOT:
        raise PrepareError("DISCOVERY_ROOT_NOT_PINNED", str(logical_root))
    if root != logical_root and root != DISCOVERY_TRANSPORT_ROOT:
        # Preserve the historical diagnostic for an arbitrary caller root;
        # only the fixed transport path is a legal alternate namespace.
        raise PrepareError("DISCOVERY_ROOT_NOT_PINNED", str(root))
    if profile_path != PROFILE_PATH:
        raise PrepareError("PROFILE_NOT_PINNED", str(profile_path))
    logical_profile_path = _logical_consumer_profile_path(profile_path)
    module = _discovery_module(logical_profile_path)
    try:
        if root == logical_root:
            receipt = module.validate_receipt(root)
        else:
            receipt = module.validate_receipt(root, logical_root=logical_root)
    except Exception as exc:
        raise PrepareError(
            getattr(exc, "kind", "DISCOVERY_RECEIPT_INVALID"), str(exc)
        ) from exc
    binding = _binding_projection(
        receipt, root, logical_root=logical_root,
        logical_profile_path=logical_profile_path
    )
    _consumer_profile_binding(profile_path, binding)
    return binding


def _root_descriptor(path: Path) -> dict[str, Any]:
    """Read an already-created host output root without repairing it."""
    path = _check_absolute(str(path), "prepare root")
    _check_parents(path, "prepare root")
    try:
        descriptor = os.open(
            str(path), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        info = os.fstat(descriptor)
        os.fsync(descriptor)
    except OSError as exc:
        raise PrepareError("PREPARE_ROOT_INVALID", str(path)) from exc
    finally:
        try:
            os.close(descriptor)
        except (UnboundLocalError, OSError):
            pass
    if (
        not stat.S_ISDIR(info.st_mode)
        or stat.S_IMODE(info.st_mode) != 0o700
        or info.st_nlink < 2
    ):
        raise PrepareError("PREPARE_ROOT_INVALID", str(path))
    return {
        "path": str(path),
        "type": "directory",
        "mode": 0o700,
        "uid": info.st_uid,
        "gid": info.st_gid,
        "nlink": info.st_nlink,
        "device": info.st_dev,
        "inode": info.st_ino,
    }


def _fresh_root(path: Path) -> dict[str, Any]:
    path = _check_absolute(str(path), "prepare root")
    _check_parents(path, "prepare root")
    try:
        os.mkdir(str(path), 0o700)
    except FileExistsError as exc:
        raise PrepareError("PREPARE_ROOT_COLLISION", str(path)) from exc
    except OSError as exc:
        raise PrepareError("PREPARE_ROOT_CREATE_FAILED", str(path)) from exc
    try:
        descriptor = os.open(
            str(path), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        info = os.fstat(descriptor)
        os.fsync(descriptor)
    except OSError as exc:
        raise PrepareError("PREPARE_ROOT_INVALID", str(path)) from exc
    finally:
        try:
            os.close(descriptor)
        except (UnboundLocalError, OSError):
            pass
    if not stat.S_ISDIR(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o700:
        raise PrepareError("PREPARE_ROOT_INVALID", str(path))
    return {
        "path": str(path),
        "type": "directory",
        "mode": 0o700,
        "uid": info.st_uid,
        "gid": info.st_gid,
        "nlink": info.st_nlink,
        "device": info.st_dev,
        "inode": info.st_ino,
    }


def _mkdir(path: Path) -> None:
    _check_parents(path, "prepare directory")
    try:
        os.mkdir(str(path), 0o700)
    except FileExistsError:
        try:
            info = path.lstat()
        except OSError as exc:
            raise PrepareError("PREPARE_DIRECTORY_INVALID", str(path)) from exc
        if (
            not stat.S_ISDIR(info.st_mode)
            or stat.S_IMODE(info.st_mode) != 0o700
            or info.st_nlink < 2
        ):
            raise PrepareError("PREPARE_DIRECTORY_INVALID", str(path))
    except OSError as exc:
        raise PrepareError("PREPARE_DIRECTORY_CREATE_FAILED", str(path)) from exc


def _directory_descriptor(path: Path, relative: str, owner: tuple[int, int]) -> dict[str, int | str]:
    """Read one expected directory without creating or following links."""
    _check_parents(path, "precreated directory")
    try:
        descriptor = os.open(
            str(path), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        info = os.fstat(descriptor)
    except OSError as exc:
        raise PrepareError("PREPARE_DIRECTORY_MISSING", relative) from exc
    finally:
        try:
            os.close(descriptor)
        except (UnboundLocalError, OSError):
            pass
    if (
        not stat.S_ISDIR(info.st_mode)
        or stat.S_IMODE(info.st_mode) != 0o700
        or (info.st_uid, info.st_gid) != tuple(owner)
        or info.st_nlink < 2
    ):
        raise PrepareError("PREPARE_DIRECTORY_METADATA_INVALID", relative)
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


def _precreated_directory_snapshot(root: Path, root_identity: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Require the complete host-owned tree before any prepare command.

    The production helper receives a bind-mounted output root.  Its directory
    set is therefore an input contract: missing directories are not repaired,
    and any runtime-created directory (including a nested Docker mount target)
    is rejected before the phase can claim success.
    """
    owner = (int(root_identity["uid"]), int(root_identity["gid"]))
    expected_dirs = set(PREPARE_PRECREATED_DIRECTORY_RELATIVES)
    expected_input_files = {
        "input/" + name
        for name in PREPARE_INPUT_FILE_NAMES
    }
    expected_input_files.update(
        path + ".sha256" for path in tuple(expected_input_files)
    )
    observed_dirs: set[str] = set()
    observed_input_files: set[str] = set()
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as exc:
            raise PrepareError("PREPARE_DIRECTORY_READ_FAILED", str(current)) from exc
        for entry in entries:
            info = entry.stat(follow_symlinks=False)
            relative = Path(entry.path).relative_to(root).as_posix()
            if stat.S_ISLNK(info.st_mode):
                raise PrepareError("PREPARE_DIRECTORY_LINK", relative)
            if stat.S_ISDIR(info.st_mode):
                if relative not in expected_dirs:
                    raise PrepareError("PREPARE_DIRECTORY_UNEXPECTED", relative)
                _directory_descriptor(Path(entry.path), relative, owner)
                observed_dirs.add(relative)
                pending.append(Path(entry.path))
                continue
            if stat.S_ISREG(info.st_mode):
                if relative.startswith("input/"):
                    if relative not in expected_input_files:
                        raise PrepareError("PREPARE_INPUT_UNEXPECTED", relative)
                    if (
                        stat.S_IMODE(info.st_mode) != 0o444
                        or info.st_nlink != 1
                        or (info.st_uid, info.st_gid) != owner
                    ):
                        raise PrepareError("PREPARE_INPUT_METADATA_INVALID", relative)
                    observed_input_files.add(relative)
                else:
                    # Existing bind directories may contain sealed/cache
                    # files produced by the phase.  Files are not directory
                    # children and are validated by their artifact/cache
                    # contracts; they must nevertheless be regular,
                    # single-link objects so a hardlink or replacement can
                    # never be hidden by the directory snapshot.
                    if info.st_nlink != 1 or stat.S_IMODE(info.st_mode) & 0o002:
                        raise PrepareError("PREPARE_DIRECTORY_FILE_INVALID", relative)
                continue
            raise PrepareError("PREPARE_DIRECTORY_SPECIAL_FILE", relative)
    if observed_dirs != expected_dirs:
        raise PrepareError("PREPARE_DIRECTORY_MISSING", "precreated tree differs")
    if observed_input_files != expected_input_files:
        raise PrepareError("PREPARE_INPUT_MISSING", "precreated input differs")
    return [
        _directory_descriptor(root / relative, relative, owner)
        for relative in sorted(expected_dirs, key=lambda item: (item.count("/"), item))
    ]


def _verify_precreated_directory_snapshot(
    root: Path, snapshot: list[Mapping[str, Any]], root_identity: Mapping[str, Any]
) -> None:
    """Reopen every precreated directory and require exact identity stability."""
    expected = [dict(item) for item in snapshot]
    if (
        len(expected) != len(PREPARE_PRECREATED_DIRECTORY_RELATIVES)
        or {item.get("path") for item in expected}
        != set(PREPARE_PRECREATED_DIRECTORY_RELATIVES)
    ):
        raise PrepareError("PREPARE_DIRECTORY_SNAPSHOT_INVALID", "path set")
    actual = _precreated_directory_snapshot(root, root_identity)
    if actual != expected:
        raise PrepareError("PREPARE_DIRECTORY_IDENTITY_DRIFT", "precreated tree")


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        count = os.write(descriptor, payload[offset:])
        if count <= 0:
            raise PrepareError("WRITE_NO_PROGRESS", "sealed output")
        offset += count


def _seal(root: Path, relative: str, payload: bytes) -> dict[str, Any]:
    """Seal a fresh output file and exact two-token sidecar."""
    relative = _safe_relative(relative, "output artifact")
    if len(payload) > MAX_OUTPUT_BYTES:
        raise PrepareError("OUTPUT_OVERSIZE", relative)
    path = root / relative
    _check_parents(path, "output artifact parent")
    sidecar = Path(str(path) + ".sha256")
    digest = _sha256(payload)
    created: list[tuple[Path, int, int]] = []
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
    try:
        descriptor = os.open(str(path), flags, 0o600)
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
                raise PrepareError("OUTPUT_IDENTITY_CHANGED", relative)
        finally:
            os.close(descriptor)
        parent_descriptor = os.open(
            str(path.parent), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        try:
            os.fsync(parent_descriptor)
        finally:
            os.close(parent_descriptor)
        side_payload = (digest + " " + path.name + "\n").encode("ascii")
        side_descriptor = os.open(str(sidecar), flags, 0o600)
        side_info = os.fstat(side_descriptor)
        created.append((sidecar, side_info.st_dev, side_info.st_ino))
        try:
            _write_all(side_descriptor, side_payload)
            os.fchmod(side_descriptor, 0o444)
            os.fsync(side_descriptor)
            side_after = os.fstat(side_descriptor)
            if (
                not stat.S_ISREG(side_after.st_mode)
                or side_after.st_nlink != 1
                or side_after.st_size != len(side_payload)
                or stat.S_IMODE(side_after.st_mode) != 0o444
                or (side_after.st_dev, side_after.st_ino)
                != (side_info.st_dev, side_info.st_ino)
            ):
                raise PrepareError("SIDECAR_IDENTITY_CHANGED", relative)
        finally:
            os.close(side_descriptor)
        parent_descriptor = os.open(
            str(path.parent), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        try:
            os.fsync(parent_descriptor)
        finally:
            os.close(parent_descriptor)
        descriptor_value = {
            "path": relative,
            "sidecar": relative + ".sha256",
            "bytes": len(payload),
            "sha256": digest,
            "mode": 0o444,
            "nlink": 1,
        }
        _read_sealed(path, len(payload), relative)
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


def _source_bindings(binding: Mapping[str, Any]) -> tuple[str, list[dict[str, Any]]]:
    """Map the pinned source graph to local file URIs in fixed order."""
    selected = {
        item["relative_path"]: item for item in binding["selected_files"]
    }
    lines: list[str] = []
    projections: list[dict[str, Any]] = []
    for record in binding["source_records"]:
        relative = _safe_relative(record["relative_path"], "source record")
        selected_item = selected.get(relative)
        if selected_item is None or relative not in SOURCE_RECORD_PATHS:
            raise PrepareError("SOURCE_BINDING_INVALID", relative)
        if record["type"] not in ENABLED_SOURCE_TYPES:
            # The pinned Fuerte gbpdistro is retained as discovery evidence,
            # but its local YAML still references remote platform files.  It
            # is irrelevant to the current Humble rules and is never enabled
            # in the disconnected source list.
            continue
        local_path = Path(EXPECTED_LOCAL_SOURCE_ROOT) / relative
        if local_path.as_posix() != EXPECTED_LOCAL_SOURCE_ROOT + "/" + relative:
            raise PrepareError("SOURCE_PATH_INVALID", relative)
        uri = local_path.as_uri()
        if not uri.startswith("file:///") or ".." in uri.split("/"):
            raise PrepareError("SOURCE_URI_INVALID", relative)
        suffix = ""
        if "distribution" in record:
            suffix = " " + record["distribution"]
        lines.append("{} {}{}".format(record["type"], uri, suffix))
        projections.append({
            "relative_path": relative,
            "type": record["type"],
            "line": record["line"],
            "local_uri": uri,
            "bytes": selected_item["bytes"],
            "sha256": selected_item["sha256"],
            "artifact": dict(selected_item["artifact"]),
        })
    return "\n".join(lines) + "\n", projections


def _distribution_bytes(
    binding: Mapping[str, Any], discovery_root: Path, distro: str
) -> bytes:
    """Reopen the pinned commit archive and select one exact regular member."""
    if distro not in SUPPORTED_DISTROS:
        raise PrepareError("DISTRO_INVALID", distro)
    spec = OFFLINE_DISTRIBUTIONS[distro]
    try:
        archive = binding["rosdistro"]["archive_artifact"]
        relative = _safe_relative(archive["path"], "rosdistro archive")
    except (KeyError, TypeError) as exc:
        raise PrepareError(
            "OFFLINE_DISTRIBUTION_ARCHIVE_INVALID", "archive binding"
        ) from exc
    archive_payload, descriptor = _read_sealed(
        discovery_root / relative, EXPECTED_ARCHIVE_BYTES, "rosdistro archive"
    )
    if (
        descriptor["bytes"] != EXPECTED_ARCHIVE_BYTES
        or descriptor["sha256"] != EXPECTED_ARCHIVE_SHA256
    ):
        raise PrepareError(
            "OFFLINE_DISTRIBUTION_ARCHIVE_INVALID", "archive identity"
        )
    try:
        with tarfile.open(fileobj=io.BytesIO(archive_payload), mode="r:gz") as archive_file:
            matches = [
                member for member in archive_file.getmembers()
                if member.name == spec["member"]
            ]
            if len(matches) != 1:
                raise PrepareError(
                    "OFFLINE_DISTRIBUTION_MEMBER_INVALID", "member count"
                )
            member = matches[0]
            if not member.isreg() or member.size != spec["bytes"]:
                raise PrepareError(
                    "OFFLINE_DISTRIBUTION_MEMBER_INVALID", "member metadata"
                )
            extracted = archive_file.extractfile(member)
            if extracted is None:
                raise PrepareError(
                    "OFFLINE_DISTRIBUTION_MEMBER_INVALID", "member stream"
                )
            payload = extracted.read(spec["bytes"] + 1)
    except PrepareError:
        raise
    except (OSError, tarfile.TarError) as exc:
        raise PrepareError(
            "OFFLINE_DISTRIBUTION_ARCHIVE_INVALID", "archive decode"
        ) from exc
    if (
        len(payload) != spec["bytes"]
        or _sha256(payload) != spec["sha256"]
    ):
        raise PrepareError(
            "OFFLINE_DISTRIBUTION_MEMBER_INVALID", "member identity"
        )
    return payload


def _humble_distribution_bytes(
    binding: Mapping[str, Any], discovery_root: Path
) -> bytes:
    """Compatibility wrapper for the original Humble-only helper."""
    return _distribution_bytes(binding, discovery_root, "humble")


def _normalize_runner_result(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise PrepareError("RUNNER_RESULT_INVALID", label)
    if set(value) != {"returncode", "stdout", "stderr", "network_attempts"}:
        raise PrepareError("RUNNER_RESULT_INVALID", label)
    returncode = value.get("returncode")
    stdout = value.get("stdout")
    stderr = value.get("stderr")
    attempts = value.get("network_attempts")
    if (
        type(returncode) is not int
        or not isinstance(stdout, (bytes, str))
        or not isinstance(stderr, (bytes, str))
        or attempts != []
    ):
        raise PrepareError("RUNNER_RESULT_INVALID", label)
    if isinstance(stdout, str):
        stdout = stdout.encode("utf-8")
    if isinstance(stderr, str):
        stderr = stderr.encode("utf-8")
    if len(stdout) > MAX_OUTPUT_BYTES or len(stderr) > MAX_OUTPUT_BYTES:
        raise PrepareError("COMMAND_OUTPUT_OVERSIZE", label)
    return {
        "returncode": returncode,
        "stdout": stdout,
        "stderr": stderr,
        "network_attempts": [],
    }


def _production_runner(
    argv: list[str], env: Mapping[str, str], timeout: int, network: str
) -> dict[str, Any]:
    """Run one fixed offline command with no shell or inherited environment."""
    if network != "none":
        raise PrepareError("COMMAND_POLICY_INVALID", "network must be none")
    _validate_offline_command(argv, env, "production runner")
    try:
        result = subprocess.run(
            list(argv), shell=False, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout, env=dict(env), close_fds=True,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout if isinstance(exc.stdout, bytes) else b""
        stderr = exc.stderr if isinstance(exc.stderr, bytes) else b""
        return {
            "returncode": 124, "stdout": stdout[:MAX_OUTPUT_BYTES],
            "stderr": stderr[:MAX_OUTPUT_BYTES], "network_attempts": [],
        }
    except OSError as exc:
        raise PrepareError("COMMAND_EXEC_FAILED", str(exc)) from exc
    return {
        "returncode": int(result.returncode),
        "stdout": bytes(result.stdout), "stderr": bytes(result.stderr),
        "network_attempts": [],
    }


def _run(
    root: Path,
    runner: Callable[..., Any],
    phase: dict[str, Any],
    label: str,
    argv: list[str],
    env: Mapping[str, str],
    command_index: int,
) -> dict[str, Any]:
    if not argv or any(not isinstance(item, str) or not item for item in argv):
        raise PrepareError("COMMAND_INVALID", label)
    _validate_offline_command(argv, env, label)
    try:
        raw = runner(argv, dict(env), COMMAND_TIMEOUT_SECONDS, "none")
    except Exception as exc:
        raise PrepareError("RUNNER_FAILED", label) from exc
    result = _normalize_runner_result(raw, label)
    stdout_for_url_check = result["stdout"]
    stderr_for_url_check = result["stderr"]
    if (
        label in {"rosdep_version", "rosdep_update", "rosdep_resolve"}
        and stderr_for_url_check.startswith(
            ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
        )
    ):
        # Ubuntu 24.04's Python emits this fixed local import warning before
        # every rosdep subcommand.  Its documentation URL is not a network
        # attempt; remove one byte-exact prefix, scan any remaining output,
        # and still seal the original stderr below.
        stderr_for_url_check = stderr_for_url_check[
            len(ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING):
        ]
    if any(token in stdout_for_url_check.lower() for token in (b"http://", b"https://")):
        raise PrepareError("NETWORK_FALLBACK_OBSERVED", label)
    if any(token in stderr_for_url_check.lower() for token in (b"http://", b"https://")):
        raise PrepareError("NETWORK_FALLBACK_OBSERVED", label)
    prefix = "commands/{:03d}-{}".format(command_index, label)
    stdout_descriptor = _seal(root, prefix + ".stdout", result["stdout"])
    stderr_descriptor = _seal(root, prefix + ".stderr", result["stderr"])
    command = {
        "label": label,
        "argv": list(argv),
        "env": dict(env),
        "network": "none",
        "timeout_seconds": COMMAND_TIMEOUT_SECONDS,
        "returncode": result["returncode"],
        "stdout": stdout_descriptor,
        "stderr": stderr_descriptor,
        "network_attempts": [],
    }
    record_descriptor = _seal(
        root,
        prefix + ".json",
        canonical_bytes(command) + b"\n",
    )
    command["record"] = record_descriptor
    phase.setdefault("commands", []).append(command)
    if result["returncode"] != 0:
        raise PrepareError(
            "COMMAND_FAILED", "{} exited {}".format(label, result["returncode"])
        )
    return command


def _command_output(root: Path, command: Mapping[str, Any], key: str) -> bytes:
    descriptor = command.get(key)
    if not isinstance(descriptor, Mapping):
        raise PrepareError("COMMAND_OUTPUT_INVALID", key)
    relative = _safe_relative(descriptor.get("path"), key)
    payload, actual = _read_sealed(
        root / relative, MAX_OUTPUT_BYTES, "command output"
    )
    if (
        actual["sha256"] != descriptor.get("sha256")
        or actual["bytes"] != descriptor.get("bytes")
    ):
        raise PrepareError("COMMAND_OUTPUT_CHANGED", key)
    return payload


def _parse_versions(stdout: bytes, binding: Mapping[str, Any]) -> list[dict[str, str]]:
    try:
        lines = stdout.decode("utf-8", "strict").splitlines()
    except UnicodeDecodeError as exc:
        raise PrepareError("DPKG_VERSION_OUTPUT_INVALID", "utf-8") from exc
    expected = [
        (item["name"], item["version"], item["architecture"])
        for item in binding["packages"]
    ]
    if len(lines) != len(expected):
        raise PrepareError("DPKG_VERSION_BINDING_INVALID", "row count")
    parsed: list[dict[str, str]] = []
    for line, (name, version, architecture) in zip(lines, expected):
        fields = line.split("\t")
        if len(fields) != 4 or tuple(fields[:3]) != (name, version, architecture):
            raise PrepareError("DPKG_VERSION_BINDING_INVALID", line)
        if fields[3] != "install ok installed":
            raise PrepareError("DPKG_STATUS_INVALID", line)
        parsed.append({
            "name": fields[0], "version": fields[1],
            "architecture": fields[2], "status": fields[3],
        })
    return parsed


def _parse_files(stdout: bytes, package: Mapping[str, Any]) -> list[str]:
    try:
        text = stdout.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise PrepareError("DPKG_FILES_OUTPUT_INVALID", package["name"]) from exc
    lines = text.splitlines()
    if not lines:
        raise PrepareError("DPKG_FILES_EMPTY", package["name"])
    result: list[str] = []
    for line in lines:
        # dpkg-query emits ``/.`` as the package's root-directory marker.
        # It is the one dot component that is both absolute and non-escaping;
        # normalize it to the canonical root while continuing to reject dot
        # components everywhere else.
        if line == "/.":
            line = "/"
        if (
            not line.startswith("/")
            or "\x00" in line
            or "\r" in line
            or "\n" in line
            or Path(line).as_posix() != line
            or (line != "/" and any(
                part in ("", ".", "..") for part in line.split("/")[1:]
            ))
        ):
            raise PrepareError("DPKG_FILE_PATH_INVALID", package["name"])
        result.append(line)
    if len(set(result)) != len(result):
        raise PrepareError("DPKG_FILES_DUPLICATE", package["name"])
    return result


def _parse_cache(stdout: bytes) -> list[dict[str, Any]]:
    if len(stdout) > MAX_OUTPUT_BYTES:
        raise PrepareError("ROSDEP_CACHE_OVERSIZE", "cache inventory")
    try:
        lines = stdout.decode("utf-8", "strict").splitlines()
    except UnicodeDecodeError as exc:
        raise PrepareError("ROSDEP_CACHE_INVALID", "utf-8") from exc
    if len(lines) > MAX_CACHE_ENTRIES:
        raise PrepareError("ROSDEP_CACHE_OVERSIZE", "entry count")
    entries: list[dict[str, Any]] = []
    seen: set[str] = set()
    for line in lines:
        if len(line.encode("utf-8")) > MAX_CACHE_LINE_BYTES:
            raise PrepareError("ROSDEP_CACHE_ENTRY_OVERSIZE", "line")
        fields = line.split("\t")
        if len(fields) != 3:
            raise PrepareError("ROSDEP_CACHE_INVALID", "fields")
        relative, kind, size_text = fields
        relative = _safe_relative(relative, "cache entry")
        if relative in seen or kind not in {"f", "d"}:
            raise PrepareError("ROSDEP_CACHE_INVALID", relative)
        if not re.fullmatch(r"0|[1-9][0-9]*", size_text):
            raise PrepareError("ROSDEP_CACHE_INVALID", relative)
        size = int(size_text, 10)
        if size > MAX_INPUT_BYTES:
            raise PrepareError("ROSDEP_CACHE_OVERSIZE", relative)
        seen.add(relative)
        entries.append({"path": relative, "type": kind, "bytes": size})
    return entries


def _phase_failure(phase: Mapping[str, Any], error: PrepareError) -> dict[str, Any]:
    phase["status"] = "FAILED"
    return {
        "phase": phase["phase"],
        "kind": error.kind,
        "message": str(error),
    }


def _validate_output_tree(root: Path, artifact_paths: set[str],
                          *, allow_host_bindings: bool = False) -> None:
    """Reject unrecorded output files, links, and directory replacements."""
    try:
        root_info = os.lstat(str(root))
    except OSError as exc:
        raise PrepareError("PREPARE_OUTPUT_INVALID", str(root)) from exc
    if (
        stat.S_ISLNK(root_info.st_mode)
        or not stat.S_ISDIR(root_info.st_mode)
        or stat.S_IMODE(root_info.st_mode) != 0o700
        or root_info.st_nlink < 2
    ):
        raise PrepareError("PREPARE_OUTPUT_INVALID", str(root))
    expected_dirs = set(PREPARE_OUTPUT_DIRECTORY_RELATIVES)
    if allow_host_bindings:
        expected_dirs.update(HOST_PRECREATED_RELATIVES)
    expected_files = set(artifact_paths)
    expected_files.update(path + ".sha256" for path in artifact_paths)
    expected_files.update({
        "registration_plugin_rosdep_prepare.receipt.json",
        "registration_plugin_rosdep_prepare.receipt.json.sha256",
    })
    observed_dirs: set[str] = set()
    observed_files: set[str] = set()
    observed_unbound_files: set[str] = set()
    expected_input_files = {
        "input/" + name for name in PREPARE_INPUT_FILE_NAMES
    }
    expected_input_files.update(
        path + ".sha256" for path in tuple(expected_input_files)
    )
    owner = (int(root_info.st_uid), int(root_info.st_gid))
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as exc:
            raise PrepareError("PREPARE_OUTPUT_INVALID", str(current)) from exc
        for entry in entries:
            info = entry.stat(follow_symlinks=False)
            relative = Path(entry.path).relative_to(root).as_posix()
            if stat.S_ISLNK(info.st_mode):
                raise PrepareError("PREPARE_OUTPUT_LINK", relative)
            if stat.S_ISDIR(info.st_mode):
                if relative not in expected_dirs:
                    raise PrepareError("PREPARE_HOST_BINDING_INVALID", relative)
                if stat.S_IMODE(info.st_mode) != 0o700 or info.st_nlink < 2:
                    raise PrepareError("PREPARE_OUTPUT_DIRECTORY_INVALID", relative)
                observed_dirs.add(relative)
                pending.append(Path(entry.path))
            elif stat.S_ISREG(info.st_mode):
                is_host_file = allow_host_bindings and any(
                    relative == base or relative.startswith(base + "/")
                    for base in HOST_PRECREATED_RELATIVES
                )
                if info.st_nlink != 1 or stat.S_IMODE(info.st_mode) & 0o002:
                    raise PrepareError("PREPARE_HOST_BINDING_INVALID", relative)
                if is_host_file:
                    if relative.startswith("input/"):
                        if relative not in expected_input_files:
                            raise PrepareError("PREPARE_INPUT_UNEXPECTED", relative)
                        if (stat.S_IMODE(info.st_mode) != 0o444 or
                                (info.st_uid, info.st_gid) != owner):
                            raise PrepareError(
                                "PREPARE_INPUT_METADATA_INVALID", relative)
                    observed_files.add(relative)
                    continue
                if stat.S_IMODE(info.st_mode) != 0o444:
                    raise PrepareError("PREPARE_OUTPUT_FILE_INVALID", relative)
                observed_files.add(relative)
                observed_unbound_files.add(relative)
            else:
                raise PrepareError("PREPARE_OUTPUT_SPECIAL_FILE", relative)
    if observed_dirs != expected_dirs or observed_unbound_files != expected_files:
        raise PrepareError(
            "PREPARE_OUTPUT_ALLOWLIST_INVALID",
            "expected dirs/files differ from observed",
        )


def _validate_command_record(
    root: Path,
    command: Mapping[str, Any],
    artifact_index: Mapping[str, Mapping[str, Any]],
) -> None:
    expected_fields = {
        "label", "argv", "env", "network", "timeout_seconds", "returncode",
        "stdout", "stderr", "network_attempts", "record",
    }
    if set(command) != expected_fields:
        raise PrepareError("PREPARE_COMMAND_INVALID", "fields")
    label = command.get("label")
    argv = command.get("argv")
    env = command.get("env")
    if (
        not isinstance(label, str)
        or not isinstance(argv, list)
        or not isinstance(env, Mapping)
        or command.get("network") != "none"
        or command.get("timeout_seconds") != COMMAND_TIMEOUT_SECONDS
        or type(command.get("returncode")) is not int
        or command.get("network_attempts") != []
    ):
        raise PrepareError("PREPARE_COMMAND_INVALID", str(label))
    _validate_offline_command(argv, env, label)
    for key in ("stdout", "stderr", "record"):
        descriptor = command.get(key)
        if not isinstance(descriptor, Mapping):
            raise PrepareError("PREPARE_COMMAND_INVALID", label)
        path = descriptor.get("path")
        if not isinstance(path, str) or artifact_index.get(path) != descriptor:
            raise PrepareError("PREPARE_COMMAND_ARTIFACT_INVALID", label)
    record_path = root / command["record"]["path"]
    payload, _ = _read_sealed(record_path, MAX_OUTPUT_BYTES, "command record")
    try:
        recorded = json.loads(payload.decode("utf-8", "strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PrepareError("PREPARE_COMMAND_RECORD_INVALID", label) from exc
    expected = dict(command)
    expected.pop("record")
    if recorded != expected:
        raise PrepareError("PREPARE_COMMAND_RECORD_INVALID", label)


def _validate_discovery_transport_projection(
    value: Any, binding: Any, *, actual_root: Path | None = None
) -> None:
    """Recompute and reopen the sealed discovery transport projection."""
    if not isinstance(value, Mapping) or not isinstance(binding, Mapping):
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "fields")
    required = {
        "schema", "schema_version", "logical_root", "transport_root",
        "mount", "artifact_count", "artifacts", "identity_sha256",
    }
    if set(value) != required:
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "fields")
    logical = binding.get("root")
    transport = value.get("transport_root")
    if not isinstance(logical, str) or not isinstance(transport, str):
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "roots")
    expected = _discovery_transport_projection(
        binding, Path(transport), logical_root=Path(logical),
        actual_root=actual_root)
    if dict(value) != expected:
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "projection")


def _validate_prepare_shape(value: Mapping[str, Any], root: Path,
                            *, allow_host_bindings: bool = False,
                            logical_root: Path | None = None,
                            transport_root: Path | None = None,
                            discovery_root: Path | None = None,
                            reopen_input_binding: bool = True) -> None:
    root = _check_absolute(str(root), "prepare transport root")
    if logical_root is None:
        logical_root = root
    else:
        logical_root = _check_absolute(str(logical_root), "prepare logical root")
    if transport_root is None:
        transport_root = root
    else:
        transport_root = _check_absolute(
            str(transport_root), "prepare transport root")
    required = {
        "schema", "schema_version", "status", "outcome", "benchmark_eligible",
        "promotion", "active_profile_switch", "claim_eligible", "discovery",
        "discovery_profile_sha256", "consumer_profile_sha256", "consumer_profile",
        "execution", "phase_order", "phases", "artifacts", "summary",
        "failure", "root", "transport_root", "root_mount",
        "discovery_transport", "input_binding", "started_at_unix",
        "ended_at_unix", "canonical_sha256",
    }
    if set(value) != required:
        raise PrepareError("PREPARE_RECEIPT_FIELDS_INVALID", "receipt fields")
    if (
        value["schema"] != PREPARE_SCHEMA
        or value["schema_version"] != SCHEMA_VERSION
        or value["benchmark_eligible"] is not False
        or value["promotion"] != "FORBIDDEN_UNTIL_SIGNED_REVIEW"
        or value["active_profile_switch"] is not False
        or value["claim_eligible"] is not False
        or value["phase_order"] != ["dependency_install", "rosdep_prepare"]
        or value["canonical_sha256"] != canonical_hash(value)
    ):
        raise PrepareError("PREPARE_RECEIPT_BINDING_INVALID", "receipt contract")
    execution = value["execution"]
    if (
        not isinstance(execution, Mapping)
        or set(execution) != {
            "image_digest", "platform", "pull", "build", "network",
            "runner_injected", "external_execution"
        }
        or execution["image_digest"] != EXPECTED_IMAGE_DIGEST
        or execution["platform"] != EXPECTED_IMAGE_PLATFORM
        or execution["pull"] is not False
        or execution["build"] is not False
        or execution["network"] != "none"
        or type(execution["runner_injected"]) is not bool
        or type(execution["external_execution"]) is not bool
        or execution["runner_injected"] == execution["external_execution"]
    ):
        raise PrepareError("PREPARE_EXECUTION_INVALID", "runner policy")
    consumer_profile = value["consumer_profile"]
    logical_profile_path = _logical_consumer_profile_path(PROFILE_PATH)
    if (
        value["discovery_profile_sha256"] != DISCOVERY_PROFILE_SHA256
        or not isinstance(value["consumer_profile_sha256"], str)
        or SAFE_SHA.fullmatch(value["consumer_profile_sha256"]) is None
        or not isinstance(consumer_profile, Mapping)
        or set(consumer_profile) != {"path", "sha256"}
        or consumer_profile["path"] != logical_profile_path
        or consumer_profile["sha256"] != value["consumer_profile_sha256"]
    ):
        raise PrepareError("PREPARE_PROFILE_IDENTITY_INVALID", "profile identities")
    _validate_prepare_input_binding(
        value.get("input_binding"), reopen=reopen_input_binding
    )
    root_value = value["root"]
    if (
        not isinstance(root_value, Mapping)
        or set(root_value) != {
            "path", "type", "mode", "uid", "gid", "nlink", "device", "inode"
        }
        or root_value.get("path") != str(logical_root)
        or root_value.get("type") != "directory"
        or root_value.get("mode") != 0o700
        or type(root_value.get("uid")) is not int
        or type(root_value.get("gid")) is not int
        or type(root_value.get("nlink")) is not int
        or root_value.get("nlink") < 2
        or type(root_value.get("device")) is not int
        or type(root_value.get("inode")) is not int
    ):
        raise PrepareError("PREPARE_ROOT_BINDING_INVALID", "root")
    transport_value = value["transport_root"]
    if (
        not isinstance(transport_value, Mapping)
        or set(transport_value) != {
            "path", "type", "mode", "uid", "gid", "nlink", "device", "inode"
        }
        or transport_value.get("path") != str(transport_root)
        or transport_value.get("type") != "directory"
        or transport_value.get("mode") != 0o700
        or type(transport_value.get("uid")) is not int
        or type(transport_value.get("gid")) is not int
        or type(transport_value.get("nlink")) is not int
        or transport_value.get("nlink") < 2
        or type(transport_value.get("device")) is not int
        or type(transport_value.get("inode")) is not int
    ):
        raise PrepareError("PREPARE_ROOT_BINDING_INVALID", "transport root")
    identity_fields = ("type", "mode", "uid", "gid", "nlink", "device", "inode")
    if any(root_value.get(key) != transport_value.get(key) for key in identity_fields):
        raise PrepareError("PREPARE_ROOT_BINDING_INVALID", "logical/transport identity")
    root_mount = value["root_mount"]
    if (
        not isinstance(root_mount, Mapping)
        or set(root_mount) != {
            "source", "target", "read_only", "type", "noexec", "identity"
        }
        or root_mount.get("source") != str(logical_root)
        or root_mount.get("target") != str(transport_root)
        or root_mount.get("read_only") is not False
        or root_mount.get("type") != "bind"
        or root_mount.get("noexec") is not False
        or root_mount.get("identity") != {
            key: transport_value[key]
            for key in ("device", "inode", "uid", "gid", "mode", "nlink")
        }
    ):
        raise PrepareError("PREPARE_ROOT_BINDING_INVALID", "root mount")
    if type(value["started_at_unix"]) is not int or type(value["ended_at_unix"]) is not int:
        raise PrepareError("PREPARE_TIME_INVALID", "timestamps")
    if value["ended_at_unix"] < value["started_at_unix"]:
        raise PrepareError("PREPARE_TIME_INVALID", "ordering")
    phases = value["phases"]
    if not isinstance(phases, list) or len(phases) != 2:
        raise PrepareError("PREPARE_PHASE_INVALID", "phase count")
    if (
        [item.get("phase") for item in phases if isinstance(item, Mapping)]
        != value["phase_order"]
    ):
        raise PrepareError("PREPARE_PHASE_INVALID", "phase order")
    status = value["status"]
    failure = value["failure"]
    if status == "REVIEW_REQUIRED":
        if value["outcome"] != "PASS_REVIEW_REQUIRED" or failure is not None:
            raise PrepareError("PREPARE_STATUS_INVALID", "success shape")
        if any(item.get("status") != "PASS" for item in phases):
            raise PrepareError("PREPARE_STATUS_INVALID", "success phases")
    elif status == "PARTIAL_FAILURE_REVIEW_REQUIRED":
        if value["outcome"] != "PARTIAL_FAILURE_REVIEW_REQUIRED":
            raise PrepareError("PREPARE_STATUS_INVALID", "partial outcome")
        if not isinstance(failure, Mapping):
            raise PrepareError("PREPARE_STATUS_INVALID", "partial failure")
        if not any(item.get("status") == "FAILED" for item in phases):
            raise PrepareError("PREPARE_STATUS_INVALID", "failed phase")
    else:
        raise PrepareError("PREPARE_STATUS_INVALID", "status")
    artifact_paths: set[str] = set()
    artifact_index: dict[str, Mapping[str, Any]] = {}
    if not isinstance(value["artifacts"], list):
        raise PrepareError("PREPARE_ARTIFACTS_INVALID", "list")
    for item in value["artifacts"]:
        if not isinstance(item, Mapping):
            raise PrepareError("PREPARE_ARTIFACTS_INVALID", "descriptor")
        path = _safe_relative(item.get("path"), "receipt artifact")
        sidecar = _safe_relative(item.get("sidecar"), "receipt sidecar")
        if sidecar != path + ".sha256" or path in artifact_paths:
            raise PrepareError("PREPARE_ARTIFACTS_INVALID", path)
        if (
            set(item) != {
                "path", "sidecar", "bytes", "sha256", "mode", "nlink"
            }
            or type(item.get("bytes")) is not int
            or item.get("bytes") < 0
            or item.get("mode") != 0o444
            or item.get("nlink") != 1
            or not isinstance(item.get("sha256"), str)
            or SAFE_SHA.fullmatch(item.get("sha256")) is None
        ):
            raise PrepareError("PREPARE_ARTIFACTS_INVALID", path)
        artifact_paths.add(path)
        artifact_index[path] = item
        _, actual = _read_sealed(root / path, MAX_OUTPUT_BYTES, path)
        if (
            actual.get("bytes") != item.get("bytes")
            or actual.get("sha256") != item.get("sha256")
            or actual.get("mode") != item.get("mode")
            or actual.get("nlink") != item.get("nlink")
        ):
            raise PrepareError("PREPARE_ARTIFACTS_INVALID", path)
    _validate_output_tree(
        root, artifact_paths, allow_host_bindings=allow_host_bindings)
    _validate_discovery_transport_projection(
        value.get("discovery_transport"), value.get("discovery"),
        actual_root=discovery_root)
    for phase in phases:
        if not isinstance(phase, Mapping):
            raise PrepareError("PREPARE_PHASE_INVALID", "phase object")
        commands = phase.get("commands")
        if not isinstance(commands, list):
            raise PrepareError("PREPARE_PHASE_INVALID", phase.get("phase"))
        labels: set[str] = set()
        for command in commands:
            if not isinstance(command, Mapping):
                raise PrepareError("PREPARE_COMMAND_INVALID", "object")
            _validate_command_record(root, command, artifact_index)
            if command["label"] in labels:
                raise PrepareError("PREPARE_COMMAND_INVALID", "duplicate label")
            labels.add(command["label"])


def validate_prepare_receipt(
    root: Path,
    *,
    discovery_root: Path = DISCOVERY_ROOT,
    allow_host_bindings: bool = False,
    logical_root: Path | None = None,
    transport_root: Path | None = None,
    reopen_input_binding: bool = True,
) -> dict[str, Any]:
    """Reopen a prepare receipt and all output descriptors strictly."""
    root = _check_absolute(str(root), "prepare root")
    data, descriptor = _read_sealed(
        root / "registration_plugin_rosdep_prepare.receipt.json",
        MAX_RECEIPT_BYTES,
        "prepare receipt",
    )
    try:
        value = json.loads(data.decode("utf-8", "strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PrepareError("PREPARE_RECEIPT_INVALID", "JSON") from exc
    if not isinstance(value, dict):
        raise PrepareError("PREPARE_RECEIPT_INVALID", "object")
    if logical_root is None:
        logical_root = root
    else:
        logical_root = _check_absolute(str(logical_root), "prepare logical root")
    if transport_root is None:
        transport_root = root
    else:
        transport_root = _check_absolute(
            str(transport_root), "prepare transport root")
    discovery_root = _check_absolute(str(discovery_root), "discovery root")
    logical_discovery_root = (
        PINNED_DISCOVERY_ROOT
        if discovery_root == DISCOVERY_TRANSPORT_ROOT
        else discovery_root
    )
    _validate_prepare_shape(
        value, root, allow_host_bindings=allow_host_bindings,
        logical_root=logical_root, transport_root=transport_root,
        discovery_root=discovery_root,
        reopen_input_binding=reopen_input_binding)
    try:
        root_info = os.lstat(str(root))
    except OSError as exc:
        raise PrepareError("PREPARE_ROOT_INVALID", str(root)) from exc
    recorded_root = value["root"]
    recorded_transport = value["transport_root"]
    if (
        stat.S_ISLNK(root_info.st_mode)
        or not stat.S_ISDIR(root_info.st_mode)
        or stat.S_IMODE(root_info.st_mode) != recorded_transport["mode"]
        or root_info.st_uid != recorded_transport["uid"]
        or root_info.st_gid != recorded_transport["gid"]
        or root_info.st_dev != recorded_transport["device"]
        or root_info.st_ino != recorded_transport["inode"]
        or root_info.st_nlink < recorded_transport["nlink"]
        or any(recorded_root[key] != recorded_transport[key] for key in (
            "type", "mode", "uid", "gid", "nlink", "device", "inode"
        ))
    ):
        raise PrepareError("PREPARE_ROOT_CHANGED", "root identity")
    if descriptor["sha256"] != _sha256(data):
        raise PrepareError("PREPARE_RECEIPT_CHANGED", "receipt")
    binding = validate_discovery_receipt(
        discovery_root, logical_root=logical_discovery_root)
    if value["discovery"] != binding:
        raise PrepareError("PREPARE_DISCOVERY_BINDING_INVALID", "binding")
    transport_projection_root = value["discovery_transport"].get(
        "transport_root")
    if not isinstance(transport_projection_root, str):
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "root")
    expected_transport = _discovery_transport_projection(
        binding, Path(transport_projection_root),
        logical_root=logical_discovery_root, actual_root=discovery_root)
    if value["discovery_transport"] != expected_transport:
        raise PrepareError("DISCOVERY_TRANSPORT_BINDING_INVALID", "receipt")
    consumer_profile = _consumer_profile_binding(PROFILE_PATH, binding)
    if (value["discovery_profile_sha256"] != binding["profile"]["sha256"] or
            value["consumer_profile"] != consumer_profile or
            value["consumer_profile_sha256"] != consumer_profile["sha256"]):
        raise PrepareError("PREPARE_PROFILE_IDENTITY_INVALID", "profile binding")
    return value


def prepare_rosdep(
    output_root: Path,
    *,
    discovery_root: Path = DISCOVERY_ROOT,
    profile_path: Path = PROFILE_PATH,
    logical_output_root: Path | None = None,
    runner: Callable[..., Any] | None = None,
    clock: Callable[[], int] | None = None,
    precreated: bool = False,
    input_binding: Mapping[str, Any] | None = None,
    distro: str = "humble",
) -> dict[str, Any]:
    """Run the two offline phases through an injected runner and seal truth."""
    if runner is None:
        raise PrepareError("RUNNER_REQUIRED", "live execution is not implicit")
    if distro not in SUPPORTED_DISTROS:
        raise PrepareError("DISTRO_INVALID", distro)
    root = _check_absolute(str(output_root), "prepare transport root")
    if logical_output_root is None:
        logical_output_root = root
    else:
        logical_output_root = _check_absolute(
            str(logical_output_root), "prepare logical root")
    logical_discovery_root = (
        PINNED_DISCOVERY_ROOT
        if discovery_root == DISCOVERY_TRANSPORT_ROOT
        else discovery_root
    )
    binding = validate_discovery_receipt(
        discovery_root, profile_path=profile_path,
        logical_root=logical_discovery_root)
    consumer_profile = _consumer_profile_binding(profile_path, binding)
    discovery_transport = _discovery_transport_projection(
        binding, discovery_root, logical_root=logical_discovery_root)
    if input_binding is None:
        input_binding = _prepare_input_binding()
    else:
        # Callers may provide a previously sealed transport binding, but the
        # prepare phase still owns the final semantic/readback check below.
        _validate_prepare_input_binding(input_binding)
        input_binding = json.loads(json.dumps(input_binding))
    transport_identity = (
        _root_descriptor(root) if precreated else _fresh_root(root)
    )
    root_identity = dict(transport_identity)
    root_identity["path"] = str(logical_output_root)
    precreated_snapshot: list[dict[str, Any]] | None = None
    if precreated:
        # Every directory in the bind/output tree is an input contract.  A
        # production container must never repair a missing directory or let
        # Docker create a nested mount target.  The complete snapshot is
        # reopened after the commands and before the receipt is sealed.
        precreated_snapshot = _precreated_directory_snapshot(
            root, root_identity)
    else:
        _mkdir(root / "commands")
        _mkdir(root / "artifacts")
        _mkdir(root / "artifacts/rosdep")
        _mkdir(root / "artifacts/rosdep/sources.list.d")
        _mkdir(root / "artifacts/rosdep/cache")
    now = clock or (lambda: int(time.time()))
    started = int(now())
    phases: list[dict[str, Any]] = [
        {"phase": "dependency_install", "status": "NOT_STARTED", "commands": []},
        {"phase": "rosdep_prepare", "status": "NOT_STARTED", "commands": []},
    ]
    artifacts: list[dict[str, Any]] = []
    failure: dict[str, Any] | None = None
    summary: dict[str, Any] = {
        "dependency_install": "NOT_STARTED",
        "rosdep_prepare": "NOT_STARTED",
        "package_count": 0,
        "selected_file_count": 0,
        "network_attempts": [],
        "offline_only": True,
    }
    try:
        install_phase = phases[0]
        install_phase["status"] = "RUNNING"
        container_debs = [
            str(
                DISCOVERY_TRANSPORT_ROOT / _safe_relative(
                    item["artifact"]["path"], "package transport path"
                )
            )
            for item in binding["packages"]
        ]
        _run(
            root, runner, install_phase, "dpkg_install", [
                "dpkg", "--install", "--no-triggers", *container_debs
            ], EXPECTED_INSTALL_ENV, 1
        )
        versions = _run(
            root, runner, install_phase, "dpkg_versions", [
                "dpkg-query", "--show",
                "--showformat=${Package}\\t${Version}\\t${Architecture}\\t${Status}\\n",
                *[item["name"] for item in binding["packages"]],
            ], EXPECTED_INSTALL_ENV, 2
        )
        version_rows = _parse_versions(
            _command_output(root, versions, "stdout"), binding
        )
        file_rows: dict[str, list[str]] = {}
        for index, package in enumerate(binding["packages"], 3):
            files = _run(
                root, runner, install_phase,
                "dpkg_files_{}".format(package["name"]),
                ["dpkg-query", "--listfiles", package["name"]],
                EXPECTED_INSTALL_ENV, index,
            )
            file_rows[package["name"]] = _parse_files(
                _command_output(root, files, "stdout"), package
            )
        install_phase.update({
            "status": "PASS",
            "packages": version_rows,
            "files": file_rows,
            "offline": True,
        })
        summary["dependency_install"] = "PASS"
        summary["package_count"] = len(version_rows)
        for command in install_phase["commands"]:
            artifacts.extend([
                command["stdout"], command["stderr"], command["record"]
            ])

        prepare_phase = phases[1]
        prepare_phase["status"] = "RUNNING"
        source_text, source_bindings = _source_bindings(binding)
        source_descriptor = _seal(
            root, "artifacts/rosdep/sources.list.d/20-local.list",
            source_text.encode("utf-8")
        )
        prepare_phase["source_list"] = source_descriptor
        prepare_phase["source_bindings"] = source_bindings
        artifacts.append(source_descriptor)
        distribution_relative = _offline_distribution_relative(distro)
        distribution_descriptor = _seal(
            root, distribution_relative,
            _distribution_bytes(binding, discovery_root, distro),
        )
        prepare_phase["distribution"] = distribution_descriptor
        artifacts.append(distribution_descriptor)
        offline_index_descriptor = _seal(
            root, "artifacts/rosdep/index-v4-offline.yaml",
            _offline_index_bytes(distro),
        )
        artifacts.append(offline_index_descriptor)
        version = _run(
            root, runner, prepare_phase, "rosdep_version", [
                "rosdep", "--version"
            ], EXPECTED_NETWORK_ENV, 7
        )
        version_text = _command_output(root, version, "stdout").decode(
            "utf-8", "strict"
        ).strip()
        if version_text != "0.26.0":
            raise PrepareError("ROSDEP_VERSION_INVALID", version_text)
        update = _run(
            root, runner, prepare_phase, "rosdep_update", [
                "rosdep", "update",
            ], EXPECTED_NETWORK_ENV, 8
        )
        resolve = _run(
            root, runner, prepare_phase, "rosdep_resolve", [
                "rosdep", "resolve", "--rosdistro", distro,
                "--sources-cache-dir", EXPECTED_GENERATED_ROSDEP_CACHE,
                *EXPECTED_ROSDEP_KEYS,
            ], EXPECTED_NETWORK_ENV, 9
        )
        inventory = _run(
            root, runner, prepare_phase, "rosdep_cache_inventory", [
                "find", EXPECTED_GENERATED_ROSDEP_CACHE,
                "-xdev", "-mindepth", "1",
                "-maxdepth", "8", "-printf", "%P\\t%y\\t%s\\n",
            ], EXPECTED_NETWORK_ENV, 10
        )
        cache_entries = _parse_cache(
            _command_output(root, inventory, "stdout")
        )
        cache_descriptor = _seal(
            root, "artifacts/rosdep/cache/tree.json",
            canonical_bytes({"entries": cache_entries}) + b"\n"
        )
        prepare_phase.update({
            "status": "PASS",
            "network_policy": {
                "mode": "none",
                "attempts": [],
                "fallback": "forbidden",
            },
            "env": dict(EXPECTED_NETWORK_ENV),
            "rosdep_version": version_text,
            "source_list": source_descriptor,
            "source_bindings": source_bindings,
            "cache_tree": cache_descriptor,
            "cache_entries": cache_entries,
            "resolution": {
                "update": _command_output(
                    root, update, "stdout"
                ).decode("utf-8", "replace"),
                "resolve": _command_output(
                    root, resolve, "stdout"
                ).decode("utf-8", "replace"),
            },
        })
        artifacts.append(cache_descriptor)
        summary.update({
            "rosdep_prepare": "PASS",
            "selected_file_count": len(binding["selected_files"]),
        })
        for command in prepare_phase["commands"]:
            artifacts.extend([
                command["stdout"], command["stderr"], command["record"]
            ])
    except PrepareError as exc:
        for phase in phases:
            if phase["status"] == "RUNNING":
                failure = _phase_failure(phase, exc)
                summary[phase["phase"]] = "FAILED"
                break
        if failure is None:
            failure = {
                "phase": "prepare",
                "kind": exc.kind,
                "message": str(exc),
            }
    except Exception as exc:
        for phase in phases:
            if phase["status"] == "RUNNING":
                phase["status"] = "FAILED"
                failure = {
                    "phase": phase["phase"],
                    "kind": "UNEXPECTED_PREPARE_FAILURE",
                    "message": str(exc),
                }
                summary[phase["phase"]] = "FAILED"
                break
    finally:
        for phase in phases:
            if phase["status"] == "RUNNING":
                phase["status"] = "FAILED"
            for command in phase.get("commands", []):
                for key in ("stdout", "stderr", "record"):
                    descriptor = command.get(key)
                    if isinstance(descriptor, Mapping):
                        artifacts.append(dict(descriptor))
        unique: dict[str, dict[str, Any]] = {}
        for artifact in artifacts:
            path = artifact.get("path") if isinstance(artifact, Mapping) else None
            if isinstance(path, str):
                unique[path] = dict(artifact)
        artifacts = [unique[path] for path in sorted(unique)]
        if precreated_snapshot is not None:
            try:
                _verify_precreated_directory_snapshot(
                    root, precreated_snapshot, root_identity)
            except PrepareError as exc:
                if failure is None:
                    failure = {
                        "phase": "rosdep_prepare",
                        "kind": exc.kind,
                        "message": str(exc),
                    }
                    summary["rosdep_prepare"] = "FAILED"
                    phases[-1]["status"] = "FAILED"
    ended = int(now())
    status = "REVIEW_REQUIRED" if failure is None else "PARTIAL_FAILURE_REVIEW_REQUIRED"
    outcome = "PASS_REVIEW_REQUIRED" if failure is None else "PARTIAL_FAILURE_REVIEW_REQUIRED"
    value: dict[str, Any] = {
        "schema": PREPARE_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "outcome": outcome,
        "benchmark_eligible": False,
        "promotion": "FORBIDDEN_UNTIL_SIGNED_REVIEW",
        "active_profile_switch": False,
        "claim_eligible": False,
        "discovery": binding,
        "discovery_profile_sha256": DISCOVERY_PROFILE_SHA256,
        "consumer_profile_sha256": consumer_profile["sha256"],
        "consumer_profile": consumer_profile,
        "execution": {
            "image_digest": EXPECTED_IMAGE_DIGEST,
            "platform": EXPECTED_IMAGE_PLATFORM,
            "pull": False,
            "build": False,
            "network": "none",
            "runner_injected": runner is not _production_runner,
            "external_execution": runner is _production_runner,
        },
        "phase_order": ["dependency_install", "rosdep_prepare"],
        "phases": phases,
        "artifacts": artifacts,
        "summary": summary,
        "failure": failure,
        "root": root_identity,
        "transport_root": transport_identity,
        "root_mount": {
            "source": str(logical_output_root),
            "target": str(root),
            "read_only": False,
            "type": "bind",
            "noexec": False,
            "identity": {
                key: transport_identity[key]
                for key in ("device", "inode", "uid", "gid", "mode", "nlink")
            },
        },
        "discovery_transport": discovery_transport,
        "input_binding": input_binding,
        "started_at_unix": started,
        "ended_at_unix": ended,
    }
    value["canonical_sha256"] = canonical_hash(value)
    _seal(
        root,
        "registration_plugin_rosdep_prepare.receipt.json",
        canonical_bytes(value) + b"\n",
    )
    return validate_prepare_receipt(
        root, discovery_root=discovery_root,
        allow_host_bindings=precreated,
        logical_root=logical_output_root,
        transport_root=root)


def reject_promotion(receipt: Mapping[str, Any]) -> None:
    """Reject candidate prepare evidence as a promotion authorization."""
    if (
        receipt.get("promotion") != "FORBIDDEN_UNTIL_SIGNED_REVIEW"
        or receipt.get("benchmark_eligible") is not False
        or receipt.get("claim_eligible") is not False
        or receipt.get("active_profile_switch") is not False
    ):
        raise PrepareError("PROMOTION_POLICY_INVALID", "non-promoting shape")
    raise PrepareError("PROMOTION_REVIEW_REQUIRED", "signed external review required")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("validate-discovery")
    check.add_argument("--root", type=Path, default=DISCOVERY_ROOT)
    prepare = sub.add_parser("prepare")
    prepare.add_argument("--output-root", type=Path, required=True)
    prepare.add_argument("--logical-output-root", type=Path)
    prepare.add_argument("--discovery-root", type=Path, default=DISCOVERY_ROOT)
    prepare.add_argument("--profile", type=Path, default=PROFILE_PATH)
    prepare.add_argument("--distro", choices=SUPPORTED_DISTROS, required=True)
    prepare.add_argument("--precreated", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate-discovery":
            result = validate_discovery_receipt(args.root)
        else:
            result = prepare_rosdep(
                args.output_root,
                discovery_root=args.discovery_root,
                profile_path=args.profile,
                logical_output_root=args.logical_output_root,
                runner=_production_runner,
                precreated=args.precreated,
                distro=args.distro,
            )
        print(json.dumps(result, sort_keys=True))
        return 0
    except PrepareError as exc:
        print(json.dumps({
            "status": "FAIL_CLOSED", "kind": exc.kind, "message": str(exc)
        }, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
