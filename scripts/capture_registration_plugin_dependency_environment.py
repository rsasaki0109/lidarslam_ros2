#!/usr/bin/env python3
"""Capture the immutable base dependency environment for one release leg.

This utility runs inside the pinned ROS container immediately after the apt and
rosdep install command and before the container network is disconnected.  It
records hashes and identities only: package/source contents and environment
values that could contain credentials are never copied into the receipt.
"""

# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import argparse
import hashlib
import json
import os
import platform
import re
import stat
import subprocess
import sys
from pathlib import Path


SCHEMA = "registration-plugin-dependency-environment-v1"
SCHEMA_VERSION = 1
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
DISTROS = frozenset(("humble", "jazzy"))
ROS2_SOURCES_RELATIVE = "sources.list.d/ros2.sources"
ROS2_SOURCES_TARGET = "/usr/share/ros-apt-source/ros2.sources"
ROS2_SOURCES_TARGET_MODE = 0o644
TARGET_MAX_BYTES = 16 * 1024 * 1024


class CaptureError(RuntimeError):
    """A fail-closed dependency-environment capture error."""


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_sha(value, label):
    if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
        raise CaptureError("{} is not a lowercase SHA-256".format(label))
    return value


def _validate_image_digest(value, label):
    if not isinstance(value, str) or not IMAGE_DIGEST_RE.fullmatch(value):
        raise CaptureError("{} is not a sha256 image digest".format(label))
    return value


def _stable_relative(value, label):
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise CaptureError("{} is not a portable relative path".format(label))
    path = Path(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise CaptureError("{} is absolute or non-canonical: {}".format(label, value))
    normalized = path.as_posix()
    if normalized != value:
        raise CaptureError("{} is not normalized: {}".format(label, value))
    return value


def _regular(path, label):
    path = Path(path)
    try:
        mode = os.lstat(str(path)).st_mode
        links = os.stat(str(path), follow_symlinks=False).st_nlink
    except OSError as exc:
        raise CaptureError("{} cannot be inspected: {}".format(label, exc))
    if not stat.S_ISREG(mode) or links != 1:
        raise CaptureError("{} must be a single-link regular file: {}".format(label, path))
    return path


def _directory(path, label):
    path = Path(path)
    try:
        mode = os.lstat(str(path)).st_mode
    except OSError as exc:
        raise CaptureError("{} cannot be inspected: {}".format(label, exc))
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise CaptureError("{} must be a non-symlink directory: {}".format(label, path))
    return path


def _stat_identity(info):
    """Return metadata used to detect replacement or mutation races."""
    return (
        info.st_dev, info.st_ino, info.st_mode, info.st_uid, info.st_gid,
        info.st_nlink, info.st_size,
    )


def _parent_identity(path, label):
    """Inspect every parent without resolving or following symlinks."""
    path = Path(path)
    if not path.is_absolute():
        raise CaptureError("{} must be absolute: {}".format(label, path))
    current = path.parent
    identities = []
    while True:
        try:
            info = os.lstat(str(current))
        except OSError as exc:
            raise CaptureError("{} parent cannot be inspected: {}".format(label, exc))
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise CaptureError("{} parent must be a non-symlink directory: {}".format(
                label, current))
        identities.append((str(current), _stat_identity(info)))
        if current == Path("/"):
            break
        current = current.parent
    return tuple(identities)


def _target_expected_path(filesystem_root, apt_root):
    """Return the fixed target, with a private filesystem root for fixtures."""
    provided_root = filesystem_root is not None
    if filesystem_root is None:
        filesystem_root = Path("/")
    filesystem_root = Path(filesystem_root)
    if not filesystem_root.is_absolute():
        raise CaptureError("capture filesystem root must be absolute")
    _directory(filesystem_root, "capture filesystem root")
    if filesystem_root == Path("/"):
        if not provided_root and Path(apt_root) != Path("/etc/apt"):
            raise CaptureError("custom apt root requires an explicit filesystem root")
        return ROS2_SOURCES_TARGET
    # A synthetic root is test-only; the production default above remains the
    # exact absolute path inside the container.  Do not canonicalize it.
    if Path(apt_root) == Path("/etc/apt"):
        raise CaptureError("custom filesystem root requires a matching apt root")
    return str(filesystem_root / ROS2_SOURCES_TARGET.lstrip("/"))


def _digest_fd(fd, size, label):
    digest = hashlib.sha256()
    total = 0
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        total += len(block)
        if total > size:
            raise CaptureError("{} grew while being read".format(label))
        digest.update(block)
    if total != size:
        raise CaptureError("{} size changed while being read".format(label))
    return digest.hexdigest()


def _record_resolved_target(path, label, *, require_root_owner):
    """Read the fixed symlink target through one stable no-follow descriptor."""
    path = Path(path)
    parents_before = _parent_identity(path, label)
    if not hasattr(os, "O_NOFOLLOW"):
        raise CaptureError("{} cannot enforce no-follow opens".format(label))
    flags = os.O_RDONLY | os.O_NOFOLLOW
    flags |= getattr(os, "O_CLOEXEC", 0)
    try:
        fd = os.open(str(path), flags)
    except OSError as exc:
        raise CaptureError("{} cannot be opened without following a symlink: {}".format(
            label, exc))
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode):
            raise CaptureError("{} target is not a regular file: {}".format(label, path))
        if before.st_nlink != 1:
            raise CaptureError("{} target is not single-link: {}".format(label, path))
        if require_root_owner and (before.st_uid != 0 or before.st_gid != 0):
            raise CaptureError("{} target is not root-owned: {}".format(label, path))
        if stat.S_IMODE(before.st_mode) != ROS2_SOURCES_TARGET_MODE:
            raise CaptureError("{} target mode is not 0644: {}".format(label, path))
        if before.st_size <= 0 or before.st_size > TARGET_MAX_BYTES:
            raise CaptureError("{} target size is outside the bounded limit: {}".format(
                label, path))
        first_digest = _digest_fd(fd, before.st_size, label)
        os.lseek(fd, 0, os.SEEK_SET)
        second_digest = _digest_fd(fd, before.st_size, label)
        after = os.fstat(fd)
        if _stat_identity(before) != _stat_identity(after):
            raise CaptureError("{} target metadata changed while being read: {}".format(
                label, path))
        if first_digest != second_digest:
            raise CaptureError("{} target content changed while being read: {}".format(
                label, path))
        descriptor = {
            "path": str(path),
            "size_bytes": before.st_size,
            "sha256": first_digest,
            "mode": stat.S_IMODE(before.st_mode),
            "uid": before.st_uid,
            "gid": before.st_gid,
            "nlink": before.st_nlink,
        }
    except OSError as exc:
        raise CaptureError("{} target read failed: {}".format(label, exc))
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
    try:
        after_path = os.lstat(str(path))
    except OSError as exc:
        raise CaptureError("{} target disappeared after reading: {}".format(label, exc))
    if _stat_identity(after_path) != _stat_identity(before):
        raise CaptureError("{} target path changed while being read: {}".format(label, path))
    if _parent_identity(path, label) != parents_before:
        raise CaptureError("{} target parent changed while being read: {}".format(label, path))
    return descriptor


def _record_allowed_ros2_symlink(path, stable_path, *, filesystem_root, apt_root):
    """Record the only source-list symlink admitted by the contract."""
    path = Path(path)
    stable_path = _stable_relative(stable_path, "apt source symlink path")
    expected_target = _target_expected_path(filesystem_root, apt_root)
    parents_before = _parent_identity(path, "apt source symlink")
    try:
        info = os.lstat(str(path))
        target = os.readlink(str(path))
    except OSError as exc:
        raise CaptureError("apt source symlink cannot be inspected: {}".format(exc))
    if not stat.S_ISLNK(info.st_mode):
        raise CaptureError("apt source exception is not a symlink: {}".format(path))
    if target != expected_target:
        raise CaptureError(
            "apt source symlink target is not the fixed absolute target: {}".format(path))
    if stat.S_IMODE(info.st_mode) != 0o777 or info.st_nlink != 1:
        raise CaptureError("apt source symlink metadata is unsafe: {}".format(path))
    production_root = filesystem_root is None or Path(filesystem_root) == Path("/")
    if production_root and (info.st_uid != 0 or info.st_gid != 0):
        raise CaptureError("apt source symlink is not root-owned: {}".format(path))
    if info.st_size != len(target.encode("utf-8")):
        raise CaptureError("apt source symlink size is inconsistent: {}".format(path))
    target_descriptor = _record_resolved_target(
        Path(target), "apt source symlink target", require_root_owner=production_root)
    try:
        after = os.lstat(str(path))
        target_after = os.readlink(str(path))
    except OSError as exc:
        raise CaptureError("apt source symlink changed while being read: {}".format(exc))
    if (_stat_identity(after) != _stat_identity(info) or
            target_after != target or
            _parent_identity(path, "apt source symlink") != parents_before):
        raise CaptureError("apt source symlink changed while being read: {}".format(path))
    return {
        "path": stable_path,
        "target": target,
        "size_bytes": info.st_size,
        "mode": stat.S_IMODE(info.st_mode),
        "uid": info.st_uid,
        "gid": info.st_gid,
        "nlink": info.st_nlink,
        "target_file": target_descriptor,
    }


def _record_file(path, stable_path, label):
    path = _regular(path, label)
    stable_path = _stable_relative(stable_path, label + " path")
    return {
        "path": stable_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
    }


def _record_tree(root, stable_prefix, label, required=True, predicate=None):
    """Hash regular files below one fixed root without following links."""
    root = _directory(root, label)
    entries = []
    for current, directories, files in os.walk(str(root), topdown=True, followlinks=False):
        current_path = Path(current)
        for directory_name in list(directories):
            candidate = current_path / directory_name
            try:
                if stat.S_ISLNK(os.lstat(str(candidate)).st_mode):
                    raise CaptureError("{} contains a symlink directory: {}".format(label, candidate))
            except OSError as exc:
                raise CaptureError("{} directory cannot be inspected: {}".format(label, exc))
        for file_name in files:
            candidate = current_path / file_name
            relative = candidate.relative_to(root).as_posix()
            if predicate is not None and not predicate(relative):
                continue
            stable = stable_prefix + "/" + relative if relative else stable_prefix
            entries.append(_record_file(candidate, stable, label))
    entries.sort(key=lambda item: item["path"])
    if len({item["path"] for item in entries}) != len(entries):
        raise CaptureError("{} contains duplicate normalized paths".format(label))
    if required and not entries:
        raise CaptureError("{} is empty".format(label))
    return {
        "entries": entries,
        "entry_count": len(entries),
        "tree_sha256": _sha256_bytes(_canonical(entries).encode("utf-8")),
    }


def _record_source_config_tree(root, stable_prefix, label, *, filesystem_root,
                               apt_root):
    """Capture apt source files with one explicit, fixed symlink exception."""
    root = _directory(root, label)
    entries = []
    symlink_entries = []
    for current, directories, files in os.walk(str(root), topdown=True, followlinks=False):
        current_path = Path(current)
        for directory_name in list(directories):
            candidate = current_path / directory_name
            try:
                if stat.S_ISLNK(os.lstat(str(candidate)).st_mode):
                    raise CaptureError("{} contains a symlink directory: {}".format(
                        label, candidate))
            except OSError as exc:
                raise CaptureError("{} directory cannot be inspected: {}".format(label, exc))
        for file_name in files:
            candidate = current_path / file_name
            relative = candidate.relative_to(root).as_posix()
            stable = stable_prefix + "/" + relative if relative else stable_prefix
            try:
                mode = os.lstat(str(candidate)).st_mode
            except OSError as exc:
                raise CaptureError("{} file cannot be inspected: {}".format(label, exc))
            if stat.S_ISLNK(mode):
                if relative != ROS2_SOURCES_RELATIVE.split("/", 1)[1]:
                    raise CaptureError("{} contains an unapproved symlink: {}".format(
                        label, candidate))
                symlink_entries.append(_record_allowed_ros2_symlink(
                    candidate, stable, filesystem_root=filesystem_root, apt_root=apt_root))
            else:
                entries.append(_record_file(candidate, stable, label))
    entries.sort(key=lambda item: item["path"])
    symlink_entries.sort(key=lambda item: item["path"])
    if len(symlink_entries) > 1:
        raise CaptureError("{} contains duplicate ros2.sources symlinks".format(label))
    if not entries and not symlink_entries:
        raise CaptureError("{} is empty".format(label))
    tree_projection = {"entries": entries, "symlink_entries": symlink_entries}
    return {
        "entries": entries,
        "entry_count": len(entries),
        "symlink_entries": symlink_entries,
        "symlink_count": len(symlink_entries),
        "tree_sha256": _sha256_bytes(_canonical(tree_projection).encode("utf-8")),
    }


def _record_direct_and_tree(root, stable_prefix, directory_name, label, *,
                            filesystem_root=None):
    """Capture sources.list plus the sources.list.d tree without auth files."""
    root = _directory(root, label + " root")
    entries = []
    symlink_entries = []
    direct = root / "sources.list"
    if direct.exists() or direct.is_symlink():
        entries.append(_record_file(direct, stable_prefix + "/sources.list", label))
    listed = root / directory_name
    if listed.exists() or listed.is_symlink():
        listed_section = _record_source_config_tree(
            listed, stable_prefix + "/" + directory_name,
            label + " directory", filesystem_root=filesystem_root, apt_root=root)
        entries.extend(listed_section["entries"])
        symlink_entries.extend(listed_section["symlink_entries"])
    entries.sort(key=lambda item: item["path"])
    symlink_entries.sort(key=lambda item: item["path"])
    if not entries and not symlink_entries:
        raise CaptureError("{} has no source configuration files".format(label))
    tree_projection = {"entries": entries, "symlink_entries": symlink_entries}
    return {
        "entries": entries,
        "entry_count": len(entries),
        "symlink_entries": symlink_entries,
        "symlink_count": len(symlink_entries),
        "tree_sha256": _sha256_bytes(_canonical(tree_projection).encode("utf-8")),
    }


def _record_multiple_trees(roots, label, required=True):
    entries = []
    for stable_prefix, root in roots:
        if Path(root).exists() or Path(root).is_symlink():
            entries.extend(_record_tree(root, stable_prefix, label)["entries"])
    entries.sort(key=lambda item: item["path"])
    if required and not entries:
        raise CaptureError("{} has no files".format(label))
    if len({item["path"] for item in entries}) != len(entries):
        raise CaptureError("{} contains duplicate normalized paths".format(label))
    return {
        "entries": entries,
        "entry_count": len(entries),
        "tree_sha256": _sha256_bytes(_canonical(entries).encode("utf-8")),
    }


def _run(argv, label):
    try:
        completed = subprocess.run(
            [str(item) for item in argv], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, universal_newlines=True, check=False,
        )
    except OSError as exc:
        raise CaptureError("{} could not start: {}".format(label, exc))
    if completed.returncode != 0:
        raise CaptureError("{} failed with exit {}".format(label, completed.returncode))
    return completed.stdout


def _dpkg_packages(query_runner=None):
    query = "${Package}\t${Version}\t${Architecture}\t${Status}\n"
    if query_runner is None:
        output = _run(["dpkg-query", "-W", "-f=" + query], "dpkg-query")
    else:
        output = query_runner(query)
    packages = []
    for line in output.splitlines():
        if not line:
            continue
        fields = line.split("\t")
        if len(fields) != 4 or any(not field for field in fields):
            raise CaptureError("dpkg-query emitted a malformed package row")
        packages.append({
            "name": fields[0], "version": fields[1],
            "architecture": fields[2], "status": fields[3],
        })
    packages.sort(key=lambda item: (
        item["name"], item["version"], item["architecture"], item["status"],
    ))
    keys = [tuple(item.values()) for item in packages]
    if not packages or len(set(keys)) != len(keys):
        raise CaptureError("dpkg package closure is empty or duplicated")
    return {
        "packages": packages,
        "package_count": len(packages),
        "canonical_sha256": _sha256_bytes(_canonical(packages).encode("utf-8")),
    }


def _architecture(query_runner=None):
    if query_runner is None:
        value = _run(["dpkg", "--print-architecture"], "dpkg architecture").strip()
    else:
        value = query_runner().strip()
    if value not in ("amd64", "arm64", "armhf", "i386"):
        raise CaptureError("unsupported or missing dpkg architecture: {}".format(value))
    machine = platform.machine().lower()
    return value, machine


def build_manifest(distro, image_digest, install_command_sha256,
                   install_exit_code, capture_script_sha256,
                   *, apt_root=Path("/etc/apt"), apt_lists_root=Path("/var/lib/apt/lists"),
                   rosdep_sources_root=Path("/etc/ros/rosdep/sources.list.d"),
                   rosdep_cache_roots=None, dpkg_query_runner=None,
                   architecture_runner=None, filesystem_root=None):
    """Build a deterministic manifest; filesystem roots are injectable for tests."""
    if distro not in DISTROS:
        raise CaptureError("unsupported ROS distro: {}".format(distro))
    _validate_image_digest(image_digest, "base image digest")
    _validate_sha(install_command_sha256, "install command hash")
    _validate_sha(capture_script_sha256, "capture script hash")
    if install_exit_code != 0:
        raise CaptureError("dependency install did not exit zero")
    if rosdep_cache_roots is None:
        rosdep_cache_roots = (
            ("rosdep_cache/var", Path("/var/lib/ros/rosdep")),
            ("rosdep_cache/root", Path("/root/.ros/rosdep")),
        )
    dpkg_arch, machine = _architecture(architecture_runner)
    apt_sources = _record_direct_and_tree(
        apt_root, "apt_sources", "sources.list.d", "apt source config",
        filesystem_root=filesystem_root)
    apt_indexes = _record_tree(
        apt_lists_root, "apt_indexes", "apt package indexes")
    release_entries = [
        entry for entry in apt_indexes["entries"]
        if "_InRelease" in entry["path"] or entry["path"].endswith("/Release")
    ]
    if not release_entries:
        raise CaptureError("apt Release/InRelease identity is missing")
    apt_indexes["release_indexes"] = release_entries
    rosdep_sources = _record_tree(
        rosdep_sources_root, "rosdep_sources", "rosdep source list")
    rosdep_cache = _record_multiple_trees(
        rosdep_cache_roots, "rosdep cache")
    packages = _dpkg_packages(dpkg_query_runner)
    manifest = {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": "PASS",
        "base_image": {
            "digest": image_digest,
            "os": "linux",
            "architecture": "amd64" if dpkg_arch == "amd64" else dpkg_arch,
            "dpkg_architecture": dpkg_arch,
            "machine": machine,
        },
        "ros": {"distro": distro},
        "dpkg": packages,
        "apt": {"source_config": apt_sources, "indexes": apt_indexes},
        "rosdep": {"sources": rosdep_sources, "cache": rosdep_cache},
        "install_command": {"sha256": install_command_sha256, "exit_code": install_exit_code},
        "capture": {"script_sha256": capture_script_sha256},
        "secret_policy": {
            "environment_recorded": [],
            "credential_values_recorded": False,
            "proxy_values_recorded": False,
        },
    }
    # The canonical identity intentionally excludes schema/status wrappers and
    # the identity field itself.  The host summary uses this same projection
    # when comparing absent/present legs, so a capture cannot pass one side of
    # the contract while binding a different dependency closure on the other.
    projection = {
        "base_image": manifest["base_image"],
        "ros": manifest["ros"],
        "dpkg": manifest["dpkg"],
        "apt": manifest["apt"],
        "rosdep": manifest["rosdep"],
        "install_command": manifest["install_command"],
        "capture": manifest["capture"],
        "secret_policy": manifest["secret_policy"],
    }
    manifest["canonical_sha256"] = _sha256_bytes(
        _canonical(projection).encode("utf-8"))
    return manifest


def _write_exclusive(path, payload):
    path = Path(path)
    if not path.is_absolute() or path.exists() or path.is_symlink():
        raise CaptureError("output is not a fresh absolute path: {}".format(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise CaptureError("output parent is invalid: {}".format(path.parent))
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        try:
            os.close(fd)
        except OSError:
            pass
        raise
    os.chmod(str(path), 0o444)
    return path


def seal_manifest(path, manifest):
    payload = (_canonical(manifest) + "\n").encode("utf-8")
    receipt = _write_exclusive(path, payload)
    digest = _sha256_bytes(payload)
    sidecar = _write_exclusive(
        Path(str(receipt) + ".sha256"),
        (digest + "  " + receipt.name + "\n").encode("ascii"),
    )
    return {
        "path": str(receipt), "sidecar": str(sidecar),
        "sha256": digest, "sidecar_sha256": _sha256_file(sidecar),
    }


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--distro", required=True, choices=sorted(DISTROS))
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--install-command-sha256", required=True)
    parser.add_argument("--install-exit-code", required=True, type=int)
    parser.add_argument("--capture-script-sha256", required=True)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        manifest = build_manifest(
            args.distro, args.image_digest, args.install_command_sha256,
            args.install_exit_code, args.capture_script_sha256,
        )
        seal = seal_manifest(args.output, manifest)
        print(json.dumps({"status": "PASS", "manifest": seal}, sort_keys=True))
        return 0
    except (CaptureError, OSError, ValueError) as exc:
        print("dependency environment capture: FAIL_CLOSED: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
