#!/usr/bin/env python3
"""Host-owned directory identity contract for release-leg evidence.

The outer launcher creates the fixed directory layout before creating a
container.  The in-container runner can only open and use those directories;
it must never create, replace, chmod, or chown them.  This small module keeps
the filesystem checks shared without making a JSON receipt itself an authority
for the directory identity.
"""

from __future__ import print_function

import os
import stat
from pathlib import Path


SCHEMA = "registration-plugin-host-evidence-directory-v1"
SCHEMA_VERSION = 1
DIRECTORY_MODE = 0o700


class EvidenceDirectoryError(RuntimeError):
    """A fail-closed evidence-directory contract error."""

    def __init__(self, kind, message):
        super(EvidenceDirectoryError, self).__init__(message)
        self.kind = kind


def _absolute_normalized(path, label):
    try:
        value = os.fspath(path)
    except TypeError as exc:
        raise EvidenceDirectoryError("PATH_INVALID", "{} is not a path".format(label)) from exc
    if (not isinstance(value, str) or not value.startswith("/") or
            "\x00" in value or "\n" in value or "\r" in value or
            Path(value).as_posix() != value or
            any(part in ("", ".", "..") for part in value.split("/")[1:])):
        raise EvidenceDirectoryError(
            "PATH_INVALID", "{} is not absolute and normalized".format(label))
    return Path(value)


def _relative(value, label):
    if (not isinstance(value, str) or not value or value.startswith("/") or
            "\\" in value or "\x00" in value or "\n" in value or "\r" in value):
        raise EvidenceDirectoryError("PATH_INVALID", "{} is not a safe relative path".format(label))
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise EvidenceDirectoryError("PATH_INVALID", "{} contains traversal".format(label))
    if "/".join(parts) != value:
        raise EvidenceDirectoryError("PATH_INVALID", "{} is not normalized".format(label))
    return value


def _open_directory(path, label):
    """Open every component with O_NOFOLLOW, avoiding parent path races."""
    path = _absolute_normalized(path, label)
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open("/", flags)
    except OSError as exc:
        raise EvidenceDirectoryError("DIRECTORY_OPEN_FAILED", "{}: {}".format(label, exc)) from exc
    try:
        for component in path.parts[1:]:
            try:
                child = os.open(component, flags, dir_fd=descriptor)
            except OSError as exc:
                raise EvidenceDirectoryError(
                    "DIRECTORY_OPEN_FAILED", "{}: {}".format(label, exc)) from exc
            os.close(descriptor)
            descriptor = child
        return descriptor
    except Exception:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def _descriptor(path, relative, descriptor, expected_mode, expected_owner):
    try:
        info = os.fstat(descriptor)
    except OSError as exc:
        raise EvidenceDirectoryError("DIRECTORY_STAT_FAILED", str(exc)) from exc
    if not stat.S_ISDIR(info.st_mode):
        raise EvidenceDirectoryError("DIRECTORY_NOT_REGULAR", str(path))
    mode = stat.S_IMODE(info.st_mode)
    if mode != expected_mode:
        raise EvidenceDirectoryError(
            "DIRECTORY_MODE_DRIFT", "{} mode {:o}, expected {:o}".format(path, mode, expected_mode))
    if info.st_nlink < 2:
        raise EvidenceDirectoryError("DIRECTORY_NLINK_INVALID", str(path))
    if expected_owner is not None and (info.st_uid, info.st_gid) != tuple(expected_owner):
        raise EvidenceDirectoryError(
            "DIRECTORY_OWNER_DRIFT",
            "{} owner {}:{}, expected {}:{}".format(
                path, info.st_uid, info.st_gid, expected_owner[0], expected_owner[1]),
        )
    return {
        "path": relative,
        "device": int(info.st_dev),
        "inode": int(info.st_ino),
        "uid": int(info.st_uid),
        "gid": int(info.st_gid),
        "mode": mode,
        "nlink": int(info.st_nlink),
    }


def _inspect(path, relative, expected_mode=DIRECTORY_MODE, expected_owner=None):
    descriptor = _open_directory(path, "evidence directory")
    try:
        before = _descriptor(path, relative, descriptor, expected_mode, expected_owner)
        after = _descriptor(path, relative, descriptor, expected_mode, expected_owner)
    finally:
        os.close(descriptor)
    if before != after:
        raise EvidenceDirectoryError("DIRECTORY_CHANGED", str(path))
    return after


def _close_parent(path, label):
    return _open_directory(path.parent, label + " parent")


def _mkdir_fresh(path, relative, expected_mode=DIRECTORY_MODE, expected_owner=None):
    parent_fd = _close_parent(path, "evidence directory")
    try:
        try:
            os.stat(path.name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            pass
        except OSError as exc:
            raise EvidenceDirectoryError(
                "DIRECTORY_COLLISION", "{}: {}".format(path, exc)) from exc
        else:
            raise EvidenceDirectoryError("DIRECTORY_COLLISION", str(path))
        try:
            os.mkdir(path.name, expected_mode, dir_fd=parent_fd)
            os.fsync(parent_fd)
        except OSError as exc:
            raise EvidenceDirectoryError(
                "DIRECTORY_CREATE_FAILED", "{}: {}".format(path, exc)) from exc
    finally:
        os.close(parent_fd)
    return _inspect(path, relative, expected_mode, expected_owner)


def _validate_root(root, expected_mode=DIRECTORY_MODE, expected_owner=None):
    root = _absolute_normalized(root, "evidence root")
    return _inspect(root, ".", expected_mode, expected_owner)


def _normalize_paths(relatives):
    try:
        values = list(relatives)
    except TypeError as exc:
        raise EvidenceDirectoryError("DIRECTORY_LAYOUT_INVALID", "paths are not iterable") from exc
    normalized = [_relative(value, "directory layout path") for value in values]
    if len(set(normalized)) != len(normalized):
        raise EvidenceDirectoryError("DIRECTORY_LAYOUT_DUPLICATE", "duplicate directory path")
    return sorted(normalized, key=lambda value: (value.count("/"), value))


def create_fresh_root(path, mode=DIRECTORY_MODE, owner=None):
    """Atomically create a fresh host-owned root and return its identity."""
    path = _absolute_normalized(path, "evidence root")
    if owner is None:
        owner = (os.getuid(), os.getgid())
    parent_fd = _close_parent(path, "evidence root")
    try:
        try:
            os.stat(path.name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            pass
        except OSError as exc:
            raise EvidenceDirectoryError("ROOT_COLLISION", str(path)) from exc
        else:
            raise EvidenceDirectoryError("ROOT_NOT_FRESH", str(path))
        try:
            os.mkdir(path.name, mode, dir_fd=parent_fd)
            os.fsync(parent_fd)
        except OSError as exc:
            raise EvidenceDirectoryError("ROOT_CREATE_FAILED", str(exc)) from exc
    finally:
        os.close(parent_fd)
    descriptor = _inspect(path, ".", mode, owner)
    descriptor["path"] = str(path)
    return descriptor


def create_layout(root, relatives, mode=DIRECTORY_MODE, owner=None):
    """Create every fixed child directory before a container is created."""
    root = _absolute_normalized(root, "evidence root")
    if owner is None:
        owner = (os.getuid(), os.getgid())
    _validate_root(root, mode, owner)
    paths = _normalize_paths(relatives)
    try:
        with os.scandir(str(root)) as entries:
            if next(entries, None) is not None:
                raise EvidenceDirectoryError(
                    "DIRECTORY_LAYOUT_NOT_FRESH", "evidence root is not empty")
    except OSError as exc:
        raise EvidenceDirectoryError("DIRECTORY_LAYOUT_INVALID", str(exc)) from exc
    entries = []
    for relative in paths:
        path = root.joinpath(*relative.split("/"))
        parent = path.parent
        try:
            _inspect(parent, parent.relative_to(root).as_posix() or ".", mode, owner)
        except ValueError as exc:
            raise EvidenceDirectoryError("PATH_ESCAPE", str(path)) from exc
        entries.append(_mkdir_fresh(path, relative, mode, owner))
    # A parent directory's link count grows while nested directories are
    # created.  Capture the contract only after the complete fixed layout is
    # present; the pre-container snapshot is then an exact identity, not a
    # best-effort lower bound that could hide a later directory addition.
    root_before = _inspect(root, ".", mode, owner)
    entries = [_inspect(root.joinpath(*relative.split("/")), relative, mode, owner)
               for relative in paths]
    entries.sort(key=lambda value: value["path"])
    return {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "root": str(root),
        "mode": mode,
        "owner": {"uid": int(owner[0]), "gid": int(owner[1])},
        "paths": [entry["path"] for entry in entries],
        "root_before": root_before,
        "before": entries,
    }


def snapshot_existing(root, relatives, mode=DIRECTORY_MODE, owner=None):
    """Snapshot a pre-created layout without creating or changing anything."""
    root = _absolute_normalized(root, "evidence root")
    # Host-owned layouts must have one explicit owner.  ``None`` means derive
    # it from the root, never "accept any owner" for a child directory.
    root_identity = _validate_root(root, mode, owner)
    if owner is None:
        owner = (root_identity["uid"], root_identity["gid"])
    paths = _normalize_paths(relatives)
    entries = []
    for relative in paths:
        path = root.joinpath(*relative.split("/"))
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise EvidenceDirectoryError("PATH_ESCAPE", str(path)) from exc
        entries.append(_inspect(path, relative, mode, owner))
    entries.sort(key=lambda value: value["path"])
    return {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "root": str(root),
        "mode": mode,
        "owner": {"uid": int(root_identity["uid"]), "gid": int(root_identity["gid"])},
        "paths": [entry["path"] for entry in entries],
        "root_before": root_identity,
        "before": entries,
    }


def _normalize_root_directory_additions(values):
    """Normalize explicit post-snapshot directories directly below root."""
    if not isinstance(values, (list, tuple)):
        raise EvidenceDirectoryError(
            "DIRECTORY_CONTRACT_INVALID", "root directory additions")
    normalized = []
    for value in values:
        if (not isinstance(value, str) or not value or value in (".", "..") or
                "/" in value or "\\" in value):
            raise EvidenceDirectoryError(
                "DIRECTORY_CONTRACT_INVALID", "root directory addition path")
        normalized.append(value)
    if normalized != sorted(set(normalized)):
        raise EvidenceDirectoryError(
            "DIRECTORY_CONTRACT_INVALID", "root directory additions drift")
    return tuple(normalized)


def verify_snapshot(root, snapshot, mode=DIRECTORY_MODE, owner=None,
                    allowed_root_directory_additions=()):
    """Reopen a snapshot and require every directory identity to be unchanged."""
    if not isinstance(snapshot, dict) or snapshot.get("schema") != SCHEMA or \
            snapshot.get("schema_version") != SCHEMA_VERSION:
        raise EvidenceDirectoryError("DIRECTORY_CONTRACT_INVALID", "directory contract schema")
    root = _absolute_normalized(root, "evidence root")
    if snapshot.get("root") != str(root) or snapshot.get("mode") != mode:
        raise EvidenceDirectoryError("DIRECTORY_CONTRACT_INVALID", "directory root/mode drift")
    paths = _normalize_paths(snapshot.get("paths", []))
    expected_entries = snapshot.get("before")
    if (not isinstance(expected_entries, list) or
            [item.get("path") for item in expected_entries] != sorted(paths)):
        raise EvidenceDirectoryError("DIRECTORY_CONTRACT_INVALID", "directory paths drift")
    expected_owner = owner
    if expected_owner is None:
        owner_value = snapshot.get("owner")
        if not isinstance(owner_value, dict) or set(owner_value) != {"uid", "gid"}:
            raise EvidenceDirectoryError("DIRECTORY_CONTRACT_INVALID", "directory owner missing")
        expected_owner = (owner_value["uid"], owner_value["gid"])
    additions = _normalize_root_directory_additions(
        allowed_root_directory_additions)
    managed_top_levels = {path.split("/", 1)[0] for path in paths}
    if any(value in managed_top_levels for value in additions):
        raise EvidenceDirectoryError(
            "DIRECTORY_CONTRACT_INVALID", "managed root directory addition")
    # Bind every permitted link-count increase to a named, safely reopened
    # directory.  This keeps the default exact and cannot hide an unrelated
    # directory addition with a raw numeric allowance.
    for value in additions:
        _inspect(root / value, value, mode, expected_owner)
    root_expected = snapshot.get("root_before")
    root_after = _validate_root(root, mode, expected_owner)
    if not _identity_stable(root_expected, root_after,
                            nlink_delta=len(additions)):
        raise EvidenceDirectoryError("DIRECTORY_IDENTITY_DRIFT", "evidence root changed")
    after = []
    for relative in paths:
        path = root.joinpath(*relative.split("/"))
        after.append(_inspect(path, relative, mode, expected_owner))
    after.sort(key=lambda value: value["path"])
    if len(after) != len(expected_entries) or any(
            not _identity_stable(expected, observed)
            for expected, observed in zip(expected_entries, after)):
        raise EvidenceDirectoryError("DIRECTORY_IDENTITY_DRIFT", "managed directory changed")
    return after


def _identity_stable(expected, observed, nlink_delta=0):
    """Compare an inode while permitting legitimate child-directory links."""
    if not isinstance(expected, dict) or not isinstance(observed, dict):
        return False
    for key in ("path", "device", "inode", "uid", "gid", "mode"):
        if observed.get(key) != expected.get(key):
            return False
    return (type(nlink_delta) is int and nlink_delta >= 0 and
            isinstance(expected.get("nlink"), int) and
            isinstance(observed.get("nlink"), int) and
            expected["nlink"] >= 2 and
            observed["nlink"] == expected["nlink"] + nlink_delta)


def finalize(snapshot, after, allowed_root_directory_additions=()):
    """Return a serializable contract with both pre/post observations."""
    if not isinstance(snapshot, dict) or not isinstance(after, list):
        raise EvidenceDirectoryError("DIRECTORY_CONTRACT_INVALID", "cannot finalize contract")
    result = dict(snapshot)
    additions = _normalize_root_directory_additions(
        allowed_root_directory_additions)
    if additions:
        result["allowed_root_directory_additions"] = list(additions)
    result["after"] = after
    result["status"] = "PASS"
    return result


def failed(snapshot, error):
    """Describe a failed verification without asserting a false PASS."""
    result = dict(snapshot) if isinstance(snapshot, dict) else {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
    }
    result["status"] = "FAIL_CLOSED"
    result["failure"] = str(error)
    return result


def release_runner_paths(dependency_names=(), include_prefetch=False):
    """Return the fixed runner layout relative to one per-leg evidence root."""
    paths = {
        "work", "work/src", "work/build", "work/install", "work/logs",
        "work/logs/colcon-build", "work/test-results", "work/consumer",
        "work/template", "work/evidence", "work/evidence/odr-evidence",
    }
    names = sorted(set(dependency_names))
    if names:
        paths.update({"work/archives", "work/vendor"})
        for name in names:
            paths.update({
                "work/vendor/{}-extract".format(name),
                "work/vendor/{}-build".format(name),
                "work/vendor/{}-install".format(name),
            })
    if include_prefetch:
        paths.update({"prefetch", "prefetch/archives"})
    return tuple(sorted(paths, key=lambda value: (value.count("/"), value)))


def launcher_paths(child_relative, dependency_names=(), include_prefetch=False):
    """Return all outer/inner fixed directories relative to the host root."""
    child_relative = _relative(child_relative, "evidence child")
    paths = {"launcher", "launcher/diagnostics", child_relative}
    for relative in release_runner_paths(dependency_names, include_prefetch):
        paths.add(child_relative + "/" + relative)
    return tuple(sorted(paths, key=lambda value: (value.count("/"), value)))


if __name__ == "__main__":
    raise SystemExit("registration_plugin_evidence_directory is a library module; import it instead of running it directly.")
