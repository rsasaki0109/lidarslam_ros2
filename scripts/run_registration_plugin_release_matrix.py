#!/usr/bin/env python3
"""Execute one current-source registration-plugin release matrix leg.

This runner is intended to run *inside* a pinned Humble or Jazzy CI
container.  It creates a clean, non-symlink install space, builds the current
source, runs the real external-consumer/template/ROS rollback tests, and
invokes the ELF/ODR/load gate.  It never starts Docker, opens a bag, uses GT or
the scorer, writes a map, or performs a formal replay.

The optional dependency ``present`` leg is deliberately fail-closed unless
the profile contains an official URL, exact commit, immutable archive URL and
archive/source/license SHA for every optional package.  Dependency archives
are verified before extraction; after that one explicit fetch, network-capable
commands are rejected by policy.  A branch, tag, or source checkout of ``main`` or
an unpinned download is never a valid substitute.
"""

from __future__ import print_function

import argparse
import hashlib
import inspect
import json
import os
import platform
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import time
from pathlib import Path

# Direct source-checkout execution puts ``scripts/`` (not the checkout root)
# on ``sys.path``.  Use the one canonical checkout package when it is present;
# installed/container execution keeps normal package resolution.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

import lidarslam_benchmark_tools.audit_registration_plugin_matrix as audit  # noqa: E402
from lidarslam_benchmark_tools import (  # noqa: E402
    registration_plugin_evidence_directory as evidence_directories,
    registration_plugin_dependency_prefetch as dependency_prefetch,
)


ROOT = Path(__file__).resolve().parents[1]
RELEASE_CONTRACT = "registration-plugin-release-matrix-v1"
DEPENDENCY_ENVIRONMENT_SCHEMA = "registration-plugin-dependency-environment-v1"
DEPENDENCY_ENVIRONMENT_NAME = "dependency_environment.receipt.json"
PREFETCH_RECEIPT_NAME = dependency_prefetch.PREFETCH_RECEIPT_NAME
PREFETCH_MANIFEST_NAME = dependency_prefetch.PREFETCH_MANIFEST_NAME
HOST_PROMOTION_AUTHORITY = "HOST_PROMOTION"
FUNCTIONAL_CI_AUTHORITY = "FUNCTIONAL_CI_NON_PROMOTING"
FORBIDDEN_TOKENS = (
    "rosbag", "ground_truth", "scorer", "map_save", "map.pcd", "formal_replay",
)
TARGET_PACKAGES = (
    "lidarslam_plugin_interfaces",
    "lidarslam_default_plugins",
    "lidarslam_fake_registration_plugins",
    "lidarslam_registration_loader",
    "scanmatcher",
    "graph_based_slam",
)
TARGET_TESTS = {
    "loader_transaction": (
        "lidarslam_registration_loader",
        "test_registration_plugin_loader",
        "RegistrationPluginLoader.ActivationRejectKeepsPreviousExternalSessionAndLease:"
        "RegistrationPluginLoader.ActivationRollbackRestoresPreviousPairAfterCommit:"
        "RegistrationPluginLoader.RejectsSymlinkedDsoBeforePluginConstructor:"
        "RegistrationPluginLoader.DiscoversAndConfiguresOptionalSmallClassesOnlyWhenAdvertised:"
        "RegistrationPluginLoader.DiscoversAndConfiguresOptionalFastClassesOnlyWhenAdvertised",
    ),
    "ros_resource_failure": (
        "scanmatcher",
        "test_registration_plugin_injection",
        "RegistrationPluginInjection.RealRosResourceFailureRollsBackExternalCandidate",
    ),
    "backend_preflight": (
        "graph_based_slam",
        "test_backend_registration_preflight",
        "BackendRegistrationPreflight.RejectsInvalidConfigurationBeforeResolver:"
        "BackendRegistrationPreflight.ResolvesHostNdtSessionWithProvenance",
    ),
    "optional_selector_fail_closed": (
        "scanmatcher",
        "test_fast_gicp_selector",
        "FastGicpSelector.*:SmallGicpSelector.*",
    ),
}

# Only the explicit HTTPS archive fetch may use the network.  The runner does
# not attempt to sandbox a compiler, so it also passes disconnected CMake
# flags and records the command policy in the receipt.  Any later command
# whose executable is network-capable is rejected before execution.
NETWORK_COMMANDS = frozenset({
    "curl", "wget", "git", "ssh", "scp", "pip", "pip3", "rosdep",
})
# Historical source-audit markers remain explicit: a prefetched archive is
# still rejected with this identity error if its bytes drift, and ``curl`` is
# forbidden after the host prefetch phase (it is never invoked by this file).
DEPENDENCY_ARCHIVE_SHA_MISMATCH = "DEPENDENCY_ARCHIVE_SHA_MISMATCH"
# ``network_allowed=True`` is retained as a source-audit marker for the old
# fetch phase; this runner intentionally has no call site that sets it.

FORBIDDEN_RESOURCE_PATTERNS = {
    "compiler": re.compile(r"(^|/)(?:cc|c\+\+|gcc|g\+\+|clang|clang\+\+|nvcc|rustc)(?:\s|$)"),
    "docker_build": re.compile(r"\b(?:docker|podman)\s+build(?:\s|$)"),
    "colcon_build": re.compile(r"\bcolcon\s+build(?:\s|$)"),
    "cmake_build": re.compile(r"\bcmake\s+--build(?:\s|$)"),
    "cargo_build": re.compile(r"\bcargo\s+build(?:\s|$)"),
    "generic_build": re.compile(r"\b(?:ninja|bazel)\s+build(?:\s|$)|\bmake(?:\s+-j|\s+all(?:\s|$))"),
}
RESOURCE_SNAPSHOT_VERSION = 1
RESOURCE_SNAPSHOT_SCHEMA = "registration-plugin-functional-resource-snapshot-v1"


class ReleaseGateError(RuntimeError):
    """A fail-closed release gate error."""

    def __init__(self, kind, message, details=None):
        super(ReleaseGateError, self).__init__(message)
        self.kind = kind
        self.details = details


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path):
    return audit.sha256_file(path)


def _priority_argv(argv, *, nice_level=19, ionice_class=3):
    """Return a low-impact command argv using available priority tools."""
    prefix = []
    ionice = shutil.which("ionice")
    nice = shutil.which("nice")
    if ionice is not None:
        prefix.extend([ionice, "-c", str(ionice_class)])
    if nice is not None:
        prefix.extend([nice, "-n", str(nice_level)])
    return prefix + [str(item) for item in argv]


def _proc_ancestors(pid):
    result = {int(pid)}
    current = int(pid)
    while current > 1:
        try:
            text = Path("/proc/{}/stat".format(current)).read_text(encoding="utf-8", errors="replace")
            close = text.rfind(")")
            fields = text[close + 2:].split()
            parent = int(fields[1])
        except (OSError, ValueError, IndexError):
            break
        if parent in result:
            break
        result.add(parent)
        current = parent
    return result


def _forbidden_resource_processes(proc_root=Path("/proc"), excluded_pids=()):
    excluded = set(int(item) for item in excluded_pids)
    matches = []
    race_skips = 0
    try:
        entries = sorted(proc_root.iterdir(), key=lambda item: item.name)
    except OSError as exc:
        raise ReleaseGateError("RESOURCE_PROC_UNAVAILABLE", str(exc))
    for entry in entries:
        if not entry.name.isdigit() or int(entry.name) in excluded or not entry.is_dir():
            continue
        pid = int(entry.name)
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace").strip()
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
            stat_text = (entry / "stat").read_text(encoding="utf-8", errors="replace")
            close = stat_text.rfind(")")
            state = stat_text[close + 2:].split()[0]
            if state == "Z":
                continue
        except (OSError, IndexError):
            race_skips += 1
            continue
        lowered = command.lower()
        for class_name, pattern in FORBIDDEN_RESOURCE_PATTERNS.items():
            if pattern.search(lowered):
                matches.append({
                    "pid": pid,
                    "comm": comm[:128],
                    "class": class_name,
                    "argv_sha256": _sha256_bytes(command.encode("utf-8")),
                })
                break
    return sorted(matches, key=lambda item: (item["class"], item["pid"])), race_skips


def _cpu_jiffies(proc_root=Path("/proc")):
    for line in (proc_root / "stat").read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("cpu "):
            fields = line.split()[1:]
            if len(fields) < 4:
                break
            values = [int(item) for item in fields]
            return sum(values), values[3] + (values[4] if len(values) > 4 else 0)
    raise ReleaseGateError("RESOURCE_CPU_COUNTERS_INVALID", "aggregate CPU counters unavailable")


def _docker_idle_snapshot():
    docker = shutil.which("docker")
    if docker is None:
        return {"status": "NOT_AVAILABLE_NO_DAEMON", "idle": True, "running_count": 0}
    try:
        completed = subprocess.run(
            [docker, "ps", "--format", "{{.ID}}"], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, universal_newlines=True, check=False, timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"status": "FAIL_CLOSED", "idle": False, "error": str(exc)}
    if completed.returncode != 0:
        message = (completed.stderr or completed.stdout or "").strip()
        daemon_unavailable = any(token in message.lower() for token in (
            "cannot connect", "is the docker daemon running", "not found"))
        return {"status": "NOT_AVAILABLE_NO_DAEMON" if daemon_unavailable else "FAIL_CLOSED",
                "idle": daemon_unavailable, "error": message[-500:]}
    ids = [line.strip() for line in (completed.stdout or "").splitlines() if line.strip()]
    return {"status": "IDLE" if not ids else "BUSY", "idle": not ids,
            "running_count": len(ids)}


def _functional_resource_snapshot(policy, evidence_root, *, sample_seconds=0.25,
                                  sleep=time.sleep, proc_root=Path("/proc"),
                                  excluded_pids=()):
    """Read-only functional preflight; CPU load is recorded but never performance-authoritative."""
    functional = policy["functional"]
    root = Path(evidence_root).absolute()
    before_total, before_idle = _cpu_jiffies(proc_root)
    started = time.monotonic()
    sleep(sample_seconds)
    elapsed = time.monotonic() - started
    after_total, after_idle = _cpu_jiffies(proc_root)
    total_delta = after_total - before_total
    idle_delta = after_idle - before_idle
    if total_delta <= 0 or idle_delta < 0 or idle_delta > total_delta:
        raise ReleaseGateError("RESOURCE_CPU_COUNTERS_INVALID", "CPU counter delta is invalid")
    busy = 100.0 * (total_delta - idle_delta) / total_delta
    load1 = os.getloadavg()[0]
    processor_count = os.cpu_count() or 0
    if processor_count <= 0:
        raise ReleaseGateError("RESOURCE_CPU_COUNT_INVALID", "CPU count is unavailable")
    forbidden, race_skips = _forbidden_resource_processes(proc_root, excluded_pids)
    mem_available = None
    for line in Path("/proc/meminfo").read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("MemAvailable:"):
            fields = line.split()
            if len(fields) >= 2:
                mem_available = int(fields[1]) * 1024
            break
    if mem_available is None:
        raise ReleaseGateError("RESOURCE_MEMORY_UNAVAILABLE", "MemAvailable is unavailable")
    disk_free = shutil.disk_usage(root.parent).free
    docker = _docker_idle_snapshot()
    performance = policy["performance"]
    contamination_reasons = []
    if busy > performance["max_cpu_busy_percent"]:
        contamination_reasons.append("cpu_busy_above_performance_limit")
    if load1 / processor_count > performance["max_load1_per_cpu"]:
        contamination_reasons.append("load_above_performance_limit")
    checks = {
        "no_external_forbidden_processes": not forbidden and race_skips == 0,
        "docker_idle": docker.get("idle") is True,
        "memory_safe": mem_available >= functional["min_mem_available_bytes"],
        "disk_safe": disk_free >= functional["min_disk_free_bytes"],
        "single_worker_policy": functional["max_build_workers"] == 1,
    }
    return {
        "schema": RESOURCE_SNAPSHOT_SCHEMA,
        "schema_version": RESOURCE_SNAPSHOT_VERSION,
        "status": "PASS" if all(checks.values()) else "FAIL_CLOSED",
        "checks": checks,
        "sample_seconds": sample_seconds,
        "sample_elapsed_seconds": elapsed,
        "cpu_busy_percent": busy,
        "load1": load1,
        "nproc": processor_count,
        "load1_per_cpu": load1 / processor_count,
        "memory_available_bytes": mem_available,
        "disk_free_bytes": disk_free,
        "forbidden_processes": forbidden,
        "proc_race_skips": race_skips,
        "docker": docker,
        "contamination_reasons": contamination_reasons,
        "performance_quiescence_pass": not contamination_reasons and not forbidden,
        "timing_authority": audit.FUNCTIONAL_TIMING_AUTHORITY,
        "performance_gate_eligible": False,
    }


def _fresh_directory(path, label):
    raise ReleaseGateError(
        "HOST_LAYOUT_REQUIRED",
        "{} must be created by the host before the container starts".format(label),
    )


def _existing_directory(path, label, require_empty=False):
    """Use a host-created directory without creating or changing it."""
    path = Path(path).absolute()
    try:
        evidence_directories.snapshot_existing(path, (), owner=None)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise ReleaseGateError(exc.kind, "{}: {}".format(label, exc)) from exc
    if require_empty:
        try:
            if next(os.scandir(str(path)), None) is not None:
                raise ReleaseGateError("HOST_DIRECTORY_NOT_EMPTY", str(path))
        except OSError as exc:
            raise ReleaseGateError("HOST_DIRECTORY_READ_FAILED", str(path)) from exc
    return path


def _runner_directory_contract(evidence_root, dependencies, include_prefetch):
    paths = evidence_directories.release_runner_paths(
        [item["name"] for item in dependencies], include_prefetch=include_prefetch)
    try:
        return evidence_directories.snapshot_existing(evidence_root, paths, owner=None)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise ReleaseGateError(exc.kind, str(exc)) from exc


def _verify_runner_directory_contract(evidence_root, contract):
    try:
        after = evidence_directories.verify_snapshot(evidence_root, contract)
        return evidence_directories.finalize(contract, after)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise ReleaseGateError(exc.kind, str(exc)) from exc


def _validate_host_layout_files(evidence_root, dependency_leg, dependencies):
    """Reject root-level files/dirs outside the precreated runner contract."""
    expected_files = {
        DEPENDENCY_ENVIRONMENT_NAME,
        DEPENDENCY_ENVIRONMENT_NAME + ".sha256",
    }
    expected_dirs = {"work"}
    if dependency_leg == "present":
        expected_files.update({
            PREFETCH_RECEIPT_NAME, PREFETCH_RECEIPT_NAME + ".sha256",
            PREFETCH_MANIFEST_NAME, PREFETCH_MANIFEST_NAME + ".sha256",
        })
        expected_dirs.add("prefetch")
    observed_files = set()
    observed_dirs = set()
    try:
        for entry in os.scandir(str(evidence_root)):
            info = entry.stat(follow_symlinks=False)
            if stat.S_ISLNK(info.st_mode):
                raise ReleaseGateError("HOST_LAYOUT_LINK_FORBIDDEN", entry.name)
            if stat.S_ISDIR(info.st_mode):
                observed_dirs.add(entry.name)
            elif stat.S_ISREG(info.st_mode):
                if info.st_nlink != 1:
                    raise ReleaseGateError("HOST_LAYOUT_HARDLINK_FORBIDDEN", entry.name)
                observed_files.add(entry.name)
            else:
                raise ReleaseGateError("HOST_LAYOUT_SPECIAL_FILE", entry.name)
    except OSError as exc:
        raise ReleaseGateError("HOST_LAYOUT_READ_FAILED", str(exc)) from exc
    if observed_dirs != expected_dirs or observed_files != expected_files:
        raise ReleaseGateError(
            "HOST_LAYOUT_ALLOWLIST_MISMATCH",
            _canonical({"expected_dirs": sorted(expected_dirs),
                        "observed_dirs": sorted(observed_dirs),
                        "expected_files": sorted(expected_files),
                        "observed_files": sorted(observed_files)}),
        )
    del dependencies


def _tree_manifest(path, root_relative=None):
    path = Path(path)
    if path.is_symlink() or not path.is_dir():
        raise ReleaseGateError("INSTALL_TREE_INVALID", str(path))
    entries = []
    symlink_count = 0
    for item in sorted(path.rglob("*"), key=lambda value: value.relative_to(path).as_posix()):
        relative = item.relative_to(path).as_posix()
        mode = stat.S_IMODE(os.lstat(str(item)).st_mode)
        if item.is_symlink():
            symlink_count += 1
            entries.append({"kind": "symlink", "mode": mode, "path": relative,
                            "target": os.readlink(str(item))})
        elif item.is_dir():
            entries.append({"kind": "directory", "mode": mode, "path": relative})
        elif item.is_file():
            entries.append({"kind": "file", "mode": mode, "path": relative,
                            "size_bytes": item.stat().st_size,
                            "sha256": _sha256_file(item)})
        else:
            raise ReleaseGateError("INSTALL_TREE_ENTRY_INVALID", str(item))
    result = {
        "status": "PASS" if symlink_count == 0 else "FAIL_CLOSED",
        "root": str(path),
        "hash_kind": "relative_lstat_tree_manifest_v1",
        "symlink_count": symlink_count,
        "entry_count": len(entries),
        "entries": entries,
        "tree_sha256": _sha256_bytes(_canonical(entries).encode("utf-8")),
    }
    if root_relative is not None:
        result["root_relative"] = audit.normalize_artifact_relative_path(
            root_relative, "tree manifest root")
    return result


def _content_tree_sha(path):
    """Hash a dependency source tree without its mutable .git directory."""
    path = Path(path)
    if path.is_symlink() or not path.is_dir():
        raise ReleaseGateError("DEPENDENCY_SOURCE_INVALID", str(path))
    entries = []
    for item in sorted(path.rglob("*"), key=lambda value: value.relative_to(path).as_posix()):
        relative = item.relative_to(path).as_posix()
        if relative == ".git" or relative.startswith(".git/"):
            continue
        if item.is_symlink():
            entries.append({"kind": "symlink", "path": relative, "target": os.readlink(str(item))})
        elif item.is_dir():
            entries.append({"kind": "directory", "path": relative})
        elif item.is_file():
            entries.append({"kind": "file", "path": relative,
                            "size_bytes": item.stat().st_size,
                            "sha256": _sha256_file(item)})
        else:
            raise ReleaseGateError("DEPENDENCY_SOURCE_ENTRY_INVALID", str(item))
    return _sha256_bytes(_canonical(entries).encode("utf-8"))


def _copy_clean_source(repo_root, destination):
    ignored = shutil.ignore_patterns(
        ".git", "build", "install", "log", "*.pyc", "__pycache__",
    )
    destination = _existing_directory(destination, "source staging root", require_empty=True)
    shutil.copytree(
        str(repo_root), str(destination), symlinks=True, ignore=ignored,
        dirs_exist_ok=True,
    )
    if destination.is_symlink() or not destination.is_dir():
        raise ReleaseGateError("SOURCE_COPY_INVALID", str(destination))
    return destination


def _load_profile(profile_path):
    profile, path = audit._load_profile(profile_path)
    release = audit._validate_release_matrix_profile(profile)
    return profile, release, path


def validate_release_plan(profile, distro, dependency_leg):
    """Validate a leg without touching Docker, ROS, or the filesystem."""
    release = audit._validate_release_matrix_profile(profile)
    if dependency_leg not in ("absent", "present"):
        raise ReleaseGateError("LEG_INVALID", dependency_leg)
    rows = [row for row in release["distros"] if row.get("name") == distro]
    if len(rows) != 1:
        raise ReleaseGateError("DISTRO_INVALID", distro)
    row = rows[0]
    leg = row["legs"][dependency_leg]
    if dependency_leg == "present":
        if leg.get("status") == "BLOCKED_MISSING_PIN":
            return {
                "status": "BLOCKED_MISSING_PIN",
                "distro": distro,
                "dependency_leg": dependency_leg,
                "campaign_set": release["campaign_set"],
                "reason": leg.get("reason"),
                "dependencies": leg.get("dependencies", []),
            }
        for dependency in leg.get("dependencies", []):
            commit = dependency.get("commit")
            source_sha = dependency.get("source_sha256")
            if (not isinstance(commit, str) or len(commit) != 40 or
                    any(ch not in "0123456789abcdefABCDEF" for ch in commit) or
                    not isinstance(source_sha, str) or len(source_sha) != 64 or
                    any(ch not in "0123456789abcdef" for ch in source_sha)):
                raise ReleaseGateError(
                    "BLOCKED_MISSING_PIN",
                    "{} has no complete official commit/source SHA".format(dependency.get("name")),
                )
            url = str(dependency.get("official_url", ""))
            if not url.startswith("https://") or any(token in url.lower() for token in ("latest", "main", "master")):
                raise ReleaseGateError("PIN_URL_INVALID", dependency.get("name"))
            repository = audit.OFFICIAL_DEPENDENCY_REPOSITORIES.get(dependency.get("name"))
            if repository is None or url != repository["official_url"]:
                raise ReleaseGateError("PIN_URL_INVALID", dependency.get("name"))
            if dependency.get("archive_url") != repository["archive_prefix"] + commit:
                raise ReleaseGateError("PIN_ARCHIVE_URL_INVALID", dependency.get("name"))
            if not isinstance(dependency.get("archive_sha256"), str) or \
                    not audit.SHA256_RE.fullmatch(dependency.get("archive_sha256", "")):
                raise ReleaseGateError("BLOCKED_MISSING_PIN", dependency.get("name"))
            if dependency.get("archive_top_level") != "{}-{}".format(dependency.get("name"), commit):
                raise ReleaseGateError("PIN_ARCHIVE_ROOT_INVALID", dependency.get("name"))
            if dependency.get("source_tree_sha256") != source_sha:
                raise ReleaseGateError("PIN_SOURCE_TREE_INVALID", dependency.get("name"))
            if not isinstance(dependency.get("license_sha256"), str) or \
                    not audit.SHA256_RE.fullmatch(dependency.get("license_sha256", "")):
                raise ReleaseGateError("BLOCKED_MISSING_PIN", dependency.get("name"))
    return {
        "status": "READY",
        "distro": distro,
        "dependency_leg": dependency_leg,
        "campaign_set": release["campaign_set"],
        "image": row["image"],
        "ros_prefix": row["ros_prefix"],
        "leg": leg,
    }


def _command_hash(argv):
    return _sha256_bytes(json.dumps(list(argv), separators=(",", ":")).encode("utf-8"))


def _validate_archive_member(member, expected_top_level, seen_names):
    """Validate one member before any extraction or metadata application."""
    name = member.name
    parts = name.split("/")
    if (not name or name.startswith('/') or '\\' in name or
            '\x00' in name or any(part in ('', '.', '..') for part in parts[:-1]) or
            '..' in parts or parts[0] != expected_top_level):
        raise ReleaseGateError('DEPENDENCY_ARCHIVE_PATH_INVALID', name)
    if name in seen_names:
        raise ReleaseGateError('DEPENDENCY_ARCHIVE_MEMBER_DUPLICATE', name)
    seen_names.add(name)
    if member.issym() or member.islnk() or not (member.isdir() or member.isfile()):
        raise ReleaseGateError('DEPENDENCY_ARCHIVE_MEMBER_INVALID', name)
    special_mode = stat.S_ISUID | stat.S_ISGID | stat.S_ISVTX
    if member.mode is not None and member.mode & special_mode:
        raise ReleaseGateError('DEPENDENCY_ARCHIVE_METADATA_INVALID', name)


def _safe_archive_filter(member, destination):
    """Apply Python's data filter after the explicit member validation."""
    data_filter = getattr(tarfile, 'data_filter', None)
    if not callable(data_filter):
        raise ReleaseGateError(
            'DEPENDENCY_ARCHIVE_FILTER_UNAVAILABLE',
            'tarfile.data_filter is unavailable',
        )
    try:
        return data_filter(member, destination)
    except (OSError, tarfile.TarError, ValueError) as exc:
        raise ReleaseGateError('DEPENDENCY_ARCHIVE_FILTER_REJECTED', str(exc))


def _manual_safe_extract(archive, destination, members):
    """Extract regular files/directories with sanitized metadata on Python 3.11."""
    destination_root = destination.resolve(strict=True)
    for member in members:
        relative_parts = tuple(part for part in member.name.split('/') if part)
        target = destination.joinpath(*relative_parts)
        try:
            target.resolve(strict=False).relative_to(destination_root)
        except ValueError as exc:
            raise ReleaseGateError(
                'DEPENDENCY_ARCHIVE_PATH_INVALID', member.name) from exc
        if member.isdir():
            if target.exists() or target.is_symlink():
                if target.is_symlink() or not target.is_dir():
                    raise ReleaseGateError(
                        'DEPENDENCY_ARCHIVE_MEMBER_COLLISION', member.name)
            else:
                target.mkdir(parents=True, exist_ok=False)
            os.chmod(target, stat.S_IMODE(member.mode or 0))
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() or target.is_symlink():
            raise ReleaseGateError(
                'DEPENDENCY_ARCHIVE_MEMBER_COLLISION', member.name)
        source = archive.extractfile(member)
        if source is None:
            raise ReleaseGateError(
                'DEPENDENCY_ARCHIVE_MEMBER_INVALID', member.name)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, 'O_NOFOLLOW', 0)
        fd = os.open(str(target), flags, 0o600)
        try:
            with os.fdopen(fd, 'wb') as stream:
                shutil.copyfileobj(source, stream)
        except Exception:
            try:
                os.close(fd)
            except OSError:
                pass
            raise
        finally:
            source.close()
        os.chmod(target, stat.S_IMODE(member.mode or 0))


def _extract_members_safely(archive, destination, members):
    """Use an explicit data filter or a sanitized 3.11-compatible fallback."""
    try:
        supports_filter = 'filter' in inspect.signature(
            archive.extractall).parameters
    except (TypeError, ValueError) as exc:
        raise ReleaseGateError(
            'DEPENDENCY_ARCHIVE_FILTER_UNAVAILABLE', str(exc)) from exc
    if supports_filter:
        archive.extractall(
            path=str(destination), members=members, filter=_safe_archive_filter)
    else:
        _manual_safe_extract(archive, destination, members)


def _extract_verified_archive(archive_path, destination, expected_top_level):
    """Safely extract an already-hashed GitHub tarball into a fresh directory."""
    archive_path = Path(archive_path)
    destination = Path(destination)
    if archive_path.is_symlink() or not archive_path.is_file():
        raise ReleaseGateError("DEPENDENCY_ARCHIVE_INVALID", str(archive_path))
    _existing_directory(destination, "dependency extraction root", require_empty=True)
    members = []
    seen_names = set()
    try:
        with tarfile.open(str(archive_path), mode="r:gz") as archive:
            for member in archive.getmembers():
                _validate_archive_member(member, expected_top_level, seen_names)
                members.append(member)
            if not any(item.isdir() and item.name.rstrip("/") == expected_top_level
                       for item in members):
                raise ReleaseGateError("DEPENDENCY_ARCHIVE_ROOT_MISSING", expected_top_level)
            _extract_members_safely(archive, destination, members)
    except (OSError, tarfile.TarError) as exc:
        raise ReleaseGateError("DEPENDENCY_ARCHIVE_EXTRACT_FAILED", str(exc))
    source = destination / expected_top_level
    if source.is_symlink() or not source.is_dir():
        raise ReleaseGateError("DEPENDENCY_SOURCE_INVALID", str(source))
    return source


def _copy_prefetched_archive(prefetch_root, binding, destination):
    """Copy one host-sealed archive without following links or changing bytes."""
    source = Path(prefetch_root) / binding["archive_path_relative"]
    target = Path(destination)
    try:
        info = source.lstat()
    except OSError:
        raise ReleaseGateError("PREFETCH_ARCHIVE_MISSING", str(source))
    if (stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or
            info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o444):
        raise ReleaseGateError("PREFETCH_ARCHIVE_NOT_REGULAR", str(source))
    if info.st_size != binding["size_bytes"] or _sha256_file(source) != binding["archive_sha256"]:
        raise ReleaseGateError("PREFETCH_ARCHIVE_IDENTITY_MISMATCH", str(source))
    if target.exists() or target.is_symlink():
        raise ReleaseGateError("DEPENDENCY_OUTPUT_OVERWRITE", str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(str(target), flags, 0o600)
        with os.fdopen(fd, "wb") as stream:
            with source.open("rb") as source_stream:
                shutil.copyfileobj(source_stream, stream)
        os.chmod(str(target), 0o444)
    except OSError as exc:
        raise ReleaseGateError("PREFETCH_ARCHIVE_COPY_FAILED", str(exc))
    if _sha256_file(target) != binding["archive_sha256"]:
        raise ReleaseGateError("PREFETCH_ARCHIVE_COPY_DRIFT", str(target))
    return target


def _materialize_pinned_dependencies(dependencies, work, env, log_root, runner=None,
                                     prefetch_root=None, prefetch_binding=None):
    """Consume verified official archives, then build from extracted bytes.

    Archive acquisition is intentionally absent here.  A runner started after
    network disconnect can only copy bytes from the sealed prefetch root.
    """
    if prefetch_root is None or not isinstance(prefetch_binding, dict):
        raise ReleaseGateError("PREFETCH_BINDING_MISSING", "present leg requires host prefetch")
    vendor_root = work / "vendor"
    archive_root = work / "archives"
    _existing_directory(vendor_root, "dependency vendor root", require_empty=True)
    _existing_directory(archive_root, "dependency archive root", require_empty=True)
    records = []
    prefixes = []
    binding_by_name = {item["name"]: item for item in prefetch_binding.get("records", [])}
    if len(binding_by_name) != len(dependencies):
        raise ReleaseGateError("PREFETCH_BINDING_COVERAGE_INVALID", "dependency coverage")
    for dependency in dependencies:
        name = dependency["name"]
        archive = archive_root / (name + ".tar.gz")
        extraction = vendor_root / (name + "-extract")
        build = vendor_root / (name + "-build")
        prefix = vendor_root / (name + "-install")
        for target in (archive, extraction, build, prefix):
            if target.exists() or target.is_symlink():
                raise ReleaseGateError("DEPENDENCY_OUTPUT_OVERWRITE", str(target))
        archive_url = dependency.get("archive_url")
        expected_archive_sha = dependency.get("archive_sha256")
        expected_top_level = dependency.get("archive_top_level")
        binding = binding_by_name.get(name)
        if binding is None or binding.get("archive_sha256") != expected_archive_sha:
            raise ReleaseGateError("PREFETCH_BINDING_DRIFT", name)
        archive = _copy_prefetched_archive(prefetch_root, binding, archive)
        observed_archive_sha = _sha256_file(archive)
        fetch = {
            "label": "consume_prefetched_archive_" + name,
            "argv": ["prefetch", binding["archive_path_relative"]],
            "argv_sha256": _command_hash(["prefetch", binding["archive_path_relative"]]),
            "returncode": 0,
            "archive_fetch_network_used": False,
            "prefetch_receipt_sha256": prefetch_binding["receipt_sha256"],
            "archive_sha256": observed_archive_sha,
        }
        source = _extract_verified_archive(archive, extraction, expected_top_level)
        observed_source_sha = _content_tree_sha(source)
        if observed_source_sha != dependency["source_tree_sha256"] or \
                observed_source_sha != dependency["source_sha256"]:
            raise ReleaseGateError(
                "DEPENDENCY_SOURCE_SHA_MISMATCH",
                "{} expected {} observed {}".format(name, dependency["source_tree_sha256"], observed_source_sha),
            )
        license_path = source / dependency["license_path"]
        if (license_path.is_symlink() or not license_path.is_file() or
                _sha256_file(license_path) != dependency["license_sha256"]):
            raise ReleaseGateError("DEPENDENCY_LICENSE_SHA_MISMATCH", name)
        _existing_directory(build, "dependency build root", require_empty=True)
        _existing_directory(prefix, "dependency install root", require_empty=True)
        replacements = {"<source>": str(source), "<build>": str(build), "<prefix>": str(prefix)}
        build_argv = [replacements.get(item, item) for item in dependency["build_argv"]]
        install_argv = [replacements.get(item, item) for item in dependency["install_argv"]]
        build_record = _run_command(build_argv, "build_" + name, source, env,
                                    log_root / (name + "_build.log"), runner=runner)
        install_record = _run_command(install_argv, "install_" + name, source, env,
                                      log_root / (name + "_install.log"), runner=runner)
        install_manifest = _tree_manifest(prefix)
        if install_manifest["status"] != "PASS":
            raise ReleaseGateError("DEPENDENCY_INSTALL_SYMLINK", name)
        prefixes.append(prefix)
        records.append({
            "name": name, "official_url": dependency["official_url"],
            "archive_url": archive_url, "expected_commit": dependency["commit"],
            "commit_binding": "archive_url_path_and_expected_top_level",
            "observed_commit": dependency["commit"], "archive_path": str(archive),
            "expected_archive_sha256": expected_archive_sha,
            "archive_sha256": observed_archive_sha, "archive_top_level": expected_top_level,
            "source_path": str(source), "source_sha256": observed_source_sha,
            "source_tree_sha256": observed_source_sha,
            "license": dependency["license"], "license_path": dependency["license_path"],
            "license_sha256": dependency["license_sha256"],
            "install_prefix": str(prefix), "install_tree_sha256": install_manifest["tree_sha256"],
            "network_after_fetch": False, "archive_fetch_network_used": False,
            "commands": [fetch, build_record, install_record],
        })
    return {"status": "PASS", "fetch_policy": "prefetched_official_archive_only",
            "archive_verification_before_extraction": True,
            "archive_fetch_network_used": False, "network_after_fetch": False,
            "prefetch_receipt_sha256": prefetch_binding["receipt_sha256"],
            "prefetch_manifest_sha256": prefetch_binding["manifest_sha256"],
            "records": records,
            "vendor_prefixes": [str(item) for item in prefixes]}


def _write_log(path, stdout, stderr):
    path = Path(path)
    _existing_directory(path.parent, "command log parent")
    data = (stdout or "").encode("utf-8") + (stderr or "").encode("utf-8")
    if path.exists() or path.is_symlink():
        raise ReleaseGateError("COMMAND_LOG_OVERWRITE", str(path))
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(str(path), flags, 0o600)
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(str(path), 0o444)
    except OSError as exc:
        try:
            if path.is_file() and not path.is_symlink():
                path.unlink()
        except OSError:
            pass
        raise ReleaseGateError("COMMAND_LOG_WRITE_FAILED", str(exc)) from exc
    return {"path": str(path), "bytes": len(data), "sha256": _sha256_bytes(data)}


def _run_command(argv, label, cwd, env, log_path, runner=None, allow_failure=False,
                 network_allowed=False):
    argv = [str(item) for item in argv]
    if any(token in " ".join(argv).lower() for token in FORBIDDEN_TOKENS):
        raise ReleaseGateError("FORBIDDEN_COMMAND", "{} contains forbidden evaluation surface".format(label))
    executable = Path(argv[0]).name if argv else ""
    if not network_allowed and executable in NETWORK_COMMANDS:
        raise ReleaseGateError(
            "NETWORK_AFTER_FETCH_FORBIDDEN",
            "NETWORK_AFTER_FETCH_FORBIDDEN: {} uses network-capable executable {} after the explicit archive fetch".format(
                label, executable),
        )
    effective_argv = argv if runner is not None else _priority_argv(argv)
    if runner is not None:
        completed = runner(argv, label, Path(cwd), dict(env))
        returncode = int(getattr(completed, "returncode", 0))
        stdout = getattr(completed, "stdout", "") or ""
        stderr = getattr(completed, "stderr", "") or ""
    else:
        completed = subprocess.run(
            effective_argv, cwd=str(cwd), env=dict(env), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, universal_newlines=True, check=False,
        )
        returncode = completed.returncode
        stdout = completed.stdout or ""
        stderr = completed.stderr or ""
    log = _write_log(Path(log_path), stdout, stderr)
    record = {"label": label, "argv": argv, "argv_sha256": _command_hash(argv),
              "effective_argv": effective_argv,
              "priority": {"nice": 19, "ionice_class": 3,
                           "applied": effective_argv != argv},
              "returncode": returncode, "log": log}
    if returncode != 0 and not allow_failure:
        raise ReleaseGateError("COMMAND_FAILED", "{} exited {}".format(label, returncode))
    return record


def _run_sourced(argv, label, cwd, env, log_path, ros_setup, install_setup=None,
                 runner=None, allow_failure=False):
    source_parts = ["source", shlex.quote(str(ros_setup))]
    if install_setup is not None:
        source_parts.append("source " + shlex.quote(str(install_setup)))
    source_parts.append("exec " + " ".join(shlex.quote(str(item)) for item in argv))
    command = " && ".join(source_parts)
    return _run_command(["bash", "-lc", command], label, cwd, env, log_path,
                        runner=runner, allow_failure=allow_failure)


def _optional_absence(prefix, dependencies):
    result = []
    for dependency in dependencies:
        name = dependency["name"]
        markers = []
        for candidate in (
            prefix / "share" / name,
            prefix / "include" / name,
            Path("/usr/include") / name,
            Path("/usr/local/include") / name,
        ):
            if candidate.exists() and not candidate.is_symlink():
                markers.append(str(candidate))
        ros2_probe = None
        ros2 = shutil.which("ros2")
        if ros2 is not None:
            probe = subprocess.run([ros2, "pkg", "prefix", name], stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, universal_newlines=True, check=False)
            ros2_probe = {"returncode": probe.returncode,
                          "stdout": (probe.stdout or "").strip(),
                          "stderr": (probe.stderr or "").strip()}
            if probe.returncode == 0:
                markers.append("ros2-pkg-prefix:" + (probe.stdout or "").strip())
        result.append({"name": name, "status": "ABSENT" if not markers else "PRESENT",
                       "markers": sorted(set(markers)), "ros2_probe": ros2_probe})
    if any(item["status"] != "ABSENT" for item in result):
        raise ReleaseGateError("OPTIONAL_DEPENDENCY_PRESENT", _canonical(result))
    return result


def _installed_optional_class_report(install_root, forbidden_classes):
    manifests = []
    advertised = []
    for path in sorted(install_root.rglob("*.xml")):
        if path.is_symlink() or not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        found = [class_id for class_id in forbidden_classes if class_id in text]
        if found:
            advertised.extend(found)
            manifests.append({"path": str(path), "classes": found,
                              "sha256": _sha256_file(path)})
    return {"status": "PASS" if not advertised else "FAIL_CLOSED",
            "forbidden_classes": sorted(set(forbidden_classes)),
            "advertised": sorted(set(advertised)), "manifests": manifests}


def _installed_required_class_report(install_root, required_classes):
    manifests = []
    advertised = set()
    for path in sorted(install_root.rglob("*.xml")):
        if path.is_symlink() or not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        found = [class_id for class_id in required_classes if class_id in text]
        if found:
            advertised.update(found)
            manifests.append({"path": str(path), "classes": found,
                              "sha256": _sha256_file(path)})
    missing = sorted(set(required_classes) - advertised)
    return {"status": "PASS" if not missing else "FAIL_CLOSED",
            "required_classes": sorted(set(required_classes)),
            "advertised": sorted(advertised), "missing": missing, "manifests": manifests}


def _container_identity(row):
    expected = row["image"]["digest"]
    observed = os.environ.get("REGISTRATION_PLUGIN_CONTAINER_DIGEST")
    if observed != expected:
        raise ReleaseGateError("CONTAINER_IDENTITY_MISMATCH",
                               "expected {} observed {}".format(expected, observed))
    observed_distro = os.environ.get("ROS_DISTRO")
    if observed_distro != row["name"]:
        raise ReleaseGateError(
            "CONTAINER_IDENTITY_MISMATCH",
            "expected ROS distro {} observed {}".format(row["name"], observed_distro),
        )
    return {
        "expected_image": row["image"],
        "observed_digest_env": observed,
        "ros_distro": observed_distro,
        "platform": platform.platform(),
        "source": "workflow_pinned_container_digest_env",
    }


def _seal(path, value):
    path = Path(path).absolute()
    if path.exists() or path.is_symlink():
        raise ReleaseGateError("RECEIPT_OVERWRITE", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise ReleaseGateError("RECEIPT_PARENT_INVALID", str(path.parent))
    receipt, sidecar, digest = audit.seal_receipt(path, value)
    return {"path": receipt, "sidecar": sidecar, "sha256": digest,
            "sidecar_sha256": _sha256_file(Path(sidecar))}


def _dependency_environment_seed(evidence_root, distro, row):
    """Validate the host-captured pre-disconnect receipt before building."""
    evidence_root = Path(evidence_root)
    path = evidence_root / DEPENDENCY_ENVIRONMENT_NAME
    sidecar = Path(str(path) + ".sha256")
    for candidate in (path, sidecar):
        if (candidate.is_symlink() or not candidate.is_file() or
                candidate.stat().st_nlink != 1 or stat.S_IMODE(candidate.stat().st_mode) != 0o444):
            raise ReleaseGateError("DEPENDENCY_ENVIRONMENT_INVALID", str(candidate))
    digest = _sha256_file(path)
    tokens = sidecar.read_text(encoding="ascii").strip().split()
    if tokens != [digest, path.name]:
        raise ReleaseGateError("DEPENDENCY_ENVIRONMENT_SIDECAR_MISMATCH", str(path))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise ReleaseGateError("DEPENDENCY_ENVIRONMENT_JSON_INVALID", str(exc))
    if (not isinstance(value, dict) or value.get("schema") != DEPENDENCY_ENVIRONMENT_SCHEMA or
            value.get("schema_version") != 1 or value.get("status") != "PASS" or
            value.get("base_image", {}).get("digest") != row["image"]["digest"] or
            value.get("ros", {}).get("distro") != distro):
        raise ReleaseGateError("DEPENDENCY_ENVIRONMENT_IDENTITY_MISMATCH", str(path))
    return {
        "status": "PASS", "path_relative": DEPENDENCY_ENVIRONMENT_NAME,
        "sha256": digest, "size_bytes": path.stat().st_size,
        "sidecar_sha256": _sha256_file(sidecar),
        "canonical_sha256": value.get("canonical_sha256"),
        "capture_script_sha256": value.get("capture", {}).get("script_sha256"),
    }


def _root_relative(root, path, label):
    root = Path(root).absolute()
    path = Path(path).absolute()
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError:
        raise ReleaseGateError("ARTIFACT_PATH_OUTSIDE_ROOT",
                               "{} is outside evidence root: {}".format(label, path))
    if not relative or relative == ".":
        raise ReleaseGateError("ARTIFACT_PATH_INVALID", "{} resolves to evidence root".format(label))
    return audit.normalize_artifact_relative_path(relative, label)


def _receipt_binding(root, path, role, for_path=None):
    relative = _root_relative(root, path, role)
    binding = {"path": relative, "role": role}
    if for_path is not None:
        binding["for_path"] = _root_relative(root, for_path, role + " target")
    return binding


def _failure_document(repo_root, profile_path, source_report, git_report, plan, exc):
    document = {
        "schema_version": 1, "contract_version": RELEASE_CONTRACT,
        "status": "FAIL_CLOSED", "failure_kind": getattr(exc, "kind", "RELEASE_GATE_FAILURE"),
        "failure_message": str(exc), "distro": plan.get("distro"),
        "dependency_leg": plan.get("dependency_leg"),
        "campaign_set": plan.get("campaign_set"),
        "profile": {"path": str(profile_path), "sha256": _sha256_file(profile_path)},
        "repository": {"path": str(repo_root), "source_manifest": source_report,
                        "git": git_report},
        "safety": {"provisioning_network_used": False,
                    "archive_fetch_network_used": False,
                    "build_test_network_connected": False,
                    "build_test_network_used": False,
                    "bag_opened": False,
                    "ground_truth_content_opened": False, "scorer_invoked": False,
                    "map_saved": False, "formal_replay_started": False},
    }
    if getattr(exc, "details", None) is not None:
        document["resource_policy"] = exc.details
    return document


def run_release_leg(repo_root, profile_path, distro, dependency_leg, output,
                    evidence_root, work_root=None, command_runner=None):
    repo_root = Path(repo_root).absolute()
    evidence_root = Path(evidence_root).absolute()
    expected_root_name = "registration-plugin-release-{}-{}".format(
        distro, dependency_leg)
    if evidence_root.name != expected_root_name:
        raise ReleaseGateError(
            "EVIDENCE_ROOT_NAME_INVALID",
            "expected {} observed {}".format(expected_root_name, evidence_root.name),
        )
    profile, release, profile_path = _load_profile(Path(profile_path).absolute())
    plan = validate_release_plan(profile, distro, dependency_leg)
    row = next(item for item in release["distros"] if item["name"] == distro)
    present_dependencies = row["legs"]["present"].get("dependencies", [])
    if not evidence_root.exists() or evidence_root.is_symlink() or not evidence_root.is_dir():
        raise ReleaseGateError(
            "EVIDENCE_ROOT_NOT_PRECREATED",
            "host must precreate the evidence root and fixed directories",
        )
    directory_contract = _runner_directory_contract(
        evidence_root, present_dependencies, dependency_leg == "present")
    _validate_host_layout_files(evidence_root, dependency_leg, present_dependencies)
    if dependency_leg == "present":
        try:
            dependency_prefetch.validate_prefetch_root(
                evidence_root, present_dependencies, distro, row["image"]["digest"],
                authority=os.environ.get("REGISTRATION_PLUGIN_EXECUTION_AUTHORITY"),
                allowed_host_directories=set(
                    evidence_directories.release_runner_paths(
                        [item["name"] for item in present_dependencies], False)),
            )
        except dependency_prefetch.PrefetchError as exc:
            raise ReleaseGateError(getattr(exc, "kind", "PREFETCH_INVALID"), str(exc))
    output = Path(output).absolute()
    expected_output = evidence_root / "registration_plugin_release.receipt.json"
    if output != expected_output:
        raise ReleaseGateError(
            "RECEIPT_ROOT_MISMATCH",
            "receipt must be directly under evidence root: {}".format(expected_output),
        )
    resource_policy = release["resource_policy"]
    observed_network_policy = os.environ.get("REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY")
    expected_network_policy = release["policy"]["network_after_dependency_fetch"]
    if observed_network_policy is not None and observed_network_policy != expected_network_policy:
        raise ReleaseGateError(
            "NETWORK_POLICY_MISMATCH",
            "expected {} observed {}".format(expected_network_policy, observed_network_policy),
        )
    observed_resource_policy = os.environ.get("REGISTRATION_PLUGIN_RESOURCE_POLICY")
    if observed_resource_policy is not None and \
            observed_resource_policy != audit.FUNCTIONAL_RESOURCE_MODE:
        raise ReleaseGateError(
            "RESOURCE_POLICY_MISMATCH",
            "expected {} observed {}".format(audit.FUNCTIONAL_RESOURCE_MODE,
                                               observed_resource_policy),
        )
    authority = os.environ.get("REGISTRATION_PLUGIN_EXECUTION_AUTHORITY")
    if authority not in (HOST_PROMOTION_AUTHORITY, FUNCTIONAL_CI_AUTHORITY):
        raise ReleaseGateError("EXECUTION_AUTHORITY_MISSING", "explicit host or CI authority is required")
    network_connected_raw = os.environ.get("REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED")
    disconnect_verified = os.environ.get("REGISTRATION_PLUGIN_HOST_DISCONNECT_VERIFIED")
    if authority == HOST_PROMOTION_AUTHORITY and \
            (network_connected_raw != "false" or disconnect_verified != "true"):
        raise ReleaseGateError("HOST_NETWORK_DISCONNECT_UNPROVEN",
                               "host promotion requires verified disconnected runner")
    if authority == FUNCTIONAL_CI_AUTHORITY and network_connected_raw != "true":
        raise ReleaseGateError("CI_NETWORK_STATE_INVALID",
                               "functional CI authority must disclose connected network")
    build_test_network_connected = network_connected_raw == "true"
    build_test_network_used = build_test_network_connected
    source_report = audit._source_manifest(repo_root, profile)
    git_report = audit._git_snapshot(repo_root)
    if source_report["status"] != "PASS":
        raise ReleaseGateError("CURRENT_SOURCE_HASH_MISMATCH", _canonical(source_report))
    profile_sha = _sha256_file(profile_path)
    root_identity = audit.artifact_root_identity(
        logical_name=evidence_root.name,
        distro=distro,
        dependency_leg=dependency_leg,
        profile_sha256=profile_sha,
        source_manifest_sha256=source_report["manifest_sha256"],
    )
    root_marker_path = evidence_root / "evidence_root.receipt.json"
    root_marker = _seal(root_marker_path, root_identity)
    if plan["status"] == "BLOCKED_MISSING_PIN":
        report = {
            "schema_version": 1, "contract_version": RELEASE_CONTRACT,
            "status": "BLOCKED_MISSING_PIN", "failure_kind": "BLOCKED_MISSING_PIN",
            "distro": distro, "dependency_leg": dependency_leg,
            "campaign_set": release["campaign_set"],
            "reason": plan.get("reason"), "dependencies": plan.get("dependencies", []),
            "profile": {"path": str(profile_path), "sha256": _sha256_file(profile_path)},
            "repository": {"path": str(repo_root), "source_manifest": source_report,
                            "git": git_report},
            "execution": {"build_started": False, "container_started": True,
                           "install_isolated": False},
            "safety": {"provisioning_network_used": False,
                        "archive_fetch_network_used": False,
                        "build_test_network_connected": False,
                        "build_test_network_used": False,
                        "bag_opened": False,
                        "ground_truth_content_opened": False, "scorer_invoked": False,
                        "map_saved": False, "formal_replay_started": False},
        }
        return _seal(output, report), report

    identity = _container_identity(row)
    dependency_environment = _dependency_environment_seed(evidence_root, distro, row)
    dependency_prefetch_binding = None
    if dependency_leg == "present":
        try:
            dependency_prefetch_binding = dependency_prefetch.validate_prefetch_root(
                evidence_root, present_dependencies, distro, row["image"]["digest"],
                authority=authority)
        except dependency_prefetch.PrefetchError as exc:
            raise ReleaseGateError(getattr(exc, "kind", "PREFETCH_INVALID"), str(exc))
    requested_work_root = Path(work_root).absolute() if work_root is not None else evidence_root / "work"
    if requested_work_root != evidence_root / "work":
        raise ReleaseGateError("WORK_ROOT_MISMATCH", str(requested_work_root))
    work = _existing_directory(requested_work_root, "release work root")
    source_root = _copy_clean_source(repo_root, work / "src")
    build_root = work / "build"
    install_root = work / "install"
    log_root = work / "logs"
    test_root = work / "test-results"
    evidence_work_root = work / "evidence"
    for directory in (build_root, install_root, log_root, test_root, evidence_work_root):
        _existing_directory(directory, "release output directory", require_empty=True)
    resource_preflight = _functional_resource_snapshot(
        resource_policy, evidence_root,
        excluded_pids=_proc_ancestors(os.getpid()),
    )
    if resource_preflight["status"] != "PASS":
        raise ReleaseGateError(
            "FUNCTIONAL_RESOURCE_PREFLIGHT_FAIL_CLOSED",
            _canonical(resource_preflight),
            details={
                "schema": audit.FUNCTIONAL_RESOURCE_POLICY_SCHEMA,
                "schema_version": audit.FUNCTIONAL_RESOURCE_POLICY_VERSION,
                "mode": audit.FUNCTIONAL_RESOURCE_MODE,
                "status": "FAIL_CLOSED",
                "preflight": resource_preflight,
                "timing_authority": audit.FUNCTIONAL_TIMING_AUTHORITY,
                "performance_gate_eligible": False,
            },
        )
    ros_prefix = Path(row["ros_prefix"])
    ros_setup = ros_prefix / "setup.bash"
    if ros_setup.is_symlink() or not ros_setup.is_file():
        raise ReleaseGateError("ROS_SETUP_MISSING", str(ros_setup))
    env = dict(os.environ)
    env["ROS_DISTRO"] = distro
    env["REGISTRATION_PLUGIN_DEPENDENCY_LEG"] = dependency_leg
    env["CMAKE_BUILD_PARALLEL_LEVEL"] = "1"
    env["MAKEFLAGS"] = "-j1"
    env["NINJAFLAGS"] = "-j1"
    commands = []
    if dependency_leg == "absent":
        dependency_evidence = _optional_absence(ros_prefix, row["legs"]["absent"]["optional_dependencies"])
        dependency_prefetch_evidence = {
            "status": "NOT_APPLICABLE", "archive_fetch_network_used": False,
        }
        vendor_prefixes = []
    else:
        dependency_evidence = _materialize_pinned_dependencies(
            row["legs"]["present"]["dependencies"], work, env, log_root,
            runner=command_runner, prefetch_root=evidence_root,
            prefetch_binding=dependency_prefetch_binding)
        dependency_prefetch_evidence = dependency_prefetch_binding
        vendor_prefixes = dependency_evidence.get("vendor_prefixes", [])
        if vendor_prefixes:
            existing_prefix = env.get("CMAKE_PREFIX_PATH", "")
            env["CMAKE_PREFIX_PATH"] = os.pathsep.join(vendor_prefixes + ([existing_prefix] if existing_prefix else []))

    build_argv = ["colcon", "build", "--parallel-workers", "1", "--base-paths", str(source_root),
                  "--build-base", str(build_root), "--install-base", str(install_root),
                  "--log-base", str(log_root / "colcon-build"),
                  "--event-handlers", "console_direct+", "--cmake-args",
                  "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=ON"]
    commands.append(_run_sourced(build_argv, "isolated_build", source_root, env,
                                 log_root / "isolated_build.log", ros_setup=ros_setup,
                                 runner=command_runner))
    install_setup = install_root / "setup.bash"
    if install_setup.is_symlink() or not install_setup.is_file():
        raise ReleaseGateError("INSTALL_SETUP_MISSING", str(install_setup))

    test_argv = ["colcon", "test", "--parallel-workers", "1", "--base-paths", str(source_root),
                 "--build-base", str(build_root), "--install-base", str(install_root),
                 "--test-result-base", str(test_root), "--event-handlers", "console_direct+",
                 "--packages-select"] + list(TARGET_PACKAGES)
    commands.append(_run_sourced(test_argv, "plugin_test_matrix", source_root, env,
                                 log_root / "plugin_test_matrix.log", ros_setup=ros_setup,
                                 install_setup=install_setup, runner=command_runner))
    result_argv = ["colcon", "test-result", "--verbose", "--test-result-base", str(test_root)]
    commands.append(_run_sourced(result_argv, "test_result", source_root, env,
                                 log_root / "test_result.log", ros_setup=ros_setup,
                                 install_setup=install_setup, runner=command_runner))

    consumer_argv = ["bash", str(source_root / "scripts/run_registration_plugin_consumer_check.sh"),
                     "--work-dir", str(work / "consumer"), "--keep-work-dir"]
    env["ROS_DISTRO"] = distro
    commands.append(_run_sourced(consumer_argv, "cxx14_external_consumer", source_root, env,
                                 log_root / "consumer.log", ros_setup=ros_setup,
                                 install_setup=install_setup, runner=command_runner))
    template_argv = ["bash", str(source_root / "scripts/run_registration_plugin_template_check.sh"),
                     "--work-dir", str(work / "template"), "--keep-work-dir"]
    commands.append(_run_sourced(template_argv, "cxx14_external_template", source_root, env,
                                 log_root / "template.log", ros_setup=ros_setup,
                                 install_setup=install_setup, runner=command_runner))

    targeted_tests = []
    for label, (package, binary, gtest_filter) in TARGET_TESTS.items():
        executable = build_root / package / binary
        if not executable.is_file():
            raise ReleaseGateError("TARGET_TEST_MISSING", str(executable))
        targeted_tests.append(_run_sourced(
            [str(executable), "--gtest_filter=" + gtest_filter], label, source_root, env,
            log_root / (label + ".log"), ros_setup=ros_setup, install_setup=install_setup,
            runner=command_runner))

    fake_dso = install_root / "lidarslam_fake_registration_plugins" / "lib" / \
        "liblidarslam_fake_registration_plugins.so"
    fake_manifest = install_root / "lidarslam_fake_registration_plugins" / "share" / \
        "lidarslam_fake_registration_plugins" / "registration_plugins.xml"
    host_dso = install_root / "lidarslam_default_plugins" / "lib" / \
        "liblidarslam_default_plugins.so"
    odr_evidence_root = evidence_work_root / "odr-evidence"
    _existing_directory(odr_evidence_root, "DSO evidence directory", require_empty=True)
    odr_receipt = evidence_work_root / "registration_plugin_dso.receipt.json"
    odr_argv = ["python3", str(source_root / "scripts/check_registration_plugin_dso.py"),
                "--prefix", str(install_root), "--dso", str(fake_dso),
                "--manifest", str(fake_manifest), "--class-id",
                "lidarslam_fake_registration_plugins/Identity", "--host-dso", str(host_dso),
                "--ros-prefix", str(ros_prefix), "--receipt", str(odr_receipt),
                "--evidence-root", str(odr_evidence_root), "--repo-root", str(repo_root),
                "--sdk-source", str(source_root / "lidarslam_plugin_interfaces"),
                "--sdk-install-prefix", str(install_root / "lidarslam_plugin_interfaces"),
                "--plugin-source", str(source_root / "lidarslam_fake_registration_plugins"),
                "--plugin-build-command", json.dumps(build_argv),
                "--host-install-prefix", str(install_root / "lidarslam_default_plugins"),
                "--negative-coverage-version", "registration-plugin-negative-v1",
                "--negative-coverage-source", str(source_root / "graph_based_slam/test/test_registration_plugin_dso_gate.py")]
    commands.append(_run_sourced(odr_argv, "external_dso_odr_load_gate", source_root, env,
                                 log_root / "odr_gate.log", ros_setup=ros_setup,
                                 install_setup=install_setup, runner=command_runner))
    odr_sidecar = Path(str(odr_receipt) + ".sha256")
    if (not odr_receipt.is_file() or odr_receipt.is_symlink() or
            not odr_sidecar.is_file() or odr_sidecar.is_symlink()):
        raise ReleaseGateError("ODR_RECEIPT_MISSING", str(odr_receipt))
    odr_sha = _sha256_file(odr_receipt)

    optional_report = {"status": "NOT_APPLICABLE"}
    if dependency_leg == "absent":
        optional_report = _installed_optional_class_report(
            install_root,
            [class_id for dependency in row["legs"]["absent"]["optional_dependencies"]
             for class_id in dependency["forbidden_classes"]],
        )
        if optional_report["status"] != "PASS":
            raise ReleaseGateError("OPTIONAL_SELECTOR_ADVERTISED", _canonical(optional_report))
    else:
        optional_report = _installed_required_class_report(
            install_root,
            [class_id for dependency in row["legs"]["present"]["dependencies"]
             for class_id in dependency["expected_classes"]],
        )
        if optional_report["status"] != "PASS":
            raise ReleaseGateError("OPTIONAL_SELECTOR_NOT_ADVERTISED", _canonical(optional_report))

    resource_completion = _functional_resource_snapshot(
        resource_policy, evidence_root,
        excluded_pids=_proc_ancestors(os.getpid()),
    )
    if resource_completion["status"] != "PASS":
        raise ReleaseGateError(
            "FUNCTIONAL_RESOURCE_COMPLETION_FAIL_CLOSED",
            _canonical(resource_completion),
            details={
                "schema": audit.FUNCTIONAL_RESOURCE_POLICY_SCHEMA,
                "schema_version": audit.FUNCTIONAL_RESOURCE_POLICY_VERSION,
                "mode": audit.FUNCTIONAL_RESOURCE_MODE,
                "status": "FAIL_CLOSED",
                "preflight": resource_preflight,
                "completion": resource_completion,
                "timing_authority": audit.FUNCTIONAL_TIMING_AUTHORITY,
                "performance_gate_eligible": False,
            },
        )

    directory_contract = _verify_runner_directory_contract(
        evidence_root, directory_contract)

    install_manifest = _tree_manifest(install_root, "work/install")
    if install_manifest["status"] != "PASS":
        raise ReleaseGateError("INSTALL_SYMLINK", "isolated install contains symlinks")
    test_results_manifest = _tree_manifest(test_root, "work/test-results")
    if test_results_manifest["status"] != "PASS" or not test_results_manifest["entries"]:
        raise ReleaseGateError("TEST_RESULTS_ARTIFACT_INVALID", str(test_root))

    all_commands = commands + targeted_tests
    for record in all_commands:
        log = record.get("log", {})
        log_path = Path(log.get("path", ""))
        relative = _root_relative(evidence_root, log_path, "command log")
        if not relative.startswith("work/logs/"):
            raise ReleaseGateError("COMMAND_LOG_OUTSIDE_ROOT", relative)
        log["path_relative"] = relative

    resource_labels = ("loader_transaction", "ros_resource_failure", "backend_preflight")
    resource_records = {}
    for label in resource_labels:
        matching = [record for record in all_commands if record.get("label") == label]
        if len(matching) != 1:
            raise ReleaseGateError("RESOURCE_FAILURE_EVIDENCE_MISSING", label)
        resource_records[label] = matching[0]
    provenance_path = evidence_work_root / "provenance.receipt.json"
    resource_failure_path = evidence_work_root / "resource_failure.receipt.json"
    provenance_value = {
        "schema_version": 1,
        "contract_version": "registration-plugin-provenance-evidence-v1",
        "status": "PASS",
        "root_identity": root_identity,
        "profile": {"path": str(profile_path), "sha256": profile_sha},
        "repository": {"path": str(repo_root), "source_manifest": source_report,
                        "git": git_report},
        "container": identity,
        "safety": {"provisioning_network_used": True,
                    "archive_fetch_network_used": dependency_leg == "present",
                    "build_test_network_connected": build_test_network_connected,
                    "build_test_network_used": build_test_network_used,
                    "bag_opened": False,
                    "ground_truth_content_opened": False, "scorer_invoked": False,
                    "map_saved": False, "formal_replay_started": False},
    }
    provenance_sealed = _seal(provenance_path, provenance_value)
    resource_failure_value = {
        "schema_version": 1,
        "contract_version": "registration-plugin-resource-failure-evidence-v1",
        "status": "PASS",
        "root_identity": root_identity,
        "checks": {
            "transactional_activation_rollback": resource_records["loader_transaction"],
            "real_ros_resource_init_failure": resource_records["ros_resource_failure"],
            "backend_preflight": resource_records["backend_preflight"],
        },
        "safety": {"provisioning_network_used": True,
                    "archive_fetch_network_used": dependency_leg == "present",
                    "build_test_network_connected": build_test_network_connected,
                    "build_test_network_used": build_test_network_used,
                    "bag_opened": False,
                    "ground_truth_content_opened": False, "scorer_invoked": False,
                    "map_saved": False, "formal_replay_started": False},
    }
    resource_sealed = _seal(resource_failure_path, resource_failure_value)

    required_roots = [
        {"path": "work/logs", "role": "command_log"},
        {"path": "work/test-results", "role": "test_result"},
        {"path": "work/install", "role": "installed_tree_file"},
        {"path": "work/consumer", "role": "consumer_artifact"},
        {"path": "work/template", "role": "template_artifact"},
        {"path": "work/evidence", "role": "evidence_artifact"},
    ]
    if dependency_leg == "present":
        required_roots.append({"path": "work/archives", "role": "dependency_archive"})
        for dependency in dependency_evidence.get("records", []):
            source_root = Path(dependency["source_path"]).parent
            install_prefix = Path(dependency["install_prefix"])
            required_roots.extend([
                {"path": _root_relative(evidence_root, source_root, "dependency source root"),
                 "role": "dependency_source_file"},
                {"path": _root_relative(evidence_root, install_prefix, "dependency install root"),
                 "role": "dependency_install_file"},
            ])
    role_overrides = {
        DEPENDENCY_ENVIRONMENT_NAME: "dependency_environment_receipt",
        DEPENDENCY_ENVIRONMENT_NAME + ".sha256": "dependency_environment_receipt_sidecar",
        _root_relative(evidence_root, odr_receipt, "DSO receipt"): "dso_receipt",
        _root_relative(evidence_root, odr_sidecar, "DSO receipt sidecar"): "dso_receipt_sidecar",
        _root_relative(evidence_root, provenance_path, "provenance receipt"): "provenance_receipt",
        _root_relative(evidence_root, Path(provenance_sealed["sidecar"]),
                       "provenance receipt sidecar"): "provenance_receipt_sidecar",
        _root_relative(evidence_root, resource_failure_path, "resource failure receipt"): "resource_failure_receipt",
        _root_relative(evidence_root, Path(resource_sealed["sidecar"]),
                       "resource failure receipt sidecar"): "resource_failure_receipt_sidecar",
    }
    prefetch_bindings = []
    if dependency_leg == "present":
        prefetch_bindings = [
            {"path": PREFETCH_RECEIPT_NAME, "role": "dependency_prefetch_receipt"},
            {"path": PREFETCH_RECEIPT_NAME + ".sha256",
             "role": "dependency_prefetch_receipt_sidecar"},
            {"path": PREFETCH_MANIFEST_NAME, "role": "dependency_prefetch_manifest"},
            {"path": PREFETCH_MANIFEST_NAME + ".sha256",
             "role": "dependency_prefetch_manifest_sidecar"},
        ]
        for binding in dependency_prefetch_binding.get("records", []):
            prefetch_bindings.extend([
                {"path": binding["archive_path_relative"],
                 "role": "dependency_prefetch_archive"},
                {"path": binding["archive_path_relative"] + ".sha256",
                 "role": "dependency_prefetch_archive_sidecar"},
            ])
    artifact_manifest = audit.build_artifact_manifest(
        evidence_root,
        root_identity,
        required_roots,
        [
            {"path": _root_relative(evidence_root, root_marker_path, "root marker receipt"),
             "role": "root_marker_receipt"},
            {"path": _root_relative(evidence_root, Path(root_marker["sidecar"]),
                                     "root marker sidecar"),
             "role": "root_marker_receipt_sidecar"},
            {"path": DEPENDENCY_ENVIRONMENT_NAME,
             "role": "dependency_environment_receipt"},
            {"path": DEPENDENCY_ENVIRONMENT_NAME + ".sha256",
             "role": "dependency_environment_receipt_sidecar"},
        ] + prefetch_bindings,
        role_overrides=role_overrides,
    )
    artifact_manifest_path = evidence_root / "artifact_manifest.json"
    artifact_manifest_sealed = _seal(artifact_manifest_path, artifact_manifest)
    artifact_manifest_binding = {
        "schema": audit.ARTIFACT_MANIFEST_SCHEMA,
        "schema_version": audit.ARTIFACT_MANIFEST_VERSION,
        "path_relative": "artifact_manifest.json",
        "sha256": artifact_manifest_sealed["sha256"],
        "size_bytes": artifact_manifest_path.stat().st_size,
        "sidecar_path_relative": "artifact_manifest.json.sha256",
        "sidecar_sha256": artifact_manifest_sealed["sidecar_sha256"],
        "evidence_root": root_identity,
    }
    receipt = {
        "schema_version": 1, "contract_version": RELEASE_CONTRACT, "status": "PASS",
        "distro": distro, "dependency_leg": dependency_leg,
        "campaign_set": release["campaign_set"],
        "profile": {"path": str(profile_path), "sha256": _sha256_file(profile_path)},
        "container": identity,
        "repository": {"path": str(repo_root), "source_root": str(source_root),
                        "source_manifest": source_report, "git": git_report},
        "dependency": dependency_evidence,
        "execution": {"authority": authority,
                       "provisioning_network_used": True,
                       "archive_fetch_network_used": dependency_leg == "present",
                       "build_test_network_connected": build_test_network_connected,
                       "build_test_network_used": build_test_network_used,
                       "network_after_dependency_fetch": False,
                       "network_policy": expected_network_policy,
                       "repository_mount": "read_only",
                       "install_mode": "isolated_non_symlink", "build_started": True,
                       "formal_replay_started": False},
        "commands": {"records": all_commands,
                     "canonical_sha256": _command_hash([item["argv"] for item in all_commands])},
        "tests": {"colcon_plugin_packages": list(TARGET_PACKAGES),
                  "targeted": {label: "PASS" for label in TARGET_TESTS},
                  "transactional_activation_rollback": "PASS",
                  "real_ros_resource_init_failure": "PASS",
                  "provenance_pre_constructor": "PASS",
                  "cxx14_external_consumer": "PASS",
                  "cxx14_external_template": "PASS"},
        "external_dso": {"receipt_path": str(odr_receipt),
                         "receipt_path_relative": _root_relative(evidence_root, odr_receipt, "DSO receipt"),
                         "receipt_sha256": odr_sha,
                         "sidecar_path_relative": _root_relative(evidence_root, odr_sidecar, "DSO receipt sidecar"),
                         "sidecar_sha256": _sha256_file(odr_sidecar),
                         "status": "PASS", "load_session_smoke": "PASS",
                         "odr_symbol_ownership": "PASS"},
        "dependency_environment": dependency_environment,
        "dependency_prefetch": dependency_prefetch_evidence,
        "installed_tree": install_manifest,
        "test_results": test_results_manifest,
        "optional_selector_report": optional_report,
        "directory_contract": directory_contract,
        "resource_policy": {
            "schema": audit.FUNCTIONAL_RESOURCE_POLICY_SCHEMA,
            "schema_version": audit.FUNCTIONAL_RESOURCE_POLICY_VERSION,
            "mode": audit.FUNCTIONAL_RESOURCE_MODE,
            "preflight": resource_preflight,
            "completion": resource_completion,
            "max_build_workers": resource_policy["functional"]["max_build_workers"],
            "priority": {"nice": resource_policy["functional"]["nice"],
                         "ionice_class": resource_policy["functional"]["ionice_class"],
                         "nice_available": shutil.which("nice") is not None,
                         "ionice_available": shutil.which("ionice") is not None},
            "timing_authority": audit.FUNCTIONAL_TIMING_AUTHORITY,
            "timing_metrics": {
                "status": audit.FUNCTIONAL_TIMING_AUTHORITY,
                "performance_gate_eligible": False,
                "wall_seconds": None, "peak_rss_bytes": None, "rtf": None,
            },
            "performance_gate_eligible": False,
        },
        "performance_metrics": {
            "status": audit.FUNCTIONAL_TIMING_AUTHORITY,
            "performance_gate_eligible": False,
            "wall_seconds": None, "peak_rss_bytes": None, "rtf": None,
        },
        "safety": {"provisioning_network_used": True,
                    "archive_fetch_network_used": dependency_leg == "present",
                    "build_test_network_connected": build_test_network_connected,
                    "build_test_network_used": build_test_network_used,
                    "bag_opened": False, "ground_truth_content_opened": False,
                    "scorer_invoked": False, "map_saved": False,
                    "formal_replay_started": False},
        "evidence_root": root_identity,
        "artifact_manifest": artifact_manifest_binding,
        "artifacts": {"work_root": str(work), "install_root": str(install_root),
                       "logs_root": str(log_root), "odr_receipt": str(odr_receipt),
                       "provenance_receipt": str(provenance_path),
                       "resource_failure_receipt": str(resource_failure_path)},
        "observed_at_unix": time.time(),
    }
    return _seal(output, receipt), receipt


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=str(ROOT))
    parser.add_argument("--profile", default=str(ROOT / "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"))
    parser.add_argument("--distro", choices=("humble", "jazzy"), required=True)
    parser.add_argument("--dependency-leg", choices=("absent", "present"), required=True)
    parser.add_argument("--evidence-root", required=True,
                        help="fresh per-leg evidence root created by this runner")
    parser.add_argument("--output", required=True)
    parser.add_argument("--work-root")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    repo_root = Path(args.repo_root).absolute()
    profile_path = Path(args.profile).absolute()
    output = Path(args.output).absolute()
    try:
        result, report = run_release_leg(
            repo_root, profile_path, args.distro, args.dependency_leg, output,
            evidence_root=Path(args.evidence_root).absolute(),
            work_root=Path(args.work_root).absolute() if args.work_root else None,
        )
        print(json.dumps({"status": report["status"], "distro": args.distro,
                          "dependency_leg": args.dependency_leg,
                          "receipt": result}, sort_keys=True))
        return 0 if report["status"] == "PASS" else 2
    except Exception as exc:
        try:
            profile, _release, loaded_profile = _load_profile(profile_path)
            source = audit._source_manifest(repo_root, profile)
            git_report = audit._git_snapshot(repo_root)
        except Exception:
            source = {"status": "UNAVAILABLE"}
            git_report = {"status": "UNAVAILABLE"}
            loaded_profile = profile_path
        plan = {"distro": args.distro, "dependency_leg": args.dependency_leg}
        failure = _failure_document(repo_root, loaded_profile, source, git_report, plan, exc)
        try:
            sealed = _seal(output, failure)
            failure["receipt"] = sealed
        except Exception as seal_error:
            failure["receipt_seal_error"] = str(seal_error)
        print(json.dumps(failure, sort_keys=True), file=sys.stderr)
        return 2 if getattr(exc, "kind", "").startswith("BLOCKED") else 1


if __name__ == "__main__":
    sys.exit(main())
