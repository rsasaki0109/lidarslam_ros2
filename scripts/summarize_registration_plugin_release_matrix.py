#!/usr/bin/env python3
"""Fail-closed summary gate for the four Humble/Jazzy release receipts."""

from __future__ import print_function

import argparse
import hashlib
import json
import re
import shlex
import stat
import sys
from pathlib import Path

# Direct source-checkout execution puts ``scripts/`` (not the checkout root)
# on ``sys.path``.  Use the one canonical checkout package when it is present;
# installed summary execution keeps normal package resolution.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

import lidarslam_benchmark_tools.audit_registration_plugin_matrix as audit  # noqa: E402
from lidarslam_benchmark_tools import (  # noqa: E402
    capture_registration_plugin_dependency_closure as capture_executor,
    registration_plugin_dependency_closure as dependency_closure,
    registration_plugin_dependency_contract as dependency_contract,
    registration_plugin_dependency_prefetch as dependency_prefetch,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = "registration-plugin-release-matrix-summary-v1"
EXPECTED_ROWS = {("humble", "absent"), ("humble", "present"),
                 ("jazzy", "absent"), ("jazzy", "present")}
DEPENDENCY_ENVIRONMENT_SCHEMA = "registration-plugin-dependency-environment-v1"
DEPENDENCY_ENVIRONMENT_VERSION = 1
DEPENDENCY_ENVIRONMENT_RELATIVE = "dependency_environment.receipt.json"
# Finite host-side bounds are checked before any digest or JSON read.  16 MiB
# leaves ample room for a large dpkg/apt/rosdep identity closure while keeping
# an untrusted preseed from forcing an unbounded allocation/read; the sidecar
# contains only one digest and filename.
DEPENDENCY_ENVIRONMENT_MAX_BYTES = 16 * 1024 * 1024
DEPENDENCY_ENVIRONMENT_SIDECAR_MAX_BYTES = 256
ROS2_SOURCES_STABLE_PATH = "apt_sources/sources.list.d/ros2.sources"
ROS2_SOURCES_TARGET = "/usr/share/ros-apt-source/ros2.sources"
ROS2_SOURCES_TARGET_MODE = 0o644
HOST_PROMOTION_AUTHORITY = "HOST_PROMOTION"
HOST_RECEIPT_SCHEMA = "registration-plugin-host-release-leg-v1"
HOST_RECEIPT_NAME = "registration_plugin_host.receipt.json"
HOST_RECEIPT_ID_RE = re.compile(r"^[0-9a-f]{12,64}$")
HOST_RECEIPT_MAX_BYTES = 16 * 1024 * 1024
HOST_RECEIPT_SIDECAR_MAX_BYTES = 256
HOST_LOG_MAX_BYTES = 4 * 1024 * 1024
REQUIRED_ENV_KEYS = (
    "ROS_DISTRO",
    "REGISTRATION_PLUGIN_CONTAINER_DIGEST",
    "REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY",
    "REGISTRATION_PLUGIN_RESOURCE_POLICY",
    "CMAKE_BUILD_PARALLEL_LEVEL",
    "MAKEFLAGS",
    "NINJAFLAGS",
)
HOST_RUNNER_ENV = {
    "REGISTRATION_PLUGIN_EXECUTION_AUTHORITY": HOST_PROMOTION_AUTHORITY,
    "REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED": "false",
    "REGISTRATION_PLUGIN_HOST_DISCONNECT_VERIFIED": "true",
}
DEPENDENCY_CAPTURE_SCRIPT = "/workspace/src/scripts/capture_registration_plugin_dependency_environment.py"
RUNNER_SCRIPT = "/workspace/src/scripts/run_registration_plugin_release_matrix.py"


class SummaryError(RuntimeError):
    pass


def _sha256(path):
    return audit.sha256_file(path)


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _read_receipt(path):
    path = Path(path).absolute()
    if path.is_symlink() or not path.is_file():
        raise SummaryError("receipt is not a regular file: {}".format(path))
    sidecar = Path(str(path) + ".sha256")
    if sidecar.is_symlink() or not sidecar.is_file():
        raise SummaryError("receipt sidecar is missing or symlinked: {}".format(sidecar))
    if (stat.S_IMODE(path.stat().st_mode) != 0o444 or
            stat.S_IMODE(sidecar.stat().st_mode) != 0o444 or
            path.stat().st_nlink != 1 or sidecar.stat().st_nlink != 1):
        raise SummaryError("receipt and sidecar must be mode 0444: {}".format(path))
    if not 0 < path.stat().st_size <= HOST_RECEIPT_MAX_BYTES:
        raise SummaryError("receipt size is outside bounded limit: {}".format(path))
    if not 0 < sidecar.stat().st_size <= HOST_RECEIPT_SIDECAR_MAX_BYTES:
        raise SummaryError("receipt sidecar size is outside bounded limit: {}".format(sidecar))
    digest = _sha256(path)
    try:
        expected = sidecar.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        raise SummaryError("receipt sidecar is not ASCII: {}".format(exc))
    if len(expected) != 2 or expected[0] != digest or expected[1] != path.name:
        raise SummaryError("receipt sidecar mismatch: {}".format(path))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SummaryError("receipt JSON invalid: {}".format(exc))
    if not isinstance(value, dict):
        raise SummaryError("receipt JSON is not an object: {}".format(path))
    return path, value, digest, _sha256(sidecar)


def _artifact_json(root, relative, label):
    """Read one sealed JSON artifact only through a validated root-relative path."""
    try:
        path = audit._artifact_file(root, relative, label)
    except audit.AuditError as exc:
        raise SummaryError(str(exc))
    if stat.S_IMODE(path.stat().st_mode) != 0o444 or path.stat().st_nlink != 1:
        raise SummaryError("{} must be a single-link mode-0444 file: {}".format(label, relative))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SummaryError("{} JSON is invalid: {}".format(label, exc))
    if not isinstance(value, dict):
        raise SummaryError("{} JSON is not an object: {}".format(label, relative))
    return path, value


def _dependency_file_section(value, label, release_indexes=False, source_config=False):
    if not isinstance(value, dict):
        raise SummaryError("{} is not an object".format(label))
    expected = {"entries", "entry_count", "tree_sha256"}
    if release_indexes:
        expected.add("release_indexes")
    if source_config:
        expected.update({"symlink_entries", "symlink_count"})
    if set(value) != expected:
        raise SummaryError("{} fields are not canonical".format(label))
    entries = value.get("entries")
    if not isinstance(entries, list) or (not entries and not source_config):
        raise SummaryError("{} is empty".format(label))
    observed = []
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {"path", "size_bytes", "sha256"}:
            raise SummaryError("{} entry is malformed".format(label))
        try:
            path = audit.normalize_artifact_relative_path(entry["path"], label + " path")
        except audit.AuditError as exc:
            raise SummaryError(str(exc))
        if (path != entry["path"] or type(entry["size_bytes"]) is not int or
                entry["size_bytes"] < 0 or
                not isinstance(entry["sha256"], str) or
                not audit.SHA256_RE.fullmatch(entry["sha256"])):
            raise SummaryError("{} entry identity is invalid".format(label))
        observed.append({"path": path, "size_bytes": entry["size_bytes"],
                         "sha256": entry["sha256"]})
    if observed != sorted(observed, key=lambda item: item["path"]):
        raise SummaryError("{} entries are not deterministically sorted".format(label))
    if len({item["path"] for item in observed}) != len(observed):
        raise SummaryError("{} contains duplicate paths".format(label))
    symlink_entries = []
    if source_config:
        symlink_entries = value.get("symlink_entries")
        if not isinstance(symlink_entries, list) or len(symlink_entries) > 1:
            raise SummaryError("{} symlink entries are malformed".format(label))
        if symlink_entries != sorted(symlink_entries, key=lambda item: item.get("path", "")):
            raise SummaryError("{} symlink entries are not sorted".format(label))
        for entry in symlink_entries:
            if not isinstance(entry, dict) or set(entry) != {
                    "path", "target", "size_bytes", "mode", "uid", "gid", "nlink",
                    "target_file"}:
                raise SummaryError("{} symlink entry is malformed".format(label))
            target_file = entry["target_file"]
            if not isinstance(target_file, dict) or set(target_file) != {
                    "path", "size_bytes", "sha256", "mode", "uid", "gid", "nlink"}:
                raise SummaryError("{} symlink target descriptor is malformed".format(label))
            if (entry["path"] != ROS2_SOURCES_STABLE_PATH or
                    entry["target"] != ROS2_SOURCES_TARGET or
                    type(entry["size_bytes"]) is not int or
                    entry["size_bytes"] != len(entry["target"].encode("utf-8")) or
                    entry["mode"] != 0o777 or entry["uid"] != 0 or entry["gid"] != 0 or
                    entry["nlink"] != 1 or
                    target_file["path"] != ROS2_SOURCES_TARGET or
                    type(target_file["size_bytes"]) is not int or
                    target_file["size_bytes"] <= 0 or
                    not isinstance(target_file["sha256"], str) or
                    not audit.SHA256_RE.fullmatch(target_file["sha256"]) or
                    target_file["mode"] != ROS2_SOURCES_TARGET_MODE or
                    target_file["uid"] != 0 or target_file["gid"] != 0 or
                    target_file["nlink"] != 1):
                raise SummaryError("{} symlink target identity is invalid".format(label))
            if any(item["path"] == ROS2_SOURCES_STABLE_PATH for item in observed):
                raise SummaryError("{} symlink path is duplicated in regular entries".format(label))
        if value["symlink_count"] != len(symlink_entries):
            raise SummaryError("{} symlink count changed".format(label))
        if not observed and not symlink_entries:
            raise SummaryError("{} has no source configuration files".format(label))
    if value["entry_count"] != len(observed):
        raise SummaryError("{} entry count changed".format(label))
    tree_projection = ({"entries": observed, "symlink_entries": symlink_entries}
                       if source_config else observed)
    if value["tree_sha256"] != _sha256_bytes(
            json.dumps(tree_projection, sort_keys=True, separators=(",", ":")).encode("utf-8")):
        raise SummaryError("{} tree identity changed".format(label))
    if release_indexes:
        release = value.get("release_indexes")
        if not isinstance(release, list) or not release:
            raise SummaryError("apt Release/InRelease identity is missing")
        if any(not isinstance(item, dict) or set(item) != {
                "path", "size_bytes", "sha256"} for item in release):
            raise SummaryError("apt Release/InRelease entry is malformed")
        if release != sorted(release, key=lambda item: item.get("path", "")):
            raise SummaryError("apt Release/InRelease entries are not sorted")
        if any(item not in observed for item in release):
            raise SummaryError("apt Release/InRelease entry is not bound to indexes")
        if len({item["path"] for item in release}) != len(release):
            raise SummaryError("apt Release/InRelease entries are duplicated")
    result = {
        "entries": observed,
        "entry_count": len(observed),
        "tree_sha256": value["tree_sha256"],
    }
    if source_config:
        result["symlink_entries"] = list(symlink_entries)
        result["symlink_count"] = len(symlink_entries)
    if release_indexes:
        result["release_indexes"] = list(value["release_indexes"])
    return result


def _dependency_projection(value):
    return {
        "base_image": value["base_image"],
        "ros": value["ros"],
        "dpkg": value["dpkg"],
        "apt": value["apt"],
        "rosdep": value["rosdep"],
        "install_command": value["install_command"],
        "capture": value["capture"],
        "secret_policy": value["secret_policy"],
    }


def _validate_dependency_environment_manifest(root, relative, distro, image,
                                              install_command_sha256,
                                              capture_script_sha256):
    """Reopen and validate the pre-disconnect base dependency closure."""
    if relative != DEPENDENCY_ENVIRONMENT_RELATIVE:
        raise SummaryError("dependency environment path is not canonical")
    try:
        path = audit._artifact_file(root, relative, "dependency environment receipt")
        sidecar = audit._artifact_file(
            root, relative + ".sha256", "dependency environment sidecar")
    except audit.AuditError as exc:
        raise SummaryError(str(exc))
    for candidate in (path, sidecar):
        if stat.S_IMODE(candidate.stat().st_mode) != 0o444 or candidate.stat().st_nlink != 1:
            raise SummaryError("dependency environment artifacts must be mode-0444 single-link files")
    manifest_size = path.stat().st_size
    sidecar_size = sidecar.stat().st_size
    if not 0 < manifest_size <= DEPENDENCY_ENVIRONMENT_MAX_BYTES:
        raise SummaryError(
            "dependency environment manifest size is outside bounded limit: {}".format(
                manifest_size))
    if not 0 < sidecar_size <= DEPENDENCY_ENVIRONMENT_SIDECAR_MAX_BYTES:
        raise SummaryError(
            "dependency environment sidecar size is outside bounded limit: {}".format(
                sidecar_size))
    digest = _sha256(path)
    try:
        tokens = sidecar.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        raise SummaryError("dependency environment sidecar is not ASCII: {}".format(exc))
    if tokens != [digest, path.name]:
        raise SummaryError("dependency environment sidecar mismatch")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SummaryError("dependency environment JSON is invalid: {}".format(exc))
    if not isinstance(value, dict):
        raise SummaryError("dependency environment is not an object")
    required = {
        "schema", "schema_version", "status", "base_image", "ros", "dpkg",
        "apt", "rosdep", "install_command", "capture", "secret_policy",
        "canonical_sha256",
    }
    if set(value) != required or value.get("schema") != DEPENDENCY_ENVIRONMENT_SCHEMA or \
            value.get("schema_version") != DEPENDENCY_ENVIRONMENT_VERSION or \
            value.get("status") != "PASS":
        raise SummaryError("dependency environment schema/status is invalid")
    base = value["base_image"]
    if not isinstance(base, dict) or set(base) != {
            "digest", "os", "architecture", "dpkg_architecture", "machine"}:
        raise SummaryError("dependency environment base identity is malformed")
    if (base["digest"] != image.get("digest") or base["os"] != "linux" or
            base["architecture"] != "amd64" or base["dpkg_architecture"] != "amd64" or
            base["machine"] not in ("amd64", "x86_64")):
        raise SummaryError("dependency environment base identity mismatch")
    ros = value["ros"]
    if ros != {"distro": distro}:
        raise SummaryError("dependency environment ROS distro mismatch")
    packages = value["dpkg"]
    if not isinstance(packages, dict) or set(packages) != {
            "packages", "package_count", "canonical_sha256"}:
        raise SummaryError("dpkg closure is malformed")
    package_rows = packages["packages"]
    if not isinstance(package_rows, list) or not package_rows:
        raise SummaryError("dpkg closure is empty")
    if any(not isinstance(item, dict) or set(item) != {
            "name", "version", "architecture", "status"} or
            any(not isinstance(item[field], str) or not item[field]
                for field in ("name", "version", "architecture", "status"))
            for item in package_rows):
        raise SummaryError("dpkg package row is malformed")
    if package_rows != sorted(package_rows, key=lambda item: (
            item["name"], item["version"], item["architecture"], item["status"])):
        raise SummaryError("dpkg package closure is not sorted")
    if len({tuple(item.values()) for item in package_rows}) != len(package_rows):
        raise SummaryError("dpkg package closure contains duplicates")
    if packages["package_count"] != len(package_rows) or \
            packages["canonical_sha256"] != _sha256_bytes(
                json.dumps(package_rows, sort_keys=True, separators=(",", ":")).encode("utf-8")):
        raise SummaryError("dpkg package closure hash mismatch")
    apt = value["apt"]
    if not isinstance(apt, dict) or set(apt) != {"source_config", "indexes"}:
        raise SummaryError("apt closure is malformed")
    apt_sources = _dependency_file_section(
        apt["source_config"], "apt source config", source_config=True)
    apt_indexes = _dependency_file_section(apt["indexes"], "apt indexes", release_indexes=True)
    if not any("_InRelease" in item["path"] or item["path"].endswith("/Release")
               for item in apt_indexes["release_indexes"]):
        raise SummaryError("apt Release/InRelease file identity is missing")
    apt = {"source_config": apt_sources, "indexes": apt_indexes}
    rosdep = value["rosdep"]
    if not isinstance(rosdep, dict) or set(rosdep) != {"sources", "cache"}:
        raise SummaryError("rosdep closure is malformed")
    rosdep = {
        "sources": _dependency_file_section(rosdep["sources"], "rosdep sources"),
        "cache": _dependency_file_section(rosdep["cache"], "rosdep cache"),
    }
    install = value["install_command"]
    capture = value["capture"]
    security = value["secret_policy"]
    if install != {"sha256": install_command_sha256, "exit_code": 0}:
        raise SummaryError("dependency install command identity mismatch")
    if capture != {"script_sha256": capture_script_sha256}:
        raise SummaryError("dependency capture script identity mismatch")
    if security != {
            "environment_recorded": [],
            "credential_values_recorded": False,
            "proxy_values_recorded": False,
    }:
        raise SummaryError("dependency environment secret policy drift")
    projection = {
        "base_image": base, "ros": ros,
        "dpkg": {
            "packages": package_rows, "package_count": len(package_rows),
            "canonical_sha256": packages["canonical_sha256"],
        },
        "apt": apt, "rosdep": rosdep, "install_command": install,
        "capture": capture, "secret_policy": security,
    }
    if value["canonical_sha256"] != _sha256_bytes(
            json.dumps(projection, sort_keys=True, separators=(",", ":")).encode("utf-8")):
        raise SummaryError("dependency environment canonical hash mismatch")
    return {
        "status": "PASS", "path_relative": relative, "sha256": digest,
        "size_bytes": path.stat().st_size, "sidecar_sha256": _sha256(sidecar),
        "projection": projection,
        "projection_sha256": _sha256_bytes(
            json.dumps(projection, sort_keys=True, separators=(",", ":")).encode("utf-8")),
    }


def _host_log(root, relative, record, logs_by_path):
    """Reopen one host command log and bind it to the sealed outer receipt."""
    if not isinstance(relative, str) or not relative.startswith("launcher/"):
        raise SummaryError("host command log is outside launcher root")
    try:
        path = audit._artifact_file(root, relative, "host command log")
    except audit.AuditError as exc:
        raise SummaryError(str(exc))
    info = path.stat()
    if (stat.S_IMODE(info.st_mode) != 0o444 or info.st_nlink != 1 or
            info.st_size > HOST_LOG_MAX_BYTES):
        raise SummaryError("host command log is not an immutable single-link file")
    digest = _sha256(path)
    observed = logs_by_path.get(relative)
    if (not isinstance(observed, dict) or observed.get("sha256") != digest or
            observed.get("bytes") != info.st_size or
            record.get("log", {}).get("sha256") != digest or
            record.get("log", {}).get("bytes") != info.st_size):
        raise SummaryError("host command log binding mismatch: {}".format(relative))
    return path


def _validate_absence_log(path, name, label):
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise SummaryError("{} is not valid UTF-8: {}".format(label, exc))
    allowed = (
        "Error: No such object: {}".format(name),
        "Error: No such container: {}".format(name),
    )
    if text not in tuple(line + suffix for line in allowed for suffix in ("", "\n")):
        raise SummaryError("{} does not prove exact container absence".format(label))


def _validate_effective_argv(record, expected_raw, label):
    raw = record.get("argv")
    effective = record.get("effective_argv")
    if not isinstance(raw, list) or not isinstance(effective, list):
        raise SummaryError("{} argv record is malformed".format(label))
    if raw != expected_raw:
        raise SummaryError("{} raw argv is not the canonical command".format(label))
    if effective == raw:
        prefix = ()
    else:
        try:
            docker_index = effective.index("docker")
        except ValueError:
            raise SummaryError("{} effective argv has no docker suffix".format(label))
        if effective[docker_index:] != raw:
            raise SummaryError("{} effective argv does not preserve raw argv".format(label))
        prefix = effective[:docker_index]
    if effective == raw:
        prefix = ()
    valid_prefixes = (
        ("ionice", "-c", "3", "nice", "-n", "19"),
        ("nice", "-n", "19"),
        ("ionice", "-c", "3"),
        (),
    )
    normalized = tuple(Path(token).name if index in (0, 3) else token
                       for index, token in enumerate(prefix))
    if normalized not in valid_prefixes:
        raise SummaryError("{} effective argv priority prefix is invalid".format(label))
    expected_hash = _sha256_bytes(json.dumps(raw, separators=(",", ":")).encode("utf-8"))
    if record.get("argv_sha256") != expected_hash:
        raise SummaryError("{} argv hash is stale".format(label))
    return raw


def _expected_environment(release, distro, image):
    return {
        "ROS_DISTRO": distro,
        "REGISTRATION_PLUGIN_CONTAINER_DIGEST": image["digest"],
        "REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY":
            release["policy"]["network_after_dependency_fetch"],
        "REGISTRATION_PLUGIN_RESOURCE_POLICY": release["resource_policy"]["functional"]["mode"],
        "CMAKE_BUILD_PARALLEL_LEVEL": "1",
        "MAKEFLAGS": "-j1",
        "NINJAFLAGS": "-j1",
    }


def _expected_runner_shell(distro, dependency_leg):
    args = [
        "python3", RUNNER_SCRIPT,
        "--repo-root", "/workspace/src",
        "--profile", "/workspace/src/configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json",
        "--distro", distro,
        "--dependency-leg", dependency_leg,
        "--evidence-root", "/workspace/evidence-parent/registration-plugin-release-{}-{}".format(
            distro, dependency_leg),
        "--output", "/workspace/evidence-parent/registration-plugin-release-{}-{}/registration_plugin_release.receipt.json".format(
            distro, dependency_leg),
    ]
    return "source /opt/ros/{}/setup.bash && PYTHONPATH=/workspace/src:/workspace/src/scripts exec {}".format(
        distro, " ".join(shlex.quote(item) for item in args))


def _expected_capture_argv(name, distro, dependency_leg, image, dependency_install):
    return [
        "docker", "exec", name, "python3", DEPENDENCY_CAPTURE_SCRIPT,
        "--output", "/workspace/evidence-parent/registration-plugin-release-{}-{}/{}".format(
            distro, dependency_leg, DEPENDENCY_ENVIRONMENT_RELATIVE),
        "--distro", distro,
        "--image-digest", image["digest"],
        "--install-command-sha256", _sha256_bytes(
            dependency_install.encode("utf-8")),
        "--install-exit-code", "0",
        "--capture-script-sha256", _sha256(
            ROOT / "scripts/capture_registration_plugin_dependency_environment.py"),
    ]


def _validate_command_record(record, expected_raw, label):
    if record.get("returncode") != 0:
        raise SummaryError("{} did not pass".format(label))
    return _validate_effective_argv(record, expected_raw, label)


def _validate_docker_run_record(record, name, image_reference, host_root,
                                release, distro):
    expected_env = _expected_environment(
        release, distro, next(item["image"] for item in release["distros"]
                              if item["name"] == distro))
    expected_raw = ["docker", "run", "-d", "--name", name]
    for key in REQUIRED_ENV_KEYS:
        expected_raw.extend(["--env", "{}={}".format(key, expected_env[key])])
    expected_raw.extend([
        "-v", "{}:/workspace/src:ro".format(ROOT),
        "-v", "{}:/workspace/evidence-parent:rw".format(host_root),
        image_reference, "tail", "-f", "/dev/null",
    ])
    raw = _validate_command_record(record, expected_raw, "docker run")
    if raw[:4] != ["docker", "run", "-d", "--name"] or raw[4] != name:
        raise SummaryError("docker run name contract is invalid")
    return raw


def _validate_inspected_mounts(mounts, host_root):
    if not isinstance(mounts, list) or len(mounts) != 2:
        raise SummaryError("post-disconnect inspect mount list is incomplete")
    by_destination = {item.get("Destination"): item for item in mounts
                      if isinstance(item, dict)}
    expected = {
        "/workspace/src": (str(ROOT), False, "ro"),
        "/workspace/evidence-parent": (str(host_root), True, "rw"),
    }
    if set(by_destination) != set(expected):
        raise SummaryError("post-disconnect inspect mount destinations changed")
    for destination, (source, writable, mode) in expected.items():
        observed = by_destination[destination]
        if (observed.get("Type") != "bind" or observed.get("Source") != source or
                observed.get("Destination") != destination or
                observed.get("RW") is not writable or observed.get("Mode") != mode):
            raise SummaryError("post-disconnect inspect mount identity changed")


def _validate_host_receipt(inner_path, inner_value, profile, release, profile_path,
                           current_source):
    """Require launcher-owned evidence before promoting an inner row.

    Environment flags in the inner runner are self-reported.  Only the sealed
    outer host receipt can bind the child bytes to the post-disconnect inspect,
    command records, cleanup proof, and profile storage identity.
    """
    inner_root = Path(inner_path).parent
    host_root = inner_root.parent
    try:
        audit._artifact_root(host_root)
    except audit.AuditError as exc:
        raise SummaryError("host evidence root is invalid: {}".format(exc))
    host_path = host_root / HOST_RECEIPT_NAME
    _, host, host_digest, host_sidecar_digest = _read_receipt(host_path)
    del host_digest, host_sidecar_digest
    row = (inner_value.get("distro"), inner_value.get("dependency_leg"))
    expected_campaign = release.get("campaign_set")
    if (host.get("schema") != HOST_RECEIPT_SCHEMA or host.get("schema_version") != 1 or
            host.get("status") != "PASS" or host.get("distro") != row[0] or
            host.get("dependency_leg") != row[1]):
        raise SummaryError("outer host receipt is not a PASS for the inner row")
    if host.get("campaign_set") != expected_campaign:
        raise SummaryError("outer host campaign_set identity drift")
    if host.get("profile") != {
            "path": str(profile_path), "sha256": _sha256(profile_path)}:
        raise SummaryError("outer host receipt profile binding drift")
    source = host.get("source")
    if (not isinstance(source, dict) or
            source.get("manifest_sha256") != current_source["manifest_sha256"] or
            source.get("runner_sha256") != _sha256(
                ROOT / "scripts/run_registration_plugin_release_matrix.py") or
            source.get("launcher_sha256") != _sha256(
                ROOT / "scripts/run_registration_plugin_release_leg.py") or
            source.get("prefetch_sha256") != _sha256(
                ROOT / "scripts/registration_plugin_dependency_prefetch.py")):
        raise SummaryError("outer host receipt source binding drift")
    storage = host.get("evidence_storage")
    expected_storage = release.get("evidence_storage")
    if (not isinstance(storage, dict) or storage.get("contract") != expected_storage or
            not isinstance(storage.get("observed"), dict)):
        raise SummaryError("outer host evidence-storage contract is missing or stale")
    observed_storage = storage["observed"]
    for key in ("mountpoint", "filesystem", "source_uuid"):
        if observed_storage.get(key) != expected_storage.get(key):
            raise SummaryError("outer host evidence-storage identity drift: {}".format(key))
    container = host.get("container")
    image = next(item["image"] for item in release["distros"] if item["name"] == row[0])
    if (not isinstance(container, dict) or
            not isinstance(container.get("id"), str) or
            not HOST_RECEIPT_ID_RE.fullmatch(container["id"]) or
            container.get("start_count") != 1 or
            container.get("image_reference") != image["reference"]):
        raise SummaryError("outer host container identity is incomplete")
    mounts = container.get("mounts")
    if not isinstance(mounts, list) or len(mounts) != 2:
        raise SummaryError("outer host mount identity is incomplete")
    mounts_by_destination = {item.get("destination"): item for item in mounts
                             if isinstance(item, dict)}
    repo_mount = mounts_by_destination.get("/workspace/src")
    evidence_mount = mounts_by_destination.get("/workspace/evidence-parent")
    if (not isinstance(repo_mount, dict) or repo_mount.get("mode") != "ro" or
            repo_mount.get("source") != str(ROOT) or
            not isinstance(evidence_mount, dict) or evidence_mount.get("mode") != "rw" or
            evidence_mount.get("source") != str(host_root)):
        raise SummaryError("outer host mount identity is not bound to the evidence root")
    image_observed = host.get("image")
    if (not isinstance(image_observed, dict) or image_observed.get("id") != image["digest"] or
            image_observed.get("os") != "linux" or
            image_observed.get("architecture") != "amd64" or
            not any(str(item).endswith("@" + image["digest"])
                    for item in image_observed.get("repo_digests", []))):
        raise SummaryError("outer host image identity is not pinned")
    if host.get("cleanup", {}).get("post_remove_absent") is not True:
        raise SummaryError("outer host cleanup did not prove container absence")
    expected_child = inner_path.relative_to(host_root).as_posix()
    binding = host.get("runner_receipt_binding")
    inner_sidecar = Path(str(inner_path) + ".sha256")
    if (not isinstance(binding, dict) or binding.get("status") != "PASS" or
            binding.get("path_relative") != expected_child or
            binding.get("sidecar_path_relative") != expected_child + ".sha256" or
            binding.get("sha256") != _sha256(inner_path) or
            binding.get("sidecar_sha256") != _sha256(inner_sidecar) or
            binding.get("evidence_root") != inner_value.get("evidence_root")):
        raise SummaryError("outer host child runner receipt binding is missing or stale")
    if host.get("runner_receipt") != inner_value:
        raise SummaryError("outer host nested runner receipt differs from child bytes")
    revalidation = host.get("artifact_revalidation")
    inner_manifest = inner_value.get("artifact_manifest", {})
    if (not isinstance(revalidation, dict) or
            revalidation.get("root") != str(inner_root) or
            revalidation.get("root_identity") != inner_value.get("evidence_root") or
            revalidation.get("manifest_sha256") != inner_manifest.get("sha256") or
            type(revalidation.get("manifest_entry_count")) is not int or
            revalidation.get("manifest_entry_count") <= 0):
        raise SummaryError("outer host artifact revalidation is not child-bound")
    safety = host.get("safety", {})
    for key, expected in {
            "provisioning_network_used": True,
            "archive_fetch_network_used": row[1] == "present",
            "build_test_network_connected": False,
            "build_test_network_used": False,
            "network_after_dependency_fetch": False,
    }.items():
        if safety.get(key) is not expected:
            raise SummaryError("outer host network phase {} is not truthful".format(key))
    if "network_used" in safety:
        raise SummaryError("outer host ambiguous network_used field is forbidden")
    if any(safety.get(key) is not False for key in (
            "bag_opened", "ground_truth_content_opened", "scorer_invoked",
            "map_saved", "formal_replay_started")):
        raise SummaryError("outer host safety boundary is not closed")
    records = host.get("commands")
    logs = host.get("logs")
    if not isinstance(records, list) or not isinstance(logs, list):
        raise SummaryError("outer host command/log records are missing")
    logs_by_path = {}
    for entry in logs:
        if (not isinstance(entry, dict) or not isinstance(entry.get("path"), str) or
                entry["path"] in logs_by_path):
            raise SummaryError("outer host log index is malformed")
        logs_by_path[entry["path"]] = entry
    command_by_label = {}
    command_indices = {}
    for record in records:
        if not isinstance(record, dict) or not isinstance(record.get("label"), str):
            raise SummaryError("outer host command record is malformed")
        label = record["label"]
        if label in command_by_label and not label.startswith("inspect_"):
            raise SummaryError("outer host command label is duplicated: {}".format(label))
        command_by_label[label] = record
        command_indices.setdefault(label, []).append(len(command_indices.get("_order", [])))
        command_indices.setdefault("_order", []).append(label)
        log = record.get("log")
        if not isinstance(log, dict) or not isinstance(log.get("path"), str):
            raise SummaryError("outer host command log binding is missing")
        try:
            relative = Path(log["path"]).resolve(strict=False).relative_to(host_root).as_posix()
        except (OSError, RuntimeError, ValueError) as exc:
            raise SummaryError("outer host command log is outside host root: {}".format(exc))
        _host_log(host_root, relative, record, logs_by_path)
    expected_environment = _expected_environment(release, row[0], image)
    if host.get("environment") != {
            "required": list(REQUIRED_ENV_KEYS), "values": expected_environment}:
        raise SummaryError("outer host environment contract is not canonical")
    image_records = [record for record in records
                     if record.get("label") == "docker_image_inspect"]
    if len(image_records) != 1:
        raise SummaryError("outer host image inspect record is missing or duplicated")
    image_record = image_records[0]
    _validate_command_record(
        image_record, ["docker", "image", "inspect", image["reference"]],
        "image inspect")
    image_log_relative = Path(image_record["log"]["path"]).resolve(strict=False).relative_to(
        host_root).as_posix()
    image_log = _host_log(host_root, image_log_relative, image_record, logs_by_path)
    try:
        image_payload = json.loads(image_log.read_text(encoding="utf-8"))
        observed_image = image_payload[0]
    except (OSError, UnicodeError, ValueError, IndexError, TypeError) as exc:
        raise SummaryError("outer host image inspect is invalid: {}".format(exc))
    if (not isinstance(observed_image, dict) or observed_image.get("Id") != image["digest"] or
            observed_image.get("Os") != "linux" or
            observed_image.get("Architecture") != "amd64" or
            not any(str(item).endswith("@" + image["digest"])
                    for item in observed_image.get("RepoDigests", []))):
        raise SummaryError("outer host image inspect does not prove pinned image")
    if (image_observed.get("id") != observed_image.get("Id") or
            image_observed.get("repo_digests") != observed_image.get("RepoDigests") or
            image_observed.get("os") != observed_image.get("Os") or
            image_observed.get("architecture") != observed_image.get("Architecture")):
        raise SummaryError("outer host image self-report differs from inspect bytes")
    image_index = next(index for index, record in enumerate(records)
                       if record is image_record)
    preexisting_records = [record for record in records
                            if record.get("log", {}).get("path", "").endswith(
                                "preexisting_container_inspect.log")]
    if len(preexisting_records) != 1 or preexisting_records[0].get("returncode") != 1 or \
            preexisting_records[0].get("argv") != ["docker", "inspect", container.get("name")]:
        raise SummaryError("outer host preexisting-name inspect absence record is missing")
    preexisting_record = preexisting_records[0]
    _validate_effective_argv(
        preexisting_record, ["docker", "inspect", container.get("name")],
        "preexisting-name inspect")
    preexisting_relative = Path(preexisting_record["log"]["path"]).resolve(
        strict=False).relative_to(host_root).as_posix()
    preexisting_log = _host_log(host_root, preexisting_relative, preexisting_record, logs_by_path)
    _validate_absence_log(preexisting_log, container.get("name"), "preexisting-name inspect log")
    preexisting_index = next(index for index, record in enumerate(records)
                             if record is preexisting_record)
    for label in ("docker_run", "dependency_install",
                  "dependency_environment_capture", "release_runner"):
        if command_by_label.get(label, {}).get("returncode") != 0:
            raise SummaryError("outer host required command did not pass: {}".format(label))
    ordered_labels = command_indices.get("_order", [])

    def command_index(label):
        indexes = [index for index, observed in enumerate(ordered_labels)
                   if observed == label]
        if len(indexes) != 1:
            raise SummaryError("outer host command is missing or duplicated: {}".format(label))
        return indexes[0]

    docker_run_index = command_index("docker_run")
    if not image_index < preexisting_index < docker_run_index:
        raise SummaryError("outer host image/preexisting-name/docker-run order is invalid")
    _validate_docker_run_record(
        command_by_label["docker_run"], container.get("name"), image["reference"],
        host_root, release, row[0])
    dependency_install_index = command_index("dependency_install")
    _validate_command_record(
        command_by_label["dependency_install"],
        ["docker", "exec", container.get("name"), "bash", "-lc",
         dependency_contract.DEPENDENCY_INSTALL_COMMAND],
        "dependency install")
    capture_index = command_index("dependency_environment_capture")
    _validate_command_record(
        command_by_label["dependency_environment_capture"],
        _expected_capture_argv(
            container.get("name"), row[0], row[1], image,
            dependency_contract.DEPENDENCY_INSTALL_COMMAND),
        "dependency environment capture")
    if not docker_run_index < dependency_install_index < capture_index:
        raise SummaryError("outer host provisioning command order is invalid")
    prefetch_index = None
    if row[1] == "present":
        prefetch = command_by_label.get("dependency_prefetch")
        if not isinstance(prefetch, dict) or prefetch.get("returncode") != 0:
            raise SummaryError("outer host present prefetch command is missing")
        prefetch_argv = [
            "python3", "scripts/registration_plugin_dependency_prefetch.py",
            "--distro", row[0], "--authority", HOST_PROMOTION_AUTHORITY]
        if (prefetch.get("argv") != prefetch_argv or
                prefetch.get("argv_sha256") != _sha256_bytes(
                    json.dumps(prefetch_argv, separators=(",", ":")).encode("utf-8")) or
                prefetch.get("script_sha256") != _sha256(
                    ROOT / "scripts/registration_plugin_dependency_prefetch.py") or
                prefetch.get("network_phase") != "provisioning"):
            raise SummaryError("outer host prefetch command identity is stale")
        _validate_effective_argv(prefetch, prefetch_argv, "dependency prefetch")
        prefetch_index = command_index("dependency_prefetch")
        if not capture_index < prefetch_index:
            raise SummaryError("outer host prefetch is not after dependency capture")
    elif "dependency_prefetch" in command_by_label:
        raise SummaryError("outer host absent leg unexpectedly fetched archives")
    disconnects = [record for record in records
                   if record.get("label", "").startswith("network_disconnect_")]
    if (not disconnects or any(
            record.get("returncode") != 0 or
            len(record.get("argv", [])) != 5 or
            record.get("argv", [])[0:3] != ["docker", "network", "disconnect"] or
            record.get("argv", [])[4] != container.get("name")
            for record in disconnects)):
        raise SummaryError("outer host network disconnect command is missing or failed")
    for record in disconnects:
        _validate_command_record(record, record["argv"], "network disconnect")
    disconnect_indices = [index for index, label in enumerate(ordered_labels)
                          if label.startswith("network_disconnect_")]
    phase_end = prefetch_index if prefetch_index is not None else capture_index
    if any(index <= phase_end for index in disconnect_indices):
        raise SummaryError("outer host network disconnect precedes provisioning completion")
    runner_command = command_by_label["release_runner"]
    runner_raw = ["docker", "exec"]
    for key in REQUIRED_ENV_KEYS:
        runner_raw.extend(["--env", "{}={}".format(key, expected_environment[key])])
    for key in sorted(HOST_RUNNER_ENV):
        runner_raw.extend(["--env", "{}={}".format(key, HOST_RUNNER_ENV[key])])
    runner_raw.extend([
        container.get("name"), "bash", "-lc", _expected_runner_shell(row[0], row[1]),
    ])
    _validate_command_record(runner_command, runner_raw, "release runner")
    runner_tokens = set(str(item) for item in runner_command.get("argv", []))
    for expected in (
            "REGISTRATION_PLUGIN_EXECUTION_AUTHORITY=HOST_PROMOTION",
            "REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED=false",
            "REGISTRATION_PLUGIN_HOST_DISCONNECT_VERIFIED=true"):
        if expected not in runner_tokens:
            raise SummaryError("outer host runner command lacks {}".format(expected))
    if not any("run_registration_plugin_release_matrix.py" in str(item)
               for item in runner_command.get("argv", [])):
        raise SummaryError("outer host runner command is not the pinned release runner")
    inspect_records = [record for record in records
                       if record.get("log", {}).get("path", "").endswith(
                           "post_disconnect_inspect.log")]
    if len(inspect_records) != 1 or inspect_records[0].get("returncode") != 0:
        raise SummaryError("outer host post-disconnect inspect record is missing")
    inspect_record = inspect_records[0]
    _validate_command_record(
        inspect_record, ["docker", "inspect", container.get("name")],
        "post-disconnect inspect")
    inspect_index = next(index for index, record in enumerate(records)
                         if record is inspect_record)
    release_index = command_index("release_runner")
    if (not disconnect_indices or max(disconnect_indices) >= inspect_index or
            inspect_index >= release_index):
        raise SummaryError("outer host disconnect/inspect/runner order is invalid")
    inspect_relative = Path(inspect_record["log"]["path"]).resolve(strict=False).relative_to(
        host_root).as_posix()
    inspect_path = _host_log(host_root, inspect_relative, inspect_record, logs_by_path)
    try:
        inspected = json.loads(inspect_path.read_text(encoding="utf-8"))
        observed_container = inspected[0]
    except (OSError, UnicodeError, ValueError, IndexError, TypeError) as exc:
        raise SummaryError("outer post-disconnect inspect is invalid: {}".format(exc))
    if (not isinstance(observed_container, dict) or
            observed_container.get("Name", "").lstrip("/") != container.get("name") or
            observed_container.get("Id") != container.get("id") or
            observed_container.get("Image") != image["digest"] or
            observed_container.get("Config", {}).get("Image") != image["reference"] or
            observed_container.get("NetworkSettings", {}).get("Networks") != {}):
        raise SummaryError("outer post-disconnect inspect does not prove isolation")
    _validate_inspected_mounts(observed_container.get("Mounts"), host_root)
    cleanup = host.get("cleanup")
    if (not isinstance(cleanup, dict) or
            not isinstance(cleanup.get("stop_requested"), bool) or
            cleanup.get("stopped") is not True or
            cleanup.get("remove_requested") is not True or
            cleanup.get("remove_forced") is not False or
            cleanup.get("post_remove_absent") is not True or
            not isinstance(cleanup.get("exit_code"), int)):
        raise SummaryError("outer host cleanup state is incomplete or unsafe")

    def cleanup_record(name, suffix, returncode, argv):
        record = cleanup.get(name)
        if not isinstance(record, dict) or record.get("returncode") != returncode or \
                record.get("argv") != argv or not record.get("log", {}).get("path", "").endswith(suffix):
            raise SummaryError("outer host cleanup record is invalid: {}".format(name))
        try:
            relative = Path(record["log"]["path"]).resolve(strict=False).relative_to(
                host_root).as_posix()
        except (OSError, RuntimeError, ValueError) as exc:
            raise SummaryError("outer host cleanup log is outside host root: {}".format(exc))
        _host_log(host_root, relative, record, logs_by_path)
        if returncode == 0:
            _validate_command_record(record, argv, "cleanup {}".format(name))
        else:
            _validate_effective_argv(record, argv, "cleanup {}".format(name))
        return record

    cleanup_record("initial_inspect", "cleanup_inspect_initial.log", 0,
                   ["docker", "inspect", container.get("name")])
    cleanup_record("post_stop_inspect", "cleanup_inspect_post_stop.log", 0,
                   ["docker", "inspect", container.get("name")])
    if cleanup.get("stop_requested"):
        cleanup_record("stop", "docker_stop.log", 0,
                       ["docker", "stop", container.get("name")])
    elif "stop" in cleanup:
        raise SummaryError("outer host reported an unrequested stop command")
    cleanup_record("remove", "docker_remove.log", 0,
                   ["docker", "rm", container.get("name")])
    post_remove_record = cleanup_record(
        "post_remove_inspect", "post_remove_inspect.log", 1,
        ["docker", "inspect", container.get("name")])
    post_remove_relative = Path(post_remove_record["log"]["path"]).resolve(
        strict=False).relative_to(host_root).as_posix()
    _validate_absence_log(
        _host_log(host_root, post_remove_relative, post_remove_record, logs_by_path),
        container.get("name"), "post-remove inspect log")
    return {
        "status": "PASS", "receipt_path": str(host_path),
        "receipt_sha256": _sha256(host_path),
        "runner_path_relative": expected_child,
        "runner_receipt_sha256": binding["sha256"],
        "post_disconnect_inspect": inspect_relative,
    }


def _manifest_binding(root, value, expected_identity):
    binding = value.get("artifact_manifest")
    if not isinstance(binding, dict) or \
            binding.get("schema") != audit.ARTIFACT_MANIFEST_SCHEMA or \
            binding.get("schema_version") != audit.ARTIFACT_MANIFEST_VERSION or \
            binding.get("path_relative") != "artifact_manifest.json" or \
            binding.get("sidecar_path_relative") != "artifact_manifest.json.sha256" or \
            binding.get("evidence_root") != expected_identity:
        raise SummaryError("artifact manifest binding is missing or not root-bound")
    try:
        manifest_path = audit._artifact_file(root, binding["path_relative"], "artifact manifest")
        sidecar_path = audit._artifact_file(root, binding["sidecar_path_relative"],
                                            "artifact manifest sidecar")
    except (audit.AuditError, KeyError) as exc:
        raise SummaryError(str(exc))
    for path, label in ((manifest_path, "artifact manifest"),
                        (sidecar_path, "artifact manifest sidecar")):
        if stat.S_IMODE(path.stat().st_mode) != 0o444 or path.stat().st_nlink != 1:
            raise SummaryError("{} must be a single-link mode-0444 file".format(label))
    manifest_sha = _sha256(manifest_path)
    if (binding.get("sha256") != manifest_sha or
            binding.get("size_bytes") != manifest_path.stat().st_size or
            binding.get("sidecar_sha256") != _sha256(sidecar_path)):
        raise SummaryError("artifact manifest binding bytes changed")
    try:
        tokens = sidecar_path.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        raise SummaryError("artifact manifest sidecar is not ASCII: {}".format(exc))
    if len(tokens) != 2 or tokens != [manifest_sha, "artifact_manifest.json"]:
        raise SummaryError("artifact manifest sidecar content mismatch")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        audit.validate_artifact_manifest(
            root, manifest, expected_identity,
            required_roles=value["_artifact_required_roles"],
        )
    except KeyError as exc:
        raise SummaryError("artifact required-role contract missing: {}".format(exc))
    except (OSError, UnicodeError, ValueError, audit.AuditError) as exc:
        raise SummaryError(str(exc))
    if not isinstance(manifest, dict):
        raise SummaryError("artifact manifest JSON is not an object")
    return manifest


def _validate_required_roots(manifest, release, dependency_leg):
    expected_base = {
        "work/logs": "command_log",
        "work/test-results": "test_result",
        "work/install": "installed_tree_file",
        "work/consumer": "consumer_artifact",
        "work/template": "template_artifact",
        "work/evidence": "evidence_artifact",
    }
    expected = dict(expected_base)
    if dependency_leg == "present":
        expected["work/archives"] = "dependency_archive"
    specs = manifest.get("required_roots")
    if not isinstance(specs, list):
        raise SummaryError("artifact manifest required_roots is missing")
    observed = {}
    for spec in specs:
        if not isinstance(spec, dict) or spec.get("required") is False:
            raise SummaryError("artifact required root is malformed or optional")
        path = spec.get("path")
        role = spec.get("role")
        if not isinstance(path, str) or not isinstance(role, str) or path in observed:
            raise SummaryError("artifact required roots contain duplicate/malformed paths")
        observed[path] = role
    for path, role in expected.items():
        if observed.get(path) != role:
            raise SummaryError("artifact required root contract mismatch: {}".format(path))
    if dependency_leg == "absent":
        if set(observed) != set(expected):
            raise SummaryError("absent artifact manifest declares undeclared roots")
    else:
        extra = set(observed) - set(expected)
        for path in extra:
            if not path.startswith("work/vendor/") or observed[path] not in {
                    "dependency_source_file", "dependency_install_file"}:
                raise SummaryError("present artifact manifest declares an unsafe root: {}".format(path))
        expected_dependencies = {"fast_gicp", "small_gicp"}
        source_paths = {path for path, role in observed.items()
                        if role == "dependency_source_file"}
        install_paths = {path for path, role in observed.items()
                         if role == "dependency_install_file"}
        if source_paths != {"work/vendor/{}-extract".format(name)
                            for name in expected_dependencies} or \
                install_paths != {"work/vendor/{}-install".format(name)
                                  for name in expected_dependencies}:
            raise SummaryError("present dependency artifact roots are incomplete")


def _validate_resource_policy(value, release):
    expected = release["resource_policy"]
    functional = expected["functional"]
    policy = value.get("resource_policy")
    if (not isinstance(policy, dict) or
            policy.get("schema") != audit.FUNCTIONAL_RESOURCE_POLICY_SCHEMA or
            policy.get("schema_version") != audit.FUNCTIONAL_RESOURCE_POLICY_VERSION or
            policy.get("mode") != audit.FUNCTIONAL_RESOURCE_MODE or
            policy.get("max_build_workers") != 1 or
            policy.get("timing_authority") != audit.FUNCTIONAL_TIMING_AUTHORITY or
            policy.get("performance_gate_eligible") is not False):
        raise SummaryError("functional resource policy is missing or authoritative")
    if policy.get("priority", {}).get("nice") != functional["nice"] or \
            policy.get("priority", {}).get("ionice_class") != functional["ionice_class"]:
        raise SummaryError("functional priority policy drift")
    for name in ("preflight", "completion"):
        snapshot = policy.get(name)
        if not isinstance(snapshot, dict) or snapshot.get("status") != "PASS" or \
                snapshot.get("timing_authority") != audit.FUNCTIONAL_TIMING_AUTHORITY or \
                snapshot.get("performance_gate_eligible") is not False:
            raise SummaryError("functional resource {} snapshot is not PASS".format(name))
        checks = snapshot.get("checks")
        if not isinstance(checks, dict) or any(value is not True for value in checks.values()):
            raise SummaryError("functional resource {} checks are incomplete".format(name))
        if snapshot.get("forbidden_processes") or snapshot.get("docker", {}).get("idle") is not True:
            raise SummaryError("functional resource {} is contaminated by forbidden activity".format(name))
    metrics = policy.get("timing_metrics")
    top_metrics = value.get("performance_metrics")
    for candidate in (metrics, top_metrics):
        if (not isinstance(candidate, dict) or
                candidate.get("status") != audit.FUNCTIONAL_TIMING_AUTHORITY or
                candidate.get("performance_gate_eligible") is not False or
                candidate.get("rtf") is not None or candidate.get("peak_rss_bytes") is not None):
            raise SummaryError("authoritative timing/resource metrics were recorded")


def _validate_inner_container_binding(value, image, distro):
    """Bind the child receipt to the selected ROS distro and image row."""
    observed = value.get("container")
    if not isinstance(observed, dict) or set(observed) != {
            "expected_image", "observed_digest_env", "ros_distro", "platform", "source"}:
        raise SummaryError("inner container identity is missing or non-canonical")
    if (observed.get("expected_image") != image or
            observed.get("observed_digest_env") != image.get("digest") or
            observed.get("ros_distro") != distro or
            not isinstance(observed.get("platform"), str) or not observed["platform"] or
            observed.get("source") != "workflow_pinned_container_digest_env"):
        raise SummaryError("inner container identity is not bound to the selected distro/image")


def _validate_campaign_binding(value, release):
    """Require the sealed child row to belong to this precommitted campaign set."""
    expected = release.get("campaign_set")
    if value.get("campaign_set") != expected:
        raise SummaryError("inner receipt campaign_set identity drift")


def validate_dependency_capture_root(root, *, expected_plan=None):
    """Reopen one host capture receipt at the summary boundary.

    A provisioning capture is evidence of a connected phase only.  This
    adapter verifies the capture executor's full receipt and its v2 contract
    projection, but deliberately never turns ``REVIEW_REQUIRED`` into runtime
    authorization.  ``expected_plan`` is optional for callers that already
    have the immutable outer plan; when supplied it is checked by the same
    capture validator before the summary consumes the result.
    """
    root = Path(root).absolute()
    try:
        if root.is_symlink() or not root.is_dir():
            raise SummaryError("dependency capture root is not a regular directory")
        result = capture_executor.validate_receipt(root, expected_plan=expected_plan)
    except SummaryError:
        raise
    except Exception as exc:
        raise SummaryError("dependency capture receipt is invalid: {}".format(exc)) from exc
    receipt_path = root / capture_executor.RECEIPT_NAME
    try:
        _, value, receipt_sha, sidecar_sha = _read_receipt(receipt_path)
    except Exception as exc:
        raise SummaryError("dependency capture receipt cannot be reopened: {}".format(exc)) from exc
    capture_contract = value.get("capture_contract")
    if not isinstance(capture_contract, dict) or \
            capture_contract.get("gpgv_status_policy") != dependency_closure.GPGV_STATUS_POLICY or \
            capture_contract.get("gpg_inventory_policy") != dependency_closure.GPG_INVENTORY_POLICY:
        raise SummaryError("dependency capture gpgv/inventory policy identity drift")
    if capture_contract != dependency_closure.capture_contract():
        raise SummaryError("dependency capture v2 contract identity drift")
    rosdep_prepare = value.get("rosdep_prepare")
    if rosdep_prepare != capture_contract["rosdep_prepare"]:
        raise SummaryError("dependency capture rosdep_prepare projection drift")
    if result.get("benchmark_eligible") is not False or result.get("status") not in {
            "REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED"}:
        raise SummaryError("dependency capture is not explicitly non-promoting")
    return {
        "status": "REVIEW_REQUIRED_NONPROMOTING",
        "receipt_status": result["status"],
        "outcome": result["outcome"],
        "receipt_sha256": receipt_sha,
        "sidecar_sha256": sidecar_sha,
        "capture_contract": capture_contract,
        "rosdep_prepare": rosdep_prepare,
        "benchmark_eligible": False,
        # The capture validator has already reopened and checked these maps
        # against every release/package/signature record.  Keep the exact
        # projection in the summary so shared evidence is visible without
        # rerunning or independently inventing a second identity scheme.
        "reference_map": {
            field: dict(value["reference_map"][field])
            for field in capture_executor.REFERENCE_MAP_FIELDS
        },
        "reference_counts": dict(value["reference_counts"]),
        "reference_policy": capture_contract["reference_policy"],
    }


def _validate_leg_artifacts(path, value, profile, release, profile_path):
    row = (value.get("distro"), value.get("dependency_leg"))
    root = path.parent
    expected_root_name = "registration-plugin-release-{}-{}".format(*row)
    if path.name != release["artifact_integrity"]["receipt_name"] or root.name != expected_root_name:
        raise SummaryError("receipt is not in its declared fresh per-leg root: {}".format(path))
    try:
        audit._artifact_root(root)
    except audit.AuditError as exc:
        raise SummaryError(str(exc))
    if value.get("profile", {}).get("path") != str(profile_path):
        raise SummaryError("receipt profile path is not the current profile")
    profile_sha = _sha256(profile_path)
    if value.get("profile", {}).get("sha256") != profile_sha:
        raise SummaryError("receipt profile hash drift")
    current_source = audit._source_manifest(ROOT, profile)
    if current_source.get("status") != "PASS":
        raise SummaryError("current source manifest is not PASS")
    recorded_source = value.get("repository", {}).get("source_manifest", {})
    if (recorded_source.get("status") != "PASS" or
            recorded_source.get("manifest_sha256") != current_source["manifest_sha256"]):
        raise SummaryError("receipt source manifest is stale or incomplete")
    host_revalidation = _validate_host_receipt(
        path, value, profile, release, profile_path, current_source)
    expected_identity = audit.artifact_root_identity(
        logical_name=root.name, distro=row[0], dependency_leg=row[1],
        profile_sha256=profile_sha, source_manifest_sha256=current_source["manifest_sha256"],
    )
    if value.get("evidence_root") != expected_identity:
        raise SummaryError("receipt evidence-root identity drift")
    _validate_resource_policy(value, release)
    marker_path, marker = _artifact_json(root, "evidence_root.receipt.json", "evidence root marker")
    if marker != expected_identity:
        raise SummaryError("evidence-root marker mismatch")
    artifact_roles = set(release["artifact_integrity"]["required_roles"])
    value_for_binding = dict(value)
    value_for_binding["_artifact_required_roles"] = artifact_roles
    manifest = _manifest_binding(root, value_for_binding, expected_identity)
    _validate_required_roots(manifest, release, row[1])
    image = next(item["image"] for item in release["distros"] if item["name"] == row[0])
    _validate_campaign_binding(value, release)
    _validate_inner_container_binding(value, image, row[0])
    dependency_binding = value.get("dependency_environment")
    if not isinstance(dependency_binding, dict) or \
            dependency_binding.get("path_relative") != DEPENDENCY_ENVIRONMENT_RELATIVE:
        raise SummaryError("dependency environment binding is missing")
    dependency_command_sha = dependency_contract.dependency_install_command_sha256()
    dependency_result = _validate_dependency_environment_manifest(
        root, dependency_binding["path_relative"], row[0], image,
        dependency_command_sha,
        _sha256(ROOT / "scripts/capture_registration_plugin_dependency_environment.py"),
    )
    if (dependency_binding.get("sha256") != dependency_result["sha256"] or
            dependency_binding.get("size_bytes") != dependency_result["size_bytes"] or
            dependency_binding.get("sidecar_sha256") != dependency_result["sidecar_sha256"]):
            raise SummaryError("dependency environment binding bytes changed")
    entry_by_path = {entry["path"]: entry for entry in manifest["entries"]}
    execution = value.get("execution")
    if not isinstance(execution, dict) or execution.get("authority") != HOST_PROMOTION_AUTHORITY:
        raise SummaryError("receipt is not a host-bound promotion execution")
    expected_archive_network = row[1] == "present"
    expected_phase = {
        "provisioning_network_used": True,
        "archive_fetch_network_used": expected_archive_network,
        "build_test_network_connected": False,
        "build_test_network_used": False,
        "network_after_dependency_fetch": False,
    }
    for key, expected in expected_phase.items():
        if execution.get(key) is not expected:
            raise SummaryError("execution network phase {} is not truthful".format(key))
    if "network_used" in execution:
        raise SummaryError("ambiguous execution network_used field is forbidden")
    prefetch_binding = value.get("dependency_prefetch")
    if row[1] == "present":
        if not isinstance(prefetch_binding, dict) or prefetch_binding.get("status") != "PASS":
            raise SummaryError("present leg prefetch binding is missing")
        if (prefetch_binding.get("authority") != HOST_PROMOTION_AUTHORITY or
                prefetch_binding.get("receipt_path_relative") != dependency_prefetch.PREFETCH_RECEIPT_NAME or
                prefetch_binding.get("manifest_path_relative") != dependency_prefetch.PREFETCH_MANIFEST_NAME):
            raise SummaryError("present leg prefetch authority/path binding is invalid")
        for key in ("receipt_sha256", "receipt_sidecar_sha256", "manifest_sha256",
                    "manifest_sidecar_sha256"):
            if not isinstance(prefetch_binding.get(key), str) or \
                    not audit.SHA256_RE.fullmatch(prefetch_binding[key]):
                raise SummaryError("present leg prefetch hash binding is invalid")
        records = prefetch_binding.get("records")
        if not isinstance(records, list) or [item.get("name") for item in records] != [
                "fast_gicp", "small_gicp"]:
            raise SummaryError("present leg prefetch record coverage is invalid")
        for record in records:
            if (record.get("archive_path_relative") != "prefetch/archives/{}.tar.gz".format(record["name"]) or
                    type(record.get("size_bytes")) is not int or record["size_bytes"] <= 0 or
                    not isinstance(record.get("archive_sha256"), str) or
                    not audit.SHA256_RE.fullmatch(record["archive_sha256"])):
                raise SummaryError("present leg prefetch archive binding is invalid")
        for relative, role in (
                (dependency_prefetch.PREFETCH_RECEIPT_NAME, "dependency_prefetch_receipt"),
                (dependency_prefetch.PREFETCH_RECEIPT_NAME + ".sha256",
                 "dependency_prefetch_receipt_sidecar"),
                (dependency_prefetch.PREFETCH_MANIFEST_NAME, "dependency_prefetch_manifest"),
                (dependency_prefetch.PREFETCH_MANIFEST_NAME + ".sha256",
                 "dependency_prefetch_manifest_sidecar")):
            if entry_by_path.get(relative, {}).get("role") != role:
                raise SummaryError("prefetch artifact is not manifest-bound: {}".format(relative))
        for relative in (
                dependency_prefetch.PREFETCH_RECEIPT_NAME,
                dependency_prefetch.PREFETCH_MANIFEST_NAME):
            for candidate in (relative, relative + ".sha256"):
                candidate_path = audit._artifact_file(root, candidate, "prefetch artifact")
                if (stat.S_IMODE(candidate_path.stat().st_mode) != 0o444 or
                        candidate_path.stat().st_nlink != 1):
                    raise SummaryError("prefetch artifact mode/link invalid: {}".format(candidate))
            sidecar_relative = relative + ".sha256"
            sidecar_path = audit._artifact_file(root, sidecar_relative, "prefetch sidecar")
            tokens = sidecar_path.read_text(encoding="ascii").strip().split()
            if tokens != [entry_by_path[relative]["sha256"], Path(relative).name]:
                raise SummaryError("prefetch sidecar binding mismatch: {}".format(relative))
        for record in records:
            relative = record["archive_path_relative"]
            if entry_by_path.get(relative, {}).get("sha256") != record["archive_sha256"] or \
                    entry_by_path.get(relative + ".sha256", {}).get("role") != "dependency_prefetch_archive_sidecar":
                raise SummaryError("prefetch archive is not manifest-bound: {}".format(relative))
            archive_path = audit._artifact_file(root, relative, "prefetch archive")
            sidecar_path = audit._artifact_file(root, relative + ".sha256", "prefetch archive sidecar")
            if (stat.S_IMODE(archive_path.stat().st_mode) != 0o444 or
                    stat.S_IMODE(sidecar_path.stat().st_mode) != 0o444 or
                    archive_path.stat().st_nlink != 1 or sidecar_path.stat().st_nlink != 1):
                raise SummaryError("prefetch archive mode/link invalid: {}".format(relative))
            tokens = sidecar_path.read_text(encoding="ascii").strip().split()
            if tokens != [record["archive_sha256"], Path(relative).name]:
                raise SummaryError("prefetch archive sidecar mismatch: {}".format(relative))
    elif prefetch_binding != {"status": "NOT_APPLICABLE", "archive_fetch_network_used": False}:
        raise SummaryError("absent leg must declare no prefetch")
    try:
        audit.validate_tree_manifest(root, value.get("installed_tree"), "installed tree")
        audit.validate_tree_manifest(root, value.get("test_results"), "test results")
    except audit.AuditError as exc:
        raise SummaryError(str(exc))
    external = value.get("external_dso", {})
    dso_path = external.get("receipt_path_relative")
    dso_sidecar = external.get("sidecar_path_relative")
    if (not isinstance(dso_path, str) or not isinstance(dso_sidecar, str) or
            entry_by_path.get(dso_path, {}).get("role") != "dso_receipt" or
            entry_by_path.get(dso_sidecar, {}).get("role") != "dso_receipt_sidecar"):
        raise SummaryError("external DSO receipt is not bound to the manifest")
    _, dso_value = _artifact_json(root, dso_path, "DSO receipt")
    if dso_value.get("status") != "PASS" or dso_value.get("load_session_smoke", {}).get("status") != "PASS":
        raise SummaryError("external DSO receipt is not a full PASS")
    if external.get("receipt_sha256") != entry_by_path[dso_path]["sha256"] or \
            external.get("sidecar_sha256") != entry_by_path[dso_sidecar]["sha256"]:
        raise SummaryError("external DSO receipt hash binding mismatch")
    evidence_entries = {entry["path"]: entry for entry in manifest["entries"]}
    for relative, role, contract in (
            ("work/evidence/provenance.receipt.json", "provenance_receipt",
             "registration-plugin-provenance-evidence-v1"),
            ("work/evidence/resource_failure.receipt.json", "resource_failure_receipt",
             "registration-plugin-resource-failure-evidence-v1")):
        if evidence_entries.get(relative, {}).get("role") != role:
            raise SummaryError("required evidence receipt is not bound: {}".format(relative))
        _, evidence_value = _artifact_json(root, relative, role)
        if evidence_value.get("status") != "PASS" or evidence_value.get("contract_version") != contract or \
                evidence_value.get("root_identity") != expected_identity:
            raise SummaryError("required evidence receipt is not a root-bound PASS: {}".format(relative))
    for command in value.get("commands", {}).get("records", []):
        log_relative = command.get("log", {}).get("path_relative")
        if not isinstance(log_relative, str) or entry_by_path.get(log_relative, {}).get("role") != "command_log":
            raise SummaryError("command log is missing from artifact manifest")
    return {
        "root": str(root), "manifest_sha256": _sha256(root / "artifact_manifest.json"),
        "manifest_entry_count": manifest["entry_count"], "root_identity": expected_identity,
        "host_receipt": host_revalidation,
        "dependency_environment_projection_sha256":
            dependency_result["projection_sha256"],
    }


def _seal(path, value):
    path = Path(path).absolute()
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink():
        raise SummaryError("summary output is not fresh: {}".format(path))
    receipt, sidecar, digest = audit.seal_receipt(path, value)
    return {"path": receipt, "sidecar": sidecar, "sha256": digest,
            "sidecar_sha256": _sha256(Path(sidecar))}


def summarize(profile_path, receipts, output):
    profile, profile_path = audit._load_profile(Path(profile_path).absolute())
    release = audit._validate_release_matrix_profile(profile)
    capture_contract = audit._validate_capture_contract(
        ROOT, release["dependency_closure"]["capture_contract"])
    expected_source = None
    expected_campaign = release["campaign_set"]
    rows = []
    seen = set()
    for receipt_path in receipts:
        path, value, digest, sidecar_digest = _read_receipt(receipt_path)
        if value.get("contract_version") != "registration-plugin-release-matrix-v1":
            raise SummaryError("receipt contract mismatch: {}".format(path))
        row = (value.get("distro"), value.get("dependency_leg"))
        if row not in EXPECTED_ROWS or row in seen:
            raise SummaryError("unexpected or duplicate matrix row: {}".format(row))
        seen.add(row)
        if value.get("status") != "PASS":
            raise SummaryError("matrix row is not PASS: {} ({})".format(row, value.get("status")))
        artifact_revalidation = _validate_leg_artifacts(
            path, value, profile, release, profile_path
        )
        source = value.get("repository", {}).get("source_manifest", {})
        if source.get("status") != "PASS":
            raise SummaryError("current source manifest is not PASS: {}".format(row))
        if value.get("campaign_set") != expected_campaign:
            raise SummaryError("matrix row campaign_set identity drift: {}".format(row))
        if expected_source is None:
            expected_source = source.get("manifest_sha256")
        elif source.get("manifest_sha256") != expected_source:
            raise SummaryError("source manifest drift across matrix rows")
        safety = value.get("safety", {})
        expected_archive_network = row[1] == "present"
        for key, expected in {
                "provisioning_network_used": True,
                "archive_fetch_network_used": expected_archive_network,
                "build_test_network_connected": False,
                "build_test_network_used": False,
        }.items():
            if safety.get(key) is not expected:
                raise SummaryError("safety network phase {} is not truthful: {}".format(key, row))
        if "network_used" in safety:
            raise SummaryError("ambiguous safety network_used field is forbidden")
        if any(safety.get(key) is not False for key in (
                "bag_opened", "ground_truth_content_opened", "scorer_invoked",
                "map_saved", "formal_replay_started")):
            raise SummaryError("safety field is not false: {}".format(row))
        if value.get("installed_tree", {}).get("status") != "PASS":
            raise SummaryError("installed tree is not an independent PASS: {}".format(row))
        if value.get("external_dso", {}).get("status") != "PASS" or \
                value.get("external_dso", {}).get("load_session_smoke") != "PASS":
            raise SummaryError("external DSO gate is not PASS: {}".format(row))
        rows.append({"distro": row[0], "dependency_leg": row[1],
                     "status": value["status"], "receipt_path": str(path),
                     "receipt_sha256": digest, "sidecar_sha256": sidecar_digest,
                     "source_manifest_sha256": expected_source,
                     "dependency_environment_projection_sha256":
                         artifact_revalidation["dependency_environment_projection_sha256"]})
    if seen != EXPECTED_ROWS:
        raise SummaryError("matrix is incomplete: {}".format(sorted(EXPECTED_ROWS - seen)))
    dependency_rows = {}
    for row in rows:
        key = (row["distro"], row["dependency_leg"])
        dependency_rows[key] = row["dependency_environment_projection_sha256"]
    dependency_closures = {}
    for distro in ("humble", "jazzy"):
        absent = dependency_rows[(distro, "absent")]
        present = dependency_rows[(distro, "present")]
        if absent != present:
            raise SummaryError(
                "base dependency closure drift across {} absent/present legs".format(distro)
            )
        dependency_closures[distro] = {
            "status": "PASS", "projection_sha256": absent,
            "compared_legs": ["absent", "present"],
        }
    report = {
        "schema_version": 1, "contract_version": CONTRACT, "status": "PASS",
        "profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
        "rows": sorted(rows, key=lambda item: (item["distro"], item["dependency_leg"])),
        "source_manifest_sha256": expected_source,
        "campaign_set": expected_campaign,
        "dependency_closures": dependency_closures,
        "dependency_capture_contract": capture_contract,
        "promotion": "external_plugin_release_matrix_current_source",
        "safety": {"authority": HOST_PROMOTION_AUTHORITY,
                    "provisioning_network_used": True,
                    "archive_fetch_network_used_by_leg": {
                        "absent": False, "present": True,
                    },
                    "build_test_network_connected": False,
                    "build_test_network_used": False,
                    "bag_opened": False,
                    "ground_truth_content_opened": False, "scorer_invoked": False,
                    "map_saved": False, "formal_replay_started": False},
    }
    return _seal(output, report), report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=str(ROOT / "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"))
    parser.add_argument("--receipt", action="append", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        result, report = summarize(args.profile, args.receipt, args.output)
        print(json.dumps({"status": report["status"], "receipt": result}, sort_keys=True))
        return 0
    except Exception as exc:
        failure = {"schema_version": 1, "contract_version": CONTRACT,
                   "status": "FAIL_CLOSED", "failure": str(exc),
                   "safety": {"authority": "FAIL_CLOSED",
                               "provisioning_network_used": False,
                               "archive_fetch_network_used": False,
                               "build_test_network_connected": False,
                               "build_test_network_used": False,
                               "bag_opened": False,
                               "ground_truth_content_opened": False, "scorer_invoked": False,
                               "map_saved": False, "formal_replay_started": False}}
        try:
            result = _seal(args.output, failure)
            failure["receipt"] = result
        except Exception as seal_error:
            failure["receipt_seal_error"] = str(seal_error)
        print(json.dumps(failure, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
