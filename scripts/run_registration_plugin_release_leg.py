#!/usr/bin/env python3
"""Own one isolated registration-plugin release-leg container lifecycle.

The in-container runner remains the authority for compilation and evidence
contracts.  This host launcher owns the surrounding identity, network phase,
diagnostics, and stopped-only cleanup so a manual ``docker exec`` cannot omit
the pinned identity environment.  It never retries a command or promotes
functional timing to a benchmark result.
"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

# Direct source-checkout execution puts ``scripts/`` (not the checkout root)
# on ``sys.path``.  Bootstrap only the canonical checkout that owns this
# launcher; installed/container execution keeps normal package resolution.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

from lidarslam_benchmark_tools import (  # noqa: E402
    audit_registration_plugin_matrix as audit,
    registration_plugin_evidence_directory as evidence_directories,
    registration_plugin_dependency_contract as dependency_contract,
    registration_plugin_dependency_closure as dependency_closure,
    registration_plugin_dependency_prefetch as dependency_prefetch,
    run_registration_plugin_release_matrix as release_runner,
    summarize_registration_plugin_release_matrix as release_summary,
)

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = "registration-plugin-host-release-leg-v1"
CONTAINER_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,62}$")
HEX_ID_RE = re.compile(r"^[0-9a-f]{12,64}$")
FORBIDDEN_PATH_CHARS = ("\x00", "\n", "\r")
MOUNTINFO_PATH = Path("/proc/self/mountinfo")
DEVICE_UUID_PATH = Path("/dev/disk/by-uuid")
MOUNTINFO_ESCAPE_RE = re.compile(r"\\([0-7]{3})")
REQUIRED_ENV_KEYS = (
    "ROS_DISTRO",
    "REGISTRATION_PLUGIN_CONTAINER_DIGEST",
    "REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY",
    "REGISTRATION_PLUGIN_RESOURCE_POLICY",
    "CMAKE_BUILD_PARALLEL_LEVEL",
    "MAKEFLAGS",
    "NINJAFLAGS",
)

DEPENDENCY_SCRIPT = dependency_contract.DEPENDENCY_INSTALL_COMMAND
DEPENDENCY_CAPTURE_SCRIPT = "/workspace/src/scripts/capture_registration_plugin_dependency_environment.py"
DEPENDENCY_MANIFEST_NAME = "dependency_environment.receipt.json"
HOST_PROMOTION_AUTHORITY = "HOST_PROMOTION"
FUNCTIONAL_CI_AUTHORITY = "FUNCTIONAL_CI_NON_PROMOTING"


class LauncherError(RuntimeError):
    """A fail-closed host-launcher error."""

    def __init__(self, kind, message, details=None):
        super(LauncherError, self).__init__(message)
        self.kind = kind
        self.details = details


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path):
    return audit.sha256_file(path)


def _safe_string(value, label):
    raw = os.fspath(value)
    if any(character in raw for character in FORBIDDEN_PATH_CHARS):
        raise LauncherError("UNSAFE_PATH", "{} contains control characters".format(label))
    return raw


def _canonical_existing_directory(value, label):
    raw = _safe_string(value, label)
    path = Path(raw)
    try:
        canonical = path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("PATH_INVALID", "{} cannot be canonicalized: {}".format(label, exc))
    if not canonical.is_dir() or canonical.is_symlink():
        raise LauncherError("PATH_INVALID", "{} is not a regular directory".format(label))
    return canonical


def _fresh_root_target(value, label):
    raw = _safe_string(value, label)
    path = Path(raw)
    if not path.is_absolute():
        raise LauncherError("PATH_NOT_ABSOLUTE", "{} must be absolute".format(label))
    if any(part in ("..", ".") for part in path.parts):
        raise LauncherError("UNSAFE_PATH", "{} contains traversal components".format(label))
    try:
        parent = path.parent.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("PATH_INVALID", "{} parent cannot be canonicalized: {}".format(label, exc))
    if not parent.is_dir() or parent.is_symlink():
        raise LauncherError("PATH_INVALID", "{} parent is not a regular directory".format(label))
    canonical = parent / path.name
    if canonical.exists() or canonical.is_symlink():
        raise LauncherError("ROOT_NOT_FRESH", "{} already exists: {}".format(label, canonical))
    return canonical


def _decode_mountinfo_path(value):
    return MOUNTINFO_ESCAPE_RE.sub(
        lambda match: chr(int(match.group(1), 8)), value)


def _read_mountinfo():
    """Return live mount entries without trusting the mountpoint directory."""
    try:
        lines = MOUNTINFO_PATH.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise LauncherError("EVIDENCE_MOUNT_INSPECTION_FAILED", str(exc))
    entries = []
    for line in lines:
        before, separator, after = line.partition(" - ")
        fields = before.split()
        post_fields = after.split()
        if not separator or len(fields) < 6 or len(post_fields) < 2:
            continue
        entries.append({
            "mountpoint": _decode_mountinfo_path(fields[4]),
            "filesystem": post_fields[0],
            "source": _decode_mountinfo_path(post_fields[1]),
        })
    return entries


def _device_uuid(source):
    try:
        canonical_source = Path(source).resolve(strict=True)
        candidates = sorted(DEVICE_UUID_PATH.iterdir(), key=lambda path: path.name)
    except OSError:
        return None
    for candidate in candidates:
        try:
            if candidate.resolve(strict=True) == canonical_source:
                return candidate.name.lower()
        except OSError:
            continue
    return None


def _mounted_storage_identity(mountpoint):
    try:
        canonical_mountpoint = Path(mountpoint).resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("EVIDENCE_MOUNT_UNAVAILABLE", str(exc))
    for entry in _read_mountinfo():
        if Path(entry["mountpoint"]) != canonical_mountpoint:
            continue
        source_uuid = _device_uuid(entry["source"])
        return {
            "mountpoint": str(canonical_mountpoint),
            "filesystem": entry["filesystem"],
            "source": entry["source"],
            "source_uuid": source_uuid,
        }
    return None


def _validate_evidence_storage(evidence_root, storage):
    """Fail closed unless the evidence root is on the profile-bound mount."""
    contract = audit._validate_evidence_storage_contract(storage)
    root_raw = Path(_safe_string(evidence_root, "evidence root"))
    if not root_raw.is_absolute() or any(part in ("..", ".") for part in root_raw.parts):
        raise LauncherError("UNSAFE_PATH", "evidence root is not an absolute normalized path")
    mountpoint_raw = Path(contract["mountpoint"])
    if mountpoint_raw.is_symlink():
        raise LauncherError("EVIDENCE_MOUNT_UNAVAILABLE", "evidence mountpoint is a symlink")
    try:
        mountpoint = mountpoint_raw.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("EVIDENCE_MOUNT_UNAVAILABLE", str(exc))
    if not mountpoint.is_dir() or mountpoint.is_symlink():
        raise LauncherError("EVIDENCE_MOUNT_UNAVAILABLE", "evidence mountpoint is not a directory")
    try:
        parent = root_raw.parent.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("PATH_INVALID", "evidence root parent cannot be canonicalized: {}".format(exc))
    if not parent.is_dir() or parent.is_symlink():
        raise LauncherError("PATH_INVALID", "evidence root parent is not a regular directory")
    try:
        parent.relative_to(mountpoint)
    except ValueError:
        raise LauncherError(
            "EVIDENCE_MOUNT_PATH_MISMATCH",
            "evidence root parent is outside the profile mountpoint",
        )
    observed = _mounted_storage_identity(mountpoint)
    if observed is None:
        raise LauncherError(
            "EVIDENCE_MOUNT_UNAVAILABLE",
            "profile evidence mountpoint is not mounted",
        )
    mismatches = {}
    for key in ("filesystem", "source_uuid"):
        if observed.get(key) != contract.get(key):
            mismatches[key] = {
                "expected": contract.get(key),
                "observed": observed.get(key),
            }
    if mismatches:
        raise LauncherError(
            "EVIDENCE_MOUNT_IDENTITY_MISMATCH",
            _canonical({"mountpoint": str(mountpoint), "mismatches": mismatches}),
        )
    return {"contract": contract, "observed": observed}


def _reserve_root(path):
    try:
        evidence_directories.create_fresh_root(path)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise LauncherError(exc.kind, str(exc)) from exc
    return Path(path)


def _host_directory_layout(plan, root):
    """Create the fixed host-owned layout before Docker can see the root."""
    child = Path(plan["evidence_child"])
    root = Path(root)
    try:
        child_relative = child.relative_to(root).as_posix()
    except ValueError as exc:
        raise LauncherError("EVIDENCE_LAYOUT_PATH_INVALID", str(child)) from exc
    dependencies = [item["name"] for item in plan["leg"].get("dependencies", [])]
    paths = evidence_directories.launcher_paths(
        child_relative, dependencies, include_prefetch=bool(dependencies))
    try:
        contract = evidence_directories.create_layout(root, paths)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise LauncherError(exc.kind, str(exc)) from exc
    return contract


def _verify_host_directory_layout(root, contract):
    """Reopen every host-owned directory and require exact identity stability."""
    try:
        after = evidence_directories.verify_snapshot(root, contract)
        return evidence_directories.finalize(contract, after)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise LauncherError(exc.kind, str(exc)) from exc


def _container_name(root, distro, dependency_leg):
    seed = "registration-plugin-{}-{}-{}".format(
        distro, dependency_leg, root.name.lower())
    if len(seed) > 63:
        suffix = hashlib.sha256(str(root).encode("utf-8")).hexdigest()[:12]
        seed = "registration-plugin-{}-{}-{}".format(distro, dependency_leg, suffix)
    if not CONTAINER_NAME_RE.fullmatch(seed):
        raise LauncherError("CONTAINER_NAME_INVALID", seed)
    return seed


def _command_hash(argv):
    return _sha256_bytes(json.dumps(list(argv), separators=(",", ":")).encode("utf-8"))


def _write_exclusive(path, payload, mode=0o644):
    path = Path(path)
    if path.exists() or path.is_symlink():
        raise LauncherError("ARTIFACT_OVERWRITE", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise LauncherError("ARTIFACT_PARENT_INVALID", str(path.parent))
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(str(path), flags, mode)
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
    return path


def _priority_argv(argv):
    return release_runner._priority_argv(argv, nice_level=19, ionice_class=3)


def _normalize_output(value):
    if value is None:
        return b""
    if isinstance(value, bytes):
        return value
    return str(value).encode("utf-8", errors="replace")


def _run_host_command(argv, label, log_path, executor=None, priority=False, allow_failure=True):
    raw_argv = [str(item) for item in argv]
    effective = _priority_argv(raw_argv) if priority and executor is None else raw_argv
    if executor is None:
        completed = subprocess.run(
            effective, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            check=False,
        )
    else:
        completed = executor(raw_argv, label)
    output = _normalize_output(getattr(completed, "stdout", completed if isinstance(completed, (bytes, str)) else ""))
    if getattr(completed, "stderr", None):
        output += _normalize_output(completed.stderr)
    record = {
        "label": label,
        "argv": raw_argv,
        "argv_sha256": _command_hash(raw_argv),
        "effective_argv": effective,
        "priority": {"nice": 19, "ionice_class": 3,
                      "applied": effective != raw_argv},
        "returncode": int(getattr(completed, "returncode", 0)),
    }
    if log_path is not None:
        payload = output
        _write_exclusive(log_path, payload)
        record["log"] = {"path": str(log_path), "bytes": len(payload),
                          "sha256": _sha256_bytes(payload)}
    if record["returncode"] != 0 and not allow_failure:
        raise LauncherError("COMMAND_FAILED", "{} exited {}".format(label, record["returncode"]))
    return record, output


def _parse_inspect(output, label):
    try:
        value = json.loads(output.decode("utf-8"))
        if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], dict):
            raise ValueError("expected one inspect object")
        return value[0]
    except (ValueError, UnicodeDecodeError) as exc:
        raise LauncherError("DOCKER_INSPECT_INVALID", "{}: {}".format(label, exc))


def _absence_error_lines(name):
    return (
        "Error: No such object: {}".format(name),
        "Error: No such container: {}".format(name),
        "error: no such object: {}".format(name),
        "error: no such container: {}".format(name),
    )


def _validate_absence_output(output, name):
    """Accept only Docker's exact single-line not-found response."""
    try:
        text = output.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise LauncherError("DOCKER_INSPECT_ABSENCE_UNPROVEN", str(exc))
    allowed = _absence_error_lines(name)
    expected = tuple(line + suffix for line in allowed for suffix in ("", "\n"))
    expected += tuple("[]\n" + line for line in expected)
    if text not in expected:
        raise LauncherError(
            "DOCKER_INSPECT_ABSENCE_UNPROVEN",
            "unexpected docker inspect absence output for {}".format(name),
        )


def _docker_inspect(name, log_path, executor=None):
    record, output = _run_host_command(
        ["docker", "inspect", name], "inspect_" + name, log_path,
        executor=executor, priority=False, allow_failure=True)
    if record["returncode"] != 0:
        if record["returncode"] != 1:
            raise LauncherError(
                "DOCKER_INSPECT_ABSENCE_UNPROVEN",
                "docker inspect {} exited {}".format(name, record["returncode"]),
            )
        _validate_absence_output(output, name)
        return None, record
    return _parse_inspect(output, "container inspect"), record


def _validate_container_identity(observed, expected, container_id=None):
    if not isinstance(observed, dict):
        raise LauncherError("CONTAINER_IDENTITY_MISMATCH", "container inspect is unavailable")
    observed_id = str(observed.get("Id", "")).split("sha256:")[-1]
    if container_id is not None and not observed_id.startswith(str(container_id)):
        raise LauncherError("CONTAINER_IDENTITY_MISMATCH", "container id changed")
    if observed.get("Name", "").lstrip("/") != expected["name"]:
        raise LauncherError("CONTAINER_IDENTITY_MISMATCH", "container name changed")
    if observed.get("Config", {}).get("Image") != expected["image_reference"]:
        raise LauncherError("CONTAINER_IDENTITY_MISMATCH", "container image reference changed")
    if expected.get("image_id") is not None and observed.get("Image") != expected["image_id"]:
        raise LauncherError("CONTAINER_IDENTITY_MISMATCH", "container image digest changed")
    mounts = observed.get("Mounts")
    if not isinstance(mounts, list) or len(mounts) != 2:
        raise LauncherError("CONTAINER_MOUNT_MISMATCH", "expected exactly two mounts")
    by_destination = {item.get("Destination"): item for item in mounts}
    expected_mounts = expected.get("mounts")
    if not isinstance(expected_mounts, list) or len(expected_mounts) != 2:
        raise LauncherError("CONTAINER_MOUNT_MISMATCH", "expected mount contract is incomplete")
    if set(by_destination) != {item["destination"] for item in expected_mounts}:
        raise LauncherError("CONTAINER_MOUNT_MISMATCH", "container mount destinations changed")
    for contract in expected_mounts:
        observed_mount = by_destination.get(contract["destination"])
        expected_rw = contract["mode"] == "rw"
        if (not isinstance(observed_mount, dict) or observed_mount.get("Type") != "bind" or
                observed_mount.get("Source") != contract["source"] or
                observed_mount.get("Destination") != contract["destination"] or
                observed_mount.get("RW") is not expected_rw or
                observed_mount.get("Mode") != contract["mode"]):
            raise LauncherError(
                "CONTAINER_MOUNT_MISMATCH",
                "container mount identity changed: {}".format(contract["destination"]),
            )
    return True


def _validate_image_identity(value, image_contract, label="image"):
    if not isinstance(value, dict):
        raise LauncherError("IMAGE_IDENTITY_MISMATCH", "{} inspect unavailable".format(label))
    expected_digest = image_contract["digest"]
    if value.get("Id") != expected_digest:
        raise LauncherError("IMAGE_IDENTITY_MISMATCH", "expected {} observed {}".format(expected_digest, value.get("Id")))
    repo_digests = value.get("RepoDigests") or []
    if not any(str(item).endswith("@" + expected_digest) for item in repo_digests):
        raise LauncherError("IMAGE_IDENTITY_MISMATCH", "RepoDigests does not bind exact digest")
    if value.get("Os") != "linux" or value.get("Architecture") != "amd64":
        raise LauncherError("IMAGE_PLATFORM_MISMATCH", "expected linux/amd64")
    return {
        "id": value.get("Id"), "repo_digests": repo_digests,
        "os": value.get("Os"), "architecture": value.get("Architecture"),
        "manifest": image_contract.get("manifest"),
    }


def _build_runner_shell(distro, dependency_leg):
    setup = "/opt/ros/{}/setup.bash".format(distro)
    args = [
        "python3", "/workspace/src/scripts/run_registration_plugin_release_matrix.py",
        "--repo-root", "/workspace/src",
        "--profile", "/workspace/src/configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json",
        "--distro", distro,
        "--dependency-leg", dependency_leg,
        "--evidence-root", "/workspace/evidence-parent/registration-plugin-release-{}-{}".format(distro, dependency_leg),
        "--output", "/workspace/evidence-parent/registration-plugin-release-{}-{}/registration_plugin_release.receipt.json".format(distro, dependency_leg),
    ]
    quoted = " ".join(shlex.quote(item) for item in args)
    return "source {} && PYTHONPATH=/workspace/src:/workspace/src/scripts exec {}".format(
        shlex.quote(setup), quoted)


def _dependency_capture_argv(plan, dependency_record):
    """Build the one capture command required before network disconnect."""
    command_sha = _sha256_bytes(DEPENDENCY_SCRIPT.encode("utf-8"))
    capture_sha = _sha256_file(
        ROOT / "scripts/capture_registration_plugin_dependency_environment.py"
    )
    output = "/workspace/evidence-parent/registration-plugin-release-{}-{}/{}".format(
        plan["distro"], plan["dependency_leg"], DEPENDENCY_MANIFEST_NAME
    )
    return [
        "docker", "exec", plan["container_name"], "python3",
        DEPENDENCY_CAPTURE_SCRIPT,
        "--output", output,
        "--distro", plan["distro"],
        "--image-digest", plan["image"]["digest"],
        "--install-command-sha256", command_sha,
        "--install-exit-code", str(dependency_record["returncode"]),
        "--capture-script-sha256", capture_sha,
    ]


def build_plan(repo_root, profile_path, distro, dependency_leg, evidence_root):
    """Validate all identity/path inputs and return a pure launch plan."""
    if distro not in ("humble", "jazzy") or dependency_leg not in ("absent", "present"):
        raise LauncherError("MATRIX_ROW_INVALID", "unsupported distro or dependency leg")
    repo = _canonical_existing_directory(repo_root, "repository root")
    profile_raw = _safe_string(profile_path, "profile path")
    profile_path_obj = Path(profile_raw)
    try:
        canonical_profile = profile_path_obj.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise LauncherError("PATH_INVALID", "profile path cannot be canonicalized: {}".format(exc))
    if not canonical_profile.is_file() or canonical_profile.is_symlink():
        raise LauncherError("PATH_INVALID", "profile path is not a regular file")
    profile, canonical_profile = audit._load_profile(canonical_profile)
    release = audit._validate_release_matrix_profile(profile)
    plan = release_runner.validate_release_plan(profile, distro, dependency_leg)
    if plan.get("status") != "READY":
        raise LauncherError("MATRIX_PLAN_NOT_READY", _canonical(plan))
    source = audit._source_manifest(repo, profile)
    if source.get("status") != "PASS":
        raise LauncherError("CURRENT_SOURCE_HASH_MISMATCH", _canonical(source))
    git = audit._git_snapshot(repo)
    row = next(item for item in release["distros"] if item["name"] == distro)
    image_contract = audit._validate_image_contract(
        row["image"], distro, require_humble_manifest=distro == "humble")
    evidence_storage = _validate_evidence_storage(
        evidence_root, release["evidence_storage"])
    root = _fresh_root_target(evidence_root, "evidence root")
    name = _container_name(root, distro, dependency_leg)
    resource = release["resource_policy"]
    functional = resource["functional"]
    env = {
        "ROS_DISTRO": distro,
        "REGISTRATION_PLUGIN_CONTAINER_DIGEST": image_contract["digest"],
        "REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY": release["policy"]["network_after_dependency_fetch"],
        "REGISTRATION_PLUGIN_RESOURCE_POLICY": functional["mode"],
        "CMAKE_BUILD_PARALLEL_LEVEL": "1",
        "MAKEFLAGS": "-j1",
        "NINJAFLAGS": "-j1",
    }
    if any(key not in env or not env[key] for key in REQUIRED_ENV_KEYS):
        raise LauncherError("REQUIRED_ENV_MISSING", "launcher environment is incomplete")
    evidence_child = root / "registration-plugin-release-{}-{}".format(distro, dependency_leg)
    output = evidence_child / "registration_plugin_release.receipt.json"
    dependency_names = [item["name"] for item in plan["leg"].get("dependencies", [])]
    child_relative = evidence_child.relative_to(root).as_posix()
    host_directory_relatives = evidence_directories.launcher_paths(
        child_relative, dependency_names, include_prefetch=bool(dependency_names))
    image_reference = image_contract["reference"]
    mounts = [
        {"source": str(repo), "destination": "/workspace/src", "mode": "ro"},
        {"source": str(root), "destination": "/workspace/evidence-parent", "mode": "rw"},
    ]
    run_argv = [
        "docker", "run", "-d", "--name", name,
    ]
    for key in REQUIRED_ENV_KEYS:
        run_argv.extend(["--env", "{}={}".format(key, env[key])])
    run_argv.extend([
        "-v", "{}:/workspace/src:ro".format(repo),
        "-v", "{}:/workspace/evidence-parent:rw".format(root),
        image_reference, "tail", "-f", "/dev/null",
    ])
    dependency_argv = ["docker", "exec", name, "bash", "-lc", DEPENDENCY_SCRIPT]
    runner_argv = ["docker", "exec"]
    for key in REQUIRED_ENV_KEYS:
        runner_argv.extend(["--env", "{}={}".format(key, env[key])])
    # These values are applied only to the post-disconnect runner command.
    # The host proves the network state immediately before executing it.
    runner_env = {
        "REGISTRATION_PLUGIN_EXECUTION_AUTHORITY": HOST_PROMOTION_AUTHORITY,
        "REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED": "false",
        "REGISTRATION_PLUGIN_HOST_DISCONNECT_VERIFIED": "true",
    }
    for key, value in sorted(runner_env.items()):
        runner_argv.extend(["--env", "{}={}".format(key, value)])
    runner_argv.extend([name, "bash", "-lc", _build_runner_shell(distro, dependency_leg)])
    return {
        "schema": CONTRACT, "schema_version": 1,
        "repo_root": str(repo), "profile_path": str(canonical_profile),
        "profile_sha256": _sha256_file(canonical_profile),
        "source_manifest": source, "git": git,
        "distro": distro, "dependency_leg": dependency_leg,
        "campaign_set": release["campaign_set"],
        "image": image_contract, "image_reference": image_reference,
        "container_name": name, "evidence_root": str(root),
        "evidence_child": str(evidence_child), "output": str(output),
        "leg": plan["leg"],
        "host_directory_relatives": host_directory_relatives,
        "evidence_storage": evidence_storage,
        "mounts": mounts, "env": env,
        "runner_env": runner_env,
        "resource_policy": resource,
        "run_argv": run_argv, "dependency_argv": dependency_argv,
        "runner_argv": runner_argv,
    }


def _capture_diagnostics(plan, root, executor=None):
    logs = root / "launcher" / "diagnostics"
    try:
        evidence_directories.snapshot_existing(logs, (), owner=None)
    except evidence_directories.EvidenceDirectoryError as exc:
        raise LauncherError(exc.kind, str(exc)) from exc
    records = []
    for label, argv in (
            ("docker_inspect", ["docker", "inspect", plan["container_name"]]),
            ("docker_logs", ["docker", "logs", plan["container_name"]]),
            ("docker_stats", ["docker", "stats", "--no-stream", plan["container_name"]]),
            ("docker_top", ["docker", "top", plan["container_name"]])):
        record, _ = _run_host_command(argv, label, logs / (label + ".log"),
                                       executor=executor, priority=False, allow_failure=True)
        records.append(record)
    manifest = {"schema": "registration-plugin-host-diagnostics-v1", "container": plan["container_name"],
                "captured_before_cleanup": True, "records": records}
    payload = (_canonical(manifest) + "\n").encode("utf-8")
    manifest_path = _write_exclusive(root / "launcher" / "docker_diagnostics.json", payload)
    sidecar_payload = ("{}  {}\n".format(_sha256_bytes(payload), manifest_path.name)).encode("ascii")
    sidecar = _write_exclusive(Path(str(manifest_path) + ".sha256"), sidecar_payload)
    for path in list(logs.glob("*.log")) + [manifest_path, sidecar]:
        os.chmod(path, 0o444)
    return {"path": str(manifest_path), "sha256": _sha256_bytes(payload),
            "sidecar": str(sidecar), "sidecar_sha256": _sha256_bytes(sidecar_payload),
            "records": records}


def _inspect_container_for_cleanup(plan, executor, root, phase):
    if phase not in ("initial", "post_stop"):
        raise LauncherError("CLEANUP_PHASE_INVALID", "invalid cleanup phase: {}".format(phase))
    path = root / "launcher" / "cleanup_inspect_{}.log".format(phase)
    return _docker_inspect(plan["container_name"], path, executor=executor)


def _cleanup(plan, root, executor=None, container_id=None):
    cleanup = {"diagnostics_before_cleanup": True, "stop_requested": False,
               "remove_requested": False, "remove_forced": False,
               "post_remove_absent": False}
    observed, inspect_record = _inspect_container_for_cleanup(
        plan, executor, root, "initial")
    cleanup["initial_inspect"] = inspect_record
    if observed is None:
        cleanup["error"] = "container identity unavailable; removal refused"
        return cleanup
    try:
        _validate_container_identity(observed, {
            "name": plan["container_name"],
            "image_reference": plan["image_reference"],
            "image_id": plan["image"]["digest"],
            "mounts": plan["mounts"],
        },
                                     container_id=container_id)
    except LauncherError as exc:
        cleanup["error"] = str(exc)
        return cleanup
    state = observed.get("State", {})
    if state.get("Running"):
        cleanup["stop_requested"] = True
        stop_record, _ = _run_host_command(
            ["docker", "stop", plan["container_name"]], "docker_stop",
            root / "launcher" / "docker_stop.log", executor=executor,
            priority=False, allow_failure=True)
        cleanup["stop"] = stop_record
    observed_after, after_record = _inspect_container_for_cleanup(
        plan, executor, root, "post_stop")
    cleanup["post_stop_inspect"] = after_record
    if observed_after is None:
        cleanup["error"] = "container disappeared before stopped-only removal"
        return cleanup
    try:
        _validate_container_identity(observed_after, {
            "name": plan["container_name"],
            "image_reference": plan["image_reference"],
            "image_id": plan["image"]["digest"],
            "mounts": plan["mounts"],
        },
                                     container_id=container_id)
    except LauncherError as exc:
        cleanup["error"] = str(exc)
        return cleanup
    if observed_after.get("State", {}).get("Running"):
        cleanup["error"] = "container is still running; removal refused"
        return cleanup
    cleanup["stopped"] = True
    cleanup["oom_killed"] = bool(observed_after.get("State", {}).get("OOMKilled"))
    cleanup["exit_code"] = observed_after.get("State", {}).get("ExitCode")
    cleanup["remove_requested"] = True
    remove_record, _ = _run_host_command(
        ["docker", "rm", plan["container_name"]], "docker_remove",
        root / "launcher" / "docker_remove.log", executor=executor,
        priority=False, allow_failure=True)
    cleanup["remove"] = remove_record
    absent, absent_record = _docker_inspect(
        plan["container_name"], root / "launcher" / "post_remove_inspect.log", executor=executor)
    cleanup["post_remove_inspect"] = absent_record
    cleanup["post_remove_absent"] = absent is None
    for path in (root / "launcher").rglob("*.log"):
        os.chmod(path, 0o444)
    return cleanup


def _seal_closure(root, report):
    path = root / "registration_plugin_host.receipt.json"
    receipt, sidecar, digest = audit.seal_receipt(path, report)
    return {"path": receipt, "sidecar": sidecar, "sha256": digest,
            "sidecar_sha256": _sha256_file(sidecar)}


def run_leg(repo_root, profile_path, distro, dependency_leg, evidence_root,
            executor=None, sleep=time.sleep, prefetcher=None,
            dependency_closure_root=None):
    plan = build_plan(repo_root, profile_path, distro, dependency_leg, evidence_root)
    if dependency_closure_root is None:
        raise LauncherError(
            "DEPENDENCY_CLOSURE_REQUIRED",
            "a signed PASS dependency closure is required before a release leg",
        )
    try:
        dependency_closure.validate_runtime_closure(
            Path(dependency_closure_root), distro=distro,
            image_digest=plan["image"]["digest"], dependency_leg=dependency_leg)
    except Exception as exc:
        raise LauncherError(
            getattr(exc, "kind", "DEPENDENCY_CLOSURE_INVALID"), str(exc)
        ) from exc
    _validate_evidence_storage(
        plan["evidence_root"], plan["evidence_storage"]["contract"])
    root = _reserve_root(Path(plan["evidence_root"]))
    _validate_evidence_storage(
        root, plan["evidence_storage"]["contract"])
    container_started = False
    container_id = None
    commands = []
    first_failure = None
    diagnostics = None
    cleanup = {"status": "NOT_STARTED"}
    preflight = None
    image_identity = None
    runner_receipt = None
    artifact_revalidation = {"status": "NOT_REACHED"}
    dependency_environment = {"status": "NOT_REACHED"}
    dependency_prefetch_evidence = {"status": "NOT_APPLICABLE" if dependency_leg == "absent" else "NOT_REACHED"}
    runner_receipt_binding = {"status": "NOT_REACHED"}
    directory_contract = None
    try:
        directory_contract = _host_directory_layout(plan, root)
        preflight = release_runner._functional_resource_snapshot(
            plan["resource_policy"], root.parent,
            excluded_pids=release_runner._proc_ancestors(os.getpid()))
        if preflight["status"] != "PASS":
            raise LauncherError("FUNCTIONAL_RESOURCE_PREFLIGHT_FAIL_CLOSED", _canonical(preflight))
        image_log = root / "launcher" / "docker_image_inspect.log"
        image_record, image_output = _run_host_command(
            ["docker", "image", "inspect", plan["image_reference"]],
            "docker_image_inspect", image_log, executor=executor, priority=False, allow_failure=False)
        image_identity = _validate_image_identity(
            _parse_inspect(image_output, "image inspect"), plan["image"], "image")
        commands.append(image_record)
        name_check, name_record = _docker_inspect(
            plan["container_name"], root / "launcher" / "preexisting_container_inspect.log", executor=executor)
        commands.append(name_record)
        if name_check is not None:
            raise LauncherError("CONTAINER_NAME_ALREADY_EXISTS", plan["container_name"])
        run_record, run_output = _run_host_command(
            plan["run_argv"], "docker_run", root / "launcher" / "docker_run.log",
            executor=executor, priority=True, allow_failure=False)
        commands.append(run_record)
        container_id = run_output.decode("utf-8", errors="replace").strip().splitlines()[-1]
        if not HEX_ID_RE.fullmatch(container_id):
            raise LauncherError("CONTAINER_ID_INVALID", container_id)
        container_started = True
        inspected, inspect_record = _docker_inspect(
            plan["container_name"], root / "launcher" / "post_run_inspect.log", executor=executor)
        commands.append(inspect_record)
        _validate_container_identity(inspected, {"name": plan["container_name"],
                                                  "image_reference": plan["image_reference"],
                                                  "image_id": plan["image"]["digest"],
                                                  "mounts": plan["mounts"]},
                                     container_id=container_id)
        dependency_record, _ = _run_host_command(
            plan["dependency_argv"], "dependency_install", root / "launcher" / "dependency_install.log",
            executor=executor, priority=True, allow_failure=False)
        commands.append(dependency_record)
        dependency_capture_record, _ = _run_host_command(
            _dependency_capture_argv(plan, dependency_record),
            "dependency_environment_capture",
            root / "launcher" / "dependency_environment_capture.log",
            executor=executor, priority=True, allow_failure=False)
        commands.append(dependency_capture_record)
        dependency_environment = release_summary._validate_dependency_environment_manifest(
            Path(plan["evidence_child"]), DEPENDENCY_MANIFEST_NAME, plan["distro"],
            plan["image"], _sha256_bytes(DEPENDENCY_SCRIPT.encode("utf-8")),
            _sha256_file(
                ROOT / "scripts/capture_registration_plugin_dependency_environment.py"
            ),
        )
        if dependency_leg == "present":
            try:
                dependency_prefetch_evidence = dependency_prefetch.prefetch_pinned_archives(
                    plan["leg"]["dependencies"], plan["evidence_child"], distro,
                    plan["image"]["digest"], authority=HOST_PROMOTION_AUTHORITY,
                    fetcher=prefetcher,
                    allowed_host_directories=set(
                        evidence_directories.release_runner_paths(
                            [item["name"] for item in plan["leg"]["dependencies"]],
                            include_prefetch=False)),
                )
            except dependency_prefetch.PrefetchError as exc:
                raise LauncherError(getattr(exc, "kind", "PREFETCH_FAILED"), str(exc))
            prefetch_argv = [
                "python3", "scripts/registration_plugin_dependency_prefetch.py",
                "--distro", distro, "--authority", HOST_PROMOTION_AUTHORITY,
            ]
            prefetch_log = root / "launcher" / "dependency_prefetch.log"
            prefetch_payload = (_canonical(dependency_prefetch_evidence) + "\n").encode("utf-8")
            _write_exclusive(prefetch_log, prefetch_payload)
            commands.append({
                "label": "dependency_prefetch",
                "argv": prefetch_argv,
                "argv_sha256": _command_hash(prefetch_argv),
                "effective_argv": prefetch_argv,
                "priority": {"nice": 19, "ionice_class": 3, "applied": False},
                "returncode": 0,
                "script_sha256": _sha256_file(
                    ROOT / "scripts/registration_plugin_dependency_prefetch.py"),
                "network_phase": "provisioning",
                "log": {"path": str(prefetch_log),
                        "bytes": len(prefetch_payload),
                        "sha256": _sha256_bytes(prefetch_payload)},
            })
        networks = inspected.get("NetworkSettings", {}).get("Networks", {})
        for network in sorted(networks):
            disconnect_record, _ = _run_host_command(
                ["docker", "network", "disconnect", network, plan["container_name"]],
                "network_disconnect_" + network,
                root / "launcher" / ("network_disconnect_" + network + ".log"),
                executor=executor, priority=False, allow_failure=False)
            commands.append(disconnect_record)
        disconnected, disconnected_record = _docker_inspect(
            plan["container_name"], root / "launcher" / "post_disconnect_inspect.log", executor=executor)
        commands.append(disconnected_record)
        _validate_container_identity(disconnected, {
            "name": plan["container_name"],
            "image_reference": plan["image_reference"],
            "image_id": plan["image"]["digest"],
            "mounts": plan["mounts"],
        }, container_id=container_id)
        if disconnected.get("NetworkSettings", {}).get("Networks") != {}:
            raise LauncherError("NETWORK_DISCONNECT_FAILED", _canonical(disconnected))
        runner_record, _ = _run_host_command(
            plan["runner_argv"], "release_runner", root / "launcher" / "runner_exec.log",
            executor=executor, priority=True, allow_failure=True)
        commands.append(runner_record)
        diagnostics = _capture_diagnostics(plan, root, executor=executor)
        runner_path = Path(plan["output"])
        if runner_path.is_file() and not runner_path.is_symlink():
            try:
                runner_receipt = json.loads(runner_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise LauncherError("RUNNER_RECEIPT_INVALID", str(exc))
        if runner_record["returncode"] != 0:
            kind = runner_receipt.get("failure_kind") if runner_receipt else "RELEASE_RUNNER_FAILED"
            raise LauncherError(kind or "RELEASE_RUNNER_FAILED",
                                "release runner exited {}".format(runner_record["returncode"]))
        if not runner_receipt or runner_receipt.get("status") != "PASS":
            raise LauncherError("RUNNER_PASS_RECEIPT_MISSING", "runner did not produce PASS")
        profile, profile_file = audit._load_profile(Path(profile_path).resolve(strict=True))
        release = audit._validate_release_matrix_profile(profile)
        artifact_revalidation = release_summary._validate_leg_artifacts(
            runner_path, runner_receipt, profile, release, profile_file)
        runner_sidecar = Path(str(runner_path) + ".sha256")
        if (runner_path.is_symlink() or not runner_path.is_file() or
                runner_sidecar.is_symlink() or not runner_sidecar.is_file()):
            raise LauncherError("RUNNER_RECEIPT_BINDING_MISSING",
                                "runner receipt or sidecar is not a regular file")
        runner_receipt_binding = {
            "status": "PASS",
            "path_relative": runner_path.relative_to(root).as_posix(),
            "sha256": _sha256_file(runner_path),
            "sidecar_path_relative": runner_sidecar.relative_to(root).as_posix(),
            "sidecar_sha256": _sha256_file(runner_sidecar),
            "evidence_root": runner_receipt.get("evidence_root"),
        }
    except Exception as exc:
        first_failure = {"kind": getattr(exc, "kind", "LAUNCHER_FAILURE"), "message": str(exc)}
    finally:
        if container_started:
            if diagnostics is None:
                try:
                    diagnostics = _capture_diagnostics(plan, root, executor=executor)
                except Exception as exc:
                    diagnostics = {"status": "FAIL_CLOSED", "error": str(exc)}
            try:
                cleanup = _cleanup(plan, root, executor=executor, container_id=container_id)
            except Exception as exc:
                cleanup = {"status": "FAIL_CLOSED", "error": str(exc)}
    if cleanup.get("post_remove_absent") is not True:
        if first_failure is None:
            first_failure = {"kind": "CLEANUP_FAIL_CLOSED", "message": "container cleanup was not proven"}
    if directory_contract is not None:
        try:
            directory_contract = _verify_host_directory_layout(root, directory_contract)
        except LauncherError as exc:
            directory_contract = evidence_directories.failed(directory_contract, exc)
            if first_failure is None:
                first_failure = {"kind": exc.kind, "message": str(exc)}
    status = "PASS" if first_failure is None and cleanup.get("post_remove_absent") is True and artifact_revalidation.get("status", "PASS") != "NOT_REACHED" else "FAIL_CLOSED"
    logs = []
    for path in (root / "launcher").rglob("*.log"):
        os.chmod(path, 0o444)
        logs.append({"path": str(path.relative_to(root)), "sha256": _sha256_file(path), "bytes": path.stat().st_size})
    report = {
        "schema": CONTRACT, "schema_version": 1, "status": status,
        "failure": first_failure, "distro": distro, "dependency_leg": dependency_leg,
        "campaign_set": plan["campaign_set"],
        "profile": {"path": plan["profile_path"], "sha256": plan["profile_sha256"]},
        "source": {"manifest_sha256": plan["source_manifest"]["manifest_sha256"],
                   "runner_sha256": _sha256_file(ROOT / "scripts/run_registration_plugin_release_matrix.py"),
                   "launcher_sha256": _sha256_file(Path(__file__)),
                   "prefetch_sha256": _sha256_file(
                       ROOT / "scripts/registration_plugin_dependency_prefetch.py")},
        "image": image_identity or plan["image"],
        "container": {"name": plan["container_name"], "id": container_id,
                      "start_count": 1 if container_started else 0,
                      "image_reference": plan["image_reference"], "mounts": plan["mounts"]},
        "environment": {"required": REQUIRED_ENV_KEYS, "values": plan["env"]},
        "preflight": preflight, "commands": commands, "logs": logs,
        "diagnostics": diagnostics, "cleanup": cleanup,
        "directory_contract": directory_contract or {
            "schema": evidence_directories.SCHEMA,
            "schema_version": evidence_directories.SCHEMA_VERSION,
            "status": "NOT_REACHED",
        },
        "dependency_environment": dependency_environment,
        "dependency_prefetch": dependency_prefetch_evidence,
        "evidence_storage": plan["evidence_storage"],
        "runner_receipt": runner_receipt,
        "runner_receipt_binding": runner_receipt_binding,
        "artifact_revalidation": artifact_revalidation,
        "resource_policy": {"mode": plan["resource_policy"]["functional"]["mode"],
                             "timing_authority": audit.FUNCTIONAL_TIMING_AUTHORITY,
                             "performance_gate_eligible": False},
        "safety": {"provisioning_network_used": True,
                    "archive_fetch_network_used": dependency_leg == "present",
                    "build_test_network_connected": False,
                    "build_test_network_used": False,
                    "network_after_dependency_fetch": False,
                    "docker_build": False, "docker_pull": False, "bag_opened": False,
                    "ground_truth_content_opened": False, "scorer_invoked": False,
                    "map_saved": False, "formal_replay_started": False},
        "observed_at_unix": time.time(),
    }
    seal = _seal_closure(root, report)
    report["closure"] = seal
    return report


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=str(ROOT))
    parser.add_argument("--profile", default=str(ROOT / "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"))
    parser.add_argument("--distro", choices=("humble", "jazzy"), required=True)
    parser.add_argument("--dependency-leg", choices=("absent", "present"), required=True)
    parser.add_argument("--evidence-root", required=True)
    parser.add_argument(
        "--dependency-closure-root",
        help="fresh sealed PASS dependency closure required for runtime execution",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        plan = build_plan(args.repo_root, args.profile, args.distro,
                          args.dependency_leg, args.evidence_root)
        if args.dry_run:
            print(json.dumps({"status": "DRY_RUN", "plan": plan}, sort_keys=True))
            return 0
        report = run_leg(args.repo_root, args.profile, args.distro,
                         args.dependency_leg, args.evidence_root,
                         dependency_closure_root=args.dependency_closure_root)
        print(json.dumps(report, sort_keys=True))
        return 0 if report["status"] == "PASS" else 1
    except Exception as exc:
        failure = {"schema": CONTRACT, "schema_version": 1, "status": "FAIL_CLOSED",
                   "failure": {"kind": getattr(exc, "kind", "LAUNCHER_FAILURE"),
                               "message": str(exc)}}
        print(json.dumps(failure, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
