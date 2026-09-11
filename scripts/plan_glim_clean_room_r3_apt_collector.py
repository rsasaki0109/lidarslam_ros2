#!/usr/bin/env python3
"""Pure planner for the opt-in GLIM r3 apt collector candidate.

No command in this module is executed.  The plan describes the only allowed
provisioning sequence: update, materialize the host-reviewed URI ledger,
snapshot the base dpkg status, download-only, copy the downloaded debs to a
fresh staging directory, install, collect from read-only mounts, disconnect,
inspect and clean up.  The collector is a candidate and remains non-promoting
until a real container run and review.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
R3_ROOT = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
COLLECTOR_PATH = R3_ROOT / "apt_closure_collector.py"
COLLECTOR_SCHEMA_PATH = R3_ROOT / "apt_closure_collector.schema.json"
CAPTURE_PATH = R3_ROOT / "apt_closure_capture.py"
CAPTURE_SCHEMA_PATH = R3_ROOT / "apt_closure_capture.schema.json"
PLANNER_SCHEMA = "glim_clean_room_r3_apt_collector_plan_v2"
BASE_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
ALLOWLIST_DEST = "/opt/package-allowlist.json"
COLLECTOR_DEST = "/opt/apt_closure_collector.py"
CAPTURE_DEST = "/opt/apt_closure_capture.py"
OUTPUT_DEST = "/workspace/capture"
LEDGER_DEST = "/workspace/capture/apt-acquisition-ledger.json"
LEDGER_MOUNT_DEST = "/opt/apt-acquisition-ledger.json"
SNAPSHOT_DEST = "/workspace/capture/base-dpkg-status.snapshot"
STAGED_OUTPUT_DEST = "/workspace/capture/staged-debs"


class PlanError(ValueError):
    """Raised when the collector candidate cannot be planned exactly."""


def _sha(path: Path) -> str:
    try:
        info = path.lstat()
    except OSError as error:
        raise PlanError(f"identity file cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or \
            info.st_nlink != 1 or info.st_size <= 0:
        raise PlanError(f"identity file is not a non-empty regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=True).encode("utf-8")).hexdigest()


def _fresh(path: Path) -> Path:
    path = Path(path)
    if not path.is_absolute() or path.exists() or path.is_symlink() or \
            not path.parent.is_dir() or path.parent.is_symlink():
        raise PlanError(f"collector root must be fresh and absolute: {path}")
    return path


def _package_specs(allowlist: Mapping[str, Any]) -> list[str]:
    return [f"{item['name']}={item['version']}" for item in allowlist["union"]]


def fixed_commands(*, image_reference: str, container_name: str,
                   output_root: Path, allowlist: Mapping[str, Any],
                   allowlist_path: Path, ledger_input_path: Path,
                   collector_source_path: Path = COLLECTOR_PATH) -> dict[str, list[str]]:
    """Build the exact argv vectors; callers must not supply shell fragments."""
    if image_reference != BASE_IMAGE:
        raise PlanError("collector image is not the pinned base")
    mounts = [
        {"source": str(output_root), "destination": OUTPUT_DEST, "read_only": False},
        {"source": str(collector_source_path.resolve()), "destination": COLLECTOR_DEST,
         "read_only": True},
        {"source": str(CAPTURE_PATH.resolve()), "destination": CAPTURE_DEST,
         "read_only": True},
        {"source": str(CAPTURE_SCHEMA_PATH.resolve()),
         "destination": "/opt/apt_closure_capture.schema.json", "read_only": True},
        {"source": str(allowlist_path.resolve()), "destination": ALLOWLIST_DEST,
         "read_only": True},
        {"source": str(ledger_input_path.resolve()), "destination": LEDGER_MOUNT_DEST,
         "read_only": True},
    ]
    inspect = ["docker", "image", "inspect", "--format", "{{json .}}", image_reference]
    absent = ["docker", "container", "inspect", "--format", "{{json .}}", container_name]
    run = ["docker", "run", "--name", container_name, "--network", "bridge",
           "--platform", "linux/amd64", "--entrypoint", "sleep"]
    for mount in mounts:
        mode = "ro" if mount["read_only"] else "rw"
        run.extend(["--mount", f"type=bind,src={mount['source']},"
                    f"dst={mount['destination']},{mode}"])
    run.extend([image_reference, "300"])
    packages = _package_specs(allowlist)
    update = ["docker", "exec", container_name, "apt-get", "update",
              "-o", "Acquire::Retries=0", "-o", "Acquire::Languages=none"]
    ledger = ["docker", "exec", container_name, "python3", COLLECTOR_DEST,
              "uri-ledger", "--input", LEDGER_MOUNT_DEST,
              "--output", LEDGER_DEST]
    snapshot = ["docker", "exec", container_name, "python3", COLLECTOR_DEST,
                "snapshot-base", "--root", "/", "--output", SNAPSHOT_DEST]
    download = ["docker", "exec", container_name, "apt-get", "--download-only",
                "install", "--yes", "--no-install-recommends", *packages]
    stage = ["docker", "exec", container_name, "python3", COLLECTOR_DEST,
             "stage-debs", "--root", "/", "--ledger", LEDGER_MOUNT_DEST,
             "--output", STAGED_OUTPUT_DEST]
    install = ["docker", "exec", container_name, "apt-get", "install", "--yes",
               "--no-install-recommends", *packages]
    collect = ["docker", "exec", container_name, "python3", COLLECTOR_DEST, "collect",
               "--root", "/", "--allowlist", ALLOWLIST_DEST, "--ledger", LEDGER_DEST,
               "--base-status", SNAPSHOT_DEST, "--staged-debs", STAGED_OUTPUT_DEST,
               "--output", OUTPUT_DEST + "/collector-output"]
    disconnect = ["docker", "network", "disconnect", "bridge", container_name]
    inspect_after = ["docker", "container", "inspect", "--format", "{{json .}}", container_name]
    stop = ["docker", "stop", "--time", "10", container_name]
    remove = ["docker", "container", "rm", container_name]
    return {"image_inspect": inspect, "output_root_absent_check": ["test", "!", "-e",
            str(output_root)], "output_root_create": ["mkdir", "--mode", "0700", "--",
            str(output_root)], "output_root_lstat": ["stat", "--format",
            "%F\\t%a\\t%u\\t%g\\t%h", "--", str(output_root)],
            "preexisting_name_inspect": absent, "container_run": run,
        "provisioning_update": update, "acquisition_ledger": ledger,
            "base_status_snapshot": snapshot, "download_only": download,
            "stage_downloads": stage, "provisioning_install": install,
            "collector": collect, "disconnect": disconnect,
            "post_disconnect_inspect": inspect_after, "stop": stop, "remove": remove,
            "post_remove_inspect": absent}


def workflow(commands: Mapping[str, list[str]]) -> list[dict[str, Any]]:
    phases = [
        ("image_inspect", 0), ("output_root_absent_check", 0),
        ("output_root_create", 0), ("output_root_lstat", 0),
        ("preexisting_name_inspect", 1), ("container_run", 0),
        ("provisioning_update", 0), ("acquisition_ledger", 0),
        ("base_status_snapshot", 0), ("download_only", 0),
        ("stage_downloads", 0), ("provisioning_install", 0),
        ("collector", 0), ("disconnect", 0),
        ("post_disconnect_inspect", 0), ("stop", 0), ("remove", 0),
        ("post_remove_inspect", 1),
    ]
    return [{"ordinal": ordinal, "phase": phase, "argv": list(commands[phase]),
             "expected_exit_status": status}
            for ordinal, (phase, status) in enumerate(phases)]


def plan_collector(*, output_root: Path, allowlist: Mapping[str, Any],
                   allowlist_path: Path, ledger_input_path: Path,
                   outer_root: Path | None = None,
                   image_reference: str = BASE_IMAGE) -> dict[str, Any]:
    output_root = _fresh(output_root)
    allowlist_path = Path(allowlist_path)
    ledger_input_path = Path(ledger_input_path)
    _sha(allowlist_path)
    _sha(ledger_input_path)
    outer_root = _fresh(Path(str(output_root) + ".outer") if outer_root is None else outer_root)
    if image_reference != BASE_IMAGE:
        raise PlanError("collector image differs from pinned base")
    container_name = "glim-r3-collector-" + hashlib.sha256(
        str(output_root).encode("utf-8")).hexdigest()[:20]
    commands = fixed_commands(image_reference=image_reference,
                              container_name=container_name, output_root=output_root,
                              allowlist=allowlist, allowlist_path=allowlist_path,
                              ledger_input_path=ledger_input_path)
    planned_workflow = workflow(commands)
    plan = {
        "schema": PLANNER_SCHEMA, "schema_version": 1,
        "status": "REVIEW_REQUIRED",
        "execute": False,
        "capture_mode": "IMPLEMENTATION_CANDIDATE_NOT_RUNTIME_VALIDATED",
        "benchmark_eligible": False,
        "network_used": True,
        "build_test_network": "NOT_STARTED",
        "image_reference": image_reference,
        "platform": "linux/amd64",
        "output_root": str(output_root), "outer_root": str(outer_root),
        "container_name": container_name,
        "host_owner": {"uid": os.getuid(), "gid": os.getgid()},
        "workflow": planned_workflow,
        "workflow_sha256": canonical_hash({"workflow": planned_workflow}),
        "package_allowlist": allowlist,
        "allowlist_path": str(allowlist_path.resolve()),
        "ledger_input_path": str(Path(ledger_input_path).resolve()),
        "ledger_input_sha256": _sha(Path(ledger_input_path)),
        "mounts": [
            {"source": str(output_root), "destination": OUTPUT_DEST, "read_only": False,
             "sha256": "0" * 64},
            {"source": str(COLLECTOR_PATH.resolve()), "destination": COLLECTOR_DEST,
             "read_only": True, "sha256": _sha(COLLECTOR_PATH)},
            {"source": str(CAPTURE_PATH.resolve()), "destination": CAPTURE_DEST,
             "read_only": True, "sha256": _sha(CAPTURE_PATH)},
            {"source": str(CAPTURE_SCHEMA_PATH.resolve()),
             "destination": "/opt/apt_closure_capture.schema.json", "read_only": True,
             "sha256": _sha(CAPTURE_SCHEMA_PATH)},
            {"source": str(allowlist_path.resolve()), "destination": ALLOWLIST_DEST,
             "read_only": True, "sha256": _sha(allowlist_path)},
            {"source": str(ledger_input_path.resolve()), "destination": LEDGER_MOUNT_DEST,
             "read_only": True, "sha256": _sha(ledger_input_path)},
        ],
        "mounts_sha256": canonical_hash({"mounts": [
            {"source": str(output_root), "destination": OUTPUT_DEST, "read_only": False,
             "sha256": "0" * 64},
            {"source": str(COLLECTOR_PATH.resolve()), "destination": COLLECTOR_DEST,
             "read_only": True, "sha256": _sha(COLLECTOR_PATH)},
            {"source": str(CAPTURE_PATH.resolve()), "destination": CAPTURE_DEST,
             "read_only": True, "sha256": _sha(CAPTURE_PATH)},
            {"source": str(CAPTURE_SCHEMA_PATH.resolve()),
             "destination": "/opt/apt_closure_capture.schema.json", "read_only": True,
             "sha256": _sha(CAPTURE_SCHEMA_PATH)},
            {"source": str(allowlist_path.resolve()), "destination": ALLOWLIST_DEST,
             "read_only": True, "sha256": _sha(allowlist_path)},
            {"source": str(ledger_input_path.resolve()), "destination": LEDGER_MOUNT_DEST,
             "read_only": True, "sha256": _sha(ledger_input_path)},
        ]}),
        "collector_path": str(COLLECTOR_PATH.relative_to(ROOT)),
        "collector_sha256": _sha(COLLECTOR_PATH),
        "collector_schema_path": str(COLLECTOR_SCHEMA_PATH.relative_to(ROOT)),
        "collector_schema_sha256": _sha(COLLECTOR_SCHEMA_PATH),
        "capture_tool_sha256": _sha(CAPTURE_PATH),
        "capture_schema_sha256": _sha(CAPTURE_SCHEMA_PATH),
        "production_manifest_modified": False,
    }
    plan["plan_identity_sha256"] = canonical_hash(plan)
    return plan


if __name__ == "__main__":
    raise SystemExit("plan_glim_clean_room_r3_apt_collector is a library module; import it instead of running it directly.")
