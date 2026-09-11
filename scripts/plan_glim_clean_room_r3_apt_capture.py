#!/usr/bin/env python3
"""Plan the non-promoting GLIM r3 apt/deb capture boundary.

This module is a pure planner.  It never invokes Docker, apt, rosdep, a
downloader, or a shell.  The returned workflow contains fixed argv vectors for
the host image/identity checks, container provisioning, contract-only capture,
disconnect, inspection, and cleanup phases.  User supplied provisioning or
capture shell fragments are not accepted.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
R3_ROOT = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
CAPTURE_PATH = R3_ROOT / "apt_closure_capture.py"
CAPTURE_SCHEMA_PATH = R3_ROOT / "apt_closure_capture.schema.json"
OUTER_PATH = R3_ROOT / "apt_closure_outer.py"
OUTER_SCHEMA_PATH = R3_ROOT / "apt_closure_outer.schema.json"
PLANNER_SCHEMA = "glim_clean_room_r3_apt_capture_plan_v2"
BASE_IMAGE = "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
ALLOWLIST_DEST = "/opt/package-allowlist.json"
CAPTURE_DEST = "/opt/apt_closure_capture.py"
CAPTURE_OUTPUT_DEST = "/workspace/capture"
MAX_JSON_BYTES = 8 * 1024 * 1024


class PlanError(ValueError):
    """Raised when a capture plan is not exact and safe."""


def _load_capture():
    spec = importlib.util.spec_from_file_location("r3_apt_capture_for_plan", CAPTURE_PATH)
    if spec is None or spec.loader is None:
        raise PlanError("r3 capture tool cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAPTURE = _load_capture()


def _sha(path: Path) -> str:
    try:
        info = path.lstat()
    except OSError as error:
        raise PlanError(f"identity input cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or \
            info.st_nlink != 1 or info.st_size <= 0:
        raise PlanError(f"identity input is not a regular non-empty single-link file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file() or path.stat().st_size <= 0 or \
            path.stat().st_size > MAX_JSON_BYTES:
        raise PlanError(f"JSON input is not a bounded regular file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PlanError(f"invalid JSON input: {path}: {error}") from error
    if not isinstance(value, dict):
        raise PlanError(f"JSON input is not an object: {path}")
    return value


def _fresh_absolute(path: Path) -> Path:
    path = Path(path)
    if not path.is_absolute() or path.exists() or path.is_symlink() or \
            not path.parent.is_dir() or path.parent.is_symlink():
        raise PlanError(f"capture root must be a fresh absolute path: {path}")
    return path


def _safe_container_name(output_root: Path) -> str:
    identity = hashlib.sha256(str(output_root).encode("utf-8")).hexdigest()[:20]
    return f"glim-r3-apt-{identity}"


def _package_specs(allowlist: Mapping[str, Any]) -> list[str]:
    return [f"{item['name']}={item['version']}" for item in allowlist["union"]]


def _fixed_commands(*, image_reference: str, container_name: str,
                    mounts: list[dict[str, Any]],
                    allowlist: Mapping[str, Any]) -> dict[str, Any]:
    """Return the only permitted host/container argv sequence."""
    inspect_format = "{{json .}}"
    image_inspect = ["docker", "image", "inspect", "--format", inspect_format,
                     image_reference]
    absent_inspect = ["docker", "container", "inspect", "--format", inspect_format,
                      container_name]
    output_absent = ["test", "!", "-e", mounts[0]["source"]]
    output_create = ["mkdir", "--mode", "0700", "--", mounts[0]["source"]]
    output_lstat = ["stat", "--format", "%F\\t%a\\t%u\\t%g\\t%h", "--",
                    mounts[0]["source"]]
    run = ["docker", "run", "--name", container_name, "--network", "bridge",
           "--platform", "linux/amd64", "--entrypoint", "sleep"]
    for mount in mounts:
        mode = "ro" if mount["read_only"] else "rw"
        run.extend(["--mount", f"type=bind,src={mount['source']},"
                    f"dst={mount['destination']},{mode}"])
    run.extend([image_reference, "300"])
    update = ["docker", "exec", container_name, "apt-get", "update",
              "-o", "Acquire::Retries=0", "-o", "Acquire::Languages=none"]
    install = ["docker", "exec", container_name, "apt-get", "install", "--yes",
               "--no-install-recommends", *_package_specs(allowlist)]
    inner_provision = ["apt-get", "install", "--yes", "--no-install-recommends",
                       *_package_specs(allowlist)]
    inner_capture = ["python3", CAPTURE_DEST, "contract-only", "--allowlist",
                     ALLOWLIST_DEST, "--output", CAPTURE_OUTPUT_DEST]
    capture = ["docker", "exec", container_name, *inner_capture]
    disconnect = ["docker", "network", "disconnect", "bridge", container_name]
    post_inspect = ["docker", "container", "inspect", "--format", inspect_format,
                    container_name]
    stop = ["docker", "stop", "--time", "10", container_name]
    remove = ["docker", "container", "rm", container_name]
    return {
        "image_inspect": image_inspect,
        "output_root_absent_check": output_absent,
        "output_root_create": output_create,
        "output_root_lstat": output_lstat,
        "preexisting_name_inspect": absent_inspect,
        "container_run": run,
        "provisioning_update": update,
        "provisioning_install": install,
        "capture": capture,
        "disconnect": disconnect,
        "post_disconnect_inspect": post_inspect,
        "stop": stop,
        "remove": remove,
        "post_remove_inspect": absent_inspect,
        "inner_provisioning": inner_provision,
        "inner_capture": inner_capture,
    }


def _workflow(commands: Mapping[str, list[str]]) -> list[dict[str, Any]]:
    phases = [
        ("image_inspect", 0),
        ("output_root_absent_check", 0),
        ("output_root_create", 0),
        ("output_root_lstat", 0),
        ("preexisting_name_inspect", 1),
        ("container_run", 0),
        ("provisioning_update", 0),
        ("provisioning_install", 0),
        # The installed tool is explicitly CONTRACT_ONLY until a real
        # collector exists.  78 is the stable NOT_IMPLEMENTED exit.
        ("capture", 78),
        ("disconnect", 0),
        ("post_disconnect_inspect", 0),
        ("stop", 0),
        ("remove", 0),
        ("post_remove_inspect", 1),
    ]
    return [{"phase": phase, "argv": list(commands[phase]),
             "expected_exit_status": exit_status, "ordinal": ordinal}
            for ordinal, (phase, exit_status) in enumerate(phases)]


def _plan_identity(plan: Mapping[str, Any]) -> str:
    projection = dict(plan)
    projection.pop("plan_identity_sha256", None)
    binding = dict(projection.get("outer_binding", {}))
    binding.pop("plan_identity_sha256", None)
    binding.pop("receipt_sha256", None)
    projection["outer_binding"] = binding
    return CAPTURE.canonical_hash(projection)


def plan_capture(*, output_root: Path, allowlist: Mapping[str, Any],
                 allowlist_path: Path, outer_root: Path | None = None,
                 provisioning_command: list[str] | None = None,
                 capture_command: list[str] | None = None,
                 image_reference: str = BASE_IMAGE) -> dict[str, Any]:
    """Return a deterministic review-only plan; never execute it.

    The legacy command parameters are accepted only to fail closed with a
    useful error.  They are intentionally not incorporated into the plan.
    """
    output_root = _fresh_absolute(output_root)
    outer_root = _fresh_absolute(
        Path(str(output_root) + ".outer") if outer_root is None else outer_root)
    allowlist_path = Path(allowlist_path)
    allowlist_file_sha256 = _sha(allowlist_path)
    image_reference = CAPTURE._image_reference(image_reference)
    if image_reference != BASE_IMAGE:
        raise PlanError("capture image differs from pinned r3 base")
    if provisioning_command is not None or capture_command is not None:
        raise PlanError("arbitrary provisioning/capture commands are forbidden; use fixed workflow")
    allowlist = CAPTURE._validate_allowlist(allowlist)
    container_name = _safe_container_name(output_root)
    mounts = [
        {"source": str(output_root), "destination": CAPTURE_OUTPUT_DEST, "read_only": False},
        {"source": str(CAPTURE_PATH.resolve()), "destination": CAPTURE_DEST, "read_only": True},
        {"source": str(allowlist_path.resolve()), "destination": ALLOWLIST_DEST,
         "read_only": True},
    ]
    commands = _fixed_commands(image_reference=image_reference,
                               container_name=container_name,
                               mounts=mounts, allowlist=allowlist)
    workflow = _workflow(commands)
    plan = {
        "schema": PLANNER_SCHEMA,
        "schema_version": 2,
        "status": "REVIEW_REQUIRED",
        "execute": False,
        "capture_mode": "CONTRACT_ONLY",
        "benchmark_eligible": False,
        "network_used": True,
        "build_test_network": "NOT_STARTED",
        "image_reference": image_reference,
        "platform": "linux/amd64",
        "output_root": str(output_root),
        "outer_root": str(outer_root),
        "container_name": container_name,
        "host_owner": {"uid": os.getuid(), "gid": os.getgid()},
        "mounts": mounts,
        "container_argv": commands["container_run"],
        "workflow": workflow,
        "workflow_sha256": CAPTURE.canonical_hash({"workflow": workflow}),
        "provisioning_command": commands["inner_provisioning"],
        "provisioning_command_sha256": CAPTURE.canonical_hash(
            {"argv": commands["inner_provisioning"]}),
        "provisioning_sequence": [commands["provisioning_update"],
                                   commands["provisioning_install"]],
        "capture_command": commands["inner_capture"],
        "capture_command_sha256": CAPTURE.canonical_hash(
            {"argv": commands["inner_capture"]}),
        "package_allowlist": allowlist,
        "package_allowlist_sha256": CAPTURE.canonical_hash(allowlist),
        "allowlist_path": str(allowlist_path.resolve()),
        "allowlist_file_sha256": allowlist_file_sha256,
        "capture_tool_sha256": _sha(CAPTURE_PATH),
        "capture_schema_sha256": _sha(CAPTURE_SCHEMA_PATH),
        "outer_tool_sha256": _sha(OUTER_PATH),
        "outer_schema_sha256": _sha(OUTER_SCHEMA_PATH),
        "planner_sha256": _sha(Path(__file__).resolve()),
        "outer_binding": {
            "image_reference": image_reference,
            "container_name": container_name,
            "workflow_sha256": CAPTURE.canonical_hash({"workflow": workflow}),
            "output_root": str(output_root),
            "outer_root": str(outer_root),
            "allowlist_file_sha256": allowlist_file_sha256,
            "plan_identity_sha256": "BOUND_AFTER_PLAN_HASH",
            "receipt_sha256": "BOUND_AFTER_OUTER_REOPEN",
        },
    }
    plan["plan_identity_sha256"] = _plan_identity(plan)
    plan["outer_binding"]["plan_identity_sha256"] = plan["plan_identity_sha256"]
    return plan


def plan_from_file(*, output_root: Path, allowlist_path: Path,
                   outer_root: Path | None = None) -> dict[str, Any]:
    allowlist = _read_json(allowlist_path)
    return plan_capture(output_root=output_root, allowlist=allowlist,
                        allowlist_path=allowlist_path, outer_root=outer_root)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--outer-root", type=Path)
    parser.add_argument("--allowlist", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = plan_from_file(output_root=args.output_root,
                                outer_root=args.outer_root,
                                allowlist_path=args.allowlist)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, PlanError, TypeError, ValueError) as error:
        print(json.dumps({"status": "FAIL_CLOSED", "reason": str(error)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
