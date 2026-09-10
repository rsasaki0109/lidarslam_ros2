#!/usr/bin/env python3
"""Plan, but never execute, the r3 package-closure acquisition workflow.

The plan is a fixed argv/data contract for a future host executor.  It has no
shell fragment, downloader, Docker call, apt call, or network operation.  The
build and runtime resolver/download roots are distinct and fresh; their exact
union is later composed by ``apt_allowlist_composer.py``.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import stat
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
COMPOSER_PATH = R3 / "apt_allowlist_composer.py"
COLLECTOR_PATH = R3 / "apt_closure_collector.py"
REQUIREMENTS_SCHEMA_PATH = R3 / "apt_package_requirements.schema.json"
RESOLVER_SCHEMA_PATH = R3 / "apt_resolver_output.schema.json"
PROPOSAL_SCHEMA_PATH = R3 / "apt_allowlist_generation_proposal.schema.json"
SCHEMA = "glim_clean_room_r3_apt_allowlist_generation_plan_v1"
HOST_EXECUTOR_STATUS = "ABSENT_FAIL_CLOSED"
BASE_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)


class PlanError(ValueError):
    """Raised when the fixed candidate plan cannot be emitted."""


def _load_composer():
    spec = importlib.util.spec_from_file_location("r3_allowlist_composer_for_plan", COMPOSER_PATH)
    if spec is None or spec.loader is None:
        raise PlanError("allowlist composer cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


COMPOSER = _load_composer()


def _regular_sha(path: Path, label: str) -> tuple[str, bytes]:
    try:
        info = path.lstat()
    except OSError as error:
        raise PlanError(f"{label} cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0:
        raise PlanError(f"{label} must be a non-empty single-link regular file: {path}")
    data = path.read_bytes()
    if len(data) != info.st_size:
        raise PlanError(f"{label} changed while being read: {path}")
    return hashlib.sha256(data).hexdigest(), data


def _fresh(path: Path, label: str) -> Path:
    path = Path(path)
    if not path.is_absolute() or path.exists() or path.is_symlink() or \
            not path.parent.is_dir() or path.parent.is_symlink():
        raise PlanError(f"{label} must be a fresh absolute path: {path}")
    return path


def _requirements(path: Path) -> tuple[dict[str, Any], str]:
    digest, data = _regular_sha(path, "package requirements")
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise PlanError(f"package requirements are invalid JSON: {error}") from error
    try:
        value = COMPOSER.validate_requirements(value)
    except COMPOSER.ComposerError as error:
        raise PlanError(str(error)) from error
    return value, digest


def _command_hash(argv: list[str]) -> str:
    return hashlib.sha256(json.dumps(argv, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=True).encode("utf-8")).hexdigest()


def _requirements_names(requirements: Mapping[str, Any], role: str) -> list[str]:
    return [item["name"] for item in requirements[role]]


def _host_path(output_root: Path, *parts: str) -> str:
    """Return an absolute host path below the fresh output root."""
    candidate = output_root.joinpath(*parts)
    if not candidate.is_absolute() or output_root not in candidate.parents:
        raise PlanError("host output path escaped the fresh output root")
    return str(candidate)


def _mount(source: Path, destination: str, *, read_only: bool) -> dict[str, Any]:
    digest, _data = _regular_sha(source, f"mount source {destination}")
    return {"source": str(source.resolve()), "destination": destination,
            "read_only": read_only, "sha256": digest, "source_state": "EXISTS"}


def _workflow_entry(ordinal: int, phase: str, argv: list[str], expected: int,
                    **metadata: Any) -> dict[str, Any]:
    entry: dict[str, Any] = {"ordinal": ordinal, "phase": phase, "argv": list(argv),
                             "argv_sha256": _command_hash(argv),
                             "expected_exit_status": expected}
    entry.update(metadata)
    return entry


def fixed_workflow(*, image_reference: str, container_name: str,
                   output_root: Path, requirements: Mapping[str, Any],
                   requirements_path: Path | None = None) -> list[dict[str, Any]]:
    """Describe the complete host/container workflow without executing it.

    Container paths only occur in Docker argv.  Every host-side input/output
    is an absolute path below ``output_root`` (or a verified source mount), so
    a future executor cannot accidentally read ``/workspace`` on the host.
    """
    if image_reference != BASE_IMAGE:
        raise PlanError("only the pinned Jazzy base image is accepted")
    output_root = Path(output_root)
    if requirements_path is None:
        raise PlanError("requirements_path is required for the fixed mount contract")
    requirements_path = Path(requirements_path)
    requirements_host = str(requirements_path.resolve())
    output = str(output_root)
    container_output = "/workspace/capture"
    build_cache = container_output + "/apt-cache/build"
    runtime_cache = container_output + "/apt-cache/runtime"
    build_names = _requirements_names(requirements, "build")
    runtime_names = _requirements_names(requirements, "runtime")
    host = lambda *parts: _host_path(output_root, *parts)
    raw_base = host("raw", "base-dpkg-status.tsv")
    raw_sources = host("raw", "apt-sources.tar")
    raw_rosdep = host("raw", "rosdep-sources.tar")
    build_log = host("raw", "build-print-uris.log")
    runtime_log = host("raw", "runtime-print-uris.log")
    build_argv = host("raw", "build-print-uris.argv.json")
    runtime_argv = host("raw", "runtime-print-uris.argv.json")
    build_debs = host("staged-debs", "build")
    runtime_debs = host("staged-debs", "runtime")
    build_resolver = host("resolver", "build.json")
    runtime_resolver = host("resolver", "runtime.json")
    base_snapshot = host("sealed", "base-dpkg-status.json")
    source_root = host("sealed", "apt-source-root")
    source_manifest = host("sealed", "apt-source-manifest.json")
    proposal_root = host("proposal")
    image_inspect = ["docker", "image", "inspect", "--format", "{{json .}}", image_reference]
    absent = ["docker", "container", "inspect", "--format", "{{json .}}", container_name]
    output_absent = ["test", "!", "-e", output]
    output_create = ["mkdir", "--mode", "0700", "--", output]
    output_lstat = ["stat", "--format", "%F\\t%a\\t%u\\t%g\\t%h", "--", output]
    mounts = [
        (container_output, "rw"),
        ("/opt/package-requirements.json", "ro"),
        ("/opt/apt_allowlist_composer.py", "ro"),
        ("/opt/apt_package_requirements.schema.json", "ro"),
        ("/opt/apt_resolver_output.schema.json", "ro"),
        ("/opt/apt_allowlist_generation_proposal.schema.json", "ro"),
        ("/opt/apt_closure_collector.py", "ro"),
    ]
    source_for_dest = {
        "/opt/package-requirements.json": requirements_path,
        "/opt/apt_allowlist_composer.py": COMPOSER_PATH,
        "/opt/apt_package_requirements.schema.json": REQUIREMENTS_SCHEMA_PATH,
        "/opt/apt_resolver_output.schema.json": RESOLVER_SCHEMA_PATH,
        "/opt/apt_allowlist_generation_proposal.schema.json": PROPOSAL_SCHEMA_PATH,
        "/opt/apt_closure_collector.py": COLLECTOR_PATH,
    }
    run = ["docker", "run", "--name", container_name, "--network", "bridge",
           "--platform", "linux/amd64", "--entrypoint", "sleep"]
    for destination, mode in mounts:
        if destination == container_output:
            source = output
        else:
            source = str(source_for_dest[destination].resolve())
        run.extend(["--mount", f"type=bind,src={source},dst={destination},{mode}"])
    run.extend([image_reference, "300"])
    apt_update = ["docker", "exec", container_name, "apt-get", "update", "-o",
                  "Acquire::Retries=0", "-o", "Acquire::Languages=none"]
    output_layout = ["docker", "exec", container_name, "install", "-d", "-m", "0700",
                     container_output + "/raw", container_output + "/apt-cache/build",
                     container_output + "/apt-cache/runtime"]
    base = ["docker", "exec", container_name, "dpkg-query", "-W", "-f",
            "${Package}\t${Version}\t${Architecture}\t${Status}\n"]
    source = ["docker", "exec", container_name, "tar", "-cf", container_output + "/raw/apt-sources.tar",
              "--", "/etc/apt/sources.list", "/etc/apt/sources.list.d", "/var/lib/apt/lists"]
    rosdep = ["docker", "exec", container_name, "tar", "-cf", container_output + "/raw/rosdep-sources.tar",
              "--", "/etc/ros/rosdep", "/var/lib/rosdep"]
    def resolver(role: str, cache: str, names: list[str]) -> list[str]:
        return ["docker", "exec", container_name, "apt-get", "--print-uris",
                "--download-only", "--yes", "--no-install-recommends", "-o",
                f"Dir::Cache::archives={cache}", "install", *names]
    def download(cache: str, names: list[str]) -> list[str]:
        return ["docker", "exec", container_name, "apt-get", "--download-only", "--yes",
                "--no-install-recommends", "-o", f"Dir::Cache::archives={cache}",
                "install", *names]
    disconnect = ["docker", "network", "disconnect", "bridge", container_name]
    post = ["docker", "container", "inspect", "--format", "{{json .}}", container_name]
    stop = ["docker", "stop", "--time", "10", container_name]
    remove = ["docker", "container", "rm", container_name]
    commands: list[tuple[str, list[str], int, dict[str, Any]]] = [
        ("image_inspect", image_inspect, 0, {}),
        ("output_root_absent_check", output_absent, 0, {}),
        ("output_root_create", output_create, 0, {}),
        ("output_root_lstat", output_lstat, 0, {"host_path": output}),
        ("preexisting_name_inspect", absent, 1, {}),
        ("container_run", run, 0, {"mounts": [list(item) for item in mounts]}),
        ("container_output_layout", output_layout, 0, {}),
        ("apt_update", apt_update, 0, {}),
        ("base_status_snapshot", base, 0, {"host_stdout_path": raw_base}),
        ("apt_source_snapshot", source, 0,
         {"container_output_path": container_output + "/raw/apt-sources.tar"}),
        ("rosdep_snapshot", rosdep, 0,
         {"container_output_path": container_output + "/raw/rosdep-sources.tar"}),
        ("build_resolver", resolver("build", build_cache, build_names), 0,
         {"host_stdout_path": build_log, "host_argv_path": build_argv, "role": "build"}),
        ("build_download", download(build_cache, build_names), 0, {"role": "build"}),
        ("build_stage", ["host", "stage-debs", "--role", "build", "--source",
                          host("apt-cache", "build"), "--destination", build_debs], 0,
         {"host_paths": [host("apt-cache", "build"), build_debs], "role": "build"}),
        ("runtime_resolver", resolver("runtime", runtime_cache, runtime_names), 0,
         {"host_stdout_path": runtime_log, "host_argv_path": runtime_argv, "role": "runtime"}),
        ("runtime_download", download(runtime_cache, runtime_names), 0, {"role": "runtime"}),
        ("runtime_stage", ["host", "stage-debs", "--role", "runtime", "--source",
                            host("apt-cache", "runtime"), "--destination", runtime_debs], 0,
         {"host_paths": [host("apt-cache", "runtime"), runtime_debs], "role": "runtime"}),
        ("disconnect", disconnect, 0, {}),
        ("post_disconnect_inspect", post, 0, {}),
        ("stop", stop, 0, {}),
        ("remove", remove, 0, {}),
        ("post_remove_inspect", absent, 1, {}),
        ("seal_base_snapshot", ["host", "seal-base-status", "--raw", raw_base,
                                 "--output", base_snapshot], 0,
         {"host_paths": [raw_base, base_snapshot]}),
        ("materialize_source_snapshot", ["host", "materialize-apt-sources", "--raw",
                                          raw_sources, "--rosdep-raw", raw_rosdep,
                                          "--root", source_root, "--manifest", source_manifest], 0,
         {"host_paths": [raw_sources, raw_rosdep, source_root, source_manifest]}),
        ("compose_build_resolver", ["host", "compose-resolver", "--role", "build", "--raw-log",
                                     build_log, "--deb-root", build_debs, "--requirements",
                                     requirements_host, "--command-json", build_argv,
                                     "--base", base_snapshot,
                                     "--source-root", source_root, "--source-manifest", source_manifest,
                                     "--output", build_resolver], 0,
         {"host_paths": [build_log, build_argv, build_debs, requirements_host, base_snapshot,
                          source_root, source_manifest, build_resolver], "role": "build"}),
        ("compose_runtime_resolver", ["host", "compose-resolver", "--role", "runtime", "--raw-log",
                                       runtime_log, "--deb-root", runtime_debs, "--requirements",
                                       requirements_host, "--command-json", runtime_argv,
                                       "--base", base_snapshot,
                                       "--source-root", source_root, "--source-manifest", source_manifest,
                                       "--output", runtime_resolver], 0,
         {"host_paths": [runtime_log, runtime_argv, runtime_debs, requirements_host,
                          base_snapshot, source_root, source_manifest, runtime_resolver],
          "role": "runtime"}),
        ("seal_candidate", ["host", "compose-apt-allowlist", "--requirements",
                             requirements_host, "--base", base_snapshot,
                             "--build-resolver", build_resolver, "--runtime-resolver", runtime_resolver,
                             "--build-raw-log", build_log, "--runtime-raw-log", runtime_log,
                             "--source-root", source_root, "--source-manifest", source_manifest,
                             "--build-debs", build_debs, "--runtime-debs", runtime_debs,
                             "--output", proposal_root], 0,
         {"host_paths": [base_snapshot, build_resolver, runtime_resolver, build_log, runtime_log,
                          source_root, source_manifest, build_debs, runtime_debs, proposal_root]}),
        ("cleanup", ["host", "cleanup-acquisition-caches", "--paths",
                      host("apt-cache", "build"), host("apt-cache", "runtime")], 0,
         {"host_paths": [host("apt-cache", "build"), host("apt-cache", "runtime")]})]
    phases = [_workflow_entry(index, phase, argv, expected, **metadata)
              for index, (phase, argv, expected, metadata) in enumerate(commands)]
    _validate_workflow(phases, output_root=output_root, image_reference=image_reference,
                       container_name=container_name, mounts=mounts,
                       source_for_dest=source_for_dest)
    return phases


def _validate_workflow(workflow: list[dict[str, Any]], *, output_root: Path,
                       image_reference: str, container_name: str,
                       mounts: list[tuple[str, str]],
                       source_for_dest: Mapping[str, Path]) -> None:
    expected = ["image_inspect", "output_root_absent_check", "output_root_create",
                "output_root_lstat", "preexisting_name_inspect", "container_run",
                "container_output_layout", "apt_update", "base_status_snapshot",
                "apt_source_snapshot", "rosdep_snapshot",
                "build_resolver", "build_download", "build_stage", "runtime_resolver",
                "runtime_download", "runtime_stage", "disconnect", "post_disconnect_inspect",
                "stop", "remove", "post_remove_inspect", "seal_base_snapshot",
                "materialize_source_snapshot", "compose_build_resolver",
                "compose_runtime_resolver", "seal_candidate", "cleanup"]
    if [item.get("phase") for item in workflow] != expected:
        raise PlanError("fixed workflow phase order is incomplete or reordered")
    if [item.get("ordinal") for item in workflow] != list(range(len(expected))):
        raise PlanError("fixed workflow ordinals are not contiguous")
    for item in workflow:
        argv = item.get("argv")
        if not isinstance(argv, list) or item.get("argv_sha256") != _command_hash(argv):
            raise PlanError(f"{item.get('phase')} argv identity drift")
        if any(token in {"bash", "sh", "-c", "-lc", "curl", "wget"} for token in argv):
            raise PlanError(f"{item.get('phase')} contains an arbitrary shell/network token")
    run = workflow[5]["argv"]
    expected_prefix = ["docker", "run", "--name", container_name, "--network", "bridge",
                       "--platform", "linux/amd64", "--entrypoint", "sleep"]
    if run[:len(expected_prefix)] != expected_prefix or run[-2:] != [image_reference, "300"]:
        raise PlanError("container run identity is not fixed")
    observed_mounts = []
    index = len(expected_prefix)
    while index < len(run) - 2:
        if index + 1 >= len(run) - 2 or run[index] != "--mount":
            raise PlanError("container run has an extra or malformed argument")
        observed_mounts.append(run[index + 1])
        index += 2
    if observed_mounts != [f"type=bind,src={output_root},dst={dest},{mode}"
                            if dest == "/workspace/capture" else
                            f"type=bind,src={source_for_dest[dest].resolve()},dst={dest},{mode}"
                            for dest, mode in mounts]:
        raise PlanError("container run mounts are not exact")
    if workflow[4]["expected_exit_status"] != 1 or workflow[21]["expected_exit_status"] != 1:
        raise PlanError("container absence phases must expect only exact absence status")
    run_index = 5
    for item in workflow:
        if item["phase"].startswith("compose") or item["phase"] in {
                "seal_candidate", "seal_base_snapshot", "materialize_source_snapshot",
                "build_stage", "runtime_stage", "cleanup"}:
            for path in item.get("host_paths", []):
                if not Path(path).is_absolute() or "/workspace/" in str(path):
                    raise PlanError("host phase contains a container path")
        for key in ("host_stdout_path", "host_argv_path"):
            if item.get(key) is not None:
                if not Path(item[key]).is_absolute() or "/workspace/" in item[key]:
                    raise PlanError("host log/argv path contains a container path")
        if item.get("container_output_path") is not None and \
                (not item["container_output_path"].startswith("/workspace/capture/") or
                 any(part in {"", ".", ".."} for part in
                     item["container_output_path"][len("/workspace/capture/"):].split("/") or
                     []) or "\\" in item["container_output_path"]):
            raise PlanError("container output path is not on the writable output mount")
        if item["phase"] != "container_run" and item["argv"][:2] == ["docker", "exec"]:
            if item["ordinal"] <= run_index:
                raise PlanError("docker exec precedes container creation")
    for role, phase in (("build", "build_resolver"), ("runtime", "runtime_resolver")):
        argv = next(item["argv"] for item in workflow if item["phase"] == phase)
        if argv.count("install") != 1 or argv.index("install") <= 0:
            raise PlanError(f"{role} resolver lacks exact install subcommand")
    build_cache = _host_path(output_root, "apt-cache", "build")
    runtime_cache = _host_path(output_root, "apt-cache", "runtime")
    if build_cache == runtime_cache:
        raise PlanError("build/runtime host archive roots must be distinct")


def validate_fixed_workflow(plan_value: Mapping[str, Any]) -> None:
    """Reopen and validate a planned workflow after transport/storage.

    This is intentionally independent of execution.  It is used by outer
    receipt code and adversarial tests to reject omitted/reordered phases,
    altered mounts, host/container path confusion, or a resolver without the
    explicit ``install`` subcommand.
    """
    try:
        roots = plan_value["fresh_roots"]
        output_root = Path(roots["output"])
        container_name = str(plan_value["container_name"])
        image_reference = str(plan_value["pinned_image"]["reference"])
        mount_docs = plan_value["mounts"]
        workflow = plan_value["workflow"]
    except (KeyError, TypeError) as error:
        raise PlanError("planned workflow identity is incomplete") from error
    if not isinstance(mount_docs, list) or not isinstance(workflow, list):
        raise PlanError("planned workflow/mounts are not lists")
    if plan_value.get("host_executor") != {
            "status": HOST_EXECUTOR_STATUS, "entrypoint": "", "sha256": "0" * 64}:
        raise PlanError("host executor is not explicitly absent/fail-closed")
    mounts: list[tuple[str, str]] = []
    source_for_dest: dict[str, Path] = {}
    for item in mount_docs:
        if not isinstance(item, Mapping) or not isinstance(item.get("destination"), str) or \
                not isinstance(item.get("read_only"), bool):
            raise PlanError("planned mount is malformed")
        mounts.append((item["destination"], "ro" if item["read_only"] else "rw"))
        if item["destination"] != "/workspace/capture":
            source_for_dest[item["destination"]] = Path(str(item["source"]))
            actual, _data = _regular_sha(source_for_dest[item["destination"]],
                                         f"planned mount {item['destination']}")
            if item.get("source_state") != "EXISTS" or item.get("sha256") != actual:
                raise PlanError(f"planned mount identity drift: {item['destination']}")
        elif item.get("source_state") != "CREATED_BY_WORKFLOW" or \
                item.get("sha256") != "0" * 64 or Path(str(item.get("source"))).exists():
            raise PlanError("planned output mount is not a fresh workflow-created root")
    _validate_workflow(workflow, output_root=output_root,
                       image_reference=image_reference, container_name=container_name,
                       mounts=mounts, source_for_dest=source_for_dest)
    for role in ("build", "runtime"):
        phase = f"{role}_resolver"
        expected = next(item["argv"][3:] for item in workflow if item["phase"] == phase)
        if plan_value.get("resolver_commands", {}).get(role) != expected:
            raise PlanError(f"{role} resolver command identity drift")
    projection = dict(plan_value)
    projection.pop("plan_identity_sha256", None)
    expected_identity = hashlib.sha256(
        json.dumps(projection, sort_keys=True, separators=(",", ":"),
                   ensure_ascii=True).encode()).hexdigest()
    if plan_value.get("plan_identity_sha256") != expected_identity:
        raise PlanError("planned workflow plan identity drift")


def plan(*, requirements_path: Path, output_root: Path,
         outer_root: Path | None = None, image_reference: str = BASE_IMAGE) -> dict[str, Any]:
    requirements, requirements_file_sha = _requirements(Path(requirements_path))
    requirements_path = Path(requirements_path)
    output_root = _fresh(Path(output_root), "acquisition output root")
    outer_root = _fresh(Path(str(output_root) + ".outer") if outer_root is None else Path(outer_root),
                        "outer evidence root")
    name = "glim-r3-apt-closure-" + hashlib.sha256(str(output_root).encode()).hexdigest()[:20]
    workflow = fixed_workflow(image_reference=image_reference, container_name=name,
                              output_root=output_root, requirements=requirements,
                              requirements_path=requirements_path)
    mounts = [
        {"source": str(output_root), "destination": "/workspace/capture",
         "read_only": False, "sha256": "0" * 64, "source_state": "CREATED_BY_WORKFLOW"},
        _mount(requirements_path, "/opt/package-requirements.json", read_only=True),
        _mount(COMPOSER_PATH, "/opt/apt_allowlist_composer.py", read_only=True),
        _mount(REQUIREMENTS_SCHEMA_PATH, "/opt/apt_package_requirements.schema.json",
               read_only=True),
        _mount(RESOLVER_SCHEMA_PATH, "/opt/apt_resolver_output.schema.json", read_only=True),
        _mount(PROPOSAL_SCHEMA_PATH, "/opt/apt_allowlist_generation_proposal.schema.json",
               read_only=True),
        _mount(COLLECTOR_PATH, "/opt/apt_closure_collector.py", read_only=True),
    ]
    source_bindings = {item["destination"]: {"source": item["source"],
                                               "sha256": item["sha256"]}
                       for item in mounts if item["destination"] != "/workspace/capture"}
    plan_value: dict[str, Any] = {
        "schema": SCHEMA, "schema_version": 1,
        "status": "PROPOSED_REVIEW_REQUIRED", "benchmark_eligible": False,
        "execution": "NOT_RUN", "network_used": "PROVISIONING_ONLY_NOT_RUN",
        "pinned_image": {"reference": image_reference, "platform": "linux/amd64",
                          "ros_distribution": "jazzy", "pull_policy": "never"},
        "requirements": {"path": Path(requirements_path).name,
                          "file_sha256": requirements_file_sha,
                          "canonical_sha256": requirements["canonical_sha256"]},
        "fresh_roots": {"output": str(output_root), "outer": str(outer_root),
                        "build_archive": str(output_root / "apt-cache" / "build"),
                        "runtime_archive": str(output_root / "apt-cache" / "runtime")},
        "container_paths": {"output": "/workspace/capture",
                             "build_archive": "/workspace/capture/apt-cache/build",
                             "runtime_archive": "/workspace/capture/apt-cache/runtime"},
        "container_name": name,
        "roles": {"build": {"top_level": _requirements_names(requirements, "build"),
                              "resolver": "print_uris_then_download_only"},
                  "runtime": {"top_level": _requirements_names(requirements, "runtime"),
                                "resolver": "print_uris_then_download_only"}},
        "workflow": workflow,
        "resolver_commands": {
            "build": next(item["argv"][3:] for item in workflow
                          if item["phase"] == "build_resolver"),
            "runtime": next(item["argv"][3:] for item in workflow
                             if item["phase"] == "runtime_resolver")},
        "mounts": mounts,
        "source_bindings": source_bindings,
        "outer_receipt_schema": "glim_clean_room_r3_apt_allowlist_outer_receipt_v1",
        "host_executor": {"status": HOST_EXECUTOR_STATUS, "entrypoint": "",
                           "sha256": "0" * 64},
        "host_execution_contract": {
            "host_output_paths_only": True,
            "executor_status": HOST_EXECUTOR_STATUS,
            "resolver_documents_must_be_runtime_generated": True,
            "network_phases": ["apt_update", "base_status_snapshot", "apt_source_snapshot",
                                "rosdep_snapshot", "build_resolver", "build_download",
                                "runtime_resolver", "runtime_download"],
            "post_disconnect_phases": ["post_disconnect_inspect", "stop", "remove",
                                        "post_remove_inspect"]},
        "union_policy": "canonical union of separately resolved versioned identities; base-installed excluded",
        "review_gate": {"proposal_status": "PROPOSED_REVIEW_REQUIRED",
                        "custodian_signature_required": True,
                        "collector_input_allowed": False,
                        "production_manifest_modified": False},
    }
    plan_value["plan_identity_sha256"] = hashlib.sha256(
        json.dumps(plan_value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()
    return plan_value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--requirements", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--outer-root", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(plan(requirements_path=args.requirements, output_root=args.output_root,
                              outer_root=args.outer_root), sort_keys=True))
        return 0
    except (PlanError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
