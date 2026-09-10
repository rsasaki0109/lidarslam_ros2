#!/usr/bin/env python3
"""Build the additive GLIM public-core link candidate, fail closed.

This launcher has two deliberately separate modes.  ``--validate-only`` is an
offline recipe/source-closure check.  The normal mode builds the three exact
official source checkouts out of tree, installs them into a fresh prefix, and
configures the clean-room CMake smoke against the exported GLIM target.  It
never edits a source checkout, benchmark profile, selection, or receipt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlparse


CANDIDATE_ROOT = Path(__file__).resolve().parents[1]
PHASE1_ROOT = CANDIDATE_ROOT / "phase1"
MANIFEST_PATH = PHASE1_ROOT / "source_closure.json"
VALIDATOR_PATH = PHASE1_ROOT / "validate_phase1_recipe.py"
BASE_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
EXPECTED_COMMITS = {
    "glim": "faa264a1bce1bda406f73457e35511f56cdc2eaa",
    "gtsam": "2f3e56c0ddbd3a1aa54ed043643b553d26a069f6",
    "gtsam_points": "9d32e7dbecf6015560d84b4901d6b0a6f483ec46",
}
BRIDGE_TOKEN = "glim_" + "ros2"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
HEX40 = re.compile(r"^[0-9a-f]{40}$")


class Phase1Failure(RuntimeError):
    """A fail-closed preflight or build failure."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_tree_sha256(path: Path, *, exclude_git: bool = True) -> Tuple[str, int]:
    if path.is_symlink() or not path.is_dir():
        raise Phase1Failure("source tree is not a real directory: " + str(path))
    entries: List[Tuple[str, Path]] = []
    for directory, directory_names, file_names in os.walk(path, followlinks=False):
        directory_path = Path(directory)
        if exclude_git:
            directory_names[:] = [name for name in directory_names if name != ".git"]
        for name in sorted(directory_names):
            directory_entry = directory_path / name
            if directory_entry.is_symlink():
                raise Phase1Failure("symlink in source tree: " + str(directory_entry))
            if not directory_entry.is_dir():
                raise Phase1Failure("non-directory source-tree entry: " + str(directory_entry))
        for name in sorted(file_names):
            candidate = directory_path / name
            metadata = candidate.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                raise Phase1Failure("symlink in source tree: " + str(candidate))
            if not stat.S_ISREG(metadata.st_mode):
                raise Phase1Failure("non-regular source-tree entry: " + str(candidate))
            if metadata.st_nlink != 1:
                raise Phase1Failure("hard-link ambiguity in source tree: " + str(candidate))
            entries.append((candidate.relative_to(path).as_posix(), candidate))
    entries.sort(key=lambda item: item[0])
    digest = hashlib.sha256()
    for relative, candidate in entries:
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        with candidate.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest(), len(entries)


def _run_capture(argv: Sequence[str], cwd: Optional[Path] = None) -> str:
    try:
        result = subprocess.run(
            list(argv), cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            check=False)
    except OSError as error:
        raise Phase1Failure("cannot execute " + str(argv[0]) + ": " + str(error))
    if result.returncode != 0:
        raise Phase1Failure(
            "command failed ({}): {}\n{}".format(
                result.returncode, " ".join(argv), result.stderr.strip()))
    return result.stdout.strip()


def _component_map(manifest: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    components = manifest.get("components")
    if not isinstance(components, list):
        raise Phase1Failure("manifest components are missing")
    result: Dict[str, Mapping[str, Any]] = {}
    for component in components:
        if not isinstance(component, Mapping) or not isinstance(component.get("name"), str):
            raise Phase1Failure("manifest component is malformed")
        result[component["name"]] = component
    if set(result) != set(EXPECTED_COMMITS):
        raise Phase1Failure("manifest component set is not the exact core closure")
    return result


def _verify_archive(component: Mapping[str, Any], archive_dir: Path) -> Dict[str, Any]:
    url = component.get("archive_url")
    if not isinstance(url, str) or not url.startswith("https://github.com/"):
        raise Phase1Failure("archive URL is not official HTTPS GitHub")
    expected_name = Path(urlparse(url).path).name
    archive_candidates = [archive_dir / expected_name]
    repository = str(component.get("repository_url", "")).rstrip("/").rsplit("/", 1)[-1]
    if repository.endswith(".git"):
        repository = repository[:-4]
    commit = str(component.get("commit"))
    archive_candidates.append(archive_dir / (repository + "-" + commit + ".tar.gz"))
    existing = [path for path in archive_candidates if path.exists()]
    if not existing:
        raise Phase1Failure(
            "missing official commit archive (accepted names: " +
            ", ".join(str(path) for path in archive_candidates) + ")")
    if len(existing) > 1:
        raise Phase1Failure("ambiguous local archive aliases: " + ", ".join(str(path) for path in existing))
    archive = existing[0]
    if archive.is_symlink() or not archive.is_file():
        raise Phase1Failure("missing official commit archive: " + str(archive))
    metadata = archive.lstat()
    if metadata.st_nlink != 1:
        raise Phase1Failure("archive has hard-link ambiguity: " + str(archive))
    actual_size = archive.stat().st_size
    actual_sha = _sha256_file(archive)
    if actual_size != component.get("archive_bytes") or actual_sha != component.get("archive_sha256"):
        raise Phase1Failure("archive byte/SHA mismatch: " + str(archive))
    return {
        "path": str(archive.resolve()),
        "bytes": actual_size,
        "sha256": actual_sha,
        "url": url,
    }


def _verify_license_files(source: Path, component: Mapping[str, Any]) -> List[Dict[str, Any]]:
    observed: List[Dict[str, Any]] = []
    artifacts = component.get("license_files")
    if not isinstance(artifacts, list) or not artifacts:
        raise Phase1Failure("license artifact closure is empty for " + str(component.get("name")))
    for artifact in artifacts:
        if not isinstance(artifact, Mapping):
            raise Phase1Failure("malformed license artifact")
        relative = artifact.get("path")
        if not isinstance(relative, str) or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise Phase1Failure("unsafe license path")
        path = source / relative
        if path.is_symlink() or not path.is_file():
            raise Phase1Failure("missing license artifact: " + str(path))
        metadata = path.lstat()
        if metadata.st_nlink != 1:
            raise Phase1Failure("license artifact has hard-link ambiguity: " + str(path))
        actual = _sha256_file(path)
        if actual != artifact.get("sha256"):
            raise Phase1Failure("license SHA mismatch: " + str(path))
        observed.append({
            "path": relative,
            "identity": artifact.get("identity"),
            "sha256": actual,
            "bytes": path.stat().st_size,
        })
    return observed


def _verify_checkout(name: str, source: Path, component: Mapping[str, Any]) -> Dict[str, Any]:
    source = source.resolve()
    if BRIDGE_TOKEN in str(source).lower():
        raise Phase1Failure("source path contains the forbidden bridge component")
    if source.is_symlink() or not source.is_dir():
        raise Phase1Failure("source checkout is not a real directory: " + str(source))
    expected_commit = EXPECTED_COMMITS[name]
    actual_commit = _run_capture(["git", "-C", str(source), "rev-parse", "HEAD"])
    if actual_commit != expected_commit:
        raise Phase1Failure("{} checkout commit mismatch".format(name))
    remote = _run_capture(["git", "-C", str(source), "config", "--get", "remote.origin.url"])
    expected_remote = str(component.get("repository_url"))
    if remote.rstrip("/") != expected_remote.rstrip("/"):
        raise Phase1Failure("{} checkout remote mismatch".format(name))
    status = _run_capture(["git", "-C", str(source), "status", "--porcelain", "--untracked-files=all"])
    if status:
        raise Phase1Failure("{} checkout is dirty".format(name))
    submodules = _run_capture(["git", "-C", str(source), "submodule", "status", "--recursive"])
    if submodules:
        raise Phase1Failure("{} has an unrecorded submodule closure".format(name))
    tree_sha, file_count = _regular_tree_sha256(source)
    if tree_sha != component.get("source_tree_sha256"):
        raise Phase1Failure("{} source tree SHA mismatch".format(name))
    licenses = _verify_license_files(source, component)
    return {
        "name": name,
        "path": str(source),
        "commit": actual_commit,
        "remote": remote,
        "source_tree_sha256": tree_sha,
        "source_tree_file_count": file_count,
        "license_files": licenses,
    }


def _inspect_image(reference: str) -> Dict[str, Any]:
    if shutil.which("docker") is None:
        return {"reference": reference, "status": "NOT_AVAILABLE", "reason": "docker executable missing"}
    try:
        result = subprocess.run(
            ["docker", "image", "inspect", reference],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    except OSError as error:
        return {"reference": reference, "status": "NOT_AVAILABLE", "reason": str(error)}
    if result.returncode != 0:
        return {"reference": reference, "status": "NOT_AVAILABLE", "reason": result.stderr.strip()}
    try:
        payload = json.loads(result.stdout)
        item = payload[0]
    except (ValueError, IndexError, TypeError) as error:
        return {"reference": reference, "status": "INVALID_INSPECT", "reason": str(error)}
    return {
        "reference": reference,
        "status": "AVAILABLE_NOT_BUILT",
        "id": item.get("Id"),
        "repo_digests": item.get("RepoDigests", []),
        "architecture": item.get("Architecture"),
        "os": item.get("Os"),
    }


def _validate_recipe() -> Dict[str, Any]:
    result = subprocess.run(
        [sys.executable, str(VALIDATOR_PATH), "--json"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    if result.returncode != 0:
        raise Phase1Failure("recipe validator failed: " + result.stdout.strip() + " " + result.stderr.strip())
    try:
        value = json.loads(result.stdout)
    except ValueError as error:
        raise Phase1Failure("recipe validator emitted invalid JSON: " + str(error))
    if value.get("status") != "PASS":
        raise Phase1Failure("recipe validator did not PASS")
    return value


def _recipe_files() -> Dict[str, Dict[str, Any]]:
    paths = {
        "manifest": MANIFEST_PATH,
        "schema": PHASE1_ROOT / "source_closure.schema.json",
        "dockerfile": PHASE1_ROOT / "Dockerfile",
        "launcher": Path(__file__).resolve(),
        "validator": VALIDATOR_PATH,
        "boost_tree_hash": PHASE1_ROOT / "boost_tree_hash.py",
    }
    result: Dict[str, Dict[str, Any]] = {}
    for name, path in paths.items():
        if not path.is_file() or path.is_symlink():
            raise Phase1Failure("missing or linked recipe file: " + str(path))
        result[name] = {"path": str(path), "sha256": _sha256_file(path),
                        "bytes": path.stat().st_size}
    return result


def _preflight_sources(
    manifest: Mapping[str, Any], source_paths: Mapping[str, Path], archive_dir: Path
) -> Dict[str, Any]:
    components = _component_map(manifest)
    identities: Dict[str, Any] = {}
    archives: Dict[str, Any] = {}
    for name in ("glim", "gtsam", "gtsam_points"):
        if name not in source_paths:
            raise Phase1Failure("missing source argument for " + name)
        identities[name] = _verify_checkout(name, source_paths[name], components[name])
        archives[name] = _verify_archive(components[name], archive_dir)
    return {
        "components": identities,
        "archives": archives,
        "manifest_sha256": _sha256_file(MANIFEST_PATH),
    }


def _fresh_root(parent: Path, requested: Optional[Path]) -> Path:
    parent = parent.resolve()
    if requested is not None:
        requested = requested.resolve()
        if requested.exists():
            raise Phase1Failure("output root already exists; no reuse is permitted: " + str(requested))
        requested.parent.mkdir(parents=True, exist_ok=True)
        requested.mkdir()
        return requested
    parent.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix="glim-clean-room-phase1.", dir=str(parent)))


def _write_json(path: Path, value: Mapping[str, Any]) -> str:
    if path.exists():
        raise Phase1Failure("refusing to overwrite receipt: " + str(path))
    data = (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n").encode("utf-8")
    temporary = path.with_name(path.name + ".tmp")
    if temporary.exists():
        raise Phase1Failure("temporary receipt already exists: " + str(temporary))
    temporary.write_bytes(data)
    temporary.replace(path)
    return hashlib.sha256(data).hexdigest()


def _run_step(
    root: Path,
    label: str,
    argv: Sequence[str],
    env: Mapping[str, str],
    steps: List[Dict[str, Any]],
) -> None:
    log_path = root / "logs" / (label + ".log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command_text = json.dumps(list(argv), ensure_ascii=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write("command=" + command_text + "\n")
        log.flush()
        result = subprocess.run(
            list(argv), stdout=log, stderr=subprocess.STDOUT,
            env=dict(env), text=True, check=False)
    step = {"label": label, "argv": list(argv), "log": str(log_path),
            "returncode": result.returncode, "log_sha256": _sha256_file(log_path)}
    steps.append(step)
    if result.returncode != 0:
        raise Phase1Failure("build step failed: " + label)


def _cmake_configure(
    root: Path, label: str, source: Path, build: Path, prefix: Path,
    options: Sequence[str], env: Mapping[str, str], steps: List[Dict[str, Any]],
) -> None:
    argv = [
        "cmake", "-S", str(source), "-B", str(build),
        "-G", "Unix Makefiles", "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
    ] + list(options)
    _run_step(root, label + "_configure", argv, env, steps)


def _cmake_build_install(
    root: Path, label: str, build: Path, env: Mapping[str, str], steps: List[Dict[str, Any]],
) -> None:
    _run_step(root, label + "_build",
              ["cmake", "--build", str(build), "--parallel", env["CMAKE_BUILD_PARALLEL_LEVEL"]],
              env, steps)
    _run_step(root, label + "_install",
              ["cmake", "--install", str(build)], env, steps)


def _artifact_inventory(prefixes: Iterable[Path]) -> List[Dict[str, Any]]:
    inventory: List[Dict[str, Any]] = []
    for prefix in prefixes:
        if not prefix.is_dir():
            raise Phase1Failure("install prefix missing: " + str(prefix))
        for path in sorted(prefix.rglob("*"), key=lambda item: item.as_posix()):
            if path.is_symlink():
                target = path.resolve()
                if not target.is_file():
                    raise Phase1Failure("dangling installed symlink: " + str(path))
                inventory.append({
                    "path": str(path.relative_to(prefix)),
                    "kind": "symlink",
                    "target": os.readlink(str(path)),
                    "target_sha256": _sha256_file(target),
                })
            elif path.is_file():
                inventory.append({
                    "path": str(path.relative_to(prefix)),
                    "kind": "regular",
                    "bytes": path.stat().st_size,
                    "sha256": _sha256_file(path),
                })
    if not inventory:
        raise Phase1Failure("installed artifact inventory is empty")
    return inventory


def _scan_no_bridge(root: Path, files: Iterable[Path]) -> None:
    for path in files:
        if not path.is_file() or path.is_symlink():
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        if BRIDGE_TOKEN in text.lower():
            raise Phase1Failure("forbidden bridge token in generated candidate file: " + str(path))


def build(args: argparse.Namespace, manifest: Mapping[str, Any], recipe_result: Mapping[str, Any]) -> int:
    root = _fresh_root(Path(args.output_parent), Path(args.output_root) if args.output_root else None)
    logs = root / "logs"
    logs.mkdir()
    steps: List[Dict[str, Any]] = []
    receipt: Dict[str, Any] = {
        "schema_version": 1,
        "receipt_kind": "glim_clean_room_phase1_public_api_candidate",
        "candidate_id": manifest.get("candidate_id"),
        "status": "FAIL_CLOSED",
        "benchmark_execution": "FORBIDDEN",
        "dataset_execution": "FORBIDDEN",
        "gt_execution": "FORBIDDEN",
        "scorer_execution": "FORBIDDEN",
        "docker_image": {
            "build_status": "NOT_RUN_HOST_DIRECT",
            "base": _inspect_image(args.base_image),
        },
        "recipe_files": _recipe_files(),
        "recipe_validation": dict(recipe_result),
        "steps": steps,
        "root": str(root),
    }
    receipt_path = root / "phase1.receipt.json"
    try:
        archive_dir = Path(args.archive_dir).resolve()
        source_paths = {
            "glim": Path(args.glim_source),
            "gtsam": Path(args.gtsam_source),
            "gtsam_points": Path(args.gtsam_points_source),
        }
        receipt["source_closure"] = _preflight_sources(manifest, source_paths, archive_dir)
        env = os.environ.copy()
        for key in ("ROS_VERSION", "ROS_DISTRO", "ROS_PYTHON_VERSION", "AMENT_PREFIX_PATH",
                    "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_AUTOMATIC_DISCOVERY_RANGE"):
            env.pop(key, None)
        env["CMAKE_BUILD_PARALLEL_LEVEL"] = str(args.jobs)
        env["MAKEFLAGS"] = "-j" + str(args.jobs)
        build_root = root / "build"
        install_root = root / "install"
        gtsam_build = build_root / "gtsam"
        gtsam_install = install_root / "gtsam"
        points_build = build_root / "gtsam_points"
        points_install = install_root / "gtsam_points"
        glim_build = build_root / "glim"
        glim_install = install_root / "glim"
        adapter_build = build_root / "adapter"

        _cmake_configure(
            root, "gtsam", source_paths["gtsam"], gtsam_build, gtsam_install,
            [
                "-DBUILD_SHARED_LIBS=ON", "-DGTSAM_BUILD_TESTS=OFF",
                "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF", "-DGTSAM_BUILD_TIMING_ALWAYS=OFF",
                "-DGTSAM_BUILD_PYTHON=OFF", "-DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF",
                "-DGTSAM_WITH_TBB=OFF", "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF",
                "-DGTSAM_BUILD_UNSTABLE=ON", "-DGTSAM_USE_SYSTEM_EIGEN=ON",
                "-DGTSAM_ENABLE_TIMING=OFF",
            ], env, steps)
        _cmake_build_install(root, "gtsam", gtsam_build, env, steps)

        points_env = dict(env)
        points_env["CMAKE_PREFIX_PATH"] = str(gtsam_install)
        _cmake_configure(
            root, "gtsam_points", source_paths["gtsam_points"], points_build, points_install,
            [
                "-DCMAKE_PREFIX_PATH=" + str(gtsam_install), "-DBUILD_TESTS=OFF",
                "-DBUILD_TESTS_PCL=OFF", "-DBUILD_DEMO=OFF", "-DBUILD_EXAMPLE=OFF",
                "-DBUILD_TOOLS=OFF", "-DBUILD_WITH_TBB=OFF", "-DBUILD_WITH_OPENMP=ON",
                "-DBUILD_WITH_CUDA=OFF", "-DBUILD_WITH_MARCH_NATIVE=OFF",
            ], points_env, steps)
        _cmake_build_install(root, "gtsam_points", points_build, points_env, steps)

        glim_env = dict(env)
        glim_env["CMAKE_PREFIX_PATH"] = str(gtsam_install) + os.pathsep + str(points_install)
        _cmake_configure(
            root, "glim", source_paths["glim"], glim_build, glim_install,
            [
                "-DCMAKE_PREFIX_PATH=" + str(gtsam_install) + ";" + str(points_install),
                "-DBUILD_WITH_CUDA=OFF", "-DBUILD_WITH_VIEWER=OFF",
                "-DBUILD_WITH_OPENCV=OFF", "-DBUILD_WITH_MARCH_NATIVE=OFF",
            ], glim_env, steps)
        _cmake_build_install(root, "glim", glim_build, glim_env, steps)

        adapter_env = dict(env)
        adapter_env["CMAKE_PREFIX_PATH"] = ";".join(
            [str(glim_install), str(points_install), str(gtsam_install)])
        _cmake_configure(
            root, "adapter", CANDIDATE_ROOT, adapter_build, install_root / "adapter",
            [
                "-DCMAKE_PREFIX_PATH=" + str(glim_install) + ";" + str(points_install) + ";" + str(gtsam_install),
                "-DGLIM_CORE_CMAKE_PREFIX=" + str(glim_install),
                "-DGLIM_CORE_COMMIT=" + EXPECTED_COMMITS["glim"],
            ], adapter_env, steps)
        _run_step(root, "adapter_build",
                  ["cmake", "--build", str(adapter_build), "--parallel", str(args.jobs)],
                  adapter_env, steps)
        runtime_env = dict(adapter_env)
        runtime_env["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(glim_install / "lib"), str(points_install / "lib"),
             str(gtsam_install / "lib"), runtime_env.get("LD_LIBRARY_PATH", "")])
        _run_step(root, "adapter_ctest",
                  ["ctest", "--test-dir", str(adapter_build), "--output-on-failure"],
                  runtime_env, steps)

        compile_commands: List[Path] = []
        for build_dir in (gtsam_build, points_build, glim_build, adapter_build):
            candidate = build_dir / "compile_commands.json"
            if candidate.is_file():
                compile_commands.append(candidate)
        _scan_no_bridge(root, compile_commands)
        _scan_no_bridge(root, [glim_install / "lib" / "libglim.so"])
        receipt["compile_commands"] = [
            {"path": str(path), "sha256": _sha256_file(path)} for path in compile_commands
        ]
        receipt["installed_artifacts"] = {
            "gtsam": _artifact_inventory([gtsam_install]),
            "gtsam_points": _artifact_inventory([points_install]),
            "glim": _artifact_inventory([glim_install]),
        }
        receipt["status"] = "PASS"
        receipt["result"] = {
            "public_api_link_smoke": "PASS",
            "contract_and_source_guard": "PASS",
            "benchmark_eligible": False,
        }
    except (Phase1Failure, OSError, ValueError) as error:
        receipt["failure"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        receipt["steps"] = steps
        receipt_sha = _write_json(receipt_path, receipt)
        sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
        if sidecar.exists():
            raise Phase1Failure("refusing to overwrite receipt sidecar")
        sidecar.write_text(receipt_sha + "  " + receipt_path.name + "\n", encoding="ascii")
    print(json.dumps({
        "status": receipt["status"],
        "root": str(root),
        "receipt": str(receipt_path),
        "receipt_sha256": receipt_sha,
        "steps": len(steps),
    }, sort_keys=True))
    return 0 if receipt["status"] == "PASS" else 1


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-only", action="store_true",
                        help="validate recipe and immutable local source/archive identities only")
    parser.add_argument("--glim-source")
    parser.add_argument("--gtsam-source")
    parser.add_argument("--gtsam-points-source")
    parser.add_argument("--archive-dir")
    parser.add_argument("--output-parent", default="/tmp")
    parser.add_argument("--output-root")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--base-image", default=BASE_IMAGE)
    args = parser.parse_args(argv)
    if args.jobs < 1 or args.jobs > 4:
        parser.error("--jobs must be in [1,4]")
    try:
        recipe_result = _validate_recipe()
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping):
            raise Phase1Failure("manifest root is not an object")
        if args.base_image != BASE_IMAGE:
            raise Phase1Failure("base image must remain the pinned digest")
        if args.validate_only:
            if not args.archive_dir or not args.glim_source or not args.gtsam_source or not args.gtsam_points_source:
                raise Phase1Failure("validate-only still requires all exact source/archive paths")
            identities = _preflight_sources(
                manifest,
                {"glim": Path(args.glim_source), "gtsam": Path(args.gtsam_source),
                 "gtsam_points": Path(args.gtsam_points_source)},
                Path(args.archive_dir).resolve())
            identities["docker_image"] = _inspect_image(args.base_image)
            print(json.dumps({"status": "PASS", "mode": "VALIDATE_ONLY", **identities}, sort_keys=True))
            return 0
        required = (args.glim_source, args.gtsam_source, args.gtsam_points_source, args.archive_dir)
        if any(value is None for value in required):
            parser.error("build mode requires all source and archive paths")
        return build(args, manifest, recipe_result)
    except (Phase1Failure, OSError, ValueError, json.JSONDecodeError) as error:
        print("Phase 1 launcher: FAIL_CLOSED: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
