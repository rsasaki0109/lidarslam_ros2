#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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
"""Run one clean, host-only Jazzy validation of the registration-plugin chain.

The command builder deliberately keeps colcon's global options (especially
``--log-base``) before the verb.  This is a host validation tool: it never
copies or edits the source tree and it never starts Docker, opens benchmark
data, or accesses the evidence volume.  A work root is single-use and all
build, install, test, colcon-home, and receipt files are placed below it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET


TARGET_PACKAGES = (
    "ndt_omp_ros2",
    "lidarslam_plugin_interfaces",
    "lidarslam_registration_loader",
    "lidarslam_default_plugins",
    "lidarslam_fake_registration_plugins",
    "lidarslam_msgs",
    "scanmatcher",
    "graph_based_slam",
)
EXPECTED_INSTALLED_MODULE_COUNT = 298
HISTORICAL_SAMPLE_LIMIT = 8
TEST_TIMEOUT_SECONDS = 1800
CONTRACT_FIELDS = (
    "schema",
    "schema_version",
    "class_id",
    "plugin_xml_sha256",
    "plugin_xml_size_bytes",
    "dso_sha256",
    "dso_size_bytes",
    "abi_epoch",
    "toolchain_tag",
    "interface_contract_sha256",
    "api_min_major",
    "api_min_minor",
    "api_max_major",
    "api_max_minor",
    "required_capability_bits",
    "optional_capability_bits",
    "target_policy",
    "correspondence_metric",
    "thread_model",
    "cancellation_model",
    "config_schema_id",
    "config_schema_version",
    "config_schema_sha256",
    "manifest_sha256",
)
HEX_SHA = re.compile(r"^[0-9a-f]{64}$")
ROOT_PREFIX = "lidarslam-plugin-chain-colcon.jazzy-validation."
REPO = Path(__file__).resolve().parents[1]


class ValidationError(RuntimeError):
    """A fail-closed preflight or postcondition failure."""

    def __init__(self, kind: str, message: str):
        super().__init__(message)
        self.kind = kind


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _path_is_within(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def validate_fresh_root(path: Path, repo: Path = REPO) -> Path:
    """Validate a not-yet-created, absolute work root and return its path."""
    raw = str(path)
    if not raw or "\x00" in raw or "\n" in raw or "\r" in raw:
        raise ValidationError("UNSAFE_WORK_ROOT", "work root contains unsafe characters")
    if not path.is_absolute() or path != path.absolute() or path != path.resolve(strict=False):
        raise ValidationError("UNSAFE_WORK_ROOT", "work root must be an absolute canonical path")
    if _path_is_within(path, repo) or _path_is_within(repo, path):
        raise ValidationError("WORK_ROOT_SOURCE_OVERLAP", "work root overlaps the source tree")
    if path.exists() or path.is_symlink():
        raise ValidationError("WORK_ROOT_NOT_FRESH", "work root already exists")
    if path.name in {"", ".", ".."} or path == Path("/"):
        raise ValidationError("UNSAFE_WORK_ROOT", "work root is too broad")
    return path


def underlay_paths(distro: str) -> tuple[Path, Path]:
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", distro):
        raise ValidationError("UNSAFE_DISTRO", "distro is not a simple identifier")
    prefix = Path("/opt/ros") / distro
    setup = prefix / "setup.bash"
    if not setup.is_file() or setup.is_symlink():
        raise ValidationError("ROS_UNDERLAY_UNAVAILABLE", f"exact underlay is unavailable: {setup}")
    return prefix, setup


def validate_clean_underlay_environment(environment: dict[str, str], distro: str, repo: Path = REPO) -> None:
    """Reject inherited underlays, source paths, and repository PYTHONPATH."""
    prefix, _ = underlay_paths(distro)
    allowed = str(prefix)
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        value = environment.get(key, "")
        if value and any(item and Path(item).resolve() != prefix.resolve() for item in value.split(os.pathsep)):
            raise ValidationError("OLD_UNDERLAY", f"{key} contains a non-/opt/ros/{distro} path")
    pythonpath = environment.get("PYTHONPATH", "")
    if str(repo.resolve()) in pythonpath or any(
        _path_is_within(Path(item), repo) for item in pythonpath.split(os.pathsep) if item
    ):
        raise ValidationError("REPOSITORY_PYTHONPATH", "repository path leaked into PYTHONPATH")
    if allowed not in (environment.get("AMENT_PREFIX_PATH", "") or allowed):
        raise ValidationError("ROS_UNDERLAY_MISSING", "requested ROS underlay is not selected")


def clean_environment(distro: str, work_root: Path, repo: Path = REPO) -> dict[str, str]:
    """Construct an environment with no inherited underlay or source path."""
    prefix, _ = underlay_paths(distro)
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    ros_python = prefix / "lib" / f"python{python_version}" / "site-packages"
    env = {
        "HOME": os.environ.get("HOME", str(Path.home())),
        "PATH": f"{prefix / 'bin'}:/usr/local/bin:/usr/bin:/bin",
        "ROS_DISTRO": distro,
        "ROS_VERSION": "2",
        "ROS_PYTHON_VERSION": str(sys.version_info.major),
        "AMENT_PREFIX_PATH": str(prefix),
        "CMAKE_PREFIX_PATH": str(prefix),
        "PYTHONPATH": str(ros_python),
        "LD_LIBRARY_PATH": f"{prefix / 'lib'}:/usr/lib/x86_64-linux-gnu:/usr/lib",
        "COLCON_HOME": str(work_root / "colcon-home"),
        "ROS_LOG_DIR": str(work_root / "ros-logs"),
        "CMAKE_BUILD_PARALLEL_LEVEL": "1",
        "MAKEFLAGS": "-j1",
        "NINJAFLAGS": "-j1",
        "LC_ALL": "C.UTF-8",
        "LANG": "C.UTF-8",
    }
    validate_clean_underlay_environment(env, distro, repo)
    return env


def _global_colcon_argv(verb: str, log_root: Path) -> list[str]:
    if verb not in {"build", "test", "test-result"}:
        raise ValueError(f"unsupported colcon verb: {verb}")
    return ["colcon", "--log-base", str(log_root), verb]


def build_colcon_argv(verb: str, source_root: Path, roots: dict[str, Path]) -> list[str]:
    """Build one deterministic colcon command with global options first."""
    argv = _global_colcon_argv(verb, roots["log"])
    if verb == "build":
        argv.extend([
            "--base-paths", str(source_root),
            "--build-base", str(roots["build"]),
            "--install-base", str(roots["install"]),
            "--merge-install",
            "--parallel-workers", "1",
            "--packages-select", *TARGET_PACKAGES,
            "--event-handlers", "console_direct+",
            "--cmake-args", "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
            "-DBUILD_TESTING=ON",
            "-DLIDARSLAM_BUILD_REGISTRATION_STRESS=ON",
            "-DLIDARSLAM_REGISTRATION_STRESS_TSAN=OFF",
        ])
    elif verb == "test":
        argv.extend([
            "--base-paths", str(source_root),
            "--build-base", str(roots["build"]),
            "--install-base", str(roots["install"]),
            "--merge-install",
            "--test-result-base", str(roots["test_results"]),
            "--parallel-workers", "1",
            "--packages-select", *TARGET_PACKAGES,
            "--event-handlers", "console_direct+",
        ])
    else:
        argv.extend(["--test-result-base", str(roots["test_results"]), "--verbose"])
    return argv


def assert_global_options_before_verb(argv: list[str]) -> None:
    try:
        verb_index = next(index for index, value in enumerate(argv) if value in {"build", "test", "test-result"})
    except StopIteration as exc:
        raise ValidationError("COLCON_VERB_MISSING", "colcon verb is missing") from exc
    if "--log-base" not in argv[:verb_index]:
        raise ValidationError("COLCON_GLOBAL_OPTION_ORDER", "--log-base must precede the colcon verb")
    if "--log-base" in argv[verb_index + 1:]:
        raise ValidationError("COLCON_GLOBAL_OPTION_ORDER", "--log-base may not appear after the verb")
    if verb_index == 0 or argv[0] != "colcon":
        raise ValidationError("COLCON_EXECUTABLE", "argv must begin with colcon")


def _priority_argv(argv: list[str]) -> list[str]:
    prefix: list[str] = []
    ionice = shutil.which("ionice")
    nice = shutil.which("nice")
    if ionice:
        prefix.extend([ionice, "-c", "3"])
    if nice:
        prefix.extend([nice, "-n", "19"])
    return prefix + argv


def _shell_argv(underlay: Path, argv: list[str]) -> list[str]:
    quoted = shlex.join(_priority_argv(argv))
    return ["bash", "--noprofile", "--norc", "-c", f"source {shlex.quote(str(underlay / 'setup.bash'))} && exec {quoted}"]


def _source_files(repo: Path) -> list[Path]:
    ignored = {".git", "build", "install", "log", "test-results", "__pycache__", ".pytest_cache"}
    files: list[Path] = []
    for path in repo.rglob("*"):
        if not path.is_file() or path.is_symlink() or any(part in ignored for part in path.relative_to(repo).parts):
            continue
        files.append(path)
    return sorted(files, key=lambda item: item.relative_to(repo).as_posix())


def source_manifest(repo: Path) -> dict[str, object]:
    entries = []
    for path in _source_files(repo):
        relative = path.relative_to(repo).as_posix()
        payload = path.read_bytes()
        entries.append({"path": relative, "size_bytes": len(payload), "sha256": sha256_bytes(payload)})
    return {
        "schema": "lidarslam-host-validation-source-manifest-v1",
        "entries": entries,
        "manifest_sha256": sha256_bytes(_canonical(entries).encode("utf-8")),
    }


def require_source_manifest_match(start: dict[str, object], end: dict[str, object]) -> None:
    """Fail closed if any source byte changed during the external build."""
    if start != end or start.get("manifest_sha256") != end.get("manifest_sha256"):
        raise ValidationError(
            "SOURCE_MANIFEST_DRIFT",
            "source manifest changed between validation start and completion",
        )


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    path.chmod(stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)


def _run_logged(argv: list[str], env: dict[str, str], cwd: Path, log_path: Path, timeout: int | None = None) -> dict[str, object]:
    started = time.time()
    with log_path.open("w", encoding="utf-8") as stream:
        stream.write("$ " + shlex.join(argv) + "\n")
        stream.flush()
        try:
            completed = subprocess.run(
                argv, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT,
                check=False, timeout=timeout,
            )
            returncode = completed.returncode
            timed_out = False
        except subprocess.TimeoutExpired:
            returncode = 124
            timed_out = True
            stream.write("\nTIMEOUT\n")
    return {
        "argv": argv,
        "returncode": returncode,
        "timeout_seconds": timeout,
        "timed_out": timed_out,
        "duration_seconds": round(time.time() - started, 3),
        "log": str(log_path),
        "log_sha256": sha256_file(log_path),
    }


def _contract_identity(manifest: dict[str, object]) -> str:
    order = (
        ("schema", "schema"), ("schema_version", "schema_version"), ("class_id", "class_id"),
        ("plugin_xml_sha256", "plugin_xml_sha256"), ("plugin_xml_size_bytes", "plugin_xml_size_bytes"),
        ("dso_sha256", "dso_sha256"), ("dso_size_bytes", "dso_size_bytes"), ("abi_epoch", "abi_epoch"),
        ("toolchain_tag", "toolchain_tag"), ("interface_contract_sha256", "interface_contract_sha256"),
        ("api_min_major", "api_min_major"), ("api_min_minor", "api_min_minor"),
        ("api_max_major", "api_max_major"), ("api_max_minor", "api_max_minor"),
        ("required_capability_bits", "required_capability_bits"),
        ("optional_capability_bits", "optional_capability_bits"), ("target_policy", "target_policy"),
        ("correspondence_metric", "correspondence_metric"), ("thread_model", "thread_model"),
        ("cancellation_model", "cancellation_model"), ("config_schema_id", "config_schema_id"),
        ("config_schema_version", "config_schema_version"), ("config_schema_sha256", "config_schema_sha256"),
    )
    return "|".join(f"{key}={manifest[name]}" for key, name in order)


def verify_contract_sidecars(install_root: Path) -> dict[str, object]:
    sidecars = sorted(install_root.rglob("*.contract.json"))
    if not sidecars:
        raise ValidationError("CONTRACT_SIDECARS_MISSING", "no installed registration contract sidecars found")
    dsos = [path for path in (install_root / "lib").rglob("*.so*") if path.is_file() and not path.is_symlink()]
    dso_hashes = {sha256_file(path): path for path in dsos}
    records = []
    for sidecar in sidecars:
        if sidecar.is_symlink() or not sidecar.is_file() or not _path_is_within(sidecar, install_root):
            raise ValidationError("CONTRACT_SIDECAR_UNSAFE", f"sidecar is not a regular installed file: {sidecar}")
        try:
            value = json.loads(sidecar.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ValidationError("CONTRACT_SIDECAR_INVALID", str(sidecar)) from exc
        if set(value) != set(CONTRACT_FIELDS):
            raise ValidationError("CONTRACT_SIDECAR_SCHEMA", f"unexpected fields in {sidecar}")
        for field in ("plugin_xml_sha256", "dso_sha256", "interface_contract_sha256", "config_schema_sha256", "manifest_sha256"):
            if not isinstance(value[field], str) or not HEX_SHA.fullmatch(value[field]):
                raise ValidationError("CONTRACT_SIDECAR_HASH", f"invalid {field} in {sidecar}")
        stem = sidecar.name.removesuffix(".contract.json")
        xml_marker = ".xml."
        if xml_marker not in stem:
            raise ValidationError("CONTRACT_XML_PATH", f"cannot derive XML path from {sidecar}")
        xml_path = sidecar.parent / (stem.split(xml_marker, 1)[0] + ".xml")
        if xml_path.is_symlink() or not xml_path.is_file():
            raise ValidationError("CONTRACT_XML_MISSING", f"contract XML is missing: {xml_path}")
        if sha256_file(xml_path) != value["plugin_xml_sha256"] or xml_path.stat().st_size != value["plugin_xml_size_bytes"]:
            raise ValidationError("CONTRACT_XML_DRIFT", f"XML identity mismatch: {xml_path}")
        dso = dso_hashes.get(value["dso_sha256"])
        if dso is None or dso.stat().st_size != value["dso_size_bytes"]:
            raise ValidationError("CONTRACT_DSO_DRIFT", f"DSO identity is absent or stale: {sidecar}")
        if sha256_bytes(_contract_identity(value).encode("utf-8")) != value["manifest_sha256"]:
            raise ValidationError("CONTRACT_MANIFEST_DRIFT", f"manifest digest mismatch: {sidecar}")
        records.append({
            "path": str(sidecar), "sha256": sha256_file(sidecar), "xml": str(xml_path),
            "dso": str(dso), "class_id": value["class_id"],
        })
    return {"count": len(records), "records": records}


def _installed_site_paths(install_root: Path) -> list[Path]:
    paths = sorted(path for path in install_root.rglob("site-packages") if path.is_dir() and not path.is_symlink())
    if not paths:
        raise ValidationError("PYTHON_SURFACE_MISSING", "fresh install has no site-packages directory")
    return paths


def _python_surface_script() -> str:
    return r'''
import importlib
import os
import pathlib
import sys

site_roots = [pathlib.Path(item).resolve() for item in
              (os.environ.get("LIDARSLAM_VALIDATION_SITE_ROOTS", "") or "").split(os.pathsep)
              if item]
if not site_roots:
    site_roots = [pathlib.Path(item).resolve() for item in sys.path
                  if item and (pathlib.Path(item) / "lidarslam_benchmark_tools").is_dir()]
root = next((item for item in site_roots
             if (item / "lidarslam_benchmark_tools").is_dir()), None)
if root is None:
    print("FAIL=<package>|missing-installed-package||")
    raise SystemExit(1)
package = root / "lidarslam_benchmark_tools"
modules = []
for path in sorted(package.rglob("*.py")):
    if path.name == "__init__.py" or "__pycache__" in path.parts:
        continue
    rel = path.relative_to(package).with_suffix("")
    modules.append(".".join(rel.parts))
failures = []
for module in modules:
    name = "lidarslam_benchmark_tools." + module
    try:
        loaded = importlib.import_module(name)
        origin = pathlib.Path(getattr(loaded, "__file__", "")).resolve()
        if not any(site == origin or site in origin.parents for site in site_roots):
            failures.append((name, "outside-install", str(origin)))
    except Exception as exc:
        failures.append((name, type(exc).__name__ + ": " + str(exc), ""))
print("MODULE_COUNT=" + str(len(modules)))
for name, error, origin in failures:
    print("FAIL=" + name + "|" + error + "|" + origin)
raise SystemExit(1 if failures else 0)
'''


def verify_installed_python_surface(
    install_root: Path,
    env: dict[str, str],
    expected_module_count: int = EXPECTED_INSTALLED_MODULE_COUNT,
) -> dict[str, object]:
    site_paths = _installed_site_paths(install_root)
    host_validator = "validate_registration_plugin_jazzy.py"
    if any((path / "lidarslam_benchmark_tools" / host_validator).exists() for path in site_paths):
        raise ValidationError(
            "HOST_VALIDATOR_INSTALLED",
            "host-only validation launcher leaked into the installed benchmark package",
        )
    surface_env = dict(env)
    surface_env["PYTHONPATH"] = os.pathsep.join([str(path) for path in site_paths] + [env["PYTHONPATH"]])
    surface_env["LIDARSLAM_VALIDATION_SITE_ROOTS"] = os.pathsep.join(str(path) for path in site_paths)
    result = subprocess.run(
        [sys.executable, "-c", _python_surface_script()], env=surface_env,
        cwd=install_root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, check=False,
    )
    if result.returncode != 0:
        raise ValidationError("PYTHON_SURFACE_IMPORT", result.stdout[-4000:])
    count = next((int(line.split("=", 1)[1]) for line in result.stdout.splitlines() if line.startswith("MODULE_COUNT=")), 0)
    if count != expected_module_count:
        raise ValidationError(
            "PYTHON_SURFACE_INCOMPLETE",
            f"expected {expected_module_count} installed Python modules, observed {count}",
        )
    wrapper = install_root / "lib" / "graph_based_slam" / "lidarslam_benchmark_tool"
    if not wrapper.is_file() or wrapper.is_symlink():
        raise ValidationError("PYTHON_WRAPPER_MISSING", str(wrapper))
    help_result = subprocess.run(
        [str(wrapper), "--help"], env=surface_env, cwd=install_root,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False,
    )
    if help_result.returncode != 0:
        raise ValidationError("PYTHON_WRAPPER_HELP", help_result.stdout[-4000:])
    return {
        "site_paths": [str(path) for path in site_paths],
        "module_count": count,
        "origin_count": count,
        "wrapper": str(wrapper),
    }


def test_result_summary(
    test_root: Path, *, include_ctest: bool = True, include_junit: bool = True,
) -> dict[str, object]:
    """Summarize CTest Site XML and JUnit XML without treating emptiness as success.

    ``colcon test-result`` consumes CTest ``Testing/Test.xml`` files, while
    ament emits the per-test JUnit files below the build tree.  The previous
    implementation only recognized JUnit suites, so a valid CTest result
    directory was incorrectly recorded as zero tests.  Keep the two formats
    explicit and mark malformed/empty files so a partial run cannot look like
    a clean result.
    """
    summary: dict[str, object] = {
        "files": 0, "tests": 0, "failures": 0, "errors": 0, "skipped": 0,
        "invalid_files": 0, "partial_files": 0,
        "source_kind_counts": {"ctest": 0, "junit": 0},
    }
    for path in sorted(test_root.rglob("*.xml")):
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError):
            summary["files"] = int(summary["files"]) + 1
            summary["invalid_files"] = int(summary["invalid_files"]) + 1
            continue
        is_ctest = root.tag == "Site" or root.find("./Testing") is not None
        if is_ctest and not include_ctest:
            continue
        if not is_ctest and not include_junit:
            continue
        summary["files"] = int(summary["files"]) + 1
        kind = "ctest" if is_ctest else "junit"
        counts = summary["source_kind_counts"]
        assert isinstance(counts, dict)
        counts[kind] = int(counts[kind]) + 1
        if is_ctest:
            tests = root.findall("./Testing/Test")
            if not tests:
                summary["partial_files"] = int(summary["partial_files"]) + 1
                continue
            for test in tests:
                summary["tests"] = int(summary["tests"]) + 1
                status = test.attrib.get("Status", "").lower()
                if status in {"passed", "pass"}:
                    continue
                if status in {"notrun", "not-run", "skipped", "skip"}:
                    summary["skipped"] = int(summary["skipped"]) + 1
                elif status in {"failed", "fail", "timeout", "timedout"}:
                    summary["failures"] = int(summary["failures"]) + 1
                else:
                    summary["errors"] = int(summary["errors"]) + 1
            continue
        suites = [suite for suite in root.iter("testsuite")
                  if not any(child.tag == "testsuite" for child in suite)]
        if not suites and root.tag == "testsuite":
            suites = [root]
        if not suites:
            summary["partial_files"] = int(summary["partial_files"]) + 1
            continue
        for suite in suites:
            for key in ("tests", "failures", "errors", "skipped"):
                try:
                    summary[key] = int(summary[key]) + int(suite.attrib.get(key, "0"))
                except (TypeError, ValueError):
                    summary["invalid_files"] = int(summary["invalid_files"]) + 1
    return summary


def parse_colcon_test_result_summary(log_path: Path) -> dict[str, int]:
    """Parse the single colcon test-result summary and reject missing output."""
    text = log_path.read_text(encoding="utf-8")
    matches = re.findall(
        r"^Summary:\s+(\d+) tests,\s+(\d+) errors,\s+(\d+) failures,\s+(\d+) skipped\s*$",
        text, flags=re.MULTILINE,
    )
    if len(matches) != 1:
        raise ValidationError(
            "TEST_RESULT_SUMMARY_MISSING",
            f"expected exactly one colcon summary in {log_path}, found {len(matches)}",
        )
    tests, errors, failures, skipped = (int(value) for value in matches[0])
    return {
        "tests": tests, "errors": errors, "failures": failures, "skipped": skipped,
    }


def validate_test_result_consistency(
    ctest_summary: dict[str, object], colcon_summary: dict[str, int],
    junit_summary: dict[str, object],
) -> None:
    """Fail closed on zero/partial XML or disagreement with colcon output."""
    for label, summary in (("CTest XML", ctest_summary), ("JUnit XML", junit_summary)):
        for field in ("invalid_files", "partial_files"):
            if int(summary.get(field, 0)):
                raise ValidationError(
                    "TEST_RESULT_XML_PARTIAL",
                    f"{label} contains {summary[field]} invalid/partial XML files",
                )
    if int(ctest_summary.get("files", 0)) == 0 or int(ctest_summary.get("tests", 0)) == 0:
        raise ValidationError(
            "TEST_RESULT_XML_EMPTY",
            "CTest result XML contains no completed test entries",
        )
    if int(junit_summary.get("failures", 0)) or int(junit_summary.get("errors", 0)):
        raise ValidationError(
            "TEST_RESULT_JUNIT_FAILURES",
            "JUnit XML contains recorded failures/errors; refusing to report a clean result",
        )
    for field in ("tests", "errors", "failures", "skipped"):
        if int(ctest_summary[field]) != int(colcon_summary[field]):
            raise ValidationError(
                "TEST_RESULT_SUMMARY_DRIFT",
                f"colcon {field}={colcon_summary[field]} differs from CTest XML "
                f"{ctest_summary[field]}",
            )
    return None


def _historical_root_summary(root: Path) -> dict[str, object] | None:
    failures = []
    xml_count = 0
    for xml in sorted(root.rglob("*.xml")):
        try:
            parsed = ET.parse(xml).getroot()
        except (OSError, ET.ParseError):
            continue
        xml_count += 1
        for case in parsed.iter("testcase"):
            if case.find("failure") is not None or case.find("error") is not None:
                failures.append(case.attrib.get("name", xml.stem))
    if not failures:
        return None
    distinct = sorted(set(failures))
    return {
        "root": str(root),
        "xml_file_count": xml_count,
        "failure_case_count": len(failures),
        "distinct_failure_name_count": len(distinct),
        "distinct_failure_names_sha256": sha256_bytes(_canonical(distinct).encode("utf-8")),
        "sample_failure_names": distinct[:HISTORICAL_SAMPLE_LIMIT],
        "sample_limit": HISTORICAL_SAMPLE_LIMIT,
    }


def historical_failure_map() -> dict[str, object]:
    roots = sorted(Path("/tmp").glob("lidarslam-plugin-chain-colcon.*"))
    records = []
    for root in roots:
        if not root.is_dir() or root.name.startswith(ROOT_PREFIX):
            continue
        summary = _historical_root_summary(root)
        if summary is not None:
            records.append(summary)
    return {"preserved_roots": records, "root_count": len(records), "sample_limit": HISTORICAL_SAMPLE_LIMIT}


def _make_roots(work_root: Path) -> dict[str, Path]:
    roots = {
        "build": work_root / "build",
        "install": work_root / "install",
        "log": work_root / "colcon-log",
        "test_results": work_root / "test-results",
    }
    for path in roots.values():
        path.mkdir(parents=True, exist_ok=False)
    (work_root / "colcon-home").mkdir()
    (work_root / "ros-logs").mkdir()
    return roots


def run_validation(repo: Path, distro: str, work_root: Path) -> int:
    validate_fresh_root(work_root, repo)
    prefix, setup = underlay_paths(distro)
    work_root.mkdir(parents=True, exist_ok=False)
    roots = _make_roots(work_root)
    env = clean_environment(distro, work_root, repo)
    source = source_manifest(repo)
    _write_json(work_root / "source-manifest.json", source)
    _write_json(work_root / "environment.json", {
        "distro": distro, "underlay_prefix": str(prefix), "underlay_setup": str(setup),
        "environment": env, "target_packages": list(TARGET_PACKAGES),
    })
    build = build_colcon_argv("build", repo, roots)
    test = build_colcon_argv("test", repo, roots)
    result = build_colcon_argv("test-result", repo, roots)
    for argv in (build, test, result):
        assert_global_options_before_verb(argv)
    commands = {"build": _shell_argv(prefix, build), "test": _shell_argv(prefix, test), "test-result": _shell_argv(prefix, result)}
    _write_json(work_root / "commands.json", commands)
    build_record = _run_logged(commands["build"], env, repo, work_root / "build.stdout.log")
    records: dict[str, object] = {"build": build_record, "historical": historical_failure_map()}
    source_end = source_manifest(repo)
    _write_json(work_root / "source-manifest-end.json", source_end)
    records["source_manifest"] = {
        "start_sha256": source["manifest_sha256"],
        "end_sha256": source_end["manifest_sha256"],
        "status": "PASS" if source == source_end else "FAIL",
    }
    try:
        require_source_manifest_match(source, source_end)
    except ValidationError as error:
        records["post_build"] = {"status": "FAIL", "kind": error.kind, "message": str(error)}
        _write_json(work_root / "validation.json", records)
        return 1
    if build_record["returncode"] == 0:
        try:
            records["contracts"] = verify_contract_sidecars(roots["install"])
            records["python_surface"] = verify_installed_python_surface(roots["install"], env)
            records["post_build"] = "PASS"
        except ValidationError as error:
            records["post_build"] = {"status": "FAIL", "kind": error.kind, "message": str(error)}
            _write_json(work_root / "validation.json", records)
            return 1
        test_record = _run_logged(
            commands["test"], env, repo, work_root / "test.stdout.log",
            timeout=TEST_TIMEOUT_SECONDS,
        )
        result_record = _run_logged(commands["test-result"], env, repo, work_root / "test-result.stdout.log")
        records["test"] = test_record
        records["test_result"] = result_record
        ctest_summary = test_result_summary(
            roots["test_results"], include_ctest=True, include_junit=False,
        )
        junit_summary = test_result_summary(
            roots["build"], include_ctest=False, include_junit=True,
        )
        records["test_summary"] = ctest_summary
        records["junit_summary"] = junit_summary
        summary_ok = True
        try:
            colcon_summary = parse_colcon_test_result_summary(
                work_root / "test-result.stdout.log")
            records["colcon_test_result_summary"] = colcon_summary
            validate_test_result_consistency(
                ctest_summary, colcon_summary, junit_summary)
        except ValidationError as error:
            summary_ok = False
            records["test_summary_validation"] = {
                "status": "FAIL_CLOSED", "kind": error.kind, "message": str(error),
            }
        else:
            records["test_summary_validation"] = {"status": "PASS"}
        records["test_summary_ok"] = summary_ok
    else:
        records["post_build"] = "NOT_RUN_BUILD_FAILED"
    _write_json(work_root / "validation.json", records)
    if build_record["returncode"] != 0:
        return int(build_record["returncode"])
    return 0 if (
        records["test"]["returncode"] == 0
        and records["test_result"]["returncode"] == 0
        and records.get("test_summary_ok") is True
    ) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distro", default="jazzy")
    parser.add_argument("--repo", type=Path, default=REPO)
    parser.add_argument("--work-root", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    repo = args.repo.resolve()
    if not repo.is_dir():
        raise SystemExit("source tree does not exist")
    if args.work_root is None:
        if args.dry_run:
            work = Path("/tmp") / (ROOT_PREFIX + "dry-run")
        else:
            work = Path(tempfile.mkdtemp(prefix=ROOT_PREFIX))
            # mkdtemp has already created the root; move validation to the
            # command path without allowing a reused explicit root.
            shutil.rmtree(work)
    else:
        work = args.work_root
    validate_fresh_root(work, repo)
    prefix, _ = underlay_paths(args.distro)
    roots = {"build": work / "build", "install": work / "install", "log": work / "colcon-log", "test_results": work / "test-results"}
    build = build_colcon_argv("build", repo, roots)
    test = build_colcon_argv("test", repo, roots)
    result = build_colcon_argv("test-result", repo, roots)
    for command in (build, test, result):
        assert_global_options_before_verb(command)
    if args.dry_run:
        print(json.dumps({"distro": args.distro, "underlay": str(prefix), "work_root": str(work), "commands": [build, test, result]}, indent=2))
        return 0
    return run_validation(repo, args.distro, work)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValidationError as error:
        print(f"{error.kind}: {error}", file=sys.stderr)
        raise SystemExit(2)
