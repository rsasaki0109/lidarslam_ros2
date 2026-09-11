#!/usr/bin/env python3
"""Validate the additive GLIM Phase 1 recipe without building or downloading.

The validator is intentionally host-only and offline.  It checks that the
recipe names only the three pinned official repositories, that all archive and
license fields are immutable, and that the recipe surface contains no bridge
package token.  Source checkout/archive revalidation is performed by the
launcher when a later task explicitly authorizes a build.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, List, Mapping, Optional


ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "phase1" / "source_closure.json"
DOCKERFILE_PATH = ROOT / "phase1" / "Dockerfile"
LAUNCHER_PATH = ROOT / "phase1" / "phase1_build.py"
SCHEMA_PATH = ROOT / "phase1" / "source_closure.schema.json"
BOOST_TREE_HASH_PATH = ROOT / "phase1" / "boost_tree_hash.py"
CONTAINER_EVIDENCE_PATH = ROOT / "phase1" / "container_evidence.py"
CONTAINER_SCHEMA_PATH = ROOT / "phase1" / "container_receipt.schema.json"
BRIDGE_TOKEN = "glim_" + "ros2"
PINNED_BASE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
EXPECTED = {
    "glim": "faa264a1bce1bda406f73457e35511f56cdc2eaa",
    "gtsam": "2f3e56c0ddbd3a1aa54ed043643b553d26a069f6",
    "gtsam_points": "9d32e7dbecf6015560d84b4901d6b0a6f483ec46",
}
EXPECTED_CANDIDATE_ID = "glim-clean-room-phase1-core-public-api-v2"
EXPECTED_BOOST = {
    "name": "boost",
    "version": "1.83.0",
    "release_url": "https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2",
    "archive_filename": "boost_1_83_0.tar.bz2",
    "archive_sha256": "6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e",
    "archive_bytes": 122892751,
    "archive_root": "boost_1_83_0",
    "source_tree_sha256": "feb033ff2277bff1befa57ca65ede54a2c7a1fd4f6498d238daae8470dd2fbb0",
    "source_tree_file_count": 76253,
    "source_tree_hash_kind": "relative_path_content_sha256_v1_excluding_git_metadata",
    "license_path": "LICENSE_1_0.txt",
    "license_identity": "Boost Software License 1.0",
    "license_sha256": "c9bff75738922193e67fa726fa225535870d2aa1059f91452c411736284ad566",
    "license_bytes": 1338,
}
BOOST_LIBRARIES = [
    "serialization", "graph", "filesystem", "thread", "program_options",
    "date_time", "timer", "chrono", "regex", "system",
]
HEX64 = re.compile(r"^[0-9a-f]{64}$")


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _check_manifest(manifest: Mapping[str, Any]) -> None:
    _require(manifest.get("schema_version") == 2, "schema_version is not 2")
    _require(manifest.get("candidate_id") == EXPECTED_CANDIDATE_ID,
             "candidate identity drift")
    _require(manifest.get("status") == "CANDIDATE_NOT_BENCHMARK_EVIDENCE",
             "candidate must remain benchmark-ineligible")
    _require(manifest.get("base_image", {}).get("reference") == PINNED_BASE,
             "base image digest drift")
    policy = manifest.get("build_policy")
    _require(isinstance(policy, Mapping), "build_policy is missing")
    for key, expected in {
        "build_type": "Release",
        "march_native": False,
        "cuda": False,
        "viewer": False,
        "opencv": False,
        "gtsam_tests": False,
        "gtsam_examples": False,
        "gtsam_timing": False,
        "gtsam_python": False,
        "gtsam_matlab": False,
        "gtsam_tbb": False,
        "gtsam_points_tests": False,
        "gtsam_points_demos": False,
        "gtsam_points_tools": False,
    }.items():
        _require(policy.get(key) == expected, "build policy drift: " + key)
    jobs = policy.get("parallel_jobs")
    _require(isinstance(jobs, int) and 1 <= jobs <= 4,
             "parallel_jobs must be bounded to [1,4]")
    _require(manifest.get("forbidden_bridge_tokens") == ["glim", "ros2"],
             "bridge token policy drift")

    boost = manifest.get("boost_source")
    _require(isinstance(boost, Mapping), "boost_source is missing")
    _require(boost.get("name") == EXPECTED_BOOST["name"],
             "Boost source name drift")
    _require(boost.get("version") == EXPECTED_BOOST["version"],
             "Boost version drift")
    for key in ("release_url", "archive_filename", "archive_root",
                "source_tree_hash_kind"):
        _require(boost.get(key) == EXPECTED_BOOST[key],
                 "Boost source field drift: " + key)
    for key in ("archive_sha256", "source_tree_sha256"):
        _require(boost.get(key) == EXPECTED_BOOST[key] and
                 HEX64.fullmatch(str(boost.get(key))) is not None,
                 "Boost SHA-256 drift: " + key)
    for key in ("archive_bytes", "source_tree_file_count"):
        _require(boost.get(key) == EXPECTED_BOOST[key],
                 "Boost source count drift: " + key)
    licenses = boost.get("license_files")
    _require(isinstance(licenses, list) and len(licenses) == 1,
             "Boost license closure must have exactly one artifact")
    license_entry = licenses[0]
    _require(isinstance(license_entry, Mapping),
             "Boost license entry is malformed")
    for key, expected in {
        "path": EXPECTED_BOOST["license_path"],
        "identity": EXPECTED_BOOST["license_identity"],
        "sha256": EXPECTED_BOOST["license_sha256"],
        "bytes": EXPECTED_BOOST["license_bytes"],
    }.items():
        _require(license_entry.get(key) == expected,
                 "Boost license field drift: " + key)
    build = boost.get("build")
    _require(isinstance(build, Mapping), "Boost build policy is missing")
    for key, expected in {
        "prefix": "/opt/install/boost",
        "jobs": 2,
        "variant": "release",
        "cxxstd": 17,
        "threading": "multi",
        "link": "shared",
        "runtime_link": "shared",
        "march_native": False,
    }.items():
        _require(build.get(key) == expected,
                 "Boost build policy drift: " + key)
    _require(build.get("libraries") == BOOST_LIBRARIES,
             "Boost library closure drift")

    components = manifest.get("components")
    _require(isinstance(components, list) and len(components) == 3,
             "exactly three source components are required")
    observed = set()
    for component in components:
        _require(isinstance(component, Mapping), "component is not an object")
        name = component.get("name")
        _require(name in EXPECTED and name not in observed,
                 "unexpected or duplicate component: " + str(name))
        observed.add(name)
        commit = component.get("commit")
        _require(commit == EXPECTED[name], "commit drift for " + name)
        repository = component.get("repository_url")
        _require(isinstance(repository, str) and
                 re.fullmatch(r"https://github\.com/[A-Za-z0-9_.-]+/"
                              r"[A-Za-z0-9_.-]+\.git", repository) is not None,
                 "repository is not an official HTTPS GitHub URL: " + name)
        archive_url = component.get("archive_url")
        _require(isinstance(archive_url, str) and archive_url.endswith(
            "/archive/" + commit + ".tar.gz"),
                 "archive is not commit-addressed for " + name)
        _require(isinstance(component.get("archive_bytes"), int) and
                 component["archive_bytes"] > 0,
                 "archive byte count is invalid for " + name)
        _require(HEX64.fullmatch(str(component.get("archive_sha256"))) is not None,
                 "archive SHA-256 is invalid for " + name)
        _require(HEX64.fullmatch(str(component.get("source_tree_sha256"))) is not None,
                 "source tree SHA-256 is invalid for " + name)
        _require(component.get("submodules") == [],
                 "unexpected submodule closure for " + name)
        licenses = component.get("license_files")
        _require(isinstance(licenses, list) and licenses,
                 "license artifact closure is missing for " + name)
        for artifact in licenses:
            _require(isinstance(artifact, Mapping), "license entry is malformed")
            path = artifact.get("path")
            _require(isinstance(path, str) and not path.startswith("/") and
                     ".." not in Path(path).parts,
                     "license path is unsafe for " + name)
            _require(HEX64.fullmatch(str(artifact.get("sha256"))) is not None,
                     "license SHA-256 is invalid for " + name)
    _require(observed == set(EXPECTED), "component set is incomplete")


def _check_recipe_surface() -> None:
    for path in (MANIFEST_PATH, DOCKERFILE_PATH, LAUNCHER_PATH, SCHEMA_PATH,
                 BOOST_TREE_HASH_PATH, CONTAINER_EVIDENCE_PATH,
                 CONTAINER_SCHEMA_PATH):
        _require(path.is_file(), "missing recipe file: " + str(path))
        text = path.read_text(encoding="utf-8")
        _require(BRIDGE_TOKEN not in text,
                 "forbidden bridge token in recipe file: " + str(path))

    container_schema = json.loads(
        CONTAINER_SCHEMA_PATH.read_text(encoding="utf-8"))
    _require(container_schema.get("$schema") ==
             "http://json-schema.org/draft-07/schema#",
             "container receipt schema dialect drift")
    _require("supersedes_receipt" in container_schema.get("properties", {}),
             "container receipt schema lacks correction identity")
    _require("CORRECTED_SEALED" in container_schema["properties"]
             ["receipt_status"]["enum"],
             "container receipt schema lacks CORRECTED_SEALED")
    correction = container_schema.get("allOf", [])[0]
    _require("supersedes_receipt" in correction.get("then", {}).get("required", []),
             "container receipt schema does not require supersedes_receipt")
    evidence_text = CONTAINER_EVIDENCE_PATH.read_text(encoding="utf-8")
    for token in ("LDD_REPORT", "LDD_UNRESOLVED", "marker_bounded",
                  "CORRECTED_SEALED", "benchmark_eligible"):
        _require(token in evidence_text,
                 "container evidence parser is missing token: " + token)

    dockerfile = DOCKERFILE_PATH.read_text(encoding="utf-8")
    _require("FROM " + PINNED_BASE + " AS build" in dockerfile,
             "Dockerfile is not pinned to the declared base image")
    _require("AS boost-build" in dockerfile,
             "Dockerfile has no isolated Boost source build stage")
    _require("https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2" in dockerfile,
             "Dockerfile is missing the official Boost release URL")
    _require("sha256sum -c" in dockerfile,
             "Dockerfile does not verify downloaded archive bytes")
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    for component in manifest["components"]:
        _require(component["archive_sha256"] in dockerfile,
                 "Dockerfile is missing archive SHA for " + component["name"])
    _require("phase1-source-licenses.sha256" in dockerfile and
             "sha256sum -c /opt/phase1-source-licenses.sha256" in dockerfile,
             "Dockerfile does not capture and verify license artifacts")
    boost = manifest["boost_source"]
    _require(boost["archive_sha256"] in dockerfile and
             boost["source_tree_sha256"] in dockerfile and
             boost["license_files"][0]["sha256"] in dockerfile,
             "Dockerfile is missing a Boost archive/tree/license identity")
    _require("bootstrap.sh" in dockerfile and "b2" in dockerfile,
             "Dockerfile does not build Boost with bootstrap/b2")
    for token in (
        "--with-serialization", "--with-graph", "--with-filesystem",
        "--with-thread", "--with-program_options", "--with-date_time",
        "--with-timer", "--with-chrono", "--with-regex", "--with-system",
        "variant=release", "cxxstd=17", "threading=multi", "link=shared",
        "-DBOOST_ROOT=/opt/install/boost",
        "-DBOOST_INCLUDEDIR=/opt/install/boost/include",
        "-DBOOST_LIBRARYDIR=/opt/install/boost/lib",
        "-DBoost_NO_SYSTEM_PATHS=ON",
        "LD_LIBRARY_PATH=/opt/install/boost/lib",
    ):
        _require(token in dockerfile,
                 "Dockerfile is missing Boost closure/build token: " + token)
    _require("test -f /usr/include/boost/serialization/serialization.hpp" not in dockerfile and
             "libboost_serialization.so" not in dockerfile.split("AS boost-build", 1)[0],
             "base stage still assumes unavailable system Boost")
    for forbidden in ("apt-get", "apt ", "apk add", "yum install", "dnf install"):
        _require(forbidden not in dockerfile,
                 "recipe must not use package-manager fallback: " + forbidden)
    _require("GLIM_BOOST_PREFIX" in dockerfile,
             "adapter CMake must receive an explicit Boost prefix")
    _require("CMAKE_BUILD_TYPE=Release" in dockerfile,
             "Dockerfile is not an explicit Release build")
    _require("BUILD_WITH_MARCH_NATIVE=OFF" in dockerfile,
             "Dockerfile does not disable native CPU tuning")
    _require("CMAKE_BUILD_PARALLEL_LEVEL=2" in dockerfile,
             "Dockerfile parallelism is not bounded")
    _require("ctest" in dockerfile and "glim_core_public_api_link_smoke" in dockerfile,
             "Dockerfile does not run the public API smoke")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true",
                        help="emit a machine-readable PASS result")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        _require(isinstance(manifest, Mapping), "manifest root is not an object")
        _check_manifest(manifest)
        _check_recipe_surface()
        result = {
            "status": "PASS",
            "candidate_id": manifest["candidate_id"],
            "manifest_sha256": _canonical_sha256(manifest),
            "docker_build": "NOT_RUN",
            "benchmark_execution": "FORBIDDEN",
        }
    except (OSError, ValueError, json.JSONDecodeError) as error:
        if args.json:
            print(json.dumps({"status": "FAIL", "reason": str(error)},
                             sort_keys=True))
        else:
            print("Phase 1 recipe validation: FAIL: " + str(error), file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        print("Phase 1 recipe validation: PASS")
        print("Docker build: NOT_RUN")
        print("Benchmark/dataset/GT/scorer execution: FORBIDDEN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
