#!/usr/bin/env python3
"""Audit that production registration processing enters a session boundary.

This is a read-only source audit.  It deliberately does not inspect or run a
ROS graph: the invariant is that the live frontend and graph shells cannot
silently bypass the session lease with a raw PCL or raw RegistrationPlugin
call.  Small standalone odometry/analysis programs are separate, explicitly
allowlisted seams and are not treated as live frontend/backend consumers.
"""

# Copyright 2026 Sasaki
# All rights reserved.

from __future__ import print_function

import argparse
import json
import re
import sys
from pathlib import Path


PRODUCTION_FILES = (
    "scanmatcher/src/scanmatcher_component.cpp",
    "graph_based_slam/src/graph_based_slam_component.cpp",
    "graph_based_slam/src/graph_slam_offline_runner.cpp",
)

# These are processing calls, not metadata/configuration accessors.  The
# prefix is intentionally narrow so session calls such as
# registration_plugin_session_->align() remain valid.
RAW_PROCESSING_RE = re.compile(
    r"\b(?:registration_|registration_plugin_)\s*->\s*"
    r"(?:align|setInputTarget|setInputSource|getFitnessScore|"
    r"getFinalTransformation|hasConverged)\s*\("
)

RAW_TERNARY_RE = re.compile(
    r"registration_plugin_session_\s*!=\s*nullptr\s*\?"
)

REQUIRED_MARKERS = {
    "scanmatcher/src/scanmatcher_component.cpp": (
        "RegistrationResolver",
        "RegistrationActivationTransaction",
        "registration_plugin_session_->align(",
        "registration_plugin_session_->setInputTarget(",
        "makeHostBuiltinNdtRegistration",
        "makeHostBuiltinGicpRegistration",
    ),
    "graph_based_slam/src/graph_based_slam_component.cpp": (
        "RegistrationPluginSession::createHostSession",
        "RegistrationPluginSessionAdapter",
        "RegistrationActivationTransaction",
    ),
    "graph_based_slam/src/graph_slam_offline_runner.cpp": (
        "RegistrationPluginSession::createHostSession",
        "RegistrationPluginSessionAdapter",
        "RegistrationActivationTransaction",
    ),
}

ALLOWLISTED_STANDALONE_FILES = {
    "scanmatcher/src/small_gicp_odom_node.cpp":
        "standalone stateful odometry executable; no ScanMatcherComponent callbacks",
    "graph_based_slam/src/map_ndt_residual_report_main.cpp":
        "offline analysis executable; no live frontend/backend processing",
}


def audit_text(text, relative_path, *, allowlisted=False):
    """Audit one source buffer, returning deterministic diagnostics."""
    if allowlisted:
        return {
            "path": relative_path,
            "status": "ALLOWLISTED_STANDALONE",
            "violations": [],
            "reason": ALLOWLISTED_STANDALONE_FILES.get(relative_path, "explicit seam"),
        }

    violations = []
    for line_number, line in enumerate(text.splitlines(), 1):
        if RAW_PROCESSING_RE.search(line):
            violations.append({
                "line": line_number,
                "kind": "RAW_PROCESSING_CALL",
                "text": line.strip(),
            })
    for match in RAW_TERNARY_RE.finditer(text):
        violations.append({
            "line": text.count("\n", 0, match.start()) + 1,
            "kind": "SESSION_RAW_FALLBACK_TERNARY",
            "text": match.group(0),
        })

    missing = [marker for marker in REQUIRED_MARKERS.get(relative_path, ())
               if marker not in text]
    if missing:
        violations.append({
            "line": 0,
            "kind": "MISSING_SESSION_MARKER",
            "text": ", ".join(missing),
        })
    return {
        "path": relative_path,
        "status": "PASS" if not violations else "FAIL",
        "violations": violations,
    }


def audit_repository(repo_root):
    """Audit all production shell sources without touching runtime state."""
    root = Path(repo_root).resolve()
    files = []
    for relative in PRODUCTION_FILES:
        path = root / relative
        if not path.is_file() or path.is_symlink():
            files.append({
                "path": relative,
                "status": "FAIL",
                "violations": [{
                    "line": 0,
                    "kind": "MISSING_OR_SYMLINK_SOURCE",
                    "text": str(path),
                }],
            })
            continue
        files.append(audit_text(path.read_text(encoding="utf-8"), relative))
    return {
        "schema": "registration-plugin-processing-boundary-audit-v1",
        "schema_version": 1,
        "status": "PASS" if all(item["status"] == "PASS" for item in files) else "FAIL",
        "production_files": files,
        "allowlisted_standalone_files": dict(sorted(ALLOWLISTED_STANDALONE_FILES.items())),
        "fallback_policy": "no implicit raw-PCL or raw-plugin processing fallback",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    report = audit_repository(args.repo)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
