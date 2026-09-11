#!/usr/bin/env python3
"""Offline validator for the additive Phase 3d r3 candidate surface."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.run_glim_clean_room_r3_candidate import (  # noqa: E402
    CANDIDATE_ROOT,
    MANIFEST_PATH,
    CandidateError,
    _manifest,
    canonical_profile_sha256,
    sha256_file,
)
from scripts import registration_plugin_dependency_closure as dependency_closure  # noqa: E402


def validate() -> dict[str, Any]:
    manifest = _manifest()
    schema_path = CANDIDATE_ROOT / "candidate.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    if schema.get("$id") != "glim-clean-room-phase3d-r3-candidate-v1":
        raise CandidateError("candidate schema identity drift")
    if schema.get("properties", {}).get("benchmark_eligible", {}).get("const") is not False:
        raise CandidateError("candidate schema may not permit benchmark eligibility")
    if manifest.get("capture_contract") != dependency_closure.capture_contract():
        raise CandidateError("capture contract binding drift")
    binding = manifest.get("frozen_r2_binding", {})
    if binding.get("selection_id") != "competitive-execution-selection-2026-08-r2" or \
            binding.get("closure_revision") != 2:
        raise CandidateError("candidate is not bound to retained r2 selection")
    profile_path = ROOT / str(binding.get("profile_path", ""))
    selection_path = ROOT / str(binding.get("selection_path", ""))
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    selection = yaml.safe_load(selection_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or canonical_profile_sha256(profile) != \
            binding.get("profile_canonical_sha256"):
        raise CandidateError("retained r2 profile identity drift")
    if sha256_file(selection_path) != binding.get("selection_file_sha256") or \
            not isinstance(selection, dict) or \
            selection.get("selection_id") != binding.get("selection_id") or \
            selection.get("closure_id") != binding.get("closure_id") or \
            selection.get("closure_revision") != binding.get("closure_revision") or \
            selection.get("closure_identity_sha256") != \
            binding.get("closure_identity_sha256"):
        raise CandidateError("retained r2 selection identity drift")
    phase3d_root = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d"
    if not (phase3d_root / "CMakeLists.txt").is_file() or \
            not (phase3d_root / "package.xml").is_file():
        raise CandidateError("Phase 3d install surface is incomplete")
    return {
        "status": "PASS",
        "candidate_id": manifest["candidate_id"],
        "manifest_sha256": sha256_file(MANIFEST_PATH),
        "candidate_schema_sha256": sha256_file(schema_path),
        "docker_build": "NOT_RUN",
        "benchmark_execution": "FORBIDDEN",
        "active_r2_modified": False,
        "legal_status": manifest["source_policy"]["legal_status"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        result = validate()
    except (CandidateError, OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        result = {"status": "FAIL", "reason": str(error)}
        if args.json:
            print(json.dumps(result, sort_keys=True))
        else:
            print("Phase 3d r3 candidate validation: FAIL", file=sys.stderr)
            print(str(error), file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        print("Phase 3d r3 candidate validation: PASS")
        print("Docker build: NOT_RUN")
        print("Benchmark execution: FORBIDDEN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
