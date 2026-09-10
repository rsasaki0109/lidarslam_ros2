#!/usr/bin/env python3
"""Validate the additive, non-promoting GLIM r3 apt collector candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import stat
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
MANIFEST_PATH = R3 / "apt_closure_collector_candidate.json"
SCHEMA_PATH = R3 / "apt_closure_collector_candidate.schema.json"
BASE_IMAGE = (
    "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)
SHA_RE = re.compile(r"^[0-9a-f]{64}$")

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import registration_plugin_dependency_closure as dependency_closure  # noqa: E402


class CandidateError(ValueError):
    """Raised when the collector candidate is not independently bound."""


def sha256_file(path: Path) -> str:
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or not path.is_file() or info.st_nlink != 1 or info.st_size <= 0:
        raise CandidateError(f"identity file is not a regular single-link file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=True).encode("utf-8")).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CandidateError(f"invalid candidate JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise CandidateError(f"candidate JSON is not an object: {path}")
    return value


def validate() -> dict[str, Any]:
    manifest = _read_json(MANIFEST_PATH)
    schema = _read_json(SCHEMA_PATH)
    if schema.get("$id") != "glim-clean-room-r3-apt-closure-collector-candidate-v1":
        raise CandidateError("collector candidate schema identity drift")
    try:
        import jsonschema
        jsonschema.Draft202012Validator(schema).validate(manifest)
    except ImportError as error:
        raise CandidateError(f"jsonschema backend unavailable: {error}") from error
    if manifest["base_image"]["reference"] != BASE_IMAGE or \
            manifest["status"] != "IMPLEMENTATION_CANDIDATE_NOT_RUNTIME_VALIDATED" or \
            manifest["capture_mode"] != manifest["status"] or \
            manifest["benchmark_eligible"] is not False:
        raise CandidateError("collector candidate is not non-promoting")
    if manifest.get("capture_contract") != dependency_closure.capture_contract():
        raise CandidateError("capture contract binding drift")
    phases = manifest["workflow_phases"]
    expected_phases = [
        "image_inspect", "output_root_absent_check", "output_root_create",
        "output_root_lstat", "preexisting_name_inspect", "container_run",
        "provisioning_update", "acquisition_ledger", "base_status_snapshot",
        "download_only", "stage_downloads", "provisioning_install", "collector", "disconnect",
        "post_disconnect_inspect", "stop", "remove", "post_remove_inspect"]
    if [item["ordinal"] for item in phases] != list(range(18)) or \
            [item["phase"] for item in phases] != expected_phases or \
            [item["expected_exit_status"] for item in phases] != [0, 0, 0, 0, 1, 0,
                                                                    0, 0, 0, 0, 0, 0,
                                                                    0, 0, 0, 0, 0, 1]:
        raise CandidateError("collector workflow phase/exit contract drift")
    if manifest["workflow_phases_sha256"] != canonical_hash(phases):
        raise CandidateError("collector workflow identity drift")
    command = manifest["collector_command"]
    if manifest["collector_command_sha256"] != canonical_hash(command):
        raise CandidateError("collector command identity drift")
    if manifest["mount_contract"] != {
            "count": 6, "plan_authority": "OUTER_RECEIPT_ONLY", "metadata_file": "FORBIDDEN",
            "writable_destinations": ["/workspace/capture"],
            "read_only_destinations": [
                "/opt/apt_closure_collector.py", "/opt/apt_closure_capture.py",
                "/opt/apt_closure_capture.schema.json", "/opt/package-allowlist.json",
                "/opt/apt-acquisition-ledger.json"]}:
        raise CandidateError("collector mount/host-plan contract drift")
    if "--metadata" in command or "container-metadata.json" in command or \
            "--plan" in command or "--base-status" not in command or \
            "--staged-debs" not in command:
        raise CandidateError("collector command still depends on a host plan or missing output paths")
    if any(token in item.lower() for item in command for token in (
            "bash", "sh -c", "--privileged", "network host", "glim_ros2", "glim_ros")):
        raise CandidateError("collector command has forbidden shell/bridge surface")
    relative_files = {
        "collector_tool": manifest["collector_tool"],
        "collector_receipt_schema": manifest["collector_receipt_schema"],
        "staged_deb_manifest": manifest["staged_deb_manifest"],
        "collector_plan_schema": manifest["collector_plan_schema"],
        "candidate_validator": manifest["candidate_validator"],
        "planner": manifest["planner"],
    }
    consumer = manifest["capture_consumer"]
    relative_files.update({
        "capture_tool": {"path": consumer["tool_path"], "sha256": consumer["tool_sha256"]},
        "capture_schema": {"path": consumer["schema_path"], "sha256": consumer["schema_sha256"]},
        "outer_tool": {"path": consumer["outer_tool_path"], "sha256": consumer["outer_tool_sha256"]},
        "outer_schema": {"path": consumer["outer_schema_path"],
                         "sha256": consumer["outer_schema_sha256"]},
        "signature_promotion": {"path": consumer["signature_promotion_path"],
                                 "sha256": consumer["signature_promotion_sha256"]},
        "signature_receipt_index_schema": {
            "path": consumer["signature_receipt_index_schema_path"],
            "sha256": consumer["signature_receipt_index_schema_sha256"]},
    })
    for label, identity in relative_files.items():
        path = ROOT / identity["path"]
        if sha256_file(path) != identity["sha256"]:
            raise CandidateError(f"{label} hash drift")
    if consumer["seal_function"] != "seal_to_capture" or \
            consumer["outer_verifier"] != "apt_closure_outer.verify_outer_root" or \
            consumer["promotion_mode"] != "EXPLICIT_INDEX_REQUIRED":
        raise CandidateError("collector capture/outer connection drift")
    if manifest["network_policy"] != {
            "provisioning_phase": "CONNECTED_ONLY", "collector_phase": "FILESYSTEM_ONLY",
            "build_test_network": "NONE", "docker_build": "NOT_RUN"}:
        raise CandidateError("collector network policy is not truthful")
    if manifest["production_policy"] != {
            "active_r2_modified": False, "production_manifest_modified": False,
            "review_required": True}:
        raise CandidateError("collector production policy is unsafe")
    collector_text = (ROOT / manifest["collector_tool"]["path"]).read_text(encoding="utf-8")
    if any(token in collector_text for token in ("glim_ros2", "glim_ros")):
        raise CandidateError("collector source contains forbidden ROS bridge provenance")
    return {
        "status": "PASS", "candidate_id": manifest["candidate_id"],
        "manifest_sha256": sha256_file(MANIFEST_PATH),
        "schema_sha256": sha256_file(SCHEMA_PATH),
        "capture_mode": manifest["capture_mode"], "benchmark_eligible": False,
        "docker_build": "NOT_RUN", "runtime_collector": "NOT_RUN",
        "active_r2_modified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        result = validate()
    except (CandidateError, OSError, ValueError, json.JSONDecodeError) as error:
        result = {"status": "FAIL", "reason": str(error)}
        if args.json:
            print(json.dumps(result, sort_keys=True))
        else:
            print(f"GLIM r3 apt collector candidate: FAIL: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        print("GLIM r3 apt collector candidate: PASS")
        print("Runtime collector: NOT_RUN")
        print("Docker build: NOT_RUN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
