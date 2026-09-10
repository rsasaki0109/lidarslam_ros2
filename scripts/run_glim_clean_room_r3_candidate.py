#!/usr/bin/env python3
"""Plan or run the additive GLIM clean-room Phase 3d r3 candidate.

This runner is deliberately separate from the retained r2 GLIM runner.  It
reopens the frozen profile/selection and a caller-provided input manifest,
constructs one digest-pinned, network-disabled container command, and seals
the same per-attempt identity/resource/output/failure boundary used by the
competitive execution contract.  The default is a dry plan; no dataset,
ground truth, or scorer is opened by this module.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Mapping

import yaml


_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (  # noqa: E402,I100
    seal_receipt,
    validate_receipt,
)
from lidarslam_benchmark_tools.competitive_identity_hash import (  # noqa: E402,I100
    canonical_profile_sha256,
)


ROOT = _SOURCE_ROOT
CANDIDATE_ROOT = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
MANIFEST_PATH = CANDIDATE_ROOT / "candidate_manifest.json"
SCHEMA_PATH = CANDIDATE_ROOT / "candidate.schema.json"
OFFLINE_MANIFEST_PATH = CANDIDATE_ROOT / "offline_dependency_manifest.json"
OFFLINE_SCHEMA_PATH = CANDIDATE_ROOT / "offline_dependency_manifest.schema.json"
PREFETCH_TOOL_PATH = CANDIDATE_ROOT / "dependency_prefetch.py"
OFFLINE_PLANNER_PATH = ROOT / "scripts/plan_glim_clean_room_r3_offline_build.py"
APT_CAPTURE_TOOL_PATH = CANDIDATE_ROOT / "apt_closure_capture.py"
APT_CAPTURE_SCHEMA_PATH = CANDIDATE_ROOT / "apt_closure_capture.schema.json"
APT_PROPOSAL_SCHEMA_PATH = CANDIDATE_ROOT / "apt_closure_proposal.schema.json"
APT_CAPTURE_PLANNER_PATH = ROOT / "scripts/plan_glim_clean_room_r3_apt_capture.py"
APT_OUTER_TOOL_PATH = CANDIDATE_ROOT / "apt_closure_outer.py"
APT_OUTER_SCHEMA_PATH = CANDIDATE_ROOT / "apt_closure_outer.schema.json"
DOCKERFILE_PATH = CANDIDATE_ROOT / "Dockerfile"
ENTRYPOINT_PATH = CANDIDATE_ROOT / "entrypoint.sh"
COLLECTOR_PATH = CANDIDATE_ROOT / "collect_phase3d.py"
PHASE1_CLOSURE = ROOT / (
    "docker/benchmark_adapters/glim_clean_room/phase1/source_closure.json")
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
CONTAINER_SHA_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
FORBIDDEN_BRIDGE_RE = re.compile(r"glim[_-]?ros2", re.IGNORECASE)
FORBIDDEN_DATA_RE = re.compile(
    r"(?:ground[_-]?truth|scorer|/gt(?:/|$)|gt_content)", re.IGNORECASE)
MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_OUTPUT_FILE_BYTES = 512 * 1024 * 1024
SYSTEM_NAME = "glim_clean_room_phase3d_r3"
RECEIPT_KIND = "glim_clean_room_phase3d_r3_attempt_v1"


class CandidateError(ValueError):
    """Raised when the opt-in candidate cannot be proven safe to run."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path, *, max_bytes: int = MAX_OUTPUT_FILE_BYTES) -> str:
    _regular(path, "file", max_bytes=max_bytes)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    if path.is_symlink() or not path.is_dir():
        raise CandidateError(f"tree is not a regular directory: {path}")
    files = sorted(path.rglob("*"))
    regular_files: list[Path] = []
    seen_inodes: set[tuple[int, int]] = set()
    for item in files:
        if item.is_symlink():
            raise CandidateError(f"tree contains a symlink: {item}")
        if item.is_dir():
            continue
        _regular(item, "tree file", max_bytes=MAX_OUTPUT_FILE_BYTES)
        stat_result = item.stat()
        inode = (int(stat_result.st_dev), int(stat_result.st_ino))
        if stat_result.st_nlink != 1 or inode in seen_inodes:
            raise CandidateError(f"tree contains a hardlink or duplicate inode: {item}")
        seen_inodes.add(inode)
        regular_files.append(item)
    if not regular_files:
        raise CandidateError(f"tree is empty: {path}")
    digest = hashlib.sha256()
    for item in regular_files:
        digest.update(item.relative_to(path).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with item.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, max_bytes: int) -> None:
    if path.is_symlink() or not path.is_file():
        raise CandidateError(f"{label} is not a regular file: {path}")
    stat_result = path.stat()
    if stat_result.st_nlink != 1:
        raise CandidateError(f"{label} has unexpected link count: {path}")
    if stat_result.st_size <= 0 or stat_result.st_size > max_bytes:
        raise CandidateError(f"{label} has invalid size: {path}")


def _read_json(path: Path, *, max_bytes: int = MAX_JSON_BYTES,
               reject_forbidden: bool = True) -> dict[str, Any]:
    _regular(path, "JSON", max_bytes=max_bytes)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CandidateError(f"invalid JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise CandidateError(f"JSON root is not an object: {path}")
    if reject_forbidden:
        _reject_forbidden(value, str(path))
    return value


def _reject_forbidden(value: Any, label: str) -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            key_text = str(key).lower()
            if key_text in {"ground_truth_content_opened", "scorer_invoked"}:
                if not isinstance(item, bool):
                    raise CandidateError(f"GT-blind proof field is not boolean in {label}: {key}")
                continue
            if FORBIDDEN_DATA_RE.search(key_text):
                raise CandidateError(f"forbidden data/scorer field in {label}: {key}")
            _reject_forbidden(item, label)
    elif isinstance(value, list):
        for item in value:
            _reject_forbidden(item, label)
    elif isinstance(value, str) and FORBIDDEN_DATA_RE.search(value):
        raise CandidateError(f"forbidden data/scorer marker in {label}")


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise CandidateError(f"{label} must be lowercase SHA-256")
    return value


def _load_yaml(path: Path, *, reject_forbidden: bool = True) -> dict[str, Any]:
    _regular(path, "YAML", max_bytes=MAX_JSON_BYTES)
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise CandidateError(f"invalid YAML: {path}: {error}") from error
    if not isinstance(value, dict):
        raise CandidateError(f"YAML root is not an object: {path}")
    if reject_forbidden:
        _reject_forbidden(value, str(path))
    return value


def _select(document: Mapping[str, Any], selector: str) -> Mapping[str, Any]:
    current: Any = document
    for component in selector.split("/"):
        if not component or not isinstance(current, Mapping) or component not in current:
            raise CandidateError(f"profile selector is missing: {selector}")
        current = current[component]
    if not isinstance(current, Mapping):
        raise CandidateError(f"profile selector is not a mapping: {selector}")
    return current


def _resolve_repo_path(value: str, *, base: Path = ROOT) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate.resolve()
    if any(part in {"", ".", ".."} for part in candidate.parts):
        raise CandidateError(f"unsafe relative path: {value}")
    return (base / candidate).resolve()


def _manifest() -> dict[str, Any]:
    # The candidate manifest declares the forbidden-field vocabulary itself;
    # validate that vocabulary below, but do not reject its declaration while
    # parsing the manifest.
    manifest = _read_json(MANIFEST_PATH, reject_forbidden=False)
    if manifest.get("schema_version") != 1 or \
            manifest.get("candidate_id") != "glim-clean-room-phase3d-r3-candidate-v1" or \
            manifest.get("candidate_revision") != 1:
        raise CandidateError("candidate manifest identity drift")
    if manifest.get("status") != "OPT_IN_NOT_READY" or \
            manifest.get("benchmark_eligible") is not False:
        raise CandidateError("candidate must remain opt-in and benchmark-ineligible")
    base = manifest.get("base_image")
    if not isinstance(base, Mapping) or \
            not re.fullmatch(r"ros@sha256:[0-9a-f]{64}",
                             str(base.get("reference", ""))) or \
            base.get("distribution") != "jazzy" or \
            base.get("architecture") != "linux/amd64" or \
            base.get("pull_policy") != "never" or \
            base.get("network_during_run") != "none":
        raise CandidateError("candidate base image contract is invalid")
    closure = manifest.get("core_source_closure")
    if not isinstance(closure, Mapping):
        raise CandidateError("core source closure is missing")
    closure_path = ROOT / str(closure.get("manifest_path", ""))
    if sha256_file(closure_path) != _require_sha(
            closure.get("manifest_sha256"), "core closure manifest SHA"):
        raise CandidateError("core source closure manifest drift")
    components = closure.get("components")
    if not isinstance(components, list) or len(components) != 3:
        raise CandidateError("candidate core closure must contain exactly three components")
    phase1 = _read_json(PHASE1_CLOSURE, max_bytes=4 * 1024 * 1024)
    phase1_components = {item.get("name"): item for item in phase1.get("components", [])}
    for component in components:
        if not isinstance(component, Mapping) or component.get("name") not in phase1_components:
            raise CandidateError("candidate component set is not the Phase 1 core set")
        reference = phase1_components[component["name"]]
        for field in ("commit", "archive_sha256", "source_tree_sha256"):
            if component.get(field) != reference.get(field):
                raise CandidateError(
                    f"candidate core closure drift: {component['name']} {field}")
        licenses = reference.get("license_files") or []
        if not licenses or component.get("license_path") != licenses[0].get("path") or \
                component.get("license_sha256") != licenses[0].get("sha256"):
            raise CandidateError(
                f"candidate license closure drift: {component['name']}")
    offline = manifest.get("offline_dependency_closure")
    if not isinstance(offline, Mapping) or \
            offline.get("status") != "NOT_READY_APT_CLOSURE_UNSEALED":
        raise CandidateError("offline dependency closure must remain NOT_READY")
    for field, path in (
            ("manifest_sha256", OFFLINE_MANIFEST_PATH),
            ("schema_sha256", OFFLINE_SCHEMA_PATH)):
        if sha256_file(path) != _require_sha(offline.get(field), f"offline {field}"):
            raise CandidateError(f"offline dependency {field} drift")
    offline_document = _read_json(OFFLINE_MANIFEST_PATH, reject_forbidden=False)
    if offline_document.get("manifest_kind") != \
            "glim_clean_room_phase3d_r3_offline_dependency_manifest_v1" or \
            offline_document.get("candidate_id") != manifest.get("candidate_id") or \
            offline_document.get("status") != "NOT_READY_APT_CLOSURE_UNSEALED" or \
            offline_document.get("apt_deb_closure", {}).get("status") != \
            "NOT_READY_APT_CAPTURE_REQUIRED":
        raise CandidateError("offline dependency manifest is not the sealed NOT_READY closure")
    build_contract = manifest.get("offline_build_contract")
    if not isinstance(build_contract, Mapping) or \
            build_contract.get("build_network") != "none" or \
            build_contract.get("pull_policy") != "never" or \
            build_contract.get("jobs") != 2 or \
            build_contract.get("execution") != "NOT_RUN":
        raise CandidateError("offline build contract is invalid or executed")
    for field, path in (
            ("recipe_sha256", DOCKERFILE_PATH),
            ("prefetch_tool_sha256", PREFETCH_TOOL_PATH),
            ("planner_sha256", OFFLINE_PLANNER_PATH)):
        if sha256_file(path) != _require_sha(build_contract.get(field),
                                             f"offline {field}"):
            raise CandidateError(f"offline build {field} drift")
    capture_contract = manifest.get("apt_closure_capture_contract")
    if not isinstance(capture_contract, Mapping) or \
            capture_contract.get("status") != "REVIEW_REQUIRED" or \
            capture_contract.get("capture_mode") != "CONTRACT_ONLY" or \
            capture_contract.get("network_used") != "provisioning_and_capture_only" or \
            capture_contract.get("production_manifest_status") != \
            "NOT_READY_APT_CLOSURE_UNSEALED" or \
            capture_contract.get("composition") != "separate_unsigned_proposal_only":
        raise CandidateError("apt closure capture contract is not review-only")
    for field, path in (
            ("tool_sha256", APT_CAPTURE_TOOL_PATH),
            ("schema_sha256", APT_CAPTURE_SCHEMA_PATH),
            ("proposal_schema_sha256", APT_PROPOSAL_SCHEMA_PATH),
            ("outer_tool_sha256", APT_OUTER_TOOL_PATH),
            ("outer_schema_sha256", APT_OUTER_SCHEMA_PATH),
            ("planner_sha256", APT_CAPTURE_PLANNER_PATH)):
        if sha256_file(path) != _require_sha(capture_contract.get(field),
                                             f"apt capture {field}"):
            raise CandidateError(f"apt capture {field} drift")
    policy = manifest.get("source_policy")
    if not isinstance(policy, Mapping) or \
            policy.get("forbidden_token_parts") != ["glim_", "ros2"]:
        raise CandidateError("source contamination policy drift")
    _validate_recipe_surface()
    return manifest


def _validate_recipe_surface() -> None:
    for path in (DOCKERFILE_PATH, ENTRYPOINT_PATH, COLLECTOR_PATH, MANIFEST_PATH,
                 SCHEMA_PATH, OFFLINE_MANIFEST_PATH, OFFLINE_SCHEMA_PATH,
                 PREFETCH_TOOL_PATH, OFFLINE_PLANNER_PATH,
                 APT_CAPTURE_TOOL_PATH, APT_CAPTURE_SCHEMA_PATH,
                 APT_PROPOSAL_SCHEMA_PATH, APT_OUTER_TOOL_PATH,
                 APT_OUTER_SCHEMA_PATH, APT_CAPTURE_PLANNER_PATH):
        _regular(path, "candidate recipe", max_bytes=MAX_OUTPUT_FILE_BYTES)
        text = path.read_text(encoding="utf-8")
        if FORBIDDEN_BRIDGE_RE.search(text):
            raise CandidateError(f"bridge token in candidate recipe: {path}")
    dockerfile = DOCKERFILE_PATH.read_text(encoding="utf-8")
    for token in (
            "FROM ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f",
            "COPY prefetch/", "python3 /opt/dependency_prefetch.py verify",
            "sha256sum -c -", "-DBUILD_WITH_MARCH_NATIVE=OFF", "BUILD_TESTING=OFF"):
        if token not in dockerfile:
            raise CandidateError(f"candidate Docker recipe misses: {token}")
    for forbidden in ("ADD https://", "apt-get", "curl ", "wget ",
                      "--privileged", "--network host", "glim_ros2", "latest"):
        if forbidden in dockerfile.lower():
            raise CandidateError(f"unsafe Docker recipe token: {forbidden}")


def validate_frozen_bindings(
        *, manifest: Mapping[str, Any], profile_path: Path, selection_path: Path,
        input_root: Path, calibration_root: Path, config_root: Path,
        input_manifest_path: Path, sequence_id: str, run_index: int) -> dict[str, Any]:
    """Reopen all non-GT identities required before a candidate run."""
    frozen = manifest["frozen_r2_binding"]
    expected_profile = ROOT / frozen["profile_path"]
    expected_selection = ROOT / frozen["selection_path"]
    if profile_path.resolve() != expected_profile.resolve() or \
            selection_path.resolve() != expected_selection.resolve():
        raise CandidateError("candidate may not substitute the retained r2 profile/selection")
    # The profile document may contain GT metadata for the separate scorer.
    # Reopen only the selected non-GT input/config mapping below; never walk or
    # expose unrelated profile fields to this transport runner.
    profile_document = _load_yaml(profile_path, reject_forbidden=False)
    profile_sha = canonical_profile_sha256(profile_document)
    if profile_sha != frozen["profile_canonical_sha256"]:
        raise CandidateError("frozen profile canonical SHA mismatch")
    profile_root = profile_document.get("competitive_slam_profile")
    if not isinstance(profile_root, Mapping) or \
            profile_root.get("profile_revision_id") != frozen["profile_revision_id"]:
        raise CandidateError("profile revision mismatch")
    selected = _select(profile_document, frozen["profile_selector"])
    input_decl = selected.get("input")
    config_decl = selected.get("config")
    if not isinstance(input_decl, Mapping) or not isinstance(config_decl, Mapping):
        raise CandidateError("selected profile lacks frozen input/config declarations")
    _reject_forbidden(input_decl, "selected profile input")
    _reject_forbidden(config_decl, "selected profile config")
    expected_input = _resolve_repo_path(str(input_decl.get("path")))
    expected_config = _resolve_repo_path(str(config_decl.get("path")))
    if input_root.resolve() != expected_input or config_root.resolve() != expected_config:
        raise CandidateError("input/config path differs from selected frozen profile")
    if sha256_tree(input_root) != _require_sha(
            input_decl.get("tree_sha256"), "profile input tree SHA"):
        raise CandidateError("frozen input tree SHA mismatch")
    if sha256_tree(config_root) != _require_sha(
            config_decl.get("tree_sha256"), "profile config tree SHA"):
        raise CandidateError("frozen config tree SHA mismatch")
    if calibration_root.is_symlink() or not calibration_root.is_dir():
        raise CandidateError("calibration root is not a regular directory")
    calibration_sha = sha256_tree(calibration_root)
    input_document = _read_json(input_manifest_path)
    required = manifest["input_contract"]["required_fields"]
    for field in required:
        if field not in input_document:
            raise CandidateError(f"input manifest field is missing: {field}")
    if input_document.get("manifest_kind") != manifest["input_contract"]["manifest_kind"]:
        raise CandidateError("input manifest kind mismatch")
    if input_document.get("profile_path") != frozen["profile_path"] or \
            input_document.get("profile_selector") != frozen["profile_selector"]:
        raise CandidateError("input manifest profile binding mismatch")
    if input_document.get("canonical_rosbag2_tree_sha256") != \
            input_decl.get("tree_sha256") or \
            input_document.get("config_tree_sha256") != config_decl.get("tree_sha256"):
        raise CandidateError("input manifest input/config identity mismatch")
    if input_document.get("calibration_tree_sha256") != calibration_sha or \
            Path(str(input_document.get("calibration_root"))).resolve() != \
            calibration_root.resolve():
        raise CandidateError("input manifest calibration identity mismatch")
    if input_document.get("sequence_id") != sequence_id or \
            input_document.get("run_index") != run_index:
        raise CandidateError("input manifest schedule mismatch")
    for field in ("lidar_topic", "imu_topic"):
        if not isinstance(input_document.get(field), str) or \
                not input_document[field].startswith("/"):
            raise CandidateError(f"input manifest topic is invalid: {field}")
    selection = _load_yaml(selection_path, reject_forbidden=False)
    if selection.get("selection_id") != frozen["selection_id"] or \
            selection.get("profile_path") != frozen["profile_path"] or \
            selection.get("profile_revision_id") != frozen["profile_revision_id"] or \
            selection.get("closure_id") != frozen["closure_id"] or \
            selection.get("closure_revision") != frozen["closure_revision"] or \
            selection.get("closure_identity_sha256") != frozen["closure_identity_sha256"]:
        raise CandidateError("retained r2 selection identity mismatch")
    if sha256_file(selection_path) != frozen["selection_file_sha256"]:
        raise CandidateError("retained r2 selection file SHA mismatch")
    return {
        "profile_sha256": profile_sha,
        "selection_file_sha256": sha256_file(selection_path),
        "input_tree_sha256": input_document["canonical_rosbag2_tree_sha256"],
        "config_tree_sha256": input_document["config_tree_sha256"],
        "calibration_tree_sha256": calibration_sha,
        "input_manifest_sha256": sha256_file(input_manifest_path),
        "sequence_id": sequence_id,
        "run_index": run_index,
        "lidar_topic": input_document["lidar_topic"],
        "imu_topic": input_document["imu_topic"],
    }


def campaign_id_for(identity: Mapping[str, Any]) -> str:
    payload = {
        "candidate_id": "glim-clean-room-phase3d-r3-candidate-v1",
        "profile_sha256": identity["profile_sha256"],
        "selection_file_sha256": identity["selection_file_sha256"],
        "sequence_id": identity["sequence_id"],
        "run_index": identity["run_index"],
    }
    return sha256_bytes(json.dumps(payload, sort_keys=True,
                                   separators=(",", ":")).encode("utf-8"))


def build_container_argv(
        *, manifest: Mapping[str, Any], input_root: Path,
        calibration_root: Path, config_root: Path, output_root: Path,
        identity: Mapping[str, Any]) -> tuple[list[str], list[dict[str, Any]]]:
    image = manifest["base_image"]["reference"]
    campaign = campaign_id_for(identity)
    name = (
        f"phase3d-r3-{campaign[:16]}-{int(identity['run_index']):03d}")
    mounts = [
        {"source": str(input_root.resolve()),
         "destination": "/phase3d/input", "read_only": True},
        {"source": str(calibration_root.resolve()),
         "destination": "/phase3d/calibration", "read_only": True},
        {"source": str(config_root.resolve()),
         "destination": "/phase3d/config", "read_only": True},
        {"source": str(output_root.resolve()),
         "destination": "/phase3d/output", "read_only": False},
    ]
    argv = [
        "docker", "run", "--rm", "--init", "--network", "none", "--read-only",
        "--cap-drop=ALL", "--security-opt", "no-new-privileges", "--name", name,
    ]
    for mount in mounts:
        mode = "ro" if mount["read_only"] else "rw"
        argv += [
            "--mount",
            f"type=bind,src={mount['source']},"
            f"dst={mount['destination']},{mode}",
        ]
    argv += [
        "-e", "PHASE3D_INPUT_ROOT=/phase3d/input",
        "-e", "PHASE3D_CALIBRATION_ROOT=/phase3d/calibration",
        "-e", "PHASE3D_CONFIG_ROOT=/phase3d/config",
        "-e", "PHASE3D_OUTPUT_ROOT=/phase3d/output",
        "-e", f"PHASE3D_LIDAR_TOPIC={identity['lidar_topic']}",
        "-e", f"PHASE3D_IMU_TOPIC={identity['imu_topic']}",
        image, "/opt/phase3d-r3/entrypoint.sh",
    ]
    return argv, mounts


def _output_tree_hash(path: Path) -> str:
    digest = hashlib.sha256()
    names = {"trajectory.json", "map.json", "resource.json", "failure.json"}
    for item in sorted(path.iterdir(), key=lambda candidate: candidate.name):
        if item.name not in names:
            continue
        digest.update(item.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(item)))
    return digest.hexdigest()


def _validate_success_artifacts(path: Path) -> dict[str, str]:
    expected = {"trajectory.json", "map.json", "resource.json"}
    names = {item.name for item in path.iterdir()}
    if names != expected:
        raise CandidateError(f"success output set is not exact: {sorted(names)}")
    trajectory = _read_json(path / "trajectory.json")
    if trajectory.get("kind") != "glim_clean_room_phase3d_trajectory_v1" or \
            not isinstance(trajectory.get("samples"), list) or not trajectory["samples"]:
        raise CandidateError("trajectory artifact contract is invalid")
    previous: tuple[int, int] | None = None
    frame = trajectory.get("frame_id")
    for index, sample in enumerate(trajectory["samples"]):
        if sample.get("order") != index or sample.get("frame_id") != frame:
            raise CandidateError("trajectory order/frame contract is invalid")
        stamp = sample.get("stamp")
        if not isinstance(stamp, Mapping) or \
                isinstance(stamp.get("sec"), bool) or not isinstance(stamp.get("sec"), int) or \
                isinstance(stamp.get("nanosec"), bool) or not isinstance(stamp.get("nanosec"), int):
            raise CandidateError("trajectory stamp is invalid")
        key = (stamp["sec"], stamp["nanosec"])
        if previous is not None and key <= previous:
            raise CandidateError("trajectory stamps are not monotonic")
        previous = key
        for group in ("position", "orientation"):
            values = sample.get(group)
            if not isinstance(values, Mapping) or any(
                    isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(float(value)) for value in values.values()):
                raise CandidateError("trajectory pose is not finite")
    mapping = _read_json(path / "map.json")
    if mapping.get("kind") != "glim_clean_room_phase3d_map_v1" or \
            mapping.get("frame_id") != frame:
        raise CandidateError("map frame/kind contract is invalid")
    try:
        payload = base64.b64decode(mapping["data_base64"], validate=True)
    except (KeyError, ValueError) as error:
        raise CandidateError("map payload is not canonical base64") from error
    if not payload or int(mapping.get("row_step", 0)) * \
            int(mapping.get("height", 0)) != len(payload):
        raise CandidateError("map payload size contract is invalid")
    resource_doc = _read_json(path / "resource.json")
    if resource_doc.get("kind") != "glim_clean_room_phase3d_resource_v1" or \
            resource_doc.get("network_used") is not False or \
            not isinstance(resource_doc.get("peak_rss_bytes"), int) or \
            resource_doc["peak_rss_bytes"] <= 0:
        raise CandidateError("resource artifact contract is invalid")
    return {name: sha256_file(path / name) for name in expected}


def _validate_failure_artifact(path: Path) -> dict[str, str]:
    names = {item.name for item in path.iterdir()}
    if names != {"failure.json"}:
        raise CandidateError("failure output must contain exactly failure.json")
    failure = _read_json(path / "failure.json")
    if failure.get("kind") != "glim_clean_room_phase3d_failure_v1" or \
            failure.get("terminal") is not True:
        raise CandidateError("failure artifact is not terminal")
    proof = failure.get("gt_blind")
    if not isinstance(proof, Mapping) or proof.get("ground_truth_content_opened") is not False or \
            proof.get("scorer_invoked") is not False:
        raise CandidateError("failure artifact is not GT-blind")
    return {"failure.json": sha256_file(path / "failure.json")}


def seal_attempt(
        *, output_root: Path, manifest: Mapping[str, Any], identity: Mapping[str, Any],
        argv: list[str], mounts: list[dict[str, Any]], exit_status: int,
        timed_out: bool) -> dict[str, Any]:
    artifacts = (_validate_success_artifacts(output_root)
                 if exit_status == 0 else _validate_failure_artifact(output_root))
    campaign_id = campaign_id_for(identity)
    base: dict[str, Any] = {
        "receipt_kind": RECEIPT_KIND,
        "receipt_hash_kind": "canonical_execution_attempt_receipt_sha256_v1",
        "campaign_id": campaign_id,
        "schedule": {
            "system": SYSTEM_NAME,
            "sequence": identity["sequence_id"],
            "repetition": identity["run_index"],
        },
        "identity": {
            "profile_canonical_sha256": identity["profile_sha256"],
            "selection_receipt_file_sha256": identity["selection_file_sha256"],
            "image_digest": manifest["base_image"]["reference"],
            # The actual pretty-file SHA is sealed in attempt.index.json after
            # this canonical receipt is written.  Keep this required legacy
            # field nonzero and bind the authoritative file identity through
            # the external index, avoiding a self-hash cycle.
            "execution_receipt_file_sha256": sha256_file(MANIFEST_PATH),
            "candidate_manifest_sha256": sha256_file(MANIFEST_PATH),
            "candidate_source_tree_sha256": sha256_tree(CANDIDATE_ROOT),
            "input_tree_sha256": identity["input_tree_sha256"],
            "config_tree_sha256": identity["config_tree_sha256"],
            "calibration_tree_sha256": identity["calibration_tree_sha256"],
            "input_manifest_sha256": identity["input_manifest_sha256"],
            "offline_dependency_manifest_sha256": manifest[
                "offline_dependency_closure"]["manifest_sha256"],
            "offline_recipe_sha256": manifest["offline_build_contract"][
                "recipe_sha256"],
            "offline_prefetch_tool_sha256": manifest["offline_build_contract"][
                "prefetch_tool_sha256"],
            "offline_planner_sha256": manifest["offline_build_contract"][
                "planner_sha256"],
            "resource_measurement_revision": "phase3d-r3-resource-v1",
        },
        "argv": argv,
        "mounts": mounts,
        "execution": {
            "exit_status": int(exit_status),
            "timed_out": bool(timed_out),
            "network": "none",
        },
        "completion": {
            "complete": exit_status == 0 and not timed_out,
            "trajectory": exit_status == 0 and not timed_out,
            "map": exit_status == 0 and not timed_out,
            "failure_terminal": exit_status != 0 or timed_out,
        },
        "artifact_hashes": artifacts,
        "output_tree_sha256": _output_tree_hash(output_root),
        "gt_blind_proof": {
            "ground_truth_content_opened": False,
            "scorer_invoked": False,
            "collector_role": "transport_only",
        },
    }
    receipt = seal_receipt(base)
    validate_receipt(receipt, require_success=False)
    receipt_path = output_root / "attempt.json"
    index_path = output_root / "attempt.index.json"
    if receipt_path.exists() or index_path.exists():
        raise CandidateError("attempt receipt already exists")
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    receipt_file_sha = sha256_file(receipt_path)
    index = {
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_r3_attempt_index_v1",
        "receipt_sha256": receipt["execution_receipt_sha256"],
        "receipt_file_sha256": receipt_file_sha,
        "output_tree_sha256": receipt["output_tree_sha256"],
        "candidate_id": manifest["candidate_id"],
    }
    index_path.write_text(json.dumps(index, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
    return receipt


def run_candidate(args: argparse.Namespace) -> dict[str, Any]:
    manifest = _manifest()
    profile_path = Path(args.profile).resolve()
    selection_path = Path(args.selection).resolve()
    input_root = Path(args.input_root).resolve()
    calibration_root = Path(args.calibration_root).resolve()
    config_root = Path(args.config_root).resolve()
    input_manifest_path = Path(args.input_manifest).resolve()
    if args.run_index < 1 or args.run_index > 1000:
        raise CandidateError("run index is outside the bounded range")
    identity = validate_frozen_bindings(
        manifest=manifest, profile_path=profile_path, selection_path=selection_path,
        input_root=input_root, calibration_root=calibration_root,
        config_root=config_root, input_manifest_path=input_manifest_path,
        sequence_id=args.sequence_id, run_index=args.run_index)
    argv, mounts = build_container_argv(
        manifest=manifest, input_root=input_root, calibration_root=calibration_root,
        config_root=config_root, output_root=Path(args.output).resolve(),
        identity=identity)
    if not args.execute:
        return {
            "status": "DRY_PLAN",
            "candidate_id": manifest["candidate_id"],
            "benchmark_eligible": False,
            "profile_sha256": identity["profile_sha256"],
            "selection_sha256": identity["selection_file_sha256"],
            "campaign_id": campaign_id_for(identity),
            "argv": argv,
            "mounts": mounts,
            "execution": "NOT_RUN",
        }
    output_root = Path(args.output).resolve()
    if output_root.exists() or output_root.is_symlink():
        raise CandidateError("output root must not pre-exist")
    output_root.mkdir(parents=True)
    timed_out = False
    exit_status = 1
    try:
        result = subprocess.run(argv, check=False, timeout=1800,
                                capture_output=True, text=False)
        exit_status = int(result.returncode)
        stdout_path = output_root.with_name(output_root.name + ".stdout.log")
        stderr_path = output_root.with_name(output_root.name + ".stderr.log")
        if stdout_path.exists() or stderr_path.exists():
            raise CandidateError("runner log path already exists")
        stdout_path.write_bytes(result.stdout or b"")
        stderr_path.write_bytes(result.stderr or b"")
    except subprocess.TimeoutExpired as error:
        timed_out = True
        exit_status = 124
        output_root.joinpath("failure.json").write_text(json.dumps({
            "schema_version": 1,
            "kind": "glim_clean_room_phase3d_failure_v1",
            "terminal": True,
            "stage": "host_timeout",
            "error_type": type(error).__name__,
            "error": "candidate container exceeded bounded timeout",
            "gt_blind": {"ground_truth_content_opened": False, "scorer_invoked": False},
        }, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    receipt = seal_attempt(
        output_root=output_root, manifest=manifest, identity=identity,
        argv=argv, mounts=mounts, exit_status=exit_status, timed_out=timed_out)
    return {
        "status": "PASS" if exit_status == 0 and not timed_out else "FAIL_CLOSED",
        "benchmark_eligible": False,
        "candidate_id": manifest["candidate_id"],
        "campaign_id": receipt["campaign_id"],
        "execution_receipt_sha256": receipt["execution_receipt_sha256"],
        "output_tree_sha256": receipt["output_tree_sha256"],
        "exit_status": exit_status,
        "timed_out": timed_out,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path,
        default=ROOT / (
            "configs/slam_benchmark_profiles/competitive_slam_v1.yaml"))
    parser.add_argument("--selection", type=Path,
        default=ROOT / (
            "configs/slam_benchmark_profiles/"
            "competitive_execution_selection_2026-08-r2.yaml"))
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--calibration-root", type=Path, required=True)
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--input-manifest", type=Path, required=True)
    parser.add_argument("--sequence-id", required=True)
    parser.add_argument("--run-index", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true",
                        help="run the candidate container; omitted means dry plan")
    parser.add_argument("--json", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        result = run_candidate(args)
    except (CandidateError, OSError, ValueError, yaml.YAMLError) as error:
        result = {
            "status": "FAIL_CLOSED",
            "candidate_id": "glim-clean-room-phase3d-r3-candidate-v1",
            "benchmark_eligible": False,
            "reason": str(error),
        }
        if args.json:
            print(json.dumps(result, sort_keys=True))
        else:
            print("Phase 3d r3 candidate: FAIL_CLOSED: " + str(error), file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        print("Phase 3d r3 candidate: " + result["status"])
        print("Benchmark eligibility: false")
        print("Execution: " + str(result.get("execution", result.get("exit_status"))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
