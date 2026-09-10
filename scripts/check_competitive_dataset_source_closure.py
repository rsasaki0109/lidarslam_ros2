#!/usr/bin/env python3
"""Audit the immutable source contract for every competitive dataset entry.

The checker is intentionally metadata-only.  It never opens a bag, calibration
tree, ground-truth file, trajectory, or scorer artifact.  A hash recorded while
the evidence volume is unavailable is therefore ``RECORDED_ONLY`` and cannot
be upgraded to a claim-eligible revalidation by this tool.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping
from urllib.parse import urlparse

import yaml


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / "configs/slam_benchmark_profiles/competitive_slam_v1.yaml"
SCHEMA_VERSION = 1
HASH_KIND = "canonical_dataset_source_closure_sha256_v1"
RECORDED_ONLY = "RECORDED_ONLY"
REVALIDATED = "REVALIDATED"
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
IMMUTABLE_VERSION_RE = re.compile(r"^(?:[0-9a-fA-F]{40}|v?[0-9]+(?:\.[0-9]+){1,3})$")
REQUIRED_PARTITIONS = ("bringup", "development", "regression_only", "historical", "fresh")
DEFAULT_REQUIRED_SYSTEMS = ("ours", "glim", "fast_livo2")
_HASH_FIELDS = ("sha256", "input_manifest_sha256", "ground_truth_sha256",
                "calibration_archive_sha256", "bag_sha256",
                "raw_rosbag1_sha256")


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_dataset_source_closure_sha256(policy: Mapping[str, Any]) -> str:
    """Hash the declared dataset-source policy deterministically."""
    return hashlib.sha256(_canonical_json(dict(policy))).hexdigest()


def _profile_document(profile: Mapping[str, Any]) -> Mapping[str, Any]:
    value = profile.get("competitive_slam_profile", profile)
    return value if isinstance(value, Mapping) else {}


def _policy_from_profile(profile: Mapping[str, Any]) -> Mapping[str, Any] | None:
    contract = _profile_document(profile)
    gate = contract.get("evidence_gate_v2")
    if not isinstance(gate, Mapping):
        return None
    policy = gate.get("dataset_source_closure")
    return policy if isinstance(policy, Mapping) else None


def _sha(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _mapping(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _positive_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(float(value)) and float(value) > 0.0)


def _path_like(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    return "/" in value or "\\" in value or "://" in value or value.startswith((".", "~"))


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _check_file_binding(
        configured: Mapping[str, Any], path_key: str, sha_key: str,
        root: Path, not_ready: list[str], errors: list[str]) -> bool:
    if path_key not in configured and sha_key not in configured:
        return True
    path_value = configured.get(path_key)
    expected = configured.get(sha_key)
    if not _text(path_value) or not _sha(expected):
        _record_missing(not_ready, f"dataset source closure {path_key}/{sha_key} is missing")
        return False
    path = Path(path_value)
    if path.is_absolute() or ".." in path.parts:
        errors.append(f"dataset source closure {path_key} must be repository-relative")
        return False
    resolved = (root / path).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError:
        errors.append(f"dataset source closure {path_key} resolves outside repository")
        return False
    if not resolved.is_file() or resolved.is_symlink():
        _record_missing(not_ready, f"dataset source closure {path_key} is not a regular file")
        return False
    if _file_sha256(resolved) != expected.lower():
        errors.append(f"dataset source closure {path_key} SHA-256 does not match bytes")
        return False
    return True


def _check_ntu_viral_selection_manifest(
        configured: Mapping[str, Any], root: Path,
        profile_entries: Mapping[tuple[str, str], Mapping[str, Any]],
        policy_entries: Mapping[tuple[str, str], Mapping[str, Any]],
        not_ready: list[str], errors: list[str]) -> bool:
    """Validate the NTU preregistration without opening any dataset bytes.

    The selection file is deliberately a metadata-only receipt.  It is allowed
    to describe official moving download references, but it may not smuggle a
    guessed byte identity into the claim path or treat the development
    ``tnp_01`` replay as a fresh/evaluation sequence.
    """
    path_value = configured.get("selection_path")
    if not _text(path_value):
        return True
    path = (root / Path(path_value)).resolve()
    if not path.is_file() or path.is_symlink():
        return False
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        errors.append(f"dataset selection manifest cannot be read: {exc}")
        return False
    if not isinstance(document, Mapping):
        errors.append("dataset selection manifest must be a mapping")
        return False
    valid = True
    expected_id = configured.get("selection_id")
    if expected_id is not None and document.get("selection_id") != expected_id:
        errors.append("dataset selection manifest selection_id does not match policy")
        valid = False
    if document.get("status") != "NOT_READY":
        _record_missing(not_ready, "NTU VIRAL selection must remain NOT_READY until byte revalidation")
        valid = False
    if document.get("family_id") != "ntu_viral":
        errors.append("NTU VIRAL selection family_id must be ntu_viral")
        valid = False
    if document.get("partition") != "historical":
        errors.append("NTU VIRAL selection must be a historical preregistration")
        valid = False

    expected_selected: dict[str, str] = {}
    for key, descriptor in policy_entries.items():
        if key[0] != "historical" or descriptor.get("family_id") != "ntu_viral":
            continue
        if descriptor.get("evaluation_eligible", True) is True:
            expected_selected[str(descriptor.get("sequence_id"))] = key[1]
    selected = document.get("selected_sequences")
    if not isinstance(selected, list) or not selected:
        _record_missing(not_ready, "NTU VIRAL selected_sequences are missing")
        valid = False
        selected = []
    observed_selected: dict[str, Mapping[str, Any]] = {}
    for item in selected:
        if not isinstance(item, Mapping):
            errors.append("NTU VIRAL selected sequence entry is malformed")
            valid = False
            continue
        sequence_id = item.get("sequence_id")
        profile_key = item.get("profile_key")
        if not _text(sequence_id) or not _text(profile_key):
            _record_missing(not_ready, "NTU VIRAL selected sequence identity is missing")
            valid = False
            continue
        if sequence_id in observed_selected:
            errors.append(f"NTU VIRAL selected sequence is duplicated: {sequence_id}")
            valid = False
        observed_selected[str(sequence_id)] = item
        if expected_selected.get(str(sequence_id)) != profile_key:
            errors.append(f"NTU VIRAL selected sequence is not bound to its profile: {sequence_id}")
            valid = False
        if item.get("evaluation_eligible") is not True or item.get("fresh_eligible") is not False:
            errors.append(
                f"NTU VIRAL selected sequence {sequence_id} has an invalid evaluation/fresh eligibility")
            valid = False
        for field in ("selection_rationale", "environment", "expected_files",
                      "official_duration_seconds", "byte_identity"):
            value = item.get(field)
            if field == "expected_files":
                if not isinstance(value, list) or not value or any(not _text(v) for v in value):
                    _record_missing(not_ready, f"NTU VIRAL {sequence_id} expected_files are missing")
                    valid = False
            elif field == "byte_identity":
                if not isinstance(value, Mapping):
                    _record_missing(not_ready, f"NTU VIRAL {sequence_id} byte identity is missing")
                    valid = False
                else:
                    required_byte_fields = (
                        "input_sha256", "input_size_bytes", "ground_truth_sha256",
                        "ground_truth_size_bytes", "calibration_sha256",
                        "calibration_size_bytes")
                    for byte_field in required_byte_fields:
                        if byte_field not in value or value.get(byte_field) is not None:
                            errors.append(
                                f"NTU VIRAL {sequence_id}.{byte_field} must remain null before acquisition")
                            valid = False
                    if value.get("validation_status") != "NOT_READY":
                        _record_missing(
                            not_ready,
                            f"NTU VIRAL {sequence_id} byte identity is not marked NOT_READY")
                        valid = False
            elif field == "official_duration_seconds":
                if not _positive_number(value):
                    _record_missing(not_ready, f"NTU VIRAL {sequence_id} official duration is missing")
                    valid = False
            elif not _text(value):
                _record_missing(not_ready, f"NTU VIRAL {sequence_id}.{field} is missing")
                valid = False
        systems = item.get("supported_systems")
        required_systems = configured.get("required_systems", list(DEFAULT_REQUIRED_SYSTEMS))
        if (not isinstance(systems, list) or
                set(systems) != set(required_systems)):
            errors.append(f"NTU VIRAL {sequence_id} does not bind every required runner")
            valid = False
    if set(observed_selected) != set(expected_selected):
        errors.append("NTU VIRAL selected sequence set does not match profile/policy")
        valid = False

    excluded = document.get("excluded_sequences")
    found_tnp = False
    if not isinstance(excluded, list):
        _record_missing(not_ready, "NTU VIRAL excluded_sequences are missing")
        valid = False
        excluded = []
    for item in excluded:
        if not isinstance(item, Mapping):
            errors.append("NTU VIRAL excluded sequence entry is malformed")
            valid = False
            continue
        if item.get("sequence_id") == "tnp_01":
            found_tnp = True
            if item.get("evaluation_eligible") is not False or item.get("fresh_eligible") is not False:
                errors.append("NTU VIRAL tnp_01 must be excluded from evaluation and fresh partitions")
                valid = False
            if not _text(item.get("reason")) or "expos" not in item.get("reason", "").lower():
                _record_missing(not_ready, "NTU VIRAL tnp_01 exposure exclusion reason is missing")
                valid = False
    if not found_tnp:
        errors.append("NTU VIRAL tnp_01 training/development exclusion is missing")
        valid = False
    if "tnp_01" not in set(document.get("disjointness", {}).get(
            "excluded_training_sequences", [])):
        errors.append("NTU VIRAL disjointness must exclude tnp_01")
        valid = False
    if "hilti_slam_challenge_2022" not in set(document.get("disjointness", {}).get(
            "excluded_historical_families", [])):
        errors.append("NTU VIRAL disjointness must exclude the Hilti family")
        valid = False
    tnp_policy_entries = [
        descriptor for key, descriptor in policy_entries.items()
        if key[0] == "development" and descriptor.get("sequence_id") == "tnp_01"
    ]
    if (not tnp_policy_entries or any(
            descriptor.get("evaluation_eligible") is not False or
            descriptor.get("fresh_eligible") is not False
            for descriptor in tnp_policy_entries)):
        errors.append("NTU VIRAL policy must keep tnp_01 evaluation/fresh excluded")
        valid = False

    acquisition = document.get("acquisition")
    if not isinstance(acquisition, Mapping):
        _record_missing(not_ready, "NTU VIRAL acquisition recipe is missing")
        valid = False
    else:
        for field in ("recipe_id", "recipe_revision", "materialization", "status",
                      "required_before_execution"):
            if not _text(acquisition.get(field)) if field != "required_before_execution" else not isinstance(acquisition.get(field), list):
                _record_missing(not_ready, f"NTU VIRAL acquisition.{field} is missing")
                valid = False
        if acquisition.get("status") != "NOT_READY":
            _record_missing(not_ready, "NTU VIRAL acquisition recipe is not NOT_READY")
            valid = False
    source = document.get("official_sources")
    if not isinstance(source, Mapping):
        _record_missing(not_ready, "NTU VIRAL official source citations are missing")
        valid = False
    else:
        for field in ("project_url", "dataset_repository_url", "ground_truth_repository_url",
                      "evaluation_tutorial_url", "citation", "terms_url"):
            if not _text(source.get(field)):
                _record_missing(not_ready, f"NTU VIRAL official source {field} is missing")
                valid = False
    disjoint = document.get("disjointness")
    if not isinstance(disjoint, Mapping):
        _record_missing(not_ready, "NTU VIRAL disjointness declaration is missing")
        valid = False
    else:
        for field in ("excluded_training_sequences", "excluded_historical_families",
                      "required_immutable_ids"):
            if not isinstance(disjoint.get(field), list) or not disjoint.get(field):
                _record_missing(not_ready, f"NTU VIRAL disjointness.{field} is missing")
                valid = False
    return valid


def _check_ntu_viral_reviewed_pin(
        configured: Mapping[str, Any], root: Path,
        profile_entries: Mapping[tuple[str, str], Mapping[str, Any]],
        policy_entries: Mapping[tuple[str, str], Mapping[str, Any]],
        not_ready: list[str], errors: list[str]) -> bool:
    """Require a separately reviewed immutable pin before NTU can be used.

    Stage A candidate receipts and Stage B proposals are intentionally not
    accepted here.  This checker remains metadata-only: it verifies the
    reviewed manifest bytes and role identities, but never opens GT/input
    content and never treats candidate hashes as profile byte revalidation.
    """
    forbidden = ("candidate_receipt_path", "candidate_receipt_sha256",
                 "candidate_receipt_status", "candidate_manifest_path")
    if any(key in configured for key in forbidden):
        errors.append("dataset source closure cannot bind a candidate receipt; only a reviewed profile pin is accepted")
        return False
    # Older synthetic policy fixtures without an NTU entry predate this
    # acquisition contract. Keep them useful for generic closure tests, while
    # any policy that actually declares NTU must opt into the reviewed-pin
    # requirement explicitly and therefore fail closed when it is omitted.
    if "reviewed_pin_required" not in configured:
        has_ntu = any(
            descriptor.get("family_id") == "ntu_viral"
            for descriptor in policy_entries.values())
        if not has_ntu:
            return True
        _record_missing(not_ready, "reviewed NTU VIRAL profile pin requirement is missing")
        return False
    if configured.get("reviewed_pin_required") is not True:
        _record_missing(not_ready, "reviewed NTU VIRAL profile pin requirement is not enabled")
        return False
    status = configured.get("reviewed_pin_manifest_status")
    path_value = configured.get("reviewed_pin_manifest_path")
    expected_sha = configured.get("reviewed_pin_manifest_sha256")
    if status != "REVIEWED" or not _text(path_value) or not _sha(expected_sha):
        _record_missing(not_ready, "reviewed NTU VIRAL profile pin manifest is missing or not REVIEWED")
        return False
    if not _check_file_binding(configured, "reviewed_pin_manifest_path",
                               "reviewed_pin_manifest_sha256", root,
                               not_ready, errors):
        return False
    path = (root / Path(str(path_value))).resolve()
    sidecar = path.with_name(path.name + ".sha256")
    if not sidecar.is_file() or sidecar.is_symlink():
        _record_missing(not_ready, "reviewed NTU VIRAL profile pin sidecar is missing")
        return False
    try:
        sidecar_fields = sidecar.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        errors.append(f"reviewed NTU VIRAL profile pin sidecar cannot be read: {exc}")
        return False
    if len(sidecar_fields) != 2 or sidecar_fields[1] != path.name or sidecar_fields[0].lower() != str(expected_sha).lower():
        errors.append("reviewed NTU VIRAL profile pin sidecar does not match the manifest")
        return False
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f"reviewed NTU VIRAL profile pin cannot be read: {exc}")
        return False
    if not isinstance(document, Mapping):
        errors.append("reviewed NTU VIRAL profile pin must be a mapping")
        return False
    valid = True
    if document.get("manifest_kind") != "ntu_viral_immutable_pin_manifest":
        errors.append("reviewed NTU VIRAL profile pin manifest kind is invalid")
        valid = False
    if document.get("status") != "REVIEWED" or document.get("claim_eligible") is not True:
        _record_missing(not_ready, "reviewed NTU VIRAL pin is not claim eligible")
        valid = False
    expected_id = configured.get("selection_id")
    if document.get("selection_id") != expected_id:
        errors.append("reviewed NTU VIRAL profile pin selection_id does not match policy")
        valid = False
    if document.get("selection_sha256") != configured.get("selection_sha256"):
        errors.append("reviewed NTU VIRAL profile pin selection SHA-256 does not match policy")
        valid = False
    if document.get("runner_gt_paths_exposed") is not False:
        errors.append("reviewed NTU VIRAL profile pin exposes ground truth to runners")
        valid = False
    runner = document.get("runner_input_manifest")
    if (not isinstance(runner, Mapping) or runner.get("ground_truth_paths_exposed") is not False or
            runner.get("ground_truth_relative_paths") != []):
        errors.append("reviewed NTU VIRAL profile pin runner manifest violates GT separation")
        valid = False
    for key in ("input_artifacts", "ground_truth_artifacts", "calibration_artifacts"):
        value = document.get(key)
        if not isinstance(value, list) or not value:
            _record_missing(not_ready, f"reviewed NTU VIRAL profile pin {key} are missing")
            valid = False
    roles = document.get("profile_diff")
    if (not isinstance(roles, Mapping) or roles.get("selection_id") != expected_id or
            roles.get("required_review_status") != "REVIEWED" or
            roles.get("required_profile_byte_identity_status") != "REVALIDATED"):
        _record_missing(not_ready, "reviewed NTU VIRAL profile pin diff is incomplete")
        valid = False
    observed_sequences = {
        str(item.get("sequence_id")) for item in roles.get("sequence_roles", [])
    } if isinstance(roles, Mapping) and isinstance(roles.get("sequence_roles"), list) else set()
    expected_sequences = {
        str(descriptor.get("sequence_id")) for key, descriptor in policy_entries.items()
        if key[0] == "historical" and descriptor.get("family_id") == "ntu_viral" and
        descriptor.get("evaluation_eligible") is True
    }
    if observed_sequences != expected_sequences:
        errors.append("reviewed NTU VIRAL profile pin does not cover exactly every selected sequence")
        valid = False
    # A reviewed manifest is not enough by itself: the profile policy must
    # explicitly carry revalidated identities for every input/GT/calibration
    # role.  This keeps a proposal or candidate receipt from upgrading bytes.
    for sequence in sorted(expected_sequences):
        descriptor = next((v for k, v in policy_entries.items()
                           if k[0] == "historical" and v.get("sequence_id") == sequence), None)
        if not isinstance(descriptor, Mapping):
            valid = False
            continue
        for role_key in ("input", "ground_truth", "calibration"):
            value = descriptor.get(role_key)
            if not isinstance(value, Mapping) or value.get("validation_status") != REVALIDATED:
                _record_missing(not_ready, f"reviewed NTU VIRAL profile pin lacks REVALIDATED {sequence} {role_key}")
                valid = False
    return valid


def _profile_entries(contract: Mapping[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    """Flatten every profile dataset/sequence into a stable partition key."""
    datasets = contract.get("datasets")
    if not isinstance(datasets, Mapping):
        return {}
    entries: dict[tuple[str, str], dict[str, Any]] = {}
    for partition in ("bringup", "development", "regression_only"):
        group = datasets.get(partition)
        if not isinstance(group, Mapping):
            continue
        for key, raw in group.items():
            item = dict(raw) if isinstance(raw, Mapping) else {}
            entries[(partition, str(key))] = {
                "profile_partition": partition,
                "profile_key": str(key),
                "dataset_id": item.get("dataset", str(key)),
                "sequence_id": item.get("sequence", str(key)),
                "profile": item,
            }
    for source_key, output_partition in (("holdout_slots", "historical"),
                                         ("fresh_holdout_slots", "fresh")):
        group = datasets.get(source_key)
        if not isinstance(group, Mapping):
            continue
        for key, raw in group.items():
            item = dict(raw) if isinstance(raw, Mapping) else {}
            entries[(output_partition, str(key))] = {
                "profile_partition": output_partition,
                "profile_key": str(key),
                "dataset_id": item.get("dataset"),
                "sequence_id": item.get("sequence"),
                "profile": item,
            }
    # NTU VIRAL is preregistered in dedicated profile groups so its historical
    # subset cannot silently become one of the existing Hilti holdout slots.
    # Prefixing the policy key keeps the profile/policy join unambiguous while
    # retaining the official sequence IDs (eee_01, nya_01, spms_01, tnp_01).
    for source_key, output_partition in (("ntu_viral_development", "development"),
                                         ("ntu_viral_historical", "historical")):
        group = datasets.get(source_key)
        if not isinstance(group, Mapping):
            continue
        for key, raw in group.items():
            item = dict(raw) if isinstance(raw, Mapping) else {}
            entries[(output_partition, f"ntu_viral_{key}")] = {
                "profile_partition": output_partition,
                "profile_key": f"ntu_viral_{key}",
                "dataset_id": item.get("dataset", f"ntu_viral_{key}"),
                "sequence_id": item.get("sequence", str(key)),
                "profile": item,
            }
    return entries


def _policy_entries(policy: Mapping[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    raw_entries = policy.get("datasets", policy.get("entries"))
    if isinstance(raw_entries, Mapping):
        raw_entries = list(raw_entries.values())
    if not isinstance(raw_entries, list):
        return {}
    result: dict[tuple[str, str], dict[str, Any]] = {}
    for raw in raw_entries:
        if not isinstance(raw, Mapping):
            continue
        partition = raw.get("profile_partition", raw.get("partition"))
        key = raw.get("profile_key", raw.get("slot_id", raw.get("dataset_id")))
        if isinstance(partition, str) and isinstance(key, str):
            result[(partition, key)] = dict(raw)
    return result


def _profile_identity(item: Mapping[str, Any]) -> dict[str, Any]:
    """Project legacy profile fields into the closure's artifact vocabulary."""
    return {
        "dataset_id": item.get("dataset"),
        "sequence_id": item.get("sequence"),
        "evaluation_eligible": item.get("evaluation_eligible"),
        "fresh_eligible": item.get("fresh_eligible"),
        "input": {
            "artifact_id": item.get("input_artifact_id"),
            "sha256": item.get("bag_sha256", item.get("raw_rosbag1_sha256")),
            "manifest_sha256": item.get("input_manifest_sha256"),
            "size_bytes": item.get("bag_expected_bytes"),
            "hash_kind": item.get("raw_rosbag1_hash_kind", "recorded_profile_hash"),
            "validation_status": RECORDED_ONLY,
        },
        "ground_truth": {
            "artifact_id": item.get("ground_truth_git_blob_oid"),
            "source_url": item.get("ground_truth_url"),
            "method": item.get("ground_truth_kind"),
            "frame": item.get("ground_truth_frame"),
            "timestamp_basis": item.get("ground_truth_timestamp_basis"),
            "sha256": item.get("ground_truth_sha256"),
            "size_bytes": item.get("ground_truth_expected_bytes"),
            "hash_kind": item.get("ground_truth_hash_kind", "recorded_profile_hash"),
            "validation_status": RECORDED_ONLY,
        },
        "calibration": {
            "artifact_id": item.get("calibration_tree_revision"),
            "sha256": item.get("calibration_archive_sha256"),
            "size_bytes": item.get("calibration_expected_bytes"),
            "hash_kind": item.get("calibration_archive_hash_kind", "recorded_profile_hash"),
            "frame_convention": item.get("calibration_frame_convention"),
            "timestamp_basis": item.get("calibration_timestamp_basis"),
            "validation_status": RECORDED_ONLY,
        },
        "source": {
            "input_url": item.get("bag_url"),
            "ground_truth_url": item.get("ground_truth_url"),
            "immutable_version": item.get("source_revision", item.get("calibration_tree_revision")),
        },
        "sensor_contract": {
            "modalities": item.get("modalities", item.get("required_modalities")),
            "topics": item.get("sensor_topics"),
            "formats": item.get("sensor_formats"),
            "timestamp_basis": item.get("sensor_timestamp_basis"),
        },
        "sequence": {
            "duration_seconds": item.get("duration_seconds"),
            "message_counts": item.get("message_counts"),
        },
    }


def _deep_merge(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if isinstance(result.get(key), Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _entry_identity(profile_entry: Mapping[str, Any], descriptor: Mapping[str, Any]) -> dict[str, Any]:
    merged = _deep_merge(_profile_identity(profile_entry.get("profile", {})), descriptor)
    merged["dataset_id"] = descriptor.get("dataset_id", profile_entry.get("dataset_id"))
    merged["sequence_id"] = descriptor.get("sequence_id", profile_entry.get("sequence_id"))
    return merged


def _check_profile_binding(
        profile_entry: Mapping[str, Any], descriptor: Mapping[str, Any],
        label: str, errors: list[str]) -> bool:
    """Reject a descriptor that silently re-labels the profile's identity."""
    valid = True
    for field in ("dataset_id", "sequence_id"):
        declared = descriptor.get(field)
        expected = profile_entry.get(field)
        if declared is not None and expected is not None and declared != expected:
            errors.append(f"{label}.{field} does not match profile")
            valid = False
    profile_item = profile_entry.get("profile", {})
    for field in ("evaluation_eligible", "fresh_eligible"):
        declared = descriptor.get(field)
        expected = profile_item.get(field) if isinstance(profile_item, Mapping) else None
        if (declared is not None and expected is not None and declared != expected):
            errors.append(f"{label}.{field} does not match profile")
            valid = False
    profile_identity = _profile_identity(profile_entry.get("profile", {}))
    for artifact_name in ("input", "calibration", "ground_truth"):
        declared_artifact = _mapping(descriptor.get(artifact_name))
        expected_artifact = _mapping(profile_identity.get(artifact_name))
        if declared_artifact is None or expected_artifact is None:
            continue
        for field in ("sha256", "size_bytes"):
            declared = declared_artifact.get(field)
            expected = expected_artifact.get(field)
            if declared is not None and expected is not None and declared != expected:
                errors.append(f"{label}.{artifact_name}.{field} does not match profile")
                valid = False
    return valid


def _record_missing(not_ready: list[str], label: str) -> None:
    if label not in not_ready:
        not_ready.append(label)


def _check_artifact(
        entry: Mapping[str, Any], name: str, errors: list[str],
        not_ready: list[str]) -> tuple[bool, str | None, str | None]:
    artifact = _mapping(entry.get(name))
    if artifact is None:
        _record_missing(not_ready, f"{name} identity is missing")
        return False, None, None
    valid = True
    for field in ("artifact_id", "sha256", "size_bytes", "hash_kind", "validation_status"):
        if field not in artifact or artifact.get(field) in (None, "", []):
            _record_missing(not_ready, f"{name}.{field} is missing")
            valid = False
    if "sha256" in artifact and artifact.get("sha256") is not None and not _sha(artifact.get("sha256")):
        errors.append(f"{name}.sha256 must be a 64-hex SHA-256")
        valid = False
    if "size_bytes" in artifact and artifact.get("size_bytes") is not None and not _positive_int(artifact.get("size_bytes")):
        errors.append(f"{name}.size_bytes must be a positive integer")
        valid = False
    status = artifact.get("validation_status")
    if status not in (RECORDED_ONLY, REVALIDATED):
        if status is None:
            _record_missing(not_ready, f"{name}.validation_status is missing")
        else:
            errors.append(f"{name}.validation_status is invalid")
        valid = False
    elif status == RECORDED_ONLY:
        _record_missing(not_ready, f"{name} hash/bytes are recorded-only and not revalidated")
        valid = False
    else:
        if not _sha(artifact.get("revalidation_receipt_sha256")):
            _record_missing(not_ready, f"{name}.revalidation_receipt_sha256 is missing")
            valid = False
        if not _text(artifact.get("revalidation_mount_identity")):
            _record_missing(not_ready, f"{name}.revalidation_mount_identity is missing")
            valid = False
    return valid, artifact.get("sha256"), artifact.get("artifact_id")


def _check_source(entry: Mapping[str, Any], errors: list[str], not_ready: list[str]) -> bool:
    source = _mapping(entry.get("source"))
    if source is None:
        _record_missing(not_ready, "official source identity is missing")
        return False
    valid = True
    for field in ("project_name", "publisher", "project_url"):
        if not _text(source.get(field)):
            _record_missing(not_ready, f"official source {field} is missing")
            valid = False
    if source.get("official_primary") is not True:
        _record_missing(not_ready, "official primary source attestation is missing")
        valid = False
    version = source.get("immutable_version")
    if not _text(version):
        _record_missing(not_ready, "official source immutable version is missing")
        valid = False
    elif not IMMUTABLE_VERSION_RE.fullmatch(version):
        _record_missing(not_ready, "official source reference is moving or not immutable")
        valid = False
    refs = _mapping(source.get("download_refs"))
    if refs is None:
        _record_missing(not_ready, "official immutable download references are missing")
        return False
    gt = _mapping(entry.get("ground_truth"))
    gt_applicable = not (isinstance(gt, Mapping) and
                         gt.get("role") in {"not_applicable", "cross_validation_only"})
    roles = ("input", "calibration", "ground_truth") if gt_applicable else (
        "input", "calibration")
    for role in roles:
        ref = _mapping(refs.get(role))
        if ref is None:
            _record_missing(not_ready, f"official source download reference missing: {role}")
            valid = False
            continue
        if not _text(ref.get("url")):
            _record_missing(not_ready, f"official source {role} URL is missing")
            valid = False
        if ref.get("immutable") is not True:
            _record_missing(not_ready, f"official source {role} URL is moving or unpinned")
            valid = False
        if ref.get("primary") is not True:
            _record_missing(not_ready, f"official source {role} is not marked primary")
            valid = False
        if (_text(version) and IMMUTABLE_VERSION_RE.fullmatch(version) and
                _text(ref.get("url")) and version not in ref.get("url", "")):
            _record_missing(not_ready, f"official source {role} URL is not bound to immutable version")
            valid = False
        if _text(ref.get("url")) and urlparse(ref["url"]).scheme != "https":
            errors.append(f"official source {role} URL must use https")
            valid = False
    return valid


def _check_license(entry: Mapping[str, Any], not_ready: list[str]) -> bool:
    license_info = _mapping(entry.get("license_terms"))
    if license_info is None:
        _record_missing(not_ready, "license/terms identity is missing")
        return False
    valid = True
    for field in ("license_id", "terms_url", "citation"):
        if not _text(license_info.get(field)):
            _record_missing(not_ready, f"license/terms {field} is missing")
            valid = False
    if license_info.get("status") != "READY":
        _record_missing(not_ready, "official dataset license/terms are not READY")
        valid = False
    return valid


def _check_sensor_and_geometry(
        entry: Mapping[str, Any], errors: list[str], not_ready: list[str]) -> bool:
    sensor = _mapping(entry.get("sensor_contract"))
    gt = _mapping(entry.get("ground_truth"))
    calibration = _mapping(entry.get("calibration"))
    valid = True
    if sensor is None:
        _record_missing(not_ready, "sensor contract is missing")
        return False
    for field in ("topics", "formats", "timestamp_basis"):
        value = sensor.get(field)
        if (not isinstance(value, list) or not value or
                any(not _text(item) for item in value)) if field != "timestamp_basis" else not _text(value):
            _record_missing(not_ready, f"sensor contract {field} is missing")
            valid = False
    if gt is None:
        _record_missing(not_ready, "ground-truth identity is missing")
        return False
    if gt.get("role") in {"not_applicable", "cross_validation_only"}:
        if calibration is None:
            _record_missing(not_ready, "calibration identity is missing")
            return False
        for field in ("frame_convention", "timestamp_basis"):
            if not _text(calibration.get(field)):
                _record_missing(not_ready, f"calibration {field} is missing")
                valid = False
        return valid
    for field in ("method", "frame", "timestamp_basis"):
        if not _text(gt.get(field)):
            _record_missing(not_ready, f"ground truth {field} is missing")
            valid = False
    if calibration is None:
        _record_missing(not_ready, "calibration identity is missing")
        return False
    for field in ("frame_convention", "timestamp_basis"):
        if not _text(calibration.get(field)):
            _record_missing(not_ready, f"calibration {field} is missing")
            valid = False
    alignment = _mapping(entry.get("alignment"))
    if (_text(gt.get("timestamp_basis")) and _text(sensor.get("timestamp_basis")) and
            gt.get("timestamp_basis") != sensor.get("timestamp_basis") and
            not _text(alignment.get("timestamp_conversion") if alignment else None)):
        errors.append("ground truth timestamp basis differs from sensor basis without an explicit conversion")
        valid = False
    if (_text(gt.get("frame")) and _text(calibration.get("frame_convention")) and
            gt.get("frame") != calibration.get("frame_convention") and
            not _text(alignment.get("frame_transform") if alignment else None)):
        errors.append("ground truth frame differs from calibration convention without an explicit transform")
        valid = False
    return valid


def _check_sequence(entry: Mapping[str, Any], not_ready: list[str]) -> bool:
    sequence = _mapping(entry.get("sequence"))
    if sequence is None:
        _record_missing(not_ready, "sequence duration/count identity is missing")
        return False
    valid = True
    if not _positive_number(sequence.get("duration_seconds")):
        _record_missing(not_ready, "sequence duration_seconds is missing")
        valid = False
    counts = sequence.get("message_counts")
    if not isinstance(counts, Mapping) or not counts:
        _record_missing(not_ready, "sequence message_counts are missing")
        valid = False
    elif any(not _positive_int(value) for value in counts.values()):
        _record_missing(not_ready, "sequence message_counts are incomplete or invalid")
        valid = False
    return valid


def _check_supported_systems(
        entry: Mapping[str, Any], required_systems: tuple[str, ...],
        errors: list[str], not_ready: list[str]) -> bool:
    observed = entry.get("supported_systems")
    if not isinstance(observed, list) or not observed:
        _record_missing(not_ready, "runner support declaration is missing")
        return False
    if any(not isinstance(item, str) or not item for item in observed):
        errors.append("runner support declaration contains a malformed system")
        return False
    if len(set(observed)) != len(observed):
        errors.append("runner support declaration contains duplicate systems")
        return False
    missing = sorted(set(required_systems) - set(observed))
    extra = sorted(set(observed) - set(required_systems))
    if missing:
        errors.append("dataset does not support every required runner: " + ", ".join(missing))
    if extra:
        errors.append("dataset declares unsupported runner(s): " + ", ".join(extra))
    return not missing and not extra


def verify_dataset_source_closure(
        profile: Mapping[str, Any], *, root: Path = ROOT,
        policy: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return a fail-closed metadata audit for all profile dataset entries."""
    root = root.resolve()
    configured = policy if isinstance(policy, Mapping) else _policy_from_profile(profile)
    if not isinstance(configured, Mapping) or configured.get("required") is not True:
        return {
            "schema_version": SCHEMA_VERSION,
            "status": "NOT_REQUIRED",
            "pass": True,
            "claim_eligible": True,
            "errors": [],
            "not_ready": [],
        }
    errors: list[str] = []
    not_ready: list[str] = []
    contract = _profile_document(profile)
    profile_entries = _profile_entries(contract)
    policy_entries = _policy_entries(configured)
    if not policy_entries:
        _record_missing(not_ready, "dataset_source_closure.datasets is missing")
    _check_file_binding(configured, "schema_path", "schema_sha256", root,
                        not_ready, errors)
    _check_file_binding(configured, "auditor_path", "auditor_sha256", root,
                        not_ready, errors)
    _check_file_binding(configured, "selection_path", "selection_sha256", root,
                        not_ready, errors)
    _check_file_binding(configured, "selection_schema_path", "selection_schema_sha256",
                        root, not_ready, errors)
    _check_ntu_viral_selection_manifest(
        configured, root, profile_entries, policy_entries, not_ready, errors)
    reviewed_pin_pass = _check_ntu_viral_reviewed_pin(
        configured, root, profile_entries, policy_entries, not_ready, errors)
    hash_policy = _mapping(configured.get("hash_policy"))
    if hash_policy is None:
        _record_missing(not_ready, "dataset_source_closure.hash_policy is missing")
    else:
        if hash_policy.get("recorded_status") != RECORDED_ONLY:
            errors.append("dataset_source_closure.hash_policy.recorded_status must be RECORDED_ONLY")
        if hash_policy.get("revalidated_status") != REVALIDATED:
            errors.append("dataset_source_closure.hash_policy.revalidated_status must be REVALIDATED")
        if hash_policy.get("revalidation_requires_mount_identity") is not True:
            errors.append("dataset_source_closure.hash_policy must require mount identity for revalidation")
        if hash_policy.get("ground_truth_content_opened") is not False:
            errors.append("dataset_source_closure.hash_policy ground_truth_content_opened must be false")
    expected_keys = set(profile_entries)
    observed_keys = set(policy_entries)
    for key in sorted(expected_keys - observed_keys):
        _record_missing(not_ready, f"dataset source descriptor missing: {key[0]}/{key[1]}")
    for key in sorted(observed_keys - expected_keys):
        errors.append(f"dataset source descriptor is not declared by profile: {key[0]}/{key[1]}")

    required_systems_value = configured.get("required_systems", DEFAULT_REQUIRED_SYSTEMS)
    if (not isinstance(required_systems_value, list) or
            any(not isinstance(item, str) or not item for item in required_systems_value)):
        errors.append("dataset_source_closure.required_systems is malformed")
        required_systems = DEFAULT_REQUIRED_SYSTEMS
    else:
        required_systems = tuple(required_systems_value)
    seen_ids: dict[str, tuple[str, str]] = {}
    seen_sequences: dict[str, tuple[str, str]] = {}
    seen_hashes: dict[str, tuple[str, str]] = {}
    families: set[str] = set()
    family_validity: dict[str, list[bool]] = {}
    fresh_keys: set[tuple[str, str]] = set()
    historical_keys: set[tuple[str, str]] = set()
    per_dataset: dict[str, Any] = {}
    for key in sorted(expected_keys & observed_keys):
        profile_entry = profile_entries[key]
        descriptor = policy_entries[key]
        label = f"{key[0]}/{key[1]}"
        binding_valid = _check_profile_binding(
            profile_entries[key], descriptor, label, errors)
        merged = _entry_identity(profile_entry, descriptor)
        valid = binding_valid
        dataset_id = merged.get("dataset_id")
        sequence_id = merged.get("sequence_id")
        if not _text(dataset_id):
            _record_missing(not_ready, f"{label}.dataset_id is missing")
            valid = False
        if not _text(sequence_id):
            _record_missing(not_ready, f"{label}.sequence_id is missing")
            valid = False
        if _text(dataset_id):
            previous = seen_ids.get(dataset_id)
            if previous and previous != key:
                errors.append(f"dataset ID overlaps {label} and {previous[0]}/{previous[1]}")
                valid = False
            seen_ids[dataset_id] = key
        if _text(sequence_id):
            previous = seen_sequences.get(sequence_id)
            if previous and previous != key:
                errors.append(f"sequence ID overlaps {label} and {previous[0]}/{previous[1]}")
                valid = False
            seen_sequences[sequence_id] = key
        family = merged.get("family_id")
        gt_role = _mapping(merged.get("ground_truth"))
        gt_applicable = not (isinstance(gt_role, Mapping) and
                             gt_role.get("role") in {"not_applicable", "cross_validation_only"})
        evaluation_eligible = merged.get("evaluation_eligible")
        if evaluation_eligible is None:
            evaluation_eligible = True
        if not isinstance(evaluation_eligible, bool):
            _record_missing(not_ready, f"{label}.evaluation_eligible is missing or malformed")
            evaluation_eligible = False
            valid = False
        fresh_eligible = merged.get("fresh_eligible")
        if fresh_eligible is None:
            fresh_eligible = True
        if not isinstance(fresh_eligible, bool):
            _record_missing(not_ready, f"{label}.fresh_eligible is missing or malformed")
            fresh_eligible = False
            valid = False
        if not evaluation_eligible and fresh_eligible:
            errors.append(f"{label} is evaluation-excluded but fresh_eligible is true")
            valid = False
        if gt_applicable and evaluation_eligible:
            if not _text(family):
                _record_missing(not_ready, f"{label}.family_id is missing")
                valid = False
            else:
                families.add(family)
        artifact_names = ["input", "calibration"]
        if gt_applicable:
            artifact_names.append("ground_truth")
        for artifact_name in artifact_names:
            artifact_valid, artifact_hash, artifact_id = _check_artifact(
                merged, artifact_name, errors, not_ready)
            valid = artifact_valid and valid
            if artifact_name in {"input", "ground_truth"}:
                for identity in (artifact_hash, artifact_id):
                    if identity in (None, ""):
                        continue
                    previous = seen_hashes.get(str(identity))
                    if previous and previous != key:
                        errors.append(f"immutable {artifact_name} identity overlaps {label} and {previous[0]}/{previous[1]}")
                        valid = False
                    seen_hashes[str(identity)] = key
        valid = _check_source(merged, errors, not_ready) and valid
        valid = _check_license(merged, not_ready) and valid
        valid = _check_sensor_and_geometry(merged, errors, not_ready) and valid
        valid = _check_sequence(merged, not_ready) and valid
        valid = _check_supported_systems(merged, required_systems, errors, not_ready) and valid
        if gt_applicable and evaluation_eligible and _text(family):
            family_validity.setdefault(str(family), []).append(valid)
        if key[0] == "fresh":
            fresh_keys.add(key)
        if key[0] == "historical":
            historical_keys.add(key)
        per_dataset[label] = {
            "pass": valid,
            "dataset_id": dataset_id,
            "sequence_id": sequence_id,
            "family_id": family,
            "partition": key[0],
        }
    for partition in configured.get("required_partitions", REQUIRED_PARTITIONS):
        if partition not in {key[0] for key in expected_keys}:
            _record_missing(not_ready, f"required dataset partition is missing: {partition}")
    minimum_families = configured.get("minimum_gt_dataset_families", 2)
    if not isinstance(minimum_families, int) or isinstance(minimum_families, bool) or minimum_families < 2:
        errors.append("minimum_gt_dataset_families must be an integer >= 2")
        minimum_families = 2
    passed_families = {
        family for family, values in family_validity.items()
        if values and all(values)
    }
    if len(passed_families) < minimum_families:
        _record_missing(
            not_ready,
            f"only {len(passed_families)} PASS GT dataset family/families; require at least {minimum_families}")
    if configured.get("require_fresh_holdout") is True and not fresh_keys:
        _record_missing(not_ready, "fresh holdout dataset partition is missing")
    if configured.get("require_historical_partition") is True and not historical_keys:
        _record_missing(not_ready, "historical dataset partition is missing")
    status_reason = configured.get("status_reason")
    if configured.get("status") != "READY":
        if isinstance(status_reason, list):
            for reason in status_reason:
                _record_missing(not_ready, "declared policy: " + str(reason))
        else:
            _record_missing(not_ready, "dataset source closure policy is not READY")
    elif not_ready:
        errors.append("dataset source closure is declared READY but has unresolved requirements")
    status = "INVALID" if errors else ("NOT_READY" if not_ready else "PASS")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "pass": status == "PASS",
        "claim_eligible": status == "PASS",
        "errors": sorted(set(errors)),
        "not_ready": sorted(set(not_ready)),
        "checks": {
            "profile_dataset_entries": {
                "pass": expected_keys == observed_keys,
                "expected_count": len(expected_keys),
                "observed_count": len(observed_keys),
            },
            "gt_dataset_families": {
                "pass": len(passed_families) >= minimum_families,
                "families": sorted(families),
                "passed_families": sorted(passed_families),
                "family_entry_validity": {
                    family: list(values)
                    for family, values in sorted(family_validity.items())
                },
                "minimum": minimum_families,
            },
            "fresh_holdout_partition": {
                "pass": bool(fresh_keys) if configured.get("require_fresh_holdout") is True else True,
                "entries": sorted(f"{a}/{b}" for a, b in fresh_keys),
            },
            "historical_partition": {
                "pass": bool(historical_keys) if configured.get("require_historical_partition") is True else True,
                "entries": sorted(f"{a}/{b}" for a, b in historical_keys),
            },
            "hashes_are_metadata_only": {
                "pass": True,
                "gt_content_opened": False,
                "recorded_only_is_not_revalidated": True,
            },
            "reviewed_profile_pin": {
                "pass": reviewed_pin_pass,
                "required": configured.get("reviewed_pin_required") is True,
                "candidate_receipts_accepted": False,
                "gt_content_opened": False,
            },
        },
        "per_dataset": per_dataset,
        "dataset_source_closure_sha256": canonical_dataset_source_closure_sha256(configured),
    }


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    result = verify_dataset_source_closure(profile, root=ROOT)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
