#!/usr/bin/env python3
"""Compose immutable host-gate evidence from a v12 binder document.

The compositor accepts only the v12 host-bound document and the pinned v12
profile.  It never parses the feeder, callback, terminal, or timing source
documents directly; those observations were validated by the binder.  It
does, however, hash each bound source path to detect replacement or symlink
drift before declaring a PASS.  A successful result keeps transport,
terminal-support, and timing metrics in separate sections and is always
ground-truth/scorer blind.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Optional

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from lidarslam_benchmark_tools.benchmark_phase_contract import (  # noqa: E402
    CONTRACT_VERSION_V5,
    PhaseContractError,
    atomic_write_json,
    file_sha256,
    validate_terminal_support_context_v5,
)


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml"
PROFILE_SHA256 = "675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d"
PROFILE_KEY = "m6a10_fast_livo2_v2c_v12_formal_ready"
PROFILE_SCHEMA = 3
PROFILE_SYSTEM = "fast_livo2"
PROFILE_STATUS = "source_unit_gate_validated"
PROFILE_CONTRACT_ID = "m6a10-v2c-fast-livo2-terminal-support-context-v12-formal-ready"

TOPICS = ("lidar", "imu", "image")
CONSUMER_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_RAW_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
FEEDER_CONTRACT = "m6a10-v2c-fast-livo2-single-inflight-feeder-v1"
TIMING_CONTRACT = "m6a10-online-compute-v3-timing-v1"
BOUND_CONTRACT = "m6a10-v12-host-bound-transport-terminal-v1"
COMPOSED_CONTRACT = "m6a10-v12-host-composed-transport-terminal-v1"
SENSOR_DURATION_SECONDS = 579.278127298
MAX_END_GAP_SECONDS = 0.25
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352

RAW_AUTHORITY_FIELDS = frozenset({
    "input", "profile_sha256", "profile_path", "raw_path", "raw_sha256",
    "expected", "expected_messages", "expected_topic_counts", "published",
    "published_messages", "published_topic_counts", "ack", "acked",
    "acked_messages", "acked_topic_counts", "acknowledged",
    "acknowledged_messages", "acknowledged_topic_counts", "binding",
    "validation", "required_evaluation_end_timestamp_seconds",
})


def _error(kind: str, message: str) -> PhaseContractError:
    return PhaseContractError(f"{kind}: {message}")


def _reject_symlink_components(path: Path, label: str) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise _error("SYMLINK_REJECTED", f"{label} contains symlink {current}")


def _regular(path: Path, label: str) -> None:
    _reject_symlink_components(path, label)
    if path.is_symlink() or not path.is_file():
        raise _error("NOT_REGULAR", f"{label} is not a regular file")
    if path.name.endswith(".part"):
        raise _error("STAGING_INPUT", f"{label} is a staging file")


def _json(path: Path, label: str) -> dict[str, Any]:
    _regular(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error("JSON_INVALID", f"{label} JSON is invalid") from exc
    if not isinstance(value, dict):
        raise _error("JSON_NOT_OBJECT", f"{label} must be an object")
    return value


def _counts(value: Any, label: str) -> dict[str, int]:
    if not isinstance(value, dict) or set(value) != set(TOPICS):
        raise _error("COUNTS_INVALID", f"{label} must contain exactly {TOPICS}")
    result: dict[str, int] = {}
    for topic in TOPICS:
        count = value.get(topic)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise _error("COUNTS_INVALID", f"{label}.{topic} is invalid")
        result[topic] = count
    return result


def _finite(value: Any, label: str, *, nonnegative: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error("NUMBER_INVALID", f"{label} is not numeric")
    result = float(value)
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        raise _error("NUMBER_INVALID", f"{label} is not finite and valid")
    return result


def _profile(
    path: Path = PROFILE_PATH,
    expected_profile_sha256: Optional[str] = PROFILE_SHA256,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    _regular(path, "profile")
    observed_sha = file_sha256(path)
    if expected_profile_sha256 is not None and observed_sha != expected_profile_sha256:
        raise _error("PROFILE_DRIFT", "v12 profile SHA-256 differs from pin")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise _error("PROFILE_INVALID", "profile YAML is invalid") from exc
    if not isinstance(document, dict) or not isinstance(document.get("competitive_slam_profile"), dict):
        raise _error("PROFILE_INVALID", "competitive_slam_profile is missing")
    profile = document["competitive_slam_profile"].get(PROFILE_KEY)
    if not isinstance(profile, dict) or profile.get("schema_version") != PROFILE_SCHEMA or \
            profile.get("profile_key") != PROFILE_KEY or profile.get("system") != PROFILE_SYSTEM or \
            profile.get("status") != PROFILE_STATUS or profile.get("contract_id") != PROFILE_CONTRACT_ID:
        raise _error("PROFILE_INVALID", "v12 profile schema/key/system/status mismatch")
    input_value = profile.get("input")
    phase = profile.get("phase")
    if not isinstance(input_value, dict) or not isinstance(phase, dict):
        raise _error("PROFILE_INVALID", "v12 profile input/phase missing")
    expected_counts = _counts(input_value.get("expected_topic_counts"), "profile.expected_topic_counts")
    if (input_value.get("path") != "/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag" or
            input_value.get("bytes") != 11290464091 or
            input_value.get("sha256") != "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310" or
            input_value.get("expected_messages") != 236687 or
            expected_counts != {"lidar": 5793, "imu": 225102, "image": 5792}):
        raise _error("PROFILE_INPUT_DRIFT", "v12 profile input identity differs from pin")
    if phase.get("schema_version") != PROFILE_SCHEMA or phase.get("contract_version") != CONSUMER_CONTRACT or \
            phase.get("mode") != "unpaced_ack" or phase.get("maximum_end_gap_seconds") != MAX_END_GAP_SECONDS or \
            phase.get("required_evaluation_end_timestamp_seconds") != REQUIRED_END_TIMESTAMP_SECONDS:
        raise _error("PROFILE_PHASE_INVALID", "v12 profile phase contract differs from pin")
    execution = profile.get("execution")
    if not isinstance(execution, dict) or execution.get("status") != "build_not_run" or \
            execution.get("formal_replay_started") is not False:
        raise _error("PROFILE_EXECUTION_INVALID", "v12 profile is not build-not-run")
    expected = {
        "bag_path": input_value["path"],
        "bag_bytes": input_value["bytes"],
        "bag_sha256": input_value["sha256"],
        "expected_messages": input_value["expected_messages"],
        "expected_topic_counts": expected_counts,
        "required_end_timestamp_seconds": REQUIRED_END_TIMESTAMP_SECONDS,
        "maximum_end_gap_seconds": MAX_END_GAP_SECONDS,
        "sensor_duration_seconds": SENSOR_DURATION_SECONDS,
    }
    return document, expected, observed_sha


def _reject_raw_authority(value: Mapping[str, Any], label: str) -> None:
    present = sorted(key for key in value if isinstance(key, str) and key.lower() in RAW_AUTHORITY_FIELDS)
    if present:
        raise _error("RAW_AUTHORITY_FIELDS", f"{label} contains authority fields {present}")
    for key in value:
        if not isinstance(key, str):
            raise _error("RAW_AUTHORITY_FIELDS", f"{label} has a non-string key")
        lowered = key.lower()
        if lowered.startswith(("expected_", "published_", "ack_", "acked_", "acknowledged_")):
            raise _error("RAW_AUTHORITY_FIELDS", f"{label} contains authority field {key}")


def _safety(value: Mapping[str, Any], label: str) -> None:
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise _error("SAFETY_FLAGS", f"{label} GT/scorer flags are not false")


def _source_file(value: Any, label: str) -> tuple[str, str]:
    if not isinstance(value, str) or not value.startswith("/"):
        raise _error("SOURCE_PATH_INVALID", f"{label} path is not absolute")
    path = Path(value)
    _regular(path, label)
    resolved = str(path.resolve())
    if resolved != value:
        raise _error("SOURCE_PATH_INVALID", f"{label} path is not canonical")
    try:
        digest = file_sha256(path)
    except OSError as exc:
        raise _error("SOURCE_HASH_FAILED", f"{label} cannot be hashed") from exc
    return resolved, digest


def _sha_field(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise _error("SOURCE_SHA_INVALID", f"{label} is not a lowercase SHA-256")
    return value


def _validate_bound_shape(value: Mapping[str, Any], profile_path: Path, expected: Mapping[str, Any]) -> None:
    required = {"schema_version", "contract_id", "status", "profile", "input", "feeder", "transport",
                "terminal_support_context", "timing", "sources", "safety"}
    if set(value) != required:
        raise _error("BOUND_SCHEMA", "bound document fields are missing or contaminated")
    if value.get("schema_version") != 1 or value.get("contract_id") != BOUND_CONTRACT or value.get("status") != "BOUND":
        raise _error("BOUND_CONTRACT", "bound schema/contract/status invalid")
    profile = value.get("profile")
    if not isinstance(profile, dict) or set(profile) != {"path", "sha256", "key"} or \
            profile.get("path") != str(profile_path.resolve()) or profile.get("key") != PROFILE_KEY:
        raise _error("BOUND_PROFILE", "bound profile identity mismatch")
    _sha_field(profile.get("sha256"), "bound.profile.sha256")
    input_value = value.get("input")
    if not isinstance(input_value, dict):
        raise _error("BOUND_INPUT", "bound input is missing")
    if input_value.get("bag_path") != expected["bag_path"] or input_value.get("bag_bytes") != expected["bag_bytes"] or \
            input_value.get("bag_sha256") != expected["bag_sha256"] or input_value.get("expected_messages") != expected["expected_messages"] or \
            _counts(input_value.get("expected_topic_counts"), "bound.input.expected_topic_counts") != expected["expected_topic_counts"] or \
            input_value.get("required_end_timestamp_seconds") != expected["required_end_timestamp_seconds"]:
        raise _error("BOUND_INPUT", "bound input identity/counts differ from profile")
    safety = value.get("safety")
    if not isinstance(safety, dict) or safety.get("bag_opened") is not False or \
            safety.get("ground_truth_content_opened") is not False or safety.get("scorer_invoked") is not False or \
            safety.get("map_saved") is not False or safety.get("formal_replay_started") is not False:
        raise _error("SAFETY_FLAGS", "bound safety fields are not false")


def _validate_sources(value: Mapping[str, Any], bound_path: Path, profile_path: Path) -> dict[str, dict[str, Any]]:
    names = ("feeder", "transport", "terminal_support_context", "timing")
    sources = value.get("sources")
    if not isinstance(sources, dict) or set(sources) != {"feeder_sha256", "callback_sha256", "terminal_sha256", "timing_sha256"}:
        raise _error("SOURCE_BINDINGS", "bound source hash map is incomplete")
    section_to_source = {
        "feeder": "feeder_sha256", "transport": "callback_sha256",
        "terminal_support_context": "terminal_sha256", "timing": "timing_sha256",
    }
    result: dict[str, dict[str, Any]] = {}
    resolved_paths: list[str] = []
    hashes: list[str] = []
    forbidden = {str(bound_path.resolve()), str(profile_path.resolve())}
    for name in names:
        section = value.get(name)
        if not isinstance(section, dict) or set(section) != {"path", "sha256", "raw", "validation"} or \
                not isinstance(section.get("raw"), dict) or not isinstance(section.get("validation"), dict):
            raise _error("SOURCE_BINDINGS", f"bound {name} section is incomplete")
        declared = _sha_field(section.get("sha256"), f"bound.{name}.sha256")
        path, observed = _source_file(section.get("path"), f"bound.{name}")
        if path in forbidden:
            raise _error("SOURCE_BINDINGS", f"bound {name} aliases profile or bound document")
        if observed != declared:
            raise _error("SOURCE_HASH_DRIFT", f"bound {name} source SHA differs from bound SHA")
        source_key = section_to_source[name]
        if sources.get(source_key) != declared:
            raise _error("SOURCE_HASH_DRIFT", f"bound sources.{source_key} differs from section")
        resolved_paths.append(path)
        hashes.append(declared)
        result[name] = section
    if len(set(resolved_paths)) != len(resolved_paths) or len(set(hashes)) != len(hashes):
        raise _error("SOURCE_BINDINGS", "bound source paths and hashes must be unique")
    return result


def _validate_feeder(raw: Mapping[str, Any], validation: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    if raw.get("schema_version") != 1 or raw.get("contract_id") != FEEDER_CONTRACT or raw.get("status") != "pass":
        raise _error("FEEDER_CONTRACT", "bound feeder raw contract/status invalid")
    if raw.get("bag_path") != expected["bag_path"] or raw.get("bag_bytes") != expected["bag_bytes"] or raw.get("bag_sha256") != expected["bag_sha256"]:
        raise _error("FEEDER_INPUT", "bound feeder bag identity differs from profile")
    counts = expected["expected_topic_counts"]
    expected_counts = _counts(raw.get("expected_topic_counts"), "feeder.expected_topic_counts")
    published = _counts(raw.get("published_topic_counts"), "feeder.published_topic_counts")
    acknowledged = _counts(raw.get("acked_topic_counts"), "feeder.acked_topic_counts")
    if expected_counts != counts or published != counts or acknowledged != counts:
        raise _error("FEEDER_COUNTS", "bound feeder exact counts failed")
    if raw.get("published_messages") != expected["expected_messages"] or raw.get("single_inflight") is not True or \
            raw.get("publisher_queue_size") != 1 or raw.get("ack_backpressure_verified") is not True:
        raise _error("FEEDER_TRANSACTION", "bound feeder single-inflight proof failed")
    if raw.get("duplicate") is True or raw.get("stale") is True or raw.get("duplicate_of") is not None:
        raise _error("FEEDER_DUPLICATE", "bound feeder receipt is duplicate/stale")
    _safety(raw, "bound feeder")
    summary = {"expected": dict(counts), "published": dict(counts), "acknowledged": dict(counts), "messages": expected["expected_messages"]}
    if dict(validation) != summary:
        raise _error("BOUND_VALIDATION", "bound feeder validation summary drifted")
    return summary


def _validate_transport(raw: Mapping[str, Any], validation: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_raw_authority(raw, "bound callback raw")
    if raw.get("schema_version") != 3 or raw.get("contract_version") != CONSUMER_CONTRACT or \
            raw.get("transport_contract_version") != TRANSPORT_CONTRACT or raw.get("phase_mode") != "unpaced_ack" or \
            raw.get("system") != PROFILE_SYSTEM or raw.get("status") != "pass":
        raise _error("TRANSPORT_CONTRACT", "bound callback schema/contract/status invalid")
    _safety(raw, "bound callback")
    ledger = raw.get("consumer")
    if not isinstance(ledger, dict):
        raise _error("TRANSPORT_LEDGER", "bound callback consumer ledger missing")
    counts = expected["expected_topic_counts"]
    if _counts(ledger.get("received_topic_counts"), "callback.received_topic_counts") != counts or \
            ledger.get("received_messages") != expected["expected_messages"] or ledger.get("acked_messages") != expected["expected_messages"]:
        raise _error("TRANSPORT_COUNTS", "bound callback received/ACK counts failed")
    if ledger.get("ack_exact") is not True or ledger.get("transport_outstanding_at_drain") != 0 or \
            ledger.get("maximum_allowed_transport_outstanding_messages") != 1 or \
            isinstance(ledger.get("maximum_transport_outstanding_messages"), bool) or \
            not isinstance(ledger.get("maximum_transport_outstanding_messages"), int) or \
            not 0 <= ledger["maximum_transport_outstanding_messages"] <= 1:
        raise _error("TRANSPORT_OUTSTANDING", "bound callback transport ledger failed")
    for key in ("mapper_internal_deque_current_messages", "mapper_internal_deque_peak_messages"):
        if isinstance(ledger.get(key), bool) or not isinstance(ledger.get(key), int) or ledger[key] < 0:
            raise _error("TRANSPORT_DIAGNOSTIC", f"bound callback {key} invalid")
    if ledger.get("eof_observed") is not True or ledger.get("drain_complete") is not True or \
            ledger.get("dropped_messages") != 0 or ledger.get("queue_overflow") != 0 or \
            ledger.get("processing_failures") != 0 or ledger.get("ack_backpressure_verified") is not True:
        raise _error("TRANSPORT_COMPLETION", "bound callback completion counters failed")
    summary = {
        "received_topic_counts": dict(counts), "received_messages": expected["expected_messages"],
        "acked_messages": expected["expected_messages"], "transport_outstanding_at_drain": 0,
        "maximum_transport_outstanding_messages": ledger["maximum_transport_outstanding_messages"],
        "mapper_internal_deque_current_messages": ledger["mapper_internal_deque_current_messages"],
        "mapper_internal_deque_peak_messages": ledger["mapper_internal_deque_peak_messages"],
    }
    if dict(validation) != summary:
        raise _error("BOUND_VALIDATION", "bound transport validation summary drifted")
    return summary


def _terminal_adapter(raw: Mapping[str, Any], feeder: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    adapted = copy.deepcopy(dict(raw))
    adapted["schema_version"] = 3
    adapted["contract_version"] = CONTRACT_VERSION_V5
    adapted["required_evaluation_end_timestamp_seconds"] = expected["required_end_timestamp_seconds"]
    adapted["maximum_end_gap_seconds"] = expected["maximum_end_gap_seconds"]
    received = _counts(raw.get("received_topic_counts"), "terminal.received_topic_counts")
    adapted["counts"] = {
        "expected": dict(expected["expected_topic_counts"]), "published": dict(feeder["published"]),
        "received": received, "acknowledged": dict(feeder["acknowledged"]),
    }
    return validate_terminal_support_context_v5(
        adapted, maximum_end_gap_seconds=expected["maximum_end_gap_seconds"], require_pass=True)


def _validate_terminal(raw: Mapping[str, Any], validation: Mapping[str, Any], feeder: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_raw_authority(raw, "bound terminal raw")
    if any(field in raw for field in ("contract_version", "counts", "maximum_end_gap_seconds")):
        raise _error("RAW_AUTHORITY_FIELDS", "bound terminal raw contains host v5 fields")
    if raw.get("schema_version") != 1 or raw.get("contract_id") != TERMINAL_RAW_CONTRACT or \
            raw.get("phase_mode") != "unpaced_ack" or raw.get("system") != PROFILE_SYSTEM or raw.get("status") != "pass":
        raise _error("TERMINAL_CONTRACT", "bound terminal schema/contract/status invalid")
    _safety(raw, "bound terminal")
    _counts(raw.get("received_topic_counts"), "terminal.received_topic_counts")
    for field in ("dropped_counts", "overflow_counts"):
        counters = _counts(raw.get(field), f"terminal.{field}")
        if any(counters.values()):
            raise _error("TERMINAL_COUNTERS", f"bound terminal {field} are nonzero")
    failures = raw.get("processing_failures")
    if isinstance(failures, bool) or not isinstance(failures, int) or failures != 0:
        raise _error("TERMINAL_COUNTERS", "bound terminal processing_failures is nonzero")
    result = _terminal_adapter(raw, feeder, expected)
    if validation.get("status") != "pass" or validation.get("validation") != result.get("validation"):
        raise _error("BOUND_VALIDATION", "bound terminal v5 validation summary drifted")
    details = result["validation"]
    return {
        "status": "pass", "contract_version": CONTRACT_VERSION_V5,
        "backend_boundary_timestamp_seconds": details["backend_boundary_timestamp_seconds"],
        "support_context_classification": details["support_context_classification"],
        "support_context_counts": dict(details["support_context_counts"]),
        "support_context_total_count": details["support_context_total_count"],
        "completed_counts": dict(details["completed_counts"]),
        "received_counts": dict(details["received_counts"]),
        "count_conservation_passed": details["count_conservation_passed"],
        "residual_lidar_forbidden": details["residual_lidar_forbidden"],
        "trajectory_end_gap_seconds": details["trajectory_end_gap_seconds"],
        "maximum_end_gap_seconds": details["maximum_end_gap_seconds"],
        "stable_terminal_polls": raw["terminal_observation"]["stable_poll_count"],
        "ground_truth_content_opened": False, "scorer_invoked": False,
    }


def _validate_timing(raw: Mapping[str, Any], validation: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_raw_authority(raw, "bound timing raw")
    if raw.get("schema_version") != 1 or raw.get("contract_version") != TIMING_CONTRACT or raw.get("status") != "PASS":
        raise _error("TIMING_CONTRACT", "bound timing schema/contract/status invalid")
    _safety(raw, "bound timing")
    if raw.get("boundary") != "input_start_to_drain_end":
        raise _error("TIMING_BOUNDARY", "bound timing boundary invalid")
    try:
        start = int(raw["input_start_monotonic_ns"]); end = int(raw["drain_end_monotonic_ns"])
        duration = float(raw["duration_seconds"]); sensor = float(raw["sensor_duration_seconds"]); rtf = float(raw["online_compute_rtf"])
    except (KeyError, TypeError, ValueError) as exc:
        raise _error("TIMING_FIELDS", "bound timing numeric fields invalid") from exc
    if start <= 0 or end <= start or not all(math.isfinite(item) for item in (duration, sensor, rtf)) or \
            duration < 0.0 or sensor != expected["sensor_duration_seconds"] or rtf < 0.0 or \
            abs(duration - (end - start) / 1e9) > 1e-9 or abs(rtf - duration / sensor) > 1e-12:
        raise _error("TIMING_FIELDS", "bound timing monotonic/duration/RTF proof invalid")
    summary = {
        "start_monotonic_ns": start, "end_monotonic_ns": end, "duration_seconds": duration,
        "sensor_duration_seconds": sensor, "online_compute_rtf": rtf,
    }
    if dict(validation) != summary:
        raise _error("BOUND_VALIDATION", "bound timing validation summary drifted")
    return summary


def _atomic_create(path: Path, value: Mapping[str, Any]) -> str:
    _reject_symlink_components(path, "output")
    if path.exists() or path.is_symlink() or path.name.endswith(".part"):
        raise _error("OUTPUT_OVERWRITE", f"output already exists: {path}")
    atomic_write_json(path, dict(value))
    path.chmod(0o444)
    return file_sha256(path)


def _invalid_document(profile_path: Path, bound_path: Path, error: BaseException) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "contract_id": COMPOSED_CONTRACT,
        "status": "invalid",
        "failure_kind": type(error).__name__,
        "failure_reason": str(error),
        "profile_path": str(profile_path.resolve()),
        "bound_path": str(bound_path.resolve()),
        "transport": {"status": "invalid"},
        "terminal_support_context": {"status": "invalid"},
        "timing": {"status": "invalid"},
        "safety": {
            "bag_opened": False, "ground_truth_content_opened": False,
            "scorer_invoked": False, "map_saved": False, "formal_replay_started": False,
        },
    }


def _compose_validated(profile_path: Path, bound_path: Path, expected_profile_sha256: Optional[str]) -> dict[str, Any]:
    _, expected, profile_sha = _profile(profile_path, expected_profile_sha256)
    bound = _json(bound_path, "bound evidence")
    _validate_bound_shape(bound, profile_path, expected)
    if bound["profile"]["sha256"] != profile_sha:
        raise _error("BOUND_PROFILE", "bound profile SHA differs from pinned profile")
    sections = _validate_sources(bound, bound_path, profile_path)
    feeder = _validate_feeder(sections["feeder"]["raw"], sections["feeder"]["validation"], expected)
    transport = _validate_transport(sections["transport"]["raw"], sections["transport"]["validation"], expected)
    terminal = _validate_terminal(sections["terminal_support_context"]["raw"], sections["terminal_support_context"]["validation"], feeder, expected)
    timing = _validate_timing(sections["timing"]["raw"], sections["timing"]["validation"], expected)
    source_refs = {
        name: {"path": section["path"], "sha256": section["sha256"]}
        for name, section in sections.items()
    }
    return {
        "schema_version": 1,
        "contract_id": COMPOSED_CONTRACT,
        "status": "pass",
        "profile": {"path": str(profile_path.resolve()), "sha256": profile_sha, "key": PROFILE_KEY},
        "input": {
            "bag_path": expected["bag_path"], "bag_bytes": expected["bag_bytes"],
            "bag_sha256": expected["bag_sha256"], "expected_messages": expected["expected_messages"],
            "expected_topic_counts": dict(expected["expected_topic_counts"]),
            "required_end_timestamp_seconds": expected["required_end_timestamp_seconds"],
        },
        "feeder": {
            "path": sections["feeder"]["path"], "sha256": sections["feeder"]["sha256"],
            "expected_topic_counts": dict(feeder["expected"]), "published_topic_counts": dict(feeder["published"]),
            "acknowledged_topic_counts": dict(feeder["acknowledged"]), "published_messages": feeder["messages"],
            "single_inflight": True,
        },
        "transport": {
            "path": sections["transport"]["path"], "sha256": sections["transport"]["sha256"],
            "contract_version": CONSUMER_CONTRACT, "transport_contract_version": TRANSPORT_CONTRACT,
            "status": "pass", "received_topic_counts": dict(transport["received_topic_counts"]),
            "received_messages": transport["received_messages"], "acked_messages": transport["acked_messages"],
            "transport_outstanding_at_drain": transport["transport_outstanding_at_drain"],
            "maximum_transport_outstanding_messages": transport["maximum_transport_outstanding_messages"],
            "mapper_internal_deque_current_messages": transport["mapper_internal_deque_current_messages"],
            "mapper_internal_deque_peak_messages": transport["mapper_internal_deque_peak_messages"],
            "ground_truth_content_opened": False, "scorer_invoked": False,
        },
        "terminal_support_context": {
            "path": sections["terminal_support_context"]["path"], "sha256": sections["terminal_support_context"]["sha256"],
            **terminal,
        },
        "timing": {
            "path": sections["timing"]["path"], "sha256": sections["timing"]["sha256"],
            "status": "PASS", "boundary": "input_start_to_drain_end", **timing,
            "after_transport_and_terminal_validation": True,
            "ground_truth_content_opened": False, "scorer_invoked": False,
        },
        "source_bindings": source_refs,
        "safety": {
            "bag_opened": False, "ground_truth_content_opened": False,
            "scorer_invoked": False, "map_saved": False, "formal_replay_started": False,
        },
        "validation": {
            "status": "pass", "transport_status": "pass", "terminal_status": "pass",
            "timing_status": "PASS", "source_hashes_verified": True,
            "profile_input_verified": True, "v5_terminal_validator": "pass",
        },
    }


def compose(
    *, profile_path: Path = PROFILE_PATH, bound_path: Optional[Path] = None,
    output_path: Optional[Path] = None,
    expected_profile_sha256: Optional[str] = PROFILE_SHA256,
    bound_evidence_path: Optional[Path] = None,
) -> dict[str, Any]:
    """Validate one bound document and optionally publish an immutable result.

    Validation errors remain exceptions for fail-closed callers.  When an
    output path is supplied, a separate immutable ``status=invalid`` receipt
    is published before the exception is re-raised; an existing output is
    never overwritten.
    """
    selected_bound = bound_path if bound_path is not None else bound_evidence_path
    if selected_bound is None:
        raise _error("BOUND_INPUT", "bound evidence path is required")
    profile_path = Path(profile_path)
    selected_bound = Path(selected_bound)
    try:
        result = _compose_validated(profile_path, selected_bound, expected_profile_sha256)
    except (PhaseContractError, OSError, ValueError) as exc:
        if output_path is not None:
            _atomic_create(Path(output_path), _invalid_document(profile_path, selected_bound, exc))
        if isinstance(exc, PhaseContractError):
            raise
        raise _error("COMPOSITION_REJECT", str(exc)) from exc
    if output_path is not None:
        output = Path(output_path)
        _atomic_create(output, result)
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    parser.add_argument("--profile-sha256", default=PROFILE_SHA256)
    parser.add_argument("--bound", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _parser().parse_args(argv)
    result = compose(profile_path=args.profile, bound_path=args.bound,
                     output_path=args.output, expected_profile_sha256=args.profile_sha256)
    print(json.dumps({"status": result["status"], "contract_id": result["contract_id"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PhaseContractError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"FAST-LIVO2 v12 evidence composition failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
