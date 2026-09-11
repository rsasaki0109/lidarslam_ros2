#!/usr/bin/env python3
"""Bind v12 feeder, callback, terminal, and timing evidence.

This is a host-only binder.  It never opens a bag, starts Docker/ROS, reads
ground truth, or invokes a scorer.  Mapper documents remain raw observations;
the binder records their paths and hashes and constructs a separate bound
document.  Terminal validation is performed on an in-memory adapter only, so
authority fields are never injected into the raw terminal file.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
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

RAW_AUTHORITY_FIELDS = frozenset({
    "input", "profile_sha256", "profile_path", "raw_path", "raw_sha256",
    "expected", "expected_messages", "expected_topic_counts", "published",
    "published_messages", "published_topic_counts", "ack", "acked",
    "acked_messages", "acked_topic_counts", "acknowledged",
    "acknowledged_messages", "acknowledged_topic_counts", "binding",
    "validation", "required_evaluation_end_timestamp_seconds",
})


class BinderError(ValueError):
    """Fail-closed host-binding error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def _error(kind: str, message: str) -> BinderError:
    return BinderError(kind, message)


def _regular(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise _error("SYMLINK_OR_NOT_REGULAR", f"{label} is not a regular file")
    if path.name.endswith(".part"):
        raise _error("STAGING_INPUT", f"{label} is a staging file")


def _reject_symlink_components(path: Path, label: str) -> None:
    current = Path(path.absolute().anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise _error("SYMLINK_REJECTED", f"{label} contains symlink {current}")


def _json(path: Path, label: str) -> tuple[dict[str, Any], bytes, str]:
    _reject_symlink_components(path, label)
    _regular(path, label)
    try:
        raw = path.read_bytes()
        value = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error("JSON_INVALID", f"{label} JSON is invalid") from exc
    if not isinstance(value, dict):
        raise _error("JSON_NOT_OBJECT", f"{label} must be a JSON object")
    return value, raw, hashlib.sha256(raw).hexdigest()


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


def _profile(
    profile_path: Path = PROFILE_PATH,
    expected_profile_sha256: Optional[str] = PROFILE_SHA256,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    _reject_symlink_components(profile_path, "profile")
    _regular(profile_path, "profile")
    observed_sha = file_sha256(profile_path)
    if expected_profile_sha256 is not None and observed_sha != expected_profile_sha256:
        raise _error("PROFILE_DRIFT", "v12 profile SHA-256 differs from pin")
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
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
        raise _error("PROFILE_INVALID", "v12 profile input/phase is missing")
    expected_counts = _counts(input_value.get("expected_topic_counts"), "profile.expected_topic_counts")
    if (input_value.get("path") != "/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag" or
            input_value.get("bytes") != 11290464091 or
            input_value.get("sha256") != "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310" or
            input_value.get("expected_messages") != 236687 or
            expected_counts != {"lidar": 5793, "imu": 225102, "image": 5792}):
        raise _error("PROFILE_INPUT_DRIFT", "v12 profile input identity differs from pin")
    if phase.get("schema_version") != PROFILE_SCHEMA or phase.get("contract_version") != CONSUMER_CONTRACT or \
            phase.get("mode") != "unpaced_ack" or phase.get("maximum_end_gap_seconds") != 0.25 or \
            phase.get("required_evaluation_end_timestamp_seconds") != 1623491515.148352:
        raise _error("PROFILE_PHASE_INVALID", "v12 profile phase contract differs from pin")
    execution = profile.get("execution")
    if not isinstance(execution, dict) or execution.get("status") != "build_not_run" or \
            execution.get("formal_replay_started") is not False:
        raise _error("PROFILE_EXECUTION_INVALID", "v12 profile is not build-not-run")
    expected = {
        "bag_path": input_value["path"], "bag_bytes": input_value["bytes"],
        "bag_sha256": input_value["sha256"], "expected_messages": input_value["expected_messages"],
        "expected_topic_counts": expected_counts,
        "required_end_timestamp_seconds": float(phase["required_evaluation_end_timestamp_seconds"]),
        "maximum_end_gap_seconds": float(phase["maximum_end_gap_seconds"]),
        "sensor_duration_seconds": 579.278127298,
    }
    return document, expected, observed_sha


def _reject_authority(value: Mapping[str, Any], label: str) -> None:
    present = sorted(key for key in value if isinstance(key, str) and key.lower() in RAW_AUTHORITY_FIELDS)
    if present:
        raise _error("RAW_AUTHORITY_FIELDS", f"{label} contains authority fields: {present}")
    for key in value:
        if not isinstance(key, str):
            raise _error("RAW_AUTHORITY_FIELDS", f"{label} has a non-string key")
        lowered = key.lower()
        if lowered.startswith(("expected_", "published_", "ack_", "acked_", "acknowledged_")):
            raise _error("RAW_AUTHORITY_FIELDS", f"{label} contains authority field {key}")


def _safety(value: Mapping[str, Any], label: str) -> None:
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise _error("SAFETY_FLAGS", f"{label} GT/scorer flags are not false")


def _validate_feeder(value: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    if value.get("schema_version") != 1 or value.get("contract_id") != FEEDER_CONTRACT or value.get("status") != "pass":
        raise _error("FEEDER_CONTRACT", "feeder schema/contract/status invalid")
    if value.get("bag_path") != expected["bag_path"] or value.get("bag_bytes") != expected["bag_bytes"] or value.get("bag_sha256") != expected["bag_sha256"]:
        raise _error("FEEDER_INPUT_IDENTITY", "feeder bag identity mismatch")
    counts = expected["expected_topic_counts"]
    if _counts(value.get("expected_topic_counts"), "feeder.expected") != counts or \
            _counts(value.get("published_topic_counts"), "feeder.published") != counts or \
            _counts(value.get("acked_topic_counts"), "feeder.acked") != counts:
        raise _error("FEEDER_COUNTS", "feeder expected/published/ACK counts mismatch")
    if value.get("published_messages") != expected["expected_messages"] or value.get("single_inflight") is not True or \
            value.get("publisher_queue_size") != 1 or value.get("ack_backpressure_verified") is not True:
        raise _error("FEEDER_TRANSACTION", "feeder single-inflight proof invalid")
    if value.get("duplicate") is True or value.get("stale") is True or value.get("duplicate_of") is not None:
        raise _error("FEEDER_DUPLICATE", "feeder receipt is duplicate or stale")
    _safety(value, "feeder")
    return {"expected": dict(counts), "published": dict(counts), "acknowledged": dict(counts), "messages": expected["expected_messages"]}


def _validate_callback(value: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_authority(value, "callback raw")
    if value.get("schema_version") != 3 or value.get("contract_version") != CONSUMER_CONTRACT or \
            value.get("transport_contract_version") != TRANSPORT_CONTRACT or value.get("phase_mode") != "unpaced_ack" or \
            value.get("status") != "pass" or value.get("system") != PROFILE_SYSTEM:
        raise _error("CALLBACK_CONTRACT", "callback schema/phase/transport/status invalid")
    _safety(value, "callback")
    ledger = value.get("consumer")
    if not isinstance(ledger, dict):
        raise _error("CALLBACK_LEDGER", "callback consumer ledger missing")
    counts = expected["expected_topic_counts"]
    if _counts(ledger.get("received_topic_counts"), "callback.received") != counts or \
            ledger.get("received_messages") != expected["expected_messages"] or ledger.get("acked_messages") != expected["expected_messages"]:
        raise _error("CALLBACK_COUNTS", "callback received/ACK counts mismatch")
    if ledger.get("ack_exact") is not True or ledger.get("transport_outstanding_at_drain") != 0 or \
            ledger.get("maximum_allowed_transport_outstanding_messages") != 1 or \
            not isinstance(ledger.get("maximum_transport_outstanding_messages"), int) or \
            ledger["maximum_transport_outstanding_messages"] < 0 or ledger["maximum_transport_outstanding_messages"] > 1:
        raise _error("CALLBACK_TRANSPORT", "callback transport outstanding proof invalid")
    for key in ("mapper_internal_deque_current_messages", "mapper_internal_deque_peak_messages"):
        if isinstance(ledger.get(key), bool) or not isinstance(ledger.get(key), int) or ledger[key] < 0:
            raise _error("CALLBACK_DEQUE", f"callback {key} diagnostic invalid")
    if ledger.get("eof_observed") is not True or ledger.get("drain_complete") is not True or \
            ledger.get("dropped_messages") != 0 or ledger.get("queue_overflow") != 0 or \
            ledger.get("processing_failures") != 0 or ledger.get("ack_backpressure_verified") is not True:
        raise _error("CALLBACK_COMPLETION", "callback completion counters invalid")
    return {
        "received_topic_counts": dict(counts), "received_messages": expected["expected_messages"],
        "acked_messages": expected["expected_messages"],
        "transport_outstanding_at_drain": 0,
        "maximum_transport_outstanding_messages": ledger["maximum_transport_outstanding_messages"],
        "mapper_internal_deque_current_messages": ledger["mapper_internal_deque_current_messages"],
        "mapper_internal_deque_peak_messages": ledger["mapper_internal_deque_peak_messages"],
    }


def _terminal_adapter(raw: Mapping[str, Any], feeder: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    adapted = copy.deepcopy(dict(raw))
    adapted["schema_version"] = 3
    adapted["contract_version"] = CONTRACT_VERSION_V5
    adapted["required_evaluation_end_timestamp_seconds"] = expected["required_end_timestamp_seconds"]
    adapted["maximum_end_gap_seconds"] = expected["maximum_end_gap_seconds"]
    received = _counts(raw.get("received_topic_counts"), "terminal.received")
    adapted["counts"] = {
        "expected": dict(expected["expected_topic_counts"]),
        "published": dict(feeder["published"]), "received": received,
        "acknowledged": dict(feeder["acknowledged"]),
    }
    return validate_terminal_support_context_v5(
        adapted, maximum_end_gap_seconds=expected["maximum_end_gap_seconds"], require_pass=True)


def _validate_terminal(value: Mapping[str, Any], feeder: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_authority(value, "terminal raw")
    # The terminal mapper document is schema-1 and therefore must not carry
    # the host-adapted v5 fields.  Validate these fields before constructing
    # the in-memory v5 adapter so a raw document cannot smuggle authority into
    # the host-bound result or have a malformed top-level counter ignored.
    if "contract_version" in value or "counts" in value or "maximum_end_gap_seconds" in value:
        raise _error("RAW_AUTHORITY_FIELDS", "terminal raw contains host v5 authority fields")
    if value.get("schema_version") != 1 or value.get("contract_id") != TERMINAL_RAW_CONTRACT or \
            value.get("status") != "pass" or value.get("phase_mode") != "unpaced_ack" or value.get("system") != PROFILE_SYSTEM:
        raise _error("TERMINAL_CONTRACT", "terminal raw schema/contract/status invalid")
    _safety(value, "terminal")
    _counts(value.get("received_topic_counts"), "terminal.received")
    for field in ("dropped_counts", "overflow_counts"):
        counters = _counts(value.get(field), f"terminal.{field}")
        if any(counters.values()):
            raise _error("TERMINAL_COUNTERS", f"terminal {field} are nonzero")
    failures = value.get("processing_failures")
    if isinstance(failures, bool) or not isinstance(failures, int) or failures != 0:
        raise _error("TERMINAL_COUNTERS", "terminal processing_failures is nonzero or invalid")
    return _terminal_adapter(value, feeder, expected)


def _validate_timing(value: Mapping[str, Any], expected: Mapping[str, Any]) -> dict[str, Any]:
    _reject_authority(value, "timing raw")
    if value.get("schema_version") != 1 or value.get("contract_version") != TIMING_CONTRACT or value.get("status") != "PASS":
        raise _error("TIMING_CONTRACT", "timing schema/contract/status invalid")
    _safety(value, "timing")
    if value.get("boundary") != "input_start_to_drain_end":
        raise _error("TIMING_BOUNDARY", "timing boundary is not input_start_to_drain_end")
    try:
        start = int(value["input_start_monotonic_ns"]); end = int(value["drain_end_monotonic_ns"])
        duration = float(value["duration_seconds"]); sensor = float(value["sensor_duration_seconds"]); rtf = float(value["online_compute_rtf"])
    except (KeyError, TypeError, ValueError) as exc:
        raise _error("TIMING_FIELDS", "timing numeric fields are invalid") from exc
    if start <= 0 or end <= start or not all(math.isfinite(x) for x in (duration, sensor, rtf)) or \
            duration < 0 or sensor != expected["sensor_duration_seconds"] or rtf < 0 or \
            abs(duration - (end - start) / 1e9) > 1e-9:
        raise _error("TIMING_FIELDS", "timing monotonic/duration/RTF proof invalid")
    return {"start_monotonic_ns": start, "end_monotonic_ns": end, "duration_seconds": duration, "sensor_duration_seconds": sensor, "online_compute_rtf": rtf}


def _atomic_create(path: Path, value: Mapping[str, Any]) -> str:
    _reject_symlink_components(path, "output")
    if path.exists() or path.is_symlink() or path.name.endswith(".part"):
        raise _error("OUTPUT_OVERWRITE", f"output already exists: {path}")
    atomic_write_json(path, dict(value))
    path.chmod(0o444)
    return file_sha256(path)


def bind_consumer_evidence(
    feeder_path: Path,
    callback_path: Path,
    terminal_path: Path,
    timing_path: Path,
    output_path: Path,
    profile_path: Path = PROFILE_PATH,
    expected_profile_sha256: Optional[str] = PROFILE_SHA256,
    input_path: Optional[str] = None,
    input_bytes: Optional[int] = None,
    input_sha256: Optional[str] = None,
) -> dict[str, Any]:
    """Validate and atomically bind one v12 evidence set."""
    _, expected, profile_sha = _profile(profile_path, expected_profile_sha256)
    if input_path is not None and input_path != expected["bag_path"]:
        raise _error("INPUT_ARGUMENT_DRIFT", "authoritative input path differs from profile")
    if input_bytes is not None and input_bytes != expected["bag_bytes"]:
        raise _error("INPUT_ARGUMENT_DRIFT", "authoritative input bytes differ from profile")
    if input_sha256 is not None and input_sha256 != expected["bag_sha256"]:
        raise _error("INPUT_ARGUMENT_DRIFT", "authoritative input SHA differs from profile")
    feeder, feeder_bytes, feeder_sha = _json(Path(feeder_path), "feeder")
    callback, callback_bytes, callback_sha = _json(Path(callback_path), "callback")
    terminal, terminal_bytes, terminal_sha = _json(Path(terminal_path), "terminal")
    # Timing is intentionally loaded only after both raw mapper documents are
    # present and validated; it cannot authorize a missing raw observation.
    timing, timing_bytes, timing_sha = _json(Path(timing_path), "timing")
    feeder_summary = _validate_feeder(feeder, expected)
    callback_summary = _validate_callback(callback, expected)
    terminal_summary = _validate_terminal(terminal, feeder_summary, expected)
    timing_summary = _validate_timing(timing, expected)
    document: dict[str, Any] = {
        "schema_version": 1,
        "contract_id": BOUND_CONTRACT,
        "status": "BOUND",
        "profile": {"path": str(Path(profile_path).resolve()), "sha256": profile_sha, "key": PROFILE_KEY},
        "input": {"bag_path": expected["bag_path"], "bag_bytes": expected["bag_bytes"], "bag_sha256": expected["bag_sha256"], "expected_messages": expected["expected_messages"], "expected_topic_counts": dict(expected["expected_topic_counts"]), "required_end_timestamp_seconds": expected["required_end_timestamp_seconds"]},
        "feeder": {"path": str(Path(feeder_path).resolve()), "sha256": feeder_sha, "raw": feeder, "validation": feeder_summary},
        "transport": {"path": str(Path(callback_path).resolve()), "sha256": callback_sha, "raw": callback, "validation": callback_summary},
        "terminal_support_context": {"path": str(Path(terminal_path).resolve()), "sha256": terminal_sha, "raw": terminal, "validation": terminal_summary},
        "timing": {"path": str(Path(timing_path).resolve()), "sha256": timing_sha, "raw": timing, "validation": timing_summary},
        "sources": {"feeder_sha256": feeder_sha, "callback_sha256": callback_sha, "terminal_sha256": terminal_sha, "timing_sha256": timing_sha},
        "safety": {"bag_opened": False, "ground_truth_content_opened": False, "scorer_invoked": False, "map_saved": False, "formal_replay_started": False},
    }
    output_sha = _atomic_create(Path(output_path), document)
    return {"status": "BOUND", "output_path": str(Path(output_path).resolve()), "output_sha256": output_sha, "document": document}


def bind(*args: Any, **kwargs: Any) -> dict[str, Any]:
    return bind_consumer_evidence(*args, **kwargs)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    parser.add_argument("--profile-sha256", default=PROFILE_SHA256)
    parser.add_argument("--feeder", type=Path, required=True)
    parser.add_argument("--callback", type=Path, required=True)
    parser.add_argument("--terminal", type=Path, required=True)
    parser.add_argument("--timing", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-path")
    parser.add_argument("--input-bytes", type=int)
    parser.add_argument("--input-sha256")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _parser().parse_args(argv)
    receipt = bind_consumer_evidence(
        feeder_path=args.feeder, callback_path=args.callback, terminal_path=args.terminal,
        timing_path=args.timing, output_path=args.output, profile_path=args.profile,
        expected_profile_sha256=args.profile_sha256, input_path=args.input_path,
        input_bytes=args.input_bytes, input_sha256=args.input_sha256)
    print(json.dumps({key: value for key, value in receipt.items() if key != "document"}, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BinderError, PhaseContractError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"FAST-LIVO2 v12 evidence binding failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
