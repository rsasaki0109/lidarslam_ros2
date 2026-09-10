#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Common, GT-blind phase evidence contract for competitive replays.

The contract deliberately lives outside any ROS implementation.  Wrappers
only report lifecycle boundaries and an opaque trajectory-coverage proof;
the runner decides whether the resulting online-compute metric is admissible.
All writes are atomic so a killed container cannot leave a plausible PASS
document behind.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any


SCHEMA_VERSION = 1
CONTRACT_VERSION = 'm6a10-online-compute-v1'
ONLINE_RTF_LIMIT = 1.0
DEFAULT_END_GAP_SECONDS = 0.25
SCHEMA_VERSION_V2 = 2
CONTRACT_VERSION_V2 = 'm6a10-online-compute-v2'
PHASE_MODES_V2 = ('paced_1x', 'unpaced_ack')
DEFAULT_CALLBACK_LATENCY_SECONDS_V2 = 0.25
DEFAULT_BACKLOG_MESSAGES_V2 = 0
SCHEMA_VERSION_V3 = 3
CONTRACT_VERSION_V3 = 'm6a10-online-compute-v3-terminal-support-context'
TERMINAL_SUPPORT_CONTEXT_TOPICS_V3 = ('lidar', 'imu', 'image')
TERMINAL_SUPPORT_CONTEXT_ALLOWED_TOPICS_V3 = ('imu', 'image')
CONTRACT_VERSION_V5 = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
TERMINAL_SUPPORT_CONTEXT_TOPICS_V5 = ('lidar', 'imu', 'image')
TERMINAL_SUPPORT_CONTEXT_ALLOWED_TOPICS_V5 = ('imu', 'image')
TERMINAL_SUPPORT_CONTEXT_V5_CLASSIFICATION = (
    'nonlidar_at_or_after_boundary_support_context')
TERMINAL_SUPPORT_CONTEXT_V5_EQUAL_REASON = (
    'equal_to_completed_boundary_nonlidar_support_context')
EVENT_NAMES = (
    'startup_start', 'startup_end', 'input_start', 'input_end',
    'drain_start', 'drain_end', 'postprocess_start', 'postprocess_end',
    'save_start', 'save_end', 'shutdown_start', 'shutdown_end')
ORDERED_EVENTS = EVENT_NAMES
TIME_RE = re.compile(
    r'Elapsed \(wall clock\) time .*?:\s*([0-9:.]+)', re.MULTILINE)


class PhaseContractError(ValueError):
    """Raised when phase evidence is absent, inconsistent, or unsafe."""


def _now_ns() -> int:
    value = time.monotonic_ns()
    if value <= 0:
        raise PhaseContractError('monotonic clock returned a non-positive value')
    return value


def _finite(value: Any, name: str, *, nonnegative: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PhaseContractError(f'{name} must be numeric')
    result = float(value)
    if not math.isfinite(result) or (nonnegative and result < 0):
        raise PhaseContractError(f'{name} must be finite and non-negative')
    return result


def _integer(value: Any, name: str, *, nonnegative: bool = True) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or \
            (nonnegative and value < 0):
        raise PhaseContractError(f'{name} must be a non-negative integer')
    return value


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()


def atomic_write_json(path: Path, value: Any) -> None:
    path = path.resolve(strict=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    if path.exists():
        raise PhaseContractError(f'phase evidence already exists: {path}')
    if part.exists():
        raise PhaseContractError(f'phase evidence staging file exists: {part}')
    with part.open('wb') as stream:
        stream.write(canonical_json(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(part, path)


def atomic_replace_json(path: Path, value: Any) -> None:
    """Atomically update an in-progress event document.

    Final evidence uses :func:`atomic_write_json` and is never overwritten;
    only the private event journal is allowed to advance between markers.
    """
    path = path.resolve(strict=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    if part.exists():
        raise PhaseContractError(f'phase event staging file exists: {part}')
    with part.open('wb') as stream:
        stream.write(canonical_json(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(part, path)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def new_events(system: str, input_duration_seconds: float,
               *, max_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
               contract_version: str = CONTRACT_VERSION,
               phase_mode: str = 'paced_1x') -> dict[str, Any]:
    if system not in {'ours', 'glim_cpu', 'fast_livo2'}:
        raise PhaseContractError(f'unknown system: {system}')
    duration = _finite(input_duration_seconds, 'input_duration_seconds')
    if duration <= 0:
        raise PhaseContractError('input_duration_seconds must be positive')
    if contract_version not in {CONTRACT_VERSION, CONTRACT_VERSION_V2}:
        raise PhaseContractError(
            f'unsupported phase contract version: {contract_version}')
    if phase_mode not in PHASE_MODES_V2:
        raise PhaseContractError(f'unsupported phase mode: {phase_mode}')
    now = _now_ns()
    events = {
        'schema_version': (SCHEMA_VERSION_V2 if contract_version == CONTRACT_VERSION_V2
                           else SCHEMA_VERSION),
        'contract_version': contract_version,
        'system': system,
        'created_at_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'run_start_monotonic_ns': now,
        'input_duration_seconds': duration,
        'maximum_trajectory_end_gap_seconds': _finite(
            max_end_gap_seconds, 'maximum_trajectory_end_gap_seconds'),
        'events_monotonic_ns': {
            name: (now if name == 'startup_start' else None)
            for name in EVENT_NAMES},
        'coverage': {
            'status': 'unverified', 'mode': None, 'complete': False,
            'expected_messages': None, 'consumed_messages': None,
            'dropped_messages': None, 'queue_overflow': None,
            'last_input_timestamp_seconds': None,
            'required_end_timestamp_seconds': None,
            'end_gap_seconds': None,
        },
        'resource': {
            'status': 'unavailable', 'scope': None,
            'cpu_user_seconds': None, 'cpu_system_seconds': None,
            'io_input_operations': None, 'io_output_operations': None,
            'source': None},
        'exit_status': None,
    }
    if contract_version == CONTRACT_VERSION_V2:
        events['phase_mode'] = phase_mode
        events['consumer'] = {
            'ack_source_kind': None,
            'ack_source': None,
            'ack_semantics': None,
            'publisher_count_used': None,
            'eof_observed': False,
            'eof_source': None,
            'expected_messages': None,
            'expected_topic_counts': None,
            'received_messages': None,
            'processed_messages': None,
            'dropped_messages': None,
            'queue_overflow': None,
            'backlog_at_drain': None,
            'maximum_backlog_messages': None,
            'maximum_registration_queue_messages': None,
            'first_processed_timestamp_seconds': None,
            'last_processed_timestamp_seconds': None,
            'required_end_timestamp_seconds': None,
            'maximum_callback_latency_seconds': None,
            'paced_input_rate': None,
            'paced_input_rate_verified': False,
            'ack_backpressure_verified': False,
            'single_message_buffer_verified': False,
            'pacing_late_messages': None,
            'processing_failures': None,
            'drain_complete': None,
            'counter_evidence_path': None,
        }
    return events


def load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PhaseContractError(f'cannot read phase events: {path}: {error}') from error
    if not isinstance(value, dict):
        raise PhaseContractError('phase events must be a JSON object')
    return value


def mark_event(events: dict[str, Any], name: str, stamp_ns: int | None = None) -> None:
    if name not in EVENT_NAMES:
        raise PhaseContractError(f'unknown phase event: {name}')
    values = events.get('events_monotonic_ns')
    if not isinstance(values, dict) or values.get(name) is not None:
        raise PhaseContractError(f'phase event already marked or missing: {name}')
    stamp = _now_ns() if stamp_ns is None else _integer(stamp_ns, name)
    values[name] = stamp


def set_coverage(events: dict[str, Any], *, mode: str,
                 complete: bool, last_timestamp: float | None,
                 required_timestamp: float | None,
                 dropped_messages: int | None = None,
                 queue_overflow: int | None = None,
                 expected_messages: int | None = None,
                 consumed_messages: int | None = None) -> None:
    if mode not in {'trajectory_timestamp_coverage', 'processed_message_counts'}:
        raise PhaseContractError(f'unsupported coverage mode: {mode}')
    if not isinstance(complete, bool):
        raise PhaseContractError('coverage complete must be boolean')
    coverage = events.get('coverage')
    if not isinstance(coverage, dict):
        raise PhaseContractError('coverage object missing')
    for value, name in ((dropped_messages, 'dropped_messages'),
                        (queue_overflow, 'queue_overflow'),
                        (expected_messages, 'expected_messages'),
                        (consumed_messages, 'consumed_messages')):
        if value is not None:
            _integer(value, name)
    for value, name in ((last_timestamp, 'last_input_timestamp_seconds'),
                        (required_timestamp, 'required_end_timestamp_seconds')):
        if value is not None:
            _finite(value, name)
    gap = None
    if last_timestamp is not None and required_timestamp is not None:
        gap = float(required_timestamp) - float(last_timestamp)
        if not math.isfinite(gap):
            raise PhaseContractError('coverage end gap is not finite')
    coverage.update({
        'status': 'verified' if complete else 'invalid',
        'mode': mode, 'complete': complete,
        'expected_messages': expected_messages,
        'consumed_messages': consumed_messages,
        'dropped_messages': dropped_messages,
        'queue_overflow': queue_overflow,
        'last_input_timestamp_seconds': last_timestamp,
        'required_end_timestamp_seconds': required_timestamp,
        'end_gap_seconds': gap,
    })


def _elapsed_report(value: str) -> float:
    fields = [float(item) for item in value.split(':')]
    if not 1 <= len(fields) <= 3:
        raise ValueError(value)
    return sum(field * (60 ** index)
               for index, field in enumerate(reversed(fields)))


def read_time_report(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {'status': 'unavailable', 'scope': None, 'source': str(path),
                'cpu_user_seconds': None, 'cpu_system_seconds': None,
                'io_input_operations': None, 'io_output_operations': None}
    text = path.read_text(encoding='utf-8', errors='replace')

    def number(pattern: str) -> float | None:
        match = re.search(pattern, text)
        if not match:
            return None
        try:
            result = float(match.group(1))
        except ValueError:
            return None
        return result if math.isfinite(result) and result >= 0 else None
    integer = re.search(r'File system inputs:\s*(\d+)', text)
    output = re.search(r'File system outputs:\s*(\d+)', text)
    values = {
        'cpu_user_seconds': number(r'User time \(seconds\):\s*([0-9.eE+-]+)'),
        'cpu_system_seconds': number(r'System time \(seconds\):\s*([0-9.eE+-]+)'),
        # GNU time reports filesystem input/output operations, not bytes.
        'io_input_operations': int(integer.group(1)) if integer else None,
        'io_output_operations': int(output.group(1)) if output else None,
    }
    required = ('cpu_user_seconds', 'cpu_system_seconds',
                'io_input_operations', 'io_output_operations')
    status = 'verified' if all(values[key] is not None for key in required) else 'invalid'
    values.update({'status': status, 'scope': 'wrapper_process', 'source': str(path)})
    return values


def _validate_coverage(coverage: Any, max_gap: float) -> tuple[bool, str]:
    if not isinstance(coverage, dict):
        return False, 'coverage_missing'
    if coverage.get('status') != 'verified' or coverage.get('complete') is not True:
        return False, 'coverage_not_verified'
    mode = coverage.get('mode')
    if mode == 'trajectory_timestamp_coverage':
        for key in ('last_input_timestamp_seconds', 'required_end_timestamp_seconds',
                    'end_gap_seconds'):
            try:
                _finite(coverage.get(key), f'coverage.{key}', nonnegative=False)
            except PhaseContractError:
                return False, f'coverage_{key}_invalid'
        if coverage['end_gap_seconds'] > max_gap:
            return False, 'trajectory_end_gap_exceeds_limit'
    elif mode == 'processed_message_counts':
        expected = coverage.get('expected_messages')
        consumed = coverage.get('consumed_messages')
        try:
            _integer(expected, 'coverage.expected_messages')
            _integer(consumed, 'coverage.consumed_messages')
        except PhaseContractError:
            return False, 'message_counts_missing'
        if expected != consumed:
            return False, 'message_counts_mismatch'
    else:
        return False, 'coverage_mode_invalid'
    for key in ('dropped_messages', 'queue_overflow'):
        try:
            value = _integer(coverage.get(key), f'coverage.{key}')
        except PhaseContractError:
            return False, f'{key}_missing'
        if value != 0:
            return False, f'{key}_nonzero'
    return True, ''


def set_consumer_evidence(events: dict[str, Any], evidence: dict[str, Any],
                          *, source_path: str | None = None) -> None:
    """Attach application-owned consumer acknowledgements to a v2 journal.

    The producer must be the implementation's processing callback/ack path.
    Publisher-side counts are explicitly rejected so a bag publisher cannot be
    mistaken for a consumer.  The application counts are also promoted to
    the v2 coverage view.  This is an explicit source transition, not an
    inference: the implementation callback is the authoritative processed
    boundary, while the consumer validator independently checks timestamps,
    EOF, and acknowledgement semantics.  Missing or malformed counts stay
    missing so final validation remains fail-closed; ``None`` is never
    converted to zero.
    """
    if events.get('schema_version') != SCHEMA_VERSION_V2 or \
            events.get('contract_version') != CONTRACT_VERSION_V2:
        raise PhaseContractError('consumer evidence requires phase contract v2')
    if not isinstance(evidence, dict):
        raise PhaseContractError('consumer evidence must be an object')
    consumer = evidence.get('consumer', evidence)
    if not isinstance(consumer, dict):
        raise PhaseContractError('consumer evidence object is missing')
    target = events.get('consumer')
    if not isinstance(target, dict):
        raise PhaseContractError('phase consumer object is missing')
    fields = tuple(target)
    copied = {key: consumer.get(key) for key in fields}
    if source_path is not None:
        copied['counter_evidence_path'] = source_path
    events['consumer'] = copied

    # A trajectory writer can lag the application callback (or be absent in a
    # short fixture), so its timestamp coverage cannot prove received,
    # processed, or dropped-message counts.  When the producer supplies the
    # complete integer counter set, make that provenance explicit in the
    # shared coverage contract.  Contradictory values are rejected instead
    # of silently overwritten.
    coverage = events.get('coverage')
    counter_keys = {
        'expected_messages': consumer.get('expected_messages'),
        'consumed_messages': consumer.get('processed_messages'),
        'dropped_messages': consumer.get('dropped_messages'),
        'queue_overflow': consumer.get('queue_overflow'),
    }
    if isinstance(coverage, dict) and all(
            isinstance(value, int) and not isinstance(value, bool)
            for value in counter_keys.values()):
        for key, value in counter_keys.items():
            existing = coverage.get(key)
            if existing is not None and existing != value:
                raise PhaseContractError(
                    f'consumer coverage conflicts with {key}')
        coverage.update({
            'status': ('verified' if consumer.get('eof_observed') is True
                       else 'invalid'),
            'mode': 'processed_message_counts',
            'complete': consumer.get('eof_observed') is True,
            **counter_keys,
            'consumer_counter_evidence': source_path or
            consumer.get('counter_evidence_path'),
        })


def _validate_consumer_v2(consumer: Any, phase_mode: Any, *,
                          maximum_end_gap_seconds: float,
                          maximum_callback_latency_seconds: float,
                          maximum_backlog_messages: int) -> tuple[bool, str, dict[str, Any]]:
    """Validate callback acknowledgement, EOF, backlog and pacing evidence."""
    if not isinstance(consumer, dict):
        return False, 'consumer_evidence_missing', {}
    if phase_mode not in PHASE_MODES_V2:
        return False, 'phase_mode_invalid', {'phase_mode': phase_mode}
    required_strings = ('ack_source_kind', 'ack_source', 'ack_semantics', 'eof_source',
                        'counter_evidence_path')
    for key in required_strings:
        value = consumer.get(key)
        if not isinstance(value, str) or not value.strip():
            return False, f'consumer.{key}_missing', {}
    if consumer.get('ack_source_kind') not in {
            'consumer_callback', 'synchronous_processing'}:
        return False, 'consumer_ack_source_is_not_processing_boundary', {}
    if consumer.get('publisher_count_used') is not False:
        return False, 'publisher_count_cannot_prove_consumer_processing', {}
    if consumer.get('eof_observed') is not True:
        return False, 'consumer_eof_missing', {}
    integer_fields = ('expected_messages', 'received_messages', 'processed_messages',
                      'dropped_messages', 'queue_overflow', 'backlog_at_drain',
                      'maximum_backlog_messages')
    integers: dict[str, int] = {}
    for key in integer_fields:
        try:
            integers[key] = _integer(consumer.get(key), f'consumer.{key}')
        except PhaseContractError:
            return False, f'consumer.{key}_missing_or_invalid', {}
    if not (integers['expected_messages'] == integers['received_messages'] ==
            integers['processed_messages']):
        return False, 'consumer_message_counts_not_equal', integers
    for key in ('dropped_messages', 'queue_overflow'):
        if integers[key] != 0:
            return False, f'consumer_{key}_nonzero', integers
    if integers['backlog_at_drain'] != 0:
        return False, 'consumer_backlog_not_empty_at_drain', integers
    if integers['maximum_backlog_messages'] > maximum_backlog_messages:
        return False, 'consumer_backlog_bound_exceeded', {
            **integers, 'maximum_allowed_backlog_messages': maximum_backlog_messages}
    timestamp_fields = (
        'first_processed_timestamp_seconds',
        'last_processed_timestamp_seconds',
        'required_end_timestamp_seconds')
    timestamps: dict[str, float] = {}
    for key in timestamp_fields:
        try:
            timestamps[key] = _finite(consumer.get(key), f'consumer.{key}',
                                      nonnegative=False)
        except PhaseContractError:
            return False, f'consumer.{key}_missing_or_invalid', integers
    end_gap = timestamps['required_end_timestamp_seconds'] - \
        timestamps['last_processed_timestamp_seconds']
    if not math.isfinite(end_gap) or end_gap > maximum_end_gap_seconds:
        return False, 'consumer_timestamp_coverage_exceeds_limit', {
            **integers, 'end_gap_seconds': end_gap}
    try:
        callback_latency = _finite(
            consumer.get('maximum_callback_latency_seconds'),
            'consumer.maximum_callback_latency_seconds')
    except PhaseContractError:
        return False, 'consumer_callback_latency_missing_or_invalid', integers
    if callback_latency > maximum_callback_latency_seconds:
        return False, 'consumer_callback_latency_exceeds_limit', {
            **integers, 'maximum_callback_latency_seconds': callback_latency}
    paced_rate = consumer.get('paced_input_rate')
    paced_verified = consumer.get('paced_input_rate_verified') is True
    ack_backpressure = consumer.get('ack_backpressure_verified') is True
    if phase_mode == 'paced_1x':
        try:
            if _finite(paced_rate, 'consumer.paced_input_rate') != 1.0:
                return False, 'paced_input_rate_is_not_one', {}
        except PhaseContractError:
            return False, 'paced_input_rate_missing_or_invalid', {}
        if not paced_verified:
            return False, 'paced_input_rate_not_verified', {}
    else:
        if not ack_backpressure:
            return False, 'ack_backpressure_not_verified', {}
        # A synchronous callback-return boundary is the acknowledgement
        # contract.  BufferableBag may use a bounded disk/decompression
        # prefetch window; capacity=1 is optional and must not become a
        # throughput gate.  If present, the diagnostic flag must remain a
        # boolean so malformed evidence still fails closed.
        single_message_buffer = consumer.get('single_message_buffer_verified')
        if single_message_buffer is not None and not isinstance(
                single_message_buffer, bool):
            return False, 'single_message_buffer_flag_invalid', {}
    return True, '', {**integers, **timestamps,
                      'end_gap_seconds': end_gap,
                      'maximum_callback_latency_seconds': callback_latency,
                      'paced_input_rate_verified': paced_verified,
                      'ack_backpressure_verified': ack_backpressure}


def validate_consumer_evidence_v2(
        value: dict[str, Any], *,
        maximum_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
        maximum_callback_latency_seconds: float =
        DEFAULT_CALLBACK_LATENCY_SECONDS_V2,
        maximum_backlog_messages: int = DEFAULT_BACKLOG_MESSAGES_V2) -> dict[str, Any]:
    """Validate the application-owned v2 evidence before map post-processing.

    This is intentionally stricter than the phase journal validator's
    ``consumer`` attachment step.  A timeout/invalid evidence document may
    still be useful diagnostic output, but it can never be a completion proof.
    The standalone CLI is used by the shell runner at the map-save barrier.
    """
    if value.get('schema_version') != SCHEMA_VERSION_V2 or \
            value.get('contract_version') != CONTRACT_VERSION_V2:
        raise PhaseContractError('consumer evidence schema/version mismatch')
    if value.get('status') != 'pass':
        raise PhaseContractError('consumer evidence status is not pass')
    consumer = value.get('consumer', value)
    if not isinstance(consumer, dict):
        raise PhaseContractError('consumer evidence object is missing')
    if consumer.get('drain_complete') is not True:
        raise PhaseContractError('consumer evidence drain is not complete')
    processing_failures = consumer.get('processing_failures')
    if processing_failures != 0:
        raise PhaseContractError('consumer processing failures are nonzero')
    ok, reason, details = _validate_consumer_v2(
        consumer, value.get('phase_mode'),
        maximum_end_gap_seconds=maximum_end_gap_seconds,
        maximum_callback_latency_seconds=maximum_callback_latency_seconds,
        maximum_backlog_messages=maximum_backlog_messages)
    if not ok:
        raise PhaseContractError(reason)
    return {
        'schema_version': SCHEMA_VERSION_V2,
        'contract_version': CONTRACT_VERSION_V2,
        'phase_mode': value.get('phase_mode'),
        'status': 'pass',
        'validation': details,
    }


def _v3_topic_counts(value: Any, name: str) -> dict[str, int]:
    """Read an exact lidar/IMU/image count map for the v3 contract."""
    if not isinstance(value, dict) or set(value) != set(
            TERMINAL_SUPPORT_CONTEXT_TOPICS_V3):
        raise PhaseContractError(f'{name}_missing_or_invalid')
    result: dict[str, int] = {}
    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V3:
        try:
            result[topic] = _integer(value.get(topic), f'{name}.{topic}')
        except PhaseContractError as error:
            raise PhaseContractError(f'{name}_{topic}_missing_or_invalid') \
                from error
    return result


def _v3_forbidden_record_label(record: dict[str, Any]) -> bool:
    """Return whether a residual record is mislabeled as processed/dropped."""
    return any(key in record for key in (
        'processed', 'dropped', 'processed_count', 'dropped_count',
        'processed_messages', 'dropped_messages'))


def _v3_validate_support_record(
        record: Any, topic: str, boundary_timestamp: float) -> None:
    """Validate one residual record without assigning it a processing state."""
    if not isinstance(record, dict):
        raise PhaseContractError('support_context_record_invalid')
    if _v3_forbidden_record_label(record):
        raise PhaseContractError(
            'support_context_record_has_forbidden_processing_label')
    if record.get('topic') != topic:
        raise PhaseContractError('support_context_record_topic_mismatch')
    record_id = record.get('record_id')
    if not isinstance(record_id, str) or not record_id.strip():
        raise PhaseContractError('support_context_record_id_missing_or_invalid')
    try:
        timestamp = _finite(record.get('timestamp_seconds'),
                            f'{topic} support context timestamp',
                            nonnegative=False)
    except PhaseContractError as error:
        raise PhaseContractError(
            f'buffer_{topic}_record_timestamp_missing_or_invalid') from error
    if timestamp <= boundary_timestamp:
        raise PhaseContractError('support_context_record_not_strictly_post_boundary')
    if record.get('support_context_proven') is not True:
        raise PhaseContractError('support_context_record_not_proven')
    if record.get('post_boundary') is not True:
        raise PhaseContractError('support_context_record_not_post_boundary')
    if record.get('sync_predicate_evaluated') is not True:
        raise PhaseContractError('support_context_sync_predicate_unproven')
    if record.get('can_form_synchronization_unit') is not False:
        raise PhaseContractError(
            'support_context_record_can_form_synchronization_unit')
    if record.get('reason_code') != 'strictly_after_completed_boundary':
        raise PhaseContractError('support_context_record_reason_invalid')


def _v3_validate_buffer(
        topic: str, value: Any, boundary_timestamp: float
        ) -> tuple[int, list[dict[str, Any]], dict[str, Any]]:
    """Validate one per-topic terminal buffer and return its records."""
    if not isinstance(value, dict):
        raise PhaseContractError(f'buffer_{topic}_missing_or_invalid')
    try:
        count = _integer(value.get('count'), f'buffer.{topic}.count')
    except PhaseContractError as error:
        raise PhaseContractError(
            f'buffer_{topic}_count_missing_or_invalid') from error
    records = value.get('records')
    if not isinstance(records, list):
        raise PhaseContractError(f'buffer_{topic}_records_missing_or_invalid')
    if len(records) != count:
        raise PhaseContractError(f'buffer_{topic}_count_does_not_match_records')
    timestamps: list[float] = []
    ids: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise PhaseContractError(f'buffer_{topic}_record_invalid')
        record_id = record.get('record_id')
        if not isinstance(record_id, str) or not record_id.strip():
            raise PhaseContractError(f'buffer_{topic}_record_id_missing_or_invalid')
        if record_id in ids:
            raise PhaseContractError(f'buffer_{topic}_record_id_duplicate')
        ids.add(record_id)
        try:
            timestamps.append(_finite(
                record.get('timestamp_seconds'),
                f'buffer.{topic}.record.timestamp_seconds',
                nonnegative=False))
        except PhaseContractError as error:
            raise PhaseContractError(
                f'buffer_{topic}_record_timestamp_missing_or_invalid') from error
        if record.get('topic') != topic:
            raise PhaseContractError(f'buffer_{topic}_record_topic_mismatch')
    oldest = value.get('oldest_timestamp_seconds')
    newest = value.get('newest_timestamp_seconds')
    if not records:
        if oldest is not None or newest is not None:
            raise PhaseContractError(f'buffer_{topic}_empty_timestamp_invalid')
    else:
        try:
            oldest_value = _finite(
                oldest, f'buffer.{topic}.oldest_timestamp_seconds',
                nonnegative=False)
            newest_value = _finite(
                newest, f'buffer.{topic}.newest_timestamp_seconds',
                nonnegative=False)
        except PhaseContractError as error:
            raise PhaseContractError(
                f'buffer_{topic}_timestamp_bounds_missing_or_invalid') from error
        if not math.isclose(oldest_value, min(timestamps), rel_tol=0.0,
                            abs_tol=1e-12) or not math.isclose(
                                newest_value, max(timestamps), rel_tol=0.0,
                                abs_tol=1e-12):
            raise PhaseContractError(f'buffer_{topic}_timestamp_bounds_mismatch')
    if topic == 'lidar' and records:
        # Lidar residuals can participate in a future synchronization unit;
        # they are never terminal support context, even if timestamped.
        raise PhaseContractError('residual_lidar_not_support_context')
    for record in records:
        _v3_validate_support_record(record, topic, boundary_timestamp)
    return count, records, {
        'count': count,
        'oldest_timestamp_seconds': oldest if records else None,
        'newest_timestamp_seconds': newest if records else None,
    }


def _validate_terminal_support_context_v3(
        value: dict[str, Any], *, maximum_end_gap_seconds: float
        ) -> dict[str, Any]:
    """Validate the explicit v3 terminal support-context evidence schema.

    This validator is intentionally independent of the v2 event journal.  A
    producer must provide exact per-topic counts, a stable backend completion
    boundary, and a timestamped explanation for every residual IMU/image
    record.  Residual records are neither processed nor dropped by this
    contract; they are only classified as post-boundary support context.
    """
    if value.get('status') != 'pass':
        raise PhaseContractError('document_status_not_pass')
    if value.get('phase_mode') not in PHASE_MODES_V2:
        raise PhaseContractError('phase_mode_invalid')
    try:
        required_end = _finite(
            value.get('required_evaluation_end_timestamp_seconds'),
            'required_evaluation_end_timestamp_seconds', nonnegative=False)
    except PhaseContractError as error:
        raise PhaseContractError(
            'required_evaluation_end_timestamp_invalid') from error
    try:
        configured_gap = _finite(
            value.get('maximum_end_gap_seconds'), 'maximum_end_gap_seconds')
        allowed_gap = _finite(
            maximum_end_gap_seconds, 'maximum_end_gap_seconds')
    except PhaseContractError as error:
        raise PhaseContractError('maximum_end_gap_seconds_invalid') from error
    if configured_gap > allowed_gap:
        raise PhaseContractError('maximum_end_gap_exceeds_validator_limit')

    counts = value.get('counts')
    if not isinstance(counts, dict):
        raise PhaseContractError('counts_missing_or_invalid')
    count_maps: dict[str, dict[str, int]] = {}
    for name in ('expected', 'published', 'received', 'acknowledged'):
        count_maps[name] = _v3_topic_counts(counts.get(name), f'counts_{name}')
    expected = count_maps['expected']
    for name in ('published', 'received', 'acknowledged'):
        if count_maps[name] != expected:
            raise PhaseContractError(f'{name}_count_mismatch')

    backend = value.get('backend')
    if not isinstance(backend, dict):
        raise PhaseContractError('backend_missing_or_invalid')
    if backend.get('quiescent') is not True or \
            backend.get('quiescence_observed') is not True:
        raise PhaseContractError('backend_not_quiescent')
    in_flight = backend.get('in_flight')
    if not isinstance(in_flight, dict) or \
            not isinstance(in_flight.get('active'), bool):
        raise PhaseContractError('backend_in_flight_state_missing')
    if in_flight.get('active') is not False:
        raise PhaseContractError('backend_in_flight')
    boundary = backend.get('completed_boundary')
    if not isinstance(boundary, dict) or boundary.get('observed') is not True:
        raise PhaseContractError('completed_boundary_missing_or_unobserved')
    try:
        boundary_timestamp = _finite(
            boundary.get('timestamp_seconds'),
            'backend.completed_boundary.timestamp_seconds', nonnegative=False)
    except PhaseContractError as error:
        raise PhaseContractError('completed_boundary_timestamp_invalid') from error
    try:
        _integer(boundary.get('sequence'), 'backend.completed_boundary.sequence')
    except PhaseContractError as error:
        raise PhaseContractError('completed_boundary_sequence_invalid') from error
    if not isinstance(boundary.get('source'), str) or not boundary['source'].strip():
        raise PhaseContractError('completed_boundary_source_invalid')
    completed_counts = _v3_topic_counts(
        backend.get('completed_counts'), 'backend_completed_counts')
    dropped_counts = _v3_topic_counts(
        backend.get('dropped_counts'), 'backend_dropped_counts')
    overflow_counts = _v3_topic_counts(
        backend.get('overflow_counts'), 'backend_overflow_counts')
    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V3:
        if dropped_counts[topic] != 0 or overflow_counts[topic] != 0:
            raise PhaseContractError(f'drops_or_overflow_nonzero_{topic}')
    try:
        processing_failures = _integer(
            backend.get('processing_failures'), 'backend.processing_failures')
    except PhaseContractError as error:
        raise PhaseContractError('processing_failures_missing_or_invalid') from error
    if processing_failures != 0:
        raise PhaseContractError('processing_failures_nonzero')
    try:
        completed_units = _integer(
            backend.get('completed_synchronization_units'),
            'backend.completed_synchronization_units')
    except PhaseContractError as error:
        raise PhaseContractError('completed_synchronization_units_invalid') from error

    buffers = value.get('buffers')
    if not isinstance(buffers, dict) or set(buffers) != set(
            TERMINAL_SUPPORT_CONTEXT_TOPICS_V3):
        raise PhaseContractError('per_topic_buffers_missing_or_invalid')
    buffer_counts: dict[str, int] = {}
    buffer_details: dict[str, dict[str, Any]] = {}
    residual_records: list[dict[str, Any]] = []
    record_ids: set[str] = set()
    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V3:
        count, records, details = _v3_validate_buffer(
            topic, buffers.get(topic), boundary_timestamp)
        buffer_counts[topic] = count
        buffer_details[topic] = details
        for record in records:
            record_id = record['record_id']
            if record_id in record_ids:
                raise PhaseContractError('support_context_record_id_duplicate')
            record_ids.add(record_id)
            residual_records.append(record)

    terminal = value.get('terminal_support_context')
    if not isinstance(terminal, dict):
        raise PhaseContractError('terminal_support_context_missing_or_invalid')
    if terminal.get('classification') != 'post_boundary_support_context_only':
        raise PhaseContractError('terminal_support_context_classification_invalid')
    terminal_counts = _v3_topic_counts(
        terminal.get('by_topic'), 'terminal_support_context_by_topic')
    if terminal_counts != buffer_counts:
        raise PhaseContractError('terminal_support_context_count_mismatch')
    try:
        terminal_total = _integer(
            terminal.get('total_count'), 'terminal_support_context.total_count')
    except PhaseContractError as error:
        raise PhaseContractError('terminal_support_context_total_invalid') from error
    if terminal_total != len(residual_records):
        raise PhaseContractError('terminal_support_context_total_mismatch')
    terminal_records = terminal.get('records')
    if not isinstance(terminal_records, list) or len(terminal_records) != terminal_total:
        raise PhaseContractError('terminal_support_context_records_invalid')
    terminal_ids: list[str] = []
    for record in terminal_records:
        if not isinstance(record, dict):
            raise PhaseContractError('support_context_record_invalid')
        topic = record.get('topic')
        if topic not in TERMINAL_SUPPORT_CONTEXT_ALLOWED_TOPICS_V3:
            raise PhaseContractError('support_context_record_topic_invalid')
        _v3_validate_support_record(record, topic, boundary_timestamp)
        terminal_ids.append(record['record_id'])
    if len(set(terminal_ids)) != len(terminal_ids) or \
            sorted(terminal_ids) != sorted(record_ids):
        raise PhaseContractError('terminal_support_context_records_mismatch')

    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V3:
        if completed_counts[topic] + buffer_counts[topic] != \
                count_maps['received'][topic]:
            raise PhaseContractError(f'count_conservation_failed_{topic}')

    trajectory = value.get('trajectory')
    if not isinstance(trajectory, dict) or \
            trajectory.get('coverage_verified') is not True:
        raise PhaseContractError('trajectory_coverage_unverified')
    try:
        last_timestamp = _finite(
            trajectory.get('last_timestamp_seconds'),
            'trajectory.last_timestamp_seconds', nonnegative=False)
        end_gap = _finite(
            trajectory.get('end_gap_seconds'), 'trajectory.end_gap_seconds')
    except PhaseContractError as error:
        raise PhaseContractError('trajectory_coverage_timestamp_invalid') from error
    calculated_gap = required_end - last_timestamp
    if not math.isclose(end_gap, calculated_gap, rel_tol=0.0, abs_tol=1e-9):
        raise PhaseContractError('trajectory_end_gap_mismatch')
    if end_gap > configured_gap:
        raise PhaseContractError('trajectory_end_gap_exceeds_limit')
    if boundary_timestamp < required_end:
        raise PhaseContractError('backend_boundary_does_not_cover_required_end')
    if value.get('ground_truth_content_opened') is not False:
        raise PhaseContractError('ground_truth_content_opened')
    if value.get('scorer_invoked') is not False:
        raise PhaseContractError('scorer_invoked')
    return {
        'phase_contract_version': CONTRACT_VERSION_V3,
        'phase_mode': value.get('phase_mode'),
        'required_evaluation_end_timestamp_seconds': required_end,
        'backend_boundary_timestamp_seconds': boundary_timestamp,
        'backend_completed_synchronization_units': completed_units,
        'backend_quiescent': True,
        'in_flight': False,
        'per_topic_buffers': buffer_details,
        'support_context_counts': buffer_counts,
        'support_context_total_count': terminal_total,
        'completed_counts': completed_counts,
        'received_counts': count_maps['received'],
        'count_conservation_passed': True,
        'drops_and_overflow_zero': True,
        'processing_failures': processing_failures,
        'trajectory_last_timestamp_seconds': last_timestamp,
        'trajectory_end_gap_seconds': end_gap,
        'maximum_end_gap_seconds': configured_gap,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def validate_terminal_support_context_v3(
        value: dict[str, Any], *,
        maximum_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
        require_pass: bool = False) -> dict[str, Any]:
    """Validate v3 terminal support-context evidence fail-closed.

    Schema/version dispatch is explicit so a v2 document can never gain the
    v3 residual-buffer exception accidentally.  Malformed v3 documents return
    an ``invalid`` result for diagnostic closure, while ``require_pass`` turns
    the same result into a hard contract error.
    """
    if not isinstance(value, dict):
        raise PhaseContractError('terminal support context evidence must be an object')
    if value.get('schema_version') != SCHEMA_VERSION_V3 or \
            value.get('contract_version') != CONTRACT_VERSION_V3:
        raise PhaseContractError('terminal support context schema/version mismatch')
    result = dict(value)
    try:
        details = _validate_terminal_support_context_v3(
            value, maximum_end_gap_seconds=maximum_end_gap_seconds)
    except PhaseContractError as error:
        result['status'] = 'invalid'
        result['validation'] = {'status': 'invalid', 'reason': str(error)}
        if require_pass:
            raise PhaseContractError(
                f'phase evidence is not a passing v3 gate: {error}') from error
        return result
    result['status'] = 'pass'
    result['validation'] = details
    return result


def _v5_validate_support_record(
        record: Any, topic: str, boundary_timestamp: float) -> None:
    """Validate v5 residual support context without changing v3 semantics.

    v5 is deliberately a separate dispatch branch.  Only IMU/image records
    exactly at or after the completed boundary are admissible; lidar remains
    forbidden and every record still needs the source-provided sync proof.
    """
    if not isinstance(record, dict):
        raise PhaseContractError('support_context_record_invalid')
    if _v3_forbidden_record_label(record):
        raise PhaseContractError(
            'support_context_record_has_forbidden_processing_label')
    if record.get('topic') != topic:
        raise PhaseContractError('support_context_record_topic_mismatch')
    record_id = record.get('record_id')
    if not isinstance(record_id, str) or not record_id.strip():
        raise PhaseContractError('support_context_record_id_missing_or_invalid')
    timestamp = _finite(
        record.get('timestamp_seconds'),
        f'{topic} support context timestamp', nonnegative=False)
    if topic == 'lidar':
        raise PhaseContractError('residual_lidar_not_support_context')
    if timestamp < boundary_timestamp:
        raise PhaseContractError('support_context_record_pre_boundary')
    equal = math.isclose(timestamp, boundary_timestamp, rel_tol=0.0,
                         abs_tol=1e-12)
    if record.get('support_context_proven') is not True:
        raise PhaseContractError('support_context_record_not_proven')
    if record.get('sync_predicate_evaluated') is not True:
        raise PhaseContractError('support_context_sync_predicate_unproven')
    if record.get('can_form_synchronization_unit') is not False:
        raise PhaseContractError(
            'support_context_record_can_form_synchronization_unit')
    if equal:
        if record.get('post_boundary') is not False or \
                record.get('equal_to_completed_boundary') is not True or \
                record.get('reason_code') != TERMINAL_SUPPORT_CONTEXT_V5_EQUAL_REASON:
            raise PhaseContractError('support_context_equal_boundary_classification_invalid')
    else:
        if record.get('post_boundary') is not True or \
                record.get('equal_to_completed_boundary') is not False or \
                record.get('reason_code') != 'strictly_after_completed_boundary':
            raise PhaseContractError('support_context_strict_boundary_classification_invalid')


def _v5_validate_buffer(
        topic: str, value: Any, boundary_timestamp: float
        ) -> tuple[int, list[dict[str, Any]]]:
    if not isinstance(value, dict):
        raise PhaseContractError(f'buffer_{topic}_missing_or_invalid')
    count = _integer(value.get('count'), f'buffer.{topic}.count')
    records = value.get('records')
    if not isinstance(records, list) or len(records) != count:
        raise PhaseContractError(f'buffer_{topic}_count_does_not_match_records')
    ids: set[str] = set()
    timestamps: list[float] = []
    for record in records:
        if not isinstance(record, dict):
            raise PhaseContractError(f'buffer_{topic}_record_invalid')
        record_id = record.get('record_id')
        if not isinstance(record_id, str) or not record_id.strip() or record_id in ids:
            raise PhaseContractError(f'buffer_{topic}_record_id_invalid_or_duplicate')
        ids.add(record_id)
        timestamps.append(_finite(
            record.get('timestamp_seconds'),
            f'buffer.{topic}.record.timestamp_seconds', nonnegative=False))
        if record.get('topic') != topic:
            raise PhaseContractError(f'buffer_{topic}_record_topic_mismatch')
    if records:
        oldest = _finite(value.get('oldest_timestamp_seconds'),
                         f'buffer.{topic}.oldest_timestamp_seconds',
                         nonnegative=False)
        newest = _finite(value.get('newest_timestamp_seconds'),
                         f'buffer.{topic}.newest_timestamp_seconds',
                         nonnegative=False)
        if not math.isclose(oldest, min(timestamps), rel_tol=0.0, abs_tol=1e-12) or \
                not math.isclose(newest, max(timestamps), rel_tol=0.0, abs_tol=1e-12):
            raise PhaseContractError(f'buffer_{topic}_timestamp_bounds_mismatch')
    elif value.get('oldest_timestamp_seconds') is not None or \
            value.get('newest_timestamp_seconds') is not None:
        raise PhaseContractError(f'buffer_{topic}_empty_timestamp_invalid')
    for record in records:
        _v5_validate_support_record(record, topic, boundary_timestamp)
    return count, records


def _validate_terminal_support_context_v5(
        value: dict[str, Any], *, maximum_end_gap_seconds: float
        ) -> dict[str, Any]:
    for forbidden in (
            'input', 'profile_sha256', 'profile_path', 'raw_path', 'raw_sha256',
            'expected', 'expected_topic_counts', 'published',
            'published_topic_counts', 'ack', 'acknowledged_topic_counts'):
        if forbidden in value:
            raise PhaseContractError(f'authority_field_forbidden_{forbidden}')
    if value.get('status') != 'pass':
        raise PhaseContractError('document_status_not_pass')
    if value.get('phase_mode') not in PHASE_MODES_V2:
        raise PhaseContractError('phase_mode_invalid')
    required_end = _finite(
        value.get('required_evaluation_end_timestamp_seconds'),
        'required_evaluation_end_timestamp_seconds', nonnegative=False)
    configured_gap = _finite(value.get('maximum_end_gap_seconds'),
                              'maximum_end_gap_seconds')
    allowed_gap = _finite(maximum_end_gap_seconds,
                           'maximum_end_gap_seconds')
    if configured_gap < 0.0 or configured_gap > allowed_gap:
        raise PhaseContractError('maximum_end_gap_invalid')
    counts = value.get('counts')
    if not isinstance(counts, dict):
        raise PhaseContractError('counts_missing_or_invalid')
    count_maps = {
        name: _v3_topic_counts(counts.get(name), f'counts_{name}')
        for name in ('expected', 'published', 'received', 'acknowledged')}
    expected = count_maps['expected']
    for name in ('published', 'received', 'acknowledged'):
        if count_maps[name] != expected:
            raise PhaseContractError(f'{name}_count_mismatch')
    backend = value.get('backend')
    if not isinstance(backend, dict) or backend.get('quiescent') is not True or \
            backend.get('quiescence_observed') is not True:
        raise PhaseContractError('backend_not_quiescent')
    flight = backend.get('in_flight')
    if not isinstance(flight, dict) or flight.get('active') is not False:
        raise PhaseContractError('backend_in_flight')
    boundary = backend.get('completed_boundary')
    if not isinstance(boundary, dict) or boundary.get('observed') is not True:
        raise PhaseContractError('completed_boundary_missing_or_unobserved')
    boundary_timestamp = _finite(
        boundary.get('timestamp_seconds'),
        'backend.completed_boundary.timestamp_seconds', nonnegative=False)
    if _integer(boundary.get('sequence'), 'backend.completed_boundary.sequence') < 0 or \
            not isinstance(boundary.get('source'), str) or not boundary['source'].strip():
        raise PhaseContractError('completed_boundary_identity_invalid')
    completed = _v3_topic_counts(
        backend.get('completed_counts'), 'backend_completed_counts')
    dropped = _v3_topic_counts(backend.get('dropped_counts'), 'backend_dropped_counts')
    overflow = _v3_topic_counts(backend.get('overflow_counts'), 'backend_overflow_counts')
    if any(dropped.values()) or any(overflow.values()):
        raise PhaseContractError('drops_or_overflow_nonzero')
    if _integer(backend.get('processing_failures'), 'backend.processing_failures') != 0:
        raise PhaseContractError('processing_failures_nonzero')
    buffers = value.get('buffers')
    if not isinstance(buffers, dict) or set(buffers) != set(TERMINAL_SUPPORT_CONTEXT_TOPICS_V5):
        raise PhaseContractError('per_topic_buffers_missing_or_invalid')
    buffer_counts: dict[str, int] = {}
    buffer_records: list[dict[str, Any]] = []
    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V5:
        count, records = _v5_validate_buffer(
            topic, buffers.get(topic), boundary_timestamp)
        buffer_counts[topic] = count
        buffer_records.extend(records)
    if buffer_counts['lidar'] != 0:
        raise PhaseContractError('residual_lidar_not_support_context')
    for topic in TERMINAL_SUPPORT_CONTEXT_TOPICS_V5:
        if completed[topic] + buffer_counts[topic] != count_maps['received'][topic]:
            raise PhaseContractError(f'count_conservation_failed_{topic}')
    terminal = value.get('terminal_support_context')
    if not isinstance(terminal, dict) or \
            terminal.get('classification') != TERMINAL_SUPPORT_CONTEXT_V5_CLASSIFICATION:
        raise PhaseContractError('terminal_support_context_classification_invalid')
    terminal_counts = _v3_topic_counts(
        terminal.get('by_topic'), 'terminal_support_context_by_topic')
    if terminal_counts != buffer_counts or \
            _integer(terminal.get('total_count'), 'terminal_support_context.total_count') != len(buffer_records):
        raise PhaseContractError('terminal_support_context_count_mismatch')
    terminal_records = terminal.get('records')
    if not isinstance(terminal_records, list) or len(terminal_records) != len(buffer_records):
        raise PhaseContractError('terminal_support_context_records_invalid')
    expected_ids = sorted(record['record_id'] for record in buffer_records)
    observed_ids: list[str] = []
    for record in terminal_records:
        topic = record.get('topic') if isinstance(record, dict) else None
        if topic not in TERMINAL_SUPPORT_CONTEXT_ALLOWED_TOPICS_V5:
            raise PhaseContractError('support_context_record_topic_invalid')
        _v5_validate_support_record(record, topic, boundary_timestamp)
        observed_ids.append(record['record_id'])
    if sorted(observed_ids) != expected_ids or len(set(observed_ids)) != len(observed_ids):
        raise PhaseContractError('terminal_support_context_records_mismatch')
    observation = value.get('terminal_observation')
    if not isinstance(observation, dict) or observation.get('eof_observed') is not True or \
            observation.get('stable') is not True or observation.get('identical_snapshot') is not True or \
            observation.get('sync_predicate_evaluated') is not True:
        raise PhaseContractError('consumer_terminal_observation_unproven')
    if _integer(observation.get('poll_count'), 'terminal_observation.poll_count') < 2 or \
            _integer(observation.get('stable_poll_count'), 'terminal_observation.stable_poll_count') < 2:
        raise PhaseContractError('consumer_terminal_poll_count_invalid')
    first_poll = _finite(observation.get('first_poll_wall_seconds'),
                         'terminal_observation.first_poll_wall_seconds')
    stable_poll = _finite(observation.get('stable_poll_wall_seconds'),
                          'terminal_observation.stable_poll_wall_seconds')
    minimum_poll = _finite(observation.get('minimum_poll_wall_seconds'),
                           'terminal_observation.minimum_poll_wall_seconds')
    if stable_poll < first_poll or stable_poll - first_poll < minimum_poll:
        raise PhaseContractError('consumer_terminal_poll_interval_invalid')
    trajectory = value.get('trajectory')
    if not isinstance(trajectory, dict) or trajectory.get('coverage_verified') is not True:
        raise PhaseContractError('trajectory_coverage_unverified')
    last_timestamp = _finite(trajectory.get('last_timestamp_seconds'),
                             'trajectory.last_timestamp_seconds', nonnegative=False)
    end_gap = _finite(trajectory.get('end_gap_seconds'), 'trajectory.end_gap_seconds')
    if not math.isclose(end_gap, required_end - last_timestamp, rel_tol=0.0, abs_tol=1e-9):
        raise PhaseContractError('trajectory_end_gap_mismatch')
    if end_gap < 0.0 or end_gap > configured_gap:
        raise PhaseContractError('trajectory_end_gap_out_of_bounds')
    if value.get('ground_truth_content_opened') is not False or value.get('scorer_invoked') is not False:
        raise PhaseContractError('safety_flags_invalid')
    return {
        'phase_contract_version': CONTRACT_VERSION_V5,
        'phase_mode': value.get('phase_mode'),
        'required_evaluation_end_timestamp_seconds': required_end,
        'backend_boundary_timestamp_seconds': boundary_timestamp,
        'support_context_classification': TERMINAL_SUPPORT_CONTEXT_V5_CLASSIFICATION,
        'support_context_counts': buffer_counts,
        'support_context_total_count': len(buffer_records),
        'completed_counts': completed,
        'received_counts': count_maps['received'],
        'count_conservation_passed': True,
        'residual_lidar_forbidden': True,
        'trajectory_end_gap_seconds': end_gap,
        'maximum_end_gap_seconds': configured_gap,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def validate_terminal_support_context_v5(
        value: dict[str, Any], *,
        maximum_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
        require_pass: bool = False) -> dict[str, Any]:
    """Validate v5 equality-aware terminal evidence, fail closed."""
    if not isinstance(value, dict) or value.get('schema_version') != SCHEMA_VERSION_V3 or \
            value.get('contract_version') != CONTRACT_VERSION_V5:
        raise PhaseContractError('terminal support context schema/version mismatch for v5')
    result = dict(value)
    try:
        details = _validate_terminal_support_context_v5(
            value, maximum_end_gap_seconds=maximum_end_gap_seconds)
    except PhaseContractError as error:
        result['status'] = 'invalid'
        result['validation'] = {'status': 'invalid', 'reason': str(error)}
        if require_pass:
            raise PhaseContractError(
                f'phase evidence is not a passing v5 gate: {error}') from error
        return result
    result['status'] = 'pass'
    result['validation'] = details
    return result


def _timeout_diagnostic_path(base_path: Path) -> Path:
    suffix = base_path.suffix
    stem = base_path.name[:-len(suffix)] if suffix else base_path.name
    return base_path.with_name(f'{stem}.timeout{suffix}')


def _validate_consumer_eof_boundary(value: Any) -> None:
    """Validate the immutable application EOF marker written before wait().

    The marker is deliberately separate from final consumer evidence: it
    establishes input_end/drain_start before asynchronous GLIM work drains.
    It is never a completion proof by itself.
    """
    if not isinstance(value, dict) or value.get('schema_version') != 1 or \
            value.get('kind') != 'm6a10_consumer_eof_boundary' or \
            value.get('status') != 'eof':
        raise PhaseContractError('consumer EOF boundary schema mismatch')
    if value.get('contract_version') != CONTRACT_VERSION_V2 or \
            value.get('phase_mode') not in PHASE_MODES_V2 or \
            value.get('eof_observed') is not True:
        raise PhaseContractError('consumer EOF boundary contract metadata invalid')
    if not isinstance(value.get('eof_source'), str) or not value['eof_source'].strip():
        raise PhaseContractError('consumer EOF boundary source missing')
    integer_fields = (
        'expected_messages', 'received_messages', 'processed_messages',
        'dropped_messages', 'queue_overflow', 'processing_failures')
    integers: dict[str, int] = {}
    for key in integer_fields:
        integers[key] = _integer(value.get(key), f'consumer EOF {key}')
    if not (integers['expected_messages'] == integers['received_messages'] ==
            integers['processed_messages']):
        raise PhaseContractError('consumer EOF counts are not equal')
    if integers['dropped_messages'] != 0 or integers['queue_overflow'] != 0:
        raise PhaseContractError('consumer EOF reports drops or overflow')
    if integers['processing_failures'] != 0:
        raise PhaseContractError('consumer EOF reports processing failure')
    first = _finite(value.get('first_processed_timestamp_seconds'),
                    'consumer EOF first timestamp', nonnegative=False)
    last = _finite(value.get('last_processed_timestamp_seconds'),
                   'consumer EOF last timestamp', nonnegative=False)
    if first < 0 or last < first:
        raise PhaseContractError('consumer EOF timestamps are invalid')


def consumer_eof_state(evidence_path: Path) -> tuple[int, dict[str, Any]]:
    """Classify the pre-wait EOF sidecar: 0 valid, 1 invalid, 2 pending."""
    part_path = Path(f'{evidence_path}.part')
    if part_path.exists():
        return 1, {'status': 'invalid', 'reason': 'consumer EOF sidecar part exists'}
    if not evidence_path.exists():
        return 2, {'status': 'pending', 'reason': 'consumer EOF sidecar missing'}
    try:
        value = load_object(evidence_path)
        _validate_consumer_eof_boundary(value)
    except (PhaseContractError, OSError, ValueError) as error:
        return 1, {'status': 'invalid', 'reason': str(error)}
    return 0, {'status': 'eof', 'path': str(evidence_path)}


def _validate_timeout_diagnostic(value: dict[str, Any]) -> None:
    """Require the complete bounded-drain snapshot before returning 124."""
    if value.get('schema_version') != 1 or \
            value.get('kind') != 'm6a10_drain_diagnostic' or \
            value.get('event') != 'timeout' or \
            value.get('status') != 'timeout':
        raise PhaseContractError('timeout diagnostic schema/event mismatch')
    if value.get('contract_version') != CONTRACT_VERSION_V2 or \
            value.get('phase_mode') not in PHASE_MODES_V2 or \
            value.get('benchmark_only') is not True:
        raise PhaseContractError('timeout diagnostic contract metadata invalid')
    if not isinstance(value.get('reason'), str) or not value['reason'].strip():
        raise PhaseContractError('timeout diagnostic reason missing')
    try:
        _finite(value.get('drain_timeout_seconds'),
                'timeout diagnostic drain_timeout_seconds')
    except PhaseContractError as error:
        raise PhaseContractError(str(error)) from error
    snapshot = value.get('snapshot')
    counters = value.get('counters')
    if not isinstance(snapshot, dict) or not isinstance(counters, dict):
        raise PhaseContractError('timeout diagnostic snapshot/counters missing')
    for key in (
            'lidar_buffer_size', 'imu_buffer_size',
            'front_lidar_min_timestamp_ns', 'front_lidar_max_timestamp_ns',
            'last_imu_timestamp_ns', 'timestamp_gap_ns'):
        _integer(snapshot.get(key), f'timeout snapshot {key}',
                 nonnegative=(key != 'timestamp_gap_ns'))
    for key in ('atomic_can_process', 'registration_active'):
        if not isinstance(snapshot.get(key), bool):
            raise PhaseContractError(f'timeout snapshot {key} invalid')
    for key in (
            'expected_messages', 'received_messages', 'processed_messages',
            'dropped_messages', 'queue_overflow', 'processing_failures'):
        _integer(counters.get(key), f'timeout counters {key}')
    if not isinstance(value.get('eof_observed'), bool) or \
            not isinstance(value.get('drain_complete'), bool):
        raise PhaseContractError('timeout diagnostic EOF/drain flags invalid')
    if not isinstance(value.get('evidence_path'), str) or \
            not value['evidence_path'].strip():
        raise PhaseContractError('timeout diagnostic evidence path missing')


def consumer_evidence_state(
        evidence_path: Path, diagnostic_base_path: Path, *,
        maximum_callback_latency_seconds: float =
        DEFAULT_CALLBACK_LATENCY_SECONDS_V2,
        maximum_backlog_messages: int = DEFAULT_BACKLOG_MESSAGES_V2
        ) -> tuple[int, dict[str, Any]]:
    """Classify v2 evidence for the runner's completion barrier.

    Return codes are deliberately shell-friendly: ``0`` pass, ``1`` malformed
    or invalid, ``2`` pending (no final evidence yet), and ``124`` for a valid
    terminal drain timeout.  A timeout takes precedence over an invalid
    consumer document so the outer wrapper preserves the diagnostic's exit
    contract rather than silently converting it to success.
    """
    timeout_path = _timeout_diagnostic_path(diagnostic_base_path)
    if timeout_path.exists():
        try:
            diagnostic = load_object(timeout_path)
            _validate_timeout_diagnostic(diagnostic)
        except PhaseContractError as error:
            return 1, {'status': 'invalid', 'reason': str(error)}
        return 124, {'status': 'timeout', 'path': str(timeout_path)}
    if not evidence_path.exists():
        return 2, {'status': 'pending', 'reason': 'consumer evidence missing'}
    try:
        evidence = load_object(evidence_path)
        validated = validate_consumer_evidence_v2(
            evidence,
            maximum_callback_latency_seconds=maximum_callback_latency_seconds,
            maximum_backlog_messages=maximum_backlog_messages)
    except PhaseContractError as error:
        return 1, {'status': 'invalid', 'reason': str(error)}
    return 0, validated


def validate_evidence_v2(
        value: dict[str, Any], *, maximum_online_rtf: float = ONLINE_RTF_LIMIT,
        maximum_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
        maximum_callback_latency_seconds: float =
        DEFAULT_CALLBACK_LATENCY_SECONDS_V2,
        maximum_backlog_messages: int = DEFAULT_BACKLOG_MESSAGES_V2,
        require_pass: bool = False) -> dict[str, Any]:
    """Validate M6a10-v2 paced followability or ack-gated throughput.

    In ``paced_1x`` mode, exact consumer counts/EOF/drain and timestamp
    coverage are the primary gate; the online RTF is diagnostic.  In
    ``unpaced_ack`` mode, the same acknowledgement proof is required and the
    acknowledgement interval is additionally gated by the preregistered
    throughput RTF limit.  Neither mode accepts publisher counts as proof.
    """
    if value.get('schema_version') != SCHEMA_VERSION_V2 or \
            value.get('contract_version') != CONTRACT_VERSION_V2:
        raise PhaseContractError('phase evidence schema/version mismatch for v2')
    phase_mode = value.get('phase_mode')
    events = value.get('events_monotonic_ns')
    if not isinstance(events, dict):
        raise PhaseContractError('phase timestamps missing')
    stamps: list[int] = []
    for name in ORDERED_EVENTS:
        try:
            stamp = _integer(events.get(name), f'events.{name}')
        except PhaseContractError as error:
            raise PhaseContractError(f'missing phase boundary: {name}') from error
        stamps.append(stamp)
    if any(later < earlier for earlier, later in zip(stamps, stamps[1:])):
        raise PhaseContractError('phase boundaries are not monotonic')
    duration = _finite(value.get('input_duration_seconds'), 'input_duration_seconds')
    if duration <= 0:
        raise PhaseContractError('input duration is not positive')
    online_seconds = (events['drain_end'] - events['input_start']) / 1e9
    if online_seconds <= 0:
        raise PhaseContractError('online acknowledgement interval is not positive')
    online_rtf = online_seconds / duration
    if not math.isfinite(online_rtf) or online_rtf < 0:
        raise PhaseContractError('online throughput RTF is not finite')
    coverage_ok, coverage_reason = _validate_coverage(
        value.get('coverage'), _finite(
            value.get('maximum_trajectory_end_gap_seconds'),
            'maximum_trajectory_end_gap_seconds'))
    consumer_ok, consumer_reason, consumer_details = _validate_consumer_v2(
        value.get('consumer'), phase_mode,
        maximum_end_gap_seconds=maximum_end_gap_seconds,
        maximum_callback_latency_seconds=maximum_callback_latency_seconds,
        maximum_backlog_messages=maximum_backlog_messages)
    resource = value.get('resource')
    resource_ok = isinstance(resource, dict) and resource.get('status') == 'verified'
    if resource_ok:
        for key in ('cpu_user_seconds', 'cpu_system_seconds',
                    'io_input_operations', 'io_output_operations'):
            try:
                _finite(resource.get(key), f'resource.{key}')
            except PhaseContractError:
                resource_ok = False
                break
    exit_status = value.get('exit_status')
    if isinstance(exit_status, bool) or not isinstance(exit_status, int):
        raise PhaseContractError('exit_status is missing or invalid')
    base_ok = exit_status == 0 and coverage_ok and consumer_ok and resource_ok
    paced_followability = phase_mode == 'paced_1x' and base_ok
    unpaced_throughput = phase_mode == 'unpaced_ack' and base_ok and \
        online_rtf <= _finite(maximum_online_rtf, 'maximum_online_rtf')
    result = dict(value)
    result['durations_seconds'] = {
        name: (events[end] - events[start]) / 1e9
        for name, start, end in (
            ('startup', 'startup_start', 'startup_end'),
            ('online_acknowledgement', 'input_start', 'drain_end'),
            ('input_consumption', 'input_start', 'input_end'),
            ('required_drain', 'drain_start', 'drain_end'),
            ('postprocess', 'postprocess_start', 'postprocess_end'),
            ('save', 'save_start', 'save_end'),
            ('shutdown', 'shutdown_start', 'shutdown_end'),
            ('wall', 'startup_start', 'shutdown_end'))}
    result['runtime'] = {
        'phase_contract_version': CONTRACT_VERSION_V2,
        'phase_mode': phase_mode,
        'online_acknowledgement_seconds': online_seconds,
        'online_compute_rtf': online_rtf,
        'wall_seconds': result['durations_seconds']['wall'],
        'maximum_unpaced_throughput_rtf': float(maximum_online_rtf),
        'paced_followability_passed': paced_followability,
        'unpaced_throughput_gate_passed': unpaced_throughput,
        'online_compute_rtf_is_diagnostic_for_paced_mode': True,
        'wall_rtf_is_diagnostic_only': True,
    }
    result['validation'] = {
        'coverage_passed': coverage_ok,
        'coverage_reason': coverage_reason,
        'consumer_passed': consumer_ok,
        'consumer_reason': consumer_reason,
        'consumer_details': consumer_details,
        'maximum_allowed_callback_latency_seconds': (
            maximum_callback_latency_seconds),
        'maximum_allowed_backlog_messages': maximum_backlog_messages,
        'resource_passed': resource_ok,
        'phase_order_passed': True,
        'publisher_count_rejected': True,
        'primary_gate': ('paced_followability' if phase_mode == 'paced_1x'
                         else 'unpaced_ack_throughput'),
    }
    result['status'] = 'pass' if paced_followability or unpaced_throughput else 'invalid'
    if require_pass and result['status'] != 'pass':
        raise PhaseContractError(
            f'phase evidence is not a passing v2 gate: {result["status"]}')
    return result


def validate_evidence(value: dict[str, Any], *, maximum_online_rtf: float = ONLINE_RTF_LIMIT,
                      maximum_end_gap_seconds: float = DEFAULT_END_GAP_SECONDS,
                      require_pass: bool = False) -> dict[str, Any]:
    if value.get('contract_version') == CONTRACT_VERSION_V5:
        return validate_terminal_support_context_v5(
            value, maximum_end_gap_seconds=maximum_end_gap_seconds,
            require_pass=require_pass)
    if value.get('schema_version') == SCHEMA_VERSION_V3 or \
            value.get('contract_version') == CONTRACT_VERSION_V3:
        if value.get('schema_version') != SCHEMA_VERSION_V3 or \
                value.get('contract_version') != CONTRACT_VERSION_V3:
            raise PhaseContractError(
                'phase evidence schema/version mismatch for v3')
        return validate_terminal_support_context_v3(
            value, maximum_end_gap_seconds=maximum_end_gap_seconds,
            require_pass=require_pass)
    if value.get('schema_version') != SCHEMA_VERSION or \
            value.get('contract_version') != CONTRACT_VERSION:
        raise PhaseContractError('phase evidence schema/version mismatch')
    events = value.get('events_monotonic_ns')
    if not isinstance(events, dict):
        raise PhaseContractError('phase timestamps missing')
    stamps: list[int] = []
    for name in ORDERED_EVENTS:
        try:
            stamp = _integer(events.get(name), f'events.{name}')
        except PhaseContractError as error:
            raise PhaseContractError(f'missing phase boundary: {name}') from error
        stamps.append(stamp)
    if any(later < earlier for earlier, later in zip(stamps, stamps[1:])):
        raise PhaseContractError('phase boundaries are not monotonic')
    duration = _finite(value.get('input_duration_seconds'), 'input_duration_seconds')
    if duration <= 0:
        raise PhaseContractError('input duration is not positive')
    online_seconds = (events['drain_end'] - events['input_start']) / 1e9
    if online_seconds <= 0:
        raise PhaseContractError('online compute interval is not positive')
    online_rtf = online_seconds / duration
    if not math.isfinite(online_rtf) or online_rtf < 0:
        raise PhaseContractError('online_compute_rtf is not finite')
    coverage_ok, coverage_reason = _validate_coverage(
        value.get('coverage'), _finite(
            value.get('maximum_trajectory_end_gap_seconds'),
            'maximum_trajectory_end_gap_seconds'))
    resource = value.get('resource')
    resource_ok = isinstance(resource, dict) and resource.get('status') == 'verified'
    if resource_ok:
        for key in ('cpu_user_seconds', 'cpu_system_seconds',
                    'io_input_operations', 'io_output_operations'):
            try:
                _finite(resource.get(key), f'resource.{key}')
            except PhaseContractError:
                resource_ok = False
                break
    exit_status = value.get('exit_status')
    if isinstance(exit_status, bool) or not isinstance(exit_status, int):
        raise PhaseContractError('exit_status is missing or invalid')
    gate_pass = online_rtf <= _finite(maximum_online_rtf, 'maximum_online_rtf')
    result = dict(value)
    result['durations_seconds'] = {
        name: (events[end] - events[start]) / 1e9
        for name, start, end in (
            ('startup', 'startup_start', 'startup_end'),
            ('online_compute', 'input_start', 'drain_end'),
            ('input_consumption', 'input_start', 'input_end'),
            ('required_drain', 'drain_start', 'drain_end'),
            ('postprocess', 'postprocess_start', 'postprocess_end'),
            ('save', 'save_start', 'save_end'),
            ('shutdown', 'shutdown_start', 'shutdown_end'),
            ('wall', 'startup_start', 'shutdown_end'))}
    result['runtime'] = {
        'online_compute_seconds': online_seconds,
        'online_compute_rtf': online_rtf,
        'wall_seconds': result['durations_seconds']['wall'],
        'maximum_online_compute_rtf': float(maximum_online_rtf),
        'online_compute_rtf_gate_passed': gate_pass,
        'wall_rtf_is_diagnostic_only': True,
    }
    result['validation'] = {
        'coverage_passed': coverage_ok,
        'coverage_reason': coverage_reason,
        'resource_passed': resource_ok,
        'phase_order_passed': True,
        'no_input_drops_or_queue_overflow': coverage_ok,
    }
    result['status'] = 'pass' if exit_status == 0 and gate_pass and coverage_ok and resource_ok \
        else 'invalid'
    if require_pass and result['status'] != 'pass':
        status = result['status']
        raise PhaseContractError(
            f'phase evidence is not a passing online gate: {status}')
    return result


def finalize(
        events: dict[str, Any], exit_status: int,
        resource_report: Path | None,
        *,
        maximum_callback_latency_seconds: float =
        DEFAULT_CALLBACK_LATENCY_SECONDS_V2,
        maximum_backlog_messages: int = DEFAULT_BACKLOG_MESSAGES_V2
        ) -> dict[str, Any]:
    events['exit_status'] = _integer(exit_status, 'exit_status', nonnegative=False)
    values = events.get('events_monotonic_ns')
    if not isinstance(values, dict):
        raise PhaseContractError('phase event map missing')
    # Shutdown is a wrapper-finalizer boundary.  It is intentionally recorded
    # after all implementation-specific save/postprocess events.
    if values.get('shutdown_start') is None:
        values['shutdown_start'] = _now_ns()
    if values.get('shutdown_end') is None:
        values['shutdown_end'] = _now_ns()
    if resource_report is not None:
        events['resource'] = read_time_report(resource_report)
    # A failure still gets an atomic evidence document, but validation is
    # deliberately strict and marks it invalid when any boundary is absent.
    try:
        if events.get('contract_version') == CONTRACT_VERSION_V2:
            return validate_evidence_v2(
                events,
                maximum_callback_latency_seconds=(
                    maximum_callback_latency_seconds),
                maximum_backlog_messages=maximum_backlog_messages)
        return validate_evidence(events)
    except PhaseContractError as error:
        result = dict(events)
        result['status'] = 'invalid'
        result['validation'] = {'status': 'invalid', 'reason': str(error)}
        result['runtime'] = {
            'online_compute_rtf': None,
            'wall_rtf_is_diagnostic_only': True}
        return result


def _write_loaded(path: Path, value: dict[str, Any]) -> None:
    atomic_replace_json(path, value)


def cli(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    init = sub.add_parser('init')
    init.add_argument('--output', type=Path, required=True)
    init.add_argument('--system', required=True)
    init.add_argument('--input-duration', type=float, required=True)
    init.add_argument('--max-end-gap', type=float, default=DEFAULT_END_GAP_SECONDS)
    init.add_argument('--contract-version', default=CONTRACT_VERSION,
                      choices=(CONTRACT_VERSION, CONTRACT_VERSION_V2))
    init.add_argument('--phase-mode', default='paced_1x', choices=PHASE_MODES_V2)
    mark = sub.add_parser('mark')
    mark.add_argument('--events', type=Path, required=True)
    mark.add_argument('--name', required=True)
    coverage = sub.add_parser('coverage')
    coverage.add_argument('--events', type=Path, required=True)
    coverage.add_argument('--mode', required=True)
    coverage.add_argument('--complete', action='store_true')
    coverage.add_argument('--last-timestamp', type=float)
    coverage.add_argument('--required-timestamp', type=float)
    coverage.add_argument('--dropped', type=int)
    coverage.add_argument('--queue-overflow', type=int)
    coverage.add_argument('--expected-messages', type=int)
    coverage.add_argument('--consumed-messages', type=int)
    consumer = sub.add_parser('consumer')
    consumer.add_argument('--events', type=Path, required=True)
    consumer.add_argument('--input', type=Path, required=True)
    consumer_state = sub.add_parser('consumer-state')
    consumer_state.add_argument('--input', type=Path, required=True)
    consumer_state.add_argument('--diagnostic-base', type=Path, required=True)
    consumer_state.add_argument('--maximum-callback-latency-seconds',
                                type=float,
                                default=DEFAULT_CALLBACK_LATENCY_SECONDS_V2)
    consumer_state.add_argument('--maximum-backlog-messages', type=int,
                                default=DEFAULT_BACKLOG_MESSAGES_V2)
    consumer_eof_state_parser = sub.add_parser('consumer-eof-state')
    consumer_eof_state_parser.add_argument('--input', type=Path, required=True)
    finalize_parser = sub.add_parser('finalize')
    finalize_parser.add_argument('--events', type=Path, required=True)
    finalize_parser.add_argument('--output', type=Path, required=True)
    finalize_parser.add_argument('--exit-status', type=int, required=True)
    finalize_parser.add_argument('--resource-report', type=Path)
    finalize_parser.add_argument('--maximum-callback-latency-seconds',
                                 type=float,
                                 default=DEFAULT_CALLBACK_LATENCY_SECONDS_V2)
    finalize_parser.add_argument('--maximum-backlog-messages', type=int,
                                 default=DEFAULT_BACKLOG_MESSAGES_V2)
    validate = sub.add_parser('validate')
    validate.add_argument('--input', type=Path, required=True)
    validate.add_argument('--require-pass', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.command == 'init':
            atomic_write_json(args.output, new_events(
                args.system, args.input_duration,
                max_end_gap_seconds=args.max_end_gap,
                contract_version=args.contract_version,
                phase_mode=args.phase_mode))
        elif args.command == 'mark':
            value = load_object(args.events)
            mark_event(value, args.name)
            _write_loaded(args.events, value)
        elif args.command == 'coverage':
            value = load_object(args.events)
            set_coverage(value, mode=args.mode, complete=args.complete,
                         last_timestamp=args.last_timestamp,
                         required_timestamp=args.required_timestamp,
                         dropped_messages=args.dropped,
                         queue_overflow=args.queue_overflow,
                         expected_messages=args.expected_messages,
                         consumed_messages=args.consumed_messages)
            _write_loaded(args.events, value)
        elif args.command == 'consumer':
            value = load_object(args.events)
            evidence = load_object(args.input)
            set_consumer_evidence(value, evidence, source_path=str(args.input))
            _write_loaded(args.events, value)
        elif args.command == 'consumer-state':
            status, value = consumer_evidence_state(
                args.input, args.diagnostic_base,
                maximum_callback_latency_seconds=(
                    args.maximum_callback_latency_seconds),
                maximum_backlog_messages=args.maximum_backlog_messages)
            print(json.dumps(value, indent=2, sort_keys=True))
            return status
        elif args.command == 'consumer-eof-state':
            status, value = consumer_eof_state(args.input)
            print(json.dumps(value, indent=2, sort_keys=True))
            return status
        elif args.command == 'finalize':
            value = finalize(load_object(args.events), args.exit_status,
                             args.resource_report,
                             maximum_callback_latency_seconds=(
                                 args.maximum_callback_latency_seconds),
                             maximum_backlog_messages=(
                                 args.maximum_backlog_messages))
            atomic_write_json(args.output, value)
            return 0 if value.get('status') == 'pass' else 2
        else:
            document = load_object(args.input)
            if document.get('contract_version') == CONTRACT_VERSION_V5:
                validator = validate_terminal_support_context_v5
            elif document.get('schema_version') == SCHEMA_VERSION_V3 or \
                    document.get('contract_version') == CONTRACT_VERSION_V3:
                validator = validate_terminal_support_context_v3
            else:
                validator = (validate_evidence_v2
                             if document.get('contract_version') == CONTRACT_VERSION_V2
                             else validate_evidence)
            value = validator(document, require_pass=args.require_pass)
            print(json.dumps(value, indent=2, sort_keys=True))
        return 0
    except (PhaseContractError, OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(cli())
