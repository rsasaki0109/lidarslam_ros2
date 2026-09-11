#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Additive v15 feeder adapter for the FAST-LIVO2 M6a10 benchmark.  The
# immutable v14 feeder remains schema-2-only.  This module keeps its command
# line and publishing implementation, while adding an explicit schema-2 /
# schema-3 consumer-status dispatch at the status boundary.

"""Schema-compatible M6a10 feeder adapter.

The v12 production consumer emits a schema-3 document for the v5 transport
contract.  The historical feeder accepted only schema 2 and therefore failed
before its first callback.  v15 keeps the historical implementation in a
separate module and replaces only its status validator.  ROS and rosbag are
imported lazily by the legacy implementation, so the contract validator is
safe to exercise in host-only Python tests.
"""

from __future__ import annotations

import importlib
import json
import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional


SCHEMA_VERSION_LEGACY = 2
SCHEMA_VERSION_V5 = 3
SYSTEM_ID = 'fast_livo2'
LEGACY_CONTRACTS = frozenset((
    'm6a10-online-compute-v2',
    'm6a10-online-compute-v3-terminal-support-context',
    'm6a10-online-compute-v4-terminal-bounded-end-gap',
))
V5_CONTRACT = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
V5_TRANSPORT_CONTRACT = 'm6a10-v12-callback-ack-transport-outstanding-v1'
V5_PHASE_MODE = 'unpaced_ack'
TOPIC_KEYS = ('lidar', 'imu', 'image')

LEGACY_CONSUMER_FIELDS = frozenset((
    'received_messages', 'received_topic_counts', 'dropped_messages',
    'processing_failures', 'queue_overflow', 'backlog_at_drain',
    'maximum_backlog_messages', 'maximum_callback_latency_seconds',
))
V5_CONSUMER_FIELDS = frozenset((
    'ack_source_kind', 'ack_source', 'ack_semantics', 'publisher_count_used',
    'eof_observed', 'eof_source', 'expected_messages',
    'expected_topic_counts', 'received_messages', 'processed_messages',
    'acked_messages', 'received_topic_counts', 'dropped_messages',
    'queue_overflow', 'queue_overflow_observable', 'processing_failures',
    'backlog_at_drain', 'maximum_backlog_messages',
    'maximum_allowed_backlog_messages', 'transport_outstanding_at_drain',
    'maximum_transport_outstanding_messages',
    'maximum_allowed_transport_outstanding_messages',
    'mapper_internal_deque_current_messages',
    'mapper_internal_deque_peak_messages', 'ack_exact',
    'maximum_callback_latency_seconds',
    'maximum_allowed_callback_latency_seconds',
    'first_processed_timestamp_seconds', 'last_processed_timestamp_seconds',
    'required_end_timestamp_seconds', 'paced_input_rate',
    'paced_input_rate_verified', 'ack_backpressure_verified',
    'acknowledgement_contract', 'queue_capacity_messages',
    'queue_drop_detection', 'single_message_buffer_verified',
    'counter_evidence_path', 'drain_complete',
))
V5_ONLY_FIELDS = frozenset((
    'transport_contract_version', 'transport_outstanding_at_drain',
    'maximum_transport_outstanding_messages',
    'maximum_allowed_transport_outstanding_messages',
    'mapper_internal_deque_current_messages',
    'mapper_internal_deque_peak_messages', 'acked_messages', 'ack_exact',
    'ack_source_kind', 'ack_source', 'ack_semantics',
    'acknowledgement_contract', 'queue_capacity_messages',
    'queue_drop_detection',
))


class ConsumerStatusError(RuntimeError):
    """Raised when a consumer document cannot be trusted by the feeder."""


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConsumerStatusError('%s must be an object' % name)
    return value


def _require_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ConsumerStatusError('%s must be boolean' % name)
    return value


def _require_nonnegative_int(value: Any, name: str) -> int:
    # bool is an int subclass, but accepting it here would make a malformed
    # JSON document look like a valid counter.
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ConsumerStatusError('%s must be a non-negative integer' % name)
    return value


def _require_number(value: Any, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConsumerStatusError('%s must be numeric' % name)
    number = float(value)
    if not math.isfinite(number) or (nonnegative and number < 0.0):
        raise ConsumerStatusError('%s must be finite%s' % (
            name, ' and non-negative' if nonnegative else ''))
    return number


def _require_topics(value: Any, name: str) -> Dict[str, int]:
    source = _require_mapping(value, name)
    if set(source) != set(TOPIC_KEYS):
        raise ConsumerStatusError('%s must contain exactly lidar/imu/image' % name)
    result = {}
    for topic in TOPIC_KEYS:
        result[topic] = _require_nonnegative_int(source[topic], '%s.%s' % (name, topic))
    return result


def _reject_v5_fields(value: Mapping[str, Any], consumer: Mapping[str, Any]) -> None:
    top_level = set(value).intersection(V5_ONLY_FIELDS)
    consumer_level = set(consumer).intersection(V5_ONLY_FIELDS)
    if top_level or consumer_level:
        fields = sorted(top_level.union(consumer_level))
        raise ConsumerStatusError(
            'schema-2 document contains schema-3 transport fields: %s' %
            ','.join(fields))


def _validate_legacy(value: Mapping[str, Any]) -> Mapping[str, Any]:
    """Preserve the old schema-2 checks, with an explicit version boundary."""
    if value.get('system') != SYSTEM_ID:
        raise ConsumerStatusError('consumer status schema/system mismatch')
    contract = value.get('contract_version')
    if contract is not None and contract not in LEGACY_CONTRACTS:
        raise ConsumerStatusError('schema-2 contract is not a legacy contract')
    consumer = _require_mapping(value.get('consumer'), 'consumer status consumer')
    _reject_v5_fields(value, consumer)
    if any(key not in consumer for key in LEGACY_CONSUMER_FIELDS):
        raise ConsumerStatusError('consumer status lacks required counters')
    # These are the only shape checks added to the historical path.  The
    # legacy count semantics remain in the old feeder's _counts function.
    _require_mapping(consumer.get('received_topic_counts'),
                     'consumer.received_topic_counts')
    return value


def _validate_support_and_buffers(consumer: Mapping[str, Any]) -> None:
    """Validate optional v5 support/buffer views without inventing fields.

    Consumer v5 production output keeps terminal support in the terminal
    document, so these views are optional here.  If a producer supplies them,
    their classification and topic conservation must still be fail-closed.
    """
    for key in ('terminal_support_context', 'support_context'):
        if key not in consumer:
            continue
        support = _require_mapping(consumer[key], 'consumer.%s' % key)
        classification = support.get('classification')
        if classification != 'nonlidar_at_or_after_boundary_support_context':
            raise ConsumerStatusError('v5 support classification mismatch')
        by_topic = _require_topics(support.get('by_topic'),
                                   'consumer.%s.by_topic' % key)
        if by_topic['lidar'] != 0:
            raise ConsumerStatusError('LiDAR cannot be support context')
        total = _require_nonnegative_int(
            support.get('total_count'), 'consumer.%s.total_count' % key)
        if total != by_topic['imu'] + by_topic['image']:
            raise ConsumerStatusError('support topic counts do not conserve')

    for key in ('buffers', 'buffer'):
        if key not in consumer:
            continue
        buffers = _require_mapping(consumer[key], 'consumer.%s' % key)
        if set(buffers) != set(TOPIC_KEYS):
            raise ConsumerStatusError('v5 buffers must contain exactly three topics')
        for topic in TOPIC_KEYS:
            entry = _require_mapping(buffers[topic],
                                     'consumer.%s.%s' % (key, topic))
            count = _require_nonnegative_int(entry.get('count'),
                                             'consumer.%s.%s.count' % (key, topic))
            support_count = entry.get('support_count', 0)
            support_count = _require_nonnegative_int(
                support_count, 'consumer.%s.%s.support_count' % (key, topic))
            if support_count > count:
                raise ConsumerStatusError('support buffer count exceeds buffer count')
            proven = entry.get('support_context_proven')
            if proven is not None:
                proven = _require_bool(
                    proven, 'consumer.%s.%s.support_context_proven' % (key, topic))
                if topic == 'lidar' and proven:
                    raise ConsumerStatusError('LiDAR support context is forbidden')


def _validate_v5(value: Mapping[str, Any]) -> Mapping[str, Any]:
    if value.get('system') != SYSTEM_ID:
        raise ConsumerStatusError('schema-3 system mismatch')
    if value.get('contract_version') != V5_CONTRACT:
        raise ConsumerStatusError('schema-3 contract mismatch')
    if value.get('transport_contract_version') != V5_TRANSPORT_CONTRACT:
        raise ConsumerStatusError('schema-3 transport contract mismatch')
    if value.get('phase_mode') != V5_PHASE_MODE:
        raise ConsumerStatusError('schema-3 phase mode mismatch')
    if value.get('status') not in ('invalid', 'pass'):
        raise ConsumerStatusError('schema-3 status must be invalid or pass')
    if value.get('benchmark_only') is not True:
        raise ConsumerStatusError('schema-3 benchmark_only must be true')
    for field in ('ground_truth_content_opened', 'scorer_invoked'):
        if field in value and _require_bool(value[field], field):
            raise ConsumerStatusError('%s must be false' % field)
    # The v12 emitter never includes a legacy contract_id.  Rejecting it
    # prevents a host document that mixes schema-1 terminal authority with
    # callback transport authority.
    if 'contract_id' in value:
        raise ConsumerStatusError('schema-3 document contains legacy contract_id')

    consumer = _require_mapping(value.get('consumer'), 'consumer status consumer')
    if any(key not in consumer for key in V5_CONSUMER_FIELDS):
        raise ConsumerStatusError('schema-3 consumer lacks required v5 fields')
    expected = _require_topics(consumer['expected_topic_counts'],
                               'consumer.expected_topic_counts')
    received = _require_topics(consumer['received_topic_counts'],
                               'consumer.received_topic_counts')
    expected_total = _require_nonnegative_int(
        consumer['expected_messages'], 'consumer.expected_messages')
    received_total = _require_nonnegative_int(
        consumer['received_messages'], 'consumer.received_messages')
    processed_total = _require_nonnegative_int(
        consumer['processed_messages'], 'consumer.processed_messages')
    acked_total = _require_nonnegative_int(
        consumer['acked_messages'], 'consumer.acked_messages')
    if expected_total != sum(expected.values()):
        raise ConsumerStatusError('expected topic counts do not conserve')
    if received_total != sum(received.values()):
        raise ConsumerStatusError('received topic counts do not conserve')
    if processed_total != received_total:
        raise ConsumerStatusError('processed count differs from received count')
    if acked_total > received_total:
        raise ConsumerStatusError('ACK count exceeds received count')
    # Some transport adapters expose the callback's accepted counter
    # explicitly; the production v12 header derives it from received counts
    # and therefore omits the optional fields.  When present, both forms must
    # agree exactly rather than silently becoming a second authority.
    if 'accepted_messages' in consumer:
        accepted_total = _require_nonnegative_int(
            consumer['accepted_messages'], 'consumer.accepted_messages')
        if accepted_total != received_total:
            raise ConsumerStatusError('accepted count differs from received count')
    if 'accepted_topic_counts' in consumer:
        accepted = _require_topics(consumer['accepted_topic_counts'],
                                   'consumer.accepted_topic_counts')
        if accepted != received:
            raise ConsumerStatusError('accepted topic counts differ from received')

    for field in ('dropped_messages', 'queue_overflow', 'processing_failures',
                  'backlog_at_drain', 'maximum_backlog_messages',
                  'maximum_allowed_backlog_messages',
                  'transport_outstanding_at_drain',
                  'maximum_transport_outstanding_messages',
                  'maximum_allowed_transport_outstanding_messages',
                  'mapper_internal_deque_current_messages',
                  'mapper_internal_deque_peak_messages',
                  'queue_capacity_messages'):
        _require_nonnegative_int(consumer[field], 'consumer.%s' % field)
    if consumer['maximum_allowed_transport_outstanding_messages'] != 1:
        raise ConsumerStatusError('v5 transport outstanding limit must be one')
    if consumer['maximum_transport_outstanding_messages'] > 1:
        raise ConsumerStatusError('v5 transport outstanding peak exceeds one')
    if consumer['transport_outstanding_at_drain'] != received_total - acked_total:
        raise ConsumerStatusError('transport outstanding is not received minus ACK')
    if consumer['mapper_internal_deque_current_messages'] > \
            consumer['mapper_internal_deque_peak_messages']:
        raise ConsumerStatusError('mapper deque current exceeds its peak')
    if consumer['backlog_at_drain'] != consumer['mapper_internal_deque_current_messages']:
        raise ConsumerStatusError('mapper deque diagnostic differs from backlog')
    if consumer['maximum_backlog_messages'] != \
            consumer['mapper_internal_deque_peak_messages']:
        raise ConsumerStatusError('mapper deque peak differs from backlog peak')

    if consumer['publisher_count_used'] is not False:
        raise ConsumerStatusError('publisher counts cannot be transport evidence')
    for field in ('eof_observed', 'queue_overflow_observable',
                  'ack_backpressure_verified', 'paced_input_rate_verified',
                  'single_message_buffer_verified', 'drain_complete', 'ack_exact'):
        _require_bool(consumer[field], 'consumer.%s' % field)
    if consumer['ack_source_kind'] != 'consumer_callback':
        raise ConsumerStatusError('ACK source kind mismatch')
    if consumer['ack_source'] != 'LIVMapper subscriber callback return':
        raise ConsumerStatusError('ACK source mismatch')
    if consumer['ack_semantics'] != 'callback_acceptance_not_backend_completion':
        raise ConsumerStatusError('ACK semantics mismatch')
    if consumer['acknowledgement_contract'] != \
            'one_publish_waits_for_callback_then_one_ack_service_call':
        raise ConsumerStatusError('ACK contract mismatch')
    if consumer['queue_drop_detection'] != 'exact_counts_plus_single_inflight_ack':
        raise ConsumerStatusError('queue drop detection mismatch')
    if consumer['queue_capacity_messages'] != 1:
        raise ConsumerStatusError('queue capacity must be one')
    if not isinstance(consumer['eof_source'], str) or not consumer['eof_source']:
        raise ConsumerStatusError('EOF source missing')
    if not isinstance(consumer['counter_evidence_path'], str) or \
            not consumer['counter_evidence_path']:
        raise ConsumerStatusError('counter evidence path missing')

    _require_number(consumer['maximum_callback_latency_seconds'],
                    'consumer.maximum_callback_latency_seconds', nonnegative=True)
    allowed_latency = _require_number(
        consumer['maximum_allowed_callback_latency_seconds'],
        'consumer.maximum_allowed_callback_latency_seconds', nonnegative=True)
    if consumer['maximum_callback_latency_seconds'] > allowed_latency:
        raise ConsumerStatusError('callback latency exceeds bound')
    for field in ('first_processed_timestamp_seconds',
                  'last_processed_timestamp_seconds',
                  'required_end_timestamp_seconds'):
        if consumer[field] is not None:
            _require_number(consumer[field], 'consumer.%s' % field)
    if consumer['paced_input_rate'] is not None:
        _require_number(consumer['paced_input_rate'], 'consumer.paced_input_rate',
                        nonnegative=True)

    # A status snapshot before the first ACK is deliberately invalid.  A
    # final pass must prove exact ACK, EOF, zero transport outstanding, and
    # all backpressure gates; this is the boundary the old feeder never saw.
    if value['status'] == 'pass':
        if not consumer['ack_exact'] or acked_total != expected_total:
            raise ConsumerStatusError('pass status lacks exact ACK')
        if not consumer['eof_observed'] or not consumer['drain_complete']:
            raise ConsumerStatusError('pass status lacks EOF/drain evidence')
        if consumer['transport_outstanding_at_drain'] != 0:
            raise ConsumerStatusError('pass status has transport outstanding')
        if not consumer['queue_overflow_observable'] or \
                not consumer['ack_backpressure_verified']:
            raise ConsumerStatusError('pass status lacks ACK backpressure proof')
        if any(consumer[field] != 0 for field in (
                'dropped_messages', 'queue_overflow', 'processing_failures')):
            raise ConsumerStatusError('pass status contains a failure counter')

    # The callback producer does not normally emit support/buffer views, but
    # if an adapter carries them forward they must obey the v5 non-LiDAR rule.
    _validate_support_and_buffers(consumer)
    return value


def validate_consumer_status(value: Any) -> Mapping[str, Any]:
    """Dispatch and validate a decoded consumer status document."""
    value = _require_mapping(value, 'consumer status')
    schema = value.get('schema_version')
    if schema == SCHEMA_VERSION_LEGACY:
        return _validate_legacy(value)
    if schema == SCHEMA_VERSION_V5:
        return _validate_v5(value)
    raise ConsumerStatusError('unknown consumer status schema_version')


def _status(proxy: Any, deadline: Any,
            bounded_call: Optional[Callable[..., Any]] = None) -> Dict[str, Any]:
    """Call the real bounded RPC and dispatch schema 2 or schema 3."""
    if bounded_call is None:
        legacy = importlib.import_module('fast_livo2_m6a10_feeder_legacy')
        bounded_call = legacy._bounded_call
    response = bounded_call(proxy, deadline, 'consumer status RPC')
    if not response.success:
        raise ConsumerStatusError('consumer status rejected: %s' % response.message)
    try:
        value = json.loads(response.message)
    except (TypeError, ValueError) as error:
        raise ConsumerStatusError('consumer status was not JSON') from error
    return dict(validate_consumer_status(value))


def _legacy_module() -> Any:
    """Load the immutable feeder only when the production CLI is invoked."""
    return importlib.import_module('fast_livo2_m6a10_feeder_legacy')


def _wait_for_drain(output: Path, timeout: float,
                    eof_already_observed: bool = False) -> Dict[str, Any]:
    """Drain using transport outstanding for v5, mapper backlog for v2-v4.

    The immutable feeder's drain helper predates v5 and treats the mapper's
    internal deque as the transport queue.  v5 intentionally exposes that
    deque as a diagnostic (it may contain the 81-message support tail), so a
    v15 drain waits for ``received - ACK`` instead.  Schema-2/legacy behavior
    remains byte-for-byte delegated to the old predicate.
    """
    legacy = _legacy_module()
    legacy.rospy.init_node('m6a10_fast_livo2_drainer', anonymous=True,
                           disable_signals=True)
    deadline = legacy.Deadline(timeout)
    status_proxy, _, eof_proxy = legacy.wait_for_services(deadline)
    checkpoint = legacy.drain_progress_path(output)
    zero = {key: 0 for key in legacy.TOPIC_KEYS.values()}
    legacy.write_progress(checkpoint, phase='drain_services_ready', records=0,
                          published=zero, acked=zero)
    if not eof_already_observed:
        eof_response = legacy._bounded_call(eof_proxy, deadline,
                                            'consumer EOF RPC')
        if not eof_response.success:
            raise RuntimeError('consumer EOF rejected: %s' % eof_response.message)
    latest = None
    while not legacy.rospy.is_shutdown():
        deadline.remaining('consumer drain status polling')
        latest = _status(status_proxy, deadline, legacy._bounded_call)
        consumer = latest['consumer']
        if latest.get('schema_version') == SCHEMA_VERSION_V5:
            remaining = consumer['transport_outstanding_at_drain']
        else:
            remaining = consumer['backlog_at_drain']
        if int(remaining) == 0:
            break
        time.sleep(0.05)
    if latest is None:
        raise RuntimeError('consumer drain did not produce a status')
    consumer = latest['consumer']
    remaining = (consumer['transport_outstanding_at_drain']
                 if latest.get('schema_version') == SCHEMA_VERSION_V5
                 else consumer['backlog_at_drain'])
    if int(remaining) != 0:
        raise legacy.DeadlineExceeded('consumer transport did not drain to zero')
    legacy.write_progress(checkpoint, phase='drain_complete', records=0,
                          published=zero, acked=zero)
    receipt = {
        'schema_version': 1,
        'contract_id': 'm6a10-v2c-fast-livo2-drain-v1',
        'status': 'pass',
        'eof_service': legacy.EOF_SERVICE,
        'drain_complete': True,
        'backlog_at_drain': int(consumer['backlog_at_drain']),
        'consumer_status': latest,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    if latest.get('schema_version') == SCHEMA_VERSION_V5:
        receipt['transport_outstanding_at_drain'] = int(
            consumer['transport_outstanding_at_drain'])
        receipt['drain_semantics'] = 'v5_transport_outstanding_only'
    legacy.atomic_json(output, receipt)
    return latest


def main() -> int:
    """Preserve the legacy CLI and publisher/drainer implementation."""
    legacy = _legacy_module()
    legacy._status = lambda proxy, deadline: _status(
        proxy, deadline, legacy._bounded_call)
    legacy.wait_for_drain = _wait_for_drain
    return legacy.main()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        # Keep the old immutable diagnostic contract when running in the
        # image.  Import failure is also reported without touching input data.
        try:
            legacy = _legacy_module()
            argv = __import__('sys').argv
            output = (argv[argv.index('--output') + 1]
                      if '--output' in argv and argv.index('--output') + 1 < len(argv)
                      else '.')
            legacy.write_failure_diagnostic(__import__('pathlib').Path(output), error)
        except Exception:
            pass
        print('FAST M6a10 feeder error: %s' % error, flush=True)
        raise SystemExit(2)
