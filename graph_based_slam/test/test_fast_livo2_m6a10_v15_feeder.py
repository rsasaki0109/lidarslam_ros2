# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Host-only contract tests for the additive v15 feeder adapter."""

import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'scripts/fast_livo2_m6a10_v15_feeder.py'
LEGACY_SOURCE = ROOT / 'scripts/fast_livo2_m6a10_feeder.py'
V12_FIXTURE = ROOT / \
    'graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json'


def _module():
    spec = importlib.util.spec_from_file_location(
        'm6a10_v15_feeder_test', SOURCE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _consumer_v5(status='invalid', received=0, acked=0, lidar=0,
                 imu=0, image=0, expected_lidar=1, expected_imu=0,
                 expected_image=0, mapper_current=0, mapper_peak=0,
                 transport_peak=0, eof=False, drain=False):
    return {
        'ack_source_kind': 'consumer_callback',
        'ack_source': 'LIVMapper subscriber callback return',
        'ack_semantics': 'callback_acceptance_not_backend_completion',
        'publisher_count_used': False,
        'eof_observed': eof,
        'eof_source': '/m6a10/consumer_eof service',
        'expected_messages': expected_lidar + expected_imu + expected_image,
        'expected_topic_counts': {
            'lidar': expected_lidar, 'imu': expected_imu, 'image': expected_image,
        },
        'received_messages': received,
        'processed_messages': received,
        'acked_messages': acked,
        'received_topic_counts': {'lidar': lidar, 'imu': imu, 'image': image},
        'dropped_messages': 0,
        'queue_overflow': 0,
        'queue_overflow_observable': status == 'pass',
        'processing_failures': 0,
        'backlog_at_drain': mapper_current,
        'maximum_backlog_messages': mapper_peak,
        'maximum_allowed_backlog_messages': 1,
        'transport_outstanding_at_drain': received - acked,
        'maximum_transport_outstanding_messages': transport_peak,
        'maximum_allowed_transport_outstanding_messages': 1,
        'mapper_internal_deque_current_messages': mapper_current,
        'mapper_internal_deque_peak_messages': mapper_peak,
        'ack_exact': acked == received == (
            expected_lidar + expected_imu + expected_image),
        'maximum_callback_latency_seconds': 0.000001,
        'maximum_allowed_callback_latency_seconds': 0.25,
        'first_processed_timestamp_seconds': 10.0 if received else None,
        'last_processed_timestamp_seconds': 10.0 if received else None,
        'required_end_timestamp_seconds': -1,
        'paced_input_rate': 1.0,
        'paced_input_rate_verified': False,
        'ack_backpressure_verified': status == 'pass',
        'acknowledgement_contract':
            'one_publish_waits_for_callback_then_one_ack_service_call',
        'queue_capacity_messages': 1,
        'queue_drop_detection': 'exact_counts_plus_single_inflight_ack',
        'single_message_buffer_verified': False,
        'counter_evidence_path': '/out/consumer_evidence.json',
        'drain_complete': drain,
    }


def _v5(status='invalid', **kwargs):
    module = _module()
    value = {
        'schema_version': 3,
        'contract_version': module.V5_CONTRACT,
        'transport_contract_version': module.V5_TRANSPORT_CONTRACT,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': status,
        'benchmark_only': True,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'consumer': _consumer_v5(status=status, **kwargs),
    }
    return value


def _legacy():
    return {
        'schema_version': 2,
        'contract_version': 'm6a10-online-compute-v2',
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'consumer': {
            'received_messages': 0,
            'received_topic_counts': {'lidar': 0, 'imu': 0, 'image': 0},
            'dropped_messages': 0,
            'processing_failures': 0,
            'queue_overflow': 0,
            'backlog_at_drain': 0,
            'maximum_backlog_messages': 0,
            'maximum_callback_latency_seconds': 0.0,
        },
    }


def test_schema2_legacy_status_and_cli_boundary_remain_supported():
    module = _module()
    assert module.validate_consumer_status(_legacy())['schema_version'] == 2
    source = SOURCE.read_text(encoding='utf-8')
    assert 'legacy.main()' in source
    assert 'legacy.wait_for_drain = _wait_for_drain' in source
    assert 'v5_transport_outstanding_only' in source
    assert '--bag' not in source  # parser/CLI remains the delegated legacy CLI
    assert 'fast_livo2_m6a10_feeder_legacy' in source


def test_v12_schema3_real_shape_passes_with_mapper_tail_diagnostic():
    module = _module()
    value = json.loads(V12_FIXTURE.read_text(encoding='utf-8'))
    assert module.validate_consumer_status(
        value)['consumer']['mapper_internal_deque_peak_messages'] == 81


def test_v12_schema3_initial_invalid_snapshot_passes_dispatch():
    module = _module()
    value = _v5()
    assert module.validate_consumer_status(value)['status'] == 'invalid'


def test_status_call_uses_bounded_injected_rpc_and_proceeds_to_next_step():
    module = _module()
    value = _v5(status='pass', received=1, acked=1, lidar=1,
                mapper_current=0, mapper_peak=0, transport_peak=1,
                eof=True, drain=True)
    response = type('Response', (), {
        'success': True, 'message': json.dumps(value),
    })()
    calls = []

    def bounded(proxy, deadline, operation):
        calls.append((proxy, deadline, operation))
        return response

    result = module._status('status-proxy', 'deadline', bounded)
    assert result['contract_version'] == module.V5_CONTRACT
    assert calls == [('status-proxy', 'deadline', 'consumer status RPC')]


def test_sealed_v14_failure_shape_would_be_rejected_by_legacy_only_gate_but_v15_accepts():
    module = _module()
    value = _v5(status='invalid')

    def immutable_v14_gate(document):
        if (document.get('schema_version') != 2 or
                document.get('system') != 'fast_livo2'):
            raise RuntimeError('consumer status schema/system mismatch')

    with pytest.raises(RuntimeError, match='schema/system mismatch'):
        immutable_v14_gate(value)
    assert module.validate_consumer_status(value)['schema_version'] == 3


@pytest.mark.parametrize('mutator', [
    lambda value: value.pop('transport_contract_version'),
    lambda value: value.__setitem__('transport_contract_version', 'wrong'),
    lambda value: value.__setitem__('phase_mode', 'paced_1x'),
    lambda value: value.__setitem__('system', 'ours'),
    lambda value: value['consumer'].__setitem__('transport_outstanding_at_drain', 2),
    lambda value: value['consumer'].__setitem__('maximum_transport_outstanding_messages', 2),
    lambda value: value['consumer'].__setitem__('received_messages', 2),
    lambda value: value['consumer'].__setitem__('acked_messages', 2),
    lambda value: value['consumer'].__setitem__('ack_source', 'publisher_count'),
    lambda value: value['consumer'].__setitem__('mapper_internal_deque_current_messages', 2),
])
def test_schema3_contract_and_counter_negative_cases(mutator):
    module = _module()
    value = _v5(status='pass', received=1, acked=1, lidar=1,
                mapper_current=0, mapper_peak=0, transport_peak=1,
                eof=True, drain=True)
    mutator(value)
    with pytest.raises(module.ConsumerStatusError):
        module.validate_consumer_status(value)


def test_unknown_schema_and_schema2_v5_field_mixing_fail_closed():
    module = _module()
    unknown = _legacy()
    unknown['schema_version'] = 99
    with pytest.raises(module.ConsumerStatusError, match='unknown'):
        module.validate_consumer_status(unknown)

    mixed = _legacy()
    mixed['transport_contract_version'] = module.V5_TRANSPORT_CONTRACT
    with pytest.raises(module.ConsumerStatusError, match='schema-3 transport'):
        module.validate_consumer_status(mixed)

    mixed = _legacy()
    mixed['consumer']['ack_exact'] = True
    with pytest.raises(module.ConsumerStatusError, match='schema-3 transport'):
        module.validate_consumer_status(mixed)


def test_v5_support_is_non_lidar_only_and_conserves_counts():
    module = _module()
    value = _v5()
    value['consumer']['terminal_support_context'] = {
        'classification': 'nonlidar_at_or_after_boundary_support_context',
        'total_count': 2,
        'by_topic': {'lidar': 0, 'imu': 1, 'image': 1},
    }
    assert module.validate_consumer_status(value)['schema_version'] == 3

    invalid = copy.deepcopy(value)
    invalid['consumer']['terminal_support_context']['by_topic']['lidar'] = 1
    with pytest.raises(module.ConsumerStatusError):
        module.validate_consumer_status(invalid)


def test_v5_status_requires_final_ack_eof_and_zero_failures_for_pass():
    module = _module()
    value = _v5(status='pass', received=1, acked=0, lidar=1,
                mapper_current=0, mapper_peak=0, transport_peak=1,
                eof=True, drain=True)
    with pytest.raises(module.ConsumerStatusError, match='exact ACK'):
        module.validate_consumer_status(value)


def test_source_is_lazy_ros_import_and_python38_friendly():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    imported = {node.names[0].name for node in tree.body
                if isinstance(node, ast.Import)}
    assert 'rosbag' not in imported
    assert 'rospy' not in imported
    source = SOURCE.read_text(encoding='utf-8')
    assert ' | ' not in source
    assert 'dict[' not in source
    assert LEGACY_SOURCE.is_file()
    assert hashlib.sha256(LEGACY_SOURCE.read_bytes()).hexdigest() == \
        '1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831'
