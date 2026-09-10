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

"""Synthetic, GT-free tests for the M6a10 phase contract."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'benchmark_phase_contract', ROOT / 'scripts/benchmark_phase_contract.py')
PHASE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(PHASE)


def _passing_events():
    value = PHASE.new_events('ours', 10.0)
    stamps = {name: (index + 1) * 1_000_000_000
              for index, name in enumerate(PHASE.EVENT_NAMES)}
    value['events_monotonic_ns'] = stamps
    PHASE.set_coverage(
        value, mode='trajectory_timestamp_coverage', complete=True,
        last_timestamp=109.9, required_timestamp=110.0,
        dropped_messages=0, queue_overflow=0)
    value['resource'] = {
        'status': 'verified', 'scope': 'wrapper_process',
        'cpu_user_seconds': 1.0, 'cpu_system_seconds': 0.5,
        'io_input_operations': 10, 'io_output_operations': 20,
        'source': 'fixture',
    }
    value['exit_status'] = 0
    return value


def _passing_v2_events(mode='paced_1x'):
    value = PHASE.new_events(
        'ours', 10.0, contract_version=PHASE.CONTRACT_VERSION_V2,
        phase_mode=mode)
    stamps = {name: (index + 1) * 1_000_000_000
              for index, name in enumerate(PHASE.EVENT_NAMES)}
    value['events_monotonic_ns'] = stamps
    PHASE.set_coverage(
        value, mode='processed_message_counts', complete=True,
        last_timestamp=None, required_timestamp=None,
        expected_messages=4, consumed_messages=4,
        dropped_messages=0, queue_overflow=0)
    value['consumer'] = {
        'ack_source_kind': 'consumer_callback',
        'ack_source': 'fixture.consumer_callback',
        'ack_semantics': 'entry_exit_ack',
        'publisher_count_used': False,
        'eof_observed': True,
        'eof_source': 'fixture.eof_marker',
        'expected_messages': 4,
        'received_messages': 4,
        'processed_messages': 4,
        'dropped_messages': 0,
        'queue_overflow': 0,
        'backlog_at_drain': 0,
        'maximum_backlog_messages': 0,
        'first_processed_timestamp_seconds': 100.0,
        'last_processed_timestamp_seconds': 109.9,
        'required_end_timestamp_seconds': 110.0,
        'maximum_callback_latency_seconds': 0.1,
        'paced_input_rate': 1.0,
        'paced_input_rate_verified': mode == 'paced_1x',
        'ack_backpressure_verified': mode == 'unpaced_ack',
        'single_message_buffer_verified': False,
        'counter_evidence_path': 'fixture.consumer.json',
        'drain_complete': True,
        'processing_failures': 0,
    }
    value['status'] = 'pass'
    value['resource'] = {
        'status': 'verified', 'scope': 'wrapper_process',
        'cpu_user_seconds': 1.0, 'cpu_system_seconds': 0.5,
        'io_input_operations': 10, 'io_output_operations': 20,
        'source': 'fixture',
    }
    value['exit_status'] = 0
    return value


def test_online_compute_rtf_excludes_startup_save_and_shutdown():
    value = _passing_events()
    checked = PHASE.validate_evidence(value, require_pass=True)
    # input_start=3s, drain_end=6s, so 3/10=0.3.  The wall interval is
    # intentionally longer and must remain diagnostic only.
    assert checked['runtime']['online_compute_rtf'] == pytest.approx(0.3)
    assert checked['runtime']['wall_rtf_is_diagnostic_only'] is True
    assert checked['status'] == 'pass'


def test_unpaced_high_water_uses_explicit_preregistered_backlog_bound():
    value = _passing_v2_events(mode='unpaced_ack')
    value['consumer']['maximum_backlog_messages'] = 7
    checked = PHASE.validate_consumer_evidence_v2(
        value, maximum_backlog_messages=7)
    assert checked['status'] == 'pass'
    with pytest.raises(PHASE.PhaseContractError):
        PHASE.validate_consumer_evidence_v2(
            value, maximum_backlog_messages=6)


def test_finalize_uses_the_same_preregistered_backlog_bound_as_consumer():
    value = _passing_v2_events(mode='unpaced_ack')
    value['consumer']['maximum_backlog_messages'] = 7
    checked = PHASE.finalize(
        value, 0, None, maximum_backlog_messages=7)
    assert checked['status'] == 'pass'
    assert checked['validation']['maximum_allowed_backlog_messages'] == 7

    value = _passing_v2_events(mode='unpaced_ack')
    value['consumer']['maximum_backlog_messages'] = 7
    invalid = PHASE.finalize(value, 0, None)
    assert invalid['status'] == 'invalid'
    assert invalid['validation']['consumer_reason'] == (
        'consumer_backlog_bound_exceeded')


@pytest.mark.parametrize('field', ['input_start', 'drain_end', 'save_start'])
def test_phase_order_is_fail_closed(field):
    value = _passing_events()
    value['events_monotonic_ns'][field] = 1
    with pytest.raises(PHASE.PhaseContractError):
        PHASE.validate_evidence(value)


@pytest.mark.parametrize('key', ['dropped_messages', 'queue_overflow'])
def test_drop_and_queue_overflow_are_fail_closed(key):
    value = _passing_events()
    value['coverage'][key] = 1
    checked = PHASE.validate_evidence(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['coverage_passed'] is False


def test_unmeasured_drop_and_queue_counters_are_not_assumed_zero():
    value = _passing_events()
    PHASE.set_coverage(
        value, mode='trajectory_timestamp_coverage', complete=True,
        last_timestamp=109.9, required_timestamp=110.0)
    checked = PHASE.validate_evidence(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['coverage_reason'] == 'dropped_messages_missing'


def test_trajectory_coverage_gap_is_fail_closed():
    value = _passing_events()
    value['coverage']['end_gap_seconds'] = 0.5
    checked = PHASE.validate_evidence(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['coverage_reason'] == \
        'trajectory_end_gap_exceeds_limit'


def test_processed_message_counts_require_exact_consumption():
    value = _passing_events()
    PHASE.set_coverage(value, mode='processed_message_counts', complete=True,
                       last_timestamp=None, required_timestamp=None,
                       expected_messages=4, consumed_messages=3,
                       dropped_messages=0, queue_overflow=0)
    assert PHASE.validate_evidence(value)['status'] == 'invalid'


def test_atomic_event_updates_and_final_evidence(tmp_path):
    events_path = tmp_path / 'events.json'
    output_path = tmp_path / 'phase_evidence.json'
    resource = tmp_path / 'time.txt'
    resource.write_text(
        'User time (seconds): 1.0\nSystem time (seconds): 0.5\n'
        'File system inputs: 10\nFile system outputs: 20\n')
    initial = PHASE.new_events('glim_cpu', 10.0)
    initial['events_monotonic_ns']['startup_start'] = 1_000_000_000
    PHASE.atomic_write_json(events_path, initial)
    for index, name in enumerate(PHASE.EVENT_NAMES[1:], 2):
        value = PHASE.load_object(events_path)
        PHASE.mark_event(value, name, index * 1_000_000_000)
        PHASE.atomic_replace_json(events_path, value)
    value = PHASE.load_object(events_path)
    PHASE.set_coverage(value, mode='trajectory_timestamp_coverage', complete=True,
                       last_timestamp=109.9, required_timestamp=110.0,
                       dropped_messages=0, queue_overflow=0)
    PHASE.atomic_replace_json(events_path, value)
    result = PHASE.finalize(PHASE.load_object(events_path), 0, resource)
    PHASE.atomic_write_json(output_path, result)
    assert json.loads(output_path.read_text())['status'] == 'pass'
    with pytest.raises(PHASE.PhaseContractError):
        PHASE.atomic_write_json(output_path, result)


def test_failed_exit_still_emits_invalid_machine_document():
    value = _passing_events()
    result = PHASE.finalize(value, 17, None)
    assert result['status'] == 'invalid'
    assert result['exit_status'] == 17


def test_v2_paced_followability_is_primary_and_wall_rtf_is_diagnostic():
    value = _passing_v2_events()
    for index, name in enumerate(PHASE.EVENT_NAMES[5:], 20):
        value['events_monotonic_ns'][name] = index * 1_000_000_000
    checked = PHASE.validate_evidence_v2(value, require_pass=True)
    assert checked['runtime']['phase_mode'] == 'paced_1x'
    assert checked['runtime']['online_compute_rtf'] == pytest.approx(1.7)
    assert checked['runtime']['paced_followability_passed'] is True
    assert checked['status'] == 'pass'


@pytest.mark.parametrize('field', [
    'received_messages', 'processed_messages', 'dropped_messages',
    'queue_overflow', 'eof_observed', 'backlog_at_drain',
])
def test_v2_consumer_proof_is_fail_closed(field):
    value = _passing_v2_events()
    if field == 'eof_observed':
        value['consumer'][field] = False
    elif field in ('dropped_messages', 'queue_overflow', 'backlog_at_drain'):
        value['consumer'][field] = 1
    else:
        value['consumer'][field] = 3
    checked = PHASE.validate_evidence_v2(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['consumer_passed'] is False


def test_v2_rejects_publisher_counts_as_consumer_ack():
    value = _passing_v2_events()
    value['consumer']['publisher_count_used'] = True
    checked = PHASE.validate_evidence_v2(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['consumer_reason'] == \
        'publisher_count_cannot_prove_consumer_processing'


def test_v2_consumer_evidence_is_explicitly_attached(tmp_path):
    value = _passing_v2_events()
    source = {'consumer': dict(value['consumer'])}
    source['consumer']['counter_evidence_path'] = 'producer-owned.json'
    evidence_path = tmp_path / 'consumer.json'
    evidence_path.write_text(json.dumps(source), encoding='utf-8')
    PHASE.set_consumer_evidence(
        value, source['consumer'], source_path=str(evidence_path))
    assert value['consumer']['counter_evidence_path'] == str(evidence_path)
    assert PHASE.validate_evidence_v2(value, require_pass=True)['status'] == 'pass'

    missing = _passing_v2_events()
    missing['consumer']['processed_messages'] = None
    assert PHASE.validate_evidence_v2(missing)['status'] == 'invalid'


def test_v2_consumer_counts_promote_coverage_without_publisher_counts(tmp_path):
    value = _passing_v2_events()
    value['coverage'].update({
        'mode': 'trajectory_timestamp_coverage',
        'expected_messages': None,
        'consumed_messages': None,
        'dropped_messages': None,
        'queue_overflow': None,
        'last_input_timestamp_seconds': 109.9,
        'required_end_timestamp_seconds': 110.0,
        'end_gap_seconds': 0.1,
    })
    evidence = dict(value['consumer'])
    evidence_path = tmp_path / 'producer-consumer.json'
    PHASE.set_consumer_evidence(value, evidence, source_path=str(evidence_path))
    assert value['coverage']['mode'] == 'processed_message_counts'
    assert value['coverage']['expected_messages'] == 4
    assert value['coverage']['consumed_messages'] == 4
    assert value['coverage']['dropped_messages'] == 0
    assert value['coverage']['queue_overflow'] == 0
    assert PHASE.validate_evidence_v2(value, require_pass=True)['status'] == 'pass'


def test_v2_consumer_coverage_conflict_is_fail_closed():
    value = _passing_v2_events()
    value['coverage']['dropped_messages'] = 1
    with pytest.raises(PHASE.PhaseContractError, match='conflicts'):
        PHASE.set_consumer_evidence(value, value['consumer'])


def test_v2_finalize_without_application_consumer_proof_is_invalid():
    value = _passing_v2_events()
    value['consumer'] = None
    result = PHASE.finalize(value, 0, None)
    assert result['status'] == 'invalid'
    assert result['validation']['consumer_reason'] == 'consumer_evidence_missing'


def test_v2_unpaced_requires_ack_backpressure_and_uses_throughput_gate():
    value = _passing_v2_events('unpaced_ack')
    checked = PHASE.validate_evidence_v2(value, require_pass=True)
    assert checked['runtime']['unpaced_throughput_gate_passed'] is True
    value['consumer']['single_message_buffer_verified'] = False
    assert PHASE.validate_evidence_v2(value, require_pass=True)['status'] == 'pass'
    value['consumer']['ack_backpressure_verified'] = False
    checked = PHASE.validate_evidence_v2(value)
    assert checked['status'] == 'invalid'
    assert checked['validation']['consumer_reason'] == \
        'ack_backpressure_not_verified'
