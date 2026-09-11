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

"""GT-free tests for the additive M6a10 v10 terminal-support contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'benchmark_phase_contract', ROOT / 'scripts/benchmark_phase_contract.py'
)
PHASE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(PHASE)


TOPICS = ('lidar', 'imu', 'image')


def _record(topic: str, index: int, timestamp: float) -> dict:
    return {
        'record_id': f'{topic}-{index}',
        'topic': topic,
        'timestamp_seconds': timestamp,
        'support_context_proven': True,
        'post_boundary': True,
        'sync_predicate_evaluated': True,
        'can_form_synchronization_unit': False,
        'reason_code': 'strictly_after_completed_boundary',
    }


def _buffer(records: list[dict]) -> dict:
    timestamps = [row['timestamp_seconds'] for row in records]
    return {
        'count': len(records),
        'oldest_timestamp_seconds': min(timestamps) if timestamps else None,
        'newest_timestamp_seconds': max(timestamps) if timestamps else None,
        'records': records,
    }


def _document(*, residual_imu: int = 0, residual_image: int = 0) -> dict:
    expected = {'lidar': 2, 'imu': 3, 'image': 2}
    residual = {
        'lidar': [],
        'imu': [_record('imu', 2, 10.1 + i * 0.01) for i in range(residual_imu)],
        'image': [_record('image', 1, 10.2 + i * 0.01) for i in range(residual_image)],
    }
    completed = {topic: expected[topic] - len(residual[topic]) for topic in TOPICS}
    buffers = {topic: _buffer(residual[topic]) for topic in TOPICS}
    support_records = [record for topic in TOPICS for record in residual[topic]]
    support_counts = {topic: len(residual[topic]) for topic in TOPICS}
    zeros = {topic: 0 for topic in TOPICS}
    return {
        'schema_version': PHASE.SCHEMA_VERSION_V3,
        'contract_version': PHASE.CONTRACT_VERSION_V3,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'required_evaluation_end_timestamp_seconds': 10.0,
        'maximum_end_gap_seconds': 0.25,
        'counts': {
            'expected': expected,
            'published': dict(expected),
            'received': dict(expected),
            'acknowledged': dict(expected),
        },
        'buffers': buffers,
        'backend': {
            'quiescent': True,
            'quiescence_observed': True,
            'completed_boundary': {
                'observed': True,
                'timestamp_seconds': 10.0,
                'sequence': 42,
                'source': 'fixture.backend_completion_boundary',
            },
            'completed_counts': completed,
            'completed_synchronization_units': 2,
            'dropped_counts': zeros,
            'overflow_counts': zeros,
            'processing_failures': 0,
            'in_flight': {
                'active': False,
                'topic': None,
                'unit_id': None,
            },
        },
        'trajectory': {
            'coverage_verified': True,
            'last_timestamp_seconds': 10.0,
            'end_gap_seconds': 0.0,
        },
        'terminal_support_context': {
            'classification': 'post_boundary_support_context_only',
            'total_count': len(support_records),
            'by_topic': support_counts,
            'records': support_records,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _invalid(document: dict) -> dict:
    result = PHASE.validate_terminal_support_context_v3(document)
    assert result['status'] == 'invalid'
    return result


def test_exact_valid_terminal_support_context_passes():
    result = PHASE.validate_terminal_support_context_v3(
        _document(residual_imu=1, residual_image=1), require_pass=True
    )
    assert result['status'] == 'pass'
    assert result['validation']['support_context_counts'] == {'lidar': 0, 'imu': 1, 'image': 1}
    assert result['validation']['count_conservation_passed'] is True


def test_classic_zero_backlog_passes():
    result = PHASE.validate_evidence(_document(), require_pass=True)
    assert result['status'] == 'pass'
    assert result['validation']['support_context_counts'] == {'lidar': 0, 'imu': 0, 'image': 0}


def test_missing_per_topic_backlog_fails_closed():
    value = _document(residual_imu=1)
    del value['buffers']['imu']['count']
    result = _invalid(value)
    assert result['validation']['reason'] == 'buffer_imu_count_missing_or_invalid'


def test_missing_buffer_timestamp_fails_closed():
    value = _document(residual_image=1)
    value['buffers']['image']['records'][0].pop('timestamp_seconds')
    result = _invalid(value)
    assert result['validation']['reason'] == 'buffer_image_record_timestamp_missing_or_invalid'


def test_residual_lidar_fails_even_if_post_boundary():
    value = _document()
    value['buffers']['lidar'] = _buffer([_record('lidar', 2, 10.1)])
    value['terminal_support_context']['by_topic']['lidar'] = 1
    value['terminal_support_context']['total_count'] = 1
    value['terminal_support_context']['records'] = value['buffers']['lidar']['records']
    value['backend']['completed_counts']['lidar'] = 1
    result = _invalid(value)
    assert result['validation']['reason'] == 'residual_lidar_not_support_context'


def test_pre_boundary_residual_fails_closed():
    value = _document(residual_imu=1)
    value['buffers']['imu']['records'][0]['timestamp_seconds'] = 9.9
    value['buffers']['imu']['oldest_timestamp_seconds'] = 9.9
    value['buffers']['imu']['newest_timestamp_seconds'] = 9.9
    value['terminal_support_context']['records'][0]['timestamp_seconds'] = 9.9
    result = _invalid(value)
    assert result['validation']['reason'] == 'support_context_record_not_strictly_post_boundary'


def test_unstable_or_in_flight_backend_fails_closed():
    value = _document(residual_imu=1)
    value['backend']['quiescent'] = False
    result = _invalid(value)
    assert result['validation']['reason'] == 'backend_not_quiescent'

    value = _document(residual_imu=1)
    value['backend']['in_flight']['active'] = True
    result = _invalid(value)
    assert result['validation']['reason'] == 'backend_in_flight'


def test_count_conservation_fails_closed():
    value = _document(residual_imu=1)
    value['backend']['completed_counts']['imu'] = 1
    result = _invalid(value)
    assert result['validation']['reason'] == 'count_conservation_failed_imu'


def test_support_context_cannot_be_labeled_processed_or_dropped():
    value = _document(residual_imu=1)
    value['terminal_support_context']['records'][0]['processed'] = False
    result = _invalid(value)
    assert (
        result['validation']['reason'] == 'support_context_record_has_forbidden_processing_label'
    )


def test_support_context_requires_source_sync_predicate_proof():
    value = _document(residual_imu=1)
    value['terminal_support_context']['records'][0].pop('sync_predicate_evaluated')
    result = _invalid(value)
    assert result['validation']['reason'] == 'support_context_sync_predicate_unproven'


def test_missing_required_end_timestamp_fails_closed():
    value = _document(residual_imu=1)
    value['required_evaluation_end_timestamp_seconds'] = None
    result = _invalid(value)
    assert result['validation']['reason'] == 'required_evaluation_end_timestamp_invalid'


def test_v2_contract_still_uses_original_validator():
    value = {
        'schema_version': PHASE.SCHEMA_VERSION_V2,
        'contract_version': PHASE.CONTRACT_VERSION_V2,
    }
    with pytest.raises(PHASE.PhaseContractError, match='schema/version'):
        PHASE.validate_evidence(value)


def test_mixed_v3_schema_and_contract_metadata_fails_closed():
    value = _document()
    value['schema_version'] = PHASE.SCHEMA_VERSION_V2
    with pytest.raises(PHASE.PhaseContractError, match='schema/version'):
        PHASE.validate_evidence(value)

    value = _document()
    value['contract_version'] = PHASE.CONTRACT_VERSION_V2
    with pytest.raises(PHASE.PhaseContractError, match='schema/version'):
        PHASE.validate_evidence(value)


def test_v10_patch_and_profile_are_additive_source_level_gate():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    profile = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'
    compositor = ROOT / 'scripts/compose_fast_livo2_terminal_evidence.py'
    dockerfile = ROOT / 'docker/fast_livo2_m6a10_v10.Dockerfile'
    build_script = ROOT / 'scripts/build_fast_livo2_m6a10_v10_image.sh'
    text = patch.read_text(encoding='utf-8')
    profile_text = profile.read_text(encoding='utf-8')
    compositor_text = compositor.read_text(encoding='utf-8')
    dockerfile_text = dockerfile.read_text(encoding='utf-8')
    build_text = build_script.read_text(encoding='utf-8')
    assert 'm6a10-online-compute-v3-terminal-support-context' in text
    assert 'terminal_support_context' in text
    assert 'completed_boundary' in text
    assert 'in_flight' in text
    assert 'per-topic' in text
    assert 'm6a10_fast_livo2_v2c_v10' in profile_text
    assert 'm6a10-v2c-fast-livo2-terminal-support-context-v10' in profile_text
    assert 'formal_replay_forbidden: true' in profile_text
    assert 'ground_truth_mount_exposed: false' in profile_text
    assert 'm6a10-fast-livo2-consumer-terminal-v1' in text
    assert 'bind_authenticated_feeder_counts' not in text
    assert 'validate_terminal_support_context_v3' in compositor_text
    assert 'atomic_write_json' in compositor_text
    assert 'required_evaluation_end_source' in profile_text
    assert 'runtime_ready: true' in profile_text
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount' in dockerfile_text
    assert 'catkin_make -DCMAKE_BUILD_TYPE=Release -j2' in dockerfile_text
    assert 'docker build --network none --pull=false' in build_text
    assert 'FAST_LIVO2_M6A10_V10_PATCH_SHA256=${PATCH_SHA256}' in build_text


def _source_patch(text: str) -> str:
    start = text.index('diff --git a/src/LIVMapper.cpp')
    return text[start:]


def _source_patch_plain(text: str) -> str:
    """Render the added/context source lines of the unified diff."""
    lines = []
    for line in _source_patch(text).splitlines():
        if line.startswith(('+++', '---')):
            continue
        if line.startswith('-'):
            continue
        if line.startswith(('+', ' ')):
            lines.append(line[1:])
        else:
            lines.append(line)
    return '\n'.join(lines)


def _all_input_mutations_are_hooked(source: str) -> bool:
    lines = source.splitlines()
    topics = {
        'lid_raw_data_buffer.pop_front();': 'Lidar',
        'imu_buffer.pop_front();': 'Imu',
        'img_buffer.pop_front();': 'Image',
    }
    for index, line in enumerate(lines):
        for mutation, topic in topics.items():
            if mutation in line:
                prior = '\n'.join(lines[max(0, index - 10): index])
                if not re.search(rf'observe_queue_pop\([\s\S]{{0,260}}Topic::{topic}', prior):
                    return False
        if 'lid_raw_data_buffer.clear();' in line:
            prior = '\n'.join(lines[max(0, index - 10): index])
            if not re.search(r'observe_queue_clear\([\s\S]{0,180}Topic::Lidar', prior):
                return False
    return True


def test_v10_source_gate_has_real_mapper_call_sites_for_every_helper():
    """The additive adapter is only credible when its APIs are source-wired."""
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    source = _source_patch_plain(patch.read_text(encoding='utf-8'))
    required_calls = (
        'm6a10_terminal_support_context.begin(',
        'm6a10_terminal_support_context.begin_synchronization_unit(',
        'm6a10_terminal_support_context.abort_synchronization_unit(',
        'm6a10_terminal_support_context.begin_estimator(',
        'm6a10_terminal_support_context.observe_queue_pop(',
        'm6a10_terminal_support_context.observe_queue_clear(',
        'm6a10_terminal_support_context.observe_completed_estimator(',
        'm6a10_terminal_support_context.observe_terminal_eof(',
        'm6a10_terminal_support_context.observe_terminal_sync_predicate(',
        'm6a10_terminal_support_context.pending_record_ids(',
        'm6a10_terminal_support_context.pending_record_topic(',
        'm6a10_terminal_support_context.prove_terminal_record(',
        'm6a10_terminal_support_context.snapshot_pending_buffers(',
        'm6a10_terminal_support_context.finalize_service(',
        'm6a10_terminal_support_context.status_service(',
        'm6a10_terminal_support_context.benchmark_sensor_stamp(',
    )
    for call in required_calls:
        assert source.count(call) >= 1, call
    assert 'bind_authenticated_feeder_counts' not in source


def test_applied_patch_stack_keeps_v2_and_adds_only_v3_callback_authority():
    """The v10 delta must extend the immutable v2 helper without widening it."""
    base = (ROOT / 'docker/patches/fast_livo2.m6a10-v2c.patch').read_text(encoding='utf-8')
    v8 = (ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch').read_text(
        encoding='utf-8'
    )
    v9 = (ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v9-wallrate.patch').read_text(
        encoding='utf-8'
    )
    v10 = (
        ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    ).read_text(encoding='utf-8')

    original_gate = (
        'enabled_ = contract != nullptr &&\n'
        '               std::string(contract) == "m6a10-online-compute-v2";'
    )
    base_added = '\n'.join(line[1:] for line in base.splitlines() if line.startswith('+'))
    assert original_gate in base_added
    assert 'm6a10-online-compute-v3-terminal-support-context' not in base
    assert 'm6a10-online-compute-v3-terminal-support-context' not in v8
    assert 'm6a10-online-compute-v3-terminal-support-context' not in v9
    assert '-    ' + original_gate.splitlines()[0] in v10
    assert 'const std::string contract_value = contract == nullptr ? std::string() :' in v10
    assert 'contract_value == "m6a10-online-compute-v2" ||' in v10
    assert 'contract_value == "m6a10-online-compute-v3-terminal-support-context";' in v10

    def enabled(contract: str) -> bool:
        return contract in {
            'm6a10-online-compute-v2',
            'm6a10-online-compute-v3-terminal-support-context',
        }

    assert enabled('m6a10-online-compute-v2')
    assert enabled('m6a10-online-compute-v3-terminal-support-context')
    for contract in ('', 'm6a10-online-compute-v1', 'other-contract'):
        assert not enabled(contract)


def test_v3_applied_source_advertises_callback_ack_and_terminal_services():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    source = _source_patch_plain(patch.read_text(encoding='utf-8'))
    service_start = source.index('void LIVMapper::initializeSubscribersAndPublishers')
    service_end = source.index('void LIVMapper::', service_start + 20)
    services = source[service_start:service_end]
    assert 'm6a10_consumer_evidence.enabled() ||' in services
    assert 'if (m6a10_consumer_evidence.enabled())' in services
    assert 'if (m6a10_terminal_support_context.enabled())' in services
    for name in (
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/terminal_status',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ):
        assert name in services


def test_v10_source_gate_instruments_queue_mutations_and_rejects_discards():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    text = patch.read_text(encoding='utf-8')
    source = _source_patch_plain(text)
    # Each input FIFO mutation is paired with a ledger hook.  The source
    # context line is intentionally retained in the patch, so these checks
    # catch a future pop/clear added without the corresponding observation.
    for topic, mutation in (
        ('Lidar', 'lid_raw_data_buffer.pop_front();'),
        ('Imu', 'imu_buffer.pop_front();'),
        ('Image', 'img_buffer.pop_front();'),
    ):
        assert mutation in source
        assert re.search(
            rf'observe_queue_pop\([\s\S]{{0,260}}Topic::{topic}[\s\S]{{0,120}}'
            rf'\n[-+ ]*\s*{re.escape(mutation)}',
            source,
        ), (topic, mutation)
    assert re.search(
        r'observe_queue_clear\([\s\S]{0,180}Topic::Lidar[\s\S]{0,100}'
        r'\n\s*lid_raw_data_buffer\.clear\(\)',
        source,
    )
    assert 'increment(dropped_, topic)' in text
    assert 'synchronization_unit_aborted' in text
    assert _all_input_mutations_are_hooked(source)


def test_v10_source_gate_rejects_a_missing_pop_hook():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    source = _source_patch_plain(patch.read_text(encoding='utf-8'))
    broken = source.replace(
        '    m6a10_terminal_support_context.observe_queue_pop(\n'
        '      M6A10TerminalSupportContext::Topic::Lidar, true);\n'
        '    lid_raw_data_buffer.pop_front();',
        '    lid_raw_data_buffer.pop_front();',
        1,
    )
    assert broken != source
    assert not _all_input_mutations_are_hooked(broken)


def test_v10_terminal_service_requires_distinct_stable_polls_and_eof_predicate():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    text = patch.read_text(encoding='utf-8')
    source = _source_patch_plain(text)
    status_start = source.index('bool LIVMapper::m6a10_terminal_status_service(')
    status_end = source.index('bool LIVMapper::m6a10_terminal_finalize_service(', status_start)
    status = source[status_start:status_end]
    assert status.count('snapshot_pending_buffers(') == 1
    assert 'std::lock_guard<std::mutex> buffer_guard(mtx_buffer)' in status
    assert 'observe_terminal_sync_predicate(' in status
    assert 'pending_record_ids(' in status
    assert 'prove_terminal_record(' in status
    assert 'minimum_poll_wall_seconds_' in text
    assert 'identical && now - pending_snapshot_wall_seconds_' in text
    assert 'observe_terminal_eof(' in source


def test_v10_estimator_boundary_and_benchmark_stamp_are_not_received_time():
    patch = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
    source = _source_patch_plain(patch.read_text(encoding='utf-8'))
    estimator_start = source.index('m6a10_terminal_support_context.begin_estimator()')
    estimator_end = (
        source.index(
            'm6a10_terminal_support_context.observe_completed_estimator(', estimator_start
        )
        + 220
    )
    run = source[estimator_start:estimator_end]
    assert run.index('begin_estimator(') < run.index('stateEstimationAndMapping();')
    assert run.index('stateEstimationAndMapping();') < run.index('observe_completed_estimator(')
    assert 'LidarMeasures.measures.back().vio_time' in run
    assert 'LidarMeasures.measures.back().lio_time' in run
    assert 'last_timestamp_lidar' not in run[run.index('completed_sensor_timestamp'):]
    assert source.count('benchmark_sensor_stamp()') >= 3


def test_v10_profile_is_runtime_ready_but_formal_replay_remains_fail_closed():
    """Compile/no-input readiness does not authorize a formal replay claim."""
    profile = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'
    text = profile.read_text(encoding='utf-8')
    assert 'runtime_ready: true' in text
    assert 'COMPILE_SYNTHETIC_DUAL_SERVICE_VALIDATED_FAIL_CLOSED' in text
    assert 'formal_replay_forbidden: true' in text
