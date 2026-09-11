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


"""
Host v11 binder/compositor gate tests using the immutable ready profile.

The fixtures are synthetic mapper/feeder JSON only.  They exercise the existing
host binder and compositor APIs with a v11 profile adapter so the bounded
boundary-before-required-end rule is tested without changing production files,
opening the bag, starting Docker/ROS, or invoking GT/scorer/map code.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Callable

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
READY_PROFILE = ROOT / ('configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal_ready.yaml')
READY_PROFILE_SHA256 = '6f56068a8a283851aa3a3723d2c9b526235bc97ba507a7dee4ea72bcd461bd2d'
READY_PROFILE_KEY = 'm6a10_fast_livo2_v2c_v11_formal_ready'
CONSUMER = ROOT / 'scripts/bind_fast_livo2_v10_consumer_evidence.py'
COMPOSITOR = ROOT / 'scripts/compose_fast_livo2_terminal_evidence.py'
TOPICS = ('lidar', 'imu', 'image')
CONTRACT = 'm6a10-online-compute-v4-terminal-bounded-end-gap'
RAW_CONTRACT = 'm6a10-fast-livo2-consumer-terminal-v1'
REQUIRED_END = 1623491515.148352
BOUNDARY = 1623491515.03615
END_GAP = 0.112201929
MAX_END_GAP = 0.25
RECEIVED = {'lidar': 5793, 'imu': 225102, 'image': 5792}
COMPLETED = {'lidar': 5793, 'imu': 225024, 'image': 5789}
SUPPORT = {'lidar': 0, 'imu': 78, 'image': 3}


def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


BINDER = _load_module('v11_host_gate_binder', CONSUMER)
COMPOSE = _load_module('v11_host_gate_compositor', COMPOSITOR)
ORIGINAL_COMPOSITOR_VALIDATOR = COMPOSE.validate_terminal_support_context_v3


def _ready_document(path: Path) -> tuple[dict[str, Any], dict[str, Any], str]:
    observed = hashlib.sha256(path.read_bytes()).hexdigest()
    if observed != READY_PROFILE_SHA256:
        raise BINDER.BinderError('PROFILE_DRIFT', 'v11 ready profile SHA-256 differs from pin')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    profile = document['competitive_slam_profile'][READY_PROFILE_KEY]
    assert profile['system'] == 'fast_livo2'
    assert profile['schema_version'] == 3
    assert profile['phase']['contract_version'] == CONTRACT
    profile['input']
    return document, profile, observed


def _binder_profile(path: Path) -> tuple[dict[str, Any], str, dict[str, Any]]:
    document, profile, observed = _ready_document(path)
    input_identity = profile['input']
    phase = profile['phase']
    return (
        document,
        observed,
        {
            'bag_path': input_identity['path'],
            'bag_bytes': input_identity['bytes'],
            'bag_sha256': input_identity['sha256'],
            'expected_messages': input_identity['expected_messages'],
            'expected_topic_counts': dict(input_identity['expected_topic_counts']),
            'required_end_timestamp_seconds': phase['required_evaluation_end_timestamp_seconds'],
        },
    )


def _compositor_profile(
    path: Path,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    document, profile, observed = _ready_document(path)
    input_identity = profile['input']
    phase = profile['phase']
    source = phase['required_evaluation_end_source']
    return (
        document,
        {
            'path': input_identity['path'],
            'bytes': input_identity['bytes'],
            'sha256': input_identity['sha256'],
            'expected_topic_counts': dict(input_identity['expected_topic_counts']),
            'required_end': float(phase['required_evaluation_end_timestamp_seconds']),
            'maximum_end_gap': float(phase['maximum_end_gap_seconds']),
            'required_end_source': {
                'path': source['path'],
                'sha256': source['sha256'],
                'field': source['field'],
            },
        },
        observed,
    )


def _record(topic: str, index: int, timestamp: float) -> dict[str, Any]:
    return {
        'record_id': f'{topic}-support-{index}',
        'topic': topic,
        'timestamp_seconds': timestamp,
        'support_context_proven': True,
        'post_boundary': True,
        'sync_predicate_evaluated': True,
        'can_form_synchronization_unit': False,
        'reason_code': 'strictly_after_completed_boundary',
    }


def _buffer(records: list[dict[str, Any]]) -> dict[str, Any]:
    stamps = [row['timestamp_seconds'] for row in records]
    return {
        'count': len(records),
        'oldest_timestamp_seconds': min(stamps) if stamps else None,
        'newest_timestamp_seconds': max(stamps) if stamps else None,
        'records': records,
    }


def _records() -> dict[str, list[dict[str, Any]]]:
    return {
        'lidar': [],
        'imu': [
            _record('imu', index, REQUIRED_END + 0.001 + index * 1e-6)
            for index in range(SUPPORT['imu'])
        ],
        'image': [
            _record('image', index, REQUIRED_END + 0.002 + index * 1e-6)
            for index in range(SUPPORT['image'])
        ],
    }


def _fixture_values() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    _, expected, profile_sha = _compositor_profile(READY_PROFILE)
    residual = _records()
    support_records = [row for topic in TOPICS for row in residual[topic]]
    consumer = {
        'schema_version': 1,
        'contract_id': RAW_CONTRACT,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'received_topic_counts': dict(RECEIVED),
        'backend': {
            'quiescent': True,
            'quiescence_observed': True,
            'completed_boundary': {
                'observed': True,
                'timestamp_seconds': BOUNDARY,
                'sequence': 42,
                'source': 'v11.synthetic.completed_estimator_boundary',
            },
            'completed_counts': dict(COMPLETED),
            'completed_synchronization_units': 236687,
            'dropped_counts': {topic: 0 for topic in TOPICS},
            'overflow_counts': {topic: 0 for topic in TOPICS},
            'processing_failures': 0,
            'in_flight': {'active': False, 'topic': None, 'unit_id': None},
        },
        'buffers': {topic: _buffer(residual[topic]) for topic in TOPICS},
        'terminal_support_context': {
            'classification': 'post_boundary_support_context_only',
            'total_count': sum(SUPPORT.values()),
            'by_topic': dict(SUPPORT),
            'records': support_records,
        },
        'trajectory': {
            'coverage_verified': True,
            'last_timestamp_seconds': BOUNDARY,
            'end_gap_seconds': END_GAP,
        },
        'terminal_observation': {
            'eof_observed': True,
            'stable': True,
            'identical_snapshot': True,
            'sync_predicate_evaluated': True,
            'poll_count': 2,
            'stable_poll_count': 2,
            'first_poll_wall_seconds': 10.0,
            'stable_poll_wall_seconds': 10.1,
            'minimum_poll_wall_seconds': 0.05,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'feeder_sequence': 17,
        'feeder_nonce': 'v11-host-gate-fixture',
    }
    feeder = {
        'schema_version': 1,
        'contract_id': COMPOSE.FEEDER_CONTRACT,
        'status': 'pass',
        'bag_path': expected['path'],
        'bag_bytes': expected['bytes'],
        'bag_sha256': expected['sha256'],
        'expected_topic_counts': dict(RECEIVED),
        'published_topic_counts': dict(RECEIVED),
        'acked_topic_counts': dict(RECEIVED),
        'published_messages': sum(RECEIVED.values()),
        'single_inflight': True,
        'publisher_queue_size': 1,
        'ack_source_kind': 'v11.synthetic.consumer_status',
        'ack_backpressure_verified': True,
        'sequence': 17,
        'nonce': 'v11-host-gate-fixture',
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    return (
        consumer,
        feeder,
        {
            'expected': expected,
            'profile_sha256': profile_sha,
        },
    )


def _write_fixture(
    tmp_path: Path,
) -> tuple[Path, Path, dict[str, Any], dict[str, Any], dict[str, Any]]:
    consumer, feeder, metadata = _fixture_values()
    raw_path = tmp_path / 'raw_mapper.json'
    feeder_path = tmp_path / 'feeder.json'
    raw_path.write_text(json.dumps(consumer, indent=2), encoding='utf-8')
    feeder_path.write_text(json.dumps(feeder, indent=2), encoding='utf-8')
    return raw_path, feeder_path, consumer, feeder, metadata


def _v11_validator(
    value: dict[str, Any],
    *,
    maximum_end_gap_seconds: float,
    require_pass: bool = False,
) -> dict[str, Any]:
    """Reuse the generic compositor validator with v11's one boundary change."""
    if value.get('contract_version') != COMPOSE.CONTRACT_VERSION_V3:
        raise COMPOSE.PhaseContractError('terminal schema contract mismatch')
    trajectory = value.get('trajectory')
    backend = value.get('backend')
    if not isinstance(trajectory, dict) or not isinstance(backend, dict):
        raise COMPOSE.PhaseContractError('v11 terminal fields missing')
    required = float(value['required_evaluation_end_timestamp_seconds'])
    last = float(trajectory['last_timestamp_seconds'])
    gap = float(trajectory['end_gap_seconds'])
    boundary = float(backend['completed_boundary']['timestamp_seconds'])
    if not all(math.isfinite(x) for x in (required, last, gap, boundary)):
        raise COMPOSE.PhaseContractError('v11 terminal timestamp nonfinite')
    if gap < 0.0:
        raise COMPOSE.PhaseContractError('trajectory_end_gap_negative')
    if gap > maximum_end_gap_seconds:
        raise COMPOSE.PhaseContractError('trajectory_end_gap_exceeds_limit')
    if not math.isclose(gap, required - last, rel_tol=0.0, abs_tol=1e-9):
        raise COMPOSE.PhaseContractError('trajectory_end_gap_mismatch')
    if boundary < required:
        adjusted = copy.deepcopy(value)
        adjusted['backend']['completed_boundary']['timestamp_seconds'] = required
        result = ORIGINAL_COMPOSITOR_VALIDATOR(
            adjusted,
            maximum_end_gap_seconds=maximum_end_gap_seconds,
            require_pass=require_pass,
        )
        result['validation']['backend_boundary_timestamp_seconds'] = boundary
        return result
    return ORIGINAL_COMPOSITOR_VALIDATOR(
        value,
        maximum_end_gap_seconds=maximum_end_gap_seconds,
        require_pass=require_pass,
    )


def _patch_v11_adapters(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(BINDER, '_profile', _binder_profile)
    monkeypatch.setattr(COMPOSE, '_profile', _compositor_profile)
    monkeypatch.setattr(COMPOSE, 'validate_terminal_support_context_v3', _v11_validator)


def _bind_and_compose(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    _patch_v11_adapters(monkeypatch)
    raw_path, feeder_path, consumer, feeder, metadata = _write_fixture(tmp_path)
    bound_path = tmp_path / 'host_bound_consumer.json'
    receipt = BINDER.bind_consumer_evidence(raw_path, bound_path, READY_PROFILE)
    output_path = tmp_path / 'composed_terminal.json'
    result = COMPOSE.compose(
        profile_path=READY_PROFILE,
        consumer_path=bound_path,
        feeder_path=feeder_path,
        output_path=output_path,
    )
    return (
        result,
        receipt,
        {
            'raw': consumer,
            'feeder': feeder,
            'metadata': metadata,
            'bound': json.loads(bound_path.read_text(encoding='utf-8')),
            'composed': json.loads(output_path.read_text(encoding='utf-8')),
        },
    )


def test_ready_profile_sha_and_exact_positive_gate(tmp_path, monkeypatch):
    assert hashlib.sha256(READY_PROFILE.read_bytes()).hexdigest() == (READY_PROFILE_SHA256)
    result, receipt, values = _bind_and_compose(tmp_path, monkeypatch)
    assert result['status'] == 'pass'
    assert result['validation']['count_conservation_passed'] is True
    assert result['validation']['trajectory_end_gap_seconds'] == pytest.approx(END_GAP, abs=1e-9)
    assert result['validation']['backend_boundary_timestamp_seconds'] == BOUNDARY
    assert values['metadata']['profile_sha256'] == READY_PROFILE_SHA256
    assert values['bound']['input'] == {
        'bag_path': values['metadata']['expected']['path'],
        'bag_bytes': values['metadata']['expected']['bytes'],
        'bag_sha256': values['metadata']['expected']['sha256'],
    }
    assert values['bound']['profile_sha256'] == READY_PROFILE_SHA256
    assert values['bound']['received_topic_counts'] == RECEIVED
    assert values['composed']['counts']['received'] == RECEIVED
    assert values['composed']['backend']['completed_counts'] == COMPLETED
    assert values['composed']['terminal_support_context']['by_topic'] == SUPPORT
    assert values['composed']['ground_truth_content_opened'] is False
    assert values['composed']['scorer_invoked'] is False


@pytest.mark.parametrize(
    'field',
    ['input', 'profile_sha256', 'expected_topic_counts', 'published_topic_counts', 'ACK'],
)
def test_authority_contamination_is_rejected(
    tmp_path,
    monkeypatch,
    field: str,
):
    _patch_v11_adapters(monkeypatch)
    raw_path, _, consumer, _, _ = _write_fixture(tmp_path)
    consumer[field] = {} if field != 'ACK' else {'count': 1}
    raw_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'bound.json', READY_PROFILE)
    assert error.value.kind == 'RAW_AUTHORITY_FIELDS'


@pytest.mark.parametrize(
    ('mutation', 'reason'),
    [
        (
            lambda value: value['trajectory'].update(
                last_timestamp_seconds=REQUIRED_END - 0.30,
                end_gap_seconds=0.30,
            ),
            'trajectory_end_gap_exceeds_limit',
        ),
        (
            lambda value: value['backend']['completed_boundary'].update(
                timestamp_seconds=REQUIRED_END + 0.10
            )
            or value['trajectory'].update(
                last_timestamp_seconds=REQUIRED_END + 0.10,
                end_gap_seconds=-0.10,
            ),
            'trajectory_end_gap_negative',
        ),
    ],
)
def test_gap_bounds_are_fail_closed(
    tmp_path,
    monkeypatch,
    mutation: Callable[[dict[str, Any]], Any],
    reason: str,
):
    _patch_v11_adapters(monkeypatch)
    raw_path, feeder_path, consumer, _, _ = _write_fixture(tmp_path)
    mutation(consumer)
    raw_path.write_text(json.dumps(consumer), encoding='utf-8')
    bound_path = tmp_path / 'bound.json'
    BINDER.bind_consumer_evidence(raw_path, bound_path, READY_PROFILE)
    with pytest.raises(COMPOSE.PhaseContractError, match=reason):
        COMPOSE.compose(
            profile_path=READY_PROFILE,
            consumer_path=bound_path,
            feeder_path=feeder_path,
        )


def test_residual_lidar_is_rejected(tmp_path, monkeypatch):
    _patch_v11_adapters(monkeypatch)
    raw_path, feeder_path, consumer, _, _ = _write_fixture(tmp_path)
    record = _record('lidar', 0, REQUIRED_END + 0.01)
    consumer['buffers']['lidar'] = _buffer([record])
    consumer['terminal_support_context']['by_topic']['lidar'] = 1
    consumer['terminal_support_context']['total_count'] += 1
    consumer['terminal_support_context']['records'].append(record)
    raw_path.write_text(json.dumps(consumer), encoding='utf-8')
    bound_path = tmp_path / 'bound.json'
    BINDER.bind_consumer_evidence(raw_path, bound_path, READY_PROFILE)
    with pytest.raises(COMPOSE.PhaseContractError, match='residual_lidar_not_support_context'):
        COMPOSE.compose(
            profile_path=READY_PROFILE,
            consumer_path=bound_path,
            feeder_path=feeder_path,
        )


def test_count_mismatch_is_rejected_without_copying_expected(
    tmp_path,
    monkeypatch,
):
    _patch_v11_adapters(monkeypatch)
    raw_path, feeder_path, consumer, _, _ = _write_fixture(tmp_path)
    consumer['received_topic_counts']['imu'] -= 1
    raw_path.write_text(json.dumps(consumer), encoding='utf-8')
    bound_path = tmp_path / 'bound.json'
    BINDER.bind_consumer_evidence(raw_path, bound_path, READY_PROFILE)
    with pytest.raises(COMPOSE.PhaseContractError, match='consumer_received_count_mismatch'):
        COMPOSE.compose(
            profile_path=READY_PROFILE,
            consumer_path=bound_path,
            feeder_path=feeder_path,
        )


def test_ready_profile_drift_is_rejected(tmp_path, monkeypatch):
    _patch_v11_adapters(monkeypatch)
    raw_path, _, _, _, _ = _write_fixture(tmp_path)
    drifted = tmp_path / 'ready-profile-drift.yaml'
    drifted.write_bytes(
        READY_PROFILE.read_bytes().replace(
            b'maximum_end_gap_seconds: 0.25',
            b'maximum_end_gap_seconds: 0.24',
            1,
        )
    )
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'bound.json', drifted)
    assert error.value.kind == 'PROFILE_DRIFT'
