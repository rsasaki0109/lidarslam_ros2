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
Host-only v12 binder tests.

Fixtures are synthetic JSON.  They never open the NTU bag, start Docker/ROS,
read GT, or invoke a scorer.  The terminal positive uses the v5 exact-boundary
non-LiDAR policy; an old v11 classification remains invalid.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/bind_fast_livo2_v12_consumer_evidence.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml'
PROFILE_SHA = '675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d'
COUNTS = {'lidar': 5793, 'imu': 225102, 'image': 5792}
BOUNDARY = 1623491515.03615
REQUIRED_END = 1623491515.148352
GAP = 0.112201929


def _load_module():
    spec = importlib.util.spec_from_file_location('m6a10_v12_binder', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


BINDER = _load_module()


def _write(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _record(
    topic: str = 'imu', *, timestamp: float = BOUNDARY, equal: bool = True
) -> dict[str, Any]:
    return {
        'record_id': 'imu-equal-0',
        'topic': topic,
        'timestamp_seconds': timestamp,
        'support_context_proven': True,
        'post_boundary': not equal,
        'equal_to_completed_boundary': equal,
        'sync_predicate_evaluated': True,
        'can_form_synchronization_unit': False,
        'reason_code': (
            'equal_to_completed_boundary_nonlidar_support_context'
            if equal
            else 'strictly_after_completed_boundary'
        ),
    }


def _buffer(topic: str, records: list[dict[str, Any]]) -> dict[str, Any]:
    stamps = [row['timestamp_seconds'] for row in records]
    return {
        'count': len(records),
        'oldest_timestamp_seconds': min(stamps) if stamps else None,
        'newest_timestamp_seconds': max(stamps) if stamps else None,
        'records': records,
    }


def _terminal(
    *,
    classification: str = 'nonlidar_at_or_after_boundary_support_context',
    record: dict[str, Any] | None = None,
) -> dict[str, Any]:
    support = [record or _record()]
    completed = {'lidar': 5793, 'imu': 225101, 'image': 5792}
    return {
        'schema_version': 1,
        'contract_id': BINDER.TERMINAL_RAW_CONTRACT,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'received_topic_counts': dict(COUNTS),
        'completed_counts': dict(completed),
        'dropped_counts': {topic: 0 for topic in COUNTS},
        'overflow_counts': {topic: 0 for topic in COUNTS},
        'processing_failures': 0,
        'backend': {
            'quiescent': True,
            'quiescence_observed': True,
            'completed_boundary': {
                'observed': True,
                'timestamp_seconds': BOUNDARY,
                'sequence': 42,
                'source': 'synthetic.completed_boundary',
            },
            'completed_counts': dict(completed),
            'completed_synchronization_units': 236687,
            'dropped_counts': {topic: 0 for topic in COUNTS},
            'overflow_counts': {topic: 0 for topic in COUNTS},
            'processing_failures': 0,
            'in_flight': {'active': False},
        },
        'buffers': {
            'lidar': _buffer('lidar', []),
            'imu': _buffer('imu', support),
            'image': _buffer('image', []),
        },
        'terminal_support_context': {
            'classification': classification,
            'total_count': 1,
            'by_topic': {'lidar': 0, 'imu': 1, 'image': 0},
            'records': support,
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
        'trajectory': {
            'coverage_verified': True,
            'last_timestamp_seconds': BOUNDARY,
            'end_gap_seconds': GAP,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _feeder() -> dict[str, Any]:
    bag_path = (
        '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
        'tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag')
    return {
        'schema_version': 1,
        'contract_id': BINDER.FEEDER_CONTRACT,
        'status': 'pass',
        'bag_path': bag_path,
        'bag_bytes': 11290464091,
        'bag_sha256': '5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310',
        'expected_topic_counts': dict(COUNTS),
        'published_topic_counts': dict(COUNTS),
        'acked_topic_counts': dict(COUNTS),
        'published_messages': 236687,
        'single_inflight': True,
        'publisher_queue_size': 1,
        'ack_backpressure_verified': True,
        'ack_source_kind': 'synthetic.consumer_status',
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _callback() -> dict[str, Any]:
    return {
        'schema_version': 3,
        'contract_version': BINDER.CONSUMER_CONTRACT,
        'transport_contract_version': BINDER.TRANSPORT_CONTRACT,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'consumer': {
            'expected_topic_counts': dict(COUNTS),
            'received_topic_counts': dict(COUNTS),
            'received_messages': 236687,
            'acked_messages': 236687,
            'transport_outstanding_at_drain': 0,
            'maximum_transport_outstanding_messages': 1,
            'maximum_allowed_transport_outstanding_messages': 1,
            'mapper_internal_deque_current_messages': 81,
            'mapper_internal_deque_peak_messages': 81,
            'ack_exact': True,
            'eof_observed': True,
            'drain_complete': True,
            'dropped_messages': 0,
            'queue_overflow': 0,
            'processing_failures': 0,
            'ack_backpressure_verified': True,
        },
        'benchmark_only': True,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _timing() -> dict[str, Any]:
    start = 1000000000
    end = 2000000000
    duration = 1.0
    return {
        'schema_version': 1,
        'contract_version': BINDER.TIMING_CONTRACT,
        'status': 'PASS',
        'boundary': 'input_start_to_drain_end',
        'input_start_monotonic_ns': start,
        'drain_end_monotonic_ns': end,
        'duration_seconds': duration,
        'sensor_duration_seconds': 579.278127298,
        'online_compute_rtf': duration / 579.278127298,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _fixture(tmp_path: Path) -> tuple[dict[str, Path], dict[str, Any]]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    values = {
        'feeder': _feeder(),
        'callback': _callback(),
        'terminal': _terminal(),
        'timing': _timing(),
    }
    paths = {name: tmp_path / f'{name}.json' for name in values}
    for name, value in values.items():
        _write(paths[name], value)
    return paths, values


def _bind(tmp_path: Path, paths: dict[str, Path], *, output: Path | None = None) -> dict[str, Any]:
    return BINDER.bind_consumer_evidence(
        feeder_path=paths['feeder'],
        callback_path=paths['callback'],
        terminal_path=paths['terminal'],
        timing_path=paths['timing'],
        output_path=output or (tmp_path / 'bound.json'),
        profile_path=PROFILE,
        expected_profile_sha256=PROFILE_SHA,
    )


def test_positive_binds_separate_sources_and_preserves_raw_hashes(tmp_path):
    paths, values = _fixture(tmp_path)
    before = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()}
    receipt = _bind(tmp_path, paths)
    assert receipt['status'] == 'BOUND'
    assert Path(receipt['output_path']).stat().st_mode & 0o222 == 0
    document = json.loads(Path(receipt['output_path']).read_text(encoding='utf-8'))
    assert document['contract_id'] == BINDER.BOUND_CONTRACT
    assert {'feeder', 'transport', 'terminal_support_context', 'timing'} <= set(document)
    assert document['transport']['raw']['transport_contract_version'] == BINDER.TRANSPORT_CONTRACT
    assert document['terminal_support_context']['validation']['status'] == 'pass'
    assert document['sources']['callback_sha256'] == before['callback']
    assert document['sources']['terminal_sha256'] == before['terminal']
    assert {
        name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()
    } == before


@pytest.mark.parametrize(
    'mutation',
    [
        'callback_authority',
        'callback_transport',
        'callback_counts',
        'terminal_old_contract',
        'terminal_equal_lidar',
        'terminal_preboundary',
        'terminal_gap',
        'terminal_drop',
        'timing_wrong',
        'profile_drift',
    ],
)
def test_negative_raw_or_profile_drift_fails_closed(tmp_path, mutation):
    paths, values = _fixture(tmp_path)
    if mutation == 'callback_authority':
        values['callback']['input'] = {'bag_path': 'wrong'}
    elif mutation == 'callback_transport':
        values['callback']['transport_contract_version'] = 'wrong'
    elif mutation == 'callback_counts':
        values['callback']['consumer']['received_messages'] = 1
    elif mutation == 'terminal_old_contract':
        values['terminal']['terminal_support_context'][
            'classification'
        ] = 'post_boundary_support_context_only'
    elif mutation == 'terminal_equal_lidar':
        values['terminal'] = _terminal(record=_record('lidar'))
        values['terminal']['terminal_support_context']['by_topic'] = {
            'lidar': 1,
            'imu': 0,
            'image': 0,
        }
        values['terminal']['buffers'] = {
            'lidar': _buffer('lidar', [_record('lidar')]),
            'imu': _buffer('imu', []),
            'image': _buffer('image', []),
        }
    elif mutation == 'terminal_preboundary':
        row = _record(timestamp=BOUNDARY - 0.01, equal=False)
        values['terminal'] = _terminal(record=row)
    elif mutation == 'terminal_gap':
        values['terminal']['trajectory']['last_timestamp_seconds'] = REQUIRED_END - 0.3
        values['terminal']['trajectory']['end_gap_seconds'] = 0.3
    elif mutation == 'terminal_drop':
        values['terminal']['dropped_counts']['imu'] = 1
        values['terminal']['backend']['dropped_counts']['imu'] = 1
    elif mutation == 'timing_wrong':
        values['timing']['online_compute_rtf'] = -1.0
    elif mutation == 'profile_drift':
        with pytest.raises(BINDER.BinderError):
            BINDER.bind_consumer_evidence(
                feeder_path=paths['feeder'],
                callback_path=paths['callback'],
                terminal_path=paths['terminal'],
                timing_path=paths['timing'],
                output_path=tmp_path / 'bound.json',
                profile_path=PROFILE,
                expected_profile_sha256='0' * 64,
            )
        return
    for name, value in values.items():
        _write(paths[name], value)
    with pytest.raises((BINDER.BinderError, BINDER.PhaseContractError)):
        _bind(tmp_path, paths)


def test_missing_timing_and_output_overwrite_are_rejected(tmp_path):
    paths, _ = _fixture(tmp_path)
    paths['timing'].unlink()
    with pytest.raises((BINDER.BinderError, OSError)):
        _bind(tmp_path, paths)
    paths, _ = _fixture(tmp_path / 'fresh')
    output = tmp_path / 'fresh' / 'bound.json'
    _bind(tmp_path / 'fresh', paths, output=output)
    with pytest.raises(BINDER.BinderError):
        _bind(tmp_path / 'fresh', paths, output=output)


@pytest.mark.parametrize(
    ('keyword', 'value'),
    [
        ('input_path', '/tmp/not-the-pinned-bag.bag'),
        ('input_bytes', 1),
        ('input_sha256', '0' * 64),
    ],
)
def test_authoritative_input_arguments_are_pinned(tmp_path, keyword, value):
    paths, _ = _fixture(tmp_path)
    arguments = {
        'feeder_path': paths['feeder'],
        'callback_path': paths['callback'],
        'terminal_path': paths['terminal'],
        'timing_path': paths['timing'],
        'output_path': tmp_path / 'bound.json',
        'profile_path': PROFILE,
        'expected_profile_sha256': PROFILE_SHA,
    }
    arguments[keyword] = value
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(**arguments)
    assert error.value.kind == 'INPUT_ARGUMENT_DRIFT'
    assert not (tmp_path / 'bound.json').exists()


def test_raw_symlink_and_terminal_authority_are_rejected(tmp_path):
    paths, values = _fixture(tmp_path)
    callback_link = tmp_path / 'callback-link.json'
    callback_link.symlink_to(paths['callback'])
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(
            paths['feeder'],
            callback_link,
            paths['terminal'],
            paths['timing'],
            tmp_path / 'symlink-bound.json',
            PROFILE,
            PROFILE_SHA,
        )
    assert error.value.kind in {'SYMLINK_REJECTED', 'SYMLINK_OR_NOT_REGULAR'}

    values['terminal']['contract_version'] = BINDER.CONSUMER_CONTRACT
    _write(paths['terminal'], values['terminal'])
    with pytest.raises(BINDER.BinderError) as error:
        _bind(tmp_path, paths)
    assert error.value.kind == 'RAW_AUTHORITY_FIELDS'


def test_attempt4_v11_terminal_classification_remains_invalid(tmp_path):
    paths, values = _fixture(tmp_path)
    values['terminal']['terminal_support_context'][
        'classification'
    ] = 'post_boundary_support_context_only'
    _write(paths['terminal'], values['terminal'])
    with pytest.raises((BINDER.BinderError, BINDER.PhaseContractError)):
        _bind(tmp_path, paths)
