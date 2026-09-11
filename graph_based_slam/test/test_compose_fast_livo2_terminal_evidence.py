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

"""Host-binding tests for the FAST-LIVO2 v10 terminal compositor."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'compose_fast_livo2_terminal_evidence',
    ROOT / 'scripts/compose_fast_livo2_terminal_evidence.py',
)
COMPOSE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(COMPOSE)


PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'
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
    stamps = [item['timestamp_seconds'] for item in records]
    return {
        'count': len(records),
        'oldest_timestamp_seconds': min(stamps) if stamps else None,
        'newest_timestamp_seconds': max(stamps) if stamps else None,
        'records': records,
    }


def _fixtures(tmp_path: Path) -> tuple[Path, Path, dict, dict]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    _, expected, profile_sha = COMPOSE._profile(PROFILE)
    residual = {
        'lidar': [],
        'imu': [_record('imu', 225102, expected['required_end'] + 0.01)],
        'image': [_record('image', 5792, expected['required_end'] + 0.02)],
    }
    received = dict(expected['expected_topic_counts'])
    completed = {topic: received[topic] - len(residual[topic]) for topic in TOPICS}
    support = [row for topic in TOPICS for row in residual[topic]]
    consumer = {
        'schema_version': 1,
        'contract_id': COMPOSE.CONSUMER_CONTRACT,
        'status': 'pass',
        'input': {
            'bag_path': expected['path'],
            'bag_bytes': expected['bytes'],
            'bag_sha256': expected['sha256'],
        },
        'profile_sha256': profile_sha,
        'received_topic_counts': received,
        'backend': {
            'quiescent': True,
            'quiescence_observed': True,
            'completed_boundary': {
                'observed': True,
                'timestamp_seconds': expected['required_end'],
                'sequence': 42,
                'source': 'fixture.completed_estimator_boundary',
            },
            'completed_counts': completed,
            'completed_synchronization_units': 2,
            'dropped_counts': {topic: 0 for topic in TOPICS},
            'overflow_counts': {topic: 0 for topic in TOPICS},
            'processing_failures': 0,
            'in_flight': {'active': False, 'topic': None, 'unit_id': None},
        },
        'buffers': {topic: _buffer(residual[topic]) for topic in TOPICS},
        'terminal_support_context': {
            'classification': 'post_boundary_support_context_only',
            'total_count': len(support),
            'by_topic': {topic: len(residual[topic]) for topic in TOPICS},
            'records': support,
        },
        'trajectory': {
            'coverage_verified': True,
            'last_timestamp_seconds': expected['required_end'],
            'end_gap_seconds': 0.0,
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
        'feeder_nonce': 'fixture-nonce',
    }
    feeder = {
        'schema_version': 1,
        'contract_id': COMPOSE.FEEDER_CONTRACT,
        'status': 'pass',
        'bag_path': expected['path'],
        'bag_bytes': expected['bytes'],
        'bag_sha256': expected['sha256'],
        'expected_topic_counts': dict(expected['expected_topic_counts']),
        'published_topic_counts': dict(expected['expected_topic_counts']),
        'acked_topic_counts': dict(expected['expected_topic_counts']),
        'published_messages': sum(expected['expected_topic_counts'].values()),
        'single_inflight': True,
        'publisher_queue_size': 1,
        'ack_source_kind': 'consumer_status_callback_counter',
        'ack_backpressure_verified': True,
        'sequence': 17,
        'nonce': 'fixture-nonce',
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    consumer_path = tmp_path / 'consumer.json'
    feeder_path = tmp_path / 'feeder.json'
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    feeder_path.write_text(json.dumps(feeder), encoding='utf-8')
    return consumer_path, feeder_path, consumer, feeder


def test_valid_host_composition_binds_receipts_and_runs_v3(tmp_path):
    consumer_path, feeder_path, _, _ = _fixtures(tmp_path)
    output = tmp_path / 'terminal.json'
    result = COMPOSE.compose(
        profile_path=PROFILE,
        consumer_path=consumer_path,
        feeder_path=feeder_path,
        output_path=output,
    )
    assert result['status'] == 'pass'
    assert result['validation']['count_conservation_passed'] is True
    assert result['binding']['exact_topic_counts_bound'] is True
    assert json.loads(output.read_text())['status'] == 'pass'


def test_consumer_and_feeder_file_hashes_are_recorded(tmp_path):
    consumer_path, feeder_path, _, _ = _fixtures(tmp_path)
    result = COMPOSE.compose(
        profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path
    )
    assert result['binding']['consumer_sha256'] == COMPOSE._sha256(consumer_path)
    assert result['binding']['feeder_sha256'] == COMPOSE._sha256(feeder_path)
    assert result['ground_truth_content_opened'] is False
    assert result['scorer_invoked'] is False


def test_observed_count_mismatch_is_rejected_without_copying_expected(tmp_path):
    consumer_path, feeder_path, consumer, _ = _fixtures(tmp_path)
    consumer['received_topic_counts']['imu'] -= 1
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='consumer_received_count_mismatch'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)


def test_feeder_bag_identity_and_ack_counts_are_bound_exactly(tmp_path):
    consumer_path, feeder_path, consumer, feeder = _fixtures(tmp_path)
    feeder['bag_sha256'] = '0' * 64
    feeder_path.write_text(json.dumps(feeder), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='feeder_bag_sha256_mismatch'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)

    _, feeder_path, _, feeder = _fixtures(tmp_path / 'second')
    feeder['acked_topic_counts']['image'] -= 1
    feeder_path.write_text(json.dumps(feeder), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='feeder_acknowledged_count_mismatch'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)


def test_missing_stale_and_duplicate_receipts_fail_closed(tmp_path):
    consumer_path, feeder_path, _, feeder = _fixtures(tmp_path)
    with pytest.raises(COMPOSE.PhaseContractError, match='feeder_missing'):
        COMPOSE.compose(
            profile_path=PROFILE,
            consumer_path=consumer_path,
            feeder_path=tmp_path / 'missing.json',
        )

    feeder['stale'] = True
    feeder_path.write_text(json.dumps(feeder), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='duplicate_or_stale'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)

    _, feeder_path, _, feeder = _fixtures(tmp_path / 'third')
    feeder['receipt_id'] = 'receipt-1'
    feeder['receipt_history'] = ['receipt-1']
    feeder_path.write_text(json.dumps(feeder), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='duplicate_id'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)


def test_optional_sequence_and_nonce_bindings_reject_stale_consumer(tmp_path):
    consumer_path, feeder_path, consumer, _ = _fixtures(tmp_path)
    consumer['feeder_sequence'] = 16
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='sequence_mismatch'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)

    consumer['feeder_sequence'] = 17
    consumer['feeder_nonce'] = 'old-nonce'
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='nonce_mismatch'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)


def test_terminal_observation_and_v3_validator_are_mandatory(tmp_path, monkeypatch):
    consumer_path, feeder_path, consumer, _ = _fixtures(tmp_path)
    consumer['terminal_observation']['sync_predicate_evaluated'] = False
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='terminal_observation_unproven'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)

    consumer['terminal_observation']['sync_predicate_evaluated'] = True
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    original = COMPOSE.validate_terminal_support_context_v3

    def fail_validator(*args, **kwargs):
        raise COMPOSE.PhaseContractError('generic_v3_validator_required')

    monkeypatch.setattr(COMPOSE, 'validate_terminal_support_context_v3', fail_validator)
    with pytest.raises(COMPOSE.PhaseContractError, match='generic_v3_validator_required'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)
    monkeypatch.setattr(COMPOSE, 'validate_terminal_support_context_v3', original)


@pytest.mark.parametrize('trajectory', [None, {'coverage_verified': False}])
def test_missing_or_uncovered_trajectory_is_rejected(tmp_path, trajectory):
    consumer_path, feeder_path, consumer, _ = _fixtures(tmp_path)
    if trajectory is None:
        consumer['trajectory'] = None
    else:
        consumer['trajectory'] = trajectory
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='trajectory_coverage_unverified'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)


def test_output_is_immutable_and_stale_part_rejected(tmp_path):
    consumer_path, feeder_path, _, _ = _fixtures(tmp_path)
    output = tmp_path / 'terminal.json'
    output.write_text('{}', encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='already exists'):
        COMPOSE.compose(
            profile_path=PROFILE,
            consumer_path=consumer_path,
            feeder_path=feeder_path,
            output_path=output,
        )

    output.unlink()
    (tmp_path / 'terminal.json.part').write_text('{}', encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='staging file'):
        COMPOSE.compose(
            profile_path=PROFILE,
            consumer_path=consumer_path,
            feeder_path=feeder_path,
            output_path=output,
        )


def test_consumer_cannot_supply_expected_or_feeder_observation_fields(tmp_path):
    consumer_path, feeder_path, consumer, _ = _fixtures(tmp_path)
    consumer['published_topic_counts'] = dict(consumer['received_topic_counts'])
    consumer_path.write_text(json.dumps(consumer), encoding='utf-8')
    with pytest.raises(COMPOSE.PhaseContractError, match='copies_observed'):
        COMPOSE.compose(profile_path=PROFILE, consumer_path=consumer_path, feeder_path=feeder_path)
