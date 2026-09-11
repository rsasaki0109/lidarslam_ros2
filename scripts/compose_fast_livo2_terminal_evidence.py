#!/usr/bin/env python3
"""Compose the host-bound FAST-LIVO2 v10 terminal evidence document.

The mapper and feeder are deliberately separate authorities.  The mapper
reports only accepted callbacks, queue dispositions, completion, and its
terminal buffer proof.  The feeder receipt reports the independently observed
publish/ACK transaction.  This module binds those documents to the immutable
profile and then runs the generic schema-3 validator before writing one
immutable result.  No expected count is copied into an observed field.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from lidarslam_benchmark_tools.benchmark_phase_contract import (  # noqa: E402
    CONTRACT_VERSION_V3,
    PhaseContractError,
    atomic_write_json,
    file_sha256,
    validate_terminal_support_context_v3,
)


TOPICS = ('lidar', 'imu', 'image')
CONSUMER_CONTRACT = 'm6a10-fast-livo2-consumer-terminal-v1'
FEEDER_CONTRACT = 'm6a10-v2c-fast-livo2-single-inflight-feeder-v1'
PROFILE_SCHEMA = 3


def _error(message: str) -> PhaseContractError:
    return PhaseContractError(message)


def _object(path: Path, name: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise _error(f'{name}_missing_or_not_regular')
    if path.name.endswith('.part'):
        raise _error(f'{name}_staging_file_not_authoritative')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error(f'{name}_json_invalid') from exc
    if not isinstance(value, dict):
        raise _error(f'{name}_must_be_object')
    return value


def _counts(value: Any, name: str) -> dict[str, int]:
    if not isinstance(value, dict) or set(value) != set(TOPICS):
        raise _error(f'{name}_missing_or_invalid')
    result: dict[str, int] = {}
    for topic in TOPICS:
        count = value.get(topic)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise _error(f'{name}_{topic}_invalid')
        result[topic] = count
    return result


def _sha256(path: Path) -> str:
    try:
        return file_sha256(path)
    except OSError as exc:
        raise _error(f'cannot_hash_{path}') from exc


def _profile(path: Path) -> tuple[dict[str, Any], dict[str, Any], str]:
    if path.is_symlink() or not path.is_file():
        raise _error('profile_missing_or_not_regular')
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise _error('profile_yaml_invalid') from exc
    if not isinstance(value, dict):
        raise _error('profile_must_be_object')
    profiles = value.get('competitive_slam_profile')
    if not isinstance(profiles, dict):
        raise _error('profile_competitive_slam_profile_missing')
    profile = profiles.get('m6a10_fast_livo2_v2c_v10')
    if not isinstance(profile, dict) or profile.get('schema_version') != PROFILE_SCHEMA:
        raise _error('profile_v10_schema_or_key_invalid')
    if profile.get('system') != 'fast_livo2' or \
            profile.get('status') != 'preregistered_not_built':
        raise _error('profile_system_or_status_invalid')
    inp = profile.get('input')
    phase = profile.get('phase')
    if not isinstance(inp, dict) or not isinstance(phase, dict):
        raise _error('profile_input_or_phase_missing')
    bag_path = inp.get('path')
    bag_sha = inp.get('sha256')
    bag_bytes = inp.get('bytes')
    if not isinstance(bag_path, str) or not bag_path.strip() or \
            not isinstance(bag_sha, str) or len(bag_sha) != 64 or \
            not isinstance(bag_bytes, int) or bag_bytes < 0:
        raise _error('profile_input_identity_invalid')
    expected = _counts(inp.get('expected_topic_counts'), 'profile_expected_topic_counts')
    required_end = phase.get('required_evaluation_end_timestamp_seconds')
    source = phase.get('required_evaluation_end_source')
    if not isinstance(required_end, (int, float)) or isinstance(required_end, bool) or \
            not isinstance(source, dict) or \
            not isinstance(source.get('path'), str) or \
            not isinstance(source.get('sha256'), str) or \
            len(source['sha256']) != 64 or \
            source.get('field') != 'dataset.sensor_end_timestamp_seconds':
        raise _error('profile_required_end_preregistration_invalid')
    required = {
        'path': bag_path,
        'sha256': bag_sha,
        'bytes': bag_bytes,
        'expected_topic_counts': expected,
        'required_end': float(required_end),
        'maximum_end_gap': phase.get('maximum_end_gap_seconds', 0.25),
        'required_end_source': {
            'path': source['path'], 'sha256': source['sha256'],
            'field': source['field'],
        },
    }
    return value, required, _sha256(path)


def _check_identity(value: dict[str, Any], expected: dict[str, Any], name: str) -> None:
    identity = value.get('input')
    if not isinstance(identity, dict):
        raise _error(f'{name}_input_identity_missing')
    if identity.get('bag_path') != expected['path']:
        raise _error(f'{name}_bag_path_mismatch')
    if identity.get('bag_sha256') != expected['sha256']:
        raise _error(f'{name}_bag_sha256_mismatch')
    if identity.get('bag_bytes') != expected['bytes']:
        raise _error(f'{name}_bag_bytes_mismatch')


def _validate_feeder(value: dict[str, Any], expected: dict[str, Any]) -> dict[str, Any]:
    if value.get('schema_version') != 1 or value.get('contract_id') != FEEDER_CONTRACT:
        raise _error('feeder_schema_or_contract_invalid')
    if value.get('status') != 'pass':
        raise _error('feeder_receipt_not_pass')
    if value.get('duplicate') is True or value.get('stale') is True or \
            value.get('duplicate_of') is not None:
        raise _error('feeder_receipt_duplicate_or_stale')
    receipt_id = value.get('receipt_id')
    history = value.get('receipt_history')
    if receipt_id is not None:
        if not isinstance(receipt_id, str) or not receipt_id.strip():
            raise _error('feeder_receipt_id_invalid')
        if isinstance(history, list) and receipt_id in history:
            raise _error('feeder_receipt_duplicate_id')
    _check_identity({
        'input': {
            'bag_path': value.get('bag_path'),
            'bag_sha256': value.get('bag_sha256'),
            'bag_bytes': value.get('bag_bytes'),
        }}, expected, 'feeder')
    expected_counts = _counts(value.get('expected_topic_counts'), 'feeder_expected')
    published = _counts(value.get('published_topic_counts'), 'feeder_published')
    acknowledged = _counts(value.get('acked_topic_counts'), 'feeder_acknowledged')
    if expected_counts != expected['expected_topic_counts']:
        raise _error('feeder_expected_count_mismatch')
    if published != expected_counts:
        raise _error('feeder_published_count_mismatch')
    if acknowledged != expected_counts:
        raise _error('feeder_acknowledged_count_mismatch')
    total = sum(expected_counts.values())
    if value.get('published_messages') != total or \
            value.get('single_inflight') is not True or \
            value.get('publisher_queue_size') != 1 or \
            value.get('ack_backpressure_verified') is not True:
        raise _error('feeder_transaction_proof_invalid')
    if not isinstance(value.get('ack_source_kind'), str) or \
            not value['ack_source_kind'].strip():
        raise _error('feeder_ack_source_missing')
    if value.get('ground_truth_content_opened') is not False or \
            value.get('scorer_invoked') is not False:
        raise _error('feeder_gt_or_scorer_flag_invalid')
    return {
        'expected': expected_counts,
        'published': published,
        'acknowledged': acknowledged,
        'receipt_id': receipt_id,
        'sequence': value.get('sequence'),
        'nonce': value.get('nonce'),
    }


def _validate_consumer(value: dict[str, Any], expected: dict[str, Any],
                       profile_sha: str, feeder: dict[str, Any]) -> None:
    if value.get('schema_version') != 1 or \
            value.get('contract_id') != CONSUMER_CONTRACT:
        raise _error('consumer_schema_or_contract_invalid')
    if value.get('status') != 'pass':
        raise _error('consumer_evidence_not_pass')
    _check_identity(value, expected, 'consumer')
    if value.get('profile_sha256') != profile_sha:
        raise _error('consumer_profile_sha256_mismatch')
    for forbidden in ('expected_topic_counts', 'published_topic_counts',
                      'acked_topic_counts', 'acknowledged_topic_counts'):
        if forbidden in value:
            raise _error(f'consumer_copies_observed_{forbidden}')
    received = _counts(value.get('received_topic_counts'), 'consumer_received')
    if received != expected['expected_topic_counts']:
        raise _error('consumer_received_count_mismatch')
    sequence = value.get('feeder_sequence')
    if feeder['sequence'] is not None and sequence is not None and sequence != feeder['sequence']:
        raise _error('consumer_feeder_sequence_mismatch')
    nonce = value.get('feeder_nonce')
    if feeder['nonce'] is not None and nonce is not None and nonce != feeder['nonce']:
        raise _error('consumer_feeder_nonce_mismatch')
    observation = value.get('terminal_observation')
    if not isinstance(observation, dict) or \
            observation.get('eof_observed') is not True or \
            observation.get('stable') is not True or \
            observation.get('identical_snapshot') is not True or \
            observation.get('sync_predicate_evaluated') is not True:
        raise _error('consumer_terminal_observation_unproven')
    for key in ('poll_count', 'stable_poll_count'):
        if isinstance(observation.get(key), bool) or not isinstance(observation.get(key), int) or \
                observation[key] < 2:
            raise _error(f'consumer_terminal_{key}_invalid')
    first = observation.get('first_poll_wall_seconds')
    stable = observation.get('stable_poll_wall_seconds')
    minimum = observation.get('minimum_poll_wall_seconds')
    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in (first, stable, minimum)) or \
            stable < first or stable - first < minimum:
        raise _error('consumer_terminal_poll_interval_invalid')
    backend = value.get('backend')
    if not isinstance(backend, dict) or backend.get('quiescent') is not True or \
            backend.get('quiescence_observed') is not True:
        raise _error('consumer_backend_quiescence_unproven')
    flight = backend.get('in_flight')
    if not isinstance(flight, dict) or flight.get('active') is not False:
        raise _error('consumer_backend_in_flight')
    if value.get('ground_truth_content_opened') is not False or \
            value.get('scorer_invoked') is not False:
        raise _error('consumer_gt_or_scorer_flag_invalid')


def compose(*, profile_path: Path, consumer_path: Path, feeder_path: Path,
            output_path: Path | None = None) -> dict[str, Any]:
    """Validate and compose one immutable v3 document."""
    _, expected, profile_sha = _profile(profile_path)
    feeder_value = _object(feeder_path, 'feeder')
    consumer_value = _object(consumer_path, 'consumer')
    feeder = _validate_feeder(feeder_value, expected)
    _validate_consumer(consumer_value, expected, profile_sha, feeder)
    received = _counts(consumer_value['received_topic_counts'], 'consumer_received')
    document: dict[str, Any] = {
        'schema_version': 3,
        'contract_version': CONTRACT_VERSION_V3,
        'phase_mode': 'unpaced_ack',
        'system': 'fast_livo2',
        'status': 'pass',
        'required_evaluation_end_timestamp_seconds': expected['required_end'],
        'maximum_end_gap_seconds': expected['maximum_end_gap'],
        'counts': {
            'expected': dict(expected['expected_topic_counts']),
            'published': dict(feeder['published']),
            'received': dict(received),
            'acknowledged': dict(feeder['acknowledged']),
        },
        'buffers': consumer_value['buffers'],
        'backend': consumer_value['backend'],
        'trajectory': consumer_value['trajectory'],
        'terminal_support_context': consumer_value['terminal_support_context'],
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'binding': {
            'profile_path': str(profile_path.resolve()),
            'profile_sha256': profile_sha,
            'consumer_path': str(consumer_path.resolve()),
            'consumer_sha256': _sha256(consumer_path),
            'feeder_path': str(feeder_path.resolve()),
            'feeder_sha256': _sha256(feeder_path),
            'bag_path': expected['path'],
            'bag_bytes': expected['bytes'],
            'bag_sha256': expected['sha256'],
            'feeder_contract': FEEDER_CONTRACT,
            'consumer_contract': CONSUMER_CONTRACT,
            'exact_topic_counts_bound': True,
            'expected_is_preregistered_only': True,
        },
        'terminal_observation': consumer_value['terminal_observation'],
    }
    # This call is mandatory: the compositor cannot declare PASS around the
    # generic schema-3 validator, even when all local bindings look correct.
    validated = validate_terminal_support_context_v3(
        document, maximum_end_gap_seconds=float(expected['maximum_end_gap']),
        require_pass=True)
    if output_path is not None:
        atomic_write_json(output_path, validated)
    return validated


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--consumer', type=Path, required=True)
    parser.add_argument('--feeder', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _args()
    compose(profile_path=args.profile.resolve(), consumer_path=args.consumer.resolve(),
            feeder_path=args.feeder.resolve(), output_path=args.output.resolve())
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, PhaseContractError, ValueError, yaml.YAMLError) as exc:
        print(f'FAST-LIVO2 v10 evidence composition failed: {exc}', file=sys.stderr)
        raise SystemExit(2)
