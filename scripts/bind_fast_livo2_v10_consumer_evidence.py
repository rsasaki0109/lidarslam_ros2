#!/usr/bin/env python3
"""Bind raw FAST-LIVO2 v10 mapper evidence to the pinned host identity.

The mapper is the authority for its observed fields.  Its raw schema-1
document intentionally has no input or profile authority fields.  This
module only verifies that shape, loads the immutable preregistered profile,
and adds the profile-derived ``input`` identity and ``profile_sha256``.
It never opens the bag, invokes Docker, runs a mapper, accesses ground truth,
or invokes a scorer.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Optional

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'
PROFILE_SHA256 = (
    'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459')
PROFILE_KEY = 'm6a10_fast_livo2_v2c_v10'
PROFILE_SCHEMA_VERSION = 3
PROFILE_SYSTEM = 'fast_livo2'
PROFILE_STATUS = 'preregistered_not_built'
PROFILE_CONTRACT_ID = 'm6a10-v2c-fast-livo2-terminal-support-context-v10'

RAW_SCHEMA_VERSION = 1
RAW_CONTRACT = 'm6a10-fast-livo2-consumer-terminal-v1'
TOPICS = ('lidar', 'imu', 'image')

EXPECTED_BAG_PATH = (
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag')
EXPECTED_BAG_BYTES = 11290464091
EXPECTED_BAG_SHA256 = (
    '5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310')
EXPECTED_MESSAGES = 236687
EXPECTED_TOPIC_COUNTS = {'lidar': 5793, 'imu': 225102, 'image': 5792}
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352

# Authority fields are preregistered or feeder-derived and must never arrive
# from the mapper.  The output document adds only ``input`` and
# ``profile_sha256``; raw path/SHA are returned in the binder receipt.
AUTHORITY_FIELDS = frozenset({
    'input', 'profile_sha256', 'profile_path', 'raw_path', 'raw_sha256',
    'expected', 'expected_messages', 'expected_topic_counts',
    'published', 'published_messages', 'published_topic_counts',
    'ack', 'acked', 'acked_messages', 'acked_topic_counts',
    'acknowledgement', 'acknowledgements',
    'acknowledged', 'acknowledged_messages', 'acknowledged_topic_counts',
    'bag_path', 'bag_bytes', 'bag_sha256',
    'required_evaluation_end_timestamp_seconds',
    'binding', 'validation',
})


class BinderError(ValueError):
    """A fail-closed raw-evidence binding error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _regular_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise BinderError('SYMLINK_OR_NOT_REGULAR', f'{label} is not a regular file')


def _reject_symlink_components(path: Path, label: str) -> None:
    """Reject an existing symlink anywhere in a path used by the binder."""
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise BinderError('SYMLINK_REJECTED', f'{label} contains symlink: {current}')


def _load_json_bytes(path: Path, label: str) -> tuple[dict[str, Any], bytes]:
    _regular_file(path, label)
    try:
        raw_bytes = path.read_bytes()
        value = json.loads(raw_bytes.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise BinderError('RAW_JSON_INVALID', f'{label} JSON is invalid') from error
    if not isinstance(value, dict):
        raise BinderError('RAW_DOCUMENT_NOT_OBJECT', f'{label} must be a JSON object')
    return value, raw_bytes


def _counts(value: Any, label: str) -> dict[str, int]:
    if not isinstance(value, dict) or set(value) != set(TOPICS):
        raise BinderError('RAW_COUNTS_INVALID', f'{label} must contain exactly {TOPICS}')
    result: dict[str, int] = {}
    for topic in TOPICS:
        count = value.get(topic)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise BinderError('RAW_COUNTS_INVALID', f'{label}.{topic} is invalid')
        result[topic] = count
    return result


def _profile(profile_path: Path = PROFILE_PATH) -> tuple[dict[str, Any], str, dict[str, Any]]:
    """Load and verify the pinned profile without opening its bag input."""
    _regular_file(profile_path, 'profile')
    observed_sha = file_sha256(profile_path)
    if observed_sha != PROFILE_SHA256:
        raise BinderError('PROFILE_DRIFT', 'profile SHA-256 differs from pinned v10 profile')
    try:
        document = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise BinderError('PROFILE_INVALID', 'profile YAML is invalid') from error
    if not isinstance(document, dict):
        raise BinderError('PROFILE_INVALID', 'profile must be a mapping')
    profiles = document.get('competitive_slam_profile')
    if not isinstance(profiles, dict):
        raise BinderError('PROFILE_INVALID', 'competitive_slam_profile is missing')
    profile = profiles.get(PROFILE_KEY)
    if not isinstance(profile, dict):
        raise BinderError('PROFILE_INVALID', 'pinned v10 profile key is missing')
    if (profile.get('schema_version') != PROFILE_SCHEMA_VERSION or
            profile.get('profile_key') != PROFILE_KEY or
            profile.get('system') != PROFILE_SYSTEM or
            profile.get('status') != PROFILE_STATUS or
            profile.get('contract_id') != PROFILE_CONTRACT_ID):
        raise BinderError('PROFILE_INVALID', 'v10 profile system/key/schema/status mismatch')

    input_identity = profile.get('input')
    phase = profile.get('phase')
    if not isinstance(input_identity, dict) or not isinstance(phase, dict):
        raise BinderError('PROFILE_INVALID', 'v10 profile input/phase is missing')
    if (phase.get('schema_version') != PROFILE_SCHEMA_VERSION or
            phase.get('contract_version') != 'm6a10-online-compute-v3-terminal-support-context' or
            phase.get('mode') != 'unpaced_ack'):
        raise BinderError('PROFILE_INVALID', 'v10 profile phase schema/contract/mode mismatch')
    if (not isinstance(input_identity.get('path'), str) or
            not isinstance(input_identity.get('bytes'), int) or
            isinstance(input_identity.get('bytes'), bool) or
            not isinstance(input_identity.get('sha256'), str) or
            not isinstance(input_identity.get('expected_messages'), int) or
            isinstance(input_identity.get('expected_messages'), bool)):
        raise BinderError('PROFILE_INPUT_IDENTITY_MISMATCH', 'profile bag identity has invalid types')
    if (input_identity.get('path') != EXPECTED_BAG_PATH or
            input_identity.get('bytes') != EXPECTED_BAG_BYTES or
            input_identity.get('sha256') != EXPECTED_BAG_SHA256 or
            input_identity.get('expected_messages') != EXPECTED_MESSAGES):
        raise BinderError('PROFILE_INPUT_IDENTITY_MISMATCH', 'profile bag identity differs from pin')
    counts = input_identity.get('expected_topic_counts')
    try:
        profile_counts = _counts(counts, 'profile expected_topic_counts')
    except BinderError as error:
        raise BinderError('PROFILE_INPUT_COUNTS_MISMATCH',
                           'profile expected topic counts are structurally invalid') from error
    if profile_counts != EXPECTED_TOPIC_COUNTS:
        raise BinderError('PROFILE_INPUT_COUNTS_MISMATCH', 'profile expected topic counts differ from pin')
    required_end = phase.get('required_evaluation_end_timestamp_seconds')
    if (isinstance(required_end, bool) or
            not isinstance(required_end, (int, float)) or
            float(required_end) != REQUIRED_END_TIMESTAMP_SECONDS):
        raise BinderError('PROFILE_END_TIMESTAMP_MISMATCH', 'profile required end differs from pin')
    source = phase.get('required_evaluation_end_source')
    if (not isinstance(source, dict) or
            source.get('field') != 'dataset.sensor_end_timestamp_seconds'):
        raise BinderError('PROFILE_END_SOURCE_INVALID', 'profile required end source is invalid')
    expected = {
        'bag_path': EXPECTED_BAG_PATH,
        'bag_bytes': EXPECTED_BAG_BYTES,
        'bag_sha256': EXPECTED_BAG_SHA256,
        'expected_messages': EXPECTED_MESSAGES,
        'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
        'required_end_timestamp_seconds': REQUIRED_END_TIMESTAMP_SECONDS,
    }
    return document, observed_sha, expected


def _reject_authority_fields(raw: Mapping[str, Any]) -> None:
    present = sorted(
        key for key in raw
        if isinstance(key, str) and key.lower() in AUTHORITY_FIELDS)
    if present:
        raise BinderError(
            'RAW_AUTHORITY_FIELDS',
            f'raw mapper document contains authority fields: {present}')
    # Keep the rejection fail-closed for future spellings of the same classes
    # without rejecting ordinary observation names such as received_topic_counts.
    for key in raw:
        if not isinstance(key, str):
            raise BinderError('RAW_AUTHORITY_FIELDS', 'raw mapper document has a non-string key')
        normalized = key.lower()
        if (normalized.startswith(('expected_', 'published_', 'ack_', 'acked_',
                                    'acknowledged_', 'acknowledgement_')) or
                normalized.endswith(('_expected', '_published', '_ack', '_acked',
                                     '_acknowledged'))):
            raise BinderError('RAW_AUTHORITY_FIELDS', f'raw mapper authority field: {key}')


def _validate_raw(raw: Mapping[str, Any]) -> None:
    if raw.get('schema_version') != RAW_SCHEMA_VERSION or \
            raw.get('contract_id') != RAW_CONTRACT:
        raise BinderError('RAW_SCHEMA_OR_CONTRACT', 'raw schema or contract is invalid')
    if raw.get('system') != PROFILE_SYSTEM:
        raise BinderError('RAW_SYSTEM_INVALID', 'raw system is not fast_livo2')
    if raw.get('status') not in ('pass', 'invalid'):
        raise BinderError('RAW_STATUS_INVALID', 'raw status must be pass or invalid')
    _reject_authority_fields(raw)
    required = {
        'received_topic_counts', 'ground_truth_content_opened', 'scorer_invoked',
    }
    missing = sorted(field for field in required if field not in raw)
    if missing:
        raise BinderError('RAW_REQUIRED_FIELD_MISSING', f'raw fields missing: {missing}')
    # This is intentionally structural only.  It must not compare observed
    # counts to preregistered expected counts or copy expected values into them.
    _counts(raw.get('received_topic_counts'), 'received_topic_counts')
    if raw.get('ground_truth_content_opened') is not False or \
            raw.get('scorer_invoked') is not False:
        raise BinderError('RAW_SAFETY_FLAGS', 'raw GT/scorer safety flags must be false')


def _augment_raw_bytes(raw_bytes: bytes, input_identity: Mapping[str, Any],
                       profile_sha: str) -> bytes:
    """Insert only the two host-authority fields without reserializing raw data."""
    end = len(raw_bytes) - 1
    while end >= 0 and raw_bytes[end] in b' \t\r\n':
        end -= 1
    if end < 0 or raw_bytes[end] != ord('}'):
        raise BinderError('RAW_JSON_INVALID', 'raw object has no closing brace')
    insertion = (
        b',\n  "input": ' +
        json.dumps(dict(input_identity), ensure_ascii=False, separators=(',', ':')).encode('utf-8') +
        b',\n  "profile_sha256": ' +
        json.dumps(profile_sha).encode('ascii') + b'\n')
    result = raw_bytes[:end] + insertion + raw_bytes[end:]
    try:
        value = json.loads(result.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise BinderError('RAW_BINDING_INVALID', 'augmented host-bound JSON is invalid') from error
    if not isinstance(value, dict) or value.get('input') != dict(input_identity) or \
            value.get('profile_sha256') != profile_sha:
        raise BinderError('RAW_BINDING_INVALID', 'host identity fields were not bound')
    return result


def _atomic_create(path: Path, payload: bytes) -> str:
    """Create a new file atomically, refusing target and stale-part overwrite."""
    if path.exists() or path.is_symlink() or path.name.endswith('.part'):
        raise BinderError('OUTPUT_OVERWRITE', f'output already exists or is staging: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise BinderError('OUTPUT_OVERWRITE', f'stale output staging file exists: {part}')
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        # Hard-link publication is atomic and cannot replace an existing path.
        os.link(part, path)
    except FileExistsError as error:
        raise BinderError('OUTPUT_OVERWRITE', f'output appeared during creation: {path}') from error
    except OSError as error:
        raise BinderError('OUTPUT_CREATE_FAILED', f'cannot publish output: {path}') from error
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    return hashlib.sha256(payload).hexdigest()


def bind_consumer_evidence(raw_path: Path, output_path: Path,
                           profile_path: Path = PROFILE_PATH) -> dict[str, Any]:
    """Bind one raw mapper document to the pinned profile and publish it."""
    raw_path = Path(raw_path)
    output_path = Path(output_path)
    profile_path = Path(profile_path)
    _reject_symlink_components(raw_path, 'raw path')
    _reject_symlink_components(profile_path, 'profile path')
    _reject_symlink_components(output_path, 'output path')
    _, profile_sha, expected = _profile(profile_path)
    raw, raw_bytes = _load_json_bytes(raw_path, 'raw mapper')
    _validate_raw(raw)
    input_identity = {
        'bag_path': expected['bag_path'],
        'bag_bytes': expected['bag_bytes'],
        'bag_sha256': expected['bag_sha256'],
    }
    bound_bytes = _augment_raw_bytes(raw_bytes, input_identity, profile_sha)
    output_sha = _atomic_create(output_path, bound_bytes)
    bound = json.loads(bound_bytes.decode('utf-8'))
    return {
        'status': 'PASS',
        'receipt_kind': 'fast_livo2_v10_host_bound_consumer_evidence',
        'raw_path': str(raw_path.resolve()),
        'raw_sha256': hashlib.sha256(raw_bytes).hexdigest(),
        'raw_bytes': len(raw_bytes),
        'profile_path': str(profile_path.resolve()),
        'profile_sha256': profile_sha,
        'output_path': str(output_path.resolve()),
        'output_sha256': output_sha,
        'safety': {
            'bag_opened': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        },
        'document': bound,
    }


def bind(raw_path: Path, output_path: Path,
         profile_path: Path = PROFILE_PATH) -> dict[str, Any]:
    """Short alias for the host binder API."""
    return bind_consumer_evidence(
        raw_path=raw_path, profile_path=profile_path, output_path=output_path)


bind_raw_consumer_evidence = bind_consumer_evidence


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=PROFILE_PATH)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _parser().parse_args(argv)
    receipt = bind_consumer_evidence(
        raw_path=args.raw, profile_path=args.profile, output_path=args.output)
    print(json.dumps({key: value for key, value in receipt.items() if key != 'document'},
                     sort_keys=True, separators=(',', ':')))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (BinderError, OSError, UnicodeError, ValueError, yaml.YAMLError) as error:
        print(f'FAST-LIVO2 v10 consumer binding failed: {error}', file=sys.stderr)
        raise SystemExit(2)
