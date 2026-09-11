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

"""Fail-closed tests for the production v10 raw-consumer host binder."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'bind_fast_livo2_v10_consumer_evidence',
    ROOT / 'scripts/bind_fast_livo2_v10_consumer_evidence.py')
BINDER = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(BINDER)


PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'


def _raw_bytes(*, received: tuple[int, int, int] = (0, 1, 2), **extra: object) -> bytes:
    """Keep unusual JSON spellings here to test byte-preserving augmentation."""
    fields = [
        b'  "schema_version": 1',
        b'  "contract_id": "m6a10-fast-livo2-consumer-terminal-v1"',
        b'  "phase_mode": "unpaced_ack"',
        b'  "system": "fast_livo2"',
        b'  "status": "pass"',
        (b'  "received_topic_counts": {"lidar": %d, "imu": %d, "image": %d}' % received),
        b'  "terminal_observation": {"minimum_poll_wall_seconds": 0.050000000000000003}',
        b'  "raw_observation": {"timestamp_seconds": 1623491515.1483520, "label": "raw"}',
        b'  "ground_truth_content_opened": false',
        b'  "scorer_invoked": false',
    ]
    for key, value in extra.items():
        fields.append(json.dumps({key: value}, separators=(',', ':')).encode('utf-8')[1:-1])
    return b'{\n' + b',\n'.join(fields) + b'\n}\n'


def _write_raw(path: Path, *, received: tuple[int, int, int] = (0, 1, 2),
               **extra: object) -> bytes:
    payload = _raw_bytes(received=received, **extra)
    path.write_bytes(payload)
    return payload


def test_binding_adds_only_host_identity_and_preserves_raw_bytes(tmp_path):
    raw_path = tmp_path / 'raw.json'
    raw_bytes = _write_raw(raw_path)
    output_path = tmp_path / 'bound.json'

    receipt = BINDER.bind_consumer_evidence(raw_path, output_path, PROFILE)
    bound_bytes = output_path.read_bytes()
    bound = json.loads(bound_bytes)
    raw = json.loads(raw_bytes)
    _, _, expected = BINDER._profile(PROFILE)

    assert set(bound) == set(raw) | {'input', 'profile_sha256'}
    assert bound['input'] == {
        'bag_path': expected['bag_path'],
        'bag_bytes': expected['bag_bytes'],
        'bag_sha256': expected['bag_sha256'],
    }
    assert bound['profile_sha256'] == BINDER.PROFILE_SHA256
    assert bound['received_topic_counts'] == {'lidar': 0, 'imu': 1, 'image': 2}
    # The exact raw prefix, including float spellings and observation order,
    # remains untouched; only fields before the final object brace are added.
    assert bound_bytes.startswith(raw_bytes[:raw_bytes.rfind(b'}')])
    assert bound_bytes.endswith(b'}\n')
    assert receipt['raw_path'] == str(raw_path.resolve())
    assert receipt['raw_sha256'] == hashlib.sha256(raw_bytes).hexdigest()
    assert receipt['raw_bytes'] == len(raw_bytes)
    assert receipt['output_sha256'] == hashlib.sha256(bound_bytes).hexdigest()
    assert receipt['safety'] == {
        'bag_opened': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    assert not (tmp_path / 'bound.json.part').exists()


@pytest.mark.parametrize(
    ('mutation', 'kind'),
    [
        (lambda value: value.update(schema_version=2), 'RAW_SCHEMA_OR_CONTRACT'),
        (lambda value: value.update(contract_id='wrong'), 'RAW_SCHEMA_OR_CONTRACT'),
        (lambda value: value.update(status='pass_with_warning'), 'RAW_STATUS_INVALID'),
        (lambda value: value.pop('ground_truth_content_opened'), 'RAW_REQUIRED_FIELD_MISSING'),
        (lambda value: value.update(expected_topic_counts={'lidar': 1, 'imu': 1, 'image': 1}),
         'RAW_AUTHORITY_FIELDS'),
        (lambda value: value.update(published_topic_counts={'lidar': 1, 'imu': 1, 'image': 1}),
         'RAW_AUTHORITY_FIELDS'),
        (lambda value: value.update(ACK={'lidar': 1}), 'RAW_AUTHORITY_FIELDS'),
        (lambda value: value.update(ack_count=1), 'RAW_AUTHORITY_FIELDS'),
        (lambda value: value.update(received_topic_counts={'lidar': 0, 'imu': -1, 'image': 2}),
         'RAW_COUNTS_INVALID'),
        (lambda value: value.update(received_topic_counts={'lidar': 0, 'imu': True, 'image': 2}),
         'RAW_COUNTS_INVALID'),
    ])
def test_raw_contract_and_authority_validation_fails_closed(tmp_path, mutation, kind):
    raw_path = tmp_path / 'raw.json'
    _write_raw(raw_path)
    value = json.loads(raw_path.read_text(encoding='utf-8'))
    mutation(value)
    raw_path.write_text(json.dumps(value), encoding='utf-8')

    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'bound.json', PROFILE)
    assert error.value.kind == kind
    assert not (tmp_path / 'bound.json').exists()


def test_profile_drift_and_symlink_inputs_are_rejected(tmp_path):
    raw_path = tmp_path / 'raw.json'
    _write_raw(raw_path)

    drifted = tmp_path / 'profile-drift.yaml'
    drifted.write_bytes(PROFILE.read_bytes().replace(
        b'status: preregistered_not_built', b'status: changed'))
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'drift.json', drifted)
    assert error.value.kind == 'PROFILE_DRIFT'

    raw_link = tmp_path / 'raw-link.json'
    raw_link.symlink_to(raw_path)
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_link, tmp_path / 'link.json', PROFILE)
    assert error.value.kind == 'SYMLINK_REJECTED'

    profile_link = tmp_path / 'profile-link.yaml'
    profile_link.symlink_to(PROFILE)
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'profile-link.json', profile_link)
    assert error.value.kind == 'SYMLINK_REJECTED'


def test_symlink_component_and_output_overwrite_are_rejected(tmp_path):
    raw_path = tmp_path / 'raw.json'
    raw_bytes = _write_raw(raw_path)

    real_dir = tmp_path / 'real'
    real_dir.mkdir()
    link_dir = tmp_path / 'linked'
    link_dir.symlink_to(real_dir, target_is_directory=True)
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, link_dir / 'bound.json', PROFILE)
    assert error.value.kind == 'SYMLINK_REJECTED'

    output = tmp_path / 'bound.json'
    output.write_bytes(b'keep-me')
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, output, PROFILE)
    assert error.value.kind == 'OUTPUT_OVERWRITE'
    assert output.read_bytes() == b'keep-me'

    part = tmp_path / 'new-bound.json.part'
    part.write_bytes(b'old-staging')
    with pytest.raises(BINDER.BinderError) as error:
        BINDER.bind_consumer_evidence(raw_path, tmp_path / 'new-bound.json', PROFILE)
    assert error.value.kind == 'OUTPUT_OVERWRITE'
    assert part.read_bytes() == b'old-staging'
    assert hashlib.sha256(raw_bytes).hexdigest() == BINDER.file_sha256(raw_path)
