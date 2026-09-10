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

"""Adversarial, synthetic-only tests for the NTU VIRAL Stage A/B contract."""

from __future__ import annotations

import copy
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tarfile
import warnings
import zipfile

from lidarslam_benchmark_tools import check_competitive_dataset_source_closure as closure
from lidarslam_benchmark_tools.authorize_ntu_viral_pin import propose_pin_manifest
from lidarslam_benchmark_tools.ntu_viral_acquisition import (
    acquire_candidate,
    inspect_safe_archive,
    NtuAcquisitionError,
)
import pytest
import yaml

# launch_testing may collect this file with a non-repository import root.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SEQUENCES = ('eee_01', 'nya_01', 'spms_01')
UUID = '3b5dc9b7-c4de-4cf2-a892-00b2c063f34e'


def _zip_bytes(member: str, payload: bytes = b'synthetic-input') -> bytes:
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(member, payload)
    return stream.getvalue()


def _duplicate_zip_bytes() -> bytes:
    stream = io.BytesIO()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        with zipfile.ZipFile(stream, 'w') as archive:
            archive.writestr('safe', b'one')
            archive.writestr('safe', b'two')
    return stream.getvalue()


def _selection(evidence: Path) -> dict:
    def ref(url: str, filename: str, archive: bool, members=()):
        return {
            'url': url,
            'official_primary': True,
            'allowed_hosts': ['fixture.example', 'mirror.fixture.example'],
            'expected_filename': filename,
            'archive': archive,
            'expected_member_paths': list(members),
        }

    selected = []
    for sequence in SEQUENCES:
        selected.append(
            {
                'sequence_id': sequence,
                'profile_key': f'ntu_viral_{sequence}',
                'selection_rationale': 'synthetic adversarial fixture',
                'environment': 'synthetic',
                'official_size_label': 'synthetic',
                'official_duration_seconds': 1.0,
                'expected_files': ['sequence_archive', 'ground_truth_csv'],
                'supported_systems': ['ours', 'glim', 'fast_livo2'],
                'evaluation_eligible': True,
                'fresh_eligible': False,
                'sensor_contract': {
                    'official_modalities': ['lidar'],
                    'topics': None,
                    'formats': None,
                    'timestamp_basis': None,
                    'status': 'NOT_READY',
                },
                'ground_truth_contract': {
                    'official_method': 'synthetic opaque fixture',
                    'frame_convention': 'synthetic',
                    'timestamp_basis': 'synthetic',
                    'status': 'NOT_READY',
                },
                'calibration_contract': {
                    'expected_artifacts': ['calib_stereo.zip', 'calib_stereo_imu.bag'],
                    'frame_convention': 'synthetic',
                    'timestamp_basis': 'synthetic',
                    'status': 'NOT_READY',
                },
                'byte_identity': {
                    'input_sha256': None,
                    'input_size_bytes': None,
                    'ground_truth_sha256': None,
                    'ground_truth_size_bytes': None,
                    'calibration_sha256': None,
                    'calibration_size_bytes': None,
                    'validation_status': 'NOT_READY',
                },
                'download_refs': {
                    'sequence_archive': ref(
                        f'https://fixture.example/{sequence}.zip',
                        f'{sequence}.zip',
                        True,
                        [f'{sequence}/input.bin'],
                    ),
                    'ground_truth_csv': ref(
                        f'https://fixture.example/{sequence}.csv', f'{sequence}.csv', False
                    ),
                },
            }
        )
    return {
        'schema_version': 1,
        'selection_id': 'synthetic-ntu-viral-selection-v1',
        'status': 'NOT_READY',
        'family_id': 'ntu_viral',
        'partition': 'historical',
        'selection_basis': 'synthetic fixture only',
        'official_sources': {},
        'acquisition': {
            'recipe_id': 'synthetic-recipe',
            'recipe_revision': 'synthetic-r1',
            'materialization': 'synthetic bytes',
            'status': 'NOT_READY',
            'required_before_execution': ['synthetic identity'],
            'evidence_mount': {
                'path': str(evidence),
                'uuid': UUID,
                'filesystem': 'ext4',
                'label': 'aiueo',
                'require_rw': True,
            },
            'candidate_root_relative': 'datasets/ntu_viral_candidates/synthetic-v1',
            'calibration_roles': {
                'calib_stereo': ref(
                    'https://fixture.example/calib.zip',
                    'calib.zip',
                    True,
                    ['calibration/camera.yaml'],
                ),
                'calib_stereo_imu': ref(
                    'https://fixture.example/calib_imu.bag', 'calib_imu.bag', False
                ),
            },
        },
        'selected_sequences': selected,
        'excluded_sequences': [
            {
                'sequence_id': 'tnp_01',
                'profile_key': 'ntu_viral_tnp_01',
                'reason': 'development training exposed synthetic fixture',
                'evaluation_eligible': False,
                'fresh_eligible': False,
            }
        ],
        'disjointness': {
            'excluded_training_sequences': ['tnp_01'],
            'excluded_historical_families': ['hilti_slam_challenge_2022'],
            'required_immutable_ids': ['input_sha256', 'ground_truth_sha256'],
        },
    }


def _profile() -> dict:
    return {
        'competitive_slam_profile': {
            'evidence_gate_v2': {},
            'datasets': {
                'ntu_viral_historical': {
                    sequence: {
                        'dataset': f'ntu_viral_{sequence}',
                        'sequence': sequence,
                        'family': 'ntu_viral',
                        'evaluation_eligible': True,
                        'fresh_eligible': False,
                    }
                    for sequence in SEQUENCES
                },
                'ntu_viral_development': {
                    'tnp_01': {
                        'dataset': 'ntu_viral_tnp_01_development',
                        'sequence': 'tnp_01',
                        'evaluation_eligible': False,
                        'fresh_eligible': False,
                    },
                },
            },
        },
    }


class Fixture:
    def __init__(self, tmp_path: Path) -> None:
        self.root = tmp_path
        self.evidence = tmp_path / 'evidence'
        self.evidence.mkdir(parents=True)
        (self.evidence / 'datasets/ntu_viral_candidates').mkdir(parents=True)
        self.selection_path = tmp_path / 'selection.yaml'
        self.profile_path = tmp_path / 'profile.yaml'
        self.selection_document = _selection(self.evidence)
        self.profile_document = _profile()
        self.payloads: dict[str, bytes] = {}
        for sequence in SEQUENCES:
            self.payloads[f'https://fixture.example/{sequence}.zip'] = _zip_bytes(
                f'{sequence}/input.bin', sequence.encode()
            )
            self.payloads[f'https://fixture.example/{sequence}.csv'] = (
                f'opaque synthetic GT for {sequence}\n'
            ).encode()
        self.payloads['https://fixture.example/calib.zip'] = _zip_bytes(
            'calibration/camera.yaml', b'camera calibration'
        )
        self.payloads['https://fixture.example/calib_imu.bag'] = b'imu calibration'
        self.write_inputs()

    def write_inputs(self) -> None:
        self.selection_path.write_text(
            yaml.safe_dump(self.selection_document, sort_keys=False), encoding='utf-8'
        )
        self.profile_path.write_text(
            yaml.safe_dump(self.profile_document, sort_keys=False), encoding='utf-8'
        )

    @property
    def candidate(self) -> Path:
        return self.evidence / self.selection_document['acquisition']['candidate_root_relative']

    def observation(self, **changes):
        result = {
            'target': str(self.evidence),
            'source': '/dev/loop-synthetic',
            'uuid': UUID,
            'fstype': 'ext4',
            'label': 'aiueo',
            'options': 'rw',
        }
        result.update(changes)
        return result

    def transport(
        self,
        *,
        final_url: str | None = None,
        fail_after: int | None = None,
        fail_code: str = 'FIXTURE_FAILURE',
    ):
        count = 0

        def copy_fixture(url: str, destination: Path):
            nonlocal count
            count += 1
            if fail_after is not None and count > fail_after:
                destination.write_bytes(b'partial fixture')
                raise NtuAcquisitionError(fail_code, 'synthetic transport failure')
            destination.write_bytes(self.payloads[url])
            target = final_url or url
            return {
                'status_code': 200,
                'final_url': target,
                'redirect_chain': [url] if target == url else [url, target],
                'headers': {'content-type': 'application/octet-stream'},
                'retrieved_at_utc': '2026-08-24T00:00:00Z',
            }

        return copy_fixture

    def acquire(
        self, *, selection=None, profile=None, candidate=None, observation=None, transport=None
    ):
        if selection is not None:
            self.selection_document = selection
        if profile is not None:
            self.profile_document = profile
        if selection is not None or profile is not None:
            self.write_inputs()
        return acquire_candidate(
            self.selection_path,
            self.profile_path,
            self.evidence,
            candidate or self.candidate,
            mount_observation=observation or self.observation(),
            transport=transport or self.transport(),
            now_utc='2026-08-24T00:00:00Z',
        )


@pytest.fixture
def fixture(tmp_path):
    return Fixture(tmp_path)


def _receipt_path(fixture: Fixture) -> Path:
    return fixture.candidate / 'candidate_receipt.json'


def _rewrite_receipt(fixture: Fixture, mutate) -> None:
    os.chmod(fixture.candidate, 0o755)
    path = _receipt_path(fixture)
    os.chmod(path, 0o644)
    os.chmod(path.with_name(path.name + '.sha256'), 0o644)
    document = json.loads(path.read_text(encoding='utf-8'))
    mutate(document)
    payload = (json.dumps(document, indent=2, sort_keys=True) + '\n').encode()
    path.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    path.with_name(path.name + '.sha256').write_text(f'{digest}  {path.name}\n', encoding='ascii')


def test_moving_redirect_and_final_url_are_recorded_and_allowlisted(fixture):
    result = fixture.acquire(
        transport=fixture.transport(final_url='https://mirror.fixture.example/moved')
    )
    assert result['status'] == 'PASS'
    receipt = json.loads(_receipt_path(fixture).read_text(encoding='utf-8'))
    assert len(receipt['artifacts']) == 8
    assert all(
        item['final_url'].startswith('https://mirror.fixture.example/')
        for item in receipt['artifacts']
    )
    assert all(item['redirect_chain'][0] == item['requested_url'] for item in receipt['artifacts'])
    assert all(item['retrieved_at_utc'] == '2026-08-24T00:00:00Z' for item in receipt['artifacts'])


def test_redirect_leaving_allowlist_seals_failure_and_cannot_retry(fixture):
    result = fixture.acquire(transport=fixture.transport(final_url='https://evil.example/moved'))
    assert result['status'] == 'FAIL_CLOSED'
    receipt = json.loads(_receipt_path(fixture).read_text(encoding='utf-8'))
    assert receipt['failure']['code'] == 'REDIRECT_POLICY'
    with pytest.raises(NtuAcquisitionError, match='already exists'):
        fixture.acquire()


@pytest.mark.parametrize(
    'payload, expected',
    [
        (_zip_bytes('../escape', b'bad'), 'not a safe relative path'),
        (_duplicate_zip_bytes(), 'duplicate zip member'),
    ],
)
def test_archive_traversal_or_duplicate_member_is_rejected(tmp_path, payload, expected):
    path = tmp_path / 'malicious.zip'
    path.write_bytes(payload)
    with pytest.raises(NtuAcquisitionError, match=expected):
        inspect_safe_archive(path, ['safe'])


def test_archive_symlink_and_hardlink_members_are_rejected(tmp_path):
    symlink_bytes = io.BytesIO()
    with tarfile.open(fileobj=symlink_bytes, mode='w') as archive:
        item = tarfile.TarInfo('link')
        item.type = tarfile.SYMTYPE
        item.linkname = 'target'
        archive.addfile(item)
    symlink_path = tmp_path / 'symlink.tar'
    symlink_path.write_bytes(symlink_bytes.getvalue())
    with pytest.raises(NtuAcquisitionError, match='unsafe tar member'):
        inspect_safe_archive(symlink_path, [])

    hardlink_bytes = io.BytesIO()
    with tarfile.open(fileobj=hardlink_bytes, mode='w') as archive:
        item = tarfile.TarInfo('hardlink')
        item.type = tarfile.LNKTYPE
        item.linkname = 'target'
        archive.addfile(item)
    hardlink_path = tmp_path / 'hardlink.tar'
    hardlink_path.write_bytes(hardlink_bytes.getvalue())
    with pytest.raises(NtuAcquisitionError, match='unsafe tar member'):
        inspect_safe_archive(hardlink_path, [])


@pytest.mark.parametrize(
    'changes',
    [
        {'uuid': '00000000-0000-0000-0000-000000000000'},
        {'fstype': 'xfs'},
        {'label': 'wrong'},
        {'options': 'ro'},
    ],
)
def test_mount_identity_mismatch_is_rejected_before_reservation(fixture, changes):
    with pytest.raises(NtuAcquisitionError):
        fixture.acquire(observation=fixture.observation(**changes))
    assert not fixture.candidate.exists()


def test_nonempty_and_reused_candidate_root_are_never_overwritten(fixture):
    fixture.candidate.mkdir(parents=True)
    sentinel = fixture.candidate / 'sentinel'
    sentinel.write_text('preserve', encoding='utf-8')
    with pytest.raises(NtuAcquisitionError, match='already exists'):
        fixture.acquire()
    assert sentinel.read_text(encoding='utf-8') == 'preserve'

    sentinel.unlink()
    fixture.candidate.rmdir()
    assert fixture.acquire()['status'] == 'PASS'
    with pytest.raises(NtuAcquisitionError, match='already exists'):
        fixture.acquire()


def test_partial_failure_is_sealed_and_partial_file_cannot_be_reused(fixture):
    result = fixture.acquire(transport=fixture.transport(fail_after=1))
    assert result['status'] == 'FAIL_CLOSED'
    assert list(fixture.candidate.rglob('*.part'))
    receipt = json.loads(_receipt_path(fixture).read_text(encoding='utf-8'))
    assert receipt['status'] == 'FAIL_CLOSED'
    with pytest.raises(NtuAcquisitionError, match='already exists'):
        fixture.acquire()


def test_missing_sequence_or_calibration_role_fails_before_reservation(fixture):
    missing_sequence = copy.deepcopy(fixture.selection_document)
    missing_sequence['selected_sequences'].pop()
    with pytest.raises(NtuAcquisitionError, match='exactly eee_01'):
        fixture.acquire(selection=missing_sequence)
    assert not fixture.candidate.exists()

    missing_calibration = copy.deepcopy(_selection(fixture.evidence))
    missing_calibration['acquisition']['calibration_roles'].pop('calib_stereo_imu')
    with pytest.raises(NtuAcquisitionError, match='calib_stereo_imu'):
        fixture.acquire(selection=missing_calibration)
    assert not fixture.candidate.exists()


def test_candidate_role_name_and_sidecar_mutation_are_rejected(fixture):
    assert fixture.acquire()['status'] == 'PASS'
    _rewrite_receipt(
        fixture,
        lambda receipt: receipt['artifacts'][0].update(
            {'relative_path': 'sequences/eee_01/wrong-role/eee_01.zip'}
        ),
    )
    with pytest.raises(NtuAcquisitionError, match='path does not match'):
        propose_pin_manifest(
            fixture.selection_path,
            fixture.profile_path,
            fixture.candidate,
            fixture.root / 'proposal.json',
            evidence_root=fixture.evidence,
        )

    isolated = Fixture(fixture.root / 'sidecar')
    assert isolated.acquire()['status'] == 'PASS'
    path = _receipt_path(isolated)
    os.chmod(isolated.candidate, 0o755)
    os.chmod(path, 0o644)
    path.write_bytes(path.read_bytes() + b'tamper')
    with pytest.raises(NtuAcquisitionError, match='sidecar'):
        propose_pin_manifest(
            isolated.selection_path,
            isolated.profile_path,
            isolated.candidate,
            isolated.root / 'proposal.json',
            evidence_root=isolated.evidence,
        )


def test_candidate_artifact_mutation_and_selection_profile_drift_fail_closed(fixture):
    assert fixture.acquire()['status'] == 'PASS'
    input_path = fixture.candidate / 'sequences/eee_01/sequence_archive/eee_01.zip'
    input_path.write_bytes(input_path.read_bytes() + b'tamper')
    with pytest.raises(NtuAcquisitionError, match='size|SHA-256'):
        propose_pin_manifest(
            fixture.selection_path,
            fixture.profile_path,
            fixture.candidate,
            fixture.root / 'proposal.json',
            evidence_root=fixture.evidence,
        )

    isolated = Fixture(fixture.root / 'drift')
    assert isolated.acquire()['status'] == 'PASS'
    drifted_selection = copy.deepcopy(isolated.selection_document)
    drifted_selection['selection_basis'] = 'changed after acquisition'
    isolated.selection_path.write_text(
        yaml.safe_dump(drifted_selection, sort_keys=False), encoding='utf-8'
    )
    with pytest.raises(NtuAcquisitionError, match='selection identity'):
        propose_pin_manifest(
            isolated.selection_path,
            isolated.profile_path,
            isolated.candidate,
            isolated.root / 'proposal.json',
            evidence_root=isolated.evidence,
        )

    isolated.selection_path.write_text(
        yaml.safe_dump(isolated.selection_document, sort_keys=False), encoding='utf-8'
    )
    drifted_profile = copy.deepcopy(isolated.profile_document)
    drifted_profile['competitive_slam_profile']['datasets']['ntu_viral_historical']['eee_01'][
        'status'
    ] = 'changed-after-acquisition'
    isolated.profile_path.write_text(
        yaml.safe_dump(drifted_profile, sort_keys=False), encoding='utf-8'
    )
    with pytest.raises(NtuAcquisitionError, match='profile identity'):
        propose_pin_manifest(
            isolated.selection_path,
            isolated.profile_path,
            isolated.candidate,
            isolated.root / 'profile-drift.json',
            evidence_root=isolated.evidence,
        )


def test_gt_path_leakage_is_rejected_and_proposal_is_deterministic(fixture):
    assert fixture.acquire()['status'] == 'PASS'
    _rewrite_receipt(
        fixture,
        lambda receipt: receipt['runner_input_manifest'].update(
            {'ground_truth_relative_paths': ['sequences/eee_01/ground_truth_csv/eee_01.csv']}
        ),
    )
    with pytest.raises(NtuAcquisitionError, match='ground-truth paths'):
        propose_pin_manifest(
            fixture.selection_path,
            fixture.profile_path,
            fixture.candidate,
            fixture.root / 'leak.json',
            evidence_root=fixture.evidence,
        )

    isolated = Fixture(fixture.root / 'deterministic')
    assert isolated.acquire()['status'] == 'PASS'
    profile_before = isolated.profile_path.read_bytes()
    first = isolated.root / 'proposal-1.json'
    second = isolated.root / 'proposal-2.json'
    propose_pin_manifest(
        isolated.selection_path,
        isolated.profile_path,
        isolated.candidate,
        first,
        evidence_root=isolated.evidence,
    )
    propose_pin_manifest(
        isolated.selection_path,
        isolated.profile_path,
        isolated.candidate,
        second,
        evidence_root=isolated.evidence,
    )
    assert first.read_bytes() == second.read_bytes()
    assert isolated.profile_path.read_bytes() == profile_before
    manifest = json.loads(first.read_text(encoding='utf-8'))
    assert manifest['status'] == 'PROPOSED_REVIEW_REQUIRED'
    assert manifest['claim_eligible'] is False
    assert manifest['runner_gt_paths_exposed'] is False
    assert manifest['runner_input_manifest']['ground_truth_relative_paths'] == []


def test_candidate_or_proposal_cannot_satisfy_preflight_without_separate_review(tmp_path):
    errors, not_ready = [], []
    candidate_policy = {
        'reviewed_pin_required': True,
        'candidate_receipt_path': 'candidate_receipt.json',
    }
    assert (
        closure._check_ntu_viral_reviewed_pin(
            candidate_policy, tmp_path, {}, {}, not_ready, errors
        )
        is False
    )
    assert any('candidate receipt' in item for item in errors)

    errors, not_ready = [], []
    proposed_policy = {
        'reviewed_pin_required': True,
        'reviewed_pin_manifest_status': 'NOT_CONFIGURED',
        'reviewed_pin_manifest_path': None,
        'reviewed_pin_manifest_sha256': None,
    }
    assert (
        closure._check_ntu_viral_reviewed_pin(proposed_policy, tmp_path, {}, {}, not_ready, errors)
        is False
    )
    assert any('not REVIEWED' in item for item in not_ready)
