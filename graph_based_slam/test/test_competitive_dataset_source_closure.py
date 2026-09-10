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

"""Adversarial tests for the metadata-only dataset source closure."""

import copy
import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_competitive_dataset_source_closure.py'
SPEC = importlib.util.spec_from_file_location('dataset_source_closure', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _artifact(prefix: str, *, status='REVALIDATED'):
    return {
        'artifact_id': prefix + '-artifact',
        'sha256': prefix[0] * 64,
        'size_bytes': 100,
        'hash_kind': 'synthetic_bytes_sha256_v1',
        'validation_status': status,
        'revalidation_receipt_sha256': 'e' * 64,
        'revalidation_mount_identity': 'synthetic-mount',
    }


def _descriptor(dataset_id, sequence_id, family_id, *, status='REVALIDATED'):
    input_prefix, calibration_prefix, gt_prefix = (
        ('d', 'e', 'f') if sequence_id.startswith('fresh') else ('a', 'b', 'c'))
    return {
        'profile_partition': 'fresh' if sequence_id.startswith('fresh') else 'historical',
        'profile_key': sequence_id,
        'dataset_id': dataset_id,
        'sequence_id': sequence_id,
        'family_id': family_id,
        'source': {
            'project_name': 'Synthetic official dataset',
            'publisher': 'Synthetic primary custodian',
            'project_url': 'https://datasets.example.invalid/project',
            'official_primary': True,
            'immutable_version': 'a' * 40,
            'download_refs': {
                role: {
                    'url': f'https://datasets.example.invalid/{role}/' + 'a' * 40,
                    'immutable': True,
                    'primary': True,
                }
                for role in ('input', 'ground_truth', 'calibration')
            },
        },
        'license_terms': {
            'status': 'READY',
            'license_id': 'CC-BY-4.0',
            'terms_url': 'https://datasets.example.invalid/terms',
            'citation': 'Synthetic dataset citation',
        },
        'sensor_contract': {
            'topics': ['/lidar', '/imu'],
            'formats': ['PointCloud2', 'Imu'],
            'timestamp_basis': 'unix_epoch_seconds',
        },
        'input': _artifact(input_prefix, status=status),
        'calibration': {
            **_artifact(calibration_prefix, status=status),
            'frame_convention': 'imu',
            'timestamp_basis': 'unix_epoch_seconds',
        },
        'ground_truth': {
            **_artifact(gt_prefix, status=status),
            'role': 'ground_truth',
            'method': 'official_dense_6dof',
            'frame': 'imu',
            'timestamp_basis': 'unix_epoch_seconds',
        },
        'sequence': {
            'duration_seconds': 10.0,
            'message_counts': {'lidar': 10, 'imu': 20},
        },
        'supported_systems': ['ours', 'glim', 'fast_livo2'],
    }


def _profile():
    contract = {
        'evidence_gate_v2': {
            'dataset_source_closure': {
                'schema_version': 1,
                'required': True,
                'status': 'READY',
                'minimum_gt_dataset_families': 2,
                'require_fresh_holdout': True,
                'require_historical_partition': True,
                'required_partitions': ['historical', 'fresh'],
                'required_systems': ['ours', 'glim', 'fast_livo2'],
                'hash_policy': {
                    'recorded_status': 'RECORDED_ONLY',
                    'revalidated_status': 'REVALIDATED',
                    'revalidation_requires_mount_identity': True,
                    'ground_truth_content_opened': False,
                },
                'datasets': [
                    _descriptor('hist-alpha', 'hist01', 'family-alpha'),
                    _descriptor('fresh-beta', 'fresh01', 'family-beta'),
                ],
            },
        },
        'datasets': {
            'holdout_slots': {
                'hist01': {
                    'dataset': 'hist-alpha', 'sequence': 'hist01',
                    'input_manifest_sha256': '1' * 64,
                },
            },
            'fresh_holdout_slots': {
                'fresh01': {
                    'dataset': 'fresh-beta', 'sequence': 'fresh01',
                    'input_manifest_sha256': '4' * 64,
                },
            },
        },
    }
    return {'competitive_slam_profile': contract}


def test_complete_two_family_fresh_holdout_closure_passes_without_data_access():
    result = MODULE.verify_dataset_source_closure(_profile())
    assert result['status'] == 'PASS'
    assert result['pass'] is True
    assert result['checks']['gt_dataset_families']['families'] == [
        'family-alpha', 'family-beta']
    assert result['checks']['hashes_are_metadata_only']['gt_content_opened'] is False


def test_fresh_historical_overlap_by_immutable_hash_fails_closed():
    profile = _profile()
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][1]['input']['sha256'] = 'a' * 64
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][0]['input']['sha256'] = 'a' * 64
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'INVALID'
    assert any('immutable input identity overlaps' in item for item in result['errors'])


def test_missing_calibration_or_ground_truth_identity_is_not_ready():
    profile = _profile()
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['status'] = 'NOT_READY'
    descriptor = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][0]
    descriptor['calibration']['sha256'] = None
    descriptor['ground_truth']['artifact_id'] = None
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'NOT_READY'
    assert any('calibration.sha256 is missing' in item for item in result['not_ready'])
    assert any('ground_truth.artifact_id is missing' in item for item in result['not_ready'])


def test_frame_and_time_basis_mismatch_is_invalid():
    profile = _profile()
    descriptor = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][0]
    descriptor['ground_truth']['frame'] = 'world'
    descriptor['ground_truth']['timestamp_basis'] = 'sensor_ticks'
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'INVALID'
    assert any('ground truth timestamp basis differs' in item for item in result['errors'])
    assert any('ground truth frame differs' in item for item in result['errors'])


def test_unsupported_rival_is_invalid():
    profile = _profile()
    descriptor = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][1]
    descriptor['supported_systems'] = ['ours', 'glim']
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'INVALID'
    assert any('does not support every required runner' in item for item in result['errors'])


def test_recorded_only_hashes_remain_not_ready():
    profile = _profile()
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['status'] = 'NOT_READY'
    descriptor = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][0]
    descriptor['input']['validation_status'] = 'RECORDED_ONLY'
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'NOT_READY'
    assert any('input hash/bytes are recorded-only' in item for item in result['not_ready'])


def test_moving_source_and_missing_license_remain_not_ready():
    profile = _profile()
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['status'] = 'NOT_READY'
    descriptor = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']['datasets'][0]
    descriptor['source']['immutable_version'] = None
    descriptor['source']['download_refs']['input']['immutable'] = False
    descriptor['license_terms']['status'] = 'NOT_READY'
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'NOT_READY'
    assert any('immutable version is missing' in item for item in result['not_ready'])
    assert any('input URL is moving or unpinned' in item for item in result['not_ready'])
    assert any('license/terms are not READY' in item for item in result['not_ready'])


def test_profile_descriptor_overlap_and_missing_entry_are_not_silently_ignored():
    profile = _profile()
    policy = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']
    policy['status'] = 'NOT_READY'
    policy['datasets'] = copy.deepcopy(policy['datasets'][:1])
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'NOT_READY'
    assert any('dataset source descriptor missing' in item for item in result['not_ready'])


def test_excluded_training_family_does_not_count_as_pass_family():
    profile = _profile()
    policy = profile['competitive_slam_profile']['evidence_gate_v2'][
        'dataset_source_closure']
    policy['status'] = 'NOT_READY'
    policy['datasets'][0]['evaluation_eligible'] = False
    policy['datasets'][0]['fresh_eligible'] = False
    result = MODULE.verify_dataset_source_closure(profile)
    assert result['status'] == 'NOT_READY'
    assert result['checks']['gt_dataset_families']['families'] == ['family-beta']
    assert result['checks']['gt_dataset_families']['passed_families'] == ['family-beta']
    assert any('require at least 2' in item for item in result['not_ready'])


def _ntu_manifest(*, input_sha256=None, tnp_eval=False):
    byte_identity = {
        'input_sha256': input_sha256,
        'input_size_bytes': None,
        'ground_truth_sha256': None,
        'ground_truth_size_bytes': None,
        'calibration_sha256': None,
        'calibration_size_bytes': None,
        'validation_status': 'NOT_READY',
    }
    return {
        'selection_id': 'ntu-test',
        'status': 'NOT_READY',
        'family_id': 'ntu_viral',
        'partition': 'historical',
        'selected_sequences': [{
            'sequence_id': 'eee_01',
            'profile_key': 'ntu_viral_eee_01',
            'selection_rationale': 'synthetic',
            'environment': 'synthetic',
            'official_duration_seconds': 1.0,
            'expected_files': ['official_sequence_archive_zip'],
            'byte_identity': byte_identity,
            'supported_systems': ['ours', 'glim', 'fast_livo2'],
        }],
        'excluded_sequences': [{
            'sequence_id': 'tnp_01',
            'profile_key': 'ntu_viral_tnp_01',
            'reason': 'training exposed synthetic',
            'evaluation_eligible': tnp_eval,
            'fresh_eligible': False,
        }],
        'acquisition': {
            'recipe_id': 'recipe', 'recipe_revision': 'v1',
            'materialization': 'synthetic', 'status': 'NOT_READY',
            'required_before_execution': ['input_sha256'],
        },
        'official_sources': {
            'project_url': 'https://example.invalid/project',
            'dataset_repository_url': 'https://example.invalid/repo',
            'ground_truth_repository_url': 'https://example.invalid/gt',
            'evaluation_tutorial_url': 'https://example.invalid/tutorial',
            'citation': 'synthetic', 'terms_url': 'https://example.invalid/terms',
        },
        'disjointness': {
            'excluded_training_sequences': ['tnp_01'],
            'excluded_historical_families': ['hilti'],
            'required_immutable_ids': ['input_sha256'],
        },
    }


def test_ntu_selection_rejects_guessed_byte_identity(tmp_path):
    manifest_path = tmp_path / 'selection.yaml'
    manifest = _ntu_manifest(input_sha256='a' * 64)
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
    not_ready, errors = [], []
    policy_entries = {
        ('historical', 'ntu_viral_eee_01'): {
            'profile_key': 'ntu_viral_eee_01', 'sequence_id': 'eee_01',
            'family_id': 'ntu_viral', 'evaluation_eligible': True,
        },
    }
    valid = MODULE._check_ntu_viral_selection_manifest(
        {
            'selection_id': 'ntu-test',
            'selection_path': 'selection.yaml',
            'required_systems': ['ours', 'glim', 'fast_livo2'],
        }, tmp_path, {}, policy_entries, not_ready, errors)
    assert valid is False
    assert any('input_sha256 must remain null' in item for item in errors)


def test_ntu_selection_rejects_tnp_training_exposure_relabel(tmp_path):
    manifest_path = tmp_path / 'selection.yaml'
    manifest = _ntu_manifest(tnp_eval=True)
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
    not_ready, errors = [], []
    valid = MODULE._check_ntu_viral_selection_manifest(
        {
            'selection_id': 'ntu-test',
            'selection_path': 'selection.yaml',
            'required_systems': ['ours', 'glim', 'fast_livo2'],
        }, tmp_path, {}, {
            ('historical', 'ntu_viral_eee_01'): {
                'sequence_id': 'eee_01', 'profile_key': 'ntu_viral_eee_01',
                'family_id': 'ntu_viral', 'evaluation_eligible': True,
            },
        }, not_ready, errors)
    assert valid is False
    assert any('tnp_01 must be excluded' in item for item in errors)


def test_production_ntu_selection_is_bound_but_not_claim_eligible():
    profile_path = ROOT / 'configs' / 'slam_benchmark_profiles' / 'competitive_slam_v1.yaml'
    profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    result = MODULE.verify_dataset_source_closure(profile, root=ROOT)
    assert result['status'] == 'NOT_READY'
    assert result['errors'] == []
    assert result['checks']['gt_dataset_families']['passed_families'] == []
    assert result['per_dataset']['development/ntu_viral_tnp_01']['pass'] is False
    assert result['per_dataset']['historical/ntu_viral_eee_01']['sequence_id'] == 'eee_01'
    assert result['per_dataset']['historical/ntu_viral_nya_01']['sequence_id'] == 'nya_01'
    assert result['per_dataset']['historical/ntu_viral_spms_01']['sequence_id'] == 'spms_01'
    assert any('ntu_viral_selection_is_precommitted' in item for item in result['not_ready'])
