#!/usr/bin/env python3

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


"""Focused tests for the read-only current-source distro matrix audit."""

import copy
import hashlib
import importlib.util
import os
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'audit_registration_plugin_matrix.py'
PROFILE_PATH = (
    REPO_ROOT
    / 'configs'
    / 'slam_benchmark_profiles'
    / 'registration_plugin_matrix_current_2026-08.json'
)


def _module():
    spec = importlib.util.spec_from_file_location('registration_matrix_audit', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def audit_module():
    return _module()


@pytest.fixture(scope='module')
def profile(audit_module):
    loaded, path = audit_module._load_profile(PROFILE_PATH)
    assert path == PROFILE_PATH
    return loaded


def test_current_source_manifest_and_consumer_contract(audit_module, profile):
    source = audit_module._source_manifest(REPO_ROOT, profile)
    assert source['status'] == 'PASS'
    assert source['files']
    assert all(entry['status'] == 'PASS' for entry in source['files'])
    contract = audit_module._consumer_static_contract(REPO_ROOT, profile)
    assert contract['status'] == 'PASS'
    assert contract['declared_cxx_standard'] == 14


def test_capture_contract_gpgv_policy_is_exact_and_required(audit_module, profile):
    contract = copy.deepcopy(profile['release_matrix']['dependency_closure']['capture_contract'])
    contract['gpgv_status_policy'] = copy.deepcopy(audit_module.GPGV_STATUS_POLICY)
    assert audit_module._validate_capture_contract(None, contract)['status'] == 'PASS'

    missing = copy.deepcopy(contract)
    missing.pop('gpgv_status_policy')
    with pytest.raises(audit_module.AuditError, match='incomplete or extra'):
        audit_module._validate_capture_contract(None, missing)

    drifted = copy.deepcopy(contract)
    drifted['gpgv_status_policy']['proof'] = 'status_tags_only'
    with pytest.raises(audit_module.AuditError, match='status/policy'):
        audit_module._validate_capture_contract(None, drifted)

    extra = copy.deepcopy(contract)
    extra['gpgv_status_policy_extra'] = True
    with pytest.raises(audit_module.AuditError, match='incomplete or extra'):
        audit_module._validate_capture_contract(None, extra)


def test_jazzy_host_matrix_and_missing_humble_are_distinct(audit_module, profile):
    jazzy = audit_module.audit(REPO_ROOT, profile, ['jazzy'])
    assert jazzy['status'] == 'PASS_HOST_TOOLCHAIN_ONLY'
    row = jazzy['distros'][0]
    assert row['status'] == 'PASS_HOST_TOOLCHAIN_ONLY'
    assert row['image']['status'] == 'NOT_PROBED'
    assert row['execution']['container'] == 'not_started'
    assert row['execution']['repository_mount']['required_for_container_gate'] == 'read_only'
    assert row['execution']['repository_mount']['status'] == 'NOT_APPLICABLE_HOST_ONLY'
    assert row['toolchain']['colcon']['status'] == 'PASS'
    assert row['install']['tree_hash_kind'] == 'configured_probe_manifest_v1'
    if row['install']['status'] != 'NOT_PRESENT':
        assert len(row['install']['tree_sha256']) == 64

    humble = audit_module.audit(REPO_ROOT, profile, ['humble'])
    assert humble['status'] == 'NO_GO:humble'
    assert humble['distros'][0]['reason'] == 'ROS_SETUP_MISSING_OR_SYMLINK'
    assert {item['name'] for item in humble['distros'][0]['optional_dependencies']} == {
        'fast_gicp',
        'small_gicp',
    }
    assert all(
        item['status'] == 'NOT_PROBED' for item in humble['distros'][0]['optional_dependencies']
    )


def test_historical_claims_are_always_superseded(audit_module, profile):
    report = audit_module.audit(REPO_ROOT, profile, ['jazzy'])
    assert report['historical_passes_promoted'] is False
    assert report['superseded_historical_count'] == len(report['historical_evidence'])
    assert report['historical_evidence']
    assert all(item['status'].startswith('SUPERSEDED_') for item in report['historical_evidence'])
    assert report['historical_evidence'][0]['status'] == 'SUPERSEDED_UNBOUND_DOCUMENT_CLAIM'


def test_historical_mutable_receipt_is_not_promoted(audit_module, tmp_path):
    receipt = tmp_path / 'historical.receipt.json'
    payload = b'{"status":"PASS"}\n'
    receipt.write_bytes(payload)
    sidecar = Path(str(receipt) + '.sha256')
    sidecar.write_text(
        '{}  {}\n'.format(hashlib.sha256(payload).hexdigest(), receipt.name),
        encoding='ascii',
    )
    receipt.chmod(0o644)
    sidecar.chmod(0o444)
    synthetic = {
        'historical': [
            {
                'id': 'mutable',
                'claim': 'fixture',
                'source_document': 'scripts/audit_registration_plugin_matrix.py',
                'receipt_paths': [str(receipt)],
            }
        ]
    }
    result = audit_module._historical_audit(REPO_ROOT, synthetic)[0]
    assert result['status'] == 'SUPERSEDED_ARTIFACT_OR_SIDECAR_INVALID'
    assert result['receipts'][0]['status'] == 'MUTABLE_MODE'
    assert result['receipts'][0]['sidecar_status'] == 'PASS'


def test_source_hash_drift_is_fail_closed(audit_module, profile):
    drifted = copy.deepcopy(profile)
    drifted['source']['files'][0]['sha256'] = '0' * 64
    drifted['distros'] = []
    report = audit_module.audit(REPO_ROOT, drifted)
    assert report['status'] == 'FAIL_CLOSED'
    assert report['current_source']['status'] == 'FAIL_CLOSED'
    assert report['current_source']['mismatches'] == [drifted['source']['files'][0]['path']]


def test_v2_source_snapshot_is_explicitly_unconfigured(audit_module, profile):
    result = audit_module._v2_source_snapshot_audit(REPO_ROOT, profile)
    assert result == {
        'status': 'NOT_CONFIGURED',
        'checked': False,
        'promotion': 'NOT_AUTHORIZED',
    }


def test_v2_source_snapshot_missing_manifest_fails_closed(audit_module, profile, tmp_path):
    snapshot_root = tmp_path / 'v2-snapshot'
    snapshot_root.mkdir()
    configured = copy.deepcopy(profile)
    configured['source']['v2_snapshot'] = {
        'root': str(snapshot_root),
        'manifest': 'source-manifest.json',
        'repositories': [{}],
    }
    result = audit_module._v2_source_snapshot_audit(REPO_ROOT, configured)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['checked'] is True
    assert result['promotion'] == 'NOT_AUTHORIZED'
    assert result['manifest'] == 'source-manifest.json'


def test_v2_source_snapshot_configuration_rejects_extra_or_relative_paths(
        audit_module, profile, tmp_path):
    configured = copy.deepcopy(profile)
    configured['source']['v2_snapshot'] = {
        'root': str(tmp_path),
        'manifest': 'source-manifest.json',
        'repositories': [],
        'extra': 'reject',
    }
    with pytest.raises(audit_module.AuditError, match='incomplete or extra'):
        audit_module._v2_source_snapshot_audit(REPO_ROOT, configured)

    configured['source']['v2_snapshot'].pop('extra')
    configured['source']['v2_snapshot']['manifest'] = '../source-manifest.json'
    with pytest.raises(audit_module.AuditError, match='canonical'):
        audit_module._v2_source_snapshot_audit(REPO_ROOT, configured)


def test_no_docker_is_used_by_default(audit_module, profile, monkeypatch):
    original_run = audit_module.subprocess.run

    def guarded_run(argv, *args, **kwargs):
        assert not argv or argv[0] != 'docker'
        return original_run(argv, *args, **kwargs)

    monkeypatch.setattr(audit_module.subprocess, 'run', guarded_run)
    report = audit_module.audit(REPO_ROOT, profile, ['jazzy'])
    assert report['safety']['docker_run'] is False
    assert report['safety']['docker_build'] is False
    assert report['safety']['network_used'] is False


def test_optional_dependency_matrix_is_explicit(audit_module, profile):
    report = audit_module.audit(REPO_ROOT, profile, ['jazzy'])
    dependencies = report['distros'][0]['optional_dependencies']
    assert {item['name'] for item in dependencies} == {'fast_gicp', 'small_gicp'}
    assert all(item['status'] in {'PRESENT', 'ABSENT'} for item in dependencies)
    assert all(item['required'] is False for item in dependencies)


def test_seal_receipt_is_immutable_and_sidecar_matches(audit_module, tmp_path):
    output = tmp_path / 'matrix.receipt.json'
    report = {'schema': 'test', 'status': 'PASS', 'safety': {'network_used': False}}
    receipt, sidecar, digest = audit_module.seal_receipt(output, report)
    receipt_path = Path(receipt)
    sidecar_path = Path(sidecar)
    payload = receipt_path.read_bytes()
    assert digest == hashlib.sha256(payload).hexdigest()
    assert sidecar_path.read_text(encoding='ascii') == '{}  {}\n'.format(digest, receipt_path.name)
    assert os.stat(receipt_path).st_mode & 0o777 == 0o444
    assert os.stat(sidecar_path).st_mode & 0o777 == 0o444
    with pytest.raises(audit_module.AuditError):
        audit_module.seal_receipt(output, report)

    symlink_output = tmp_path / 'symlink.receipt.json'
    symlink_output.symlink_to(receipt_path)
    with pytest.raises(audit_module.AuditError):
        audit_module.seal_receipt(symlink_output, report)


def test_profile_images_are_digest_pinned_and_no_run_policy(profile):
    assert profile['policy']['network'] == 'none'
    assert profile['policy']['docker_build'] is False
    assert profile['policy']['docker_pull'] is False
    assert profile['policy']['docker_run'] is False
    assert profile['policy']['formal_replay'] is False
    for distro in profile['distros']:
        assert distro['image']['digest'].startswith('sha256:')
        assert len(distro['image']['digest']) == len('sha256:') + 64


def test_profile_and_workflow_bind_official_digest_and_humble_child(audit_module, profile):
    workflow = audit_module._validate_workflow_image_contract(REPO_ROOT, profile)
    assert workflow['status'] == 'PASS'
    assert workflow['references'] == sorted(
        {
            'docker.io/library/ros:humble-ros-core@sha256:'
            'ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988',
            'docker.io/library/ros:jazzy-ros-base@sha256:'
            '31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f',
        }
    )
    humble = next(row for row in profile['distros'] if row['name'] == 'humble')
    image = humble['image']
    assert image['reference'] == image['tag'] + '@' + image['digest']
    assert image['manifest'] == {
        'index_digest': 'sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988',
        'index_media_type': 'application/vnd.oci.image.index.v1+json',
        'linux_amd64_digest':
            'sha256:32bf718e63618482ffb1fe232cf0f834635c57162e2506fb0bc0b092ef776c1e',
        'linux_amd64_media_type': 'application/vnd.oci.image.manifest.v1+json',
        'linux_amd64_os': 'linux',
        'linux_amd64_architecture': 'amd64',
    }


def test_image_contract_rejects_unpinned_or_non_official_images(audit_module, profile):
    image = copy.deepcopy(
        next(row for row in profile['distros'] if row['name'] == 'humble')['image']
    )
    image['reference'] = image['tag']
    with pytest.raises(audit_module.AuditError, match='reference'):
        audit_module._validate_image_contract(image, 'humble', require_humble_manifest=True)
    image = copy.deepcopy(
        next(row for row in profile['distros'] if row['name'] == 'humble')['image']
    )
    image['tag'] = 'quay.io/ros:humble-ros-core'
    with pytest.raises(audit_module.AuditError, match='official Docker Hub'):
        audit_module._validate_image_contract(image, 'humble', require_humble_manifest=True)


def test_workflow_image_contract_rejects_unpinned_literal(audit_module, profile, tmp_path):
    workflow = tmp_path / '.github' / 'workflows' / 'main.yml'
    workflow.parent.mkdir(parents=True)
    workflow.write_text(
        (REPO_ROOT / '.github' / 'workflows' / 'main.yml')
        .read_text(encoding='utf-8')
        .replace(
            'docker.io/library/ros:humble-ros-core@sha256:'
            'ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988',
            'ros:humble-ros-core',
            1,
        ),
        encoding='utf-8',
    )
    with pytest.raises(audit_module.AuditError, match='unpinned'):
        audit_module._validate_workflow_image_contract(tmp_path, profile)
