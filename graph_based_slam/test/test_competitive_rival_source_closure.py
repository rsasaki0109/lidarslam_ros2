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

"""Adversarial synthetic tests for the offline rival source-closure audit."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_competitive_rival_source_closure.py'
SPEC = importlib.util.spec_from_file_location('rival_source_closure', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _sha(path: Path) -> str:
    return MODULE.sha256_file(path)


def _source(name: str, owner: str, repo: str, revision: str) -> dict:
    return {
        'name': name,
        'status': 'READY',
        'repository_url': f'https://github.com/{owner}/{repo}.git',
        'revision': revision,
        'revision_kind': 'immutable_commit',
        'observed_ref': 'refs/heads/main',
        'archive_url': f'https://github.com/{owner}/{repo}/archive/{revision}.tar.gz',
        'archive_sha256': 'a' * 64,
        'source_tree_sha256': 'b' * 64,
        'source_tree_hash_kind': MODULE.TREE_HASH_KIND,
        'submodules': [],
        'license': {
            'status': 'READY',
            'files': [{'path': 'LICENSE', 'identity': 'MIT', 'sha256': 'c' * 64}],
            'components': [
                {
                    'name': name,
                    'status': 'READY',
                    'files': [{'path': 'LICENSE', 'identity': 'MIT', 'sha256': 'c' * 64}],
                    'declarations': [],
                }
            ],
        },
        'citation': {
            'primary_upstream': True,
            'commit_url': f'https://github.com/{owner}/{repo}/commit/{revision}',
            'compatibility_status': 'READY',
            'compatibility_note': 'synthetic fixture',
        },
    }


def _fixture(tmp_path: Path) -> tuple[dict, Path]:
    dockerfile = tmp_path / 'Dockerfile'
    build_script = tmp_path / 'build.sh'
    runner = tmp_path / 'runner.py'
    wrapper = tmp_path / 'wrapper.sh'
    config = tmp_path / 'config.yaml'
    dockerfile.write_text(
        'FROM docker.io/library/ros:jazzy-ros-base@sha256:' + 'd' * 64 + '\n',
        encoding='utf-8',
    )
    build_script.write_text('#!/bin/sh\n', encoding='utf-8')
    runner.write_text('# runner\n', encoding='utf-8')
    wrapper.write_text('#!/bin/sh\n', encoding='utf-8')
    config.write_text('config: fixture\n', encoding='utf-8')
    revision_glim = '1' * 40
    revision_fast = '2' * 40

    def recipe() -> dict:
        return {
            'dockerfile': {
                'path': 'Dockerfile',
                'hash_kind': 'file_sha256',
                'sha256': _sha(dockerfile),
            },
            'build_script': {
                'path': 'build.sh',
                'hash_kind': 'file_sha256',
                'sha256': _sha(build_script),
            },
            'runner': {'path': 'runner.py', 'hash_kind': 'file_sha256', 'sha256': _sha(runner)},
            'wrapper': {'path': 'wrapper.sh', 'hash_kind': 'file_sha256', 'sha256': _sha(wrapper)},
            'configs': [
                {'path': 'config.yaml', 'hash_kind': 'file_sha256', 'sha256': _sha(config)}
            ],
            'base_image': {
                'name': 'docker.io/library/ros:jazzy-ros-base',
                'digest': 'sha256:' + 'd' * 64,
            },
            'build_options': {'release': 'Release'},
        }

    closure = {
        'schema_version': 1,
        'required': True,
        'closure_id': 'synthetic-rival-source-closure-r2',
        'closure_revision': 2,
        'closure_identity_hash_kind': MODULE.CLOSURE_IDENTITY_HASH_KIND,
        'status': 'READY',
        'status_reason': [],
        'required_systems': ['ours', 'glim', 'fast_livo2'],
        'archive_hash_kind': 'downloaded_commit_archive_sha256_v1',
        'source_tree_hash_kind': MODULE.TREE_HASH_KIND,
        'revision_policy': 'exact_40_hex_commit_only_no_branch_latest_or_tag_only',
        'official_source_policy': 'github_primary_upstream_commit_and_archive_only',
        'legal_policy': {
            'source_fetch_mode': 'SOURCE_FETCH_ONLY_NO_REDISTRIBUTION',
            'redistribution_mode': 'PROHIBITED_UNPROVEN_LICENSE',
            'image_publication_status': 'BLOCKED_PENDING_LICENSE_PROVENANCE',
            'claim_eligibility': 'BLOCKED',
            'nonredistributable_sources': ['glim', 'fast_livo2'],
            'warning': 'synthetic fixture; no redistribution is authorized',
        },
        'rivals': {
            'glim': {
                'status': 'READY',
                'sources': [_source('glim', 'koide3', 'glim', revision_glim)],
                'local_patches': [
                    {
                        'order': 1,
                        'path': 'build.sh',
                        'sha256': _sha(build_script),
                        'applies_to': 'glim',
                    }
                ],
                'recipe': recipe(),
            },
            'fast_livo2': {
                'status': 'READY',
                'sources': [_source('fast_livo2', 'hku-mars', 'FAST-LIVO2', revision_fast)],
                'local_patches': [
                    {
                        'order': 1,
                        'path': 'wrapper.sh',
                        'sha256': _sha(wrapper),
                        'applies_to': 'fast_livo2',
                    }
                ],
                'recipe': recipe(),
            },
        },
    }
    selection = {
        'schema_version': 1,
        'selection_kind': 'competitive_execution_selection_revision',
        'selection_id': 'synthetic-execution-selection-r2',
        'status': 'CURRENT',
        'closure_id': closure['closure_id'],
        'closure_revision': closure['closure_revision'],
        'closure_identity_hash_kind': closure['closure_identity_hash_kind'],
        'closure_identity_sha256': MODULE.canonical_rival_source_closure_identity(closure),
        'current_recipe_bindings': {
            system: {
                **closure['rivals'][system]['recipe'],
                'patches': closure['rivals'][system]['local_patches'],
            }
            for system in ('glim', 'fast_livo2')
        },
        'supersedes': [
            {
                'path': 'historical-selection-r1.yaml',
                'sha256': 'e' * 64,
                'status': 'SUPERSEDED_RECIPE_REVISION',
                'claim_eligible': False,
                'reason': 'synthetic r2 fixture supersedes r1',
            }
        ],
    }
    selection_path = tmp_path / 'selection-r2.yaml'
    selection_path.write_text(yaml.safe_dump(selection, sort_keys=True), encoding='utf-8')
    closure['active_selection'] = {
        'selection_id': selection['selection_id'],
        'status': 'CURRENT',
        'path': selection_path.name,
        'sha256': _sha(selection_path),
        'supersedes': selection['supersedes'],
    }
    systems = {}
    for system in ('glim', 'fast_livo2'):
        systems[system] = {
            'container': {
                'recipe': {
                    'path': 'Dockerfile',
                    'sha256': _sha(dockerfile),
                    'build_entrypoint_path': 'build.sh',
                    'build_entrypoint_sha256': _sha(build_script),
                },
            },
            'runner': {'path': 'runner.py', 'sha256': _sha(runner)},
        }
    contract = {
        'rivals': {
            'glim': {
                'repository': 'https://github.com/koide3/glim.git',
                'revision': revision_glim,
                'revision_kind': 'immutable_commit',
            },
            'fast_livo2': {
                'repository': 'https://github.com/hku-mars/FAST-LIVO2.git',
                'revision': revision_fast,
                'revision_kind': 'immutable_commit',
            },
        },
        'tracks': {
            'glim_cpu': {'rival': 'glim'},
            'fast_cpu': {'rival': 'fast_livo2'},
        },
        'systems': systems,
        'evidence_gate_v2': {'rival_source_closure': closure},
    }
    return {'competitive_slam_profile': contract}, tmp_path


def test_complete_synthetic_closure_passes(tmp_path):
    profile, root = _fixture(tmp_path)
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['status'] == 'PASS'
    assert result['pass'] is True


def test_branch_or_tag_only_revision_is_rejected(tmp_path):
    profile, root = _fixture(tmp_path)
    profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure']['rivals'][
        'glim'
    ]['sources'][0]['revision_kind'] = 'branch_main'
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('revision_kind' in item for item in result['errors'])


def test_missing_archive_and_explicit_submodule_closure_fail_closed(tmp_path):
    profile, root = _fixture(tmp_path)
    source = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['glim']['sources'][0]
    source.pop('archive_sha256')
    source['submodules'] = None
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('archive_sha256' in item for item in result['incomplete'])
    assert any('submodules' in item for item in result['incomplete'])


def test_missing_license_identity_is_not_ready(tmp_path):
    profile, root = _fixture(tmp_path)
    source = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['glim']['sources'][0]
    source['license'] = {'status': 'READY', 'files': []}
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('license.files' in item for item in result['incomplete'])


def test_missing_component_license_artifact_is_not_ready(tmp_path):
    profile, root = _fixture(tmp_path)
    source = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['glim']['sources'][0]
    source['license']['components'][0] = {
        'name': 'glim',
        'status': 'NOT_READY_LEGAL_PROVENANCE',
        'files': [],
        'declarations': [],
        'not_ready_reasons': ['upstream_license_text_absent'],
    }
    source['license']['status'] = 'NOT_READY_LEGAL_PROVENANCE'
    source['license']['files'] = []
    source['license']['not_ready_reasons'] = ['upstream_license_text_absent']
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('upstream_license_text_absent' in item for item in result['not_ready'])


def test_license_metadata_declaration_does_not_replace_license_artifact(tmp_path):
    profile, root = _fixture(tmp_path)
    source = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['glim']['sources'][0]
    component = source['license']['components'][0]
    component['files'] = []
    component['status'] = 'NOT_READY_LEGAL_PROVENANCE'
    component['declarations'] = [
        {
            'path': 'package.xml',
            'identity': 'MIT',
            'sha256': 'd' * 64,
        }
    ]
    component['not_ready_reasons'] = ['license_file_absent']
    source['license']['status'] = 'NOT_READY_LEGAL_PROVENANCE'
    source['license']['files'] = []
    source['license']['not_ready_reasons'] = ['license_file_absent']
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('license_file_absent' in item for item in result['not_ready'])


def test_ordered_patch_hash_drift_is_invalid(tmp_path):
    profile, root = _fixture(tmp_path)
    patch = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['glim']['local_patches'][0]
    patch['sha256'] = '0' * 64
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('local_patches' in item for item in result['errors'])


def test_profile_recipe_runner_mismatch_is_invalid(tmp_path):
    profile, root = _fixture(tmp_path)
    profile['competitive_slam_profile']['systems']['glim']['runner']['sha256'] = '0' * 64
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('runner' in item for item in result['errors'])


def test_rival_omitted_from_all_tracks_is_invalid(tmp_path):
    profile, root = _fixture(tmp_path)
    del profile['competitive_slam_profile']['tracks']['fast_cpu']
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('omitted' in item for item in result['errors'])


def test_absolute_external_config_is_explicit_not_ready(tmp_path):
    profile, root = _fixture(tmp_path)
    recipe = profile['competitive_slam_profile']['evidence_gate_v2']['rival_source_closure'][
        'rivals'
    ]['fast_livo2']['recipe']
    recipe['configs'].append(
        {
            'path': '/opt/pinned/HILTI22.yaml',
            'hash_kind': 'external_container_file_sha256',
            'sha256': 'e' * 64,
            'status': 'NOT_READY',
            'not_ready_reasons': ['not_reopenable'],
        }
    )
    result = MODULE.verify_rival_source_closure(profile, root=root)
    assert result['pass'] is False
    assert any('not_reopenable' in item for item in result['not_ready'])
