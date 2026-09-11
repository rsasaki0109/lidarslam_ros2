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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the non-symlinked package upgrade contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'check_install_upgrade.py'
SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'install-upgrade-v1.schema.json'
WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'install-upgrade.yml'
EVIDENCE = (
    REPO_ROOT / 'docs' / 'evidence' / 'install-upgrade-2026-07-28.md'
)
RELEASE_BUNDLE_SCRIPT = REPO_ROOT / 'scripts' / 'build_release_bundle.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'check_install_upgrade',
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_prefix(prefix: Path) -> None:
    command = prefix / 'bin' / 'lidarslam-map'
    command.parent.mkdir(parents=True)
    command.write_text(
        f'#!/bin/bash\nsource {prefix}/share/lidarslam/setup.sh\n',
        encoding='utf-8',
    )
    command.chmod(0o755)
    node = prefix / 'lib' / 'lidarslam' / 'lidarslam'
    node.parent.mkdir(parents=True)
    node.write_bytes(b'\x7fELF\0fixture')
    node.chmod(0o755)
    resource = prefix / 'share' / 'lidarslam' / 'product' / 'VERSION'
    resource.parent.mkdir(parents=True)
    resource.write_text('0.6.0\n', encoding='utf-8')
    index = (
        prefix
        / 'share'
        / 'ament_index'
        / 'resource_index'
        / 'packages'
        / 'lidarslam'
    )
    index.parent.mkdir(parents=True)
    index.write_text('', encoding='utf-8')


def test_snapshot_normalizes_prefix_paths_and_ignores_binary_content(
    tmp_path: Path,
):
    module = _load_module()
    upgraded = tmp_path / 'upgraded'
    fresh = tmp_path / 'fresh'
    _write_prefix(upgraded)
    _write_prefix(fresh)
    (fresh / 'lib' / 'lidarslam' / 'lidarslam').write_bytes(
        b'\x7fELF\0different-build-id'
    )

    upgraded_snapshot = module.snapshot_prefix(upgraded)
    fresh_snapshot = module.snapshot_prefix(fresh)
    comparison = module.compare_snapshots(
        upgraded_snapshot,
        fresh_snapshot,
    )

    assert comparison == {
        'stale_paths': [],
        'missing_paths': [],
        'mismatched_paths': [],
    }
    assert (
        upgraded_snapshot['bin/lidarslam-map']['normalized_text_sha256']
        == fresh_snapshot['bin/lidarslam-map']['normalized_text_sha256']
    )
    assert (
        upgraded_snapshot['lib/lidarslam/lidarslam'][
            'normalized_text_sha256'
        ]
        is None
    )


def test_git_commands_scope_safe_directory_to_repository():
    module = _load_module()

    assert module._git_command('status', '--porcelain') == [
        'git',
        '-c',
        f'safe.directory={REPO_ROOT}',
        'status',
        '--porcelain',
    ]


def test_candidate_build_forwards_explicit_source_provenance(
    tmp_path: Path,
    monkeypatch,
):
    module = _load_module()
    observed = {}

    def fake_run_logged(command, log_path, cwd):
        observed['command'] = command
        observed['log_path'] = log_path
        observed['cwd'] = cwd
        return {'exit_code': 0}

    monkeypatch.setattr(module, '_run_logged', fake_run_logged)
    revision = 'a' * 40
    result = module._build(
        tmp_path / 'source',
        tmp_path / 'build',
        tmp_path / 'install',
        tmp_path / 'log',
        tmp_path / 'build.log',
        'Release',
        revision,
        False,
    )

    assert result == {'exit_code': 0}
    assert (
        f'-DLIDARSLAM_SOURCE_REVISION:STRING={revision}'
        in observed['command']
    )
    assert (
        '-DLIDARSLAM_SOURCE_DIRTY:STRING=false'
        in observed['command']
    )


def test_compare_reports_stale_missing_and_mode_drift(tmp_path: Path):
    module = _load_module()
    upgraded = tmp_path / 'upgraded'
    fresh = tmp_path / 'fresh'
    _write_prefix(upgraded)
    _write_prefix(fresh)
    stale = upgraded / 'share' / 'lidarslam' / 'stale.launch.py'
    stale.write_text('stale\n', encoding='utf-8')
    missing = fresh / 'share' / 'lidarslam' / 'new.yaml'
    missing.write_text('new\n', encoding='utf-8')
    (upgraded / 'bin' / 'lidarslam-map').chmod(0o644)

    comparison = module.compare_snapshots(
        module.snapshot_prefix(upgraded),
        module.snapshot_prefix(fresh),
    )

    assert comparison['stale_paths'] == [
        'share/lidarslam/stale.launch.py'
    ]
    assert comparison['missing_paths'] == ['share/lidarslam/new.yaml']
    assert comparison['mismatched_paths'] == [{
        'path': 'bin/lidarslam-map',
        'fields': ['executable'],
    }]


def test_report_schema_is_valid():
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))

    jsonschema.Draft7Validator.check_schema(schema)


def test_named_humble_and_jazzy_reports_pass_the_schema():
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))

    for distro in ('humble', 'jazzy'):
        report = json.loads(
            (
                REPO_ROOT
                / 'docs'
                / 'evidence'
                / f'install-upgrade-2026-07-28-{distro}.json'
            ).read_text(encoding='utf-8')
        )
        jsonschema.validate(report, schema)
        assert report['status'] == 'passed'
        assert report['ros_distro'] == distro
        assert all(check['passed'] for check in report['checks'])
        assert report['snapshots']['stale_paths'] == []
        assert report['snapshots']['missing_paths'] == []
        assert report['snapshots']['mismatched_paths'] == []


def test_workflow_and_distribution_docs_enforce_upgrade_gate():
    workflow = WORKFLOW.read_text(encoding='utf-8')
    distribution = (
        REPO_ROOT / 'docs' / 'distribution.md'
    ).read_text(encoding='utf-8')
    evidence = EVIDENCE.read_text(encoding='utf-8')
    release = (
        REPO_ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')
    release_bundle = RELEASE_BUNDLE_SCRIPT.read_text(encoding='utf-8')

    assert 'fetch-depth: 0' in workflow
    assert 'check_install_upgrade.py' in workflow
    assert 'install-upgrade-v1.schema.json' in workflow
    assert 'ros_distro: humble' in workflow
    assert 'ros_distro: jazzy' in workflow
    assert 'sha256:9db1a467c99d' in workflow
    assert 'sha256:7b27bdc109c2' in workflow
    assert 'python3 scripts/check_install_upgrade.py' in distribution
    assert '11 / 11 PASS' in evidence
    for filename in (
        'install-upgrade-2026-07-28.md',
        'install-upgrade-2026-07-28-humble.json',
        'install-upgrade-2026-07-28-jazzy.json',
    ):
        assert (REPO_ROOT / 'docs' / 'evidence' / filename).is_file()
    assert 'docs/evidence' in release_bundle
    assert 'scripts/build_release_bundle.py' in release
