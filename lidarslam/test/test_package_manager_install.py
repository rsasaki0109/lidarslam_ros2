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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Tests for ROS apt clean-install and in-place-upgrade evidence."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_package_manager_install.py'
SPEC = importlib.util.spec_from_file_location(
    'package_manager_install',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _state(product_version='0.9.0', rko_version='0.3.2'):
    versions = {
        'lidarslam': product_version,
        'lidarslam-msgs': product_version,
        'scanmatcher': product_version,
        'graph-based-slam': product_version,
        'ndt-omp-ros2': '0.1.0',
        'rko-lio': rko_version,
    }
    return {
        name: {
            'debian_name': f'ros-jazzy-{name}',
            'debian_version': f'{version}-1noble.20260729.000000',
            'upstream_version': version,
            'owned_paths': [f'/opt/ros/jazzy/share/{name}/package.xml'],
        }
        for name, version in versions.items()
    }


def _evaluate(**kwargs):
    values = {
        'ros_distro': 'jazzy',
        'expected_version': '0.9.0',
        'mode': 'clean-install',
        'channel': 'main',
        'package_state': _state(),
        'cli_result': (True, 'installed CLI passed'),
        'source_state': {
            'git_commit': 'a' * 40,
            'git_dirty': False,
        },
    }
    values.update(kwargs)
    return CHECKER.evaluate_install(**values)


def test_clean_install_passes_exact_product_and_dependency_contract():
    report = _evaluate()

    assert report['status'] == 'PASS'
    assert report['snapshot']['package_count'] == 6
    assert report['snapshot']['owned_path_count'] == 6
    assert report['baseline'] is None
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_product_version_dependency_and_cli_failures_are_reported():
    state = _state(rko_version='0.3.1')
    state['scanmatcher']['upstream_version'] = '0.8.9'
    state['scanmatcher']['debian_version'] = '0.8.9-1'

    report = _evaluate(
        package_state=state,
        cli_result=(False, 'installed command is missing'),
    )

    assert report['status'] == 'FAIL'
    failed = {
        check['id']
        for check in report['checks']
        if check['status'] == 'FAIL'
    }
    assert failed == {
        'product-version-scanmatcher',
        'dependency-minimum-rko-lio',
        'installed-cli-contract',
    }


def test_dirty_contract_checkout_fails_evidence():
    report = _evaluate(source_state={
        'git_commit': 'a' * 40,
        'git_dirty': True,
    })

    assert report['status'] == 'FAIL'
    check = next(
        item for item in report['checks']
        if item['id'] == 'contract-worktree-clean'
    )
    assert check['status'] == 'FAIL'


def test_upgrade_proves_baseline_version_increase_and_no_stale_paths():
    baseline_state = _state(product_version='0.7.0')
    baseline_state['lidarslam']['owned_paths'].append(
        '/opt/ros/jazzy/share/lidarslam/removed-resource'
    )
    baseline = _evaluate(
        expected_version='0.7.0',
        mode='upgrade-baseline',
        package_state=baseline_state,
    )

    report = _evaluate(
        mode='upgrade-candidate',
        channel='testing',
        baseline_report=baseline,
        existing_paths=set(),
    )

    assert baseline['status'] == 'PASS'
    assert report['status'] == 'PASS'
    assert report['baseline']['expected_version'] == '0.7.0'
    assert {
        check['id'] for check in report['checks']
    } >= {
        'baseline-report-pass',
        'version-increased',
        'no-stale-package-paths',
    }


def test_upgrade_fails_closed_when_removed_owned_path_remains():
    baseline_state = _state(product_version='0.7.0')
    stale = '/opt/ros/jazzy/share/lidarslam/removed-resource'
    baseline_state['lidarslam']['owned_paths'].append(stale)
    baseline = _evaluate(
        expected_version='0.7.0',
        mode='upgrade-baseline',
        package_state=baseline_state,
    )

    report = _evaluate(
        mode='upgrade-candidate',
        channel='testing',
        baseline_report=baseline,
        existing_paths={stale},
    )

    assert report['status'] == 'FAIL'
    stale_check = next(
        check for check in report['checks']
        if check['id'] == 'no-stale-package-paths'
    )
    assert stale_check['status'] == 'FAIL'
    assert stale in stale_check['detail']


def test_upgrade_candidate_requires_schema_valid_baseline():
    with pytest.raises(
        CHECKER.InstallCheckError,
        match='requires --baseline-report',
    ):
        _evaluate(mode='upgrade-candidate')


@pytest.mark.parametrize(
    ('debian_version', 'expected'),
    [
        ('0.9.0-1noble.20260729.000000', '0.9.0'),
        ('1:0.9.0-2jammy.20260729.000000', '0.9.0'),
        ('0.9.0+dfsg-1', '0.9.0'),
    ],
)
def test_debian_upstream_version_extraction(debian_version, expected):
    assert CHECKER._upstream_version(debian_version) == expected


def test_package_set_cannot_omit_a_runtime_dependency():
    state = _state()
    del state['ndt-omp-ros2']

    with pytest.raises(
        CHECKER.InstallCheckError,
        match='package state differs',
    ):
        _evaluate(package_state=state)


def test_dpkg_collection_requires_fully_installed_packages(monkeypatch):
    def fake_run(command):
        if command[1] == '-W':
            return 'ii \t0.9.0-1noble.20260729.000000\n'
        return '/opt/ros/jazzy/share/lidarslam/package.xml\n'

    monkeypatch.setattr(CHECKER, '_run', fake_run)
    state = CHECKER.collect_package_state(
        'jazzy',
        package_names=('lidarslam',),
    )

    assert state['lidarslam']['upstream_version'] == '0.9.0'
    assert state['lidarslam']['owned_paths'] == [
        '/opt/ros/jazzy/share/lidarslam/package.xml',
    ]

    monkeypatch.setattr(
        CHECKER,
        '_run',
        lambda command: (
            'rc \t0.9.0-1\n'
            if command[1] == '-W'
            else '/opt/ros/jazzy/file\n'
        ),
    )
    with pytest.raises(CHECKER.InstallCheckError, match='not fully installed'):
        CHECKER.collect_package_state(
            'jazzy',
            package_names=('lidarslam',),
        )


def test_baseline_cli_check_uses_its_own_installed_version(monkeypatch):
    commands = []

    def fake_run(command):
        commands.append(command)
        return 'lidarslam_ros2 0.7.0\n'

    monkeypatch.setattr(CHECKER, '_run', fake_run)

    result = CHECKER._installed_cli_check(
        'jazzy',
        mode='upgrade-baseline',
        expected_version='0.7.0',
    )

    assert result[0] is True
    assert commands == [[
        '/opt/ros/jazzy/bin/lidarslam-map',
        '--version',
    ]]


def test_workflow_and_public_docs_preserve_full_package_manager_gate():
    workflow = (
        ROOT / '.github' / 'workflows'
        / 'package-manager-install-upgrade.yml'
    ).read_text(encoding='utf-8')
    distribution = (ROOT / 'docs' / 'distribution.md').read_text(
        encoding='utf-8')
    runbook = (ROOT / 'docs' / 'rosdistro-release.md').read_text(
        encoding='utf-8')
    release_bundle = (ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8')

    assert 'workflow_dispatch:' in workflow
    assert 'schedule:' not in workflow
    assert 'ros_distro: humble' in workflow
    assert 'ros_distro: jazzy' in workflow
    assert 'upgrade-baseline' in workflow
    assert 'upgrade-candidate' in workflow
    assert 'ros2-testing-apt-source' in workflow
    assert 'check_package_manager_install.py' in workflow
    assert 'lidarslam-map run' in workflow
    assert 'validate_real_data_e2e.py' in workflow
    assert 'package-manager-install-v1.schema.json' in distribution
    assert 'package-manager-install-upgrade.yml' in runbook
    assert 'scripts/check_package_manager_install.py' in release_bundle
    assert '.github/workflows/package-manager-install-upgrade.yml' in (
        release_bundle
    )
