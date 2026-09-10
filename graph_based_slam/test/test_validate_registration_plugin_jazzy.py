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

"""Adversarial tests for the host Jazzy validation command construction."""

import importlib.util
import json
import os
from pathlib import Path
import stat

import pytest


REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'scripts' / 'validate_registration_plugin_jazzy.py'
PYTHON_INSTALL_HELPER = REPO / 'cmake' / 'lidarslam_benchmark_python.cmake'


@pytest.fixture(scope='module')
def launcher():
    spec = importlib.util.spec_from_file_location(
        'validate_registration_plugin_jazzy', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _roots(tmp_path):
    return {
        name: tmp_path / name
        for name in ('build', 'install', 'log', 'test_results')}


def test_all_colcon_verbs_put_log_base_before_verb(launcher, tmp_path):
    roots = _roots(tmp_path)
    for verb in ('build', 'test', 'test-result'):
        command = launcher.build_colcon_argv(verb, launcher.REPO, roots)
        launcher.assert_global_options_before_verb(command)
        assert command[:3] == ['colcon', '--log-base', str(roots['log'])]
        assert command[3] == verb
    build = launcher.build_colcon_argv('build', launcher.REPO, roots)
    selected = build[build.index('--packages-select') + 1:
                     build.index('--event-handlers')]
    assert selected == list(launcher.TARGET_PACKAGES)


def test_misplaced_global_option_is_rejected(launcher):
    with pytest.raises(launcher.ValidationError) as error:
        launcher.assert_global_options_before_verb(['colcon', 'build', '--log-base', '/tmp/log'])
    assert error.value.kind == 'COLCON_GLOBAL_OPTION_ORDER'


def test_old_underlay_and_repository_pythonpath_are_rejected(launcher, monkeypatch):
    monkeypatch.setattr(
        launcher,
        'underlay_paths',
        lambda distro: (Path('/opt/ros/jazzy'),
                        Path('/opt/ros/jazzy/setup.bash')),
    )
    with pytest.raises(launcher.ValidationError) as error:
        launcher.validate_clean_underlay_environment(
            {
                'AMENT_PREFIX_PATH': '/tmp/old/install:/opt/ros/jazzy',
                'CMAKE_PREFIX_PATH': '/opt/ros/jazzy',
                'PYTHONPATH': '',
            },
            'jazzy',
        )
    assert error.value.kind == 'OLD_UNDERLAY'
    with pytest.raises(launcher.ValidationError) as error:
        launcher.validate_clean_underlay_environment(
            {
                'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
                'CMAKE_PREFIX_PATH': '/opt/ros/jazzy',
                'PYTHONPATH': str(launcher.REPO),
            },
            'jazzy',
        )
    assert error.value.kind == 'REPOSITORY_PYTHONPATH'


def test_work_root_must_be_fresh_and_outside_source(launcher, tmp_path):
    existing = tmp_path / 'existing'
    existing.mkdir()
    with pytest.raises(launcher.ValidationError) as error:
        launcher.validate_fresh_root(existing)
    assert error.value.kind == 'WORK_ROOT_NOT_FRESH'
    with pytest.raises(launcher.ValidationError) as error:
        launcher.validate_fresh_root(launcher.REPO / 'validation')
    assert error.value.kind == 'WORK_ROOT_SOURCE_OVERLAP'


def test_source_manifest_is_deterministic_and_does_not_write_repo(launcher):
    before = {
        path: path.stat().st_mtime_ns
        for path in (launcher.REPO / 'scripts').glob('*.py')}
    first = launcher.source_manifest(launcher.REPO)
    second = launcher.source_manifest(launcher.REPO)
    assert first == second
    assert first['manifest_sha256']
    after = {
        path: path.stat().st_mtime_ns
        for path in (launcher.REPO / 'scripts').glob('*.py')}
    assert before == after


def test_source_manifest_drift_is_rejected(launcher):
    start = launcher.source_manifest(launcher.REPO)
    end = json.loads(json.dumps(start))
    end['manifest_sha256'] = '0' * 64
    with pytest.raises(launcher.ValidationError) as error:
        launcher.require_source_manifest_match(start, end)
    assert error.value.kind == 'SOURCE_MANIFEST_DRIFT'


def test_installed_surface_checks_module_origin_and_count(launcher, tmp_path):
    site = tmp_path / 'install' / 'lib' / 'python3.12' / 'site-packages'
    package = site / 'lidarslam_benchmark_tools'
    (package / 'nested').mkdir(parents=True)
    (package / '__init__.py').write_text('', encoding='utf-8')
    (package / 'alpha.py').write_text('VALUE = 1\n', encoding='utf-8')
    (package / 'nested' / '__init__.py').write_text('', encoding='utf-8')
    (package / 'nested' / 'beta.py').write_text('VALUE = 2\n', encoding='utf-8')
    wrapper = (
        tmp_path / 'install' / 'lib' / 'graph_based_slam'
        / 'lidarslam_benchmark_tool')
    wrapper.parent.mkdir(parents=True)
    wrapper.write_text(
        '#!/usr/bin/env python3\nprint("help")\n', encoding='utf-8')
    wrapper.chmod(stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR)
    result = launcher.verify_installed_python_surface(
        tmp_path / 'install',
        {'PYTHONPATH': '', 'PATH': os.environ['PATH']},
        expected_module_count=2,
    )
    assert result['module_count'] == result['origin_count'] == 2
    assert result['site_paths'] == [str(site)]


def test_host_validator_is_source_only_and_explicitly_excluded(launcher):
    assert SCRIPT.is_file()
    helper = PYTHON_INSTALL_HELPER.read_text(encoding='utf-8')
    assert 'PATTERN "*.py"' in helper
    assert 'PATTERN "validate_registration_plugin_jazzy.py" EXCLUDE' in helper
    assert launcher.EXPECTED_INSTALLED_MODULE_COUNT == 298


def test_nested_junit_summary_counts_leaf_suites_once(launcher, tmp_path):
    xml = tmp_path / 'nested.xml'
    xml.write_text(
        """<testsuites tests='3'><testsuite tests='3' failures='1'>
        <testsuite tests='1' failures='1'/><testsuite tests='2'/>
        </testsuite></testsuites>""",
        encoding='utf-8',
    )
    summary = launcher.test_result_summary(tmp_path)
    assert summary['files'] == 1
    assert summary['tests'] == 3
    assert summary['failures'] == 1
    assert summary['errors'] == summary['skipped'] == 0
    assert summary['invalid_files'] == summary['partial_files'] == 0
    assert summary['source_kind_counts'] == {'ctest': 0, 'junit': 1}


def test_ctest_site_summary_matches_colcon_units(launcher, tmp_path):
    (tmp_path / 'Testing').mkdir()
    (tmp_path / 'Testing' / 'Test.xml').write_text(
        """<Site><Testing>
        <Test Status='passed'><Name>ok</Name></Test>
        <Test Status='failed'><Name>bad</Name></Test>
        <Test Status='notrun'><Name>skip</Name></Test>
        </Testing></Site>""",
        encoding='utf-8')
    summary = launcher.test_result_summary(tmp_path, include_junit=False)
    assert summary['files'] == 1
    assert summary['tests'] == 3
    assert summary['failures'] == 1
    assert summary['skipped'] == 1
    assert summary['errors'] == 0
    assert summary['partial_files'] == summary['invalid_files'] == 0


def test_empty_or_malformed_result_xml_is_not_success(launcher, tmp_path):
    (tmp_path / 'empty.xml').write_text(
        '<Site><Testing/></Site>', encoding='utf-8')
    (tmp_path / 'broken.xml').write_text('<Site>', encoding='utf-8')
    summary = launcher.test_result_summary(tmp_path, include_junit=False)
    assert summary['invalid_files'] == 1
    assert summary['partial_files'] == 1
    with pytest.raises(launcher.ValidationError) as error:
        launcher.validate_test_result_consistency(
            summary,
            {'tests': 0, 'errors': 0, 'failures': 0, 'skipped': 0},
            {'files': 0, 'tests': 0, 'failures': 0, 'errors': 0,
             'skipped': 0, 'invalid_files': 0, 'partial_files': 0},
        )
    assert error.value.kind == 'TEST_RESULT_XML_PARTIAL'


def test_colcon_summary_parser_rejects_missing_summary(launcher, tmp_path):
    log = tmp_path / 'test-result.log'
    log.write_text('no summary\n', encoding='utf-8')
    with pytest.raises(launcher.ValidationError) as error:
        launcher.parse_colcon_test_result_summary(log)
    assert error.value.kind == 'TEST_RESULT_SUMMARY_MISSING'


def test_validation_uses_long_global_timeout(launcher):
    assert launcher.TEST_TIMEOUT_SECONDS >= 1800


def test_historical_summary_is_bounded_and_hashed(launcher, tmp_path):
    root = tmp_path / 'historical'
    xml_dir = root / 'test_results'
    xml_dir.mkdir(parents=True)
    cases = ''.join(
        f"""<testcase name='failure-{index}'><failure/></testcase>"""
        for index in range(20)
    )
    (xml_dir / 'results.xml').write_text(
        f"""<testsuite>{cases}</testsuite>""", encoding='utf-8')
    summary = launcher._historical_root_summary(root)
    assert summary['failure_case_count'] == 20
    assert summary['distinct_failure_name_count'] == 20
    assert len(summary['sample_failure_names']) == launcher.HISTORICAL_SAMPLE_LIMIT
    assert 'names' not in summary
    assert len(summary['distinct_failure_names_sha256']) == 64
