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


"""Focused argv and fail-closed tests for the host release-leg launcher."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / 'scripts'
PROFILE = REPO / 'configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json'
LAUNCHER = SCRIPTS / 'run_registration_plugin_release_leg.py'
CAPTURE = SCRIPTS / 'capture_registration_plugin_dependency_environment.py'


if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def _load():
    spec = importlib.util.spec_from_file_location(
        'registration_plugin_release_launcher', str(LAUNCHER)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_capture():
    spec = importlib.util.spec_from_file_location(
        'dependency_environment_capture_launcher_fixture', str(CAPTURE)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def launcher():
    return _load()


def _local_profile(launcher, tmp_path, monkeypatch):
    mountpoint = tmp_path / 'mounted-evidence'
    mountpoint.mkdir()
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    storage = profile['release_matrix']['evidence_storage']
    storage['mountpoint'] = str(mountpoint)
    local_profile = tmp_path / 'profile.json'
    local_profile.write_text(json.dumps(profile, sort_keys=True), encoding='utf-8')
    observed = {
        'mountpoint': str(mountpoint.resolve()),
        'filesystem': storage['filesystem'],
        'source_uuid': storage['source_uuid'],
        'source': '/dev/test-evidence',
    }
    monkeypatch.setattr(launcher, '_mounted_storage_identity', lambda path: observed)
    return local_profile, mountpoint


def test_plan_derives_digest_environment_and_exact_mounts(launcher, tmp_path, monkeypatch):
    local_profile, mountpoint = _local_profile(launcher, tmp_path, monkeypatch)
    evidence_root = mountpoint / 'fresh-humble-absent'
    plan = launcher.build_plan(REPO, local_profile, 'humble', 'absent', evidence_root)
    digest = 'sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988'
    assert plan['image_reference'] == 'docker.io/library/ros:humble-ros-core@' + digest
    assert plan['env']['REGISTRATION_PLUGIN_CONTAINER_DIGEST'] == digest
    assert set(plan['env']) == set(launcher.REQUIRED_ENV_KEYS)
    assert plan['run_argv'][0:4] == ['docker', 'run', '-d', '--name']
    assert '--rm' not in plan['run_argv']
    assert any(digest in token for token in plan['run_argv'])
    assert all(
        '@sha256:' in token or token != 'ros:humble-ros-core'
        for token in plan['run_argv']
        if token.startswith('docker.io/ros:')
    )
    assert plan['mounts'] == [
        {'source': str(REPO.resolve()), 'destination': '/workspace/src', 'mode': 'ro'},
        {'source': str(evidence_root), 'destination': '/workspace/evidence-parent', 'mode': 'rw'},
    ]
    run_text = '\0'.join(plan['run_argv'])
    assert str(REPO.resolve()) + ':/workspace/src:ro' in run_text
    assert str(evidence_root) + ':/workspace/evidence-parent:rw' in run_text
    assert '--env\0REGISTRATION_PLUGIN_CONTAINER_DIGEST=' + digest in run_text
    assert 'docker.io/library/ros:humble-ros-core@' in run_text


def test_runner_shell_uses_selected_dependency_leg(launcher):
    absent = launcher._build_runner_shell('humble', 'absent')
    present = launcher._build_runner_shell('humble', 'present')
    assert '--dependency-leg absent' in absent
    assert 'registration-plugin-release-humble-absent' in absent
    assert '--dependency-leg present' in present
    assert 'registration-plugin-release-humble-present' in present
    assert absent != present


def test_runner_shell_exposes_canonical_source_package_root(launcher):
    shell = launcher._build_runner_shell('jazzy', 'absent')
    assert 'PYTHONPATH=/workspace/src:/workspace/src/scripts' in shell
    assert 'PYTHONPATH=/workspace/src/scripts ' not in shell


def test_dependency_capture_is_sealed_before_network_disconnect(launcher):
    plan = {
        'container_name': 'registration-plugin-humble-absent-fixture',
        'distro': 'humble',
        'dependency_leg': 'absent',
        'image': {
            'digest': 'sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988',
        },
    }
    argv = launcher._dependency_capture_argv(plan, {'returncode': 0})
    assert argv[:5] == [
        'docker', 'exec', plan['container_name'], 'python3',
        '/workspace/src/scripts/capture_registration_plugin_dependency_environment.py',
    ]
    assert '--install-exit-code' in argv
    assert argv[argv.index('--install-exit-code') + 1] == '0'
    assert '--install-command-sha256' in argv
    assert len(argv[argv.index('--install-command-sha256') + 1]) == 64
    assert len(argv[argv.index('--capture-script-sha256') + 1]) == 64
    assert not any(token.lower().find(secret) >= 0 for token in argv
                   for secret in ('password', 'token', 'proxy', 'credential', 'secret'))
    source = LAUNCHER.read_text(encoding='utf-8')
    assert source.index('dependency_environment_capture') < source.index(
        'network_disconnect_'
    )


def test_profile_digest_reaches_capture_parser_with_prefixed_image_digest(launcher):
    capture = _load_capture()
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    digest = next(
        item['image']['digest'] for item in profile['release_matrix']['distros']
        if item['name'] == 'humble'
    )
    plan = {
        'container_name': 'registration-plugin-humble-absent-fixture',
        'distro': 'humble',
        'dependency_leg': 'absent',
        'image': {'digest': digest},
    }
    argv = launcher._dependency_capture_argv(plan, {'returncode': 0})
    capture_start = argv.index(launcher.DEPENDENCY_CAPTURE_SCRIPT) + 1
    parsed = capture._parser().parse_args(argv[capture_start:])
    assert parsed.image_digest == digest
    assert capture.IMAGE_DIGEST_RE.fullmatch(parsed.image_digest)


def test_docker_inspect_daemon_error_is_not_treated_as_absence(launcher, tmp_path):
    def daemon_error(_argv, _label):
        return SimpleNamespace(returncode=1, stdout=b'daemon unavailable\n', stderr=b'')

    with pytest.raises(launcher.LauncherError) as error:
        launcher._docker_inspect(
            'registration-plugin-name', tmp_path / 'inspect.log', executor=daemon_error
        )
    assert error.value.kind == 'DOCKER_INSPECT_ABSENCE_UNPROVEN'


def test_docker_inspect_accepts_only_exact_not_found(launcher, tmp_path):
    def absent(_argv, _label):
        return SimpleNamespace(
            returncode=1,
            stdout=b'Error: No such object: registration-plugin-name\n',
            stderr=b'',
        )

    observed, record = launcher._docker_inspect(
        'registration-plugin-name', tmp_path / 'inspect.log', executor=absent
    )
    assert observed is None
    assert record['returncode'] == 1


@pytest.mark.parametrize(
    'bad_root',
    [
        'relative-root',
        '/tmp/launcher/../traversal',
        '/tmp/launcher/unsafe\nname',
        '/tmp/launcher/unsafe\x00name',
    ],
)
def test_plan_rejects_unsafe_or_nonfresh_root(launcher, tmp_path, monkeypatch, bad_root):
    local_profile, _ = _local_profile(launcher, tmp_path, monkeypatch)
    value = bad_root
    if value.startswith('/tmp/launcher/'):
        value = str(tmp_path / value.split('/tmp/launcher/', 1)[1])
    with pytest.raises(launcher.LauncherError):
        launcher.build_plan(REPO, local_profile, 'humble', 'absent', value)


def test_plan_rejects_existing_root_and_profile_injection(launcher, tmp_path, monkeypatch):
    local_profile, mountpoint = _local_profile(launcher, tmp_path, monkeypatch)
    existing = mountpoint / 'already-used'
    existing.mkdir()
    with pytest.raises(launcher.LauncherError) as error:
        launcher.build_plan(REPO, local_profile, 'humble', 'absent', existing)
    assert error.value.kind == 'ROOT_NOT_FRESH'
    with pytest.raises(launcher.LauncherError) as error:
        launcher.build_plan(
            REPO,
            str(local_profile) + '\n--unsafe',
            'humble',
            'absent',
            mountpoint / 'fresh-profile-reject',
        )
    assert error.value.kind == 'UNSAFE_PATH'


def test_plan_rejects_unmounted_expected_storage(launcher, tmp_path, monkeypatch):
    local_profile, mountpoint = _local_profile(launcher, tmp_path, monkeypatch)
    monkeypatch.setattr(launcher, '_mounted_storage_identity', lambda path: None)
    with pytest.raises(launcher.LauncherError) as error:
        launcher.build_plan(REPO, local_profile, 'humble', 'absent', mountpoint / 'unmounted-root')
    assert error.value.kind == 'EVIDENCE_MOUNT_UNAVAILABLE'


@pytest.mark.parametrize(
    'field,value',
    [
        ('filesystem', 'xfs'),
        ('source_uuid', '00000000-0000-0000-0000-000000000000'),
    ],
)
def test_plan_rejects_wrong_evidence_storage_identity(
    launcher, tmp_path, monkeypatch, field, value
):
    local_profile, mountpoint = _local_profile(launcher, tmp_path, monkeypatch)
    observed = {
        'mountpoint': str(mountpoint.resolve()),
        'filesystem': 'ext4',
        'source_uuid': '3b5dc9b7-c4de-4cf2-a892-00b2c063f34e',
        'source': '/dev/test-evidence',
    }
    observed[field] = value
    monkeypatch.setattr(launcher, '_mounted_storage_identity', lambda path: observed)
    with pytest.raises(launcher.LauncherError) as error:
        launcher.build_plan(
            REPO, local_profile, 'humble', 'absent', mountpoint / 'wrong-identity-root'
        )
    assert error.value.kind == 'EVIDENCE_MOUNT_IDENTITY_MISMATCH'


def test_source_forbids_manual_drift_and_forced_cleanup():
    text = LAUNCHER.read_text(encoding='utf-8')
    assert 'REGISTRATION_PLUGIN_CONTAINER_DIGEST' in text
    assert '"docker", "network", "disconnect"' in text
    assert '--rm' not in text
    assert '["docker", "rm", "-f"' not in text
    assert 'safe.directory=*' not in text
    assert 'git config --global' not in text
    assert 'network_after_dependency_fetch' in text
    assert '"docker_build": False' in text


def test_cleanup_inspections_use_distinct_immutable_logs(launcher, tmp_path):
    root = tmp_path / 'reserved'
    (root / 'launcher').mkdir(parents=True)
    plan = {'container_name': 'registration-plugin-test'}

    def fake_executor(argv, label):
        del argv, label
        return SimpleNamespace(returncode=0, stdout=b'[{"Id":"abc"}]', stderr=b'')

    first, first_record = launcher._inspect_container_for_cleanup(
        plan, fake_executor, root, 'initial'
    )
    second, second_record = launcher._inspect_container_for_cleanup(
        plan, fake_executor, root, 'post_stop'
    )
    assert first == second == {'Id': 'abc'}
    assert first_record['log']['path'].endswith('cleanup_inspect_initial.log')
    assert second_record['log']['path'].endswith('cleanup_inspect_post_stop.log')
    assert (root / 'launcher' / 'cleanup_inspect_initial.log').is_file()
    assert (root / 'launcher' / 'cleanup_inspect_post_stop.log').is_file()
    with pytest.raises(launcher.LauncherError, match='invalid'):
        launcher._inspect_container_for_cleanup(plan, fake_executor, root, 'again')
