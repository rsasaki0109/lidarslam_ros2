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


"""
Static and host-only tests for the v15 no-input identity correction gate.

No test in this module invokes Docker, ROS, a dataset, GT, a scorer, or a
formal replay.  Docker/ROS behavior is covered only through fixed argv and
synthetic inspect documents.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'scripts/run_fast_livo2_m6a10_v15_no_input_identity.py'


def _module():
    spec = importlib.util.spec_from_file_location('v15_no_input_identity', SOURCE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_image_and_v15_source_identity_is_pinned():
    module = _module()
    assert module.IMAGE_ID == (
        'sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a'
    )
    assert module.FEEDER_SHA256 == (
        '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7'
    )
    assert (
        hashlib.sha256((ROOT / module.FEEDER_PATH).read_bytes()).hexdigest()
        == module.FEEDER_SHA256
    )
    assert (
        hashlib.sha256((ROOT / module.PROFILE_PATH).read_bytes()).hexdigest()
        == module.PROFILE_SHA256
    )
    assert (
        hashlib.sha256((ROOT / module.WRAPPER_PATH).read_bytes()).hexdigest()
        == module.WRAPPER_SHA256
    )
    assert (
        hashlib.sha256((ROOT / module.FIXTURE_PATH).read_bytes()).hexdigest()
        == module.FIXTURE_SHA256
    )
    assert (
        hashlib.sha256((ROOT / module.V12_PATCH_PATH).read_bytes()).hexdigest()
        == module.V12_PATCH_SHA256
    )


def test_docker_argv_uses_exact_image_id_isolation_tmpfs_and_ros_env():
    module = _module()
    argv = module.build_docker_argv('m6a10-v15-id-static-test')
    assert argv[:2] == ['docker', 'run']
    assert argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv
    assert argv.count('--tmpfs') == 3
    assert {argv[i + 1] for i, item in enumerate(argv) if item == '--tmpfs'} == {
        '/tmp:rw,nosuid,nodev',
        '/root/.ros:rw,nosuid,nodev',
        '/out:rw,nosuid,nodev',
    }
    env_values = [argv[i + 1] for i, item in enumerate(argv) if item == '--env']
    assert env_values == [
        'ROS_MASTER_URI=http://127.0.0.1:11311',
        'ROS_IP=127.0.0.1',
        'ROS_HOSTNAME=127.0.0.1',
        'ROS_HOME=/tmp/ros_home',
        'ROS_LOG_DIR=/tmp/ros_logs',
    ]
    assert argv.count('--env') == 5
    assert module.IMAGE_ID in argv
    assert module.IMAGE_TAG not in argv
    assert '--rm' not in argv
    assert '--mount' not in argv and '-v' not in argv and '--volume' not in argv
    flattened = ' '.join(argv).lower()
    for forbidden in ('rosbag', 'ground_truth', 'scorer', 'map', 'input'):
        assert forbidden not in flattened


def test_ros_environment_missing_unknown_and_drift_fail_closed():
    module = _module()
    assert module.require_ros_env(module.ROS_ENV) == module.ROS_ENV
    missing = dict(module.ROS_ENV)
    missing.pop('ROS_MASTER_URI')
    with pytest.raises(module.GateError, match='ROS env keys'):
        module.require_ros_env(missing)
    unknown = dict(module.ROS_ENV)
    unknown['ROS_PACKAGE_PATH'] = '/host/ros'
    with pytest.raises(module.GateError, match='ROS env keys'):
        module.require_ros_env(unknown)
    drifted = dict(module.ROS_ENV)
    drifted['ROS_MASTER_URI'] = 'http://host-provided:11311'
    with pytest.raises(module.GateError, match='ROS env values'):
        module.require_ros_env(drifted)


def test_selftest_is_schema3_valid_invalid_and_compiles():
    module = _module()
    source = module._selftest_python()
    compile(source, '<v15-selftest>', 'exec')
    assert 'validate_consumer_status(value)' in source
    assert 'wrong-transport-contract' in source
    assert 'ConsumerStatusError' in source
    assert module.SELFTEST_MARKER in source


def _inspect_document(module):
    return {
        'State': {'Status': 'exited', 'ExitCode': 0, 'OOMKilled': False},
        'HostConfig': {
            'NetworkMode': 'none',
            'ReadonlyRootfs': True,
            'Binds': [],
            'Tmpfs': {
                '/tmp': ['rw', 'nosuid', 'nodev'],
                '/root/.ros': ['rw', 'nosuid', 'nodev'],
                '/out': ['rw', 'nosuid', 'nodev'],
            },
        },
        'Mounts': [
            {'Type': 'tmpfs', 'Destination': '/tmp'},
            {'Type': 'tmpfs', 'Destination': '/root/.ros'},
            {'Type': 'tmpfs', 'Destination': '/out'},
        ],
    }


def test_container_validation_accepts_only_natural_clean_identity_shape():
    module = _module()
    result = module._validate_container(
        _inspect_document(module), 0, 'prefix %s suffix' % module.SELFTEST_MARKER
    )
    assert result['status'] == 'exited'
    assert result['exit_code'] == 0
    assert result['oom_killed'] is False
    assert result['host_bind_mounts'] == 0
    assert result['tmpfs_destinations'] == ['/out', '/root/.ros', '/tmp']


def test_container_validation_accepts_docker_tmpfs_only_mounts_representation():
    module = _module()
    value = _inspect_document(module)
    value['Mounts'] = []
    result = module._validate_container(value, 0, module.SELFTEST_MARKER)
    assert result['host_bind_mounts'] == 0
    assert result['tmpfs_destinations'] == ['/out', '/root/.ros', '/tmp']


@pytest.mark.parametrize(
    'mutator',
    [
        lambda value: value['HostConfig'].update({'Binds': ['/host:/out']}),
        lambda value: value['HostConfig'].update({'NetworkMode': 'default'}),
        lambda value: value['HostConfig'].update({'ReadonlyRootfs': False}),
        lambda value: value['State'].update({'OOMKilled': True}),
        lambda value: value['Mounts'].__setitem__(0, {'Type': 'bind', 'Destination': '/out'}),
        lambda value: value['State'].update({'Status': 'running'}),
    ],
)
def test_container_validation_rejects_isolation_or_lifecycle_drift(mutator):
    module = _module()
    value = _inspect_document(module)
    mutator(value)
    with pytest.raises(module.GateError):
        module._validate_container(value, 0, module.SELFTEST_MARKER)


def test_receipts_are_exclusive_immutable_and_sidecar_bound(tmp_path):
    module = _module()
    value = {'schema_version': 1, 'status': 'PASS', 'formal_replay_started': False}
    receipt, sidecar, digest = module._seal_receipt(tmp_path, value)
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert sidecar.stat().st_mode & 0o777 == 0o444
    assert json.loads(receipt.read_text(encoding='utf-8')) == value
    assert digest == hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert sidecar.read_text(encoding='ascii') == '%s  %s\n' % (digest, receipt.name)
    with pytest.raises(module.GateError, match='already exists'):
        module._seal_receipt(tmp_path, value)


def test_runner_has_one_start_path_no_retry_or_host_env_inheritance():
    _module()
    source = SOURCE.read_text(encoding='utf-8')
    assert source.count('subprocess.run(') == 1
    assert source.count('"docker", "run"') == 1
    assert 'retry' not in source.lower()
    assert 'os.environ' not in source
    assert '"--env-file"' not in source
    assert '"host_environment_inherited": False' in source
