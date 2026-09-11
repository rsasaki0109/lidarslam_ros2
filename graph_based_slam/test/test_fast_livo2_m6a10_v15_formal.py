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
Host-only v15 formal candidate and scoped-builder tests.

These tests never open the pinned bag and never invoke Docker, ROS, GT,
scoring, or map generation.  Immutable external receipts are checked by
computed bytes, sidecar filename, sidecar content, and sidecar SHA.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load('m6a10_v15_formal_test', ROOT / 'scripts/run_fast_livo2_m6a10_v15_formal.py')
AUTH = _load(
    'm6a10_v15_authorizer_test', ROOT / 'scripts/authorize_fast_livo2_m6a10_v15_formal.py'
)


def _mounts(argv):
    return [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == '--mount']


def test_v15_profile_image_source_and_authorizer_identity_are_pinned():
    profile = MODULE.verify_candidate_profile()
    assert profile['sha256'] == '017e8583e1085b1f11da4117a1c53497cdcb1055c6c6224568df2f73ba751bc7'
    assert AUTH.PROFILE_SHA256 == profile['sha256']
    assert (
        MODULE.IMAGE_ID
        == 'sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a'
    )
    assert MODULE.AUTHORIZER_SHA256 == hashlib.sha256(AUTH.SCRIPT.read_bytes()).hexdigest()
    assert MODULE.FEEDER_SHA256 == AUTH.FEEDER_SHA256
    assert MODULE.WRAPPER_SHA256 == AUTH.WRAPPER_SHA256
    text = MODULE.PROFILE_PATH.read_text(encoding='utf-8')
    assert 'V15_FORMAL_CANDIDATE_UNAUTHORIZED' in text
    assert 'formal_replay_forbidden: true' in text
    assert 'output_tmpfs: false' in text
    assert 'formal_replay_started: false' in text


def test_v15_builder_calls_captured_original_once_and_has_single_rw_output(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    original = MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV
    calls = []

    def counted(value, output):
        calls.append((value, output))
        return original(value, output)

    MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = counted
    try:
        argv = MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    finally:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = original
    assert len(calls) == 1
    mounts = _mounts(argv)
    destinations = [MODULE._mount_destination(item) for item in mounts]
    assert destinations == [
        '/input/ntu_viral.bag',
        '/out',
        MODULE.WRAPPER_PATH_IN_CONTAINER,
        MODULE.FEEDER_PATH_IN_CONTAINER,
    ]
    assert sum(item.endswith(',dst=/out,readonly=false') for item in mounts) == 1
    assert MODULE.IMAGE_ID == argv[-1]
    assert MODULE.IMAGE_TAG not in argv
    assert MODULE._tmpfs_destinations(argv) == ['/tmp', '/root/.ros']
    assert not any(item.startswith('/out:') for item in argv)
    assert '--rm' not in argv and 'rw' not in argv


def test_v15_argv_has_explicit_ros_environment_and_no_shell_or_capability():
    config = MODULE.v12.CandidateConfig(root=Path('/tmp/v15-static-root'))
    argv = MODULE.build_safe_docker_argv(config, Path('/tmp/v15-static-out'))
    env = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == '--env']
    for key, value in MODULE.ROS_ENV.items():
        assert '%s=%s' % (key, value) in env
    assert argv[argv.index('--entrypoint') + 1] == MODULE.WRAPPER_PATH_IN_CONTAINER
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--init' in argv and '--pull=never' in argv
    assert not any(
        item in {'--privileged', '--cap-add', '--device', '--pid=host'} for item in argv
    )
    assert not any(item in {'sh', 'bash', '-c', '-lc', '--shell'} for item in argv)


def test_v15_scoped_runtime_restores_builder_after_success_and_exception(tmp_path):
    before = MODULE.v12.build_safe_docker_argv
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')

    class Process:
        pid = 424242

        def wait(self):
            return 0

    calls = []

    def factory(argv, cwd):
        calls.append((list(argv), cwd))
        return Process()

    result = MODULE.run_injected_once(config, factory)
    assert result['start_count'] == 1 and result['popen_count'] == 1
    assert len(calls) == 1
    assert MODULE.v12.build_safe_docker_argv is before

    def failing(_argv, _cwd):
        raise RuntimeError('injected Popen failure')

    with pytest.raises(RuntimeError, match='injected Popen failure'):
        MODULE.run_injected_once(MODULE.v12.CandidateConfig(root=tmp_path / 'root2'), failing)
    assert MODULE.v12.build_safe_docker_argv is before


def test_v15_builder_rejects_extra_mount_and_capability_without_recursion(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    original = MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV
    baseline = original(config, tmp_path / 'out')
    try:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = lambda _c, _o: list(baseline) + [
            '--mount',
            'type=bind,src=/tmp/extra,dst=/extra,readonly',
        ]
        with pytest.raises(MODULE.CandidateError, match='allowlist'):
            MODULE.build_safe_docker_argv(config, tmp_path / 'out2')
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = lambda _c, _o: list(baseline[:-1]) + [
            '--privileged',
            baseline[-1],
        ]
        with pytest.raises(MODULE.CandidateError, match='capability'):
            MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    finally:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = original


def test_persistence_capture_binds_feeder_and_schema3_callback(tmp_path):
    root = tmp_path / 'attempt'
    output = root / 'out'
    output.mkdir(parents=True)
    fixture = json.loads(
        (
            ROOT / 'graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json'
        ).read_text()
    )
    for name, value in {
        'feeder_receipt.json': {'status': 'PASS', 'schema_version': 1},
        'callback_consumer_evidence.json': fixture,
        'consumer_evidence.json': {'schema_version': 1, 'status': 'pass'},
        'online_compute_timing.json': {'schema_version': 1, 'status': 'pass', 'rtf': 1.0},
    }.items():
        (output / name).write_text(json.dumps(value), encoding='utf-8')
    (output / 'feeder.log').write_text('feeder persisted\n', encoding='utf-8')
    (output / 'feeder_exit_status.txt').write_text('0\n', encoding='ascii')
    result = MODULE._capture_with_persistence_diagnostics(
        MODULE.v12.CandidateConfig(root=root), root
    )
    assert result['missing'] == []
    assert result['feeder_persistence'] == {
        'receipt_present': True,
        'stderr_present': True,
        'exit_status_present': True,
    }
    assert result['documents']['callback']['schema_version'] == 3
    assert result['bindings']['callback']['path'].endswith('callback_consumer_evidence.json')


def test_default_entrypoint_is_unauthorized_before_root_or_bag(tmp_path):
    root = tmp_path / 'never-created'
    assert MODULE.main(['--root', str(root)]) == 11
    assert not root.exists()


def test_authorizer_rejects_reused_roots_and_detects_immutable_build_drift(tmp_path):
    existing_auth = tmp_path / 'auth'
    existing_auth.mkdir()
    with pytest.raises(AUTH.AuthorizationError, match='already exists'):
        AUTH.authorize(existing_auth, tmp_path / 'attempt', window_runner=lambda *a, **k: {})
    with pytest.raises(AUTH.AuthorizationError, match='build SHA drift'):
        AUTH._json(
            AUTH.BUILD_RECEIPT_PATH, '0' * 64, 'v15 build', AUTH.BUILD_RECEIPT_SIDECAR_SHA256
        )
    assert AUTH._verify_immutable_lineage()['build']['sha256'] == AUTH.BUILD_RECEIPT_SHA256


def test_build_receipt_sidecar_binds_filename_content_and_computed_bytes():
    path = AUTH.BUILD_RECEIPT_PATH
    sidecar = path.with_name(path.name + '.sha256')
    assert path.name == 'build_identity.receipt.json'
    assert sidecar.name == 'build_identity.receipt.json.sha256'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == AUTH.BUILD_RECEIPT_SHA256
    assert hashlib.sha256(sidecar.read_bytes()).hexdigest() == AUTH.BUILD_RECEIPT_SIDECAR_SHA256
    assert sidecar.read_bytes() == ('%s  %s\n' % (AUTH.BUILD_RECEIPT_SHA256, path.name)).encode(
        'ascii'
    )
    value = AUTH._json(
        path, AUTH.BUILD_RECEIPT_SHA256, 'v15 build', AUTH.BUILD_RECEIPT_SIDECAR_SHA256
    )
    assert value['status'] == 'PASS'


def test_authorizer_static_surface_never_starts_runtime():
    text = AUTH.SCRIPT.read_text(encoding='utf-8')
    assert 'subprocess' not in text
    assert 'docker run' not in text.lower()
    assert 'rosbag play' not in text.lower()
    assert 'formal_replay_started: false' in AUTH.PROFILE_PATH.read_text(encoding='utf-8')
