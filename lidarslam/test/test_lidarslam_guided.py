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

"""Tests for the beginner-facing guided product workflow."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'lidarslam_guided.py'
RUNNER_SCRIPT = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('lidarslam_guided', SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_runner_module():
    spec = importlib.util.spec_from_file_location(
        'run_autoware_map_from_bag',
        RUNNER_SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _payload(*, recommendations: list[dict[str, object]]):
    return {
        'summary': {
            'bag_path': '/tmp/demo_bag',
            'duration_sec': 72.5,
            'topics': {
                'pointcloud2': [
                    {'name': '/points', 'message_count': 1200},
                ],
                'imu': [
                    {'name': '/imu', 'message_count': 7200},
                ],
                'navsatfix': [],
                'velodyne_scan': [],
            },
            'pointcloud_inspection': {
                'status': 'inspected',
                'reason': 'RKO-LIO-compatible per-point timestamp field was found.',
            },
            'timestamp_order': {
                'status': 'passed',
                'reason': 'All selected timestamps are monotonic.',
            },
        },
        'recommendations': recommendations,
        'recommended_profile_id': (
            recommendations[0]['id'] if recommendations else None
        ),
        'findings': (
            []
            if recommendations
            else [{
                'code': 'pointcloud-layout-incompatible',
                'message': 'PointCloud2 input was not usable.',
                'next_action': (
                    'Rewrite the PointCloud2 layout, then run '
                    'lidarslam-map doctor <rosbag2_dir>.'
                ),
            }]
        ),
        'missing_requirements': ['PointCloud2 input was not usable.'],
    }


def _fake_preflight(payload):
    class FakePreflight:
        PROFILE_HELP = (
            ('rko_lio_graph_public_path', 'public path'),
        )

        @staticmethod
        def validate_bag_path(_path):
            return None

        @staticmethod
        def build_preflight_payload(_path):
            return payload

        @staticmethod
        def select_profile(_payload, forced_profile_id=None):
            if forced_profile_id:
                return forced_profile_id
            return _payload['recommendations'][0]['id']

    return FakePreflight


def test_guided_help_is_short_and_actionable():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'guide a safe map run through verification' in result.stdout
    assert '--yes' in result.stdout
    assert '--viewer {none,browser,autoware,foxglove}' in result.stdout
    assert 'use "run" directly in scripts' in result.stdout


def test_guided_stops_with_actionable_not_ready_report(monkeypatch, tmp_path, capsys):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    payload = _payload(recommendations=[])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))

    result = module.main([str(bag)])

    assert result == 2
    output = capsys.readouterr().out
    assert 'NOT READY' in output
    assert 'Detected inputs:' in output
    assert 'IMU:' in output
    assert '[pointcloud-layout-incompatible]' in output
    assert 'Next: Rewrite the PointCloud2 layout' in output
    assert 'doctor <rosbag2_dir>' in output


def test_guided_stops_before_confirmation_when_runtime_is_incomplete(
    monkeypatch,
    tmp_path,
    capsys,
):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(
        module,
        'runtime_readiness',
        lambda _profile: ['missing runtime artifact: graph_based_slam/local_setup.bash'],
    )

    result = module.main([str(bag)])

    assert result == 2
    output = capsys.readouterr().out
    assert 'the input is compatible, but the local runtime is incomplete' in output
    assert 'Runtime check: FAILED' in output
    assert '[runtime-incomplete]' in output
    assert 'graph_based_slam/local_setup.bash' in output
    assert 'source_quickstart.sh' in output
    assert '--workspace' in output
    assert '--build-only' in output
    assert 'install/setup.bash' in output


def test_source_runtime_finds_documented_workspace_install(
    monkeypatch,
    tmp_path,
):
    """A standard <workspace>/src checkout must find <workspace>/install."""
    module = _load_module()
    workspace = tmp_path / 'workspace with spaces'
    repo = workspace / 'src' / 'lidar_slam_ros2'
    install = workspace / 'install'
    repo.mkdir(parents=True)
    (install / 'setup.bash').parent.mkdir(parents=True)
    (install / 'setup.bash').write_text('# test\n', encoding='utf-8')
    monkeypatch.setattr(module, 'REPO_ROOT', repo)
    monkeypatch.setattr(module, 'SOURCE_LAYOUT', True)

    assert module._source_workspace_root() == workspace
    assert module._runtime_install_roots() == (install,)
    next_steps = module._render_runtime_next_steps()
    rendered = '\n'.join(next_steps)
    assert f"cd '{repo}'" in rendered
    assert f"--workspace '{workspace}'" in rendered
    assert '--build-only' in rendered
    assert f"source '{install / 'setup.bash'}'" in rendered
    assert len(next_steps) == 4


def test_guided_delegates_run_then_viewer_without_touching_slam_logic(
    monkeypatch,
    tmp_path,
    capsys,
):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    output_dir = tmp_path / 'map'
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(module, 'runtime_readiness', lambda _profile: [])
    monkeypatch.setattr(module, 'WORK_ROOT', tmp_path)
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)

    result = module.main([
        str(bag),
        '--output-dir',
        str(output_dir),
        '--yes',
        '--viewer',
        'foxglove',
    ])

    assert result == 0
    assert len(calls) == 2
    assert calls[0][1].endswith('run_autoware_map_from_bag.py')
    assert '--output-dir' in calls[0]
    assert str(output_dir.resolve()) in calls[0]
    assert calls[1][1].endswith('view_autoware_map.py')
    assert calls[1][-2:] == ['--viewer', 'foxglove']
    assert 'Next steps:' in capsys.readouterr().out


def test_guided_requires_yes_when_no_terminal_is_available(
    monkeypatch,
    tmp_path,
    capsys,
):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(module, 'runtime_readiness', lambda _profile: [])
    monkeypatch.setattr(module.sys.stdin, 'isatty', lambda: False)

    result = module.main([str(bag)])

    assert result == 2
    assert '--yes' in capsys.readouterr().err


def test_guided_cancellation_never_starts_the_runner(
    monkeypatch,
    tmp_path,
    capsys,
):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(module, 'runtime_readiness', lambda _profile: [])
    monkeypatch.setattr(module, '_ask_to_start', lambda: False)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError('cancelled guided flow started a subprocess')

    monkeypatch.setattr(module.subprocess, 'run', fail_if_called)

    assert module.main([str(bag)]) == 0
    assert 'Cancelled. No map workflow was started.' in capsys.readouterr().out


def test_guided_runner_failure_keeps_an_actionable_inspect_command(
    monkeypatch,
    tmp_path,
    capsys,
):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    output_dir = tmp_path / 'map output'
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(module, 'runtime_readiness', lambda _profile: [])
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **_kwargs: subprocess.CompletedProcess(command, 42),
    )

    result = module.main([
        str(bag),
        '--output-dir',
        str(output_dir),
        '--yes',
    ])

    captured = capsys.readouterr()
    assert result == 42
    assert 'failed with exit code 42' in captured.err
    assert 'lidarslam-map inspect' in captured.out
    assert str(output_dir) in captured.out


def test_guided_dry_run_prints_exact_delegated_command(monkeypatch, tmp_path, capsys):
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    output_dir = tmp_path / 'map'
    payload = _payload(recommendations=[{
        'id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO public path',
        'why': ['PointCloud2 and Imu are available.'],
    }])
    monkeypatch.setattr(module, '_load_preflight_module', lambda: _fake_preflight(payload))
    monkeypatch.setattr(module, 'runtime_readiness', lambda _profile: [])
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)

    result = module.main([
        str(bag),
        '--output-dir',
        str(output_dir),
        '--dry-run',
    ])

    assert result == 0
    assert calls and '--dry-run' in calls[0]
    assert not output_dir.exists()
    assert 'Delegated command:' in capsys.readouterr().out


def test_default_output_dir_is_safe_for_bag_names(tmp_path):
    module = _load_module()
    path = module.default_output_dir(tmp_path / 'bag with spaces.db3')

    assert path.parent.name == 'output'
    assert path.name.startswith('lidarslam_bag_with_spaces_')
    assert ' ' not in path.name


def test_guided_verification_result_rejects_prefix_lookalikes(tmp_path):
    module = _load_module()
    verify_log = tmp_path / 'verify_autoware_map.log'

    assert module._verify_result(tmp_path) == 'not available'
    verify_log.write_text('RESULT: PASSING\n', encoding='utf-8')
    assert module._verify_result(tmp_path) == 'unknown'
    verify_log.write_text(
        'RESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    assert module._verify_result(tmp_path) == 'PASS'
    verify_log.write_text('RESULT: FAIL -- missing metadata\n', encoding='utf-8')
    assert module._verify_result(tmp_path) == 'FAIL'


def test_run_guided_forwards_only_the_interaction_layer(monkeypatch, tmp_path):
    runner = _load_runner_module()
    recorded: list[list[str]] = []

    class FakeGuided:
        @staticmethod
        def main(arguments):
            recorded.append(list(arguments))
            return 0

    monkeypatch.setattr(
        runner,
        '_load_script_module',
        lambda _name, _module_name: FakeGuided,
    )
    args = SimpleNamespace(
        bag=str(tmp_path / 'bag'),
        profile='rko_lio_graph_public_path',
        output_dir=str(tmp_path / 'output'),
        min_free_space_gib=5.0,
        verification_enabled=True,
        viewer='none',
        yes=True,
        dry_run=True,
        editable=True,
        resume=False,
    )

    assert runner._run_guided(args) == 0
    assert recorded == [[
        str(tmp_path / 'bag'),
        '--profile',
        'rko_lio_graph_public_path',
        '--output-dir',
        str(tmp_path / 'output'),
        '--min-free-space-gib',
        '5.0',
        '--verification',
        'required',
        '--yes',
        '--dry-run',
        '--editable',
    ]]
