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

"""Tests for the supported source dependency bootstrap helper."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'install_source_dependencies.sh'


def _run(
    *args: str,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    script: Path = SCRIPT,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(script), *args],
        check=False,
        capture_output=True,
        cwd=cwd,
        text=True,
        env=env,
    )


def _write_fake_command(
    path: Path,
    *,
    execute_arguments: bool = False,
    exit_code: int = 0,
) -> None:
    body = """#!/usr/bin/env bash
{
  printf '%s' "$(basename "$0")"
  printf ' %s' "$@"
  printf '\\n'
} >>"$CALL_LOG"
"""
    if execute_arguments:
        body += 'exec "$@"\n'
    elif exit_code:
        body += f'exit {exit_code}\n'
    path.write_text(body, encoding='utf-8')
    path.chmod(0o755)


def test_help_and_fail_closed_entrypoints(tmp_path):
    """Help must work standalone and invalid entrypoints must stop early."""
    help_result = _run('--help')
    assert help_result.returncode == 0
    assert '--workspace <path>' in help_result.stdout
    assert '--repo-only' in help_result.stdout
    assert 'source either /opt/ros/humble/setup.bash' in help_result.stdout

    sourced = subprocess.run(
        ['bash', '-c', 'source "$1"', 'bash', str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert sourced.returncode == 2
    assert 'do not source it' in sourced.stderr

    workspace = tmp_path / 'workspace with spaces'
    (workspace / 'src').mkdir(parents=True)
    env = os.environ.copy()
    env['ROS_DISTRO'] = 'rolling'
    unsupported = _run(cwd=workspace, env=env)
    assert unsupported.returncode == 2
    assert 'expected humble or jazzy' in unsupported.stderr
    assert 'could not find the workspace' not in unsupported.stderr


def test_missing_required_submodules_fail_before_package_operations(tmp_path):
    """A non-recursive clone must get one exact recovery command before APT."""
    workspace = tmp_path / 'workspace with spaces'
    fake_repo = workspace / 'src' / 'lidar_slam_ros2'
    fake_script = fake_repo / 'scripts' / SCRIPT.name
    fake_script.parent.mkdir(parents=True)
    fake_script.write_text(SCRIPT.read_text(encoding='utf-8'), encoding='utf-8')
    env = os.environ.copy()
    env['ROS_DISTRO'] = 'humble'

    result = _run(
        '--workspace',
        str(workspace),
        cwd=workspace,
        env=env,
        script=fake_script,
    )

    assert result.returncode == 2
    assert 'required source submodules are missing' in result.stderr
    assert 'Thirdparty/ndt_omp_ros2/package.xml' in result.stderr
    assert 'Thirdparty/rko_lio/package.xml' in result.stderr
    assert 'git -C ' in result.stderr
    assert 'submodule update --init --recursive' in result.stderr
    assert 'workspace\\ with\\ spaces' in result.stderr
    assert 'apt-get' not in result.stdout
    assert 'rosdep' not in result.stdout


def test_fresh_workspace_runs_init_update_install_and_check(tmp_path):
    """A fresh supported workspace must execute the complete bootstrap."""
    workspace = tmp_path / 'workspace'
    (workspace / 'src').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    call_log = tmp_path / 'calls.log'

    _write_fake_command(fake_bin / 'sudo', execute_arguments=True)
    _write_fake_command(fake_bin / 'rosdep')
    _write_fake_command(fake_bin / 'apt-get')

    env = os.environ.copy()
    env.update({
        'CALL_LOG': str(call_log),
        'HOME': str(tmp_path / 'home'),
        'LIDARSLAM_ROSDEP_SOURCES_FILE': str(
            tmp_path / 'missing-rosdep-sources'
        ),
        'PATH': f'{fake_bin}:{env["PATH"]}',
        'ROS_DISTRO': 'jazzy',
        'ROS_HOME': str(tmp_path / 'ros-home'),
    })

    no_home_env = env.copy()
    no_home_env.pop('HOME')
    no_home_env.pop('ROS_HOME')
    no_home = _run(cwd=workspace, env=no_home_env)
    assert no_home.returncode == 2
    assert 'HOME and ROS_HOME are unset' in no_home.stderr

    # A directory left behind by an interrupted rosdep update is not a valid
    # cache. Only the final index marker proves that metadata is usable.
    (tmp_path / 'ros-home' / 'rosdep' / 'sources.cache').mkdir(parents=True)

    result = _run(cwd=workspace, env=env)

    assert result.returncode == 0, result.stderr
    assert 'Source dependencies are ready for ROS 2 jazzy.' in result.stdout
    calls = call_log.read_text(encoding='utf-8').splitlines()
    cursor = 0
    if os.geteuid() != 0:
        assert calls[cursor] == 'sudo rosdep init'
        cursor += 1
    assert calls[cursor] == 'rosdep init'
    cursor += 1
    assert calls[cursor] == 'rosdep update --rosdistro jazzy'
    cursor += 1
    if os.geteuid() != 0:
        assert calls[cursor].startswith(
            'sudo env DEBIAN_FRONTEND=noninteractive apt-get update'
        )
        cursor += 1
    assert calls[cursor] == 'apt-get update'
    cursor += 1
    assert calls[cursor].startswith(
        f'rosdep install --from-paths {workspace}/src '
        '--ignore-src --rosdistro jazzy'
    )
    assert calls[cursor].endswith(' -r -y')
    cursor += 1
    assert calls[cursor] == (
        f'rosdep check --from-paths {workspace}/src '
        '--ignore-src --rosdistro jazzy'
    )
    assert cursor == len(calls) - 1


def test_complete_cache_and_explicit_workspace_skip_rosdep_bootstrap(tmp_path):
    """A complete cache should be reused with an explicit workspace path."""
    workspace = tmp_path / 'workspace with spaces'
    (workspace / 'src').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    call_log = tmp_path / 'calls.log'
    sources = tmp_path / '20-default.list'
    sources.write_text('yaml test\n', encoding='utf-8')
    ros_home = tmp_path / 'ros-home'
    cache = ros_home / 'rosdep' / 'sources.cache'
    cache.mkdir(parents=True)
    (cache / 'index').write_text('ready\n', encoding='utf-8')

    _write_fake_command(fake_bin / 'sudo', execute_arguments=True)
    _write_fake_command(fake_bin / 'rosdep')
    _write_fake_command(fake_bin / 'apt-get')

    env = os.environ.copy()
    env.update({
        'CALL_LOG': str(call_log),
        'HOME': str(tmp_path / 'home'),
        'LIDARSLAM_ROSDEP_SOURCES_FILE': str(sources),
        'PATH': f'{fake_bin}:{env["PATH"]}',
        'ROS_DISTRO': 'humble',
        'ROS_HOME': str(ros_home),
    })

    result = _run('--workspace', str(workspace), cwd=tmp_path, env=env)

    assert result.returncode == 0, result.stderr
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert all(call != 'rosdep init' for call in calls)
    assert all(not call.startswith('rosdep update') for call in calls)
    assert any(
        call.startswith(f'rosdep install --from-paths {workspace}/src ')
        for call in calls
    )
    assert calls[-1] == (
        f'rosdep check --from-paths {workspace}/src '
        '--ignore-src --rosdistro humble'
    )


def test_repo_only_limits_rosdep_discovery_to_this_checkout(tmp_path):
    """Quickstart mode must not resolve unrelated workspace packages."""
    workspace = tmp_path / 'workspace with spaces'
    (workspace / 'src').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    call_log = tmp_path / 'calls.log'
    sources = tmp_path / '20-default.list'
    sources.write_text('yaml test\n', encoding='utf-8')
    cache = tmp_path / 'ros-home' / 'rosdep' / 'sources.cache'
    cache.mkdir(parents=True)
    (cache / 'index').write_text('ready\n', encoding='utf-8')

    _write_fake_command(fake_bin / 'sudo', execute_arguments=True)
    _write_fake_command(fake_bin / 'rosdep')
    _write_fake_command(fake_bin / 'apt-get')
    env = os.environ.copy()
    env.update({
        'CALL_LOG': str(call_log),
        'LIDARSLAM_ROSDEP_SOURCES_FILE': str(sources),
        'PATH': f'{fake_bin}:{env["PATH"]}',
        'ROS_DISTRO': 'jazzy',
        'ROS_HOME': str(tmp_path / 'ros-home'),
    })

    result = _run(
        '--workspace', str(workspace), '--repo-only', cwd=tmp_path, env=env
    )

    assert result.returncode == 0, result.stderr
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert any(
        call.startswith(f'rosdep install --from-paths {REPO_ROOT} ')
        for call in calls
    )
    assert calls[-1] == (
        f'rosdep check --from-paths {REPO_ROOT} '
        '--ignore-src --rosdistro jazzy'
    )
    assert all(f'--from-paths {workspace}/src ' not in call for call in calls)


def test_apt_update_failure_stops_before_dependency_install(tmp_path):
    """An APT metadata failure must not fall through to rosdep install."""
    workspace = tmp_path / 'workspace'
    (workspace / 'src').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    call_log = tmp_path / 'calls.log'
    sources = tmp_path / '20-default.list'
    sources.write_text('yaml test\n', encoding='utf-8')
    cache = tmp_path / 'ros-home' / 'rosdep' / 'sources.cache'
    cache.mkdir(parents=True)
    (cache / 'index').write_text('ready\n', encoding='utf-8')

    _write_fake_command(fake_bin / 'sudo', execute_arguments=True)
    _write_fake_command(fake_bin / 'rosdep')
    _write_fake_command(fake_bin / 'apt-get', exit_code=23)
    env = os.environ.copy()
    env.update({
        'CALL_LOG': str(call_log),
        'LIDARSLAM_ROSDEP_SOURCES_FILE': str(sources),
        'PATH': f'{fake_bin}:{env["PATH"]}',
        'ROS_DISTRO': 'humble',
        'ROS_HOME': str(tmp_path / 'ros-home'),
    })

    result = _run('--workspace', str(workspace), cwd=tmp_path, env=env)

    assert result.returncode == 23
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert any('apt-get update' in call for call in calls)
    assert all(not call.startswith('rosdep install') for call in calls)
    assert 'Source dependencies are ready' not in result.stdout


def test_final_rosdep_check_failure_never_prints_success(tmp_path):
    """A failed post-install check must preserve failure and suppress success."""
    workspace = tmp_path / 'workspace'
    (workspace / 'src').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    call_log = tmp_path / 'calls.log'
    sources = tmp_path / '20-default.list'
    sources.write_text('yaml test\n', encoding='utf-8')
    cache = tmp_path / 'ros-home' / 'rosdep' / 'sources.cache'
    cache.mkdir(parents=True)
    (cache / 'index').write_text('ready\n', encoding='utf-8')

    _write_fake_command(fake_bin / 'sudo', execute_arguments=True)
    _write_fake_command(fake_bin / 'apt-get')
    rosdep = fake_bin / 'rosdep'
    rosdep.write_text(
        """#!/usr/bin/env bash
printf 'rosdep %s\\n' "$*" >>"$CALL_LOG"
if [[ "${1:-}" == check ]]; then
  exit 17
fi
""",
        encoding='utf-8',
    )
    rosdep.chmod(0o755)
    env = os.environ.copy()
    env.update({
        'CALL_LOG': str(call_log),
        'LIDARSLAM_ROSDEP_SOURCES_FILE': str(sources),
        'PATH': f'{fake_bin}:{env["PATH"]}',
        'ROS_DISTRO': 'jazzy',
        'ROS_HOME': str(tmp_path / 'ros-home'),
    })

    result = _run('--workspace', str(workspace), cwd=tmp_path, env=env)

    assert result.returncode == 17
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert calls[-1].startswith('rosdep check --from-paths ')
    assert 'Source dependencies are ready' not in result.stdout
