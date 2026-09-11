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

"""CLI regression tests for the product Python-suite entrypoint."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
TEST_SCRIPT = REPO_ROOT / 'scripts' / 'run_product_python_tests.sh'
BASH = Path(shutil.which('bash') or '/usr/bin/bash')


def _run(
    *args: str,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(BASH), str(TEST_SCRIPT), *args],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )


def _write_executable(path: Path, text: str) -> None:
    path.write_text(text, encoding='utf-8')
    path.chmod(0o755)


def _fake_python_environment(tmp_path: Path) -> tuple[dict[str, str], Path]:
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    dirname = shutil.which('dirname')
    assert dirname is not None
    os.symlink(dirname, bin_dir / 'dirname')

    call_log = tmp_path / 'python_calls.txt'
    _write_executable(
        bin_dir / 'python3',
        f"""#!{BASH}
set -eu
printf '%s\\n' "$*" >> "$CALL_LOG"
case "$*" in
  "-c import pytest") exit "${{PYTEST_IMPORT_RC:-0}}" ;;
  "-c import rosbag2_py") exit "${{ROSBAG_IMPORT_RC:-0}}" ;;
  "-c import rosbags") exit "${{ROSBAGS_IMPORT_RC:-0}}" ;;
  *graph_based_slam/test*) exit "${{GRAPH_RC:-0}}" ;;
  *lidarslam/test*) exit "${{LIDARSLAM_RC:-0}}" ;;
  *) exit 0 ;;
esac
""",
    )
    env = {
        **os.environ,
        'CALL_LOG': str(call_log),
        'PATH': str(bin_dir),
        'ROS_DISTRO': 'jazzy',
    }
    return env, call_log


def _calls(call_log: Path) -> list[str]:
    return call_log.read_text(encoding='utf-8').splitlines()


def test_help_explains_scoped_separate_process_contract():
    result = _run('--help')

    assert result.returncode == 0
    assert '--suite <name>' in result.stderr
    assert '-- <pytest args>' in result.stderr
    assert 'separate pytest' in result.stderr
    assert 'Thirdparty' in result.stderr
    assert 'Both maintained suites include ROS bag fixtures' in result.stderr


def test_unknown_and_invalid_suite_fail_before_preflight():
    unknown = _run('--bogus')
    invalid = _run('--suite', 'research')

    assert unknown.returncode == 2
    assert 'unknown option: --bogus' in unknown.stderr
    assert invalid.returncode == 2
    assert "unsupported suite 'research'" in invalid.stderr


def test_all_runs_package_directories_in_separate_pytest_processes(tmp_path: Path):
    env, call_log = _fake_python_environment(tmp_path)

    result = _run('--', '-k', 'map_bundle', env=env)

    assert result.returncode == 0, result.stderr
    pytest_calls = [line for line in _calls(call_log) if '-m pytest' in line]
    assert len(pytest_calls) == 2
    assert 'graph_based_slam/test' in pytest_calls[0]
    assert 'lidarslam/test' not in pytest_calls[0]
    assert 'lidarslam/test' in pytest_calls[1]
    assert 'graph_based_slam/test' not in pytest_calls[1]
    assert all('Thirdparty' not in line for line in pytest_calls)
    assert all('-p no:cacheprovider' in line for line in pytest_calls)
    assert all('-k map_bundle' in line for line in pytest_calls)


def test_one_suite_failure_does_not_hide_the_other_result(tmp_path: Path):
    env, call_log = _fake_python_environment(tmp_path)
    env['GRAPH_RC'] = '7'

    result = _run(env=env)

    assert result.returncode == 1
    assert 'Python product suite failed: graph_based_slam' in result.stderr
    pytest_calls = [line for line in _calls(call_log) if '-m pytest' in line]
    assert len(pytest_calls) == 2
    assert 'lidarslam/test' in pytest_calls[1]


def test_lidarslam_suite_reports_missing_ros_python_before_collection(
    tmp_path: Path,
):
    env, call_log = _fake_python_environment(tmp_path)
    env['ROSBAG_IMPORT_RC'] = '9'
    noop_setup = tmp_path / 'noop_setup.bash'
    noop_setup.write_text(':\n', encoding='utf-8')

    result = _run(
        '--suite', 'lidarslam', '--ros-setup', str(noop_setup), env=env,
    )

    assert result.returncode == 2
    assert (
        'rosbag2_py is unavailable for the selected product suite: lidarslam'
        in result.stderr
    )
    calls = _calls(call_log)
    assert '-c import rosbag2_py' in calls
    assert not any('-m pytest' in line for line in calls)


def test_graph_suite_reports_missing_ros_python_with_supported_hint(tmp_path: Path):
    env, call_log = _fake_python_environment(tmp_path)
    env['ROSBAG_IMPORT_RC'] = '9'
    noop_setup = tmp_path / 'noop_setup.bash'
    noop_setup.write_text(':\n', encoding='utf-8')

    result = _run(
        '--suite', 'graph_based_slam', '--ros-setup', str(noop_setup), env=env,
    )

    assert result.returncode == 2
    assert (
        'rosbag2_py is unavailable for the selected product suite: '
        'graph_based_slam' in result.stderr
    )
    assert '/opt/ros/humble/setup.bash' in result.stderr
    assert '/opt/ros/jazzy/setup.bash' in result.stderr
    assert not any('-m pytest' in line for line in _calls(call_log))


def test_graph_suite_rejects_ros_distro_outside_support_contract(tmp_path: Path):
    env, call_log = _fake_python_environment(tmp_path)
    env['ROS_DISTRO'] = 'rolling'

    result = _run('--suite', 'graph_based_slam', env=env)

    assert result.returncode == 2
    assert 'ROS_DISTRO=rolling is outside the Humble/Jazzy support contract' in result.stderr
    assert not any('-m pytest' in line for line in _calls(call_log))


def test_missing_pip_only_graph_dependency_fails_before_collection(tmp_path: Path):
    env, call_log = _fake_python_environment(tmp_path)
    env['ROSBAGS_IMPORT_RC'] = '8'

    result = _run('--suite', 'graph_based_slam', env=env)

    assert result.returncode == 2
    assert 'Python product-test prerequisites unavailable: rosbags' in result.stderr
    assert "python3 -m pip install 'rosbags==0.11.0'" in result.stderr
    assert not any('-m pytest' in line for line in _calls(call_log))


def test_missing_pytest_has_install_hint(tmp_path: Path):
    env, _ = _fake_python_environment(tmp_path)
    env['PYTEST_IMPORT_RC'] = '4'

    result = _run('--suite', 'lidarslam', env=env)

    assert result.returncode == 2
    assert 'pytest is unavailable' in result.stderr
    assert 'python3 -m pip install pytest' in result.stderr


def test_missing_explicit_ros_setup_is_rejected(tmp_path: Path):
    env, _ = _fake_python_environment(tmp_path)
    missing = tmp_path / 'missing_setup.bash'

    result = _run('--ros-setup', str(missing), env=env)

    assert result.returncode == 2
    assert f'ROS setup file does not exist: {missing}' in result.stderr
