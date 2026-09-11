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

"""CLI regression tests for the local default CI wrapper."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CI_SCRIPT = REPO_ROOT / 'scripts' / 'run_default_ci_checks.sh'
BASH = Path(shutil.which('bash') or '/usr/bin/bash')


def _run_ci(
    *args: str,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(BASH), str(CI_SCRIPT), *args],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )


def _combined_output(result: subprocess.CompletedProcess[str]) -> str:
    return result.stdout + result.stderr


def _write_executable(path: Path, text: str) -> None:
    path.write_text(text, encoding='utf-8')
    path.chmod(0o755)


def _minimal_path(tmp_path: Path) -> Path:
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    dirname = shutil.which('dirname')
    assert dirname is not None
    os.symlink(dirname, bin_dir / 'dirname')
    return bin_dir


def test_default_ci_help_exits_successfully():
    result = _run_ci('--help')

    assert result.returncode == 0
    assert 'run_default_ci_checks.sh' in result.stderr
    assert '--build-only' in result.stderr


def test_default_ci_fetches_rko_dependencies_for_clean_local_builds():
    assert '-DRKO_LIO_FETCH_CONTENT_DEPS=ON' in CI_SCRIPT.read_text()


def test_default_ci_rejects_missing_cmake_build_type_value():
    result = _run_ci('--cmake-build-type', '--build-only')

    assert result.returncode == 2
    assert 'error: option requires a value: --cmake-build-type' in result.stderr
    assert 'required command not found' not in result.stderr


def test_default_ci_rejects_unknown_option_with_help_hint():
    result = _run_ci('--bogus')

    assert result.returncode == 2
    assert 'error: unknown option: --bogus' in result.stderr
    assert 'run_default_ci_checks.sh --help' in result.stderr
    assert 'Building default workflow packages' not in _combined_output(result)


def test_default_ci_reports_missing_colcon_with_install_hint(tmp_path: Path):
    bin_dir = _minimal_path(tmp_path)
    env = {**os.environ, 'PATH': str(bin_dir)}

    result = _run_ci('--build-only', env=env)

    assert result.returncode == 2
    assert 'error: required command not found: colcon' in result.stderr
    assert 'python3-colcon-common-extensions' in result.stderr


def test_default_ci_wraps_colcon_build_failure(tmp_path: Path):
    bin_dir = _minimal_path(tmp_path)
    _write_executable(
        bin_dir / 'colcon',
        f'#!{BASH}\n'
        'echo "fake colcon build failure" >&2\n'
        'exit 7\n',
    )
    _write_executable(
        bin_dir / 'ros2',
        f'#!{BASH}\n'
        'exit 0\n',
    )
    env = {**os.environ, 'PATH': str(bin_dir)}

    result = _run_ci('--build-only', env=env)

    assert result.returncode == 1
    assert 'fake colcon build failure' in result.stderr
    assert 'error: colcon build failed for default workflow packages' in result.stderr


def test_every_pytest_file_is_registered_with_ctest():
    missing = []
    for package in ('graph_based_slam', 'lidarslam'):
        package_dir = REPO_ROOT / package
        cmake = (package_dir / 'CMakeLists.txt').read_text(encoding='utf-8')
        registered = {
            match.group(1)
            for match in re.finditer(
                r'ament_add_pytest_test\(\s*(test_[a-z0-9_]+)'
                r'\s+test/\1\.py',
                cmake,
            )
        }
        for match in re.finditer(
            r'set\([A-Z0-9_]+_ADDITIONAL_PYTESTS(.*?)\)',
            cmake,
            flags=re.DOTALL,
        ):
            names = re.findall(r'\btest_[a-z0-9_]+\b', match.group(1))
            registered.update(names)
        for path in sorted((package_dir / 'test').glob('test_*.py')):
            if path.stem not in registered:
                missing.append(f'{package}/test/{path.name}')

    assert missing == []
