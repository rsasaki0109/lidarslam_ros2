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

"""Tests for the one-command source build and first-map quickstart."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'source_quickstart.sh'
PLAN_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / (
    'source-quickstart-plan-v1.schema.json'
)
EXPECTED_PACKAGES = (
    'graph_based_slam',
    'lidarslam',
    'lidarslam_msgs',
    'ndt_omp_ros2',
    'rko_lio',
    'scanmatcher',
)


def _bash_q(path: Path) -> str:
    return str(path).replace(' ', '\\ ')


def _write_executable(path: Path, body: str) -> None:
    path.write_text('#!/usr/bin/env bash\n' + body, encoding='utf-8')
    path.chmod(0o755)


def _fixture(
    tmp_path: Path,
    *,
    colcon_exit: int = 0,
    include_submodules: bool = True,
    package_names: tuple[str, ...] = EXPECTED_PACKAGES,
) -> tuple[Path, Path, dict[str, str], Path]:
    workspace = tmp_path / 'workspace with spaces'
    fake_repo = workspace / 'src' / 'lidar_slam_ros2'
    scripts = fake_repo / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(SCRIPT, scripts / SCRIPT.name)
    if include_submodules:
        for relative in (
            'Thirdparty/ndt_omp_ros2/package.xml',
            'Thirdparty/rko_lio/package.xml',
        ):
            source_file = fake_repo / relative
            source_file.parent.mkdir(parents=True, exist_ok=True)
            source_file.write_text('<package/>\n', encoding='utf-8')

    call_log = tmp_path / 'calls.log'
    dependency_helper = scripts / 'install_source_dependencies.sh'
    _write_executable(
        dependency_helper,
        'printf \'dependencies %s\\n\' "$*" >>"$CALL_LOG"\n',
    )

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _write_executable(
        fake_bin / 'git',
        (
            'printf \'git %s\\n\' "$*" >>"$CALL_LOG"\n'
            'if [[ "${1:-}" == -C && "${3:-}" == submodule ]]; then\n'
            '  mkdir -p "$2/Thirdparty/ndt_omp_ros2" '
            '"$2/Thirdparty/rko_lio"\n'
            "  printf '<package/>\\n' "
            '>"$2/Thirdparty/ndt_omp_ros2/package.xml"\n'
            "  printf '<package/>\\n' "
            '>"$2/Thirdparty/rko_lio/package.xml"\n'
            'fi\n'
        ),
    )
    _write_executable(fake_bin / 'rosdep', 'exit 0\n')
    _write_executable(
        fake_bin / 'colcon',
        (
            'printf \'colcon %s\\n\' "$*" >>"$CALL_LOG"\n'
            'if [[ "${1:-}" == list ]]; then\n'
            + ''.join(
                f"  printf '{name}\\n'\n" for name in package_names
            )
            + '  exit 0\n'
            'fi\n'
            f'if [[ {colcon_exit} -ne 0 ]]; then exit {colcon_exit}; fi\n'
            'mkdir -p "$PWD/install/lidarslam/bin"\n'
            'cp "$FAKE_LIDARSLAM_MAP" '
            '"$PWD/install/lidarslam/bin/lidarslam-map"\n'
            ': "${COLCON_TRACE}"\n'
            "printf 'export QUICKSTART_TEST_INSTALL=1\\n"
            "export PATH=\"%s:$PATH\"\\n' "
            '"$PWD/install/lidarslam/bin" '
            '>"$PWD/install/setup.bash"\n'
        ),
    )
    _write_executable(
        fake_bin / 'lidarslam-map',
        'printf \'lidarslam-map %s\\n\' "$*" >>"$CALL_LOG"\n',
    )

    ros_root = tmp_path / 'opt ros'
    setup = ros_root / 'jazzy' / 'setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text(
        'if [[ -z "${AMENT_TRACE_SETUP_FILES}" ]]; then :; fi\n'
        'export ROS_DISTRO=jazzy\n',
        encoding='utf-8',
    )
    os_release = tmp_path / 'os-release'
    os_release.write_text('VERSION_ID="24.04"\n', encoding='utf-8')

    env = os.environ.copy()
    env.pop('ROS_DISTRO', None)
    env.pop('AMENT_TRACE_SETUP_FILES', None)
    env.pop('COLCON_TRACE', None)
    env.update({
        'CALL_LOG': str(call_log),
        'LIDARSLAM_OS_RELEASE_FILE': str(os_release),
        'LIDARSLAM_ROS_PREFIX_ROOT': str(ros_root),
        'FAKE_LIDARSLAM_MAP': str(fake_bin / 'lidarslam-map'),
        'PATH': f'{fake_bin}:{env["PATH"]}',
    })
    return workspace, fake_repo, env, call_log


def _run(
    script: Path,
    *args: str,
    cwd: Path,
    env: dict[str, str],
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(script), *args],
        check=False,
        capture_output=True,
        cwd=cwd,
        env=env,
        text=True,
    )


def test_help_and_sourced_entrypoint_fail_closed():
    help_result = subprocess.run(
        ['bash', str(SCRIPT), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0
    assert '--build-only' in help_result.stdout
    assert '--dry-run' in help_result.stdout
    assert '--json' in help_result.stdout
    assert 'does not' in help_result.stdout
    assert 'install a ROS distribution' in help_result.stdout

    sourced = subprocess.run(
        ['bash', '-c', 'source "$1"', 'bash', str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert sourced.returncode == 2
    assert 'do not source it' in sourced.stderr


def test_dry_run_is_read_only_and_prints_repo_scoped_plan(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name
    before = sorted(str(path.relative_to(workspace)) for path in workspace.rglob('*'))

    result = _run(script, '--viewer', 'none', '--dry-run', cwd=fake_repo, env=env)

    assert result.returncode == 0, result.stderr
    assert 'Build scope: this repository only (6 ROS packages)' in result.stdout
    assert 'Commands (--dry-run; nothing executed)' in result.stdout
    assert '--repo-only' in result.stdout
    assert f'--base-paths {_bash_q(fake_repo)}' in result.stdout
    assert '--packages-select ' + ' '.join(EXPECTED_PACKAGES) in result.stdout
    assert 'lidarslam-map demo' in result.stdout
    assert '--viewer none' in result.stdout
    assert not call_log.exists()
    after = sorted(str(path.relative_to(workspace)) for path in workspace.rglob('*'))
    assert after == before


def test_json_dry_run_is_schema_valid_and_read_only(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name
    before = sorted(str(path.relative_to(workspace)) for path in workspace.rglob('*'))

    result = _run(
        script,
        '--viewer', 'none',
        '--dry-run',
        '--json',
        cwd=fake_repo,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert result.stderr == ''
    plan = json.loads(result.stdout)
    schema = json.loads(PLAN_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(plan)
    assert plan['repository']['path'] == str(fake_repo.resolve())
    assert plan['repository']['packages'] == list(EXPECTED_PACKAGES)
    assert plan['workspace'] == {
        'path': str(workspace.resolve()),
        'build_path': str(workspace.resolve() / 'build'),
        'install_path': str(workspace.resolve() / 'install'),
        'log_path': str(workspace.resolve() / 'log'),
        'demo_root': str(workspace.resolve()),
    }
    assert plan['ros'] == {
        'distribution': 'jazzy',
        'setup_path': str(
            Path(env['LIDARSLAM_ROS_PREFIX_ROOT']) / 'jazzy' / 'setup.bash'
        ),
    }
    assert plan['options'] == {'build_only': False, 'viewer': 'none'}
    assert plan['preflight']['submodules']['missing'] == []
    assert plan['preflight']['tools']['missing'] == []
    assert plan['preflight']['package_inventory']['verified_during_live_run'] is False
    assert plan['planned_actions']['run_fixed_demo'] is True
    assert plan['commands'][-1] == [
        'lidarslam-map', 'demo', str(workspace.resolve()), '--viewer', 'none'
    ]
    assert plan['side_effects'] == {
        'network_accessed': False,
        'apt_executed': False,
        'submodule_checkout': False,
        'workspace_build_executed': False,
        'demo_executed': False,
        'filesystem_writes': False,
    }
    assert not call_log.exists()
    after = sorted(str(path.relative_to(workspace)) for path in workspace.rglob('*'))
    assert after == before


def test_json_build_only_reports_missing_bootstrap_without_writes(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(
        tmp_path,
        include_submodules=False,
    )
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(
        script,
        '--build-only',
        '--dry-run',
        '--json',
        cwd=fake_repo,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    schema = json.loads(PLAN_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(plan)
    assert plan['preflight']['submodules']['missing'] == [
        'Thirdparty/ndt_omp_ros2/package.xml',
        'Thirdparty/rko_lio/package.xml',
    ]
    assert plan['planned_actions']['initialize_submodules'] is True
    assert plan['planned_actions']['install_tools'] is False
    assert plan['planned_actions']['run_fixed_demo'] is False
    assert plan['commands'][0] == [
        'git', '-C', str(fake_repo.resolve()),
        'submodule', 'update', '--init', '--recursive',
    ]
    assert all(command[0] != 'lidarslam-map' for command in plan['commands'])
    assert not call_log.exists()
    assert not (fake_repo / 'Thirdparty/rko_lio/package.xml').exists()


def test_json_requires_dry_run(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(script, '--json', cwd=fake_repo, env=env)

    assert result.returncode == 2
    assert '--json requires --dry-run' in result.stderr
    assert not call_log.exists()


def test_default_run_builds_repo_then_runs_verified_demo(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(script, '--viewer', 'none', cwd=fake_repo, env=env)

    assert result.returncode == 0, result.stderr
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert calls[0] == f'colcon list --base-paths {fake_repo} --names-only'
    assert calls[1] == f'dependencies --workspace {workspace} --repo-only'
    assert calls[2] == (
        f'colcon build --base-paths {fake_repo} --packages-select '
        f'{" ".join(EXPECTED_PACKAGES)} --symlink-install '
        '--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF'
    )
    assert calls[3] == f'lidarslam-map demo {workspace} --viewer none'
    assert 'exact maintained inventory confirmed' in result.stdout
    assert 'Source quickstart: COMPLETE' in result.stdout
    product_command = workspace / 'install/lidarslam/bin/lidarslam-map'
    assert 'no activation step' in result.stdout
    assert _bash_q(product_command) in result.stdout
    assert f'source {_bash_q(workspace)}/install/setup.bash' in result.stdout
    assert f'{_bash_q(product_command)} start /path/to/rosbag2' in result.stdout


def test_build_only_skips_demo_and_prints_manual_next_step(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(script, '--build-only', cwd=fake_repo, env=env)

    assert result.returncode == 0, result.stderr
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert len(calls) == 3
    assert all(not call.startswith('lidarslam-map ') for call in calls)
    assert 'Finish: build only' in result.stdout
    assert 'Try the fixed public demo:' in result.stdout
    product_command = workspace / 'install/lidarslam/bin/lidarslam-map'
    assert f'{_bash_q(product_command)} demo {_bash_q(workspace)}' in result.stdout


def test_missing_pinned_submodules_are_planned_then_initialized(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(
        tmp_path, include_submodules=False
    )
    script = fake_repo / 'scripts' / SCRIPT.name

    plan = _run(script, '--build-only', '--dry-run', cwd=fake_repo, env=env)
    assert plan.returncode == 0, plan.stderr
    assert 'source: initialize pinned git submodules' in plan.stdout
    assert 'git -C' in plan.stdout
    assert not call_log.exists()
    assert not (fake_repo / 'Thirdparty/rko_lio/package.xml').exists()

    result = _run(script, '--build-only', cwd=fake_repo, env=env)
    assert result.returncode == 0, result.stderr
    calls = call_log.read_text(encoding='utf-8').splitlines()
    assert calls[0] == f'git -C {fake_repo} submodule update --init --recursive'
    assert calls[1] == f'colcon list --base-paths {fake_repo} --names-only'
    assert calls[2] == f'dependencies --workspace {workspace} --repo-only'
    assert (fake_repo / 'Thirdparty/ndt_omp_ros2/package.xml').is_file()
    assert (fake_repo / 'Thirdparty/rko_lio/package.xml').is_file()


def test_build_failure_preserves_code_and_prints_idempotent_retry(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path, colcon_exit=19)
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(script, '--viewer', 'none', cwd=fake_repo, env=env)

    assert result.returncode == 19
    assert 'stopped during workspace build (exit 19)' in result.stderr
    assert 'retry: bash' in result.stderr
    assert f'--workspace {_bash_q(workspace)}' in result.stderr
    assert '--ros-distro jazzy' in result.stderr
    assert '--viewer none' in result.stderr
    assert all(
        not call.startswith('lidarslam-map ')
        for call in call_log.read_text(encoding='utf-8').splitlines()
    )


def test_unexpected_source_package_fails_before_dependencies_or_build(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(
        tmp_path,
        package_names=EXPECTED_PACKAGES + ('research_only_package',),
    )
    script = fake_repo / 'scripts' / SCRIPT.name

    result = _run(script, '--build-only', cwd=fake_repo, env=env)

    assert result.returncode == 2
    assert '[source-package-inventory-mismatch]' in result.stderr
    assert 'research_only_package' in result.stderr
    assert call_log.read_text(encoding='utf-8').splitlines() == [
        f'colcon list --base-paths {fake_repo} --names-only'
    ]
    assert not (workspace / 'install').exists()


def test_missing_ros_install_rejects_before_any_command(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name
    env['LIDARSLAM_ROS_PREFIX_ROOT'] = str(tmp_path / 'missing ros')

    result = _run(script, '--dry-run', cwd=fake_repo, env=env)

    assert result.returncode == 2
    assert 'ROS 2 jazzy is not installed' in result.stderr
    assert 'next: install ROS 2 jazzy' in result.stderr
    assert not call_log.exists()


def test_os_release_is_parsed_as_data_not_sourced(tmp_path):
    workspace, fake_repo, env, call_log = _fixture(tmp_path)
    script = fake_repo / 'scripts' / SCRIPT.name
    marker = tmp_path / 'must-not-exist'
    hostile_os_release = tmp_path / 'hostile-os-release'
    hostile_os_release.write_text(
        f'VERSION_ID="$(touch {marker})"\n', encoding='utf-8'
    )
    env['LIDARSLAM_OS_RELEASE_FILE'] = str(hostile_os_release)

    result = _run(script, '--dry-run', cwd=fake_repo, env=env)

    assert result.returncode == 2
    assert 'could not select ROS 2' in result.stderr
    assert not marker.exists()
    assert not call_log.exists()
