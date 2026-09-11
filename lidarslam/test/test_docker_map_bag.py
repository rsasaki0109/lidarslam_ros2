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

"""Tests for the one-command Docker own-bag host launcher."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'docker_map_bag.sh'
PLAN_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / (
    'docker-map-bag-plan-v1.schema.json'
)


def _bag(tmp_path: Path, name: str = 'field bag') -> Path:
    bag = tmp_path / name
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    return bag


def _docker_environment(
    tmp_path: Path,
    *,
    info_exit: int = 0,
    run_exit: int = 0,
    create_artifacts: bool = True,
    contract_exit: int = 0,
) -> tuple[dict[str, str], Path, Path]:
    stub_dir = tmp_path / 'bin'
    stub_dir.mkdir()
    info_capture = tmp_path / 'docker-info-called'
    run_capture = tmp_path / 'docker-run-args.txt'
    docker = stub_dir / 'docker'
    docker.write_text(
        '#!/usr/bin/env bash\n'
        'set -eu\n'
        'case "${1:-}" in\n'
        '  info)\n'
        '    : > "$DOCKER_TEST_INFO_CAPTURE"\n'
        '    exit "$DOCKER_TEST_INFO_EXIT"\n'
        '    ;;\n'
        '  run)\n'
        '    if [[ "${*: -1}" == --help ]]; then\n'
        '      exit "$DOCKER_TEST_CONTRACT_EXIT"\n'
        '    fi\n'
        '    printf "%s\\n" "$@" > "$DOCKER_TEST_RUN_CAPTURE"\n'
        '    if [[ "$DOCKER_TEST_CREATE_ARTIFACTS" == 1 ]]; then\n'
        '      output=\n'
        '      for argument in "$@"; do\n'
        '        case "$argument" in\n'
        '          type=bind,src=*,dst=/output)\n'
        '            output=${argument#type=bind,src=}\n'
        '            output=${output%,dst=/output}\n'
        '            ;;\n'
        '        esac\n'
        '      done\n'
        '      test -n "$output"\n'
        '      mkdir -p "$output/setup" "$output/map"\n'
        '      : > "$output/setup/session.json"\n'
        '      : > "$output/setup/session.html"\n'
        '      : > "$output/map/run_manifest.json"\n'
        '      : > "$output/map/first_map_validation_receipt.json"\n'
        '    fi\n'
        '    exit "$DOCKER_TEST_RUN_EXIT"\n'
        '    ;;\n'
        '  image)\n'
        '    [[ "${2:-}" == inspect ]] || exit 97\n'
        '    printf "%s\\n" "$DOCKER_TEST_IMAGE_ID"\n'
        '    ;;\n'
        '  *) exit 98 ;;\n'
        'esac\n',
        encoding='utf-8',
    )
    docker.chmod(0o755)
    env = os.environ.copy()
    env.update({
        'DOCKER_TEST_INFO_CAPTURE': str(info_capture),
        'DOCKER_TEST_INFO_EXIT': str(info_exit),
        'DOCKER_TEST_RUN_CAPTURE': str(run_capture),
        'DOCKER_TEST_RUN_EXIT': str(run_exit),
        'DOCKER_TEST_CREATE_ARTIFACTS': '1' if create_artifacts else '0',
        'DOCKER_TEST_CONTRACT_EXIT': str(contract_exit),
        'DOCKER_TEST_IMAGE_ID': 'sha256:' + 'b' * 64,
        'PATH': f'{stub_dir}:/usr/bin:/bin',
    })
    return env, info_capture, run_capture


def _run(
    tmp_path: Path,
    arguments: list[str],
    *,
    info_exit: int = 0,
    run_exit: int = 0,
    create_artifacts: bool = True,
    contract_exit: int = 0,
) -> tuple[subprocess.CompletedProcess[str], Path, Path]:
    env, info_capture, run_capture = _docker_environment(
        tmp_path,
        info_exit=info_exit,
        run_exit=run_exit,
        create_artifacts=create_artifacts,
        contract_exit=contract_exit,
    )
    result = subprocess.run(
        ['/bin/bash', str(SCRIPT), *arguments],
        cwd=tmp_path,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )
    return result, info_capture, run_capture


def test_help_is_beginner_facing_and_script_must_not_be_sourced(tmp_path):
    result = subprocess.run(
        ['/bin/bash', str(SCRIPT), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert 'ROSBAG2_DIR [-- START_OPTIONS...]' in result.stdout
    assert 'bag is mounted read-only' in result.stdout
    assert 'lidarslam-map start' in result.stdout
    assert '--dry-run' in result.stdout
    assert '--json' in result.stdout
    assert '--version' in result.stdout

    version = subprocess.run(
        ['/bin/bash', str(SCRIPT), '--version'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert version.returncode == 0
    assert version.stdout == (
        'lidarslam-map-docker development (working-tree)\n'
    )

    sourced = subprocess.run(
        ['/bin/bash', '-c', f'source {SCRIPT!s}'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert sourced.returncode == 2
    assert 'do not source it' in sourced.stderr


def test_dry_run_does_not_call_docker_or_create_output(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'new output'

    result, info_capture, run_capture = _run(
        tmp_path,
        ['--output-dir', str(output), '--dry-run', str(bag)],
    )

    assert result.returncode == 0, result.stderr
    assert 'Dry run: complete' in result.stdout
    assert str(bag.resolve()) in result.stdout
    assert str(output.resolve()) in result.stdout
    assert 'readonly' in result.stdout
    assert 'lidarslam-map start /input' in result.stdout
    assert not info_capture.exists()
    assert not run_capture.exists()
    assert not output.exists()


def test_json_dry_run_is_schema_valid_and_read_only(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'json output'

    result, info_capture, run_capture = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            '--dry-run',
            '--json',
            str(bag),
            '--',
            '--yes',
            '--editable',
        ],
    )

    assert result.returncode == 0, result.stderr
    assert result.stderr == ''
    plan = json.loads(result.stdout)
    schema = json.loads(PLAN_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(plan)
    assert plan['input']['path'] == str(bag.resolve())
    assert plan['output']['path'] == str(output.resolve())
    assert plan['output']['status'] == 'absent'
    assert plan['route']['start_args'] == ['--yes', '--editable']
    assert plan['execution']['confirmation'] == 'reviewed_yes'
    assert plan['image']['resolved_id'] is None
    assert plan['image']['resolution'] == 'deferred_until_live_run'
    assert plan['side_effects'] == {
        'docker_called': False,
        'network_accessed': False,
        'filesystem_writes': False,
        'input_mount_read_only': True,
    }
    assert not info_capture.exists()
    assert not run_capture.exists()
    assert not output.exists()


def test_json_requires_dry_run(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, info_capture, run_capture = _run(
        tmp_path,
        ['--output-dir', str(output), '--json', str(bag)],
    )

    assert result.returncode == 2
    assert '[json-requires-dry-run]' in result.stderr
    assert '--dry-run' in result.stderr
    assert not info_capture.exists()
    assert not run_capture.exists()
    assert not output.exists()


def test_live_route_mounts_input_read_only_and_uses_high_level_start(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'mapped output'

    result, info_capture, run_capture = _run(
        tmp_path,
        [
            '--ros-distro', 'jazzy',
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
            '--accept-profile-extrinsics',
            '--editable',
        ],
    )

    assert result.returncode == 0, result.stderr
    assert info_capture.is_file()
    assert output.is_dir()
    arguments = run_capture.read_text(encoding='utf-8').splitlines()
    assert arguments[0] == 'run'
    assert '--rm' in arguments
    assert '--pull=never' in arguments
    network_index = arguments.index('--network')
    assert arguments[network_index + 1] == 'none'
    assert arguments[arguments.index('--user') + 1] == (
        f'{os.getuid()}:{os.getgid()}'
    )
    mounts = [
        arguments[index + 1]
        for index, value in enumerate(arguments)
        if value == '--mount'
    ]
    assert (
        f'type=bind,src={bag.resolve()},dst=/input,readonly' in mounts
    )
    assert f'type=bind,src={output.resolve()},dst=/output' in mounts
    assert all('/var/run/docker.sock' not in value for value in arguments)
    image_index = arguments.index(
        'sha256:' + 'b' * 64
    )
    assert arguments[image_index + 1:image_index + 4] == [
        'lidarslam-map', 'start', '/input'
    ]
    assert arguments[arguments.index('--output-dir') + 1] == '/output/setup'
    assert arguments[arguments.index('--map-output-dir') + 1] == '/output/map'
    assert '--yes' in arguments
    assert '--accept-profile-extrinsics' in arguments
    assert '--editable' in arguments
    assert arguments[-2:] == ['--viewer', 'none']
    assert 'Docker own-bag map: COMPLETE' in result.stdout
    assert 'Resolved image ID: sha256:' in result.stdout
    assert f'{output.resolve()}/setup/session.html' in result.stdout


def test_noninteractive_live_run_requires_explicit_reviewed_yes(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, info_capture, run_capture = _run(
        tmp_path,
        ['--output-dir', str(output), str(bag)],
    )

    assert result.returncode == 2
    assert '[confirmation-required]' in result.stderr
    assert '--dry-run' in result.stderr
    assert not info_capture.exists()
    assert not run_capture.exists()
    assert not output.exists()


def test_invalid_bag_is_rejected_before_docker_or_writes(tmp_path):
    bag = tmp_path / 'not-a-bag'
    bag.mkdir()
    output = tmp_path / 'output'

    result, info_capture, _ = _run(
        tmp_path,
        ['--output-dir', str(output), '--dry-run', str(bag)],
    )

    assert result.returncode == 2
    assert '[metadata-missing]' in result.stderr
    assert 'regular metadata.yaml' in result.stderr
    assert not info_capture.exists()
    assert not output.exists()


def test_metadata_and_output_symlinks_are_rejected(tmp_path):
    metadata = tmp_path / 'shared-metadata.yaml'
    metadata.write_text('rosbag2_bagfile_information: {}\n', encoding='utf-8')
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').symlink_to(metadata)

    metadata_result, info_capture, _ = _run(
        tmp_path,
        ['--dry-run', str(bag)],
    )
    assert metadata_result.returncode == 2
    assert '[metadata-missing]' in metadata_result.stderr
    assert not info_capture.exists()

    (bag / 'metadata.yaml').unlink()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    target = tmp_path / 'real-output'
    target.mkdir()
    output_link = tmp_path / 'output-link'
    output_link.symlink_to(target, target_is_directory=True)
    second = tmp_path / 'second'
    second.mkdir()
    output_result, second_info, _ = _run(
        second,
        ['--output-dir', str(output_link), '--dry-run', str(bag)],
    )
    assert output_result.returncode == 2
    assert '[output-symlink]' in output_result.stderr
    assert not second_info.exists()


def test_existing_output_and_input_overlap_are_fail_closed(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'
    output.mkdir()
    (output / 'existing-map.pcd').write_text('keep\n', encoding='utf-8')

    occupied, info_capture, _ = _run(
        tmp_path,
        ['--output-dir', str(output), '--dry-run', str(bag)],
    )
    assert occupied.returncode == 2
    assert '[output-not-empty]' in occupied.stderr
    assert (output / 'existing-map.pcd').read_text(encoding='utf-8') == 'keep\n'
    assert not info_capture.exists()

    second = tmp_path / 'second'
    second.mkdir()
    overlap, _, _ = _run(
        second,
        ['--output-dir', str(bag / 'map'), '--dry-run', str(bag)],
    )
    assert overlap.returncode == 2
    assert '[path-overlap]' in overlap.stderr
    assert not (bag / 'map').exists()


def test_launcher_owned_container_paths_cannot_be_overridden(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, info_capture, _ = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            '--dry-run',
            str(bag),
            '--',
            '--map-output-dir=/tmp/escape',
        ],
    )

    assert result.returncode == 2
    assert '[managed-option]' in result.stderr
    assert not info_capture.exists()
    assert not output.exists()


def test_verification_and_dry_run_cannot_be_weakened_inside_container(tmp_path):
    bag = _bag(tmp_path)

    for forbidden in ('--verification=off', '--dry-run', '--json', '--help'):
        case_root = tmp_path / forbidden.removeprefix('--').replace('=', '-')
        case_root.mkdir()
        case_bag = _bag(case_root)
        result, info_capture, _ = _run(
            case_root,
            [
                '--output-dir', str(case_root / 'output'),
                '--dry-run',
                str(case_bag),
                '--',
                forbidden,
            ],
        )
        assert result.returncode == 2
        assert '[managed-option]' in result.stderr
        assert not info_capture.exists()

    assert bag.is_dir()


def test_daemon_failure_does_not_create_output(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, info_capture, run_capture = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
        ],
        info_exit=1,
    )

    assert result.returncode == 70
    assert '[docker-unavailable]' in result.stderr
    assert info_capture.is_file()
    assert not run_capture.exists()
    assert not output.exists()


def test_old_image_contract_is_rejected_before_output_creation(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, info_capture, run_capture = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
        ],
        contract_exit=2,
    )

    assert result.returncode == 70
    assert '[image-contract-missing]' in result.stderr
    assert 'reviewed revision' in result.stderr
    assert info_capture.is_file()
    assert not run_capture.exists()
    assert not output.exists()


def test_container_failure_is_propagated_and_output_is_retained(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, _, run_capture = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
        ],
        run_exit=23,
        create_artifacts=False,
    )

    assert result.returncode == 23
    assert '[docker-map-failed]' in result.stderr
    assert f'Output retained: {output.resolve()}' in result.stderr
    assert 'review the container diagnosis above' in result.stderr
    assert 'setup/session.html, then rerun' not in result.stderr
    assert output.is_dir()
    assert run_capture.is_file()


def test_container_failure_links_a_retained_session_when_available(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, _, _ = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
        ],
        run_exit=24,
    )

    assert result.returncode == 24
    assert f'review {output.resolve()}/setup/session.html' in result.stderr
    assert 'review the container diagnosis above' not in result.stderr


def test_zero_exit_without_verified_artifacts_is_not_reported_complete(tmp_path):
    bag = _bag(tmp_path)
    output = tmp_path / 'output'

    result, _, run_capture = _run(
        tmp_path,
        [
            '--output-dir', str(output),
            str(bag),
            '--',
            '--yes',
        ],
        create_artifacts=False,
    )

    assert result.returncode == 1
    assert '[docker-result-missing]' in result.stderr
    assert 'setup/session.json' in result.stderr
    assert 'map/first_map_validation_receipt.json' in result.stderr
    assert 'Docker own-bag map: COMPLETE' not in result.stdout
    assert output.is_dir()
    assert run_capture.is_file()


def test_custom_immutable_image_is_preserved_in_plan(tmp_path):
    bag = _bag(tmp_path)
    image = (
        'ghcr.io/rsasaki0109/lidar_slam_ros2:humble@sha256:'
        + 'a' * 64
    )

    result, info_capture, _ = _run(
        tmp_path,
        ['--image', image, '--dry-run', str(bag)],
    )

    assert result.returncode == 0, result.stderr
    assert image in result.stdout
    assert not info_capture.exists()
