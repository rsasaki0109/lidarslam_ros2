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

"""Regression tests for the one-command Docker demo."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
FIRST_MAP_SCRIPT = REPO_ROOT / 'scripts' / 'run_first_map_demo.sh'
DOCKER_SCRIPT = REPO_ROOT / 'scripts' / 'run_docker_demo.sh'
BAG_NAME = 'rosbag2_2024_04_16-14_17_01'
ARCHIVE_SHA256 = (
    'f8f89eebf2aaf9cc1d465bfa5451bbb5'
    '99cd92d079b59949104bb4e5cb619bdd'
)
METADATA_SHA256 = (
    '65d66875f49248e38ff14d80e6e749fb'
    '50606f6f80bd4be337160e3752691e9a'
)
STORAGE_SHA256 = (
    '3bbd390a97e57af47ad6699baa36eb4c'
    '5f39f61b35275505ecaf221c126354f5'
)


def _run_demo(
    tmp_path: Path,
    script: Path,
    cli_exit: int = 0,
    *,
    resume: bool = False,
):
    data_dir = tmp_path / 'datasets'
    bag_dir = data_dir / 'driving_slam_mid360' / 'extracted' / BAG_NAME / BAG_NAME
    bag_dir.mkdir(parents=True)
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    intake_manifest = (
        data_dir / 'driving_slam_mid360'
        / 'mid360_robot_public_dataset_intake.json'
    )
    intake_manifest.write_text(
        json.dumps({
            'status': 'READY',
            'dataset': {'id': 'driving_slam_mid360'},
            'download': {
                'status': 'VERIFIED',
                'expected_size_bytes': 517088133,
                'size_bytes': 517088133,
                'expected_sha256': ARCHIVE_SHA256,
                'sha256': ARCHIVE_SHA256,
                'size_verified': True,
                'sha256_verified': True,
            },
            'extraction': {
                'status': 'VERIFIED',
                'files': [
                    {
                        'path': f'{BAG_NAME}/metadata.yaml',
                        'expected_size_bytes': 5590,
                        'size_bytes': 5590,
                        'expected_sha256': METADATA_SHA256,
                        'sha256': METADATA_SHA256,
                        'size_verified': True,
                        'sha256_verified': True,
                    },
                    {
                        'path': f'{BAG_NAME}/{BAG_NAME}_0.db3',
                        'expected_size_bytes': 1468932096,
                        'size_bytes': 1468932096,
                        'expected_sha256': STORAGE_SHA256,
                        'sha256': STORAGE_SHA256,
                        'size_verified': True,
                        'sha256_verified': True,
                    },
                ],
            },
            'selected_bag_path': str(bag_dir),
        }),
        encoding='utf-8',
    )

    stub_dir = tmp_path / 'bin'
    stub_dir.mkdir()
    capture_path = tmp_path / 'runner_args.txt'
    cli_stub = stub_dir / 'lidarslam-map'
    cli_stub.write_text(
        '#!/bin/sh\n'
        'printf "%s\\n" "$@" > "$DEMO_TEST_CAPTURE"\n'
        'exit "$DEMO_TEST_EXIT"\n',
        encoding='utf-8',
    )
    cli_stub.chmod(0o755)
    intake_capture = tmp_path / 'intake_args.txt'
    python_stub = stub_dir / 'python3'
    python_stub.write_text(
        '#!/bin/sh\n'
        'case "$1" in\n'
        '  */download_mid360_robot_public_dataset.py)\n'
        '    printf "%s\\n" "$@" > "$DEMO_TEST_INTAKE_CAPTURE"\n'
        '    exit 0\n'
        '    ;;\n'
        'esac\n'
        'exec "$DEMO_TEST_REAL_PYTHON" "$@"\n',
        encoding='utf-8',
    )
    python_stub.chmod(0o755)

    env = os.environ.copy()
    env.update(
        {
            'DEMO_DATA_DIR': str(data_dir),
            'DEMO_OUTPUT_DIR': str(tmp_path / 'output'),
            'DEMO_TEST_CAPTURE': str(capture_path),
            'DEMO_TEST_EXIT': str(cli_exit),
            'DEMO_TEST_INTAKE_CAPTURE': str(intake_capture),
            'DEMO_TEST_REAL_PYTHON': sys.executable,
            'DEMO_RESUME': '1' if resume else '0',
            'PATH': f'{stub_dir}:{env["PATH"]}',
        }
    )
    result = subprocess.run(
        ['/bin/bash', str(script)],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )
    return result, bag_dir, capture_path


def test_demo_selects_nested_directory_that_contains_metadata(tmp_path: Path):
    result, bag_dir, capture_path = _run_demo(tmp_path, FIRST_MAP_SCRIPT)

    assert result.returncode == 0, result.stderr
    assert f'bag: {bag_dir}' in result.stdout
    assert 'download_mid360_robot_public_dataset.py' in (
        tmp_path / 'intake_args.txt'
    ).read_text(encoding='utf-8')
    runner_args = capture_path.read_text(encoding='utf-8').splitlines()
    assert runner_args[:2] == ['run', str(bag_dir)]
    assert runner_args[runner_args.index('--profile') + 1] == (
        'rko_lio_graph_mid360_preset'
    )
    assert runner_args[runner_args.index('--output-dir') + 1] == str(
        tmp_path / 'output'
    )
    assert 'run_manifest.json' in result.stdout
    assert 'verify_autoware_map.log' in result.stdout
    assert 'autoware_map_diagnosis.json' in result.stdout
    assert 'first_map_validation_receipt.json' in result.stdout
    assert 'Demo stage 1/3' in result.stdout
    assert 'Demo stage 3/3' in result.stdout


def test_docker_wrapper_runs_the_canonical_first_map_demo(tmp_path: Path):
    result, bag_dir, capture_path = _run_demo(tmp_path, DOCKER_SCRIPT)

    assert result.returncode == 0, result.stderr
    assert f'bag: {bag_dir}' in result.stdout
    runner_args = capture_path.read_text(encoding='utf-8').splitlines()
    assert runner_args[:2] == ['run', str(bag_dir)]
    assert runner_args[runner_args.index('--profile') + 1] == (
        'rko_lio_graph_mid360_preset'
    )


def test_demo_resume_delegates_to_runner_resume_without_remapping_flag_loss(
    tmp_path: Path,
):
    result, bag_dir, capture_path = _run_demo(
        tmp_path,
        FIRST_MAP_SCRIPT,
        resume=True,
    )

    assert result.returncode == 0, result.stderr
    assert 'resume terminal map verification and evidence' in result.stdout
    runner_args = capture_path.read_text(encoding='utf-8').splitlines()
    assert runner_args[:2] == ['run', str(bag_dir)]
    assert runner_args[-1] == '--resume'


def test_demo_reports_failure_without_claiming_success_artifacts(tmp_path: Path):
    result, _, _ = _run_demo(tmp_path, FIRST_MAP_SCRIPT, cli_exit=7)

    assert result.returncode == 7
    assert 'first-map run failed with exit code 7' in result.stderr
    assert '== first-map artifacts ==' not in result.stdout


def test_demo_delegates_to_versioned_product_contract():
    script = FIRST_MAP_SCRIPT.read_text(encoding='utf-8')
    wrapper = DOCKER_SCRIPT.read_text(encoding='utf-8')

    assert 'lidarslam-map run "${bag_dir}"' in script
    assert '--profile rko_lio_graph_mid360_preset' in script
    assert 'run_manifest.json' in script
    assert 'verify_autoware_map.log' in script
    assert 'autoware_map_diagnosis.json' in script
    assert 'autoware_map_diagnosis.md' in script
    assert 'first_map_validation_receipt.json' in script
    assert 'first_map_validation_receipt.md' in script
    assert 'DEMO_RESUME' in script
    assert '--resume' in script
    assert 'run_rko_lio_graph_autoware_dogfood.sh' not in script
    assert 'LIDARSLAM_HOST_UID' in script
    assert 'LIDARSLAM_HOST_GID' in script
    assert 'chown -R "${HOST_UID}:${HOST_GID}" "${target}"' in script
    assert '.postprocess.lock' in script
    assert 'python3 "${SCRIPT_DIR}/download_mid360_robot_public_dataset.py"' in script
    assert '${REPO_ROOT}/scripts/' not in script
    assert 'exec "${SCRIPT_DIR}/run_first_map_demo.sh" "$@"' in wrapper
    assert 'lidarslam-map run' not in wrapper


def test_demo_rejects_incomplete_host_ownership_contract(tmp_path: Path):
    env = os.environ.copy()
    env.update(
        {
            'DEMO_DATA_DIR': str(tmp_path / 'datasets'),
            'DEMO_OUTPUT_DIR': str(tmp_path / 'output' / 'mid360_demo'),
            'LIDARSLAM_HOST_UID': str(os.getuid()),
        }
    )
    env.pop('LIDARSLAM_HOST_GID', None)

    result = subprocess.run(
        ['/bin/bash', str(DOCKER_SCRIPT)],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )

    assert result.returncode == 2
    assert 'set both LIDARSLAM_HOST_UID and LIDARSLAM_HOST_GID' in result.stderr


def test_first_map_demo_explains_missing_sourced_cli_before_download(tmp_path: Path):
    env = os.environ.copy()
    env.update(
        {
            'DEMO_DATA_DIR': str(tmp_path / 'datasets'),
            'DEMO_OUTPUT_DIR': str(tmp_path / 'output'),
            'PATH': '/usr/bin:/bin',
        }
    )

    result = subprocess.run(
        ['/bin/bash', str(FIRST_MAP_SCRIPT)],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )

    assert result.returncode == 127
    assert 'source the built workspace first' in result.stderr
    assert not (tmp_path / 'datasets').exists()
