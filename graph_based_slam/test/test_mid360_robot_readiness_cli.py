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

"""CLI tests for MID-360 robot readiness checks."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'check_mid360_robot_readiness.py'


def _write_metadata(
    tmp_path: Path,
    *,
    pointcloud: bool = True,
    imu: bool = True,
    tf: bool = True,
) -> Path:
    bag_dir = tmp_path / 'mid360_robot_bag'
    if pointcloud and imu:
        command = [
            sys.executable,
            str(REPO_ROOT / 'scripts' / 'generate_mid360_robot_sample_bag.py'),
            str(bag_dir),
            '--duration-sec',
            '5',
            '--pointcloud-rate-hz',
            '10',
            '--imu-rate-hz',
            '100',
            '--point-count',
            '16',
        ]
        if not tf:
            command.append('--no-tf-static')
        subprocess.run(command, check=True, cwd=REPO_ROOT)
        return bag_dir

    bag_dir.mkdir()
    topics = []
    if pointcloud:
        topics.append(('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 50))
    if imu:
        topics.append(('/livox/imu', 'sensor_msgs/msg/Imu', 500))
    if tf:
        topics.append(('/tf_static', 'tf2_msgs/msg/TFMessage', 1))

    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 5_000_000_000},
            'message_count': sum(count for _, _, count in topics),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for name, msg_type, count in topics
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def _write_profile(tmp_path: Path, *, pointcloud_topic='/livox/lidar') -> Path:
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'readiness_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': pointcloud_topic,
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return profile_path


def _run_readiness(
    bag_dir: Path,
    output_dir: Path,
    profile_path: Path | None = None,
    *extra_args: str,
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(SCRIPT_PATH),
        str(bag_dir),
        '--output-dir',
        str(output_dir),
    ]
    if profile_path is not None:
        command.extend(['--robot-profile', str(profile_path)])
    command.extend(extra_args)
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )


def test_readiness_pass_writes_report_and_manifest(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    profile_path = _write_profile(tmp_path)
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir, profile_path, '--write-manifest')
    report = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert result.returncode == 0
    assert report['status'] == 'PASS'
    assert report['counts']['fail'] == 0
    assert math.isclose(
        report['bag_diagnostics']['topics']['pointcloud']['metadata_rate_hz'],
        10.0,
        rel_tol=0.02,
    )
    assert math.isclose(
        report['bag_diagnostics']['topics']['imu']['metadata_rate_hz'],
        100.0,
        rel_tol=0.02,
    )
    assert any(check['id'] == 'pointcloud_metadata_rate' for check in report['checks'])
    assert (output_dir / 'mid360_robot_readiness.md').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()


def test_readiness_missing_imu_fails(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path, imu=False)
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir)
    report = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'imu' for check in report['checks'])


def test_readiness_no_tf_warns(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path, tf=False)
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir)
    report = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert result.returncode == 0
    assert report['status'] == 'WARN'
    assert any(check['id'] == 'tf_metadata' and check['status'] == 'warn'
               for check in report['checks'])


def test_readiness_expected_topic_mismatch_fails(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    profile_path = _write_profile(tmp_path, pointcloud_topic='/wrong/lidar')
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir, profile_path)
    report = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'expected_pointcloud_topic'
               for check in report['checks'])


def test_readiness_json_output_is_machine_readable(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir, None, '--json')
    payload = json.loads(result.stdout)

    assert result.returncode == 0
    assert payload['status'] == 'PASS'
    assert payload['selected_topics']['pointcloud'] == '/livox/lidar'


def test_readiness_invalid_profile_still_writes_fail_report(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    profile_path = tmp_path / 'bad_profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({'robot_name': 12}),
        encoding='utf-8',
    )
    output_dir = tmp_path / 'out'

    result = _run_readiness(bag_dir, output_dir, profile_path)
    report = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['checks'][0]['id'] == 'readiness_setup'
