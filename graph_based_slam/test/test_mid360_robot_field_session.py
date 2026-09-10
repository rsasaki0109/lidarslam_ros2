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

"""CLI tests for the MID-360 robot field-session runner."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_mid360_robot_field_session.py'
WRAPPER_PATH = REPO_ROOT / 'scripts' / 'run_mid360_robot_field_session.sh'


def _write_profile(tmp_path: Path) -> Path:
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'field_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return profile_path


def _fake_ros2_bin(tmp_path: Path) -> Path:
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    ros2 = fake_bin / 'ros2'
    ros2.write_text(
        f"""#!{sys.executable}
from pathlib import Path
import sys

args = sys.argv[1:]
if args[:2] == ['bag', 'record'] and '-o' in args:
    bag_path = Path(args[args.index('-o') + 1])
    bag_path.mkdir(parents=True, exist_ok=True)
    bag_path.joinpath('metadata.yaml').write_text('''rosbag2_bagfile_information:
  duration:
    nanoseconds: 5000000000
  message_count: 551
  topics_with_message_count:
  - topic_metadata:
      name: /livox/lidar
      type: sensor_msgs/msg/PointCloud2
      serialization_format: cdr
      offered_qos_profiles: ''
    message_count: 50
  - topic_metadata:
      name: /livox/imu
      type: sensor_msgs/msg/Imu
      serialization_format: cdr
      offered_qos_profiles: ''
    message_count: 500
  - topic_metadata:
      name: /tf_static
      type: tf2_msgs/msg/TFMessage
      serialization_format: cdr
      offered_qos_profiles: ''
    message_count: 1
''', encoding='utf-8')
    raise SystemExit(0)
raise SystemExit(0)
""",
        encoding='utf-8',
    )
    ros2.chmod(0o755)
    return fake_bin


def test_field_session_dry_run_writes_plans(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'dry_01',
            '--duration-sec',
            '10',
            '--output-dir',
            str(output_dir),
            '--skip-host-readiness',
            '--dry-run',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    assert payload['status'] == 'PASS'
    assert payload['dry_run'] is True
    assert payload['counts']['planned'] == 2
    assert (bag_root / 'dry_01_record_plan.json').is_file()
    assert (bag_root / 'dry_01_profile.yaml').is_file()
    assert (output_dir / 'mid360_robot_field_session.json').is_file()
    assert (output_dir / 'mid360_robot_session_dashboard.html').is_file()


def test_field_session_records_fake_bag_and_runs_post_check(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    fake_bin = _fake_ros2_bin(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'
    env = os.environ.copy()
    env['PATH'] = f"{fake_bin}:{env['PATH']}"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'field_01',
            '--duration-sec',
            '1',
            '--output-dir',
            str(output_dir),
            '--skip-host-readiness',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        env=env,
    )
    payload = json.loads(result.stdout)

    assert payload['status'] == 'PASS'
    assert payload['dry_run'] is False
    assert (bag_root / 'field_01' / 'metadata.yaml').is_file()
    assert (output_dir / 'mid360_robot_recording_check.json').is_file()
    assert (output_dir / 'mid360_robot_readiness.json').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()
    assert (output_dir / 'mid360_robot_session_dashboard.html').is_file()
    assert any(step['id'] == 'map' and step['status'] == 'planned'
               for step in payload['steps'])


def test_field_session_shell_wrapper_record_only(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash',
            str(WRAPPER_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'record_only',
            '--output-dir',
            str(output_dir),
            '--skip-host-readiness',
            '--record-only',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert 'MID-360 Robot Field Session' in result.stdout
    assert (output_dir / 'mid360_robot_field_session.md').is_file()
    report = json.loads((output_dir / 'mid360_robot_field_session.json').read_text())
    assert any(step['id'] == 'post_recording_check' and step['status'] == 'skipped'
               for step in report['steps'])
