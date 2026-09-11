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

"""CLI tests for the MID-360 robot post-recording check."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
RECORD_SCRIPT = REPO_ROOT / 'scripts' / 'record_mid360_robot_bag.sh'
CHECK_SCRIPT = REPO_ROOT / 'scripts' / 'check_mid360_robot_recording.py'
CHECK_WRAPPER = REPO_ROOT / 'scripts' / 'check_mid360_robot_recording.sh'


def _write_profile(tmp_path: Path) -> Path:
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'record_check_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return profile_path


def _write_metadata(
    bag_dir: Path,
    *,
    pointcloud: bool = True,
    imu: bool = True,
    tf: bool = True,
) -> None:
    import rosbag2_py
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage

    def topic_metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:  # Humble TopicMetadata predates the numeric id
            return rosbag2_py.TopicMetadata(**kwargs)

    topics = []
    if pointcloud:
        topics.append(('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 50))
    if imu:
        topics.append(('/livox/imu', 'sensor_msgs/msg/Imu', 500))
    if tf:
        topics.append(('/tf_static', 'tf2_msgs/msg/TFMessage', 1))
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for topic_id, (name, msg_type, _) in enumerate(topics):
        writer.create_topic(topic_metadata(topic_id, name, msg_type))

    points = PointCloud2()
    points.header.frame_id = 'livox_frame'
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points.point_step = 16
    points.row_step = 16
    points.data = list(struct.pack('<ffff', 1.0, 2.0, 3.0, 0.0))
    points.is_dense = True
    imu_message = Imu()
    imu_message.header.frame_id = 'livox_frame'
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'livox_frame'
    transform.transform.rotation.w = 1.0
    message_by_type = {
        'sensor_msgs/msg/PointCloud2': points,
        'sensor_msgs/msg/Imu': imu_message,
        'tf2_msgs/msg/TFMessage': TFMessage(transforms=[transform]),
    }
    for name, msg_type, count in topics:
        message = message_by_type[msg_type]
        interval_ns = max(1, 5_000_000_000 // max(1, count))
        for index in range(count):
            writer.write(
                name,
                serialize_message(message),
                1_000_000_000 + index * interval_ns,
            )
    if hasattr(writer, 'close'):
        writer.close()


def _record_dry_run(tmp_path: Path, run_id: str = 'stand_01') -> tuple[Path, Path, Path]:
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'
    subprocess.run(
        [
            'bash',
            str(RECORD_SCRIPT),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            run_id,
            '--duration-sec',
            '5',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    return (
        bag_root / run_id,
        bag_root / f'{run_id}_profile.yaml',
        bag_root / f'{run_id}_record_plan.json',
    )


def test_recording_check_passes_and_writes_readiness_and_map_plan(tmp_path: Path):
    bag_dir, profile_path, record_plan_path = _record_dry_run(tmp_path)
    _write_metadata(bag_dir)
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(CHECK_SCRIPT),
            '--bag',
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--record-plan',
            str(record_plan_path),
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['readiness_status'] == 'PASS'
    assert report['recording_plan']['run_id'] == 'stand_01'
    assert (output_dir / 'mid360_robot_recording_check.json').is_file()
    assert (output_dir / 'mid360_robot_readiness.json').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()


def test_recording_check_warns_when_record_plan_is_missing(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_dir = tmp_path / 'bags' / 'stand_no_plan'
    _write_metadata(bag_dir)
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(CHECK_SCRIPT),
            '--bag',
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'WARN'
    assert report['readiness_status'] == 'PASS'
    assert any(check['id'] == 'recording_plan' and check['status'] == 'warn'
               for check in report['checks'])


def test_recording_check_fails_when_bag_missing_imu(tmp_path: Path):
    bag_dir, profile_path, record_plan_path = _record_dry_run(tmp_path, run_id='bad_imu')
    _write_metadata(bag_dir, imu=False)
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(CHECK_SCRIPT),
            '--bag',
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--record-plan',
            str(record_plan_path),
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['readiness_status'] == 'FAIL'
    assert any(check['id'] == 'map_dry_run_plan' and check['status'] == 'fail'
               for check in report['checks'])


def test_recording_check_shell_wrapper_outputs_human_report(tmp_path: Path):
    bag_dir, profile_path, record_plan_path = _record_dry_run(tmp_path, run_id='stand_02')
    _write_metadata(bag_dir)
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash',
            str(CHECK_WRAPPER),
            '--bag',
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--record-plan',
            str(record_plan_path),
            '--output-dir',
            str(output_dir),
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert 'MID-360 Robot Recording Check' in result.stdout
    assert 'Recording check JSON:' in result.stdout
    assert (output_dir / 'mid360_robot_recording_check.md').is_file()
