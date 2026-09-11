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

"""Regression tests for the MID-360 robot preflight wrapper."""

from __future__ import annotations

import importlib
import json
from pathlib import Path
import struct
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'preflight_mid360_robot_bag.py'


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    return importlib.import_module('preflight_mid360_robot_bag')


def _write_metadata(tmp_path: Path, topics: list[tuple[str, str, int]]) -> Path:
    import rosbag2_py
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

    bag_dir = tmp_path / 'mid360_robot_bag'
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
    imu = Imu()
    imu.header.frame_id = 'livox_frame'
    tf_message = TFMessage()
    message_by_type = {
        'sensor_msgs/msg/PointCloud2': points,
        'sensor_msgs/msg/Imu': imu,
        'tf2_msgs/msg/TFMessage': tf_message,
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
    return bag_dir


def test_mid360_robot_preflight_emits_tuned_launch(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 50),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 500),
            ('/tf_static', 'tf2_msgs/msg/TFMessage', 1),
        ],
    )

    payload = module.build_mid360_robot_payload(
        bag_dir,
        base_frame='base_link',
        lidar_frame='livox_frame',
        imu_frame='livox_frame',
    )

    assert payload['ready_for_mid360_launch'] is True
    assert 'lidarslam_mid360_rko_graph.yaml' in payload['launch_command']
    assert 'rko_lio_mid360.yaml' in payload['launch_command']
    assert 'lidar_topic:=/livox/lidar' in payload['launch_command']
    assert 'imu_topic:=/livox/imu' in payload['launch_command']
    assert any(
        check['id'] == 'mid360_preset' and check['status'] == 'ok'
        for check in payload['checks']
    )


def test_mid360_robot_preflight_json_cli_reports_missing_imu(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 50),
        ],
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir), '--json'],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)

    assert payload['ready_for_mid360_launch'] is False
    assert payload['launch_command'] == ''
    assert any(
        check['id'] == 'imu' and check['status'] == 'fail'
        for check in payload['checks']
    )
