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

"""Tests for the MID-360 bag stamp rewriter."""

from __future__ import annotations

import importlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('rosbags') is None,
    reason='rosbags python module not available (pip install rosbags)',
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
CLI_PATH = SCRIPT_DIR / 'rewrite_mid360_robot_bag_stamps.py'


def _rewriter_module():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    return importlib.import_module('mid360_robot_bag_stamp_rewriter')


def _sample_bag_module():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    return importlib.import_module('mid360_robot_sample_bag')


def _write_sample_bag(path: Path) -> None:
    sample = _sample_bag_module()
    sample.Mid360SampleBagWriter(
        sample.SampleBagConfig(
            output_path=path,
            duration_sec=1.0,
            pointcloud_rate_hz=10.0,
            imu_rate_hz=100.0,
            force=True,
        )
    ).write()


def _corrupt_pointcloud_stamps(bag: Path, jump_sec: float) -> None:
    """
    Rewrite header.stamp of each PointCloud2 message to add a large jump.

    Mimics the upstream behaviour seen in some public MID-360 datasets where
    LiDAR header.stamp drifts away from rosbag2 receive time by hundreds of
    seconds. Used only as a test fixture.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.LATEST)
    tmp_bag = bag.with_name(bag.name + '_tmp')
    if tmp_bag.exists():
        import shutil
        shutil.rmtree(tmp_bag)

    jump_ns = int(jump_sec * 1_000_000_000)
    with AnyReader([bag], default_typestore=typestore) as reader, Writer(
        tmp_bag,
        version=8,
    ) as writer:
        output_connections: dict[str, object] = {}
        for conn in reader.connections:
            kwargs = {
                'typestore': typestore,
                'serialization_format': conn.ext.serialization_format,
                'offered_qos_profiles': conn.ext.offered_qos_profiles,
            }
            if conn.digest and getattr(conn.msgdef, 'data', ''):
                kwargs['msgdef'] = conn.msgdef.data
                kwargs['rihs01'] = conn.digest
            output_connections[conn.topic] = writer.add_connection(
                conn.topic, conn.msgtype, **kwargs,
            )
        for conn, timestamp_ns, raw in reader.messages():
            if conn.msgtype == 'sensor_msgs/msg/PointCloud2':
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                bad_ns = int(timestamp_ns) + jump_ns
                msg.header.stamp.sec = bad_ns // 1_000_000_000
                msg.header.stamp.nanosec = bad_ns % 1_000_000_000
                raw = typestore.serialize_cdr(msg, conn.msgtype)
            writer.write(output_connections[conn.topic], timestamp_ns, raw)

    import shutil
    shutil.rmtree(bag)
    tmp_bag.rename(bag)


def _read_pointcloud_header_stamps(bag: Path) -> list[int]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.LATEST)
    stamps: list[int] = []
    with AnyReader([bag], default_typestore=typestore) as reader:
        for conn, _, raw in reader.messages():
            if conn.msgtype != 'sensor_msgs/msg/PointCloud2':
                continue
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            stamps.append(
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )
    return stamps


def _read_receive_times(bag: Path, msgtype: str) -> list[int]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.LATEST)
    out: list[int] = []
    with AnyReader([bag], default_typestore=typestore) as reader:
        for conn, timestamp_ns, _ in reader.messages():
            if conn.msgtype != msgtype:
                continue
            out.append(int(timestamp_ns))
    return out


def test_rewriter_resets_pointcloud_stamps_to_receive_time(tmp_path: Path):
    source_bag = tmp_path / 'corrupted_bag'
    output_bag = tmp_path / 'rewritten_bag'

    _write_sample_bag(source_bag)
    _corrupt_pointcloud_stamps(source_bag, jump_sec=500.0)

    rewriter = _rewriter_module()
    summary = rewriter.BagStampRewriter().rewrite(
        rewriter.BagStampRewriterOptions(
            input_bag=source_bag,
            output_bag=output_bag,
            force=True,
        )
    )

    assert summary['status'] == 'PASS'
    assert summary['result']['total_messages'] > 0
    assert summary['result']['rewritten_messages'] > 0

    pc_per_topic = summary['result']['per_topic'].get('/livox/lidar', {})
    assert pc_per_topic.get('msgtype') == 'sensor_msgs/msg/PointCloud2'
    assert pc_per_topic.get('rewritten') == 10  # 10Hz over 1.0s, exclusive end
    assert pc_per_topic.get('max_stamp_delta_sec', 0) > 499.0

    receive_times = _read_receive_times(output_bag, 'sensor_msgs/msg/PointCloud2')
    rewritten_stamps = _read_pointcloud_header_stamps(output_bag)
    assert receive_times == rewritten_stamps

    json_path = output_bag.parent / rewriter.BAG_STAMP_REWRITER_JSON
    md_path = output_bag.parent / rewriter.BAG_STAMP_REWRITER_MARKDOWN
    assert json_path.is_file()
    assert md_path.is_file()


def test_rewriter_passthrough_when_unaffected(tmp_path: Path):
    source_bag = tmp_path / 'clean_bag'
    output_bag = tmp_path / 'rewritten_bag'

    _write_sample_bag(source_bag)

    rewriter = _rewriter_module()
    summary = rewriter.BagStampRewriter().rewrite(
        rewriter.BagStampRewriterOptions(
            input_bag=source_bag,
            output_bag=output_bag,
            force=True,
        )
    )

    assert summary['status'] == 'PASS'
    pc_per_topic = summary['result']['per_topic'].get('/livox/lidar', {})
    assert pc_per_topic.get('rewritten') == 10
    assert pc_per_topic.get('max_stamp_delta_sec', 1.0) < 1e-6


def test_rewriter_respects_explicit_msgtype_list(tmp_path: Path):
    source_bag = tmp_path / 'corrupted_bag'
    output_bag = tmp_path / 'rewritten_bag'

    _write_sample_bag(source_bag)
    _corrupt_pointcloud_stamps(source_bag, jump_sec=200.0)

    rewriter = _rewriter_module()
    summary = rewriter.BagStampRewriter().rewrite(
        rewriter.BagStampRewriterOptions(
            input_bag=source_bag,
            output_bag=output_bag,
            rewrite_msgtypes=('sensor_msgs/msg/Imu',),
            force=True,
        )
    )

    assert summary['status'] == 'PASS'
    pc_per_topic = summary['result']['per_topic'].get('/livox/lidar', {})
    imu_per_topic = summary['result']['per_topic'].get('/livox/imu', {})
    assert pc_per_topic.get('rewritten', 0) == 0
    assert pc_per_topic.get('passthrough', 0) == 10
    assert imu_per_topic.get('rewritten', 0) == 100


def test_cli_writes_summary(tmp_path: Path):
    source_bag = tmp_path / 'corrupted_bag'
    output_bag = tmp_path / 'rewritten_bag'

    _write_sample_bag(source_bag)
    _corrupt_pointcloud_stamps(source_bag, jump_sec=300.0)

    result = subprocess.run(
        [
            sys.executable,
            str(CLI_PATH),
            '--input-bag', str(source_bag),
            '--output-bag', str(output_bag),
            '--force',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    summary = json.loads(result.stdout)
    assert summary['status'] == 'PASS'
    assert summary['result']['rewritten_messages'] > 0
    assert (output_bag / 'metadata.yaml').is_file()
