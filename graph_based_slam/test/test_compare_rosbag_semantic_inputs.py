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


"""Unit tests for ROS1/ROS2 semantic message digests."""

from dataclasses import dataclass
import importlib.util
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'compare_rosbag_semantic_inputs.py'
SPEC = importlib.util.spec_from_file_location('semantic_inputs', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
try:
    SPEC.loader.exec_module(MODULE)
    HAS_ROSBAGS = True
except ImportError:  # pragma: no cover - environment dependent
    HAS_ROSBAGS = False

pytestmark = pytest.mark.skipif(not HAS_ROSBAGS, reason='rosbags is not installed')


@dataclass
class Header:
    stamp: int
    frame_id: str


@dataclass
class Ros1Header:
    seq: int
    stamp: int
    frame_id: str


@dataclass
class Cloud:
    header: Header
    width: int
    data: np.ndarray


@dataclass
class Point:
    offset_time: int
    x: float
    line: int
    __msgtype__: str = 'fixture/Point'


@dataclass
class PointArray:
    points: list[Point]


def test_equal_content_in_distinct_objects_has_equal_digest():
    left = Cloud(Header(42, 'lidar'), 2, np.array([1, 2, 3], dtype=np.uint8))
    right = Cloud(Header(42, 'lidar'), 2, np.array([1, 2, 3], dtype=np.uint8))
    assert MODULE.message_hash('/points', 100, left) == MODULE.message_hash(
        '/points', 100, right)


def test_ros1_header_sequence_is_ignored_for_ros2_equivalence():
    ros1 = Cloud(Ros1Header(999, 42, 'lidar'), 2,
                 np.array([1, 2, 3], dtype=np.uint8))
    ros2 = Cloud(Header(42, 'lidar'), 2,
                 np.array([1, 2, 3], dtype=np.uint8))
    assert MODULE.message_hash('/points', 100, ros1) == MODULE.message_hash(
        '/points', 100, ros2)


def test_payload_timestamp_and_field_changes_are_detected():
    base = Cloud(Header(42, 'lidar'), 2, np.array([1, 2, 3], dtype=np.uint8))
    payload = Cloud(Header(42, 'lidar'), 2, np.array([1, 2, 4], dtype=np.uint8))
    field = Cloud(Header(42, 'lidar'), 3, np.array([1, 2, 3], dtype=np.uint8))
    digest = MODULE.message_hash('/points', 100, base)
    assert digest != MODULE.message_hash('/points', 100, payload)
    assert digest != MODULE.message_hash('/points', 100, field)
    assert digest != MODULE.message_hash('/points', 101, base)


def test_compare_reports_first_mismatch_and_count_difference():
    def row(messages):
        import hashlib
        aggregate = hashlib.sha256()
        for value in messages:
            aggregate.update(bytes.fromhex(value))
        return {'count': len(messages), 'aggregate': aggregate,
                'messages': messages}

    a, b = '00' * 32, '11' * 32
    result = MODULE.compare({'/imu': row([a, a])}, {'/imu': row([a, b, a])})
    assert result['all_topics_equal'] is False
    assert result['topics'][0]['first_mismatch_index'] == 1


def test_numeric_dataclass_sequence_fast_path_detects_value_and_order():
    base = PointArray([Point(1, 2.0, 3), Point(4, 5.0, 6)])
    same = PointArray([Point(1, 2.0, 3), Point(4, 5.0, 6)])
    changed = PointArray([Point(1, 2.0, 3), Point(4, 5.1, 6)])
    reordered = PointArray([Point(4, 5.0, 6), Point(1, 2.0, 3)])
    digest = MODULE.message_hash('/lidar', 100, base)
    assert digest == MODULE.message_hash('/lidar', 100, same)
    assert digest != MODULE.message_hash('/lidar', 100, changed)
    assert digest != MODULE.message_hash('/lidar', 100, reordered)
