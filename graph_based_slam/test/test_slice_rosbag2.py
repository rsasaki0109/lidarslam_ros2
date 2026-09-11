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

"""Tests for deterministic raw-message rosbag2 slicing."""

import importlib.util
from pathlib import Path
import sqlite3

import pytest
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import get_types_from_msg, get_typestore, Stores


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'slice_rosbag2', ROOT / 'scripts/slice_rosbag2.py')
SLICER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SLICER)


def test_slice_bag_filters_time_and_topics_without_reserializing(tmp_path):
    source, destination = tmp_path / 'source', tmp_path / 'slice'
    typestore = get_typestore(Stores.ROS2_JAZZY)
    message_type = 'std_msgs/msg/String'
    message_class = typestore.types[message_type]
    raw_messages = [
        typestore.serialize_cdr(message_class(data=value), message_type)
        for value in ('before', 'inside', 'outside_topic', 'after')]
    with Writer(source, version=8) as writer:
        keep = writer.add_connection('/keep', message_type, typestore=typestore)
        drop = writer.add_connection('/drop', message_type, typestore=typestore)
        writer.write(keep, 1_000_000_000, raw_messages[0])
        writer.write(keep, 2_000_000_000, raw_messages[1])
        writer.write(drop, 2_100_000_000, raw_messages[2])
        writer.write(keep, 4_000_000_000, raw_messages[3])

    counts = SLICER.slice_bag(
        source, destination, duration_seconds=1.5,
        topics={'/keep'}, start_offset_seconds=1.0)
    assert counts == {'/keep': 1}
    with AnyReader([destination]) as reader:
        rows = list(reader.messages())
    assert len(rows) == 1
    assert rows[0][0].topic == '/keep'
    assert rows[0][1] == 2_000_000_000
    assert rows[0][2] == raw_messages[1]


def test_slice_bag_rejects_missing_topic(tmp_path):
    source = tmp_path / 'source'
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with Writer(source, version=8) as writer:
        writer.add_connection('/present', 'std_msgs/msg/String', typestore=typestore)
    try:
        SLICER.slice_bag(source, tmp_path / 'slice', 1.0, {'/missing'})
    except ValueError as error:
        assert 'absent from source bag' in str(error)
    else:
        raise AssertionError('missing topic was accepted')


def test_slice_reads_standard_rosbag2_without_embedded_definitions(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    source = tmp_path / 'standard_source'
    destination = tmp_path / 'standard_slice'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    metadata_kwargs = {
        'name': '/standard',
        'type': 'std_msgs/msg/String',
        'serialization_format': 'cdr',
    }
    try:
        metadata = rosbag2_py.TopicMetadata(id=0, **metadata_kwargs)
    except TypeError:  # Humble TopicMetadata predates the numeric id
        metadata = rosbag2_py.TopicMetadata(**metadata_kwargs)
    writer.create_topic(metadata)
    writer.write(
        '/standard',
        serialize_message(String(data='kept')),
        1_000_000_000,
    )
    if hasattr(writer, 'close'):
        writer.close()
    database = next(source.glob('*.db3'))
    with sqlite3.connect(database) as connection:
        connection.execute(
            "UPDATE schema SET schema_version = 3, ros_distro = 'humble'"
        )
        connection.execute('DROP TABLE IF EXISTS message_definitions')

    counts = SLICER.slice_bag(
        source,
        destination,
        duration_seconds=1.0,
        topics={'/standard'},
    )

    assert counts == {'/standard': 1}
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([destination], default_typestore=typestore) as reader:
        rows = list(reader.messages())
    assert len(rows) == 1
    assert rows[0][0].topic == '/standard'
    humble = get_typestore(Stores.ROS2_HUMBLE)
    assert rows[0][0].digest == humble.hash_rihs01('std_msgs/msg/String')


def test_slice_rejects_unverifiable_type_without_embedded_definition(tmp_path):
    source = tmp_path / 'custom_source'
    typestore = get_typestore(Stores.EMPTY)
    typestore.register(get_types_from_msg(
        'string data\n', 'fixture_msgs/msg/Custom'
    ))
    with Writer(source, version=8) as writer:
        custom = writer.add_connection(
            '/custom', 'fixture_msgs/msg/Custom', typestore=typestore
        )
        writer.write(custom, 1_000_000_000, b'custom-payload')
    database = next(source.glob('*.db3'))
    with sqlite3.connect(database) as connection:
        connection.execute(
            "UPDATE schema SET schema_version = 3, ros_distro = 'humble'"
        )
        connection.execute('DROP TABLE message_definitions')

    with pytest.raises(ValueError, match='has no embedded definition'):
        SLICER.slice_bag(
            source,
            tmp_path / 'custom_slice',
            duration_seconds=1.0,
            topics={'/custom'},
        )


def test_portable_definition_rejects_source_hash_mismatch():
    with pytest.raises(ValueError, match='source type hash mismatch'):
        SLICER._portable_msgdef(
            'std_msgs/msg/String', 'RIHS01_' + '0' * 64
        )
