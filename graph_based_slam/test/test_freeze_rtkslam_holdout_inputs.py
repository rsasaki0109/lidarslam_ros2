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

"""Tests for RTK-SLAM competitive input freezing."""

import argparse
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'freeze_rtkslam_holdout_inputs.py'
SPEC = importlib.util.spec_from_file_location('freeze_rtkslam', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _fixture(tmp_path: Path) -> argparse.Namespace:
    ros2 = tmp_path / 'ros2'
    ros2.mkdir()
    metadata = {'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'duration': {'nanoseconds': 20},
        'starting_time': {'nanoseconds_since_epoch': 10}, 'message_count': 4,
        'topics_with_message_count': [
            {'topic_metadata': {'name': topic}}
            for topic in sorted(MODULE.REQUIRED_ROS2_TOPICS)]}}
    (ros2 / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    (ros2 / 'bag.db3').write_bytes(b'canonical')
    files = {}
    for name in (
            'ros1_bag', 'reference', 'reference_metadata', 'calibration',
            'rko_param', 'lidarslam_param', 'fast_mapping_launch',
            'fast_mapping_map_launch', 'fast_config', 'fast_camera_config',
            'fast_image_identity'):
        files[name] = tmp_path / name
        files[name].write_bytes(name.encode())
    semantic = tmp_path / 'semantic.json'
    semantic.write_text(json.dumps({
        'all_topics_equal': True,
        'topics': [{'topic': topic, 'equal': True}
                   for topic in sorted(MODULE.SEMANTIC_TOPICS)]}))
    candidate = tmp_path / 'candidate.json'
    candidate.write_text(json.dumps({
        'candidate_status': 'frozen_before_holdout',
        'repository_revision': 'a' * 40,
        'source_tree': {'sha256': 'b' * 64}}))
    glim = tmp_path / 'glim'
    glim.mkdir()
    (glim / 'config.json').write_text('{}')
    return argparse.Namespace(
        slot='holdout_1', sequence='fixture', ros2_bag=ros2,
        semantic_report=semantic, candidate_manifest=candidate,
        glim_config_dir=glim, **files)


def test_manifest_freezes_canonical_and_verified_derivative(tmp_path):
    args = _fixture(tmp_path)
    manifest = MODULE.build_manifest(args)
    assert manifest['status'] == 'frozen'
    assert manifest['representations']['semantic_all_topics_equal'] is True
    assert len(manifest['hashes']['raw_rosbag1_sha256']) == 64
    assert len(manifest['hashes']['canonical_rosbag2_tree_sha256']) == 64
    assert manifest['candidate']['source_tree_sha256'] == 'b' * 64


def test_semantic_mismatch_is_rejected(tmp_path):
    args = _fixture(tmp_path)
    args.semantic_report.write_text(json.dumps({'all_topics_equal': False}))
    with pytest.raises(ValueError, match='did not pass'):
        MODULE.build_manifest(args)


def test_missing_ros2_sensor_topic_is_rejected(tmp_path):
    args = _fixture(tmp_path)
    document = yaml.safe_load((args.ros2_bag / 'metadata.yaml').read_text())
    document['rosbag2_bagfile_information']['topics_with_message_count'].pop()
    (args.ros2_bag / 'metadata.yaml').write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='lacks topics'):
        MODULE.build_manifest(args)
