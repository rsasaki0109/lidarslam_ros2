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

"""Tests for strict NTU-VIRAL competitive input freezing."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'freeze_ntu', ROOT / 'scripts' / 'freeze_ntu_viral_holdout_inputs.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _fixture(tmp_path: Path) -> argparse.Namespace:
    archive = tmp_path / 'official.zip'
    archive.write_bytes(b'official archive')
    original = tmp_path / 'original.bag'
    normalized = tmp_path / 'normalized.bag'
    original.write_bytes(b'original')
    normalized.write_bytes(b'normalized')
    ros2 = tmp_path / 'ros2'
    ros2.mkdir()
    metadata = {'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'duration': {'nanoseconds': 20},
        'starting_time': {'nanoseconds_since_epoch': 10}, 'message_count': 3,
        'topics_with_message_count': [
            {'topic_metadata': {'name': topic}}
            for topic in sorted(MODULE.REQUIRED_TOPICS)]}}
    (ros2 / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    (ros2 / 'bag.db3').write_bytes(b'converted')
    normalization = tmp_path / 'normalization.json'
    normalization.write_text(json.dumps({
        'source_sha256': MODULE.sha256_file(original),
        'destination_sha256': MODULE.sha256_file(normalized)}))
    semantic = tmp_path / 'semantic.json'
    semantic.write_text(json.dumps({
        'all_topics_equal': True,
        'topics': [{'topic': topic, 'equal': True}
                   for topic in sorted(MODULE.REQUIRED_TOPICS)]}))
    candidate = tmp_path / 'candidate.json'
    candidate.write_text(json.dumps({
        'candidate_status': 'frozen_before_holdout',
        'repository_revision': 'a' * 40,
        'source_tree': {'sha256': 'b' * 64}}))
    files = {}
    for name in ('reference', 'reference_metadata', 'calibration', 'rko_param',
                 'lidarslam_param', 'fast_mapping_launch',
                 'fast_mapping_map_launch', 'fast_official_config',
                 'fast_official_camera_config', 'fast_image_identity'):
        files[name] = tmp_path / name
        files[name].write_bytes(name.encode())
    glim = tmp_path / 'glim'
    glim.mkdir()
    (glim / 'config.json').write_text('{}')
    return argparse.Namespace(
        slot='unseen_1', sequence='fixture', original_ros1_bag=original,
        official_archive=archive,
        archive_expected_bytes=archive.stat().st_size,
        archive_expected_md5=hashlib.md5(archive.read_bytes()).hexdigest(),
        normalized_ros1_bag=normalized, ros2_bag=ros2,
        normalization_report=normalization, semantic_report=semantic,
        candidate_manifest=candidate, glim_config_dir=glim,
        rko_lio_param=files['rko_param'],
        rko_liv_param=files['rko_param'], **files)


def test_manifest_proves_source_normalization_and_conversion(tmp_path):
    manifest = MODULE.build_manifest(_fixture(tmp_path))
    assert manifest['status'] == 'frozen'
    assert manifest['hashes']['original_rosbag1_sha256'] != (
        manifest['hashes']['raw_rosbag1_sha256'])
    assert len(manifest['hashes']['canonical_rosbag2_tree_sha256']) == 64
    assert manifest['hashes']['official_archive_bytes'] == 16
    assert manifest['hashes']['official_archive_md5'] == hashlib.md5(
        b'official archive').hexdigest()
    assert manifest['candidate']['source_tree_sha256'] == 'b' * 64
    assert manifest['hashes']['rko_lio_param_sha256'] == (
        manifest['hashes']['rko_liv_param_sha256'])


def test_normalization_hash_drift_is_rejected(tmp_path):
    args = _fixture(tmp_path)
    args.normalized_ros1_bag.write_bytes(b'changed')
    with pytest.raises(ValueError, match='destination hash mismatch'):
        MODULE.build_manifest(args)


def test_official_archive_size_drift_is_rejected(tmp_path):
    args = _fixture(tmp_path)
    args.archive_expected_bytes += 1
    with pytest.raises(ValueError, match='official archive size mismatch'):
        MODULE.build_manifest(args)
