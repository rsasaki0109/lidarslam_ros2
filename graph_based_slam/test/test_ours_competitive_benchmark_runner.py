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

"""Tests for the in-workspace competitive benchmark sensor contract."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'run_ours_competitive_benchmark',
    ROOT / 'scripts' / 'run_ours_competitive_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


def test_sensor_arguments_support_frozen_non_hilti_topics():
    args = argparse.Namespace(
        lidar_topic='/os1_cloud_node1/points',
        imu_topic='/imu/imu', base_frame='base_link')
    assert RUNNER.sensor_arguments(args) == [
        '--lidar-topic', '/os1_cloud_node1/points',
        '--imu-topic', '/imu/imu', '--base-frame', 'base_link']


def test_visual_sensor_contract_requires_explicit_matching_topic(tmp_path):
    params = tmp_path / 'rko.yaml'
    params.write_text(
        'direct_visual_frontend: true\n'
        'visual_image_topic: /left/image_raw\n'
        'visual_camera_distortion_model: plumb_bob\n')
    assert RUNNER.visual_sensor_contract(params, '/left/image_raw') == {
        'enabled': True,
        'camera_topic': '/left/image_raw',
        'distortion_model': 'plumb_bob',
    }
    with pytest.raises(ValueError, match='required'):
        RUNNER.visual_sensor_contract(params, '')
    with pytest.raises(ValueError, match='mismatch'):
        RUNNER.visual_sensor_contract(params, '/right/image_raw')


def test_visual_sensor_contract_keeps_lidar_imu_run_camera_free(tmp_path):
    params = tmp_path / 'rko.yaml'
    params.write_text('direct_visual_frontend: false\n')
    assert RUNNER.visual_sensor_contract(params, '') == {
        'enabled': False,
        'camera_topic': None,
        'distortion_model': None,
    }


def test_rko_base_pose_contract_uses_body_not_lidar_lever_arm(tmp_path):
    metadata = tmp_path / 'reference.json'
    metadata.write_text(json.dumps({
        'body_to_prism_translation_m': {'x': -0.29, 'y': -0.01, 'z': -0.27},
        'lidar_to_prism_translation_m': {'x': -0.24, 'y': -0.01, 'z': -0.33},
    }))
    assert RUNNER.base_to_prism_contract(metadata) == {
        'source_frame': 'body',
        'target_frame': 'leica_prism',
        'offset_m': {'x': -0.29, 'y': -0.01, 'z': -0.27},
    }


def test_rko_pose_contract_prefers_generic_reference_point(tmp_path):
    metadata = tmp_path / 'reference.json'
    metadata.write_text(json.dumps({
        'reference_point_frame': 'base_center',
        'imu_to_reference_translation_m': {
            'x': -0.073, 'y': -0.023, 'z': -0.172},
        'imu_to_prism_translation_m': {'x': 9, 'y': 9, 'z': 9},
    }))
    assert RUNNER.base_to_prism_contract(metadata) == {
        'source_frame': 'imu',
        'target_frame': 'base_center',
        'offset_m': {'x': -0.073, 'y': -0.023, 'z': -0.172},
    }


def test_ntu_rko_lidar_to_base_is_inverse_of_glim_lidar_from_imu():
    rko = yaml.safe_load((
        ROOT / 'lidarslam/param/rko_lio_ntu_viral_direct_visual.yaml').read_text())
    glim = json.loads((
        ROOT / 'configs/glim/ntu_viral_ouster_cpu/config_sensors.json').read_text())
    base_t_lidar = rko['extrinsic_lidar2base_quat_xyzw_xyz'][4:]
    lidar_t_imu = glim['sensors']['T_lidar_imu'][:3]
    assert base_t_lidar == [-value for value in lidar_t_imu]
    calibration = json.loads((
        ROOT / 'configs/ntu_viral/official_shared_calibration.json').read_text())
    body_t_lidar = calibration['frames']['body_T_lidar']
    assert base_t_lidar == body_t_lidar[:3]
    assert rko['extrinsic_lidar2base_quat_xyzw_xyz'][:4] == body_t_lidar[3:]


def test_ntu_camera_to_base_reconstructs_official_fast_livo2_rcl_pcl():
    rko = yaml.safe_load((
        ROOT / 'lidarslam/param/rko_lio_ntu_viral_direct_visual.yaml').read_text())

    def transform(values):
        qx, qy, qz, qw, x, y, z = values
        rotation = np.array([
            [1 - 2 * (qy * qy + qz * qz),
             2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw),
             1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)],
        ])
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = [x, y, z]
        return result

    base_t_camera = transform(rko['extrinsic_cam2base_quat_xyzw_xyz'])
    base_t_lidar = transform(rko['extrinsic_lidar2base_quat_xyzw_xyz'])
    camera_t_lidar = np.linalg.inv(base_t_camera) @ base_t_lidar
    expected_rotation = np.array([
        [0.0218308, 0.99976, -0.00201407],
        [-0.0131205, 0.00230088, 0.999911],
        [0.999676, -0.0218025, 0.0131676],
    ])
    expected_translation = np.array([0.122993, 0.0398643, -0.0577101])
    assert camera_t_lidar[:3, :3] == pytest.approx(expected_rotation, abs=1e-6)
    assert camera_t_lidar[:3, 3] == pytest.approx(expected_translation, abs=1e-6)
