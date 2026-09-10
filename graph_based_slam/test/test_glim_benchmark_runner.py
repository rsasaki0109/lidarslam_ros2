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


"""Unit tests for the GLIM benchmark artifact parsers."""

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'run_glim_benchmark', ROOT / 'scripts/run_glim_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
SCORE_SPEC = importlib.util.spec_from_file_location(
    'score_repeated_trajectory_benchmark',
    ROOT / 'scripts/score_repeated_trajectory_benchmark.py')
SCORER = importlib.util.module_from_spec(SCORE_SPEC)
SCORE_SPEC.loader.exec_module(SCORER)
EXPORT_SPEC = importlib.util.spec_from_file_location(
    'export_glim_dump_map', ROOT / 'scripts/export_glim_dump_map.py')
EXPORTER = importlib.util.module_from_spec(EXPORT_SPEC)
EXPORT_SPEC.loader.exec_module(EXPORTER)


def test_source_checkout_cli_help_resolves_package_surface():
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/run_glim_benchmark.py'), '--help'],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert '--runs' in result.stdout


def test_bag_bounds_reads_rosbag2_nanoseconds(tmp_path):
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'starting_time': {'nanoseconds_since_epoch': 2_000_000_000},
        'duration': {'nanoseconds': 500_000_000},
    }}))
    assert RUNNER.bag_bounds(metadata) == (2.0, 2.5)


def test_glim_v2_environment_is_complete_and_profile_bound():
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text())['competitive_slam_profile']
    phase = yaml.safe_load(yaml.safe_dump(profile['m6a10_glim_v2b']))
    # The runner helper is tested with its pre-execution input contract.  The
    # profile separately records that the image identity is build-verified;
    # this test must not silently turn that into a replay authorization.
    phase['status'] = 'preregistered_not_executed'
    phase['result'] = None
    env = RUNNER.glim_v2_environment(phase, 'unpaced_ack', 12.5)
    assert env['GLIM_PROFILE'] == 'ntu_viral_cpu'
    assert env['M6A10_PHASE_CONTRACT_VERSION'] == \
        'm6a10-online-compute-v2'
    assert env['M6A10_PHASE_MODE'] == 'unpaced_ack'
    assert env['M6A10_GLIM_BUILD_WITH_CV_BRIDGE'] == 'ON'
    assert env['M6A10_GLIM_EXPECTED_MESSAGES'] == '236687'
    assert env['M6A10_GLIM_EXPECTED_IMU_MESSAGES'] == '225102'
    assert env['M6A10_GLIM_EXPECTED_POINTS_MESSAGES'] == '5793'
    assert env['M6A10_GLIM_EXPECTED_IMAGE_MESSAGES'] == '5792'
    assert env['M6A10_GLIM_REQUIRED_END_TIMESTAMP_SECONDS'] == '12.5'
    assert env['M6A10_GLIM_MAX_CALLBACK_LATENCY_NS'] == '250000000'
    assert env['M6A10_GLIM_MAX_CALLBACK_LATENCY_SECONDS'] == '0.25'
    assert env['M6A10_GLIM_MAX_BACKLOG_MESSAGES'] == '100000'


def test_glim_v2_environment_rejects_missing_bound():
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text())['competitive_slam_profile']
    phase = yaml.safe_load(yaml.safe_dump(profile['m6a10_glim_v2b']))
    phase['status'] = 'preregistered_not_executed'
    phase['result'] = None
    phase['phase'].pop('maximum_backlog_messages')
    try:
        RUNNER.glim_v2_environment(phase, 'paced_1x', 12.5)
    except ValueError as error:
        assert 'incomplete' in str(error)
    else:
        raise AssertionError('missing GLIM backlog bound was accepted')


def test_glim_v2_binding_rejects_input_config_or_label_mismatch(tmp_path):
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text())['competitive_slam_profile']
    phase = profile['m6a10_glim_v2b']
    bag = Path(phase['input']['path'])
    config = ROOT / phase['config']['path']
    labels = {
        'benchmark.glim.revision': profile['rivals']['glim']['revision'],
        'benchmark.glim_ros2.revision': profile['rivals']['glim'][
            'ros2_revision'],
        'benchmark.glim.build_with_cv_bridge': 'ON',
        'benchmark.glim.m6a10_core_patch_sha256':
            phase['source']['core_patch_sha256'],
        'benchmark.glim.m6a10_ros2_patch_sha256':
            phase['source']['ros2_patch_sha256'],
    }
    kwargs = {
        'bag_path': bag,
        'bag_hash': phase['input']['tree_sha256'],
        'config_path': config,
        'config_hash': phase['config']['tree_sha256'],
        'labels': labels,
        'glim_revision': profile['rivals']['glim']['revision'],
        'glim_ros2_revision': profile['rivals']['glim']['ros2_revision'],
    }
    RUNNER.validate_glim_v2_binding(phase, **kwargs)
    with pytest.raises(ValueError, match='bag path'):
        RUNNER.validate_glim_v2_binding(
            phase, **{**kwargs, 'bag_path': tmp_path / 'wrong'})
    with pytest.raises(ValueError, match='config tree SHA'):
        RUNNER.validate_glim_v2_binding(
            phase, **{**kwargs, 'config_hash': '0' * 64})
    with pytest.raises(RuntimeError, match='labels'):
        RUNNER.validate_glim_v2_binding(
            phase, **{**kwargs, 'labels': {**labels,
                                           'benchmark.glim_ros2.revision': 'bad'}})


def test_glim_v2_docker_mount_is_canonical_input_only(tmp_path):
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text())['competitive_slam_profile']
    phase = yaml.safe_load(yaml.safe_dump(profile['m6a10_glim_v2b']))
    phase['status'] = 'preregistered_not_executed'
    phase['result'] = None
    bag = Path(phase['input']['path'])
    args = type('Args', (), {
        'bag': bag,
        'image': phase['build']['image_tag'],
        'phase_mode': 'unpaced_ack',
    })()
    command = RUNNER.build_docker_command(
        args, tmp_path / 'run_01', 1, 12.5, phase)
    assert f'{bag}:/data:ro' in command
    assert f'{bag.parent}:/data:ro' not in command
    assert 'BAG_PATH=/data' in command
    assert '--network' in command and command[command.index('--network') + 1] == 'none'
    assert '--read-only' in command
    assert '--pull=never' in command
    mount = command[command.index('-v') + 1]
    assert mount == f'{bag}:/data:ro'


def test_glim_v2_output_root_accepts_only_passed_quiescence(tmp_path):
    output = tmp_path / 'v4'
    output.mkdir()
    (output / 'quiescence.json').write_text(
        '{"runner_start_allowed": true, "status": "PASS"}\n')
    RUNNER.prepare_output_directory(output, {'quiescence_preflight': {}})
    (output / 'unexpected').write_text('stale\n')
    with pytest.raises(ValueError, match='only the preregistered'):
        RUNNER.prepare_output_directory(output, {'quiescence_preflight': {}})


def test_glim_tree_hash_uses_materialization_contract(tmp_path):
    (tmp_path / 'metadata.yaml').write_bytes(b'metadata\n')
    (tmp_path / 'nested').mkdir()
    (tmp_path / 'nested' / 'payload.db3').write_bytes(b'payload\0')
    from materialize_m6a10_synchronized_tail import sha256_tree

    assert RUNNER.sha256_tree(tmp_path) == sha256_tree(tmp_path)['sha256']


def test_trajectory_info_reads_tum(tmp_path):
    trajectory = tmp_path / 'traj_lidar.txt'
    trajectory.write_text(
        '# TUM\n1.0 0 0 0 0 0 0 1\n2.5 1 0 0 0 0 0 1\n')
    assert RUNNER.trajectory_info(trajectory) == {
        'samples': 2, 'first_stamp': 1.0, 'last_stamp': 2.5}


def test_common_reference_uses_intersection_of_trajectory_ranges(tmp_path):
    reference = tmp_path / 'gt.tum'
    reference.write_text('\n'.join(
        f'{stamp} 0 0 0 0 0 0 1' for stamp in range(5)) + '\n')
    first, second = tmp_path / 'first.tum', tmp_path / 'second.tum'
    first.write_text('1 0 0 0 0 0 0 1\n4 0 0 0 0 0 0 1\n')
    second.write_text('0 0 0 0 0 0 0 1\n3 0 0 0 0 0 0 1\n')
    selected, excluded = SCORER.select_common_reference(
        reference, [first, second])
    assert [float(line.split()[0]) for line in selected] == [1.0, 2.0, 3.0]
    assert excluded == [0.0, 4.0]


def test_glim_compact_points_are_transformed_to_world(tmp_path):
    submap = tmp_path / '000000'
    submap.mkdir()
    import numpy as np
    np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32).tofile(
        submap / 'points_compact.bin')
    (submap / 'data.txt').write_text(
        'T_world_origin: \n1 0 0 10\n0 1 0 20\n0 0 1 30\n0 0 0 1\n')
    points = EXPORTER.load_world_points(submap)
    assert np.allclose(points, [[11.0, 22.0, 33.0]])
