"""Lock the Voxel-SLAM v34 development candidate contract."""

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v34.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)
FEATURE_KEYS = {
    'vertical_accel_bias_enabled',
    'vertical_accel_bias_window',
    'vertical_accel_bias_gain',
    'vertical_accel_bias_min_innovation',
    'vertical_accel_bias_max_correction',
    'vertical_accel_bias_max_velocity_change',
}


def test_patch_contains_causal_accel_bias_state_update():
    source = PATCH.read_text(encoding='utf-8')
    assert 'vertical_bias_history' in source
    assert 'vertical_accel_bias_window' in source
    assert 'bias_error = 2.0 * median / window_dt' in source
    assert 'x_curr.ba -= gravity_body * correction' in source
    assert 'vertical_accel_bias_last_update' in source
    assert 'default false' not in source


def test_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v34' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v34_config_changes_only_the_opt_in_accel_bias_feature(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v34 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v34/{dataset}.yaml').read_text())
    assert v34['General'] == v17['General']
    assert v34['LocalBA'] == v17['LocalBA']
    assert v34['Loop'] == v17['Loop']
    assert v34['GBA'] == v17['GBA']
    assert {
        key for key in v34['Odometry'] if key not in v17['Odometry']
    } == FEATURE_KEYS
    for key in FEATURE_KEYS:
        assert key in v34['Odometry']
    assert v34['Odometry']['vertical_accel_bias_enabled'] is True
