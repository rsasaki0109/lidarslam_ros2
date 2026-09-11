"""Lock the Voxel-SLAM v33 development candidate contract."""

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v33.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)
FEATURE_KEYS = {
    'vertical_velocity_bias_enabled',
    'vertical_velocity_bias_window',
    'vertical_velocity_bias_gain',
    'vertical_velocity_bias_min_residual',
    'vertical_velocity_bias_max_correction',
    'vertical_velocity_bias_max_velocity_change',
}


def test_patch_contains_causal_robust_consistency_update():
    source = PATCH.read_text(encoding='utf-8')
    assert 'vertical_velocity_history' in source
    assert 'vertical_velocity_bias_window' in source
    assert 'sort(innovations.begin(), innovations.end())' in source
    assert 'mad_weight' in source
    assert 'x_curr.v += gravity_axis * correction' in source
    assert 'default false' not in source


def test_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v33' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v33_config_changes_only_the_opt_in_vertical_bias_feature(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v33 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v33/{dataset}.yaml').read_text())
    assert v33['General'] == v17['General']
    assert v33['LocalBA'] == v17['LocalBA']
    assert v33['Loop'] == v17['Loop']
    assert v33['GBA'] == v17['GBA']
    assert {
        key for key in v33['Odometry'] if key not in v17['Odometry']
    } == FEATURE_KEYS
    for key in FEATURE_KEYS:
        assert key in v33['Odometry']
    assert v33['Odometry']['vertical_velocity_bias_enabled'] is True
