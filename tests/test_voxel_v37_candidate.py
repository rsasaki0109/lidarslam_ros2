"""Lock the weak-axis-triggered Voxel-SLAM v37 quarantine contract."""

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v37.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)
FEATURE_KEYS = {
    'vertical_accel_bias_enabled',
    'vertical_accel_bias_quarantine_enabled',
    'vertical_accel_bias_window',
    'vertical_accel_bias_gain',
    'vertical_accel_bias_min_innovation',
    'vertical_accel_bias_max_correction',
    'vertical_accel_bias_max_velocity_change',
}


def test_patch_contains_frozen_onset_rule_and_isolated_shadow():
    source = PATCH.read_text(encoding='utf-8')
    assert 'update_vertical_accel_bias_quarantine' in source
    assert 'eigen_ratio_threshold = 0.2' in source
    assert 'horizontal_norm_threshold = 0.9' in source
    assert 'velocity_projection_threshold = 3.0' in source
    assert 'required_streak = 5' in source
    assert 'vertical_accel_bias_quarantine_shadow' in source
    assert 'v37 vertical ba quarantine' in source
    assert 'v37 vertical ba quarantined' in source
    assert 'if(quarantine_active)' in source
    assert 'x_curr.ba -= gravity_body * correction' in source


def test_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v37' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v37_config_changes_only_the_opt_in_quarantine(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v37 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v37/{dataset}.yaml').read_text())
    assert v37['General'] == v17['General']
    assert v37['LocalBA'] == v17['LocalBA']
    assert v37['Loop'] == v17['Loop']
    assert v37['GBA'] == v17['GBA']
    assert {
        key for key in v37['Odometry'] if key not in v17['Odometry']
    } == FEATURE_KEYS
    assert v37['Odometry']['vertical_accel_bias_enabled'] is True
    assert v37['Odometry']['vertical_accel_bias_quarantine_enabled'] is True
