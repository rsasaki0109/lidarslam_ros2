"""Lock the Voxel-SLAM v35 development candidate contract."""

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v35.patch'
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
    'vertical_accel_bias_observability_min_eigen',
}


def test_patch_contains_observability_gated_bias_update():
    source = PATCH.read_text(encoding='utf-8')
    assert 'bool observable;' in source
    assert 'update_vertical_accel_bias(bool scan_observable)' in source
    assert 'vertical_accel_bias_observability_min_eigen' in source
    assert 'sample.observable' in source
    assert 'evalue[0] >= vertical_accel_bias_observability_min_eigen' in source
    assert 'v35 vertical ba gate skip' in source
    assert 'x_curr.ba -= gravity_body * correction' in source
    assert 'default false' not in source


def test_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v35' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v35_config_changes_only_the_opt_in_observability_gate(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v35 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v35/{dataset}.yaml').read_text())
    assert v35['General'] == v17['General']
    assert v35['LocalBA'] == v17['LocalBA']
    assert v35['Loop'] == v17['Loop']
    assert v35['GBA'] == v17['GBA']
    assert {
        key for key in v35['Odometry'] if key not in v17['Odometry']
    } == FEATURE_KEYS
    for key in FEATURE_KEYS:
        assert key in v35['Odometry']
    assert v35['Odometry']['vertical_accel_bias_enabled'] is True
    assert v35['Odometry']['vertical_accel_bias_observability_min_eigen'] == 14.0
