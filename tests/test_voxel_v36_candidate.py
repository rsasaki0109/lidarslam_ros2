"""Lock the behavior-preserving Voxel-SLAM v36 diagnostic contract."""

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v36.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)
FEATURE_KEYS = {
    'v36_observability_diagnostic_enabled',
    'v36_observability_diagnostic_window',
    'v36_observability_diagnostic_gain',
    'v36_observability_diagnostic_min_innovation',
    'v36_observability_diagnostic_max_correction',
    'v36_observability_diagnostic_max_velocity_change',
}


def test_patch_is_behavior_preserving_and_logs_shadow_residuals():
    source = PATCH.read_text(encoding='utf-8')
    assert 'v36_observability_diagnostic_enabled' in source
    assert 'v36_observability.csv' in source
    assert 'write_vertical_observability_diagnostic' in source
    assert 'weak_trigger_candidate' in source
    assert 'bias_error = 2.0 * median / window_dt' in source
    assert 'x_curr.ba' not in source
    assert 'x_curr.v +=' not in source
    assert 'weak_axis_bridge_active =' not in source
    assert 'default false' not in source


def test_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v36' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v36_config_changes_only_the_opt_in_diagnostic(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v36 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v36/{dataset}.yaml').read_text())
    assert v36['General'] == v17['General']
    assert v36['LocalBA'] == v17['LocalBA']
    assert v36['Loop'] == v17['Loop']
    assert v36['GBA'] == v17['GBA']
    assert {
        key for key in v36['Odometry'] if key not in v17['Odometry']
    } == FEATURE_KEYS
    assert v36['Odometry']['v36_observability_diagnostic_enabled'] is True
    assert v36['Odometry']['v36_observability_diagnostic_window'] == 8.0


def test_diagnostic_csv_has_stable_machine_readable_columns():
    source = PATCH.read_text(encoding='utf-8')
    for field in (
            'scan_index,t,match_num,evalue0,evalue1,evalue2,',
            'weak_eigen_x,weak_eigen_y,weak_eigen_z,',
            'weak_gravity_alignment,weak_horizontal_norm,',
            'weak_velocity_projection_abs,weak_streak_before,',
            'weak_bridge_active_before,weak_recovery_streak,',
            'position_x,position_y,position_z,velocity_x,velocity_y,',
            'velocity_z,gravity_position,gravity_velocity,',
            'innovation_count,window_dt,velocity_change,innovation_median,',
            'innovation_mad,bias_error_candidate,correction_candidate,',
            'residual_candidate_valid,weak_trigger_candidate'):
        assert field in source
