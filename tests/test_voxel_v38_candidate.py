"""Lock the visual longitudinal shadow source/unit contract."""

import subprocess
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v38_visual_longitudinal_shadow.patch'


def test_shadow_contract_is_default_off_and_state_isolated():
    source = PATCH.read_text(encoding='utf-8')
    assert 'visual_longitudinal_shadow_enabled, false' in source
    assert 'visual_longitudinal_shadow_max_age_sec' in source
    assert 'visual_longitudinal_shadow_max_speed_mps' in source
    assert 'visual_longitudinal_shadow_topic' in source
    assert 'visual_longitudinal_shadow_gain' in source
    assert 'visual_longitudinal_shadow_max_velocity_change_mps' in source
    assert 'stamp_sec' in source
    assert 'velocity_mps' in source
    assert 'confidence' in source
    assert 'ingest_visual_longitudinal_shadow' in source
    assert 'visual_longitudinal_shadow_handler' in source
    assert 'apply_visual_longitudinal_shadow' in source
    assert 'visual_longitudinal_shadow_applied_count' in source
    assert 'x_curr.ba' not in source
    assert 'x_curr.v' not in source
    assert 'x_curr.p' not in source


def test_shadow_patch_applies_to_frozen_v17_base():
    base = ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp'
    with __import__('tempfile').TemporaryDirectory() as temp_dir:
        target = Path(temp_dir) / 'VoxelSLAM/src/voxelslam.cpp'
        target.parent.mkdir(parents=True)
        target.write_bytes(base.read_bytes())
        applied = subprocess.run(
            ['patch', '-p1', '-i', str(PATCH)],
            cwd=temp_dir, capture_output=True, text=True, check=False)
    assert applied.returncode == 0, applied.stderr


@pytest.mark.parametrize('dataset', [
    'navinst_indoor02', 'oxford_spires_keble_05', 'urbannav_hk_tunnel_1'])
def test_v38_config_is_v17_plus_explicit_opt_in(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v38 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v38/{dataset}.yaml').read_text())
    for section in ('General', 'LocalBA', 'Loop', 'GBA'):
        assert v38[section] == v17[section]
    assert v38['Odometry'] | {}  # keep the section explicit for diagnostics
    extra = {
        key: value for key, value in v38['Odometry'].items()
        if key not in v17['Odometry']}
    assert extra == {
        'visual_longitudinal_shadow_enabled': True,
        'visual_longitudinal_shadow_topic':
            '/voxel_slam/visual_longitudinal_shadow',
        'visual_longitudinal_shadow_max_age_sec': 0.2,
        'visual_longitudinal_shadow_max_speed_mps': 20.0,
        'visual_longitudinal_shadow_gain': 0.25,
        'visual_longitudinal_shadow_max_velocity_change_mps': 0.5,
    }
