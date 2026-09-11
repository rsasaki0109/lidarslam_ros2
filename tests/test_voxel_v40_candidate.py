"""Lock the default-off full-scan GBA graph contract for v40."""

import hashlib
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v40.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)


def test_v40_patch_restores_anchored_full_scan_graph():
    source = PATCH.read_text(encoding='utf-8')
    assert 'build_no_loop_gba_graph' in source
    assert 'initial.insert(i, pose3)' in source
    assert 'add_edge(i-1, i' in source
    assert 'graph.addPrior(0, pose0, fixd_noise)' in source
    assert 'smp->id = scan_count - 1' in source
    assert 'validate_no_loop_gba_keyframes' in source
    assert 'smp->id <= previous_id' in source
    assert 'smp->id >= scan_count' in source
    assert 'GBA/enable", no_loop_gba_enabled, false' in source
    assert 'initial.insert(kf->id, pose3)' not in source
    assert 'stepsizes.push_back(max_kf_id + 1)' not in source


def test_v40_patch_applies_to_frozen_v17_base():
    base = ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp'
    with __import__('tempfile').TemporaryDirectory() as temp_dir:
        target = Path(temp_dir) / 'VoxelSLAM/src/voxelslam.cpp'
        target.parent.mkdir(parents=True)
        target.write_bytes(base.read_bytes())
        applied = subprocess.run(
            ['patch', '-p1', '-i', str(PATCH)], cwd=temp_dir,
            capture_output=True, text=True, check=False)
        patched_digest = hashlib.sha256(target.read_bytes()).hexdigest()
    assert applied.returncode == 0, applied.stderr
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert (
        f'benchmark.candidate.voxelslam_cpp.sha256="{patched_digest}"'
        in dockerfile
    )


def test_v40_patch_sha256_is_recorded_by_dockerfile():
    digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert f'benchmark.candidate.patch.sha256="{digest}"' in dockerfile
    assert 'FROM voxel_slam_v17 AS voxel_slam_v40' in dockerfile


@pytest.mark.parametrize('dataset', DATASETS)
def test_v40_config_changes_only_global_gba_contract(dataset):
    v17 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v17/{dataset}.yaml').read_text())
    v40 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v40/{dataset}.yaml').read_text())
    for section in ('General', 'Odometry', 'LocalBA', 'Loop'):
        assert v40[section] == v17[section]
    assert v40['Loop']['enable'] == 0
    assert {
        key: value for key, value in v40['GBA'].items()
        if key not in ('enable', 'keyframe_stride',
                       'min_keyframe_distance', 'total_max_iter')
    } == {
        key: value for key, value in v17['GBA'].items()
        if key != 'total_max_iter'
    }
    assert v40['GBA']['enable'] is True
    assert v40['GBA']['keyframe_stride'] == 1
    assert v40['GBA']['min_keyframe_distance'] == 5.0
    assert v40['GBA']['total_max_iter'] == 1
