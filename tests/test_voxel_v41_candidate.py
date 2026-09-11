"""Lock the default-off cancellable/resource-bounded GBA v41 contract."""

import hashlib
from pathlib import Path
import subprocess
import tempfile

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'docker/patches/voxel_slam_dev/v41.patch'
DATASETS = (
    'navinst_indoor02',
    'oxford_spires_keble_05',
    'urbannav_hk_tunnel_1',
)


def materialize_v41_source() -> tuple[str, str]:
    base = ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp'
    with tempfile.TemporaryDirectory() as temp_dir:
        target = Path(temp_dir) / 'VoxelSLAM/src/voxelslam.cpp'
        target.parent.mkdir(parents=True)
        target.write_bytes(base.read_bytes())
        applied = subprocess.run(
            ['patch', '-p1', '-i', str(PATCH)], cwd=temp_dir,
            capture_output=True, text=True, check=False)
        source = target.read_text(encoding='utf-8')
        digest = hashlib.sha256(target.read_bytes()).hexdigest()
    assert applied.returncode == 0, applied.stderr
    return source, digest


def test_v41_patch_repairs_request_race_and_busy_wait():
    source, _ = materialize_v41_source()
    assert 'atomic<int> gba_flag{0}' in source
    assert 'atomic<bool> gba_producer_done{false}' in source
    assert 'gba_producer_done.load() && gba_flag.load() == 0' in source
    assert 'while(gba_flag);' not in source
    assert 'while(gba_flag.load() != 0 && ros::ok())' in source
    assert 'ros::WallDuration(0.01).sleep()' in source


def test_v41_patch_is_default_off_and_cancel_is_before_writeback():
    source, _ = materialize_v41_source()
    assert ('GBA/runtime_guard_enable",\n'
            '                  gba_runtime_guard_enabled, false') in source
    assert ('GBA/runtime_diagnostic_enable",\n'
            '                  gba_runtime_diagnostic_enabled, false') in source
    assert 'cancel_rss_limit' in source
    assert 'cancel_backend_deadline' in source
    cancel = source.index('cancelled_before_writeback')
    writeback = source.index('set_state(results.at', cancel)
    assert cancel < writeback
    assert 'worker_cancelled_no_writeback' in source
    assert 'state_saved_unmodified' in source


def test_v41_patch_and_source_sha256_are_recorded_by_dockerfile():
    source, source_digest = materialize_v41_source()
    assert source
    patch_digest = hashlib.sha256(PATCH.read_bytes()).hexdigest()
    dockerfile = (
        ROOT / 'docker/lio_odometry_rivals_benchmark.Dockerfile'
    ).read_text(encoding='utf-8')
    assert 'FROM voxel_slam_v17 AS voxel_slam_v41' in dockerfile
    assert f'benchmark.candidate.patch.sha256="{patch_digest}"' in dockerfile
    assert (
        f'benchmark.candidate.voxelslam_cpp.sha256="{source_digest}"'
        in dockerfile
    )


@pytest.mark.parametrize('dataset', DATASETS)
def test_v41_config_changes_only_runtime_guard_from_v40(dataset):
    v40 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v40/{dataset}.yaml').read_text())
    v41 = yaml.safe_load(
        (ROOT / f'configs/voxel_slam_v41/{dataset}.yaml').read_text())
    for section in ('General', 'Odometry', 'LocalBA', 'Loop'):
        assert v41[section] == v40[section]
    runtime_keys = {
        'runtime_diagnostic_enable', 'runtime_guard_enable',
        'backend_deadline_seconds', 'max_rss_mib',
    }
    assert {key: value for key, value in v41['GBA'].items()
            if key not in runtime_keys} == v40['GBA']
    assert v41['GBA']['runtime_diagnostic_enable'] is True
    assert v41['GBA']['runtime_guard_enable'] is True
    assert v41['GBA']['backend_deadline_seconds'] == 30.0
    assert v41['GBA']['max_rss_mib'] == 330.0
