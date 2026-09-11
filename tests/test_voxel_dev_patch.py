"""Lock the Voxel-SLAM development patch tooling contract."""

import importlib.util
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/voxel_dev_patch.py'

sys.path.insert(0, str(ROOT / 'scripts'))

SPEC = importlib.util.spec_from_file_location('voxel_patch', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_patch_dir_exists():
    assert (ROOT / 'docker/patches/voxel_slam_dev').is_dir()


def test_patches_have_repository_relative_paths():
    for name in ('v23', 'v31', 'v32', 'v33', 'v34', 'v35', 'v36', 'v37', 'v38_visual_longitudinal_shadow', 'v40', 'v41', 'zconstraint', 'gba'):
        patch = ROOT / f'docker/patches/voxel_slam_dev/{name}.patch'
        assert patch.is_file(), f'{name}.patch missing'
        text = patch.read_text(encoding='utf-8', errors='replace')
        assert text.startswith('--- a/VoxelSLAM/src/voxelslam.cpp\n')
        assert '+++ b/VoxelSLAM/src/voxelslam.cpp\n' in text


def test_patches_apply_to_base_source():
    """Every development patch must apply to the frozen v17 source."""
    base = ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp'
    assert base.is_file(), f'base source missing: {base}'
    import shutil
    import tempfile
    for name in ('v23', 'v31', 'v32', 'v33', 'v34', 'v35', 'v36', 'v37', 'v38_visual_longitudinal_shadow', 'v40', 'v41', 'zconstraint', 'gba'):
        patch = ROOT / f'docker/patches/voxel_slam_dev/{name}.patch'
        with tempfile.TemporaryDirectory() as td:
            target = Path(td) / 'VoxelSLAM/src/voxelslam.cpp'
            target.parent.mkdir(parents=True)
            shutil.copy2(base, target)
            applied = subprocess.run(
                ['patch', '-p1', '-i', str(patch)],
                cwd=td, capture_output=True, text=True, check=False)
            assert applied.returncode == 0, (
                f'{name}.patch failed: {applied.stderr}')


def test_write_rejects_noop_patch():
    """A source identical to base must be rejected."""
    import pytest
    with pytest.raises(ValueError, match='no patch'):
        MODULE.write_patch(
            'noop', ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/'
            'voxelslam.cpp')
