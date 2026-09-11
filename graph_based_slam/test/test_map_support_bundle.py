"""Tests for privacy-safe map support bundles."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tarfile


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'create_map_support_bundle.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('create_map_support_bundle', SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bundle_redacts_paths_and_omits_large_artifacts(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'private-user' / 'run'
    run_dir.mkdir(parents=True)
    private_path = f'{run_dir}/dataset/secret_bag'
    (run_dir / 'autoware_map_diagnosis.md').write_text(
        f'run: {run_dir}\nbag: {private_path}\n', encoding='utf-8'
    )
    (run_dir / 'map.pcd').write_bytes(b'point cloud must not be shared')
    (run_dir / 'rosbag.db3').write_bytes(b'bag must not be shared')
    output = tmp_path / 'support.tar.gz'

    module.create_bundle(run_dir, output)

    with tarfile.open(output, 'r:gz') as archive:
        names = archive.getnames()
        diagnosis = archive.extractfile('support/autoware_map_diagnosis.md').read().decode()
        manifest = json.loads(
            archive.extractfile('support/support_bundle_manifest.json').read()
        )
    assert 'support/map.pcd' not in names
    assert 'support/rosbag.db3' not in names
    assert str(run_dir) not in diagnosis
    assert '${RUN_DIR}' in diagnosis
    assert manifest['privacy']['bags_maps_and_pointclouds_included'] is False


def test_bundle_rejects_empty_run(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'empty'
    run_dir.mkdir()
    try:
        module.create_bundle(run_dir, tmp_path / 'support.tar.gz')
    except ValueError as exc:
        assert 'no supported diagnostics' in str(exc)
    else:
        raise AssertionError('empty run should be rejected')

