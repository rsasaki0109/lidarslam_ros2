"""Tests for the operational map run manifest."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'map_run_manifest.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('map_run_manifest', SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fixture(tmp_path: Path):
    bag = tmp_path / 'private' / 'demo_bag'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('version: 5\n', encoding='utf-8')
    (bag / 'demo_0.db3').write_bytes(b'bag')
    output = tmp_path / 'private' / 'output'
    output.mkdir()
    (output / 'verify_autoware_map.log').write_text('RESULT: PASS\n', encoding='utf-8')
    config = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'
    return bag, output, config


def test_manifest_is_portable_and_records_operational_identity(tmp_path: Path):
    module = _load_module()
    bag, output, config = _fixture(tmp_path)
    command = [
        'runner', '--bag', str(bag), '--output-dir', str(output),
        '--config', str(config),
    ]
    manifest = module.build_manifest(
        repo_root=REPO_ROOT,
        output_dir=output,
        bag_path=bag,
        profile_id='test_profile',
        command=command,
        status='success',
    )

    encoded = json.dumps(manifest)
    assert str(tmp_path) not in encoded
    assert manifest['schema_version'] == 2
    assert '${BAG_DIR}' in manifest['workflow']['command']
    assert '${OUTPUT_DIR}' in manifest['workflow']['command']
    assert manifest['input']['metadata']['sha256']
    assert manifest['input']['storage_files'] == [
        {
            'name': 'demo_0.db3',
            'size_bytes': 3,
            'sha256': module.sha256_file(bag / 'demo_0.db3'),
        },
    ]
    assert manifest['input']['tree_sha256']
    assert manifest['input']['identity_strength'] == (
        'metadata_and_all_storage_bytes_sha256'
    )
    assert len(manifest['source']['tracked_diff_sha256']) == 64
    assert isinstance(manifest['source']['untracked_files_present'], bool)
    assert manifest['source']['identity_strength'] == (
        'revision_plus_tracked_binary_diff_sha256'
    )
    assert manifest['configs'][0]['sha256']
    assert manifest['outputs'][0]['sha256']


def test_input_tree_hash_changes_when_same_size_storage_bytes_change(tmp_path: Path):
    module = _load_module()
    bag, output, _ = _fixture(tmp_path)
    before = module.build_manifest(
        repo_root=REPO_ROOT,
        output_dir=output,
        bag_path=bag,
        profile_id='test_profile',
        command=['runner', str(bag), str(output)],
        status='success',
    )['input']

    (bag / 'demo_0.db3').write_bytes(b'BAG')
    after = module.build_manifest(
        repo_root=REPO_ROOT,
        output_dir=output,
        bag_path=bag,
        profile_id='test_profile',
        command=['runner', str(bag), str(output)],
        status='success',
    )['input']

    assert before['storage_files'][0]['size_bytes'] == after['storage_files'][0]['size_bytes']
    assert before['storage_files'][0]['sha256'] != after['storage_files'][0]['sha256']
    assert before['tree_sha256'] != after['tree_sha256']


def test_input_identity_rejects_symlinked_storage(tmp_path: Path):
    module = _load_module()
    bag, _, _ = _fixture(tmp_path)
    target = tmp_path / 'outside.db3'
    target.write_bytes(b'bag')
    (bag / 'linked.db3').symlink_to(target)

    try:
        module._input_identity(bag)
    except ValueError as error:
        assert 'must not be a symlink' in str(error)
    else:
        raise AssertionError('symlinked storage was accepted')


def test_output_inventory_hashes_large_files_without_size_only_fallback(tmp_path: Path):
    module = _load_module()
    _, output, _ = _fixture(tmp_path)
    large = output / 'pointcloud_map.pcd'
    large.write_bytes(b'a' * (2 * 1024 * 1024 + 1))

    record = next(
        item for item in module._output_inventory(output)
        if item['path'] == 'pointcloud_map.pcd'
    )
    assert record['size_bytes'] == 2 * 1024 * 1024 + 1
    assert record['sha256'] == module.sha256_file(large)
    assert set(record) == {'path', 'size_bytes', 'sha256'}


def test_output_inventory_rejects_symlink(tmp_path: Path):
    module = _load_module()
    _, output, _ = _fixture(tmp_path)
    outside = tmp_path / 'outside.pcd'
    outside.write_bytes(b'point cloud')
    (output / 'linked.pcd').symlink_to(outside)

    try:
        module._output_inventory(output)
    except ValueError as error:
        assert 'must not be a symlink' in str(error)
    else:
        raise AssertionError('symlinked output was accepted')


def test_write_manifest_replaces_temporary_file_atomically(tmp_path: Path):
    module = _load_module()
    bag, output, _ = _fixture(tmp_path)
    target = module.write_manifest(
        repo_root=REPO_ROOT,
        output_dir=output,
        bag_path=bag,
        profile_id='test_profile',
        command=['runner', str(bag), str(output)],
        status='runtime_failed',
    )

    payload = json.loads(target.read_text(encoding='utf-8'))
    assert payload['status'] == 'runtime_failed'
    assert not (output / '.map_run_manifest.json.tmp').exists()
    assert all(item['path'] != 'map_run_manifest.json' for item in payload['outputs'])
