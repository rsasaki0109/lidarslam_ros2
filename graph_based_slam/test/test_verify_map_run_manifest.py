"""Tests for read-only operational map-run manifest verification."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_SCRIPT = REPO_ROOT / 'scripts' / 'map_run_manifest.py'
VERIFY_SCRIPT = REPO_ROOT / 'scripts' / 'verify_map_run_manifest.py'


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fixture(tmp_path: Path):
    manifest = _load('map_run_manifest', MANIFEST_SCRIPT)
    sys.modules['map_run_manifest'] = manifest
    verifier = _load('verify_map_run_manifest', VERIFY_SCRIPT)
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('version: 5\n', encoding='utf-8')
    (bag / 'bag_0.db3').write_bytes(b'abc')
    output = tmp_path / 'output'
    output.mkdir()
    (output / 'verify.log').write_text('PASS\n', encoding='utf-8')
    config = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'
    command = [
        'runner', '--bag', str(bag), '--output-dir', str(output),
        '--config', str(config),
    ]
    manifest.write_manifest(
        repo_root=REPO_ROOT,
        output_dir=output,
        bag_path=bag,
        profile_id='fixture',
        command=command,
        status='success',
    )
    return verifier, bag, output


def test_verifier_reopens_matching_manifest(tmp_path: Path):
    verifier, bag, output = _fixture(tmp_path)
    result = verifier.verify_manifest(
        output_dir=output, bag_dir=bag, repo_root=REPO_ROOT)
    assert result['status'] == 'PASS'
    assert all(check['status'] == 'PASS' for check in result['checks'].values())
    assert result['safety']['read_only'] is True


def test_verifier_detects_same_size_input_mutation(tmp_path: Path):
    verifier, bag, output = _fixture(tmp_path)
    (bag / 'bag_0.db3').write_bytes(b'XYZ')
    result = verifier.verify_manifest(
        output_dir=output, bag_dir=bag, repo_root=REPO_ROOT)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['checks']['input']['status'] == 'MISMATCH'


def test_verifier_detects_output_mutation(tmp_path: Path):
    verifier, bag, output = _fixture(tmp_path)
    (output / 'verify.log').write_text('FAIL\n', encoding='utf-8')
    result = verifier.verify_manifest(
        output_dir=output, bag_dir=bag, repo_root=REPO_ROOT)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['checks']['outputs']['status'] == 'MISMATCH'


def test_verifier_rejects_symlinked_manifest(tmp_path: Path):
    verifier, bag, output = _fixture(tmp_path)
    manifest = output / 'map_run_manifest.json'
    outside = tmp_path / 'outside.json'
    manifest.rename(outside)
    manifest.symlink_to(outside)
    try:
        verifier.verify_manifest(
            output_dir=output, bag_dir=bag, repo_root=REPO_ROOT)
    except verifier.VerificationError as error:
        assert 'single-link regular file' in str(error)
    else:
        raise AssertionError('symlinked manifest was accepted')
