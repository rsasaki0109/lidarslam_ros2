# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for content-bound, fail-closed GLIM trajectory caching."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / 'scripts'
SCRIPT = SCRIPT_DIR / 'glim_reference_cache.py'
sys.path.insert(0, str(SCRIPT_DIR))
try:
    SPEC = importlib.util.spec_from_file_location(
        'glim_reference_cache_test',
        SCRIPT,
    )
    assert SPEC is not None and SPEC.loader is not None
    CACHE = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(CACHE)
finally:
    sys.path.remove(str(SCRIPT_DIR))


def _inputs(tmp_path: Path) -> dict[str, Path]:
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  version: 9\n',
        encoding='utf-8',
    )
    (bag / 'bag_0.db3').write_bytes(b'bag-payload-v1')
    config = tmp_path / 'config'
    config.mkdir()
    (config / 'config.json').write_text(
        '{"global":{"config_odometry":"cpu.json"}}\n',
        encoding='utf-8',
    )
    runtime = tmp_path / 'glim_rosbag'
    runtime.write_bytes(b'glim-runtime-v1')
    harness = tmp_path / 'compare_with_glim.sh'
    harness.write_text('#!/usr/bin/env bash\n', encoding='utf-8')
    trajectory = tmp_path / 'traj_lidar.txt'
    trajectory.write_text(
        '1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    return {
        'bag': bag,
        'config': config,
        'runtime': runtime,
        'harness': harness,
        'trajectory': trajectory,
    }


def _build(inputs: dict[str, Path], **overrides) -> dict:
    values = {
        'bag_dir': inputs['bag'],
        'config_dir': inputs['config'],
        'runtime_kind': 'local-install',
        'runtime_paths': [inputs['runtime']],
        'runtime_tokens': ['ros=jazzy;arch=x86_64'],
        'harness_path': inputs['harness'],
        'points_topic': '/points',
        'imu_topic': '/imu',
        'mode': 'lidar-imu',
        'preset': 'cpu',
        'no_imu': False,
        'viewer': False,
        'omp_threads': '1',
    }
    values.update(overrides)
    return CACHE.build_identity(**values)


def test_identity_binds_bytes_runtime_request_and_hides_paths(tmp_path: Path):
    """Every input that can change the GLIM trajectory changes the key."""
    inputs = _inputs(tmp_path)
    base = _build(inputs)
    keys = {base['cache_key_sha256']}

    for path, payload in (
        (inputs['bag'] / 'bag_0.db3', b'bag-payload-v2'),
        (inputs['config'] / 'config.json', b'{"changed":true}\n'),
        (inputs['runtime'], b'glim-runtime-v2'),
        (inputs['harness'], b'#!/bin/false\n'),
    ):
        original = path.read_bytes()
        path.write_bytes(payload)
        keys.add(_build(inputs)['cache_key_sha256'])
        path.write_bytes(original)

    keys.add(
        _build(inputs, points_topic='/other-points')['cache_key_sha256']
    )
    assert len(keys) == 6
    rendered = json.dumps(base, sort_keys=True)
    assert str(tmp_path) not in rendered
    assert base['authority'] == {
        'reference_kind': 'cross_validation',
        'ground_truth': False,
        'remote_mutations_performed': False,
    }
    assert len(base['identity']['request']['cache_helper_sha256']) == 64


def test_store_and_lookup_require_exact_manifest_and_trajectory(
    tmp_path: Path,
):
    """A cache hit is returned only after all content checks pass."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    CACHE.write_identity(identity_path, _build(inputs))
    manifest = CACHE.store_entry(
        identity_path,
        cache_dir,
        inputs['trajectory'],
    )
    trajectory_path, manifest_path = CACHE.lookup_entry(
        identity_path,
        cache_dir,
    )
    assert trajectory_path.read_bytes() == inputs['trajectory'].read_bytes()
    assert manifest_path.is_file()
    assert manifest['entry_state'] == 'CACHED'
    assert manifest['trajectory']['line_count'] == 2

    created_at = manifest['created_at']
    repeated = CACHE.store_entry(
        identity_path,
        cache_dir,
        inputs['trajectory'],
    )
    assert repeated['created_at'] == created_at


def test_lookup_miss_is_read_only(tmp_path: Path):
    """A cache lookup cannot create local state when no cache exists."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'missing-cache'
    CACHE.write_identity(identity_path, _build(inputs))

    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='no verified',
    ):
        CACHE.lookup_entry(identity_path, cache_dir)
    assert not cache_dir.exists()


def test_tampering_identity_drift_and_collision_fail_closed(tmp_path: Path):
    """Neither stale identity nor changed bytes can become a cache hit."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    CACHE.write_identity(identity_path, _build(inputs))
    CACHE.store_entry(identity_path, cache_dir, inputs['trajectory'])
    trajectory_path, _ = CACHE.lookup_entry(identity_path, cache_dir)

    trajectory_path.write_text(
        '1.0 0 0 0 0 0 0 1\n2.0 9 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='contradict',
    ):
        CACHE.lookup_entry(identity_path, cache_dir)
    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='collision|nondeterministic',
    ):
        CACHE.store_entry(identity_path, cache_dir, inputs['trajectory'])

    inputs['bag'].joinpath('bag_0.db3').write_bytes(b'different bag')
    drifted_path = tmp_path / 'drifted.json'
    CACHE.write_identity(drifted_path, _build(inputs))
    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='no verified',
    ):
        CACHE.lookup_entry(drifted_path, cache_dir)


def test_racing_store_never_deletes_another_writer_entry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A losing writer preserves the complete entry that won publication."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    CACHE.write_identity(identity_path, _build(inputs))
    cache_dir.mkdir()
    winning_payload = (
        '1.0 0 0 0 0 0 0 1\n'
        '2.0 9 0 0 0 0 0 1\n'
    ).encode()

    def publish_competing_entry(source: Path, target: Path) -> bool:
        del source
        target.write_bytes(winning_payload)
        return False

    monkeypatch.setattr(CACHE, '_copy_once', publish_competing_entry)

    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='collision|nondeterministic',
    ):
        CACHE.store_entry(identity_path, cache_dir, inputs['trajectory'])

    trajectory_target = next(cache_dir.glob('*.traj_lidar.txt'))
    assert trajectory_target.read_bytes() == winning_payload


def test_racing_identical_manifest_returns_first_complete_entry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Concurrent identical stores accept the first atomic manifest."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    CACHE.write_identity(identity_path, _build(inputs))
    original = CACHE._write_json_once

    def publish_competing_manifest(path: Path, payload: dict) -> None:
        if path.name.endswith('.manifest.json'):
            competing = dict(payload)
            competing['created_at'] = '2026-08-17T00:00:00Z'
            original(path, competing)
            raise CACHE.GlimReferenceCacheError(
                'simulated simultaneous manifest publication'
            )
        original(path, payload)

    monkeypatch.setattr(CACHE, '_write_json_once', publish_competing_manifest)

    manifest = CACHE.store_entry(
        identity_path,
        cache_dir,
        inputs['trajectory'],
    )

    assert manifest['created_at'] == '2026-08-17T00:00:00Z'
    assert CACHE.lookup_entry(identity_path, cache_dir)[0].is_file()


def test_writer_removes_only_its_own_invalid_published_copy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A source mutation cannot strand a malformed entry under the key."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    CACHE.write_identity(identity_path, _build(inputs))
    cache_dir.mkdir()

    def publish_invalid_entry(source: Path, target: Path) -> bool:
        del source
        target.write_text('incomplete\n', encoding='utf-8')
        return True

    monkeypatch.setattr(CACHE, '_copy_once', publish_invalid_entry)

    with pytest.raises(CACHE.GlimReferenceCacheError):
        CACHE.store_entry(identity_path, cache_dir, inputs['trajectory'])

    assert list(cache_dir.iterdir()) == []


def test_store_rejects_manifest_symlink_even_when_target_is_valid(
    tmp_path: Path,
):
    """A manifest alias cannot become an idempotent cache-store result."""
    inputs = _inputs(tmp_path)
    identity = _build(inputs)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    source_cache = tmp_path / 'source-cache'
    CACHE.write_identity(identity_path, identity)
    source_manifest = CACHE.store_entry(
        identity_path,
        source_cache,
        inputs['trajectory'],
    )
    cache_dir.mkdir()
    key = identity['cache_key_sha256']
    (cache_dir / f'{key}.traj_lidar.txt').write_bytes(
        inputs['trajectory'].read_bytes()
    )
    manifest_target = cache_dir / f'{key}.manifest.json'
    source_manifest_path = source_cache / manifest_target.name
    assert source_manifest['entry_state'] == 'CACHED'
    manifest_target.symlink_to(source_manifest_path)

    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='manifest is not a regular file',
    ):
        CACHE.store_entry(identity_path, cache_dir, inputs['trajectory'])

    assert manifest_target.is_symlink()


@pytest.mark.parametrize(
    'contents',
    [
        '1.0 0 0 0 0 0 0 1\n',
        '1.0 0 0 0 0 0 0 1\n1.0 1 0 0 0 0 0 1\n',
        '1.0 0 0 0 0 0 nan 1\n2.0 1 0 0 0 0 0 1\n',
        '1.0 0 0 0 0 0 1\n2.0 1 0 0 0 0 1\n',
    ],
)
def test_malformed_tum_trajectory_is_never_cached(
    tmp_path: Path,
    contents: str,
):
    """A cache cannot preserve malformed or ambiguous TUM data."""
    trajectory = tmp_path / 'trajectory.tum'
    trajectory.write_text(contents, encoding='utf-8')
    with pytest.raises(CACHE.GlimReferenceCacheError):
        CACHE._validate_trajectory(trajectory)


def test_symlink_inputs_are_rejected(tmp_path: Path):
    """Filesystem aliases cannot escape the content identity boundary."""
    inputs = _inputs(tmp_path)
    inputs['bag'].joinpath('alias.db3').symlink_to('bag_0.db3')
    with pytest.raises(
        CACHE.GlimReferenceCacheError,
        match='symlinks',
    ):
        _build(inputs)


def test_cli_round_trip_and_shell_integration_contract(tmp_path: Path, capsys):
    """The public helper and shell harness expose only verified cache reuse."""
    inputs = _inputs(tmp_path)
    identity_path = tmp_path / 'identity.json'
    cache_dir = tmp_path / 'cache'
    common = [
        '--identity', str(identity_path),
        '--cache-dir', str(cache_dir),
    ]
    assert CACHE.main([
        'identity',
        '--bag-dir', str(inputs['bag']),
        '--config-dir', str(inputs['config']),
        '--runtime-kind', 'local-install',
        '--runtime-path', str(inputs['runtime']),
        '--runtime-token', 'ros=jazzy;arch=x86_64',
        '--harness', str(inputs['harness']),
        '--points-topic', '/points',
        '--imu-topic', '/imu',
        '--mode', 'lidar-imu',
        '--preset', 'cpu',
        '--no-imu', 'false',
        '--viewer', 'false',
        '--omp-threads', '1',
        '--output', str(identity_path),
    ]) == 0
    cache_key = capsys.readouterr().out.strip()
    assert len(cache_key) == 64
    assert CACHE.main([
        'store', *common,
        '--trajectory', str(inputs['trajectory']),
    ]) == 0
    capsys.readouterr()
    assert CACHE.main(['lookup', *common]) == 0
    assert capsys.readouterr().out.strip().endswith('.traj_lidar.txt')

    source = (ROOT / 'scripts' / 'compare_with_glim.sh').read_text()
    assert 'glim_cache_key()' not in source
    assert 'use_verified_glim_traj' in source
    assert 'glim_reference_cache.py' in source
    assert '--no-glim-cache' in source
    assert 'GLIM_CACHE_IDENTITY_READY' in source
    assert '${GLIM_CACHE_KEY}_traj_lidar.txt' not in source

    release_builder = (
        ROOT / 'scripts' / 'build_release_bundle.py'
    ).read_text(encoding='utf-8')
    assert "'scripts/compare_with_glim.sh'" in release_builder
    assert "'scripts/glim_reference_cache.py'" in release_builder
