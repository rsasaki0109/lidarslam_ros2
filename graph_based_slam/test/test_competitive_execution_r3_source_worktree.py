#!/usr/bin/env python3
# flake8: noqa
"""Tests for the non-promoting r3 source-worktree snapshot contract."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import stat

from jsonschema import Draft7Validator, RefResolver
import pytest

import scripts.capture_competitive_execution_r3_source_worktree as snapshot
from scripts import capture_competitive_execution_identity as identity


REVISION = 'a' * 40


def _contract():
    candidate = {
        'path': 'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08-r3-candidate.json',
        'file_sha256': '1' * 64,
        'candidate_identity_sha256': '2' * 64,
        'candidate_id': 'competitive-execution-selection-2026-08-r3-candidate',
        'status': 'NOT_READY',
    }
    profile = {
        'path': 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml',
        'file_sha256': '3' * 64,
        'canonical_sha256': '4' * 64,
        'canonical_hash_kind': 'canonical_profile_sha256_v1',
    }
    bindings = {
        'runner': {'ours': {'path': 'scripts/ours.py', 'sha256': '5' * 64}},
        'scorer': {'ape': {'path': 'scripts/ape.py', 'sha256': '6' * 64}},
        'memory': {'rss': {'path': 'scripts/rss.py', 'sha256': '7' * 64}},
    }
    source = {
        'schema': 'competitive_execution_selection_r3_source_manifest_v1',
        'schema_version': 1,
        'candidate_path': candidate['path'],
        'candidate_file_sha256': candidate['file_sha256'],
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'source_bindings_sha256': identity.canonical_hash(bindings),
        'source_bindings': bindings,
        'source_provenance_status': 'DIRTY_EXTERNAL_REVIEW_REQUIRED',
        'producer_bindings': {
            'snapshot_producer': {
                'path': snapshot.SCRIPT_REL,
                'file_sha256': hashlib.sha256(snapshot.SCRIPT.read_bytes()).hexdigest(),
                'schema_path': snapshot.SCHEMA_REL,
            },
            'snapshot_schema': {
                'path': snapshot.SCHEMA_REL,
                'file_sha256': hashlib.sha256(
                    (snapshot.ROOT / snapshot.SCHEMA_REL).read_bytes()).hexdigest(),
            },
            'snapshot_strict_schema': {
                'path': snapshot.STRICT_SCHEMA_REL,
                'file_sha256': hashlib.sha256(
                    (snapshot.ROOT / snapshot.STRICT_SCHEMA_REL).read_bytes()).hexdigest(),
            },
            'snapshot_sidecar_schema': {
                'path': snapshot.SIDECAR_SCHEMA_REL,
                'file_sha256': hashlib.sha256(
                    (snapshot.ROOT / snapshot.SIDECAR_SCHEMA_REL).read_bytes()).hexdigest(),
            },
            'snapshot_implementation': {
                'path': snapshot.IMPLEMENTATION_REL,
                'file_sha256': hashlib.sha256(
                    snapshot.IMPLEMENTATION.read_bytes()).hexdigest(),
            },
            'identity_contract': {
                'path': snapshot.IDENTITY_CONTRACT_REL,
                'file_sha256': hashlib.sha256(
                    (snapshot.ROOT / snapshot.IDENTITY_CONTRACT_REL).read_bytes()).hexdigest(),
            },
        },
    }
    return candidate, profile, source


def _runner(root: Path, *, listed: str = 'src/a.txt\n', dirty: bool = True):
    diff = 'diff --git a/tracked b/tracked\n+changed\n'
    cached = ''

    def run(command: list[str], cwd: Path):
        assert command[:3] == ['git', '-C', str(root)]
        args = tuple(command[3:])
        if args == ('rev-parse', 'HEAD'):
            return 0, REVISION + '\n', ''
        if args == ('status', '--porcelain=v1', '--untracked-files=all'):
            return 0, ((' M tracked\n' if dirty else '') + listed), ''
        if args == ('diff', '--binary', '--no-ext-diff'):
            return 0, diff, ''
        if args == ('diff', '--cached', '--binary', '--no-ext-diff'):
            return 0, cached, ''
        if args == ('ls-files', '--others', '--exclude-standard'):
            return 0, listed, ''
        if args == ('submodule', 'status', '--recursive'):
            return 0, '', ''
        raise AssertionError(command)

    return run


def _prepare(tmp_path: Path, monkeypatch, *, dirty: bool = True,
             listed: str = 'src/a.txt\n'):
    root = tmp_path / 'checkout'
    (root / 'src').mkdir(parents=True)
    (root / 'src' / 'a.txt').write_bytes(b'source fixture\n')
    monkeypatch.setattr(snapshot, '_candidate_contract', _contract)
    runner = _runner(root, dirty=dirty, listed=listed)
    output = tmp_path / 'snapshot.json'
    document = snapshot.capture_snapshot(
        root=root, output=output, runner=runner,
        captured_at='2026-08-26T00:00:00Z')
    return root, output, document, runner


def test_clean_or_dirty_snapshot_round_trips_and_reuses_hash_contract(tmp_path, monkeypatch):
    root, output, document, runner = _prepare(tmp_path, monkeypatch)
    result = snapshot.validate_snapshot(output, root=root, runner=runner)
    assert result['structural_valid'] is True
    assert result['status'] == 'NOT_REVIEWED_EXTERNAL'
    assert result['benchmark_eligible'] is False
    assert document['worktree']['worktree_dirty'] is True
    row = document['worktree']['untracked_files'][0]
    assert row == {
        'path': 'src/a.txt',
        'sha256': hashlib.sha256(b'source fixture\n').hexdigest(),
        'bytes': len(b'source fixture\n'),
    }
    assert document['worktree']['untracked_content_sha256'] == identity.canonical_hash([{
        'path': 'src/a.txt', 'sha256': row['sha256']}])
    assert document['worktree']['tracked_diff_sha256'] == identity.sha256_bytes(
        ('diff --git a/tracked b/tracked\n+changed\n').encode())
    assert stat.S_IMODE(output.stat().st_mode) == 0o444
    assert stat.S_IMODE(Path(str(output) + '.sha256.json').stat().st_mode) == 0o444


def test_clean_snapshot_round_trips_without_untracked_rows(tmp_path, monkeypatch):
    root, output, document, runner = _prepare(
        tmp_path, monkeypatch, dirty=False, listed='')
    result = snapshot.validate_snapshot(output, root=root, runner=runner)
    assert result['structural_valid'] is True
    assert document['worktree']['worktree_dirty'] is False
    assert document['worktree']['untracked_files'] == []
    assert document['worktree']['untracked_file_count'] == 0


def test_schema_and_sidecar_are_strict(tmp_path, monkeypatch):
    _root, output, document, _runner_value = _prepare(tmp_path, monkeypatch)
    schema_path = snapshot.ROOT / snapshot.SCHEMA_REL
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    Draft7Validator.check_schema(schema)
    # Resolve the alias schema against the checked-in implementation schema.
    implementation = json.loads((snapshot.ROOT / (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_source_worktree_snapshot_v1.schema.json')).read_text())
    validator = Draft7Validator(
        implementation,
        resolver=RefResolver(schema_path.as_uri(), implementation))
    assert not list(validator.iter_errors(document))
    sidecar = json.loads(Path(str(output) + '.sha256.json').read_text())
    assert sidecar['artifact_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'traversal'])
def test_untracked_aliases_fail_closed(tmp_path, monkeypatch, kind):
    root = tmp_path / 'checkout'
    (root / 'src').mkdir(parents=True)
    (root / 'src' / 'a.txt').write_bytes(b'source fixture\n')
    if kind == 'symlink':
        (root / 'src' / 'alias.txt').symlink_to('a.txt')
        listed = 'src/alias.txt\n'
    elif kind == 'hardlink':
        (root / 'src' / 'alias.txt').hardlink_to(root / 'src' / 'a.txt')
        listed = 'src/alias.txt\n'
    else:
        listed = '../outside.txt\n'
    monkeypatch.setattr(snapshot, '_candidate_contract', _contract)
    with pytest.raises(snapshot.SourceWorktreeError):
        snapshot.capture_snapshot(
            root=root, output=tmp_path / 'snapshot.json',
            runner=_runner(root, listed=listed),
            captured_at='2026-08-26T00:00:00Z')


def test_tampered_immutable_pair_and_collision_fail_closed(tmp_path, monkeypatch):
    root, output, _document, runner = _prepare(tmp_path, monkeypatch)
    with pytest.raises(snapshot.SourceWorktreeError, match='immutable'):
        output.chmod(0o644)
        snapshot.validate_snapshot(output, root=root, runner=runner)
    output.chmod(0o444)
    with pytest.raises(snapshot.SourceWorktreeError, match='already exists'):
        snapshot.capture_snapshot(
            root=root, output=output, runner=runner,
            captured_at='2026-08-26T00:00:00Z')


def test_root_output_inside_worktree_is_rejected(tmp_path, monkeypatch):
    root = tmp_path / 'checkout'
    root.mkdir()
    monkeypatch.setattr(snapshot, '_candidate_contract', _contract)
    with pytest.raises(snapshot.SourceWorktreeError, match='outside'):
        snapshot.capture_snapshot(
            root=root, output=root / 'snapshot.json', runner=_runner(root))
