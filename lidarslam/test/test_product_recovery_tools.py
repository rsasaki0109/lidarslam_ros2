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

"""Safety contracts for manifest migration and immutable image rollback."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from migrate_run_manifest import migrate_file  # noqa: E402, I100
from plan_image_rollback import build_rollback_plan  # noqa: E402
from product_schema import validate_contract  # noqa: E402


def _manifest(status: str = 'succeeded') -> dict[str, object]:
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v1.schema.json'
        ),
        'run_id': '12345678-1234-4234-8234-123456789abc',
        'status': status,
        'input': {
            'bag_path': '/data/bag',
            'metadata_path': '/data/bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [
                {'path': 'bag.db3', 'size_bytes': 100, 'sha256': 'b' * 64}
            ],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.6.0',
            'git_commit': 'c' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.6.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {'id': 'fixture', 'label': 'Fixture'},
        'execution': {
            'argv': ['lidarslam-map', 'run', '/data/bag'],
            'command_shell': 'lidarslam-map run /data/bag',
            'started_at': '2026-07-29T00:00:00Z',
            'finished_at': '2026-07-29T00:01:00Z',
            'exit_code': 0,
        },
        'output': {
            'requested_dir': '/data/map',
            'working_dir': '/data/map.partial',
            'finalized': True,
            'diagnosis_status': 'success',
            'artifact_checksums': [],
        },
    }


def _release_record() -> dict[str, object]:
    return {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': 'jazzy',
        'platform': 'linux/amd64',
        'tag': 'ghcr.io/example/lidar_slam_ros2:v0.6.0-jazzy',
        'digest': f"sha256:{'d' * 64}",
        'git_commit': 'e' * 40,
        'product_version': '0.6.0',
        'cli_version': 'lidarslam_ros2 0.6.0',
    }


def _write_manifest(run_dir: Path, payload: dict[str, object]) -> Path:
    run_dir.mkdir()
    path = run_dir / 'run_manifest.json'
    path.write_text(json.dumps(payload), encoding='utf-8')
    return path


@pytest.mark.parametrize('verification_enabled', [True, False])
def test_migration_is_explicit_inspect_only_and_preserves_source(
    tmp_path: Path,
    verification_enabled: bool,
):
    source = _write_manifest(tmp_path / 'run', _manifest())
    original = source.read_bytes()
    destination = tmp_path / 'migrated.json'

    result = migrate_file(
        source.parent,
        destination,
        verification_enabled=verification_enabled,
    )
    migrated = json.loads(destination.read_text(encoding='utf-8'))

    validate_contract(migrated, 'run-manifest-v2.schema.json')
    assert source.read_bytes() == original
    assert migrated['lifecycle']['stage'] == 'complete'
    assert migrated['lifecycle']['verification_enabled'] is verification_enabled
    assert result['resume_allowed'] is False
    assert result['source_sha256'] != ''
    assert result['destination_sha256'] != ''


@pytest.mark.parametrize('status', ['planned', 'running'])
def test_migration_refuses_nonterminal_state(tmp_path: Path, status: str):
    run_dir = tmp_path / 'run'
    _write_manifest(run_dir, _manifest(status))

    with pytest.raises(ValueError, match='only terminal'):
        migrate_file(
            run_dir,
            tmp_path / 'migrated.json',
            verification_enabled=True,
        )


def test_migration_refuses_missing_terminal_execution_state(tmp_path: Path):
    payload = _manifest('failed')
    payload['execution']['finished_at'] = None
    payload['execution']['exit_code'] = None
    run_dir = tmp_path / 'run'
    _write_manifest(run_dir, payload)

    with pytest.raises(ValueError, match='finished_at'):
        migrate_file(
            run_dir,
            tmp_path / 'migrated.json',
            verification_enabled=False,
        )


def test_migration_never_replaces_source_or_existing_destination(
    tmp_path: Path,
):
    run_dir = tmp_path / 'run'
    source = _write_manifest(run_dir, _manifest())
    with pytest.raises(ValueError, match='must not replace'):
        migrate_file(run_dir, source, verification_enabled=True)

    destination = tmp_path / 'existing.json'
    destination.write_text('keep me', encoding='utf-8')
    with pytest.raises(FileExistsError, match='refusing overwrite'):
        migrate_file(run_dir, destination, verification_enabled=True)
    assert destination.read_text(encoding='utf-8') == 'keep me'


def test_migration_refuses_broken_destination_symlink(tmp_path: Path):
    run_dir = tmp_path / 'run'
    _write_manifest(run_dir, _manifest())
    destination = tmp_path / 'linked.json'
    destination.symlink_to(tmp_path / 'missing-target.json')

    with pytest.raises(FileExistsError, match='refusing overwrite'):
        migrate_file(run_dir, destination, verification_enabled=True)

    assert destination.is_symlink()
    assert not (tmp_path / 'missing-target.json').exists()


def test_migration_refuses_destination_created_during_publish(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    run_dir = tmp_path / 'run'
    _write_manifest(run_dir, _manifest())
    destination = tmp_path / 'raced.json'
    real_link = __import__('os').link

    def race_link(source: Path, target: Path) -> None:
        Path(target).write_text('other process', encoding='utf-8')
        real_link(source, target)

    monkeypatch.setattr('migrate_run_manifest.os.link', race_link)
    with pytest.raises(FileExistsError):
        migrate_file(run_dir, destination, verification_enabled=True)

    assert destination.read_text(encoding='utf-8') == 'other process'
    assert not (tmp_path / '.raced.json.tmp').exists()


def test_migration_cli_requires_explicit_verification(tmp_path: Path):
    run_dir = tmp_path / 'run'
    _write_manifest(run_dir, _manifest())
    completed = subprocess.run(
        [
            str(REPO_ROOT / 'scripts' / 'lidarslam'),
            'migrate-manifest',
            str(run_dir),
            '--output',
            str(tmp_path / 'migrated.json'),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert '--verification' in completed.stderr


def test_rollback_plan_uses_only_digest_reference():
    record = _release_record()
    plan = build_rollback_plan(
        record,
        source_record=Path('/download/release-image-jazzy.json'),
    )

    validate_contract(plan, 'rollback-plan-v1.schema.json')
    immutable = f"ghcr.io/example/lidar_slam_ros2@{record['digest']}"
    assert plan['source_record'] == 'release-image-jazzy.json'
    assert plan['immutable_ref'] == immutable
    assert plan['moving_tag_mutated'] is False
    assert all(immutable in command for command in plan['commands'].values())
    assert record['tag'] not in ' '.join(plan['commands'].values())


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('product_version', '0.7.0', 'version does not match'),
        ('ros_distro', 'humble', 'distro does not match'),
        ('cli_version', 'lidarslam_ros2 0.7.0', 'cli_version'),
        ('digest', 'sha256:bad', 'validation failed'),
    ],
)
def test_rollback_plan_rejects_inconsistent_evidence(
    field: str,
    value: str,
    message: str,
):
    record = _release_record()
    record[field] = value

    with pytest.raises(ValueError, match=message):
        build_rollback_plan(
            record,
            source_record=Path('release-image-jazzy.json'),
        )
