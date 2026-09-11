#!/usr/bin/env python3
"""Create a privacy-conscious operational manifest for a map-authoring run."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 2


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _git_state(repo_root: Path) -> dict[str, Any]:
    def git(*args: str) -> str | None:
        result = subprocess.run(
            ['git', '-C', str(repo_root), *args],
            check=False,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip() if result.returncode == 0 else None

    revision = git('rev-parse', 'HEAD')
    status = git('status', '--porcelain', '--untracked-files=no')
    untracked = git('ls-files', '--others', '--exclude-standard')
    diff = subprocess.run(
        [
            'git', '-C', str(repo_root), 'diff', '--binary', '--no-ext-diff',
            'HEAD', '--', '.',
        ],
        check=False,
        capture_output=True,
    )
    tracked_diff_sha256 = (
        hashlib.sha256(diff.stdout).hexdigest() if diff.returncode == 0 else None
    )
    return {
        'revision': revision,
        'tracked_worktree_dirty': bool(status),
        'tracked_diff_sha256': tracked_diff_sha256,
        'untracked_files_present': bool(untracked),
        'identity_strength': 'revision_plus_tracked_binary_diff_sha256',
        'note': (
            'Untracked file contents are not included; use the release evidence '
            'source manifest for publication claims.'
        ),
    }


def _portable_command(
    command: list[str], repo_root: Path, bag_path: Path, output_dir: Path,
) -> list[str]:
    replacements = (
        (str(output_dir), '${OUTPUT_DIR}'),
        (str(bag_path), '${BAG_DIR}'),
        (str(repo_root), '${REPO_ROOT}'),
    )
    result = []
    for argument in command:
        portable = argument
        for original, replacement in replacements:
            portable = portable.replace(original, replacement)
        result.append(portable)
    return result


def _input_identity(bag_path: Path) -> dict[str, Any]:
    metadata = bag_path / 'metadata.yaml'
    if metadata.is_symlink() or not metadata.is_file():
        raise ValueError('bag metadata must be a regular non-symlink file')
    metadata_record = {
        'name': metadata.name,
        'size_bytes': metadata.stat().st_size,
        'sha256': sha256_file(metadata),
    }
    storage_files = []
    for path in sorted(bag_path.iterdir()):
        if path.name == 'metadata.yaml':
            continue
        if path.is_symlink():
            raise ValueError(f'bag storage entry must not be a symlink: {path.name}')
        if path.is_file():
            storage_files.append({
                'name': path.name,
                'size_bytes': path.stat().st_size,
                'sha256': sha256_file(path),
            })
    tree_projection = {
        'metadata': metadata_record,
        'storage_files': storage_files,
    }
    tree_sha256 = hashlib.sha256(json.dumps(
        tree_projection,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')).hexdigest()
    return {
        'name': bag_path.name,
        'metadata': metadata_record,
        'storage_files': storage_files,
        'tree_sha256': tree_sha256,
        'identity_strength': 'metadata_and_all_storage_bytes_sha256',
        'tree_hash_contract': 'canonical-json-metadata-and-ordered-storage-records-v1',
    }


def _config_identities(command: list[str], repo_root: Path) -> list[dict[str, Any]]:
    configs = []
    seen: set[Path] = set()
    for argument in command:
        path = Path(argument)
        if path.suffix not in {'.yaml', '.yml', '.json'} or not path.is_file():
            continue
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            portable_path = str(resolved.relative_to(repo_root))
        except ValueError:
            portable_path = resolved.name
        configs.append({
            'path': portable_path,
            'sha256': sha256_file(resolved),
            'size_bytes': resolved.stat().st_size,
        })
    return configs


def _output_inventory(output_dir: Path) -> list[dict[str, Any]]:
    inventory = []
    manifest_name = 'map_run_manifest.json'
    for path in sorted(output_dir.rglob('*')):
        if path.is_symlink():
            raise ValueError(
                f'output inventory entry must not be a symlink: '
                f'{path.relative_to(output_dir).as_posix()}'
            )
        if not path.is_file() or path.name == manifest_name:
            continue
        stat = path.stat()
        if stat.st_nlink != 1:
            raise ValueError(
                f'output inventory entry must be single-link: '
                f'{path.relative_to(output_dir).as_posix()}'
            )
        record: dict[str, Any] = {
            'path': path.relative_to(output_dir).as_posix(),
            'size_bytes': stat.st_size,
            'sha256': sha256_file(path),
        }
        inventory.append(record)
    return inventory


def build_manifest(
    *,
    repo_root: Path,
    output_dir: Path,
    bag_path: Path,
    profile_id: str,
    command: list[str],
    status: str,
) -> dict[str, Any]:
    return {
        'schema_version': SCHEMA_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'status': status,
        'workflow': {
            'profile_id': profile_id,
            'command': _portable_command(command, repo_root, bag_path, output_dir),
        },
        'source': _git_state(repo_root),
        'environment': {
            'ros_distro': os.environ.get('ROS_DISTRO'),
        },
        'input': _input_identity(bag_path),
        'configs': _config_identities(command, repo_root),
        'outputs': _output_inventory(output_dir),
    }


def write_manifest(
    *,
    repo_root: Path,
    output_dir: Path,
    bag_path: Path,
    profile_id: str,
    command: list[str],
    status: str,
) -> Path:
    manifest = build_manifest(
        repo_root=repo_root,
        output_dir=output_dir,
        bag_path=bag_path,
        profile_id=profile_id,
        command=command,
        status=status,
    )
    target = output_dir / 'map_run_manifest.json'
    temporary = output_dir / '.map_run_manifest.json.tmp'
    temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, target)
    return target


if __name__ == "__main__":
    raise SystemExit("map_run_manifest is a library module; import it instead of running it directly.")
