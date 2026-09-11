#!/usr/bin/env python3
"""Deterministic provenance for a dirty competitive candidate worktree."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any, Iterable


SOURCE_ROOTS = (
    'Thirdparty/rko_lio', 'configs', 'docker', 'graph_based_slam',
    'lidarslam', 'lidarslam_msgs', 'scanmatcher', 'scripts', 'tools')
IGNORED_DIRS = {'.git', '.pytest_cache', '__pycache__', 'build', 'install', 'log', 'output'}
IGNORED_SUFFIXES = {'.o', '.pyc', '.so', '.swp'}


def _candidate_files(root: Path, source_roots: Iterable[str]) -> list[Path]:
    files: list[Path] = []
    for source_root in source_roots:
        base = root / source_root
        if not base.exists():
            continue
        if base.is_file() or base.is_symlink():
            files.append(base)
            continue
        for path in base.rglob('*'):
            relative = path.relative_to(root)
            if any(part in IGNORED_DIRS for part in relative.parts):
                continue
            if path.is_dir() or path.suffix in IGNORED_SUFFIXES:
                continue
            files.append(path)
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def source_tree_digest(
        root: Path, source_roots: Iterable[str] = SOURCE_ROOTS) -> dict[str, Any]:
    root = root.resolve()
    digest = hashlib.sha256()
    files = _candidate_files(root, source_roots)
    for path in files:
        relative = path.relative_to(root).as_posix().encode()
        digest.update(len(relative).to_bytes(8, 'big'))
        digest.update(relative)
        if path.is_symlink():
            payload = ('symlink:' + str(path.readlink())).encode()
        else:
            payload = path.read_bytes()
        digest.update(len(payload).to_bytes(8, 'big'))
        digest.update(payload)
    return {'sha256': digest.hexdigest(), 'file_count': len(files),
            'source_roots': list(source_roots)}


def git_revision(path: Path) -> str:
    return subprocess.run(
        ['git', '-C', str(path), 'rev-parse', 'HEAD'], check=True,
        text=True, capture_output=True).stdout.strip()


def verify_candidate_manifest(root: Path, manifest_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema_version') != 1:
        raise ValueError('candidate manifest schema_version must be 1')
    recorded = manifest.get('source_tree', {})
    current = source_tree_digest(root, recorded.get('source_roots', SOURCE_ROOTS))
    if current['sha256'] != recorded.get('sha256'):
        raise ValueError(
            'candidate source tree hash mismatch: '
            f"expected {recorded.get('sha256')}, got {current['sha256']}")
    return {
        'path': str(manifest_path),
        'sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        'source_tree_sha256': current['sha256'],
        'source_file_count': current['file_count'],
    }


if __name__ == "__main__":
    raise SystemExit("competitive_candidate_provenance is a library module; import it instead of running it directly.")
