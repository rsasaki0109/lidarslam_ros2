#!/usr/bin/env python3

"""Deterministic input and software identities for benchmark evidence."""

from __future__ import annotations

import hashlib
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


def file_identity(
    path: Path,
    display_path: str | None = None,
) -> dict[str, Any]:
    """Hash one stable file and return its portable identity."""
    path = path.expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'provenance file is missing: {path}')
    before = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    after = path.stat()
    if (
        before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
        or before.st_ino != after.st_ino
    ):
        raise RuntimeError(f'file changed while hashing: {path}')
    return {
        'path': display_path or str(path),
        'size_bytes': after.st_size,
        'sha256': digest.hexdigest(),
    }


def bag_identity(bag_path: Path) -> dict[str, Any]:
    """Hash rosbag2 metadata and every referenced storage file."""
    bag_path = bag_path.expanduser().resolve()
    metadata_path = bag_path / 'metadata.yaml'
    metadata_identity = file_identity(metadata_path, 'metadata.yaml')
    metadata_bytes = metadata_path.read_bytes()
    if (
        hashlib.sha256(metadata_bytes).hexdigest()
        != metadata_identity['sha256']
    ):
        raise RuntimeError(f'file changed after hashing: {metadata_path}')
    metadata = yaml.safe_load(metadata_bytes) or {}
    bag_info = metadata.get('rosbag2_bagfile_information') or {}
    relative_paths = bag_info.get('relative_file_paths') or []
    if not isinstance(relative_paths, list) or not all(
        isinstance(path, str) for path in relative_paths
    ):
        raise ValueError(
            'rosbag2 metadata relative_file_paths must be a list of strings'
        )
    if not relative_paths:
        relative_paths = [
            path.relative_to(bag_path).as_posix()
            for pattern in ('*.db3', '*.mcap')
            for path in sorted(bag_path.glob(pattern))
        ]

    storage_files: list[dict[str, Any]] = []
    seen: set[str] = set()
    for relative_path in relative_paths:
        path = (bag_path / relative_path).resolve()
        try:
            normalized = path.relative_to(bag_path).as_posix()
        except ValueError as exc:
            raise ValueError(
                'rosbag2 metadata references a file outside the bag: '
                f'{relative_path}'
            ) from exc
        if normalized in seen:
            continue
        seen.add(normalized)
        storage_files.append(file_identity(path, normalized))
    if not storage_files:
        raise ValueError(f'rosbag2 has no storage files: {bag_path}')

    return {
        'bag_path': str(bag_path),
        'metadata': metadata_identity,
        'storage_identifier': bag_info.get('storage_identifier'),
        'storage_files': storage_files,
        'identity_algorithm': 'sha256',
    }


def _git_state(repo_root: Path) -> dict[str, Any]:
    # CI commonly runs tests in a container while the checkout is owned by the
    # host runner. Trust only the repository path explicitly supplied by the
    # caller; otherwise Git's dubious-ownership guard makes valid provenance
    # silently degrade to null.
    git_command = ['git', '-c', f'safe.directory={repo_root}']
    commit = subprocess.run(
        [*git_command, 'rev-parse', 'HEAD'],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    status = subprocess.run(
        [*git_command, 'status', '--porcelain', '--untracked-files=normal'],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if commit.returncode != 0:
        detail = commit.stderr.strip() or 'unknown git error'
        raise RuntimeError(f'failed to identify benchmark git commit: {detail}')
    if status.returncode != 0:
        detail = status.stderr.strip() or 'unknown git error'
        raise RuntimeError(f'failed to identify benchmark git status: {detail}')
    return {
        'git_commit': commit.stdout.strip(),
        'git_dirty': bool(status.stdout.strip()),
    }


def software_identity(
    repo_root: Path,
    *,
    parameter_files: list[Path],
    runtime_artifacts: list[tuple[str, Path]],
    benchmark_harness: Path,
    metrics_writer: Path,
) -> dict[str, Any]:
    """Identify the source tree, configuration, harness, and runtime files."""
    repo_root = repo_root.expanduser().resolve()
    package_versions: dict[str, str] = {}
    package_xml_paths = list(repo_root.glob('*/package.xml'))
    package_xml_paths.extend(repo_root.glob('Thirdparty/*/package.xml'))
    for package_xml in sorted(package_xml_paths):
        try:
            root = ET.parse(package_xml).getroot()
        except ET.ParseError:
            continue
        name = root.findtext('name')
        version = root.findtext('version')
        if name and version:
            package_versions[name.strip()] = version.strip()
    version_path = repo_root / 'VERSION'
    product_version = (
        version_path.read_text(encoding='utf-8').strip()
        if version_path.is_file() else package_versions.get('lidarslam')
    )
    return {
        'product_version': product_version,
        **_git_state(repo_root),
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'package_versions': dict(sorted(package_versions.items())),
        'parameter_files': [
            file_identity(path) for path in parameter_files
        ],
        'runtime_artifacts': [
            {'label': label, **file_identity(path)}
            for label, path in runtime_artifacts
        ],
        'benchmark_harness': file_identity(benchmark_harness),
        'metrics_writer': file_identity(metrics_writer),
    }
