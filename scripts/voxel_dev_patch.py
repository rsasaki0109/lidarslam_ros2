#!/usr/bin/env python3
"""Generate and validate Voxel-SLAM development patches.

Each SOTA-v6 screen is a small delta over the frozen v17 candidate
(`voxel_slam_sota_v6_weak_axis_bridge`). This tool materializes those
deltas as unified patches in the repository (`docker/patches/
voxel_slam_dev/`) so every experiment is reproducible from the base
revision plus one patch, instead of a per-version source copy.

Usage:
  python3 scripts/voxel_dev_patch.py list
  python3 scripts/voxel_dev_patch.py write --name v23 \
      --source /path/to/voxel_slam_sota_v6_v23/voxelslam.cpp
  python3 scripts/voxel_dev_patch.py check
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PATCH_DIR = ROOT / 'docker/patches/voxel_slam_dev'
BASE_SOURCE = (
    ROOT.parent / 'voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def diff_unified(base: Path, modified: Path) -> bytes:
    """Return a unified diff with repository-relative paths."""
    proc = subprocess.run(
        ['diff', '-u', '--label', 'a/VoxelSLAM/src/voxelslam.cpp',
         '--label', 'b/VoxelSLAM/src/voxelslam.cpp',
         str(base), str(modified)],
        capture_output=True, text=True, check=False)
    if proc.returncode not in (0, 1):
        raise ValueError(f'diff failed: {proc.stderr}')
    return proc.stdout.encode()


def manifest_entry(name: str, source: Path) -> dict[str, Any]:
    patch = PATCH_DIR / f'{name}.patch'
    return {
        'name': name, 'source': str(source),
        'patch': str(patch.relative_to(ROOT)),
        'patch_sha256': sha256(patch) if patch.exists() else None,
        'base_sha256': sha256(BASE_SOURCE) if BASE_SOURCE.exists() else None,
    }


def write_patch(name: str, source: Path) -> dict[str, Any]:
    if not BASE_SOURCE.exists():
        raise ValueError(f'base source absent: {BASE_SOURCE}')
    if not source.is_file():
        raise ValueError(f'modified source absent: {source}')
    PATCH_DIR.mkdir(parents=True, exist_ok=True)
    raw = diff_unified(BASE_SOURCE, source)
    if not raw:
        raise ValueError(f'{name}: source equals base (no patch)')
    patch_path = PATCH_DIR / f'{name}.patch'
    patch_path.write_bytes(raw)
    entry = manifest_entry(name, source)
    print(json.dumps(entry, indent=2))
    return entry


def list_patches() -> None:
    if not PATCH_DIR.exists():
        print('no patches')
        return
    for patch in sorted(PATCH_DIR.glob('*.patch')):
        print(f'{patch.stem}: {patch.stat().st_size} bytes')


def check_patches() -> None:
    """Verify every patch applies cleanly to the base source.

    The base source lives outside the repository; apply the patch in a
    temporary directory that mirrors the `a/VoxelSLAM/src/` path so the
    repository-relative patch headers resolve.
    """
    if not BASE_SOURCE.exists():
        raise ValueError(f'base source absent: {BASE_SOURCE}')
    failed = []
    if not PATCH_DIR.exists():
        print('no patches to check')
        return
    import tempfile
    for patch in sorted(PATCH_DIR.glob('*.patch')):
        with tempfile.TemporaryDirectory() as td:
            target = Path(td) / 'VoxelSLAM/src/voxelslam.cpp'
            target.parent.mkdir(parents=True)
            shutil.copy2(BASE_SOURCE, target)
            applied = subprocess.run(
                ['patch', '-p1', '-i', str(patch)],
                cwd=td, capture_output=True, text=True, check=False)
        status = 'ok' if applied.returncode == 0 else 'FAIL'
        if applied.returncode != 0:
            failed.append(patch.name)
        print(f'{patch.name}: {status}')
    if failed:
        raise ValueError(f'patches failed to apply: {failed}')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='command', required=True)
    sub.add_parser('list')
    sub.add_parser('check')
    w = sub.add_parser('write')
    w.add_argument('--name', required=True)
    w.add_argument('--source', type=Path, required=True)
    args = ap.parse_args()

    if args.command == 'list':
        list_patches()
    elif args.command == 'check':
        check_patches()
    else:
        write_patch(args.name, args.source.resolve())
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
