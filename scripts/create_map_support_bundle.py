#!/usr/bin/env python3
"""Create a small, redacted support bundle from a map-authoring run."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import re
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TEXT_ARTIFACTS = (
    'map_run_manifest.json',
    'autoware_map_diagnosis.json',
    'autoware_map_diagnosis.md',
    'verify_autoware_map.log',
    'map_save.log',
    'lidarslam.launch.log',
    'slam.launch.log',
)
MAX_TEXT_BYTES = 256 * 1024


def _redact(text: str, run_dir: Path) -> str:
    replacements = (
        (str(run_dir), '${RUN_DIR}'),
        (str(Path.home()), '${HOME}'),
    )
    for source, replacement in replacements:
        if source and source != '/':
            text = text.replace(source, replacement)
    text = re.sub(r'/home/[^/\s"\']+', '${HOME}', text)
    return text


def _tail_text(path: Path) -> tuple[str, bool]:
    data = path.read_bytes()
    truncated = len(data) > MAX_TEXT_BYTES
    if truncated:
        data = data[-MAX_TEXT_BYTES:]
    text = data.decode('utf-8', errors='replace')
    if truncated:
        text = '[... earlier log content omitted ...]\n' + text
    return text, truncated


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def build_bundle_entries(run_dir: Path) -> tuple[dict[str, bytes], list[dict[str, Any]]]:
    entries: dict[str, bytes] = {}
    inventory = []
    for name in TEXT_ARTIFACTS:
        source = run_dir / name
        if not source.is_file():
            continue
        text, truncated = _tail_text(source)
        payload = _redact(text, run_dir).encode('utf-8')
        archive_name = f'support/{name}'
        entries[archive_name] = payload
        inventory.append({
            'path': archive_name,
            'sha256': _sha256(payload),
            'size_bytes': len(payload),
            'tail_truncated': truncated,
        })
    return entries, inventory


def create_bundle(run_dir: Path, output: Path) -> Path:
    run_dir = run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise ValueError(f'run directory does not exist or is not a directory: {run_dir}')
    entries, inventory = build_bundle_entries(run_dir)
    if not entries:
        raise ValueError(
            'no supported diagnostics were found; run diagnose_autoware_map_run.py --write first'
        )
    bundle_manifest = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_run_name': run_dir.name,
        'privacy': {
            'absolute_home_and_run_paths_redacted': True,
            'bags_maps_and_pointclouds_included': False,
            'logs_are_tail_limited_bytes': MAX_TEXT_BYTES,
        },
        'files': inventory,
    }
    entries['support/support_bundle_manifest.json'] = (
        json.dumps(bundle_manifest, indent=2, sort_keys=True) + '\n'
    ).encode('utf-8')

    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f'.{output.name}.tmp')
    with tarfile.open(temporary, mode='w:gz') as archive:
        for name, payload in sorted(entries.items()):
            info = tarfile.TarInfo(name=name)
            info.size = len(payload)
            info.mode = 0o644
            info.mtime = 0
            archive.addfile(info, io.BytesIO(payload))
    temporary.replace(output)
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument(
        '--output',
        type=Path,
        help='Archive path (default: <run_dir>_support.tar.gz beside the run).',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    output = args.output or run_dir.with_name(f'{run_dir.name}_support.tar.gz')
    try:
        result = create_bundle(run_dir, output)
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    print(f'support bundle: {result}')
    print('privacy: absolute home/run paths redacted; bags, maps, and point clouds omitted')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

