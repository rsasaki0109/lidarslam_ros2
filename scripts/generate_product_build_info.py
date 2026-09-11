#!/usr/bin/env python3
"""Generate deterministic source-revision metadata for installed products."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Sequence


REVISION_PATTERN = re.compile(r'^[0-9a-f]{40}$')


def _normalize_revision(value: str) -> str:
    revision = value.strip().lower()
    if not REVISION_PATTERN.fullmatch(revision):
        raise ValueError('source revision must be exactly 40 hexadecimal characters')
    return revision


def _parse_dirty(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError('source dirty override must be "true" or "false"')


def _run_git(source_dir: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            'git',
            '-c',
            f'safe.directory={source_dir}',
            *args,
        ],
        cwd=source_dir,
        check=False,
        capture_output=True,
        text=True,
    )


def _git_state(source_dir: Path) -> tuple[str, bool] | None:
    try:
        top_level = _run_git(source_dir, 'rev-parse', '--show-toplevel')
        if top_level.returncode != 0:
            return None
        if Path(top_level.stdout.strip()).resolve() != source_dir.resolve():
            return None
        commit = _run_git(source_dir, 'rev-parse', 'HEAD')
        status = _run_git(
            source_dir,
            'status',
            '--porcelain',
            '--untracked-files=no',
        )
    except OSError:
        return None
    if commit.returncode != 0 or status.returncode != 0:
        return None
    return _normalize_revision(commit.stdout), bool(status.stdout.strip())


def build_info(
    source_dir: Path,
    revision_override: str | None = None,
    dirty_override: str | None = None,
) -> dict[str, object]:
    """Resolve explicit build provenance first, then a repository checkout."""
    if dirty_override is not None and revision_override is None:
        raise ValueError('source dirty override requires a source revision override')
    if revision_override is not None:
        return {
            'schema_version': 1,
            'revision': _normalize_revision(revision_override),
            'dirty': (
                _parse_dirty(dirty_override)
                if dirty_override is not None
                else None
            ),
            'source': 'override',
        }

    state = _git_state(source_dir)
    if state is None:
        return {
            'schema_version': 1,
            'revision': None,
            'dirty': None,
            'source': 'unknown',
        }
    revision, dirty = state
    return {
        'schema_version': 1,
        'revision': revision,
        'dirty': dirty,
        'source': 'git',
    }


def write_build_info(output_path: Path, payload: dict[str, object]) -> None:
    """Write atomically and avoid changing the file when content is identical."""
    rendered = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if output_path.is_file():
        try:
            if output_path.read_text(encoding='utf-8') == rendered:
                return
        except OSError:
            pass
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f'.{output_path.name}.tmp')
    temporary.write_text(rendered, encoding='utf-8')
    temporary.replace(output_path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the build-system interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--revision')
    parser.add_argument('--dirty')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Generate build information or reject an invalid explicit identity."""
    args = parse_args(argv)
    try:
        payload = build_info(
            args.source_dir.expanduser().resolve(),
            args.revision,
            args.dirty,
        )
        write_build_info(args.output.expanduser().resolve(), payload)
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
