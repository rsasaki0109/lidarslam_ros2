#!/usr/bin/env python3
"""Build and verify one reproducible pre-tag release-bundle candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any
import uuid

from build_release_bundle import build_release_bundle
from check_published_release import verify_release_bundle_payload


REPO_ROOT = Path(__file__).resolve().parents[1]


def rehearse_release_bundle(
    root: Path,
    output: Path,
    *,
    tag: str,
    git_commit: str,
) -> dict[str, Any]:
    """Build twice, verify both archives, and publish one checked candidate."""
    output = output.resolve()
    if output.exists():
        raise ValueError(f'refusing to overwrite release bundle: {output}')

    version = (root / 'VERSION').read_text(encoding='utf-8').strip()
    with tempfile.TemporaryDirectory(
        prefix='lidarslam-release-bundle-',
    ) as temp_dir:
        temporary_root = Path(temp_dir)
        first = temporary_root / 'first.tar.gz'
        second = temporary_root / 'second.tar.gz'
        first_manifest = build_release_bundle(
            root,
            first,
            tag=tag,
            git_commit=git_commit,
        )
        second_manifest = build_release_bundle(
            root,
            second,
            tag=tag,
            git_commit=git_commit,
        )
        first_payload = first.read_bytes()
        second_payload = second.read_bytes()
        verify_release_bundle_payload(
            first_payload,
            version=version,
            tag=tag,
            commit=git_commit,
        )
        verify_release_bundle_payload(
            second_payload,
            version=version,
            tag=tag,
            commit=git_commit,
        )
        if first_manifest != second_manifest or first_payload != second_payload:
            raise ValueError(
                'repeated release-bundle builds are not byte-identical'
            )

        output.parent.mkdir(parents=True, exist_ok=True)
        temporary_output = output.with_name(
            f'.{output.name}.{uuid.uuid4().hex}.tmp'
        )
        try:
            shutil.copyfile(first, temporary_output)
            os.link(temporary_output, output)
        except FileExistsError as exc:
            raise ValueError(
                f'refusing to overwrite release bundle: {output}'
            ) from exc
        finally:
            temporary_output.unlink(missing_ok=True)

    return {
        'status': 'PASS',
        'bundle': str(output),
        'sha256': hashlib.sha256(first_payload).hexdigest(),
        'size_bytes': len(first_payload),
        'files': len(first_manifest['files']),
        'tag': tag,
        'git_commit': git_commit,
    }


def _git_output(root: Path, *args: str) -> str:
    return subprocess.run(
        ['git', *args],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'output',
        type=Path,
        help='Path for the verified candidate .tar.gz (must not exist)',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if _git_output(REPO_ROOT, 'status', '--porcelain'):
            raise ValueError(
                'release-bundle rehearsal requires a clean source worktree'
            )
        version = (REPO_ROOT / 'VERSION').read_text(
            encoding='utf-8'
        ).strip()
        tag = f'v{version}'
        git_commit = _git_output(REPO_ROOT, 'rev-parse', 'HEAD')
        tagged_commit = subprocess.run(
            [
                'git',
                'rev-parse',
                '--verify',
                f'refs/tags/{tag}^{{commit}}',
            ],
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        if (
            tagged_commit.returncode == 0
            and tagged_commit.stdout.strip() != git_commit
        ):
            raise ValueError(
                f'tag {tag} already names {tagged_commit.stdout.strip()}, '
                f'not HEAD {git_commit}'
            )
        report = rehearse_release_bundle(
            REPO_ROOT,
            args.output,
            tag=tag,
            git_commit=git_commit,
        )
    except (OSError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'release-bundle rehearsal error: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
