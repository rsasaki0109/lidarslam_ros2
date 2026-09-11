#!/usr/bin/env python3
"""Render the contract-bound social card without unverified benchmark copy."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import tempfile
from typing import Sequence

import generate_social_autoware_demo_video as media


DEFAULT_OUTPUT = (
    media.REPO_ROOT
    / 'lidarslam'
    / 'images'
    / 'social_autoware_map_authoring.png'
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        '--slide',
        default='promise',
        choices=('promise', 'beginner-path', 'own-bag', 'proof'),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output = args.out.expanduser().resolve()
    try:
        if output.is_symlink():
            raise media.MediaError(f'refusing symlink output: {output}')
        if output.parent.is_symlink() or not output.parent.is_dir():
            raise media.MediaError(
                f'output parent must be a regular directory: {output.parent}'
            )
        contract = media.load_contract()
        slide = next(
            item for item in contract['slides'] if item['id'] == args.slide
        )
        card = media.render_slide(contract, slide, size=(1600, 900))
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{output.name}.',
            suffix='.tmp',
            dir=output.parent,
        )
        os.close(descriptor)
        temporary = Path(temporary_name)
        try:
            card.save(temporary, format='PNG')
            os.replace(temporary, output)
        except Exception:
            temporary.unlink(missing_ok=True)
            raise
    except (media.MediaError, OSError, StopIteration) as exc:
        print(f'social card error: {exc}', file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
