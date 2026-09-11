#!/usr/bin/env python3
"""Create a privacy-bounded first-map receipt from product run artifacts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from first_map_validation_receipt import (
    ReceiptError,
    build_receipt,
    render_markdown,
    write_receipt,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate run_manifest.json, autoware_map_diagnosis.json, and '
            'verify_autoware_map.log, then emit a geometry-free receipt.'
        ),
    )
    parser.add_argument(
        'run_dir',
        metavar='output_dir',
        type=Path,
        help='Completed lidarslam-map output directory.',
    )
    parser.add_argument(
        '--write',
        action='store_true',
        help=(
            'Write first_map_validation_receipt.json and .md into output_dir.'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print JSON instead of Markdown.',
    )
    return parser.parse_args()


def main() -> int:
    """Generate, optionally write, and render one validation receipt."""
    args = _parse_args()
    try:
        receipt = build_receipt(args.run_dir)
        paths = {}
        if args.write:
            receipt, paths = write_receipt(args.run_dir, receipt)
    except ReceiptError as exc:
        print(f'first-map receipt invalid: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(render_markdown(receipt), end='')
    if paths:
        print(f"Receipt JSON: {paths['json']}", file=sys.stderr)
        print(f"Receipt Markdown: {paths['markdown']}", file=sys.stderr)
    return 0 if receipt['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
