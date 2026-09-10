#!/usr/bin/env python3
"""Capture the non-secret hardware fingerprint used by competition runners."""

from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
import sys

from lidarslam_benchmark_tools.run_fast_livo2_benchmark import benchmark_machine_fingerprint


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    document = {
        'schema_version': 1,
        'captured_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        **benchmark_machine_fingerprint(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(json.dumps(document, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
