#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Convert official Newer College LiDAR ground-truth CSV to TUM + metadata."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys


SOURCE = 'official_newer_college_math_hard_icp_map_gt_csv'


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_numeric_row(row: list[str], line_number: int) -> tuple[int, list[float]]:
    if len(row) < 9:
        raise ValueError(
            f'line {line_number}: expected at least 9 CSV columns, got {len(row)}'
        )
    try:
        seconds = int(row[0].strip())
        nanoseconds = int(row[1].strip())
        values = [float(value.strip()) for value in row[2:9]]
    except ValueError as exc:
        raise ValueError(f'line {line_number}: invalid numeric value') from exc
    if not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError(
            f'line {line_number}: nanoseconds must be in [0, 1000000000)'
        )
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'line {line_number}: pose values must be finite')
    quaternion = values[3:7]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not 0.95 <= norm <= 1.05:
        raise ValueError(
            f'line {line_number}: quaternion norm {norm:.6g} is outside [0.95, 1.05]'
        )
    values[3:7] = [value / norm for value in quaternion]
    return seconds * 1_000_000_000 + nanoseconds, values


def convert_csv(source: Path) -> tuple[list[str], dict[str, int]]:
    """Parse the official positional CSV and return deterministic TUM lines."""
    lines: list[str] = []
    stamps: list[int] = []
    saw_data = False
    with source.open(newline='', encoding='utf-8-sig') as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if not row or all(not value.strip() for value in row):
                continue
            if row[0].lstrip().startswith('#'):
                continue
            try:
                stamp_ns, values = _parse_numeric_row(row, line_number)
            except ValueError:
                header_tokens = {
                    value.strip().lower().replace(' ', '_') for value in row[:2]
                }
                if not saw_data and (
                    any('sec' in token for token in header_tokens)
                    and any('nano' in token or 'nsec' in token
                            for token in header_tokens)
                ):
                    # The official download may include one descriptive header row.
                    continue
                raise
            saw_data = True
            if stamps and stamp_ns <= stamps[-1]:
                raise ValueError(
                    f'line {line_number}: timestamps must be strictly increasing'
                )
            stamps.append(stamp_ns)
            seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
            lines.append(
                f'{seconds}.{nanoseconds:09d} '
                + ' '.join(f'{value:.12g}' for value in values)
            )
    if len(lines) < 3:
        raise ValueError('reference contains fewer than three poses')
    return lines, {
        'pose_count': len(lines),
        'first_stamp_ns': stamps[0],
        'last_stamp_ns': stamps[-1],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--body-to-reference-x', required=True, type=float)
    parser.add_argument('--body-to-reference-y', required=True, type=float)
    parser.add_argument('--body-to-reference-z', required=True, type=float)
    args = parser.parse_args()

    source = args.csv.expanduser().resolve()
    output = args.output.expanduser().resolve()
    metadata_path = args.metadata.expanduser().resolve()
    calibration = args.calibration.expanduser().resolve()
    if output.exists() or metadata_path.exists():
        raise ValueError('refusing to overwrite reference artifacts')
    if not source.is_file():
        raise ValueError(f'ground-truth CSV not found: {source}')
    if not calibration.is_file():
        raise ValueError(f'collection calibration not found: {calibration}')
    offset = (
        args.body_to_reference_x,
        args.body_to_reference_y,
        args.body_to_reference_z,
    )
    if not all(math.isfinite(value) for value in offset):
        raise ValueError('body-to-reference translation must be finite')

    tum_lines, summary = convert_csv(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(tum_lines) + '\n', encoding='utf-8')
    metadata = {
        'schema_version': 1,
        'dataset': 'newer_college_multi_camera',
        'sequence': 'maths_hard',
        'kind': 'ground_truth',
        'source': SOURCE,
        'dataset_citation': (
            'Multi-Camera LiDAR Inertial Extension to the Newer College '
            'Dataset (CC BY-NC-SA 4.0)'
        ),
        'reference_point_frame': 'Base',
        'body_to_reference_translation_m': dict(zip('xyz', offset)),
        'reference_translation_source': str(calibration),
        'reference_translation_source_sha256': _sha256(calibration),
        'source_csv': str(source),
        'source_csv_sha256': _sha256(source),
        'reference_tum': str(output),
        'reference_tum_sha256': _sha256(output),
        'max_time_diff_sec': 0.05,
        **summary,
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(f'reference_tum: {output}')
    print(f'reference_metadata: {metadata_path}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, csv.Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2) from None
