#!/usr/bin/env python3
"""Convert official NTU-VIRAL Leica CSV ground truth to frozen TUM data."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import yaml


# Official evaluation tutorial, T_B_prism translation (metres).
BODY_TO_PRISM = (-0.293656, -0.012288, -0.273095)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def lidar_to_body_translation(rko_param: Path) -> tuple[float, float, float]:
    document = yaml.safe_load(rko_param.read_text(encoding='utf-8')) or {}
    values = document.get('extrinsic_lidar2base_quat_xyzw_xyz')
    if not isinstance(values, list) or len(values) != 7:
        raise ValueError('invalid extrinsic_lidar2base_quat_xyzw_xyz')
    qx, qy, qz, qw, tx, ty, tz = (float(value) for value in values)
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm <= 0.0:
        raise ValueError('zero-norm lidar-to-base quaternion')
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    # Invert base_T_lidar: the offset from the LiDAR origin to the base
    # origin, expressed in the LiDAR frame, is -R_base_lidar^T t_base_lidar.
    return (
        -((1 - 2 * (qy * qy + qz * qz)) * tx +
          2 * (qx * qy + qz * qw) * ty +
          2 * (qx * qz - qy * qw) * tz),
        -(2 * (qx * qy - qz * qw) * tx +
          (1 - 2 * (qx * qx + qz * qz)) * ty +
          2 * (qy * qz + qx * qw) * tz),
        -(2 * (qx * qz + qy * qw) * tx +
          2 * (qy * qz - qx * qw) * ty +
          (1 - 2 * (qx * qx + qy * qy)) * tz),
    )


def convert(source: Path, output: Path) -> dict[str, object]:
    rows: list[str] = []
    seen: set[int] = set()
    with source.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            stamp_ns = int(row['field.header.stamp'])
            if stamp_ns in seen:
                continue
            seen.add(stamp_ns)
            values = [float(row[key]) for key in (
                'field.pose.position.x', 'field.pose.position.y',
                'field.pose.position.z', 'field.pose.orientation.x',
                'field.pose.orientation.y', 'field.pose.orientation.z',
                'field.pose.orientation.w')]
            rows.append(f'{stamp_ns * 1e-9:.9f} ' + ' '.join(
                f'{value:.12g}' for value in values))
    if len(rows) < 3:
        raise ValueError('reference contains fewer than three unique poses')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    return {
        'pose_count': len(rows),
        'first_stamp_ns': min(seen),
        'last_stamp_ns': max(seen),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--rko-param', required=True, type=Path)
    parser.add_argument('--ground-truth-revision', required=True)
    args = parser.parse_args()
    if args.output.exists() or args.metadata.exists():
        raise ValueError('refusing to overwrite reference artifacts')
    summary = convert(args.csv.resolve(), args.output.resolve())
    lidar_to_body = lidar_to_body_translation(args.rko_param.resolve())
    lidar_to_prism = tuple(a + b for a, b in zip(
        BODY_TO_PRISM, lidar_to_body))
    metadata = {
        'schema_version': 1,
        'sequence': args.sequence,
        'source': 'official_ntu_viral_leica_prism_ground_truth_csv',
        'source_csv': str(args.csv.resolve()),
        'source_csv_sha256': sha256_file(args.csv.resolve()),
        'ground_truth_repository_revision': args.ground_truth_revision,
        'reference_tum': str(args.output.resolve()),
        'reference_tum_sha256': sha256_file(args.output.resolve()),
        **summary,
        'body_to_prism_translation_m': dict(zip('xyz', BODY_TO_PRISM)),
        'imu_to_prism_translation_m': dict(zip('xyz', BODY_TO_PRISM)),
        'lidar_to_body_translation_m': dict(zip('xyz', lidar_to_body)),
        'body_T_lidar_translation_m': {
            'x': -lidar_to_body[0],
            'y': -lidar_to_body[1],
            'z': -lidar_to_body[2],
        },
        'lidar_to_prism_translation_m': dict(zip('xyz', lidar_to_prism)),
        'lever_arm_source': (
            'NTU-VIRAL official evaluation tutorial T_B_prism'),
    }
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(metadata, indent=2, sort_keys=True) + '\n')
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
