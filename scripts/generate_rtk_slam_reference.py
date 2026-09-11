#!/usr/bin/env python3

"""
Build a sparse ground-truth reference from RTK-SLAM dataset checkpoints.

The RTK-SLAM dataset (Univ. Stuttgart, arXiv:2604.07151, CC-BY 4.0) ships its
ground truth as sparse geodetic-total-station checkpoints in a CSV with the
columns ``point_id,easting,northing,height,env,timestamp``. This converts that
CSV into a sparse TUM trajectory (one pose per checkpoint, identity
orientation) that ``write_aligned_trajectory_metrics.py`` scores exactly like
the NTU VIRAL prism and Newer College ICP-to-survey-map references: each
checkpoint is timestamp-matched to the estimate and the matched set is
SE(3)-aligned (Umeyama) before the per-point RMSE. That SE(3)-aligned
checkpoint RMSE is the
v0.5 gate metric (``ape_rmse_gt_m``); the dataset's zero-alignment absolute
RMSE needs a GNSS-anchored estimate, which a LiDAR-inertial config does not
produce, so it is out of scope here.

UTM eastings/northings are large (~1e5..1e6 m); the first checkpoint by time is
subtracted as a local origin so the emitted coordinates stay small and
readable. ``_rigid_align`` already centers before its SVD, so the alignment is
invariant to this offset. The source string contains ``gt`` so
``_infer_reference_kind`` classifies the reference as ground truth.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
from pathlib import Path

REQUIRED_COLUMNS = ('point_id', 'easting', 'northing', 'height', 'env', 'timestamp')
_NUMERIC_REQUIRED = ('easting', 'northing', 'height', 'timestamp')

# Official RTK-SLAM calibration: the public evaluation expects trajectory
# positions at the robot base center, while RKO-LIO's identity-extrinsic
# benchmark preset reports the IMU origin.
IMU_TO_REFERENCE_TRANSLATION_M = (-0.073, -0.023, -0.172)


def parse_checkpoints(text: str) -> list[dict]:
    """Parse RTK-SLAM checkpoint CSV text into a list of dicts (by column name)."""
    reader = csv.DictReader(io.StringIO(text))
    if reader.fieldnames is None:
        raise ValueError('checkpoint CSV has no header row')
    fieldmap = {name.strip(): name for name in reader.fieldnames}
    missing = [column for column in REQUIRED_COLUMNS if column not in fieldmap]
    if missing:
        raise ValueError(f'checkpoint CSV missing columns: {missing}')

    rows: list[dict] = []
    for raw in reader:
        values = {column: raw.get(fieldmap[column]) for column in REQUIRED_COLUMNS}
        if any(values[column] is None or str(values[column]).strip() == ''
               for column in _NUMERIC_REQUIRED):
            continue
        rows.append({
            'point_id': str(values['point_id']).strip(),
            'easting': float(values['easting']),
            'northing': float(values['northing']),
            'height': float(values['height']),
            'env': str(values['env']).strip(),
            'timestamp': float(values['timestamp']),
        })
    if not rows:
        raise ValueError('checkpoint CSV has no usable data rows')
    return rows


def localize(checkpoints: list[dict]) -> tuple[list[dict], dict]:
    """Sort checkpoints by time and subtract the first one as a local origin."""
    ordered = sorted(checkpoints, key=lambda checkpoint: checkpoint['timestamp'])
    origin = {
        'easting': ordered[0]['easting'],
        'northing': ordered[0]['northing'],
        'height': ordered[0]['height'],
    }
    local: list[dict] = []
    for checkpoint in ordered:
        local.append({
            'timestamp': checkpoint['timestamp'],
            'x': checkpoint['easting'] - origin['easting'],
            'y': checkpoint['northing'] - origin['northing'],
            'z': checkpoint['height'] - origin['height'],
            'env': checkpoint['env'],
            'point_id': checkpoint['point_id'],
        })
    return local, origin


def to_tum_lines(local_rows: list[dict]) -> list[str]:
    """Render localized checkpoints as TUM lines with identity orientation."""
    lines: list[str] = []
    for row in local_rows:
        lines.append(
            f"{row['timestamp']:.9f} "
            f"{row['x']:.9f} {row['y']:.9f} {row['z']:.9f} "
            '0.000000000 0.000000000 0.000000000 1.000000000',
        )
    return lines


def source_name(sequence: str) -> str:
    """Return the reference source string; contains 'gt' so kind == ground_truth."""
    slug = sequence.strip().lower().replace(' ', '_').replace('-', '_')
    return f'rtk_slam_{slug}_gt'


def env_breakdown(local_rows: list[dict]) -> dict:
    """Count checkpoints per environment label."""
    counts: dict = {}
    for row in local_rows:
        counts[row['env']] = counts.get(row['env'], 0) + 1
    return counts


def build_reference(csv_path: Path, sequence: str) -> tuple[list[str], dict]:
    """Read a checkpoint CSV and return (tum_lines, metadata-without-paths)."""
    checkpoints = parse_checkpoints(csv_path.read_text(encoding='utf-8'))
    local, origin = localize(checkpoints)
    tum_lines = to_tum_lines(local)
    meta = {
        'source': source_name(sequence),
        'kind': 'ground_truth',
        'dataset': 'rtk_slam',
        'dataset_citation': (
            'RTK-SLAM Dataset, Univ. Stuttgart, arXiv:2604.07151 (CC-BY 4.0)'
        ),
        'sequence': sequence,
        'frame': 'local_enu_from_utm',
        'reference_point_frame': 'base_center',
        'imu_to_reference_translation_m': dict(
            zip('xyz', IMU_TO_REFERENCE_TRANSLATION_M)
        ),
        'reference_translation_source': (
            'RTK-SLAM calib/calib.yaml reference_offsets.base_center'
        ),
        'units': 'meters',
        'max_time_diff_sec': 2.0,
        'local_origin_utm': origin,
        'checkpoint_count': len(local),
        'env_breakdown': env_breakdown(local),
        'metric_note': (
            'Score with write_aligned_trajectory_metrics.py; the SE(3)-aligned '
            'checkpoint RMSE is the v0.5 gate metric (ape_rmse_gt_m). '
            'Use max_time_diff_sec for sparse checkpoint association.'
        ),
    }
    return tum_lines, meta


def main() -> int:
    """CLI entry point: checkpoint CSV -> sparse TUM + reference JSON sidecar."""
    parser = argparse.ArgumentParser(
        description=(
            'Convert an RTK-SLAM checkpoint CSV into a sparse TUM ground-truth '
            'reference scorable by write_aligned_trajectory_metrics.py.'
        ),
    )
    parser.add_argument(
        '--checkpoints',
        required=True,
        help='Path to the RTK-SLAM checkpoint CSV '
             '(point_id,easting,northing,height,env,timestamp)',
    )
    parser.add_argument(
        '--sequence',
        required=True,
        help='Sequence name, e.g. stadtgarten1 (used in the source slug)',
    )
    parser.add_argument(
        '--out',
        default=None,
        help='Output TUM path (default: output/rtk_slam_<sequence>_gt.tum)',
    )
    parser.add_argument(
        '--write-meta',
        default=None,
        help='JSON sidecar path (default: output/rtk_slam_<sequence>_reference.json)',
    )
    args = parser.parse_args()

    csv_path = Path(args.checkpoints).expanduser().resolve()
    slug = args.sequence.strip().lower().replace(' ', '_').replace('-', '_')
    out_path = Path(args.out).expanduser().resolve() if args.out else \
        Path(f'output/rtk_slam_{slug}_gt.tum').resolve()
    meta_path = Path(args.write_meta).expanduser().resolve() if args.write_meta else \
        Path(f'output/rtk_slam_{slug}_reference.json').resolve()

    tum_lines, meta = build_reference(csv_path, args.sequence)
    meta['reference_tum_path'] = str(out_path)
    meta['csv_path'] = str(csv_path)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text('\n'.join(tum_lines) + ('\n' if tum_lines else ''), encoding='utf-8')
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps(meta, indent=2) + '\n', encoding='utf-8')

    print(f'reference_tum: {out_path}')
    print(f'checkpoints: {meta["checkpoint_count"]} env={meta["env_breakdown"]}')
    print(f'reference_meta: {meta_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
