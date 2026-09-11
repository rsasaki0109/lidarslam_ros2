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

"""
Unit and integration tests for the RTK-SLAM reference generator.

The unit tests cover the pure CSV->sparse-TUM logic (column-name mapping,
local-origin subtraction, identity orientation, ground-truth source naming).
The integration test proves the end-to-end contract without the 182 GB
dataset: it runs the generator CLI on a synthetic checkpoint CSV, then scores
the result with write_aligned_trajectory_metrics.py and asserts the reference
is classified as ground truth and yields an SE(3)-aligned checkpoint RMSE.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PATH = REPO_ROOT / 'scripts' / 'generate_rtk_slam_reference.py'
SCORER_PATH = REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py'


def _load_generator():
    spec = importlib.util.spec_from_file_location(
        'generate_rtk_slam_reference',
        GENERATOR_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _synthetic_csv(count: int, shuffle_columns: bool = False) -> str:
    """Build a checkpoint CSV with large UTM coordinates, like the real dataset."""
    if shuffle_columns:
        header = 'timestamp,point_id,env,height,northing,easting'
    else:
        header = 'point_id,easting,northing,height,env,timestamp'
    lines = [header]
    for i in range(count):
        easting = 513000.0 + i
        northing = 5403000.0 + 2.0 * i
        height = 250.0 + 0.1 * i
        env = 'park' if i % 2 == 0 else 'hall'
        timestamp = float(i)
        if shuffle_columns:
            lines.append(
                f'{timestamp:.6f},cp{i},{env},{height:.3f},{northing:.3f},{easting:.3f}',
            )
        else:
            lines.append(
                f'cp{i},{easting:.3f},{northing:.3f},{height:.3f},{env},{timestamp:.6f}',
            )
    return '\n'.join(lines) + '\n'


def _parse_tum_positions(text: str) -> list[tuple[float, float, float, float]]:
    poses = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        t, x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        poses.append((t, x, y, z))
    return poses


def test_parse_checkpoints_maps_columns_by_name():
    """Column order must not matter; columns are mapped by header name."""
    module = _load_generator()
    rows = module.parse_checkpoints(_synthetic_csv(3, shuffle_columns=True))
    assert len(rows) == 3
    assert rows[0]['point_id'] == 'cp0'
    assert rows[0]['easting'] == 513000.0
    assert rows[0]['env'] == 'park'


def test_parse_checkpoints_rejects_missing_columns():
    """A CSV missing a required column should raise a clear error."""
    module = _load_generator()
    bad = 'point_id,easting,northing,height,timestamp\ncp0,1,2,3,0.0\n'
    raised = False
    try:
        module.parse_checkpoints(bad)
    except ValueError as exc:
        raised = True
        assert 'env' in str(exc)
    assert raised


def test_parse_checkpoints_skips_blank_rows():
    """Blank or numerically empty rows are skipped, not fatal."""
    module = _load_generator()
    text = (
        'point_id,easting,northing,height,env,timestamp\n'
        'cp0,513000,5403000,250,park,0.0\n'
        '\n'
        'cp1,513001,5403002,250.1,hall,1.0\n'
    )
    rows = module.parse_checkpoints(text)
    assert len(rows) == 2


def test_localize_sorts_by_time_and_subtracts_origin():
    """The earliest checkpoint becomes the local origin; UTM shrinks to small."""
    module = _load_generator()
    checkpoints = module.parse_checkpoints(_synthetic_csv(4))
    # Reverse to confirm localize sorts by timestamp rather than input order.
    local, origin = module.localize(list(reversed(checkpoints)))
    assert origin['easting'] == 513000.0
    assert local[0]['timestamp'] == 0.0
    assert local[0]['x'] == 0.0 and local[0]['y'] == 0.0 and local[0]['z'] == 0.0
    # Coordinates are now metres-scale, not 1e5-scale UTM.
    assert abs(local[-1]['x']) < 100.0
    assert abs(local[-1]['y']) < 100.0


def test_to_tum_lines_have_identity_orientation_and_eight_columns():
    """Each TUM line has 8 columns with an identity (0 0 0 1) quaternion."""
    module = _load_generator()
    local, _ = module.localize(module.parse_checkpoints(_synthetic_csv(3)))
    lines = module.to_tum_lines(local)
    assert len(lines) == 3
    for line in lines:
        parts = line.split()
        assert len(parts) == 8
        assert parts[4:] == ['0.000000000', '0.000000000', '0.000000000', '1.000000000']


def test_source_name_contains_gt_token():
    """The source slug must contain 'gt' so the scorer infers ground_truth."""
    module = _load_generator()
    assert module.source_name('Stadtgarten 1') == 'rtk_slam_stadtgarten_1_gt'
    assert 'gt' in module.source_name('hall-2')


def test_env_breakdown_counts_per_label():
    """env_breakdown tallies checkpoints per environment label."""
    module = _load_generator()
    local, _ = module.localize(module.parse_checkpoints(_synthetic_csv(4)))
    counts = module.env_breakdown(local)
    assert counts == {'park': 2, 'hall': 2}


def test_generated_reference_scores_as_ground_truth(tmp_path):
    """Generator CLI output scores as an SE(3)-aligned ground-truth checkpoint RMSE."""
    csv_path = tmp_path / 'checkpoints.csv'
    csv_path.write_text(_synthetic_csv(12), encoding='utf-8')

    ref_tum = tmp_path / 'rtk_slam_test_gt.tum'
    meta_path = tmp_path / 'rtk_slam_test_reference.json'
    gen = subprocess.run(
        [
            'python3', str(GENERATOR_PATH),
            '--checkpoints', str(csv_path),
            '--sequence', 'test',
            '--out', str(ref_tum),
            '--write-meta', str(meta_path),
        ],
        capture_output=True, text=True, check=False, cwd=REPO_ROOT,
    )
    assert gen.returncode == 0, gen.stderr
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['source'] == 'rtk_slam_test_gt'
    assert meta['kind'] == 'ground_truth'
    assert meta['checkpoint_count'] == 12
    assert meta['max_time_diff_sec'] == 2.0
    assert meta['reference_point_frame'] == 'base_center'
    assert meta['imu_to_reference_translation_m'] == {
        'x': -0.073, 'y': -0.023, 'z': -0.172}
    assert 'calib/calib.yaml' in meta['reference_translation_source']

    # Build an estimate that differs from GT by a pure rigid translation, which
    # SE(3) alignment removes -> RMSE ~ 0 over the sparse checkpoint set.
    ref_poses = _parse_tum_positions(ref_tum.read_text(encoding='utf-8'))
    assert len(ref_poses) == 12
    est_tum = tmp_path / 'est.tum'
    est_lines = []
    for t, x, y, z in ref_poses:
        est_lines.append(
            f'{t:.9f} {x + 1.5:.9f} {y - 2.0:.9f} {z + 0.5:.9f} 0 0 0 1',
        )
    est_tum.write_text('\n'.join(est_lines) + '\n', encoding='utf-8')

    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'data.db3').write_bytes(b'synthetic bag payload')
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 1000000000\n',
        encoding='utf-8',
    )

    out_dir = tmp_path / 'output' / 'bench'
    # Omit --reference-kind so the kind is inferred from the source slug.
    score = subprocess.run(
        [
            'python3', str(SCORER_PATH),
            '--out-dir', str(out_dir),
            '--bag', str(bag_dir),
            '--reference-tum', str(ref_tum),
            '--corrected-tum', str(est_tum),
            '--reference-source', 'rtk_slam_test_gt',
            '--runtime-artifact', 'true=/bin/true',
        ],
        capture_output=True, text=True, check=False, cwd=REPO_ROOT,
    )
    assert score.returncode == 0, score.stderr
    metrics = json.loads((out_dir / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['reference']['kind'] == 'ground_truth'
    assert metrics['reference']['source'] == 'rtk_slam_test_gt'
    assert metrics['evo']['ape']['alignment'] == 'se3_umeyama'
    assert metrics['evo']['ape']['pairs'] == 12
    assert metrics['evo']['ape']['rmse'] < 1e-6


def test_match_tolerance_recovers_offset_sparse_checkpoints(tmp_path):
    """A wide --match-tolerance pairs checkpoints to a downsampled estimate."""
    csv_path = tmp_path / 'checkpoints.csv'
    csv_path.write_text(_synthetic_csv(12), encoding='utf-8')
    ref_tum = tmp_path / 'gt.tum'
    subprocess.run(
        [
            'python3', str(GENERATOR_PATH),
            '--checkpoints', str(csv_path),
            '--sequence', 'test',
            '--out', str(ref_tum),
            '--write-meta', str(tmp_path / 'ref.json'),
        ],
        capture_output=True, text=True, check=True, cwd=REPO_ROOT,
    )

    # Estimate timestamps sit 0.5 s off every checkpoint: beyond the default
    # 0.15 s cascade, inside a 2.0 s tolerance. Positions are unchanged, so the
    # wide-tolerance match recovers all 12 checkpoints.
    ref_poses = _parse_tum_positions(ref_tum.read_text(encoding='utf-8'))
    est_tum = tmp_path / 'est.tum'
    est_tum.write_text(
        '\n'.join(
            f'{t + 0.5:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1'
            for t, x, y, z in ref_poses
        ) + '\n',
        encoding='utf-8',
    )

    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'data.db3').write_bytes(b'synthetic bag payload')
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 1000000000\n',
        encoding='utf-8',
    )

    def _score(out_dir, extra):
        return subprocess.run(
            [
                'python3', str(SCORER_PATH),
                '--out-dir', str(out_dir),
                '--bag', str(bag_dir),
                '--reference-tum', str(ref_tum),
                '--corrected-tum', str(est_tum),
                '--reference-source', 'rtk_slam_test_gt',
                '--runtime-artifact', 'true=/bin/true',
            ] + extra,
            capture_output=True, text=True, check=False, cwd=REPO_ROOT,
        )

    # The default cascade cannot pair anything 0.5 s away, so it fails.
    default = _score(tmp_path / 'out_default', [])
    assert default.returncode != 0

    wide = _score(tmp_path / 'out_wide', ['--match-tolerance', '2.0'])
    assert wide.returncode == 0, wide.stderr
    metrics = json.loads(
        (tmp_path / 'out_wide' / 'metrics.json').read_text(encoding='utf-8'),
    )
    assert metrics['evo']['ape']['pairs'] == 12
