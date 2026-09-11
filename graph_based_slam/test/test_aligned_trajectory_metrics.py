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

"""Tests for the aligned trajectory metrics writer."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py'


def _write_tum(path: Path, rows: list[tuple[float, float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for t, x, y, z in rows:
        lines.append(f'{t:.3f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def test_write_aligned_trajectory_metrics(tmp_path):
    """The writer should emit cross-validation metrics from TUM trajectories."""
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'data.db3').write_bytes(b'synthetic bag payload')
    (bag_dir / 'metadata.yaml').write_text(
        '\n'.join(
            [
                'rosbag2_bagfile_information:',
                '  duration:',
                '    nanoseconds: 1000000000',
            ],
        )
        + '\n',
        encoding='utf-8',
    )

    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    raw_tum = tmp_path / 'raw.tum'
    rows = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0),
        (2.0, 2.0, 1.0, 0.0),
        (3.0, 3.0, 1.0, 0.0),
        (4.0, 4.0, 2.0, 0.0),
        (5.0, 5.0, 3.0, 0.0),
        (6.0, 6.0, 3.0, 0.0),
        (7.0, 7.0, 4.0, 0.0),
        (8.0, 8.0, 4.0, 0.0),
        (9.0, 9.0, 5.0, 0.0),
    ]
    ref_rows = rows
    est_rows = [(t, x + 1.5, y - 2.0, z + 0.5) for t, x, y, z in rows]
    raw_rows = [(t, x + 3.0, y - 1.0, z + 0.5) for t, x, y, z in rows]
    _write_tum(ref_tum, ref_rows)
    _write_tum(est_tum, est_rows)
    _write_tum(raw_tum, raw_rows)

    graph_log = tmp_path / 'graph_slam.log'
    graph_log.write_text(
        '\n'.join(
            [
                '[INFO] [x] [graph_based_slam]: Odom input: 50 submaps, distance: 82.6m',
                '[INFO] [x] [graph_based_slam]: Odom input: 600 submaps, distance: 1022.3m',
                'id_loop_point 1:0 id_loop_point 2:606',
            ],
        )
        + '\n',
        encoding='utf-8',
    )

    out_dir = tmp_path / 'output' / 'bench_mid360'
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--out-dir',
            str(out_dir),
            '--bag',
            str(bag_dir),
            '--reference-tum',
            str(ref_tum),
            '--corrected-tum',
            str(est_tum),
            '--raw-tum',
            str(raw_tum),
            '--graph-log',
            str(graph_log),
            '--reference-source',
            'glim_mid360_reference',
            '--reference-kind',
            'cross_validation',
            '--reference-label',
            'GLIM',
            '--points-topic',
            '/livox/lidar',
            '--points-frame',
            'livox_frame',
            '--robot-frame',
            'livox_frame',
            '--wall-sec',
            '0.5',
            '--runtime-artifact',
            'true=/bin/true',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    metrics = json.loads((out_dir / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['reference']['kind'] == 'cross_validation'
    assert metrics['reference']['source'] == 'glim_mid360_reference'
    assert metrics['points_topic'] == '/livox/lidar'
    assert metrics['frames']['points_frame_id'] == 'livox_frame'
    assert metrics['graph_based_slam']['max_loop_search_distance_m'] == 1022.3
    assert metrics['graph_based_slam']['loop_count'] == 1
    assert metrics['graph_based_slam']['loop_count_attempted'] == 1
    assert metrics['graph_based_slam']['last_loop_edge'] == {
        'from_index': 0,
        'to_index': 606,
    }
    assert metrics['graph_based_slam']['last_loop_edge_attempted'] == {
        'from_index': 0,
        'to_index': 606,
    }
    assert metrics['cross_validation']['reference_label'] == 'GLIM'
    assert metrics['evo']['ape']['alignment'] == 'se3_umeyama'
    assert metrics['evo']['ape']['pairs'] == 10
    assert metrics['evo']['ape']['rmse'] < 1e-6


def test_write_aligned_trajectory_metrics_tracks_rejected_loop_candidates(tmp_path):
    """Rejected duplicate loops should not count as accepted loop closures."""
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'data.db3').write_bytes(b'synthetic bag payload')
    (bag_dir / 'metadata.yaml').write_text(
        '\n'.join(
            [
                'rosbag2_bagfile_information:',
                '  duration:',
                '    nanoseconds: 1000000000',
            ],
        )
        + '\n',
        encoding='utf-8',
    )

    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    rows = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0),
        (2.0, 2.0, 0.0, 0.0),
        (3.0, 3.0, 0.0, 0.0),
        (4.0, 4.0, 0.0, 0.0),
        (5.0, 5.0, 0.0, 0.0),
        (6.0, 6.0, 0.0, 0.0),
        (7.0, 7.0, 0.0, 0.0),
        (8.0, 8.0, 0.0, 0.0),
        (9.0, 9.0, 0.0, 0.0),
    ]
    _write_tum(ref_tum, rows)
    _write_tum(est_tum, rows)

    graph_log = tmp_path / 'graph_slam.log'
    graph_log.write_text(
        '\n'.join(
            [
                'PoseAdjustment distance:13.3, score:5.1',
                'id_loop_point 1:8 id_loop_point 2:585',
                'loop_candidate_source:distance',
                'PoseAdjustment distance:3.3, score:11.2',
                'id_loop_point 1:2 id_loop_point 2:588',
                'loop edge skipped as redundant or lower quality',
            ],
        )
        + '\n',
        encoding='utf-8',
    )

    out_dir = tmp_path / 'output' / 'bench_mid360'
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--out-dir',
            str(out_dir),
            '--bag',
            str(bag_dir),
            '--reference-tum',
            str(ref_tum),
            '--corrected-tum',
            str(est_tum),
            '--graph-log',
            str(graph_log),
            '--reference-source',
            'glim_mid360_reference',
            '--reference-kind',
            'cross_validation',
            '--reference-label',
            'GLIM',
            '--runtime-artifact',
            'true=/bin/true',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    metrics = json.loads((out_dir / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['graph_based_slam']['loop_count'] == 1
    assert metrics['graph_based_slam']['loop_count_attempted'] == 2
    assert metrics['graph_based_slam']['last_loop_edge'] == {
        'from_index': 8,
        'to_index': 585,
    }
    assert metrics['graph_based_slam']['last_loop_edge_attempted'] == {
        'from_index': 2,
        'to_index': 588,
    }
