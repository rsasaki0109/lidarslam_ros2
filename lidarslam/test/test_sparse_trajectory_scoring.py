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
Regression tests for the sparse (submap-rate) trajectory scoring mode.

Background (docs/research/project-plan-archive.md Sec. 2.4, HILTI 2022 exp01): pose graph optimization was
a verified no-op (corrected == raw odometry, 0.000 m diff) yet the sparse
"corrected" submap trajectory (median 1.2 s / max 19.9 s pose spacing) scored
an APE of 0.891 m against mm-level total-station control points, versus
0.066 m for the dense raw trajectory. The cause was a scoring artifact, not
a SLAM regression: `ape_from_tum.py --interpolate` linearly blends the two
estimated poses bracketing each (sparse, static-dwell) reference timestamp;
when those two poses are many seconds apart the straight-line assumption
between them does not hold and the interpolated position can be many metres
off. The fix is `--sparse-match`: pair each reference sample with its
temporally nearest estimate sample (never fabricate a blended position),
report it (and any rejected reference points) via diagnostics instead of
silently dropping them.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'ape_from_tum.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('ape_from_tum', SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


APE = _load_module()


def _write_tum(path: Path, rows: list[tuple[float, float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for t, x, y, z in rows:
        lines.append(f'{t:.4f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _run_ape(
    ref_tum: Path, est_tum: Path, out: Path, *extra_args: str,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--ref', str(ref_tum),
            '--est', str(est_tum),
            '--out', str(out),
            *extra_args,
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )


def _parse_report(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        values[key.strip()] = value.strip()
    return values


# ---------------------------------------------------------------------------
# Synthetic sparse-submap scenario shared by the CLI-level tests.
#
# A submap-rate estimated trajectory travels along a straight line at 3 m/s
# from t=0 to t=12, then -- exactly like a HILTI control-point dwell where no
# submap is created while the platform is stationary -- there is no submap
# between t=12 and t=32. The platform actually paused right where the t=12
# submap was taken (the true, GT-surveyed position at t=13 is only 0.05 m /
# 0.02 m off the t=12 pose), then resumed and made a long, differently
# directed traverse before the next submap at t=32. From t=32 onward it again
# moves in a straight line, matched 1:1 by both trajectories.
# ---------------------------------------------------------------------------

_EST_ROWS = [
    (0.0, 0.0, 0.0, 0.0),
    (4.0, 12.0, 0.0, 0.0),
    (8.0, 24.0, 0.0, 0.0),
    (12.0, 36.0, 0.0, 0.0),
    (32.0, 136.0, 40.0, 0.0),
    (36.0, 148.0, 40.0, 0.0),
    (40.0, 160.0, 40.0, 0.0),
    (44.0, 172.0, 40.0, 0.0),
    (48.0, 184.0, 40.0, 0.0),
]

_REF_ROWS = [
    (0.0, 0.0, 0.0, 0.0),
    (4.0, 12.0, 0.0, 0.0),
    (8.0, 24.0, 0.0, 0.0),
    (12.0, 36.0, 0.0, 0.0),
    (13.0, 36.05, 0.02, 0.0),  # the sparse "control point" that falls in the gap
    (36.0, 148.0, 40.0, 0.0),
    (40.0, 160.0, 40.0, 0.0),
    (44.0, 172.0, 40.0, 0.0),
    (48.0, 184.0, 40.0, 0.0),
]


def test_interpolate_produces_artifact_error_and_sparse_match_scores_correctly(tmp_path):
    """Interpolation fabricates metres of error in the gap; sparse-match scores correctly."""
    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    _write_tum(ref_tum, _REF_ROWS)
    _write_tum(est_tum, _EST_ROWS)

    interp_out = tmp_path / 'ape_interpolate.txt'
    result_interp = _run_ape(
        ref_tum, est_tum, interp_out,
        '--interpolate', '--max-time-diff', '3.0',
    )
    assert result_interp.returncode == 0, result_interp.stderr
    interp_report = _parse_report(interp_out)
    assert interp_report['mode'] == 'interpolate'
    assert interp_report['pairs'] == str(len(_REF_ROWS))
    assert interp_report['rejected_ref_points'] == '0'
    # The bracket spanning the gap (t=12 -> t=32) is 20 s wide.
    assert float(interp_report['max_time_gap']) == pytest.approx(20.0)
    interp_max_error = float(interp_report['max'])

    sparse_out = tmp_path / 'ape_sparse_match.txt'
    result_sparse = _run_ape(ref_tum, est_tum, sparse_out, '--sparse-match')
    assert result_sparse.returncode == 0, result_sparse.stderr
    sparse_report = _parse_report(sparse_out)
    assert sparse_report['mode'] == 'sparse_match'
    assert sparse_report['pairs'] == str(len(_REF_ROWS))
    assert sparse_report['rejected_ref_points'] == '0'
    # The nearest submap to the t=13 control point is t=12, 1 s away.
    assert float(sparse_report['max_time_gap']) == pytest.approx(1.0)
    sparse_max_error = float(sparse_report['max'])

    # The artifact: naive interpolation across the 20 s gap is off by
    # several metres, while nearest-neighbour matching against the real,
    # temporally-close submap pose is off by a few centimetres.
    assert interp_max_error > 3.0
    assert sparse_max_error < 0.5
    assert sparse_max_error < interp_max_error


def test_sparse_match_association_matches_nearest_pose_not_interpolated(tmp_path):
    """Sparse-match association returns the estimate's raw position, not a fabricated blend."""
    ref_xyz, est_xyz, diagnostics = APE.associate(
        _REF_ROWS_AS_TUPLES(), _EST_ROWS_AS_TUPLES(), 10.0,
    )
    # ref index 4 is the t=13 control point (see _REF_ROWS).
    assert est_xyz[4] == (36.0, 0.0, 0.0)
    assert diagnostics['pairs'] == len(_REF_ROWS)
    assert diagnostics['rejected_ref_points'] == 0
    assert diagnostics['max_time_gap'] == pytest.approx(1.0)

    ref_xyz_i, est_xyz_i, diagnostics_i = APE.interpolate_association(
        _REF_ROWS_AS_TUPLES(), _EST_ROWS_AS_TUPLES(), 3.0,
    )
    ex, ey, ez = est_xyz_i[4]
    assert ex == pytest.approx(41.0)
    assert ey == pytest.approx(2.0)
    assert diagnostics_i['max_time_gap'] == pytest.approx(20.0)


def _REF_ROWS_AS_TUPLES():
    return [(t, (x, y, z)) for t, x, y, z in _REF_ROWS]


def _EST_ROWS_AS_TUPLES():
    return [(t, (x, y, z)) for t, x, y, z in _EST_ROWS]


def test_sparse_match_rejects_out_of_tolerance_reference_points_with_diagnostics(tmp_path):
    """Out-of-tolerance reference points must be rejected visibly, never silently dropped."""
    ref_rows = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0),
        (2.0, 2.0, 0.0, 0.0),
        (3.0, 3.0, 0.0, 0.0),
        (100.0, 999.0, 999.0, 999.0),  # far outside the estimate's time range
    ]
    est_rows = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0),
        (2.0, 2.0, 0.0, 0.0),
        (3.0, 3.0, 0.0, 0.0),
    ]
    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    _write_tum(ref_tum, ref_rows)
    _write_tum(est_tum, est_rows)

    out = tmp_path / 'ape_sparse_match.txt'
    result = _run_ape(ref_tum, est_tum, out, '--sparse-match', '--max-time-diff', '5.0')
    assert result.returncode == 0, result.stderr

    report = _parse_report(out)
    assert report['mode'] == 'sparse_match'
    assert report['pairs'] == '4'
    assert report['total_ref_points'] == '5'
    assert report['rejected_ref_points'] == '1'
    # The rejection must not be silent: a diagnostic warning goes to stderr.
    assert 'rejected 1/5 reference point' in result.stderr
    assert 'sparse_match' in result.stderr


def test_interpolate_and_sparse_match_are_mutually_exclusive(tmp_path):
    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    rows = [(float(i), float(i), 0.0, 0.0) for i in range(4)]
    _write_tum(ref_tum, rows)
    _write_tum(est_tum, rows)

    out = tmp_path / 'ape.txt'
    result = _run_ape(ref_tum, est_tum, out, '--interpolate', '--sparse-match')
    assert result.returncode != 0
    assert 'mutually exclusive' in result.stderr


def test_default_mode_is_unchanged(tmp_path):
    """Default nearest-neighbour behaviour and existing report keys must be preserved."""
    rows = [(float(i), float(i) * 2.0, 0.0, 0.0) for i in range(6)]
    ref_tum = tmp_path / 'ref.tum'
    est_tum = tmp_path / 'est.tum'
    _write_tum(ref_tum, rows)
    _write_tum(est_tum, rows)

    out = tmp_path / 'ape.txt'
    result = _run_ape(ref_tum, est_tum, out)
    assert result.returncode == 0, result.stderr
    report = _parse_report(out)
    assert report['mode'] == 'nearest_neighbor'
    assert float(report['rmse']) == pytest.approx(0.0, abs=1e-9)
    assert report['pairs'] == str(len(rows))
    assert report['rejected_ref_points'] == '0'
