"""Contract tests for the report-only visual weak-axis join."""

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'visual_weak_join', ROOT / 'scripts/join_visual_weak_axis_diagnostic.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _weak_row(timestamp: float, ratio: float = 0.1) -> dict[str, str]:
    return {
        't': str(timestamp), 'evalue0': str(ratio), 'evalue1': '1.0',
        'weak_eigen_x': '0.0', 'weak_eigen_y': '1.0',
        'weak_eigen_z': '0.0', 'weak_horizontal_norm': '1.0',
        'position_x': '0.0', 'position_y': '0.0', 'position_z': '0.0',
        'velocity_x': '0.0', 'velocity_y': '1.0', 'velocity_z': '0.0',
    }


def _pair(timestamp: float, speed: float = 4.0) -> dict:
    return {
        'valid': True, 'lidar_timestamp_sec': timestamp,
        'direction_base': [0.0, 1.0, 0.0], 'speed_mps': speed,
    }


def test_identity_quaternion_and_streak_are_deterministic():
    np.testing.assert_allclose(
        MODULE.quaternion_xyzw_rotation((0.0, 0.0, 0.0, 1.0)), np.eye(3))
    assert MODULE.max_true_streak(
        np.asarray([False, True, True, True, True, True, False])) == (5, 6)


def test_join_accepts_sustained_weak_projection_and_suppresses_strong_axis():
    weak_rows = [_weak_row(float(index)) for index in range(6)]
    weak_rows[0] = _weak_row(0.0, ratio=1.0)
    weak_rows[2]['weak_eigen_y'] = '-1.0'
    weak_times = np.arange(6, dtype=np.float64)
    odometry_times = weak_times.copy()
    rotations = [np.eye(3) for _ in odometry_times]
    report = MODULE.project_visual_pairs(
        [_pair(float(index)) for index in range(6)], weak_rows, weak_times,
        odometry_times, rotations, np.asarray([0.0, 0.0, 1.0]), 0.08,
        0.2, 0.9, 0.1, 3.0, 'timestamp', 0)
    assert report['decision'] == 'GO_WEAK_AXIS_PROJECTION'
    assert report['counts']['well_conditioned_pairs_suppressed'] == 1
    assert report['quality']['max_strong_streak_scans'] == 5
    assert len(report['observations']) == 5
    assert report['observations'][0]['velocity_mps'] == 4.0
    assert report['observations'][0]['pair_indices'] == [1]
    assert report['observations'][1]['velocity_mps'] == 4.0
