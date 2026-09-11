# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the radar-less tunnel A/B tools."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess

import numpy as np
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
EVALUATOR_PATH = REPO_ROOT / 'scripts/evaluate_degeneracy_trajectory.py'
RUNNER_PATH = REPO_ROOT / 'scripts/run_radarless_tunnel_ab.sh'
PRESET_PATH = (
    REPO_ROOT / 'lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml'
)


def _load_evaluator():
    spec = importlib.util.spec_from_file_location(
        'evaluate_degeneracy_trajectory',
        EVALUATOR_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_evaluator()


def _write_tum(path: Path, timestamps: np.ndarray, xyz: np.ndarray) -> None:
    rows = [
        f'{stamp:.6f} {point[0]:.9f} {point[1]:.9f} {point[2]:.9f} '
        '0 0 0 1'
        for stamp, point in zip(timestamps, xyz)
    ]
    path.write_text('\n'.join(rows) + '\n', encoding='utf-8')


def test_reference_metrics_recover_time_aligned_scale_and_rigid_delta(
    tmp_path: Path,
) -> None:
    """Reach ratio should expose scale while rigid alignment removes frame choice."""
    timestamps = np.linspace(0.0, 20.0, 41)
    reference_xyz = np.column_stack(
        (timestamps, 0.05 * timestamps**2, np.sin(timestamps / 4.0)),
    )
    rotation = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    )
    candidate_xyz = (rotation.T @ (reference_xyz - [3.0, -2.0, 0.5]).T).T

    reference_path = tmp_path / 'reference.tum'
    candidate_path = tmp_path / 'candidate.tum'
    _write_tum(reference_path, timestamps, reference_xyz)
    _write_tum(candidate_path, timestamps, candidate_xyz)

    result = EVALUATOR.evaluate(
        candidate_path,
        expected_endpoint_distance=None,
        reference_path=reference_path,
        min_reference_reach_m=2.0,
    )

    assert result['reference']['reach_ratio']['final'] == pytest.approx(1.0)
    assert result['reference']['reach_ratio']['max'] == pytest.approx(1.0)
    assert (
        result['reference']['aligned_translation_delta_m']['rmse']
        < 1.0e-8
    )


def test_reference_metrics_report_under_reach(tmp_path: Path) -> None:
    """A scale-deficient candidate should retain its time-aligned reach ratio."""
    timestamps = np.linspace(0.0, 20.0, 41)
    reference_xyz = np.column_stack(
        (timestamps, np.zeros_like(timestamps), np.zeros_like(timestamps)),
    )
    candidate_xyz = 0.6 * reference_xyz
    reference_path = tmp_path / 'reference.tum'
    candidate_path = tmp_path / 'candidate.tum'
    _write_tum(reference_path, timestamps, reference_xyz)
    _write_tum(candidate_path, timestamps, candidate_xyz)

    result = EVALUATOR.evaluate(
        candidate_path,
        expected_endpoint_distance=20.0,
        reference_path=reference_path,
        min_reference_reach_m=10.0,
    )

    assert result['endpoint_distance_m'] == pytest.approx(12.0)
    assert result['reference']['reach_ratio']['final'] == pytest.approx(0.6)
    assert result['reference']['reach_ratio']['max'] == pytest.approx(0.6)


def _minimal_runner_inputs(tmp_path: Path) -> dict[str, Path]:
    dataset = tmp_path / 'dataset'
    bag = dataset / 'ros2_radar' / 'tunnel'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n')
    setup = tmp_path / 'setup.bash'
    setup.write_text('# test setup\n', encoding='utf-8')
    base = tmp_path / 'base.yaml'
    base.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')
    candidate = tmp_path / 'candidate.yaml'
    candidate.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')
    timestamps = np.array([0.0, 1.0, 2.0])
    reference = tmp_path / 'reference.tum'
    _write_tum(
        reference,
        timestamps,
        np.column_stack(
            (timestamps, np.zeros_like(timestamps), np.zeros_like(timestamps)),
        ),
    )
    return {
        'dataset': dataset,
        'setup': setup,
        'base': base,
        'candidate': candidate,
        'reference': reference,
    }


def test_runner_dry_run_prints_isolated_control_and_candidate(
    tmp_path: Path,
) -> None:
    """Dry-run should freeze both arms without requiring ROS executables."""
    paths = _minimal_runner_inputs(tmp_path)
    result = subprocess.run(
        [
            'bash',
            str(RUNNER_PATH),
            '--dataset-root',
            str(paths['dataset']),
            '--output-root',
            str(tmp_path / 'output'),
            '--setup',
            str(paths['setup']),
            '--base-params',
            str(paths['base']),
            '--candidate-preset',
            str(paths['candidate']),
            '--reference-tum',
            str(paths['reference']),
            '--candidate-param',
            'kinematic_blend_yaw_gate_min_scans:=10',
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.count('DRY_RUN') == 2
    assert 'ROS_DOMAIN_ID=210' in result.stdout
    assert 'ROS_DOMAIN_ID=211' in result.stdout
    assert 'degeneracy_off.ros.yaml' in result.stdout
    assert 'kinematic_blend_yaw_gate_min_scans:=10' in result.stdout


def test_runner_blocks_reserved_final_holdouts() -> None:
    """The development runner must not consume exp02/03/21."""
    result = subprocess.run(
        ['bash', str(RUNNER_PATH), '--sequence', 'exp02', '--dry-run'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'reserved final holdout' in result.stderr


def test_adopted_preset_freezes_scene_speed_and_yaw_gates() -> None:
    """The validated A/B arm must remain reproducible from the preset alone."""
    document = yaml.safe_load(PRESET_PATH.read_text(encoding='utf-8'))
    params = document['/**']['ros__parameters']

    assert params['kinematic_velocity_blend'] is True
    assert params['kinematic_blend_yaw_gate_min_scans'] == 10
    assert params['kinematic_blend_range_scene_gate'] is True
    assert params['kinematic_blend_scene_near_range_m'] == pytest.approx(3.0)
    assert params['kinematic_blend_scene_max_near_fraction'] == pytest.approx(
        0.5,
    )
    assert params['kinematic_blend_scene_far_range_m'] == pytest.approx(10.0)
    assert params['kinematic_blend_scene_min_far_fraction'] == pytest.approx(
        0.05,
    )
    assert params['kinematic_blend_scene_min_valid_points'] == 100
    assert params['kinematic_blend_scene_reenable_delay_sec'] == pytest.approx(
        60.0,
    )
    assert params['kinematic_blend_max_activation_speed_mps'] == pytest.approx(
        2.5,
    )
    assert params['kinematic_blend_speed_gate_min_scans'] == 3
    assert params['kinematic_blend_speed_reenable_delay_sec'] == pytest.approx(
        60.0,
    )
