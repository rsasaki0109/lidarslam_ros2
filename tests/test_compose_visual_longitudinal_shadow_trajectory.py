"""Contract tests for the isolated v39 output trajectory composer."""

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'compose_visual_shadow',
    ROOT / 'scripts/compose_visual_longitudinal_shadow_trajectory.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _config() -> dict:
    return {
        'weak_axis': {
            'eigen_ratio_max': 0.2, 'horizontal_norm_min': 0.9,
            'min_speed_mps': 3.0, 'required_streak_scans': 2},
        'timing': {
            'max_join_sec': 0.08, 'max_observation_age_sec': 0.2,
            'max_integration_dt_sec': 1.0},
        'filter': {
            'gain': 1.0, 'max_velocity_change_mps': 10.0,
            'max_speed_mps': 20.0},
    }


def _poses() -> list:
    return [MODULE.Pose(
        f'{index}.0', float(index), np.asarray([10.0 * index, 0.0, 0.0]),
        np.asarray([0.0, 0.0, 0.0, 1.0]), ('0', '0', '0', '1'))
        for index in range(5)]


def _weak_rows(ratio: float = 0.1) -> list[dict[str, str]]:
    return [{
        't': str(float(index)), 'evalue0': str(ratio), 'evalue1': '1.0',
        'weak_eigen_x': '1.0', 'weak_eigen_y': '0.0',
        'weak_eigen_z': '0.0', 'weak_horizontal_norm': '1.0',
        'velocity_x': '10.0', 'velocity_y': '0.0', 'velocity_z': '0.0',
    } for index in range(5)]


def _vector(decision: str = 'GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE') -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'decision': decision,
        'contract': {
            'velocity_frame': 'base', 'axis_projection_at_consumer': True,
            'mapper_state_mutated': False, 'max_speed_mps': 20.0},
        'gravity_axis_world': [0.0, 0.0, 1.0],
        'observations': [] if decision.startswith('NO_GO') else [{
            'stamp_sec': 2.0, 'velocity_base_mps': [4.0, 0.0, 0.0],
            'confidence': 1.0}],
    }


def test_output_shadow_changes_only_position_and_consumes_once():
    poses = _poses()
    positions, runtime = MODULE.compose_shadow_positions(
        poses, _weak_rows(), _vector(), _config())
    assert runtime['decision'] == 'GO_OUTPUT_ONLY_SHADOW_SCREEN'
    assert runtime['applied_observations'] == 1
    np.testing.assert_allclose(positions[0], poses[0].position)
    np.testing.assert_allclose(positions[1], poses[1].position)
    assert positions[2][0] == 14.0
    assert positions[3][0] == 18.0
    assert positions[4][0] == 22.0
    for pose in poses:
        np.testing.assert_allclose(pose.quaternion, [0.0, 0.0, 0.0, 1.0])


def test_no_go_and_well_conditioned_inputs_are_exact_noops():
    poses = _poses()
    positions, runtime = MODULE.compose_shadow_positions(
        poses, _weak_rows(), _vector('NO_GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE'),
        _config())
    assert runtime['decision'] == 'NO_OP_OUTPUT_SHADOW'
    for position, pose in zip(positions, poses):
        np.testing.assert_array_equal(position, pose.position)


def test_reference_point_track_receives_same_world_translation():
    poses = _poses()
    reference_poses = [MODULE.Pose(
        pose.stamp_text, pose.stamp,
        pose.position + np.asarray([0.0, 2.0, 1.0]),
        np.asarray([0.0, 0.0, 1.0, 0.0]), ('0', '0', '1', '0'))
        for pose in poses]
    shadow_positions, _ = MODULE.compose_shadow_positions(
        poses, _weak_rows(), _vector(), _config())
    reference_positions = MODULE.translate_reference_positions(
        poses, reference_poses, shadow_positions)
    for baseline, shadow, reference, translated in zip(
            poses, shadow_positions, reference_poses, reference_positions):
        np.testing.assert_allclose(
            translated - reference.position, shadow - baseline.position)
        np.testing.assert_allclose(
            reference.quaternion, [0.0, 0.0, 1.0, 0.0])

    positions, runtime = MODULE.compose_shadow_positions(
        poses, _weak_rows(ratio=1.0), _vector(), _config())
    assert runtime['applied_observations'] == 0
    for position, pose in zip(positions, poses):
        np.testing.assert_array_equal(position, pose.position)
