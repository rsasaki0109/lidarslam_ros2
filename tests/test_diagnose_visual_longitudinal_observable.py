"""Contract tests for the sensor-only visual motion diagnostic."""

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'visual_longitudinal',
    ROOT / 'scripts/diagnose_visual_longitudinal_observable.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_quaternion_rotation_and_projection_are_deterministic():
    rotation = MODULE.quaternion_xyzw_rotation((0.0, 0.0, 0.0, 1.0))
    model = MODULE.CameraModel(
        width=640, height=480, fx=400.0, fy=400.0, cx=320.0, cy=240.0,
        distortion_model='plumb_bob', distortion=(0.0, 0.0, 0.0, 0.0))
    pixels, transformed, valid = MODULE.project_points(
        np.array([[0.0, 0.0, 5.0], [1.0, 0.0, 5.0]]), model, rotation,
        np.zeros(3))
    np.testing.assert_allclose(pixels, [[320.0, 240.0], [400.0, 240.0]])
    np.testing.assert_allclose(transformed[0], [0.0, 0.0, 5.0])
    assert valid.tolist() == [True, True]


def test_sensor_transform_contract_keeps_camera_to_base_for_motion_direction():
    camera_to_base = MODULE.quaternion_xyzw_rotation(
        (0.0, 0.0, 0.7071067811865476, 0.7071067811865476))
    lidar_to_base = np.eye(3)
    lidar_to_camera, motion_to_base, offset = MODULE.compose_sensor_transforms(
        camera_to_base, np.array([1.0, 2.0, 3.0]), lidar_to_base,
        np.array([1.0, 2.5, 3.0]))
    np.testing.assert_allclose(lidar_to_camera, camera_to_base.T)
    np.testing.assert_allclose(motion_to_base, camera_to_base)
    np.testing.assert_allclose(offset, camera_to_base.T @ np.array([0.0, 0.5, 0.0]))


def test_metric_scale_recovers_known_translation():
    rng = np.random.default_rng(7)
    points = np.column_stack((
        rng.uniform(-2.0, 2.0, 80),
        rng.uniform(-1.0, 1.0, 80),
        rng.uniform(5.0, 20.0, 80)))
    rotation = np.eye(3)
    direction = np.array([0.0, 0.0, 1.0])
    observations = (points + 0.8 * direction)[:, :2]
    observations /= (points + 0.8 * direction)[:, 2:3]
    solved = MODULE.solve_metric_scale(
        points, observations, rotation, direction, max_scale_m=2.0)
    assert solved is not None
    scale, residual, solved_direction = solved
    assert abs(scale - 0.8) < 1.0e-3
    assert residual < 1.0e-5
    np.testing.assert_allclose(solved_direction, direction)


def test_metric_scale_rejects_projective_outliers_without_lower_bound_lock():
    rng = np.random.default_rng(12)
    points = np.column_stack((
        rng.uniform(-2.0, 2.0, 100),
        rng.uniform(-1.0, 1.0, 100),
        rng.uniform(5.0, 20.0, 100)))
    direction = np.array([0.2, 0.1, 0.97])
    direction /= np.linalg.norm(direction)
    translated = points + 0.35 * direction
    observations = translated[:, :2] / translated[:, 2:3]
    observations[:20] = rng.uniform(-0.5, 0.5, (20, 2))
    solved = MODULE.solve_metric_scale(
        points, observations, np.eye(3), direction, max_scale_m=2.0)
    assert solved is not None
    scale, residual, solved_direction = solved
    assert abs(scale - 0.35) < 0.02
    assert residual < 0.02
    np.testing.assert_allclose(solved_direction, direction)


def test_summary_requires_sustained_sensor_motion_without_ground_truth():
    results = [
        MODULE.PairResult(True, 'ok', direction_base=(1.0, 0.0, 0.0),
                          speed_mps=2.0, scale_m=0.2, residual_norm=0.001),
    ] * 5
    report = MODULE.summarize(
        results, MODULE.Counter(), [Path('/tmp/sensor.bag')], '/camera',
        '/points', 5, 5, {
            'min_valid_pairs': 5, 'min_valid_streak': 5,
            'min_valid_fraction': 0.5, 'min_direction_coherence': 0.35,
        })
    assert report['accuracy_ground_truth_accessed'] is False
    assert report['decision'] == 'GO_SENSOR_MOTION'
