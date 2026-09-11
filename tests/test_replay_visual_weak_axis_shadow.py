"""Contract tests for the opt-in ROS shadow replay bridge."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'visual_weak_shadow_replay',
    ROOT / 'scripts/replay_visual_weak_axis_shadow.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _report() -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'decision': 'GO_REPORT_ONLY_SHADOW_SOURCE',
        'contract': {'max_speed_mps': 20.0},
        'observations': [
            {'stamp_sec': 1.0, 'velocity_mps': -2.0, 'confidence': 0.5},
            {'stamp_sec': 1.1, 'velocity_mps': 3.0, 'confidence': 0.8},
        ],
    }


def test_bridge_validates_sorted_metric_payload_without_ros_import():
    assert MODULE.validated_observations(_report()) == [
        {'stamp_sec': 1.0, 'velocity_mps': -2.0, 'confidence': 0.5},
        {'stamp_sec': 1.1, 'velocity_mps': 3.0, 'confidence': 0.8},
    ]


def test_bridge_rejects_bad_order_speed_and_no_go():
    report = _report()
    report['observations'][1]['stamp_sec'] = 0.9
    try:
        MODULE.validated_observations(report)
    except ValueError as error:
        assert 'not ordered' in str(error)
    else:
        raise AssertionError('out-of-order timestamps must be rejected')

    report = _report()
    report['observations'][0]['velocity_mps'] = 21.0
    try:
        MODULE.validated_observations(report)
    except ValueError as error:
        assert 'speed bound' in str(error)
    else:
        raise AssertionError('out-of-bound velocity must be rejected')

    report = _report()
    report['decision'] = 'NO_GO_REPORT_ONLY_SHADOW_SOURCE'
    try:
        MODULE.validated_observations(report)
    except ValueError as error:
        assert 'did not pass' in str(error)
    else:
        raise AssertionError('no-go report must not be replayed')
