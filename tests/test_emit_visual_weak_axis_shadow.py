"""Contract tests for the report-only scalar shadow producer."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'visual_weak_shadow', ROOT / 'scripts/emit_visual_weak_axis_shadow.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _join_report() -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'decision': 'GO_WEAK_AXIS_PROJECTION',
        'weak_axis_definition': {
            'max_join_sec': 0.08,
            'eigen_ratio_max': 0.2,
        },
        'observations': [{
            'weak_row_index': 7,
            'stamp_sec': 10.0,
            'velocity_mps': -4.0,
            'pair_count': 1,
            'pair_indices': [3],
            'join_error_sec': 0.01,
            'median_inliers': 40.0,
            'median_tracks': 60.0,
            'median_residual_norm': 0.002,
            'eigen_ratio': 0.05,
            'weak_horizontal_norm': 1.0,
        }],
    }


def _visual_report() -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'config': {'min_inliers': 15, 'max_residual_norm': 0.02},
    }


def test_shadow_payload_is_metric_bounded_and_ground_truth_free():
    report = MODULE.build_shadow_document(
        _join_report(), _visual_report(), Path('join.json'), Path('pairs.json'))
    assert report['decision'] == 'GO_REPORT_ONLY_SHADOW_SOURCE'
    assert report['accuracy_ground_truth_accessed'] is False
    assert report['contract']['velocity_unit'] == 'm/s'
    observation = report['observations'][0]
    assert observation['stamp_sec'] == 10.0
    assert observation['velocity_mps'] == -4.0
    assert 0.0 <= observation['confidence'] <= 1.0
    assert report['contract']['estimator_state_mutated'] is False


def test_shadow_producer_rejects_stale_contract_speed_and_no_go():
    join = _join_report()
    join['observations'][0]['velocity_mps'] = 25.0
    report = MODULE.build_shadow_document(
        join, _visual_report(), Path('join.json'), Path('pairs.json'),
        max_speed_mps=20.0, min_confidence=0.0)
    assert report['decision'] == 'NO_GO_REPORT_ONLY_SHADOW_SOURCE'
    assert report['rejections'] == {'speed_bound': 1}

    no_go = _join_report()
    no_go['decision'] = 'NO_GO_WEAK_AXIS_PROJECTION'
    try:
        MODULE.build_shadow_document(
            no_go, _visual_report(), Path('join.json'), Path('pairs.json'))
    except ValueError as error:
        assert 'did not pass' in str(error)
    else:
        raise AssertionError('no-go join must not become a producer input')
