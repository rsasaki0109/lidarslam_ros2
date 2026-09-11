"""Contract tests for the v39 base-frame visual velocity producer."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'visual_vector_shadow',
    ROOT / 'scripts/emit_visual_velocity_vector_shadow.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _join(decision: str = 'GO_WEAK_AXIS_PROJECTION') -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'decision': decision,
        'weak_axis_definition': {
            'max_join_sec': 0.08, 'eigen_ratio_max': 0.2},
        'quality': {'gravity_axis_world': [0.0, 0.0, 1.0]},
        'observations': [{
            'weak_row_index': 7, 'stamp_sec': 10.0,
            'velocity_mps': 4.0, 'pair_count': 2,
            'pair_indices': [0, 1], 'join_error_sec': 0.01,
            'median_inliers': 40.0, 'median_tracks': 60.0,
            'median_residual_norm': 0.002, 'eigen_ratio': 0.05,
            'weak_horizontal_norm': 1.0,
        }],
    }


def _visual() -> dict:
    return {
        'accuracy_ground_truth_accessed': False,
        'config': {'min_inliers': 15, 'max_residual_norm': 0.02},
        'pairs': [
            {'valid': True, 'direction_base': [1.0, 0.0, 0.0],
             'speed_mps': 4.0},
            {'valid': True, 'direction_base': [0.0, 1.0, 0.0],
             'speed_mps': 6.0},
        ],
    }


def test_vector_contract_preserves_base_frame_and_consumer_projection():
    report = MODULE.build_vector_document(
        _join(), _visual(), Path('join.json'), Path('pairs.json'),
        min_confidence=0.0)
    assert report['decision'] == 'GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE'
    assert report['accuracy_ground_truth_accessed'] is False
    assert report['contract']['velocity_frame'] == 'base'
    assert report['contract']['axis_projection_at_consumer'] is True
    assert report['contract']['mapper_state_mutated'] is False
    assert report['observations'][0]['velocity_base_mps'] == [2.0, 3.0, 0.0]


def test_no_go_join_cannot_emit_candidate_observations():
    report = MODULE.build_vector_document(
        _join('NO_GO_WEAK_AXIS_PROJECTION'), _visual(),
        Path('join.json'), Path('pairs.json'), require_go=False)
    assert report['decision'] == 'NO_GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE'
    assert report['observations'] == []


def test_terminal_summary_omits_large_observation_payload(tmp_path):
    report = MODULE.build_vector_document(
        _join(), _visual(), Path('join.json'), Path('pairs.json'),
        min_confidence=0.0)
    summary = MODULE.terminal_summary(report, tmp_path / 'vectors.json')
    assert summary['decision'] == 'GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE'
    assert summary['emitted_observations'] == 1
    assert 'observations' not in summary
