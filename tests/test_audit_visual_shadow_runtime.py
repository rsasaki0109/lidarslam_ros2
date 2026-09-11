"""Contract tests for the ground-truth-free v38 runtime audit."""

import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit_visual_shadow', ROOT / 'scripts/audit_visual_shadow_runtime.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _tum(offsets: list[float]) -> str:
    return ''.join(
        f'{index}.0 {value} 0 0 0 0 0 1\n'
        for index, value in enumerate(offsets))


def test_audit_locates_post_observation_divergence_and_source_failures(tmp_path):
    baseline = tmp_path / 'baseline.tum'
    candidate = tmp_path / 'candidate.tum'
    payload = tmp_path / 'payload.json'
    source = tmp_path / 'candidate.patch'
    baseline.write_text(_tum([0, 1, 2, 3, 4]), encoding='utf-8')
    candidate.write_text(_tum([0, 1, 2, 5, 9]), encoding='utf-8')
    payload.write_text(json.dumps({
        'accuracy_ground_truth_accessed': False,
        'observations': [{'stamp_sec': 2.0}]}), encoding='utf-8')
    source.write_text(
        'void apply_visual_longitudinal_shadow() { '
        'weak_axis_speed += correction; }', encoding='utf-8')

    report = MODULE.build_audit(baseline, candidate, payload, source)
    assert report['accuracy_ground_truth_accessed'] is False
    assert report['decision'] == 'FAIL_VISUAL_SHADOW_RUNTIME_CONTRACT'
    crossing = report['trajectory']['first_threshold_crossings']['1_m']
    assert crossing['seconds_after_first_observation'] == 1.0
    assert report['trajectory'][
        'maximum_position_delta_after_first_observation_m'] == 5.0
    assert set(report['failed_source_contracts']) == {
        'candidate_axis_projection_uses_vector_observation',
        'mapper_state_is_not_feedback_target',
        'observation_consumed_at_most_once',
        'state_timestamp_controls_application',
    }
