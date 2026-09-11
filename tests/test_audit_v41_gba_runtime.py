"""Synthetic contracts for the v41 guarded-GBA runtime audit."""

import importlib.util
import json
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit_v41_runtime', ROOT / 'scripts/audit_v41_gba_runtime.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def make_inputs(tmp_path: Path, *, optimized_marker: bool = False,
                peak_rss_mib: float = 343.91796875):
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump({
        'GBA': {'max_rss_mib': 330.0},
    }), encoding='utf-8')
    run = {
        'accuracy_ground_truth_accessed': False,
        'accuracy_metrics_present': False,
        'execution': {
            'container_exit_status': 0,
            'mapper_exit_status': 0,
            'replay_exit_status': 0,
        },
        'completion': {
            'trajectory_complete': True,
            'process_exit_status': 0,
        },
        'runtime': {
            'processing_realtime_factor': 0.835,
            'peak_rss_mb': peak_rss_mib,
        },
        'trajectory': {'samples': 5831},
        'provenance': {'config_sha256': MODULE.sha256(config)},
    }
    run_json = tmp_path / 'run.json'
    run_json.write_text(json.dumps(run), encoding='utf-8')
    stages = [
        'cancel_rss_limit elapsed_sec=0 rss_kb=337508 hwm_kb=352172',
        'worker_exit_cancelled', 'frontend_complete',
        'backend_no_writeback', 'state_saved_unmodified',
    ]
    if optimized_marker:
        stages.append('writeback_complete')
    mapper_log = tmp_path / 'mapper.log'
    mapper_log.write_text(''.join(
        f'V41_GBA stage={stage}\n' for stage in stages), encoding='utf-8')
    baseline_state = tmp_path / 'baseline.state'
    candidate_state = tmp_path / 'candidate.state'
    baseline_map = tmp_path / 'baseline.pcd'
    candidate_map = tmp_path / 'candidate.pcd'
    baseline_state.write_bytes(b'unchanged state\n')
    candidate_state.write_bytes(baseline_state.read_bytes())
    baseline_map.write_bytes(b'unchanged map\n')
    candidate_map.write_bytes(baseline_map.read_bytes())
    return (run_json, mapper_log, config, candidate_state, baseline_state,
            candidate_map, baseline_map)


def test_clean_resource_cancel_is_classified_as_retirement(tmp_path):
    report = MODULE.build_report(*make_inputs(tmp_path))
    assert report['decision'] == (
        'REJECT_V41_RESOURCE_GATE_RETIRE_BUILTIN_HBA')
    assert all(report['contracts'].values())
    assert report['markers']['optimized_stages_present'] == []


def test_optimized_writeback_after_cancel_invalidates_contract(tmp_path):
    report = MODULE.build_report(
        *make_inputs(tmp_path, optimized_marker=True))
    assert report['decision'] == 'INVALID_V41_RUNTIME_CONTRACT'
    assert report['contracts']['cancellation_no_writeback'] is False


def test_identity_drift_invalidates_contract(tmp_path):
    inputs = list(make_inputs(tmp_path))
    inputs[3].write_bytes(b'mutated state\n')
    report = MODULE.build_report(*inputs)
    assert report['decision'] == 'INVALID_V41_RUNTIME_CONTRACT'
    assert report['contracts']['unmodified_v17_fallback'] is False


def test_non_exceeding_peak_invalidates_resource_rejection(tmp_path):
    report = MODULE.build_report(
        *make_inputs(tmp_path, peak_rss_mib=320.0))
    assert report['decision'] == 'INVALID_V41_RUNTIME_CONTRACT'
    assert report['contracts']['resource_gate_failed'] is False
