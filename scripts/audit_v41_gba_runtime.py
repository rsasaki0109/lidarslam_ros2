#!/usr/bin/env python3
"""Classify a v41 guarded-GBA run without opening accuracy or map truth."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import yaml


MARKER = re.compile(r'V41_GBA stage=([^ ]+)(.*)$')
FIELD = re.compile(r'([a-z_]+)=([^ ]+)')
OPTIMIZED_STAGES = {
    'isam_begin', 'isam_complete', 'writeback_complete',
    'map_publish_complete', 'state_saved_optimized',
}
REQUIRED_CANCEL_STAGES = {
    'cancel_rss_limit', 'worker_exit_cancelled', 'frontend_complete',
    'backend_no_writeback', 'state_saved_unmodified',
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_markers(log_text: str) -> list[dict[str, Any]]:
    markers: list[dict[str, Any]] = []
    for line in log_text.splitlines():
        match = MARKER.search(line)
        if not match:
            continue
        values: dict[str, Any] = {'stage': match.group(1)}
        for key, raw in FIELD.findall(match.group(2)):
            try:
                values[key] = float(raw) if '.' in raw else int(raw)
            except ValueError:
                values[key] = raw
        markers.append(values)
    return markers


def build_report(
        run_json_path: Path, mapper_log_path: Path, config_path: Path,
        candidate_state_path: Path, baseline_state_path: Path,
        candidate_map_path: Path, baseline_map_path: Path,
        ) -> dict[str, Any]:
    run = json.loads(run_json_path.read_text(encoding='utf-8'))
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    markers = parse_markers(
        mapper_log_path.read_text(encoding='utf-8', errors='replace'))
    counts = Counter(marker['stage'] for marker in markers)
    cancel_rows = [row for row in markers
                   if row['stage'] == 'cancel_rss_limit']
    max_rss_mib = float(config['GBA']['max_rss_mib'])
    peak_rss_mib = float(run['runtime']['peak_rss_mb'])
    cancel_hwm_kb = (
        int(cancel_rows[0].get('hwm_kb', -1))
        if len(cancel_rows) == 1 else -1)

    clean_completion = (
        run['execution'] == {
            'container_exit_status': 0,
            'mapper_exit_status': 0,
            'replay_exit_status': 0,
        } and
        run['completion']['trajectory_complete'] is True and
        run['completion']['process_exit_status'] == 0 and
        run['accuracy_ground_truth_accessed'] is False and
        run['accuracy_metrics_present'] is False)
    cancellation_contract = (
        REQUIRED_CANCEL_STAGES.issubset(counts) and
        counts['cancel_rss_limit'] == 1 and
        not (OPTIMIZED_STAGES & counts.keys()))
    resource_gate_failed = (
        math.isfinite(peak_rss_mib) and peak_rss_mib > max_rss_mib and
        cancel_hwm_kb == int(round(peak_rss_mib * 1024.0)))
    candidate_state_sha = sha256(candidate_state_path)
    baseline_state_sha = sha256(baseline_state_path)
    candidate_map_sha = sha256(candidate_map_path)
    baseline_map_sha = sha256(baseline_map_path)
    unmodified_fallback = (
        candidate_state_sha == baseline_state_sha and
        candidate_map_sha == baseline_map_sha)
    config_bound = (
        sha256(config_path) ==
        run['provenance']['config_sha256'])

    valid_rejection = (
        clean_completion and cancellation_contract and resource_gate_failed
        and unmodified_fallback and config_bound)
    return {
        'schema_version': 1,
        'status': 'ground_truth_free_v41_guarded_gba_runtime_audit',
        'accuracy_ground_truth_accessed': False,
        'audit_mutates_trajectory_or_map': False,
        'decision': (
            'REJECT_V41_RESOURCE_GATE_RETIRE_BUILTIN_HBA'
            if valid_rejection else 'INVALID_V41_RUNTIME_CONTRACT'),
        'contracts': {
            'clean_completion': clean_completion,
            'cancellation_no_writeback': cancellation_contract,
            'resource_gate_failed': resource_gate_failed,
            'unmodified_v17_fallback': unmodified_fallback,
            'config_hash_bound': config_bound,
        },
        'runtime': {
            'processing_realtime_factor':
                run['runtime']['processing_realtime_factor'],
            'peak_rss_mib': peak_rss_mib,
            'max_rss_mib': max_rss_mib,
            'cancel_hwm_kb': cancel_hwm_kb,
            'trajectory_samples': run['trajectory']['samples'],
        },
        'markers': {
            'counts': dict(sorted(counts.items())),
            'optimized_stages_present': sorted(
                OPTIMIZED_STAGES & counts.keys()),
        },
        'identity': {
            'candidate_state_sha256': candidate_state_sha,
            'baseline_state_sha256': baseline_state_sha,
            'candidate_map_sha256': candidate_map_sha,
            'baseline_map_sha256': baseline_map_sha,
        },
        'inputs': {
            'run_json': str(run_json_path.resolve()),
            'run_json_sha256': sha256(run_json_path),
            'mapper_log': str(mapper_log_path.resolve()),
            'mapper_log_sha256': sha256(mapper_log_path),
            'config': str(config_path.resolve()),
            'config_sha256': sha256(config_path),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-json', required=True, type=Path)
    parser.add_argument('--mapper-log', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--candidate-state', required=True, type=Path)
    parser.add_argument('--baseline-state', required=True, type=Path)
    parser.add_argument('--candidate-map', required=True, type=Path)
    parser.add_argument('--baseline-map', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    report = build_report(
        args.run_json, args.mapper_log, args.config, args.candidate_state,
        args.baseline_state, args.candidate_map, args.baseline_map)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'decision': report['decision'],
        'contracts': report['contracts'],
        'output': str(args.output.resolve()),
    }, indent=2, sort_keys=True))
    return 0 if report['decision'].startswith('REJECT_') else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError, yaml.YAMLError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
