#!/usr/bin/env python3
"""Resume a frozen SOTA-v2 suite while preserving recorded stage failures.

This recovery harness intentionally lives outside the candidate source roots.
It reuses the frozen orchestrator's validation and stage construction, verifies
that every recorded command is an exact prefix of the frozen 21-stage plan,
and then attempts every remaining stage even when a rival returns non-zero.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[2]


def atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    """Replace one JSON status file atomically."""
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    temporary.replace(path)


def utc_now() -> str:
    """Return one timezone-aware event timestamp."""
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256(path: Path) -> str:
    """Hash one recovery or frozen control artifact."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_equal(label: str, actual: Any, expected: Any) -> None:
    """Reject any drift from the frozen or already-recorded plan."""
    if actual != expected:
        raise ValueError(
            f'{label} mismatch: expected {expected!r}, got {actual!r}')


def validated_resume_state(args: argparse.Namespace) -> tuple[
        dict[str, Any], dict[str, Any], list[dict[str, Any]], Path, Any]:
    """Validate all freezes and the recorded stage prefix without mutation."""
    sys.path.insert(0, str(ROOT / 'scripts'))
    try:
        import run_sota_v2_blind_suite as suite
        from competitive_candidate_provenance import verify_candidate_manifest
    except ModuleNotFoundError as error:
        raise RuntimeError(
            'the archived SOTA-v2 recovery dependencies are not present in '
            'this revision') from error

    status_path = args.output / 'orchestration.json'
    if not status_path.is_file():
        raise ValueError(f'missing orchestration status: {status_path}')

    subprocess.run([
        sys.executable,
        str(ROOT / 'scripts/validate_competitive_sota_profile.py'),
        '--profile', str(args.profile), '--require-stage', 'inputs_frozen'],
        cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    document = yaml.safe_load(args.profile.read_text(encoding='utf-8'))
    profile = document['competitive_slam_profile']
    matrix = yaml.safe_load(args.matrix.read_text(encoding='utf-8'))
    rows = suite.validate_matrix(args.matrix, args.profile, profile)
    candidate = verify_candidate_manifest(ROOT, args.candidate_manifest)
    stages = suite.stage_commands(args, matrix, rows)
    plan = json.loads(status_path.read_text(encoding='utf-8'))

    require_equal('profile path', plan['profile']['path'], str(args.profile))
    require_equal('profile sha256', plan['profile']['sha256'],
                  suite.sha256(args.profile))
    require_equal('matrix path', plan['matrix']['path'], str(args.matrix))
    require_equal('matrix sha256', plan['matrix']['sha256'],
                  suite.sha256(args.matrix))
    for key in ('path', 'sha256', 'source_tree_sha256', 'source_file_count'):
        require_equal(f'candidate {key}', plan['candidate'][key], candidate[key])
    require_equal('stage count', plan['stage_count'], len(stages))
    if len(plan['stages']) >= len(stages):
        raise ValueError('no remaining stages to resume')
    for index, recorded in enumerate(plan['stages']):
        require_equal(f'stage {index} name', recorded['name'],
                      stages[index]['name'])
        require_equal(f'stage {index} command', recorded['command'],
                      stages[index]['command'])
        if 'returncode' not in recorded or 'finished_at' not in recorded:
            raise ValueError(f'stage {index} is not a completed record')
    if not any(int(row['returncode']) != 0 for row in plan['stages']):
        raise ValueError('resume requires at least one recorded stage failure')
    return plan, profile, stages, status_path, suite


def main() -> int:
    """Resume at the first unrecorded stage and preserve every result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, default=(
        ROOT / 'configs/slam_benchmark_profiles/sota_v2_execution_matrix.yaml'))
    parser.add_argument('--profile', type=Path, default=(
        ROOT / 'configs/slam_benchmark_profiles/competitive_slam_sota_v2.yaml'))
    parser.add_argument('--candidate-manifest', type=Path, required=True)
    parser.add_argument('--fast-livo2-asset-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    for name in ('matrix', 'profile', 'candidate_manifest',
                 'fast_livo2_asset_root', 'output'):
        setattr(args, name, getattr(args, name).resolve())

    plan, profile, stages, status_path, suite = validated_resume_state(args)
    next_index = len(plan['stages'])
    summary = {
        'recorded_stage_count': next_index,
        'resume_from_zero_based_index': next_index,
        'resume_from_stage': stages[next_index]['name'],
        'remaining_stage_count': len(stages) - next_index,
        'recorded_failures': [
            {'name': row['name'], 'returncode': row['returncode']}
            for row in plan['stages'] if int(row['returncode']) != 0],
    }
    print(json.dumps(summary, indent=2), flush=True)
    if args.check_only:
        return 0

    recovery_dir = args.output / 'protocol_recovery'
    recovery_dir.mkdir(exist_ok=True)
    frozen_copy = recovery_dir / Path(__file__).name
    shutil.copyfile(Path(__file__).resolve(), frozen_copy)
    event = {
        'id': 'continue_after_recorded_stage_failure_v1',
        'applied_at': utc_now(),
        'reason': (
            'The frozen orchestrator stopped after recording a non-zero rival '
            'stage. Continue the exact remaining command suffix so every '
            'preregistered raw stage is attempted; preserve all failures.'),
        'trigger_stage': plan['stages'][-1]['name'],
        'trigger_returncode': plan['stages'][-1]['returncode'],
        'resume_from_zero_based_index': next_index,
        'resume_from_stage': stages[next_index]['name'],
        'frozen_orchestrator_sha256': suite.sha256(
            ROOT / 'scripts/run_sota_v2_blind_suite.py'),
        'recovery_harness': {
            'path': str(frozen_copy), 'sha256': sha256(frozen_copy)},
        'accuracy_metrics_inspected': False,
        'algorithm_or_dataset_configuration_changed': False,
    }
    plan.setdefault('protocol_recovery_events', []).append(event)
    plan['status'] = 'running_with_recorded_stage_failures'
    atomic_write_json(status_path, plan)

    for stage in stages[next_index:]:
        row = {
            **stage,
            'started_at': utc_now(),
            'quiet_preflight': suite.require_quiet(profile),
            'recovery_event_id': event['id'],
        }
        completed = subprocess.run(stage['command'], cwd=ROOT, check=False)
        row.update({'returncode': completed.returncode,
                    'finished_at': utc_now()})
        plan['stages'].append(row)
        plan['status'] = 'running_with_recorded_stage_failures'
        atomic_write_json(status_path, plan)

    failures = [row for row in plan['stages']
                if int(row['returncode']) != 0]
    plan['stage_failure_count'] = len(failures)
    plan['status'] = ('raw_measurements_complete_unscored_with_stage_failures'
                      if failures else 'raw_measurements_complete_unscored')
    atomic_write_json(status_path, plan)
    print(json.dumps({
        'requested_stages': len(stages),
        'recorded_stages': len(plan['stages']),
        'stage_failures': [
            {'name': row['name'], 'returncode': row['returncode']}
            for row in failures],
        'status': plan['status'],
    }, indent=2), flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError, TypeError,
            json.JSONDecodeError, yaml.YAMLError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
