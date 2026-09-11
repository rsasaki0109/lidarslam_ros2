#!/usr/bin/env python3
"""Compose runner artifacts into the strict competitive sequence-gate schema."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any

import yaml

# Direct source-checkout invocation must resolve the canonical package without
# requiring PYTHONPATH.  Installed entry points use the package normally.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity)


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _runtime_value(run: dict[str, Any], key: str) -> Any:
    runtime = run.get('runtime') or {}
    value = runtime.get(key)
    if value is None and key == 'peak_rss_mb':
        mapper = runtime.get('mapper') or {}
        value = mapper.get(key)
    return value


def _completion(run: dict[str, Any]) -> tuple[bool, int | None]:
    completion = run.get('completion') or run
    complete = bool(completion.get('trajectory_complete'))
    status = completion.get('process_exit_status')
    return complete, None if status is None else int(status)


def calibration_hash(manifest: dict[str, Any]) -> str:
    hashes = manifest.get('hashes') or {}
    value = hashes.get('calibration_sha256')
    if value is None:
        value = hashes.get('calibration_archive_sha256')
    if not isinstance(value, str) or len(value) != 64:
        raise ValueError('input manifest has no valid calibration SHA-256')
    try:
        int(value, 16)
    except ValueError as exc:
        raise ValueError('input manifest calibration SHA-256 is not hexadecimal') from exc
    return value


def compose(*, system: str, track: str, manifest_path: Path,
            reference_path: Path, machine_path: Path,
            trajectory_path: Path, map_path: Path,
            profile_path: Path = DEFAULT_PROFILE,
            processing_path: Path | None = None,
            loop_path: Path | None = None,
            visual_path: Path | None = None) -> dict[str, Any]:
    contract = yaml.safe_load(profile_path.read_text())['competitive_slam_profile']
    closure_policy = contract.get('evidence_gate_v2', {}).get(
        'rival_source_closure', {})
    closure_identity = None
    if isinstance(closure_policy, dict) and closure_policy.get('required') is True:
        closure_identity = current_rival_source_closure_identity(
            {'competitive_slam_profile': contract}, root=ROOT)
    if track not in contract['tracks']:
        raise ValueError(f'unknown competition track: {track}')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('status') != 'frozen':
        raise ValueError('input manifest is not frozen')
    machine = json.loads(machine_path.read_text())
    machine_id = machine.get('machine_id')
    if not isinstance(machine_id, str) or len(machine_id) != 64:
        raise ValueError('machine artifact has no SHA-256 machine_id')
    trajectory = json.loads(trajectory_path.read_text())
    mapping_source = json.loads(map_path.read_text())
    aggregate = trajectory['aggregate']
    common_reference_sha256 = trajectory['reference']['common_sha256']
    if not isinstance(common_reference_sha256, str) or len(
            common_reference_sha256) != 64:
        raise ValueError('trajectory summary has no valid common-reference SHA-256')
    runs = trajectory['runs']
    valid_repetitions = int(trajectory['valid_repetitions'])
    failures = sum(
        not complete or status != 0
        for complete, status in (_completion(run) for run in runs))

    rss_values = [
        float(value) for run in runs
        for value in [_runtime_value(run, 'peak_rss_mb')]
        if value is not None]
    peak_rss = aggregate.get('peak_rss_max_mb')
    if peak_rss is None:
        if not rss_values:
            raise ValueError('trajectory summary has no peak RSS evidence')
        peak_rss = max(rss_values)
    runtime: dict[str, Any] = {'peak_rss_max_mb': float(peak_rss)}

    # Preserve authoritative per-run resource receipts when a producer
    # declares them.  The sequence gate must reopen every repetition; keeping
    # only the aggregate peak would allow a missing or contaminated run to be
    # hidden by a convenient maximum.  Do not synthesize receipts for legacy
    # report-only trajectories.
    declared_resource_runs = any(
        isinstance(run, dict) and any(
            key in run for key in ('resource_evidence', 'resource_receipt',
                                   'resource'))
        for run in runs)
    declared_resources = trajectory.get('resource_evidence')
    if declared_resources is None:
        declared_resources = trajectory.get('resource_receipts')
    if isinstance(declared_resources, list):
        declared_resource_runs = True
        if len(declared_resources) != len(runs):
            raise ValueError(
                'trajectory resource evidence count must match trajectory runs')
    resource_rows: list[dict[str, Any]] | None = None
    execution_rows: list[dict[str, Any]] | None = None
    declared_execution = any(
        isinstance(run, dict) and any(
            key in run for key in (
                'execution_receipt_sha256', 'execution_receipt_file_sha256',
                'campaign_id'))
        for run in runs)
    if declared_execution:
        execution_rows = []
        for index, run in enumerate(runs, 1):
            if not isinstance(run, dict):
                raise ValueError('execution receipt run must be a mapping')
            canonical = run.get('execution_receipt_sha256')
            file_sha = run.get('execution_receipt_file_sha256')
            campaign_id = run.get('campaign_id')
            if any(not isinstance(value, str) or len(value) != 64 or
                   any(char not in '0123456789abcdef' for char in value)
                   for value in (canonical, file_sha, campaign_id)):
                raise ValueError(
                    f'run {index} has incomplete execution receipt identity')
            execution_rows.append({
                'dataset': manifest['sequence'],
                'run_index': index,
                'campaign_id': campaign_id,
                'execution_receipt_sha256': canonical,
                'execution_receipt_file_sha256': file_sha,
            })
    if declared_resource_runs:
        resource_rows = []
        for index, run in enumerate(runs, 1):
            value: Any = None
            if isinstance(declared_resources, list) and index <= len(declared_resources):
                value = declared_resources[index - 1]
            if isinstance(run, dict):
                value = run.get('resource_evidence', run.get(
                    'resource_receipt', run.get('resource', value)))
            row: dict[str, Any] = {
                'dataset': manifest['sequence'],
                'sequence': manifest['sequence'],
                'run_index': index,
                'complete': bool((run.get('completion') or run).get(
                    'complete', (run.get('completion') or run).get(
                        'trajectory_complete', False))),
                'process_exit_status': (run.get('completion') or run).get(
                    'process_exit_status'),
                'trajectory_complete': bool(
                    (run.get('completion') or run).get(
                        'trajectory_complete', False)),
                'sequence_failure': bool(
                    (run.get('completion') or run).get(
                        'sequence_failure', False)),
                'catastrophic_failure': bool(
                    (run.get('completion') or run).get(
                        'catastrophic_failure', False)),
                'verified_false_loops': (run.get('completion') or run).get(
                    'verified_false_loops', 0),
                'runtime': {
                    'peak_rss_mb': _runtime_value(run, 'peak_rss_mb')},
                'resource_evidence': value,
            }
            resource_rows.append(row)

    if processing_path is not None:
        processing = json.loads(processing_path.read_text())
        if processing.get('valid_processing_rtf_evidence'):
            runtime['processing_rtf_median'] = float(
                processing['processing_rtf_upper_bound_median'])
    else:
        rtf_values = [
            float(value) for run in runs
            for value in [_runtime_value(run, 'processing_realtime_factor')]
            if value is not None]
        if len(rtf_values) == valid_repetitions:
            runtime['processing_rtf_median'] = statistics.median(rtf_values)

    # M6a10's online interval is the primary realtime metric.  Keep it
    # separate from the legacy wall/processing value so a missing phase
    # document cannot be silently treated as a valid sequence result.
    online_values = [
        float(value) for run in runs
        for value in [_runtime_value(run, 'online_compute_rtf')]
        if value is not None]
    if len(online_values) == valid_repetitions:
        runtime['online_compute_rtf_median'] = statistics.median(online_values)
    wall_values = [
        float(value) for run in runs
        for value in [_runtime_value(run, 'wall_realtime_factor')]
        if value is not None]
    if len(wall_values) == valid_repetitions:
        runtime['wall_realtime_factor_median'] = statistics.median(wall_values)
    phase_versions = {
        _runtime_value(run, 'phase_contract_version') for run in runs
        if _runtime_value(run, 'phase_contract_version') is not None}
    phase_modes = {
        _runtime_value(run, 'phase_mode') for run in runs
        if _runtime_value(run, 'phase_mode') is not None}
    if phase_versions == {'m6a10-online-compute-v2'} and len(phase_modes) == 1:
        runtime['phase_contract_version'] = 'm6a10-online-compute-v2'
        runtime['phase_mode'] = next(iter(phase_modes))
        runtime['paced_followability_passed'] = all(
            _runtime_value(run, 'paced_followability_passed') is True
            for run in runs)
        runtime['unpaced_throughput_gate_passed'] = all(
            _runtime_value(run, 'unpaced_throughput_gate_passed') is True
            for run in runs)

    map_valid_repetitions = int(mapping_source['valid_repetitions'])
    map_valid = bool(mapping_source.get(
        'aggregation_valid', mapping_source.get('aggregate') is not None))
    meaningful = int(mapping_source.get(
        'meaningful_repetitions',
        map_valid_repetitions if map_valid else 0))
    mapping: dict[str, Any] = {
        'aggregation_valid': map_valid,
        'valid_repetitions': map_valid_repetitions,
        'meaningful_repetitions': meaningful,
    }
    map_aggregate = mapping_source.get('aggregate')
    if map_valid:
        if not isinstance(map_aggregate, dict):
            raise ValueError('valid map summary has no aggregate')
        mapping.update({
            'plane_thickness_mean_worst_m': float(
                map_aggregate['plane_thickness_mean_worst_m']),
            'plane_thickness_p95_worst_m': float(
                map_aggregate['plane_thickness_p95_worst_m']),
            'planar_coverage_worst': float(
                map_aggregate['planar_coverage_worst']),
        })

    result: dict[str, Any] = {
        'schema_version': 1,
        'system': system,
        'sequence': manifest['sequence'],
        'track': track,
        'input_manifest_sha256': sha256(manifest_path),
        'reference_sha256': sha256(reference_path),
        'evaluation_reference_sha256': common_reference_sha256,
        'calibration_sha256': calibration_hash(manifest),
        'machine_id': machine_id,
        'excluded_capabilities': contract['excluded_capabilities'],
        'repetitions': {'valid': valid_repetitions, 'failures': failures},
        'trajectory': {
            'ape_rmse_median_m': float(aggregate['ape_rmse_median_m'])},
        'runtime': runtime,
        'mapping': mapping,
        'provenance': {
            'profile_sha256': sha256(profile_path),
            'machine_artifact_sha256': sha256(machine_path),
            'trajectory_summary_sha256': sha256(trajectory_path),
            'map_summary_sha256': sha256(map_path),
        },
    }
    if closure_identity is not None:
        result['provenance']['rival_source_closure'] = closure_identity
    if resource_rows is not None:
        result['resource_evidence'] = resource_rows
    if execution_rows is not None:
        result['execution_evidence'] = execution_rows
    if processing_path is not None:
        result['provenance']['processing_validation_sha256'] = sha256(
            processing_path)
    if loop_path is not None:
        loop_source = json.loads(loop_path.read_text())
        loop = loop_source.get('loop_closure', loop_source)
        result['loop_closure'] = {
            'verified_false_edges': int(loop['verified_false_edges'])}
        result['provenance']['loop_report_sha256'] = sha256(loop_path)
    if visual_path is not None:
        visual_source = json.loads(visual_path.read_text())
        visual = visual_source.get('visual', visual_source.get(
            'aggregate', visual_source))
        result['visual'] = {
            'heldout_rgb_l2_median': float(visual['heldout_rgb_l2_median']),
            'heldout_rgb_inlier_20': float(visual['heldout_rgb_inlier_20']),
        }
        result['provenance']['visual_report_sha256'] = sha256(visual_path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system', required=True)
    parser.add_argument('--track', required=True)
    parser.add_argument('--input-manifest', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--machine', type=Path, required=True)
    parser.add_argument('--trajectory-summary', type=Path, required=True)
    parser.add_argument('--map-summary', type=Path, required=True)
    parser.add_argument('--processing-validation', type=Path)
    parser.add_argument('--loop-report', type=Path)
    parser.add_argument('--visual-report', type=Path)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    result = compose(
        system=args.system, track=args.track,
        manifest_path=args.input_manifest, reference_path=args.reference,
        machine_path=args.machine, trajectory_path=args.trajectory_summary,
        map_path=args.map_summary, profile_path=args.profile,
        processing_path=args.processing_validation, loop_path=args.loop_report,
        visual_path=args.visual_report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
