#!/usr/bin/env python3
"""Validate the definition-only v44g raw-shadow failure profile contract.

This auditor deliberately validates a contract, not a replay.  It hashes the
already sealed v44f rejection chain and checks that a future diagnostic
profile would capture attempted RSS/RTF and fixed decoder/reorder/core timing
before the unchanged v44e resource gate.  It never imports a bag decoder,
opens a raw bag, or invokes the shadow adapter.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import resource
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_ID = 'v44g-raw-shadow-failure-profile-contract-20260813'
EXPECTED_PHASES = ('decoder', 'reorder', 'core')
EXPECTED_RECORD_FIELDS = (
    'sequence_id',
    'repetition',
    'scan_index',
    'record_type',
    'status',
    'sensor_start_ns',
    'sensor_end_ns',
    'sensor_elapsed_ns',
    'phase_wall_ns',
    'cumulative_phase_wall_ns',
    'total_wall_ns',
    'attempted_rss_mib',
    'attempted_processing_rtf',
    'maximum_rss_mib',
    'maximum_processing_rtf',
    'gate_evaluation_order',
    'terminal_reason',
)
EXPECTED_CHECKS = (
    'v44f_audit_hash_and_id',
    'v44f_aggregate_hash_payload_and_decision',
    'v44e_gate_values_exact',
    'implementation_hashes_exact',
    'target_is_first_failure_only',
    'phase_inventory_fixed',
    'phase_clock_is_diagnostic_only',
    'attempted_values_are_required',
    'profile_precedes_unchanged_gate',
    'rtf_formula_is_fixed',
    'bounded_profile_output',
    'no_threshold_relaxation',
    'definition_only_boundary',
    'forbidden_routes_closed',
    'auditor_is_static',
)


class ContractError(ValueError):
    """The v44g definition-only contract is not satisfied."""


class MemoryBudgetError(RuntimeError):
    """The bounded static audit exceeded its resource allowance."""


def require(condition: bool, message: str) -> None:
    if not bool(condition):
        raise ContractError(message)


def canonical_json(value: Any) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def resolve_path(value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ContractError(f'cannot read JSON: {path}') from error
    require(isinstance(value, dict), f'JSON root is not an object: {path}')
    return value


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float,
                 maximum_incremental_rss_mib: float) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.maximum_incremental_rss_mib = float(maximum_incremental_rss_mib)
        self.baseline_rss_mib = current_rss_mib()
        self.peak_rss_mib = self.baseline_rss_mib
        self.absolute_ceiling_enforced = (
            self.baseline_rss_mib <= self.maximum_rss_mib)

    def check(self, label: str) -> None:
        value = current_rss_mib()
        self.peak_rss_mib = max(self.peak_rss_mib, value)
        if self.absolute_ceiling_enforced and value > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS exceeds absolute bound at {label}: {value:.3f} MiB')
        if value - self.baseline_rss_mib > self.maximum_incremental_rss_mib:
            raise MemoryBudgetError(
                f'RSS exceeds incremental bound at {label}: {value:.3f} MiB')


def _bound_json(binding: Mapping[str, Any], label: str) -> dict[str, Any]:
    path = resolve_path(binding['path'])
    require(path.is_file() and not path.is_symlink(),
            f'{label} is absent or symlinked')
    require(path.stat().st_size == int(binding['bytes']),
            f'{label} byte size differs')
    require(sha256_file(path) == binding['sha256'],
            f'{label} SHA-256 differs')
    return load_json(path)


def validate_contract_shape(contract: Mapping[str, Any]) -> None:
    require(set(contract) == {
        'schema_version', 'contract_id', 'stage', 'rationale',
        'prerequisites', 'failure_profile', 'immutable_gate',
        'execution_boundary', 'required_checks', 'audit_resources', 'decision'},
        'v44g top-level key inventory differs')
    require(contract['schema_version'] == 1,
            'unsupported v44g schema_version')
    require(contract['contract_id'] == CONTRACT_ID,
            'v44g contract ID differs')
    require(tuple(contract['required_checks']) == EXPECTED_CHECKS,
            'v44g check inventory or order differs')
    require(int(contract['decision']['required_validation_repetitions']) == 2,
            'v44g requires exactly two validation repetitions')
    require(int(contract['decision']['required_check_count']) ==
            len(EXPECTED_CHECKS), 'v44g check count differs')
    require(contract['decision']['on_valid_audit'] ==
            'AUTHORIZE_V44G_FAILURE_PROFILE_CONTRACT_DEFINITION_ONLY',
            'v44g decision differs')
    require(contract['decision']['failure_profile_execution_authorized'] is False,
            'v44g must not authorize profile execution')
    require(contract['decision']['raw_replay_continuation_authorized'] is False,
            'v44g must not authorize replay continuation')
    require(contract['decision']['accuracy_screen_authorized'] is False,
            'v44g must not authorize accuracy')
    require(contract['decision']['primary_mutation_authorized'] is False,
            'v44g must not authorize primary mutation')
    require(contract['decision']['threshold_relaxation_authorized'] is False,
            'v44g must not authorize threshold relaxation')


def load_and_validate_contract(contract_path: Path) -> tuple[
        dict[str, Any], str, dict[str, Any], dict[str, Any], dict[str, Any]]:
    contract_path = resolve_path(contract_path)
    contract = load_json(contract_path)
    validate_contract_shape(contract)
    prerequisites = contract['prerequisites']
    v44f_audit = _bound_json(prerequisites['v44f_audit'], 'v44f audit')
    v44f_aggregate = _bound_json(
        prerequisites['v44f_aggregate'], 'v44f aggregate')
    v44e_contract = _bound_json(
        prerequisites['v44e_contract'], 'v44e execution contract')
    for key in ('adapter', 'estimator_core'):
        binding = prerequisites[key]
        path = resolve_path(binding['path'])
        require(path.is_file() and not path.is_symlink(),
                f'{key} is absent or symlinked')
        require(path.stat().st_size == int(binding['bytes']),
                f'{key} byte size differs')
        require(sha256_file(path) == binding['sha256'],
                f'{key} SHA-256 differs')
    auditor = prerequisites['auditor']
    require(resolve_path(auditor['path']) == Path(__file__).resolve(),
            'executing v44g auditor path differs')
    require(sha256_file(Path(__file__).resolve()) == auditor['sha256'],
            'executing v44g auditor SHA-256 differs')
    require(v44f_audit.get('contract_id') ==
            prerequisites['v44f_audit']['required_contract_id'],
            'v44f audit contract ID differs')
    require(v44f_aggregate.get('status') == 'PASS'
            and v44f_aggregate.get('decision') == prerequisites[
                'v44f_aggregate']['required_decision'],
            'v44f aggregate decision differs')
    require(v44f_aggregate.get('aggregate_payload_sha256') == prerequisites[
        'v44f_aggregate']['required_payload_sha256'],
        'v44f aggregate payload differs')
    require(v44e_contract.get('contract_id') ==
            prerequisites['v44e_contract']['required_contract_id'],
            'v44e contract ID differs')
    return (
        contract, sha256_file(contract_path), v44f_audit,
        v44f_aggregate, v44e_contract)


def validate_definition(
        contract: Mapping[str, Any], v44f_audit: Mapping[str, Any],
        v44f_aggregate: Mapping[str, Any],
        v44e_contract: Mapping[str, Any]) -> dict[str, bool]:
    checks = {identifier: False for identifier in EXPECTED_CHECKS}

    def passed(identifier: str, condition: bool, message: str) -> None:
        require(identifier in checks, f'unknown v44g check: {identifier}')
        require(condition, message)
        checks[identifier] = True

    prerequisites = contract['prerequisites']
    passed(
        'v44f_audit_hash_and_id',
        v44f_audit.get('contract_id') ==
        prerequisites['v44f_audit']['required_contract_id']
        and v44f_audit.get('decision', {}).get('on_valid_audit') ==
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE',
        'v44f audit authorization or decision differs')
    passed(
        'v44f_aggregate_hash_payload_and_decision',
        v44f_aggregate.get('status') == 'PASS'
        and v44f_aggregate.get('decision') ==
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE'
        and v44f_aggregate.get('bounded_failure_profile_contract_definition_authorized')
        is True
        and v44f_aggregate.get('raw_replay_continuation_authorized') is False,
        'v44f aggregate does not authorize this boundary')

    runtime = v44e_contract['runtime_resources']
    gate = contract['immutable_gate']
    passed(
        'v44e_gate_values_exact',
        float(gate['maximum_rss_mib']) == float(runtime['maximum_rss_mib'])
        and float(gate['maximum_processing_rtf']) == float(
            runtime['maximum_processing_rtf']),
        'v44e resource gate values differ')

    passed(
        'implementation_hashes_exact',
        prerequisites['adapter']['sha256'] ==
        v44f_audit['prerequisites']['adapter']['sha256']
        and prerequisites['estimator_core']['sha256'] ==
        v44f_audit['prerequisites']['estimator_core']['sha256'],
        'v44e adapter or core implementation binding differs')

    profile = contract['failure_profile']
    target = profile['target']
    expected = v44f_audit['expected_failure']
    passed(
        'target_is_first_failure_only',
        target['sequence_id'] == expected['sequence_id']
        and int(target['repetition']) == int(expected['repetition'])
        and int(target['scan_index']) == int(expected['failed_after_scan_index'])
        and int(target['maximum_sequence_count']) == 1
        and int(target['maximum_scan_count']) == 1
        and target['full_replay_allowed'] is False,
        'v44g target is broader than the sealed first failure')

    phases = profile['phase_inventory']
    passed(
        'phase_inventory_fixed',
        tuple(item['id'] for item in phases) == EXPECTED_PHASES
        and tuple(int(item['order']) for item in phases) == (1, 2, 3)
        and all(item['counts_toward_processing_rtf'] is True
                and item['overlap_allowed'] is False for item in phases),
        'decoder/reorder/core phase inventory differs')

    clock = profile['clock']
    passed(
        'phase_clock_is_diagnostic_only',
        clock['source'] == 'monotonic_ns'
        and clock['integer_nanoseconds'] is True
        and clock['estimator_input'] is False
        and clock['wall_clock_used_for_estimator'] is False,
        'failure-profile clock may affect estimator state')

    record = profile['record']
    passed(
        'attempted_values_are_required',
        tuple(record['required_fields']) == EXPECTED_RECORD_FIELDS
        and record['attempted_rss_is_required'] is True
        and record['attempted_processing_rtf_is_required'] is True
        and record['placeholder_rtf_allowed'] is False,
        'failure profile does not persist attempted resource values')

    passed(
        'profile_precedes_unchanged_gate',
        tuple(profile['gate_evaluation_order']) == (
            'capture_failure_profile', 'compare_rss',
            'compare_processing_rtf', 'terminal_fail')
        and profile['profile_capture_precedes_gate'] is True,
        'failure profile is not captured before the gate')

    formula = profile['formula']
    passed(
        'rtf_formula_is_fixed',
        formula['sensor_elapsed_source'] ==
        'current_scan_end_ns_minus_first_seen_header_ns'
        and formula['processing_rtf'] ==
        '(cumulative_decoder_wall_ns + cumulative_reorder_wall_ns + '
        'cumulative_core_wall_ns) / sensor_elapsed_seconds'
        and formula['gate_uses_same_value'] is True,
        'attempted RTF formula differs from v44e')

    passed(
        'bounded_profile_output',
        int(record['maximum_records']) == 4
        and int(record['maximum_bytes']) == 65536
        and record['overwrite_allowed'] is False
        and profile['output']['only_diagnostic_output'] is True,
        'failure profile output is not bounded')

    passed(
        'no_threshold_relaxation',
        gate['comparison_rss'] == 'strictly_greater_is_terminal'
        and gate['comparison_processing_rtf'] ==
        'strictly_greater_is_terminal'
        and contract['decision']['threshold_relaxation_authorized'] is False,
        'failure profile changes the frozen resource gate')

    boundary = contract['execution_boundary']
    passed(
        'definition_only_boundary',
        boundary['mode'] == 'definition_only'
        and boundary['failure_profile_execution_authorized'] is False
        and boundary['raw_bags_may_be_opened'] is False
        and int(boundary['maximum_runs']) == 0
        and int(boundary['maximum_sequences']) == 0
        and int(boundary['maximum_scans']) == 0,
        'v44g accidentally authorizes execution')

    passed(
        'forbidden_routes_closed',
        boundary['accuracy_or_reference_map_inputs_accessed'] is False
        and boundary['primary_trajectory_or_map_mutation'] is False
        and boundary['ros_output_published'] is False
        and boundary['loop_or_global_correction'] is False,
        'v44g opens a forbidden route')

    source = Path(__file__).read_text(encoding='utf-8')
    tree = ast.parse(source)
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(item.name.split('.')[0] for item in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }
    passed(
        'auditor_is_static',
        'rosbags' not in roots
        and 'subprocess' not in roots
        and 'v44_raw_shadow_replay_adapter' not in roots
        and not {'open_bag', 'open_rosbag'}.intersection(called_names),
        'v44g auditor contains a raw replay path')

    require(all(checks.values()), 'one or more v44g checks failed')
    return checks


def write_json_bounded(path: Path, value: Mapping[str, Any], maximum: int) -> None:
    encoded = (json.dumps(
        dict(value), indent=2, sort_keys=True, allow_nan=False)
        + '\n').encode('utf-8')
    if len(encoded) > int(maximum):
        raise MemoryBudgetError('v44g report exceeds output capacity')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)


def validate_once(
        contract_path: Path, repetition: int, output: Path) -> dict[str, Any]:
    contract, contract_digest, v44f_audit, v44f_aggregate, v44e_contract = (
        load_and_validate_contract(contract_path))
    required = int(contract['decision']['required_validation_repetitions'])
    require(1 <= int(repetition) <= required,
            'v44g repetition is outside contract')
    resources = contract['audit_resources']
    memory = MemoryGuard(
        resources['maximum_rss_mib'], resources['maximum_incremental_rss_mib'])
    memory.check('start')
    checks = validate_definition(
        contract, v44f_audit, v44f_aggregate, v44e_contract)
    deterministic = {
        'check_results': checks,
        'phase_order': list(EXPECTED_PHASES),
        'target_sequence_id': contract['failure_profile']['target'][
            'sequence_id'],
        'target_scan_index': contract['failure_profile']['target']['scan_index'],
        'maximum_processing_rtf': contract['immutable_gate'][
            'maximum_processing_rtf'],
        'maximum_rss_mib': contract['immutable_gate']['maximum_rss_mib'],
        'execution_authorized': False,
        'raw_bag_opened': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
    }
    report = {
        'schema_version': 1,
        'audit': 'v44g_raw_shadow_failure_profile_contract_validation',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'auditor_sha256': sha256_file(Path(__file__).resolve()),
        'repetition': int(repetition),
        'status': 'PASS',
        'decision': contract['decision']['on_valid_audit'],
        'deterministic': deterministic,
        'report_payload_sha256': payload_sha256(deterministic),
        'resource_usage': {
            'baseline_rss_mib': memory.baseline_rss_mib,
            'peak_rss_mib': memory.peak_rss_mib,
            'peak_incremental_rss_mib': (
                memory.peak_rss_mib - memory.baseline_rss_mib),
            'absolute_ceiling_enforced': memory.absolute_ceiling_enforced,
        },
        'failure_profile_execution_authorized': False,
        'raw_replay_continuation_authorized': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
        'threshold_relaxation_authorized': False,
    }
    memory.check('before_report_write')
    write_json_bounded(output, report, int(resources['maximum_report_bytes']))
    return report


def validate_source_report(
        report: Mapping[str, Any], contract: Mapping[str, Any],
        contract_digest: str) -> None:
    require(report.get('audit') ==
            'v44g_raw_shadow_failure_profile_contract_validation',
            'v44g source report audit ID differs')
    require(report.get('contract_id') == contract['contract_id']
            and report.get('contract_sha256') == contract_digest,
            'v44g source report contract binding differs')
    require(report.get('auditor_sha256') ==
            contract['prerequisites']['auditor']['sha256'],
            'v44g source report auditor hash differs')
    require(report.get('status') == 'PASS'
            and report.get('decision') ==
            contract['decision']['on_valid_audit'],
            'v44g source report decision differs')
    deterministic = report.get('deterministic')
    require(isinstance(deterministic, dict)
            and report.get('report_payload_sha256') ==
            payload_sha256(deterministic),
            'v44g source report payload differs')
    require(set(deterministic.get('check_results', {})) ==
            set(EXPECTED_CHECKS)
            and all(deterministic['check_results'].values()),
            'v44g source report checks differ')
    for key in (
            'failure_profile_execution_authorized',
            'raw_replay_continuation_authorized',
            'accuracy_screen_authorized',
            'primary_mutation_authorized',
            'threshold_relaxation_authorized'):
        require(report.get(key) is False,
                f'v44g source report unexpectedly opens {key}')


def aggregate_reports(
        contract_path: Path, reports: list[Path], output: Path,
        markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_digest, _, _, _ = load_and_validate_contract(contract_path)
    required = int(contract['decision']['required_validation_repetitions'])
    require(len(reports) == required,
            f'v44g aggregate requires exactly {required} reports')
    loaded = []
    for path_value in reports:
        path = resolve_path(path_value)
        report = load_json(path)
        validate_source_report(report, contract, contract_digest)
        loaded.append((path, report))
    require(sorted(int(item[1]['repetition']) for item in loaded) == [1, 2],
            'v44g repetitions are incomplete or duplicated')
    payloads = {item[1]['report_payload_sha256'] for item in loaded}
    require(len(payloads) == 1, 'v44g contract audit is not repeatable')
    deterministic = {
        'validation_complete': True,
        'validation_repeatable': True,
        'validation_repetition_count': 2,
        'report_payload_sha256': next(iter(payloads)),
        'check_count': len(EXPECTED_CHECKS),
        'phase_order': list(EXPECTED_PHASES),
        'maximum_processing_rtf': contract['immutable_gate'][
            'maximum_processing_rtf'],
        'execution_authorized': False,
        'raw_bag_opened': False,
    }
    aggregate = {
        'schema_version': 1,
        'audit': 'v44g_raw_shadow_failure_profile_contract_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'auditor_sha256': contract['prerequisites']['auditor']['sha256'],
        'status': 'PASS',
        'decision': contract['decision']['on_valid_audit'],
        'deterministic': deterministic,
        'aggregate_payload_sha256': payload_sha256(deterministic),
        'source_reports': [
            {'path': str(path), 'sha256': sha256_file(path)}
            for path, _ in loaded],
        'failure_profile_execution_authorized': False,
        'raw_replay_continuation_authorized': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
        'threshold_relaxation_authorized': False,
    }
    maximum = int(contract['audit_resources']['maximum_report_bytes'])
    write_json_bounded(output, aggregate, maximum)
    if markdown_output is not None:
        lines = [
            '# v44g raw-shadow failure-profile contract audit', '',
            f"- audit status: `{aggregate['status']}`",
            f"- decision: `{aggregate['decision']}`",
            f"- deterministic report payload: `{next(iter(payloads))}`",
            f"- aggregate payload: `{aggregate['aggregate_payload_sha256']}`",
            '- target: `navinst_indoor02`, scan `0` only',
            '- fixed phases: `decoder -> reorder -> core`',
            '- profile execution authorized: `false`',
            '- raw bag opened: `false`',
            '- threshold relaxation: `false`', '',
        ]
        encoded = '\n'.join(lines).encode('utf-8')
        if len(encoded) > maximum:
            raise MemoryBudgetError('v44g markdown exceeds output capacity')
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_bytes(encoded)
    return aggregate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    validate = subparsers.add_parser('validate')
    validate.add_argument('--contract', required=True, type=Path)
    validate.add_argument('--repetition', required=True, type=int)
    validate.add_argument('--output', required=True, type=Path)
    aggregate = subparsers.add_parser('aggregate')
    aggregate.add_argument('--contract', required=True, type=Path)
    aggregate.add_argument('--report', required=True, action='append', type=Path)
    aggregate.add_argument('--output', required=True, type=Path)
    aggregate.add_argument('--markdown-output', type=Path)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    if arguments.command == 'validate':
        report = validate_once(
            arguments.contract, arguments.repetition, arguments.output)
        return 0 if report['status'] == 'PASS' else 2
    aggregate = aggregate_reports(
        arguments.contract, arguments.report, arguments.output,
        arguments.markdown_output)
    return 0 if aggregate['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
