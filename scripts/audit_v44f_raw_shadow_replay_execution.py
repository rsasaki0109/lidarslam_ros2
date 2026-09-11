#!/usr/bin/env python3
"""Audit the fail-fast v44 raw-shadow replay execution result."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import resource
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_ID = 'v44f-raw-shadow-replay-execution-audit-20260810'
EXPECTED_CHECKS = (
    'v44e_contract_hash_and_id',
    'v44e_static_aggregate_hash_payload_and_decision',
    'adapter_and_core_hashes',
    'execution_report_hash_size_and_path',
    'diagnostic_hash_size_and_path',
    'authorization_chain_exact',
    'first_attempt_order_exact',
    'raw_replay_was_attempted',
    'resource_failure_is_terminal',
    'no_valid_state_or_state_payload',
    'protected_v17_hashes_unchanged',
    'forbidden_routes_remained_closed',
    'diagnostic_records_are_canonical_and_complete',
    'prefix_drop_precedes_terminal_failure',
    'offending_rtf_value_not_persisted',
    'remaining_five_runs_not_attempted',
    'fail_fast_policy_was_observed',
    'raw_bags_are_not_opened_by_auditor',
)


class ContractError(ValueError):
    """The frozen v44f execution-result contract is not satisfied."""


class MemoryBudgetError(RuntimeError):
    """The bounded audit exceeded its memory or output allowance."""


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


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError) as error:
        raise ContractError(f'cannot read JSONL: {path}') from error
    for index, line in enumerate(lines, 1):
        require(bool(line), f'{path}:{index}: empty JSONL record')
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise ContractError(f'{path}:{index}: malformed JSON') from error
        require(isinstance(record, dict),
                f'{path}:{index}: record is not an object')
        require(line == canonical_json(record),
                f'{path}:{index}: record is not canonical JSON')
        records.append(record)
    return records


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
        'prerequisites', 'execution_evidence', 'expected_failure',
        'fail_fast', 'required_checks', 'audit_resources', 'decision'},
        'v44f top-level key inventory differs')
    require(contract['schema_version'] == 1,
            'unsupported v44f schema_version')
    require(contract['contract_id'] == CONTRACT_ID,
            'v44f contract ID differs')
    require(tuple(contract['required_checks']) == EXPECTED_CHECKS,
            'v44f check inventory or order differs')
    require(int(contract['decision']['required_validation_repetitions']) == 2,
            'v44f requires exactly two audit repetitions')
    require(int(contract['decision']['required_check_count']) ==
            len(EXPECTED_CHECKS), 'v44f check count differs')
    require(contract['decision']['on_valid_audit'] ==
            'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE',
            'v44f decision differs')
    require(contract['decision']['raw_replay_continuation_authorized'] is False,
            'v44f must not authorize replay continuation')
    require(contract['decision']['accuracy_screen_authorized'] is False,
            'v44f must not authorize accuracy')
    require(contract['decision']['primary_mutation_authorized'] is False,
            'v44f must not authorize primary mutation')
    require(contract['decision'][
        'bounded_failure_profile_contract_definition_authorized'] is True,
        'v44f next diagnostic boundary differs')
    fail_fast = contract['fail_fast']
    require(fail_fast['stop_after_first_failure'] is True,
            'v44f fail-fast policy differs')
    require(int(fail_fast['expected_attempted_run_count']) == 1,
            'v44f attempted run count differs')
    require(int(fail_fast['expected_unattempted_run_count']) == 5,
            'v44f unattempted run count differs')


def load_and_validate_contract(
        contract_path: Path) -> tuple[
            dict[str, Any], str, dict[str, Any], dict[str, Any]]:
    contract_path = resolve_path(contract_path)
    contract = load_json(contract_path)
    validate_contract_shape(contract)
    prerequisites = contract['prerequisites']
    v44e_contract = _bound_json(
        prerequisites['v44e_contract'], 'v44e execution contract')
    require(v44e_contract.get('contract_id') == prerequisites[
        'v44e_contract']['required_contract_id'],
        'v44e prerequisite contract ID differs')
    aggregate = _bound_json(
        prerequisites['v44e_static_aggregate'], 'v44e static aggregate')
    require(aggregate.get('status') == 'PASS'
            and aggregate.get('decision') == prerequisites[
                'v44e_static_aggregate']['required_decision'],
            'v44e static authorization decision differs')
    require(aggregate.get('aggregate_payload_sha256') == prerequisites[
        'v44e_static_aggregate']['required_payload_sha256'],
        'v44e static aggregate payload differs')
    require(aggregate.get('raw_shadow_replay_execution_authorized') is True
            and aggregate.get('raw_replay_executed') is False,
            'v44e static aggregate authority differs')
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
            'executing v44f auditor path differs')
    require(sha256_file(Path(__file__).resolve()) == auditor['sha256'],
            'executing v44f auditor SHA-256 differs')
    return contract, sha256_file(contract_path), v44e_contract, aggregate


def validate_failure_payload(
        report: Mapping[str, Any], diagnostics: Sequence[Mapping[str, Any]],
        contract: Mapping[str, Any], v44e_contract: Mapping[str, Any],
        aggregate: Mapping[str, Any]) -> dict[str, bool]:
    checks = {identifier: False for identifier in EXPECTED_CHECKS}

    def passed(identifier: str, condition: bool, message: str) -> None:
        require(identifier in checks, f'unknown v44f check: {identifier}')
        require(condition, message)
        checks[identifier] = True

    expected = contract['expected_failure']
    evidence = contract['execution_evidence']
    passed('v44e_contract_hash_and_id',
           report.get('contract_id') == v44e_contract['contract_id']
           and report.get('contract_sha256') == contract['prerequisites'][
               'v44e_contract']['sha256'],
           'execution report v44e contract binding differs')
    passed('v44e_static_aggregate_hash_payload_and_decision',
           report.get('authorization_aggregate_sha256') ==
           contract['prerequisites']['v44e_static_aggregate']['sha256']
           and report.get('authorization_decision') == aggregate['decision'],
           'execution report authorization chain differs')
    passed('adapter_and_core_hashes',
           report.get('adapter_sha256') ==
           contract['prerequisites']['adapter']['sha256']
           and report.get('core_result', {}).get('architecture_contract_id') ==
           'v44b-fixed-lag-shadow-architecture-20260810',
           'execution implementation binding differs')
    passed('execution_report_hash_size_and_path', True,
           'execution report file binding differs')
    passed('diagnostic_hash_size_and_path', True,
           'execution diagnostics file binding differs')
    passed('authorization_chain_exact',
           report.get('authorization_decision') ==
           'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION',
           'execution authorization decision differs')
    deterministic = report.get('deterministic')
    require(isinstance(deterministic, dict),
            'execution deterministic payload is absent')
    require(report.get('report_payload_sha256') == payload_sha256(deterministic),
            'execution report payload SHA-256 differs')
    passed('first_attempt_order_exact',
           report.get('sequence_id') == expected['sequence_id']
           and int(report.get('repetition', 0)) == int(expected['repetition'])
           and evidence['attempt_order'][0] == {
               'sequence_id': report.get('sequence_id'),
               'repetition': report.get('repetition')},
           'first attempted run order differs')
    passed('raw_replay_was_attempted',
           report.get('raw_replay_executed') is True,
           'raw replay was not recorded as attempted')
    core = report.get('core_result', {})
    passed('resource_failure_is_terminal',
           report.get('status') == 'FAIL'
           and core.get('status') == 'FAIL'
           and core.get('reason') == expected['core_reason']
           and deterministic.get('runtime_error') ==
           expected['runtime_error']
           and float(expected['maximum_processing_rtf']) == float(
               v44e_contract['runtime_resources']['maximum_processing_rtf']),
           'resource failure is not the exact terminal result')
    passed('no_valid_state_or_state_payload',
           core.get('valid_shadow_result') is False
           and int(core.get('active_state_count', -1)) == 0
           and core.get('state_payload_sha256') is None
           and deterministic.get('core_state_payload_sha256') is None,
           'failed execution exposes a valid state or state payload')
    protected_before = deterministic.get('protected_v17_before')
    protected_after = deterministic.get('protected_v17_after')
    passed('protected_v17_hashes_unchanged',
           deterministic.get('protected_v17_unchanged') is True
           and protected_before == protected_after ==
           expected['protected_v17_sha256'],
           'protected v17 hashes changed')
    passed('forbidden_routes_remained_closed',
           report.get('accuracy_or_reference_map_inputs_accessed') is False
           and report.get('primary_trajectory_or_map_mutated') is False
           and report.get('ros_output_published') is False,
           'a forbidden execution route was opened')
    passed('diagnostic_records_are_canonical_and_complete',
           len(diagnostics) == 2
           and core.get('diagnostic_record_count') == 2
           and core.get('diagnostic_payload_sha256') ==
           payload_sha256(list(diagnostics)),
           'execution diagnostic payload differs')
    first, terminal = diagnostics
    passed('prefix_drop_precedes_terminal_failure',
           first.get('record_type') == 'scan'
           and int(first.get('scan_index', -1)) == int(
               expected['failed_after_scan_index'])
           and first.get('status') == 'dropped_unbracketed_prefix'
           and first.get('reason') == 'IMU interval is not bracketed'
           and terminal == {
               'reason': expected['core_reason'],
               'record_type': 'terminal', 'status': 'FAIL',
               'valid_shadow_result': False, 'valid_state_count': 0},
           'prefix/terminal diagnostic sequence differs')
    passed('offending_rtf_value_not_persisted',
           expected['offending_rtf_numeric_value_persisted'] is False
           and float(first.get('processing_rtf', -1.0)) == 0.0
           and report.get('stream') is None,
           'offending RTF persistence inventory differs')
    passed('remaining_five_runs_not_attempted', True,
           'one or more remaining runs were attempted')
    passed('fail_fast_policy_was_observed',
           contract['fail_fast']['stop_after_first_failure'] is True,
           'fail-fast policy differs')
    passed('raw_bags_are_not_opened_by_auditor', True,
           'v44f auditor opened a raw bag')
    require(all(checks.values()), 'one or more v44f checks failed')
    return checks


def audit_execution(
        contract: Mapping[str, Any], v44e_contract: Mapping[str, Any],
        aggregate: Mapping[str, Any]) -> tuple[
            dict[str, bool], dict[str, Any], list[dict[str, Any]]]:
    evidence = contract['execution_evidence']
    report_binding = evidence['failure_report']
    diagnostic_binding = evidence['failure_diagnostics']
    report_path = resolve_path(report_binding['path'])
    diagnostic_path = resolve_path(diagnostic_binding['path'])
    for binding, path, label in (
            (report_binding, report_path, 'execution report'),
            (diagnostic_binding, diagnostic_path, 'execution diagnostics')):
        require(path.is_file() and not path.is_symlink(),
                f'{label} is absent or symlinked')
        require(path.stat().st_size == int(binding['bytes']),
                f'{label} byte size differs')
        require(sha256_file(path) == binding['sha256'],
                f'{label} SHA-256 differs')
    report = load_json(report_path)
    diagnostics = load_jsonl(diagnostic_path)
    checks = validate_failure_payload(
        report, diagnostics, contract, v44e_contract, aggregate)
    raw_root = resolve_path(evidence['raw_replay_root'])
    observed_entries = sorted(
        str(path.resolve()) for path in raw_root.rglob('*'))
    run_directory = report_path.parent
    expected_entries = sorted([
        str(run_directory.parent), str(run_directory),
        str(report_path), str(diagnostic_path)])
    require(observed_entries == expected_entries,
            'remaining raw replay path inventory differs from fail-fast result')
    expected = contract['expected_failure']
    for name, digest in expected['protected_v17_sha256'].items():
        artifact = next(
            item for item in v44e_contract['source_binding']['sequences'][0][
                'protected_v17_artifacts'] if item['name'] == name)
        path = resolve_path(artifact['path'])
        require(path.stat().st_size == int(artifact['bytes'])
                and sha256_file(path) == digest,
                f'current protected {name} differs after failure')
    return checks, report, diagnostics


def write_json_bounded(path: Path, value: Mapping[str, Any], maximum: int) -> None:
    encoded = (json.dumps(
        dict(value), indent=2, sort_keys=True, allow_nan=False)
        + '\n').encode('utf-8')
    if len(encoded) > int(maximum):
        raise MemoryBudgetError('v44f report exceeds output capacity')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)


def validate_once(
        contract_path: Path, repetition: int, output: Path) -> dict[str, Any]:
    contract, contract_digest, v44e_contract, aggregate = (
        load_and_validate_contract(contract_path))
    required = int(contract['decision']['required_validation_repetitions'])
    require(1 <= int(repetition) <= required,
            'v44f repetition is outside contract')
    resources = contract['audit_resources']
    memory = MemoryGuard(
        resources['maximum_rss_mib'],
        resources['maximum_incremental_rss_mib'])
    memory.check('start')
    checks, execution_report, diagnostics = audit_execution(
        contract, v44e_contract, aggregate)
    deterministic = {
        'execution_report_sha256': contract['execution_evidence'][
            'failure_report']['sha256'],
        'execution_diagnostics_sha256': contract['execution_evidence'][
            'failure_diagnostics']['sha256'],
        'check_results': checks,
        'attempted_run_count': 1,
        'unattempted_run_count': 5,
        'first_failure': {
            'sequence_id': execution_report['sequence_id'],
            'repetition': execution_report['repetition'],
            'reason': execution_report['core_result']['reason'],
            'state_payload_sha256': execution_report[
                'core_result']['state_payload_sha256'],
            'protected_v17_unchanged': execution_report[
                'deterministic']['protected_v17_unchanged'],
            'diagnostic_record_count': len(diagnostics),
        },
        'raw_replay_continuation_authorized': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
    }
    report = {
        'schema_version': 1,
        'audit': 'v44f_raw_shadow_replay_execution_validation',
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
        'raw_replay_continuation_authorized': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
        'bounded_failure_profile_contract_definition_authorized': True,
    }
    memory.check('before_report_write')
    write_json_bounded(output, report, int(resources['maximum_report_bytes']))
    return report


def validate_source_report(
        report: Mapping[str, Any], contract: Mapping[str, Any],
        contract_digest: str) -> None:
    require(report.get('audit') ==
            'v44f_raw_shadow_replay_execution_validation',
            'v44f source report audit ID differs')
    require(report.get('contract_id') == contract['contract_id']
            and report.get('contract_sha256') == contract_digest,
            'v44f source report contract binding differs')
    require(report.get('auditor_sha256') ==
            contract['prerequisites']['auditor']['sha256'],
            'v44f source report auditor hash differs')
    require(report.get('status') == 'PASS'
            and report.get('decision') ==
            contract['decision']['on_valid_audit'],
            'v44f source report decision differs')
    deterministic = report.get('deterministic')
    require(isinstance(deterministic, dict)
            and report.get('report_payload_sha256') ==
            payload_sha256(deterministic),
            'v44f source report payload differs')
    require(set(deterministic.get('check_results', {})) ==
            set(EXPECTED_CHECKS)
            and all(deterministic['check_results'].values()),
            'v44f source report checks differ')
    for key in (
            'raw_replay_continuation_authorized',
            'accuracy_screen_authorized', 'primary_mutation_authorized'):
        require(report.get(key) is False,
                f'v44f source report unexpectedly opens {key}')


def aggregate_reports(
        contract_path: Path, reports: list[Path], output: Path,
        markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_digest, _, _ = load_and_validate_contract(contract_path)
    required = int(contract['decision']['required_validation_repetitions'])
    require(len(reports) == required,
            f'v44f aggregate requires exactly {required} reports')
    loaded = []
    for path_value in reports:
        path = resolve_path(path_value)
        report = load_json(path)
        validate_source_report(report, contract, contract_digest)
        loaded.append((path, report))
    require(sorted(int(item[1]['repetition']) for item in loaded) == [1, 2],
            'v44f repetitions are incomplete or duplicated')
    payloads = {item[1]['report_payload_sha256'] for item in loaded}
    require(len(payloads) == 1, 'v44f execution audit is not repeatable')
    deterministic = {
        'validation_complete': True,
        'validation_repeatable': True,
        'validation_repetition_count': 2,
        'report_payload_sha256': next(iter(payloads)),
        'check_count': len(EXPECTED_CHECKS),
        'attempted_run_count': 1,
        'unattempted_run_count': 5,
        'failure_reason': contract['expected_failure']['core_reason'],
        'raw_replay_continuation_authorized': False,
    }
    aggregate = {
        'schema_version': 1,
        'audit': 'v44f_raw_shadow_replay_execution_aggregate',
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
        'raw_replay_continuation_authorized': False,
        'accuracy_screen_authorized': False,
        'primary_mutation_authorized': False,
        'bounded_failure_profile_contract_definition_authorized': True,
    }
    maximum = int(contract['audit_resources']['maximum_report_bytes'])
    write_json_bounded(output, aggregate, maximum)
    if markdown_output is not None:
        lines = [
            '# v44f raw shadow replay execution audit', '',
            f"- audit status: `{aggregate['status']}`",
            f"- route decision: `{aggregate['decision']}`",
            f"- deterministic report payload: `{next(iter(payloads))}`",
            f"- aggregate payload: `{aggregate['aggregate_payload_sha256']}`",
            '- attempted runs: `1`', '- unattempted runs: `5`',
            f"- terminal reason: `{deterministic['failure_reason']}`",
            '- raw replay continuation: `false`',
            '- accuracy / primary mutation: `false`', '',
        ]
        encoded = '\n'.join(lines).encode('utf-8')
        if len(encoded) > maximum:
            raise MemoryBudgetError('v44f markdown exceeds output capacity')
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
