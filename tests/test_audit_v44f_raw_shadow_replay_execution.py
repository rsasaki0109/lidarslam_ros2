"""Fail-closed tests for the v44f raw execution-result audit."""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDITOR_PATH = ROOT / 'scripts/audit_v44f_raw_shadow_replay_execution.py'
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/'
    'v44f_raw_shadow_replay_execution_audit.json')
SPEC = importlib.util.spec_from_file_location('audit_v44f_execution_test', AUDITOR_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


SEALED_WORKSPACE = (
    Path(contract()['prerequisites']['auditor']['path']).resolve()
    == AUDITOR_PATH.resolve()
)
SEALED_TEST_PREFIXES = (
    'test_contract_binds_',
    'test_execution_evidence_',
    'test_validate_once_',
    'test_two_audits_',
    'test_nonterminal_',
    'test_protected_hash_',
    'test_diagnostic_order_',
    'test_report_or_diagnostic_',
    'test_aggregate_rejects_',
)


@pytest.fixture(autouse=True)
def require_sealed_workspace(request):
    if (not SEALED_WORKSPACE
            and request.node.name.startswith(SEALED_TEST_PREFIXES)):
        pytest.skip('requires the original hash-sealed v44f workspace')


def context():
    return MODULE.load_and_validate_contract(CONTRACT_PATH)


def evidence():
    value, _, v44e, aggregate = context()
    report = MODULE.load_json(MODULE.resolve_path(
        value['execution_evidence']['failure_report']['path']))
    diagnostics = MODULE.load_jsonl(MODULE.resolve_path(
        value['execution_evidence']['failure_diagnostics']['path']))
    return value, v44e, aggregate, report, diagnostics


def test_contract_binds_v44e_authorization_implementation_and_failure():
    value, digest, v44e, aggregate = context()
    assert len(digest) == 64
    assert value['contract_id'] == MODULE.CONTRACT_ID
    assert v44e['contract_id'] == (
        'v44e-raw-shadow-replay-execution-20260810')
    assert aggregate['decision'] == (
        'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION')
    assert aggregate['raw_replay_executed'] is False
    assert MODULE.sha256_file(AUDITOR_PATH) == (
        value['prerequisites']['auditor']['sha256'])


def test_contract_freezes_fail_fast_rejection_and_next_boundary():
    value = contract()
    assert tuple(value['required_checks']) == MODULE.EXPECTED_CHECKS
    assert len(MODULE.EXPECTED_CHECKS) == 18
    assert value['fail_fast']['stop_after_first_failure'] is True
    assert value['fail_fast']['expected_attempted_run_count'] == 1
    assert value['fail_fast']['expected_unattempted_run_count'] == 5
    assert value['fail_fast']['threshold_relaxation_or_rerun_allowed'] is False
    assert value['decision']['on_valid_audit'] == (
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE')
    assert value['decision']['raw_replay_continuation_authorized'] is False
    assert value['decision']['accuracy_screen_authorized'] is False
    assert value['decision'][
        'bounded_failure_profile_contract_definition_authorized'] is True


def test_execution_evidence_passes_all_checks():
    value, v44e, aggregate, report, diagnostics = evidence()
    checks = MODULE.validate_failure_payload(
        report, diagnostics, value, v44e, aggregate)
    assert tuple(checks) == MODULE.EXPECTED_CHECKS
    assert all(checks.values())
    checked, observed_report, observed_diagnostics = MODULE.audit_execution(
        value, v44e, aggregate)
    assert checked == checks
    assert observed_report['status'] == 'FAIL'
    assert len(observed_diagnostics) == 2


def test_validate_once_passes_audit_but_rejects_route(tmp_path):
    report = MODULE.validate_once(CONTRACT_PATH, 1, tmp_path / 'run.json')
    assert report['status'] == 'PASS'
    assert report['decision'] == (
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE')
    assert report['deterministic']['attempted_run_count'] == 1
    assert report['deterministic']['unattempted_run_count'] == 5
    assert report['deterministic']['first_failure']['state_payload_sha256'] is None
    assert report['raw_replay_continuation_authorized'] is False
    assert report['accuracy_screen_authorized'] is False


def test_two_audits_are_repeatable_and_aggregate_rejection(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    first = MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    second = MODULE.validate_once(CONTRACT_PATH, 2, second_path)
    assert first['report_payload_sha256'] == second['report_payload_sha256']
    aggregate = MODULE.aggregate_reports(
        CONTRACT_PATH, [first_path, second_path], tmp_path / 'aggregate.json',
        tmp_path / 'aggregate.md')
    assert aggregate['status'] == 'PASS'
    assert aggregate['decision'] == (
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE')
    assert aggregate['deterministic']['validation_repeatable'] is True
    assert aggregate['raw_replay_continuation_authorized'] is False
    assert aggregate['bounded_failure_profile_contract_definition_authorized'] is True
    assert (tmp_path / 'aggregate.md').is_file()


@pytest.mark.parametrize('mutation,match', [
    (lambda value: value['required_checks'].reverse(),
     'check inventory or order'),
    (lambda value: value['decision'].__setitem__(
        'raw_replay_continuation_authorized', True), 'must not authorize'),
    (lambda value: value['decision'].__setitem__(
        'accuracy_screen_authorized', True), 'must not authorize accuracy'),
    (lambda value: value['fail_fast'].__setitem__(
        'stop_after_first_failure', False), 'fail-fast policy'),
    (lambda value: value['fail_fast'].__setitem__(
        'expected_attempted_run_count', 6), 'attempted run count'),
])
def test_contract_relaxations_are_rejected(mutation, match):
    value = contract()
    mutation(value)
    with pytest.raises(MODULE.ContractError, match=match):
        MODULE.validate_contract_shape(value)


def test_nonterminal_or_valid_state_failure_payload_is_rejected():
    value, v44e, aggregate, report, diagnostics = evidence()
    changed = copy.deepcopy(report)
    changed['core_result']['status'] = 'PASS'
    with pytest.raises(MODULE.ContractError, match='terminal result'):
        MODULE.validate_failure_payload(
            changed, diagnostics, value, v44e, aggregate)
    changed = copy.deepcopy(report)
    changed['core_result']['state_payload_sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='valid state'):
        MODULE.validate_failure_payload(
            changed, diagnostics, value, v44e, aggregate)


def test_protected_hash_or_forbidden_route_mutation_is_rejected():
    value, v44e, aggregate, report, diagnostics = evidence()
    changed = copy.deepcopy(report)
    changed['deterministic']['protected_v17_after']['v17_state'] = '0' * 64
    changed['report_payload_sha256'] = MODULE.payload_sha256(
        changed['deterministic'])
    with pytest.raises(MODULE.ContractError, match='protected v17'):
        MODULE.validate_failure_payload(
            changed, diagnostics, value, v44e, aggregate)
    changed = copy.deepcopy(report)
    changed['ros_output_published'] = True
    with pytest.raises(MODULE.ContractError, match='forbidden'):
        MODULE.validate_failure_payload(
            changed, diagnostics, value, v44e, aggregate)


def test_diagnostic_order_or_persisted_rtf_mutation_is_rejected():
    value, v44e, aggregate, report, diagnostics = evidence()
    changed = copy.deepcopy(diagnostics)
    changed.reverse()
    changed_report = copy.deepcopy(report)
    changed_report['core_result']['diagnostic_payload_sha256'] = (
        MODULE.payload_sha256(changed))
    with pytest.raises(MODULE.ContractError, match='prefix/terminal'):
        MODULE.validate_failure_payload(
            changed_report, changed, value, v44e, aggregate)
    changed = copy.deepcopy(diagnostics)
    changed[0]['processing_rtf'] = 1.2
    changed_report = copy.deepcopy(report)
    changed_report['core_result']['diagnostic_payload_sha256'] = (
        MODULE.payload_sha256(changed))
    with pytest.raises(MODULE.ContractError, match='RTF persistence'):
        MODULE.validate_failure_payload(
            changed_report, changed, value, v44e, aggregate)


def test_report_or_diagnostic_hash_binding_mutation_is_rejected(tmp_path):
    value = contract()
    report_binding = value['execution_evidence']['failure_report']
    original = report_binding['sha256']
    report_binding['sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='execution report SHA-256'):
        _, _, v44e, aggregate = context()
        MODULE.audit_execution(value, v44e, aggregate)
    report_binding['sha256'] = original
    value['execution_evidence']['failure_diagnostics']['bytes'] += 1
    with pytest.raises(MODULE.ContractError, match='diagnostics byte size'):
        _, _, v44e, aggregate = context()
        MODULE.audit_execution(value, v44e, aggregate)


def test_auditor_has_no_raw_decoder_or_replay_command():
    source = AUDITOR_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(item.name.split('.')[0] for item in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert 'rosbags' not in roots
    assert "add_parser('replay')" not in source
    parser = MODULE.build_parser()
    assert set(parser._subparsers._group_actions[0].choices) == {
        'validate', 'aggregate'}


def test_aggregate_rejects_duplicate_repetition(tmp_path):
    first = tmp_path / 'first.json'
    duplicate = tmp_path / 'duplicate.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first)
    MODULE.validate_once(CONTRACT_PATH, 1, duplicate)
    with pytest.raises(MODULE.ContractError, match='incomplete or duplicated'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first, duplicate], tmp_path / 'aggregate.json')
