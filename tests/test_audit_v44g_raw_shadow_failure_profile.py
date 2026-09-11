"""Fail-closed tests for the definition-only v44g profile contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDITOR_PATH = ROOT / 'scripts' / 'audit_v44g_raw_shadow_failure_profile.py'
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/'
    'v44g_raw_shadow_failure_profile_contract.json')
SPEC = importlib.util.spec_from_file_location(
    'audit_v44g_raw_shadow_failure_profile_test', AUDITOR_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def contract() -> dict:
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


SEALED_WORKSPACE = (
    Path(contract()['prerequisites']['auditor']['path']).resolve()
    == AUDITOR_PATH.resolve()
)
SEALED_TEST_PREFIXES = (
    'test_contract_binds_',
    'test_definition_audit_',
    'test_validate_once_',
    'test_contract_relaxations_',
    'test_aggregate_rejects_',
)


@pytest.fixture(autouse=True)
def require_sealed_workspace(request):
    if (not SEALED_WORKSPACE
            and request.node.name.startswith(SEALED_TEST_PREFIXES)):
        pytest.skip('requires the original hash-sealed v44g workspace')


def context():
    return MODULE.load_and_validate_contract(CONTRACT_PATH)


def test_contract_binds_sealed_v44f_rejection_and_unchanged_gate():
    value, digest, v44f, aggregate, v44e = context()

    assert len(digest) == 64
    assert value['contract_id'] == MODULE.CONTRACT_ID
    assert v44f['contract_id'] == (
        'v44f-raw-shadow-replay-execution-audit-20260810')
    assert aggregate['decision'] == (
        'REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE')
    assert aggregate['bounded_failure_profile_contract_definition_authorized']
    assert v44e['runtime_resources']['maximum_processing_rtf'] == 0.85
    assert v44e['runtime_resources']['maximum_rss_mib'] == 330.0


def test_definition_audit_passes_all_checks():
    value, _, v44f, aggregate, v44e = context()
    checks = MODULE.validate_definition(value, v44f, aggregate, v44e)
    assert tuple(checks) == MODULE.EXPECTED_CHECKS
    assert all(checks.values())


def test_validate_once_and_aggregate_are_repeatable(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    first = MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    second = MODULE.validate_once(CONTRACT_PATH, 2, second_path)

    assert first['status'] == 'PASS'
    assert first['decision'] == (
        'AUTHORIZE_V44G_FAILURE_PROFILE_CONTRACT_DEFINITION_ONLY')
    assert first['report_payload_sha256'] == second['report_payload_sha256']
    aggregate = MODULE.aggregate_reports(
        CONTRACT_PATH,
        [first_path, second_path],
        tmp_path / 'aggregate.json',
        tmp_path / 'aggregate.md',
    )
    assert aggregate['status'] == 'PASS'
    assert aggregate['deterministic']['validation_repeatable'] is True
    assert aggregate['deterministic']['raw_bag_opened'] is False
    assert (tmp_path / 'aggregate.md').is_file()


@pytest.mark.parametrize(
    'mutation,match',
    [
        (
            lambda value: value['execution_boundary'].__setitem__(
                'raw_bags_may_be_opened', True),
            'accidentally authorizes execution',
        ),
        (
            lambda value: value['failure_profile']['record'].__setitem__(
                'placeholder_rtf_allowed', True),
            'does not persist attempted resource values',
        ),
        (
            lambda value: value['failure_profile']['target'].__setitem__(
                'maximum_scan_count', 6),
            'broader than the sealed first failure',
        ),
        (
            lambda value: value['immutable_gate'].__setitem__(
                'maximum_processing_rtf', 0.9),
            'resource gate values differ',
        ),
        (
            lambda value: value['failure_profile']['formula'].__setitem__(
                'gate_uses_same_value', False),
            'attempted RTF formula differs',
        ),
    ],
)
def test_contract_relaxations_are_rejected(mutation, match):
    value = contract()
    mutation(value)
    _, _, v44f, aggregate, v44e = context()
    with pytest.raises(MODULE.ContractError, match=match):
        MODULE.validate_definition(value, v44f, aggregate, v44e)


def test_definition_only_contract_cannot_authorize_threshold_or_replay():
    value = contract()
    value['decision']['failure_profile_execution_authorized'] = True
    with pytest.raises(MODULE.ContractError, match='profile execution'):
        MODULE.validate_contract_shape(value)

    value = contract()
    value['decision']['threshold_relaxation_authorized'] = True
    with pytest.raises(MODULE.ContractError, match='threshold relaxation'):
        MODULE.validate_contract_shape(value)


def test_auditor_has_no_raw_decoder_or_replay_import():
    source = AUDITOR_PATH.read_text(encoding='utf-8')
    tree = MODULE.ast.parse(source)
    roots = set()
    for node in MODULE.ast.walk(tree):
        if isinstance(node, MODULE.ast.Import):
            roots.update(item.name.split('.')[0] for item in node.names)
        elif isinstance(node, MODULE.ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert 'rosbags' not in roots
    assert 'subprocess' not in roots
    assert "add_parser('replay')" not in source


def test_aggregate_rejects_duplicate_repetition(tmp_path):
    first = tmp_path / 'first.json'
    duplicate = tmp_path / 'duplicate.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first)
    MODULE.validate_once(CONTRACT_PATH, 1, duplicate)
    with pytest.raises(MODULE.ContractError, match='incomplete or duplicated'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first, duplicate], tmp_path / 'aggregate.json')


def test_contract_is_path_and_output_bounded():
    value = contract()
    assert value['failure_profile']['target']['full_replay_allowed'] is False
    assert value['execution_boundary']['maximum_runs'] == 0
    assert value['execution_boundary']['maximum_scans'] == 0
    assert value['failure_profile']['record']['maximum_bytes'] == 65536
    assert value['failure_profile']['output']['only_diagnostic_output'] is True
