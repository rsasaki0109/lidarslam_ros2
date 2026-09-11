"""Fail-closed tests for the v44d shadow-source static audit."""

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDITOR_PATH = ROOT / 'scripts/audit_v44d_fixed_lag_shadow_source.py'
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v44d_fixed_lag_shadow_source_audit.json')
SOURCE_PATH = ROOT / 'scripts/v44_fixed_lag_shadow_estimator.py'
SPEC = importlib.util.spec_from_file_location('audit_v44d_source', AUDITOR_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def write_contract(tmp_path: Path, value: dict) -> Path:
    path = tmp_path / 'v44d.json'
    path.write_text(json.dumps(value, allow_nan=True), encoding='utf-8')
    return path


def bind_source_copy(tmp_path: Path, value: dict, source: str) -> Path:
    source_path = tmp_path / 'shadow_source.py'
    source_path.write_text(source, encoding='utf-8')
    result = copy.deepcopy(value)
    result['implementation']['path'] = str(source_path)
    result['implementation']['sha256'] = MODULE.sha256_file(source_path)
    result['implementation']['maximum_source_bytes'] = max(
        len(source.encode('utf-8')) + 1,
        int(result['implementation']['maximum_source_bytes']))
    return write_contract(tmp_path, result)


def test_contract_is_exactly_bound_to_v44b_v44c1_and_source():
    value, digest, architecture, synthetic, aggregate, source = (
        MODULE.load_and_validate_contract(CONTRACT_PATH))
    assert len(digest) == 64
    assert value['contract_id'] == 'v44d-fixed-lag-shadow-source-audit-20260810'
    assert architecture['contract_id'] == 'v44b-fixed-lag-shadow-architecture-20260810'
    assert synthetic['contract_id'] == 'v44c1-fixed-lag-synthetic-contracts-20260810'
    assert aggregate['decision'] == (
        'AUTHORIZE_V44_STAGE4_REPORT_ONLY_SHADOW_IMPLEMENTATION')
    assert MODULE.sha256_file(source) == value['implementation']['sha256']


def test_contract_freezes_all_checks_probes_and_closed_authority():
    value = contract()
    assert tuple(value['required_static_checks']) == MODULE.EXPECTED_STATIC_CHECKS
    assert tuple(value['smoke_probe']['required_probe_ids']) == MODULE.EXPECTED_PROBES
    assert len(MODULE.EXPECTED_STATIC_CHECKS) == 36
    assert len(MODULE.EXPECTED_PROBES) == 10
    assert value['decision'][
        'raw_shadow_replay_contract_definition_authorized_on_pass'] is True
    assert value['decision'][
        'raw_shadow_replay_execution_authorized_on_pass'] is False
    assert value['decision'][
        'accuracy_or_reference_map_inputs_authorized_on_pass'] is False
    assert value['decision'][
        'primary_trajectory_or_map_mutation_authorized_on_pass'] is False


def test_validate_report_passes_all_static_and_smoke_checks(tmp_path):
    report = MODULE.validate_once(CONTRACT_PATH, 1, tmp_path / 'run.json')
    assert report['status'] == 'PASS'
    assert report['decision'] == (
        'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_CONTRACT_DEFINITION')
    assert all(report['deterministic']['static_checks'].values())
    assert all(item['status'] == 'PASS'
               for item in report['deterministic']['smoke_probes'].values())
    assert report['deterministic']['raw_runtime_adapter_present'] is False
    assert report['deterministic']['raw_replay_executed'] is False
    assert report['raw_shadow_replay_execution_authorized'] is False


def test_two_reports_have_identical_deterministic_payload_and_aggregate(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    first = MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    second = MODULE.validate_once(CONTRACT_PATH, 2, second_path)
    assert first['report_payload_sha256'] == second['report_payload_sha256']
    aggregate = MODULE.aggregate_reports(
        CONTRACT_PATH, [first_path, second_path], tmp_path / 'aggregate.json',
        tmp_path / 'aggregate.md')
    assert aggregate['status'] == 'PASS'
    assert aggregate['deterministic']['validation_repeatable'] is True
    assert aggregate['deterministic']['static_check_count'] == 36
    assert aggregate['deterministic']['smoke_probe_count'] == 10
    assert aggregate['raw_shadow_replay_contract_definition_authorized'] is True
    assert aggregate['raw_shadow_replay_execution_authorized'] is False
    assert (tmp_path / 'aggregate.md').is_file()


def test_cli_surface_contains_only_static_validate_and_aggregate():
    parser = MODULE.build_parser()
    choices = parser._subparsers._group_actions[0].choices
    assert set(choices) == {'validate', 'aggregate'}
    source = AUDITOR_PATH.read_text(encoding='utf-8')
    for prohibited in (
            "add_argument('--bag", "add_argument('--trajectory",
            "add_argument('--reference-map", "add_argument('--ground-truth",
            'import rosbags', 'import rclpy'):
        assert prohibited not in source


@pytest.mark.parametrize('key', [
    'architecture_contract_sha256',
    'synthetic_contract_sha256',
    'synthetic_aggregate_sha256',
])
def test_prerequisite_hash_mutation_is_rejected(tmp_path, key):
    value = contract()
    value['prerequisite'][key] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='SHA-256 differs'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_source_hash_or_size_mutation_is_rejected(tmp_path):
    value = contract()
    value['implementation']['sha256'] = 'f' * 64
    with pytest.raises(MODULE.ContractError, match='implementation SHA-256'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))
    value = contract()
    value['implementation']['maximum_source_bytes'] = 1
    with pytest.raises(MODULE.ContractError, match='source byte capacity'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_relaxed_execution_authority_is_rejected(tmp_path):
    value = contract()
    value['decision']['raw_shadow_replay_execution_authorized_on_pass'] = True
    with pytest.raises(MODULE.ContractError, match='must keep'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_missing_or_reordered_static_check_is_rejected(tmp_path):
    value = contract()
    value['required_static_checks'] = value['required_static_checks'][::-1]
    with pytest.raises(MODULE.ContractError, match='inventory or order'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_relaxed_resource_bound_or_import_overlap_is_rejected(tmp_path):
    value = contract()
    value['audit_resource_bounds']['maximum_report_bytes'] = 0
    with pytest.raises(MODULE.ContractError, match='must be positive'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))
    value = contract()
    value['allowed_import_roots'].append('pathlib')
    with pytest.raises(MODULE.ContractError, match='overlap'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_forbidden_import_in_hash_bound_source_is_rejected_statically(tmp_path):
    path = bind_source_copy(
        tmp_path, contract(), SOURCE_PATH.read_text(encoding='utf-8')
        + '\nimport pathlib\n')
    with pytest.raises(MODULE.ContractError, match='outside allowlist'):
        MODULE.validate_once(path, 1, tmp_path / 'report.json')


def test_top_level_effect_in_hash_bound_source_is_rejected_before_import(tmp_path):
    path = bind_source_copy(
        tmp_path, contract(), SOURCE_PATH.read_text(encoding='utf-8')
        + "\nprint('forbidden')\n")
    with pytest.raises(MODULE.ContractError, match='top-level runtime side effect'):
        MODULE.validate_once(path, 1, tmp_path / 'report.json')


def test_forbidden_builtin_call_in_dead_code_is_still_rejected(tmp_path):
    path = bind_source_copy(
        tmp_path, contract(), SOURCE_PATH.read_text(encoding='utf-8')
        + "\ndef forbidden_reader():\n    return open('x')\n")
    with pytest.raises(MODULE.ContractError, match='forbidden builtins'):
        MODULE.validate_once(path, 1, tmp_path / 'report.json')


def test_dataset_specific_symbol_is_rejected_even_without_execution(tmp_path):
    path = bind_source_copy(
        tmp_path, contract(), SOURCE_PATH.read_text(encoding='utf-8')
        + '\ndef oxford_branch():\n    return False\n')
    with pytest.raises(MODULE.ContractError, match='dataset-specific symbol'):
        MODULE.validate_once(path, 1, tmp_path / 'report.json')


def test_public_input_field_expansion_is_rejected(tmp_path):
    source = SOURCE_PATH.read_text(encoding='utf-8').replace(
        '    serialized_size_bytes: int\n\n    def __post_init__(self) -> None:',
        '    serialized_size_bytes: int\n'
        '    orientation: tuple[float, float, float, float]\n\n'
        '    def __post_init__(self) -> None:', 1)
    path = bind_source_copy(tmp_path, contract(), source)
    with pytest.raises(MODULE.ContractError, match='dataclass fields differ'):
        MODULE.validate_once(path, 1, tmp_path / 'report.json')


def test_state_dimension_factor_order_or_authority_mutation_is_rejected(tmp_path):
    replacements = [
        ('STATE_DOF = 15', 'STATE_DOF = 16', 'required implementation constant'),
        ("'gauge': 0,", "'gauge': 4,", 'required implementation constant'),
        ("'raw_shadow_replay': False", "'raw_shadow_replay': True",
         'required implementation constant'),
    ]
    for index, (old, new, message) in enumerate(replacements):
        source = SOURCE_PATH.read_text(encoding='utf-8').replace(old, new, 1)
        local = tmp_path / str(index)
        local.mkdir()
        path = bind_source_copy(local, contract(), source)
        with pytest.raises(MODULE.ContractError, match=message):
            MODULE.validate_once(path, 1, local / 'report.json')


def test_missing_required_method_or_diagnostic_field_is_rejected(tmp_path):
    source = SOURCE_PATH.read_text(encoding='utf-8').replace(
        '    def _marginalize_oldest(self) -> None:',
        '    def _retired_marginalize_oldest(self) -> None:', 1)
    first = tmp_path / 'method'
    first.mkdir()
    path = bind_source_copy(first, contract(), source)
    with pytest.raises(MODULE.ContractError, match='required estimator method'):
        MODULE.validate_once(path, 1, first / 'report.json')
    source = SOURCE_PATH.read_text(encoding='utf-8').replace(
        "'lidar_observable_rank': int(observable_rank),",
        "'removed_observable_rank': int(observable_rank),", 1)
    second = tmp_path / 'diagnostic'
    second.mkdir()
    path = bind_source_copy(second, contract(), source)
    with pytest.raises(MODULE.ContractError, match='diagnostic field'):
        MODULE.validate_once(path, 1, second / 'report.json')


def test_aggregate_rejects_duplicate_repetition(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    MODULE.validate_once(CONTRACT_PATH, 1, second_path)
    with pytest.raises(MODULE.ContractError, match='incomplete or duplicated'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first_path, second_path], tmp_path / 'aggregate.json')


def test_aggregate_rejects_tampered_or_nonrepeatable_payload(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    second = MODULE.validate_once(CONTRACT_PATH, 2, second_path)
    second['deterministic']['static_metrics']['ast_node_count'] += 1
    second['report_payload_sha256'] = MODULE.payload_sha256(second['deterministic'])
    second_path.write_text(json.dumps(second), encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='not deterministic'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first_path, second_path], tmp_path / 'aggregate.json')


def test_report_payload_tampering_without_rehash_is_rejected(tmp_path):
    first_path = tmp_path / 'run_01.json'
    second_path = tmp_path / 'run_02.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first_path)
    second = MODULE.validate_once(CONTRACT_PATH, 2, second_path)
    second['deterministic']['raw_replay_executed'] = True
    second_path.write_text(json.dumps(second), encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='payload hash differs'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first_path, second_path], tmp_path / 'aggregate.json')


def test_preloaded_host_uses_incremental_rss_without_relaxing_standalone_ceiling(
        monkeypatch):
    values = iter((200.0, 220.0, 270.0))
    monkeypatch.setattr(MODULE, 'current_rss_mib', lambda: next(values))
    guard = MODULE.MemoryGuard(160.0, 64.0)
    assert guard.absolute_enforced is False
    guard.check('within_increment')
    with pytest.raises(MODULE.MemoryBudgetError, match='incremental RSS'):
        guard.check('over_increment')
