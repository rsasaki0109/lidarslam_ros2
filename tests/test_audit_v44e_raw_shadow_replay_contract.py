"""Fail-closed tests for the v44e execution-contract static gate."""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDITOR_PATH = ROOT / 'scripts/audit_v44e_raw_shadow_replay_contract.py'
ADAPTER_PATH = ROOT / 'scripts/v44_raw_shadow_replay_adapter.py'
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/'
    'v44e_raw_shadow_replay_execution_contract.json')
SPEC = importlib.util.spec_from_file_location('audit_v44e_contract_test', AUDITOR_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


SEALED_WORKSPACE = (
    Path(contract()['execution']['required_contract_path']).resolve()
    == CONTRACT_PATH.resolve()
)
SEALED_TEST_PREFIXES = (
    'test_contract_is_bound_',
    'test_validate_once_',
    'test_two_static_reports_',
    'test_source_manifest_',
    'test_readiness_digest_',
    'test_protected_v17_',
    'test_source_report_',
    'test_aggregate_rejects_',
)


@pytest.fixture(autouse=True)
def require_sealed_workspace(request):
    if (not SEALED_WORKSPACE
            and request.node.name.startswith(SEALED_TEST_PREFIXES)):
        pytest.skip('requires the original hash-sealed v44e workspace')


def test_contract_is_bound_to_v44a_v44b_v44d_core_adapter_and_auditor():
    value, digest, manifest, aggregate, architecture = (
        MODULE.load_and_validate_contract(CONTRACT_PATH))
    assert len(digest) == 64
    assert value['contract_id'] == (
        'v44e-raw-shadow-replay-execution-20260810')
    assert manifest['source_set_id'] == (
        'sota-v6-v44-exact-raw-lidar-imu-20260810')
    assert aggregate['status'] == 'PASS'
    assert architecture['contract_id'] == (
        'v44b-fixed-lag-shadow-architecture-20260810')
    assert MODULE.sha256_file(ADAPTER_PATH) == value['adapter']['sha256']
    assert MODULE.sha256_file(AUDITOR_PATH) == value['static_auditor']['sha256']


def test_contract_freezes_checks_probes_resources_and_authority():
    value = contract()
    assert tuple(value['required_static_checks']) == MODULE.EXPECTED_STATIC_CHECKS
    assert tuple(value['smoke_probe']['required_probe_ids']) == (
        MODULE.EXPECTED_PROBES)
    assert len(MODULE.EXPECTED_STATIC_CHECKS) == 29
    assert len(MODULE.EXPECTED_PROBES) == 10
    assert value['runtime_resources']['maximum_rss_mib'] == 330.0
    assert value['runtime_resources']['maximum_processing_rtf'] == 0.85
    assert value['runtime_resources']['check_after_every_scan'] is True
    assert value['decision'][
        'raw_shadow_replay_execution_authorized_on_pass'] is True
    assert value['decision']['raw_replay_executed_by_static_gate'] is False
    assert value['decision'][
        'accuracy_or_reference_map_inputs_authorized_on_pass'] is False
    assert value['decision'][
        'primary_trajectory_or_map_mutation_authorized_on_pass'] is False
    assert value['decision']['ros_publication_authorized_on_pass'] is False


def test_validate_once_passes_every_check_without_opening_a_bag(tmp_path):
    report = MODULE.validate_once(CONTRACT_PATH, 1, tmp_path / 'run.json')
    assert report['status'] == 'PASS'
    assert report['decision'] == (
        'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION')
    assert all(report['deterministic']['static_checks'].values())
    assert all(item['status'] == 'PASS'
               for item in report['deterministic']['smoke_probes'].values())
    assert report['deterministic']['raw_bag_opened'] is False
    assert report['deterministic']['raw_replay_executed'] is False
    assert report['raw_shadow_replay_execution_authorized'] is True
    assert report['accuracy_or_reference_map_inputs_authorized'] is False


def test_two_static_reports_are_repeatable_and_aggregate(tmp_path):
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
    assert aggregate['deterministic']['static_check_count'] == 29
    assert aggregate['deterministic']['smoke_probe_count'] == 10
    assert aggregate['deterministic']['raw_bag_opened'] is False
    assert aggregate['raw_shadow_replay_execution_authorized'] is True
    assert aggregate['raw_replay_executed'] is False
    assert (tmp_path / 'aggregate.md').is_file()


def test_static_auditor_has_no_raw_bag_dependency_or_runtime_command():
    tree = ast.parse(AUDITOR_PATH.read_text(encoding='utf-8'))
    assert 'rosbags' not in MODULE.imported_roots(tree)
    parser = MODULE.build_parser()
    choices = parser._subparsers._group_actions[0].choices
    assert set(choices) == {'validate', 'aggregate'}
    source = AUDITOR_PATH.read_text(encoding='utf-8')
    assert "add_parser('replay')" not in source
    assert "add_argument('--bag" not in source


@pytest.mark.parametrize('mutation,match', [
    (lambda value: value['required_static_checks'].reverse(),
     'check inventory or order'),
    (lambda value: value['smoke_probe']['required_probe_ids'].pop(),
     'probe inventory or order'),
    (lambda value: value['decision'].__setitem__(
        'raw_replay_executed_by_static_gate', True),
     'must not execute'),
    (lambda value: value['decision'].__setitem__(
        'accuracy_or_reference_map_inputs_authorized_on_pass', True),
     'must keep'),
    (lambda value: value['runtime_resources'].__setitem__(
        'maximum_rss_mib', 331.0), '330 MiB'),
    (lambda value: value['runtime_resources'].__setitem__(
        'maximum_processing_rtf', 0.9), '0.85'),
    (lambda value: value['output'].__setitem__(
        'overwrite_allowed', True), 'overwrite'),
    (lambda value: value['execution'].__setitem__(
        'raw_bag_open_allowed_only_after_authorization', False),
     'authorize before'),
])
def test_relaxed_contract_shape_is_rejected(mutation, match):
    value = contract()
    mutation(value)
    with pytest.raises(MODULE.ContractError, match=match):
        MODULE.validate_contract_shape(value)


def test_source_manifest_or_bag_binding_mutation_is_rejected():
    value, _, manifest, aggregate, _ = MODULE.load_and_validate_contract(
        CONTRACT_PATH)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0]['lidar_topic'] = '/wrong'
    with pytest.raises(MODULE.ContractError, match='lidar_topic differs'):
        MODULE.validate_sources(changed, manifest, aggregate)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0]['bag']['sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='raw bag binding differs'):
        MODULE.validate_sources(changed, manifest, aggregate)


def test_readiness_digest_delay_and_calibration_mutations_are_rejected():
    value, _, manifest, aggregate, _ = MODULE.load_and_validate_contract(
        CONTRACT_PATH)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0][
        'serialized_stream_sha256']['imu'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='stream digest differs'):
        MODULE.validate_sources(changed, manifest, aggregate)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0][
        'maximum_receive_minus_header_ns'] += 1
    with pytest.raises(MODULE.ContractError, match='watermark differs'):
        MODULE.validate_sources(changed, manifest, aggregate)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0]['calibration'][
        'body_from_lidar']['translation_m'][2] += 0.1
    with pytest.raises(MODULE.ContractError, match='translation differs'):
        MODULE.validate_sources(changed, manifest, aggregate)


def test_protected_v17_hash_mutation_is_rejected():
    value, _, manifest, aggregate, _ = MODULE.load_and_validate_contract(
        CONTRACT_PATH)
    changed = copy.deepcopy(value)
    changed['source_binding']['sequences'][0]['protected_v17_artifacts'][0][
        'sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='v17_state SHA-256 differs'):
        MODULE.validate_sources(changed, manifest, aggregate)


def test_adapter_ast_has_lazy_decoder_and_no_forbidden_imports():
    value = contract()
    tree = ast.parse(ADAPTER_PATH.read_text(encoding='utf-8'))
    roots = MODULE.imported_roots(tree)
    assert roots <= set(value['adapter']['allowed_import_roots'])
    rosbags = [node for node in ast.walk(tree)
               if isinstance(node, ast.ImportFrom)
               and node.module and node.module.startswith('rosbags')]
    assert len(rosbags) == 1
    assert rosbags[0] not in tree.body
    attributes = {node.attr for node in ast.walk(tree)
                  if isinstance(node, ast.Attribute)}
    assert attributes.isdisjoint({
        'orientation', 'orientation_covariance',
        'angular_velocity_covariance', 'linear_acceleration_covariance',
        'publish', 'create_publisher'})


def test_source_report_payload_or_authority_mutation_is_rejected(tmp_path):
    report = MODULE.validate_once(CONTRACT_PATH, 1, tmp_path / 'valid.json')
    value, digest, _, _, _ = MODULE.load_and_validate_contract(CONTRACT_PATH)
    changed = copy.deepcopy(report)
    changed['deterministic']['raw_bag_opened'] = True
    with pytest.raises(MODULE.ContractError, match='payload differs'):
        MODULE.validate_source_report(changed, value, digest)
    changed = copy.deepcopy(report)
    changed['raw_shadow_replay_execution_authorized'] = False
    with pytest.raises(MODULE.ContractError, match='did not authorize'):
        MODULE.validate_source_report(changed, value, digest)
    changed = copy.deepcopy(report)
    changed['accuracy_or_reference_map_inputs_authorized'] = True
    with pytest.raises(MODULE.ContractError, match='unexpectedly opens'):
        MODULE.validate_source_report(changed, value, digest)


def test_aggregate_rejects_duplicate_repetition(tmp_path):
    first = tmp_path / 'first.json'
    duplicate = tmp_path / 'duplicate.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first)
    MODULE.validate_once(CONTRACT_PATH, 1, duplicate)
    with pytest.raises(MODULE.ContractError, match='incomplete or duplicated'):
        MODULE.aggregate_reports(
            CONTRACT_PATH, [first, duplicate], tmp_path / 'aggregate.json')
