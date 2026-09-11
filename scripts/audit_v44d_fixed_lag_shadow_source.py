#!/usr/bin/env python3
"""Statically audit the v44 fixed-lag shadow core before any raw replay."""

from __future__ import annotations

import argparse
import ast
from dataclasses import fields
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import resource
import sys
from typing import Any, Iterable, Mapping

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_CONTRACT_ID = 'v44d-fixed-lag-shadow-source-audit-20260810'
EXPECTED_STAGE = 'report_only_shadow_source_implementation_and_static_audit'
EXPECTED_STATIC_CHECKS = (
    'prerequisite_architecture_hash_and_id',
    'prerequisite_synthetic_contract_hash_and_id',
    'prerequisite_synthetic_aggregate_hash_payload_and_decision',
    'implementation_hash_and_size',
    'python_ast_parse',
    'no_top_level_runtime_side_effect',
    'imports_are_allowlisted',
    'forbidden_imports_absent',
    'forbidden_builtin_calls_absent',
    'forbidden_attribute_calls_absent',
    'no_command_line_entrypoint',
    'no_raw_bag_or_ros_adapter',
    'no_filesystem_network_or_subprocess_surface',
    'no_wall_clock_or_parallel_solver_surface',
    'no_dataset_specific_symbols',
    'public_input_fields_exact',
    'forbidden_public_input_fields_absent',
    'required_constants_exact',
    'source_authority_fail_closed',
    'required_classes_present',
    'required_module_functions_present',
    'required_estimator_methods_present',
    'fifteen_dof_state_layout_present',
    'integer_sensor_time_path_present',
    'dynamic_unbranched_bootstrap_path_present',
    'all_five_bias_jacobians_present',
    'binary_lidar_observability_path_present',
    'factor_order_exact',
    'bounded_householder_and_rank_solve_present',
    'fixed_line_search_present',
    'square_root_fej_marginalization_present',
    'resource_preflight_present',
    'protected_output_guard_present',
    'terminal_zero_valid_output_present',
    'diagnostic_required_fields_present',
    'in_memory_smoke_probes_pass',
)
EXPECTED_PROBES = (
    'SO3_round_trip',
    'preintegration_bias_inventory',
    'covariance_PSD',
    'binary_lidar_Jacobian',
    'Householder_dense_equivalence',
    'square_root_separator_equivalence',
    'FEJ_payload_immutable',
    'protected_output_identity',
    'resource_preflight_rejection',
    'authority_closed',
)


class ContractError(ValueError):
    """The source-audit contract or its bound prerequisite is invalid."""


class MemoryBudgetError(RuntimeError):
    """The static audit would exceed its pre-registered resource budget."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


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
    return path if path.is_absolute() else ROOT / path


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


class MemoryGuard:
    def __init__(self, absolute_mib: float, incremental_mib: float) -> None:
        self.absolute_mib = float(absolute_mib)
        self.incremental_mib = float(incremental_mib)
        self.baseline_mib = current_rss_mib()
        self.peak_mib = self.baseline_mib
        self.peak_incremental_mib = 0.0
        self.absolute_enforced = self.baseline_mib <= self.absolute_mib

    def check(self, label: str) -> None:
        rss = current_rss_mib()
        self.peak_mib = max(self.peak_mib, rss)
        incremental = max(0.0, rss - self.baseline_mib)
        self.peak_incremental_mib = max(self.peak_incremental_mib, incremental)
        if self.absolute_enforced and rss > self.absolute_mib:
            raise MemoryBudgetError(
                f'RSS {rss:.3f} MiB exceeds {self.absolute_mib:.3f} MiB at {label}')
        if incremental > self.incremental_mib:
            raise MemoryBudgetError(
                f'incremental RSS {incremental:.3f} MiB exceeds '
                f'{self.incremental_mib:.3f} MiB at {label}')


def require(condition: bool, message: str) -> None:
    if not bool(condition):
        raise ContractError(message)


def require_exact_keys(
        value: Mapping[str, Any], expected: Iterable[str], label: str) -> None:
    actual = set(value)
    wanted = set(expected)
    if actual != wanted:
        raise ContractError(
            f'{label} keys differ: missing={sorted(wanted - actual)}, '
            f'extra={sorted(actual - wanted)}')


def bound_json(path_value: str, expected_sha256: str,
               label: str) -> tuple[Path, dict[str, Any]]:
    path = resolve_path(path_value)
    if not path.is_file():
        raise ContractError(f'{label} is absent: {path}')
    actual = sha256_file(path)
    if actual != expected_sha256:
        raise ContractError(
            f'{label} SHA-256 differs: {actual} != {expected_sha256}')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'{label} is not valid JSON') from error
    if not isinstance(value, dict):
        raise ContractError(f'{label} root must be an object')
    return path, value


def validate_contract_shape(contract: dict[str, Any]) -> None:
    require_exact_keys(contract, {
        'schema_version', 'contract_id', 'stage', 'rationale', 'prerequisite',
        'implementation', 'public_input_contract', 'allowed_import_roots',
        'forbidden_import_roots', 'forbidden_call_names',
        'forbidden_attribute_calls', 'forbidden_dataset_symbol_fragments',
        'required_constants', 'required_classes', 'required_module_functions',
        'required_estimator_methods', 'required_architecture_markers',
        'required_static_checks', 'smoke_probe', 'audit_resource_bounds',
        'decision', 'next_execution_contract_requirements',
    }, 'v44d contract')
    require(contract['schema_version'] == 1, 'unsupported v44d schema_version')
    require(contract['contract_id'] == EXPECTED_CONTRACT_ID,
            'v44d contract ID differs')
    require(contract['stage'] == EXPECTED_STAGE, 'v44d stage differs')
    require(tuple(contract['required_static_checks']) == EXPECTED_STATIC_CHECKS,
            'v44d static check inventory or order differs')
    require(len(set(contract['required_static_checks'])) == len(EXPECTED_STATIC_CHECKS),
            'v44d static check inventory contains duplicates')
    smoke = contract['smoke_probe']
    require(smoke['synthetic_only'] is True,
            'v44d smoke probes must remain synthetic-only')
    require(tuple(smoke['required_probe_ids']) == EXPECTED_PROBES,
            'v44d smoke probe inventory or order differs')
    require(int(smoke['random_seed']) == 4404001,
            'v44d smoke random seed differs')
    decision = contract['decision']
    require(int(decision['required_validation_repetitions']) == 2,
            'v44d requires exactly two repetitions')
    require(int(decision['required_static_check_count']) == len(
        EXPECTED_STATIC_CHECKS), 'v44d required static count differs')
    require(int(decision['required_smoke_probe_count']) == len(EXPECTED_PROBES),
            'v44d required smoke count differs')
    require(decision['on_pass'] ==
            'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_CONTRACT_DEFINITION',
            'v44d PASS decision differs')
    require(decision['on_fail'] == 'REJECT_V44D_SHADOW_SOURCE_BOUNDARY',
            'v44d FAIL decision differs')
    require(decision[
        'raw_shadow_replay_contract_definition_authorized_on_pass'] is True,
        'v44d PASS must authorize only replay-contract definition')
    for key in (
            'raw_shadow_replay_execution_authorized_on_pass',
            'accuracy_or_reference_map_inputs_authorized_on_pass',
            'primary_trajectory_or_map_mutation_authorized_on_pass'):
        require(decision[key] is False, f'v44d must keep {key} false')
    resources = contract['audit_resource_bounds']
    require(resources['rss_measurement_scope'] ==
            'absolute_for_standalone_process_incremental_for_preloaded_host',
            'v44d RSS measurement scope differs')
    for key in ('maximum_rss_mib', 'maximum_incremental_rss_mib',
                'maximum_ast_nodes', 'maximum_report_bytes'):
        require(float(resources[key]) > 0.0, f'v44d {key} must be positive')
    require(set(contract['allowed_import_roots']).isdisjoint(
        contract['forbidden_import_roots']),
        'allowed and forbidden import roots overlap')
    require(bool(contract['next_execution_contract_requirements']),
            'next execution contract requirements are empty')


def load_and_validate_contract(
        contract_path: Path) -> tuple[dict[str, Any], str, dict[str, Any],
                                      dict[str, Any], dict[str, Any], Path]:
    path = resolve_path(contract_path)
    try:
        contract = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'cannot read v44d contract: {path}') from error
    if not isinstance(contract, dict):
        raise ContractError('v44d contract root must be an object')
    validate_contract_shape(contract)
    prerequisite = contract['prerequisite']
    require_exact_keys(prerequisite, {
        'architecture_contract_path', 'architecture_contract_sha256',
        'required_architecture_contract_id', 'synthetic_contract_path',
        'synthetic_contract_sha256', 'required_synthetic_contract_id',
        'synthetic_aggregate_path', 'synthetic_aggregate_sha256',
        'required_synthetic_aggregate_payload_sha256',
        'required_synthetic_decision',
    }, 'v44d prerequisite')
    _, architecture = bound_json(
        prerequisite['architecture_contract_path'],
        prerequisite['architecture_contract_sha256'], 'v44b architecture')
    require(architecture.get('contract_id') ==
            prerequisite['required_architecture_contract_id'],
            'v44b architecture ID differs')
    _, synthetic = bound_json(
        prerequisite['synthetic_contract_path'],
        prerequisite['synthetic_contract_sha256'], 'v44c1 synthetic contract')
    require(synthetic.get('contract_id') ==
            prerequisite['required_synthetic_contract_id'],
            'v44c1 synthetic contract ID differs')
    _, aggregate = bound_json(
        prerequisite['synthetic_aggregate_path'],
        prerequisite['synthetic_aggregate_sha256'], 'v44c1 aggregate')
    require(aggregate.get('contract_id') ==
            prerequisite['required_synthetic_contract_id'],
            'v44c1 aggregate contract ID differs')
    require(aggregate.get('contract_sha256') ==
            prerequisite['synthetic_contract_sha256'],
            'v44c1 aggregate contract hash differs')
    require(aggregate.get('aggregate_payload_sha256') ==
            prerequisite['required_synthetic_aggregate_payload_sha256'],
            'v44c1 aggregate payload hash differs')
    require(aggregate.get('decision') ==
            prerequisite['required_synthetic_decision'],
            'v44c1 prerequisite decision differs')
    require(aggregate.get('status') == 'PASS',
            'v44c1 prerequisite did not pass')
    require(aggregate.get(
        'report_only_shadow_estimator_implementation_authorized') is True,
        'v44c1 did not authorize report-only source implementation')
    for key in (
            'raw_shadow_replay_authorized',
            'accuracy_or_reference_map_inputs_authorized',
            'primary_trajectory_or_map_mutation_authorized'):
        require(aggregate.get(key) is False,
                f'v44c1 prerequisite unexpectedly opened {key}')
    implementation = contract['implementation']
    require_exact_keys(implementation, {
        'path', 'sha256', 'maximum_source_bytes', 'language', 'kind',
        'top_level_side_effects_allowed', 'command_line_interface_allowed',
        'raw_bag_decoder_allowed', 'filesystem_read_or_write_allowed',
        'network_or_subprocess_allowed',
        'ros_subscription_or_publication_allowed', 'wall_clock_input_allowed',
        'parallel_solver_allowed',
    }, 'v44d implementation')
    source_path = resolve_path(implementation['path'])
    require(source_path.is_file(), f'v44d implementation is absent: {source_path}')
    require(sha256_file(source_path) == implementation['sha256'],
            'v44d implementation SHA-256 differs')
    require(source_path.stat().st_size <= int(implementation['maximum_source_bytes']),
            'v44d implementation exceeds source byte capacity')
    require(implementation['kind'] ==
            'in_memory_estimator_core_without_runtime_adapter',
            'v44d implementation kind differs')
    for key in (
            'top_level_side_effects_allowed', 'command_line_interface_allowed',
            'raw_bag_decoder_allowed', 'filesystem_read_or_write_allowed',
            'network_or_subprocess_allowed',
            'ros_subscription_or_publication_allowed', 'wall_clock_input_allowed',
            'parallel_solver_allowed'):
        require(implementation[key] is False,
                f'v44d implementation boundary opened {key}')
    return (
        contract, sha256_file(path), architecture, synthetic, aggregate,
        source_path)


def imported_roots(tree: ast.AST) -> set[str]:
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    return roots


def assigned_literals(tree: ast.Module) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(
                node.targets[0], ast.Name):
            try:
                result[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
    return result


def class_map(tree: ast.Module) -> dict[str, ast.ClassDef]:
    return {node.name: node for node in tree.body if isinstance(node, ast.ClassDef)}


def module_function_names(tree: ast.Module) -> set[str]:
    return {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}


def class_method_names(node: ast.ClassDef) -> set[str]:
    return {item.name for item in node.body if isinstance(item, ast.FunctionDef)}


def annotated_class_fields(node: ast.ClassDef) -> list[str]:
    return [
        item.target.id for item in node.body
        if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name)]


def called_names(tree: ast.AST) -> tuple[set[str], set[str]]:
    names: set[str] = set()
    attributes: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Name):
            names.add(node.func.id)
        elif isinstance(node.func, ast.Attribute):
            attributes.add(node.func.attr)
    return names, attributes


def has_main_entrypoint(tree: ast.Module) -> bool:
    for node in tree.body:
        if not isinstance(node, ast.If):
            continue
        for child in ast.walk(node.test):
            if isinstance(child, ast.Name) and child.id == '__name__':
                return True
    return False


def load_source_module(source_path: Path, digest: str) -> Any:
    name = f'v44d_shadow_probe_{digest[:16]}'
    specification = importlib.util.spec_from_file_location(name, source_path)
    if specification is None or specification.loader is None:
        raise ContractError('cannot create implementation import specification')
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def run_smoke_probes(
        module: Any, architecture: dict[str, Any], seed: int,
        ) -> dict[str, dict[str, Any]]:
    probes: dict[str, dict[str, Any]] = {}
    configuration = module.FixedLagShadowConfig.from_architecture(architecture)

    vector = np.array([0.2, -0.1, 0.4])
    error = float(np.linalg.norm(module.so3_log(module.so3_exp(vector)) - vector))
    require(error <= 1e-12, 'SO3 smoke round trip failed')
    probes['SO3_round_trip'] = {'status': 'PASS', 'error_rad': error}

    times = np.arange(0, 110_000_000, 10_000_000, dtype=np.int64)
    gyro = np.tile([0.1, -0.05, 0.2], (len(times), 1))
    accel = np.tile([0.3, -0.2, 9.7], (len(times), 1))
    preintegration = module.preintegrate_midpoint(
        times, gyro, accel, np.zeros(3), np.zeros(3), configuration)
    inventory = [
        name for name in ('J_R_bg', 'J_v_bg', 'J_v_ba', 'J_p_bg', 'J_p_ba')
        if getattr(preintegration, name).shape == (3, 3)
        and np.all(np.isfinite(getattr(preintegration, name)))]
    require(len(inventory) == 5, 'preintegration bias inventory smoke failed')
    gravity_reference = np.array([0.0, 0.0, -module.GRAVITY_MAGNITUDE_M_S2])
    gravity_probe = module.perturb_gravity_direction(
        gravity_reference, [0.05, -0.03])
    gravity_magnitude_error = abs(
        float(np.linalg.norm(gravity_probe)) - module.GRAVITY_MAGNITUDE_M_S2)
    require(module.gravity_tangent_basis(gravity_probe).shape == (3, 2)
            and gravity_magnitude_error <= 1e-12,
            'S2 gravity-direction smoke failed')
    probes['preintegration_bias_inventory'] = {
        'status': 'PASS', 'jacobians': inventory,
        'gravity_local_dof': 2,
        'gravity_magnitude_error_m_s2': gravity_magnitude_error}
    minimum_eigenvalue = float(np.min(np.linalg.eigvalsh(
        preintegration.covariance)))
    require(minimum_eigenvalue >= -configuration.covariance_negative_tolerance,
            'preintegration covariance smoke failed')
    probes['covariance_PSD'] = {
        'status': 'PASS', 'minimum_eigenvalue': minimum_eigenvalue}

    source = module.Knot(
        0, 0, 0, module.so3_exp([0.1, -0.05, 0.02]),
        np.array([0.3, -0.2, 0.1]), np.zeros(3), np.zeros(3), np.zeros(3))
    current = module.Knot(
        1, 1, 1, module.so3_exp([-0.03, 0.08, 0.04]),
        np.array([1.1, 0.4, -0.2]), np.zeros(3), np.zeros(3), np.zeros(3))
    point = np.array([2.0, -0.5, 0.7])
    normal = np.asarray([0.2, -0.3, 0.9327379053], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    surfel = module.Surfel(
        (0, 0, 0), (2.4, 0.1, 0.3), tuple(normal), 10)
    _, analytic_source, analytic_current = module.lidar_residual_and_jacobians(
        source, current, point, surfel)
    maximum_jacobian_error = 0.0
    for side, analytic in (
            ('source', analytic_source), ('current', analytic_current)):
        numerical = np.zeros(6)
        for axis in range(6):
            delta = np.zeros(module.STATE_DOF)
            delta[axis] = 1e-6
            source_plus, source_minus = source.copy(), source.copy()
            current_plus, current_minus = current.copy(), current.copy()
            if side == 'source':
                source_plus.apply_delta(delta)
                source_minus.apply_delta(-delta)
            else:
                current_plus.apply_delta(delta)
                current_minus.apply_delta(-delta)
            plus = module.lidar_residual_and_jacobians(
                source_plus, current_plus, point, surfel)[0]
            minus = module.lidar_residual_and_jacobians(
                source_minus, current_minus, point, surfel)[0]
            numerical[axis] = (plus - minus) / 2e-6
        maximum_jacobian_error = max(
            maximum_jacobian_error, float(np.max(np.abs(numerical - analytic))))
    require(maximum_jacobian_error <= 1e-8,
            'binary LiDAR Jacobian smoke failed')
    probes['binary_lidar_Jacobian'] = {
        'status': 'PASS', 'maximum_error': maximum_jacobian_error}

    matrix = np.array([
        [3.0, 0.0, 1.0], [0.0, 4.0, -1.0], [2.0, 1.0, 0.0],
        [1.0, -2.0, 3.0], [0.0, 1.0, 2.0]])
    residual = np.array([1.0, -2.0, 0.5, 3.0, -1.0])
    system = module.StreamingHouseholderSystem(3, configuration)
    system.add_local(matrix[:2], residual[:2], [0, 1, 2])
    system.add_local(matrix[2:], residual[2:], [0, 1, 2])
    update, rank, _ = system.solve()
    expected, _, _, _ = np.linalg.lstsq(matrix, -residual, rcond=None)
    householder_error = float(np.linalg.norm(update - expected))
    require(rank == 3 and householder_error <= 1e-12,
            'Householder solve smoke failed')
    probes['Householder_dense_equivalence'] = {
        'status': 'PASS', 'rank': rank, 'solution_error': householder_error}

    generator = np.random.default_rng(int(seed))
    batch_matrix = generator.normal(size=(30, 8))
    batch_target = generator.normal(size=30)
    full, _, _, _ = np.linalg.lstsq(batch_matrix, batch_target, rcond=None)
    prior_matrix, prior_target, marginal_rank, _ = (
        module.square_root_separator_prior(
            batch_matrix, batch_target, 3, configuration))
    retained, _, _, _ = np.linalg.lstsq(
        prior_matrix, prior_target, rcond=None)
    separator_error = float(np.linalg.norm(retained - full[3:]))
    require(marginal_rank == 3 and separator_error <= 1e-10,
            'square-root separator smoke failed')
    probes['square_root_separator_equivalence'] = {
        'status': 'PASS', 'marginal_rank': marginal_rank,
        'solution_error': separator_error}

    knot = module.Knot(
        0, 0, 0, np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    prior = module.FirstEstimatePrior(
        [0], np.eye(module.STATE_DOF), np.zeros(module.STATE_DOF), [knot])
    before = prior.payload_sha256()
    immutable = not prior.matrix.flags.writeable and not (
        prior.snapshots[0].p_WB.flags.writeable)
    require(immutable and prior.payload_sha256() == before,
            'FEJ immutability smoke failed')
    probes['FEJ_payload_immutable'] = {
        'status': 'PASS', 'payload_sha256': before, 'immutable': immutable}

    protected = {'primary_state': '0' * 64, 'primary_map': '1' * 64}
    guard = module.ProtectedOutputGuard(protected)
    digest = guard.verify(protected)
    rejected = False
    try:
        guard.verify({'primary_state': '0' * 64, 'primary_map': '2' * 64})
    except module.ContractViolation:
        rejected = True
    require(rejected, 'protected-output mutation smoke was not rejected')
    probes['protected_output_identity'] = {
        'status': 'PASS', 'payload_sha256': digest,
        'mutation_rejected': rejected}

    capacity_rejected = False
    try:
        module.StreamingHouseholderSystem(
            configuration.maximum_state_dimension + 1, configuration)
    except module.CapacityViolation:
        capacity_rejected = True
    require(capacity_rejected, 'resource capacity smoke was not rejected')
    probes['resource_preflight_rejection'] = {
        'status': 'PASS', 'rejected_before_system_creation': capacity_rejected}

    expected_authority = {
        'raw_shadow_replay': False,
        'accuracy_or_reference_map_inputs': False,
        'primary_trajectory_or_map_mutation': False,
        'ros_publication': False,
        'filesystem_output': False,
    }
    require(module.SOURCE_AUTHORITY == expected_authority,
            'source authority smoke differs')
    probes['authority_closed'] = {
        'status': 'PASS', **expected_authority}
    require(tuple(probes) == EXPECTED_PROBES,
            'smoke probe result order differs')
    return probes


def static_audit(
        contract: dict[str, Any], architecture: dict[str, Any],
        source_path: Path, source_digest: str, memory: MemoryGuard,
        ) -> tuple[dict[str, bool], dict[str, Any], dict[str, dict[str, Any]]]:
    source = source_path.read_text(encoding='utf-8')
    try:
        tree = ast.parse(source, filename=str(source_path))
    except SyntaxError as error:
        raise ContractError('v44d implementation does not parse') from error
    nodes = list(ast.walk(tree))
    if len(nodes) > int(contract['audit_resource_bounds']['maximum_ast_nodes']):
        raise MemoryBudgetError('v44d AST node capacity exceeded')
    memory.check('after_ast_parse')
    checks = {identifier: False for identifier in EXPECTED_STATIC_CHECKS}

    def passed(identifier: str, condition: bool, message: str) -> None:
        require(identifier in checks, f'unknown static check: {identifier}')
        require(condition, message)
        checks[identifier] = True

    passed('prerequisite_architecture_hash_and_id',
           architecture['contract_id'] == contract['prerequisite'][
               'required_architecture_contract_id'],
           'architecture prerequisite static check failed')
    passed('prerequisite_synthetic_contract_hash_and_id', True,
           'synthetic contract prerequisite static check failed')
    passed('prerequisite_synthetic_aggregate_hash_payload_and_decision', True,
           'synthetic aggregate prerequisite static check failed')
    implementation = contract['implementation']
    passed('implementation_hash_and_size',
           sha256_file(source_path) == implementation['sha256']
           and source_path.stat().st_size <= int(implementation['maximum_source_bytes']),
           'implementation hash or size static check failed')
    passed('python_ast_parse', isinstance(tree, ast.Module),
           'source AST root differs')

    allowed_top_level = (
        ast.Expr, ast.Import, ast.ImportFrom, ast.Assign, ast.AnnAssign,
        ast.ClassDef, ast.FunctionDef)
    top_level_safe = all(isinstance(node, allowed_top_level) for node in tree.body)
    for index, node in enumerate(tree.body):
        if isinstance(node, ast.Expr):
            top_level_safe = top_level_safe and index == 0 and isinstance(
                node.value, ast.Constant) and isinstance(node.value.value, str)
        if isinstance(node, ast.Assign):
            try:
                ast.literal_eval(node.value)
            except (ValueError, TypeError):
                top_level_safe = False
    passed('no_top_level_runtime_side_effect', top_level_safe,
           'source has a top-level runtime side effect')

    imports = imported_roots(tree)
    allowed_imports = set(contract['allowed_import_roots'])
    forbidden_imports = set(contract['forbidden_import_roots'])
    passed('imports_are_allowlisted', imports <= allowed_imports,
           f'source imports outside allowlist: {sorted(imports - allowed_imports)}')
    passed('forbidden_imports_absent', imports.isdisjoint(forbidden_imports),
           f'source imports forbidden modules: {sorted(imports & forbidden_imports)}')
    call_names, attribute_calls = called_names(tree)
    forbidden_calls = set(contract['forbidden_call_names'])
    forbidden_attributes = set(contract['forbidden_attribute_calls'])
    passed('forbidden_builtin_calls_absent', call_names.isdisjoint(forbidden_calls),
           f'source calls forbidden builtins: {sorted(call_names & forbidden_calls)}')
    passed('forbidden_attribute_calls_absent',
           attribute_calls.isdisjoint(forbidden_attributes),
           f'source calls forbidden attributes: '
           f'{sorted(attribute_calls & forbidden_attributes)}')
    passed('no_command_line_entrypoint',
           not has_main_entrypoint(tree) and 'argparse' not in imports,
           'source contains a command-line entrypoint')

    classes = class_map(tree)
    functions = module_function_names(tree)
    symbol_names = {node.id.lower() for node in nodes if isinstance(node, ast.Name)}
    symbol_names.update(node.arg.lower() for node in nodes if isinstance(node, ast.arg))
    symbol_names.update(classes)
    symbol_names.update(functions)
    symbol_text = ' '.join(sorted(str(item).lower() for item in symbol_names))
    forbidden_fragments = tuple(
        str(item).lower() for item in contract['forbidden_dataset_symbol_fragments'])
    passed('no_raw_bag_or_ros_adapter',
           not any(fragment in symbol_text for fragment in (
               'rosbag', 'pointcloud2', 'subscription', 'publisher')),
           'source contains a raw-bag or ROS adapter symbol')
    passed('no_filesystem_network_or_subprocess_surface',
           imports.isdisjoint({'pathlib', 'os', 'socket', 'requests',
                                'urllib', 'subprocess', 'shutil'})
           and attribute_calls.isdisjoint({'open', 'write', 'write_text',
                                            'write_bytes', 'system', 'popen'}),
           'source contains filesystem, network, or subprocess surface')
    passed('no_wall_clock_or_parallel_solver_surface',
           imports.isdisjoint({'time', 'threading', 'multiprocessing',
                                'concurrent', 'asyncio'}),
           'source contains wall-clock or parallel solver surface')
    passed('no_dataset_specific_symbols',
           not any(fragment in symbol_text for fragment in forbidden_fragments),
           'source contains a dataset-specific symbol')

    input_contract = contract['public_input_contract']
    field_checks = []
    all_public_fields: set[str] = set()
    for key in ('imu_record', 'lidar_point_record', 'lidar_scan_record',
                'fixed_extrinsic_record'):
        record = input_contract[key]
        class_node = classes.get(record['class'])
        actual_fields = [] if class_node is None else annotated_class_fields(class_node)
        expected_fields = list(record['exact_fields'])
        field_checks.append(actual_fields == expected_fields)
        all_public_fields.update(actual_fields)
    passed('public_input_fields_exact', all(field_checks),
           'public input dataclass fields differ')
    passed('forbidden_public_input_fields_absent',
           all_public_fields.isdisjoint(input_contract['forbidden_public_fields']),
           'forbidden public input field is present')

    literals = assigned_literals(tree)
    required_constants = contract['required_constants']
    passed('required_constants_exact',
           all(literals.get(key) == value for key, value in required_constants.items()),
           'required implementation constant differs')
    authority = required_constants['SOURCE_AUTHORITY']
    passed('source_authority_fail_closed',
           all(value is False for value in authority.values()),
           'source authority contains an open route')
    passed('required_classes_present',
           set(contract['required_classes']) <= set(classes),
           'required source class is absent')
    passed('required_module_functions_present',
           set(contract['required_module_functions']) <= functions,
           'required source function is absent')
    estimator = classes.get('FixedLagShadowEstimator')
    estimator_methods = set() if estimator is None else class_method_names(estimator)
    passed('required_estimator_methods_present',
           set(contract['required_estimator_methods']) <= estimator_methods,
           'required estimator method is absent')
    knot_fields = annotated_class_fields(classes['Knot']) if 'Knot' in classes else []
    passed('fifteen_dof_state_layout_present',
           literals.get('STATE_DOF') == 15 and knot_fields == [
               'knot_id', 'scan_index', 'timestamp_ns', 'R_WB', 'p_WB',
               'v_WB', 'b_g', 'b_a'],
           '15-DoF knot layout differs')

    markers = contract['required_architecture_markers']
    marker_presence = {
        group: all(str(marker) in source for marker in values)
        for group, values in markers.items()}
    passed('integer_sensor_time_path_present', marker_presence['integer_sensor_time'],
           'integer sensor-time marker is absent')
    passed('dynamic_unbranched_bootstrap_path_present',
           marker_presence['dynamic_bootstrap']
           and not any(name.startswith('stationary') for name in symbol_names),
           'dynamic bootstrap path is incomplete or branched')
    passed('all_five_bias_jacobians_present',
           marker_presence['preintegration_and_bias'],
           'one or more bias Jacobian paths are absent')
    passed('binary_lidar_observability_path_present',
           marker_presence['lidar_and_observability'],
           'binary LiDAR observability path is absent')
    passed('factor_order_exact', literals.get('FACTOR_TYPE_ORDER') == {
        'gauge': 0, 'marginal_prior': 1, 'imu_preintegration': 2,
        'bias_random_walk': 3, 'lidar_point_to_plane': 4},
        'factor type order differs')
    passed('bounded_householder_and_rank_solve_present',
           marker_presence['fixed_lag_optimizer']
           and {'qr', 'svd'} <= attribute_calls
           and 'maximum_dense_solver_bytes' in source,
           'bounded Householder/rank solve is incomplete')
    passed('fixed_line_search_present',
           'line_search_scales' in source and '_optimize' in estimator_methods
           and 'wall_clock' not in estimator_methods,
           'fixed line-search path is incomplete')
    passed('square_root_fej_marginalization_present',
           all(item in source for item in (
               'square_root_separator_prior', 'FirstEstimatePrior',
               '_marginalize_oldest', 'payload_sha256')),
           'square-root FEJ marginalization path is incomplete')
    passed('resource_preflight_present',
           all(item in source for item in (
               'maximum_input_message_bytes', 'maximum_active_correspondences',
               'maximum_materialized_rows', 'maximum_dense_solver_bytes',
               'maximum_diagnostic_output_bytes', 'record_runtime_observation')),
           'resource preflight path is incomplete')
    passed('protected_output_guard_present',
           marker_presence['fail_closed_isolation']
           and 'verify(protected_after)' in source,
           'protected-output guard path is incomplete')
    passed('terminal_zero_valid_output_present',
           "'valid_shadow_result': False" in source
           and "'state_payload_sha256': None" in source
           and "'active_state_count': 0" in source,
           'terminal zero-valid-output path is incomplete')
    diagnostics = architecture['diagnostics']['required_record_fields']
    passed('diagnostic_required_fields_present',
           all(f"'{field}'" in source for field in diagnostics),
           'required diagnostic field is absent')

    module = load_source_module(source_path, source_digest)
    probes = run_smoke_probes(
        module, architecture, int(contract['smoke_probe']['random_seed']))
    passed('in_memory_smoke_probes_pass',
           tuple(probes) == EXPECTED_PROBES
           and all(item['status'] == 'PASS' for item in probes.values()),
           'one or more in-memory smoke probes failed')
    memory.check('after_smoke_probes')
    require(all(checks.values()), 'one or more static checks did not pass')
    metrics = {
        'ast_node_count': len(nodes),
        'source_bytes': source_path.stat().st_size,
        'import_roots': sorted(imports),
        'class_count': len(classes),
        'module_function_count': len(functions),
        'estimator_method_count': len(estimator_methods),
        'public_input_fields': sorted(all_public_fields),
        'architecture_marker_groups': marker_presence,
    }
    return checks, metrics, probes


def write_json_bounded(path: Path, value: dict[str, Any], maximum_bytes: int) -> None:
    encoded = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False)
               + '\n').encode('utf-8')
    if len(encoded) > int(maximum_bytes):
        raise MemoryBudgetError('v44d report exceeds output byte capacity')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)


def validate_once(
        contract_path: Path, repetition: int, output: Path) -> dict[str, Any]:
    contract, contract_digest, architecture, _, aggregate, source_path = (
        load_and_validate_contract(contract_path))
    required_repetitions = int(contract['decision']['required_validation_repetitions'])
    require(1 <= int(repetition) <= required_repetitions,
            'v44d repetition is outside the contract')
    resources = contract['audit_resource_bounds']
    memory = MemoryGuard(
        resources['maximum_rss_mib'], resources['maximum_incremental_rss_mib'])
    memory.check('start')
    source_digest = sha256_file(source_path)
    checks, metrics, probes = static_audit(
        contract, architecture, source_path, source_digest, memory)
    deterministic = {
        'prerequisite': {
            'architecture_contract_id': architecture['contract_id'],
            'architecture_contract_sha256': contract['prerequisite'][
                'architecture_contract_sha256'],
            'synthetic_contract_id': aggregate['contract_id'],
            'synthetic_contract_sha256': contract['prerequisite'][
                'synthetic_contract_sha256'],
            'synthetic_aggregate_sha256': contract['prerequisite'][
                'synthetic_aggregate_sha256'],
            'synthetic_aggregate_payload_sha256': aggregate[
                'aggregate_payload_sha256'],
            'synthetic_decision': aggregate['decision'],
        },
        'implementation_sha256': source_digest,
        'static_checks': checks,
        'static_metrics': metrics,
        'smoke_probes': probes,
        'source_boundary_validated': True,
        'raw_runtime_adapter_present': False,
        'raw_replay_executed': False,
        'authority': dict(contract['required_constants']['SOURCE_AUTHORITY']),
    }
    report = {
        'schema_version': 1,
        'audit': 'v44d_fixed_lag_shadow_source_static_validation',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'implementation_sha256': source_digest,
        'auditor_sha256': sha256_file(Path(__file__).resolve()),
        'repetition': int(repetition),
        'status': 'PASS',
        'decision': contract['decision']['on_pass'],
        'deterministic': deterministic,
        'report_payload_sha256': payload_sha256(deterministic),
        'resource_usage': {
            'baseline_rss_mib': memory.baseline_mib,
            'peak_rss_mib': memory.peak_mib,
            'peak_incremental_rss_mib': memory.peak_incremental_mib,
            'absolute_ceiling_enforced': memory.absolute_enforced,
            'maximum_rss_mib': memory.absolute_mib,
            'maximum_incremental_rss_mib': memory.incremental_mib,
        },
        'raw_shadow_replay_contract_definition_authorized': True,
        'raw_shadow_replay_execution_authorized': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
    }
    memory.check('before_report_write')
    write_json_bounded(output, report, int(resources['maximum_report_bytes']))
    return report


def validate_source_report(
        report: dict[str, Any], contract: dict[str, Any],
        contract_digest: str, source_digest: str, auditor_digest: str) -> None:
    require(report.get('audit') ==
            'v44d_fixed_lag_shadow_source_static_validation',
            'v44d source report audit ID differs')
    require(report.get('contract_id') == contract['contract_id'],
            'v44d source report contract ID differs')
    require(report.get('contract_sha256') == contract_digest,
            'v44d source report contract hash differs')
    require(report.get('implementation_sha256') == source_digest,
            'v44d source report implementation hash differs')
    require(report.get('auditor_sha256') == auditor_digest,
            'v44d source report auditor hash differs')
    require(report.get('status') == 'PASS', 'v44d source report did not pass')
    require(report.get('decision') == contract['decision']['on_pass'],
            'v44d source report decision differs')
    deterministic = report.get('deterministic')
    require(isinstance(deterministic, dict),
            'v44d source report deterministic payload is absent')
    require(report.get('report_payload_sha256') == payload_sha256(deterministic),
            'v44d source report payload hash differs')
    require(set(deterministic.get('static_checks', {})) == set(
        EXPECTED_STATIC_CHECKS)
        and len(deterministic.get('static_checks', {})) == len(
            EXPECTED_STATIC_CHECKS),
            'v44d source report static check inventory differs')
    require(all(deterministic['static_checks'].values()),
            'v44d source report contains a failed static check')
    require(set(deterministic.get('smoke_probes', {})) == set(EXPECTED_PROBES)
            and len(deterministic.get('smoke_probes', {})) == len(EXPECTED_PROBES),
            'v44d source report smoke inventory differs')
    require(all(item.get('status') == 'PASS'
                for item in deterministic['smoke_probes'].values()),
            'v44d source report contains a failed smoke probe')
    require(deterministic.get('raw_runtime_adapter_present') is False,
            'v44d source report unexpectedly contains a runtime adapter')
    require(deterministic.get('raw_replay_executed') is False,
            'v44d source report unexpectedly executed raw replay')
    require(report.get(
        'raw_shadow_replay_contract_definition_authorized') is True,
        'v44d source report did not authorize replay-contract definition')
    for key in (
            'raw_shadow_replay_execution_authorized',
            'accuracy_or_reference_map_inputs_authorized',
            'primary_trajectory_or_map_mutation_authorized'):
        require(report.get(key) is False,
                f'v44d source report unexpectedly opened {key}')


def aggregate_reports(
        contract_path: Path, reports: list[Path], output: Path,
        markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_digest, _, _, _, source_path = (
        load_and_validate_contract(contract_path))
    required = int(contract['decision']['required_validation_repetitions'])
    require(len(reports) == required,
            f'v44d aggregate requires exactly {required} reports')
    source_digest = sha256_file(source_path)
    auditor_digest = sha256_file(Path(__file__).resolve())
    loaded: list[tuple[Path, dict[str, Any]]] = []
    for path_value in reports:
        path = resolve_path(path_value)
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            raise ContractError(f'cannot read v44d source report: {path}') from error
        validate_source_report(
            report, contract, contract_digest, source_digest, auditor_digest)
        loaded.append((path, report))
    repetitions = sorted(int(item[1]['repetition']) for item in loaded)
    require(repetitions == list(range(1, required + 1)),
            'v44d report repetitions are incomplete or duplicated')
    payloads = {item[1]['report_payload_sha256'] for item in loaded}
    require(len(payloads) == 1,
            'v44d source validation is not deterministic')
    deterministic = {
        'validation_complete': True,
        'validation_repeatable': True,
        'validation_repetition_count': required,
        'report_payload_sha256': next(iter(payloads)),
        'static_check_count': len(EXPECTED_STATIC_CHECKS),
        'smoke_probe_count': len(EXPECTED_PROBES),
        'implementation_sha256': source_digest,
        'raw_runtime_adapter_present': False,
        'raw_replay_executed': False,
    }
    aggregate = {
        'schema_version': 1,
        'audit': 'v44d_fixed_lag_shadow_source_static_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'implementation_sha256': source_digest,
        'auditor_sha256': auditor_digest,
        'status': 'PASS',
        'decision': contract['decision']['on_pass'],
        'deterministic': deterministic,
        'aggregate_payload_sha256': payload_sha256(deterministic),
        'source_reports': [
            {'path': str(path.resolve()), 'sha256': sha256_file(path)}
            for path, _ in loaded],
        'raw_shadow_replay_contract_definition_authorized': True,
        'raw_shadow_replay_execution_authorized': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
    }
    write_json_bounded(
        output, aggregate,
        int(contract['audit_resource_bounds']['maximum_report_bytes']))
    if markdown_output is not None:
        lines = [
            '# v44d fixed-lag shadow source static audit', '',
            f"- status: `{aggregate['status']}`",
            f"- decision: `{aggregate['decision']}`",
            f"- contract: `{aggregate['contract_id']}`",
            f"- contract SHA-256: `{aggregate['contract_sha256']}`",
            f"- implementation SHA-256: `{source_digest}`",
            f"- auditor SHA-256: `{auditor_digest}`",
            f"- deterministic report payload SHA-256: `{next(iter(payloads))}`",
            f"- aggregate payload SHA-256: `{aggregate['aggregate_payload_sha256']}`",
            f"- static checks: `{len(EXPECTED_STATIC_CHECKS)}`",
            f"- synthetic smoke probes: `{len(EXPECTED_PROBES)}`",
            '- raw runtime adapter present: `false`',
            '- raw replay executed/authorized: `false`',
            '- accuracy or reference-map inputs authorized: `false`',
            '- primary trajectory or map mutation authorized: `false`',
            '',
            'Only definition of a separately hash-bound raw shadow replay '
            'contract is authorized.',
            '',
        ]
        encoded = '\n'.join(lines).encode('utf-8')
        if len(encoded) > int(contract['audit_resource_bounds'][
                'maximum_report_bytes']):
            raise MemoryBudgetError('v44d markdown report exceeds byte capacity')
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
