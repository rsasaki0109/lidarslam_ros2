#!/usr/bin/env python3
"""Statically audit the v44e raw-shadow replay contract and adapter."""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import resource
import sys
import tempfile
from types import SimpleNamespace
from typing import Any, Mapping

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_STATIC_CHECKS = (
    'v44d_contract_hash_id_and_boundary',
    'v44d_aggregate_hash_payload_and_decision',
    'v44a_contract_manifest_and_aggregate_bound',
    'architecture_and_estimator_core_bound',
    'three_source_records_exactly_match_manifest',
    'three_bag_paths_sizes_and_hashes_explicit',
    'readiness_reports_bind_stream_digests_and_delays',
    'sensor_adapters_and_extrinsics_exact',
    'protected_v17_state_and_map_artifacts_exact',
    'adapter_hash_path_and_size',
    'adapter_python_ast_parse',
    'adapter_imports_are_allowlisted',
    'rosbags_import_is_lazy',
    'no_ros_network_or_subprocess_surface',
    'cli_has_only_contract_sequence_and_repetition',
    'no_user_selected_bag_or_output_path',
    'authorization_aggregate_required_before_bag_open',
    'authority_is_report_only_and_fail_closed',
    'required_adapter_symbols_present',
    'canonical_lidar_and_imu_decoder_present',
    'orientation_and_covariance_are_not_consumed',
    'bounded_header_time_reorder_present',
    'per_scan_rss_and_rtf_enforcement_present',
    'protected_hashes_compared_before_and_after',
    'exclusive_bounded_evidence_writer_present',
    'core_receives_no_dataset_identity',
    'accuracy_reference_ros_and_primary_routes_closed',
    'static_gate_does_not_open_raw_bags',
    'synthetic_adapter_probes_pass',
)
EXPECTED_PROBES = (
    'imu_decode_exact',
    'pointcloud_decode_exact',
    'pointcloud_schema_rejection',
    'message_capacity_rejection',
    'header_time_watermark_order',
    'reorder_capacity_rejection',
    'extrinsic_decode_exact',
    'authorization_fail_closed',
    'protected_identity_fail_closed',
    'bounded_exclusive_writer',
)
EXPECTED_ADAPTER_CLASSES = {
    'ContractError', 'AuthorizationError', 'CapacityError',
    'BufferedSensorMessage', 'DeterministicEventReorder',
    'BoundedEvidenceWriter',
}
EXPECTED_ADAPTER_FUNCTIONS = {
    'canonical_json', 'payload_sha256', 'sha256_file', 'load_json',
    'exact_path', 'file_identity', 'verify_bound_file', 'current_rss_mib',
    'header_stamp_ns', 'point_dtype', 'pointcloud_view',
    'decode_imu_message', 'decode_lidar_message',
    'validate_authorization_payload', 'validate_authorization_source_reports',
    'validate_contract_for_runtime',
    'load_runtime_context', 'source_binding', 'body_from_lidar',
    'hash_protected_artifacts', 'expected_run_directory',
    'record_runtime_after_scans', 'consume_buffered_event',
    'stream_raw_bag', 'run_replay', 'build_parser', 'main',
}


class ContractError(ValueError):
    """A v44e source, prerequisite, or report violates the frozen gate."""


class MemoryBudgetError(RuntimeError):
    """The standalone static audit exceeded its bounded resources."""


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


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float,
                 maximum_incremental_rss_mib: float) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.maximum_incremental_rss_mib = float(maximum_incremental_rss_mib)
        self.baseline_rss_mib = self.current()
        self.peak_rss_mib = self.baseline_rss_mib

    @staticmethod
    def current() -> float:
        status_path = Path('/proc/self/status')
        if status_path.is_file():
            for line in status_path.read_text(encoding='utf-8').splitlines():
                if line.startswith('VmRSS:'):
                    return float(line.split()[1]) / 1024.0
        return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0

    def check(self, label: str) -> None:
        current = self.current()
        self.peak_rss_mib = max(self.peak_rss_mib, current)
        if current > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS exceeds absolute bound at {label}: {current:.3f} MiB')
        if current - self.baseline_rss_mib > self.maximum_incremental_rss_mib:
            raise MemoryBudgetError(
                f'RSS exceeds incremental bound at {label}: {current:.3f} MiB')


def validate_contract_shape(contract: Mapping[str, Any]) -> None:
    required = {
        'schema_version', 'contract_id', 'stage', 'rationale',
        'prerequisites', 'estimator_core', 'adapter', 'static_auditor',
        'authority', 'source_binding', 'messages', 'reorder_buffer',
        'runtime_resources', 'output', 'authorization', 'execution',
        'required_static_checks', 'smoke_probe', 'audit_resources', 'decision',
    }
    require(set(contract) == required,
            'v44e top-level key inventory differs')
    require(contract['schema_version'] == 1,
            'unsupported v44e schema_version')
    require(contract['contract_id'] ==
            'v44e-raw-shadow-replay-execution-20260810',
            'v44e contract ID differs')
    require(tuple(contract['required_static_checks']) == EXPECTED_STATIC_CHECKS,
            'v44e static check inventory or order differs')
    require(tuple(contract['smoke_probe']['required_probe_ids']) ==
            EXPECTED_PROBES,
            'v44e smoke probe inventory or order differs')
    require(int(contract['decision']['required_static_repetitions']) == 2,
            'v44e requires exactly two static repetitions')
    require(int(contract['decision']['required_static_check_count']) ==
            len(EXPECTED_STATIC_CHECKS),
            'v44e static check count differs')
    require(int(contract['decision']['required_smoke_probe_count']) ==
            len(EXPECTED_PROBES),
            'v44e smoke probe count differs')
    require(contract['decision']['on_pass'] ==
            'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION',
            'v44e PASS decision differs')
    require(contract['decision']['on_fail'] ==
            'REJECT_V44E_RAW_SHADOW_REPLAY_BOUNDARY',
            'v44e FAIL decision differs')
    require(contract['decision'][
        'raw_shadow_replay_execution_authorized_on_pass'] is True,
        'v44e PASS must authorize only bounded raw replay')
    require(contract['decision']['raw_replay_executed_by_static_gate'] is False,
            'v44e static gate must not execute raw replay')
    for key in (
            'accuracy_or_reference_map_inputs_authorized_on_pass',
            'primary_trajectory_or_map_mutation_authorized_on_pass',
            'ros_publication_authorized_on_pass'):
        require(contract['decision'][key] is False,
                f'v44e must keep {key} false')
    authority = contract['authority']
    require(authority == {
        'raw_shadow_replay': True,
        'accuracy_or_reference_map_inputs': False,
        'primary_trajectory_or_map_mutation': False,
        'primary_bias_writeback': False,
        'ros_publication': False,
        'loop_or_global_correction': False,
        'diagnostic_evidence_output_only': True,
    }, 'v44e authority inventory differs')
    require(contract['execution']['required_repetitions_per_sequence'] == 2,
            'v44e raw repetition count differs')
    require(contract['execution']['required_sequence_count'] == 3,
            'v44e raw sequence count differs')
    require(contract['execution'][
        'raw_bag_open_allowed_only_after_authorization'] is True,
        'v44e must authorize before opening a bag')
    require(contract['authorization'][
        'source_reports_revalidated_at_runtime'] is True,
        'v44e must revalidate authorization source reports')
    require(contract['output']['filenames'] == [
        'diagnostics.jsonl', 'run.json'],
        'v44e output filename inventory differs')
    require(contract['output']['overwrite_allowed'] is False,
            'v44e output overwrite must remain false')
    for key in (
            'maximum_buffered_messages', 'maximum_buffered_serialized_bytes'):
        require(int(contract['reorder_buffer'][key]) > 0,
                f'v44e {key} must be positive')
    for key in ('maximum_bytes_per_run',):
        require(int(contract['output'][key]) > 0,
                f'v44e {key} must be positive')
    for key in (
            'maximum_rss_mib', 'maximum_processing_rtf',
            'maximum_input_message_bytes'):
        require(float(contract['runtime_resources'][key]) > 0.0,
                f'v44e {key} must be positive')
    require(contract['runtime_resources']['maximum_rss_mib'] == 330.0,
            'v44e RSS ceiling must remain 330 MiB')
    require(contract['runtime_resources']['maximum_processing_rtf'] == 0.85,
            'v44e RTF ceiling must remain 0.85')
    require(contract['runtime_resources']['check_after_every_scan'] is True,
            'v44e resources must be checked after every scan')


def _load_bound_json(binding: Mapping[str, Any], label: str) -> dict[str, Any]:
    path = resolve_path(binding['path'])
    require(path.is_file(), f'{label} is absent')
    require(sha256_file(path) == binding['sha256'],
            f'{label} SHA-256 differs')
    if 'bytes' in binding:
        require(path.stat().st_size == int(binding['bytes']),
                f'{label} byte size differs')
    return load_json(path)


def load_and_validate_contract(
        contract_path: Path) -> tuple[
            dict[str, Any], str, dict[str, Any], dict[str, Any], dict[str, Any]]:
    contract_path = resolve_path(contract_path)
    contract = load_json(contract_path)
    validate_contract_shape(contract)
    require(contract_path == resolve_path(
        contract['execution']['required_contract_path']),
        'v44e required contract path differs')
    prerequisites = contract['prerequisites']
    v44d_contract = _load_bound_json(
        prerequisites['v44d_contract'], 'v44d contract')
    require(v44d_contract.get('contract_id') == prerequisites[
        'v44d_contract']['required_contract_id'],
        'v44d prerequisite contract ID differs')
    v44d_aggregate = _load_bound_json(
        prerequisites['v44d_aggregate'], 'v44d aggregate')
    require(v44d_aggregate.get('decision') == prerequisites[
        'v44d_aggregate']['required_decision'],
        'v44d prerequisite decision differs')
    require(v44d_aggregate.get('aggregate_payload_sha256') == prerequisites[
        'v44d_aggregate']['required_payload_sha256'],
        'v44d prerequisite payload differs')
    require(v44d_aggregate.get(
        'raw_shadow_replay_contract_definition_authorized') is True,
        'v44d did not authorize execution-contract definition')
    require(v44d_aggregate.get(
        'raw_shadow_replay_execution_authorized') is False,
        'v44d unexpectedly authorized raw execution')
    v44a_contract = _load_bound_json(
        prerequisites['v44a_contract'], 'v44a readiness contract')
    source_manifest = _load_bound_json(
        prerequisites['v44a_source_manifest'], 'v44a source manifest')
    v44a_aggregate = _load_bound_json(
        prerequisites['v44a_aggregate'], 'v44a aggregate')
    require(v44a_contract.get('contract_id') == prerequisites[
        'v44a_contract']['required_contract_id'],
        'v44a readiness contract ID differs')
    require(source_manifest.get('source_set_id') == prerequisites[
        'v44a_source_manifest']['required_source_set_id'],
        'v44a source set ID differs')
    require(v44a_aggregate.get('status') == 'PASS'
            and v44a_aggregate.get('aggregate_payload_sha256') ==
            prerequisites['v44a_aggregate']['required_payload_sha256'],
            'v44a aggregate readiness differs')
    architecture = _load_bound_json(
        prerequisites['architecture'], 'v44b architecture')
    require(architecture.get('contract_id') == prerequisites[
        'architecture']['required_contract_id'],
        'v44b architecture ID differs')
    core_path = resolve_path(contract['estimator_core']['path'])
    require(core_path.is_file(), 'v44 estimator core is absent')
    require(sha256_file(core_path) == contract['estimator_core']['sha256'],
            'v44 estimator core SHA-256 differs')
    require(core_path.stat().st_size == int(contract['estimator_core']['bytes']),
            'v44 estimator core byte size differs')
    adapter_path = resolve_path(contract['adapter']['path'])
    require(adapter_path == ROOT / 'scripts/v44_raw_shadow_replay_adapter.py',
            'v44e adapter path differs')
    require(adapter_path.is_file(), 'v44e adapter is absent')
    require(sha256_file(adapter_path) == contract['adapter']['sha256'],
            'v44e adapter SHA-256 differs')
    require(adapter_path.stat().st_size <= int(
        contract['adapter']['maximum_source_bytes']),
        'v44e adapter exceeds source byte capacity')
    auditor_path = resolve_path(contract['static_auditor']['path'])
    require(auditor_path == Path(__file__).resolve(),
            'v44e executing auditor path differs')
    require(sha256_file(auditor_path) == contract['static_auditor']['sha256'],
            'v44e executing auditor SHA-256 differs')
    return (
        contract, sha256_file(contract_path), source_manifest,
        v44a_aggregate, architecture)


def quaternion_rotation(values: list[float]) -> np.ndarray:
    qx, qy, qz, qw = (float(item) for item in values)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    require(math.isfinite(norm) and norm > 1e-12,
            'calibration quaternion is invalid')
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def validate_sources(
        contract: Mapping[str, Any], source_manifest: Mapping[str, Any],
        v44a_aggregate: Mapping[str, Any]) -> list[dict[str, Any]]:
    sources = contract['source_binding']['sequences']
    manifest_sources = source_manifest['sequences']
    require(len(sources) == len(manifest_sources) == 3,
            'v44e requires exactly three source records')
    require([item['sequence_id'] for item in sources] ==
            [item['sequence_id'] for item in manifest_sources],
            'v44e source order differs from v44a')
    aggregate_by_id = {
        item['sequence_id']: item for item in v44a_aggregate['sequence_results']}
    summaries = []
    try:
        import yaml
    except ModuleNotFoundError as error:
        raise ContractError('PyYAML is required for calibration audit') from error
    manifest_fields = (
        'sequence_id', 'lidar_topic', 'lidar_frame', 'imu_topic', 'imu_frame',
        'base_frame', 'expected_lidar_messages', 'expected_lidar_points',
        'expected_imu_messages')
    for source, manifest in zip(sources, manifest_sources):
        for key in manifest_fields:
            require(source[key] == manifest[key],
                    f"{source['sequence_id']} {key} differs from v44a")
        bag = source['bag']
        require(bag == {
            'path': manifest['bag_path'], 'bytes': manifest['bag_size_bytes'],
            'sha256': manifest['bag_sha256']},
            f"{source['sequence_id']} raw bag binding differs")
        bag_path = resolve_path(bag['path'])
        require(bag_path.is_file() and not bag_path.is_symlink(),
                f"{source['sequence_id']} raw bag is absent or symlinked")
        require(bag_path.stat().st_size == int(bag['bytes']),
                f"{source['sequence_id']} raw bag byte size differs")
        readiness_binding = source['readiness_report']
        readiness = _load_bound_json(
            readiness_binding, f"{source['sequence_id']} readiness report")
        deterministic = readiness.get('deterministic', {})
        require(readiness.get('status') == 'PASS'
                and deterministic.get('sequence_id') == source['sequence_id'],
                f"{source['sequence_id']} readiness report differs")
        inventory = deterministic['inventory']
        require(source['serialized_stream_sha256'] == {
            'lidar': inventory['lidar']['serialized_payload_sha256'],
            'imu': inventory['imu']['serialized_payload_sha256']},
            f"{source['sequence_id']} serialized stream digest differs")
        maximum_delay = max(
            int(inventory['lidar']['receive_minus_header_ns']['maximum']),
            int(inventory['imu']['receive_minus_header_ns']['maximum']))
        require(int(source['maximum_receive_minus_header_ns']) == maximum_delay,
                f"{source['sequence_id']} receive watermark differs")
        aggregate = aggregate_by_id[source['sequence_id']]
        require(aggregate['ready'] is True and aggregate['repeatable'] is True,
                f"{source['sequence_id']} v44a readiness is not repeatable")
        calibration = source['calibration']
        adapter_path = resolve_path(calibration['sensor_adapter_path'])
        require(adapter_path.is_file()
                and sha256_file(adapter_path) ==
                calibration['sensor_adapter_sha256']
                == manifest['sensor_adapter_sha256'],
                f"{source['sequence_id']} sensor adapter SHA-256 differs")
        document = yaml.safe_load(adapter_path.read_text(encoding='utf-8'))
        parameters = document['rko_parameters']
        imu = [float(value) for value in
               parameters['extrinsic_imu2base_quat_xyzw_xyz']]
        lidar = [float(value) for value in
                 parameters['extrinsic_lidar2base_quat_xyzw_xyz']]
        require(imu == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                f"{source['sequence_id']} IMU-to-body is not identity")
        expected_transform = calibration['body_from_lidar']
        require(np.allclose(
            quaternion_rotation(lidar[:4]),
            np.asarray(expected_transform['rotation'], dtype=np.float64),
            atol=1e-14, rtol=0.0),
            f"{source['sequence_id']} LiDAR rotation differs")
        require(np.allclose(
            np.asarray(lidar[4:], dtype=np.float64),
            np.asarray(expected_transform['translation_m'], dtype=np.float64),
            atol=1e-15, rtol=0.0),
            f"{source['sequence_id']} LiDAR translation differs")
        run_manifest = _load_bound_json(
            source['protected_v17_run_manifest'],
            f"{source['sequence_id']} protected v17 run manifest")
        protected = {item['name']: item
                     for item in source['protected_v17_artifacts']}
        require(set(protected) == {'v17_state', 'v17_map'},
                f"{source['sequence_id']} protected artifact inventory differs")
        for name, item in protected.items():
            path = resolve_path(item['path'])
            require(path.is_file() and not path.is_symlink(),
                    f"{source['sequence_id']} {name} is absent or symlinked")
            require(path.stat().st_size == int(item['bytes']),
                    f"{source['sequence_id']} {name} byte size differs")
            require(sha256_file(path) == item['sha256'],
                    f"{source['sequence_id']} {name} SHA-256 differs")
        map_manifest = run_manifest['mapping_artifact']['source_map']
        require(map_manifest['path'] == protected['v17_map']['path']
                and map_manifest['sha256'] == protected['v17_map']['sha256']
                and int(map_manifest['bytes']) == int(
                    protected['v17_map']['bytes']),
                f"{source['sequence_id']} protected map manifest differs")
        require(run_manifest['completion']['trajectory_complete'] is True,
                f"{source['sequence_id']} protected v17 state is incomplete")
        summaries.append({
            'sequence_id': source['sequence_id'],
            'bag_sha256': bag['sha256'],
            'lidar_serialized_sha256': source[
                'serialized_stream_sha256']['lidar'],
            'imu_serialized_sha256': source[
                'serialized_stream_sha256']['imu'],
            'maximum_receive_minus_header_ns': maximum_delay,
            'protected_state_sha256': protected['v17_state']['sha256'],
            'protected_map_sha256': protected['v17_map']['sha256'],
        })
    return summaries


def imported_roots(tree: ast.AST) -> set[str]:
    result = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            result.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            result.add(node.module.split('.')[0])
    return result


def class_and_function_names(tree: ast.Module) -> tuple[set[str], set[str]]:
    classes = {node.name for node in tree.body if isinstance(node, ast.ClassDef)}
    functions = {node.name for node in tree.body
                 if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    return classes, functions


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ContractError(f'cannot load module: {path}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _fake_header(stamp_ns: int, frame_id: str) -> Any:
    return SimpleNamespace(
        stamp=SimpleNamespace(
            sec=int(stamp_ns) // 1_000_000_000,
            nanosec=int(stamp_ns) % 1_000_000_000),
        frame_id=frame_id)


def _fake_fields(message_contract: Mapping[str, Any]) -> list[Any]:
    return [SimpleNamespace(**item)
            for item in message_contract['required_lidar_fields']]


def run_smoke_probes(
        adapter: Any, core: Any, contract: Mapping[str, Any]) -> dict[str, Any]:
    probes: dict[str, Any] = {}
    source = contract['source_binding']['sequences'][0]
    message_contract = contract['messages']
    vector = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
    imu_message = SimpleNamespace(
        header=_fake_header(12_345_678_901, source['imu_frame']),
        angular_velocity=vector(0.1, -0.2, 0.3),
        linear_acceleration=vector(1.0, 2.0, 9.5))
    imu = adapter.decode_imu_message(
        imu_message, source, core, 7, 256,
        int(message_contract['maximum_message_bytes']))
    require(imu.timestamp_ns == 12_345_678_901
            and imu.source_index == 7
            and imu.angular_velocity_B_rad_s == (0.1, -0.2, 0.3),
            'synthetic IMU decode differs')
    probes['imu_decode_exact'] = {
        'status': 'PASS', 'timestamp_ns': imu.timestamp_ns,
        'source_index': imu.source_index}

    dtype = adapter.point_dtype(message_contract)
    values = np.zeros(2, dtype=dtype)
    values['x'] = [1.0, 2.0]
    values['y'] = [-1.0, -2.0]
    values['z'] = [0.5, 0.75]
    values['t'] = [0, 100_000_000]
    values['ring'] = [2, 3]
    point_message = SimpleNamespace(
        header=_fake_header(20_000_000_000, source['lidar_frame']),
        height=1, width=2, fields=_fake_fields(message_contract),
        is_bigendian=False, point_step=48, row_step=96,
        data=values.view(np.uint8), is_dense=True)
    scan = adapter.decode_lidar_message(
        point_message, source, message_contract, core, 3, 512,
        int(message_contract['maximum_message_bytes']))
    require(scan.scan_index == 3 and scan.end_ns == 20_100_000_000
            and len(scan.points) == 2 and scan.points[1].ring == 3,
            'synthetic PointCloud2 decode differs')
    probes['pointcloud_decode_exact'] = {
        'status': 'PASS', 'point_count': len(scan.points),
        'scan_end_ns': scan.end_ns}

    malformed = SimpleNamespace(**vars(point_message))
    malformed.fields = list(malformed.fields[:-1])
    rejected = False
    try:
        adapter.decode_lidar_message(
            malformed, source, message_contract, core, 4, 512,
            int(message_contract['maximum_message_bytes']))
    except adapter.ContractError:
        rejected = True
    require(rejected, 'malformed PointCloud2 schema was accepted')
    probes['pointcloud_schema_rejection'] = {
        'status': 'PASS', 'rejected': rejected}

    rejected = False
    try:
        adapter.decode_imu_message(
            imu_message, source, core, 8,
            int(message_contract['maximum_message_bytes']) + 1,
            int(message_contract['maximum_message_bytes']))
    except adapter.ContractError:
        rejected = True
    require(rejected, 'oversized synthetic message was accepted')
    probes['message_capacity_rejection'] = {
        'status': 'PASS', 'rejected': rejected}

    reorder = adapter.DeterministicEventReorder(30, 8, 1024)
    events = [
        adapter.BufferedSensorMessage((100, 1, 0), 120, 'lidar', 0, 10, object()),
        adapter.BufferedSensorMessage((95, 0, 0), 121, 'imu', 0, 10, object()),
        adapter.BufferedSensorMessage((100, 0, 1), 125, 'imu', 1, 10, object()),
    ]
    emitted = []
    for event in events:
        emitted.extend(reorder.push(event))
    emitted.extend(reorder.finish())
    keys = [item.sort_key for item in emitted]
    require(keys == [(95, 0, 0), (100, 0, 1), (100, 1, 0)],
            'header-time watermark ordering differs')
    probes['header_time_watermark_order'] = {
        'status': 'PASS', 'ordered_keys': [list(item) for item in keys]}

    bounded = adapter.DeterministicEventReorder(100, 1, 10)
    bounded.push(adapter.BufferedSensorMessage(
        (10, 0, 0), 10, 'imu', 0, 10, object()))
    rejected = False
    try:
        bounded.push(adapter.BufferedSensorMessage(
            (11, 0, 1), 11, 'imu', 1, 1, object()))
    except adapter.CapacityError:
        rejected = True
    require(rejected, 'reorder capacity overflow was accepted')
    probes['reorder_capacity_rejection'] = {
        'status': 'PASS', 'rejected': rejected}

    extrinsic = adapter.body_from_lidar(source, core)
    rotation, translation = extrinsic.arrays()
    require(np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-14)
            and math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-14)
            and np.allclose(
                translation,
                source['calibration']['body_from_lidar']['translation_m'],
                atol=0.0, rtol=0.0),
            'synthetic extrinsic decode differs')
    probes['extrinsic_decode_exact'] = {
        'status': 'PASS', 'determinant': float(np.linalg.det(rotation))}

    authorization = {
        'audit': contract['authorization']['required_aggregate_audit'],
        'contract_id': contract['contract_id'],
        'contract_sha256': 'a' * 64,
        'adapter_sha256': contract['adapter']['sha256'],
        'auditor_sha256': contract['static_auditor']['sha256'],
        'status': 'PASS',
        'decision': 'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION',
        'deterministic': {'validated': True},
        'raw_shadow_replay_execution_authorized': True,
        'raw_replay_executed': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
        'ros_publication_authorized': False,
    }
    authorization['aggregate_payload_sha256'] = adapter.payload_sha256(
        authorization['deterministic'])
    adapter.validate_authorization_payload(
        authorization, contract, 'a' * 64, contract['adapter']['sha256'])
    changed = dict(authorization)
    changed['raw_shadow_replay_execution_authorized'] = False
    rejected = False
    try:
        adapter.validate_authorization_payload(
            changed, contract, 'a' * 64, contract['adapter']['sha256'])
    except adapter.ContractError:
        rejected = True
    require(rejected, 'closed authorization was accepted')
    probes['authorization_fail_closed'] = {
        'status': 'PASS', 'closed_authorization_rejected': rejected}

    protected = {'v17_map': '1' * 64, 'v17_state': '2' * 64}
    guard = core.ProtectedOutputGuard(protected)
    guard.verify(protected)
    rejected = False
    try:
        guard.verify({'v17_map': '3' * 64, 'v17_state': '2' * 64})
    except core.ContractViolation:
        rejected = True
    require(rejected, 'protected mutation was accepted')
    probes['protected_identity_fail_closed'] = {
        'status': 'PASS', 'mutation_rejected': rejected}

    with tempfile.TemporaryDirectory(prefix='v44e-static-') as directory:
        root = Path(directory).resolve()
        run = root / 'raw_replay' / 'synthetic' / 'run_01'
        writer = adapter.BoundedEvidenceWriter(run, root, 4096)
        writer.write_diagnostics([{'record_type': 'scan', 'scan_index': 0}])
        writer.write_report({'status': 'PASS'})
        rejected = False
        try:
            writer.write_report({'status': 'PASS'})
        except adapter.ContractError:
            rejected = True
        require(rejected
                and sorted(item.name for item in run.iterdir()) == [
                    'diagnostics.jsonl', 'run.json'],
                'bounded exclusive writer probe failed')
        probes['bounded_exclusive_writer'] = {
            'status': 'PASS', 'overwrite_rejected': rejected,
            'written_bytes': writer.written_bytes}
    require(tuple(probes) == EXPECTED_PROBES,
            'synthetic probe result order differs')
    return probes


def static_audit(
        contract: Mapping[str, Any], source_manifest: Mapping[str, Any],
        v44a_aggregate: Mapping[str, Any], architecture: Mapping[str, Any],
        memory: MemoryGuard) -> tuple[dict[str, bool], dict[str, Any], dict[str, Any]]:
    checks = {identifier: False for identifier in EXPECTED_STATIC_CHECKS}

    def passed(identifier: str, condition: bool, message: str) -> None:
        require(identifier in checks, f'unknown static check: {identifier}')
        require(condition, message)
        checks[identifier] = True

    passed('v44d_contract_hash_id_and_boundary', True,
           'v44d prerequisite contract failed')
    passed('v44d_aggregate_hash_payload_and_decision', True,
           'v44d prerequisite aggregate failed')
    passed('v44a_contract_manifest_and_aggregate_bound', True,
           'v44a prerequisites failed')
    passed('architecture_and_estimator_core_bound',
           architecture['resource_bounds']['maximum_rss_mib'] ==
           contract['runtime_resources']['maximum_rss_mib']
           and architecture['resource_bounds']['maximum_processing_rtf'] ==
           contract['runtime_resources']['maximum_processing_rtf']
           and architecture['resource_bounds']['maximum_input_message_bytes'] ==
           contract['runtime_resources']['maximum_input_message_bytes']
           and architecture['resource_bounds']['maximum_diagnostic_output_bytes'] ==
           contract['output']['maximum_bytes_per_run'],
           'v44e runtime resources differ from v44b')
    source_summaries = validate_sources(
        contract, source_manifest, v44a_aggregate)
    passed('three_source_records_exactly_match_manifest',
           len(source_summaries) == 3,
           'v44e source records differ from manifest')
    passed('three_bag_paths_sizes_and_hashes_explicit',
           all(len(item['bag_sha256']) == 64 for item in source_summaries),
           'v44e bag binding is incomplete')
    passed('readiness_reports_bind_stream_digests_and_delays',
           all(item['maximum_receive_minus_header_ns'] >= 0
               and len(item['lidar_serialized_sha256']) == 64
               and len(item['imu_serialized_sha256']) == 64
               for item in source_summaries),
           'v44e readiness stream binding is incomplete')
    passed('sensor_adapters_and_extrinsics_exact', True,
           'v44e sensor calibration differs')
    passed('protected_v17_state_and_map_artifacts_exact',
           all(len(item['protected_state_sha256']) == 64
               and len(item['protected_map_sha256']) == 64
               for item in source_summaries),
           'v44e protected v17 artifacts differ')
    memory.check('after_source_bindings')

    adapter_path = resolve_path(contract['adapter']['path'])
    source = adapter_path.read_text(encoding='utf-8')
    try:
        tree = ast.parse(source, filename=str(adapter_path))
    except SyntaxError as error:
        raise ContractError('v44e adapter does not parse') from error
    nodes = list(ast.walk(tree))
    require(len(nodes) <= int(contract['audit_resources']['maximum_ast_nodes']),
            'v44e adapter AST exceeds node capacity')
    passed('adapter_hash_path_and_size',
           sha256_file(adapter_path) == contract['adapter']['sha256']
           and adapter_path.stat().st_size <= int(
               contract['adapter']['maximum_source_bytes']),
           'v44e adapter hash or size differs')
    passed('adapter_python_ast_parse', isinstance(tree, ast.Module),
           'v44e adapter AST root differs')
    imports = imported_roots(tree)
    allowed = set(contract['adapter']['allowed_import_roots'])
    passed('adapter_imports_are_allowlisted', imports <= allowed,
           f'v44e adapter imports outside allowlist: {sorted(imports - allowed)}')
    rosbags_nodes = [node for node in ast.walk(tree)
                     if isinstance(node, ast.ImportFrom)
                     and node.module and node.module.startswith('rosbags')]
    passed('rosbags_import_is_lazy', len(rosbags_nodes) == 1
           and not any(node in tree.body for node in rosbags_nodes),
           'v44e rosbags import must occur once inside raw execution')
    passed('no_ros_network_or_subprocess_surface',
           imports.isdisjoint({
               'rclpy', 'rospy', 'sensor_msgs', 'geometry_msgs', 'socket',
               'requests', 'urllib', 'subprocess', 'multiprocessing',
               'concurrent'}),
           'v44e adapter contains ROS, network, or subprocess surface')
    parser = load_module(
        adapter_path, f'v44e_adapter_audit_{contract["adapter"]["sha256"][:12]}')
    cli = parser.build_parser()
    choices = cli._subparsers._group_actions[0].choices
    replay_actions = choices['replay']._actions
    option_strings = {option for action in replay_actions
                      for option in action.option_strings}
    passed('cli_has_only_contract_sequence_and_repetition',
           set(choices) == {'replay'} and option_strings == {
               '-h', '--help', '--contract', '--sequence-id', '--repetition'},
           'v44e CLI option inventory differs')
    passed('no_user_selected_bag_or_output_path',
           '--bag' not in source and '--output' not in source
           and '--trajectory' not in source,
           'v44e exposes a user-selected input/output path')
    load_context = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef)
                        and node.name == 'load_runtime_context')
    stream_function = next(node for node in tree.body
                           if isinstance(node, ast.FunctionDef)
                           and node.name == 'stream_raw_bag')
    passed('authorization_aggregate_required_before_bag_open',
           source.find('validate_authorization_payload') <
           source.find('from rosbags.highlevel import AnyReader')
           and source.find('validate_authorization_source_reports') <
           source.find('from rosbags.highlevel import AnyReader')
           and load_context.lineno < stream_function.lineno,
           'v44e authorization does not precede raw-bag open')
    passed('authority_is_report_only_and_fail_closed',
           parser.ADAPTER_AUTHORITY == contract['authority'],
           'v44e adapter authority differs')
    classes, functions = class_and_function_names(tree)
    passed('required_adapter_symbols_present',
           EXPECTED_ADAPTER_CLASSES <= classes
           and EXPECTED_ADAPTER_FUNCTIONS <= functions,
           'v44e required adapter symbol is absent')
    passed('canonical_lidar_and_imu_decoder_present',
           all(marker in source for marker in (
               'required_lidar_fields', 'lidar_point_step_bytes',
               'header_stamp_ns', 'angular_velocity_B_rad_s',
               'linear_acceleration_B_m_s2', 'LidarPoint', 'LidarScan')),
           'v44e canonical decoder path is incomplete')
    attribute_names = {node.attr for node in nodes if isinstance(node, ast.Attribute)}
    passed('orientation_and_covariance_are_not_consumed',
           attribute_names.isdisjoint({
               'orientation', 'orientation_covariance',
               'angular_velocity_covariance', 'linear_acceleration_covariance'}),
           'v44e adapter consumes orientation or covariance')
    passed('bounded_header_time_reorder_present',
           all(marker in source for marker in (
               'DeterministicEventReorder', 'maximum_receive_delay_ns',
               'maximum_buffered_messages', 'maximum_buffered_bytes',
               'watermark', 'EVENT_KIND_ORDER')),
           'v44e bounded header-time reorder is incomplete')
    passed('per_scan_rss_and_rtf_enforcement_present',
           all(marker in source for marker in (
               'record_runtime_after_scans', 'current_rss_mib',
               'processing_seconds', 'sensor_duration_seconds')),
           'v44e per-scan resource enforcement is incomplete')
    passed('protected_hashes_compared_before_and_after',
           source.count('hash_protected_artifacts(source)') == 2
           and 'protected_before == protected_after' in source,
           'v44e protected before/after hash check is incomplete')
    passed('exclusive_bounded_evidence_writer_present',
           all(marker in source for marker in (
               "path.open('xb')", 'maximum_bytes',
               'DIAGNOSTIC_FILENAME', 'RUN_REPORT_FILENAME',
               'diagnostic output overwrite was refused')),
           'v44e bounded evidence writer is incomplete')
    call_attributes = {node.func.attr for node in nodes
                       if isinstance(node, ast.Call)
                       and isinstance(node.func, ast.Attribute)}
    passed('core_receives_no_dataset_identity',
           'sequence_id' not in source[source.find('def body_from_lidar'):
                                       source.find('def hash_protected_artifacts')]
           and call_attributes.isdisjoint({'publish', 'create_publisher'}),
           'v44e core receives dataset identity or publication callback')
    passed('accuracy_reference_ros_and_primary_routes_closed',
           parser.ADAPTER_AUTHORITY['accuracy_or_reference_map_inputs'] is False
           and parser.ADAPTER_AUTHORITY[
               'primary_trajectory_or_map_mutation'] is False
           and parser.ADAPTER_AUTHORITY['ros_publication'] is False,
           'v44e forbidden route is open')
    passed('static_gate_does_not_open_raw_bags',
           'rosbags' not in imported_roots(ast.parse(
               Path(__file__).read_text(encoding='utf-8'))),
           'v44e static auditor imports raw-bag support')
    core = load_module(
        resolve_path(contract['estimator_core']['path']),
        f'v44e_core_audit_{contract["estimator_core"]["sha256"][:12]}')
    probes = run_smoke_probes(parser, core, contract)
    passed('synthetic_adapter_probes_pass',
           tuple(probes) == EXPECTED_PROBES
           and all(item['status'] == 'PASS' for item in probes.values()),
           'v44e synthetic adapter probe failed')
    require(all(checks.values()), 'one or more v44e static checks failed')
    memory.check('after_static_audit')
    metrics = {
        'adapter_source_bytes': adapter_path.stat().st_size,
        'adapter_ast_nodes': len(nodes),
        'adapter_import_roots': sorted(imports),
        'adapter_class_count': len(classes),
        'adapter_module_function_count': len(functions),
        'source_sequence_count': len(source_summaries),
        'protected_artifact_count': 2 * len(source_summaries),
    }
    return checks, metrics, {'probes': probes, 'sources': source_summaries}


def write_json_bounded(path: Path, value: Mapping[str, Any], maximum: int) -> None:
    encoded = (json.dumps(
        dict(value), indent=2, sort_keys=True, allow_nan=False)
        + '\n').encode('utf-8')
    if len(encoded) > int(maximum):
        raise MemoryBudgetError('v44e report exceeds byte capacity')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)


def validate_once(
        contract_path: Path, repetition: int, output: Path) -> dict[str, Any]:
    contract, contract_digest, source_manifest, aggregate, architecture = (
        load_and_validate_contract(contract_path))
    required = int(contract['decision']['required_static_repetitions'])
    require(1 <= int(repetition) <= required,
            'v44e static repetition is outside contract')
    resources = contract['audit_resources']
    memory = MemoryGuard(
        resources['maximum_rss_mib'],
        resources['maximum_incremental_rss_mib'])
    memory.check('start')
    checks, metrics, details = static_audit(
        contract, source_manifest, aggregate, architecture, memory)
    deterministic = {
        'prerequisite': {
            'v44d_contract_sha256': contract['prerequisites'][
                'v44d_contract']['sha256'],
            'v44d_aggregate_sha256': contract['prerequisites'][
                'v44d_aggregate']['sha256'],
            'v44d_decision': contract['prerequisites'][
                'v44d_aggregate']['required_decision'],
            'v44a_source_manifest_sha256': contract['prerequisites'][
                'v44a_source_manifest']['sha256'],
            'v44a_aggregate_sha256': contract['prerequisites'][
                'v44a_aggregate']['sha256'],
        },
        'adapter_sha256': contract['adapter']['sha256'],
        'estimator_core_sha256': contract['estimator_core']['sha256'],
        'static_checks': checks,
        'static_metrics': metrics,
        'smoke_probes': details['probes'],
        'source_bindings': details['sources'],
        'raw_replay_executed': False,
        'raw_bag_opened': False,
        'raw_shadow_replay_execution_contract_validated': True,
        'authority': dict(contract['authority']),
    }
    report = {
        'schema_version': 1,
        'audit': 'v44e_raw_shadow_replay_contract_static_validation',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'adapter_sha256': contract['adapter']['sha256'],
        'auditor_sha256': sha256_file(Path(__file__).resolve()),
        'repetition': int(repetition),
        'status': 'PASS',
        'decision': contract['decision']['on_pass'],
        'deterministic': deterministic,
        'report_payload_sha256': payload_sha256(deterministic),
        'resource_usage': {
            'baseline_rss_mib': memory.baseline_rss_mib,
            'peak_rss_mib': memory.peak_rss_mib,
            'peak_incremental_rss_mib': (
                memory.peak_rss_mib - memory.baseline_rss_mib),
            'maximum_rss_mib': memory.maximum_rss_mib,
            'maximum_incremental_rss_mib': memory.maximum_incremental_rss_mib,
        },
        'raw_shadow_replay_execution_authorized': True,
        'raw_replay_executed': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
        'ros_publication_authorized': False,
    }
    write_json_bounded(
        output, report, int(resources['maximum_report_bytes']))
    return report


def validate_source_report(
        report: Mapping[str, Any], contract: Mapping[str, Any],
        contract_digest: str) -> None:
    require(report.get('audit') ==
            'v44e_raw_shadow_replay_contract_static_validation',
            'v44e source report audit ID differs')
    require(report.get('contract_id') == contract['contract_id']
            and report.get('contract_sha256') == contract_digest,
            'v44e source report contract binding differs')
    require(report.get('adapter_sha256') == contract['adapter']['sha256'],
            'v44e source report adapter hash differs')
    require(report.get('auditor_sha256') == contract['static_auditor']['sha256'],
            'v44e source report auditor hash differs')
    require(report.get('status') == 'PASS'
            and report.get('decision') == contract['decision']['on_pass'],
            'v44e source report did not pass')
    deterministic = report.get('deterministic')
    require(isinstance(deterministic, dict)
            and report.get('report_payload_sha256') ==
            payload_sha256(deterministic),
            'v44e source report payload differs')
    require(set(deterministic.get('static_checks', {})) ==
            set(EXPECTED_STATIC_CHECKS)
            and len(deterministic.get('static_checks', {})) ==
            len(EXPECTED_STATIC_CHECKS)
            and all(deterministic['static_checks'].values()),
            'v44e source report static checks differ')
    require(set(deterministic.get('smoke_probes', {})) == set(EXPECTED_PROBES)
            and len(deterministic.get('smoke_probes', {})) ==
            len(EXPECTED_PROBES)
            and all(item.get('status') == 'PASS'
                    for item in deterministic['smoke_probes'].values()),
            'v44e source report smoke probes differ')
    require(deterministic.get('raw_replay_executed') is False
            and deterministic.get('raw_bag_opened') is False,
            'v44e static report unexpectedly opened or replayed raw data')
    require(report.get('raw_shadow_replay_execution_authorized') is True,
            'v44e source report did not authorize bounded replay')
    for key in (
            'accuracy_or_reference_map_inputs_authorized',
            'primary_trajectory_or_map_mutation_authorized',
            'ros_publication_authorized'):
        require(report.get(key) is False,
                f'v44e source report unexpectedly opens {key}')


def aggregate_reports(
        contract_path: Path, reports: list[Path], output: Path,
        markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_digest, _, _, _ = load_and_validate_contract(contract_path)
    required = int(contract['decision']['required_static_repetitions'])
    require(len(reports) == required,
            f'v44e aggregate requires exactly {required} reports')
    loaded = []
    for path_value in reports:
        path = resolve_path(path_value)
        report = load_json(path)
        validate_source_report(report, contract, contract_digest)
        loaded.append((path, report))
    repetitions = sorted(int(item[1]['repetition']) for item in loaded)
    require(repetitions == list(range(1, required + 1)),
            'v44e static repetitions are incomplete or duplicated')
    payloads = {item[1]['report_payload_sha256'] for item in loaded}
    require(len(payloads) == 1,
            'v44e static validation is not deterministic')
    deterministic = {
        'validation_complete': True,
        'validation_repeatable': True,
        'validation_repetition_count': required,
        'report_payload_sha256': next(iter(payloads)),
        'static_check_count': len(EXPECTED_STATIC_CHECKS),
        'smoke_probe_count': len(EXPECTED_PROBES),
        'adapter_sha256': contract['adapter']['sha256'],
        'estimator_core_sha256': contract['estimator_core']['sha256'],
        'source_sequence_count': 3,
        'raw_replay_executed': False,
        'raw_bag_opened': False,
    }
    aggregate = {
        'schema_version': 1,
        'audit': 'v44e_raw_shadow_replay_contract_static_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'adapter_sha256': contract['adapter']['sha256'],
        'auditor_sha256': contract['static_auditor']['sha256'],
        'status': 'PASS',
        'decision': contract['decision']['on_pass'],
        'deterministic': deterministic,
        'aggregate_payload_sha256': payload_sha256(deterministic),
        'source_reports': [
            {'path': str(path), 'sha256': sha256_file(path)}
            for path, _ in loaded],
        'raw_shadow_replay_execution_authorized': True,
        'raw_replay_executed': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
        'ros_publication_authorized': False,
    }
    maximum = int(contract['audit_resources']['maximum_report_bytes'])
    write_json_bounded(output, aggregate, maximum)
    if markdown_output is not None:
        lines = [
            '# v44e raw shadow replay execution-contract static audit', '',
            f"- status: `{aggregate['status']}`",
            f"- decision: `{aggregate['decision']}`",
            f"- contract SHA-256: `{contract_digest}`",
            f"- adapter SHA-256: `{contract['adapter']['sha256']}`",
            f"- auditor SHA-256: `{contract['static_auditor']['sha256']}`",
            f"- deterministic report payload SHA-256: `{next(iter(payloads))}`",
            f"- aggregate payload SHA-256: `{aggregate['aggregate_payload_sha256']}`",
            f"- static checks: `{len(EXPECTED_STATIC_CHECKS)}`",
            f"- synthetic adapter probes: `{len(EXPECTED_PROBES)}`",
            '- raw bags opened/replayed by this gate: `false`',
            '- bounded report-only raw replay execution authorized: `true`',
            '- accuracy/reference-map input, primary mutation, ROS output: `false`',
            '',
        ]
        encoded = '\n'.join(lines).encode('utf-8')
        if len(encoded) > maximum:
            raise MemoryBudgetError('v44e markdown report exceeds byte capacity')
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
