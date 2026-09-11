#!/usr/bin/env python3
"""Audit retained independent place-identity channels without changing SLAM output."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import resource
import time
from typing import Any, Iterable

import numpy as np


class ContractError(ValueError):
    """Raised when a retained input violates the frozen v43 contract."""


class MemoryBudgetError(RuntimeError):
    """Raised when continuing would exceed a frozen resource bound."""


def canonical_json(payload: Any) -> str:
    return json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(payload: Any) -> str:
    return hashlib.sha256(canonical_json(payload).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.peak_rss_mib = current_rss_mib()

    def check(self, label: str) -> None:
        rss = current_rss_mib()
        self.peak_rss_mib = max(self.peak_rss_mib, rss)
        if rss > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS {rss:.3f} MiB exceeds {self.maximum_rss_mib:.3f} MiB at {label}')


def load_contract(path: Path) -> tuple[dict[str, Any], str]:
    contract = json.loads(path.read_text(encoding='utf-8'))
    required = {
        'schema_version', 'contract_id', 'frames', 'intensity',
        'camera_manifest', 'memory', 'decision',
    }
    missing = sorted(required - set(contract))
    if missing:
        raise ContractError(f'contract missing keys: {missing}')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported contract schema_version')
    if contract['frames'] != {'world': 'map', 'body': 'base_link'}:
        raise ContractError('v43 frame identity must be map <- base_link')
    intensity = contract['intensity']
    if intensity['required_fields'] != ['x', 'y', 'z', 'intensity']:
        raise ContractError('v43 requires exact XYZI fields')
    if int(intensity['minimum_distinct_values']) > int(intensity['distinct_value_cap']):
        raise ContractError('distinct_value_cap is below the pass threshold')
    if float(contract['memory']['maximum_rss_mib']) <= 0.0:
        raise ContractError('maximum_rss_mib must be positive')
    return contract, sha256_file(path)


def load_state_count(path: Path) -> int:
    previous = -math.inf
    count = 0
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) < 8:
            raise ContractError(f'{path}:{line_number}: expected at least 8 fields')
        try:
            values = [float(value) for value in fields[:8]]
        except ValueError as error:
            raise ContractError(f'{path}:{line_number}: non-numeric state') from error
        if not all(math.isfinite(value) for value in values):
            raise ContractError(f'{path}:{line_number}: non-finite state')
        if values[0] <= previous:
            raise ContractError(f'{path}:{line_number}: timestamps are not increasing')
        quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
        if quaternion_norm <= 1e-12:
            raise ContractError(f'{path}:{line_number}: zero quaternion')
        previous = values[0]
        count += 1
    if count == 0:
        raise ContractError(f'{path}: no states')
    return count


def indexed_chunks(directory: Path, expected: int) -> list[Path]:
    try:
        chunks = sorted(directory.glob('*.pcd'), key=lambda path: int(path.stem))
        indices = [int(path.stem) for path in chunks]
    except ValueError as error:
        raise ContractError(f'{directory}: non-numeric PCD name') from error
    if indices != list(range(expected)):
        raise ContractError(f'{directory}: chunks are not contiguous for {expected} states')
    return chunks


def pcd_intensity(path: Path, maximum_chunk_bytes: int) -> np.ndarray:
    fields: dict[str, list[str]] = {}
    with path.open('rb') as stream:
        for _ in range(64):
            raw = stream.readline()
            if not raw:
                raise ContractError(f'{path}: incomplete PCD header')
            parts = raw.decode('ascii', errors='strict').split()
            if parts:
                fields[parts[0]] = parts[1:]
            if raw.strip() == b'DATA binary':
                offset = stream.tell()
                break
        else:
            raise ContractError(f'{path}: PCD header is too long')
    expected = (
        ['x', 'y', 'z', 'intensity'], ['4'] * 4, ['F'] * 4,
        ['1'] * 4, ['binary'])
    actual = tuple(fields.get(key) for key in ('FIELDS', 'SIZE', 'TYPE', 'COUNT', 'DATA'))
    if actual != expected or 'POINTS' not in fields:
        raise ContractError(f'{path}: expected packed float32 XYZI binary PCD')
    points = int(fields['POINTS'][0])
    payload_bytes = points * 16
    if payload_bytes > int(maximum_chunk_bytes):
        raise MemoryBudgetError(
            f'{path}: {payload_bytes} bytes exceeds maximum_chunk_bytes')
    if path.stat().st_size - offset != payload_bytes:
        raise ContractError(f'{path}: PCD byte count differs from POINTS')
    values = np.fromfile(path, dtype='<f4', count=points * 4, offset=offset)
    if values.size != points * 4:
        raise ContractError(f'{path}: truncated PCD payload')
    return values.reshape(points, 4)[:, 3]


class IntensityAccumulator:
    """Bounded streaming statistics and exact intensity-channel digest."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.chunk_count = 0
        self.point_count = 0
        self.finite_count = 0
        self.nonzero_count = 0
        self.varying_chunk_count = 0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.total = 0.0
        self.total_squared = 0.0
        self.distinct: set[int] = set()
        self.digest = hashlib.sha256()
        self.peak_chunk_bytes = 0

    def add(self, intensity: np.ndarray) -> None:
        values = np.asarray(intensity, dtype='<f4')
        self.chunk_count += 1
        self.point_count += int(len(values))
        self.peak_chunk_bytes = max(self.peak_chunk_bytes, int(values.nbytes))
        self.digest.update(values.tobytes(order='C'))
        finite_mask = np.isfinite(values)
        finite = values[finite_mask]
        self.finite_count += int(len(finite))
        if not len(finite):
            return
        local_minimum = float(np.min(finite))
        local_maximum = float(np.max(finite))
        self.minimum = min(self.minimum, local_minimum)
        self.maximum = max(self.maximum, local_maximum)
        epsilon = float(self.config['nonzero_epsilon'])
        self.nonzero_count += int(np.count_nonzero(np.abs(finite) > epsilon))
        if local_maximum - local_minimum >= float(self.config['minimum_dynamic_range']):
            self.varying_chunk_count += 1
        self.total += float(np.sum(finite, dtype=np.float64))
        as_double = finite.astype(np.float64)
        self.total_squared += float(np.dot(as_double, as_double))
        cap = int(self.config['distinct_value_cap'])
        if len(self.distinct) < cap:
            sample_count = min(int(self.config['distinct_samples_per_chunk']), len(finite))
            indices = np.linspace(0, len(finite) - 1, sample_count, dtype=np.int64)
            bits = finite[indices].view(np.uint32)
            for value in bits:
                self.distinct.add(int(value))
                if len(self.distinct) >= cap:
                    break

    def result(self) -> dict[str, Any]:
        finite_fraction = self.finite_count / max(1, self.point_count)
        nonzero_fraction = self.nonzero_count / max(1, self.finite_count)
        varying_fraction = self.varying_chunk_count / max(1, self.chunk_count)
        dynamic_range = (
            self.maximum - self.minimum if self.finite_count else 0.0)
        mean = self.total / max(1, self.finite_count)
        variance = max(
            0.0, self.total_squared / max(1, self.finite_count) - mean * mean)
        checks = {
            'finite_fraction': finite_fraction >= float(
                self.config['minimum_finite_fraction']),
            'nonzero_fraction': nonzero_fraction >= float(
                self.config['minimum_nonzero_fraction']),
            'dynamic_range': dynamic_range >= float(
                self.config['minimum_dynamic_range']),
            'distinct_values': len(self.distinct) >= int(
                self.config['minimum_distinct_values']),
            'varying_chunk_fraction': varying_fraction >= float(
                self.config['minimum_varying_chunk_fraction']),
        }
        return {
            'available': all(checks.values()),
            'checks': checks,
            'chunk_count': self.chunk_count,
            'point_count': self.point_count,
            'finite_count': self.finite_count,
            'finite_fraction': finite_fraction,
            'nonzero_count': self.nonzero_count,
            'nonzero_fraction': nonzero_fraction,
            'minimum': self.minimum if self.finite_count else None,
            'maximum': self.maximum if self.finite_count else None,
            'dynamic_range': dynamic_range,
            'mean': mean if self.finite_count else None,
            'standard_deviation': math.sqrt(variance) if self.finite_count else None,
            'distinct_values_observed_up_to_cap': len(self.distinct),
            'varying_chunk_count': self.varying_chunk_count,
            'varying_chunk_fraction': varying_fraction,
            'intensity_payload_sha256': self.digest.hexdigest(),
            'peak_chunk_bytes': self.peak_chunk_bytes,
        }


def camera_manifest_status(path: Path | None, sequence_id: str,
                           config: dict[str, Any]) -> dict[str, Any]:
    if path is None:
        return {
            'available': False,
            'reason': 'camera_manifest_not_retained',
        }
    if not path.is_file():
        return {
            'available': False,
            'reason': 'camera_manifest_path_missing',
        }
    manifest = json.loads(path.read_text(encoding='utf-8'))
    reasons = []
    if manifest.get('schema_version') != int(config['schema_version']):
        reasons.append('schema_version')
    if manifest.get('sequence_id') != sequence_id:
        reasons.append('sequence_id')
    if int(manifest.get('frame_count', 0)) < int(config['minimum_frame_count']):
        reasons.append('frame_count')
    for field in config['required_hash_fields']:
        value = manifest.get(field)
        if not isinstance(value, str) or len(value) != 64:
            reasons.append(field)
    return {
        'available': not reasons,
        'reason': None if not reasons else 'invalid_camera_manifest',
        'failed_checks': sorted(reasons),
        'frame_count': int(manifest.get('frame_count', 0)),
        'manifest_sha256': sha256_file(path),
    }


def chunk_manifest(chunks: Iterable[Path]) -> dict[str, Any]:
    entries = [(path.name, path.stat().st_size) for path in chunks]
    return {
        'count': len(entries),
        'total_bytes': int(sum(size for _, size in entries)),
        'name_size_manifest_sha256': payload_sha256(entries),
    }


def audit_sequence(*, contract_path: Path, voxel_dir: Path, sequence_id: str,
                   repetition: int, output: Path, protected_map: Path | None,
                   camera_manifest: Path | None) -> dict[str, Any]:
    started = time.monotonic()
    contract, contract_sha = load_contract(contract_path)
    memory = MemoryGuard(float(contract['memory']['maximum_rss_mib']))
    state_path = voxel_dir / 'alidarState.txt'
    report: dict[str, Any] = {
        'schema_version': 1,
        'audit': 'v43_independent_place_identity_availability',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'status': 'FAIL',
    }
    try:
        state_sha_before = sha256_file(state_path)
        map_sha_before = sha256_file(protected_map) if protected_map else None
        state_count = load_state_count(state_path)
        chunks = indexed_chunks(voxel_dir, state_count)
        accumulator = IntensityAccumulator(contract['intensity'])
        maximum_chunk_bytes = int(contract['memory']['maximum_chunk_bytes'])
        for index, path in enumerate(chunks):
            intensity = pcd_intensity(path, maximum_chunk_bytes)
            accumulator.add(intensity)
            del intensity
            if index % 128 == 0:
                memory.check('full_chunk_intensity_scan')
        intensity_result = accumulator.result()
        camera_result = camera_manifest_status(
            camera_manifest, sequence_id, contract['camera_manifest'])
        independent_signal = bool(
            intensity_result['available'] or camera_result['available'])
        state_sha_after = sha256_file(state_path)
        map_sha_after = sha256_file(protected_map) if protected_map else None
        if state_sha_before != state_sha_after or map_sha_before != map_sha_after:
            raise ContractError('protected v17 input changed during report-only audit')
        deterministic = {
            'sequence_id': sequence_id,
            'contract_id': contract['contract_id'],
            'contract_sha256': contract_sha,
            'input': {
                'state_count': state_count,
                'state_sha256': state_sha_before,
                'chunk_manifest': chunk_manifest(chunks),
                'protected_map_sha256': map_sha_before,
            },
            'intensity': intensity_result,
            'camera': camera_result,
            'independent_place_identity_signal_available': independent_signal,
            'place_identity_challenge_status': (
                'AUTHORIZED_FOR_IMPLEMENTATION' if independent_signal
                else 'NOT_RUN_NO_INDEPENDENT_SIGNAL'),
            'protected_inputs_unchanged': True,
        }
        report['deterministic'] = deterministic
        report['deterministic_payload_sha256'] = payload_sha256(deterministic)
        report['status'] = 'PASS'
    except Exception as error:
        report['error_type'] = type(error).__name__
        report['error'] = str(error)
    finally:
        try:
            memory.check('audit_finalize')
        except MemoryBudgetError as error:
            report['status'] = 'FAIL'
            report['error_type'] = type(error).__name__
            report['error'] = str(error)
        report['runtime'] = {
            'wall_seconds': time.monotonic() - started,
            'peak_rss_mib': memory.peak_rss_mib,
            'maximum_rss_mib': memory.maximum_rss_mib,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n',
                          encoding='utf-8')
    return report


def aggregate_reports(*, contract_path: Path, reports: list[Path],
                      expected_sequences: list[str], output: Path,
                      markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_sha = load_contract(contract_path)
    loaded = [(path, json.loads(path.read_text(encoding='utf-8'))) for path in reports]
    groups: dict[str, list[tuple[Path, dict[str, Any]]]] = {
        sequence: [] for sequence in expected_sequences}
    for path, report in loaded:
        sequence = report.get('sequence_id')
        if sequence not in groups:
            raise ContractError(f'unexpected sequence {sequence!r} in {path}')
        groups[sequence].append((path, report))
    required = int(contract['decision']['required_repetitions'])
    complete = True
    repeatable = True
    unavailable = []
    sequence_results = []
    for sequence in expected_sequences:
        items = sorted(groups[sequence], key=lambda item: int(item[1].get('repetition', 0)))
        hashes = [item[1].get('deterministic_payload_sha256') for item in items]
        integrity = [
            report.get('status') == 'PASS'
            and report.get('contract_sha256') == contract_sha
            and report.get('deterministic_payload_sha256') ==
            payload_sha256(report.get('deterministic', {}))
            for _, report in items
        ]
        item_complete = len(items) == required and all(integrity)
        item_repeatable = item_complete and len(set(hashes)) == 1
        complete &= item_complete
        repeatable &= item_repeatable
        deterministic = items[0][1].get('deterministic', {}) if items else {}
        signal = bool(deterministic.get('independent_place_identity_signal_available'))
        if not signal:
            unavailable.append(sequence)
        intensity = deterministic.get('intensity', {})
        camera = deterministic.get('camera', {})
        sequence_results.append({
            'sequence_id': sequence,
            'complete': item_complete,
            'repeatable': item_repeatable,
            'deterministic_payload_sha256': hashes[0] if item_repeatable else None,
            'intensity_available': bool(intensity.get('available')),
            'intensity_point_count': int(intensity.get('point_count', 0)),
            'intensity_nonzero_count': int(intensity.get('nonzero_count', 0)),
            'intensity_dynamic_range': float(intensity.get('dynamic_range', 0.0)),
            'intensity_payload_sha256': intensity.get('intensity_payload_sha256'),
            'camera_available': bool(camera.get('available')),
            'independent_signal_available': signal,
        })
    require_all = bool(
        contract['decision']['require_independent_signal_on_every_sequence'])
    if not complete or not repeatable:
        decision = 'REJECT_V43_INCOMPLETE_OR_NONREPEATABLE_AUDIT'
    elif require_all and unavailable:
        decision = 'CLOSE_OXFORD_GLOBAL_CORRECTION_ROUTE_NO_INDEPENDENT_PLACE_IDENTITY'
    else:
        decision = 'AUTHORIZE_V43_PLACE_IDENTITY_CHALLENGE_IMPLEMENTATION'
    aggregate = {
        'schema_version': 1,
        'audit': 'v43_independent_place_identity_availability_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'status': 'PASS' if complete and repeatable else 'FAIL',
        'decision': decision,
        'place_identity_challenge_authorized': decision.startswith('AUTHORIZE_'),
        'oxford_global_correction_route_open': decision.startswith('AUTHORIZE_'),
        'sequences_without_independent_signal': unavailable,
        'sequence_results': sequence_results,
        'source_reports': [
            {'path': str(path.resolve()), 'sha256': sha256_file(path)} for path, _ in loaded],
    }
    aggregate['aggregate_payload_sha256'] = payload_sha256({
        key: value for key, value in aggregate.items() if key != 'source_reports'})
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if markdown_output:
        lines = [
            '# v43 independent place-identity availability', '',
            f"- decision: `{decision}`",
            f"- challenge authorized: `{str(aggregate['place_identity_challenge_authorized']).lower()}`",
            f"- Oxford global-correction route open: `{str(aggregate['oxford_global_correction_route_open']).lower()}`",
            '', '## Sequences', '',
        ]
        for item in sequence_results:
            lines.append(
                f"- `{item['sequence_id']}`: repeatable={str(item['repeatable']).lower()}, "
                f"points={item['intensity_point_count']}, "
                f"nonzero={item['intensity_nonzero_count']}, "
                f"dynamic_range={item['intensity_dynamic_range']:.6g}, "
                f"camera={str(item['camera_available']).lower()}")
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return aggregate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    audit = subparsers.add_parser('audit')
    audit.add_argument('--contract', type=Path, required=True)
    audit.add_argument('--voxel-dir', type=Path, required=True)
    audit.add_argument('--sequence-id', required=True)
    audit.add_argument('--repetition', type=int, required=True)
    audit.add_argument('--output', type=Path, required=True)
    audit.add_argument('--protected-map', type=Path)
    audit.add_argument('--camera-manifest', type=Path)
    aggregate = subparsers.add_parser('aggregate')
    aggregate.add_argument('--contract', type=Path, required=True)
    aggregate.add_argument('--report', type=Path, action='append', required=True)
    aggregate.add_argument('--expected-sequence', action='append', required=True)
    aggregate.add_argument('--output', type=Path, required=True)
    aggregate.add_argument('--markdown-output', type=Path)
    return parser.parse_args()


def main() -> int:
    options = parse_args()
    if options.command == 'audit':
        report = audit_sequence(
            contract_path=options.contract,
            voxel_dir=options.voxel_dir,
            sequence_id=options.sequence_id,
            repetition=options.repetition,
            output=options.output,
            protected_map=options.protected_map,
            camera_manifest=options.camera_manifest,
        )
        return 0 if report['status'] == 'PASS' else 2
    aggregate = aggregate_reports(
        contract_path=options.contract,
        reports=options.report,
        expected_sequences=options.expected_sequence,
        output=options.output,
        markdown_output=options.markdown_output,
    )
    return 0 if aggregate['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
