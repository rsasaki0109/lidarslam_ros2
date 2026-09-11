#!/usr/bin/env python3
"""Non-destructive, verifiable edits for lidar_slam map bundles."""

from __future__ import annotations

from contextlib import redirect_stdout
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
from typing import Any

from mid360_robot_loop_alignment_analyzer import _decompress_lzf

from verify_autoware_map import MapVerifier, parse_pcd_header

from verify_map_bundle import BundleVerifier

import yaml


EDIT_PLAN_SCHEMA_VERSION = 1
EDIT_PLAN_NAME = 'map_edit_plan.json'
EDIT_RECEIPT_NAME = 'map_edit_receipt.json'
LOOP_HEADER = ['from', 'to', 'fitness', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']


class MapEditError(ValueError):
    """An edit cannot be applied without violating the map-edit contract."""


@dataclass(frozen=True)
class PcdLayout:
    """Validated PCD field layout and payload metadata."""

    fields: list[str]
    sizes: list[int]
    types: list[str]
    counts: list[int]
    points: int
    data: str
    header_bytes: int

    @property
    def field_widths(self) -> list[int]:
        """Return each PCD field's byte width for one point."""
        return [size * count for size, count in zip(self.sizes, self.counts)]

    @property
    def row_size(self) -> int:
        """Return the byte width of one interleaved point record."""
        return sum(self.field_widths)


def sha256_file(path: Path) -> str:
    """Return a stable SHA-256 identity for one artifact."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_directory(path: Path) -> str:
    """Hash relative names and bytes for every regular file in a directory."""
    digest = hashlib.sha256()
    for candidate in sorted(path.rglob('*'), key=lambda item: item.as_posix()):
        if candidate.is_symlink():
            raise MapEditError(
                f'source identity refuses symlinks inside artifact directory: {candidate}'
            )
        if not candidate.is_file():
            continue
        relative = candidate.relative_to(path).as_posix().encode('utf-8')
        digest.update(struct.pack('<Q', len(relative)))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(candidate)))
    return digest.hexdigest()


def resolve_bundle_dir(path: Path) -> Path:
    """Accept either a map bundle or its run-directory parent."""
    resolved = path.expanduser().resolve()
    if (resolved / 'map_bundle.yaml').is_file():
        return resolved
    if (resolved / 'map' / 'map_bundle.yaml').is_file():
        return resolved / 'map'
    raise MapEditError(
        f'map_bundle.yaml was not found under {resolved}; pass a completed map bundle '
        'or its run directory.'
    )


def _reject_source_symlinks(bundle_dir: Path) -> None:
    """Prevent a map edit from importing files outside its source bundle."""
    for candidate in bundle_dir.rglob('*'):
        if candidate.is_symlink():
            raise MapEditError(
                'source map bundle must not contain symlinks: '
                f'{candidate.relative_to(bundle_dir)}'
            )


def load_edit_plan(path: Path) -> dict[str, Any]:
    """Load and validate the versioned map-edit plan."""
    try:
        plan = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise MapEditError(f'cannot read edit plan {path}: {exc}') from exc
    if not isinstance(plan, dict):
        raise MapEditError('edit plan must contain a JSON object')
    if plan.get('schema_version') != EDIT_PLAN_SCHEMA_VERSION:
        raise MapEditError(
            f'unsupported edit-plan schema_version: {plan.get("schema_version")!r}'
        )
    source = plan.get('source')
    if not isinstance(source, dict):
        raise MapEditError('edit plan source must contain an object')
    for name in (
        'map_bundle_sha256',
        'full_map_sha256',
        'pointcloud_map_sha256',
        'loop_edges_sha256',
    ):
        if not _is_sha256(source.get(name)):
            raise MapEditError(f'source.{name} must be a 64-character SHA-256')
    operations = plan.get('operations')
    if not isinstance(operations, list) or not operations:
        raise MapEditError('edit plan must contain at least one operation')
    seen_ids: set[str] = set()
    for index, operation in enumerate(operations):
        _validate_operation(operation, index, seen_ids)
    return plan


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in '0123456789abcdef' for character in value.lower())
    )


def _validate_operation(
    operation: object,
    index: int,
    seen_ids: set[str],
) -> None:
    if not isinstance(operation, dict):
        raise MapEditError(f'operation {index} must contain an object')
    operation_id = operation.get('id')
    if not isinstance(operation_id, str) or not operation_id.strip():
        raise MapEditError(f'operation {index} must have a non-empty id')
    if operation_id in seen_ids:
        raise MapEditError(f'duplicate operation id: {operation_id}')
    seen_ids.add(operation_id)
    operation_type = operation.get('type')
    if operation_type == 'remove_box':
        minimum = _xyz(operation.get('min_xyz'), f'{operation_id}.min_xyz')
        maximum = _xyz(operation.get('max_xyz'), f'{operation_id}.max_xyz')
        if any(low >= high for low, high in zip(minimum, maximum)):
            raise MapEditError(
                f'{operation_id} requires min_xyz to be strictly below max_xyz on every axis'
            )
    elif operation_type == 'disable_loop_edge':
        for key in ('from', 'to'):
            value = operation.get(key)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise MapEditError(f'{operation_id}.{key} must be a non-negative integer')
        if operation['from'] == operation['to']:
            raise MapEditError(f'{operation_id} cannot disable a self-edge')
    else:
        raise MapEditError(f'unsupported operation type in {operation_id}: {operation_type!r}')


def _xyz(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise MapEditError(f'{name} must contain exactly three numbers')
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise MapEditError(f'{name} must contain exactly three numbers') from exc
    if not all(math.isfinite(item) for item in result):
        raise MapEditError(f'{name} contains a non-finite number')
    return result  # type: ignore[return-value]


def validate_source_identity(bundle_dir: Path, plan: dict[str, Any]) -> str:
    """Fail closed when a plan is applied to a different map bundle."""
    actual = sha256_file(bundle_dir / 'map_bundle.yaml')
    expected = str(plan['source']['map_bundle_sha256']).lower()
    if actual != expected:
        raise MapEditError(
            'edit plan source mismatch: map_bundle.yaml changed or belongs '
            f'to another map (expected {expected}, got {actual}). Reopen '
            'the current preview and export a new plan.'
        )
    identities = {
        'full_map_sha256': sha256_file(_bundle_artifact_path(bundle_dir, 'full_map')),
        'pointcloud_map_sha256': sha256_directory(
            _bundle_artifact_path(bundle_dir, 'pointcloud_map')
        ),
        'loop_edges_sha256': sha256_file(_bundle_artifact_path(bundle_dir, 'loop_edges')),
    }
    for name, actual_artifact_hash in identities.items():
        if actual_artifact_hash != str(plan['source'][name]).lower():
            artifact = name.removesuffix('_sha256')
            raise MapEditError(
                f'edit plan source mismatch: {artifact} changed. Reopen the current preview '
                'and export a new plan.'
            )
    return actual


def _load_bundle_manifest(bundle_dir: Path) -> dict[str, Any]:
    try:
        manifest = yaml.safe_load(
            (bundle_dir / 'map_bundle.yaml').read_text(encoding='utf-8')
        )
    except (OSError, yaml.YAMLError) as exc:
        raise MapEditError(f'cannot read map_bundle.yaml: {exc}') from exc
    if not isinstance(manifest, dict) or not isinstance(manifest.get('artifacts'), dict):
        raise MapEditError('map_bundle.yaml does not contain an artifacts mapping')
    return manifest


def _bundle_artifact_path(bundle_dir: Path, name: str) -> Path:
    manifest = _load_bundle_manifest(bundle_dir)
    value = manifest['artifacts'].get(name)
    if not isinstance(value, str) or not value:
        raise MapEditError(f'map bundle artifact is missing: {name}')
    relative = Path(value)
    if relative.is_absolute():
        raise MapEditError(f'map bundle artifact must be relative: {name}')
    path = (bundle_dir / relative).resolve()
    try:
        path.relative_to(bundle_dir.resolve())
    except ValueError as exc:
        raise MapEditError(f'map bundle artifact escapes bundle: {name}') from exc
    return path


def accepted_loop_edges(bundle_dir: Path) -> list[dict[str, Any]]:
    """Read accepted loop constraints, never inferred loop candidates."""
    path = _bundle_artifact_path(bundle_dir, 'loop_edges')
    try:
        with path.open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != LOOP_HEADER:
                raise MapEditError(f'invalid loop_edges.csv header: {reader.fieldnames!r}')
            rows = list(reader)
    except (OSError, csv.Error) as exc:
        raise MapEditError(f'cannot read accepted loop edges: {exc}') from exc
    result = []
    for row in rows:
        try:
            result.append({
                **row,
                'from': int(row['from']),
                'to': int(row['to']),
                'fitness': float(row['fitness']),
            })
        except (KeyError, TypeError, ValueError) as exc:
            raise MapEditError('loop_edges.csv contains an invalid row') from exc
    return result


def apply_map_edit(
    *,
    source_dir: Path,
    plan_path: Path,
    output_dir: Path,
    backend_input: Path | None = None,
    params_path: Path | None = None,
    setup_path: Path | None = None,
    replay_script: Path | None = None,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Apply one edit plan into a new, verified map bundle."""
    bundle_dir = resolve_bundle_dir(source_dir)
    _reject_source_symlinks(bundle_dir)
    plan_path = plan_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    plan = load_edit_plan(plan_path)
    source_hash = validate_source_identity(bundle_dir, plan)
    if backend_input is None:
        inferred_backend = bundle_dir / 'backend_input'
        if (inferred_backend / 'metadata.yaml').is_file():
            backend_input = inferred_backend
    if params_path is None:
        inferred_params = bundle_dir / 'graph_params.ros.yaml'
        if inferred_params.is_file():
            params_path = inferred_params
    if output_dir.exists():
        raise MapEditError(
            f'output already exists: {output_dir}. Choose a new directory; '
            'edits never overwrite maps.'
        )
    try:
        output_dir.relative_to(bundle_dir)
    except ValueError:
        pass
    else:
        raise MapEditError(
            'output directory must not be inside the source map bundle. Choose a sibling '
            'or another destination so temporary files cannot become source artifacts.'
        )
    operations = list(plan['operations'])
    loop_operations = [item for item in operations if item['type'] == 'disable_loop_edge']
    _validate_loop_replay_inputs(
        loop_operations,
        backend_input=backend_input,
        params_path=params_path,
        setup_path=setup_path,
    )
    if dry_run:
        return {
            'schema_version': 1,
            'status': 'DRY_RUN',
            'source_dir': str(bundle_dir),
            'output_dir': str(output_dir),
            'plan_path': str(plan_path),
            'operation_count': len(operations),
            'requires_backend_replay': bool(loop_operations),
            'backend_input': str(backend_input) if backend_input else None,
            'params_path': str(params_path) if params_path else None,
            'setup_path': str(setup_path) if setup_path else 'active_environment',
        }

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f'.{output_dir.name}.editing-',
        dir=output_dir.parent,
    ) as temporary:
        staging = Path(temporary) / 'candidate'
        # The source bundle was checked above, so this copy cannot import an
        # external symlink target or preserve a link that later writes back.
        shutil.copytree(bundle_dir, staging, symlinks=False)
        replay_report: dict[str, Any] = {'performed': False}
        if loop_operations:
            replay_report = _replay_without_disabled_loops(
                staging,
                loop_operations,
                backend_input=backend_input,
                params_path=params_path,
                setup_path=setup_path,
                replay_script=replay_script,
            )
        boxes = [
            (_xyz(item['min_xyz'], 'min_xyz'), _xyz(item['max_xyz'], 'max_xyz'))
            for item in operations
            if item['type'] == 'remove_box'
        ]
        point_report = _apply_remove_boxes(staging, boxes) if boxes else {
            'performed': False,
            'boxes': 0,
            'tile_points_before': _metadata_point_count(staging),
            'tile_points_after': _metadata_point_count(staging),
            'tile_points_removed': 0,
            'tiles_removed': 0,
        }
        canonical_plan = staging / EDIT_PLAN_NAME
        canonical_plan.write_text(
            json.dumps(plan, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        manifest = _load_bundle_manifest(staging)
        manifest['edit'] = {
            'schema_version': 1,
            'source_map_bundle_sha256': source_hash,
            'source_artifacts': dict(plan['source']),
            'plan': EDIT_PLAN_NAME,
            'receipt': EDIT_RECEIPT_NAME,
        }
        (staging / 'map_bundle.yaml').write_text(
            yaml.safe_dump(manifest, sort_keys=False),
            encoding='utf-8',
        )
        validation = _validate_candidate(staging)
        receipt = {
            'schema_version': 1,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS',
            'source_dir': str(bundle_dir),
            'output_dir': str(output_dir),
            'source_map_bundle_sha256': source_hash,
            'source_artifacts': dict(plan['source']),
            'candidate_map_bundle_sha256': sha256_file(staging / 'map_bundle.yaml'),
            'plan_sha256': sha256_file(canonical_plan),
            'operations': operations,
            'point_edit': point_report,
            'loop_replay': replay_report,
            'validation': validation,
        }
        (staging / EDIT_RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(staging, output_dir)
    return receipt


def _validate_loop_replay_inputs(
    operations: list[dict[str, Any]],
    *,
    backend_input: Path | None,
    params_path: Path | None,
    setup_path: Path | None,
) -> None:
    if not operations:
        return
    missing = []
    if backend_input is None or not backend_input.is_dir() or not (
        backend_input / 'metadata.yaml'
    ).is_file():
        missing.append('--backend-input')
    if params_path is None or not params_path.is_file():
        missing.append('--params')
    active_ros = bool(os.environ.get('ROS_DISTRO')) and shutil.which('ros2') is not None
    if (setup_path is None and not active_ros) or (
        setup_path is not None and not setup_path.is_file()
    ):
        missing.append('--setup (or an active ROS environment)')
    if missing:
        raise MapEditError(
            'disabling a loop edge requires deterministic backend replay; missing or invalid: '
            + ', '.join(missing)
            + '. Re-record an editable run with /rko_lio/odometry and /rko_lio/frame, '
            'or remove the loop operation from the plan.'
        )


def _replay_without_disabled_loops(
    staging: Path,
    operations: list[dict[str, Any]],
    *,
    backend_input: Path | None,
    params_path: Path | None,
    setup_path: Path | None,
    replay_script: Path | None,
) -> dict[str, Any]:
    assert backend_input is not None and params_path is not None
    source_rows = accepted_loop_edges(staging)
    disabled = {(int(item['from']), int(item['to'])) for item in operations}
    available = {(int(item['from']), int(item['to'])) for item in source_rows}
    missing = sorted(disabled - available)
    if missing:
        raise MapEditError(
            'edit plan refers to loop edges that are not accepted in this bundle: '
            + ', '.join(f'{source}->{target}' for source, target in missing)
        )
    retained = [
        row for row in source_rows
        if (int(row['from']), int(row['to'])) not in disabled
    ]
    replay_root = staging / '.map_edit_replay'
    replay_root.mkdir()
    fixed_edges = replay_root / 'retained_loop_edges.csv'
    with fixed_edges.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=LOOP_HEADER)
        writer.writeheader()
        for row in retained:
            writer.writerow({key: row[key] for key in LOOP_HEADER})
    script = (
        replay_script.expanduser().resolve()
        if replay_script else Path(__file__).resolve().parent / 'run_offline_determinism_check.sh'
    )
    if not script.is_file():
        raise MapEditError(f'offline replay helper is missing: {script}')
    command = [
        'bash', str(script),
        '--bag', str(backend_input.expanduser().resolve()),
        '--params', str(params_path.expanduser().resolve()),
        '--runs', '1',
        '--output-dir', str(replay_root / 'result'),
        '--save-maps',
        '--param', f'fixed_loop_edges_path:={fixed_edges}',
        '--param', 'refine:=true',
    ]
    if setup_path is not None:
        command[6:6] = ['--setup', str(setup_path.expanduser().resolve())]
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        tail = '\n'.join((completed.stdout + '\n' + completed.stderr).splitlines()[-20:])
        raise MapEditError(
            f'offline replay failed with exit code {completed.returncode}:\n{tail}'
        )
    run_dir = replay_root / 'result' / 'run1'
    required = {
        'map_optimized.pcd': run_dir / 'map_optimized.pcd',
        'trajectory_optimized.tum': run_dir / 'trajectory_optimized.tum',
        'pose_graph.g2o': run_dir / 'pose_graph.g2o',
        'loop_edges.csv': run_dir / 'loop_edges.csv',
    }
    absent = [name for name, path in required.items() if not path.is_file()]
    if absent:
        raise MapEditError('offline replay did not produce: ' + ', '.join(absent))
    replay_summary = replay_root / 'result' / 'offline_determinism_summary.md'
    runner_log = run_dir / 'runner.log'
    missing_evidence = [
        name for name, path in (
            ('offline_determinism_summary.md', replay_summary),
            ('runner.log', runner_log),
        ) if not path.is_file()
    ]
    if missing_evidence:
        raise MapEditError(
            'offline replay did not preserve required evidence: '
            + ', '.join(missing_evidence)
        )
    for source_name, artifact_name in (
        ('map_optimized.pcd', 'full_map'),
        ('trajectory_optimized.tum', 'trajectory'),
        ('pose_graph.g2o', 'pose_graph'),
        ('loop_edges.csv', 'loop_edges'),
    ):
        shutil.copy2(
            required[source_name], _bundle_artifact_path(staging, artifact_name)
        )
    _retile_full_map(staging)
    manifest = _load_bundle_manifest(staging)
    manifest['loop_edge_count'] = len(retained)
    (staging / 'map_bundle.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
    )
    replay_id = hashlib.sha256(
        json.dumps(sorted(disabled), separators=(',', ':')).encode('utf-8')
    ).hexdigest()[:12]
    evidence_dir = staging / 'map_edit_replay_evidence' / replay_id
    evidence_dir.mkdir(parents=True, exist_ok=False)
    evidence_sources = {
        'retained_loop_edges.csv': fixed_edges,
        'offline_determinism_summary.md': replay_summary,
        'runner.log': runner_log,
    }
    evidence: dict[str, dict[str, str]] = {}
    for name, source in evidence_sources.items():
        destination = evidence_dir / name
        shutil.copy2(source, destination)
        evidence[name] = {
            'path': destination.relative_to(staging).as_posix(),
            'sha256': sha256_file(destination),
        }
    helper_stdout = evidence_dir / 'helper.stdout.log'
    helper_stdout.write_text(completed.stdout, encoding='utf-8')
    evidence[helper_stdout.name] = {
        'path': helper_stdout.relative_to(staging).as_posix(),
        'sha256': sha256_file(helper_stdout),
    }
    helper_stderr = evidence_dir / 'helper.stderr.log'
    helper_stderr.write_text(completed.stderr, encoding='utf-8')
    evidence[helper_stderr.name] = {
        'path': helper_stderr.relative_to(staging).as_posix(),
        'sha256': sha256_file(helper_stderr),
    }
    shutil.rmtree(replay_root)
    return {
        'performed': True,
        'backend_input': str(backend_input.expanduser().resolve()),
        'params_path': str(params_path.expanduser().resolve()),
        'setup_path': (
            str(setup_path.expanduser().resolve())
            if setup_path is not None else 'active_environment'
        ),
        'accepted_edges_before': len(source_rows),
        'accepted_edges_after': len(retained),
        'disabled_edges': [f'{source}->{target}' for source, target in sorted(disabled)],
        'evidence': evidence,
    }


def _pcd_layout(path: Path) -> PcdLayout:
    header = parse_pcd_header(str(path))
    fields = [str(item).lower() for item in header.get('fields') or []]
    sizes = [int(item) for item in header.get('size') or []]
    types = [str(item).upper() for item in header.get('type') or []]
    counts = [int(item) for item in header.get('count') or []]
    if fields and not counts:
        counts = [1] * len(fields)
    if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise MapEditError(f'invalid PCD field metadata: {path}')
    if any(size <= 0 or count <= 0 for size, count in zip(sizes, counts)):
        raise MapEditError(f'invalid PCD field width: {path}')
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        raise MapEditError(f'PCD must contain x, y, and z fields: {path}')
    points = int(header.get('points') or 0)
    if points <= 0:
        points = int(header.get('width') or 0) * int(header.get('height') or 1)
    return PcdLayout(
        fields=fields,
        sizes=sizes,
        types=types,
        counts=counts,
        points=points,
        data=str(header.get('data') or '').lower(),
        header_bytes=int(header.get('header_bytes') or 0),
    )


def _binary_records(path: Path) -> tuple[PcdLayout, list[bytes]]:
    layout = _pcd_layout(path)
    with path.open('rb') as stream:
        stream.seek(layout.header_bytes)
        payload = stream.read()
    if layout.data == 'binary':
        expected = layout.row_size * layout.points
        if len(payload) < expected:
            raise MapEditError(f'truncated binary PCD payload: {path}')
        return layout, [
            payload[offset:offset + layout.row_size]
            for offset in range(0, expected, layout.row_size)
        ]
    if layout.data == 'binary_compressed':
        if len(payload) < 8:
            raise MapEditError(f'missing binary_compressed size header: {path}')
        compressed_size, uncompressed_size = struct.unpack_from('<II', payload, 0)
        compressed = payload[8:8 + compressed_size]
        if len(compressed) != compressed_size:
            raise MapEditError(f'truncated binary_compressed PCD payload: {path}')
        raw = _decompress_lzf(compressed, uncompressed_size)
        expected = layout.row_size * layout.points
        if len(raw) != expected:
            raise MapEditError(
                f'binary_compressed PCD expanded to {len(raw)} bytes, expected {expected}: {path}'
            )
        field_offsets = []
        offset = 0
        for width in layout.field_widths:
            field_offsets.append(offset)
            offset += width * layout.points
        records = []
        for point_index in range(layout.points):
            record = bytearray()
            for field_offset, width in zip(field_offsets, layout.field_widths):
                start = field_offset + point_index * width
                record.extend(raw[start:start + width])
            records.append(bytes(record))
        return layout, records
    if layout.data == 'ascii':
        text = payload.decode('ascii', errors='strict')
        records = []
        formats = _field_formats(layout)
        for line in text.splitlines():
            values = line.split()
            if not values:
                continue
            if len(values) != len(formats):
                raise MapEditError(f'invalid ASCII PCD row in {path}')
            record = bytearray()
            for value, fmt in zip(values, formats):
                try:
                    number: int | float = float(value) if fmt in ('f', 'd') else int(value)
                    record.extend(struct.pack('<' + fmt, number))
                except (ValueError, struct.error) as exc:
                    raise MapEditError(f'invalid ASCII PCD value in {path}: {value}') from exc
            records.append(bytes(record))
        if len(records) != layout.points:
            raise MapEditError(
                f'ASCII PCD rows {len(records)} != POINTS {layout.points}: {path}'
            )
        return layout, records
    raise MapEditError(f'unsupported PCD DATA {layout.data!r}: {path}')


def _field_formats(layout: PcdLayout) -> list[str]:
    mapping = {
        ('F', 4): 'f', ('F', 8): 'd',
        ('U', 1): 'B', ('U', 2): 'H', ('U', 4): 'I',
        ('I', 1): 'b', ('I', 2): 'h', ('I', 4): 'i',
    }
    result = []
    for size, data_type, count in zip(layout.sizes, layout.types, layout.counts):
        fmt = mapping.get((data_type, size))
        if fmt is None:
            raise MapEditError(f'unsupported PCD field type: {data_type}{size}')
        result.extend([fmt] * count)
    return result


def _xyz_from_record(layout: PcdLayout, record: bytes) -> tuple[float, float, float]:
    field_offsets: dict[str, int] = {}
    offset = 0
    for field, width in zip(layout.fields, layout.field_widths):
        field_offsets[field] = offset
        offset += width
    result = []
    for name in ('x', 'y', 'z'):
        index = layout.fields.index(name)
        fmt = _field_formats(PcdLayout(
            fields=[name], sizes=[layout.sizes[index]], types=[layout.types[index]],
            counts=[1], points=1, data='binary', header_bytes=0,
        ))[0]
        result.append(float(struct.unpack_from('<' + fmt, record, field_offsets[name])[0]))
    return result[0], result[1], result[2]


def _render_binary_header(layout: PcdLayout, points: int) -> bytes:
    lines = [
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS ' + ' '.join(layout.fields),
        'SIZE ' + ' '.join(str(item) for item in layout.sizes),
        'TYPE ' + ' '.join(layout.types),
        'COUNT ' + ' '.join(str(item) for item in layout.counts),
        f'WIDTH {points}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {points}',
        'DATA binary',
        '',
    ]
    return '\n'.join(lines).encode('ascii')


def _write_binary_pcd(path: Path, layout: PcdLayout, records: list[bytes]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as stream:
        stream.write(_render_binary_header(layout, len(records)))
        for record in records:
            stream.write(record)


def _inside_boxes(
    xyz: tuple[float, float, float],
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
) -> bool:
    return any(
        all(low <= value <= high for value, low, high in zip(xyz, minimum, maximum))
        for minimum, maximum in boxes
    )


def _filter_pcd(
    path: Path,
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
) -> tuple[int, int]:
    layout, records = _binary_records(path)
    kept = [
        record
        for record in records
        if not _inside_boxes(_xyz_from_record(layout, record), boxes)
    ]
    _write_binary_pcd(path, layout, kept)
    return len(records), len(kept)


def _metadata_tile_path(pointcloud_dir: Path, name: str) -> Path:
    """Resolve one metadata tile without allowing traversal or symlinks."""
    relative = Path(name)
    if (
        relative.is_absolute()
        or len(relative.parts) != 1
        or relative.name != name
        or '/' in name
        or '\\' in name
    ):
        raise MapEditError(
            f'pointcloud metadata contains an unsafe tile path: {name!r}'
        )
    path = pointcloud_dir / relative
    if path.is_symlink():
        raise MapEditError(
            f'pointcloud metadata tile must not be a symlink: {name!r}'
        )
    return path


def _apply_remove_boxes(
    staging: Path,
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
) -> dict[str, Any]:
    pointcloud_dir = _bundle_artifact_path(staging, 'pointcloud_map')
    metadata_path = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise MapEditError('pointcloud_map_metadata.yaml must contain a mapping')
    tile_names = [str(key) for key in metadata if str(key).endswith('.pcd')]
    before = 0
    after = 0
    removed_tiles = []
    for name in tile_names:
        path = _metadata_tile_path(pointcloud_dir, name)
        tile_before, tile_after = _filter_pcd(path, boxes)
        before += tile_before
        after += tile_after
        if tile_after == 0:
            path.unlink()
            metadata.pop(name, None)
            removed_tiles.append(name)
    if after <= 0:
        raise MapEditError('remove_box operations would leave the tiled map empty')
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    full_map = _bundle_artifact_path(staging, 'full_map')
    full_before, full_after = _filter_pcd(full_map, boxes)
    if full_after <= 0:
        raise MapEditError('remove_box operations would leave map.pcd empty')
    return {
        'performed': True,
        'boxes': len(boxes),
        'tile_points_before': before,
        'tile_points_after': after,
        'tile_points_removed': before - after,
        'full_map_points_before': full_before,
        'full_map_points_after': full_after,
        'full_map_points_removed': full_before - full_after,
        'tiles_removed': len(removed_tiles),
        'removed_tile_names': removed_tiles,
        'output_data_encoding': 'binary',
        'fields_preserved': True,
    }


def _metadata_point_count(staging: Path) -> int:
    pointcloud_dir = _bundle_artifact_path(staging, 'pointcloud_map')
    metadata = yaml.safe_load(
        (pointcloud_dir / 'pointcloud_map_metadata.yaml').read_text(encoding='utf-8')
    )
    if not isinstance(metadata, dict):
        return 0
    total = 0
    for name in metadata:
        if str(name).endswith('.pcd'):
            total += _pcd_layout(
                _metadata_tile_path(pointcloud_dir, str(name))
            ).points
    return total


def _retile_full_map(staging: Path) -> None:
    full_map = _bundle_artifact_path(staging, 'full_map')
    layout, records = _binary_records(full_map)
    pointcloud_dir = _bundle_artifact_path(staging, 'pointcloud_map')
    metadata_path = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise MapEditError('pointcloud_map_metadata.yaml must contain a mapping')
    try:
        x_resolution = float(metadata['x_resolution'])
        y_resolution = float(metadata['y_resolution'])
    except (KeyError, TypeError, ValueError) as exc:
        raise MapEditError('pointcloud map metadata has invalid resolutions') from exc
    if x_resolution <= 0 or y_resolution <= 0:
        raise MapEditError('pointcloud map resolutions must be positive')
    cells: dict[tuple[int, int], list[bytes]] = {}
    for record in records:
        x, y, _ = _xyz_from_record(layout, record)
        lower_x = math.floor(x / x_resolution) * x_resolution
        lower_y = math.floor(y / y_resolution) * y_resolution
        if not lower_x.is_integer() or not lower_y.is_integer():
            raise MapEditError(
                'replayed map grid does not have integer tile coordinates required by Autoware'
            )
        cells.setdefault((int(lower_x), int(lower_y)), []).append(record)
    if not cells:
        raise MapEditError('offline replay produced an empty map')
    shutil.rmtree(pointcloud_dir)
    pointcloud_dir.mkdir()
    new_metadata: dict[str, Any] = {
        'x_resolution': x_resolution,
        'y_resolution': y_resolution,
    }
    for (lower_x, lower_y), cell_records in sorted(cells.items()):
        name = f'{lower_x}_{lower_y}.pcd'
        _write_binary_pcd(pointcloud_dir / name, layout, cell_records)
        new_metadata[name] = [lower_x, lower_y]
    (pointcloud_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump(new_metadata, sort_keys=False), encoding='utf-8'
    )
    _write_binary_pcd(full_map, layout, records)


def _validate_candidate(staging: Path) -> dict[str, Any]:
    bundle_verifier = BundleVerifier(staging)
    bundle_ok = bundle_verifier.run()
    map_verifier = MapVerifier(
        _bundle_artifact_path(staging, 'pointcloud_map'),
        check_bounds=True,
        verbose=False,
    )
    with redirect_stdout(io.StringIO()):
        map_ok = map_verifier.run()
    if not bundle_ok or not map_ok:
        messages = bundle_verifier.failures + map_verifier.failures
        raise MapEditError('candidate validation failed: ' + '; '.join(messages))
    return {
        'map_bundle': 'PASS',
        'autoware_pointcloud_map': 'PASS',
        'map_bundle_checks': bundle_verifier.passes,
        'autoware_pass_count': len(map_verifier.passes),
        'autoware_warning_count': len(map_verifier.warnings),
    }
