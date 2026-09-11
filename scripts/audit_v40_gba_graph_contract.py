#!/usr/bin/env python3
"""Audit the no-loop Voxel GBA graph contract without reading ground truth.

The historical v21 screen selected sparse keyframes whose ``Keyframe::id``
was already the source scan index, but initialized GTSAM with only those
sparse keys while declaring and writing back a dense scan range.  This tool
replays the keyframe selection from a behavior-preserving trajectory and
checks whether the safe next design is a full-scan anchored odometry graph,
not an identity-changing keyframe remap.  It never writes a trajectory or map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_tum_positions(path: Path) -> list[np.ndarray]:
    positions: list[np.ndarray] = []
    previous_stamp = -math.inf
    for line_number, raw in enumerate(
            path.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'{path}:{line_number}: invalid TUM row')
        values = np.asarray([float(value) for value in fields], dtype=np.float64)
        if not np.isfinite(values).all() or values[0] <= previous_stamp:
            raise ValueError(
                f'{path}:{line_number}: invalid or non-increasing values')
        previous_stamp = float(values[0])
        positions.append(values[1:4])
    if not positions:
        raise ValueError(f'{path}: empty trajectory')
    return positions


def select_keyframe_scan_ids(
        positions: list[np.ndarray], window_size: int,
        keyframe_stride: int, min_keyframe_distance_m: float,
        ) -> list[int]:
    """Replay the v21 no-loop block/keyframe selection exactly."""
    if (window_size < 1 or keyframe_stride < 1 or
            not math.isfinite(min_keyframe_distance_m) or
            min_keyframe_distance_m < 0.0):
        raise ValueError('invalid keyframe selection configuration')
    selected: list[int] = []
    last_position: np.ndarray | None = None
    for block_end in range(window_size - 1, len(positions), window_size):
        buffer_count = block_end + 1
        position = positions[block_end]
        distance_ok = (
            last_position is None or
            float(np.linalg.norm(position - last_position)) >=
            min_keyframe_distance_m)
        if distance_ok and buffer_count % keyframe_stride == 0:
            selected.append(block_end)
            last_position = position
    return selected


def validate_keyframe_scan_ids(
        scan_count: int, keyframe_scan_ids: list[int]) -> dict[str, Any]:
    if scan_count < 1:
        raise ValueError('scan count must be positive')
    duplicates = sorted({value for value in keyframe_scan_ids
                         if keyframe_scan_ids.count(value) > 1})
    out_of_range = sorted({value for value in keyframe_scan_ids
                           if value < 0 or value >= scan_count})
    strictly_increasing = all(
        right > left for left, right in zip(
            keyframe_scan_ids, keyframe_scan_ids[1:]))
    return {
        'valid_scan_index_namespace': (
            not duplicates and not out_of_range and strictly_increasing),
        'strictly_increasing': strictly_increasing,
        'unique': not duplicates,
        'all_in_scan_range': not out_of_range,
        'duplicates': duplicates,
        'out_of_range': out_of_range,
        'dense_keyframe_ordinal_namespace': keyframe_scan_ids == list(
            range(len(keyframe_scan_ids))),
    }


def graph_contract(
        scan_count: int, keyframe_scan_ids: list[int]) -> dict[str, Any]:
    declared_v21_keys = (
        keyframe_scan_ids[-1] + 1 if keyframe_scan_ids else 0)
    missing_sparse_keys = declared_v21_keys - len(set(keyframe_scan_ids))
    trailing_scans = scan_count - declared_v21_keys
    return {
        'v21_sparse_graph': {
            'initial_keys': len(set(keyframe_scan_ids)),
            'declared_dense_writeback_keys': declared_v21_keys,
            'missing_initial_keys_in_declared_range': missing_sparse_keys,
            'trailing_scans_outside_declared_range': trailing_scans,
            'anchored_prior_present': False,
            'odometry_chain_edges': 0,
            'structurally_valid': (
                declared_v21_keys > 0 and missing_sparse_keys == 0 and
                trailing_scans == 0),
        },
        'full_scan_graph': {
            'initial_keys': scan_count,
            'declared_dense_writeback_keys': scan_count,
            'missing_initial_keys_in_declared_range': 0,
            'trailing_scans_outside_declared_range': 0,
            'anchored_prior_present': True,
            'odometry_chain_edges': max(0, scan_count - 1),
            'hba_endpoint_namespace': 'source_scan_index',
            'structurally_valid': scan_count > 0,
        },
    }


def audit_source_contract(base_text: str, v21_text: str) -> dict[str, bool]:
    def contains(text: str, pattern: str) -> bool:
        return re.search(pattern, text, re.MULTILINE | re.DOTALL) is not None

    return {
        'base_keyframe_id_is_source_scan_index': contains(
            base_text, r'smp->id\s*=\s*buf_base\s*-\s*1\s*;'),
        'base_hba_edges_keep_keyframe_scan_ids': contains(
            base_text,
            r'gba_edges\.push\(s1\.mp,\s*s2\.mp,\s*s1\.id,\s*s2\.id'),
        'base_full_graph_initializes_every_scan': contains(
            base_text, r'initial\.insert\(j,\s*pose3\)'),
        'base_full_graph_has_odometry_chain': contains(
            base_text, r'add_edge\(j-1,\s*j,'),
        'base_full_graph_has_anchor_prior': contains(
            base_text, r'graph\.addPrior\(i,\s*pose3,\s*fixd_noise\)'),
        'base_writeback_uses_dense_scan_range': contains(
            base_text,
            r'for\(int j=stepsizes\[ii\];\s*j<stepsizes\[ii\+1\];\s*j\+\+\)'),
        'v21_sparse_initial_uses_only_keyframe_ids': contains(
            v21_text, r'initial\.insert\(kf->id,\s*pose3\)'),
        'v21_declares_range_through_max_keyframe_id': contains(
            v21_text, r'stepsizes\.push_back\(max_kf_id\s*\+\s*1\)'),
        'v21_calls_dense_writeback_with_sparse_initial': contains(
            v21_text,
            r'stepsizes\.push_back\(max_kf_id\s*\+\s*1\).*?'
            r'topDownProcess\(initial,\s*graph,\s*ids,\s*stepsizes\)'),
    }


def build_report(
        dataset_label: str, trajectory_path: Path, base_source_path: Path,
        v21_source_path: Path, window_size: int, keyframe_stride: int,
        min_keyframe_distance_m: float,
        ) -> dict[str, Any]:
    positions = load_tum_positions(trajectory_path)
    keyframe_ids = select_keyframe_scan_ids(
        positions, window_size, keyframe_stride, min_keyframe_distance_m)
    identity = validate_keyframe_scan_ids(len(positions), keyframe_ids)
    graphs = graph_contract(len(positions), keyframe_ids)
    source = audit_source_contract(
        base_source_path.read_text(encoding='utf-8', errors='replace'),
        v21_source_path.read_text(encoding='utf-8', errors='replace'))
    required_source_checks = all(source.values())
    go = (
        identity['valid_scan_index_namespace'] and
        len(keyframe_ids) >= 10 and required_source_checks and
        graphs['full_scan_graph']['structurally_valid'] and
        not graphs['v21_sparse_graph']['structurally_valid'])
    return {
        'schema_version': 1,
        'status': 'ground_truth_free_v40_gba_graph_contract_audit',
        'accuracy_ground_truth_accessed': False,
        'trajectory_or_map_mutated': False,
        'decision': (
            'GO_FULL_SCAN_GRAPH_RESTORATION_NO_ID_REMAP' if go
            else 'NO_GO_GBA_GRAPH_CONTRACT'),
        'dataset_label': dataset_label,
        'inputs': {
            'trajectory_tum': str(trajectory_path.resolve()),
            'trajectory_sha256': sha256(trajectory_path),
            'base_source': str(base_source_path.resolve()),
            'base_source_sha256': sha256(base_source_path),
            'v21_source': str(v21_source_path.resolve()),
            'v21_source_sha256': sha256(v21_source_path),
        },
        'selection': {
            'window_size_scans': window_size,
            'keyframe_stride': keyframe_stride,
            'min_keyframe_distance_m': min_keyframe_distance_m,
            'scan_count': len(positions),
            'keyframe_count': len(keyframe_ids),
            'first_keyframe_scan_id': (
                keyframe_ids[0] if keyframe_ids else None),
            'last_keyframe_scan_id': (
                keyframe_ids[-1] if keyframe_ids else None),
            'keyframe_scan_ids': keyframe_ids,
        },
        'identity_contract': identity,
        'graph_contract': graphs,
        'source_contract': source,
        'conclusion': (
            'Keyframe ids are valid sparse source-scan ids. Restore the '
            'anchored full-scan odometry graph and attach HBA edges in that '
            'existing namespace; do not reinterpret ids as dense keyframe '
            'ordinals.'),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-label', required=True)
    parser.add_argument('--trajectory-tum', required=True, type=Path)
    parser.add_argument('--base-source', required=True, type=Path)
    parser.add_argument('--v21-source', required=True, type=Path)
    parser.add_argument('--window-size', type=int, default=10)
    parser.add_argument('--keyframe-stride', type=int, default=1)
    parser.add_argument('--min-keyframe-distance-m', type=float, default=5.0)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    report = build_report(
        args.dataset_label, args.trajectory_tum, args.base_source,
        args.v21_source, args.window_size, args.keyframe_stride,
        args.min_keyframe_distance_m)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'dataset': report['dataset_label'],
        'decision': report['decision'],
        'scan_count': report['selection']['scan_count'],
        'keyframe_count': report['selection']['keyframe_count'],
        'v21_missing_initial_keys': report['graph_contract'][
            'v21_sparse_graph']['missing_initial_keys_in_declared_range'],
        'output': str(args.output.resolve()),
    }, indent=2, sort_keys=True))
    return 0 if report['decision'].startswith('GO_') else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
