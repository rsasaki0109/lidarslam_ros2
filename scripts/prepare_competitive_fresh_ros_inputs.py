#!/usr/bin/env python3
"""Prepare canonical ROS 2 inputs from a managed fresh-holdout download.

This is deliberately a conversion checkpoint, not a downloader or benchmark
runner.  It reads only the raw-bag fields of ``downloaded_hashed`` manifests;
the ground-truth path is validated as a relative label and is never opened.
The conversion and semantic comparison are staged and published atomically,
with a receipt that binds the managed-root plan, raw-bag identity, exact tool
versions, commands, and output hashes.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lidarslam_benchmark_tools.freeze_competitive_fresh_holdouts import (  # noqa: E402
    _assert_regular,
    _atomic_json,
    _metadata_topics,
    _require_relative_path,
    _semantic_payload,
    _verify_file,
    build_ros2_identity,
    CANONICAL_TOPIC_CONTRACT,
    SEMANTIC_HASH_KIND,
    sha256_file,
)


ROSBAGS_CONVERT = 'rosbags-convert'
ROSBAGS_VERSION = '0.11.0'
COMPARATOR = SCRIPTS / 'compare_rosbag_semantic_inputs.py'
PREPARATION_SCHEMA_VERSION = 1
SEQUENCE_RE = re.compile(r'^exp[0-9]{2}$')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
REQUIRED_CONVERTER_HELP = (
    '--src', '--dst', '--dst-storage', 'sqlite3', '--compress', 'none',
    '--src-typestore', 'ros1_noetic', '--dst-typestore', 'copy')


def _json_load(path: Path, label: str) -> dict[str, Any]:
    _assert_regular(path, label)
    if not path.is_file():
        raise ValueError(f'{label} is missing: {path}')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{label} must contain a JSON object: {path}')
    return value


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise ValueError(f'{label} must be a 64-hex SHA-256')
    return value.lower()


def _relative_under_root(root: Path, value: Any, label: str) -> Path:
    relative = _require_relative_path(value, label)
    path = root / relative
    try:
        path.resolve(strict=False).relative_to(root.resolve())
    except ValueError as exc:
        raise ValueError(f'{label} escapes managed root') from exc
    return path


def _managed_marker(root: Path) -> dict[str, Any]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f'managed root must be a real directory: {root}')
    marker = root / '.freeze-root.json'
    document = _json_load(marker, 'managed-root marker')
    _require_sha(document.get('plan_sha256'), 'managed-root plan_sha256')
    _require_sha(document.get('selection_receipt_sha256'),
                 'managed-root selection_receipt_sha256')
    return document


def _manifest_for(root: Path, sequence: str) -> tuple[Path, dict[str, Any]]:
    if SEQUENCE_RE.fullmatch(sequence) is None:
        raise ValueError(f'invalid sequence: {sequence}')
    slot_root = root / 'slots' / sequence
    if slot_root.is_symlink() or not slot_root.is_dir():
        raise ValueError(f'{sequence} slot directory is missing or unsafe')
    manifest_path = slot_root / 'manifest.json'
    try:
        manifest_path.resolve(strict=False).relative_to(root.resolve())
    except ValueError as exc:
        raise ValueError(f'{sequence} manifest escapes managed root') from exc
    manifest = _json_load(manifest_path, f'{sequence} manifest')
    if manifest.get('status') != 'downloaded_hashed':
        raise ValueError(f'{sequence} manifest is not downloaded_hashed')
    source = manifest.get('source')
    if not isinstance(source, dict):
        raise ValueError(f'{sequence} manifest has no source identity')
    plan_sha = _require_sha(source.get('plan_sha256'),
                            f'{sequence} source.plan_sha256')
    marker = _managed_marker(root)
    if plan_sha != marker['plan_sha256']:
        raise ValueError(f'{sequence} manifest plan identity differs from marker')
    raw = manifest.get('raw_bag')
    if not isinstance(raw, dict):
        raise ValueError(f'{sequence} manifest has no raw_bag')
    raw_path = _relative_under_root(root, raw.get('path'), f'{sequence} raw_bag.path')
    observed = _verify_file(raw_path, {
        'kind': f'{sequence} raw_bag',
        'expected_bytes': raw.get('bytes'),
        'expected_sha256': raw.get('sha256'),
    })
    if observed['bytes'] != raw.get('bytes') or observed['sha256'] != raw.get('sha256'):
        raise ValueError(f'{sequence} raw-bag identity changed')
    # Validate only the GT label and its declared hash.  Do not stat, resolve,
    # hash, or otherwise open this path: blind evaluation depends on this
    # conversion step remaining GT-opaque.
    ground_truth = manifest.get('ground_truth')
    if not isinstance(ground_truth, dict):
        raise ValueError(f'{sequence} manifest has no ground_truth metadata')
    _require_relative_path(ground_truth.get('path'), f'{sequence} ground_truth.path')
    _require_sha(ground_truth.get('sha256'), f'{sequence} ground_truth.sha256')
    manifest_path = manifest_path.resolve()
    return manifest_path, manifest


def _manifest_paths(root: Path, sequences: list[str] | None) -> list[str]:
    if sequences:
        selected = list(dict.fromkeys(sequences))
    else:
        slots_root = root / 'slots'
        if slots_root.is_symlink() or not slots_root.is_dir():
            raise ValueError(f'managed root has no slots directory: {slots_root}')
        selected = sorted(
            path.name for path in slots_root.iterdir()
            if path.is_dir() and not path.is_symlink() and
            (path / 'manifest.json').exists())
    if not selected:
        raise ValueError('no downloaded-hashed slot manifests selected')
    for sequence in selected:
        _manifest_for(root, sequence)
    return selected


def _assert_safe_tree(path: Path, root: Path) -> None:
    if path.is_symlink():
        raise ValueError(f'ROS 2 output must not be a symlink: {path}')
    if not path.is_dir():
        raise ValueError(f'ROS 2 output is not a directory: {path}')
    root_resolved = root.resolve()
    for directory, directories, files in os.walk(path, followlinks=False):
        for name in (*directories, *files):
            candidate = Path(directory) / name
            if candidate.is_symlink():
                raise ValueError(f'ROS 2 output contains symlink: {candidate}')
            try:
                candidate.resolve(strict=False).relative_to(root_resolved)
            except ValueError as exc:
                raise ValueError(f'ROS 2 output escapes managed root: {candidate}') from exc


def _probe_runtime() -> dict[str, Any]:
    try:
        rosbags_version = importlib.metadata.version('rosbags')
    except importlib.metadata.PackageNotFoundError as exc:
        raise ValueError('rosbags package is not installed') from exc
    if rosbags_version != ROSBAGS_VERSION:
        raise ValueError(
            f'rosbags {ROSBAGS_VERSION} is required, found {rosbags_version}')
    try:
        numpy_version = importlib.metadata.version('numpy')
    except importlib.metadata.PackageNotFoundError as exc:
        raise ValueError('numpy package is not installed') from exc
    converter_path = shutil.which(ROSBAGS_CONVERT)
    if converter_path is None:
        raise ValueError('rosbags-convert is not installed')
    help_result = subprocess.run(
        [ROSBAGS_CONVERT, '--help'], check=True, capture_output=True,
        text=True)
    help_text = f'{help_result.stdout}\n{help_result.stderr}'
    missing = [token for token in REQUIRED_CONVERTER_HELP if token not in help_text]
    if missing:
        raise ValueError(
            f'rosbags-convert CLI is incompatible; missing {missing}')
    converter_file = Path(converter_path).resolve()
    _assert_regular(converter_file, 'rosbags-convert executable')
    if not converter_file.is_file():
        raise ValueError(f'rosbags-convert executable is missing: {converter_file}')
    if not COMPARATOR.is_file() or COMPARATOR.is_symlink():
        raise ValueError(f'semantic comparator is missing or unsafe: {COMPARATOR}')
    return {
        'rosbags_version': rosbags_version,
        'python_version': platform.python_version(),
        'numpy_version': numpy_version,
        'converter_executable': str(converter_file),
        'converter_script_sha256': sha256_file(converter_file),
        'converter_help_sha256': hashlib.sha256(
            help_text.encode('utf-8')).hexdigest(),
        'comparator_script': str(COMPARATOR.relative_to(ROOT)),
        'comparator_script_sha256': sha256_file(COMPARATOR),
    }


def _converter_argv(raw_path: Path, staging_path: Path) -> list[str]:
    return [
        ROSBAGS_CONVERT, '--src', str(raw_path), '--dst', str(staging_path),
        '--dst-storage', 'sqlite3', '--compress', 'none',
        '--src-typestore', 'ros1_noetic', '--dst-typestore', 'copy',
    ]


def _comparator_argv(raw_path: Path, ros2_path: Path,
                     report_path: Path) -> list[str]:
    command = [sys.executable, str(COMPARATOR), '--left', str(raw_path),
               '--right', str(ros2_path)]
    for topic, _msg_type, _role in CANONICAL_TOPIC_CONTRACT:
        command.extend(['--topic', topic])
    command.extend(['--output', str(report_path)])
    return command


def _validate_topics(ros2_path: Path) -> list[dict[str, str]]:
    topics = _metadata_topics(ros2_path)
    by_name = {item['name']: item['type'] for item in topics}
    for name, msg_type, _role in CANONICAL_TOPIC_CONTRACT:
        if by_name.get(name) != msg_type:
            raise ValueError(f'ROS 2 topic contract mismatch: {name}')
    return topics


def _validate_semantic_report(path: Path) -> dict[str, Any]:
    report = _json_load(path, 'semantic-equivalence report')
    _semantic_payload(report)
    return report


def _receipt_paths(slot_root: Path) -> dict[str, Path]:
    return {
        'ros2_part': slot_root / 'canonical_ros2.part',
        'ros2': slot_root / 'canonical_ros2',
        'semantic_part': slot_root / 'semantic_equivalence.json.part',
        'semantic': slot_root / 'semantic_equivalence.json',
        'receipt_part': slot_root / 'preparation_receipt.json.part',
        'receipt': slot_root / 'preparation_receipt.json',
    }


def _reject_unmanaged_slot_entries(slot_root: Path) -> None:
    allowed = {
        'manifest.json', 'source', 'ground_truth', 'calibration',
        'canonical_ros2.part', 'canonical_ros2',
        'semantic_equivalence.json.part', 'semantic_equivalence.json',
        'preparation_receipt.json.part', 'preparation_receipt.json',
    }
    for child in slot_root.iterdir():
        if child.name not in allowed:
            raise ValueError(f'unmanaged slot entry: {child}')
        if child.name in {'source', 'ground_truth', 'calibration'} and child.is_symlink():
            raise ValueError(f'slot input directory must not be a symlink: {child}')


def _validate_output_state(paths: dict[str, Path], resume: bool) -> None:
    for path in paths.values():
        if path.is_symlink():
            raise ValueError(f'preparation output must not be a symlink: {path}')
    pairs = (
        ('ros2', 'ros2_part'),
        ('semantic', 'semantic_part'),
        ('receipt', 'receipt_part'),
    )
    any_artifact = any(path.exists() for path in paths.values())
    if any_artifact and not resume:
        raise ValueError('preparation staging exists; use --resume')
    for final_name, part_name in pairs:
        if paths[final_name].exists() and paths[part_name].exists():
            raise ValueError(
                f'preparation output has both final and part: {final_name}')
    if paths['receipt'].exists():
        if (not paths['ros2'].exists() or not paths['semantic'].exists() or
                paths['ros2_part'].exists() or paths['semantic_part'].exists() or
                paths['receipt_part'].exists()):
            raise ValueError('receipt final is not a complete commit marker')
    elif paths['receipt_part'].exists():
        if not ((paths['ros2'].exists() or paths['ros2_part'].exists()) and
                (paths['semantic'].exists() or paths['semantic_part'].exists())):
            raise ValueError('receipt part lacks prepared artifacts')
    if paths['semantic'].exists() and not paths['ros2'].exists():
        raise ValueError('semantic final requires canonical_ros2 final')


def _receipt_identity(receipt: dict[str, Any], manifest_path: Path,
                      manifest: dict[str, Any], marker: dict[str, Any],
                      runtime: dict[str, Any], ros2_path: Path,
                      semantic_path: Path) -> dict[str, Any]:
    if receipt.get('schema_version') != PREPARATION_SCHEMA_VERSION or receipt.get(
            'receipt_kind') != 'competitive_fresh_ros_input_preparation':
        raise ValueError('preparation receipt schema/kind is invalid')
    if receipt.get('status') != 'prepared':
        raise ValueError('preparation receipt is not prepared')
    if receipt.get('sequence') != manifest.get('sequence'):
        raise ValueError('preparation receipt sequence changed')
    if receipt.get('plan_sha256') != marker['plan_sha256']:
        raise ValueError('preparation receipt plan identity differs from marker')
    if receipt.get('manifest_sha256') != sha256_file(manifest_path):
        raise ValueError('preparation receipt manifest identity changed')
    raw = manifest['raw_bag']
    if receipt.get('raw_bag') != {
            'path': raw['path'], 'bytes': raw['bytes'], 'sha256': raw['sha256']}:
        raise ValueError('preparation receipt raw-bag identity changed')
    if receipt.get('runtime') != runtime:
        raise ValueError('preparation receipt tool identity changed')
    raw_path = _relative_under_root(manifest_path.parents[2], raw['path'],
                                    'raw_bag.path')
    expected_converter = _converter_argv(raw_path, ros2_path.parent / 'canonical_ros2.part')
    if receipt.get('converter_argv') != expected_converter:
        raise ValueError('preparation converter argv identity changed')
    report_part = ros2_path.parent / 'semantic_equivalence.json.part'
    comparator_variants = [
        _comparator_argv(raw_path, ros2_path.parent / 'canonical_ros2.part',
                         report_part),
        _comparator_argv(raw_path, ros2_path.parent / 'canonical_ros2',
                         report_part),
    ]
    if receipt.get('comparator_argv') not in comparator_variants:
        raise ValueError('preparation comparator argv identity changed')
    if receipt.get('ground_truth_content_opened') is not False:
        raise ValueError('preparation receipt does not preserve GT opacity')
    _assert_safe_tree(ros2_path, manifest_path.parents[2])
    topics = _validate_topics(ros2_path)
    _validate_semantic_report(semantic_path)
    identity = build_ros2_identity(ros2_path, semantic_path)
    if receipt.get('ros2_tree_sha256') != identity['canonical_rosbag2_tree_sha256']:
        raise ValueError('preparation ROS 2 tree hash changed')
    if receipt.get('semantic_report_sha256') != sha256_file(semantic_path):
        raise ValueError('preparation semantic report hash changed')
    if receipt.get('semantic_equivalence_sha256') != identity[
            'semantic_equivalence_sha256']:
        raise ValueError('preparation semantic identity changed')
    if receipt.get('semantic_equivalence_hash_kind') != SEMANTIC_HASH_KIND:
        raise ValueError('preparation semantic hash kind changed')
    if receipt.get('semantic_report_all_topics_equal') is not True:
        raise ValueError('preparation semantic report is not equal')
    if receipt.get('topics') != topics:
        raise ValueError('preparation topic metadata changed')
    return identity


def _publish_staged(paths: dict[str, Path]) -> None:
    if paths['receipt'].exists():
        raise ValueError('receipt commit marker already exists')
    if not paths['receipt_part'].exists():
        raise ValueError('missing staged preparation receipt')
    for final_name, part_name in (
            ('ros2', 'ros2_part'), ('semantic', 'semantic_part')):
        if paths[final_name].exists() and paths[part_name].exists():
            raise ValueError(f'final and part coexist during publish: {final_name}')
    if paths['ros2_part'].exists():
        paths['ros2_part'].replace(paths['ros2'])
    if paths['semantic_part'].exists():
        paths['semantic_part'].replace(paths['semantic'])
    paths['receipt_part'].replace(paths['receipt'])
    if not (paths['ros2'].exists() and paths['semantic'].exists() and
            paths['receipt'].exists()):
        raise ValueError('atomic preparation publish is incomplete')


def _prepare_slot(root: Path, sequence: str, resume: bool,
                  runtime: dict[str, Any], marker: dict[str, Any]) -> dict[str, Any]:
    manifest_path, manifest = _manifest_for(root, sequence)
    slot_root = manifest_path.parent
    _reject_unmanaged_slot_entries(slot_root)
    paths = _receipt_paths(slot_root)
    had_artifacts = any(path.exists() for path in paths.values())
    _validate_output_state(paths, resume)
    if paths['receipt'].exists():
        identity = _receipt_identity(
            _json_load(paths['receipt'], 'preparation receipt'), manifest_path,
            manifest, marker, runtime, paths['ros2'], paths['semantic'])
        return {'sequence': sequence, 'status': 'prepared', 'resumed': True,
                'ros2_tree_sha256': identity['canonical_rosbag2_tree_sha256'],
                'semantic_equivalence_sha256': identity[
                    'semantic_equivalence_sha256']}
    raw_path = _relative_under_root(root, manifest['raw_bag']['path'],
                                    f'{sequence} raw_bag.path')
    ros2_path = (paths['ros2'] if paths['ros2'].exists() else
                 paths['ros2_part'] if paths['ros2_part'].exists() else None)
    if ros2_path is None:
        converter_argv = _converter_argv(raw_path, paths['ros2_part'])
        subprocess.run(converter_argv, check=True, capture_output=True, text=True)
        ros2_path = paths['ros2_part']
    _assert_safe_tree(ros2_path, root)
    topics = _validate_topics(ros2_path)
    semantic_path = (paths['semantic'] if paths['semantic'].exists() else
                     paths['semantic_part'] if paths['semantic_part'].exists() else None)
    if semantic_path is None:
        comparator_argv = _comparator_argv(raw_path, ros2_path,
                                           paths['semantic_part'])
        subprocess.run(comparator_argv, check=True, capture_output=True, text=True)
        semantic_path = paths['semantic_part']
    report = _validate_semantic_report(semantic_path)
    identity = build_ros2_identity(ros2_path, semantic_path)
    if paths['receipt_part'].exists():
        staged_receipt = _json_load(paths['receipt_part'],
                                    'staged preparation receipt')
        verified_identity = _receipt_identity(
            staged_receipt, manifest_path, manifest, marker, runtime,
            ros2_path, semantic_path)
        _publish_staged(paths)
        return {'sequence': sequence, 'status': 'prepared', 'resumed': True,
                'ros2_tree_sha256': verified_identity[
                    'canonical_rosbag2_tree_sha256'],
                'semantic_equivalence_sha256': verified_identity[
                    'semantic_equivalence_sha256']}
    converter_argv = _converter_argv(raw_path, paths['ros2_part'])
    comparator_argv = _comparator_argv(raw_path, ros2_path,
                                       paths['semantic_part'])
    receipt = {
        'schema_version': PREPARATION_SCHEMA_VERSION,
        'receipt_kind': 'competitive_fresh_ros_input_preparation',
        'status': 'prepared',
        'sequence': sequence,
        'plan_sha256': marker['plan_sha256'],
        'manifest_sha256': sha256_file(manifest_path),
        'raw_bag': {
            'path': manifest['raw_bag']['path'],
            'bytes': manifest['raw_bag']['bytes'],
            'sha256': manifest['raw_bag']['sha256'],
        },
        'ground_truth_content_opened': False,
        'runtime': runtime,
        'converter_argv': converter_argv,
        'comparator_argv': comparator_argv,
        'topics': topics,
        'ros2_tree_sha256': identity['canonical_rosbag2_tree_sha256'],
        'semantic_report_sha256': sha256_file(semantic_path),
        'semantic_equivalence_sha256': identity[
            'semantic_equivalence_sha256'],
        'semantic_equivalence_hash_kind': SEMANTIC_HASH_KIND,
        'semantic_report_all_topics_equal': report['all_topics_equal'],
    }
    _atomic_json(paths['receipt_part'], receipt)
    _publish_staged(paths)
    return {'sequence': sequence, 'status': 'prepared', 'resumed': had_artifacts,
            'ros2_tree_sha256': identity['canonical_rosbag2_tree_sha256'],
            'semantic_equivalence_sha256': identity[
                'semantic_equivalence_sha256']}


def _lock_root(root: Path) -> tuple[int, Path]:
    lock = root / '.prepare.lock'
    try:
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise ValueError(f'preparation lock exists: {lock}') from exc
    os.write(descriptor, f'{os.getpid()}\n'.encode('ascii'))
    return descriptor, lock


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path,
                        help='managed fresh-holdout root')
    scope = parser.add_mutually_exclusive_group(required=True)
    scope.add_argument('--all', action='store_true', help='prepare all manifests')
    scope.add_argument('--sequence', action='append', metavar='SEQ',
                       help='prepare one or more sequences')
    parser.add_argument('--resume', action='store_true',
                        help='resume only identity-verified staging/output')
    args = parser.parse_args(argv)
    root = args.root.expanduser().resolve()
    marker = _managed_marker(root)
    sequences = _manifest_paths(root, args.sequence)
    runtime = _probe_runtime()
    descriptor, lock = _lock_root(root)
    try:
        results = [_prepare_slot(root, sequence, args.resume, runtime, marker)
                   for sequence in sequences]
    finally:
        os.close(descriptor)
        lock.unlink(missing_ok=True)
    print(json.dumps({'status': 'prepared', 'sequences': results},
                     indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            subprocess.CalledProcessError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
