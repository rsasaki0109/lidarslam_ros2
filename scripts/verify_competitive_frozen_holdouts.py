#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Deep, read-only verification for frozen competitive holdout artifacts.

The verifier is intentionally separate from the downloader/finalizer.  It
rechecks the official selection contract, every frozen slot, preparation
receipt provenance, and the canonical ROS/input identities.  Ground truth is
treated as an opaque byte stream: this program hashes it but never parses,
prints, or includes its path in the result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lidarslam_benchmark_tools.freeze_competitive_fresh_holdouts import (  # noqa: E402
    _metadata_topics,
    _require_git_blob,
    _require_relative_path,
    _require_sha,
    _semantic_payload,
    CALIBRATION_HASH_KIND,
    calibration_tree_sha256,
    CANONICAL_TOPIC_CONTRACT,
    git_blob_sha1,
    INPUT_HASH_KIND,
    SEMANTIC_HASH_KIND,
)

import yaml  # noqa: E402


SCHEMA_VERSION = 1
SELECTION_KIND = 'm5b_fresh_holdout_selection'
PREPARATION_KIND = 'competitive_fresh_ros_input_preparation'
ROSBAGS_VERSION = '0.11.0'
SEQUENCE_RE = re.compile(r'^exp[0-9]{2}$')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
GIT_COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
PYTHON_VERSION_RE = re.compile(r'^\d+\.\d+(?:\.\d+)?$')
EXPECTED_SEQUENCES = ('exp14', 'exp16', 'exp18')
EXPECTED_CANDIDATES = ('fresh_1', 'fresh_2', 'fresh_3')


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def canonical_json_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def canonical_json_file_sha256(value: Any) -> str:
    """Hash the freezer's canonical JSON file bytes, including its newline."""
    return hashlib.sha256(_canonical_json(value) + b'\n').hexdigest()


def _sha256_file(path: Path, label: str) -> str:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'{label} is not a regular file')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _opaque_identity(path: Path, expected_bytes: Any, label: str,
                     expected_sha: str | None = None,
                     expected_blob: str | None = None) -> dict[str, Any]:
    """Hash a file without exposing its path or contents to callers."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'{label} artifact is missing or unsafe')
    if (isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or
            expected_bytes < 0):
        raise ValueError(f'{label} expected byte count is invalid')
    digest = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            size += len(block)
            digest.update(block)
    actual_sha = digest.hexdigest()
    if size != expected_bytes:
        raise ValueError(f'{label} byte count mismatch')
    if expected_sha is not None and actual_sha != expected_sha:
        raise ValueError(f'{label} SHA-256 mismatch')
    result = {'bytes': size, 'sha256': actual_sha}
    if expected_blob is not None:
        actual_blob = git_blob_sha1(path)
        if actual_blob != expected_blob:
            raise ValueError(f'{label} Git blob identity mismatch')
        result['git_blob_sha1'] = actual_blob
    return result


def _json_object(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'{label} is missing or unsafe')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a JSON object')
    return value


def _safe_relative(root: Path, value: Any, label: str) -> Path:
    relative = _require_relative_path(value, label)
    path = root / relative
    try:
        path.resolve(strict=False).relative_to(root.resolve())
    except ValueError as exc:
        raise ValueError(f'{label} escapes managed root') from exc
    return path


def _selection_lineage_shas(selection: dict[str, Any], current_sha: str) -> set[str]:
    """Return the current selection SHA plus explicitly declared frozen anchors.

    The managed root is intentionally immutable after download/finalize.  When
    the repository receipt is enriched with the identities produced by that
    root, its file SHA necessarily changes.  Only anchors declared by the
    enriched receipt can bridge that change; accepting an arbitrary historical
    SHA would make the managed artifacts replayable under an unrelated
    selection.
    """
    accepted = {current_sha}
    provenance = selection.get('selection_provenance')
    if provenance is None:
        return accepted
    if not isinstance(provenance, dict):
        raise ValueError('selection_provenance must be a mapping')
    for key in (
            'managed_root_anchor_selection_receipt_sha256',
            'frozen_manifest_source_selection_receipt_sha256'):
        value = provenance.get(key)
        if value is None:
            continue
        accepted.add(_require_sha(value, f'selection_provenance.{key}'))
    if len(accepted) == 1:
        raise ValueError('selection_provenance must declare a selection anchor')
    return accepted


def _managed_marker(root: Path, selection_path: Path,
                    accepted_selection_shas: set[str] | None = None
                    ) -> tuple[dict[str, Any], str]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError('managed root is missing or unsafe')
    marker = _json_object(root / '.freeze-root.json', 'managed-root marker')
    marker['plan_sha256'] = _require_sha(marker.get('plan_sha256'),
                                         'marker plan_sha256')
    selection_sha = _sha256_file(selection_path, 'selection receipt')
    accepted = accepted_selection_shas or {selection_sha}
    marker_selection_sha = marker.get('selection_receipt_sha256')
    if marker_selection_sha not in accepted:
        raise ValueError('marker selection receipt SHA-256 mismatch')
    destination = marker.get('destination_root')
    if destination is not None and Path(destination).resolve() != root.resolve():
        raise ValueError('marker destination root mismatch')
    return marker, selection_sha


def _load_selection(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError('selection receipt is missing or unsafe')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('receipt_kind') != SELECTION_KIND:
        raise ValueError('selection receipt kind is invalid')
    source = document.get('official_source')
    revision = source.get('revision') if isinstance(source, dict) else None
    if (not isinstance(source, dict) or not isinstance(revision, str) or
            GIT_COMMIT_RE.fullmatch(revision) is None):
        raise ValueError('selection official source is invalid')
    selected = (document.get('selection_decision') or {}).get('selected_candidates')
    if not isinstance(selected, dict) or set(selected) != set(EXPECTED_CANDIDATES):
        raise ValueError('selection must contain exactly fresh_1/fresh_2/fresh_3')
    sequences = []
    for candidate_name in EXPECTED_CANDIDATES:
        candidate = selected[candidate_name]
        if not isinstance(candidate, dict):
            raise ValueError('selection candidate is malformed')
        sequence = candidate.get('sequence')
        if not isinstance(sequence, str) or SEQUENCE_RE.fullmatch(sequence) is None:
            raise ValueError('selection candidate sequence is invalid')
        sequences.append(sequence)
    if tuple(sorted(sequences)) != EXPECTED_SEQUENCES:
        raise ValueError('selection candidate sequences are not exp14/16/18')
    return document


def _safe_tree_sha256(path: Path, root: Path, label: str) -> str:
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f'{label} tree is missing or unsafe')
    root_resolved = root.resolve()
    files = []
    for directory, directories, names in os.walk(path, followlinks=False):
        for name in (*directories, *names):
            candidate = Path(directory) / name
            if candidate.is_symlink():
                raise ValueError(f'{label} tree contains a symlink')
            try:
                candidate.resolve(strict=False).relative_to(root_resolved)
            except ValueError as exc:
                raise ValueError(f'{label} tree escapes managed root') from exc
        files.extend(Path(directory) / name for name in names)
    digest = hashlib.sha256()
    for candidate in sorted(files, key=lambda item: item.relative_to(path).as_posix()):
        if not candidate.is_file():
            raise ValueError(f'{label} tree contains a non-file')
        relative = candidate.relative_to(path).as_posix()
        digest.update(relative.encode('utf-8'))
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def _semantic_identity(path: Path) -> tuple[dict[str, Any], str, str]:
    report = _json_object(path, 'semantic report')
    semantic = _semantic_payload(report)
    return report, canonical_json_sha256(semantic), _sha256_file(path, 'semantic report')


def _base_manifest_from_final(final_manifest: dict[str, Any]) -> dict[str, Any]:
    base = json.loads(json.dumps(final_manifest))
    base['status'] = 'downloaded_hashed'
    base['canonical_rosbag2'] = None
    base['semantic_equivalence_sha256'] = None
    base['input_manifest_sha256'] = None
    base.pop('input_manifest_hash_kind', None)
    blind = dict(base.get('blind_audit') or {})
    blind.pop('canonical_rosbag2_metadata_only', None)
    blind.pop('semantic_report_payload_only', None)
    base['blind_audit'] = blind
    return base


def _verify_calibration(root: Path, manifest: dict[str, Any],
                        candidate: dict[str, Any], sequence: str) -> dict[str, Any]:
    selection_calibration = (candidate.get('_official_calibration') or {})
    calibration = manifest.get('calibration')
    if not isinstance(calibration, dict):
        raise ValueError(f'{sequence} calibration manifest is missing')
    if calibration.get('hash_kind') != CALIBRATION_HASH_KIND:
        raise ValueError(f'{sequence} calibration hash kind mismatch')
    expected_files = selection_calibration.get('relevant_files')
    if not isinstance(expected_files, dict):
        raise ValueError(f'{sequence} selection calibration is missing')
    entries = calibration.get('files')
    if not isinstance(entries, list):
        raise ValueError(f'{sequence} calibration files are missing')
    by_name = {}
    observed_entries = []
    tree_path = _require_relative_path(selection_calibration.get('tree_path'),
                                       f'{sequence} calibration tree path')
    selection_tree_oid = _require_git_blob(
        selection_calibration.get('tree_oid'),
        f'{sequence} selection calibration tree OID')
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(f'{sequence} calibration entry is malformed')
        logical = _require_relative_path(entry.get('logical_path'),
                                         f'{sequence} calibration logical path')
        name = Path(logical).name
        if name in by_name:
            raise ValueError(f'{sequence} calibration filename is duplicated')
        by_name[name] = entry
        if logical != (Path(tree_path) / name).as_posix():
            raise ValueError(f'{sequence} calibration logical path mismatch')
        path = _safe_relative(root, entry.get('path'),
                              f'{sequence} calibration path')
        expected_path = (Path('slots') / sequence / 'calibration' / 'files' / name)
        if path.relative_to(root).as_posix() != expected_path.as_posix():
            raise ValueError(f'{sequence} calibration storage path mismatch')
        expected = expected_files.get(name)
        if not isinstance(expected, dict):
            raise ValueError(f'{sequence} calibration file is not preregistered')
        expected_bytes = expected.get('expected_bytes')
        expected_blob = _require_git_blob(expected.get('git_blob_oid'),
                                          f'{sequence} calibration Git blob')
        observed = _opaque_identity(path, expected_bytes,
                                    f'{sequence} calibration file',
                                    expected_sha=entry.get('sha256'),
                                    expected_blob=expected_blob)
        if entry.get('bytes') != observed['bytes'] or entry.get(
                'git_blob_sha1') != observed['git_blob_sha1']:
            raise ValueError(f'{sequence} calibration manifest identity mismatch')
        observed_entries.append({
            'logical_path': logical,
            'path': _require_relative_path(entry.get('path'),
                                           f'{sequence} calibration path'),
            **observed,
        })
    if set(by_name) != set(expected_files):
        raise ValueError(f'{sequence} calibration file set mismatch')
    if calibration.get('tree_path') != tree_path:
        raise ValueError(f'{sequence} calibration tree path mismatch')
    manifest_tree_oid = _require_git_blob(
        calibration.get('tree_oid'), f'{sequence} calibration tree OID')
    if manifest_tree_oid != selection_tree_oid:
        raise ValueError(f'{sequence} calibration tree OID mismatch')
    tree_sha = calibration_tree_sha256(observed_entries)
    if tree_sha != calibration.get('sha256'):
        raise ValueError(f'{sequence} calibration tree hash mismatch')
    return {
        'tree_sha256': tree_sha,
        'file_count': len(observed_entries),
    }


def _runtime_and_commands(receipt: dict[str, Any], raw_path: Path, ros2_path: Path,
                          semantic_path: Path) -> None:
    runtime = receipt.get('runtime')
    if not isinstance(runtime, dict):
        raise ValueError('preparation runtime identity is missing')
    if runtime.get('rosbags_version') != ROSBAGS_VERSION:
        raise ValueError('preparation rosbags version is not 0.11.0')
    for key in ('python_version', 'numpy_version'):
        if not isinstance(runtime.get(key), str) or not runtime[key]:
            raise ValueError(f'preparation {key} identity is missing')
    if PYTHON_VERSION_RE.fullmatch(runtime['python_version']) is None:
        raise ValueError('preparation Python version is malformed')
    for key in ('converter_script_sha256', 'converter_help_sha256',
                'comparator_script_sha256'):
        _require_sha(runtime.get(key), f'preparation runtime {key}')
    converter = runtime.get('converter_executable')
    if not isinstance(converter, str):
        raise ValueError('preparation converter executable is missing')
    converter_path = Path(converter)
    if converter_path.is_symlink() or not converter_path.is_file():
        raise ValueError('preparation converter executable is unavailable')
    if _sha256_file(converter_path, 'preparation converter executable') != runtime[
            'converter_script_sha256']:
        raise ValueError('preparation converter executable hash mismatch')
    if runtime.get('comparator_script') != 'scripts/compare_rosbag_semantic_inputs.py':
        raise ValueError('preparation comparator script path mismatch')
    comparator_path = ROOT / runtime['comparator_script']
    if _sha256_file(comparator_path, 'preparation comparator script') != runtime[
            'comparator_script_sha256']:
        raise ValueError('preparation comparator script hash mismatch')
    expected_converter = [
        'rosbags-convert', '--src', str(raw_path), '--dst',
        str(ros2_path.parent / 'canonical_ros2.part'), '--dst-storage', 'sqlite3',
        '--compress', 'none', '--src-typestore', 'ros1_noetic',
        '--dst-typestore', 'copy',
    ]
    if receipt.get('converter_argv') != expected_converter:
        raise ValueError('preparation converter argv mismatch')
    report_part = semantic_path.parent / 'semantic_equivalence.json.part'
    comparator_variants = []
    for input_path in (ros2_path.parent / 'canonical_ros2.part',
                       ros2_path.parent / 'canonical_ros2'):
        command = [sys.executable, str(ROOT / runtime['comparator_script']),
                   '--left', str(raw_path), '--right', str(input_path)]
        for topic, _msg_type, _role in CANONICAL_TOPIC_CONTRACT:
            command.extend(['--topic', topic])
        command.extend(['--output', str(report_part)])
        comparator_variants.append(command)
    if receipt.get('comparator_argv') not in comparator_variants:
        raise ValueError('preparation comparator argv mismatch')


def _verify_slot(root: Path, selection: dict[str, Any], marker: dict[str, Any],
                 selection_shas: set[str], candidate_name: str,
                 candidate: dict[str, Any]) -> dict[str, Any]:
    sequence = candidate['sequence']
    slot_root = root / 'slots' / sequence
    manifest_path = slot_root / 'manifest.json'
    manifest = _json_object(manifest_path, f'{sequence} manifest')
    if manifest.get('status') != 'frozen_unopened':
        raise ValueError(f'{sequence} manifest is not frozen_unopened')
    source = manifest.get('source')
    official = selection['official_source']
    source_selection_sha = source.get('selection_receipt_sha256') if isinstance(
        source, dict) else None
    if (not isinstance(source, dict) or source.get('plan_sha256') != marker[
            'plan_sha256'] or source_selection_sha not in selection_shas or
            source.get('revision') != official['revision']):
        raise ValueError(f'{sequence} source identity mismatch')
    if manifest.get('sequence') != sequence:
        raise ValueError(f'{sequence} manifest sequence mismatch')
    expected_hash_policy = {
        'ground_truth': 'opaque_file_sha256_only_no_parser',
        'calibration': CALIBRATION_HASH_KIND,
        'input_manifest': INPUT_HASH_KIND,
        'semantic_equivalence': SEMANTIC_HASH_KIND,
    }
    if manifest.get('hash_policy') != expected_hash_policy:
        raise ValueError(f'{sequence} manifest hash policy mismatch')
    blind_audit = manifest.get('blind_audit')
    if (not isinstance(blind_audit, dict) or
            blind_audit.get('ground_truth_content_opened') is not False):
        raise ValueError(f'{sequence} manifest ground-truth opacity violation')
    raw = manifest.get('raw_bag')
    bag = candidate.get('bag')
    if not isinstance(raw, dict) or not isinstance(bag, dict):
        raise ValueError(f'{sequence} raw-bag metadata is missing')
    raw_path = _safe_relative(root, raw.get('path'), f'{sequence} raw-bag path')
    raw_relative = raw_path.relative_to(root)
    if raw_relative.parts[:3] != ('slots', sequence, 'source'):
        raise ValueError(f'{sequence} raw-bag storage path mismatch')
    expected_lfs = _require_sha(bag.get('lfs_sha256'), f'{sequence} bag LFS SHA')
    expected_bag_blob = _require_git_blob(bag.get('git_blob_oid'),
                                          f'{sequence} bag LFS pointer blob')
    observed_raw = _opaque_identity(
        raw_path, bag.get('expected_bytes'), f'{sequence} raw bag',
        expected_sha=expected_lfs)
    if raw.get('bytes') != observed_raw['bytes'] or raw.get('sha256') != observed_raw[
            'sha256']:
        raise ValueError(f'{sequence} raw-bag manifest identity mismatch')
    ground_truth = manifest.get('ground_truth')
    selected_gt = candidate.get('ground_truth')
    if not isinstance(ground_truth, dict) or not isinstance(selected_gt, dict):
        raise ValueError(f'{sequence} ground-truth metadata is missing')
    if ground_truth.get('content_opened') is not False:
        raise ValueError(f'{sequence} ground-truth opacity violation')
    gt_path = _safe_relative(root, ground_truth.get('path'),
                             f'{sequence} ground-truth path')
    gt_relative = gt_path.relative_to(root)
    if gt_relative.parts[:3] != ('slots', sequence, 'ground_truth'):
        raise ValueError(f'{sequence} ground-truth storage path mismatch')
    expected_gt_blob = _require_git_blob(selected_gt.get('git_blob_oid'),
                                         f'{sequence} ground-truth Git blob')
    expected_gt_sha = selected_gt.get('sha256')
    if expected_gt_sha is not None:
        expected_gt_sha = _require_sha(expected_gt_sha, f'{sequence} ground-truth SHA')
    observed_gt = _opaque_identity(
        gt_path, selected_gt.get('expected_bytes'), f'{sequence} ground truth',
        expected_sha=expected_gt_sha, expected_blob=expected_gt_blob)
    if (ground_truth.get('bytes') != observed_gt['bytes'] or
            ground_truth.get('sha256') != observed_gt['sha256']):
        raise ValueError(f'{sequence} ground-truth manifest identity mismatch')
    selected_archive_sha = candidate.get('calibration_archive_sha256')
    if selected_archive_sha is not None:
        selected_archive_sha = _require_sha(
            selected_archive_sha, f'{sequence} calibration archive SHA')
        if manifest['calibration'].get('archive_sha256') != selected_archive_sha:
            raise ValueError(f'{sequence} calibration archive SHA mismatch')
    candidate_with_calibration = dict(candidate)
    candidate_with_calibration['_official_calibration'] = official['calibration']
    calibration_result = _verify_calibration(
        root, manifest, candidate_with_calibration, sequence)
    ros2_path = slot_root / 'canonical_ros2'
    semantic_path = slot_root / 'semantic_equivalence.json'
    if ros2_path.is_symlink() or not ros2_path.is_dir():
        raise ValueError(f'{sequence} canonical ROS 2 tree is unsafe')
    metadata_path = ros2_path / 'metadata.yaml'
    if metadata_path.is_symlink() or not metadata_path.is_file():
        raise ValueError(f'{sequence} canonical ROS 2 metadata is unsafe')
    topics = _metadata_topics(ros2_path)
    by_name = {row['name']: row['type'] for row in topics}
    for name, msg_type, _role in CANONICAL_TOPIC_CONTRACT:
        if by_name.get(name) != msg_type:
            raise ValueError(f'{sequence} canonical topic contract mismatch')
    ros2_tree_sha = _safe_tree_sha256(ros2_path, root, f'{sequence} canonical ROS 2')
    semantic_report, semantic_sha, semantic_file_sha = _semantic_identity(semantic_path)
    semantic_payload = _semantic_payload(semantic_report)
    canonical_identity = {
        'canonical_rosbag2_tree_sha256': ros2_tree_sha,
        'topics': topics,
        'required_topic_contract': [
            {'name': name, 'type': msg_type, 'role': role}
            for name, msg_type, role in CANONICAL_TOPIC_CONTRACT],
        'semantic_equivalence_sha256': semantic_sha,
        'semantic_equivalence_hash_kind': SEMANTIC_HASH_KIND,
    }
    if manifest.get('canonical_rosbag2') != canonical_identity:
        raise ValueError(f'{sequence} canonical ROS 2 identity mismatch')
    if manifest.get('semantic_equivalence_sha256') != semantic_sha:
        raise ValueError(f'{sequence} semantic identity mismatch')
    input_payload = {
        'hash_kind': INPUT_HASH_KIND,
        'raw_bag_sha256': raw['sha256'],
        'canonical_rosbag2_tree_sha256': ros2_tree_sha,
        'semantic_equivalence_sha256': semantic_sha,
        'required_topic_contract': canonical_identity['required_topic_contract'],
    }
    input_sha = canonical_json_sha256(input_payload)
    if (manifest.get('input_manifest_hash_kind') != INPUT_HASH_KIND or
            manifest.get('input_manifest_sha256') != input_sha):
        raise ValueError(f'{sequence} input manifest identity mismatch')
    receipt_path = slot_root / 'preparation_receipt.json'
    receipt = _json_object(receipt_path, f'{sequence} preparation receipt')
    if receipt.get('schema_version') != SCHEMA_VERSION or receipt.get(
            'receipt_kind') != PREPARATION_KIND or receipt.get('status') != 'prepared':
        raise ValueError(f'{sequence} preparation receipt kind/status mismatch')
    if receipt.get('sequence') != sequence or receipt.get('plan_sha256') != marker[
            'plan_sha256']:
        raise ValueError(f'{sequence} preparation receipt plan identity mismatch')
    base_manifest_sha = canonical_json_file_sha256(
        _base_manifest_from_final(manifest))
    if receipt.get('manifest_sha256') != base_manifest_sha:
        raise ValueError(f'{sequence} preparation base manifest hash mismatch')
    if receipt.get('raw_bag') != {
            'path': raw['path'], 'bytes': raw['bytes'], 'sha256': raw['sha256']}:
        raise ValueError(f'{sequence} preparation raw identity mismatch')
    if receipt.get('ground_truth_content_opened') is not False:
        raise ValueError(f'{sequence} preparation ground-truth opacity violation')
    _runtime_and_commands(receipt, raw_path, ros2_path, semantic_path)
    if receipt.get('topics') != topics:
        raise ValueError(f'{sequence} preparation topic identity mismatch')
    if receipt.get('ros2_tree_sha256') != ros2_tree_sha:
        raise ValueError(f'{sequence} preparation ROS 2 tree hash mismatch')
    if receipt.get('semantic_report_sha256') != semantic_file_sha:
        raise ValueError(f'{sequence} preparation semantic report file hash mismatch')
    if receipt.get('semantic_equivalence_sha256') != semantic_sha or receipt.get(
            'semantic_equivalence_hash_kind') != SEMANTIC_HASH_KIND:
        raise ValueError(f'{sequence} preparation semantic identity mismatch')
    if receipt.get('semantic_report_all_topics_equal') is not True:
        raise ValueError(f'{sequence} preparation semantic report is not equal')
    manifest_file_sha = _sha256_file(manifest_path, f'{sequence} manifest')
    receipt_file_sha = _sha256_file(
        receipt_path, f'{sequence} preparation receipt')
    return {
        'candidate': candidate_name,
        'sequence': sequence,
        'status': 'PASS',
        'manifest_sha256': manifest_file_sha,
        'preparation_receipt_sha256': receipt_file_sha,
        'manifest_file_sha256': manifest_file_sha,
        'preparation_receipt_file_sha256': receipt_file_sha,
        'raw_bag_sha256': observed_raw['sha256'],
        'raw_bag_git_lfs_pointer_blob_oid': expected_bag_blob,
        'ground_truth_sha256': observed_gt['sha256'],
        'ground_truth_git_blob_sha1': observed_gt['git_blob_sha1'],
        'calibration_tree_sha256': calibration_result['tree_sha256'],
        'canonical_rosbag2_tree_sha256': ros2_tree_sha,
        'semantic_report_sha256': semantic_file_sha,
        'semantic_report_all_topics_equal': semantic_report['all_topics_equal'],
        'semantic_topics': semantic_payload['topics'],
        'semantic_equivalence_sha256': semantic_sha,
        'input_manifest_sha256': input_sha,
    }


def _atomic_output(path: Path, value: dict[str, Any]) -> None:
    if path.is_symlink():
        raise ValueError('output path must not be a symlink')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(json.dumps(value, sort_keys=True, indent=2) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--selection', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    root = args.root.expanduser().resolve()
    selection_path = args.selection.expanduser().resolve()
    result: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'verifier_kind': 'competitive_frozen_holdouts_deep_verification',
        'status': 'FAIL',
        'slots': [],
        'errors': [],
    }
    try:
        selection = _load_selection(selection_path)
        selection_sha = _sha256_file(selection_path, 'selection receipt')
        selection_shas = _selection_lineage_shas(selection, selection_sha)
        marker, _ = _managed_marker(root, selection_path, selection_shas)
        selected = selection['selection_decision']['selected_candidates']
        slots_root = root / 'slots'
        if slots_root.is_symlink() or not slots_root.is_dir():
            raise ValueError('managed root slots directory is missing or unsafe')
        slot_entries = list(slots_root.iterdir())
        if any(path.is_symlink() or not path.is_dir() for path in slot_entries):
            raise ValueError('managed root slots contain an unsafe entry')
        slot_names = sorted(path.name for path in slot_entries)
        manifests = sorted(
            path.parent.name for path in slots_root.glob('*/manifest.json'))
        if (tuple(slot_names) != EXPECTED_SEQUENCES or
                tuple(manifests) != EXPECTED_SEQUENCES):
            raise ValueError('managed root must contain exactly exp14/exp16/exp18 manifests')
        for candidate_name in EXPECTED_CANDIDATES:
            result['slots'].append(_verify_slot(
                root, selection, marker, selection_shas, candidate_name,
                selected[candidate_name]))
        result['status'] = 'PASS'
        result['selection_receipt_sha256'] = selection_sha
        result['accepted_selection_receipt_sha256'] = sorted(selection_shas)
        result['plan_sha256'] = marker['plan_sha256']
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        result['errors'].append(str(exc))
    _atomic_output(args.output.expanduser().resolve(), result)
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
