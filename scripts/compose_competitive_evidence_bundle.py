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

"""Compose a deterministic, fail-closed competitive evidence bundle.

The composer is deliberately narrower than a benchmark runner.  It consumes
an already-scored, non-GT evidence document and explicit source-root/path/hash
receipts.  It never opens a dataset, ground-truth artifact, or scorer and it
never invokes a subprocess.  Every source is reopened through a no-follow
descriptor while copying into a new staging root; a changed inode, hard link,
symlink, size, or digest aborts composition.

The default output is intentionally ``NOT_READY`` because the independent
fresh-holdout authorization is an external trust boundary.  Such a bundle is
useful as an immutable candidate, but the existing claim verifier will reject
it until authorization is complete.  A claim status can only be selected
after the caller explicitly asks for it and the authorization verifier passes;
no local signature or scorer result is fabricated here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat
import sys
import tempfile
from typing import Any, Iterable, Mapping
import unicodedata

import yaml

# Direct source-checkout invocation must resolve the canonical package without
# requiring PYTHONPATH.  Installed entry points use the package normally.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools import package_root
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_profile_sha256)
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization)
    from lidarslam_benchmark_tools.evaluate_competitive_suite_gate import (
        _v2_score_artifact_payload)
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (
        FILE_HASH_KIND, MANIFEST_FILENAME, MANIFEST_KIND,
        MANIFEST_SIDECAR_FILENAME, PROFILE_CANONICAL_HASH_KIND,
        REQUIRED_ROLES, canonical_manifest_bytes, canonical_manifest_sha256,
        sha256_bytes)
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt)
except ModuleNotFoundError:  # pragma: no cover - defensive installed fallback
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]

    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_profile_sha256)
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization)
    from lidarslam_benchmark_tools.evaluate_competitive_suite_gate import (
        _v2_score_artifact_payload)
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (
        FILE_HASH_KIND, MANIFEST_FILENAME, MANIFEST_KIND,
        MANIFEST_SIDECAR_FILENAME, PROFILE_CANONICAL_HASH_KIND,
        REQUIRED_ROLES, canonical_manifest_bytes, canonical_manifest_sha256,
        sha256_bytes)
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt)


ROOT = package_root()
SCHEMA_PATH = ROOT / 'configs/slam_benchmark_profiles/competitive_evidence_bundle_v1.schema.json'
COMPOSER_SCHEMA_PATH = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_evidence_bundle_composer_v1.schema.json')
COMPOSER_SCHEMA_VERSION = 1
COMPOSER_RECEIPT_KIND = 'competitive_slam_evidence_bundle_composition'
FAILURE_ARTIFACT_SCHEMA_VERSION = 1
# The execution receipt is separate from metric/resource bytes.  It is the
# immutable per-attempt identity that proves repetitions were actually run;
# deterministic trajectory/map/score bytes are not required to differ.
RUN_SOURCE_ROLES = ('trajectory', 'map', 'resource', 'execution')
_SHA256_RE = r'^[0-9a-f]{64}$'
_REVISION_RE = r'^[0-9a-f]{40}$'
_DRIVE_RE = re.compile(r'^[A-Za-z]:')


class CompositionError(ValueError):
    """A source or contract violation that must remain fail-closed."""


def _reject_duplicate_json_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CompositionError(f'duplicate execution receipt key: {key}')
        result[key] = value
    return result


def _sha(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64 or
            any(char not in '0123456789abcdef' for char in value)):
        raise CompositionError(f'{label} must be lowercase 64-hex')
    return value


def _size(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CompositionError(f'{label} must be a non-negative integer')
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or '\x00' in value:
        raise CompositionError(f'{label} must be a non-empty string')
    if '\\' in value or unicodedata.normalize('NFC', value) != value:
        raise CompositionError(f'{label} is not canonical POSIX text')
    if (PurePosixPath(value).is_absolute() or PureWindowsPath(value).is_absolute()
            or _DRIVE_RE.match(value)):
        raise CompositionError(f'{label} must be relative')
    parts = value.split('/')
    if any(part in {'', '.', '..'} for part in parts):
        raise CompositionError(f'{label} contains traversal or an empty component')
    normalized = PurePosixPath(value).as_posix()
    if normalized != value:
        raise CompositionError(f'{label} is not normalized')
    return value


def _contains_gt(value: str) -> bool:
    lowered = value.lower()
    components = set(lowered.replace('\\', '/').split('/'))
    return bool(components & {'gt', 'ground_truth', 'ground-truth', 'groundtruth'} or
                'ground_truth' in lowered or 'ground-truth' in lowered)


def _component(value: Any, label: str) -> str:
    if (not isinstance(value, str) or not value or '\x00' in value or
            '/' in value or '\\' in value or value in {'.', '..'} or
            unicodedata.normalize('NFC', value) != value or _contains_gt(value)):
        raise CompositionError(f'{label} is not a safe identity component')
    return value


def _regular_path(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise CompositionError(f'{label} is missing, symlinked, or not a file')
    try:
        info = path.lstat()
    except OSError as exc:
        raise CompositionError(f'{label} cannot be inspected') from exc
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise CompositionError(f'{label} must be a single-link regular file')
    return path


def _read_nofollow(path: Path, label: str) -> bytes:
    """Read one stable, single-link regular file through an owned FD."""
    nofollow = getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(path, os.O_RDONLY | nofollow)
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise CompositionError(f'{label} must be a single-link regular file')
        with os.fdopen(fd, 'rb', closefd=True) as stream:
            fd = -1
            data = stream.read()
            after = os.fstat(stream.fileno())
        if (after.st_dev != before.st_dev or after.st_ino != before.st_ino or
                after.st_nlink != 1 or after.st_size != before.st_size):
            raise CompositionError(f'{label} changed during read')
        return data
    finally:
        if fd >= 0:
            os.close(fd)


def _safe_document(path_value: Any, label: str, *, base: Path) -> tuple[dict[str, Any], str]:
    path = _document_path(path_value, label, base=base)
    data = _read_nofollow(path, label)
    digest = hashlib.sha256(data).hexdigest()
    try:
        document = yaml.safe_load(data.decode('utf-8'))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise CompositionError(f'{label} is not UTF-8 YAML/JSON') from exc
    if not isinstance(document, dict):
        raise CompositionError(f'{label} must contain a mapping')
    return document, digest


def _document_path(path_value: Any, label: str, *, base: Path) -> Path:
    if isinstance(path_value, Path):
        path_value = str(path_value)
    if not isinstance(path_value, str) or not path_value:
        raise CompositionError(f'{label} is missing')
    if _contains_gt(path_value):
        raise CompositionError(f'{label} points at a forbidden GT path')
    path = Path(path_value)
    if not path.is_absolute():
        path = base / path
    return _regular_path(path, label)


def _recheck_document(
        path_value: Any, label: str, *, base: Path, expected_sha256: str) -> None:
    """Confirm an input document stayed byte-identical through composition."""
    path = _document_path(path_value, label, base=base)
    observed = hashlib.sha256(_read_nofollow(path, label)).hexdigest()
    if observed != expected_sha256:
        raise CompositionError(f'{label} changed during composition')


def _source_root(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or not Path(value).is_absolute():
        raise CompositionError(f'{label} must be an absolute source root')
    if _contains_gt(value):
        raise CompositionError(f'{label} points at a forbidden GT path')
    root = Path(value)
    if root.is_symlink() or not root.is_dir():
        raise CompositionError(f'{label} is missing, symlinked, or not a directory')
    return root.resolve(strict=True)


def _source_descriptor(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise CompositionError(f'{label} must be a mapping')
    root = _source_root(value.get('source_root'), f'{label}.source_root')
    relative = _safe_relative(value.get('source_path'), f'{label}.source_path')
    if _contains_gt(relative):
        raise CompositionError(f'{label}.source_path contains a forbidden GT component')
    receipt_sha = value.get('receipt_sha256')
    if receipt_sha is not None:
        receipt_sha = _sha(receipt_sha, f'{label}.receipt_sha256')
    return {
        'source_root': root,
        'source_path': relative,
        'size_bytes': _size(value.get('size_bytes'), f'{label}.size_bytes'),
        'sha256': _sha(value.get('sha256'), f'{label}.sha256'),
        **({'receipt_sha256': receipt_sha} if receipt_sha is not None else {}),
    }


def _read_verified_source(source: Mapping[str, Any], label: str) -> bytes:
    """Reopen a declared source and verify the receipt before parsing bytes."""
    path = _safe_source_path(source['source_root'], source['source_path'], label)
    data = _read_nofollow(path, label)
    observed = hashlib.sha256(data).hexdigest()
    if len(data) != source['size_bytes'] or observed != source['sha256']:
        raise CompositionError(
            f'{label} source receipt drift: expected {source["size_bytes"]}/'
            f'{source["sha256"]}, observed {len(data)}/{observed}')
    return data


def _authorization_failure_log(authorization: Mapping[str, Any],
                               label: str) -> Mapping[str, Any]:
    auth = authorization.get('authorization', authorization)
    failure_log = auth.get('failure_log') if isinstance(auth, Mapping) else None
    if (not isinstance(failure_log, Mapping) or
            failure_log.get('complete') is not True or
            not isinstance(failure_log.get('events'), list)):
        raise CompositionError(
            f'{label} must contain a complete list-valued failure_log')
    for index, event in enumerate(failure_log['events']):
        if not isinstance(event, Mapping):
            raise CompositionError(f'{label}.failure_log.events[{index}] is malformed')
    return failure_log


def _canonical_failure_artifact_bytes(failure_log: Mapping[str, Any]) -> bytes:
    payload = {
        'schema_version': FAILURE_ARTIFACT_SCHEMA_VERSION,
        'failure_log': failure_log,
    }
    try:
        encoded = json.dumps(
            payload, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
            allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise CompositionError(
            'authorization failure_log is not canonical JSON') from exc
    return (encoded + '\n').encode('utf-8')


def _validate_failure_source(source: Mapping[str, Any],
                             authorization: Mapping[str, Any]) -> None:
    """Require the claim bundle failure source to equal the sealed log."""
    failure_log = _authorization_failure_log(authorization, 'authorization')
    expected = _canonical_failure_artifact_bytes(failure_log)
    observed = _read_verified_source(source, 'global failure artifact')
    if observed != expected:
        raise CompositionError(
            'global failure artifact must be canonical authorization failure_log JSON')
    try:
        def object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
            result: dict[str, Any] = {}
            for key, value in pairs:
                if key in result:
                    raise CompositionError(
                        'global failure artifact contains duplicate JSON keys')
                result[key] = value
            return result

        parsed = json.loads(observed.decode('utf-8'), object_pairs_hook=object_pairs)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CompositionError(
            'global failure artifact must be one UTF-8 JSON document') from exc
    if parsed != {
            'schema_version': FAILURE_ARTIFACT_SCHEMA_VERSION,
            'failure_log': failure_log,
    }:
        raise CompositionError(
            'global failure artifact does not exactly match authorization failure_log')


def _safe_source_path(root: Path, relative: str, label: str) -> Path:
    root = Path(root)
    current = root
    for index, component in enumerate(relative.split('/')):
        current = current / component
        try:
            info = current.lstat()
        except FileNotFoundError as exc:
            raise CompositionError(f'{label} is missing') from exc
        if stat.S_ISLNK(info.st_mode):
            raise CompositionError(f'{label} contains a symlink')
        if index < len(relative.split('/')) - 1 and not stat.S_ISDIR(info.st_mode):
            raise CompositionError(f'{label} has a non-directory ancestor')
    if not stat.S_ISREG(current.stat().st_mode) or current.stat().st_nlink != 1:
        raise CompositionError(f'{label} must be a single-link regular file')
    return current


def _copy_verified(source: Mapping[str, Any], destination: Path, label: str) -> dict[str, Any]:
    root = source['source_root']
    relative = source['source_path']
    source_path = _safe_source_path(root, relative, label)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        raise CompositionError(f'{label} destination already exists')
    nofollow = getattr(os, 'O_NOFOLLOW', 0)
    source_fd = os.open(source_path, os.O_RDONLY | nofollow)
    destination_fd = -1
    digest = hashlib.sha256()
    copied = 0
    try:
        before = os.fstat(source_fd)
        if (not stat.S_ISREG(before.st_mode) or before.st_nlink != 1):
            raise CompositionError(f'{label} source is not a single-link regular file')
        destination_fd = os.open(
            destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(source_fd, 'rb', closefd=True) as source_stream, \
                os.fdopen(destination_fd, 'wb', closefd=True) as destination_stream:
            source_fd = -1
            destination_fd = -1
            for block in iter(lambda: source_stream.read(1024 * 1024), b''):
                destination_stream.write(block)
                digest.update(block)
                copied += len(block)
            destination_stream.flush()
            os.fsync(destination_stream.fileno())
            after = os.fstat(source_stream.fileno())
            if (after.st_dev != before.st_dev or after.st_ino != before.st_ino or
                    after.st_nlink != 1 or after.st_size != before.st_size):
                raise CompositionError(f'{label} source changed during copy')
    except Exception:
        if source_fd >= 0:
            os.close(source_fd)
        if destination_fd >= 0:
            os.close(destination_fd)
        try:
            destination.unlink()
        except FileNotFoundError:
            pass
        raise
    actual = digest.hexdigest()
    if copied != source['size_bytes'] or actual != source['sha256']:
        try:
            destination.unlink()
        except FileNotFoundError:
            pass
        raise CompositionError(
            f'{label} source receipt drift: expected {source["size_bytes"]}/'
            f'{source["sha256"]}, observed {copied}/{actual}')
    result = {'size_bytes': copied, 'sha256': actual}
    if source.get('receipt_sha256') is not None:
        result['receipt_sha256'] = source['receipt_sha256']
    return result


def _write_new(path: Path, data: bytes, label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise CompositionError(f'{label} destination already exists')
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'wb') as stream:
            fd = -1
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if fd >= 0:
            os.close(fd)


def _sidecar(path: str, digest: str) -> bytes:
    return f'{digest}  {path}\n'.encode('utf-8')


def _artifact_ref(
        root: Path, relative: str, payload: Mapping[str, Any], label: str) -> dict[str, Any]:
    digest = str(payload['sha256'])
    sidecar_path = relative + '.sha256'
    sidecar_bytes = _sidecar(relative, digest)
    _write_new(root / sidecar_path, sidecar_bytes, f'{label} sidecar')
    return {
        'path': relative,
        'size_bytes': int(payload['size_bytes']),
        'sha256': digest,
        **({'receipt_sha256': payload['receipt_sha256']}
           if payload.get('receipt_sha256') is not None else {}),
        'sidecar_path': sidecar_path,
        'sidecar_sha256': sha256_bytes(sidecar_bytes),
    }


def _score_bytes(system: str, run: Mapping[str, Any]) -> bytes:
    payload = _v2_score_artifact_payload(system, run)
    return (json.dumps(payload, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _validate_run(
        system: str, run: Mapping[str, Any], *,
        require_score_digest: bool = True) -> dict[str, Any]:
    dataset = _component(run.get('dataset'), f'{system}.run.dataset')
    index = run.get('run_index')
    if isinstance(index, bool) or not isinstance(index, int) or index < 1:
        raise CompositionError(f'{system}:{dataset} run_index is invalid')
    required = ('complete', 'process_exit_status', 'trajectory_complete',
                'sequence_failure', 'catastrophic_failure', 'verified_false_loops')
    if any(key not in run for key in required):
        raise CompositionError(f'{system}:{dataset}:{index} completion fields are incomplete')
    if (run.get('complete') is not True or run.get('process_exit_status') != 0 or
            run.get('trajectory_complete') is not True or
            run.get('sequence_failure') is not False or
            run.get('catastrophic_failure') is not False or
            run.get('verified_false_loops') != 0):
        raise CompositionError(f'{system}:{dataset}:{index} is not a complete successful run')
    campaign_id = _sha(
        run.get('campaign_id'),
        f'{system}:{dataset}:{index} campaign_id')
    trajectory = run.get('trajectory')
    runtime = run.get('runtime')
    mapping = run.get('map', run.get('mapping'))
    if not isinstance(trajectory, Mapping) or not isinstance(runtime, Mapping) or \
            not isinstance(mapping, Mapping):
        raise CompositionError(f'{system}:{dataset}:{index} score mappings are incomplete')
    for value, label, positive in (
            (trajectory.get('ape_rmse_m'), 'APE', True),
            (runtime.get('processing_rtf'), 'processing RTF', False),
            (runtime.get('peak_rss_mb'), 'peak RSS', True),
            (mapping.get('plane_thickness_mean_m'), 'map mean', True),
            (mapping.get('plane_thickness_p95_m'), 'map p95', True),
            (mapping.get('planar_coverage'), 'map coverage', False)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not math.isfinite(float(value)) or (positive and float(value) <= 0) or \
                (not positive and float(value) < 0):
            raise CompositionError(f'{system}:{dataset}:{index} {label} is invalid')
    artifacts = run.get('artifacts')
    if not isinstance(artifacts, Mapping):
        raise CompositionError(f'{system}:{dataset}:{index} artifacts are missing')
    trajectory_sha = _sha(artifacts.get('trajectory_sha256'),
                          f'{system}:{dataset}:{index} trajectory_sha256')
    map_sha = _sha(artifacts.get('map_sha256'),
                   f'{system}:{dataset}:{index} map_sha256')
    resource = None
    for key in ('resource_evidence', 'resource_receipt', 'resource'):
        if isinstance(run.get(key), Mapping):
            resource = run[key]
            break
    if resource is None:
        raise CompositionError(f'{system}:{dataset}:{index} resource evidence is missing')
    if (resource.get('functional_only') is True or
            resource.get('performance_gate_eligible') is False):
        raise CompositionError(f'{system}:{dataset}:{index} functional-only resource is forbidden')
    timing = str(resource.get('timing_authority', '')).upper()
    if 'CONTAMINATED' in timing or 'NON_AUTHORITATIVE' in timing:
        raise CompositionError(f'{system}:{dataset}:{index} contaminated resource is forbidden')
    resource_sha = _sha(
        resource.get('receipt_sha256', resource.get('evidence_sha256')),
        f'{system}:{dataset}:{index} resource receipt SHA')
    execution_receipt_sha = _sha(
        run.get('execution_receipt_sha256'),
        f'{system}:{dataset}:{index} execution receipt SHA')
    execution_file_sha = _sha(
        run.get('execution_receipt_file_sha256'),
        f'{system}:{dataset}:{index} execution receipt file SHA')
    score_data = _score_bytes(system, run)
    observed_score_sha = hashlib.sha256(score_data).hexdigest()
    if require_score_digest:
        declared_score_sha = _sha(
            run.get('score_artifact_sha256'),
            f'{system}:{dataset}:{index} score_artifact_sha256')
        if declared_score_sha != observed_score_sha:
            raise CompositionError(
                f'{system}:{dataset}:{index} scored evidence digest drift')
    return {
        'system': _component(system, 'system'),
        'dataset': dataset,
        'run_index': index,
        'campaign_id': campaign_id,
        'trajectory_sha256': trajectory_sha,
        'map_sha256': map_sha,
        'resource_sha256': resource_sha,
        'execution_receipt_sha256': execution_receipt_sha,
        'execution_receipt_file_sha256': execution_file_sha,
        'score_bytes': score_data,
    }


def _collect_runs(
        evidence: Mapping[str, Any], *,
        require_score_digest: bool = True,
) -> dict[tuple[str, str, int], dict[str, Any]]:
    systems = evidence.get('systems')
    if not isinstance(systems, Mapping) or not systems:
        raise CompositionError('evidence.systems must be a non-empty mapping')
    collected: dict[tuple[str, str, int], dict[str, Any]] = {}
    for raw_system, record in systems.items():
        system = _component(raw_system, 'system')
        if not isinstance(record, Mapping) or not isinstance(record.get('runs'), list):
            raise CompositionError(f'{system}.runs is missing or not a list')
        for run in record['runs']:
            if not isinstance(run, Mapping):
                raise CompositionError(f'{system}.runs contains a non-mapping')
            normalized = _validate_run(
                system, run, require_score_digest=require_score_digest)
            key = (normalized['system'], normalized['dataset'], normalized['run_index'])
            if key in collected:
                raise CompositionError(f'duplicate evidence run identity: {key}')
            collected[key] = normalized
    campaigns = {item['campaign_id'] for item in collected.values()}
    if len(campaigns) != 1:
        raise CompositionError(
            'scored evidence runs must bind one immutable campaign identity')
    return collected


def _collect_global_sources(spec: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    raw = spec.get('artifacts')
    if not isinstance(raw, list):
        raise CompositionError('spec.artifacts must be a list')
    sources: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping):
            raise CompositionError(f'artifacts[{index}] must be a mapping')
        role = item.get('role')
        if not isinstance(role, str) or role not in REQUIRED_ROLES:
            raise CompositionError(f'artifacts[{index}].role is not a required role')
        if role in sources:
            raise CompositionError(f'duplicate global artifact role: {role}')
        sources[role] = _source_descriptor(item, f'artifacts[{index}]')
    missing = sorted(set(REQUIRED_ROLES) - set(sources))
    if missing or len(sources) != len(raw):
        raise CompositionError('global artifact roles are incomplete: ' + ','.join(missing))
    return sources


def _collect_run_sources(
        spec: Mapping[str, Any],
        expected: Mapping[tuple[str, str, int], Mapping[str, Any]],
) -> dict[tuple[str, str, int], dict[str, dict[str, Any]]]:
    raw = spec.get('run_artifact_bindings')
    if not isinstance(raw, list):
        raise CompositionError('spec.run_artifact_bindings must be a list')
    sources: dict[tuple[str, str, int], dict[str, dict[str, Any]]] = {}
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping):
            raise CompositionError(f'run_artifact_bindings[{index}] must be a mapping')
        system = _component(item.get('system'), f'run_artifact_bindings[{index}].system')
        dataset = _component(item.get('dataset'), f'run_artifact_bindings[{index}].dataset')
        run_index = item.get('run_index')
        if isinstance(run_index, bool) or not isinstance(run_index, int) or run_index < 1:
            raise CompositionError(f'run_artifact_bindings[{index}].run_index is invalid')
        key = (system, dataset, run_index)
        if key in sources:
            raise CompositionError(f'duplicate source run identity: {key}')
        if key not in expected:
            raise CompositionError(f'source run identity is not in evidence: {key}')
        role_sources: dict[str, dict[str, Any]] = {}
        for role in RUN_SOURCE_ROLES:
            role_sources[role] = _source_descriptor(
                item.get(role), f'run_artifact_bindings[{index}].{role}')
            expected_hash = (
                expected[key]['execution_receipt_file_sha256']
                if role == 'execution' else expected[key][f'{role}_sha256'])
            if role_sources[role]['sha256'] != expected_hash:
                raise CompositionError(
                    f'{key}/{role} source receipt does not match scored evidence')
            if role == 'execution':
                expected_receipt = expected[key]['execution_receipt_sha256']
                if role_sources[role].get('receipt_sha256') != expected_receipt:
                    raise CompositionError(
                        f'{key}/execution canonical receipt identity does not '
                        'match scored evidence')
                try:
                    payload = _read_nofollow(
                        _safe_source_path(
                            role_sources[role]['source_root'],
                            role_sources[role]['source_path'],
                            f'{key}/execution'),
                        f'{key}/execution')
                    document = json.loads(
                        payload.decode('utf-8'),
                        object_pairs_hook=_reject_duplicate_json_keys)
                    validate_receipt(
                        document, expected_system=system,
                        expected_dataset=dataset, expected_run_index=run_index,
                        expected_campaign_id=expected[key]['campaign_id'],
                        require_success=True, expected_sha256=expected_receipt)
                except (UnicodeDecodeError, json.JSONDecodeError,
                        CompositionError, ExecutionReceiptError, OSError,
                        TypeError) as exc:
                    raise CompositionError(
                        f'{key}/execution receipt is invalid: {exc}') from exc
        sources[key] = role_sources
    missing = sorted(set(expected) - set(sources))
    if missing or len(sources) != len(raw):
        raise CompositionError(f'run artifact coverage is incomplete: {missing}')
    return sources


def _reject_source_aliases(
        global_sources: Mapping[str, Mapping[str, Any]],
        run_sources: Mapping[tuple[str, str, int], Mapping[str, Mapping[str, Any]]]) -> None:
    """Reject one physical source file being reused for multiple roles/runs."""
    seen: dict[tuple[int, int], str] = {}
    descriptors: list[tuple[str, Mapping[str, Any]]] = []
    descriptors.extend((f'global/{role}', source)
                       for role, source in global_sources.items())
    for key, sources in run_sources.items():
        identity = ':'.join((str(key[0]), str(key[1]), str(key[2])))
        descriptors.extend((f'run/{identity}/{role}', source)
                           for role, source in sources.items())
    for label, source in descriptors:
        path = _safe_source_path(source['source_root'], source['source_path'], label)
        info = path.stat()
        inode = (info.st_dev, info.st_ino)
        prior = seen.get(inode)
        if prior is not None:
            raise CompositionError(f'source file alias is reused by {prior} and {label}')
        seen[inode] = label


def _metadata(spec: Mapping[str, Any], sources: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    profile = spec.get('profile')
    scorer = spec.get('scorer')
    revision = spec.get('revision')
    if not isinstance(profile, Mapping) or not isinstance(scorer, Mapping) or \
            not isinstance(revision, Mapping):
        raise CompositionError('profile, scorer, and revision metadata are required')
    profile_name = _component(profile.get('name'), 'profile.name')
    profile_canonical = _sha(profile.get('canonical_sha256'), 'profile.canonical_sha256')
    scorer_fingerprint = _sha(scorer.get('fingerprint'), 'scorer.fingerprint')
    systems = revision.get('systems')
    if not isinstance(systems, Mapping) or not systems:
        raise CompositionError('revision.systems must be a non-empty mapping')
    for system, value in systems.items():
        _component(system, 'revision.system')
        if not isinstance(value, str) or len(value) != 40 or \
                any(char not in '0123456789abcdef' for char in value):
            raise CompositionError(f'revision.systems.{system} must be lowercase 40-hex')
    config = sources['config']
    config_bytes = _read_verified_source(config, 'config source')
    try:
        config_document = yaml.safe_load(config_bytes.decode('utf-8'))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise CompositionError('config source is not valid UTF-8 YAML') from exc
    if not isinstance(config_document, Mapping):
        raise CompositionError('config source must contain a mapping')
    if canonical_profile_sha256(config_document) != profile_canonical:
        raise CompositionError('profile canonical SHA does not match config source')
    document = config_document.get('competitive_slam_profile', config_document)
    if not isinstance(document, Mapping) or document.get('name') != profile_name:
        raise CompositionError('profile name does not match config source')
    revision = sources['revision']
    revision_bytes = _read_verified_source(revision, 'revision source')
    try:
        revision_document = json.loads(revision_bytes.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CompositionError('revision source is not valid UTF-8 JSON') from exc
    revision_identity = (
        revision_document.get('systems')
        if isinstance(revision_document, Mapping) and
        isinstance(revision_document.get('systems'), Mapping)
        else revision_document)
    if not isinstance(revision_identity, Mapping):
        raise CompositionError('revision source must contain a systems mapping')
    normalized_declared = {
        str(system): str(value).lower() for system, value in systems.items()}
    normalized_observed = {
        str(system): str(value).lower()
        for system, value in revision_identity.items()}
    if normalized_observed != normalized_declared:
        raise CompositionError('revision metadata does not match revision source')
    score_handoff = spec.get('post_score_authorization')
    if score_handoff is not None:
        if not isinstance(score_handoff, Mapping) or score_handoff.get('status') != 'PASS':
            raise CompositionError('post-score authorization must be PASS')
        receipt_sha = _sha(
            score_handoff.get('receipt_sha256'),
            'post_score_authorization.receipt_sha256')
        handoff_fingerprint = _sha(
            score_handoff.get('scorer_fingerprint'),
            'post_score_authorization.scorer_fingerprint')
        handoff_evidence_sha = _sha(
            score_handoff.get('input_evidence_sha256'),
            'post_score_authorization.input_evidence_sha256')
        scored_evidence_sha = _sha(
            score_handoff.get('scored_evidence_sha256'),
            'post_score_authorization.scored_evidence_sha256')
        if handoff_fingerprint != scorer_fingerprint:
            raise CompositionError('post-score authorization scorer fingerprint drift')
        score_handoff = {
            'status': 'PASS',
            'receipt_sha256': receipt_sha,
            'scorer_fingerprint': handoff_fingerprint,
            'input_evidence_sha256': handoff_evidence_sha,
            'scored_evidence_sha256': scored_evidence_sha,
        }
    return {
        'profile_name': profile_name,
        'profile_canonical_sha256': profile_canonical,
        'scorer_fingerprint': scorer_fingerprint,
        'revision_systems': {str(key): value for key, value in systems.items()},
        'score_handoff': score_handoff,
    }


def _manifest(
        *, status: str, metadata: Mapping[str, Any], global_refs: Mapping[str, Mapping[str, Any]],
        run_refs: Iterable[Mapping[str, Any]], authorization: Any) -> dict[str, Any]:
    by_role = dict(global_refs)
    manifest: dict[str, Any] = {
        'schema_version': 1,
        'manifest_kind': MANIFEST_KIND,
        'manifest_path': MANIFEST_FILENAME,
        'canonical_root': '.',
        'claim_status': status,
        'manifest_sidecar_path': MANIFEST_SIDECAR_FILENAME,
        'profile': {
            'path': by_role['config']['path'],
            'sha256': by_role['config']['sha256'],
            'sha256_kind': FILE_HASH_KIND,
            'canonical_sha256': metadata['profile_canonical_sha256'],
            'canonical_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
            'name': metadata['profile_name'],
        },
        'scorer': {
            'path': by_role['scorer']['path'],
            'sha256': by_role['scorer']['sha256'],
            'fingerprint': metadata['scorer_fingerprint'],
        },
        'revision': {
            'path': by_role['revision']['path'],
            'sha256': by_role['revision']['sha256'],
            'systems': metadata['revision_systems'],
        },
        'artifacts': [dict(by_role[role], role=role) for role in REQUIRED_ROLES],
        'run_artifact_bindings': list(run_refs),
    }
    if authorization is not None:
        manifest['authorization'] = authorization
    if metadata.get('score_handoff') is not None:
        manifest['score_handoff'] = dict(metadata['score_handoff'])
    manifest['manifest_sha256'] = canonical_manifest_sha256(manifest)
    sidecar = _sidecar(MANIFEST_FILENAME, manifest['manifest_sha256'])
    manifest['manifest_sidecar_sha256'] = sha256_bytes(sidecar)
    return manifest


def _schema_status(manifest: Mapping[str, Any]) -> dict[str, Any]:
    if manifest.get('claim_status') != 'claim_eligible':
        return {
            'status': 'NOT_READY', 'pass': False,
            'reason': 'ineligible candidate manifests are intentionally not schema-claimable',
        }
    try:
        import jsonschema
        schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
        resolver = jsonschema.RefResolver(SCHEMA_PATH.as_uri(), schema)
        validator = jsonschema.Draft7Validator(schema, resolver=resolver)
        errors = sorted(validator.iter_errors(manifest), key=lambda item: list(item.path))
        return {'status': 'PASS' if not errors else 'FAIL_CLOSED', 'pass': not errors,
                'errors': [error.message for error in errors]}
    except (OSError, ImportError, ValueError, TypeError) as exc:
        return {'status': 'FAIL_CLOSED', 'pass': False,
                'errors': [f'schema validation unavailable: {exc}']}


def _validate_composition_spec_schema(spec: Mapping[str, Any]) -> None:
    """Validate the producer input before opening any evidence source."""
    try:
        import jsonschema
        schema = json.loads(COMPOSER_SCHEMA_PATH.read_text(encoding='utf-8'))
        validator = jsonschema.Draft7Validator(schema)
        errors = sorted(validator.iter_errors(spec), key=lambda item: list(item.path))
    except (OSError, ImportError, ValueError, TypeError) as exc:
        raise CompositionError(
            f'composition input schema is unavailable: {exc}') from exc
    if errors:
        messages = '; '.join(error.message for error in errors[:8])
        raise CompositionError(f'composition spec schema invalid: {messages}')


def _seal_tree(root: Path) -> None:
    for directory, dirnames, filenames in os.walk(root, topdown=False, followlinks=False):
        for name in filenames:
            path = Path(directory) / name
            if path.is_symlink() or not path.is_file():
                raise CompositionError('output tree contains a non-regular file')
            path.chmod(0o444)
        for name in dirnames:
            path = Path(directory) / name
            if path.is_symlink() or not path.is_dir():
                raise CompositionError('output tree contains a symlink or non-directory')
            path.chmod(0o555)
    root.chmod(0o555)


def compose_bundle(
        spec_path: Path, output_root: Path, *, allow_claim: bool = False) -> dict[str, Any]:
    """Compose one new bundle root, returning a sealed composition receipt."""
    spec_sha = ''
    evidence_sha = ''
    staging: Path | None = None
    spec, spec_sha = _safe_document(spec_path, 'composition spec', base=Path.cwd())
    _validate_composition_spec_schema(spec)
    if spec.get('invoke_scorer') is True or spec.get('scorer_command') is not None:
        raise CompositionError('composer never invokes a scorer')
    evidence_value = spec.get('evidence_path')
    evidence, evidence_sha = _safe_document(
        evidence_value, 'scored evidence', base=spec_path.resolve().parent)
    score_handoff = spec.get('post_score_authorization')
    if isinstance(score_handoff, Mapping):
        declared_evidence_sha = _sha(
            score_handoff.get('scored_evidence_sha256'),
            'post_score_authorization.scored_evidence_sha256')
        if declared_evidence_sha != evidence_sha:
            raise CompositionError(
                'post-score authorization is not bound to scored evidence bytes')
    runs = _collect_runs(evidence)
    global_sources = _collect_global_sources(spec)
    authorization = spec.get('authorization')
    if authorization is not None and not isinstance(authorization, Mapping):
        raise CompositionError('spec.authorization must be a mapping when present')
    metadata = _metadata(spec, global_sources)
    run_sources = _collect_run_sources(spec, runs)
    _reject_source_aliases(global_sources, run_sources)
    output = Path(output_root)
    if _contains_gt(str(output)):
        raise CompositionError('output root contains a forbidden GT component')
    if output.exists() or output.is_symlink():
        raise CompositionError('output root already exists; overwrite is forbidden')
    parent = output.parent
    if not parent.is_dir() or parent.is_symlink():
        raise CompositionError('output parent must be an existing non-symlink directory')
    staging = Path(tempfile.mkdtemp(prefix=output.name + '.staging.', dir=parent))
    try:
        global_refs: dict[str, dict[str, Any]] = {}
        global_destinations = {
            role: f'artifacts/{role}.artifact' for role in REQUIRED_ROLES}
        for role in REQUIRED_ROLES:
            copied = _copy_verified(
                global_sources[role], staging / global_destinations[role],
                f'global artifact {role}')
            global_refs[role] = _artifact_ref(
                staging, global_destinations[role], copied, f'global artifact {role}')
        run_refs: list[dict[str, Any]] = []
        for key in sorted(runs):
            system, dataset, run_index = key
            root_relative = f'runs/{system}/{dataset}/{run_index}'
            refs: dict[str, dict[str, Any]] = {}
            for role in RUN_SOURCE_ROLES:
                destination = f'{root_relative}/{role}.artifact'
                copied = _copy_verified(
                    run_sources[key][role], staging / destination,
                    f'run artifact {system}:{dataset}:{run_index}/{role}')
                refs[role] = _artifact_ref(
                    staging, destination, copied,
                    f'run artifact {system}:{dataset}:{run_index}/{role}')
            score_destination = f'{root_relative}/score.json'
            score_data = runs[key]['score_bytes']
            _write_new(staging / score_destination, score_data, 'run score artifact')
            refs['score'] = _artifact_ref(
                staging, score_destination,
                {'size_bytes': len(score_data), 'sha256': hashlib.sha256(score_data).hexdigest()},
                'run score artifact')
            run_refs.append({
                'system': system, 'dataset': dataset, 'run_index': run_index,
                **refs,
            })
        _recheck_document(
            spec_path, 'composition spec', base=Path.cwd(), expected_sha256=spec_sha)
        _recheck_document(
            evidence_value, 'scored evidence', base=spec_path.resolve().parent,
            expected_sha256=evidence_sha)
        requested_status = 'claim_eligible' if allow_claim else 'NOT_READY'
        candidate = _manifest(
            status=requested_status, metadata=metadata, global_refs=global_refs,
            run_refs=run_refs, authorization=authorization)
        authorization_result = {'status': 'NOT_READY', 'pass': False,
                                'reason': 'external authorization was not requested'}
        if allow_claim:
            if not isinstance(authorization, Mapping):
                authorization_result = {
                    'status': 'NOT_READY', 'pass': False,
                    'reason': 'an explicit authorization chain is required',
                }
            else:
                config_document = yaml.safe_load(
                    (staging / global_refs['config']['path']).read_text(encoding='utf-8'))
                profile_document = (
                    config_document if isinstance(config_document, Mapping) and
                    'competitive_slam_profile' in config_document else
                    {'competitive_slam_profile': config_document})
                authorization_result = verify_fresh_holdout_authorization(
                    candidate,
                    policy=spec.get('authorization_policy', {'required': True}),
                    profile=profile_document,
                    expected_profile_sha256=metadata['profile_canonical_sha256'])
            if (authorization_result.get('pass') is not True or
                    authorization_result.get('status') != 'PASS' or
                    authorization_result.get('required') is not True):
                candidate = _manifest(
                    status='NOT_READY', metadata=metadata, global_refs=global_refs,
                    run_refs=run_refs, authorization=authorization)
            else:
                _validate_failure_source(global_sources['failure'], authorization)
        sidecar = _sidecar(MANIFEST_FILENAME, candidate['manifest_sha256'])
        _write_new(staging / MANIFEST_FILENAME, canonical_manifest_bytes(candidate), 'manifest')
        _write_new(staging / MANIFEST_SIDECAR_FILENAME, sidecar, 'manifest sidecar')
        schema_result = _schema_status(candidate)
        _seal_tree(staging)
        os.rename(staging, output)
        return {
            'schema_version': COMPOSER_SCHEMA_VERSION,
            'receipt_kind': COMPOSER_RECEIPT_KIND,
            'status': 'PASS' if candidate['claim_status'] == 'claim_eligible' else 'NOT_READY',
            'pass': candidate['claim_status'] == 'claim_eligible',
            'claim_eligible': candidate['claim_status'] == 'claim_eligible',
            'output_root': str(output),
            'manifest_path': MANIFEST_FILENAME,
            'manifest_sha256': candidate['manifest_sha256'],
            'manifest_file_sha256': hashlib.sha256(
                canonical_manifest_bytes(candidate)).hexdigest(),
            'spec_sha256': spec_sha,
            'evidence_sha256': evidence_sha,
            'run_count': len(runs),
            'global_artifact_count': len(global_refs),
            'authorization': authorization_result,
            'schema_validation': schema_result,
            'errors': [],
        }
    except Exception as exc:
        failure_path = staging / 'composition.failure.json' if staging is not None else None
        try:
            if failure_path is None:
                raise OSError('composition staging root was not created')
            failure_path.write_text(json.dumps({
                'schema_version': COMPOSER_SCHEMA_VERSION,
                'receipt_kind': COMPOSER_RECEIPT_KIND,
                'status': 'FAIL_CLOSED', 'error': str(exc),
                'spec_sha256': spec_sha, 'evidence_sha256': evidence_sha,
                'staging_root': str(staging),
            }, sort_keys=True, indent=2) + '\n', encoding='utf-8')
            failure_path.chmod(0o444)
            _seal_tree(staging)
        except (OSError, CompositionError):
            pass
        if isinstance(exc, CompositionError):
            raise
        raise CompositionError(str(exc)) from exc


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--receipt', type=Path)
    parser.add_argument('--allow-claim', action='store_true')
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        result = compose_bundle(args.spec, args.output, allow_claim=args.allow_claim)
    except (CompositionError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        result = {
            'schema_version': COMPOSER_SCHEMA_VERSION,
            'receipt_kind': COMPOSER_RECEIPT_KIND,
            'status': 'FAIL_CLOSED', 'pass': False, 'claim_eligible': False,
            'errors': [str(exc)],
        }
    encoded = json.dumps(result, indent=2, sort_keys=True) + '\n'
    if args.receipt is not None:
        if _contains_gt(str(args.receipt)):
            print('refusing a GT-named receipt path', file=sys.stderr)
            return 2
        try:
            _write_new(args.receipt, encoded.encode('utf-8'), 'composition receipt')
            args.receipt.chmod(0o444)
        except (CompositionError, OSError) as exc:
            print(f'cannot seal composition receipt: {exc}', file=sys.stderr)
            return 2
    print(encoded, end='')
    return 0 if result.get('pass') is True else 1


if __name__ == '__main__':
    sys.exit(main())
