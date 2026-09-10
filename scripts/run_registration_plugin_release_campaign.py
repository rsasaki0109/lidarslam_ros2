#!/usr/bin/env python3
"""Run the fixed four-leg registration-plugin release campaign.

This host orchestrator owns campaign ordering and cross-leg identity only.  A
leg is still executed by ``run_registration_plugin_release_leg.py`` and a
complete campaign is summarized only by
``summarize_registration_plugin_release_matrix.py``.  The campaign is
non-promoting: it never opens a bag, reads GT, invokes a scorer, saves a map,
or turns functional timing into a performance claim.

``--preflight-only`` performs no Docker operation and may write one fresh
sealed preflight receipt below ``/tmp``.  Runtime execution requires the
profile-bound evidence mount and the read-only image/container probes; tests
inject those probes and the leg executor without starting Docker.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Mapping

# Direct checkout invocation places ``scripts/`` on sys.path.  Resolve the
# canonical package before importing the authoritative leg/summarizer modules.
_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if ((_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file() and
        str(_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SOURCE_ROOT))

from lidarslam_benchmark_tools import (  # noqa: E402
    audit_registration_plugin_matrix as audit,
    run_registration_plugin_release_leg as leg_launcher,
    summarize_registration_plugin_release_matrix as release_summary,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json'
SCHEMA = 'registration-plugin-release-campaign-v1'
CONTRACT = 'registration-plugin-release-campaign-v1'
PARTIAL_RECEIPT_SCHEMA = 'registration-plugin-release-child-partial-v1'
PARTIAL_RECEIPT_NAME = 'registration_plugin_partial.receipt.json'
CAMPAIGN_AGGREGATION_SCHEMA = 'registration-plugin-release-campaign-aggregation-v1'
CAMPAIGN_SCHEMA_PATH = (
    'configs/slam_benchmark_profiles/registration_plugin_release_campaign_v1.schema.json')
PARTIAL_SCHEMA_PATH = (
    'configs/slam_benchmark_profiles/registration_plugin_release_child_partial_v1.schema.json')
EXPECTED_ROWS = (
    ('humble', 'absent'),
    ('humble', 'present'),
    ('jazzy', 'absent'),
    ('jazzy', 'present'),
)
CAMPAIGN_ROOT_RE = re.compile(r'^[a-z0-9][a-z0-9_.-]{7,127}$')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
MAX_JSON_BYTES = 16 * 1024 * 1024
SIDECAR_MAX_BYTES = 256


class CampaignError(RuntimeError):
    """A fail-closed campaign orchestration error."""

    def __init__(self, kind: str, message: str):
        super().__init__(message)
        self.kind = kind


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def _canonical_hash(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical({key: item for key, item in value.items()
                                      if key != 'canonical_sha256'})).hexdigest()


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    return audit.sha256_file(path)


def _campaign_aggregation_contract() -> dict[str, Any]:
    """Return the profile-bound, non-promoting row aggregation contract."""
    return {
        'schema': CAMPAIGN_AGGREGATION_SCHEMA, 'schema_version': 1,
        'tool': {'path': 'scripts/run_registration_plugin_release_campaign.py',
                 'sha256': _sha256_file(ROOT / 'scripts/run_registration_plugin_release_campaign.py')},
        'campaign_schema': {
            'path': CAMPAIGN_SCHEMA_PATH,
            'sha256': _sha256_file(ROOT / CAMPAIGN_SCHEMA_PATH),
        },
        'partial_receipt_schema': {
            'path': PARTIAL_SCHEMA_PATH,
            'sha256': _sha256_file(ROOT / PARTIAL_SCHEMA_PATH),
        },
        'row_state_policy': 'fixed-four-state-vector',
        'partial_rows_policy': 'sealed-started-child-only',
        'normal_rows_exclude_partial': True,
        'status': 'REVIEW_REQUIRED', 'benchmark_eligible': False,
        'promotion': 'FORBIDDEN_UNTIL_SIGNED_RECEIPT',
    }


def _safe_absolute(path_value: Any, label: str) -> Path:
    if not isinstance(path_value, str) or not path_value.startswith('/') or \
            '\x00' in path_value or Path(path_value).as_posix() != path_value or \
            any(part in {'', '.', '..'} for part in path_value.split('/')[1:]):
        raise CampaignError('PATH_INVALID', '{} is not absolute and normalized'.format(label))
    return Path(path_value)


def _parent_without_symlink(path: Path, label: str) -> None:
    current = Path(path.anchor)
    missing_ancestor = False
    for part in path.parent.parts[1:]:
        current /= part
        if missing_ancestor:
            continue
        try:
            info = current.lstat()
        except FileNotFoundError:
            # A fresh campaign/row target may have an absent ancestor.  Keep
            # checking the existing prefix so symlinked parents are rejected;
            # the caller creates the missing ancestor explicitly.
            missing_ancestor = True
            continue
        except OSError as error:
            raise CampaignError(
                'PATH_INVALID', '{} parent is unavailable'.format(label)) from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise CampaignError(
                'PATH_INVALID', '{} parent is not a regular directory'.format(label))


def _fresh_target(path_value: Any, label: str) -> Path:
    path = _safe_absolute(str(path_value), label)
    _parent_without_symlink(path, label)
    if path.exists() or path.is_symlink():
        raise CampaignError('ROOT_NOT_FRESH', '{} already exists'.format(label))
    return path


def _fresh_directory(path: Path, label: str) -> Path:
    _fresh_target(str(path), label)
    try:
        path.mkdir(mode=0o755)
    except OSError as error:
        raise CampaignError('ROOT_RESERVATION_FAILED', '{}: {}'.format(label, error)) from error
    return path


def _validate_preflight_output(path_value: Any, release: Mapping[str, Any]) -> Path:
    """Allow preflight writes only in /tmp or the profile evidence mount."""
    path = _safe_absolute(str(path_value), 'preflight output')
    _parent_without_symlink(path, 'preflight output')
    allowed_roots = [Path('/tmp')]
    storage = release.get('evidence_storage', {})
    mountpoint = storage.get('mountpoint')
    if isinstance(mountpoint, str) and mountpoint.startswith('/'):
        allowed_roots.append(Path(mountpoint))
    if not any(path.parent == root or root in path.parents for root in allowed_roots):
        raise CampaignError(
            'PREFLIGHT_OUTPUT_SCOPE',
            'preflight output must be below /tmp or the profile evidence mount')
    return path


def _regular_immutable(path: Path, label: str, *, max_bytes: int = MAX_JSON_BYTES) -> bytes:
    _parent_without_symlink(path, label)
    try:
        before = path.lstat()
    except OSError as error:
        raise CampaignError('ARTIFACT_INVALID', '{} unavailable'.format(label)) from error
    if (stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1 or stat.S_IMODE(before.st_mode) != 0o444 or
            before.st_size <= 0 or before.st_size > max_bytes):
        raise CampaignError('ARTIFACT_INVALID', '{} is not immutable bounded JSON'.format(label))
    try:
        with path.open('rb') as stream:
            first = os.fstat(stream.fileno())
            data = stream.read(max_bytes + 1)
            after = os.fstat(stream.fileno())
    except OSError as error:
        raise CampaignError('ARTIFACT_INVALID', '{} cannot be read'.format(label)) from error
    if (first.st_ino != before.st_ino or first.st_dev != before.st_dev or
            after.st_ino != before.st_ino or after.st_dev != before.st_dev or
            after.st_nlink != 1 or after.st_size != before.st_size or
            len(data) != before.st_size or len(data) > max_bytes):
        raise CampaignError('ARTIFACT_CHANGED', '{} changed while being read'.format(label))
    return data


def _seal_pair(path_value: Any, value: Mapping[str, Any]) -> dict[str, Any]:
    """Seal a fresh receipt and sidecar, removing only owned partial files."""
    path = _fresh_target(str(path_value), 'receipt')
    sidecar = Path(str(path) + '.sha256')
    _fresh_target(str(sidecar), 'receipt sidecar')
    document = dict(value)
    document['canonical_sha256'] = _canonical_hash(document)
    data = _canonical(document) + b'\n'
    created: list[Path] = []
    try:
        for target, payload in (
                (path, data),
                (sidecar, ('{}  {}\n'.format(_sha256_bytes(data), path.name)).encode('ascii'))):
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
            if hasattr(os, 'O_NOFOLLOW'):
                flags |= os.O_NOFOLLOW
            fd = os.open(str(target), flags, 0o600)
            created.append(target)
            try:
                with os.fdopen(fd, 'wb') as stream:
                    stream.write(payload)
                    stream.flush()
                    os.fsync(stream.fileno())
            except Exception:
                try:
                    os.close(fd)
                except OSError:
                    pass
                raise
            os.chmod(str(target), 0o444)
        return {'path': str(path), 'sidecar': str(sidecar),
                'sha256': _sha256_bytes(data),
                'sidecar_sha256': _sha256_file(sidecar),
                'canonical_sha256': document['canonical_sha256']}
    except Exception:
        for target in reversed(created):
            try:
                if target.is_file() and not target.is_symlink():
                    target.unlink()
            except OSError:
                pass
        raise


def _safe_relative(path_value: Any, label: str) -> str:
    """Return one normalized root-relative POSIX path.

    Partial-row bindings are deliberately relative to the campaign root.  A
    path which is merely parseable by ``Path`` is not enough: absolute paths,
    empty components, dot traversal, and platform separators would make the
    receipt ambiguous during a later reopen.
    """
    if (not isinstance(path_value, str) or not path_value or
            path_value.startswith('/') or '\\' in path_value or
            Path(path_value).as_posix() != path_value or
            any(part in {'', '.', '..'} for part in path_value.split('/'))):
        raise CampaignError('PARTIAL_ROW_PATH_INVALID', '{} is not normalized'.format(label))
    return path_value


def _partial_receipt_value(
        row: Mapping[str, Any], campaign: Mapping[str, Any],
        snapshot: Mapping[str, Any], failure: Mapping[str, Any],
        started_at: float, ended_at: float
) -> dict[str, Any]:
    """Build the sealed receipt for a child which did not complete its row.

    This is a distinct artifact from the normal host/inner PASS receipts.  It
    carries enough campaign and row identity to make the failure useful in an
    offline audit while making no completion or promotion claim.
    """
    row_identity = {
        'index': row['index'], 'distro': row['distro'],
        'dependency_leg': row['dependency_leg'],
        'child_root': row['child_root'],
        'container_name': row['container_name'],
    }
    return {
        'schema': PARTIAL_RECEIPT_SCHEMA, 'schema_version': 1,
        'status': 'FAIL_CLOSED', 'benchmark_eligible': False,
        'execution_started': True, 'campaign_id': campaign['campaign_id'],
        'campaign_set': campaign, 'row_count': len(EXPECTED_ROWS),
        'row_order': [list(item) for item in EXPECTED_ROWS],
        'row': row_identity, 'profile': snapshot['profile'],
        'source_manifest_sha256': snapshot['source']['manifest_sha256'],
        'git_revision': snapshot['git']['revision'], 'failure': dict(failure),
        'started_at_unix': started_at, 'ended_at_unix': ended_at,
    }


def _record_partial_row(
        campaign_root: Path, row: Mapping[str, Any], campaign: Mapping[str, Any],
        snapshot: Mapping[str, Any], failure: Mapping[str, Any],
        started_at: float, ended_at: float
) -> dict[str, Any]:
    """Seal and describe one started-but-incomplete child.

    The child root is normally reserved by the leg launcher.  If a launcher
    fails before making it, the campaign owner creates the still-fresh child
    directory solely to preserve the failure receipt.  No existing child or
    partial receipt is overwritten.
    """
    root = Path(campaign_root)
    child = _safe_absolute(row['child_root'], 'partial child root')
    if child.parent != root or child.name != (
            'registration-plugin-release-{}-{}'.format(
                row['distro'], row['dependency_leg'])):
        raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', str(child))
    if child.is_symlink() or (child.exists() and not child.is_dir()):
        raise CampaignError('PARTIAL_ROW_CHILD_INVALID', str(child))
    if not child.exists():
        _fresh_directory(child, 'partial child root')
    partial_path = child / PARTIAL_RECEIPT_NAME
    partial_sidecar = Path(str(partial_path) + '.sha256')
    if partial_path.exists() or partial_path.is_symlink() or \
            partial_sidecar.exists() or partial_sidecar.is_symlink():
        raise CampaignError('PARTIAL_ROW_COLLISION', str(partial_path))
    value = _partial_receipt_value(
        row, campaign, snapshot, failure, started_at, ended_at)
    seal = _seal_pair(partial_path, value)
    partial_value, descriptor = _read_sealed(
        partial_path, PARTIAL_RECEIPT_SCHEMA, 'partial child receipt')
    if partial_value != {**value, 'canonical_sha256': seal['canonical_sha256']}:
        raise CampaignError('PARTIAL_ROW_BINDING_INVALID', str(partial_path))
    relative = _safe_relative(
        partial_path.relative_to(root).as_posix(), 'partial child receipt path')
    sidecar_relative = _safe_relative(relative + '.sha256', 'partial child sidecar path')
    return {
        'index': row['index'], 'distro': row['distro'],
        'dependency_leg': row['dependency_leg'], 'child_root': str(child),
        'container_name': row['container_name'], 'status': 'FAIL_CLOSED',
        'path_relative': relative, 'file_bytes': descriptor['file_bytes'],
        'file_sha256': descriptor['file_sha256'],
        'canonical_sha256': partial_value['canonical_sha256'],
        'sidecar_path_relative': sidecar_relative,
        'sidecar_sha256': descriptor['sidecar_sha256'],
        'failure': dict(failure),
    }


def _read_sealed(
        path_value: Any, expected_schema: str, label: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = _safe_absolute(str(path_value), label)
    data = _regular_immutable(path, label)
    sidecar = Path(str(path) + '.sha256')
    side_data = _regular_immutable(sidecar, label + ' sidecar', max_bytes=SIDECAR_MAX_BYTES)
    try:
        value = json.loads(data.decode('utf-8'))
        tokens = side_data.decode('ascii').strip().split()
    except (UnicodeError, ValueError) as error:
        raise CampaignError('ARTIFACT_INVALID', '{} JSON/sidecar invalid'.format(label)) from error
    if (not isinstance(value, dict) or value.get('schema') != expected_schema or
            value.get('schema_version') != 1 or
            tokens != [_sha256_bytes(data), path.name] or
            value.get('canonical_sha256') != _canonical_hash(value)):
        raise CampaignError('ARTIFACT_BINDING_INVALID', '{} identity drift'.format(label))
    return value, {'path': str(path), 'file_bytes': len(data),
                   'file_sha256': _sha256_bytes(data),
                   'sidecar_sha256': _sha256_file(sidecar),
                   'sidecar_path': str(sidecar)}


def _read_existing_receipt(path_value: Any, contract: str,
                           label: str) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reopen an authoritative leg receipt sealed by the existing contracts."""
    path = _safe_absolute(str(path_value), label)
    data = _regular_immutable(path, label)
    sidecar = Path(str(path) + '.sha256')
    side_data = _regular_immutable(sidecar, label + ' sidecar', max_bytes=SIDECAR_MAX_BYTES)
    try:
        value = json.loads(data.decode('utf-8'))
        tokens = side_data.decode('ascii').strip().split()
    except (UnicodeError, ValueError) as error:
        raise CampaignError('ARTIFACT_INVALID', '{} JSON/sidecar invalid'.format(label)) from error
    if (not isinstance(value, dict) or value.get('schema_version') != 1 or
            value.get('schema') != contract and value.get('contract_version') != contract or
            tokens != [_sha256_bytes(data), path.name]):
        raise CampaignError('ARTIFACT_BINDING_INVALID', '{} identity drift'.format(label))
    return value, {'path': str(path), 'file_bytes': len(data),
                   'file_sha256': _sha256_bytes(data),
                   'sidecar_sha256': _sha256_file(sidecar),
                   'sidecar_path': str(sidecar)}


def _load_profile(profile_path: Path) -> tuple[dict[str, Any], dict[str, Any], Path]:
    profile, canonical = audit._load_profile(profile_path)
    release = audit._validate_release_matrix_profile(profile)
    campaign = release.get('campaign_set')
    if campaign.get('rows') != [
            {'distro': distro, 'dependency_leg': leg} for distro, leg in EXPECTED_ROWS]:
        raise CampaignError(
            'CAMPAIGN_ROWS_INVALID',
            'profile campaign rows are not the exact four-row order')
    if release.get('campaign_aggregation') != _campaign_aggregation_contract():
        raise CampaignError(
            'CAMPAIGN_AGGREGATION_INVALID',
            'profile campaign aggregation contract is stale or incomplete')
    return profile, release, canonical


def _source_snapshot(
        repo_root: Path, profile: Mapping[str, Any], profile_path: Path
) -> dict[str, Any]:
    source = audit._source_manifest(repo_root, profile)
    if source.get('status') != 'PASS':
        raise CampaignError('CURRENT_SOURCE_HASH_MISMATCH', json.dumps(source, sort_keys=True))
    git = audit._git_snapshot(repo_root)
    return {'profile': {'path': str(profile_path), 'sha256': _sha256_file(profile_path)},
            'source': source, 'git': git}


def _snapshot_equal(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    return (left.get('profile') == right.get('profile') and
            left.get('source') == right.get('source') and
            left.get('git') == right.get('git'))


def _campaign_root_identity(campaign_root: Path, campaign: Mapping[str, Any],
                            snapshot: Mapping[str, Any]) -> dict[str, Any]:
    payload = {
        'schema': 'registration-plugin-campaign-root-v1', 'schema_version': 1,
        'logical_name': campaign_root.name, 'campaign_set': dict(campaign),
        'profile_sha256': snapshot['profile']['sha256'],
        'source_manifest_sha256': snapshot['source']['manifest_sha256'],
    }
    payload['identity_sha256'] = _sha256_bytes(_canonical(payload))
    return payload


def _image_probe_default(reference: str) -> Mapping[str, Any]:
    try:
        result = subprocess.run(
            ['docker', 'image', 'inspect', reference], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=False, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise CampaignError('IMAGE_PROBE_FAILED', str(error)) from error
    if result.returncode != 0:
        raise CampaignError('IMAGE_UNAVAILABLE', result.stderr.decode('utf-8', 'replace')[-512:])
    try:
        value = json.loads(result.stdout.decode('utf-8'))
        if not isinstance(value, list) or len(value) != 1:
            raise ValueError('expected one image object')
        return value[0]
    except (UnicodeError, ValueError) as error:
        raise CampaignError('IMAGE_PROBE_INVALID', str(error)) from error


def _container_probe_default(name: str) -> Mapping[str, Any] | None:
    try:
        result = subprocess.run(
            ['docker', 'inspect', name], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=False, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise CampaignError('CONTAINER_PROBE_FAILED', str(error)) from error
    if result.returncode == 1:
        stderr = result.stderr.decode('utf-8', 'replace')
        absence_lines = (
            'Error: No such object: {}'.format(name),
            'Error: No such container: {}'.format(name),
            'error: no such object: {}'.format(name),
            'error: no such container: {}'.format(name),
        )
        absence_messages = tuple(
            line + suffix for line in absence_lines for suffix in ('', '\n')
        )
        if result.stdout in (b'', b'[]\n') and stderr in absence_messages:
            return None
    if result.returncode != 0:
        raise CampaignError(
            'CONTAINER_PROBE_FAILED',
            result.stderr.decode('utf-8', 'replace')[-512:])
    try:
        value = json.loads(result.stdout.decode('utf-8'))
        if not isinstance(value, list) or len(value) != 1:
            raise ValueError('expected one container object')
        return value[0]
    except (UnicodeError, ValueError) as error:
        raise CampaignError('CONTAINER_PROBE_INVALID', str(error)) from error


def _runtime_probes(rows: list[dict[str, Any]], *, image_probe: Callable | None,
                    container_probe: Callable | None, required: bool) -> dict[str, Any]:
    if image_probe is None or container_probe is None:
        if required:
            raise CampaignError('RUNTIME_PROBE_REQUIRED',
                                'execution requires image and container collision probes')
        return {'status': 'NOT_RUNTIME_VALIDATED', 'images': [], 'containers': []}
    images: dict[str, Any] = {}
    containers: list[dict[str, Any]] = []
    for row in rows:
        image = row['image']
        reference = image['reference']
        if reference not in images:
            observed = image_probe(reference)
            images[reference] = leg_launcher._validate_image_identity(
                observed, image, 'campaign image')
        observed_container = container_probe(row['container_name'])
        if observed_container is not None:
            raise CampaignError('CONTAINER_NAME_ALREADY_EXISTS',
                                '{} already exists'.format(row['container_name']))
        containers.append({'name': row['container_name'], 'status': 'ABSENT'})
    return {'status': 'PASS', 'images': list(images.values()), 'containers': containers}


def build_preflight(repo_root: Path, profile_path: Path, campaign_root: Path, *,
                    image_probe: Callable | None = None,
                    container_probe: Callable | None = None,
                    require_storage: bool = False,
                    runtime_probes_required: bool = False) -> dict[str, Any]:
    """Build a pure preflight report; no campaign directory is created."""
    repo_root = Path(repo_root).absolute()
    profile_path = Path(profile_path).absolute()
    if not repo_root.is_dir() or repo_root.is_symlink():
        raise CampaignError('REPOSITORY_INVALID', str(repo_root))
    profile, release, profile_path = _load_profile(profile_path)
    root = _fresh_target(campaign_root, 'campaign root')
    snapshot = _source_snapshot(repo_root, profile, profile_path)
    storage = {'status': 'NOT_APPLICABLE'}
    if require_storage:
        try:
            storage = leg_launcher._validate_evidence_storage(
                root, release['evidence_storage'])
            storage = dict(storage)
            storage['status'] = 'PASS'
        except Exception as error:
            raise CampaignError('EVIDENCE_STORAGE_INVALID', str(error)) from error
    rows: list[dict[str, Any]] = []
    for index, (distro, dependency_leg) in enumerate(EXPECTED_ROWS):
        distro_row = next(item for item in release['distros'] if item['name'] == distro)
        child = root / 'registration-plugin-release-{}-{}'.format(distro, dependency_leg)
        _fresh_target(child, 'row evidence root')
        rows.append({
            'index': index, 'distro': distro, 'dependency_leg': dependency_leg,
            'child_root': str(child),
            'container_name': leg_launcher._container_name(child, distro, dependency_leg),
            'image': dict(distro_row['image']), 'status': 'NOT_STARTED',
            'phase_state': {'docker': 'NOT_STARTED', 'build_test': 'NOT_STARTED',
                            'evidence': None},
        })
    probes = _runtime_probes(rows, image_probe=image_probe,
                             container_probe=container_probe,
                             required=runtime_probes_required)
    storage_ready = not require_storage or storage['status'] != 'NOT_APPLICABLE'
    status = 'PREFLIGHT_PASS' if probes['status'] == 'PASS' and storage_ready \
        else 'PREFLIGHT_NOT_RUNTIME_VALIDATED'
    observed_at = int(time.time())
    return {
        'schema': SCHEMA, 'schema_version': 1, 'status': status,
        'benchmark_eligible': False, 'execution_started': False,
        'campaign_id': release['campaign_set']['campaign_id'],
        'campaign_set': release['campaign_set'], 'profile': snapshot['profile'],
        'source_manifest': snapshot['source'], 'git': snapshot['git'],
        'campaign_root': {'path': str(root), 'fresh': True,
                          'identity': _campaign_root_identity(
                              root, release['campaign_set'], snapshot)},
        'evidence_storage': storage, 'runtime_probes': probes, 'rows': rows,
        'partial_rows': [], 'partial_row_count': 0,
        'summary': {'status': 'NOT_RUN', 'reason': 'preflight_only'},
        'safety': {'docker': {'state': 'NOT_STARTED', 'evidence': []},
                   'build_test': {'state': 'NOT_STARTED', 'evidence': []},
                   'bag_opened': False, 'ground_truth_content_opened': False,
                   'scorer_invoked': False, 'map_saved': False,
                   'formal_replay_started': False},
        'started_at_unix': observed_at, 'ended_at_unix': observed_at,
        'observed_at_unix': observed_at,
    }


PHASE_STATES = ('NOT_STARTED', 'ATTEMPTED', 'CONFIRMED')


def _phase_state_for_leg(host_path: Path, *, invoked: bool) -> dict[str, Any]:
    """Derive phase state from the authoritative host receipt when present."""
    attempted = 'ATTEMPTED' if invoked else 'NOT_STARTED'
    state = {'docker': attempted, 'build_test': attempted, 'evidence': None}
    if not host_path.exists() or host_path.is_symlink():
        return state
    try:
        value, descriptor = _read_existing_receipt(
            host_path, 'registration-plugin-host-release-leg-v1',
            'phase host receipt')
    except CampaignError:
        return state
    container = value.get('container', {})
    if container.get('start_count') == 1:
        state['docker'] = 'CONFIRMED'
    if value.get('status') == 'PASS' and value.get(
            'runner_receipt_binding', {}).get('status') == 'PASS':
        state['build_test'] = 'CONFIRMED'
    state['evidence'] = descriptor
    return state


def _aggregate_phase_state(rows: list[Mapping[str, Any]], key: str) -> dict[str, Any]:
    states = [row.get('phase_state', {}).get(key, 'NOT_STARTED') for row in rows]
    for expected in ('CONFIRMED', 'ATTEMPTED', 'NOT_STARTED'):
        if expected in states:
            return {'state': expected,
                    'evidence': [row['phase_state']['evidence'] for row in rows
                                 if row.get('phase_state', {}).get(key) == expected and
                                 row.get('phase_state', {}).get('evidence') is not None]}
    raise CampaignError('PHASE_STATE_INVALID', key)


def _reject_extra(value: Mapping[str, Any], allowed: set[str], label: str) -> None:
    extra = sorted(set(value) - allowed)
    if extra:
        raise CampaignError(
            'CAMPAIGN_SHAPE_INVALID', '{} has extra fields {}'.format(label, extra))


def _validate_phase_object(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', '{} is not an object'.format(label))
    _reject_extra(value, {'state', 'evidence'}, label)
    if value.get('state') not in PHASE_STATES or not isinstance(value.get('evidence'), list):
        raise CampaignError('CAMPAIGN_PHASE_INVALID', label)
    for index, descriptor in enumerate(value['evidence']):
        _validate_descriptor(descriptor, '{}.evidence[{}]'.format(label, index))


def _validate_descriptor(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', '{} is not an object'.format(label))
    _reject_extra(value, {
        'path', 'file_bytes', 'file_sha256', 'sidecar_sha256', 'sidecar_path'}, label)
    if (not isinstance(value.get('path'), str) or not value['path'] or
            not isinstance(value.get('sidecar_path'), str) or
            not value['sidecar_path'] or not isinstance(value.get('file_bytes'), int) or
            isinstance(value.get('file_bytes'), bool) or value['file_bytes'] <= 0 or
            not SHA256_RE.fullmatch(str(value.get('file_sha256', ''))) or
            not SHA256_RE.fullmatch(str(value.get('sidecar_sha256', '')))):
        raise CampaignError(
            'CAMPAIGN_ARTIFACT_INVALID',
            '{} is not a sealed descriptor'.format(label))


def _validate_partial_receipt_value(
        value: Mapping[str, Any], row: Mapping[str, Any],
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        source: Mapping[str, Any], git: Mapping[str, Any], label: str
) -> None:
    """Validate the identity carried by a sealed child partial receipt."""
    if not isinstance(value, Mapping):
        raise CampaignError('PARTIAL_ROW_BINDING_INVALID', '{} is not an object'.format(label))
    _reject_extra(value, {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'execution_started', 'campaign_id', 'campaign_set', 'row_count',
        'row_order', 'row', 'profile', 'source_manifest_sha256',
        'git_revision', 'failure', 'started_at_unix', 'ended_at_unix',
        'canonical_sha256'}, label)
    if (value.get('schema') != PARTIAL_RECEIPT_SCHEMA or
            value.get('schema_version') != 1 or
            value.get('status') != 'FAIL_CLOSED' or
            value.get('benchmark_eligible') is not False or
            value.get('execution_started') is not True or
            value.get('campaign_id') != campaign.get('campaign_id') or
            value.get('campaign_set') != campaign or
            value.get('row_count') != len(EXPECTED_ROWS) or
            value.get('row_order') != [list(item) for item in EXPECTED_ROWS] or
            value.get('profile') != profile or
            value.get('source_manifest_sha256') != source.get('manifest_sha256') or
            value.get('git_revision') != git.get('revision')):
        raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', '{} identity drift'.format(label))
    row_identity = value.get('row')
    expected_identity = {
        'index': row.get('index'), 'distro': row.get('distro'),
        'dependency_leg': row.get('dependency_leg'),
        'child_root': row.get('child_root'),
        'container_name': row.get('container_name'),
    }
    if row_identity != expected_identity:
        raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', '{} row drift'.format(label))
    failure = value.get('failure')
    if (not isinstance(failure, Mapping) or
            set(failure) != {'kind', 'phase', 'row', 'message'} or
            failure.get('phase') != 'leg' or
            failure.get('row') != row.get('index') or
            not isinstance(failure.get('kind'), str) or
            not isinstance(failure.get('message'), str)):
        raise CampaignError('PARTIAL_ROW_FAILURE_INVALID', '{} failure drift'.format(label))
    for key in ('started_at_unix', 'ended_at_unix'):
        timestamp = value.get(key)
        if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool) or
                timestamp < 0):
            raise CampaignError('PARTIAL_ROW_TIME_INVALID', '{} {}'.format(label, key))
    if value['ended_at_unix'] < value['started_at_unix']:
        raise CampaignError('PARTIAL_ROW_TIME_INVALID', '{} timestamps reversed'.format(label))
    if not SHA256_RE.fullmatch(str(value.get('source_manifest_sha256', ''))) or \
            not re.fullmatch(r'[0-9a-f]{40}', str(value.get('git_revision', ''))):
        raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', '{} source/git invalid'.format(label))
    if not SHA256_RE.fullmatch(str(value.get('canonical_sha256', ''))):
        raise CampaignError('PARTIAL_ROW_BINDING_INVALID', '{} canonical hash missing'.format(label))


def _validate_partial_rows(
        value: Mapping[str, Any], rows: list[Mapping[str, Any]],
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        source: Mapping[str, Any], git: Mapping[str, Any]
) -> None:
    """Reopen every started-but-incomplete child receipt exactly once.

    ``rows`` remains the fixed campaign state machine.  A partial receipt is
    never a completed row and is represented only by this separate, ordered
    projection.  The filesystem reopen here prevents a plausible-looking
    path/hash projection from becoming an unreferenced child artifact.
    """
    partial_rows = value.get('partial_rows')
    partial_count = value.get('partial_row_count')
    if (not isinstance(partial_rows, list) or
            type(partial_count) is not int or partial_count != len(partial_rows) or
            partial_count < 0 or partial_count > len(EXPECTED_ROWS)):
        raise CampaignError('PARTIAL_ROWS_CARDINALITY_INVALID',
                            'partial row count/list mismatch')
    expected_indexes = {
        int(row['index']) for row in rows
        if row.get('status') == 'FAIL_CLOSED' and row.get('command') is not None
    }
    observed_indexes: list[int] = []
    root = _safe_absolute(value['campaign_root']['path'], 'campaign root')
    for offset, partial in enumerate(partial_rows):
        label = 'partial_rows[{}]'.format(offset)
        if not isinstance(partial, Mapping):
            raise CampaignError('PARTIAL_ROWS_SHAPE_INVALID', '{} is not an object'.format(label))
        _reject_extra(partial, {
            'index', 'distro', 'dependency_leg', 'child_root', 'container_name',
            'status', 'path_relative', 'file_bytes', 'file_sha256',
            'canonical_sha256', 'sidecar_path_relative', 'sidecar_sha256',
            'failure'}, label)
        index = partial.get('index')
        if (type(index) is not int or index < 0 or index >= len(EXPECTED_ROWS) or
                (observed_indexes and index <= observed_indexes[-1])):
            raise CampaignError('PARTIAL_ROWS_ORDER_INVALID', '{} index/order'.format(label))
        row = rows[index]
        expected_distro, expected_leg = EXPECTED_ROWS[index]
        if (partial.get('distro') != expected_distro or
                partial.get('dependency_leg') != expected_leg or
                partial.get('child_root') != row.get('child_root') or
                partial.get('container_name') != row.get('container_name') or
                partial.get('status') != 'FAIL_CLOSED' or
                row.get('status') != 'FAIL_CLOSED' or
                row.get('command') is None or
                partial.get('failure') != row.get('failure')):
            raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', '{} row projection'.format(label))
        child = _safe_absolute(row.get('child_root'), '{} child root'.format(label))
        if child.parent != root or child.name != (
                'registration-plugin-release-{}-{}'.format(expected_distro, expected_leg)):
            raise CampaignError('PARTIAL_ROW_IDENTITY_INVALID', '{} child path'.format(label))
        expected_relative = child.relative_to(root).as_posix() + '/' + PARTIAL_RECEIPT_NAME
        path_relative = _safe_relative(partial.get('path_relative'), label + ' path')
        sidecar_relative = _safe_relative(
            partial.get('sidecar_path_relative'), label + ' sidecar path')
        if (path_relative != expected_relative or
                sidecar_relative != expected_relative + '.sha256' or
                type(partial.get('file_bytes')) is not int or
                partial['file_bytes'] <= 0 or
                not SHA256_RE.fullmatch(str(partial.get('file_sha256', ''))) or
                not SHA256_RE.fullmatch(str(partial.get('canonical_sha256', ''))) or
                not SHA256_RE.fullmatch(str(partial.get('sidecar_sha256', '')))):
            raise CampaignError('PARTIAL_ROW_BINDING_INVALID', '{} descriptor'.format(label))
        partial_path = root / path_relative
        partial_value, descriptor = _read_sealed(
            partial_path, PARTIAL_RECEIPT_SCHEMA, label + ' receipt')
        _validate_partial_receipt_value(
            partial_value, row, campaign, profile, source, git,
            label + ' receipt')
        if (descriptor['file_bytes'] != partial['file_bytes'] or
                descriptor['file_sha256'] != partial['file_sha256'] or
                partial_value['canonical_sha256'] != partial['canonical_sha256'] or
                descriptor['sidecar_sha256'] != partial['sidecar_sha256']):
            raise CampaignError('PARTIAL_ROW_BINDING_INVALID', '{} hash drift'.format(label))
        observed_indexes.append(index)
    if set(observed_indexes) != expected_indexes:
        raise CampaignError('PARTIAL_ROWS_CARDINALITY_INVALID',
                            'started child is missing a partial receipt')
    # An independently-created partial receipt must not hide outside the
    # projection, even when a forged report omits a started row.  Inspect the
    # authoritative child names under the root as well as the row projection;
    # this keeps ``rows=[]``-style partial input fail-closed before it can look
    # like an empty campaign.
    expected_children = {
        'registration-plugin-release-{}-{}'.format(distro, dependency_leg): index
        for index, (distro, dependency_leg) in enumerate(EXPECTED_ROWS)
    }
    if root.is_dir() and not root.is_symlink():
        try:
            children = list(root.iterdir())
        except OSError as error:
            raise CampaignError('PARTIAL_ROWS_ROOT_INVALID', str(root)) from error
        for child in children:
            if not child.name.startswith('registration-plugin-release-'):
                continue
            candidate = child / PARTIAL_RECEIPT_NAME
            if not (candidate.exists() or candidate.is_symlink()):
                continue
            index = expected_children.get(child.name)
            if index is None or index not in observed_indexes:
                raise CampaignError(
                    'PARTIAL_ROWS_UNREFERENCED',
                    'unreferenced partial receipt: {}'.format(candidate))


def _validate_image_contract(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_ROWS_INVALID', '{} is not an object'.format(label))
    _reject_extra(value, {'tag', 'digest', 'reference', 'manifest'}, label)
    for key in ('tag', 'digest', 'reference'):
        if not isinstance(value.get(key), str) or not value[key]:
            raise CampaignError('CAMPAIGN_ROWS_INVALID', '{}.{} missing'.format(label, key))
    if 'manifest' in value:
        manifest = value['manifest']
        if not isinstance(manifest, Mapping):
            raise CampaignError(
                'CAMPAIGN_ROWS_INVALID',
                '{}.manifest is not an object'.format(label))
        _reject_extra(manifest, {
            'index_digest', 'index_media_type', 'linux_amd64_digest',
            'linux_amd64_media_type', 'linux_amd64_os', 'linux_amd64_architecture'},
                      '{}.manifest'.format(label))


def _validate_source_manifest(value: Any) -> None:
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'source manifest missing')
    _reject_extra(value, {'status', 'hash_kind', 'files', 'manifest_sha256', 'mismatches',
                          'v2_snapshot'},
                  'source_manifest')
    if (value.get('status') != 'PASS' or
            value.get('hash_kind') != 'relative_path_content_sha256_v1' or
            not SHA256_RE.fullmatch(str(value.get('manifest_sha256', ''))) or
            not isinstance(value.get('files'), list) or not value['files'] or
            not isinstance(value.get('mismatches'), list) or value['mismatches']):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'source manifest invalid')
    paths = set()
    for index, item in enumerate(value['files']):
        label = 'source_manifest.files[{}]'.format(index)
        if not isinstance(item, Mapping):
            raise CampaignError('CAMPAIGN_SHAPE_INVALID', '{} is not an object'.format(label))
        _reject_extra(item, {'path', 'expected_sha256', 'status', 'actual_sha256',
                             'size_bytes', 'error'}, label)
        if (not isinstance(item.get('path'), str) or not item['path'] or
                item['path'] in paths or
                not SHA256_RE.fullmatch(str(item.get('expected_sha256', ''))) or
                item.get('status') != 'PASS' or
                not SHA256_RE.fullmatch(str(item.get('actual_sha256', ''))) or
                not isinstance(item.get('size_bytes'), int) or
                isinstance(item.get('size_bytes'), bool) or item['size_bytes'] < 0 or
                'error' in item):
            raise CampaignError('CAMPAIGN_IDENTITY_INVALID', '{} invalid'.format(label))
        paths.add(item['path'])
    snapshot = value.get('v2_snapshot')
    if snapshot is not None:
        if not isinstance(snapshot, Mapping):
            raise CampaignError(
                'CAMPAIGN_IDENTITY_INVALID', 'source_manifest.v2_snapshot invalid')
        base = {'status', 'checked', 'promotion'}
        status_value = snapshot.get('status')
        if status_value == 'NOT_CONFIGURED':
            _reject_extra(snapshot, base, 'source_manifest.v2_snapshot')
            if (snapshot.get('checked') is not False or
                    snapshot.get('promotion') != 'NOT_AUTHORIZED'):
                raise CampaignError(
                    'CAMPAIGN_IDENTITY_INVALID', 'source_manifest.v2_snapshot invalid')
        elif status_value == 'PASS_REOPENED_REVIEW_REQUIRED':
            _reject_extra(snapshot, base | {'root', 'manifest', 'manifest_sha256'},
                          'source_manifest.v2_snapshot')
            root = snapshot.get('root')
            manifest = snapshot.get('manifest')
            if (snapshot.get('checked') is not True or
                    snapshot.get('promotion') != 'NOT_AUTHORIZED' or
                    not isinstance(root, str) or not os.path.isabs(root) or
                    Path(root).as_posix() != root or
                    not isinstance(manifest, str) or not manifest or
                    os.path.isabs(manifest) or '\\' in manifest or
                    any(part in ('', '.', '..') for part in manifest.split('/')) or
                    Path(manifest).as_posix() != manifest or
                    not SHA256_RE.fullmatch(str(snapshot.get('manifest_sha256', '')))):
                raise CampaignError(
                    'CAMPAIGN_IDENTITY_INVALID', 'source_manifest.v2_snapshot invalid')
        else:
            raise CampaignError(
                'CAMPAIGN_IDENTITY_INVALID', 'source_manifest.v2_snapshot invalid')


def _validate_git_snapshot(value: Any) -> None:
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'git snapshot missing')
    _reject_extra(value, {'revision', 'head_tree', 'dirty', 'dirty_tree_sha256',
                          'dirty_components'}, 'git')
    if (not re.fullmatch(r'[0-9a-f]{40}', str(value.get('revision', ''))) or
            not re.fullmatch(r'[0-9a-f]{40}', str(value.get('head_tree', ''))) or
            not isinstance(value.get('dirty'), bool) or
            not SHA256_RE.fullmatch(str(value.get('dirty_tree_sha256', '')))):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'git snapshot invalid')
    components = value.get('dirty_components')
    if not isinstance(components, Mapping):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'git dirty components missing')
    _reject_extra(components, {'status', 'diff_sha256', 'untracked'}, 'git.dirty_components')
    if (not isinstance(components.get('status'), str) or
            not SHA256_RE.fullmatch(str(components.get('diff_sha256', ''))) or
            not isinstance(components.get('untracked'), list)):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'git dirty components invalid')
    for index, item in enumerate(components['untracked']):
        label = 'git.dirty_components.untracked[{}]'.format(index)
        if not isinstance(item, Mapping):
            raise CampaignError('CAMPAIGN_SHAPE_INVALID', '{} is not an object'.format(label))
        _reject_extra(item, {'path', 'kind', 'sha256'}, label)
        if (not isinstance(item.get('path'), str) or not item['path'] or
                item.get('kind') not in {'file', 'directory', 'symlink'}):
            raise CampaignError('CAMPAIGN_IDENTITY_INVALID', '{} invalid'.format(label))
        if item['kind'] == 'file' and not SHA256_RE.fullmatch(str(item.get('sha256', ''))):
            raise CampaignError('CAMPAIGN_IDENTITY_INVALID', '{} file hash missing'.format(label))


def _validate_campaign_report(value: Mapping[str, Any]) -> None:
    """Independent fail-closed validation before any campaign receipt seal."""
    if not isinstance(value, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', 'campaign report is not an object')
    allowed = {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'execution_started', 'campaign_id', 'campaign_set', 'profile',
        'source_manifest', 'git', 'campaign_root', 'evidence_storage',
        'runtime_probes', 'rows', 'partial_rows', 'partial_row_count', 'summary',
        'failure', 'execution',
        'safety', 'started_at_unix', 'ended_at_unix', 'observed_at_unix',
        'canonical_sha256',
    }
    _reject_extra(value, allowed, 'campaign report')
    if value.get('schema') != SCHEMA or value.get('schema_version') != 1:
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', 'campaign schema/version mismatch')
    status = value.get('status')
    if status not in {'PREFLIGHT_PASS', 'PREFLIGHT_NOT_RUNTIME_VALIDATED',
                      'PASS', 'PARTIAL_FAILURE', 'FAIL_CLOSED'}:
        raise CampaignError('CAMPAIGN_STATUS_INVALID', str(status))
    if value.get('benchmark_eligible') is not False:
        raise CampaignError('CAMPAIGN_PROMOTION_INVALID', 'benchmark eligibility must be false')
    profile_candidate = value.get('profile')
    if (not isinstance(value.get('campaign_id'), str) or
            not isinstance(profile_candidate, Mapping) or
            not SHA256_RE.fullmatch(str(profile_candidate.get('sha256', '')))):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'campaign/profile identity missing')
    campaign = value.get('campaign_set')
    if not isinstance(campaign, Mapping):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'campaign_set missing')
    _reject_extra(campaign, {
        'schema', 'schema_version', 'campaign_id', 'rows', 'row_set_sha256',
        'identity_sha256'}, 'campaign_set')
    if campaign.get('schema') != 'registration-plugin-campaign-set-v1' or \
            campaign.get('schema_version') != 1:
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'campaign_set schema mismatch')
    if (value.get('campaign_id') != campaign.get('campaign_id') or
            not CAMPAIGN_ROOT_RE.fullmatch(str(value.get('campaign_id', '')))):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'campaign_id projection changed')
    expected_rows = [{'distro': distro, 'dependency_leg': leg}
                     for distro, leg in EXPECTED_ROWS]
    if campaign.get('rows') != expected_rows:
        raise CampaignError('CAMPAIGN_ROWS_INVALID', 'campaign_set rows changed')
    for key in ('row_set_sha256', 'identity_sha256'):
        if not SHA256_RE.fullmatch(str(campaign.get(key, ''))):
            raise CampaignError('CAMPAIGN_IDENTITY_INVALID', '{} missing'.format(key))
    profile = value.get('profile')
    if not isinstance(profile, Mapping):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'profile missing')
    _reject_extra(profile, {'path', 'sha256'}, 'profile')
    if (not isinstance(profile.get('path'), str) or
            not profile['path'].startswith('/') or
            not SHA256_RE.fullmatch(profile.get('sha256', ''))):
        raise CampaignError('CAMPAIGN_IDENTITY_INVALID', 'profile identity invalid')
    source = value.get('source_manifest')
    _validate_source_manifest(source)
    git = value.get('git')
    _validate_git_snapshot(git)
    root = value.get('campaign_root')
    if not isinstance(root, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', 'campaign_root missing')
    _reject_extra(root, {'path', 'fresh', 'identity'}, 'campaign_root')
    if not isinstance(root.get('path'), str) or root.get('fresh') is not True:
        raise CampaignError('CAMPAIGN_ROOT_INVALID', 'campaign root is not fresh')
    _safe_absolute(root['path'], 'campaign root')
    root_identity = root.get('identity')
    if not isinstance(root_identity, Mapping):
        raise CampaignError('CAMPAIGN_ROOT_INVALID', 'campaign root identity missing')
    _reject_extra(root_identity, {
        'schema', 'schema_version', 'logical_name', 'campaign_set',
        'profile_sha256', 'source_manifest_sha256', 'identity_sha256'},
                  'campaign_root.identity')
    if (root_identity.get('schema') != 'registration-plugin-campaign-root-v1' or
            root_identity.get('schema_version') != 1 or
            not isinstance(root_identity.get('logical_name'), str) or
            not root_identity['logical_name']):
        raise CampaignError('CAMPAIGN_ROOT_INVALID', 'root identity schema invalid')
    for key in ('profile_sha256', 'source_manifest_sha256', 'identity_sha256'):
        if not SHA256_RE.fullmatch(str(root_identity.get(key, ''))):
            raise CampaignError('CAMPAIGN_ROOT_INVALID', '{} missing'.format(key))
    if (root_identity.get('campaign_set') != campaign or
            root_identity.get('profile_sha256') != profile['sha256'] or
            root_identity.get('source_manifest_sha256') != source['manifest_sha256']):
        raise CampaignError('CAMPAIGN_ROOT_INVALID', 'root identity projection changed')
    storage = value.get('evidence_storage')
    if not isinstance(storage, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', 'evidence_storage missing')
    _reject_extra(storage, {'status', 'contract', 'observed'}, 'evidence_storage')
    probes = value.get('runtime_probes')
    if not isinstance(probes, Mapping):
        raise CampaignError('CAMPAIGN_SHAPE_INVALID', 'runtime_probes is not an object')
    _reject_extra(probes, {'status', 'images', 'containers'}, 'runtime_probes')
    if (probes.get('status') not in {'PASS', 'NOT_RUNTIME_VALIDATED'} or
            not isinstance(probes.get('images'), list) or
            not isinstance(probes.get('containers'), list)):
        raise CampaignError('CAMPAIGN_PROBE_INVALID', 'runtime probe shape invalid')
    for index, image in enumerate(probes['images']):
        label = 'runtime_probes.images[{}]'.format(index)
        if not isinstance(image, Mapping):
            raise CampaignError('CAMPAIGN_PROBE_INVALID', label)
        _reject_extra(image, {'id', 'repo_digests', 'os', 'architecture', 'manifest'}, label)
        if (not isinstance(image.get('id'), str) or not image['id'] or
                not isinstance(image.get('repo_digests'), list) or
                not all(isinstance(item, str) for item in image['repo_digests']) or
                image.get('os') != 'linux' or image.get('architecture') != 'amd64' or
                image.get('manifest') is not None and not isinstance(image['manifest'], Mapping)):
            raise CampaignError('CAMPAIGN_PROBE_INVALID', label)
        if isinstance(image.get('manifest'), Mapping):
            _reject_extra(image['manifest'], {
                'index_digest', 'index_media_type', 'linux_amd64_digest',
                'linux_amd64_media_type', 'linux_amd64_os', 'linux_amd64_architecture'},
                          label + '.manifest')
    for index, container in enumerate(probes['containers']):
        label = 'runtime_probes.containers[{}]'.format(index)
        if not isinstance(container, Mapping):
            raise CampaignError('CAMPAIGN_PROBE_INVALID', label)
        _reject_extra(container, {'name', 'status'}, label)
        if (not isinstance(container.get('name'), str) or not container['name'] or
                container.get('status') != 'ABSENT'):
            raise CampaignError('CAMPAIGN_PROBE_INVALID', label)
    rows = value.get('rows')
    if not isinstance(rows, list) or len(rows) != len(EXPECTED_ROWS):
        raise CampaignError('CAMPAIGN_ROWS_INVALID', 'campaign must contain exactly four rows')
    row_allowed = {
        'index', 'distro', 'dependency_leg', 'child_root', 'container_name', 'image',
        'status', 'command', 'started_at_unix', 'ended_at_unix',
        'host_receipt', 'host_status', 'phase_state', 'failure'}
    for index, (row, expected) in enumerate(zip(rows, EXPECTED_ROWS)):
        if not isinstance(row, Mapping):
            raise CampaignError('CAMPAIGN_ROWS_INVALID', 'row is not an object')
        _reject_extra(row, row_allowed, 'row {}'.format(index))
        if (row.get('index') != index or
                (row.get('distro'), row.get('dependency_leg')) != expected):
            raise CampaignError('CAMPAIGN_ROWS_INVALID', 'row order/identity changed')
        if status.startswith('PREFLIGHT'):
            required_row_fields = {
                'index', 'distro', 'dependency_leg', 'child_root', 'container_name',
                'image', 'status', 'phase_state'}
        else:
            required_row_fields = {
                'index', 'distro', 'dependency_leg', 'child_root', 'container_name',
                'status', 'command', 'started_at_unix', 'ended_at_unix',
                'host_receipt', 'phase_state', 'failure'}
        missing = sorted(required_row_fields - set(row))
        if missing:
            raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                'row {} missing {}'.format(index, missing))
        if status.startswith('PREFLIGHT'):
            _validate_image_contract(row.get('image'), 'row image')
        elif row.get('image') is not None:
            _validate_image_contract(row.get('image'), 'row image')
        if row.get('status') not in {'PASS', 'FAIL_CLOSED', 'NOT_STARTED'}:
            raise CampaignError('CAMPAIGN_ROWS_INVALID', 'row status invalid')
        phase = row.get('phase_state')
        if not isinstance(phase, Mapping):
            raise CampaignError('CAMPAIGN_PHASE_INVALID', 'row phase missing')
        _reject_extra(phase, {'docker', 'build_test', 'evidence'}, 'row phase')
        if (phase.get('docker') not in PHASE_STATES or
                phase.get('build_test') not in PHASE_STATES):
            raise CampaignError('CAMPAIGN_PHASE_INVALID', 'row phase state invalid')
        if phase.get('evidence') is not None:
            _validate_descriptor(phase['evidence'], 'row {} phase evidence'.format(index))
        if row.get('status') == 'PASS' and (phase.get('docker') != 'CONFIRMED' or
                                            phase.get('build_test') != 'CONFIRMED'):
            raise CampaignError('CAMPAIGN_PHASE_INVALID', 'PASS row lacks confirmed phases')
        if row.get('status') == 'PASS' and row.get('failure') is not None:
            raise CampaignError('CAMPAIGN_ROWS_INVALID', 'PASS row has failure')
        if row.get('status') == 'FAIL_CLOSED' and not isinstance(
                row.get('failure'), Mapping):
            raise CampaignError('CAMPAIGN_ROWS_INVALID', 'failed row lacks failure')
        row_failure = row.get('failure')
        if row_failure is not None:
            if not isinstance(row_failure, Mapping):
                raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                    'row failure is not an object')
            _reject_extra(row_failure, {'kind', 'phase', 'row', 'message'},
                          'row {} failure'.format(index))
            if (not isinstance(row_failure.get('kind'), str) or
                    not isinstance(row_failure.get('phase'), str) or
                    row_failure.get('phase') not in {
                        'before_first_row', 'before_row', 'leg', 'summary',
                        'after_root_creation', 'campaign_stop'} or
                    not isinstance(row_failure.get('message'), str)):
                raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                    'row {} failure identity invalid'.format(index))
        if status.startswith('PREFLIGHT'):
            if row.get('status') != 'NOT_STARTED' or set(row) != required_row_fields:
                raise CampaignError('CAMPAIGN_PREFLIGHT_INVALID',
                                    'preflight row has runtime fields')
        else:
            if not set(row).issubset(required_row_fields | {'host_status'}):
                raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                    'runtime row has unknown fields')
            command = row.get('command')
            unattempted_failure = row.get('status') == 'FAIL_CLOSED' and command is None
            if row.get('status') == 'NOT_STARTED' or unattempted_failure:
                if (command is not None or row.get('started_at_unix') is not None or
                        row.get('ended_at_unix') is not None or
                        row.get('host_receipt') is not None):
                    raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                        'unattempted row has runtime evidence')
            else:
                if (not isinstance(command, Mapping) or
                        set(command) != {'argv', 'argv_sha256'} or
                        not isinstance(command.get('argv'), list) or
                        not all(isinstance(item, str) for item in command['argv']) or
                        not SHA256_RE.fullmatch(str(command.get('argv_sha256', ''))) or
                        command['argv_sha256'] != _sha256_bytes(_canonical(command['argv']))):
                    raise CampaignError('CAMPAIGN_ROWS_INVALID',
                                        'started row command identity invalid')
                if not isinstance(row.get('started_at_unix'), (int, float)) or \
                        isinstance(row.get('started_at_unix'), bool) or \
                        not isinstance(row.get('ended_at_unix'), (int, float)) or \
                        isinstance(row.get('ended_at_unix'), bool):
                    raise CampaignError('CAMPAIGN_TIME_INVALID',
                                        'started row timestamps missing')
            host_receipt = row.get('host_receipt')
            if row.get('status') == 'PASS':
                if not isinstance(host_receipt, Mapping):
                    raise CampaignError('CAMPAIGN_ARTIFACT_INVALID',
                                        'PASS row host receipt missing')
                _reject_extra(host_receipt, {
                    'path', 'file_bytes', 'file_sha256', 'sidecar_sha256',
                    'sidecar_path', 'inner_path', 'inner_sha256'},
                              'row {} host receipt'.format(index))
                if (not isinstance(host_receipt.get('inner_path'), str) or
                        not host_receipt['inner_path'] or
                        not SHA256_RE.fullmatch(str(host_receipt.get('inner_sha256', '')))):
                    raise CampaignError('CAMPAIGN_ARTIFACT_INVALID',
                                        'PASS row inner receipt identity invalid')
                _validate_descriptor(
                    {key: host_receipt[key] for key in (
                        'path', 'file_bytes', 'file_sha256', 'sidecar_sha256',
                        'sidecar_path')},
                    'row {} host receipt'.format(index))
            elif host_receipt is not None:
                raise CampaignError('CAMPAIGN_ARTIFACT_INVALID',
                                    'non-PASS row has host receipt')
        for time_key in ('started_at_unix', 'ended_at_unix'):
            if row.get(time_key) is not None and not isinstance(
                    row.get(time_key), (int, float)):
                raise CampaignError('CAMPAIGN_TIME_INVALID', 'row timestamp invalid')
        if row.get('started_at_unix') is not None and row.get('ended_at_unix') is not None and \
                row['ended_at_unix'] < row['started_at_unix']:
            raise CampaignError('CAMPAIGN_TIME_INVALID', 'row timestamps are reversed')
    _validate_partial_rows(value, rows, campaign, profile, source, git)
    summary = value.get('summary')
    if not isinstance(summary, Mapping):
        raise CampaignError('CAMPAIGN_SUMMARY_INVALID', 'summary missing')
    _reject_extra(
        summary, {'status', 'reason', 'path', 'file_sha256', 'sidecar_sha256'},
        'summary')
    summary_status = summary.get('status')
    if summary_status not in {'PASS', 'NOT_RUN', 'FAIL_CLOSED'}:
        raise CampaignError('CAMPAIGN_SUMMARY_INVALID', 'summary status invalid')
    if summary_status == 'PASS':
        if (not isinstance(summary.get('path'), str) or
                not SHA256_RE.fullmatch(summary.get('file_sha256', '')) or
                not SHA256_RE.fullmatch(summary.get('sidecar_sha256', ''))):
            raise CampaignError('CAMPAIGN_SUMMARY_INVALID', 'PASS summary lacks sealed identity')
    elif any(summary.get(key) is not None for key in (
            'path', 'file_sha256', 'sidecar_sha256')):
        raise CampaignError('CAMPAIGN_SUMMARY_INVALID', 'non-PASS summary has artifact')
    failure = value.get('failure')
    if failure is not None:
        if not isinstance(failure, Mapping):
            raise CampaignError('CAMPAIGN_FAILURE_INVALID', 'failure is not an object')
        _reject_extra(failure, {'kind', 'phase', 'row', 'message'}, 'failure')
        if (not isinstance(failure.get('kind'), str) or
                not isinstance(failure.get('phase'), str) or
                failure.get('phase') not in {
                    'before_first_row', 'before_row', 'leg', 'summary',
                    'after_root_creation'}):
            raise CampaignError('CAMPAIGN_FAILURE_INVALID', 'failure identity missing')
    execution_started = value.get('execution_started')
    if status.startswith('PREFLIGHT'):
        if execution_started is not False or failure is not None or summary_status != 'NOT_RUN':
            raise CampaignError('CAMPAIGN_PREFLIGHT_INVALID', 'preflight/runtime fields mixed')
        expected_probe_status = 'PASS' if status == 'PREFLIGHT_PASS' else 'NOT_RUNTIME_VALIDATED'
        if probes.get('status') != expected_probe_status:
            raise CampaignError('CAMPAIGN_PREFLIGHT_INVALID',
                                'preflight probe status does not match report')
        if any(row['status'] != 'NOT_STARTED' for row in rows):
            raise CampaignError('CAMPAIGN_PREFLIGHT_INVALID', 'preflight has started row')
    else:
        if execution_started is not True:
            raise CampaignError('CAMPAIGN_RUNTIME_INVALID', 'runtime execution flag missing')
        execution = value.get('execution')
        if not isinstance(execution, Mapping):
            raise CampaignError('CAMPAIGN_RUNTIME_INVALID', 'execution record missing')
        _reject_extra(execution, {
            'row_order', 'retries', 'parallelism', 'launcher_authority',
            'summary_authority', 'failure_phase'}, 'execution')
        if (execution.get('row_order') != [list(item) for item in EXPECTED_ROWS] or
                execution.get('retries') != 0 or execution.get('parallelism') != 1):
            raise CampaignError('CAMPAIGN_RUNTIME_INVALID', 'execution policy changed')
        if status == 'PASS' and (failure is not None or summary_status != 'PASS' or
                                 any(row['status'] != 'PASS' for row in rows)):
            raise CampaignError('CAMPAIGN_RUNTIME_INVALID', 'PASS consistency failure')
        if status == 'PASS' and probes.get('status') != 'PASS':
            raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                'PASS campaign lacks runtime probe evidence')
        if status != 'PASS':
            if not isinstance(failure, Mapping):
                raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                    'failed campaign lacks failure record')
            failed_indices = [row['index'] for row in rows
                              if row['status'] == 'FAIL_CLOSED']
            if summary_status == 'PASS':
                raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                    'partial campaign has PASS summary')
            if failure['phase'] == 'summary':
                if failed_indices or any(row['status'] != 'PASS' for row in rows):
                    raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                        'summary failure has inconsistent rows')
            else:
                if len(failed_indices) != 1:
                    raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                        'runtime failure must identify one failed row')
                failed_index = failed_indices[0]
                if (any(row['status'] != 'PASS' for row in rows[:failed_index]) or
                        any(row['status'] != 'NOT_STARTED' for row in rows[failed_index + 1:])):
                    raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                        'rows after failure are not stopped')
                if failure.get('row') is not None and failure['row'] != failed_index:
                    raise CampaignError('CAMPAIGN_RUNTIME_INVALID',
                                        'failure row does not match failed row')
    safety = value.get('safety')
    if not isinstance(safety, Mapping):
        raise CampaignError('CAMPAIGN_SAFETY_INVALID', 'safety missing')
    _reject_extra(safety, {'docker', 'build_test', 'bag_opened',
                           'ground_truth_content_opened', 'scorer_invoked',
                           'map_saved', 'formal_replay_started'}, 'safety')
    _validate_phase_object(safety.get('docker'), 'safety.docker')
    _validate_phase_object(safety.get('build_test'), 'safety.build_test')
    for key in ('bag_opened', 'ground_truth_content_opened', 'scorer_invoked',
                'map_saved', 'formal_replay_started'):
        if safety.get(key) is not False:
            raise CampaignError('CAMPAIGN_SAFETY_INVALID', '{} must be false'.format(key))
    for key in ('docker', 'build_test'):
        expected = _aggregate_phase_state(rows, key)['state']
        if safety[key].get('state') != expected:
            raise CampaignError(
                'CAMPAIGN_SAFETY_INVALID',
                '{} state is not row-derived'.format(key))
    for key in ('started_at_unix', 'ended_at_unix'):
        if not isinstance(value.get(key), (int, float)) or value[key] < 0:
            raise CampaignError('CAMPAIGN_TIME_INVALID', key)
    if value['ended_at_unix'] < value['started_at_unix']:
        raise CampaignError('CAMPAIGN_TIME_INVALID', 'campaign timestamps are reversed')
    if 'canonical_sha256' in value and value['canonical_sha256'] != _canonical_hash(value):
        raise CampaignError('CAMPAIGN_BINDING_INVALID', 'canonical hash changed')


def _validate_host_receipt(
        path: Path, row: Mapping[str, Any], snapshot: Mapping[str, Any],
        campaign: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    value, descriptor = _read_existing_receipt(
        path, 'registration-plugin-host-release-leg-v1', 'host leg receipt')
    if (value.get('status') != 'PASS' or value.get('distro') != row['distro'] or
            value.get('dependency_leg') != row['dependency_leg'] or
            value.get('campaign_set') != campaign or
            value.get('profile') != snapshot['profile'] or
            value.get('source', {}).get('manifest_sha256') !=
            snapshot['source']['manifest_sha256']):
        raise CampaignError('HOST_RECEIPT_IDENTITY_INVALID', str(path))
    container = value.get('container', {})
    if (container.get('name') != row['container_name'] or
            container.get('start_count') != 1 or
            value.get('cleanup', {}).get('post_remove_absent') is not True):
        raise CampaignError('HOST_RECEIPT_CLEANUP_INVALID', str(path))
    safety = value.get('safety', {})
    if any(safety.get(key) is not False for key in (
            'bag_opened', 'ground_truth_content_opened', 'scorer_invoked',
            'map_saved', 'formal_replay_started')):
        raise CampaignError('HOST_RECEIPT_SAFETY_INVALID', str(path))
    binding = value.get('runner_receipt_binding')
    if not isinstance(binding, Mapping) or binding.get('status') != 'PASS':
        raise CampaignError('INNER_RECEIPT_BINDING_MISSING', str(path))
    relative = binding.get('path_relative')
    if (not isinstance(relative, str) or not relative or relative.startswith('/') or
            any(part in {'', '.', '..'} for part in relative.split('/'))):
        raise CampaignError('INNER_RECEIPT_PATH_INVALID', str(path))
    inner = path.parent / relative
    inner_value, inner_desc = _read_existing_receipt(
        inner, 'registration-plugin-release-matrix-v1', 'inner leg receipt')
    if (inner_value.get('status') != 'PASS' or inner_value.get('distro') != row['distro'] or
            inner_value.get('dependency_leg') != row['dependency_leg'] or
            inner_value.get('campaign_set') != campaign or
            inner_value.get('profile') != snapshot['profile'] or
            inner_value.get('repository', {}).get('source_manifest', {}).get('manifest_sha256') !=
            snapshot['source']['manifest_sha256']):
        raise CampaignError('INNER_RECEIPT_IDENTITY_INVALID', str(inner))
    if binding.get('sha256') != inner_desc['file_sha256']:
        raise CampaignError('INNER_RECEIPT_HASH_INVALID', str(inner))
    return value, descriptor, inner


def _row_command(repo_root: Path, profile_path: Path, row: Mapping[str, Any],
                 child_root: Path) -> list[str]:
    return [
        sys.executable, str(repo_root / 'scripts/run_registration_plugin_release_leg.py'),
        '--repo-root', str(repo_root), '--profile', str(profile_path),
        '--distro', row['distro'], '--dependency-leg', row['dependency_leg'],
        '--evidence-root', str(child_root),
    ]


def _run_campaign_owned(repo_root: Path, profile_path: Path,
                        campaign_root: Path, *,
                        image_probe: Callable | None = None,
                        container_probe: Callable | None = None,
                        leg_executor: Callable | None = None,
                        summary_executor: Callable | None = None,
                        require_storage: bool = True,
                        now: Callable[[], float] = time.time,
                        _preflight: Mapping[str, Any] | None = None,
                        _owned_root: Path | None = None
                        ) -> tuple[dict[str, Any], dict[str, Any]]:
    """Execute rows after the caller has reserved the campaign root."""
    preflight = _preflight or build_preflight(
        repo_root, profile_path, campaign_root, image_probe=image_probe,
        container_probe=container_probe, require_storage=require_storage,
        runtime_probes_required=True,
    )
    if preflight['status'] != 'PREFLIGHT_PASS':
        raise CampaignError('PREFLIGHT_NOT_READY', preflight['status'])
    root = Path(preflight['campaign_root']['path'])
    if _owned_root is None:
        _fresh_directory(root, 'campaign root')
    profile, release, profile_path = _load_profile(Path(profile_path).absolute())
    snapshot = _source_snapshot(Path(repo_root).absolute(), profile, profile_path)
    if not _snapshot_equal(snapshot, {
            'profile': preflight['profile'], 'source': preflight['source_manifest'],
            'git': preflight['git']}):
        raise CampaignError('PREFLIGHT_IDENTITY_CHANGED', 'source changed before first row')
    rows: list[dict[str, Any]] = []
    partial_rows: list[dict[str, Any]] = []
    inner_paths: list[Path] = []
    failure: dict[str, Any] | None = None
    started = now()
    for row in preflight['rows']:
        current, current_release, current_profile_path = _load_profile(profile_path)
        current_snapshot = _source_snapshot(
            Path(repo_root).absolute(), current, current_profile_path)
        if (current_release['campaign_set'] != release['campaign_set'] or
                not _snapshot_equal(current_snapshot, snapshot)):
            failure = {'kind': 'SAFETY_IDENTITY_DRIFT',
                       'phase': 'before_row', 'row': row['index'],
                       'message': 'profile/source/dirty-tree identity changed'}
            break
        if require_storage:
            try:
                observed_storage = leg_launcher._validate_evidence_storage(
                    root, release['evidence_storage'])
                observed_storage = dict(observed_storage)
                observed_storage['status'] = 'PASS'
            except Exception as error:
                failure = {'kind': 'EVIDENCE_STORAGE_DRIFT', 'phase': 'before_row',
                           'row': row['index'], 'message': str(error)}
                break
            if observed_storage != preflight['evidence_storage']:
                failure = {'kind': 'EVIDENCE_STORAGE_DRIFT', 'phase': 'before_row',
                           'row': row['index'], 'message': 'storage identity changed'}
                break
        try:
            _runtime_probes([row], image_probe=image_probe,
                            container_probe=container_probe, required=True)
        except CampaignError as error:
            failure = {'kind': error.kind, 'phase': 'before_row',
                       'row': row['index'], 'message': str(error)}
            break
        child = Path(row['child_root'])
        _fresh_target(str(child), 'row evidence root')
        command = _row_command(Path(repo_root).absolute(), profile_path, row, child)
        row_report: dict[str, Any] = {
            'index': row['index'], 'distro': row['distro'],
            'dependency_leg': row['dependency_leg'], 'child_root': str(child),
            'container_name': row['container_name'], 'status': 'NOT_STARTED',
            'command': {'argv': command, 'argv_sha256': _sha256_bytes(_canonical(command))},
            'started_at_unix': now(), 'ended_at_unix': None,
            'host_receipt': None, 'failure': None,
            'phase_state': {'docker': 'NOT_STARTED', 'build_test': 'NOT_STARTED',
                            'evidence': None},
        }
        rows.append(row_report)
        host_path = child / 'registration_plugin_host.receipt.json'
        leg_invoked = False
        try:
            leg_invoked = True
            if leg_executor is None:
                leg_result = leg_launcher.run_leg(
                    repo_root, profile_path, row['distro'], row['dependency_leg'], child)
            else:
                leg_result = leg_executor(row, child, command)
            if isinstance(leg_result, Mapping) and leg_result.get('closure', {}).get('path'):
                host_path = Path(leg_result['closure']['path'])
            row_report['phase_state'] = _phase_state_for_leg(
                host_path, invoked=leg_invoked)
            host_value, host_desc, inner_path = _validate_host_receipt(
                host_path, row, snapshot, release['campaign_set'])
            row_report.update({
                'status': 'PASS', 'ended_at_unix': now(),
                'host_receipt': {**host_desc, 'inner_path': str(inner_path),
                                 'inner_sha256': _sha256_file(inner_path)},
                'host_status': host_value['status'],
            })
            inner_paths.append(inner_path)
        except Exception as error:
            row_report['phase_state'] = _phase_state_for_leg(
                host_path, invoked=leg_invoked)
            failure_value = {'kind': getattr(error, 'kind', 'LEG_FAILURE'),
                             'phase': 'leg', 'row': row['index'],
                             'message': str(error)}
            if leg_invoked:
                try:
                    partial_rows.append(_record_partial_row(
                        root, row, release['campaign_set'], snapshot,
                        failure_value, row_report['started_at_unix'],
                        row_report['ended_at_unix'] or now()))
                except Exception as partial_error:
                    failure_value = {
                        'kind': 'PARTIAL_RECEIPT_SEAL_FAILED', 'phase': 'leg',
                        'row': row['index'],
                        'message': '{}; partial receipt: {}'.format(
                            error, partial_error),
                    }
            row_report.update({
                'status': 'FAIL_CLOSED', 'ended_at_unix': now(),
                'failure': failure_value,
            })
            failure = row_report['failure']
            break
    if failure is not None:
        started_rows = {item['index'] for item in rows}
        if (failure.get('phase') in {'before_first_row', 'before_row'} and
                not any(item['status'] == 'FAIL_CLOSED' for item in rows)):
            blocked_index = min(
                (index for index in range(len(EXPECTED_ROWS)) if index not in started_rows),
                default=None)
            if blocked_index is not None:
                blocked_distro, blocked_leg = EXPECTED_ROWS[blocked_index]
                blocked = next(item for item in preflight['rows']
                               if item['index'] == blocked_index)
                failure['row'] = blocked_index
                rows.append({
                    'index': blocked_index, 'distro': blocked_distro,
                    'dependency_leg': blocked_leg,
                    'child_root': blocked['child_root'],
                    'container_name': blocked['container_name'],
                    'status': 'FAIL_CLOSED', 'command': None,
                    'started_at_unix': None, 'ended_at_unix': None,
                    'host_receipt': None,
                    'phase_state': {'docker': 'NOT_STARTED', 'build_test': 'NOT_STARTED',
                                    'evidence': None},
                    'failure': dict(failure),
                })
                started_rows.add(blocked_index)
        rows.extend({
            'index': index, 'distro': distro, 'dependency_leg': dependency_leg,
            'child_root': str(root / 'registration-plugin-release-{}-{}'.format(
                distro, dependency_leg)),
            'container_name': leg_launcher._container_name(
                root / 'registration-plugin-release-{}-{}'.format(distro, dependency_leg),
                distro, dependency_leg),
            'status': 'NOT_STARTED', 'command': None, 'started_at_unix': None,
            'ended_at_unix': None, 'host_receipt': None,
            'phase_state': {'docker': 'NOT_STARTED', 'build_test': 'NOT_STARTED',
                            'evidence': None},
            'failure': {'kind': 'NOT_STARTED_AFTER_FAILURE',
                        'phase': 'campaign_stop',
                        'message': 'campaign stopped after first failed row'},
        } for index, (distro, dependency_leg) in enumerate(EXPECTED_ROWS)
          if index not in started_rows)
    summary_value: dict[str, Any] | None = None
    summary_desc: dict[str, Any] | None = None
    summary_status = 'NOT_RUN'
    if failure is None and len(rows) == len(EXPECTED_ROWS) and all(
            item['status'] == 'PASS' for item in rows):
        try:
            final_profile, final_release, final_profile_path = _load_profile(profile_path)
            final_snapshot = _source_snapshot(
                Path(repo_root).absolute(), final_profile, final_profile_path)
            if (final_release['campaign_set'] != release['campaign_set'] or
                    not _snapshot_equal(final_snapshot, snapshot)):
                raise CampaignError(
                    'SAFETY_IDENTITY_DRIFT',
                    'profile/source/dirty-tree identity changed before summary')
            summary_path = root / 'registration_plugin_release_matrix.summary.json'
            if summary_executor is None:
                summary_result = release_summary.summarize(
                    profile_path, inner_paths, summary_path)
            else:
                summary_result = summary_executor(profile_path, inner_paths, summary_path)
            summary_path = (
                Path(summary_result[0]['path'])
                if isinstance(summary_result, tuple) else summary_path)
            summary_value, summary_desc = _read_existing_receipt(
                summary_path, 'registration-plugin-release-matrix-summary-v1', 'campaign summary')
            if (summary_value.get('status') != 'PASS' or
                    summary_value.get('campaign_set') != release['campaign_set'] or
                    summary_value.get('profile', {}).get('sha256') !=
                    snapshot['profile']['sha256'] or
                    summary_value.get('source_manifest_sha256') !=
                    snapshot['source']['manifest_sha256']):
                raise CampaignError('SUMMARY_IDENTITY_INVALID', str(summary_path))
            summary_status = 'PASS'
        except Exception as error:
            failure = {'kind': getattr(error, 'kind', 'SUMMARY_FAILURE'),
                       'phase': 'summary', 'message': str(error)}
            summary_status = 'FAIL_CLOSED'
    report = {
        'schema': SCHEMA, 'schema_version': 1,
        'status': ('PASS' if failure is None and summary_status == 'PASS' else
                   'PARTIAL_FAILURE' if rows else 'FAIL_CLOSED'),
        'benchmark_eligible': False, 'execution_started': True,
        'campaign_id': release['campaign_set']['campaign_id'],
        'campaign_set': release['campaign_set'], 'profile': snapshot['profile'],
        'source_manifest': snapshot['source'], 'git': snapshot['git'],
        'campaign_root': {'path': str(root), 'fresh': True,
                          'identity': preflight['campaign_root']['identity']},
        'evidence_storage': preflight['evidence_storage'],
        'runtime_probes': preflight.get('runtime_probes'),
        'rows': sorted(rows, key=lambda item: item['index']),
        'partial_rows': sorted(partial_rows, key=lambda item: item['index']),
        'partial_row_count': len(partial_rows),
        'summary': {'status': summary_status,
                    'path': summary_desc['path'] if summary_desc else None,
                    'file_sha256': summary_desc['file_sha256'] if summary_desc else None,
                    'sidecar_sha256': summary_desc['sidecar_sha256'] if summary_desc else None},
        'failure': failure,
        'execution': {'row_order': [list(item) for item in EXPECTED_ROWS],
                      'retries': 0, 'parallelism': 1,
                      'launcher_authority': str(
                          ROOT / 'scripts/run_registration_plugin_release_leg.py'),
                      'summary_authority': str(
                          ROOT / 'scripts/summarize_registration_plugin_release_matrix.py')},
        'safety': {'docker': _aggregate_phase_state(rows, 'docker'),
                   'build_test': _aggregate_phase_state(rows, 'build_test'),
                   'bag_opened': False, 'ground_truth_content_opened': False,
                   'scorer_invoked': False, 'map_saved': False,
                   'formal_replay_started': False},
        'started_at_unix': started, 'ended_at_unix': now(),
    }
    _validate_campaign_report(report)
    seal = _seal_pair(root / 'registration_plugin_release_campaign.receipt.json', report)
    report['seal'] = seal
    return report, seal


def _failure_phase(kind: str) -> str:
    if kind in {'PREFLIGHT_IDENTITY_CHANGED', 'CURRENT_SOURCE_HASH_MISMATCH',
                'REPOSITORY_INVALID'}:
        return 'before_first_row'
    if kind in {'EVIDENCE_STORAGE_DRIFT', 'EVIDENCE_STORAGE_INVALID',
                'RUNTIME_PROBE_REQUIRED', 'CONTAINER_NAME_ALREADY_EXISTS',
                'IMAGE_PROBE_FAILED', 'IMAGE_UNAVAILABLE',
                'IMAGE_PROBE_INVALID'}:
        return 'before_row'
    return 'after_root_creation'


def _failure_report(preflight: Mapping[str, Any], root: Path,
                    error: BaseException, now: Callable[[], float]) -> dict[str, Any]:
    kind = getattr(error, 'kind', 'CAMPAIGN_FAILURE')
    phase = _failure_phase(kind)
    rows = []
    for index, row in enumerate(preflight['rows']):
        is_failure_row = index == 0
        rows.append({
            'index': index, 'distro': row['distro'],
            'dependency_leg': row['dependency_leg'],
            'child_root': row['child_root'],
            'container_name': row['container_name'],
            'status': 'FAIL_CLOSED' if is_failure_row else 'NOT_STARTED',
            'command': None, 'started_at_unix': None, 'ended_at_unix': None,
            'host_receipt': None,
            'phase_state': {'docker': 'NOT_STARTED', 'build_test': 'NOT_STARTED',
                            'evidence': None},
            'failure': ({'kind': kind, 'phase': phase, 'message': str(error)}
                        if is_failure_row else {
                            'kind': 'NOT_STARTED_AFTER_FAILURE',
                            'phase': 'campaign_stop',
                            'message': 'campaign stopped before this row started'}),
        })
    timestamp = now()
    return {
        'schema': SCHEMA, 'schema_version': 1, 'status': 'FAIL_CLOSED',
        'benchmark_eligible': False, 'execution_started': True,
        'campaign_id': preflight['campaign_id'],
        'campaign_set': preflight['campaign_set'], 'profile': preflight['profile'],
        'source_manifest': preflight['source_manifest'], 'git': preflight['git'],
        'campaign_root': {'path': str(root), 'fresh': True,
                          'identity': preflight['campaign_root']['identity']},
        'evidence_storage': preflight['evidence_storage'],
        'runtime_probes': preflight.get('runtime_probes'), 'rows': rows,
        'partial_rows': [], 'partial_row_count': 0,
        'summary': {'status': 'NOT_RUN', 'path': None, 'file_sha256': None,
                    'sidecar_sha256': None},
        'failure': {'kind': kind, 'phase': phase, 'message': str(error)},
        'execution': {
            'row_order': [list(item) for item in EXPECTED_ROWS],
            'retries': 0, 'parallelism': 1,
            'launcher_authority': str(ROOT / 'scripts/run_registration_plugin_release_leg.py'),
            'summary_authority': str(
                ROOT / 'scripts/summarize_registration_plugin_release_matrix.py'),
            'failure_phase': phase,
        },
        'safety': {'docker': {'state': 'NOT_STARTED', 'evidence': []},
                   'build_test': {'state': 'NOT_STARTED', 'evidence': []},
                   'bag_opened': False, 'ground_truth_content_opened': False,
                   'scorer_invoked': False, 'map_saved': False,
                   'formal_replay_started': False},
        'started_at_unix': timestamp, 'ended_at_unix': timestamp,
    }


def run_campaign(repo_root: Path, profile_path: Path, campaign_root: Path, *,
                 image_probe: Callable | None = None,
                 container_probe: Callable | None = None,
                 leg_executor: Callable | None = None,
                 summary_executor: Callable | None = None,
                 require_storage: bool = True,
                 now: Callable[[], float] = time.time
                 ) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reserve a fresh root, then seal every post-reservation failure."""
    preflight = build_preflight(
        repo_root, profile_path, campaign_root, image_probe=image_probe,
        container_probe=container_probe, require_storage=require_storage,
        runtime_probes_required=True,
    )
    if preflight['status'] != 'PREFLIGHT_PASS':
        raise CampaignError('PREFLIGHT_NOT_READY', preflight['status'])
    root = Path(preflight['campaign_root']['path'])
    _fresh_directory(root, 'campaign root')
    try:
        return _run_campaign_owned(
            repo_root, profile_path, campaign_root,
            image_probe=image_probe, container_probe=container_probe,
            leg_executor=leg_executor, summary_executor=summary_executor,
            require_storage=require_storage, now=now,
            _preflight=preflight, _owned_root=root)
    except Exception as error:
        report = _failure_report(preflight, root, error, now)
        try:
            _validate_campaign_report(report)
            seal = _seal_pair(root / 'registration_plugin_release_campaign.receipt.json', report)
        except Exception as seal_error:
            raise CampaignError(
                'CAMPAIGN_SEAL_FAILED',
                '{}; failure receipt could not be sealed: {}'.format(
                    error, seal_error)) from seal_error
        report['seal'] = seal
        return report, seal


def run_preflight(
        repo_root: Path, profile_path: Path, campaign_root: Path, output: Path,
        *, image_probe: Callable | None = None,
        container_probe: Callable | None = None
) -> tuple[dict[str, Any], dict[str, Any]]:
    _, release, _ = _load_profile(Path(profile_path).absolute())
    output = _validate_preflight_output(output, release)
    report = build_preflight(repo_root, profile_path, campaign_root,
                             image_probe=image_probe,
                             container_probe=container_probe,
                             require_storage=False,
                             runtime_probes_required=False)
    _validate_campaign_report(report)
    seal = _seal_pair(output, report)
    return report, seal


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', default=str(ROOT))
    parser.add_argument('--profile', default=str(PROFILE))
    parser.add_argument('--campaign-root', required=True)
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--output', help='fresh preflight output; required with --preflight-only')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.preflight_only:
            if not args.output:
                raise CampaignError(
                    'OUTPUT_REQUIRED', '--output is required with --preflight-only')
            report, seal = run_preflight(
                Path(args.repo_root), Path(args.profile), Path(args.campaign_root),
                Path(args.output))
        else:
            report, seal = run_campaign(
                Path(args.repo_root), Path(args.profile), Path(args.campaign_root),
                image_probe=_image_probe_default, container_probe=_container_probe_default)
        print(json.dumps({'status': report['status'], 'receipt': seal}, sort_keys=True))
        return 0 if report['status'] in {'PASS', 'PREFLIGHT_PASS',
                                         'PREFLIGHT_NOT_RUNTIME_VALIDATED'} else 1
    except Exception as error:
        print(json.dumps({'schema': SCHEMA, 'schema_version': 1,
                          'status': 'FAIL_CLOSED',
                          'failure': {'kind': getattr(error, 'kind', 'CAMPAIGN_FAILURE'),
                                      'message': str(error)}}, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
