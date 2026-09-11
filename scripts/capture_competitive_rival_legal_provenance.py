#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Seal a non-promoting review packet for the current rival source closure.

This tool is intentionally offline.  It reopens the checked-in profile,
current r2 selection, local recipe bytes, and the offline closure checker, then
creates a bounded immutable packet describing what still needs independent
upstream/custodian review.  It never turns package metadata into a license,
never follows a redirect, and never calls an upstream URL.  Remote request
records are emitted as ``NOT_RUN`` placeholders and are not evidence of an
upstream response.
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
from typing import Any, Mapping
from urllib.parse import quote, urlsplit

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts.check_competitive_rival_source_closure import (  # noqa: E402
    ARCHIVE_RE,
    CLOSURE_IDENTITY_HASH_KIND,
    COMMIT_URL_RE,
    current_rival_source_closure_identity,
    GITHUB_REPOSITORY_RE,
    ROOT,
    sha256_file,
    sha256_tree,
    verify_rival_source_closure,
)
from scripts.prepare_competitive_execution_selection_r3_handoff import (  # noqa: E402
    _exclusive_write,
    _read_immutable,
    _remove_owned,
    CandidateError,
    MAX_HANDOFF_BYTES,
)

import yaml  # noqa: E402


CAPTURE_SCHEMA_VERSION = 1
CAPTURE_KIND = 'competitive_rival_legal_provenance_capture_v1'
CAPTURE_FILE = 'legal_provenance_capture.json'
CAPTURE_SIDECAR = CAPTURE_FILE + '.sha256'
PROFILE_REL = 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SYSTEMS = ('glim', 'fast_livo2')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
REVISION_RE = re.compile(r'^[0-9a-f]{40}$')
ISO_UTC_RE = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$')
MAX_LOCAL_FILE_BYTES = 64 * 1024 * 1024
MAX_CAPTURE_BYTES = min(MAX_HANDOFF_BYTES, 8 * 1024 * 1024)
MAX_REMOTE_RESPONSE_BYTES = 2 * 1024 * 1024
REMOTE_HOSTS = frozenset({'github.com', 'raw.githubusercontent.com', 'api.github.com'})
SAFE_RESPONSE_HEADERS = frozenset(
    {'content-length', 'content-type', 'etag', 'last-modified'})
RECIPE_ARTIFACT_ROLES = ('dockerfile', 'build_script', 'runner', 'wrapper')
PINNED_ARCHIVE_PATH_KIND = 'pinned_upstream_archive_relative_path'
PINNED_ARCHIVE_STATUS = 'PINNED_UPSTREAM_ARCHIVE_BOUND'


class LegalCaptureError(ValueError):
    """A malformed, stale, unsafe, or non-promoting legal packet."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _identity(value: Mapping[str, Any], field: str) -> str:
    return _sha(_canonical({key: item for key, item in value.items()
                            if key != field}))


def _require_absolute(path: Path, label: str) -> Path:
    if not path.is_absolute() or '..' in path.parts:
        raise LegalCaptureError(f'{label} must be absolute and traversal-free')
    return path


def _assert_no_symlink_ancestors(path: Path, label: str) -> None:
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise LegalCaptureError(f'{label} contains a symlink: {current}')


def _directory_identity(path: Path, label: str) -> dict[str, int]:
    path = _require_absolute(path, label)
    _assert_no_symlink_ancestors(path, label)
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise LegalCaptureError(f'{label} cannot be inspected: {exc}') from exc
    if not stat.S_ISDIR(info.st_mode) or path.is_symlink():
        raise LegalCaptureError(f'{label} must be a real directory')
    return {'device': info.st_dev, 'inode': info.st_ino,
            'mode': stat.S_IMODE(info.st_mode)}


def _same_identity(first: Mapping[str, int], second: Mapping[str, int]) -> bool:
    return dict(first) == dict(second)


def _safe_relative(root: Path, value: Any, label: str,
                   *, allow_directory: bool = True) -> Path:
    root = _require_absolute(root, 'repository root')
    _directory_identity(root, 'repository root')
    if not isinstance(value, str) or not value or value.startswith('/'):
        raise LegalCaptureError(f'{label} must be repository-relative')
    relative = Path(value)
    if relative.is_absolute() or '..' in relative.parts:
        raise LegalCaptureError(f'{label} escapes repository root')
    current = root
    for component in relative.parts:
        current /= component
        if current.is_symlink():
            raise LegalCaptureError(f'{label} contains a symlink')
    try:
        info = os.lstat(current)
    except OSError as exc:
        raise LegalCaptureError(f'{label} is unavailable: {exc}') from exc
    if stat.S_ISREG(info.st_mode):
        if info.st_nlink != 1:
            raise LegalCaptureError(f'{label} must be single-link')
    elif stat.S_ISDIR(info.st_mode) and allow_directory:
        pass
    else:
        raise LegalCaptureError(f'{label} must be a regular file or directory')
    return current


def _read_repository_file(root: Path, relative: str, label: str) -> bytes:
    path = _safe_relative(root, relative, label, allow_directory=False)
    try:
        payload, _ = _read_immutable(
            path, label, max_bytes=MAX_LOCAL_FILE_BYTES, read_only=False)
    except CandidateError as exc:
        raise LegalCaptureError(str(exc)) from exc
    return payload


def _profile_path(root: Path, profile_path: Path | str) -> tuple[Path, str]:
    candidate = Path(profile_path)
    if candidate.is_absolute():
        path = _safe_relative(root, candidate.relative_to(root).as_posix(),
                              'profile path', allow_directory=False)
    else:
        path = _safe_relative(root, candidate.as_posix(), 'profile path',
                              allow_directory=False)
    return path, path.relative_to(root).as_posix()


def _timestamp(value: str | None) -> str:
    if value is None:
        return dt.datetime.now(dt.timezone.utc).replace(
            microsecond=0).isoformat().replace('+00:00', 'Z')
    if ISO_UTC_RE.fullmatch(value) is None:
        raise LegalCaptureError('captured_at must be an RFC3339 UTC timestamp')
    try:
        parsed = dt.datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise LegalCaptureError('captured_at is not a valid timestamp') from exc
    if parsed.tzinfo is None:
        raise LegalCaptureError('captured_at must include UTC')
    return value


def _sha_descriptor(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise LegalCaptureError(f'{label} must be a lowercase SHA-256')
    return value


def _artifact_observation(root: Path, descriptor: Any, label: str) -> dict[str, Any]:
    if not isinstance(descriptor, Mapping):
        raise LegalCaptureError(f'{label} must be a mapping')
    relative = descriptor.get('path')
    declared = descriptor.get('sha256')
    kind = descriptor.get('hash_kind')
    if 'path_kind' in descriptor:
        raise LegalCaptureError(
            f'{label} has an unsupported non-local path_kind')
    if not isinstance(relative, str) or not relative or not isinstance(kind, str):
        raise LegalCaptureError(f'{label} path/hash kind is missing')
    if not isinstance(declared, str) or SHA_RE.fullmatch(declared) is None:
        raise LegalCaptureError(f'{label}.sha256 is malformed')
    result: dict[str, Any] = {
        'path': relative,
        'hash_kind': kind,
        'declared_sha256': declared.lower(),
        'actual_sha256': None,
        'status': 'MISSING',
    }
    try:
        path = _safe_relative(root, relative, label)
    except LegalCaptureError as exc:
        result['status'] = 'INVALID_PATH'
        result['reason'] = str(exc)
        return result
    try:
        metadata = os.lstat(path)
        if stat.S_ISREG(metadata.st_mode):
            if metadata.st_size <= 0 or metadata.st_size > MAX_LOCAL_FILE_BYTES:
                raise LegalCaptureError(f'{label} exceeds local size bound')
            actual = sha256_file(path)
        elif stat.S_ISDIR(metadata.st_mode):
            actual = sha256_tree(path)
        else:
            raise LegalCaptureError(f'{label} is not a regular file/directory')
        result['actual_sha256'] = actual
        result['status'] = 'MATCH' if actual.lower() == declared.lower() else 'DRIFT'
    except (OSError, ValueError, LegalCaptureError) as exc:
        result['status'] = 'INVALID_PATH'
        result['reason'] = str(exc)
    return result


def _pinned_archive_observation(descriptor: Any,
                                sources: list[Mapping[str, Any]],
                                label: str) -> dict[str, Any]:
    """Bind an archive-relative artifact without pretending it was local."""
    if not isinstance(descriptor, Mapping):
        raise LegalCaptureError(f'{label} must be a mapping')
    required = {
        'path', 'path_kind', 'hash_kind', 'sha256', 'source', 'role', 'status'}
    if set(descriptor) != required or \
            descriptor.get('path_kind') != PINNED_ARCHIVE_PATH_KIND or \
            descriptor.get('hash_kind') != 'upstream_archive_file_sha256':
        raise LegalCaptureError(f'{label} pinned archive descriptor is not exact')
    path = descriptor.get('path')
    source_name = descriptor.get('source')
    role = descriptor.get('role')
    status = descriptor.get('status')
    declared = descriptor.get('sha256')
    if not isinstance(path, str) or not path or Path(path).is_absolute() or \
            '..' in Path(path).parts or not isinstance(source_name, str) or \
            not source_name or not isinstance(role, str) or not role or \
            status != 'READY':
        raise LegalCaptureError(f'{label} pinned archive descriptor is malformed')
    declared_sha = _sha_descriptor(declared, f'{label}.sha256')
    matches: list[tuple[Mapping[str, Any], Mapping[str, Any]]] = []
    for source in sources:
        if source.get('name') != source_name:
            continue
        artifacts = source.get('archive_artifacts')
        if not isinstance(artifacts, list):
            continue
        for artifact in artifacts:
            if not isinstance(artifact, Mapping):
                continue
            if artifact.get('path') == path and artifact.get('role') == role:
                matches.append((source, artifact))
    if len(matches) != 1:
        raise LegalCaptureError(
            f'{label} does not have one matching source archive_artifact')
    source, artifact = matches[0]
    artifact_fields = {'role', 'path', 'hash_kind', 'sha256', 'status'}
    if set(artifact) != artifact_fields or \
            artifact.get('hash_kind') != 'upstream_archive_file_sha256' or \
            artifact.get('status') != 'READY' or \
            artifact.get('sha256') != declared_sha:
        raise LegalCaptureError(f'{label} source archive binding drifted')
    source_archive_sha = source.get('archive_sha256')
    source_archive_url = source.get('archive_url')
    source_revision = source.get('revision')
    if not isinstance(source_archive_sha, str) or \
            SHA_RE.fullmatch(source_archive_sha) is None or \
            not isinstance(source_archive_url, str) or \
            not isinstance(source_revision, str) or \
            REVISION_RE.fullmatch(source_revision) is None:
        raise LegalCaptureError(f'{label} source archive identity is malformed')
    return {
        'path': path,
        'path_kind': PINNED_ARCHIVE_PATH_KIND,
        'hash_kind': 'upstream_archive_file_sha256',
        'declared_sha256': declared_sha,
        'actual_sha256': None,
        'status': PINNED_ARCHIVE_STATUS,
        'role': role,
        'source': source_name,
        'source_revision': source_revision,
        'source_archive_url': source_archive_url,
        'source_archive_sha256': source_archive_sha,
        'observation': 'NOT_LOCALLY_OBSERVED',
    }


def _repository_parts(url: Any, label: str) -> tuple[str, str]:
    if not isinstance(url, str):
        raise LegalCaptureError(f'{label} must be an official HTTPS repository URL')
    match = GITHUB_REPOSITORY_RE.fullmatch(url)
    if match is None:
        raise LegalCaptureError(f'{label} is not an official GitHub repository URL')
    return match.group('owner'), match.group('repo')


def validate_official_url(url: Any, kind: str) -> None:
    """Reject non-primary URLs, redirects-as-identities, and URL ambiguity."""
    if not isinstance(url, str) or len(url.encode('utf-8')) > 2048:
        raise LegalCaptureError('official URL is missing or too long')
    parsed = urlsplit(url)
    if parsed.scheme != 'https' or parsed.username or parsed.password or \
            parsed.port is not None or parsed.fragment:
        raise LegalCaptureError('official URL must be HTTPS without credentials/fragment')
    if parsed.hostname not in REMOTE_HOSTS or parsed.hostname != parsed.netloc:
        raise LegalCaptureError('official URL host is not allowlisted')
    if kind == 'repository':
        if GITHUB_REPOSITORY_RE.fullmatch(url) is None:
            raise LegalCaptureError('repository URL is not canonical')
        return
    if kind == 'archive':
        if ARCHIVE_RE.fullmatch(url) is None:
            raise LegalCaptureError('archive URL is not an exact commit archive')
        return
    if kind == 'commit':
        if COMMIT_URL_RE.fullmatch(url) is None:
            raise LegalCaptureError('commit URL is not canonical')
        return
    if kind == 'raw':
        parts = parsed.path.split('/')
        if len(parts) < 4 or not REVISION_RE.fullmatch(parts[3]):
            raise LegalCaptureError('raw URL is not commit-addressed')
        if parsed.query:
            raise LegalCaptureError('raw URL must not contain a query')
        return
    if kind == 'tree_api':
        if parsed.hostname != 'api.github.com' or \
                not re.fullmatch(r'/repos/[^/]+/[^/]+/git/trees/[0-9a-f]{40}',
                                 parsed.path) or parsed.query != 'recursive=1':
            raise LegalCaptureError('tree API URL is not exact')
        return
    raise LegalCaptureError(f'unknown official URL kind: {kind}')


def validate_remote_observation(request: Mapping[str, Any],
                                observation: Mapping[str, Any]) -> None:
    """Validate a future response without treating it as legal approval."""
    request_fields = {'kind', 'url', 'expected_sha256'}
    observation_fields = {
        'kind', 'url', 'expected_sha256', 'response_status', 'final_url',
        'http_status', 'redirect_count', 'body_size', 'body_sha256',
        'headers', 'reason',
    }
    if set(request) != request_fields or set(observation) != observation_fields:
        raise LegalCaptureError('remote observation field set is not exact')
    validate_official_url(request['url'], request['kind'])
    for value, label in ((request['kind'], 'request.kind'),
                         (request['url'], 'request.url')):
        if not isinstance(value, str) or not value:
            raise LegalCaptureError(f'{label} is malformed')
    expected = request['expected_sha256']
    if expected is not None:
        _sha_descriptor(expected, 'request.expected_sha256')
    if observation['kind'] != request['kind'] or observation['url'] != request['url'] or \
            observation['expected_sha256'] != expected:
        raise LegalCaptureError('remote request binding drifted')
    status = observation['response_status']
    if status not in {'NOT_RUN', 'OBSERVED', 'PENDING'}:
        raise LegalCaptureError('remote response status is invalid')
    if isinstance(observation['redirect_count'], bool) or \
            not isinstance(observation['redirect_count'], int) or \
            observation['redirect_count'] < 0:
        raise LegalCaptureError('remote redirect count is invalid')
    if not isinstance(observation['headers'], Mapping):
        raise LegalCaptureError('remote headers must be a mapping')
    for key, value in observation['headers'].items():
        if not isinstance(key, str) or key != key.lower() or key not in SAFE_RESPONSE_HEADERS or \
                not isinstance(value, str) or len(value.encode('utf-8')) > 256 or \
                any(ord(char) < 32 or ord(char) == 127 for char in value):
            raise LegalCaptureError('remote headers contain an unsafe/unknown field')
    if status == 'NOT_RUN':
        if observation['http_status'] is not None or \
                observation['final_url'] is not None or observation['body_size'] is not None or \
                observation['body_sha256'] is not None or observation['headers'] != {} or \
                observation['redirect_count'] != 0 or \
                not isinstance(observation['reason'], str) or \
                not observation['reason']:
            raise LegalCaptureError('NOT_RUN response has observed fields')
        return
    if status == 'OBSERVED':
        if observation['final_url'] != request['url'] or \
                observation['redirect_count'] != 0:
            raise LegalCaptureError('redirected response cannot be an identity')
        if isinstance(observation['http_status'], bool) or \
                not isinstance(observation['http_status'], int) or \
                observation['http_status'] != 200 or \
                observation['body_size'] is None or isinstance(observation['body_size'], bool) or \
                not isinstance(observation['body_size'], int) or \
                observation['body_size'] < 0 or \
                observation['body_size'] > MAX_REMOTE_RESPONSE_BYTES or \
                not isinstance(observation['body_sha256'], str) or \
                SHA_RE.fullmatch(observation['body_sha256']) is None or \
                observation['reason'] is not None:
            raise LegalCaptureError('observed remote response is malformed')
        if expected is not None and observation['body_sha256'] != expected:
            raise LegalCaptureError('observed remote bytes differ from declaration')
    else:
        if observation['http_status'] is not None or \
                observation['final_url'] is not None or \
                observation['body_size'] is not None or \
                observation['body_sha256'] is not None or \
                observation['headers'] != {} or \
                observation['redirect_count'] != 0 or \
                observation['reason'] is None or \
                not isinstance(observation['reason'], str) or \
                not observation['reason']:
            raise LegalCaptureError('pending remote response needs a reason')


def _remote_request(kind: str, url: str, expected_sha256: str | None) -> dict[str, Any]:
    validate_official_url(url, kind)
    if expected_sha256 is not None:
        _sha_descriptor(expected_sha256, 'remote request expected SHA-256')
    request = {'kind': kind, 'url': url,
               'expected_sha256': expected_sha256.lower()
               if isinstance(expected_sha256, str) else None}
    observation = {
        **request, 'response_status': 'NOT_RUN', 'final_url': None,
        'http_status': None,
        'redirect_count': 0, 'body_size': None, 'body_sha256': None,
        'headers': {}, 'reason': 'network_not_used_in_capture',
    }
    validate_remote_observation(request, observation)
    return observation


def _license_paths(source: Mapping[str, Any]) -> list[tuple[str, str, str]]:
    license_data = source.get('license')
    if not isinstance(license_data, Mapping):
        return []
    values: list[tuple[str, str, str]] = []
    seen: set[tuple[str, str]] = set()
    for key, entries in (('license_file', license_data.get('files')),):
        if isinstance(entries, list):
            for item in entries:
                if isinstance(item, Mapping) and isinstance(item.get('path'), str):
                    pair = (key, item['path'])
                    if pair not in seen:
                        seen.add(pair)
                        values.append((key, item['path'], item.get('sha256')))
    components = license_data.get('components')
    if isinstance(components, list):
        for component in components:
            if not isinstance(component, Mapping):
                continue
            for key, entries in (('license_file', component.get('files')),
                                 ('license_declaration', component.get('declarations'))):
                if not isinstance(entries, list):
                    continue
                for item in entries:
                    if isinstance(item, Mapping) and isinstance(item.get('path'), str):
                        pair = (key, item['path'])
                        if pair not in seen:
                            seen.add(pair)
                            values.append((key, item['path'], item.get('sha256')))
    return values


def _source_observation(source: Mapping[str, Any], label: str) -> dict[str, Any]:
    name = source.get('name')
    repository = source.get('repository_url')
    revision = source.get('revision')
    archive = source.get('archive_url')
    citation = source.get('citation')
    if not isinstance(name, str) or not name or not isinstance(revision, str) or \
            REVISION_RE.fullmatch(revision) is None:
        raise LegalCaptureError(f'{label} source identity is malformed')
    validate_official_url(repository, 'repository')
    validate_official_url(archive, 'archive')
    if not isinstance(citation, Mapping):
        raise LegalCaptureError(f'{label}.citation is missing')
    commit_url = citation.get('commit_url')
    validate_official_url(commit_url, 'commit')
    owner, repo = _repository_parts(repository, f'{label}.repository_url')
    archive_match = ARCHIVE_RE.fullmatch(archive)
    commit_match = COMMIT_URL_RE.fullmatch(commit_url)
    if archive_match is None or commit_match is None or \
            archive_match.group('revision').lower() != revision.lower() or \
            commit_match.group('revision').lower() != revision.lower():
        raise LegalCaptureError(f'{label} URL/revision binding is inconsistent')
    requests = [
        _remote_request('repository', repository, None),
        _remote_request('commit', commit_url, None),
        _remote_request('archive', archive, source.get('archive_sha256')),
    ]
    for kind, relative, expected in _license_paths(source):
        relative_path = Path(relative)
        if relative_path.is_absolute() or '..' in relative_path.parts:
            raise LegalCaptureError(f'{label} license path traverses the source tree')
        raw_url = (
            f'https://raw.githubusercontent.com/{owner}/{repo}/{revision}/'
            f'{quote(relative_path.as_posix(), safe="/")}'
        )
        requests.append(_remote_request('raw', raw_url, expected
                                        if isinstance(expected, str) else None))
    if not _license_paths(source):
        tree_url = (
            f'https://api.github.com/repos/{owner}/{repo}/git/trees/'
            f'{revision}?recursive=1'
        )
        requests.append(_remote_request('tree_api', tree_url, None))
    license_data = source.get('license')
    license_status = license_data.get('status') if isinstance(license_data, Mapping) else None
    reasons = license_data.get('not_ready_reasons', []) \
        if isinstance(license_data, Mapping) else []
    return {
        'name': name,
        'status': source.get('status'),
        'repository_url': repository,
        'revision': revision,
        'revision_kind': source.get('revision_kind'),
        'archive_url': archive,
        'archive_sha256': source.get('archive_sha256'),
        'source_tree_sha256': source.get('source_tree_sha256'),
        'source_tree_hash_kind': source.get('source_tree_hash_kind'),
        'license_status': license_status,
        'license_not_ready_reasons': list(reasons)
        if isinstance(reasons, list) else [],
        'license_paths': [
            {'kind': kind, 'path': path, 'sha256': expected}
            for kind, path, expected in _license_paths(source)
        ],
        'citation': {
            'primary_upstream': citation.get('primary_upstream'),
            'commit_url': commit_url,
            'compatibility_status': citation.get('compatibility_status'),
        },
        'remote_requests': requests,
    }


def _flatten_sources(source: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    result = [source]
    submodules = source.get('submodules')
    if isinstance(submodules, list):
        for child in submodules:
            if isinstance(child, Mapping):
                result.extend(_flatten_sources(child))
    return result


def _recipe_observation(root: Path, recipe: Any, label: str,
                        sources: list[Mapping[str, Any]]) -> dict[str, Any]:
    if not isinstance(recipe, Mapping):
        return {'status': 'INVALID', 'artifacts': [],
                'base_image': None, 'reason': f'{label} is missing'}
    artifacts: list[dict[str, Any]] = []
    valid = True
    for role in RECIPE_ARTIFACT_ROLES:
        if role in recipe:
            item = _artifact_observation(root, recipe[role], f'{label}.{role}')
            item['role'] = role
            artifacts.append(item)
            valid = valid and item['status'] == 'MATCH'
        else:
            valid = False
            artifacts.append({'role': role, 'path': '', 'hash_kind': 'missing',
                              'declared_sha256': None, 'actual_sha256': None,
                              'status': 'MISSING'})
    configs = recipe.get('configs')
    if isinstance(configs, list):
        for index, item in enumerate(configs):
            item_label = f'{label}.configs[{index}]'
            if isinstance(item, Mapping) and 'path_kind' in item:
                if item.get('path_kind') != PINNED_ARCHIVE_PATH_KIND:
                    raise LegalCaptureError(
                        f'{item_label} has an unknown path_kind')
                observed = _pinned_archive_observation(item, sources, item_label)
            else:
                observed = _artifact_observation(root, item, item_label)
            observed['role'] = f'config[{index}]'
            artifacts.append(observed)
            valid = valid and observed['status'] in {
                'MATCH', PINNED_ARCHIVE_STATUS}
    else:
        valid = False
    base_image = recipe.get('base_image')
    if not isinstance(base_image, Mapping) or not isinstance(base_image.get('name'), str) or \
            not isinstance(base_image.get('digest'), str):
        base_projection = None
        valid = False
    else:
        base_projection = {'name': base_image['name'], 'digest': base_image['digest']}
    return {
        'status': 'CURRENT_BYTES_MATCH_DECLARATIONS' if valid else 'DRIFT_OR_MISSING',
        'artifacts': artifacts,
        'base_image': base_projection,
    }


def _system_observation(root: Path, system: str, rival: Mapping[str, Any]) -> dict[str, Any]:
    sources = rival.get('sources')
    if not isinstance(sources, list) or not sources:
        raise LegalCaptureError(f'rival {system} source list is missing')
    source_rows: list[dict[str, Any]] = []
    for source in sources:
        if not isinstance(source, Mapping):
            raise LegalCaptureError(f'rival {system} source is not a mapping')
        source_rows.extend(_source_observation(item, f'{system}.source')
                           for item in _flatten_sources(source))
    recipe = _recipe_observation(
        root, rival.get('recipe'), f'{system}.recipe',
        [item for source in sources for item in _flatten_sources(source)])
    legal_blocked = any(row['license_status'] != 'READY' for row in source_rows)
    return {
        'status': 'NOT_READY_LEGAL_PROVENANCE' if legal_blocked else 'REVIEW_REQUIRED',
        'source_count': len(source_rows),
        'sources': source_rows,
        'recipe': recipe,
    }


def _profile_binding(root: Path, profile_path: Path | str,
                     profile: Mapping[str, Any]) -> dict[str, Any]:
    path, relative = _profile_path(root, profile_path)
    payload = _read_repository_file(root, relative, 'profile')
    try:
        parsed = yaml.safe_load(payload.decode('utf-8'))
    except (UnicodeError, yaml.YAMLError) as exc:
        raise LegalCaptureError(f'profile is not valid YAML: {exc}') from exc
    if not isinstance(parsed, Mapping) or dict(parsed) != dict(profile):
        raise LegalCaptureError('profile changed during capture')
    return {'path': relative, 'sha256': _sha(payload),
            'size': len(payload)}


def _checker_projection(result: Mapping[str, Any], source_sha: str) -> dict[str, Any]:
    return {
        'status': result.get('status'), 'pass': result.get('pass') is True,
        'errors': list(result.get('errors', [])),
        'incomplete': list(result.get('incomplete', [])),
        'not_ready': list(result.get('not_ready', [])),
        'checker_source_sha256': source_sha,
    }


def build_capture_document(*, root: Path, profile_path: Path | str,
                           observed_at: str | None = None) -> dict[str, Any]:
    root = _require_absolute(root, 'repository root')
    _directory_identity(root, 'repository root')
    profile_file, profile_relative = _profile_path(root, profile_path)
    payload = _read_repository_file(root, profile_relative, 'profile')
    try:
        profile = yaml.safe_load(payload.decode('utf-8'))
    except (UnicodeError, yaml.YAMLError) as exc:
        raise LegalCaptureError(f'profile is not valid YAML: {exc}') from exc
    if not isinstance(profile, Mapping):
        raise LegalCaptureError('profile must be a mapping')
    contract = profile.get('competitive_slam_profile', profile)
    if not isinstance(contract, Mapping):
        raise LegalCaptureError('competitive profile must be a mapping')
    policy = contract.get('evidence_gate_v2', {}).get('rival_source_closure') \
        if isinstance(contract.get('evidence_gate_v2'), Mapping) else None
    if not isinstance(policy, Mapping):
        raise LegalCaptureError('rival source closure is missing')
    try:
        identity = current_rival_source_closure_identity(profile, root=root)
    except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError) as exc:
        raise LegalCaptureError(f'current closure identity is invalid: {exc}') from exc
    selection_path = identity['selection_path']
    selection_payload = _read_repository_file(root, selection_path, 'r2 selection')
    closure_result = verify_rival_source_closure(profile, root=root, receipt=None)
    checker_path = Path(__file__).resolve()
    checker_payload = checker_path.read_bytes()
    systems = policy.get('rivals')
    if not isinstance(systems, Mapping) or set(systems) != set(SYSTEMS):
        raise LegalCaptureError('rival source closure must cover exactly glim/fast_livo2')
    system_rows = {
        system: _system_observation(root, system, systems[system])
        for system in SYSTEMS
    }
    checker = _checker_projection(closure_result, _sha(checker_payload))
    blockers = {
        'unsigned_external_review_required',
        'remote_upstream_verification_not_run',
        'benchmark_and_claim_promotion_forbidden',
    }
    blockers.update(str(item) for item in checker['errors'])
    blockers.update(str(item) for item in checker['incomplete'])
    blockers.update(str(item) for item in checker['not_ready'])
    document: dict[str, Any] = {
        'schema_version': CAPTURE_SCHEMA_VERSION,
        'capture_kind': CAPTURE_KIND,
        'status': 'REVIEW_REQUIRED',
        'review_status': 'UNSIGNED_REVIEW_REQUIRED',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'captured_at': _timestamp(observed_at),
        'closure_binding': {
            'closure_id': identity['closure_id'],
            'closure_revision': identity['closure_revision'],
            'closure_identity_hash_kind': CLOSURE_IDENTITY_HASH_KIND,
            'closure_identity_sha256': identity['closure_identity_sha256'],
            'selection_id': identity['selection_id'],
            'selection_path': identity['selection_path'],
            'selection_sha256': identity['selection_sha256'],
            'selection_file_size': len(selection_payload),
        },
        'profile_binding': _profile_binding(root, profile_path, profile),
        'checker': checker,
        'systems': system_rows,
        'remote_policy': {
            'allowed_hosts': sorted(REMOTE_HOSTS),
            'require_https': True,
            'require_exact_commit_urls': True,
            'redirects_allowed': False,
            'max_response_bytes': MAX_REMOTE_RESPONSE_BYTES,
            'network_used': False,
            'responses_status': 'NOT_RUN',
        },
        'source_policy': {
            'official_source_policy': policy.get('official_source_policy'),
            'archive_hash_kind': policy.get('archive_hash_kind'),
            'source_tree_hash_kind': policy.get('source_tree_hash_kind'),
            'legal_policy': copy.deepcopy(policy.get('legal_policy')),
        },
        'producer': {
            'path': 'scripts/capture_competitive_rival_legal_provenance.py',
            'sha256': _sha(checker_payload),
        },
        'blockers': sorted(blockers),
    }
    document['capture_identity_sha256'] = _identity(
        document, 'capture_identity_sha256')
    return document


def _write_pair(directory: Path, document: Mapping[str, Any]) -> list[tuple[Path, dict[str, int]]]:
    payload = (json.dumps(document, indent=2, sort_keys=True, ensure_ascii=True) +
               '\n').encode('utf-8')
    if len(payload) > MAX_CAPTURE_BYTES:
        raise LegalCaptureError('capture document exceeds bounded size')
    path = directory / CAPTURE_FILE
    identity = _exclusive_write(path, payload)
    files = [(path, identity)]
    try:
        sidecar = directory / CAPTURE_SIDECAR
        side_identity = _exclusive_write(
            sidecar, (_sha(payload) + '  ' + CAPTURE_FILE + '\n').encode())
        files.append((sidecar, side_identity))
    except Exception:
        _remove_owned(path, identity, 'legal capture pair rollback')
        raise
    return files


def _read_pair(directory: Path) -> tuple[dict[str, Any], str]:
    path = directory / CAPTURE_FILE
    try:
        payload, _ = _read_immutable(path, 'legal capture', max_bytes=MAX_CAPTURE_BYTES)
        sidecar, _ = _read_immutable(path.with_name(CAPTURE_SIDECAR),
                                     'legal capture sidecar', max_bytes=256)
    except CandidateError as exc:
        raise LegalCaptureError(str(exc)) from exc
    expected = (_sha(payload) + '  ' + CAPTURE_FILE + '\n').encode()
    if sidecar != expected:
        raise LegalCaptureError('legal capture sidecar is stale')
    try:
        final_payload, _ = _read_immutable(path, 'legal capture',
                                           max_bytes=MAX_CAPTURE_BYTES)
        final_sidecar, _ = _read_immutable(
            path.with_name(CAPTURE_SIDECAR), 'legal capture sidecar',
            max_bytes=256)
    except CandidateError as exc:
        raise LegalCaptureError(str(exc)) from exc
    if final_payload != payload or final_sidecar != sidecar:
        raise LegalCaptureError('legal capture changed during validation')
    try:
        document = json.loads(payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise LegalCaptureError(f'legal capture JSON is invalid: {exc}') from exc
    if not isinstance(document, dict):
        raise LegalCaptureError('legal capture must be an object')
    return document, _sha(payload)


def _validate_document_shape(document: Mapping[str, Any]) -> None:
    required = {
        'schema_version', 'capture_kind', 'status', 'review_status',
        'benchmark_eligible', 'claim_eligible', 'captured_at',
        'closure_binding', 'profile_binding', 'checker', 'systems',
        'remote_policy', 'source_policy', 'producer', 'blockers',
        'capture_identity_sha256',
    }
    if set(document) != required:
        raise LegalCaptureError('legal capture field set is not exact')
    if document['schema_version'] != CAPTURE_SCHEMA_VERSION or \
            document['capture_kind'] != CAPTURE_KIND or \
            document['status'] != 'REVIEW_REQUIRED' or \
            document['review_status'] != 'UNSIGNED_REVIEW_REQUIRED' or \
            document['benchmark_eligible'] is not False or \
            document['claim_eligible'] is not False:
        raise LegalCaptureError('legal capture is promotable or has invalid status')
    if not isinstance(document['captured_at'], str) or \
            ISO_UTC_RE.fullmatch(document['captured_at']) is None:
        raise LegalCaptureError('legal capture timestamp is invalid')
    if not isinstance(document['systems'], Mapping) or \
            set(document['systems']) != set(SYSTEMS):
        raise LegalCaptureError('legal capture rival coverage is not exact')
    if not isinstance(document['blockers'], list) or not document['blockers'] or \
            any(not isinstance(item, str) or not item for item in document['blockers']):
        raise LegalCaptureError('legal capture blockers are missing')
    policy = document['remote_policy']
    if policy != {
            'allowed_hosts': sorted(REMOTE_HOSTS), 'require_https': True,
            'require_exact_commit_urls': True, 'redirects_allowed': False,
            'max_response_bytes': MAX_REMOTE_RESPONSE_BYTES,
            'network_used': False, 'responses_status': 'NOT_RUN'}:
        raise LegalCaptureError('remote policy is not exact')
    if _identity(document, 'capture_identity_sha256') != \
            document['capture_identity_sha256']:
        raise LegalCaptureError('legal capture identity mismatch')
    for system, value in document['systems'].items():
        if not isinstance(value, Mapping) or set(value) != {
                'status', 'source_count', 'sources', 'recipe'}:
            raise LegalCaptureError(f'{system} legal projection shape is invalid')
        if value['status'] not in {'NOT_READY_LEGAL_PROVENANCE', 'REVIEW_REQUIRED'}:
            raise LegalCaptureError(f'{system} legal projection may not be READY')
        if not isinstance(value['source_count'], int) or value['source_count'] < 1 or \
                not isinstance(value['sources'], list) or \
                len(value['sources']) != value['source_count']:
            raise LegalCaptureError(f'{system} source coverage is invalid')
        if not isinstance(value['recipe'], Mapping) or \
                set(value['recipe']) != {'status', 'artifacts', 'base_image'} or \
                value['recipe']['status'] not in {
                    'CURRENT_BYTES_MATCH_DECLARATIONS', 'DRIFT_OR_MISSING', 'INVALID'}:
            raise LegalCaptureError(f'{system} recipe projection is invalid')
        artifacts = value['recipe']['artifacts']
        if not isinstance(artifacts, list) or not artifacts:
            raise LegalCaptureError(f'{system} recipe artifact coverage is invalid')
        for artifact in artifacts:
            _validate_recipe_artifact(artifact, system)
        for source in value['sources']:
            _validate_source_projection(source, system)


def _validate_recipe_artifact(artifact: Any, system: str) -> None:
    if not isinstance(artifact, Mapping) or \
            not isinstance(artifact.get('role'), str) or \
            not isinstance(artifact.get('path'), str) or \
            not isinstance(artifact.get('hash_kind'), str) or \
            artifact.get('actual_sha256') is not None and \
            not isinstance(artifact.get('actual_sha256'), str) or \
            artifact.get('declared_sha256') is not None and \
            not isinstance(artifact.get('declared_sha256'), str):
        raise LegalCaptureError(f'{system} recipe artifact is malformed')
    status = artifact.get('status')
    if status == PINNED_ARCHIVE_STATUS:
        required = {
            'path', 'path_kind', 'hash_kind', 'declared_sha256',
            'actual_sha256', 'status', 'role', 'source', 'source_revision',
            'source_archive_url', 'source_archive_sha256', 'observation'}
        if set(artifact) != required or \
                artifact.get('path_kind') != PINNED_ARCHIVE_PATH_KIND or \
                artifact.get('hash_kind') != 'upstream_archive_file_sha256' or \
                artifact.get('actual_sha256') is not None or \
                artifact.get('observation') != 'NOT_LOCALLY_OBSERVED' or \
                not isinstance(artifact.get('source'), str) or \
                not isinstance(artifact.get('source_revision'), str) or \
                REVISION_RE.fullmatch(artifact['source_revision']) is None or \
                not isinstance(artifact.get('source_archive_url'), str) or \
                not isinstance(artifact.get('source_archive_sha256'), str):
            raise LegalCaptureError(f'{system} pinned recipe artifact is invalid')
        validate_official_url(artifact['source_archive_url'], 'archive')
        _sha_descriptor(artifact['declared_sha256'], 'pinned artifact SHA')
        _sha_descriptor(artifact['source_archive_sha256'], 'source archive SHA')
        if Path(artifact['path']).is_absolute() or \
                '..' in Path(artifact['path']).parts:
            raise LegalCaptureError(f'{system} pinned artifact path escapes source')
        return
    allowed = {
        'role', 'path', 'hash_kind', 'declared_sha256', 'actual_sha256',
        'status', 'reason'}
    if set(artifact) - allowed or status not in {
            'MATCH', 'DRIFT', 'MISSING', 'INVALID_PATH'}:
        raise LegalCaptureError(f'{system} local recipe artifact is invalid')
    for field in ('declared_sha256', 'actual_sha256'):
        value = artifact.get(field)
        if value is not None:
            _sha_descriptor(value, f'{system} recipe {field}')
    if 'reason' in artifact and (not isinstance(artifact['reason'], str) or
                                 not artifact['reason']):
        raise LegalCaptureError(f'{system} recipe artifact reason is invalid')


def _validate_source_projection(source: Any, system: str) -> None:
    if not isinstance(source, Mapping) or set(source) != {
            'name', 'status', 'repository_url', 'revision', 'revision_kind',
            'archive_url', 'archive_sha256', 'source_tree_sha256',
            'source_tree_hash_kind', 'license_status',
            'license_not_ready_reasons', 'license_paths', 'citation',
            'remote_requests'}:
        raise LegalCaptureError(f'{system} source projection is not exact')
    validate_official_url(source['repository_url'], 'repository')
    validate_official_url(source['archive_url'], 'archive')
    if not isinstance(source['revision'], str) or \
            REVISION_RE.fullmatch(source['revision']) is None or \
            source['revision_kind'] != 'immutable_commit':
        raise LegalCaptureError(f'{system} source revision is invalid')
    if not isinstance(source['license_not_ready_reasons'], list):
        raise LegalCaptureError(f'{system} license reason shape is invalid')
    if source['license_status'] not in {
            'READY', 'NOT_READY', 'NOT_READY_LEGAL_PROVENANCE', None}:
        raise LegalCaptureError(f'{system} license status is invalid')
    if not isinstance(source['license_paths'], list):
        raise LegalCaptureError(f'{system} license path projection is invalid')
    citation = source['citation']
    if not isinstance(citation, Mapping) or set(citation) != {
            'primary_upstream', 'commit_url', 'compatibility_status'} or \
            citation['primary_upstream'] is not True:
        raise LegalCaptureError(f'{system} citation projection is invalid')
    validate_official_url(citation['commit_url'], 'commit')
    if not isinstance(source['remote_requests'], list) or not source['remote_requests']:
        raise LegalCaptureError(f'{system} remote request coverage is missing')
    for request in source['remote_requests']:
        validate_remote_observation(
            {key: request[key] for key in ('kind', 'url', 'expected_sha256')},
            request)


def validate_capture_set(*, output_dir: Path, root: Path = ROOT,
                         profile_path: Path | str = PROFILE_REL) -> dict[str, Any]:
    output_dir = _require_absolute(output_dir, 'legal capture output')
    _directory_identity(output_dir, 'legal capture output')
    names = {item.name for item in output_dir.iterdir()}
    if names != {CAPTURE_FILE, CAPTURE_SIDECAR}:
        raise LegalCaptureError('legal capture output has missing or extra files')
    document, payload_sha = _read_pair(output_dir)
    _validate_document_shape(document)
    expected = build_capture_document(
        root=root, profile_path=profile_path,
        observed_at=document['captured_at'])
    expected_identity = expected.pop('capture_identity_sha256')
    observed_identity = document.pop('capture_identity_sha256')
    if expected != document or expected_identity != observed_identity:
        raise LegalCaptureError('current closure/profile/recipe bytes drifted')
    return {
        'status': document['status'],
        'review_status': document['review_status'],
        'benchmark_eligible': False,
        'claim_eligible': False,
        'capture_sha256': payload_sha,
        'closure_identity_sha256': document['closure_binding'][
            'closure_identity_sha256'],
    }


def capture_legal_provenance(*, output_dir: Path, root: Path = ROOT,
                             profile_path: Path | str = PROFILE_REL,
                             observed_at: str | None = None) -> dict[str, Any]:
    output_dir = _require_absolute(output_dir, 'legal capture output')
    if output_dir.exists() or output_dir.is_symlink():
        raise LegalCaptureError('legal capture output must be fresh and absent')
    _require_absolute(root, 'repository root')
    _directory_identity(root, 'repository root')
    parent = output_dir.parent
    _require_absolute(parent, 'legal capture parent')
    _assert_no_symlink_ancestors(parent, 'legal capture parent')
    parent.mkdir(parents=True, exist_ok=True)
    if parent.is_symlink():
        raise LegalCaptureError('legal capture parent is a symlink')
    document = build_capture_document(
        root=root, profile_path=profile_path, observed_at=observed_at)
    try:
        output_dir.mkdir(mode=0o700)
        _directory_identity(output_dir, 'legal capture output')
        owned = _write_pair(output_dir, document)
    except Exception:
        if output_dir.is_dir() and not output_dir.is_symlink():
            for path, identity in locals().get('owned', []):
                try:
                    _remove_owned(path, identity, 'legal capture rollback')
                except CandidateError:
                    pass
        raise
    validate_capture_set(output_dir=output_dir, root=root, profile_path=profile_path)
    return document


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    capture = sub.add_parser('capture')
    capture.add_argument('--repo-root', type=Path, default=ROOT)
    capture.add_argument('--profile', type=Path,
                         default=ROOT / PROFILE_REL)
    capture.add_argument('--output-dir', type=Path, required=True)
    capture.add_argument('--observed-at', default=None)
    validate = sub.add_parser('validate')
    validate.add_argument('--repo-root', type=Path, default=ROOT)
    validate.add_argument('--profile', type=Path,
                          default=ROOT / PROFILE_REL)
    validate.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'capture':
        result = capture_legal_provenance(
            output_dir=args.output_dir, root=args.repo_root,
            profile_path=args.profile, observed_at=args.observed_at)
    else:
        result = validate_capture_set(
            output_dir=args.output_dir, root=args.repo_root,
            profile_path=args.profile)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(_main())
    except (OSError, UnicodeError, ValueError, TypeError, yaml.YAMLError,
            json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc
