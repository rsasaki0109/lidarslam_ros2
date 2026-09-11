#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Prepare two fail-closed usability scorecard worksheets as one pair.

Common cohort, input, and environment metadata is entered once. The command
creates one worksheet per product with opposite product order, refuses to
overwrite either destination, and can verify both public identities and docs
through bounded GET requests before writing. The generated files are recording
worksheets, not evidence of a completed trial.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import jsonschema

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from prepare_usability_scorecard import (  # noqa: E402
    ScorecardError,
    build_template,
    validate_trial,
)


PRODUCTS = (
    ('lidarslam', 'lidarslam_ros2'),
    ('glim', 'glim'),
)
REPO_ROOT = SCRIPT_DIR.parent
PREPARATION_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'usability-scorecard-pair-preparation-v1.schema.json'
)
PREPARATION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'usability-scorecard-pair-preparation-v1.schema.json'
)
PREPARATION_RECEIPT_NAME = (
    'usability-scorecard-pair-preparation-v1.json'
)
GITHUB_REPOSITORIES = {
    'lidarslam_ros2': 'rsasaki0109/lidar_slam_ros2',
    'glim': 'koide3/glim',
}
REGISTRIES = {
    'lidarslam_ros2': {
        'source': 'ghcr.io/rsasaki0109/lidar_slam_ros2',
        'token_url': (
            'https://ghcr.io/token?service=ghcr.io&scope='
            'repository%3Arsasaki0109%2Flidar_slam_ros2%3Apull'
        ),
        'manifest_root': (
            'https://ghcr.io/v2/rsasaki0109/lidar_slam_ros2/manifests/'
        ),
    },
    'glim': {
        'source': 'docker.io/koide3/glim_ros2',
        'token_url': (
            'https://auth.docker.io/token?service=registry.docker.io&scope='
            'repository%3Akoide3%2Fglim_ros2%3Apull'
        ),
        'manifest_root': (
            'https://registry-1.docker.io/v2/koide3/glim_ros2/manifests/'
        ),
    },
}
ALLOWED_DOCUMENTATION_HOSTS = {
    'lidarslam_ros2': {'github.com', 'rsasaki0109.github.io'},
    'glim': {'github.com', 'koide3.github.io'},
}
SHA_PATTERN = re.compile(r'^[0-9a-f]{40}$')
DIGEST_PATTERN = re.compile(r'^sha256:[0-9a-f]{64}$')
MAX_JSON_BYTES = 2 * 1024 * 1024
MAX_DOCUMENT_BYTES = 2 * 1024 * 1024
MAX_MANIFEST_BYTES = 4 * 1024 * 1024
OCI_ACCEPT = ', '.join((
    'application/vnd.oci.image.index.v1+json',
    'application/vnd.oci.image.manifest.v1+json',
    'application/vnd.docker.distribution.manifest.list.v2+json',
    'application/vnd.docker.distribution.manifest.v2+json',
))
HttpGet = Callable[
    [str, Mapping[str, str], int],
    tuple[int, bytes, Mapping[str, str], str],
]


def _http_get(
    url: str,
    headers: Mapping[str, str],
    limit: int,
) -> tuple[int, bytes, Mapping[str, str], str]:
    """Perform one size-bounded GET without persisting its response body."""
    request = urllib.request.Request(
        url,
        headers=dict(headers),
        method='GET',
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            status = response.status
            payload = response.read(limit + 1)
            response_headers = dict(response.headers.items())
            final_url = response.geturl()
    except urllib.error.HTTPError as exc:
        status = exc.code
        payload = exc.read(limit + 1)
        response_headers = dict(exc.headers.items())
        final_url = exc.geturl()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise ScorecardError(f'public identity GET failed: {exc}') from exc
    if len(payload) > limit:
        raise ScorecardError('public identity GET exceeded its byte limit')
    return status, payload, response_headers, final_url


def _json_document(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ScorecardError(f'{label} returned invalid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise ScorecardError(f'{label} JSON root is not an object')
    return value


def _github_json(
    repository: str,
    path: str,
    http_get: HttpGet,
) -> dict[str, Any]:
    url = f'https://api.github.com/repos/{repository}/{path}'
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-usability-pair-public-preflight/1',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    headers.update(github_api_authorization(url, method='GET'))
    status, payload, _, final_url = http_get(
        url,
        headers,
        MAX_JSON_BYTES,
    )
    if status != 200 or final_url != url:
        raise ScorecardError(
            f'GitHub identity is not readable at its canonical URL: '
            f'HTTP {status}'
        )
    return _json_document(payload, 'GitHub identity')


def _resolve_github_revision(
    product_id: str,
    version: str,
    kind: str,
    revision: str,
    http_get: HttpGet,
) -> tuple[str, str]:
    repository = GITHUB_REPOSITORIES[product_id]
    source = f'github.com/{repository}'
    if kind == 'git-commit':
        if SHA_PATTERN.fullmatch(revision) is None:
            raise ScorecardError('git-commit revision must be a full SHA')
        commit = _github_json(
            repository,
            f'commits/{revision}',
            http_get,
        )
        if commit.get('sha') != revision:
            raise ScorecardError('GitHub commit response changed identity')
        return source, revision
    if kind != 'release-tag':
        raise ScorecardError(f'unsupported GitHub revision kind: {kind}')
    if revision.removeprefix('v') != version.removeprefix('v'):
        raise ScorecardError(
            'release-tag revision differs from product version'
        )
    encoded = urllib.parse.quote(revision, safe='')
    tag_ref = _github_json(
        repository,
        f'git/ref/tags/{encoded}',
        http_get,
    )
    if tag_ref.get('ref') != f'refs/tags/{revision}':
        raise ScorecardError('GitHub tag response changed identity')
    target = tag_ref.get('object')
    for _ in range(3):
        if not isinstance(target, dict):
            raise ScorecardError('GitHub tag target is not an object')
        target_type = target.get('type')
        target_sha = target.get('sha')
        if not isinstance(target_sha, str):
            raise ScorecardError('GitHub tag target has no SHA')
        if target_type == 'commit':
            if SHA_PATTERN.fullmatch(target_sha) is None:
                raise ScorecardError('GitHub tag commit is not a full SHA')
            return source, target_sha
        if target_type != 'tag' or SHA_PATTERN.fullmatch(target_sha) is None:
            raise ScorecardError('GitHub tag target type is unsupported')
        tag = _github_json(
            repository,
            f'git/tags/{target_sha}',
            http_get,
        )
        target = tag.get('object')
    raise ScorecardError('GitHub annotated tag exceeds dereference limit')


def _header(
    headers: Mapping[str, str],
    name: str,
) -> str | None:
    expected = name.lower()
    return next(
        (value for key, value in headers.items() if key.lower() == expected),
        None,
    )


def _resolve_registry_digest(
    product_id: str,
    digest: str,
    http_get: HttpGet,
) -> tuple[str, str]:
    if DIGEST_PATTERN.fullmatch(digest) is None:
        raise ScorecardError('image-digest revision is invalid')
    registry = REGISTRIES[product_id]
    status, payload, _, final_token_url = http_get(
        registry['token_url'],
        {'User-Agent': 'lidarslam-usability-pair-public-preflight/1'},
        MAX_JSON_BYTES,
    )
    if status != 200 or final_token_url != registry['token_url']:
        raise ScorecardError(
            f'public registry token is not readable at its canonical URL: '
            f'HTTP {status}'
        )
    token = _json_document(payload, 'registry token').get('token')
    if not isinstance(token, str) or not token:
        raise ScorecardError('public registry token response has no token')
    manifest_url = registry['manifest_root'] + digest
    status, _, headers, final_url = http_get(
        manifest_url,
        {
            'Accept': OCI_ACCEPT,
            'Authorization': f'Bearer {token}',
            'User-Agent': 'lidarslam-usability-pair-public-preflight/1',
        },
        MAX_MANIFEST_BYTES,
    )
    observed_digest = _header(headers, 'Docker-Content-Digest')
    if (
        status != 200
        or final_url != manifest_url
        or observed_digest != digest
    ):
        raise ScorecardError(
            'public registry manifest does not resolve to the exact digest'
        )
    return registry['source'], digest


def _validate_documentation_boundary(
    product_id: str,
    url: str,
    *,
    redirect: bool = False,
) -> None:
    label = 'documentation redirect' if redirect else 'documentation URL'
    try:
        parsed = urllib.parse.urlparse(url)
        parsed_port = parsed.port
    except ValueError as exc:
        raise ScorecardError(f'{product_id} {label} is malformed') from exc
    if (
        parsed.scheme != 'https'
        or parsed.hostname not in ALLOWED_DOCUMENTATION_HOSTS[product_id]
        or parsed.username is not None
        or parsed.password is not None
        or parsed_port is not None
        or parsed.fragment
    ):
        raise ScorecardError(
            f'{product_id} {label} is outside its public boundary'
        )


def _verify_documentation(
    product_id: str,
    url: str,
    http_get: HttpGet,
) -> tuple[int, str]:
    _validate_documentation_boundary(product_id, url)
    status, _, _, final_url = http_get(
        url,
        {
            'Accept': 'text/html,application/xhtml+xml',
            'User-Agent': 'lidarslam-usability-pair-public-preflight/1',
        },
        MAX_DOCUMENT_BYTES,
    )
    if status != 200:
        raise ScorecardError(
            f'{product_id} documentation is not publicly readable'
        )
    _validate_documentation_boundary(product_id, final_url, redirect=True)
    return status, final_url


def _validate_public_inputs(records: Sequence[dict[str, Any]]) -> None:
    """Reject all local identity errors before performing the first GET."""
    expected_products = ['lidarslam_ros2', 'glim']
    products = [record['product']['id'] for record in records]
    if products != expected_products:
        raise ScorecardError('public identity product order changed')
    for record in records:
        product = record['product']
        product_id = product['id']
        kind = product['revision']['kind']
        revision = product['revision']['value']
        if kind == 'git-commit' and SHA_PATTERN.fullmatch(revision) is None:
            raise ScorecardError('git-commit revision must be a full SHA')
        if kind == 'release-tag' and (
            revision.removeprefix('v')
            != product['version'].removeprefix('v')
        ):
            raise ScorecardError(
                'release-tag revision differs from product version'
            )
        if (
            kind == 'image-digest'
            and DIGEST_PATTERN.fullmatch(revision) is None
        ):
            raise ScorecardError('image-digest revision is invalid')
        _validate_documentation_boundary(
            product_id,
            product['documentation_root_url'],
        )


def verify_public_pair(
    records: Sequence[dict[str, Any]],
    *,
    http_get: HttpGet = _http_get,
) -> dict[str, Any]:
    """Verify both canonical product identities and docs through GETs."""
    _validate_public_inputs(records)
    results = []
    for record in records:
        product = record['product']
        product_id = product['id']
        kind = product['revision']['kind']
        revision = product['revision']['value']
        if kind == 'image-digest':
            source, resolved = _resolve_registry_digest(
                product_id,
                revision,
                http_get,
            )
        else:
            source, resolved = _resolve_github_revision(
                product_id,
                product['version'],
                kind,
                revision,
                http_get,
            )
        docs_status, final_url = _verify_documentation(
            product_id,
            product['documentation_root_url'],
            http_get,
        )
        results.append({
            'product_id': product_id,
            'identity_source': source,
            'revision_kind': kind,
            'requested_revision': revision,
            'resolved_revision': resolved,
            'documentation_url': product['documentation_root_url'],
            'documentation_http_status': docs_status,
            'documentation_final_url': final_url,
        })
    return {
        'performed': True,
        'status': 'PASS',
        'network_reads_performed': True,
        'results': results,
    }


def _now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _add_product_arguments(
    parser: argparse.ArgumentParser,
    prefix: str,
    product: str,
) -> None:
    """Add the identity and documentation options for one product."""
    title = product.replace('_', ' ')
    parser.add_argument(
        f'--{prefix}-version',
        required=True,
        help=f'{title} product version',
    )
    parser.add_argument(
        f'--{prefix}-revision-kind',
        choices=('git-commit', 'release-tag', 'image-digest'),
        required=True,
        help=f'{title} revision identity kind',
    )
    parser.add_argument(
        f'--{prefix}-revision',
        required=True,
        help=f'{title} exact revision value',
    )
    parser.add_argument(
        f'--{prefix}-documentation-url',
        required=True,
        help=f'{title} public documentation root URL',
    )
    parser.add_argument(f'--{prefix}-trial-id')
    parser.add_argument(
        f'--{prefix}-publicly-resolvable',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        f'--{prefix}-machine-fingerprint-sha256',
        help=(
            f'optional {title} host fingerprint; overrides the common '
            'fingerprint'
        ),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    for prefix, product in PRODUCTS:
        _add_product_arguments(parser, prefix, product)

    parser.add_argument('--captured-at')
    parser.add_argument('--cohort-id', required=True)
    parser.add_argument('--comparison-pair-id', required=True)
    parser.add_argument('--input-id', required=True)
    parser.add_argument(
        '--lidarslam-order',
        choices=('first', 'second'),
        default='first',
        help='which product the operator attempts first (default: first)',
    )
    parser.add_argument(
        '--operator-class',
        choices=('maintainer', 'external'),
        default='external',
    )
    parser.add_argument('--not-first-attempt', action='store_true')
    parser.add_argument('--clean-start', action='store_true')
    parser.add_argument(
        '--ros-distro',
        choices=('humble', 'jazzy'),
        required=True,
    )
    parser.add_argument(
        '--os-family',
        choices=('ubuntu-22.04', 'ubuntu-24.04'),
        required=True,
    )
    parser.add_argument(
        '--architecture',
        choices=('x86_64', 'aarch64'),
        required=True,
    )
    parser.add_argument('--hardware-class', required=True)
    parser.add_argument(
        '--machine-fingerprint-sha256',
        help=(
            'common host fingerprint; provide per-product overrides when '
            'the paired runs use different hosts'
        ),
    )
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument(
        '--verify-public',
        action='store_true',
        help=(
            'GET-check both canonical identities and documentation URLs '
            'before writing either worksheet'
        ),
    )
    return parser


def _fingerprint(args: argparse.Namespace, prefix: str) -> str:
    value = getattr(args, f'{prefix}_machine_fingerprint_sha256')
    value = value or args.machine_fingerprint_sha256
    if value is None:
        raise ScorecardError(
            'provide --machine-fingerprint-sha256 or both per-product '
            'fingerprint overrides'
        )
    return value


def _product_args(
    args: argparse.Namespace,
    prefix: str,
    product: str,
    product_order: str,
    captured_at: str,
    publicly_resolvable: bool,
) -> argparse.Namespace:
    """Translate pair options into the single-worksheet builder contract."""
    return argparse.Namespace(
        trial_id=(
            getattr(args, f'{prefix}_trial_id')
            or f'ux-pair-{product}-{datetime.now(timezone.utc):%Y%m%d}'
        ),
        captured_at=captured_at,
        product=product,
        version=getattr(args, f'{prefix}_version'),
        revision_kind=getattr(args, f'{prefix}_revision_kind'),
        revision=getattr(args, f'{prefix}_revision'),
        documentation_url=getattr(args, f'{prefix}_documentation_url'),
        publicly_resolvable=publicly_resolvable,
        operator_class=args.operator_class,
        cohort_id=args.cohort_id,
        not_first_attempt=args.not_first_attempt,
        product_order=product_order,
        comparison_pair_id=args.comparison_pair_id,
        clean_start=args.clean_start,
        ros_distro=args.ros_distro,
        os_family=args.os_family,
        architecture=args.architecture,
        hardware_class=args.hardware_class,
        machine_fingerprint_sha256=_fingerprint(args, prefix),
        input_id=args.input_id,
    )


def _validate_pair(
    lidarslam: dict,
    glim: dict,
) -> None:
    """Check the pair invariants before either worksheet is written."""
    for field in (
        'comparison_pair_id',
        'ros_distro',
        'os_family',
        'architecture',
        'hardware_class',
    ):
        if lidarslam['environment'][field] != glim['environment'][field]:
            raise ScorecardError(f'paired environment {field} differs')
    for field in ('class', 'cohort_id', 'first_attempt'):
        if lidarslam['operator'][field] != glim['operator'][field]:
            raise ScorecardError(f'paired operator {field} differs')
    if (
        {
            lidarslam['operator']['product_order'],
            glim['operator']['product_order'],
        }
        != {'first', 'second'}
    ):
        raise ScorecardError(
            'paired product order must contain first and second')
    if lidarslam['trial_id'] == glim['trial_id']:
        raise ScorecardError('paired trial IDs must be different')
    for left_task, right_task in zip(lidarslam['tasks'], glim['tasks']):
        if left_task['input_id'] != right_task['input_id']:
            raise ScorecardError(
                f"paired input differs for {left_task['task_id']}")


def _json_payload(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode('utf-8')


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _write_pair(
    output_dir: Path,
    records: Sequence[dict[str, Any]],
    receipt: Mapping[str, Any],
) -> tuple[list[Path], Path]:
    """Stage and exclusively publish both records and their receipt."""
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = [output_dir / f"{record['trial_id']}.json" for record in records]
    receipt_path = output_dir / PREPARATION_RECEIPT_NAME
    if receipt_path in paths:
        raise ScorecardError(
            f'trial ID reserves receipt filename: {PREPARATION_RECEIPT_NAME}'
        )
    destinations = paths + [receipt_path]
    existing = [
        path for path in destinations if path.exists() or path.is_symlink()
    ]
    if existing:
        raise ScorecardError(
            'refusing to overwrite existing preparation artifact: '
            + str(existing[0])
        )
    worksheet_payloads = [_json_payload(record) for record in records]
    expected_hashes = [item['sha256'] for item in receipt['files']]
    if [_sha256(payload) for payload in worksheet_payloads] != expected_hashes:
        raise ScorecardError('worksheet payload changed after receipt review')
    payloads = worksheet_payloads + [_json_payload(receipt)]
    created: list[Path] = []
    try:
        with tempfile.TemporaryDirectory(
            prefix='.usability-pair-',
            dir=output_dir,
        ) as temporary:
            staged_paths = []
            for index, payload in enumerate(payloads):
                staged = Path(temporary) / f'{index}.json'
                with staged.open('xb') as stream:
                    stream.write(payload)
                    stream.flush()
                    os.fsync(stream.fileno())
                staged_paths.append(staged)
            for staged, path in zip(staged_paths, destinations):
                os.link(staged, path)
                created.append(path)
    except (OSError, TypeError) as exc:
        for path in reversed(created):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            except OSError as rollback_exc:
                raise ScorecardError(
                    'cannot roll back partially published worksheet pair: '
                    f'{rollback_exc}'
                ) from exc
        raise ScorecardError(
            f'cannot publish worksheet pair and receipt atomically: {exc}'
        ) from exc
    return paths, receipt_path


def _not_run_public_check() -> dict[str, Any]:
    return {
        'performed': False,
        'status': 'NOT_RUN',
        'network_reads_performed': False,
        'results': [],
    }


def _load_preparation_schema() -> dict[str, Any]:
    try:
        schema = json.loads(PREPARATION_SCHEMA.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScorecardError(
            f'cannot read pair preparation schema: {exc}'
        ) from exc
    if not isinstance(schema, dict):
        raise ScorecardError('pair preparation schema is not an object')
    return schema


def validate_preparation_receipt(
    receipt: dict[str, Any],
    records: Sequence[dict[str, Any]],
) -> None:
    """Validate one schema- and content-bound preparation receipt."""
    schema = _load_preparation_schema()
    try:
        jsonschema.Draft7Validator.check_schema(schema)
    except jsonschema.SchemaError as exc:
        raise ScorecardError(
            f'pair preparation schema is invalid: {exc.message}'
        ) from exc
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(receipt),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        path = '.'.join(str(part) for part in first.absolute_path)
        raise ScorecardError(
            f'pair preparation receipt failed at {path or "<root>"}: '
            f'{first.message}'
        )
    if receipt['schema_uri'] != PREPARATION_SCHEMA_URI:
        raise ScorecardError('pair preparation schema URI is unsupported')
    if receipt['receipt_filename'] != PREPARATION_RECEIPT_NAME:
        raise ScorecardError(
            'pair preparation receipt filename is unsupported'
        )
    products = [record['product']['id'] for record in records]
    if products != ['lidarslam_ros2', 'glim']:
        raise ScorecardError('pair preparation product order changed')
    expected_files = [
        {
            'filename': f"{record['trial_id']}.json",
            'product': record['product']['id'],
            'product_order': record['operator']['product_order'],
            'trial_id': record['trial_id'],
            'sha256': _sha256(_json_payload(record)),
        }
        for record in records
    ]
    if receipt['files'] != expected_files:
        raise ScorecardError(
            'pair preparation receipt file binding differs from worksheets'
        )
    check = receipt['public_identity_check']
    expected_public = check['status'] == 'PASS'
    if any(
        record['product']['publicly_resolvable'] is not expected_public
        for record in records
    ):
        raise ScorecardError(
            'worksheet public identity status differs from its GET check'
        )
    result_products = [
        result['product_id'] for result in check['results']
    ]
    if result_products not in ([], products):
        raise ScorecardError(
            'public identity results are incomplete or reordered'
        )
    if expected_public:
        for record, result in zip(records, check['results']):
            product = record['product']
            kind = product['revision']['kind']
            requested = product['revision']['value']
            expected_source = (
                REGISTRIES[product['id']]['source']
                if kind == 'image-digest'
                else 'github.com/' + GITHUB_REPOSITORIES[product['id']]
            )
            expected_fields = {
                'product_id': product['id'],
                'identity_source': expected_source,
                'revision_kind': kind,
                'requested_revision': requested,
                'documentation_url': product['documentation_root_url'],
            }
            if any(
                result.get(name) != value
                for name, value in expected_fields.items()
            ):
                raise ScorecardError(
                    'public identity result differs from its worksheet'
                )
            resolved = result['resolved_revision']
            if kind in ('git-commit', 'image-digest'):
                if resolved != requested:
                    raise ScorecardError(
                        'public identity resolved revision changed'
                    )
            elif SHA_PATTERN.fullmatch(resolved) is None:
                raise ScorecardError(
                    'public release tag did not resolve to a full commit SHA'
                )
            _validate_documentation_boundary(
                product['id'],
                result['documentation_final_url'],
                redirect=True,
            )
    kinds = [record['product']['revision']['kind'] for record in records]
    network_mode = 'GET_ONLY' if expected_public else 'NONE'
    expected_registry = (
        'GET_ONLY' if expected_public and 'image-digest' in kinds else 'NONE'
    )
    expected_github = (
        'GET_ONLY'
        if expected_public and any(kind != 'image-digest' for kind in kinds)
        else 'NONE'
    )
    authority = receipt['authority']
    if authority != {
        'github_requests': expected_github,
        'registry_requests': expected_registry,
        'documentation_requests': network_mode,
        'github_writes_authorized': False,
        'local_worksheets_written': True,
        'local_preparation_receipt_written': True,
        'rollback_on_publication_error': True,
        'remote_mutations_performed': False,
    }:
        raise ScorecardError('pair preparation authority is inconsistent')


def main(
    argv: Sequence[str] | None = None,
    *,
    public_verifier: Callable[
        [Sequence[dict[str, Any]]],
        dict[str, Any],
    ] | None = None,
) -> int:
    """Prepare one incomplete worksheet per product without overwriting."""
    args = _parser().parse_args(argv)
    try:
        if (
            args.lidarslam_publicly_resolvable
            or args.glim_publicly_resolvable
        ):
            raise ScorecardError(
                'manual publicly-resolvable flags are not accepted by the '
                'paired workflow; use --verify-public'
            )
        captured_at = args.captured_at or _now()
        lidarslam_order = args.lidarslam_order
        glim_order = 'second' if lidarslam_order == 'first' else 'first'
        lidarslam = build_template(_product_args(
            args,
            'lidarslam',
            'lidarslam_ros2',
            lidarslam_order,
            captured_at,
            False,
        ))
        glim = build_template(_product_args(
            args,
            'glim',
            'glim',
            glim_order,
            captured_at,
            False,
        ))
        _validate_pair(lidarslam, glim)
        records = (lidarslam, glim)
        public_check = _not_run_public_check()
        if args.verify_public:
            verifier = public_verifier or verify_public_pair
            public_check = verifier(records)
            if public_check.get('status') != 'PASS':
                raise ScorecardError(
                    'public identity verification did not pass'
                )
            for record in records:
                record['product']['publicly_resolvable'] = True
                validate_trial(record)
            _validate_pair(lidarslam, glim)
        paths = [
            args.output_dir / f"{record['trial_id']}.json"
            for record in records
        ]
        receipt = {
            'schema_version': 1,
            'schema_uri': PREPARATION_SCHEMA_URI,
            'receipt_filename': PREPARATION_RECEIPT_NAME,
            'status': 'PREPARED_INCOMPLETE',
            'comparison_pair_id': args.comparison_pair_id,
            'public_identity_check': public_check,
            'files': [
                {
                    'filename': path.name,
                    'product': record['product']['id'],
                    'product_order': record['operator']['product_order'],
                    'trial_id': record['trial_id'],
                    'sha256': _sha256(_json_payload(record)),
                }
                for path, record in zip(paths, (lidarslam, glim))
            ],
            'authority': {
                'github_requests': (
                    'GET_ONLY'
                    if args.verify_public and any(
                        record['product']['revision']['kind'] != 'image-digest'
                        for record in records
                    )
                    else 'NONE'
                ),
                'registry_requests': (
                    'GET_ONLY'
                    if args.verify_public and any(
                        record['product']['revision']['kind'] == 'image-digest'
                        for record in records
                    )
                    else 'NONE'
                ),
                'documentation_requests': (
                    'GET_ONLY' if args.verify_public else 'NONE'
                ),
                'github_writes_authorized': False,
                'local_worksheets_written': True,
                'local_preparation_receipt_written': True,
                'rollback_on_publication_error': True,
                'remote_mutations_performed': False,
            },
        }
        validate_preparation_receipt(receipt, records)
        written_paths, receipt_path = _write_pair(
            args.output_dir,
            records,
            receipt,
        )
        if written_paths != paths:
            raise ScorecardError(
                'written worksheet paths changed after review'
            )
        if receipt_path != args.output_dir / PREPARATION_RECEIPT_NAME:
            raise ScorecardError('written receipt path changed after review')
        sys.stdout.buffer.write(_json_payload(receipt))
        print(
            'Worksheets are incomplete; do not add them to the reviewed '
            'evidence index until the observed pair is complete.',
            file=sys.stderr,
        )
    except (OSError, ScorecardError) as exc:
        print(f'usability scorecard pair error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
