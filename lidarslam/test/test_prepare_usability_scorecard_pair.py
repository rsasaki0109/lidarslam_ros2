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

"""Tests for paired fail-closed usability scorecard worksheet preparation."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Mapping

import jsonschema


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_usability_scorecard_pair.py'
SPEC = importlib.util.spec_from_file_location(
    'prepare_usability_scorecard_pair_test',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)

LIDARSLAM_SHA = 'a' * 40
GLIM_SHA = 'faa264a1bce1bda406f73457e35511f56cdc2eaa'
TAG_OBJECT_SHA = 'b' * 40
LIDARSLAM_DOCS = (
    f'https://github.com/rsasaki0109/lidar_slam_ros2/tree/'
    f'{LIDARSLAM_SHA}/docs'
)
GLIM_DOCS = 'https://koide3.github.io/glim/'


def _args(output_dir: Path, **overrides: str | None) -> list[str]:
    values = {
        '--lidarslam-version': '0.9.1',
        '--lidarslam-revision-kind': 'git-commit',
        '--lidarslam-revision': LIDARSLAM_SHA,
        '--lidarslam-documentation-url': 'https://example.test/lidarslam',
        '--lidarslam-trial-id': 'lidarslam-pair-trial-a',
        '--glim-version': '1.0.0',
        '--glim-revision-kind': 'release-tag',
        '--glim-revision': 'v1.0.0',
        '--glim-documentation-url': 'https://example.test/glim',
        '--glim-trial-id': 'glim-pair-trial-a',
        '--cohort-id': 'external-paired-operator-a',
        '--comparison-pair-id': 'paired-jazzy-machine-class-a',
        '--input-id': 'fixed-demo-v1',
        '--ros-distro': 'jazzy',
        '--os-family': 'ubuntu-24.04',
        '--architecture': 'x86_64',
        '--hardware-class': 'eight-core-32gib-x86_64',
        '--machine-fingerprint-sha256': 'b' * 64,
        '--output-dir': str(output_dir),
    }
    values.update(overrides)
    result = []
    for key, value in values.items():
        if value is None:
            continue
        result.extend((key, value))
    return result


def _records(output_dir: Path) -> dict[str, dict]:
    return {
        path.stem: json.loads(path.read_text(encoding='utf-8'))
        for path in output_dir.glob('*.json')
        if path.name != PREPARE.PREPARATION_RECEIPT_NAME
    }


def _verified_args(output_dir: Path, **overrides: str | None) -> list[str]:
    values = {
        '--lidarslam-documentation-url': LIDARSLAM_DOCS,
        '--glim-version': '1.2.2',
        '--glim-revision': 'v1.2.2',
        '--glim-documentation-url': GLIM_DOCS,
    }
    values.update(overrides)
    return _args(output_dir, **values) + ['--verify-public']


class FakeHttp:
    """Return exact bounded HTTP responses and retain requested URLs."""

    def __init__(self, responses: Mapping[str, tuple]):
        """Store the finite response map."""
        self.responses = responses
        self.calls: list[tuple[str, Mapping[str, str], int]] = []

    def __call__(self, url, headers, limit):
        """Return one mapped response and reject unexpected requests."""
        self.calls.append((url, headers, limit))
        if url not in self.responses:
            raise AssertionError(f'unexpected GET: {url}')
        return self.responses[url]


def _json_response(url: str, value: dict) -> tuple:
    return (200, json.dumps(value).encode(), {}, url)


def _public_http(
    *,
    lidar_sha: str = LIDARSLAM_SHA,
    glim_ref_type: str = 'tag',
    glim_ref_sha: str = TAG_OBJECT_SHA,
    lidar_docs_final: str = LIDARSLAM_DOCS,
    lidar_docs_status: int = 200,
) -> FakeHttp:
    lidar_api = (
        'https://api.github.com/repos/rsasaki0109/lidar_slam_ros2/'
        f'commits/{LIDARSLAM_SHA}'
    )
    glim_ref_api = (
        'https://api.github.com/repos/koide3/glim/git/ref/tags/v1.2.2'
    )
    responses = {
        lidar_api: _json_response(lidar_api, {'sha': lidar_sha}),
        glim_ref_api: _json_response(glim_ref_api, {
            'ref': 'refs/tags/v1.2.2',
            'object': {'type': glim_ref_type, 'sha': glim_ref_sha},
        }),
        LIDARSLAM_DOCS: (
            lidar_docs_status,
            b'lidarslam docs',
            {},
            lidar_docs_final,
        ),
        GLIM_DOCS: (200, b'glim docs', {}, GLIM_DOCS),
    }
    if glim_ref_type == 'tag':
        glim_tag_api = (
            'https://api.github.com/repos/koide3/glim/git/tags/'
            f'{glim_ref_sha}'
        )
        responses[glim_tag_api] = _json_response(glim_tag_api, {
            'object': {'type': 'commit', 'sha': GLIM_SHA},
        })
    return FakeHttp(responses)


def _verifier(http: FakeHttp):
    return lambda records: PREPARE.verify_public_pair(
        records,
        http_get=http,
    )


def test_github_identity_client_uses_scoped_read_credential(monkeypatch):
    """Public pair identity reads use the shared GitHub GET auth boundary."""
    captured = {}

    def http_get(url, headers, limit):
        captured['url'] = url
        captured['headers'] = headers
        captured['limit'] = limit
        return 200, b'{"sha": "abc"}', {}, url

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')

    payload = PREPARE._github_json(
        'owner/repo',
        'commits/abc',
        http_get,
    )

    assert payload == {'sha': 'abc'}
    assert captured['url'] == (
        'https://api.github.com/repos/owner/repo/commits/abc'
    )
    assert captured['headers']['Authorization'] == (
        'Bearer read-only-test-token'
    )
    assert captured['limit'] == PREPARE.MAX_JSON_BYTES


def test_pair_has_shared_metadata_and_opposite_order(tmp_path, capsys):
    """One command prepares two independently valid incomplete worksheets."""
    assert PREPARE.main(_args(tmp_path)) == 0
    manifest = json.loads(capsys.readouterr().out)
    records = _records(tmp_path)

    assert manifest['status'] == 'PREPARED_INCOMPLETE'
    assert manifest['public_identity_check'] == {
        'performed': False,
        'status': 'NOT_RUN',
        'network_reads_performed': False,
        'results': [],
    }
    assert manifest['authority'] == {
        'github_requests': 'NONE',
        'registry_requests': 'NONE',
        'documentation_requests': 'NONE',
        'github_writes_authorized': False,
        'local_worksheets_written': True,
        'local_preparation_receipt_written': True,
        'rollback_on_publication_error': True,
        'remote_mutations_performed': False,
    }
    receipt_path = tmp_path / PREPARE.PREPARATION_RECEIPT_NAME
    assert receipt_path.exists()
    assert json.loads(receipt_path.read_text(encoding='utf-8')) == manifest
    assert {
        item['filename']: item['sha256'] for item in manifest['files']
    } == {
        path.name: PREPARE._sha256(path.read_bytes())
        for path in tmp_path.glob('*-pair-trial-a.json')
    }
    assert set(records) == {'lidarslam-pair-trial-a', 'glim-pair-trial-a'}
    lidarslam = records['lidarslam-pair-trial-a']
    glim = records['glim-pair-trial-a']
    assert lidarslam['product']['id'] == 'lidarslam_ros2'
    assert glim['product']['id'] == 'glim'
    assert lidarslam['operator']['product_order'] == 'first'
    assert glim['operator']['product_order'] == 'second'
    assert lidarslam['environment']['comparison_pair_id'] == (
        glim['environment']['comparison_pair_id']
    )
    assert lidarslam['environment']['machine_fingerprint_sha256'] == (
        glim['environment']['machine_fingerprint_sha256']
    )
    assert all(
        task['outcome']['status'] == 'FAIL'
        for record in (lidarslam, glim)
        for task in record['tasks']
    )
    assert all(
        record['product']['publicly_resolvable'] is False
        for record in (lidarslam, glim)
    )


def test_per_product_fingerprints_are_supported(tmp_path, capsys):
    """Different hosts work without repeating the pair metadata."""
    args = _args(
        tmp_path,
        **{
            '--machine-fingerprint-sha256': None,
            '--lidarslam-machine-fingerprint-sha256': 'c' * 64,
            '--glim-machine-fingerprint-sha256': 'd' * 64,
        },
    )
    args = [value for value in args if value != 'None']
    assert PREPARE.main(args) == 0
    capsys.readouterr()
    records = _records(tmp_path)
    assert records['lidarslam-pair-trial-a']['environment'][
        'machine_fingerprint_sha256'] == 'c' * 64
    assert records['glim-pair-trial-a']['environment'][
        'machine_fingerprint_sha256'] == 'd' * 64


def test_existing_destination_refuses_pair_without_overwriting(tmp_path,
                                                               capsys):
    """An existing destination blocks both writes."""
    existing = tmp_path / 'glim-pair-trial-a.json'
    existing.write_text('keep\n', encoding='utf-8')
    assert PREPARE.main(_args(tmp_path)) == 2
    assert not (tmp_path / 'lidarslam-pair-trial-a.json').exists()
    assert existing.read_text(encoding='utf-8') == 'keep\n'
    assert 'refusing to overwrite' in capsys.readouterr().err


def test_missing_fingerprint_fails_closed(tmp_path, capsys):
    """A pair cannot be prepared without an identity for each host."""
    args = _args(tmp_path, **{'--machine-fingerprint-sha256': None})
    args = [value for value in args if value != 'None']
    assert PREPARE.main(args) == 2
    assert 'fingerprint' in capsys.readouterr().err


def test_manual_public_claim_is_rejected_before_writes(tmp_path, capsys):
    """The paired workflow cannot self-attest one product as public."""
    args = _args(tmp_path) + ['--lidarslam-publicly-resolvable']
    assert PREPARE.main(args) == 2
    assert not list(tmp_path.glob('*.json'))
    assert '--verify-public' in capsys.readouterr().err


def test_public_preflight_resolves_exact_commit_tag_and_docs(tmp_path,
                                                             capsys):
    """One GET-only preflight must pass for both products before writes."""
    http = _public_http()
    assert PREPARE.main(
        _verified_args(tmp_path),
        public_verifier=_verifier(http),
    ) == 0
    manifest = json.loads(capsys.readouterr().out)
    records = _records(tmp_path)

    check = manifest['public_identity_check']
    assert check['status'] == 'PASS'
    assert [item['product_id'] for item in check['results']] == [
        'lidarslam_ros2',
        'glim',
    ]
    assert check['results'][0]['resolved_revision'] == LIDARSLAM_SHA
    assert check['results'][1]['resolved_revision'] == GLIM_SHA
    assert all(
        record['product']['publicly_resolvable'] is True
        for record in records.values()
    )
    assert manifest['authority'] == {
        'github_requests': 'GET_ONLY',
        'registry_requests': 'NONE',
        'documentation_requests': 'GET_ONLY',
        'github_writes_authorized': False,
        'local_worksheets_written': True,
        'local_preparation_receipt_written': True,
        'rollback_on_publication_error': True,
        'remote_mutations_performed': False,
    }
    schema = json.loads(PREPARE.PREPARATION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(manifest)
    assert len(http.calls) == 5


def test_public_commit_identity_drift_leaves_no_pair(tmp_path, capsys):
    """A changed commit response fails before either worksheet is written."""
    http = _public_http(lidar_sha='c' * 40)
    assert PREPARE.main(
        _verified_args(tmp_path),
        public_verifier=_verifier(http),
    ) == 2
    assert not list(tmp_path.glob('*.json'))
    assert 'changed identity' in capsys.readouterr().err


def test_public_tag_must_match_declared_version_without_network(tmp_path,
                                                                capsys):
    """A mismatched release label is rejected before requesting the tag."""
    calls = []

    def no_get(url, headers, limit):
        calls.append(url)
        raise AssertionError('network must not be reached')

    assert PREPARE.main(
        _verified_args(tmp_path, **{'--glim-version': '1.2.1'}),
        public_verifier=lambda records: PREPARE.verify_public_pair(
            records,
            http_get=no_get,
        ),
    ) == 2
    assert calls == []
    assert not list(tmp_path.glob('*.json'))
    assert 'differs from product version' in capsys.readouterr().err


def test_documentation_redirect_cannot_escape_product_boundary(tmp_path,
                                                               capsys):
    """A successful HTTP response on an unapproved redirect is not public."""
    http = _public_http(lidar_docs_final='https://evil.example/docs')
    assert PREPARE.main(
        _verified_args(tmp_path),
        public_verifier=_verifier(http),
    ) == 2
    assert not list(tmp_path.glob('*.json'))
    assert 'outside its public boundary' in capsys.readouterr().err


def test_documentation_source_host_is_fixed_per_product(tmp_path, capsys):
    """A caller cannot substitute an arbitrary responsive docs host."""
    assert PREPARE.main(
        _verified_args(
            tmp_path,
            **{'--lidarslam-documentation-url': 'https://example.test/docs'},
        ),
        public_verifier=_verifier(_public_http()),
    ) == 2
    assert not list(tmp_path.glob('*.json'))
    assert 'outside its public boundary' in capsys.readouterr().err


def test_registry_digest_must_match_canonical_manifest(tmp_path, capsys):
    """An image identity is proven by the canonical registry digest header."""
    digest = 'sha256:' + 'd' * 64
    token_url = PREPARE.REGISTRIES['lidarslam_ros2']['token_url']
    manifest_url = (
        PREPARE.REGISTRIES['lidarslam_ros2']['manifest_root'] + digest
    )
    http = _public_http()
    http.responses[token_url] = _json_response(token_url, {'token': 'public'})
    http.responses[manifest_url] = (
        200,
        b'{}',
        {'docker-content-digest': digest},
        manifest_url,
    )
    args = _verified_args(tmp_path, **{
        '--lidarslam-revision-kind': 'image-digest',
        '--lidarslam-revision': digest,
    })
    assert PREPARE.main(args, public_verifier=_verifier(http)) == 0
    manifest = json.loads(capsys.readouterr().out)
    result = manifest['public_identity_check']['results'][0]
    assert result['identity_source'] == (
        'ghcr.io/rsasaki0109/lidar_slam_ros2'
    )
    assert result['resolved_revision'] == digest
    assert manifest['authority']['registry_requests'] == 'GET_ONLY'
    assert manifest['authority']['github_requests'] == 'GET_ONLY'


def test_registry_digest_drift_leaves_no_pair(tmp_path, capsys):
    """A registry response for a different digest cannot authorize writes."""
    digest = 'sha256:' + 'd' * 64
    token_url = PREPARE.REGISTRIES['lidarslam_ros2']['token_url']
    manifest_url = (
        PREPARE.REGISTRIES['lidarslam_ros2']['manifest_root'] + digest
    )
    http = _public_http()
    http.responses[token_url] = _json_response(token_url, {'token': 'public'})
    http.responses[manifest_url] = (
        200,
        b'{}',
        {'Docker-Content-Digest': 'sha256:' + 'e' * 64},
        manifest_url,
    )
    args = _verified_args(tmp_path, **{
        '--lidarslam-revision-kind': 'image-digest',
        '--lidarslam-revision': digest,
    })
    assert PREPARE.main(args, public_verifier=_verifier(http)) == 2
    assert not list(tmp_path.glob('*.json'))
    assert 'exact digest' in capsys.readouterr().err


def test_second_exclusive_publish_failure_rolls_back_first(tmp_path,
                                                           capsys,
                                                           monkeypatch):
    """A race on the second destination does not leave a half pair."""
    original_link = PREPARE.os.link
    link_count = 0

    def fail_second_link(source, destination):
        nonlocal link_count
        link_count += 1
        if link_count == 2:
            raise FileExistsError('simulated destination race')
        original_link(source, destination)

    monkeypatch.setattr(PREPARE.os, 'link', fail_second_link)
    assert PREPARE.main(_args(tmp_path)) == 2
    assert not list(tmp_path.iterdir())
    assert 'atomically' in capsys.readouterr().err


def test_receipt_publish_failure_rolls_back_both_worksheets(tmp_path,
                                                            capsys,
                                                            monkeypatch):
    """A race on the receipt leaves no unbound worksheet behind."""
    original_link = PREPARE.os.link
    link_count = 0

    def fail_receipt_link(source, destination):
        nonlocal link_count
        link_count += 1
        if link_count == 3:
            raise FileExistsError('simulated receipt destination race')
        original_link(source, destination)

    monkeypatch.setattr(PREPARE.os, 'link', fail_receipt_link)
    assert PREPARE.main(_args(tmp_path)) == 2
    assert not list(tmp_path.iterdir())
    assert 'receipt atomically' in capsys.readouterr().err


def test_existing_receipt_refuses_both_worksheets(tmp_path, capsys):
    """A stale receipt cannot be silently paired with fresh worksheets."""
    receipt = tmp_path / PREPARE.PREPARATION_RECEIPT_NAME
    receipt.write_text('keep\n', encoding='utf-8')
    assert PREPARE.main(_args(tmp_path)) == 2
    assert list(tmp_path.iterdir()) == [receipt]
    assert receipt.read_text(encoding='utf-8') == 'keep\n'
    assert 'preparation artifact' in capsys.readouterr().err


def test_network_implementation_has_no_write_method_or_subprocess():
    """The public verifier stays a GET-only local preparation surface."""
    source = SCRIPT.read_text(encoding='utf-8')
    assert "method='GET'" in source
    for method in ('POST', 'PUT', 'PATCH', 'DELETE'):
        assert f"method='{method}'" not in source
    assert 'import subprocess' not in source
