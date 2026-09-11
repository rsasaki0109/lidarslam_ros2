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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the fail-closed published GitHub Release audit."""

from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_published_release.py'
SPEC = importlib.util.spec_from_file_location('published_release', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)

VERSION = '0.9.0'
TAG = f'v{VERSION}'
COMMIT = 'c' * 40
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True) + '\n').encode()


def _bundle(*, version=VERSION, tamper_member=False):
    tag = f'v{version}'
    version_payload = f'{version}\n'.encode()
    manifest = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/release-bundle-manifest-v1.schema.json'
        ),
        'status': 'PASS',
        'tag': tag,
        'product_version': version,
        'git_commit': COMMIT,
        'files': [{
            'path': 'VERSION',
            'size_bytes': len(version_payload),
            'sha256': hashlib.sha256(version_payload).hexdigest(),
        }],
    }
    archived_version = b'tampered\n' if tamper_member else version_payload
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode='w:gz') as archive:
        for name, payload in (
            ('release_bundle/VERSION', archived_version),
            (
                'release_bundle/release-bundle-manifest-v1.json',
                _json_bytes(manifest),
            ),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))
    return output.getvalue()


def _image(distro, digest_character, *, version=VERSION):
    tag = f'v{version}'
    return {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': distro,
        'platform': 'linux/amd64',
        'tag': f'ghcr.io/{REPOSITORY}:{tag}-{distro}',
        'digest': 'sha256:' + digest_character * 64,
        'git_commit': COMMIT,
        'product_version': version,
        'cli_version': f'lidarslam_ros2 {version}',
    }


def _rollback(image):
    distro = image['ros_distro']
    immutable = 'ghcr.io/{repo}@{digest}'.format(
        repo=REPOSITORY,
        digest=image['digest'],
    )
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/rollback-plan-v1.schema.json'
        ),
        'status': 'PASS',
        'source_record': f'release-image-{distro}.json',
        'repository': REPOSITORY,
        'ros_distro': distro,
        'platform': image['platform'],
        'product_version': image['product_version'],
        'git_commit': COMMIT,
        'tag': image['tag'],
        'digest': image['digest'],
        'immutable_ref': immutable,
        'moving_tag_mutated': False,
        'commands': {
            'pull': f'docker pull {immutable}',
            'attestation': (
                f'gh attestation verify oci://{immutable} -R {REPOSITORY}'
            ),
            'cli_smoke': (
                f'docker run --rm {immutable} lidarslam-map --version'
            ),
        },
    }


def _launcher(*, version, commit=COMMIT):
    source = (
        ROOT / 'scripts' / 'docker_map_bag.sh'
    ).read_text(encoding='utf-8')
    return source.replace(
        'LIDARSLAM_DOCKER_LAUNCHER_VERSION="development"',
        f'LIDARSLAM_DOCKER_LAUNCHER_VERSION="v{version}"',
    ).replace(
        'LIDARSLAM_DOCKER_LAUNCHER_REVISION="working-tree"',
        f'LIDARSLAM_DOCKER_LAUNCHER_REVISION="{commit}"',
    ).encode()


def _snapshot(*, version=VERSION, include_launcher=False):
    tag = f'v{version}'
    images = {
        'humble': _image('humble', 'a', version=version),
        'jazzy': _image('jazzy', 'b', version=version),
    }
    promotion = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/release-promotion-v1.schema.json'
        ),
        'status': 'PASS',
        'mode': 'applied',
        'repository': REPOSITORY,
        'product_version': version,
        'git_commit': COMMIT,
        'moving_tag_mutated': False,
        'images': [
            {
                'ros_distro': distro,
                'tag': image['tag'],
                'digest': image['digest'],
                'immutable_ref': (
                    'ghcr.io/{repo}@{digest}'.format(
                        repo=REPOSITORY,
                        digest=image['digest'],
                    )
                ),
                'action': 'create',
            }
            for distro, image in images.items()
        ],
        'created_tags': [image['tag'] for image in images.values()],
        'reused_tags': [],
    }
    bundle_name = f'lidarslam_ros2_{tag}_release_bundle.tar.gz'
    asset_payloads = {
        bundle_name: _bundle(version=version),
        'release-image-humble.json': _json_bytes(images['humble']),
        'release-image-jazzy.json': _json_bytes(images['jazzy']),
        'rollback-plan-humble.json': _json_bytes(
            _rollback(images['humble'])),
        'rollback-plan-jazzy.json': _json_bytes(
            _rollback(images['jazzy'])),
        'release-promotion.json': _json_bytes(promotion),
    }
    if include_launcher:
        asset_payloads['lidarslam-map-docker'] = _launcher(version=version)
    return {
        'errors': [],
        'tag_commit': COMMIT,
        'release': {
            'tag_name': tag,
            'draft': False,
            'prerelease': False,
            'html_url': (
                f'https://github.com/{REPOSITORY}/releases/tag/{tag}'
            ),
        },
        'asset_payloads': asset_payloads,
        'image_tag_digests': {
            image['tag']: image['digest']
            for image in images.values()
        },
    }


def test_complete_stable_release_with_cross_checked_assets_is_published():
    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=_snapshot(),
    )

    assert report['status'] == 'PUBLISHED'
    assert len(report['assets']) == 6
    assert all(asset['status'] == 'PASS' for asset in report['assets'])
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_next_release_requires_verified_clone_free_launcher():
    version = '0.9.1'
    report = AUDIT.evaluate_publication(
        version=version,
        snapshot=_snapshot(version=version, include_launcher=True),
    )

    assert report['status'] == 'PUBLISHED'
    assert len(report['assets']) == 7
    launcher = next(
        asset for asset in report['assets']
        if asset['name'] == 'lidarslam-map-docker'
    )
    assert launcher['status'] == 'PASS'


def test_missing_or_tampered_clone_free_launcher_blocks_next_release():
    version = '0.9.1'
    missing = AUDIT.evaluate_publication(
        version=version,
        snapshot=_snapshot(version=version),
    )
    assert missing['status'] == 'BLOCKED'
    required = next(
        check for check in missing['checks']
        if check['id'] == 'required-assets'
    )
    assert required['status'] == 'FAIL'

    snapshot = _snapshot(version=version, include_launcher=True)
    snapshot['asset_payloads']['lidarslam-map-docker'] = _launcher(
        version=version,
        commit='d' * 40,
    )
    tampered = AUDIT.evaluate_publication(
        version=version,
        snapshot=snapshot,
    )
    assert tampered['status'] == 'BLOCKED'
    launcher = next(
        asset for asset in tampered['assets']
        if asset['name'] == 'lidarslam-map-docker'
    )
    assert launcher['status'] == 'FAIL'
    assert 'revision identity differs' in launcher['detail']


def test_historical_release_rejects_unexpected_launcher_asset():
    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=_snapshot(include_launcher=True),
    )

    assert report['status'] == 'BLOCKED'
    required = next(
        check for check in report['checks']
        if check['id'] == 'required-assets'
    )
    assert required['status'] == 'FAIL'


def test_absent_tag_and_release_are_not_published():
    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot={
            'errors': [],
            'tag_commit': None,
            'release': None,
            'asset_payloads': {},
        },
    )

    assert report['status'] == 'NOT_PUBLISHED'


def test_remote_inspection_uses_explicit_tag_404(monkeypatch):
    urls = []

    def fake_request(url):
        urls.append(url)
        return 404, None

    monkeypatch.setattr(AUDIT, '_request_json', fake_request)
    monkeypatch.setattr(
        AUDIT,
        '_registry_tag_digest',
        lambda tag: None,
    )

    snapshot = AUDIT.inspect_remote(VERSION)

    assert snapshot['errors'] == []
    assert snapshot['tag_commit'] is None
    assert snapshot['release'] is None
    assert any('/git/ref/tags/' in url for url in urls)
    assert not any('/commits/' in url for url in urls)
    assert [item['status'] for item in snapshot['image_reports']] == [
        'ABSENT',
        'ABSENT',
    ]


def test_missing_release_reports_each_versioned_image_as_absent():
    report = AUDIT.evaluate_publication(
        version='0.9.1',
        snapshot={
            'errors': [],
            'tag_commit': None,
            'release': None,
            'asset_payloads': {},
            'image_reports': [
                {
                    'tag': f'ghcr.io/{REPOSITORY}:v0.9.1-humble',
                    'status': 'ABSENT',
                    'digest': None,
                    'detail': 'GHCR tag is not published',
                },
                {
                    'tag': f'ghcr.io/{REPOSITORY}:v0.9.1-jazzy',
                    'status': 'ABSENT',
                    'digest': None,
                    'detail': 'GHCR tag is not published',
                },
            ],
        },
    )

    assert report['status'] == 'NOT_PUBLISHED'
    assert [item['status'] for item in report['images']] == [
        'ABSENT',
        'ABSENT',
    ]


def test_image_audit_failure_cannot_become_published():
    snapshot = _snapshot()
    snapshot['image_reports'] = [
        {
            'tag': f'ghcr.io/{REPOSITORY}:{TAG}-humble',
            'status': 'ABSENT',
            'digest': None,
            'detail': 'GHCR tag is not published',
        },
        {
            'tag': f'ghcr.io/{REPOSITORY}:{TAG}-jazzy',
            'status': 'PUBLISHED',
            'digest': snapshot['image_tag_digests'][
                f'ghcr.io/{REPOSITORY}:{TAG}-jazzy'
            ],
            'detail': 'GHCR tag resolves to the recorded digest',
        },
    ]

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'


def test_bounded_request_retries_transient_timeout(monkeypatch):
    """One transient timeout must not turn a published release into BLOCKED."""
    calls = []
    sleeps = []

    class FakeResponse:
        status = 200
        headers = {}

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        @staticmethod
        def read(_limit):
            return b'{}'

    def fake_urlopen(_request, timeout):
        calls.append(timeout)
        if len(calls) < AUDIT.REQUEST_ATTEMPTS:
            raise AUDIT.urllib.error.URLError(TimeoutError('timed out'))
        return FakeResponse()

    monkeypatch.setattr(AUDIT.urllib.request, 'urlopen', fake_urlopen)
    monkeypatch.setattr(AUDIT.time, 'sleep', sleeps.append)

    assert AUDIT._request('https://example.test/release', limit=10) == (
        200, b'{}')
    assert calls == [30, 30, 30]
    assert sleeps == [1, 2]


def test_registry_digest_resolver_uses_public_bearer_token(monkeypatch):
    """The live resolver must use scoped bearer auth and OCI media types."""
    digest = 'sha256:' + 'a' * 64

    def fake_request(url, *, limit):
        assert url.startswith('https://ghcr.io/token?')
        assert limit == AUDIT.MAX_REGISTRY_TOKEN_BYTES
        return 200, b'{"token":"public-pull-token"}'

    class FakeResponse:
        status = 200
        headers = {'Docker-Content-Digest': digest}

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        @staticmethod
        def read(limit):
            assert limit == AUDIT.MAX_REGISTRY_MANIFEST_BYTES + 1
            return b'{}'

    def fake_urlopen(request, timeout):
        assert request.full_url.endswith('/manifests/v0.9.0-jazzy')
        assert request.get_header('Authorization') == (
            'Bearer public-pull-token'
        )
        assert 'application/vnd.oci.image.index.v1+json' in (
            request.get_header('Accept')
        )
        assert timeout == 30
        return FakeResponse()

    monkeypatch.setattr(AUDIT, '_request', fake_request)
    monkeypatch.setattr(AUDIT.urllib.request, 'urlopen', fake_urlopen)

    assert AUDIT._registry_tag_digest('v0.9.0-jazzy') == digest


def test_tag_without_release_is_in_progress():
    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot={
            'errors': [],
            'tag_commit': COMMIT,
            'release': None,
            'asset_payloads': {},
        },
    )

    assert report['status'] == 'IN_PROGRESS'


def test_remote_error_is_blocked_not_absent():
    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot={
            'errors': ['GitHub returned HTTP 503'],
            'tag_commit': None,
            'release': None,
            'asset_payloads': {},
        },
    )

    assert report['status'] == 'BLOCKED'
    assert report['remote']['errors'] == ['GitHub returned HTTP 503']


def test_prerelease_flag_blocks_stable_candidate():
    snapshot = _snapshot()
    snapshot['release']['prerelease'] = True

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'
    check = next(
        item for item in report['checks']
        if item['id'] == 'stable-release-channel'
    )
    assert check['status'] == 'FAIL'


def test_tampered_bundle_member_is_blocked():
    snapshot = _snapshot()
    bundle_name = f'lidarslam_ros2_{TAG}_release_bundle.tar.gz'
    snapshot['asset_payloads'][bundle_name] = _bundle(tamper_member=True)

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'
    bundle = next(
        asset for asset in report['assets']
        if asset['name'] == bundle_name
    )
    assert bundle['status'] == 'FAIL'


def test_cross_asset_commit_mismatch_is_blocked():
    snapshot = _snapshot()
    image = json.loads(
        snapshot['asset_payloads']['release-image-jazzy.json'])
    image['git_commit'] = 'd' * 40
    snapshot['asset_payloads']['release-image-jazzy.json'] = _json_bytes(image)

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'
    check = next(
        item for item in report['checks']
        if item['id'] == 'cross-asset-identity'
    )
    assert check['status'] == 'FAIL'


def test_missing_required_asset_is_blocked():
    snapshot = _snapshot()
    del snapshot['asset_payloads']['rollback-plan-humble.json']

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'
    check = next(
        item for item in report['checks']
        if item['id'] == 'required-assets'
    )
    assert check['status'] == 'FAIL'


def test_moved_live_image_tag_is_blocked():
    """A version tag moved away from its release record must block the audit."""
    snapshot = _snapshot()
    tag = f'ghcr.io/{REPOSITORY}:{TAG}-jazzy'
    snapshot['image_tag_digests'][tag] = 'sha256:' + 'd' * 64

    report = AUDIT.evaluate_publication(
        version=VERSION,
        snapshot=snapshot,
    )

    assert report['status'] == 'BLOCKED'
    check = next(
        item for item in report['checks']
        if item['id'] == 'live-image-tag-digests'
    )
    assert check['status'] == 'FAIL'


def test_release_workflow_docs_and_bundle_require_publication_audit():
    workflow = (
        ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')
    releasing = (ROOT / 'RELEASING.md').read_text(encoding='utf-8')
    reliability = (ROOT / 'docs' / 'operational-reliability.md').read_text(
        encoding='utf-8')
    release_bundle = (ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8')

    assert 'scripts/check_published_release.py' in workflow
    assert '--require-published' in workflow
    assert 'published-release-audit.json' in workflow
    assert "format('refs/tags/{0}', inputs.tag_name)" in workflow
    assert 'REQUESTED_TAG: ${{ github.event.inputs.tag_name || github.ref_name }}' in workflow
    assert 'TAG_NAME="${REQUESTED_TAG}"' in workflow
    assert 'TAG_NAME="${{' not in workflow
    assert 'TAG_REF="refs/tags/${TAG_NAME}"' in workflow
    assert 'git show-ref --verify --quiet "${TAG_REF}"' in workflow
    assert 'git rev-parse --verify "${TAG_REF}^{commit}"' in workflow
    assert 'checkout does not match immutable tag ${TAG_REF}' in workflow
    assert (
        "ref: ${{ format('refs/tags/{0}', "
        'needs.metadata.outputs.tag_name) }}'
    ) in workflow
    assert workflow.count('artifact-metadata: write') == 2
    assert workflow.count('contents: write') == 1
    assert workflow.count('packages: write') == 2
    assert workflow.count('attestations: write') == 2
    assert workflow.count('id-token: write') == 2
    assert 'scripts/build_docker_launcher_asset.py' in workflow
    assert '--output lidarslam-map-docker' in workflow
    assert 'lidarslam-map-docker --version' in workflow
    assert 'check_published_release.py --require-published' in releasing
    assert 'published-release-v1.schema.json' in reliability
    assert 'scripts/check_published_release.py' in release_bundle
    assert 'scripts/build_docker_launcher_asset.py' in release_bundle
