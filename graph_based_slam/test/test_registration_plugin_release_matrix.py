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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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


"""Static and receipt-level tests for the current-source release matrix."""

import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import stat
import tarfile
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[2]
PROFILE = REPO / 'configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json'
AUDIT = REPO / 'scripts/audit_registration_plugin_matrix.py'
RUNNER = REPO / 'scripts/run_registration_plugin_release_matrix.py'
SUMMARY = REPO / 'scripts/summarize_registration_plugin_release_matrix.py'
DEPENDENCY_CONTRACT = REPO / 'scripts/registration_plugin_dependency_contract.py'
DEPENDENCY_CAPTURE = REPO / 'scripts/capture_registration_plugin_dependency_environment.py'
WORKFLOW = REPO / '.github/workflows/main.yml'


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def audit_module():
    return _load(AUDIT, 'release_matrix_audit')


@pytest.fixture(scope='module')
def release_module():
    return _load(RUNNER, 'release_matrix_runner')


def test_profile_has_two_distros_and_two_non_skippable_legs(audit_module, release_module):
    profile, _ = audit_module._load_profile(PROFILE)
    release = audit_module._validate_release_matrix_profile(profile)
    assert {row['name'] for row in release['distros']} == {'humble', 'jazzy'}
    campaign = release['campaign_set']
    assert campaign['campaign_id'] == 'registration-plugin-release-matrix-current-source-2026-08-set1'
    assert campaign['rows'] == [
        {'distro': 'humble', 'dependency_leg': 'absent'},
        {'distro': 'humble', 'dependency_leg': 'present'},
        {'distro': 'jazzy', 'dependency_leg': 'absent'},
        {'distro': 'jazzy', 'dependency_leg': 'present'},
    ]
    for distro in ('humble', 'jazzy'):
        assert release_module.validate_release_plan(profile, distro, 'absent')['status'] == 'READY'
        assert release_module.validate_release_plan(
            profile, distro, 'absent')['campaign_set'] == campaign
        plan = release_module.validate_release_plan(profile, distro, 'present')
        assert plan['status'] == 'READY'
        assert plan['campaign_set'] == campaign
        assert [item['name'] for item in plan['leg']['dependencies']] == [
            'fast_gicp',
            'small_gicp',
        ]
        assert all(len(item['commit']) == 40 for item in plan['leg']['dependencies'])
        assert all(
            item['archive_url'].endswith(item['commit']) for item in plan['leg']['dependencies']
        )
        assert all(
            item['archive_sha256'] and item['license_sha256']
            for item in plan['leg']['dependencies']
        )


def test_runner_container_identity_requires_matching_ros_distro(
    audit_module, release_module, monkeypatch
):
    profile, _ = audit_module._load_profile(PROFILE)
    release = audit_module._validate_release_matrix_profile(profile)
    row = next(item for item in release['distros'] if item['name'] == 'jazzy')
    monkeypatch.setenv('REGISTRATION_PLUGIN_CONTAINER_DIGEST', row['image']['digest'])
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    with pytest.raises(release_module.ReleaseGateError, match='ROS distro'):
        release_module._container_identity(row)


def test_present_pin_policy_rejects_archive_drift_and_partial_pins(audit_module):
    profile, _ = audit_module._load_profile(PROFILE)
    broken = json.loads(json.dumps(profile))
    dependency = broken['release_matrix']['distros'][0]['legs']['present']['dependencies'][0]
    dependency['archive_url'] = dependency['archive_url'].replace(dependency['commit'], '0' * 40)
    with pytest.raises(audit_module.AuditError, match='archive URL/commit'):
        audit_module._validate_release_matrix_profile(broken)
    broken = json.loads(json.dumps(profile))
    dependency = broken['release_matrix']['distros'][0]['legs']['present']['dependencies'][0]
    dependency['archive_sha256'] = 'g' * 64
    with pytest.raises(audit_module.AuditError, match='not immutably pinned'):
        audit_module._validate_release_matrix_profile(broken)
    broken = json.loads(json.dumps(profile))
    dependency = broken['release_matrix']['distros'][0]['legs']['present']['dependencies'][0]
    dependency['official_url'] = 'https://example.invalid/latest.git'
    with pytest.raises(audit_module.AuditError, match='non-official'):
        audit_module._validate_release_matrix_profile(broken)
    broken = json.loads(json.dumps(profile))
    dependency = broken['release_matrix']['distros'][0]['legs']['present']['dependencies'][0]
    dependency['license_sha256'] = 'g' * 64
    with pytest.raises(audit_module.AuditError, match='immutably pinned'):
        audit_module._validate_release_matrix_profile(broken)


def test_blocked_present_leg_must_be_explicit_and_unpinned(audit_module):
    profile, _ = audit_module._load_profile(PROFILE)
    blocked = json.loads(json.dumps(profile))
    leg = blocked['release_matrix']['distros'][0]['legs']['present']
    leg['status'] = 'BLOCKED_MISSING_PIN'
    for dependency in leg['dependencies']:
        for key in (
            'commit',
            'source_sha256',
            'archive_url',
            'archive_sha256',
            'archive_top_level',
            'source_tree_sha256',
            'license',
            'license_path',
            'license_sha256',
            'compatibility',
        ):
            dependency[key] = None
    audit_module._validate_release_matrix_profile(blocked)
    malformed = json.loads(json.dumps(blocked))
    malformed['release_matrix']['distros'][0]['legs']['present']['dependencies'][0][
        'archive_sha256'
    ] = ('0' * 64)
    with pytest.raises(audit_module.AuditError, match='partial pin'):
        audit_module._validate_release_matrix_profile(malformed)


def test_functional_resource_policy_cannot_authorize_performance(audit_module):
    profile, _ = audit_module._load_profile(PROFILE)
    policy = profile['release_matrix']['resource_policy']
    assert policy['functional']['mode'] == 'functional_non_authoritative_v1'
    assert policy['functional']['timing_authority'] == 'NON_AUTHORITATIVE_CONTAMINATED'
    assert policy['functional']['performance_gate_eligible'] is False
    assert policy['performance']['max_cpu_busy_percent'] == 5.0
    broken = json.loads(json.dumps(profile))
    broken['release_matrix']['resource_policy']['functional']['performance_gate_eligible'] = True
    with pytest.raises(audit_module.AuditError, match='functional resource policy'):
        audit_module._validate_release_matrix_profile(broken)
    broken = json.loads(json.dumps(profile))
    broken['release_matrix']['resource_policy']['performance']['max_cpu_busy_percent'] = 6.0
    with pytest.raises(audit_module.AuditError, match='performance quiescence policy'):
        audit_module._validate_release_matrix_profile(broken)


def test_low_impact_worker_and_priority_contract(release_module):
    command = release_module._priority_argv(['colcon', 'build'])
    assert command[-2:] == ['colcon', 'build']
    assert '-n' in command and '19' in command
    assert '-c' in command and '3' in command


def test_git_trust_is_canonical_and_scoped_per_subprocess(audit_module, tmp_path, monkeypatch):
    repo = tmp_path / 'repo'
    repo.mkdir()
    alias = tmp_path / 'repo-alias'
    alias.symlink_to(repo, target_is_directory=True)

    argv = audit_module._git_argv(alias, 'status', '--porcelain=v1')
    canonical = str(repo.resolve())
    assert argv[:5] == ['git', '-c', 'safe.directory=' + canonical, '-C', canonical]
    assert '--global' not in argv
    assert 'safe.directory=*' not in argv

    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    monkeypatch.setattr(audit_module.subprocess, 'run', fake_run)
    audit_module._git_snapshot(alias)
    assert calls
    assert all(
        command[:5] == ['git', '-c', 'safe.directory=' + canonical, '-C', canonical]
        for command in calls
    )


@pytest.mark.parametrize('unsafe', ['repo\n-injected', 'repo\r-injected', 'repo\x00-injected'])
def test_git_trust_rejects_config_injection_paths(audit_module, tmp_path, unsafe):
    with pytest.raises(audit_module.AuditError, match='unsafe control characters'):
        audit_module._git_argv(str(tmp_path / unsafe), 'status')


def test_git_trust_rejects_missing_or_non_directory_root(audit_module, tmp_path):
    with pytest.raises(audit_module.AuditError, match='cannot be canonicalized'):
        audit_module._git_argv(tmp_path / 'missing', 'status')
    file_path = tmp_path / 'not-a-directory'
    file_path.write_text('x', encoding='utf-8')
    with pytest.raises(audit_module.AuditError, match='not a directory'):
        audit_module._git_argv(file_path, 'status')


def test_release_commands_cover_clean_build_consumer_dso_and_ros_rollback():
    text = RUNNER.read_text(encoding='utf-8')
    for marker in (
        'isolated_build',
        'cxx14_external_consumer',
        'cxx14_external_template',
        'external_dso_odr_load_gate',
        'ActivationRollbackRestoresPreviousPairAfterCommit',
        'RealRosResourceFailureRollsBackExternalCandidate',
        'provenance_pre_constructor',
        'installed_tree',
        'formal_replay_started',
    ):
        assert marker in text
    assert '--skip-load-smoke' not in text
    assert 'git clone' not in text
    assert 'git checkout' not in text
    assert 'git fetch' not in text
    for marker in (
        'curl',
        'tarfile',
        'DEPENDENCY_ARCHIVE_SHA_MISMATCH',
        'network_allowed=True',
        'NETWORK_AFTER_FETCH_FORBIDDEN',
        'license_sha256',
    ):
        assert marker in text
    assert 'subprocess.run(["docker"' not in text


def test_workflow_has_matrix_artifacts_and_archive_pin_policy():
    text = WORKFLOW.read_text(encoding='utf-8')
    assert 'registration-plugin-release-matrix' in text
    assert 'dependency_leg' in text
    assert 'REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY' in text
    assert 'forbidden_by_command_policy' in text
    assert 'retention-days: 14' in text
    assert 'registration-plugin-release-matrix-summary' in text
    assert 'continue-on-error: false' in text
    assert 'container: ros:humble-ros-core' not in text
    assert 'container: ros:jazzy-ros-core' not in text
    assert (
        'docker.io/library/ros:humble-ros-core@sha256:'
        'ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988'
        in text
    )
    assert (
        'docker.io/library/ros:jazzy-ros-base@sha256:'
        '31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f'
        in text
    )
    assert text.count('export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"') == 2


def test_archive_extraction_is_safe_and_commit_root_bound(release_module, tmp_path):
    archive_path = tmp_path / 'good.tar.gz'
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        root.mode = 0o755
        archive.addfile(root)
        payload = b'immutable source\n'
        item = tarfile.TarInfo('pkg-deadbeef/LICENSE')
        item.mode = 0o644
        item.size = len(payload)
        archive.addfile(item, io.BytesIO(payload))
    destination = tmp_path / 'extract'
    release_module.evidence_directories.create_fresh_root(destination)
    source = release_module._extract_verified_archive(
        archive_path, destination, 'pkg-deadbeef'
    )
    assert (source / 'LICENSE').read_bytes() == b'immutable source\n'

    bad_path = tmp_path / 'bad-path.tar.gz'
    with tarfile.open(str(bad_path), 'w:gz') as archive:
        item = tarfile.TarInfo('pkg-deadbeef/../escape')
        item.size = 0
        archive.addfile(item)
    bad_destination = tmp_path / 'bad-extract'
    release_module.evidence_directories.create_fresh_root(bad_destination)
    with pytest.raises(release_module.ReleaseGateError) as error:
        release_module._extract_verified_archive(
            bad_path, bad_destination, 'pkg-deadbeef'
        )
    assert error.value.kind == 'DEPENDENCY_ARCHIVE_PATH_INVALID'


def test_archive_extraction_rejects_absent_host_owned_destination(release_module, tmp_path):
    archive_path = tmp_path / 'missing-destination.tar.gz'
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        root.mode = 0o755
        archive.addfile(root)

    with pytest.raises(release_module.ReleaseGateError) as error:
        release_module._extract_verified_archive(
            archive_path, tmp_path / 'absent-extract', 'pkg-deadbeef'
        )
    assert error.value.kind == 'DIRECTORY_OPEN_FAILED'


def test_archive_extraction_uses_python311_safe_fallback(release_module, tmp_path, monkeypatch):
    archive_path = tmp_path / 'fallback.tar.gz'
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        root.mode = 0o755
        archive.addfile(root)
        payload = b'fallback source\n'
        item = tarfile.TarInfo('pkg-deadbeef/LICENSE')
        item.mode = 0o644
        item.size = len(payload)
        archive.addfile(item, io.BytesIO(payload))

    monkeypatch.setattr(
        release_module.inspect,
        'signature',
        lambda _extractall: SimpleNamespace(parameters={}),
    )
    destination = tmp_path / 'fallback-extract'
    release_module.evidence_directories.create_fresh_root(destination)
    source = release_module._extract_verified_archive(
        archive_path, destination, 'pkg-deadbeef'
    )
    assert (source / 'LICENSE').read_bytes() == b'fallback source\n'


def test_archive_metadata_is_sanitized_without_content_drift(release_module, tmp_path):
    archive_path = tmp_path / 'metadata.tar.gz'
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        root.mode = 0o755
        root.uid = 1234
        root.gid = 5678
        root.uname = 'untrusted'
        root.gname = 'untrusted'
        root.mtime = 1
        archive.addfile(root)
        payload = b'metadata-independent source\n'
        item = tarfile.TarInfo('pkg-deadbeef/LICENSE')
        item.mode = 0o644
        item.uid = 1234
        item.gid = 5678
        item.uname = 'untrusted'
        item.gname = 'untrusted'
        item.mtime = 1
        item.size = len(payload)
        archive.addfile(item, io.BytesIO(payload))
    destination = tmp_path / 'metadata-extract'
    release_module.evidence_directories.create_fresh_root(destination)
    source = release_module._extract_verified_archive(
        archive_path, destination, 'pkg-deadbeef'
    )
    license_path = source / 'LICENSE'
    assert license_path.read_bytes() == payload
    metadata = license_path.stat()
    assert stat.S_IMODE(metadata.st_mode) == 0o644
    assert metadata.st_uid == os.getuid()
    assert metadata.st_gid == os.getgid()


@pytest.mark.parametrize(
    ('member_name', 'member_type', 'expected_kind'),
    [
        ('/absolute', tarfile.REGTYPE, 'DEPENDENCY_ARCHIVE_PATH_INVALID'),
        ('pkg-deadbeef/./alias', tarfile.REGTYPE, 'DEPENDENCY_ARCHIVE_PATH_INVALID'),
        ('pkg-deadbeef/link', tarfile.SYMTYPE, 'DEPENDENCY_ARCHIVE_MEMBER_INVALID'),
        ('pkg-deadbeef/hardlink', tarfile.LNKTYPE, 'DEPENDENCY_ARCHIVE_MEMBER_INVALID'),
        ('pkg-deadbeef/device', tarfile.CHRTYPE, 'DEPENDENCY_ARCHIVE_MEMBER_INVALID'),
        ('pkg-deadbeef/fifo', tarfile.FIFOTYPE, 'DEPENDENCY_ARCHIVE_MEMBER_INVALID'),
        ('pkg-deadbeef/setuid', tarfile.REGTYPE, 'DEPENDENCY_ARCHIVE_METADATA_INVALID'),
    ],
)
def test_archive_rejects_unsafe_members_before_extraction(
    release_module, tmp_path, member_name, member_type, expected_kind
):
    archive_path = tmp_path / (expected_kind + '.tar.gz')
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        root.mode = 0o755
        archive.addfile(root)
        item = tarfile.TarInfo(member_name)
        item.type = member_type
        item.mode = 0o4755 if expected_kind == 'DEPENDENCY_ARCHIVE_METADATA_INVALID' else 0o644
        if member_type in (tarfile.SYMTYPE, tarfile.LNKTYPE):
            item.linkname = '../../outside'
        if member_type == tarfile.CHRTYPE:
            item.devmajor = 1
            item.devminor = 3
        archive.addfile(item)
    destination = tmp_path / 'extract'
    release_module.evidence_directories.create_fresh_root(destination)
    with pytest.raises(release_module.ReleaseGateError) as error:
        release_module._extract_verified_archive(
            archive_path, destination, 'pkg-deadbeef'
        )
    assert error.value.kind == expected_kind


def test_archive_rejects_duplicate_member_names(release_module, tmp_path):
    archive_path = tmp_path / 'duplicate.tar.gz'
    with tarfile.open(str(archive_path), 'w:gz') as archive:
        root = tarfile.TarInfo('pkg-deadbeef/')
        root.type = tarfile.DIRTYPE
        archive.addfile(root)
        for payload in (b'first', b'second'):
            item = tarfile.TarInfo('pkg-deadbeef/duplicate.txt')
            item.size = len(payload)
            archive.addfile(item, io.BytesIO(payload))
    destination = tmp_path / 'extract'
    release_module.evidence_directories.create_fresh_root(destination)
    with pytest.raises(release_module.ReleaseGateError) as error:
        release_module._extract_verified_archive(
            archive_path, destination, 'pkg-deadbeef'
        )
    assert error.value.kind == 'DEPENDENCY_ARCHIVE_MEMBER_DUPLICATE'


def test_network_capable_commands_are_only_allowed_for_explicit_fetch(release_module, tmp_path):
    with pytest.raises(release_module.ReleaseGateError, match='NETWORK_AFTER_FETCH_FORBIDDEN'):
        release_module._run_command(
            ['curl', 'https://example.invalid/archive'],
            'post_fetch',
            tmp_path,
            {},
            tmp_path / 'post-fetch.log',
            runner=lambda *_: SimpleNamespace(returncode=0),
        )
    record = release_module._run_command(
        ['curl', 'https://example.invalid/archive', '--output', str(tmp_path / 'archive')],
        'explicit_fetch',
        tmp_path,
        {},
        tmp_path / 'fetch.log',
        runner=lambda *_: SimpleNamespace(returncode=0, stdout='', stderr=''),
        network_allowed=True,
    )
    assert record['returncode'] == 0


def _fixture_tree(root, relative):
    directory = root / relative
    entries = []
    for item in sorted(
        directory.rglob('*'), key=lambda value: value.relative_to(directory).as_posix()
    ):
        path = item.relative_to(directory).as_posix()
        mode = stat.S_IMODE(os.lstat(str(item)).st_mode)
        if item.is_dir():
            entries.append({'kind': 'directory', 'mode': mode, 'path': path})
        else:
            payload = item.read_bytes()
            entries.append(
                {
                    'kind': 'file',
                    'mode': mode,
                    'path': path,
                    'size_bytes': len(payload),
                    'sha256': hashlib.sha256(payload).hexdigest(),
                }
            )
    return {
        'status': 'PASS',
        'root_relative': relative,
        'hash_kind': 'relative_lstat_tree_manifest_v1',
        'symlink_count': 0,
        'entry_count': len(entries),
        'entries': entries,
        'tree_sha256': hashlib.sha256(
            json.dumps(entries, sort_keys=True, separators=(',', ':')).encode('utf-8')
        ).hexdigest(),
    }


def _dependency_fixture_manifest(audit_module, root, distro, variant='1'):
    """Create a sealed synthetic in-container dependency identity.

    The fixture contains identities for container files rather than host files;
    the host summary must still reopen the receipt, sidecar, canonical hash,
    and per-distro absent/present projection before accepting it.
    """
    profile, _ = audit_module._load_profile(PROFILE)
    image = next(
        item['image'] for item in profile['release_matrix']['distros']
        if item['name'] == distro
    )
    contract = _load(DEPENDENCY_CONTRACT, 'dependency_contract_fixture')
    install_sha = hashlib.sha256(
        contract.DEPENDENCY_INSTALL_COMMAND.encode('utf-8')
    ).hexdigest()
    capture_sha = hashlib.sha256(DEPENDENCY_CAPTURE.read_bytes()).hexdigest()

    def section(rows, release_indexes=None, source_config=False):
        entries = []
        for relative, payload in rows:
            entries.append({
                'path': relative,
                'size_bytes': len(payload),
                'sha256': hashlib.sha256(payload).hexdigest(),
            })
        entries.sort(key=lambda item: item['path'])
        result = {
            'entries': entries,
            'entry_count': len(entries),
            'tree_sha256': hashlib.sha256(
                json.dumps(entries, sort_keys=True, separators=(',', ':')).encode('utf-8')
            ).hexdigest(),
        }
        if release_indexes is not None:
            result['release_indexes'] = [
                next(item for item in entries if item['path'] == relative)
                for relative in release_indexes
            ]
        if source_config:
            result['symlink_entries'] = []
            result['symlink_count'] = 0
            result['tree_sha256'] = hashlib.sha256(
                json.dumps(
                    {'entries': entries, 'symlink_entries': []},
                    sort_keys=True, separators=(',', ':')
                ).encode('utf-8')
            ).hexdigest()
        return result

    package_rows = [{
        'name': 'base-runtime',
        'version': '1.0-{}'.format(variant),
        'architecture': 'amd64',
        'status': 'install ok installed',
    }]
    package_hash = hashlib.sha256(
        json.dumps(package_rows, sort_keys=True, separators=(',', ':')).encode('utf-8')
    ).hexdigest()
    projection = {
        'base_image': {
            'digest': image['digest'],
            'os': 'linux',
            'architecture': 'amd64',
            'dpkg_architecture': 'amd64',
            'machine': 'x86_64',
        },
        'ros': {'distro': distro},
        'dpkg': {
            'packages': package_rows,
            'package_count': len(package_rows),
            'canonical_sha256': package_hash,
        },
        'apt': {
            'source_config': section([
                ('sources.list', b'deb fixture main\n'),
            ], source_config=True),
            'indexes': section([
                ('lists/fixture_InRelease', b'fixture release\n'),
                ('lists/fixture_Packages', b'fixture packages\n'),
            ], release_indexes=['lists/fixture_InRelease']),
        },
        'rosdep': {
            'sources': section([
                ('sources.list.d/20-default.list', b'fixture rosdep source\n'),
            ]),
            'cache': section([
                ('cache/index.yaml', b'fixture rosdep cache\n'),
            ]),
        },
        'install_command': {'sha256': install_sha, 'exit_code': 0},
        'capture': {'script_sha256': capture_sha},
        'secret_policy': {
            'environment_recorded': [],
            'credential_values_recorded': False,
            'proxy_values_recorded': False,
        },
    }
    manifest = {
        'schema': 'registration-plugin-dependency-environment-v1',
        'schema_version': 1,
        'status': 'PASS',
    }
    manifest.update(projection)
    manifest['canonical_sha256'] = hashlib.sha256(
        json.dumps(projection, sort_keys=True, separators=(',', ':')).encode('utf-8')
    ).hexdigest()
    receipt = root / 'dependency_environment.receipt.json'
    sealed = audit_module.seal_receipt(receipt, manifest)
    return {
        'path_relative': receipt.name,
        'sha256': sealed[2],
        'size_bytes': receipt.stat().st_size,
        'sidecar_sha256': audit_module.sha256_file(Path(sealed[1])),
        'canonical_sha256': manifest['canonical_sha256'],
        'capture_script_sha256': capture_sha,
    }


def _fake_pass_receipt(audit_module, path, distro, leg, source_sha, dependency_variant='1'):
    del source_sha
    profile, profile_path = audit_module._load_profile(PROFILE)
    source = audit_module._source_manifest(REPO, profile)
    host_root = path.parent / 'host-{}-{}'.format(distro, leg)
    host_root.mkdir()
    root = host_root / 'registration-plugin-release-{}-{}'.format(distro, leg)
    root.mkdir()
    work = root / 'work'
    for relative in ('logs', 'test-results', 'install', 'consumer', 'template', 'evidence'):
        (work / relative).mkdir(parents=True)
    (work / 'logs' / 'command.log').write_bytes(b'command passed\n')
    (work / 'test-results' / 'result.xml').write_bytes(b"<testsuite tests='1'/>\n")
    (work / 'install' / 'setup.bash').write_bytes(b'# isolated install\n')
    (work / 'consumer' / 'consumer.txt').write_bytes(b'consumer PASS\n')
    (work / 'template' / 'template.txt').write_bytes(b'template PASS\n')
    if leg == 'present':
        (work / 'archives').mkdir()
        (work / 'archives' / 'fast_gicp.tar.gz').write_bytes(b'archive\n')
        (work / 'archives' / 'small_gicp.tar.gz').write_bytes(b'archive\n')
        for name in ('fast_gicp', 'small_gicp'):
            (work / 'vendor' / (name + '-extract')).mkdir(parents=True)
            (work / 'vendor' / (name + '-extract') / 'source.txt').write_bytes(b'source\n')
            (work / 'vendor' / (name + '-install')).mkdir()
            (work / 'vendor' / (name + '-install') / 'lib.so').write_bytes(b'install\n')
        (root / 'prefetch' / 'archives').mkdir(parents=True)
        prefetch_records = []
        prefetch_manifest_entries = []
        for name in ('fast_gicp', 'small_gicp'):
            archive = root / 'prefetch' / 'archives' / (name + '.tar.gz')
            archive.write_bytes(b'archive\n')
            archive_sha = audit_module.sha256_file(archive)
            archive.chmod(0o444)
            archive_sidecar = Path(str(archive) + '.sha256')
            archive_sidecar.write_text('{}  {}\n'.format(archive_sha, archive.name), encoding='ascii')
            archive_sidecar.chmod(0o444)
            relative = 'prefetch/archives/' + archive.name
            prefetch_records.append({
                'name': name, 'archive_path_relative': relative,
                'size_bytes': archive.stat().st_size, 'archive_sha256': archive_sha,
            })
            prefetch_manifest_entries.append({
                'name': name, 'path_relative': relative,
                'sidecar_path_relative': relative + '.sha256',
                'size_bytes': archive.stat().st_size, 'sha256': archive_sha,
            })
        prefetch_manifest = root / 'dependency_prefetch_manifest.json'
        audit_module.seal_receipt(
            prefetch_manifest,
            {'schema': 'synthetic-prefetch-manifest-v1', 'entries': prefetch_manifest_entries},
        )
        prefetch_receipt = root / 'dependency_prefetch.receipt.json'
        audit_module.seal_receipt(
            prefetch_receipt,
            {'schema': 'synthetic-prefetch-receipt-v1', 'status': 'PASS',
             'records': prefetch_records},
        )
    (work / 'evidence' / 'odr-evidence').mkdir()
    (work / 'evidence' / 'odr-evidence' / 'marker.txt').write_bytes(b'odr\n')
    profile_sha = audit_module.sha256_file(profile_path)
    image = next(item['image'] for item in profile['release_matrix']['distros']
                 if item['name'] == distro)
    campaign_set = profile['release_matrix']['campaign_set']
    identity = audit_module.artifact_root_identity(
        logical_name=root.name,
        distro=distro,
        dependency_leg=leg,
        profile_sha256=profile_sha,
        source_manifest_sha256=source['manifest_sha256'],
    )
    audit_module.seal_receipt(root / 'evidence_root.receipt.json', identity)
    dependency_binding = _dependency_fixture_manifest(
        audit_module, root, distro, dependency_variant
    )
    odr_path = work / 'evidence' / 'registration_plugin_dso.receipt.json'
    audit_module.seal_receipt(
        odr_path,
        {
            'schema_version': 1,
            'kind': 'lidarslam-registration-plugin-dso-odr-gate',
            'status': 'PASS',
            'load_session_smoke': {'status': 'PASS'},
        },
    )
    provenance_path = work / 'evidence' / 'provenance.receipt.json'
    audit_module.seal_receipt(
        provenance_path,
        {
            'schema_version': 1,
            'contract_version': 'registration-plugin-provenance-evidence-v1',
            'status': 'PASS',
            'root_identity': identity,
        },
    )
    resource_path = work / 'evidence' / 'resource_failure.receipt.json'
    audit_module.seal_receipt(
        resource_path,
        {
            'schema_version': 1,
            'contract_version': 'registration-plugin-resource-failure-evidence-v1',
            'status': 'PASS',
            'root_identity': identity,
        },
    )
    required_roots = [
        {'path': 'work/logs', 'role': 'command_log'},
        {'path': 'work/test-results', 'role': 'test_result'},
        {'path': 'work/install', 'role': 'installed_tree_file'},
        {'path': 'work/consumer', 'role': 'consumer_artifact'},
        {'path': 'work/template', 'role': 'template_artifact'},
        {'path': 'work/evidence', 'role': 'evidence_artifact'},
    ]
    if leg == 'present':
        required_roots.append({'path': 'work/archives', 'role': 'dependency_archive'})
        for name in ('fast_gicp', 'small_gicp'):
            required_roots.extend(
                [
                    {
                        'path': 'work/vendor/{}-extract'.format(name),
                        'role': 'dependency_source_file',
                    },
                    {
                        'path': 'work/vendor/{}-install'.format(name),
                        'role': 'dependency_install_file',
                    },
                ]
            )
        explicit_prefetch = [
            {'path': 'dependency_prefetch.receipt.json', 'role': 'dependency_prefetch_receipt'},
            {'path': 'dependency_prefetch.receipt.json.sha256',
             'role': 'dependency_prefetch_receipt_sidecar'},
            {'path': 'dependency_prefetch_manifest.json', 'role': 'dependency_prefetch_manifest'},
            {'path': 'dependency_prefetch_manifest.json.sha256',
             'role': 'dependency_prefetch_manifest_sidecar'},
        ]
        for name in ('fast_gicp', 'small_gicp'):
            relative = 'prefetch/archives/' + name + '.tar.gz'
            explicit_prefetch.extend([
                {'path': relative, 'role': 'dependency_prefetch_archive'},
                {'path': relative + '.sha256', 'role': 'dependency_prefetch_archive_sidecar'},
            ])
    else:
        explicit_prefetch = []
    role_overrides = {
        'dependency_environment.receipt.json': 'dependency_environment_receipt',
        'dependency_environment.receipt.json.sha256': 'dependency_environment_receipt_sidecar',
        'work/evidence/registration_plugin_dso.receipt.json': 'dso_receipt',
        'work/evidence/registration_plugin_dso.receipt.json.sha256': 'dso_receipt_sidecar',
        'work/evidence/provenance.receipt.json': 'provenance_receipt',
        'work/evidence/provenance.receipt.json.sha256': 'provenance_receipt_sidecar',
        'work/evidence/resource_failure.receipt.json': 'resource_failure_receipt',
        'work/evidence/resource_failure.receipt.json.sha256': 'resource_failure_receipt_sidecar',
    }
    manifest = audit_module.build_artifact_manifest(
        root,
        identity,
        required_roots,
        [
            {'path': 'evidence_root.receipt.json', 'role': 'root_marker_receipt'},
            {'path': 'evidence_root.receipt.json.sha256', 'role': 'root_marker_receipt_sidecar'},
            {'path': 'dependency_environment.receipt.json', 'role': 'dependency_environment_receipt'},
            {'path': 'dependency_environment.receipt.json.sha256',
             'role': 'dependency_environment_receipt_sidecar'},
        ] + explicit_prefetch,
        role_overrides=role_overrides,
    )
    manifest_path = root / 'artifact_manifest.json'
    sealed_manifest = audit_module.seal_receipt(manifest_path, manifest)
    manifest_sidecar = Path(sealed_manifest[1])
    resource_snapshot = {
        'status': 'PASS',
        'checks': {
            'no_external_forbidden_processes': True,
            'docker_idle': True,
            'memory_safe': True,
            'disk_safe': True,
            'single_worker_policy': True,
        },
        'forbidden_processes': [],
        'docker': {'idle': True},
        'timing_authority': 'NON_AUTHORITATIVE_CONTAMINATED',
        'performance_gate_eligible': False,
    }
    value = {
        'schema_version': 1,
        'contract_version': 'registration-plugin-release-matrix-v1',
        'status': 'PASS',
        'distro': distro,
        'dependency_leg': leg,
        'campaign_set': campaign_set,
        'container': {
            'expected_image': image,
            'observed_digest_env': image['digest'],
            'ros_distro': distro,
            'platform': 'synthetic-linux-amd64',
            'source': 'workflow_pinned_container_digest_env',
        },
        'dependency_environment': dependency_binding,
        'dependency_prefetch': ({
            'status': 'PASS', 'authority': 'HOST_PROMOTION',
            'receipt_path_relative': 'dependency_prefetch.receipt.json',
            'receipt_sha256': audit_module.sha256_file(root / 'dependency_prefetch.receipt.json'),
            'receipt_sidecar_sha256': audit_module.sha256_file(
                root / 'dependency_prefetch.receipt.json.sha256'),
            'manifest_path_relative': 'dependency_prefetch_manifest.json',
            'manifest_sha256': audit_module.sha256_file(root / 'dependency_prefetch_manifest.json'),
            'manifest_sidecar_sha256': audit_module.sha256_file(
                root / 'dependency_prefetch_manifest.json.sha256'),
            'records': prefetch_records,
        } if leg == 'present' else {
            'status': 'NOT_APPLICABLE', 'archive_fetch_network_used': False,
        }),
        'execution': {
            'authority': 'HOST_PROMOTION', 'provisioning_network_used': True,
            'archive_fetch_network_used': leg == 'present',
            'build_test_network_connected': False, 'build_test_network_used': False,
            'network_after_dependency_fetch': False,
        },
        'profile': {'path': str(profile_path), 'sha256': profile_sha},
        'repository': {'source_manifest': source},
        'commands': {
            'records': [
                {'label': 'loader_transaction', 'log': {'path_relative': 'work/logs/command.log'}}
            ]
        },
        'installed_tree': _fixture_tree(root, 'work/install'),
        'test_results': _fixture_tree(root, 'work/test-results'),
        'external_dso': {
            'receipt_path_relative': 'work/evidence/registration_plugin_dso.receipt.json',
            'receipt_sha256': audit_module.sha256_file(odr_path),
            'sidecar_path_relative': 'work/evidence/registration_plugin_dso.receipt.json.sha256',
            'sidecar_sha256': audit_module.sha256_file(Path(str(odr_path) + '.sha256')),
            'status': 'PASS',
            'load_session_smoke': 'PASS',
        },
        'resource_policy': {
            'schema': 'registration-plugin-functional-resource-policy-v1',
            'schema_version': 1,
            'mode': 'functional_non_authoritative_v1',
            'preflight': resource_snapshot,
            'completion': resource_snapshot,
            'max_build_workers': 1,
            'priority': {'nice': 19, 'ionice_class': 3},
            'timing_authority': 'NON_AUTHORITATIVE_CONTAMINATED',
            'timing_metrics': {
                'status': 'NON_AUTHORITATIVE_CONTAMINATED',
                'performance_gate_eligible': False,
                'wall_seconds': None,
                'peak_rss_bytes': None,
                'rtf': None,
            },
            'performance_gate_eligible': False,
        },
        'performance_metrics': {
            'status': 'NON_AUTHORITATIVE_CONTAMINATED',
            'performance_gate_eligible': False,
            'wall_seconds': None,
            'peak_rss_bytes': None,
            'rtf': None,
        },
        'evidence_root': identity,
        'artifact_manifest': {
            'schema': audit_module.ARTIFACT_MANIFEST_SCHEMA,
            'schema_version': audit_module.ARTIFACT_MANIFEST_VERSION,
            'path_relative': 'artifact_manifest.json',
            'sha256': sealed_manifest[2],
            'size_bytes': manifest_path.stat().st_size,
            'sidecar_path_relative': 'artifact_manifest.json.sha256',
            'sidecar_sha256': audit_module.sha256_file(manifest_sidecar),
            'evidence_root': identity,
        },
        'safety': {
            'authority': 'HOST_PROMOTION',
            'provisioning_network_used': True,
            'archive_fetch_network_used': leg == 'present',
            'build_test_network_connected': False,
            'build_test_network_used': False,
            'bag_opened': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'map_saved': False,
            'formal_replay_started': False,
        },
    }
    receipt = root / 'registration_plugin_release.receipt.json'
    audit_module.seal_receipt(receipt, value)
    launcher_dir = host_root / 'launcher'
    launcher_dir.mkdir()
    image = next(item['image'] for item in profile['release_matrix']['distros']
                 if item['name'] == distro)
    container_name = 'fixture-container-{}-{}'.format(distro, leg)
    container_id = ('a' if distro == 'humble' else 'b') * 64
    dependency_contract = _load(DEPENDENCY_CONTRACT, 'dependency_contract_host_fixture')
    environment_values = {
        'ROS_DISTRO': distro,
        'REGISTRATION_PLUGIN_CONTAINER_DIGEST': image['digest'],
        'REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY': profile['release_matrix']['policy'][
            'network_after_dependency_fetch'],
        'REGISTRATION_PLUGIN_RESOURCE_POLICY': profile['release_matrix']['resource_policy'][
            'functional']['mode'],
        'CMAKE_BUILD_PARALLEL_LEVEL': '1', 'MAKEFLAGS': '-j1', 'NINJAFLAGS': '-j1',
    }
    capture_argv = [
        'docker', 'exec', container_name, 'python3',
        '/workspace/src/scripts/capture_registration_plugin_dependency_environment.py',
        '--output', '/workspace/evidence-parent/registration-plugin-release-{}-{}/{}'.format(
            distro, leg, 'dependency_environment.receipt.json'),
        '--distro', distro,
        '--image-digest', image['digest'],
        '--install-command-sha256', hashlib.sha256(
            dependency_contract.DEPENDENCY_INSTALL_COMMAND.encode('utf-8')).hexdigest(),
        '--install-exit-code', '0',
        '--capture-script-sha256', hashlib.sha256(DEPENDENCY_CAPTURE.read_bytes()).hexdigest(),
    ]
    runner_shell = (
        'source /opt/ros/{}/setup.bash && PYTHONPATH=/workspace/src:/workspace/src/scripts '
        'exec python3 /workspace/src/scripts/run_registration_plugin_release_matrix.py '
        '--repo-root /workspace/src '
        '--profile /workspace/src/configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json '
        '--distro {} --dependency-leg {} '
        '--evidence-root /workspace/evidence-parent/registration-plugin-release-{}-{} '
        '--output /workspace/evidence-parent/registration-plugin-release-{}-{}/registration_plugin_release.receipt.json'
    ).format(distro, distro, leg, distro, leg, distro, leg)
    runner_argv = ['docker', 'exec']
    for key in (
        'ROS_DISTRO', 'REGISTRATION_PLUGIN_CONTAINER_DIGEST',
        'REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY', 'REGISTRATION_PLUGIN_RESOURCE_POLICY',
        'CMAKE_BUILD_PARALLEL_LEVEL', 'MAKEFLAGS', 'NINJAFLAGS',
    ):
        runner_argv.extend(['--env', '{}={}'.format(key, environment_values[key])])
    for key, runner_value in (
        ('REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED', 'false'),
        ('REGISTRATION_PLUGIN_EXECUTION_AUTHORITY', 'HOST_PROMOTION'),
        ('REGISTRATION_PLUGIN_HOST_DISCONNECT_VERIFIED', 'true'),
    ):
        runner_argv.extend(['--env', '{}={}'.format(key, runner_value)])
    runner_argv.extend([container_name, 'bash', '-lc', runner_shell])
    host_logs = []

    def host_log(name, payload):
        log_path = launcher_dir / name
        log_path.write_bytes(payload)
        log_path.chmod(0o444)
        host_logs.append({
            'path': 'launcher/' + name,
            'sha256': audit_module.sha256_file(log_path),
            'bytes': log_path.stat().st_size,
        })
        return log_path

    def host_command(label, argv, log_name, payload=b'command passed\n', returncode=0):
        log_path = host_log(log_name, payload)
        return {
            'label': label,
            'argv': argv,
            'argv_sha256': hashlib.sha256(
                json.dumps(argv, separators=(',', ':')).encode('utf-8')
            ).hexdigest(),
            'effective_argv': argv,
            'priority': {'nice': 19, 'ionice_class': 3, 'applied': False},
            'returncode': returncode,
            'log': {
                'path': str(log_path),
                'bytes': log_path.stat().st_size,
                'sha256': audit_module.sha256_file(log_path),
            },
        }

    inspect_payload = json.dumps([{
        'Id': container_id,
        'Name': '/' + container_name,
        'Image': image['digest'],
        'Config': {'Image': image['reference']},
        'Mounts': [
            {'Type': 'bind', 'Source': str(REPO), 'Destination': '/workspace/src',
             'Mode': 'ro', 'RW': False},
            {'Type': 'bind', 'Source': str(host_root),
             'Destination': '/workspace/evidence-parent', 'Mode': 'rw', 'RW': True},
        ],
        'NetworkSettings': {'Networks': {}},
    }], sort_keys=True).encode('utf-8')
    image_inspect_payload = json.dumps([{
        'Id': image['digest'], 'RepoDigests': [image['reference']],
        'Os': 'linux', 'Architecture': 'amd64',
    }], sort_keys=True).encode('utf-8')
    host_commands = [
        host_command('docker_image_inspect',
                     ['docker', 'image', 'inspect', image['reference']],
                     'docker_image_inspect.log', image_inspect_payload),
        host_command('inspect_' + container_name,
                     ['docker', 'inspect', container_name],
                     'preexisting_container_inspect.log',
                     ('Error: No such object: {}\n'.format(container_name)).encode('utf-8'),
                     returncode=1),
        host_command('docker_run',
                     ['docker', 'run', '-d', '--name', container_name,
                      '--env', 'ROS_DISTRO={}'.format(environment_values['ROS_DISTRO']),
                      '--env', 'REGISTRATION_PLUGIN_CONTAINER_DIGEST={}'.format(
                          environment_values['REGISTRATION_PLUGIN_CONTAINER_DIGEST']),
                      '--env', 'REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY={}'.format(
                          environment_values['REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY']),
                      '--env', 'REGISTRATION_PLUGIN_RESOURCE_POLICY={}'.format(
                          environment_values['REGISTRATION_PLUGIN_RESOURCE_POLICY']),
                      '--env', 'CMAKE_BUILD_PARALLEL_LEVEL=1',
                      '--env', 'MAKEFLAGS=-j1', '--env', 'NINJAFLAGS=-j1',
                      '-v', '{}:/workspace/src:ro'.format(REPO),
                      '-v', '{}:/workspace/evidence-parent:rw'.format(host_root),
                      image['reference'], 'tail', '-f', '/dev/null'],
                     'docker_run.log'),
        host_command('dependency_install',
                     ['docker', 'exec', container_name, 'bash', '-lc',
                      dependency_contract.DEPENDENCY_INSTALL_COMMAND],
                     'dependency_install.log'),
        host_command('dependency_environment_capture',
                     capture_argv,
                     'dependency_environment_capture.log'),
    ]
    if leg == 'present':
        prefetch_command = host_command(
            'dependency_prefetch',
            ['python3', 'scripts/registration_plugin_dependency_prefetch.py',
             '--distro', distro, '--authority', 'HOST_PROMOTION'],
            'dependency_prefetch.log',
        )
        prefetch_command['script_sha256'] = audit_module.sha256_file(
            REPO / 'scripts/registration_plugin_dependency_prefetch.py')
        prefetch_command['network_phase'] = 'provisioning'
        host_commands.append(prefetch_command)
    host_commands.extend([
        host_command('network_disconnect_bridge',
                     ['docker', 'network', 'disconnect', 'bridge', container_name],
                     'network_disconnect_bridge.log'),
        host_command('inspect_' + container_name,
                     ['docker', 'inspect', container_name],
                     'post_disconnect_inspect.log', inspect_payload),
        host_command('release_runner', [
            *runner_argv,
        ], 'runner_exec.log'),
    ])
    cleanup_records = {
        'initial_inspect': host_command(
            'cleanup_initial_inspect', ['docker', 'inspect', container_name],
            'cleanup_inspect_initial.log', inspect_payload),
        'post_stop_inspect': host_command(
            'cleanup_post_stop_inspect', ['docker', 'inspect', container_name],
            'cleanup_inspect_post_stop.log', inspect_payload),
        'stop': host_command(
            'docker_stop', ['docker', 'stop', container_name], 'docker_stop.log'),
        'remove': host_command(
            'docker_remove', ['docker', 'rm', container_name], 'docker_remove.log'),
        'post_remove_inspect': host_command(
            'cleanup_post_remove_inspect', ['docker', 'inspect', container_name],
            'post_remove_inspect.log',
            ('Error: No such object: {}\n'.format(container_name)).encode('utf-8'),
            returncode=1),
    }
    host_storage = profile['release_matrix']['evidence_storage']
    host_report = {
        'schema': 'registration-plugin-host-release-leg-v1',
        'schema_version': 1,
        'status': 'PASS',
        'failure': None,
        'distro': distro,
        'dependency_leg': leg,
        'campaign_set': campaign_set,
        'profile': {'path': str(profile_path), 'sha256': profile_sha},
        'environment': {
            'required': [
                'ROS_DISTRO', 'REGISTRATION_PLUGIN_CONTAINER_DIGEST',
                'REGISTRATION_PLUGIN_DEPENDENCY_NETWORK_POLICY',
                'REGISTRATION_PLUGIN_RESOURCE_POLICY', 'CMAKE_BUILD_PARALLEL_LEVEL',
                'MAKEFLAGS', 'NINJAFLAGS',
            ],
            'values': environment_values,
        },
        'source': {
            'manifest_sha256': source['manifest_sha256'],
            'runner_sha256': audit_module.sha256_file(
                REPO / 'scripts/run_registration_plugin_release_matrix.py'),
            'launcher_sha256': audit_module.sha256_file(
                REPO / 'scripts/run_registration_plugin_release_leg.py'),
            'prefetch_sha256': audit_module.sha256_file(
                REPO / 'scripts/registration_plugin_dependency_prefetch.py'),
        },
        'image': {
            'id': image['digest'],
            'repo_digests': [image['reference']],
            'os': 'linux', 'architecture': 'amd64',
        },
        'container': {
            'name': container_name, 'id': container_id, 'start_count': 1,
            'image_reference': image['reference'],
            'mounts': [
                {'source': str(REPO), 'destination': '/workspace/src', 'mode': 'ro'},
                {'source': str(host_root), 'destination': '/workspace/evidence-parent',
                 'mode': 'rw'},
            ],
        },
        'commands': host_commands,
        'logs': host_logs,
        'cleanup': {
            'status': 'PASS', 'stop_requested': True, 'stopped': True,
            'remove_requested': True, 'remove_forced': False,
            'post_remove_absent': True, 'exit_code': 0,
            **cleanup_records,
        },
        'evidence_storage': {
            'contract': host_storage,
            'observed': {
                'mountpoint': host_storage['mountpoint'],
                'filesystem': host_storage['filesystem'],
                'source_uuid': host_storage['source_uuid'],
                'source': '/dev/fixture-evidence',
            },
        },
        'runner_receipt': value,
        'runner_receipt_binding': {
            'status': 'PASS',
            'path_relative': root.name + '/registration_plugin_release.receipt.json',
            'sha256': audit_module.sha256_file(receipt),
            'sidecar_path_relative': root.name + '/registration_plugin_release.receipt.json.sha256',
            'sidecar_sha256': audit_module.sha256_file(Path(str(receipt) + '.sha256')),
            'evidence_root': identity,
        },
        'artifact_revalidation': {
            'root': str(root),
            'manifest_sha256': sealed_manifest[2],
            'manifest_entry_count': manifest['entry_count'],
            'root_identity': identity,
        },
        'safety': {
            'provisioning_network_used': True,
            'archive_fetch_network_used': leg == 'present',
            'build_test_network_connected': False,
            'build_test_network_used': False,
            'network_after_dependency_fetch': False,
            'bag_opened': False, 'ground_truth_content_opened': False,
            'scorer_invoked': False, 'map_saved': False,
            'formal_replay_started': False,
        },
    }
    audit_module.seal_receipt(
        host_root / 'registration_plugin_host.receipt.json', host_report
    )
    return receipt


def _rewrite_sealed_fixture(path, value):
    payload = (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode('utf-8')
    sidecar = Path(str(path) + '.sha256')
    os.chmod(path, 0o644)
    path.write_bytes(payload)
    os.chmod(path, 0o444)
    os.chmod(sidecar, 0o644)
    sidecar.write_text('{}  {}\n'.format(hashlib.sha256(payload).hexdigest(), path.name),
                       encoding='ascii')
    os.chmod(sidecar, 0o444)


def test_summary_requires_all_four_pass_receipts(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary')
    source_sha = 'a' * 64
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            path = tmp_path / (distro + '-' + leg + '.receipt.json')
            receipts.append(_fake_pass_receipt(audit_module, path, distro, leg, source_sha))
    output = tmp_path / 'summary.receipt.json'
    result, report = summary_module.summarize(PROFILE, receipts, output)
    assert report['status'] == 'PASS'
    assert Path(result['path']).stat().st_mode & 0o777 == 0o444

    missing = receipts[:-1]
    with pytest.raises(summary_module.SummaryError, match='incomplete'):
        summary_module.summarize(PROFILE, missing, tmp_path / 'missing.receipt.json')


def test_summary_rejects_mixed_campaign_set_identity(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_mixed_campaign')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    inner_path = receipts[0]
    inner_value = json.loads(inner_path.read_text(encoding='utf-8'))
    alternate = json.loads(json.dumps(inner_value['campaign_set']))
    alternate['campaign_id'] = 'registration-plugin-release-matrix-other-set1'
    identity_payload = {key: alternate[key] for key in (
        'schema', 'schema_version', 'campaign_id', 'rows', 'row_set_sha256')}
    alternate['identity_sha256'] = hashlib.sha256(
        audit_module._canonical(identity_payload).encode('utf-8')).hexdigest()
    inner_value['campaign_set'] = alternate
    _rewrite_sealed_fixture(inner_path, inner_value)

    host_path = inner_path.parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_path.read_text(encoding='utf-8'))
    host_value['campaign_set'] = alternate
    host_value['runner_receipt'] = inner_value
    host_value['runner_receipt_binding']['sha256'] = audit_module.sha256_file(inner_path)
    host_value['runner_receipt_binding']['sidecar_sha256'] = audit_module.sha256_file(
        Path(str(inner_path) + '.sha256'))
    _rewrite_sealed_fixture(host_path, host_value)
    with pytest.raises(summary_module.SummaryError, match='campaign_set'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'mixed-campaign.json')


def test_summary_requires_outer_host_disconnect_receipt(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_outer_host')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module,
                tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_receipt.unlink()
    with pytest.raises(summary_module.SummaryError, match='host receipt|missing|regular file'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'missing-host.receipt.json')

    tamper_dir = tmp_path / 'tamper-host'
    tamper_dir.mkdir()
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module,
                tamper_dir / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    os.chmod(host_receipt, 0o644)
    host_receipt.write_bytes(host_receipt.read_bytes().replace(
        b'HOST_PROMOTION', b'FUNCTIONAL_CI_NON_PROMOTING', 1))
    os.chmod(host_receipt, 0o444)
    with pytest.raises(summary_module.SummaryError, match='sidecar|host receipt'):
        summary_module.summarize(PROFILE, receipts, tamper_dir / 'tampered-host.receipt.json')


def test_summary_rejects_reordered_outer_host_commands(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_reordered_host')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    host_value['commands'] = list(reversed(host_value['commands']))
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='order'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'reordered.json')


def test_summary_rejects_extra_docker_run_flag(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_extra_docker_run')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    docker_run = next(record for record in host_value['commands']
                      if record['label'] == 'docker_run')
    docker_run['argv'].insert(-2, '--privileged')
    docker_run['effective_argv'] = list(docker_run['argv'])
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='docker run'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'extra-docker-run.json')


def test_summary_rejects_wrong_dependency_shell(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_wrong_dependency_shell')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    dependency_install = next(record for record in host_value['commands']
                              if record['label'] == 'dependency_install')
    dependency_install['argv'][-1] = 'apt-get update'
    dependency_install['effective_argv'] = list(dependency_install['argv'])
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='dependency install'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'wrong-dependency-shell.json')


def test_summary_rejects_extra_runner_environment(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_extra_runner_environment')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    runner = next(record for record in host_value['commands']
                  if record['label'] == 'release_runner')
    runner['argv'].insert(2, '--env=UNTRUSTED=1')
    runner['effective_argv'] = list(runner['argv'])
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='release runner'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'extra-runner-environment.json')


def test_summary_rejects_image_self_report_drift(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_image_drift')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    host_value['image']['id'] = 'sha256:' + '0' * 64
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='image'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'image-drift.json')


def test_summary_rejects_swapped_host_mount_contract(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_mount_drift')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    host_value['container']['mounts'][0]['mode'] = 'rw'
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='mount'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'mount-drift.json')


def test_summary_requires_image_and_preexisting_inspects(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_image_preexisting_required')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    host_value['commands'] = [
        record for record in host_value['commands']
        if record['label'] != 'docker_image_inspect'
    ]
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='image inspect'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'missing-image.json')


def test_summary_rejects_cleanup_self_report_tamper(audit_module, tmp_path):
    summary_module = _load(SUMMARY, 'release_matrix_summary_cleanup_tamper')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    host_receipt = receipts[0].parent.parent / 'registration_plugin_host.receipt.json'
    host_value = json.loads(host_receipt.read_text(encoding='utf-8'))
    host_value['cleanup']['remove']['returncode'] = 1
    _rewrite_sealed_fixture(host_receipt, host_value)
    with pytest.raises(summary_module.SummaryError, match='cleanup'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'cleanup-tamper.json')


def test_summary_bounds_outer_receipt_and_inspect_logs(audit_module, tmp_path, monkeypatch):
    summary_module = _load(SUMMARY, 'release_matrix_summary_host_size')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    monkeypatch.setattr(summary_module, 'HOST_RECEIPT_MAX_BYTES', 1)
    with pytest.raises(summary_module.SummaryError, match='size'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'oversized-host.json')

    inspect_dir = tmp_path / 'inspect-size'
    inspect_dir.mkdir()
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module, inspect_dir / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    monkeypatch.setattr(summary_module, 'HOST_RECEIPT_MAX_BYTES', 16 * 1024 * 1024)
    monkeypatch.setattr(summary_module, 'HOST_LOG_MAX_BYTES', 1)
    with pytest.raises(summary_module.SummaryError, match='log'):
        summary_module.summarize(PROFILE, receipts, inspect_dir / 'oversized-log.json')


def test_summary_rejects_missing_or_tampered_dependency_environment(
    audit_module, tmp_path
):
    summary_module = _load(SUMMARY, 'release_matrix_summary_missing_dependency')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module,
                tmp_path / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    missing_root = receipts[0].parent
    (missing_root / 'dependency_environment.receipt.json').unlink()
    with pytest.raises(summary_module.SummaryError, match='missing'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'missing-dependency.json')

    tamper_dir = tmp_path / 'tamper'
    tamper_dir.mkdir()
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module,
                tamper_dir / '{}-{}.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
            ))
    tampered = receipts[0].parent / 'dependency_environment.receipt.json'
    os.chmod(tampered, 0o644)
    tampered.write_bytes(tampered.read_bytes().replace(b'base-runtime', b'changed-runtime'))
    with pytest.raises(
        summary_module.SummaryError,
        match='sidecar|dependency environment|artifact bytes',
    ):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'tampered-dependency.json')


@pytest.mark.parametrize('kind', [
    'empty_manifest', 'empty_sidecar', 'oversized_manifest', 'oversized_sidecar',
])
def test_dependency_environment_size_bounds_are_checked_before_read(
    audit_module, tmp_path, monkeypatch, kind
):
    summary_module = _load(SUMMARY, 'release_matrix_summary_dependency_size_' + kind)
    fixture_dir = tmp_path / kind
    fixture_dir.mkdir()
    receipt = _fake_pass_receipt(
        audit_module, fixture_dir / 'humble-absent.receipt.json',
        'humble', 'absent', 'a' * 64,
    )
    root = receipt.parent
    manifest = root / 'dependency_environment.receipt.json'
    sidecar = root / 'dependency_environment.receipt.json.sha256'
    if kind == 'empty_manifest':
        os.chmod(manifest, 0o644)
        manifest.write_bytes(b'')
        os.chmod(manifest, 0o444)
    elif kind == 'empty_sidecar':
        os.chmod(sidecar, 0o644)
        sidecar.write_bytes(b'')
        os.chmod(sidecar, 0o444)
    elif kind == 'oversized_manifest':
        monkeypatch.setattr(summary_module, 'DEPENDENCY_ENVIRONMENT_MAX_BYTES', 1)
    else:
        monkeypatch.setattr(summary_module, 'DEPENDENCY_ENVIRONMENT_SIDECAR_MAX_BYTES', 1)
    profile, _ = audit_module._load_profile(PROFILE)
    image = next(
        item['image'] for item in profile['release_matrix']['distros']
        if item['name'] == 'humble'
    )
    contract = _load(DEPENDENCY_CONTRACT, 'dependency_contract_size_' + kind)
    with pytest.raises(summary_module.SummaryError, match='size'):
        summary_module._validate_dependency_environment_manifest(
            root, 'dependency_environment.receipt.json', 'humble', image,
            hashlib.sha256(contract.DEPENDENCY_INSTALL_COMMAND.encode('utf-8')).hexdigest(),
            hashlib.sha256(DEPENDENCY_CAPTURE.read_bytes()).hexdigest(),
        )


def test_summary_rejects_base_dependency_closure_drift_between_legs(
    audit_module, tmp_path
):
    summary_module = _load(SUMMARY, 'release_matrix_summary_dependency_drift')
    receipts = []
    for distro in ('humble', 'jazzy'):
        for leg in ('absent', 'present'):
            receipts.append(_fake_pass_receipt(
                audit_module,
                tmp_path / '{}-{}-drift.receipt.json'.format(distro, leg),
                distro, leg, 'a' * 64,
                dependency_variant='2' if (distro, leg) == ('jazzy', 'present') else '1',
            ))
    with pytest.raises(summary_module.SummaryError, match='closure drift'):
        summary_module.summarize(PROFILE, receipts, tmp_path / 'drift.json')
