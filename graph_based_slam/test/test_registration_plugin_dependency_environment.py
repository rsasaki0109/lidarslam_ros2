#!/usr/bin/env python3
"""Synthetic tests for the sealed dependency-environment capture utility."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


REPO = Path(__file__).resolve().parents[2]
CAPTURE = REPO / 'scripts/capture_registration_plugin_dependency_environment.py'
SUMMARY = REPO / 'scripts/summarize_registration_plugin_release_matrix.py'
SCHEMA = REPO / 'configs/slam_benchmark_profiles/' \
    'registration_plugin_dependency_environment_v1.schema.json'


def _load():
    spec = importlib.util.spec_from_file_location('dependency_environment_capture', str(CAPTURE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fixture_roots(tmp_path, *, ros2_link=False):
    filesystem_root = tmp_path / 'filesystem' if ros2_link else tmp_path
    apt = filesystem_root / 'etc' / 'apt' if ros2_link else tmp_path / 'apt'
    apt_lists = tmp_path / 'apt-lists'
    rosdep_sources = tmp_path / 'rosdep-sources'
    rosdep_cache = tmp_path / 'rosdep-cache'
    (apt / 'sources.list.d').mkdir(parents=True)
    apt.joinpath('sources.list').write_text('deb fixture main\n', encoding='utf-8')
    if ros2_link:
        target = filesystem_root / 'usr/share/ros-apt-source/ros2.sources'
        target.parent.mkdir(parents=True)
        target.write_text('Types: deb\nURIs: https://fixture.invalid\n', encoding='utf-8')
        target.chmod(0o644)
        (apt / 'sources.list.d' / 'ros2.sources').symlink_to(str(target))
    else:
        (apt / 'sources.list.d' / 'fixture.list').write_text(
            'deb fixture extra\n', encoding='utf-8')
    apt_lists.mkdir()
    (apt_lists / 'fixture_InRelease').write_text('release\n', encoding='utf-8')
    (apt_lists / 'fixture_Packages').write_text('packages\n', encoding='utf-8')
    rosdep_sources.mkdir()
    (rosdep_sources / '20-default.list').write_text('yaml fixture\n', encoding='utf-8')
    rosdep_cache.mkdir()
    (rosdep_cache / 'index.yaml').write_text('cache: fixture\n', encoding='utf-8')
    return apt, apt_lists, rosdep_sources, rosdep_cache


def _build(module, tmp_path, **kwargs):
    ros2_link = kwargs.pop('ros2_link', False)
    apt, apt_lists, rosdep_sources, rosdep_cache = _fixture_roots(
        tmp_path, ros2_link=ros2_link)
    values = {
        'distro': 'jazzy',
        'image_digest': 'sha256:' + 'a' * 64,
        'install_command_sha256': 'b' * 64,
        'install_exit_code': 0,
        'capture_script_sha256': 'c' * 64,
        'apt_root': apt,
        'apt_lists_root': apt_lists,
        'rosdep_sources_root': rosdep_sources,
        'rosdep_cache_roots': (('rosdep_cache/fixture', rosdep_cache),),
        'dpkg_query_runner': lambda query: (
            'base-runtime\t1.0-1\tamd64\tinstall ok installed\n'
        ),
        'architecture_runner': lambda: 'amd64',
    }
    if ros2_link:
        values['filesystem_root'] = tmp_path / 'filesystem'
    values.update(kwargs)
    return module.build_manifest(**values)


def _fixture_roots_with_ros2_link(tmp_path, *, target=None):
    filesystem_root = tmp_path / 'filesystem'
    apt = filesystem_root / 'etc' / 'apt'
    apt_lists = tmp_path / 'apt-lists'
    rosdep_sources = tmp_path / 'rosdep-sources'
    rosdep_cache = tmp_path / 'rosdep-cache'
    (apt / 'sources.list.d').mkdir(parents=True)
    apt.joinpath('sources.list').write_text('deb fixture main\n', encoding='utf-8')
    target_path = filesystem_root / 'usr/share/ros-apt-source/ros2.sources'
    target_path.parent.mkdir(parents=True)
    target_path.write_text('Types: deb\nURIs: https://fixture.invalid\n', encoding='utf-8')
    target_path.chmod(0o644)
    link = apt / 'sources.list.d' / 'ros2.sources'
    link.symlink_to(str(target if target is not None else target_path))
    apt_lists.mkdir()
    (apt_lists / 'fixture_InRelease').write_text('release\n', encoding='utf-8')
    (apt_lists / 'fixture_Packages').write_text('packages\n', encoding='utf-8')
    rosdep_sources.mkdir()
    (rosdep_sources / '20-default.list').write_text('yaml fixture\n', encoding='utf-8')
    rosdep_cache.mkdir()
    (rosdep_cache / 'index.yaml').write_text('cache: fixture\n', encoding='utf-8')
    return filesystem_root, apt, apt_lists, rosdep_sources, rosdep_cache


def _build_existing(module, root, **kwargs):
    """Build a manifest from an already-created synthetic filesystem."""
    ros2_link = kwargs.pop('ros2_link', True)
    assert ros2_link
    filesystem_root = root / 'filesystem'
    values = {
        'distro': 'jazzy',
        'image_digest': 'sha256:' + 'a' * 64,
        'install_command_sha256': 'b' * 64,
        'install_exit_code': 0,
        'capture_script_sha256': 'c' * 64,
        'apt_root': filesystem_root / 'etc/apt',
        'apt_lists_root': root / 'apt-lists',
        'rosdep_sources_root': root / 'rosdep-sources',
        'rosdep_cache_roots': (('rosdep_cache/fixture', root / 'rosdep-cache'),),
        'dpkg_query_runner': lambda query: (
            'base-runtime\t1.0-1\tamd64\tinstall ok installed\n'
        ),
        'architecture_runner': lambda: 'amd64',
        'filesystem_root': filesystem_root,
    }
    values.update(kwargs)
    return module.build_manifest(**values)


def test_capture_manifest_canonical_projection_is_reproducible(tmp_path):
    module = _load()
    manifest = _build(module, tmp_path)
    projection = {key: manifest[key] for key in (
        'base_image', 'ros', 'dpkg', 'apt', 'rosdep', 'install_command',
        'capture', 'secret_policy',
    )}
    expected = hashlib.sha256(
        json.dumps(projection, sort_keys=True, separators=(',', ':')).encode('utf-8')
    ).hexdigest()
    assert manifest['canonical_sha256'] == expected
    assert manifest['secret_policy'] == {
        'environment_recorded': [],
        'credential_values_recorded': False,
        'proxy_values_recorded': False,
    }


def test_capture_source_never_serializes_environment_values():
    source = CAPTURE.read_text(encoding='utf-8')
    assert 'os.environ' not in source
    assert 'credential_values_recorded' in source
    assert 'proxy_values_recorded' in source


def test_capture_requires_prefixed_image_digest(tmp_path):
    module = _load()
    with pytest.raises(module.CaptureError, match='image digest'):
        _build(module, tmp_path, image_digest='a' * 64)


def test_capture_manifest_seal_is_exclusive_and_sidecar_bound(tmp_path):
    module = _load()
    manifest = _build(module, tmp_path)
    output = tmp_path / 'dependency_environment.receipt.json'
    sealed = module.seal_manifest(output, manifest)
    assert output.stat().st_mode & 0o777 == 0o444
    assert Path(sealed['sidecar']).read_text(encoding='ascii').split() == [
        sealed['sha256'], output.name,
    ]
    with pytest.raises(module.CaptureError, match='fresh'):
        module.seal_manifest(output, manifest)


def test_capture_rejects_symlink_and_nonzero_install(tmp_path):
    module = _load()
    apt, apt_lists, rosdep_sources, rosdep_cache = _fixture_roots(tmp_path)
    (apt / 'sources.list.d' / 'unsafe.list').symlink_to(apt / 'sources.list')
    with pytest.raises(module.CaptureError, match='symlink|regular'):
        module.build_manifest(
            'jazzy', 'sha256:' + 'a' * 64, 'b' * 64, 0, 'c' * 64,
            apt_root=apt, apt_lists_root=apt_lists,
            rosdep_sources_root=rosdep_sources,
            rosdep_cache_roots=(('rosdep_cache/fixture', rosdep_cache),),
            dpkg_query_runner=lambda query: 'base\t1\tamd64\tinstall ok installed\n',
            architecture_runner=lambda: 'amd64',
        )
    with pytest.raises(module.CaptureError, match='did not exit zero'):
        _build(module, tmp_path / 'nonzero', install_exit_code=1)


@pytest.mark.parametrize('distro', ['humble', 'jazzy'])
def test_capture_allows_only_fixed_ros2_source_symlink(tmp_path, distro):
    module = _load()
    manifest = _build(module, tmp_path, ros2_link=True, distro=distro)
    source = manifest['apt']['source_config']
    assert source['symlink_count'] == 1
    assert source['entries'] == [
        {
            'path': 'apt_sources/sources.list',
            'size_bytes': len(b'deb fixture main\n'),
            'sha256': hashlib.sha256(b'deb fixture main\n').hexdigest(),
        }
    ]
    link = source['symlink_entries'][0]
    target = tmp_path / 'filesystem/usr/share/ros-apt-source/ros2.sources'
    assert link['path'] == 'apt_sources/sources.list.d/ros2.sources'
    assert link['target'] == str(target)
    assert link['target_file']['path'] == str(target)
    assert link['target_file']['mode'] == 0o644
    assert link['target_file']['nlink'] == 1


def test_capture_and_summary_allow_source_directory_with_only_fixed_symlink(tmp_path):
    module = _load()
    root = tmp_path / 'only-link'
    _build(module, root, ros2_link=True)
    (root / 'filesystem/etc/apt/sources.list').unlink()
    manifest = _build_existing(module, root)
    source = json.loads(json.dumps(manifest['apt']['source_config']))
    assert source['entries'] == []
    assert source['symlink_count'] == 1
    link = source['symlink_entries'][0]
    target = '/usr/share/ros-apt-source/ros2.sources'
    link.update({'target': target, 'uid': 0, 'gid': 0})
    link['size_bytes'] = len(target.encode('utf-8'))
    link['target_file'].update({'path': target, 'uid': 0, 'gid': 0})
    source['tree_sha256'] = hashlib.sha256(json.dumps(
        {'entries': source['entries'], 'symlink_entries': source['symlink_entries']},
        sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
    summary_spec = importlib.util.spec_from_file_location(
        'release_matrix_summary_only_link', str(SUMMARY))
    summary = importlib.util.module_from_spec(summary_spec)
    summary_spec.loader.exec_module(summary)
    validated = summary._dependency_file_section(
        source, 'apt source config', source_config=True)
    assert validated['entry_count'] == 0


def test_dependency_environment_schema_accepts_capture_projection(tmp_path):
    module = _load()
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    Draft202012Validator.check_schema(schema)
    manifest = _build(module, tmp_path)
    Draft202012Validator(schema).validate(manifest)


@pytest.mark.parametrize('target_kind', ['wrong', 'relative'])
def test_capture_rejects_wrong_ros2_source_target(tmp_path, target_kind):
    module = _load()
    manifest_root = tmp_path / target_kind
    _build(module, manifest_root, ros2_link=True)
    link = manifest_root / 'filesystem/etc/apt/sources.list.d/ros2.sources'
    link.unlink()
    target = ('/usr/share/ros-apt-source/wrong.sources' if target_kind == 'wrong'
              else 'ros2.sources')
    link.symlink_to(target)
    with pytest.raises(module.CaptureError, match='fixed absolute target'):
        _build_existing(module, manifest_root)


def test_capture_rejects_target_symlink_parent_and_hardlink(tmp_path):
    module = _load()
    root = tmp_path / 'target-symlink'
    _build(module, root, ros2_link=True)
    target = root / 'filesystem/usr/share/ros-apt-source/ros2.sources'
    target.unlink()
    real_target = target.with_name('real.sources')
    real_target.write_text('target\n', encoding='utf-8')
    real_target.chmod(0o644)
    target.symlink_to(real_target.name)
    with pytest.raises(module.CaptureError, match='without following|regular'):
        _build_existing(module, root)

    root = tmp_path / 'parent-symlink'
    _build(module, root, ros2_link=True)
    share = root / 'filesystem/usr/share'
    real_share = root / 'filesystem/usr/share-real'
    share.rename(real_share)
    share.symlink_to(real_share.name, target_is_directory=True)
    with pytest.raises(module.CaptureError, match='parent'):
        _build_existing(module, root)

    root = tmp_path / 'target-hardlink'
    _build(module, root, ros2_link=True)
    target = root / 'filesystem/usr/share/ros-apt-source/ros2.sources'
    target.with_name('second.sources').hardlink_to(target)
    with pytest.raises(module.CaptureError, match='single-link'):
        _build_existing(module, root)


def test_capture_rejects_target_content_mutation_between_hashes(tmp_path, monkeypatch):
    module = _load()
    root = tmp_path / 'mutating-target'
    original = module._digest_fd
    calls = []

    def mutate_after_first_hash(fd, size, label):
        result = original(fd, size, label)
        calls.append(result)
        if len(calls) == 1:
            target = Path(os.readlink('/proc/self/fd/{}'.format(fd)))
            target.write_text('changed target bytes\n', encoding='utf-8')
            target.chmod(0o644)
        return result

    _build(module, root / 'initial', ros2_link=True)
    monkeypatch.setattr(module, '_digest_fd', mutate_after_first_hash)
    with pytest.raises(module.CaptureError, match='content changed|metadata changed|size changed'):
        _build(module, root / 'second', ros2_link=True)


def test_summary_validates_ros2_symlink_descriptor_separately(tmp_path):
    module = _load()
    summary_spec = importlib.util.spec_from_file_location('release_matrix_summary', str(SUMMARY))
    summary = importlib.util.module_from_spec(summary_spec)
    summary_spec.loader.exec_module(summary)
    source = _build(module, tmp_path)['apt']['source_config']
    target = '/usr/share/ros-apt-source/ros2.sources'
    target_file = {
        'path': target,
        'size_bytes': 12,
        'sha256': 'a' * 64,
        'mode': 0o644,
        'uid': 0,
        'gid': 0,
        'nlink': 1,
    }
    source['symlink_entries'] = [{
        'path': 'apt_sources/sources.list.d/ros2.sources',
        'target': target,
        'size_bytes': len(target.encode('utf-8')),
        'mode': 0o777,
        'uid': 0,
        'gid': 0,
        'nlink': 1,
        'target_file': target_file,
    }]
    source['symlink_count'] = 1
    source['tree_sha256'] = hashlib.sha256(json.dumps(
        {'entries': source['entries'], 'symlink_entries': source['symlink_entries']},
        sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
    validated = summary._dependency_file_section(
        source, 'apt source config', source_config=True)
    assert validated['symlink_count'] == 1
    source['symlink_entries'][0]['target'] = '/tmp/other.sources'
    with pytest.raises(summary.SummaryError, match='target identity'):
        summary._dependency_file_section(source, 'apt source config', source_config=True)
