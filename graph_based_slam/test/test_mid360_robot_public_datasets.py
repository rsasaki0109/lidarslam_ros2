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

"""Tests for public MID-360 dataset intake helpers."""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import stat
import subprocess
import sys
import zipfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
DOWNLOAD_SCRIPT = SCRIPT_DIR / 'download_mid360_robot_public_dataset.py'
sys.path.insert(0, str(SCRIPT_DIR))

import mid360_robot_public_datasets as public_datasets  # noqa: E402,I100
from mid360_robot_public_datasets import (  # noqa: E402,I101
    PublicDataset,
    PublicDatasetExtractedFile,
    PublicDatasetFile,
    PublicDatasetIntake,
    PublicDatasetIntakeOptions,
    download_verified_artifact,
    get_public_dataset,
    public_dataset_registry,
)


class _Response(io.BytesIO):
    """Small context-managed HTTP response used by download tests."""

    def __init__(self, payload, *, status, headers):
        super().__init__(payload)
        self.status = status
        self.headers = headers

    def getcode(self):
        """Return the synthetic HTTP status."""
        return self.status


def _pinned_file(payload: bytes) -> PublicDatasetFile:
    return PublicDatasetFile(
        id='fixture',
        filename='fixture.zip',
        url='https://example.test/fixture.zip',
        md5=hashlib.md5(payload).hexdigest(),
        sha256=hashlib.sha256(payload).hexdigest(),
        size_bytes=len(payload),
    )


def _download_options(
    tmp_path: Path,
    *,
    force: bool = False,
    verify_md5: bool = True,
):
    return PublicDatasetIntakeOptions(
        dataset_id='fixture',
        dataset_root=tmp_path,
        force=force,
        extract=False,
        verify_md5=verify_md5,
    )


def test_stream_download_reports_periodic_progress(monkeypatch):
    """Progress includes periodic byte thresholds and final completion."""
    data = b'x' * (3 * 1024 * 1024)
    response = io.BytesIO(data)
    response.headers = {'Content-Length': str(len(data))}
    output = io.BytesIO()
    progress = io.StringIO()
    monkeypatch.setattr(public_datasets, 'DOWNLOAD_PROGRESS_BYTES', 1024 * 1024)

    intake = PublicDatasetIntake(REPO_ROOT, progress_stream=progress)
    intake._stream_download(
        PublicDatasetFile(
            id='fixture',
            filename='fixture.zip',
            url='file://fixture',
        ),
        response,
        output,
        intake._new_hashers(),
        total_bytes=len(data),
        expected_response_bytes=len(data),
    )

    lines = progress.getvalue().splitlines()
    assert lines[0] == 'Downloading fixture.zip: 0 B / 3.0 MiB (0.0%)'
    assert sum(line.startswith('Downloading fixture.zip:') for line in lines) == 4
    assert lines[-1].startswith('Downloaded fixture.zip: 3.0 MiB / 3.0 MiB (100.0%)')
    assert output.getvalue() == data


def test_stream_download_reports_when_time_interval_elapses(monkeypatch):
    """Progress also updates after the configured time interval."""
    data = b'x' * (2 * 1024 * 1024)
    response = io.BytesIO(data)
    response.headers = {}
    progress = io.StringIO()
    clock = iter((0.0, 6.0, 6.0, 6.0))
    monkeypatch.setattr(public_datasets, 'DOWNLOAD_PROGRESS_BYTES', len(data) * 2)
    monkeypatch.setattr(public_datasets.time, 'monotonic', lambda: next(clock))

    PublicDatasetIntake(REPO_ROOT, progress_stream=progress)._stream_download(
        PublicDatasetFile(id='fixture', filename='fixture.zip', url='file://fixture'),
        response,
        io.BytesIO(),
        PublicDatasetIntake._new_hashers(),
        expected_response_bytes=len(data),
    )

    lines = progress.getvalue().splitlines()
    assert 'Downloading fixture.zip: 1.0 MiB, 170.7 KiB/s' in lines
    assert lines[-1].startswith('Downloaded fixture.zip: 2.0 MiB, 341.3 KiB/s')


def test_download_resumes_exact_range_and_records_identity(
    tmp_path, monkeypatch
):
    """A valid 206 response appends only the missing suffix."""
    payload = b'0123456789abcdefghijklmnopqrstuvwxyz'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')
    prefix = payload[:11]
    part_path.write_bytes(prefix)

    def fake_urlopen(request):
        assert request.get_header('Range') == f'bytes={len(prefix)}-'
        assert request.get_header('Accept-encoding') == 'identity'
        suffix = payload[len(prefix):]
        return _Response(
            suffix,
            status=206,
            headers={
                'Content-Length': str(len(suffix)),
                'Content-Range': (
                    f'bytes {len(prefix)}-{len(payload) - 1}/'
                    f'{len(payload)}'
                ),
            },
        )

    monkeypatch.setattr(public_datasets.urllib.request, 'urlopen', fake_urlopen)
    messages = []
    result = PublicDatasetIntake(
        REPO_ROOT, progress_stream=None
    )._download(
        file_record,
        archive_path,
        _download_options(tmp_path),
        messages,
    )

    assert archive_path.read_bytes() == payload
    assert not part_path.exists()
    assert result['status'] == 'VERIFIED'
    assert result['source'] == 'resume'
    assert result['resumed_bytes'] == len(prefix)
    assert result['transferred_bytes'] == len(payload) - len(prefix)
    assert result['size_verified'] is True
    assert result['sha256_verified'] is True
    assert result['md5_verified'] is True
    assert messages[0].startswith('Resuming fixture.zip at byte 11')


def test_download_restarts_safely_when_server_ignores_range(
    tmp_path, monkeypatch
):
    """A 200 response replaces a partial file instead of concatenating."""
    payload = b'complete immutable payload'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')
    part_path.write_bytes(b'old-prefix')

    def fake_urlopen(request):
        assert request.get_header('Range') == 'bytes=10-'
        return _Response(
            payload,
            status=200,
            headers={'Content-Length': str(len(payload))},
        )

    monkeypatch.setattr(public_datasets.urllib.request, 'urlopen', fake_urlopen)
    messages = []
    result = PublicDatasetIntake(
        REPO_ROOT, progress_stream=None
    )._download(
        file_record,
        archive_path,
        _download_options(tmp_path),
        messages,
    )

    assert archive_path.read_bytes() == payload
    assert result['source'] == 'network'
    assert result['resumed_bytes'] == 0
    assert result['transferred_bytes'] == len(payload)
    assert any('safely restarting' in message for message in messages)


def test_download_rejects_wrong_content_range_without_changing_partial(
    tmp_path, monkeypatch
):
    """A server cannot redirect a resume request to a different offset."""
    payload = b'0123456789abcdef'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')
    prefix = payload[:5]
    part_path.write_bytes(prefix)

    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: _Response(
            payload[4:],
            status=206,
            headers={
                'Content-Length': str(len(payload) - 4),
                'Content-Range': f'bytes 4-15/{len(payload)}',
            },
        ),
    )

    with pytest.raises(ValueError, match='requested offset'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path),
            [],
        )

    assert part_path.read_bytes() == prefix
    assert not archive_path.exists()


def test_download_retains_but_never_finalizes_digest_mismatch(
    tmp_path, monkeypatch
):
    """Bad bytes remain a diagnostic partial and never become the archive."""
    payload = b'expected payload'
    changed = b'X' + payload[1:]
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')
    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: _Response(
            changed,
            status=200,
            headers={'Content-Length': str(len(changed))},
        ),
    )

    with pytest.raises(ValueError, match='SHA-256, MD5 mismatch'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path),
            [],
        )

    assert part_path.read_bytes() == changed
    assert not archive_path.exists()


def test_skip_md5_cannot_disable_registered_sha256(tmp_path, monkeypatch):
    """The legacy MD5 switch never weakens a registered SHA-256 pin."""
    payload = b'expected payload'
    changed = b'X' + payload[1:]
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: _Response(
            changed,
            status=200,
            headers={'Content-Length': str(len(changed))},
        ),
    )

    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path, verify_md5=False),
            [],
        )

    assert not archive_path.exists()


def test_interrupted_response_retains_resumable_prefix(tmp_path, monkeypatch):
    """A transport failure preserves received bytes only under .part."""
    payload = b'complete payload after retry'
    prefix = payload[:9]
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')

    class InterruptedResponse:
        status = 200
        headers = {'Content-Length': str(len(payload))}

        def __init__(self):
            self.calls = 0

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _size=-1):
            self.calls += 1
            if self.calls == 1:
                return prefix
            raise OSError('injected connection loss')

    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: InterruptedResponse(),
    )

    with pytest.raises(OSError, match='injected connection loss'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path),
            [],
        )

    assert part_path.read_bytes() == prefix
    assert not archive_path.exists()


def test_download_rejects_oversized_response_before_finalization(
    tmp_path, monkeypatch
):
    """Expected size bounds the stream even if the response sends more."""
    payload = b'expected payload'
    changed = payload + b'extra'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: _Response(
            changed,
            status=200,
            headers={},
        ),
    )

    with pytest.raises(ValueError, match='exceeded expected size'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path),
            [],
        )

    assert not archive_path.exists()


def test_complete_partial_is_verified_without_network(tmp_path, monkeypatch):
    """A complete interrupted file can finalize without another transfer."""
    payload = b'already complete payload'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    part_path = archive_path.with_suffix('.zip.part')
    part_path.write_bytes(payload)

    def fail_urlopen(_request):
        raise AssertionError('network must not be used')

    monkeypatch.setattr(public_datasets.urllib.request, 'urlopen', fail_urlopen)
    result = PublicDatasetIntake(
        REPO_ROOT, progress_stream=None
    )._download(
        file_record,
        archive_path,
        _download_options(tmp_path),
        [],
    )

    assert archive_path.read_bytes() == payload
    assert result['source'] == 'complete-part'
    assert result['resumed_bytes'] == len(payload)
    assert result['transferred_bytes'] == 0


def test_existing_archive_is_rehashed_and_tampering_fails(tmp_path):
    """An existing filename is not treated as proof of cache validity."""
    payload = b'expected cached payload'
    file_record = _pinned_file(payload)
    archive_path = tmp_path / file_record.filename
    archive_path.write_bytes(b'X' + payload[1:])

    with pytest.raises(ValueError, match='SHA-256, MD5 mismatch'):
        PublicDatasetIntake(
            REPO_ROOT, progress_stream=None
        )._download(
            file_record,
            archive_path,
            _download_options(tmp_path),
            [],
        )

    assert archive_path.read_bytes() != payload


def test_verified_artifact_wrapper_rehashes_a_pinned_cache(
    tmp_path,
    monkeypatch,
):
    """Publication tooling can reuse the hardened downloader without a registry."""
    payload = b'validated standalone artifact'
    file_record = _pinned_file(payload)
    destination = tmp_path / file_record.filename
    destination.write_bytes(payload)
    monkeypatch.setattr(
        public_datasets.urllib.request,
        'urlopen',
        lambda _request: pytest.fail('verified cache must not use network'),
    )

    report, messages = download_verified_artifact(
        file_record,
        destination,
        progress_stream=None,
    )

    assert report['status'] == 'VERIFIED'
    assert report['source'] == 'cache'
    assert report['sha256_verified'] is True
    assert messages[0].startswith('Verified existing archive')


def test_verified_artifact_wrapper_requires_size_and_sha256(tmp_path):
    """A host audit cannot fall back to filename-only or MD5-only trust."""
    file_record = PublicDatasetFile(
        id='fixture',
        filename='fixture.zip',
        url='https://example.test/fixture.zip',
        md5='0' * 32,
    )

    with pytest.raises(ValueError, match='require expected size and SHA-256'):
        download_verified_artifact(
            file_record,
            tmp_path / file_record.filename,
            progress_stream=None,
        )


def test_verified_artifact_wrapper_rejects_download_symlink(tmp_path):
    """A publication audit cannot be redirected through a final-path symlink."""
    payload = b'validated standalone artifact'
    file_record = _pinned_file(payload)
    target = tmp_path / 'outside.zip'
    target.write_bytes(payload)
    destination = tmp_path / file_record.filename
    destination.symlink_to(target)

    with pytest.raises(ValueError, match='must not be a symlink'):
        download_verified_artifact(
            file_record,
            destination,
            progress_stream=None,
        )

    assert target.read_bytes() == payload


def test_public_dataset_registry_contains_recommended_mid360_sources():
    datasets = {dataset.id: dataset for dataset in public_dataset_registry()}

    assert 'driving_slam_mid360' in datasets
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in datasets
    assert datasets['driving_slam_mid360'].file_by_id().md5 == (
        '0836c50859bb1af591966b69da166186'
    )
    assert datasets['driving_slam_mid360'].file_by_id().sha256 == (
        'f8f89eebf2aaf9cc1d465bfa5451bbb599cd92d079b59949104bb4e5cb619bdd'
    )
    assert (
        datasets['driving_slam_mid360'].file_by_id().size_bytes
        == 517_088_133
    )
    extracted = {
        item.path: item
        for item in datasets[
            'driving_slam_mid360'
        ].file_by_id().extracted_files
    }
    assert extracted[
        'rosbag2_2024_04_16-14_17_01/metadata.yaml'
    ].sha256 == (
        '65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a'
    )
    assert extracted[
        'rosbag2_2024_04_16-14_17_01/'
        'rosbag2_2024_04_16-14_17_01_0.db3'
    ].size_bytes == 1_468_932_096
    assert datasets['driving_slam_mid360'].license == (
        'Creative Commons Attribution 4.0 International.'
    )
    assert datasets['hard_pointcloud_mid360_outdoor_kidnap_a'].profile[
        'expected_pointcloud_topic'
    ] == '/livox/points'
    hard_files = {
        file_record.id
        for file_record in datasets['hard_pointcloud_mid360_outdoor_kidnap_a'].files
    }
    assert 'outdoor_kidnap_b' in hard_files
    assert 'outdoor_hard_01a' in hard_files


def test_get_public_dataset_rejects_unknown_id():
    try:
        get_public_dataset('missing_dataset')
    except ValueError as exc:
        assert 'unknown public MID-360 dataset' in str(exc)
    else:
        raise AssertionError('missing dataset id should raise ValueError')


def test_public_dataset_dry_run_writes_plan_without_archive(tmp_path: Path):
    intake = PublicDatasetIntake(REPO_ROOT)
    report = intake.run(
        PublicDatasetIntakeOptions(
            dataset_id='driving_slam_mid360',
            dataset_root=tmp_path / 'datasets',
            dry_run=True,
        )
    )

    assert report['status'] == 'DRY_RUN'
    assert 'download_mid360_robot_public_dataset.py' in report['commands']['download']
    assert report['commands']['recording_check'] == ''
    assert Path(report['manifest_json']).is_file()
    assert not Path(report['archive_path']).exists()


def test_public_dataset_intake_extracts_local_zip_and_finds_bag(tmp_path: Path):
    archive_src = tmp_path / 'source.zip'
    with zipfile.ZipFile(archive_src, 'w') as archive:
        archive.writestr(
            'bag/metadata.yaml',
            '\n'.join([
                'rosbag2_bagfile_information:',
                '  duration:',
                '    nanoseconds: 1000000000',
                '  message_count: 0',
                '  topics_with_message_count: []',
            ]),
        )
    md5 = hashlib.md5(archive_src.read_bytes()).hexdigest()
    dataset = PublicDataset(
        id='local_public_mid360',
        title='Local public MID-360 fixture',
        source_url='file://fixture',
        description='local fixture',
        license='test',
        citation='test',
        files=(
            PublicDatasetFile(
                id='fixture',
                filename='fixture.zip',
                url=archive_src.as_uri(),
                md5=md5,
                size_label='small',
            ),
        ),
        default_file_id='fixture',
        profile={
            'robot_name': 'local_public_mid360',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '',
            'expected_imu_topic': '/livox/imu',
        },
    )

    report = PublicDatasetIntake(
        REPO_ROOT,
        registry={'local_public_mid360': dataset},
    ).run(
        PublicDatasetIntakeOptions(
            dataset_id='local_public_mid360',
            dataset_root=tmp_path / 'datasets',
        )
    )

    assert report['status'] == 'READY'
    assert Path(report['archive_path']).is_file()
    assert Path(report['profile_path']).is_file()
    assert Path(report['selected_bag_path']).name == 'bag'
    assert 'check_mid360_robot_recording.sh' in report['commands']['recording_check']

    manifest = json.loads(Path(report['manifest_json']).read_text(encoding='utf-8'))
    assert manifest['selected_bag_path'] == report['selected_bag_path']
    assert manifest['download']['status'] == 'VERIFIED'
    assert manifest['download']['md5_verified'] is True
    assert manifest['download']['sha256'] == hashlib.sha256(
        archive_src.read_bytes()
    ).hexdigest()


def test_pinned_extraction_is_verified_and_cached_tampering_fails(
    tmp_path: Path,
):
    """A pinned archive name cannot bless changed extracted bag bytes."""
    metadata = b'rosbag2_bagfile_information: {}\n'
    storage = b'synthetic sqlite bytes\n'
    archive_src = tmp_path / 'source.zip'
    with zipfile.ZipFile(archive_src, 'w') as archive:
        archive.writestr('bag/metadata.yaml', metadata)
        archive.writestr('bag/bag_0.db3', storage)
    archive_bytes = archive_src.read_bytes()
    file_record = PublicDatasetFile(
        id='fixture',
        filename='fixture.zip',
        url=archive_src.as_uri(),
        md5=hashlib.md5(archive_bytes).hexdigest(),
        sha256=hashlib.sha256(archive_bytes).hexdigest(),
        size_bytes=len(archive_bytes),
        extracted_files=(
            PublicDatasetExtractedFile(
                path='bag/metadata.yaml',
                sha256=hashlib.sha256(metadata).hexdigest(),
                size_bytes=len(metadata),
            ),
            PublicDatasetExtractedFile(
                path='bag/bag_0.db3',
                sha256=hashlib.sha256(storage).hexdigest(),
                size_bytes=len(storage),
            ),
        ),
    )
    dataset = PublicDataset(
        id='pinned_fixture',
        title='Pinned fixture',
        source_url='file://fixture',
        description='test',
        license='test',
        citation='test',
        files=(file_record,),
        default_file_id='fixture',
        profile={},
    )
    intake = PublicDatasetIntake(
        REPO_ROOT,
        registry={'pinned_fixture': dataset},
        progress_stream=None,
    )
    options = PublicDatasetIntakeOptions(
        dataset_id='pinned_fixture',
        dataset_root=tmp_path / 'datasets',
    )

    first = intake.run(options)

    assert first['status'] == 'READY'
    assert first['download']['status'] == 'VERIFIED'
    assert first['extraction']['status'] == 'VERIFIED'
    assert first['extraction']['source'] == 'fresh'
    assert all(
        item['size_verified'] and item['sha256_verified']
        for item in first['extraction']['files']
    )

    extracted_metadata = Path(first['extract_dir']) / 'bag' / 'metadata.yaml'
    extracted_metadata.write_bytes(b'X' + metadata[1:])
    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        intake.run(options)

    assert Path(first['archive_path']).read_bytes() == archive_bytes


def test_extract_rejects_traversal_before_creating_output(tmp_path):
    """No archive member can escape the transaction directory."""
    archive_path = tmp_path / 'unsafe.zip'
    with zipfile.ZipFile(archive_path, 'w') as archive:
        archive.writestr('../escape', b'escape\n')
    extract_dir = tmp_path / 'extracted'

    with pytest.raises(ValueError, match='unsafe ZIP member path'):
        PublicDatasetIntake(REPO_ROOT)._extract(
            PublicDatasetFile(
                id='fixture',
                filename='unsafe.zip',
                url='file://fixture',
            ),
            archive_path,
            extract_dir,
            _download_options(tmp_path),
            [],
        )

    assert not extract_dir.exists()
    assert not (tmp_path / 'escape').exists()
    assert not (tmp_path / '.extracted.partial').exists()


def test_extract_rejects_zip_symlink(tmp_path):
    """A ZIP symlink is rejected instead of materialized or followed."""
    archive_path = tmp_path / 'symlink.zip'
    info = zipfile.ZipInfo('bag/link')
    info.create_system = 3
    info.external_attr = (stat.S_IFLNK | 0o777) << 16
    with zipfile.ZipFile(archive_path, 'w') as archive:
        archive.writestr(info, b'../../outside')

    with pytest.raises(ValueError, match='not a regular file'):
        PublicDatasetIntake(REPO_ROOT)._extract(
            PublicDatasetFile(
                id='fixture',
                filename='symlink.zip',
                url='file://fixture',
            ),
            archive_path,
            tmp_path / 'extracted',
            _download_options(tmp_path),
            [],
        )


def test_failed_forced_extract_preserves_previous_final(
    tmp_path, monkeypatch
):
    """An extraction failure cannot destroy a previously complete bag."""
    archive_path = tmp_path / 'valid.zip'
    with zipfile.ZipFile(archive_path, 'w') as archive:
        archive.writestr('bag/metadata.yaml', b'new metadata\n')
    extract_dir = tmp_path / 'extracted'
    extract_dir.mkdir()
    (extract_dir / 'owner.txt').write_text('previous\n', encoding='utf-8')

    def fail_copy(*_args, **_kwargs):
        raise OSError('injected extraction failure')

    monkeypatch.setattr(public_datasets.shutil, 'copyfileobj', fail_copy)
    with pytest.raises(ValueError, match='injected extraction failure'):
        PublicDatasetIntake(REPO_ROOT)._extract(
            PublicDatasetFile(
                id='fixture',
                filename='valid.zip',
                url='file://fixture',
            ),
            archive_path,
            extract_dir,
            _download_options(tmp_path, force=True),
            [],
        )

    assert (extract_dir / 'owner.txt').read_text(encoding='utf-8') == (
        'previous\n'
    )
    assert (tmp_path / '.extracted.partial').is_dir()
    assert not (tmp_path / '.extracted.previous').exists()


def test_successful_forced_extract_swaps_only_after_completion(tmp_path):
    """A complete transaction atomically replaces the previous directory."""
    archive_path = tmp_path / 'valid.zip'
    with zipfile.ZipFile(archive_path, 'w') as archive:
        archive.writestr('bag/metadata.yaml', b'new metadata\n')
    extract_dir = tmp_path / 'extracted'
    extract_dir.mkdir()
    (extract_dir / 'owner.txt').write_text('previous\n', encoding='utf-8')

    PublicDatasetIntake(REPO_ROOT)._extract(
        PublicDatasetFile(
            id='fixture',
            filename='valid.zip',
            url='file://fixture',
        ),
        archive_path,
        extract_dir,
        _download_options(tmp_path, force=True),
        [],
    )

    assert not (extract_dir / 'owner.txt').exists()
    assert (extract_dir / 'bag' / 'metadata.yaml').read_bytes() == (
        b'new metadata\n'
    )
    assert not (tmp_path / '.extracted.partial').exists()
    assert not (tmp_path / '.extracted.previous').exists()


def test_public_dataset_cli_list_json():
    result = subprocess.run(
        [sys.executable, str(DOWNLOAD_SCRIPT), '--list', '--json'],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    ids = {dataset['id'] for dataset in payload['datasets']}
    assert 'driving_slam_mid360' in ids
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in ids


def test_public_dataset_cli_dry_run(tmp_path: Path):
    result = subprocess.run(
        [
            sys.executable,
            str(DOWNLOAD_SCRIPT),
            '--dataset',
            'driving_slam_mid360',
            '--dataset-root',
            str(tmp_path / 'datasets'),
            '--dry-run',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'DRY_RUN'
    assert report['dataset']['id'] == 'driving_slam_mid360'
    assert Path(report['manifest_json']).is_file()
