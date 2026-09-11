#!/usr/bin/env python3
"""Public MID-360 dataset registry and intake helpers."""

from __future__ import annotations

import hashlib
import json
import re
import shlex
import shutil
import stat
import sys
import time
import urllib.error
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO, TextIO

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_DATASET_INTAKE_JSON = 'mid360_robot_public_dataset_intake.json'
PUBLIC_DATASET_INTAKE_MARKDOWN = 'mid360_robot_public_dataset_intake.md'
DOWNLOAD_CHUNK_BYTES = 1024 * 1024
DOWNLOAD_PROGRESS_BYTES = 32 * 1024 * 1024
DOWNLOAD_PROGRESS_INTERVAL_SEC = 5.0
DOWNLOAD_USER_AGENT = 'lidar_slam_ros2-public-dataset-intake/1'
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
MD5_PATTERN = re.compile(r'^[0-9a-f]{32}$')
CONTENT_RANGE_PATTERN = re.compile(
    r'^bytes (?P<start>[0-9]+)-(?P<end>[0-9]+)/(?P<total>[0-9]+)$'
)


def payload_to_json(payload: dict[str, Any]) -> str:
    """Serialize without importing source-only robot tooling."""
    return json.dumps(payload, indent=2, sort_keys=True)


@dataclass(frozen=True)
class PublicDatasetExtractedFile:
    """Immutable identity of one security-relevant extracted file."""

    path: str
    sha256: str
    size_bytes: int

    def __post_init__(self) -> None:
        """Reject paths or identities that cannot be checked safely."""
        relative = PurePosixPath(self.path)
        if (
            not self.path
            or self.path.startswith('/')
            or '\\' in self.path
            or '\x00' in self.path
            or relative.as_posix() != self.path
            or any(
                part in ('', '.', '..')
                for part in self.path.split('/')
            )
        ):
            raise ValueError(
                f'invalid extracted file path: {self.path!r}'
            )
        if SHA256_PATTERN.fullmatch(self.sha256.lower()) is None:
            raise ValueError(
                f'invalid extracted file SHA-256 for {self.path}: '
                f'{self.sha256!r}'
            )
        if self.size_bytes <= 0:
            raise ValueError(
                f'invalid extracted file size for {self.path}: '
                f'{self.size_bytes}'
            )


@dataclass(frozen=True)
class PublicDatasetFile:
    """Downloadable file in a public MID-360 dataset."""

    id: str
    filename: str
    url: str
    md5: str = ''
    sha256: str = ''
    size_bytes: int | None = None
    size_label: str = ''
    archive_format: str = 'zip'
    notes: str = ''
    extracted_files: tuple[PublicDatasetExtractedFile, ...] = ()

    def __post_init__(self) -> None:
        """Reject malformed immutable identities at registry load time."""
        if self.md5 and MD5_PATTERN.fullmatch(self.md5.lower()) is None:
            raise ValueError(f'invalid MD5 for {self.id}: {self.md5!r}')
        if (
            self.sha256
            and SHA256_PATTERN.fullmatch(self.sha256.lower()) is None
        ):
            raise ValueError(
                f'invalid SHA-256 for {self.id}: {self.sha256!r}'
            )
        if self.size_bytes is not None and self.size_bytes <= 0:
            raise ValueError(
                f'invalid expected size for {self.id}: {self.size_bytes}'
            )
        if self.extracted_files and (
            not self.sha256 or self.size_bytes is None
        ):
            raise ValueError(
                f'extracted file identities for {self.id} require a '
                'size- and SHA-256-pinned archive'
            )
        extracted_paths = [item.path for item in self.extracted_files]
        if len(extracted_paths) != len(set(extracted_paths)):
            raise ValueError(
                f'duplicate extracted file identity for {self.id}'
            )

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible file record."""
        return asdict(self)


@dataclass(frozen=True)
class PublicDataset:
    """Public MID-360 dataset entry."""

    id: str
    title: str
    source_url: str
    description: str
    license: str
    citation: str
    files: tuple[PublicDatasetFile, ...]
    default_file_id: str
    profile: dict[str, Any]
    notes: tuple[str, ...] = ()
    supports_recording_check: bool = True

    def file_by_id(self, file_id: str = '') -> PublicDatasetFile:
        selected = file_id or self.default_file_id
        for file_record in self.files:
            if file_record.id == selected:
                return file_record
        valid = ', '.join(file_record.id for file_record in self.files)
        raise ValueError(f'unknown dataset file {selected!r}; valid files: {valid}')

    def to_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'title': self.title,
            'source_url': self.source_url,
            'description': self.description,
            'license': self.license,
            'citation': self.citation,
            'files': [file_record.to_dict() for file_record in self.files],
            'default_file_id': self.default_file_id,
            'profile': self.profile,
            'notes': list(self.notes),
            'supports_recording_check': self.supports_recording_check,
        }


@dataclass(frozen=True)
class PublicDatasetIntakeOptions:
    """Options for one public dataset intake run."""

    dataset_id: str
    dataset_root: Path
    file_id: str = ''
    output_dir: Path | None = None
    dry_run: bool = False
    force: bool = False
    extract: bool = True
    verify_md5: bool = True


PUBLIC_MID360_DATASETS: dict[str, PublicDataset] = {
    'driving_slam_mid360': PublicDataset(
        id='driving_slam_mid360',
        title='Driving SLAM Test with Livox MID360',
        source_url='https://zenodo.org/records/14841855',
        description='Small ROS2 bag for LiDAR-IMU SLAM testing with a Livox MID-360.',
        license='Creative Commons Attribution 4.0 International.',
        citation='Koide, Kenji. Driving SLAM Test with Livox MID360. Zenodo. DOI: 10.5281/zenodo.14841855',
        files=(
            PublicDatasetFile(
                id='rosbag2_2024_04_16',
                filename='rosbag2_2024_04_16-14_17_01.zip',
                url='https://zenodo.org/records/14841855/files/rosbag2_2024_04_16-14_17_01.zip?download=1',
                md5='0836c50859bb1af591966b69da166186',
                sha256=(
                    'f8f89eebf2aaf9cc1d465bfa5451bbb5'
                    '99cd92d079b59949104bb4e5cb619bdd'
                ),
                size_bytes=517088133,
                size_label='517.1 MB',
                notes='Recommended first public MID-360 ROS2 intake target.',
                extracted_files=(
                    PublicDatasetExtractedFile(
                        path=(
                            'rosbag2_2024_04_16-14_17_01/'
                            'metadata.yaml'
                        ),
                        sha256=(
                            '65d66875f49248e38ff14d80e6e749fb'
                            '50606f6f80bd4be337160e3752691e9a'
                        ),
                        size_bytes=5590,
                    ),
                    PublicDatasetExtractedFile(
                        path=(
                            'rosbag2_2024_04_16-14_17_01/'
                            'rosbag2_2024_04_16-14_17_01_0.db3'
                        ),
                        sha256=(
                            '3bbd390a97e57af47ad6699baa36eb4c'
                            '5f39f61b35275505ecaf221c126354f5'
                        ),
                        size_bytes=1468932096,
                    ),
                ),
            ),
        ),
        default_file_id='rosbag2_2024_04_16',
        profile={
            'robot_name': 'livox_mid360_public_driving_slam',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            # Leave point cloud auto-selected because public MID-360 bags differ
            # between /livox/lidar and /livox/points for PointCloud2.
            'expected_pointcloud_topic': '',
            'expected_imu_topic': '/livox/imu',
            'mount': {
                'xyz': [0.0, 0.0, 0.0],
                'q_xyzw': [0.0, 0.0, 0.0, 1.0],
                'note': 'Public-data profile; replace frames/extrinsics for robot data.',
            },
        },
        notes=(
            'Use this first because it is much smaller than the multi-sequence datasets.',
            'Run the generated recording-check command after extraction.',
        ),
    ),
    'hard_pointcloud_mid360_outdoor_kidnap_a': PublicDataset(
        id='hard_pointcloud_mid360_outdoor_kidnap_a',
        title='Hard Point Cloud Localization Dataset - outdoor_kidnap_a',
        source_url='https://zenodo.org/records/10122133',
        description='Outdoor Livox MID360 ROS2 bag with aggressive localization failure cases.',
        license='Creative Commons Attribution 4.0 International.',
        citation='Koide, Kenji. Hard Point Cloud Localization Dataset. Zenodo. DOI: 10.5281/zenodo.10122133',
        files=(
            PublicDatasetFile(
                id='outdoor_kidnap_a',
                filename='outdoor_kidnap_a.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_kidnap_a.zip?download=1',
                md5='3c6941b8c70ca41c79dae83758632625',
                size_label='650.9 MB',
                notes='Smaller outdoor MID360 kidnap sequence from the hard localization set.',
            ),
            PublicDatasetFile(
                id='outdoor_kidnap_b',
                filename='outdoor_kidnap_b.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_kidnap_b.zip?download=1',
                md5='893ed4a732c5c0c9dc38d069e7056a69',
                size_label='1.3 GB',
                notes='Second half of outdoor_kidnap; needed with outdoor_kidnap_a for loop evaluation.',
            ),
            PublicDatasetFile(
                id='outdoor_hard_01a',
                filename='outdoor_hard_01a.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_hard_01a.zip?download=1',
                md5='d126024c8f310c2c48239f136d7e0ed0',
                size_label='1.6 GB',
                notes='First half of outdoor_hard_01.',
            ),
            PublicDatasetFile(
                id='outdoor_hard_01b',
                filename='outdoor_hard_01b.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_hard_01b.zip?download=1',
                md5='d397c28cca763d34844189cb73de8e02',
                size_label='1.3 GB',
                notes='Second half of outdoor_hard_01.',
            ),
            PublicDatasetFile(
                id='outdoor_hard_02a',
                filename='outdoor_hard_02a.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_hard_02a.zip?download=1',
                md5='8f56fdfa93f4fa456e234f58e193a08c',
                size_label='1.5 GB',
                notes='First half of outdoor_hard_02.',
            ),
            PublicDatasetFile(
                id='outdoor_hard_02b',
                filename='outdoor_hard_02b.zip',
                url='https://zenodo.org/records/10122133/files/outdoor_hard_02b.zip?download=1',
                md5='267377a88402f825ab70a161cc48983c',
                size_label='1.3 GB',
                notes='Second half of outdoor_hard_02.',
            ),
        ),
        default_file_id='outdoor_kidnap_a',
        profile={
            'robot_name': 'livox_mid360_public_hard_pointcloud',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/points',
            'expected_imu_topic': '/livox/imu',
            'mount': {
                'xyz': [0.0, 0.0, 0.0],
                'q_xyzw': [0.0, 0.0, 0.0, 1.0],
                'note': 'Dataset documents outdoor LiDAR-IMU transform as identity.',
            },
        },
        notes=(
            'Good second target after driving_slam_mid360 because it stresses failure handling.',
            'Dataset also includes /livox/lidar as Livox CustomMsg, but this pipeline uses PointCloud2.',
            'Use analyze_mid360_robot_public_loop_candidates.py before downloading multi-GB split bags for loop evaluation.',
        ),
    ),
}


def public_dataset_registry() -> tuple[PublicDataset, ...]:
    """Return known public MID-360 datasets sorted by id."""
    return tuple(PUBLIC_MID360_DATASETS[key] for key in sorted(PUBLIC_MID360_DATASETS))


def get_public_dataset(dataset_id: str) -> PublicDataset:
    """Look up a public MID-360 dataset."""
    try:
        return PUBLIC_MID360_DATASETS[dataset_id]
    except KeyError as exc:
        valid = ', '.join(sorted(PUBLIC_MID360_DATASETS))
        raise ValueError(f'unknown public MID-360 dataset {dataset_id!r}; valid: {valid}') from exc


class PublicDatasetIntake:
    """Download, extract, and describe a public MID-360 dataset."""

    def __init__(
        self,
        repo_root: Path,
        registry: dict[str, PublicDataset] | None = None,
        progress_stream: TextIO | None = sys.stderr,
    ) -> None:
        self._repo_root = repo_root
        self._registry = registry or PUBLIC_MID360_DATASETS
        self._progress_stream = progress_stream

    def build_plan(self, options: PublicDatasetIntakeOptions) -> dict[str, Any]:
        """Build a reproducible intake plan without touching the network."""
        dataset = self._dataset(options.dataset_id)
        file_record = dataset.file_by_id(options.file_id)
        paths = self._paths(dataset, file_record, options)
        return self._base_report(
            status='DRY_RUN' if options.dry_run else 'PLANNED',
            dataset=dataset,
            file_record=file_record,
            paths=paths,
            options=options,
            download=self._download_plan(file_record),
            extraction=self._extraction_plan(file_record),
            bag_candidates=[],
            selected_bag_path='',
            messages=[],
        )

    def run(self, options: PublicDatasetIntakeOptions) -> dict[str, Any]:
        """Run public dataset intake and write a manifest."""
        dataset = self._dataset(options.dataset_id)
        file_record = dataset.file_by_id(options.file_id)
        paths = self._paths(dataset, file_record, options)
        messages: list[str] = []

        if options.dry_run:
            report = self._base_report(
                status='DRY_RUN',
                dataset=dataset,
                file_record=file_record,
                paths=paths,
                options=options,
                download=self._download_plan(file_record),
                extraction=self._extraction_plan(file_record),
                bag_candidates=[],
                selected_bag_path='',
                messages=['Dry-run only; no files downloaded or extracted.'],
            )
            self._write_manifest(report, paths['dataset_dir'])
            return report

        paths['dataset_dir'].mkdir(parents=True, exist_ok=True)
        self._write_profile(dataset, paths['profile_path'])
        download = self._download(
            file_record,
            paths['archive_path'],
            options,
            messages,
        )
        archive_ready = download['status'] in {'VERIFIED', 'HASHED'}
        extraction = self._extraction_plan(file_record)

        if options.extract and archive_ready:
            extraction = self._extract(
                file_record,
                paths['archive_path'],
                paths['extract_dir'],
                options,
                messages,
            )

        bag_candidates = self._find_bag_dirs(
            paths['extract_dir'],
            file_record,
        )
        selected_bag_path = str(bag_candidates[0]) if bag_candidates else ''
        status = 'READY' if selected_bag_path else 'DOWNLOADED'
        if options.extract and not selected_bag_path:
            status = 'WARN'
            messages.append('No rosbag2 metadata.yaml was found under the extract directory.')

        report = self._base_report(
            status=status,
            dataset=dataset,
            file_record=file_record,
            paths=paths,
            options=options,
            download=download,
            extraction=extraction,
            bag_candidates=[str(path) for path in bag_candidates],
            selected_bag_path=selected_bag_path,
            messages=messages,
        )
        self._write_manifest(report, paths['dataset_dir'])
        return report

    def _dataset(self, dataset_id: str) -> PublicDataset:
        try:
            return self._registry[dataset_id]
        except KeyError as exc:
            valid = ', '.join(sorted(self._registry))
            raise ValueError(f'unknown public MID-360 dataset {dataset_id!r}; valid: {valid}') from exc

    def _paths(
        self,
        dataset: PublicDataset,
        file_record: PublicDatasetFile,
        options: PublicDatasetIntakeOptions,
    ) -> dict[str, Path]:
        dataset_dir = options.dataset_root.expanduser().resolve() / dataset.id
        extract_dir = dataset_dir / 'extracted' / Path(file_record.filename).stem
        output_dir = (
            options.output_dir.expanduser().resolve()
            if options.output_dir
            else self._repo_root / 'output' / 'mid360_public' / dataset.id
        )
        return {
            'dataset_dir': dataset_dir,
            'archive_path': dataset_dir / 'archives' / file_record.filename,
            'extract_dir': extract_dir,
            'profile_path': dataset_dir / f'{dataset.id}_profile.yaml',
            'output_dir': output_dir,
            'manifest_json': dataset_dir / PUBLIC_DATASET_INTAKE_JSON,
            'manifest_markdown': dataset_dir / PUBLIC_DATASET_INTAKE_MARKDOWN,
        }

    def _base_report(
        self,
        status: str,
        dataset: PublicDataset,
        file_record: PublicDatasetFile,
        paths: dict[str, Path],
        options: PublicDatasetIntakeOptions,
        download: dict[str, Any],
        extraction: dict[str, Any],
        bag_candidates: list[str],
        selected_bag_path: str,
        messages: list[str],
    ) -> dict[str, Any]:
        check_command = self._recording_check_command(
            selected_bag_path=selected_bag_path,
            profile_path=paths['profile_path'],
            output_dir=paths['output_dir'],
        )
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'dataset': dataset.to_dict(),
            'file': file_record.to_dict(),
            'dataset_root': str(options.dataset_root.expanduser().resolve()),
            'dataset_dir': str(paths['dataset_dir']),
            'archive_path': str(paths['archive_path']),
            'extract_dir': str(paths['extract_dir']),
            'profile_path': str(paths['profile_path']),
            'output_dir': str(paths['output_dir']),
            'manifest_json': str(paths['manifest_json']),
            'manifest_markdown': str(paths['manifest_markdown']),
            'bag_candidates': bag_candidates,
            'selected_bag_path': selected_bag_path,
            'download': download,
            'extraction': extraction,
            'messages': messages,
            'options': {
                'file_id': options.file_id or dataset.default_file_id,
                'dry_run': options.dry_run,
                'force': options.force,
                'extract': options.extract,
                'verify_md5': options.verify_md5,
            },
            'commands': {
                'download': self._download_command(dataset, file_record, options),
                'recording_check': check_command,
            },
        }

    def _download(
        self,
        file_record: PublicDatasetFile,
        archive_path: Path,
        options: PublicDatasetIntakeOptions,
        messages: list[str],
    ) -> dict[str, Any]:
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        part_path = archive_path.with_suffix(archive_path.suffix + '.part')
        self._reject_unsafe_download_path(archive_path, 'archive')
        self._reject_unsafe_download_path(part_path, 'partial archive')

        if archive_path.is_file() and not options.force:
            size, digests = self._hash_file(archive_path)
            report = self._verified_download_report(
                file_record,
                size_bytes=size,
                digests=digests,
                source='cache',
                resumed_bytes=0,
                transferred_bytes=0,
                verify_md5=options.verify_md5,
            )
            messages.append(
                f'Verified existing archive: {archive_path.name} '
                f'(sha256 {digests["sha256"]}).'
            )
            return report

        if options.force and part_path.is_file():
            part_path.unlink()

        resume_bytes = part_path.stat().st_size if part_path.is_file() else 0
        if (
            file_record.size_bytes is not None
            and resume_bytes > file_record.size_bytes
        ):
            raise ValueError(
                f'partial archive is larger than expected for '
                f'{file_record.filename}: {resume_bytes} > '
                f'{file_record.size_bytes}; use --force to restart'
            )
        if (
            resume_bytes > 0
            and file_record.size_bytes is not None
            and resume_bytes == file_record.size_bytes
        ):
            size, digests = self._hash_file(part_path)
            report = self._verified_download_report(
                file_record,
                size_bytes=size,
                digests=digests,
                source='complete-part',
                resumed_bytes=resume_bytes,
                transferred_bytes=0,
                verify_md5=options.verify_md5,
            )
            part_path.replace(archive_path)
            messages.append(
                f'Verified and finalized complete partial archive: '
                f'{archive_path.name}.'
            )
            return report

        hashers = self._new_hashers()
        if resume_bytes:
            hashed_size = self._update_hashers_from_file(part_path, hashers)
            if hashed_size != resume_bytes:
                raise ValueError('partial archive changed while it was hashed')
            messages.append(
                f'Resuming {file_record.filename} at byte {resume_bytes}.'
            )

        request = self._download_request(file_record, resume_bytes)
        try:
            response = urllib.request.urlopen(request)
        except urllib.error.HTTPError as exc:
            raise ValueError(
                f'download request failed for {file_record.filename}: '
                f'HTTP {exc.code}'
            ) from exc

        with response:
            accepted_bytes, total_bytes, expected_response_bytes = (
                self._response_plan(
                    response,
                    requested_offset=resume_bytes,
                    expected_total=file_record.size_bytes,
                )
            )
            if accepted_bytes != resume_bytes:
                if resume_bytes:
                    messages.append(
                        'Server ignored the Range request; safely restarting '
                        f'{file_record.filename} from byte 0.'
                    )
                hashers = self._new_hashers()
                mode = 'wb'
            else:
                mode = 'ab' if accepted_bytes else 'wb'
            with part_path.open(mode) as output:
                transferred_bytes = self._stream_download(
                    file_record,
                    response,
                    output,
                    hashers,
                    initial_bytes=accepted_bytes,
                    total_bytes=total_bytes,
                    expected_response_bytes=expected_response_bytes,
                )

        final_size = part_path.stat().st_size
        digests = {
            name: hasher.hexdigest()
            for name, hasher in hashers.items()
        }
        report = self._verified_download_report(
            file_record,
            size_bytes=final_size,
            digests=digests,
            source='resume' if accepted_bytes else 'network',
            resumed_bytes=accepted_bytes,
            transferred_bytes=transferred_bytes,
            verify_md5=options.verify_md5,
        )
        part_path.replace(archive_path)
        messages.append(
            f'Verified {archive_path.name}: {final_size} bytes, '
            f'sha256 {digests["sha256"]}.'
        )
        return report

    @staticmethod
    def _download_plan(file_record: PublicDatasetFile) -> dict[str, Any]:
        return {
            'status': 'NOT_RUN',
            'source': None,
            'expected_size_bytes': file_record.size_bytes,
            'expected_sha256': file_record.sha256 or None,
            'expected_md5': file_record.md5 or None,
            'size_bytes': None,
            'sha256': None,
            'md5': None,
            'size_verified': None,
            'sha256_verified': None,
            'md5_verified': None,
            'resumed_bytes': 0,
            'transferred_bytes': 0,
        }

    @staticmethod
    def _extraction_plan(
        file_record: PublicDatasetFile,
    ) -> dict[str, Any]:
        return {
            'status': 'NOT_RUN',
            'source': None,
            'identity_algorithm': (
                'sha256' if file_record.extracted_files else None
            ),
            'files': [
                {
                    'path': item.path,
                    'expected_size_bytes': item.size_bytes,
                    'expected_sha256': item.sha256,
                    'size_bytes': None,
                    'sha256': None,
                    'size_verified': None,
                    'sha256_verified': None,
                }
                for item in file_record.extracted_files
            ],
        }

    @staticmethod
    def _reject_unsafe_download_path(path: Path, label: str) -> None:
        if path.is_symlink():
            raise ValueError(f'{label} must not be a symlink: {path}')
        if path.exists() and not path.is_file():
            raise ValueError(f'{label} must be a regular file: {path}')

    @staticmethod
    def _new_hashers() -> dict[str, Any]:
        return {
            'md5': hashlib.md5(usedforsecurity=False),
            'sha256': hashlib.sha256(),
        }

    @classmethod
    def _update_hashers_from_file(
        cls,
        path: Path,
        hashers: dict[str, Any],
    ) -> int:
        size = 0
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(DOWNLOAD_CHUNK_BYTES), b''):
                for hasher in hashers.values():
                    hasher.update(chunk)
                size += len(chunk)
        return size

    @classmethod
    def _hash_file(cls, path: Path) -> tuple[int, dict[str, str]]:
        hashers = cls._new_hashers()
        size = cls._update_hashers_from_file(path, hashers)
        return size, {
            name: hasher.hexdigest()
            for name, hasher in hashers.items()
        }

    @staticmethod
    def _hash_sha256_file(path: Path) -> tuple[int, str]:
        digest = hashlib.sha256()
        size = 0
        with path.open('rb') as stream:
            for chunk in iter(
                lambda: stream.read(DOWNLOAD_CHUNK_BYTES),
                b'',
            ):
                digest.update(chunk)
                size += len(chunk)
        return size, digest.hexdigest()

    @staticmethod
    def _download_request(
        file_record: PublicDatasetFile,
        offset: int,
    ) -> urllib.request.Request:
        headers = {
            'Accept-Encoding': 'identity',
            'User-Agent': DOWNLOAD_USER_AGENT,
        }
        if offset:
            headers['Range'] = f'bytes={offset}-'
        return urllib.request.Request(file_record.url, headers=headers)

    @classmethod
    def _response_plan(
        cls,
        response: BinaryIO,
        *,
        requested_offset: int,
        expected_total: int | None,
    ) -> tuple[int, int | None, int | None]:
        status = getattr(response, 'status', None)
        if status is None:
            getcode = getattr(response, 'getcode', None)
            status = getcode() if callable(getcode) else None
        content_length = cls._response_content_length(response)
        headers = getattr(response, 'headers', None) or {}

        if status == 206:
            content_range = headers.get('Content-Range')
            match = CONTENT_RANGE_PATTERN.fullmatch(content_range or '')
            if match is None:
                raise ValueError(
                    'HTTP 206 response has no valid Content-Range'
                )
            start = int(match.group('start'))
            end = int(match.group('end'))
            total = int(match.group('total'))
            if start != requested_offset or end < start or total <= end:
                raise ValueError(
                    'HTTP Content-Range does not match the requested offset'
                )
            response_bytes = end - start + 1
            if (
                content_length is not None
                and content_length != response_bytes
            ):
                raise ValueError(
                    'HTTP Content-Length does not match Content-Range'
                )
        elif status in (None, 200):
            start = 0
            total = content_length
            response_bytes = content_length
        else:
            raise ValueError(f'unexpected download HTTP status: {status}')

        if expected_total is not None and total not in (None, expected_total):
            raise ValueError(
                f'remote size {total} does not match expected '
                f'{expected_total}'
            )
        return start, expected_total or total, response_bytes

    def _stream_download(
        self,
        file_record: PublicDatasetFile,
        response: BinaryIO,
        output: BinaryIO,
        hashers: dict[str, Any],
        *,
        initial_bytes: int = 0,
        total_bytes: int | None = None,
        expected_response_bytes: int | None = None,
    ) -> int:
        started_at = time.monotonic()
        last_report_at = started_at
        last_report_bytes = initial_bytes
        downloaded_bytes = initial_bytes
        transferred_bytes = 0
        self._print_download_progress(
            file_record,
            downloaded_bytes,
            total_bytes,
            0.0,
        )

        while True:
            chunk = response.read(DOWNLOAD_CHUNK_BYTES)
            if not chunk:
                break
            if (
                file_record.size_bytes is not None
                and downloaded_bytes + len(chunk) > file_record.size_bytes
            ):
                raise ValueError(
                    f'download exceeded expected size for '
                    f'{file_record.filename}'
                )
            for hasher in hashers.values():
                hasher.update(chunk)
            output.write(chunk)
            downloaded_bytes += len(chunk)
            transferred_bytes += len(chunk)
            now = time.monotonic()
            if (
                now - last_report_at >= DOWNLOAD_PROGRESS_INTERVAL_SEC
                or downloaded_bytes - last_report_bytes
                >= DOWNLOAD_PROGRESS_BYTES
            ):
                self._print_download_progress(
                    file_record,
                    downloaded_bytes,
                    total_bytes,
                    now - started_at,
                    session_bytes=transferred_bytes,
                )
                last_report_at = now
                last_report_bytes = downloaded_bytes

        if (
            expected_response_bytes is not None
            and transferred_bytes != expected_response_bytes
        ):
            raise ValueError(
                f'truncated download for {file_record.filename}: received '
                f'{transferred_bytes}, expected {expected_response_bytes}'
            )

        elapsed_sec = time.monotonic() - started_at
        self._print_download_progress(
            file_record,
            downloaded_bytes,
            total_bytes,
            elapsed_sec,
            complete=True,
            session_bytes=transferred_bytes,
        )
        return transferred_bytes

    @staticmethod
    def _response_content_length(response: BinaryIO) -> int | None:
        headers = getattr(response, 'headers', None)
        if headers is None:
            return None
        value = headers.get('Content-Length')
        try:
            length = int(value)
        except (TypeError, ValueError):
            return None
        return length if length >= 0 else None

    def _print_download_progress(
        self,
        file_record: PublicDatasetFile,
        downloaded_bytes: int,
        total_bytes: int | None,
        elapsed_sec: float,
        *,
        complete: bool = False,
        session_bytes: int | None = None,
    ) -> None:
        if self._progress_stream is None:
            return
        downloaded = self._format_bytes(downloaded_bytes)
        if total_bytes:
            percent = min(100.0, downloaded_bytes * 100.0 / total_bytes)
            amount = (
                f'{downloaded} / {self._format_bytes(total_bytes)} '
                f'({percent:.1f}%)'
            )
        else:
            amount = downloaded
        speed = ''
        speed_bytes = (
            downloaded_bytes if session_bytes is None else session_bytes
        )
        if elapsed_sec > 0 and speed_bytes > 0:
            speed = f', {self._format_bytes(int(speed_bytes / elapsed_sec))}/s'
        state = 'Downloaded' if complete else 'Downloading'
        print(
            f'{state} {file_record.filename}: {amount}{speed}',
            file=self._progress_stream,
            flush=True,
        )

    @staticmethod
    def _format_bytes(value: int) -> str:
        amount = float(value)
        units = ('B', 'KiB', 'MiB', 'GiB', 'TiB')
        for unit in units:
            if amount < 1024.0 or unit == units[-1]:
                if unit == 'B':
                    return f'{int(amount)} {unit}'
                return f'{amount:.1f} {unit}'
            amount /= 1024.0
        raise AssertionError('unreachable')

    @staticmethod
    def _verified_download_report(
        file_record: PublicDatasetFile,
        *,
        size_bytes: int,
        digests: dict[str, str],
        source: str,
        resumed_bytes: int,
        transferred_bytes: int,
        verify_md5: bool,
    ) -> dict[str, Any]:
        size_verified = (
            None
            if file_record.size_bytes is None
            else size_bytes == file_record.size_bytes
        )
        sha256_verified = (
            None
            if not file_record.sha256
            else digests['sha256'].lower() == file_record.sha256.lower()
        )
        md5_verified = (
            None
            if not verify_md5 or not file_record.md5
            else digests['md5'].lower() == file_record.md5.lower()
        )
        failures = [
            name for name, result in (
                ('size', size_verified),
                ('SHA-256', sha256_verified),
                ('MD5', md5_verified),
            )
            if result is False
        ]
        if failures:
            raise ValueError(
                f'{", ".join(failures)} mismatch for '
                f'{file_record.filename}; downloaded data was retained and '
                'must not be used (use --force to restart)'
            )
        verified = any(
            result is True
            for result in (size_verified, sha256_verified, md5_verified)
        )
        return {
            'status': 'VERIFIED' if verified else 'HASHED',
            'source': source,
            'expected_size_bytes': file_record.size_bytes,
            'expected_sha256': file_record.sha256 or None,
            'expected_md5': file_record.md5 or None,
            'size_bytes': size_bytes,
            'sha256': digests['sha256'],
            'md5': digests['md5'],
            'size_verified': size_verified,
            'sha256_verified': sha256_verified,
            'md5_verified': md5_verified,
            'resumed_bytes': resumed_bytes,
            'transferred_bytes': transferred_bytes,
        }

    def _extract(
        self,
        file_record: PublicDatasetFile,
        archive_path: Path,
        extract_dir: Path,
        options: PublicDatasetIntakeOptions,
        messages: list[str],
    ) -> dict[str, Any]:
        if extract_dir.is_symlink():
            raise ValueError(
                f'extract directory must not be a symlink: {extract_dir}'
            )
        if extract_dir.exists() and not extract_dir.is_dir():
            raise ValueError(
                f'extract destination is not a directory: {extract_dir}'
            )
        if extract_dir.exists() and not options.force:
            if file_record.extracted_files:
                report = self._verify_extracted_files(
                    file_record,
                    extract_dir,
                    source='cache',
                )
                messages.append(
                    f'Verified existing extraction: {extract_dir} '
                    f'({len(report["files"])} pinned files).'
                )
                return report
            messages.append(
                f'Extract directory already exists without registered '
                f'member identities: {extract_dir}'
            )
            return {
                **self._extraction_plan(file_record),
                'status': 'REUSED',
                'source': 'cache',
            }

        if file_record.archive_format != 'zip':
            raise ValueError(
                f'unsupported archive format: {file_record.archive_format}'
            )

        partial_dir = extract_dir.with_name(f'.{extract_dir.name}.partial')
        previous_dir = extract_dir.with_name(f'.{extract_dir.name}.previous')
        for path, label in (
            (partial_dir, 'partial extract directory'),
            (previous_dir, 'previous extract directory'),
        ):
            if path.is_symlink():
                raise ValueError(f'{label} must not be a symlink: {path}')
            if path.exists() and not path.is_dir():
                raise ValueError(f'{label} is not a directory: {path}')
        if previous_dir.exists():
            raise ValueError(
                f'previous extract transaction still exists: '
                f'{previous_dir}; recover it before retrying'
            )
        if partial_dir.exists():
            if not options.force:
                raise ValueError(
                    f'partial extraction exists: {partial_dir}; inspect it '
                    'or use --force to restart extraction'
                )
            shutil.rmtree(partial_dir)

        try:
            archive = zipfile.ZipFile(archive_path)
        except (OSError, zipfile.BadZipFile) as exc:
            raise ValueError(
                f'archive is not a readable ZIP: {archive_path}: {exc}'
            ) from exc
        with archive:
            members = self._safe_zip_members(archive)
            partial_dir.mkdir(parents=True)
            for info, relative in members:
                destination = partial_dir.joinpath(*relative.parts)
                if info.is_dir():
                    destination.mkdir(parents=True, exist_ok=True)
                    continue
                destination.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with archive.open(info, mode='r') as source:
                        with destination.open('xb') as output:
                            shutil.copyfileobj(
                                source,
                                output,
                                length=DOWNLOAD_CHUNK_BYTES,
                            )
                except (OSError, RuntimeError, zipfile.BadZipFile) as exc:
                    raise ValueError(
                        f'failed to extract ZIP member {info.filename}: '
                        f'{exc}'
                    ) from exc

        if file_record.extracted_files:
            report = self._verify_extracted_files(
                file_record,
                partial_dir,
                source='fresh',
            )
        else:
            report = {
                **self._extraction_plan(file_record),
                'status': 'EXTRACTED',
                'source': 'fresh',
            }

        moved_previous = False
        if extract_dir.exists():
            extract_dir.rename(previous_dir)
            moved_previous = True
        try:
            partial_dir.rename(extract_dir)
        except OSError:
            if moved_previous and not extract_dir.exists():
                previous_dir.rename(extract_dir)
            raise
        if moved_previous:
            shutil.rmtree(previous_dir)
        messages.append(f'Extracted {archive_path.name} into {extract_dir}.')
        return report

    @classmethod
    def _verify_extracted_files(
        cls,
        file_record: PublicDatasetFile,
        extract_dir: Path,
        *,
        source: str,
    ) -> dict[str, Any]:
        """Hash pinned extracted files and reject any identity mismatch."""
        checked: list[dict[str, Any]] = []
        for expected in file_record.extracted_files:
            path = cls._checked_extracted_path(extract_dir, expected.path)
            size, sha256 = cls._hash_sha256_file(path)
            size_verified = size == expected.size_bytes
            sha256_verified = (
                sha256.lower() == expected.sha256.lower()
            )
            checked.append({
                'path': expected.path,
                'expected_size_bytes': expected.size_bytes,
                'expected_sha256': expected.sha256,
                'size_bytes': size,
                'sha256': sha256,
                'size_verified': size_verified,
                'sha256_verified': sha256_verified,
            })
            failures = [
                label for label, passed in (
                    ('size', size_verified),
                    ('SHA-256', sha256_verified),
                )
                if not passed
            ]
            if failures:
                raise ValueError(
                    f'{", ".join(failures)} mismatch for extracted file '
                    f'{expected.path}; the extraction must not be used '
                    '(use --force to replace it from the verified archive)'
                )
        return {
            'status': 'VERIFIED',
            'source': source,
            'identity_algorithm': 'sha256',
            'files': checked,
        }

    @staticmethod
    def _checked_extracted_path(
        extract_dir: Path,
        relative_path: str,
    ) -> Path:
        current = extract_dir
        parts = PurePosixPath(relative_path).parts
        for index, part in enumerate(parts):
            current = current / part
            try:
                details = current.lstat()
            except OSError as exc:
                raise ValueError(
                    f'pinned extracted file is missing: {relative_path}'
                ) from exc
            if stat.S_ISLNK(details.st_mode):
                raise ValueError(
                    f'pinned extracted path must not be a symlink: '
                    f'{relative_path}'
                )
            if index < len(parts) - 1 and not stat.S_ISDIR(
                details.st_mode
            ):
                raise ValueError(
                    f'pinned extracted parent is not a directory: '
                    f'{relative_path}'
                )
            if index == len(parts) - 1 and not stat.S_ISREG(
                details.st_mode
            ):
                raise ValueError(
                    f'pinned extracted file is not regular: {relative_path}'
                )
        return current

    @staticmethod
    def _safe_zip_members(
        archive: zipfile.ZipFile,
    ) -> list[tuple[zipfile.ZipInfo, PurePosixPath]]:
        members: list[tuple[zipfile.ZipInfo, PurePosixPath]] = []
        names: set[str] = set()
        folded_names: set[str] = set()
        try:
            infos = archive.infolist()
        except zipfile.BadZipFile as exc:
            raise ValueError(
                f'cannot read ZIP member directory: {exc}'
            ) from exc
        for info in infos:
            raw_name = info.filename
            name = raw_name[:-1] if info.is_dir() else raw_name
            relative = PurePosixPath(name)
            if (
                not name
                or name.startswith('/')
                or '\\' in name
                or '\x00' in name
                or relative.as_posix() != name
                or any(part in ('', '.', '..') for part in name.split('/'))
            ):
                raise ValueError(f'unsafe ZIP member path: {raw_name!r}')
            folded = name.casefold()
            if name in names or folded in folded_names:
                raise ValueError(f'duplicate ZIP member path: {raw_name!r}')
            names.add(name)
            folded_names.add(folded)
            if info.flag_bits & 0x41:
                raise ValueError(f'encrypted ZIP member: {raw_name!r}')
            mode = info.external_attr >> 16
            file_type = stat.S_IFMT(mode)
            if info.is_dir():
                if file_type not in (0, stat.S_IFDIR):
                    raise ValueError(
                        f'ZIP directory has an unsafe type: {raw_name!r}'
                    )
            elif file_type not in (0, stat.S_IFREG):
                raise ValueError(
                    f'ZIP member is not a regular file: {raw_name!r}'
                )
            members.append((info, relative))
        if not members:
            raise ValueError('ZIP archive contains no members')
        return members

    @staticmethod
    def _find_bag_dirs(
        extract_dir: Path,
        file_record: PublicDatasetFile,
    ) -> list[Path]:
        if not extract_dir.is_dir():
            return []
        pinned_metadata = [
            extract_dir.joinpath(*PurePosixPath(item.path).parts).parent
            for item in file_record.extracted_files
            if PurePosixPath(item.path).name == 'metadata.yaml'
        ]
        if pinned_metadata:
            return sorted(
                path.resolve()
                for path in pinned_metadata
                if (path / 'metadata.yaml').is_file()
                and not (path / 'metadata.yaml').is_symlink()
            )
        matches = [path.parent for path in extract_dir.rglob('metadata.yaml') if path.is_file()]
        return sorted(set(path.resolve() for path in matches))

    @staticmethod
    def _write_profile(dataset: PublicDataset, profile_path: Path) -> None:
        profile_path.parent.mkdir(parents=True, exist_ok=True)
        profile_path.write_text(
            yaml.safe_dump(dataset.profile, sort_keys=False),
            encoding='utf-8',
        )

    def _write_manifest(self, report: dict[str, Any], dataset_dir: Path) -> None:
        dataset_dir.mkdir(parents=True, exist_ok=True)
        json_path = dataset_dir / PUBLIC_DATASET_INTAKE_JSON
        markdown_path = dataset_dir / PUBLIC_DATASET_INTAKE_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(
            render_public_dataset_intake_markdown(report) + '\n',
            encoding='utf-8',
        )

    def _download_command(
        self,
        dataset: PublicDataset,
        file_record: PublicDatasetFile,
        options: PublicDatasetIntakeOptions,
    ) -> str:
        command = [
            'python3',
            str(self._repo_root / 'scripts' / 'download_mid360_robot_public_dataset.py'),
            '--dataset',
            dataset.id,
            '--file',
            file_record.id,
            '--dataset-root',
            str(options.dataset_root.expanduser().resolve()),
        ]
        if options.output_dir:
            command.extend(['--output-dir', str(options.output_dir.expanduser().resolve())])
        if options.force:
            command.append('--force')
        if not options.extract:
            command.append('--no-extract')
        if not options.verify_md5:
            command.append('--skip-md5')
        return shlex.join(command)

    def _recording_check_command(
        self,
        selected_bag_path: str,
        profile_path: Path,
        output_dir: Path,
    ) -> str:
        if not selected_bag_path:
            return ''
        command = [
            'bash',
            str(self._repo_root / 'scripts' / 'check_mid360_robot_recording.sh'),
            '--bag',
            selected_bag_path,
            '--robot-profile',
            str(profile_path),
            '--output-dir',
            str(output_dir),
        ]
        return shlex.join(command)


def download_verified_artifact(
    file_record: PublicDatasetFile,
    destination: Path,
    *,
    force: bool = False,
    verify_md5: bool = True,
    progress_stream: TextIO | None = sys.stderr,
) -> tuple[dict[str, Any], list[str]]:
    """Download one size/SHA-pinned artifact through the hardened intake."""
    if file_record.size_bytes is None or not file_record.sha256:
        raise ValueError(
            'verified artifact downloads require expected size and SHA-256'
        )
    destination = destination.expanduser()
    if destination.is_symlink():
        raise ValueError(
            f'artifact destination must not be a symlink: {destination}'
        )
    destination = destination.parent.resolve() / destination.name
    if destination.name != file_record.filename:
        raise ValueError(
            f'artifact destination must retain filename '
            f'{file_record.filename!r}'
        )
    options = PublicDatasetIntakeOptions(
        dataset_id='verified_artifact',
        dataset_root=destination.parent,
        force=force,
        extract=False,
        verify_md5=verify_md5,
    )
    messages: list[str] = []
    report = PublicDatasetIntake(
        destination.parent,
        progress_stream=progress_stream,
    )._download(file_record, destination, options, messages)
    return report, messages


def render_public_dataset_list(datasets: tuple[PublicDataset, ...]) -> str:
    """Render a concise public dataset list."""
    lines = ['MID-360 Public Datasets', '']
    for dataset in datasets:
        default_file = dataset.file_by_id()
        lines.extend([
            f'- {dataset.id}: {dataset.title}',
            f'  source: {dataset.source_url}',
            f'  default_file: {default_file.filename} ({default_file.size_label or "unknown size"})',
        ])
    return '\n'.join(lines)


def render_public_dataset_intake_markdown(report: dict[str, Any]) -> str:
    """Render public dataset intake as Markdown."""
    dataset = report['dataset']
    file_record = report['file']
    download = report['download']
    extraction = report.get('extraction') or {'status': 'NOT_REPORTED'}
    lines = [
        '# MID-360 Public Dataset Intake',
        '',
        f"- status: `{report['status']}`",
        f"- created_at: `{report['created_at']}`",
        f"- dataset_id: `{dataset['id']}`",
        f"- title: `{dataset['title']}`",
        f"- source: `{dataset['source_url']}`",
        f"- file: `{file_record['filename']}`",
        f"- size: `{file_record.get('size_label') or 'unknown'}`",
        f"- download_status: `{download['status']}`",
        f"- extraction_status: `{extraction['status']}`",
        f"- expected_size_bytes: `{download['expected_size_bytes']}`",
        f"- expected_sha256: `{download['expected_sha256'] or ''}`",
        f"- actual_size_bytes: `{download['size_bytes']}`",
        f"- actual_sha256: `{download['sha256'] or ''}`",
        f"- resumed_bytes: `{download['resumed_bytes']}`",
        f"- transferred_bytes: `{download['transferred_bytes']}`",
        f"- archive_path: `{report['archive_path']}`",
        f"- extract_dir: `{report['extract_dir']}`",
        f"- profile_path: `{report['profile_path']}`",
        f"- selected_bag_path: `{report.get('selected_bag_path') or ''}`",
        '',
        '## Messages',
        '',
    ]
    messages = report.get('messages') or []
    if messages:
        lines.extend(f'- {message}' for message in messages)
    else:
        lines.append('- none')

    lines.extend(['', '## Bag Candidates', ''])
    candidates = report.get('bag_candidates') or []
    if candidates:
        lines.extend(f'- `{candidate}`' for candidate in candidates)
    else:
        lines.append('- none')

    lines.extend(['', '## Commands', '', '```bash'])
    if report['commands'].get('download'):
        lines.append(report['commands']['download'])
    if report['commands'].get('recording_check'):
        lines.append(report['commands']['recording_check'])
    lines.append('```')
    return '\n'.join(lines)


def public_dataset_payload() -> dict[str, Any]:
    """Return the registry as a JSON-serializable payload."""
    return {
        'datasets': [dataset.to_dict() for dataset in public_dataset_registry()],
    }


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_datasets is a library module; import it instead of running it directly.")
