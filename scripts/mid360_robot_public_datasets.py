#!/usr/bin/env python3
"""Public MID-360 dataset registry and intake helpers."""

from __future__ import annotations

import hashlib
import json
import shlex
import shutil
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_DATASET_INTAKE_JSON = 'mid360_robot_public_dataset_intake.json'
PUBLIC_DATASET_INTAKE_MARKDOWN = 'mid360_robot_public_dataset_intake.md'


@dataclass(frozen=True)
class PublicDatasetFile:
    """Downloadable file in a public MID-360 dataset."""

    id: str
    filename: str
    url: str
    md5: str = ''
    size_label: str = ''
    archive_format: str = 'zip'
    notes: str = ''

    def to_dict(self) -> dict[str, str]:
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
        license='See Zenodo record.',
        citation='Koide, Kenji. Driving SLAM Test with Livox MID360. Zenodo. DOI: 10.5281/zenodo.14841855',
        files=(
            PublicDatasetFile(
                id='rosbag2_2024_04_16',
                filename='rosbag2_2024_04_16-14_17_01.zip',
                url='https://zenodo.org/records/14841855/files/rosbag2_2024_04_16-14_17_01.zip?download=1',
                md5='0836c50859bb1af591966b69da166186',
                size_label='517.1 MB',
                notes='Recommended first public MID-360 ROS2 intake target.',
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
    ) -> None:
        self._repo_root = repo_root
        self._registry = registry or PUBLIC_MID360_DATASETS

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
                bag_candidates=[],
                selected_bag_path='',
                messages=['Dry-run only; no files downloaded or extracted.'],
            )
            self._write_manifest(report, paths['dataset_dir'])
            return report

        paths['dataset_dir'].mkdir(parents=True, exist_ok=True)
        self._write_profile(dataset, paths['profile_path'])
        archive_ready = self._download(file_record, paths['archive_path'], options, messages)
        if archive_ready and options.verify_md5 and file_record.md5:
            self._verify_md5(paths['archive_path'], file_record.md5)
            messages.append(f'MD5 verified for {paths["archive_path"].name}.')

        if options.extract and archive_ready:
            self._extract(file_record, paths['archive_path'], paths['extract_dir'], options, messages)

        bag_candidates = self._find_bag_dirs(paths['extract_dir'])
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
    ) -> bool:
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        if archive_path.is_file() and not options.force:
            messages.append(f'Archive already exists: {archive_path}')
            return True

        part_path = archive_path.with_suffix(archive_path.suffix + '.part')
        if part_path.exists():
            part_path.unlink()
        md5 = hashlib.md5()
        with urllib.request.urlopen(file_record.url) as response, part_path.open('wb') as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                md5.update(chunk)
                output.write(chunk)
        part_path.replace(archive_path)
        messages.append(f'Downloaded {archive_path} with md5 {md5.hexdigest()}.')
        return True

    @staticmethod
    def _verify_md5(archive_path: Path, expected_md5: str) -> None:
        md5 = hashlib.md5()
        with archive_path.open('rb') as archive:
            while True:
                chunk = archive.read(1024 * 1024)
                if not chunk:
                    break
                md5.update(chunk)
        actual = md5.hexdigest()
        if actual.lower() != expected_md5.lower():
            raise ValueError(
                f'MD5 mismatch for {archive_path}: expected {expected_md5}, got {actual}'
            )

    def _extract(
        self,
        file_record: PublicDatasetFile,
        archive_path: Path,
        extract_dir: Path,
        options: PublicDatasetIntakeOptions,
        messages: list[str],
    ) -> None:
        if extract_dir.exists() and not options.force:
            messages.append(f'Extract directory already exists: {extract_dir}')
            return
        if extract_dir.exists():
            shutil.rmtree(extract_dir)
        extract_dir.mkdir(parents=True, exist_ok=True)

        if file_record.archive_format != 'zip':
            raise ValueError(f'unsupported archive format: {file_record.archive_format}')
        with zipfile.ZipFile(archive_path) as archive:
            archive.extractall(extract_dir)
        messages.append(f'Extracted {archive_path.name} into {extract_dir}.')

    @staticmethod
    def _find_bag_dirs(extract_dir: Path) -> list[Path]:
        if not extract_dir.is_dir():
            return []
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
