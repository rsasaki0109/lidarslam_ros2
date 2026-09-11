#!/usr/bin/env python3
"""Validate the clean-prefix lidarslam product CLI installation."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zipfile

import yaml


# This checker imports installed product modules directly. Keep that validation
# read-only just like the installed launcher it is checking.
sys.dont_write_bytecode = True


REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_RUNTIME_MANIFEST = (
    REPO_ROOT / 'lidarslam' / 'product-runtime-files.txt'
)
CLI_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
OPTION_PATTERN = re.compile(
    r'(?<![A-Za-z0-9-])(--[a-z][a-z0-9-]*|-h)(?![A-Za-z0-9-])'
)
VALUE_OPTION_PATTERN = re.compile(
    r'(?P<option>--[a-z][a-z0-9-]*)[ =]'
    r'(?P<value>\{[^}\n]+\}|<[^>\n]+>|[A-Z][A-Z0-9_]*)'
)
ROS_ENVIRONMENT_KEYS = (
    'AMENT_PREFIX_PATH',
    'CMAKE_PREFIX_PATH',
    'COLCON_PREFIX_PATH',
    'LD_LIBRARY_PATH',
    'PYTHONPATH',
    'ROS_DISTRO',
    'ROS_PYTHON_VERSION',
    'ROS_VERSION',
)


def _runtime_names(path: Path) -> tuple[str, ...]:
    names = tuple(
        line.strip()
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    )
    if not names:
        raise RuntimeError(f'product runtime manifest is empty: {path}')
    return names


def _python_bytecode_snapshot(
    root: Path,
) -> dict[str, tuple[str, int, int, str]]:
    """Snapshot Python cache artifacts that could mutate an install prefix."""
    if not root.is_dir():
        return {}
    snapshot: dict[str, tuple[str, int, int, str]] = {}
    for path in sorted(root.rglob('*')):
        is_cache_dir = path.is_dir() and path.name == '__pycache__'
        is_bytecode = path.is_file() and path.suffix in ('.pyc', '.pyo')
        if not (is_cache_dir or is_bytecode):
            continue
        stat = path.stat()
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if is_bytecode else ''
        snapshot[path.relative_to(root).as_posix()] = (
            'file' if is_bytecode else 'directory',
            stat.st_size,
            stat.st_mtime_ns,
            digest,
        )
    return snapshot


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load installed product module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run(
    command: list[str],
    cwd: Path,
    env_updates: dict[str, str] | None = None,
    *,
    clean_ros_environment: bool = False,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop('LIDARSLAM_CLI_NAME', None)
    if clean_ros_environment:
        for name in ROS_ENVIRONMENT_KEYS:
            env.pop(name, None)
    if env_updates:
        env.update(env_updates)
    return subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def _require_success(
    result: subprocess.CompletedProcess[str],
    label: str,
) -> None:
    if result.returncode != 0:
        raise RuntimeError(
            f'{label} failed with {result.returncode}\n'
            f'stdout:\n{result.stdout}\nstderr:\n{result.stderr}'
        )


def _option_definition_lines(rendered_help: str) -> str:
    return '\n'.join(
        line
        for line in rendered_help.splitlines()
        if line.lstrip().startswith('-')
    )


def _validate_installed_help(
    command_path: Path,
    work_dir: Path,
) -> None:
    """Match clean-prefix full help against the source v1 contract."""
    contract = json.loads(CLI_CONTRACT.read_text(encoding='utf-8'))
    for command, command_contract in contract['commands'].items():
        result = _run(
            [str(command_path), command, '--help-all'],
            work_dir,
        )
        _require_success(result, f'installed {command} --help-all')
        definition_lines = _option_definition_lines(result.stdout)
        rendered_options = set(OPTION_PATTERN.findall(definition_lines))
        expected_options = {
            name
            for option in command_contract['options']
            for name in option['names']
        }
        if rendered_options != expected_options:
            raise RuntimeError(
                f'installed {command} option inventory differs from '
                f'contract: rendered={sorted(rendered_options)}, '
                f'expected={sorted(expected_options)}'
            )

        rendered_values = {
            match.group('option'): match.group('value')
            for match in VALUE_OPTION_PATTERN.finditer(definition_lines)
        }
        expected_values = {
            option: value['value_name']
            for option, value in contract['value_options'][command].items()
        }
        if rendered_values != expected_values:
            raise RuntimeError(
                f'installed {command} value contract differs: '
                f'rendered={rendered_values}, expected={expected_values}'
            )


def _write_bag_fixture(path: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    def topic_metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:  # Humble TopicMetadata predates the numeric id
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        topic_metadata(0, '/points', 'sensor_msgs/msg/PointCloud2')
    )
    writer.create_topic(topic_metadata(1, '/imu', 'sensor_msgs/msg/Imu'))

    points = PointCloud2()
    points.header.frame_id = 'lidar'
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points.is_bigendian = False
    points.point_step = 16
    points.row_step = 16
    points.data = list(struct.pack('<ffff', 1.0, 2.0, 3.0, 0.0))
    points.is_dense = True
    writer.write('/points', serialize_message(points), 1_000_000_000)
    writer.write('/imu', serialize_message(Imu()), 1_000_000_001)
    if hasattr(writer, 'close'):
        writer.close()


def _write_unsupported_bag_fixture(path: Path) -> None:
    path.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 1_000_000_000},
            'message_count': 0,
            'topics_with_message_count': [],
        },
    }
    (path / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata),
        encoding='utf-8',
    )


def _historical_manifest_fixture() -> dict[str, object]:
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v1.schema.json'
        ),
        'run_id': '12345678-1234-4234-8234-123456789abc',
        'status': 'succeeded',
        'input': {
            'bag_path': '/data/bag',
            'metadata_path': '/data/bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.6.0',
            'git_commit': 'b' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.6.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {'id': 'fixture', 'label': 'Fixture'},
        'execution': {
            'argv': ['lidarslam-map', 'run', '/data/bag'],
            'command_shell': 'lidarslam-map run /data/bag',
            'started_at': '2026-07-29T00:00:00Z',
            'finished_at': '2026-07-29T00:01:00Z',
            'exit_code': 0,
        },
        'output': {
            'requested_dir': '/data/map',
            'working_dir': '/data/map.partial',
            'finalized': True,
            'diagnosis_status': 'success',
            'artifact_checksums': [],
        },
    }


def _release_image_fixture() -> dict[str, object]:
    return {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': 'jazzy',
        'platform': 'linux/amd64',
        'tag': 'ghcr.io/example/lidar_slam_ros2:v0.6.0-jazzy',
        'digest': f"sha256:{'c' * 64}",
        'git_commit': 'd' * 40,
        'product_version': '0.6.0',
        'cli_version': 'lidarslam_ros2 0.6.0',
    }


def validate_install(
    prefix: Path,
    expected_source_revision: str | None = None,
) -> None:
    """Validate commands, resources, isolation, and delegated behavior."""
    prefix = prefix.expanduser().resolve()
    path_command = prefix / 'bin' / 'lidarslam-map'
    ros_shim = prefix / 'lib' / 'lidarslam' / 'lidarslam-cli'
    historical_node = prefix / 'lib' / 'lidarslam' / 'lidarslam'
    setup_file = prefix / 'setup.bash'
    package_share = prefix / 'share' / 'lidarslam'
    product_root = package_share / 'product'
    product_scripts = product_root / 'scripts'
    bash_completion = product_root / 'completions' / 'lidarslam-map.bash'
    product_schemas = product_root / 'schemas'
    product_build_info = product_root / 'product-build-info.json'
    installed_runtime_manifest = product_root / 'product-runtime-files.txt'

    bytecode_before = _python_bytecode_snapshot(prefix)

    for path in (path_command, ros_shim, historical_node):
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'installed executable is missing: {path}')
    source_runtime_names = _runtime_names(SOURCE_RUNTIME_MANIFEST)
    installed_runtime_names = _runtime_names(installed_runtime_manifest)
    if installed_runtime_names != source_runtime_names:
        raise RuntimeError('installed product runtime manifest differs from source')
    for name in source_runtime_names:
        path = product_scripts / name
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'installed product resource is missing: {path}')
    if not (product_root / 'VERSION').is_file():
        raise RuntimeError(f'installed VERSION is missing under {product_root}')
    for schema_name in (
        'run-manifest-v1.schema.json',
        'run-manifest-v2.schema.json',
        'release-image-v1.schema.json',
        'rollback-plan-v1.schema.json',
        'first-map-validation-receipt-v1.schema.json',
        'first-map-handoff-v1.schema.json',
        'first-map-demo-plan-v1.schema.json',
        'sensor-setup-v1.schema.json',
        'sensor-setup-rejection-v1.schema.json',
        'map-session-recovery-v1.schema.json',
        'map-session-index-v1.schema.json',
        'map-session-catalog-v1.schema.json',
        'map-session-comparison-v1.schema.json',
        'support-bundle-v1.schema.json',
        'map-edit-plan-v1.schema.json',
        'map-edit-receipt-v1.schema.json',
        'map-project-v1.schema.json',
        'map-merge-receipt-v1.schema.json',
        'system-doctor-v1.schema.json',
    ):
        schema_path = product_schemas / schema_name
        if not schema_path.is_file():
            raise RuntimeError(f'installed product schema is missing: {schema_path}')
    if not bash_completion.is_file():
        raise RuntimeError(f'installed Bash completion is missing: {bash_completion}')
    completion_syntax = _run(
        ['bash', '-n', str(bash_completion)],
        prefix,
    )
    _require_success(completion_syntax, 'installed Bash completion syntax')
    if not product_build_info.is_file():
        raise RuntimeError(
            f'installed product build information is missing: {product_build_info}'
        )
    build_info = json.loads(product_build_info.read_text(encoding='utf-8'))
    revision = build_info.get('revision')
    dirty = build_info.get('dirty')
    if not (
        build_info.get('schema_version') == 1
        and isinstance(revision, str)
        and len(revision) == 40
        and all(character in '0123456789abcdef' for character in revision)
        and isinstance(dirty, bool)
        and build_info.get('source') in ('git', 'override')
    ):
        raise RuntimeError(
            f'installed product build information is incomplete: {build_info}'
        )
    if (
        expected_source_revision is not None
        and revision != expected_source_revision
    ):
        raise RuntimeError(
            f'installed source revision {revision} does not match expected '
            f'{expected_source_revision}'
        )
    runner = _load_module(
        product_scripts / 'run_autoware_map_from_bag.py',
        'installed_run_autoware_map_from_bag',
    )
    software_identity = runner._software_identity()
    if (
        software_identity.get('git_commit') != revision
        or software_identity.get('git_dirty') is not dirty
    ):
        raise RuntimeError(
            'installed runner did not consume product build information: '
            f'{software_identity}'
        )
    if (product_scripts / 'gaussian_splatting_train.py').exists():
        raise RuntimeError('research-only scripts leaked into the product install')
    if shutil.which('ros2') is None:
        raise RuntimeError('ros2 is required to validate the installed ROS shim')
    if not setup_file.is_file():
        raise RuntimeError(f'installed setup file is missing: {setup_file}')

    existing_ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    clean_ament_prefix = str(prefix)
    if existing_ament_prefix:
        clean_ament_prefix += os.pathsep + existing_ament_prefix
    ros_env = {'AMENT_PREFIX_PATH': clean_ament_prefix}

    with tempfile.TemporaryDirectory(prefix='lidarslam-installed-cli-') as tmp:
        work_dir = Path(tmp)
        bag_dir = work_dir / 'sample_bag'
        unsupported_bag_dir = work_dir / 'unsupported_bag'
        output_dir = work_dir / 'map_output'
        _write_bag_fixture(bag_dir)
        _write_unsupported_bag_fixture(unsupported_bag_dir)

        path_setup = _run(
            [
                'bash',
                '-c',
                (
                    'set -e; source "$1"; '
                    'command -v lidarslam-map; lidarslam-map --version'
                ),
                'bash',
                str(setup_file),
            ],
            work_dir,
        )
        _require_success(path_setup, 'installed PATH setup')
        if str(path_command) not in path_setup.stdout:
            raise RuntimeError('setup.bash did not expose prefix/bin/lidarslam-map')

        for command in (path_command, ros_shim):
            result = _run([str(command), '--version'], work_dir)
            _require_success(result, f'{command.name} --version')
            if not result.stdout.startswith('lidarslam_ros2 '):
                raise RuntimeError(f'unexpected version output: {result.stdout!r}')

        _validate_installed_help(path_command, work_dir)

        no_command = _run([str(path_command)], work_dir)
        if (
            no_command.returncode != 2
            or no_command.stdout
            or 'Usage:' not in no_command.stderr
        ):
            raise RuntimeError(
                'installed non-interactive no-argument invocation did not '
                'preserve the usage error contract'
            )

        historical_run = work_dir / 'historical_run'
        historical_run.mkdir()
        historical_source = historical_run / 'run_manifest.json'
        historical_source.write_text(
            json.dumps(_historical_manifest_fixture()),
            encoding='utf-8',
        )
        migrated_path = work_dir / 'migrated_manifest.json'
        migration = _run(
            [
                str(path_command),
                'migrate-manifest',
                str(historical_run),
                '--output',
                str(migrated_path),
                '--verification',
                'required',
                '--json',
            ],
            work_dir,
        )
        _require_success(migration, 'installed migrate-manifest')
        migrated = json.loads(migrated_path.read_text(encoding='utf-8'))
        if migrated.get('lifecycle', {}).get('stage') != 'complete':
            raise RuntimeError('installed migration was not inspect-only')
        if json.loads(migration.stdout).get('resume_allowed') is not False:
            raise RuntimeError('installed migration did not refuse resume')

        release_record = work_dir / 'release-image-jazzy.json'
        release_record.write_text(
            json.dumps(_release_image_fixture()),
            encoding='utf-8',
        )
        rollback = _run(
            [
                str(path_command),
                'rollback-plan',
                str(release_record),
                '--json',
            ],
            work_dir,
        )
        _require_success(rollback, 'installed rollback-plan')
        rollback_payload = json.loads(rollback.stdout)
        if rollback_payload.get('moving_tag_mutated') is not False:
            raise RuntimeError('installed rollback plan could move a tag')
        immutable_ref = rollback_payload.get('immutable_ref', '')
        if '@sha256:' not in immutable_ref:
            raise RuntimeError('installed rollback plan is not digest-pinned')

        executables = _run(
            ['ros2', 'pkg', 'executables', 'lidarslam'],
            work_dir,
            ros_env,
        )
        _require_success(executables, 'ros2 package executable discovery')
        discovered = set(executables.stdout.splitlines())
        if 'lidarslam lidarslam' not in discovered:
            raise RuntimeError('historical ROS executable is not discoverable')
        if 'lidarslam lidarslam-cli' not in discovered:
            raise RuntimeError('product CLI ROS shim is not discoverable')

        ros_version = _run(
            ['ros2', 'run', 'lidarslam', 'lidarslam-cli', '--version'],
            work_dir,
            ros_env,
        )
        _require_success(ros_version, 'ros2 run lidarslam lidarslam-cli')
        if not ros_version.stdout.startswith('lidarslam_ros2 '):
            raise RuntimeError(
                f'unexpected ROS shim version output: {ros_version.stdout!r}'
            )

        product_schema = _load_module(
            product_scripts / 'product_schema.py',
            'installed_product_schema',
        )
        system_doctor = _run(
            [
                str(path_command),
                'doctor',
                '--json',
                '--min-free-space-gib',
                '0.001',
            ],
            work_dir,
            clean_ros_environment=True,
        )
        _require_success(
            system_doctor,
            'direct installed system doctor from a fresh shell',
        )
        system_payload = json.loads(system_doctor.stdout)
        product_schema.validate_contract(
            system_payload,
            'system-doctor-v1.schema.json',
        )
        if (
            system_payload.get('status') != 'ready'
            or system_payload.get('mode') != 'system'
            or system_payload.get('findings')
            or system_payload.get('product', {}).get('layout') != 'installed'
            or system_payload.get('writes_performed') is not False
            or system_payload.get('network_accessed') is not False
            or str(prefix) in system_doctor.stdout
        ):
            raise RuntimeError(
                'direct installed system doctor was not ready, read-only, '
                'or privacy-bounded'
            )

        doctor = _run(
            [str(path_command), 'doctor', str(bag_dir), '--json'],
            work_dir,
            clean_ros_environment=True,
        )
        _require_success(doctor, 'direct installed doctor from a fresh shell')
        payload = json.loads(doctor.stdout)
        if payload.get('recommended_profile_id') != 'rko_lio_graph_public_path':
            raise RuntimeError(
                'direct installed doctor did not auto-activate the product '
                'environment or selected an unexpected profile'
            )

        rejection_output = work_dir / 'rejected_session'
        rejection = _run(
            [
                str(path_command),
                'start',
                str(unsupported_bag_dir),
                '--output-dir',
                str(rejection_output),
                '--yes',
                '--dry-run',
                '--json',
            ],
            work_dir,
        )
        if rejection.returncode != 2:
            raise RuntimeError(
                'installed start did not reject an unsupported bag with '
                f'exit 2: {rejection.returncode}'
            )
        rejection_payload = json.loads(rejection.stdout)
        product_schema.validate_contract(
            rejection_payload,
            'sensor-setup-rejection-v1.schema.json',
        )
        if rejection_payload.get('reason', {}).get('code') != (
            'no-maintained-profile'
        ):
            raise RuntimeError('installed start returned an unstable reason code')
        if rejection_payload.get('files_written') is not False:
            raise RuntimeError('installed start rejection claimed to write files')
        if rejection_output.exists():
            raise RuntimeError('installed start rejection created its output')

        demo_work = work_dir / 'demo-dry-run'
        demo_plan = _run(
            [
                str(path_command),
                'demo',
                str(demo_work),
                '--viewer',
                'none',
                '--min-free-space-gib',
                '0.001',
                '--dry-run',
                '--json',
            ],
            work_dir,
        )
        _require_success(demo_plan, 'installed demo --dry-run --json')
        demo_payload = json.loads(demo_plan.stdout)
        product_schema.validate_contract(
            demo_payload,
            'first-map-demo-plan-v1.schema.json',
        )
        if (
            demo_payload.get('status') != 'ready'
            or demo_payload.get('ready') is not True
            or demo_payload.get('cache', {}).get('download_required')
            is not True
            or demo_payload.get('dataset', {}).get('archive_size_bytes')
            != 517088133
        ):
            raise RuntimeError('installed demo returned an invalid dry-run plan')
        if demo_work.exists():
            raise RuntimeError('installed demo dry-run wrote its work directory')

        recovery_session = work_dir / 'failed_session'
        recovery_session.mkdir()
        recovery_map = recovery_session / 'map'
        recovery_map.mkdir()
        recovery_command = [
            str(path_command),
            'run',
            str(bag_dir),
            '--profile',
            'rko_lio_graph_public_path',
            '--output-dir',
            str(recovery_map),
            '--verification',
            'required',
        ]
        (recovery_map / 'run_manifest.json').write_text(
            json.dumps({
                'schema_version': 2,
                'status': 'failed',
                'lifecycle': {'stage': 'complete'},
                'execution': {
                    'finished_at': '2026-08-12T00:00:00Z',
                    'exit_code': 17,
                },
            }),
            encoding='utf-8',
        )
        (recovery_map / 'autoware_map_diagnosis.json').write_text(
            json.dumps({
                'status': 'runtime_failed',
                'problem_hints': [
                    'The output filesystem ran out of writable space or quota.'
                ],
                'verify': {'result': 'unknown'},
            }),
            encoding='utf-8',
        )
        installed_wizard = _load_module(
            product_scripts / 'sensor_setup_wizard.py',
            'installed_sensor_setup_wizard',
        )
        recovery_manifest = {
            'bundle_path': str(recovery_session),
            'input': {'bag_path': str(bag_dir)},
            'profile': {
                'id': 'rko_lio_graph_public_path',
                'label': 'RKO-LIO graph public path',
            },
            'run': {
                'output_dir': str(recovery_map),
                'argv': recovery_command,
            },
        }
        running_payload = installed_wizard._session_index_payload(
            type('Args', (), {
                'verification': 'required',
            })(),
            recovery_manifest,
            runner_exit_code=None,
            running_stage='workflow_running',
            active_run_dir=recovery_map,
        )
        running_path, running_html = installed_wizard._write_session_index(
            recovery_session,
            running_payload,
        )
        product_schema.validate_contract(
            json.loads(running_path.read_text(encoding='utf-8')),
            'map-session-index-v1.schema.json',
        )
        if running_payload.get('quality', {}).get('overall') != 'pending':
            raise RuntimeError(
                'installed live session did not keep quality pending'
            )
        if running_html is None or not running_html.is_file():
            raise RuntimeError(
                'installed start did not create its live session page'
            )
        running_html_text = running_html.read_text(encoding='utf-8')
        if (
            'Mapping sensor data' not in running_html_text
            or 'Map quality' not in running_html_text
            or '<meta http-equiv="refresh" content="2">' not in (
                running_html_text
            )
        ):
            raise RuntimeError(
                'installed live session page omitted durable progress'
            )
        recovery_payload = installed_wizard._session_recovery_payload(
            recovery_manifest,
            17,
        )
        recovery_path = installed_wizard._write_session_recovery(
            recovery_session,
            recovery_payload,
        )
        session_payload = installed_wizard._session_index_payload(
            type('Args', (), {
                'verification': 'required',
            })(),
            recovery_manifest,
            runner_exit_code=17,
            recovery=recovery_payload,
            recovery_path=recovery_path,
        )
        session_path, session_html = installed_wizard._write_session_index(
            recovery_session,
            session_payload,
        )
        if session_html is None or not session_html.is_file():
            raise RuntimeError(
                'installed start recovery did not create session.html'
            )
        if session_payload.get('quality', {}).get('overall') != (
            'action_required'
        ):
            raise RuntimeError(
                'installed recovery session omitted quality action state'
            )
        session_html_text = session_html.read_text(encoding='utf-8')
        if 'storage-exhausted' not in session_html_text:
            raise RuntimeError(
                'installed session browser omitted the stable reason code'
            )
        if (
            '<script src=' in session_html_text
            or '<link ' in session_html_text
        ):
            raise RuntimeError(
                'installed session browser depends on an external resource'
            )
        persisted_recovery = json.loads(
            recovery_path.read_text(encoding='utf-8')
        )
        product_schema.validate_contract(
            persisted_recovery,
            'map-session-recovery-v1.schema.json',
        )
        product_schema.validate_contract(
            json.loads(session_path.read_text(encoding='utf-8')),
            'map-session-index-v1.schema.json',
        )
        if persisted_recovery.get('reason', {}).get('code') != (
            'storage-exhausted'
        ):
            raise RuntimeError(
                'installed start recovery returned an unstable reason code'
            )
        if persisted_recovery.get('retry', {}).get('output_dir') != (
            str(recovery_session / 'map.retry')
        ):
            raise RuntimeError(
                'installed start recovery did not choose a fresh retry output'
            )

        verified_session = work_dir / 'verified_session'
        verified_map = verified_session / 'map'
        verified_map.mkdir(parents=True)
        (verified_map / 'pointcloud_map').mkdir()
        verified_manifest_path = verified_map / 'run_manifest.json'
        verified_diagnosis_path = (
            verified_map / 'autoware_map_diagnosis.json'
        )
        verified_log_path = verified_map / 'verify_autoware_map.log'
        verified_manifest_path.write_text(
            json.dumps({
                'status': 'succeeded',
                'lifecycle': {
                    'stage': 'complete',
                    'runner_exit_code': 0,
                },
            }),
            encoding='utf-8',
        )
        verified_diagnosis_path.write_text(
            json.dumps({'status': 'success'}),
            encoding='utf-8',
        )
        verified_log_path.write_text('RESULT: PASS\n', encoding='utf-8')

        def digest(path: Path) -> str:
            return hashlib.sha256(path.read_bytes()).hexdigest()

        manifest_digest = digest(verified_manifest_path)
        validation_checks = [
            'manifest_succeeded',
            'lifecycle_complete',
            'runner_exit_zero',
            'diagnosis_success',
            'autoware_verification_pass',
            'diagnosis_bound_to_manifest',
            'verify_log_bound_to_manifest',
        ]
        validation_receipt = {
            'schema_version': 1,
            'schema_uri': (
                'https://rsasaki0109.github.io/lidar_slam_ros2/'
                'schemas/first-map-validation-receipt-v1.schema.json'
            ),
            'status': 'PASS',
            'run': {
                'run_id': 'installed-quality-check',
                'product_version': '0.9.0',
                'git_commit': 'a' * 40,
                'profile_id': 'rko_lio_graph_public_path',
            },
            'verification': {
                'manifest_status': 'succeeded',
                'diagnosis_status': 'success',
                'autoware_status': 'PASS',
                'manifest_sha256': manifest_digest,
            },
            'evidence': {
                'manifest': {
                    'filename': 'run_manifest.json',
                    'sha256': manifest_digest,
                },
                'diagnosis': {
                    'filename': 'autoware_map_diagnosis.json',
                    'available': True,
                    'sha256': digest(verified_diagnosis_path),
                },
                'verify_log': {
                    'filename': 'verify_autoware_map.log',
                    'available': True,
                    'sha256': digest(verified_log_path),
                },
            },
            'checks': [
                {
                    'id': check_id,
                    'passed': True,
                    'observed': 'Installed evidence check passed.',
                }
                for check_id in validation_checks
            ],
            'shareability': {
                'contains_map_geometry': False,
                'contains_private_paths': False,
                'contains_exact_command': False,
                'review_before_sharing': True,
            },
        }
        (verified_map / 'first_map_validation_receipt.json').write_text(
            json.dumps(validation_receipt),
            encoding='utf-8',
        )
        verified_receipt_markdown = (
            verified_map / 'first_map_validation_receipt.md'
        )
        verified_receipt_markdown.write_text(
            '# First-map validation receipt\n\nRESULT: PASS\n',
            encoding='utf-8',
        )
        verified_manifest = {
            'bundle_path': str(verified_session),
            'input': {'bag_path': str(bag_dir)},
            'profile': {
                'id': 'rko_lio_graph_public_path',
                'label': 'RKO-LIO graph public path',
            },
            'run': {
                'output_dir': str(verified_map),
                'argv': recovery_command,
            },
        }
        verified_payload = installed_wizard._session_index_payload(
            type('Args', (), {'verification': 'required'})(),
            verified_manifest,
            runner_exit_code=0,
        )
        verified_path, verified_html = (
            installed_wizard._write_session_index(
                verified_session,
                verified_payload,
            )
        )
        product_schema.validate_contract(
            json.loads(verified_path.read_text(encoding='utf-8')),
            'map-session-index-v1.schema.json',
        )
        if verified_payload.get('quality', {}).get('overall') != 'pass':
            raise RuntimeError(
                'installed terminal session did not preserve '
                'receipt-bound PASS'
            )
        if verified_html is None or 'Evidence integrity' not in (
            verified_html.read_text(encoding='utf-8')
        ):
            raise RuntimeError(
                'installed terminal session omitted the quality scorecard'
            )

        session_history_json = _run(
            [
                str(path_command),
                'sessions',
                str(work_dir),
                '--status',
                'verified',
                '--json',
            ],
            work_dir,
        )
        _require_success(session_history_json, 'installed sessions --json')
        history_payload = json.loads(session_history_json.stdout)
        product_schema.validate_contract(
            history_payload,
            'map-session-catalog-v1.schema.json',
        )
        if (
            history_payload.get('summary', {}).get('displayed') != 1
            or history_payload['sessions'][0]['status'] != 'verified'
        ):
            raise RuntimeError(
                'installed session history did not filter the verified run'
            )
        history_page = work_dir / 'sessions.html'
        if history_page.exists():
            raise RuntimeError(
                'installed sessions --json wrote browser output'
            )
        session_history_html = _run(
            [
                str(path_command),
                'sessions',
                str(work_dir),
                '--viewer',
                'none',
            ],
            work_dir,
        )
        _require_success(session_history_html, 'installed sessions browser')
        history_html_text = history_page.read_text(encoding='utf-8')
        if (
            'Recent map sessions' not in history_html_text
            or 'Open session' not in history_html_text
            or 'Copy compare command' not in history_html_text
            or 'Copy support command' not in history_html_text
            or '<script src=' in history_html_text
            or '<link ' in history_html_text
        ):
            raise RuntimeError(
                'installed session history omitted its standalone browser UI'
            )

        comparison_json = _run(
            [
                str(path_command),
                'compare',
                str(verified_session),
                str(recovery_session),
                '--json',
            ],
            work_dir,
        )
        _require_success(comparison_json, 'installed compare --json')
        comparison_payload = json.loads(comparison_json.stdout)
        product_schema.validate_contract(
            comparison_payload,
            'map-session-comparison-v1.schema.json',
        )
        if (
            comparison_payload.get('summary', {}).get('total') != 14
            or comparison_payload.get('policy', {}).get('numeric_score')
            is not False
            or comparison_payload.get('policy', {}).get('winner_selected')
            is not False
        ):
            raise RuntimeError(
                'installed comparison violated its descriptive policy'
            )
        comparison_page = work_dir / 'session-comparison.html'
        if comparison_page.exists():
            raise RuntimeError('installed compare --json wrote browser output')
        comparison_html = _run(
            [
                str(path_command),
                'compare',
                str(verified_session),
                str(recovery_session),
                '--output',
                str(comparison_page),
                '--viewer',
                'none',
            ],
            work_dir,
        )
        _require_success(comparison_html, 'installed compare browser')
        comparison_html_text = comparison_page.read_text(encoding='utf-8')
        if (
            'Compare map sessions' not in comparison_html_text
            or 'does not invent a numeric' not in comparison_html_text
            or '<script src=' in comparison_html_text
            or '<link ' in comparison_html_text
        ):
            raise RuntimeError(
                'installed comparison omitted its standalone browser UI'
            )

        def tree_snapshot(root: Path) -> dict[str, tuple[str, str | int]]:
            snapshot: dict[str, tuple[str, str | int]] = {}
            for path in sorted(root.rglob('*')):
                relative = str(path.relative_to(root))
                if path.is_symlink():
                    snapshot[relative] = ('symlink', os.readlink(path))
                elif path.is_file():
                    snapshot[relative] = ('file', digest(path))
                else:
                    snapshot[relative] = ('directory', 0)
            return snapshot

        before_first_map_handoff = tree_snapshot(work_dir)
        first_map_handoff = _run(
            [
                str(path_command),
                'report',
                str(verified_session),
            ],
            work_dir,
        )
        _require_success(
            first_map_handoff,
            'installed report',
        )
        required_handoff_text = (
            'First-map report: READY FOR REVIEW',
            'Copy into the issue form:',
            'Result: PASS — verified first map completed',
            f'Release, commit, or image digest: {"a" * 40}',
            'Verification summary:',
            'manifest_status=succeeded',
            'diagnosis_status=success',
            'autoware_status=PASS',
            f'manifest_sha256={manifest_digest}',
            str(verified_map / 'first_map_validation_receipt.json'),
            (
                'https://github.com/rsasaki0109/lidar_slam_ros2/'
                'issues/new?template=first-map-validation.yml'
            ),
            'Attach after review:',
            'Complete before submitting:',
            'Public documentation path: <Docker First Map',
            'Redacted command shape: <executable and options',
            'Privacy: use REDACTED for credentials',
        )
        if any(
            text not in first_map_handoff.stdout
            for text in required_handoff_text
        ):
            raise RuntimeError(
                'installed report omitted its review handoff'
            )
        if tree_snapshot(work_dir) != before_first_map_handoff:
            raise RuntimeError(
                'installed report changed local session state'
            )

        before_receipt_only_handoff = tree_snapshot(work_dir)
        receipt_only_handoff = _run(
            [
                str(path_command),
                'report',
                str(verified_map),
                '--json',
            ],
            work_dir,
        )
        _require_success(
            receipt_only_handoff,
            'installed report from fixed-demo output',
        )
        receipt_only_payload = json.loads(receipt_only_handoff.stdout)
        product_schema.validate_contract(
            receipt_only_payload,
            'first-map-handoff-v1.schema.json',
        )
        if (
            receipt_only_payload.get('status') != 'READY_FOR_REVIEW'
            or receipt_only_payload.get('receipt_status') != 'PASS'
            or receipt_only_payload.get('run', {}).get('profile_id')
            != 'rko_lio_graph_public_path'
            or receipt_only_payload.get('verification_summary', '').find(
                f'manifest_sha256={manifest_digest}'
            ) < 0
        ):
            raise RuntimeError(
                'installed report rejected the receipt-only fixed-demo output'
            )
        if tree_snapshot(work_dir) != before_receipt_only_handoff:
            raise RuntimeError(
                'installed receipt-only report changed local session state'
            )

        support_zip = work_dir / 'installed-support.zip'
        support_json = _run(
            [
                str(path_command),
                'support',
                str(verified_session),
                '--json',
            ],
            work_dir,
        )
        _require_success(support_json, 'installed support --json')
        support_payload = json.loads(support_json.stdout)
        product_schema.validate_contract(
            support_payload,
            'support-bundle-v1.schema.json',
        )
        privacy = support_payload.get('privacy', {})
        if (
            privacy.get('contains_map_geometry') is not False
            or privacy.get('contains_raw_sensor_data') is not False
            or privacy.get('contains_raw_logs') is not False
            or privacy.get('contains_parameter_contents') is not False
            or privacy.get('local_paths_redacted') is not True
            or privacy.get('command_secrets_redacted') is not True
            or privacy.get('review_before_sharing') is not True
        ):
            raise RuntimeError(
                'installed support report violated its privacy contract'
            )
        if str(work_dir) in support_json.stdout or support_zip.exists():
            raise RuntimeError(
                'installed support --json leaked a path or wrote a ZIP'
            )
        support_archive = _run(
            [
                str(path_command),
                'support',
                str(verified_session),
                '--output',
                str(support_zip),
            ],
            work_dir,
        )
        _require_success(support_archive, 'installed support ZIP')
        with zipfile.ZipFile(support_zip) as archive:
            if archive.namelist() != [
                'README.txt',
                'issue-body.md',
                'support-report.json',
            ]:
                raise RuntimeError(
                    'installed support ZIP has an unexpected member set'
                )
            support_members = '\n'.join(
                archive.read(name).decode('utf-8')
                for name in archive.namelist()
            )
        if str(work_dir) in support_members:
            raise RuntimeError('installed support ZIP leaked a local path')
        if 'Review all three ZIP members' not in support_archive.stdout:
            raise RuntimeError(
                'installed support ZIP omitted its public-sharing warning'
            )

        dry_run = _run(
            [
                str(path_command),
                'run',
                str(bag_dir),
                '--output-dir',
                str(output_dir),
                '--dry-run',
            ],
            work_dir,
        )
        _require_success(dry_run, 'installed run --dry-run')
        expected_script = (
            product_scripts / 'run_rko_lio_graph_autoware_dogfood.sh'
        )
        expected_param = prefix / 'share' / 'lidarslam' / 'param' / 'lidarslam.yaml'
        if str(expected_script) not in dry_run.stdout:
            raise RuntimeError('dry-run did not resolve the installed workflow script')
        if str(expected_param) not in dry_run.stdout:
            raise RuntimeError('dry-run did not resolve the installed parameter file')
        if output_dir.exists():
            raise RuntimeError('installed dry-run created its output directory')

        output_dir.mkdir()
        inspect = _run(
            [str(path_command), 'inspect', str(output_dir), '--json'],
            work_dir,
        )
        _require_success(inspect, 'installed inspect')
        if json.loads(inspect.stdout).get('status') != 'incomplete':
            raise RuntimeError('installed inspect returned an unexpected status')

        view_help = _run(
            [str(path_command), 'view', '--help'],
            work_dir,
        )
        _require_success(view_help, 'installed view --help')
        if '--viewer {browser,autoware,foxglove}' not in view_help.stdout:
            raise RuntimeError('installed view help is missing viewer choices')
        if '--runtime-dir' in view_help.stdout:
            raise RuntimeError('installed default view help leaked advanced options')
        view_help_all = _run(
            [str(path_command), 'view', '--help-all'],
            work_dir,
        )
        _require_success(view_help_all, 'installed view --help-all')
        if '--runtime-dir' not in view_help_all.stdout:
            raise RuntimeError('installed full view help is missing runtime options')

        view_incomplete = _run(
            [str(path_command), 'view', str(output_dir)],
            work_dir,
        )
        if view_incomplete.returncode != 2:
            raise RuntimeError(
                'installed view did not reject an incomplete map output'
            )
        if 'map output is incomplete' not in view_incomplete.stderr:
            raise RuntimeError(
                'installed view returned an unexpected incomplete-output error'
            )

    bytecode_after = _python_bytecode_snapshot(prefix)
    if bytecode_after != bytecode_before:
        changed_bytecode = sorted(
            path
            for path in set(bytecode_before) | set(bytecode_after)
            if bytecode_before.get(path) != bytecode_after.get(path)
        )
        raise RuntimeError(
            'installed product CLI changed Python bytecode in its install '
            f'prefix: {changed_bytecode[:5]}'
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--prefix',
        type=Path,
        required=True,
        help='Clean CMake install prefix to validate.',
    )
    parser.add_argument(
        '--expected-source-revision',
        help='Require the installed product to record this exact Git commit.',
    )
    args = parser.parse_args()
    try:
        validate_install(args.prefix, args.expected_source_revision)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=__import__('sys').stderr)
        return 1
    print(f'installed product CLI validated: {args.prefix.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
