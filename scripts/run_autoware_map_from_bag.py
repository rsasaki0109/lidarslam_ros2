#!/usr/bin/env python3
"""Run the shortest supported Autoware-compatible map-authoring path for a bag."""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Sequence
import uuid
import xml.etree.ElementTree as ET

import yaml

try:
    from product_profiles import PROFILE_HELP, PROFILE_IDS, select_profile
except ModuleNotFoundError as exc:
    if exc.name != 'product_profiles':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from product_profiles import PROFILE_HELP, PROFILE_IDS, select_profile
    finally:
        sys.path.pop(0)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (REPO_ROOT / 'lidarslam' / 'package.xml').is_file()
PACKAGE_SHARE = REPO_ROOT / 'lidarslam' if SOURCE_LAYOUT else REPO_ROOT.parent
SHARE_ROOT = PACKAGE_SHARE.parent
WORK_ROOT = REPO_ROOT if SOURCE_LAYOUT else Path.cwd()
MANIFEST_NAME = 'run_manifest.json'
VALIDATION_RECEIPT_NAMES = {
    'first_map_validation_receipt.json',
    'first_map_validation_receipt.md',
}
VALIDATION_ISSUE_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/issues/new'
    '?template=first-map-validation.yml'
)
PRODUCT_BUILD_INFO_PATH = REPO_ROOT / 'product-build-info.json'
MANIFEST_SCHEMA_VERSION = 2
MANIFEST_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/run-manifest-v2.schema.json'
)
WORKFLOW_SHUTDOWN_GRACE_SECS = 10.0
DEFAULT_MIN_FREE_SPACE_GIB = 5.0
EMERGENCY_EVIDENCE_RESERVE_BYTES = 2 * 1024 * 1024
EMERGENCY_EVIDENCE_RESERVE_NAME = '.terminal-evidence-reserve'
PRODUCT_SESSION_OUTPUT_ENV = 'LIDARSLAM_PRODUCT_SESSION_OUTPUT'
PRODUCT_SESSION_OUTPUT_CONCISE = 'concise'
CONCISE_WORKFLOW_LOG_NAME = 'map_workflow.log'
CONCISE_WORKFLOW_FAILURE_TAIL_LINES = 80
CONCISE_WORKFLOW_STATUS_LINES = frozenset({
    'Mapping is ready; processing the recorded sensor data ...',
    'Mapping is ready; waiting for the first map output ...',
})
PACKAGE_XML_PATHS = (
    SHARE_ROOT / 'lidarslam' / 'package.xml',
    SHARE_ROOT / 'graph_based_slam' / 'package.xml',
    SHARE_ROOT / 'lidarslam_msgs' / 'package.xml',
    SHARE_ROOT / 'scanmatcher' / 'package.xml',
    (
        REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'package.xml'
        if SOURCE_LAYOUT
        else SHARE_ROOT / 'rko_lio' / 'package.xml'
    ),
)


class _TerminationRequested(Exception):
    """Represent an external termination signal while the workflow is active."""

    def __init__(self, signum: int):
        super().__init__(signum)
        self.signum = signum


def _terminate_process_group(
    process: subprocess.Popen,
    signum: int,
    grace_secs: float = WORKFLOW_SHUTDOWN_GRACE_SECS,
) -> None:
    """Forward a signal to the isolated workflow group and reap its leader."""
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        process.wait()
        return

    deadline = time.monotonic() + grace_secs
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            process.wait()
            return
        except PermissionError:
            pass
        time.sleep(0.05)

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def _run_workflow(
    command: list[str],
    cwd: Path,
    concise_output: bool = False,
    workflow_log: Path | None = None,
) -> tuple[int, bool, str | None]:
    """Run a workflow in an isolated process group with durable signal results."""
    process: subprocess.Popen | None = None
    log_stream = None
    returncode = None

    def request_termination(signum, _frame):
        raise _TerminationRequested(signum)

    previous_sigterm = signal.signal(signal.SIGTERM, request_termination)
    requested_signal = None
    try:
        popen_options: dict[str, object] = {}
        if concise_output:
            if workflow_log is None:
                raise ValueError(
                    'concise workflow output requires a durable log path'
                )
            log_stream = workflow_log.open('w', encoding='utf-8')
            popen_options.update({
                'stdout': subprocess.PIPE,
                'text': True,
                'encoding': 'utf-8',
                'errors': 'replace',
                'bufsize': 1,
            })
        process = subprocess.Popen(
            command,
            cwd=cwd,
            start_new_session=True,
            **popen_options,
        )
        try:
            if concise_output:
                if process.stdout is None or log_stream is None:
                    raise RuntimeError(
                        'concise workflow output pipe was not created'
                    )
                for line in process.stdout:
                    log_stream.write(line)
                    log_stream.flush()
                    if line.rstrip('\r\n') in CONCISE_WORKFLOW_STATUS_LINES:
                        print(line, end='', flush=True)
            returncode = process.wait()
        except KeyboardInterrupt:
            requested_signal = signal.SIGINT
        except _TerminationRequested as exc:
            requested_signal = exc.signum
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        if process is not None and process.stdout is not None:
            process.stdout.close()
        if log_stream is not None:
            log_stream.close()

    if requested_signal is not None:
        if process is None:
            raise RuntimeError(
                'workflow supervision ended without a process result'
            )
        _terminate_process_group(process, requested_signal)
        signal_name = signal.Signals(requested_signal).name
        return (
            128 + requested_signal,
            True,
            f'map workflow interrupted by {signal_name}',
        )

    if returncode is None:
        raise RuntimeError('workflow supervision ended without a process result')
    if concise_output and returncode != 0 and workflow_log is not None:
        _print_concise_workflow_failure(workflow_log)
    return returncode, False, None


def _print_concise_workflow_failure(workflow_log: Path) -> None:
    """Replay bounded child stdout only when a concise workflow fails."""
    try:
        with workflow_log.open(encoding='utf-8', errors='replace') as stream:
            recent_lines = deque(
                stream,
                maxlen=CONCISE_WORKFLOW_FAILURE_TAIL_LINES,
            )
    except OSError as exc:
        print(
            f'warning: failed to read retained workflow output: {exc}',
            file=sys.stderr,
        )
        return

    if recent_lines:
        print('Map workflow failed. Recent workflow details:', file=sys.stderr)
        for line in recent_lines:
            print(
                line,
                end='' if line.endswith('\n') else '\n',
                file=sys.stderr,
            )
    print(f'Complete workflow output: {workflow_log}', file=sys.stderr)


def _terminal_workflow_error(
    status: str,
    exit_code: int,
) -> str | None:
    """Return the stable terminal error recorded after post-processing."""
    if status == 'interrupted':
        signal_names = {
            130: 'SIGINT',
            143: 'SIGTERM',
        }
        signal_name = signal_names.get(exit_code)
        if signal_name:
            return f'map workflow interrupted by {signal_name}'
        return f'map workflow interrupted with exit code {exit_code}'
    if exit_code != 0:
        return f'map workflow exited with code {exit_code}'
    return None


def _load_script_module(script_name: str, module_name: str):
    script_path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load module {module_name} from {script_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_guided(args: argparse.Namespace) -> int:
    """Run the human-facing confirmation layer without duplicating the runner."""
    guided = _load_script_module('lidarslam_guided.py', 'lidarslam_guided')
    guided_args = [args.bag]
    if args.profile:
        guided_args.extend(['--profile', args.profile])
    if args.output_dir:
        guided_args.extend(['--output-dir', args.output_dir])
    guided_args.extend([
        '--min-free-space-gib',
        str(args.min_free_space_gib),
        '--verification',
        'required' if args.verification_enabled else 'off',
    ])
    if args.viewer != 'none':
        guided_args.extend(['--viewer', args.viewer])
    if args.yes:
        guided_args.append('--yes')
    if args.dry_run:
        guided_args.append('--dry-run')
    if args.editable:
        guided_args.append('--editable')
    return guided.main(guided_args)


def _select_profile(payload: dict[str, object], forced_profile_id: str | None) -> str:
    return select_profile(payload, forced_profile_id)


def build_execution_plan(
    bag_path: Path,
    profile_id: str | None,
    output_dir: Path,
    verify_map: bool,
    pointcloud_inspector=None,
    timestamp_inspector=None,
    preflight_payload: dict[str, object] | None = None,
    lidarslam_param: Path | None = None,
    rko_param: Path | None = None,
    base_frame: str = 'base_link',
    lidar_frame: str | None = None,
    imu_frame: str | None = None,
    editable: bool = False,
) -> dict[str, object]:
    preflight = _load_script_module('preflight_autoware_map_bag.py', 'preflight_autoware_map_bag')
    payload = preflight_payload or preflight.build_preflight_payload(
        bag_path,
        pointcloud_inspector=pointcloud_inspector,
        timestamp_inspector=timestamp_inspector,
    )
    selected_profile = _select_profile(payload, profile_id)

    recommendations = {item['id']: item for item in payload['recommendations']}
    if selected_profile not in recommendations:
        available_profiles = ', '.join(recommendations) if recommendations else 'none'
        raise RuntimeError(
            f'profile is not compatible with this bag: {selected_profile}. '
            f'Available profiles: {available_profiles}. '
            'Run preflight to inspect the detected topics and missing requirements.'
        )

    summary = payload['summary']
    output_dir = output_dir.expanduser().resolve()
    custom_rko_params = lidarslam_param is not None or rko_param is not None
    if custom_rko_params and (lidarslam_param is None or rko_param is None):
        raise ValueError(
            '--lidarslam-param and --rko-param must be provided together'
        )
    if custom_rko_params and selected_profile not in {
        'rko_lio_graph_public_path',
        'rko_lio_graph_mid360_preset',
    }:
        raise ValueError(
            'custom RKO-LIO parameter files require an rko_lio_graph profile'
        )
    for label, path in (
        ('--lidarslam-param', lidarslam_param),
        ('--rko-param', rko_param),
    ):
        if path is not None and not path.expanduser().resolve().is_file():
            raise ValueError(f'{label} file does not exist: {path}')

    if selected_profile == 'rko_lio_graph_public_path':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param', str(
                lidarslam_param.expanduser().resolve()
                if lidarslam_param is not None
                else PACKAGE_SHARE / 'param' / 'lidarslam.yaml'
            ),
            '--rko-param', str(
                rko_param.expanduser().resolve()
                if rko_param is not None
                else PACKAGE_SHARE / 'param' / 'rko_lio_ntu_viral.yaml'
            ),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'rko_lio_graph_mid360_preset':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param',
            str(
                lidarslam_param.expanduser().resolve()
                if lidarslam_param is not None
                else PACKAGE_SHARE / 'param' / 'lidarslam_mid360_rko_graph.yaml'
            ),
            '--rko-param', str(PACKAGE_SHARE / 'param' / 'rko_lio_mid360.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'pointcloud_gnss_smoke':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        gnss = summary['topics']['navsatfix'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_open_data_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--points-topic', pointcloud,
            '--gnss-topic', gnss,
            '--param', str(PACKAGE_SHARE / 'param' / 'lidarslam.yaml'),
            '--save-dir', str(output_dir),
        ]
        if summary['capabilities']['has_imu']:
            command.extend(['--imu-topic', summary['topics']['imu'][0]['name']])
        if verify_map:
            command.append('--verify-map')
    elif selected_profile == 'packet_applanix_smoke':
        packet = summary['topics']['velodyne_scan'][0]['name']
        gsof49 = summary['topics']['applanix_gsof49'][0]['name']
        command = [
            'bash',
            str(SCRIPT_DIR / 'run_open_data_applanix_velodyne_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--packet-topic', packet,
            '--gsof49-topic', gsof49,
            '--param', str(PACKAGE_SHARE / 'param' / 'lidarslam.yaml'),
            '--save-dir', str(output_dir),
        ]
        if summary['capabilities']['has_applanix_gsof50']:
            command.extend(['--gsof50-topic', summary['topics']['applanix_gsof50'][0]['name']])
        if verify_map:
            command.append('--verify-map')
    else:
        raise RuntimeError(f'profile is not executable yet: {selected_profile}')

    if selected_profile in {
        'rko_lio_graph_public_path',
        'rko_lio_graph_mid360_preset',
    }:
        if selected_profile == 'rko_lio_graph_mid360_preset' and rko_param is not None:
            rko_index = command.index('--rko-param') + 1
            command[rko_index] = str(rko_param.expanduser().resolve())
        command.extend(['--base-frame', base_frame])
        if lidar_frame:
            command.extend(['--lidar-frame', lidar_frame])
        if imu_frame:
            command.extend(['--imu-frame', imu_frame])

    if editable:
        if selected_profile not in {
            'rko_lio_graph_public_path',
            'rko_lio_graph_mid360_preset',
        }:
            raise ValueError(
                '--editable requires an rko_lio_graph profile because only '
                'that backend has deterministic loop replay support'
            )
        command = [
            'bash',
            str(SCRIPT_DIR / 'record_backend_input.sh'),
            '--output-dir',
            str(output_dir / 'backend_input'),
            '--',
            *command,
        ]

    return {
        'payload': payload,
        'profile_id': selected_profile,
        'label': recommendations[selected_profile]['label'],
        'command': command,
        'output_dir': output_dir,
    }


def validate_bag_path(bag_path: Path) -> None:
    if bag_path.is_file():
        if bag_path.suffix == '.db3':
            raise FileNotFoundError(
                f'rosbag2 path points to a .db3 file: {bag_path}. '
                'Pass the rosbag2 directory that contains metadata.yaml, not the .db3 file.'
            )
        raise FileNotFoundError(
            f'rosbag2 path is a file, not a directory: {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    if not bag_path.exists():
        raise FileNotFoundError(
            f'rosbag2 directory does not exist: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not bag_path.is_dir():
        raise FileNotFoundError(
            f'rosbag2 path is not a directory: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )


def validate_output_dir(output_dir: Path) -> None:
    if output_dir.exists() and not output_dir.is_dir():
        raise ValueError(f'output directory path is a file: {output_dir}')
    if output_dir.exists():
        raise ValueError(
            f'output directory already exists: {output_dir}. '
            'Choose a new directory; existing outputs are never overwritten.'
        )

    for parent in output_dir.parents:
        if parent.exists():
            if not parent.is_dir():
                raise ValueError(f'output directory parent is not a directory: {parent}')
            return


def _minimum_free_space_gib(value: str) -> float:
    """Parse a positive finite output-storage reserve."""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a number greater than zero') from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError('must be a finite number greater than zero')
    return parsed


def _positive_seconds(value: str) -> int:
    """Parse a positive whole-second duration."""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'must be a positive integer'
        ) from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return parsed


def check_output_storage(
    output_dir: Path,
    minimum_free_space_gib: float,
) -> dict[str, object]:
    """Refuse a run whose output filesystem is below the configured reserve."""
    probe_path = output_dir
    while not probe_path.exists():
        parent = probe_path.parent
        if parent == probe_path:
            raise ValueError(
                f'cannot find an existing parent for output directory: {output_dir}'
            )
        probe_path = parent
    if not probe_path.is_dir():
        raise ValueError(f'output storage probe path is not a directory: {probe_path}')

    required_free_bytes = math.ceil(minimum_free_space_gib * 1024**3)
    try:
        observed_free_bytes = shutil.disk_usage(probe_path).free
    except OSError as exc:
        raise ValueError(
            f'cannot inspect free space for output directory {output_dir}: {exc}'
        ) from exc
    if observed_free_bytes < required_free_bytes:
        observed_gib = observed_free_bytes / 1024**3
        raise ValueError(
            'insufficient free space for map output: '
            f'require at least {minimum_free_space_gib:.2f} GiB under '
            f'{probe_path}, but only {observed_gib:.2f} GiB is available. '
            'Free storage or choose another --output-dir; lower '
            '--min-free-space-gib only after sizing the expected map.'
        )
    return {
        'probe_path': str(probe_path.resolve()),
        'required_free_bytes': required_free_bytes,
        'observed_free_bytes': observed_free_bytes,
    }


def _allocate_emergency_evidence_reserve(working_dir: Path) -> Path:
    """Reserve real blocks that can be released for terminal evidence."""
    reserve_path = working_dir / EMERGENCY_EVIDENCE_RESERVE_NAME
    descriptor = os.open(
        reserve_path,
        os.O_CREAT | os.O_EXCL | os.O_WRONLY,
        0o600,
    )
    try:
        os.posix_fallocate(
            descriptor,
            0,
            EMERGENCY_EVIDENCE_RESERVE_BYTES,
        )
    except OSError:
        reserve_path.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)
    return reserve_path


def _release_emergency_evidence_reserve(reserve_path: Path) -> None:
    """Release reserved blocks before manifest and diagnosis finalization."""
    reserve_path.unlink()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _file_identity(path: Path, display_path: str) -> dict[str, object]:
    before = path.stat()
    digest = _sha256(path)
    after = path.stat()
    if (
        before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
        or before.st_ino != after.st_ino
    ):
        raise RuntimeError(f'file changed while hashing: {path}')
    return {
        'path': display_path,
        'size_bytes': after.st_size,
        'sha256': digest,
    }


def _git_state() -> dict[str, object]:
    if not SOURCE_LAYOUT:
        try:
            payload = json.loads(
                PRODUCT_BUILD_INFO_PATH.read_text(encoding='utf-8')
            )
        except (OSError, json.JSONDecodeError):
            return {'commit': None, 'dirty': None}
        commit = payload.get('revision')
        dirty = payload.get('dirty')
        if not (
            payload.get('schema_version') == 1
            and payload.get('source') in ('git', 'override', 'unknown')
            and (
                commit is None
                or (
                    isinstance(commit, str)
                    and len(commit) == 40
                    and all(character in '0123456789abcdef' for character in commit)
                )
            )
            and (dirty is None or isinstance(dirty, bool))
        ):
            return {'commit': None, 'dirty': None}
        return {'commit': commit, 'dirty': dirty}
    commit_result = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    status_result = subprocess.run(
        ['git', 'status', '--porcelain', '--untracked-files=no'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return {
        'commit': (
            commit_result.stdout.strip()
            if commit_result.returncode == 0
            else None
        ),
        'dirty': (
            bool(status_result.stdout.strip())
            if status_result.returncode == 0
            else None
        ),
    }


def _product_version() -> str:
    return (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()


def _package_versions() -> dict[str, str]:
    versions = {}
    for package_xml in PACKAGE_XML_PATHS:
        if not package_xml.is_file():
            continue
        root = ET.parse(package_xml).getroot()
        name = root.findtext('name')
        version = root.findtext('version')
        if name and version:
            versions[name.strip()] = version.strip()
    return dict(sorted(versions.items()))


def _software_identity() -> dict[str, object]:
    git_state = _git_state()
    return {
        'product_version': _product_version(),
        'git_commit': git_state['commit'],
        'git_dirty': git_state['dirty'],
        'package_versions': _package_versions(),
        'ros_distro': os.environ.get('ROS_DISTRO'),
    }


def _bag_identity(bag_path: Path) -> dict[str, object]:
    metadata_path = bag_path / 'metadata.yaml'
    metadata_before = metadata_path.stat()
    metadata_bytes = metadata_path.read_bytes()
    metadata_after = metadata_path.stat()
    if (
        metadata_before.st_size != metadata_after.st_size
        or metadata_before.st_mtime_ns != metadata_after.st_mtime_ns
        or metadata_before.st_ino != metadata_after.st_ino
    ):
        raise RuntimeError(f'file changed while reading: {metadata_path}')
    metadata = yaml.safe_load(metadata_bytes) or {}
    bag_info = metadata.get('rosbag2_bagfile_information') or {}
    relative_paths = bag_info.get('relative_file_paths') or []
    if not isinstance(relative_paths, list) or not all(
        isinstance(path, str) for path in relative_paths
    ):
        raise ValueError(
            'rosbag2 metadata relative_file_paths must be a list of strings'
        )
    if not relative_paths:
        relative_paths = [
            path.relative_to(bag_path).as_posix()
            for pattern in ('*.db3', '*.mcap')
            for path in sorted(bag_path.glob(pattern))
        ]

    storage_files = []
    seen_paths = set()
    resolved_bag_path = bag_path.resolve()
    for relative_path in relative_paths:
        path = (bag_path / str(relative_path)).resolve()
        try:
            normalized_relative = path.relative_to(resolved_bag_path).as_posix()
        except ValueError as exc:
            raise ValueError(
                f'rosbag2 metadata references a file outside the bag: {relative_path}'
            ) from exc
        if normalized_relative in seen_paths:
            continue
        if not path.is_file():
            raise ValueError(
                f'rosbag2 storage file referenced by metadata is missing: {path}'
            )
        seen_paths.add(normalized_relative)
        storage_files.append(_file_identity(path, normalized_relative))

    return {
        'bag_path': str(bag_path),
        'metadata_path': str(metadata_path),
        'metadata_size_bytes': metadata_after.st_size,
        'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
        'storage_identifier': bag_info.get('storage_identifier'),
        'storage_files': storage_files,
        'identity_algorithm': 'sha256',
    }


def _write_manifest(run_dir: Path, manifest: dict[str, object]) -> None:
    destination = run_dir / MANIFEST_NAME
    temporary = run_dir / f'.{MANIFEST_NAME}.tmp'
    try:
        temporary.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, destination)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def _announce_lifecycle(stage: str, action: str) -> None:
    """Report a durable stage without inventing percentage or ETA."""
    if _concise_product_session_output():
        return
    print(f'Lifecycle stage: {stage} — {action}', flush=True)


def _concise_product_session_output() -> bool:
    """Return whether a parent start session owns the human progress view."""
    return (
        os.environ.get(PRODUCT_SESSION_OUTPUT_ENV)
        == PRODUCT_SESSION_OUTPUT_CONCISE
    )


def _print_execution_summary(
    plan: dict[str, object],
    output_dir: Path,
    working_dir: Path,
    storage_preflight: dict[str, object],
) -> None:
    """Print a concise start handoff or the complete direct-run plan."""
    observed_gib = storage_preflight['observed_free_bytes'] / 1024**3
    required_gib = storage_preflight['required_free_bytes'] / 1024**3
    if _concise_product_session_output():
        print(f"Map profile: {plan['label']}")
        print(
            f'Storage check: {observed_gib:.2f} GiB available '
            f'({required_gib:.2f} GiB required)',
            flush=True,
        )
        return

    print(f"Selected profile: {plan['label']}")
    print(f'Output directory: {output_dir}')
    print(f'Atomic working directory: {working_dir}')
    print(
        'Storage preflight: '
        f'{observed_gib:.2f} GiB free; '
        f'{required_gib:.2f} GiB required '
        f"under {storage_preflight['probe_path']}"
    )
    print('Command:')
    print('  ' + shlex.join(plan['command']), flush=True)


def _artifact_checksums(run_dir: Path) -> list[dict[str, object]]:
    artifacts = []
    for path in sorted(item for item in run_dir.rglob('*') if item.is_file()):
        relative = path.relative_to(run_dir)
        if (
            relative.as_posix() == MANIFEST_NAME
            or relative.name.startswith(f'.{MANIFEST_NAME}.')
            or relative.as_posix() in VALIDATION_RECEIPT_NAMES
            or relative.name.startswith('.first_map_validation_receipt.')
        ):
            continue
        artifacts.append(_file_identity(path, relative.as_posix()))
    return artifacts


def _build_manifest(
    bag_path: Path,
    final_output_dir: Path,
    working_output_dir: Path,
    plan: dict[str, object],
    verify_map: bool = True,
) -> dict[str, object]:
    return {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'schema_uri': MANIFEST_SCHEMA_URI,
        'run_id': str(uuid.uuid4()),
        'status': 'planned',
        'lifecycle': {
            'stage': 'initialized',
            'resume_count': 0,
            'verification_enabled': verify_map,
            'runner_exit_code': None,
            'last_error': None,
        },
        'input': _bag_identity(bag_path),
        'software': _software_identity(),
        'profile': {
            'id': plan['profile_id'],
            'label': plan['label'],
        },
        'execution': {
            'argv': plan['command'],
            'command_shell': shlex.join(plan['command']),
            'started_at': None,
            'finished_at': None,
            'exit_code': None,
        },
        'output': {
            'requested_dir': str(final_output_dir),
            'working_dir': str(working_output_dir),
            'finalized': False,
            'artifact_checksums': [],
        },
    }


def _load_manifest(run_dir: Path) -> dict[str, object]:
    manifest_path = run_dir / MANIFEST_NAME
    if not manifest_path.is_file():
        raise ValueError(f'run manifest not found: {manifest_path}')
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'run manifest is not readable JSON: {manifest_path}: {exc}') from exc
    if not isinstance(manifest, dict):
        raise ValueError(f'run manifest root must be an object: {manifest_path}')
    return manifest


def _validate_resume_state(
    bag_path: Path,
    final_output_dir: Path,
    working_output_dir: Path,
    plan: dict[str, object],
    verify_map: bool,
) -> tuple[Path, dict[str, object]]:
    existing_dirs = [
        path for path in (working_output_dir, final_output_dir) if path.exists()
    ]
    if len(existing_dirs) != 1:
        if not existing_dirs:
            raise ValueError(
                'no resumable output found. Expected exactly one of '
                f'{working_output_dir} or {final_output_dir}'
            )
        raise ValueError(
            'ambiguous resume state: both the partial and final output exist: '
            f'{working_output_dir}, {final_output_dir}'
        )

    run_dir = existing_dirs[0]
    if not run_dir.is_dir():
        raise ValueError(f'resume output path is not a directory: {run_dir}')
    manifest = _load_manifest(run_dir)
    if manifest.get('schema_version') != MANIFEST_SCHEMA_VERSION:
        raise ValueError(
            'resume requires run manifest schema v2; older manifests remain '
            'inspectable but cannot be resumed safely'
        )
    if manifest.get('schema_uri') != MANIFEST_SCHEMA_URI:
        raise ValueError('resume manifest schema_uri does not match schema v2')

    lifecycle = manifest.get('lifecycle')
    execution = manifest.get('execution')
    output = manifest.get('output')
    if not all(isinstance(item, dict) for item in (lifecycle, execution, output)):
        raise ValueError('resume manifest is missing lifecycle, execution, or output state')

    stage = lifecycle.get('stage')
    valid_stages = {
        'initialized',
        'workflow_running',
        'workflow_finished',
        'verifying',
        'verified',
        'finalizing',
        'finalized',
        'diagnosing',
        'diagnosed',
        'checksumming',
        'complete',
    }
    if stage not in valid_stages:
        raise ValueError(f'resume manifest has an unknown lifecycle stage: {stage!r}')
    if stage in ('initialized', 'workflow_running'):
        raise ValueError(
            f'refusing resume from lifecycle stage {stage!r}: the workflow may '
            'still be running, so starting post-processing could corrupt evidence'
        )
    if stage == 'complete':
        raise ValueError(
            'run is already complete; use `./scripts/lidarslam inspect '
            f'{final_output_dir}` instead'
        )
    if execution.get('finished_at') is None or execution.get('exit_code') is None:
        raise ValueError('resume requires a durably recorded terminal workflow result')
    if lifecycle.get('verification_enabled') is not verify_map:
        raise ValueError(
            'resume verification option mismatch; use the same '
            '--verification mode as the original run'
        )

    expected_values = (
        ('input identity', manifest.get('input'), _bag_identity(bag_path)),
        ('software identity', manifest.get('software'), _software_identity()),
        (
            'profile',
            manifest.get('profile'),
            {'id': plan['profile_id'], 'label': plan['label']},
        ),
        ('execution argv', execution.get('argv'), plan['command']),
        ('requested output', output.get('requested_dir'), str(final_output_dir)),
        ('working output', output.get('working_dir'), str(working_output_dir)),
    )
    for label, actual, expected in expected_values:
        if actual != expected:
            raise ValueError(
                f'resume {label} mismatch; use the original bag, software, '
                'profile, options, and output path'
            )

    if run_dir == working_output_dir and output.get('finalized') is True:
        raise ValueError('partial output claims it was already finalized')

    resume_count = lifecycle.get('resume_count')
    if not isinstance(resume_count, int) or isinstance(resume_count, bool):
        raise ValueError('resume manifest lifecycle.resume_count must be an integer')
    lifecycle['resume_count'] = resume_count + 1
    workflow_exit_code = execution['exit_code']
    if manifest.get('status') == 'interrupted':
        if workflow_exit_code not in (130, 143):
            raise ValueError(
                'interrupted resume state requires workflow exit code 130 or 143'
            )
        manifest['status'] = 'interrupted'
    else:
        manifest['status'] = 'succeeded' if workflow_exit_code == 0 else 'failed'
    return run_dir, manifest


def _normalize_ros_log_latest(working_dir: Path) -> None:
    """Keep ROS's transaction-local ``latest`` link valid after rename."""
    log_dir = working_dir / '.ros_log'
    latest = log_dir / 'latest'
    if log_dir.is_symlink() or not latest.is_symlink():
        return

    target = Path(os.readlink(latest))
    if not target.is_absolute():
        return

    normalized_log_dir = Path(os.path.normpath(str(log_dir)))
    normalized_target = Path(os.path.normpath(str(target)))
    try:
        relative_target = normalized_target.relative_to(normalized_log_dir)
    except ValueError:
        return

    rebased_target = log_dir / relative_target
    if not rebased_target.exists():
        latest.unlink()
        return

    portable_target = os.path.relpath(rebased_target, start=log_dir)
    temporary = latest.with_name(f'.latest.{uuid.uuid4().hex}.tmp')
    try:
        temporary.symlink_to(
            portable_target,
            target_is_directory=rebased_target.is_dir(),
        )
        os.replace(temporary, latest)
    except OSError:
        temporary.unlink(missing_ok=True)
        raise


def _finalize_output(working_dir: Path, final_dir: Path) -> None:
    if final_dir.exists():
        raise RuntimeError(
            f'output collision detected before finalization: {final_dir}'
        )
    _normalize_ros_log_latest(working_dir)
    os.replace(working_dir, final_dir)


def maybe_open_viewer(args: argparse.Namespace, output_dir: Path) -> None:
    """Delegate deprecated combined run/view requests to the view command."""
    if args.viewer == 'none':
        return

    print(
        'warning: run viewer options are deprecated; run the map first, then '
        'use "lidarslam-map view <output_dir> --viewer '
        f'{args.viewer}".',
        file=sys.stderr,
    )
    command = [
        sys.executable,
        str(SCRIPT_DIR / 'view_autoware_map.py'),
        str(output_dir),
        '--viewer',
        args.viewer,
    ]
    if args.autoware_core_dir:
        command.extend(['--autoware-core-dir', args.autoware_core_dir])
    if args.work_dir:
        command.extend(['--work-dir', args.work_dir])
    if args.viewer_run_dir:
        command.extend(['--runtime-dir', args.viewer_run_dir])
    if args.viewer_rebuild:
        command.append('--rebuild')
    if args.auto_exit_secs is not None:
        command.extend(['--auto-exit-secs', str(args.auto_exit_secs)])

    subprocess.run(command, check=True, cwd=WORK_ROOT)


def maybe_verify_map(output_dir: Path, enabled: bool) -> None:
    if not enabled:
        return

    pointcloud_map_dir = output_dir / 'pointcloud_map'
    if not pointcloud_map_dir.is_dir():
        return

    verify_log_path = output_dir / 'verify_autoware_map.log'
    verify_command = [
        'python3',
        str(SCRIPT_DIR / 'verify_autoware_map.py'),
        str(pointcloud_map_dir),
    ]
    with verify_log_path.open('w', encoding='utf-8') as stream:
        result = subprocess.run(
            verify_command,
            check=False,
            cwd=WORK_ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
        )
    if result.returncode != 0:
        print(f'Warning: verify_autoware_map.py failed. See {verify_log_path}')


def print_next_steps(args: argparse.Namespace, output_dir: Path) -> None:
    print('Next steps:')
    print(
        '  Diagnosis: '
        f'python3 scripts/diagnose_autoware_map_run.py {shlex.quote(str(output_dir))} --write'
    )
    verify_log = output_dir / 'verify_autoware_map.log'
    if verify_log.is_file():
        print(f'  Verify log: {verify_log}')
    print(f'  Saved map:  {output_dir / "pointcloud_map"}')
    lanelet2_map = output_dir / 'lanelet2_map.osm'
    if lanelet2_map.is_file():
        print(f'  Lanelet2:   {lanelet2_map}')
    receipt_json = output_dir / 'first_map_validation_receipt.json'
    if receipt_json.is_file():
        print(f'  Review shareable receipt: {receipt_json}')
        print(f'  Report this run (PASS or FAIL): {VALIDATION_ISSUE_URL}')

    if args.viewer == 'none':
        print(
            '  Open 3D preview: '
            f'lidarslam-map view {shlex.quote(str(output_dir))}'
        )
        print(
            '  Open in Foxglove: '
            'lidarslam-map view '
            f'{shlex.quote(str(output_dir))} --viewer foxglove'
        )
        print(
            '  Open in Autoware viewer: '
            f'lidarslam-map view {shlex.quote(str(output_dir))} '
            '--viewer autoware'
        )


def write_diagnostics(output_dir: Path, bag_path: Path) -> dict[str, object]:
    diagnose = _load_script_module('diagnose_autoware_map_run.py', 'diagnose_autoware_map_run')
    summary = diagnose.summarize_run(output_dir, bag_path)
    markdown = diagnose.render_markdown(summary)
    (output_dir / 'autoware_map_diagnosis.md').write_text(markdown, encoding='utf-8')
    (output_dir / 'autoware_map_diagnosis.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding='utf-8',
    )
    return summary


def write_first_map_validation_receipt(
    output_dir: Path,
) -> tuple[dict[str, object], dict[str, Path]]:
    """Write the privacy-bounded external validation receipt."""
    receipt_module = _load_script_module(
        'first_map_validation_receipt.py',
        'first_map_validation_receipt',
    )
    return receipt_module.write_receipt(output_dir)


@contextmanager
def _postprocess_lock(output_dir: Path):
    lock_path = output_dir.with_name(f'.{output_dir.name}.postprocess.lock')
    try:
        stream = lock_path.open('a+', encoding='utf-8')
    except OSError as exc:
        raise RuntimeError(f'cannot open post-processing lock {lock_path}: {exc}') from exc
    try:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                f'post-processing is already active for {output_dir}'
            ) from exc
        yield
    finally:
        stream.close()


def _postprocess_run(
    args: argparse.Namespace,
    manifest: dict[str, object],
    bag_path: Path,
    working_dir: Path,
    output_dir: Path,
    run_dir: Path,
) -> int:
    lifecycle = manifest['lifecycle']
    execution = manifest['execution']
    workflow_exit_code = execution['exit_code']
    current_dir = run_dir
    try:
        _write_manifest(current_dir, manifest)
        lifecycle['stage'] = 'verifying'
        _write_manifest(current_dir, manifest)
        _announce_lifecycle('verifying', 'verify generated map artifacts')
        maybe_verify_map(current_dir, enabled=args.verification_enabled)
        lifecycle['stage'] = 'verified'
        _write_manifest(current_dir, manifest)

        if current_dir == working_dir:
            lifecycle['stage'] = 'finalizing'
            _write_manifest(current_dir, manifest)
            _announce_lifecycle('finalizing', 'publish output atomically')
            _finalize_output(working_dir, output_dir)
            current_dir = output_dir
        elif current_dir != output_dir:
            raise RuntimeError(f'unexpected resume directory: {current_dir}')

        manifest['output']['finalized'] = True
        lifecycle['stage'] = 'finalized'
        _write_manifest(current_dir, manifest)

        lifecycle['stage'] = 'diagnosing'
        _write_manifest(current_dir, manifest)
        _announce_lifecycle('diagnosing', 'build operator diagnosis')
        diagnosis = write_diagnostics(current_dir, bag_path)
        manifest['output']['diagnosis_status'] = diagnosis['status']
        lifecycle['stage'] = 'diagnosed'
        _write_manifest(current_dir, manifest)

        interrupted = manifest['status'] == 'interrupted'
        if interrupted:
            runner_exit_code = workflow_exit_code
        elif workflow_exit_code != 0:
            runner_exit_code = workflow_exit_code
            manifest['status'] = 'failed'
        elif args.verification_enabled and diagnosis['status'] != 'success':
            runner_exit_code = 1
            manifest['status'] = 'failed'
        else:
            runner_exit_code = 0
            manifest['status'] = 'succeeded'

        lifecycle['stage'] = 'checksumming'
        _write_manifest(current_dir, manifest)
        _announce_lifecycle(
            'checksumming',
            'bind artifacts and validation receipt',
        )
        manifest['output']['artifact_checksums'] = _artifact_checksums(current_dir)
        lifecycle['stage'] = 'complete'
        lifecycle['runner_exit_code'] = runner_exit_code
        lifecycle['last_error'] = _terminal_workflow_error(
            manifest['status'],
            workflow_exit_code,
        )
        _write_manifest(current_dir, manifest)
        receipt, _ = write_first_map_validation_receipt(current_dir)
        if (
            manifest['status'] == 'succeeded'
            and lifecycle['verification_enabled']
            and receipt['status'] != 'PASS'
        ):
            raise RuntimeError(
                'successful run did not produce a passing first-map '
                'validation receipt'
            )
        return runner_exit_code
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(f'error: failed to finalize output: {exc}', file=sys.stderr)
        manifest_dir = output_dir if output_dir.exists() else working_dir
        if manifest_dir.exists():
            if manifest.get('status') not in ('failed', 'interrupted'):
                manifest['status'] = 'failed'
            manifest['output']['finalized'] = output_dir.exists()
            lifecycle['runner_exit_code'] = 70
            lifecycle['last_error'] = str(exc)
            try:
                manifest['output']['artifact_checksums'] = _artifact_checksums(
                    manifest_dir
                )
                _write_manifest(manifest_dir, manifest)
                try:
                    write_first_map_validation_receipt(manifest_dir)
                except (OSError, RuntimeError, ValueError) as receipt_exc:
                    print(
                        'warning: failed to preserve first-map validation '
                        f'receipt: {receipt_exc}',
                        file=sys.stderr,
                    )
            except (OSError, RuntimeError) as manifest_exc:
                print(
                    f'warning: failed to preserve terminal manifest: {manifest_exc}',
                    file=sys.stderr,
                )
        return 70


def _postprocess_with_lock(
    args: argparse.Namespace,
    manifest: dict[str, object],
    bag_path: Path,
    working_dir: Path,
    output_dir: Path,
    run_dir: Path,
) -> int:
    try:
        with _postprocess_lock(output_dir):
            return _postprocess_run(
                args,
                manifest,
                bag_path,
                working_dir,
                output_dir,
                run_dir,
            )
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 70


def _profile_help_text() -> str:
    lines = ['Workflow profiles:']
    for profile_id, description in PROFILE_HELP:
        lines.append(f'  {profile_id}: {description}')
    return '\n'.join(lines)


def _help_epilog() -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/run_autoware_map_from_bag.py',
    )
    return '\n'.join([
        'The input must be the rosbag2 directory that contains metadata.yaml.',
        'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.',
        '',
        _profile_help_text(),
        '',
        'Expected successful outputs:',
        '  pointcloud_map/',
        '  map_projector_info.yaml',
        '  verify_autoware_map.log',
        '  autoware_map_diagnosis.md',
        '  run_manifest.json',
        '  first_map_validation_receipt.md',
        '',
        'Examples:',
        f'  {command} /path/to/rosbag2 --dry-run',
        f'  {command} /path/to/rosbag2 --output-dir output/my_map',
        f'  {command} /path/to/rosbag2 --guided',
        f'  {command} /path/to/rosbag2 --output-dir output/my_map --resume',
    ])


def parse_args(
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    """Parse the map-run product options."""
    show_all_help = os.environ.get('LIDARSLAM_CLI_HELP_MODE') != 'core'

    def extended_help(text: str) -> str:
        return text if show_all_help else argparse.SUPPRESS

    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Inspect a rosbag2 directory, choose a supported Autoware-compatible '
            'map workflow, and write map artifacts under output/ by default.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(),
    )
    parser.add_argument(
        'bag',
        metavar='rosbag2_dir',
        help='Directory containing metadata.yaml.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show advanced and deprecated options.',
    )
    map_options = parser.add_argument_group('map selection and output')
    map_options.add_argument(
        '--profile',
        choices=PROFILE_IDS,
        metavar='<id>',
        help='Force a compatible profile instead of the default recommendation.',
    )
    map_options.add_argument(
        '--output-dir',
        metavar='<dir>',
        help=(
            'Directory for generated map outputs and logs. Defaults to '
            'output/autoware_map_authoring_<bag>_<timestamp>.'
        ),
    )
    setup_options = parser.add_argument_group('sensor setup bundle overrides')
    setup_options.add_argument(
        '--lidarslam-param',
        metavar='<file>',
        help='graph_based_slam YAML generated by "lidarslam-map setup".',
    )
    setup_options.add_argument(
        '--rko-param',
        metavar='<file>',
        help='RKO-LIO YAML generated by "lidarslam-map setup".',
    )
    setup_options.add_argument(
        '--base-frame',
        default='base_link',
        metavar='<frame>',
        help='Robot base frame (default: base_link).',
    )
    setup_options.add_argument(
        '--lidar-frame',
        metavar='<frame>',
        help='LiDAR frame override detected or selected during setup.',
    )
    setup_options.add_argument(
        '--imu-frame',
        metavar='<frame>',
        help='IMU frame override detected or selected during setup.',
    )
    safety_options = parser.add_argument_group('safety and lifecycle')
    safety_options.add_argument(
        '--min-free-space-gib',
        type=_minimum_free_space_gib,
        default=DEFAULT_MIN_FREE_SPACE_GIB,
        metavar='<GiB>',
        help=(
            'Refuse to start when the output filesystem has less than this '
            f'reserve (default: {DEFAULT_MIN_FREE_SPACE_GIB:g} GiB).'
        ),
    )
    safety_options.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the selected command without executing it.',
    )
    safety_options.add_argument(
        '--editable',
        action='store_true',
        help=(
            'Retain deterministic backend replay input with the map so '
            'accepted loop constraints can be disabled later.'
        ),
    )
    safety_options.add_argument(
        '--resume',
        action='store_true',
        help=(
            'Resume verification, finalization, diagnosis, and checksums for a '
            'terminal schema-v2 run; the map workflow is never re-executed.'
        ),
    )
    safety_options.add_argument(
        '--guided',
        action='store_true',
        help=(
            'Show the detected inputs and exact plan, ask for confirmation, '
            'then delegate to this same runner.'
        ),
    )
    safety_options.add_argument(
        '--yes',
        action='store_true',
        help='With --guided, start without asking for confirmation.',
    )
    viewer_options = parser.add_argument_group(
        'deprecated viewer compatibility options'
    )
    viewer_options.add_argument(
        '--viewer',
        choices=['none', 'browser', 'autoware', 'foxglove'],
        default='none',
        help=extended_help(
            'Deprecated: open the saved map after the run. Prefer '
            '"lidarslam-map view <output_dir>" (default: none).'
        ),
    )
    advanced_viewer_options = parser.add_argument_group(
        'deprecated advanced viewer compatibility options'
    )
    advanced_viewer_options.add_argument(
        '--autoware-core-dir',
        metavar='<dir>',
        help=extended_help('autoware_core checkout used by the Docker viewer.'),
    )
    advanced_viewer_options.add_argument(
        '--work-dir',
        metavar='<dir>',
        help=extended_help(
            'Runtime workspace directory for Autoware/Foxglove viewers.'
        ),
    )
    advanced_viewer_options.add_argument(
        '--viewer-run-dir',
        metavar='<dir>',
        help=extended_help('Existing built viewer runtime to reuse.'),
    )
    advanced_viewer_options.add_argument(
        '--viewer-rebuild',
        action='store_true',
        help=extended_help('Rebuild the viewer runtime before opening.'),
    )
    advanced_viewer_options.add_argument(
        '--auto-exit-secs',
        type=_positive_seconds,
        metavar='<seconds>',
        help=extended_help('Auto-close the viewer after N seconds.'),
    )
    verification_options = parser.add_argument_group(
        'verification'
    )
    verification_options.add_argument(
        '--verification',
        choices=['required', 'off'],
        help=(
            'Map verification mode (default: required). Use off only for '
            'diagnosis; an unverified run is never reported as verified.'
        ),
    )
    verification_options.add_argument(
        '--no-verify-map',
        action='store_true',
        help=extended_help('Deprecated alias for "--verification off".'),
    )
    return parser.parse_args(argv)


def validate_option_combinations(args: argparse.Namespace) -> None:
    """Reject viewer options that would otherwise be silently ignored."""
    viewer_specific = (
        ('--autoware-core-dir', args.autoware_core_dir),
        ('--work-dir', args.work_dir),
        ('--viewer-run-dir', args.viewer_run_dir),
        ('--viewer-rebuild', args.viewer_rebuild),
        ('--auto-exit-secs', args.auto_exit_secs is not None),
    )
    if args.autoware_core_dir and args.viewer != 'autoware':
        raise ValueError('--autoware-core-dir requires --viewer autoware')
    active = [name for name, enabled in viewer_specific if enabled]
    if args.viewer in {'none', 'browser'} and active:
        joined = ', '.join(active)
        raise ValueError(
            f'{joined} requires --viewer autoware or --viewer foxglove'
        )
    if args.yes and not args.guided:
        raise ValueError('--yes requires --guided')
    if bool(args.lidarslam_param) != bool(args.rko_param):
        raise ValueError(
            '--lidarslam-param and --rko-param must be provided together'
        )
    if args.guided and any((
        args.lidarslam_param,
        args.rko_param,
        args.base_frame != 'base_link',
        args.lidar_frame,
        args.imu_frame,
    )):
        raise ValueError(
            'sensor setup bundle overrides cannot be combined with --guided; '
            'run the exact command written by "lidarslam-map setup"'
        )
    if args.guided and args.resume:
        raise ValueError(
            '--guided cannot be combined with --resume; use run --resume '
            'directly for terminal post-processing'
        )
    if args.guided and any(
        value
        for _name, value in (
            ('--autoware-core-dir', args.autoware_core_dir),
            ('--work-dir', args.work_dir),
            ('--viewer-run-dir', args.viewer_run_dir),
            ('--viewer-rebuild', args.viewer_rebuild),
            ('--auto-exit-secs', args.auto_exit_secs is not None),
        )
    ):
        raise ValueError(
            '--guided cannot be combined with advanced viewer options; '
            'run the map first, then use "lidarslam-map view"'
        )


def resolve_verification_mode(args: argparse.Namespace) -> bool:
    """Return whether map verification is required and emit migration warnings."""
    if args.no_verify_map and args.verification is not None:
        raise ValueError(
            '--no-verify-map cannot be combined with --verification; use '
            '"--verification off"'
        )
    if args.no_verify_map:
        print(
            'warning: --no-verify-map is deprecated; use "--verification off". '
            'Map verification is disabled.',
            file=sys.stderr,
        )
        return False
    if args.verification == 'off':
        print(
            'warning: map verification is disabled; a successful workflow '
            'exit will not be reported as verified.',
            file=sys.stderr,
        )
        return False
    return True


def main() -> int:
    """Run or resume a map workflow under the product lifecycle contract."""
    args = parse_args()
    try:
        validate_option_combinations(args)
        args.verification_enabled = resolve_verification_mode(args)
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.resume and args.dry_run:
        print('error: --resume cannot be combined with --dry-run', file=sys.stderr)
        return 2
    if args.resume and not args.output_dir:
        print('error: --resume requires an explicit --output-dir', file=sys.stderr)
        return 2
    if args.guided:
        try:
            return _run_guided(args)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            print(f'error: guided workflow could not start: {exc}', file=sys.stderr)
            return 70

    bag_path = Path(args.bag).expanduser().resolve()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else (
        WORK_ROOT / 'output' / f'autoware_map_authoring_{bag_path.stem}_{timestamp}'
    )
    working_dir = output_dir.with_name(f'{output_dir.name}.partial')
    storage_preflight = None
    try:
        validate_bag_path(bag_path)
        if not args.resume:
            validate_output_dir(output_dir)
            validate_output_dir(working_dir)
        storage_preflight = check_output_storage(
            output_dir,
            args.min_free_space_gib,
        )
        plan_arguments = {
            'bag_path': bag_path,
            'profile_id': args.profile,
            'output_dir': working_dir,
            'verify_map': args.verification_enabled,
        }
        if args.editable:
            plan_arguments['editable'] = True
        if any((
            args.lidarslam_param,
            args.rko_param,
            args.base_frame != 'base_link',
            args.lidar_frame,
            args.imu_frame,
        )):
            plan_arguments.update({
                'lidarslam_param': (
                    Path(args.lidarslam_param) if args.lidarslam_param else None
                ),
                'rko_param': Path(args.rko_param) if args.rko_param else None,
                'base_frame': args.base_frame,
                'lidar_frame': args.lidar_frame,
                'imu_frame': args.imu_frame,
            })
        plan = build_execution_plan(
            **plan_arguments,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    _print_execution_summary(
        plan,
        output_dir,
        working_dir,
        storage_preflight,
    )

    if args.dry_run:
        return 0
    if args.resume:
        try:
            with _postprocess_lock(output_dir):
                run_dir, manifest = _validate_resume_state(
                    bag_path,
                    output_dir,
                    working_dir,
                    plan,
                    args.verification_enabled,
                )
                print(f'Resuming terminal post-processing from: {run_dir}')
                exit_code = _postprocess_run(
                    args,
                    manifest,
                    bag_path,
                    working_dir,
                    output_dir,
                    run_dir,
                )
        except ValueError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 2
        except RuntimeError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 70
        return _finish_run(args, plan, output_dir, exit_code)

    created_working_dir = False
    emergency_reserve = None
    try:
        manifest = _build_manifest(
            bag_path,
            output_dir,
            working_dir,
            plan,
            verify_map=args.verification_enabled,
        )
        working_dir.mkdir(parents=True, exist_ok=False)
        created_working_dir = True
        emergency_reserve = _allocate_emergency_evidence_reserve(working_dir)
        manifest['status'] = 'running'
        manifest['lifecycle']['stage'] = 'workflow_running'
        manifest['execution']['started_at'] = _utc_now()
        _write_manifest(working_dir, manifest)
        _announce_lifecycle('workflow_running', 'run the mapping workflow')
    except (OSError, RuntimeError, ValueError, ET.ParseError, yaml.YAMLError) as exc:
        if emergency_reserve is not None:
            try:
                _release_emergency_evidence_reserve(emergency_reserve)
            except OSError:
                pass
        if created_working_dir and not (working_dir / MANIFEST_NAME).exists():
            try:
                working_dir.rmdir()
            except OSError:
                pass
        print(
            f'error: failed to initialize output directory {working_dir}: {exc}',
            file=sys.stderr,
        )
        return 2

    exit_code = 0
    interrupted = False
    workflow_error = None
    try:
        exit_code, interrupted, workflow_error = _run_workflow(
            plan['command'],
            WORK_ROOT,
            _concise_product_session_output(),
            working_dir / CONCISE_WORKFLOW_LOG_NAME,
        )
    except OSError as exc:
        print(f'error: failed to start map workflow: {exc}', file=sys.stderr)
        exit_code = 70
        workflow_error = f'failed to start map workflow: {exc}'
    finally:
        if emergency_reserve is not None:
            try:
                _release_emergency_evidence_reserve(emergency_reserve)
            except OSError as exc:
                print(
                    'error: failed to release terminal evidence reserve: '
                    f'{exc}',
                    file=sys.stderr,
                )
                exit_code = 70
                workflow_error = (
                    f'failed to release terminal evidence reserve: {exc}'
                )

    manifest['execution']['exit_code'] = exit_code
    manifest['execution']['finished_at'] = _utc_now()
    manifest['status'] = (
        'interrupted' if interrupted else ('succeeded' if exit_code == 0 else 'failed')
    )
    manifest['lifecycle']['stage'] = 'workflow_finished'
    manifest['lifecycle']['last_error'] = (
        workflow_error
        or _terminal_workflow_error(manifest['status'], exit_code)
    )
    exit_code = _postprocess_with_lock(
        args,
        manifest,
        bag_path,
        working_dir,
        output_dir,
        working_dir,
    )
    return _finish_run(args, plan, output_dir, exit_code)


def _finish_run(
    args: argparse.Namespace,
    plan: dict[str, object],
    output_dir: Path,
    exit_code: int,
) -> int:
    if exit_code != 0:
        stop_label = _sealed_interruption_label(output_dir, exit_code)
        if stop_label is not None:
            print(
                f'Map stopped by {stop_label}; retained evidence: '
                f'{output_dir}',
                file=sys.stderr,
            )
        else:
            print(
                f'error: map run failed with exit code {exit_code}.',
                file=sys.stderr,
            )
            print(
                'failed command:',
                shlex.join(plan['command']),
                file=sys.stderr,
            )
        if (output_dir / 'autoware_map_diagnosis.md').is_file():
            print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
        if (output_dir / 'first_map_validation_receipt.md').is_file():
            print(
                'First-map receipt: '
                f'{output_dir / "first_map_validation_receipt.md"}'
            )
        receipt_json = output_dir / 'first_map_validation_receipt.json'
        if receipt_json.is_file():
            print(f'Review shareable receipt: {receipt_json}')
            print(
                'Report this run (PASS or FAIL): '
                f'{VALIDATION_ISSUE_URL}'
            )
        return exit_code

    concise_output = _concise_product_session_output()
    if not concise_output:
        print_next_steps(args, output_dir)
    try:
        maybe_open_viewer(args, output_dir)
    except subprocess.CalledProcessError as exc:
        print(f'error: viewer failed with exit code {exc.returncode}.', file=sys.stderr)
        return exc.returncode or 1
    if not concise_output:
        print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
        print(f'Run manifest: {output_dir / MANIFEST_NAME}')
        print(
            'First-map receipt: '
            f'{output_dir / "first_map_validation_receipt.md"}'
        )
    return 0


def _sealed_interruption_label(
    output_dir: Path,
    exit_code: int,
) -> str | None:
    """Classify only a complete, evidence-bound operator interruption."""
    labels = {
        130: ('SIGINT', 'Ctrl-C'),
        143: ('SIGTERM', 'SIGTERM'),
    }
    signal = labels.get(exit_code)
    if signal is None:
        return None
    signal_name, label = signal
    try:
        manifest = json.loads(
            (output_dir / MANIFEST_NAME).read_text(encoding='utf-8')
        )
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    execution = manifest.get('execution')
    lifecycle = manifest.get('lifecycle')
    if not isinstance(execution, dict) or not isinstance(lifecycle, dict):
        return None
    if (
        manifest.get('status') != 'interrupted'
        or execution.get('exit_code') != exit_code
        or lifecycle.get('stage') != 'complete'
        or lifecycle.get('runner_exit_code') != exit_code
        or lifecycle.get('last_error')
        != f'map workflow interrupted by {signal_name}'
    ):
        return None
    return label


if __name__ == '__main__':
    raise SystemExit(main())
