#!/usr/bin/env python3
"""Generate a reviewable, reproducible sensor setup bundle from rosbag2."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import html
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
from string import Template
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Sequence

import yaml

try:
    from product_profiles import PROFILE_HELP, select_profile
except ModuleNotFoundError as exc:
    if exc.name != 'product_profiles':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from product_profiles import PROFILE_HELP, select_profile
    finally:
        sys.path.pop(0)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (REPO_ROOT / 'lidarslam' / 'package.xml').is_file()
PACKAGE_SHARE = REPO_ROOT / 'lidarslam' if SOURCE_LAYOUT else REPO_ROOT.parent
WORK_ROOT = REPO_ROOT if SOURCE_LAYOUT else Path.cwd()
SCHEMA_VERSION = 1
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/sensor-setup-v1.schema.json'
)
REJECTION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/sensor-setup-rejection-v1.schema.json'
)
RECOVERY_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/map-session-recovery-v1.schema.json'
)
SESSION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/map-session-index-v1.schema.json'
)
RECOVERY_NAME = 'map_session_recovery.json'
SESSION_NAME = 'session.json'
SESSION_HTML_NAME = 'session.html'
MAP_PREVIEW_RELATIVE_PATH = (
    Path('preview') / 'mid360_robot_3d_map_preview.html'
)
SESSION_PROGRESS_POLL_SECONDS = 0.25
SESSION_PROGRESS_HEARTBEAT_SECONDS = 30.0
SESSION_INTERRUPT_GRACE_SECONDS = 20.0
SESSION_INTERRUPT_TERMINATE_SECONDS = 10.0
PRODUCT_SESSION_OUTPUT_ENV = 'LIDARSLAM_PRODUCT_SESSION_OUTPUT'
PRODUCT_SESSION_OUTPUT_CONCISE = 'concise'
SESSION_PROGRESS_STAGES = {
    'preparing': ('preparing', 1, 'Preparing pinned session'),
    'initialized': ('preparing', 1, 'Preparing output'),
    'workflow_running': ('mapping', 2, 'Mapping sensor data'),
    'workflow_finished': ('verifying', 3, 'Map workflow finished'),
    'verifying': ('verifying', 3, 'Verifying map quality'),
    'verified': ('verifying', 3, 'Map verification passed'),
    'finalizing': ('finalizing', 4, 'Finalizing map output'),
    'finalized': ('finalizing', 4, 'Map output finalized'),
    'diagnosing': ('evidence', 5, 'Writing diagnostics'),
    'diagnosed': ('evidence', 5, 'Diagnostics ready'),
    'checksumming': ('evidence', 5, 'Checksumming evidence'),
    'preparing_preview': ('evidence', 6, 'Preparing 3D map review'),
    'complete': ('complete', 6, 'Session complete'),
}
VALIDATION_RECEIPT_NAME = 'first_map_validation_receipt.json'
VALIDATION_RECEIPT_SCHEMA = 'first-map-validation-receipt-v1.schema.json'
VALIDATION_CHECK_IDS = (
    'manifest_succeeded',
    'lifecycle_complete',
    'runner_exit_zero',
    'diagnosis_success',
    'autoware_verification_pass',
    'diagnosis_bound_to_manifest',
    'verify_log_bound_to_manifest',
)
DEFAULT_MIN_FREE_SPACE_GIB = 5.0
RKO_PROFILES = {
    'rko_lio_graph_public_path': (
        'lidarslam.yaml',
        'rko_lio_ntu_viral.yaml',
    ),
    'rko_lio_graph_mid360_preset': (
        'lidarslam_mid360_rko_graph.yaml',
        'rko_lio_mid360.yaml',
    ),
}
PROFILE_INPUTS = {
    'rko_lio_graph_public_path': {
        'lidar_key': 'pointcloud2',
        'lidar_type': 'sensor_msgs/msg/PointCloud2',
        'imu_key': 'imu',
        'gnss_key': None,
        'navigation_key': None,
        'navigation_quality_key': None,
    },
    'rko_lio_graph_mid360_preset': {
        'lidar_key': 'pointcloud2',
        'lidar_type': 'sensor_msgs/msg/PointCloud2',
        'imu_key': 'imu',
        'gnss_key': None,
        'navigation_key': None,
        'navigation_quality_key': None,
    },
    'pointcloud_gnss_smoke': {
        'lidar_key': 'pointcloud2',
        'lidar_type': 'sensor_msgs/msg/PointCloud2',
        'imu_key': 'imu',
        'gnss_key': 'navsatfix',
        'navigation_key': None,
        'navigation_quality_key': None,
    },
    'packet_applanix_smoke': {
        'lidar_key': 'velodyne_scan',
        'lidar_type': 'velodyne_msgs/msg/VelodyneScan',
        'imu_key': None,
        'gnss_key': None,
        'navigation_key': 'applanix_gsof49',
        'navigation_quality_key': 'applanix_gsof50',
    },
}
SETUP_PROFILE_IDS = tuple(profile_id for profile_id, _ in PROFILE_HELP)
if set(SETUP_PROFILE_IDS) != set(PROFILE_INPUTS):
    raise RuntimeError(
        'every maintained product profile must define a sensor setup input '
        'contract'
    )
TRANSFORM_KEYS = {
    'lidar_to_base': 'extrinsic_lidar2base_quat_xyzw_xyz',
    'imu_to_base': 'extrinsic_imu2base_quat_xyzw_xyz',
}


class _SessionTerminationRequested(Exception):
    """Carry an external termination signal through delegated supervision."""

    def __init__(self, signum: int):
        super().__init__(signum)
        self.signum = signum


def _start_mode() -> bool:
    command = os.environ.get('LIDARSLAM_CLI_COMMAND', '')
    return command.rsplit(' ', 1)[-1] == 'start'


def _safe_output_stem(value: str) -> str:
    rendered = re.sub(r'[^A-Za-z0-9._-]+', '_', value).strip('._-')
    return rendered or 'bag'


def _default_output_dir(bag_path: Path, *, start_mode: bool) -> Path:
    if start_mode:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name = f'lidarslam_{_safe_output_stem(bag_path.stem)}_{timestamp}'
    else:
        name = f'sensor_setup_{_safe_output_stem(bag_path.stem)}'
    return (WORK_ROOT / 'output' / name).resolve()


def _load_script_module(script_name: str, module_name: str):
    path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load {module_name} from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_transform(value: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in value.split(',')]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'expected seven comma-separated numbers: qx,qy,qz,qw,x,y,z'
        ) from exc
    if len(values) != 7 or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError(
            'expected seven finite comma-separated numbers: qx,qy,qz,qw,x,y,z'
        )
    quaternion_norm = math.sqrt(sum(item * item for item in values[:4]))
    if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=0.01):
        raise argparse.ArgumentTypeError(
            'quaternion must be unit length '
            f'(observed norm {quaternion_norm:.6f})'
        )
    return values


def _frame_id(value: str) -> str:
    value = value.strip()
    if not value or any(character.isspace() for character in value):
        raise argparse.ArgumentTypeError(
            'frame id must be non-empty and contain no spaces'
        )
    return value


def _profile_help_text() -> str:
    lines = ['Maintained setup profiles:']
    lines.extend(
        f'  {profile_id}: {description}'
        for profile_id, description in PROFILE_HELP
        if profile_id in SETUP_PROFILE_IDS
    )
    return '\n'.join(lines)


def _help_epilog() -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'lidarslam-map setup',
    )
    examples = [
        f'  {command} /path/to/rosbag2',
    ]
    if _start_mode():
        examples.extend([
            f'  {command} /path/to/rosbag2 --yes --dry-run',
            (
                f'  {command} /path/to/rosbag2 --profile '
                'pointcloud_gnss_smoke --yes --dry-run'
            ),
            f'  {command} /path/to/rosbag2 --editable --viewer browser',
        ])
    else:
        examples.extend([
            f'  {command} /path/to/rosbag2 --accept-profile-extrinsics',
            (
                f'  {command} /path/to/rosbag2 --profile '
                'packet_applanix_smoke'
            ),
            (
                f'  {command} /path/to/rosbag2 '
                '--lidar-to-base 0,0,0,1,0.10,0,0.20 '
                '--imu-to-base 0,0,0,1,0,0,0'
            ),
        ])
    return '\n'.join([
        'Calibration is never guessed. For RKO-LIO, either confirm that the',
        'tracked profile extrinsics match the robot or supply both '
        'transforms.',
        'The GNSS and packet profiles pin their actual input topics.',
        'RKO-LIO transform and frame overrides do not apply to those '
        'profiles.',
        'Transform order is qx,qy,qz,qw,x,y,z (translation in metres).',
        '',
        _profile_help_text(),
        '',
        'Examples:',
        *examples,
    ])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the stable sensor setup command options."""
    start_mode = _start_mode()
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Detect and configure the sensors, create a verified map, and '
            'open the result.'
            if start_mode else
            'Detect sensor topics, timestamps, and frames, then generate a '
            'reviewable parameter bundle and exact map command.'
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
        help='Show all options (this command has no hidden options).',
    )
    parser.add_argument(
        '--profile',
        choices=SETUP_PROFILE_IDS,
        metavar='<id>',
        help='Force a compatible profile instead of automatic selection.',
    )
    parser.add_argument(
        '--output-dir',
        metavar='<dir>',
        help=(
            'New session directory '
            '(default: output/lidarslam_<bag>_<timestamp>).'
            if start_mode else
            'New setup bundle directory '
            '(default: output/sensor_setup_<bag>).'
        ),
    )
    parser.add_argument(
        '--map-output-dir',
        metavar='<dir>',
        help=(
            'Map output used by the generated command '
            '(default: <bundle>/map).'
        ),
    )
    parser.add_argument(
        '--accept-profile-extrinsics',
        action='store_true',
        help='Confirm tracked RKO-LIO extrinsics match this robot (RKO only).',
    )
    parser.add_argument(
        '--lidar-to-base',
        type=_parse_transform,
        metavar='<qx,qy,qz,qw,x,y,z>',
        help='Explicit LiDAR-to-base transform in metres (RKO only).',
    )
    parser.add_argument(
        '--imu-to-base',
        type=_parse_transform,
        metavar='<qx,qy,qz,qw,x,y,z>',
        help='Explicit IMU-to-base transform in metres (RKO only).',
    )
    parser.add_argument(
        '--base-frame',
        type=_frame_id,
        default='base_link',
        metavar='<frame>',
        help='Robot base frame for RKO-LIO (default: base_link).',
    )
    parser.add_argument(
        '--lidar-frame',
        type=_frame_id,
        metavar='<frame>',
        help='Override the PointCloud2 header frame (RKO only).',
    )
    parser.add_argument(
        '--imu-frame',
        type=_frame_id,
        metavar='<frame>',
        help='Override the Imu header frame (RKO only).',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit the result as machine-readable JSON.',
    )
    parser.add_argument(
        '--run',
        dest='run_now',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    parser.set_defaults(
        yes=False,
        dry_run=False,
        editable=False,
        viewer='none',
        min_free_space_gib=DEFAULT_MIN_FREE_SPACE_GIB,
        verification='required',
    )
    if start_mode:
        parser.add_argument(
            '--yes',
            action='store_true',
            help=(
                'Confirm the displayed sensor setup and start without '
                'prompting.'
            ),
        )
        parser.add_argument(
            '--dry-run',
            action='store_true',
            help=(
                'Inspect and print the exact session plan without writing or '
                'mapping.'
            ),
        )
        parser.add_argument(
            '--editable',
            action='store_true',
            help='Retain RKO backend input for later loop editing (RKO only).',
        )
        parser.add_argument(
            '--viewer',
            choices=['none', 'browser', 'autoware', 'foxglove'],
            default='browser',
            help='Open the completed map in this viewer (default: browser).',
        )
        parser.add_argument(
            '--min-free-space-gib',
            type=float,
            default=DEFAULT_MIN_FREE_SPACE_GIB,
            metavar='<GiB>',
            help='Required free-space reserve before mapping (default: 5).',
        )
        parser.add_argument(
            '--verification',
            choices=['required', 'off'],
            default='required',
            help='Map verification mode (default: required).',
        )
    return parser.parse_args(argv)


def _validate_calibration_options(args: argparse.Namespace) -> None:
    custom = args.lidar_to_base is not None or args.imu_to_base is not None
    if custom and (args.lidar_to_base is None or args.imu_to_base is None):
        raise ValueError(
            '--lidar-to-base and --imu-to-base must be provided together'
        )
    if custom and args.accept_profile_extrinsics:
        raise ValueError(
            '--accept-profile-extrinsics cannot be combined with explicit '
            'transforms'
        )


def _selected_frame(
    payload: dict[str, Any],
    msg_type: str,
    override: str | None,
    fallback: str,
) -> tuple[str, str]:
    if override:
        return override, 'cli_override'
    if msg_type == 'sensor_msgs/msg/PointCloud2':
        detected = payload['summary']['pointcloud_inspection'].get('frame_id')
    else:
        detected = None
        for topic in payload['summary']['timestamp_order']['topics']:
            if topic['msg_type'] == msg_type and topic.get('frame_id'):
                detected = topic['frame_id']
                break
    if detected:
        return detected, 'bag_header'
    return fallback, 'fallback'


def _parameter_sources(profile_id: str) -> tuple[Path, Path] | None:
    names = RKO_PROFILES.get(profile_id)
    if names is None:
        return None
    return (
        PACKAGE_SHARE / 'param' / names[0],
        PACKAGE_SHARE / 'param' / names[1],
    )


def _profile_topics(
    summary: dict[str, Any],
    profile_id: str,
) -> dict[str, str | None]:
    """Resolve only the bag topics that the selected workflow consumes."""
    spec = PROFILE_INPUTS[profile_id]

    def select(key: str | None, *, required: bool = False) -> str | None:
        records = summary['topics'].get(key, []) if key else []
        if records:
            return records[0]['name']
        if required and key:
            raise ValueError(
                f'profile input disappeared after preflight: {profile_id} '
                f'requires topic group {key}'
            )
        return None

    return {
        'lidar': select(spec['lidar_key'], required=True),
        'lidar_type': spec['lidar_type'],
        'imu': select(
            spec['imu_key'],
            required=profile_id in RKO_PROFILES,
        ),
        'gnss': select(
            spec['gnss_key'],
            required=spec['gnss_key'] is not None,
        ),
        'navigation': select(
            spec['navigation_key'],
            required=spec['navigation_key'] is not None,
        ),
        'navigation_quality': select(spec['navigation_quality_key']),
    }


def _validate_profile_options(
    profile_id: str,
    args: argparse.Namespace,
) -> None:
    """Reject RKO-only controls instead of silently ignoring them."""
    if profile_id in RKO_PROFILES:
        return
    unsupported = []
    if args.accept_profile_extrinsics:
        unsupported.append('--accept-profile-extrinsics')
    if args.lidar_to_base is not None or args.imu_to_base is not None:
        unsupported.extend(['--lidar-to-base', '--imu-to-base'])
    if args.base_frame != 'base_link':
        unsupported.append('--base-frame')
    if args.lidar_frame is not None:
        unsupported.append('--lidar-frame')
    if args.imu_frame is not None:
        unsupported.append('--imu-frame')
    if args.editable:
        unsupported.append('--editable')
    if unsupported:
        rendered = ', '.join(dict.fromkeys(unsupported))
        raise ValueError(
            f'{profile_id} does not consume these RKO-LIO-only options: '
            f'{rendered}. Remove them or select an rko_lio_graph profile.'
        )


def _profile_transforms(rko_path: Path) -> dict[str, list[float]]:
    payload = yaml.safe_load(rko_path.read_text(encoding='utf-8')) or {}
    if not isinstance(payload, dict):
        raise ValueError(
            f'RKO-LIO parameter YAML root must be an object: {rko_path}'
        )
    transforms = {}
    for public_name, yaml_name in TRANSFORM_KEYS.items():
        value = payload.get(yaml_name)
        if not isinstance(value, list) or len(value) != 7:
            raise ValueError(f'missing valid {yaml_name} in {rko_path}')
        if not all(
            isinstance(item, (int, float)) and math.isfinite(item)
            for item in value
        ):
            raise ValueError(f'missing valid {yaml_name} in {rko_path}')
        transforms[public_name] = [float(item) for item in value]
    return transforms


def _product_command() -> str:
    if SOURCE_LAYOUT:
        return './scripts/lidarslam'
    return 'lidarslam-map'


def _concrete_next_action(value: str, bag_path: Path) -> str:
    """Render preflight recovery guidance as a copy-ready local command."""
    rendered = value.replace('<rosbag2_dir>', shlex.quote(str(bag_path)))
    if SOURCE_LAYOUT:
        rendered = rendered.replace(
            'lidarslam-map ',
            f'{_product_command()} ',
        )
    return rendered


def _not_ready_payload(
    *,
    bag_path: Path,
    payload: dict[str, Any],
    code: str,
    message: str,
    next_command: str,
    findings: Sequence[dict[str, str]] | None = None,
) -> dict[str, Any]:
    """Build a stable, non-writing own-bag rejection contract."""
    summary = payload['summary']
    rendered_findings = []
    for finding in findings if findings is not None else payload['findings']:
        rendered_findings.append({
            'code': finding['code'],
            'message': finding['message'],
            'next_action': _concrete_next_action(
                finding['next_action'],
                bag_path,
            ),
        })
    topic_keys = (
        'pointcloud2',
        'imu',
        'navsatfix',
        'velodyne_scan',
        'applanix_gsof49',
        'applanix_gsof50',
    )
    return {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': REJECTION_SCHEMA_URI,
        'status': 'not_ready',
        'bag_path': str(bag_path),
        'reason': {'code': code, 'message': message},
        'detected': {
            'topics': {
                key: [item['name'] for item in summary['topics'][key]]
                for key in topic_keys
            },
            'pointcloud_check': summary['pointcloud_inspection']['status'],
            'timestamp_check': summary['timestamp_order']['status'],
        },
        'findings': rendered_findings,
        'next_command': next_command,
        'files_written': False,
    }


def _review_payload(
    *,
    bag_path: Path,
    output_dir: Path,
    profile_id: str,
    label: str,
    payload: dict[str, Any],
    transforms: dict[str, list[float]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    subcommand = 'start' if args.run_now else 'setup'
    rerun = [
        _product_command(),
        subcommand,
        str(bag_path),
        '--profile',
        profile_id,
        '--output-dir',
        str(output_dir),
    ]
    rerun.append('--yes' if args.run_now else '--accept-profile-extrinsics')
    if args.map_output_dir:
        rerun.extend(['--map-output-dir', str(args.map_output_dir)])
    if args.run_now:
        if args.editable:
            rerun.append('--editable')
        if args.viewer != 'browser':
            rerun.extend(['--viewer', args.viewer])
        if args.min_free_space_gib != DEFAULT_MIN_FREE_SPACE_GIB:
            rerun.extend([
                '--min-free-space-gib',
                str(args.min_free_space_gib),
            ])
        if args.verification != 'required':
            rerun.extend(['--verification', args.verification])
    summary = payload['summary']
    return {
        'schema_version': SCHEMA_VERSION,
        'status': 'review_required',
        'bag_path': str(bag_path),
        'profile': {'id': profile_id, 'label': label},
        'detected': {
            'lidar_topic': summary['topics']['pointcloud2'][0]['name'],
            'imu_topic': summary['topics']['imu'][0]['name'],
            'point_timestamp_field': summary['pointcloud_inspection'][
                'timestamp_field'
            ],
            'timestamp_order': summary['timestamp_order']['status'],
        },
        'profile_extrinsics': transforms,
        'message': (
            'Confirm that these tracked profile extrinsics match the physical '
            'robot, or rerun with both explicit transforms.'
        ),
        'next_command': shlex.join(rerun),
    }


def _write_parameter_snapshot(
    source: Path,
    destination: Path,
    custom_transforms: dict[str, list[float]] | None,
) -> None:
    if custom_transforms is None:
        shutil.copy2(source, destination)
        return
    payload = yaml.safe_load(source.read_text(encoding='utf-8')) or {}
    if not isinstance(payload, dict):
        raise ValueError(f'parameter YAML root must be an object: {source}')
    for public_name, yaml_name in TRANSFORM_KEYS.items():
        payload[yaml_name] = custom_transforms[public_name]
    destination.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding='utf-8',
    )


def _readme(manifest: dict[str, Any]) -> str:
    calibration = manifest['calibration']
    frames = manifest['frames']
    topics = manifest['topics']

    def shown(value: str | None) -> str:
        return value if value is not None else 'not used'

    lines = [
        '# Sensor setup bundle',
        '',
        'Status: READY',
        '',
        'Generated from one inspected rosbag2 input. This bundle pins',
        'the selected topics, frames, calibration decision, and parameter '
        'files.',
        '',
        f"- Profile: `{manifest['profile']['id']}`",
        f"- LiDAR topic: `{topics['lidar']}` (`{topics['lidar_type']}`)",
        f"- IMU topic: `{shown(topics['imu'])}`",
        f"- GNSS topic: `{shown(topics['gnss'])}`",
        f"- Navigation topic: `{shown(topics['navigation'])}`",
        (
            '- Navigation quality topic: '
            f"`{shown(topics['navigation_quality'])}`"
        ),
        (
            '- Point timestamp field: '
            f"`{shown(manifest['pointcloud']['timestamp_field'])}`"
        ),
        f"- Base frame: `{shown(frames['base']['id'])}`",
        (
            f"- LiDAR frame: `{shown(frames['lidar']['id'])}` "
            f"({frames['lidar']['source']})"
        ),
        (
            f"- IMU frame: `{shown(frames['imu']['id'])}` "
            f"({frames['imu']['source']})"
        ),
        f"- Calibration: `{calibration['source']}`",
        '',
        'Run the map:',
        '',
        '```bash',
        manifest['run']['command_shell'],
        '```',
        '',
        'Do not edit `sensor_setup.json`. If the physical mounting changes,',
        'generate a new bundle with the measured transforms.',
        '',
    ]
    if not calibration['required']:
        lines[-3:-1] = [
            'Do not edit `sensor_setup.json`. Generate a new bundle when the',
            'input bag or selected sensor workflow changes.',
        ]
    return '\n'.join(lines)


def _render_text(
    result: dict[str, Any],
    *,
    confirmation_follows: bool = False,
) -> str:
    if result['status'] == 'not_ready':
        topics = result['detected']['topics']

        def shown(values: list[str]) -> str:
            return ', '.join(values) if values else 'not found'

        lines = [
            'Sensor session: NOT READY',
            f"Bag: {result['bag_path']}",
            (
                f"Reason: [{result['reason']['code']}] "
                f"{result['reason']['message']}"
            ),
            '',
            'Detected inputs:',
            f"  PointCloud2: {shown(topics['pointcloud2'])}",
            f"  Imu: {shown(topics['imu'])}",
            f"  NavSatFix: {shown(topics['navsatfix'])}",
            f"  VelodyneScan: {shown(topics['velodyne_scan'])}",
            f"  Applanix GSOF49: {shown(topics['applanix_gsof49'])}",
            f"  Applanix GSOF50: {shown(topics['applanix_gsof50'])}",
            (
                '  PointCloud check: '
                f"{result['detected']['pointcloud_check']}"
            ),
            (
                '  Timestamp check: '
                f"{result['detected']['timestamp_check']}"
            ),
            '',
            'What needs attention:',
        ]
        for finding in result['findings']:
            lines.extend([
                f"  [{finding['code']}] {finding['message']}",
                f"    Next: {finding['next_action']}",
            ])
        lines.extend([
            '',
            'Inspect the complete bag diagnosis:',
            f"  {result['next_command']}",
            '',
            'No files were written and mapping was not started.',
        ])
        return '\n'.join(lines)
    if result['status'] == 'review_required':
        transforms = result['profile_extrinsics']
        lines = [
            'Sensor setup: REVIEW REQUIRED',
            f"Bag: {result['bag_path']}",
            f"Profile: {result['profile']['label']}",
            f"LiDAR topic: {result['detected']['lidar_topic']}",
            f"IMU topic: {result['detected']['imu_topic']}",
            (
                'Point timestamp: '
                f"{result['detected']['point_timestamp_field']}"
            ),
            f"Header timestamps: {result['detected']['timestamp_order']}",
            f"Profile LiDAR -> base: {transforms['lidar_to_base']}",
            f"Profile IMU -> base: {transforms['imu_to_base']}",
            '',
            result['message'],
            '',
        ]
        if confirmation_follows:
            lines.extend([
                'Do this now:',
                '  Review the values above, then answer the confirmation '
                'prompt below.',
                '',
                'Otherwise cancel and rerun with measured --lidar-to-base and '
                '--imu-to-base.',
                'No files were written yet.',
            ])
        else:
            lines.extend([
                'If they match, generate the bundle with:',
                f"  {result['next_command']}",
                '',
                'Otherwise supply --lidar-to-base and --imu-to-base.',
                'No files were written.',
            ])
        return '\n'.join(lines)
    calibration = result['calibration']
    dry_run = result['status'] == 'dry_run'
    topics = result['topics']

    def shown(value: str | None) -> str:
        return value if value is not None else 'not used'

    lines = [
        f"Sensor session: {'DRY RUN' if dry_run else 'READY'}",
        (
            f"Bundle: {result['bundle_path']}"
            if not dry_run else
            f"Bundle would be: {result['bundle_path']}"
        ),
        f"Profile: {result['profile']['label']}",
        (
            f"LiDAR: {topics['lidar']} [{topics['lidar_type']}] "
            f"({shown(result['frames']['lidar']['id'])})"
        ),
        (
            f"IMU: {shown(topics['imu'])} "
            f"({shown(result['frames']['imu']['id'])})"
        ),
        f"GNSS: {shown(topics['gnss'])}",
        f"Navigation: {shown(topics['navigation'])}",
        f"Point timestamp: {shown(result['pointcloud']['timestamp_field'])}",
        f"Calibration: {calibration['source']}",
    ]
    if calibration['required']:
        lines.extend([
            f"LiDAR -> base: {calibration['lidar_to_base_quat_xyzw_xyz']}",
            f"IMU -> base: {calibration['imu_to_base_quat_xyzw_xyz']}",
        ])
    lines.extend([
        '',
        'Map command:',
        f"  {result['run']['command_shell']}",
    ])
    if not dry_run:
        lines.extend([
            '',
            f"Review: {Path(result['bundle_path']) / 'README.md'}",
        ])
    else:
        lines.extend([
            '',
            'No files were written and mapping was not started.',
        ])
    return '\n'.join(lines)


def generate(
    args: argparse.Namespace,
    *,
    publish: bool = True,
) -> dict[str, Any]:
    """Inspect one bag and return a review result or ready setup manifest."""
    _validate_calibration_options(args)
    preflight = _load_script_module(
        'preflight_autoware_map_bag.py',
        'sensor_setup_preflight',
    )
    runner = _load_script_module(
        'run_autoware_map_from_bag.py',
        'sensor_setup_runner',
    )
    bag_path = Path(args.bag).expanduser().resolve()
    preflight.validate_bag_path(bag_path)
    payload = preflight.build_preflight_payload(bag_path)
    recommendations = {
        item['id']: item
        for item in payload['recommendations']
    }
    doctor_command = shlex.join([
        _product_command(),
        'doctor',
        str(bag_path),
    ])
    if not recommendations:
        return _not_ready_payload(
            bag_path=bag_path,
            payload=payload,
            code='no-maintained-profile',
            message=(
                'No maintained workflow can safely consume the detected '
                'sensor inputs.'
            ),
            next_command=doctor_command,
        )
    profile_id = select_profile(payload, args.profile)
    if profile_id not in SETUP_PROFILE_IDS:
        raise ValueError(
            f'profile has no maintained sensor setup contract: {profile_id}'
        )
    if profile_id not in recommendations:
        available = ', '.join(recommendations) if recommendations else 'none'
        retry = [
            _product_command(),
            'start' if args.run_now else 'setup',
            str(bag_path),
        ]
        if args.run_now:
            retry.extend(['--yes', '--dry-run'])
        message = (
            f'The forced profile {profile_id} is incompatible with this bag. '
            f'Compatible profiles: {available}.'
        )
        return _not_ready_payload(
            bag_path=bag_path,
            payload=payload,
            code='profile-incompatible',
            message=message,
            findings=[{
                'code': 'profile-incompatible',
                'message': message,
                'next_action': shlex.join(retry),
            }],
            next_command=doctor_command,
        )
    _validate_profile_options(profile_id, args)
    label = recommendations[profile_id]['label']
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else _default_output_dir(bag_path, start_mode=args.run_now)
    )
    map_output_dir = (
        Path(args.map_output_dir).expanduser().resolve()
        if args.map_output_dir
        else output_dir / 'map'
    )
    parameter_sources = _parameter_sources(profile_id)
    profile_transforms = None
    custom_transforms = None
    accepted_profile_extrinsics = (
        args.accept_profile_extrinsics or (args.run_now and args.yes)
    )
    if parameter_sources is not None:
        for source in parameter_sources:
            if not source.is_file():
                raise FileNotFoundError(
                    f'profile parameter file not found: {source}'
                )
        profile_transforms = _profile_transforms(parameter_sources[1])
        if args.lidar_to_base is not None:
            custom_transforms = {
                'lidar_to_base': args.lidar_to_base,
                'imu_to_base': args.imu_to_base,
            }
        elif not accepted_profile_extrinsics:
            return _review_payload(
                bag_path=bag_path,
                output_dir=output_dir,
                profile_id=profile_id,
                label=label,
                payload=payload,
                transforms=profile_transforms,
                args=args,
            )
    if output_dir.exists():
        raise ValueError(
            f'output directory already exists: {output_dir}. '
            'Choose a new --output-dir so a reviewed setup is never '
            'overwritten.'
        )

    summary = payload['summary']
    topics = _profile_topics(summary, profile_id)
    if topics['lidar_type'] == 'sensor_msgs/msg/PointCloud2':
        lidar_frame, lidar_frame_source = _selected_frame(
            payload,
            'sensor_msgs/msg/PointCloud2',
            args.lidar_frame,
            'lidar',
        )
    else:
        lidar_frame, lidar_frame_source = None, 'not_applicable'
    if topics['imu'] is not None:
        imu_frame, imu_frame_source = _selected_frame(
            payload,
            'sensor_msgs/msg/Imu',
            args.imu_frame,
            'imu',
        )
    else:
        imu_frame, imu_frame_source = None, 'not_applicable'
    parameters_dir = output_dir / 'params'
    command = [
        _product_command(),
        'run',
        str(bag_path),
        '--profile',
        profile_id,
        '--output-dir',
        str(map_output_dir),
    ]
    parameter_records = []
    temp_dir = output_dir.with_name(
        f'.{output_dir.name}.partial.{os.getpid()}'
    )
    if temp_dir.exists():
        raise ValueError(f'temporary output path already exists: {temp_dir}')
    temp_params = temp_dir / 'params'
    try:
        temp_params.mkdir(parents=True)
        if parameter_sources is not None:
            destinations = (
                temp_params / 'lidarslam.yaml',
                temp_params / 'rko_lio.yaml',
            )
            _write_parameter_snapshot(
                parameter_sources[0], destinations[0], None
            )
            _write_parameter_snapshot(
                parameter_sources[1],
                destinations[1],
                custom_transforms,
            )
            final_destinations = (
                parameters_dir / destinations[0].name,
                parameters_dir / destinations[1].name,
            )
            command.extend([
                '--lidarslam-param',
                str(final_destinations[0]),
                '--rko-param',
                str(final_destinations[1]),
                '--base-frame',
                args.base_frame,
                '--lidar-frame',
                lidar_frame,
                '--imu-frame',
                imu_frame,
            ])
            for role, source, temporary, final in zip(
                ('graph_backend', 'rko_lio'),
                parameter_sources,
                destinations,
                final_destinations,
            ):
                parameter_records.append({
                    'role': role,
                    'source_path': str(source),
                    'bundle_path': str(final.relative_to(output_dir)),
                    'size_bytes': temporary.stat().st_size,
                    'sha256': _sha256(temporary),
                })

        if args.run_now:
            if args.editable:
                command.append('--editable')
            command.extend([
                '--min-free-space-gib',
                str(args.min_free_space_gib),
                '--verification',
                args.verification,
            ])

        calibration = {
            'required': parameter_sources is not None,
            'source': (
                'explicit_cli_transforms'
                if custom_transforms is not None
                else (
                    'accepted_profile_extrinsics'
                    if parameter_sources
                    else 'not_applicable'
                )
            ),
            'lidar_to_base_quat_xyzw_xyz': (
                custom_transforms['lidar_to_base']
                if custom_transforms is not None
                else (
                    profile_transforms['lidar_to_base']
                    if profile_transforms
                    else None
                )
            ),
            'imu_to_base_quat_xyzw_xyz': (
                custom_transforms['imu_to_base']
                if custom_transforms is not None
                else (
                    profile_transforms['imu_to_base']
                    if profile_transforms
                    else None
                )
            ),
        }
        manifest = {
            'schema_version': SCHEMA_VERSION,
            'schema_uri': SCHEMA_URI,
            'status': 'ready',
            'created_at': _utc_now(),
            'bundle_path': str(output_dir),
            'input': runner._bag_identity(bag_path),
            'profile': {'id': profile_id, 'label': label},
            'topics': topics,
            'frames': {
                'base': {
                    'id': args.base_frame if parameter_sources else None,
                    'source': (
                        'cli_or_default'
                        if parameter_sources else 'not_applicable'
                    ),
                },
                'lidar': {'id': lidar_frame, 'source': lidar_frame_source},
                'imu': {'id': imu_frame, 'source': imu_frame_source},
            },
            'pointcloud': {
                'inspection_status': summary['pointcloud_inspection'][
                    'status'
                ],
                'timestamp_field': summary['pointcloud_inspection'][
                    'timestamp_field'
                ],
                'fields': summary['pointcloud_inspection']['fields'],
            },
            'timestamp_order': summary['timestamp_order'],
            'calibration': calibration,
            'parameters': parameter_records,
            'run': {
                'output_dir': str(map_output_dir),
                'argv': command,
                'command_shell': shlex.join(command),
            },
        }
        (temp_dir / 'sensor_setup.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        (temp_dir / 'README.md').write_text(
            _readme(manifest), encoding='utf-8'
        )
        if publish:
            output_dir.parent.mkdir(parents=True, exist_ok=True)
            os.replace(temp_dir, output_dir)
        else:
            shutil.rmtree(temp_dir)
            manifest['status'] = 'dry_run'
        return manifest
    except Exception:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise


def _ask_to_start(*, calibration_review: bool) -> bool | None:
    if not sys.stdin.isatty():
        print(
            'error: [confirmation-required] start needs confirmation on a '
            'terminal; rerun with --yes after reviewing the displayed setup.',
            file=sys.stderr,
        )
        return None
    prompt = (
        'Do these profile extrinsics match this robot, and start mapping? '
        '[y/N] '
        if calibration_review else
        'Start mapping with this sensor setup? [Y/n] '
    )
    try:
        answer = input(prompt).strip().lower()
    except EOFError:
        print(
            'error: [confirmation-required] no confirmation was received; '
            'rerun with --yes.',
            file=sys.stderr,
        )
        return None
    if calibration_review:
        return answer in {'y', 'yes'}
    return answer in {'', 'y', 'yes'}


def _read_json_object(path: Path) -> dict[str, Any] | None:
    """Read a retained JSON object without hiding the original run failure."""
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _retained_run_dir(map_output: Path) -> tuple[Path | None, bool]:
    """Return evidence and whether atomic output state is ambiguous."""
    candidates = (
        map_output,
        map_output.with_name(f'{map_output.name}.partial'),
    )
    existing = [candidate for candidate in candidates if candidate.is_dir()]
    return (existing[0] if existing else None, len(existing) > 1)


def _fresh_retry_output(map_output: Path) -> Path:
    """Choose a non-existing sibling for a safe pinned-setup retry."""
    index = 1
    while True:
        suffix = '.retry' if index == 1 else f'.retry-{index}'
        candidate = map_output.with_name(f'{map_output.name}{suffix}')
        partial = candidate.with_name(f'{candidate.name}.partial')
        if not candidate.exists() and not partial.exists():
            return candidate
        index += 1


def _replace_output_dir(command: Sequence[str], output_dir: Path) -> list[str]:
    """Return the pinned run command with one fresh output directory."""
    updated = list(command)
    try:
        index = updated.index('--output-dir')
    except ValueError as exc:
        raise ValueError('pinned map command has no --output-dir') from exc
    if index + 1 >= len(updated):
        raise ValueError('pinned map command has an empty --output-dir')
    updated[index + 1] = str(output_dir)
    return updated


def _verification_mode(
    args: argparse.Namespace,
    manifest: dict[str, Any],
) -> str:
    """Return the explicit start verification mode from args or its pin."""
    selected = getattr(args, 'verification', None)
    if selected in {'required', 'off'}:
        return selected
    command = list(manifest['run']['argv'])
    try:
        index = command.index('--verification')
    except ValueError:
        return 'required'
    if index + 1 < len(command) and command[index + 1] in {'required', 'off'}:
        return command[index + 1]
    return 'required'


def _fresh_verified_output(map_output: Path) -> Path:
    """Choose a fresh output for replacing a diagnostic unverified map."""
    index = 1
    while True:
        suffix = '.verified' if index == 1 else f'.verified-{index}'
        candidate = map_output.with_name(f'{map_output.name}{suffix}')
        partial = candidate.with_name(f'{candidate.name}.partial')
        if not candidate.exists() and not partial.exists():
            return candidate
        index += 1


def _verified_retry_command(
    manifest: dict[str, Any],
    map_output: Path,
) -> str:
    """Build one pinned, non-overwriting command with verification enabled."""
    command = _replace_output_dir(
        manifest['run']['argv'],
        _fresh_verified_output(map_output),
    )
    try:
        index = command.index('--verification')
    except ValueError:
        command.extend(['--verification', 'required'])
    else:
        if index + 1 >= len(command):
            raise ValueError('pinned map command has an empty --verification')
        command[index + 1] = 'required'
    return shlex.join(command)


def _existing_artifact(path: Path) -> str | None:
    """Return one file or directory path only when it exists."""
    return str(path) if path.exists() else None


def _session_progress(
    stage: str,
    *,
    stopped: bool = False,
    updated_at: str | None = None,
) -> dict[str, Any]:
    """Map a durable runner lifecycle stage to one honest six-step phase."""
    try:
        phase, current_step, label = SESSION_PROGRESS_STAGES[stage]
    except KeyError as exc:
        raise ValueError(
            f'unknown map session progress stage: {stage}'
        ) from exc
    if stopped:
        phase = 'stopped'
        label = (
            'Session ended with action required'
            if stage == 'complete' else
            f'Action required after: {label}'
        )
    return {
        'phase': phase,
        'stage': stage,
        'current_step': current_step,
        'total_steps': 6,
        'label': label,
        'updated_at': updated_at or _utc_now(),
    }


def _run_lifecycle_stage(run_dir: Path) -> str | None:
    """Read one known lifecycle stage from an atomically written manifest."""
    manifest = _read_json_object(run_dir / 'run_manifest.json')
    lifecycle = manifest.get('lifecycle') if manifest is not None else None
    if not isinstance(lifecycle, dict):
        return None
    stage = lifecycle.get('stage')
    return stage if stage in SESSION_PROGRESS_STAGES else None


def _observed_session_progress(
    map_output: Path,
) -> tuple[str, Path]:
    """Return the current lifecycle stage and its final or partial evidence."""
    partial = map_output.with_name(f'{map_output.name}.partial')
    for run_dir in (partial, map_output):
        stage = _run_lifecycle_stage(run_dir)
        if stage is not None:
            return stage, run_dir
    return 'preparing', map_output


def _quality_check(
    check_id: str,
    label: str,
    status: str,
    observed: str,
    source_checks: Sequence[str] = (),
) -> dict[str, Any]:
    """Build one bounded, human-readable quality evidence card."""
    return {
        'id': check_id,
        'label': label,
        'status': status,
        'observed': observed,
        'source_checks': list(source_checks),
    }


def _validated_validation_receipt(
    run_dir: Path,
) -> tuple[dict[str, Any] | None, dict[str, dict[str, Any]], str, Path]:
    """Read and semantically validate the terminal quality receipt."""
    path = run_dir / VALIDATION_RECEIPT_NAME
    if not path.is_file():
        return None, {}, 'missing', path
    payload = _read_json_object(path)
    if payload is None:
        return None, {}, 'invalid', path
    try:
        product_schema = _load_script_module(
            'product_schema.py',
            'sensor_setup_quality_schema',
        )
        product_schema.validate_contract(
            payload,
            VALIDATION_RECEIPT_SCHEMA,
        )
    except (ImportError, OSError, RuntimeError, ValueError):
        return None, {}, 'invalid', path

    checks: dict[str, dict[str, Any]] = {}
    for item in payload['checks']:
        check_id = item['id']
        if check_id in checks:
            return None, {}, 'invalid', path
        checks[check_id] = item
    if set(checks) != set(VALIDATION_CHECK_IDS):
        return None, {}, 'invalid', path

    evidence = payload['evidence']
    manifest_path = run_dir / evidence['manifest']['filename']
    try:
        manifest_digest = _sha256(manifest_path)
    except OSError:
        return None, {}, 'invalid', path
    if not (
        manifest_digest == evidence['manifest']['sha256']
        == payload['verification']['manifest_sha256']
    ):
        return None, {}, 'invalid', path
    for key in ('diagnosis', 'verify_log'):
        identity = evidence[key]
        evidence_path = run_dir / identity['filename']
        if identity['available']:
            try:
                digest = _sha256(evidence_path)
            except OSError:
                return None, {}, 'invalid', path
            if digest != identity['sha256']:
                return None, {}, 'invalid', path
        elif evidence_path.exists():
            return None, {}, 'invalid', path
    return payload, checks, 'valid', path


def _pending_quality(verification_mode: str) -> dict[str, Any]:
    """Describe checks whose authoritative terminal evidence is not ready."""
    verification_status = (
        'not_run' if verification_mode == 'off' else 'pending'
    )
    verification_observed = (
        'Disabled for this diagnostic run.'
        if verification_mode == 'off' else
        'Waiting for required map verification.'
    )
    return {
        'overall': 'pending',
        'headline': 'Quality evidence will appear as the run completes.',
        'source': {'status': 'pending', 'artifact': None},
        'checks': [
            _quality_check(
                'workflow',
                'Workflow completion',
                'pending',
                'The runner is still active.',
            ),
            _quality_check(
                'map_output',
                'Map output',
                'pending',
                'Map artifacts are still being produced.',
            ),
            _quality_check(
                'verification',
                'Autoware verification',
                verification_status,
                verification_observed,
            ),
            _quality_check(
                'evidence',
                'Evidence integrity',
                'pending',
                'Checksums are written near the end of the lifecycle.',
            ),
        ],
    }


def _recovery_quality(
    verification_mode: str,
    artifacts: dict[str, str | None],
    recovery: dict[str, Any],
) -> dict[str, Any]:
    """Describe a failed run without claiming unperformed quality checks."""
    reason_code = recovery['reason']['code']
    verification_status = 'unavailable'
    verification_observed = 'Required verification did not complete.'
    if verification_mode == 'off':
        verification_status = 'not_run'
        verification_observed = 'Disabled for this diagnostic run.'
    elif reason_code == 'map-verification-failed':
        verification_status = 'fail'
        verification_observed = 'Required Autoware verification failed.'
    evidence_retained = (
        artifacts['recovery_receipt'] is not None
        and (
            artifacts['run_manifest'] is not None
            or artifacts['diagnosis_json'] is not None
        )
    )
    return {
        'overall': 'action_required',
        'headline': 'The map is not ready for trusted use.',
        'source': {
            'status': 'recovery',
            'artifact': artifacts['recovery_receipt'],
        },
        'checks': [
            _quality_check(
                'workflow',
                'Workflow completion',
                'fail',
                f'Stopped with [{reason_code}].',
            ),
            _quality_check(
                'map_output',
                'Map output',
                'fail',
                'The session did not reach a trusted map result.',
            ),
            _quality_check(
                'verification',
                'Autoware verification',
                verification_status,
                verification_observed,
            ),
            _quality_check(
                'evidence',
                'Recovery evidence',
                'pass' if evidence_retained else 'unavailable',
                (
                    'Recovery receipt and retained run evidence are available.'
                    if evidence_retained else
                    'Complete recovery evidence is not available.'
                ),
            ),
        ],
    }


def _terminal_quality(
    status: str,
    verification_mode: str,
    evidence_root: Path,
    artifacts: dict[str, str | None],
) -> dict[str, Any]:
    """Summarize only schema-valid, semantically complete quality evidence."""
    receipt, checks, source_status, receipt_path = (
        _validated_validation_receipt(evidence_root)
    )
    if receipt is None:
        overall = (
            'not_verified' if status == 'unverified' else 'unavailable'
        )
        source_artifact = str(receipt_path) if receipt_path.is_file() else None
        map_available = artifacts['pointcloud_map'] is not None
        return {
            'overall': overall,
            'headline': (
                'Verification was intentionally not run.'
                if status == 'unverified' else
                'The validation receipt is unavailable.'
            ),
            'source': {
                'status': source_status,
                'artifact': source_artifact,
            },
            'checks': [
                _quality_check(
                    'workflow',
                    'Workflow completion',
                    'pass',
                    'The delegated runner returned exit code 0.',
                ),
                _quality_check(
                    'map_output',
                    'Map output',
                    'pass' if map_available else 'unavailable',
                    (
                        'Point-cloud map artifacts are present.'
                        if map_available else
                        'Map artifacts could not be summarized.'
                    ),
                ),
                _quality_check(
                    'verification',
                    'Autoware verification',
                    'not_run' if verification_mode == 'off' else 'unavailable',
                    (
                        'Disabled for this diagnostic run.'
                        if verification_mode == 'off' else
                        'No valid verification receipt is available.'
                    ),
                ),
                _quality_check(
                    'evidence',
                    'Evidence integrity',
                    'unavailable',
                    'Receipt-bound checksums could not be summarized.',
                ),
            ],
        }

    def passed(*check_ids: str) -> bool:
        return all(checks[check_id]['passed'] for check_id in check_ids)

    workflow_ids = (
        'manifest_succeeded',
        'lifecycle_complete',
        'runner_exit_zero',
    )
    workflow_passed = passed(*workflow_ids)
    diagnosis_status = receipt['verification']['diagnosis_status']
    map_passed = checks['diagnosis_success']['passed'] or (
        verification_mode == 'off' and diagnosis_status == 'map_saved'
    )
    verification_passed = checks['autoware_verification_pass']['passed']
    diagnosis_bound = checks['diagnosis_bound_to_manifest']['passed']
    verify_bound = checks['verify_log_bound_to_manifest']['passed']
    evidence_passed = (
        diagnosis_bound
        and (verify_bound or verification_mode == 'off')
    )
    quality_checks = [
        _quality_check(
            'workflow',
            'Workflow completion',
            'pass' if workflow_passed else 'fail',
            (
                'Manifest, lifecycle, and runner exit checks passed.'
                if workflow_passed else
                'One or more runner completion checks failed.'
            ),
            workflow_ids,
        ),
        _quality_check(
            'map_output',
            'Map output',
            'pass' if map_passed else 'fail',
            (
                f'Diagnosis reports {diagnosis_status}.'
                if map_passed else
                'The map diagnosis did not report a usable output.'
            ),
            ('diagnosis_success',),
        ),
        _quality_check(
            'verification',
            'Autoware verification',
            (
                'not_run' if verification_mode == 'off' else
                ('pass' if verification_passed else 'fail')
            ),
            (
                'Disabled for this diagnostic run.'
                if verification_mode == 'off' else
                (
                    'Required Autoware verification passed.'
                    if verification_passed else
                    'Required Autoware verification did not pass.'
                )
            ),
            ('autoware_verification_pass',),
        ),
        _quality_check(
            'evidence',
            'Evidence integrity',
            'pass' if evidence_passed else 'fail',
            (
                'Produced evidence matches manifest checksums.'
                if evidence_passed else
                'One or more produced evidence files are not manifest-bound.'
            ),
            (
                ('diagnosis_bound_to_manifest',)
                if verification_mode == 'off' else
                (
                    'diagnosis_bound_to_manifest',
                    'verify_log_bound_to_manifest',
                )
            ),
        ),
    ]
    if verification_mode == 'off':
        overall = 'not_verified'
        headline = 'Map output exists, but verification was not run.'
    elif receipt['status'] == 'PASS' and all(
        item['status'] == 'pass' for item in quality_checks
    ):
        overall = 'pass'
        headline = 'All required map quality evidence passed.'
    else:
        overall = 'action_required'
        headline = 'One or more required quality checks need attention.'
    return {
        'overall': overall,
        'headline': headline,
        'source': {'status': 'valid', 'artifact': str(receipt_path)},
        'checks': quality_checks,
    }


def _session_index_payload(
    args: argparse.Namespace,
    manifest: dict[str, Any],
    *,
    runner_exit_code: int | None,
    recovery: dict[str, Any] | None = None,
    recovery_path: Path | None = None,
    preview_path: Path | None = None,
    running_stage: str | None = None,
    active_run_dir: Path | None = None,
) -> dict[str, Any]:
    """Build the common successful, unverified, or recovery session index."""
    setup_bundle = Path(manifest['bundle_path'])
    bag_path = Path(manifest['input']['bag_path'])
    map_output = Path(manifest['run']['output_dir'])
    verification_mode = _verification_mode(args, manifest)
    evidence_root = active_run_dir or map_output
    if recovery is not None and recovery['run_dir'] is not None:
        evidence_root = Path(recovery['run_dir'])

    artifacts = {
        'setup_manifest': _existing_artifact(
            setup_bundle / 'sensor_setup.json'
        ),
        'run_manifest': _existing_artifact(
            evidence_root / 'run_manifest.json'
        ),
        'diagnosis_json': _existing_artifact(
            evidence_root / 'autoware_map_diagnosis.json'
        ),
        'validation_receipt': _existing_artifact(
            evidence_root / 'first_map_validation_receipt.json'
        ),
        'pointcloud_map': _existing_artifact(
            evidence_root / 'pointcloud_map'
        ),
        'map_preview_html': (
            _existing_artifact(preview_path)
            if preview_path is not None else None
        ),
        'backend_input': _existing_artifact(
            evidence_root / 'backend_input'
        ),
        'recovery_receipt': (
            _existing_artifact(recovery_path)
            if recovery_path is not None else None
        ),
    }
    actions: list[dict[str, str]] = []

    def add_action(kind: str, label: str, command: str | None) -> None:
        if command is None or any(
            item['command'] == command for item in actions
        ):
            return
        actions.append({
            'kind': kind,
            'label': label,
            'command': command,
        })

    inspect_command = shlex.join([
        _product_command(),
        'inspect',
        str(evidence_root),
        '--bag',
        str(bag_path),
        '--write',
    ])
    support_command = shlex.join([
        _product_command(),
        'support',
        str(setup_bundle),
    ])
    report_command = shlex.join([
        _product_command(),
        'report',
        str(setup_bundle),
    ])
    created_at = _utc_now()
    reason = None
    findings: list[dict[str, str]] = []
    if running_stage is not None:
        if recovery is not None or runner_exit_code is not None:
            raise ValueError(
                'running session cannot have recovery or runner exit state'
            )
        status = 'running'
        result = 'pending'
        if running_stage == 'preparing_preview':
            summary = {
                'title': 'Preparing the map review.',
                'message': (
                    'The map workflow completed. The offline 3D review is '
                    'being prepared, and this page will refresh when ready.'
                ),
            }
        else:
            summary = {
                'title': 'Mapping in progress.',
                'message': (
                    'This page refreshes automatically as the durable map '
                    'lifecycle advances.'
                ),
            }
        progress = _session_progress(
            running_stage,
            updated_at=created_at,
        )
    elif recovery is not None:
        status = 'action_required'
        result = (
            'not_run' if verification_mode == 'off' else 'not_completed'
        )
        reason = dict(recovery['reason'])
        findings = [
            {'code': item['code'], 'message': item['message']}
            for item in recovery['findings']
        ]
        next_kind = 'recover'
        next_label = 'Do this next'
        if recovery['resume']['command'] == recovery['next_command']:
            next_kind = 'resume'
            next_label = 'Finish the retained session safely'
        elif recovery['retry']['command'] == recovery['next_command']:
            next_kind = 'retry'
            next_label = 'Retry the pinned setup safely'
        add_action(next_kind, next_label, recovery['next_command'])
        if recovery['retry']['available']:
            add_action(
                'retry',
                'Retry the same pinned setup in a fresh output',
                recovery['retry']['command'],
            )
        add_action(
            'inspect',
            'Rebuild the full diagnosis',
            recovery['inspect_command'],
        )
        add_action(
            'support',
            'Prepare a privacy-first support report',
            support_command,
        )
        summary = {
            'title': 'Mapping needs attention.',
            'message': recovery['reason']['message'],
        }
        progress_stage = _run_lifecycle_stage(evidence_root) or 'preparing'
        progress = _session_progress(
            progress_stage,
            stopped=True,
            updated_at=created_at,
        )
    else:
        if runner_exit_code != 0:
            raise ValueError('completed session requires runner exit code 0')
        view_command = shlex.join([
            _product_command(),
            'view',
            str(map_output),
        ])
        if verification_mode == 'required':
            status = 'verified'
            result = 'PASS'
            summary = {
                'title': 'Map ready to review.',
                'message': (
                    'Required map checks passed. Review the 3D result and '
                    'keep the validation receipt with the map.'
                ),
            }
        else:
            status = 'unverified'
            result = 'not_run'
            summary = {
                'title': 'Map created without verification.',
                'message': (
                    'This diagnostic run completed, but it is not a verified '
                    'map. Create a fresh verified output before relying on it.'
                ),
            }
            add_action(
                'verify',
                'Create a fresh verified map',
                _verified_retry_command(manifest, map_output),
            )
        view_label = (
            'Review map and prepare edits in 3D'
            if artifacts['backend_input'] is not None else
            'Review the map in 3D'
        )
        add_action('view', view_label, view_command)
        if verification_mode == 'required':
            add_action(
                'share',
                'Prepare a first-map report',
                report_command,
            )
        add_action('inspect', 'Inspect map evidence', inspect_command)
        progress = _session_progress('complete', updated_at=created_at)

    if status == 'running':
        quality = _pending_quality(verification_mode)
    elif recovery is not None:
        quality = _recovery_quality(
            verification_mode,
            artifacts,
            recovery,
        )
    else:
        quality = _terminal_quality(
            status,
            verification_mode,
            evidence_root,
            artifacts,
        )

    return {
        'schema_version': 1,
        'schema_uri': SESSION_SCHEMA_URI,
        'created_at': created_at,
        'status': status,
        'bag_path': str(bag_path),
        'setup_bundle': str(setup_bundle),
        'map_output': str(map_output),
        'profile': {
            'id': manifest['profile']['id'],
            'label': manifest['profile']['label'],
        },
        'runner_exit_code': runner_exit_code,
        'verification': {
            'mode': verification_mode,
            'result': result,
        },
        'progress': progress,
        'quality': quality,
        'summary': summary,
        'reason': reason,
        'findings': findings,
        'actions': actions,
        'artifacts': artifacts,
        'files_preserved': True,
    }


def _resumable_manifest(manifest: dict[str, Any] | None) -> bool:
    """Match the runner's safe terminal post-processing resume boundary."""
    if manifest is None or manifest.get('schema_version') != 2:
        return False
    lifecycle = manifest.get('lifecycle')
    execution = manifest.get('execution')
    if not isinstance(lifecycle, dict) or not isinstance(execution, dict):
        return False
    return (
        lifecycle.get('stage') in {
            'workflow_finished',
            'verifying',
            'verified',
            'finalizing',
            'finalized',
            'diagnosing',
            'diagnosed',
            'checksumming',
        }
        and execution.get('finished_at') is not None
        and execution.get('exit_code') is not None
    )


def _session_recovery_payload(
    manifest: dict[str, Any],
    runner_exit_code: int,
) -> dict[str, Any]:
    """Build stable, copy-ready recovery guidance for a failed start."""
    bag_path = Path(manifest['input']['bag_path'])
    setup_bundle = Path(manifest['bundle_path'])
    map_output = Path(manifest['run']['output_dir'])
    run_dir, ambiguous_output = _retained_run_dir(map_output)
    diagnosis = None
    run_manifest = None
    if run_dir is not None:
        diagnosis = _read_json_object(run_dir / 'autoware_map_diagnosis.json')
        run_manifest = _read_json_object(run_dir / 'run_manifest.json')
        if diagnosis is None:
            try:
                diagnose = _load_script_module(
                    'diagnose_autoware_map_run.py',
                    'sensor_setup_run_diagnosis',
                )
                diagnosis = diagnose.summarize_run(run_dir, bag_path)
            except (FileNotFoundError, OSError, RuntimeError, ValueError):
                diagnosis = None

    inspect_target = run_dir if run_dir is not None else map_output
    inspect_command = shlex.join([
        _product_command(),
        'inspect',
        str(inspect_target),
        '--bag',
        str(bag_path),
        '--write',
    ])
    pinned_command = list(manifest['run']['argv'])
    retry_output = _fresh_retry_output(map_output)
    retry_argv = _replace_output_dir(pinned_command, retry_output)
    retry_command = shlex.join(retry_argv)
    resume_available = (
        not ambiguous_output and _resumable_manifest(run_manifest)
    )
    resume_command = (
        shlex.join([*pinned_command, '--resume'])
        if resume_available else None
    )

    evidence_names = {
        'run_manifest': 'run_manifest.json',
        'diagnosis_json': 'autoware_map_diagnosis.json',
        'diagnosis_markdown': 'autoware_map_diagnosis.md',
        'launch_log': 'lidarslam.launch.log',
        'map_save_log': 'map_save.log',
        'verify_log': 'verify_autoware_map.log',
        'first_map_receipt': 'first_map_validation_receipt.json',
    }
    evidence: dict[str, str | None] = {}
    for key, name in evidence_names.items():
        path = run_dir / name if run_dir is not None else None
        if key == 'launch_log' and path is not None and not path.is_file():
            alternate = run_dir / 'slam.launch.log'
            path = alternate if alternate.is_file() else path
        evidence[key] = (
            str(path) if path is not None and path.is_file() else None
        )

    hints = []
    diagnosis_status = None
    verify_result = None
    if diagnosis is not None:
        hints = [
            str(item) for item in diagnosis.get('problem_hints', [])
            if isinstance(item, str)
        ]
        diagnosis_status = diagnosis.get('status')
        verify = diagnosis.get('verify')
        if isinstance(verify, dict):
            verify_result = verify.get('result')
    signals = '\n'.join(hints).lower()
    manifest_status = run_manifest.get('status') if run_manifest else None
    manifest_path = (
        run_dir / 'run_manifest.json' if run_dir is not None else None
    )

    findings: list[dict[str, str]] = []

    def add_finding(code: str, message: str, next_action: str) -> None:
        if any(item['code'] == code for item in findings):
            return
        findings.append({
            'code': code,
            'message': message,
            'next_action': next_action,
        })

    log_action = inspect_command
    if evidence['launch_log'] is not None:
        log_action = shlex.join(['tail', '-n', '120', evidence['launch_log']])
    map_save_action = inspect_command
    if evidence['map_save_log'] is not None:
        map_save_action = shlex.join(
            ['tail', '-n', '120', evidence['map_save_log']]
        )
    verify_action = inspect_command
    if evidence['verify_log'] is not None:
        verify_action = shlex.join(['less', evidence['verify_log']])

    if ambiguous_output:
        add_finding(
            'ambiguous-output-state',
            'Both final and partial map output directories exist, so '
            'automatic resume is unsafe.',
            inspect_command,
        )
    if (
        manifest_path is not None
        and manifest_path.is_file()
        and run_manifest is None
    ):
        add_finding(
            'run-manifest-unreadable',
            'The retained run manifest is not readable as a JSON object.',
            inspect_command,
        )
    if (
        run_manifest is not None
        and isinstance(run_manifest.get('lifecycle'), dict)
        and run_manifest['lifecycle'].get('stage') in {
            'initialized',
            'workflow_running',
        }
    ):
        add_finding(
            'workflow-state-uncertain',
            'The manifest does not prove that the original workflow reached a '
            'terminal state.',
            inspect_command,
        )
    if resume_available:
        add_finding(
            'postprocessing-incomplete',
            'Mapping stopped after the workflow ended but before all terminal '
            'verification and evidence steps completed.',
            resume_command or inspect_command,
        )
    if any(token in signals for token in (
        'ran out of writable space',
        'no space left on device',
        'disk quota exceeded',
        'enospc',
        'raw_fallocate',
    )):
        storage_target = (
            run_dir.parent if run_dir is not None else map_output.parent
        )
        add_finding(
            'storage-exhausted',
            'The output filesystem ran out of writable space or quota.',
            shlex.join(['df', '-h', str(storage_target)]),
        )
    if manifest_status == 'interrupted' or 'interrupted by sig' in signals:
        add_finding(
            'workflow-interrupted',
            'The map workflow was interrupted before it could complete.',
            inspect_command,
        )
    if 'parameter-file parsing error' in signals:
        add_finding(
            'ros-parameters-invalid',
            'ROS rejected a parameter file used by the pinned setup.',
            log_action,
        )
    if 'tf messages were malformed or incomplete' in signals:
        add_finding(
            'tf-messages-invalid',
            'Recorded TF messages are missing required frame identifiers.',
            log_action,
        )
    if 'tf tree connectivity was missing' in signals:
        add_finding(
            'tf-tree-disconnected',
            'The requested sensor and base frames are not connected in TF.',
            log_action,
        )
    if 'map_save service call failed' in signals:
        add_finding(
            'map-save-failed',
            'The map-save service failed or timed out.',
            map_save_action,
        )
    if 'ros node died' in signals:
        add_finding(
            'ros-node-died',
            'A ROS node exited during mapping.',
            log_action,
        )
    if 'zero gnss edges' in signals:
        add_finding(
            'gnss-constraints-missing',
            'GNSS was enabled but the backend accepted no GNSS constraints.',
            inspect_command,
        )
    if diagnosis_status == 'verify_failed' or verify_result == 'FAIL':
        add_finding(
            'map-verification-failed',
            'Map artifacts were produced but failed required quality checks.',
            verify_action,
        )
    if run_dir is None:
        dry_run_command = shlex.join([*retry_argv, '--dry-run'])
        add_finding(
            'runner-start-failed',
            'The map runner stopped before it retained an output directory.',
            dry_run_command,
        )
    elif not findings and diagnosis_status == 'incomplete':
        add_finding(
            'map-output-incomplete',
            'The retained output is missing required map artifacts.',
            inspect_command,
        )
    elif not findings:
        add_finding(
            'workflow-failed',
            'The map workflow returned a non-zero result without a more '
            'specific recognized signature.',
            inspect_command,
        )

    reason = findings[0]
    return {
        'schema_version': 1,
        'schema_uri': RECOVERY_SCHEMA_URI,
        'created_at': _utc_now(),
        'status': 'action_required',
        'bag_path': str(bag_path),
        'setup_bundle': str(setup_bundle),
        'profile': {
            'id': manifest['profile']['id'],
            'label': manifest['profile']['label'],
        },
        'run_dir': str(run_dir) if run_dir is not None else None,
        'runner_exit_code': runner_exit_code,
        'reason': {
            'code': reason['code'],
            'message': reason['message'],
        },
        'findings': findings,
        'evidence': evidence,
        'resume': {
            'available': resume_available,
            'command': resume_command,
        },
        'retry': {
            'available': not resume_available,
            'command': None if resume_available else retry_command,
            'output_dir': None if resume_available else str(retry_output),
            'preserves_pinned_setup': True,
        },
        'next_command': reason['next_action'],
        'inspect_command': inspect_command if run_dir is not None else None,
        'files_preserved': True,
    }


def _atomic_write_text(destination: Path, content: str) -> None:
    """Replace one product-owned text artifact without a partial file."""
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=destination.parent,
            prefix=f'.{destination.name}.writing-',
            suffix='.tmp',
            delete=False,
        ) as stream:
            stream.write(content)
            temporary = Path(stream.name)
        os.replace(temporary, destination)
    except OSError:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
        raise


def _write_session_recovery(
    setup_bundle: Path,
    payload: dict[str, Any],
) -> Path:
    """Persist the detailed machine-readable recovery source."""
    destination = setup_bundle / RECOVERY_NAME
    _atomic_write_text(
        destination,
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
    )
    return destination


def _session_action_card(
    action: dict[str, str],
    *,
    primary: bool,
) -> str:
    """Render one escaped session action with a local copy button."""
    command = action['command']
    escaped_command = html.escape(command, quote=True)
    badge = '<span class="recommended">Recommended</span>' if primary else ''
    return (
        '<article class="command-card">'
        f'{badge}<h3>{html.escape(action["label"])}</h3>'
        f'<code>{html.escape(command)}</code>'
        f'<button type="button" data-copy="{escaped_command}">'
        'Copy command</button>'
        '</article>'
    )


def _render_session_html(payload: dict[str, Any]) -> str:
    """Return one self-contained, injection-safe session landing page."""
    status = payload['status']
    status_labels = {
        'running': 'RUNNING',
        'verified': 'VERIFIED',
        'unverified': 'UNVERIFIED',
        'action_required': 'ACTION REQUIRED',
    }
    eyebrow = status_labels[status]
    if payload['reason'] is not None:
        eyebrow = f'{eyebrow} · {payload["reason"]["code"]}'
    action_cards = ''.join(
        _session_action_card(action, primary=index == 0)
        for index, action in enumerate(payload['actions'])
    )
    actions_section = ''
    if action_cards:
        actions_section = (
            '<section><h2>Next actions</h2>'
            f'<div class="commands">{action_cards}</div></section>'
        )
    artifact_rows = ''.join(
        '<li>'
        f'<span>{html.escape(key.replace("_", " ").title())}</span>'
        f'<code>{html.escape(value)}</code>'
        '</li>'
        for key, value in payload['artifacts'].items()
        if value is not None
    )
    if not artifact_rows:
        artifact_rows = (
            '<li><span>Evidence</span>'
            '<code>No retained artifact is available yet.</code></li>'
        )
    findings_section = ''
    if payload['findings']:
        finding_cards = ''.join(
            '<article class="finding">'
            f'<span class="code">{html.escape(item["code"])}</span>'
            f'<h3>{html.escape(item["message"])}</h3>'
            '</article>'
            for item in payload['findings']
        )
        findings_section = (
            '<section><h2>What needs attention</h2>'
            f'<div class="findings">{finding_cards}</div></section>'
        )
    preview_section = ''
    preview_path = payload['artifacts']['map_preview_html']
    if preview_path is not None:
        preview_uri = html.escape(
            Path(preview_path).expanduser().resolve().as_uri(),
            quote=True,
        )
        edit_note = (
            ' Editable replay input is retained, so the 3D review can '
            'export a source-pinned edit plan.'
            if payload['artifacts']['backend_input'] is not None else ''
        )
        preview_section = (
            '<section><h2>3D map review</h2>'
            '<article class="preview-card">'
            '<div><h3>Offline browser preview</h3>'
            '<p>Open the map, trajectory, and loop review controls.'
            f'{html.escape(edit_note)}</p></div>'
            f'<a href="{preview_uri}">Open 3D review</a>'
            '</article></section>'
        )
    progress = payload['progress']
    progress_ratio = round(
        progress['current_step'] / progress['total_steps'] * 100
    )
    refresh_note = (
        '<p class="refresh-note">This page refreshes every 2 seconds.</p>'
        if status == 'running' else ''
    )
    progress_section = (
        '<section class="progress-section"><div class="progress-heading">'
        '<div><span>Map lifecycle</span>'
        f'<h2>{html.escape(progress["label"])}</h2></div>'
        f'<strong>Step {progress["current_step"]} of '
        f'{progress["total_steps"]}</strong></div>'
        '<div class="progress-track" role="progressbar" '
        f'aria-valuenow="{progress["current_step"]}" aria-valuemin="1" '
        f'aria-valuemax="{progress["total_steps"]}">'
        f'<span style="width:{progress_ratio}%"></span></div>'
        f'<code class="stage">{html.escape(progress["stage"])}</code>'
        f'{refresh_note}</section>'
    )
    refresh_meta = (
        '<meta http-equiv="refresh" content="2">'
        if status == 'running' else ''
    )
    quality = payload['quality']
    quality_labels = {
        'pending': 'PENDING',
        'pass': 'PASS',
        'not_verified': 'NOT VERIFIED',
        'action_required': 'ACTION REQUIRED',
        'unavailable': 'UNAVAILABLE',
    }
    check_labels = {
        'pending': 'PENDING',
        'pass': 'PASS',
        'not_run': 'NOT RUN',
        'fail': 'FAIL',
        'unavailable': 'UNAVAILABLE',
    }
    quality_cards = ''.join(
        '<article class="quality-card">'
        f'<span class="quality-status {html.escape(item["status"])}">'
        f'{check_labels[item["status"]]}</span>'
        f'<h3>{html.escape(item["label"])}</h3>'
        f'<p>{html.escape(item["observed"])}</p>'
        + (
            '<code class="quality-source-checks">'
            f'{html.escape(", ".join(item["source_checks"]))}</code>'
            if item['source_checks'] else ''
        )
        + '</article>'
        for item in quality['checks']
    )
    quality_source = quality['source']
    source_text = html.escape(quality_source['status'])
    if quality_source['artifact'] is not None:
        source_text = (
            f'{source_text} · {html.escape(quality_source["artifact"])}'
        )
    quality_section = (
        '<section class="quality-section">'
        '<div class="quality-heading"><div>'
        '<span>Evidence-backed summary</span>'
        '<h2>Map quality</h2></div>'
        f'<strong class="quality-overall {quality["overall"]}">'
        f'{quality_labels[quality["overall"]]}</strong></div>'
        f'<p class="quality-headline">{html.escape(quality["headline"])}</p>'
        f'<div class="quality-grid">{quality_cards}</div>'
        f'<p class="quality-source">Source: {source_text}</p></section>'
    )

    template = Template("""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  $refresh_meta
  <meta http-equiv="Content-Security-Policy"
        content="default-src 'none'; style-src 'unsafe-inline';
                 script-src 'unsafe-inline'; object-src 'none';
                 base-uri 'none'; form-action 'none'">
  <title>lidarslam_ros2 session · $status_label</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #090d12;
      --panel: #111923;
      --line: #263548;
      --text: #edf4ff;
      --muted: #9fb0c5;
      --accent: #54d6ff;
      --accent-ink: #071820;
      --good: #7fe0ae;
    }
    body.verified { --accent: #7fe0ae; --accent-ink: #082016; }
    body.unverified { --accent: #ffd166; --accent-ink: #211805; }
    body.action_required { --accent: #ff9f5a; --accent-ink: #251204; }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: radial-gradient(
        circle at 85% 0%, #1c2b3d 0, var(--bg) 38rem
      );
      color: var(--text);
      font: 15px/1.55 system-ui, sans-serif;
    }
    main {
      width: min(1080px, calc(100% - 32px));
      margin: 0 auto;
      padding: 48px 0 72px;
    }
    header {
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: clamp(24px, 5vw, 48px);
      background: linear-gradient(
        135deg, color-mix(in srgb, var(--accent) 14%, transparent),
        rgba(84,214,255,.04)
      );
      box-shadow: 0 20px 70px rgba(0,0,0,.35);
    }
    .eyebrow, .code, .recommended {
      display: inline-block;
      border-radius: 999px;
      padding: 5px 10px;
      font: 700 12px/1.2 ui-monospace, monospace;
      letter-spacing: .04em;
    }
    .eyebrow { color: var(--accent-ink); background: var(--accent); }
    .code { color: var(--text); background: #25384c; }
    .recommended {
      color: var(--good);
      background: rgba(127,224,174,.1);
      border: 1px solid rgba(127,224,174,.3);
    }
    h1 {
      margin: 16px 0 8px;
      font-size: clamp(34px, 7vw, 64px);
      line-height: 1.02;
      letter-spacing: -.045em;
    }
    h2 { margin: 44px 0 14px; font-size: 23px; }
    h3 { margin: 10px 0; font-size: 16px; }
    p { margin: 8px 0; }
    .lead { max-width: 780px; color: var(--muted); font-size: 18px; }
    .meta {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
      gap: 10px;
      margin-top: 24px;
    }
    .meta div, .command-card, .finding, .evidence, .preview-card {
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(17,25,35,.92);
    }
    .meta div { padding: 13px 15px; }
    .meta span, .evidence span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: .08em;
    }
    .meta strong { display: block; margin-top: 4px; overflow-wrap: anywhere; }
    .meta .id {
      display: block;
      margin-top: 5px;
      color: var(--accent);
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .commands, .findings {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(290px, 1fr));
      gap: 14px;
    }
    .progress-section {
      margin-top: 28px;
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 18px 20px;
      background: rgba(17,25,35,.72);
    }
    .progress-heading {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 18px;
    }
    .progress-heading span {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: .08em;
    }
    .progress-heading h2 { margin: 4px 0 10px; }
    .progress-heading strong { margin-bottom: 12px; color: var(--accent); }
    .progress-track {
      height: 10px;
      overflow: hidden;
      border-radius: 999px;
      background: #080d13;
      border: 1px solid var(--line);
    }
    .progress-track span {
      display: block;
      height: 100%;
      border-radius: inherit;
      background: linear-gradient(90deg, var(--accent), #54d6ff);
      transition: width .25s ease;
    }
    .stage {
      display: inline-block;
      margin-top: 10px;
      color: var(--muted);
    }
    .refresh-note { color: var(--muted); font-size: 13px; }
    .quality-section { margin-top: 36px; }
    .quality-heading {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 18px;
    }
    .quality-heading span {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: .08em;
    }
    .quality-heading h2 { margin: 4px 0 0; }
    .quality-overall, .quality-status {
      display: inline-block;
      border-radius: 999px;
      font: 700 11px/1.2 ui-monospace, monospace;
      letter-spacing: .04em;
    }
    .quality-overall { padding: 7px 11px; }
    .quality-status { padding: 4px 8px; }
    .quality-overall.pass, .quality-status.pass {
      color: var(--good);
      background: rgba(127,224,174,.1);
      border: 1px solid rgba(127,224,174,.3);
    }
    .quality-overall.pending, .quality-status.pending {
      color: #54d6ff;
      background: rgba(84,214,255,.1);
      border: 1px solid rgba(84,214,255,.3);
    }
    .quality-overall.not_verified, .quality-status.not_run {
      color: #ffd166;
      background: rgba(255,209,102,.1);
      border: 1px solid rgba(255,209,102,.3);
    }
    .quality-overall.action_required, .quality-status.fail {
      color: #ff9f5a;
      background: rgba(255,159,90,.1);
      border: 1px solid rgba(255,159,90,.3);
    }
    .quality-overall.unavailable, .quality-status.unavailable {
      color: #b5c1d0;
      background: rgba(181,193,208,.08);
      border: 1px solid rgba(181,193,208,.22);
    }
    .quality-headline { color: var(--muted); font-size: 17px; }
    .quality-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
      margin-top: 16px;
    }
    .quality-card {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 16px;
      background: rgba(17,25,35,.92);
    }
    .quality-card p { color: var(--muted); }
    .quality-source-checks {
      display: block;
      color: #71849a;
      font-size: 11px;
      overflow-wrap: anywhere;
    }
    .quality-source {
      color: #71849a;
      font: 12px/1.5 ui-monospace, monospace;
      overflow-wrap: anywhere;
    }
    .command-card { position: relative; padding: 17px; }
    .command-card code {
      display: block;
      margin: 12px 0 14px;
      color: var(--accent);
      overflow-wrap: anywhere;
      white-space: pre-wrap;
    }
    button, .preview-card a {
      display: inline-block;
      border: 1px solid #3d526a;
      border-radius: 9px;
      padding: 9px 12px;
      background: #1a2a3b;
      color: var(--text);
      cursor: pointer;
      font-weight: 700;
      text-decoration: none;
    }
    button:hover, .preview-card a:hover { border-color: var(--accent); }
    button.copied { border-color: var(--good); color: var(--good); }
    .finding { padding: 18px; }
    .preview-card {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 20px;
      padding: 20px;
    }
    .preview-card p { color: var(--muted); }
    .evidence { padding: 4px 18px; }
    .evidence ul { list-style: none; margin: 0; padding: 0; }
    .evidence li {
      display: grid;
      grid-template-columns: 190px 1fr;
      gap: 16px;
      padding: 14px 0;
      border-bottom: 1px solid var(--line);
    }
    .evidence li:last-child { border-bottom: 0; }
    .evidence code { overflow-wrap: anywhere; color: #c8d7e9; }
    footer { margin-top: 42px; color: var(--muted); }
    @media (max-width: 640px) {
      main {
        width: min(100% - 20px, 1080px);
        padding-top: 20px;
      }
      .preview-card { align-items: stretch; flex-direction: column; }
      .preview-card a { text-align: center; }
      .progress-heading { align-items: start; flex-direction: column; gap: 0; }
      .quality-heading {
        align-items: start;
        flex-direction: column;
        gap: 8px;
      }
      .quality-grid { grid-template-columns: 1fr; }
      .evidence li { grid-template-columns: 1fr; gap: 4px; }
    }
  </style>
</head>
<body class="$status_class">
<main>
  <header>
    <span class="eyebrow">$eyebrow</span>
    <h1>$title</h1>
    <p class="lead">$message</p>
    <div class="meta">
      <div>
        <span>Profile</span>
        <strong>$profile_label</strong>
        <code class="id">$profile_id</code>
      </div>
      <div><span>Verification</span><strong>$verification</strong></div>
      <div><span>Runner exit</span><strong>$runner_exit</strong></div>
      <div><span>Map</span><strong>$map_output</strong></div>
      <div><span>Bag</span><strong>$bag_path</strong></div>
    </div>
  </header>
  $progress_section
  $quality_section
  $preview_section
  $actions_section
  $findings_section
  <section>
    <h2>Evidence kept</h2>
    <div class="evidence"><ul>$artifact_rows</ul></div>
  </section>
  <footer>
    <p>The source bag, pinned setup, and retained run evidence were not
       overwritten.</p>
    <p>Updated $created_at by lidarslam_ros2.</p>
  </footer>
</main>
<script>
  function fallbackCopy(value) {
    const area = document.createElement('textarea');
    area.value = value;
    area.setAttribute('readonly', '');
    area.style.position = 'fixed';
    area.style.opacity = '0';
    document.body.appendChild(area);
    area.select();
    document.execCommand('copy');
    area.remove();
  }
  function markCopied(button) {
    button.textContent = 'Copied';
    button.classList.add('copied');
    window.setTimeout(function () {
      button.textContent = 'Copy command';
      button.classList.remove('copied');
    }, 1600);
  }
  document.querySelectorAll('[data-copy]').forEach(function (button) {
    button.addEventListener('click', function () {
      const value = button.getAttribute('data-copy');
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(value).then(function () {
          markCopied(button);
        }).catch(function () {
          fallbackCopy(value);
          markCopied(button);
        });
      } else {
        fallbackCopy(value);
        markCopied(button);
      }
    });
  });
</script>
</body>
</html>
""")
    verification = payload['verification']
    return template.substitute(
        status_label=html.escape(status_labels[status]),
        status_class=html.escape(status),
        refresh_meta=refresh_meta,
        eyebrow=html.escape(eyebrow),
        title=html.escape(payload['summary']['title']),
        message=html.escape(payload['summary']['message']),
        profile_label=html.escape(payload['profile']['label']),
        profile_id=html.escape(payload['profile']['id']),
        verification=html.escape(
            f'{verification["mode"]} · {verification["result"]}'
        ),
        runner_exit=(
            payload['runner_exit_code']
            if payload['runner_exit_code'] is not None else 'Running'
        ),
        map_output=html.escape(payload['map_output']),
        bag_path=html.escape(payload['bag_path']),
        progress_section=progress_section,
        quality_section=quality_section,
        preview_section=preview_section,
        actions_section=actions_section,
        findings_section=findings_section,
        artifact_rows=artifact_rows,
        created_at=html.escape(payload['created_at']),
    )


def _write_session_index(
    setup_bundle: Path,
    payload: dict[str, Any],
) -> tuple[Path, Path | None]:
    """Persist the session index and its best-effort human browser view."""
    destination = setup_bundle / SESSION_NAME
    _atomic_write_text(
        destination,
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
    )
    report_path = setup_bundle / SESSION_HTML_NAME
    try:
        _atomic_write_text(report_path, _render_session_html(payload))
    except Exception:
        # The derived human view must never suppress the JSON session index.
        return destination, None
    return destination, report_path


def _write_running_session(
    args: argparse.Namespace,
    manifest: dict[str, Any],
    stage: str,
    active_run_dir: Path,
) -> tuple[Path, Path | None, dict[str, Any]]:
    """Atomically update the common session handoff for one live stage."""
    payload = _session_index_payload(
        args,
        manifest,
        runner_exit_code=None,
        running_stage=stage,
        active_run_dir=active_run_dir,
    )
    session_path, report_path = _write_session_index(
        Path(manifest['bundle_path']),
        payload,
    )
    return session_path, report_path, payload


def _monitor_session_progress(
    args: argparse.Namespace,
    manifest: dict[str, Any],
    stop_event: threading.Event,
) -> None:
    """Mirror durable stages and emit bounded no-ETA terminal heartbeats."""
    map_output = Path(manifest['run']['output_dir'])
    last_stage = 'preparing'
    write_warning_reported = False
    started_at = time.monotonic()
    last_terminal_report = started_at
    while not stop_event.wait(SESSION_PROGRESS_POLL_SECONDS):
        observed_at = time.monotonic()
        stage, run_dir = _observed_session_progress(map_output)
        if stage == last_stage:
            if (
                stage != 'complete'
                and observed_at - last_terminal_report
                >= SESSION_PROGRESS_HEARTBEAT_SECONDS
            ):
                progress = _session_progress(stage)
                elapsed = _format_elapsed(observed_at - started_at)
                print(
                    'Map heartbeat '
                    f"[{progress['current_step']}/{progress['total_steps']}]: "
                    f"{progress['label']}; still running, elapsed {elapsed}"
                )
                sys.stdout.flush()
                last_terminal_report = observed_at
            continue
        if stage == 'complete':
            # The terminal writer will bind the observed runner exit result.
            last_stage = stage
            continue
        last_terminal_report = observed_at
        try:
            _, _, payload = _write_running_session(
                args,
                manifest,
                stage,
                run_dir,
            )
        except (OSError, ValueError) as exc:
            if not write_warning_reported:
                print(
                    'warning: [session-progress-write-failed] live progress '
                    f'could not update session.html: {exc}',
                    file=sys.stderr,
                )
                write_warning_reported = True
            last_stage = stage
            continue
        progress = payload['progress']
        print(
            'Map progress '
            f"[{progress['current_step']}/{progress['total_steps']}]: "
            f"{progress['label']}"
        )
        sys.stdout.flush()
        last_stage = stage


def _format_elapsed(seconds: float) -> str:
    """Format monotonic elapsed seconds without implying an ETA."""
    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, remaining_seconds = divmod(remainder, 60)
    if hours:
        return f'{hours:02}:{minutes:02}:{remaining_seconds:02}'
    return f'{minutes:02}:{remaining_seconds:02}'


def _render_session_recovery(
    payload: dict[str, Any],
    receipt_path: Path | None,
    session_path: Path | None,
    report_path: Path | None,
) -> str:
    """Render one bounded first-action failure handoff for a person."""
    reason = payload['reason']
    lines = [
        '',
        'Map session: ACTION REQUIRED',
        f"Reason: [{reason['code']}] {reason['message']}",
    ]
    remaining_codes = [
        f"[{finding['code']}]"
        for finding in payload['findings'][1:]
    ]
    if remaining_codes:
        lines.append(f"Also detected: {', '.join(remaining_codes)}")
    lines.extend(['', 'Next:', f"  {payload['next_command']}"])
    details_path = (
        report_path
        or receipt_path
        or session_path
        or Path(payload['setup_bundle'])
    )
    lines.append(f'Details: {details_path}')
    return '\n'.join(lines)


def _maybe_open_session_report(
    args: argparse.Namespace,
    report_path: Path | None,
) -> None:
    """Best-effort open the common report without changing map status."""
    if report_path is None or args.viewer != 'browser':
        return
    try:
        viewer = _load_script_module(
            'view_autoware_map.py',
            'sensor_setup_session_browser',
        )
        if not viewer.desktop_session_available():
            print(
                'No desktop session detected; open session.html manually.'
            )
            return
        opened = viewer.open_browser(report_path.as_uri())
    except Exception as exc:
        # Browser integration must never replace the delegated map exit code.
        print(
            f'warning: could not open the session browser: {exc}',
            file=sys.stderr,
        )
        return
    if opened:
        print('Session browser opened.')
    else:
        print(
            'warning: no browser accepted session.html; open it manually.',
            file=sys.stderr,
        )


def _quality_check_status(payload: dict[str, Any], check_id: str) -> str:
    """Return one evidence-backed check status for terminal summaries."""
    quality = payload.get('quality')
    checks = quality.get('checks') if isinstance(quality, dict) else None
    if not isinstance(checks, list):
        return 'unavailable'
    for item in checks:
        if isinstance(item, dict) and item.get('id') == check_id:
            status = item.get('status')
            if isinstance(status, str) and status:
                return status
    return 'unavailable'


def _completion_next_action(payload: dict[str, Any]) -> dict[str, str] | None:
    """Choose a safe, copy-ready action for a completed terminal session."""
    quality = payload.get('quality')
    overall = quality.get('overall') if isinstance(quality, dict) else None
    preferred_kinds = {
        'pass': ('view',),
        'not_verified': ('verify',),
        'action_required': ('inspect', 'support', 'view'),
        'unavailable': ('inspect', 'support', 'view'),
    }.get(overall, ('view',))
    actions = payload.get('actions')
    if not isinstance(actions, list):
        return None
    for kind in preferred_kinds:
        for action in actions:
            if (
                isinstance(action, dict)
                and action.get('kind') == kind
                and isinstance(action.get('command'), str)
            ):
                return {'kind': kind, 'command': action['command']}
    return None


def _render_session_completion_summary(
    args: argparse.Namespace,
    payload: dict[str, Any],
    map_output: Path,
    *,
    session_path: Path,
    preview_path: Path | None,
    report_path: Path | None,
    viewer_returncode: int,
) -> str:
    """Render one copy-ready terminal handoff after a successful run."""
    verification_status = _quality_check_status(payload, 'verification')
    verification_label = verification_status.replace('_', ' ').upper()
    heading = {
        'verified': 'Map session: VERIFIED',
        'unverified': 'Map session: UNVERIFIED',
    }.get(payload.get('status'), 'Map session: COMPLETED')
    lines = ['', heading, f'  Map output:        {map_output}']
    lines.append(f'  Verification:      {verification_label}')
    if payload.get('status') == 'unverified':
        lines.append(
            '  Warning:           verification was skipped; do not rely on '
            'this map as verified'
        )
    if preview_path is not None:
        lines.append(f'  Viewer:            {preview_path}')
    elif args.viewer == 'none':
        lines.append('  Viewer:            not opened (--viewer none)')
    elif args.viewer == 'browser':
        lines.append(
            '  Viewer:            3D review unavailable; use Next below'
        )
    elif viewer_returncode == 0:
        lines.append(
            f'  Viewer:            {args.viewer} command completed'
        )
    else:
        lines.append(
            f'  Viewer:            {args.viewer} failed; use Next below'
        )

    lines.append(f'  Session index:     {session_path}')
    lines.append(
        '  Session page:      '
        f'{report_path if report_path is not None else "unavailable"}'
    )

    artifacts = payload.get('artifacts')
    if not isinstance(artifacts, dict):
        artifacts = {}
    run_manifest = artifacts.get('run_manifest')
    validation_receipt = artifacts.get('validation_receipt')
    run_manifest_text = (
        run_manifest if run_manifest is not None else 'unavailable'
    )
    validation_receipt_text = (
        validation_receipt if validation_receipt is not None else 'unavailable'
    )
    lines.append(
        f'  Run manifest:      {run_manifest_text}'
    )
    lines.append(
        f'  First-map receipt:  {validation_receipt_text}'
    )

    next_action = None
    if viewer_returncode != 0:
        actions = payload.get('actions')
        if isinstance(actions, list):
            for action in actions:
                if (
                    isinstance(action, dict)
                    and action.get('kind') == 'view'
                    and isinstance(action.get('command'), str)
                ):
                    next_action = {
                        'kind': 'view',
                        'command': action['command'],
                    }
                    break
    if next_action is None:
        next_action = _completion_next_action(payload)
    if next_action is None:
        next_action = {
            'kind': 'view',
            'command': shlex.join([
                _product_command(),
                'view',
                str(map_output),
            ]),
        }
    lines.append(f'  Next:              {next_action["command"]}')

    quality = payload.get('quality')
    if isinstance(quality, dict) and quality.get('overall') == 'pass':
        actions = payload.get('actions')
        if isinstance(actions, list):
            for action in actions:
                if (
                    isinstance(action, dict)
                    and action.get('kind') == 'share'
                    and isinstance(action.get('command'), str)
                ):
                    lines.append(f'  Report:            {action["command"]}')
                    break
    return '\n'.join(lines)


def _render_session_completion_fallback(
    verification_mode: str,
    map_output: Path,
) -> str:
    """Keep one usable terminal card when the derived session index fails."""
    lines = [
        '',
        'Map session: COMPLETED',
        f'  Map output:        {map_output}',
        '  Session summary:   unavailable; inspect retained map evidence',
    ]
    if verification_mode == 'off':
        lines.append(
            '  Warning:           verification was skipped; do not rely on '
            'this map as verified'
        )
    lines.append(
        '  Next:              '
        f'{_product_command()} view {shlex.quote(str(map_output))}'
    )
    return '\n'.join(lines)


def _run_delegated_session(
    command: list[str],
) -> subprocess.CompletedProcess:
    """Reap the delegated runner after Ctrl-C so it can seal evidence."""
    child_env = os.environ.copy()
    child_env[PRODUCT_SESSION_OUTPUT_ENV] = PRODUCT_SESSION_OUTPUT_CONCISE
    process: subprocess.Popen | None = None

    def request_termination(signum, _frame):
        raise _SessionTerminationRequested(signum)

    previous_sigterm = signal.signal(signal.SIGTERM, request_termination)
    try:
        process = subprocess.Popen(
            command,
            cwd=WORK_ROOT,
            env=child_env,
            start_new_session=True,
        )
        try:
            return_code = process.wait()
            return subprocess.CompletedProcess(command, return_code)
        except KeyboardInterrupt:
            requested_signal = signal.SIGINT
        except _SessionTerminationRequested as exc:
            requested_signal = exc.signum

        if process.poll() is None:
            try:
                os.killpg(process.pid, requested_signal)
            except ProcessLookupError:
                pass
        request_label = (
            'Stop requested.'
            if requested_signal == signal.SIGINT
            else 'Stop requested by '
            f'{signal.Signals(requested_signal).name}.'
        )
        print(
            f'\n{request_label} Waiting up to '
            f'{SESSION_INTERRUPT_GRACE_SECONDS:g} seconds for map cleanup '
            'and terminal evidence...',
            file=sys.stderr,
        )
        try:
            return_code = process.wait(
                timeout=SESSION_INTERRUPT_GRACE_SECONDS,
            )
        except (
            KeyboardInterrupt,
            _SessionTerminationRequested,
            subprocess.TimeoutExpired,
        ):
            print(
                'warning: [runner-shutdown-grace-expired] requesting '
                'delegated runner termination while preserving retained '
                'evidence.',
                file=sys.stderr,
            )
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            try:
                return_code = process.wait(
                    timeout=SESSION_INTERRUPT_TERMINATE_SECONDS,
                )
            except (
                KeyboardInterrupt,
                _SessionTerminationRequested,
                subprocess.TimeoutExpired,
            ):
                print(
                    'warning: [runner-shutdown-forced] delegated runner did '
                    'not stop after termination; forcing final reap.',
                    file=sys.stderr,
                )
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                return_code = process.wait()
        return subprocess.CompletedProcess(command, return_code)
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


def _run_session(args: argparse.Namespace, manifest: dict[str, Any]) -> int:
    map_output = Path(manifest['run']['output_dir'])
    command = list(manifest['run']['argv'])
    setup_bundle = Path(manifest['bundle_path'])
    verification_mode = _verification_mode(args, manifest)
    print('')
    if verification_mode == 'required':
        print('Starting the verified map session:')
    else:
        print('Starting a diagnostic map session without verification:')
    print(f"  Setup: {manifest['bundle_path']}")
    print(f'  Map:   {map_output}')
    initial_report_path = None
    try:
        session_path, initial_report_path, initial = _write_running_session(
            args,
            manifest,
            'preparing',
            map_output,
        )
        progress = initial['progress']
        print(
            'Map progress '
            f"[{progress['current_step']}/{progress['total_steps']}]: "
            f"{progress['label']}"
        )
        print(f'  Live session: {session_path}')
        if initial_report_path is not None:
            print(f'  Live page:    {initial_report_path}')
        else:
            print(
                'warning: [session-html-write-failed] session.json was kept, '
                'but the live session page could not be written.',
                file=sys.stderr,
            )
    except (OSError, ValueError) as exc:
        print(
            'warning: [session-index-write-failed] mapping will continue, but '
            f'the live session handoff could not be written: {exc}',
            file=sys.stderr,
        )
    _maybe_open_session_report(args, initial_report_path)
    sys.stdout.flush()
    progress_stop = threading.Event()
    progress_monitor = threading.Thread(
        target=_monitor_session_progress,
        args=(args, manifest, progress_stop),
        name='lidarslam-session-progress',
        daemon=True,
    )
    progress_monitor.start()
    try:
        completed = _run_delegated_session(command)
    finally:
        progress_stop.set()
        progress_monitor.join()
    if completed.returncode != 0:
        recovery = _session_recovery_payload(manifest, completed.returncode)
        code = recovery['reason']['code']
        if code != 'workflow-interrupted':
            print(
                f'error: [{code}] map session needs attention; runner exit '
                f'code {completed.returncode}.',
                file=sys.stderr,
            )
        receipt_path = None
        session_path = None
        report_path = None
        try:
            receipt_path = _write_session_recovery(
                setup_bundle,
                recovery,
            )
        except OSError as exc:
            print(
                'warning: [recovery-receipt-write-failed] could not write the '
                f'recovery receipt: {exc}',
                file=sys.stderr,
            )
        try:
            session = _session_index_payload(
                args,
                manifest,
                runner_exit_code=completed.returncode,
                recovery=recovery,
                recovery_path=receipt_path,
            )
            session_path, report_path = _write_session_index(
                setup_bundle,
                session,
            )
            if report_path is None:
                print(
                    'warning: [session-html-write-failed] session.json was '
                    'kept, but session.html could not be written.',
                    file=sys.stderr,
                )
        except (OSError, ValueError) as exc:
            print(
                'warning: [session-index-write-failed] could not write the '
                f'common session handoff: {exc}',
                file=sys.stderr,
            )
        print(_render_session_recovery(
            recovery,
            receipt_path,
            session_path,
            report_path,
        ))
        if initial_report_path is None:
            _maybe_open_session_report(args, report_path)
        return completed.returncode

    viewer_returncode = 0
    preview_path = None
    if args.viewer == 'browser':
        try:
            _, preview_report_path, preview_session = _write_running_session(
                args,
                manifest,
                'preparing_preview',
                map_output,
            )
            progress = preview_session['progress']
            print(
                'Map progress '
                f"[{progress['current_step']}/{progress['total_steps']}]: "
                f"{progress['label']}"
            )
            if initial_report_path is None:
                initial_report_path = preview_report_path
                _maybe_open_session_report(args, initial_report_path)
        except (OSError, ValueError) as exc:
            print(
                'warning: [session-progress-write-failed] the map is '
                f'complete, but preview progress could not be written: {exc}',
                file=sys.stderr,
            )
        view_command = [
            _product_command(),
            'view',
            str(map_output),
            '--viewer',
            'browser',
            '--no-open',
        ]
        sys.stdout.flush()
        viewer = subprocess.run(view_command, check=False, cwd=WORK_ROOT)
        viewer_returncode = viewer.returncode
        candidate = map_output / MAP_PREVIEW_RELATIVE_PATH
        preview_path = candidate if candidate.is_file() else None

    session = None
    session_path = None
    report_path = None
    try:
        session = _session_index_payload(
            args,
            manifest,
            runner_exit_code=0,
            preview_path=preview_path,
        )
        session_path, report_path = _write_session_index(
            setup_bundle,
            session,
        )
        if report_path is None:
            print(
                'warning: [session-html-write-failed] session.json was kept, '
                'but session.html could not be written.',
                file=sys.stderr,
            )
    except (OSError, ValueError) as exc:
        print(
            'warning: [session-index-write-failed] the map is complete, but '
            f'the common session handoff could not be written: {exc}',
            file=sys.stderr,
        )

    if args.viewer == 'browser':
        if initial_report_path is None:
            _maybe_open_session_report(args, report_path)
    elif args.viewer != 'none':
        view_command = [
            _product_command(),
            'view',
            str(map_output),
            '--viewer',
            args.viewer,
        ]
        sys.stdout.flush()
        viewer = subprocess.run(view_command, check=False, cwd=WORK_ROOT)
        viewer_returncode = viewer.returncode

    if session is not None and session_path is not None:
        print(
            _render_session_completion_summary(
                args,
                session,
                map_output,
                session_path=session_path,
                preview_path=preview_path,
                report_path=report_path,
                viewer_returncode=viewer_returncode,
            )
        )
    else:
        print(_render_session_completion_fallback(
            verification_mode,
            map_output,
        ))

    if viewer_returncode != 0:
        print(
            'warning: [viewer-failed] the map workflow is complete, but the '
            f'viewer failed with exit code {viewer_returncode}.',
            file=sys.stderr,
        )
        return viewer_returncode
    return 0


def _runtime_readiness(profile_id: str) -> tuple[list[str], list[str]]:
    guided = _load_script_module(
        'lidarslam_guided.py',
        'sensor_setup_guided_runtime',
    )
    return (
        guided.runtime_readiness(profile_id),
        guided._render_runtime_next_steps(),
    )


def _print_runtime_failure(issues: list[str], next_steps: list[str]) -> None:
    print(
        'error: [runtime-incomplete] the sensor input is compatible, but the '
        'local runtime is incomplete.',
        file=sys.stderr,
    )
    for issue in issues:
        print(f'  - {issue}', file=sys.stderr)
    print('Next steps:', file=sys.stderr)
    for step in next_steps:
        print(f'  {step}', file=sys.stderr)


def _check_runtime(profile_id: str) -> int:
    try:
        issues, next_steps = _runtime_readiness(profile_id)
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f'error: runtime preflight failed: {exc}', file=sys.stderr)
        return 70
    if issues:
        _print_runtime_failure(issues, next_steps)
        return 2
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run the sensor setup wizard or one-command verified session."""
    args = parse_args(argv)
    if (
        not math.isfinite(args.min_free_space_gib)
        or args.min_free_space_gib <= 0
    ):
        print(
            'error: [invalid-free-space] --min-free-space-gib must be finite '
            'and greater than zero',
            file=sys.stderr,
        )
        return 2
    if args.run_now and args.json and not args.dry_run:
        print(
            'error: [live-json-unsupported] --json requires --dry-run for '
            'start; live mapping emits progress instead of one JSON document.',
            file=sys.stderr,
        )
        return 2
    bag_path = Path(args.bag).expanduser().resolve()
    if not args.output_dir:
        args.output_dir = str(
            _default_output_dir(bag_path, start_mode=args.run_now)
        )
    confirmed = args.yes
    runtime_checked = False
    try:
        result = generate(args, publish=not args.dry_run)
    except (
        FileNotFoundError,
        OSError,
        RuntimeError,
        ValueError,
        yaml.YAMLError,
    ) as exc:
        print(f'error: [setup-rejected] {exc}', file=sys.stderr)
        print(
            f'Next: {_product_command()} doctor {shlex.quote(str(bag_path))}',
            file=sys.stderr,
        )
        return 2
    if result['status'] == 'not_ready':
        if args.json:
            print(json.dumps(result, indent=2, sort_keys=True))
        else:
            print(_render_text(result))
        return 2
    if result['status'] == 'review_required':
        if args.json:
            print(json.dumps(result, indent=2, sort_keys=True))
        else:
            print(_render_text(
                result,
                confirmation_follows=(
                    args.run_now
                    and not args.dry_run
                    and sys.stdin.isatty()
                ),
            ))
        if not args.run_now or args.dry_run:
            return 0
        decision = _ask_to_start(calibration_review=True)
        if decision is None:
            return 2
        if not decision:
            print(
                'Cancelled. No setup files were written and mapping was not '
                'started.'
            )
            return 0
        runtime_result = _check_runtime(result['profile']['id'])
        if runtime_result != 0:
            return runtime_result
        runtime_checked = True
        args.accept_profile_extrinsics = True
        confirmed = True
        try:
            result = generate(args)
        except (
            FileNotFoundError,
            OSError,
            RuntimeError,
            ValueError,
            yaml.YAMLError,
        ) as exc:
            print(f'error: [setup-rejected] {exc}', file=sys.stderr)
            print(
                f'Next: {_product_command()} doctor '
                f'{shlex.quote(str(bag_path))}',
                file=sys.stderr,
            )
            return 2
    if args.run_now and not args.dry_run and not runtime_checked:
        runtime_result = _check_runtime(result['profile']['id'])
        if runtime_result != 0:
            return runtime_result
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    elif not (args.run_now and not args.dry_run and confirmed):
        print(_render_text(result))
    if not args.run_now or args.dry_run:
        return 0
    if not confirmed:
        decision = _ask_to_start(calibration_review=False)
        if decision is None:
            return 2
        if not decision:
            print(
                'Cancelled. The setup bundle was kept; mapping was not '
                'started.'
            )
            return 0
    return _run_session(args, result)


if __name__ == '__main__':
    raise SystemExit(main())
