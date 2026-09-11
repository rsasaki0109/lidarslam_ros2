#!/usr/bin/env python3
"""Provide a safe, beginner-friendly entry point for map authoring.

The guided command owns only the interaction layer.  Bag inspection, profile
selection, map execution, verification, and lifecycle handling remain in the
existing product tools so that the guided and automation paths cannot drift.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (REPO_ROOT / 'lidarslam' / 'package.xml').is_file()
WORK_ROOT = REPO_ROOT if SOURCE_LAYOUT else Path.cwd()
RUNNER = SCRIPT_DIR / 'run_autoware_map_from_bag.py'
VIEWER = SCRIPT_DIR / 'view_autoware_map.py'
DEFAULT_MIN_FREE_SPACE_GIB = 5.0

# These are the artifacts that the delegated workflows need after the ROS
# workspace is sourced.  Keeping this check here makes a missing build visible
# before the user confirms a long run; the runner remains authoritative for
# actually starting and supervising the workflow.
_PROFILE_RUNTIME_ARTIFACTS: dict[str, tuple[tuple[str, tuple[str, ...]], ...]] = {
    'rko_lio_graph_public_path': (
        (
            'lidarslam',
            (
                'share/lidarslam/local_setup.bash',
                'share/lidarslam/launch/rko_lio_slam.launch.py',
                'share/lidarslam/param/lidarslam.yaml',
                'share/lidarslam/param/rko_lio_ntu_viral.yaml',
            ),
        ),
        (
            'rko_lio',
            (
                'share/rko_lio/local_setup.bash',
                'lib/rko_lio/offline_node',
            ),
        ),
        (
            'graph_based_slam',
            (
                'share/graph_based_slam/local_setup.bash',
                'lib/graph_based_slam/graph_based_slam_node',
            ),
        ),
    ),
    'rko_lio_graph_mid360_preset': (
        (
            'lidarslam',
            (
                'share/lidarslam/local_setup.bash',
                'share/lidarslam/launch/rko_lio_slam.launch.py',
                'share/lidarslam/param/lidarslam_mid360_rko_graph.yaml',
                'share/lidarslam/param/rko_lio_mid360.yaml',
            ),
        ),
        (
            'rko_lio',
            (
                'share/rko_lio/local_setup.bash',
                'lib/rko_lio/offline_node',
            ),
        ),
        (
            'graph_based_slam',
            (
                'share/graph_based_slam/local_setup.bash',
                'lib/graph_based_slam/graph_based_slam_node',
            ),
        ),
    ),
    'pointcloud_gnss_smoke': (
        (
            'lidarslam',
            (
                'share/lidarslam/local_setup.bash',
                'share/lidarslam/launch/lidarslam.launch.py',
                'share/lidarslam/param/lidarslam.yaml',
            ),
        ),
        (
            'graph_based_slam',
            (
                'share/graph_based_slam/local_setup.bash',
                'lib/graph_based_slam/graph_based_slam_node',
            ),
        ),
    ),
    'packet_applanix_smoke': (
        (
            'lidarslam',
            (
                'share/lidarslam/local_setup.bash',
                'share/lidarslam/launch/lidarslam.launch.py',
                'share/lidarslam/param/lidarslam.yaml',
            ),
        ),
        (
            'graph_based_slam',
            (
                'share/graph_based_slam/local_setup.bash',
                'lib/graph_based_slam/graph_based_slam_node',
            ),
        ),
    ),
}


def _load_preflight_module():
    """Load the sibling preflight module in source and installed layouts."""
    spec = importlib.util.spec_from_file_location(
        'lidarslam_guided_preflight',
        SCRIPT_DIR / 'preflight_autoware_map_bag.py',
    )
    if spec is None or spec.loader is None:
        raise RuntimeError('failed to load the product preflight module')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_profiles() -> tuple[str, ...]:
    """Return maintained profile IDs for help without duplicating the list."""
    try:
        module = _load_preflight_module()
        profiles = getattr(module, 'PROFILE_HELP', ())
        return tuple(profile_id for profile_id, _ in profiles)
    except (ImportError, OSError, RuntimeError, AttributeError, ValueError):
        # The runner remains authoritative if an installed package is being
        # inspected while its optional profile helper is unavailable.
        return ()


def _safe_output_stem(value: str) -> str:
    stem = re.sub(r'[^A-Za-z0-9._-]+', '_', value).strip('._-')
    return stem or 'bag'


def default_output_dir(bag_path: Path, now: datetime | None = None) -> Path:
    """Choose a predictable, non-overwriting default output directory."""
    now = now or datetime.now()
    return (
        Path.cwd()
        / 'output'
        / f'lidarslam_{_safe_output_stem(bag_path.stem)}_{now:%Y%m%d_%H%M%S}'
    ).resolve()


def _format_duration(value: Any) -> str:
    if value is None:
        return 'unknown'
    seconds = float(value)
    if seconds < 60:
        return f'{seconds:.1f} s'
    minutes, remainder = divmod(seconds, 60)
    return f'{int(minutes)} min {remainder:.0f} s'


def _format_topic(records: list[dict[str, Any]]) -> str:
    if not records:
        return 'not found'
    record = records[0]
    return f"{record['name']} ({record['message_count']:,} messages)"


def _recommendation_summary(
    payload: dict[str, Any],
    profile_id: str | None = None,
) -> str:
    recommendations = payload['recommendations']
    if not recommendations:
        return 'no maintained workflow can be selected safely'
    selected = recommendations[0]
    if profile_id:
        selected = next(
            (item for item in recommendations if item['id'] == profile_id),
            None,
        )
        if selected is None:
            return f'{profile_id} is not compatible with this bag'
    return f"{selected['label']} [{selected['id']}]"


def render_ready_screen(
    payload: dict[str, Any],
    output_dir: Path,
    *,
    viewer: str,
    dry_run: bool,
    profile_id: str | None = None,
    editable: bool = False,
) -> str:
    """Render the small decision screen shown before a long map run."""
    summary = payload['summary']
    inspection = summary['pointcloud_inspection']
    timestamps = summary['timestamp_order']
    action = 'would start' if dry_run else 'will start after confirmation'
    lines = [
        'lidarslam guided map setup',
        '--------------------------------',
        f"Input:       {summary['bag_path']}",
        f"Duration:    {_format_duration(summary['duration_sec'])}",
        f"LiDAR:       {_format_topic(summary['topics']['pointcloud2'])}",
        f"IMU:         {_format_topic(summary['topics']['imu'])}",
        f"Point check: {inspection['status']} — {inspection['reason']}",
        f"Time check:  {timestamps['status']} — {timestamps['reason']}",
        f"Workflow:    {_recommendation_summary(payload, profile_id)}",
        f"Output:      {output_dir}",
        f"Later edits: {'enabled' if editable else 'region cleanup only'}",
        f"Viewer:      {viewer}",
        '',
        f"Ready: {action}.",
    ]
    return '\n'.join(lines)


def render_not_ready_screen(
    payload: dict[str, Any],
    *,
    runtime_issues: Sequence[str] = (),
    profile_id: str | None = None,
) -> str:
    """Render an actionable explanation when no automatic path is safe."""
    summary = payload['summary']
    inspection = summary['pointcloud_inspection']
    timestamps = summary['timestamp_order']
    lines = [
        'lidarslam guided map setup',
        '--------------------------------',
        f"Input:    {summary['bag_path']}",
        f"Duration: {_format_duration(summary['duration_sec'])}",
        '',
        'Detected inputs:',
        f"  LiDAR: {_format_topic(summary['topics']['pointcloud2'])}",
        f"  IMU:   {_format_topic(summary['topics']['imu'])}",
        f"  GNSS:  {_format_topic(summary['topics']['navsatfix'])}",
        f"  Packet: {_format_topic(summary['topics']['velodyne_scan'])}",
        '',
        f"Point check: {inspection['status']} — {inspection['reason']}",
        f"Time check:  {timestamps['status']} — {timestamps['reason']}",
        '',
        (
            'Status: NOT READY — the input is compatible, but the local runtime is incomplete.'
            if runtime_issues
            else 'Status: NOT READY — no maintained workflow can be selected safely.'
        ),
    ]
    if profile_id:
        lines.extend([
            f'Workflow: {profile_id}',
            'Runtime check: FAILED',
        ])
    lines.extend([
        '',
        'What needs attention:',
    ])
    if runtime_issues:
        lines.append('  [runtime-incomplete] Required installed artifacts are missing.')
        lines.extend(f'    - {item}' for item in runtime_issues)
    findings = payload.get('findings') or []
    missing = payload.get('missing_requirements') or []
    if findings and not runtime_issues:
        for finding in findings:
            lines.append(f"  [{finding['code']}] {finding['message']}")
            lines.append(f"    Next: {finding['next_action']}")
    elif missing and not runtime_issues:
        lines.extend(f'  - {item}' for item in missing)
    elif not runtime_issues:
        lines.append('  - The bag did not satisfy a maintained input contract.')
    lines.extend([
        '',
        'Next steps:',
    ])
    if runtime_issues:
        lines.extend(f'  {item}' for item in _render_runtime_next_steps())
    else:
        lines.extend([
            '  1. Fix or rewrite the bag according to the checks above.',
            '  2. Run: lidarslam-map doctor <rosbag2_dir>',
            '  3. Start again only after doctor recommends a workflow.',
        ])
    return '\n'.join(lines)


def build_run_command(args: argparse.Namespace, bag_path: Path, output_dir: Path) -> list[str]:
    """Build the existing run command used by both dry-run and execution."""
    command = [
        sys.executable,
        str(RUNNER),
        str(bag_path),
        '--output-dir',
        str(output_dir),
    ]
    if args.profile:
        command.extend(['--profile', args.profile])
    if args.min_free_space_gib is not None:
        command.extend(['--min-free-space-gib', str(args.min_free_space_gib)])
    if args.verification is not None:
        command.extend(['--verification', args.verification])
    if args.dry_run:
        command.append('--dry-run')
    if args.editable:
        command.append('--editable')
    return command


def _ask_to_start() -> bool | None:
    """Return yes/no, or ``None`` when confirmation cannot be requested."""
    if not sys.stdin.isatty():
        print(
            'error: guided mode needs confirmation on a terminal; '
            'rerun with --yes for automation.',
            file=sys.stderr,
        )
        return None
    try:
        answer = input('Start mapping now? [Y/n] ').strip().lower()
    except EOFError:
        print(
            'error: no confirmation was received; rerun with --yes.',
            file=sys.stderr,
        )
        return None
    return answer in {'', 'y', 'yes'}


def _verify_result(output_dir: Path) -> str:
    verify_log = output_dir / 'verify_autoware_map.log'
    if not verify_log.is_file():
        return 'not available'
    text = verify_log.read_text(encoding='utf-8', errors='replace')
    for line in text.splitlines():
        result = line.strip()
        if result == 'RESULT: PASS' or result.startswith('RESULT: PASS --'):
            return 'PASS'
        if result == 'RESULT: FAIL' or result.startswith('RESULT: FAIL --'):
            return 'FAIL'
    return 'unknown'


def print_completion(output_dir: Path) -> None:
    """Print a compact completion card with copy-ready follow-up commands."""
    map_dir = output_dir / 'pointcloud_map'
    map_state = 'saved' if (map_dir / 'pointcloud_map_metadata.yaml').is_file() else 'missing'
    print('')
    print('lidarslam guided map result')
    print('--------------------------------')
    print(f'Map bundle:  {map_state}')
    print(f'Verification: {_verify_result(output_dir)}')
    print(f'Output:      {output_dir}')
    if (output_dir / 'backend_input' / 'metadata.yaml').is_file() and (
        output_dir / 'graph_params.ros.yaml'
    ).is_file():
        print('Later edits: accepted-loop replay is ready in this output')
    print('')
    print('Next steps:')
    print(f'  Inspect: lidarslam-map inspect {shlex.quote(str(output_dir))} --write')
    print(f'  3D preview: lidarslam-map view {shlex.quote(str(output_dir))}')
    print(f'  Foxglove: lidarslam-map view {shlex.quote(str(output_dir))} --viewer foxglove')
    print(f'  Autoware: lidarslam-map view {shlex.quote(str(output_dir))} --viewer autoware')


def _source_workspace_root() -> Path:
    """Resolve the colcon workspace that owns this source checkout."""
    # The documented layout is <workspace>/src/lidar_slam_ros2.  Keep a
    # repository-local fallback for developer checkouts that build in place,
    # then support the existing direct-child workspace layout used by local
    # product candidates.
    if REPO_ROOT.parent.name == 'src':
        return REPO_ROOT.parent.parent
    if (REPO_ROOT / 'install' / 'setup.bash').is_file():
        return REPO_ROOT
    return REPO_ROOT.parent


def _runtime_install_roots() -> tuple[Path, ...]:
    """Return prefixes in which the delegated workflow may find its packages."""
    if SOURCE_LAYOUT:
        workspace_root = _source_workspace_root()
        # Prefer the documented colcon workspace, while retaining support for
        # an intentional repository-local build.
        for candidate in (workspace_root / 'install', REPO_ROOT / 'install'):
            if (candidate / 'setup.bash').is_file():
                return (candidate,)
        return ()

    # Installed product scripts live at <prefix>/share/lidarslam/product/scripts.
    # A colcon isolated workspace keeps the other packages beside the product
    # prefix, while a merged/deb install exposes them through AMENT_PREFIX_PATH.
    prefix = REPO_ROOT.parents[2]
    candidates = [
        Path(item)
        for item in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if item
    ]
    candidates.extend((prefix, prefix.parent))
    unique: list[Path] = []
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if candidate not in unique:
            unique.append(candidate)
    return tuple(unique)


def _runtime_artifact_path(
    install_roots: Sequence[Path],
    package: str,
    relative: str,
) -> Path | None:
    """Resolve an artifact in both isolated and merged colcon installs."""
    for install_root in install_roots:
        for candidate in (
            install_root / package / relative,
            install_root / relative,
        ):
            if candidate.is_file():
                return candidate
    return None


def runtime_readiness(profile_id: str | None) -> list[str]:
    """Return actionable missing-runtime messages for a maintained profile."""
    requirements = _PROFILE_RUNTIME_ARTIFACTS.get(profile_id or '', ())
    if not requirements:
        return []

    issues: list[str] = []
    install_roots = _runtime_install_roots()
    if not install_roots:
        if SOURCE_LAYOUT:
            issues.append(
                'ROS workspace install/setup.bash was not found; build this workspace first.'
            )
        else:
            issues.append(
                'The installed ROS prefix setup.bash was not found; source the product/ROS environment first.'
            )
        return issues

    for package, artifacts in requirements:
        for relative in artifacts:
            if _runtime_artifact_path(install_roots, package, relative) is None:
                issues.append(f'missing runtime artifact: {package}/{relative}')
    return issues


def _render_runtime_next_steps() -> list[str]:
    if SOURCE_LAYOUT:
        workspace_root = _source_workspace_root()
        helper = REPO_ROOT / 'scripts' / 'source_quickstart.sh'
        setup = workspace_root / 'install' / 'setup.bash'
        return [
            f'1. Open the repository: cd {shlex.quote(str(REPO_ROOT))}',
            '2. Prepare and build it: '
            f'bash {shlex.quote(str(helper))} '
            f'--workspace {shlex.quote(str(workspace_root))} --build-only',
            f'3. Source it: source {shlex.quote(str(setup))}',
            '4. Re-run this command with --dry-run, then start the map.',
        ]
    return [
        '1. Source the installed ROS/product environment (for example: source <prefix>/setup.bash).',
        '2. Run: lidarslam-map doctor <rosbag2_dir>',
        '3. Start again after the runtime artifacts are available.',
    ]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    profiles = _load_profiles()
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Inspect a rosbag2, explain the selected workflow, and guide a '
            'safe map run through verification.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'The guided mode is for people; use "run" directly in scripts.\n'
            'The input must be the rosbag2 directory that contains metadata.yaml.\n'
            'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.\n\n'
            'Examples:\n'
            '  lidarslam-map run /path/to/rosbag2 --guided\n'
            '  lidarslam-map run /path/to/rosbag2 --guided --yes\n'
            '  lidarslam-map run /path/to/rosbag2 --guided --dry-run'
        ),
    )
    parser.add_argument(
        'bag',
        metavar='rosbag2_dir',
        help='Directory containing metadata.yaml.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no hidden advanced options).',
    )
    if profiles:
        parser.add_argument(
            '--profile',
            choices=profiles,
            metavar='<id>',
            help='Force a maintained profile instead of the doctor recommendation.',
        )
    else:
        parser.add_argument('--profile', metavar='<id>')
    parser.add_argument(
        '--output-dir',
        metavar='<dir>',
        help='Directory for generated map outputs (default: output/lidarslam_<bag>_<timestamp>).',
    )
    parser.add_argument(
        '--min-free-space-gib',
        type=float,
        default=DEFAULT_MIN_FREE_SPACE_GIB,
        metavar='<GiB>',
        help=f'Refuse to start below this free-space reserve (default: {DEFAULT_MIN_FREE_SPACE_GIB:g}).',
    )
    parser.add_argument(
        '--verification',
        choices=['required', 'off'],
        help='Map verification mode (default: required; off is diagnosis-only).',
    )
    parser.add_argument(
        '--viewer',
        choices=['none', 'browser', 'autoware', 'foxglove'],
        default='none',
        help='Open the completed map in this viewer (default: none).',
    )
    parser.add_argument(
        '--yes',
        action='store_true',
        help='Start without asking for confirmation.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show the decision and exact delegated command without mapping.',
    )
    parser.add_argument(
        '--editable',
        action='store_true',
        help='Keep backend replay input so accepted loops can be disabled later.',
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.min_free_space_gib) or args.min_free_space_gib <= 0:
        parser.error('--min-free-space-gib must be finite and greater than zero')
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    bag_path = Path(args.bag).expanduser().resolve()
    try:
        preflight = _load_preflight_module()
        preflight.validate_bag_path(bag_path)
        payload = preflight.build_preflight_payload(bag_path)
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else default_output_dir(bag_path)
    )
    selected_profile = None
    if payload['recommendations']:
        selected_profile = preflight.select_profile(payload, args.profile)
    if selected_profile and selected_profile not in {
        item['id'] for item in payload['recommendations']
    }:
        payload = {
            **payload,
            'recommendations': [],
            'findings': [{
                'code': 'profile-incompatible',
                'message': (
                    f'Profile {selected_profile!r} is not compatible with this bag.'
                ),
                'next_action': (
                    'Remove --profile or choose one listed by '
                    'lidarslam-map doctor <rosbag2_dir>.'
                ),
            }],
            'missing_requirements': [
                f'Profile {selected_profile!r} is not compatible with this bag.',
                'Remove --profile or choose one listed by lidarslam-map doctor.',
            ],
        }
    if not payload['recommendations']:
        print(render_not_ready_screen(payload))
        return 2

    runtime_issues = runtime_readiness(selected_profile)
    if runtime_issues:
        print(render_not_ready_screen(
            payload,
            runtime_issues=runtime_issues,
            profile_id=selected_profile,
        ))
        return 2

    print(render_ready_screen(
        payload,
        output_dir,
        viewer=args.viewer,
        dry_run=args.dry_run,
        profile_id=selected_profile,
        editable=args.editable,
    ))
    command = build_run_command(args, bag_path, output_dir)
    if args.dry_run:
        print('')
        print('Delegated command:')
        print('  ' + shlex.join(command))
        return subprocess.run(command, check=False, cwd=WORK_ROOT).returncode

    if not args.yes:
        decision = _ask_to_start()
        if decision is None:
            return 2
        if not decision:
            print('Cancelled. No map workflow was started.')
            return 0

    print('')
    print('Starting map workflow. Progress and final diagnosis will be kept in:')
    print(f'  {output_dir}')
    result = subprocess.run(command, check=False, cwd=WORK_ROOT)
    if result.returncode != 0:
        print(
            f'guided map run failed with exit code {result.returncode}.',
            file=sys.stderr,
        )
        print(f'Inspect retained evidence: lidarslam-map inspect {shlex.quote(str(output_dir))} --write')
        return result.returncode

    print_completion(output_dir)
    if args.viewer != 'none':
        view_command = [
            sys.executable,
            str(VIEWER),
            str(output_dir),
            '--viewer',
            args.viewer,
        ]
        viewer_result = subprocess.run(view_command, check=False, cwd=WORK_ROOT)
        if viewer_result.returncode != 0:
            print(
                'warning: map run completed, but the viewer failed with '
                f'exit code {viewer_result.returncode}.',
                file=sys.stderr,
            )
            return viewer_result.returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
