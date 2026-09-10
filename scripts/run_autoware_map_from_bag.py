#!/usr/bin/env python3
"""Run the shortest supported Autoware-compatible map-authoring path for a bag."""

from __future__ import annotations

import argparse
import importlib.util
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PROFILE_CHOICES = (
    'rko_lio_graph_public_path',
    'rko_lio_graph_mid360_preset',
    'pointcloud_gnss_smoke',
    'packet_applanix_smoke',
)
PROFILE_HELP = (
    (
        'rko_lio_graph_public_path',
        'PointCloud2 + Imu through RKO-LIO and graph_based_slam.',
    ),
    (
        'rko_lio_graph_mid360_preset',
        'Livox/MID360 PointCloud2 + Imu with tracked tuned params.',
    ),
    ('pointcloud_gnss_smoke', 'PointCloud2 + NavSatFix smoke workflow.'),
    ('packet_applanix_smoke', 'VelodyneScan + Applanix GSOF49 smoke workflow.'),
)


def _load_script_module(script_name: str, module_name: str):
    script_path = REPO_ROOT / 'scripts' / script_name
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load module {module_name} from {script_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _select_profile(payload: dict[str, object], forced_profile_id: str | None) -> str:
    if forced_profile_id:
        return forced_profile_id

    recommendations = payload['recommendations']
    recommendation_ids = {item['id'] for item in recommendations}
    summary = payload['summary']
    pointcloud_topics = summary['topics']['pointcloud2']
    imu_topics = summary['topics']['imu']
    bag_path_lower = summary['bag_path'].lower()
    looks_like_livox = (
        'mid360' in bag_path_lower
        or any('livox' in item['name'].lower() for item in pointcloud_topics + imu_topics)
    )
    if looks_like_livox and 'rko_lio_graph_mid360_preset' in recommendation_ids:
        return 'rko_lio_graph_mid360_preset'

    recommended_profile_id = payload['recommended_profile_id']
    if not recommended_profile_id:
        raise RuntimeError('no compatible public path was found for this bag')
    return recommended_profile_id


def build_execution_plan(
    bag_path: Path,
    profile_id: str | None,
    output_dir: Path,
    verify_map: bool,
) -> dict[str, object]:
    preflight = _load_script_module('preflight_autoware_map_bag.py', 'preflight_autoware_map_bag')
    payload = preflight.build_preflight_payload(bag_path)
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

    if selected_profile == 'rko_lio_graph_public_path':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param',
            str(REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'),
            '--rko-param',
            str(REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_ntu_viral.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'rko_lio_graph_mid360_preset':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        imu = summary['topics']['imu'][0]['name']
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', pointcloud,
            '--imu-topic', imu,
            '--lidarslam-param',
            str(REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'),
            '--rko-param', str(REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'),
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
    elif selected_profile == 'pointcloud_gnss_smoke':
        pointcloud = summary['topics']['pointcloud2'][0]['name']
        gnss = summary['topics']['navsatfix'][0]['name']
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_open_data_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--points-topic', pointcloud,
            '--gnss-topic', gnss,
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
            str(REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh'),
            '--bag', str(bag_path),
            '--packet-topic', packet,
            '--gsof49-topic', gsof49,
            '--save-dir', str(output_dir),
        ]
        if summary['capabilities']['has_applanix_gsof50']:
            command.extend(['--gsof50-topic', summary['topics']['applanix_gsof50'][0]['name']])
        if verify_map:
            command.append('--verify-map')
    else:
        raise RuntimeError(f'profile is not executable yet: {selected_profile}')

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
        raise ValueError(f'output directory path is a file, not a directory: {output_dir}')
    if output_dir.is_dir() and any(output_dir.iterdir()):
        raise ValueError(
            f'output directory is non-empty: {output_dir}. '
            'Choose a new output directory; operational resume is not supported yet, '
            'and existing artifacts are never mixed with a new run.'
        )

    for parent in output_dir.parents:
        if parent.exists():
            if not parent.is_dir():
                raise ValueError(f'output directory parent is not a directory: {parent}')
            return


def maybe_open_viewer(args: argparse.Namespace, output_dir: Path) -> None:
    if args.viewer == 'none':
        return

    if args.viewer == 'foxglove':
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
            str(output_dir),
        ]
    else:
        command = [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware.sh'),
            str(output_dir),
        ]
        if args.autoware_core_dir:
            command.extend(['--autoware-core-dir', args.autoware_core_dir])

    if args.work_dir:
        command.extend(['--work-dir', args.work_dir])
    if args.viewer_run_dir:
        command.extend(['--run-dir', args.viewer_run_dir])
    if args.viewer_rebuild:
        command.append('--rebuild')
    if args.auto_exit_secs is not None:
        command.extend(['--auto-exit-secs', str(args.auto_exit_secs)])

    subprocess.run(command, check=True, cwd=REPO_ROOT)


def maybe_verify_map(output_dir: Path, enabled: bool) -> None:
    if not enabled:
        return

    pointcloud_map_dir = output_dir / 'pointcloud_map'
    if not pointcloud_map_dir.is_dir():
        return

    verify_log_path = output_dir / 'verify_autoware_map.log'
    verify_command = [
        'python3',
        str(REPO_ROOT / 'scripts' / 'verify_autoware_map.py'),
        str(pointcloud_map_dir),
    ]
    with verify_log_path.open('w', encoding='utf-8') as stream:
        result = subprocess.run(
            verify_command,
            check=False,
            cwd=REPO_ROOT,
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
    print(
        '  Support bundle: '
        f'python3 scripts/create_map_support_bundle.py {shlex.quote(str(output_dir))}'
    )
    verify_log = output_dir / 'verify_autoware_map.log'
    if verify_log.is_file():
        print(f'  Verify log: {verify_log}')
    print(f'  Saved map:  {output_dir / "pointcloud_map"}')
    lanelet2_map = output_dir / 'lanelet2_map.osm'
    if lanelet2_map.is_file():
        print(f'  Lanelet2:   {lanelet2_map}')

    if args.viewer == 'none':
        print(
            '  Open in Foxglove: '
            'bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh '
            f'{shlex.quote(str(output_dir))}'
        )
        print(
            '  Open in Autoware viewer: '
            'bash scripts/run_graph_slam_pointcloud_map_in_autoware.sh '
            f'{shlex.quote(str(output_dir))}'
        )


def write_diagnostics(output_dir: Path, bag_path: Path) -> dict[str, object]:
    diagnose = _load_script_module('diagnose_autoware_map_run.py', 'diagnose_autoware_map_run')
    summary = diagnose.summarize_run(output_dir, bag_path)
    markdown = diagnose.render_markdown(summary)
    (output_dir / 'autoware_map_diagnosis.md').write_text(markdown, encoding='utf-8')
    (output_dir / 'autoware_map_diagnosis.json').write_text(
        __import__('json').dumps(summary, indent=2, sort_keys=True),
        encoding='utf-8',
    )
    return summary


def write_run_manifest(
    output_dir: Path,
    bag_path: Path,
    plan: dict[str, object],
    status: str,
) -> Path:
    manifest = _load_script_module('map_run_manifest.py', 'map_run_manifest')
    return manifest.write_manifest(
        repo_root=REPO_ROOT,
        output_dir=output_dir,
        bag_path=bag_path,
        profile_id=plan['profile_id'],
        command=plan['command'],
        status=status,
    )


def _profile_help_text() -> str:
    lines = ['Workflow profiles:']
    for profile_id, description in PROFILE_HELP:
        lines.append(f'  {profile_id}: {description}')
    return '\n'.join(lines)


def _help_epilog() -> str:
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
        '  map_run_manifest.json',
        '',
        'Examples:',
        '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 --dry-run',
        (
            '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 '
            '--output-dir output/my_map'
        ),
        '  python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2 --viewer foxglove',
    ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
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
        '--profile',
        choices=PROFILE_CHOICES,
        metavar='<id>',
        help='Force a compatible profile instead of the default recommendation.',
    )
    parser.add_argument(
        '--output-dir',
        metavar='<dir>',
        help=(
            'Directory for generated map outputs and logs. Defaults to '
            'output/autoware_map_authoring_<bag>_<timestamp>.'
        ),
    )
    parser.add_argument(
        '--viewer',
        choices=['none', 'autoware', 'foxglove'],
        default='none',
        help='Open the saved map after the run (default: none).',
    )
    parser.add_argument(
        '--autoware-core-dir',
        help='autoware_core checkout used by the Docker viewer.',
    )
    parser.add_argument(
        '--work-dir',
        help='Runtime workspace directory for Autoware/Foxglove viewers.',
    )
    parser.add_argument('--viewer-run-dir', help='Existing built viewer runtime to reuse.')
    parser.add_argument(
        '--viewer-rebuild',
        action='store_true',
        help='Rebuild the viewer runtime before opening.',
    )
    parser.add_argument(
        '--auto-exit-secs',
        type=int,
        help='Auto-close the viewer after N seconds.',
    )
    parser.add_argument(
        '--no-verify-map',
        action='store_true',
        help='Skip verify_autoware_map.py in smoke wrappers.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the selected command without executing it.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else (
        REPO_ROOT / 'output' / f'autoware_map_authoring_{bag_path.stem}_{timestamp}'
    )
    try:
        validate_bag_path(bag_path)
        validate_output_dir(output_dir)
        plan = build_execution_plan(
            bag_path=bag_path,
            profile_id=args.profile,
            output_dir=output_dir,
            verify_map=not args.no_verify_map,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    print(f"Selected profile: {plan['label']}")
    print(f"Output directory: {plan['output_dir']}")
    print('Command:')
    print('  ' + shlex.join(plan['command']))

    if args.dry_run:
        return 0

    try:
        output_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f'error: failed to create output directory {output_dir}: {exc}', file=sys.stderr)
        return 2

    command_error: subprocess.CalledProcessError | None = None
    try:
        subprocess.run(plan['command'], check=True, cwd=REPO_ROOT)
    except subprocess.CalledProcessError as exc:
        command_error = exc
    finally:
        if output_dir.exists():
            try:
                maybe_verify_map(output_dir, enabled=not args.no_verify_map)
                diagnosis = write_diagnostics(output_dir, bag_path)
                write_run_manifest(
                    output_dir,
                    bag_path,
                    plan,
                    status=diagnosis['status'],
                )
            except (OSError, RuntimeError, ValueError) as exc:
                print(f'warning: failed to write run diagnostics/manifest: {exc}', file=sys.stderr)

    if command_error is not None:
        print(
            f'error: map workflow failed with exit code {command_error.returncode}.',
            file=sys.stderr,
        )
        print('failed command:', shlex.join(plan['command']), file=sys.stderr)
        if (output_dir / 'autoware_map_diagnosis.md').is_file():
            print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
        return command_error.returncode or 1

    print_next_steps(args, output_dir)
    try:
        maybe_open_viewer(args, output_dir)
    except subprocess.CalledProcessError as exc:
        print(f'error: viewer failed with exit code {exc.returncode}.', file=sys.stderr)
        return exc.returncode or 1
    print(f'Diagnosis written to: {output_dir / "autoware_map_diagnosis.md"}')
    print(f'Run manifest written to: {output_dir / "map_run_manifest.json"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
