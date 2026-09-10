#!/usr/bin/env python3
"""Build a MID-360 robot map run plan."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_tools import (
    AutowarePreflightAdapter,
    MapRunOptions,
    Mid360RunManifestWriter,
    Mid360RunDiagnosisPlanner,
    Mid360MapRunPlanner,
    Mid360RobotPreflight,
    RobotFrames,
    RobotProfile,
    RobotProfileLoader,
    payload_to_json,
    resolve_robot_frames,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def build_plan_payload(
    bag_path: Path,
    frames: RobotFrames,
    options: MapRunOptions,
    profile: RobotProfile | None = None,
) -> dict[str, object]:
    """Build preflight and command-plan payloads for a MID-360 map run."""
    preflight = Mid360RobotPreflight(AutowarePreflightAdapter(REPO_ROOT))
    preflight_payload = preflight.build_payload(bag_path, frames, profile=profile)
    plan = Mid360MapRunPlanner(REPO_ROOT).build_plan(
        bag_path=bag_path,
        payload=preflight_payload,
        frames=frames,
        options=options,
    )
    return {
        'preflight': preflight_payload,
        'plan': plan.to_dict(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Plan or run a MID-360 robot mapping job.')
    parser.add_argument('bag', help='Path to a rosbag2 directory that contains metadata.yaml.')
    parser.add_argument('--robot-profile', help='Robot profile YAML for expected topics and frames.')
    parser.add_argument('--base-frame', default='', help='Robot body frame.')
    parser.add_argument('--lidar-frame', default='', help='MID-360 LiDAR frame.')
    parser.add_argument('--imu-frame', default='', help='MID-360 IMU frame.')
    parser.add_argument('--output-dir', help='Directory for SLAM outputs and logs.')
    parser.add_argument('--run-name', default='', help='RKO-LIO run_name.')
    parser.add_argument('--save-timeout-secs', default='', help='Timeout waiting for map files.')
    parser.add_argument('--startup-timeout-secs', default='', help='Timeout waiting for SLAM startup.')
    parser.add_argument(
        '--viewer',
        choices=['none', 'autoware', 'foxglove'],
        default='none',
        help='Viewer to open after the map run.',
    )
    viewer_aliases = parser.add_mutually_exclusive_group()
    viewer_aliases.add_argument(
        '--foxglove',
        action='store_const',
        const='foxglove',
        dest='viewer_alias',
        help='Alias for --viewer foxglove.',
    )
    viewer_aliases.add_argument(
        '--autoware',
        action='store_const',
        const='autoware',
        dest='viewer_alias',
        help='Alias for --viewer autoware.',
    )
    viewer_aliases.add_argument(
        '--no-viewer',
        action='store_const',
        const='none',
        dest='viewer_alias',
        help='Alias for --viewer none.',
    )
    parser.add_argument('--viewer-rebuild', action='store_true', help='Rebuild viewer runtime.')
    parser.add_argument('--viewer-run-dir', default='', help='Existing viewer runtime.')
    parser.add_argument('--autoware-core-dir', default='', help='autoware_core checkout.')
    parser.add_argument('--work-dir', default='', help='Runtime workspace directory.')
    parser.add_argument('--auto-exit-secs', default='', help='Auto-close viewer after N seconds.')
    parser.add_argument('--keep-launch', action='store_true', help='Keep SLAM launch alive.')
    parser.add_argument('--json', action='store_true', help='Emit the run plan as JSON.')
    parser.add_argument('--dry-run', action='store_true', help='Print commands without executing.')
    parser.add_argument(
        '--write-manifest',
        action='store_true',
        help='Write mid360_robot_run_plan.json and .md under output-dir.',
    )
    parser.add_argument(
        '--write-diagnosis',
        action='store_true',
        help='After a real run, write autoware_map_diagnosis.md/json under output-dir.',
    )
    args = parser.parse_args()
    if args.viewer_alias is not None:
        args.viewer = args.viewer_alias
    return args


def _options_from_args(args: argparse.Namespace) -> MapRunOptions:
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else None
    return MapRunOptions(
        output_dir=output_dir,
        run_name=args.run_name,
        save_timeout_secs=args.save_timeout_secs,
        startup_timeout_secs=args.startup_timeout_secs,
        viewer=args.viewer,
        viewer_rebuild=args.viewer_rebuild,
        viewer_run_dir=args.viewer_run_dir,
        autoware_core_dir=args.autoware_core_dir,
        work_dir=args.work_dir,
        auto_exit_secs=args.auto_exit_secs,
        keep_launch=args.keep_launch,
    )


def _profile_from_args(args: argparse.Namespace) -> RobotProfile | None:
    if not args.robot_profile:
        return None
    return RobotProfileLoader().load(Path(args.robot_profile).expanduser().resolve())


def _frames_from_args(args: argparse.Namespace, profile: RobotProfile | None) -> RobotFrames:
    return resolve_robot_frames(
        base_frame=args.base_frame,
        lidar_frame=args.lidar_frame,
        imu_frame=args.imu_frame,
        profile=profile,
    )


def _print_human_report(payload: dict[str, object]) -> None:
    preflight_payload = payload['preflight']
    plan = payload['plan']
    preflight = Mid360RobotPreflight(AutowarePreflightAdapter(REPO_ROOT))
    print('MID-360 robot preflight:')
    print(preflight.render_text_report(preflight_payload))
    print()
    print('Run command:')
    print(f"  {plan['dogfood_command_shell']}")
    if plan['foxglove_command_shell']:
        print('Foxglove command:')
        print(f"  {plan['foxglove_command_shell']}")


def _write_manifest_if_requested(args: argparse.Namespace, payload: dict[str, object]) -> None:
    if not args.write_manifest:
        return
    paths = Mid360RunManifestWriter().write(payload)
    print(f"Manifest JSON: {paths['json']}", file=sys.stderr)
    print(f"Manifest Markdown: {paths['markdown']}", file=sys.stderr)


def _attach_diagnosis_plan_if_requested(
    args: argparse.Namespace,
    payload: dict[str, object],
    bag_path: Path,
) -> None:
    if not args.write_diagnosis:
        return
    output_dir = Path(payload['plan']['output_dir'])
    payload['diagnosis'] = Mid360RunDiagnosisPlanner(REPO_ROOT).build_plan(
        output_dir=output_dir,
        bag_path=bag_path,
    ).to_dict()


def _run_diagnosis_if_requested(args: argparse.Namespace, payload: dict[str, object]) -> None:
    diagnosis = payload.get('diagnosis')
    if not args.write_diagnosis or not diagnosis:
        return
    output_dir = Path(payload['plan']['output_dir'])
    output_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(diagnosis['command'], check=True, cwd=REPO_ROOT)
    diagnosis['ran'] = True
    print(f"Diagnosis Markdown: {diagnosis['markdown_path']}")
    print(f"Diagnosis JSON: {diagnosis['json_path']}")


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    profile = _profile_from_args(args)
    frames = _frames_from_args(args, profile)
    payload = build_plan_payload(
        bag_path=bag_path,
        frames=frames,
        options=_options_from_args(args),
        profile=profile,
    )
    _attach_diagnosis_plan_if_requested(args, payload, bag_path)
    if args.json:
        _write_manifest_if_requested(args, payload)
        print(payload_to_json(payload))
        return 0

    _print_human_report(payload)
    _write_manifest_if_requested(args, payload)
    if args.dry_run:
        return 0

    plan = payload['plan']
    dogfood_error = None
    diagnosis_error = None
    try:
        subprocess.run(plan['dogfood_command'], check=True, cwd=REPO_ROOT)
    except subprocess.CalledProcessError as exc:
        dogfood_error = exc
    finally:
        if args.write_diagnosis:
            try:
                _run_diagnosis_if_requested(args, payload)
            except subprocess.CalledProcessError as exc:
                diagnosis_error = exc
            _write_manifest_if_requested(args, payload)
    if dogfood_error is not None:
        if diagnosis_error is not None:
            print(
                'Warning: diagnosis command failed after the SLAM command failed.',
                file=sys.stderr,
            )
        raise dogfood_error
    if diagnosis_error is not None:
        raise diagnosis_error
    if plan['foxglove_command']:
        subprocess.run(plan['foxglove_command'], check=True, cwd=REPO_ROOT)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
