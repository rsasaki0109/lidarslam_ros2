#!/usr/bin/env python3
"""Run a complete MID-360 field-session simulation from a generated rosbag2."""

from __future__ import annotations

import shlex
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_dashboard import DASHBOARD_HTML, write_dashboard
from lidarslam_benchmark_tools.mid360_robot_record_tools import (
    Mid360RecordManifestWriter,
    Mid360RobotRecordPlanner,
    RecordOptions,
    RecordPlan,
)
from lidarslam_benchmark_tools.mid360_robot_recording_check_tools import (
    Mid360RecordingCheckReporter,
    RecordingCheckInputs,
    build_readiness_for_recording,
    load_recording_plan,
    write_readiness_artifacts,
)
from lidarslam_benchmark_tools.mid360_robot_sample_bag import Mid360SampleBagWriter, SampleBagConfig
from lidarslam_benchmark_tools.mid360_robot_tools import (
    RobotProfile,
    RobotProfileLoader,
    payload_to_json,
    resolve_robot_frames,
)


FIELD_SESSION_JSON = 'mid360_robot_field_session.json'
FIELD_SESSION_MARKDOWN = 'mid360_robot_field_session.md'
SAMPLE_SESSION_SCENARIOS = ('pass', 'low-rate', 'frame-mismatch', 'missing-tf')


@dataclass(frozen=True)
class SampleSessionOptions:
    """Options for one synthetic MID-360 field session."""

    profile_path: Path
    bag_root: Path
    output_dir: Path
    run_id: str = 'mid360_sample_session'
    duration_sec: float = 5.0
    pointcloud_rate_hz: float = 10.0
    imu_rate_hz: float = 100.0
    point_count: int = 32
    scenario: str = 'pass'
    force: bool = False


@dataclass(frozen=True)
class SampleSessionStep:
    """One synthetic field-session step."""

    id: str
    status: str
    message: str
    command: list[str]
    returncode: int | None = None


@dataclass(frozen=True)
class EffectiveSampleSettings:
    """Resolved synthetic bag settings after applying a QA scenario."""

    duration_sec: float
    pointcloud_rate_hz: float
    imu_rate_hz: float
    point_count: int
    base_frame: str
    lidar_frame: str
    imu_frame: str
    write_tf_static: bool


class Mid360SampleSessionRunner:
    """Generate sample inputs and run the regular post-recording toolchain."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    def run(self, options: SampleSessionOptions) -> dict[str, Any]:
        profile_path = options.profile_path.expanduser().resolve()
        profile = RobotProfileLoader().load(profile_path)
        settings = resolve_effective_sample_settings(profile, options)
        plan = Mid360RobotRecordPlanner().build_plan(
            profile,
            RecordOptions(
                bag_root=options.bag_root.expanduser().resolve(),
                run_id=options.run_id,
                duration_sec=f'{options.duration_sec:g}',
            ),
        )
        output_dir = options.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

        steps: list[SampleSessionStep] = []
        Mid360RecordManifestWriter().write(profile, plan)
        steps.append(
            SampleSessionStep(
                id='recording_plan',
                status='ok',
                message='Synthetic recording plan and profile snapshot written.',
                command=list(plan.command),
            )
        )

        sample_summary = self._write_sample_bag(profile, plan, settings, options)
        steps.append(
            SampleSessionStep(
                id='recording',
                status='ok',
                message='Synthetic MID-360 rosbag2 generated.',
                command=self._sample_bag_command(options, plan, settings),
            )
        )

        recording_report = self._run_post_recording_check(
            profile=profile,
            plan=plan,
            output_dir=output_dir,
        )
        steps.append(
            SampleSessionStep(
                id='post_recording_check',
                status=self._step_status_from_report(recording_report),
                message='Post-recording check completed.',
                command=self._post_check_command(plan, output_dir),
            )
        )

        map_step = self._map_step(plan, output_dir, steps[-1])
        steps.append(map_step)

        report = self._build_report(
            options=options,
            profile_path=profile_path,
            plan=plan,
            output_dir=output_dir,
            steps=steps,
            sample_summary=sample_summary,
            settings=settings,
        )
        self.write_report(report, output_dir)
        dashboard_path = write_dashboard(output_dir)
        report['dashboard_html_path'] = str(dashboard_path)
        self.write_report(report, output_dir)
        return report

    def _write_sample_bag(
        self,
        profile: RobotProfile,
        plan: RecordPlan,
        settings: EffectiveSampleSettings,
        options: SampleSessionOptions,
    ) -> dict[str, Any]:
        config = SampleBagConfig(
            output_path=plan.bag_path,
            duration_sec=settings.duration_sec,
            pointcloud_rate_hz=settings.pointcloud_rate_hz,
            imu_rate_hz=settings.imu_rate_hz,
            point_count=settings.point_count,
            pointcloud_topic=profile.expected_pointcloud_topic,
            imu_topic=profile.expected_imu_topic,
            base_frame=settings.base_frame,
            lidar_frame=settings.lidar_frame,
            imu_frame=settings.imu_frame,
            write_tf_static=settings.write_tf_static,
            force=options.force,
        )
        return Mid360SampleBagWriter(config).write().to_dict()

    def _run_post_recording_check(
        self,
        *,
        profile: RobotProfile,
        plan: RecordPlan,
        output_dir: Path,
    ) -> dict[str, Any]:
        frames = resolve_robot_frames(profile=profile)
        payload, readiness_report, _ = build_readiness_for_recording(
            repo_root=self._repo_root,
            bag_path=plan.bag_path,
            output_dir=output_dir,
            profile=profile,
            frames=frames,
        )
        paths = write_readiness_artifacts(payload, readiness_report, output_dir)
        inputs = RecordingCheckInputs(
            bag_path=plan.bag_path,
            output_dir=output_dir,
            profile_path=plan.profile_snapshot_path,
            record_plan_path=plan.manifest_json_path,
        )
        reporter = Mid360RecordingCheckReporter()
        report = reporter.build_report(
            inputs=inputs,
            profile=profile,
            recording_plan=load_recording_plan(plan.manifest_json_path),
            readiness_report=readiness_report,
            payload=payload,
            paths=paths,
        )
        reporter.write(report, output_dir)
        return report

    def _build_report(
        self,
        *,
        options: SampleSessionOptions,
        profile_path: Path,
        plan: RecordPlan,
        output_dir: Path,
        steps: list[SampleSessionStep],
        sample_summary: dict[str, Any],
        settings: EffectiveSampleSettings,
    ) -> dict[str, Any]:
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': self._status_from_steps(steps),
            'sample_session': True,
            'dry_run': False,
            'run_map': False,
            'record_only': False,
            'run_id': plan.run_id,
            'bag_root': str(plan.bag_root),
            'bag_path': str(plan.bag_path),
            'output_dir': str(output_dir),
            'profile_path': str(profile_path),
            'profile_snapshot_path': str(plan.profile_snapshot_path),
            'record_plan_json_path': str(plan.manifest_json_path),
            'record_plan_markdown_path': str(plan.manifest_markdown_path),
            'host_readiness_json_path': '',
            'recording_check_json_path': str(output_dir / 'mid360_robot_recording_check.json'),
            'readiness_json_path': str(output_dir / 'mid360_robot_readiness.json'),
            'map_plan_json_path': str(output_dir / 'mid360_robot_run_plan.json'),
            'field_session_json_path': str(output_dir / FIELD_SESSION_JSON),
            'field_session_markdown_path': str(output_dir / FIELD_SESSION_MARKDOWN),
            'dashboard_html_path': str(output_dir / DASHBOARD_HTML),
            'sample_bag': sample_summary,
            'scenario': options.scenario,
            'sample_options': {
                'duration_sec': options.duration_sec,
                'pointcloud_rate_hz': options.pointcloud_rate_hz,
                'imu_rate_hz': options.imu_rate_hz,
                'point_count': options.point_count,
            },
            'effective_sample_options': asdict(settings),
            'steps': [asdict(step) for step in steps],
            'counts': self._count_steps(steps),
        }

    @staticmethod
    def write_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / FIELD_SESSION_JSON
        markdown_path = output_dir / FIELD_SESSION_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(render_sample_session_markdown(report) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    def _sample_bag_command(
        self,
        options: SampleSessionOptions,
        plan: RecordPlan,
        settings: EffectiveSampleSettings,
    ) -> list[str]:
        command = [
            'python3',
            str(self._repo_root / 'scripts' / 'generate_mid360_robot_sample_bag.py'),
            str(plan.bag_path),
            '--duration-sec',
            f'{settings.duration_sec:g}',
            '--pointcloud-rate-hz',
            f'{settings.pointcloud_rate_hz:g}',
            '--imu-rate-hz',
            f'{settings.imu_rate_hz:g}',
            '--point-count',
            str(settings.point_count),
            '--base-frame',
            settings.base_frame,
            '--lidar-frame',
            settings.lidar_frame,
            '--imu-frame',
            settings.imu_frame,
        ]
        if not settings.write_tf_static:
            command.append('--no-tf-static')
        if options.force:
            command.append('--force')
        return command

    def _map_step(
        self,
        plan: RecordPlan,
        output_dir: Path,
        post_step: SampleSessionStep,
    ) -> SampleSessionStep:
        command = self._map_command(plan, output_dir)
        if post_step.status == 'fail':
            return SampleSessionStep(
                id='map',
                status='skipped',
                message='Mapping skipped because sample post-recording checks failed.',
                command=command,
            )
        return SampleSessionStep(
            id='map',
            status='planned',
            message='Mapping command not executed; sample session wrote the map dry-run plan.',
            command=command,
        )

    def _post_check_command(self, plan: RecordPlan, output_dir: Path) -> list[str]:
        return [
            'bash',
            str(self._repo_root / 'scripts' / 'check_mid360_robot_recording.sh'),
            '--bag',
            str(plan.bag_path),
            '--robot-profile',
            str(plan.profile_snapshot_path),
            '--record-plan',
            str(plan.manifest_json_path),
            '--output-dir',
            str(output_dir),
        ]

    def _map_command(self, plan: RecordPlan, output_dir: Path) -> list[str]:
        return [
            'bash',
            str(self._repo_root / 'scripts' / 'run_mid360_robot_map.sh'),
            str(plan.bag_path),
            '--robot-profile',
            str(plan.profile_snapshot_path),
            '--output-dir',
            str(output_dir),
            '--write-manifest',
            '--write-diagnosis',
        ]

    @staticmethod
    def _step_status_from_report(report: dict[str, Any]) -> str:
        status = str(report.get('status') or '').upper()
        if status == 'PASS':
            return 'ok'
        if status == 'WARN':
            return 'warn'
        return 'fail'

    @staticmethod
    def _status_from_steps(steps: list[SampleSessionStep]) -> str:
        if any(step.status == 'fail' for step in steps):
            return 'FAIL'
        if any(step.status == 'warn' for step in steps):
            return 'WARN'
        return 'PASS'

    @staticmethod
    def _count_steps(steps: list[SampleSessionStep]) -> dict[str, int]:
        return {
            'ok': sum(1 for step in steps if step.status == 'ok'),
            'warn': sum(1 for step in steps if step.status == 'warn'),
            'planned': sum(1 for step in steps if step.status == 'planned'),
            'skipped': sum(1 for step in steps if step.status == 'skipped'),
            'fail': sum(1 for step in steps if step.status == 'fail'),
        }


def render_sample_session_markdown(report: dict[str, Any]) -> str:
    """Render a concise Markdown report for the synthetic session."""
    lines = [
        '# MID-360 Robot Sample Session',
        '',
        f"- status: `{report['status']}`",
        f"- created_at: `{report['created_at']}`",
        f"- scenario: `{report.get('scenario', 'pass')}`",
        f"- run_id: `{report['run_id']}`",
        f"- bag_path: `{report['bag_path']}`",
        f"- output_dir: `{report['output_dir']}`",
        '',
        '## Sample Bag',
        '',
    ]
    sample = report.get('sample_bag') or {}
    lines.extend([
        f"- pointcloud_messages: `{sample.get('pointcloud_messages')}`",
        f"- imu_messages: `{sample.get('imu_messages')}`",
        f"- tf_static_messages: `{sample.get('tf_static_messages')}`",
        '',
        '## Artifacts',
        '',
        f"- profile_snapshot: `{report['profile_snapshot_path']}`",
        f"- record_plan_json: `{report['record_plan_json_path']}`",
        f"- recording_check_json: `{report['recording_check_json_path']}`",
        f"- readiness_json: `{report['readiness_json_path']}`",
        f"- map_plan_json: `{report['map_plan_json_path']}`",
        f"- dashboard_html: `{report['dashboard_html_path']}`",
        '',
        '## Steps',
        '',
    ])
    for step in report.get('steps', []):
        lines.append(f"- `{step['status']}` `{step['id']}`: {step['message']}")
        if step.get('command'):
            lines.extend(['', '```bash', shlex.join(step['command']), '```', ''])
    return '\n'.join(lines)


def resolve_effective_sample_settings(
    profile: RobotProfile,
    options: SampleSessionOptions,
) -> EffectiveSampleSettings:
    """Resolve final bag-generation settings for a sample-session scenario."""
    if options.scenario not in SAMPLE_SESSION_SCENARIOS:
        raise ValueError(
            f'unknown sample session scenario: {options.scenario}. '
            f'Expected one of: {", ".join(SAMPLE_SESSION_SCENARIOS)}'
        )

    pointcloud_rate_hz = options.pointcloud_rate_hz
    imu_rate_hz = options.imu_rate_hz
    base_frame = profile.frames.base_frame
    lidar_frame = profile.frames.lidar_frame
    imu_frame = profile.frames.imu_frame
    write_tf_static = True

    if options.scenario == 'low-rate':
        pointcloud_rate_hz = min(pointcloud_rate_hz, 2.0)
        imu_rate_hz = min(imu_rate_hz, 20.0)
    elif options.scenario == 'frame-mismatch':
        lidar_frame = _mismatched_frame(profile.frames.lidar_frame, 'sample_wrong_lidar_frame')
        imu_frame = _mismatched_frame(profile.frames.imu_frame, 'sample_wrong_imu_frame')
    elif options.scenario == 'missing-tf':
        write_tf_static = False

    return EffectiveSampleSettings(
        duration_sec=options.duration_sec,
        pointcloud_rate_hz=pointcloud_rate_hz,
        imu_rate_hz=imu_rate_hz,
        point_count=options.point_count,
        base_frame=base_frame,
        lidar_frame=lidar_frame,
        imu_frame=imu_frame,
        write_tf_static=write_tf_static,
    )


def _mismatched_frame(expected_frame: str, fallback: str) -> str:
    """Return a deterministic frame name that differs from the expected profile frame."""
    if expected_frame != fallback:
        return fallback
    return f'{fallback}_alt'
