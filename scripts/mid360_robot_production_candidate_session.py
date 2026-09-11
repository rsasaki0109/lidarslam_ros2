#!/usr/bin/env python3
"""Run a production-candidate MID-360 robot mapping session workflow."""

from __future__ import annotations

import shlex
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools import resolve_benchmark_resource
from lidarslam_benchmark_tools.mid360_robot_dashboard import DASHBOARD_HTML, write_dashboard
from lidarslam_benchmark_tools.mid360_robot_production_readiness import (
    PRODUCTION_READINESS_JSON,
    PRODUCTION_READINESS_MARKDOWN,
)
from lidarslam_benchmark_tools.mid360_robot_public_rko_adoption_gate import RKO_ADOPTION_GATE_JSON
from lidarslam_benchmark_tools.mid360_robot_public_rko_sweep import RKO_SWEEP_JSON
from lidarslam_benchmark_tools.mid360_robot_record_tools import (
    Mid360RecordManifestWriter,
    Mid360RobotRecordPlanner,
    RecordOptions,
    RecordPlan,
)
from lidarslam_benchmark_tools.mid360_robot_tools import RobotProfileLoader, payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_CANDIDATE_SESSION_JSON = 'mid360_robot_production_candidate_session.json'
PRODUCTION_CANDIDATE_SESSION_MARKDOWN = 'mid360_robot_production_candidate_session.md'
DEFAULT_PUBLIC_RKO_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'rko_sweep'
DEFAULT_PUBLIC_RKO_SWEEP = DEFAULT_PUBLIC_RKO_OUTPUT_DIR / RKO_SWEEP_JSON
DEFAULT_PUBLIC_RKO_CONFIG = (
    REPO_ROOT
    / 'configs'
    / 'mid360_robot'
    / 'rko_lio_mid360_low_voxel_no_deskew.yaml'
)
SEGMENT_MAP_ALIGNMENT_JSON = 'mid360_robot_public_segment_map_cloud_alignment.json'
SEGMENT_MAP_ALIGNMENT_MARKDOWN = 'mid360_robot_public_segment_map_cloud_alignment.md'
SEGMENT_MAP_ALIGNMENT_PLY = 'mid360_robot_public_segment_map_cloud_alignment.ply'


@dataclass(frozen=True)
class ProductionCandidateSessionOptions:
    """Operator options for one production-candidate MID-360 robot session."""

    profile_path: Path
    bag_root: Path
    output_dir: Path | None = None
    run_id: str = ''
    duration_sec: str = '600'
    include_tf: bool = True
    include_tf_static: bool = True
    extra_topics: tuple[str, ...] = ()
    storage_id: str = ''
    max_cache_size: str = ''
    compression_mode: str = ''
    compression_format: str = ''
    host_root: Path = Path('/')
    skip_host_readiness: bool = False
    record_only: bool = False
    skip_map: bool = False
    skip_public_gate: bool = False
    from_existing_artifacts: bool = False
    execute: bool = False
    public_rko_run: bool = False
    public_rko_sweep: Path | None = None
    public_rko_config: Path = DEFAULT_PUBLIC_RKO_CONFIG
    public_rko_output_dir: Path | None = None
    adoption_gate: Path | None = None
    segment_map_alignment: Path | None = None
    allow_non_best: bool = False
    map_run_name: str = ''
    map_save_timeout_secs: str = ''
    map_startup_timeout_secs: str = ''
    min_bag_duration_sec: float = 600.0
    min_pointcloud_hz: float = 5.0
    min_imu_hz: float = 50.0
    allow_warnings: bool = False
    allow_public_bag: bool = False


@dataclass(frozen=True)
class ProductionCandidateStep:
    """One production-candidate workflow step."""

    id: str
    status: str
    message: str
    command: list[str]
    returncode: int | None = None


class Mid360ProductionCandidateSessionRunner:
    """Run the real-robot workflow up to the production-readiness gate."""

    def __init__(self, repo_root: Path = REPO_ROOT) -> None:
        self._repo_root = repo_root

    def run(self, options: ProductionCandidateSessionOptions, *, quiet: bool = False) -> dict[str, Any]:
        """Build or execute the production-candidate workflow."""
        profile_path = options.profile_path.expanduser().resolve()
        profile = RobotProfileLoader().load(profile_path)
        plan = Mid360RobotRecordPlanner().build_plan(profile, self._record_options(options))
        output_dir = self._resolve_output_dir(options, plan.run_id)
        output_dir.mkdir(parents=True, exist_ok=True)

        steps: list[ProductionCandidateStep] = []
        Mid360RecordManifestWriter().write(profile, plan)
        steps.append(
            ProductionCandidateStep(
                id='recording_plan',
                status='ok',
                message='Recording plan and profile snapshot written.',
                command=self._record_command(options, plan.run_id, dry_run=True),
            )
        )

        self._append_host_step(steps, options, output_dir, quiet=quiet)
        self._append_recording_step(steps, options, plan, quiet=quiet)
        self._append_post_check_step(steps, options, plan, output_dir, quiet=quiet)
        self._append_map_step(steps, options, plan, output_dir, quiet=quiet)
        self._append_public_gate_step(steps, options, output_dir, quiet=quiet)
        self._append_production_readiness_step(steps, options, output_dir, quiet=quiet)

        report = self._build_report(
            options=options,
            profile_path=profile_path,
            plan=plan,
            output_dir=output_dir,
            steps=steps,
        )
        self.write_report(report, output_dir)
        dashboard_path = write_dashboard(output_dir)
        report['dashboard_html_path'] = str(dashboard_path)
        report['artifact_paths']['dashboard_html'] = str(dashboard_path)
        self.write_report(report, output_dir)
        return report

    def _append_host_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
        *,
        quiet: bool,
    ) -> None:
        command = self._host_command(options, output_dir)
        if options.from_existing_artifacts:
            steps.append(_existing_artifact_step(
                'host_readiness',
                'Host readiness artifact reused.',
                output_dir / 'jetson_mid360_host_readiness.json',
                command,
            ))
        elif options.skip_host_readiness:
            steps.append(_skipped_step('host_readiness', 'Host readiness skipped by option.', command))
        elif self._blocked(steps):
            steps.append(_blocked_step('host_readiness', command))
        elif options.execute:
            steps.append(_run_step('host_readiness', 'Host readiness completed.', command, self._repo_root, quiet))
        else:
            steps.append(_planned_step('host_readiness', 'Host readiness command planned.', command))

    def _append_recording_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        plan: RecordPlan,
        *,
        quiet: bool,
    ) -> None:
        command = self._record_command(options, plan.run_id, dry_run=not options.execute)
        if options.from_existing_artifacts:
            steps.append(_existing_artifact_step(
                'recording',
                'Recording evidence artifacts reused.',
                plan.bag_path,
                command,
                required=False,
            ))
        elif self._blocked(steps):
            steps.append(_blocked_step('recording', command))
        elif options.execute:
            steps.append(_run_step('recording', 'Recording completed.', command, self._repo_root, quiet))
        else:
            steps.append(_planned_step('recording', 'Recording command planned.', command))

    def _append_post_check_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        plan: RecordPlan,
        output_dir: Path,
        *,
        quiet: bool,
    ) -> None:
        command = self._post_check_command(plan, output_dir)
        if options.from_existing_artifacts:
            steps.append(_existing_artifact_step(
                'post_recording_check',
                'Post-recording check artifacts reused.',
                output_dir / 'mid360_robot_recording_check.json',
                command,
            ))
        elif options.record_only:
            steps.append(_skipped_step('post_recording_check', 'Post-recording check skipped by --record-only.', command))
        elif self._blocked(steps):
            steps.append(_blocked_step('post_recording_check', command))
        elif options.execute:
            steps.append(_run_step('post_recording_check', 'Post-recording check completed.', command, self._repo_root, quiet))
        else:
            steps.append(_planned_step('post_recording_check', 'Post-recording check command planned.', command))

    def _append_map_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        plan: RecordPlan,
        output_dir: Path,
        *,
        quiet: bool,
    ) -> None:
        command = self._map_command(options, plan, output_dir)
        if options.from_existing_artifacts:
            steps.append(_existing_artifact_step(
                'map',
                'Map diagnosis artifact reused.',
                output_dir / 'autoware_map_diagnosis.json',
                command,
            ))
        elif options.record_only:
            steps.append(_skipped_step('map', 'Mapping skipped by --record-only.', command))
        elif options.skip_map:
            steps.append(_skipped_step('map', 'Mapping skipped by --skip-map.', command))
        elif self._blocked(steps):
            steps.append(_blocked_step('map', command))
        elif options.execute:
            steps.append(_run_step('map', 'Mapping and map diagnosis completed.', command, self._repo_root, quiet))
        else:
            steps.append(_planned_step('map', 'Mapping and map diagnosis command planned.', command))

    def _append_public_gate_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
        *,
        quiet: bool,
    ) -> None:
        command = self._public_rko_gate_command(options, output_dir)
        if options.from_existing_artifacts:
            steps.append(_existing_artifact_step(
                'public_rko_adoption_gate',
                'Public RKO adoption gate artifact reused.',
                self._adoption_gate_path(options, output_dir),
                command,
            ))
        elif options.record_only:
            steps.append(_skipped_step('public_rko_adoption_gate', 'Public RKO adoption gate skipped by --record-only.', command))
        elif options.skip_public_gate:
            steps.append(_skipped_step(
                'public_rko_adoption_gate',
                'Public RKO adoption gate generation skipped; production gate uses the configured adoption-gate JSON.',
                command,
            ))
        elif self._blocked(steps):
            steps.append(_blocked_step('public_rko_adoption_gate', command))
        elif options.execute:
            steps.append(_run_step(
                'public_rko_adoption_gate',
                'Public RKO adoption gate completed.',
                command,
                self._repo_root,
                quiet,
            ))
        else:
            steps.append(_planned_step('public_rko_adoption_gate', 'Public RKO adoption gate command planned.', command))

    def _append_production_readiness_step(
        self,
        steps: list[ProductionCandidateStep],
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
        *,
        quiet: bool,
    ) -> None:
        command = self._production_readiness_command(options, output_dir)
        if options.from_existing_artifacts and self._blocked(steps):
            steps.append(_blocked_step('production_readiness', command))
        elif options.record_only:
            steps.append(_skipped_step('production_readiness', 'Production readiness skipped by --record-only.', command))
        elif options.skip_map and not options.from_existing_artifacts:
            steps.append(_skipped_step('production_readiness', 'Production readiness skipped because mapping was skipped.', command))
        elif self._blocked(steps):
            steps.append(_blocked_step('production_readiness', command))
        elif options.execute:
            steps.append(_run_step(
                'production_readiness',
                'Production readiness gate completed.',
                command,
                self._repo_root,
                quiet,
            ))
        else:
            steps.append(_planned_step('production_readiness', 'Production readiness gate command planned.', command))

    def _record_options(self, options: ProductionCandidateSessionOptions) -> RecordOptions:
        return RecordOptions(
            bag_root=options.bag_root.expanduser().resolve(),
            run_id=options.run_id,
            include_tf=options.include_tf,
            include_tf_static=options.include_tf_static,
            extra_topics=tuple(options.extra_topics or ()),
            storage_id=options.storage_id,
            max_cache_size=options.max_cache_size,
            compression_mode=options.compression_mode,
            compression_format=options.compression_format,
            duration_sec=options.duration_sec,
        )

    def _resolve_output_dir(self, options: ProductionCandidateSessionOptions, run_id: str) -> Path:
        if options.output_dir is not None:
            return options.output_dir.expanduser().resolve()
        return self._repo_root / 'output' / f'mid360_robot_production_candidate_{run_id}'

    def _record_command(
        self,
        options: ProductionCandidateSessionOptions,
        run_id: str,
        *,
        dry_run: bool,
    ) -> list[str]:
        command = [
            'bash',
            str(self._repo_root / 'scripts' / 'record_mid360_robot_bag.sh'),
            '--robot-profile',
            str(options.profile_path.expanduser().resolve()),
            '--bag-root',
            str(options.bag_root.expanduser().resolve()),
            '--run-id',
            run_id,
        ]
        if options.duration_sec:
            command.extend(['--duration-sec', options.duration_sec])
        for topic in options.extra_topics or ():
            command.extend(['--extra-topic', topic])
        if not options.include_tf:
            command.append('--no-tf')
        if not options.include_tf_static:
            command.append('--no-tf-static')
        if options.storage_id:
            command.extend(['--storage-id', options.storage_id])
        if options.max_cache_size:
            command.extend(['--max-cache-size', options.max_cache_size])
        if options.compression_mode:
            command.extend(['--compression-mode', options.compression_mode])
        if options.compression_format:
            command.extend(['--compression-format', options.compression_format])
        if dry_run:
            command.append('--dry-run')
        return command

    def _host_command(self, options: ProductionCandidateSessionOptions, output_dir: Path) -> list[str]:
        return [
            'python3',
            str(self._repo_root / 'scripts' / 'check_jetson_mid360_host_readiness.py'),
            '--bag-dir',
            str(options.bag_root.expanduser().resolve()),
            '--output-dir',
            str(output_dir),
            '--host-root',
            str(options.host_root.expanduser().resolve()),
        ]

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

    def _map_command(
        self,
        options: ProductionCandidateSessionOptions,
        plan: RecordPlan,
        output_dir: Path,
    ) -> list[str]:
        command = [
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
        if options.map_run_name:
            command.extend(['--run-name', options.map_run_name])
        if options.map_save_timeout_secs:
            command.extend(['--save-timeout-secs', options.map_save_timeout_secs])
        if options.map_startup_timeout_secs:
            command.extend(['--startup-timeout-secs', options.map_startup_timeout_secs])
        return command

    def _public_rko_gate_command(
        self,
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
    ) -> list[str]:
        mode = '--run' if options.public_rko_run else '--from-existing'
        command = [
            'python3',
            str(self._repo_root / 'scripts' / 'run_mid360_robot_public_rko_adoption_gate.py'),
            mode,
            '--output-dir',
            str(self._public_rko_output_dir(options, output_dir)),
            '--config',
            str(options.public_rko_config.expanduser().resolve()),
        ]
        sweep = self._public_rko_sweep(options)
        if sweep is not None:
            command.extend(['--sweep', str(sweep)])
        if options.allow_non_best:
            command.append('--allow-non-best')
        return command

    def _production_readiness_command(
        self,
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
    ) -> list[str]:
        command = [
            'python3',
            str(resolve_benchmark_resource(
                'scripts/check_mid360_robot_production_readiness.py')),
            '--artifact-dir',
            str(output_dir),
            '--host-readiness',
            str(output_dir / 'jetson_mid360_host_readiness.json'),
            '--recording-check',
            str(output_dir / 'mid360_robot_recording_check.json'),
            '--readiness',
            str(output_dir / 'mid360_robot_readiness.json'),
            '--map-diagnosis',
            str(output_dir / 'autoware_map_diagnosis.json'),
            '--adoption-gate',
            str(self._adoption_gate_path(options, output_dir)),
            '--output-dir',
            str(output_dir),
            '--min-bag-duration-sec',
            f'{options.min_bag_duration_sec:g}',
            '--min-pointcloud-hz',
            f'{options.min_pointcloud_hz:g}',
            '--min-imu-hz',
            f'{options.min_imu_hz:g}',
        ]
        if options.allow_warnings:
            command.append('--allow-warnings')
        if options.allow_public_bag:
            command.append('--allow-public-bag')
        return command

    def _public_rko_output_dir(
        self,
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
    ) -> Path:
        if options.public_rko_output_dir is not None:
            return options.public_rko_output_dir.expanduser().resolve()
        return output_dir / 'public_rko_adoption_gate'

    def _public_rko_sweep(self, options: ProductionCandidateSessionOptions) -> Path | None:
        if options.public_rko_sweep is not None:
            return options.public_rko_sweep.expanduser().resolve()
        if options.public_rko_run:
            return None
        return DEFAULT_PUBLIC_RKO_SWEEP

    def _adoption_gate_path(
        self,
        options: ProductionCandidateSessionOptions,
        output_dir: Path,
    ) -> Path:
        if options.adoption_gate is not None:
            return options.adoption_gate.expanduser().resolve()
        if options.skip_public_gate:
            return DEFAULT_PUBLIC_RKO_OUTPUT_DIR / RKO_ADOPTION_GATE_JSON
        return self._public_rko_output_dir(options, output_dir) / RKO_ADOPTION_GATE_JSON

    def _build_report(
        self,
        *,
        options: ProductionCandidateSessionOptions,
        profile_path: Path,
        plan: RecordPlan,
        output_dir: Path,
        steps: list[ProductionCandidateStep],
    ) -> dict[str, Any]:
        public_output_dir = self._public_rko_output_dir(options, output_dir)
        segment_map_alignment_json = _optional_artifact_path(
            output_dir,
            options.segment_map_alignment,
            SEGMENT_MAP_ALIGNMENT_JSON,
        )
        segment_map_alignment_markdown = _sibling_optional_artifact_path(
            segment_map_alignment_json,
            SEGMENT_MAP_ALIGNMENT_MARKDOWN,
        )
        segment_map_alignment_ply = _sibling_optional_artifact_path(
            segment_map_alignment_json,
            SEGMENT_MAP_ALIGNMENT_PLY,
        )
        artifacts = {
            'profile_snapshot': str(plan.profile_snapshot_path),
            'record_plan_json': str(plan.manifest_json_path),
            'record_plan_markdown': str(plan.manifest_markdown_path),
            'host_readiness_json': str(output_dir / 'jetson_mid360_host_readiness.json'),
            'recording_check_json': str(output_dir / 'mid360_robot_recording_check.json'),
            'readiness_json': str(output_dir / 'mid360_robot_readiness.json'),
            'map_plan_json': str(output_dir / 'mid360_robot_run_plan.json'),
            'map_diagnosis_json': str(output_dir / 'autoware_map_diagnosis.json'),
            'public_rko_adoption_gate_json': str(self._adoption_gate_path(options, output_dir)),
            'public_rko_adoption_gate_markdown': str(public_output_dir / 'mid360_robot_public_rko_adoption_gate.md'),
            'production_readiness_json': str(output_dir / PRODUCTION_READINESS_JSON),
            'production_readiness_markdown': str(output_dir / PRODUCTION_READINESS_MARKDOWN),
            'segment_map_alignment_json': str(segment_map_alignment_json),
            'segment_map_alignment_markdown': str(segment_map_alignment_markdown),
            'segment_map_alignment_ply': str(segment_map_alignment_ply),
            'dashboard_html': str(output_dir / DASHBOARD_HTML),
            'production_candidate_session_json': str(output_dir / PRODUCTION_CANDIDATE_SESSION_JSON),
            'production_candidate_session_markdown': str(output_dir / PRODUCTION_CANDIDATE_SESSION_MARKDOWN),
        }
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _status_from_steps(steps),
            'production_candidate_session': True,
            'dry_run': not options.execute,
            'record_only': options.record_only,
            'skip_map': options.skip_map,
            'skip_public_gate': options.skip_public_gate,
            'from_existing_artifacts': options.from_existing_artifacts,
            'public_rko_run': options.public_rko_run,
            'run_id': plan.run_id,
            'bag_root': str(plan.bag_root),
            'bag_path': str(plan.bag_path),
            'output_dir': str(output_dir),
            'profile_path': str(profile_path),
            'profile_snapshot_path': str(plan.profile_snapshot_path),
            'record_plan_json_path': str(plan.manifest_json_path),
            'record_plan_markdown_path': str(plan.manifest_markdown_path),
            'host_readiness_json_path': artifacts['host_readiness_json'],
            'recording_check_json_path': artifacts['recording_check_json'],
            'readiness_json_path': artifacts['readiness_json'],
            'map_plan_json_path': artifacts['map_plan_json'],
            'map_diagnosis_json_path': artifacts['map_diagnosis_json'],
            'public_rko_adoption_gate_json_path': artifacts['public_rko_adoption_gate_json'],
            'production_readiness_json_path': artifacts['production_readiness_json'],
            'dashboard_html_path': artifacts['dashboard_html'],
            'production_candidate_session_json_path': artifacts['production_candidate_session_json'],
            'production_candidate_session_markdown_path': artifacts['production_candidate_session_markdown'],
            'thresholds': {
                'min_bag_duration_sec': options.min_bag_duration_sec,
                'min_pointcloud_hz': options.min_pointcloud_hz,
                'min_imu_hz': options.min_imu_hz,
                'allow_warnings': options.allow_warnings,
                'allow_public_bag': options.allow_public_bag,
            },
            'artifact_paths': artifacts,
            'steps': [asdict(step) for step in steps],
            'counts': _count_steps(steps),
            'next_actions': _next_actions(options, steps),
        }

    @staticmethod
    def write_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        """Write JSON and Markdown production-candidate reports."""
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / PRODUCTION_CANDIDATE_SESSION_JSON
        markdown_path = output_dir / PRODUCTION_CANDIDATE_SESSION_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(render_production_candidate_session_markdown(report) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    @staticmethod
    def _blocked(steps: list[ProductionCandidateStep]) -> bool:
        return any(step.status == 'fail' for step in steps)


def render_production_candidate_session_markdown(report: dict[str, Any]) -> str:
    """Render a concise production-candidate session report."""
    lines = [
        '# MID-360 Robot Production Candidate Session',
        '',
        f"- status: `{report['status']}`",
        f"- created_at: `{report['created_at']}`",
        f"- dry_run: `{report['dry_run']}`",
        f"- run_id: `{report['run_id']}`",
        f"- bag_path: `{report['bag_path']}`",
        f"- output_dir: `{report['output_dir']}`",
        '',
        '## Artifacts',
        '',
    ]
    artifacts = report.get('artifact_paths') or {}
    for key in (
        'profile_snapshot',
        'record_plan_json',
        'host_readiness_json',
        'recording_check_json',
        'readiness_json',
        'map_diagnosis_json',
        'public_rko_adoption_gate_json',
        'production_readiness_json',
        'segment_map_alignment_json',
        'dashboard_html',
        'production_candidate_session_json',
    ):
        lines.append(f"- {key}: `{artifacts.get(key, '')}`")

    lines.extend(['', '## Steps', ''])
    for step in report.get('steps', []):
        lines.append(f"- `{step.get('status', '')}` `{step.get('id', '')}`: {step.get('message', '')}")
        if step.get('command'):
            lines.extend(['', '```bash', shlex.join(step['command']), '```', ''])

    actions = report.get('next_actions') or []
    lines.extend(['', '## Next Actions', ''])
    if actions:
        for action in actions:
            lines.append(f'- {action}')
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _run_step(
    step_id: str,
    success_message: str,
    command: list[str],
    repo_root: Path,
    quiet: bool,
) -> ProductionCandidateStep:
    completed = subprocess.run(
        command,
        check=False,
        cwd=repo_root,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.DEVNULL if quiet else None,
    )
    status = 'ok' if completed.returncode == 0 else 'fail'
    message = success_message if status == 'ok' else f'{success_message} failed.'
    return ProductionCandidateStep(
        id=step_id,
        status=status,
        message=message,
        command=command,
        returncode=completed.returncode,
    )


def _planned_step(step_id: str, message: str, command: list[str]) -> ProductionCandidateStep:
    return ProductionCandidateStep(
        id=step_id,
        status='planned',
        message=message,
        command=command,
    )


def _skipped_step(step_id: str, message: str, command: list[str] | None = None) -> ProductionCandidateStep:
    return ProductionCandidateStep(
        id=step_id,
        status='skipped',
        message=message,
        command=command or [],
    )


def _existing_artifact_step(
    step_id: str,
    message: str,
    artifact_path: Path,
    command: list[str],
    *,
    required: bool = True,
) -> ProductionCandidateStep:
    path = artifact_path.expanduser().resolve()
    if path.exists() or not required:
        suffix = f' ({path})' if path.exists() else ''
        return ProductionCandidateStep(
            id=step_id,
            status='ok',
            message=f'{message}{suffix}',
            command=command,
        )
    return ProductionCandidateStep(
        id=step_id,
        status='fail',
        message=f'Missing required existing artifact: {path}',
        command=command,
        returncode=1,
    )


def _blocked_step(step_id: str, command: list[str]) -> ProductionCandidateStep:
    return _skipped_step(step_id, 'Step skipped because an earlier production-candidate step failed.', command)


def _status_from_steps(steps: list[ProductionCandidateStep]) -> str:
    if any(step.status == 'fail' for step in steps):
        return 'FAIL'
    if any(step.status == 'warn' for step in steps):
        return 'WARN'
    return 'PASS'


def _optional_artifact_path(output_dir: Path, explicit_path: Path | None, default_name: str) -> Path:
    if explicit_path is not None:
        return explicit_path.expanduser().resolve()
    return output_dir / default_name


def _sibling_optional_artifact_path(json_path: Path, sibling_name: str) -> Path:
    return json_path.parent / sibling_name


def _count_steps(steps: list[ProductionCandidateStep]) -> dict[str, int]:
    return {
        'ok': sum(1 for step in steps if step.status == 'ok'),
        'warn': sum(1 for step in steps if step.status == 'warn'),
        'planned': sum(1 for step in steps if step.status == 'planned'),
        'skipped': sum(1 for step in steps if step.status == 'skipped'),
        'fail': sum(1 for step in steps if step.status == 'fail'),
    }


def _next_actions(
    options: ProductionCandidateSessionOptions,
    steps: list[ProductionCandidateStep],
) -> list[str]:
    if not options.execute:
        if options.from_existing_artifacts:
            return ['Run the same command with --run to evaluate the existing artifacts through the production gate.']
        return ['Run the same command with --run on the Jetson when the planned commands and paths are correct.']
    failed = [step for step in steps if step.status == 'fail']
    if not failed:
        return []
    first = failed[0]
    if first.id == 'host_readiness':
        return ['Fix Jetson host readiness before recording a production candidate bag.']
    if first.id == 'recording':
        return ['Inspect ros2 bag record output, storage space, and MID-360 topic availability.']
    if first.id == 'post_recording_check':
        return ['Open mid360_robot_recording_check.md and fix bag/profile/topic issues before mapping.']
    if first.id == 'map':
        return ['Inspect Autoware/RKO-LIO logs and autoware_map_diagnosis.md before rerunning the production gate.']
    if first.id == 'public_rko_adoption_gate':
        return ['Regenerate the public RKO sweep/adoption evidence or pass --skip-public-gate with a valid --adoption-gate path.']
    if first.id == 'production_readiness':
        return ['Open mid360_robot_production_readiness.md and address each failing production gate check.']
    return ['Inspect the failed step output and rerun the production-candidate session.']


if __name__ == "__main__":
    raise SystemExit("mid360_robot_production_candidate_session is a library module; import it instead of running it directly.")
