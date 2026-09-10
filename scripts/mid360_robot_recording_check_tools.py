#!/usr/bin/env python3
"""Post-recording checks for MID-360 robot bags."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import (
    AutowarePreflightAdapter,
    MapRunOptions,
    Mid360MapRunPlanner,
    Mid360ReadinessReporter,
    Mid360RobotPreflight,
    Mid360RunManifestWriter,
    RobotFrames,
    RobotProfile,
    payload_to_json,
)


RECORDING_CHECK_JSON = 'mid360_robot_recording_check.json'
RECORDING_CHECK_MARKDOWN = 'mid360_robot_recording_check.md'


@dataclass(frozen=True)
class RecordingCheck:
    """A single post-recording consistency check."""

    id: str
    status: str
    message: str


@dataclass(frozen=True)
class RecordingCheckInputs:
    """Resolved inputs for post-recording checks."""

    bag_path: Path
    output_dir: Path
    profile_path: Path
    record_plan_path: Path | None = None


def auto_record_plan_path(bag_path: Path) -> Path:
    """Return the default sidecar recording plan path for a bag directory."""
    return bag_path.parent / f'{bag_path.name}_record_plan.json'


def auto_profile_snapshot_path(bag_path: Path) -> Path:
    """Return the default sidecar profile snapshot path for a bag directory."""
    return bag_path.parent / f'{bag_path.name}_profile.yaml'


def load_recording_plan(path: Path | None) -> dict[str, Any]:
    """Load a recording plan JSON sidecar when it exists."""
    if path is None or not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def build_readiness_for_recording(
    repo_root: Path,
    bag_path: Path,
    output_dir: Path,
    profile: RobotProfile,
    frames: RobotFrames,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    """Build readiness and map-plan payloads for a recorded MID-360 bag."""
    preflight = Mid360RobotPreflight(AutowarePreflightAdapter(repo_root))
    preflight_payload = preflight.build_payload(bag_path, frames, profile=profile)
    payload: dict[str, Any] = {'preflight': preflight_payload}
    plan_error = ''

    try:
        plan = Mid360MapRunPlanner(repo_root).build_plan(
            bag_path=bag_path,
            payload=preflight_payload,
            frames=frames,
            options=MapRunOptions(output_dir=output_dir),
        )
        payload['plan'] = plan.to_dict()
    except ValueError as exc:
        plan_error = str(exc)

    report = Mid360ReadinessReporter().build_report(
        payload=payload,
        output_dir=output_dir,
        plan_error=plan_error,
    )
    return payload, report, plan_error


class Mid360RecordingCheckReporter:
    """Build and write post-recording check reports."""

    def build_report(
        self,
        inputs: RecordingCheckInputs,
        profile: RobotProfile | None,
        recording_plan: dict[str, Any],
        readiness_report: dict[str, Any],
        payload: dict[str, Any],
        paths: dict[str, str],
    ) -> dict[str, Any]:
        checks = self._build_checks(inputs, profile, recording_plan, readiness_report, payload)
        check_payload = [asdict(check) for check in checks]
        record_plan_path = inputs.record_plan_path or auto_record_plan_path(inputs.bag_path)
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': self._status_from_checks(check_payload),
            'bag_path': str(inputs.bag_path),
            'output_dir': str(inputs.output_dir),
            'profile_path': str(inputs.profile_path),
            'record_plan_path': str(record_plan_path),
            'recording_plan': self._recording_plan_summary(recording_plan),
            'readiness_status': readiness_report.get('status', 'FAIL'),
            'readiness_json_path': paths.get('readiness_json', ''),
            'readiness_markdown_path': paths.get('readiness_markdown', ''),
            'map_plan_json_path': paths.get('map_plan_json', ''),
            'map_plan_markdown_path': paths.get('map_plan_markdown', ''),
            'checks': check_payload,
            'counts': self._count_checks(check_payload),
            'selected_topics': readiness_report.get('selected_topics', {}),
            'frames': readiness_report.get('frames', {}),
        }

    def build_error_report(
        self,
        bag_path: Path,
        output_dir: Path,
        profile_path: Path | None,
        record_plan_path: Path | None,
        message: str,
    ) -> dict[str, Any]:
        check = {
            'id': 'recording_check_setup',
            'status': 'fail',
            'message': message,
        }
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'FAIL',
            'bag_path': str(bag_path),
            'output_dir': str(output_dir),
            'profile_path': str(profile_path or ''),
            'record_plan_path': str(record_plan_path or ''),
            'recording_plan': {},
            'readiness_status': 'FAIL',
            'readiness_json_path': '',
            'readiness_markdown_path': '',
            'map_plan_json_path': '',
            'map_plan_markdown_path': '',
            'checks': [check],
            'counts': self._count_checks([check]),
            'selected_topics': {},
            'frames': {},
        }

    def write(self, report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / RECORDING_CHECK_JSON
        markdown_path = output_dir / RECORDING_CHECK_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(self.render_markdown(report) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    @staticmethod
    def render_markdown(report: dict[str, Any]) -> str:
        lines = [
            '# MID-360 Robot Recording Check',
            '',
            f"- status: `{report['status']}`",
            f"- created_at: `{report['created_at']}`",
            f"- bag_path: `{report['bag_path']}`",
            f"- profile_path: `{report['profile_path']}`",
            f"- record_plan_path: `{report['record_plan_path']}`",
            f"- readiness_status: `{report['readiness_status']}`",
            '',
            '## Outputs',
            '',
            f"- readiness_json: `{report['readiness_json_path']}`",
            f"- readiness_markdown: `{report['readiness_markdown_path']}`",
            f"- map_plan_json: `{report['map_plan_json_path']}`",
            f"- map_plan_markdown: `{report['map_plan_markdown_path']}`",
            '',
            '## Checks',
            '',
        ]
        for check in report['checks']:
            lines.append(f"- `{check['status']}` `{check['id']}`: {check['message']}")

        plan = report.get('recording_plan') or {}
        if plan:
            lines.extend([
                '',
                '## Recording Plan',
                '',
                f"- run_id: `{plan.get('run_id')}`",
                f"- bag_path: `{plan.get('bag_path')}`",
                f"- profile_snapshot_path: `{plan.get('profile_snapshot_path')}`",
                '',
                '### Recorded Topics',
                '',
            ])
            for topic in plan.get('topics', []):
                lines.append(f"- `{topic}`")
        return '\n'.join(lines)

    @staticmethod
    def _build_checks(
        inputs: RecordingCheckInputs,
        profile: RobotProfile | None,
        recording_plan: dict[str, Any],
        readiness_report: dict[str, Any],
        payload: dict[str, Any],
    ) -> list[RecordingCheck]:
        checks = [
            RecordingCheck(
                id='metadata_yaml',
                status='ok' if (inputs.bag_path / 'metadata.yaml').is_file() else 'fail',
                message=(
                    f'metadata.yaml exists under {inputs.bag_path}'
                    if (inputs.bag_path / 'metadata.yaml').is_file()
                    else f'metadata.yaml is missing under {inputs.bag_path}'
                ),
            ),
            Mid360RecordingCheckReporter._readiness_check(readiness_report),
            RecordingCheck(
                id='map_dry_run_plan',
                status='ok' if 'plan' in payload else 'fail',
                message=(
                    'MID-360 map dry-run plan was generated.'
                    if 'plan' in payload
                    else 'MID-360 map dry-run plan was not generated.'
                ),
            ),
        ]
        checks.extend(
            Mid360RecordingCheckReporter._record_plan_checks(
                inputs,
                profile,
                recording_plan,
            )
        )
        return checks

    @staticmethod
    def _readiness_check(readiness_report: dict[str, Any]) -> RecordingCheck:
        status = readiness_report.get('status', 'FAIL')
        if status == 'PASS':
            return RecordingCheck('readiness_status', 'ok', 'Bag readiness passed.')
        if status == 'WARN':
            return RecordingCheck('readiness_status', 'warn', 'Bag readiness has warnings.')
        return RecordingCheck('readiness_status', 'fail', 'Bag readiness failed.')

    @staticmethod
    def _record_plan_checks(
        inputs: RecordingCheckInputs,
        profile: RobotProfile | None,
        recording_plan: dict[str, Any],
    ) -> list[RecordingCheck]:
        if not recording_plan:
            return [
                RecordingCheck(
                    id='recording_plan',
                    status='warn',
                    message='Recording plan sidecar was not found; checking bag/profile only.',
                )
            ]

        checks = [
            Mid360RecordingCheckReporter._record_plan_bag_check(inputs, recording_plan),
            Mid360RecordingCheckReporter._record_plan_profile_check(inputs, recording_plan),
        ]
        if profile is not None:
            checks.append(
                Mid360RecordingCheckReporter._record_plan_topics_check(profile, recording_plan)
            )
        return checks

    @staticmethod
    def _record_plan_bag_check(
        inputs: RecordingCheckInputs,
        recording_plan: dict[str, Any],
    ) -> RecordingCheck:
        planned_bag = Path(str(recording_plan.get('bag_path', ''))).expanduser()
        if planned_bag == inputs.bag_path:
            return RecordingCheck(
                'recording_plan_bag_path', 'ok',
                'Recording plan bag path matches.',
            )
        return RecordingCheck(
            'recording_plan_bag_path',
            'warn',
            f'Recording plan bag path differs: {planned_bag} != {inputs.bag_path}',
        )

    @staticmethod
    def _record_plan_profile_check(
        inputs: RecordingCheckInputs,
        recording_plan: dict[str, Any],
    ) -> RecordingCheck:
        planned_profile = Path(
            str(recording_plan.get('profile_snapshot_path', ''))
        ).expanduser()
        if planned_profile == inputs.profile_path:
            return RecordingCheck(
                'recording_plan_profile_snapshot',
                'ok',
                'Profile snapshot path matches the recording plan.',
            )
        return RecordingCheck(
            'recording_plan_profile_snapshot',
            'warn',
            (
                'Profile path differs from recording plan snapshot: '
                f'{planned_profile} != {inputs.profile_path}'
            ),
        )

    @staticmethod
    def _record_plan_topics_check(
        profile: RobotProfile,
        recording_plan: dict[str, Any],
    ) -> RecordingCheck:
        topics = set(recording_plan.get('topics') or [])
        missing = [
            topic for topic in (
                profile.expected_pointcloud_topic,
                profile.expected_imu_topic,
            )
            if topic and topic not in topics
        ]
        if not missing:
            return RecordingCheck(
                'recording_plan_topics',
                'ok',
                'Recording plan includes expected point cloud and IMU topics.',
            )
        return RecordingCheck(
            'recording_plan_topics',
            'warn',
            f'Recording plan is missing expected topics: {", ".join(missing)}',
        )

    @staticmethod
    def _recording_plan_summary(recording_plan: dict[str, Any]) -> dict[str, Any]:
        if not recording_plan:
            return {}
        return {
            'run_id': recording_plan.get('run_id', ''),
            'bag_path': recording_plan.get('bag_path', ''),
            'topics': recording_plan.get('topics', []),
            'profile_snapshot_path': recording_plan.get('profile_snapshot_path', ''),
        }

    @staticmethod
    def _status_from_checks(checks: list[dict[str, Any]]) -> str:
        if any(check['status'] == 'fail' for check in checks):
            return 'FAIL'
        if any(check['status'] == 'warn' for check in checks):
            return 'WARN'
        return 'PASS'

    @staticmethod
    def _count_checks(checks: list[dict[str, Any]]) -> dict[str, int]:
        return {
            'ok': sum(1 for check in checks if check['status'] == 'ok'),
            'warn': sum(1 for check in checks if check['status'] == 'warn'),
            'fail': sum(1 for check in checks if check['status'] == 'fail'),
        }


def write_readiness_artifacts(
    payload: dict[str, Any],
    readiness_report: dict[str, Any],
    output_dir: Path,
) -> dict[str, str]:
    """Write readiness and map-plan artifacts used by the recording check."""
    paths = Mid360ReadinessReporter().write(readiness_report, output_dir)
    result = {
        'readiness_json': str(paths['json']),
        'readiness_markdown': str(paths['markdown']),
        'map_plan_json': '',
        'map_plan_markdown': '',
    }
    if 'plan' in payload:
        manifest_paths = Mid360RunManifestWriter().write(payload)
        result['map_plan_json'] = str(manifest_paths['json'])
        result['map_plan_markdown'] = str(manifest_paths['markdown'])
    return result
