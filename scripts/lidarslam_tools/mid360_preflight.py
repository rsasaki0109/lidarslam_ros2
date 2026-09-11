"""MID-360 topic selection, validation, and launch preflight."""

from __future__ import annotations

import shlex
import textwrap
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .autoware_preflight import AutowarePreflightAdapter
from .mid360_bag_diagnostics import Mid360BagDiagnosticsBuilder
from .mid360_models import PreflightCheck, RobotFrames, RobotProfile, TopicSelection

MID360_PROFILE_ID = "rko_lio_graph_mid360_preset"

class Mid360RobotPreflight:
    """Build and render MID-360 robot preflight reports."""

    def __init__(
        self,
        adapter: AutowarePreflightAdapter,
        diagnostics_builder: Mid360BagDiagnosticsBuilder | None = None,
    ) -> None:
        self._adapter = adapter
        self._diagnostics_builder = diagnostics_builder or Mid360BagDiagnosticsBuilder()

    def build_payload(
        self,
        bag_path: Path,
        frames: RobotFrames,
        profile: RobotProfile | None = None,
    ) -> dict[str, Any]:
        """Build a robot-focused preflight payload from rosbag2 metadata."""
        autoware_payload = self._adapter.build_payload(bag_path)
        summary = autoware_payload['summary']
        topics = self._select_topics(summary, profile)
        has_mid360 = self._has_mid360_recommendation(autoware_payload)
        bag_diagnostics = self._diagnostics_builder.build(bag_path, summary, topics, frames)
        checks = self._build_checks(
            summary,
            topics,
            has_mid360,
            profile,
            bag_diagnostics,
        )

        return {
            'summary': summary,
            'checks': [asdict(check) for check in checks],
            'ready_for_mid360_launch': (
                topics.ready
                and self._profile_topics_ok(summary, profile)
                and not any(check.status == 'fail' for check in checks)
            ),
            'launch_command': self._build_launch_command(summary, topics, frames),
            'frames': asdict(frames),
            'selected_topics': asdict(topics),
            'bag_diagnostics': bag_diagnostics,
            'robot_profile': profile.to_dict() if profile else {},
            'next_actions': [
                'Confirm base_link -> livox_frame static transform from measurement.',
                'Record one stationary bag and one short walking bag before a full route.',
                'Run /map_save after the offline launch completes and verify pointcloud_map.',
            ],
            'autoware_preflight': autoware_payload,
        }

    def render_text_report(self, payload: dict[str, Any]) -> str:
        """Render a human-readable robot preflight report."""
        summary = payload['summary']
        duration = summary['duration_sec']
        duration_text = f'{duration:.3f}s' if duration is not None else 'unknown'

        lines = [
            'MID-360 Robot Bag Preflight',
            f"bag: {summary['bag_path']}",
            f"duration: {duration_text}",
            f"messages: {summary['message_count']}",
            '',
            'Checks:',
        ]
        for check in payload['checks']:
            lines.append(f"  [{check['status'].upper()}] {check['message']}")

        diagnostics = payload.get('bag_diagnostics') or {}
        if diagnostics:
            pointcloud = diagnostics['topics']['pointcloud']
            imu = diagnostics['topics']['imu']
            lines.extend([
                '',
                'Bag diagnostics:',
                f"  pointcloud metadata rate: {self._format_hz(pointcloud['metadata_rate_hz'])}",
                f"  imu metadata rate: {self._format_hz(imu['metadata_rate_hz'])}",
                f"  pointcloud sampled frames: {self._format_frames(pointcloud)}",
                f"  imu sampled frames: {self._format_frames(imu)}",
                (
                    '  message sampling: available'
                    if diagnostics['sample_reader']['available']
                    else f"  message sampling: unavailable ({diagnostics['sample_reader']['reason']})"
                ),
            ])

        if payload['launch_command']:
            lines.extend([
                '',
                'Recommended MID-360 launch:',
                textwrap.indent(payload['launch_command'], '  '),
            ])
        else:
            lines.extend([
                '',
                'Recommended MID-360 launch: unavailable until PointCloud2 and Imu topics exist.',
            ])

        lines.append('')
        lines.append('Next actions:')
        for action in payload['next_actions']:
            lines.append(f'  - {action}')

        return '\n'.join(lines)

    @staticmethod
    def _format_hz(value: float | None) -> str:
        return f'{value:.2f} Hz' if value is not None else 'unknown'

    @staticmethod
    def _format_frames(topic_diagnostics: dict[str, Any]) -> str:
        frames = topic_diagnostics.get('sampled_frame_ids') or []
        return ', '.join(frames) if frames else 'not sampled'

    def _select_topics(
        self,
        summary: dict[str, Any],
        profile: RobotProfile | None,
    ) -> TopicSelection:
        pointcloud = self._select_topic_name(
            summary,
            'pointcloud2',
            profile.expected_pointcloud_topic if profile else '',
        )
        imu = self._select_topic_name(
            summary,
            'imu',
            profile.expected_imu_topic if profile else '',
        )
        return TopicSelection(pointcloud=pointcloud, imu=imu)

    @staticmethod
    def _select_topic_name(summary: dict[str, Any], key: str, expected: str) -> str | None:
        topics = summary['topics'][key]
        if expected:
            return expected if any(item['name'] == expected for item in topics) else None
        return topics[0]['name'] if topics else None

    @staticmethod
    def _has_mid360_recommendation(payload: dict[str, Any]) -> bool:
        return any(item['id'] == MID360_PROFILE_ID for item in payload['recommendations'])

    @staticmethod
    def _build_checks(
        summary: dict[str, Any],
        topics: TopicSelection,
        has_mid360: bool,
        profile: RobotProfile | None,
        bag_diagnostics: dict[str, Any],
    ) -> list[PreflightCheck]:
        checks = [
            PreflightCheck(
                id='pointcloud2',
                status='ok' if topics.pointcloud else 'fail',
                message=(
                    f'PointCloud2 topic: {topics.pointcloud}'
                    if topics.pointcloud else 'No PointCloud2 topic found.'
                ),
            ),
            PreflightCheck(
                id='imu',
                status='ok' if topics.imu else 'fail',
                message=f'Imu topic: {topics.imu}' if topics.imu else 'No Imu topic found.',
            ),
            PreflightCheck(
                id='mid360_preset',
                status='ok' if has_mid360 else 'warn',
                message=(
                    'Livox/MID-360 preset recommendation is available.'
                    if has_mid360
                    else 'Bag path/topics do not look Livox/MID-360-specific.'
                ),
            ),
            PreflightCheck(
                id='tf_metadata',
                status='ok' if summary['capabilities']['has_tf'] else 'warn',
                message=(
                    'TF or TF_STATIC topic exists in the bag metadata.'
                    if summary['capabilities']['has_tf']
                    else 'No TF/TF_STATIC topic found in metadata; pass static frames explicitly.'
                ),
            ),
        ]
        if profile and profile.expected_pointcloud_topic:
            checks.append(
                Mid360RobotPreflight._expected_topic_check(
                    summary,
                    key='pointcloud2',
                    check_id='expected_pointcloud_topic',
                    expected=profile.expected_pointcloud_topic,
                )
            )
        if profile and profile.expected_imu_topic:
            checks.append(
                Mid360RobotPreflight._expected_topic_check(
                    summary,
                    key='imu',
                    check_id='expected_imu_topic',
                    expected=profile.expected_imu_topic,
                )
            )
        checks.extend(Mid360RobotPreflight._bag_diagnostic_checks(bag_diagnostics))
        return checks

    @staticmethod
    def _bag_diagnostic_checks(bag_diagnostics: dict[str, Any]) -> list[PreflightCheck]:
        topics = bag_diagnostics.get('topics') or {}
        checks = []
        for key, label in (
            ('pointcloud', 'PointCloud2'),
            ('imu', 'Imu'),
        ):
            topic_diagnostics = topics.get(key) or {}
            rate_check = Mid360RobotPreflight._metadata_rate_check(
                topic_diagnostics,
                check_id=f'{key}_metadata_rate',
                label=label,
            )
            if rate_check:
                checks.append(rate_check)
            frame_check = Mid360RobotPreflight._sample_frame_check(
                topic_diagnostics,
                check_id=f'{key}_frame_id',
                label=label,
            )
            if frame_check:
                checks.append(frame_check)
        checks.extend(Mid360RobotPreflight._tf_sample_checks(bag_diagnostics.get('tf') or {}))
        return checks

    @staticmethod
    def _metadata_rate_check(
        topic_diagnostics: dict[str, Any],
        check_id: str,
        label: str,
    ) -> PreflightCheck | None:
        topic = topic_diagnostics.get('topic')
        if not topic:
            return None
        rate_hz = topic_diagnostics.get('metadata_rate_hz')
        min_rate_hz = float(topic_diagnostics.get('min_metadata_rate_hz') or 0.0)
        if rate_hz is None:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=f'{label} metadata rate is unknown because bag duration is missing.',
            )
        if float(rate_hz) < min_rate_hz:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=(
                    f'{label} metadata rate is {float(rate_hz):.2f} Hz on {topic}; '
                    f'expected at least {min_rate_hz:.2f} Hz for a field check.'
                ),
            )
        return PreflightCheck(
            id=check_id,
            status='ok',
            message=f'{label} metadata rate is {float(rate_hz):.2f} Hz on {topic}.',
        )

    @staticmethod
    def _sample_frame_check(
        topic_diagnostics: dict[str, Any],
        check_id: str,
        label: str,
    ) -> PreflightCheck | None:
        frame_ids = topic_diagnostics.get('sampled_frame_ids') or []
        if not frame_ids:
            if topic_diagnostics.get('sampled_message_count', 0) > 0:
                return PreflightCheck(
                    id=check_id,
                    status='fail',
                    message=(
                        f'{label} sampled messages have no non-empty '
                        'frame_id; fix the publisher header.frame_id and '
                        'repeat preflight.'
                    ),
                )
            return None
        expected = topic_diagnostics.get('expected_frame_id') or ''
        if expected and topic_diagnostics.get('matches_expected_frame') is False:
            return PreflightCheck(
                id=check_id,
                status='fail',
                message=(
                    f'{label} sampled frame_id does not match expected {expected}: '
                    f'{", ".join(frame_ids)}'
                ),
            )
        if topic_diagnostics.get('stable_frame_id') is False:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=(
                    f'{label} sampled frame_id changed '
                    f"{topic_diagnostics.get('frame_id_changes')} times: "
                    f'{", ".join(frame_ids)}'
                ),
            )
        return PreflightCheck(
            id=check_id,
            status='ok',
            message=f'{label} sampled frame_id is stable: {frame_ids[0]}',
        )

    @staticmethod
    def _tf_sample_checks(tf_diagnostics: dict[str, Any]) -> list[PreflightCheck]:
        if not tf_diagnostics.get('sampled_message_count'):
            return []
        checks = []
        for key, label in (
            ('base_to_lidar_connected', 'base frame to lidar frame'),
            ('base_to_imu_connected', 'base frame to imu frame'),
        ):
            connected = tf_diagnostics.get(key)
            if connected is None:
                continue
            checks.append(
                PreflightCheck(
                    id=f'tf_{key}',
                    status='ok' if connected else 'warn',
                    message=(
                        f'TF samples connect {label}.'
                        if connected
                        else f'TF samples do not connect {label}; verify static extrinsics.'
                    ),
                )
            )
        return checks

    @staticmethod
    def _expected_topic_check(
        summary: dict[str, Any],
        key: str,
        check_id: str,
        expected: str,
    ) -> PreflightCheck:
        available = [item['name'] for item in summary['topics'][key]]
        if expected in available:
            return PreflightCheck(
                id=check_id,
                status='ok',
                message=f'Profile expected topic is present: {expected}',
            )
        return PreflightCheck(
            id=check_id,
            status='fail',
            message=(
                f'Profile expected topic is missing: {expected}. '
                f'Available: {", ".join(available) if available else "none"}'
            ),
        )

    @staticmethod
    def _profile_topics_ok(summary: dict[str, Any], profile: RobotProfile | None) -> bool:
        if profile is None:
            return True
        if profile.expected_pointcloud_topic:
            available = [item['name'] for item in summary['topics']['pointcloud2']]
            if profile.expected_pointcloud_topic not in available:
                return False
        if profile.expected_imu_topic:
            available = [item['name'] for item in summary['topics']['imu']]
            if profile.expected_imu_topic not in available:
                return False
        return True

    @staticmethod
    def _build_launch_command(
        summary: dict[str, Any],
        topics: TopicSelection,
        frames: RobotFrames,
    ) -> str:
        if not topics.ready:
            return ''

        return textwrap.dedent(
            f"""\
            ros2 launch lidarslam rko_lio_slam.launch.py \\
              main_param_dir:=lidarslam/param/lidarslam_mid360_rko_graph.yaml \\
              rko_param_file:=lidarslam/param/rko_lio_mid360.yaml \\
              bag_path:={shlex.quote(summary['bag_path'])} \\
              lidar_topic:={shlex.quote(topics.pointcloud or '')} \\
              imu_topic:={shlex.quote(topics.imu or '')} \\
              base_frame:={shlex.quote(frames.base_frame)} \\
              lidar_frame:={shlex.quote(frames.lidar_frame)} \\
              imu_frame:={shlex.quote(frames.imu_frame)}"""
        )
