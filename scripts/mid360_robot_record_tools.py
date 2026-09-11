#!/usr/bin/env python3
"""MID-360 robot rosbag2 recording planner."""

from __future__ import annotations

import json
import re
import shlex
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import RobotProfile, payload_to_json


@dataclass(frozen=True)
class RecordOptions:
    """Operator options for one MID-360 robot recording."""

    bag_root: Path
    run_id: str = ''
    include_tf: bool = True
    include_tf_static: bool = True
    extra_topics: tuple[str, ...] = ()
    storage_id: str = ''
    max_cache_size: str = ''
    compression_mode: str = ''
    compression_format: str = ''
    duration_sec: str = ''


@dataclass(frozen=True)
class RecordPlan:
    """One executable rosbag2 recording plan."""

    run_id: str
    bag_root: Path
    bag_path: Path
    manifest_json_path: Path
    manifest_markdown_path: Path
    profile_snapshot_path: Path
    topics: tuple[str, ...]
    command: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            'run_id': self.run_id,
            'bag_root': str(self.bag_root),
            'bag_path': str(self.bag_path),
            'manifest_json_path': str(self.manifest_json_path),
            'manifest_markdown_path': str(self.manifest_markdown_path),
            'profile_snapshot_path': str(self.profile_snapshot_path),
            'topics': list(self.topics),
            'command': list(self.command),
            'command_shell': shlex.join(self.command),
        }


class Mid360RobotRecordPlanner:
    """Build ros2 bag record commands from a robot profile."""

    def build_plan(self, profile: RobotProfile, options: RecordOptions) -> RecordPlan:
        if not profile.expected_pointcloud_topic:
            raise ValueError('robot profile expected_pointcloud_topic is required for recording')
        if not profile.expected_imu_topic:
            raise ValueError('robot profile expected_imu_topic is required for recording')

        run_id = self._run_id(profile.robot_name, options.run_id)
        bag_root = options.bag_root
        bag_path = bag_root / run_id
        manifest_json_path = bag_root / f'{run_id}_record_plan.json'
        manifest_markdown_path = bag_root / f'{run_id}_record_plan.md'
        profile_snapshot_path = bag_root / f'{run_id}_profile.yaml'
        topics = self._topics(profile, options)
        command = self._command(bag_path, topics, options)
        return RecordPlan(
            run_id=run_id,
            bag_root=bag_root,
            bag_path=bag_path,
            manifest_json_path=manifest_json_path,
            manifest_markdown_path=manifest_markdown_path,
            profile_snapshot_path=profile_snapshot_path,
            topics=tuple(topics),
            command=tuple(command),
        )

    @staticmethod
    def _run_id(robot_name: str, requested: str) -> str:
        if requested:
            candidate = requested
        else:
            timestamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
            candidate = f'mid360_{robot_name}_{timestamp}'
        sanitized = re.sub(r'[^A-Za-z0-9_.-]+', '_', candidate).strip('._-')
        if not sanitized:
            raise ValueError('run_id must contain at least one safe filename character')
        return sanitized

    @staticmethod
    def _topics(profile: RobotProfile, options: RecordOptions) -> list[str]:
        topics = [
            profile.expected_pointcloud_topic,
            profile.expected_imu_topic,
        ]
        if options.include_tf:
            topics.append('/tf')
        if options.include_tf_static:
            topics.append('/tf_static')
        topics.extend(options.extra_topics)
        return list(dict.fromkeys(topic for topic in topics if topic))

    @staticmethod
    def _command(bag_path: Path, topics: list[str], options: RecordOptions) -> list[str]:
        command = ['ros2', 'bag', 'record', '-o', str(bag_path)]
        if options.storage_id:
            command.extend(['--storage', options.storage_id])
        if options.max_cache_size:
            command.extend(['--max-cache-size', options.max_cache_size])
        if options.compression_mode:
            command.extend(['--compression-mode', options.compression_mode])
        if options.compression_format:
            command.extend(['--compression-format', options.compression_format])
        command.extend(topics)
        if options.duration_sec:
            command = ['timeout', options.duration_sec] + command
        return command


class Mid360RecordManifestWriter:
    """Write recording plan artifacts before rosbag2 recording starts."""

    def build_manifest(self, profile: RobotProfile, plan: RecordPlan) -> dict[str, Any]:
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'run_id': plan.run_id,
            'bag_root': str(plan.bag_root),
            'bag_path': str(plan.bag_path),
            'topics': list(plan.topics),
            'command': list(plan.command),
            'command_shell': shlex.join(plan.command),
            'robot_profile': profile.to_dict(),
            'profile_snapshot_path': str(plan.profile_snapshot_path),
            'post_recording_next_steps': [
                'Run ros2 bag info on the recorded bag.',
                'Run check_mid360_robot_readiness.py with the same robot profile.',
                'Keep the profile snapshot with the bag artifacts.',
            ],
        }

    def write(self, profile: RobotProfile, plan: RecordPlan) -> dict[str, Path]:
        plan.bag_root.mkdir(parents=True, exist_ok=True)
        manifest = self.build_manifest(profile, plan)
        plan.manifest_json_path.write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
        plan.manifest_markdown_path.write_text(
            self.render_markdown(manifest) + '\n',
            encoding='utf-8',
        )
        self._write_profile_snapshot(profile, plan.profile_snapshot_path)
        return {
            'json': plan.manifest_json_path,
            'markdown': plan.manifest_markdown_path,
            'profile': plan.profile_snapshot_path,
        }

    @staticmethod
    def render_markdown(manifest: dict[str, Any]) -> str:
        lines = [
            '# MID-360 Robot Recording Plan',
            '',
            f"- created_at: `{manifest['created_at']}`",
            f"- run_id: `{manifest['run_id']}`",
            f"- bag_root: `{manifest['bag_root']}`",
            f"- bag_path: `{manifest['bag_path']}`",
            f"- profile_snapshot_path: `{manifest['profile_snapshot_path']}`",
            '',
            '## Topics',
            '',
        ]
        for topic in manifest['topics']:
            lines.append(f"- `{topic}`")
        lines.extend([
            '',
            '## Command',
            '',
            '```bash',
            manifest['command_shell'],
            '```',
            '',
            '## Robot Profile',
            '',
        ])
        profile = manifest.get('robot_profile') or {}
        lines.extend([
            f"- robot_name: `{profile.get('robot_name')}`",
            f"- expected_pointcloud_topic: `{profile.get('expected_pointcloud_topic')}`",
            f"- expected_imu_topic: `{profile.get('expected_imu_topic')}`",
            '',
            '## Next Steps',
            '',
        ])
        for item in manifest.get('post_recording_next_steps', []):
            lines.append(f"- {item}")
        return '\n'.join(lines)

    @staticmethod
    def _write_profile_snapshot(profile: RobotProfile, path: Path) -> None:
        source_path = Path(profile.source_path) if profile.source_path else None
        if source_path and source_path.is_file():
            path.write_text(source_path.read_text(encoding='utf-8'), encoding='utf-8')
            return
        path.write_text(yaml.safe_dump(profile.to_dict(), sort_keys=False), encoding='utf-8')


def record_plan_to_json(profile: RobotProfile, plan: RecordPlan) -> str:
    """Serialize a record plan with the profile snapshot payload."""
    payload = Mid360RecordManifestWriter().build_manifest(profile, plan)
    return json.dumps(payload, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit("mid360_robot_record_tools is a library module; import it instead of running it directly.")
