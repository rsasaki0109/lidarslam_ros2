#!/usr/bin/env python3
"""Plan segment-reset RKO-LIO runs for public MID-360 loop candidates."""

from __future__ import annotations

import json
import shlex
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_public_bag_segments import (
    PUBLIC_BAG_SEGMENTS_JSON,
    PUBLIC_BAG_SEGMENTS_MARKDOWN,
    PublicBagSegmentOptions,
    RosbagPointCloudTimingReader,
    recommend_segment,
    render_public_bag_segments_markdown,
    split_contiguous_scan_segments,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_LOOP_SEGMENT_RESET_JSON = 'mid360_robot_public_loop_segment_reset.json'
PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN = 'mid360_robot_public_loop_segment_reset.md'


@dataclass(frozen=True)
class LoopSegmentResetOptions:
    """Options for planning reset-based loop runs."""

    loop_candidates_json: Path
    bag_path: Path
    output_dir: Path
    sequence_id: str = 'outdoor_kidnap'
    pointcloud_topic: str = '/livox/points'
    imu_topic: str = '/livox/imu'
    candidate_index: int = 0
    max_scan_gap_sec: float = 10000.0
    min_segment_duration_sec: float = 5.0
    max_scans: int = 0
    min_keypoints: int = 10
    voxel_size: float = 1.0
    min_range: float = 1.0
    max_range: float = 100.0
    clip_output_root: Path = Path('datasets/mid360_public_loop_segments')
    rko_output_root: Path = Path('output/mid360_public/outdoor_kidnap_segment_reset')
    lidarslam_param: Path = Path('lidarslam/param/lidarslam_mid360_rko_graph.yaml')
    rko_param: Path = Path('configs/mid360_robot/rko_lio_mid360_kidnap_tolerant.yaml')
    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'


class LoopSegmentResetPlanner:
    """Map GT loop endpoints to RKO-safe scan segments."""

    def __init__(self, timing_reader: Any | None = None) -> None:
        self._timing_reader = timing_reader or RosbagPointCloudTimingReader()

    def plan(self, options: LoopSegmentResetOptions) -> dict[str, Any]:
        """Build and write a segment-reset plan."""
        payload = _load_json(options.loop_candidates_json)
        sequence = _select_sequence(payload, options.sequence_id)
        candidate = _select_candidate(sequence, options.candidate_index)
        segment_options = PublicBagSegmentOptions(
            max_scan_gap_sec=max(0.0, float(options.max_scan_gap_sec)),
            min_segment_duration_sec=max(0.0, float(options.min_segment_duration_sec)),
            max_scans=max(0, int(options.max_scans)),
            min_keypoints=max(0, int(options.min_keypoints)),
            voxel_size=max(0.01, float(options.voxel_size)),
            min_range=max(0.0, float(options.min_range)),
            max_range=max(0.0, float(options.max_range)),
        )
        timings = self._timing_reader.read_scan_timings(
            options.bag_path.expanduser().resolve(),
            options.pointcloud_topic,
            max_scans=segment_options.max_scans,
            voxel_size=segment_options.voxel_size,
            min_range=segment_options.min_range,
            max_range=segment_options.max_range,
        )
        split = split_contiguous_scan_segments(timings, segment_options)
        dataset_id = str(sequence.get('dataset_id') or options.sequence_id)
        segment_report = _segment_report(
            options=options,
            dataset_id=dataset_id,
            timings=timings,
            split=split,
        )
        paths = write_loop_segment_artifacts(
            output_dir=options.output_dir,
            segment_report=segment_report,
            plan_report=None,
        )

        start_match = _match_loop_endpoint(timings, candidate, 'start')
        end_match = _match_loop_endpoint(timings, candidate, 'end')
        start_segment = _segment_for_scan(split['segments'], int(start_match['matched_scan_index']))
        end_segment = _segment_for_scan(split['segments'], int(end_match['matched_scan_index']))
        reset_pair = {
            'start': _endpoint_payload(start_match, start_segment),
            'end': _endpoint_payload(end_match, end_segment),
        }
        commands = _commands(options, dataset_id, paths['segments_json'], reset_pair)
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _status(reset_pair),
            'loop_candidates_json': str(options.loop_candidates_json.expanduser().resolve()),
            'bag_path': str(options.bag_path.expanduser().resolve()),
            'output_dir': str(options.output_dir.expanduser().resolve()),
            'sequence_id': options.sequence_id,
            'dataset_id': dataset_id,
            'candidate_index': int(options.candidate_index),
            'candidate': candidate,
            'selected_topics': {
                'pointcloud': options.pointcloud_topic,
                'imu': options.imu_topic,
            },
            'segment_options': {
                'max_scan_gap_sec': segment_options.max_scan_gap_sec,
                'min_segment_duration_sec': segment_options.min_segment_duration_sec,
                'max_scans': segment_options.max_scans,
                'min_keypoints': segment_options.min_keypoints,
                'voxel_size': segment_options.voxel_size,
                'min_range': segment_options.min_range,
                'max_range': segment_options.max_range,
            },
            'scan_count': len(timings),
            'segment_count': len(split['segments']),
            'gap_count': len(split['gaps']),
            'reset_pair': reset_pair,
            'artifacts': {
                'segments_json': str(paths['segments_json']),
                'segments_markdown': str(paths['segments_markdown']),
                'plan_json': str(options.output_dir.expanduser().resolve() / PUBLIC_LOOP_SEGMENT_RESET_JSON),
                'plan_markdown': str(
                    options.output_dir.expanduser().resolve() / PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN
                ),
            },
            'commands': commands,
            'next_actions': _next_actions(reset_pair),
        }
        write_loop_segment_artifacts(
            output_dir=options.output_dir,
            segment_report=segment_report,
            plan_report=report,
        )
        return report


def render_loop_segment_reset_markdown(report: dict[str, Any]) -> str:
    """Render the segment-reset plan as Markdown."""
    reset_pair = report.get('reset_pair') or {}
    commands = report.get('commands') or {}
    lines = [
        '# MID-360 Public Loop Segment Reset Plan',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- sequence_id: `{report.get('sequence_id', '')}`",
        f"- candidate_index: `{report.get('candidate_index', '')}`",
        f"- scan_count: `{report.get('scan_count', 0)}`",
        f"- segment_count: `{report.get('segment_count', 0)}`",
        f"- gap_count: `{report.get('gap_count', 0)}`",
        '',
        '## Reset Pair',
        '',
    ]
    for key in ('start', 'end'):
        endpoint = reset_pair.get(key) or {}
        segment = endpoint.get('segment') or {}
        lines.extend([
            f"### {key}",
            '',
            f"- candidate_stamp: `{endpoint.get('candidate_stamp', '')}`",
            f"- matched_scan_index: `{endpoint.get('matched_scan_index', '')}`",
            f"- stamp_error_sec: `{endpoint.get('stamp_error_sec', 0.0):.6f}`",
            f"- segment_id: `{segment.get('segment_id', '')}`",
            f"- segment_scan_range: `{segment.get('start_scan_index', '')}..{segment.get('end_scan_index', '')}`",
            f"- segment_duration_sec: `{segment.get('duration_sec', 0.0):.3f}`",
            f"- ready_for_clip: `{segment.get('ready_for_clip', False)}`",
            '',
        ])
    if commands:
        lines.extend(['## Commands', ''])
        for key in ('clip_start', 'clip_end', 'run_start_rko', 'run_end_rko'):
            value = commands.get(key)
            if value:
                lines.extend([f'### {key}', '', f'```bash\n{value}\n```', ''])
    steps = report.get('next_actions') or []
    if steps:
        lines.extend(['## Next Actions', ''])
        lines.extend(f'- {step}' for step in steps)
    return '\n'.join(lines).rstrip()


def write_loop_segment_artifacts(
    *,
    output_dir: Path,
    segment_report: dict[str, Any],
    plan_report: dict[str, Any] | None,
) -> dict[str, Path]:
    """Write segment and optional plan artifacts."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    segments_json = output_dir / PUBLIC_BAG_SEGMENTS_JSON
    segments_markdown = output_dir / PUBLIC_BAG_SEGMENTS_MARKDOWN
    segments_json.write_text(payload_to_json(segment_report) + '\n', encoding='utf-8')
    segments_markdown.write_text(
        render_public_bag_segments_markdown(segment_report) + '\n',
        encoding='utf-8',
    )
    paths = {
        'segments_json': segments_json,
        'segments_markdown': segments_markdown,
    }
    if plan_report is not None:
        plan_json = output_dir / PUBLIC_LOOP_SEGMENT_RESET_JSON
        plan_markdown = output_dir / PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN
        plan_json.write_text(payload_to_json(plan_report) + '\n', encoding='utf-8')
        plan_markdown.write_text(
            render_loop_segment_reset_markdown(plan_report) + '\n',
            encoding='utf-8',
        )
        paths.update({'plan_json': plan_json, 'plan_markdown': plan_markdown})
    return paths


def _segment_report(
    *,
    options: LoopSegmentResetOptions,
    dataset_id: str,
    timings: list[dict[str, Any]],
    split: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    segments = split['segments']
    row = {
        'dataset_id': dataset_id,
        'title': f'{options.sequence_id} merged loop bag',
        'status': 'PASS' if any(s.get('ready_for_clip') for s in segments) else 'FAIL',
        'error': '',
        'selected_bag_path': str(options.bag_path.expanduser().resolve()),
        'selected_topics': {
            'pointcloud': options.pointcloud_topic,
            'imu': options.imu_topic,
        },
        'scan_count': len(timings),
        'segments': segments,
        'gaps': split['gaps'],
        'recommended_segment': recommend_segment(segments),
        'suggested_next_steps': [],
    }
    return {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'status': row['status'],
        'manifest_path': '',
        'output_dir': str(options.output_dir.expanduser().resolve()),
        'selection': {'dataset_ids': [dataset_id]},
        'options': {
            'max_scan_gap_sec': options.max_scan_gap_sec,
            'min_segment_duration_sec': options.min_segment_duration_sec,
            'max_scans': options.max_scans,
            'min_keypoints': options.min_keypoints,
            'voxel_size': options.voxel_size,
            'min_range': options.min_range,
            'max_range': options.max_range,
        },
        'datasets': [row],
        'skipped': [],
        'counts': {
            'total': 1,
            'pass': 1 if row['status'] == 'PASS' else 0,
            'warn': 0,
            'fail': 1 if row['status'] == 'FAIL' else 0,
            'skipped': 0,
        },
    }


def _match_loop_endpoint(
    timings: list[dict[str, Any]],
    candidate: dict[str, Any],
    endpoint: str,
) -> dict[str, Any]:
    if not timings:
        raise ValueError('no scan timings available')
    candidate_stamp = float(candidate[f'{endpoint}_stamp'])
    candidate_index = int(candidate.get(f'{endpoint}_index') or 0)
    nearest = min(
        timings,
        key=lambda scan: abs(_scan_stamp(scan) - candidate_stamp),
    )
    direct = timings[candidate_index] if 0 <= candidate_index < len(timings) else None
    matched = direct if direct and abs(_scan_stamp(direct) - candidate_stamp) <= 0.5 else nearest
    stamp = _scan_stamp(matched)
    return {
        'endpoint': endpoint,
        'candidate_stamp': candidate_stamp,
        'candidate_index': candidate_index,
        'matched_scan_index': int(matched['scan_index']),
        'matched_scan_stamp': stamp,
        'stamp_error_sec': stamp - candidate_stamp,
        'matched_receive_time_ns': int(matched['receive_time_ns']),
        'matched_keypoint_count': int(matched.get('keypoint_count') or 0),
        'matched_clipped_point_count': int(matched.get('clipped_point_count') or 0),
    }


def _scan_stamp(scan: dict[str, Any]) -> float:
    header_ns = int(scan.get('header_stamp_ns') or 0)
    if header_ns:
        return header_ns / 1_000_000_000.0
    return float(scan.get('processed_min_sec') or 0.0)


def _segment_for_scan(
    segments: list[dict[str, Any]],
    scan_index: int,
) -> dict[str, Any]:
    for segment in segments:
        if int(segment['start_scan_index']) <= scan_index <= int(segment['end_scan_index']):
            return segment
    return {}


def _endpoint_payload(match: dict[str, Any], segment: dict[str, Any]) -> dict[str, Any]:
    return {
        **match,
        'status': 'PASS' if segment and segment.get('ready_for_clip') else 'FAIL',
        'segment': segment,
    }


def _status(reset_pair: dict[str, Any]) -> str:
    start = reset_pair.get('start') or {}
    end = reset_pair.get('end') or {}
    if start.get('status') == 'PASS' and end.get('status') == 'PASS':
        return 'PASS'
    if start.get('segment') or end.get('segment'):
        return 'WARN'
    return 'FAIL'


def _commands(
    options: LoopSegmentResetOptions,
    dataset_id: str,
    segments_json: Path,
    reset_pair: dict[str, Any],
) -> dict[str, str]:
    commands: dict[str, str] = {}
    for endpoint in ('start', 'end'):
        segment = ((reset_pair.get(endpoint) or {}).get('segment') or {})
        segment_id = str(segment.get('segment_id') or '')
        if not segment_id:
            continue
        clip_bag = (
            options.clip_output_root.expanduser().resolve()
            / dataset_id
            / segment_id
            / 'rosbag2'
        )
        clip_cmd = [
            'python3',
            'scripts/clip_mid360_robot_public_bag_segment.py',
            '--segments',
            str(segments_json),
            '--dataset',
            dataset_id,
            '--segment',
            segment_id,
            '--output-root',
            str(options.clip_output_root.expanduser().resolve()),
            '--force',
            '--json',
        ]
        rko_cmd = [
            'bash',
            'scripts/run_rko_lio_graph_autoware_dogfood.sh',
            '--bag',
            str(clip_bag),
            '--lidar-topic',
            options.pointcloud_topic,
            '--imu-topic',
            options.imu_topic,
            '--lidarslam-param',
            str(options.lidarslam_param),
            '--rko-param',
            str(options.rko_param),
            '--base-frame',
            options.base_frame,
            '--lidar-frame',
            options.lidar_frame,
            '--imu-frame',
            options.imu_frame,
            '--output-dir',
            str(options.rko_output_root.expanduser().resolve() / segment_id),
            '--run-name',
            f'{dataset_id}_{segment_id}',
            '--wait-for-offline-completion',
            '--skip-viewer',
        ]
        commands[f'clip_{endpoint}'] = shlex.join(clip_cmd)
        commands[f'run_{endpoint}_rko'] = shlex.join(rko_cmd)
    return commands


def _next_actions(reset_pair: dict[str, Any]) -> list[str]:
    if _status(reset_pair) != 'PASS':
        return ['Inspect the failed endpoint and adjust segment thresholds or loop candidate selection.']
    start_segment = ((reset_pair.get('start') or {}).get('segment') or {}).get('segment_id', '')
    end_segment = ((reset_pair.get('end') or {}).get('segment') or {}).get('segment_id', '')
    if start_segment == end_segment:
        return ['Run the single segment clip through RKO-LIO; a reset is not required for this loop pair.']
    return [
        'Clip both endpoint segments and run RKO-LIO separately for each segment.',
        'Use the cloud-overlap analyzer as the reference gate while adding relocalization/segment stitching.',
    ]


def _select_sequence(payload: dict[str, Any], sequence_id: str) -> dict[str, Any]:
    for sequence in payload.get('sequences') or []:
        if sequence.get('sequence_id') == sequence_id:
            return sequence
    raise ValueError(f'sequence not found: {sequence_id}')


def _select_candidate(sequence: dict[str, Any], candidate_index: int) -> dict[str, Any]:
    candidates = sequence.get('loop_candidates') or []
    if not candidates:
        raise ValueError(f'sequence has no loop candidates: {sequence.get("sequence_id", "")}')
    if candidate_index < 0 or candidate_index >= len(candidates):
        raise ValueError(f'candidate index out of range: {candidate_index}')
    return candidates[candidate_index]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding='utf-8'))


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_loop_segment_reset is a library module; import it instead of running it directly.")
